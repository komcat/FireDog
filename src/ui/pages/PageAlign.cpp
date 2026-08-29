// PageAlign.cpp : the Alignment page (View menu, full-page) — author and run
// alignment procedures, and inspect the local frame (O1) they teach.
// Definitions of App members — see App.h.
//
// The page does NOT capture points itself: Run launches
// alignment_procedure.lua through the Lua engine, exactly as a recipe's
// align_run step would, and the crosshair pick surfaces in the global pick
// window. Teaching and a scripted re-run therefore take the identical code
// path — there is only one way a frame can come into existence.
//
// Editing follows the recipes model (explicit Save + dirty baseline), not the
// positions autosave model: a procedure is a document. A *taught frame* is the
// exception — align_solve persists it the moment it is solved, because a frame
// lost to a crash means re-teaching the part.
#include "App.h"

#include "imgui.h"

#include "command/ExecutionContext.h"   // SystemScriptPath — the standard procedures
#include "core/Log.h"
#include "vision/HalconDetector.h"   // Available() — auto stations need a licence

#include <cfloat>    // FLT_MIN — the step buttons fill their column
#include <cmath>     // fabs — the guard band's deviations
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

// A station position taught from this page is PRIVATE to its procedure: it is
// named after the owner, and marked `hidden` so it stays out of the Motion
// page's table and the recipe editor's position picker. The name is the
// ownership record — no schema change, and an orphan can always be traced back
// to the procedure that made it.
//
// "algn_" + 8-hex uid + "_1" = 15 chars, inside SavedPos::name's 32.
std::string StationPosName(const std::string& procUid, int station)
{
    return "algn_" + procUid + "_" + std::to_string(station);
}

bool IsOwnedStationPos(const SavedPos& sp, const std::string& procUid)
{
    return sp.name == StationPosName(procUid, 1) || sp.name == StationPosName(procUid, 2);
}

// The same convention one level down: where the CAMERA stands to see the object
// being placed on one nominal. Private to the nominal, for the reason the field
// itself gives — it is a viewing pose for a gripped part, not the target.
//
// "algn_" + 8 + "_" + 8 + "_v" = 24 chars, inside SavedPos::name's 32.
std::string NominalViewPosName(const std::string& procUid, const std::string& nomUid)
{
    return "algn_" + procUid + "_" + nomUid + "_v";
}

// Everything a procedure owns, by name — stations AND every nominal's view.
// Prefix rather than an enumeration, because the sweep runs at DELETE time and
// a nominal that was removed earlier must not leave a hidden row behind.
bool IsOwnedAlignPos(const SavedPos& sp, const std::string& procUid)
{
    const std::string prefix = "algn_" + procUid + "_";
    return std::string(sp.name).rfind(prefix, 0) == 0;
}

// Look a position up by uid on one device (hidden rows included).
const SavedPos* FindPos(const SavedPosMap& positions, const std::string& device,
                        const std::string& uid)
{
    const auto it = positions.find(device);
    if (it == positions.end() || uid.empty()) return nullptr;
    for (const SavedPos& sp : it->second)
        if (sp.uid == uid) return &sp;
    return nullptr;
}

// The saved-position combo for a station, over the procedure's own device.
// Stores the uid (rename-safe), shows the name. Hidden rows are deliberately
// NOT offered: those are private station positions (this procedure's or another
// one's), and the way to get one is the Teach button, not the picker.
bool StationPositionCombo(const char* label, const SavedPosMap& positions,
                          const std::string& device, std::string& posUid)
{
    bool changed = false;
    const auto it = positions.find(device);
    std::string preview = posUid.empty() ? "(pick)" : posUid;
    if (it != positions.end()) {
        for (const SavedPos& sp : it->second)
            if (sp.uid == posUid) { preview = sp.name; break; }
    }
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo(label, preview.c_str())) {
        if (it == positions.end() || it->second.empty()) {
            ImGui::TextDisabled("(no saved positions for '%s')", device.c_str());
        } else {
            for (const SavedPos& sp : it->second) {
                if (sp.hidden) continue;
                if (ImGui::Selectable(sp.name, sp.uid == posUid)) {
                    posUid  = sp.uid;
                    changed = true;
                }
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

// ---- the step selector's status dots ---------------------------------------
// Three colours for three states, on the two rows where the question has an
// answer: a station located by a taught detection model, one marked by hand,
// and one that is not finished. Why "not finished" is stricter than "has a
// position" is argued at DrawAlignStepSelector.
constexpr float kDotW      = 18.0f;   // the dot's column, so labels line up
constexpr float kDotR      = 6.0f;
const ImU32     kDotAuto   = IM_COL32( 90, 215, 115, 255);   // green
const ImU32     kDotManual = IM_COL32( 90, 215, 240, 255);   // cyan
const ImU32     kDotOpen   = IM_COL32(255, 165,  60, 255);   // orange
// Red is reserved for ONE thing: a measurement that was checked and did not
// pass. Orange is "not finished", which is a different fact and must not wear
// the colour of a failure.
const ImU32     kDotBad    = IM_COL32(240,  85,  75, 255);

// The panes the step selector switches between. Named because the switch, the
// selector and every reset would otherwise be three lists of bare integers that
// have to agree.
enum : int {
    kAlignViewE1 = 0,
    kAlignViewE2,
    kAlignViewRun,
    kAlignViewNominals,
    kAlignViewCheck,
};

// A dot vertically centred against a button `h` tall. It advances the cursor by
// the full dot column, so a row WITHOUT one lines its label up by spending the
// same width on a Dummy.
void StatusDot(ImU32 col, float h)
{
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddCircleFilled(
        ImVec2(p.x + kDotW * 0.5f, p.y + h * 0.5f), kDotR, col, 20);
    ImGui::Dummy(ImVec2(kDotW, h));
}

} // namespace

// Launch the procedure script. Same channel the Motion page's XCAN button uses:
// parameters go in through LuaEngine::Env::initialCtx, where fd.param reads
// them. Gated on the WHOLE orchestration, never a subset of the engines.
void App::StartAlignmentRun(const AlignProc& a)
{
    // The SHIPPED file, always: this is a standard procedure, edited in step
    // with the commands it calls, so a product's copy of the name is ignored
    // here exactly as ExecutionContext::ResolveScript ignores it for align_run.
    // One name, one file, on every machine.
    const std::string script = SystemScriptPath("alignment_procedure.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"align_name", a.uid},        // uid, not name: a rename mid-run is harmless
        {"align_prefix", std::string("frame")},
    };
    alignStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &alignStartErr_))
        FD_WARN("align", "alignment run rejected: %s", alignStartErr_.c_str());
}

// The repeatability sweep. Same channel and the same gating as the single run
// above — one engine Start for the whole sweep, so nothing else can start a run
// between iterations and Stop ends the lot. The script itself refuses an `n`
// outside the bounds, so a hand-written ctx cannot ask for a thousand.
void App::StartAlignmentRepeat(const AlignProc& a, int n)
{
    if (n < kAlignRepeatMin) n = kAlignRepeatMin;
    if (n > kAlignRepeatMax) n = kAlignRepeatMax;
    const std::string script = SystemScriptPath("alignment_repeat.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"align_name", a.uid},
        {"align_prefix", std::string("frame")},
        {"align_repeat", static_cast<double>(n)},
    };
    alignStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &alignStartErr_))
        FD_WARN("align", "repeatability sweep rejected: %s", alignStartErr_.c_str());
    else
        FD_INFO("align", "'%s': repeatability sweep of %d runs started", a.name.c_str(), n);
}

// Capture the device's CURRENT pose as this station's private position.
// Update-in-place when the procedure already owns one, so the uid — and
// therefore anything referencing it — survives a re-teach.
void App::TeachAlignStation(AlignProc& a, int station)
{
    const std::string name = StationPosName(a.uid, station);
    const std::string uid  = CapturePrivatePose(a.device, name);
    if (uid.empty()) return;
    (station == 1 ? a.s1 : a.s2).posUid = uid;
    session_.SavePositions();
    FD_INFO("align", "'%s' station %d taught as '%s'", a.name.c_str(), station, name.c_str());
}

// The one place a hidden, owner-named position is written. Two callers now
// (a station and a nominal's camera view) and they must not drift: an
// update-in-place that assigned a fresh uid would silently orphan whatever
// already pointed at the old one. "" when the device cannot be read.
std::string App::CapturePrivatePose(const std::string& device, const std::string& name)
{
    IMotionController* dev = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == device) { dev = hardware_.MotionDevice(i); break; }
    }
    if (!dev || !dev->IsConnected()) return std::string();

    std::vector<SavedPos>& list = session_.Positions()[device];
    SavedPos* tgt = nullptr;
    for (SavedPos& sp : list)
        if (name == sp.name) { tgt = &sp; break; }
    if (!tgt) {
        SavedPos np;
        std::snprintf(np.name, sizeof(np.name), "%s", name.c_str());
        np.uid    = GenerateUid();
        np.hidden = true;   // private: off the Motion table and the recipe picker
        list.push_back(std::move(np));
        tgt = &list.back();
    }

    const std::map<std::string, double> live = dev->Positions();
    tgt->vals.clear();
    for (const std::string& ax : dev->Axes()) {
        const auto it = live.find(ax);
        tgt->vals[ax] = (it != live.end()) ? it->second : 0.0;
    }
    return tgt->uid;
}

// ---- a nominal's camera view ----------------------------------------------
// Where the camera stands to SEE the object being placed on this nominal —
// a different XY from the nominal, and usually a different Z, because a gripped
// part is not at the substrate's focus. Owned by the nominal so the placement
// can drive there by itself instead of needing a move_to_position in front of
// every run.

const SavedPos* App::AlignNominalView(const AlignProc& a, const AlignNominal& n) const
{
    return FindPos(session_.Positions(), a.device, n.viewUid);
}

void App::TeachAlignNominalView(AlignProc& a, AlignNominal& n)
{
    const std::string name = NominalViewPosName(a.uid, n.uid);
    const std::string uid  = CapturePrivatePose(a.device, name);
    if (uid.empty()) return;
    n.viewUid = uid;
    session_.SavePositions();
    session_.SaveAligns();   // the reference is the taught thing; persist it too
    FD_INFO("align", "'%s' nominal '%s': camera view taught as '%s'",
            a.name.c_str(), n.name.c_str(), name.c_str());
}

// A plain UI move to a pose a human taught, like a station's Go — not the
// armed class, which is for COMPUTED destinations.
void App::GoToAlignNominalView(const AlignProc& a, const AlignNominal& n)
{
    const SavedPos* pos = AlignNominalView(a, n);
    if (!pos) return;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) != a.device) continue;
        IMotionController* dev = hardware_.MotionDevice(i);
        if (!dev || !dev->IsConnected()) return;
        std::vector<std::pair<std::string, double>> targets;
        for (const std::string& ax : dev->Axes()) {
            const auto it = pos->vals.find(ax);
            if (it != pos->vals.end()) targets.emplace_back(ax, it->second);
        }
        if (!targets.empty()) workflow_.MoveToPosition(i, targets);
        return;
    }
}

// Deleting a nominal takes its view position with it, exactly as it takes its
// detection models — a hidden orphan is something nobody goes looking for.
void App::DropAlignNominalView(const AlignProc& a, AlignNominal& n)
{
    n.viewUid.clear();          // whether or not the row is there to erase
    session_.SaveAligns();      // the reference is document state
    const auto it = session_.Positions().find(a.device);
    if (it == session_.Positions().end()) return;
    std::vector<SavedPos>& list = it->second;
    const std::string name = NominalViewPosName(a.uid, n.uid);
    for (std::size_t i = list.size(); i-- > 0;) {
        if (list[i].name != name) continue;
        list.erase(list.begin() + static_cast<long>(i));
        session_.SavePositions();
        FD_INFO("align", "'%s' nominal '%s': dropped its camera view position",
                a.name.c_str(), n.name.c_str());
        return;
    }
}

// Erase the private positions a procedure owns — its two stations and every
// nominal's camera view. Without this they would linger invisibly: hidden rows
// are off the Motion table by default, so an orphan is something nobody would
// ever go looking for.
void App::DropAlignStationPositions(const AlignProc& a)
{
    const auto it = session_.Positions().find(a.device);
    if (it == session_.Positions().end()) return;
    std::vector<SavedPos>& list = it->second;
    const std::size_t before = list.size();
    for (std::size_t i = list.size(); i-- > 0;) {
        if (IsOwnedAlignPos(list[i], a.uid)) list.erase(list.begin() + static_cast<long>(i));
    }
    if (list.size() != before) {
        session_.SavePositions();
        FD_INFO("align", "'%s': dropped %zu private position(s)",
                a.name.c_str(), before - list.size());
    }
}

void App::DrawAlignPanel()
{
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  leftW  = 230.0f;
    const float  rightW = 330.0f;
    const float  midW   = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;

    ImGui::BeginChild("alList", ImVec2(leftW, avail.y), ImGuiChildFlags_Borders);
    DrawAlignListPanel();
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("alDetail", ImVec2(midW, avail.y), ImGuiChildFlags_Borders);
    DrawAlignDetailPanel();
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("alFrame", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);
    DrawAlignFramePanel();
    ImGui::EndChild();
}

// ---- Left: the procedure list ---------------------------------------------

void App::DrawAlignListPanel()
{
    std::vector<AlignProc>& list = session_.Aligns();
    const bool busy = orch_.Busy();

    ImGui::TextUnformatted("Procedures");
    ImGui::SameLine();
    if (session_.AlignsDirty()) ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "*");
    ImGui::Separator();

    ImGui::BeginDisabled(busy);
    if (ImGui::SmallButton("New")) {
        AlignProc a;
        a.uid  = GenerateUid();
        a.name = "alignment " + std::to_string(list.size() + 1);
        list.push_back(a);
        alignSel_ = static_cast<int>(list.size()) - 1;
        alignNameBuf_[0] = '\0';
        alignView_ = 0;
    }
    const bool haveSel = alignSel_ >= 0 && alignSel_ < static_cast<int>(list.size());
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveSel);
    if (ImGui::SmallButton("Duplicate")) {
        const AlignProc& src = list[static_cast<std::size_t>(alignSel_)];
        AlignProc a = src;
        a.uid    = GenerateUid();      // a copy is a NEW identity...
        a.name  += " copy";
        a.result = AlignResult{};      // ...and it has not been taught yet
        // The NOMINALS come along (the plain copy above): they are local
        // coordinates, so they mean the same thing in whatever frame the
        // duplicate eventually teaches. Only the frame itself is per-part.
        // Private station positions belong to the ORIGINAL. Copy them under the
        // new owner so the duplicate is usable, rather than sharing rows the
        // original's Delete would take away.
        std::vector<SavedPos>& pl = session_.Positions()[a.device];
        for (int s = 1; s <= 2; ++s) {
            const SavedPos* sp = FindPos(session_.Positions(), src.device,
                                         (s == 1 ? src.s1 : src.s2).posUid);
            if (!sp || !IsOwnedStationPos(*sp, src.uid)) continue;
            SavedPos np = *sp;
            std::snprintf(np.name, sizeof(np.name), "%s", StationPosName(a.uid, s).c_str());
            np.uid = GenerateUid();
            (s == 1 ? a.s1 : a.s2).posUid = np.uid;
            pl.push_back(std::move(np));
        }
        // ...and so does every nominal's CAMERA VIEW, for the same reason: the
        // nominals came along with the plain copy, and a view they share with
        // the original would vanish the day the original is deleted.
        for (std::size_t i = 0; i < a.nominals.size() && i < src.nominals.size(); ++i) {
            const SavedPos* vp = FindPos(session_.Positions(), src.device,
                                         src.nominals[i].viewUid);
            if (!vp) continue;
            SavedPos np = *vp;   // copy BEFORE the push_back that may reallocate
            std::snprintf(np.name, sizeof(np.name), "%s",
                          NominalViewPosName(a.uid, a.nominals[i].uid).c_str());
            np.uid = GenerateUid();
            a.nominals[i].viewUid = np.uid;
            pl.push_back(std::move(np));
        }
        // Private DETECTION MODELS copy the same way and for the same reason —
        // patch file included, so the duplicate is self-contained.
        CopyAlignStationDetectors(src, a);
        list.push_back(a);
        alignSel_ = static_cast<int>(list.size()) - 1;
        alignNameBuf_[0] = '\0';
        alignView_ = 0;
        session_.SavePositions();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Delete")) {
        DropAlignStationPositions(list[static_cast<std::size_t>(alignSel_)]);
        DropAlignStationDetectors(list[static_cast<std::size_t>(alignSel_)]);
        list.erase(list.begin() + alignSel_);
        alignSel_ = list.empty() ? -1 : 0;
        alignNameBuf_[0] = '\0';
        alignView_ = 0;
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    ImGui::BeginDisabled(busy || !session_.AlignsDirty());
    if (ImGui::SmallButton("Save")) session_.SaveAligns();
    ImGui::EndDisabled();
    ImGui::Separator();

    if (list.empty())
        ImGui::TextWrapped("No alignment procedures yet. New creates one; assign a "
                           "station position to each of E1 and E2, then Run.");
    for (int i = 0; i < static_cast<int>(list.size()); ++i) {
        ImGui::PushID(i);
        const AlignProc& a = list[static_cast<std::size_t>(i)];
        if (ImGui::Selectable(a.name.c_str(), alignSel_ == i)) {
            alignSel_ = i;
            alignNameBuf_[0] = '\0';
            alignView_ = 0;   // always land on E1, whatever the last one showed
        }
        ImGui::SameLine();
        if (a.result.valid)
            ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.5f, 1.0f), "*");
        else
            ImGui::TextDisabled("-");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(a.result.valid ? "frame taught" : "no frame taught yet");
        ImGui::PopID();
    }
}

// ---- Middle: a header, a step selector, and ONE step at a time -------------

void App::DrawAlignStationCard(AlignProc& a, int station)
{
    AlignStation& st = (station == 1) ? a.s1 : a.s2;
    const bool busy = orch_.Busy();

    ImGui::PushID(station);
    ImGui::SeparatorText(station == 1 ? "Position 1  ->  E1 (local origin)"
                                      : "Position 2  ->  E2 (local X+)");

    int devIdx = -1;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a.device) { devIdx = static_cast<int>(i); break; }
    }
    IMotionController* dev =
        devIdx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(devIdx)) : nullptr;

    // The card is two columns, split down the middle: WHERE the station is on
    // the left (its private position, the pose behind it, exposure, lighting)
    // and HOW the point gets marked on the right (manual crosshair, or a taught
    // detector with its status block). They are independent decisions, and the
    // detector block is by far the tallest thing here - stacked, it pushed the
    // second station and the Run button off the bottom of the panel.
    //
    // Stretch weights rather than fixed widths, so the two station cards line
    // up with each other at any panel width and the thumbnail keeps its room.
    if (ImGui::BeginTable("station", 2, ImGuiTableFlags_BordersInnerV |
                                        ImGuiTableFlags_Resizable |
                                        ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("##where", ImGuiTableColumnFlags_WidthStretch, 0.55f);
        ImGui::TableSetupColumn("##how",   ImGuiTableColumnFlags_WidthStretch, 0.45f);
        ImGui::TableNextRow();

        ImGui::TableSetColumnIndex(0);
        ImGui::BeginDisabled(busy);
        StationPositionCombo("gantry position", session_.Positions(), a.device, st.posUid);
        ImGui::EndDisabled();
        ImGui::SameLine();

        // Go: a plain UI move to the station, so the operator can confirm the
        // teach point without launching the whole procedure. Layer 2 owns it.
        const SavedPos* pos = FindPos(session_.Positions(), a.device, st.posUid);
        ImGui::BeginDisabled(busy || !pos || !dev || !dev->IsConnected());
        // NOTE: `pos` points into the positions vector. Teaching may push_back into
        // that same vector, so it is re-resolved after the Teach button below.
        if (ImGui::Button("Go")) {
            std::vector<std::pair<std::string, double>> targets;
            for (const std::string& ax : dev->Axes()) {
                const auto it = pos->vals.find(ax);
                if (it != pos->vals.end()) targets.emplace_back(ax, it->second);
            }
            if (!targets.empty())
                workflow_.MoveToPosition(static_cast<std::size_t>(devIdx), targets);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();

        // Teach: jog the gantry where you want the station, then capture it as this
        // procedure's PRIVATE position. Re-teaching updates in place so the uid
        // stays stable — anything already pointing at it keeps working.
        ImGui::BeginDisabled(busy || !dev || !dev->IsConnected());
        if (ImGui::Button(pos && IsOwnedStationPos(*pos, a.uid) ? "Re-teach here" : "Teach here")) {
            TeachAlignStation(a, station);
            pos = FindPos(session_.Positions(), a.device, st.posUid);   // may have reallocated
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Save the CURRENT %s pose as this station's own position\n"
                              "(private to this procedure: hidden from the Motion page\n"
                              "table and the recipe position picker).", a.device.c_str());

        // Show the numbers here, since a private position is not on the Motion page.
        if (pos) {
            std::string line;
            for (const auto& [axis, v] : pos->vals) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%s %.4f", line.empty() ? "" : "  ",
                              axis.c_str(), v);
                line += buf;
            }
            if (IsOwnedStationPos(*pos, a.uid))
                ImGui::TextDisabled("private  |  %s", line.c_str());
            else
                ImGui::TextDisabled("shared '%s'  |  %s", pos->name, line.c_str());
        }

        ImGui::BeginDisabled(busy);
        float expo = static_cast<float>(st.exposureUs);
        ImGui::SetNextItemWidth(140.0f);
        if (ImGui::InputFloat("exposure (us)", &expo, 0.0f, 0.0f, "%.0f"))
            st.exposureUs = expo < 0.0f ? 0.0 : static_cast<double>(expo);
        ImGui::SameLine();
        ImGui::TextDisabled("(0 = leave)");

        // Lighting is a PLACEHOLDER: no illumination device exists in the hardware
        // config yet, so "none" is the only setting that does anything today. The
        // fields are here so a procedure taught now keeps its intent.
        const char* kinds[] = {"none", "io", "psu"};
        int kindIdx = st.lighting.kind == "io" ? 1 : (st.lighting.kind == "psu" ? 2 : 0);
        ImGui::SetNextItemWidth(90.0f);
        if (ImGui::Combo("lighting", &kindIdx, kinds, 3))
            st.lighting.kind = (kindIdx == 0) ? "" : kinds[kindIdx];
        if (kindIdx != 0) {
            char devBuf[64], chBuf[48];
            std::snprintf(devBuf, sizeof(devBuf), "%s", st.lighting.device.c_str());
            std::snprintf(chBuf, sizeof(chBuf), "%s", st.lighting.channel.c_str());
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            if (ImGui::InputText("device", devBuf, sizeof(devBuf))) st.lighting.device = devBuf;
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            if (ImGui::InputText(kindIdx == 1 ? "pin" : "channel", chBuf, sizeof(chBuf)))
                st.lighting.channel = chBuf;
            ImGui::SameLine();
            float lv = static_cast<float>(st.lighting.value);
            ImGui::SetNextItemWidth(80.0f);
            if (ImGui::InputFloat(kindIdx == 1 ? "on" : "volts", &lv, 0.0f, 0.0f, "%.2f"))
                st.lighting.value = static_cast<double>(lv);
        }

        // One BeginDisabled per column rather than one spanning both: the pair
        // has to open and close inside the same cell.
        ImGui::EndDisabled();

        ImGui::TableSetColumnIndex(1);
        ImGui::BeginDisabled(busy);
        // How the E point is marked. Manual = a crosshair on the live image (plain
        // ImGui over a Pylon frame, so it never depends on a vision licence). Auto =
        // a taught detection model, which DOES need HALCON — hence the licence line
        // right here rather than a surprise at run time.
        int mode = (st.mode == "auto") ? 1 : 0;
        if (ImGui::RadioButton("Manual (crosshair)", mode == 0)) st.mode = "manual";
        ImGui::SameLine();
        if (ImGui::RadioButton("Auto (detect)", mode == 1)) st.mode = "auto";
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Locate the feature with a taught detection model\n"
                              "(circle/fiducial or gray pattern match). Needs HALCON.");
        if (st.mode == "auto") {
            ImGui::Indent();

            // The detection model is PRIVATE to this station — auto-named
            // `vdet_<procUid>_<n>`, created by the teach window, deleted with the
            // procedure. There is no picker because there is nothing to pick: the
            // ROI, threshold and taught patch only mean anything at this station's
            // pose and exposure. `detector` caches its uid so align_station and any
            // recipe step resolve it the same way they resolve everything else.
            const VisionModel* model = StationDetector(a, station);

            if (ImGui::Button(model ? "Teach detection..." : "Set up detection..."))
                OpenVisionTeach(a, station);

            std::string why;
            if (!HalconDetector::Available(&why)) {
                ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", why.c_str());
            } else if (!model) {
                ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                                   "no detection set up - this station will fail at run time");
            } else {
                ImGui::SameLine();
                ImGui::TextDisabled("%s%s", model->method.c_str(),
                                    (model->method == "ncc" && model->templateFile.empty())
                                        ? "  (no template taught)" : "");
                // A pre-processed model behaves nothing like a raw one, and the
                // chain is invisible from here otherwise — an operator reading a
                // card would have no idea the image was thresholded first.
                const std::string chain = VisionFilterSummary(*model);
                if (!chain.empty()) ImGui::TextDisabled("pre-process: %s", chain.c_str());
                if (model->rel.runs > 0) {
                    ImGui::TextDisabled("%d/%d ok, sigma %.2f px, tested %s",
                                        model->rel.ok, model->rel.runs,
                                        model->rel.stdXPx > model->rel.stdYPx ? model->rel.stdXPx
                                                                              : model->rel.stdYPx,
                                        model->rel.testedAt.c_str());
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                                       "never reliability-tested");
                }

                // The picture the detector matches, with a cross on the point it
                // REPORTS. It is the only thing on this page that says WHICH point
                // of the feature a run will use — and it is the same picture the
                // fallback shows when detection fails and an operator, who did not
                // tune this model, is asked to mark that point by hand.
                PatchTex& pt = alignTmpl_[(station == 2) ? 1 : 0];
                if (EnsurePatchTex(pt, *model)) {
                    ImGui::BeginGroup();
                    DrawPatchThumb(pt, 120.0f, 120.0f);
                    ImGui::EndGroup();
                    ImGui::SameLine();
                    ImGui::BeginGroup();
                    ImGui::TextDisabled("what this station looks for");
                    ImGui::TextDisabled("the cross is the point it reports");
                    ImGui::EndGroup();
                }
            }

            ImGui::Checkbox("fall back to a manual pick", &st.autoFallbackManual);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Off, an auto station FAILS when detection does - which is\n"
                                  "the point: a silent fallback would let a procedure claim\n"
                                  "to be automatic while a human placed every point.");
            ImGui::Unindent();
        }
        ImGui::EndDisabled();
        ImGui::EndTable();
    }
    ImGui::PopID();
}

void App::DrawAlignDetailPanel()
{
    std::vector<AlignProc>& list = session_.Aligns();
    if (alignSel_ < 0 || alignSel_ >= static_cast<int>(list.size())) {
        ImGui::TextDisabled("Select an alignment procedure.");
        return;
    }
    AlignProc& a = list[static_cast<std::size_t>(alignSel_)];
    const bool busy = orch_.Busy();

    // ---- Header: WHICH procedure, and on WHAT ------------------------------
    // The two facts that hold for every step below, so they stay put while the
    // pane under them swaps: the procedure's name, and the motion device its
    // stations are poses on.
    ImGui::SeparatorText("Set up origin frame");
    if (alignNameBuf_[0] == '\0')
        std::snprintf(alignNameBuf_, sizeof(alignNameBuf_), "%s", a.name.c_str());
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(240.0f);
    if (ImGui::InputText("name", alignNameBuf_, sizeof(alignNameBuf_)))
        a.name = alignNameBuf_;
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("uid %s", a.uid.c_str());

    // Device combo: which motion device the two stations belong to.
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(240.0f);
    if (ImGui::BeginCombo("device", a.device.c_str())) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string n = hardware_.MotionName(i);
            if (ImGui::Selectable(n.c_str(), n == a.device)) a.device = n;
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::Separator();

    // ---- The four steps: choose on the left, ONE of them draws on the right -
    // Stacked, the two station cards, Run and the nominals were taller than the
    // page: Run — the thing all of it exists to reach — sat below the fold, and
    // the two cards, which differ only in a heading, had to be told apart by
    // reading. One at a time, chosen from a column that also REPORTS where each
    // station stands, is the same information without the scroll.
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    float selW = 190.0f;
    if (avail.x < selW * 3.0f) selW = avail.x * 0.33f;   // never eat the pane
    if (selW < 90.0f) selW = 90.0f;   // a NEGATIVE width means "fill" to ImGui

    ImGui::BeginChild("alSteps", ImVec2(selW, avail.y), ImGuiChildFlags_Borders);
    DrawAlignStepSelector(a);
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("alStep", ImVec2(0.0f, avail.y), ImGuiChildFlags_Borders);
    switch (alignView_) {
        case kAlignViewE2:       DrawAlignStationCard(a, 2); break;
        case kAlignViewRun:      DrawAlignRunPanel(a);       break;
        case kAlignViewNominals: DrawAlignNominalList(a);    break;
        case kAlignViewCheck:    DrawAlignCheckPanel(a);     break;
        default:                 DrawAlignStationCard(a, 1); break;
    }
    ImGui::EndChild();
}

// The selector column: which step the pane beside it is showing, and — for the
// two stations — whether that step is finished. A dot rather than a word,
// because the column is narrow and the answer is meant to be glanceable:
// green a station located by a taught detection model, cyan one marked by hand,
// orange one that is not finished.
//
// "Not finished" is deliberately STRICTER for an auto station than "has a
// position": one with no detection model behind it has everything a run needs
// except the single thing it will actually use, and it fails at run time, on
// the machine, with a part loaded. A green dot there would be a promise the
// station cannot keep — so an auto station without a model (or an NCC one with
// no patch cut yet) reads exactly as orange as an untaught one, and the card
// beside it says which of the two it is.
//
// Run is NOT gated here. The button always swaps the pane in; it is the *Run
// alignment* button inside that stays disabled, with the reason written under
// it — a selector that refuses to open is a selector that cannot explain
// itself.
//
// Run and Frame check carry dots of their own, and they answer different
// questions from a station's. Run's is about THIS SESSION: green means a LIVE
// frame exists, which is the only state in which anything may move against it —
// the record left by last week's part is deliberately not green. Frame check's
// is about the last measurement: green passed the band, red failed it, orange
// means nothing checked it.
void App::DrawAlignStepSelector(AlignProc& a)
{
    // Tall enough for the two-line label on the last one, and for the dot
    // beside it to read as a lamp rather than a bullet point.
    const float btnH = ImGui::GetFrameHeight() * 1.8f;

    ImGui::TextDisabled("Steps");
    ImGui::Separator();

    for (int station = 1; station <= 2; ++station) {
        const AlignStation& st = (station == 1) ? a.s1 : a.s2;
        ImU32       dot = kDotOpen;
        const char* why = "not taught: no station position assigned";
        if (!st.posUid.empty()) {
            if (st.mode == "auto") {
                const VisionModel* m = StationDetector(a, station);
                const bool ready =
                    m && !(m->method == "ncc" && m->templateFile.empty());
                dot = ready ? kDotAuto : kDotOpen;
                why = ready ? "taught - located automatically (detection model)"
                            : "auto, but no detection model taught:\n"
                              "this station would fail at run time";
            } else {
                dot = kDotManual;
                why = "taught - the point is marked by hand on the live image";
            }
        }

        ImGui::PushID(station);
        StatusDot(dot, btnH);
        ImGui::SameLine();
        const bool sel = (alignView_ == station - 1);   // kAlignViewE1 / E2
        if (sel)
            ImGui::PushStyleColor(ImGuiCol_Button,
                                  ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(station == 1 ? "E1" : "E2", ImVec2(-FLT_MIN, btnH)))
            alignView_ = station - 1;
        if (sel) ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\n%s",
                              station == 1 ? "E1 - the local origin"
                                           : "E2 - the local X+ direction",
                              why);
        ImGui::PopID();
    }

    ImGui::Spacing();

    // Run: green only while a LIVE frame exists. `a.result.valid` is
    // deliberately NOT enough — that is the record of the last teach, and a
    // green light over a frame nothing may move against is the whole live/record
    // distinction undone in the UI.
    AlignResult live;
    const bool haveLive = workflow_.LiveFrame(a.uid, &live);

    // Frame check: what the band said about the last run. `runs.back()` and not
    // the band's own state, because the question is "did the last part pass",
    // and a band switched on after a run has judged nothing yet.
    const AlignRun* last = a.runs.empty() ? nullptr : &a.runs.back();
    ImU32       checkDot = kDotOpen;
    const char* checkWhy = "no band set - nothing checks the frame";
    if (a.guard.enabled && last && last->guarded) {
        checkDot = last->ok ? kDotAuto : kDotBad;
        checkWhy = last->ok ? "the last run was inside the band"
                            : "the last run was OUT OF BAND";
    } else if (a.guard.enabled) {
        checkWhy = "band set, but no run has been checked against it yet";
    }

    // Their labels line up with the stations' because a row without a dot
    // spends the same width on a Dummy.
    struct Step { const char* label; int view; ImU32 dot; const char* tip; };
    const Step rest[3] = {
        {"Run alignment", kAlignViewRun, haveLive ? kDotAuto : kDotOpen,
         haveLive ? "A LIVE frame is taught - move_local and Move here will use it."
                  : "No live frame in this session. Run the procedure: a saved\n"
                    "frame is a record of the last teach, not a licence to move."},
        {"Frame check", kAlignViewCheck, checkDot, checkWhy},
        {"Teach nominal\nposition", kAlignViewNominals, 0,
         "The target points on the PART, in local (O1) coordinates - added,\n"
         "checked and placed onto from here."},
    };
    for (const Step& s : rest) {
        ImGui::PushID(s.view);
        if (s.dot) StatusDot(s.dot, btnH);
        else       ImGui::Dummy(ImVec2(kDotW, btnH));
        ImGui::SameLine();
        const bool sel = (alignView_ == s.view);
        if (sel)
            ImGui::PushStyleColor(ImGuiCol_Button,
                                  ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(s.label, ImVec2(-FLT_MIN, btnH))) alignView_ = s.view;
        if (sel) ImGui::PopStyleColor();
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", s.tip);
        ImGui::PopID();
    }
}

// Step 3: MAKE the frame, then check it. The button that produces the frame and
// the local->machine probe that reads it back belong on one pane — a run whose
// result is never probed is how a bad teach survives as far as a placement.
// The right-hand PANEL keeps the frame's numbers: that is the record of the
// teach, this is what you do with it.
void App::DrawAlignRunPanel(AlignProc& a)
{
    const bool busy = orch_.Busy();

    ImGui::SeparatorText("Run");
    const bool ready = !a.s1.posUid.empty() && !a.s2.posUid.empty();
    const LuaEngine::Status lst = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    ImGui::BeginDisabled(busy || !ready);
    if (ImGui::Button("Run alignment", ImVec2(150.0f, 0.0f))) StartAlignmentRun(a);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(90.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();

    if (!ready)
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "assign a saved position to both stations first");
    else if (busy && !running)
        ImGui::TextDisabled("another run is active");
    if (!alignStartErr_.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", alignStartErr_.c_str());
    if (running || lst.state != LuaEngine::State::Idle) {
        ImGui::TextDisabled("%s  %s", LuaEngine::StateName(lst.state),
                            lst.error.empty() ? "" : lst.error.c_str());
    }

    DrawAlignFrameActions(a);
}

// The band, and the roll of what every run measured against it.
//
// The two halves of the band are different KINDS of number and the pane says so
// rather than leaving an operator to discover it by widening the wrong one:
// span is on the drawing (the same on every part, so a breach means the wrong
// feature was marked), theta is how this one happens to sit in the fixture (it
// differs every load, legitimately, so its reference is a nominal orientation
// and its band is wide).
void App::DrawAlignCheckPanel(AlignProc& a)
{
    const bool busy = orch_.Busy();

    ImGui::SeparatorText("Guard band");
    ImGui::BeginDisabled(busy);
    ImGui::Checkbox("check every solved frame against a band", &a.guard.enabled);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off, a frame is only checked for being geometrically\n"
                          "possible. On, the band is drawn on the live image while\n"
                          "E2 is marked, so the mark can be moved into it - and an\n"
                          "auto station that detects outside it falls back to a\n"
                          "manual pick instead of teaching a wrong frame.");

    ImGui::BeginDisabled(!a.guard.enabled);
    float f = 0.0f;
    ImGui::SetNextItemWidth(120.0f);
    f = static_cast<float>(a.guard.spanMm);
    if (ImGui::InputFloat("span reference (mm)", &f, 0.0f, 0.0f, "%.4f"))
        a.guard.spanMm = f < 0.0 ? 0.0 : static_cast<double>(f);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    f = static_cast<float>(a.guard.spanTolMm);
    if (ImGui::InputFloat("+/- (mm)##spantol", &f, 0.0f, 0.0f, "%.4f"))
        a.guard.spanTolMm = f < 0.0f ? 0.0 : static_cast<double>(f);
    ImGui::SameLine();
    // BOTH references come off a run rather than being typed. The span on the
    // drawing is between two DESIGN features and this one is between the two the
    // operator actually marks; and a fixture that holds every part at 0.55 deg
    // is a fact about the fixture, so making somebody read it off the frame
    // panel and retype it only invites a typo. What the run cannot supply is the
    // TOLERANCE — that is a judgement about how much a good part may vary, and
    // it stays typed.
    ImGui::BeginDisabled(a.runs.empty());
    if (ImGui::SmallButton("Set from last run")) {
        a.guard.spanMm  = a.runs.back().spanMm;
        a.guard.thetaDeg = a.runs.back().thetaDeg;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Copy the span AND the angle the last run measured. Do this on\n"
                          "a part you have checked, then widen each tolerance to what a\n"
                          "good part may vary by - not to what makes today's part pass.\n"
                          "The angle is where the fixture holds a part, so keep its band\n"
                          "wide: it is here to catch one in backwards, not a mis-pick.");
    ImGui::TextDisabled("   the distance E1->E2 on a good part: a breach means the wrong "
                        "feature was marked");

    ImGui::SetNextItemWidth(120.0f);
    f = static_cast<float>(a.guard.thetaDeg);
    if (ImGui::InputFloat("theta reference (deg)", &f, 0.0f, 0.0f, "%.3f"))
        a.guard.thetaDeg = static_cast<double>(f);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    f = static_cast<float>(a.guard.thetaTolDeg);
    if (ImGui::InputFloat("+/- (deg)##thetatol", &f, 0.0f, 0.0f, "%.3f"))
        a.guard.thetaTolDeg = f < 0.0f ? 0.0 : static_cast<double>(f);
    ImGui::TextDisabled("   where the fixture HOLDS a part (0 = square to the machine) - "
                        "keep the band wide:");
    ImGui::TextDisabled("   it is here to catch one loaded backwards, not a mis-picked "
                        "feature");
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    // What the band said about the last run, in the numbers it said it with.
    ImGui::SeparatorText("Last run");
    if (a.runs.empty()) {
        ImGui::TextDisabled("No run recorded yet.");
    } else {
        const AlignRun& r = a.runs.back();
        double spanErr = 0.0, thetaErr = 0.0;
        const bool nowOk = AlignGuardCheck(a.guard, r.spanMm, r.thetaDeg,
                                           &spanErr, &thetaErr);
        ImGui::Text("%s", r.at.empty() ? "(no timestamp)" : r.at.c_str());
        if (!a.guard.enabled) {
            ImGui::TextDisabled("span %.4f mm   theta %.4f deg   (no band)",
                                r.spanMm, r.thetaDeg);
        } else {
            const ImVec4 okCol(0.4f, 0.9f, 0.5f, 1.0f), badCol(1.0f, 0.55f, 0.3f, 1.0f);
            ImGui::TextColored(std::fabs(spanErr) <= a.guard.spanTolMm ? okCol : badCol,
                               "span  %.4f mm  (%+.4f)", r.spanMm, spanErr);
            ImGui::TextColored(std::fabs(thetaErr) <= a.guard.thetaTolDeg ? okCol : badCol,
                               "theta %.4f deg (%+.4f)", r.thetaDeg, thetaErr);
            // The verdict RECORDED with the run, beside the one the band gives
            // now: editing a tolerance must not silently rewrite history.
            if (r.guarded && r.ok != nowOk)
                ImGui::TextDisabled("recorded as %s under the band in force at the time",
                                    r.ok ? "in band" : "OUT OF BAND");
            else if (!r.guarded)
                ImGui::TextDisabled("this run was not checked - the band was off when it ran");
        }
    }

    // ---- The repeatability sweep, and what it proves ------------------------
    // The band says what a good part may vary by. This says whether the
    // procedure can actually hold it — the same question a Cpk answers on a
    // production line, asked of the thing that measures rather than the thing
    // measured.
    ImGui::SeparatorText("Repeatability");
    const bool ready   = !a.s1.posUid.empty() && !a.s2.posUid.empty();
    const bool running = orch_.lua.Running();

    ImGui::BeginDisabled(busy || !ready);
    char runBtn[48];
    std::snprintf(runBtn, sizeof(runBtn), "Run alignment x %d", alignRepeatN_);
    if (ImGui::Button(runBtn, ImVec2(190.0f, 0.0f))) StartAlignmentRepeat(a, alignRepeatN_);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Run the WHOLE procedure %d times - travel, settle, exposure\n"
                          "and detection every iteration, exactly as a recipe runs it.\n"
                          "Each run stores its own row below, and the capability figures\n"
                          "are computed from them. A failed run ends the sweep: a Cpk\n"
                          "over only the runs that worked is a lie by omission.",
                          alignRepeatN_);
    ImGui::SameLine();
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(140.0f);
    if (ImGui::InputInt("runs", &alignRepeatN_, 1, 5)) {
        if (alignRepeatN_ < kAlignRepeatMin) alignRepeatN_ = kAlignRepeatMin;
        if (alignRepeatN_ > kAlignRepeatMax) alignRepeatN_ = kAlignRepeatMax;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();

    if (!ready) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "assign a saved position to both stations first");
    } else if (a.s1.mode != "auto" || a.s2.mode != "auto") {
        // Worth saying before the sweep rather than after twenty picks: a manual
        // station asks the operator on EVERY iteration, and what comes out then
        // is the repeatability of a person, which is a real number but not the
        // one this button is usually pressed for.
        ImGui::TextDisabled("a manual station asks for a pick on every run - set both "
                            "stations to auto");
        ImGui::TextDisabled("to measure the DETECTOR's repeatability instead of the "
                            "operator's");
    }
    if (!alignStartErr_.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", alignStartErr_.c_str());
    if (running)
        ImGui::TextDisabled("running - %zu rows recorded so far", a.runs.size());

    // Capability over the last `alignRepeatN_` rows: the sweep's own runs when
    // one has just finished, and the most recent N otherwise. The window is the
    // sweep size deliberately - a figure computed over a different span than the
    // sweep would be answering a question nobody asked.
    const std::size_t want = static_cast<std::size_t>(alignRepeatN_);
    const std::size_t take = a.runs.size() < want ? a.runs.size() : want;
    if (take >= 2 && a.guard.enabled) {
        std::vector<double> spans, thetas;
        spans.reserve(take);
        thetas.reserve(take);
        for (std::size_t i = a.runs.size() - take; i < a.runs.size(); ++i) {
            spans.push_back(a.runs[i].spanMm);
            thetas.push_back(a.runs[i].thetaDeg);
        }
        const AlignCpk cs = ComputeAlignCpk(spans,  a.guard.spanMm,   a.guard.spanTolMm);
        const AlignCpk ct = ComputeAlignCpk(thetas, a.guard.thetaDeg, a.guard.thetaTolDeg);

        ImGui::SeparatorText("Capability");
        ImGui::TextDisabled("over the last %zu of %zu runs, against the band above",
                            take, a.runs.size());
        // 1.33 is the usual "capable" line and 1.00 the point at which the band
        // is exactly three sigma wide - the numbers are useless without them,
        // and an operator should not have to know the convention to read this.
        if (ImGui::BeginTable("alcpk", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                          ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("");
            ImGui::TableSetupColumn("mean");
            ImGui::TableSetupColumn("sigma");
            ImGui::TableSetupColumn("min");
            ImGui::TableSetupColumn("max");
            ImGui::TableSetupColumn("Cp");
            ImGui::TableSetupColumn("Cpk");
            ImGui::TableHeadersRow();

            const auto row = [](const char* label, const AlignCpk& c, const char* fmt) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                char buf[32];
                const auto cell = [&](int col, double v) {
                    ImGui::TableSetColumnIndex(col);
                    std::snprintf(buf, sizeof(buf), fmt, v);
                    ImGui::TextUnformatted(buf);
                };
                cell(1, c.mean);
                cell(2, c.sigma);
                cell(3, c.minV);
                cell(4, c.maxV);
                if (!c.valid) {
                    ImGui::TableSetColumnIndex(5);
                    ImGui::TextDisabled("-");
                    ImGui::TableSetColumnIndex(6);
                    ImGui::TextDisabled("no tolerance set");
                    return;
                }
                if (c.noSpread) {
                    ImGui::TableSetColumnIndex(5);
                    ImGui::TextDisabled("inf");
                    ImGui::TableSetColumnIndex(6);
                    ImGui::TextDisabled("inf (no measurable spread)");
                    return;
                }
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.2f", c.cp);
                ImGui::TableSetColumnIndex(6);
                const ImVec4 col = c.cpk >= 1.33 ? ImVec4(0.4f, 0.9f, 0.5f, 1.0f)
                                 : c.cpk >= 1.00 ? ImVec4(1.0f, 0.85f, 0.4f, 1.0f)
                                                 : ImVec4(1.0f, 0.55f, 0.3f, 1.0f);
                ImGui::TextColored(col, "%.2f", c.cpk);
            };
            row("span (mm)",  cs, "%.4f");
            row("theta (deg)", ct, "%.4f");
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Cpk >= 1.33 capable   1.00 = the band is exactly three sigma "
                            "wide   below that, runs will fall outside it");
        // Cp high with Cpk low is the signature worth naming: the spread is
        // fine, it is simply not where the band says it should be - a detector
        // on the wrong edge, or a reference taken from a different part.
        if (cs.valid && !cs.noSpread && cs.cp >= 1.33 && cs.cpk < 1.0)
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                               "span repeats well but sits off the reference by %+.4f mm - "
                               "re-take the reference, or check which feature is marked",
                               cs.mean - a.guard.spanMm);
    } else if (a.guard.enabled) {
        ImGui::TextDisabled("run the sweep to get a capability figure (needs 2+ runs)");
    } else {
        ImGui::TextDisabled("set a band above to get a capability figure from a sweep");
    }

    // The roll. Newest first: the question asked here is almost always "what did
    // the last few parts do", and scrolling to the bottom to answer it is the
    // kind of small friction that stops it being asked at all.
    ImGui::SeparatorText("Recent runs");
    ImGui::TextDisabled("%zu of %zu kept, newest first", a.runs.size(), kAlignRunHistoryMax);
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || a.runs.empty());
    if (ImGui::SmallButton("Clear history")) {
        a.runs.clear();
        session_.SaveAligns();   // taught by the machine, persisted like the frame
    }
    ImGui::EndDisabled();

    // A fixed ten rows rather than "whatever is left": the blocks above it grow
    // (a capability table appears once a sweep has run), and a table sized by
    // the remainder would shrink to nothing exactly when there is most to read.
    // Its own scrollbar holds the other ninety.
    const ImVec2 rollSize(0.0f, ImGui::GetTextLineHeightWithSpacing() * 11.0f);
    if (ImGui::BeginTable("alruns", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                       ImGuiTableFlags_ScrollY |
                                       ImGuiTableFlags_SizingStretchProp, rollSize)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("when",  ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("E1",    ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableSetupColumn("E2",    ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableSetupColumn("dx",    ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("dy",    ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("span",  ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableSetupColumn("theta", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableHeadersRow();
        for (std::size_t i = a.runs.size(); i-- > 0;) {
            const AlignRun& r = a.runs[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            // The verdict is the ROW's colour, not a column: an out-of-band run
            // has to be findable by glance in a list of a hundred.
            if (r.guarded && !r.ok)
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(90, 40, 30, 180));
            ImGui::TextUnformatted(r.at.empty() ? "-" : r.at.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.4f, %.4f", r.e1x, r.e1y);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.4f, %.4f", r.e2x, r.e2y);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%+.4f", r.dx);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%+.4f", r.dy);
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.4f", r.spanMm);
            ImGui::TableSetColumnIndex(6);
            ImGui::Text("%.3f", r.thetaDeg);
        }
        ImGui::EndTable();
    }
}

// ---- The derived frame ----------------------------------------------------

// The taught nominals: target points on the PART, held in local (O1)
// coordinates so they follow it. Teaching happens on its own takeover page
// (PageAlignNominal.cpp) — a mark needs a big live image, and no panel here is
// big enough. This is the fourth step of the middle panel: they are only
// listed, re-taught, deleted, and loaded into the local->machine probe on the
// Run step so "Move here" can drive to one.
//
// LIST on the left, ONE nominal's controls on the right. Every nominal used to
// carry its own copy of all six buttons, so a procedure with four of them put
// twenty-four buttons on the page and the numbered sequence — the only thing
// that says what to do first — was repeated four times over. Now the sequence
// appears once, and the list answers the question the buttons cannot: which
// point is the one about to be used.
void App::DrawAlignNominalList(AlignProc& a)
{
    const bool busy = orch_.Busy();

    // The four primary buttons carry their step number, because nothing about a
    // row of equal-looking SmallButtons says which one comes first, and the
    // answer is not guessable: 3 and 4 both LOOK like ways to use the nominal,
    // but 3 only drives the camera to check it. The maintenance buttons
    // (Go/Clear view, Re-teach, Delete) are deliberately left unnumbered and
    // pushed to the right of a gap - they are not part of the sequence, and
    // numbering them would claim they were.
    ImGui::SeparatorText("Nominals (local)");
    ImGui::TextDisabled("1 add the point   2 where the gripped object is watched from   "
                        "3 check it   4 place onto it");

    // The camera's own device, for the view buttons: teaching one captures its
    // live pose, so an unconnected device can only be told about, not asked.
    IMotionController* cam = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a.device) { cam = hardware_.MotionDevice(i); break; }
    }
    const bool camLive = cam && cam->IsConnected();

    // The step pane is this list's alone now, so both children take whatever
    // height is left rather than a fixed nine lines: a roll of nominals is
    // exactly the thing that grows, and it used to scroll inside a box with
    // empty pane below it.
    const float paneH = 0.0f;

    // ---- Left: New, then the roll of taught points --------------------------
    // A THIRD of the cell for the roll, never less than 150 px: it holds names,
    // and the pane beside it holds six buttons that must not wrap.
    float listW = ImGui::GetContentRegionAvail().x * 0.33f;
    if (listW < 150.0f) listW = 150.0f;
    ImGui::BeginChild("alnomlist", ImVec2(listW, paneH), ImGuiChildFlags_Borders);
    ImGui::BeginDisabled(busy);
    // "New", not "Teach": this button and the pane's "Re-teach" open the SAME
    // page, and the only difference is whether a uid is carried in. Calling
    // both of them "teach" said nothing about the one thing that separates
    // them — this one makes a nominal that did not exist, that one re-marks the
    // selected point and keeps everything hanging off its uid.
    if (ImGui::SmallButton("1. New nominal...")) OpenAlignNominalTeach(a, "");
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("STEP 1 - ADD a nominal: mark two features on the live image;\n"
                          "their MIDPOINT is stored in this frame's local coordinates, so\n"
                          "it names the same place on the part however the next one is\n"
                          "loaded. To re-mark one that already exists, select it and use\n"
                          "Re-teach - this one always makes a new nominal.");
    ImGui::Separator();
    if (a.nominals.empty()) ImGui::TextDisabled("none taught yet");
    for (const AlignNominal& n : a.nominals) {
        // Selection is by uid, so a row keeps naming the same point when one
        // above it is deleted. "(no view)" is the one fact worth carrying into
        // the list rather than leaving to the pane: without one, step 4 opens
        // looking wherever the gantry happens to be standing.
        const bool hasView = AlignNominalView(a, n) != nullptr;
        char row[80];
        std::snprintf(row, sizeof(row), "%s%s", n.name.c_str(),
                      hasView ? "" : "   (no view)");
        ImGui::PushID(n.uid.c_str());
        if (ImGui::Selectable(row, alignNomSel_ == n.uid)) alignNomSel_ = n.uid;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("local X %.4f  Y %.4f mm\ntaught %s\ncamera view: %s",
                              n.lx, n.ly, n.taughtAt.c_str(),
                              hasView ? "taught" : "none");
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // Resolve the selection AFTER the list, so a nominal taught or deleted this
    // frame is already accounted for. Empty or stale falls back to the first
    // one: a pane of controls with no subject is a pane that cannot be used.
    AlignNominal* sel = alignNomSel_.empty() ? nullptr : FindAlignNominal(a, alignNomSel_);
    if (!sel && !a.nominals.empty()) {
        sel = &a.nominals.front();
        alignNomSel_ = sel->uid;
    }

    // ---- Right: everything you can do to the SELECTED nominal ---------------
    // Deletion is DEFERRED out of this child: erasing from the vector the pane
    // is reading invalidates `sel` mid-draw.
    std::string dropUid;
    ImGui::BeginChild("alnomdetail", ImVec2(0.0f, paneH), ImGuiChildFlags_Borders);
    if (!sel) {
        ImGui::TextDisabled("Nothing selected - teach the first nominal.");
    } else {
        AlignNominal& n = *sel;
        ImGui::PushID(n.uid.c_str());
        ImGui::TextUnformatted(n.name.c_str());
        ImGui::TextDisabled("  X %.4f  Y %.4f mm   taught %s", n.lx, n.ly,
                            n.taughtAt.c_str());
        ImGui::Separator();

        // The CAMERA VIEW for this nominal — where the gantry stands to see the
        // object being placed here, which is neither the nominal's own XY nor
        // its focus height. Taught from wherever the camera is now; a placement
        // drives here itself, so nothing in front of it has to.
        // NOTE: `vp` points into the positions vector; it is re-resolved below,
        // after the buttons that can reallocate or erase from it.
        const SavedPos* vp = AlignNominalView(a, n);
        ImGui::BeginDisabled(busy || !camLive);
        if (ImGui::SmallButton(vp ? "2. Re-teach view" : "2. Teach view"))
            TeachAlignNominalView(a, n);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("STEP 2 - The view of the GRIPPED OBJECT, not of the bare\n"
                              "nominal. Grip a real one, bring it to where it will hover\n"
                              "over this point, then jog %s until its two features are\n"
                              "sharp and centred and press this.\n"
                              "The Z that focuses the part surface does NOT focus an\n"
                              "object held above it: teach this with an empty gripper and\n"
                              "every mark - and every model cut from one - comes out\n"
                              "blurred. A placement drives here before it marks, so\n"
                              "nothing in front of the run has to.",
                              a.device.c_str());
        // Same gap as on the row below: what follows checks or undoes step 2,
        // it does not advance the sequence.
        ImGui::SameLine(0.0f, 24.0f);
        // Go: a plain UI move to a pose a human taught, like a station's — not
        // the armed class, which is for COMPUTED destinations.
        ImGui::BeginDisabled(busy || !camLive || !vp);
        if (ImGui::SmallButton("Go view")) GoToAlignNominalView(a, n);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !vp);
        if (ImGui::SmallButton("Clear view")) DropAlignNominalView(a, n);
        ImGui::EndDisabled();
        // Re-resolve: `vp` points INTO the positions vector, and both buttons
        // above can push into it or erase from it.
        vp = AlignNominalView(a, n);
        if (vp) {
            // A private position is not on the Motion page, so its numbers only
            // exist here.
            std::string line;
            for (const auto& [axis, v] : vp->vals) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%s %.3f", line.empty() ? "" : "  ",
                              axis.c_str(), v);
                line += buf;
            }
            ImGui::TextDisabled("  view  %s", line.c_str());
        } else {
            ImGui::TextDisabled("  view  none - a placement will not move the camera");
        }

        ImGui::BeginDisabled(busy);
        // Use: copy into the local->machine probe above. The probe is what
        // "Move here" reads, so this is the only wiring a nominal needs to
        // become a destination — and it still goes through the live-frame gate.
        if (ImGui::SmallButton("3. Use")) {
            alignProbeX_ = n.lx;
            alignProbeY_ = n.ly;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("STEP 3 - Load this nominal into the local X / local Y\n"
                              "boxes above. Tick Arm, then Move here drives the camera\n"
                              "to it - which is how you check the point is where you\n"
                              "think it is. Needs a LIVE frame: run the procedure first.");
        ImGui::SameLine();
        // Place: measure where a gripped object actually is and work out the
        // correction onto this nominal. Opening it drives the CAMERA to this
        // nominal's view (step 2) — the marking it asks for is made through the
        // camera, so it has to be looking at the right place first. The OBJECT
        // does not move until it is asked to, on that page.
        if (ImGui::SmallButton("4. Place...")) OpenAlignPlace(a, n.uid);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("STEP 4 - Drives %s to this nominal's camera view,\n"
                              "then opens the page to mark the gripped object; the\n"
                              "correction onto this nominal is computed from it. The\n"
                              "object itself does not move until you ask for it there.",
                              a.device.c_str());
        // A gap, then the two that undo rather than advance.
        ImGui::SameLine(0.0f, 24.0f);
        if (ImGui::SmallButton("Re-teach")) OpenAlignNominalTeach(a, n.uid);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Re-mark the two features of THIS nominal, on the part that\n"
                              "is on the machine now. Same page as step 1, but it keeps\n"
                              "the uid - so its camera view, its detection models and any\n"
                              "recipe step naming it all survive. (Delete + New would not.)");
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) dropUid = n.uid;
        ImGui::EndDisabled();
        ImGui::PopID();
    }
    ImGui::EndChild();

    // The deferred delete, now that nothing points into the vector any more.
    if (!dropUid.empty()) {
        for (std::size_t i = 0; i < a.nominals.size(); ++i) {
            if (a.nominals[i].uid != dropUid) continue;
            FD_INFO("align", "'%s': dropped nominal '%s'", a.name.c_str(),
                    a.nominals[i].name.c_str());
            DropNominalDetectors(a, a.nominals[i]);   // its private object models go too
            DropAlignNominalView(a, a.nominals[i]);   // and its private camera view
            a.nominals.erase(a.nominals.begin() + static_cast<long>(i));
            session_.SaveAligns();   // taught by hand — persisted like the frame
            alignNomSel_.clear();    // the fallback above picks the first one
            break;
        }
    }
}

// The frame's two ACTIONS - check a local coordinate against it, and drive
// there. Split out of the right-hand frame panel so it can sit under Run on the
// middle panel's Run step: the right column is the RECORD of what was taught,
// the middle is what you do with it.
//
// It resolves the frame itself rather than taking one as a parameter. The LIVE
// frame is the only one that may move the stage, and a caller that handed in
// `a.result` would be exactly the bug that distinction exists to prevent.
void App::DrawAlignFrameActions(AlignProc& a)
{
    AlignResult live;
    const bool haveLive = workflow_.LiveFrame(a.uid, &live);
    const AlignResult& r = haveLive ? live : a.result;
    // Nothing has ever been taught, so there is no transform to probe against.
    // Silent rather than a disabled block: the frame panel already says so.
    if (!haveLive && !a.result.valid) return;

    // Check the frame, then optionally drive to the point — the same transform
    // the move_local command applies. No document mutation either way.
    ImGui::SeparatorText("Local -> machine");
    float lx = static_cast<float>(alignProbeX_), ly = static_cast<float>(alignProbeY_);
    ImGui::SetNextItemWidth(110.0f);
    if (ImGui::InputFloat("local X", &lx, 0.0f, 0.0f, "%.4f")) alignProbeX_ = lx;
    ImGui::SetNextItemWidth(110.0f);
    if (ImGui::InputFloat("local Y", &ly, 0.0f, 0.0f, "%.4f")) alignProbeY_ = ly;
    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(r, alignProbeX_, alignProbeY_, &mx, &my);
    ImGui::Text("machine  %.4f, %.4f mm", mx, my);
    ImGui::TextDisabled("(0,0) is E1; (%.4f, 0) is E2", r.spanMm);

    // Drive there. XY only — the frame is 2-D, so Z is deliberately left where
    // it is rather than being invented. Behind the shared jog arming flag: the
    // destination is COMPUTED (a mis-taught frame sends the stage somewhere
    // unexpected), which is the same risk class as click-to-move, unlike the
    // station cards' Go, which drives to a position a human taught.
    ImGui::Spacing();
    int devIdx = -1;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a.device) { devIdx = static_cast<int>(i); break; }
    }
    IMotionController* dev =
        devIdx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(devIdx)) : nullptr;
    // haveLive is part of the gate, not just the display: this destination is
    // COMPUTED from a frame, so it carries exactly the hazard move_local
    // refuses. A button that would happily drive against a record of last
    // week's part is the same bug with a friendlier face.
    const bool canMove =
        haveLive && jogArmed_ && !orch_.Busy() && dev && dev->IsConnected();

    ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms
    ImGui::SameLine();
    ImGui::BeginDisabled(!canMove);
    if (ImGui::Button("Move here", ImVec2(110.0f, 0.0f))) {
        const std::vector<std::pair<std::string, double>> targets = {{"X", mx}, {"Y", my}};
        if (!workflow_.MoveToPosition(static_cast<std::size_t>(devIdx), targets))
            FD_WARN("align", "move to local (%.4f, %.4f) rejected", alignProbeX_, alignProbeY_);
    }
    ImGui::EndDisabled();
    if (!haveLive)
        ImGui::TextDisabled("no live frame - run the procedure");
    else if (!dev || !dev->IsConnected())
        ImGui::TextDisabled("%s not connected", a.device.c_str());
    else if (!jogArmed_)
        ImGui::TextDisabled("arm first");
    else if (orch_.Busy())
        ImGui::TextDisabled("a run is active");
}

void App::DrawAlignFramePanel()
{
    std::vector<AlignProc>& list = session_.Aligns();
    if (alignSel_ < 0 || alignSel_ >= static_cast<int>(list.size())) {
        ImGui::TextDisabled("Local frame (O1)");
        return;
    }
    AlignProc& a = list[static_cast<std::size_t>(alignSel_)];

    // LIVE frame first, because that is the one that can actually move the
    // stage. The document's copy is a record of the last teach and may describe
    // a part that left the machine days ago — showing them as one number would
    // be the whole safety argument undone in the UI.
    AlignResult live;
    const bool haveLive = workflow_.LiveFrame(a.uid, &live);
    const AlignResult& r = haveLive ? live : a.result;

    ImGui::TextUnformatted("Local frame (O1)");
    ImGui::Separator();

    if (haveLive) {
        ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.5f, 1.0f), "LIVE  taught %s",
                           live.capturedAt.c_str());
        ImGui::TextDisabled("usable by move_local until the next program start,");
        ImGui::TextDisabled("DUT open or product load.");
    } else if (a.result.valid) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "NO LIVE FRAME");
        ImGui::TextWrapped("The numbers below are the RECORD of the last teach (%s). "
                           "move_local will refuse until this procedure is run again "
                           "in the current session - a saved frame describes where a "
                           "part sat then, not where this one sits now.",
                           a.result.capturedAt.c_str());
        ImGui::Spacing();
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "not taught yet");
        ImGui::TextWrapped("Run the procedure: E1 becomes the origin and E1->E2 the "
                           "local X+ axis; local Y+ is X+ rotated +90 degrees.");
        return;
    }
    ImGui::Spacing();
    ImGui::Text("origin   %.4f, %.4f mm", r.originX, r.originY);
    ImGui::Text("theta    %.4f deg", r.thetaDeg);
    ImGui::Text("span     %.4f mm", r.spanMm);
    ImGui::Spacing();
    ImGui::SeparatorText("Marked points (machine mm)");
    ImGui::Text("E1  %.4f, %.4f", r.e1.wx, r.e1.wy);
    ImGui::TextDisabled("    gantry %.4f, %.4f  px %.0f, %.0f", r.e1.gx, r.e1.gy,
                        r.e1.px, r.e1.py);
    ImGui::Text("E2  %.4f, %.4f", r.e2.wx, r.e2.wy);
    ImGui::TextDisabled("    gantry %.4f, %.4f  px %.0f, %.0f", r.e2.gx, r.e2.gy,
                        r.e2.px, r.e2.py);
}

} // namespace firedog
