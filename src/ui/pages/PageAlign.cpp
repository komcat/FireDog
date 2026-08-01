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

#include "core/Log.h"

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

} // namespace

// Launch the procedure script. Same channel the Motion page's XCAN button uses:
// parameters go in through LuaEngine::Env::initialCtx, where fd.param reads
// them. Gated on the WHOLE orchestration, never a subset of the engines.
void App::StartAlignmentRun(const AlignProc& a)
{
    // The product's copy of the script wins over the shipped one.
    std::string script = "config/scripts/alignment_procedure.lua";
    for (const ScriptInfo& si : session_.Scripts()) {
        if (si.name == "alignment_procedure") { script = si.path; break; }
    }
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"align_name", a.uid},        // uid, not name: a rename mid-run is harmless
        {"align_prefix", std::string("frame")},
    };
    alignStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &alignStartErr_))
        FD_WARN("align", "alignment run rejected: %s", alignStartErr_.c_str());
}

// Capture the device's CURRENT pose as this station's private position.
// Update-in-place when the procedure already owns one, so the uid — and
// therefore anything referencing it — survives a re-teach.
void App::TeachAlignStation(AlignProc& a, int station)
{
    IMotionController* dev = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a.device) { dev = hardware_.MotionDevice(i); break; }
    }
    if (!dev || !dev->IsConnected()) return;

    const std::string name = StationPosName(a.uid, station);
    std::vector<SavedPos>& list = session_.Positions()[a.device];
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
    (station == 1 ? a.s1 : a.s2).posUid = tgt->uid;
    session_.SavePositions();
    FD_INFO("align", "'%s' station %d taught as '%s'", a.name.c_str(), station, name.c_str());
}

// Erase the private positions a procedure owns. Without this they would linger
// invisibly: hidden rows are off the Motion table by default, so an orphan is
// something nobody would ever go looking for.
void App::DropAlignStationPositions(const AlignProc& a)
{
    const auto it = session_.Positions().find(a.device);
    if (it == session_.Positions().end()) return;
    std::vector<SavedPos>& list = it->second;
    const std::size_t before = list.size();
    for (std::size_t i = list.size(); i-- > 0;) {
        if (IsOwnedStationPos(list[i], a.uid)) list.erase(list.begin() + static_cast<long>(i));
    }
    if (list.size() != before) {
        session_.SavePositions();
        FD_INFO("align", "'%s': dropped %zu private station position(s)",
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
        list.push_back(a);
        alignSel_ = static_cast<int>(list.size()) - 1;
        alignNameBuf_[0] = '\0';
        session_.SavePositions();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Delete")) {
        DropAlignStationPositions(list[static_cast<std::size_t>(alignSel_)]);
        list.erase(list.begin() + alignSel_);
        alignSel_ = list.empty() ? -1 : 0;
        alignNameBuf_[0] = '\0';
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

// ---- Middle: the two stations + Run ---------------------------------------

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

    // Auto mode is structure only — there is no feature detector in the build.
    int mode = (st.mode == "auto") ? 1 : 0;
    if (ImGui::RadioButton("Manual (crosshair)", mode == 0)) st.mode = "manual";
    ImGui::SameLine();
    if (ImGui::RadioButton("Auto (detect)", mode == 1)) st.mode = "auto";
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("No feature detector is implemented yet - an auto station\n"
                          "fails at run time unless it may fall back to manual.");
    if (st.mode == "auto") {
        ImGui::Indent();
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "feature detection is not implemented");
        ImGui::Checkbox("fall back to a manual pick", &st.autoFallbackManual);
        ImGui::Unindent();
    }
    ImGui::EndDisabled();
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

    DrawAlignStationCard(a, 1);
    DrawAlignStationCard(a, 2);

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
}

// ---- Right: the derived frame ---------------------------------------------

void App::DrawAlignFramePanel()
{
    const std::vector<AlignProc>& list = session_.Aligns();
    if (alignSel_ < 0 || alignSel_ >= static_cast<int>(list.size())) {
        ImGui::TextDisabled("Local frame (O1)");
        return;
    }
    const AlignProc&  a = list[static_cast<std::size_t>(alignSel_)];
    const AlignResult& r = a.result;

    ImGui::TextUnformatted("Local frame (O1)");
    ImGui::Separator();
    if (!r.valid) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "not taught yet");
        ImGui::TextWrapped("Run the procedure: E1 becomes the origin and E1->E2 the "
                           "local X+ axis; local Y+ is X+ rotated +90 degrees.");
        return;
    }

    ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.5f, 1.0f), "taught %s", r.capturedAt.c_str());
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
    const bool canMove = jogArmed_ && !orch_.Busy() && dev && dev->IsConnected();

    ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms
    ImGui::SameLine();
    ImGui::BeginDisabled(!canMove);
    if (ImGui::Button("Move here", ImVec2(110.0f, 0.0f))) {
        const std::vector<std::pair<std::string, double>> targets = {{"X", mx}, {"Y", my}};
        if (!workflow_.MoveToPosition(static_cast<std::size_t>(devIdx), targets))
            FD_WARN("align", "move to local (%.4f, %.4f) rejected", alignProbeX_, alignProbeY_);
    }
    ImGui::EndDisabled();
    if (!dev || !dev->IsConnected())
        ImGui::TextDisabled("%s not connected", a.device.c_str());
    else if (!jogArmed_)
        ImGui::TextDisabled("arm first");
    else if (orch_.Busy())
        ImGui::TextDisabled("a run is active");
}

} // namespace firedog
