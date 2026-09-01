// PageAlignNominal.cpp : the Nominal teach page (a full-page takeover, reached
// only from the Alignment page's frame panel). Definitions of App members —
// see App.h.
//
// A nominal is a target point on the PART — where something is meant to end up —
// held in the alignment's local frame (O1) rather than in machine coordinates.
// That is the whole point: the frame moves with each part, so a nominal taught
// once against one part names the same physical place on every part after it,
// without anybody re-teaching anything.
//
// It is taught as the CENTRE of two or four marks, because the point that
// matters is usually the centre of something — a die, a pocket, a footprint —
// and a centre has nothing under it to aim a crosshair at. Marking corners puts
// the crosshair on a real feature every time and leaves the centre to
// arithmetic.
//
// Two marks or four is a real choice, not a preference. Two opposite corners
// give the centre in one stroke and cannot check themselves: a mark that lands
// off its feature moves the centre by half its error and looks exactly like a
// good mark. Four give the same centre as the average of a ring, and a spread
// that disagrees with the part is the only thing that can show such a mark up.
// The registration's imprint makes the identical trade for the identical reason.
//
// The page can also draw a REGISTERED TOOL's tip and a resizable box centred on
// it. That box is a viewing aid and is never stored: a nominal is a point on the
// part, and how wide the gripper is says nothing about where that point belongs.
// What it answers is the question an operator actually has while teaching one —
// will the tool clear what is beside it — and no number on the panel answers it.
//
// NOTHING MOVES here. The page reads the live image and the device's pose and
// writes a document field, so it needs no arming checkbox — the same argument
// the crosshair pick window makes. Jogging between the two marks is expected
// and safe: each mark latches the pose it was taken at, so the two may come
// from opposite ends of the travel.
//
// Why a takeover and not a panel on the Alignment page: marking a feature needs
// a big live image, and the Alignment page's columns are strips. It follows the
// vision teach page in every respect, and the image itself is
// `DrawMarkPicker` (ui/Widgets.h) — the same mapping the crosshair pick window
// uses, so a pixel means the same thing on every surface that has one.
#include "App.h"

#include "imgui.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid
#include "ui/Widgets.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);

// Trim a UI name buffer; empty falls back so a nominal is never nameless (the
// name is how a script addresses it).
std::string CleanName(const char* buf)
{
    std::string s = buf ? buf : "";
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.erase(s.begin());
    while (!s.empty() && (s.back()  == ' ' || s.back()  == '\t')) s.pop_back();
    return s.empty() ? std::string("nominal") : s;
}

} // namespace

// ---- Opening ---------------------------------------------------------------

// `nomUid` empty = teach a NEW nominal; otherwise re-teach that one, keeping its
// uid so anything referencing it survives. The marks always start empty even on
// a re-teach: the stored ones describe the part that was on the machine when it
// was first taught, and re-teaching means marking this part.
void App::OpenAlignNominalTeach(const AlignProc& a, const std::string& nomUid)
{
    // Level checked at the TOP, not left to SwitchPage's refusal: everything
    // below mutates state (the model, the marks, the selections) before it
    // switches, so a refused switch would leave those edits behind on a page
    // the caller never reaches. Same reason the Run deck's right-click checks
    // early. Every caller today sits on an Admin page, so this is insurance,
    // not a fix — but it is what makes the rule true everywhere.
    if (!CanAccess(Page::AlignNominal)) return;
    nomProcUid_ = a.uid;
    nomEditUid_ = nomUid;
    for (int i = 0; i < kNomMarkMax; ++i) {
        nomHave_[i] = false;
        nomPt_[i]   = AlignPoint{};
    }
    nomActive_  = 0;
    nomZoom_    = 1.5f;
    nomEnvDeg_  = 0.0f;
    nomErr_.clear();
    nomPx_ = (cameraW_ > 0) ? static_cast<float>(cameraW_) * 0.5f : 0.0f;
    nomPy_ = (cameraH_ > 0) ? static_cast<float>(cameraH_) * 0.5f : 0.0f;

    const AlignNominal* n = FindAlignNominal(a, nomUid);
    // A re-teach opens on the count it was taught with: that is a fact about the
    // feature - four corners on a square pocket stay four - and asking the
    // operator to re-choose it every time is how a four-mark nominal silently
    // becomes a two-mark one.
    if (n) nomCount_ = (n->points == 4) ? 4 : 2;
    const std::string name =
        n ? n->name : ("nominal " + std::to_string(a.nominals.size() + 1));
    std::snprintf(nomNameBuf_, sizeof(nomNameBuf_), "%s", name.c_str());

    SwitchPage(Page::AlignNominal);   // every page switch routes through here
}

// ---- The page --------------------------------------------------------------

void App::DrawAlignNominalPanel()
{
    std::vector<AlignProc>& list = session_.Aligns();
    AlignProc* a = FindAlignProc(list, nomProcUid_);
    if (!a) {
        ImGui::TextDisabled("The alignment procedure this page was opened for is gone.");
        if (ImGui::Button("Back to Alignment")) SwitchPage(Page::Align);
        return;
    }

    // The LIVE frame, never the recorded one. A nominal is stored in local
    // coordinates, so the conversion that produces them has to come from a frame
    // that describes the part actually under the camera — teaching against last
    // week's record would produce local numbers that describe nothing here, and
    // every later part would then be driven to them.
    AlignResult frame;
    const bool haveLive = workflow_.LiveFrame(a->uid, &frame);

    // The pose the marks are measured against: the procedure's own device, the
    // same one align_station reads. RequestFastPoll every frame — a demand
    // signal, not a mode (the Motion page does exactly this).
    IMotionController* dev = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a->device) { dev = hardware_.MotionDevice(i); break; }
    }
    double gx = 0.0, gy = 0.0;
    bool   havePose = false;
    if (dev && dev->IsConnected()) {
        dev->RequestFastPoll();
        const std::map<std::string, double> pos = dev->Positions();
        const auto ix = pos.find("X"), iy = pos.find("Y");
        if (ix != pos.end() && iy != pos.end()) {
            gx = ix->second;
            gy = iy->second;
            havePose = true;
        }
    }

    UpdateCameraTexture();   // newest camera frame into the GL texture
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
    const bool canMark = haveImg && havePose;

    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  rightW = 360.0f;
    const float  imgW   = avail.x - rightW - ImGui::GetStyle().ItemSpacing.x;

    // ---- Left: the live image ----------------------------------------------
    ImGui::BeginChild("anImg", ImVec2(imgW, avail.y), ImGuiChildFlags_Borders);
    {
        CameraMark marks[kNomMarkMax];
        static const char* kLbl[kNomMarkMax] = {"pt1", "pt2", "pt3", "pt4"};
        for (int i = 0; i < nomCount_; ++i) {
            marks[i].wx    = nomPt_[i].wx;
            marks[i].wy    = nomPt_[i].wy;
            marks[i].have  = nomHave_[i];
            marks[i].label = kLbl[i];
        }
        MarkPicker mp;
        mp.tex           = cameraTex_;
        mp.imgW          = cameraW_;
        mp.imgH          = cameraH_;
        mp.pixelToMmX    = pixelToMmX_;
        mp.pixelToMmY    = pixelToMmY_;
        mp.havePose      = havePose;
        mp.gx            = gx;
        mp.gy            = gy;
        mp.marks         = marks;
        mp.markCount     = nomCount_;
        mp.midpointLabel = "nominal";
        // The tool, if one is chosen and its tip can be predicted right now.
        // Same refusal as everywhere else: a tip that cannot be predicted draws
        // nothing, and the panel says why - the picture never carries a guess.
        if (nomToolDev_[0]) {
            RegVec3     tip;
            std::string why;
            if (workflow_.DeviceRegTip(nomToolDev_, tip, &why)) {
                mp.toolHave    = true;
                mp.toolWx      = tip.x;
                mp.toolWy      = tip.y;
                mp.toolLabel   = nomToolDev_;
                mp.boxWmm      = &nomBoxW_;
                mp.boxHmm      = &nomBoxH_;
                mp.boxDragging = &nomBoxDrag_;
            }
        }
        mp.envelopeDeg   = nomEnvDeg_;
        EnsureMarkIcons();
        mp.icons[0]      = markIcons_[0];
        mp.icons[1]      = markIcons_[1];
        mp.activeMark    = nomActive_;
        mp.px            = &nomPx_;
        mp.py            = &nomPy_;
        mp.zoom          = &nomZoom_;
        const MarkPickerResult r =
            DrawMarkPicker("##nomInput", ImGui::GetContentRegionAvail(), mp);
        // Right-click drove somewhere: click-to-move is an armed action (the
        // rule the Aux window's version follows), and layer 2 owns the move.
        if (r.moveRequested) {
            if (jogArmed_) workflow_.MoveGantryRelativeMM(r.dxMM, r.dyMM);
            else FD_WARN("align", "click-to-move ignored - arm jog first");
        }
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: the marks, the arithmetic, and Save -------------------------
    ImGui::BeginChild("anCtl", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);

    ImGui::TextUnformatted(a->name.c_str());
    ImGui::TextDisabled("teach a nominal target in the local frame (O1)");
    ImGui::Separator();

    if (haveLive) {
        ImGui::TextColored(kGood, "LIVE frame  taught %s", frame.capturedAt.c_str());
        ImGui::TextDisabled("origin %.4f, %.4f mm   theta %.4f deg",
                            frame.originX, frame.originY, frame.thetaDeg);
    } else {
        ImGui::TextColored(kBad, "NO LIVE FRAME");
        ImGui::TextWrapped("A nominal is stored in LOCAL coordinates, so the frame that "
                           "converts it has to describe the part under the camera right "
                           "now. Run the procedure on the Alignment page first - taught "
                           "through the record of an older part, the numbers would "
                           "describe nothing on this one, and every part after it would "
                           "be driven to them.");
    }
    ImGui::Separator();

    if (havePose)
        ImGui::TextDisabled("%s  X %.4f  Y %.4f mm", a->device.c_str(), gx, gy);
    else
        ImGui::TextColored(kBad, "%s not connected (or reports no X/Y)", a->device.c_str());
    if (!haveImg)
        ImGui::TextColored(kBad, "no camera feed");

    ImGui::SeparatorText("Marks");
    ImGui::TextDisabled("Mark features whose CENTRE is the target -");
    ImGui::TextDisabled("corners of the footprint, say.");

    // Two or four. Switching down to two keeps marks 3 and 4 in memory but
    // out of the arithmetic, so a mis-click on the selector costs nothing;
    // switching up asks for the two that are missing.
    {
        int want = nomCount_;
        ImGui::TextDisabled("marks");
        ImGui::SameLine();
        if (ImGui::RadioButton("2", &want, 2)) {}
        ImGui::SameLine();
        if (ImGui::RadioButton("4", &want, 4)) {}
        if (want != nomCount_) {
            nomCount_ = want;
            // Point at the first one still missing, so the button that says
            // "next" is the one that is actually next.
            nomActive_ = 0;
            for (int i = 0; i < nomCount_; ++i)
                if (!nomHave_[i]) { nomActive_ = i; break; }
            nomErr_.clear();
        }
        ImGui::SameLine();
        ImGui::TextDisabled(nomCount_ == 2
                                ? "opposite corners"
                                : "a ring - the spread checks them");
    }

    const auto capture = [&](int i) {
        AlignPoint p;
        p.gx = gx;
        p.gy = gy;
        p.px = nomPx_;
        p.py = nomPy_;
        p.wx = gx + static_cast<double>(nomPx_ - static_cast<float>(cameraW_) * 0.5f) *
                        static_cast<double>(pixelToMmX_);
        p.wy = gy + static_cast<double>(nomPy_ - static_cast<float>(cameraH_) * 0.5f) *
                        static_cast<double>(pixelToMmY_);
        nomPt_[i]   = p;
        nomHave_[i] = true;
        // The next one still missing - which is "the other one" for a pair and
        // the next corner of the ring for four.
        nomActive_ = i;
        for (int k = 1; k <= nomCount_; ++k) {
            const int j = (i + k) % nomCount_;
            if (!nomHave_[j]) { nomActive_ = j; break; }
        }
        nomErr_.clear();
    };

    for (int i = 0; i < nomCount_; ++i) {
        ImGui::PushID(i);
        char btn[32];
        std::snprintf(btn, sizeof(btn), "Set pt%d here", i + 1);
        ImGui::BeginDisabled(!canMark);
        if (ImGui::Button(btn, ImVec2(120.0f, 0.0f))) capture(i);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (nomHave_[i]) {
            ImGui::Text("%.4f, %.4f", nomPt_[i].wx, nomPt_[i].wy);
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear")) { nomHave_[i] = false; nomActive_ = i; }
            ImGui::TextDisabled("   %s at %.4f, %.4f   px %.0f, %.0f",
                                a->device.c_str(), nomPt_[i].gx, nomPt_[i].gy,
                                nomPt_[i].px, nomPt_[i].py);
        } else {
            ImGui::TextDisabled(nomActive_ == i ? "(not marked - next)" : "(not marked)");
        }
        ImGui::PopID();
    }
    ImGui::TextDisabled("Jog between the marks freely: each latches the");
    ImGui::TextDisabled("pose it was taken at.");

    ImGui::SeparatorText("Result");
    bool both = true;
    for (int i = 0; i < nomCount_; ++i) both = both && nomHave_[i];
    if (!both) {
        ImGui::TextDisabled("mark all %d points", nomCount_);
    } else {
        // The same average the solver computes. Written here rather than
        // called out of it because the solver needs a live frame and this
        // panel must read the centre back with or without one.
        double mx = 0.0, my = 0.0;
        for (int i = 0; i < nomCount_; ++i) { mx += nomPt_[i].wx; my += nomPt_[i].wy; }
        mx /= nomCount_;
        my /= nomCount_;
        double spread = 0.0;
        if (nomCount_ == 2) {
            const double dx = nomPt_[1].wx - nomPt_[0].wx;
            const double dy = nomPt_[1].wy - nomPt_[0].wy;
            spread = std::sqrt(dx * dx + dy * dy);
        } else {
            for (int i = 0; i < nomCount_; ++i) {
                const double dx = nomPt_[i].wx - mx, dy = nomPt_[i].wy - my;
                spread += std::sqrt(dx * dx + dy * dy);
            }
            spread = 2.0 * spread / nomCount_;
        }
        ImGui::Text("centre    %.4f, %.4f mm", mx, my);
        ImGui::Text("%s  %.4f mm", nomCount_ == 2 ? "span     " : "spread   ", spread);
        if (nomCount_ == 4)
            ImGui::TextDisabled("twice the mean distance from the centre - a width");

        // Overlay only. A footprint marked at an angle still has the same
        // midpoint - the marks are two measured points and their average does
        // not turn - so this spins the drawn box and nothing else. It is here to
        // let the operator SEE the outline sit on the part; if it had to be
        // stored, it would belong on the frame, not on a nominal.
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat("box angle", &nomEnvDeg_, 0.5f, -180.0f, 180.0f, "%.1f deg");
        ImGui::SameLine();
        if (ImGui::SmallButton("0")) nomEnvDeg_ = 0.0f;
        ImGui::TextDisabled("drawing only - the saved midpoint is unaffected");
        if (haveLive) {
            double lx = 0.0, ly = 0.0;
            AlignMachineToLocal(frame, mx, my, &lx, &ly);
            ImGui::TextColored(kGood, "local X %.4f   local Y %.4f mm", lx, ly);
            ImGui::TextDisabled("this pair is what gets saved");
        } else {
            ImGui::TextColored(kWarn, "no live frame - nothing to convert through");
        }
    }

    ImGui::SeparatorText("Tool footprint");
    // A viewing aid, and the panel says so where somebody might mistake it
    // for part of the teach: nothing here is saved on the nominal.
    ImGui::TextDisabled("Drawn only - none of this is stored on the nominal.");
    {
        // The registered devices, plus "none". Only tools registered against
        // THIS image's gantry can be drawn on it: a tip in another origin,
        // projected through this pose, is off by the distance between them.
        const std::string refName = workflow_.GantryName();
        const char* cur = nomToolDev_[0] ? nomToolDev_ : "(none)";
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo("tool", cur)) {
            if (ImGui::Selectable("(none)", nomToolDev_[0] == 0)) nomToolDev_[0] = 0;
            for (const auto& [name, r] : workflow_.DeviceRegs()) {
                if (!r.valid || r.refDevice != refName) continue;
                const bool sel = name == nomToolDev_;
                if (ImGui::Selectable(name.c_str(), sel))
                    std::snprintf(nomToolDev_, sizeof(nomToolDev_), "%s", name.c_str());
            }
            ImGui::EndCombo();
        }
        if (nomToolDev_[0]) {
            RegVec3     tip;
            std::string why;
            if (workflow_.DeviceRegTip(nomToolDev_, tip, &why)) {
                ImGui::TextColored(kGood, "tip %.4f, %.4f mm", tip.x, tip.y);
                ImGui::SetNextItemWidth(90.0f);
                if (ImGui::InputFloat("w mm", &nomBoxW_, 0.0f, 0.0f, "%.3f"))
                    SaveUiSettings();
                ImGui::SameLine();
                ImGui::SetNextItemWidth(90.0f);
                if (ImGui::InputFloat("h mm", &nomBoxH_, 0.0f, 0.0f, "%.3f"))
                    SaveUiSettings();
                nomBoxW_ = fmaxf(0.02f, nomBoxW_);
                nomBoxH_ = fmaxf(0.02f, nomBoxH_);
                ImGui::TextDisabled("drag a corner of the box on the image to resize;");
                ImGui::TextDisabled("it stays centred on the tip, always.");
            } else {
                // The reason, in full, on the panel - the picture stays clean
                // and the operator still learns why there is no box.
                ImGui::TextColored(kWarn, "no tip: %s", why.c_str());
            }
        }
    }
    // Persist the box on the drag END, the way every splitter in this app does -
    // and outside the branches above, so a tip that goes unavailable mid-drag
    // (the device turned, the controller dropped) still writes what was set.
    if (nomBoxWasDrag_ && !nomBoxDrag_) SaveUiSettings();
    nomBoxWasDrag_ = nomBoxDrag_;

    ImGui::Separator();
    ImGui::SetNextItemWidth(200.0f);
    ImGui::InputText("name", nomNameBuf_, sizeof(nomNameBuf_));

    const bool canSave = both && haveLive;
    ImGui::BeginDisabled(!canSave);
    if (ImGui::Button("Save nominal", ImVec2(140.0f, 0.0f))) {
        AlignNominal n;
        std::string err;
        if (!SolveAlignNominalN(frame, nomPt_, nomCount_, &n, &err)) {
            nomErr_ = err;
        } else {
            n.name = CleanName(nomNameBuf_);
            // A name is how a script addresses a nominal, so two of them under
            // one procedure would make that lookup a coin toss. Refuse rather
            // than silently renaming.
            const AlignNominal* clash = FindAlignNominal(*a, n.name);
            if (clash && clash->uid != nomEditUid_) {
                nomErr_ = "'" + n.name + "' already exists in this procedure";
            } else {
                AlignNominal* tgt = FindAlignNominal(*a, nomEditUid_);
                if (tgt) {
                    n.uid = tgt->uid;    // re-teach keeps the identity
                    *tgt  = n;
                } else {
                    n.uid = GenerateUid();
                    a->nominals.push_back(n);
                    nomEditUid_ = n.uid;
                }
                // Persist NOW, like a taught frame and for the same reason: this
                // took two hand-placed marks, and a crash that loses them costs
                // the operator the whole job again.
                session_.SaveAligns();
                FD_INFO("align", "'%s': nominal '%s' local (%.4f, %.4f) mm, "
                                 "centre (%.4f, %.4f) of %d marks, span %.4f",
                        a->name.c_str(), n.name.c_str(), n.lx, n.ly, n.mx, n.my,
                        n.points, n.spanMm);
                SwitchPage(Page::Align);
            }
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(110.0f, 0.0f))) SwitchPage(Page::Align);

    if (!nomErr_.empty()) ImGui::TextColored(kBad, "%s", nomErr_.c_str());
    if (!canSave && both && !haveLive)
        ImGui::TextDisabled("a live frame is required to save");

    ImGui::EndChild();
}

} // namespace firedog
