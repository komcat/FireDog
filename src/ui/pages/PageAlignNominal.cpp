// PageAlignNominal.cpp : the Nominal teach page (a full-page takeover, reached
// only from the Alignment page's nominal pane). Definitions of App members —
// see App.h.
//
// A nominal is a target point on the PART held in the alignment's local frame
// (O1) rather than in machine coordinates. That is the whole point: the frame
// moves with each part, so a nominal taught once against one part names the
// same physical place on every part after it, without anybody re-teaching.
//
// The operator's only job here is to put the crosshair on 1, 2 or 3 positions.
// The nominal is their AVERAGE:
//   1  the crosshair IS the point;
//   2  two opposite corners of something whose centre is the point - a centre
//      has nothing under it to aim at, so the arithmetic finds it;
//   3  three corners of a footprint, or the same feature three times to average
//      the pick noise down.
// The spread shown beside the result is the only thing that can show up a mark
// placed off its feature, so it is shown and never judged.
//
// NOTHING MOVES here unless the operator right-clicks with jog armed. The page
// reads the live image and the device's pose and writes a document field, the
// same argument the crosshair pick window makes. Jogging between marks is
// expected: each mark latches the pose it was taken at.
//
// The image is `DrawMarkPicker` (ui/Widgets.h) — the same mapping the crosshair
// pick window uses, so a pixel means the same thing on every surface.
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
    // below mutates state (the marks, the selections) before it switches, so a
    // refused switch would leave those edits behind on a page the caller never
    // reaches. Same reason the Run deck's right-click checks early.
    if (!CanAccess(Page::AlignNominal)) return;
    nomProcUid_ = a.uid;
    nomEditUid_ = nomUid;
    for (int i = 0; i < kNomMarkMax; ++i) {
        nomHave_[i] = false;
        nomPt_[i]   = AlignPoint{};
    }
    nomActive_  = 0;
    nomZoom_    = 1.5f;
    nomErr_.clear();
    nomPx_ = (cameraW_ > 0) ? static_cast<float>(cameraW_) * 0.5f : 0.0f;
    nomPy_ = (cameraH_ > 0) ? static_cast<float>(cameraH_) * 0.5f : 0.0f;

    const AlignNominal* n = FindAlignNominal(a, nomUid);
    // A re-teach opens on the count it was taught with: that is a fact about the
    // feature, and asking the operator to re-choose it every time is how a
    // three-mark nominal silently becomes a one-mark one. A nominal taught with
    // four on the older page opens on three, the most this page asks for.
    if (n) {
        nomCount_ = n->points;
        if (nomCount_ < 1)           nomCount_ = 1;
        if (nomCount_ > kNomMarkMax) nomCount_ = kNomMarkMax;
    }
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
        static const char* kLbl[kNomMarkMax] = {"pt1", "pt2", "pt3"};
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
        // One mark IS the nominal; the picker draws an average only from two.
        mp.midpointLabel = (nomCount_ >= 2) ? "nominal" : nullptr;
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

    // ---- Right: the marks, the average, and Save ----------------------------
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

    ImGui::SeparatorText("Points");
    ImGui::TextDisabled("Move the crosshair onto each position and set it.");
    ImGui::TextDisabled("The nominal is the AVERAGE of the points.");

    // 1, 2 or 3. Switching down keeps the extra marks in memory but out of the
    // average, so a mis-click on the selector costs nothing; switching up asks
    // for the ones that are missing.
    {
        int want = nomCount_;
        ImGui::TextDisabled("points");
        for (int k = 1; k <= kNomMarkMax; ++k) {
            ImGui::SameLine();
            char lbl[8];
            std::snprintf(lbl, sizeof(lbl), "%d", k);
            ImGui::RadioButton(lbl, &want, k);
        }
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
        ImGui::TextDisabled(nomCount_ == 1   ? "the point itself"
                            : nomCount_ == 2 ? "average of two"
                                             : "average of three");
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
        // The next one still missing.
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
            ImGui::TextDisabled(nomActive_ == i ? "(not set - next)" : "(not set)");
        }
        ImGui::PopID();
    }
    if (nomCount_ > 1) {
        ImGui::TextDisabled("Jog between the points freely: each latches the");
        ImGui::TextDisabled("pose it was taken at.");
    }

    ImGui::SeparatorText("Result");
    bool all = true;
    for (int i = 0; i < nomCount_; ++i) all = all && nomHave_[i];
    if (!all) {
        ImGui::TextDisabled(nomCount_ == 1 ? "set the point" : "set all %d points",
                            nomCount_);
    } else {
        // The same average the solver computes. Written here rather than
        // called out of it because the solver needs a live frame and this
        // panel must read the centre back with or without one.
        double mx = 0.0, my = 0.0;
        for (int i = 0; i < nomCount_; ++i) { mx += nomPt_[i].wx; my += nomPt_[i].wy; }
        mx /= nomCount_;
        my /= nomCount_;
        ImGui::Text("average   %.4f, %.4f mm", mx, my);
        if (nomCount_ == 2) {
            const double dx = nomPt_[1].wx - nomPt_[0].wx;
            const double dy = nomPt_[1].wy - nomPt_[0].wy;
            ImGui::Text("span      %.4f mm", std::sqrt(dx * dx + dy * dy));
        } else if (nomCount_ >= 3) {
            double spread = 0.0;
            for (int i = 0; i < nomCount_; ++i) {
                const double dx = nomPt_[i].wx - mx, dy = nomPt_[i].wy - my;
                spread += std::sqrt(dx * dx + dy * dy);
            }
            spread = 2.0 * spread / nomCount_;
            ImGui::Text("spread    %.4f mm", spread);
            ImGui::TextDisabled("twice the mean distance from the average");
        }
        if (haveLive) {
            double lx = 0.0, ly = 0.0;
            AlignMachineToLocal(frame, mx, my, &lx, &ly);
            ImGui::TextColored(kGood, "local X %.4f   local Y %.4f mm", lx, ly);
            ImGui::TextDisabled("this pair is what gets saved");
        } else {
            ImGui::TextColored(kWarn, "no live frame - nothing to convert through");
        }
    }

    ImGui::Separator();
    ImGui::SetNextItemWidth(200.0f);
    ImGui::InputText("name", nomNameBuf_, sizeof(nomNameBuf_));

    const bool canSave = all && haveLive;
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
                    // A re-teach replaces the POINT and keeps everything else
                    // the nominal owns: its uid, and whatever private poses and
                    // models still hang off it until they are removed.
                    tgt->name     = n.name;
                    tgt->lx       = n.lx;
                    tgt->ly       = n.ly;
                    tgt->p1       = n.p1;
                    tgt->p2       = n.p2;
                    tgt->p3       = n.p3;
                    tgt->p4       = n.p4;
                    tgt->points   = n.points;
                    tgt->mx       = n.mx;
                    tgt->my       = n.my;
                    tgt->spanMm   = n.spanMm;
                    tgt->taughtAt = n.taughtAt;
                    tgt->frameAt  = n.frameAt;
                    tgt->tunedAt.clear();   // a fresh teach supersedes any tune
                    n = *tgt;
                } else {
                    n.uid = GenerateUid();
                    a->nominals.push_back(n);
                    nomEditUid_ = n.uid;
                }
                // Persist NOW, like a taught frame and for the same reason: this
                // took hand-placed marks, and a crash that loses them costs the
                // operator the whole job again.
                session_.SaveAligns();
                FD_INFO("align", "'%s': nominal '%s' local (%.4f, %.4f) mm, "
                                 "average (%.4f, %.4f) of %d point(s), span %.4f",
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
    if (!canSave && all && !haveLive)
        ImGui::TextDisabled("a live frame is required to save");

    ImGui::EndChild();
}

} // namespace firedog
