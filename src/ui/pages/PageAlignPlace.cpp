// PageAlignPlace.cpp : the Placement page (a full-page takeover, reached only
// from a nominal on the Alignment page). Definitions of App members —
// see App.h.
//
// The question it answers: an object is gripped by hex-left (or hex-right), a
// nominal says where it is supposed to end up — how far, and along which of
// THAT device's axes, would it have to move?
//
// It shows every intermediate value, then issues the move behind the shared
// arming flag. The numbers stay on screen because the destination is COMPUTED:
// a mis-taught frame or a mis-marked object produces an ordinary-looking number
// that happens to be wrong, and the only defence is a human reading it.
//
// Two guards beyond the arming flag:
//   * a visible, editable CEILING on the correction (`plcMaxMM_`) — a constant
//     nobody can see would be raised by editing the source;
//   * the marks are CLEARED by a move, because they describe where the object
//     was before it. Leaving them would show a correction already spent, and a
//     second press would apply it twice. Re-measuring is what closes the loop:
//     the residual is the accuracy number, and no transform can promise it.
//
// ---- Why the correction is a DELTA, and only ever a delta -------------------
//
// A nominal is a point in the gantry's world, because that is where the camera
// measures. The hexapod's origin bears no known relation to it — nobody has
// ever measured one against the other — and `AxisTransform`
// (hardware/HardwareConfig.h) is a signed permutation with NO offset, so it can
// carry a direction between the two frames but never a position. An absolute
// machine XY therefore cannot be commanded to hex-left at all; only a
// difference can, and a difference is all this page computes:
//
//     object   = midpoint of two marks           (machine mm, via the camera)
//     nominal  = LocalToMachine(live frame, n)   (machine mm, this part's pose)
//     delta    = nominal - object                (machine mm — a VECTOR)
//     per-axis = transform * delta               (the mover's own axes)
//
// The frame's origin cancels in that subtraction, which is exactly why the
// result is transportable to a device that knows nothing about it. Rotation
// does not cancel — both points are converted through the SAME live frame, so
// the vector already carries the part's actual orientation.
//
// The mapping goes through Workflow::GlobalDeltaToAxes and not JogGlobal:
// JogGlobal collapses a direction to its single dominant axis, which is right
// for a jog button and would silently drop half of a diagonal correction here.
// The move itself is Workflow::MoveRelativeGlobal, which re-derives the same
// per-axis deltas from the same transform — so what the page prints and what
// the machine is told cannot drift apart.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);

} // namespace

// ---- Opening ---------------------------------------------------------------

void App::OpenAlignPlace(const AlignProc& a, const std::string& nomUid)
{
    plcProcUid_ = a.uid;
    plcNomUid_  = nomUid;
    plcHave_[0] = plcHave_[1] = false;
    plcPt_[0]   = AlignPoint{};
    plcPt_[1]   = AlignPoint{};
    plcActive_  = 0;
    plcZoom_    = 1.5f;
    plcLastMove_.clear();
    plcPx_ = (cameraW_ > 0) ? static_cast<float>(cameraW_) * 0.5f : 0.0f;
    plcPy_ = (cameraH_ > 0) ? static_cast<float>(cameraH_) * 0.5f : 0.0f;

    // Default the mover to the first device that is NOT the one carrying the
    // camera: moving that one moves the view, not the object. Only a default —
    // the combo can pick anything, including (with a warning) that device.
    if (plcDevice_.empty()) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string n = hardware_.MotionName(i);
            if (n != a.device) { plcDevice_ = n; break; }
        }
    }

    SwitchPage(Page::AlignPlace);   // every page switch routes through here
}

// ---- The page --------------------------------------------------------------

void App::DrawAlignPlacePanel()
{
    std::vector<AlignProc>& list = session_.Aligns();
    AlignProc* a = FindAlignProc(list, plcProcUid_);
    if (!a) {
        ImGui::TextDisabled("The alignment procedure this page was opened for is gone.");
        if (ImGui::Button("Back to Alignment")) SwitchPage(Page::Align);
        return;
    }

    // The LIVE frame, for the reason §5b of the design doc gives: both the
    // object measurement and the nominal have to be expressed against the part
    // that is on the machine now. A correction computed half from this part and
    // half from a record would be wrong by the difference and look right.
    AlignResult frame;
    const bool haveLive = workflow_.LiveFrame(a->uid, &frame);

    // The pose the marks are measured against: the procedure's device, because
    // that is the one carrying the camera.
    IMotionController* cam = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a->device) { cam = hardware_.MotionDevice(i); break; }
    }
    double gx = 0.0, gy = 0.0;
    bool   havePose = false;
    if (cam && cam->IsConnected()) {
        cam->RequestFastPoll();
        const std::map<std::string, double> pos = cam->Positions();
        const auto ix = pos.find("X"), iy = pos.find("Y");
        if (ix != pos.end() && iy != pos.end()) {
            gx = ix->second;
            gy = iy->second;
            havePose = true;
        }
    }

    // The mover: the device actually holding the object.
    int moverIdx = -1;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == plcDevice_) { moverIdx = static_cast<int>(i); break; }
    }
    IMotionController* mover =
        moverIdx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(moverIdx)) : nullptr;

    UpdateCameraTexture();
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
    const bool canMark = haveImg && havePose;

    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  rightW = 400.0f;
    const float  imgW   = avail.x - rightW - ImGui::GetStyle().ItemSpacing.x;

    // ---- Left: the live image ----------------------------------------------
    ImGui::BeginChild("apImg", ImVec2(imgW, avail.y), ImGuiChildFlags_Borders);
    {
        const CameraMark marks[2] = {
            {plcPt_[0].wx, plcPt_[0].wy, plcHave_[0], "pt1"},
            {plcPt_[1].wx, plcPt_[1].wy, plcHave_[1], "pt2"},
        };
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
        mp.markCount     = 2;
        mp.midpointLabel = "object";
        mp.px            = &plcPx_;
        mp.py            = &plcPy_;
        mp.zoom          = &plcZoom_;
        DrawMarkPicker("##plcInput", ImGui::GetContentRegionAvail(), mp);
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: target, measurement, correction -----------------------------
    ImGui::BeginChild("apCtl", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);

    ImGui::TextUnformatted(a->name.c_str());
    ImGui::TextDisabled("place a gripped object on a taught nominal");
    ImGui::Separator();

    if (haveLive) {
        ImGui::TextColored(kGood, "LIVE frame  taught %s", frame.capturedAt.c_str());
    } else {
        ImGui::TextColored(kBad, "NO LIVE FRAME");
        ImGui::TextWrapped("Both the target and the measurement are expressed through the "
                           "frame, so it has to be the one taught for the part on the "
                           "machine now. Run the procedure on the Alignment page first.");
    }

    // ---- Target -------------------------------------------------------------
    ImGui::SeparatorText("Target (nominal)");
    const AlignNominal* nom = FindAlignNominal(*a, plcNomUid_);
    if (a->nominals.empty()) {
        ImGui::TextColored(kWarn, "this procedure has no nominals taught");
    } else {
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("nominal", nom ? nom->name.c_str() : "(pick)")) {
            for (const AlignNominal& n : a->nominals) {
                if (ImGui::Selectable(n.name.c_str(), nom && n.uid == nom->uid))
                    plcNomUid_ = n.uid;
            }
            ImGui::EndCombo();
        }
    }
    double nomMx = 0.0, nomMy = 0.0;
    if (nom) {
        ImGui::Text("local     %.4f, %.4f mm", nom->lx, nom->ly);
        if (haveLive) {
            // Where that local point sits on THIS part. Recomputed every frame
            // from the live frame — this is the number that moves with the part.
            AlignLocalToMachine(frame, nom->lx, nom->ly, &nomMx, &nomMy);
            ImGui::Text("machine   %.4f, %.4f mm", nomMx, nomMy);
        }
    }

    // ---- The mover ----------------------------------------------------------
    ImGui::SeparatorText("Mover (holds the object)");
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo("device", plcDevice_.empty() ? "(pick)" : plcDevice_.c_str())) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string n = hardware_.MotionName(i);
            if (ImGui::Selectable(n.c_str(), n == plcDevice_)) plcDevice_ = n;
        }
        ImGui::EndCombo();
    }
    if (!plcDevice_.empty() && plcDevice_ == a->device)
        ImGui::TextColored(kWarn, "that is the device carrying the camera - moving it\n"
                                  "moves the view, not the object");
    if (mover) {
        mover->RequestFastPoll();
        if (!mover->IsConnected()) {
            ImGui::TextColored(kBad, "%s not connected", plcDevice_.c_str());
        } else {
            if (transformCfg_.find(plcDevice_) == transformCfg_.end())
                ImGui::TextColored(kWarn, "no axis transform configured - assuming identity");
            // Live pose, always — a move is issued and returns immediately (no
            // blocking wait on the UI thread), so this is where an operator
            // watches it arrive before re-measuring. Without it the from/to
            // readout vanishes with the marks and there is nothing to watch.
            const std::map<std::string, double> mp = mover->Positions();
            std::string cur;
            for (const auto& [axis, v] : mp) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%s %.4f", cur.empty() ? "" : "  ",
                              axis.c_str(), v);
                cur += buf;
            }
            ImGui::TextDisabled("%s", cur.c_str());
        }
    } else if (!plcDevice_.empty()) {
        ImGui::TextColored(kBad, "'%s' is not in the fleet", plcDevice_.c_str());
    }

    // ---- Measurement --------------------------------------------------------
    ImGui::SeparatorText("Object (measure it now)");
    if (havePose)
        ImGui::TextDisabled("%s  X %.4f  Y %.4f mm", a->device.c_str(), gx, gy);
    else
        ImGui::TextColored(kBad, "%s not connected (or reports no X/Y)", a->device.c_str());
    if (!haveImg) ImGui::TextColored(kBad, "no camera feed");
    ImGui::TextDisabled("Mark two features whose midpoint IS the object's");
    ImGui::TextDisabled("reference point - the same pair the nominal used.");

    const auto capture = [&](int i) {
        AlignPoint p;
        p.gx = gx;
        p.gy = gy;
        p.px = plcPx_;
        p.py = plcPy_;
        p.wx = gx + static_cast<double>(plcPx_ - static_cast<float>(cameraW_) * 0.5f) *
                        static_cast<double>(pixelToMmX_);
        p.wy = gy + static_cast<double>(plcPy_ - static_cast<float>(cameraH_) * 0.5f) *
                        static_cast<double>(pixelToMmY_);
        plcPt_[i]   = p;
        plcHave_[i] = true;
        plcActive_  = 1 - i;
    };

    for (int i = 0; i < 2; ++i) {
        ImGui::PushID(i);
        char btn[32];
        std::snprintf(btn, sizeof(btn), "Set pt%d here", i + 1);
        ImGui::BeginDisabled(!canMark);
        if (ImGui::Button(btn, ImVec2(120.0f, 0.0f))) capture(i);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (plcHave_[i]) {
            ImGui::Text("%.4f, %.4f", plcPt_[i].wx, plcPt_[i].wy);
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear")) { plcHave_[i] = false; plcActive_ = i; }
        } else {
            ImGui::TextDisabled(plcActive_ == i ? "(not marked - next)" : "(not marked)");
        }
        ImGui::PopID();
    }

    // ---- The correction -----------------------------------------------------
    ImGui::SeparatorText("Correction");
    // Above the gate on purpose: the marks are cleared by a move, so this is
    // exactly the moment the operator wants to see what was just issued —
    // it is the number the re-measure will be compared against.
    if (!plcLastMove_.empty())
        ImGui::TextWrapped("last: %s", plcLastMove_.c_str());
    const bool both = plcHave_[0] && plcHave_[1];
    if (!both || !nom || !haveLive) {
        ImGui::TextDisabled("needs a live frame, a nominal, and both marks");
        ImGui::EndChild();
        return;
    }

    const double objMx = (plcPt_[0].wx + plcPt_[1].wx) * 0.5;
    const double objMy = (plcPt_[0].wy + plcPt_[1].wy) * 0.5;
    double objLx = 0.0, objLy = 0.0;
    AlignMachineToLocal(frame, objMx, objMy, &objLx, &objLy);

    ImGui::Text("object   machine  %.4f, %.4f mm", objMx, objMy);
    ImGui::Text("         local    %.4f, %.4f mm", objLx, objLy);

    // The delta. Taken in MACHINE mm as nominal - object: the frame's origin
    // cancels, which is what makes the result meaningful to a device that has
    // never been related to that origin. Shown in local too, because that is
    // the number an operator can compare against the drawing.
    const double dMx = nomMx - objMx;
    const double dMy = nomMy - objMy;
    const double dLx = nom->lx - objLx;
    const double dLy = nom->ly - objLy;
    ImGui::Spacing();
    ImGui::TextColored(kGood, "delta local    %+.4f, %+.4f mm", dLx, dLy);
    ImGui::Text("delta machine  %+.4f, %+.4f mm", dMx, dMy);
    ImGui::Text("distance       %.4f mm", std::sqrt(dMx * dMx + dMy * dMy));

    // ...and the same vector expressed in the mover's own axes. Layer 2 owns
    // this arithmetic (Workflow::GlobalDeltaToAxes) — it is the same call the
    // real move will make, so what the page prints and what it would eventually
    // command cannot drift apart.
    ImGui::Spacing();
    std::vector<std::pair<std::string, double>> axes;
    if (moverIdx < 0 || !workflow_.GlobalDeltaToAxes(static_cast<std::size_t>(moverIdx),
                                                     dMx, dMy, 0.0, &axes)) {
        ImGui::TextColored(kBad, "pick a mover device to map the correction onto its axes");
        ImGui::EndChild();
        return;
    }
    ImGui::SeparatorText("Relative move it would take");
    std::string line;
    for (const auto& [axis, d] : axes) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%s%s %+.4f", line.empty() ? "" : "   ",
                      axis.c_str(), d);
        line += buf;
    }
    ImGui::TextColored(kGood, "%s   %s", plcDevice_.c_str(), line.c_str());
    ImGui::TextDisabled("(Z is 0: the frame is 2-D)");

    // The resulting absolute pose, so travel limits can be eyeballed before
    // anybody wires the move.
    if (mover && mover->IsConnected()) {
        const std::map<std::string, double> pos = mover->Positions();
        std::string now, tgt;
        for (const auto& [axis, d] : axes) {
            const auto it = pos.find(axis);
            if (it == pos.end()) continue;
            char b1[48], b2[48];
            std::snprintf(b1, sizeof(b1), "%s%s %.4f", now.empty() ? "" : "   ",
                          axis.c_str(), it->second);
            std::snprintf(b2, sizeof(b2), "%s%s %.4f", tgt.empty() ? "" : "   ",
                          axis.c_str(), it->second + d);
            now += b1;
            tgt += b2;
        }
        ImGui::TextDisabled("from   %s", now.c_str());
        ImGui::TextDisabled("to     %s", tgt.c_str());
    }

    // ---- The move -----------------------------------------------------------
    // Behind the shared arming flag, like the Alignment page's Move here and for
    // the same reason: this destination is COMPUTED, so a mis-taught frame or a
    // mis-marked object sends the stage somewhere nobody chose. Same risk class
    // as click-to-move; a station card's Go, which drives to a pose a human
    // taught, is not.
    ImGui::Separator();
    const double dist = std::sqrt(dMx * dMx + dMy * dMy);
    ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::InputFloat("max mm", &plcMaxMM_, 0.0f, 0.0f, "%.2f");
    if (plcMaxMM_ < 0.0f) plcMaxMM_ = 0.0f;

    const bool tooFar    = dist > static_cast<double>(plcMaxMM_);
    const bool canMove   = jogArmed_ && !tooFar && !orch_.Busy() &&
                           mover && mover->IsConnected();
    ImGui::BeginDisabled(!canMove);
    if (ImGui::Button("Move object to nominal", ImVec2(200.0f, 0.0f))) {
        // Layer 2 owns the move: it re-derives the per-axis deltas from the same
        // transform, adds them to the LIVE pose and issues one coordinated
        // command. The page hands over the machine-frame vector and nothing else.
        if (workflow_.MoveRelativeGlobal(static_cast<std::size_t>(moverIdx), dMx, dMy, 0.0)) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                          "issued %s: %s   (was %.4f mm off)",
                          plcDevice_.c_str(), line.c_str(), dist);
            plcLastMove_ = buf;
            // Drop the marks. They describe where the object was BEFORE the
            // move, so leaving them would show a correction that is already
            // spent — and a second press would apply it twice. Clearing forces
            // the re-measure that turns this into a closed loop.
            plcHave_[0] = plcHave_[1] = false;
            plcActive_  = 0;
        } else {
            plcLastMove_ = "move rejected - see the log";
        }
    }
    ImGui::EndDisabled();

    if (!mover || !mover->IsConnected())
        ImGui::TextDisabled("%s not connected", plcDevice_.c_str());
    else if (tooFar)
        ImGui::TextColored(kBad, "%.4f mm exceeds the %.2f mm ceiling - check the frame\n"
                                 "and the marks, then raise it deliberately",
                           dist, static_cast<double>(plcMaxMM_));
    else if (!jogArmed_)
        ImGui::TextDisabled("arm first");
    else if (orch_.Busy())
        ImGui::TextDisabled("a run is active");
    else
        ImGui::TextDisabled("marks clear after the move - measure again to see the residual");

    ImGui::EndChild();
}

} // namespace firedog
