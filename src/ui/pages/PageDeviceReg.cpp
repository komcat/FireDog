// PageDeviceReg.cpp : the Registration page (View menu, F8) — where a device's
// tool tip is in the camera device's space, taught in three steps.
// Definitions of App members — see App.h.
//
// The teach is the PAGE: the corner picks live on this page's image, and a
// second way to create a registration is a second answer to where the tool is.
// Nothing in it moves the registered device - the operator jogs the CAMERA
// onto the tool wherever it stands, and the device's pose is read, not set.
//
// What a recipe gets is the CONSUMING end: `device_reg` to read the record,
// `move_to_point` to move through it, `device_reg_check_run` to check it.
// Those are in command/CommandsDeviceReg.cpp.
//
// The three steps:
//   1  CAMERA ON THE TOOL  the operator jogs the camera over the tool and
//                          focuses. The focused Z IS the tip's height.
//   2  CORNERS             marks on the live image, in machine mm, each with
//                          the camera pose it was taken at. The first one also
//                          latches the registered device's pose (X Y Z U V W),
//                          whatever it is: the operator never has to know it.
//   3  COMPUTE             centre of the marks + the focused Z = the tip at the
//                          latched pose, and t = tip - R*h. Refused if the
//                          device moved after the first mark.
//
// Where it is taught buys no accuracy by itself: the typed rotation's error is
// zero at the taught pose and grows with distance from it, so the best place to
// teach is where the device works.
//
// The right-hand column is the device pair over three tabs: Registration (the
// record, the footprint, the fleet), Capture tool points (the three steps),
// Verify and test (Move tool to camera, Move camera to tool). Each step is
// headed "Step n ..." and turns green when the RECORD answers it, so a
// registration taught months ago opens as green steps - this is a one-time
// setup, and the page must say so rather than look untaught. A tab goes green
// when everything on it is done; Verify only on a PASSED check.
//
// Nothing on this page actuates on its own. Every move is the operator's own
// jog or one of the two Verify buttons - behind the jog arm, and behind a
// confirm popup for anything over kRegConfirmMm.
#include "App.h"

#include "imgui.h"

#include "core/AlignLighting.h"         // AlignLightingSummary — the camera view
#include "core/DeviceReg.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "ui/Widgets.h"
#include "ui/RoiFilter.h"               // the image toolbar's ROI + binary view

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.55f, 0.85f, 0.55f, 1.0f);

void SetBuf(char* buf, std::size_t n, const std::string& text)
{
    std::snprintf(buf, n, "%s", text.c_str());
}

// Fleet index for a configured name. Layer 2 owns every move, but it takes an
// index, and the page's combos hold names — so this is the one translation, in
// one place, rather than three loops among the buttons.
int MotionIndexOf(const HardwareManager& hw, const std::string& name)
{
    for (std::size_t i = 0; i < hw.MotionCount(); ++i)
        if (hw.MotionName(i) == name) return static_cast<int>(i);
    return -1;
}

void DeviceCombo(const char* label, char* buf, std::size_t n,
                 const HardwareManager& hw, const char* tip)
{
    ImGui::SetNextItemWidth(180.0f);
    if (ImGui::BeginCombo(label, buf)) {
        for (std::size_t i = 0; i < hw.MotionCount(); ++i) {
            const std::string name = hw.MotionName(i);
            if (ImGui::Selectable(name.c_str(), name == buf))
                SetBuf(buf, n, name);
        }
        ImGui::EndCombo();
    }
    if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
}

} // namespace

// ---- The teach steps --------------------------------------------------------

// Step 2, one corner. The first mark of a set also latches the registered
// device's live pose: the tip is computed at wherever the device stood, so it
// is read here rather than set by anybody. A corner taken without that pose is
// refused - a centre with no pose to hang it on is a point, not a tip.
void App::CaptureDeviceRegCorner(double gx, double gy, double gz, bool haveZ)
{
    regTeachMsg_.clear();
    if (!haveZ) {
        regTeachMsg_ = std::string("'") + regRef_ + "' reports no Z - the focused Z is "
                       "the tip's height";
        return;
    }

    bool anyHeld = false;
    for (int i = 0; i < kRegMarkMax; ++i) anyHeld = anyHeld || regMarkHave_[i];
    // A set started against another pair is not this pair's set.
    if (anyHeld && (regMarkDev_ != regDevice_ || regMarkRef_ != regRef_)) {
        ClearDeviceRegMarks();
        anyHeld = false;
    }
    if (!anyHeld || !regMarkPoseHave_) {
        std::map<std::string, double> live;
        if (!workflow_.DevicePose(regDevice_, live)) {
            regTeachMsg_ = std::string("'") + regDevice_ + "' is not connected - its "
                           "pose is half of the measurement";
            return;
        }
        const auto ix = live.find("X"), iy = live.find("Y"), iz = live.find("Z");
        if (ix == live.end() || iy == live.end() || iz == live.end()) {
            regTeachMsg_ = std::string("'") + regDevice_ + "' does not report X/Y/Z";
            return;
        }
        regMarkPose_ = RegVec3{ix->second, iy->second, iz->second};
        const auto iu = live.find("U"), iv = live.find("V"), iw = live.find("W");
        regMarkHasRot_ = iu != live.end() && iv != live.end() && iw != live.end();
        regMarkRot_    = regMarkHasRot_ ? RegVec3{iu->second, iv->second, iw->second}
                                        : RegVec3{};
        regMarkPoseHave_ = true;
        regMarkDev_      = regDevice_;
        regMarkRef_      = regRef_;
    }

    AlignPoint p;
    p.gx   = gx;
    p.gy   = gy;
    p.gz   = gz;
    p.hasZ = true;
    p.px   = regPx_;
    p.py   = regPy_;
    // The crosshair's offset from the image centre, in mm, added to the pose it
    // was taken at — so a mark keeps meaning the same place when the operator
    // jogs between corners.
    p.wx = gx + static_cast<double>(regPx_ - static_cast<float>(cameraW_) * 0.5f) *
                    static_cast<double>(pixelToMmX_);
    p.wy = gy + static_cast<double>(regPy_ - static_cast<float>(cameraH_) * 0.5f) *
                    static_cast<double>(pixelToMmY_);
    regMark_[regMarkActive_]     = p;
    regMarkHave_[regMarkActive_] = true;
    if (regMarkActive_ + 1 < regMarkCount_) ++regMarkActive_;
}

void App::ClearDeviceRegMarks()
{
    for (int i = 0; i < kRegMarkMax; ++i) regMarkHave_[i] = false;
    regMarkActive_   = 0;
    regMarkDrag_     = -1;
    regMarkPoseHave_ = false;
}

// Step 3. The centre of the marks is the tip in XY; the camera's focused Z is
// the tip in Z; the device pose latched at the first mark is where it stood.
// Everything before this was capture; this is the only step that computes, and
// the only one that can refuse.
void App::CompleteDeviceRegTip()
{
    regTeachMsg_.clear();
    DeviceReg r = workflow_.DeviceRegFor(regDevice_);
    r.device    = regDevice_;
    r.refDevice = regRef_;

    int    n = 0;
    double cx = 0.0, cy = 0.0, zLo = 0.0, zHi = 0.0, zSum = 0.0;
    for (int i = 0; i < regMarkCount_; ++i) {
        if (!regMarkHave_[i]) continue;
        const AlignPoint& m = regMark_[i];
        cx   += m.wx;
        cy   += m.wy;
        zSum += m.gz;
        zLo   = n ? std::min(zLo, m.gz) : m.gz;
        zHi   = n ? std::max(zHi, m.gz) : m.gz;
        ++n;
    }
    if (n < 2) {
        regTeachMsg_ = "mark at least two corners of the tool (step 2)";
        return;
    }
    if (n < regMarkCount_) {
        regTeachMsg_ = "mark every corner, or set the count to what you actually took";
        return;
    }
    if (!regMarkPoseHave_ || regMarkDev_ != regDevice_ || regMarkRef_ != regRef_) {
        regTeachMsg_ = "these marks were taken for another device pair - Clear and mark "
                       "again";
        return;
    }
    // One focus for the whole set. A small refocus between corners is the same
    // surface and averages out; a large one means a corner was marked on
    // something else, and its Z would be averaged into the tip's height.
    constexpr double kZSpreadMm = 0.05;
    if (zHi - zLo > kZSpreadMm) {
        char b[200];
        std::snprintf(b, sizeof(b),
                      "the camera Z changed by %.3f mm between corners (over %.2f) - "
                      "focus once on the tip, then Clear and mark every corner",
                      zHi - zLo, kZSpreadMm);
        regTeachMsg_ = b;
        return;
    }

    // The marks were taken against the tool as it stood at the first mark, and
    // the tip is computed as if it still stands there. A device moved in
    // between would register the tool somewhere it was never measured.
    {
        std::map<std::string, double> live;
        if (!workflow_.DevicePose(regDevice_, live)) {
            regTeachMsg_ = std::string("'") + regDevice_ + "' is not connected";
            return;
        }
        const auto ix = live.find("X"), iy = live.find("Y"), iz = live.find("Z");
        if (ix == live.end() || iy == live.end() || iz == live.end()) {
            regTeachMsg_ = std::string("'") + regDevice_ + "' does not report X/Y/Z";
            return;
        }
        const double dx = ix->second - regMarkPose_.x, dy = iy->second - regMarkPose_.y,
                     dz = iz->second - regMarkPose_.z;
        const double off = std::sqrt(dx * dx + dy * dy + dz * dz);
        bool rotOff = false;
        if (regMarkHasRot_) {
            const auto iu = live.find("U"), iv = live.find("V"), iw = live.find("W");
            if (iu != live.end() && iv != live.end() && iw != live.end())
                rotOff = std::fabs(iu->second - regMarkRot_.x) > 0.01 ||
                         std::fabs(iv->second - regMarkRot_.y) > 0.01 ||
                         std::fabs(iw->second - regMarkRot_.z) > 0.01;
        }
        if (off > 0.01 || rotOff) {
            char b[200];
            std::snprintf(b, sizeof(b),
                          "'%s' has moved since the first mark (%.4f mm%s) - Clear and "
                          "mark again with it standing still",
                          regDevice_, off, rotOff ? ", rotations changed" : "");
            regTeachMsg_ = b;
            return;
        }
    }

    cx /= static_cast<double>(n);
    cy /= static_cast<double>(n);
    const double cz = zSum / static_cast<double>(n);
    // No width check. An earlier version measured the mark's width and refused
    // the teach when it disagreed with a typed tool width; it refused good
    // picks. The picks are the operator's judgement, and the footprint box on
    // the image is where a bad one becomes visible.

    // The rotation. It is the permutation composed out of the axis transforms,
    // adopted explicitly here so `rotSource` records what it is: nothing measured
    // it, and every consumer has to be able to tell.
    if (r.rotSource.empty()) {
        RegVec3 cols[3];
        bool    seeded = true;
        if (!workflow_.DeviceRegAimColumns(regDevice_, regRef_, cols, &seeded)) {
            regTeachMsg_ = std::string("no motion device named '") + regDevice_ +
                           "' or '" + regRef_ + "'";
            return;
        }
        if (const std::string why =
                DeviceRegSetRotation(r, cols, "axis-transforms"); !why.empty()) {
            regTeachMsg_ = why;
            return;
        }
    }

    const RegVec3 h       = regMarkPose_;
    const bool    hadTip  = r.valid;
    const RegVec3 before  = DeviceRegPredict(r, h);
    const RegVec3 tip{cx, cy, cz};
    if (const std::string why = DeviceRegSetTip(r, h, tip, n); !why.empty()) {
        regTeachMsg_ = why;
        return;
    }
    // Whatever U/V/W the device stood at is the rotation this record describes
    // (refused past kDeviceRegRotTolDeg unless the pivot is at the tip).
    r.taughtRot = regMarkRot_;
    r.hasRot    = regMarkHasRot_;
    // Where the camera looked from, kept as evidence: the first mark's pose
    // with the set's focused Z.
    r.tipView    = RegVec3{regMark_[0].gx, regMark_[0].gy, cz};
    r.hasTipView = true;
    workflow_.QueueDeviceReg(r);

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "tip taught from %d picks with '%s' at X %.3f Y %.3f Z %.3f. "
                  "UNVERIFIED until the check runs.",
                  n, regDevice_, h.x, h.y, h.z);
    regTeachMsg_ = buf;
    if (hadTip) {
        const double moved = std::sqrt((tip.x - before.x) * (tip.x - before.x) +
                                       (tip.y - before.y) * (tip.y - before.y) +
                                       (tip.z - before.z) * (tip.z - before.z));
        FD_INFO("devreg", "'%s' tip taught at pose (%.4f, %.4f, %.4f) U %.4f V %.4f "
                          "W %.4f: tip (%.4f, %.4f, %.4f), t=(%.4f, %.4f, %.4f), %d "
                          "picks, Z spread %.4f mm; %.4f mm from where the old record "
                          "put it",
                regDevice_, h.x, h.y, h.z, regMarkRot_.x, regMarkRot_.y,
                regMarkRot_.z, tip.x, tip.y, tip.z, r.t.x, r.t.y, r.t.z, n, zHi - zLo,
                moved);
    } else {
        FD_INFO("devreg", "'%s' tip taught at pose (%.4f, %.4f, %.4f) U %.4f V %.4f "
                          "W %.4f: tip (%.4f, %.4f, %.4f), t=(%.4f, %.4f, %.4f), %d "
                          "picks, Z spread %.4f mm",
                regDevice_, h.x, h.y, h.z, regMarkRot_.x, regMarkRot_.y, regMarkRot_.z,
                tip.x, tip.y, tip.z, r.t.x, r.t.y, r.t.z, n, zHi - zLo);
    }

    ClearDeviceRegMarks();
}

// ---- The camera view the tool is marked under --------------------------------
//
// HARD-CODED to the one camera and its light group, by request: this page is
// always looked at through main_camera, and an operator re-teaching after a
// tool change wants the picture they marked under last time, not whatever the
// last run left behind. Kept in UiSettings; applied on an edit or by Apply,
// never on its own.
void App::DrawDeviceRegCameraView()
{
    const bool busy = orch_.Busy();
    ImGui::SeparatorText("Camera view - main_camera");

    const ConnState cst = hardware_.CameraState();
    PylonCamera&    cam = hardware_.Camera();
    const bool camOk = cst == ConnState::Connected;
    // Seed from the camera once, so the slider starts where the picture is.
    // Not saved until edited: a value nobody chose is not a setting.
    if (regExposureUs_ <= 0.0 && camOk) {
        const double cur = cam.ExposureUs();
        if (cur > 0.0) regExposureUs_ = cur;
    }
    double lo = 0.0, hi = 0.0;
    if (camOk) cam.ExposureRange(lo, hi);
    if (lo <= 0.0 || hi <= lo) { lo = 20.0; hi = 100000.0; }

    bool save = false;
    ImGui::BeginDisabled(!camOk || busy);
    float us = static_cast<float>(regExposureUs_);
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::SliderFloat("exposure us", &us, static_cast<float>(lo), static_cast<float>(hi),
                           "%.0f", ImGuiSliderFlags_Logarithmic))
        regExposureUs_ = static_cast<double>(us);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        workflow_.SetCameraExposure(regExposureUs_);
        save = true;
    }
    ImGui::EndDisabled();
    if (!camOk) ImGui::TextDisabled("connect main_camera to set its exposure");

    // The light: the main camera's group only. A click both keeps the setting
    // and lights it (through the poll thread, never a blocking set here).
    const std::vector<std::string> names = workflow_.LightNames();
    bool known = false;
    for (const std::string& nm : names) known = known || nm == kRegLightName;
    if (!known)
        ImGui::TextColored(kWarn, "no light named '%s' in hardware.json", kRegLightName);
    const bool lightOk = hardware_.LightState() == ConnState::Connected && !busy && known;
    regLight_.channel = kRegLightName;
    bool applyLight = false;
    if (LightCalloutPicker("reglight", regLight_, {std::string(kRegLightName)}, lightOk,
                           &applyLight))
        save = true;
    if (applyLight) ApplyLightingNow(regLight_);

    // Apply both: the way back to the saved picture after a run, or a restart,
    // changed the camera's exposure or the light.
    ImGui::BeginDisabled(busy || (!camOk && !lightOk));
    if (ImGui::Button("Apply saved view", ImVec2(180.0f, 0.0f))) {
        if (camOk && regExposureUs_ > 0.0) workflow_.SetCameraExposure(regExposureUs_);
        if (lightOk) ApplyLightingNow(regLight_);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Set main_camera to %.0f us and %s.\nKept for the next time "
                          "this page is used.", regExposureUs_,
                          AlignLightingSummary(regLight_).c_str());
    if (save) SaveUiSettings();
}

// ---- The page ---------------------------------------------------------------

void App::DrawDeviceRegPanel()
{
    const DeviceReg reg = workflow_.DeviceRegFor(regDevice_);
    const bool busy     = orch_.Busy();
    const bool sameDev  = std::string(regDevice_) == std::string(regRef_);

    // The reference device's live pose — what a mark is projected through, and
    // what a captured corner latches (its Z is the focused height).
    double gx = 0.0, gy = 0.0, gz = 0.0;
    bool   havePose = false, haveGz = false;
    {
        std::map<std::string, double> pose;
        if (workflow_.DevicePose(regRef_, pose)) {
            const auto ix = pose.find("X"), iy = pose.find("Y"), iz = pose.find("Z");
            if (ix != pose.end() && iy != pose.end()) {
                gx = ix->second;
                gy = iy->second;
                havePose = true;
            }
            if (iz != pose.end()) {
                gz     = iz->second;
                haveGz = true;
            }
        }
    }

    UpdateCameraTexture();
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;

    // Driving to a pose the operator taught is still driving: gated on the jog
    // arm like every other surface that moves something, and issued by layer 2.
    const auto driveAxes = [&](const char* devName,
                               const std::vector<std::pair<std::string, double>>& targets) {
        if (!jogArmed_) {
            regTeachMsg_ = "arm jog first - this drives a device";
            return;
        }
        const int idx = MotionIndexOf(hardware_, devName);
        if (idx < 0) {
            regTeachMsg_ = std::string("'") + devName + "' is not in the motion fleet";
            return;
        }
        regTeachMsg_.clear();
        // A refused move must SAY so: layer 2 returns false for a device that
        // is not connected, an axis it does not have, or a safety block, and a
        // button that simply does nothing reads as a button that is broken.
        if (!workflow_.MoveToPosition(static_cast<std::size_t>(idx), targets)) {
            const IMotionController* m = hardware_.MotionDevice(static_cast<std::size_t>(idx));
            const std::string why = m ? m->LastError() : std::string();
            regTeachMsg_ = std::string("'") + devName + "' refused the move" +
                           (why.empty() ? std::string(" - is it connected?") : ": " + why);
            FD_WARN("devreg", "%s", regTeachMsg_.c_str());
        }
    };

    // The control column is draggable: its steps carry long explanatory
    // text and the operator's font size moves all of it, so no constant width
    // reads the same on every station. Clamped every frame, not just on the
    // drag: the window is resizable too, and a width saved on a wide monitor
    // must not push the image off a narrow one.
    constexpr float kSplitW = 6.0f;
    constexpr float kMinImgW = 240.0f;
    constexpr float kMinCtlW = 260.0f;

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (regCtlW_ < kMinCtlW) regCtlW_ = kMinCtlW;
    const float maxCtlW = avail.x - kSplitW - kMinImgW;
    if (regCtlW_ > maxCtlW) regCtlW_ = fmaxf(kMinCtlW, maxCtlW);
    // Floored: the two minimums together can exceed a very narrow window, and
    // ImGui reads a negative width as "available minus this", which would turn
    // a cramped layout into a scrambled one.
    const float rightW = regCtlW_;
    const float imgW   = fmaxf(40.0f, avail.x - rightW - kSplitW);

    // ---- Left: the live image, and the corner marks on it --------------------
    ImGui::BeginChild("drImg", ImVec2(imgW, avail.y), ImGuiChildFlags_Borders);
    {
        // The live ROI filter (ui/RoiFilter): its toolbar above the picture,
        // then the overlay it draws over it. Display only.
        const RoiFilterFrame frame = MainCameraFrame();
        if (DrawRoiFilterToolbar("drRoi", regFilter_, regFilterSt_, frame, regPx_, regPy_))
            SaveUiSettings();
        const RoiOverlay roi = UpdateRoiFilter(host_, regFilter_, regFilterSt_, frame,
                                               regPx_, regPy_);

        CameraMark marks[kRegMarkMax];
        // Numbered like the corner buttons, so "3" on the image is the "3" below.
        static const char* kLbl[kRegMarkMax] = {"1", "2", "3", "4"};
        for (int i = 0; i < regMarkCount_; ++i) {
            marks[i].wx    = regMark_[i].wx;
            marks[i].wy    = regMark_[i].wy;
            marks[i].have  = regMarkHave_[i];
            marks[i].label = kLbl[i];
        }
        MarkPicker mp;
        mp.tex        = cameraTex_;
        mp.imgW       = cameraW_;
        mp.imgH       = cameraH_;
        mp.pixelToMmX = pixelToMmX_;
        mp.pixelToMmY = pixelToMmY_;
        mp.havePose   = havePose;
        mp.gx         = gx;
        mp.gy         = gy;
        mp.marks      = marks;
        mp.markCount  = regMarkCount_;
        // Reference lines across the picture through every corner, and corners
        // that can be dragged into place after they were captured.
        mp.markLines  = true;
        mp.markDrag   = &regMarkDrag_;
        // No midpoint label: that draws the point between marks 0 and 1, which
        // means something for a two-mark nominal and nothing for four corners.
        // The centre this page computes is the average of all of them.
        EnsureMarkIcons();
        mp.icons[0]   = markIcons_[0];
        mp.icons[1]   = markIcons_[1];
        mp.activeMark = regMarkActive_;
        mp.px         = &regPx_;
        mp.py         = &regPy_;
        mp.zoom       = &regZoom_;
        // The tool is a coarse target: half the fit shows the whole frame to
        // find it, three times is enough to put a corner on its edge.
        mp.zoomMin    = 0.5f;
        mp.zoomMax    = 3.0f;

        mp.roi        = roi;

        // The registered tip, projected through the same pose as the marks,
        // with the tool's footprint drawn around it. The box is CENTRED on the
        // tip by the widget, so a corner drag can only change its size - and
        // that size is the record's, seeded here every frame the box is not
        // being dragged and written back when a drag ends. A record with no
        // footprint draws no box: the size is typed under Registration first.
        RegVec3     tipNow;
        std::string tipWhy;
        if (workflow_.DeviceRegTip(regDevice_, tipNow, &tipWhy)) {
            mp.toolHave  = true;
            mp.toolWx    = tipNow.x;
            mp.toolWy    = tipNow.y;
            mp.toolLabel = "tip";
            if (!regBoxDragging_) {
                regBoxW_ = static_cast<float>(reg.toolWmm);
                regBoxH_ = static_cast<float>(reg.toolLmm);
            }
            mp.boxWmm      = &regBoxW_;
            mp.boxHmm      = &regBoxH_;
            mp.boxDragging = &regBoxDragging_;
            // The jaws turn with the device's U, so the box does. Live, like
            // the tip it is centred on: a footprint drawn at the taught angle
            // would describe the gripper as it was, not as it is.
            std::map<std::string, double> devPose;
            if (workflow_.DevicePose(regDevice_, devPose))
                if (const auto iu = devPose.find("U"); iu != devPose.end())
                    mp.boxDeg = static_cast<float>(iu->second);
        }
        const bool boxWasDragging = regBoxDragging_;

        const MarkPickerResult r =
            DrawMarkPicker("##drInput", ImGui::GetContentRegionAvail(), mp);

        // A dragged corner is re-latched against the pose it is seen through
        // NOW, exactly as a fresh capture would be - including the focused Z,
        // which Compute's one-focus check reads. The device pose latched at the
        // first corner stays: a drag adjusts a pick, it does not start a set.
        if (r.markMoved >= 0 && r.markMoved < kRegMarkMax && regMarkHave_[r.markMoved]) {
            AlignPoint& p = regMark_[r.markMoved];
            p.gx = gx;
            p.gy = gy;
            p.px = r.markPx;
            p.py = r.markPy;
            p.wx = r.markWx;
            p.wy = r.markWy;
            if (haveGz) {
                p.gz   = gz;
                p.hasZ = true;
            }
        }

        // A drag that just ended is an edit of the record, by the one route
        // every edit of it takes.
        if (boxWasDragging && !regBoxDragging_ &&
            (static_cast<double>(regBoxW_) != reg.toolWmm ||
             static_cast<double>(regBoxH_) != reg.toolLmm)) {
            DeviceReg edited = reg;
            edited.toolWmm = static_cast<double>(regBoxW_);
            edited.toolLmm = static_cast<double>(regBoxH_);
            workflow_.QueueDeviceReg(edited);
            FD_INFO("devreg", "'%s': tool footprint %.3f x %.3f mm (dragged)",
                    regDevice_, edited.toolWmm, edited.toolLmm);
        }
        // Click-to-move is an armed action wherever it appears — the same rule
        // the Aux window and the two teach pages follow — and layer 2 owns it.
        if (r.moveRequested) {
            if (jogArmed_) workflow_.MoveGantryRelativeMM(r.dxMM, r.dyMM);
            else FD_WARN("devreg", "click-to-move ignored - arm jog first");
        }
    }
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##drsplit", true, kSplitW, avail.y)) {
        regCtlW_ -= ImGui::GetIO().MouseDelta.x;
        regCtlSplitting_ = true;
    } else if (regCtlSplitting_) {
        regCtlSplitting_ = false;
        SaveUiSettings();   // persist on drag END, like every other splitter here
    }
    ImGui::SameLine(0.0f, 0.0f);

    // ---- Right: the device, then three tabs ----------------------------------
    // Registration (the record) | Capture tool points (the teach, steps 1-3) |
    // Verify and test (the two drives). Stacked,
    // the whole thing was three screens tall and the record - the one part an
    // operator comes back to - sat above a teach they do once per gripper.
    ImGui::BeginChild("drCtl", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);

    ImGui::TextDisabled("Relates a device's own axis origin to the camera gantry's,");
    ImGui::TextDisabled("so its tool tip has a known position at every moment.");

    ImGui::SeparatorText("Device");
    DeviceCombo("device", regDevice_, sizeof(regDevice_), hardware_,
                "The device carrying the tool - a gripper on a hexapod.\n"
                "Each is registered separately: they share no mount and no tool.");
    DeviceCombo("reference", regRef_, sizeof(regRef_), hardware_,
                "The device the CAMERA rides on. Everything is measured and\n"
                "reported in ITS raw axis coordinates, which is the space a\n"
                "pick and a taught position already live in.");
    if (sameDev)
        ImGui::TextColored(kBad, "a device cannot be registered to itself");
    if (!havePose)
        ImGui::TextColored(kBad, "'%s' not connected (or reports no X/Y)", regRef_);
    if (!haveImg)
        ImGui::TextColored(kBad, "no camera feed");
    // The last thing a button said, above the tabs so it is read whichever tab
    // the button was on: a refused capture and a refused drive on Verify both
    // report here.
    if (!regTeachMsg_.empty())
        ImGui::TextWrapped("%s", regTeachMsg_.c_str());

    // Which of the three steps the record already answers. Done is read off the
    // RECORD, not the session: a tip taught last month is still taught, and an
    // operator opening this page to look at it should find green steps,
    // not empty ones. The corner marks are the one exception - they are
    // consumed into the tip and cleared - so steps 1-2 read done from the tip
    // they produced, or from marks taken this session.
    int regMarked = 0;
    for (int i = 0; i < regMarkCount_; ++i) regMarked += regMarkHave_[i] ? 1 : 0;
    // A tip computed by the teach predicts, at the pose it was taught at,
    // exactly the point it measured (t = tipMeas - R*h). A record whose rotation
    // was re-adopted after its tip does not - and is not a taught tip.
    const bool tipTaught = [&] {
        if (!reg.valid || !reg.hasImprintPose) return false;
        const RegVec3 at = DeviceRegPredict(reg, reg.imprintPose);
        return std::fabs(at.x - reg.tipMeas.x) < 1e-6 &&
               std::fabs(at.y - reg.tipMeas.y) < 1e-6 &&
               std::fabs(at.z - reg.tipMeas.z) < 1e-6;
    }();
    const bool step1Done = tipTaught || regMarked > 0;
    const bool step2Done = tipTaught || regMarked == regMarkCount_;
    const bool step3Done = tipTaught;

    // A tab's label goes green once everything on it is done, so the row of
    // tabs reads as the progress bar for a one-time setup. The push wraps only
    // BeginTabItem: the label is drawn there, and the content keeps its colours.
    const auto tabBegin = [&](const char* label, bool done) {
        if (done) ImGui::PushStyleColor(ImGuiCol_Text, kGood);
        const bool open = ImGui::BeginTabItem(label);
        if (done) ImGui::PopStyleColor();
        return open;
    };
    const auto stepHeader = [&](const char* title, bool done) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s%s", title, done ? "  - done" : "");
        if (done) ImGui::PushStyleColor(ImGuiCol_Text, kGood);
        ImGui::SeparatorText(buf);
        if (done) ImGui::PopStyleColor();
    };

    if (!ImGui::BeginTabBar("drTabs")) {
        ImGui::EndChild();
        return;
    }

    // ==== Tab: Registration - what the machine currently believes =============
    if (tabBegin("Registration", reg.valid)) {
    ImGui::Spacing();
    if (!reg.valid) {
        ImGui::TextColored(kBad, "NOT REGISTERED");
        ImGui::TextDisabled("move_to_point refuses and a placement measures from");
        ImGui::TextDisabled("scratch every time. Teach the tip on Capture tool points.");
    } else {
        ImGui::TextColored(kGood, "tip at device zero  %+.4f  %+.4f  %+.4f mm",
                           reg.t.x, reg.t.y, reg.t.z);
        RegVec3     tip;
        std::string why;
        if (workflow_.DeviceRegTip(regDevice_, tip, &why))
            ImGui::Text("tip now             %+.4f  %+.4f  %+.4f mm", tip.x, tip.y, tip.z);
        else
            ImGui::TextColored(kWarn, "tip now: unavailable - %s", why.c_str());
        // Where it was taught matters: the typed rotation's error is zero there
        // and grows with distance from it.
        if (reg.hasImprintPose)
            ImGui::TextDisabled("taught with '%s' at X %.3f  Y %.3f  Z %.3f", regDevice_,
                                reg.imprintPose.x, reg.imprintPose.y, reg.imprintPose.z);

        // The rotation's provenance, stated plainly. Nothing measured it, and a
        // reader who assumes otherwise will trust an approach further than it
        // deserves — which is precisely what the check exists to bound.
        ImGui::TextDisabled("rotation from %s, adopted %s",
                            reg.rotSource.empty() ? "(none)" : reg.rotSource.c_str(),
                            reg.regAt.empty() ? "(undated)" : reg.regAt.c_str());
        ImGui::Text("tip taught %s", reg.tipAt.empty() ? "(undated)" : reg.tipAt.c_str());

        // The one assumption on this page that is about the MACHINE rather than
        // the measurement, so it is stated where the record is read and not
        // buried in a header. With the controller's pivot on the tool tip, a
        // rotation turns the tool about the very point the record stores and
        // does not move it - so the taught rotations describe every rotation and
        // the guard has nothing to protect. Off, the tip swings, and a record
        // taken at one rotation describes no other.
        {
            bool pivot = reg.pivotAtTip;
            if (ImGui::Checkbox("pivot is at the tool tip", &pivot)) {
                DeviceReg edited = reg;
                edited.pivotAtTip = pivot;
                workflow_.QueueDeviceReg(edited);
                FD_INFO("devreg", "'%s': pivot-at-tip %s - rotations %s the tip",
                        regDevice_, pivot ? "ON" : "OFF",
                        pivot ? "no longer blank" : "again blank");
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("ON: U/V/W turn the tool about the tip itself, so the\n"
                                  "tip does not move and the record is good at every\n"
                                  "rotation. This is how both grippers here are set up.\n"
                                  "OFF: the tip swings about a pivot somewhere else, so\n"
                                  "predicting past %.2f deg from the taught rotations is\n"
                                  "refused - no marker, and every move through the\n"
                                  "registration fails until the device is turned back.",
                                  kDeviceRegRotTolDeg);
            if (reg.hasRot && !reg.pivotAtTip)
                ImGui::TextDisabled("taught at U %.3f  V %.3f  W %.3f", reg.taughtRot.x,
                                    reg.taughtRot.y, reg.taughtRot.z);
        }
        if (reg.tipPoints > 0)
            ImGui::TextDisabled("tip from %d corner picks", reg.tipPoints);

        // Three states, not two. A check that RAN and disagreed is not the same
        // thing as a check that was never run: one has measured what the rotation
        // costs and reported a number, the other has measured nothing. Saying
        // "nothing has measured the tip" to an operator who just watched the check
        // drive, focus and pick reads as the page having missed it.
        if (reg.verified) {
            ImGui::TextColored(kGood, "verified - %.4f mm at the check, %s",
                               reg.checkResidualMm,
                               reg.checkedAt.empty() ? "(undated)" : reg.checkedAt.c_str());
        } else if (!reg.checkedAt.empty()) {
            // How far the check stood from the taught pose is the whole reading of
            // the residual: the same miss wherever you check is the tool, one that
            // grows with distance is the rotation.
            const double cdx = reg.checkPose.x - reg.imprintPose.x;
            const double cdy = reg.checkPose.y - reg.imprintPose.y;
            const double cdz = reg.checkPose.z - reg.imprintPose.z;
            const double cd  = std::sqrt(cdx * cdx + cdy * cdy + cdz * cdz);
            ImGui::TextColored(kBad, "CHECK FAILED - %.4f mm out, %.3f mm from the",
                               reg.checkResidualMm, cd);
            ImGui::TextColored(kBad, "taught pose, %s", reg.checkedAt.c_str());
            ImGui::TextWrapped("The same miss at every distance is the TOOL - re-teach "
                               "the tip. A miss that grows with distance is "
                               "the ROTATION, and it is only as square as the mount the "
                               "axis transforms describe. Check again further out to "
                               "tell the two apart.");
        } else {
            ImGui::TextColored(kWarn, "UNVERIFIED - nothing has measured the tip away\n"
                                      "from the pose it was taught at, so nothing has\n"
                                      "put a number on what the rotation costs");
        }
    }

    // ---- The tool's footprint ------------------------------------------------
    // Typed, not measured, and part of the RECORD: it describes the jaws on
    // this device, which is machine state exactly like the tip they are centred
    // on. Drawn around the tip on this image, the Gripped Object page and the
    // Aux view, turned by the device's live U; nothing refuses on it. On the
    // record's tab, and NOT gated on the record being valid: the size of the
    // jaws is known before the tip is, and an operator setting up a new gripper
    // types it before the tip is marked.
    ImGui::SeparatorText("Tool footprint");
    {
        double w = reg.toolWmm, l = reg.toolLmm;
        bool   changed = false;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::InputDouble("##toolw", &w, 0.0, 0.0, "%.3f")) changed = true;
        ImGui::SameLine();
        ImGui::TextUnformatted("x");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::InputDouble("W x L mm", &l, 0.0, 0.0, "%.3f")) changed = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The jaws' outline, centred on the tip: width along %s's\n"
                              "X, length along its Y, turned with the device's U. Drawn\n"
                              "to scale on every camera view that shows the tip, so 'is\n"
                              "the gripper standing on the part' is a picture. Its\n"
                              "corners can be dragged on the image once it has a size.",
                              regRef_);
        if (w <= 0.0 || l <= 0.0)
            ImGui::TextDisabled("0 draws nothing - enter the jaws' size to see the box");
        if (changed) {
            if (w < 0.0) w = 0.0;
            if (l < 0.0) l = 0.0;
            if (w > 500.0) w = 500.0;
            if (l > 500.0) l = 500.0;
            // The record may not exist yet (nothing taught): name it, exactly
            // as CompleteDeviceRegTip does, or SetDeviceReg drops a nameless one.
            DeviceReg edited = reg;
            edited.device    = regDevice_;
            edited.refDevice = regRef_;
            edited.toolWmm   = w;
            edited.toolLmm   = l;
            workflow_.QueueDeviceReg(edited);
        }
    }

    // ---- The fleet -----------------------------------------------------------
    ImGui::SeparatorText("All registered devices");
    for (const auto& [name, r] : workflow_.DeviceRegs()) {
        if (!r.valid) continue;
        ImGui::TextColored(r.verified ? kGood : kWarn, "%-12s", name.c_str());
        ImGui::SameLine();
        // Same three states as the panel above, in one word each: a failed check
        // has measured something and must not read like one that never ran.
        char state[48];
        if (r.verified) snprintf(state, sizeof(state), "verified");
        else if (!r.checkedAt.empty())
            snprintf(state, sizeof(state), "CHECK FAILED %.3f mm", r.checkResidualMm);
        else snprintf(state, sizeof(state), "UNVERIFIED");
        ImGui::Text("-> %-12s t (%+.3f, %+.3f, %+.3f)  %s", r.refDevice.c_str(),
                    r.t.x, r.t.y, r.t.z, state);
    }

    ImGui::Spacing();
    ImGui::TextWrapped("A registration is an APPROACH. Its rotation is a typed "
                       "permutation and its tip is one measured point, so it gets a "
                       "tool close - a placement still measures and corrects. What it "
                       "buys is that the correction loop lands on its first pass.");
    ImGui::EndTabItem();
    }

    // ==== Tab: Capture tool points - the whole teach ==========================
    // Nothing here moves the registered device. The operator jogs the CAMERA
    // onto the tool wherever it stands; the device's pose is READ at the first
    // mark and the tip is computed there, so nobody has to know or set it.
    if (tabBegin("Capture tool points", step1Done && step2Done && step3Done)) {
    ImGui::Spacing();
    ImGui::TextWrapped("Leave '%s' where it stands - ideally where it works. Put the "
                       "camera on its tool, mark the corners, compute. Nothing moves "
                       "the tool.", regDevice_);

    // The registered device's live pose, and whether it has moved since the
    // first mark of the set. Read, never set.
    {
        std::map<std::string, double> live;
        const bool haveLive = workflow_.DevicePose(regDevice_, live);
        const auto get = [&](const char* a) {
            const auto it = live.find(a);
            return it == live.end() ? 0.0 : it->second;
        };
        if (!haveLive) {
            ImGui::TextColored(kBad, "'%s' not connected - its pose is half of the "
                                     "measurement", regDevice_);
        } else {
            ImGui::TextDisabled("'%s' at X %.4f  Y %.4f  Z %.4f", regDevice_, get("X"),
                                get("Y"), get("Z"));
            if (live.count("U") && live.count("V") && live.count("W"))
                ImGui::TextDisabled("    U %.4f  V %.4f  W %.4f", get("U"), get("V"),
                                    get("W"));
        }
        if (haveLive && regMarked > 0 && regMarkPoseHave_ && regMarkDev_ == regDevice_) {
            const double dx = get("X") - regMarkPose_.x, dy = get("Y") - regMarkPose_.y,
                         dz = get("Z") - regMarkPose_.z;
            const bool rotOff =
                regMarkHasRot_ && (std::fabs(get("U") - regMarkRot_.x) > 0.01 ||
                                   std::fabs(get("V") - regMarkRot_.y) > 0.01 ||
                                   std::fabs(get("W") - regMarkRot_.z) > 0.01);
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.01 || rotOff)
                ImGui::TextColored(kWarn, "'%s' has moved since the first mark - Clear "
                                          "and mark again", regDevice_);
        }
    }

    // ---- Step 1: the camera on the tool --------------------------------------
    stepHeader("Step 1  Camera on the tool", step1Done);
    ImGui::TextWrapped("Jog '%s' until the gripper tip is in the picture and FOCUS on "
                       "it. The focused Z is the tip's height - focus once, then mark "
                       "every corner.", regRef_);
    if (havePose && haveGz)
        ImGui::TextDisabled("camera at X %.4f  Y %.4f  Z %.4f", gx, gy, gz);

    // ---- Step 2: the corners -------------------------------------------------
    stepHeader("Step 2  Mark the tool", step2Done);
    {
        ImGui::TextWrapped("Put the crosshair on the corners of the gripper tip, "
                           "spaced evenly around it, and capture each.");
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("corners", &regMarkCount_, 0, 0);
        if (regMarkCount_ < 2) regMarkCount_ = 2;
        if (regMarkCount_ > kRegMarkMax) regMarkCount_ = kRegMarkMax;
        ImGui::SameLine();
        if (regMarkCount_ == 2)
            ImGui::TextColored(kWarn, "two must be straight across the middle");
        else
            ImGui::TextDisabled("spaced evenly around the mark");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Averaging N picks gives the centre only if they are\n"
                              "placed symmetrically. Two must be a true diameter;\n"
                              "four around the rim is far more forgiving.");

        for (int i = 0; i < regMarkCount_; ++i) {
            ImGui::PushID(i);
            if (i) ImGui::SameLine();
            const bool active = (regMarkActive_ == i);
            if (active)
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
            char lbl[16];
            std::snprintf(lbl, sizeof(lbl), "%s%d", regMarkHave_[i] ? "*" : "", i + 1);
            if (ImGui::Button(lbl, ImVec2(52.0f, 0.0f))) regMarkActive_ = i;
            if (active) ImGui::PopStyleColor();
            ImGui::PopID();
        }

        ImGui::BeginDisabled(!haveImg || !havePose || sameDev);
        if (ImGui::Button("Capture corner", ImVec2(180.0f, 0.0f)))
            CaptureDeviceRegCorner(gx, gy, gz, haveGz);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Latches the crosshair with '%s's pose. The first corner\n"
                              "also reads where '%s' stands - that pose is the one the\n"
                              "tip is taught at, rotations included.",
                              regRef_, regDevice_);
        ImGui::SameLine();
        if (ImGui::Button("Clear", ImVec2(80.0f, 0.0f))) ClearDeviceRegMarks();

        ImGui::TextDisabled("%d of %d marked", regMarked, regMarkCount_);
        if (regMarked > 0 && regMarkPoseHave_)
            ImGui::TextDisabled("pinned to '%s' at X %.4f  Y %.4f  Z %.4f",
                                regMarkDev_.c_str(), regMarkPose_.x, regMarkPose_.y,
                                regMarkPose_.z);
    }

    // ---- Step 3: compute -----------------------------------------------------
    stepHeader("Step 3  Compute the tip", step3Done);
    {
        ImGui::TextWrapped("The centre of the corners is the tip in XY and the focused "
                           "Z its height, at the pose '%s' stood in.", regDevice_);
        ImGui::BeginDisabled(busy || sameDev);
        if (ImGui::Button("Compute and save tip", ImVec2(180.0f, 0.0f)))
            CompleteDeviceRegTip();
        ImGui::EndDisabled();
        if (reg.valid && reg.hasImprintPose)
            ImGui::TextDisabled("tip taught %s from %d picks at X %.3f  Y %.3f  Z %.3f - "
                                "computing again replaces it",
                                reg.tipAt.empty() ? "(undated)" : reg.tipAt.c_str(),
                                reg.tipPoints, reg.imprintPose.x, reg.imprintPose.y,
                                reg.imprintPose.z);
    }

    // ---- The picture the tool is marked under --------------------------------
    DrawDeviceRegCameraView();
    ImGui::EndTabItem();
    }

    // ==== Tab: Verify and test - drive through the registration ==============
    // Two buttons, each the registration read one way: the TOOL to where the
    // camera looks, the CAMERA to where the tool is. Watching the tool arrive
    // under the cross is how an operator sees what a record is worth. Green
    // once a check has passed (the check itself runs from a recipe step,
    // device_reg_check_run).
    //
    // A move longer than kRegConfirmMm asks first: the popup holds the move
    // exactly as computed at the click, and Confirm issues that.
    if (tabBegin("Verify and test", reg.verified)) {
    ImGui::Spacing();

    // Ask, or go: the one place both buttons decide.
    const auto requestMove = [&](const char* devName,
                                 std::vector<std::pair<std::string, double>> targets,
                                 double travel, const std::string& what) {
        if (!jogArmed_) {
            regTeachMsg_ = "arm jog first - this drives a device";
            return;
        }
        if (travel > kRegConfirmMm) {
            regConfirmDev_     = devName;
            regConfirmTargets_ = std::move(targets);
            char b[96];
            std::snprintf(b, sizeof(b), "\n\n%.3f mm of travel - more than %.1f mm.", travel,
                          kRegConfirmMm);
            regConfirmText_ = what + b;
            regConfirmOpen_ = true;
            ImGui::OpenPopup("Confirm move##devreg");
            return;
        }
        driveAxes(devName, targets);
    };

    double cx = 0.0, cy = 0.0, cz = 0.0;
    bool   haveCam = false;
    {
        std::map<std::string, double> refPose;
        if (workflow_.DevicePose(regRef_, refPose)) {
            const auto ix = refPose.find("X"), iy = refPose.find("Y"), iz = refPose.find("Z");
            if (ix != refPose.end() && iy != refPose.end() && iz != refPose.end()) {
                cx = ix->second; cy = iy->second; cz = iz->second;
                haveCam = true;
            }
        }
    }

    if (!reg.valid)
        ImGui::TextColored(kWarn, "nothing to drive through until the tip is taught");
    else if (!reg.verified)
        ImGui::TextDisabled("unverified - watch the first moves: the tool may not land\n"
                            "exactly under the cross, and that is what this shows");

    // ---- Move tool to camera -------------------------------------------------
    // The same solve a recipe's move_to_point does (h = R^T (p - t)), with the
    // same rotation guard. It stays an APPROACH: nothing here closes a loop.
    ImGui::SeparatorText("Move tool to camera");
    {
        ImGui::TextWrapped("Drives '%s' so its tip lands where '%s' is looking now.",
                           regDevice_, regRef_);
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("Z offset", &regSendZOff_, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Added to the target Z in '%s' AXIS space.\n"
                              "NEGATIVE lifts the tool AWAY from the focused\n"
                              "surface (measured on this machine, 2026-09-01);\n"
                              "positive drives it toward the surface, and 0 sends\n"
                              "the tip to the focused surface itself - a TOUCH.\n"
                              "The sign is a fact about this gantry, not a rule:\n"
                              "its transform inverts Z. Re-check it after any\n"
                              "change to the transforms file.", regRef_);
        if (regSendZOff_ == 0.0)
            ImGui::TextColored(kWarn, "Z offset 0 = the tip TOUCHES the focused surface");

        const RegVec3 target{cx, cy, cz + regSendZOff_};
        const RegVec3 h = DeviceRegSolve(reg, target);
        std::string   refuse;
        double        travel = 0.0;
        if (!reg.valid)    refuse = "no registration to drive through";
        else if (!haveCam) refuse = std::string("no live X/Y/Z from '") + regRef_ + "'";
        std::map<std::string, double> devPose;
        if (refuse.empty()) {
            if (!workflow_.DevicePose(regDevice_, devPose)) {
                refuse = std::string("'") + regDevice_ + "' is not connected";
            } else {
                // A record taken at one set of rotations describes no other
                // (unless the pivot is at the tip). Same guard as the command.
                const auto iu = devPose.find("U"), iv = devPose.find("V"),
                           iw = devPose.find("W");
                if (reg.hasRot && iu != devPose.end() && iv != devPose.end() &&
                    iw != devPose.end()) {
                    std::string why;
                    if (!DeviceRegRotationOk(reg, RegVec3{iu->second, iv->second, iw->second},
                                             &why))
                        refuse = why;
                }
                const auto ix = devPose.find("X"), iy = devPose.find("Y"),
                           iz = devPose.find("Z");
                if (ix != devPose.end() && iy != devPose.end() && iz != devPose.end()) {
                    const double dx = h.x - ix->second, dy = h.y - iy->second,
                                 dz = h.z - iz->second;
                    travel = std::sqrt(dx * dx + dy * dy + dz * dz);
                } else if (refuse.empty()) {
                    refuse = std::string("'") + regDevice_ + "' does not report X/Y/Z";
                }
            }
        }
        if (refuse.empty())
            ImGui::TextDisabled("puts '%s' at (%.4f, %.4f, %.4f), %.3f mm", regDevice_,
                                h.x, h.y, h.z, travel);
        else
            ImGui::TextColored(kWarn, "%s", refuse.c_str());

        ImGui::BeginDisabled(busy || sameDev || !refuse.empty());
        if (IconButton(UiIcon("rg_tool_to_cam"), "Move tool to camera", ImVec2(200.0f, 0.0f))) {
            char what[256];
            std::snprintf(what, sizeof(what),
                          "Move '%s' so its tip lands under the camera?\n\n"
                          "'%s' to X %.4f  Y %.4f  Z %.4f%s",
                          regDevice_, regDevice_, h.x, h.y, h.z,
                          reg.verified ? "" : "\nThe registration is UNVERIFIED.");
            FD_INFO("devreg", "move tool to camera: '%s' to (%.4f, %.4f, %.4f), %.3f mm%s",
                    regDevice_, h.x, h.y, h.z, travel,
                    reg.verified ? "" : " - unverified registration");
            requestMove(regDevice_, {{"X", h.x}, {"Y", h.y}, {"Z", h.z}}, travel, what);
        }
        ImGui::EndDisabled();
    }

    // ---- Move camera to tool -------------------------------------------------
    // The tip comes from Workflow::DeviceRegTip - the SAME predictor the
    // overlay and the tip topics use, refusals and all.
    ImGui::SeparatorText("Move camera to tool");
    {
        ImGui::TextWrapped("Drives '%s' so the camera centre lands on '%s's tip. The "
                           "tool does not move.", regRef_, regDevice_);
        ImGui::Checkbox("focus Z too", &regLookZ_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Off: only X and Y move, and whatever the camera is\n"
                              "focused on now stays in focus.\n"
                              "On: Z goes to the height that focuses ON THE TIP -\n"
                              "which is the camera moving toward the tool. Watch it.");

        RegVec3     tip;
        std::string refuse;
        const bool  haveTip = workflow_.DeviceRegTip(regDevice_, tip, &refuse);
        if (haveTip) refuse.clear();
        if (refuse.empty() && !haveCam)
            refuse = std::string("no live X/Y/Z from '") + regRef_ + "'";
        double travel = 0.0;
        if (haveTip && haveCam) {
            const double dx = tip.x - cx, dy = tip.y - cy;
            const double dz = regLookZ_ ? tip.z - cz : 0.0;
            travel = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        if (refuse.empty())
            ImGui::TextDisabled("camera to (%.4f, %.4f%s), %.3f mm", tip.x, tip.y,
                                regLookZ_ ? ", Z too" : "", travel);
        else
            ImGui::TextColored(kWarn, "%s", refuse.c_str());

        ImGui::BeginDisabled(busy || sameDev || !refuse.empty());
        if (IconButton(UiIcon("rg_cam_to_tool"), "Move camera to tool", ImVec2(200.0f, 0.0f))) {
            char what[256];
            if (regLookZ_)
                std::snprintf(what, sizeof(what),
                              "Move '%s' over '%s's tip?\n\n'%s' to X %.4f  Y %.4f  Z %.4f\n"
                              "Z too - the camera moves toward the tool.",
                              regRef_, regDevice_, regRef_, tip.x, tip.y, tip.z);
            else
                std::snprintf(what, sizeof(what),
                              "Move '%s' over '%s's tip?\n\n'%s' to X %.4f  Y %.4f (XY only)",
                              regRef_, regDevice_, regRef_, tip.x, tip.y);
            FD_INFO("devreg", "move camera to tool: '%s' to the '%s' tip (%.4f, %.4f, %.4f), "
                              "%s, %.3f mm",
                    regRef_, regDevice_, tip.x, tip.y, tip.z,
                    regLookZ_ ? "XYZ" : "XY only", travel);
            if (regLookZ_)
                requestMove(regRef_, {{"X", tip.x}, {"Y", tip.y}, {"Z", tip.z}}, travel, what);
            else
                requestMove(regRef_, {{"X", tip.x}, {"Y", tip.y}}, travel, what);
        }
        ImGui::EndDisabled();
    }
    if (!jogArmed_)
        ImGui::TextDisabled("arm jog (Jog window) - these drive a device");

    // ---- The confirm popup ---------------------------------------------------
    // Raised only by a click on this tab while nothing runs (both buttons gate
    // on orch_.Busy()), and it closes itself if a run starts anyway: a modal
    // must never sit over the prompt window a worker is blocked on.
    if (ImGui::BeginPopupModal("Confirm move##devreg", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        if (busy || !regConfirmOpen_) {
            regConfirmOpen_ = false;
            ImGui::CloseCurrentPopup();
        }
        // A fixed wrap width: TextWrapped in an auto-resizing window wraps at
        // whatever width the window had last frame.
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 400.0f);
        ImGui::TextUnformatted(regConfirmText_.c_str());
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (ImGui::Button("Move", ImVec2(120.0f, 0.0f))) {
            FD_INFO("devreg", "move confirmed: '%s'", regConfirmDev_.c_str());
            driveAxes(regConfirmDev_.c_str(), regConfirmTargets_);
            regConfirmOpen_ = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)) ||
            ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            regConfirmOpen_ = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    ImGui::EndTabItem();
    }

    ImGui::EndTabBar();
    ImGui::EndChild();
}

} // namespace firedog
