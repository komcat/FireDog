// PageDeviceReg.cpp : the Registration page (View menu, F8) — where a device's
// tool tip is in the camera device's space, taught in four steps.
// Definitions of App members — see App.h.
//
// The teach lives HERE rather than in a procedure script, and that is the one
// structural decision worth defending. Every step needs an operator at the
// machine: placing a coupon, jogging a tool into contact by eye, and marking the
// corners of the mark it left. There is nothing for a recipe to automate, a
// prompt-driven script would only be this page with the buttons hidden behind
// modal windows — and a second way to create a registration is a second answer
// to where the tool is.
//
// What a recipe gets instead is the CONSUMING end: `device_reg` to read the
// record, `move_to_point` to move through it, `device_reg_check_run` to check
// it. Those are in command/CommandsDeviceReg.cpp.
//
// The four steps, and why they are in this order:
//   1  IMPRINT POSE   the operator jogs the tool down onto a coupon until it
//                     marks, and captures that pose. Then Home, so the tool is
//                     out of the way before the camera goes looking.
//   2  CAMERA VIEW    the operator drives the camera over the mark, focuses,
//                     and captures that pose. The focused Z IS the tip's height:
//                     the tool was resting on the surface now in focus.
//   3  CORNERS        marks on the live image, in machine mm.
//   4  COMPLETE       centre of the marks + the focused Z = the tip. t follows.
//
// The right-hand column is the device pair over four tabs: Registration (the
// record, the footprint, the fleet), Teach Pose (steps 1-2), Capture tool
// points (steps 3-4), Verify and test (the check, Send tool here, Look at the
// tool). Each step is headed "Step n ..." and turns green when the RECORD
// answers it, so a registration taught months ago opens as four green steps -
// this is a one-time setup, and the page must say so rather than look untaught.
// A tab goes green when everything on it is done; Verify only on a PASSED check.
//
// Nothing on this page actuates on its own. Every move is either the operator's
// own jog or an explicit "drive to the pose you taught" button.
#include "App.h"

#include "imgui.h"

#include "command/ExecutionContext.h"   // SystemScriptPath — the check procedure
#include "core/DeviceReg.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "ui/Widgets.h"

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

// A device's taught positions, offered as a source for one of the three poses.
// Returns the picked one for this frame only — the caller resolves it to
// coordinates immediately (App::AdoptDeviceRegPose says why).
//
// Hidden positions are withheld: they are the alignment system's private
// `algn_...` poses and are noise to somebody teaching a registration. Same rule
// the recipe picker follows — `hidden` governs what is OFFERED, never what
// resolves.
const SavedPos* TaughtPosePicker(const char* id, const SavedPosMap& all,
                                 const std::string& device)
{
    const SavedPos* picked = nullptr;
    ImGui::SetNextItemWidth(140.0f);
    if (!ImGui::BeginCombo(id, "or taught...")) return nullptr;

    const auto it = all.find(device);
    if (it == all.end() || it->second.empty()) {
        ImGui::TextDisabled("no taught positions for '%s'", device.c_str());
    } else {
        bool any = false;
        for (const SavedPos& sp : it->second) {
            if (sp.hidden) continue;
            any = true;
            if (ImGui::Selectable(sp.name)) picked = &sp;
        }
        if (!any) ImGui::TextDisabled("only system positions taught");
    }
    ImGui::EndCombo();
    return picked;
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

// The one writer of the three teachable poses. Capturing a live pose and
// adopting a taught one both come through here, so the two cannot drift — and
// the rotation latch, which only the imprint slot wants, is stated once.
void App::StoreDeviceRegPose(RegPoseSlot slot, const RegVec3& p, const std::string& how)
{
    DeviceReg r = workflow_.DeviceRegFor(regDevice_);
    r.device    = regDevice_;
    r.refDevice = regRef_;

    const char* what = "";
    switch (slot) {
    case kRegImprint:
        r.imprintPose    = p;
        r.hasImprintPose = true;
        what             = "imprint pose";
        // The rotations this record is pinned to, read LIVE from the device
        // whichever way the pose arrived. Naming a pose does not turn anything,
        // and nothing on this machine commands U/V/W — so the device's current
        // rotations are what the tool will be at when it makes the mark, which
        // is the only moment they have to describe.
        {
            std::map<std::string, double> live;
            if (workflow_.DevicePose(regDevice_, live)) {
                const auto iu = live.find("U"), iv = live.find("V"), iw = live.find("W");
                if (iu != live.end() && iv != live.end() && iw != live.end()) {
                    r.taughtRot = RegVec3{iu->second, iv->second, iw->second};
                    r.hasRot    = true;
                }
            }
        }
        break;
    case kRegHome:
        r.homePose    = p;
        r.hasHomePose = true;
        what          = "home pose";
        break;
    case kRegView:
        r.tipView    = p;
        r.hasTipView = true;
        what         = "camera view";
        break;
    }

    workflow_.QueueDeviceReg(r);
    regTeachMsg_ = std::string(what) + " " + how;
    FD_INFO("devreg", "'%s' %s %s: (%.4f, %.4f, %.4f)", regDevice_, what, how.c_str(),
            p.x, p.y, p.z);
}

// Stand the machine where the pose belongs and latch it. The imprint and home
// poses are on the registered device; the camera view is on the reference one,
// and its Z is not decoration — the focused reference Z over the coupon IS the
// height the tool was at, so capture it AFTER focusing.
void App::CaptureDeviceRegPose(RegPoseSlot slot)
{
    regTeachMsg_.clear();
    const char* dev = (slot == kRegView) ? regRef_ : regDevice_;

    std::map<std::string, double> pose;
    if (!workflow_.DevicePose(dev, pose)) {
        regTeachMsg_ = std::string("'") + dev + "' is not connected";
        return;
    }
    const auto ix = pose.find("X"), iy = pose.find("Y"), iz = pose.find("Z");
    if (ix == pose.end() || iy == pose.end() || iz == pose.end()) {
        regTeachMsg_ = std::string("'") + dev + "' does not report X/Y/Z";
        return;
    }
    StoreDeviceRegPose(slot, RegVec3{ix->second, iy->second, iz->second}, "captured");
}

// Take the pose from a position the product already has taught, instead of
// standing the machine there to make a new one. A machine usually has a home for
// each device long before anybody registers one.
//
// It resolves to COORDINATES here and stores those — never the position's uid.
// Saved positions belong to the product document and a registration is machine
// state, so a record holding a uid would go blank the moment a different product
// was loaded, or none was. The convenience is worth having; the dependency is
// not, and this is the same reason the three poses live in the record at all.
void App::AdoptDeviceRegPose(RegPoseSlot slot, const SavedPos& pos)
{
    regTeachMsg_.clear();
    const auto ix = pos.vals.find("X"), iy = pos.vals.find("Y"), iz = pos.vals.find("Z");
    if (ix == pos.vals.end() || iy == pos.vals.end() || iz == pos.vals.end()) {
        // Positions are per-axis maps and need not define every axis. A pose
        // built from a partial one would silently read the missing axes as zero,
        // which on a hexapod is a real place and not the one anybody meant.
        regTeachMsg_ = std::string("'") + pos.name + "' does not define X, Y and Z";
        return;
    }
    StoreDeviceRegPose(slot, RegVec3{ix->second, iy->second, iz->second},
                       std::string("from taught position '") + pos.name + "'");
}

// Step 4. The centre of the marks is the tip in XY; the view's focused Z is the
// tip in Z. Everything before this was capture; this is the only step that
// computes, and the only one that can refuse.
void App::CompleteDeviceRegTip()
{
    regTeachMsg_.clear();
    DeviceReg r = workflow_.DeviceRegFor(regDevice_);
    r.device    = regDevice_;
    r.refDevice = regRef_;

    if (!r.hasImprintPose) { regTeachMsg_ = "step 1 first: capture the imprint pose"; return; }
    if (!r.hasTipView)     { regTeachMsg_ = "step 2 first: capture the camera view"; return; }

    int n = 0;
    double cx = 0.0, cy = 0.0;
    for (int i = 0; i < regMarkCount_; ++i) {
        if (!regMarkHave_[i]) continue;
        cx += regMark_[i].wx;
        cy += regMark_[i].wy;
        ++n;
    }
    if (n < 2) {
        regTeachMsg_ = "mark at least two corners of the imprint (step 3)";
        return;
    }
    if (n < regMarkCount_) {
        regTeachMsg_ = "mark every corner, or set the count to what you actually took";
        return;
    }
    cx /= static_cast<double>(n);
    cy /= static_cast<double>(n);
    // No width check. An earlier version measured the mark's width and refused
    // the teach when it disagreed with a typed tool width; the width of an
    // imprint is a fact about the coupon and the ink rather than the tool, so it
    // refused good picks. The picks are the operator's judgement, and the
    // footprint box on the image is where a bad one becomes visible.

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

    const RegVec3 before = DeviceRegPredict(r, r.imprintPose);
    const RegVec3 tip{cx, cy, r.tipView.z};
    if (const std::string why =
            DeviceRegSetTip(r, r.imprintPose, tip, n); !why.empty()) {
        regTeachMsg_ = why;
        return;
    }

    const double moved = std::sqrt((tip.x - before.x) * (tip.x - before.x) +
                                   (tip.y - before.y) * (tip.y - before.y) +
                                   (tip.z - before.z) * (tip.z - before.z));
    workflow_.QueueDeviceReg(r);

    char buf[224];
    std::snprintf(buf, sizeof(buf),
                  "tip taught: (%.4f, %.4f, %.4f) from %d picks. UNVERIFIED until "
                  "the check runs.",
                  r.t.x, r.t.y, r.t.z, n);
    regTeachMsg_ = buf;
    FD_INFO("devreg", "'%s' tip taught by imprint: t=(%.4f, %.4f, %.4f), %.4f mm from "
                      "where the old record put it; %d picks",
            regDevice_, r.t.x, r.t.y, r.t.z, moved, n);

    for (int i = 0; i < kRegMarkMax; ++i) regMarkHave_[i] = false;
    regMarkActive_ = 0;
}

// The check is still a procedure: it drives two devices to a computed pose and
// waits on an operator pick, which is exactly the shape a script is for.
void App::StartDeviceRegCheckRun()
{
    const std::string script = SystemScriptPath("device_reg_check.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"reg_device", std::string(regDevice_)},
        {"reg_check_dx_mm", regCheckDx_},
        {"reg_check_dy_mm", regCheckDy_},
        {"reg_check_dz_mm", regCheckDz_},
        {"reg_tol_mm", regCheckTol_},
        {"reg_autofocus", regAutofocus_},
        {"reg_focus_range_mm", regFocusRangeMm_},
        {"reg_v_travel", regVTravel_},
        {"reg_v_jog", regVJog_},
        {"reg_settle_ms", static_cast<double>(regSettleMs_)},
    };
    regStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &regStartErr_))
        FD_WARN("devreg", "registration check rejected: %s", regStartErr_.c_str());
}

// ---- The page ---------------------------------------------------------------

void App::DrawDeviceRegPanel()
{
    const DeviceReg reg = workflow_.DeviceRegFor(regDevice_);
    const bool busy     = orch_.Busy();
    const bool sameDev  = std::string(regDevice_) == std::string(regRef_);

    // The reference device's live pose — what a mark is projected through, and
    // what the two capture buttons latch.
    double gx = 0.0, gy = 0.0;
    bool   havePose = false;
    {
        std::map<std::string, double> pose;
        if (workflow_.DevicePose(regRef_, pose)) {
            const auto ix = pose.find("X"), iy = pose.find("Y");
            if (ix != pose.end() && iy != pose.end()) {
                gx = ix->second;
                gy = iy->second;
                havePose = true;
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
    const auto driveTo = [&](const char* devName, const RegVec3& p) {
        driveAxes(devName, {{"X", p.x}, {"Y", p.y}, {"Z", p.z}});
    };

    // The control column is draggable: its four steps carry long explanatory
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
        CameraMark marks[kRegMarkMax];
        static const char* kLbl[kRegMarkMax] = {"c1", "c2", "c3", "c4"};
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
        // An imprint is a coarse mark: half the fit shows the whole frame to
        // find it, three times is enough to put a corner on its edge.
        mp.zoomMin    = 0.5f;
        mp.zoomMax    = 3.0f;

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

    // ---- Right: the device, then four tabs -----------------------------------
    // Registration (the record) | Teach Pose (steps 1-2) | Capture tool points
    // (steps 3-4) | Verify and test (the check and the two drives). Stacked,
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
    // the button was on: a capture on Teach Pose and a refused drive on Verify
    // both report here.
    if (!regTeachMsg_.empty())
        ImGui::TextWrapped("%s", regTeachMsg_.c_str());

    // Which of the four steps the record already answers. Done is read off the
    // RECORD, not the session: a tip taught last month is still taught, and an
    // operator opening this page to look at it should find four green steps,
    // not four empty ones. The corner marks are the one exception - they are
    // consumed into the tip and cleared - so step 3 reads done from the tip
    // they produced, or from a full set marked this session.
    int regMarked = 0;
    for (int i = 0; i < regMarkCount_; ++i) regMarked += regMarkHave_[i] ? 1 : 0;
    const bool step1Done = reg.hasImprintPose;
    const bool step2Done = reg.hasTipView;
    const bool step3Done = reg.valid || regMarked == regMarkCount_;
    const bool step4Done = reg.valid;

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
        ImGui::TextDisabled("scratch every time. Teach the tip on the next two tabs.");
    } else {
        ImGui::TextColored(kGood, "tip at device zero  %+.4f  %+.4f  %+.4f mm",
                           reg.t.x, reg.t.y, reg.t.z);
        RegVec3     tip;
        std::string why;
        if (workflow_.DeviceRegTip(regDevice_, tip, &why))
            ImGui::Text("tip now             %+.4f  %+.4f  %+.4f mm", tip.x, tip.y, tip.z);
        else
            ImGui::TextColored(kWarn, "tip now: unavailable - %s", why.c_str());

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
                               "the tip by imprint. A miss that grows with distance is "
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
    // types it while the coupon dries.
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

    // ==== Tab: Teach Pose - steps 1 and 2 =====================================
    if (tabBegin("Teach Pose", step1Done && step2Done)) {
    ImGui::Spacing();
    ImGui::TextDisabled("Two poses on the tool's device, one on the camera's.");

    // ---- Step 1: the imprint pose --------------------------------------------
    stepHeader("Step 1  Imprint pose", step1Done);
    ImGui::TextWrapped("Put a marking coupon under the tool. Jog '%s' DOWN by hand "
                       "until it just marks the coupon - barely, not pressing - then "
                       "capture that pose.", regDevice_);
    {
        ImGui::BeginDisabled(busy || sameDev);
        if (ImGui::Button("Capture imprint pose", ImVec2(180.0f, 0.0f)))
            CaptureDeviceRegPose(kRegImprint);
        ImGui::SameLine();
        if (const SavedPos* sp =
                TaughtPosePicker("##impFrom", session_.Positions(), regDevice_))
            AdoptDeviceRegPose(kRegImprint, *sp);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !reg.hasImprintPose);
        if (ImGui::Button("Go to it", ImVec2(80.0f, 0.0f)))
            driveTo(regDevice_, reg.imprintPose);
        ImGui::EndDisabled();
        if (reg.hasImprintPose)
            ImGui::TextDisabled("X %.4f  Y %.4f  Z %.4f", reg.imprintPose.x,
                                reg.imprintPose.y, reg.imprintPose.z);
        else
            ImGui::TextColored(kWarn, "not captured");

        ImGui::Spacing();
        ImGui::BeginDisabled(busy || sameDev);
        if (ImGui::Button("Capture home pose", ImVec2(180.0f, 0.0f)))
            CaptureDeviceRegPose(kRegHome);
        ImGui::SameLine();
        // The likeliest of the three to already exist: a machine has a home for
        // each device long before anybody registers one.
        if (const SavedPos* sp =
                TaughtPosePicker("##homeFrom", session_.Positions(), regDevice_))
            AdoptDeviceRegPose(kRegHome, *sp);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !reg.hasHomePose);
        if (ImGui::Button("Go home", ImVec2(80.0f, 0.0f)))
            driveTo(regDevice_, reg.homePose);
        ImGui::EndDisabled();
        if (reg.hasHomePose)
            ImGui::TextDisabled("X %.4f  Y %.4f  Z %.4f", reg.homePose.x,
                                reg.homePose.y, reg.homePose.z);
        else
            ImGui::TextColored(kWarn, "not captured - teach somewhere clear of the "
                                      "coupon,\nso the tool is out of the camera's way");
    }

    // ---- Step 2: the camera view ---------------------------------------------
    stepHeader("Step 2  Camera view over the mark", step2Done);
    ImGui::TextWrapped("Send the tool home, then drive '%s' until the mark is in the "
                       "picture and FOCUS on it. The focused Z is the tip's height - "
                       "the tool was resting on the surface now in focus.", regRef_);
    {
        ImGui::BeginDisabled(busy || sameDev);
        if (ImGui::Button("Capture camera view", ImVec2(180.0f, 0.0f)))
            CaptureDeviceRegPose(kRegView);
        ImGui::SameLine();
        // On the REFERENCE device: this is where the camera stands, not the tool.
        if (const SavedPos* sp =
                TaughtPosePicker("##viewFrom", session_.Positions(), regRef_))
            AdoptDeviceRegPose(kRegView, *sp);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !reg.hasTipView);
        if (ImGui::Button("Go to view", ImVec2(80.0f, 0.0f)))
            driveTo(regRef_, reg.tipView);
        ImGui::EndDisabled();
        if (reg.hasTipView) {
            ImGui::TextDisabled("X %.4f  Y %.4f  Z %.4f", reg.tipView.x,
                                reg.tipView.y, reg.tipView.z);
            // A taught position carries whatever Z it was saved at, which is not
            // necessarily a FOCUSED one — and this Z is the tip's height
            // measurement, not a convenience. Adopting one is fine as a way to
            // get there; focusing and re-capturing is what makes it a number.
            ImGui::TextDisabled("(the Z must be a focused one - it IS the tip's height)");
        } else {
            ImGui::TextColored(kWarn, "not captured");
        }
    }
    ImGui::EndTabItem();
    }

    // ==== Tab: Capture tool points - steps 3 and 4 ============================
    if (tabBegin("Capture tool points", step3Done && step4Done)) {
    ImGui::Spacing();
    ImGui::TextDisabled("Mark the imprint on the live image, then compute the tip.");

    // ---- Step 3: the corners -------------------------------------------------
    stepHeader("Step 3  Mark the imprint", step3Done);
    {
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

        ImGui::BeginDisabled(!haveImg || !havePose);
        if (ImGui::Button("Capture corner", ImVec2(180.0f, 0.0f))) {
            AlignPoint p;
            p.gx = gx;
            p.gy = gy;
            p.px = regPx_;
            p.py = regPy_;
            // The crosshair's offset from the image centre, in mm, added to the
            // pose it was taken at — so a mark keeps meaning the same place when
            // the operator jogs between corners.
            p.wx = gx + static_cast<double>(regPx_ - static_cast<float>(cameraW_) * 0.5f) *
                            static_cast<double>(pixelToMmX_);
            p.wy = gy + static_cast<double>(regPy_ - static_cast<float>(cameraH_) * 0.5f) *
                            static_cast<double>(pixelToMmY_);
            regMark_[regMarkActive_]     = p;
            regMarkHave_[regMarkActive_] = true;
            if (regMarkActive_ + 1 < regMarkCount_) ++regMarkActive_;
            regTeachMsg_.clear();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Clear", ImVec2(80.0f, 0.0f))) {
            for (int i = 0; i < kRegMarkMax; ++i) regMarkHave_[i] = false;
            regMarkActive_ = 0;
        }

        ImGui::TextDisabled("%d of %d marked", regMarked, regMarkCount_);
    }

    // ---- Step 4: complete ----------------------------------------------------
    stepHeader("Step 4  Compute the tip", step4Done);
    {
        ImGui::TextDisabled("The centre of the corners is the tip in XY; the view's");
        ImGui::TextDisabled("focused Z is the tip in Z.");
        ImGui::BeginDisabled(busy || sameDev);
        if (ImGui::Button("Compute and save tip", ImVec2(180.0f, 0.0f)))
            CompleteDeviceRegTip();
        ImGui::EndDisabled();
        if (reg.valid)
            ImGui::TextDisabled("tip taught %s from %d picks - computing again replaces it",
                                reg.tipAt.empty() ? "(undated)" : reg.tipAt.c_str(),
                                reg.tipPoints);
    }
    ImGui::EndTabItem();
    }

    // ==== Tab: Verify and test - the check and the two drives =================
    // Green once the check has PASSED, never on a teach alone: this is the one
    // tab whose "done" is a measurement rather than a capture.
    if (tabBegin("Verify and test", reg.verified)) {
    ImGui::Spacing();

    // ---- Check ---------------------------------------------------------------
    ImGui::SeparatorText("Check");
    ImGui::TextWrapped("Drives to a pose AWAY from the one the tip was taught at, "
                       "predicts the tip and measures how far off it is. With the "
                       "rotation coming from the axis transforms, this is the only "
                       "thing that ever measures what that assumption costs.");
    {
        ImGui::SetNextItemWidth(70.0f);
        ImGui::InputDouble("dX", &regCheckDx_, 0.0, 0.0, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70.0f);
        ImGui::InputDouble("dY", &regCheckDy_, 0.0, 0.0, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70.0f);
        ImGui::InputDouble("dZ", &regCheckDz_, 0.0, 0.0, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("tol", &regCheckTol_, 0.0, 0.0, "%.4f");
        if (regCheckTol_ < 0.001) regCheckTol_ = 0.001;

        const double offset = std::sqrt(regCheckDx_ * regCheckDx_ +
                                        regCheckDy_ * regCheckDy_ +
                                        regCheckDz_ * regCheckDz_);
        if (offset <= 0.0)
            ImGui::TextColored(kWarn, "all zero - this would check the very pose the tip\n"
                                      "was taught at, which returns the measurement by\n"
                                      "construction and can verify nothing.");
        else
            ImGui::TextDisabled("checks %.3f mm from the taught pose", offset);

        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("v travel", &regVTravel_, 0.0, 0.0, "%.2f");
        if (regVTravel_ < 0.0) regVTravel_ = 0.0;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("v jog", &regVJog_, 0.0, 0.0, "%.2f");
        if (regVJog_ < 0.0) regVJog_ = 0.0;

        // Off by default, and worth the paragraph: the tip Z in the record is a
        // HAND-focused Z from step 2, so a check that focuses by sweep measures
        // how far the two methods disagree - 0.15 mm on a flat coupon is normal -
        // and reports it as a registration error. With it off, the operator
        // focuses the same way they taught, watching the live dZ in the pick
        // window until it closes.
        ImGui::Checkbox("autofocus at the check pose", &regAutofocus_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Off: the pick window shows the live dX/dY/dZ against the\n"
                              "prediction and you focus by hand - the same way step 2\n"
                              "taught the tip Z.\n"
                              "On: a sweep sets Z before the pick. Only trust it if the\n"
                              "camera view in step 2 was captured after a sweep too.");
        ImGui::SameLine();
        ImGui::BeginDisabled(!regAutofocus_);
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("focus mm", &regFocusRangeMm_, 0.0, 0.0, "%.2f");
        ImGui::EndDisabled();
        if (regFocusRangeMm_ < 0.01) regFocusRangeMm_ = 0.01;

        const bool running = orch_.lua.Running();
        // Every Run surface gates on the WHOLE orchestration, never on one
        // engine: the program runner goes momentarily idle between queue items,
        // and a check on the engines alone lets a second run slip in.
        ImGui::BeginDisabled(busy || !reg.valid);
        if (ImGui::Button("Check registration", ImVec2(180.0f, 0.0f)))
            StartDeviceRegCheckRun();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
        ImGui::EndDisabled();
        if (!reg.valid)
            ImGui::TextDisabled("nothing to check until the tip is taught");
        if (!regStartErr_.empty())
            ImGui::TextColored(kBad, "%s", regStartErr_.c_str());
        if (running) {
            const LuaEngine::Status lst = orch_.lua.GetStatus();
            ImGui::TextDisabled("%s  %s", LuaEngine::StateName(lst.state),
                                lst.error.empty() ? "" : lst.error.c_str());
        }
    }

    // ---- Send the tool here --------------------------------------------------
    // The consuming end of the registration, on the page that taught it: drive
    // the registered device so its TIP lands where the reference device is
    // standing right now - the point under the camera. It is the same solve
    // move_to_point does (h = R^T (p - t)), and it belongs here because a
    // registration that has never been driven through has never been seen to
    // work: reading a residual off a log is not the same as watching the tool
    // arrive where the camera is looking.
    //
    // It stays an APPROACH. Nothing here closes a loop, so it must never be
    // read as a placement - the rotation is typed, the tip is one point, and
    // the last 20 um belongs to placement_procedure.lua measuring and
    // correcting.
    ImGui::SeparatorText("Send the tool here");
    {
        double rx = 0.0, ry = 0.0, rz = 0.0;
        bool   haveRefXYZ = false;
        std::map<std::string, double> refPose;
        if (workflow_.DevicePose(regRef_, refPose)) {
            const auto ix = refPose.find("X"), iy = refPose.find("Y"),
                       iz = refPose.find("Z");
            if (ix != refPose.end() && iy != refPose.end() && iz != refPose.end()) {
                rx = ix->second; ry = iy->second; rz = iz->second;
                haveRefXYZ = true;
            }
        }

        ImGui::TextWrapped("Drives '%s' so its tip lands where '%s' is standing now - "
                           "the point under the camera. The same solve a recipe's "
                           "move_to_point step does.", regDevice_, regRef_);

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
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("max mm", &regSendMaxMm_, 0.0, 0.0, "%.1f");
        if (regSendMaxMm_ < 0.0) regSendMaxMm_ = 0.0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Refuses a move longer than this. A stale record turns a\n"
                              "short approach into a lunge, and that is the failure this\n"
                              "button has to survive. 0 = no ceiling.");
        ImGui::SameLine();
        ImGui::Checkbox("allow unverified", &regSendUnver_);

        // Everything the move needs, computed live so the destination can be
        // read BEFORE anything is armed. The same three refusals move_to_point
        // makes, in the same order: a second opinion about when a registration
        // may be driven through is how two surfaces start disagreeing.
        const RegVec3 target{rx, ry, rz + regSendZOff_};
        const RegVec3 h = DeviceRegSolve(reg, target);
        std::string   refuse;
        double        travel     = 0.0;
        bool          haveTravel = false;
        if (!reg.valid)       refuse = "no registration to drive through";
        else if (!haveRefXYZ) refuse = std::string("no live X/Y/Z from '") + regRef_ + "'";
        else if (!reg.verified && !regSendUnver_)
            refuse = "UNVERIFIED - run the check, or tick allow unverified and watch it";
        std::map<std::string, double> devPose;
        if (refuse.empty() && workflow_.DevicePose(regDevice_, devPose)) {
            // A hexapod's tip swings about its pivot, so a record taken at one
            // set of rotations describes no other. Same guard as the command.
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
                travel     = std::sqrt(dx * dx + dy * dy + dz * dz);
                haveTravel = true;
            }
        }
        if (refuse.empty() && haveTravel && regSendMaxMm_ > 0.0 &&
            travel > regSendMaxMm_) {
            char b[128];
            std::snprintf(b, sizeof(b), "%.3f mm of travel exceeds the %.1f mm ceiling",
                          travel, regSendMaxMm_);
            refuse = b;
        }

        if (haveRefXYZ)
            ImGui::TextDisabled("tip to (%.4f, %.4f, %.4f) in '%s' space",
                                target.x, target.y, target.z, regRef_);
        if (reg.valid && haveRefXYZ) {
            if (haveTravel)
                ImGui::Text("puts '%s' at (%.4f, %.4f, %.4f), %.3f mm of travel",
                            regDevice_, h.x, h.y, h.z, travel);
            else
                ImGui::Text("puts '%s' at (%.4f, %.4f, %.4f)", regDevice_,
                            h.x, h.y, h.z);
        }
        if (!refuse.empty()) ImGui::TextColored(kWarn, "%s", refuse.c_str());

        // Armed like every other surface that moves something, and gated on
        // the WHOLE orchestration rather than on one engine.
        ImGui::BeginDisabled(busy || sameDev || !refuse.empty());
        if (ImGui::Button("Send tool here", ImVec2(180.0f, 0.0f))) {
            FD_WARN("devreg", "sending '%s' to the '%s' point (%.4f, %.4f, %.4f): "
                              "pose (%.4f, %.4f, %.4f), %.3f mm of travel%s",
                    regDevice_, regRef_, target.x, target.y, target.z,
                    h.x, h.y, h.z, travel,
                    reg.verified ? "" : " - THE REGISTRATION IS UNVERIFIED");
            driveTo(regDevice_, h);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !reg.hasImprintPose);
        if (ImGui::Button("Back to imprint", ImVec2(140.0f, 0.0f)))
            driveTo(regDevice_, reg.imprintPose);
        ImGui::EndDisabled();
        if (!jogArmed_)
            ImGui::TextDisabled("arm jog (Jog window) - this drives a device");
    }

    // ---- Look at the tool ----------------------------------------------------
    // The inverse of the button above, and the one that costs nothing: leave the
    // tool where it is and drive the CAMERA until its centre is on the tip.
    //
    // It belongs on this page because it is how a registration is READ. The
    // overlay draws a cross where the record says the tip is; this puts the
    // camera there, so the operator sees the tool itself next to the cross that
    // claims to be on it. A registration that is out by half a millimetre is
    // invisible in a residual and obvious in that picture.
    //
    // The tip comes from Workflow::DeviceRegTip - the SAME predictor the overlay
    // and the tip topics use, refusals and all - never a second prediction here.
    ImGui::SeparatorText("Look at the tool");
    {
        ImGui::TextWrapped("Drives '%s' so the camera centre lands on '%s's tip, "
                           "predicted from where it is standing now. The tool does "
                           "not move. Same step as a recipe's move_camera_to_tool.",
                           regRef_, regDevice_);

        RegVec3     tip;
        std::string refuse;
        const bool  haveTip = workflow_.DeviceRegTip(regDevice_, tip, &refuse);
        if (haveTip) refuse.clear();

        // Where the camera stands, and so how far it would go. XY only unless Z
        // is asked for, which is also exactly what gets commanded.
        double cx = 0.0, cy = 0.0, cz = 0.0;
        bool   haveCam = false;
        {
            std::map<std::string, double> refPose;
            if (workflow_.DevicePose(regRef_, refPose)) {
                const auto ix = refPose.find("X"), iy = refPose.find("Y"),
                           iz = refPose.find("Z");
                if (ix != refPose.end() && iy != refPose.end() && iz != refPose.end()) {
                    cx = ix->second; cy = iy->second; cz = iz->second;
                    haveCam = true;
                }
            }
        }

        // Z is opt-in. The tip's reference Z is by construction the height that
        // FOCUSES on it, so commanding it is the useful thing - and it is also
        // the leg that drives the lens down toward a tool of unknown height, so
        // it must never be a side effect of asking to look at something.
        ImGui::Checkbox("focus Z too", &regLookZ_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Off: only X and Y move, and whatever the camera is\n"
                              "focused on now stays in focus.\n"
                              "On: Z goes to the height that focuses ON THE TIP -\n"
                              "which is the camera moving toward the tool. Watch it.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("look max mm", &regLookMaxMm_, 0.0, 0.0, "%.1f");
        if (regLookMaxMm_ < 0.0) regLookMaxMm_ = 0.0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Refuses a move longer than this. Wider than the tool's\n"
                              "ceiling above, because this crosses the workspace to a\n"
                              "parked tool rather than descending the last few mm.\n"
                              "0 = no ceiling.");

        double travel     = 0.0;
        bool   haveTravel = false;
        if (haveTip && haveCam) {
            const double dx = tip.x - cx, dy = tip.y - cy;
            const double dz = regLookZ_ ? tip.z - cz : 0.0;
            travel     = std::sqrt(dx * dx + dy * dy + dz * dz);
            haveTravel = true;
        }
        if (refuse.empty() && !haveCam)
            refuse = std::string("no live X/Y/Z from '") + regRef_ + "'";
        if (refuse.empty() && haveTravel && regLookMaxMm_ > 0.0 && travel > regLookMaxMm_) {
            char b[128];
            std::snprintf(b, sizeof(b), "%.3f mm of travel exceeds the %.1f mm ceiling",
                          travel, regLookMaxMm_);
            refuse = b;
        }

        if (haveTip) {
            ImGui::Text("tip at (%.4f, %.4f, %.4f) in '%s' space", tip.x, tip.y, tip.z,
                        regRef_);
            if (haveTravel)
                ImGui::TextDisabled("%.3f mm of travel%s", travel,
                                    regLookZ_ ? "" : " (XY only)");
        }
        // No verified gate, deliberately - and move_camera_to_tool makes the
        // identical judgement, so the two surfaces still answer as one. That
        // gate asks whether a record nothing has checked may move a TOOL;
        // nothing here touches the part, the camera crosses its own workspace,
        // and LOOKING is how an operator finds out what an unverified record
        // costs. Refusing it would hide the very error the picture exists to
        // show. The rotation guard and the ceiling still apply: a tip that
        // cannot be predicted has no position to drive to.
        if (!refuse.empty()) ImGui::TextColored(kWarn, "%s", refuse.c_str());
        else if (!reg.verified)
            ImGui::TextDisabled("unverified - so the cross may not land on the tool;\n"
                                "that disagreement is what this is for");

        ImGui::BeginDisabled(busy || sameDev || !refuse.empty());
        if (ImGui::Button("Look at the tool", ImVec2(180.0f, 0.0f))) {
            FD_INFO("devreg", "'%s' to the '%s' tip at (%.4f, %.4f, %.4f), %s, "
                              "%.3f mm of travel",
                    regRef_, regDevice_, tip.x, tip.y, tip.z,
                    regLookZ_ ? "XYZ" : "XY only", travel);
            if (regLookZ_) driveAxes(regRef_, {{"X", tip.x}, {"Y", tip.y}, {"Z", tip.z}});
            else           driveAxes(regRef_, {{"X", tip.x}, {"Y", tip.y}});
        }
        ImGui::EndDisabled();
        if (!jogArmed_)
            ImGui::TextDisabled("arm jog (Jog window) - this drives a device");
    }
    ImGui::EndTabItem();
    }

    ImGui::EndTabBar();
    ImGui::EndChild();
}

} // namespace firedog
