// PageGrip.cpp : the Gripped Object page (a full-page takeover). Definitions of
// App members — see App.h.
//
// The question it answers: a gripper has picked something up — where in the jaws
// did it land? The device registration says where the TOOL is; this measures
// where the OBJECT is, and the difference is the grip error.
//
//     object  =  the centre marked (or detected) through the camera
//     tip     =  Workflow::DeviceRegTip(device)
//     offset  =  object - tip
//
// Both in the reference device's axis space, so the origin cancels and what is
// left is a vector — the only thing that crosses to a device whose origin has
// never been related to the camera's. See docs/gripped-object-design.md.
//
// ---- Why the page exists at all ---------------------------------------------
//
// The measuring is a recipe step (`grip_measure`), so this page is not where the
// work happens. What it is for is the two things a recipe cannot do:
//
//   * TEACHING. Adopting a reference offset means an operator looking at a grip
//     they believe is good and saying so. A run that promoted its own
//     measurement to the standard would make every grip a good grip, and the
//     number would drift a little further every shift with nothing to notice it.
//   * SEEING. The image carries the marks, the midpoint AND the registered tip
//     side by side, projected through the same pose. Half a millimetre of grip
//     error is a number in a log and a glaring picture here.
//
// The Measure button launches the SAME shipped script the recipe step runs, for
// the reason the Alignment and Placement pages do: the procedure an operator
// taught and the one that runs at three in the morning have to be one thing.
#include "App.h"

#include "imgui.h"

#include "command/ExecutionContext.h"   // SystemScriptPath - the standard procedure
#include "core/DeviceReg.h"
#include "core/Log.h"
#include "core/Product.h"               // GenerateUid / NowStamp
#include "ui/Widgets.h"
#include "vision/HalconDetector.h"      // Available() - detection needs a licence

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

// The private inspect pose, named after its owner and hidden — the third user of
// the convention `algn_<procUid>_<n>` established (PageAlign.cpp). The name IS
// the ownership record: no schema change, and an orphan can always be traced
// back to the profile that made it.
//
// "grip_" + 8 + "_i" = 15 chars, inside SavedPos::name's 32.
std::string GripPosName(const std::string& gripUid)
{
    return "grip_" + gripUid + "_i";
}

} // namespace

// ---- Maintenance the ownership implies -------------------------------------

// Deleting a profile drops its private pose. A prefix sweep like the alignment
// one, though this owner has exactly one child today: the profile is what the
// name says it belongs to, and a second private pose added later must not need
// this function rewritten to be cleaned up.
void App::DropGripPositions(const GripProfile& g)
{
    if (g.device.empty()) return;
    const auto it = session_.Positions().find(g.device);
    if (it == session_.Positions().end()) return;
    std::vector<SavedPos>& list = it->second;
    const std::size_t before = list.size();
    const std::string prefix = "grip_" + g.uid + "_";
    for (std::size_t i = list.size(); i-- > 0;) {
        if (std::string(list[i].name).rfind(prefix, 0) == 0)
            list.erase(list.begin() + static_cast<long>(i));
    }
    if (list.size() != before) {
        session_.SavePositions();
        FD_INFO("grip", "'%s': dropped %zu private position(s)", g.name.c_str(),
                before - list.size());
    }
}

// ---- Launching the standard procedure ---------------------------------------

// The SHIPPED file, always — the same rule ExecutionContext::ResolveScript
// applies for grip_measure. A product that forked it would go on running
// whatever order of legs it was copied at, and the symptom would look like a bad
// grip rather than like a missing step.
void App::StartGripMeasureRun(const GripProfile& g)
{
    const std::string script = SystemScriptPath("grip_measure.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"grip_profile_uid",   g.uid},        // uid, not name: a rename mid-run is harmless
        {"grip_go_to_inspect", gripRunInspect_},
        {"grip_look",          gripRunLook_},
        {"grip_focus_z",       gripFocusZ_},
        {"grip_autofocus",     gripAutofocus_},
        {"grip_focus_range_mm", gripFocusRangeMm_},
        {"grip_settle_ms",     static_cast<double>(300)},
        // The page never fails on a verdict: it is a teaching surface, and the
        // whole point of running it here is to LOOK at a measurement that may be
        // out. A recipe step defaults the other way.
        {"grip_require_ok",    false},
        {"grip_prefix",        std::string("grip")},
    };
    gripStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &gripStartErr_))
        FD_WARN("grip", "grip measurement rejected: %s", gripStartErr_.c_str());
}

// ---- The page ---------------------------------------------------------------

void App::DrawGripPanel()
{
    std::vector<GripProfile>& list = session_.Grips();
    const bool busy = orch_.Busy();

    if (gripSel_ >= static_cast<int>(list.size())) gripSel_ = list.empty() ? -1 : 0;
    GripProfile* g = (gripSel_ >= 0) ? &list[static_cast<std::size_t>(gripSel_)] : nullptr;

    UpdateCameraTexture();
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;

    // The registration behind the selected profile, and the tip it predicts.
    // Read every frame through layer 2's ONE predictor, refusals and all: a page
    // that computed its own would be a second answer to where the tool is.
    DeviceReg   reg;
    RegVec3     tip;
    bool        haveTip = false;
    std::string tipWhy;
    if (g && !g->device.empty()) {
        reg = workflow_.DeviceRegFor(g->device);
        haveTip = workflow_.DeviceRegTip(g->device, tip, &tipWhy);
    }

    // The pose the marks are measured against: the device carrying the CAMERA,
    // which the registration names. A mark is a feature seen through a lens, so
    // it is relative to where the lens is.
    const std::string camName = reg.valid ? reg.refDevice : workflow_.GantryName();
    double gx = 0.0, gy = 0.0;
    bool   havePose = false;
    if (!camName.empty()) {
        std::map<std::string, double> pose;
        if (workflow_.DevicePose(camName, pose)) {
            const auto ix = pose.find("X"), iy = pose.find("Y");
            if (ix != pose.end() && iy != pose.end()) {
                gx = ix->second;
                gy = iy->second;
                havePose = true;
            }
        }
    }

    // Driving is still driving: gated on the jog arm like every other surface
    // that moves something, and issued by layer 2.
    const auto driveAxes = [&](const std::string& devName,
                               const std::vector<std::pair<std::string, double>>& targets) {
        if (!jogArmed_) {
            gripTeachMsg_ = "arm jog first - this drives a device";
            return;
        }
        // Layer 2 owns every move and it takes an INDEX, while this page's
        // combos hold names. Resolved inline rather than through a shared
        // helper: the loop is four lines and the alternative is device access
        // in ui/Widgets, which that header exists to keep out.
        int idx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i)
            if (hardware_.MotionName(i) == devName) { idx = static_cast<int>(i); break; }
        if (idx < 0) {
            gripTeachMsg_ = "'" + devName + "' is not in the motion fleet";
            return;
        }
        gripTeachMsg_.clear();
        workflow_.MoveToPosition(static_cast<std::size_t>(idx), targets);
    };

    // ---- Layout: image left, controls right, draggable splitter -------------
    // The same three constants and the same drag-end persistence the
    // Registration page uses; this page carries the same kind of explanatory
    // text and the operator's font size moves all of it.
    constexpr float kSplitW  = 6.0f;
    constexpr float kMinImgW = 240.0f;
    constexpr float kMinCtlW = 260.0f;

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (gripCtlW_ < kMinCtlW) gripCtlW_ = kMinCtlW;
    const float maxCtlW = avail.x - kSplitW - kMinImgW;
    if (gripCtlW_ > maxCtlW) gripCtlW_ = fmaxf(kMinCtlW, maxCtlW);
    const float rightW = gripCtlW_;
    const float imgW   = fmaxf(40.0f, avail.x - rightW - kSplitW);

    ImGui::BeginChild("gripImg", ImVec2(imgW, avail.y), ImGuiChildFlags_Borders);
    {
        CameraMark marks[kGripMarkMax];
        constexpr const char* kNames[kGripMarkMax] = {"pt1", "pt2"};
        for (int i = 0; i < kGripMarkMax; ++i) {
            marks[i].wx    = gripMark_[i].wx;
            marks[i].wy    = gripMark_[i].wy;
            marks[i].have  = gripMarkHave_[i];
            marks[i].label = kNames[i];
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
        mp.markCount  = kGripMarkMax;
        // The midpoint IS the object's centre - the thing the two marks exist to
        // define - so it is drawn, unlike the registration page's four corners
        // where a centre of the imprint is what the maths produces later.
        mp.midpointLabel = "object";
        // The footprint's size for the picker, seeded from the registration.
        // Locals, not App state: the box is read-only here (no drag flag), so
        // there is nothing to carry between frames.
        float boxW = 0.0f, boxH = 0.0f;
        // ...and the TIP beside it, which is what the centre is measured
        // against. Drawn differently on purpose: a mark is a thing somebody
        // measured, a tip is a thing the machine computed out of a registration,
        // and reading the second as the first is how a rotated hexapod goes on
        // looking placed.
        if (haveTip) {
            mp.toolHave  = true;
            mp.toolWx    = tip.x;
            mp.toolWy    = tip.y;
            mp.toolLabel = g->device.c_str();
            // ...and the jaws around it, at the size the registration holds.
            // Read-only on this page: the footprint is machine state and is
            // taught on the Registration page.
            boxW      = static_cast<float>(reg.toolWmm);
            boxH      = static_cast<float>(reg.toolLmm);
            mp.boxWmm = &boxW;
            mp.boxHmm = &boxH;
            // ...turned by the device's live U, as the jaws are.
            std::map<std::string, double> devPose;
            if (workflow_.DevicePose(g->device, devPose))
                if (const auto iu = devPose.find("U"); iu != devPose.end())
                    mp.boxDeg = static_cast<float>(iu->second);
        }
        EnsureMarkIcons();
        mp.icons[0]   = markIcons_[0];
        mp.icons[1]   = markIcons_[1];
        mp.activeMark = (g && g->mode == "auto") ? -1 : gripMarkActive_;
        mp.px         = &gripPx_;
        mp.py         = &gripPy_;
        mp.zoom       = &gripZoom_;

        const MarkPickerResult r =
            DrawMarkPicker("##gripInput", ImGui::GetContentRegionAvail(), mp);
        if (r.moveRequested) {
            if (jogArmed_) workflow_.MoveGantryRelativeMM(r.dxMM, r.dyMM);
            else FD_WARN("grip", "click-to-move ignored - arm jog first");
        }
    }
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##gripsplit", true, kSplitW, avail.y)) {
        gripCtlW_ -= ImGui::GetIO().MouseDelta.x;
        gripCtlSplitting_ = true;
    } else if (gripCtlSplitting_) {
        gripCtlSplitting_ = false;
        SaveUiSettings();          // persist on drag END, like every other splitter
    }
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginChild("gripCtl", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);

    // ---- The profiles -------------------------------------------------------
    ImGui::SeparatorText("Profiles");
    {
        ImGui::BeginDisabled(busy);
        if (ImGui::SmallButton("New")) {
            GripProfile n;
            n.uid  = GenerateUid();
            n.name = "object " + std::to_string(list.size() + 1);
            // Default the gripper to the first REGISTERED device that is not the
            // camera's: without a registration this page can measure nothing, so
            // offering one that works beats an empty combo.
            for (const auto& [name, r] : workflow_.DeviceRegs()) {
                if (r.valid) { n.device = name; break; }
            }
            list.push_back(std::move(n));
            gripSel_ = static_cast<int>(list.size()) - 1;
            g = &list.back();
            for (int i = 0; i < kGripMarkMax; ++i) gripMarkHave_[i] = false;
            gripMarkActive_ = 0;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!g);
        if (ImGui::SmallButton("Duplicate") && g) {
            const GripProfile src = *g;   // by value: the push_back below reallocates
            GripProfile copy = src;
            copy.uid   = GenerateUid();   // a copy is a NEW identity...
            copy.name += " copy";
            // ...and it has not been taught. The reference offset describes one
            // object in one gripper; carrying it over would hand the copy a
            // standard nobody ever measured for it.
            copy.haveRef = false;
            copy.refDx = copy.refDy = 0.0;
            copy.taughtAt.clear();
            copy.lastDx = copy.lastDy = 0.0;
            copy.measuredAt.clear();
            copy.inspectUid.clear();

            // Re-parent the private pose under the new owner, so the copy is
            // self-contained rather than sharing a row the original's Delete
            // would take away.
            if (!src.inspectUid.empty() && !src.device.empty()) {
                std::vector<SavedPos>& pos = session_.Positions()[src.device];
                for (std::size_t i = 0; i < pos.size(); ++i) {
                    if (pos[i].uid != src.inspectUid) continue;
                    SavedPos np = pos[i];   // copy BEFORE the push_back reallocates
                    np.uid = GenerateUid();
                    std::snprintf(np.name, sizeof(np.name), "%s",
                                  GripPosName(copy.uid).c_str());
                    pos.push_back(std::move(np));
                    copy.inspectUid = pos.back().uid;
                    break;
                }
                session_.SavePositions();
            }
            CopyGripDetector(src, copy);   // patch file included
            list.push_back(std::move(copy));
            gripSel_ = static_cast<int>(list.size()) - 1;
            g = &list.back();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete") && g) {
            DropGripPositions(*g);
            DropGripDetector(*g);
            list.erase(list.begin() + gripSel_);
            gripSel_ = list.empty() ? -1 : 0;
            g = list.empty() ? nullptr : &list[0];
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!session_.GripsDirty());
        // Stable id, changing label: an ImGui id derives from the string, and a
        // button that gets a new one when the document goes dirty loses the
        // click that made it dirty.
        if (ImGui::SmallButton(session_.GripsDirty() ? "Save *###gripsave"
                                                     : "Save###gripsave"))
            session_.SaveGrips();
        ImGui::EndDisabled();
        ImGui::EndDisabled();

        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginListBox("##griplist", ImVec2(0.0f, 90.0f))) {
            for (int i = 0; i < static_cast<int>(list.size()); ++i) {
                ImGui::PushID(i);
                if (ImGui::Selectable(list[static_cast<std::size_t>(i)].name.c_str(),
                                      i == gripSel_)) {
                    gripSel_ = i;
                    for (int k = 0; k < kGripMarkMax; ++k) gripMarkHave_[k] = false;
                    gripMarkActive_ = 0;
                    gripTeachMsg_.clear();
                }
                ImGui::PopID();
            }
            ImGui::EndListBox();
        }
    }

    if (!g) {
        ImGui::TextWrapped("No profile selected. A profile says how to measure where "
                           "one kind of object sits in one gripper's jaws - the pose "
                           "to hold it at, how to find it, how wide it should be, and "
                           "how far off it may be. A recipe's grip_measure step names "
                           "one.");
        ImGui::EndChild();
        return;
    }

    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s", g->name.c_str());
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::InputText("##gripname", buf, sizeof(buf))) g->name = buf;
    }

    // ---- 1  The gripper ------------------------------------------------------
    ImGui::SeparatorText("1  Gripper");
    {
        ImGui::SetNextItemWidth(180.0f);
        if (ImGui::BeginCombo("holds it", g->device.empty() ? "(pick)" : g->device.c_str())) {
            for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
                const std::string n = hardware_.MotionName(i);
                if (ImGui::Selectable(n.c_str(), n == g->device)) {
                    if (n != g->device) {
                        // The private pose belongs to the OLD device's list, so
                        // it cannot follow: drop it rather than leave a hidden
                        // row nobody will ever find.
                        DropGripPositions(*g);
                        g->inspectUid.clear();
                        g->haveRef = false;   // a different arm is a different grip
                        g->taughtAt.clear();
                    }
                    g->device = n;
                }
            }
            ImGui::EndCombo();
        }

        // The registration is the whole foundation, so its state is stated here
        // rather than left for a run to discover.
        if (g->device.empty()) {
            ImGui::TextColored(kWarn, "pick the device holding the object");
        } else if (!reg.valid) {
            ImGui::TextColored(kBad, "'%s' is NOT REGISTERED", g->device.c_str());
            ImGui::TextWrapped("Nothing knows where its tool is, and an object's "
                               "position only means anything as an offset from that "
                               "tool. Teach it on the Registration page (F8).");
        } else {
            ImGui::TextColored(reg.verified ? kGood : kWarn,
                               "registered to %s%s", reg.refDevice.c_str(),
                               reg.verified ? ", verified" : ", UNVERIFIED");
            if (haveTip)
                ImGui::Text("tip now  %+.4f  %+.4f  %+.4f mm", tip.x, tip.y, tip.z);
            else
                ImGui::TextColored(kWarn, "no tip: %s", tipWhy.c_str());
            // When the tip was last measured, because that date is what can
            // invalidate a reference offset without anything here noticing. A
            // constant registration error CANCELS between the reference and a
            // measurement - both subtract the same predicted tip - but a tip
            // re-taught since the reference was adopted moves one of the two.
            ImGui::TextDisabled("tip taught %s", reg.tipAt.empty() ? "(undated)"
                                                                   : reg.tipAt.c_str());
            if (g->haveRef && !g->taughtAt.empty() && !reg.tipAt.empty() &&
                reg.tipAt > g->taughtAt)
                ImGui::TextColored(kBad, "the tip was re-taught AFTER this reference\n"
                                         "offset - re-adopt it on a known-good grip");
        }
    }

    // ---- 2  The inspect pose -------------------------------------------------
    ImGui::SeparatorText("2  Inspect pose");
    ImGui::TextWrapped("Where to hold the object so the camera can see it. It carries "
                       "the ROTATIONS as well as XYZ, which is what makes two "
                       "measurements comparable: a gripper turned since the teach "
                       "swings the object and reads as a mis-grip.");
    {
        const SavedPos* ip = nullptr;
        if (!g->device.empty() && !g->inspectUid.empty()) {
            const auto it = session_.Positions().find(g->device);
            if (it != session_.Positions().end())
                for (const SavedPos& sp : it->second)
                    if (sp.uid == g->inspectUid) { ip = &sp; break; }
        }

        ImGui::BeginDisabled(busy || g->device.empty());
        if (ImGui::Button(ip ? "Re-teach here" : "Teach here", ImVec2(140.0f, 0.0f))) {
            // CapturePrivatePose updates IN PLACE when the name already exists,
            // so a re-teach keeps the uid and whatever references it.
            const std::string uid = CapturePrivatePose(g->device, GripPosName(g->uid));
            if (uid.empty()) {
                gripTeachMsg_ = "'" + g->device + "' is not connected";
            } else {
                g->inspectUid = uid;
                session_.SaveGrips();
                gripTeachMsg_.clear();
                FD_INFO("grip", "'%s': inspect pose taught on %s", g->name.c_str(),
                        g->device.c_str());
            }
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !ip);
        if (ImGui::Button("Go to inspect", ImVec2(140.0f, 0.0f)) && ip) {
            // A taught position is addressed by uid, so the watched-move path
            // resolves it: it keeps only the axes this device actually has, and
            // it raises the MOVING bar with a Stop button. driveAxes below is
            // for the computed points, which have no uid to resolve.
            if (!jogArmed_) {
                gripTeachMsg_ = "arm jog first - this drives a device";
            } else if (!StartWatchedMove(g->device, g->inspectUid,
                                         g->name + " inspect pose")) {
                gripTeachMsg_ = "the move was refused - is " + g->device + " connected?";
            } else {
                gripTeachMsg_.clear();
            }
        }
        ImGui::EndDisabled();

        if (ip) {
            std::string line;
            for (const auto& [axis, v] : ip->vals) {
                char b[48];
                std::snprintf(b, sizeof(b), "%s%s %.3f", line.empty() ? "" : "  ",
                              axis.c_str(), v);
                line += b;
            }
            ImGui::TextDisabled("%s", line.c_str());
        } else {
            ImGui::TextColored(kWarn, "none taught - a run will measure the tool");
            ImGui::TextColored(kWarn, "wherever it finds it");
        }
    }

    // ---- 3  Look at it -------------------------------------------------------
    ImGui::SeparatorText("3  Look at it");
    ImGui::TextWrapped("Drives '%s' so the camera centre lands on the tool's tip - the "
                       "same move a run makes with move_camera_to_tool. The gripper "
                       "does not move.",
                       camName.empty() ? "(no camera device)" : camName.c_str());
    {
        ImGui::Checkbox("focus Z too", &gripFocusZ_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Off: only X and Y move.\n"
                              "On: Z goes to the height that focuses ON THE TIP,\n"
                              "which is the camera moving toward the tool. Watch it.");

        ImGui::BeginDisabled(busy || !haveTip || camName.empty());
        if (ImGui::Button("Camera to tool", ImVec2(140.0f, 0.0f))) {
            if (gripFocusZ_)
                driveAxes(camName, {{"X", tip.x}, {"Y", tip.y}, {"Z", tip.z}});
            else
                driveAxes(camName, {{"X", tip.x}, {"Y", tip.y}});
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || camName.empty());
        if (ImGui::Button("Autofocus", ImVec2(120.0f, 0.0f))) StartQuickAutoFocus();
        ImGui::EndDisabled();
    }

    // ---- 4  How the object is found ------------------------------------------
    ImGui::SeparatorText("4  Find the object");
    {
        std::string why;
        const bool halcon = HalconDetector::Available(&why);
        const bool isAuto = (g->mode == "auto");

        // The mode is a CHOICE the operator can take back, not a consequence of
        // a model existing: a taught patch costs real time at the microscope and
        // choosing manual for one run must not destroy it.
        if (ImGui::RadioButton("Manual (2 marks)", !isAuto) && isAuto) {
            g->mode = "manual";
            session_.SaveGrips();
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Detect", isAuto) && !isAuto) {
            g->mode = "auto";
            session_.SaveGrips();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Locate the object with a taught detection model - one\n"
                              "point, its centre. Needs HALCON; the two-mark form is\n"
                              "a crosshair pick and never does.");

        if (isAuto) {
            ImGui::Indent();
            const VisionModel* dm = GripDetector(*g);
            ImGui::BeginDisabled(busy || !halcon);
            if (ImGui::Button(dm ? "Teach detection..." : "Set up detection..."))
                OpenVisionTeachForGrip(*g);
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (!halcon)  ImGui::TextColored(kBad, "%s", why.c_str());
            else if (!dm) ImGui::TextColored(kWarn, "nothing taught");
            else          ImGui::TextDisabled("%s", dm->method.c_str());
            if (dm) {
                PatchTex& pt = alignTmpl_[0];
                if (EnsurePatchTex(pt, *dm)) {
                    DrawPatchThumb(pt, 96.0f, 96.0f);
                    ImGui::SameLine();
                    ImGui::TextDisabled("the cross is the\npoint reported");
                }
            }
            ImGui::Unindent();

            if (ImGui::Checkbox("ask the operator when detection fails",
                                &g->autoFallbackManual))
                session_.SaveGrips();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("On, a failed detection prompts for the marks instead\n"
                                  "of failing the step - always with a warning in the\n"
                                  "log, so a run cannot look automatic while a human\n"
                                  "marks it.");
        } else {
            ImGui::TextWrapped("Mark two points on opposite edges: their MIDPOINT is "
                               "the object's centre.");
            for (int i = 0; i < kGripMarkMax; ++i) {
                ImGui::PushID(i);
                ImGui::RadioButton(i == 0 ? "1" : "2", gripMarkActive_ == i);
                if (ImGui::IsItemClicked()) gripMarkActive_ = i;
                ImGui::SameLine();
                if (gripMarkHave_[i])
                    ImGui::Text("%.4f  %.4f", gripMark_[i].wx, gripMark_[i].wy);
                else
                    ImGui::TextDisabled("(not marked)");
                ImGui::PopID();
            }
            ImGui::BeginDisabled(!haveImg || !havePose);
            if (ImGui::Button("Capture point", ImVec2(140.0f, 0.0f))) {
                // Machine mm = pose + (crosshair - image centre) x pixelToMm,
                // latched WITH the pose it was taken at, so the operator may jog
                // between the two marks and each still means the same physical
                // place. The identical formula every teach page uses.
                AlignPoint p;
                p.gx = gx;
                p.gy = gy;
                p.px = gripPx_;
                p.py = gripPy_;
                p.wx = gx + static_cast<double>(gripPx_ - static_cast<float>(cameraW_) * 0.5f) *
                                static_cast<double>(pixelToMmX_);
                p.wy = gy + static_cast<double>(gripPy_ - static_cast<float>(cameraH_) * 0.5f) *
                                static_cast<double>(pixelToMmY_);
                gripMark_[gripMarkActive_]     = p;
                gripMarkHave_[gripMarkActive_] = true;
                if (gripMarkActive_ + 1 < kGripMarkMax) ++gripMarkActive_;
                gripTeachMsg_.clear();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Clear", ImVec2(80.0f, 0.0f))) {
                for (int i = 0; i < kGripMarkMax; ++i) gripMarkHave_[i] = false;
                gripMarkActive_ = 0;
            }
            if (!havePose)
                ImGui::TextColored(kWarn, "no live pose from '%s'",
                                   camName.empty() ? "(none)" : camName.c_str());
        }
    }

    // ---- 5  What counts as right ---------------------------------------------
    ImGui::SeparatorText("5  Reference");
    {
        // No size field. The marks' separation is where the operator could see
        // an edge, not the object's width - see GripProfile.h.
        ImGui::SetNextItemWidth(90.0f);
        if (ImGui::InputDouble("offset tol mm", &g->tolMm, 0.0, 0.0, "%.4f"))
            if (g->tolMm < 0.0001) g->tolMm = 0.0001;

        // The live measurement, from whatever is on the page right now: the two
        // marks in manual mode, and in auto mode the last run's numbers, which
        // is all a page CAN show - a detection happens on a worker.
        double cx = 0.0, cy = 0.0, dx = 0.0, dy = 0.0;
        bool   haveMeas = false;
        if (g->mode != "auto" && gripMarkHave_[0] && gripMarkHave_[1] && haveTip) {
            cx = (gripMark_[0].wx + gripMark_[1].wx) * 0.5;
            cy = (gripMark_[0].wy + gripMark_[1].wy) * 0.5;
            dx = cx - tip.x;
            dy = cy - tip.y;
            haveMeas = true;
        } else if (!g->measuredAt.empty()) {
            dx = g->lastDx;
            dy = g->lastDy;
            haveMeas = true;
        }

        if (haveMeas) {
            ImGui::Text("offset  %+.4f  %+.4f mm", dx, dy);
            std::string why;
            const bool ok = GripVerdict(*g, dx, dy, &why);
            if (ok) ImGui::TextColored(kGood, "within tolerance");
            else    ImGui::TextColored(kWarn, "%s", why.c_str());
        } else {
            ImGui::TextDisabled("nothing measured yet");
        }

        ImGui::BeginDisabled(busy || !haveMeas);
        if (ImGui::Button("Adopt as the reference", ImVec2(190.0f, 0.0f))) {
            g->refDx    = dx;
            g->refDy    = dy;
            g->haveRef  = true;
            g->taughtAt = NowStamp();
            session_.SaveGrips();
            FD_INFO("grip", "'%s': reference offset adopted (%+.4f, %+.4f) mm",
                    g->name.c_str(), dx, dy);
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Says THIS grip is a good one. Everything a run judges is\n"
                              "measured against it, so adopt it on a part you have\n"
                              "actually checked - a reference adopted from a bad grip\n"
                              "makes every bad grip pass.");

        if (g->haveRef)
            ImGui::TextDisabled("reference (%+.4f, %+.4f) mm, %s", g->refDx, g->refDy,
                                g->taughtAt.empty() ? "(undated)" : g->taughtAt.c_str());
        else
            ImGui::TextColored(kWarn, "no reference - a run reports the offset\n"
                                      "and has no verdict to give");
        if (!g->measuredAt.empty())
            ImGui::TextDisabled("last run (%+.4f, %+.4f) mm, %s", g->lastDx, g->lastDy,
                                g->measuredAt.c_str());
    }

    // ---- Measure now ----------------------------------------------------------
    ImGui::SeparatorText("Measure");
    ImGui::TextWrapped("Runs the same procedure a recipe's grip_measure step runs: "
                       "the inspect pose, the camera onto the tool, then the marks. "
                       "It reports here rather than failing on a bad grip - looking "
                       "at one is the point.");
    {
        ImGui::Checkbox("go to the inspect pose", &gripRunInspect_);
        ImGui::Checkbox("drive the camera to the tool", &gripRunLook_);
        ImGui::Checkbox("autofocus over it", &gripAutofocus_);
        if (gripAutofocus_) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80.0f);
            if (ImGui::InputDouble("range mm", &gripFocusRangeMm_, 0.0, 0.0, "%.2f"))
                if (gripFocusRangeMm_ < 0.01) gripFocusRangeMm_ = 0.01;
        }

        const bool running = orch_.lua.Running();
        // Every Run surface gates on the WHOLE orchestration, never one engine:
        // the program runner goes momentarily idle between queue items, and a
        // check on the engines alone lets a second run slip in.
        ImGui::BeginDisabled(busy || !reg.valid || !jogArmed_);
        if (ImGui::Button("Measure now", ImVec2(160.0f, 0.0f))) StartGripMeasureRun(*g);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
        ImGui::EndDisabled();

        if (!jogArmed_) ImGui::TextDisabled("arm jog (Jog window) - this drives devices");
        if (!gripStartErr_.empty()) ImGui::TextColored(kBad, "%s", gripStartErr_.c_str());
        if (running) {
            const LuaEngine::Status st = orch_.lua.GetStatus();
            ImGui::TextDisabled("%s  %s", LuaEngine::StateName(st.state),
                                st.error.empty() ? "" : st.error.c_str());
        }
    }

    if (!gripTeachMsg_.empty()) ImGui::TextWrapped("%s", gripTeachMsg_.c_str());

    ImGui::Spacing();
    ImGui::TextWrapped("The offset is a DELTA in %s's axis space, which is why it can "
                       "cross to a device that has never shared an origin with the "
                       "camera. A recipe's grip_move_to_camera step subtracts it from "
                       "the tip's target, so the OBJECT lands under the camera; a "
                       "placement still measures and corrects from there.",
                       camName.empty() ? "the camera device" : camName.c_str());

    ImGui::EndChild();
}

} // namespace firedog
