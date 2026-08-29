// WndJog.cpp : the Jog window (View menu) and the velocity combo it shares
// with the Motion page. Definitions of App members — see App.h.
//
// The window has two tabs: "Jog" (the motion fleet) and "Camera" (the camera
// fleet — device combo + the Camera page's exposure controls, reused rather
// than copied so the seed/apply rule lives in one place, plus the live focus
// score and the Autofocus button).
//
// The Autofocus button does NOT sweep inline: it starts the `auto_focus`
// command as a one-step ad-hoc recipe on the engine worker, because the sweep
// blocks for tens of seconds and the UI thread may never block.
//
// Jog KINEMATICS are not here: the UI only picks a device, a step and a
// direction, and Workflow::JogGlobal maps that through the device's
// AxisTransform. Jog is armed via a checkbox and defaults OFF because the
// W/D-A/S-R/F keys drive real motion.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {
// Jog velocities (mm/s) per controller type.
constexpr double kVelPI[]  = {1.0, 2.0, 5.0, 10.0};
constexpr double kVelACS[] = {1.0, 5.0, 10.0, 20.0, 40.0};
} // namespace

void App::DrawJogWindow()
{
    // Spawns near the top-right (visible); drag it out to make it its own OS
    // window (multi-viewport). &showJog_ gives it a close box that syncs the menu.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + mv->Size.x - 262.0f, mv->Pos.y + 60.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250.0f, 640.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Jog", &showJog_)) {
        // Two tabs so the window covers both "move the stage" and "see it":
        // Jog = the motion fleet, Camera = the camera fleet (device + exposure).
        if (ImGui::BeginTabBar("JogTabs")) {
            if (ImGui::BeginTabItem("Jog")) {
                DrawJogPanel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Camera")) {
                DrawJogCameraTab();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

// Camera tab of the Jog window: pick the camera, set its exposure, watch the
// live focus score, run an autofocus sweep. The exposure widgets are the Camera
// page's (DrawCameraExposureControls), so both surfaces share one seeding rule
// and one apply path. Connect/disconnect stays on the Camera page — this tab
// only tunes a camera that is already up.
//
// The order is the bring-up order: get a camera, get the exposure right (the
// luma warning says when it is not), confirm the score responds, then let the
// sweep use it.
void App::DrawJogCameraTab()
{
    // Device combo. HardwareManager owns a single Pylon camera today, so this
    // lists one entry; the loop is what a second camera would grow into.
    const ConnState st = hardware_.CameraState();
    ImGui::TextUnformatted("Device");
    ImGui::SetNextItemWidth(-FLT_MIN);
    const std::string& camName = hardware_.CameraName();
    if (ImGui::BeginCombo("##camdev", camName.c_str())) {
        ImGui::Selectable(camName.c_str(), true);
        ImGui::EndCombo();
    }
    DeviceStatusLine(st, camName.c_str());

    ImGui::Separator();
    ImGui::TextUnformatted("Exposure");
    DrawCameraExposureControls();

    // ---- Live focus score ---------------------------------------------------
    // The same measure auto_focus maximises (Laplacian variance over a centred
    // ROI), sampled per frame while this is ticked. Two uses: sanity-check the
    // ROI and exposure before trusting a sweep, and watch the number peak while
    // jogging Z by hand. Off by default — it costs a score per frame.
    ImGui::Separator();
    ImGui::TextUnformatted("Focus");
    bool stream = workflow_.FocusStreaming();
    if (ImGui::Checkbox("Live score", &stream)) workflow_.SetFocusStreaming(stream, focusRoiPx_);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    constexpr int kRois[] = {0, 128, 256, 512, 1024};
    char roiLabel[24];
    if (focusRoiPx_ > 0) std::snprintf(roiLabel, sizeof(roiLabel), "%d px", focusRoiPx_);
    else                 std::snprintf(roiLabel, sizeof(roiLabel), "full frame");
    if (ImGui::BeginCombo("##focusroi", roiLabel)) {
        for (int r : kRois) {
            char b[24];
            if (r > 0) std::snprintf(b, sizeof(b), "%d px", r);
            else       std::snprintf(b, sizeof(b), "full frame");
            if (ImGui::Selectable(b, focusRoiPx_ == r)) {
                focusRoiPx_ = r;
                workflow_.SetFocusStreaming(workflow_.FocusStreaming(), focusRoiPx_);
            }
        }
        ImGui::EndCombo();
    }

    const Workflow::FocusSample f = workflow_.LatestFocus();
    if (!stream) {
        ImGui::TextDisabled("not sampling");
    } else if (!f.ok) {
        ImGui::TextDisabled("waiting for frames...");
    } else {
        ImGui::Text("%.1f", f.score);
        // Mean luma is the guard rail: a blown-out or black ROI makes the score
        // meaningless, and exposure is right above to fix it.
        if (f.meanLuma > 250.0 || f.meanLuma < 5.0)
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1.0f),
                               "luma %.0f - check exposure", f.meanLuma);
        else
            ImGui::TextDisabled("luma %.0f   frame %llu", f.meanLuma, f.frameSeq);
    }

    // ---- Autofocus (gantry-main Z, downward sweep) --------------------------
    // Real motion, so it follows the same arming rule as jog, and it gates on
    // orch_.Busy() like every other Run/Go surface. The sweep itself runs on the
    // engine worker as a one-step ad-hoc recipe — it blocks for tens of seconds,
    // which must never happen on the UI thread.
    ImGui::Separator();
    ImGui::TextUnformatted("Autofocus");
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragFloat("range mm", &afRangeMM_, 0.01f, 0.01f, 10.0f, "%.2f");
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragFloat("step mm", &afStepMM_, 0.001f, 0.001f, 1.0f, "%.3f");
    if (afDirIndex_ < 0 || afDirIndex_ >= kAfDirectionCount) afDirIndex_ = 0;
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::BeginCombo("direction", kAfDirections[afDirIndex_].label)) {
        for (int i = 0; i < kAfDirectionCount; ++i)
            if (ImGui::Selectable(kAfDirections[i].label, afDirIndex_ == i)) afDirIndex_ = i;
        ImGui::EndCombo();
    }

    const bool afBusy = orch_.Busy();
    const bool afReady = jogArmed_ && !afBusy && st == ConnState::Connected;
    char afLabel[32];
    std::snprintf(afLabel, sizeof(afLabel), "Autofocus  %s", kAfDirections[afDirIndex_].label);
    ImGui::BeginDisabled(!afReady);
    if (ImGui::Button(afLabel, ImVec2(-FLT_MIN, 0.0f))) StartQuickAutoFocus();
    ImGui::EndDisabled();
    // AllowWhenDisabled: the tooltip is most useful exactly when the button is
    // greyed out.
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Sweep gantry-main Z (%s) %.2f mm from here in %.3f mm\n"
                          "steps, scoring each step, then stop at the sharpest Z.\n"
                          "Needs Arm (Jog tab) and a connected camera.",
                          kAfDirections[afDirIndex_].value,
                          static_cast<double>(afRangeMM_), static_cast<double>(afStepMM_));

    // Why the button is unavailable, rather than a dead grey rectangle.
    if (!jogArmed_)                        ImGui::TextDisabled("arm jog first (Jog tab)");
    else if (st != ConnState::Connected)   ImGui::TextDisabled("camera not connected");
    else if (afBusy)                       ImGui::TextDisabled("another run is active");

    // Progress + result, but only for OUR run: the engine is shared, and a
    // recipe someone else started has no business reporting in this tab.
    const RecipeEngine::Status rs = orch_.recipe.GetStatus();
    if (rs.recipeName == kQuickAutoFocusName) {
        if (rs.state == RecipeEngine::State::Running) {
            ImGui::TextUnformatted(rs.currentStep.c_str());
            // STOP lives on the Jog tab, but a run started here should be
            // stoppable from here too.
            if (ImGui::Button("Stop##af", ImVec2(-FLT_MIN, 0.0f))) orch_.StopAll();
        } else if (!rs.error.empty() && (rs.state == RecipeEngine::State::Failed ||
                                         rs.state == RecipeEngine::State::Stopped)) {
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", rs.error.c_str());
        } else if (rs.state == RecipeEngine::State::Completed) {
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.45f, 1.0f), "focused");
        }
    }
    if (!afErr_.empty())
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", afErr_.c_str());
}

// Build the one-step recipe and hand it to the engine — the same "Run Step"
// mechanism the Recipe Editor uses, so validation (device known, camera
// Connected, params in range) is the engine's, not a second copy here.
void App::StartQuickAutoFocus()
{
    afErr_.clear();
    if (afDirIndex_ < 0 || afDirIndex_ >= kAfDirectionCount) afDirIndex_ = 0;

    RecipeStep step;
    step.type     = "auto_focus";
    step.nickname = std::string("Autofocus ") + kAfDirections[afDirIndex_].label;
    step.params   = {
        {"device", kQuickAutoFocusDevice},
        {"axis", "Z"},
        {"direction", kAfDirections[afDirIndex_].value},
        {"z_range_mm", static_cast<double>(afRangeMM_)},
        {"z_step_mm", static_cast<double>(afStepMM_)},
        {"roi_px", focusRoiPx_},
        {"trace", true},                  // one CSV line per sample into the log
    };

    Recipe one;
    one.uid  = "quick-autofocus";
    one.name = kQuickAutoFocusName;
    one.steps.push_back(std::move(step));

    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &afErr_))
        FD_WARN("vision", "autofocus not started: %s", afErr_.c_str());
}

// Jog controls (its own window). Buttons + W/D·A/S·R/F keys route through the
// workflow, which maps global direction -> local axis via the device transform.
// Everything but Stop is gated by the Arm toggle.
void App::DrawJogPanel()
{
    const size_t n = hardware_.MotionCount();
    if (n == 0) { ImGui::TextDisabled("no motion devices"); return; }
    if (jogDevice_ < 0 || static_cast<size_t>(jogDevice_) >= n) jogDevice_ = 0;

    // --- Active device list (left) + live axis readout of the selection (right) ---
    // The row is split in half: which device the keys drive, and where that
    // device currently is. Both boxes are one height, sized for the widest axis
    // count in the fleet (a hexapod's six) so switching device does not make the
    // panel below jump.
    size_t maxAxes = 3;
    for (size_t i = 0; i < n; ++i)
        if (IMotionController* d = hardware_.MotionDevice(i))
            maxAxes = (std::max)(maxAxes, d->Axes().size());
    const float rowH  = ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(maxAxes)
                        + ImGui::GetStyle().FramePadding.y * 2.0f;
    const float halfW = ImGui::GetContentRegionAvail().x * 0.5f - ImGui::GetStyle().ItemSpacing.x * 0.5f;

    ImGui::TextUnformatted("Device");
    ImGui::BeginChild("jogdevs", ImVec2(halfW, rowH), ImGuiChildFlags_Borders);
    for (size_t i = 0; i < n; ++i) {
        IMotionController* d = hardware_.MotionDevice(i);
        if (!d) continue;
        const ConnState st = hardware_.MotionState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        if (ImGui::Selectable(hardware_.MotionName(i).c_str(), jogDevice_ == static_cast<int>(i)))
            jogDevice_ = static_cast<int>(i);
        ImGui::PopID();
    }
    ImGui::EndChild();

    IMotionController* dev = hardware_.MotionDevice(static_cast<size_t>(jogDevice_));

    // Live X/Y/Z/U/V/W of the selected device, straight off the poll cache — the
    // same numbers the Motion page shows, in the panel that is moving them.
    ImGui::SameLine();
    ImGui::BeginChild("jogpos", ImVec2(0.0f, rowH), ImGuiChildFlags_Borders);
    if (!dev) {
        ImGui::TextDisabled("no device");
    } else {
        const std::vector<std::string>      axNames = dev->Axes();
        const std::map<std::string, double> live    = dev->Positions();
        for (const std::string& ax : axNames) {
            const auto it = live.find(ax);
            if (it != live.end()) ImGui::Text("%s %9.4f", ax.c_str(), it->second);
            else                  ImGui::TextDisabled("%s      --", ax.c_str());
        }
    }
    ImGui::EndChild();

    if (!dev) { ImGui::TextDisabled("select a device"); return; }
    const std::vector<std::string> axes = dev->Axes();
    const size_t                   di   = static_cast<size_t>(jogDevice_);
    dev->RequestFastPoll();   // keep positions fresh for accurate relative jogs

    // --- Arm toggle + STOP (always active; the software big-red-button) ---
    ImGui::Checkbox("Arm", &jogArmed_);
    ImGui::SameLine();
    if (jogArmed_) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "ARMED");
    else           ImGui::TextDisabled("disarmed");
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("STOP", ImVec2(-FLT_MIN, 30.0f))) orch_.StopAll();
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stops ALL motion devices and aborts any running\n"
                          "recipe / script / program / path move.\n"
                          "(Laser, PSU and IO outputs are not touched.)");
    ImGui::Separator();

    DrawVelocityCombo(di);

    // --- Step size button array ---
    ImGui::TextUnformatted("Step size");
    ImGui::SameLine();
    ImGui::TextDisabled("Q / E");
    const float bw    = 54.0f;
    const float availw = ImGui::GetContentRegionAvail().x;
    int perRow = static_cast<int>(availw / (bw + 6.0f));
    if (perRow < 1) perRow = 1;
    for (int i = 0; i < kStepCount; ++i) {
        if (i % perRow != 0) ImGui::SameLine();
        const bool sel = (jogStepIndex_ == i);
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        if (ImGui::Button(kSteps[i].label, ImVec2(bw, 0.0f))) jogStepIndex_ = i;
        if (sel) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // --- Translational cluster (global X/Y/Z), routed through the workflow ---
    // Left/Right = X-/X+ (A/D), In/Out = Y-/Y+ (W/S), Up/Down = Z+/Z- (R/F). The
    // device transform (in Workflow) maps global direction -> local axis + sign.
    const double step = kSteps[jogStepIndex_].mm;
    const ImVec2 jb(58.0f, 0.0f);   // auto height -> fits the two-line "In\nW" labels
    ImGui::BeginDisabled(!jogArmed_);
    ImGui::Dummy(ImVec2(jb.x + 8.0f, 0.0f)); ImGui::SameLine();
    if (ImGui::Button("In\nW", jb))    workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);
    if (ImGui::Button("Left\nA", jb))  workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step); ImGui::SameLine();
    if (ImGui::Button("Out\nS", jb))   workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step); ImGui::SameLine();
    if (ImGui::Button("Right\nD", jb)) workflow_.JogGlobal(di, 1.0f,  0.0f, 0.0f, step);
    if (ImGui::Button("Up\nR", jb))    workflow_.JogGlobal(di, 0.0f,  0.0f, 1.0f, step); ImGui::SameLine();
    if (ImGui::Button("Down\nF", jb))  workflow_.JogGlobal(di, 0.0f,  0.0f, -1.0f, step);

    // --- Rotational cluster (GLOBAL U/V/W = theta x/y/z), only if the device
    // has rotational axes. Routed through JogGlobalRot so the transform's
    // rotation block maps global -> signed local axis (test it live from the
    // Transforms editor). jogRotAxis_ selects the GLOBAL rotation axis.
    const int rotCount = static_cast<int>(axes.size()) - 3;
    if (rotCount > 0) {
        if (jogRotAxis_ < 0 || jogRotAxis_ >= 3) jogRotAxis_ = 0;
        static const char* kRotNames[3] = {"U \xce\xb8x", "V \xce\xb8y", "W \xce\xb8z"};

        // Mapped-target readout: dominant entry of the rotation block's column,
        // from the same working copy the Transforms editor hot-applies.
        std::string mapped = "unmapped";
        {
            AxisTransform t;   // identity if none configured
            const auto it = transformCfg_.find(dev->Name());
            if (it != transformCfg_.end()) t = it->second;
            int ax = 0;
            for (int i = 1; i < 3; ++i)
                if (std::fabs(t.r[i][jogRotAxis_]) > std::fabs(t.r[ax][jogRotAxis_])) ax = i;
            if (std::fabs(t.r[ax][jogRotAxis_]) > 1e-9 && 3 + ax < static_cast<int>(axes.size()))
                mapped = axes[3 + ax] + (t.r[ax][jogRotAxis_] >= 0.0 ? "+" : "-");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Rotational");
        ImGui::SameLine();
        ImGui::TextDisabled("-> local %s", mapped.c_str());
        ImGui::BeginGroup();
        for (int i = 0; i < 3; ++i) {
            const bool sel = (jogRotAxis_ == i);
            if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.55f, 0.10f, 1.0f));
            if (ImGui::Button(kRotNames[i], ImVec2(jb.x, 0.0f))) jogRotAxis_ = i;
            if (sel) ImGui::PopStyleColor();
        }
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        const float gu = (jogRotAxis_ == 0) ? 1.0f : 0.0f;
        const float gv = (jogRotAxis_ == 1) ? 1.0f : 0.0f;
        const float gw = (jogRotAxis_ == 2) ? 1.0f : 0.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("+##rot", ImVec2(jb.x, 0.0f))) workflow_.JogGlobalRot(di, gu, gv, gw, +step);
        ImGui::PopStyleColor();
        char stepReadout[24];
        std::snprintf(stepReadout, sizeof(stepReadout), "%s##rotstep", kSteps[jogStepIndex_].label);
        ImGui::Button(stepReadout, ImVec2(jb.x, 0.0f));   // step readout (unique id)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("-##rot", ImVec2(jb.x, 0.0f))) workflow_.JogGlobalRot(di, gu, gv, gw, -step);
        ImGui::PopStyleColor();
        ImGui::EndGroup();
    }
    ImGui::EndDisabled();

    // --- Keyboard shortcuts (this window focused): one action per key press ---
    // Q/E pick the step size (down/up the array); A/D = Left/Right, W/S =
    // In/Out, R/F = Up/Down. Q/E sit OUTSIDE the arm gate on purpose: choosing
    // a distance moves nothing, and reaching for the size before arming is the
    // normal order. Everything that actually moves stays armed-only.
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        if (ImGui::IsKeyPressed(ImGuiKey_Q, false) && jogStepIndex_ > 0)              --jogStepIndex_;
        if (ImGui::IsKeyPressed(ImGuiKey_E, false) && jogStepIndex_ < kStepCount - 1) ++jogStepIndex_;

        if (jogArmed_) {
            if (ImGui::IsKeyPressed(ImGuiKey_A, false)) workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step);  // Left  X-
            if (ImGui::IsKeyPressed(ImGuiKey_D, false)) workflow_.JogGlobal(di,  1.0f, 0.0f, 0.0f, step);  // Right X+
            if (ImGui::IsKeyPressed(ImGuiKey_W, false)) workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);  // In    Y-
            if (ImGui::IsKeyPressed(ImGuiKey_S, false)) workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step);  // Out   Y+
            if (ImGui::IsKeyPressed(ImGuiKey_R, false)) workflow_.JogGlobal(di, 0.0f, 0.0f,  1.0f, step);  // Up    Z+
            if (ImGui::IsKeyPressed(ImGuiKey_F, false)) workflow_.JogGlobal(di, 0.0f, 0.0f, -1.0f, step);  // Down  Z-
        }
    }
}

void App::DrawVelocityCombo(std::size_t di)
{
    IMotionController* dev = hardware_.MotionDevice(di);
    if (!dev) return;
    const bool    acs  = dev->Type() == "ACS";
    const double* vels = acs ? kVelACS : kVelPI;
    const int     velN = acs ? 5 : 4;
    const int     key  = static_cast<int>(di);
    int& idx = velIndex_[key];
    if (idx < 0 || idx >= velN) idx = 0;

    // Deliberately NO velocity push at connect (and none at disconnect/exit):
    // the device keeps whatever velocity it already has until the operator —
    // or a recipe/script velocity override — explicitly sets one. Until then
    // the combo shows "(device)" instead of pretending to know the value.
    ImGui::TextUnformatted("Velocity (mm/s)");
    ImGui::SetNextItemWidth(-FLT_MIN);
    char prev[24];
    if (velCustomSet_[key] > 0.0)
        std::snprintf(prev, sizeof(prev), "%g (custom)", velCustomSet_[key]);
    else if (!velPushed_[key])
        std::snprintf(prev, sizeof(prev), "(device)");
    else
        std::snprintf(prev, sizeof(prev), "%g", vels[idx]);
    if (ImGui::BeginCombo("##vel", prev)) {
        for (int i = 0; i < velN; ++i) {
            char b[16]; std::snprintf(b, sizeof(b), "%g", vels[i]);
            if (ImGui::Selectable(b, velCustomSet_[key] <= 0.0 && idx == i)) {
                idx = i;
                velCustomSet_[key] = 0.0;   // back to a preset
                workflow_.SetJogVelocity(di, vels[i]);
                velPushed_[key] = true;
            }
        }
        ImGui::EndCombo();
    }

    // Custom velocity not in the preset list. Clamped to the preset ceiling for
    // this controller type (PI 10, ACS 40 mm/s) — raise kVel* to allow more.
    const double velMax = vels[velN - 1];
    double& custom = velCustomEdit_[key];
    if (custom <= 0.0) custom = vels[idx];   // seed the box on first show
    ImGui::SetNextItemWidth(-64.0f);
    ImGui::InputDouble("##velcustom", &custom, 0.0, 0.0, "%.3f");
    ImGui::SameLine();
    if (ImGui::SmallButton("Apply")) {
        if (custom < 0.001)  custom = 0.001;
        if (custom > velMax) custom = velMax;
        velCustomSet_[key] = custom;
        workflow_.SetJogVelocity(di, custom);
        velPushed_[key] = true;
        FD_INFO("motion", "%s custom velocity %.3f mm/s", dev->Name().c_str(), custom);
    }
}

} // namespace firedog
