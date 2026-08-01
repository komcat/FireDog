// WndJog.cpp : the Jog window (View menu) and the velocity combo it shares
// with the Motion page. Definitions of App members — see App.h.
//
// Jog KINEMATICS are not here: the UI only picks a device, a step and a
// direction, and Workflow::JogGlobal maps that through the device's
// AxisTransform. Jog is armed via a checkbox and defaults OFF because the
// W/D-A/S-R/F keys drive real motion.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <cfloat>
#include <cmath>
#include <cstdio>

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
        DrawJogPanel();
    }
    ImGui::End();
}

// Jog controls (its own window). Buttons + W/D·A/S·R/F keys route through the
// workflow, which maps global direction -> local axis via the device transform.
// Everything but Stop is gated by the Arm toggle.
void App::DrawJogPanel()
{
    const size_t n = hardware_.MotionCount();
    if (n == 0) { ImGui::TextDisabled("no motion devices"); return; }
    if (jogDevice_ < 0 || static_cast<size_t>(jogDevice_) >= n) jogDevice_ = 0;

    // --- Active device list ---
    ImGui::TextUnformatted("Device");
    ImGui::BeginChild("jogdevs", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * 3.4f),
                      ImGuiChildFlags_Borders);
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

    // --- Keyboard jog (armed + this window focused): one step per key press ---
    // A/D = Left/Right, W/S = In/Out, R/F = Up/Down.
    if (jogArmed_ && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        if (ImGui::IsKeyPressed(ImGuiKey_A, false)) workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step);  // Left  X-
        if (ImGui::IsKeyPressed(ImGuiKey_D, false)) workflow_.JogGlobal(di,  1.0f, 0.0f, 0.0f, step);  // Right X+
        if (ImGui::IsKeyPressed(ImGuiKey_W, false)) workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);  // In    Y-
        if (ImGui::IsKeyPressed(ImGuiKey_S, false)) workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step);  // Out   Y+
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) workflow_.JogGlobal(di, 0.0f, 0.0f,  1.0f, step);  // Up    Z+
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) workflow_.JogGlobal(di, 0.0f, 0.0f, -1.0f, step);  // Down  Z-
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
