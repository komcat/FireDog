// WndPrompts.cpp : the two GLOBAL run windows — the recipe/script `prompt` and
// the `manual_adjust` jog window. Definitions of App members — see App.h.
//
// Both are NON-modal and kept always-on-top via BringWindowToDisplayFront (the
// only imgui_internal.h use in the project): only the engine worker blocks on
// the answer, so the rest of the UI stays usable for camera and jog while a run
// waits. Both edge-detect a NEW pending request, because a back-to-back one can
// arrive while the window is still shown.
#include "App.h"

#include "imgui.h"
#include "imgui_internal.h"   // BringWindowToDisplayFront (always-on-top windows)

#include "ui/Widgets.h"
#include "core/Log.h"

#include <cmath>
#include <cstdio>

namespace firedog {

// Global orchestration prompt — drawn every frame from DrawMainWindow, so a
// blocking recipe `prompt` step or script fd.prompt() surfaces no matter which
// page or window the operator is on. Only one engine runs at a time, so at
// most one prompt can be pending. Deliberately NON-modal: only the engine
// worker is blocked, the rest of the UI (jog, camera, click-to-move) stays
// usable — the window is just kept on top so the prompt can't get buried.
void App::DrawRecipePromptWindow()
{
    const RecipeEngine::Status rst = orch_.recipe.GetStatus();
    const LuaEngine::Status    lst = orch_.lua.GetStatus();
    const bool fromScript = rst.pendingPrompt.empty() && !lst.pendingPrompt.empty();
    const std::string& pending = fromScript ? lst.pendingPrompt : rst.pendingPrompt;
    const bool wantsInput =
        fromScript ? lst.pendingPromptInput : rst.pendingPromptInput;

    // Edge-detect a NEW prompt (a back-to-back prompt can arrive while the
    // window is still visible, so IsWindowAppearing alone is not enough).
    const bool newPrompt = !pending.empty() && !promptWasPending_;
    promptWasPending_ = !pending.empty();
    if (pending.empty()) return;   // nothing pending (or stop released it)
    if (newPrompt) promptInputBuf_[0] = '\0';

    // Operator-facing dialog: it defines its OWN high-contrast palette (opaque
    // near-black body, pure-white text, amber accent frame, bright buttons) so
    // it reads identically in every app theme, and runs at 1.5x the app font
    // so it stands out from whatever page is behind it.
    const float promptFont = ImGui::GetStyle().FontSizeBase * 1.5f;
    ImGui::PushFont(nullptr, promptFont);
    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.06f, 0.08f, 0.12f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.00f, 1.00f, 1.00f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg,       ImVec4(0.13f, 0.34f, 0.64f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.13f, 0.34f, 0.64f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ImVec4(1.00f, 0.76f, 0.25f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.42f, 0.76f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.26f, 0.55f, 0.92f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.10f, 0.30f, 0.58f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,       ImVec4(0.14f, 0.17f, 0.24f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Separator,     ImVec4(1.00f, 0.76f, 0.25f, 0.60f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 14.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 8.0f));

    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Recipe Prompt", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top
    ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 560.0f);
    ImGui::TextUnformatted(pending.c_str());
    ImGui::PopTextWrapPos();
    const auto answer = [&](bool ok) {
        if (fromScript) orch_.lua.AnswerPrompt(ok, promptInputBuf_);
        else            orch_.recipe.AnswerPrompt(ok, promptInputBuf_);
    };
    bool submit = false;
    if (wantsInput) {
        // Barcode scanners type + Enter, so Enter submits like OK.
        if (newPrompt || ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(400.0f);
        submit = ImGui::InputText("##promptin", promptInputBuf_,
                                  sizeof(promptInputBuf_),
                                  ImGuiInputTextFlags_EnterReturnsTrue);
    }
    ImGui::Separator();
    const ImVec2 btn(promptFont * 5.0f, 0.0f);   // buttons scale with the font
    if (ImGui::Button("OK", btn) || submit) answer(true);
    ImGui::SameLine();
    if (ImGui::Button("Cancel", btn)) answer(false);
    ImGui::End();

    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(10);
    ImGui::PopFont();
}

// The manual_adjust window (bobcat's ManualAdjustWidget): the engine worker
// blocks in AwaitAdjustInternal while the operator jogs the device from here.
// Jogs go through Workflow::JogGlobal exactly like the Jog window (transform-
// mapped, safety-inhibited); positions stay fresh via RequestFastPoll.
// NON-modal on purpose: while adjusting, the operator still needs the rest of
// the app (camera view, Jog window, click-to-move) — the window is only kept
// on top, it doesn't block input to anything else.
void App::DrawManualAdjustWindow()
{
    const RecipeEngine::Status rst = orch_.recipe.GetStatus();
    const LuaEngine::Status    lst = orch_.lua.GetStatus();
    const bool fromScript = !rst.pendingAdjust && lst.pendingAdjust;
    const bool pending    = rst.pendingAdjust || lst.pendingAdjust;
    const ExecutionContext::AdjustRequest& req =
        fromScript ? lst.adjustReq : rst.adjustReq;

    const bool newAdjust = pending && !adjustWasPending_;
    adjustWasPending_ = pending;
    if (!pending) return;   // nothing pending (or stop released it)
    if (newAdjust) {
        // Seed the preset selection from the step's step_um (closest match).
        adjustStepIdx_ = 0;
        const double stepMm = req.stepUm / 1000.0;
        for (int i = 1; i < kStepCount; ++i) {
            if (std::fabs(kSteps[i].mm - stepMm) <
                std::fabs(kSteps[adjustStepIdx_].mm - stepMm))
                adjustStepIdx_ = i;
        }
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Manual Adjust", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top

    ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 460.0f);
    ImGui::TextUnformatted(req.message.c_str());
    ImGui::PopTextWrapPos();
    ImGui::Separator();

    // Resolve the device each frame (cheap; the command validated it exists).
    int di = -1;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == req.device) { di = static_cast<int>(i); break; }
    }
    IMotionController* dev =
        di >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(di)) : nullptr;

    ImGui::TextDisabled("Device: %s", req.device.c_str());
    if (dev) {
        dev->RequestFastPoll();   // live readout + accurate relative jogs
        std::string line;
        for (const auto& [axis, val] : dev->Positions()) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "%s%s %.4f", line.empty() ? "" : "  ",
                          axis.c_str(), val);
            line += buf;
        }
        ImGui::TextUnformatted(line.empty() ? "(no position data)" : line.c_str());
    }
    ImGui::Spacing();

    // Step presets (same table as the Jog window).
    ImGui::TextUnformatted("Step");
    ImGui::SameLine();
    for (int i = 0; i < kStepCount; ++i) {
        if (i % 8 != 0) ImGui::SameLine();
        const bool sel = (adjustStepIdx_ == i);
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        if (ImGui::SmallButton(kSteps[i].label)) adjustStepIdx_ = i;
        if (sel) ImGui::PopStyleColor();
    }
    ImGui::Spacing();

    // Jog row (global frame, like the Jog window; per-axis enables from the
    // step). Workflow inhibits motion during a critical safety alert anyway —
    // the visual disable just says so up front.
    const double step = kSteps[adjustStepIdx_].mm;
    // Same critical-alert query the engines use (SafetyBlocked is private).
    const bool safetyBlocked =
        workflow_.SafetyAlertActive() && workflow_.SafetyAlert().critical;
    const bool canJog = dev && !safetyBlocked;
    const std::size_t udi = static_cast<std::size_t>(di < 0 ? 0 : di);
    const ImVec2 jb(52.0f, 0.0f);
    ImGui::BeginDisabled(!canJog || !req.enableX);
    if (ImGui::Button("X-", jb)) workflow_.JogGlobal(udi, -1.0f, 0.0f, 0.0f, step);
    ImGui::SameLine();
    if (ImGui::Button("X+", jb)) workflow_.JogGlobal(udi, 1.0f, 0.0f, 0.0f, step);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canJog || !req.enableY);
    if (ImGui::Button("Y-", jb)) workflow_.JogGlobal(udi, 0.0f, -1.0f, 0.0f, step);
    ImGui::SameLine();
    if (ImGui::Button("Y+", jb)) workflow_.JogGlobal(udi, 0.0f, 1.0f, 0.0f, step);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canJog || !req.enableZ);
    if (ImGui::Button("Z-", jb)) workflow_.JogGlobal(udi, 0.0f, 0.0f, -1.0f, step);
    ImGui::SameLine();
    if (ImGui::Button("Z+", jb)) workflow_.JogGlobal(udi, 0.0f, 0.0f, 1.0f, step);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("STOP", jb) && di >= 0) workflow_.StopMotion(udi);
    ImGui::PopStyleColor(2);
    if (safetyBlocked)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                           "Safety alert active - motion inhibited");
    ImGui::Separator();

    const auto answer = [&](bool ok) {
        if (fromScript) orch_.lua.AnswerAdjust(ok);
        else            orch_.recipe.AnswerAdjust(ok);
    };
    if (ImGui::Button("OK (Complete)", ImVec2(140.0f, 0.0f))) answer(true);
    ImGui::SameLine();
    if (ImGui::Button("Cancel (Fail Step)", ImVec2(140.0f, 0.0f))) answer(false);
    ImGui::End();
}

} // namespace firedog
