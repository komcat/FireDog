// WndPrompts.cpp : the GLOBAL run windows — the recipe/script `prompt`, the
// `manual_adjust` jog window, and the `align_station` crosshair pick.
// Definitions of App members — see App.h.
//
// All are NON-modal and kept always-on-top via BringWindowToDisplayFront (the
// only imgui_internal.h use in the project): only the engine worker blocks on
// the answer, so the rest of the UI stays usable for camera and jog while a run
// waits. All edge-detect a NEW pending request, because a back-to-back one can
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

// The align_station crosshair pick: the engine worker blocks in
// AwaitPickInternal while the operator marks a feature on the live image.
// NOTHING MOVES here — the answer is a point in the current field of view, so
// the whole window is safe with jog disarmed and needs no arming checkbox.
//
// The pixel->mm conversion happens HERE, not in the command: only the UI knows
// the live pixelToMm calibration. The crosshair starts at the image centre, so
// OK without a click means "the feature is dead centre" (offset 0,0).
void App::DrawAlignPickWindow()
{
    const RecipeEngine::Status rst = orch_.recipe.GetStatus();
    const LuaEngine::Status    lst = orch_.lua.GetStatus();
    const bool fromScript = !rst.pendingPick && lst.pendingPick;
    const bool pending    = rst.pendingPick || lst.pendingPick;
    const ExecutionContext::PickRequest& req = fromScript ? lst.pickReq : rst.pickReq;

    const bool newPick = pending && !pickWasPending_;
    pickWasPending_ = pending;
    if (!pending) return;   // nothing pending (or stop released it)

    UpdateCameraTexture();   // pull the newest camera frame into the GL texture
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
    if (newPick) {   // start dead centre: OK with no click means "no offset"
        pickPx_ = haveImg ? static_cast<float>(cameraW_) * 0.5f : 0.0f;
        pickPy_ = haveImg ? static_cast<float>(cameraH_) * 0.5f : 0.0f;
    }

    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(900.0f, 720.0f), ImGuiCond_FirstUseEver);
    // BringWindowToDisplayFront only changes DRAW ORDER, not focus — without
    // this the window comes up on top but the keyboard still belongs to
    // whatever had it, so the arrow-key nudge would do nothing until the
    // operator happened to click the window.
    if (newPick) ImGui::SetNextWindowFocus();
    char title[128];
    std::snprintf(title, sizeof(title), "Alignment - mark E%d###AlignPick", req.station);
    ImGui::Begin(title, nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavInputs |
                 ImGuiWindowFlags_NoScrollWithMouse);   // the wheel zooms, not scrolls
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top

    ImGui::TextUnformatted(req.message.c_str());
    ImGui::TextDisabled("%s  -  E%d  -  click to place (recentres), arrow keys nudge "
                        "1 px (Shift = 10 px), wheel zooms",
                        req.alignName.c_str(), req.station);
    ImGui::Separator();

    // Footer height reserved so the image never pushes the buttons off-window.
    const float footerH = ImGui::GetFrameHeightWithSpacing() * 2.0f + 8.0f;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y = avail.y > footerH + 40.0f ? avail.y - footerH : 40.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // ---- View transform: screen px per CAMERA px, and which camera-px window
    // is on screen. `fit` is the old cover fit (fill the box, crop the
    // overflow); zoom multiplies it, so zoom 1.0 reproduces the previous view
    // exactly and the default 1.5 magnifies the feature by 50%.
    //
    // The window follows the MARK (clamped to the image), so a zoomed view can
    // never hide the crosshair and the arrow-key nudge always stays visible.
    // Because zoom >= 1 and `fit` is a cover fit, the visible window is always
    // inside the image — no letterboxing to handle.
    const float fit = haveImg ? fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_))
                              : 1.0f;
    const float s  = fit * pickZoom_;                  // screen px per camera px
    const float vw = avail.x / s, vh = avail.y / s;    // visible camera-px extent
    const auto viewClamp = [](float v, float half, float extent) {
        return (half * 2.0f >= extent) ? extent * 0.5f
                                       : fminf(fmaxf(v, half), extent - half);
    };
    const float viewLeft =
        viewClamp(pickPx_, vw * 0.5f, static_cast<float>(cameraW_)) - vw * 0.5f;
    const float viewTop =
        viewClamp(pickPy_, vh * 0.5f, static_cast<float>(cameraH_)) - vh * 0.5f;
    // camera px -> screen, and back.
    const auto toScreen = [&](float px, float py) {
        return ImVec2(origin.x + (px - viewLeft) * s, origin.y + (py - viewTop) * s);
    };

    if (haveImg) {
        const ImVec2 uv0(viewLeft / static_cast<float>(cameraW_),
                         viewTop / static_cast<float>(cameraH_));
        const ImVec2 uv1((viewLeft + vw) / static_cast<float>(cameraW_),
                         (viewTop + vh) / static_cast<float>(cameraH_));
        ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, uv0, uv1);
    } else {
        dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y),
                          IM_COL32(18, 18, 22, 255));
        const char* msg = "no camera feed - cannot mark a point";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(origin.x + (avail.x - ts.x) * 0.5f,
                           origin.y + (avail.y - ts.y) * 0.5f),
                    IM_COL32(220, 140, 140, 255), msg);
        ImGui::Dummy(avail);
    }

    ImGui::SetCursorScreenPos(origin);
    const bool imgClicked = ImGui::InvisibleButton("##pickInput", avail);
    const bool imgHovered = ImGui::IsItemHovered();

    if (haveImg && imgClicked) {
        const ImVec2 m = ImGui::GetIO().MousePos;
        pickPx_ = viewLeft + (m.x - origin.x) / s;
        pickPy_ = viewTop  + (m.y - origin.y) / s;
    }
    // Wheel zoom over the image. Applies from the NEXT frame (the view was
    // already computed above) — a frame of latency nobody can see.
    if (haveImg && imgHovered && ImGui::GetIO().MouseWheel != 0.0f) {
        pickZoom_ *= (ImGui::GetIO().MouseWheel > 0.0f) ? 1.25f : 0.8f;
        pickZoom_ = fminf(fmaxf(pickZoom_, 1.0f), 12.0f);
    }
    // Refinement without a zoom: one press = one CAMERA pixel (hold to repeat,
    // Shift for 10). Gated on focus-or-hover rather than taken globally — the
    // window is non-modal, so an arrow press aimed elsewhere must stay there;
    // hover counts because the mouse is naturally over the image while marking.
    const bool keysHere =
        ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) ||
        ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows |
                               ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
    if (haveImg && keysHere) {
        const float nudge = ImGui::GetIO().KeyShift ? 10.0f : 1.0f;
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))  pickPx_ -= nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) pickPx_ += nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))    pickPy_ -= nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))  pickPy_ += nudge;
        pickPx_ = fminf(fmaxf(pickPx_, 0.0f), static_cast<float>(cameraW_));
        pickPy_ = fminf(fmaxf(pickPy_, 0.0f), static_cast<float>(cameraH_));
    }

    const float dxPix = pickPx_ - static_cast<float>(cameraW_) * 0.5f;
    const float dyPix = pickPy_ - static_cast<float>(cameraH_) * 0.5f;
    const double dxMM = static_cast<double>(dxPix) * pixelToMmX_;
    const double dyMM = static_cast<double>(dyPix) * pixelToMmY_;

    if (haveImg) {
        // Clip the overlay to the image box: zoomed in, the image-centre
        // reference can sit outside the visible window entirely.
        dl->PushClipRect(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), true);

        // The optical/stage reference: image centre, wherever it now maps to.
        const ImVec2 ctr = toScreen(static_cast<float>(cameraW_) * 0.5f,
                                    static_cast<float>(cameraH_) * 0.5f);
        dl->AddLine(ImVec2(origin.x, ctr.y), ImVec2(origin.x + avail.x, ctr.y),
                    IM_COL32(80, 230, 120, 140), 1.0f);
        dl->AddLine(ImVec2(ctr.x, origin.y), ImVec2(ctr.x, origin.y + avail.y),
                    IM_COL32(80, 230, 120, 140), 1.0f);

        const ImVec2 mk = toScreen(pickPx_, pickPy_);
        const ImU32 markCol = IM_COL32(255, 200, 60, 255);
        const float mh = 22.0f;
        dl->AddLine(ImVec2(mk.x - mh, mk.y), ImVec2(mk.x + mh, mk.y), markCol, 2.0f);
        dl->AddLine(ImVec2(mk.x, mk.y - mh), ImVec2(mk.x, mk.y + mh), markCol, 2.0f);
        dl->AddCircle(mk, 5.0f, markCol, 0, 2.0f);

        char lbl[128];
        std::snprintf(lbl, sizeof(lbl), "px %.1f, %.1f   dX %+.4f mm   dY %+.4f mm",
                      pickPx_, pickPy_, dxMM, dyMM);
        const ImVec2 lts = ImGui::CalcTextSize(lbl);
        ImVec2 lp(mk.x + 16.0f, mk.y + 10.0f);
        lp.x = fminf(lp.x, origin.x + avail.x - lts.x - 8.0f);
        lp.y = fminf(lp.y, origin.y + avail.y - lts.y - 6.0f);
        dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                          ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                          IM_COL32(0, 0, 0, 170), 3.0f);
        dl->AddText(lp, IM_COL32(255, 235, 150, 255), lbl);
        dl->PopClipRect();
    }

    ImGui::Separator();
    const auto answer = [&](bool ok) {
        ExecutionContext::PickResult r;
        r.px   = pickPx_;
        r.py   = pickPy_;
        r.dxMM = dxMM;
        r.dyMM = dyMM;
        if (fromScript) orch_.lua.AnswerPick(ok, r);
        else            orch_.recipe.AnswerPick(ok, r);
    };
    ImGui::BeginDisabled(!haveImg);
    if (ImGui::Button("OK (Mark here)", ImVec2(160.0f, 0.0f))) answer(true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel (Fail Step)", ImVec2(160.0f, 0.0f))) answer(false);
    ImGui::SameLine();
    ImGui::TextUnformatted("zoom");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("##pickzoom", &pickZoom_, 1.0f, 12.0f, "x%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) pickZoom_ = 1.0f;
    ImGui::SameLine();
    ImGui::TextDisabled("%.5f / %.5f mm per px", pixelToMmX_, pixelToMmY_);
    ImGui::End();
}

} // namespace firedog
