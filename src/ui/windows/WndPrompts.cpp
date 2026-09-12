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
#include "ui/Theme.h"   // kTeachOrange (the teach prompt's Save row)
#include "core/Log.h"
#include "vision/VisionModel.h"   // FindVisionModel — the fallback's taught patch

#include <cfloat>
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
    // near-black body, pure-white text, amber accent frame, dark grey buttons)
    // so it reads identically in every app theme, and runs at 1.5x the app font
    // so it stands out from whatever page is behind it. The buttons are grey
    // rather than a saturated colour on purpose: they are the only things here
    // that can be pressed, and a neutral fill leaves the amber frame as the
    // one accent — a blue slab the width of the box competes with the sentence
    // the operator is meant to read first.
    const float promptFont = ImGui::GetStyle().FontSizeBase * 1.5f;
    ImGui::PushFont(nullptr, promptFont);
    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.06f, 0.08f, 0.12f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.00f, 1.00f, 1.00f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg,       ImVec4(0.13f, 0.34f, 0.64f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.13f, 0.34f, 0.64f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ImVec4(1.00f, 0.76f, 0.25f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.22f, 0.23f, 0.25f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.34f, 0.35f, 0.38f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.14f, 0.15f, 0.17f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,       ImVec4(0.14f, 0.17f, 0.24f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Separator,     ImVec4(1.00f, 0.76f, 0.25f, 0.60f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 14.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 8.0f));

    const ImGuiViewport* pvp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(pvp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    // WIDE on purpose. This is the window that stops the line: the machine is
    // standing still until somebody reads it, often from a step back at the
    // bench, so it takes a share of the viewport instead of shrink-wrapping
    // the sentence. A size CONSTRAINT rather than SetNextWindowSize, because
    // AlwaysAutoResize must keep owning the height — constraints are applied
    // after the auto-fit, so the window widens to the floor and still grows
    // downward exactly as far as its content needs. It scales off the viewport
    // so the same prompt reads the same on a laptop panel and a bench monitor.
    const float promptW = fminf(fmaxf(pvp->Size.x * 0.55f, 640.0f), 1400.0f);
    ImGui::SetNextWindowSizeConstraints(ImVec2(promptW, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
    ImGui::Begin("Recipe Prompt", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top
    // Wrap at the window's own edge (0.0f), not at a fixed 560 px: the width is
    // the viewport's to decide now, and a hard-coded wrap would keep breaking
    // the message at the old width inside a wider box.
    ImGui::PushTextWrapPos(0.0f);
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
        ImGui::SetNextItemWidth(-FLT_MIN);   // full width, like everything else here
        submit = ImGui::InputText("##promptin", promptInputBuf_,
                                  sizeof(promptInputBuf_),
                                  ImGuiInputTextFlags_EnterReturnsTrue);
    }
    ImGui::Separator();
    // The two answers split the row. They scale with the font as before, but a
    // wide window with two small buttons parked in its corner reads as an empty
    // box — and OK/Cancel are pressed with a glove on, in a hurry, by somebody
    // who has just walked back to the bench.
    const float btnAvail = ImGui::GetContentRegionAvail().x;
    const float btnW = fmaxf(promptFont * 5.0f,
                             (btnAvail - ImGui::GetStyle().ItemSpacing.x) * 0.5f);
    const ImVec2 btn(btnW, 0.0f);
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

    // Header: the message on the left, and on the right the DEVICE this prompt
    // is about, big, on a solid block. Two hexapods and a gantry share this
    // window, and jogging the wrong one is the mistake the box exists to
    // prevent - the small "Device:" line below was not enough from a step back.
    {
        const ImGuiStyle& style = ImGui::GetStyle();
        const float boxW = 220.0f;
        const float boxH = ImGui::GetTextLineHeight() * 3.0f + style.WindowPadding.y * 2.0f;
        const float textW = 300.0f;
        ImGui::BeginGroup();
        if (req.teach) ImGui::TextColored(kTeachOrange, "TEACH  %s", req.positionName.c_str());
        ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + textW);
        ImGui::TextUnformatted(req.message.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndGroup();
        ImGui::SameLine(textW + style.ItemSpacing.x * 2.0f);
        // Orange for a teach (it matches the Save button it leads to), the
        // arm's amber for a plain manual_adjust; dark text on both for contrast.
        ImGui::PushStyleColor(ImGuiCol_ChildBg,
                              req.teach ? kTeachOrange : ImVec4(1.0f, 0.75f, 0.20f, 1.0f));
        ImGui::BeginChild("adjdev", ImVec2(boxW, boxH), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        DrawBigLabel(req.device.c_str(),
                     ImVec2(boxW - style.WindowPadding.x * 2.0f, boxH - style.WindowPadding.y * 2.0f),
                     ImVec4(0.08f, 0.08f, 0.08f, 1.0f));
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }
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
        // A teach prompt shows what Save would replace, and by how much the
        // operator has moved off it - a wrong step size shows up as a number.
        if (req.teach && !req.target.empty()) {
            const std::map<std::string, double> live = dev->Positions();
            std::string saved, delta;
            for (const auto& [axis, val] : req.target) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%s %.4f", saved.empty() ? "" : "  ",
                              axis.c_str(), val);
                saved += buf;
                const auto li = live.find(axis);
                if (li == live.end()) continue;
                std::snprintf(buf, sizeof(buf), "%s%s %+.4f", delta.empty() ? "" : "  ",
                              axis.c_str(), li->second - val);
                delta += buf;
            }
            ImGui::TextDisabled("saved  %s", saved.c_str());
            if (!delta.empty()) ImGui::TextColored(kTeachOrange, "delta  %s", delta.c_str());
        }
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
    if (req.teach) {
        // Same bool channel; different words, because the answer means
        // something else: true writes the position, false keeps it and the
        // run goes on. Stop (the run strip) is the only abort.
        ImGui::PushStyleColor(ImGuiCol_Button,        kTeachOrange);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kTeachOrangeHov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kTeachOrangeAct);
        // 170 px: "Skip (keep saved)" clipped at the 140 the OK/Cancel pair use.
        if (ImGui::Button("Save position", ImVec2(170.0f, 0.0f))) answer(true);
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Write the device's current pose into '%s'.",
                              req.positionName.c_str());
        ImGui::SameLine();
        if (ImGui::Button("Skip (keep saved)", ImVec2(170.0f, 0.0f))) answer(false);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Leave '%s' as it is and continue.", req.positionName.c_str());
    } else {
        if (ImGui::Button("OK (Complete)", ImVec2(140.0f, 0.0f))) answer(true);
        ImGui::SameLine();
        if (ImGui::Button("Cancel (Fail Step)", ImVec2(140.0f, 0.0f))) answer(false);
    }
    ImGui::End();
}

// The align_station crosshair pick: the engine worker blocks in
// AwaitPickInternal while the operator marks a feature on the live image.
// MARKING never moves anything — the answer is a point in the current field of
// view. One thing here does: a RIGHT click drives the gantry so the clicked
// feature lands under the centre, which is behind the shared arming flag. The
// feature exists because the thing to mark is often outside a 3 mm field of
// view, and hunting for it with the Jog window while a run waits is worse.
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
    // Same window, same handshake, two vocabularies: a frame teach marks E
    // points, a placement marks points on an object. The ### id is fixed so it
    // stays ONE window whichever asked.
    char title[160];
    if (req.label.empty())
        std::snprintf(title, sizeof(title), "Alignment - mark E%d###AlignPick", req.station);
    else
        std::snprintf(title, sizeof(title), "Mark %s###AlignPick", req.label.c_str());
    ImGui::Begin(title, nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavInputs |
                 ImGuiWindowFlags_NoScrollWithMouse);   // the wheel zooms, not scrolls
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top

    ImGui::TextUnformatted(req.message.c_str());
    char what[64];
    if (req.label.empty()) std::snprintf(what, sizeof(what), "E%d", req.station);
    else                   std::snprintf(what, sizeof(what), "%s", req.label.c_str());
    ImGui::TextDisabled("%s  -  %s  -  left click places, RIGHT click drives there "
                        "(arm first), arrows nudge 1 px (Shift = 10), wheel zooms",
                        req.alignName.c_str(), what);

    // The FALLBACK banner. This window is also the ordinary manual pick, so it
    // only appears when detection was tried and failed — and then it is the
    // most important thing on screen. The operator at the machine is rarely the
    // person who tuned the model: without its taught patch they are marking
    // their own reading of "the corner", and the wrong corner of the same
    // feature is a frame wrong by the whole feature, with nothing on screen
    // saying so.
    if (!req.autoError.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "detection failed - mark %s by hand:", what);
        ImGui::SameLine();
        ImGui::TextWrapped("%s", req.autoError.c_str());

        const VisionModel* fbModel =
            req.detectorUid.empty()
                ? nullptr
                : FindVisionModel(session_.VisionModels(), req.detectorUid);
        if (fbModel && EnsurePatchTex(pickTmpl_, *fbModel)) {
            ImGui::BeginGroup();
            DrawPatchThumb(pickTmpl_, 110.0f, 110.0f);
            ImGui::EndGroup();
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::TextUnformatted("This is what the detector was looking for.");
            ImGui::TextDisabled("Put the crosshair where the cross sits on the patch -");
            ImGui::TextDisabled("that is the point this procedure was taught with.");
            ImGui::EndGroup();
        } else if (fbModel) {
            // A blob method matches a SHAPE, so there is no picture to show;
            // saying which point it reports still beats saying nothing.
            ImGui::TextDisabled("'%s' (%s) reports the %s - mark that point.",
                                fbModel->name.c_str(), fbModel->method.c_str(),
                                fbModel->method == "circle" ? "centre of the circle"
                                                            : "corner of the square");
        }
    }
    ImGui::Separator();

    // Footer height reserved so the image never pushes the buttons off-window.
    // The button row is twice as tall when it leads with the step diagrams.
    const float footerH = ImGui::GetFrameHeightWithSpacing() * 2.0f + 8.0f +
                          (req.stepIcons ? ImGui::GetFrameHeight() : 0.0f) +
                          // the guard band's two readout lines, when there is one
                          (req.band.have ? ImGui::GetTextLineHeightWithSpacing() * 2.0f
                                         : 0.0f) +
                          // the live-residual readout: two lines and the focus row
                          (req.target.have ? ImGui::GetTextLineHeightWithSpacing() * 2.0f +
                                                 ImGui::GetFrameHeightWithSpacing()
                                           : 0.0f);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y = avail.y > footerH + 40.0f ? avail.y - footerH : 40.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // ---- View transform: screen px per CAMERA px, and which camera-px window
    // is on screen. Shared with the vision teach window (ui/Widgets.h) so a
    // pixel means the same thing in both surfaces: a cover fit times a zoom,
    // with the window following the MARK so a zoomed view can never hide the
    // crosshair and the arrow-key nudge always stays visible.
    const CameraView view = MakeCameraView(origin, avail, cameraW_, cameraH_,
                                           pickPx_, pickPy_, pickZoom_);
    const auto toScreen = [&](float px, float py) { return view.ToScreen(px, py); };

    if (haveImg) {
        ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, view.Uv0(), view.Uv1());
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

    if (haveImg && imgClicked)
        view.ToCamera(ImGui::GetIO().MousePos, &pickPx_, &pickPy_);

    // RIGHT click drives the gantry so the clicked feature lands under the
    // centre. This is the one thing in this window that MOVES, so it is behind
    // the shared arming flag — click-to-move is named in the rules for exactly
    // this reason, and the window is otherwise safe with jog disarmed.
    //
    // No orch_.Busy() gate, deliberately: this window only ever exists DURING a
    // run, and being able to bring a feature into view is the whole point of
    // asking a human. manual_adjust jogs mid-run on the same argument.
    //
    // The crosshair follows to the centre, because that is where the clicked
    // feature is going. Leaving it on its old PIXEL would be wrong in a way
    // nothing on screen shows: after the move, that pixel looks somewhere else,
    // and OK would capture a point off by the whole move.
    if (haveImg && imgHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
        pixelToMmX_ > 0.0f && pixelToMmY_ > 0.0f) {
        float cx = 0.0f, cy = 0.0f;
        view.ToCamera(ImGui::GetIO().MousePos, &cx, &cy);
        const double mdx = static_cast<double>(cx - static_cast<float>(cameraW_) * 0.5f) *
                           static_cast<double>(pixelToMmX_);
        const double mdy = static_cast<double>(cy - static_cast<float>(cameraH_) * 0.5f) *
                           static_cast<double>(pixelToMmY_);
        if (jogArmed_) {
            workflow_.MoveGantryRelativeMM(mdx, mdy);
            pickPx_ = static_cast<float>(cameraW_) * 0.5f;
            pickPy_ = static_cast<float>(cameraH_) * 0.5f;
        } else {
            FD_WARN("align", "click-to-move ignored - tick Arm in the pick window");
        }
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

    // ---- The guard band ----------------------------------------------------
    // Set only for the E2 of a procedure that has a band: E1 is already
    // captured, so where E2 is ALLOWED to be is known, and it is a region on
    // this image. The verdict is recomputed every frame from the crosshair, so
    // moving the mark is the re-try — there is no loop to run and no second
    // question to answer. Both the pose and the calibration are needed to
    // project it; without them the band is simply not shown, exactly like a hint.
    double gxNow = 0.0, gyNow = 0.0;
    const bool havePose = workflow_.GantryXY(gxNow, gyNow) &&
                          pixelToMmX_ > 0.0f && pixelToMmY_ > 0.0f;
    const bool bandOn = req.band.have && havePose;
    double bandSpan = 0.0, bandTheta = 0.0, bandSpanErr = 0.0, bandThetaErr = 0.0;
    bool   bandOk = true;
    if (bandOn) {
        AlignPoint e1, e2;
        e1.wx = req.band.cx;
        e1.wy = req.band.cy;
        e2.wx = gxNow + dxMM;
        e2.wy = gyNow + dyMM;
        AlignSpanTheta(e1, e2, &bandSpan, &bandTheta);
        AlignGuard g;
        g.enabled     = true;
        g.spanMm      = req.band.spanRef;
        g.spanTolMm   = req.band.spanTol;
        g.thetaDeg    = req.band.thetaRef;
        g.thetaTolDeg = req.band.thetaTol;
        bandOk = AlignGuardCheck(g, bandSpan, bandTheta, &bandSpanErr, &bandThetaErr);
    }

    if (haveImg) {
        // Clip the overlay to the image box: zoomed in, the image-centre
        // reference can sit outside the visible window entirely.
        dl->PushClipRect(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), true);

        // Everything drawn from MACHINE mm goes through this: the guard band
        // and the run's reference points alike. Built once, because two copies
        // of the pose and calibration is how they start disagreeing.
        WorldMarkView wv;
        wv.view       = view;
        wv.boxMin     = origin;
        wv.boxMax     = ImVec2(origin.x + avail.x, origin.y + avail.y);
        wv.gx         = gxNow;
        wv.gy         = gyNow;
        wv.pixelToMmX = pixelToMmX_;
        wv.pixelToMmY = pixelToMmY_;
        wv.imgW       = cameraW_;
        wv.imgH       = cameraH_;

        // The band FIRST, under everything: it is a region to aim inside, not a
        // mark, and it must never sit on top of the crosshair being placed. It
        // turns orange the moment the mark leaves it, so the colour answers
        // "am I inside?" without reading the numbers below.
        if (bandOn) {
            WorldBand wb;
            wb.have = true;
            wb.cx   = req.band.cx;
            wb.cy   = req.band.cy;
            wb.rMin = req.band.spanRef - req.band.spanTol;
            wb.rMax = req.band.spanRef + req.band.spanTol;
            wb.aMin = req.band.thetaRef - req.band.thetaTol;
            wb.aMax = req.band.thetaRef + req.band.thetaTol;
            wb.col  = bandOk ? IM_COL32(90, 230, 130, 200)
                             : IM_COL32(255, 150, 60, 220);
            DrawWorldBand(dl, wv, wb);
        }

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

        // The run's own reference points: the nominal being aimed at, and any
        // mark already taken. Projected through the LIVE gantry pose, so they
        // stay on their features as the stage moves — and an off-screen one
        // becomes an arrow on the edge with its distance, which is the only
        // thing that distinguishes "the target is over there" from "the target
        // is where I am looking".
        if (!req.hints.empty() && havePose) {
            for (const ExecutionContext::PickHint& h : req.hints) {
                CameraMark cm;
                cm.wx    = h.wx;
                cm.wy    = h.wy;
                cm.have  = true;
                cm.label = h.label.c_str();
                // The nominal reads as the target, not as another mark: it is
                // the only one here the operator is NOT placing.
                cm.col   = (&h == &req.hints.front()) ? IM_COL32(255, 90, 90, 255)
                                                      : IM_COL32(120, 200, 255, 255);
                DrawWorldMark(dl, wv, cm, 18.0f);
            }
        }
        dl->PopClipRect();
    }

    ImGui::Separator();

    // The band in numbers, under the picture of it. Both halves are shown even
    // when only one is out: "the span is right and the angle is not" is a
    // different fault (a part turned) from its opposite (the wrong feature), and
    // the operator is the one who has to tell them apart.
    if (bandOn) {
        const ImVec4 okCol(0.4f, 0.9f, 0.5f, 1.0f);
        const ImVec4 badCol(1.0f, 0.65f, 0.25f, 1.0f);
        ImGui::TextColored(std::fabs(bandSpanErr) <= req.band.spanTol ? okCol : badCol,
                           "span %.4f mm (%.4f +/- %.4f, %+.4f)",
                           bandSpan, req.band.spanRef, req.band.spanTol, bandSpanErr);
        ImGui::SameLine();
        ImGui::TextColored(std::fabs(bandThetaErr) <= req.band.thetaTol ? okCol : badCol,
                           "   theta %.3f deg (%.3f +/- %.3f, %+.3f)",
                           bandTheta, req.band.thetaRef, req.band.thetaTol, bandThetaErr);
        if (bandOk) ImGui::TextColored(okCol, "IN BAND");
        else        ImGui::TextColored(badCol, "OUT OF BAND - move the mark into the "
                                               "shaded band, or accept it as it is");
    } else if (req.band.have) {
        ImGui::TextDisabled("guard band not shown - no gantry pose or no pixel calibration");
        ImGui::TextDisabled(" ");
    }

    // ---- The live residual --------------------------------------------------
    // Set only by the registration check, and it is the whole reason that check
    // can be driven by hand: the operator is being asked to measure a distance,
    // and until now that distance only existed in the log AFTER the run had
    // already failed. It is recomputed every frame from the crosshair and the
    // live pose, so nudging the mark or jogging Z closes the number while you
    // watch it.
    //
    // dZ is the DEVICE Z, not anything read off the picture: a camera cannot
    // show height, and the taught tip Z is simply the gantry Z that focuses on
    // the tip. Focus by hand and the Z error IS the number.
    if (req.target.have) {
        double liveZ = 0.0;
        bool   haveZ = false;
        std::map<std::string, double> pose;
        if (!req.target.device.empty() && workflow_.DevicePose(req.target.device, pose)) {
            const auto iz = pose.find("Z");
            if (iz != pose.end()) { liveZ = iz->second; haveZ = true; }
        }
        const double ex  = havePose ? (gxNow + dxMM) - req.target.x : 0.0;
        const double ey  = havePose ? (gyNow + dyMM) - req.target.y : 0.0;
        const double ez  = haveZ    ? liveZ - req.target.z          : 0.0;
        const double res = std::sqrt(ex * ex + ey * ey + ez * ez);
        // Neutral when no tolerance was given: a number wearing a colour is a
        // verdict, and the caller has to have asked for one.
        const bool   judge = req.target.tolMm > 0.0;
        const ImVec4 tOk(0.4f, 0.9f, 0.5f, 1.0f);
        const ImVec4 tBad(1.0f, 0.65f, 0.25f, 1.0f);
        const ImVec4 tCol = !judge ? ImVec4(0.75f, 0.75f, 0.78f, 1.0f)
                                   : (res <= req.target.tolMm ? tOk : tBad);
        ImGui::TextColored(tCol,
                           "off prediction  dX %+.4f  dY %+.4f  dZ %+.4f  =  %.4f mm%s",
                           ex, ey, ez, res,
                           judge ? (res <= req.target.tolMm ? "   ON TARGET" : "   OUT")
                                 : "");
        if (!havePose || !haveZ)
            ImGui::TextDisabled("partial - %s%s",
                                havePose ? "" : "no gantry pose or pixel calibration  ",
                                haveZ ? "" : "no live Z from the device");
        else if (judge)
            ImGui::TextDisabled("tolerance %.4f mm - jog Z until dZ closes, then mark the "
                                "tip and OK", req.target.tolMm);
        else
            ImGui::TextDisabled("jog Z until dZ closes, then mark the tip and OK");

        // The focus score, here and not only in the Jog window: with the check's
        // autofocus switched off this IS how Z gets set, and an operator should
        // not have to keep a second window open to see the number they are
        // maximising. Off by default - it costs a score per frame.
        bool stream = workflow_.FocusStreaming();
        if (ImGui::Checkbox("live focus score", &stream))
            workflow_.SetFocusStreaming(stream, focusRoiPx_);
        ImGui::SameLine();
        const Workflow::FocusSample fs = workflow_.LatestFocus();
        if (!stream)     ImGui::TextDisabled("not sampling");
        else if (!fs.ok) ImGui::TextDisabled("waiting for frames...");
        else             ImGui::Text("%.1f   luma %.0f", fs.score, fs.meanLuma);
    }

    const auto answer = [&](bool ok) {
        ExecutionContext::PickResult r;
        r.px   = pickPx_;
        r.py   = pickPy_;
        r.dxMM = dxMM;
        r.dyMM = dyMM;
        if (fromScript) orch_.lua.AnswerPick(ok, r);
        else            orch_.recipe.AnswerPick(ok, r);
    };
    // The step diagrams lead the button row, and ONLY when an object is being
    // marked: they draw two corners of a part, which is what place_mark asks
    // for and is not what E1/E2 are (two features that fix an axis, anywhere on
    // the part). The CALLER says so — a non-empty label used to imply it, until
    // a pick that was neither case (one needle imprint) inherited the wrong
    // picture along with its title.
    if (req.stepIcons) {
        EnsureMarkIcons();
        const MarkIcon icons[2] = {markIcons_[0], markIcons_[1]};
        DrawMarkIconsInline(icons, req.station - 1, ImGui::GetFrameHeight() * 2.0f);
    }
    // Out of band, OK changes its NAME rather than disappearing. The band is a
    // sanity check, not an interlock: the person at the machine can see the
    // part and the software cannot, so refusing them would only teach them to
    // switch the band off. What it must not do is let the accept pass for an
    // ordinary OK — hence the label, the colour, and the warning in the log,
    // which is what a run that comes out 200 um wrong is read back from.
    const bool outOfBand = bandOn && !bandOk;
    ImGui::BeginDisabled(!haveImg);
    if (outOfBand) {
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.60f, 0.36f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.72f, 0.44f, 0.14f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.82f, 0.50f, 0.16f, 1.0f));
    }
    if (ImGui::Button(outOfBand ? "Accept out of band" : "OK (Mark here)",
                      ImVec2(outOfBand ? 200.0f : 160.0f, 0.0f))) {
        if (outOfBand) {
            FD_WARN("align", "'%s': %s accepted OUT OF BAND - span %.4f (%.4f +/- %.4f), "
                             "theta %.3f (%.3f +/- %.3f)",
                    req.alignName.c_str(), what, bandSpan, req.band.spanRef,
                    req.band.spanTol, bandTheta, req.band.thetaRef, req.band.thetaTol);
        }
        answer(true);
    }
    if (outOfBand) ImGui::PopStyleColor(3);
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
    // The gate for right-click-to-move, and the only control here that arms
    // anything. Shared with the Jog window, so a session that armed once stays
    // armed — which is the behaviour an operator marking several points wants.
    ImGui::Checkbox("Arm move", &jogArmed_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Right-click on the image drives the gantry so that point\n"
                          "lands under the centre crosshair. Marking itself never\n"
                          "moves anything.");
    ImGui::SameLine();
    ImGui::TextDisabled("%.5f / %.5f mm per px", pixelToMmX_, pixelToMmY_);
    ImGui::End();
}

} // namespace firedog
