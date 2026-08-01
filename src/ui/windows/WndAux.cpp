// WndAux.cpp : the Aux window — camera image + Keithley chart in one panel,
// positioned outside the main viewport on first use so multi-viewport spawns it
// as its own OS window (drag it to a second monitor). Definitions of App
// members — see App.h.
//
// Click-to-move maps an image click through the pixel->mm calibration to
// Workflow::MoveGantryRelativeMM. It drives the real gantry, so it defaults OFF
// and must be armed explicitly; the cursor crosshair + dX/dY readout are only
// drawn while it is armed, since they describe the move a click would command.
//
// The Z arrows (gantry-main, up = Z-, down = Z+) are ordinary jog: they go
// through Workflow::JogGantryZ and are gated by the shared jogArmed_ flag. The
// Autofocus button beside them is the second launcher of the sweep the Jog
// window's Camera tab owns — same App state, same StartQuickAutoFocus, so the
// two can never disagree about what a sweep does.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"
#include "ui/UiSettings.h"

#include <cmath>
#include <cstdio>

namespace firedog {

namespace {
// Size of the cursor crosshair drawn over the image, in screen px.
constexpr float kCursorCrossPx = 30.0f;

// Step sizes offered by the Aux window's gantry-Z buttons — a short subset of
// the shared kSteps table (indices into it, so the mm values never drift).
constexpr int kAuxZSteps[]  = {5, 6, 9, 12, 14};   // 5um, 10um, 100um, 1mm, 5mm
constexpr int kAuxZStepCount = static_cast<int>(sizeof(kAuxZSteps) / sizeof(kAuxZSteps[0]));
} // namespace

void App::DrawAuxWindow()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + mv->Size.x + 40.0f, mv->Pos.y + 40.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(700.0f, 760.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Aux - Camera + Signal", &showAux_)) {
        UpdateCameraTexture();   // pull the newest camera frame into the GL texture

        // ---- Background: live camera fills the entire content region ----
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 avail  = ImGui::GetContentRegionAvail();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
        if (haveImg) {
            // "cover" fit: fill the box, preserve aspect, crop the overflow via UV.
            ImVec2 uv0(0.0f, 0.0f), uv1(1.0f, 1.0f);
            const float imgA = static_cast<float>(cameraW_) / static_cast<float>(cameraH_);
            const float boxA = avail.x / avail.y;
            if (boxA > imgA) { const float s = imgA / boxA; uv0.y = (1.0f - s) * 0.5f; uv1.y = 1.0f - uv0.y; }
            else             { const float s = boxA / imgA; uv0.x = (1.0f - s) * 0.5f; uv1.x = 1.0f - uv0.x; }
            ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, uv0, uv1);
        } else {
            dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), IM_COL32(18, 18, 22, 255));
            const char* msg = "waiting for camera feed...";
            const ImVec2 ts = ImGui::CalcTextSize(msg);
            dl->AddText(ImVec2(origin.x + (avail.x - ts.x) * 0.5f, origin.y + (avail.y - ts.y) * 0.5f),
                        IM_COL32(150, 150, 150, 255), msg);
            ImGui::Dummy(avail);   // reserve the region so layout below is correct
        }

        // ---- Interaction layer: crosshair + world-distance + click-to-move ----
        // An invisible button over the image captures hover/click. It's emitted
        // BEFORE the chart overlay child, so the chart (drawn later, on top) grabs
        // clicks in its corner and the stage won't move when you use the chart.
        ImGui::SetCursorScreenPos(origin);
        const bool imgClicked = ImGui::InvisibleButton("##imgInput", avail);
        const bool imgHovered = ImGui::IsItemHovered();
        const ImVec2 center(origin.x + avail.x * 0.5f, origin.y + avail.y * 0.5f);

        // Fixed center crosshair (the optical/stage reference).
        const ImU32 chair = IM_COL32(80, 230, 120, 200);
        dl->AddLine(ImVec2(origin.x, center.y), ImVec2(origin.x + avail.x, center.y), chair, 1.0f);
        dl->AddLine(ImVec2(center.x, origin.y), ImVec2(center.x, origin.y + avail.y), chair, 1.0f);

        // The cursor marker + world-distance readout only appear while
        // click-to-move is armed: they annotate the move a click would command,
        // so with the feature off they are just clutter over the image.
        if (haveImg && imgHovered && enableClickMove_) {
            const ImVec2 m = ImGui::GetIO().MousePos;
            // Screen -> camera pixels. The image is "cover"-fit with a single
            // uniform scale, so 1 camera px = `scale` screen px on both axes.
            const float scale = fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_));
            const float dxPix = (m.x - center.x) / scale;         // camera px from center
            const float dyPix = (m.y - center.y) / scale;
            const float dxMM  = dxPix * pixelToMmX_;              // real-world offset
            const float dyMM  = dyPix * pixelToMmY_;

            // Line from center to cursor + a 30x30 px crosshair on the cursor
            // (red: armed, so a click will command the gantry).
            const ImU32 markCol = IM_COL32(255, 90, 90, 255);
            dl->AddLine(center, m, IM_COL32(255, 220, 90, 160), 1.0f);
            const float mh = kCursorCrossPx * 0.5f;
            dl->AddLine(ImVec2(m.x - mh, m.y), ImVec2(m.x + mh, m.y), markCol, 1.5f);
            dl->AddLine(ImVec2(m.x, m.y - mh), ImVec2(m.x, m.y + mh), markCol, 1.5f);

            // World-distance label near the cursor (kept inside the image).
            char lbl[96];
            std::snprintf(lbl, sizeof(lbl), "dX %+.3f mm   dY %+.3f mm", dxMM, dyMM);
            const ImVec2 lts = ImGui::CalcTextSize(lbl);
            ImVec2 lp(m.x + 14.0f, m.y + 8.0f);
            lp.x = fminf(lp.x, origin.x + avail.x - lts.x - 8.0f);
            lp.y = fminf(lp.y, origin.y + avail.y - lts.y - 6.0f);
            dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                              ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                              IM_COL32(0, 0, 0, 165), 3.0f);
            dl->AddText(lp, IM_COL32(255, 235, 150, 255), lbl);

            // Left-click -> move gantry-main (ACS) by this XY offset, via layer 2.
            if (imgClicked) workflow_.MoveGantryRelativeMM(dxMM, dyMM);
        }

        // ---- Overlay: camera status, top-left (dark plate for readability) ----
        char camline[128];
        std::snprintf(camline, sizeof(camline), "%s  [%s]  %dx%d",
                      hardware_.CameraName().c_str(), StateText(hardware_.CameraState()),
                      cameraW_, cameraH_);
        const ImVec2 cts = ImGui::CalcTextSize(camline);
        dl->AddRectFilled(ImVec2(origin.x + 6.0f, origin.y + 6.0f),
                          ImVec2(origin.x + 14.0f + cts.x, origin.y + 12.0f + cts.y),
                          IM_COL32(0, 0, 0, 140), 4.0f);
        dl->AddText(ImVec2(origin.x + 10.0f, origin.y + 9.0f), IM_COL32(230, 230, 235, 255), camline);

        // ---- Overlay: view toggles, just under the status plate ----
        // A child window blocks the image input button beneath it, so ticking a
        // box never triggers a click-to-move.
        ImGui::SetCursorScreenPos(ImVec2(origin.x + 6.0f, origin.y + 12.0f + cts.y + 8.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.06f, 0.08f, 0.78f));
        ImGui::BeginChild("AuxControls", ImVec2(0.0f, 0.0f),
                          ImGuiChildFlags_AutoResizeX | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar);
        if (ImGui::Checkbox("Chart", &showChart_)) SaveUiSettings();
        ImGui::SameLine();
        if (ImGui::Checkbox("Click-to-move", &enableClickMove_)) SaveUiSettings();
        ImGui::SameLine();
        ImGui::Checkbox("Arm jog", &jogArmed_);   // same flag the Jog window arms

        // ---- gantry-main Z: up = Z-, down = Z+, with a step-size combo ----
        // Real motion, so it follows the jog arming rule. Layer 2 owns the
        // kinematics (JogGantryZ finds the gantry and applies the transform).
        if (auxZStepIndex_ < 0 || auxZStepIndex_ >= kAuxZStepCount) auxZStepIndex_ = 2;
        const double zstep = kSteps[kAuxZSteps[auxZStepIndex_]].mm;
        ImGui::BeginDisabled(!jogArmed_);
        ImGui::BeginGroup();
        if (ImGui::ArrowButton("##auxZUp", ImGuiDir_Up))   workflow_.JogGantryZ(-zstep);
        if (ImGui::ArrowButton("##auxZDown", ImGuiDir_Down)) workflow_.JogGantryZ(+zstep);
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::TextDisabled("gantry-main Z");
        ImGui::SetNextItemWidth(110.0f);
        if (ImGui::BeginCombo("##auxZStep", kSteps[kAuxZSteps[auxZStepIndex_]].label)) {
            for (int i = 0; i < kAuxZStepCount; ++i)
                if (ImGui::Selectable(kSteps[kAuxZSteps[i]].label, auxZStepIndex_ == i))
                    auxZStepIndex_ = i;
            ImGui::EndCombo();
        }
        ImGui::EndGroup();
        ImGui::EndDisabled();

        // ---- Autofocus, over the live image -------------------------------
        // The second launcher of the SAME sweep as the Jog window's Camera tab
        // (shared App state, one StartQuickAutoFocus): this is where an operator
        // actually is — watching the picture — when they decide focus is off.
        // Range/step stay in the Camera tab; only the direction is here, because
        // that is the knob a sweep ending on an edge tells you to change.
        if (afDirIndex_ < 0 || afDirIndex_ >= kAfDirectionCount) afDirIndex_ = 0;
        const bool afBusy  = orch_.Busy();
        const bool afReady = jogArmed_ && !afBusy &&
                             hardware_.CameraState() == ConnState::Connected;
        char afLabel[40];
        std::snprintf(afLabel, sizeof(afLabel), "Autofocus %s##auxAf",
                      kAfDirections[afDirIndex_].label);
        ImGui::BeginDisabled(!afReady);
        if (ImGui::Button(afLabel)) StartQuickAutoFocus();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Sweep gantry-main Z (%s) %.2f mm from here in %.3f mm steps\n"
                              "and stop at the sharpest Z. Range and step live in the\n"
                              "Jog window's Camera tab. Needs Arm jog + a connected camera.",
                              kAfDirections[afDirIndex_].value,
                              static_cast<double>(afRangeMM_),
                              static_cast<double>(afStepMM_));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(72.0f);
        if (ImGui::BeginCombo("##auxAfDir", kAfDirections[afDirIndex_].label)) {
            for (int i = 0; i < kAfDirectionCount; ++i)
                if (ImGui::Selectable(kAfDirections[i].label, afDirIndex_ == i)) afDirIndex_ = i;
            ImGui::EndCombo();
        }

        // Progress for OUR run only — the engine is shared with every other
        // Run surface, and this plate must not narrate somebody else's recipe.
        const RecipeEngine::Status afStatus = orch_.recipe.GetStatus();
        if (afStatus.recipeName == kQuickAutoFocusName) {
            if (afStatus.state == RecipeEngine::State::Running) {
                ImGui::TextUnformatted(afStatus.currentStep.c_str());
                if (ImGui::Button("Stop##auxAf")) orch_.StopAll();
            } else if (!afStatus.error.empty() &&
                       (afStatus.state == RecipeEngine::State::Failed ||
                        afStatus.state == RecipeEngine::State::Stopped)) {
                ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s",
                                   afStatus.error.c_str());
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();

        // ---- Overlay: signal chart, pinned to the top-right corner ----
        if (showChart_) {
            const float margin = 10.0f;
            const float panelW = avail.x * 0.45f;
            const float panelH = avail.y * 0.40f;
            ImGui::SetCursorScreenPos(ImVec2(origin.x + avail.x - panelW - margin, origin.y + margin));
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.06f, 0.08f, 0.85f));
            ImGui::BeginChild("SignalOverlay", ImVec2(panelW, panelH), ImGuiChildFlags_Borders,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            DrawSignalOverlay();
            ImGui::EndChild();
            ImGui::PopStyleColor();
        }
    }
    ImGui::End();
}

void App::DrawImageOverlay()
{
    ImGui::TextUnformatted("Vision");
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float w = 480.0f, h = 360.0f;
    ImGui::Image((ImTextureID)(intptr_t)viewportTex_, ImVec2(w, h));   // C-style cast: void*/ImU64 ImTextureID

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 c(origin.x + w * 0.5f, origin.y + h * 0.5f);
    dl->AddLine(ImVec2(c.x - 20, c.y), ImVec2(c.x + 20, c.y), IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddLine(ImVec2(c.x, c.y - 20), ImVec2(c.x, c.y + 20), IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddRect(ImVec2(origin.x + 120, origin.y + 90), ImVec2(origin.x + 300, origin.y + 240),
                IM_COL32(90, 250, 120, 255), 0.0f, 0, 2.0f);
    dl->AddText(ImVec2(origin.x + 122, origin.y + 72), IM_COL32(90, 250, 120, 255), "match 0.98");
}

void App::DrawSignalOverlay()
{
    DrawKeithleyChart(ImVec2(-1, -1));   // fill the overlay child
}

} // namespace firedog
