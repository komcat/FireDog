// WndRoiFilterView.cpp : the ROI Filter view setup helper (the Recipe Editor's
// "Open ROI Filter view setup..." button on a roi_filter_view_setup step).
// Definitions of App members - see App.h.
//
// A roi_filter_view_setup step is seven numbers that only mean something
// against a picture: an exposure, a light call-out, an ROI size and a binary
// threshold. This window shows the live main camera beside them, with the
// step's ROI filter drawn over it, so the operator tunes the step by looking.
//
// It is a VIEW of the step, not a second record, exactly like the Dispense
// helper: it edits the step the Recipe Editor has selected
// (App::EditorSelectedStep), in place, through the one params <-> CameraSetup
// mapping (command/RoiFilterView.h), and its Save IS the editor's Save.
//
// What it does to the machine: an exposure edit and a light click are applied
// at once (the light QUEUED, never a blocking set here), because the picture
// is the point; "Try on camera" applies the whole setup the way a run would,
// including the Aux view's filter (App::ApplyCameraSetupNow). The filter drawn
// HERE is this window's own preview and never leaves it. Nothing applies while
// a run is active - the run owns the camera then.
#include "App.h"

#include "imgui.h"

#include "command/CommandRegistry.h"
#include "command/Commands.h"        // kRoiFilterViewHelper
#include "command/Recipe.h"          // kElseBranchBase
#include "command/RoiFilterView.h"   // the step <-> CameraSetup mapping
#include "hardware/HardwareManager.h"
#include "ui/RoiFilter.h"
#include "ui/Widgets.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace firedog {

namespace {

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);

constexpr float kRfvCtlW = 380.0f;   // the control column's width

} // namespace

void App::DrawRoiFilterViewHelperWindow()
{
    // The button is gated, but the open bit outlives a page switch and a role
    // never should.
    if (!CanAccess(Page::RecipeEdit)) { showRoiViewHelper_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(1180.0f, 740.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(kRoiFilterViewHelper, &showRoiViewHelper_)) {
        ImGui::End();
        return;
    }

    RecipeStep* s = EditorSelectedStep();
    const CommandSpec* spec = orch_.registry.Find("roi_filter_view_setup");
    if (!spec || !s || s->type != "roi_filter_view_setup") {
        ImGui::TextDisabled("Select a roi_filter_view_setup step in the Recipe Editor - this "
                            "window edits the step that is selected there.");
        ImGui::End();
        return;
    }

    // The step as it will RUN: stored params, defaults for the rest, through
    // the same mapping the command uses.
    const nlohmann::json merged = EffectiveParams(*spec, s->params);
    CameraSetup setup = RoiFilterViewSetupFromParams(ParamValues(merged));
    // A step that leaves the light still remembers the call-out it had, so the
    // picker can offer it back. Read here, not in the mapping: it is the
    // picker's starting point, not something a run applies.
    if (!setup.lighting.Set()) {
        setup.lighting.color = merged.value("color", std::string("white"));
        setup.lighting.value = merged.value("percent", 50.0);
    }
    const CameraSetup before = setup;

    const bool busy = orch_.Busy();
    UpdateCameraTexture();
    const RoiFilterFrame frame = MainCameraFrame();

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  imgW  = std::max(80.0f, avail.x - kRfvCtlW - ImGui::GetStyle().ItemSpacing.x);

    // ---- Left: the live main camera, the whole frame, the step's filter on it ----
    ImGui::BeginChild("rfvImg", ImVec2(imgW, avail.y), ImGuiChildFlags_Borders);
    {
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 box    = ImGui::GetContentRegionAvail();
        ImDrawList*  dl     = ImGui::GetWindowDrawList();
        if (cameraTex_ && frame.Valid() && box.x > 1.0f && box.y > 1.0f) {
            // The WHOLE frame, letterboxed: a zoom below the cover fit by the
            // contain/cover ratio. A view setup is about the picture as a
            // whole - the ROI's place in it most of all.
            const float fw = static_cast<float>(frame.w), fh = static_cast<float>(frame.h);
            const float cover   = fmaxf(box.x / fw, box.y / fh);
            const float contain = fminf(box.x / fw, box.y / fh);
            const CameraView view = MakeCameraView(origin, box, frame.w, frame.h, fw * 0.5f,
                                                   fh * 0.5f, contain / cover);
            DrawCameraViewImage(dl, cameraTex_, view);
            // The preview of THIS step's filter, centred on the image centre -
            // where the Aux view, which a run drives, centres it too.
            const RoiOverlay ov = UpdateRoiFilter(host_, setup.filter, rfvFilterSt_, frame,
                                                  0.0f, 0.0f);
            DrawRoiOverlay(dl, view, ov);
            const ImU32 chair = IM_COL32(80, 230, 120, 160);
            dl->AddLine(view.ToScreen(fw * 0.5f, 0.0f), view.ToScreen(fw * 0.5f, fh), chair);
            dl->AddLine(view.ToScreen(0.0f, fh * 0.5f), view.ToScreen(fw, fh * 0.5f), chair);
            DrawZoomBadge(dl, view, contain / cover);
        } else {
            dl->AddRectFilled(origin, ImVec2(origin.x + box.x, origin.y + box.y),
                              IM_COL32(18, 18, 22, 255));
            const char* msg = "no camera feed - connect main_camera to see the view";
            const ImVec2 ts = ImGui::CalcTextSize(msg);
            dl->AddText(ImVec2(origin.x + (box.x - ts.x) * 0.5f, origin.y + (box.y - ts.y) * 0.5f),
                        IM_COL32(220, 140, 140, 255), msg);
        }
        ImGui::Dummy(box);
    }
    ImGui::EndChild();

    ImGui::SameLine();

    // ---- Right: which step, then the settings ----------------------------------
    ImGui::BeginChild("rfvCtl", ImVec2(0.0f, avail.y), ImGuiChildFlags_Borders);
    {
        std::string path;
        for (const std::size_t c : edStepSel_) {
            if (!path.empty()) path += ".";
            if (c >= kElseBranchBase) path += "else." + std::to_string(c - kElseBranchBase + 1);
            else                      path += std::to_string(c + 1);
        }
        const Recipe& r = session_.Recipes()[static_cast<std::size_t>(recipeSel_)];
        ImGui::TextWrapped("%s  -  step %s%s%s", r.name.c_str(), path.c_str(),
                           s->nickname.empty() ? "" : "  ", s->nickname.c_str());

        // Save is the Recipe Editor's own Save (the whole recipe file); Close
        // only hides the window - the edits stay in the document.
        const bool  dirty = session_.RecipesDirty();
        const float bw    = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        ImGui::BeginDisabled(busy || !dirty);
        if (IconButton(UiIcon("Save"), dirty ? "Save*" : "Save", ImVec2(bw, 0.0f)))
            session_.SaveRecipes();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip(dirty ? "Save the recipe file (every recipe) - the Recipe "
                                      "Editor's Save"
                                    : "No unsaved recipe changes");
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(bw, 0.0f))) showRoiViewHelper_ = false;
        if (busy) ImGui::TextColored(kWarn, "A run is active - read-only until it ends.");

        ImGui::BeginDisabled(busy);

        // -- Exposure --
        ImGui::SeparatorText("Exposure - main_camera");
        PylonCamera& cam   = hardware_.Camera();
        const bool   camOk = hardware_.CameraState() == ConnState::Connected;
        bool leave = setup.exposureUs <= 0.0;
        if (ImGui::Checkbox("leave as it is", &leave)) {
            // Turning it on to a number nobody chose would be a guess: start
            // from what the camera is set to now.
            const double cur = camOk ? cam.ExposureUs() : 0.0;
            setup.exposureUs = leave ? 0.0 : (cur > 0.0 ? cur : 1000.0);
        }
        if (!leave) {
            double lo = 0.0, hi = 0.0;
            if (camOk) cam.ExposureRange(lo, hi);
            if (lo <= 0.0 || hi <= lo) { lo = 20.0; hi = 100000.0; }
            float us = static_cast<float>(setup.exposureUs);
            ImGui::SetNextItemWidth(220.0f);
            if (ImGui::SliderFloat("us##rfvexp", &us, static_cast<float>(lo),
                                   static_cast<float>(hi), "%.0f", ImGuiSliderFlags_Logarithmic))
                setup.exposureUs = static_cast<double>(us);
            // Set on release: one exposure change per edit, not one per pixel
            // the slider moved.
            if (ImGui::IsItemDeactivatedAfterEdit() && camOk)
                workflow_.SetCameraExposure(setup.exposureUs);
            ImGui::SameLine();
            ImGui::BeginDisabled(!camOk);
            if (ImGui::Button("Read")) {
                const double cur = cam.ExposureUs();
                if (cur > 0.0) setup.exposureUs = cur;
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("Take the camera's current exposure into the step.");
        }
        if (!camOk) ImGui::TextDisabled("main_camera not connected - stored, not applied");

        // -- Light --
        ImGui::SeparatorText("Light");
        const bool lightOk = hardware_.LightState() == ConnState::Connected;
        bool applyLight = false;
        LightCalloutPicker("rfvLight", setup.lighting, workflow_.LightNames(), lightOk,
                           &applyLight);
        if (applyLight) ApplyLightingNow(setup.lighting);

        // -- ROI filter --
        ImGui::SeparatorText("ROI filter - display only");
        DrawRoiFilterToolbar("rfvRoi", setup.filter, rfvFilterSt_, frame, 0.0f, 0.0f);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Centred on the view's centre. Drawn over the operator's live "
                            "view (the Aux window) when the step runs; nothing measures "
                            "on it.");
        ImGui::PopTextWrapPos();

        ImGui::EndDisabled();

        // -- What the step does, and trying it --
        ImGui::SeparatorText("This step");
        CameraSetup shown = setup;
        if (!shown.lighting.Set()) shown.lighting = AlignLighting{};
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextUnformatted(CameraSetupSummary(shown).c_str());
        ImGui::PopTextWrapPos();

        ImGui::BeginDisabled(busy);
        if (ImGui::Button("Try on camera", ImVec2(-1.0f, 0.0f))) ApplyCameraSetupNow(shown);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Apply it now the way a run would: exposure, light, and the\n"
                              "filter on the Aux window's live view.");
        ImGui::BeginDisabled(!auxFilter_.Active());
        if (ImGui::Button("Clear the Aux view filter", ImVec2(-1.0f, 0.0f))) auxFilter_ = ViewFilter{};
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Put the Aux window back on the raw picture. Changes no step.");
    }
    ImGui::EndChild();

    // Write back only on an edit, through the one mapping - a Draw that merely
    // looked at a step must not rewrite it.
    if (!(setup == before)) RoiFilterViewSetupToParams(setup, s->params);

    ImGui::End();
}

} // namespace firedog
