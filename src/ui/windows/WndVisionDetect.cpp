// WndVisionDetect.cpp : the Vision detect helper (the Recipe Editor's "Open
// Vision detect helper..." button on a detect_object step). Definitions of App
// members - see App.h.
//
// A detect_object step finds an object with its OWN detection model and
// reports X / Y (machine mm) and/or P (deg). Both halves only mean something
// against a picture, so they are edited here: the three ticks at the top, and
// under them the alignment module's teach panels (App::DrawVisionTeachBody,
// the same body the Vision teach page draws), pointed at this step's model.
//
// It is a VIEW of the step, like the other helpers: it edits the step the
// Recipe Editor has selected (App::EditorSelectedStep) in place. Its Save
// writes the recipes (which syncs step-owned models - a copied step gets its
// own) and the models.
//
// Ownership: the model is named vdet_step_<uid> (kStepDetectorPrefix) and the
// step holds the uid. Creating one is a button, never a side effect of opening
// the window - a Draw function must not mutate the document. A model another
// step also references (a pasted copy, not yet saved) is shown but not tuned
// until it is made this step's own: tuning it would re-tune the other step.
#include "App.h"

#include "imgui.h"

#include "command/CommandRegistry.h"
#include "command/Commands.h"   // kVisionDetectHelper
#include "command/Recipe.h"     // kElseBranchBase
#include "core/Product.h"       // GenerateUid
#include "vision/VisionModel.h"
#include "ui/Widgets.h"

#include <cstddef>
#include <cstdio>
#include <functional>
#include <map>
#include <string>

namespace firedog {

namespace {

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.55f, 0.85f, 0.55f, 1.0f);

// How many detect_object steps in the library reference `uid`.
int CountStepOwners(const std::vector<Recipe>& recipes, const std::string& uid)
{
    int n = 0;
    std::function<void(const std::vector<RecipeStep>&)> walk =
        [&](const std::vector<RecipeStep>& steps) {
            for (const RecipeStep& s : steps) {
                if (s.type == "detect_object" && s.params.is_object()) {
                    const auto it = s.params.find("model");
                    if (it != s.params.end() && it->is_string() &&
                        it->get<std::string>() == uid)
                        ++n;
                }
                walk(s.children);
                walk(s.elseChildren);
            }
        };
    for (const Recipe& r : recipes) walk(r.steps);
    return n;
}

} // namespace

void App::DrawVisionDetectHelperWindow()
{
    // The button is gated, but the open bit outlives a page switch and a role
    // never should.
    if (!CanAccess(Page::RecipeEdit)) { showVisionDetHelper_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(1400.0f, 860.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(kVisionDetectHelper, &showVisionDetHelper_)) {
        ImGui::End();
        return;
    }

    RecipeStep* s = EditorSelectedStep();
    const CommandSpec* spec = orch_.registry.Find("detect_object");
    if (!spec || !s || s->type != "detect_object") {
        ImGui::TextDisabled("Select a detect_object step in the Recipe Editor - this window "
                            "edits the step that is selected there.");
        ImGui::End();
        return;
    }
    const bool busy = orch_.Busy();
    const nlohmann::json merged = EffectiveParams(*spec, s->params);

    // ---- Which step; Save; Close ----------------------------------------------
    std::string path;
    for (const std::size_t c : edStepSel_) {
        if (!path.empty()) path += ".";
        if (c >= kElseBranchBase) path += "else." + std::to_string(c - kElseBranchBase + 1);
        else                      path += std::to_string(c + 1);
    }
    const Recipe& rec = session_.Recipes()[static_cast<std::size_t>(recipeSel_)];
    char owner[192];
    std::snprintf(owner, sizeof(owner), "recipe '%s'  -  step %s%s%s", rec.name.c_str(),
                  path.c_str(), s->nickname.empty() ? "" : "  ", s->nickname.c_str());
    ImGui::TextUnformatted(owner);
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    // One Save for both documents: the recipes (the ticks, the model uid - and
    // the sync that gives a copied step its own model) and the models.
    const bool dirty = session_.RecipesDirty() || session_.VisionDirty();
    ImGui::BeginDisabled(busy || !dirty);
    if (IconButton(UiIcon("Save"), dirty ? "Save*" : "Save")) {
        session_.SaveRecipes();
        session_.SaveVision();
        if (const std::string u = EffectiveParams(*spec, s->params).value("model", std::string());
            !u.empty())
            workflow_.InvalidateDetector(u);   // the next detect rebuilds from the saved model
        visionStatus_ = "saved";
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip(dirty ? "Save the recipe file (every recipe) and the detection "
                                  "models"
                                : "Nothing unsaved");
    ImGui::SameLine();
    if (ImGui::Button("Close")) showVisionDetHelper_ = false;
    if (busy) ImGui::TextColored(kWarn, "A run is active - read-only until it ends.");

    // ---- What the step reports ------------------------------------------------
    const std::string uid = merged.value("model", std::string());
    VisionModel* m = uid.empty() ? nullptr : FindVisionModel(session_.VisionModels(), uid);
    const std::string camDev = merged.value("camera_device", std::string("gantry-main"));
    const std::string prefix = merged.value("ctx_prefix", std::string("det"));

    ImGui::SeparatorText("Report");
    {
        bool tx = merged.value("take_x", true), ty = merged.value("take_y", true),
             tp = merged.value("take_p", false);
        ImGui::BeginDisabled(busy);
        if (ImGui::Checkbox("X", &tx)) s->params["take_x"] = tx;
        ImGui::SameLine();
        if (ImGui::Checkbox("Y", &ty)) s->params["take_y"] = ty;
        ImGui::SameLine();
        if (ImGui::Checkbox("P (theta)", &tp)) s->params["take_p"] = tp;
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The object's rotation from the orientation the model was\n"
                              "taught at, as the camera picture shows it: degrees,\n"
                              "CLOCKWISE POSITIVE (+90 = a quarter turn clockwise,\n"
                              "-90 = anticlockwise). Needs an ncc or corner model with\n"
                              "an angle tolerance above 0.");
        ImGui::SameLine();
        std::string axes;
        if (tx) axes += "X";
        if (ty) axes += "Y";
        if (tp) axes += "P";
        if (axes.empty())
            ImGui::TextColored(kBad, "nothing ticked - the step would report nothing");
        else
            ImGui::TextDisabled("-> object result '%s' (%s on %s)", prefix.c_str(),
                                axes.c_str(), camDev.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Named after ctx_prefix. Any later recipe of the program\n"
                              "can move to it with move_to_result; it is cleared at\n"
                              "program start, DUT open and product load. The same values\n"
                              "are also in ctx (%s_x / _y / _p, %s_ok) for this recipe.",
                              prefix.c_str(), prefix.c_str());
        ImGui::TextDisabled("X / Y = '%s' axis position at the crosshair + the object's "
                            "offset in the picture, mm", camDev.c_str());
        if (tp && m) {
            std::string why;
            if (!DetectionImageRotation(*m, VisionDetection{}, nullptr, &why))
                ImGui::TextColored(kWarn, "P is ticked but %s", why.c_str());
        }
    }

    // ---- The step's model ------------------------------------------------------
    const bool canTeach = CanAccess(Page::VisionTeach);
    if (!m) {
        ImGui::SeparatorText("Detection model");
        if (uid.empty()) ImGui::TextUnformatted("This step has no detection model yet.");
        else ImGui::TextColored(kWarn, "Its model (%s) is not in this product - create a new "
                                       "one and teach it.", uid.c_str());
        ImGui::BeginDisabled(busy || !canTeach);
        if (ImGui::Button("Create detection model")) {
            VisionModel fresh;
            fresh.uid  = GenerateUid();
            fresh.name = StepDetectorName(fresh.uid);
            s->params["model"] = fresh.uid;
            session_.VisionModels().push_back(std::move(fresh));
            // The model is saved now, the step's reference to it with the
            // recipes. A SaveRecipes in between keeps it: this step, in
            // memory, already owns it.
            session_.SaveVision();
        }
        ImGui::EndDisabled();
        if (!canTeach) ImGui::TextDisabled("Teaching a detection model needs Admin.");
        ImGui::End();
        return;
    }

    const int owners = CountStepOwners(session_.Recipes(), m->uid);
    if (owners > 1) {
        ImGui::SeparatorText("Detection model");
        ImGui::TextColored(kWarn, "This model is also used by %d other step(s) - this step is "
                                  "a copy.", owners - 1);
        ImGui::TextWrapped("Give this step its own copy before tuning it, or the other step "
                           "is re-tuned with it. (Saving the recipes does the same for every "
                           "copied step.)");
        ImGui::BeginDisabled(busy || !canTeach);
        if (ImGui::Button("Make it this step's own")) {
            const VisionModel snapshot = *m;   // the push_back below moves the vector
            const std::string nu = session_.CopyVisionModelAs(snapshot, std::string());
            if (VisionModel* fresh = FindVisionModel(session_.VisionModels(), nu))
                fresh->name = StepDetectorName(nu);
            s->params["model"] = nu;
            session_.SaveVision();
        }
        ImGui::EndDisabled();
        ImGui::End();
        return;
    }
    if (!canTeach) {
        ImGui::SeparatorText("Detection model");
        ImGui::Text("%s  (%s)", m->name.c_str(), m->method.c_str());
        ImGui::TextDisabled("Teaching a detection model needs Admin.");
        ImGui::End();
        return;
    }
    if (currentPage_ == Page::VisionTeach) {
        ImGui::TextColored(kWarn, "The Vision teach page is open - close it to tune this "
                                  "step's model here.");
        ImGui::End();
        return;
    }
    if (busy) {
        ImGui::TextDisabled("%s  (%s)", m->name.c_str(), m->method.c_str());
        ImGui::End();
        return;
    }

    // Point the shared teach state at this model. UI state, not the document:
    // the teach panels keep their selection, zoom and last detection on App.
    if (visionSelUid_ != m->uid) {
        ResetVisionTeachState();
        visionSelUid_ = m->uid;
        visionFocusX_ = static_cast<float>(cameraW_) * 0.5f;
        visionFocusY_ = static_cast<float>(cameraH_) * 0.5f;
        visionTmpl_.key.clear();
    }
    visionOwnerLabel_ = owner;

    // What the step would report from the last detection, through the same two
    // functions the command uses - so the numbers here are the numbers a run
    // writes, give or take where the gantry stands when it does.
    if (visionHaveDet_ && visionDet_.ok) {
        std::map<std::string, double> pose;
        const bool havePose = workflow_.DevicePose(camDev, pose) && pose.count("X") &&
                              pose.count("Y");
        char line[192] = "";
        int  n = 0;
        const auto room = [&] { return sizeof(line) - static_cast<std::size_t>(n); };
        if (havePose) {
            if (merged.value("take_x", true))
                n += std::snprintf(line + n, room(), "  X %.4f",
                                   pose["X"] + visionDet_.dxMM);
            if (merged.value("take_y", true))
                n += std::snprintf(line + n, room(), "  Y %.4f",
                                   pose["Y"] + visionDet_.dyMM);
        }
        if (merged.value("take_p", false)) {
            double pDeg = 0.0;
            if (DetectionImageRotation(*m, visionDet_, &pDeg, nullptr))
                std::snprintf(line + n, room(), "  P %+.3f deg", pDeg);
        }
        if (line[0]) ImGui::TextColored(kGood, "last detection would report:%s", line);
        else if (!havePose)
            ImGui::TextColored(kWarn, "'%s' not connected - no X / Y to report",
                               camDev.c_str());
    }

    // The two lines the teach page's strip carries: the last failure, and what a
    // drag on the image is about to set.
    if (!visionErr_.empty()) ImGui::TextColored(kBad, "%s", visionErr_.c_str());
    if (visionDragMode_ != 0)
        ImGui::TextColored(kWarn, "%s",
                           visionDragMode_ == 1
                               ? "drag a rectangle on the image to set the search ROI"
                               : "drag a rectangle around the feature to cut the template");
    ImGui::Separator();

    DrawVisionTeachBody(*m);
    ImGui::End();
}

} // namespace firedog
