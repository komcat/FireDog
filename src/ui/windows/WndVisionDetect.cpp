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
#include "core/AlignLighting.h" // AlignLightingSummary
#include "core/Log.h"
#include "core/Product.h"       // GenerateUid
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "vision/FocusMetric.h" // RgbToGray8 - the thumbnail crop
#include "vision/VisionModel.h"
#include "ui/RoiFilter.h"       // RoiFilterFrame (MainCameraFrame)
#include "ui/Widgets.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <system_error>
#include <vector>

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

// Fleet index for a configured motion device name, -1 when there is none.
int MotionIndexOf(const HardwareManager& hw, const std::string& name)
{
    for (std::size_t i = 0; i < hw.MotionCount(); ++i)
        if (hw.MotionName(i) == name) return static_cast<int>(i);
    return -1;
}

// The step's taught view, from its effective params.
bool ViewPointOf(const nlohmann::json& merged, double* xyz)
{
    if (!merged.contains("view_point")) return false;
    const nlohmann::json& v = merged["view_point"];
    if (!v.is_object()) return false;
    static const char* kAx[3] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; ++a) {
        if (!v.contains(kAx[a]) || !v[kAx[a]].is_number()) return false;
        xyz[a] = v[kAx[a]].get<double>();
    }
    return true;
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
    // Where the camera goes before it looks, and what happens around the look.
    // Before the Admin gate: anybody who may edit the recipe may drive to the
    // view and set the run options; Teach here writes the model too, so it
    // carries the gate itself.
    DrawDetectViewSection(*s, merged, *m, busy, canTeach);

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

// ---- Camera position + run options --------------------------------------------

void App::DrawDetectViewSection(RecipeStep& s, const nlohmann::json& merged, VisionModel& m,
                                bool busy, bool canTeach)
{
    ImGui::SeparatorText("Camera position");
    const std::string camDev = merged.value("camera_device", std::string("gantry-main"));

    double vp[3] = {0.0, 0.0, 0.0};
    const bool taught = ViewPointOf(merged, vp);
    if (taught)
        ImGui::Text("X %.4f  Y %.4f  Z %.4f", vp[0], vp[1], vp[2]);
    else
        ImGui::TextDisabled("not taught - the step detects wherever the camera stands");
    ImGui::SameLine();
    ImGui::TextDisabled("(%s axis)", camDev.c_str());

    // The guide: what the camera saw when the view was taught. The centre half
    // of the frame - where the object should sit for the next person teaching.
    if (EnsureViewThumbTex(m)) {
        const float w = std::min(ImGui::GetContentRegionAvail().x, 360.0f);
        const float h = w * static_cast<float>(visionViewTex_.h) /
                        static_cast<float>(visionViewTex_.w);
        ImGui::Image((ImTextureID)(intptr_t)visionViewTex_.tex, ImVec2(w, h));
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The taught view: the centre half of the frame, snapped by\n"
                              "Teach here. Put the camera where this picture was taken.");
    } else {
        ImGui::TextDisabled("(no thumbnail - Teach here takes one)");
    }

    // The live pose, for Teach here (and a fast poll while the window is up, so
    // it is fresh when the button is pressed - the PointXyz form's habit).
    double live[3] = {0.0, 0.0, 0.0};
    bool   hasLive = false;
    const int devIdx = MotionIndexOf(hardware_, camDev);
    IMotionController* mc =
        devIdx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(devIdx)) : nullptr;
    if (mc && mc->IsConnected()) {
        mc->RequestFastPoll();
        const std::map<std::string, double> pos = mc->Positions();
        static const char* kAx[3] = {"X", "Y", "Z"};
        hasLive = true;
        for (int a = 0; a < 3 && hasLive; ++a) {
            const auto it = pos.find(kAx[a]);
            hasLive = it != pos.end();
            if (hasLive) live[a] = it->second;
        }
    }

    // Go drives a device: the jog arm and the whole orchestration gate it.
    ImGui::BeginDisabled(busy || !taught || !jogArmed_);
    if (ImGui::Button("Go")) StartDetectViewGo(merged);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        if (!jogArmed_)
            ImGui::SetTooltip("Arm jog first (Jog window) - this drives %s.", camDev.c_str());
        else
            ImGui::SetTooltip("Drive %s to the taught view, the way a run does\n"
                              "(via the safe Z when Z safe lift is on).", camDev.c_str());
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || !hasLive || !canTeach);
    if (ImGui::Button("Teach here")) {
        s.params["view_point"] = {{"X", live[0]}, {"Y", live[1]}, {"Z", live[2]}};
        // The exposure the camera is at now goes into the step's model - the
        // one place a run reads it from. The light is the model's already: it
        // is set (and lit) in the camera strip under the image.
        if (hardware_.CameraState() == ConnState::Connected) {
            const double expo = hardware_.Camera().ExposureUs();
            if (expo > 0.0) m.exposureUs = expo;
        }
        // The thumbnail: the centre half of the frame, grey, beside the model.
        const RoiFilterFrame f = MainCameraFrame();
        bool snapped = false;
        if (f.Valid()) {
            std::vector<unsigned char> gray;
            const FocusRoi want{f.w / 4, f.h / 4, f.w / 2, f.h / 2};
            const FocusRoi got = RgbToGray8(f.rgb, f.w, f.h, want, gray);
            if (got.w > 0 && got.h > 0) {
                std::error_code ec;
                std::filesystem::create_directories(session_.VisionDir(), ec);
                snapped = SaveGrayPgm(session_.VisionDir() + "/" + ViewThumbFile(m.uid),
                                      gray.data(), got.w, got.h);
                visionViewTex_.key.clear();   // reload the new picture
            }
        }
        char buf[200];
        std::snprintf(buf, sizeof(buf), "view taught at X %.4f Y %.4f Z %.4f, exposure %.0f us%s",
                      live[0], live[1], live[2], m.exposureUs,
                      snapped ? ", thumbnail saved" : " - NO thumbnail (no camera frame)");
        visionStatus_ = buf;
        FD_INFO("vision", "'%s': %s", m.name.c_str(), buf);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        if (!canTeach)
            ImGui::SetTooltip("Teaching the view writes the detection model too - Admin.");
        else if (!hasLive)
            ImGui::SetTooltip("%s: not connected, or reports no X/Y/Z", camDev.c_str());
        else
            ImGui::SetTooltip("Record, in one go:\n"
                              "  %s at X %.4f  Y %.4f  Z %.4f (into the step)\n"
                              "  the camera's current exposure (into the model)\n"
                              "  a thumbnail of the centre half of the frame",
                              camDev.c_str(), live[0], live[1], live[2]);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || !taught);
    if (ImGui::Button("Clear")) s.params.erase("view_point");
    ImGui::EndDisabled();

    ImGui::TextDisabled("exposure %.0f us, light %s - the model's, applied before the move",
                        m.exposureUs, AlignLightingSummary(m.lighting).c_str());

    // ---- Run options ----------------------------------------------------------
    ImGui::SeparatorText("Run");
    ImGui::BeginDisabled(busy);
    bool zLift = merged.value("z_lift", true);
    if (ImGui::Checkbox("Z safe lift", &zLift)) s.params["z_lift"] = zLift;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("On: Z goes to the safe Z first, then X/Y across, then Z down to\n"
                          "the taught height. Off: one direct X/Y/Z move.");
    ImGui::SameLine();
    ImGui::BeginDisabled(!zLift);
    double safeZ = merged.value("safe_z_mm", 25.0);
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::InputDouble("safe Z (axis)", &safeZ, 0.0, 0.0, "%.3f"))
        s.params["safe_z_mm"] = safeZ;
    ImGui::EndDisabled();

    bool center = merged.value("center_after", false);
    if (ImGui::Checkbox("Center the camera on the result", &center))
        s.params["center_after"] = center;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("After a result, move %s X/Y onto it (Z unchanged) - the\n"
                          "same drive move_to_result uses.", camDev.c_str());
    ImGui::SameLine();
    ImGui::BeginDisabled(!center);
    double cmax = merged.value("center_max_mm", 5.0);
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::InputDouble("max mm", &cmax, 0.0, 0.0, "%.2f")) s.params["center_max_mm"] = cmax;
    ImGui::EndDisabled();

    bool manual = merged.value("manual_fallback", false);
    if (ImGui::Checkbox("Nothing found: ask the operator (P = 0)", &manual))
        s.params["manual_fallback"] = manual;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off: a miss fails the step (or sets %s_ok = false when\n"
                          "require is off).\n"
                          "On: the crosshair window opens; the point the operator\n"
                          "confirms is the result, marked manual, with P = 0.",
                          merged.value("ctx_prefix", std::string("det")).c_str());
    ImGui::EndDisabled();
}

void App::StartDetectViewGo(const nlohmann::json& merged)
{
    double vp[3] = {0.0, 0.0, 0.0};
    if (!ViewPointOf(merged, vp)) return;
    const std::string camDev = merged.value("camera_device", std::string("gantry-main"));
    const bool   zLift = merged.value("z_lift", true);
    const double safeZ = merged.value("safe_z_mm", 25.0);
    const double vel   = merged.value("velocity_mm_s", 0.0);

    Recipe one;
    one.uid  = "detect-view-go";
    one.name = "Go to the detect view";
    const auto leg = [&](const char* axes, const nlohmann::json& t) {
        RecipeStep st;
        st.type     = "move_axis_to";
        st.nickname = std::string("detect view: ") + axes;
        st.params   = {{"device", camDev}, {"axes", axes}};
        for (const auto& el : t.items()) st.params[el.key()] = el.value();
        if (vel > 0.0) st.params["velocity_mm_s"] = vel;
        one.steps.push_back(std::move(st));
    };
    if (zLift) {
        leg("Z", {{"z_mm", safeZ}});
        leg("XY", {{"x_mm", vp[0]}, {"y_mm", vp[1]}});
        leg("Z", {{"z_mm", vp[2]}});
    } else {
        leg("XYZ", {{"x_mm", vp[0]}, {"y_mm", vp[1]}, {"z_mm", vp[2]}});
    }
    std::string err;
    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &err)) {
        visionStatus_ = "Go refused: " + err;
        FD_WARN("vision", "detect view Go refused: %s", err.c_str());
    } else {
        visionStatus_ = "driving to the taught view";
    }
}

bool App::EnsureViewThumbTex(const VisionModel& m)
{
    namespace fs = std::filesystem;
    const std::string path = session_.VisionDir() + "/" + ViewThumbFile(m.uid);
    std::error_code ec;
    const auto t = fs::last_write_time(path, ec);
    if (ec) {
        visionViewTex_.key.clear();
        visionViewTex_.w = visionViewTex_.h = 0;
        return false;
    }
    // Keyed on the file and its write time: a re-teach rewrites the same name.
    const std::string key = path + "|" + std::to_string(t.time_since_epoch().count());
    if (key != visionViewTex_.key) {
        visionViewTex_.key = key;
        std::vector<unsigned char> gray;
        int w = 0, h = 0;
        if (LoadGrayPgm(path, &gray, &w, &h) && w > 0 && h > 0) {
            std::vector<unsigned char> rgb(static_cast<std::size_t>(w) * h * 3);
            for (std::size_t i = 0; i < gray.size(); ++i)
                rgb[i * 3 + 0] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = gray[i];
            visionViewTex_.tex = host_.EnsureTextureRGB(visionViewTex_.tex, w, h, rgb.data());
            visionViewTex_.w   = w;
            visionViewTex_.h   = h;
        } else {
            visionViewTex_.w = visionViewTex_.h = 0;
        }
    }
    return visionViewTex_.tex != 0 && visionViewTex_.w > 0 && visionViewTex_.h > 0;
}

} // namespace firedog
