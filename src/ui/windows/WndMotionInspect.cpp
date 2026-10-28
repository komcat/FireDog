// WndMotionInspect.cpp : the floating Inspect Motion window (the Run strip's
// "Inspect Motion" button). Definitions of App members — see App.h.
//
// What the deck's Execute would MOVE, how fast, and at what camera exposure,
// read straight off the recipes it would launch: the selected item, or the
// whole slot when nothing is selected — the same scope Execute uses. Read-only,
// so it stays live while a run is going and raises no modal. Sub-recipes are
// inlined through InlineSubRecipes, the call the engine makes at Start, so a
// move buried in a called recipe is listed where it runs.
//
// velocity_mm_s persists on the device and an exposure on the camera, so a step
// that leaves either at 0 inherits whatever an earlier step set. The window
// carries both forward in walk order and shows the inherited value dimmed with
// a '*'. That is an ESTIMATE, and says so: a loop or an if decides at run time
// what came before, and the value a device holds before the first setter is
// not known here at all ("current").
#include "App.h"

#include "imgui.h"

#include "command/CommandRegistry.h"
#include "command/Program.h"
#include "command/Recipe.h"

#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace firedog {

namespace {

// A value that persists past the step that set it; `at` names that step.
struct Carried {
    double      v = 0.0;
    std::string at;
};

struct InspectWalk {
    const CommandRegistry&          reg;
    const SavedPosMap&              positions;
    const std::vector<VisionModel>& models;
    bool                            all;        // every step, not only motion
    std::map<std::string, Carried>  vel;        // device -> last velocity set
    Carried                         expo;       // v == 0 with `at` set: a station's
    int                             rows = 0;   // rows drawn for the current recipe
};

const ImVec4 kHeadCol(0.55f, 0.75f, 1.0f, 1.0f);
const ImVec4 kSetCol(1.0f, 0.85f, 0.45f, 1.0f);   // a value THIS step sets

// align_station (and align_run, which runs it) sets the exposure from the
// station / its detection tiers, not from a param.
bool ExposureFromStation(const std::string& cmd)
{
    return cmd == "align_station" || cmd == "align_run";
}

const ParamSpec* FindParam(const CommandSpec& spec, const char* name)
{
    for (const ParamSpec& ps : spec.params)
        if (ps.name == name) return &ps;
    return nullptr;
}

// The number a step runs with: the stored value, else the schema default, else 0.
double EffectiveNum(const CommandSpec& spec, const nlohmann::json& params, const char* name)
{
    const ParamSpec* ps = FindParam(spec, name);
    if (!ps) return 0.0;
    if (params.is_object()) {
        const auto it = params.find(name);
        if (it != params.end() && it->is_number()) return it->get<double>();
    }
    return ps->defaultValue.is_number() ? ps->defaultValue.get<double>() : 0.0;
}

bool IsMotionStep(const CommandSpec& spec)
{
    return FindParam(spec, "velocity_mm_s") || FindParam(spec, "exposure_us") ||
           FindParam(spec, "approach_velocity_mm_s") ||
           spec.name.rfind("move", 0) == 0 || spec.name == "stop_motion" ||
           spec.name == "camera_exposure" || ExposureFromStation(spec.name);
}

const VisionModel* FindModel(const std::vector<VisionModel>& models, const std::string& key)
{
    for (const VisionModel& m : models)
        if (m.uid == key) return &m;
    for (const VisionModel& m : models)
        if (m.name == key) return &m;
    return nullptr;
}

// Where the step goes, in words: saved positions by NAME, a taught point, a
// detection model, and the explicit axis / delta params of the relative moves.
std::string StepTarget(const CommandSpec& spec, const nlohmann::json& params,
                       const std::string& device, const InspectWalk& w)
{
    std::string out;
    const auto add = [&out](const std::string& s) {
        if (s.empty()) return;
        if (!out.empty()) out += "  ";
        out += s;
    };
    for (const ParamSpec& ps : spec.params) {
        const auto it = params.is_object() ? params.find(ps.name) : params.end();
        const bool stored = params.is_object() && it != params.end();
        if (ps.type == ParamType::PositionUid) {
            const std::string uid = EffectiveString(spec, params, ps.name);
            if (uid.empty()) continue;
            std::string name = uid;
            const auto dit = w.positions.find(device);
            if (dit != w.positions.end())
                for (const SavedPos& sp : dit->second)
                    if (sp.uid == uid) { name = sp.name; break; }
            // move_path names two: "from A  to B".
            add(ps.name == "position" ? name : ps.name + " " + name);
        } else if (ps.type == ParamType::PointXyz) {
            if (stored && it->is_object()) {
                char b[96];
                std::snprintf(b, sizeof(b), "X %.3f Y %.3f Z %.3f", it->value("X", 0.0),
                              it->value("Y", 0.0), it->value("Z", 0.0));
                add(b);
            }
        } else if (ps.type == ParamType::VisionUid) {
            const std::string key = EffectiveString(spec, params, ps.name);
            const VisionModel* m = FindModel(w.models, key);
            add(m ? m->name : key);
        } else if (stored && it->is_string() &&
                   (ps.name == "axis" || ps.name == "axes" || ps.name == "lift_axis")) {
            add(it->get<std::string>());
        } else if (stored && it->is_number() &&
                   (ps.name == "delta_mm" || ps.name == "dx_mm" || ps.name == "dy_mm" ||
                    ps.name == "dz_mm" || ps.name == "x_mm" || ps.name == "y_mm" ||
                    ps.name == "z_mm" || ps.name == "lift_mm")) {
            char b[48];
            std::snprintf(b, sizeof(b), "%s %+.4g", ps.name.c_str(), it->get<double>());
            add(b);
        }
    }
    return out;
}

void InheritedTip(const char* what, const std::string& at)
{
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Inherited: this step leaves the %s alone, so it runs at\n"
                          "what step %s set (an estimate - loops and ifs decide at run time).",
                          what, at.c_str());
}

// One command step: a table row (when it is a motion step, or `all`), and the
// carried velocity / exposure updated for the steps after it.
void InspectLeaf(InspectWalk& w, const RecipeStep& s, const std::string& path, bool live)
{
    const CommandSpec* spec = w.reg.Find(s.type);
    if (!spec) {
        if (!w.all) return;
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("%s", path.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s (unknown)", s.type.c_str());
        ++w.rows;
        return;
    }
    const bool motion = IsMotionStep(*spec);
    if (!motion && !w.all) return;

    const nlohmann::json& p = s.params;
    const std::string device = EffectiveString(*spec, p, "device");
    const bool hasVel  = FindParam(*spec, "velocity_mm_s") != nullptr;
    const double vel   = hasVel ? EffectiveNum(*spec, p, "velocity_mm_s") : 0.0;
    const double apprV = EffectiveNum(*spec, p, "approach_velocity_mm_s");
    const double apprMm  = EffectiveNum(*spec, p, "approach_mm");
    const double apprPct = EffectiveNum(*spec, p, "approach_pct");

    // What this step does to the camera: a number it sets (expoSet > 0), the
    // station's (fromStation), or nothing.
    double expoSet = 0.0;
    std::string expoNote;
    const bool fromStation = ExposureFromStation(spec->name);
    if (spec->name == "camera_exposure") {
        expoSet = EffectiveNum(*spec, p, "us");
    } else if (FindParam(*spec, "exposure_us")) {
        expoSet = EffectiveNum(*spec, p, "exposure_us");
        if (expoSet <= 0.0 && FindParam(*spec, "model")) {
            // vision_detect / vision_test: 0 = the model's own exposure.
            if (const VisionModel* m = FindModel(w.models, EffectiveString(*spec, p, "model"))) {
                expoSet = m->exposureUs;
                if (expoSet > 0.0) expoNote = " model";
            }
        }
    }

    ImGui::TableNextRow();
    if (!live) ImGui::BeginDisabled();

    ImGui::TableSetColumnIndex(0);
    ImGui::TextDisabled("%s", path.c_str());

    ImGui::TableSetColumnIndex(1);
    if (!s.nickname.empty()) ImGui::Text("%s", s.nickname.c_str());
    else                     ImGui::Text("%s", spec->name.c_str());
    if (!live) { ImGui::SameLine(); ImGui::TextDisabled("(off)"); }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("%s\n%s", spec->name.c_str(), spec->description.c_str());

    ImGui::TableSetColumnIndex(2);
    if (!device.empty()) ImGui::Text("%s", device.c_str());

    ImGui::TableSetColumnIndex(3);
    const std::string target = StepTarget(*spec, p, device, w);
    if (!target.empty()) ImGui::TextUnformatted(target.c_str());

    // Velocity (mm/s).
    ImGui::TableSetColumnIndex(4);
    const auto carriedVel = w.vel.find(device);
    if (hasVel && vel > 0.0) {
        ImGui::TextColored(kSetCol, "%.4g", vel);
    } else if (!device.empty() && carriedVel != w.vel.end() && (hasVel || motion)) {
        ImGui::TextDisabled("%.4g*", carriedVel->second.v);
        InheritedTip("velocity", carriedVel->second.at);
    } else if (hasVel || (motion && !device.empty())) {
        ImGui::TextDisabled("current");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("0 = keep whatever velocity the device holds when the\n"
                              "run gets here; no earlier step in this scope sets it.");
    }

    // Approach leg.
    ImGui::TableSetColumnIndex(5);
    if (apprV > 0.0) {
        if (apprMm > 0.0)       ImGui::Text("%.4g, last %.4g mm", apprV, apprMm);
        else if (apprPct > 0.0) ImGui::Text("%.4g, last %.4g%%", apprV, apprPct);
        else                    ImGui::Text("%.4g", apprV);
    }

    // Exposure (us).
    ImGui::TableSetColumnIndex(6);
    if (expoSet > 0.0) {
        ImGui::TextColored(kSetCol, "%.0f%s", expoSet, expoNote.c_str());
    } else if (fromStation) {
        ImGui::TextColored(kSetCol, "station");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Set by the alignment station / its detection tiers\n"
                              "(see the Alignment page), not by this step.");
    } else if (FindParam(*spec, "exposure_us") || spec->name.rfind("move", 0) == 0) {
        // Only where the picture matters: a move arrives and is looked at.
        if (w.expo.v > 0.0) {
            ImGui::TextDisabled("%.0f*", w.expo.v);
            InheritedTip("camera exposure", w.expo.at);
        } else if (!w.expo.at.empty()) {
            ImGui::TextDisabled("station*");
            InheritedTip("camera exposure", w.expo.at);
        } else {
            ImGui::TextDisabled("current");
        }
    }

    if (!live) ImGui::EndDisabled();
    ++w.rows;

    // A disabled step changes nothing for the steps after it.
    if (!live) return;
    if (hasVel && vel > 0.0 && !device.empty()) w.vel[device] = {vel, path};
    if (expoSet > 0.0)    w.expo = {expoSet, path};
    else if (fromStation) w.expo = {0.0, path};
}

void InspectSteps(InspectWalk& w, const std::vector<RecipeStep>& steps,
                  const std::string& prefix, bool live)
{
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const RecipeStep& s = steps[i];
        const std::string path = prefix + std::to_string(i + 1);
        const bool on = live && s.enabled;
        if (IsStructuralStep(s.type)) {
            if (w.all) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextDisabled("%s", path.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("[%s]%s%s", s.type.c_str(),
                                    s.nickname.empty() ? "" : " ", s.nickname.c_str());
                ++w.rows;
            }
            InspectSteps(w, s.children, path + ".", on);
            if (!s.elseChildren.empty())
                InspectSteps(w, s.elseChildren, path + ".else.", on);
        } else {
            InspectLeaf(w, s, path, on);
        }
    }
}

} // namespace

void App::DrawMotionInspectWindow()
{
    // The button is gated, but the open bit outlives a page switch and a role
    // never should.
    if (!CanUse(kRoleInspect)) { showMotionInspect_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(980.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Inspect Motion", &showMotionInspect_)) {
        ImGui::End();
        return;
    }

    const std::vector<Program>& progs = session_.Programs();
    const Program* prog = (progSel_ >= 0 && progSel_ < static_cast<int>(progs.size()))
                              ? &progs[static_cast<std::size_t>(progSel_)] : nullptr;
    const ProgramSlot* slot =
        (prog && deckSlot_ >= 0 && deckSlot_ < static_cast<int>(prog->slots.size()))
            ? &prog->slots[static_cast<std::size_t>(deckSlot_)] : nullptr;
    if (!slot) {
        ImGui::TextDisabled("No program / slot in the Run deck.");
        ImGui::End();
        return;
    }

    // The same scope Execute runs: the selected node, else the whole slot.
    const ProgramNode* node = deckNode_.empty() ? nullptr : FindNode(slot->items, deckNode_);
    std::vector<ProgramNode> scope;
    if (node) scope.push_back(*node);
    else      scope = slot->items;

    if (node) ImGui::Text("Item: %s", node->IsFolder() ? node->name.c_str() : "");
    else      ImGui::Text("Slot %d: %s", deckSlot_ + 1, slot->name.c_str());
    if (node && !node->IsFolder()) {
        ImGui::SameLine(0.0f, 0.0f);
        const Recipe* r = RecipeByUid(session_.Recipes(), node->recipeUid);
        ImGui::Text("%s", r ? r->name.c_str() : "(missing recipe)");
    }
    ImGui::SameLine(0.0f, 24.0f);
    ImGui::Checkbox("Every step", &motionInspectAll_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("List every step, not only moves and the steps that set\n"
                          "a velocity or the camera exposure.");
    ImGui::TextDisabled("Velocity mm/s, exposure us. Bright = set by that step;  "
                        "* = inherited from an earlier step (persists on the device / "
                        "camera);  current = not set in this scope.");
    ImGui::Separator();

    InspectWalk w{orch_.registry, session_.Positions(), session_.VisionModels(),
                  motionInspectAll_, {}, {}, 0};

    constexpr ImGuiTableFlags kFlags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_BordersOuter |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("##motinspect", 7, kFlags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Step",     ImGuiTableColumnFlags_WidthStretch, 0.6f);
        ImGui::TableSetupColumn("Command",  ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableSetupColumn("Device",   ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Target",   ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableSetupColumn("Velocity", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("Approach", ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableSetupColumn("Exposure", ImGuiTableColumnFlags_WidthStretch, 0.9f);
        ImGui::TableHeadersRow();

        // Walk the program tree in run order. A folder is listed ONCE whatever
        // its loop count: the question is what the steps say, not how often.
        const auto walk = [&](const auto& self, const std::vector<ProgramNode>& items,
                              int depth) -> void {
            for (const ProgramNode& n : items) {
                ImGui::PushID(n.uid.c_str());
                const std::string indent(static_cast<std::size_t>(depth) * 2, ' ');
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                       ImGui::GetColorU32(ImGuiCol_TableHeaderBg));
                ImGui::TableSetColumnIndex(1);
                if (n.IsFolder()) {
                    ImGui::TextColored(kHeadCol, "%s[folder] %s  x%d", indent.c_str(),
                                       n.name.c_str(), n.loop);
                    self(self, n.children, depth + 1);
                    ImGui::PopID();
                    continue;
                }
                const Recipe* r = RecipeByUid(session_.Recipes(), n.recipeUid);
                if (!r) {
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                                       "%s(missing recipe %s)", indent.c_str(),
                                       n.recipeUid.c_str());
                    ImGui::PopID();
                    continue;
                }
                ImGui::TextColored(kHeadCol, "%s%s", indent.c_str(), r->name.c_str());
                // Inline on a copy, as the engine does at Start; a broken
                // reference still lists what the recipe itself holds.
                Recipe run = *r;
                std::string err;
                if (!InlineSubRecipes(run, session_.Recipes(), &err)) {
                    run = *r;
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", err.c_str());
                }
                w.rows = 0;
                InspectSteps(w, run.steps, "", true);
                if (w.rows == 0) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextDisabled("(no motion steps)");
                }
                ImGui::PopID();
            }
        };
        walk(walk, scope, 0);
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace firedog
