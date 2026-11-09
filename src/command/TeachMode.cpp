// TeachMode.cpp — see TeachMode.h for what the transform does and why.
#include "command/TeachMode.h"

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "command/MotionWaits.h"   // kTeachParamName

#include <utility>

namespace firedog {

namespace {

bool DeclaresTeach(const CommandSpec& spec)
{
    for (const ParamSpec& p : spec.params)
        if (p.name == kTeachParamName) return true;
    return false;
}

std::size_t Walk(std::vector<RecipeStep>& steps, const CommandRegistry& reg,
                 const std::set<std::string>* only)
{
    std::size_t movers = 0;
    for (RecipeStep& s : steps) {
        if (!s.enabled) continue;
        if (s.type == "group" || s.type == "sub_recipe") {
            movers += Walk(s.children, reg, only);
        } else if (s.type == "parallel") {
            // One stage at a time: a teach run puts the operator at the Manual
            // Adjust window for each mover, and two lanes prompting at once
            // would be two windows for one pair of hands. The lanes still run,
            // just in order — the taught positions are the same either way.
            s.type = "group";
            s.params = nlohmann::json::object();
            movers += Walk(s.children, reg, only);
        } else if (s.type == "loop") {
            s.params = nlohmann::json::object();
            s.params["count"] = 1;
            movers += Walk(s.children, reg, only);
        } else if (s.type == "if") {
            // Then-branch first, else-branch after, as one group. The else rows
            // lose their editor highlight for the run (they never had a
            // correct one - RecipeEngine.h documents the quirk).
            s.type = "group";
            s.params = nlohmann::json::object();
            for (RecipeStep& e : s.elseChildren) s.children.push_back(std::move(e));
            s.elseChildren.clear();
            movers += Walk(s.children, reg, only);
        } else if (s.type == "stop_recipe") {
            s.enabled = false;
        } else {
            const CommandSpec* spec = reg.Find(s.type);
            if (spec && DeclaresTeach(*spec)) {
                // Set explicitly either way: an unpicked mover must not prompt
                // even if its author ticked `teach` on the step itself.
                const bool teach =
                    !only || only->count(TeachKey(s.params.value("device", std::string()),
                                                  TeachTargetUid(s))) > 0;
                s.params[kTeachParamName] = teach;
                if (teach) ++movers;
            } else {
                // Unknown commands are disabled too: a teach run is not the
                // place to discover a typo, the next real run reports it.
                s.enabled = false;
            }
        }
    }
    return movers;
}

void Collect(const std::vector<RecipeStep>& steps, std::vector<TeachTarget>& out)
{
    for (const RecipeStep& s : steps) {
        if (!s.enabled) continue;
        // After the transform the only containers left are group / sub_recipe
        // / loop (parallel and if became groups), all with plain children.
        if (!s.children.empty()) {
            Collect(s.children, out);
            continue;
        }
        if (!s.params.is_object() || !s.params.value(kTeachParamName, false)) continue;
        out.push_back({s.params.value("device", std::string()), TeachTargetUid(s)});
    }
}

} // namespace

std::string TeachKey(const std::string& device, const std::string& positionUid)
{
    return device + '\x1f' + positionUid;   // unit separator: in neither name
}

std::string TeachTargetUid(const RecipeStep& s)
{
    if (!s.params.is_object()) return {};
    return s.params.value(s.type == "move_path" ? "to" : "position", std::string());
}

std::size_t MakeTeachRecipe(Recipe& r, const CommandRegistry& reg,
                            const std::set<std::string>* only)
{
    return Walk(r.steps, reg, only);
}

std::vector<TeachTarget> ListTeachTargets(const Recipe& r, const CommandRegistry& reg,
                                          const std::vector<Recipe>& library)
{
    // Exactly RecipeEngine::Start's order: transform, inline, transform again.
    Recipe run = r;
    MakeTeachRecipe(run, reg);
    std::string err;
    if (InlineSubRecipes(run, library, &err)) MakeTeachRecipe(run, reg);
    std::vector<TeachTarget> out;
    Collect(run.steps, out);
    return out;
}

} // namespace firedog
