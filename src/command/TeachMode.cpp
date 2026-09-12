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

std::size_t Walk(std::vector<RecipeStep>& steps, const CommandRegistry& reg)
{
    std::size_t movers = 0;
    for (RecipeStep& s : steps) {
        if (!s.enabled) continue;
        if (s.type == "group" || s.type == "sub_recipe") {
            movers += Walk(s.children, reg);
        } else if (s.type == "loop") {
            s.params = nlohmann::json::object();
            s.params["count"] = 1;
            movers += Walk(s.children, reg);
        } else if (s.type == "if") {
            // Then-branch first, else-branch after, as one group. The else rows
            // lose their editor highlight for the run (they never had a
            // correct one - RecipeEngine.h documents the quirk).
            s.type = "group";
            s.params = nlohmann::json::object();
            for (RecipeStep& e : s.elseChildren) s.children.push_back(std::move(e));
            s.elseChildren.clear();
            movers += Walk(s.children, reg);
        } else if (s.type == "stop_recipe") {
            s.enabled = false;
        } else {
            const CommandSpec* spec = reg.Find(s.type);
            if (spec && DeclaresTeach(*spec)) {
                s.params[kTeachParamName] = true;
                ++movers;
            } else {
                // Unknown commands are disabled too: a teach run is not the
                // place to discover a typo, the next real run reports it.
                s.enabled = false;
            }
        }
    }
    return movers;
}

} // namespace

std::size_t MakeTeachRecipe(Recipe& r, const CommandRegistry& reg)
{
    return Walk(r.steps, reg);
}

} // namespace firedog
