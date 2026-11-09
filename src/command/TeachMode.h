// TeachMode.h : layer 2.5 — a TEACH run as a recipe transform.
//
// Teach mode walks a recipe's saved-position moves and nothing else, and after
// each arrival lets the operator fine-tune and Save the position (the movers'
// own `teach` param, MotionWaits.h). The engine has NO teach mode: Start hands
// its private copy of the recipe through MakeTeachRecipe and runs the result
// with the ordinary StepRunner, so validation, progress, step paths, stop and
// step mode all stay the one implementation they already are.
//
// What the transform does, and why:
//   * a leaf whose command declares `teach` keeps running with teach = true;
//     every other leaf is DISABLED - not removed, so step paths (the editor's
//     highlight, the runner's status) keep their indices;
//   * a loop runs its body ONCE - the same position taught ten times is ten
//     prompts for one answer;
//   * an `if` becomes a group of then-children FOLLOWED BY its else-children -
//     the condition reads values the skipped commands never wrote, and both
//     branches hold moves that need teaching;
//   * stop_recipe is disabled - a reject-path stop must not end a teach walk.
// Idempotent, so Start may apply it before validation and again after
// sub-recipes are inlined (their bodies need the same treatment).
//
// A PICK narrows which positions stop: the operator ticks them in the Run
// page's Teach positions window before anything moves. A mover whose target is
// not picked still MOVES (teach = false) - the run follows the program to the
// next picked position - it just does not open Manual Adjust. The pick is keyed
// by device + position uid (TeachKey), so one position visited three times
// stops three times, exactly as an unpicked teach run would.
#pragma once

#include "command/Recipe.h"

#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

// The pick key of a position on a device.
std::string TeachKey(const std::string& device, const std::string& positionUid);

// The saved position a teach mover writes: `to` for move_path (the node it
// arrives at), `position` for every other mover. "" when the step names none.
std::string TeachTargetUid(const RecipeStep& s);

// Rewrites `r` in place; returns how many movers will PROMPT (0 is legal - the
// run then completes at once and the log says so). `only` = nullptr teaches
// every mover; a set (TeachKey) prompts only at those, the rest move silently.
std::size_t MakeTeachRecipe(Recipe& r, const CommandRegistry& reg,
                            const std::set<std::string>* only = nullptr);

// One stop a teach run would make, in run order.
struct TeachTarget {
    std::string device, uid;
};
// What a teach run of `r` would stop at, in order, repeats kept - computed by
// applying the SAME transform to a copy with its sub-recipes inlined, so the
// list cannot disagree with the run. An unresolvable sub_recipe lists what is
// left (the run itself will refuse and say why).
std::vector<TeachTarget> ListTeachTargets(const Recipe& r, const CommandRegistry& reg,
                                          const std::vector<Recipe>& library);

} // namespace firedog
