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
#pragma once

#include "command/Recipe.h"

#include <cstddef>

namespace firedog {

class CommandRegistry;

// Rewrites `r` in place; returns how many teach movers remain enabled (0 is
// legal - the run then completes at once and the log says so).
std::size_t MakeTeachRecipe(Recipe& r, const CommandRegistry& reg);

} // namespace firedog
