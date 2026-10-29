// DryRun.h : layer 2.5 — a DRY run as a recipe transform.
//
// A dry run exercises the machine's MECHANICS and nothing else: motion, IO
// outputs/inputs and pneumatic slides run as written, while every prompt,
// instrument read/write, vision step, script, DUT write and document write is
// skipped. Its use is a looping folder that walks a program's moves over and
// over without a part, an operator or the instruments — a motion soak.
//
// Same shape as teach mode (command/TeachMode.h), on purpose: the engine has
// no dry-run mode. RecipeEngine::Start hands its private copy through
// MakeDryRunRecipe and runs the result with the ordinary StepRunner, so
// validation, progress, step paths, stop and the safety abort stay the one
// implementation they already are.
//
// What the transform does, and why:
//   * a leaf whose command is on the ALLOWLIST (DryRun.cpp) stays enabled;
//     every other leaf is DISABLED — not removed, so step paths keep their
//     indices. An allowlist and not a category test: a command added later is
//     skipped until somebody decides it belongs, the only default that cannot
//     fail open (a new scan filed under "Motion" would otherwise start reading
//     a meter in the middle of a soak);
//   * control flow is LEFT ALONE — loops, ifs, parallel blocks and
//     stop_recipe run as written. An if whose key a skipped step would have
//     written sees it missing (comparisons on missing data are false), so it
//     takes its else-branch; that is the honest answer, not a guess;
//   * a move whose input came from a skipped step (a ctx key a measurement
//     would have written) FAILS on the missing key rather than moving
//     somewhere invented — the dry run then stops and says which step.
// Idempotent, so Start applies it before validation (a disconnected meter
// behind a skipped step does not refuse the run) and again after sub-recipes
// are inlined.
//
// A dry run is not a run of the part: no $run DUT event, no deck mark, no
// cycle advance, no cycle values (RecipeEngine / ProgramRunner / App).
#pragma once

#include "command/Recipe.h"

#include <cstddef>
#include <string>

namespace firedog {

class CommandRegistry;

// True when a dry run keeps this command.
bool IsDryRunCommand(const std::string& name);

// Rewrites `r` in place; returns how many leaves remain enabled (0 is legal —
// the run then completes at once and the log says so).
std::size_t MakeDryRunRecipe(Recipe& r, const CommandRegistry& reg);

} // namespace firedog
