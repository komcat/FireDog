// Orchestration.h : layer 2.5 — the one "who is driving the machine" owner.
//
// The command registry, both engines and the program runner are a single unit:
// they are constructed together, they must be destroyed together and BEFORE
// Workflow/HardwareManager (the engine dtors join their worker threads while
// the hardware is still alive — same discipline as SafetyWatch), and every Run
// surface in the UI has to check all of them plus the path executor before it
// starts anything. Bundling them into one member keeps that contract in one
// place instead of four parallel member slots that can drift apart.
#pragma once

#include "command/CommandRegistry.h"
#include "command/LuaEngine.h"
#include "command/ProgramRunner.h"
#include "command/RecipeEngine.h"
#include "workflow/Workflow.h"

namespace firedog {

class HardwareManager;

struct Orchestration {
    Orchestration(Workflow& wf, HardwareManager& hw)
        : workflow(wf),
          recipe(wf, hw, registry),
          lua(wf, hw, registry),
          programs(recipe) {}

    Orchestration(const Orchestration&) = delete;
    Orchestration& operator=(const Orchestration&) = delete;

    // Declaration order IS the contract: the engines hold a const ref to the
    // registry, the runner drives the recipe engine, and destruction runs in
    // reverse (runner, lua, recipe, registry).
    Workflow&       workflow;    // path executor: gated by Busy, stopped by StopAll
    CommandRegistry registry;    // registered once after Workflow::Start, read-only after
    RecipeEngine    recipe;
    LuaEngine       lua;
    ProgramRunner   programs;    // ticked from App::Update

    // One orchestration owner at a time. EVERY Run/Go surface gates on this —
    // note the program runner is momentarily idle between queue items, so
    // checking the engines alone would let a second run slip in.
    bool Busy() const
    {
        return recipe.Running() || lua.Running() ||
               programs.Active() || workflow.PathActive();
    }

    // Software big-red-button: abandon every run and stop all motion.
    void StopAll();
};

} // namespace firedog
