// ProgramRunner.h : layer 2.5 — sequences RecipeEngine runs to execute a
// program, one slot, or one selected recipe (ONE code path: a resolved queue).
//
// No thread and no engine of its own: App ticks it every frame (the path-
// executor precedent) and each queue item is an ordinary RecipeEngine run —
// so every recipe keeps its own validation, progress, prompts, $run DUT
// event, and stop behavior. Stop() marks the queue dead then RequestStop()s
// the running engine, which already propagates into every command wait and
// any Lua sub-run: one button, the whole chain dies.
//
// At the end of a program/slot run (any terminal state) it writes a $run DUT
// event {kind, name, uid, result, recipes_done}; single-recipe runs add
// nothing extra (the recipe's own $run covers it).
// Design: docs/run-page-design.md.
#pragma once

#include "command/Recipe.h"
#include "command/RecipeEngine.h"

#include <cstddef>
#include <string>
#include <vector>

namespace firedog {

class ProgramRunner {
public:
    explicit ProgramRunner(RecipeEngine& engine) : engine_(engine) {}

    // A resolved queue item (recipe snapshotted at Start — a run executes
    // against one consistent world even if the library is edited meanwhile).
    struct Item {
        std::string slotName;
        Recipe      recipe;
    };
    // Per-item outcome for the run view.
    enum class Mark { Pending, Running, Done, Failed, Stopped };

    // kind: "program" | "slot" | "recipe" (controls the $run event; "recipe"
    // writes none). `uid` = program uid ("" for ad-hoc recipe runs).
    bool Start(std::string kind, std::string label, std::string uid,
               std::vector<Item> queue, RecipeEngine::Env env, std::string* err);
    void Tick();    // call every frame from App::Update
    void Stop();    // kill the queue + stop the running recipe

    bool Active() const { return active_; }

    struct ItemStatus {
        std::string slotName, recipeName;
        Mark        mark = Mark::Pending;
    };
    struct Status {
        bool        active = false;
        std::string kind, label;          // what is running (or last ran)
        std::size_t index = 0, total = 0; // current item / queue size
        std::string result;               // terminal summary ("" while active)
        std::vector<ItemStatus> items;
    };
    const Status& GetStatus() const { return status_; }   // UI-thread only

private:
    void Finish(const char* result, const std::string& detail);

    RecipeEngine& engine_;

    bool active_        = false;
    bool stopReq_       = false;
    bool waitingEngine_ = false;
    std::size_t idx_    = 0;
    std::string kind_, label_, uid_;
    std::vector<Item>  queue_;
    RecipeEngine::Env  env_;
    Status status_;
};

} // namespace firedog
