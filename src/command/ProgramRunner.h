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

#include "command/Program.h"
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
        std::string path;      // folder breadcrumb inside the slot ("" at top level)
        std::string nodeUid;   // the tree node this run came from (deck highlight)
        int         pass   = 1;  // loop pass of the innermost folder, 1-based
        int         passes = 1;  // that folder's loop count (1 = not in a loop)
        bool        dryRun = false;  // under a dry-run folder (command/DryRun.h)
        Recipe      recipe;
    };
    // Per-item outcome for the run view.
    enum class Mark { Pending, Running, Done, Failed, Stopped };

    // kind: "slot" | "folder" | "recipe" (controls the $run event; "recipe"
    // writes none — the recipe's own $run already covers a single-recipe run).
    // `uid` = program uid ("" for ad-hoc recipe runs).
    // `opts` is re-applied to EVERY queue item's engine Start: the runner goes
    // idle between items, so it carries its own copy rather than reading a
    // toggle that may have moved. A teach run (opts.teach) writes no $run.
    // An item's own `dryRun` is OR-ed into its copy: one queue may mix dry
    // and real items (a slot holding a dry folder), so it is per item.
    bool Start(std::string kind, std::string label, std::string uid,
               std::vector<Item> queue, RecipeEngine::Env env, std::string* err,
               RecipeEngine::StartOpts opts = {});
    void Tick();    // call every frame from App::Update
    void Stop();    // kill the queue + stop the running recipe

    bool Active() const { return active_; }

    struct ItemStatus {
        std::string slotName, recipeName, path, nodeUid;
        int         pass = 1, passes = 1;
        bool        dryRun = false;
        Mark        mark = Mark::Pending;
    };
    struct Status {
        bool        active = false;
        std::string kind, label;          // what is running (or last ran)
        std::size_t index = 0, total = 0; // current item / queue size
        std::string result;               // terminal summary ("" while active)
        std::vector<ItemStatus> items;
        // Outlives the run (like the rest of this struct), so the frame that
        // sees the runner go idle can still tell a teach run from a real one.
        bool        teach = false;
        // EVERY item ran dry. Then the run as a whole is not a run of the part:
        // no $run, no cycle reset/advance. A mixed queue is a real run whose
        // dry items simply leave no accent (ItemStatus::dryRun).
        bool        dryRun = false;
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
    RecipeEngine::StartOpts opts_;
    Status status_;
};

// ---- Run-request resolution ------------------------------------------------
// The two granularities the Run deck offers. They differ ONLY in how much of
// the program ends up in the queue — one code path from here on.
//
// There is deliberately NO whole-program scope. A multi-slot program is a
// station sequence an operator walks — part in, run this slot, part moves on —
// so launching every slot back-to-back from one press would drive the machine
// through stations whose parts are not there yet.
enum class RunScope { Slot, Node };

// A resolved request, ready for ProgramRunner::Start.
struct RunQueue {
    std::string kind;    // "slot" | "recipe" — drives the $run event
    std::string label;   // human-readable, e.g. "prog / slot / recipe"
    std::vector<ProgramRunner::Item> items;
};

// Resolve a deck selection into a queue. `slotIdx` is always required;
// `nodeUid` is read only by RunScope::Node (empty is fine otherwise) and may
// name EITHER a recipe node or a folder — a folder resolves to its whole
// subtree, loops included, which is the only way to run one repeated group
// without running the slot around it.
//
// FOLDER LOOPS ARE EXPANDED HERE, at Start, into plain queue items: the runner
// stays a flat sequence of ordinary RecipeEngine runs, so a loop cannot invent
// a second execution path with its own stop/prompt/traceability behaviour. The
// cost is that the queue length is the EXPANDED length — hence kMaxQueueItems,
// which refuses a mistyped loop count up front instead of after 40 minutes.
//
// A folder's `dryRun` marks every item beneath it, nested folders included;
// a single recipe node selected inside a dry folder runs dry as well — the
// deck shows it inside that folder, so that is what pressing it must mean.
//
// Fails (empty `items`, reason in *err) on an out-of-range selection or a
// recipe uid the library does not contain. A missing uid is a HARD stop, not a
// skipped item: a program that silently runs 4 of its 5 recipes is worse than
// one that refuses to start. Recipes are snapshotted into the queue, so the run
// executes against one consistent world even if the library is edited meanwhile.
RunQueue BuildRunQueue(const Program& prog, RunScope scope,
                       int slotIdx, const std::string& nodeUid,
                       const std::vector<Recipe>& library, std::string* err);

} // namespace firedog
