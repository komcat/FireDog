// ProgramRunner.cpp — UI-thread only (Start/Tick/Stop from App), no locks.
#include "command/ProgramRunner.h"

#include "core/DutSession.h"
#include "core/Log.h"

namespace firedog {

bool ProgramRunner::Start(std::string kind, std::string label, std::string uid,
                          std::vector<Item> queue, RecipeEngine::Env env,
                          std::string* err)
{
    if (active_) {
        if (err) *err = "a run is already active";
        return false;
    }
    if (engine_.Running()) {
        if (err) *err = "the recipe engine is busy";
        return false;
    }
    if (queue.empty()) {
        if (err) *err = "nothing to run (empty queue)";
        return false;
    }

    kind_    = std::move(kind);
    label_   = std::move(label);
    uid_     = std::move(uid);
    queue_   = std::move(queue);
    env_     = std::move(env);
    idx_     = 0;
    stopReq_ = false;
    waitingEngine_ = false;
    active_  = true;

    status_ = Status{};
    status_.active = true;
    status_.kind   = kind_;
    status_.label  = label_;
    status_.total  = queue_.size();
    status_.items.reserve(queue_.size());
    for (const Item& it : queue_)
        status_.items.push_back({it.slotName, it.recipe.name, Mark::Pending});

    FD_INFO("program", "%s '%s' started (%zu recipe run(s))", kind_.c_str(),
            label_.c_str(), queue_.size());
    Tick();   // start the first item immediately
    return active_;
}

void ProgramRunner::Stop()
{
    if (!active_) return;
    stopReq_ = true;
    engine_.RequestStop();   // propagates into command waits + Lua sub-runs
}

void ProgramRunner::Tick()
{
    if (!active_) return;

    if (waitingEngine_) {
        if (engine_.Running()) return;   // current recipe still going
        const RecipeEngine::Status es = engine_.GetStatus();
        waitingEngine_ = false;
        if (es.state == RecipeEngine::State::Completed) {
            status_.items[idx_].mark = Mark::Done;
            ++idx_;
        } else if (es.state == RecipeEngine::State::Stopped) {
            status_.items[idx_].mark = Mark::Stopped;
            Finish("Stopped", queue_[idx_].recipe.name);
            return;
        } else {
            status_.items[idx_].mark = Mark::Failed;
            Finish("Failed", queue_[idx_].recipe.name + ": " + es.error);
            return;
        }
    }

    if (stopReq_) {
        Finish("Stopped", "");
        return;
    }
    if (idx_ >= queue_.size()) {
        Finish("Completed", "");
        return;
    }

    status_.index            = idx_;
    status_.items[idx_].mark = Mark::Running;
    std::string err;
    RecipeEngine::Env env = env_;   // fresh copy per recipe run
    if (!engine_.Start(queue_[idx_].recipe, std::move(env), &err)) {
        status_.items[idx_].mark = Mark::Failed;
        Finish("Failed", queue_[idx_].recipe.name + ": " + err);
        return;
    }
    waitingEngine_ = true;
}

void ProgramRunner::Finish(const char* result, const std::string& detail)
{
    active_        = false;
    status_.active = false;
    status_.result = detail.empty() ? result : std::string(result) + " - " + detail;

    std::size_t done = 0;
    for (const ItemStatus& it : status_.items)
        if (it.mark == Mark::Done) ++done;

    FD_INFO("program", "%s '%s' finished: %s (%zu/%zu recipe(s) done)",
            kind_.c_str(), label_.c_str(), status_.result.c_str(), done,
            queue_.size());
    // Program/slot runs get their own traceability event; a single-recipe run
    // is already covered by the recipe's own $run.
    if (kind_ != "recipe") {
        DutSession::Instance().RecordRun(kind_, label_, result, uid_,
                                         static_cast<int>(done));
    }
}

// ---- Run-request resolution ------------------------------------------------

RunQueue BuildRunQueue(const Program& prog, RunScope scope,
                       int slotIdx, int recipeIdx,
                       const std::vector<Recipe>& library, std::string* err)
{
    const auto fail = [&](std::string why) {
        if (err) *err = std::move(why);
        return RunQueue{};
    };

    RunQueue out;
    out.kind  = "slot";
    out.label = prog.name;

    // Resolve one uid into the queue; a miss is fatal for the whole request.
    bool ok = true;
    const auto add = [&](const std::string& slotName, const std::string& uid) {
        if (!ok) return;
        const Recipe* r = RecipeByUid(library, uid);
        if (!r) {
            ok = false;
            if (err) *err = "recipe uid '" + uid + "' not found (slot '" + slotName + "')";
            return;
        }
        out.items.push_back({slotName, *r});
    };

    if (slotIdx < 0 || slotIdx >= static_cast<int>(prog.slots.size()))
        return fail("no slot selected");
    const ProgramSlot& s = prog.slots[static_cast<std::size_t>(slotIdx)];
    if (scope == RunScope::Slot) {
        out.label = prog.name + " / " + s.name;
        for (const std::string& uid : s.recipes)
            add(s.name, uid);
    } else {
        if (recipeIdx < 0 || recipeIdx >= static_cast<int>(s.recipes.size()))
            return fail("no recipe selected");
        out.kind = "recipe";
        add(s.name, s.recipes[static_cast<std::size_t>(recipeIdx)]);
        if (ok)
            out.label = prog.name + " / " + s.name + " / " + out.items.front().recipe.name;
    }
    if (!ok) return RunQueue{};
    return out;
}

} // namespace firedog
