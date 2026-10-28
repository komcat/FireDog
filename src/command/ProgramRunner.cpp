// ProgramRunner.cpp — UI-thread only (Start/Tick/Stop from App), no locks.
#include "command/ProgramRunner.h"

#include "core/DutSession.h"
#include "core/Log.h"

#include <functional>
#include <string>
#include <utility>

namespace firedog {

bool ProgramRunner::Start(std::string kind, std::string label, std::string uid,
                          std::vector<Item> queue, RecipeEngine::Env env,
                          std::string* err, RecipeEngine::StartOpts opts)
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
    opts_    = std::move(opts);
    idx_     = 0;
    stopReq_ = false;
    waitingEngine_ = false;
    active_  = true;

    status_ = Status{};
    status_.active = true;
    status_.kind   = kind_;
    status_.label  = label_;
    status_.total  = queue_.size();
    status_.teach  = opts_.teach;
    status_.dryRun = true;
    status_.items.reserve(queue_.size());
    for (const Item& it : queue_) {
        ItemStatus st;
        st.slotName   = it.slotName;
        st.recipeName = it.recipe.name;
        st.path       = it.path;
        st.nodeUid    = it.nodeUid;
        st.pass       = it.pass;
        st.passes     = it.passes;
        st.dryRun     = it.dryRun;
        st.mark       = Mark::Pending;
        if (!it.dryRun) status_.dryRun = false;
        status_.items.push_back(std::move(st));
    }

    FD_INFO("program", "%s '%s' started (%zu recipe run(s))%s%s", kind_.c_str(),
            label_.c_str(), queue_.size(), opts_.teach ? " [TEACH]" : "",
            status_.dryRun ? " [DRY RUN]" : "");
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
    RecipeEngine::StartOpts opts = opts_;
    opts.dryRun = opts.dryRun || queue_[idx_].dryRun;
    if (!engine_.Start(queue_[idx_].recipe, std::move(env), &err, opts)) {
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
    // is already covered by the recipe's own $run. A teach run is not a run of
    // the part and leaves no trace on it; nor is an all-dry one.
    if (kind_ != "recipe" && !opts_.teach && !status_.dryRun) {
        DutSession::Instance().RecordRun(kind_, label_, result, uid_,
                                         static_cast<int>(done));
    }
}

// ---- Run-request resolution ------------------------------------------------

namespace {
// The expanded-queue ceiling. A run of 20,000 recipes is already beyond any
// shift; past this the number is a typo, and the honest place to say so is
// before the machine moves.
constexpr std::size_t kMaxQueueItems = 20000;
// Folder nesting depth. Deep enough for any real deck, shallow enough that a
// file hand-edited into a cycle cannot recurse the UI thread off its stack.
constexpr int kMaxDepth = 8;

// True when a folder ABOVE `uid` (not the node itself) is a dry-run folder.
bool UnderDryFolder(const std::vector<ProgramNode>& items, const std::string& uid,
                    bool dry = false, int depth = 0)
{
    if (depth > kMaxDepth) return false;
    for (const ProgramNode& n : items) {
        if (n.uid == uid) return dry;
        if (n.IsFolder() && NodeContains(n, uid))
            return UnderDryFolder(n.children, uid, dry || n.dryRun, depth + 1);
    }
    return false;
}
} // namespace

RunQueue BuildRunQueue(const Program& prog, RunScope scope,
                       int slotIdx, const std::string& nodeUid,
                       const std::vector<Recipe>& library, std::string* err)
{
    const auto fail = [&](std::string why) {
        if (err) *err = std::move(why);
        return RunQueue{};
    };

    RunQueue out;
    out.kind  = "slot";
    out.label = prog.name;

    if (slotIdx < 0 || slotIdx >= static_cast<int>(prog.slots.size()))
        return fail("no slot selected");
    const ProgramSlot& s = prog.slots[static_cast<std::size_t>(slotIdx)];

    bool ok = true;
    std::string why;

    // Flatten one node list into queue items. `path` is the folder breadcrumb
    // the item ran under; `pass`/`passes` describe the INNERMOST enclosing
    // loop, which is the one an operator watching the deck is counting.
    // `dry` is inherited downward: a dry folder makes its whole subtree dry.
    std::function<void(const std::vector<ProgramNode>&, const std::string&, int, int, int, bool)> emit;
    emit = [&](const std::vector<ProgramNode>& items, const std::string& path,
               int pass, int passes, int depth, bool dry) {
        if (!ok) return;
        if (depth > kMaxDepth) {
            ok = false;
            why = "folder nesting deeper than " + std::to_string(kMaxDepth) + " levels";
            return;
        }
        for (const ProgramNode& n : items) {
            if (!ok) return;
            if (n.IsFolder()) {
                const int loops = n.loop < 1 ? 1 : n.loop;
                const std::string sub = path.empty() ? n.name : path + " / " + n.name;
                for (int i = 1; i <= loops && ok; ++i)
                    emit(n.children, sub, i, loops, depth + 1, dry || n.dryRun);
                continue;
            }
            const Recipe* r = RecipeByUid(library, n.recipeUid);
            if (!r) {
                ok  = false;
                why = "recipe uid '" + n.recipeUid + "' not found (slot '" + s.name + "'" +
                      (path.empty() ? std::string() : ", folder '" + path + "'") + ")";
                return;
            }
            if (out.items.size() >= kMaxQueueItems) {
                ok  = false;
                why = "expanded run is over " + std::to_string(kMaxQueueItems) +
                      " recipes - check the folder loop counts";
                return;
            }
            ProgramRunner::Item it;
            it.slotName = s.name;
            it.path     = path;
            it.nodeUid  = n.uid;
            it.pass     = pass;
            it.passes   = passes;
            it.dryRun   = dry;
            it.recipe   = *r;
            out.items.push_back(std::move(it));
        }
    };

    if (scope == RunScope::Slot) {
        out.label = prog.name + " / " + s.name;
        emit(s.items, std::string(), 1, 1, 0, false);
    } else {
        const ProgramNode* n = FindNode(s.items, nodeUid);
        if (!n) return fail("nothing selected in the deck");
        const bool dry = UnderDryFolder(s.items, nodeUid);
        if (n->IsFolder()) {
            // A folder runs as a folder: its loop count belongs to the node,
            // not to the list around it, so selecting it must repeat it.
            out.kind  = "folder";
            out.label = prog.name + " / " + s.name + " / " + n->name;
            const int loops = n->loop < 1 ? 1 : n->loop;
            for (int i = 1; i <= loops && ok; ++i)
                emit(n->children, n->name, i, loops, 1, dry || n->dryRun);
        } else {
            out.kind = "recipe";
            const Recipe* r = RecipeByUid(library, n->recipeUid);
            if (!r) return fail("recipe uid '" + n->recipeUid + "' not found");
            ProgramRunner::Item it;
            it.slotName = s.name;
            it.nodeUid  = n->uid;
            it.dryRun   = dry;
            it.recipe   = *r;
            out.items.push_back(std::move(it));
            out.label = prog.name + " / " + s.name + " / " + r->name;
        }
    }

    if (!ok) return fail(std::move(why));
    if (out.items.empty()) return fail("nothing to run (no recipes in the selection)");
    return out;
}

} // namespace firedog
