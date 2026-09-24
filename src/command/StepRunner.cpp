// StepRunner.cpp — ported verbatim from RecipeEngine's ExecSteps/ExecLeaf
// (Phase C2 extraction); behavior must stay identical for recipes.
#include "command/StepRunner.h"

#include "command/CommandRegistry.h"
#include "core/Log.h"

#include <algorithm>
#include <exception>
#include <thread>

namespace firedog {

namespace {
// Pre-order relation of a step path to the start-at path:
//  -1 = entirely before the start (skip the whole subtree)
//   0 = ancestor of the start (descend, keep skipping inside)
//  +1 = the start itself or anything after it (resume normal execution)
int RelToStart(const std::vector<std::size_t>& p, const std::vector<std::size_t>& s)
{
    for (std::size_t k = 0; k < p.size() && k < s.size(); ++k) {
        if (p[k] != s[k]) return p[k] < s[k] ? -1 : +1;
    }
    return p.size() < s.size() ? 0 : +1;
}

// Leaf count of ONE step (same rules as CountLeafSteps) — how many leaves a
// skipped subtree would have contributed, so progress starts at the right spot.
std::size_t LeafCountOf(const RecipeStep& s)
{
    if (s.type == "loop") {
        std::size_t body = CountLeafSteps(s.children);
        if (s.params.contains("count") && s.params["count"].is_number()) {
            const double c = s.params["count"].get<double>();
            if (c > 0) body *= static_cast<std::size_t>(c);
        }
        return body;
    }
    if (s.type == "if")
        return std::max(CountLeafSteps(s.children), CountLeafSteps(s.elseChildren));
    if (s.type == "group" || s.type == "sub_recipe" || s.type == "parallel")
        return CountLeafSteps(s.children);
    if (s.type == "stop_recipe") return 0;
    return 1;
}

// Joins every thread it was handed on the way out, so a lane thread can never
// outlive the block that started it — whatever path (exception included)
// leaves ExecParallel.
struct JoinAll {
    std::vector<std::thread> threads;
    ~JoinAll()
    {
        for (std::thread& t : threads)
            if (t.joinable()) t.join();
    }
};
} // namespace

StepRunner::StepRunner(const CommandRegistry& reg, ExecutionContext& ctx, Callbacks cb)
    : reg_(reg), ctx_(ctx), cb_(std::move(cb))
{
}

StepRunner::Outcome StepRunner::Run(const std::vector<RecipeStep>& steps,
                                    std::vector<std::size_t> startAt)
{
    error_.clear();
    startAt_  = std::move(startAt);
    skipping_ = !startAt_.empty();
    return ExecSteps(steps, "", {}, 1);
}

StepRunner::Outcome StepRunner::ExecSteps(const std::vector<RecipeStep>& steps,
                                          const std::string& pathPrefix,
                                          const std::vector<std::size_t>& basePath,
                                          int depth)
{
    if (depth > kMaxRecipeDepth) {   // validated at load; belt and braces
        error_ = "nesting exceeds depth " + std::to_string(kMaxRecipeDepth);
        return Outcome::Fail;
    }

    for (std::size_t i = 0; i < steps.size(); ++i) {
        const RecipeStep& s = steps[i];
        if (!s.enabled) continue;
        if (ctx_.ShouldStop()) return Outcome::Aborted;

        const std::string where =
            (pathPrefix.empty() ? "" : pathPrefix + ".") + std::to_string(i + 1) +
            " (" + (s.nickname.empty() ? s.type : s.nickname) + ")";
        std::vector<std::size_t> path = basePath;
        path.push_back(i);

        if (skipping_) {
            const int rel = RelToStart(path, startAt_);
            if (rel < 0) {   // before the start step — skip, but count leaves
                if (cb_.onLeafDone) {
                    const std::size_t n = LeafCountOf(s);
                    for (std::size_t k = 0; k < n; ++k) cb_.onLeafDone();
                }
                continue;
            }
            if (rel > 0) skipping_ = false;   // reached — run normally from here
            // rel == 0: ancestor of the start — descend with skipping still on
            // (a loop consumes the skip in its first iteration only).
        }

        if (s.type == "group" || s.type == "sub_recipe") {
            const Outcome o = ExecSteps(s.children, where, path, depth + 1);
            if (o != Outcome::Ok) return o;
        } else if (s.type == "parallel") {
            // Atomic: a run started INSIDE the block runs the whole block. Half
            // a parallel move is not a thing an operator asked for.
            skipping_ = false;
            const Outcome o = ExecParallel(s, where, path);
            if (o != Outcome::Ok) return o;
        } else if (s.type == "loop") {
            if (s.params.contains("count")) {
                const int count = s.params["count"].get<int>();
                for (int k = 0; k < count; ++k) {
                    if (ctx_.ShouldStop()) return Outcome::Aborted;
                    const Outcome o = ExecSteps(s.children, where, path, depth + 1);
                    if (o != Outcome::Ok) return o;
                }
            } else {   // while-form; bounded by max_iterations
                const int maxIter = s.params.value("max_iterations", kDefaultMaxIterations);
                int k = 0;
                for (;; ++k) {
                    if (ctx_.ShouldStop()) return Outcome::Aborted;
                    if (k >= maxIter) {
                        FD_WARN("recipe", "%s: while-loop hit max_iterations (%d)",
                                where.c_str(), maxIter);
                        break;
                    }
                    std::string condErr;
                    const std::optional<bool> go =
                        EvalCondition(s.params["while"], ctx_, &condErr);
                    if (!go.has_value()) {
                        error_ = where + ": " + condErr;
                        return Outcome::Fail;
                    }
                    if (!*go) break;
                    const Outcome o = ExecSteps(s.children, where, path, depth + 1);
                    if (o != Outcome::Ok) return o;
                }
            }
        } else if (s.type == "if") {
            std::string condErr;
            const std::optional<bool> v = EvalCondition(s.params, ctx_, &condErr);
            if (!v.has_value()) {
                error_ = where + ": " + condErr;
                return Outcome::Fail;
            }
            const Outcome o =
                ExecSteps(*v ? s.children : s.elseChildren, where, path, depth + 1);
            if (o != Outcome::Ok) return o;
        } else if (s.type == "stop_recipe") {
            const bool ok = s.params.value("ok", true);
            const std::string msg = s.params.value("message", std::string());
            FD_INFO("recipe", "%s: stop_recipe (%s)%s%s", where.c_str(),
                    ok ? "success" : "fail", msg.empty() ? "" : " - ", msg.c_str());
            if (!ok && error_.empty())
                error_ = msg.empty() ? "stopped by recipe" : msg;
            return ok ? Outcome::StopOk : Outcome::StopFail;
        } else {
            const Outcome o = ExecLeaf(s, where, path);
            if (o != Outcome::Ok) return o;
        }
    }
    return Outcome::Ok;
}

StepRunner::Outcome StepRunner::ExecLeaf(const RecipeStep& s, const std::string& where,
                                         const std::vector<std::size_t>& path)
{
    // Step-mode gate: in step mode the engine blocks here until the operator
    // grants the leaf (Next) or leaves step mode; false = stop/safety abort.
    if (cb_.onBeforeLeaf && !cb_.onBeforeLeaf(where, path))
        return Outcome::Aborted;

    const CommandSpec* spec = reg_.Find(s.type);
    if (!spec) {   // validated at Start; belt and braces
        error_ = where + ": unknown command";
        return Outcome::Fail;
    }
    nlohmann::json merged;
    const std::string pe = ValidateParams(*spec, s.params, &merged);
    if (!pe.empty()) {
        error_ = where + ": " + pe;
        return Outcome::Fail;
    }
    const ParamValues pv(std::move(merged));

    const int attempts = s.onError == RecipeStep::OnError::Retry ? s.retryCount + 1 : 1;
    std::string lastErr;
    for (int a = 0; a < attempts; ++a) {
        if (ctx_.ShouldStop()) return Outcome::Aborted;
        if (cb_.onStep) {
            if (attempts > 1) {
                cb_.onStep(where + " attempt " + std::to_string(a + 1) + "/" +
                               std::to_string(attempts),
                           path);
            } else {
                cb_.onStep(where, path);
            }
        }

        CommandResult r;
        try {
            r = ExecuteWithDelays(*spec, ctx_, pv);
        } catch (const std::exception& e) {
            r = CommandResult::Fail(std::string("exception: ") + e.what());
        } catch (...) {
            r = CommandResult::Fail("non-standard exception");
        }

        if (r.ok) {
            if (cb_.onLeafDone) cb_.onLeafDone();
            return Outcome::Ok;
        }
        lastErr = r.error;
        FD_WARN("recipe", "%s failed: %s%s", where.c_str(), r.error.c_str(),
                a + 1 < attempts ? " - retrying" : "");
    }

    if (ctx_.ShouldStop()) return Outcome::Aborted;
    if (s.onError == RecipeStep::OnError::Continue) {
        FD_WARN("recipe", "%s: continuing past failure (on_error: continue)",
                where.c_str());
        if (cb_.onLeafDone) cb_.onLeafDone();
        return Outcome::Ok;
    }
    error_ = where + ": " + lastErr;
    return Outcome::Fail;
}

StepRunner::Outcome StepRunner::ExecParallel(const RecipeStep& s, const std::string& where,
                                             const std::vector<std::size_t>& path)
{
    // The children are dealt into LANES by device, in the order written: a
    // lane is the ordered list of every child naming one device, lanes run at
    // once, and the block ends when the last lane is done. So "1 left, 2 right,
    // 3 left, 4 right" is two lanes of two — 3 waits for 1 and never for 2.
    // There is no barrier inside a block; a barrier is the end of one block
    // and the start of the next.
    //
    // Everything that can be refused is refused HERE, before any thread
    // starts: once two stages are moving there is no clean place left to say no.
    struct Item {
        const RecipeStep*  step = nullptr;
        const CommandSpec* spec = nullptr;
        ParamValues        pv;
        std::string        where;
    };
    struct Lane {
        std::string       device;
        std::vector<Item> items;
        std::string       error;   // first hard failure, with the item's path
    };
    std::vector<Lane> lanes;
    std::size_t leaves = 0;
    for (std::size_t i = 0; i < s.children.size(); ++i) {
        const RecipeStep& c = s.children[i];
        if (!c.enabled) continue;
        Item it;
        it.step  = &c;
        it.where = where + "." + std::to_string(i + 1) + " (" +
                   (c.nickname.empty() ? c.type : c.nickname) + ")";
        it.spec = reg_.Find(c.type);
        if (!it.spec) {   // validated at Start; belt and braces
            error_ = it.where + ": unknown command";
            return Outcome::Fail;
        }
        if (it.spec->parallelBy.empty()) {
            error_ = it.where + ": " + c.type + " cannot run in a parallel block";
            return Outcome::Fail;
        }
        nlohmann::json merged;
        const std::string pe = ValidateParams(*it.spec, c.params, &merged);
        if (!pe.empty()) {
            error_ = it.where + ": " + pe;
            return Outcome::Fail;
        }
        if (merged.value("teach", false)) {
            error_ = it.where + ": teach cannot run in a parallel block";
            return Outcome::Fail;
        }
        const std::string device = merged.value(it.spec->parallelBy, std::string());
        it.pv = ParamValues(std::move(merged));
        Lane* lane = nullptr;
        for (Lane& l : lanes)
            if (l.device == device) { lane = &l; break; }
        if (!lane) {
            lanes.push_back(Lane{device, {}, {}});
            lane = &lanes.back();
        }
        lane->items.push_back(std::move(it));
        ++leaves;
    }
    if (lanes.empty()) return Outcome::Ok;   // every child off: an empty group

    // Step mode gates the BLOCK, not a lane — the lanes cannot be granted one
    // at a time and still be parallel.
    if (cb_.onBeforeLeaf && !cb_.onBeforeLeaf(where, path)) return Outcome::Aborted;

    // One lane, run to the end on its own thread. Each item is the very same
    // ExecuteWithDelays a sequential leaf gets — the two-speed move, the settle
    // rule and the stop handling are untouched, which is the point of forking
    // rather than re-implementing an issue/wait split for every mover. A
    // child's own on_error applies within its lane exactly as it would in a
    // group: retry re-runs that item, continue steps past it, abort ends the
    // lane there (the items after it do not run; the other lanes finish).
    const auto runLane = [this](Lane& lane) {
        for (const Item& it : lane.items) {
            const int attempts =
                it.step->onError == RecipeStep::OnError::Retry ? it.step->retryCount + 1 : 1;
            std::string lastErr;
            bool ok = false;
            for (int a = 0; a < attempts && !ok; ++a) {
                if (ctx_.ShouldStop()) { lane.error = it.where + ": stopped"; return; }
                CommandResult r;
                try {
                    r = ExecuteWithDelays(*it.spec, ctx_, it.pv);
                } catch (const std::exception& e) {
                    r = CommandResult::Fail(std::string("exception: ") + e.what());
                } catch (...) {
                    r = CommandResult::Fail("non-standard exception");
                }
                ok = r.ok;
                if (!ok) {
                    lastErr = r.error;
                    FD_WARN("recipe", "%s failed: %s%s", it.where.c_str(), r.error.c_str(),
                            a + 1 < attempts ? " - retrying" : "");
                }
            }
            if (ok) continue;
            if (it.step->onError == RecipeStep::OnError::Continue) {
                FD_WARN("recipe", "%s: continuing past failure (on_error: continue)",
                        it.where.c_str());
                continue;
            }
            lane.error = it.where + ": " + lastErr;
            return;
        }
    };

    const int attempts = s.onError == RecipeStep::OnError::Retry ? s.retryCount + 1 : 1;
    std::string lastErr;
    for (int a = 0; a < attempts; ++a) {
        if (ctx_.ShouldStop()) return Outcome::Aborted;
        if (cb_.onStep) {
            if (attempts > 1) {
                cb_.onStep(where + " attempt " + std::to_string(a + 1) + "/" +
                               std::to_string(attempts),
                           path);
            } else {
                cb_.onStep(where, path);
            }
        }
        for (Lane& l : lanes) {
            l.error.clear();
            FD_INFO("recipe", "%s: parallel lane '%s' (%zu step%s)", where.c_str(),
                    l.device.c_str(), l.items.size(), l.items.size() == 1 ? "" : "s");
        }

        // Fork. A lane that cannot get a thread fails like any other lane; the
        // ones already running finish. Every lane is joined at the closing brace.
        {
            JoinAll join;
            join.threads.reserve(lanes.size());
            for (Lane& l : lanes) {
                try {
                    join.threads.emplace_back([&runLane, &l] { runLane(l); });
                } catch (const std::exception& e) {
                    l.error = where + ": lane '" + l.device + "' could not start: " + e.what();
                }
            }
        }

        // A lane that failed has already stopped ITS device (that is what every
        // mover does on a timeout or a stop) and ran nothing after the failing
        // item; the other lanes were let finish, because a sibling's failure
        // says nothing about whether this lane's path is safe to abandon halfway.
        std::string errs;
        for (const Lane& l : lanes) {
            if (l.error.empty()) continue;
            if (!errs.empty()) errs += "; ";
            errs += l.error;
        }
        if (errs.empty()) {
            if (cb_.onLeafDone)
                for (std::size_t k = 0; k < leaves; ++k) cb_.onLeafDone();
            return Outcome::Ok;
        }
        lastErr = errs;
        FD_WARN("recipe", "%s failed: %s%s", where.c_str(), errs.c_str(),
                a + 1 < attempts ? " - retrying the block" : "");
    }

    if (ctx_.ShouldStop()) return Outcome::Aborted;
    if (s.onError == RecipeStep::OnError::Continue) {
        FD_WARN("recipe", "%s: continuing past failure (on_error: continue)",
                where.c_str());
        if (cb_.onLeafDone)
            for (std::size_t k = 0; k < leaves; ++k) cb_.onLeafDone();
        return Outcome::Ok;
    }
    error_ = lastErr;   // each lane's message already carries the failing item's path
    return Outcome::Fail;
}

} // namespace firedog
