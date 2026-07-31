// StepRunner.cpp — ported verbatim from RecipeEngine's ExecSteps/ExecLeaf
// (Phase C2 extraction); behavior must stay identical for recipes.
#include "command/StepRunner.h"

#include "command/CommandRegistry.h"
#include "core/Log.h"

#include <exception>

namespace firedog {

StepRunner::StepRunner(const CommandRegistry& reg, ExecutionContext& ctx, Callbacks cb)
    : reg_(reg), ctx_(ctx), cb_(std::move(cb))
{
}

StepRunner::Outcome StepRunner::Run(const std::vector<RecipeStep>& steps)
{
    error_.clear();
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

        if (s.type == "group" || s.type == "sub_recipe") {
            const Outcome o = ExecSteps(s.children, where, path, depth + 1);
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

} // namespace firedog
