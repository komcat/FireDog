// RecipeEngine.cpp
#include "command/RecipeEngine.h"

#include "command/CommandRegistry.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <chrono>
#include <exception>

namespace firedog {

const char* RecipeEngine::StateName(State s)
{
    switch (s) {
    case State::Idle:      return "Idle";
    case State::Running:   return "Running";
    case State::Completed: return "Completed";
    case State::Failed:    return "Failed";
    case State::Stopped:   return "Stopped";
    }
    return "?";
}

RecipeEngine::RecipeEngine(Workflow& wf, HardwareManager& hw,
                           const CommandRegistry& reg)
    : wf_(wf), hw_(hw), reg_(reg)
{
}

RecipeEngine::~RecipeEngine()
{
    RequestStop();
    if (worker_.joinable()) worker_.join();
}

bool RecipeEngine::Start(const Recipe& recipe, Env env, std::string* err)
{
    if (running_.load()) {
        if (err) *err = "a recipe is already running";
        return false;
    }
    if (wf_.PathActive()) {
        if (err) *err = "the Motion page path executor is busy - cancel it first";
        return false;
    }
    if (worker_.joinable()) worker_.join();   // reap the previous finished run

    // Run-start validation: structure + params + references + everything
    // referenced actually Connected. Recipes never connect hardware.
    const std::vector<RecipeIssue> issues = ValidateRecipe(
        recipe, reg_, hw_, env.positions, env.maps, env.library, true);
    if (!issues.empty()) {
        std::string msg = issues.front().where + ": " + issues.front().message;
        if (issues.size() > 1)
            msg += " (+" + std::to_string(issues.size() - 1) + " more issue(s))";
        for (const RecipeIssue& i : issues)
            FD_WARN("recipe", "validation: %s: %s", i.where.c_str(), i.message.c_str());
        if (err) *err = msg;
        return false;
    }

    Recipe run = recipe;
    std::string inlineErr;
    if (!InlineSubRecipes(run, env.library, &inlineErr)) {
        FD_WARN("recipe", "start rejected: %s", inlineErr.c_str());
        if (err) *err = inlineErr;
        return false;
    }

    {
        const std::scoped_lock lock(mu_);
        state_       = State::Running;
        recipeName_  = run.name;
        currentStep_ = "";
        error_       = "";
        prompt_      = "";
        leafDone_    = 0;
        leafTotal_   = CountLeafSteps(run.steps);
    }
    stopReq_.store(false);
    running_.store(true);
    FD_INFO("recipe", "run started: '%s'", run.name.c_str());
    worker_ = std::thread(&RecipeEngine::ThreadMain, this, std::move(run), std::move(env));
    return true;
}

void RecipeEngine::RequestStop()
{
    stopReq_.store(true);
    cv_.notify_all();   // release a blocked prompt
}

void RecipeEngine::AnswerPrompt(bool ok)
{
    {
        const std::scoped_lock lock(mu_);
        if (prompt_.empty()) return;   // nothing pending
        prompt_.clear();
        promptAnswer_   = ok;
        promptAnswered_ = true;
    }
    cv_.notify_all();
}

RecipeEngine::Status RecipeEngine::GetStatus() const
{
    const std::scoped_lock lock(mu_);
    Status s;
    s.state         = state_;
    s.recipeName    = recipeName_;
    s.currentStep   = currentStep_;
    s.stepPath      = curPath_;
    s.leafDone      = leafDone_;
    s.leafTotal     = leafTotal_;
    s.error         = error_;
    s.pendingPrompt = prompt_;
    if (runCtx_) s.ctx = runCtx_->CtxSnapshot();   // its own mutex; no deadlock
    return s;
}

void RecipeEngine::SetCurrentStep(const std::string& s,
                                  const std::vector<std::size_t>& path)
{
    const std::scoped_lock lock(mu_);
    currentStep_ = s;
    curPath_     = path;
}

bool RecipeEngine::SafetyAborted() const
{
    return wf_.SafetyAlertActive() && wf_.SafetyAlert().critical;
}

bool RecipeEngine::AwaitPromptInternal(const std::string& message)
{
    std::unique_lock lock(mu_);
    prompt_         = message;
    promptAnswered_ = false;
    promptAnswer_   = false;
    // Wait in slices so a stop request always gets through.
    while (!promptAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    prompt_.clear();
    return promptAnswered_ && promptAnswer_ && !stopReq_.load();
}

void RecipeEngine::ThreadMain(Recipe recipe, Env env)
{
    ExecutionContext ctx(
        wf_, hw_, env.positions, env.maps,
        ExecutionContext::Hooks{
            [this] { return stopReq_.load() || SafetyAborted(); },
            [this](const std::string& m) { return AwaitPromptInternal(m); },
        });
    {
        const std::scoped_lock lock(mu_);
        runCtx_ = &ctx;
    }

    Outcome out = Outcome::Fail;
    try {
        out = ExecSteps(recipe.steps, ctx, "", {}, 1);
    } catch (const std::exception& e) {
        // Should have been contained at the step boundary — last line of defense.
        FD_ERROR("recipe", "unexpected engine exception: %s", e.what());
        const std::scoped_lock lock(mu_);
        if (error_.empty()) error_ = e.what();
    } catch (...) {
        FD_ERROR("recipe", "unexpected non-standard engine exception");
    }

    // Terminal state — every path lands here with the cause logged.
    {
        const std::scoped_lock lock(mu_);
        runCtx_ = nullptr;   // ctx dies with this frame; status stops reading it
        if (stopReq_.load() || out == Outcome::Aborted) {
            state_ = State::Stopped;
            if (error_.empty())
                error_ = SafetyAborted() ? "safety alert" : "stopped by operator";
        } else if (out == Outcome::Ok || out == Outcome::StopOk) {
            state_ = State::Completed;
        } else {
            state_ = State::Failed;
            if (error_.empty()) error_ = "step failed";
        }
        currentStep_ = "";
        curPath_.clear();
        FD_INFO("recipe", "run '%s' finished: %s%s%s", recipeName_.c_str(),
                StateName(state_), error_.empty() ? "" : " - ", error_.c_str());
    }
    running_.store(false);
}

RecipeEngine::Outcome RecipeEngine::ExecSteps(const std::vector<RecipeStep>& steps,
                                              ExecutionContext& ctx,
                                              const std::string& pathPrefix,
                                              const std::vector<std::size_t>& basePath,
                                              int depth)
{
    if (depth > kMaxRecipeDepth) {   // validated at load; belt and braces
        const std::scoped_lock lock(mu_);
        error_ = "nesting exceeds depth " + std::to_string(kMaxRecipeDepth);
        return Outcome::Fail;
    }

    for (std::size_t i = 0; i < steps.size(); ++i) {
        const RecipeStep& s = steps[i];
        if (!s.enabled) continue;
        if (stopReq_.load() || SafetyAborted()) return Outcome::Aborted;

        const std::string where =
            (pathPrefix.empty() ? "" : pathPrefix + ".") + std::to_string(i + 1) +
            " (" + (s.nickname.empty() ? s.type : s.nickname) + ")";
        std::vector<std::size_t> path = basePath;
        path.push_back(i);

        if (s.type == "group" || s.type == "sub_recipe") {
            const Outcome o = ExecSteps(s.children, ctx, where, path, depth + 1);
            if (o != Outcome::Ok) return o;
        } else if (s.type == "loop") {
            if (s.params.contains("count")) {
                const int count = s.params["count"].get<int>();
                for (int k = 0; k < count; ++k) {
                    if (stopReq_.load() || SafetyAborted()) return Outcome::Aborted;
                    const Outcome o = ExecSteps(s.children, ctx, where, path, depth + 1);
                    if (o != Outcome::Ok) return o;
                }
            } else {   // while-form; bounded by max_iterations
                const int maxIter = s.params.value("max_iterations", kDefaultMaxIterations);
                int k = 0;
                for (;; ++k) {
                    if (stopReq_.load() || SafetyAborted()) return Outcome::Aborted;
                    if (k >= maxIter) {
                        FD_WARN("recipe", "%s: while-loop hit max_iterations (%d)",
                                where.c_str(), maxIter);
                        break;
                    }
                    std::string condErr;
                    const std::optional<bool> go =
                        EvalCondition(s.params["while"], ctx, &condErr);
                    if (!go.has_value()) {
                        const std::scoped_lock lock(mu_);
                        error_ = where + ": " + condErr;
                        return Outcome::Fail;
                    }
                    if (!*go) break;
                    const Outcome o = ExecSteps(s.children, ctx, where, path, depth + 1);
                    if (o != Outcome::Ok) return o;
                }
            }
        } else if (s.type == "if") {
            std::string condErr;
            const std::optional<bool> v = EvalCondition(s.params, ctx, &condErr);
            if (!v.has_value()) {
                const std::scoped_lock lock(mu_);
                error_ = where + ": " + condErr;
                return Outcome::Fail;
            }
            const Outcome o =
                ExecSteps(*v ? s.children : s.elseChildren, ctx, where, path, depth + 1);
            if (o != Outcome::Ok) return o;
        } else if (s.type == "stop_recipe") {
            const bool ok = s.params.value("ok", true);
            const std::string msg = s.params.value("message", std::string());
            FD_INFO("recipe", "%s: stop_recipe (%s)%s%s", where.c_str(),
                    ok ? "success" : "fail", msg.empty() ? "" : " - ", msg.c_str());
            if (!ok) {
                const std::scoped_lock lock(mu_);
                if (error_.empty()) error_ = msg.empty() ? "stopped by recipe" : msg;
            }
            return ok ? Outcome::StopOk : Outcome::StopFail;
        } else {
            const Outcome o = ExecLeaf(s, ctx, where, path);
            if (o != Outcome::Ok) return o;
        }
    }
    return Outcome::Ok;
}

RecipeEngine::Outcome RecipeEngine::ExecLeaf(const RecipeStep& s,
                                             ExecutionContext& ctx,
                                             const std::string& where,
                                             const std::vector<std::size_t>& path)
{
    const CommandSpec* spec = reg_.Find(s.type);
    if (!spec) {   // validated at Start; belt and braces
        const std::scoped_lock lock(mu_);
        error_ = where + ": unknown command";
        return Outcome::Fail;
    }
    nlohmann::json merged;
    const std::string pe = ValidateParams(*spec, s.params, &merged);
    if (!pe.empty()) {
        const std::scoped_lock lock(mu_);
        error_ = where + ": " + pe;
        return Outcome::Fail;
    }
    const ParamValues pv(std::move(merged));

    const int attempts = s.onError == RecipeStep::OnError::Retry ? s.retryCount + 1 : 1;
    std::string lastErr;
    for (int a = 0; a < attempts; ++a) {
        if (stopReq_.load() || SafetyAborted()) return Outcome::Aborted;
        if (attempts > 1) {
            SetCurrentStep(where + " attempt " + std::to_string(a + 1) + "/" +
                           std::to_string(attempts), path);
        } else {
            SetCurrentStep(where, path);
        }

        CommandResult r;
        try {
            r = spec->execute(ctx, pv);
        } catch (const std::exception& e) {
            r = CommandResult::Fail(std::string("exception: ") + e.what());
        } catch (...) {
            r = CommandResult::Fail("non-standard exception");
        }

        if (r.ok) {
            const std::scoped_lock lock(mu_);
            ++leafDone_;
            return Outcome::Ok;
        }
        lastErr = r.error;
        FD_WARN("recipe", "%s failed: %s%s", where.c_str(), r.error.c_str(),
                a + 1 < attempts ? " - retrying" : "");
    }

    if (stopReq_.load() || SafetyAborted()) return Outcome::Aborted;
    if (s.onError == RecipeStep::OnError::Continue) {
        FD_WARN("recipe", "%s: continuing past failure (on_error: continue)",
                where.c_str());
        const std::scoped_lock lock(mu_);
        ++leafDone_;
        return Outcome::Ok;
    }
    {
        const std::scoped_lock lock(mu_);
        error_ = where + ": " + lastErr;
    }
    return Outcome::Fail;
}

} // namespace firedog
