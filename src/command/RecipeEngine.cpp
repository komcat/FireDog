// RecipeEngine.cpp — the worker thread + status plumbing; step execution
// itself lives in StepRunner (shared with the Lua tier's fd.run_recipe).
#include "command/RecipeEngine.h"

#include "command/CommandRegistry.h"
#include "command/StepRunner.h"
#include "core/DutSession.h"
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
        curPath_.clear();
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

void RecipeEngine::AnswerPrompt(bool ok, std::string text)
{
    {
        const std::scoped_lock lock(mu_);
        if (prompt_.empty()) return;   // nothing pending
        prompt_.clear();
        promptText_     = std::move(text);
        promptAnswer_   = ok;
        promptAnswered_ = true;
    }
    cv_.notify_all();
}

void RecipeEngine::AnswerAdjust(bool ok)
{
    {
        const std::scoped_lock lock(mu_);
        if (!adjustPending_) return;   // nothing pending
        adjustPending_  = false;
        adjustAnswer_   = ok;
        adjustAnswered_ = true;
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
    s.error              = error_;
    s.pendingPrompt      = prompt_;
    s.pendingPromptInput = promptWantsInput_;
    s.pendingAdjust      = adjustPending_;
    s.adjustReq          = adjustReq_;
    if (runCtx_) s.ctx = runCtx_->CtxSnapshot();   // its own mutex; no deadlock
    return s;
}

bool RecipeEngine::SafetyAborted() const
{
    return wf_.SafetyAlertActive() && wf_.SafetyAlert().critical;
}

bool RecipeEngine::AwaitPromptInternal(const std::string& message, bool wantsInput,
                                       std::string* textOut)
{
    std::unique_lock lock(mu_);
    prompt_           = message;
    promptWantsInput_ = wantsInput;
    promptText_.clear();
    promptAnswered_ = false;
    promptAnswer_   = false;
    // Wait in slices so a stop request always gets through.
    while (!promptAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    prompt_.clear();
    promptWantsInput_ = false;
    const bool ok = promptAnswered_ && promptAnswer_ && !stopReq_.load();
    if (ok && textOut) *textOut = promptText_;
    return ok;
}

bool RecipeEngine::AwaitAdjustInternal(const ExecutionContext::AdjustRequest& req)
{
    std::unique_lock lock(mu_);
    adjustReq_      = req;
    adjustPending_  = true;
    adjustAnswered_ = false;
    adjustAnswer_   = false;
    // Wait in slices so a stop request always gets through.
    while (!adjustAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    adjustPending_ = false;
    return adjustAnswered_ && adjustAnswer_ && !stopReq_.load();
}

void RecipeEngine::ThreadMain(Recipe recipe, Env env)
{
    ExecutionContext ctx(
        wf_, hw_, env.positions, env.maps,
        ExecutionContext::Hooks{
            [this] { return stopReq_.load() || SafetyAborted(); },
            [this](const std::string& m) { return AwaitPromptInternal(m, false, nullptr); },
            [this](const std::string& m, std::string* out) {
                return AwaitPromptInternal(m, true, out);
            },
            [this](const ExecutionContext::AdjustRequest& r) {
                return AwaitAdjustInternal(r);
            },
        });
    {
        const std::scoped_lock lock(mu_);
        runCtx_ = &ctx;
    }

    StepRunner::Outcome out = StepRunner::Outcome::Fail;
    std::string runErr;
    try {
        StepRunner runner(
            reg_, ctx,
            StepRunner::Callbacks{
                [this](const std::string& label, const std::vector<std::size_t>& path) {
                    const std::scoped_lock lock(mu_);
                    currentStep_ = label;
                    curPath_     = path;
                },
                [this] {
                    const std::scoped_lock lock(mu_);
                    ++leafDone_;
                },
            });
        out    = runner.Run(recipe.steps);
        runErr = runner.Error();
    } catch (const std::exception& e) {
        // Should have been contained at the step boundary — last line of defense.
        FD_ERROR("recipe", "unexpected engine exception: %s", e.what());
        runErr = e.what();
    } catch (...) {
        FD_ERROR("recipe", "unexpected non-standard engine exception");
        runErr = "unknown error";
    }

    // Terminal state — every path lands here with the cause logged.
    {
        const std::scoped_lock lock(mu_);
        runCtx_ = nullptr;   // ctx dies with this frame; status stops reading it
        error_  = runErr;
        if (stopReq_.load() || out == StepRunner::Outcome::Aborted) {
            state_ = State::Stopped;
            if (error_.empty())
                error_ = SafetyAborted() ? "safety alert" : "stopped by operator";
        } else if (out == StepRunner::Outcome::Ok ||
                   out == StepRunner::Outcome::StopOk) {
            state_ = State::Completed;
            error_.clear();
        } else {
            state_ = State::Failed;
            if (error_.empty()) error_ = "step failed";
        }
        currentStep_ = "";
        curPath_.clear();
        FD_INFO("recipe", "run '%s' finished: %s%s%s", recipeName_.c_str(),
                StateName(state_), error_.empty() ? "" : " - ", error_.c_str());
        // Traceability: attribute this run to the open DUT (soft no-op if none).
        DutSession::Instance().RecordRun("recipe", recipeName_, StateName(state_));
    }
    running_.store(false);
}

} // namespace firedog
