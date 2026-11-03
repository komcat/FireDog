// RecipeEngine.cpp — the worker thread + status plumbing; step execution
// itself lives in StepRunner (shared with the Lua tier's fd.run_recipe).
#include "command/RecipeEngine.h"

#include "command/CommandRegistry.h"
#include "command/DryRun.h"
#include "command/StepRunner.h"
#include "command/TeachMode.h"
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

bool RecipeEngine::Start(const Recipe& recipe, Env env, std::string* err,
                         StartOpts opts)
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

    // A teach run executes a REWRITTEN copy (command/TeachMode.h): only the
    // saved-position moves stay enabled, each with teach = true. Applied
    // before validation - so a disconnected instrument behind a skipped step
    // does not refuse the run, while the movers' devices, positions and maps
    // are still checked - and again after inlining, for the sub-recipe bodies.
    Recipe run = recipe;
    if (opts.teach) {
        const std::size_t movers = MakeTeachRecipe(run, reg_);
        FD_INFO("recipe", "teach run: %zu move(s) to teach in '%s'", movers,
                run.name.c_str());
    }
    // A dry run is the same kind of rewrite (command/DryRun.h): motion, IO and
    // pneumatics only, so a meter left off behind a skipped step is no reason
    // to refuse a motion soak.
    if (opts.dryRun) {
        const std::size_t kept = MakeDryRunRecipe(run, reg_);
        FD_INFO("recipe", "dry run: %zu motion/IO step(s) kept in '%s'", kept,
                run.name.c_str());
    }

    // Run-start validation: structure + params + references + everything
    // referenced actually Connected. Recipes never connect hardware.
    const std::vector<RecipeIssue> issues = ValidateRecipe(
        run, reg_, hw_, env.positions, env.maps, env.aligns, env.visionModels,
        env.grips, env.trays, env.library, true);
    if (!issues.empty()) {
        std::string msg = issues.front().where + ": " + issues.front().message;
        if (issues.size() > 1)
            msg += " (+" + std::to_string(issues.size() - 1) + " more issue(s))";
        for (const RecipeIssue& i : issues)
            FD_WARN("recipe", "validation: %s: %s", i.where.c_str(), i.message.c_str());
        if (err) *err = msg;
        return false;
    }

    std::string inlineErr;
    if (!InlineSubRecipes(run, env.library, &inlineErr)) {
        FD_WARN("recipe", "start rejected: %s", inlineErr.c_str());
        if (err) *err = inlineErr;
        return false;
    }
    if (opts.teach) MakeTeachRecipe(run, reg_);   // the inlined bodies, same rule
    if (opts.dryRun) MakeDryRunRecipe(run, reg_);

    // The on-fail recipe is resolved, validated (devices Connected) and
    // inlined NOW, not when it is needed: a cleanup that cannot run must
    // refuse the run before the machine moves, not after the pick has failed
    // with the gripper inside the part. A teach run has none - the operator is
    // standing at the machine and every non-mover is off anyway. A dry run
    // gets the same rewrite as the recipe (its moves are what a home is).
    Recipe onFail;   // uid "" = none
    if (!run.onFailUid.empty() && !opts.teach) {
        const Recipe* src = nullptr;
        for (const Recipe& r : env.library)
            if (r.uid == run.onFailUid) { src = &r; break; }
        if (!src) {
            if (err) *err = "on-fail recipe uid '" + run.onFailUid + "' not found";
            FD_WARN("recipe", "start rejected: on-fail recipe uid '%s' not found",
                    run.onFailUid.c_str());
            return false;
        }
        onFail = *src;
        onFail.onFailUid.clear();   // never chained, so never checked
        if (opts.dryRun) MakeDryRunRecipe(onFail, reg_);
        const std::vector<RecipeIssue> fIssues = ValidateRecipe(
            onFail, reg_, hw_, env.positions, env.maps, env.aligns, env.visionModels,
            env.grips, env.trays, env.library, true);
        std::string fErr;
        if (!fIssues.empty())
            fErr = fIssues.front().where + ": " + fIssues.front().message;
        else if (InlineSubRecipes(onFail, env.library, &fErr) && opts.dryRun)
            MakeDryRunRecipe(onFail, reg_);   // the inlined bodies, same rule
        if (!fErr.empty()) {
            const std::string msg = "on-fail recipe '" + onFail.name + "': " + fErr;
            FD_WARN("recipe", "start rejected: %s", msg.c_str());
            if (err) *err = msg;
            return false;
        }
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
        stepGranted_ = false;
        stepPaused_  = false;
        nextStep_.clear();
        nextPath_.clear();
        lastCtx_.clear();   // it describes the LAST finished run, never a stale one
        // Skip-until path: strip the editor's else-branch markers — the
        // runner walks plain indices (same quirk as run highlighting).
        startPath_.clear();
        for (const std::size_t c : opts.startPath)
            startPath_.push_back(c >= kElseBranchBase ? c - kElseBranchBase : c);
    }
    stepMode_.store(opts.stepMode);
    teach_.store(opts.teach);
    dryRun_.store(opts.dryRun);
    stopReq_.store(false);
    running_.store(true);
    FD_INFO("recipe", "run started: '%s'%s%s", run.name.c_str(),
            opts.teach ? " [TEACH]" : "", opts.dryRun ? " [DRY RUN]" : "");
    worker_ = std::thread(&RecipeEngine::ThreadMain, this, std::move(run), std::move(onFail),
                          std::move(env));
    return true;
}

void RecipeEngine::RequestStop()
{
    stopReq_.store(true);
    cv_.notify_all();   // release a blocked prompt / step pause
}

void RecipeEngine::StepAdvance()
{
    {
        const std::scoped_lock lock(mu_);
        if (!stepPaused_) return;   // nothing waiting
        stepGranted_ = true;
    }
    cv_.notify_all();
}

void RecipeEngine::SetStepMode(bool on)
{
    stepMode_.store(on);
    cv_.notify_all();   // off releases a pending pause (free-run resumes)
}

void RecipeEngine::AnswerChoice(int index)
{
    {
        const std::scoped_lock lock(mu_);
        if (prompt_.empty() || promptButtons_.empty()) return;   // no choice pending
        if (index < 0 || index >= static_cast<int>(promptButtons_.size())) return;
        prompt_.clear();
        promptChoice_   = index;
        promptAnswer_   = true;
        promptAnswered_ = true;
    }
    cv_.notify_all();
}

void RecipeEngine::AnswerPrompt(bool ok, std::string text)
{
    {
        const std::scoped_lock lock(mu_);
        if (prompt_.empty()) return;   // nothing pending
        if (!promptButtons_.empty()) return;   // a choice: only AnswerChoice answers it
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

void RecipeEngine::AnswerPick(bool ok, ExecutionContext::PickResult result)
{
    {
        const std::scoped_lock lock(mu_);
        if (!pickPending_) return;   // nothing pending
        pickPending_  = false;
        pickResult_   = result;
        pickAnswer_   = ok;
        pickAnswered_ = true;
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
    s.pendingPromptLevel = promptLevel_;
    s.pendingPromptButtons = promptButtons_;
    s.pendingPromptImages  = promptImages_;
    if (!prompt_.empty() && promptAutoOkAt_ != std::chrono::steady_clock::time_point{}) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            promptAutoOkAt_ - std::chrono::steady_clock::now()).count();
        s.pendingPromptAutoOkMs = static_cast<int>(left > 1 ? left : 1);
    }
    s.pendingAdjust      = adjustPending_;
    s.adjustReq          = adjustReq_;
    s.pendingPick        = pickPending_;
    s.pickReq            = pickReq_;
    s.stepMode           = stepMode_.load();
    s.teach              = teach_.load();
    s.dryRun             = dryRun_.load();
    s.stepPaused         = stepPaused_;
    s.nextStep           = nextStep_;
    s.nextPath           = nextPath_;
    if (runCtx_) s.ctx = runCtx_->CtxSnapshot();   // its own mutex; no deadlock
    s.lastCtx = lastCtx_;
    return s;
}

bool RecipeEngine::SafetyAborted() const
{
    return wf_.SafetyAlertActive() && wf_.SafetyAlert().critical;
}

bool RecipeEngine::AwaitPromptInternal(const std::string& message, bool wantsInput,
                                       std::string* textOut, int autoOkMs,
                                       PromptLevel level, const std::vector<std::string>& images)
{
    using clock = std::chrono::steady_clock;
    std::unique_lock lock(mu_);
    prompt_           = message;
    promptWantsInput_ = wantsInput;
    promptLevel_      = level;
    promptImages_     = images;
    promptText_.clear();
    promptAnswered_ = false;
    promptAnswer_   = false;
    promptAutoOkAt_ = autoOkMs > 0 ? clock::now() + std::chrono::milliseconds(autoOkMs)
                                   : clock::time_point{};
    // Wait in slices so a stop request always gets through.
    while (!promptAnswered_ && !stopReq_.load()) {
        // Nobody answered inside the window: that IS the answer, OK.
        if (autoOkMs > 0 && clock::now() >= promptAutoOkAt_) {
            FD_INFO("prompt", "no answer in %d ms - continuing (auto-OK): %s", autoOkMs,
                    message.c_str());
            promptAnswered_ = true;
            promptAnswer_   = true;
            break;
        }
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    prompt_.clear();
    promptWantsInput_ = false;
    promptAutoOkAt_   = clock::time_point{};
    const bool ok = promptAnswered_ && promptAnswer_ && !stopReq_.load();
    if (ok && textOut) *textOut = promptText_;
    return ok;
}

// The prompt handshake with the author's buttons: same slot, same window, same
// stop behaviour - only the answer is an index instead of OK / Cancel.
int RecipeEngine::AwaitChoiceInternal(const std::string& message,
                                      const std::vector<std::string>& buttons,
                                      PromptLevel level)
{
    std::unique_lock lock(mu_);
    prompt_           = message;
    promptWantsInput_ = false;
    promptLevel_      = level;
    promptButtons_    = buttons;
    promptImages_.clear();   // a choice carries no pictures
    promptChoice_     = -1;
    promptAnswered_   = false;
    promptAnswer_     = false;
    promptAutoOkAt_   = std::chrono::steady_clock::time_point{};   // a choice never answers itself
    // Wait in slices so a stop request always gets through.
    while (!promptAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    prompt_.clear();
    promptButtons_.clear();
    return (promptAnswered_ && !stopReq_.load()) ? promptChoice_ : -1;
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

bool RecipeEngine::AwaitPickInternal(const ExecutionContext::PickRequest& req,
                                     ExecutionContext::PickResult* out)
{
    std::unique_lock lock(mu_);
    pickReq_      = req;
    pickPending_  = true;
    pickAnswered_ = false;
    pickAnswer_   = false;
    // Wait in slices so a stop request always gets through.
    while (!pickAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    pickPending_ = false;
    const bool ok = pickAnswered_ && pickAnswer_ && !stopReq_.load();
    if (ok && out) *out = pickResult_;
    return ok;
}

bool RecipeEngine::AwaitStepInternal(const std::string& label,
                                     const std::vector<std::size_t>& path)
{
    if (!stepMode_.load()) return true;   // free-running — no gate
    std::unique_lock lock(mu_);
    stepPaused_ = true;
    nextStep_   = label;
    nextPath_   = path;
    // Wait in slices so stop / safety / SetStepMode(false) always get through.
    while (!stepGranted_ && stepMode_.load() && !stopReq_.load() &&
           !SafetyAborted()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    stepGranted_ = false;
    stepPaused_  = false;
    nextStep_.clear();
    nextPath_.clear();
    return !stopReq_.load() && !SafetyAborted();
}

void RecipeEngine::ThreadMain(Recipe recipe, Recipe onFail, Env env)
{
    ExecutionContext ctx(
        wf_, hw_, env.positions, env.maps, env.aligns,
        env.visionModels, env.grips, env.trays, env.visionDir, env.scriptsDir,
        ExecutionContext::Hooks{
            [this] { return stopReq_.load() || SafetyAborted(); },
            [this](const std::string& m, int autoOkMs, PromptLevel level,
                   const std::vector<std::string>& images) {
                return AwaitPromptInternal(m, false, nullptr, autoOkMs, level, images);
            },
            [this](const std::string& m, std::string* out) {
                return AwaitPromptInternal(m, true, out);
            },
            [this](const ExecutionContext::AdjustRequest& r) {
                return AwaitAdjustInternal(r);
            },
            [this](const ExecutionContext::PickRequest& r,
                   ExecutionContext::PickResult* out) {
                return AwaitPickInternal(r, out);
            },
            [this](const std::string& m, const std::vector<std::string>& b,
                   PromptLevel level) {
                return AwaitChoiceInternal(m, b, level);
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
                [this](const std::string& label, const std::vector<std::size_t>& path) {
                    return AwaitStepInternal(label, path);
                },
            });
        out    = runner.Run(recipe.steps, startPath_);
        runErr = runner.Error();
    } catch (const std::exception& e) {
        // Should have been contained at the step boundary — last line of defense.
        FD_ERROR("recipe", "unexpected engine exception: %s", e.what());
        runErr = e.what();
    } catch (...) {
        FD_ERROR("recipe", "unexpected non-standard engine exception");
        runErr = "unknown error";
    }

    // The recipe's own outcome is decided HERE, before any on-fail recipe runs:
    // a cleanup that succeeds does not make the run a success, and a Stop
    // pressed during the cleanup does not turn a failed part into a stopped one.
    const bool stopped = stopReq_.load() || out == StepRunner::Outcome::Aborted;
    const bool failed  = !stopped && out != StepRunner::Outcome::Ok &&
                         out != StepRunner::Outcome::StopOk;

    // On-fail recipe: only after a FAILURE. An operator Stop or a safety abort
    // means "stop moving", so neither runs it. Same ctx (it is still this run,
    // as a sub_recipe would be), no step-mode gate (a cleanup is not something
    // to single-step through), and the editor highlight is cleared - its step
    // paths address another recipe. The ctx's stop hook still applies.
    std::string onFailResult;   // "" = none ran
    if (failed && !onFail.uid.empty() && !SafetyAborted()) {
        FD_WARN("recipe", "'%s' failed (%s) - running on-fail recipe '%s'",
                recipe.name.c_str(), runErr.empty() ? "step failed" : runErr.c_str(),
                onFail.name.c_str());
        {
            const std::scoped_lock lock(mu_);
            currentStep_ = "on-fail '" + onFail.name + "'";
            curPath_.clear();
            leafDone_  = 0;
            leafTotal_ = CountLeafSteps(onFail.steps);
        }
        StepRunner::Outcome fOut = StepRunner::Outcome::Fail;
        std::string fErr;
        try {
            StepRunner runner(
                reg_, ctx,
                StepRunner::Callbacks{
                    [this, &onFail](const std::string& label,
                                    const std::vector<std::size_t>&) {
                        const std::scoped_lock lock(mu_);
                        currentStep_ = "on-fail '" + onFail.name + "' " + label;
                        curPath_.clear();
                    },
                    [this] {
                        const std::scoped_lock lock(mu_);
                        ++leafDone_;
                    },
                    nullptr,
                });
            fOut = runner.Run(onFail.steps);
            fErr = runner.Error();
        } catch (const std::exception& e) {
            FD_ERROR("recipe", "unexpected engine exception in on-fail recipe: %s", e.what());
            fErr = e.what();
        } catch (...) {
            FD_ERROR("recipe", "unexpected non-standard exception in on-fail recipe");
            fErr = "unknown error";
        }
        const char* fState =
            (fOut == StepRunner::Outcome::Ok || fOut == StepRunner::Outcome::StopOk)
                ? StateName(State::Completed)
            : (stopReq_.load() || fOut == StepRunner::Outcome::Aborted)
                ? StateName(State::Stopped)
                : StateName(State::Failed);
        onFailResult = fState;
        FD_INFO("recipe", "on-fail recipe '%s' finished: %s%s%s", onFail.name.c_str(),
                fState, fErr.empty() ? "" : " - ", fErr.c_str());
        if (runErr.empty()) runErr = "step failed";
        runErr += " [on-fail '" + onFail.name + "': " + fState +
                  (fErr.empty() ? "" : " - " + fErr) + "]";
    }

    // Terminal state — every path lands here with the cause logged.
    {
        const std::scoped_lock lock(mu_);
        // Capture BEFORE dropping the pointer. A UI polling Status.ctx per
        // frame would miss a value written by the recipe's final step, since
        // the run can end between two frames.
        lastCtx_ = ctx.CtxSnapshot();
        runCtx_  = nullptr;   // ctx dies with this frame; status stops reading it
        error_  = runErr;
        if (stopped) {
            state_ = State::Stopped;
            if (error_.empty())
                error_ = SafetyAborted() ? "safety alert" : "stopped by operator";
        } else if (!failed) {
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
        // A teach run drove the machine but proved nothing about a part; nor
        // did a dry run, which skipped every step that touches one.
        if (!teach_.load() && !dryRun_.load()) {
            DutSession::Instance().RecordRun("recipe", recipeName_, StateName(state_));
            // The cleanup moved the machine with the part in it: on the record too.
            if (!onFailResult.empty())
                DutSession::Instance().RecordRun("on_fail", onFail.name, onFailResult);
        }
    }
    running_.store(false);
}

} // namespace firedog
