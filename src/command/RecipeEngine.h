// RecipeEngine.h : layer 2.5 — the recipe executor.
//
// One worker thread, one run at a time. ~RecipeEngine requests stop and joins,
// ALWAYS (a plain std::thread with an explicit stop flag — jthread's built-in
// stop_token would join without raising OUR cooperative flag); detached
// threads are forbidden in this layer.
// The UI thread only calls Start/RequestStop/AnswerPrompt and polls
// GetStatus() — a running recipe can never block or stutter the interface.
//
// Every exit path leaves a defined state: exceptions are contained at the
// step boundary (a throwing command = a failed step), a critical safety alert
// aborts the run between steps, and there is no path that leaves the engine
// "Running" with no thread behind it. Hardware-off is never implicit: abort
// stops the engine (and motion commands stop their device), but instruments
// and outputs are only switched by authored steps or the safety layer.
#pragma once

#include "command/ExecutionContext.h"
#include "command/Recipe.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace firedog {

class CommandRegistry;
class Workflow;
class HardwareManager;

class RecipeEngine {
public:
    RecipeEngine(Workflow& wf, HardwareManager& hw, const CommandRegistry& reg);
    ~RecipeEngine();   // RequestStop + join — always
    RecipeEngine(const RecipeEngine&) = delete;
    RecipeEngine& operator=(const RecipeEngine&) = delete;

    // The run's resolve environment, COPIED at Start so it is stable for the
    // whole run (the UI may edit positions/maps while a recipe executes).
    // `library` resolves sub_recipe references.
    struct Env {
        SavedPosMap            positions;
        MotionMapSet           maps;
        std::vector<Recipe>    library;
        std::vector<AlignProc> aligns;   // taught local frames (align_* commands)
        std::vector<VisionModel> visionModels;   // taught detectors (vision_* commands)
        std::vector<GripProfile> grips;          // gripped-object profiles (grip_*)
        std::string              visionDir;      // where their template patches live
        std::string              scriptsDir;     // product scripts win over config/scripts
    };

    // Step-mode start options: with stepMode the worker pauses BEFORE every
    // leaf until StepAdvance() grants it (or SetStepMode(false) resumes free
    // running); startPath skips every leaf before that numeric step path
    // (else-branch markers are stripped — a start inside an else branch lands
    // on the then-branch row, the same documented quirk as run highlighting).
    struct StartOpts {
        bool stepMode = false;
        std::vector<std::size_t> startPath;   // {} = from the beginning
    };

    // Validates (structure + params + references + devices Connected), inlines
    // sub-recipes, then launches the worker. False + err if a run is already
    // active, the UI path executor is busy, or validation fails.
    bool Start(const Recipe& recipe, Env env, std::string* err = nullptr,
               StartOpts opts = {});
    void RequestStop();            // cooperative; honored within one poll interval
    // Step mode: grant exactly one paused leaf (no-op unless paused).
    void StepAdvance();
    // Toggle step mode mid-run: off releases a pending pause (free-run from
    // here); on pauses before the NEXT leaf. Also arms/disarms nothing at
    // Idle — Start's opts decide the next run's mode.
    void SetStepMode(bool on);
    // Resolves a blocking `prompt` / `prompt_input` step (text used by input).
    void AnswerPrompt(bool ok, std::string text = std::string());
    // Resolves a blocking `manual_adjust` step (jogging happened UI-side).
    void AnswerAdjust(bool ok);
    // Resolves a blocking `align_station` crosshair pick (marked UI-side).
    void AnswerPick(bool ok, ExecutionContext::PickResult result = {});

    enum class State { Idle, Running, Completed, Failed, Stopped };
    static const char* StateName(State s);

    struct Status {
        State       state = State::Idle;
        std::string recipeName;
        std::string currentStep;      // "3.2 (move_to_position) attempt 2/3"
        // Numeric 0-based path of the executing leaf (e.g. {2,1} = step 3.2)
        // so the editor's tree can highlight without parsing the string. An
        // if-step's else-branch reports the same indices as its then-branch
        // (the engine addresses whichever branch ran).
        std::vector<std::size_t> stepPath;
        std::size_t leafDone = 0, leafTotal = 0;
        std::string error;            // terminal cause when Failed/Stopped
        std::string pendingPrompt;    // non-empty while a prompt blocks
        bool        pendingPromptInput = false;   // the prompt wants a text answer
        // Manual-adjust handshake: valid while pendingAdjust is true.
        bool        pendingAdjust = false;
        ExecutionContext::AdjustRequest adjustReq;
        // Crosshair-pick handshake: valid while pendingPick is true.
        bool        pendingPick = false;
        ExecutionContext::PickRequest pickReq;
        // Step mode: while stepPaused, nextStep/nextPath describe the leaf
        // that will run when StepAdvance() grants it.
        bool        stepMode   = false;
        bool        stepPaused = false;
        std::string nextStep;
        std::vector<std::size_t> nextPath;
        std::vector<std::pair<std::string, CtxValue>> ctx;   // display snapshot
        // The ctx as it stood the instant the run ENDED, captured before the
        // engine drops its pointer. `ctx` above is live and therefore empty by
        // the time the state is terminal — this is the only way a UI can read
        // what a finished recipe left behind.
        //
        // DISPLAY ONLY. A command handing data back still goes through a
        // Workflow queue drained in App::Update; do not turn this into a second
        // result channel, or two mechanisms will disagree about what a run
        // produced. It is cleared at Start, so it always describes the LAST
        // finished run and never a stale one.
        std::vector<std::pair<std::string, CtxValue>> lastCtx;
    };
    Status GetStatus() const;        // mutex-guarded copy — cheap, UI-safe
    bool   Running() const { return running_.load(); }

private:
    // Step execution itself lives in StepRunner (shared with fd.run_recipe);
    // this class is the worker thread + status plumbing around it.
    void ThreadMain(Recipe recipe, Env env);
    bool SafetyAborted() const;   // critical unacknowledged alert?

    // Prompt handshakes (worker blocks; UI answers). Guarded by mu_.
    bool AwaitPromptInternal(const std::string& message, bool wantsInput,
                             std::string* textOut);
    bool AwaitAdjustInternal(const ExecutionContext::AdjustRequest& req);
    bool AwaitPickInternal(const ExecutionContext::PickRequest& req,
                           ExecutionContext::PickResult* out);
    // Step-mode gate: blocks before a leaf until granted/released; false =
    // stop or safety abort (the run ends Stopped).
    bool AwaitStepInternal(const std::string& label,
                           const std::vector<std::size_t>& path);

    Workflow&              wf_;
    HardwareManager&       hw_;
    const CommandRegistry& reg_;

    std::atomic<bool> stopReq_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> stepMode_{false};   // pause before every leaf while true

    mutable std::mutex      mu_;       // guards everything below
    std::condition_variable cv_;       // prompt handshake
    State       state_ = State::Idle;
    std::string recipeName_;
    std::string currentStep_;
    std::vector<std::size_t> curPath_;   // numeric path of currentStep_
    std::string error_;
    std::string prompt_;               // pending prompt text ("" = none)
    bool        promptWantsInput_ = false;
    std::string promptText_;           // the UI's typed answer
    bool        promptAnswered_ = false;
    bool        promptAnswer_   = false;
    // Manual-adjust handshake (same shape as the prompt one).
    bool        adjustPending_  = false;
    ExecutionContext::AdjustRequest adjustReq_;
    bool        adjustAnswered_ = false;
    bool        adjustAnswer_   = false;
    // Crosshair-pick handshake (same shape again).
    bool        pickPending_  = false;
    ExecutionContext::PickRequest pickReq_;
    ExecutionContext::PickResult  pickResult_;
    bool        pickAnswered_ = false;
    bool        pickAnswer_   = false;
    // Step-mode handshake (worker pauses; UI grants). Guarded by mu_.
    bool        stepGranted_ = false;
    bool        stepPaused_  = false;
    std::string nextStep_;                    // label of the paused leaf
    std::vector<std::size_t> nextPath_;       // its numeric path
    std::vector<std::size_t> startPath_;      // skip-until path for this run
    std::size_t leafDone_ = 0, leafTotal_ = 0;
    ExecutionContext* runCtx_ = nullptr;   // valid only while the worker runs;
                                           // set/cleared under mu_
    // Snapshot of runCtx_ taken at the terminal boundary — see Status::lastCtx.
    std::vector<std::pair<std::string, CtxValue>> lastCtx_;

    std::thread worker_;   // joined in Stop-and-join helper + dtor. Declared
                           // last so everything it uses outlives it.
};

} // namespace firedog
