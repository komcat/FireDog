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
        SavedPosMap         positions;
        MotionMapSet        maps;
        std::vector<Recipe> library;
    };

    // Validates (structure + params + references + devices Connected), inlines
    // sub-recipes, then launches the worker. False + err if a run is already
    // active, the UI path executor is busy, or validation fails.
    bool Start(const Recipe& recipe, Env env, std::string* err = nullptr);
    void RequestStop();            // cooperative; honored within one poll interval
    void AnswerPrompt(bool ok);    // resolves a blocking `prompt` step

    enum class State { Idle, Running, Completed, Failed, Stopped };
    static const char* StateName(State s);

    struct Status {
        State       state = State::Idle;
        std::string recipeName;
        std::string currentStep;      // "3.2 (move_to_position) attempt 2/3"
        std::size_t leafDone = 0, leafTotal = 0;
        std::string error;            // terminal cause when Failed/Stopped
        std::string pendingPrompt;    // non-empty while a prompt blocks
        std::vector<std::pair<std::string, CtxValue>> ctx;   // display snapshot
    };
    Status GetStatus() const;        // mutex-guarded copy — cheap, UI-safe
    bool   Running() const { return running_.load(); }

private:
    enum class Outcome { Ok, Fail, StopOk, StopFail, Aborted };

    void    ThreadMain(Recipe recipe, Env env);
    Outcome ExecSteps(const std::vector<RecipeStep>& steps, ExecutionContext& ctx,
                      const std::string& pathPrefix, int depth);
    Outcome ExecLeaf(const RecipeStep& s, ExecutionContext& ctx,
                     const std::string& where);
    bool    SafetyAborted() const;   // critical unacknowledged alert?
    void    SetCurrentStep(const std::string& s);

    // Prompt handshake (worker blocks; UI answers). Guarded by mu_.
    bool AwaitPromptInternal(const std::string& message);

    Workflow&              wf_;
    HardwareManager&       hw_;
    const CommandRegistry& reg_;

    std::atomic<bool> stopReq_{false};
    std::atomic<bool> running_{false};

    mutable std::mutex      mu_;       // guards everything below
    std::condition_variable cv_;       // prompt handshake
    State       state_ = State::Idle;
    std::string recipeName_;
    std::string currentStep_;
    std::string error_;
    std::string prompt_;               // pending prompt text ("" = none)
    bool        promptAnswered_ = false;
    bool        promptAnswer_   = false;
    std::size_t leafDone_ = 0, leafTotal_ = 0;
    ExecutionContext* runCtx_ = nullptr;   // valid only while the worker runs;
                                           // set/cleared under mu_

    std::thread worker_;   // joined in Stop-and-join helper + dtor. Declared
                           // last so everything it uses outlives it.
};

} // namespace firedog
