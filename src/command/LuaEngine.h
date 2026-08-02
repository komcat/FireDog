// LuaEngine.h : layer 2.5 — the Tier-2 Lua program executor.
//
// Runs plain .lua files that orchestrate the machine through the SAME command
// registry recipes use: bindings are generated per registered command
// (fd.move_to_position{...}), validated by the same ParamSpec schemas, and
// executed through the same ExecutionContext — so blocking waits abort on
// Stop exactly like recipe steps. A count/line hook additionally kills
// pure-Lua runaway loops and reports the executing line to the UI.
//
// Same RAII contract as RecipeEngine: one worker std::thread, the destructor
// requests stop and joins ALWAYS, every exit path lands in a terminal state,
// a failed command raises a Lua error (the script's pcall is its "on_error").
// Mutual exclusion with the recipe engine and the UI path executor is
// enforced by App (both Run surfaces check both engines) plus the PathActive
// check here — one orchestration owner, always.
//
// sol2/Lua headers never leak out of the .cpp.
#pragma once

#include "command/ExecutionContext.h"
#include "command/Recipe.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// Lua's C typedefs name the same struct tags — forward-declared so the
// debugger entry point can appear here without leaking Lua headers.
struct lua_State;
struct lua_Debug;

namespace firedog {

class CommandRegistry;
class Workflow;
class HardwareManager;

// A runnable .lua program on disk. `description` is the first line's
// "-- comment" (shown in the Programs list).
struct ScriptInfo {
    std::string path;
    std::string name;          // filename without extension
    std::string description;
};

// Scan `dir` for *.lua (sorted by name). Missing dir -> empty, never throws.
std::vector<ScriptInfo> ListScripts(const std::string& dir);

class LuaEngine;

// Run a .lua file synchronously on the CALLING thread with the standard fd
// bindings over `ctx`. `library` + `allowRunRecipe` gate fd.run_recipe — a
// script started by a recipe step gets false (recursion cap: depth 1, no
// script->recipe->script re-entrancy). `engine` (nullable) receives line /
// last-command reporting. Returns "" on success, else the first error line
// (full traceback goes to the Logger; a stop surfaces as "stopped"). Never
// throws. Used by LuaEngine's worker and by the run_script command.
std::string RunLuaFile(const std::string& path, const CommandRegistry& reg,
                       ExecutionContext& ctx, const std::vector<Recipe>& library,
                       bool allowRunRecipe, LuaEngine* engine);

class LuaEngine {
public:
    LuaEngine(Workflow& wf, HardwareManager& hw, const CommandRegistry& reg);
    ~LuaEngine();   // RequestStop + join — always
    LuaEngine(const LuaEngine&) = delete;
    LuaEngine& operator=(const LuaEngine&) = delete;

    // Resolve environment, copied at Start (stable for the whole run).
    struct Env {
        SavedPosMap            positions;
        MotionMapSet           maps;
        std::vector<Recipe>    library;   // fd.run_recipe resolves against this
        std::vector<AlignProc> aligns;    // taught local frames (fd.align_*)
        std::vector<VisionModel> visionModels;   // taught detectors (fd.vision_*)
        std::string              visionDir;      // where their template patches live
        // Seeded into the run's ctx before the script starts — the UI-side
        // parameter injection (fd.param reads these exactly like values a
        // recipe injected with ctx_set steps).
        std::vector<std::pair<std::string, CtxValue>> initialCtx;
    };

    bool Start(const std::string& scriptPath, Env env, std::string* err = nullptr);
    void RequestStop();            // cooperative; fd waits + the Lua hook honor it
    // Resolves a blocking fd.prompt() / fd.input() (text used by input).
    void AnswerPrompt(bool ok, std::string text = std::string());
    // Resolves a blocking fd.manual_adjust{} (jogging happened UI-side).
    void AnswerAdjust(bool ok);
    // Resolves a blocking fd.align_station{} crosshair pick (marked UI-side).
    void AnswerPick(bool ok, ExecutionContext::PickResult result = {});

    // ---- Debugger (C3) — all built on the existing line hook ---------------
    // Breakpoints are hot-swappable while running (the UI passes the selected
    // script's set). Pause breaks at the next executed line; Step runs to the
    // next line event and pauses again. While paused the Lua worker blocks in
    // the hook (Stop and safety aborts still release it).
    void SetBreakpoints(std::set<int> lines);
    void Pause();
    void Resume();
    void StepLine();

    enum class State { Idle, Running, Completed, Failed, Stopped };
    static const char* StateName(State s);

    struct Status {
        State       state = State::Idle;
        std::string scriptName;
        int         currentLine = 0;   // from the line hook (source highlight)
        bool        paused = false;    // blocked at a breakpoint / pause / step
        std::string lastCommand;       // last fd.* command issued
        std::string error;             // terminal cause when Failed/Stopped
        std::string pendingPrompt;     // non-empty while fd.prompt/input blocks
        bool        pendingPromptInput = false;   // the prompt wants a text answer
        // Manual-adjust handshake: valid while pendingAdjust is true.
        bool        pendingAdjust = false;
        ExecutionContext::AdjustRequest adjustReq;
        // Crosshair-pick handshake: valid while pendingPick is true.
        bool        pendingPick = false;
        ExecutionContext::PickRequest pickReq;
        std::vector<std::pair<std::string, CtxValue>> ctx;
        // Current function's locals, captured at the pause point.
        std::vector<std::pair<std::string, std::string>> locals;
    };
    Status GetStatus() const;
    bool   Running() const { return running_.load(); }

    // Internal — called from the Lua hook thunk (must be public-ish for the
    // C callback; not part of the user API).
    bool HookShouldStop() const;
    void OnLineHook(lua_State* L, lua_Debug* ar);   // line update + debugger
    void NoteCommand(const std::string& name);

private:
    void ThreadMain(std::string path, Env env);
    bool AwaitPromptInternal(const std::string& message, bool wantsInput,
                             std::string* textOut);
    bool AwaitAdjustInternal(const ExecutionContext::AdjustRequest& req);
    bool AwaitPickInternal(const ExecutionContext::PickRequest& req,
                           ExecutionContext::PickResult* out);
    bool SafetyAborted() const;

    Workflow&              wf_;
    HardwareManager&       hw_;
    const CommandRegistry& reg_;

    std::atomic<bool> stopReq_{false};
    std::atomic<bool> running_{false};
    std::atomic<int>  curLine_{0};
    std::atomic<bool> pauseReq_{false};   // break at the next executed line
    std::atomic<bool> stepReq_{false};    // one line, then pause again

    mutable std::mutex      mu_;   // guards everything below
    std::condition_variable cv_;   // prompt + pause handshakes
    std::set<int> breakpoints_;
    bool          paused_   = false;
    bool          resumeGo_ = false;
    std::vector<std::pair<std::string, std::string>> locals_;
    State       state_ = State::Idle;
    std::string scriptName_;
    std::string lastCommand_;
    std::string error_;
    std::string prompt_;
    bool        promptWantsInput_ = false;
    std::string promptText_;
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
    ExecutionContext* runCtx_ = nullptr;   // valid only while the worker runs

    std::thread worker_;   // declared last so everything it uses outlives it
};

} // namespace firedog
