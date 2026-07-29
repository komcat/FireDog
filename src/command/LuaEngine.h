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
#include <string>
#include <thread>
#include <utility>
#include <vector>

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
        SavedPosMap         positions;
        MotionMapSet        maps;
        std::vector<Recipe> library;   // reserved for fd.run_recipe (Phase C2)
    };

    bool Start(const std::string& scriptPath, Env env, std::string* err = nullptr);
    void RequestStop();            // cooperative; fd waits + the Lua hook honor it
    void AnswerPrompt(bool ok);    // resolves a blocking fd.prompt()

    enum class State { Idle, Running, Completed, Failed, Stopped };
    static const char* StateName(State s);

    struct Status {
        State       state = State::Idle;
        std::string scriptName;
        int         currentLine = 0;   // from the line hook (source highlight)
        std::string lastCommand;       // last fd.* command issued
        std::string error;             // terminal cause when Failed/Stopped
        std::string pendingPrompt;     // non-empty while fd.prompt blocks
        std::vector<std::pair<std::string, CtxValue>> ctx;
    };
    Status GetStatus() const;
    bool   Running() const { return running_.load(); }

    // Internal — called from the Lua hook thunk (must be public-ish for the
    // C callback; not part of the user API).
    bool HookShouldStop() const;
    void HookLine(int line) { curLine_.store(line); }
    void NoteCommand(const std::string& name);

private:
    void ThreadMain(std::string path, Env env);
    bool AwaitPromptInternal(const std::string& message);
    bool SafetyAborted() const;

    Workflow&              wf_;
    HardwareManager&       hw_;
    const CommandRegistry& reg_;

    std::atomic<bool> stopReq_{false};
    std::atomic<bool> running_{false};
    std::atomic<int>  curLine_{0};

    mutable std::mutex      mu_;   // guards everything below
    std::condition_variable cv_;   // prompt handshake
    State       state_ = State::Idle;
    std::string scriptName_;
    std::string lastCommand_;
    std::string error_;
    std::string prompt_;
    bool        promptAnswered_ = false;
    bool        promptAnswer_   = false;
    ExecutionContext* runCtx_ = nullptr;   // valid only while the worker runs

    std::thread worker_;   // declared last so everything it uses outlives it
};

} // namespace firedog
