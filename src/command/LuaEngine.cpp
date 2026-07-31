// LuaEngine.cpp
#include "command/LuaEngine.h"

#include "command/CommandRegistry.h"
#include "command/StepRunner.h"
#include "core/DataBus.h"
#include "core/DutSession.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <sol/sol.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>

namespace firedog {

// ---- Script discovery -------------------------------------------------------

std::vector<ScriptInfo> ListScripts(const std::string& dir)
{
    namespace fs = std::filesystem;
    std::vector<ScriptInfo> out;
    std::error_code ec;
    for (const fs::directory_entry& e : fs::directory_iterator(dir, ec)) {
        if (!e.is_regular_file(ec) || e.path().extension() != ".lua") continue;
        ScriptInfo si;
        si.path = e.path().string();
        si.name = e.path().stem().string();
        std::ifstream f(e.path());
        std::string first;
        if (f.is_open() && std::getline(f, first) && first.rfind("--", 0) == 0) {
            std::size_t i = 2;
            while (i < first.size() && (first[i] == '-' || first[i] == ' ')) ++i;
            si.description = first.substr(i);
            while (!si.description.empty() &&
                   (si.description.back() == '\r' || si.description.back() == ' '))
                si.description.pop_back();
        }
        out.push_back(std::move(si));
    }
    std::sort(out.begin(), out.end(),
              [](const ScriptInfo& a, const ScriptInfo& b) { return a.name < b.name; });
    return out;
}

// ---- Lua hook (stop + line reporting) ---------------------------------------
// A pointer to this stack-resident context travels in the lua_State's extra
// space; the hook fires every LINE (source highlight, engine runs only) and
// every N instructions, so a pure-Lua `while true do end` still honors Stop.
// Raising a Lua error from the hook is the sanctioned abort path (longjmp
// through interpreter C frames only).

namespace {

constexpr int kHookInstrCount = 200;

struct HookCtx {
    ExecutionContext* ctx    = nullptr;
    LuaEngine*        engine = nullptr;   // null for run_script sub-runs
};

HookCtx* HookOf(lua_State* L)
{
    return *static_cast<HookCtx**>(lua_getextraspace(L));
}

void LuaStopHook(lua_State* L, lua_Debug* ar)
{
    HookCtx* h = HookOf(L);
    if (!h || !h->ctx) return;
    if (ar->event == LUA_HOOKLINE && h->engine)
        h->engine->OnLineHook(L, ar);   // line report + debugger (may block)
    if (h->ctx->ShouldStop()) luaL_error(L, "stopped");
}

// Lua argument -> json (bool / number / string; integral doubles become json
// integers so Int-typed params validate the same as hand-written JSON).
nlohmann::json LuaValueToJson(const sol::object& v, std::string* err)
{
    switch (v.get_type()) {
    case sol::type::boolean: return v.as<bool>();
    case sol::type::number: {
        const double d = v.as<double>();
        if (std::floor(d) == d && std::fabs(d) < 9.0e15)
            return static_cast<std::int64_t>(d);
        return d;
    }
    case sol::type::string: return v.as<std::string>();
    default:
        if (err) *err = "unsupported value type (use bool/number/string)";
        return {};
    }
}

sol::object CtxValueToLua(sol::this_state ts, const CtxValue& v)
{
    if (std::holds_alternative<bool>(v)) return sol::make_object(ts, std::get<bool>(v));
    if (std::holds_alternative<double>(v)) return sol::make_object(ts, std::get<double>(v));
    return sol::make_object(ts, std::get<std::string>(v));
}

} // namespace

// ---- RunLuaFile: state + bindings + execution (shared) ----------------------
// LuaEngine's worker calls this with engine=this + allowRunRecipe=true; the
// run_script command calls it inline on the recipe worker with engine=null +
// allowRunRecipe=false (recursion cap).

std::string RunLuaFile(const std::string& path, const CommandRegistry& reg,
                       ExecutionContext& ctx, const std::vector<Recipe>& library,
                       bool allowRunRecipe, LuaEngine* engine)
{
    std::string firstErr;
    try {
        sol::state lua;
        lua.open_libraries(sol::lib::base, sol::lib::math, sol::lib::string,
                           sol::lib::table);
        HookCtx hookCtx{&ctx, engine};
        *static_cast<HookCtx**>(lua_getextraspace(lua.lua_state())) = &hookCtx;
        lua_sethook(lua.lua_state(), &LuaStopHook,
                    LUA_MASKCOUNT | LUA_MASKLINE, kHookInstrCount);

        // ---- fd: one generated function per registered command ----------
        sol::table fd = lua.create_named_table("fd");
        for (const CommandSpec* c : reg.List()) {
            const CommandSpec* spec = c;
            fd.set_function(spec->name,
                            [spec, engine, &ctx](sol::optional<sol::table> args) {
                nlohmann::json params = nlohmann::json::object();
                if (args) {
                    for (const auto& [k, v] : *args) {
                        if (k.get_type() != sol::type::string)
                            throw std::runtime_error(spec->name +
                                                     ": argument keys must be names");
                        std::string convErr;
                        nlohmann::json jv = LuaValueToJson(v, &convErr);
                        if (!convErr.empty())
                            throw std::runtime_error(spec->name + "." +
                                                     k.as<std::string>() + ": " + convErr);
                        params[k.as<std::string>()] = std::move(jv);
                    }
                }
                nlohmann::json merged;
                const std::string ve = ValidateParams(*spec, params, &merged);
                if (!ve.empty()) throw std::runtime_error(spec->name + ": " + ve);
                if (engine) engine->NoteCommand(spec->name);
                const CommandResult r =
                    ExecuteWithDelays(*spec, ctx, ParamValues(std::move(merged)));
                if (!r.ok) throw std::runtime_error(spec->name + ": " + r.error);
            });
        }

        // ---- fd.run_recipe (Lua worker only — recursion cap depth 1) -----
        if (allowRunRecipe) {
            fd.set_function("run_recipe",
                            [&reg, &ctx, &library, engine](const std::string& ref) {
                const Recipe* found = nullptr;
                for (const Recipe& r : library) {
                    if (r.uid == ref || r.name == ref) { found = &r; break; }
                }
                if (!found)
                    throw std::runtime_error("run_recipe: '" + ref + "' not found");
                const std::vector<RecipeIssue> issues = ValidateRecipe(
                    *found, reg, ctx.Hw(), ctx.Positions(), ctx.Maps(), library, true);
                if (!issues.empty())
                    throw std::runtime_error("run_recipe '" + found->name + "': " +
                                             issues.front().where + ": " +
                                             issues.front().message);
                Recipe run = *found;
                std::string ie;
                if (!InlineSubRecipes(run, library, &ie))
                    throw std::runtime_error("run_recipe '" + found->name + "': " + ie);
                FD_INFO("script", "run_recipe '%s' (%zu leaf step(s))",
                        run.name.c_str(), CountLeafSteps(run.steps));
                // Same ExecutionContext: shared ctx values, same stop token.
                StepRunner runner(
                    reg, ctx,
                    StepRunner::Callbacks{
                        [engine, &run](const std::string& label,
                                       const std::vector<std::size_t>&) {
                            if (engine) engine->NoteCommand(run.name + " " + label);
                        },
                        nullptr,
                    });
                const StepRunner::Outcome out = runner.Run(run.steps);
                if (out == StepRunner::Outcome::Aborted)
                    throw std::runtime_error("stopped");
                if (out == StepRunner::Outcome::Fail ||
                    out == StepRunner::Outcome::StopFail)
                    throw std::runtime_error("run_recipe '" + run.name + "': " +
                                             runner.Error());
            });
        }

        // ---- Hand-written conveniences ----------------------------------
        // Returns value, count. The count is the topic's monotonic publish
        // counter — scripts gate on it to demand samples measured strictly
        // after a move (fresh-sample settle). Count is read BEFORE the value
        // so a racing publish can only make the value fresher than the gate,
        // never staler. Extra return is invisible to `local v = fd.read(t)`.
        fd.set_function("read", [](sol::this_state ts, const std::string& topic)
                                    -> std::tuple<sol::object, sol::object> {
            DataStream& s = DataBus::Instance().Stream(topic);
            const std::size_t n   = s.Count();
            sol::object       cnt = sol::make_object(ts, static_cast<double>(n));
            if (n == 0) return {sol::make_object(ts, sol::lua_nil), cnt};
            return {sol::make_object(ts, s.LatestValue()), cnt};
        });
        // Takes double: ctx-injected numbers are Lua floats (150.0), and sol's
        // safeties reject float->int even when integral.
        fd.set_function("sleep", [&ctx](double ms) {
            if (!ctx.Sleep(static_cast<int>(ms))) throw std::runtime_error("stopped");
        });
        fd.set_function("log", [](const std::string& msg) {
            FD_INFO("script", "%s", msg.c_str());
        });
        fd.set_function("prompt", [&ctx](const std::string& msg) {
            return ctx.AwaitPrompt(msg);
        });
        // Text-input prompt (operator keyboard / barcode scanner): returns
        // the string, or nil on Cancel.
        fd.set_function("input", [&ctx](sol::this_state ts, const std::string& msg)
                                     -> sol::object {
            std::string text;
            if (!ctx.AwaitInput(msg, &text)) return sol::make_object(ts, sol::lua_nil);
            return sol::make_object(ts, text);
        });
        fd.set_function("clock", [] {
            using clock = std::chrono::steady_clock;
            return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
        });
        // Script parameter with a default: reads the ctx key (which a recipe
        // injects via ctx_set steps ahead of run_script), falls back to the
        // default otherwise, and LOGS the resolved value so every run records
        // what the algorithm actually used.
        fd.set_function("param", [&ctx](sol::this_state ts, const std::string& name,
                                        sol::object def) -> sol::object {
            const std::optional<CtxValue> v = ctx.CtxGet(name);
            if (v) {
                FD_INFO("script", "param %s = %s (from recipe/ctx)", name.c_str(),
                        CtxValueToString(*v).c_str());
                return CtxValueToLua(ts, *v);
            }
            std::string shown = "nil";
            if (def.get_type() == sol::type::string) shown = def.as<std::string>();
            else if (def.get_type() == sol::type::number) {
                char b[32];
                std::snprintf(b, sizeof(b), "%g", def.as<double>());
                shown = b;
            } else if (def.get_type() == sol::type::boolean) {
                shown = def.as<bool>() ? "true" : "false";
            }
            FD_INFO("script", "param %s = %s (default)", name.c_str(), shown.c_str());
            return def;
        });
        // fd.dut: read accessors over the active DUT (writes go through the
        // generated dut_* commands) — lets a script reuse the open DUT
        // without re-prompting.
        {
            sol::table dut = lua.create_table();
            dut.set_function("is_open", [] { return DutSession::Instance().GetInfo().open; });
            dut.set_function("serial",  [] { return DutSession::Instance().GetInfo().serial; });
            dut.set_function("product", [] { return DutSession::Instance().GetInfo().product; });
            fd["dut"] = dut;
        }

        fd.set_function("positions", [&ctx](sol::this_state ts,
                                            const std::string& device) {
            sol::state_view sv(ts);
            sol::table t = sv.create_table();
            const auto it = ctx.Positions().find(device);
            if (it != ctx.Positions().end()) {
                for (const SavedPos& sp : it->second) t[sp.name] = sp.uid;
            }
            return t;
        });

        // fd.ctx: metatable-backed view of the run context (read AND write).
        {
            sol::table ctxT = lua.create_table();
            sol::table meta = lua.create_table();
            meta[sol::meta_function::index] =
                [&ctx](sol::this_state ts, sol::table, const std::string& key)
                    -> sol::object {
                const std::optional<CtxValue> v = ctx.CtxGet(key);
                if (!v) return sol::make_object(ts, sol::lua_nil);
                return CtxValueToLua(ts, *v);
            };
            meta[sol::meta_function::new_index] =
                [&ctx](sol::table, const std::string& key, sol::object v) {
                std::string convErr;
                const nlohmann::json j = LuaValueToJson(v, &convErr);
                if (!convErr.empty())
                    throw std::runtime_error("fd.ctx." + key + ": " + convErr);
                if (j.is_boolean())     ctx.CtxSet(key, j.get<bool>());
                else if (j.is_number()) ctx.CtxSet(key, j.get<double>());
                else                    ctx.CtxSet(key, j.get<std::string>());
            };
            ctxT[sol::metatable_key] = meta;
            fd["ctx"] = ctxT;
        }

        // print -> Logger; fd.logf = printf-style convenience.
        lua.script(R"(
            function print(...)
                local t = {}
                for i = 1, select('#', ...) do t[i] = tostring(select(i, ...)) end
                fd.log(table.concat(t, '\t'))
            end
            function fd.logf(fmt, ...) fd.log(string.format(fmt, ...)) end
        )");

        // ---- Run the program ---------------------------------------------
        const sol::protected_function_result r =
            lua.safe_script_file(path, sol::script_pass_on_error);
        if (!r.valid()) {
            const sol::error e = r;
            const std::string what = e.what();
            FD_WARN("script", "%s", what.c_str());
            firstErr = what.substr(0, what.find('\n'));
            if (firstErr.empty()) firstErr = "script error";
        }
    } catch (const std::exception& e) {
        FD_ERROR("script", "engine exception: %s", e.what());
        firstErr = e.what();
    } catch (...) {
        FD_ERROR("script", "non-standard engine exception");
        firstErr = "unknown error";
    }
    return firstErr;
}

// ---- Engine -----------------------------------------------------------------

const char* LuaEngine::StateName(State s)
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

LuaEngine::LuaEngine(Workflow& wf, HardwareManager& hw, const CommandRegistry& reg)
    : wf_(wf), hw_(hw), reg_(reg)
{
}

LuaEngine::~LuaEngine()
{
    RequestStop();
    if (worker_.joinable()) worker_.join();
}

bool LuaEngine::SafetyAborted() const
{
    return wf_.SafetyAlertActive() && wf_.SafetyAlert().critical;
}

bool LuaEngine::HookShouldStop() const
{
    return stopReq_.load() || SafetyAborted();
}

void LuaEngine::NoteCommand(const std::string& name)
{
    const std::scoped_lock lock(mu_);
    lastCommand_ = name;
}

// ---- Debugger (C3) ----------------------------------------------------------

void LuaEngine::SetBreakpoints(std::set<int> lines)
{
    const std::scoped_lock lock(mu_);
    breakpoints_ = std::move(lines);
}

void LuaEngine::Pause()
{
    pauseReq_.store(true);
}

void LuaEngine::Resume()
{
    {
        const std::scoped_lock lock(mu_);
        resumeGo_ = true;
    }
    cv_.notify_all();
}

void LuaEngine::StepLine()
{
    stepReq_.store(true);   // pause again at the very next line event
    Resume();
}

// Runs ON THE LUA WORKER from the line hook. Decides whether to break here
// (explicit pause, a step landing, or a breakpoint), captures the current
// function's locals, then blocks until Resume/Step/Stop releases it.
void LuaEngine::OnLineHook(lua_State* L, lua_Debug* ar)
{
    const int line = ar->currentline;
    curLine_.store(line);

    bool brk = pauseReq_.exchange(false);
    if (stepReq_.exchange(false)) brk = true;
    if (!brk) {
        const std::scoped_lock lock(mu_);
        brk = breakpoints_.count(line) != 0;
    }
    if (!brk || HookShouldStop()) return;

    // Locals of the current function (skip Lua-internal "(temporary)" slots).
    std::vector<std::pair<std::string, std::string>> locs;
    for (int i = 1;; ++i) {
        const char* name = lua_getlocal(L, ar, i);
        if (!name) break;
        if (name[0] != '(') {
            const char* sv = luaL_tolstring(L, -1, nullptr);   // pushes a string
            locs.emplace_back(name, sv ? sv : "?");
            lua_pop(L, 1);   // tolstring result
        }
        lua_pop(L, 1);       // the local's value
    }

    FD_INFO("script", "paused at line %d", line);
    {
        std::unique_lock lock(mu_);
        paused_   = true;
        locals_   = std::move(locs);
        resumeGo_ = false;
        // Sliced wait so Stop and safety aborts always get through.
        while (!resumeGo_ && !HookShouldStop()) {
            cv_.wait_for(lock, std::chrono::milliseconds(100));
        }
        paused_ = false;
        locals_.clear();
    }
}

bool LuaEngine::Start(const std::string& scriptPath, Env env, std::string* err)
{
    if (running_.load()) {
        if (err) *err = "a script is already running";
        return false;
    }
    if (wf_.PathActive()) {
        if (err) *err = "the Motion page path executor is busy - cancel it first";
        return false;
    }
    if (!std::ifstream(scriptPath).is_open()) {
        if (err) *err = "cannot open script '" + scriptPath + "'";
        return false;
    }
    if (worker_.joinable()) worker_.join();   // reap the previous finished run

    {
        const std::scoped_lock lock(mu_);
        state_       = State::Running;
        scriptName_  = std::filesystem::path(scriptPath).stem().string();
        lastCommand_ = "";
        error_       = "";
        prompt_      = "";
    }
    curLine_.store(0);
    stopReq_.store(false);
    pauseReq_.store(false);
    stepReq_.store(false);
    running_.store(true);
    FD_INFO("script", "run started: '%s'", scriptPath.c_str());
    worker_ = std::thread(&LuaEngine::ThreadMain, this, scriptPath, std::move(env));
    return true;
}

void LuaEngine::RequestStop()
{
    stopReq_.store(true);
    cv_.notify_all();
}

void LuaEngine::AnswerPrompt(bool ok, std::string text)
{
    {
        const std::scoped_lock lock(mu_);
        if (prompt_.empty()) return;
        prompt_.clear();
        promptText_     = std::move(text);
        promptAnswer_   = ok;
        promptAnswered_ = true;
    }
    cv_.notify_all();
}

void LuaEngine::AnswerAdjust(bool ok)
{
    {
        const std::scoped_lock lock(mu_);
        if (!adjustPending_) return;
        adjustPending_  = false;
        adjustAnswer_   = ok;
        adjustAnswered_ = true;
    }
    cv_.notify_all();
}

LuaEngine::Status LuaEngine::GetStatus() const
{
    const std::scoped_lock lock(mu_);
    Status s;
    s.state         = state_;
    s.scriptName    = scriptName_;
    s.currentLine   = curLine_.load();
    s.paused        = paused_;
    s.lastCommand   = lastCommand_;
    s.error              = error_;
    s.pendingPrompt      = prompt_;
    s.pendingPromptInput = promptWantsInput_;
    s.pendingAdjust      = adjustPending_;
    s.adjustReq          = adjustReq_;
    s.locals             = locals_;
    if (runCtx_) s.ctx = runCtx_->CtxSnapshot();
    return s;
}

bool LuaEngine::AwaitPromptInternal(const std::string& message, bool wantsInput,
                                    std::string* textOut)
{
    std::unique_lock lock(mu_);
    prompt_           = message;
    promptWantsInput_ = wantsInput;
    promptText_.clear();
    promptAnswered_ = false;
    promptAnswer_   = false;
    while (!promptAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    prompt_.clear();
    promptWantsInput_ = false;
    const bool ok = promptAnswered_ && promptAnswer_ && !stopReq_.load();
    if (ok && textOut) *textOut = promptText_;
    return ok;
}

bool LuaEngine::AwaitAdjustInternal(const ExecutionContext::AdjustRequest& req)
{
    std::unique_lock lock(mu_);
    adjustReq_      = req;
    adjustPending_  = true;
    adjustAnswered_ = false;
    adjustAnswer_   = false;
    while (!adjustAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    adjustPending_ = false;
    return adjustAnswered_ && adjustAnswer_ && !stopReq_.load();
}

void LuaEngine::ThreadMain(std::string path, Env env)
{
    // ctx BEFORE the Lua run: everything the bindings capture must outlive
    // the lua_State (created and destroyed inside RunLuaFile).
    ExecutionContext ctx(
        wf_, hw_, env.positions, env.maps,
        ExecutionContext::Hooks{
            [this] { return HookShouldStop(); },
            [this](const std::string& m) { return AwaitPromptInternal(m, false, nullptr); },
            [this](const std::string& m, std::string* out) {
                return AwaitPromptInternal(m, true, out);
            },
            [this](const ExecutionContext::AdjustRequest& r) {
                return AwaitAdjustInternal(r);
            },
        });
    for (const auto& [key, val] : env.initialCtx) ctx.CtxSet(key, val);
    {
        const std::scoped_lock lock(mu_);
        runCtx_ = &ctx;
    }

    const std::string firstErr = RunLuaFile(path, reg_, ctx, env.library, true, this);

    {
        const std::scoped_lock lock(mu_);
        runCtx_ = nullptr;
        if (stopReq_.load() || SafetyAborted()) {
            state_ = State::Stopped;
            error_ = SafetyAborted() ? "safety alert" : "stopped by operator";
        } else if (firstErr.empty()) {
            state_ = State::Completed;
            error_.clear();
        } else {
            state_ = State::Failed;
            error_ = firstErr;
        }
        FD_INFO("script", "run '%s' finished: %s%s%s", scriptName_.c_str(),
                StateName(state_), error_.empty() ? "" : " - ", error_.c_str());
        // Traceability: attribute this run to the open DUT (soft no-op if none).
        DutSession::Instance().RecordRun("script", scriptName_, StateName(state_));
    }
    curLine_.store(0);
    running_.store(false);
}

} // namespace firedog
