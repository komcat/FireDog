// LuaEngine.cpp
#include "command/LuaEngine.h"

#include "command/CommandRegistry.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <sol/sol.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
// The engine pointer travels in the lua_State's extra space; the hook fires
// every LINE (source highlight) and every N instructions (so a pure-Lua
// `while true do end` still honors Stop). Raising a Lua error from the hook
// is the sanctioned abort path (longjmp through interpreter C frames only).

namespace {

constexpr int kHookInstrCount = 200;

LuaEngine* EngineOf(lua_State* L)
{
    return *static_cast<LuaEngine**>(lua_getextraspace(L));
}

void LuaStopHook(lua_State* L, lua_Debug* ar)
{
    LuaEngine* e = EngineOf(L);
    if (!e) return;
    if (ar->event == LUA_HOOKLINE) e->HookLine(ar->currentline);
    if (e->HookShouldStop()) luaL_error(L, "stopped");
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

void LuaEngine::AnswerPrompt(bool ok)
{
    {
        const std::scoped_lock lock(mu_);
        if (prompt_.empty()) return;
        prompt_.clear();
        promptAnswer_   = ok;
        promptAnswered_ = true;
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
    s.lastCommand   = lastCommand_;
    s.error         = error_;
    s.pendingPrompt = prompt_;
    if (runCtx_) s.ctx = runCtx_->CtxSnapshot();
    return s;
}

bool LuaEngine::AwaitPromptInternal(const std::string& message)
{
    std::unique_lock lock(mu_);
    prompt_         = message;
    promptAnswered_ = false;
    promptAnswer_   = false;
    while (!promptAnswered_ && !stopReq_.load()) {
        cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    prompt_.clear();
    return promptAnswered_ && promptAnswer_ && !stopReq_.load();
}

void LuaEngine::ThreadMain(std::string path, Env env)
{
    // ctx BEFORE lua: the Lua state (whose bindings capture &ctx) must be
    // destroyed first when this frame unwinds.
    ExecutionContext ctx(
        wf_, hw_, env.positions, env.maps,
        ExecutionContext::Hooks{
            [this] { return HookShouldStop(); },
            [this](const std::string& m) { return AwaitPromptInternal(m); },
        });
    {
        const std::scoped_lock lock(mu_);
        runCtx_ = &ctx;
    }

    bool ok = false;
    std::string firstErr;
    try {
        sol::state lua;
        lua.open_libraries(sol::lib::base, sol::lib::math, sol::lib::string,
                           sol::lib::table);
        *static_cast<LuaEngine**>(lua_getextraspace(lua.lua_state())) = this;
        lua_sethook(lua.lua_state(), &LuaStopHook,
                    LUA_MASKCOUNT | LUA_MASKLINE, kHookInstrCount);

        // ---- fd: one generated function per registered command ----------
        sol::table fd = lua.create_named_table("fd");
        for (const CommandSpec* c : reg_.List()) {
            const CommandSpec* spec = c;
            fd.set_function(spec->name,
                            [this, spec, &ctx](sol::optional<sol::table> args) {
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
                NoteCommand(spec->name);
                const CommandResult r = spec->execute(ctx, ParamValues(std::move(merged)));
                if (!r.ok) throw std::runtime_error(spec->name + ": " + r.error);
            });
        }

        // ---- Hand-written conveniences ----------------------------------
        fd.set_function("read", [](sol::this_state ts, const std::string& topic)
                                    -> sol::object {
            DataStream& s = DataBus::Instance().Stream(topic);
            if (s.Count() == 0) return sol::make_object(ts, sol::lua_nil);
            return sol::make_object(ts, s.LatestValue());
        });
        fd.set_function("sleep", [&ctx](int ms) {
            if (!ctx.Sleep(ms)) throw std::runtime_error("stopped");
        });
        fd.set_function("log", [](const std::string& msg) {
            FD_INFO("script", "%s", msg.c_str());
        });
        fd.set_function("prompt", [&ctx](const std::string& msg) {
            return ctx.AwaitPrompt(msg);
        });
        fd.set_function("clock", [] {
            using clock = std::chrono::steady_clock;
            return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
        });
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
        if (r.valid()) {
            ok = true;
        } else {
            const sol::error e = r;
            const std::string what = e.what();
            FD_WARN("script", "%s", what.c_str());
            firstErr = what.substr(0, what.find('\n'));
        }
    } catch (const std::exception& e) {
        FD_ERROR("script", "engine exception: %s", e.what());
        firstErr = e.what();
    } catch (...) {
        FD_ERROR("script", "non-standard engine exception");
        firstErr = "unknown error";
    }

    {
        const std::scoped_lock lock(mu_);
        runCtx_ = nullptr;
        if (stopReq_.load() || SafetyAborted()) {
            state_ = State::Stopped;
            error_ = SafetyAborted() ? "safety alert" : "stopped by operator";
        } else if (ok) {
            state_ = State::Completed;
        } else {
            state_ = State::Failed;
            error_ = firstErr.empty() ? "script error" : firstErr;
        }
        FD_INFO("script", "run '%s' finished: %s%s%s", scriptName_.c_str(),
                StateName(state_), error_.empty() ? "" : " - ", error_.c_str());
    }
    curLine_.store(0);
    running_.store(false);
}

} // namespace firedog
