// CommandsSystem.cpp : system/data commands — delay, log, ctx ops, DataBus
// bridge (read_topic), the generic wait_until, the operator prompt, the
// Lua escape hatch (run_script), and the typed scan wrappers (xcan,
// raster_scan) that front the algorithm scripts with real param forms.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"   // RunLuaFile (run_script)
#include "core/DataBus.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"   // final-position capture (manual_adjust)

#include <cctype>

namespace firedog {

namespace {

CommandResult ExecLog(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string level = p.GetString("level");
    const std::string msg   = ctx.Substitute(p.GetString("message"));
    LogLevel lvl = LogLevel::Info;
    if (level == "trace") lvl = LogLevel::Trace;
    else if (level == "debug") lvl = LogLevel::Debug;
    else if (level == "warn") lvl = LogLevel::Warn;
    else if (level == "error") lvl = LogLevel::Error;
    FD_LOG(lvl, "recipe", "%s", msg.c_str());
    return CommandResult::Success();
}

CommandResult ExecCtxSet(ExecutionContext& ctx, const ParamValues& p)
{
    const nlohmann::json& v = p.Raw()["value"];
    if (v.is_boolean())     ctx.CtxSet(p.GetString("key"), v.get<bool>());
    else if (v.is_number()) ctx.CtxSet(p.GetString("key"), v.get<double>());
    else                    ctx.CtxSet(p.GetString("key"), v.get<std::string>());
    return CommandResult::Success();
}

CommandResult ExecReadTopic(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string topic = p.GetString("topic");
    DataStream& s = DataBus::Instance().Stream(topic);
    if (s.Count() == 0)
        return CommandResult::Fail("no data on topic '" + topic + "'");
    ctx.CtxSet(p.GetString("ctx"), s.LatestValue());
    return CommandResult::Success();
}

CommandResult ExecWaitUntil(ExecutionContext& ctx, const ParamValues& p)
{
    // Rebuild the condition object from the flat params (they were validated
    // as a condition shape at recipe load).
    nlohmann::json cond = nlohmann::json::object();
    if (!p.GetString("topic").empty()) cond["topic"] = p.GetString("topic");
    if (!p.GetString("key").empty())   cond["key"]   = p.GetString("key");
    cond["op"] = p.GetString("op");
    if (p.Has("value")) cond["value"] = p.Raw()["value"];

    std::string condErr;
    const auto r = ctx.WaitUntil(
        [&] {
            const std::optional<bool> v = EvalCondition(cond, ctx, &condErr);
            return v.has_value() && *v;
        },
        p.GetInt("timeout_ms"), p.GetInt("poll_ms"));
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    if (r == ExecutionContext::WaitResult::Stopped) return CommandResult::Fail("stopped");
    return CommandResult::Fail(condErr.empty() ? "timeout waiting for condition"
                                               : "condition error: " + condErr);
}

CommandResult ExecPrompt(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string msg = ctx.Substitute(p.GetString("message"));
    return ctx.AwaitPrompt(msg)
               ? CommandResult::Success()
               : CommandResult::Fail("cancelled by operator");
}

CommandResult ExecPromptInput(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string msg = ctx.Substitute(p.GetString("message"));
    std::string text;
    if (!ctx.AwaitInput(msg, &text))
        return CommandResult::Fail("cancelled by operator");
    ctx.CtxSet(p.GetString("ctx"), text);
    return CommandResult::Success();
}

CommandResult ExecManualAdjust(ExecutionContext& ctx, const ParamValues& p)
{
    ExecutionContext::AdjustRequest req;
    req.message = ctx.Substitute(p.GetString("message"));
    req.device  = p.GetString("device");
    req.stepUm  = p.GetDouble("step_um");
    req.enableX = p.GetBool("enable_x");
    req.enableY = p.GetBool("enable_y");
    req.enableZ = p.GetBool("enable_z");
    const int devIdx = ctx.MotionIndexByName(req.device);
    if (devIdx < 0)
        return CommandResult::Fail("unknown motion device '" + req.device + "'");
    if (!ctx.AwaitAdjust(req))
        return CommandResult::Fail("cancelled by operator");
    // The confirmed position, into ctx as <prefix>_<axis> (lowercase axis, so
    // a hexapod yields adj_x/adj_y/adj_z/adj_u/... — same style as align_*).
    const std::string prefix = p.GetString("ctx_prefix");
    if (!prefix.empty()) {
        IMotionController* m =
            ctx.Hw().MotionDevice(static_cast<std::size_t>(devIdx));
        if (m) {
            for (const auto& [axis, val] : m->Positions()) {
                std::string key = prefix + "_";
                for (const char c : axis)
                    key += static_cast<char>(
                        std::tolower(static_cast<unsigned char>(c)));
                ctx.CtxSet(key, val);
            }
        }
    }
    return CommandResult::Success();
}

} // namespace

void RegisterSystemCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;
    (void)hw;
    using PT = ParamType;

    reg.Register({"delay", "System", "Wait a fixed duration",
                  {
                      {"ms", PT::Int, true, {}, 1, 3600000, {}, "duration (ms)"},
                  },
                  CommandSpec::Needs::None,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return c.Sleep(p.GetInt("ms"))
                                 ? CommandResult::Success()
                                 : CommandResult::Fail("stopped");
                  }});

    reg.Register({"log", "System", "Write a log line ({key} substitutes ctx values)",
                  {
                      {"level", PT::Enum, false, "info", 0, 0,
                       {"trace", "debug", "info", "warn", "error"}, "log level"},
                      {"message", PT::String, true, {}, 0, 0, {}, "message text"},
                  },
                  CommandSpec::Needs::None, ExecLog});

    reg.Register({"ctx_set", "System", "Set a run-context value",
                  {
                      {"key", PT::String, true, {}, 0, 0, {}, "ctx key"},
                      {"value", PT::Value, true, {}, 0, 0, {}, "bool, number, or string"},
                  },
                  CommandSpec::Needs::None, ExecCtxSet});

    reg.Register({"read_topic", "System", "Sample a DataBus topic into the run context",
                  {
                      {"topic", PT::TopicName, true, {}, 0, 0, {}, "databus topic"},
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  CommandSpec::Needs::None, ExecReadTopic});

    reg.Register({"wait_until", "System",
                  "Wait until a topic/ctx condition holds (op: > < >= <= == != ...)",
                  {
                      {"topic", PT::TopicName, false, "", 0, 0, {}, "databus topic (or use key)"},
                      {"key", PT::String, false, "", 0, 0, {}, "ctx key (or use topic)"},
                      {"op", PT::Enum, true, {}, 0, 0,
                       {">", "<", ">=", "<=", "==", "!=", "is_true", "is_false",
                        "exists", "not_exists"},
                       "comparison"},
                      {"value", PT::Value, false, 0.0, 0, 0, {}, "comparison value"},
                      {"timeout_ms", PT::Int, false, 10000, 10, 3600000, {}, "wait timeout"},
                      {"poll_ms", PT::Int, false, 50, 1, 5000, {}, "poll interval"},
                  },
                  CommandSpec::Needs::None, ExecWaitUntil});

    reg.Register({"prompt", "System",
                  "Show a message and wait for operator OK (Cancel fails the step)",
                  {
                      {"message", PT::String, true, {}, 0, 0, {}, "text shown to the operator"},
                  },
                  CommandSpec::Needs::None, ExecPrompt});

    reg.Register({"prompt_input", "System",
                  "Ask the operator for TEXT (keyboard or barcode scanner) into a ctx key",
                  {
                      {"message", PT::String, true, {}, 0, 0, {}, "text shown to the operator"},
                      {"ctx", PT::String, true, {}, 0, 0, {},
                       "destination ctx key (e.g. 'serial' for dut_open)"},
                  },
                  CommandSpec::Needs::None, ExecPromptInput});

    // bobcat's ManualAdjust: pause the run, let the operator jog the device
    // from a modal (step presets, live position), then OK to continue or
    // Cancel to fail the step. The jog itself is UI-issued via Workflow.
    reg.Register({"manual_adjust", "System",
                  "Pause while the operator jogs a motion device (OK continues, Cancel fails)",
                  {
                      {"message", PT::String, true, {}, 0, 0, {},
                       "instruction shown to the operator"},
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "motion device to jog"},
                      {"step_um", PT::Double, false, 10.0, 0.01, 1000.0, {},
                       "initial jog step (um); modal presets override"},
                      {"enable_x", PT::Bool, false, true, 0, 0, {}, "allow X jog"},
                      {"enable_y", PT::Bool, false, true, 0, 0, {}, "allow Y jog"},
                      {"enable_z", PT::Bool, false, true, 0, 0, {}, "allow Z jog"},
                      {"ctx_prefix", PT::String, false, "", 0, 0, {},
                       "if set, record the confirmed position as <prefix>_<axis>"},
                  },
                  CommandSpec::Needs::None, ExecManualAdjust});

    // The Lua escape hatch (bobcat's proven "Run Script" pattern): execute a
    // .lua file inline on the recipe worker with the same fd bindings and the
    // SAME run context (ctx values shared both ways). The sub-run cannot
    // start recipes (depth cap 1 — no script->recipe->script re-entrancy).
    // The registry pointer is stable: App owns it for the process lifetime.
    CommandRegistry* rp = &reg;
    reg.Register({"run_script", "System",
                  "Run a .lua program file inline (shared ctx; cannot start recipes)",
                  {
                      {"file", PT::ScriptFile, true, {}, 0, 0, {},
                       "path relative to the exe, e.g. config/scripts/foo.lua"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      const std::string err =
                          RunLuaFile(p.GetString("file"), *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});

    // Typed scan wrappers: same execution path as run_script, but the step
    // form carries the parameters — they are injected as the script's ctx
    // keys (the ctx_set convention), so any OTHER align_*/raster_* key a
    // recipe ctx_set beforehand still reaches fd.param untouched. Results
    // come back in ctx (align_* / raster_*) for dut_record / conditionals.
    reg.Register({"xcan", "Motion",
                  "Hexapod align scan (hexapod_align.lua): coarse-to-fine hill climb",
                  {
                      {"device", PT::DeviceHexapod, true, {}, 0, 0, {}, "hexapod to scan"},
                      {"topic", PT::TopicName, false, "keithley/ch1", 0, 0, {},
                       "feedback signal"},
                      {"steps_um", PT::String, false, "10,5,2,1", 0, 0, {},
                       "coarse -> fine step ladder (um); adaptive mode: the zone steps, far -> near"},
                      {"exit_target_ua", PT::Double, false, 0.0, 0.0, 1e9, {},
                       "stop-and-stay when feedback reaches this (uA; 0 = full ladder)"},
                      {"adaptive", PT::Bool, false, false, 0, 0, {},
                       "signal-zoned ladder: [zone_low, zone_high] split into one log zone "
                       "per step; passes coarser than the current zone's step are skipped"},
                      {"zone_low_ua", PT::Double, false, 1.0, 0.0001, 1e9, {},
                       "adaptive only: lower end of the signal range (uA)"},
                      {"zone_high_ua", PT::Double, false, 400.0, 0.0001, 1e9, {},
                       "adaptive only: upper end of the signal range (uA)"},
                      {"settle_min_ms", PT::Int, false, 0, 0, 5000, {},
                       "extra mechanical-settle floor before each sample (ms)"},
                      {"velocity_mm_s", PT::Double, false, 1.0, 0.0, 100.0, {},
                       "scan velocity applied before starting (deterministic settle); 0 = keep current"},
                      {"trace", PT::Bool, false, false, 0, 0, {},
                       "one 'xcan,' CSV line per sample"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/hexapod_align.lua", 0, 0, {}, "script override"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("align_device", p.GetString("device"));
                      c.CtxSet("align_topic", p.GetString("topic"));
                      c.CtxSet("align_steps_um", p.GetString("steps_um"));
                      c.CtxSet("align_exit_threshold",
                               p.GetDouble("exit_target_ua") * 1e-6);
                      c.CtxSet("align_adaptive", p.GetBool("adaptive"));
                      c.CtxSet("align_zone_low", p.GetDouble("zone_low_ua") * 1e-6);
                      c.CtxSet("align_zone_high", p.GetDouble("zone_high_ua") * 1e-6);
                      c.CtxSet("align_settle_min_ms",
                               static_cast<double>(p.GetInt("settle_min_ms")));
                      c.CtxSet("align_velocity_mm_s", p.GetDouble("velocity_mm_s"));
                      c.CtxSet("align_trace", p.GetBool("trace"));
                      const std::string err =
                          RunLuaFile(p.GetString("file"), *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});

    reg.Register({"raster_scan", "Motion",
                  "Serpentine area scan (raster_scan.lua): first-light search",
                  {
                      {"device", PT::DeviceHexapod, true, {}, 0, 0, {}, "hexapod to scan"},
                      {"topic", PT::TopicName, false, "keithley/ch1", 0, 0, {},
                       "feedback signal"},
                      {"slow_span_um", PT::Double, false, 40.0, 1.0, 2000.0, {},
                       "slow-axis travel, one-sided from start (X- by default)"},
                      {"slow_step_um", PT::Double, false, 4.0, 0.1, 500.0, {},
                       "row pitch"},
                      {"fast_span_um", PT::Double, false, 40.0, 1.0, 2000.0, {},
                       "fast-axis width, centered on start (Y +/- half)"},
                      {"fast_step_um", PT::Double, false, 4.0, 0.1, 500.0, {},
                       "sample pitch along a row"},
                      {"exit_target_ua", PT::Double, false, 0.0, 0.0, 1e9, {},
                       "stop-and-stay when feedback reaches this (uA; 0 = full grid)"},
                      {"settle_min_ms", PT::Int, false, 0, 0, 5000, {},
                       "extra mechanical-settle floor before each sample (ms)"},
                      {"velocity_mm_s", PT::Double, false, 1.0, 0.0, 100.0, {},
                       "scan velocity applied before starting (deterministic settle); 0 = keep current"},
                      {"trace", PT::Bool, false, false, 0, 0, {},
                       "one 'rast,' CSV line per sample"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/raster_scan.lua", 0, 0, {}, "script override"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("raster_device", p.GetString("device"));
                      c.CtxSet("raster_topic", p.GetString("topic"));
                      c.CtxSet("raster_slow_span_um", p.GetDouble("slow_span_um"));
                      c.CtxSet("raster_slow_step_um", p.GetDouble("slow_step_um"));
                      c.CtxSet("raster_fast_span_um", p.GetDouble("fast_span_um"));
                      c.CtxSet("raster_fast_step_um", p.GetDouble("fast_step_um"));
                      c.CtxSet("raster_exit_threshold",
                               p.GetDouble("exit_target_ua") * 1e-6);
                      c.CtxSet("raster_settle_min_ms",
                               static_cast<double>(p.GetInt("settle_min_ms")));
                      c.CtxSet("raster_velocity_mm_s", p.GetDouble("velocity_mm_s"));
                      c.CtxSet("raster_trace", p.GetBool("trace"));
                      const std::string err =
                          RunLuaFile(p.GetString("file"), *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});
}

} // namespace firedog
