// CommandsSystem.cpp : system/data commands — delay, log, ctx ops, DataBus
// bridge (read_topic), the generic wait_until, and the operator prompt.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "core/DataBus.h"
#include "core/Log.h"

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
}

} // namespace firedog
