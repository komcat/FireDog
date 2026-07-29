// CommandsDut.cpp : DUT traceability commands — lifecycle + data recording
// into the active part's record (core/DutSession). Design: docs/dut-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "core/DutSession.h"

namespace firedog {

namespace {

CommandResult ExecDutOpen(ExecutionContext& ctx, const ParamValues& p)
{
    std::string serial = p.GetString("serial");
    if (serial.empty()) {
        // Fallback: the ctx key "serial" (set by a prompt_input step or a
        // script). Explicit failure otherwise — dut_open never prompts itself.
        const std::optional<CtxValue> v = ctx.CtxGet("serial");
        if (v && std::holds_alternative<std::string>(*v))
            serial = std::get<std::string>(*v);
        if (serial.empty())
            return CommandResult::Fail(
                "no serial: set the 'serial' param or fill ctx key 'serial' "
                "(e.g. with prompt_input)");
    }
    std::string err;
    return DutSession::Instance().Open(serial, p.GetBool("new"), &err)
               ? CommandResult::Success()
               : CommandResult::Fail(err);
}

CommandResult ExecDutClose(ExecutionContext&, const ParamValues& p)
{
    const std::string s = p.GetString("status");
    std::string err;
    return DutSession::Instance().Close(s == "none" ? "" : s, &err)
               ? CommandResult::Success()
               : CommandResult::Fail(err);
}

CommandResult ExecDutSet(ExecutionContext&, const ParamValues& p)
{
    std::string err;
    return DutSession::Instance().Insert(p.GetString("key"), p.Raw()["value"], &err)
               ? CommandResult::Success()
               : CommandResult::Fail(err);
}

nlohmann::json CtxToJson(const CtxValue& v)
{
    if (std::holds_alternative<bool>(v)) return std::get<bool>(v);
    if (std::holds_alternative<double>(v)) return std::get<double>(v);
    return std::get<std::string>(v);
}

CommandResult ExecDutRecord(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string key = p.GetString("ctx");
    const std::optional<CtxValue> v = ctx.CtxGet(key);
    if (!v) return CommandResult::Fail("ctx key '" + key + "' has no value");
    std::string err;
    return DutSession::Instance().Insert(p.GetString("key"), CtxToJson(*v), &err)
               ? CommandResult::Success()
               : CommandResult::Fail(err);
}

CommandResult ExecDutRecordAll(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string prefix = p.GetString("prefix");
    nlohmann::json obj = nlohmann::json::object();
    for (const auto& [key, val] : ctx.CtxSnapshot()) {
        if (!prefix.empty() && key.rfind(prefix, 0) != 0) continue;
        // Strip the prefix (and one separator) for the field name; types are
        // preserved exactly — no string-to-number guessing.
        std::string field = key.substr(prefix.size());
        if (!field.empty() && (field[0] == '_' || field[0] == '.')) field.erase(0, 1);
        if (field.empty()) field = key;
        obj[field] = CtxToJson(val);
    }
    if (obj.empty())
        return CommandResult::Fail(prefix.empty()
                                       ? "the run context is empty"
                                       : "no ctx values match prefix '" + prefix + "'");
    std::string entryKey = prefix.empty() ? "ctx" : prefix;
    while (!entryKey.empty() && (entryKey.back() == '_' || entryKey.back() == '.'))
        entryKey.pop_back();
    std::string err;
    return DutSession::Instance().Insert(entryKey, obj, &err)
               ? CommandResult::Success()
               : CommandResult::Fail(err);
}

} // namespace

void RegisterDutCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;
    (void)hw;
    using PT = ParamType;

    reg.Register({"dut_open", "DUT",
                  "Open the DUT record for a serial (load-or-create; auto-closes the previous)",
                  {
                      {"serial", PT::String, false, "", 0, 0, {},
                       "part serial; empty = use ctx key 'serial'"},
                      {"new", PT::Bool, false, false, 0, 0, {},
                       "true = start a FRESH record (destructive re-test)"},
                  },
                  CommandSpec::Needs::None, ExecDutOpen});

    reg.Register({"dut_close", "DUT", "Close the DUT record (optional pass/fail)",
                  {
                      {"status", PT::Enum, false, "none", 0, 0,
                       {"none", "pass", "fail"}, "final status"},
                  },
                  CommandSpec::Needs::None, ExecDutClose});

    reg.Register({"dut_set", "DUT", "Record a literal value into the DUT",
                  {
                      {"key", PT::String, true, {}, 0, 0, {}, "entry key"},
                      {"value", PT::Value, true, {}, 0, 0, {}, "bool, number, or string"},
                  },
                  CommandSpec::Needs::None, ExecDutSet});

    reg.Register({"dut_record", "DUT",
                  "Record a run-context value into the DUT (measure to ctx, record from ctx)",
                  {
                      {"key", PT::String, true, {}, 0, 0, {}, "DUT entry key"},
                      {"ctx", PT::String, true, {}, 0, 0, {}, "source ctx key"},
                  },
                  CommandSpec::Needs::None, ExecDutRecord});

    reg.Register({"dut_record_all", "DUT",
                  "Record all (prefix-filtered) ctx values as ONE object entry",
                  {
                      {"prefix", PT::String, false, "", 0, 0, {},
                       "ctx key prefix filter; empty = everything"},
                  },
                  CommandSpec::Needs::None, ExecDutRecordAll});
}

} // namespace firedog
