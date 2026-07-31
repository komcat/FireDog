// CommandSpec.h : layer 2.5 — the declaration of ONE scriptable machine action.
//
// Every action a recipe (and later a Lua program) can perform is declared once
// as a CommandSpec: a stable snake_case name, a typed parameter schema, and an
// execute function that drives Workflow (layer 2). The typed schema is the
// deliberate upgrade over bobcat_engine's map<string,string> params: it buys
// load-time validation now, and auto-generated editor forms + Lua argument
// checking later. See docs/recipe-core-design.md.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

class ExecutionContext;

// Parameter types. The semantic string types (DeviceMotion, PositionUid, ...)
// are plain strings on the wire; their point is validation (resolved against
// live config at load/run start) and, later, the right editor picker widget.
enum class ParamType {
    Bool,
    Int,
    Double,
    String,
    Value,          // free-typed: bool, number, or string (ctx values)
    DeviceMotion,   // motion fleet device name
    DeviceHexapod,  // PI hexapod only (the hex-* devices) — scan algorithms
                    // rely on PI's verified settle (IsSettled); ACS reports
                    // settled immediately, so it must not be offered
    DeviceIo,       // IO fleet device name
    Slide,          // pneumatic slide name
    PositionUid,    // SavedPositions uid (resolved against the step's "device")
    MapName,        // MotionMap name (resolved against the step's "device")
    PinName,        // IO pin name (resolved against the step's "device")
    TopicName,      // DataBus topic (no validation — topics create on demand)
    ScriptFile,     // .lua path (editor: picker over the scripts dir; validated to exist)
    Enum,           // one of `options`
};

const char* ParamTypeName(ParamType t);

// Direction hint for PinName params: which pin list the editor offers and
// validation resolves against (Any = either).
enum class PinDir { Any, Input, Output };

struct ParamSpec {
    std::string    name;
    ParamType      type = ParamType::String;
    bool           required = true;
    nlohmann::json defaultValue;               // substituted when !required
    double         minVal = 0.0, maxVal = 0.0; // numeric bounds; active when minVal < maxVal
    std::vector<std::string> options;          // Enum choices
    std::string    help;                       // one line (editor tooltip)
    PinDir         pinDir = PinDir::Any;       // PinName params only
};

struct CommandResult {
    bool        ok = true;
    std::string error;

    static CommandResult Success() { return {}; }
    static CommandResult Fail(std::string e) { return {false, std::move(e)}; }
};

// Step params after validation: defaults merged, types checked — command
// implementations read without guarding.
class ParamValues {
public:
    ParamValues() = default;
    explicit ParamValues(nlohmann::json merged) : j_(std::move(merged)) {}

    bool        GetBool(const char* name) const;
    int         GetInt(const char* name) const;
    double      GetDouble(const char* name) const;
    std::string GetString(const char* name) const;
    bool        Has(const char* name) const;
    const nlohmann::json& Raw() const { return j_; }

private:
    nlohmann::json j_ = nlohmann::json::object();
};

struct CommandSpec {
    // Which direct-SDK instrument the command needs (checked Connected at run
    // start). Fleet devices are covered by DeviceMotion/DeviceIo/Slide params.
    enum class Needs { None, Camera, Keithley, Cld, Psu };

    std::string name;        // stable id, snake_case: "move_to_position"
    std::string category;    // "Motion" | "IO" | "Instruments" | "System" ...
    std::string description; // one line
    std::vector<ParamSpec> params;
    Needs needs = Needs::None;

    // Runs on the RecipeEngine worker thread — NEVER the UI thread. Blocking
    // is expected; every wait MUST go through ExecutionContext (Sleep /
    // WaitUntil) so a stop request interrupts it within one poll interval.
    std::function<CommandResult(ExecutionContext&, const ParamValues&)> execute;
};

// "" on success, else a human-readable error naming the parameter. Unknown
// keys are rejected (catches typos in hand-written JSON). On success `merged`
// (if given) receives the params with defaults substituted.
std::string ValidateParams(const CommandSpec& spec, const nlohmann::json& params,
                           nlohmann::json* merged = nullptr);

// Shared mechanical-settle delays, appended to every command that physically
// moves something (motion, IO outputs, pneumatics). Both default 0 = off:
// pre_delay_ms runs BEFORE the action, post_delay_ms AFTER it completed
// successfully. Honored centrally by ExecuteWithDelays — command exec fns
// never read them.
inline ParamSpec PreDelayParam()
{
    return {"pre_delay_ms", ParamType::Int, false, 0, 0, 60000, {},
            "delay BEFORE the action (ms; 0 = none)"};
}
inline ParamSpec PostDelayParam()
{
    return {"post_delay_ms", ParamType::Int, false, 0, 0, 60000, {},
            "delay AFTER the action completes (ms; 0 = none)"};
}

// The one choke point both executors (StepRunner + Lua bindings) call instead
// of spec.execute directly: sleeps pre_delay_ms, runs the command, sleeps
// post_delay_ms on success. Commands without the params merge them as absent
// (GetInt = 0) — zero overhead. Sleeps are stop-interruptible.
CommandResult ExecuteWithDelays(const CommandSpec& spec, ExecutionContext& ctx,
                                const ParamValues& pv);

} // namespace firedog
