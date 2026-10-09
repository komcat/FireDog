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
    IntChoice,      // an Int with named choices: `options` labels the consecutive
                    // values from minVal up. Still a NUMBER on the wire — the
                    // recipes that already hold 1 or 2 keep working, and the
                    // command still reads it with GetInt. Use it where a small
                    // integer is really an identity ("station 2" is E2, not two
                    // of anything) and a spinner makes the operator remember
                    // which number that was
    Double,
    String,
    Value,          // free-typed: bool, number, or string (ctx values)
    DeviceMotion,   // motion fleet device name
    DeviceHexapod,  // PI hexapod only (the hex-* devices) — scan algorithms
                    // rely on PI's verified settle (IsSettled); ACS reports
                    // settled immediately, so it must not be offered
    DeviceIo,       // IO fleet device name
    DevicePsu,      // PSU fleet device name; "" = the first configured PSU
                    // (so a step saved before there were several still runs)
    Slide,          // pneumatic slide name
    LightName,      // a light on the lighting controller: a channel group
                    // ("main_camera_light") or an ungrouped channel's name
    PositionUid,    // SavedPositions uid (resolved against the step's "device")
    AlignUid,       // Alignment-procedure uid (or name) in the product document
    NominalUid,     // Nominal uid (or name) WITHIN the step's "alignment" — the
                    // same scoping convention PositionUid has to "device", one
                    // level down: a nominal only means anything on its procedure
    VisionUid,      // Vision detection-model uid (or name); like AlignUid, NOT
                    // device-scoped — a model belongs to the camera, not a stage
    GripUid,        // Gripped-object profile uid (or name); like AlignUid, not
                    // device-scoped — the profile NAMES the gripper it belongs
                    // to, so scoping it to a step's "device" would be a second
                    // answer to which device is holding the object
    TrayUid,        // Waffle-tray uid (or name); not device-scoped for the same
                    // reason — the tray carries the device its corners were
                    // taught on
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
    // "" = drawn inline, like every parameter always was. Otherwise the editor
    // folds a RUN of consecutive params sharing this name under a collapsing
    // header — for a cluster that is meaningless until one of them is set (the
    // approach triple), and that would otherwise cost three permanent rows on
    // every move step's form. Last field on purpose: every existing positional
    // brace-init, PinDir included, keeps working.
    std::string    group;
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
    enum class Needs { None, Camera, Keithley, Cld, Psu, Light };

    std::string name;        // stable id, snake_case: "move_to_position"
    std::string category;    // "Motion" | "IO" | "Instruments" | "System" ...
    std::string description; // one line
    std::vector<ParamSpec> params;
    Needs needs = Needs::None;

    // Runs on the RecipeEngine worker thread — NEVER the UI thread. Blocking
    // is expected; every wait MUST go through ExecutionContext (Sleep /
    // WaitUntil) so a stop request interrupts it within one poll interval.
    std::function<CommandResult(ExecutionContext&, const ParamValues&)> execute;

    // The name of the param that says which ONE device this command drives
    // ("device" for the movers). Set = the command may be a child of a
    // `parallel` block, where every child runs at once on its own thread and
    // the block's children must all name DIFFERENT values of it — two lanes
    // on one controller would fight over its target. Empty (the default) =
    // sequential only. This is the whole opt-in, like TeachParam() is for
    // teach mode: a command that shares nothing but the ExecutionContext and
    // its own controller declares it; one that touches shared Workflow state
    // (the jog tracks, a prompt, the document) must not.
    std::string parallelBy;
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
