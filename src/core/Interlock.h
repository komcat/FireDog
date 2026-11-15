// Interlock.h : conditional, cross-device motion interlocks — the model and its
// evaluation, pure (no hardware, no UI). docs/interlock-design.md.
//
// A RULE is "WHEN all of these conditions hold, REFUSE these motions":
//
//   when   slide state  (any of the named slides - all slides when none named)
//          IO input     (device + pin name, ON or OFF)
//          axis value   (device + axis, > >= < <=, a number in AXIS space -
//                        what IMotionController::Positions() reports, so a
//                        gantry whose transform inverts Z compares raw Z)
//   blocks device axes  (empty = every axis; optionally one direction only)
//          slide extend (one slide, or any slide)
//
// Three properties every reader of this file must keep:
//
//  * FAIL CLOSED. A condition the machine cannot read (a disconnected device,
//    an IO cache not yet valid, a name that resolves to nothing, a slide whose
//    sensors show neither end, an axis on a device that is referencing) counts
//    as TRUE, with a reason. A typo makes a rule always active, never inert.
//  * TRUE AT EITHER END. An axis condition holds if the measured value, an
//    in-flight commanded target, or the target of the move being checked
//    satisfies it. Positions are cached (1 Hz on an idle controller) and a Z
//    still travelling to 60 reads 30; the commanded end closes that gap.
//  * PREDICTED. A move is checked against the state it would CREATE (its own
//    targets applied), so "Z to 60 and XY together" is refused by the rule that
//    blocks XY while Z > 50 - not allowed because Z happens to start low.
//
// The workflow layer builds an IlSnapshot from the live fleet and calls
// IlCheckIntent before every move (the gate) and IlFindViolations on a timer
// (the watch). This file decides; it never moves or stops anything.
#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

// ---- The rules ---------------------------------------------------------------

enum class IlCondKind { Slide, IoInput, Axis };
enum class IlSlideWant { Extended, NotRetracted, Retracted };
enum class IlOp { Gt, Ge, Lt, Le };

struct IlCond {
    IlCondKind kind = IlCondKind::Axis;
    // Slide: TRUE when ANY of `slides` (every slide when empty) is in `slideWant`.
    std::vector<std::string> slides;
    IlSlideWant slideWant = IlSlideWant::NotRetracted;
    // IoInput: `device` + `pin` (an INPUT pin name) reads `on`.
    // Axis:    `device` + `axis` `op` `value`.
    std::string device;
    std::string pin;
    bool        on = true;
    std::string axis;
    IlOp        op    = IlOp::Gt;
    double      value = 0.0;
};

enum class IlBlockKind { Axes, SlideExtend };

struct IlBlock {
    IlBlockKind kind = IlBlockKind::Axes;
    std::string device;              // motion device; for SlideExtend the slide ("" = any)
    std::vector<std::string> axes;   // empty = every axis of the device
    int dir = 0;                     // 0 any direction, +1 increasing only, -1 decreasing only
};

struct IlRule {
    std::string uid;
    std::string name;
    std::string note;
    bool        enabled = true;
    std::vector<IlCond>  when;       // AND; a rule with no condition never activates
    std::vector<IlBlock> blocks;
};

struct IlConfig {
    bool enabled = true;
    int  watchMs = 50;               // the mid-move watch period
    std::vector<IlRule> rules;
    // Set by the loader when the machine's file exists but cannot be read. A
    // config that cannot be read is a set of rules nobody knows: every move is
    // refused until it is fixed (or bypassed) - fail closed, like a condition.
    std::string loadError;
};

// ---- The live state the rules are judged on (built by Workflow) ----------------

enum class IlSlidePos { Extended, Retracted, Moving, Unknown };

struct IlMotionState {
    bool connected = false;
    std::vector<std::string>      axes;
    std::map<std::string, double> measured;    // axis space
    std::map<std::string, double> commanded;   // in-flight targets only
    // The last target SENT per axis, kept after arrival. A move that restates
    // it exactly is not motion on that axis: a saved position names every axis,
    // and the ones it leaves where they are would otherwise read as moving by
    // the servo's jitter against the measured value. Trusted only while the
    // axis still sits within 0.01 of it.
    std::map<std::string, double> lastCmd;
    // A reference or a controller buffer is running: every axis may move and
    // where it goes is unknown - its axis conditions read unknown (TRUE).
    bool allBusy = false;
};

struct IlIoPin {
    bool        known = false;   // false = unreadable; `why` says so
    bool        on    = false;
    std::string why;
};

struct IlSnapshot {
    std::map<std::string, IlMotionState> motion;   // by device name
    std::map<std::string, IlSlidePos>    slides;   // by slide name, the whole fleet
    std::map<std::string, IlIoPin>       io;       // by IlIoKey(device, pin)
};

std::string IlIoKey(const std::string& device, const std::string& pin);
// Every (device, pin) an IoInput condition of `cfg` reads - the snapshot
// builder resolves exactly these.
std::vector<std::pair<std::string, std::string>> IlIoPinsUsed(const IlConfig& cfg);

// ---- What is being asked -------------------------------------------------------

struct IlIntent {
    enum class Kind { Move, AllAxes, SlideExtend } kind = Kind::Move;
    std::string device;   // motion device (Move / AllAxes) or slide (SlideExtend)
    std::vector<std::pair<std::string, double>> targets;   // Move: absolute, axis space
    std::string what;     // "move", "jog", "reference", "buffer", "extend" - for the log
};

struct IlVerdict {
    bool        ok = true;
    std::string rule;     // the refusing rule's name
    std::string reason;   // one line: what was refused and why
};

// The gate. ok = the intent may go. A disabled config passes everything; a
// config with a loadError refuses everything.
IlVerdict IlCheckIntent(const IlConfig& cfg, const IlSnapshot& snap, const IlIntent& intent);

// ---- Status (UI) and violations (the watch) --------------------------------------

struct IlCondStatus {
    bool        truth   = false;
    bool        unknown = false;   // truth came from fail-closed, not a reading
    std::string text;              // "UV_Head extended", "Z 62.013 > 50.000"
};

struct IlRuleStatus {
    std::string uid;
    std::string name;
    bool        enabled = true;
    bool        active  = false;   // every condition true (and enabled)
    bool        unknown = false;   // at least one condition is fail-closed
    std::vector<IlCondStatus> conds;
};

std::vector<IlRuleStatus> IlEvaluateAll(const IlConfig& cfg, const IlSnapshot& snap);
// One condition on its own (the editor's live dot beside each row).
IlCondStatus IlEvaluateCond(const IlCond& c, const IlSnapshot& snap);

// An ACTIVE rule whose blocked axis is moving right now. The watch stops
// `device` for each one and latches a trip.
struct IlViolation {
    std::string rule;
    std::string device;
    std::string axis;     // "*" = all axes (a reference / buffer)
    std::string reason;
};
std::vector<IlViolation> IlFindViolations(const IlConfig& cfg, const IlSnapshot& snap);

// The jog window's greying: per axis of `device`, would a small step in each
// direction be refused right now, and by which rule. Advisory only - the gate
// is the authority.
struct IlDirBlock {
    bool        plus  = false;
    bool        minus = false;
    std::string rulePlus;
    std::string ruleMinus;
};
std::map<std::string, IlDirBlock> IlBlockedDirs(const IlConfig& cfg, const IlSnapshot& snap,
                                                const std::string& device);

// ---- Names, text, files ----------------------------------------------------------

const char* IlOpText(IlOp op);              // ">", ">=", "<", "<="
const char* IlSlideWantText(IlSlideWant w); // "extended", "not retracted", "retracted"
std::string IlCondText(const IlCond& c);    // "gantry-main Z > 50.000"
std::string IlBlockText(const IlBlock& b);  // "gantry-main X,Y (+ only)"

// <machine root>/config/interlocks.local.json, else the shipped
// config/interlocks.json, else an empty config. `pathUsed` gets the file read.
IlConfig    LoadInterlocks(std::string* pathUsed = nullptr);
// One file, by path - no machine root resolved (FireDogPor reads both sides
// with this). *found = the file exists. A malformed file comes back with
// loadError set, exactly as LoadInterlocks hands it to the gate.
IlConfig    LoadInterlocksFile(const std::string& path, bool* found = nullptr);
bool        SaveInterlocks(const IlConfig& cfg, std::string* err = nullptr);   // the local file
std::string InterlocksLocalPath();

} // namespace firedog
