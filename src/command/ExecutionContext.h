// ExecutionContext.h : layer 2.5 — per-run services handed to every command.
//
// One instance per recipe run, created by RecipeEngine on its worker thread.
// Commands get: cooperative cancellation (the ONLY stop mechanism they see),
// interruptible waits (every wait has a timeout — no unbounded blocking),
// a typed string-keyed ctx store (inter-step data, bobcat's SequenceContext),
// logging, and read access to the run's resolve environment (saved positions +
// motion maps, copied at Start so they are stable for the whole run).
//
// Threading: ShouldStop/Sleep/WaitUntil and every command run on the worker
// thread; CtxSnapshot may be called from the UI thread (status display) — the
// ctx store is mutex-guarded for exactly that reason.
#pragma once

#include "core/MotionMap.h"
#include "core/SavedPositions.h"

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

class Workflow;
class HardwareManager;

using CtxValue = std::variant<bool, double, std::string>;

std::string CtxValueToString(const CtxValue& v);

class ExecutionContext {
public:
    // Hooks installed by RecipeEngine (the engine owns the stop flag and the
    // prompt handshake with the UI).
    struct Hooks {
        std::function<bool()> shouldStop;                     // required
        std::function<bool(const std::string&)> awaitPrompt;  // null => prompt fails
    };

    ExecutionContext(Workflow& wf, HardwareManager& hw,
                     const SavedPosMap& positions, const MotionMapSet& maps,
                     Hooks hooks);
    ExecutionContext(const ExecutionContext&) = delete;
    ExecutionContext& operator=(const ExecutionContext&) = delete;

    // ---- Cooperative cancellation + interruptible waits ---------------------
    bool ShouldStop() const;
    // Sleep in small slices; false if a stop request interrupted it.
    bool Sleep(int ms);
    enum class WaitResult { Ok, Timeout, Stopped };
    // Poll `pred` every pollMs until it returns true / timeoutMs elapses /
    // stop is requested. timeoutMs <= 0 is a validation error upstream; it is
    // clamped to one poll here so no wait is ever unbounded.
    WaitResult WaitUntil(const std::function<bool()>& pred, int timeoutMs,
                         int pollMs = 20);

    // ---- Run context (inter-step data) --------------------------------------
    void                    CtxSet(const std::string& key, CtxValue v);
    std::optional<CtxValue> CtxGet(const std::string& key) const;
    std::vector<std::pair<std::string, CtxValue>> CtxSnapshot() const;
    // Replace "{key}" occurrences with ctx values (unknown keys left as-is).
    std::string Substitute(const std::string& text) const;

    // ---- Layer access (commands issue every action through Workflow) --------
    Workflow&        Wf() { return wf_; }
    HardwareManager& Hw() { return hw_; }   // read-side only by convention
    const SavedPosMap&  Positions() const { return positions_; }
    const MotionMapSet& Maps() const { return maps_; }

    // ---- Name resolution helpers (fleet index lookups; -1 if unknown) -------
    int MotionIndexByName(const std::string& name) const;
    int IoIndexByName(const std::string& name) const;
    int SlideIndexByName(const std::string& name) const;
    const SavedPos* FindPosition(const std::string& device, const std::string& uid) const;
    const MotionMap* FindMap(const std::string& device, const std::string& mapName) const;

    // ---- Operator prompt (blocks; false = cancelled or stopped) -------------
    bool AwaitPrompt(const std::string& message);

private:
    Workflow&           wf_;
    HardwareManager&    hw_;
    const SavedPosMap&  positions_;   // owned by the engine's run Env
    const MotionMapSet& maps_;        // (stable for the whole run)
    Hooks               hooks_;

    mutable std::mutex              ctxMu_;
    std::map<std::string, CtxValue> ctx_;
};

// Shared condition evaluation for `if` / `loop while` / `wait_until`:
//   { "key": "<ctx key>" }  OR  { "topic": "<databus topic>" }
//   "op":  > < >= <= == != is_true is_false exists not_exists
//   "value": number|string (required by the comparison ops)
// Returns nullopt + err on a malformed condition; a missing key/topic makes
// comparison ops evaluate false (so wait_until keeps waiting), while
// exists/not_exists answer the existence question directly.
std::optional<bool> EvalCondition(const nlohmann::json& cond, ExecutionContext& ctx,
                                  std::string* err = nullptr);

// "" if `op` is a valid condition operator, else an error message (used by
// recipe validation, which has no ExecutionContext yet).
std::string CheckConditionShape(const nlohmann::json& cond);

} // namespace firedog
