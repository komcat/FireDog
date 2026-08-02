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

#include "core/AlignFrame.h"
#include "vision/VisionModel.h"
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
    // Manual position adjustment (bobcat's ManualAdjust): the run pauses while
    // the UI shows a jog surface for `device`; the operator nudges the stage
    // and confirms. What the modal needs to draw, nothing more — the jog
    // itself is UI-issued through Workflow like the Jog window.
    struct AdjustRequest {
        std::string message;
        std::string device;            // motion fleet device name
        double      stepUm = 10.0;     // initial jog step (presets override)
        bool        enableX = true, enableY = true, enableZ = true;
    };

    // Crosshair pick (the Alignment Procedure's manual mode): the run pauses
    // while the UI shows the live camera image and the operator marks a feature.
    // Nothing moves — the answer is a point in the CURRENT field of view.
    struct PickRequest {
        std::string message;
        std::string alignName;         // procedure being taught (window title)
        int         station = 1;       // 1 => E1, 2 => E2
    };
    // The UI answers in camera pixels AND in millimetres: only the UI knows the
    // live pixel_to_mm calibration, so the conversion stays there and the worker
    // needs no camera-config accessor.
    struct PickResult {
        double px = 0.0, py = 0.0;     // absolute camera pixel
        double dxMM = 0.0, dyMM = 0.0; // offset from the image centre, mm
    };

    // Hooks installed by the engines (they own the stop flag and the prompt
    // handshakes with the UI).
    struct Hooks {
        std::function<bool()> shouldStop;                     // required
        std::function<bool(const std::string&)> awaitPrompt;  // null => prompt fails
        // Text-input prompt (operator types / barcode-scans); false = cancel.
        std::function<bool(const std::string&, std::string*)> awaitInput;
        // Manual jog-adjust prompt; false = cancel (or stop released it).
        std::function<bool(const AdjustRequest&)> awaitAdjust;
        // Crosshair pick prompt; false = cancel (or stop released it).
        std::function<bool(const PickRequest&, PickResult*)> awaitPick;
    };

    ExecutionContext(Workflow& wf, HardwareManager& hw,
                     SavedPosMap& positions, const MotionMapSet& maps,
                     const std::vector<AlignProc>& aligns,
                     const std::vector<VisionModel>& visionModels,
                     std::string visionDir, Hooks hooks);
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
    const std::vector<AlignProc>& Aligns() const { return aligns_; }
    const std::vector<VisionModel>& VisionModels() const { return visionModels_; }
    // Where the models' taught template patches live, for this run's document.
    const std::string& VisionDir() const { return visionDir_; }

    // ---- Name resolution helpers (fleet index lookups; -1 if unknown) -------
    int MotionIndexByName(const std::string& name) const;
    int IoIndexByName(const std::string& name) const;
    int SlideIndexByName(const std::string& name) const;
    const SavedPos* FindPosition(const std::string& device, const std::string& uid) const;
    const MotionMap* FindMap(const std::string& device, const std::string& mapName) const;
    // Alignment procedure by uid, else by name (see FindAlignProc).
    const AlignProc* FindAlign(const std::string& uidOrName) const;
    // Vision detection model by uid, else by name (see FindVisionModel).
    const VisionModel* FindVisionModel(const std::string& uidOrName) const;

    // Teach: record `vals` as the device's saved position `name` — update by
    // name (uid kept stable) or create. Mutates the RUN's copy, so later steps
    // in this run move to the fresh values, and queues the position on Workflow
    // for the UI thread to merge into the app set and persist.
    void RecordPosition(const std::string& device, const std::string& name,
                        const std::map<std::string, double>& vals);

    // ---- Operator prompts (block; false = cancelled or stopped) -------------
    bool AwaitPrompt(const std::string& message);
    bool AwaitInput(const std::string& message, std::string* textOut);
    bool AwaitAdjust(const AdjustRequest& req);
    bool AwaitPick(const PickRequest& req, PickResult* out);

private:
    Workflow&           wf_;
    HardwareManager&    hw_;
    SavedPosMap&        positions_;   // owned by the engine's run Env (worker-
                                      // thread only; RecordPosition may mutate)
    const MotionMapSet& maps_;        // stable for the whole run
    const std::vector<AlignProc>& aligns_;   // ditto
    const std::vector<VisionModel>& visionModels_;   // ditto
    std::string         visionDir_;                  // by value: it is a path,
                                                     // not a live document
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
