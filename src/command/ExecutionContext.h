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

// ---- System procedure scripts ----------------------------------------------
//
// The two procedures the app launches from its own buttons and keeps in step
// with the command family: alignment_procedure.lua and placement_procedure.lua.
// They are STANDARD — every product runs the shipped copy, and a file of the
// same name in a product's scripts folder is ignored (with a warning).
//
// A forked copy is how a product silently loses a step the app added later: the
// day placement_procedure.lua grew its place_view call, every product carrying
// its own copy would have gone on skipping it, and the symptom would have been
// a camera that simply did not move. Product-specific scripts — a scan, a
// bring-up sequence — keep the override; these two are not that.
bool IsSystemScript(const std::string& pathOrFile);

// "config/scripts/<file>" for a system script, "" for anything else. The one
// place the shipped location is spelled, so a UI launcher and ResolveScript
// cannot disagree about which file "the procedure" means.
std::string SystemScriptPath(const std::string& pathOrFile);

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
    // A point in MACHINE mm the pick window should show while the operator
    // marks: the nominal being aimed at, and whichever mark is already taken.
    // The worker computes them because only it has the live frame and the run
    // context; the UI only projects them through the current pose.
    //
    // Off-screen is the case that matters. A nominal 6 mm away is invisible in a
    // 3 mm field of view, and nothing else on screen distinguishes "the target
    // is off to the left" from "the target is where I am looking".
    struct PickHint {
        double      wx = 0.0, wy = 0.0;
        std::string label;
    };

    struct PickRequest {
        std::string message;
        std::string alignName;         // procedure being taught (window title)
        int         station = 1;       // 1 => E1, 2 => E2
        // What the operator is being asked to mark, when it is not an E point:
        // empty keeps the alignment wording ("mark E1"), set replaces it, so a
        // placement can say "point 1 of 2 on the lens" through the SAME window
        // and the same handshake. The window identity is unchanged either way.
        std::string label;
        std::vector<PickHint> hints;
        // The auto->manual FALLBACK, when the pick exists because detection
        // failed: which model was supposed to find this point, and why it did
        // not. Both empty on a station (or a mark) that was manual by design.
        //
        // They are here because the operator standing at the machine is rarely
        // the person who tuned the detector, and nothing else on the live image
        // says WHICH point of the feature the run wanted. A mark placed on the
        // wrong corner of the same feature is not a visible mistake — it is a
        // frame that is wrong by the size of the feature, and every move made
        // through it is wrong by the same amount. The UI resolves the uid to a
        // model and shows its taught patch with the reported point crossed on it.
        std::string detectorUid;
        std::string autoError;
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
                     std::string visionDir, std::string scriptsDir, Hooks hooks);
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
    // Remove a key. For a measurement a later step must NOT silently reuse:
    // place_move drops the object marks it just consumed, so a second solve
    // has to be given fresh ones rather than re-deriving a correction that has
    // already been applied. Missing keys are not an error.
    void                    CtxErase(const std::string& key);
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
    // The scripts directory this run resolves against — the product's when it
    // has one with content, else config/scripts (whatever the session actually
    // scanned).
    const std::string& ScriptsDir() const { return scriptsDir_; }

    // A product's own copy of a script wins over the shipped one. Given a
    // default like "config/scripts/raster_scan.lua", return
    // "<ScriptsDir()>/raster_scan.lua" when that file exists, else the path
    // unchanged. SYSTEM scripts are the exception — see IsSystemScript.
    //
    // This exists because a command that hard-codes the shipped path silently
    // ignores the copy an operator edited for THIS product, and a recipe step
    // running "the same" procedure must not quietly run a different file.
    std::string ResolveScript(const std::string& path) const;

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
    std::string         visionDir_;                  // by value: they are paths,
    std::string         scriptsDir_;                  // not live documents
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
