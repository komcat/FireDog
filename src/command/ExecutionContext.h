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
// thread — or, inside a `parallel` block, on one of its lane threads, which
// the block joins before it returns (StepRunner::ExecParallel). Those lanes
// share this one instance, so what they touch here must stay lock-free-safe
// (ShouldStop is a read of the engine's atomics, the waits are stateless) or
// guarded: CtxSnapshot may be called from the UI thread (status display) and
// the ctx store is mutex-guarded for exactly that reason, which also covers
// two lanes. RecordPosition and the prompts are NOT covered and no
// parallel-capable command calls them (teach is refused in a block).
#pragma once

#include "core/AlignFrame.h"
#include "core/GripProfile.h"
#include "core/WaffleTray.h"
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
// The procedures the app launches from its own buttons and keeps in step with
// the command family: alignment_procedure.lua, alignment_repeat.lua,
// needle_calibration.lua and the rest IsSystemScript names. They are STANDARD — every
// product runs the shipped copy, and a file of the same name in a product's
// scripts folder is ignored (with a warning).
//
// A forked copy is how a product silently loses a step the app added later: the
// day a standard script grows a call, every product carrying its own copy goes
// on skipping it, and the symptom is a step that silently never happens. Product-specific scripts — a scan, a
// bring-up sequence — keep the override; these are not that.
//
// The repeatability sweep is here for a third reason: it is the source of a
// CAPABILITY figure. A forked copy that quietly changed what one iteration does
// would still produce a Cpk, and a number nobody can reproduce is worse than no
// number at all.
//
// The needle calibration is here for a slightly different reason worth keeping
// straight: it measures the MACHINE, so there is no such thing as one product's
// version of the answer.
//
// dispense_procedure.lua is here for grip_measure's reason (below). How much
// glue goes where IS a product's business - but all of it is now a parameter
// of the dispense_run step (shot, dwells, gap, trim, speeds, confirmations,
// set in the Dispense helper), so a product never needs its own copy of the
// script, and a copy it once took would keep the motion it was copied at: a
// release that changed the retreat order reached no product that had forked.
//
// The two device-registration procedures are on the list for the needle's
// reason. Where a gripper is, is a fact about the machine; five products with
// five answers and no way to tell which is right is exactly what a forkable
// copy would produce. They also carry the quality gates a fit is refused on —
// or rather they call the command that does, which is why those gates live in
// core/DeviceReg.cpp and not in the script.
//
// grip_measure.lua is on the list for a fourth reason, and it is the one that
// distinguishes this list from a policy about machine state. WHAT it measures
// is per-product to its bones - the lens, the gripper, the tolerance all live
// in the product's own profile - but HOW it measures is not: pose the tool,
// look at the tip, mark, subtract. A product that forked it would keep whatever
// order of legs it was copied at, and the failure would look like a bad grip
// rather than like a missing step. The product's business is entirely in the
// record the script reads.
bool IsSystemScript(const std::string& pathOrFile);

// "config/scripts/<file>" for a system script, "" for anything else. The one
// place the shipped location is spelled, so a UI launcher and ResolveScript
// cannot disagree about which file "the procedure" means. Passed through
// LocateShippedScript, so it may name the machine root's copy instead.
std::string SystemScriptPath(const std::string& pathOrFile);

// The script folders other than a product's. The SHIPPED one is the exe's
// config/scripts: the build refreshes it and a release replaces it, so nothing
// written there survives. The MACHINE one (<machine root>/config/scripts,
// core/Paths.h) holds what was created or saved on this PC; a file there beats
// the shipped file of the same name, exactly as a product's copy beats both.
std::string MachineScriptsDir();
bool        IsShippedScript(const std::string& path);   // sits in the exe's config/scripts

// A shipped spelling ("config/scripts/<file>") whose file is MISSING beside the
// exe resolves to "<MachineScriptsDir()>/<file>" when that exists, so an install
// whose exe folder holds only the exe keeps its scripts under the machine root
// (D:/ProgramData/FireDog, else the C:/ProgramData fallback). The exe's copy
// still wins while it exists: it is the one built with these commands. Any
// other path comes back unchanged. LuaEngine::Start and RunLuaFile both apply
// it, so a launcher that spells the shipped path needs nothing else.
std::string LocateShippedScript(const std::string& path);

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
        // A TEACH prompt (a mover's `teach` param, or a teach-mode run): the
        // window offers Save / Skip instead of OK / Cancel, and shows the
        // position it would write beside the live pose. The answer travels the
        // same bool channel: true = save, false = keep the saved value.
        bool        teach = false;
        std::string positionName;              // display only
        std::map<std::string, double> target;  // the saved values, display only
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

    // The band E2 is expected to land in, centred on E1 in MACHINE mm — the
    // procedure's guard band, projected onto the live image so the operator can
    // move the crosshair until the mark is inside it. Empty on every other pick.
    //
    // It is here rather than in the UI because only the worker knows E1: it was
    // captured at the OTHER station, minutes and one move ago, and the window
    // sees nothing but the current field of view. The UI projects it exactly as
    // it projects a PickHint.
    struct PickBand {
        bool   have = false;
        double cx = 0.0, cy = 0.0;         // E1, machine mm
        double spanRef = 0.0, spanTol = 0.0;
        double thetaRef = 0.0, thetaTol = 0.0;   // degrees
    };

    // A predicted point the pick is asked to CHECK rather than aim at: the
    // window shows the live residual against it every frame - the crosshair
    // against x/y, the DEVICE's live Z against z - so an operator focusing or
    // jogging by hand watches the number close before accepting, instead of
    // learning what it was from the log after the run has already failed.
    //
    // Only the registration check sets it, because it is the only pick whose
    // whole output IS a distance from a predicted point. A hint says where to
    // look; this says how far off you still are, which is a different question.
    struct PickTarget {
        bool        have = false;
        double      x = 0.0, y = 0.0, z = 0.0;   // reference-device axis mm
        double      tolMm = 0.0;                 // 0 = show the number, no verdict
        std::string device;                      // whose live Z the readout reads
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
        // Draw the two-corner step diagrams above the buttons. TRUE only for a
        // placement's pair of object marks, which is what they picture. A label
        // alone used to imply them, and then the first caller that was neither
        // an E point nor an object mark — the needle calibration, marking one
        // imprint — got "point 1 of 2 on the object" iconography over a picture
        // of a glue dot. What the window asks for and what it illustrates have
        // to be set by the same caller.
        bool stepIcons = false;
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
        PickBand    band;
        PickTarget  target;
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
        // OK / Cancel prompt. autoOkMs > 0 answers OK by itself when that
        // long passes unanswered; 0 waits for the operator. null => fails.
        std::function<bool(const std::string&, int autoOkMs)> awaitPrompt;
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
                     const std::vector<GripProfile>& grips,
                     const std::vector<WaffleTray>& trays,
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
    // a consumed mark dropped here has to be re-taken rather than re-deriving a
    // correction that has already been applied. Missing keys are not an error.
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
    const std::vector<GripProfile>& Grips() const { return grips_; }
    const std::vector<WaffleTray>&  Trays() const { return trays_; }
    // Where the models' taught template patches live, for this run's document.
    const std::string& VisionDir() const { return visionDir_; }
    // The scripts directory this run resolves against — the product's when it
    // has one with content, else the machine's (whatever the session actually
    // scanned).
    const std::string& ScriptsDir() const { return scriptsDir_; }

    // A product's own copy of a script wins over the shipped one, and so does
    // the machine's. Given a default like "config/scripts/raster_scan.lua",
    // return "<ScriptsDir()>/raster_scan.lua" when that file exists, else
    // "<MachineScriptsDir()>/raster_scan.lua" when THAT exists, else the path
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
    // Gripped-object profile by uid, else by name. Same rule again.
    const GripProfile* FindGrip(const std::string& uidOrName) const;
    // Waffle tray by uid, else by name. Same rule again.
    const WaffleTray* FindTray(const std::string& uidOrName) const;

    // Teach: record `vals` as the device's saved position `name` — update by
    // name (uid kept stable) or create. Mutates the RUN's copy, so later steps
    // in this run move to the fresh values, and queues the position on Workflow
    // for the UI thread to merge into the app set and persist.
    void RecordPosition(const std::string& device, const std::string& name,
                        const std::map<std::string, double>& vals);
    // Teach INTO an existing position by uid (the mover's `teach` param and
    // teach-mode runs). Never creates: a uid the run's copy does not hold means
    // the product changed under the run, and a fresh row under an old name
    // would be a second answer to where that position is. Same run-copy
    // mutation and Workflow hand-off as RecordPosition. False if unknown.
    bool RecordPositionByUid(const std::string& device, const std::string& uid,
                             const std::map<std::string, double>& vals);

    // ---- Operator prompts (block; false = cancelled or stopped) -------------
    // autoOkMs > 0: the prompt answers OK by itself after that long, so an
    // operator gets a glance and a Cancel, not a stop. 0 waits for them.
    bool AwaitPrompt(const std::string& message, int autoOkMs = 0);
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
    const std::vector<GripProfile>& grips_;          // ditto
    const std::vector<WaffleTray>&  trays_;          // ditto
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

// A numeric ctx value, with *found telling a missing or non-numeric key apart
// from a genuine 0.0. Shared by the align, grip and tray commands — every
// "<prefix>_<field>" read goes through this one so a string where a number
// was expected fails the same way everywhere.
double CtxNumber(const ExecutionContext& ctx, const std::string& key, bool* found);

} // namespace firedog
