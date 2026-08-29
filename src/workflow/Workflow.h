// Workflow.h : layer 2 — orchestrates the hardware collaboration. It owns the
// startup sequence (connect + bias + poll) and, later, the alignment procedures.
// Device *objects* live in HardwareManager (layer 1); Workflow only drives them.
#pragma once

#include "core/AlignFrame.h"           // AlignResult (taught-frame hand-back queue)
#include "core/DeviceReg.h"            // DeviceReg (a gripper tip in gantry space)
#include "core/NeedleCal.h"            // NeedleCal (the dispense needle's lever arm)
#include "core/SavedPositions.h"       // SavedPos (teach hand-back queue)
#include "hardware/HardwareConfig.h"   // AxisTransform, HardwareConfig
#include "vision/HalconDetector.h"     // the cached detectors (PIMPL, no SDK leak)
#include "vision/VisionModel.h"
#include "vision/VisionResult.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "hardware/io/SafetyWatch.h"

namespace firedog {

class HardwareManager;
class ISmu;
class CLD101x;
class UDP3305S;

class Workflow {
public:
    explicit Workflow(HardwareManager& hw);
    ~Workflow();   // stops the safety watch before hw_'s devices can die

    // Bring the direct-SDK instruments online from config: async connect, set
    // the alignment bias, start the poll/publish. Call once from the UI at startup.
    void Start(const HardwareConfig& hw);

    void  Update(float dt);
    float LatestSample() const { return sample_; }
    const char* StatusText() const;

    // Move gantry-main (ACS) by a relative XY offset in mm, both axes together.
    // Called by the UI's click-to-move on the camera image. Returns false if no
    // ACS gantry is available. The UI never touches the device directly.
    bool MoveGantryRelativeMM(double dxMM, double dyMM);

    // Jog gantry-main (ACS) along GLOBAL Z by a signed step in mm. Backs the Aux
    // window's Z buttons; goes through the same transform as any other jog.
    // False if no ACS gantry is available (or Z is unmapped).
    bool JogGantryZ(double deltaMM);

    // ---- Jog (layer 2 owns the kinematics + command) ------------------------
    void SetTransforms(std::map<std::string, AxisTransform> t);
    bool SetJogVelocity(std::size_t devIdx, double mmPerSec);
    // Jog by `stepMM` along a GLOBAL direction (gx,gy,gz unit vector); the device
    // transform maps it to a signed local axis relative move.
    bool JogGlobal(std::size_t devIdx, float gx, float gy, float gz, double stepMM);
    // Jog by `step` about a GLOBAL rotation axis (gu,gv,gw = thetaX,thetaY,thetaZ
    // unit vector); the transform's rotation block maps it to a signed local
    // rotational axis (the device's axes beyond XYZ). False if unmapped.
    bool JogGlobalRot(std::size_t devIdx, float gu, float gv, float gw, double step);
    // Jog a LOCAL axis directly (e.g. rotational U/V/W) by a signed delta.
    bool JogLocalAxis(std::size_t devIdx, const std::string& axis, double deltaMM);
    // Map a GLOBAL translation delta (mm) onto a device's OWN axes through its
    // configured transform, keeping every component. Pure kinematics: nothing
    // moves, nothing is checked against limits.
    //
    // Not JogGlobal: that one collapses the direction to its single dominant
    // axis, which is right for a jog button and wrong for a 2-D correction —
    // half of a diagonal would be silently dropped. `out` gets one (axis, mm)
    // entry per mapped translation axis, ready to be added to a live pose.
    // False when the device does not exist.
    bool GlobalDeltaToAxes(std::size_t devIdx, double gxMM, double gyMM, double gzMM,
                           std::vector<std::pair<std::string, double>>* out) const;
    // Move a device BY a global translation delta (mm): GlobalDeltaToAxes maps
    // it onto the device's own axes, the result is added to its live pose, and
    // the whole thing goes out as ONE coordinated absolute move —
    // MoveGantryRelativeMM generalised to any device, which is exactly what a
    // placement correction on a hexapod is. Safety-blocked like every other
    // move. The CALLER owns the decision that the delta is sane: this is a
    // computed destination, so it belongs behind an arming flag.
    //
    // `issued` (optional) receives the ABSOLUTE per-axis targets actually
    // commanded, so a caller that has to wait for arrival waits on exactly what
    // was sent rather than recomputing it from a pose that has since moved.
    bool MoveRelativeGlobal(std::size_t devIdx, double gxMM, double gyMM, double gzMM,
                            std::vector<std::pair<std::string, double>>* issued = nullptr);
    // The first half of MoveRelativeGlobal on its own: the ABSOLUTE per-axis
    // targets a global delta works out to, without issuing anything. For a
    // caller that has to see the destination before it moves — a two-speed
    // approach has to split the distance, and it cannot split a move that has
    // already started. MoveRelativeGlobal is written in terms of this, so the
    // delta-to-targets arithmetic exists exactly once.
    bool GlobalDeltaTargets(std::size_t devIdx, double gxMM, double gyMM, double gzMM,
                            std::vector<std::pair<std::string, double>>* targets) const;
    // Stop all motion on one device (jog panel Stop button).
    bool StopMotion(std::size_t devIdx);

    // Move to an absolute per-axis target set (a saved/taught position) in one
    // coordinated command. Called by the saved-positions Move button.
    bool MoveToPosition(std::size_t devIdx,
                        const std::vector<std::pair<std::string, double>>& targets);

    // ---- Mapped-path execution (Motion page "Mapped Path" tab) --------------
    // A path is a sequence of waypoints (node label + per-axis targets), built
    // by the UI from a map's node chain. StartPathMove issues the first leg;
    // Update() watches the live position and issues the next leg once the
    // device arrives (all targeted axes within kPathTolMM). One path at a time.
    struct PathWaypoint {
        std::string label;   // node/position name (logs + progress UI)
        std::vector<std::pair<std::string, double>> targets;
    };
    struct PathProgress {
        bool        active  = false;
        std::size_t devIdx  = 0;
        std::size_t current = 0;   // waypoint being driven to (0-based)
        std::size_t total   = 0;
        std::string label;         // current waypoint label
    };
    bool StartPathMove(std::size_t devIdx, std::vector<PathWaypoint> waypoints);
    void CancelPathMove();         // stops the device and abandons the path
    bool PathActive() const { return pathExec_.active; }
    PathProgress PathStatus() const;

    // ---- Camera commands (Camera page) --------------------------------------
    // Reconnect using the config captured at Start (async, like startup).
    // False if no camera is configured or a connect is already in flight.
    bool ConnectCamera();
    bool DisconnectCamera();     // stop grabbing + close (quick, synchronous)
    bool SetCameraExposure(double us);   // requires Connected

    // ---- Vision: focus measure (layer 2 owns the sampling) ------------------
    // The sharpness score the auto_focus command maximises, and the same number
    // the UI can watch live while jogging Z by hand. Computed by
    // vision/FocusMetric (plain C++) — it does NOT depend on HALCON, so focus
    // works on a machine with no vision licence at all.
    struct FocusSample {
        bool               ok       = false;   // false = no frame available
        double             score    = 0.0;     // Laplacian variance (relative)
        double             meanLuma = 0.0;     // 0-255; catches a black/blown ROI
        unsigned long long frameSeq = 0;       // camera frame this came from
        int                width = 0, height = 0;   // frame size
        int                roiPx = 0;          // ROI edge used (0 = whole frame)
    };

    // Grab the newest frame and score it; publishes topics::kCameraFocus.
    // Thread-safe: the auto-focus worker and the UI both call it. roiPx <= 0
    // scores the whole frame.
    FocusSample MeasureFocus(int roiPx);

    // Camera frame counter — the gate an auto-focus sample uses to prove it is
    // looking at a post-move frame rather than one that straddled the travel.
    unsigned long long CameraFrameSeq() const;

    // Continuous focus sampling from Update() (UI thread). Off by default: it
    // is only worth the per-frame cost while somebody is watching the number.
    void SetFocusStreaming(bool on, int roiPx);
    bool FocusStreaming() const { return focusStream_; }
    int  FocusRoiPx() const { return focusRoiPx_; }
    FocusSample LatestFocus() const;

    // The camera's pixel -> mm calibration, live. Layer 2 owns it because an
    // AUTO alignment station has no UI in the loop: a detector reports pixels
    // and something has to turn them into millimetres on the worker thread.
    // (The manual crosshair pick still converts in the UI, where the operator's
    // click already is — see docs/alignment-procedure-design.md §3.)
    void CameraPixelToMm(double& x, double& y) const;
    void SetCameraPixelToMm(double x, double y);

    // ---- Vision: feature detection (layer 2 owns the sampling) --------------
    // Unlike the focus measure, these DO require HALCON. That asymmetry is the
    // rule, not an accident: focus and the manual pick must survive an expired
    // licence, automatic detection need not.
    //
    // Detectors are cached per model uid — building an NCC model is the
    // expensive part, and a live tuning loop would otherwise rebuild it every
    // frame. InvalidateDetector drops one after an edit, a re-teach or a delete.

    // Grab the newest frame and run `m` over it. ALWAYS fills `out` (a miss is
    // ok = false plus a reason); returns out->ok. Publishes the vision/* topics
    // and caches the result. Thread-safe: the wizard and an engine worker may
    // both call it.
    bool Detect(const VisionModel& m, const std::string& modelDir,
                VisionDetection* out);

    // Run ONLY `m`'s pre-process chain over the newest frame and hand back the
    // filtered 8-bit plane (`w`/`h` are the search ROI that survived clamping).
    // What the teach page's preview panel shows — the SAME chain Detect applies
    // one line before it dispatches to a method, so the picture cannot drift
    // from what the detector actually sees.
    bool PreprocessPreview(const VisionModel& m, std::vector<unsigned char>* gray,
                           FocusRoi* used, std::string* err);

    // Teach `m`'s template from the newest frame. Writes the .pgm under
    // `modelDir` and reports the patch size the caller stores in the model.
    // With a pre-process chain set, the patch is cut from the FILTERED plane —
    // a raw template would score nothing against a filtered live image.
    bool TeachDetector(const VisionModel& m, const std::string& modelDir,
                       const VisionRoi& patch, int* tmplW, int* tmplH,
                       std::string* err);

    void InvalidateDetector(const std::string& uid);   // empty uid = drop all
    VisionDetection LatestDetection() const;

    // ---- Keithley commands (Keithley page) ----------------------------------
    // Reconnect rebuilds the driver from the config captured at Start; the
    // runtime setters require Connected and are safe against the poll loop.
    bool ConnectKeithley();
    bool DisconnectKeithley();
    bool SetKeithleyBias(double volts);
    bool KeithleyOutputOff();
    bool SetKeithleyNplc(double nplc);
    bool SetKeithleyRangeAuto();
    bool SetKeithleyRangeFixed(double amps);
    bool SetKeithleyPollInterval(int ms);   // restart polling at the new rate

    // ---- CLD101x commands (CLD page) ----------------------------------------
    // Same pattern: reconnect from the config captured at Start; setters and
    // output switches require Connected (switches are read-back verified).
    bool ConnectCld();
    bool DisconnectCld();
    bool SetCldTemperature(double celsius);
    bool SetCldLaserCurrentMA(double mA);
    bool CldTecOn();
    bool CldTecOff();
    bool CldLaserOn();
    bool CldLaserOff();

    // ---- UDP3305S PSU commands (PSU page) -----------------------------------
    // Same pattern: reconnect from the config captured at Start; setpoints go
    // through the verified APPLy form, output switches are read-back verified.
    // Channels are 1-based.
    bool ConnectPsu();
    bool DisconnectPsu();
    bool SetPsuVI(int channel, double volts, double amps);
    bool PsuOutputOn(int channel);
    bool PsuOutputOff(int channel);

    // ---- Digital IO commands (IO page + future procedures) ------------------
    // devIdx indexes the HardwareManager IO fleet. Writes require Connected.
    bool SetIoOutput(std::size_t devIdx, int pin, bool on);
    bool SetIoOutputByName(const std::string& device, const std::string& pinName, bool on);
    bool ClearIoLatch(std::size_t devIdx, uint64_t mask);

    // ---- Pneumatic slide commands (Pneumatics page; non-blocking) -----------
    // idx indexes the HardwareManager slide fleet.
    bool ExtendSlide(std::size_t idx);
    bool RetractSlide(std::size_t idx);
    bool StopSlide(std::size_t idx);

    // ---- IO watch (editor window) -------------------------------------------
    // Replace the running watch with cfg (stops the old one first). Also used
    // by startup. Returns false if enabled but nothing could be wired.
    bool ApplyIoWatch(const IoWatchConfig& cfg);

    // ---- Safety (first-priority critical signals, e.g. E-Stop) --------------
    // A critical alert stops all motion immediately (from the watch thread)
    // and blocks new motion commands until acknowledged.
    bool SafetyAlertActive() const { return safetyWatch_ && safetyWatch_->AlertActive(); }
    SafetyWatch::Alert SafetyAlert() const
    {
        return safetyWatch_ ? safetyWatch_->ActiveAlert() : SafetyWatch::Alert{};
    }
    void AcknowledgeSafety() { if (safetyWatch_) safetyWatch_->Acknowledge(); }
    void StopAllMotion();   // hard stop on every motion device

    // ---- Taught-position hand-back (save_position command) ------------------
    // Engines run on worker threads but saved positions are App-owned UI state:
    // a command queues the taught position here; App::Update drains the queue
    // on the UI thread, merges by name into savedPos_ and persists.
    void QueueSavedPosition(const std::string& device, const SavedPos& p);
    std::vector<std::pair<std::string, SavedPos>> TakeSavedPositions();

    // ---- Taught-frame hand-back (align_solve command) -----------------------
    // Same worker->UI route as taught positions: the procedure document is
    // App-owned, so a solved frame is queued here and App::Update stores it
    // against the procedure uid and persists.
    void QueueAlignResult(const std::string& uid, const AlignResult& r);
    std::vector<std::pair<std::string, AlignResult>> TakeAlignResults();

    // ---- Captured view hand-back (capture_view / dut_capture_image) ---------
    // One frame, parked for the Run page's image box. Same worker->UI route as
    // the two above and for the same reason — a command runs on an engine
    // worker and must never touch GL — but a SLOT, not a queue: a 5 MP frame is
    // ~15 MB, the box can show exactly one, and the one worth showing is the
    // last one asked for. A queue here would hold megabytes of pictures nobody
    // will ever see.
    //
    // The UI TAKES the frame (moves the pixels out) rather than copying it: the
    // bytes exist only to become a texture, and holding a second copy in the
    // slot afterwards would double the cost of every capture for nothing.
    struct CapturedView {
        std::vector<unsigned char> rgb;   // w * h * 3
        int         w = 0, h = 0;
        std::string label;       // what the operator was asked to look at
        std::string stamp;       // when it was taken
        std::string savedPath;   // record-relative path, "" if display-only
    };
    void SetCapturedView(CapturedView v);
    bool TakeCapturedView(CapturedView* out);   // false = nothing new

    // ---- LIVE taught frames (align_solve -> align_apply / move_local) -------
    // Runtime state, deliberately NOT the document. A frame describes where ONE
    // physical part sat. The moment that part could have changed, a frame taught
    // against it is not merely stale but actively dangerous: move_local would
    // drive to coordinates derived from it without complaining, and the error
    // looks exactly like a correct move.
    //
    // So a run NEVER consumes the frame saved in the product file. That copy is
    // a RECORD — the Alignment page shows it, the Aux overlay draws its E1/E2,
    // traceability keeps it — and nothing more. Only a frame taught in the
    // current session is usable, and it is cleared at every boundary where the
    // part might have changed: program start, DUT open, product load, app start.
    //
    // Held HERE rather than in an engine Env because a program runs each slot as
    // its OWN engine Start: per-run state would be wiped between the recipe that
    // aligns and the recipe that works.
    void SetLiveFrame(const std::string& alignUid, const AlignResult& r);
    bool LiveFrame(const std::string& alignUid, AlignResult* out) const;
    void ClearLiveFrames(const char* reason);
    std::size_t LiveFrameCount() const;

    // ---- Reliability hand-back (vision_test command) ------------------------
    // The SAME route, and for a reason worth writing down: the engine's Status
    // ctx snapshot is NOT a usable channel for a finished run. RecipeEngine
    // clears runCtx_ in its terminal block BEFORE the state becomes Completed,
    // so a UI that waits for Completed and then reads Status.ctx always reads an
    // empty map. Anything a command has to hand back after the run must be
    // queued here.
    void QueueVisionReliability(const std::string& uid, const VisionReliability& r);
    std::vector<std::pair<std::string, VisionReliability>> TakeVisionReliability();

    // ---- E-point hand-back (align_station command) --------------------------
    // Same route again, but for a different reason: this one is FEEDBACK, not
    // persistence. align_solve only fires after BOTH stations are captured, so
    // without this the UI has nothing to show while station 2 is being marked —
    // exactly when an operator most wants to see that E1 landed correctly.
    struct AlignPointMark {
        std::string alignUid;
        int         station = 0;   // 1 = E1, 2 = E2
        AlignPoint  point;
    };
    void QueueAlignPoint(const std::string& uid, int station, const AlignPoint& p);
    std::vector<AlignPointMark> TakeAlignPoints();

    // ---- The dispense needle's lever arm ------------------------------------
    // Machine state, not a document — see core/NeedleCal.h for why it is
    // machine-level and why it is held in axis space. Loaded at Start, read by
    // worker threads (needle_offset, and every dispense move derived from it),
    // written by the UI thread after a calibration run hands one back.
    //
    // Unlike a taught FRAME there is no live-vs-saved distinction here, and the
    // difference is worth stating because the two look alike. A frame describes
    // one physical part, which may have been swapped since; the needle offset
    // describes the tooling, which has not. The persisted record IS the licence
    // to move, and it goes stale for exactly one reason: somebody changed the
    // needle. That is what `calibratedAt` is on the page for.
    NeedleCal Needle() const;
    void      SetNeedleCal(const NeedleCal& c);

    // Same worker->UI hand-back as a taught frame, for the same reason: a
    // command must not write a config file from an engine thread.
    void      QueueNeedleCal(const NeedleCal& c);
    std::vector<NeedleCal> TakeNeedleCals();

    // ---- Device registrations (a gripper tip in the camera's space) ---------
    // Machine state on exactly the same terms as the needle offset above, and
    // held here for the same reasons: worker threads read it (device_reg,
    // move_to_point), the UI thread writes it after a run hands one back, and
    // the persisted record IS the licence to move. See core/DeviceReg.h.
    DeviceReg DeviceRegFor(const std::string& device) const;
    std::map<std::string, DeviceReg> DeviceRegs() const;
    void      SetDeviceReg(const DeviceReg& r);
    void      QueueDeviceReg(const DeviceReg& r);
    std::vector<DeviceReg> TakeDeviceRegs();

    // The three columns to AIM a registration measurement with: the measured
    // rotation when `device` has one, otherwise the permutation implied by
    // config/motion_transforms.json (R_seed = M_ref * M_dev^T). `seeded` says
    // which — a seed is good to a couple of degrees, which is enough to put the
    // tip back in a 6 mm field of view after a 10 mm jog and nowhere near good
    // enough to be the answer.
    //
    // Column j is the direction the reference device's axes see when `device`
    // moves +1 mm on its axis j. False when either device is unknown.
    bool DeviceRegAimColumns(const std::string& device, const std::string& refDevice,
                             RegVec3 cols[3], bool* seeded) const;

    // Where a registered device's tip is right now, in the reference device's
    // axis space. False — with the reason in `why` — when there is no valid
    // registration, the device is not connected or does not report XYZ, or its
    // rotations have left the ones the fit was taught at.
    bool DeviceRegTip(const std::string& device, RegVec3& tip, std::string* why) const;

    // Live gantry XY in machine mm. Layer 2 answers "where is the gantry"
    // because layer 2 is what knows WHICH device that is (GantryIndex below);
    // a UI overlay must not go hunting for a device called "gantry-main".
    // False when there is no gantry or it does not report X/Y.
    bool GantryXY(double& x, double& y) const;

    // Live axis values of a motion device, looked up by its CONFIG name. Same
    // rule as GantryXY one line up: layer 2 owns the fleet lookup so a UI
    // readout never walks the device list itself. False when no device carries
    // that name or it is not connected; `out` is only written on true.
    bool DevicePose(const std::string& name, std::map<std::string, double>& out) const;

private:
    // Fleet index of the gantry: "gantry-main" by name, else the first ACS
    // controller. Returns hw_.MotionCount() when there is none.
    std::size_t GantryIndex() const;

    void StartCamera(const HardwareConfig& hw);
    void LaunchCameraConnect();   // async connect from camCfg_ (Start + reconnect)
    void LaunchKeithleyConnect(); // async connect from keiCfg_ (Start + reconnect)
    ISmu* ConnectedKeithley() const;           // null unless state == Connected
    void StartCld(const HardwareConfig& hw);
    void LaunchCldConnect();      // async connect from cldCfg_ (Start + reconnect)
    CLD101x* ConnectedCld() const;             // null unless state == Connected
    void StartPsu(const HardwareConfig& hw);
    void LaunchPsuConnect();      // async connect from psuCfg_ (Start + reconnect)
    UDP3305S* ConnectedPsu() const;            // null unless state == Connected
    void StartKeithley(const HardwareConfig& hw);
    void StartMotion(const HardwareConfig& hw);
    void StartIO(const HardwareConfig& hw);
    void StartPneumatics(const HardwareConfig& hw);   // wires slides to IO pins
    void StartIoWatch(const HardwareConfig& hw);      // wires edge->action bindings
    void StartSafetyWatch(const HardwareConfig& hw);  // wires safety signals
    void StartNeedleCal();   // loads the needle lever arm from its own file
    void StartDeviceRegs();  // loads the per-device tip registrations
    void PublishTips();      // one tip/<device>/<axis> triple per valid record

    // Mapped-path executor state. Mutated on the UI thread only (Start/Cancel/
    // TickPath); other threads (safety watch) request cancellation via the
    // atomic flag, honored on the next TickPath. `active` is itself atomic
    // because StopMotion — which reads it — may be called from the recipe
    // engine's worker thread; the other fields remain UI-thread-only.
    struct PathExec {
        std::atomic<bool> active{false};
        std::size_t devIdx = 0;
        std::vector<PathWaypoint> waypoints;
        std::size_t cur     = 0;
        float       legTime = 0.0f;   // seconds spent on the current leg
    };
    void TickPath(float dt);   // called from Update()

    // Taught-position queue (worker producers, UI-thread consumer).
    std::mutex teachMu_;
    std::vector<std::pair<std::string, SavedPos>> teachQueue_;

    // Taught-frame queue (same producer/consumer split).
    std::mutex alignMu_;
    std::vector<std::pair<std::string, AlignResult>> alignQueue_;

    // Reliability-result queue (same producer/consumer split).
    std::mutex visionRelMu_;
    std::vector<std::pair<std::string, VisionReliability>> visionRelQueue_;

    // Captured-E-point queue (same producer/consumer split).
    std::mutex alignPtMu_;
    std::vector<AlignPointMark> alignPtQueue_;

    // The captured-view slot. `shotPending_` is a separate flag rather than
    // "is rgb non-empty", so a take can move the pixels out and leave the
    // vector's capacity behind for the next capture to reuse.
    std::mutex   shotMu_;
    CapturedView shot_;
    bool         shotPending_ = false;

    // Live taught frames by alignment uid. Read by worker threads (move_local)
    // and by the UI (the "live" indicator), written by align_solve.
    mutable std::mutex                       frameMu_;
    std::map<std::string, AlignResult>       liveFrames_;

    // The needle lever arm, plus its worker->UI queue.
    mutable std::mutex     needleMu_;
    NeedleCal              needle_;
    std::vector<NeedleCal> needleQueue_;

    // The device registrations, keyed by device name, plus the same queue.
    mutable std::mutex               regMu_;
    std::map<std::string, DeviceReg> regs_;
    std::vector<DeviceReg>           regQueue_;
    // Devices whose tip publish is currently suppressed, so the reason is logged
    // on the edge and not once per frame. Keyed by device, value is the last
    // reason — a changed reason logs again.
    std::map<std::string, std::string> tipMuted_;

    // Motion is inhibited while a CRITICAL safety alert is unacknowledged.
    bool SafetyBlocked() const
    {
        return safetyWatch_ && safetyWatch_->AlertActive() && safetyWatch_->AlertCritical();
    }

    HardwareManager& hw_;

    // Connect futures are held here (not in the UI). Destroyed before hw_ in App,
    // so their blocking dtor still sees a live HardwareManager.
    std::future<void>              camConnect_;
    std::future<void>              keiConnect_;
    std::future<void>              cldConnect_;
    std::future<void>              psuConnect_;
    std::vector<std::future<void>> motionConnect_;   // one per motion device
    std::vector<std::future<void>> ioConnect_;       // one per IO device

    std::map<std::string, AxisTransform> transforms_;   // global->local, per device

    // Camera config captured at Start so the Camera page can reconnect later.
    CameraConfig camCfg_;
    bool         camCfgValid_ = false;

    // Focus sampling. MeasureFocus runs on whichever thread asks (UI readout or
    // the auto-focus worker), so the scratch buffer and the last result are
    // mutex-guarded; the two knobs are atomic because Update() reads them every
    // frame while the UI writes them.
    mutable std::mutex         focusMu_;
    std::vector<unsigned char> focusBuf_;      // frame scratch, reused
    FocusSample                lastFocus_;
    std::atomic<bool>          focusStream_{false};
    std::atomic<int>           focusRoiPx_{256};
    unsigned long long         focusSeqSeen_ = 0;   // Update(): last frame scored

    // Feature detection. Its own mutex, not focusMu_: a live tuning detect can
    // take tens of milliseconds, and holding that lock would stall the UI's
    // per-frame focus readout for no reason. Detectors are keyed by model uid
    // and rebuilt lazily whenever the cached copy no longer matches the model.
    mutable std::mutex                                        detectMu_;
    std::vector<unsigned char>                                detectBuf_;
    std::map<std::string, std::unique_ptr<HalconDetector>>    detectors_;
    std::map<std::string, std::string>                        detectorStamp_;
    VisionDetection                                           lastDetection_;

    // Keithley config captured at Start (same pattern as the camera).
    KeithleyConfig keiCfg_;
    bool           keiCfgValid_ = false;

    // CLD101x config captured at Start (same pattern).
    Cld101xConfig cldCfg_;
    bool          cldCfgValid_ = false;

    // UDP3305S PSU config captured at Start (same pattern).
    PsuConfig psuCfg_;
    bool      psuCfgValid_ = false;

    // Per-(device,axis) jog target: rapid presses accumulate onto one evolving
    // target so the latest press overrides the previous instead of queuing; the
    // target re-seeds from the live position after an idle gap (new burst).
    struct JogTrack { double target = 0.0; double lastT = -1.0e9; };
    std::map<std::string, JogTrack> jogTracks_;

    float clock_  = 0.0f;
    float sample_ = 0.0f;

    PathExec          pathExec_;
    std::atomic<bool> pathCancelReq_{false};   // set from any thread

    // Owned by Workflow (not HardwareManager): its callback stops motion via
    // this object, so it must die first. Declared last so it is destroyed
    // before every other member; ~Workflow also stops it explicitly.
    std::unique_ptr<SafetyWatch> safetyWatch_;
};

} // namespace firedog
