// App.h : layer 3 (UI) — main loop, Main window + Aux window (own OS window).
// The UI only renders: it reads HardwareManager::Snapshot()/accessors and the
// DataBus. It owns no device and runs no connect — Workflow (layer 2) does that.
#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "imgui.h"

#include "ui/GlHost.h"
#include "ui/Widgets.h"        // MarkIcon (the pt1/pt2 step diagrams)
#include "command/CommandRegistry.h"   // ParamSpec (schema forms)
#include "command/LuaEngine.h"         // ScriptInfo (Scripts page)
#include "command/Orchestration.h"     // registry + engines + runner, one unit
#include "command/Program.h"
#include "command/Recipe.h"
#include "core/Log.h"
#include "core/MotionMap.h"
#include "core/Product.h"
#include "core/SavedPositions.h"
#include "hardware/HardwareManager.h"
#include "hardware/HardwareConfig.h"
#include "session/ProductSession.h"
#include "workflow/Workflow.h"

namespace firedog {

// Which content the Main window shows. Hardware pages are selected via the
// Hardware menu; the editor pages (View menu) REPLACE the whole panel layout
// until closed via their top-right Close button (back to the previous page).
// VisionTeach and AlignNominal are takeovers that are deliberately NOT in the
// View menu: each teaches ONE station or nominal, so the only way in is that
// thing's own button. AlignPlace is in the menu — it carries no state the
// caller has to supply (it picks its own procedure and nominal), and being
// three clicks deep inside another page made it findable only by someone who
// already knew it was there. All three Close back to Align rather than to
// prevPage_ (a takeover->takeover switch does not update prevPage_).
enum class Page { Overview, Camera, Keithley, CLD, PSU, Motion, IO, Pneumatic,
                  WatchEdit, TransformEdit, RecipeEdit, Scripts, DutView, Run,
                  Align, VisionTeach, AlignNominal, AlignPlace, Needle };

// One XCAN launch preset (a button on the Run page's Scan band): the step
// ladder + the hexapod_align.lua tunables injected at launch. Five slots;
// right-click a button to edit in the settings modal. Persisted machine-
// global in config/scan_presets.local.json (scan_presets.json fallback).
// Device / feedback topic / trace / exit target + zones are NOT here — they
// come from the button column, the band header, and the Target presets.
struct XcanPreset {
    std::string label   = "Preset";
    bool        enabled = false;        // show as a launch button
    std::string stepsUm = "10,5,2,1";   // coarse -> fine ladder, um
    std::string axes    = "Z,Y,X";      // coordinate-descent order
    double settleMs      = 150.0;       // fresh-sample ceiling
    double settleMinMs   = 0.0;         // floor for slow receivers
    int    freshSamples  = 2;           // new publishes per measure
    int    avgCount      = 3;           // baseline averaging samples
    int    consecLimit   = 3;           // stop dir after N non-improvements
    double sigDropPct    = 15.0;        // hard break on a big relative drop
    double maxTravelUm   = 200.0;       // per-direction travel cap
    double improvePct    = 1.0;         // global-peak recovery threshold
    double minSignal     = 0.0;         // abort if |baseline| below this
    double tolMm         = 0.001;       // move arrival tolerance
    double moveTimeoutMs = 10000.0;
    double velocityMmS   = 1.0;         // scan velocity, set once
    bool   skipOpposite  = true;        // skip -dir after +dir improved
    bool   dirMemory     = true;        // last winning direction first
    bool   reuseSample   = true;        // no re-measure of unmoved points
};
inline constexpr int kXcanPresetCount = 5;

// One XCAN target preset (Target row of the Scan band): exit target + the
// adaptive step-zone range, per product power class. Five slots; left-click
// activates (yellow), RIGHT-click edits in the settings modal. Persisted in
// config/scan_presets.local.json alongside the launch presets. All in uA
// (feedback is amps; injected x1e-6 at launch).
struct XcanTarget {
    bool   enabled    = false;   // show as a Target button
    double targetUa   = 400.0;   // align_exit_threshold
    double zoneLowUa  = 20.0;    // align_zone_low  (adaptive ladder lower limit)
    double zoneHighUa = 400.0;   // align_zone_high (adaptive ladder upper limit)
};

// One climb-raster launch preset (a button on the Scan band's Climb tab): the
// climb_raster.lua tunables injected at launch. Same five-slot, right-click-to-
// edit shape as XcanPreset, persisted alongside it in scan_presets.local.json.
// Device / feedback topic / trace are NOT here (button column + band header),
// and neither is the TARGET: the climb reads the same active XcanTarget the
// XCAN tab does, because the target is a property of the product's power class,
// not of the search that drives at it. Only the exit FRACTION lives here.
struct ClimbPreset {
    std::string label   = "Preset";
    bool        enabled = false;      // show as a launch button
    double targetPct      = 85.0;     // fraction of the target that ends the run
    double zStepUm        = 1.0;      // climb travel per step
    double maxZUm         = 20.0;     // total climb travel budget
    double zProbePct      = 5.0;      // gain that means "Z alone is still paying" (0 = off)
    int    maxSkips       = 5;        // forced plane search after N skips (0 = unlimited)
    bool   spiral         = false;    // plane scan: false = cross, true = spiral
    double xyStepUm       = 0.5;      // plane grid pitch
    int    xySteps        = 6;        // grid half-width in cells (6 = 13 x 13)
    int    peakDrop       = 2;        // cross: non-improvements that mean "past the peak"
    int    ringsStop      = 2;        // spiral: rings without improvement (0 = full grid)
    double improvePct     = 2.0;      // noise margin on "is this better?"
    double contactDropPct = 10.0;     // plane best this far under the peak = contact (0 = off)
    int    stallPlanes    = 0;        // planes without a new peak before stopping (0 = off)
    std::string zAxis     = "Z";      // the climb axis
    std::string planeAxes = "X,Y";    // the two searched axes
    double settleMs       = 150.0;    // fresh-sample ceiling
    double settleMinMs    = 0.0;      // floor for slow receivers
    double zSettleMs      = 200.0;    // extra wait after a Z step
    int    freshSamples   = 2;        // new publishes per measure
    int    avgCount       = 3;        // baseline averaging samples
    double tolMm          = 0.001;    // move arrival tolerance
    double moveTimeoutMs  = 10000.0;
    double velocityMmS    = 1.0;      // scan velocity, set once
};
inline constexpr int kClimbPresetCount = 5;

class App {
public:
    int Run();

private:
    void Update(float dt);
    void DrawMainWindow();
    void ToggleEditorPage(Page p);   // enter/leave a full-page takeover (View menu + F-keys)
    void SwitchPage(Page p);         // ALL page switches route here (dirty-editor guard)
    // A UI-commanded move, watched to completion. The UI must never block on a
    // move, but a surface that DRIVES somewhere before it can be used (the
    // vision teach page, opened from a station card) has to be able to say "not
    // there yet" — otherwise the operator tunes a detector against whatever the
    // camera happened to see mid-travel. Start it, and the frame reports it on
    // every page until the axes arrive. Definitions in MainWindow.cpp.
    bool StartWatchedMove(const std::string& device, const std::string& posUid,
                          const std::string& what);
    void UpdateMoveWatch(float dt);   // arrival poll; called from Update()
    void DrawMoveStatusBar();   // the banner, drawn by the main frame
    void DrawLeftPanel();      // 20% column: hardware status + DUT + display
    void DrawDutBlock();       // active-DUT status + open/close (Overview left)
    void DrawMiddlePanel();    // main content: vision image + overlay
    void DrawLowerPanel();     // 15%-height strip: log viewer
    void DrawRightPanel();     // 15% column: Overview placeholder
    void DrawJogWindow();      // undocked Jog window (toggled from View menu)
    void DrawJogPanel();       // jog controls: device/velocity/step/direction/keys
    void DrawJogCameraTab();   // Jog window "Camera" tab: camera combo + exposure
    // Camera tab Autofocus button: runs the auto_focus command as a one-step
    // ad-hoc recipe on the engine worker (the sweep blocks; the UI must not).
    void StartQuickAutoFocus();
    void DrawVelocityCombo(std::size_t di);   // shared velocity combo (Jog + Motion page)
    // Camera page — the (single) Pylon camera.
    void DrawCameraListPanel();    // left:   status + connect/disconnect
    void DrawCameraDetailPanel();  // middle: exposure + pixel->mm + preview
    void DrawCameraExposureControls();   // exposure slider+input (page + Jog tab)
    void DrawCameraInfoPanel();    // right:  identity + stream info
    // Keithley page — the 6482 picoammeter.
    void DrawKeithleyListPanel();    // left:   status + connect/disconnect
    void DrawKeithleyDetailPanel();  // middle: bias + measurement settings + chart
    void DrawKeithleyInfoPanel();    // right:  identity + settings + live values
    // CLD page — the Thorlabs CLD101x laser + TEC controller.
    void DrawCldListPanel();       // left:   status + connect/disconnect
    void DrawCldDetailPanel();     // middle: TEC + laser controls
    void DrawCldInfoPanel();       // right:  identity + live values

    // PSU page — the UNI-T UDP3305S 3-channel DC power supply.
    void DrawPsuListPanel();       // left:   status + connect/disconnect
    void DrawPsuDetailPanel();     // middle: per-channel V/I + output switches
    void DrawPsuInfoPanel();       // right:  identity + live values
    // Motion page — all motion devices (PI + ACS) on one page. The middle
    // panel is tabbed: "Positions" (saved-position table) and "Mapped Path"
    // (node-edge graph editor + path execution).
    void DrawMotionListPanel();    // left:   device list + status
    void DrawMotionDetailPanel();  // middle: device header + tab bar
    void DrawMotionPositionsTab(std::size_t devIdx, IMotionController* m);
    void DrawMotionMapTab(std::size_t devIdx, IMotionController* m);
    void DrawMotionScanTab(std::size_t devIdx, IMotionController* m);
    void DrawMotionInfoPanel();    // right:  live axis positions
    // IO page — all IO devices (Ezi-IO + WAGO) on one page.
    void DrawIoListPanel();        // left:   device list + status
    void DrawIoDetailPanel();      // middle: selected device + output toggles
    void DrawIoInputPanel();       // right:  live input pin states
    // Pneumatics page — synthetic slides composed from IO pins.
    void DrawPneumaticListPanel();    // left:   slide list + position
    void DrawPneumaticDetailPanel();  // middle: selected slide + Extend/Retract/Stop
    void DrawPneumaticInfoPanel();    // right:  live output/sensor states
    void DrawWatchEditPanel();        // IO Watch editor (full-page, View menu)
    void DrawTransformEditPanel();    // axis-transform editor (full-page, View menu)
    // Recipe editor (full-page, View menu) — Phase B. B1 browse+run, B2 edit.
    void DrawRecipeEditPanel();       // page body: recipes | step tree | inspector
    void DrawRecipeTreeNodes(const std::vector<RecipeStep>& steps,
                             std::vector<std::size_t>& path, bool elseBranch,
                             const std::vector<std::size_t>& runPath, bool running);
    void DrawRecipeInspector(RecipeStep& s, bool canEdit);
    std::string AutoNickname(const RecipeStep& s) const;   // live label from params
    void DrawRecipeTreeToolbar(Recipe& r, bool canEdit);   // move/delete row
    void DrawRecipePalette(Recipe& r, bool canEdit);       // searchable add-step column
    bool DrawParamWidget(const ParamSpec& p, nlohmann::json& params);  // schema form row
    bool DrawConditionEditor(nlohmann::json& cond);   // if / loop-while / wait_until
    void DrawRecipeRunStrip(float height);
    void DrawRecipePromptWindow();    // GLOBAL prompt (recipe + script) — non-modal, always on top
    void DrawManualAdjustWindow();    // GLOBAL manual_adjust jog window — non-modal, always on top
    void DrawAlignPickWindow();       // GLOBAL align_station crosshair pick — non-modal, always on top
    // Alignment page (View menu, full-page): teach a local frame (O1) from two
    // operator-marked features. The capture itself runs through the engines, so
    // teaching and a scripted re-run take the identical code path.
    void DrawAlignPanel();            // page body: procedures | stations | frame
    void DrawAlignListPanel();        // left:   procedures + New/Duplicate/Delete/Save
    void DrawAlignDetailPanel();      // middle: the two stations + Run
    void DrawAlignFramePanel();       // right:  the derived frame + local->machine check
    void DrawAlignStationCard(AlignProc& a, int station);
    void StartAlignmentRun(const AlignProc& a);   // launches alignment_procedure.lua
    // Station positions taught from the Alignment page are PRIVATE to their
    // procedure: named "algn_<procUid>_<n>" and `hidden`, so they stay out of
    // the Motion page table and the recipe position picker. Teaching updates in
    // place (uid stable); deleting the procedure drops them, or they would
    // linger somewhere nobody looks.
    void TeachAlignStation(AlignProc& a, int station);
    void DropAlignStationPositions(const AlignProc& a);   // stations AND nominal views
    // Write `device`'s live pose into the hidden position named `name`,
    // creating it if new; "" when the device cannot be read. The uid is stable
    // across a re-teach, which is what keeps references alive.
    std::string CapturePrivatePose(const std::string& device, const std::string& name);
    // A nominal's CAMERA VIEW: where the procedure's device has to stand to see
    // the object being placed there — named `algn_<procUid>_<nomUid>_v` and
    // hidden, on the same ownership rule as a station's position. It is what
    // lets place_view drive there instead of every placement needing a
    // move_to_position step in front of it.
    const SavedPos* AlignNominalView(const AlignProc& a, const AlignNominal& n) const;
    void TeachAlignNominalView(AlignProc& a, AlignNominal& n);
    void GoToAlignNominalView(const AlignProc& a, const AlignNominal& n);
    void DropAlignNominalView(const AlignProc& a, AlignNominal& n);
    // Nominal teach page (a takeover, reached only from the Alignment page's
    // frame panel). Two crosshair marks on the live image; their MIDPOINT is
    // converted through the LIVE frame into local (O1) and stored on the
    // procedure, so it follows the part instead of the machine.
    void DrawAlignNominalList(AlignProc& a);       // the list, on the frame panel
    void DrawAlignNominalPanel();                  // page body (takeover frame)
    void OpenAlignNominalTeach(const AlignProc& a, const std::string& nomUid);
    // Placement page: measure where a gripped object actually is (the same two
    // marks), and work out the correction that would put it on a nominal.
    // COMPUTE ONLY — nothing is wired to move yet.
    void DrawAlignPlacePanel();                    // page body (takeover frame)
    void OpenAlignPlace(const AlignProc& a, const std::string& nomUid);
    // Launch placement_procedure.lua with this page's settings — the guided
    // twin of the manual flow beside it, and the SAME code path a recipe's
    // place_run step takes. Without it the script is only reachable from a
    // recipe, because running it bare from the Scripts page injects no
    // parameters and it fails on the first one by design.
    void StartPlacementRun(const AlignProc& a, const AlignNominal& n,
                           const std::string& device);

    // Needle page (View menu, full-page): the dispense needle's lever arm — what
    // it currently is, and the button that measures it again. Deliberately a
    // teach-and-report surface with nothing that actuates: the calibration run
    // itself fires a shot, but it does so through the engines behind the
    // procedure's own prompts, not from a UI button.
    void DrawNeedlePanel();
    void StartNeedleCalRun();   // launches needle_calibration.lua
    // Vision detection teach window (GLOBAL, non-modal — a big live image does
    // not fit the Alignment page's middle column). Opened from a station card;
    // tunes that station's detection model until it finds the feature reliably.
    // Every HALCON-backed control is disabled with the licence reason when
    // HalconDetector::Available() says no.
    //
    // A model is PRIVATE to one station, on the same principle as the private
    // station positions above: parameters and a taught patch only mean anything
    // at the pose and exposure they were tuned at, so there is nothing to share
    // and nothing to name by hand. The model is named `vdet_<procUid>_<station>`
    // and that name IS the ownership record — hence the two maintenance calls.
    void DrawVisionTeachPanel();                         // page body (takeover frame)
    void OpenVisionTeach(AlignProc& a, int station);     // creates the model, switches page
    void ResetVisionTeachState();                        // both openers start here
    const VisionModel* StationDetector(const AlignProc& a, int station) const;
    void DropAlignStationDetectors(const AlignProc& a);  // with the procedure
    void CopyAlignStationDetectors(const AlignProc& src, AlignProc& dst);
    // The same page and the same private-ownership rule, for the two marks that
    // locate an OBJECT being placed on a nominal: `vdet_<procUid>_<nomUid>_p<N>`.
    // Teaching these is what lets a placement run without an operator; a
    // detector that fails still falls back to asking, loudly.
    void OpenVisionTeachForNominal(AlignProc& a, AlignNominal& n, int index);
    const VisionModel* NominalDetector(const AlignProc& a, const AlignNominal& n,
                                       int index) const;
    void DropNominalDetectors(const AlignProc& a, const AlignNominal& n);
    // Copy a model under a new private name, patch file included; "" on failure.
    std::string CopyVisionModelAs(const VisionModel& src, const std::string& newName);
    void DrawVisionTeachImage(VisionModel& m);           // live image + ROI/box drag
    void DrawVisionCameraFoot(VisionModel& m);           // under the image: camera + zoom
    void DrawVisionTeachParams(VisionModel& m);          // method + parameter form
    // The three pre-process slots and the picture they produce. Its own function
    // because it is the one band that is not about a method: the chain runs
    // before any of them and belongs to the model, not to the form below it.
    void DrawVisionFilterBand(VisionModel& m);
    // Right column: the matched scene beside the taught patch, the detection as
    // a key/value table, and the status line.
    void DrawVisionResultPanel(const VisionModel& m);
    // Recompute the filtered plane (and the overlay built from it) when the
    // chain, the ROI or the frame has moved on. `force` is the Preview button.
    void RefreshVisionPreview(const VisionModel& m, bool force);
    // Load (or synthesise) a model's template patch into `pt`, once per key.
    // False = this method has no picture to show (a blob chain matches a shape).
    // Shared by the teach page, the Alignment page and the manual-pick window:
    // one loader, so the picture an operator is shown at run time is the same
    // one the procedure was authored against.
    bool EnsurePatchTex(PatchTex& pt, const VisionModel& m);
    // The anchor pick, in its own window rather than on the parameter column's
    // thumbnail: there one screen pixel covers several template pixels, so a
    // click was a coarse guess dressed up as a measurement. Opens by itself
    // after a (re-)teach — a fresh patch always has its anchor back at centre.
    void DrawVisionAnchorWindow(VisionModel& m);
    void OpenVisionAnchorPicker();                       // open + refit
    void StartVisionReliabilityRun(const VisionModel& m);   // ad-hoc vision_test recipe
    // Worker->UI hand-off for a finished reliability run. Drains Workflow's
    // queue (the engine's Status ctx is already gone by the time the run reads
    // Completed) and PARKS the result; the page's Accept button is what puts it
    // on the model. Called from Update(), like the other two drains.
    void DrainVisionTestResult();
    // Worker->UI hand-off for the E points, plus the idle fallback that mirrors
    // the SELECTED procedure's taught frame. Update(), like the other drains.
    void DrainAlignPoints();
    // Programs page (View menu, full-page) — Phase C1: list/run .lua programs.
    void DrawScriptsPanel();          // page body: scripts | source | run info
    void LoadScriptsList();           // session rescan + Scripts page selection sync
    void SyncScriptSelection();       // clamp scriptSel_ / drop the cached source view
    // DUT Records viewer (View menu, full-page, read-only) — Phase D2.
    void DrawDutViewPanel();          // serial list | entries table | summary
    void LoadDutList();               // scan the product's data/ for records
    // Run page (View menu, full-page) — programs = slots of recipes. R1.
    void DrawRunPanel();              // deck | execution + Scan band + log | DUT
    void StartDeckRun(RunScope scope);   // granularity follows the deck selection
    void SelectDeckProgram(int idx);     // load a program into the deck (slot 0)
    void SelectDeckSlot(int slot);       // show one slot; drops the recipe selection
    void RememberDeckSelection();        // persist program uid + slot to UiSettings
    void RestoreDeckSelection();         // re-point the deck after a document load
    void DrawRunScanBlock();          // shared channel/Target/Trace header + Raster|Climb|XCAN tabs
    void DrawRunXcanBlock();          // XCAN tab: hex-left/right launch presets
    void DrawRunQuickAlign();         // operator band: 5 one-press target+ladder launches
    void StartXcanPreset(const std::string& device, int presetIdx);
    void DrawXcanPresetModal();       // right-click settings for a launch preset
    void DrawXcanTargetModal();       // right-click settings for a target preset
    void LoadScanPresets();           // config/scan_presets.local.json -> .json
    void SaveScanPresets() const;
    void DrawRunRasterBlock();        // Raster tab: area-scan launcher (raster_scan.lua)
    void StartRasterScan(const std::string& device);
    void DrawRunClimbBlock();         // Climb tab: hex-left/right launch presets
    void StartClimbPreset(const std::string& device, int presetIdx);
    void DrawClimbPresetModal();      // right-click settings for a climb preset
    void LoadProgramsFromDisk();      // session load + deck selection reset
    void LoadRecipesFromDisk();       // session load + UI selection reset
    void DrawSafetyAlertModal();      // blocking safety alert (E-Stop etc.)
    void DrawExitConfirmModal();      // intercepts the OS close request (X / Alt+F4)
    void DrawAuxWindow();
    // Vision feedback drawn over the Aux window's live image: the detection the
    // detector just made (camera pixels) and the taught/captured E1-E2 marks
    // (machine mm, projected back into the current view). `center`/`avail` are
    // the image box; `scale` is screen px per camera px (the cover fit).
    void DrawAuxVisionMarks(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail,
                            const ImVec2& center, float scale);
    // Where each stage stands, named against the taught positions — the live
    // pose of gantry-main / hex-left / hex-right matched to the nearest saved
    // position of that device. Bottom-left of the image box; pure readout,
    // drawn straight onto `dl` so it can never eat a click meant for the image.
    void DrawAuxPositionLabels(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail);
    void DrawSignalOverlay();  // Keithley readout + chart (floats over the image)
    void DrawLogPanel();       // log viewer (Logger ringbuffer) — Main lower panel
    void DrawImageOverlay();   // image + vision overlay
    void SaveUiSettings();     // persist font choice + size to disk

    // Product manifest (File menu). Load/New replace the session's positions
    // from the product's positions file; Save runs through the manifest
    // entries. Thin wrappers: they also reset the UI selections.
    void DrawProductPopups();  // Load/New modal dialogs (opened from File menu)
    bool LoadProduct(const std::string& name);
    void NewProduct(const char* name);
    void ResetDocumentSelections();   // re-point the UI after a document reload

    GlHost host_;

    // Layer 1 owns the hardware; layer 2 orchestrates it. The UI only renders.
    HardwareManager hardware_;
    Workflow        workflow_{hardware_};

    // Layer 2.5 — command registry + both engines + the program runner, as one
    // unit (see Orchestration.h). Declared AFTER workflow_ so it is destroyed
    // FIRST (~RecipeEngine joins its worker thread while Workflow/
    // HardwareManager are still alive — same discipline as SafetyWatch).
    // Commands are registered once in Run() after Workflow::Start; the
    // registry is read-only afterwards. Gate every Run surface on orch_.Busy().
    Orchestration orch_{workflow_, hardware_};

    // The application document: product manifest + saved positions + maps
    // (+ recipes/programs/scripts as they migrate). UI-THREAD ONLY — the
    // engines only ever see a copy, handed over through Env at Start.
    ProductSession session_;

    // UI-side camera texture — frames are pulled from hardware_.Camera() via
    // UpdateCameraTexture() by every view that shows the live image (Aux,
    // Camera page); the isNew flag makes a second call per frame a no-op.
    void UpdateCameraTexture();
    // The pt1/pt2 step diagrams (assets/icons), loaded once on first use — GL
    // has to be up, and every surface that wants them draws long after that.
    // A missing file leaves tex 0, which every drawing path skips.
    void      EnsureMarkIcons();
    MarkIcon  markIcons_[2];
    bool      markIconsTried_ = false;
    // The assets/recipe_icons set, keyed by FILE stem because several commands
    // share a picture. Loaded on first use like the mark icons, and the
    // failures are cached too: a missing PNG costs one warning, not one per
    // frame. nullptr = there is nothing to draw.
    //
    // UiIcon takes the stem (the New/Save/run_execute/Stop button glyphs);
    // StepIcon takes a step TYPE and routes through StepIconFile (ui/Widgets),
    // which owns the command -> picture mapping.
    const MarkIcon* UiIcon(const char* file);
    const MarkIcon* StepIcon(const char* type) { return UiIcon(StepIconFile(type)); }
    std::map<std::string, MarkIcon> stepIcons_;

    unsigned int               cameraTex_ = 0;
    int                        cameraW_ = 0, cameraH_ = 0;
    std::vector<unsigned char> frameBuf_;

    // Camera page state: config id (for saving back into hardware.json) and
    // the exposure edit value, pulled from the camera once per connection.
    std::string cameraId_;
    float camExposureUs_ = 0.0f;
    bool  camExpoPulled_ = false;

    // Keithley page state — edit values seeded from config at startup, then
    // re-seeded from the driver once per connection (camera-exposure pattern).
    std::string keithleyId_;       // config instrument name (for saving)
    float keiBias_      = 1.0f;    // bias voltage edit (V)
    float keiNplc_      = 5.0f;    // NPLC edit
    bool  keiAutoRange_ = true;
    int   keiRangeIdx_  = 0;       // index into kKeiRanges (App.cpp)
    int   keiPollMs_    = 0;       // poll interval edit (0 = as fast as possible)
    bool  keiPulled_    = false;

    // CLD page state — same seed pattern (config at startup, then the device's
    // own setpoints once per connection).
    std::string cldId_;            // config instrument name (for saving)
    float cldTempSet_    = 25.0f;  // temperature setpoint edit (deg C)
    float cldCurrentSet_ = 50.0f;  // laser current setpoint edit (mA)
    bool  cldPulled_     = false;

    // PSU page state — edits seed from the device's setpoints at connect
    // (same psuPulled_ pattern as the CLD page).
    std::string psuId_;                          // config instrument name
    float psuVSet_[3] = {5.0f, 5.0f, 3.3f};      // per-channel voltage edits (V)
    float psuISet_[3] = {1.0f, 1.0f, 1.0f};      // per-channel current-limit edits (A)
    bool  psuPulled_  = false;

    // Keithley chart buffers — refreshed once per frame by
    // UpdateKeithleyChartData (from Update); rendered by DrawKeithleyChart in
    // both the Aux overlay and the Keithley page.
    void UpdateKeithleyChartData(float dt);
    void DrawKeithleyChart(const ImVec2& size);
    int                       chartCh_ = 2;   // 0 ch1, 1 ch2, 2 both
    bool                      chartScope_ = true;  // scope look vs classic axes
    std::vector<float>        kT1_, kV1_, kT2_, kV2_;
    std::vector<float>        kX1_, kX2_;      // X shifted to relative time (t - now)
    double                    chartClock_ = 0.0;   // smooth rolling X right-edge
    static constexpr double   kChartWindowSec = 10.0;

    unsigned int              viewportTex_ = 0;   // GL texture: main-window placeholder

    // Display controls (theme + font picker + size). Theme applies via
    // Theme::Apply; fonts bind to the GlHost font API.
    int   fontIndex_  = 0;       // index into kFonts (see App.cpp)
    float uiFontSize_ = 26.0f;   // current UI font size in px
    int   themeIndex_ = 0;       // index into Theme's theme list

    // Log viewer state — snapshots the Logger ringbuffer, refreshed on change.
    std::vector<LogEntry> logView_;
    size_t                logSeen_ = 0;       // last-seen Logger::Count()
    int                   logMinLevel_ = 2;   // index into level names (2 = Info)
    bool                  logAutoScroll_ = true;

    // Resizable panel sizes in px (0 = seed from a proportion on first frame).
    // Adjusted by the splitter bars in DrawMainWindow; clamped each frame;
    // persisted to UiSettings and shared across all pages.
    float leftW_  = 0.0f;
    float rightW_ = 0.0f;
    float lowerH_ = 0.0f;
    bool  splitterWasActive_ = false;   // to save panel sizes once a drag ends

    // The UI move being watched to completion (see StartWatchedMove). One slot:
    // a second watched move replaces the first, because the operator can only be
    // waiting for the place they asked for LAST. Frame state, not document
    // state — nothing here is persisted.
    struct MoveWatch {
        bool        active = false;
        std::size_t devIdx = 0;
        std::string device;   // display name, for the banner
        std::string what;     // "station 1 of 'calibrate plate'"
        std::vector<std::pair<std::string, double>> targets;
        float       elapsed = 0.0f;   // seconds since the move was commanded
    };
    MoveWatch moveWatch_;

    // Main-window page + which motion device is selected on the Motion page.
    // prevPage_ = the hardware page to return to when an editor page closes.
    Page currentPage_    = Page::Overview;
    Page prevPage_       = Page::Overview;
    int  selectedMotion_ = -1;   // absolute index into the HardwareManager motion fleet
    bool showHiddenPos_  = false;   // reveal hidden (system) positions in the table

    // Mapped Path tab (Motion page) UI state. The maps themselves live in
    // session_ (persisted to config/maps.local.json): nodes reference saved
    // positions by uid, edges are bidirectional, and path moves run through
    // Workflow's path executor.
    int  mapSel_    = 0;         // selected map for the current device
    int  mapSelDev_ = -1;        // device the selection belongs to (reset on switch)
    bool mapEdgeMode_ = false;   // canvas clicks pick edge endpoints
    std::string mapEdgeFrom_;    // first endpoint picked in edge mode
    std::string mapDragNode_;    // node uid being dragged on the canvas
    std::string mapCtxNode_;     // node uid under the context menu
    int         mapCtxEdge_ = -1;   // edge index under the context menu
    std::string mapSelEdgeA_, mapSelEdgeB_;   // click-selected edge (empty = none)
    ImVec2      mapAddAt_ = ImVec2(0.0f, 0.0f);   // canvas spot for "add node here"
    int  mapFrom_ = 0, mapTo_ = -1;    // Go combos (From index 0 = nearest node)
    std::vector<std::string> mapLastPath_;   // last computed path (canvas highlight)
    char mapNameBuf_[32] = "";
    bool openNewMap_ = false, openRenameMap_ = false, openDeleteMap_ = false;

    // IO page: selected device + the output Enable gate (defaults ON per user
    // preference — the checkbox remains as an explicit kill switch).
    int  selectedIo_ = -1;   // absolute index into the HardwareManager IO fleet
    bool ioArmed_    = true;

    // Pneumatics page: selected slide + the same Enable gate.
    int  selectedSlide_ = -1;   // absolute index into the HardwareManager slide fleet
    bool pneuArmed_     = true;

    // IO Watch editor (View menu page): edits a working copy of the io_watch
    // config; Apply rebuilds the live watch, Save also writes config/hardware.json.
    IoWatchConfig watchCfg_;

    // Transforms editor (View menu page): the working copy of ALL device
    // transforms (including devices not in the fleet — preserved on Save).
    // Every edit is hot-pushed to Workflow::SetTransforms so the jog panel
    // reflects it immediately; Save also writes config/motion_transforms.json.
    std::map<std::string, AxisTransform> transformCfg_;

    // Rename-position dialog (name is a label; the uid never changes).
    bool openRenamePos_ = false;    // row Edit button -> popup handoff
    int  renameRow_     = -1;       // row index in the selected device's list
    char renameBuf_[32] = "";

    // Delete-position confirmation dialog (X button asks before erasing).
    bool openDeletePos_ = false;
    int  deleteRow_     = -1;

    // Camera pixel->mm calibration (loaded from CameraConfig): drives the Aux
    // world-distance overlay and the click-to-move offset.
    float pixelToMmX_ = 0.00248f;
    float pixelToMmY_ = 0.00248f;

    // Aux window view toggles. Click-to-move defaults OFF — it drives the real
    // gantry, so it must be explicitly armed via the checkbox.
    bool showChart_       = true;
    bool enableClickMove_ = false;
    int  auxZStepIndex_   = 2;   // index into kAuxZSteps (WndAux.cpp), default 100um

    // Focus ROI for the live score in the Jog window's Camera tab (px, square;
    // 0 = whole frame). The same value the auto_focus command takes as roi_px —
    // watch the number here, then put it in the recipe step.
    int focusRoiPx_ = 256;

    // Autofocus buttons (Jog window Camera tab + Aux window): sweep distance,
    // step (mm), direction (index into kAfDirections) and the last start
    // rejection. Shared App state, so both surfaces launch the SAME sweep and
    // a range widened in one is the range the other uses.
    float       afRangeMM_  = 0.5f;
    float       afStepMM_   = 0.025f;
    int         afDirIndex_ = 0;   // 0 = Z- (down), see kAfDirections
    std::string afErr_;

    // Window visibility (View menu) + jog arming.
    bool showAux_  = true;
    bool showJog_  = true;
    bool jogArmed_ = false;   // jog buttons + W/D·A/S·R/F keys active only when armed

    // Jog panel (Overview right panel). Transforms + commands live in Workflow.
    int jogDevice_    = -1;  // index into the motion fleet
    int jogStepIndex_ = 9;   // index into kSteps (default 100um)
    int jogRotAxis_   = 0;   // selected rotational axis (0 = first beyond XYZ)

    // Per-device velocity selection, shared by the Jog panel and the Motion
    // page so both always show the same value for a device.
    std::map<int, int>    velIndex_;      // device index -> velocity-list index
    std::map<int, bool>   velPushed_;     // device index -> user explicitly set a
                                          // velocity this session ("(device)" before;
                                          // nothing is pushed at connect or exit)
    std::map<int, double> velCustomEdit_; // device index -> custom input-box value
    std::map<int, double> velCustomSet_;  // device index -> applied custom (0 = preset)

    // Recipe Editor page (View menu) UI state — the recipes themselves live in
    // session_ (product recipes file, else config/recipes.local.json).
    int                 recipeSel_ = -1;   // selected recipe (editor left panel)
    std::string         recipeStartErr_;   // last Start() rejection message
    // Selected step as a numeric path into the nested step tree; components
    // >= kElsePathBase (App.cpp) address an if-step's else branch.
    std::vector<std::size_t> edStepSel_;
    // Visible tree rows in draw order (rebuilt every frame) — arrow-key
    // navigation walks this list; collapsed children never appear in it.
    std::vector<std::vector<std::size_t>> edTreeRows_;
    bool edScrollToSel_ = false;   // keyboard move: bring selection into view
    // Step mode: Run starts paused at the selected step (or the first leaf);
    // Next executes one leaf at a time. Toggling mid-run pauses/resumes.
    bool edStepMode_ = false;
    // Editor dialogs/palette state (menu -> popup handoff, like the products).
    int  edAddMode_           = 0;       // palette insert: 0 after, 1 into, 2 else
    int  edConfirmAction_     = 0;       // unsaved-changes guard: 1 leave page, 2 reload
    Page edLeaveTarget_ = Page::Overview;   // where action 1 goes after Save/Discard
    // Width of the editor's inspector column, dragged by the splitter between
    // the step tree and the step form and persisted in UiSettings. Its own
    // field, not the main window's rightW_: the editor is a takeover that
    // replaces that five-region layout, and a step form is a different kind of
    // thing from a hardware info panel.
    float edInspectW_ = 320.0f;
    bool  edInspectSplitting_ = false;   // edge-detect the drag END
    bool edOpenRenameRecipe_  = false;
    bool edOpenDeleteRecipe_  = false;
    bool edOpenDeleteStep_    = false;
    char edNameBuf_[64]       = "";
    char edSearchBuf_[64]     = "";      // palette search filter

    // Alignment page UI state — the procedures themselves live in session_.
    int  alignSel_      = -1;    // selected procedure
    char alignNameBuf_[48] = "";  // rename field for the selected procedure
    double alignProbeX_ = 0.0;   // right panel's local->machine check inputs
    double alignProbeY_ = 0.0;
    std::string alignStartErr_;  // why the last Run refused to start

    // Needle page state. The calibration itself lives on Workflow (machine
    // state, not a document) — these are only the run parameters the page
    // injects and the two things it has to remember between frames.
    char   ndlDevice_[32] = "gantry-main";
    char   ndlSlide_[32]  = "Dispenser_Head";
    char   ndlIo_[32]     = "IOBottom";
    char   ndlPin_[32]    = "Dispenser_Shot";
    int    ndlShotMs_     = 300;
    int    ndlSettleMs_   = 500;
    int    ndlPasses_     = 2;
    double ndlClearMm_    = 1.0;
    double ndlTolMm_      = 0.05;
    double ndlVApproach_  = 20.0;
    double ndlVFinal_     = 5.0;
    bool   ndlAutofocus_  = true;
    std::string ndlStartErr_;    // why the last Run refused to start
    // The bootstrap seed: with no calibration at all the first imprint lands a
    // hundred millimetres outside a three-millimetre field of view, so the
    // operator types a tape-measure estimate once and the procedure has
    // somewhere to aim. Saved INVALID — a guess must never read as a calibration.
    double ndlSeedDx_ = 0.0, ndlSeedDy_ = 0.0, ndlSeedDz_ = 0.0;

    // Nominal teach page state — the nominals themselves live in session_.
    // Page::AlignNominal IS the "open" flag; nomProcUid_ says which procedure.
    std::string nomProcUid_;              // procedure being taught against
    std::string nomEditUid_;              // nominal being re-taught ("" = a new one)
    char        nomNameBuf_[48] = "";
    // The two marks, in MACHINE mm, each latched with the gantry pose it was
    // taken at — so the operator may jog between them and a mark still means
    // the same physical place. Drawn by projecting back through the live pose.
    AlignPoint  nomPt_[2];
    bool        nomHave_[2] = {false, false};
    int         nomActive_  = 0;          // which mark "Set" will write (0 or 1)
    float       nomPx_ = 0.0f, nomPy_ = 0.0f;   // crosshair, camera px
    float       nomZoom_ = 1.5f;          // same default as the pick window
    // Overlay only: rotates the envelope box the two marks bound, for a part
    // whose footprint sits at an angle. It changes nothing that gets saved —
    // the nominal is the midpoint, and the midpoint does not rotate.
    float       nomEnvDeg_ = 0.0f;
    std::string nomErr_;                  // why the last Save refused

    // Placement page state. Nothing here is persisted: a measurement of where
    // one object happens to sit right now is not document data, and keeping it
    // would invite somebody to move against a correction computed yesterday.
    std::string plcProcUid_;              // procedure whose frame is in play
    std::string plcNomUid_;               // the nominal being aimed at
    std::string plcDevice_;               // device HOLDING the object (the mover)
    AlignPoint  plcPt_[2];                // the object's two marks, machine mm
    bool        plcHave_[2] = {false, false};
    int         plcActive_  = 0;
    float       plcPx_ = 0.0f, plcPy_ = 0.0f;
    float       plcZoom_ = 1.5f;
    // Largest correction the page will issue. A computed destination built on a
    // mis-taught frame or a mis-marked object is not implausible-looking — it is
    // an ordinary number that happens to be huge — so the ceiling is on screen
    // and editable rather than a constant nobody can see. Beyond it, Move is
    // refused and the operator has to raise it deliberately.
    float       plcMaxMM_ = 2.0f;
    std::string plcLastMove_;   // what was last issued, to compare the re-measure against
    // Guided-run settings, the script's loop contract (place_run's defaults).
    // align_first defaults ON: the live frame is cleared at program start, DUT
    // open and product load, so a placement that assumed one would fail about
    // half the times it was used.
    bool        plcAlignFirst_ = true;
    int         plcTries_ = 2;
    float       plcTolMM_ = 0.02f;
    std::string plcStartErr_;   // why the last guided run refused to start
    // Repeatability check: repeated measurements of the SAME, stationary
    // object. Their spread is the MARKING error — the floor under any
    // placement, and the only thing that says whether a leftover residual is a
    // real miss or just two operators' worth of pixel. Object midpoints in
    // machine mm; UI-only, never persisted.
    std::vector<std::pair<double, double>> plcSamples_;

    // Vision teach page state — the models themselves live in session_.
    // Page::VisionTeach IS the "open" flag; visionSelUid_ says which model.
    std::string visionSelUid_;          // model being tuned ("" = none)
    std::string visionOwnerLabel_;      // "calibrate plate  -  station 1 (E1)"
    // Where Close goes: the page belongs to whoever opened it (a station card on
    // Align, an object mark on Placement), and a takeover->takeover switch does
    // not update prevPage_.
    Page        visionReturnPage_ = Page::Align;
    // Where the OWNER of this model stands, so the page can drive back there
    // without knowing what an alignment station is. Set by whichever Open*
    // brought us here (a station's private position, or a nominal's camera
    // view); empty when that owner has none taught, which only disables the
    // button.
    std::string visionOwnerDevice_;
    std::string visionOwnerPosUid_;
    // One line of "what just happened", bottom-right. Every action writes it —
    // preview, detect, teach, save, reliability — so the page always answers
    // "did that do anything?" without a trip to the log.
    std::string visionStatus_ = "Ready.";
    // Paint the filtered foreground over the live image. Off, the preview panel
    // is the only view of the chain, and a small panel is a poor place to judge
    // whether a threshold caught the FEATURE or the whole background.
    bool        visionOverlay_ = true;
    // Widths of the middle (parameters) and right (results) columns, dragged by
    // the two splitters and persisted in UiSettings. Their own fields rather
    // than the main window's rightW_: this page is a takeover that replaces
    // that five-region layout entirely. The image column takes the remainder.
    float       visionParamW_ = 380.0f;
    float       visionResultW_ = 300.0f;
    bool        visionParamSplitting_ = false;   // edge-detect the drag END
    bool        visionResultSplitting_ = false;
    float       visionZoom_  = 1.0f;    // magnification over the cover fit
    float       visionFocusX_ = 0.0f;   // camera px the view is centred on
    float       visionFocusY_ = 0.0f;
    // Rubber-band drag over the image. 0 = idle, 1 = dragging the search ROI,
    // 2 = dragging the template box. (The anchor is picked in its own window,
    // never on the live image — see DrawVisionAnchorWindow.)
    int   visionDragMode_ = 0;
    bool  visionDragging_ = false;
    float visionDragX0_ = 0.0f, visionDragY0_ = 0.0f;
    float visionDragX1_ = 0.0f, visionDragY1_ = 0.0f;
    // Last detection + whether to redo it on every new frame. Live is off by
    // default: a full-frame pattern match is not free, and the cost only earns
    // its keep while somebody is actually turning the knobs.
    VisionDetection    visionDet_;
    bool               visionHaveDet_ = false;
    bool               visionLive_    = false;
    unsigned long long visionSeqSeen_ = 0;
    bool               visionSlowWarned_ = false;   // "detect is slow" — say it once
    std::string        visionErr_;                  // last teach/save failure
    int                visionTestRuns_ = 20;
    // A finished reliability run, parked until the operator accepts it. NOT
    // written straight onto the model: whether 20/20 at 0.05 px is good enough
    // is a judgement about this part and this feature, and the person who just
    // watched the run is the one qualified to make it.
    std::string        visionRelUid_;      // which model it belongs to
    VisionReliability  visionRelPending_;
    bool               visionRelHave_ = false;
    // The anchor-pick window (NCC only). Zoom 0 means "fit on the next draw",
    // which is what a freshly taught patch of unknown size needs.
    bool               visionAnchorOpen_ = false;
    float              visionAnchorZoom_ = 0.0f;

    // ---- the pre-process preview -------------------------------------------
    // The filtered plane, as a texture, plus the overlay built from it. Keyed by
    // the chain signature + the ROI + the frame it was computed on, so a draw
    // that runs every frame uploads nothing until something actually changed —
    // the same discipline PatchTex uses. `visionFilterX_/Y_` is the plane's
    // full-frame origin, which is what lets the overlay land on the live image.
    unsigned int visionFilterTex_ = 0;
    unsigned int visionOverlayTex_ = 0;
    int          visionFilterW_ = 0, visionFilterH_ = 0;
    int          visionFilterX_ = 0, visionFilterY_ = 0;
    std::string  visionFilterKey_;   // chain + ROI; "" = nothing computed yet
    unsigned long long visionFilterSeq_ = 0;   // frame the picture was made from
    std::string  visionFilterErr_;   // why the last preview produced nothing
    double       visionFilterMs_ = 0.0;
    // The scene at the last detection, cropped from the RAW frame — what the
    // detector claims to have found, beside the patch it was looking for.
    unsigned int visionCropTex_ = 0;
    int          visionCropW_ = 0, visionCropH_ = 0;
    std::string  visionCropKey_;

    // ---- Aux window vision overlay -----------------------------------------
    // The last detection, aged in Update rather than timed in a Draw: a marker
    // that lingers over a scene the stage has since left is worse than none.
    // Kept as an App member (not a Workflow poll in the draw) so the age and
    // the value can never disagree within a frame.
    VisionDetection    auxDet_;
    unsigned long long auxDetSeq_ = 0;
    float              auxDetAge_ = 1.0e6f;   // seconds since it arrived
    // E1/E2 in MACHINE mm — captured live during a run, else mirrored from the
    // selected procedure's taught frame. Machine space, not pixels, because the
    // two points are captured at different gantry poses: a stored pixel would be
    // meaningless the moment the stage moves.
    AlignPoint alignMarkE_[2];
    bool       alignMarkHave_[2] = {false, false};
    bool       showVisionMarks_ = true;   // Aux checkbox, persisted
    bool       showPosLabels_   = true;   // Aux checkbox, persisted — stage names
    bool       posLabelsHidden_ = true;   // ...and whether `hidden` positions rank
    // Taught-patch thumbnails, one slot per surface that shows one. Each is
    // keyed by what it was built from (VisionModelPatchKey), so a model switch,
    // a re-teach or an anchor drag reloads it and nothing else does.
    PatchTex visionTmpl_;       // vision teach page: the NCC patch + its anchor
    PatchTex alignTmpl_[2];     // Alignment page: station 1/2, or object point 1/2
    PatchTex pickTmpl_;         // the manual-pick window, on the fallback path

    // Scripts page UI state — the scanned list itself lives in session_.
    int                      scriptSel_ = -1;
    // Per-script breakpoints (keyed by path; click a source row to toggle,
    // shift+click for a range from the anchor = last plainly clicked row).
    std::map<std::string, std::set<int>> scriptBps_;
    int scriptBpAnchor_ = -1;

    // serial|openedAt of the DUT record currently open, so Update can
    // edge-detect a NEW one and drop the taught frames with it.
    std::string dutOpenKey_;

    // DUT block (Overview left panel) + the input-prompt modal's text buffer
    // (barcode scanners type + Enter, so EnterReturnsTrue submits).
    char dutSerialBuf_[64]   = "";
    char promptInputBuf_[128] = "";

    // manual_adjust window: selected kSteps preset, re-seeded from the step's
    // step_um each time the window appears.
    int adjustStepIdx_ = 9;
    // Edge-detect a NEW pending prompt/adjust (the windows are non-modal, so a
    // back-to-back request may arrive while the window is still shown).
    bool promptWasPending_ = false;
    bool adjustWasPending_ = false;
    bool pickWasPending_   = false;
    // The crosshair the align pick window places, in ABSOLUTE camera pixels
    // (the Aux window only ever works in centre-relative offsets). Re-seeded to
    // the image centre on every new request, so OK without a click = no offset.
    float pickPx_ = 0.0f, pickPy_ = 0.0f;
    // Magnification over the cover fit (1.0 = fit). Default 1.5 = zoomed in 50%
    // so a small feature is legible; kept across picks so a chosen zoom serves
    // both stations of a procedure. Wheel over the image, or the footer slider.
    float pickZoom_ = 1.5f;
    // Set once the operator confirmed Exit; lets the intercepted GLFW
    // should-close flag through so the run loop actually ends.
    bool exitConfirmed_ = false;

    // Scanning tab (Motion page): XCAN parameter fields, injected into
    // hexapod_align.lua via LuaEngine::Env::initialCtx (same channel as
    // recipe ctx_set steps). Defaults = bobcat's XCAN coarse mode.
    int    xcanDevice_     = -1;   // -1 = seed from the page's selected device
    char   xcanTopic_[64]  = "keithley/ch1";
    char   xcanAxes_[32]   = "Z,Y,X";
    char   xcanSteps_[64]  = "10,5,2,1";
    int    xcanSettleMs_   = 300;
    int    xcanConsec_     = 3;
    int    xcanAvg_        = 3;
    double xcanSigDropPct_ = 5.0;
    double xcanTravelUm_   = 200.0;
    double xcanImprovePct_ = 1.0;
    double xcanMinSignal_  = 0.0;
    double xcanExitThr_    = 0.0;   // early-exit threshold (0 = disabled)
    double xcanTolMm_      = 0.001;
    // Per-sample CSV trace (align_trace=1): shared by the Scanning tab and the
    // Run page's XCAN band; lines persist in the daily log file for analysis.
    bool   xcanTrace_      = false;
    // Run-band exit-target slots (align_exit_threshold presets, in uA): the
    // active slot's value stops the scan the moment the feedback reaches it.
    // Left-click activates a slot, right-click edits it in a modal; 0 = off.
    int    rasterTgtSel_   = 0;   // Raster exit-target preset: 0 = 5 uA, 1 = 20 uA
    int    xcanPresetSel_  = 1;   // ACTIVE target slot (xcanTargets_; default 400 uA)
    // Run-band raster launcher knobs (raster_scan.lua): slow axis X steps down
    // from start, fast axis Y sweeps centered; axes/gating stay script defaults.
    double rasterSlowSpanUm_ = 40.0;                  // X- total travel
    double rasterSlowStepUm_ = 4.0;                   // row pitch
    double rasterFastSpanUm_ = 40.0;                  // Y total width (+/- half)
    double rasterFastStepUm_ = 4.0;                   // sample pitch per row

    // Run page UI state: the deck selection (recipe -1 = slot-level), light
    // edit mode, and the runner's last start rejection. The programs
    // themselves live in session_.
    //
    // deckSlot_ names the ONE slot the deck shows, and is -1 only while the
    // program has no slots at all — there is no "(all slots)" selection and no
    // whole-program run: Execute runs the selected recipe, or that one slot.
    int  progSel_       = -1;
    int  deckSlot_      = -1;
    int  deckRecipe_    = -1;
    bool runEdit_       = false;
    bool runOpenDelete_ = false;
    bool runOpenLoad_   = false;   // request to open the Load Program modal
    std::string runStartErr_;
    // The deck selection remembered across restarts (UiSettings run_prog_uid /
    // run_slot). By UID because a rename must not lose it; re-applied by
    // ResetDocumentSelections after every product load that still has it.
    std::string runLastProgUid_;
    int         runLastSlot_ = 0;
    // Widths of the deck and DUT columns, dragged by the two splitters beside
    // the execution view and persisted in UiSettings. Their own fields, not the
    // main window's leftW_/rightW_: Run is a takeover that replaces that
    // five-region layout, and a program deck is a different kind of thing from
    // a hardware device list.
    float runDeckW_ = 300.0f;
    float runDutW_  = 280.0f;
    bool  runSplitting_ = false;   // edge-detect the drag END (either splitter)
    // Run page XCAN block: feedback channel for the preset scans + live value.
    char runXcanTopic_[64] = "keithley/ch1";
    // XCAN launch presets (up to 5 buttons per hexapod row): defaults seeded
    // in LoadScanPresets, edited via the right-click settings modal.
    XcanPreset xcanPresets_[kXcanPresetCount];
    int  xcanCfgSel_  = 0;       // preset shown in the settings modal
    bool xcanCfgOpen_ = false;   // request to open the modal this frame
    // XCAN target presets (Target row, up to 5 buttons): defaults seeded in
    // LoadScanPresets, edited via their own right-click settings modal.
    XcanTarget xcanTargets_[kXcanPresetCount];
    int  xcanTgtCfgSel_  = 0;
    bool xcanTgtCfgOpen_ = false;
    // Climb launch presets (Climb tab, same five-slot shape). The exit TARGET
    // is not here: the Climb tab drives at the same active xcanTargets_ slot
    // the XCAN tab does — one product power class, two searches aiming at it.
    ClimbPreset climbPresets_[kClimbPresetCount];
    int  climbCfgSel_  = 0;
    bool climbCfgOpen_ = false;
    // Height of the Scan band, dragged by the splitter above it and persisted
    // in UiSettings. 0 = seed from the frame metrics on the first frame — the
    // band holds three tabs of parameter knobs now, so its old fixed six-row
    // height cropped the tab content instead of scrolling it.
    float runScanH_ = 0.0f;

    // DUT Records viewer state: scanned serials + the loaded record.
    std::vector<std::string> dutList_;
    int                      dutSel_ = -1;
    std::string              dutRecSerial_;   // which serial dutRec_ holds
    nlohmann::json           dutRec_;         // loaded dut.json (view only)
    std::string              scriptSrcPath_;    // which file scriptSrc_ holds
    std::vector<std::string> scriptSrc_;        // source lines (view only)
    std::string              scriptStartErr_;   // last Start() rejection

    bool openLoadProduct_ = false;   // menu -> popup handoff (OpenPopup must
    bool openNewProduct_  = false;   // be called outside the menu scope)
    char newProductName_[64] = "";

};

} // namespace firedog
