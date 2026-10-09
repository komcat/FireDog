// App.h : layer 3 (UI) — main loop, Main window + Aux window (own OS window).
// The UI only renders: it reads HardwareManager::Snapshot()/accessors and the
// DataBus. It owns no device and runs no connect — Workflow (layer 2) does that.
#pragma once

#include <array>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "imgui.h"

#include "ui/GlHost.h"
#include "ui/Widgets.h"        // MarkIcon (the pt1/pt2 step diagrams)
#include "ui/UiSettings.h"     // QuickIoRow (the Jog window's IO tab rows)
#include "ui/ScriptEditor.h"   // ScriptEditorState (the Scripts page's editor)
#include "command/CommandRegistry.h"   // ParamSpec (schema forms)
#include "command/LuaEngine.h"         // ScriptInfo (Scripts page)
#include "command/Orchestration.h"     // registry + engines + runner, one unit
#include "command/Program.h"
#include "command/Recipe.h"
#include "core/Log.h"
#include "core/MotionMap.h"
#include "core/Employees.h"
#include "core/Product.h"
#include "core/SavedPositions.h"
#include "hardware/HardwareManager.h"
#include "hardware/HardwareConfig.h"
#include "session/ProductSession.h"
#include "workflow/Workflow.h"
#include "core/MosaicConfig.h"
#include "mosaic/MosaicDoc.h"
#include "mosaic/MosaicWorker.h"

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
enum class Page { Overview, Camera, Keithley, CLD, PSU, Light, Motion, IO, Pneumatic,
                  WatchEdit, TransformEdit, RecipeEdit, Scripts, DutView, Run,
                  Align, VisionTeach, AlignNominal, AlignPlace, Needle,
                  DeviceReg, Grip, Tray, Mosaic };

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

// One mosaic tile's textures on the Mosaic page, at three resolutions:
// [0] 1/16 (always resident once seen - the whole picture at fit zoom),
// [1] 1/4 (the preview filed beside each frame), [2] the full frame. The two
// bigger ones are paged in by the viewer as the zoom asks for them and out
// again least-recently-used, so a mosaic of a few hundred 5 MP frames never
// has to fit on the GPU at once. `used` is the frame counter of the last draw.
struct MosaicTileTex {
    unsigned int       tex[3] = {0, 0, 0};
    int                w[3] = {0, 0, 0}, h[3] = {0, 0, 0};
    unsigned long long used[3] = {0, 0, 0};
    bool               requested[3] = {false, false, false};
};

// The access level a page demands. Anything not named in the table is Admin:
// a page added next year is locked until somebody grants it deliberately, which
// is the only default that cannot fail open. Defined in PageLogin.cpp.
UserRole MinRoleFor(Page p);

// The non-page surfaces. A window has no Page to look up, so its level lives
// here beside the table rather than inline at three call sites.
//   Jog commands motion, so it sits with the Motion page.
//   The Program Organizer edits programs, so it sits with the Recipe Editor.
//   The Aux window only ever displays, so an operator may have it.
inline constexpr UserRole kRoleJog = UserRole::Technician;
inline constexpr UserRole kRoleOrg = UserRole::Technician;
inline constexpr UserRole kRoleAux = UserRole::Operator;
//   View Data reads the same records as the DUT Records page, so it sits with it.
inline constexpr UserRole kRoleData = UserRole::Technician;
//   The Waffle Tray window picks the active cell and marks cavities used or
//   full - what an operator does as parts come and go. Its Drive here is
//   kRoleJog, like every other move.
inline constexpr UserRole kRoleTray = UserRole::Operator;

// One DUT record flattened for the View Data window: the identity columns plus
// the LAST value written under each non-event key. The append-only log may
// write a key twice (a re-run); a table has one cell per key, and the value
// the part left with is the one that describes it. Values stay json so a
// string, a number and an object all land in the same column honestly.
struct DutDataRow {
    std::string serial, opened, closed, status;
    std::vector<std::string> runs;                 // "kind 'name' - result" lines
    std::map<std::string, nlohmann::json> vals;    // key -> last value
};

// The Jog window's Pos tab: a fixed 3x3 pad of quick moves. Fixed, not
// growable, because it is a reach-without-looking surface — a pad that grows a
// row moves every button an operator had learned the place of.
inline constexpr int kPosPadCols  = 3;
inline constexpr int kPosPadRows  = 3;
inline constexpr int kPosPadSlots = kPosPadCols * kPosPadRows;

class App {
public:
    int Run();

private:
    void Update(float dt);
    void DrawMainWindow();
    void ToggleEditorPage(Page p);   // enter/leave a full-page takeover (View menu + F-keys)
    void SwitchPage(Page p);         // ALL page switches route here (dirty-editor guard)

    // ---- Users & access (docs/user-access-design.md) ----------------------
    // The login gate replaces the whole frame while nobody is logged in, so the
    // menu bar, the F-keys and every window simply do not exist yet — there is
    // no surface left to forget to gate. Definitions in PageLogin.cpp.
    void DrawLoginScreen();
    bool LogIn(const std::string& id, const std::string& pw);
    void LogOut();
    bool CanAccess(Page p) const { return user_.role >= MinRoleFor(p); }
    bool CanUse(UserRole need) const { return user_.role >= need; }
    void DrawUserMenu();          // the User menu (MainWindow.cpp)
    void DrawEmployeeManager();   // admin-only roster window (WndEmployees.cpp)
    // A UI-commanded move, watched to completion. The UI must never block on a
    // move, but a surface that DRIVES somewhere before it can be used (the
    // vision teach page, opened from a station card) has to be able to say "not
    // there yet" — otherwise the operator tunes a detector against whatever the
    // camera happened to see mid-travel. Start it, and the frame reports it on
    // every page until the axes arrive. Definitions in MainWindow.cpp.
    bool StartWatchedMove(const std::string& device, const std::string& posUid,
                          const std::string& what);
    // The same, to a pose given as axis values (a computed one — a waffle tray
    // cell — has no uid). The uid form resolves the position and calls this.
    bool StartWatchedMove(const std::string& device,
                          const std::map<std::string, double>& pose,
                          const std::string& what);
    void UpdateMoveWatch(float dt);   // arrival poll; called from Update()
    void DrawMoveStatusBar();   // the banner, drawn by the main frame
    void DrawLeftPanel();      // 20% column: hardware status + DUT + display
    // Active-DUT status + open/close (Overview left, top of the Run deck).
    // `serialScale` = the serial's font size as a multiple of the body size,
    // before it shrinks to fit the row.
    void DrawDutBlock(float serialScale = 2.0f);
    void DrawMiddlePanel();    // main content: vision image + overlay
    void DrawLowerPanel();     // 15%-height strip: log viewer
    void DrawRightPanel();     // 15% column: Overview placeholder
    void DrawJogWindow();      // undocked Jog window (toggled from View menu)
    void DrawJogPanel();       // jog controls: device/velocity/step/direction/keys
    void DrawJogCameraTab();   // Jog window "Camera" tab: camera combo + exposure
    void DrawJogPosTab();      // Jog window "Pos" tab: device + 3x3 quick-move pad
    void DrawJogIoTab();       // Jog window "IO" tab: quick output rows (toggle / pulse)
    // The nine pad slots of one device, created (blank) on first ask. UI state,
    // not the document — a Draw may call it.
    std::vector<std::string>& PosPadSlots(const std::string& device);
    // Camera tab Autofocus button: runs the auto_focus command as a one-step
    // ad-hoc recipe on the engine worker (the sweep blocks; the UI must not).
    void StartQuickAutoFocus();
    void DrawVelocityCombo(std::size_t di);   // shared velocity combo (Jog + Motion page)
    // Camera page — the (single) Pylon camera.
    void DrawCameraListPanel();    // left:   status + connect/disconnect
    void DrawCameraDetailPanel();  // middle: exposure + pixel->mm + preview
    void DrawCameraExposureControls();   // exposure slider+input (page + Jog tab)
    void DrawCameraInfoPanel();    // right:  identity + stream info
    // Keithley page — the configured SMU (6482 picoammeter or 2602B SourceMeter;
    // the driver is chosen from hardware.json and reached through ISmu).
    std::string KeithleyModelName(); // driver's model name, for headers + legend
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
    // The selected supply's channel controls, shared by the PSU page and the
    // Jog window's PSU tab (compact = narrow-window layout).
    void DrawPsuChannelControls(bool compact);
    void DrawJogPsuTab();          // Jog window: quick access to the PSU fleet

    // Light page — the CCS PD3 LED lighting controller.
    void DrawLightListPanel();     // left:   status + connect/disconnect + warnings
    void DrawLightDetailPanel();   // middle: color call-outs + per-channel sliders
    void DrawLightInfoPanel();     // right:  identity + read-back per channel
    void DrawLightCalloutControls();   // color call-out block (Light page + Jog Camera tab)
    // A detection model's "light" call-out, set NOW for judging it on the
    // live image (a LightCalloutPicker click). Queued — no socket I/O here.
    void ApplyLightingNow(const AlignLighting& l);
    // Motion page — all motion devices (PI + ACS) on one page. The middle
    // panel is tabbed: "Positions" (saved-position table) and "Mapped Path"
    // (node-edge graph editor + path execution).
    void DrawMotionListPanel();    // left:   device list + live axes + velocity
    void DrawMotionDetailPanel();  // middle: device header + tab bar
    void DrawMotionPivot(std::size_t devIdx, IMotionController* m);   // PI rotation pivot
    void DrawMotionPositionsTab(std::size_t devIdx, IMotionController* m);
    void DrawMotionMapTab(std::size_t devIdx, IMotionController* m);
    void DrawMotionScanTab(std::size_t devIdx, IMotionController* m);
    // No right panel: the live axes moved under the device list, so the
    // saved-position table in the middle gets the width instead.
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
    void DrawTeachPositionModal();    // "Teach here" confirm, raised from a form row
    // Overwrite an existing saved position (by uid) with the device's live
    // pose. In place — the uid survives, so every reference to it survives.
    bool TeachPositionByUid(const std::string& device, const std::string& uid);
    bool DrawConditionEditor(nlohmann::json& cond);   // if / loop-while / wait_until
    void DrawRecipeRunStrip(float height);
    void DrawRecipePromptWindow();    // GLOBAL prompt (recipe + script) — non-modal, always on top
    void DrawManualAdjustWindow();    // GLOBAL manual_adjust jog window — non-modal, always on top
    void DrawAlignPickWindow();       // GLOBAL align_station crosshair pick — non-modal, always on top
    // Alignment page (View menu, full-page): teach a local frame (O1) from two
    // operator-marked features. The capture itself runs through the engines, so
    // teaching and a scripted re-run take the identical code path.
    void DrawAlignPanel();            // page body: procedures | steps | frame
    void DrawAlignListPanel();        // left:   procedures + New/Duplicate/Delete/Save
    void DrawAlignDetailPanel();      // middle: header + step selector + one step
    void DrawAlignFramePanel();       // right:  the derived frame, as a record
    // The middle panel's four steps. The selector on its left says WHERE the
    // procedure stands (a dot per station: taught auto, taught manual, or not
    // taught); the pane on its right draws exactly ONE of the four, because all
    // four stacked is what pushed Run off the bottom of the page.
    void DrawAlignStepSelector(AlignProc& a);
    void DrawAlignStationCard(AlignProc& a, int station);
    void DrawAlignRunPanel(AlignProc& a);     // Run/Stop + the frame's actions
    void DrawAlignCheckPanel(AlignProc& a);   // the guard band + the run history
    void StartAlignmentRun(const AlignProc& a);   // launches alignment_procedure.lua
    // The repeatability sweep: alignment_repeat.lua, which runs the procedure
    // above `n` times through align_run. One engine Start for the whole sweep,
    // so orch_.Busy() stays true throughout and Stop ends it — a UI-side loop
    // that relaunched between iterations would go idle between them and let a
    // second run slip in, which is the bug the program runner's comment names.
    void StartAlignmentRepeat(const AlignProc& a, int n);
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
    // The camera's FOCUSED view of the nominal itself (its own private
    // position, `_c`), and the computed move that centres the camera on the
    // nominal so that pose can be taught in the first place. Separate from
    // the `_v` object view on purpose - see AlignNominal::camUid.
    const SavedPos* AlignNominalCamPos(const AlignProc& a, const AlignNominal& n) const;
    void TeachAlignNominalCamPos(AlignProc& a, AlignNominal& n);
    void GoToAlignNominalCamPos(const AlignProc& a, const AlignNominal& n);
    void DropAlignNominalCamPos(const AlignProc& a, AlignNominal& n);
    void CentreCameraOnNominal(const AlignProc& a, const AlignNominal& n);
    // Nominal teach page (a takeover, reached only from the Alignment page's
    // frame panel). Two crosshair marks on the live image; their MIDPOINT is
    // converted through the LIVE frame into local (O1) and stored on the
    // procedure, so it follows the part instead of the machine.
    // Under Run in the middle panel: the frame's two ACTIONS (probe a local
    // coordinate, drive to it) and the nominals that feed the probe. Separate
    // from DrawAlignFramePanel, which only reports what was taught.
    void DrawAlignFrameActions(AlignProc& a);
    void DrawAlignNominalList(AlignProc& a);
    // The local-frame map under the nominal pane: E1, E2, the O1 axes and every
    // nominal, `sel` highlighted, plus an optional ghost at (gx, gy) - the Tune
    // preview. Read-only; a click on a nominal selects it (alignNomSel_).
    void DrawAlignNominalMap(const AlignProc& a, const AlignNominal* sel,
                             bool ghost, double gx, double gy);
    void DrawAlignNominalPanel();                  // page body (takeover frame)
    void OpenAlignNominalTeach(const AlignProc& a, const std::string& nomUid);
    // Placement page: measure where a gripped object actually is (the same two
    // marks), and work out the correction that would put it on a nominal.
    // COMPUTE ONLY — nothing is wired to move yet.
    void DrawAlignPlacePanel();                    // page body (takeover frame)
    void OpenAlignPlace(const AlignProc& a, const std::string& nomUid);

    // Needle page (View menu, full-page): the dispense needle's lever arm — what
    // it currently is, and the button that measures it again. Deliberately a
    // teach-and-report surface with nothing that actuates: the calibration run
    // itself fires a shot, but it does so through the engines behind the
    // procedure's own prompts, not from a UI button.
    void DrawNeedlePanel();
    void StartNeedleCalRun();   // launches needle_calibration.lua

    // Registration page (View menu, full-page): where each device's tool tip is
    // in the camera device's space. Same shape as the Needle page above and for
    // the same reason — the record is machine state on Workflow, and this page
    // only shows it, injects the run parameters and launches the two STANDARD
    // procedures. It measures nothing itself.
    void DrawDeviceRegPanel();
    void StartDeviceRegCheckRun();   // launches device_reg_check.lua
    // The teach steps. They mutate machine state, not the document, and they
    // hand the result to Workflow's queue rather than writing the file — one
    // write path, drained in App::Update.
    //
    // Which of the three teachable poses is being set. The imprint pose is the
    // only one that also latches the rotations, because it is the pose the
    // measurement actually happens at.
    enum RegPoseSlot { kRegImprint = 0, kRegHome, kRegView };
    // The ONE writer of the three poses. Both ways of naming a pose — stand the
    // machine there and capture it, or adopt one the product already has taught
    // — land here, so neither can drift from the other.
    void StoreDeviceRegPose(RegPoseSlot slot, const RegVec3& p, const std::string& how);
    void CaptureDeviceRegPose(RegPoseSlot slot);            // from the live pose
    void AdoptDeviceRegPose(RegPoseSlot slot, const SavedPos& pos);
    void CompleteDeviceRegTip();

    // Gripped Object page (View menu, full-page): where the object in a
    // gripper's jaws sits, relative to that gripper's registered tip. The
    // measuring is a recipe step; this page exists to TEACH what a good grip
    // looks like and to SHOW the marks against the tip, which is the one thing
    // a log line cannot do.
    void DrawGripPanel();
    void StartGripMeasureRun(const GripProfile& g);   // launches grip_measure.lua

    // The Waffle Tray page (PageTray.cpp): a grid of positions taught from
    // three corners, with rows, columns and cells switched on and off. The
    // moving is a recipe step (move_to_cell); this page TEACHES the corners
    // and shows which cells a run has taken.
    void DrawTrayPanel();
    // The grid (status, All full / All empty, legend, a button per cell),
    // shared by the page and the floating window: one meaning per click.
    void DrawTrayGrid(WaffleTray* t, bool busy);
    // The floating Waffle Tray window (WndTray.cpp, View menu): the grid of
    // the selected tray beside whatever page is open, so an operator can pick
    // the ACTIVE cell a recipe's move_to_active_cell drives to, and mark
    // cavities used or full as parts come and go.
    void DrawTrayWindow();
    // The ONE writer of a corner, for "Teach here" (the live pose) and the
    // "or taught..." picker alike: both hand in COORDINATES. Stamps the date.
    void TeachTrayCorner(WaffleTray& t, TrayCornerSlot slot,
                         const std::map<std::string, double>& pose, const char* how);
    // Drive the tray's device to cell (r,c) as a watched move. Jog-armed and
    // idle only; a refusal lands in trayMsg_ (and the arm alert when unarmed).
    void GoToTrayCell(const WaffleTray& t, int r, int c);
    // The page's ONE writer of a tray's used cells: the Workflow cursor (what
    // tray_next reads) and the document (what survives a restart) together,
    // so they cannot disagree. `cells` empty = every cell.
    void SetTrayCellsUsed(const WaffleTray& t, const std::vector<TrayCell>& cells,
                          bool used, const char* why);

    // ---- Mosaic page (PageMosaic.cpp; docs/mosaic-design.md) ---------------
    // Large-area capture: an area taught by 3 or 4 corners, walked frame by
    // frame by mosaic_capture (a one-step recipe, like the quick autofocus),
    // and the result viewed, annotated and exported here. The area record is
    // MACHINE state (config/mosaic.local.json) and this page is its only writer.
    void DrawMosaicPanel();
    void DrawMosaicSetup();                // left: folder, areas, corners, plan, capture
    void DrawMosaicView();                 // centre: the picture (pan/zoom/annotate)
    void DrawMosaicSide();                 // right: markers, labels, exports
    // App::Update's share: the capture's start/tile/end events, the worker's
    // decoded tiles (GL uploads, a few per frame), texture paging, exports.
    void UpdateMosaic();
    void OpenMosaic(const std::string& dir, bool live);
    void CloseMosaic();
    void ReleaseMosaicTextures();
    void SaveMosaicCfg();                  // the one writer of config/mosaic.local.json
    void SaveMosaicAnnotationsNow();       // the one writer of annotations.json
    void StartMosaicCapture(const MosaicArea& a);
    // mosaic_calibrate over whatever is under the camera now (Admin: it
    // teaches the machine how its camera sits).
    void StartMosaicCalibrate();
    // A measured camera map becomes a PROPOSAL, never the map in use: the page
    // shows it beside the current one and an Admin applies or discards it.
    // Both measurements land here - the stepping calibration and the fit to a
    // mosaic's seams.
    void ProposeMosaicCal(const double m[4], const std::string& from, const std::string& note);
    // The plan the capture would walk right now: the live camera frame size and
    // pixel map, the calibration if there is one. The page's summary and the
    // view's preview both call this, so they cannot show two different plans.
    MosaicPlan MosaicPlanFor(const MosaicArea& a);
    // Drive the gantry so the camera centre lands on canvas point (cx, cy) of
    // the open mosaic, at the Z its nearest tile was shot at. Jog-armed, idle.
    void GoToMosaicPoint(double cx, double cy, const std::string& what);
    // Drive the gantry back to the pose tile `i` of the open mosaic was SHOT
    // at (its measured X/Y/Z), so the live camera shows what that frame shows.
    // Same gates as GoToMosaicPoint: kRoleJog, jog armed, idle.
    void GoToMosaicTile(int i);
    // The private pose sweep, by the convention the alignment procedures set:
    // the position is named after its owner, so deleting the owner drops it.
    void DropGripPositions(const GripProfile& g);
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
    // Creates the station's tier-`tier` model (1..kAlignDetectTiers), switches page.
    void OpenVisionTeach(AlignProc& a, int station, int tier = 1);
    void ResetVisionTeachState();                        // both openers start here
    const VisionModel* StationDetector(const AlignProc& a, int station, int tier = 1) const;
    void DropAlignStationDetectors(const AlignProc& a);  // with the procedure
    void CopyAlignStationDetectors(const AlignProc& src, AlignProc& dst);
    // The same page and the same private-ownership rule, for the two marks that
    // locate an OBJECT being placed on a nominal: `vdet_<procUid>_<nomUid>_p<N>`.
    // Teaching these is what lets a placement run without an operator; a
    // detector that fails still falls back to asking, loudly.
    void OpenVisionTeachForNominal(AlignProc& a, AlignNominal& n, int index);
    const VisionModel* NominalDetector(const AlignProc& a, const AlignNominal& n,
                                       int index) const;
    // The same three, one owner further out: a gripped-object profile's private
    // detector. Auto-named vdet_grip_<uid>, so the name is the ownership record
    // exactly as it is for a station and for a nominal's object points.
    const VisionModel* GripDetector(const GripProfile& g) const;
    void OpenVisionTeachForGrip(GripProfile& g);
    void DropGripDetector(const GripProfile& g);
    void CopyGripDetector(const GripProfile& src, GripProfile& dst);
    void DropNominalDetectors(const AlignProc& a, const AlignNominal& n);
    // Copy a model under a new private name, patch file included; "" on failure.
    std::string CopyVisionModelAs(const VisionModel& src, const std::string& newName);
    // Detection clipboard: Copy takes a model's SETTINGS (method, parameters,
    // pre-process chain, ROI, exposure) plus its patch bytes into memory; Paste
    // writes them into a station tier's OWN private model (created if absent).
    // It never shares a row or a .pgm - the ownership rule stands - and the
    // pasted model's reliability record is cleared: it was earned somewhere else.
    void CopyDetectionToClip(const VisionModel& m, const std::string& from);
    void PasteDetectionToStation(AlignProc& a, int station, int tier);
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
    // Scripts page (View menu, full-page): list | editor | run info, plus the
    // Lua debugger controls. The editor (design-doc item C4) is an InputText
    // under a syntax-colour overlay — see ui/ScriptEditor.h.
    void DrawScriptsPanel();          // page body: scripts | editor | run info
    void LoadScriptsList();           // session rescan + Scripts page selection sync
    void SyncScriptSelection();       // keep the selection on the same FILE across a rescan
    void SelectScript(int idx);       // list click: dirty guard, then load the file
    void LoadScriptEditor(const std::string& path);   // file -> buffer (drops the old one)
    bool SaveScriptEditor();          // buffer -> file, then rescan (description may change)
    void RevertScriptEditor();        // buffer <- last saved text
    void LexScript();                 // tokens + lint + line table, once per text change
    void VerifyScript();              // CheckLuaSyntax on the buffer, now
    void BuildScriptFdTable();        // completion candidates: registry + LuaFdBuiltins
    void JumpToScriptLine(int line, int col, int len);   // select + scroll + focus
    void DrawScriptEditor(bool thisRunning, bool paused, int execLine);   // the editor child
    void DrawScriptCompletionPopup(); // the tooltip-layer candidate list under the caret
    void DrawScriptModals();          // unsaved-changes guard + New Script
    bool ScriptDirty() const { return !scriptEd_.path.empty() && scriptEd_.text != scriptEd_.saved; }
    // DUT Records viewer (View menu, full-page, read-only) — Phase D2.
    void DrawDutViewPanel();          // serial list | entries table | summary
    void LoadDutList();               // scan the product's data/ for records
    // View Data window (View menu, read-only): every record of the loaded
    // product as ONE table, a row per serial and a column per recorded key,
    // plus a chart of one key across the parts. WndViewData.cpp.
    void DrawViewDataWindow();
    void DrawPositionHistoryWindow();   // the positions change journal + Restore
    void LoadDataRows();              // scan + flatten every dut.json under data/
    void SortDataRows();              // by dataSortCol_/dataSortAsc_ (header click, reload)
    // Run page (View menu, full-page) — programs = slots of recipes. R1.
    void DrawRunPanel();              // deck | execution + Scan band + log | DUT
    void StartDeckRun(RunScope scope);   // granularity follows the deck selection
    void SelectDeckProgram(int idx);     // load a program into the deck (slot 0)
    void SelectDeckSlot(int slot);       // show one slot; drops the node selection
    void RememberDeckSelection();        // persist program uid + slot to UiSettings
    void RestoreDeckSelection();         // re-point the deck after a document load
    // The deck tree, read-only. `startUid`/`endUid` are the cycle captions and
    // arrive EMPTY when the cycle does not resolve, so an invalid one cannot
    // label a row it no longer describes. Design: docs/deck-cycle-design.md.
    void DrawDeckNodes(std::vector<ProgramNode>& items, float rowH, bool running,
                       const std::string& runningNodeUid,
                       const std::string& startUid, const std::string& endUid);
    // ---- Deck cycle: start/end states + per-node pass/fail accents ----
    // A node's last outcome for THIS part. Runtime only, never persisted: a
    // green tick describes one physical part, exactly like a live frame.
    enum class NodeMark { None, Passed, Failed };
    void DrainDeckMarks();     // runner item marks -> deckMarks_ (from App::Update)
    void AdvanceDeckCycle();   // on the run-finished edge: advance, or hold on a fail
    // The cycle as two TOP-LEVEL indices. False (and nothing written) when
    // either uid is empty, does not resolve at the top level, or end sits
    // above start — an invalid cycle is undefined, never half-defined.
    bool DeckCycleRange(const ProgramSlot& slot, int* startIdx, int* endIdx) const;
    // A folder DERIVES its mark from its leaves (only leaves become queue
    // items), so it can never disagree with what it contains.
    NodeMark DeckMarkOf(const ProgramNode& n) const;
    // ---- Cycle values: what the part's recipes wrote into their run ctx ----
    // Recipe ctx does NOT flow between recipes, so this is the UI collecting
    // each finished run's final snapshot into one list for the part.
    void DrainCycleVars();          // engine-idle edge -> cycleVars_ (App::Update)
    void DrawRunValuesPanel();      // the Run page's right-column "Values" list
    void DrawRunReadouts();         // the pinned instrument readouts under it
    static bool IsOperatorCtxKey(const std::string& key);   // hides engine scratch
    // True for a key under a prefix a COMMAND or standard script writes by
    // default (the registry's ctx_prefix defaults + the scripts' families).
    // A heuristic by name: a user who points ctx_prefix at their own name gets
    // a user key; one who reuses a default prefix gets a system one.
    bool IsSystemCtxKey(const std::string& key);
    // Program Organizer (its own window, opened by the Run strip's Edit box):
    // recipe library | 1st slot tree | aisle | 2nd slot tree. The ONLY
    // program-editing surface. `panel` is 0 (1st) or 1 (2nd) throughout: each
    // panel shows a slot of its own with a selection of its own, and the aisle
    // moves or copies a selection across.
    void DrawProgramOrganizer();
    // One panel: toolbar, folder knobs, cycle line, the tree, the deferred drop.
    // `other` is the opposite panel's slot (null when hidden) - a drag from
    // there lands here as a MOVE.
    void DrawOrgPanel(ProgramSlot& slot, ProgramSlot* other, int panel);
    // `topLevel` gates the cycle context menu: only a top-level node can be a
    // cycle state, because the folder is what Execute launches.
    void DrawOrgTree(ProgramSlot& slot, std::vector<ProgramNode>& items, bool topLevel,
                     int panel);
    void DrawOrgNodeDropTarget(const std::string& anchorUid, bool intoFolder, int panel);
    // The deferred drop, after the tree draw. A node uid not in `slot` is
    // looked for in `other` and moved across.
    void ApplyOrgDrop(ProgramSlot& slot, ProgramSlot* other, int panel);
    void OrgAddRecipe(const std::string& recipeUid);   // library -> 1st panel, at its selection
    void OrgAddFolder(ProgramSlot& slot, int panel);
    // Land a node at a panel's selection: inside a selected folder (unless
    // `besideFolder`), after a selected row, else at the slot's end. The new
    // node becomes the selection.
    void OrgPlace(ProgramSlot& slot, int panel, ProgramNode n, bool besideFolder = false);
    std::string& OrgSel(int panel) { return panel == 0 ? orgSel_ : orgSel2_; }
    void DrawRunScanBlock();          // shared channel/Target/Trace header + Raster|Climb|XCAN tabs
    void DrawRunXcanBlock();          // XCAN tab: hex-left/right launch presets
    void DrawRunQuickAlign();         // operator band: 5 one-press target+ladder launches
    void DrawRunShotPanel();          // the image box above the DUT block
    void DrainCapturedView();         // worker -> UI: the captured still -> a texture
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
    // The registered devices' tool tips, projected onto the live image. Each
    // tip is marked by its NAME and edge arrow only; its dZ and the "no tip:"
    // lines stack up from the bottom-right corner, `bottomInset` above it - the
    // grip readout's plate sits in that corner. Returns the height it used, so
    // the cursor readout can stack above it.
    float DrawAuxGripperMarks(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail,
                              const ImVec2& center, float scale, float bottomInset);
    // The last grip measurement (auxGrip_): the object's cross on the image
    // and a plate in the bottom-right corner. Returns the plate's height so
    // the "no tip:" lines can stack above it; 0 when there is nothing shown.
    float DrawAuxGripResult(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail);
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

    // The Run page's image box: ONE still, captured by a recipe step and held
    // until the next one replaces it. Its own texture rather than a share of
    // cameraTex_ — that one streams, and the whole point of this box is a
    // picture that STOPS while the stage drives on to the next station.
    //
    // Filled only by DrainCapturedView (App::Update), from the Workflow slot a
    // worker parked it in. Nothing here is persisted: a photograph of one
    // physical part means nothing at the next boot, exactly like a taught frame
    // or a deck mark.
    unsigned int runShotTex_ = 0;
    int          runShotW_ = 0, runShotH_ = 0;   // the FRAME's size in pixels
    std::string  runShotLabel_;   // caption the step asked for
    std::string  runShotStamp_;   // when it was taken
    std::string  runShotPath_;    // record-relative path, "" if display-only
    // The BOX's height on screen, dragged by the splitter under it and
    // persisted like every other panel size (0 = seed on the first frame). A
    // fixed height would be wrong for the one thing this box is for: how tall
    // it has to be to read a serial depends on the lens, not on the layout.
    float        runShotPanelH_ = 0.0f;

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
    // The page drives ONE supply of the fleet at a time (psuSel_, a fleet
    // index, clamped by PsuSelected); the edits re-seed when it changes.
    int   psuSel_     = 0;
    std::size_t PsuSelected();
    float psuVSet_[3] = {5.0f, 5.0f, 3.3f};      // per-channel voltage edits (V)
    float psuISet_[3] = {1.0f, 1.0f, 1.0f};      // per-channel current-limit edits (A)
    bool  psuPulled_  = false;
    // Per-supply, per-channel CV/CC choice (0 = CV, the default; 1 = CC). The
    // supply has no mode switch - the load decides - so this only says which
    // number is the TARGET and which the LIMIT, and which reading the page
    // holds it to. Keyed by PSU name.
    std::map<std::string, std::array<int, 3>> psuModes_;

    // Light page state. The per-channel sliders follow the controller's
    // read-back whenever they are not being dragged; indexed by front-panel
    // channel.
    std::string lightId_;                        // config instrument name
    float       lightPct_[9]   = {};             // per-channel edits (%), [1..8]
    std::string lightSel_;                       // light the call-out buttons drive
    float       lightCallPct_  = 50.0f;          // call-out intensity (%)
    std::string lightErr_;                       // last call-out refusal, shown under the buttons

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

    // ---- Users & access ---------------------------------------------------
    // The roster is UI-thread-only state, like the session document: it is read
    // once at startup and rewritten only by the User Manager. `user_` with role
    // None is what raises the login gate, so it is the single source of truth
    // for "is anybody here" — never mirror it into a second bool.
    std::vector<Employee> employees_;
    ActiveUser            user_;

    // Login gate fields. loginErr_ says only "employee number or password is
    // wrong": naming which half was right tells a guesser which half to keep
    // trying, and the operator who mistyped learns nothing useful from the
    // distinction anyway.
    char        loginId_[64] = "";
    char        loginPw_[64] = "";
    std::string loginErr_;

    // User Manager window (admin only). usrSel_ indexes employees_ and is
    // re-clamped after every add/remove — a stale index into a shrunk roster
    // would reset the password of whoever moved up into the slot.
    bool        showUsers_ = false;
    char        usrId_[64]   = "";
    char        usrName_[64] = "";
    char        usrPw_[64]   = "";
    int         usrLevel_    = 0;    // 0 Operator, 1 Technician, 2 Admin
    int         usrSel_      = -1;
    bool        usrArmRemove_ = false;   // the armed checkbox in front of Remove
    std::string usrErr_, usrNote_;

    // Main-window page + which motion device is selected on the Motion page.
    // prevPage_ = the hardware page to return to when an editor page closes.
    Page currentPage_    = Page::Overview;
    Page prevPage_       = Page::Overview;
    int  selectedMotion_ = -1;   // absolute index into the HardwareManager motion fleet
    bool showHiddenPos_  = false;   // reveal hidden (system) positions in the table

    // Pivot point editor (Motion page, PI hexapods). The fields re-seed from
    // the controller's cached pivot whenever the device or its read changes
    // (PivotPoint::seq), so typing survives until the next Read / Write.
    double      pivotEdit_[3] = {0.0, 0.0, 0.0};
    int         pivotEditDev_ = -1;
    unsigned    pivotEditSeq_ = 0;
    std::string pivotMsg_;           // last write's outcome, shown under the editor
    bool        pivotMsgErr_  = false;

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
    std::string mapSelNode_;     // click-selected node (not in edge mode): the
                                 // canvas side panel offers a path or a direct
                                 // move to it
    bool        openMapDirect_ = false;   // arm the direct-move confirm modal
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

    // The Positions tab's selected row, by uid: the toolbar above the table
    // acts on it (Move / Update / Edit / Rename / Hist / Delete) instead of a
    // button per row. A uid rather than an index, so a device switch, a
    // delete or a reorder cannot land the action on a different row.
    std::string posSelUid_;

    // Rename-position dialog (name is a label; the uid never changes).
    bool openRenamePos_ = false;    // row Rename button -> popup handoff
    int  renameRow_     = -1;       // row index in the selected device's list
    char renameBuf_[32] = "";

    // New-position dialog ("+ Add current" asks for the name before saving —
    // an auto "P24" is a row nobody can identify a week later). The live
    // position is captured when OK is pressed, not when the dialog opens.
    bool openNewPos_    = false;
    char newPosBuf_[32] = "";

    // Delete-position confirmation dialog (X button asks before erasing).
    bool openDeletePos_ = false;
    int  deleteRow_     = -1;

    // Move-to-position confirmation (row Move asks before the axes go: the
    // table is dense and a mis-click sends the device somewhere else).
    bool openMovePos_ = false;
    int  moveRow_     = -1;

    // Update-position confirmation (row Upd shows saved vs live per axis and
    // asks; a delta over kPosUpdWarnMm on any axis needs a SECOND press —
    // a taught place overwritten from the wrong pose is the costliest slip
    // this table can make).
    bool openUpdPos_ = false;
    int  updRow_     = -1;
    bool updSecond_  = false;       // first press acknowledged the large delta

    // Edit-position dialog (type the axis values directly and save).
    bool                          openEditPos_ = false;
    int                           editRow_     = -1;
    std::map<std::string, double> editVals_;

    // Teach-position handoff: a PositionUid form row raises the flag, the
    // Recipe Editor panel opens the modal. A flag rather than a direct
    // OpenPopup because the row draws inside the inspector's child window.
    bool        openTeachPos_ = false;
    std::string teachPosDev_;    // motion device the step names
    std::string teachPosUid_;    // the saved position to overwrite in place

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

    // Jog tab "Tool -> cam" / "Cam -> tool": the registered move_tool_to_camera
    // and move_camera_to_tool commands, started as a one-step ad-hoc recipe
    // (StartQuickRegMove) exactly like the autofocus button - so every refusal
    // (no registration, rotation guard, unverified, ceiling) is the command's,
    // never a second copy here. The Z offset defaults to the command's own -2
    // (a stand-off on this machine), not the Registration page's 0 (a touch):
    // a one-click button in a jog window must not default to contact.
    std::string jogRegTool_;             // chosen tool when the camera device is selected
    double      jogRegZOff_    = -2.0;   // move_tool_to_camera z_offset_mm
    bool        jogRegUnver_   = false;  // move_tool_to_camera allow_unverified
    std::string regMoveErr_;             // the last start rejection
    void DrawJogRegRow(const std::string& selected);
    void StartQuickRegMove(bool toolToCam, const std::string& tool);

    // Window visibility (View menu) + jog arming.
    bool showAux_  = true;
    bool showJog_  = true;
    bool jogArmed_ = false;   // jog buttons + W/D·A/S·R/F keys active only when armed

    // Jog panel (Overview right panel). Transforms + commands live in Workflow.
    int jogDevice_    = -1;  // index into the motion fleet
    int jogStepIndex_ = 9;   // index into kSteps (default 100um)
    int jogRotAxis_   = 0;   // selected rotational axis (0 = first beyond XYZ)
    // The Jog box's position row: the taught position the device is sitting
    // on, and a HOLD on it. Checking the box while at a position keeps its
    // name while the operator jogs away; Upd then writes the live pose into
    // that record and drops the hold. The hold is a uid (a rename survives it)
    // plus the device it was taken on (switching device drops it) — never an
    // index, and never persisted: it describes an edit in progress.
    std::string jogHoldUid_;
    std::string jogHoldDev_;
    std::string jogPosStatus_;       // last Upd, one line for the box
    std::string jogPosStatusFull_;   // the same with from/to, for its tooltip
    double      jogPosStatusUntil_ = 0.0;   // ImGui time the notice leaves the box
    static constexpr double kJogPosStatusSec = 6.0;
    // Upd does not write on the click: it snapshots before/after per axis
    // here and raises the "Update Position" modal, which shows the table and
    // writes on confirm. The snapshot is what gets written — not a fresh
    // read at confirm time — so the numbers the operator approved are the
    // numbers that land.
    struct JogPosUpdate {
        std::string dev, uid, name;
        struct Row { std::string axis; double before, after; };
        std::vector<Row> rows;
        double deltaMM = 0.0;   // XYZ distance
    };
    JogPosUpdate jogUpd_;
    bool jogOpenUpdPos_ = false;   // row -> panel-level OpenPopup handoff
    void DrawJogPositionRow(IMotionController& dev, const std::vector<std::string>& axes);
    void DrawJogUpdatePosModal();

    // Jog window "Pos" tab: device name -> nine saved-position uids ("" = empty
    // slot). Persisted through UiSettings, keyed by name so a fleet reorder
    // does not shuffle the pad. A uid the loaded product does not contain is
    // NOT cleared — the operator switched product, they did not unassign — the
    // button just greys out until that product is back.
    std::map<std::string, std::vector<std::string>> posPad_;
    // Jog window "IO" tab: the quick-output rows (UiSettings quick_io). A row
    // names device + pin, never a fleet index - see QuickIoRow.
    std::vector<QuickIoRow> quickIo_;

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
    // Further selected rows beyond edStepSel_ (the ANCHOR: the inspector's
    // step, the palette's insert point). Ctrl-click toggles a row in and out,
    // Shift-click takes the visible range from the anchor. Copy and Delete
    // act on anchor + these; the single-row moves (^ v > <) are disabled while
    // any exist. Cleared by every plain click and every edit — a path is an
    // index into the tree and means nothing once the tree changes under it.
    std::vector<std::vector<std::size_t>> edStepMulti_;
    // A Shift-click is resolved AFTER the tree has drawn (edTreeRows_ is
    // complete only then): the clicked row waits here for one frame.
    std::vector<std::size_t> edShiftTo_;
    bool edShiftPending_ = false;
    void ClearStepSel() { edStepSel_.clear(); edStepMulti_.clear(); }
    void ClickStepRow(const std::vector<std::size_t>& path);   // plain/Ctrl/Shift
    // Anchor + multi in tree order, with any row whose ancestor is also selected
    // dropped (a container carries its subtree; both would copy it twice).
    std::vector<std::vector<std::size_t>> SelectedStepPaths() const;
    // Insert at the palette's target (after the anchor / into it / its else
    // branch / root when nothing is selected) and select what went in.
    void InsertStepsAtSelection(Recipe& r, std::vector<RecipeStep> steps);
    void CopySelectedSteps(const Recipe& r);   // JSON {"steps":[...]} to the OS clipboard
    void PasteSteps(Recipe& r);                // accepts that, a recipe, an array, one step
    // Remove the selection (subtrees included) and clear it; the Delete
    // modal's and Cut's one implementation. Returns how many rows went.
    std::size_t DeleteSelectedSteps(Recipe& r);
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
    // Width of the editor's command palette column, dragged by the splitter
    // between the palette and the step tree. Persisted like the inspector: a
    // long command name wants it wide, a long step list wants it narrow.
    float edPaletteW_ = 300.0f;
    bool  edPaletteSplitting_ = false;
    bool edOpenRenameRecipe_  = false;
    bool edOpenDeleteRecipe_  = false;
    bool edOpenDeleteStep_    = false;
    bool edOpenTagRecipe_     = false;
    char edNameBuf_[64]       = "";
    char edSearchBuf_[64]     = "";      // palette search filter
    char edTagBuf_[160]       = "";      // Tag modal: comma-separated tags
    // The recipe list's filter (persisted in UiSettings, see there). A recipe
    // that fails it still draws while SELECTED, dimmed: the editor beside the
    // list is showing it, and a list that cannot say which one would lie.
    bool        edShowHidden_ = false;
    std::string edTagFilter_;
    bool        edTagHide_    = false;
    bool RecipeListed(const Recipe& r) const;   // passes the filter above

    // Alignment page UI state — the procedures themselves live in session_.
    int  alignSel_      = -1;    // selected procedure
    char alignNameBuf_[48] = "";  // rename field for the selected procedure
    double alignProbeX_ = 0.0;   // right panel's local->machine check inputs
    double alignProbeY_ = 0.0;
    std::string alignStartErr_;  // why the last Run refused to start
    // Which step the middle panel's swappable area is showing. An index into a
    // fixed set of panes (kAlignViewE1 .. kAlignViewCheck, PageAlign.cpp), not a
    // document reference, so it needs no uid. Reset to E1 on every change of
    // selected procedure — a pane left on "Run" while a different, untaught
    // procedure is selected reads as a licence the new one has not earned.
    int alignView_ = 0;
    // How many runs the Frame check pane's repeatability sweep asks for, and
    // the window the Cpk beneath it is computed over. One number for both: the
    // question the sweep answers is "what did THOSE runs do", and a capability
    // figure computed over a different span than the sweep is a different claim.
    int alignRepeatN_ = kAlignRepeatDefault;
    // Which nominal the nominal pane is showing. A UID, never an index: the
    // vector is erased from and appended to while the pane is open, and an
    // index would silently start pointing at a different point. Empty = show
    // the first one.
    std::string alignNomSel_;
    // The nominal pane's inline rename. Inline and not a popup: this layer
    // raises no modal (a run may be blocked on the always-on-top prompt).
    // `alignNomRenameUid_` empty = not renaming.
    std::string alignNomRenameUid_;
    char        alignNomRenameBuf_[48] = "";
    std::string alignNomRenameErr_;
    // The nominal pane's Tune: edit lx/ly by hand, previewed on the map until
    // OK. `alignNomTuneUid_` empty = not tuning. Mode 0 = absolute (the boxes
    // ARE the new lx/ly), 1 = offset (the boxes are added to the current ones).
    std::string alignNomTuneUid_;
    int         alignNomTuneMode_ = 0;
    double      alignNomTuneX_ = 0.0, alignNomTuneY_ = 0.0;

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

    // Registration page state. The registrations themselves live on Workflow
    // (machine state, not a document) — these are the selection, the four-step
    // teach in progress, and the check's parameters.
    //
    // The teach is a PAGE flow rather than a procedure script: every step needs
    // an operator at the machine placing a coupon, jogging a tool into contact,
    // and marking the corners of the mark it left. There is nothing for a recipe
    // to automate, and a second way to create a registration would be a second
    // answer to where the tool is.
    char   regDevice_[32]   = "hex-left";
    char   regRef_[32]      = "gantry-main";
    double regFocusRangeMm_ = 0.5;
    // The four corner marks of the imprint, in MACHINE mm, each latched with the
    // pose it was taken at — so the operator may jog between them and a mark
    // still means the same place. Session state, never persisted: they describe
    // one mark on one coupon, and the answer they produce is what is kept.
    static constexpr int kRegMarkMax = 4;
    AlignPoint regMark_[kRegMarkMax];
    bool       regMarkHave_[kRegMarkMax] = {false, false, false, false};
    int        regMarkCount_  = 4;    // 2 must be a true diameter; 4 is forgiving
    int        regMarkActive_ = 0;    // which corner the next click lands on
    float      regPx_ = 0.0f, regPy_ = 0.0f, regZoom_ = 1.0f;   // MarkPicker state
    // Width of the right-hand control column, dragged by the splitter and
    // persisted in UiSettings. Its own field rather than the main window's
    // rightW_: this page is a takeover that replaces the five-region layout.
    // The four steps carry long explanatory text and the operator's font size
    // moves all of it, so no constant width is right on every station.
    float      regCtlW_ = 430.0f;
    bool       regCtlSplitting_ = false;   // edge-detect the drag END
    // The footprint box on the page's image, as the MarkPicker edits it: seeded
    // from the record every frame it is not being dragged, written back through
    // QueueDeviceReg when a corner drag ends. Floats because the widget takes
    // floats; the record is the double that matters.
    float regBoxW_ = 0.0f, regBoxH_ = 0.0f;
    bool  regBoxDragging_ = false;
    // Check parameters. The three offsets default non-zero on purpose: a check
    // at the pose the tip was taught at returns the measurement by construction
    // and can verify nothing, so the default has to be a check that tests it.
    double regCheckDx_  = 5.0, regCheckDy_ = 5.0, regCheckDz_ = 0.0;
    double regCheckTol_ = 0.05;
    double regVTravel_  = 0.0;
    double regVJog_     = 0.0;
    int    regSettleMs_ = 300;
    // Off by default. The check measures Z as "the gantry Z that focuses on the
    // tip", and the taught tip Z came from an operator focusing by hand: run the
    // two halves through different focus methods and the residual reports the
    // disagreement between the METHODS, which is not what is being checked.
    // Off, the pick window shows the live dZ and the operator closes it by hand.
    bool   regAutofocus_ = false;
    // "Send the tool here": the consuming end of the registration, tried from
    // the page that taught it. The Z offset is signed and in the REFERENCE
    // device's axis Z, because which way that counts is a property of the
    // machine (gantry-main's transform inverts Z) and this page must not
    // pretend to know it. maxMm refuses the lunge a stale record turns a short
    // approach into; allowUnverified is the same opt-in move_to_point demands.
    double regSendZOff_    = 0.0;
    double regSendMaxMm_   = 25.0;
    bool   regSendUnver_   = false;
    // "Look at the tool": the same record read backwards - the camera driven to
    // where the tip is predicted to be. Its own ceiling because it is a
    // different trip (across the workspace to a parked tool, not the last few mm
    // onto a point), and Z opt-in because the tip's reference Z is the height
    // that FOCUSES on it, which is the camera descending toward a tool whose
    // height nothing here knows.
    bool   regLookZ_       = false;
    double regLookMaxMm_   = 60.0;
    std::string regStartErr_;   // why the last Check refused to start
    std::string regTeachMsg_;   // what the last teach step did, or would not do

    // ---- Gripped Object page ------------------------------------------------
    // The profile list is the document's (session_.Grips()); everything here is
    // the page's own view of it, per the rule that page state lives on App.
    int   gripSel_ = -1;              // index into session_.Grips()
    float gripCtlW_ = 430.0f;         // control column width, persisted
    bool  gripCtlSplitting_ = false;  // edge-detect the drag END
    // The two marks, in MACHINE mm, each latched with the pose it was taken at
    // so the operator may jog between them. Session state, never persisted -
    // only the computed answer is worth keeping, and a mark describes one pick
    // of one part. The same bargain the registration's corner marks make.
    static constexpr int kGripMarkMax = 2;
    AlignPoint gripMark_[kGripMarkMax];
    bool       gripMarkHave_[kGripMarkMax] = {false, false};
    int        gripMarkActive_ = 0;
    float      gripPx_ = 0.0f, gripPy_ = 0.0f, gripZoom_ = 1.5f;
    // Whether the "Camera to tool" button commands Z as well. Off by default,
    // for the reason move_camera_to_tool's axes defaults to XY: that is the leg
    // that drives the lens toward a tool of unknown height.
    bool   gripFocusZ_ = false;
    // Manual offset for the camera's Z on that move, added to the tip's focused
    // height: for an object that sits above or below the tip. Non-zero commands
    // Z on its own, so a typed number never silently does nothing.
    double gripZOffsetMm_ = 0.0;
    // "Camera to tool" was clicked with jog unarmed: raise the alert modal on
    // the next draw. A flag, not an OpenPopup in the lambda, so the popup is
    // opened and drawn in one ID scope. Safe as a modal: the button that sets
    // it is disabled while orch_.Busy(), so no worker is behind a prompt.
    bool   gripOpenArmPopup_ = false;
    // What the page's Measure button injects. They are RUN options, not
    // properties of the object, which is why they live here and not in the
    // profile: a recipe step carries its own.
    bool   gripRunInspect_ = true;
    bool   gripRunLook_    = true;
    bool   gripAutofocus_  = false;
    double gripFocusRangeMm_ = 0.5;
    std::string gripStartErr_;   // why the last Measure refused to start
    std::string gripTeachMsg_;   // what the last teach step did, or would not do

    // ---- Waffle Tray page ----------------------------------------------------
    // The tray list is the document's (session_.Trays()); this is the page's
    // view of it. The consumed-cell shading is read off Workflow each frame.
    int   traySel_ = -1;              // index into session_.Trays()
    float trayCtlW_ = 430.0f;         // control column width, persisted
    bool  trayCtlSplitting_ = false;  // edge-detect the drag END
    bool  trayOpenArmPopup_ = false;  // a Go clicked with jog unarmed: raise the alert
    int   trayLineRow_ = 1, trayLineCol_ = 1;   // the line toggles' spinners (big trays)
    bool  showTray_ = false;          // the floating Waffle Tray window (View menu)
    std::string trayMsg_;             // what the last button did, or would not do

    // ---- Mosaic page ---------------------------------------------------------
    MosaicConfig mosaicCfg_;               // loaded at startup; SaveMosaicCfg writes it
    int          mosaicAreaSel_ = -1;      // index into mosaicCfg_.areas
    char         mosaicDirBuf_[260] = "";  // the save-folder edit
    std::string  mosaicMsg_;               // what the last button did, or would not do
    bool         mosaicOpenArmPopup_ = false;
    float        mosaicLeftW_  = 400.0f;   // column widths, persisted
    float        mosaicRightW_ = 340.0f;
    bool         mosaicSplitL_ = false, mosaicSplitR_ = false;   // drag-END edges
    std::vector<std::string> mosaicFolders_;   // mosaics in the save folder
    bool         mosaicFoldersStale_ = true;
    // The open mosaic: a folder, its capture record and its annotations.
    // Nothing here is persisted but the annotations file itself.
    std::string        mosaicDir_;
    MosaicRecord       mosaicRec_;
    bool               mosaicHaveRec_ = false;
    MosaicAnnotations  mosaicAnn_;
    bool               mosaicLive_ = false;       // the running capture is writing it
    int                mosaicLiveTotal_ = 0;      // tiles in the running capture's plan
    unsigned           mosaicGen_ = 0;            // bumps per open: stale loads are dropped
    std::vector<MosaicTileTex> mosaicTex_;        // index-aligned with mosaicRec_.tiles
    unsigned long long mosaicFrame_ = 0;          // draw counter for the texture LRU
    MosaicWorker       mosaicWorker_;             // decodes + exports off the UI thread
    unsigned           mosaicExportSeen_ = 0;     // last Progress::done reported
    std::string        mosaicExportMsg_;
    std::string        mosaicExportPath_;
    // The view: canvas point at the centre, and screen px per canvas px
    // (0 = fit on the next frame).
    double mosaicVcx_ = 0.0, mosaicVcy_ = 0.0, mosaicVs_ = 0.0;
    int    mosaicTool_ = 0;                 // 0 pan, 1 marker, 2 crop label
    bool   mosaicShowGrid_ = false;         // tile outlines
    bool   mosaicShowOutline_ = true;       // the area's outline
    bool   mosaicDragging_ = false;         // a crop label being dragged out
    double mosaicDragX0_ = 0.0, mosaicDragY0_ = 0.0;
    int    mosaicSelMarker_ = -1, mosaicSelLabel_ = -1;
    int    mosaicSelTile_ = -1;             // index into mosaicRec_.tiles (Tiles mode)
    // The annotation filter (right panel): a name search and one tag. Runtime
    // only; it also picks which crop labels Export crops writes.
    char        mosaicAnnFind_[64] = "";
    std::string mosaicAnnTag_;              // "" = every tag
    int    mosaicExportScale_ = 2;          // index into {1, 2, 4, 8}
    bool   mosaicUseRefined_ = true;        // draw/export at placement.json's positions
    double mosaicCalStepMm_ = 0.5;          // mosaic_calibrate's step
    // The camera map waiting for review. Runtime only: nothing reads it until
    // Apply copies it into mosaicCfg_.cal.
    bool        mosaicCalPendingHave_ = false;
    double      mosaicCalPendingM_[4] = {0.0, 0.0, 0.0, 0.0};
    std::string mosaicCalPendingFrom_;
    std::string mosaicCalPendingNote_;
    std::string mosaicFitDir_;              // the mosaic a running map fit measures

    // Nominal teach page state — the nominals themselves live in session_.
    // Page::AlignNominal IS the "open" flag; nomProcUid_ says which procedure.
    std::string nomProcUid_;              // procedure being taught against
    std::string nomEditUid_;              // nominal being re-taught ("" = a new one)
    char        nomNameBuf_[48] = "";
    // The marks, in MACHINE mm, each latched with the gantry pose it was taken
    // at — so the operator may jog between them and a mark still means the same
    // physical place. Drawn by projecting back through the live pose. 1, 2 or 3
    // of them, and the nominal is their average.
    static constexpr int kNomMarkMax = kAlignNominalTeachMax;
    AlignPoint  nomPt_[kNomMarkMax];
    bool        nomHave_[kNomMarkMax] = {false, false, false};
    int         nomCount_   = 1;          // 1..3 - how many marks are asked for
    int         nomActive_  = 0;          // which mark "Set" will write
    float       nomPx_ = 0.0f, nomPy_ = 0.0f;   // crosshair, camera px
    float       nomZoom_ = 1.5f;          // same default as the pick window
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
    // Height of the camera strip under the image (exposure, lighting, zoom),
    // dragged by a horizontal splitter and persisted: the image takes the rest,
    // so the operator trades picture for controls rather than losing either.
    float       visionFootH_ = 0.0f;              // 0 = fit the strip's rows
    bool        visionFootSplitting_ = false;
    // A detection tier's lighting was edited on the Alignment page and is not
    // saved yet; flushed by the page once no widget is active.
    bool        alignLightDirty_ = false;
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
    // Where each registered device's TOOL TIP is, drawn on the Aux image.
    // A readout like the two above, and the same rule as the tip topics: a
    // tip that cannot be predicted draws NOTHING, never a stale cross.
    bool       showGripperMarks_ = true;   // Aux checkbox, persisted
    // The last grip measurement, held on the Aux image until the next one
    // replaces it or the strip's Clear drops it. It is a readout of one run,
    // never the document: the profile's last* is the evidence and this is the
    // picture of it - the object cross, the offset and the verdict. Filled by the App::Update drain (UI thread), and
    // cleared with the product: the profile it names is the product's.
    struct AuxGripReadout {
        bool        have = false;
        std::string name, device, at, why;
        double      cx = 0.0, cy = 0.0, dx = 0.0, dy = 0.0;
        bool        ok = false;
    };
    AuxGripReadout auxGrip_;
    // Which handle of the object's outline a press on the picker took: -1
    // none, 0..3 a corner, 4 the body. The picker latches it here (it keeps no
    // state) so the same press does not also place the crosshair, and the
    // page saves the profile on the release edge.
    int gripBoxDrag_ = -1;
    // Whether the outline is drawn on the page, and whether it answers the
    // mouse. Editing is OFF by default: a box that can be dragged swallows
    // every click on its body, and its body is exactly where the marks the
    // profile is measured by have to be placed. Session-only, like the run
    // options beside them.
    bool gripShowOutline_ = true;
    bool gripEditOutline_ = false;
    // Whether the Aux control strip is expanded. It sits over the picture,
    // so the picture has to be able to win: collapsed it is one button.
    bool       auxPanelOpen_ = true;       // Aux strip, persisted
    // Taught-patch thumbnails, one slot per surface that shows one. Each is
    // keyed by what it was built from (VisionModelPatchKey), so a model switch,
    // a re-teach or an anchor drag reloads it and nothing else does.
    PatchTex visionTmpl_;       // vision teach page: the NCC patch + its anchor
    PatchTex alignTmpl_[2];     // object point 1/2 (placement), grip page
    // Alignment page station cards: one thumbnail per station x detection tier,
    // index (station - 1) * kAlignDetectTiers + (tier - 1).
    PatchTex alignTierTmpl_[2 * kAlignDetectTiers];
    PatchTex pickTmpl_;         // the manual-pick window, on the fallback path
    // The detection clipboard (CopyDetectionToClip). Session-only, never
    // persisted: a clipboard that survived a restart would paste settings from a
    // product that is no longer loaded.
    bool        visionClipHave_ = false;
    VisionModel visionClip_;
    std::vector<unsigned char> visionClipGray_;   // the patch, if the model had one
    int         visionClipW_ = 0, visionClipH_ = 0;
    std::string visionClipFrom_;                  // "Bench_white E1 det 1", for the tooltip

    // Scripts page UI state — the scanned list itself lives in session_.
    int                      scriptSel_ = -1;
    // Per-script breakpoints (keyed by path; click a source row to toggle,
    // shift+click for a range from the anchor = last plainly clicked row).
    std::map<std::string, std::set<int>> scriptBps_;
    int scriptBpAnchor_ = -1;
    ScriptEditorState scriptEd_;       // the editor: buffer, tokens, lint, completion
    // Unsaved-changes guard, the Recipe Editor's shape: 1 = select another
    // script (scPendingSel_), 2 = leave the page (scLeaveTarget_), 3 = reload
    // the list. Raised only while the machine is idle — see SwitchPage.
    int  scConfirmAction_ = 0;
    int  scPendingSel_    = -1;
    Page scLeaveTarget_   = Page::Overview;
    bool scNewOpen_       = false;     // New Script name prompt
    char scNewName_[64]   = "";

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
    // MAX: no exit threshold at all — the XCAN ladder runs to its end and
    // parks on the best point (align_exit_threshold 0, adaptive off, since a
    // zone needs a target). A manual maximisation, not a product power class,
    // so it is not a slot: the slot stays selected underneath and a click on
    // any target button turns MAX off. Climb cannot run under it (its target
    // is mandatory) and says so.
    bool   xcanTargetMax_  = false;
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
    // The deck selection is a NODE UID, not an index: a slot is a tree now, and
    // an index into a tree means nothing the moment a folder above it opens,
    // closes or is reordered. Empty = slot scope.
    std::string deckNode_;
    bool runEdit_       = false;   // mirrors orgOpen_: the strip's Edit checkbox
    // Teach mode (the Run strip's toggle). While on, EVERY recipe launch is a
    // teach run: the recipe is rewritten to its saved-position moves and each
    // arrival prompts fine-tune-and-Save (command/TeachMode.h). Deliberately
    // NOT in UiSettings - a mode that survives a restart would make every
    // launch silently teach - and cleared on LogOut like jogArmed_.
    bool teachMode_     = false;
    // The ONE place a launch's StartOpts pick up the toggle; every Run/Execute
    // surface builds its opts from this, or one of them forgets the flag.
    RecipeEngine::StartOpts RecipeStartOpts() const;
    // Per-node outcome of the current part's cycle, cleared ONLY by running the
    // start state (App::StartDeckRun). Not in the document and not in
    // UiSettings — see NodeMark above.
    std::map<std::string, NodeMark> deckMarks_;
    // The part's measurements, keyed by ctx key, with the recipe that wrote it.
    // Cleared on the SAME boundary as deckMarks_ — running the start node — so
    // the ticks and the numbers can never describe two different parts.
    struct CycleVar {
        std::string value;    // already formatted (CtxValueToString)
        std::string source;   // recipe name that last wrote it
    };
    std::map<std::string, CycleVar> cycleVars_;
    // Values panel filter: show only keys a recipe author named, hiding the
    // program context commands and standard scripts write under their default
    // prefixes (frame_*, pos_*, xcan_* ...). Persisted in UiSettings.
    bool runValsUserOnly_ = false;
    std::vector<std::string> sysCtxPrefixes_;   // built once, see IsSystemCtxKey
    // Instrument readouts pinned under the Values list: DataBus topics chosen
    // by the operator (persisted in UiSettings), with the last value pulled
    // for each. Refreshed on a timer, not per frame — these are glanced at,
    // and the streams behind them poll at hundreds of ms anyway.
    std::vector<RunReadout>  runReadouts_;
    std::vector<double>      runReadoutVals_;     // parallel to runReadouts_
    std::vector<bool>        runReadoutHas_;      // stream has published at all
    double                   runReadoutAt_ = 0.0; // ImGui::GetTime() of the last pull
    bool engineWasRunning_ = false;   // edge-detect ONE recipe finishing
    // The node the ACTIVE run was launched on ("" = slot scope). Recorded at
    // Start because the deck stays clickable during a run: advancing off
    // deckNode_ afterwards would step from wherever the operator clicked while
    // watching, not from what actually ran.
    std::string runNodeUid_;
    bool        runWasActive_ = false;   // edge-detect the runner going idle
    bool runOpenLoad_   = false;   // request to open the Load Program modal
    // ---- Program Organizer window ----
    // Its own window (multi-viewport: drag it to a second monitor) rather than a
    // page, so the deck and the DUT panel stay readable behind it while a
    // program is reshaped. Its slot selection is its OWN: reorganising slot 3
    // must not drag the operator's deck off the station they are running.
    bool        orgOpen_  = false;
    int         orgSlot_  = -1;      // 1st panel's slot (-1 = follow the deck)
    std::string orgSel_;             // 1st panel's selected node uid
    // The 2nd panel: another slot of the same program (or the same slot twice)
    // beside the first, so a node can be moved or copied across through the
    // aisle buttons or by drag. Its slot and selection are its own.
    bool        orgTwo_   = true;
    int         orgSlot2_ = -1;
    std::string orgSel2_;
    int         orgAddPanel_ = 0;    // where the library's add / double-click lands
    // The node clipboard (Cut / Copy / Paste on either panel's toolbar). One
    // subtree; a paste inserts a CLONE with fresh uids, so it can be pasted
    // twice and neither copy answers to the original's name.
    std::vector<ProgramNode> orgClip_;
    char        orgSlotNameBuf_[48] = "";   // a panel's Rename popup (one open at a time)
    char        orgFilter_[64] = ""; // recipe-library filter box
    // A drag-drop landing is RECORDED during the tree draw and applied after
    // it: an insert reallocates the very vector the recursion is walking, so
    // acting on the spot leaves the loop holding a dangling ProgramNode&.
    float       orgLibW_ = 240.0f;   // recipe-library column, dragged + persisted
    bool        orgSplitting_ = false;   // edge-detect the drag END
    std::string orgDropRecipe_;      // library payload (recipe uid)
    std::string orgDropNode_;        // tree payload (node uid being moved)
    std::string orgDropAnchor_;      // where it lands ("" = slot end, top level)
    bool        orgDropInto_    = false;   // anchor is a folder: go INSIDE it
    int         orgDropPanel_   = 0;       // which panel's tree recorded it
    bool        orgDropPending_ = false;
    bool        orgDelConfirm_  = false;   // open the delete-program confirm
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

    // View Data window state. dataDir_ is the folder dataRows_ was scanned
    // from: it differs from the loaded product's the moment another product
    // loads, which is how the window knows to rescan without a callback from
    // the session. Not persisted — a viewer is opened on demand. The selection
    // is a SERIAL, not an index: a header-click re-sorts the rows under it.
    bool                     showData_ = false;
    // Position History window (View menu, or the Motion page's History /
    // Hist buttons): the journal beside the positions file in use, filtered
    // by device and optionally one uid. The rows are re-read when the
    // session's journal version or the file behind it changes.
    bool                     showPosHist_ = false;
    std::string              posHistDevice_;      // "" = every device
    std::string              posHistUid_;         // "" = every position
    std::string              posHistPath_;        // what posHistEntries_ was read from
    unsigned                 posHistVer_ = ~0u;   // session version it was read at
    std::vector<PosChange>   posHistEntries_;     // oldest first, as on disk
    std::string              dataDir_;
    std::vector<DutDataRow>  dataRows_;
    std::vector<std::string> dataKeys_;       // union of keys, first-seen order
    int                      dataKeySel_  = -1;  // series A (index into dataKeys_)
    int                      dataKeySel2_ = -1;  // series B overlay (-1 = none)
    int                      dataPlotMode_ = 0;  // 0 series, 1 daily / 2 weekly / 3 monthly box
    int                      dataTimeSel_  = 0;  // 0 all, 1 this month, 2 last month, 3 YTD
    std::string              dataSelSerial_;
    char                     dataFilter_[64] = "";
    int                      dataSortCol_ = 0;     // table column the rows are ordered by
    bool                     dataSortAsc_ = true;  // (kept so a Reload keeps the order)
    std::string              scriptStartErr_;   // last Start() rejection

    bool openLoadProduct_ = false;   // menu -> popup handoff (OpenPopup must
    bool openNewProduct_  = false;   // be called outside the menu scope)
    char newProductName_[64] = "";

};

} // namespace firedog
