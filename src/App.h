// App.h : layer 3 (UI) — main loop, Main window + Aux window (own OS window).
// The UI only renders: it reads HardwareManager::Snapshot()/accessors and the
// DataBus. It owns no device and runs no connect — Workflow (layer 2) does that.
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "imgui.h"

#include "ui/GlHost.h"
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
enum class Page { Overview, Camera, Keithley, CLD, PSU, Motion, IO, Pneumatic,
                  WatchEdit, TransformEdit, RecipeEdit, Scripts, DutView, Run };

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

class App {
public:
    int Run();

private:
    void Update(float dt);
    void DrawMainWindow();
    void ToggleEditorPage(Page p);   // enter/leave a full-page takeover (View menu + F-keys)
    void SwitchPage(Page p);         // ALL page switches route here (dirty-editor guard)
    void DrawLeftPanel();      // 20% column: hardware status + DUT + display
    void DrawDutBlock();       // active-DUT status + open/close (Overview left)
    void DrawMiddlePanel();    // main content: vision image + overlay
    void DrawLowerPanel();     // 15%-height strip: log viewer
    void DrawRightPanel();     // 15% column: Overview placeholder
    void DrawJogWindow();      // undocked Jog window (toggled from View menu)
    void DrawJogPanel();       // jog controls: device/velocity/step/direction/keys
    void DrawVelocityCombo(std::size_t di);   // shared velocity combo (Jog + Motion page)
    // Camera page — the (single) Pylon camera.
    void DrawCameraListPanel();    // left:   status + connect/disconnect
    void DrawCameraDetailPanel();  // middle: exposure + pixel->mm + preview
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
    // Programs page (View menu, full-page) — Phase C1: list/run .lua programs.
    void DrawScriptsPanel();          // page body: scripts | source | run info
    void LoadScriptsList();           // session rescan + Scripts page selection sync
    void SyncScriptSelection();       // clamp scriptSel_ / drop the cached source view
    // DUT Records viewer (View menu, full-page, read-only) — Phase D2.
    void DrawDutViewPanel();          // serial list | entries table | summary
    void LoadDutList();               // scan the product's data/ for records
    // Run page (View menu, full-page) — programs = slots of recipes. R1.
    void DrawRunPanel();              // deck | execution + XCAN + log | DUT
    void StartDeckRun(RunScope scope);   // granularity follows the deck selection
    void DrawRunScanBlock();          // shared channel/Target/Trace header + Raster|XCAN tabs
    void DrawRunXcanBlock();          // XCAN tab: hex-left/right launch presets
    void StartXcanPreset(const std::string& device, int presetIdx);
    void DrawXcanPresetModal();       // right-click settings for a launch preset
    void DrawXcanTargetModal();       // right-click settings for a target preset
    void LoadScanPresets();           // config/scan_presets.local.json -> .json
    void SaveScanPresets() const;
    void DrawRunRasterBlock();        // Raster tab: area-scan launcher (raster_scan.lua)
    void StartRasterScan(const std::string& device);
    void LoadProgramsFromDisk();      // session load + deck selection reset
    void LoadRecipesFromDisk();       // session load + UI selection reset
    void DrawSafetyAlertModal();      // blocking safety alert (E-Stop etc.)
    void DrawExitConfirmModal();      // intercepts the OS close request (X / Alt+F4)
    void DrawAuxWindow();
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
    bool edOpenRenameRecipe_  = false;
    bool edOpenDeleteRecipe_  = false;
    bool edOpenDeleteStep_    = false;
    char edNameBuf_[64]       = "";
    char edSearchBuf_[64]     = "";      // palette search filter

    // Scripts page UI state — the scanned list itself lives in session_.
    int                      scriptSel_ = -1;
    // Per-script breakpoints (keyed by path; click a source row to toggle,
    // shift+click for a range from the anchor = last plainly clicked row).
    std::map<std::string, std::set<int>> scriptBps_;
    int scriptBpAnchor_ = -1;

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
    int  progSel_       = -1;
    int  deckSlot_      = -1;
    int  deckRecipe_    = -1;
    bool runEdit_       = false;
    bool runOpenDelete_ = false;
    std::string runStartErr_;
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
