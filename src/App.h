// App.h : layer 3 (UI) — main loop, Main window + Aux window (own OS window).
// The UI only renders: it reads HardwareManager::Snapshot()/accessors and the
// DataBus. It owns no device and runs no connect — Workflow (layer 2) does that.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "imgui.h"

#include "ui/GlHost.h"
#include "core/Log.h"
#include "core/Product.h"
#include "core/SavedPositions.h"
#include "hardware/HardwareManager.h"
#include "hardware/HardwareConfig.h"
#include "workflow/Workflow.h"

namespace firedog {

// A draggable item on the layout canvas (drag-and-drop demo).
struct CanvasObject {
    ImVec2      pos;    // position relative to the canvas origin
    const char* label;
    ImU32       color;
};

// Which content the Main window shows. Selected via the Hardware menu.
enum class Page { Overview, Motion, IO, Pneumatic };

class App {
public:
    int Run();

private:
    void Update(float dt);
    void DrawMainWindow();
    void DrawLeftPanel();      // 20% column: hardware status + display controls
    void DrawMiddlePanel();    // main content: vision image + overlay
    void DrawLowerPanel();     // 15%-height strip: drag-object canvas
    void DrawRightPanel();     // 15% column: Overview placeholder
    void DrawJogWindow();      // undocked Jog window (toggled from View menu)
    void DrawJogPanel();       // jog controls: device/velocity/step/direction/keys
    void DrawVelocityCombo(std::size_t di);   // shared velocity combo (Jog + Motion page)
    // Motion page — all motion devices (PI + ACS) on one page.
    void DrawMotionListPanel();    // left:   device list + status
    void DrawMotionDetailPanel();  // middle: selected device + positions
    void DrawMotionInfoPanel();    // right:  live axis positions
    // IO page — all IO devices (Ezi-IO + WAGO) on one page.
    void DrawIoListPanel();        // left:   device list + status
    void DrawIoDetailPanel();      // middle: selected device + output toggles
    void DrawIoInputPanel();       // right:  live input pin states
    // Pneumatics page — synthetic slides composed from IO pins.
    void DrawPneumaticListPanel();    // left:   slide list + position
    void DrawPneumaticDetailPanel();  // middle: selected slide + Extend/Retract/Stop
    void DrawPneumaticInfoPanel();    // right:  live output/sensor states
    void DrawWatchEditWindow();       // IO Watch editor (own window, View menu)
    void DrawSafetyAlertModal();      // blocking safety alert (E-Stop etc.)
    void DrawAuxWindow();
    void DrawSignalOverlay();  // Keithley readout + chart (floats over the image)
    void DrawLogPanel();       // log viewer (Logger ringbuffer) — Main lower panel
    void DrawImageOverlay();   // image + vision overlay
    void DrawObjectCanvas();   // click-drag objects (unused; kept for reuse)
    void SaveUiSettings();     // persist font choice + size to disk
    void SavePositions();      // persist savedPos_ (autosaved on every change)

    // Product manifest (File menu). Load/New replace savedPos_ from the
    // product's positions file; Save runs through the manifest entries.
    void DrawProductPopups();  // Load/New modal dialogs (opened from File menu)
    bool LoadProduct(const std::string& name);
    void NewProduct(const char* name);
    void SaveProduct();

    GlHost host_;

    // Layer 1 owns the hardware; layer 2 orchestrates it. The UI only renders.
    HardwareManager hardware_;
    Workflow        workflow_{hardware_};

    // UI-side camera texture — frames are pulled from hardware_.Camera() each frame.
    unsigned int               cameraTex_ = 0;
    int                        cameraW_ = 0, cameraH_ = 0;
    std::vector<unsigned char> frameBuf_;

    // Keithley chart buffers — pulled from the DataBus each frame.
    int                       chartCh_ = 2;   // 0 ch1, 1 ch2, 2 both
    std::vector<float>        kT1_, kV1_, kT2_, kV2_;
    std::vector<float>        kX1_, kX2_;      // X shifted to relative time (t - now)
    double                    chartClock_ = 0.0;   // smooth rolling X right-edge
    static constexpr double   kChartWindowSec = 10.0;

    unsigned int              viewportTex_ = 0;   // GL texture: main-window placeholder
    std::vector<CanvasObject> objects_;
    int                       dragging_ = -1;

    // Display controls (font picker + size) — bound to the GlHost font API.
    int   fontIndex_ = 0;        // index into kFonts (see App.cpp)
    float uiFontSize_ = 26.0f;   // current UI font size in px

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
    Page currentPage_    = Page::Overview;
    int  selectedMotion_ = -1;   // absolute index into the HardwareManager motion fleet
    bool showHiddenPos_  = false;   // reveal hidden (system) positions in the table

    // IO page: selected device + the output Enable gate (defaults ON per user
    // preference — the checkbox remains as an explicit kill switch).
    int  selectedIo_ = -1;   // absolute index into the HardwareManager IO fleet
    bool ioArmed_    = true;

    // Pneumatics page: selected slide + the same Enable gate.
    int  selectedSlide_ = -1;   // absolute index into the HardwareManager slide fleet
    bool pneuArmed_     = true;

    // IO Watch editor (View menu): edits a working copy of the io_watch config;
    // Apply rebuilds the live watch, Save also writes config/hardware.json.
    bool          showWatchEdit_ = false;
    IoWatchConfig watchCfg_;

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
    std::map<int, bool>   velPushed_;     // device index -> UI value pushed once
    std::map<int, double> velCustomEdit_; // device index -> custom input-box value
    std::map<int, double> velCustomSet_;  // device index -> applied custom (0 = preset)

    // Saved/taught positions per device (keyed by device name). Loaded at
    // startup, autosaved on every change — into the current product's positions
    // file when one is loaded, else config/positions.local.json.
    SavedPosMap savedPos_;

    // Current product (File menu). Not loaded => config-fallback persistence.
    ProductManifest product_;
    bool openLoadProduct_ = false;   // menu -> popup handoff (OpenPopup must
    bool openNewProduct_  = false;   // be called outside the menu scope)
    char newProductName_[64] = "";

};

} // namespace firedog
