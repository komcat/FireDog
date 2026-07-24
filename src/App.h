// App.h : layer 3 (UI) — main loop, Main window + Aux window (own OS window).
// The UI only renders: it reads HardwareManager::Snapshot()/accessors and the
// DataBus. It owns no device and runs no connect — Workflow (layer 2) does that.
#pragma once

#include <vector>

#include "imgui.h"

#include "ui/GlHost.h"
#include "core/Log.h"
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
enum class Page { Overview, PI, ACS };

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
    // PI/ACS pages — same layout, motion-device content. `type` is "PI" | "ACS".
    void DrawMotionListPanel(const char* type);    // left:   device list + status
    void DrawMotionDetailPanel(const char* type);  // middle: selected device + positions
    void DrawMotionInfoPanel(const char* type);    // right:  info / reserved
    void DrawAuxWindow();
    void DrawSignalOverlay();  // Keithley readout + chart (floats over the image)
    void DrawLogPanel();       // log viewer (Logger ringbuffer) — Main lower panel
    void DrawImageOverlay();   // image + vision overlay
    void DrawObjectCanvas();   // click-drag objects (unused; kept for reuse)
    void SaveUiSettings();     // persist font choice + size to disk

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

    // Main-window page + which motion device is selected on the PI/ACS pages.
    Page currentPage_    = Page::Overview;
    int  selectedMotion_ = -1;   // absolute index into the HardwareManager motion fleet

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
    int jogVelIndex_  = 0;   // index into the device-type velocity list
    int jogStepIndex_ = 9;   // index into kSteps (default 100um)
    int jogRotAxis_   = 0;   // selected rotational axis (0 = first beyond XYZ)
    int jogVelDevice_ = -1;  // device index we last pushed velocity to

};

} // namespace firedog
