// Widgets.h : small UI helpers and lookup tables shared by the App page files.
//
// Everything here was file-scope in the single App.cpp. It lives in a header
// now because the page split gives most of these more than one consumer, and a
// copied table silently drifts (kSteps, kFonts and kKeiRanges each serve two
// TUs). Keep this to leaf helpers: no App members, no device access, no state.
//
// Widgets.cpp is also the ONLY translation unit that includes windows.h — it
// backs OpenFolderInExplorer, so no page file has to deal with the macro
// fallout.
#pragma once

#include <string>
#include <vector>

#include "imgui.h"

#include "hardware/IDevice.h"   // ConnState

namespace firedog {

// User-adjustable UI prefs persist here (gitignored .local.json per convention),
// written next to the exe alongside the copied config/ folder.
inline constexpr const char* kUiSettingsPath = "config/ui.local.json";

// Selectable UI fonts. Paths are Windows system fonts (this is a Windows-only
// project); "" = ImGui's built-in font. Keep index 0 as the startup default.
struct FontOption { const char* label; const char* path; };
inline constexpr FontOption kFonts[] = {
    {"Segoe UI", "C:/Windows/Fonts/segoeui.ttf"},
    {"Tahoma",   "C:/Windows/Fonts/tahoma.ttf"},
    {"Verdana",  "C:/Windows/Fonts/verdana.ttf"},
    {"Consolas", "C:/Windows/Fonts/consola.ttf"},
    {"Built-in", ""},
};
inline constexpr int kFontCount = static_cast<int>(sizeof(kFonts) / sizeof(kFonts[0]));

// Jog step sizes (label + value in mm). Shared by the Jog window, the
// manual_adjust window and the Motion page.
struct StepSize { const char* label; double mm; };
inline constexpr StepSize kSteps[] = {
    {"0.1um", 0.0001}, {"0.2um", 0.0002}, {"0.5um", 0.0005},
    {"1um",   0.001},  {"2um",   0.002},  {"5um",   0.005},
    {"10um",  0.01},   {"25um",  0.025},  {"50um",  0.05},
    {"100um", 0.1},    {"250um", 0.25},   {"500um", 0.5},
    {"1mm",   1.0},    {"2mm",   2.0},    {"5mm",   5.0},
};
inline constexpr int kStepCount = static_cast<int>(sizeof(kSteps) / sizeof(kSteps[0]));

// The ad-hoc recipe the Autofocus buttons start (Jog window Camera tab + Aux
// window). Shared because BOTH surfaces match the engine's status against the
// name to report only on their own run — the engine is also driven by the
// Recipe Editor, the Run page and the programs.
inline constexpr const char* kQuickAutoFocusName   = "Autofocus (gantry-main Z)";
inline constexpr const char* kQuickAutoFocusDevice = "gantry-main";

// The ad-hoc recipe the vision teach window's Reliability button starts. Same
// reason it is a named constant: the window matches the engine's status against
// it so it reports only on its OWN run.
inline constexpr const char* kVisionTestRunName = "Detection reliability";

// Sweep directions the buttons offer: button label + the `direction` parameter
// value the auto_focus command takes. Down-first because that is the usual
// approach direction; the other two are here because a sweep whose peak lands
// on an edge is telling you to look the other way.
struct AfDirection { const char* label; const char* value; };
inline constexpr AfDirection kAfDirections[] = {
    {"Z-",   "minus"},
    {"Z+",   "plus"},
    {"Z+/-", "both"},
};
inline constexpr int kAfDirectionCount =
    static_cast<int>(sizeof(kAfDirections) / sizeof(kAfDirections[0]));

// Keithley 6482 fixed current ranges (full-scale amps, decades).
struct KeiRange { const char* label; double amps; };
inline constexpr KeiRange kKeiRanges[] = {
    {"2 nA", 2e-9}, {"20 nA", 2e-8}, {"200 nA", 2e-7}, {"2 uA", 2e-6},
    {"20 uA", 2e-5}, {"200 uA", 2e-4}, {"2 mA", 2e-3}, {"20 mA", 2e-2},
};
inline constexpr int kKeiRangeCount = static_cast<int>(sizeof(kKeiRanges) / sizeof(kKeiRanges[0]));

// Closest table row for a configured fixed range (default: 2 mA).
int KeiRangeIndexFor(double amps);

// ---- Live camera image view ------------------------------------------------
// The zoomable "cover fit plus follow-the-mark" mapping between camera pixels
// and screen pixels. Hoisted out of the alignment crosshair-pick window when
// the vision teach window needed the SAME mapping: a pixel has to mean the same
// thing in both surfaces, and two copies of this arithmetic would drift the
// moment one of them gained a feature.
//
// `fit` is a COVER fit (fill the box, crop the overflow) times a zoom >= 1, so
// the visible window is always strictly inside the image and there is no
// letterbox case. The window follows `focus` (clamped to the image), which is
// what stops a zoomed view from hiding the very point being marked.
struct CameraView {
    ImVec2 origin{};          // top-left of the image box, in screen coords
    ImVec2 size{};            // the box's size on screen
    float  scale = 1.0f;      // screen px per camera px
    float  left = 0.0f, top = 0.0f;   // top-left of the visible CAMERA-px window
    float  vw = 0.0f, vh = 0.0f;      // visible camera-px extent
    int    imgW = 0, imgH = 0;

    ImVec2 ToScreen(float px, float py) const {
        return ImVec2(origin.x + (px - left) * scale, origin.y + (py - top) * scale);
    }
    // Screen point -> camera pixel. The inverse of ToScreen; used by a click.
    void ToCamera(const ImVec2& p, float* px, float* py) const {
        if (px) *px = left + (p.x - origin.x) / scale;
        if (py) *py = top  + (p.y - origin.y) / scale;
    }
    // UVs for ImGui::Image so the texture shows exactly the visible window.
    ImVec2 Uv0() const { return ImVec2(left / static_cast<float>(imgW),
                                       top  / static_cast<float>(imgH)); }
    ImVec2 Uv1() const { return ImVec2((left + vw) / static_cast<float>(imgW),
                                       (top  + vh) / static_cast<float>(imgH)); }
};

// Build the view for an imgW x imgH image drawn at `origin` in a `size` box,
// magnified by `zoom` (>= 1) and centred on the camera pixel (focusX, focusY).
CameraView MakeCameraView(const ImVec2& origin, const ImVec2& size,
                          int imgW, int imgH, float focusX, float focusY, float zoom);

// Connection-state presentation, used by every device page.
ImVec4      StateColor(ConnState s);
const char* StateGlyph(ConnState s);
const char* StateText(ConnState s);
void        DeviceStatusLine(ConnState s, const char* name);

// A draggable splitter bar (a thin, invisible-until-hovered button). Returns
// true while being dragged; the caller applies io.MouseDelta to the adjacent
// panel size. `vertical` = a tall bar that resizes left/right; otherwise a wide
// bar that resizes up/down.
bool Splitter(const char* id, bool vertical, float thickness, float length);

// Format an amp value with an auto-scaled unit (mA / uA / nA / pA).
std::string FormatCurrent(double a);

// Achieved sample rate from the samples in the last 1 s of a stream snapshot.
float MeasuredHz(const std::vector<float>& t);

// Open a folder in Windows Explorer (best-effort). Paths from the session are
// cwd-relative (products/ lives next to the exe), so resolve to absolute; the
// folder may not exist yet (product data/ before the first DUT) — create it.
void OpenFolderInExplorer(const std::string& dir);

} // namespace firedog
