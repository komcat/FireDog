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

struct VisionModel;   // vision/VisionModel.h — only the patch helpers below need it

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

// ---- Marking features on the live image ------------------------------------
// The image + overlay + input half of a "mark a feature" surface: one movable
// crosshair and any number of already-latched marks. Hoisted out of the nominal
// teach page the moment the placement page needed the same thing — two copies
// of this projection arithmetic would drift, and the whole point is that a
// pixel means the same thing on every surface.
//
// A latched mark is held in MACHINE mm, so drawing it means projecting it back
// through the CURRENT pose: that is what keeps it on its feature after the
// stage has moved. With no pose there is nothing to project through, so marks
// are simply not drawn.
struct CameraMark {
    double      wx = 0.0, wy = 0.0;   // machine mm
    bool        have  = false;
    const char* label = nullptr;
    // 0 = the default "latched mark" colour. Set to distinguish a reference
    // point (a nominal, say) from the marks being placed.
    unsigned int col = 0;
};

// Everything needed to put a MACHINE-mm point on screen: the view, the box it
// is clipped to, and the pose + calibration the projection goes through.
struct WorldMarkView {
    CameraView view;
    ImVec2     boxMin{}, boxMax{};
    double     gx = 0.0, gy = 0.0;          // pose the image centre corresponds to
    float      pixelToMmX = 0.0f, pixelToMmY = 0.0f;
    int        imgW = 0, imgH = 0;
};

// Draw one such point. Inside the box it is a labelled cross; OUTSIDE it is an
// arrow on the box edge pointing at it, labelled with the distance.
//
// The arrow is the point of this function. A nominal 6 mm away is not merely
// invisible in a 3 mm field of view — nothing on screen distinguishes "the
// target is off to the left" from "the target is where I am looking", which is
// exactly how a run ends up measuring one feature and aiming at another.
void DrawWorldMark(ImDrawList* dl, const WorldMarkView& wv, const CameraMark& m,
                   float crossLen);

// A loaded PNG, ready to draw. Zero `tex` = not available, and every drawing
// path treats that as "skip", so a missing asset costs a diagram and nothing
// else.
struct MarkIcon {
    unsigned int tex = 0;
    int          w = 0, h = 0;
};

// Decode a PNG to 8-bit RGBA. A leaf helper (no GL, no App): the caller turns
// the pixels into a texture, because texture lifetime is state and this header
// keeps none.
bool LoadPngRGBA(const char* path, std::vector<unsigned char>* rgba,
                 int* width, int* height);

// The two "mark this corner" diagrams, laid out INLINE on the current line (the
// footer row, beside the buttons) rather than floated over the image: over the
// image they cover the thing being marked, and at a zoom that is exactly where
// the feature tends to be. The one the operator is being asked for is opaque,
// the other is dimmed to 30 %, so which point is wanted reads at a glance
// rather than from a caption somebody has to find.
//
// `active` is a 0-based index; -1 dims both. Drawn through the window's draw
// list with an ImGui::Dummy reserving the space, so the tint works on every
// ImGui version (Image()'s tint parameter did not survive 1.92). Leaves the
// cursor on the same line — call SameLine-free and just keep adding widgets.
void DrawMarkIconsInline(const MarkIcon icons[2], int active, float height);

// Inputs, plus pointers to the caller's crosshair/zoom state (App members —
// this helper keeps none of its own, per the no-state rule above).
struct MarkPicker {
    unsigned int tex  = 0;               // camera texture (0 = no feed)
    int    imgW = 0, imgH = 0;
    float  pixelToMmX = 0.0f, pixelToMmY = 0.0f;
    bool   havePose = false;
    double gx = 0.0, gy = 0.0;           // the pose marks are projected through
    const CameraMark* marks = nullptr;
    int    markCount = 0;
    // Non-null draws the midpoint of marks 0 and 1 (both latched) under this
    // label — the point those two marks exist to define, plus the envelope box
    // they bound.
    const char* midpointLabel = nullptr;
    // Spins that envelope box about the midpoint, for a footprint that sits at
    // an angle. PURELY COSMETIC: the box is drawn from it and nothing else is,
    // because the marks are two measured machine points and the midpoint of a
    // pair is the same point at every angle.
    float envelopeDeg = 0.0f;
    // Step diagrams, bottom-left. `activeMark` is the 0-based mark the operator
    // should place next; the other is dimmed.
    MarkIcon icons[2];
    int      activeMark = -1;
    float* px   = nullptr;               // crosshair, camera px   (in/out)
    float* py   = nullptr;
    float* zoom = nullptr;               // magnification >= 1     (in/out)
};

// What a right-click asked for. The widget cannot move anything itself — it
// has no device access, by this header's rule — so it reports the relative move
// that would bring the clicked point to the image centre and the caller decides
// whether that is allowed.
struct MarkPickerResult {
    bool   moveRequested = false;
    double dxMM = 0.0, dyMM = 0.0;   // relative, machine frame
};

// Fill `box` with the image, the overlay and the input handling, then a footer
// row (icons + hint + zoom slider + Fit). Arrow keys nudge the crosshair only
// while the mouse is over the hosting window, so they never steal from a field
// beside it. LEFT click places the crosshair; RIGHT click asks to drive there.
MarkPickerResult DrawMarkPicker(const char* id, const ImVec2& box, const MarkPicker& mp);

// ---- What a detection model is looking for ---------------------------------
// The picture a model matches against, plus the point it REPORTS. Three
// surfaces want it and they must not disagree: the Alignment page (so the point
// a station will use is visible while the procedure is authored), the vision
// teach page's own thumbnail, and — the reason this exists — the manual pick
// window on the auto->manual fallback. An operator asked to mark a feature by
// hand because detection failed otherwise has nothing telling them WHICH point
// of it the run wanted, and a mark 30 um off the taught one produces a frame
// that is wrong in a way nothing on screen shows.
struct PatchTex {
    unsigned int tex = 0;
    int          w = 0, h = 0;
    float        rx = 0.0f, ry = 0.0f;   // the reported point, in PATCH pixels
    std::string  key;                    // what it was built from (see below)
};

// Identity of the patch `m` would produce right now. Everything the picture or
// the reported point depends on goes in, so a re-teach (same filename, new
// content), an anchor drag or a corner change all reload — and nothing else does.
std::string VisionModelPatchKey(const VisionModel& m);

// The patch itself: 8-bit grey, row-major, w*h bytes. The taught `.pgm` for a
// pattern match, the SYNTHESISED quadrant patch for a two-edge corner (built by
// the same MakeCornerTemplate the detector matches with, so the picture cannot
// disagree with the search), and nothing at all for the blob methods — a circle
// or a solid square is found as a SHAPE, there is no image to show and the
// reported point is the shape's own centre. False = nothing to draw.
bool VisionModelPatch(const VisionModel& m, const std::string& visionDir,
                      std::vector<unsigned char>* gray, int* w, int* h,
                      float* rx, float* ry);

// Draw a loaded patch, scaled to fit maxW x maxH, with a cross on the reported
// point. Advances the cursor like any widget; returns the size drawn ((0,0) if
// there was nothing to draw, so a caller can skip its caption).
ImVec2 DrawPatchThumb(const PatchTex& pt, float maxW, float maxH);

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
