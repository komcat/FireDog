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

#include "core/AlignLighting.h"    // LightCalloutPicker
#include "core/SavedPositions.h"   // SavedPos / SavedPosMap (TaughtPosePicker)
#include "hardware/IDevice.h"      // ConnState

namespace firedog {

struct VisionModel;   // vision/VisionModel.h — only the patch helpers below need it

// User-adjustable UI prefs persist here (gitignored .local.json per convention),
// written next to the exe alongside the copied config/ folder.
inline constexpr const char* kUiSettingsPath = "config/ui.local.json";   // under MachinePath()

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
// `fit` is a COVER fit (fill the box, crop the overflow) times a zoom, so at
// zoom 1 the box is full and the visible window is strictly inside the image.
// The window follows `focus` (clamped to the image), which is what stops a
// zoomed view from hiding the very point being marked. A zoom BELOW 1 shows
// the whole frame with a letterbox: the window is then wider than the image
// and centred on it, and `DrawCameraViewImage` draws only the part that is
// image — `Uv0`/`Uv1` alone would sample past the texture's edge.
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
// magnified by `zoom` over the cover fit and centred on the camera pixel
// (focusX, focusY).
CameraView MakeCameraView(const ImVec2& origin, const ImVec2& size,
                          int imgW, int imgH, float focusX, float focusY, float zoom);

// Paint the view's box: a dark ground, then the image clipped to the part of
// the window that lies on it. At zoom >= 1 that is the whole box; below 1 it
// is the letterboxed frame. Draw-list only — the caller reserves the box
// (ImGui::Dummy / InvisibleButton) so the item and the picture stay one size.
void DrawCameraViewImage(ImDrawList* dl, unsigned int tex, const CameraView& view);

// The zoom ratio in the box's top-left corner: "x1.50 over the fit". Every
// live view shows it, because the same feature reads as a different size on
// every station and a picked point is only as good as the scale it was picked
// at.
void DrawZoomBadge(ImDrawList* dl, const CameraView& view, float zoom);

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

// A rectangle in machine mm through the same projection - a tool's footprint,
// a gripped object's outline. `ax/ay` is a point ON it that somebody measured
// (a tip, an object's centre) and `adx/ady` where that point sits relative to
// the rectangle's centre, in the rectangle's own frame: 0,0 says the point IS
// the centre. `wMm` x `lMm` is its size, `deg` its spin about the centre. The
// anchor offset turns with the box, so a corner-anchored outline stays hung off
// its corner as the jaws turn.
//
// Clipped to the box, and drawn only while its rim is in it - the CROSS that
// goes with it carries the edge arrow, and an arrow per outline would say the
// same thing twice. Nothing is drawn below a few screen px of half-size: at
// that scale an outline is a fatter dot beside the cross's ring, and two glyphs
// that mean different things at the same size is worse than one.
void DrawWorldBox(ImDrawList* dl, const WorldMarkView& wv, double ax, double ay,
                  double adx, double ady, double wMm, double lMm, double deg,
                  unsigned int col, float handlePx = 0.0f);

// An annulus SECTOR in machine mm: everywhere a point may be, given a distance
// range and a direction range from a centre. The Alignment Procedure's guard
// band is exactly that shape — E2 must be `span +/- tol` away from E1, in the
// direction `theta +/- tol` — and drawing it is what turns "mark E2" into
// "mark E2 in here".
//
// Only part of it is ever on screen, which is the point rather than a
// limitation: at a 7 mm span in a 3 mm field of view the operator sees two arcs
// and the limit rays crossing the view, and can move the crosshair between them.
// The centre itself is usually off-screen entirely.
struct WorldBand {
    bool         have = false;
    double       cx = 0.0, cy = 0.0;      // the centre, machine mm (E1)
    double       rMin = 0.0, rMax = 0.0;  // mm
    double       aMin = 0.0, aMax = 0.0;  // degrees, machine frame
    unsigned int col = 0;                 // 0 = the default "aim here" colour
};

void DrawWorldBand(ImDrawList* dl, const WorldMarkView& wv, const WorldBand& b);

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

// ---- Recipe row icons ------------------------------------------------------
// Which picture a step type draws, as a file stem under `assets/recipe_icons/`
// (nullptr = this type has no icon). A pure name->name lookup so the same
// answer serves the palette and the step tree: an operator who learns a glyph
// while picking a command must meet the same glyph in the recipe it lands in.
//
// The mapping is by command NAME, not category — Align and Motion both contain
// moves, and what the row DOES to the machine is what the reader is scanning
// for. Several commands deliberately share one file.
const char* StepIconFile(const char* type);

// Draw one row icon at text height and stay on the current line. A null or
// unloaded icon still RESERVES the column: the point of the icon is that the
// eye can run down a fixed column, and a ragged left edge costs more than the
// blank square does.
void DrawStepIconInline(const MarkIcon* icon);

// A button with an icon before its label, the pair centred in the frame the way
// a plain Button centres its text. Same icon set, drawn at text height.
//
// A null or unloaded icon degrades to exactly ImGui::Button / SmallButton — the
// opposite of the row rule above, and deliberately: a row is scanned down a
// column, a button is read where it sits, so a reserved blank square would buy
// nothing and only make the control wider.
//
// `label` must be a PLAIN label (no `##` id suffix): it is both the button's id
// and the text drawn on it.
bool IconButton(const MarkIcon* icon, const char* label,
                const ImVec2& size = ImVec2(0.0f, 0.0f));
bool IconSmallButton(const MarkIcon* icon, const char* label);

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
    // Non-null draws the CENTRE of every latched mark under this label — the
    // point those marks exist to define — plus the envelope box they bound.
    // Two marks make it a midpoint; four make it the centre of the ring. It
    // needs at least two latched, because one mark defines no centre but its
    // own.
    const char* midpointLabel = nullptr;
    // Spins that envelope box about the midpoint, for a footprint that sits at
    // an angle. PURELY COSMETIC: the box is drawn from it and nothing else is,
    // because the marks are two measured machine points and the midpoint of a
    // pair is the same point at every angle.
    float envelopeDeg = 0.0f;
    // A REFERENCE point the operator is not placing: a registered tool tip,
    // projected through the same pose as the marks. It is drawn differently from
    // a mark on purpose — a mark is a thing somebody measured, a tip is a thing
    // the machine computed out of a registration, and the two must not be read
    // as the same kind of claim. Off-screen it becomes an edge arrow like any
    // other world mark.
    bool        toolHave = false;
    double      toolWx = 0.0, toolWy = 0.0;
    const char* toolLabel = nullptr;
    // The tool's FOOTPRINT, centred on that tip: a rectangle in mm, drawn so an
    // operator can see the gripper's perimeter against the part before anything
    // is placed. Non-null size pointers draw it; a non-null `boxDragging` ALSO
    // makes its corners draggable — the box always re-centres on the tip, so
    // dragging a corner can only change the size, and there is no position for
    // it to drift into. Without the flag the box is read-only, for a page that
    // shows the footprint but does not own it.
    //
    // Both the size and the drag flag are the CALLER's state, as everything in
    // this file is: the widget keeps none. `boxDragging` latches a drag that
    // started on a corner so the same press does not also place the crosshair.
    float* boxWmm     = nullptr;
    float* boxHmm     = nullptr;
    bool*  boxDragging = nullptr;
    // Spins the footprint about the tip, degrees: the device's live U, so the
    // box turns with the jaws. A screen-space spin like `envelopeDeg`, and
    // like it purely a drawing matter - the tip is the same point at every
    // angle, and the corner drag measures its size in the box's own frame.
    float  boxDeg = 0.0f;
    // Read-only outlines in machine mm, drawn through the same projection as
    // the marks: a gripped object's, hung off the point that was measured on
    // it. Each is a DrawWorldBox; see it for the anchor. Read-only by design -
    // the size is typed on the page that owns it, and the box shows it.
    struct WorldBox {
        double       ax = 0.0, ay = 0.0;     // the anchor point, machine mm
        double       adx = 0.0, ady = 0.0;   // point - centre, in the box's frame
        double       wMm = 0.0, lMm = 0.0;
        float        deg = 0.0f;
        unsigned int col = 0;                // 0 = the default outline colour
        // All five non-null make the box EDITABLE on the image: its BODY drags
        // to move it and a CORNER drags to resize it with the opposite corner
        // held, the way a rectangle is drawn in any drawing tool. Typing an
        // anchor offset and guessing where the box lands was the alternative,
        // and nobody can do that against a live picture. Written back as the
        // caller's own size and anchor offset (mm, the box's frame) - the
        // anchor POINT never moves, the box moves around it. `drag` latches
        // which handle a press took (-1 none, 0..3 a corner, 4 the body) so the
        // same press does not also place the crosshair. Read-only otherwise.
        double* editW   = nullptr;
        double* editL   = nullptr;
        double* editAdx = nullptr;
        double* editAdy = nullptr;
        int*    drag    = nullptr;
    };
    const WorldBox* boxes    = nullptr;
    int             boxCount = 0;
    // Step diagrams, bottom-left. `activeMark` is the 0-based mark the operator
    // should place next; the other is dimmed.
    MarkIcon icons[2];
    int      activeMark = -1;
    float* px   = nullptr;               // crosshair, camera px   (in/out)
    float* py   = nullptr;
    float* zoom = nullptr;               // magnification over the fit (in/out)
    // The range the wheel and the slider keep `zoom` in. Below 1 letterboxes
    // the whole frame. The default is the pick window's; a page whose marks
    // are a coarse edge rather than a fine feature narrows it (0.5..3).
    float  zoomMin = 1.0f, zoomMax = 12.0f;
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

// A detection model's lighting (core/AlignLighting.h), in the Light page's
// call-out style: a light picker and a % slider, then one button per colour
// call-out (Red ... White + collimate), Off (hold the light dark) and None
// (leave lighting alone). The setting in force is highlighted. `lights` are the
// configured light names (Workflow::LightNames). Edits `l` in place and returns
// true when it changed (a slider only on release). `*applyNow` is set when the
// change should be LIT now — only when `canApply`; the caller sends it through
// Workflow::RequestLightCallout so a click never does socket I/O on the UI thread.
bool LightCalloutPicker(const char* id, AlignLighting& l, const std::vector<std::string>& lights,
                        bool canApply, bool* applyNow);

// A draggable splitter bar (a thin, invisible-until-hovered button). Returns
// true while being dragged; the caller applies io.MouseDelta to the adjacent
// panel size. `vertical` = a tall bar that resizes left/right; otherwise a wide
// bar that resizes up/down.
bool Splitter(const char* id, bool vertical, float thickness, float length);

// The "or taught..." combo: a saved position of `device` to take a pose from.
// Returns the picked one for this frame only — the caller resolves it to
// COORDINATES on the spot (App::AdoptDeviceRegPose says why a record must not
// hold a position uid). Hidden positions are withheld: they are the alignment
// and grip systems' private poses and are noise to somebody teaching a pose.
// Same rule the recipe picker follows — `hidden` governs what is OFFERED,
// never what resolves. Shared by the Registration and Waffle Tray pages.
const SavedPos* TaughtPosePicker(const char* id, const SavedPosMap& all,
                                 const std::string& device);

// One line of text drawn as large as `box` allows and centred in it, from the
// current cursor. The font is sized off the box height and shrunk to fit the
// width, so a short "10um" and a long device name both fill the space without
// clipping. Sizes come off the base font, so the label follows the UI scale.
// Used for the readouts an operator must be able to check from a step back:
// the Jog window's step size and device name, the Manual Adjust window's
// device box.
void DrawBigLabel(const char* text, const ImVec2& box, const ImVec4& color);

// ---- The needle motion drawing --------------------------------------------
// One picture of how the dispense needle moves, shared by the Needle page (the
// calibration) and the Dispense helper window (a dispense_run step): both
// procedures move in the same order, so they draw the same thing. It is cut
// into four regions - 1 traverse | 2-4 down to the surface | 5-7 shot and up |
// 8 back - and each caller lays a table of its OWN parameters under it with
// the same column widths, so every field sits under the step it drives.
inline constexpr int   kNeedleRegions = 4;
inline constexpr float kNeedleRegionW[kNeedleRegions] = {255.0f, 325.0f, 290.0f, 230.0f};
inline constexpr float kNeedleMotionW = 1100.0f;   // sum of kNeedleRegionW

// The value lines under each step's name, already formatted by the caller
// (its parameter names, its units). Empty = no line.
struct NeedleMotionText {
    std::string traverse;          // under "1 traverse"
    std::string approach;          // under "3 approach"
    std::string safety;            // beside the safety-height tick
    std::string final1, final2;    // under "4 final"
    std::string shot, lift;        // under "5 shot  6 lift off"
    std::string back;              // under step 8
    // After the slide comes up: false = one diagonal move back to the camera
    // (the calibration's flat coupon); true = Z climbs all the way first, then
    // XY (a real part may have features in between - the dispense).
    bool climbFirst = false;
    bool showBack   = true;        // false: step 8 is the climb only
};
// Draws at the cursor, `width` wide (>= kNeedleMotionW; the regions stretch,
// the needles and text do not) and reserves the space.
void DrawNeedleMotion(const NeedleMotionText& t, float width);

// ---- The FireDog mascot ----------------------------------------------------
// A corgi with a flame for a tail, drawn as `px`-sized squares with `origin` as
// its top-left. Shared by the login screen and the quick strip. The animation
// is a pure function of the mood and `t` (ImGui::GetTime()), so a caller keeps
// no frame state for it — only what it needs to pick the mood.
enum class FireDogMood {
    Idle,      // standing; the flame flickers, the eye blinks now and then
    Running,   // a run is in flight: trotting, the flame streams back
    Waiting,   // a run is blocked on the operator: hops for attention
    Happy,     // a run just completed: hopping
    Sad,       // the last run failed: ears down, the flame gone to smoke
    Alarm,     // safety alert: flashing red
};
inline constexpr int kFireDogW = 28;   // sprite size, in sprite pixels
inline constexpr int kFireDogH = 14;
// Draws only, reserves no space. A hop lifts the sprite one `px` above
// `origin`, so leave that much headroom.
void DrawFireDog(ImDrawList* dl, ImVec2 origin, float px, FireDogMood mood, double t);

// Format an amp value with an auto-scaled unit (mA / uA / nA / pA).
std::string FormatCurrent(double a);

// Achieved sample rate from the samples in the last 1 s of a stream snapshot.
float MeasuredHz(const std::vector<float>& t);

// Open a folder in Windows Explorer (best-effort). Paths from the session are
// cwd-relative (products/ lives next to the exe), so resolve to absolute; the
// folder may not exist yet (product data/ before the first DUT) — create it.
void OpenFolderInExplorer(const std::string& dir);

} // namespace firedog
