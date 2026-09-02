// WndAux.cpp : the Aux window — camera image + Keithley chart in one panel,
// positioned outside the main viewport on first use so multi-viewport spawns it
// as its own OS window (drag it to a second monitor). Definitions of App
// members — see App.h.
//
// Click-to-move maps an image click through the pixel->mm calibration to
// Workflow::MoveGantryRelativeMM. It drives the real gantry, so it defaults OFF
// and must be armed explicitly; the cursor crosshair + dX/dY readout are only
// drawn while it is armed, since they describe the move a click would command.
//
// The position plate at the bottom-left names where each stage stands: the live
// pose of gantry-main / hex-left / hex-right, matched to the nearest saved
// position of that device. Four decimal places of axis readout do not tell an
// operator anything; "See Focus Lens" does.
//
// The Z arrows (gantry-main, up = Z-, down = Z+) are ordinary jog: they go
// through Workflow::JogGantryZ and are gated by the shared jogArmed_ flag. The
// Autofocus button beside them is the second launcher of the sweep the Jog
// window's Camera tab owns — same App state, same StartQuickAutoFocus, so the
// two can never disagree about what a sweep does.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/SavedPositions.h"
#include "core/Log.h"
#include "core/Topics.h"
#include "ui/UiSettings.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace firedog {

namespace {
// Size of the cursor crosshair drawn over the image, in screen px.
constexpr float kCursorCrossPx = 30.0f;

// The detection marker's box is 20x20 CAMERA pixels — a true footprint, so its
// on-screen size tells you how big the feature is relative to the frame. At the
// Aux window's cover fit that is only a few screen px on a large sensor, which
// is why the crosshair arms extend a fixed distance BEYOND it: the box is the
// measurement, the arms are what makes it findable.
constexpr float kDetBoxCamPx  = 20.0f;
constexpr float kDetArmScreen = 14.0f;

// How long a detection stays on screen. A marker that lingers over a scene the
// stage has since left is worse than no marker at all.
constexpr float kDetHoldSec = 2.5f;

constexpr ImU32 kDetHitCol  = IM_COL32(80, 240, 130, 255);
constexpr ImU32 kDetMissCol = IM_COL32(245, 110, 90, 255);
constexpr ImU32 kE1Col      = IM_COL32(120, 190, 255, 255);
constexpr ImU32 kE2Col      = IM_COL32(200, 140, 255, 255);

// A registered device's tool tip. Amber rather than the E points' blue and the
// detection's green: it is neither a measurement of this frame nor a point on
// the part, it is where the machine BELIEVES a tool is - and a reader should
// not be able to confuse the three at a glance.
constexpr ImU32 kTipCol     = IM_COL32(255, 190, 70, 255);
// How near the focused plane a tip counts as being AT it, in mm of the
// reference device's Z. Only a label threshold - nothing acts on it.
constexpr double kTipAtFocusMM = 0.050;
// The plate that says a tip is missing ON PURPOSE. Dim, because it is not an
// alarm; present, because a cross that is simply absent is indistinguishable
// from an overlay that is broken - which is exactly how it was read.
constexpr ImU32 kTipGoneCol = IM_COL32(200, 170, 110, 220);

// The tool's own footprint around that tip. Same amber, dimmer than the cross:
// it is the SAME claim - one registration - drawn at the size the operator
// typed for it, not a second thing that was measured.
constexpr ImU32 kTipBoxCol    = IM_COL32(255, 190, 70, 150);
// ...and dimmer again when the tip is not near the plane in focus. Position and
// size both go through a pixel->mm calibration that belongs to THAT plane, so a
// tip 3 mm above it is drawn the right shape at the wrong magnification. The
// cross never had a size to be wrong about; the outline does.
constexpr ImU32 kTipBoxFarCol = IM_COL32(255, 190, 70, 60);
// Below this on-screen half-size an outline is not a size, it is a slightly
// fatter dot beside the cross's own 5 px ring - and two glyphs that mean
// different things at the same scale is worse than one.
constexpr float kTipBoxMinPx  = 7.0f;

// The tool's footprint on the part, centred on the predicted tip: a rectangle
// of the width and length the registration holds (DeviceReg::toolWmm/toolLmm),
// in machine mm, so it grows and shrinks with the picture under it instead of
// staying a screen-sized glyph.
//
// A cross says where a tool is and never says how much of the part it COVERS.
// At a 3 mm field of view that is the whole question an operator is asking of
// this overlay - a gripper clear of a feature and a gripper standing on it are
// the same cross. The size drawn is the one TYPED on the Registration page: the
// jaws' outline as the operator knows it, width along the reference device's X
// and length along its Y. A registration with no footprint draws no outline -
// absence is the signal here as it is everywhere else on this overlay.
//
// mm -> screen goes through pixelToMm on each axis SEPARATELY, because the two
// need not be equal: on non-square pixels a square tool is a rectangle on
// screen, and forcing it square would draw a tool that is not the one on the
// machine.
//
// `deg` spins it about the tip - the device's live U, so the outline turns with
// the jaws. Screen-space, the same way the MarkPicker spins its boxes: a
// rotation stays a rotation only where the scale is the same on both axes.
void DrawTipBox(ImDrawList* dl, const WorldMarkView& wv,
                double wx, double wy, double wMm, double lMm, double deg, ImU32 col)
{
    if (!dl || wMm <= 0.0 || lMm <= 0.0 || wv.pixelToMmX <= 0.0f || wv.pixelToMmY <= 0.0f)
        return;

    const float rx = static_cast<float>(wMm * 0.5 /
                                        static_cast<double>(wv.pixelToMmX)) * wv.view.scale;
    const float ry = static_cast<float>(lMm * 0.5 /
                                        static_cast<double>(wv.pixelToMmY)) * wv.view.scale;
    if (rx < kTipBoxMinPx && ry < kTipBoxMinPx) return;

    // The centre, through the same projection DrawWorldMark uses - a second
    // copy of it would put the outline off the cross it belongs to.
    const float px = static_cast<float>((wx - wv.gx) / static_cast<double>(wv.pixelToMmX)) +
                     static_cast<float>(wv.imgW) * 0.5f;
    const float py = static_cast<float>((wy - wv.gy) / static_cast<double>(wv.pixelToMmY)) +
                     static_cast<float>(wv.imgH) * 0.5f;
    const ImVec2 c = wv.view.ToScreen(px, py);

    // Wholly outside the box: nothing to draw. The CROSS handles that case with
    // an edge arrow, and an arrow per outline as well would say the same thing
    // twice. A footprint whose centre is off screen while its rim is not still
    // draws - that rim is exactly the news. Tested on the turned box's
    // bounding radius, so a corner that swings into view is not culled.
    const float rr = sqrtf(rx * rx + ry * ry);
    if (c.x + rr < wv.boxMin.x || c.x - rr > wv.boxMax.x ||
        c.y + rr < wv.boxMin.y || c.y - rr > wv.boxMax.y) return;

    const float th = static_cast<float>(deg) * 3.14159265358979f / 180.0f;
    const float cs = cosf(th), sn = sinf(th);
    const auto  spin = [&](float x, float y) {
        return ImVec2(c.x + x * cs - y * sn, c.y + x * sn + y * cs);
    };

    // Unlike the cross, this has EXTENT, and a tool wider than the field of view
    // would otherwise paint across the control strip and the chart. Clipped to
    // the image box, which is what it is a statement about.
    dl->PushClipRect(wv.boxMin, wv.boxMax, true);
    dl->AddQuad(spin(-rx, -ry), spin(rx, -ry), spin(rx, ry), spin(-rx, ry), col, 1.5f);
    dl->PopClipRect();
}

// A small dark-plated label, kept inside the image box.
void PlateText(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail,
               ImVec2 at, ImU32 col, const char* text)
{
    const ImVec2 ts = ImGui::CalcTextSize(text);
    at.x = fminf(at.x, origin.x + avail.x - ts.x - 6.0f);
    at.y = fminf(at.y, origin.y + avail.y - ts.y - 4.0f);
    at.x = fmaxf(at.x, origin.x + 4.0f);
    at.y = fmaxf(at.y, origin.y + 4.0f);
    dl->AddRectFilled(ImVec2(at.x - 4.0f, at.y - 2.0f),
                      ImVec2(at.x + ts.x + 4.0f, at.y + ts.y + 2.0f),
                      IM_COL32(0, 0, 0, 165), 3.0f);
    dl->AddText(at, col, text);
}

// The stages the position plate names, in the order it lists them. Config
// names, resolved through Workflow::DevicePose — a device the machine does not
// have simply drops out of the list.
const char* const kAuxPosDevices[] = {"gantry-main", "hex-left", "hex-right"};
constexpr int kAuxPosDeviceCount = static_cast<int>(sizeof(kAuxPosDevices) / sizeof(kAuxPosDevices[0]));

// How close counts as "at" that position. Deliberately much looser than the
// move layer's kMoveTolMM: this labels where the stage IS, with encoder noise
// and whatever drift a jog left behind, not whether a commanded move finished.
constexpr double kAuxPosAtTolMM = 0.050;

constexpr ImU32 kPosAtCol   = IM_COL32(120, 235, 150, 255);   // sitting on it
constexpr ImU32 kPosNearCol = IM_COL32(225, 200, 130, 255);   // nearest, but away
constexpr ImU32 kPosDimCol  = IM_COL32(150, 150, 158, 255);   // nothing to say
constexpr ImU32 kPosDevCol  = IM_COL32(190, 190, 200, 255);   // the device column

// Step sizes offered by the Aux window's gantry-Z buttons — a short subset of
// the shared kSteps table (indices into it, so the mm values never drift).
constexpr int kAuxZSteps[]  = {5, 6, 9, 12, 14};   // 5um, 10um, 100um, 1mm, 5mm
constexpr int kAuxZStepCount = static_cast<int>(sizeof(kAuxZSteps) / sizeof(kAuxZSteps[0]));
} // namespace

void App::DrawAuxWindow()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + mv->Size.x + 40.0f, mv->Pos.y + 40.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(700.0f, 760.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Aux - Camera + Signal", &showAux_)) {
        UpdateCameraTexture();   // pull the newest camera frame into the GL texture

        // ---- Background: live camera fills the entire content region ----
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 avail  = ImGui::GetContentRegionAvail();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
        if (haveImg) {
            // "cover" fit: fill the box, preserve aspect, crop the overflow via UV.
            ImVec2 uv0(0.0f, 0.0f), uv1(1.0f, 1.0f);
            const float imgA = static_cast<float>(cameraW_) / static_cast<float>(cameraH_);
            const float boxA = avail.x / avail.y;
            if (boxA > imgA) { const float s = imgA / boxA; uv0.y = (1.0f - s) * 0.5f; uv1.y = 1.0f - uv0.y; }
            else             { const float s = boxA / imgA; uv0.x = (1.0f - s) * 0.5f; uv1.x = 1.0f - uv0.x; }
            ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, uv0, uv1);
        } else {
            dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), IM_COL32(18, 18, 22, 255));
            const char* msg = "waiting for camera feed...";
            const ImVec2 ts = ImGui::CalcTextSize(msg);
            dl->AddText(ImVec2(origin.x + (avail.x - ts.x) * 0.5f, origin.y + (avail.y - ts.y) * 0.5f),
                        IM_COL32(150, 150, 150, 255), msg);
            ImGui::Dummy(avail);   // reserve the region so layout below is correct
        }

        // ---- Interaction layer: crosshair + world-distance + click-to-move ----
        // An invisible button over the image captures hover/click. It's emitted
        // BEFORE the chart overlay child, so the chart (drawn later, on top) grabs
        // clicks in its corner and the stage won't move when you use the chart.
        ImGui::SetCursorScreenPos(origin);
        const bool imgClicked = ImGui::InvisibleButton("##imgInput", avail);
        const bool imgHovered = ImGui::IsItemHovered();
        const ImVec2 center(origin.x + avail.x * 0.5f, origin.y + avail.y * 0.5f);

        // Fixed center crosshair (the optical/stage reference).
        const ImU32 chair = IM_COL32(80, 230, 120, 200);
        dl->AddLine(ImVec2(origin.x, center.y), ImVec2(origin.x + avail.x, center.y), chair, 1.0f);
        dl->AddLine(ImVec2(center.x, origin.y), ImVec2(center.x, origin.y + avail.y), chair, 1.0f);

        // The cursor marker + world-distance readout only appear while
        // click-to-move is armed: they annotate the move a click would command,
        // so with the feature off they are just clutter over the image.
        //
        // The level is checked here and not only on the checkbox below, because
        // enableClickMove_ is PERSISTED in UiSettings: a technician who left it
        // on would otherwise hand an operator a live click-to-move surface at
        // the next sign-in, with no visible control to turn it off.
        if (haveImg && imgHovered && enableClickMove_ && CanUse(kRoleJog)) {
            const ImVec2 m = ImGui::GetIO().MousePos;
            // Screen -> camera pixels. The image is "cover"-fit with a single
            // uniform scale, so 1 camera px = `scale` screen px on both axes.
            const float scale = fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_));
            const float dxPix = (m.x - center.x) / scale;         // camera px from center
            const float dyPix = (m.y - center.y) / scale;
            const float dxMM  = dxPix * pixelToMmX_;              // real-world offset
            const float dyMM  = dyPix * pixelToMmY_;

            // Line from center to cursor + a 30x30 px crosshair on the cursor
            // (red: armed, so a click will command the gantry).
            const ImU32 markCol = IM_COL32(255, 90, 90, 255);
            dl->AddLine(center, m, IM_COL32(255, 220, 90, 160), 1.0f);
            const float mh = kCursorCrossPx * 0.5f;
            dl->AddLine(ImVec2(m.x - mh, m.y), ImVec2(m.x + mh, m.y), markCol, 1.5f);
            dl->AddLine(ImVec2(m.x, m.y - mh), ImVec2(m.x, m.y + mh), markCol, 1.5f);

            // World-distance label near the cursor (kept inside the image).
            char lbl[96];
            std::snprintf(lbl, sizeof(lbl), "dX %+.3f mm   dY %+.3f mm", dxMM, dyMM);
            const ImVec2 lts = ImGui::CalcTextSize(lbl);
            ImVec2 lp(m.x + 14.0f, m.y + 8.0f);
            lp.x = fminf(lp.x, origin.x + avail.x - lts.x - 8.0f);
            lp.y = fminf(lp.y, origin.y + avail.y - lts.y - 6.0f);
            dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                              ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                              IM_COL32(0, 0, 0, 165), 3.0f);
            dl->AddText(lp, IM_COL32(255, 235, 150, 255), lbl);

            // Left-click -> move gantry-main (ACS) by this XY offset, via layer 2.
            if (imgClicked) workflow_.MoveGantryRelativeMM(dxMM, dyMM);
        }

        // ---- Vision feedback: the live detection + the E1/E2 marks ----
        // Drawn after the cursor annotations so a detection is never hidden
        // under them, and before the control plates so a checkbox stays clickable.
        if (haveImg && showVisionMarks_) {
            const float scale = fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_));
            DrawAuxVisionMarks(dl, origin, avail, center, scale);
        }

        // ---- Where the registered tools are ----
        // Its own toggle, not part of "Vision marks": a detection and an E point
        // are things that were SEEN, and a tip is a thing that was computed from
        // a registration. Reading the second as the first is how a rotated
        // hexapod goes on looking placed.
        if (haveImg && showGripperMarks_) {
            const float scale = fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_));
            DrawAuxGripperMarks(dl, origin, avail, center, scale);
        }

        // ---- Where each stage stands, named ----
        // Bottom-left, clear of the control plate above and the chart opposite.
        if (showPosLabels_) DrawAuxPositionLabels(dl, origin, avail);

        // ---- Overlay: camera status, top-left (dark plate for readability) ----
        char camline[128];
        std::snprintf(camline, sizeof(camline), "%s  [%s]  %dx%d",
                      hardware_.CameraName().c_str(), StateText(hardware_.CameraState()),
                      cameraW_, cameraH_);
        const ImVec2 cts = ImGui::CalcTextSize(camline);
        dl->AddRectFilled(ImVec2(origin.x + 6.0f, origin.y + 6.0f),
                          ImVec2(origin.x + 14.0f + cts.x, origin.y + 12.0f + cts.y),
                          IM_COL32(0, 0, 0, 140), 4.0f);
        dl->AddText(ImVec2(origin.x + 10.0f, origin.y + 9.0f), IM_COL32(230, 230, 235, 255), camline);

        // ---- Overlay: the view strip, just under the status plate ----
        // A vertical strip down the LEFT edge, one control per line, and
        // collapsible to a single button. It sits on top of the picture, and
        // the picture is the reason the window exists: a row of checkboxes
        // wide enough to name six things covered the part being watched. One
        // click puts it away, and the state is persisted, so an operator who
        // wants the frame clean keeps it clean across sessions.
        //
        // Still a child window, for the reason it always was: a child blocks
        // the image's input button beneath it, so ticking a box can never
        // also command a click-to-move.
        ImGui::SetCursorScreenPos(ImVec2(origin.x + 6.0f, origin.y + 12.0f + cts.y + 8.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.06f, 0.08f, 0.78f));
        ImGui::BeginChild("AuxControls", ImVec2(0.0f, 0.0f),
                          ImGuiChildFlags_AutoResizeX | ImGuiChildFlags_AutoResizeY |
                              ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar);

        // The handle. Collapsed it is the whole strip, so it has to say what
        // it opens - an arrow alone over a camera image reads as a jog button.
        if (ImGui::Button(auxPanelOpen_ ? "<  view###auxStrip" : ">###auxStrip")) {
            auxPanelOpen_ = !auxPanelOpen_;
            SaveUiSettings();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(auxPanelOpen_
                                  ? "Collapse this strip - the picture is what this\n"
                                    "window is for."
                                  : "Chart, click-to-move, jog, and the three image\n"
                                    "overlays.");

        if (auxPanelOpen_) {
        if (ImGui::Checkbox("Chart", &showChart_)) SaveUiSettings();
        // The Aux window is Operator-level because it MONITORS. Everything on
        // it that MOVES is Technician, the same level as the Jog window - this
        // strip would otherwise be a way around that gate, and it arms the very
        // same jogArmed_ bit.
        if (CanUse(kRoleJog)) {
            if (ImGui::Checkbox("Click-to-move", &enableClickMove_)) SaveUiSettings();
            ImGui::Checkbox("Arm jog", &jogArmed_);   // same flag the Jog window arms
        }
        ImGui::Separator();
        if (ImGui::Checkbox("Vision marks", &showVisionMarks_)) SaveUiSettings();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Overlay the last detection (20x20 camera-px box) and the\n"
                              "alignment's E1/E2 points, projected from machine mm through\n"
                              "the live gantry pose - so they stay on the feature as the\n"
                              "stage moves. Purely a readout; it changes nothing.");
        if (ImGui::Checkbox("Grippers", &showGripperMarks_)) SaveUiSettings();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Draw each REGISTERED device's tool tip where the record\n"
                              "says it is, projected through the live gantry pose - with\n"
                              "an edge arrow and its distance when the tip is outside the\n"
                              "frame, and dZ against the plane in focus.\n"
                              "Around the cross, the tool's own width to scale: the mark the\n"
                              "imprint measured, so it says how much of the part the tool\n"
                              "covers and not merely where it is. Faded when the tip is off\n"
                              "the focused plane (the mm per pixel it is drawn at belongs to\n"
                              "that plane), and absent for a registration whose tip was\n"
                              "taught without a width.\n"
                              "A tip that cannot be predicted (no registration, or the\n"
                              "device rotated away from where its tip was taught) gets NO\n"
                              "cross - never a stale one - and a dim \"no tip:\" line in the\n"
                              "bottom-right corner saying which and why.\n"
                              "Teach one on the Registration page (F8).");
        if (ImGui::Checkbox("Positions", &showPosLabels_)) SaveUiSettings();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Name where each stage stands (bottom-left): the live pose of\n"
                              "gantry-main, hex-left and hex-right matched to the nearest\n"
                              "saved position of that device. Green = sitting on it (within\n"
                              "%.3f mm); amber shows how far off the nearest one it is.",
                              kAuxPosAtTolMM);
        ImGui::Indent(14.0f);
        ImGui::BeginDisabled(!showPosLabels_);
        if (ImGui::Checkbox("+hidden", &posLabelsHidden_)) SaveUiSettings();
        ImGui::EndDisabled();
        ImGui::Unindent(14.0f);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Let hidden (system) positions - a procedure's private\n"
                              "algn_... stations - be named too. ON is the honest answer:\n"
                              "with it off the plate names the nearest VISIBLE position,\n"
                              "which may not be the one the stage is actually sitting on.");

        ImGui::Separator();

        // ---- gantry-main Z + Autofocus: motion, so Technician and up -------
        // Both drive the same axis, so they stand or fall together. Absent
        // rather than disabled below that level: an arrow that never moves
        // anything is worse than no arrow.
        if (CanUse(kRoleJog)) {
        // ---- gantry-main Z: up = Z-, down = Z+, with a step-size combo ----
        // Real motion, so it follows the jog arming rule. Layer 2 owns the
        // kinematics (JogGantryZ finds the gantry and applies the transform).
        if (auxZStepIndex_ < 0 || auxZStepIndex_ >= kAuxZStepCount) auxZStepIndex_ = 2;
        const double zstep = kSteps[kAuxZSteps[auxZStepIndex_]].mm;
        ImGui::BeginDisabled(!jogArmed_);
        ImGui::BeginGroup();
        if (ImGui::ArrowButton("##auxZUp", ImGuiDir_Up))   workflow_.JogGantryZ(-zstep);
        if (ImGui::ArrowButton("##auxZDown", ImGuiDir_Down)) workflow_.JogGantryZ(+zstep);
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::TextDisabled("gantry-main Z");
        ImGui::SetNextItemWidth(110.0f);
        if (ImGui::BeginCombo("##auxZStep", kSteps[kAuxZSteps[auxZStepIndex_]].label)) {
            for (int i = 0; i < kAuxZStepCount; ++i)
                if (ImGui::Selectable(kSteps[kAuxZSteps[i]].label, auxZStepIndex_ == i))
                    auxZStepIndex_ = i;
            ImGui::EndCombo();
        }
        ImGui::EndGroup();
        ImGui::EndDisabled();

        // ---- Autofocus, over the live image -------------------------------
        // The second launcher of the SAME sweep as the Jog window's Camera tab
        // (shared App state, one StartQuickAutoFocus): this is where an operator
        // actually is — watching the picture — when they decide focus is off.
        // Range/step stay in the Camera tab; only the direction is here, because
        // that is the knob a sweep ending on an edge tells you to change.
        if (afDirIndex_ < 0 || afDirIndex_ >= kAfDirectionCount) afDirIndex_ = 0;
        const bool afBusy  = orch_.Busy();
        const bool afReady = jogArmed_ && !afBusy &&
                             hardware_.CameraState() == ConnState::Connected;
        char afLabel[40];
        std::snprintf(afLabel, sizeof(afLabel), "Autofocus %s##auxAf",
                      kAfDirections[afDirIndex_].label);
        ImGui::BeginDisabled(!afReady);
        if (ImGui::Button(afLabel)) StartQuickAutoFocus();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Sweep gantry-main Z (%s) %.2f mm from here in %.3f mm steps\n"
                              "and stop at the sharpest Z. Range and step live in the\n"
                              "Jog window's Camera tab. Needs Arm jog + a connected camera.",
                              kAfDirections[afDirIndex_].value,
                              static_cast<double>(afRangeMM_),
                              static_cast<double>(afStepMM_));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(72.0f);
        if (ImGui::BeginCombo("##auxAfDir", kAfDirections[afDirIndex_].label)) {
            for (int i = 0; i < kAfDirectionCount; ++i)
                if (ImGui::Selectable(kAfDirections[i].label, afDirIndex_ == i)) afDirIndex_ = i;
            ImGui::EndCombo();
        }

        // Progress for OUR run only — the engine is shared with every other
        // Run surface, and this plate must not narrate somebody else's recipe.
        const RecipeEngine::Status afStatus = orch_.recipe.GetStatus();
        if (afStatus.recipeName == kQuickAutoFocusName) {
            if (afStatus.state == RecipeEngine::State::Running) {
                ImGui::TextUnformatted(afStatus.currentStep.c_str());
                if (ImGui::Button("Stop##auxAf")) orch_.StopAll();
            } else if (!afStatus.error.empty() &&
                       (afStatus.state == RecipeEngine::State::Failed ||
                        afStatus.state == RecipeEngine::State::Stopped)) {
                ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s",
                                   afStatus.error.c_str());
            }
        }
        }   // end of the Technician motion block opened above (Z + Autofocus).
            // Left un-indented deliberately: re-indenting ~60 lines would bury
            // the one line that changed in a diff nobody can read.
        }   // end of the collapsed/expanded strip.
        ImGui::EndChild();
        ImGui::PopStyleColor();

        // ---- Overlay: signal chart, pinned to the top-right corner ----
        if (showChart_) {
            const float margin = 10.0f;
            const float panelW = avail.x * 0.45f;
            const float panelH = avail.y * 0.40f;
            ImGui::SetCursorScreenPos(ImVec2(origin.x + avail.x - panelW - margin, origin.y + margin));
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.06f, 0.08f, 0.85f));
            ImGui::BeginChild("SignalOverlay", ImVec2(panelW, panelH), ImGuiChildFlags_Borders,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            DrawSignalOverlay();
            ImGui::EndChild();
            ImGui::PopStyleColor();
        }
    }
    ImGui::End();
}

// Vision feedback over the live image. Two kinds of mark, and the difference
// between them matters:
//
//   * the DETECTION is in camera pixels and belongs to one frame — it is only
//     meaningful right now, so it ages out;
//   * E1/E2 are in MACHINE millimetres and belong to the part. They are captured
//     at two different gantry poses, so a stored pixel would be a lie the moment
//     the stage moves; projecting them back through the live gantry pose is what
//     makes "E1 is over there" true at every station.
//
// The projection is the inverse of the capture in CommandsAlign.cpp:
//     wx = gx + (px - w/2) * pixel_to_mm_x    ->    (wx - gx) / pixel_to_mm_x
// so the two can only disagree if the calibration changes mid-run.
void App::DrawAuxVisionMarks(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail,
                             const ImVec2& center, float scale)
{
    const auto inBox = [&](const ImVec2& p) {
        return p.x >= origin.x && p.x <= origin.x + avail.x &&
               p.y >= origin.y && p.y <= origin.y + avail.y;
    };

    // ---- E1 / E2, projected from machine mm through the live gantry pose ----
    double gx = 0.0, gy = 0.0;
    if (workflow_.GantryXY(gx, gy) && pixelToMmX_ > 0.0f && pixelToMmY_ > 0.0f) {
        for (int i = 0; i < 2; ++i) {
            if (!alignMarkHave_[i]) continue;
            const float sx = center.x +
                static_cast<float>((alignMarkE_[i].wx - gx) / pixelToMmX_) * scale;
            const float sy = center.y +
                static_cast<float>((alignMarkE_[i].wy - gy) / pixelToMmY_) * scale;
            const ImVec2 p(sx, sy);
            if (!inBox(p)) continue;   // out of frame: say nothing rather than lie

            const ImU32 col = (i == 0) ? kE1Col : kE2Col;
            const float r = 9.0f;
            dl->AddCircle(p, r, col, 0, 1.8f);
            dl->AddLine(ImVec2(p.x - r - 6.0f, p.y), ImVec2(p.x - r + 2.0f, p.y), col, 1.8f);
            dl->AddLine(ImVec2(p.x + r - 2.0f, p.y), ImVec2(p.x + r + 6.0f, p.y), col, 1.8f);
            dl->AddLine(ImVec2(p.x, p.y - r - 6.0f), ImVec2(p.x, p.y - r + 2.0f), col, 1.8f);
            dl->AddLine(ImVec2(p.x, p.y + r - 2.0f), ImVec2(p.x, p.y + r + 6.0f), col, 1.8f);
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "E%d  %.4f, %.4f", i + 1,
                          alignMarkE_[i].wx, alignMarkE_[i].wy);
            PlateText(dl, origin, avail, ImVec2(p.x + 14.0f, p.y - 22.0f), col, lbl);
        }
    }

    // ---- The detection this frame (or the last few) ----
    if (auxDetAge_ > kDetHoldSec) return;
    // Fade out over the last half second, so it reads as "this is going stale"
    // rather than vanishing between one frame and the next.
    const float fade = auxDetAge_ < kDetHoldSec - 0.5f
                           ? 1.0f
                           : (kDetHoldSec - auxDetAge_) * 2.0f;
    const int alpha = static_cast<int>(255.0f * fminf(fmaxf(fade, 0.0f), 1.0f));

    if (!auxDet_.ok) {
        // A miss is worth drawing too: "it looked and found nothing" is
        // different information from "it never ran", and only one of them
        // means the parameters need work.
        char lbl[160];
        std::snprintf(lbl, sizeof(lbl), "no detection: %s", auxDet_.error.c_str());
        PlateText(dl, origin, avail,
                  ImVec2(center.x - 120.0f, origin.y + avail.y - 30.0f),
                  (kDetMissCol & 0x00FFFFFFu) | (static_cast<ImU32>(alpha) << 24), lbl);
        return;
    }

    const ImVec2 p(center.x + (static_cast<float>(auxDet_.xPx) -
                               static_cast<float>(cameraW_) * 0.5f) * scale,
                   center.y + (static_cast<float>(auxDet_.yPx) -
                               static_cast<float>(cameraH_) * 0.5f) * scale);
    if (!inBox(p)) return;

    const ImU32 col = (kDetHitCol & 0x00FFFFFFu) | (static_cast<ImU32>(alpha) << 24);
    const float half = kDetBoxCamPx * 0.5f * scale;   // the TRUE 20x20 camera-px box
    dl->AddRect(ImVec2(p.x - half, p.y - half), ImVec2(p.x + half, p.y + half),
                col, 0.0f, 0, 1.6f);
    const float arm = half + kDetArmScreen;           // ...plus a findable pointer
    dl->AddLine(ImVec2(p.x - arm, p.y), ImVec2(p.x - half, p.y), col, 1.6f);
    dl->AddLine(ImVec2(p.x + half, p.y), ImVec2(p.x + arm, p.y), col, 1.6f);
    dl->AddLine(ImVec2(p.x, p.y - arm), ImVec2(p.x, p.y - half), col, 1.6f);
    dl->AddLine(ImVec2(p.x, p.y + half), ImVec2(p.x, p.y + arm), col, 1.6f);

    char lbl[128];
    std::snprintf(lbl, sizeof(lbl), "%.1f, %.1f px   score %.3f   %.1f ms",
                  auxDet_.xPx, auxDet_.yPx, auxDet_.score, auxDet_.elapsedMs);
    PlateText(dl, origin, avail, ImVec2(p.x + arm + 4.0f, p.y + 6.0f), col, lbl);
}

// Where each registered device's TOOL TIP is, drawn on the live image.
//
// This is not a measurement and must not read like one. A detection is what
// the camera saw this frame; an E point is a place on the part somebody
// marked; a tip is COMPUTED - p = t + R*h, out of a registration whose
// rotation is a typed permutation out of the transforms file. It gets its own
// toggle, its own colour and the word 'reg' on its plate for that reason.
//
// The projection is the E marks' one, through the LIVE gantry pose, so the
// cross stays on the tool as the stage moves. Two rules carry over unchanged
// from the tip topics (see docs/device-registration-design.md):
//
//   * a tip that cannot be predicted draws NO POSITION - no registration, not
//     connected, or the device rotated away from where its tip was taught.
//     A readout frozen at the last good number is how a rotated hexapod goes
//     on looking placed. What it DOES draw is one dim line in the corner
//     naming the device and the refusal: absence is the signal, but silent
//     absence answers no question - an operator hunting for a gripper that is
//     not on screen cannot tell a refusal from a broken overlay, which is
//     exactly how it was read the first time.
//   * a tip registered against a DIFFERENT reference is not drawn either.
//     This image belongs to the camera's gantry, and a point in another
//     origin projected through this pose is off by the whole distance
//     between the two origins.
//
// The tip also gets its FOOTPRINT, drawn to scale by DrawTipBox above - a cross
// alone answers 'where is the gripper' and never 'is it standing on the part',
// which at these fields of view is the question being asked.
//
// Off-screen is the case that matters, and DrawWorldMark is what handles it:
// a tip 8 mm to the left is not merely invisible - nothing on the picture
// separates 'the gripper is over there' from 'the gripper is where I am
// looking'. Same helper the pick window uses, so an edge arrow means the same
// thing on both surfaces.
void App::DrawAuxGripperMarks(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail,
                              const ImVec2& center, float scale)
{
    (void)center;   // the projection goes through the shared CameraView
    (void)scale;    // ...which computes the same cover fit from the box
    if (pixelToMmX_ <= 0.0f || pixelToMmY_ <= 0.0f) return;
    double gx = 0.0, gy = 0.0;
    if (!workflow_.GantryXY(gx, gy)) return;

    // Which device this image's pose belongs to, asked of layer 2 rather than
    // assumed: a UI overlay must not go hunting for a device called
    // "gantry-main".
    const std::string refName = workflow_.GantryName();
    if (refName.empty()) return;
    // ...and its Z, so a tip can say how far it stands from the plane in
    // focus. That is the half of "where is the tool" a picture cannot show:
    // two tips 3 mm apart in Z land on the same pixel.
    double gz     = 0.0;
    bool   haveGz = false;
    {
        std::map<std::string, double> refPose;
        if (workflow_.DevicePose(refName, refPose)) {
            const auto iz = refPose.find("Z");
            if (iz != refPose.end()) { gz = iz->second; haveGz = true; }
        }
    }

    // The same cover fit the image above was drawn with (MakeCameraView uses
    // fmaxf too), centred on the image centre at zoom 1. Built from the shared
    // helper rather than by hand: two copies of this arithmetic is how an
    // overlay starts disagreeing with the picture under it.
    WorldMarkView wv;
    wv.view       = MakeCameraView(origin, avail, cameraW_, cameraH_,
                                   static_cast<float>(cameraW_) * 0.5f,
                                   static_cast<float>(cameraH_) * 0.5f, 1.0f);
    wv.boxMin     = origin;
    wv.boxMax     = ImVec2(origin.x + avail.x, origin.y + avail.y);
    wv.gx         = gx;
    wv.gy         = gy;
    wv.pixelToMmX = pixelToMmX_;
    wv.pixelToMmY = pixelToMmY_;
    wv.imgW       = cameraW_;
    wv.imgH       = cameraH_;

    // Every registration that COULD have been drawn and was not, with the reason.
    // Absence stays the signal - nothing here invents a position - but silent
    // absence answers no question: an operator looking for a gripper that is not
    // on screen cannot tell a refusal from a broken overlay, and the log says it
    // once per session.
    std::vector<std::string> gone;

    for (const auto& [name, r] : workflow_.DeviceRegs()) {
        if (!r.valid || r.refDevice != refName) continue;
        RegVec3     tip;
        std::string why;
        if (!workflow_.DeviceRegTip(name, tip, &why)) {
            // The head of the reason, up to its explanatory tail: "rotated to X
            // but taught at Y" fits a corner, the paragraph after the dash does
            // not. Elided rather than rewritten - a second wording would be a
            // second answer to why a tip is unavailable.
            const std::size_t cut = why.find(" - ");
            gone.push_back(cut == std::string::npos ? why : why.substr(0, cut));
            continue;
        }

        // How far the tip stands from the plane in focus. Wanted twice now: the
        // plate says it, and the outline below is drawn at that plane's
        // magnification, so it fades when the tip is nowhere near it.
        double     dz      = 0.0;
        const bool atFocus = haveGz && std::fabs(tip.z - gz) <= kTipAtFocusMM;
        if (haveGz) dz = tip.z - gz;

        // The plate names the device and says 'reg', because a cross that only
        // said 'hex-left' would read as an observation of one.
        char lbl[96];
        if (!haveGz)
            std::snprintf(lbl, sizeof(lbl), "%s reg", name.c_str());
        else if (atFocus)
            std::snprintf(lbl, sizeof(lbl), "%s reg  in focus", name.c_str());
        else
            std::snprintf(lbl, sizeof(lbl), "%s reg  dZ %+.3f", name.c_str(), dz);

        // How big the tool is, before the cross that says where it is: the
        // outline is context for the mark, so the mark and its plate stay
        // legible on top of it. Turned by the device's live U, as the jaws are.
        double devU = 0.0;
        {
            std::map<std::string, double> devPose;
            if (workflow_.DevicePose(name, devPose))
                if (const auto iu = devPose.find("U"); iu != devPose.end())
                    devU = iu->second;
        }
        DrawTipBox(dl, wv, tip.x, tip.y, r.toolWmm, r.toolLmm, devU,
                   (haveGz && !atFocus) ? kTipBoxFarCol : kTipBoxCol);

        CameraMark cm;
        cm.wx    = tip.x;
        cm.wy    = tip.y;
        cm.have  = true;
        cm.label = lbl;
        cm.col   = kTipCol;
        DrawWorldMark(dl, wv, cm, 16.0f);
    }

    // Bottom-right, stacked upward: the opposite corner from the position plate,
    // and out of the way of a mark that is on screen.
    const float lineH = ImGui::GetTextLineHeightWithSpacing();
    float       y     = origin.y + avail.y - 8.0f - lineH;
    for (const std::string& g : gone) {
        char line[192];
        std::snprintf(line, sizeof(line), "no tip: %s", g.c_str());
        const ImVec2 ts = ImGui::CalcTextSize(line);
        const ImVec2 at(origin.x + avail.x - ts.x - 12.0f, y);
        if (at.x < origin.x + 4.0f) break;   // narrower than the text: say nothing
        dl->AddRectFilled(ImVec2(at.x - 4.0f, at.y - 2.0f),
                          ImVec2(at.x + ts.x + 4.0f, at.y + ts.y + 2.0f),
                          IM_COL32(0, 0, 0, 165), 3.0f);
        dl->AddText(at, kTipGoneCol, line);
        y -= lineH;
    }
}

// Where each stage stands, named against the taught positions.
//
// The pose comes from layer 2 (Workflow::DevicePose) and the names from the
// session document, which is the only pairing that can be right: the positions
// belong to the loaded PRODUCT, so the same axis reading is "See Focus Lens"
// under one product and unnamed under another. Matching is the shared
// NearestSavedPos, so this plate and the Motion page's map cannot disagree
// about which taught point a pose is sitting on.
//
// Hidden positions rank by default (posLabelsHidden_): the plate answers where
// the stage IS, so dropping the true nearest would name a neighbour and call it
// home. The "+hidden" checkbox turns that off for when the algn_... names are
// noise — a readout preference, never a claim about the machine.
//
// It is drawn straight onto the draw list rather than into a child window
// because it is pure readout: a child would swallow the clicks that the image
// beneath it turns into click-to-move.
void App::DrawAuxPositionLabels(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail)
{
    struct Row { const char* dev; std::string text; ImU32 col; };
    std::vector<Row> rows;
    rows.reserve(static_cast<std::size_t>(kAuxPosDeviceCount));

    const SavedPosMap& all = session_.Positions();
    for (int i = 0; i < kAuxPosDeviceCount; ++i) {
        const char* dev = kAuxPosDevices[i];
        std::map<std::string, double> pose;
        if (!workflow_.DevicePose(dev, pose)) continue;   // absent or not connected

        const auto it = all.find(dev);
        double dist = 0.0;
        const SavedPos* p = (it != all.end())
                             ? NearestSavedPos(it->second, pose, &dist, posLabelsHidden_)
                             : nullptr;
        if (!p) { rows.push_back({dev, "(no taught position)", kPosDimCol}); continue; }

        char buf[96];
        const bool at = dist <= kAuxPosAtTolMM;
        if (at) std::snprintf(buf, sizeof(buf), "%s", p->name);
        else    std::snprintf(buf, sizeof(buf), "%s  +%.3f mm", p->name, dist);
        rows.push_back({dev, buf, at ? kPosAtCol : kPosNearCol});
    }
    if (rows.empty()) return;

    // Two columns: device, then what it is sitting on. The device column is
    // sized to the widest label so the names line up and read as a list.
    const float pad = 6.0f, gap = 10.0f;
    float devW = 0.0f, txtW = 0.0f;
    for (const Row& r : rows) {
        devW = fmaxf(devW, ImGui::CalcTextSize(r.dev).x);
        txtW = fmaxf(txtW, ImGui::CalcTextSize(r.text.c_str()).x);
    }
    const float lineH = ImGui::GetTextLineHeightWithSpacing();
    const float boxW  = devW + gap + txtW + pad * 2.0f;
    const float boxH  = lineH * static_cast<float>(rows.size()) + pad * 2.0f;

    const float x0 = origin.x + 6.0f;
    const float y1 = origin.y + avail.y - 6.0f;
    const float y0 = y1 - boxH;
    dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x0 + boxW, y1), IM_COL32(0, 0, 0, 165), 4.0f);

    float y = y0 + pad;
    for (const Row& r : rows) {
        dl->AddText(ImVec2(x0 + pad, y), kPosDevCol, r.dev);
        dl->AddText(ImVec2(x0 + pad + devW + gap, y), r.col, r.text.c_str());
        y += lineH;
    }
}

void App::DrawImageOverlay()
{
    ImGui::TextUnformatted("Vision");
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float w = 480.0f, h = 360.0f;
    ImGui::Image((ImTextureID)(intptr_t)viewportTex_, ImVec2(w, h));   // C-style cast: void*/ImU64 ImTextureID

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 c(origin.x + w * 0.5f, origin.y + h * 0.5f);
    dl->AddLine(ImVec2(c.x - 20, c.y), ImVec2(c.x + 20, c.y), IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddLine(ImVec2(c.x, c.y - 20), ImVec2(c.x, c.y + 20), IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddRect(ImVec2(origin.x + 120, origin.y + 90), ImVec2(origin.x + 300, origin.y + 240),
                IM_COL32(90, 250, 120, 255), 0.0f, 0, 2.0f);
    dl->AddText(ImVec2(origin.x + 122, origin.y + 72), IM_COL32(90, 250, 120, 255), "match 0.98");
}

void App::DrawSignalOverlay()
{
    DrawKeithleyChart(ImVec2(-1, -1));   // fill the overlay child
}

} // namespace firedog
