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

// The object a grip run MEASURED in the jaws. Its own hue - not the tip's
// amber (a belief), not the detection's green (this frame): it is a point
// that was found on an earlier frame and is held until the next run or a
// Clear, so it must read as neither.
constexpr ImU32 kGripObjCol   = IM_COL32(90, 220, 235, 255);
constexpr ImU32 kGripOkCol    = IM_COL32(120, 240, 150, 255);
constexpr ImU32 kGripBadCol   = IM_COL32(245, 120, 100, 255);
constexpr ImU32 kGripPlateCol = IM_COL32(220, 225, 235, 255);

// The tool's own footprint around that tip. Same amber, dimmer than the cross:
// it is the SAME claim - one registration - drawn at the size the operator
// typed for it, not a second thing that was measured.
constexpr ImU32 kTipBoxCol    = IM_COL32(255, 190, 70, 150);
// ...and dimmer again when the tip is not near the plane in focus. Position and
// size both go through a pixel->mm calibration that belongs to THAT plane, so a
// tip 3 mm above it is drawn the right shape at the wrong magnification. The
// cross never had a size to be wrong about; the outline does.
constexpr ImU32 kTipBoxFarCol = IM_COL32(255, 190, 70, 60);

// The tool's footprint on the part is a DrawWorldBox (ui/Widgets) centred on
// the predicted tip: the size TYPED on the Registration page, width along the
// reference device's X and length along its Y, turned by the device's live U.
// A cross says where a tool is and never how much of the part it COVERS, which
// at a 3 mm field of view is the whole question this overlay is asked. A
// registration with no footprint draws no outline - absence is the signal here
// as everywhere else on this overlay.
//
// A gripped object's outline is the same call hung off tip + offset. Same
// projection, same spin, so the two boxes cannot drift apart.
constexpr ImU32 kGripObjBoxCol = IM_COL32(90, 220, 235, 170);

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
constexpr double kAuxPosAtTolMM = kSavedPosAtTolMM;   // core/SavedPositions.h

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
        std::string cursorLbl;   // drawn in the bottom-right stack, further down
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

            // The world distance is NOT drawn at the cursor any more - a label
            // riding the mouse covers exactly what is being aimed at. It is
            // parked in the bottom-right readout stack below, once the corner's
            // other occupants have said how much room they took.
            char lbl[96];
            std::snprintf(lbl, sizeof(lbl), "dX %+.3f mm   dY %+.3f mm", dxMM, dyMM);
            cursorLbl = lbl;

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
        // ---- The last grip measurement, held until replaced or cleared ----
        // Drawn before the tips because it owns the bottom-right corner: the
        // tips' "no tip:" lines stack above its plate.
        float gripPlateH = 0.0f;
        if (haveImg) gripPlateH = DrawAuxGripResult(dl, origin, avail);

        float tipsH = 0.0f;
        if (haveImg && showGripperMarks_) {
            const float scale = fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_));
            tipsH = DrawAuxGripperMarks(dl, origin, avail, center, scale, gripPlateH);
        }

        // ---- The cursor's world distance, parked bottom-right ----
        // TOP of the corner stack (grip plate, then the tools' dZ lines, then
        // this), so the lines under it do not jump each time the cursor enters
        // or leaves the image.
        if (!cursorLbl.empty()) {
            const ImVec2 lts = ImGui::CalcTextSize(cursorLbl.c_str());
            const ImVec2 lp(origin.x + avail.x - lts.x - 12.0f,
                            origin.y + avail.y - 8.0f - gripPlateH - tipsH -
                                ImGui::GetTextLineHeightWithSpacing());
            dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                              ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                              IM_COL32(0, 0, 0, 165), 3.0f);
            dl->AddText(lp, IM_COL32(255, 235, 150, 255), cursorLbl.c_str());
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
        if (auxGrip_.have) {
            if (ImGui::Button("Clear grip result")) {
                FD_INFO("grip", "Aux readout of '%s' cleared", auxGrip_.name.c_str());
                auxGrip_ = AuxGripReadout{};
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Drop the last grip measurement from the picture. It is\n"
                                  "a readout only - the profile keeps what was measured.");
        }
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
// The tip also gets its FOOTPRINT, drawn to scale by DrawWorldBox - a cross
// alone answers 'where is the gripper' and never 'is it standing on the part',
// which at these fields of view is the question being asked.
//
// Off-screen is the case that matters, and DrawWorldMark is what handles it:
// a tip 8 mm to the left is not merely invisible - nothing on the picture
// separates 'the gripper is over there' from 'the gripper is where I am
// looking'. Same helper the pick window uses, so an edge arrow means the same
// thing on both surfaces.
float App::DrawAuxGripperMarks(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail,
                               const ImVec2& center, float scale, float bottomInset)
{
    (void)center;   // the projection goes through the shared CameraView
    (void)scale;    // ...which computes the same cover fit from the box
    if (pixelToMmX_ <= 0.0f || pixelToMmY_ <= 0.0f) return 0.0f;
    double gx = 0.0, gy = 0.0;
    if (!workflow_.GantryXY(gx, gy)) return 0.0f;

    // Which device this image's pose belongs to, asked of layer 2 rather than
    // assumed: a UI overlay must not go hunting for a device called
    // "gantry-main".
    const std::string refName = workflow_.GantryName();
    if (refName.empty()) return 0.0f;
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
    // Each drawn tip's dZ line, for the bottom-right readout. On the mark it
    // overlapped the next tip's plate whenever two tools sat near each other
    // or both ran off the same edge; in the corner every line has its own row.
    std::vector<std::string> dzLines;

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

        // The mark carries the device NAME only (with its edge arrow when off
        // screen); the dZ goes to the corner readout, which says 'reg' so a
        // number there cannot read as an observation of the tool.
        const std::string lbl = name;
        {
            char line[128];
            if (!haveGz)
                std::snprintf(line, sizeof(line), "%s reg", name.c_str());
            else if (atFocus)
                std::snprintf(line, sizeof(line), "%s reg  in focus", name.c_str());
            else
                std::snprintf(line, sizeof(line), "%s reg  dZ %+.3f mm", name.c_str(), dz);
            dzLines.push_back(line);
        }

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
        DrawWorldBox(dl, wv, tip.x, tip.y, 0.0, 0.0, r.toolWmm, r.toolLmm, devU,
                     (haveGz && !atFocus) ? kTipBoxFarCol : kTipBoxCol);

        // What the tool is HOLDING, where the profile says it sits: the object's
        // outline hung off tip + offset, so it rides along as the device moves.
        // The offset is the last run's when there is one - this pick - else the
        // reference; a profile with neither has no point to hang a box off.
        // Drawn, never judged: it is the operator's picture of the part in the
        // jaws, and the only thing the profile judges is the offset.
        for (const GripProfile& gp : session_.Grips()) {
            if (gp.device != name || !GripHasOutline(gp)) continue;
            double dx = 0.0, dy = 0.0;
            if (!gp.measuredAt.empty()) { dx = gp.lastDx; dy = gp.lastDy; }
            else if (gp.haveRef)        { dx = gp.refDx;  dy = gp.refDy;  }
            else continue;
            DrawWorldBox(dl, wv, tip.x + dx, tip.y + dy, gp.anchorDx, gp.anchorDy,
                         gp.outWmm, gp.outLmm, devU, kGripObjBoxCol);
        }

        CameraMark cm;
        cm.wx    = tip.x;
        cm.wy    = tip.y;
        cm.have  = true;
        cm.label = lbl.c_str();
        cm.col   = kTipCol;
        DrawWorldMark(dl, wv, cm, 16.0f);
    }

    // Bottom-right, stacked upward: the opposite corner from the position plate,
    // and out of the way of a mark that is on screen.
    // The drawn tips' dZ first (nearest the corner - the lines read most
    // often), then the ones that could not be drawn.
    const float lineH = ImGui::GetTextLineHeightWithSpacing();
    const float y0    = origin.y + avail.y - 8.0f - bottomInset - lineH;
    float       y     = y0;
    const auto put = [&](const char* line, ImU32 col) {
        const ImVec2 ts = ImGui::CalcTextSize(line);
        const ImVec2 at(origin.x + avail.x - ts.x - 12.0f, y);
        if (at.x < origin.x + 4.0f) return;   // narrower than the text: say nothing
        dl->AddRectFilled(ImVec2(at.x - 4.0f, at.y - 2.0f),
                          ImVec2(at.x + ts.x + 4.0f, at.y + ts.y + 2.0f),
                          IM_COL32(0, 0, 0, 165), 3.0f);
        dl->AddText(at, col, line);
        y -= lineH;
    };
    for (const std::string& d : dzLines) put(d.c_str(), kTipCol);
    for (const std::string& g : gone) {
        char line[192];
        std::snprintf(line, sizeof(line), "no tip: %s", g.c_str());
        put(line, kTipGoneCol);
    }
    return y0 - y;   // the rows used, so the cursor readout stacks above them
}

// The last grip measurement, on the picture and in words.
//
// A grip run reports where the object sat relative to the tool that held it,
// and the number goes to the log and to the profile's "last measured" - both
// of which are somewhere else. This keeps it in front of the operator: the
// object's cross where it was FOUND (projected through the live pose, so it
// stays on the spot as the stage moves and gets an edge arrow when it leaves
// the frame), and a plate with the offset and the verdict. It holds until the
// next measurement replaces it or the strip's Clear drops it - unlike a
// detection marker, which fades: a detection describes one frame, this
// describes one grip, and the grip is still in the jaws.
//
// The cross is drawn only when the picture's pose belongs to the device the
// measurement was made through, for the same reason as the tips: a point in
// gantry-main's axis space means nothing on a picture posed by another device.
float App::DrawAuxGripResult(ImDrawList* dl, const ImVec2& origin, const ImVec2& avail)
{
    if (!auxGrip_.have) return 0.0f;
    const AuxGripReadout& g = auxGrip_;

    // ---- The plate, bottom-right ----
    char l1[160], l2[160], l3[160];
    std::snprintf(l1, sizeof(l1), "%s in %s", g.name.c_str(), g.device.c_str());
    std::snprintf(l2, sizeof(l2), "offset %+.4f, %+.4f mm  (%.4f)", g.dx, g.dy,
                  std::sqrt(g.dx * g.dx + g.dy * g.dy));
    if (g.ok) std::snprintf(l3, sizeof(l3), "OK  %s", g.at.c_str());
    else      std::snprintf(l3, sizeof(l3), "%s  %s", g.why.c_str(), g.at.c_str());

    const float lineH = ImGui::GetTextLineHeightWithSpacing();
    const float pad   = 6.0f;
    float w = 0.0f;
    for (const char* s : {l1, l2, l3}) w = fmaxf(w, ImGui::CalcTextSize(s).x);
    const float boxW = w + pad * 2.0f;
    const float boxH = lineH * 3.0f + pad * 2.0f;
    const float x1   = origin.x + avail.x - 6.0f;
    const float y1   = origin.y + avail.y - 6.0f;
    const float x0   = x1 - boxW;
    const float y0   = y1 - boxH;
    if (x0 < origin.x + 4.0f) return 0.0f;   // narrower than the text: say nothing

    dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(0, 0, 0, 165), 4.0f);
    dl->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), g.ok ? kGripOkCol : kGripBadCol, 4.0f);
    float y = y0 + pad;
    dl->AddText(ImVec2(x0 + pad, y), kGripObjCol, l1);   y += lineH;
    dl->AddText(ImVec2(x0 + pad, y), kGripPlateCol, l2); y += lineH;
    dl->AddText(ImVec2(x0 + pad, y), g.ok ? kGripOkCol : kGripBadCol, l3);
    const float plateH = boxH + 6.0f;

    // ---- The cross, where the object was found ----
    if (pixelToMmX_ <= 0.0f || pixelToMmY_ <= 0.0f) return plateH;
    double gx = 0.0, gy = 0.0;
    if (!workflow_.GantryXY(gx, gy)) return plateH;
    const std::string refName = workflow_.GantryName();
    if (refName.empty()) return plateH;
    // The measurement is in the axis space of the device that CARRIES the
    // camera - the gripper's registration names it - and the picture is posed
    // by the gantry. If those are not one device the cross has no meaning here.
    const DeviceReg reg = workflow_.DeviceRegFor(g.device);
    if (!reg.valid || reg.refDevice != refName) return plateH;

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

    char lbl[96];
    std::snprintf(lbl, sizeof(lbl), "%s  %+.3f %+.3f", g.name.c_str(), g.dx, g.dy);
    CameraMark cm;
    cm.wx    = g.cx;
    cm.wy    = g.cy;
    cm.have  = true;
    cm.label = lbl;
    cm.col   = kGripObjCol;
    DrawWorldMark(dl, wv, cm, 14.0f);
    return plateH;
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

void App::DrawSignalOverlay()
{
    DrawKeithleyChart(ImVec2(-1, -1));   // fill the overlay child
}

} // namespace firedog
