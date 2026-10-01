// WndJog.cpp : the Jog window (View menu) and the velocity combo it shares
// with the Motion page. Definitions of App members — see App.h.
//
// The window has four tabs: "Jog" (the motion fleet), "Camera" (the camera
// fleet — device combo + the Camera page's exposure controls, reused rather
// than copied so the seed/apply rule lives in one place, plus the live focus
// score and the Autofocus button), "Pos" (a 3x3 pad of quick moves to
// taught positions, assigned by right-click) and "IO" (a growable list of
// output rows — one button that toggles or pulses the pin, one dwell field).
//
// The Autofocus button does NOT sweep inline: it starts the `auto_focus`
// command as a one-step ad-hoc recipe on the engine worker, because the sweep
// blocks for tens of seconds and the UI thread may never block.
//
// Jog KINEMATICS are not here: the UI only picks a device, a step and a
// direction, and Workflow::JogGlobal maps that through the device's
// AxisTransform. Jog is armed via a checkbox and defaults OFF because the
// W/D-A/S-R/F keys drive real motion.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {
// Jog velocities (mm/s) per controller type.
constexpr double kVelPI[]  = {1.0, 2.0, 5.0, 10.0};
constexpr double kVelACS[] = {1.0, 5.0, 10.0, 20.0, 40.0};

// The taught position a Pos-tab pad slot points at, or null when the uid is
// unknown to the loaded product (another product, or the position was deleted).
const SavedPos* FindPosByUid(const std::vector<SavedPos>* list, const std::string& uid)
{
    if (!list || uid.empty()) return nullptr;
    for (const SavedPos& p : *list)
        if (p.uid == uid) return &p;
    return nullptr;
}
// DrawBigLabel (the step-size and device-name readouts below) lives in
// ui/Widgets now: the Manual Adjust window draws the same device box.
} // namespace

void App::DrawJogWindow()
{
    // Spawns near the top-right (visible); drag it out to make it its own OS
    // window (multi-viewport). &showJog_ gives it a close box that syncs the menu.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + mv->Size.x - 262.0f, mv->Pos.y + 60.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250.0f, 640.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Jog", &showJog_)) {
        // Five tabs so the window covers "move the stage", "see it", "go
        // somewhere taught" and "switch something": Jog = the motion fleet,
        // Camera = the camera fleet (device + exposure), Pos = the quick-move
        // pad, IO = the quick-output rows, PSU = the supply fleet's channels.
        // BeginTabItem pushes the tab's ID, so the Arm checkbox and STOP button
        // repeated on the Pos tab do not collide with the Jog tab's.
        if (ImGui::BeginTabBar("JogTabs")) {
            if (ImGui::BeginTabItem("Jog")) {
                DrawJogPanel();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Camera")) {
                DrawJogCameraTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Pos")) {
                DrawJogPosTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("IO")) {
                DrawJogIoTab();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("PSU")) {
                DrawJogPsuTab();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

// PSU tab of the Jog window: pick a supply, then its channels - the PSU page's
// own controls (DrawPsuChannelControls, compact), so the two surfaces share one
// selection, one set of edits and one apply path. Like the PSU page it carries
// no arm and does not gate on orch_.Busy(): it is the same supply the page
// drives, and switching an output while a run waits on a prompt is what quick
// access is for. Connect/disconnect stays on the PSU page, as the Camera tab
// leaves its connect to the Camera page.
void App::DrawJogPsuTab()
{
    // A page gate does not cover a window: hold this tab to the PSU page's own
    // level, so it can never become a way around it (today both are Technician).
    if (!CanAccess(Page::PSU)) {
        ImGui::TextDisabled("the power supplies need %s access", RoleName(MinRoleFor(Page::PSU)));
        return;
    }
    if (hardware_.PsuCount() == 0) {
        ImGui::TextDisabled("no PSU configured in hardware.json");
        return;
    }
    const std::size_t sel = PsuSelected();
    const ConnState   st  = hardware_.PsuState(sel);
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##psudev", hardware_.PsuName(sel).c_str())) {
        for (std::size_t i = 0; i < hardware_.PsuCount(); ++i) {
            const ConnState rst = hardware_.PsuState(i);
            ImGui::PushID(static_cast<int>(i));
            ImGui::TextColored(StateColor(rst), "%s", StateGlyph(rst));
            ImGui::SameLine();
            if (ImGui::Selectable(hardware_.PsuName(i).c_str(), i == sel) && i != sel) {
                psuSel_    = static_cast<int>(i);
                psuPulled_ = false;   // re-seed the edits from THIS supply
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    if (st != ConnState::Connected)
        ImGui::TextDisabled("%s - connect it on the PSU page", StateText(st));
    ImGui::Separator();
    DrawPsuChannelControls(true);
}

// Camera tab of the Jog window: pick the camera, set its exposure, watch the
// live focus score, run an autofocus sweep. The exposure widgets are the Camera
// page's (DrawCameraExposureControls), so both surfaces share one seeding rule
// and one apply path. Connect/disconnect stays on the Camera page — this tab
// only tunes a camera that is already up.
//
// The order is the bring-up order: get a camera, get the exposure right (the
// luma warning says when it is not), confirm the score responds, then let the
// sweep use it.
void App::DrawJogCameraTab()
{
    // Device combo. HardwareManager owns a single Pylon camera today, so this
    // lists one entry; the loop is what a second camera would grow into.
    const ConnState st = hardware_.CameraState();
    ImGui::TextUnformatted("Device");
    ImGui::SetNextItemWidth(-FLT_MIN);
    const std::string& camName = hardware_.CameraName();
    if (ImGui::BeginCombo("##camdev", camName.c_str())) {
        ImGui::Selectable(camName.c_str(), true);
        ImGui::EndCombo();
    }
    DeviceStatusLine(st, camName.c_str());

    ImGui::Separator();
    ImGui::TextUnformatted("Exposure");
    DrawCameraExposureControls();

    // ---- Live focus score ---------------------------------------------------
    // The same measure auto_focus maximises (Laplacian variance over a centred
    // ROI), sampled per frame while this is ticked. Two uses: sanity-check the
    // ROI and exposure before trusting a sweep, and watch the number peak while
    // jogging Z by hand. Off by default — it costs a score per frame.
    ImGui::Separator();
    ImGui::TextUnformatted("Focus");
    bool stream = workflow_.FocusStreaming();
    if (ImGui::Checkbox("Live score", &stream)) workflow_.SetFocusStreaming(stream, focusRoiPx_);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    constexpr int kRois[] = {0, 128, 256, 512, 1024};
    char roiLabel[24];
    if (focusRoiPx_ > 0) std::snprintf(roiLabel, sizeof(roiLabel), "%d px", focusRoiPx_);
    else                 std::snprintf(roiLabel, sizeof(roiLabel), "full frame");
    if (ImGui::BeginCombo("##focusroi", roiLabel)) {
        for (int r : kRois) {
            char b[24];
            if (r > 0) std::snprintf(b, sizeof(b), "%d px", r);
            else       std::snprintf(b, sizeof(b), "full frame");
            if (ImGui::Selectable(b, focusRoiPx_ == r)) {
                focusRoiPx_ = r;
                workflow_.SetFocusStreaming(workflow_.FocusStreaming(), focusRoiPx_);
            }
        }
        ImGui::EndCombo();
    }

    const Workflow::FocusSample f = workflow_.LatestFocus();
    if (!stream) {
        ImGui::TextDisabled("not sampling");
    } else if (!f.ok) {
        ImGui::TextDisabled("waiting for frames...");
    } else {
        ImGui::Text("%.1f", f.score);
        // Mean luma is the guard rail: a blown-out or black ROI makes the score
        // meaningless, and exposure is right above to fix it.
        if (f.meanLuma > 250.0 || f.meanLuma < 5.0)
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1.0f),
                               "luma %.0f - check exposure", f.meanLuma);
        else
            ImGui::TextDisabled("luma %.0f   frame %llu", f.meanLuma, f.frameSeq);
    }

    // ---- Autofocus (gantry-main Z, downward sweep) --------------------------
    // Real motion, so it follows the same arming rule as jog, and it gates on
    // orch_.Busy() like every other Run/Go surface. The sweep itself runs on the
    // engine worker as a one-step ad-hoc recipe — it blocks for tens of seconds,
    // which must never happen on the UI thread.
    ImGui::Separator();
    ImGui::TextUnformatted("Autofocus");
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragFloat("range mm", &afRangeMM_, 0.01f, 0.01f, 10.0f, "%.2f");
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragFloat("step mm", &afStepMM_, 0.001f, 0.001f, 1.0f, "%.3f");
    if (afDirIndex_ < 0 || afDirIndex_ >= kAfDirectionCount) afDirIndex_ = 0;
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::BeginCombo("direction", kAfDirections[afDirIndex_].label)) {
        for (int i = 0; i < kAfDirectionCount; ++i)
            if (ImGui::Selectable(kAfDirections[i].label, afDirIndex_ == i)) afDirIndex_ = i;
        ImGui::EndCombo();
    }

    const bool afBusy = orch_.Busy();
    const bool afReady = jogArmed_ && !afBusy && st == ConnState::Connected;
    char afLabel[32];
    std::snprintf(afLabel, sizeof(afLabel), "Autofocus  %s", kAfDirections[afDirIndex_].label);
    ImGui::BeginDisabled(!afReady);
    if (ImGui::Button(afLabel, ImVec2(-FLT_MIN, 0.0f))) StartQuickAutoFocus();
    ImGui::EndDisabled();
    // AllowWhenDisabled: the tooltip is most useful exactly when the button is
    // greyed out.
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Sweep gantry-main Z (%s) %.2f mm from here in %.3f mm\n"
                          "steps, scoring each step, then stop at the sharpest Z.\n"
                          "Needs Arm (Jog tab) and a connected camera.",
                          kAfDirections[afDirIndex_].value,
                          static_cast<double>(afRangeMM_), static_cast<double>(afStepMM_));

    // Why the button is unavailable, rather than a dead grey rectangle.
    if (!jogArmed_)                        ImGui::TextDisabled("arm jog first (Jog tab)");
    else if (st != ConnState::Connected)   ImGui::TextDisabled("camera not connected");
    else if (afBusy)                       ImGui::TextDisabled("another run is active");

    // Progress + result, but only for OUR run: the engine is shared, and a
    // recipe someone else started has no business reporting in this tab.
    const RecipeEngine::Status rs = orch_.recipe.GetStatus();
    if (rs.recipeName == kQuickAutoFocusName) {
        if (rs.state == RecipeEngine::State::Running) {
            ImGui::TextUnformatted(rs.currentStep.c_str());
            // STOP lives on the Jog tab, but a run started here should be
            // stoppable from here too.
            if (ImGui::Button("Stop##af", ImVec2(-FLT_MIN, 0.0f))) orch_.StopAll();
        } else if (!rs.error.empty() && (rs.state == RecipeEngine::State::Failed ||
                                         rs.state == RecipeEngine::State::Stopped)) {
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", rs.error.c_str());
        } else if (rs.state == RecipeEngine::State::Completed) {
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.45f, 1.0f), "focused");
        }
    }
    if (!afErr_.empty())
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", afErr_.c_str());
}

// ---------------------------------------------------------------------------
// Pos tab: pick a motion device, then drive it to a taught position with one
// click on a 3x3 pad. The pad is a SHORTCUT, never a second way to create or
// edit a position — teaching stays on the Motion page, and a slot only ever
// holds the uid of a position that page already owns.
//
// The device is jogDevice_, the SAME selection the Jog tab drives: one window,
// one device under its hands. The pad is keyed per device name, so switching
// device relays the nine buttons rather than re-pointing them.
//
// Assignment is a right-click POPUP, not a modal: a modal blocks input to every
// other window including the always-on-top prompt an engine worker may be
// waiting on, and choosing which button means "load station" is no reason to
// freeze a running machine. It stays reachable while disarmed too — laying out
// the pad moves nothing.
void App::DrawJogPosTab()
{
    const std::size_t n = hardware_.MotionCount();
    if (n == 0) { ImGui::TextDisabled("no motion devices"); return; }
    if (jogDevice_ < 0 || static_cast<std::size_t>(jogDevice_) >= n) jogDevice_ = 0;
    const std::size_t di      = static_cast<std::size_t>(jogDevice_);
    const std::string devName = hardware_.MotionName(di);
    const ConnState   st      = hardware_.MotionState(di);

    ImGui::TextUnformatted("Device");
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##posdev", devName.c_str())) {
        for (std::size_t i = 0; i < n; ++i) {
            if (!hardware_.MotionDevice(i)) continue;
            const ConnState s = hardware_.MotionState(i);
            ImGui::PushID(static_cast<int>(i));
            ImGui::TextColored(StateColor(s), "%s", StateGlyph(s));
            ImGui::SameLine();
            if (ImGui::Selectable(hardware_.MotionName(i).c_str(), jogDevice_ == static_cast<int>(i)))
                jogDevice_ = static_cast<int>(i);
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    DeviceStatusLine(st, devName.c_str());

    // --- Arm + STOP, the same members the Jog tab shows ----------------------
    // Repeated here rather than pointing at the other tab: a pad button starts a
    // full travel, so the thing that stops it must be under the same thumb.
    ImGui::Separator();
    ImGui::Checkbox("Arm##pos", &jogArmed_);
    ImGui::SameLine();
    if (jogArmed_) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "ARMED");
    else           ImGui::TextDisabled("disarmed");
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("STOP##pos", ImVec2(-FLT_MIN, 30.0f))) orch_.StopAll();
    ImGui::PopStyleColor(2);

    // --- Where the stage stands now, named -----------------------------------
    // Four decimals of axis readout do not tell an operator which pad button is
    // already under them; the nearest taught name does. Hidden positions count
    // (NearestSavedPos's default): this answers "where is it", and dropping the
    // true nearest would name a neighbour instead.
    const std::vector<SavedPos>* saved = nullptr;
    {
        const auto it = session_.Positions().find(devName);
        if (it != session_.Positions().end()) saved = &it->second;
    }
    ImGui::Separator();
    if (IMotionController* dev = hardware_.MotionDevice(di)) {
        double          dist = 0.0;
        const SavedPos* here = saved ? NearestSavedPos(*saved, dev->Positions(), &dist) : nullptr;
        if (here) ImGui::TextDisabled("at %s  (%.3f mm)", here->name, dist);
        else      ImGui::TextDisabled("at (nothing taught to name it)");
    }

    // --- The pad -------------------------------------------------------------
    std::vector<std::string>& slots = PosPadSlots(devName);

    const bool busy  = orch_.Busy();
    const bool ready = jogArmed_ && !busy && st == ConnState::Connected;

    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const float bw  = (ImGui::GetContentRegionAvail().x - gap * static_cast<float>(kPosPadCols - 1))
                      / static_cast<float>(kPosPadCols);
    const float bh  = ImGui::GetTextLineHeight() * 2.6f;

    for (int s = 0; s < kPosPadSlots; ++s) {
        if (s % kPosPadCols != 0) ImGui::SameLine();
        ImGui::PushID(s);

        const SavedPos* p        = FindPosByUid(saved, slots[static_cast<std::size_t>(s)]);
        const bool      empty    = slots[static_cast<std::size_t>(s)].empty();
        const bool      dangling = !empty && !p;   // uid from another product

        // An unresolved uid must never be clickable: it names a position this
        // product does not contain, and the pad has no idea where that is.
        ImGui::BeginDisabled(!ready || !p);
        if (empty)         ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
        else if (dangling) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.30f, 0.20f, 1.0f));
        const char* label = p ? p->name : (dangling ? "?" : "--");
        const bool  hit   = ImGui::Button(label, ImVec2(bw, bh));
        if (empty || dangling) ImGui::PopStyleColor();
        ImGui::EndDisabled();

        if (hit && p) {
            // StartWatchedMove is the shared UI move: it resolves the uid, drives
            // every axis together, and reports the travel in the status banner
            // until the axes arrive — on whatever page the operator walks to.
            if (StartWatchedMove(devName, p->uid, p->name))
                FD_INFO("saved", "pad %d: %s -> '%s'", s + 1, devName.c_str(), p->name);
            else
                FD_WARN("saved", "pad %d: %s -> '%s' refused", s + 1, devName.c_str(), p->name);
        }

        // AllowWhenDisabled on both: the tooltip and the assign menu are wanted
        // exactly when the button is grey (unassigned, or jog disarmed).
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            if (p) {
                std::string tip = std::string(p->name) + "\n";
                for (const auto& av : p->vals) {
                    char b[64];
                    std::snprintf(b, sizeof(b), "%s %9.4f\n", av.first.c_str(), av.second);
                    tip += b;
                }
                tip += "right-click to reassign";
                ImGui::SetTooltip("%s", tip.c_str());
            } else if (dangling) {
                ImGui::SetTooltip("assigned position is not in the loaded product\n"
                                  "(uid %s)\nright-click to reassign",
                                  slots[static_cast<std::size_t>(s)].c_str());
            } else {
                ImGui::SetTooltip("empty slot - right-click to assign a position");
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) ImGui::OpenPopup("padassign");
        }

        if (ImGui::BeginPopup("padassign")) {
            ImGui::TextDisabled("Assign to slot %d  (%s)", s + 1, devName.c_str());
            ImGui::Separator();
            // Hidden positions are withheld, the rule the recipe picker and the
            // registration picker both follow: the `algn_...` system poses are
            // noise to somebody laying out a pad.
            bool any = false;
            if (saved) {
                // PushID by row, not by name: two positions may share a label,
                // and then one Selectable would eat the other's click.
                for (std::size_t r = 0; r < saved->size(); ++r) {
                    const SavedPos& sp = (*saved)[r];
                    if (sp.hidden) continue;
                    any = true;
                    ImGui::PushID(static_cast<int>(r));
                    if (ImGui::Selectable(sp.name, sp.uid == slots[static_cast<std::size_t>(s)])) {
                        slots[static_cast<std::size_t>(s)] = sp.uid;
                        SaveUiSettings();
                    }
                    ImGui::PopID();
                }
            }
            if (!any) ImGui::TextDisabled("no taught positions for '%s'", devName.c_str());
            ImGui::Separator();
            if (ImGui::Selectable("Clear slot")) {
                slots[static_cast<std::size_t>(s)].clear();
                SaveUiSettings();
            }
            ImGui::EndPopup();
        }
        ImGui::PopID();
    }

    // Why a filled button is unavailable, rather than a grid of dead rectangles.
    if (!jogArmed_)                      ImGui::TextDisabled("arm jog first");
    else if (st != ConnState::Connected) ImGui::TextDisabled("device not connected");
    else if (busy)                       ImGui::TextDisabled("a run is active");
}

// The nine slots of one device, created blank on first ask. posPad_ is UI state
// and not the document, so growing it from a Draw is not the mutation the
// project rule forbids. The resize also normalises a settings file that was
// hand-edited to the wrong length.
std::vector<std::string>& App::PosPadSlots(const std::string& device)
{
    std::vector<std::string>& v = posPad_[device];
    if (v.size() != static_cast<std::size_t>(kPosPadSlots))
        v.resize(static_cast<std::size_t>(kPosPadSlots));
    return v;
}

// ---------------------------------------------------------------------------
// IO tab: a growable list of output rows. Each row is one button — press it
// and the pin toggles, or, with a dwell typed beside it, goes ON for that many
// ms and OFF again — over one output the operator keeps reaching for (a vacuum,
// a light, a clamp). The IO page shows EVERY pin of ONE device; this shows the
// handful that matter, from any device, in the window already under the hand.
//
// A row names its pin by device NAME + pin NAME (or index when unnamed), never
// by fleet position, and a row the fleet cannot resolve greys out rather than
// vanishing: the config changed, the operator did not unassign it. Rows are
// added with the button at the bottom and assigned/removed through a
// right-click POPUP — never a modal, for the reason the Pos tab gives.
//
// Writes gate on ioArmed_, the IO page's own "Enable" — one kill switch for
// IO outputs, whichever surface throws them. They do NOT gate on
// orch_.Busy(): the IO page's toggles never did, and switching a light on
// while a recipe waits for a prompt is exactly what this tab is for. The dwell
// never blocks: Workflow::PulseIoOutput sets ON and its Update() tick issues
// the OFF, so the UI thread waits on nothing. A click during the dwell turns
// the pin OFF now (and cancels the timed OFF) — the way out of a pulse that
// was too long.
void App::DrawJogIoTab()
{
    // --- Enable, the IO page's own switch ------------------------------------
    ImGui::TextUnformatted("Outputs");
    ImGui::SameLine();
    ImGui::Checkbox("Enable##qio", &ioArmed_);
    ImGui::SameLine();
    if (ioArmed_) ImGui::TextColored(StateColor(ConnState::Connecting), "LIVE");
    else          ImGui::TextDisabled("disabled");
    ImGui::Separator();

    // Resolve a row against the live fleet. -1 device or pin = unresolved.
    struct Target {
        int        dev = -1;
        int        pin = -1;
        IIoDevice* d   = nullptr;
    };
    const auto resolve = [&](const QuickIoRow& r) {
        Target t;
        if (r.device.empty()) return t;
        for (std::size_t i = 0; i < hardware_.IoCount(); ++i) {
            if (hardware_.IoName(i) != r.device) continue;
            t.d = hardware_.IoDevice(i);
            if (!t.d) break;
            t.dev = static_cast<int>(i);
            int pin = t.d->OutputPinByName(r.pin);
            if (pin < 0 && !r.pin.empty()) {
                // Unnamed pins are stored by index; refuse anything that is
                // not purely digits so "3x" cannot read as pin 3.
                bool digits = true;
                for (const char c : r.pin) digits = digits && (c >= '0' && c <= '9');
                if (digits) pin = std::atoi(r.pin.c_str());
            }
            if (pin >= 0 && pin < t.d->OutputCount()) t.pin = pin;
            break;
        }
        return t;
    };

    const float msW   = ImGui::CalcTextSize("00000").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    const float unitW = ImGui::CalcTextSize("ms").x;
    const float gap   = ImGui::GetStyle().ItemSpacing.x;
    const float bh    = ImGui::GetTextLineHeight() * 1.8f;

    int removeRow = -1;   // deferred: erasing inside the loop shifts the ids
    for (int row = 0; row < static_cast<int>(quickIo_.size()); ++row) {
        QuickIoRow& r = quickIo_[static_cast<std::size_t>(row)];
        ImGui::PushID(row);

        const Target   t        = resolve(r);
        const bool     assigned = !r.device.empty();
        const bool     dangling = assigned && t.pin < 0;
        const ConnState st      = t.dev >= 0 ? hardware_.IoState(static_cast<std::size_t>(t.dev))
                                             : ConnState::Disconnected;
        const bool     conn     = t.pin >= 0 && st == ConnState::Connected;
        const IIoDevice::IoStatus s = t.d ? t.d->Status() : IIoDevice::IoStatus{};
        const bool     on       = conn && s.valid && ((s.outputs >> t.pin) & 1ull) != 0;
        const int      left     = conn ? workflow_.PendingPulseMs(static_cast<std::size_t>(t.dev), t.pin)
                                       : -1;
        const bool     canWrite = ioArmed_ && conn;

        // Label: the pin's name (or "dev:#n"), then its state. "###" keeps the
        // button's ID fixed while the countdown rewrites the text every frame,
        // or the press would release the moment the label changed under it.
        char label[96];
        if (!assigned) {
            std::snprintf(label, sizeof(label), "--###btn");
        } else if (dangling) {
            std::snprintf(label, sizeof(label), "?###btn");
        } else {
            const std::string name = t.d->OutputPinName(t.pin);
            char pinTxt[48];
            if (name.empty()) std::snprintf(pinTxt, sizeof(pinTxt), "%s:#%d", r.device.c_str(), t.pin);
            else              std::snprintf(pinTxt, sizeof(pinTxt), "%s", name.c_str());
            if (left >= 0)    std::snprintf(label, sizeof(label), "%s  %d ms###btn", pinTxt, left);
            else if (on)      std::snprintf(label, sizeof(label), "%s  ON###btn", pinTxt);
            else              std::snprintf(label, sizeof(label), "%s  off###btn", pinTxt);
        }

        // Colour is the state, so the row reads at a glance: green ON, amber
        // while a pulse is timing out, frame-grey for an empty row, rust for
        // one the fleet cannot resolve.
        int pushed = 0;
        if (!assigned) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
            pushed = 1;
        } else if (dangling) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.30f, 0.20f, 1.0f));
            pushed = 1;
        } else if (left >= 0) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.70f, 0.50f, 0.12f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.60f, 0.18f, 1.0f));
            pushed = 2;
        } else if (on) {
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.55f, 0.30f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.65f, 0.36f, 1.0f));
            pushed = 2;
        }
        ImGui::BeginDisabled(!canWrite);
        const bool hit = ImGui::Button(label, ImVec2(ImGui::GetContentRegionAvail().x - msW - unitW - gap * 2.0f, bh));
        ImGui::EndDisabled();
        if (pushed) ImGui::PopStyleColor(pushed);

        if (hit && canWrite) {
            const std::size_t di = static_cast<std::size_t>(t.dev);
            if (left >= 0)           workflow_.SetIoOutput(di, t.pin, false);          // cut the pulse short
            else if (r.dwellMs > 0)  workflow_.PulseIoOutput(di, t.pin, r.dwellMs);    // ON, then OFF after dwell
            else                     workflow_.SetIoOutput(di, t.pin, !on);            // plain toggle
        }

        // AllowWhenDisabled on both: the tooltip and the assign menu are wanted
        // exactly when the button is grey (unassigned, disabled, disconnected).
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            if (!assigned) {
                ImGui::SetTooltip("empty row - right-click to assign an output");
            } else if (dangling) {
                ImGui::SetTooltip("'%s' / '%s' is not in the IO fleet\nright-click to reassign",
                                  r.device.c_str(), r.pin.c_str());
            } else {
                std::string tip = r.device + "  output " + std::to_string(t.pin) + "\n";
                tip += r.dwellMs > 0 ? "press: ON, then OFF after " + std::to_string(r.dwellMs) + " ms"
                                     : "press: toggle";
                if (!ioArmed_)                       tip += "\noutputs disabled (Enable above)";
                else if (st != ConnState::Connected) tip += "\ndevice not connected";
                else if (left >= 0)                  tip += "\npress to switch OFF now";
                tip += "\nright-click to reassign";
                ImGui::SetTooltip("%s", tip.c_str());
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) ImGui::OpenPopup("qioassign");
        }

        // Dwell. 0 = toggle. Saved when the field is left, not per keystroke.
        ImGui::SameLine();
        ImGui::SetNextItemWidth(msW);
        int ms = r.dwellMs;
        if (ImGui::InputInt("##ms", &ms, 0, 0)) r.dwellMs = ms < 0 ? 0 : ms;
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveUiSettings();
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("dwell in ms: 0 = toggle,\n> 0 = ON for this long, then OFF");
        ImGui::SameLine();
        ImGui::TextDisabled("ms");

        if (ImGui::BeginPopup("qioassign")) {
            ImGui::TextDisabled("Assign row %d", row + 1);
            ImGui::Separator();
            bool any = false;
            for (std::size_t i = 0; i < hardware_.IoCount(); ++i) {
                IIoDevice* d = hardware_.IoDevice(i);
                if (!d) continue;
                any = true;
                const std::string& devName = hardware_.IoName(i);
                if (ImGui::BeginMenu(devName.c_str())) {
                    for (int pin = 0; pin < d->OutputCount(); ++pin) {
                        const std::string name = d->OutputPinName(pin);
                        char item[80];
                        if (name.empty()) std::snprintf(item, sizeof(item), "#%d", pin);
                        else              std::snprintf(item, sizeof(item), "#%d  %s", pin, name.c_str());
                        ImGui::PushID(pin);
                        const bool cur = t.dev == static_cast<int>(i) && t.pin == pin;
                        if (ImGui::Selectable(item, cur)) {
                            r.device = devName;
                            r.pin    = name.empty() ? std::to_string(pin) : name;
                            SaveUiSettings();
                        }
                        ImGui::PopID();
                    }
                    ImGui::EndMenu();
                }
            }
            if (!any) ImGui::TextDisabled("no IO devices configured");
            ImGui::Separator();
            if (ImGui::Selectable("Remove row")) removeRow = row;
            ImGui::EndPopup();
        }
        ImGui::PopID();
    }
    if (removeRow >= 0) {
        quickIo_.erase(quickIo_.begin() + removeRow);
        SaveUiSettings();
    }

    if (quickIo_.empty()) ImGui::TextDisabled("no rows - add one below");
    ImGui::Spacing();
    if (ImGui::Button("+ Add row", ImVec2(-FLT_MIN, 0.0f))) {
        quickIo_.emplace_back();
        SaveUiSettings();
    }
    if (!ioArmed_) ImGui::TextDisabled("outputs disabled");
}

// Build the one-step recipe and hand it to the engine — the same "Run Step"
// mechanism the Recipe Editor uses, so validation (device known, camera
// Connected, params in range) is the engine's, not a second copy here.
void App::StartQuickAutoFocus()
{
    afErr_.clear();
    if (afDirIndex_ < 0 || afDirIndex_ >= kAfDirectionCount) afDirIndex_ = 0;

    RecipeStep step;
    step.type     = "auto_focus";
    step.nickname = std::string("Autofocus ") + kAfDirections[afDirIndex_].label;
    step.params   = {
        {"device", kQuickAutoFocusDevice},
        {"axis", "Z"},
        {"direction", kAfDirections[afDirIndex_].value},
        {"z_range_mm", static_cast<double>(afRangeMM_)},
        {"z_step_mm", static_cast<double>(afStepMM_)},
        {"roi_px", focusRoiPx_},
        {"trace", true},                  // one CSV line per sample into the log
    };

    Recipe one;
    one.uid  = "quick-autofocus";
    one.name = kQuickAutoFocusName;
    one.steps.push_back(std::move(step));

    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &afErr_))
        FD_WARN("vision", "autofocus not started: %s", afErr_.c_str());
}

// ---- Tool -> cam / Cam -> tool ----------------------------------------------
//
// Which tool: the selected jog device when it is registered itself, else - with
// the camera's device selected - one of the tools registered against it. Both
// buttons run the registration commands on the engine worker (an approach can
// take seconds; the UI thread never waits), so their refusals are the ones a
// recipe step gets, word for word.
namespace {
constexpr const char* kQuickRegMoveName = "Quick registered move";
}

void App::StartQuickRegMove(bool toolToCam, const std::string& tool)
{
    regMoveErr_.clear();
    RecipeStep step;
    if (toolToCam) {
        step.type     = "move_tool_to_camera";
        step.nickname = "Tool '" + tool + "' to camera";
        step.params   = {
            {"device", tool},
            {"z_offset_mm", jogRegZOff_},
            {"allow_unverified", jogRegUnver_},
        };
    } else {
        step.type     = "move_camera_to_tool";
        step.nickname = "Camera to tool '" + tool + "'";
        step.params   = {{"device", tool}};   // XY only: the command's default
    }

    Recipe one;
    one.uid  = "quick-regmove";
    one.name = kQuickRegMoveName;
    one.steps.push_back(std::move(step));

    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &regMoveErr_))
        FD_WARN("devreg", "%s not started: %s",
                toolToCam ? "tool to camera" : "camera to tool", regMoveErr_.c_str());
}

void App::DrawJogRegRow(const std::string& selected)
{
    // Candidates: the selected device if it is a registered tool; otherwise the
    // tools registered AGAINST it (the camera's device is selected).
    std::vector<std::string> tools;
    const DeviceReg selReg = workflow_.DeviceRegFor(selected);
    if (selReg.valid) {
        tools.push_back(selected);
    } else {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string& nm = hardware_.MotionName(i);
            if (nm == selected) continue;
            const DeviceReg r = workflow_.DeviceRegFor(nm);
            if (r.valid && r.refDevice == selected) tools.push_back(nm);
        }
    }
    if (tools.empty()) return;   // nothing registered here: no row at all

    ImGui::Separator();
    ImGui::TextUnformatted("Registered tool");
    std::string tool = tools.front();
    if (tools.size() > 1) {
        if (std::find(tools.begin(), tools.end(), jogRegTool_) == tools.end())
            jogRegTool_ = tools.front();
        tool = jogRegTool_;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::BeginCombo("##regtool", tool.c_str())) {
            for (const std::string& t : tools)
                if (ImGui::Selectable(t.c_str(), t == tool)) jogRegTool_ = t;
            ImGui::EndCombo();
        }
    } else {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", tool.c_str());
    }
    const DeviceReg reg = workflow_.DeviceRegFor(tool);
    if (!reg.verified)
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "unverified registration");

    const bool busy  = orch_.Busy();
    const bool ready = jogArmed_ && !busy;
    const float half = ImGui::GetContentRegionAvail().x * 0.5f
                     - ImGui::GetStyle().ItemSpacing.x * 0.5f;
    ImGui::BeginDisabled(!ready);
    if (ImGui::Button("Tool -> cam", ImVec2(half, 0.0f))) StartQuickRegMove(true, tool);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Move '%s' so its tip goes to the point the camera\n"
                          "('%s') is looking at, Z offset %+.3f mm from the\n"
                          "focused plane (negative = stand off, on this machine).\n"
                          "An APPROACH through the registration - move_tool_to_camera.",
                          tool.c_str(), reg.refDevice.c_str(), jogRegZOff_);
    ImGui::SameLine();
    ImGui::BeginDisabled(!ready);
    if (ImGui::Button("Cam -> tool", ImVec2(-FLT_MIN, 0.0f))) StartQuickRegMove(false, tool);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Move the camera ('%s') over '%s's predicted tip,\n"
                          "X and Y only. The tool does not move.\n"
                          "move_camera_to_tool.",
                          reg.refDevice.c_str(), tool.c_str());

    ImGui::SetNextItemWidth(80.0f);
    ImGui::InputDouble("Z off##reg", &jogRegZOff_, 0.0, 0.0, "%.3f");
    jogRegZOff_ = std::clamp(jogRegZOff_, -100.0, 100.0);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Tool -> cam only: offset from the camera's focused Z.\n"
                          "NEGATIVE stands the tool off the surface on this machine;\n"
                          "0 sends the tip ONTO the focused surface - a touch.");
    ImGui::SameLine();
    ImGui::Checkbox("unverified##reg", &jogRegUnver_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Tool -> cam: allow a registration whose check has not been run.");

    if (!jogArmed_)  ImGui::TextDisabled("arm jog first");
    else if (busy)   ImGui::TextDisabled("another run is active");

    // Progress / result, only for OUR run - the engine is shared.
    const RecipeEngine::Status rs = orch_.recipe.GetStatus();
    if (rs.recipeName == kQuickRegMoveName) {
        if (rs.state == RecipeEngine::State::Running) {
            ImGui::TextUnformatted(rs.currentStep.c_str());
        } else if (!rs.error.empty() && (rs.state == RecipeEngine::State::Failed ||
                                         rs.state == RecipeEngine::State::Stopped)) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", rs.error.c_str());
            ImGui::PopTextWrapPos();
        } else if (rs.state == RecipeEngine::State::Completed) {
            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.45f, 1.0f), "arrived");
        }
    }
    if (!regMoveErr_.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", regMoveErr_.c_str());
        ImGui::PopTextWrapPos();
    }
}

// Jog controls (its own window). Buttons + W/D·A/S·R/F keys route through the
// workflow, which maps global direction -> local axis via the device transform.
// Everything but Stop is gated by the Arm toggle.
void App::DrawJogPanel()
{
    const size_t n = hardware_.MotionCount();
    if (n == 0) { ImGui::TextDisabled("no motion devices"); return; }
    if (jogDevice_ < 0 || static_cast<size_t>(jogDevice_) >= n) jogDevice_ = 0;

    // --- Active device list (left) + live axis readout of the selection (right) ---
    // The row is split in half: which device the keys drive, and where that
    // device currently is. Both boxes are one height, sized for the widest axis
    // count in the fleet (a hexapod's six) so switching device does not make the
    // panel below jump.
    size_t maxAxes = 3;
    for (size_t i = 0; i < n; ++i)
        if (IMotionController* d = hardware_.MotionDevice(i))
            maxAxes = (std::max)(maxAxes, d->Axes().size());
    const float rowH  = ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(maxAxes)
                        + ImGui::GetStyle().FramePadding.y * 2.0f;
    const float halfW = ImGui::GetContentRegionAvail().x * 0.5f - ImGui::GetStyle().ItemSpacing.x * 0.5f;

    ImGui::TextUnformatted("Device");
    ImGui::BeginChild("jogdevs", ImVec2(halfW, rowH), ImGuiChildFlags_Borders);
    // The list runs a size up from the rest of the window: which device the
    // pad drives is the one fact to check before pressing anything on it. No
    // bold for the selection — the atlas carries one typeface and no bold cut,
    // and an overdrawn copy read as doubled text. The highlight is enough.
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.25f);
    for (size_t i = 0; i < n; ++i) {
        IMotionController* d = hardware_.MotionDevice(i);
        if (!d) continue;
        const ConnState st = hardware_.MotionState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        if (ImGui::Selectable(hardware_.MotionName(i).c_str(), jogDevice_ == static_cast<int>(i)))
            jogDevice_ = static_cast<int>(i);
        ImGui::PopID();
    }
    ImGui::PopFont();
    ImGui::EndChild();

    IMotionController* dev = hardware_.MotionDevice(static_cast<size_t>(jogDevice_));

    // Live X/Y/Z/U/V/W of the selected device, straight off the poll cache — the
    // same numbers the Motion page shows, in the panel that is moving them.
    ImGui::SameLine();
    ImGui::BeginChild("jogpos", ImVec2(0.0f, rowH), ImGuiChildFlags_Borders);
    if (!dev) {
        ImGui::TextDisabled("no device");
    } else {
        const std::vector<std::string>      axNames = dev->Axes();
        const std::map<std::string, double> live    = dev->Positions();
        for (const std::string& ax : axNames) {
            const auto it = live.find(ax);
            if (it != live.end()) ImGui::Text("%s %9.4f", ax.c_str(), it->second);
            else                  ImGui::TextDisabled("%s      --", ax.c_str());
        }
    }
    ImGui::EndChild();

    if (!dev) { ImGui::TextDisabled("select a device"); return; }
    const std::vector<std::string> axes = dev->Axes();
    const size_t                   di   = static_cast<size_t>(jogDevice_);
    dev->RequestFastPoll();   // keep positions fresh for accurate relative jogs

    // --- Arm toggle + STOP (always active; the software big-red-button) ---
    ImGui::Checkbox("Arm", &jogArmed_);
    ImGui::SameLine();
    if (jogArmed_) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "ARMED");
    else           ImGui::TextDisabled("disarmed");
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("STOP", ImVec2(-FLT_MIN, 30.0f))) orch_.StopAll();
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stops ALL motion devices and aborts any running\n"
                          "recipe / script / program / path move.\n"
                          "(Laser, PSU and IO outputs are not touched.)");
    ImGui::Separator();

    // --- Velocity (left) + the current step size, big (right) ---
    // The step is what every key press moves by, so it gets a readout an
    // operator can check from a step back at the bench, beside the velocity
    // block it shares a row with. The left child has no padding, so the combo
    // lays out exactly as it does on the Motion page.
    {
        const float rowW  = ImGui::GetContentRegionAvail().x;
        const float bigW  = std::floor(rowW * 0.38f);
        const float leftW = rowW - bigW - ImGui::GetStyle().ItemSpacing.x;
        const float boxH  = ImGui::GetTextLineHeightWithSpacing()
                          + ImGui::GetFrameHeightWithSpacing()
                          + ImGui::GetFrameHeight();
        ImGui::BeginChild("jogvel", ImVec2(leftW, boxH), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        DrawVelocityCombo(di);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.30f, 0.76f, 0.32f, 1.0f));
        ImGui::BeginChild("jogstepbig", ImVec2(bigW, boxH), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        DrawBigLabel(kSteps[jogStepIndex_].label, ImVec2(bigW, boxH), ImVec4(1.0f, 0.92f, 0.12f, 1.0f));
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }

    // --- Step size button array ---
    ImGui::TextUnformatted("Step size");
    ImGui::SameLine();
    ImGui::TextDisabled("Q / E");
    const float bw    = 54.0f;
    const float availw = ImGui::GetContentRegionAvail().x;
    int perRow = static_cast<int>(availw / (bw + 6.0f));
    if (perRow < 1) perRow = 1;
    for (int i = 0; i < kStepCount; ++i) {
        if (i % perRow != 0) ImGui::SameLine();
        const bool sel = (jogStepIndex_ == i);
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        if (ImGui::Button(kSteps[i].label, ImVec2(bw, 0.0f))) jogStepIndex_ = i;
        if (sel) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // --- Translational cluster (global X/Y/Z), routed through the workflow ---
    // Left/Right = X-/X+ (A/D), In/Out = Y-/Y+ (W/S), Up/Down = Z+/Z- (R/F). The
    // device transform (in Workflow) maps global direction -> local axis + sign.
    const double step = kSteps[jogStepIndex_].mm;
    const ImVec2 jb(58.0f, 0.0f);   // auto height -> fits the two-line "In\nW" labels
    ImGui::BeginDisabled(!jogArmed_);
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(jb.x + 8.0f, 0.0f)); ImGui::SameLine();
    if (ImGui::Button("In\nW", jb))    workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);
    if (ImGui::Button("Left\nA", jb))  workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step); ImGui::SameLine();
    if (ImGui::Button("Out\nS", jb))   workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step); ImGui::SameLine();
    if (ImGui::Button("Right\nD", jb)) workflow_.JogGlobal(di, 1.0f,  0.0f, 0.0f, step);
    if (ImGui::Button("Up\nR", jb))    workflow_.JogGlobal(di, 0.0f,  0.0f, 1.0f, step); ImGui::SameLine();
    if (ImGui::Button("Down\nF", jb))  workflow_.JogGlobal(di, 0.0f,  0.0f, -1.0f, step);
    ImGui::EndGroup();
    ImGui::EndDisabled();

    // The device those keys drive, big, in the space beside the cluster: the
    // list above scrolls out of view on a short window, and pressing W on the
    // wrong hexapod is the mistake this readout exists to prevent. It sits
    // outside the arm gate so it stays legible while disarmed.
    {
        const float nameH = ImGui::GetItemRectSize().y;
        ImGui::SameLine();
        const float nameW = ImGui::GetContentRegionAvail().x;
        if (nameW > 40.0f) {
            ImGui::BeginChild("jogdevname", ImVec2(nameW, nameH), ImGuiChildFlags_Borders,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            const ImVec2 pad = ImGui::GetStyle().WindowPadding;
            // Small row above the name: hold + Upd + the taught position the
            // device is on. The status of the last Upd takes a line under the
            // name while there is one; the big label gets what is left.
            DrawJogPositionRow(*dev, axes);   // may stage jogUpd_; the modal is below
            const float lineH  = ImGui::GetTextLineHeightWithSpacing();
            const float usedH  = ImGui::GetCursorPosY() - pad.y;
            // The status is a brief notice, not a readout: it shows for a few
            // seconds after Update and then gives the line back to the name.
            // The log keeps the permanent record.
            const double left   = jogPosStatusUntil_ - ImGui::GetTime();
            const bool  showSt  = !jogPosStatus_.empty() && left > 0.0;
            const float statH   = showSt ? lineH : 0.0f;
            const float bigH    = nameH - pad.y * 2.0f - usedH - statH;
            if (bigH > ImGui::GetTextLineHeight())
                DrawBigLabel(dev->Name().c_str(), ImVec2(nameW - pad.x * 2.0f, bigH),
                             jogArmed_ ? ImVec4(1.0f, 0.75f, 0.20f, 1.0f) : ImGui::GetStyleColorVec4(ImGuiCol_Text));
            if (showSt) {
                // Fades over its last second so it leaves instead of blinking out.
                const float a = static_cast<float>(left < 1.0 ? left : 1.0);
                ImGui::TextColored(ImVec4(0.47f, 0.92f, 0.59f, a), "%s", jogPosStatus_.c_str());
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", jogPosStatusFull_.c_str());
            }
            ImGui::EndChild();
        } else {
            ImGui::NewLine();
        }
    }
    // Raised here, not inside the name box: a popup is addressed through the
    // ID stack it was opened on, and the box's child is skipped on a narrow
    // window. Its own function so the row above stays about the row.
    DrawJogUpdatePosModal();
    ImGui::BeginDisabled(!jogArmed_);

    // --- Rotational cluster (GLOBAL U/V/W = theta x/y/z), only if the device
    // has rotational axes. Routed through JogGlobalRot so the transform's
    // rotation block maps global -> signed local axis (test it live from the
    // Transforms editor). jogRotAxis_ selects the GLOBAL rotation axis.
    const int rotCount = static_cast<int>(axes.size()) - 3;
    if (rotCount > 0) {
        if (jogRotAxis_ < 0 || jogRotAxis_ >= 3) jogRotAxis_ = 0;
        static const char* kRotNames[3] = {"U \xce\xb8x", "V \xce\xb8y", "W \xce\xb8z"};

        // Mapped-target readout: dominant entry of the rotation block's column,
        // from the same working copy the Transforms editor hot-applies.
        std::string mapped = "unmapped";
        {
            AxisTransform t;   // identity if none configured
            const auto it = transformCfg_.find(dev->Name());
            if (it != transformCfg_.end()) t = it->second;
            int ax = 0;
            for (int i = 1; i < 3; ++i)
                if (std::fabs(t.r[i][jogRotAxis_]) > std::fabs(t.r[ax][jogRotAxis_])) ax = i;
            if (std::fabs(t.r[ax][jogRotAxis_]) > 1e-9 && 3 + ax < static_cast<int>(axes.size()))
                mapped = axes[3 + ax] + (t.r[ax][jogRotAxis_] >= 0.0 ? "+" : "-");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Rotational");
        ImGui::SameLine();
        ImGui::TextDisabled("-> local %s", mapped.c_str());
        ImGui::BeginGroup();
        for (int i = 0; i < 3; ++i) {
            const bool sel = (jogRotAxis_ == i);
            if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.55f, 0.10f, 1.0f));
            if (ImGui::Button(kRotNames[i], ImVec2(jb.x, 0.0f))) jogRotAxis_ = i;
            if (sel) ImGui::PopStyleColor();
        }
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        const float gu = (jogRotAxis_ == 0) ? 1.0f : 0.0f;
        const float gv = (jogRotAxis_ == 1) ? 1.0f : 0.0f;
        const float gw = (jogRotAxis_ == 2) ? 1.0f : 0.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("+##rot", ImVec2(jb.x, 0.0f))) workflow_.JogGlobalRot(di, gu, gv, gw, +step);
        ImGui::PopStyleColor();
        char stepReadout[24];
        std::snprintf(stepReadout, sizeof(stepReadout), "%s##rotstep", kSteps[jogStepIndex_].label);
        ImGui::Button(stepReadout, ImVec2(jb.x, 0.0f));   // step readout (unique id)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("-##rot", ImVec2(jb.x, 0.0f))) workflow_.JogGlobalRot(di, gu, gv, gw, -step);
        ImGui::PopStyleColor();
        ImGui::EndGroup();
    }
    ImGui::EndDisabled();

    DrawJogRegRow(dev->Name());

    // --- Keyboard shortcuts (this window focused): one action per key press ---
    // Q/E pick the step size (down/up the array); A/D = Left/Right, W/S =
    // In/Out, R/F = Up/Down. Q/E sit OUTSIDE the arm gate on purpose: choosing
    // a distance moves nothing, and reaching for the size before arming is the
    // normal order. Everything that actually moves stays armed-only.
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        if (ImGui::IsKeyPressed(ImGuiKey_Q, false) && jogStepIndex_ > 0)              --jogStepIndex_;
        if (ImGui::IsKeyPressed(ImGuiKey_E, false) && jogStepIndex_ < kStepCount - 1) ++jogStepIndex_;

        if (jogArmed_) {
            if (ImGui::IsKeyPressed(ImGuiKey_A, false)) workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step);  // Left  X-
            if (ImGui::IsKeyPressed(ImGuiKey_D, false)) workflow_.JogGlobal(di,  1.0f, 0.0f, 0.0f, step);  // Right X+
            if (ImGui::IsKeyPressed(ImGuiKey_W, false)) workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);  // In    Y-
            if (ImGui::IsKeyPressed(ImGuiKey_S, false)) workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step);  // Out   Y+
            if (ImGui::IsKeyPressed(ImGuiKey_R, false)) workflow_.JogGlobal(di, 0.0f, 0.0f,  1.0f, step);  // Up    Z+
            if (ImGui::IsKeyPressed(ImGuiKey_F, false)) workflow_.JogGlobal(di, 0.0f, 0.0f, -1.0f, step);  // Down  Z-
        }
    }
}

// Where the selected device is, by taught position (VISIBLE positions only:
// the algn_... system names mean nothing to an operator jogging by hand, so
// here a hidden neighbour is skipped and the answer may read "+0.3 mm" from
// the nearest visible one), and the hold that turns that name into a re-teach.
//
// The hold exists because Upd on the Motion page needs the operator to know
// WHICH row they are at before they move: once the stage has left the taught
// point the name is gone from every readout. Taking the hold while still ON
// the position pins the record; the jog happens; Upd writes the live pose into
// it and reports from/to/delta so a slip of the wrong step size shows up as a
// number. Same write as the Motion page's Upd, same gate (that page's), same
// save.
void App::DrawJogPositionRow(IMotionController& dev, const std::vector<std::string>& axes)
{
    const std::string devName = dev.Name();
    if (!jogHoldUid_.empty() && jogHoldDev_ != devName) {
        jogHoldUid_.clear();   // the hold named a record on the OTHER device
        jogPosStatus_.clear();
    }

    const std::map<std::string, double> live = dev.Positions();
    SavedPosMap& all = session_.Positions();
    const auto  it   = all.find(devName);
    double dist = 0.0;
    const SavedPos* nearest = (it != all.end())
                               ? NearestSavedPos(it->second, live, &dist, /*includeHidden=*/false)
                               : nullptr;
    const bool at = nearest && dist <= kSavedPosAtTolMM;

    SavedPos* held = nullptr;
    if (!jogHoldUid_.empty() && it != all.end()) {
        for (SavedPos& p : it->second) if (p.uid == jogHoldUid_) { held = &p; break; }
        if (!held) {   // product switched under the hold: nothing to write into
            jogHoldUid_.clear();
            jogPosStatus_      = "held position is gone";
            jogPosStatusFull_  = "The held position is no longer in the loaded product.";
            jogPosStatusUntil_ = ImGui::GetTime() + kJogPosStatusSec;
        }
    }

    const bool canTeach = CanAccess(Page::Motion);   // the Motion page's Upd gate
    bool hold = held != nullptr;
    ImGui::BeginDisabled(!canTeach || (!hold && !at));
    if (ImGui::Checkbox("##poshold", &hold)) {
        if (hold && nearest) {
            jogHoldUid_ = nearest->uid;
            jogHoldDev_ = devName;
            jogPosStatus_.clear();
        } else {
            jogHoldUid_.clear();   // unchecked by hand: nothing written
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        if (!canTeach)   ImGui::SetTooltip("Re-teaching a position needs the Motion page's level.");
        else if (!hold && !at) ImGui::SetTooltip("Move onto a taught position first, then check\n"
                                                 "to hold its name while you jog away from it.");
        else ImGui::SetTooltip("Holding: jog to the new spot, then Upd writes it\n"
                               "into this position. Uncheck to drop the hold.");
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!held);
    if (ImGui::SmallButton("Upd") && held) {
        // Stage only — the modal shows this table and writes on confirm.
        jogUpd_      = JogPosUpdate{};
        jogUpd_.dev  = devName;
        jogUpd_.uid  = held->uid;
        jogUpd_.name = held->name;
        double sq = 0.0;
        for (const std::string& a : axes) {
            const auto li = live.find(a);
            if (li == live.end()) continue;
            const auto oi = held->vals.find(a);
            const double o = (oi != held->vals.end()) ? oi->second : li->second;
            const double d = li->second - o;
            // Distance over the linear axes only: a degree of U added to a
            // millimetre of X is not a length.
            if (a == "X" || a == "Y" || a == "Z" || a == "x" || a == "y" || a == "z") sq += d * d;
            jogUpd_.rows.push_back({a, o, li->second});
        }
        jogUpd_.deltaMM = std::sqrt(sq);
        jogOpenUpdPos_  = true;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Review and write the device's current position into the held position.");
    ImGui::SameLine();

    if (held) {
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "%s  (held)", held->name);
        if (nearest && ImGui::IsItemHovered())
            ImGui::SetTooltip("now: %s%s", nearest->name, at ? "" : " (away)");
    } else if (!nearest) {
        ImGui::TextDisabled("(no taught position)");
    } else if (at) {
        ImGui::TextColored(ImVec4(0.47f, 0.92f, 0.59f, 1.0f), "%s", nearest->name);
    } else {
        ImGui::TextColored(ImVec4(0.88f, 0.78f, 0.51f, 1.0f), "%s  +%.3f mm", nearest->name, dist);
    }
}

// The confirmation for the Jog box's Upd: one row per axis, before / after /
// delta, and the XYZ distance. Writes the STAGED values, then saves, reports
// and drops the hold. Cancel leaves the hold in place — the operator may just
// want one more nudge.
void App::DrawJogUpdatePosModal()
{
    if (jogOpenUpdPos_) { ImGui::OpenPopup("Update Position"); jogOpenUpdPos_ = false; }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("Update Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    const JogPosUpdate& u = jogUpd_;
    ImGui::Text("Overwrite position '%s' on %s with the current position?", u.name.c_str(), u.dev.c_str());
    ImGui::Spacing();
    if (ImGui::BeginTable("updrows", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                            ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Axis");
        ImGui::TableSetupColumn("Before");
        ImGui::TableSetupColumn("After");
        ImGui::TableSetupColumn("Delta");
        ImGui::TableHeadersRow();
        for (const JogPosUpdate::Row& r : u.rows) {
            const double d = r.after - r.before;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(r.axis.c_str());
            ImGui::TableSetColumnIndex(1); ImGui::Text("%10.4f", r.before);
            ImGui::TableSetColumnIndex(2); ImGui::Text("%10.4f", r.after);
            ImGui::TableSetColumnIndex(3);
            // An unchanged axis stays quiet; a changed one is coloured so the
            // eye lands on what the jog actually moved.
            if (std::fabs(d) < 5e-5) ImGui::TextDisabled("%+10.4f", d);
            else ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.30f, 1.0f), "%+10.4f", d);
        }
        ImGui::EndTable();
    }
    ImGui::Text("XYZ distance: %.4f mm", u.deltaMM);
    if (u.deltaMM < 5e-5)
        ImGui::TextDisabled("(no change on X/Y/Z)");
    ImGui::Spacing();

    if (ImGui::Button("Update", ImVec2(110.0f, 0.0f))) {
        // The record is found again by uid: it may have gone (product switched
        // while the dialog was up), in which case nothing is written.
        SavedPos* rec = nullptr;
        const auto it = session_.Positions().find(u.dev);
        if (it != session_.Positions().end())
            for (SavedPos& p : it->second) if (p.uid == u.uid) { rec = &p; break; }
        if (rec) {
            std::string from, to;
            for (const JogPosUpdate::Row& r : u.rows) {
                rec->vals[r.axis] = r.after;
                char b[48];
                std::snprintf(b, sizeof(b), "%s%.4f", from.empty() ? "" : " ", r.before); from += b;
                std::snprintf(b, sizeof(b), "%s%.4f", to.empty()   ? "" : " ", r.after);  to += b;
            }
            char line[128], full[320];
            std::snprintf(line, sizeof(line), "'%s' updated, delta %.4f mm", rec->name, u.deltaMM);
            std::snprintf(full, sizeof(full), "position '%s' on %s updated\nfrom  %s\nto    %s\ndelta %.4f mm",
                          rec->name, u.dev.c_str(), from.c_str(), to.c_str(), u.deltaMM);
            jogPosStatus_      = line;
            jogPosStatusFull_  = full;
            jogPosStatusUntil_ = ImGui::GetTime() + kJogPosStatusSec;
            FD_INFO("saved", "jog: updated '%s' on %s from [%s] to [%s] (delta %.4f mm)",
                    rec->name, u.dev.c_str(), from.c_str(), to.c_str(), u.deltaMM);
            session_.SavePositions();
        } else {
            jogPosStatus_      = "position not updated: record is gone";
            jogPosStatusFull_  = "The held position is no longer in the loaded product; nothing was written.";
            jogPosStatusUntil_ = ImGui::GetTime() + kJogPosStatusSec;
            FD_WARN("saved", "jog: update of '%s' on %s skipped - record gone", u.name.c_str(), u.dev.c_str());
        }
        jogHoldUid_.clear();   // the hold is spent either way
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(110.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void App::DrawVelocityCombo(std::size_t di)
{
    IMotionController* dev = hardware_.MotionDevice(di);
    if (!dev) return;
    const bool    acs  = dev->Type() == "ACS";
    const double* vels = acs ? kVelACS : kVelPI;
    const int     velN = acs ? 5 : 4;
    const int     key  = static_cast<int>(di);
    int& idx = velIndex_[key];
    if (idx < 0 || idx >= velN) idx = 0;

    // Deliberately NO velocity push at connect (and none at disconnect/exit):
    // the device keeps whatever velocity it already has until the operator —
    // or a recipe/script velocity override — explicitly sets one. Until then
    // the combo shows "(device)" instead of pretending to know the value.
    ImGui::TextUnformatted("Velocity (mm/s)");
    ImGui::SetNextItemWidth(-FLT_MIN);
    char prev[24];
    if (velCustomSet_[key] > 0.0)
        std::snprintf(prev, sizeof(prev), "%g (custom)", velCustomSet_[key]);
    else if (!velPushed_[key])
        std::snprintf(prev, sizeof(prev), "(device)");
    else
        std::snprintf(prev, sizeof(prev), "%g", vels[idx]);
    if (ImGui::BeginCombo("##vel", prev)) {
        for (int i = 0; i < velN; ++i) {
            char b[16]; std::snprintf(b, sizeof(b), "%g", vels[i]);
            if (ImGui::Selectable(b, velCustomSet_[key] <= 0.0 && idx == i)) {
                idx = i;
                velCustomSet_[key] = 0.0;   // back to a preset
                workflow_.SetJogVelocity(di, vels[i]);
                velPushed_[key] = true;
            }
        }
        ImGui::EndCombo();
    }

    // Custom velocity not in the preset list. Clamped to the preset ceiling for
    // this controller type (PI 10, ACS 40 mm/s) — raise kVel* to allow more.
    const double velMax = vels[velN - 1];
    double& custom = velCustomEdit_[key];
    if (custom <= 0.0) custom = vels[idx];   // seed the box on first show
    ImGui::SetNextItemWidth(-64.0f);
    ImGui::InputDouble("##velcustom", &custom, 0.0, 0.0, "%.3f");
    ImGui::SameLine();
    if (ImGui::SmallButton("Apply")) {
        if (custom < 0.001)  custom = 0.001;
        if (custom > velMax) custom = velMax;
        velCustomSet_[key] = custom;
        workflow_.SetJogVelocity(di, custom);
        velPushed_[key] = true;
        FD_INFO("motion", "%s custom velocity %.3f mm/s", dev->Name().c_str(), custom);
    }
}

} // namespace firedog
