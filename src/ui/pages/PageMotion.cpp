// PageMotion.cpp : the Motion page — device list, tabbed detail (Positions and
// Scanning; the Mapped Path tab lives in PageMotionMap.cpp) and the live-axis
// info panel. Definitions of App members — see App.h.
//
// The Scanning tab is a test bench over hexapod_align.lua and owns NO scan
// logic: its form values are injected via LuaEngine::Env::initialCtx, exactly
// as a recipe would inject them with ctx_set steps. Open motion views call
// RequestFastPoll every frame — it is a demand signal, not a mode.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Paths.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

namespace firedog {

namespace {
// Updating a saved position from a pose that differs from the record by more
// than this on any axis asks twice. The same number is applied to every axis,
// rotary ones included: a 1° slip on U is as worth a second look as 1 mm on X.
constexpr double kPosUpdWarnMm = 1.0;

// Axis servo indicator: green when the controller reports the motor enabled.
const ImVec4 kServoOn (0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kServoOff(0.95f, 0.45f, 0.35f, 1.0f);
}

void App::DrawMotionListPanel()
{
    const size_t n = hardware_.MotionCount();

    // Keep the current selection if still valid; otherwise fall back to the
    // first device. The left panel draws before the middle/right, so this also
    // resolves the selection they read.
    if (selectedMotion_ < 0 || static_cast<size_t>(selectedMotion_) >= n ||
        !hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))) {
        selectedMotion_ = n > 0 ? 0 : -1;
    }

    ImGui::TextUnformatted("Motion devices");
    ImGui::Separator();

    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        IMotionController* m = hardware_.MotionDevice(i);
        if (!m) continue;
        any = true;
        const ConnState st = hardware_.MotionState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        const std::string& name = hardware_.MotionName(i);
        const ImVec2 sz = ImGui::CalcTextSize(name.c_str());
        if (ImGui::Selectable(name.c_str(), selectedMotion_ == static_cast<int>(i),
                              ImGuiSelectableFlags_None, ImVec2(sz.x, 0.0f))) {
            selectedMotion_ = static_cast<int>(i);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m->Type().c_str());
        ImGui::SameLine();
        ImGui::TextColored(StateColor(st), "%s", StateText(st));
        ImGui::PopID();
    }
    if (!any) ImGui::TextDisabled("no motion devices configured");

    // The live axis table and the velocity combo sit under the device list
    // rather than in a right panel of their own: they describe the device the
    // list has just selected, and the saved-position table in the middle is
    // what needs the width. The right panel is left empty for the Motion page.
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextUnformatted("Axis positions");
    ImGui::Separator();

    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;
    if (!m) { ImGui::TextDisabled("no selection"); return; }

    const std::vector<std::string>      axes = m->Axes();
    const std::map<std::string, double> pos  = m->Positions();
    // Servo state + Enable/Disable, for a controller that has the concept
    // (the ACS gantry). Toggling energises or drops a motor, so it sits behind
    // the jog arm like every other actuation, and is locked while anything runs.
    const bool servo = m->SupportsAxisEnable();
    const std::map<std::string, bool> enabled = servo ? m->AxisEnabled()
                                                      : std::map<std::string, bool>{};
    const bool connected =
        hardware_.MotionState(static_cast<size_t>(selectedMotion_)) == ConnState::Connected;
    const bool canToggle = servo && connected && jogArmed_ && !orch_.Busy();
    if (ImGui::BeginTable("motionpos", servo ? 4 : 2,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Axis");
        ImGui::TableSetupColumn("Position");
        if (servo) {
            ImGui::TableSetupColumn("Servo");
            ImGui::TableSetupColumn("##servobtn");
        }
        ImGui::TableHeadersRow();
        for (const std::string& ax : axes) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(ax.c_str());
            ImGui::TableSetColumnIndex(1);
            const auto it = pos.find(ax);
            if (it != pos.end()) ImGui::Text("%.4f", it->second);
            else                 ImGui::TextDisabled("--");
            if (!servo) continue;

            const auto en    = enabled.find(ax);
            const bool known = en != enabled.end();
            const bool on    = known && en->second;
            ImGui::TableSetColumnIndex(2);
            if (!known)  ImGui::TextDisabled("--");
            else if (on) ImGui::TextColored(kServoOn, "ON");
            else         ImGui::TextColored(kServoOff, "OFF");

            ImGui::TableSetColumnIndex(3);
            ImGui::PushID(ax.c_str());
            ImGui::BeginDisabled(!canToggle || !known);
            if (ImGui::SmallButton(on ? "Disable" : "Enable"))
                workflow_.SetAxisEnabled(static_cast<size_t>(selectedMotion_), ax, !on);
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (servo && connected) {
        ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms
        ImGui::SameLine();
        if (orch_.Busy())     ImGui::TextDisabled("servo locked while a run is active");
        else if (!jogArmed_)  ImGui::TextDisabled("arm to enable/disable an axis");
    }

    ImGui::Spacing();
    ImGui::Separator();
    DrawVelocityCombo(static_cast<size_t>(selectedMotion_));

    // Dynamic profile for THIS page's Move only (saved positions). Every other
    // move - jog, paths, recipes, the other pages - stays on the default
    // profile, and the driver puts it back before any of them runs.
    if (m->SupportsProfile()) {
        ImGui::Spacing();
        ImGui::Checkbox("Dynamic profile on Move", &motDynOn_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Move (saved positions, this page) sets each axis's\n"
                              "ACC = ratio x VEL, DEC = ACC, JERK = ratio x ACC first.\n"
                              "Every other move puts the default profile back.\n"
                              "Off at every start; recipes never use it (yet).");
        ImGui::BeginDisabled(!motDynOn_);
        ImGui::SetNextItemWidth(70.0f);
        if (ImGui::InputDouble("ACC / VEL", &motDyn_.accPerVel, 0.0, 0.0, "%g"))
            motDyn_.accPerVel = std::clamp(motDyn_.accPerVel, 0.1, 1000.0);
        ImGui::SetNextItemWidth(70.0f);
        if (ImGui::InputDouble("JERK / ACC", &motDyn_.jerkPerAcc, 0.0, 0.0, "%g"))
            motDyn_.jerkPerAcc = std::clamp(motDyn_.jerkPerAcc, 0.1, 1000.0);
        ImGui::EndDisabled();
        ImGui::TextDisabled("DEC = ACC; KDEC untouched");
        if (!motDynMsg_.empty()) ImGui::TextWrapped("%s", motDynMsg_.c_str());
    }
}

// A square box (250x250 when the panel allows) picturing the selected device,
// at the top of the Motion page's right panel, so the operator recognises the
// stage at a glance rather than by reading its name. Looked for in two places:
//   <machine root>/config/device_images/<name>.png  - this machine's own photo
//                                                     (wins; survives updates)
//   assets/devices/<name>.png                       - the SHIPPED default, in the
//                                                     repo, copied by every build
// No file draws a labelled placeholder naming where to put one; a missing
// picture costs nothing else.
void App::DrawMotionDevicePicture(const std::string& devName)
{
    // 250 px, shrunk to the panel when the right panel is narrower; centred.
    const float avail = ImGui::GetContentRegionAvail().x;
    const float kBox  = (std::max)(64.0f, (std::min)(250.0f, avail));
    if (avail > kBox) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - kBox) * 0.5f);
    const std::string rootRel = "config/device_images/" + devName + ".png";
    const std::string rootPic = MachinePath(rootRel);
    const std::string exePic  = "assets/devices/" + devName + ".png";

    auto it = motionDevPic_.find(devName);
    if (it == motionDevPic_.end()) {
        MarkIcon ic;
        std::error_code ec;
        for (const std::string& p : {rootPic, exePic}) {
            if (!std::filesystem::exists(p, ec)) continue;
            std::vector<unsigned char> rgba;
            int w = 0, h = 0;
            if (LoadPngRGBA(p.c_str(), &rgba, &w, &h)) {
                ic.tex = host_.MakeTextureRGBA(w, h, rgba.data());
                ic.w   = w;
                ic.h   = h;
                break;
            }
            FD_WARN("ui", "device picture '%s' could not be decoded", p.c_str());
        }
        it = motionDevPic_.emplace(devName, ic).first;   // cache the miss as well
    }
    const MarkIcon& pic = it->second;

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + kBox, p0.y + kBox);
    ImGui::InvisibleButton("##devpic", ImVec2(kBox, kBox));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, ImGui::GetColorU32(ImGuiCol_FrameBg), 4.0f);

    if (pic.tex && pic.w > 0 && pic.h > 0) {
        // Fit inside the box, aspect kept, centred.
        const float s  = (std::min)(kBox / static_cast<float>(pic.w),
                                    kBox / static_cast<float>(pic.h));
        const float iw = static_cast<float>(pic.w) * s, ih = static_cast<float>(pic.h) * s;
        const ImVec2 a(p0.x + (kBox - iw) * 0.5f, p0.y + (kBox - ih) * 0.5f);
        dl->AddImage((ImTextureID)(intptr_t)pic.tex, a, ImVec2(a.x + iw, a.y + ih));
    } else {
        const char* l1 = "no picture";
        const ImVec2 t1 = ImGui::CalcTextSize(l1);
        dl->AddText(ImVec2(p0.x + (kBox - t1.x) * 0.5f, p0.y + kBox * 0.5f - t1.y),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled), l1);
        const std::string l2 = devName + ".png";
        const ImVec2 t2 = ImGui::CalcTextSize(l2.c_str());
        dl->AddText(ImVec2(p0.x + (std::max)(4.0f, (kBox - t2.x) * 0.5f), p0.y + kBox * 0.5f + 2.0f),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled), l2.c_str());
    }
    dl->AddRect(p0, p1, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);

    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Device picture (square PNG, shown at up to 250x250)\n"
                          "%s   <- this machine's own (wins)\n"
                          "%s   <- shipped default\n"
                          "Right-click: reload",
                          rootPic.c_str(), exePic.c_str());
    if (ImGui::BeginPopupContextItem("##devpicctx")) {
        if (ImGui::MenuItem("Reload picture")) {
            if (pic.tex) host_.DeleteTexture(pic.tex);
            motionDevPic_.erase(it);
        }
        ImGui::EndPopup();
    }
}

void App::DrawMotionDetailPanel()
{
    const size_t n = hardware_.MotionCount();
    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;
    if (!m) {
        ImGui::TextDisabled("Select a device from the list.");
        return;
    }
    // This page is showing this device live — ask it to poll fast while we're here.
    m->RequestFastPoll();

    const size_t    idx = static_cast<size_t>(selectedMotion_);
    const ConnState st  = hardware_.MotionState(idx);

    // The device name is the one fact to check before touching anything below
    // it, so it is set large; the state rides the same line at the same size.
    const std::string& devName = hardware_.MotionName(idx);
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.8f);
    ImGui::TextUnformatted(devName.c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "[%s]", StateText(st));
    ImGui::PopFont();
    ImGui::Separator();

    // The device's picture is at the top of the right panel (DrawMotionPositionsPanel).
    const std::vector<std::string> axes = m->Axes();
    std::string axesLine;
    for (size_t i = 0; i < axes.size(); ++i) { if (i) axesLine += ' '; axesLine += axes[i]; }
    ImGui::Text("Type : %s", m->Type().c_str());
    ImGui::Text("Axes : %s", axesLine.c_str());
    if (m->SupportsReference()) DrawMotionReference(idx, m);
    if (m->SupportsPivot()) DrawMotionPivot(idx, m);
    if (m->SupportsBuffers()) DrawMotionBuffers(idx, m);

    if (st == ConnState::Failed) {
        ImGui::Spacing();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(StateColor(ConnState::Failed), "error: %s", m->LastError().c_str());
        ImGui::PopTextWrapPos();
    }

    // ---- Tabbed content: node-edge motion maps | scanning | profile ----
    // The saved-position table lives in the RIGHT panel (DrawMotionPositionsPanel)
    // so it stays on screen beside whichever tab is open; Mapped Path is first
    // and therefore the tab the page opens on.
    ImGui::Spacing();
    if (ImGui::BeginTabBar("motiontabs")) {
        if (ImGui::BeginTabItem("Mapped Path")) {
            DrawMotionMapTab(idx, m);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Scanning")) {
            DrawMotionScanTab(idx, m);
            ImGui::EndTabItem();
        }
        if (m->SupportsProfile() && ImGui::BeginTabItem("Profile")) {
            DrawMotionProfileTab(idx, m);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

// The motion profile the controller is set to, per axis — what an operator
// would otherwise type ?VEL0 / ?ACC0 / ?DEC0 / ?JERK0 / ?KDEC0 into the ACS
// terminal for. READ-ONLY: the values are the driver's cached read-back (at
// connect, after every velocity change FireDog makes, and on Read), so this
// draw never waits on the controller. Note that VEL is the one FireDog writes:
// a move step's velocity_mm_s and the jog speed set it on every axis.
void App::DrawMotionProfileTab(std::size_t devIdx, IMotionController* m)
{
    const MotionProfile pr        = m->Profile();
    const bool          connected = hardware_.MotionState(devIdx) == ConnState::Connected;

    ImGui::PushID("profile");
    ImGui::BeginDisabled(!connected);
    if (ImGui::SmallButton("Read")) m->RequestProfileRead();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Read VEL / ACC / DEC / JERK / KDEC back from the controller");
    ImGui::SameLine();
    if (!connected)
        ImGui::TextDisabled("not connected");
    else if (!pr.valid)
        ImGui::TextDisabled("reading...");
    else
        ImGui::TextDisabled("controller values, user units (mm, s)");
    if (!pr.error.empty())
        ImGui::TextColored(StateColor(ConnState::Failed), "read failed: %s", pr.error.c_str());
    if (pr.valid) {
        if (pr.dynamic) {
            ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f),
                               "DYNAMIC profile on the controller (until the next ordinary move)");
            ImGui::SameLine();
            ImGui::BeginDisabled(!connected || orch_.Busy());
            if (ImGui::SmallButton("Restore default")) {
                motDynMsg_ = workflow_.RestoreDefaultProfile(devIdx)
                                 ? std::string()
                                 : "restore failed: " + m->LastError();
            }
            ImGui::EndDisabled();
        } else if (!pr.defaults.empty()) {
            ImGui::TextDisabled("default profile (as read at connect)");
        }
    }

    if (pr.valid && !pr.axes.empty() &&
        ImGui::BeginTable("##acsprofile", 6,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                              ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Axis");
        ImGui::TableSetupColumn("VEL mm/s");
        ImGui::TableSetupColumn("ACC mm/s2");
        ImGui::TableSetupColumn("DEC mm/s2");
        ImGui::TableSetupColumn("JERK mm/s3");
        ImGui::TableSetupColumn("KDEC mm/s2");
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < pr.axes.size(); ++i) {
            const AxisProfile& a = pr.axes[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s (%zu)", a.axis.c_str(), i);   // (n) = the ACS axis index, as in ?VELn
            if (!a.valid) {
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("--");
                continue;
            }
            const double v[5] = {a.vel, a.acc, a.dec, a.jerk, a.kdec};
            for (int c = 0; c < 5; ++c) {
                ImGui::TableSetColumnIndex(c + 1);
                ImGui::Text("%g", v[c]);
            }
        }
        ImGui::EndTable();
    }

    // What every ordinary move restores: the first complete read after connect.
    if (pr.dynamic && !pr.defaults.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("Default (restored by the next ordinary move):");
        for (const AxisProfile& d : pr.defaults)
            ImGui::TextDisabled("  %s  ACC %g  DEC %g  JERK %g", d.axis.c_str(), d.acc, d.dec,
                                d.jerk);
    }
    ImGui::PopID();
}

// Reference state of a device that can lose it (a PI hexapod forgets it on a
// controller restart and then refuses every move), and the button that homes
// it again. Referencing moves the platform on its own path to the reference
// position, so it sits behind the jog arm, is locked while anything runs, and
// asks first. Stop (jog panel) aborts it.
void App::DrawMotionReference(std::size_t devIdx, IMotionController* m)
{
    using RefState = IMotionController::RefState;
    const RefState rs        = m->Referenced();
    const bool     connected = hardware_.MotionState(devIdx) == ConnState::Connected;
    const bool     canRef    = connected && jogArmed_ && !orch_.Busy() &&
                               !workflow_.PathActive() && rs != RefState::Referencing;
    const std::string& name  = hardware_.MotionName(devIdx);

    ImGui::PushID("reference");
    ImGui::TextUnformatted("Reference:");
    ImGui::SameLine();
    switch (rs) {
        case RefState::Yes:         ImGui::TextColored(kServoOn, "referenced"); break;
        case RefState::No:          ImGui::TextColored(kServoOff, "NOT referenced"); break;
        case RefState::Referencing: ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                                       "referencing..."); break;
        default:                    ImGui::TextDisabled("--"); break;
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!canRef);
    if (ImGui::SmallButton("Reference...")) { refMsg_.clear(); openRefConfirm_ = true; }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Home the hexapod (FRF, all axes). Needed after a controller\n"
                          "restart: until it is referenced, every move is refused.\n"
                          "Arm (below) to enable; Stop aborts it.");
    if (connected && rs == RefState::No && !jogArmed_) {
        ImGui::SameLine();
        ImGui::TextDisabled("arm to reference");
    }
    if (!refMsg_.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", refMsg_.c_str());

    if (openRefConfirm_) { ImGui::OpenPopup("Reference Hexapod"); openRefConfirm_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Reference Hexapod", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (!canRef) {
            ImGui::CloseCurrentPopup();
        } else {
            ImGui::Text("Reference %s?", name.c_str());
            ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f),
                               "The platform moves to its reference position on its own path.");
            ImGui::TextDisabled("Make sure the gripper and anything it holds are clear.");
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.42f, 0.12f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.52f, 0.16f, 1.0f));
            if (ImGui::Button("Reference")) {
                if (!workflow_.StartReference(devIdx))
                    refMsg_ = "reference not started: " + m->LastError();
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    ImGui::PopID();
}

// Rotation pivot of a PI hexapod (GCS SPI / qSPI, shown as X Y Z): the
// controller's value, a Read that re-queries it, and an editor that writes a
// new one. The displayed value is always the driver's cached read-back, never
// what was typed. Writing changes what every later U/V/W move does, so it sits
// behind the jog arm and is locked while anything runs.
void App::DrawMotionPivot(std::size_t devIdx, IMotionController* m)
{
    const PivotPoint pv        = m->Pivot();
    const bool       connected = hardware_.MotionState(devIdx) == ConnState::Connected;

    // Re-seed the fields on a device switch or a fresh read / write-back.
    if (pivotEditDev_ != static_cast<int>(devIdx) || pivotEditSeq_ != pv.seq) {
        if (pivotEditDev_ != static_cast<int>(devIdx)) pivotMsg_.clear();
        pivotEditDev_ = static_cast<int>(devIdx);
        pivotEditSeq_ = pv.seq;
        if (pv.valid) {
            pivotEdit_[0] = pv.x;
            pivotEdit_[1] = pv.y;
            pivotEdit_[2] = pv.z;
        }
    }

    ImGui::PushID("pivot");
    if (pv.valid) ImGui::Text("Pivot: X %.4f  Y %.4f  Z %.4f mm", pv.x, pv.y, pv.z);
    else          ImGui::TextDisabled("Pivot: -- (not read)");
    ImGui::SameLine();
    ImGui::BeginDisabled(!connected);
    if (ImGui::SmallButton("Read")) {
        pivotMsg_.clear();
        m->RequestPivotRead();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Query the pivot point from the controller (qSPI).");

    const float fw = 90.0f;
    static const char* const kLbl[3] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i) {
        if (i) ImGui::SameLine();
        ImGui::SetNextItemWidth(fw);
        ImGui::InputDouble(kLbl[i], &pivotEdit_[i], 0.0, 0.0, "%.4f");
    }

    // The C-887 accepts SPI only with the platform at zero rotation; say so
    // before the click rather than after the refusal.
    const std::map<std::string, double> pos = m->Positions();
    bool rotated = false;
    for (const char* r : {"U", "V", "W"}) {
        const auto it = pos.find(r);
        if (it != pos.end() && std::fabs(it->second) > 1e-3) rotated = true;
    }

    const bool canWrite = connected && pv.valid && jogArmed_ && !orch_.Busy() &&
                          !workflow_.PathActive();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canWrite);
    if (ImGui::Button("Write")) {
        if (workflow_.SetPivot(devIdx, pivotEdit_[0], pivotEdit_[1], pivotEdit_[2])) {
            pivotMsg_    = "pivot written";
            pivotMsgErr_ = false;
        } else {
            pivotMsg_    = "write refused: " + m->LastError();
            pivotMsgErr_ = true;
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Set the point U/V/W rotate about (X Y Z, mm).\n"
                          "Every later rotation - jog, recipes, scans - turns about it.\n"
                          "Accepted only with U, V and W at 0.\n"
                          "A written pivot also becomes this hexapod's default: FireDog\n"
                          "writes it back whenever the controller reports another value\n"
                          "(a controller restart resets it to 0,0,0).");
    ImGui::SameLine();
    ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms

    if (connected) {
        if (orch_.Busy() || workflow_.PathActive())
            ImGui::TextDisabled("pivot locked while a run or path is active");
        else if (!jogArmed_)
            ImGui::TextDisabled("arm to write a new pivot");
        else if (rotated)
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                               "U/V/W are not 0 - the controller will refuse a new pivot");
    }
    if (const std::optional<PivotDefault> d = workflow_.PivotDefaultFor(devIdx)) {
        const bool off = pv.valid && (std::fabs(pv.x - d->x) > 1e-4 ||
                                      std::fabs(pv.y - d->y) > 1e-4 ||
                                      std::fabs(pv.z - d->z) > 1e-4);
        if (off)
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                               "default X %.4f  Y %.4f  Z %.4f - controller differs; FireDog writes it back (see log)",
                               d->x, d->y, d->z);
        else
            ImGui::TextDisabled("default X %.4f  Y %.4f  Z %.4f (restored after a controller restart)",
                                d->x, d->y, d->z);
    } else {
        ImGui::TextDisabled("no default pivot yet - Write one to keep it across controller restarts");
    }
    if (!pivotMsg_.empty()) {
        if (pivotMsgErr_) ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", pivotMsg_.c_str());
        else              ImGui::TextDisabled("%s", pivotMsg_.c_str());
    }
    ImGui::PopID();
}

// ACS program buffers: Run is ACSPL+'s `START <buffer>, <label>` (buffer 1 at
// its STARTUP label; buffer 2 from its first line). A program can move the
// axes, so Run sits behind the jog arm and is locked while anything runs, or
// while that buffer already runs. Stop is red and live only while the buffer
// runs, and is never gated otherwise. Buffer 1 brings Z to 25 mm (absolute,
// axis space) once STARTUP ends on its own - for physical clearance; a Stop
// cancels that move.
void App::DrawMotionBuffers(std::size_t devIdx, IMotionController* m)
{
    const bool connected = hardware_.MotionState(devIdx) == ConnState::Connected;
    const bool canRun    = connected && jogArmed_ && !orch_.Busy() && !workflow_.PathActive();

    ImGui::PushID("acsbuf");
    // label "" = first line; thenAxis "" = no follow-up move
    struct BufferButton { int buf; const char* label; const char* thenAxis; double thenTarget; };
    static constexpr BufferButton kBuffers[] = {{1, "STARTUP", "Z", 25.0}, {2, "", "", 0.0}};

    ImGui::TextUnformatted("Buffers:");
    for (const BufferButton& b : kBuffers) {
        const int  buf     = b.buf;
        const bool running = connected && workflow_.BufferRunning(devIdx, buf);
        ImGui::PushID(buf);
        char label[48];
        if (*b.label) std::snprintf(label, sizeof(label), "Run %d, %s", buf, b.label);
        else          std::snprintf(label, sizeof(label), "Run %d", buf);
        ImGui::SameLine();
        ImGui::BeginDisabled(!canRun || running);
        if (ImGui::Button(label)) {
            bufMsgErr_ = !workflow_.RunBuffer(devIdx, buf, b.label, b.thenAxis, b.thenTarget);
            bufMsg_    = bufMsgErr_ ? "buffer " + std::to_string(buf) + " not started: " + m->LastError()
                                    : "buffer " + std::to_string(buf) + " started";
            if (!bufMsgErr_ && *b.thenAxis) {
                char then[64];
                std::snprintf(then, sizeof(then), " - %s to %.3f when it finishes",
                              b.thenAxis, b.thenTarget);
                bufMsg_ += then;
            }
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            char then[80] = "";
            if (*b.thenAxis)
                std::snprintf(then, sizeof(then), "\nThen moves %s to %.3f (absolute) once it ends.",
                              b.thenAxis, b.thenTarget);
            if (*b.label)
                ImGui::SetTooltip("START %d, %s - run ACS buffer %d at label %s.\n"
                                  "The program must be compiled on the controller.%s",
                                  buf, b.label, buf, b.label, then);
            else
                ImGui::SetTooltip("START %d - run ACS buffer %d from its first line.\n"
                                  "The program must be compiled on the controller.%s",
                                  buf, buf, then);
        }
        std::snprintf(label, sizeof(label), "Stop %d", buf);
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
        if (ImGui::Button(label)) {
            bufMsgErr_ = !workflow_.StopBuffer(devIdx, buf);
            bufMsg_    = bufMsgErr_ ? "buffer " + std::to_string(buf) + " not stopped: " + m->LastError()
                                    : "buffer " + std::to_string(buf) + " stopped";
        }
        ImGui::PopStyleColor(2);
        ImGui::EndDisabled();
        ImGui::PopID();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms

    if (connected) {
        if (orch_.Busy() || workflow_.PathActive())
            ImGui::TextDisabled("buffers locked while a run or path is active");
        else if (!jogArmed_)
            ImGui::TextDisabled("arm to run a buffer");
    }
    if (!bufMsg_.empty()) {
        if (bufMsgErr_) ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", bufMsg_.c_str());
        else            ImGui::TextDisabled("%s", bufMsg_.c_str());
    }
    ImGui::PopID();
}

// Scanning tab — test bench for the script-level XCAN aligner: edit the
// algorithm parameters here, hit XCAN, and they are injected into
// hexapod_align.lua exactly as a recipe would inject them (Env.initialCtx ==
// ctx_set steps == fd.param sources). The script is the single algorithm
// implementation — this tab owns no scan logic.
void App::DrawMotionScanTab(std::size_t devIdx, IMotionController* m)
{
    (void)m;
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    ImGui::TextUnformatted("XCAN active alignment (config/scripts/hexapod_align.lua)");
    ImGui::Separator();

    // Device to scan: PI hexapods only (the align algorithm relies on PI's
    // verified settle — ACS reports settled immediately). Defaults to the
    // page's selected device when that is a PI, else the first PI.
    const auto isPi = [&](int i) {
        return i >= 0 && i < static_cast<int>(hardware_.MotionCount()) &&
               hardware_.MotionDevice(static_cast<std::size_t>(i))->Type() == "PI";
    };
    if (!isPi(xcanDevice_)) {
        xcanDevice_ = isPi(static_cast<int>(devIdx)) ? static_cast<int>(devIdx) : -1;
        for (std::size_t i = 0; xcanDevice_ < 0 && i < hardware_.MotionCount(); ++i)
            if (isPi(static_cast<int>(i))) xcanDevice_ = static_cast<int>(i);
    }
    if (xcanDevice_ < 0) {
        ImGui::TextDisabled("(no PI hexapod in the motion fleet)");
        return;
    }
    IMotionController* dev =
        hardware_.MotionDevice(static_cast<std::size_t>(xcanDevice_));

    ImGui::BeginDisabled(running);
    const float w = 180.0f;
    ImGui::SetNextItemWidth(w);
    if (ImGui::BeginCombo("device", hardware_.MotionName(
                                        static_cast<std::size_t>(xcanDevice_)).c_str())) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (!isPi(static_cast<int>(i))) continue;
            if (ImGui::Selectable(hardware_.MotionName(i).c_str(),
                                  xcanDevice_ == static_cast<int>(i)))
                xcanDevice_ = static_cast<int>(i);
        }
        ImGui::EndCombo();
    }
    // Feedback signal: only channels that actually exist on the DataBus.
    ImGui::SetNextItemWidth(w);
    if (ImGui::BeginCombo("feedback topic", xcanTopic_)) {
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (topics.empty())
            ImGui::TextDisabled("(no channels published yet)");
        for (const std::string& t : topics) {
            if (ImGui::Selectable(t.c_str(), t == xcanTopic_))
                std::snprintf(xcanTopic_, sizeof(xcanTopic_), "%s", t.c_str());
        }
        ImGui::EndCombo();
    }
    ImGui::SetNextItemWidth(w);
    ImGui::InputText("axes (descent order)", xcanAxes_, sizeof(xcanAxes_));
    ImGui::SetNextItemWidth(w);
    ImGui::InputText("step ladder (um)", xcanSteps_, sizeof(xcanSteps_));
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("settle (ms)", &xcanSettleMs_);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("consecutive decreases", &xcanConsec_);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("significant drop (%)", &xcanSigDropPct_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("max travel per dir (um)", &xcanTravelUm_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("peak recovery (%)", &xcanImprovePct_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("baseline averages", &xcanAvg_);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("min signal (abort below)", &xcanMinSignal_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("exit threshold (0 = off)", &xcanExitThr_, 0.0, 0.0, "%g");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop scanning the moment a sample reaches this value\n"
                          "and STAY at that point (good-enough early exit).");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("move tolerance (mm)", &xcanTolMm_, 0.0, 0.0, "%g");
    ImGui::Checkbox("trace every sample", &xcanTrace_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("One 'xcan,...' CSV line per sample: time, phase, axis,\n"
                          "direction, step, positions, value, and delta vs the\n"
                          "previous sample. Persists in the daily log file:\n"
                          "  findstr \"xcan,\" logs\\firedog_*.log > scan.csv");
    ImGui::EndDisabled();

    ImGui::Spacing();
    const bool canRun = !orch_.Busy() && dev && dev->IsConnected();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("XCAN", ImVec2(110.0f, 0.0f))) {
        // The product's script overrides the config one when present.
        std::string script = "config/scripts/hexapod_align.lua";
        for (const ScriptInfo& si : session_.Scripts()) {
            if (si.name == "hexapod_align") { script = si.path; break; }
        }
        LuaEngine::Env env = session_.MakeLuaEnv();
        env.initialCtx = {
            {"align_device", hardware_.MotionName(static_cast<std::size_t>(xcanDevice_))},
            {"align_topic", std::string(xcanTopic_)},
            {"align_axes", std::string(xcanAxes_)},
            {"align_steps_um", std::string(xcanSteps_)},
            {"align_settle_ms", static_cast<double>(xcanSettleMs_)},
            {"align_consec_limit", static_cast<double>(xcanConsec_)},
            {"align_sig_drop_pct", xcanSigDropPct_},
            {"align_max_travel_um", xcanTravelUm_},
            {"align_improve_pct", xcanImprovePct_},
            {"align_avg_count", static_cast<double>(xcanAvg_)},
            {"align_min_signal", xcanMinSignal_},
            {"align_exit_threshold", xcanExitThr_},
            {"align_tol_mm", xcanTolMm_},
            {"align_trace", xcanTrace_ ? 1.0 : 0.0},
        };
        scriptStartErr_.clear();
        if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
            FD_WARN("script", "xcan start rejected: %s", scriptStartErr_.c_str());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(110.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();
    if (!canRun && !running) {
        ImGui::SameLine();
        ImGui::TextDisabled(!dev || !dev->IsConnected() ? "(device not connected)"
                                                        : "(another run is active)");
    }
    if (!scriptStartErr_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           scriptStartErr_.c_str());
    }

    // ---- Live status + results (from the shared engine/ctx) ----
    ImGui::Separator();
    const bool thisScan = st.scriptName == "hexapod_align";
    ImGui::Text("State: %s", LuaEngine::StateName(st.state));
    if (running && thisScan && !st.lastCommand.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("| %s", st.lastCommand.c_str());
    }
    if (thisScan) {
        for (const auto& [key, val] : st.ctx) {
            if (key == "align_baseline" || key == "align_best" ||
                key == "align_delta" || key == "align_gain" ||
                key == "align_exit_hit")
                ImGui::Text("%s: %s", key.c_str(), CtxValueToString(val).c_str());
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running)
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
    }
}

// Positions tab — the saved/taught position table (persisted; Move drives
// through Workflow).
// Right panel of the Motion page: the selected device's saved positions. The
// selection is resolved by the left panel, which draws first.
void App::DrawMotionPositionsPanel()
{
    const size_t n = hardware_.MotionCount();
    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;
    if (!m) {
        ImGui::TextDisabled("Select a device from the list.");
        return;
    }
    DrawMotionDevicePicture(hardware_.MotionName(static_cast<size_t>(selectedMotion_)));
    ImGui::Spacing();
    DrawMotionPositionsTab(static_cast<size_t>(selectedMotion_), m);
}

void App::DrawMotionPositionsTab(std::size_t idx, IMotionController* m)
{
    const std::vector<std::string> axes = m->Axes();

    // Laid out for the narrow right panel: a title, then one row of buttons,
    // then the selection and its tools, then the table.
    ImGui::TextUnformatted("Saved positions");
    ImGui::Separator();

    // The starting "Home" row for a device with no positions is seeded by
    // App::Update via ProductSession::SeedHomePosition — drawing must not
    // mutate the document.
    const std::string      devName = hardware_.MotionName(idx);
    std::vector<SavedPos>& saved   = session_.Positions()[devName];

    if (ImGui::SmallButton("+ Add current")) {
        // Ask for the name first — the live position is read when OK is
        // pressed, so the row records where the machine stands at that moment.
        std::snprintf(newPosBuf_, sizeof(newPosBuf_), "P%zu", saved.size());
        openNewPos_ = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("History")) {   // the change journal for this device
        posHistDevice_ = devName;
        posHistUid_.clear();
        showPosHist_ = true;
    }

    // Per-position visibility widget: system positions stay hidden from the
    // table unless revealed here. The hidden flag persists with the position.
    ImGui::SameLine();
    size_t hiddenCount = 0;
    for (const SavedPos& sp : saved) if (sp.hidden) ++hiddenCount;
    if (ImGui::SmallButton("Show/Hide...")) ImGui::OpenPopup("posvis");
    if (hiddenCount > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("(%zu hidden)", hiddenCount);
    }
    if (ImGui::BeginPopup("posvis")) {
        ImGui::TextDisabled("Checked positions are shown in the table.");
        ImGui::Checkbox("Reveal hidden rows (dimmed)", &showHiddenPos_);
        ImGui::Separator();
        for (size_t r = 0; r < saved.size(); ++r) {
            ImGui::PushID(static_cast<int>(r));
            bool visible = !saved[r].hidden;
            if (ImGui::Checkbox(saved[r].name, &visible)) {
                saved[r].hidden = !visible;
                session_.SavePositions();
            }
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }

    // ---- The selected row and its toolbar --------------------------------
    // One selection, six actions above the table, instead of six buttons on
    // every row: the table is names and numbers again, and an action lands on
    // the row the operator can see is highlighted. The selection is a uid
    // (App::posSelUid_) so a device switch or a delete cannot re-aim it.
    int sel = -1;
    for (size_t r = 0; r < saved.size(); ++r)
        if (saved[r].uid == posSelUid_) { sel = static_cast<int>(r); break; }
    if (sel >= 0 && saved[static_cast<size_t>(sel)].hidden && !showHiddenPos_)
        sel = -1;   // a hidden selection is not a visible target
    const SavedPos* selPos = (sel >= 0) ? &saved[static_cast<size_t>(sel)] : nullptr;

    if (selPos) ImGui::Text("Selected: %s", selPos->name);
    else        ImGui::TextDisabled("Selected: (click a row; double-click = Move)");
    ImGui::BeginDisabled(!selPos);
    // Move and Update both hand off to a confirmation dialog below: one
    // drives the machine, the other overwrites a taught place, and neither
    // should happen on a single click.
    if (IconToolButton(UiIcon("mo_move_to"), "posMove", "Move",
                       "Move the device to this position (asks first)")) {
        moveRow_     = sel;
        openMovePos_ = true;
    }
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_reload"), "posUpd", "Update",
                       "Update: overwrite this position with where the device is now (asks first)")) {
        updRow_     = sel;
        updSecond_  = false;
        openUpdPos_ = true;
    }
    ImGui::SameLine();
    if (IconToolButton(UiIcon("sy_variable"), "posEdit", "Edit",
                       "Edit: type the axis values directly")) {
        editRow_ = sel;
        editVals_.clear();
        for (const auto& a : axes) {
            const auto it = selPos->vals.find(a);
            editVals_[a] = (it != selPos->vals.end()) ? it->second : 0.0;
        }
        openEditPos_ = true;
    }
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_rename"), "posRen", "Rename", "Rename this position")) {
        renameRow_ = sel;
        std::snprintf(renameBuf_, sizeof(renameBuf_), "%s", selPos->name);
        openRenamePos_ = true;
    }
    ImGui::SameLine();
    if (IconToolButton(UiIcon("sy_log"), "posHist", "Hist",
                       "History: this position's change journal")) {
        posHistDevice_ = devName;
        posHistUid_    = selPos->uid;
        showPosHist_   = true;
    }
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_delete"), "posDel", "Delete",
                       "Delete this position (asks first)", true)) {
        deleteRow_     = sel;
        openDeletePos_ = true;
    }
    ImGui::EndDisabled();

    const int cols = 1 + static_cast<int>(axes.size());
    if (ImGui::BeginTable("savedpos", cols,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
            ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {   // fill the panel
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        for (const auto& a : axes) ImGui::TableSetupColumn(a.c_str(), ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        for (size_t r = 0; r < saved.size(); ++r) {
            SavedPos& sp = saved[r];
            if (sp.hidden && !showHiddenPos_) continue;   // system position
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(r));
            if (sp.hidden)   // revealed hidden row — dim the whole row's text
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));

            ImGui::TableSetColumnIndex(0);
            const bool isSel = static_cast<int>(r) == sel;
            if (ImGui::Selectable(sp.name, isSel,
                                  ImGuiSelectableFlags_SpanAllColumns |
                                      ImGuiSelectableFlags_AllowDoubleClick)) {
                posSelUid_ = sp.uid;
                // A double-click is the one-gesture Move; it still lands on
                // the confirmation dialog, never straight on the axes.
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    moveRow_     = static_cast<int>(r);
                    openMovePos_ = true;
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("uid: %s", sp.uid.c_str());

            int c = 1;
            for (const auto& a : axes) {
                ImGui::TableSetColumnIndex(c++);
                const auto it = sp.vals.find(a);
                ImGui::Text("%.4f", (it != sp.vals.end()) ? it->second : 0.0);
            }
            if (sp.hidden) ImGui::PopStyleColor();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    // ---- Delete confirmation dialog ----
    if (openDeletePos_) { ImGui::OpenPopup("Delete Position"); openDeletePos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = deleteRow_ >= 0 && static_cast<size_t>(deleteRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            const SavedPos& sp = saved[static_cast<size_t>(deleteRow_)];
            ImGui::Text("Delete position '%s' on %s?", sp.name, devName.c_str());
            ImGui::TextDisabled("uid %s — anything referencing it will break. This cannot be undone.",
                                sp.uid.c_str());
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
            if (ImGui::Button("Delete")) {
                FD_INFO("saved", "deleted '%s' (uid %s, %s)", sp.name, sp.uid.c_str(), devName.c_str());
                saved.erase(saved.begin() + deleteRow_);
                session_.SavePositions();
                if (renameRow_ == deleteRow_) renameRow_ = -1;   // row gone — drop pending rename
                if (moveRow_   == deleteRow_) moveRow_   = -1;
                if (updRow_    == deleteRow_) updRow_    = -1;
                if (editRow_   == deleteRow_) editRow_   = -1;
                deleteRow_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) { deleteRow_ = -1; ImGui::CloseCurrentPopup(); }
        }
        ImGui::EndPopup();
    }

    // Saved vs live, one row per axis, with the delta and the largest |delta|
    // handed back. Shared by the Move and Update dialogs, which show the same
    // table and differ only in which column the machine ends up on.
    const auto drawDiffTable = [&](const SavedPos& sp, const std::map<std::string, double>& live,
                                   const char* savedHdr, const char* liveHdr) -> double {
        double maxAbs = 0.0;
        if (ImGui::BeginTable("posdiff", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Axis");
            ImGui::TableSetupColumn(savedHdr);
            ImGui::TableSetupColumn(liveHdr);
            ImGui::TableSetupColumn("Delta");
            ImGui::TableHeadersRow();
            for (const auto& a : axes) {
                const auto si = sp.vals.find(a);
                const auto li = live.find(a);
                const double s = (si != sp.vals.end()) ? si->second : 0.0;
                const double l = (li != live.end())    ? li->second : 0.0;
                const double d = l - s;
                if (std::fabs(d) > maxAbs) maxAbs = std::fabs(d);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(a.c_str());
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.4f", s);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f", l);
                ImGui::TableSetColumnIndex(3);
                if (std::fabs(d) > kPosUpdWarnMm)
                    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.30f, 1.0f), "%+.4f", d);
                else
                    ImGui::Text("%+.4f", d);
            }
            ImGui::EndTable();
        }
        return maxAbs;
    };

    // ---- Move confirmation (row Move asks before the device goes) ----
    if (openMovePos_) { ImGui::OpenPopup("Move To Position"); openMovePos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Move To Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = moveRow_ >= 0 && static_cast<size_t>(moveRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            const SavedPos& sp = saved[static_cast<size_t>(moveRow_)];
            ImGui::Text("Move %s to '%s'?", devName.c_str(), sp.name);
            ImGui::TextDisabled("All axes travel together. Make sure the path is clear.");
            drawDiffTable(sp, m->Positions(), "Target", "Now");
            // The dynamic profile, shown BEFORE the move from the cached VEL so
            // the operator sees the numbers; the driver re-reads VEL itself.
            // Never while a run is active: the run's own next move would put
            // the default back mid-test.
            const bool dynMove = motDynOn_ && m->SupportsProfile() && !orch_.Busy();
            if (dynMove) {
                const MotionProfile pr = m->Profile();
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f),
                                   "Dynamic profile (ACC = %g x VEL, DEC = ACC, JERK = %g x ACC):",
                                   motDyn_.accPerVel, motDyn_.jerkPerAcc);
                for (const AxisProfile& a : pr.axes) {
                    if (!a.valid) continue;
                    const double acc = a.vel * motDyn_.accPerVel;
                    ImGui::Text("  %s  VEL %g  ->  ACC %g  DEC %g  JERK %g", a.axis.c_str(),
                                a.vel, acc, acc, acc * motDyn_.jerkPerAcc);
                }
                if (pr.defaults.empty())
                    ImGui::TextColored(StateColor(ConnState::Failed),
                                       "  default profile not read yet - the move will be refused");
            } else if (motDynOn_ && orch_.Busy()) {
                ImGui::TextDisabled("A run is active: this move uses the default profile.");
            }
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.55f, 0.26f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.68f, 0.34f, 1.0f));
            if (ImGui::Button("Move", ImVec2(120.0f, 0.0f))) {
                std::vector<std::pair<std::string, double>> tgt;
                for (const auto& a : axes) {
                    const auto it = sp.vals.find(a);
                    if (it != sp.vals.end()) tgt.emplace_back(a, it->second);
                }
                const bool ok =
                    dynMove ? workflow_.MoveToPositionDynamic(static_cast<size_t>(selectedMotion_),
                                                              tgt, motDyn_)
                            : workflow_.MoveToPosition(static_cast<size_t>(selectedMotion_), tgt);
                if (ok) {
                    FD_INFO("saved", "move %s -> '%s'%s", devName.c_str(), sp.name,
                            dynMove ? " (dynamic profile)" : "");
                    if (dynMove) motDynMsg_.clear();
                } else {
                    FD_WARN("saved", "move %s -> '%s' failed", devName.c_str(), sp.name);
                    if (dynMove) motDynMsg_ = "dynamic move refused: " + m->LastError();
                }
                moveRow_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f))) { moveRow_ = -1; ImGui::CloseCurrentPopup(); }
        }
        ImGui::EndPopup();
    }

    // ---- Update confirmation (row Upd shows before/after and asks; a large
    //      delta needs a second press) ----
    if (openUpdPos_) { ImGui::OpenPopup("Update Position"); openUpdPos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Update Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = updRow_ >= 0 && static_cast<size_t>(updRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            SavedPos& sp = saved[static_cast<size_t>(updRow_)];
            // Live is read every frame, like the New Position dialog: the
            // row records where the machine stands when Update is pressed.
            const std::map<std::string, double> live = m->Positions();
            ImGui::Text("Overwrite '%s' on %s with the current position?", sp.name, devName.c_str());
            const double maxAbs = drawDiffTable(sp, live, "Saved", "Current");
            const bool big = maxAbs > kPosUpdWarnMm;
            ImGui::Spacing();
            if (big) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.30f, 1.0f),
                                   "An axis moves by more than %.0f mm.", kPosUpdWarnMm);
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.30f, 1.0f),
                                   updSecond_ ? "Press Confirm again to overwrite."
                                              : "Press Update twice to overwrite.");
            } else if (updSecond_) {
                updSecond_ = false;   // the machine came back within range
            }
            const char* label = (big && updSecond_) ? "Confirm" : "Update";
            if (big && updSecond_) {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.55f, 0.26f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.68f, 0.34f, 1.0f));
            }
            if (ImGui::Button(label, ImVec2(120.0f, 0.0f))) {
                if (big && !updSecond_) {
                    updSecond_ = true;    // acknowledged; the next press writes
                } else {
                    for (const auto& a : axes) {
                        const auto it = live.find(a);
                        if (it != live.end()) sp.vals[a] = it->second;
                    }
                    FD_INFO("saved", "updated '%s' from current position (max delta %.4f)",
                            sp.name, maxAbs);
                    session_.SavePositions();
                    updRow_ = -1; updSecond_ = false;
                    ImGui::CloseCurrentPopup();
                }
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f))) {
                updRow_ = -1; updSecond_ = false;
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndPopup();
    }

    // ---- Edit dialog (type every axis value and save) ----
    if (openEditPos_) { ImGui::OpenPopup("Edit Position"); openEditPos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Edit Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = editRow_ >= 0 && static_cast<size_t>(editRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            SavedPos& sp = saved[static_cast<size_t>(editRow_)];
            ImGui::Text("Edit '%s' on %s", sp.name, devName.c_str());
            ImGui::TextDisabled("uid %s — values are written on Save only.", sp.uid.c_str());
            ImGui::Spacing();
            bool first = true;
            for (const auto& a : axes) {
                ImGui::PushID(a.c_str());
                ImGui::SetNextItemWidth(180.0f);
                if (first && ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
                first = false;
                ImGui::InputDouble("##v", &editVals_[a], 0.0, 0.0, "%.4f");
                ImGui::SameLine();
                // Nudge the typed value; still written on Save only. The amount
                // is the one the Recipe Editor's Tune uses (edTuneMm_).
                bool tuneEdited = false;
                TuneButton(a.c_str(), editVals_[a], edTuneMm_, tuneEdited);
                if (tuneEdited) SaveUiSettings();
                ImGui::SameLine();
                const auto it = sp.vals.find(a);
                const double cur = (it != sp.vals.end()) ? it->second : 0.0;
                ImGui::Text("%s", a.c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("(saved %.4f)", cur);
                ImGui::PopID();
            }
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.55f, 0.26f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.68f, 0.34f, 1.0f));
            if (ImGui::Button("Save", ImVec2(120.0f, 0.0f))) {
                for (const auto& a : axes) sp.vals[a] = editVals_[a];
                FD_INFO("saved", "edited '%s' (uid %s, %s) by hand", sp.name, sp.uid.c_str(), devName.c_str());
                session_.SavePositions();
                editRow_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f))) { editRow_ = -1; ImGui::CloseCurrentPopup(); }
        }
        ImGui::EndPopup();
    }

    // ---- Rename dialog (name is a label; the uid never changes) ----
    if (openRenamePos_) { ImGui::OpenPopup("Rename Position"); openRenamePos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Rename Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = renameRow_ >= 0 && static_cast<size_t>(renameRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            SavedPos& sp = saved[static_cast<size_t>(renameRow_)];
            ImGui::Text("Position uid %s (unchanged)", sp.uid.c_str());
            ImGui::SetNextItemWidth(320.0f);
            if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
            const bool submit = ImGui::InputText("##rename", renameBuf_, sizeof(renameBuf_),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
            const bool valid = renameBuf_[0] != '\0';
            ImGui::BeginDisabled(!valid);
            if (ImGui::Button("OK") || (submit && valid)) {
                FD_INFO("saved", "renamed '%s' -> '%s' (uid %s)", sp.name, renameBuf_, sp.uid.c_str());
                std::snprintf(sp.name, sizeof(sp.name), "%s", renameBuf_);
                session_.SavePositions();
                renameRow_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) { renameRow_ = -1; ImGui::CloseCurrentPopup(); }
        }
        ImGui::EndPopup();
    }

    // ---- New-position dialog ("+ Add current" names the row before saving) ----
    if (openNewPos_) { ImGui::OpenPopup("New Position"); openNewPos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("New Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Save the current %s position as:", devName.c_str());
        ImGui::SetNextItemWidth(320.0f);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        const bool submit = ImGui::InputText("##newpos", newPosBuf_, sizeof(newPosBuf_),
                                             ImGuiInputTextFlags_EnterReturnsTrue);
        const bool valid = newPosBuf_[0] != '\0';
        // The live axis values as they stand right now — shown so the operator
        // confirms the machine is where they think it is before the row exists.
        const std::map<std::string, double> live = m->Positions();
        for (const auto& a : axes) {
            const auto it = live.find(a);
            ImGui::TextDisabled("%s  %.4f", a.c_str(), (it != live.end()) ? it->second : 0.0);
        }
        ImGui::Spacing();
        ImGui::BeginDisabled(!valid);
        if (ImGui::Button("Save") || (submit && valid)) {
            SavedPos p;
            std::snprintf(p.name, sizeof(p.name), "%s", newPosBuf_);
            p.uid = GenerateUid();
            for (const auto& a : axes) {
                const auto it = live.find(a);
                p.vals[a] = (it != live.end()) ? it->second : 0.0;
            }
            FD_INFO("saved", "added '%s' (uid %s, %s)", p.name, p.uid.c_str(), devName.c_str());
            saved.push_back(std::move(p));
            session_.SavePositions();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

} // namespace firedog
