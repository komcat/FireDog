// PageDeviceReg.cpp : the Registration page (View menu, F8) — where each
// device's tool tip is in the camera device's space, and the two buttons that
// measure and check it. Definitions of App members — see App.h.
//
// The page measures nothing itself. Register launches device_registration.lua
// and Check launches device_reg_check.lua through the Lua engine, exactly as a
// recipe's device_reg_run / device_reg_check_run step would, and every operator
// interaction surfaces in the global prompt / jog / pick windows. Same bargain
// as the Needle and Alignment pages: one code path a registration can come into
// existence through, whichever surface started it.
//
// Nothing here actuates. Both procedures move two devices at once, but they do
// so from inside the procedure behind its own confirmations, on an engine
// thread — not from a button here. There is no jog and no arm checkbox because
// there is nothing on this page to arm.
#include "App.h"

#include "imgui.h"

#include "command/ExecutionContext.h"   // SystemScriptPath — the standard procedures
#include "core/DeviceReg.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <map>
#include <string>
#include <utility>

namespace firedog {

namespace {

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.55f, 0.85f, 0.55f, 1.0f);

void SetBuf(char* buf, std::size_t n, const std::string& text)
{
    std::snprintf(buf, n, "%s", text.c_str());
}

// A motion-device combo. Two of them on this page, and they differ only in the
// label and the buffer they write.
void DeviceCombo(const char* label, char* buf, std::size_t n,
                 const HardwareManager& hw, const char* tip)
{
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo(label, buf)) {
        for (std::size_t i = 0; i < hw.MotionCount(); ++i) {
            const std::string name = hw.MotionName(i);
            if (ImGui::Selectable(name.c_str(), name == buf))
                SetBuf(buf, n, name);
        }
        ImGui::EndCombo();
    }
    if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
}

} // namespace

// Launch the registration procedure. The SHIPPED file, always: it measures the
// machine, so there is no such thing as one product's version of the answer and
// ExecutionContext::ResolveScript refuses a product copy for the same reason.
void App::StartDeviceRegRun()
{
    const std::string script = SystemScriptPath("device_registration.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"reg_device", std::string(regDevice_)},
        {"reg_ref", std::string(regRef_)},
        {"reg_jog_mm", regJogMm_},
        {"reg_autofocus", regAutofocus_},
        {"reg_focus_range_mm", regFocusRangeMm_},
        {"reg_v_travel", regVTravel_},
        {"reg_v_jog", regVJog_},
        {"reg_settle_ms", static_cast<double>(regSettleMs_)},
    };
    regStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &regStartErr_))
        FD_WARN("devreg", "device registration rejected: %s", regStartErr_.c_str());
}

void App::StartDeviceRegCheckRun()
{
    const std::string script = SystemScriptPath("device_reg_check.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"reg_device", std::string(regDevice_)},
        {"reg_check_dx_mm", regCheckDx_},
        {"reg_check_dy_mm", regCheckDy_},
        {"reg_check_dz_mm", regCheckDz_},
        {"reg_tol_mm", regCheckTol_},
        {"reg_adopt_tip", regAdoptTip_},
        {"reg_autofocus", regAutofocus_},
        {"reg_focus_range_mm", regFocusRangeMm_},
        {"reg_v_travel", regVTravel_},
        {"reg_v_jog", regVJog_},
        {"reg_settle_ms", static_cast<double>(regSettleMs_)},
    };
    regStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &regStartErr_))
        FD_WARN("devreg", "registration check rejected: %s", regStartErr_.c_str());
}

void App::DrawDeviceRegPanel()
{
    ImGui::TextDisabled(
        "Relates a device's own axis origin to the camera gantry's, so its tool tip");
    ImGui::TextDisabled(
        "has a known position at every moment - not only just after looking at it.");

    // ---- Which device --------------------------------------------------------
    ImGui::SeparatorText("Device");
    DeviceCombo("device", regDevice_, sizeof(regDevice_), hardware_,
                "The device carrying the tool tip - a gripper on a hexapod.\n"
                "Each one is registered separately: they share no mount and\n"
                "no tool.");
    DeviceCombo("reference", regRef_, sizeof(regRef_), hardware_,
                "The device the CAMERA rides on. Everything is measured and\n"
                "reported in ITS raw axis coordinates, which is the space a\n"
                "pick and a taught position already live in.");

    const DeviceReg reg = workflow_.DeviceRegFor(regDevice_);

    // ---- What the machine currently believes ---------------------------------
    ImGui::SeparatorText("Registration");
    if (!reg.valid) {
        ImGui::TextColored(kBad, "NOT REGISTERED");
        ImGui::TextDisabled("Nothing knows where this device's tip is, so move_to_point");
        ImGui::TextDisabled("refuses and a placement has to measure from scratch every");
        ImGui::TextDisabled("time. Run the registration below.");
    } else {
        ImGui::TextColored(kGood, "tip at device zero:  %+.4f  %+.4f  %+.4f  mm",
                           reg.t.x, reg.t.y, reg.t.z);
        ImGui::TextDisabled("in '%s' axis space; predict with p = R*h + t",
                            reg.refDevice.c_str());

        // The live tip, which is the whole point of the record — and its ABSENCE
        // when the rotations have left the taught ones, which is the other half
        // of the point.
        RegVec3     tip;
        std::string why;
        if (workflow_.DeviceRegTip(regDevice_, tip, &why))
            ImGui::Text("tip now:             %+.4f  %+.4f  %+.4f  mm",
                        tip.x, tip.y, tip.z);
        else
            ImGui::TextColored(kWarn, "tip now: unavailable - %s", why.c_str());

        // Two dates, and which one is old decides how much work a repair is.
        ImGui::Spacing();
        ImGui::Text("rotation fitted %s",
                    reg.regAt.empty() ? "(undated)" : reg.regAt.c_str());
        ImGui::Text("tip taught      %s",
                    reg.tipAt.empty() ? "(undated)" : reg.tipAt.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The rotation belongs to the MOUNT; the tip offset\n"
                              "belongs to the mount AND the tool. Changing a gripper\n"
                              "invalidates the tip alone - one pick, with Check and\n"
                              "'adopt' below - not the whole registration.");

        if (reg.verified) {
            ImGui::TextColored(kGood, "verified - checked %.4f mm at %s",
                               reg.checkResidualMm,
                               reg.checkedAt.empty() ? "(undated)" : reg.checkedAt.c_str());
        } else {
            // Not the same as "accurate to zero". A fit predicts its own origin
            // exactly by construction, so an unchecked registration has tested
            // the measurement and not the rotation — say so where the numbers are.
            ImGui::TextColored(kWarn, "UNVERIFIED - never checked away from the pose it");
            ImGui::TextColored(kWarn, "was fitted at, so nothing has tested the rotation");
        }

        // The measurement behind the numbers. Not decoration: when a tip lands
        // somewhere surprising, "which of these is wrong" is the only useful
        // question, and re-teaching to find out throws the evidence away.
        if (ImGui::TreeNode("How it was measured")) {
            ImGui::Text("jog          %.3f mm on each axis", reg.jogMm);
            ImGui::Text("device pose  X %10.4f  Y %10.4f  Z %10.4f", reg.h0.x, reg.h0.y, reg.h0.z);
            ImGui::Text("tip at rest  X %10.4f  Y %10.4f  Z %10.4f", reg.p0.x, reg.p0.y, reg.p0.z);
            ImGui::Text("after +X     X %10.4f  Y %10.4f  Z %10.4f", reg.p1.x, reg.p1.y, reg.p1.z);
            ImGui::Text("after +Y     X %10.4f  Y %10.4f  Z %10.4f", reg.p2.x, reg.p2.y, reg.p2.z);
            ImGui::Text("after +Z     X %10.4f  Y %10.4f  Z %10.4f", reg.p3.x, reg.p3.y, reg.p3.z);
            ImGui::Spacing();
            ImGui::Text("axis scale       %.4f  %.4f  %.4f   (want 1.0000)",
                        reg.colNorm[0], reg.colNorm[1], reg.colNorm[2]);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("How far the tip actually travelled per millimetre\n"
                                  "commanded. Anything but 1 is a jog that did not\n"
                                  "complete, or two devices that disagree about what\n"
                                  "a millimetre is.");
            ImGui::Text("perpendicular  %+.3f %+.3f %+.3f deg off 90",
                        reg.orthoDeg[0], reg.orthoDeg[1], reg.orthoDeg[2]);
            ImGui::Text("Z vs X cross Y   %.3f deg      handedness %+.0f",
                        reg.crossDeg, reg.handed);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Every matrix in the shipped transforms file has\n"
                                  "det = -1: axis sign conventions are not consistently\n"
                                  "handed. This sign came from the Z jog, not from a\n"
                                  "convention.");
            if (reg.hasRot)
                ImGui::Text("fitted at rotations  U %.3f  V %.3f  W %.3f",
                            reg.taughtRot.x, reg.taughtRot.y, reg.taughtRot.z);
            else
                ImGui::TextDisabled("device reports no U/V/W - no rotation to pin to");
            ImGui::TreePop();
        }
    }

    // ---- Registration run ----------------------------------------------------
    ImGui::SeparatorText("Register (four picks)");
    ImGui::TextDisabled("Finds the tip, jogs the device once on each axis, and follows");
    ImGui::TextDisabled("with the camera. You click the tip; it does the rest.");
    {
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("jog mm", &regJogMm_, 0.0, 0.0, "%.2f");
        if (regJogMm_ < kDeviceRegMinJogMm) regJogMm_ = kDeviceRegMinJogMm;
        if (regJogMm_ > 100.0)              regJogMm_ = 100.0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Longer resolves the angle better - 10 mm picked to\n"
                              "5 um is about 0.03 deg - and it is bounded by travel\n"
                              "and clearance, NOT by the field of view: the camera\n"
                              "drives to where the tip is predicted to have gone.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("settle ms", &regSettleMs_, 0, 0);
        if (regSettleMs_ < 0)     regSettleMs_ = 0;
        if (regSettleMs_ > 60000) regSettleMs_ = 60000;

        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("v travel", &regVTravel_, 0.0, 0.0, "%.2f");
        if (regVTravel_ < 0.0)   regVTravel_ = 0.0;
        if (regVTravel_ > 100.0) regVTravel_ = 100.0;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("v jog", &regVJog_, 0.0, 0.0, "%.2f");
        if (regVJog_ < 0.0)   regVJog_ = 0.0;
        if (regVJog_ > 100.0) regVJog_ = 100.0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("0 leaves each device at whatever velocity it is\n"
                              "already set to. Hexapods are slow; this is worth\n"
                              "setting before a 10 mm jog on three axes.");

        ImGui::Checkbox("focus on the tip at every pose", &regAutofocus_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The focused reference Z IS the tip's height. Without\n"
                              "it there is no Z measurement at all and the fit has\n"
                              "one column of guesswork.");
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("focus range mm", &regFocusRangeMm_, 0.0, 0.0, "%.2f");
        if (regFocusRangeMm_ < 0.01) regFocusRangeMm_ = 0.01;
        if (regFocusRangeMm_ > 20.0) regFocusRangeMm_ = 20.0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Wider than a normal focus on purpose: before the first\n"
                              "fit the camera aims with the typed axis transform,\n"
                              "which is good to a degree or two - several hundred\n"
                              "microns of Z over a 10 mm jog.");
    }

    const bool running = orch_.lua.Running();
    const bool haveDev = hardware_.MotionCount() > 0;
    const bool sameDev = std::string(regDevice_) == std::string(regRef_);
    {
        // Every Run surface gates on the WHOLE orchestration, never on one
        // engine: the program runner goes momentarily idle between queue items,
        // and a check on the engines alone lets a second run slip in.
        ImGui::BeginDisabled(orch_.Busy() || !haveDev || sameDev);
        if (ImGui::Button(reg.valid ? "Re-register device" : "Register device",
                          ImVec2(200.0f, 0.0f)))
            StartDeviceRegRun();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
        ImGui::EndDisabled();

        if (sameDev)
            ImGui::TextColored(kBad, "device and reference are the same - a device "
                                     "cannot be registered to itself");
        else if (orch_.Busy() && !running)
            ImGui::TextDisabled("another run is active");
        else if (!running)
            ImGui::TextDisabled("moves BOTH devices - clear %.1f mm of travel on each "
                                "axis first", regJogMm_);
    }

    // ---- Check ---------------------------------------------------------------
    ImGui::SeparatorText("Check (one pick)");
    ImGui::TextDisabled("Drives to a pose AWAY from the fitted one, predicts the tip, and");
    ImGui::TextDisabled("measures how far off it is. This is what 'verified' means.");
    {
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("dX", &regCheckDx_, 0.0, 0.0, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("dY", &regCheckDy_, 0.0, 0.0, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputDouble("dZ", &regCheckDz_, 0.0, 0.0, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("tol mm", &regCheckTol_, 0.0, 0.0, "%.4f");
        if (regCheckTol_ < 0.001) regCheckTol_ = 0.001;

        const double offset = std::sqrt(regCheckDx_ * regCheckDx_ +
                                        regCheckDy_ * regCheckDy_ +
                                        regCheckDz_ * regCheckDz_);
        if (offset <= 0.0)
            // The failure this page most needs to prevent: a check at the fitted
            // pose returns the measurement by construction, reads as a beautiful
            // number, and has tested nothing.
            ImGui::TextColored(kWarn, "all three are zero - this would check the very "
                                      "pose the fit was\ntaught at, which can verify "
                                      "nothing. Move away from it.");
        else
            ImGui::TextDisabled("checks %.3f mm from the fitted pose", offset);

        ImGui::Checkbox("adopt the measured tip (the gripper was changed)", &regAdoptTip_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Re-teaches the tip offset from this one pick and keeps\n"
                              "the rotation. It zeroes the residual WHATEVER caused it,\n"
                              "so only use it when you know why the tip moved - a new\n"
                              "gripper. A rotation error adopted as a tip offset is\n"
                              "carried forward invisibly.");
        if (regAdoptTip_)
            ImGui::TextColored(kWarn, "will overwrite the tip offset and clear 'verified'");

        ImGui::BeginDisabled(orch_.Busy() || !haveDev || !reg.valid);
        if (ImGui::Button(regAdoptTip_ ? "Check and adopt tip" : "Check registration",
                          ImVec2(200.0f, 0.0f)))
            StartDeviceRegCheckRun();
        ImGui::EndDisabled();
        if (!reg.valid)
            ImGui::TextDisabled("nothing to check until this device is registered");
    }

    if (!regStartErr_.empty())
        ImGui::TextColored(kBad, "%s", regStartErr_.c_str());
    if (running) {
        const LuaEngine::Status lst = orch_.lua.GetStatus();
        ImGui::TextDisabled("%s  %s", LuaEngine::StateName(lst.state),
                            lst.error.empty() ? "" : lst.error.c_str());
    }

    // ---- The fleet -----------------------------------------------------------
    // Every registration at a glance, because "which grippers does this machine
    // actually know about" is a question with one answer and it should not take
    // three combo changes to get it.
    ImGui::SeparatorText("All registered devices");
    const std::map<std::string, DeviceReg> all = workflow_.DeviceRegs();
    bool any = false;
    for (const auto& [name, r] : all) {
        if (!r.valid) continue;
        any = true;
        ImGui::TextColored(r.verified ? kGood : kWarn, "%-14s", name.c_str());
        ImGui::SameLine();
        ImGui::Text("-> %-12s  t (%+.3f, %+.3f, %+.3f)  %s",
                    r.refDevice.c_str(), r.t.x, r.t.y, r.t.z,
                    r.verified ? "verified" : "UNVERIFIED");
    }
    if (!any) ImGui::TextDisabled("(none)");

    // The one thing a reader of this page must not conclude from a green tick.
    ImGui::Spacing();
    ImGui::TextDisabled("A registration is an APPROACH: it gets a tip close, within tens of");
    ImGui::TextDisabled("microns. A placement still measures and corrects - the registration");
    ImGui::TextDisabled("only means that loop lands on its first pass instead of its second.");
}

} // namespace firedog
