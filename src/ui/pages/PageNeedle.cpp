// PageNeedle.cpp : the Needle page (View menu, F7) — the dispense needle's
// lever arm: what it currently is, and the button that measures it again.
// Definitions of App members — see App.h.
//
// The page measures nothing itself. Run launches needle_calibration.lua through
// the Lua engine, exactly as a recipe's needle_cal_run step would, and every
// operator interaction in it surfaces in the global prompt / jog / pick windows.
// Same bargain as the Alignment page: one code path a calibration can come into
// existence through, whichever surface started it.
//
// Nothing here actuates. The calibration DOES fire a shot of glue, but it does
// so from inside the procedure behind the procedure's own confirmations, on an
// engine thread — not from a button on this page. That is why there is no
// test-dispense here and no arm checkbox: there is nothing to arm.
#include "App.h"

#include "imgui.h"

#include "command/ExecutionContext.h"   // SystemScriptPath — the standard procedures
#include "core/Log.h"
#include "core/NeedleCal.h"
#include "hardware/HardwareManager.h"
#include "hardware/io/IIoDevice.h"
#include "hardware/pneumatic/PneumaticSlide.h"

#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>

namespace firedog {

namespace {

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.55f, 0.85f, 0.55f, 1.0f);

// Write `text` into a fixed char buffer — the App members are char arrays so the
// page state survives without heap churn, and a combo only ever stores a name.
void SetBuf(char* buf, std::size_t n, const std::string& text)
{
    std::snprintf(buf, n, "%s", text.c_str());
}

} // namespace

// Launch the calibration procedure. The SHIPPED file, always: it measures the
// machine, so there is no such thing as one product's version of the answer and
// ExecutionContext::ResolveScript refuses a product copy for the same reason.
void App::StartNeedleCalRun()
{
    const std::string script = SystemScriptPath("needle_calibration.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"ndl_device", std::string(ndlDevice_)},
        {"ndl_slide", std::string(ndlSlide_)},
        {"ndl_io", std::string(ndlIo_)},
        {"ndl_pin", std::string(ndlPin_)},
        {"ndl_shot_ms", static_cast<double>(ndlShotMs_)},
        {"ndl_settle_ms", static_cast<double>(ndlSettleMs_)},
        {"ndl_clear_mm", ndlClearMm_},
        {"ndl_autofocus", ndlAutofocus_},
        {"ndl_passes", static_cast<double>(ndlPasses_)},
        {"ndl_tol_mm", ndlTolMm_},
        {"ndl_v_approach", ndlVApproach_},
        {"ndl_v_final", ndlVFinal_},
    };
    ndlStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &ndlStartErr_))
        FD_WARN("needle", "needle calibration rejected: %s", ndlStartErr_.c_str());
}

void App::DrawNeedlePanel()
{
    const NeedleCal cal = workflow_.Needle();

    // ---- What the machine currently believes ---------------------------------
    ImGui::SeparatorText("Calibration");
    if (cal.valid) {
        ImGui::TextColored(kGood, "dX %+.4f   dY %+.4f   dZ %+.4f  mm",
                           cal.dx, cal.dy, cal.dz);
        ImGui::TextDisabled("add dX/dY to a camera pose on '%s' to put the needle there;",
                            cal.device.c_str());
        ImGui::TextDisabled("add dZ to a FOCUSED camera Z to reach the touch height.");
        ImGui::Text("calibrated %s",
                    cal.calibratedAt.empty() ? "(undated)" : cal.calibratedAt.c_str());
        if (cal.verified)
            ImGui::TextDisabled("verified - a second pass agreed to within %.4f mm",
                                cal.residualMm);
        else
            // Not the same as "accurate to zero". A single-pass calibration was
            // never checked against anything, and the number on screen has no
            // error bar at all — say so where the number is.
            ImGui::TextColored(kWarn, "UNVERIFIED - measured once, never re-checked");
    } else {
        ImGui::TextColored(kBad, "NOT CALIBRATED");
        ImGui::TextDisabled("Nothing knows where the needle is relative to the camera,");
        ImGui::TextDisabled("so every dispense step will refuse to run. Seed a rough");
        ImGui::TextDisabled("offset below, then calibrate.");
    }

    // The three poses the offset was subtracted from. Not decoration: when a dot
    // lands somewhere surprising, "which of these is wrong" is the only useful
    // question, and re-teaching to find out throws the evidence away.
    if (cal.valid && ImGui::TreeNode("How it was measured")) {
        ImGui::Text("camera  X %10.4f  Y %10.4f  Z %10.4f   (focused on the coupon)",
                    cal.camX, cal.camY, cal.camZ);
        ImGui::Text("needle  X %10.4f  Y %10.4f  Z %10.4f   (touching, imprinting)",
                    cal.ndlX, cal.ndlY, cal.ndlZ);
        ImGui::Text("imprint X %10.4f  Y %10.4f              (where the dot landed)",
                    cal.dotX, cal.dotY);
        ImGui::TextDisabled("dX/dY = needle - imprint     dZ = needle Z - camera Z");
        ImGui::TreePop();
    }

    // ---- Hardware ------------------------------------------------------------
    ImGui::SeparatorText("Hardware");
    {
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("gantry", ndlDevice_)) {
            for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
                const std::string name = hardware_.MotionName(i);
                if (ImGui::Selectable(name.c_str(), name == ndlDevice_))
                    SetBuf(ndlDevice_, sizeof(ndlDevice_), name);
            }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The device carrying BOTH the camera and the needle.\n"
                              "That they share one device is what lets the offset\n"
                              "stay in raw axis coordinates and never touch the\n"
                              "axis transform.");

        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("slide", ndlSlide_)) {
            for (std::size_t i = 0; i < hardware_.SlideCount(); ++i) {
                if (!hardware_.Slide(i)) continue;
                const std::string name = hardware_.Slide(i)->Name();
                if (ImGui::Selectable(name.c_str(), name == ndlSlide_))
                    SetBuf(ndlSlide_, sizeof(ndlSlide_), name);
            }
            ImGui::EndCombo();
        }

        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("IO device", ndlIo_)) {
            for (std::size_t i = 0; i < hardware_.IoCount(); ++i) {
                const std::string name = hardware_.IoName(i);
                if (ImGui::Selectable(name.c_str(), name == ndlIo_))
                    SetBuf(ndlIo_, sizeof(ndlIo_), name);
            }
            ImGui::EndCombo();
        }

        const IIoDevice* io = nullptr;
        for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == ndlIo_) { io = hardware_.IoDevice(i); break; }
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("shot pin", ndlPin_)) {
            if (!io) {
                ImGui::TextDisabled("(pick an IO device first)");
            } else {
                for (int pin = 0; pin < io->OutputCount(); ++pin) {
                    const std::string name = io->OutputPinName(pin);
                    if (name.empty()) continue;
                    if (ImGui::Selectable(name.c_str(), name == ndlPin_))
                        SetBuf(ndlPin_, sizeof(ndlPin_), name);
                }
            }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("The glue valve. Open time is the shot size - there\n"
                              "is no pressure control in this loop.");
    }

    // ---- Run parameters ------------------------------------------------------
    ImGui::SeparatorText("Run");
    {
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("shot ms", &ndlShotMs_, 0, 0);
        if (ndlShotMs_ < 1)     ndlShotMs_ = 1;
        if (ndlShotMs_ > 60000) ndlShotMs_ = 60000;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("settle ms", &ndlSettleMs_, 0, 0);
        if (ndlSettleMs_ < 0)     ndlSettleMs_ = 0;
        if (ndlSettleMs_ > 60000) ndlSettleMs_ = 60000;

        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("clear mm", &ndlClearMm_, 0.0, 0.0, "%.2f");
        if (ndlClearMm_ < 0.0)  ndlClearMm_ = 0.0;
        if (ndlClearMm_ > 20.0) ndlClearMm_ = 20.0;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Park this far above the previous touch height before\n"
                              "handing the jog to the operator. It is the whole\n"
                              "safety margin on a re-calibration.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("tol mm", &ndlTolMm_, 0.0, 0.0, "%.4f");
        if (ndlTolMm_ < 0.001) ndlTolMm_ = 0.001;

        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("v approach", &ndlVApproach_, 0.0, 0.0, "%.2f");
        if (ndlVApproach_ < 0.001) ndlVApproach_ = 0.001;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputDouble("v final", &ndlVFinal_, 0.0, 0.0, "%.2f");
        if (ndlVFinal_ < 0.001) ndlVFinal_ = 0.001;

        ImGui::Checkbox("focus the camera on the coupon", &ndlAutofocus_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("dZ is measured DOWN FROM a focal plane. Without a\n"
                              "focus it is measured down from wherever Z happened\n"
                              "to be, and then it is not an offset at all.");

        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("passes", &ndlPasses_, 0, 0);
        if (ndlPasses_ < 1) ndlPasses_ = 1;
        if (ndlPasses_ > 5) ndlPasses_ = 5;
        ImGui::SameLine();
        if (ndlPasses_ == 1)
            ImGui::TextColored(kWarn, "one imprint - nothing checks the answer");
        else
            ImGui::TextDisabled("pass 2+ re-imprints and reports the residual");
    }

    // ---- Run / Stop ----------------------------------------------------------
    {
        const bool running = orch_.lua.Running();
        // Every Run surface gates on the WHOLE orchestration, never on one
        // engine: the program runner goes momentarily idle between queue items,
        // and a check on the engines alone lets a second run slip in.
        const bool haveDev = hardware_.MotionCount() > 0;
        ImGui::BeginDisabled(orch_.Busy() || !haveDev);
        if (ImGui::Button(cal.valid ? "Re-calibrate needle" : "Calibrate needle",
                          ImVec2(200.0f, 0.0f)))
            StartNeedleCalRun();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
        ImGui::EndDisabled();

        if (orch_.Busy() && !running)
            ImGui::TextDisabled("another run is active");
        else if (!running)
            ImGui::TextDisabled("puts a dot on a coupon, then asks you to click it");
        if (!ndlStartErr_.empty())
            ImGui::TextColored(kBad, "%s", ndlStartErr_.c_str());
        if (running) {
            const LuaEngine::Status lst = orch_.lua.GetStatus();
            ImGui::TextDisabled("%s  %s", LuaEngine::StateName(lst.state),
                                lst.error.empty() ? "" : lst.error.c_str());
        }
    }

    // ---- Bootstrap seed ------------------------------------------------------
    //
    // With no offset at all the first imprint lands about a hundred millimetres
    // outside a three-millimetre field of view, and the calibration's own
    // "now click the dot" step has nothing to show. A tape-measure estimate is
    // enough to get the imprint into frame, and once it is there the pick makes
    // it exact. Saved with valid = false on purpose: a guess must never satisfy
    // the check that stands in front of a dispense.
    ImGui::SeparatorText("Rough seed (first calibration only)");
    ImGui::TextDisabled("Measure the needle-to-lens distance by hand and enter it here.");
    ImGui::TextDisabled("It only aims the first imprint into the field of view - it is");
    ImGui::TextDisabled("stored as NOT calibrated and will not let anything dispense.");
    ImGui::SetNextItemWidth(110.0f);
    ImGui::InputDouble("seed dX", &ndlSeedDx_, 0.0, 0.0, "%.3f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    ImGui::InputDouble("seed dY", &ndlSeedDy_, 0.0, 0.0, "%.3f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    ImGui::InputDouble("seed dZ", &ndlSeedDz_, 0.0, 0.0, "%.3f");
    ImGui::BeginDisabled(orch_.Busy());
    if (ImGui::Button("Save seed", ImVec2(120.0f, 0.0f))) {
        NeedleCal seed;
        seed.valid    = false;
        seed.device   = ndlDevice_;
        seed.slide    = ndlSlide_;
        seed.ioDevice = ndlIo_;
        seed.shotPin  = ndlPin_;
        seed.dx = ndlSeedDx_;
        seed.dy = ndlSeedDy_;
        seed.dz = ndlSeedDz_;
        workflow_.SetNeedleCal(seed);
        if (SaveNeedleCalFile(NeedleCalLocalPath(), seed))
            FD_INFO("needle", "seed offset (%.3f, %.3f, %.3f) mm saved - NOT a "
                              "calibration, run the procedure next",
                    seed.dx, seed.dy, seed.dz);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Overwrites the current calibration with a guess.\n"
                          "Only worth doing when there is no calibration yet.");
}

} // namespace firedog
