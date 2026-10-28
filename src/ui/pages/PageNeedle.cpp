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

#include <algorithm>
#include <cmath>
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

// The drawing and the parameter table under it are cut into the SAME four
// regions, so each column of fields sits under the part of the motion it
// drives. Drawing units; both stretch to one shared width.
constexpr int   kMotionRegions = 4;
constexpr float kRegionW[kMotionRegions] = {255.0f, 325.0f, 290.0f, 230.0f};
constexpr float kMotionW = 1100.0f;   // sum of kRegionW
const char* const kRegionTitle[kMotionRegions] = {
    "1  Traverse", "2-4  Down to the surface", "5-7  Shot and up", "8  Inspect and check"};

// The numbers the motion drawing is labelled with — the page's own fields.
struct MotionLabels {
    double vTraverse, vApproach, vFinal, vLift;
    double safetyMm, gapMm, liftMm;
};

// The needle motion, drawn the way the process engineer drew it: traverse at
// the camera's Z, slide down, approach to the safety height, the last stretch
// slow, the touch; then lift off, slide up, and the camera back over the dot.
// Both procedures move in this order, so the drawing is the one picture of
// what a Calibrate press or a dispense_run step will do.
//
// Positions stretch horizontally to `width` (so the regions line up with the
// table's columns); heights, the needles and the text do not scale, and every
// label is placed in space its own region leaves free at kMotionW.
void DrawNeedleMotion(const MotionLabels& m, float width)
{
    constexpr float kH = 420.0f;
    constexpr float yT = 150.0f;   // traverse level (needle tip, slide up)
    constexpr float yD = 215.0f;   // slide down
    constexpr float yS = 290.0f;   // safety height
    constexpr float yF = 360.0f;   // surface

    const float sx = width / kMotionW;
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const auto X = [o, sx](float u) { return o.x + u * sx; };
    const auto P = [&](float u, float y) { return ImVec2(X(u), o.y + y); };
    const float lh = ImGui::GetTextLineHeight();

    const ImU32 cLine = ImGui::GetColorU32(ImGuiCol_Text);
    const ImU32 cDim  = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    const ImU32 cSep  = ImGui::GetColorU32(ImGuiCol_Separator);
    const ImU32 cVal  = ImGui::GetColorU32(kWarn);
    const ImU32 cBody = IM_COL32(205, 205, 210, 255);
    const ImU32 cEdge = IM_COL32(95, 95, 100, 255);
    const ImU32 cSurf = IM_COL32(60, 110, 255, 255);

    // The region boundaries, faint: the columns of the table continue them.
    float u = 0.0f;
    for (int i = 0; i + 1 < kMotionRegions; ++i) {
        u += kRegionW[i];
        dl->AddLine(P(u, 0.0f), P(u, kH), cSep, 1.0f);
    }

    // A needle with its tip at (cx, tipY): body, taper, the needle itself.
    // Its width is pixels, not drawing units - a needle does not get fatter
    // because the window did.
    const auto needle = [&](float cx, float tipY) {
        const float x = X(cx);
        const ImVec2 b0(x - 13.0f, o.y + tipY - 135.0f), b1(x + 13.0f, o.y + tipY - 75.0f);
        const ImVec2 t0(x - 13.0f, o.y + tipY - 75.0f), t1(x + 13.0f, o.y + tipY - 75.0f);
        const ImVec2 t2(x, o.y + tipY - 25.0f);
        dl->AddRectFilled(b0, b1, cBody);
        dl->AddRect(b0, b1, cEdge, 0.0f, 0, 1.5f);
        dl->AddTriangleFilled(t0, t1, t2, cBody);
        dl->AddTriangle(t0, t1, t2, cEdge, 1.5f);
        dl->AddLine(t2, ImVec2(x, o.y + tipY), cEdge, 2.0f);
    };
    const auto arrow = [&](float u0, float y0, float u1, float y1) {
        const ImVec2 a = P(u0, y0), b = P(u1, y1);
        dl->AddLine(a, b, cLine, 1.5f);
        float dx = b.x - a.x, dy = b.y - a.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1.0f) return;
        dx /= len;
        dy /= len;
        constexpr float h = 9.0f;
        dl->AddTriangleFilled(b, ImVec2(b.x - dx * h - dy * h * 0.5f, b.y - dy * h + dx * h * 0.5f),
                              ImVec2(b.x - dx * h + dy * h * 0.5f, b.y - dy * h - dx * h * 0.5f),
                              cLine);
    };
    const auto tick = [&](float u0, float u1, float y) {
        dl->AddLine(P(u0, y), P(u1, y), cDim, 1.0f);
    };
    // A step name over the value that drives it; `right` = right-aligned at u.
    const auto label = [&](float u0, float y, const char* name, const char* value,
                           bool right = false) {
        float x = X(u0);
        if (right) x -= ImGui::CalcTextSize(name).x;
        dl->AddText(ImVec2(x, o.y + y), cLine, name);
        if (value) dl->AddText(ImVec2(x, o.y + y + lh), cVal, value);
    };
    char v[96];

    // ---- 1  Traverse --------------------------------------------------------
    needle(40.0f, yT);
    arrow(40.0f, yT, 222.0f, yT);
    needle(238.0f, yT);
    std::snprintf(v, sizeof(v), "v traverse %.1f mm/s", m.vTraverse);
    label(30.0f, yT + 12.0f, "1 traverse", v);

    // ---- 2-4  Down ------------------------------------------------------------
    needle(290.0f, yD);
    tick(270.0f, 340.0f, yT);
    arrow(320.0f, yT, 320.0f, yD);
    label(332.0f, yT + 18.0f, "2 slide down", nullptr);

    tick(270.0f, 340.0f, yD);
    arrow(290.0f, yD + 3.0f, 290.0f, yS);
    std::snprintf(v, sizeof(v), "v approach %.1f mm/s", m.vApproach);
    label(332.0f, yD + 18.0f, "3 approach", v);

    tick(270.0f, 340.0f, yS);
    std::snprintf(v, sizeof(v), "safety %.2f mm", m.safetyMm);
    label(248.0f, yS - lh * 0.5f, v, nullptr, true);
    arrow(290.0f, yS, 290.0f, yF);
    std::snprintf(v, sizeof(v), "v final %.1f mm/s", m.vFinal);
    label(332.0f, yS + 6.0f, "4 final", v);
    std::snprintf(v, sizeof(v), "verify gap %.3f mm", m.gapMm);
    dl->AddText(ImVec2(X(332.0f), o.y + yS + 6.0f + 2.0f * lh), cVal, v);

    // ---- The surface ------------------------------------------------------
    dl->AddLine(P(270.0f, yF), P(850.0f, yF), cSurf, 3.0f);
    dl->AddText(ImVec2(X(300.0f), o.y + yF + 8.0f), cDim, "coupon / part surface");

    // ---- 5-7  Shot, lift off, slide up ----------------------------------------
    needle(620.0f, yF);
    arrow(655.0f, yF, 655.0f, yF - 55.0f);
    tick(640.0f, 760.0f, yF - 55.0f);
    std::snprintf(v, sizeof(v), "%.2f mm @ %.1f mm/s", m.liftMm, m.vLift);
    label(600.0f, yF + 8.0f, "5 shot  6 lift off", v);

    needle(700.0f, yF - 125.0f);
    arrow(740.0f, yF - 55.0f, 740.0f, yF - 125.0f);
    tick(680.0f, 780.0f, yF - 125.0f);
    label(752.0f, yF - 105.0f, "7 slide up", nullptr);

    // ---- 8  Camera back over the dot ----------------------------------------
    arrow(715.0f, 225.0f, 890.0f, 85.0f);
    std::snprintf(v, sizeof(v), "v traverse %.1f mm/s", m.vTraverse);
    label(880.0f, 40.0f, "8 camera back", v);

    ImGui::Dummy(ImVec2(width, kH));
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
        {"ndl_first_clear_mm", ndlFirstClearMm_},
        {"ndl_verify_gap_mm", ndlGapMm_},
        {"ndl_lift_mm", ndlLiftMm_},
        {"ndl_autofocus", ndlAutofocus_},
        {"ndl_passes", static_cast<double>(ndlPasses_)},
        {"ndl_tol_mm", ndlTolMm_},
        {"ndl_z_tol_mm", ndlZTolMm_},
        {"ndl_v_traverse", ndlVTraverse_},
        {"ndl_v_approach", ndlVApproach_},
        {"ndl_v_final", ndlVFinal_},
        {"ndl_v_liftoff", ndlVLiftoff_},
    };
    ndlStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &ndlStartErr_))
        FD_WARN("needle", "needle calibration rejected: %s", ndlStartErr_.c_str());
}

void App::DrawNeedlePanel()
{
    const NeedleCal cal = workflow_.Needle();

    // ---- The motion, drawn, with its parameters under it --------------------
    // One shared width cut into the same four regions, so each column of fields
    // sits under the part of the drawing it drives, and the drawing is labelled
    // with those fields - changing a number changes the picture.
    if (ImGui::CollapsingHeader("Needle motion", ImGuiTreeNodeFlags_DefaultOpen)) {
        const float width = std::max(ImGui::GetContentRegionAvail().x, kMotionW);
        DrawNeedleMotion({ndlVTraverse_, ndlVApproach_, ndlVFinal_, ndlVLiftoff_,
                          ndlClearMm_, ndlGapMm_, ndlLiftMm_},
                         width);

        constexpr float kFieldW = 70.0f;
        const auto num = [](const char* label, double* v, const char* fmt, double lo,
                            double hi, const char* tip) {
            ImGui::SetNextItemWidth(kFieldW);
            ImGui::InputDouble(label, v, 0.0, 0.0, fmt);
            if (*v < lo) *v = lo;
            if (*v > hi) *v = hi;
            if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
        };
        const auto inum = [](const char* label, int* v, int lo, int hi, const char* tip) {
            ImGui::SetNextItemWidth(kFieldW);
            ImGui::InputInt(label, v, 0, 0);
            if (*v < lo) *v = lo;
            if (*v > hi) *v = hi;
            if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
        };

        if (ImGui::BeginTable("##ndlparams", kMotionRegions,
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp,
                              ImVec2(width, 0.0f))) {
            for (int i = 0; i < kMotionRegions; ++i)
                ImGui::TableSetupColumn(kRegionTitle[i], ImGuiTableColumnFlags_WidthStretch,
                                        kRegionW[i]);
            ImGui::TableHeadersRow();
            ImGui::TableNextRow();

            // 1 Traverse - and the camera reference the pass starts from.
            ImGui::TableSetColumnIndex(0);
            ImGui::Checkbox("focus on the coupon", &ndlAutofocus_);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("dZ is measured DOWN FROM a focal plane. Without a\n"
                                  "focus it is measured down from wherever Z happened\n"
                                  "to be, and then it is not an offset at all.");
            num("v traverse mm/s", &ndlVTraverse_, "%.1f", 0.001, 100.0,
                "XY traverse: needle over the spot (1), camera back over the dot (8)");

            // 2-4 Down to the surface.
            ImGui::TableSetColumnIndex(1);
            inum("settle ms", &ndlSettleMs_, 0, 60000,
                 "pause after the slide extends (2), before Z moves");
            num("v approach mm/s", &ndlVApproach_, "%.1f", 0.001, 100.0,
                "Z down to the safety height (3)");
            num("safety mm", &ndlClearMm_, "%.2f", 0.0, 20.0,
                "Verify pass: the fast approach stops this far above the gap,\n"
                "and v final covers the rest.");
            num("v final mm/s", &ndlVFinal_, "%.1f", 0.001, 100.0,
                "Z from the safety height to the verify gap (4)");
            num("verify gap mm", &ndlGapMm_, "%.3f", 0.0, 2.0,
                "Verify pass: the machine stops the needle this far above\n"
                "where the calibration says it touches. The operator jogs\n"
                "the rest; how far they had to go is the Z residual.");
            num("first clear mm", &ndlFirstClearMm_, "%.2f", 0.0, 20.0,
                "Pass 1 (measure) parks this far above the PREVIOUS touch\n"
                "height, then the operator jogs down. Wider than the\n"
                "safety height on purpose: the usual reason to calibrate\n"
                "is a changed needle, which may sit lower than the old one.");

            // 5-7 Shot and up.
            ImGui::TableSetColumnIndex(2);
            inum("shot ms", &ndlShotMs_, 1, 60000,
                 "Valve open time for the imprint (5). Open time is the shot\n"
                 "size - there is no pressure control in this loop.");
            num("lift mm", &ndlLiftMm_, "%.2f", 0.0, 20.0,
                "After the shot, Z lifts this far off the dot (6) before the\n"
                "slide comes up (7).");
            num("v lift-off mm/s", &ndlVLiftoff_, "%.1f", 0.001, 100.0,
                "Z lifting off the dot after the shot (6)");

            // 8 Inspect and check.
            ImGui::TableSetColumnIndex(3);
            inum("passes", &ndlPasses_, 1, 5,
                 "1 measures once and nothing checks it; 2+ verifies");
            num("tol XY mm", &ndlTolMm_, "%.4f", 0.001, 5.0,
                "XY residual a verify pass must hit");
            num("tol Z mm", &ndlZTolMm_, "%.4f", 0.001, 5.0,
                "Z residual a verify pass must hit");
            if (ndlPasses_ == 1)
                ImGui::TextColored(kWarn, "one imprint - nothing\nchecks the answer");
            else
                ImGui::TextDisabled("pass 2+ checks XY and Z");
            ImGui::EndTable();
        }

        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("Calibration pass 1 parks %.2f mm (first clear) above the previous "
                           "touch and you jog the rest; pass 2+ runs the drawing down to the "
                           "verify gap and you close it. A dispense_run step moves the same "
                           "way with its own step parameters: v_travel, v_approach, "
                           "safety_mm, v_final down to gap_mm above the touch, lift_mm, "
                           "v_liftoff.", ndlFirstClearMm_);
        ImGui::PopStyleColor();
    }

    // ---- Run / Stop ----------------------------------------------------------
    // Big on purpose: this is the one button on the page that starts the
    // machine moving, and it sits right under the picture of what it will do.
    {
        constexpr float kRunH = 110.0f;
        const bool running = orch_.lua.Running();
        // Every Run surface gates on the WHOLE orchestration, never on one
        // engine: the program runner goes momentarily idle between queue items,
        // and a check on the engines alone lets a second run slip in.
        const bool haveDev = hardware_.MotionCount() > 0;
        ImGui::BeginDisabled(orch_.Busy() || !haveDev);
        if (ImGui::Button(cal.valid ? "Re-calibrate needle" : "Calibrate needle",
                          ImVec2(260.0f, kRunH)))
            StartNeedleCalRun();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop", ImVec2(kRunH, kRunH))) orch_.lua.RequestStop();
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::BeginGroup();
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
        ImGui::EndGroup();
    }

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
            ImGui::TextDisabled("verified - a second pass agreed to within %.4f mm XY, "
                                "%+.4f mm Z", cal.residualMm, cal.zResidualMm);
        else if (cal.needsZCheck)
            // Verified by the procedure that checked XY only. Its dZ - the number
            // a dispense drives to with nobody on the jog - was never compared
            // with anything, so it is NOT verified, and the page says why.
            ImGui::TextColored(kBad, "RE-VERIFY REQUIRED - checked in XY only, dZ never "
                                     "checked. Re-calibrate (passes >= 2).");
        else if (cal.zChecked)
            ImGui::TextColored(kBad, "VERIFY FAILED - last pass off by %.4f mm XY, "
                                     "%+.4f mm Z", cal.residualMm, cal.zResidualMm);
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

    // For a file edited or copied in by hand. Locked during a run: a dispense
    // reads the offset mid-run, and swapping it under one moves the needle.
    ImGui::BeginDisabled(orch_.Busy());
    if (ImGui::Button("Reload from file", ImVec2(200.0f, 0.0f))) workflow_.ReloadNeedleCal();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Re-read %s\n(or the shipped config/needle_cal.json when it is missing)",
                          NeedleCalLocalPath().c_str());

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
