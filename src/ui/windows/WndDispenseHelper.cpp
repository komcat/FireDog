// WndDispenseHelper.cpp : the Dispense helper window (the Recipe Editor's
// "Open Dispense helper..." button on a dispense_run step). Definitions of App
// members — see App.h.
//
// A dispense_run step carries a dozen numbers - velocities, heights, dwells -
// and as bare rows in the step form (v_travel, v_approach, ...) nothing says
// which part of the motion each one drives. This window lays them out under
// the needle motion drawing the Needle page uses (ui/Widgets DrawNeedleMotion),
// one column per stage, with the drawing labelled by the step's own values.
//
// It is a VIEW of the step, not a second record: it edits the step the Recipe
// Editor has selected (App::EditorSelectedStep), in place, and the editor's
// Save writes it. The params it owns are the ones whose ParamSpec::group is
// kDispenseHelper; where the dispense point is (position / point / device) and
// which script runs stay on the editor's form, where they are taught.
#include "App.h"

#include "imgui.h"

#include "command/CommandRegistry.h"
#include "command/Commands.h"   // kDispenseHelper
#include "command/Recipe.h"     // kElseBranchBase
#include "core/NeedleCal.h"
#include "ui/Widgets.h"         // DrawNeedleMotion

#include <algorithm>
#include <cstdio>
#include <string>

namespace firedog {

namespace {

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.55f, 0.85f, 0.55f, 1.0f);

// The dispense's column titles under the shared drawing.
const char* const kDispTitle[kNeedleRegions] = {
    "1  Camera over the point, traverse", "2-4  Down to the part", "5-7  Shot and up",
    "8  Back and inspect"};

const ParamSpec* FindParamSpec(const CommandSpec& spec, const char* name)
{
    for (const ParamSpec& ps : spec.params)
        if (ps.name == name) return &ps;
    return nullptr;
}

// The value the step runs with: stored, else the schema default.
nlohmann::json Effective(const CommandSpec& spec, const nlohmann::json& p, const char* name)
{
    if (p.is_object() && p.contains(name)) return p[name];
    const ParamSpec* ps = FindParamSpec(spec, name);
    return ps ? ps->defaultValue : nlohmann::json();
}
double Num(const CommandSpec& spec, const nlohmann::json& p, const char* name)
{
    const nlohmann::json v = Effective(spec, p, name);
    return v.is_number() ? v.get<double>() : 0.0;
}
bool Flag(const CommandSpec& spec, const nlohmann::json& p, const char* name)
{
    const nlohmann::json v = Effective(spec, p, name);
    return v.is_boolean() && v.get<bool>();
}

std::string Fmt(const char* fmt, double a, double b = 0.0, double c = 0.0)
{
    char buf[96];
    std::snprintf(buf, sizeof(buf), fmt, a, b, c);
    return buf;
}

// The param's help, wrapped, under its real name - the field label is the
// friendly one, the tooltip says what it is called in the recipe file.
void ParamTip(const ParamSpec& ps)
{
    if (!ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) return;
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
    ImGui::TextDisabled("%s", ps.name.c_str());
    ImGui::TextUnformatted(ps.help.c_str());
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

// The last fraction of a millimetre, magnified: where the needle STOPS relative
// to where it would TOUCH, and where that touch comes from. Gap and trim are
// two different corrections that both land here, and a single number in the
// big drawing cannot show which is which:
//   calibrated touch = focused camera Z + dZ          (the machine's answer)
//   touch            = calibrated touch + trim        (+ = closer to the part)
//   stop             = touch - gap                    (the standoff)
// Not to scale - drawn with both nonzero so both arrows exist.
void DrawGapTrimDetail(double gap, double trim)
{
    const float w  = ImGui::GetContentRegionAvail().x;
    const float lh = ImGui::GetTextLineHeight();
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const ImU32 cLine = ImGui::GetColorU32(ImGuiCol_Text);
    const ImU32 cDim  = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    const ImU32 cVal  = ImGui::GetColorU32(kWarn);
    const ImU32 cEdge = IM_COL32(95, 95, 100, 255);
    const ImU32 cBody = IM_COL32(205, 205, 210, 255);
    const ImU32 cSurf = IM_COL32(60, 110, 255, 255);

    const float yStop  = o.y + lh + 10.0f;
    const float yCal   = yStop + lh + 14.0f;
    const float yTouch = yCal + lh + 14.0f;
    const float x0 = o.x + 4.0f, x1 = o.x + w - 4.0f;
    const float xNeedle = o.x + 16.0f, xGap = o.x + 36.0f, xTrim = o.x + 56.0f;
    const float xText = o.x + 70.0f;

    const auto dashed = [&](float y, ImU32 col) {
        for (float x = x0; x < x1; x += 10.0f)
            dl->AddLine(ImVec2(x, y), ImVec2(std::min(x + 5.0f, x1), y), col, 1.0f);
    };
    // Double-headed: a distance, not a move.
    const auto span = [&](float x, float ya, float yb, ImU32 col) {
        dl->AddLine(ImVec2(x, ya), ImVec2(x, yb), col, 1.5f);
        dl->AddTriangleFilled(ImVec2(x, ya), ImVec2(x - 4.0f, ya + 7.0f),
                              ImVec2(x + 4.0f, ya + 7.0f), col);
        dl->AddTriangleFilled(ImVec2(x, yb), ImVec2(x - 4.0f, yb - 7.0f),
                              ImVec2(x + 4.0f, yb - 7.0f), col);
    };

    // The needle tip, stopped.
    dl->AddTriangleFilled(ImVec2(xNeedle - 8.0f, yStop - 26.0f),
                          ImVec2(xNeedle + 8.0f, yStop - 26.0f),
                          ImVec2(xNeedle, yStop - 10.0f), cBody);
    dl->AddLine(ImVec2(xNeedle, yStop - 10.0f), ImVec2(xNeedle, yStop), cEdge, 2.0f);

    // The three heights, top to bottom, each named just above its line.
    dashed(yStop, cVal);
    dl->AddText(ImVec2(xText, yStop - lh - 1.0f), cVal,
                Fmt("stop: gap %.3f mm above the touch", gap).c_str());
    dashed(yCal, cDim);
    dl->AddText(ImVec2(xText, yCal - lh - 1.0f), cDim, "calibrated touch: focus Z + dZ");
    dl->AddLine(ImVec2(x0, yTouch), ImVec2(x1, yTouch), cSurf, 3.0f);
    dl->AddText(ImVec2(xText, yTouch - lh - 1.0f), cLine,
                Fmt("touch: calibrated %+.3f mm trim", trim).c_str());
    dl->AddRectFilled(ImVec2(x0, yTouch + 2.0f), ImVec2(x1, yTouch + 10.0f),
                      IM_COL32(60, 110, 255, 60));

    span(xGap, yStop, yTouch, cVal);
    span(xTrim, yCal, yTouch, cLine);

    ImGui::Dummy(ImVec2(w, yTouch + 14.0f - o.y));
}

} // namespace

void App::DrawDispenseHelperWindow()
{
    // The button is gated, but the open bit outlives a page switch and a role
    // never should.
    if (!CanAccess(Page::RecipeEdit)) { showDispHelper_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(1180.0f, 760.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin(kDispenseHelper, &showDispHelper_)) {
        ImGui::End();
        return;
    }

    RecipeStep* s = EditorSelectedStep();
    const CommandSpec* spec = orch_.registry.Find("dispense_run");
    if (!spec || !s || s->type != "dispense_run" || !s->params.is_object()) {
        ImGui::TextDisabled("Select a dispense_run step in the Recipe Editor - this window "
                            "edits the step that is selected there.");
        ImGui::End();
        return;
    }
    nlohmann::json& p = s->params;

    // ---- Which step, and the calibration it will move by -----------------------
    {
        std::string path;
        for (const std::size_t c : edStepSel_) {
            if (!path.empty()) path += ".";
            if (c >= kElseBranchBase) path += "else." + std::to_string(c - kElseBranchBase + 1);
            else                      path += std::to_string(c + 1);
        }
        const Recipe& r = session_.Recipes()[static_cast<std::size_t>(recipeSel_)];
        ImGui::Text("%s  -  step %s%s%s", r.name.c_str(), path.c_str(),
                    s->nickname.empty() ? "" : "  ", s->nickname.c_str());

        // The touch height is not a number in this step: it is the focused
        // camera Z plus the calibrated dZ, which is why gap and trim are offsets.
        const NeedleCal cal = workflow_.Needle();
        if (!cal.valid)
            ImGui::TextColored(kBad, "NEEDLE NOT CALIBRATED - this step will refuse to run "
                                     "(Needle page).");
        else
            ImGui::TextColored(cal.verified ? kGood : kWarn,
                               "touch height = focused camera Z %+.3f mm (needle dZ)%s",
                               cal.dz, cal.verified ? "" : " - calibration UNVERIFIED");
        ImGui::TextDisabled("Edits go straight into the step; Save in the Recipe Editor. "
                            "The point, device and script stay on the editor's form.");
    }
    const bool busy = orch_.Busy();
    if (busy) ImGui::TextColored(kWarn, "A run is active - read-only until it ends.");

    // ---- The drawing, labelled with this step's values -------------------------
    const double vTravel = Num(*spec, p, "v_travel");
    const double vLift   = Num(*spec, p, "v_liftoff");
    const double liftMm  = Num(*spec, p, "lift_mm");
    const bool   verify  = Flag(*spec, p, "verify");
    const std::string travel =
        vTravel > 0.0 ? Fmt("v travel %.1f mm/s", vTravel) : std::string("v travel: keep current");

    const float width = std::max(ImGui::GetContentRegionAvail().x, kNeedleMotionW);
    NeedleMotionText txt;
    txt.traverse = travel;
    txt.approach = Fmt("v approach %.1f mm/s", Num(*spec, p, "v_approach"));
    txt.safety   = Fmt("safety %.2f mm", Num(*spec, p, "safety_mm"));
    txt.final1   = Fmt("v final %.1f mm/s", Num(*spec, p, "v_final"));
    // The stop height in one line; the close-up under the fields says how.
    txt.final2   = Fmt("stops %.3f above touch", Num(*spec, p, "gap_mm"));
    txt.shot     = Fmt("%.0f + shot %.0f + %.0f ms", Num(*spec, p, "before_ms"),
                       Num(*spec, p, "shot_ms"), Num(*spec, p, "after_ms"));
    txt.lift     = liftMm > 0.0 ? Fmt("%.2f mm @ %.1f mm/s", liftMm, vLift)
                                : std::string("0 mm: slide first");
    txt.back     = verify ? travel : std::string("verify off: stays over");
    txt.climbFirst = true;   // a real part may stand between the dot and the camera
    txt.showBack   = verify;
    DrawNeedleMotion(txt, width);

    // ---- The step's settings, each under its stage -----------------------------
    constexpr float kFieldW = 70.0f;
    const auto numD = [&](const char* label, const char* key, const char* fmt) {
        const ParamSpec* ps = FindParamSpec(*spec, key);
        if (!ps) return;
        double v = Num(*spec, p, key);
        ImGui::SetNextItemWidth(kFieldW);
        if (ImGui::InputDouble(label, &v, 0.0, 0.0, fmt)) {
            if (ps->minVal < ps->maxVal) v = std::clamp(v, ps->minVal, ps->maxVal);
            p[key] = v;
        }
        ParamTip(*ps);
    };
    const auto numI = [&](const char* label, const char* key) {
        const ParamSpec* ps = FindParamSpec(*spec, key);
        if (!ps) return;
        int v = static_cast<int>(Num(*spec, p, key));
        ImGui::SetNextItemWidth(kFieldW);
        if (ImGui::InputInt(label, &v, 0, 0)) {
            if (ps->minVal < ps->maxVal)
                v = std::clamp(v, static_cast<int>(ps->minVal), static_cast<int>(ps->maxVal));
            p[key] = v;
        }
        ParamTip(*ps);
    };
    const auto flag = [&](const char* label, const char* key) {
        const ParamSpec* ps = FindParamSpec(*spec, key);
        if (!ps) return;
        bool v = Flag(*spec, p, key);
        if (ImGui::Checkbox(label, &v)) p[key] = v;
        ParamTip(*ps);
    };

    ImGui::BeginDisabled(busy);
    if (ImGui::BeginTable("##disphelper", kNeedleRegions,
                          ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp,
                          ImVec2(width, 0.0f))) {
        for (int i = 0; i < kNeedleRegions; ++i)
            ImGui::TableSetupColumn(kDispTitle[i], ImGuiTableColumnFlags_WidthStretch,
                                    kNeedleRegionW[i]);
        ImGui::TableHeadersRow();
        ImGui::TableNextRow();

        // 1 Camera over the point, then the needle over it.
        ImGui::TableSetColumnIndex(0);
        flag("nudge onto point", "adjust");
        flag("focus at point", "autofocus");
        numD("v travel mm/s", "v_travel", "%.1f");
        if (vTravel <= 0.0) ImGui::TextDisabled("0 = keep the device's speed");

        // 2-4 Down to the part.
        ImGui::TableSetColumnIndex(1);
        flag("ask the operator", "confirm");
        numD("v approach mm/s", "v_approach", "%.1f");
        numD("safety mm", "safety_mm", "%.2f");
        numD("v final mm/s", "v_final", "%.1f");
        numD("gap mm (standoff)", "gap_mm", "%.3f");
        numD("trim mm (touch fix)", "z_trim_mm", "%.3f");
        {
            const double gap  = Num(*spec, p, "gap_mm");
            const double trim = Num(*spec, p, "z_trim_mm");
            DrawGapTrimDetail(gap, trim);
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextDisabled("gap: a deliberate standoff for the bead - the glue and "
                                "the dot size. 0 = stop at the touch.");
            // Trim corrects the CALIBRATION. Now that a calibration checks dZ,
            // a part that needs one is a calibration that needs redoing.
            if (trim != 0.0)
                ImGui::TextColored(kWarn, "trim: corrects the calibrated touch height. "
                                          "Normally 0 with a verified calibration - "
                                          "re-calibrate rather than trim every step.");
            else
                ImGui::TextDisabled("trim: corrects the calibrated touch height "
                                    "(+ = closer). Leave 0 unless this part needs it.");
            ImGui::PopTextWrapPos();
        }

        // 5-7 Shot and up.
        ImGui::TableSetColumnIndex(2);
        numI("wait before ms", "before_ms");
        numI("shot ms", "shot_ms");
        numI("dwell after ms", "after_ms");
        numD("lift mm", "lift_mm", "%.2f");
        numD("v lift-off mm/s", "v_liftoff", "%.1f");

        // 8 Back over the dot.
        ImGui::TableSetColumnIndex(3);
        flag("back over dot", "verify");
        ImGui::BeginDisabled(!verify);
        flag("image to DUT", "capture");
        // Only read with confirm off: a confirm-on run waits for the answer.
        ImGui::BeginDisabled(Flag(*spec, p, "confirm"));
        numI("auto-OK ms", "verify_ms");
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::EndTable();
    }
    ImGui::EndDisabled();

    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("The camera goes to the taught point first; with 'centre on the point' "
                       "the operator nudges X/Y onto this part, and with 'focus' the touch "
                       "height comes from a focus there instead of the taught Z. Then the "
                       "needle travels over it at the camera's Z and comes down: fast to "
                       "safety above the stop, slow to the stop - gap above the touch height, "
                       "moved closer by trim. After the shot Z lifts off, the slide comes up, "
                       "and Z climbs all the way to the camera's Z before anything moves "
                       "sideways.");
    ImGui::PopStyleColor();
    ImGui::End();
}

} // namespace firedog
