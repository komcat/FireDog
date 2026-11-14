// RecipeForms.cpp : the schema-driven parameter forms of the Recipe Editor.
// Definitions of App members — see App.h.
//
// DrawParamWidget renders one row per ParamSpec straight off the
// CommandRegistry, so a newly registered command gets a working form for free
// — keep it that way, and add a ParamSpec kind rather than a per-command
// special case. DrawConditionEditor is shared by if / loop-while / wait_until.
#include "App.h"

#include "imgui.h"

#include "ui/Theme.h"     // kTeachOrange (an untaught position)
#include "ui/Widgets.h"
#include "command/StepSummary.h"   // FormatDurationMs
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <initializer_list>
#include <map>
#include <set>
#include <system_error>
#include <string>
#include <vector>

namespace firedog {

namespace {

// A tooltip that WRAPS. ImGui::SetTooltip lays its text out on one line, and a
// parameter's help is a paragraph: on a wide monitor it runs the full width of
// the screen and off both edges, which is exactly where nobody can read it.
// Width in font units so the operator's font size moves it too.
void HelpTooltip(const std::string& text)
{
    if (text.empty()) return;
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
    ImGui::TextUnformatted(text.c_str());
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

} // namespace

// Step inspector: common fields + the schema-driven parameter form. Editable
// unless a run is active (then everything renders disabled).
void App::DrawRecipeInspector(RecipeStep& s, bool canEdit)
{
    ImGui::BeginDisabled(!canEdit);
    ImGui::Text("Type: %s", s.type.c_str());
    char nbuf[64];
    std::snprintf(nbuf, sizeof(nbuf), "%s", s.nickname.c_str());
    if (ImGui::InputText("nickname", nbuf, sizeof(nbuf))) s.nickname = nbuf;
    ImGui::SameLine();
    if (ImGui::SmallButton("gen")) s.nickname = AutoNickname(s);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Fill the nickname from the params.\n"
                          "An EMPTY nickname already shows this text in the tree,\n"
                          "auto-updating as params change.");
    ImGui::Checkbox("enabled", &s.enabled);
    int oe = static_cast<int>(s.onError);
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::Combo("on error", &oe, "abort\0continue\0retry\0"))
        s.onError = static_cast<RecipeStep::OnError>(oe);
    if (s.onError == RecipeStep::OnError::Retry) {
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::InputInt("retries", &s.retryCount) && s.retryCount < 0)
            s.retryCount = 0;
    }
    ImGui::Separator();

    if (s.type == "loop") {
        const bool isWhile = s.params.contains("while");
        if (ImGui::RadioButton("count", !isWhile) && isWhile) {
            s.params.erase("while");
            s.params.erase("max_iterations");
            s.params["count"] = 1;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("while", isWhile) && !isWhile) {
            s.params.erase("count");
            s.params.erase("index_ctx");   // counts a count loop only
            s.params["while"] = nlohmann::json::object();
        }
        if (s.params.contains("count")) {
            int c = s.params["count"].is_number()
                        ? static_cast<int>(s.params["count"].get<double>()) : 1;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("count##loop", &c)) s.params["count"] = c < 1 ? 1 : c;
            // The pass number (1-based) into a ctx key, so a child can be
            // driven by it: move_to_cell's row_key / col_key. Empty = none.
            char ik[64];
            std::snprintf(ik, sizeof(ik), "%s",
                          s.params.value("index_ctx", std::string()).c_str());
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputText("index ctx##loop", ik, sizeof(ik))) {
                if (ik[0] == '\0') s.params.erase("index_ctx");
                else               s.params["index_ctx"] = std::string(ik);
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("ctx key that receives the pass number, 1..count\n"
                                  "(e.g. \"r\", read by move_to_cell's row_key)");
        } else if (s.params.contains("while")) {
            DrawConditionEditor(s.params["while"]);
            int mi = s.params.value("max_iterations", kDefaultMaxIterations);
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("max iterations", &mi))
                s.params["max_iterations"] = mi < 1 ? 1 : mi;
        }
    } else if (s.type == "if") {
        DrawConditionEditor(s.params);
        ImGui::TextDisabled("Palette 'else' mode adds else-branch steps.");
    } else if (s.type == "sub_recipe") {
        const std::string uid = s.params.value("uid", std::string());
        const Recipe* target = nullptr;
        for (const Recipe& rr : session_.Recipes()) {
            if (rr.uid == uid) { target = &rr; break; }
        }
        if (ImGui::BeginCombo("recipe", target ? target->name.c_str() : "(pick)")) {
            for (const Recipe& rr : session_.Recipes()) {
                // Obvious self-reference filtered; deeper cycles are caught by
                // InlineSubRecipes at run start.
                if (recipeSel_ >= 0 && rr.uid == session_.Recipes()[recipeSel_].uid) continue;
                if (ImGui::Selectable(rr.name.c_str(), target == &rr))
                    s.params["uid"] = rr.uid;
            }
            ImGui::EndCombo();
        }
    } else if (s.type == "stop_recipe") {
        bool ok = s.params.value("ok", true);
        if (ImGui::Checkbox("success", &ok)) s.params["ok"] = ok;
        char mbuf[128];
        std::snprintf(mbuf, sizeof(mbuf), "%s", s.params.value("message", std::string()).c_str());
        if (ImGui::InputText("message", mbuf, sizeof(mbuf)))
            s.params["message"] = std::string(mbuf);
    } else if (s.type == "group") {
        ImGui::TextDisabled("Group: organizational only.");
    } else if (s.type == "parallel") {
        ImGui::TextWrapped("Parallel: the children are dealt into one lane per device. "
                           "Lanes run at once; within a lane the moves run in the order "
                           "written. The block ends when every lane is done. Moves "
                           "only, no teach. A barrier between two sets is a second block.");
        ImGui::TextDisabled("A child's on_error acts within its lane; the block's on the block.");
    } else if (const CommandSpec* spec = orch_.registry.Find(s.type)) {
        // The validation error goes FIRST, under enabled / on error: at the
        // bottom of a long form it sat below the fold and got clipped at the
        // right edge. Wrapped so the whole reason is readable. Computed from
        // the params as they stand at the start of this frame - an edit below
        // shows up one frame later, which nobody can see.
        {
            const std::string err = ValidateParams(*spec, s.params, nullptr);
            if (!err.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.3f, 1.0f));
                ImGui::TextWrapped("%s", err.c_str());
                ImGui::PopStyleColor();
                ImGui::Separator();
            }
        }
        ImGui::TextWrapped("%s", spec->description.c_str());
        ImGui::Spacing();
        // detect_object: what its model looks for, and the view it was taught
        // at - read-only, outside the canEdit dimming so a run does not grey them.
        if (s.type == "detect_object") {
            ImGui::EndDisabled();
            DrawDetectStepThumbs(s);
            ImGui::BeginDisabled(!canEdit);
            ImGui::Spacing();
        }
        // A command with a helper window: its button first, big, because the
        // helper is where this step's numbers are meant to be set. Outside the
        // canEdit gate's reach on purpose - the helper is a view as well, and
        // disables its own fields while a run is active.
        if (!spec->helper.empty()) {
            ImGui::EndDisabled();
            const std::string lbl = "Open " + spec->helper + "...";
            if (ImGui::Button(lbl.c_str(), ImVec2(-1.0f, ImGui::GetFrameHeight() * 2.0f)))
                OpenCommandHelper(spec->helper);
            ImGui::BeginDisabled(!canEdit);
            ImGui::Spacing();
        }
        // One row per ParamSpec, in declaration order — except that a RUN of
        // consecutive params sharing a `group` folds under a header of that
        // name. A cluster that means nothing until one of it is set (the
        // approach triple on every move command) should not cost three
        // permanent rows on a form that is already long. Consecutive-run rather
        // than gather-by-name so the order on screen is still exactly the order
        // in the registration.
        const std::vector<ParamSpec>& ps = spec->params;
        for (std::size_t i = 0; i < ps.size();) {
            if (ps[i].group.empty()) {
                DrawParamWidget(*spec, ps[i], s.params);
                ++i;
                continue;
            }
            std::size_t j = i;
            // The group the command's helper window edits: a summary line, not
            // rows - those numbers mean something only against the helper's
            // picture. Hovering lists what they are set to now.
            if (ps[i].group == spec->helper) {
                while (j < ps.size() && ps[j].group == ps[i].group) ++j;
                ImGui::TextDisabled("%d settings in the %s", static_cast<int>(j - i),
                                    spec->helper.c_str());
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    std::string list;
                    for (std::size_t k = i; k < j; ++k) {
                        const nlohmann::json v = s.params.contains(ps[k].name)
                                                     ? s.params[ps[k].name]
                                                     : ps[k].defaultValue;
                        list += ps[k].name + " = " + v.dump() + "\n";
                    }
                    HelpTooltip(list);
                }
                // A REQUIRED setting the helper owns and nobody set yet: said in
                // red here, because the rows that would show it are folded into
                // the summary and validation will refuse the run over it.
                for (std::size_t k = i; k < j; ++k) {
                    if (!ps[k].required) continue;
                    const bool has = s.params.contains(ps[k].name) &&
                                     !s.params[ps[k].name].is_null() &&
                                     !(ps[k].type == ParamType::PointXyz &&
                                       s.params[ps[k].name].is_object() &&
                                       s.params[ps[k].name].empty());
                    if (!has)
                        ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s %s - open the %s",
                                           ps[k].name.c_str(),
                                           ps[k].type == ParamType::PointXyz ? "not taught"
                                                                             : "not set",
                                           spec->helper.c_str());
                }
                i = j;
                continue;
            }
            bool open = false;
            while (j < ps.size() && ps[j].group == ps[i].group) {
                // Open by itself when the operator has set one of these, or when
                // one is REQUIRED — a validation error must never hide behind a
                // fold. ImGuiCond_Once, so they can still close it.
                if (ps[j].required || s.params.contains(ps[j].name)) open = true;
                ++j;
            }
            if (open) ImGui::SetNextItemOpen(true, ImGuiCond_Once);
            if (ImGui::CollapsingHeader(ps[i].group.c_str())) {
                ImGui::Indent();
                for (std::size_t k = i; k < j; ++k) DrawParamWidget(*spec, ps[k], s.params);
                ImGui::Unindent();
            }
            i = j;
        }
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "unknown command '%s'",
                           s.type.c_str());
    }
    ImGui::EndDisabled();
}

// One schema-driven form row: reads params[p.name] (schema default when
// absent), writes back on edit. The widget is chosen by ParamType — the whole
// reason the typed registry exists. Returns true when the value changed.
bool App::DrawParamWidget(const CommandSpec& spec, const ParamSpec& p, nlohmann::json& params)
{
    bool changed = false;
    ImGui::PushID(p.name.c_str());
    nlohmann::json cur = params.contains(p.name) ? params[p.name] : p.defaultValue;
    const char* label = p.name.c_str();
    const float w = 170.0f;

    // A picker grows to fit its preview text — a position called "See Grip
    // Focus" was being shown as "See Grip Focu" — but never past what is left
    // of the row after its label and whatever buttons follow it (`trailing`
    // px), so the form keeps its single-column shape however long a name gets.
    // When the text still does not fit, hovering shows the full name above the
    // parameter's help (`previewTip`), and the generic help tooltip at the end
    // stands down so the two do not stack.
    bool clipped  = false;
    bool tipShown = false;
    const auto fitW = [&](const std::string& preview, float trailing = 0.0f) {
        const ImGuiStyle& st = ImGui::GetStyle();
        const float need = ImGui::CalcTextSize(preview.c_str()).x +
                           st.FramePadding.x * 2.0f + ImGui::GetFrameHeight();   // + arrow
        const float labelW = ImGui::CalcTextSize(label).x + st.ItemInnerSpacing.x;
        const float maxW   = ImGui::GetContentRegionAvail().x - labelW - trailing;
        const float fit    = std::max(w, std::min(need, maxW));
        clipped = need > fit + 0.5f;
        return fit;
    };
    const auto previewTip = [&](const std::string& preview) {
        if (!clipped || !ImGui::IsItemHovered()) return;
        HelpTooltip(p.help.empty() ? preview : preview + "\n\n" + p.help);
        tipShown = true;
    };
    // What the Go / Teach here pair after a position picker takes up.
    const auto smallButtonsW = [](std::initializer_list<const char*> labels) {
        const ImGuiStyle& st = ImGui::GetStyle();
        float total = 0.0f;
        for (const char* l : labels)
            total += ImGui::CalcTextSize(l).x + st.FramePadding.x * 2.0f + st.ItemSpacing.x;
        return total;
    };

    // Device-scoped reference params (position uid / map / pin) go stale when
    // the operator hot-swaps the step's device: the stored value no longer
    // resolves for the new device, so the combo shows a bare uid (or a name
    // the device doesn't have). Render that loud — red text on a yellow frame,
    // plus a hint line — until the operator re-picks. Push before BeginCombo,
    // pop right after it so the dropdown's own items render normally.
    constexpr ImVec4 kStaleRed{0.85f, 0.08f, 0.08f, 1.0f};
    const auto pushStale = [&kStaleRed] {
        ImGui::PushStyleColor(ImGuiCol_Text,           kStaleRed);
        ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0.95f, 0.80f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(1.00f, 0.88f, 0.30f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(1.00f, 0.92f, 0.45f, 1.0f));
    };

    switch (p.type) {
    case ParamType::Bool: {
        if (p.required) {
            // A checkbox has only two pixels for THREE states: on, off, and
            // never-answered. On a required bool the last two look identical
            // while validation refuses the step ("missing required parameter"),
            // so the operator stares at an unticked box that already says OFF.
            // Two radio buttons make the answer explicit: neither is lit until
            // a pick is made, and either pick STORES the value.
            const bool has = params.contains(p.name) && params[p.name].is_boolean();
            const bool v   = has && params[p.name].get<bool>();
            if (ImGui::RadioButton("ON", has && v)) { params[p.name] = true;  changed = true; }
            ImGui::SameLine();
            if (ImGui::RadioButton("OFF", has && !v)) { params[p.name] = false; changed = true; }
            ImGui::SameLine();
            ImGui::TextUnformatted(label);
        } else {
            bool v = cur.is_boolean() && cur.get<bool>();
            if (ImGui::Checkbox(label, &v)) { params[p.name] = v; changed = true; }
        }
        break;
    }
    case ParamType::Int: {
        int v = cur.is_number() ? static_cast<int>(cur.get<double>()) : 0;
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputInt(label, &v)) {
            if (p.minVal < p.maxVal) {
                if (v < p.minVal) v = static_cast<int>(p.minVal);
                if (v > p.maxVal) v = static_cast<int>(p.maxVal);
            }
            params[p.name] = v;
            changed = true;
        }
        // A plain Int that counts MILLISECONDS (auto_ok_ms, timeout_ms,
        // pre_delay_ms...): say how long that is on the row below, so 3000
        // reads as 3 s at a glance. Display only - the step stores ms.
        const std::size_t nl = p.name.size();
        if (nl > 3 && p.name.compare(nl - 3, 3, "_ms") == 0) {
            if (v <= 0 && p.name == "auto_ok_ms") {
                ImGui::TextDisabled("= off - waits for the operator");
            } else if (v < 60000) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.3f", static_cast<double>(v) / 1000.0);
                std::string sec = buf;               // 0.500 -> 0.5, 3.000 -> 3
                while (!sec.empty() && sec.back() == '0') sec.pop_back();
                if (!sec.empty() && sec.back() == '.') sec.pop_back();
                ImGui::TextDisabled("= %s sec", sec.c_str());
            } else {
                ImGui::TextDisabled("= %s", FormatDurationMs(v).c_str());
            }
        }
        break;
    }
    case ParamType::DurationMs: {
        // Integer MILLISECONDS on the wire, always; the unit is only how it is
        // typed. The choice is one editor-wide setting (edDurationSec_, kept in
        // UiSettings), and the readout says the length in h / m / s whichever
        // unit is typed.
        const ImGuiStyle& st = ImGui::GetStyle();
        long long ms = cur.is_number() ? std::llround(cur.get<double>()) : 0;
        const float unitW = ImGui::CalcTextSize("ms").x + st.FramePadding.x * 2.0f +
                            ImGui::GetFrameHeight();
        bool edited = false;
        ImGui::SetNextItemWidth(w - unitW - st.ItemInnerSpacing.x);
        if (edDurationSec_) {
            double sec = static_cast<double>(ms) / 1000.0;
            if (ImGui::InputDouble("##dur", &sec, 1.0, 10.0, "%.3f")) {
                ms     = std::llround(sec * 1000.0);
                edited = true;
            }
        } else {
            int v = static_cast<int>(ms);
            if (ImGui::InputInt("##dur", &v, 100, 1000)) {
                ms     = v;
                edited = true;
            }
        }
        if (edited) {
            if (p.minVal < p.maxVal)
                ms = std::clamp(ms, static_cast<long long>(p.minVal),
                                static_cast<long long>(p.maxVal));
            params[p.name] = ms;
            changed = true;
        }
        ImGui::SameLine(0.0f, st.ItemInnerSpacing.x);
        ImGui::SetNextItemWidth(unitW);
        if (ImGui::BeginCombo("##unit", edDurationSec_ ? "s" : "ms")) {
            if (ImGui::Selectable("ms", !edDurationSec_)) { edDurationSec_ = false; SaveUiSettings(); }
            if (ImGui::Selectable("s", edDurationSec_))   { edDurationSec_ = true;  SaveUiSettings(); }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Type durations in ms or in s. The step always stores ms.");
        // The param's name, unless it IS the unit ("ms") - the combo says that.
        if (p.name != "ms") {
            ImGui::SameLine();
            ImGui::TextUnformatted(label);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("= %s", FormatDurationMs(ms).c_str());
        break;
    }
    case ParamType::IntChoice: {
        // `options[i]` names the value minVal + i. A value outside the list is
        // shown as the bare number rather than snapped to a neighbour: it came
        // from somewhere (a hand-edited recipe, a spec that has since grown a
        // choice), and silently rewriting it would hide that.
        const int lo = static_cast<int>(p.minVal);
        const int v  = cur.is_number() ? static_cast<int>(cur.get<double>()) : lo;
        const int idx = v - lo;
        const int  n     = static_cast<int>(p.options.size());
        const bool known = idx >= 0 && idx < n;
        char fallback[32];
        std::snprintf(fallback, sizeof(fallback), "%d", v);
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, known ? p.options[idx].c_str() : fallback)) {
            for (int i = 0; i < n; ++i) {
                const int val = lo + i;
                if (ImGui::Selectable(p.options[i].c_str(), val == v)) {
                    params[p.name] = val;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::Double: {
        double v = cur.is_number() ? cur.get<double>() : 0.0;
        // A quantity with a unit (ParamSpec::siUnit): mantissa + prefix + unit,
        // stored in the base unit. The range check is the same.
        if (!p.siUnit.empty()) {
            if (InputSiDouble(label, &v, 90.0f, p.siUnit.c_str())) {
                if (p.minVal < p.maxVal) {
                    if (v < p.minVal) v = p.minVal;
                    if (v > p.maxVal) v = p.maxVal;
                }
                params[p.name] = v;
                changed = true;
            }
            break;
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputDouble(label, &v, 0.0, 0.0, "%g")) {
            if (p.minVal < p.maxVal) {
                if (v < p.minVal) v = p.minVal;
                if (v > p.maxVal) v = p.maxVal;
            }
            params[p.name] = v;
            changed = true;
        }
        // A relative move's delta gets Tune: nudge the stored value by a small
        // amount (0.459 + 0.012 = 0.471) instead of retyping it. The amount is
        // one editor-wide setting; the step stores only the result.
        const bool tunable =
            (spec.name == "move_relative" && p.name == "delta_mm") ||
            (spec.name == "move_relative_global" &&
             (p.name == "dx_mm" || p.name == "dy_mm" || p.name == "dz_mm"));
        if (tunable) {
            // The row's last item is no longer the field, so its help is shown
            // here and the generic tooltip at the end stands down.
            if (ImGui::IsItemHovered()) HelpTooltip(p.help);
            tipShown = true;
            ImGui::SameLine();
            double now = params.contains(p.name) && params[p.name].is_number()
                             ? params[p.name].get<double>()
                             : v;
            bool tuneEdited = false;
            if (TuneButton(p.name.c_str(), now, edTuneMm_, tuneEdited, p.minVal, p.maxVal)) {
                params[p.name] = now;
                changed = true;
            }
            if (tuneEdited) SaveUiSettings();
        }
        // `z_mm` is an ABSOLUTE Z in the device's own axis space on every step
        // that has one (move_to_nominal, move_local, move_axis_to), so it gets
        // Teach here: read the Z the device is standing at now. The device is
        // the step's `device` param, else its alignment's device - the same
        // resolution the commands use. Clear unsets an optional value, which
        // for move_to_nominal means "back to the frame's Z"; a stored 0 is not
        // the same thing as no value, and the box alone cannot show which.
        if (p.name == "z_mm") {
            std::string dev = EffectiveString(spec, params, "device");
            if (dev.empty())
                if (const AlignProc* ap = FindAlignProc(session_.Aligns(),
                                                        EffectiveString(spec, params, "alignment")))
                    dev = ap->device;
            IMotionController* mc = nullptr;
            for (std::size_t i = 0; i < hardware_.MotionCount(); ++i)
                if (hardware_.MotionName(i) == dev) { mc = hardware_.MotionDevice(i); break; }
            double z  = 0.0;
            bool   hz = false;
            if (mc && mc->IsConnected()) {
                // A demand signal while the form is on screen, so the cache is
                // fresh by the time the button is pressed (the driver idles at 1 Hz).
                mc->RequestFastPoll();
                const std::map<std::string, double> pos = mc->Positions();
                const auto it = pos.find("Z");
                if (it != pos.end()) { z = it->second; hz = true; }
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(!hz);
            if (ImGui::SmallButton("Teach here")) {
                params[p.name] = z;
                changed = true;
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip(hz ? "Set z_mm to %s's current Z: %.4f (absolute, axis space)"
                                     : "%s: not connected, or reports no Z",
                                  dev.empty() ? "(no device)" : dev.c_str(), z);
            if (!p.required) {
                ImGui::SameLine();
                ImGui::BeginDisabled(!params.contains(p.name));
                if (ImGui::SmallButton("Clear")) {
                    params.erase(p.name);
                    changed = true;
                }
                ImGui::EndDisabled();
                if (!params.contains(p.name))
                    ImGui::TextDisabled("   z_mm not set - %s",
                                        params.contains("nominal")
                                            ? "Z from the frame (z_from_frame)"
                                            : "Z is left where it is");
            }
        }
        break;
    }
    case ParamType::String: {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s",
                      cur.is_string() ? cur.get<std::string>().c_str() : "");
        if (ImGui::InputText(label, buf, sizeof(buf))) {
            params[p.name] = std::string(buf);
            changed = true;
        }
        break;
    }
    case ParamType::Value: {
        int t = cur.is_boolean() ? 0 : cur.is_number() ? 1 : 2;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::Combo("##vtype", &t, "bool\0number\0string\0")) {
            params[p.name] = t == 0   ? nlohmann::json(false)
                             : t == 1 ? nlohmann::json(0.0)
                                      : nlohmann::json("");
            cur     = params[p.name];
            changed = true;
        }
        ImGui::SameLine();
        if (t == 0) {
            bool v = cur.is_boolean() && cur.get<bool>();
            if (ImGui::Checkbox(label, &v)) { params[p.name] = v; changed = true; }
        } else if (t == 1) {
            double v = cur.is_number() ? cur.get<double>() : 0.0;
            if (InputSiDouble(label, &v, 90.0f)) {
                params[p.name] = v;
                changed = true;
            }
        } else {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%s",
                          cur.is_string() ? cur.get<std::string>().c_str() : "");
            if (ImGui::InputText(label, buf, sizeof(buf))) {
                params[p.name] = std::string(buf);
                changed = true;
            }
        }
        break;
    }
    case ParamType::Enum: {
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        ImGui::SetNextItemWidth(fitW(v));
        const bool open = ImGui::BeginCombo(label, v.c_str());
        previewTip(v);
        if (open) {
            for (const std::string& o : p.options) {
                if (ImGui::Selectable(o.c_str(), o == v)) {
                    params[p.name] = o;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::DeviceMotion:
    case ParamType::DeviceHexapod:
    case ParamType::DeviceIo:
    case ParamType::DevicePsu:
    case ParamType::LightName:
    case ParamType::Slide: {
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        // An empty PSU is a real choice, not an unset one: the first supply.
        // So is an empty light: every configured channel.
        const std::string preview =
            !v.empty() ? v
            : p.type == ParamType::DevicePsu
                ? (hardware_.PsuCount() > 0 ? "(first: " + hardware_.PsuName(0) + ")"
                                            : std::string("(first PSU)"))
            : p.type == ParamType::LightName ? std::string("(every light)")
                                             : std::string("(pick)");
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        previewTip(preview);
        if (open) {
            const auto item = [&](const std::string& name) {
                if (ImGui::Selectable(name.c_str(), name == v)) {
                    params[p.name] = name;
                    changed = true;
                }
            };
            if (p.type == ParamType::DeviceMotion ||
                p.type == ParamType::DeviceHexapod) {
                for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
                    if (p.type == ParamType::DeviceHexapod &&
                        hardware_.MotionDevice(i)->Type() != "PI")
                        continue;   // scan commands are PI-only
                    item(hardware_.MotionName(i));
                }
            } else if (p.type == ParamType::DeviceIo) {
                for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
                    item(hardware_.IoName(i));
            } else if (p.type == ParamType::DevicePsu) {
                for (std::size_t i = 0; i < hardware_.PsuCount(); ++i)
                    item(hardware_.PsuName(i));
            } else if (p.type == ParamType::LightName) {
                // The same list the Light page and the call-out pickers offer.
                if (ImGui::Selectable("(every light)", v.empty())) {
                    params[p.name] = std::string();
                    changed = true;
                }
                const std::vector<std::string> lights = workflow_.LightNames();
                for (const std::string& n : lights) item(n);
                if (lights.empty()) ImGui::TextDisabled("no lights configured in hardware.json");
            } else {
                for (std::size_t i = 0; i < hardware_.SlideCount(); ++i)
                    if (hardware_.Slide(i)) item(hardware_.Slide(i)->Name());
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::PositionUid: {
        // The device the step will RUN on: a schema default (dispense_run's
        // "gantry-main") is never stored, and reading params alone left this
        // picker empty and Go / Teach here disabled.
        const std::string dev = EffectiveString(spec, params, "device");
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        // A step that can also carry its pose INLINE (a PointXyz sibling, e.g.
        // dispense_run's "point") holds one answer to where, and this picker
        // is the CHOICE: its empty entry is "custom point" (the point row below
        // appears and its Teach here stores the pose in the step), a saved
        // position hides the point row and clears it. With custom chosen the
        // Go / Teach here pair stays off this row - the point row has its own.
        const bool pointSibling =
            std::any_of(spec.params.begin(), spec.params.end(),
                        [](const ParamSpec& q) { return q.type == ParamType::PointXyz; });
        const char* const emptyName = pointSibling ? "(custom point - taught in this step)"
                                                   : "(none)";
        std::string preview   = v.empty() ? (pointSibling ? emptyName : "(pick)") : v;
        bool resolved         = v.empty();   // empty = nothing to warn about yet
        const auto it = session_.Positions().find(dev);
        if (it != session_.Positions().end()) {
            for (const SavedPos& sp : it->second)
                if (sp.uid == v) { preview = sp.name; resolved = true; break; }
        }
        const bool showPair = !(pointSibling && v.empty());
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(
            fitW(preview, showPair ? smallButtonsW({"Go", "Teach here"}) : 0.0f));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (!p.required && ImGui::Selectable(emptyName, v.empty())) {
                params.erase(p.name);
                changed = true;
            }
            if (it == session_.Positions().end() || it->second.empty())
                ImGui::TextDisabled("(no saved positions for '%s')", dev.c_str());
            else
                for (const SavedPos& sp : it->second) {
                    // System/private rows (e.g. an alignment procedure's own
                    // station positions) are not part of the recipe vocabulary.
                    // An existing reference still RESOLVES above — only the
                    // offer is withheld.
                    if (sp.hidden) continue;
                    if (ImGui::Selectable(sp.name, sp.uid == v)) {
                        params[p.name] = sp.uid;   // stores the uid, shows the name
                        for (const ParamSpec& q : spec.params)
                            if (q.type == ParamType::PointXyz) params.erase(q.name);
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }

        // Go / Teach here, next to the picker. A saved position is a taught
        // pose, and the step that uses it is where an operator notices it has
        // drifted - sending them to the Motion page to fix it is how a recipe
        // ends up pointing at a position nobody dared re-teach.
        //
        // Teaching writes the position IN PLACE: same uid, so every other
        // reference still resolves. That is also exactly why it asks first -
        // those other references now point somewhere else too, and the
        // confirmation is the only place that fact is visible.
        //
        // Both buttons inherit the inspector's BeginDisabled(!canEdit), which
        // is `!orch_.Busy()` - a Go from here during a run would be a second
        // hand on the machine.
        IMotionController* mc = nullptr;
        int devIdx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) != dev) continue;
            devIdx = static_cast<int>(i);
            mc     = hardware_.MotionDevice(i);
            break;
        }
        const bool canTeach = mc && mc->IsConnected() && resolved && !v.empty();
        bool untaught = false;   // a placeholder: Teach here, never Go
        if (!v.empty() && it != session_.Positions().end())
            for (const SavedPos& sp : it->second)
                if (sp.uid == v) { untaught = sp.untaught; break; }
        if (showPair) {
            ImGui::SameLine();
            ImGui::BeginDisabled(!canTeach || untaught);
            if (ImGui::SmallButton("Go")) {
                std::vector<std::pair<std::string, double>> tgt;
                // it->second, not Positions()[dev]: operator[] would INSERT an
                // empty device list for a name that has none. canTeach already
                // required the lookup above to have found this position.
                for (const SavedPos& sp : it->second) {
                    if (sp.uid != v) continue;
                    for (const std::string& ax : mc->Axes()) {
                        const auto av = sp.vals.find(ax);
                        if (av != sp.vals.end()) tgt.emplace_back(ax, av->second);
                    }
                    break;
                }
                if (!workflow_.MoveToPosition(static_cast<std::size_t>(devIdx), tgt))
                    FD_WARN("saved", "move %s -> '%s' failed", dev.c_str(), preview.c_str());
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(!canTeach);
            if (ImGui::SmallButton("Teach here")) {
                // Handoff only. The pose is read when the dialog is confirmed, not
                // now: the driver idles at 1 Hz and the dialog is what asks it to
                // poll fast.
                teachPosDev_  = dev;
                teachPosUid_  = v;
                openTeachPos_ = true;
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !canTeach)
                ImGui::SetTooltip("%s", !mc                 ? "the step names no motion device"
                                        : !mc->IsConnected() ? "device not connected"
                                                             : "pick a position first");
        }

        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a position of '%s' - re-pick", dev.c_str());
        else if (untaught)
            ImGui::TextColored(kTeachOrange, "not taught on this machine - jog there and Teach here");
        break;
    }
    case ParamType::PointXyz: {
        // A camera pose taught INTO the step: no saved position to create,
        // name or pick, and nothing shared with another step - so Teach here
        // writes straight into the step with no confirmation (the PositionUid
        // row asks because its write moves every other reference too). Axis
        // space of the step's device, read as the live X/Y/Z - exactly what
        // move_axis_to commands back.
        //
        // The buttons inherit the inspector's BeginDisabled(!canEdit), like the
        // position row's.
        //
        // Shown only when the position picker above says "custom point": with
        // a saved position chosen the step has no point (the pick cleared it),
        // and a Teach here here would silently switch the step back.
        const bool savedChosen =
            std::any_of(spec.params.begin(), spec.params.end(), [&](const ParamSpec& q) {
                return q.type == ParamType::PositionUid && params.contains(q.name) &&
                       params[q.name].is_string() && !params[q.name].get<std::string>().empty();
            });
        if (savedChosen) {
            tipShown = true;   // drew nothing: the previous row's item is not ours
            break;
        }
        const std::string dev = EffectiveString(spec, params, "device");
        IMotionController* mc = nullptr;
        int devIdx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) != dev) continue;
            devIdx = static_cast<int>(i);
            mc     = hardware_.MotionDevice(i);
            break;
        }
        static const char* const kXyz[3] = {"X", "Y", "Z"};
        double pt[3]   = {0.0, 0.0, 0.0};
        double live[3] = {0.0, 0.0, 0.0};
        bool taught = cur.is_object();
        for (int a = 0; a < 3 && taught; ++a) {
            taught = cur.contains(kXyz[a]) && cur[kXyz[a]].is_number();
            if (taught) pt[a] = cur[kXyz[a]].get<double>();
        }
        bool hasLive = false;
        if (mc && mc->IsConnected()) {
            // A demand signal while the form is on screen, so the cache is
            // fresh by the time the button is pressed (the driver idles at 1 Hz).
            mc->RequestFastPoll();
            const std::map<std::string, double> pos = mc->Positions();
            hasLive = true;
            for (int a = 0; a < 3 && hasLive; ++a) {
                const auto f = pos.find(kXyz[a]);
                hasLive = f != pos.end();
                if (hasLive) live[a] = f->second;
            }
        }

        if (taught)
            ImGui::Text("X %.4f  Y %.4f  Z %.4f", pt[0], pt[1], pt[2]);
        else
            ImGui::TextDisabled("(not taught)");
        ImGui::SameLine();
        ImGui::TextUnformatted(label);

        ImGui::SameLine();
        ImGui::BeginDisabled(!taught || !mc || !mc->IsConnected());
        if (ImGui::SmallButton("Go")) {
            if (!workflow_.MoveToPosition(static_cast<std::size_t>(devIdx),
                                          {{"X", pt[0]}, {"Y", pt[1]}, {"Z", pt[2]}}))
                FD_WARN("saved", "move %s -> taught %s failed", dev.c_str(), label);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!hasLive);
        if (ImGui::SmallButton("Teach here")) {
            params[p.name] = {{"X", live[0]}, {"Y", live[1]}, {"Z", live[2]}};
            for (const ParamSpec& q : spec.params)
                if (q.type == ParamType::PositionUid) params.erase(q.name);
            changed = true;
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            if (hasLive)
                ImGui::SetTooltip("Store %s's current pose in this step:\n"
                                  "X %.4f  Y %.4f  Z %.4f (axis space)",
                                  dev.c_str(), live[0], live[1], live[2]);
            else
                ImGui::SetTooltip("%s: %s", dev.empty() ? "(no device)" : dev.c_str(),
                                  !mc                  ? "not a motion device"
                                  : !mc->IsConnected() ? "not connected"
                                                       : "reports no X/Y/Z");
            tipShown = true;
        }
        if (!p.required) {
            ImGui::SameLine();
            ImGui::BeginDisabled(!taught);
            if (ImGui::SmallButton("Clear")) {
                params.erase(p.name);
                changed = true;
            }
            ImGui::EndDisabled();
        }
        break;
    }
    case ParamType::AlignUid: {
        // NOT device-scoped (unlike PositionUid): a procedure carries its own
        // device. Stores the uid, shows the name.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        bool resolved       = v.empty();
        if (const AlignProc* ap = FindAlignProc(session_.Aligns(), v)) {
            preview  = ap->name;
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (session_.Aligns().empty())
                ImGui::TextDisabled("(no alignment procedures - see the Alignment page)");
            else
                for (const AlignProc& ap : session_.Aligns()) {
                    if (ImGui::Selectable(ap.name.c_str(), ap.uid == v)) {
                        params[p.name] = ap.uid;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "unknown alignment procedure - re-pick");
        break;
    }
    case ParamType::GripUid: {
        // Not device-scoped, for a reason worth stating: the profile NAMES the
        // gripper it belongs to, so scoping the picker to a step's "device"
        // would be a second answer to which device is holding the object.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        bool resolved       = v.empty();
        if (const GripProfile* g = FindGripProfile(session_.Grips(), v)) {
            preview  = g->name;
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (session_.Grips().empty())
                ImGui::TextDisabled("(no grip profiles - see the Gripped Object page)");
            else
                for (const GripProfile& g : session_.Grips()) {
                    // The device it measures against, because two profiles for
                    // the same lens in two grippers are a normal thing to have
                    // and the names alone would not separate them.
                    std::string row = g.name;
                    if (!g.device.empty()) row += "   (" + g.device + ")";
                    if (ImGui::Selectable(row.c_str(), g.uid == v)) {
                        params[p.name] = g.uid;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "unknown grip profile - re-pick");
        break;
    }
    case ParamType::TrayUid: {
        // Not device-scoped either: the tray carries the device its corners
        // were taught on, so the step has no "device" of its own to scope by.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        bool resolved       = v.empty();
        if (const WaffleTray* t = FindWaffleTray(session_.Trays(), v)) {
            preview  = t->name;
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (session_.Trays().empty())
                ImGui::TextDisabled("(no trays - see the Waffle Tray page)");
            else
                for (const WaffleTray& t : session_.Trays()) {
                    char row[128];
                    std::snprintf(row, sizeof(row), "%s   (%s, %d x %d)", t.name.c_str(),
                                  t.device.empty() ? "no device" : t.device.c_str(),
                                  t.rows, t.cols);
                    if (ImGui::Selectable(row, t.uid == v)) {
                        params[p.name] = t.uid;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "unknown waffle tray - re-pick");
        break;
    }
    case ParamType::NominalUid: {
        // Scoped to the step's OWN "alignment", the way PositionUid is scoped to
        // its "device": a nominal is a point on one procedure's part, and the
        // list of every nominal in the product would be a list of points that
        // mostly cannot be placed from here.
        //
        // Picks store the uid, but an old recipe that holds a NAME still shows
        // as resolved — FindAlignNominal takes either, and re-typing a hand-
        // written recipe's params is not an upgrade the operator asked for.
        const std::string al = params.value("alignment", std::string());
        const AlignProc*  ap = FindAlignProc(session_.Aligns(), al);
        const std::string v  = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview  = v.empty() ? "(pick)" : v;
        // Optional and unset is a real setting here (place_mark: "empty = always
        // ask"), so an empty value is never stale.
        bool resolved = v.empty();
        if (ap) {
            if (const AlignNominal* nm = FindAlignNominal(*ap, v)) {
                preview  = nm->name;
                resolved = true;
            }
        } else {
            // No procedure picked yet: nothing to resolve AGAINST, so the value
            // is unjudged rather than wrong. The alignment row is already red.
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (!ap) {
                ImGui::TextDisabled("(pick the alignment procedure first)");
            } else if (ap->nominals.empty()) {
                ImGui::TextDisabled("(no nominals on '%s' - teach one from the "
                                    "Alignment page)", ap->name.c_str());
            } else {
                if (!p.required && ImGui::Selectable("(none)", v.empty())) {
                    params[p.name] = std::string();
                    changed = true;
                }
                for (const AlignNominal& nm : ap->nominals) {
                    // Match the name too: a recipe written by hand holds one.
                    const bool sel = nm.uid == v || nm.name == v;
                    if (ImGui::Selectable(nm.name.c_str(), sel)) {
                        params[p.name] = nm.uid;
                        changed = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a nominal of '%s' - re-pick",
                               ap->name.c_str());
        break;
    }
    case ParamType::VisionUid: {
        // Same shape as AlignUid: not device-scoped, stores the uid, shows the
        // name. The method rides along in the row because "circle" and "ncc"
        // fail for completely different reasons and the name rarely says which.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        bool resolved       = v.empty();
        if (const VisionModel* vm = FindVisionModel(session_.VisionModels(), v)) {
            preview  = vm->name;
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (session_.VisionModels().empty())
                ImGui::TextDisabled("(no detection models - teach one from an "
                                    "alignment station)");
            else
                for (const VisionModel& vm : session_.VisionModels()) {
                    const std::string row = vm.name + "  (" + vm.method + ")";
                    if (ImGui::Selectable(row.c_str(), vm.uid == v)) {
                        params[p.name] = vm.uid;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "unknown detection model - re-pick");
        break;
    }
    case ParamType::MapName: {
        const std::string dev = EffectiveString(spec, params, "device");
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const auto it = session_.Maps().find(dev);
        bool resolved = v.empty();
        if (!resolved && it != session_.Maps().end())
            for (const MotionMap& m : it->second)
                if (m.name == v) { resolved = true; break; }
        if (!resolved) pushStale();
        const std::string preview = v.empty() ? "(pick)" : v;
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (it == session_.Maps().end() || it->second.empty())
                ImGui::TextDisabled("(no maps for '%s')", dev.c_str());
            else
                for (const MotionMap& m : it->second) {
                    if (ImGui::Selectable(m.name.c_str(), m.name == v)) {
                        params[p.name] = m.name;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a map of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::PinName: {
        // "io_device" first: the needle commands' "device" is the gantry, so a
        // pin picker keyed on that would offer the axes of a motion controller.
        const bool hasIo = std::any_of(spec.params.begin(), spec.params.end(),
                                       [](const ParamSpec& q) { return q.name == "io_device"; });
        const std::string dev = EffectiveString(spec, params, hasIo ? "io_device" : "device");
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const IIoDevice* d = nullptr;
        for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == dev) { d = hardware_.IoDevice(i); break; }
        bool resolved = v.empty();
        if (!resolved && d) {
            const auto has = [&](bool inputs) {
                const int n = inputs ? d->InputCount() : d->OutputCount();
                for (int pin = 0; pin < n; ++pin)
                    if ((inputs ? d->InputPinName(pin) : d->OutputPinName(pin)) == v)
                        return true;
                return false;
            };
            resolved = (p.pinDir != PinDir::Output && has(true)) ||
                       (p.pinDir != PinDir::Input && has(false));
        }
        if (!resolved) pushStale();
        const std::string preview = v.empty() ? "(pick)" : v;
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (!d) {
                ImGui::TextDisabled("(pick a device first)");
            } else {
                const auto pins = [&](bool inputs) {
                    const int n = inputs ? d->InputCount() : d->OutputCount();
                    for (int pin = 0; pin < n; ++pin) {
                        const std::string name =
                            inputs ? d->InputPinName(pin) : d->OutputPinName(pin);
                        if (name.empty()) continue;
                        if (ImGui::Selectable(name.c_str(), name == v)) {
                            params[p.name] = name;
                            changed = true;
                        }
                    }
                };
                if (p.pinDir != PinDir::Output) pins(true);    // inputs
                if (p.pinDir != PinDir::Input)  pins(false);   // outputs
            }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a pin of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::TopicName: {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s",
                      cur.is_string() ? cur.get<std::string>().c_str() : "");
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputText(label, buf, sizeof(buf))) {
            params[p.name] = std::string(buf);
            changed = true;
        }
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (!topics.empty()) {
            ImGui::SameLine();
            if (ImGui::BeginCombo("##topics", "", ImGuiComboFlags_NoPreview)) {
                for (const std::string& t : topics) {
                    if (ImGui::Selectable(t.c_str())) {
                        params[p.name] = t;
                        changed = true;
                    }
                }
                ImGui::EndCombo();
            }
        }
        break;
    }
    case ParamType::ScriptFile: {
        // Picker over the same scanned list the Scripts page shows (product
        // scripts dir, else config/scripts), rescanned each time it opens.
        // Stores the path; shows the name.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        for (const ScriptInfo& si : session_.Scripts()) {
            if (si.path == v) { preview = si.name; break; }
        }
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        previewTip(preview);
        if (open) {
            // Rescan on open: a script dropped into the folder after the
            // product loaded must be pickable without visiting the Scripts page.
            if (ImGui::IsWindowAppearing()) LoadScriptsList();
            if (session_.Scripts().empty())
                ImGui::TextDisabled("(no scripts in %s)", session_.ScriptsDirUsed().c_str());
            for (const ScriptInfo& si : session_.Scripts()) {
                if (ImGui::Selectable(si.name.c_str(), si.path == v)) {
                    params[p.name] = si.path;
                    changed = true;
                }
                if (ImGui::IsItemHovered()) HelpTooltip(si.description);
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::ResultName: {
        // Only OBJECT RESULTS some step declares on THIS step's device. Across
        // the whole library, not this recipe alone: a result lives in
        // Workflow's store for the program, so the recipe that finds it is
        // often not the one that moves to it - and a recipe does not know which
        // program will run it. Each option says where it comes from; a name
        // whose only producer runs LATER in this same recipe is flagged, and
        // the run refuses a missing or wrong-device result again.
        const std::string dev = EffectiveString(spec, params, "device");
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        struct ResOpt { std::string name, from; bool enabled; int where; };   // where: 0 earlier here, 1 later here, 2 other recipe
        std::vector<ResOpt> opts;
        const int selR = recipeSel_;
        const auto& recipes = session_.Recipes();
        for (std::size_t ri = 0; ri < recipes.size(); ++ri) {
            const Recipe& r = recipes[ri];
            const bool here = static_cast<int>(ri) == selR;
            std::set<std::vector<std::size_t>> earlier;
            if (here && !edStepSel_.empty())
                for (const StepOutputs& so : CollectStepOutputs(r.steps, orch_.registry, &edStepSel_))
                    earlier.insert(so.path);
            for (const StepOutputs& so : CollectStepOutputs(r.steps, orch_.registry)) {
                for (const ResultOutput& ro : so.outs.results) {
                    if (ro.device != dev) continue;
                    const int where = !here ? 2 : (earlier.count(so.path) ? 0 : 1);
                    std::string from = where == 2 ? "recipe '" + r.name + "'"
                                     : where == 0 ? std::string("earlier here")
                                                  : std::string("LATER here");
                    from += ", step " + StepPathLabel(so.path) + " " + so.type + ", " + ro.axes;
                    if (!so.enabled) from += ", DISABLED";
                    opts.push_back({ro.name, from, so.enabled, where});
                }
            }
        }
        bool produced = false, onlyLater = !v.empty();
        for (const ResOpt& o : opts) {
            if (o.name != v || !o.enabled) continue;
            produced = true;
            if (o.where != 1) onlyLater = false;
        }
        const bool resolved = v.empty() || produced;
        // An OPTIONAL result param reads empty as "none" (the command says what
        // that means), and must be clearable back to it.
        const std::string preview = !v.empty()    ? v
                                  : p.required    ? std::string("(pick a result)")
                                                  : std::string("(none)");
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(fitW(preview));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        previewTip(preview);
        if (open) {
            if (!p.required && ImGui::Selectable("(none)", v.empty())) {
                params[p.name] = "";
                changed = true;
            }
            if (opts.empty())
                ImGui::TextDisabled("(no step declares an object result on '%s')", dev.c_str());
            for (std::size_t i = 0; i < opts.size(); ++i) {
                const ResOpt& o = opts[i];
                ImGui::PushID(static_cast<int>(i));
                const std::string row = o.name + "   (" + o.from + ")";
                if (ImGui::Selectable(row.c_str(), o.name == v)) {
                    params[p.name] = o.name;
                    changed = true;
                }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "no enabled step produces '%s' on '%s' - re-pick",
                               v.c_str(), dev.c_str());
        else if (produced && onlyLater)
            ImGui::TextColored(ImVec4(0.98f, 0.72f, 0.32f, 1.0f),
                               "'%s' is only produced LATER in this recipe - unless an earlier "
                               "recipe of the program finds it, this step will fail",
                               v.c_str());
        break;
    }
    case ParamType::ImageFile: {
        // Picker over the prompt-images folder (ProductSession::PromptImagesDir),
        // listed when the combo opens so a picture dropped in after the product
        // loaded is pickable at once. Stores the file NAME only - the folder
        // follows the product. A missing file is flagged here and drawn as
        // missing at run time, never refused: it is a visual aid.
        namespace fs = std::filesystem;
        const std::string dir = session_.PromptImagesDir();
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const std::string preview = v.empty() ? std::string("(none)") : v;
        const float folderW = ImGui::CalcTextSize("Folder").x +
                              ImGui::GetStyle().FramePadding.x * 2.0f +
                              ImGui::GetStyle().ItemSpacing.x;
        ImGui::SetNextItemWidth(fitW(preview, folderW));
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        previewTip(preview);
        if (open) {
            if (ImGui::IsWindowAppearing()) {
                promptImageList_.clear();
                std::error_code ec;
                for (const fs::directory_entry& e : fs::directory_iterator(dir, ec)) {
                    if (!e.is_regular_file(ec)) continue;
                    std::string ext = e.path().extension().string();
                    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp")
                        promptImageList_.push_back(e.path().filename().string());
                }
                std::sort(promptImageList_.begin(), promptImageList_.end());
            }
            if (ImGui::Selectable("(none)", v.empty())) {
                params.erase(p.name);
                changed = true;
            }
            if (promptImageList_.empty())
                ImGui::TextDisabled("(no .png/.jpg/.bmp in %s - use Folder)", dir.c_str());
            for (const std::string& f : promptImageList_) {
                if (ImGui::Selectable(f.c_str(), f == v)) {
                    params[p.name] = f;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Folder")) OpenFolderInExplorer(dir);   // creates it if missing
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Open %s - put the pictures here", dir.c_str());
            tipShown = true;
        }
        std::error_code ec;
        if (!v.empty() && !fs::is_regular_file(fs::path(dir) / v, ec))
            ImGui::TextColored(ImVec4(0.98f, 0.72f, 0.32f, 1.0f),
                               "not found in %s - the prompt will show it as missing",
                               dir.c_str());
        break;
    }
    }
    if (!tipShown && ImGui::IsItemHovered()) HelpTooltip(p.help);
    ImGui::PopID();
    return changed;
}

// ---- Numbers with an SI prefix ------------------------------------------------
// A photocurrent threshold is 0.0008 A; nobody should have to count the zeros.
// The field holds a MANTISSA and a combo beside it the prefix, so 800 + u is
// stored as 0.0008 - the plain number every condition and ctx value already
// is. The field also takes the prefix typed in: "800u", "0.8 mA", "8e-4". A
// unit after the prefix ("A", "V") is accepted and ignored: the number has no
// unit, the recipe author knows which key they are comparing.

namespace {

struct SiPrefix {
    int         exp;
    const char* sym;    // "" = none; "u" for micro (the font has no mu)
    const char* name;
};
constexpr SiPrefix kSiPrefixes[] = {
    {-15, "f", "femto"}, {-12, "p", "pico"}, {-9, "n", "nano"}, {-6, "u", "micro"},
    {-3, "m", "milli"},  {0, "", "none"},    {3, "k", "kilo"},  {6, "M", "mega"},
    {9, "G", "giga"},
};
constexpr int kNoSiPrefix = -1000;

double Pow10(int e) { return std::pow(10.0, static_cast<double>(e)); }

// The engineering exponent that puts |v| in [1, 1000).
int SiExpFor(double v)
{
    if (v == 0.0 || !std::isfinite(v)) return 0;
    const int e = static_cast<int>(std::floor(std::log10(std::fabs(v)) / 3.0)) * 3;
    return std::clamp(e, -15, 9);
}

const char* SiSym(int e)
{
    for (const SiPrefix& p : kSiPrefixes)
        if (p.exp == e) return p.sym;
    return "?";
}

// "800u" -> 800 and -6; "8e-4" -> 8e-4 and kNoSiPrefix; "0.8 mA" -> 0.8, -3.
// False on anything that is not a number, a prefix and an optional unit.
bool ParseSi(const char* s, double* mant, int* exp)
{
    char* end = nullptr;
    const double d = std::strtod(s, &end);
    if (end == s || !std::isfinite(d)) return false;
    const char* c = end;
    while (*c == ' ') ++c;
    int e = kNoSiPrefix;
    if (static_cast<unsigned char>(c[0]) == 0xC2 && static_cast<unsigned char>(c[1]) == 0xB5) {
        e = -6;   // a typed mu (UTF-8)
        c += 2;
    } else if (*c) {
        for (const SiPrefix& p : kSiPrefixes)
            if (*p.sym && *c == *p.sym) { e = p.exp; ++c; break; }
    }
    // Whatever follows is a unit: letters (and spaces) only.
    for (; *c; ++c)
        if (*c != ' ' && !std::isalpha(static_cast<unsigned char>(*c))) return false;
    *mant = d;
    *exp  = e;
    return true;
}

} // namespace

bool App::InputSiDouble(const char* label, double* v, float width, const char* unit)
{
    ImGui::PushID(label);
    const ImGuiID id = ImGui::GetID("##si");

    // The prefix shown: frozen while this field is being typed in, else the
    // author's pick when it still reads sensibly, else engineering notation.
    int exp = SiExpFor(*v);
    if (siEditId_ == id) {
        exp = siEditExp_;
    } else if (const auto it = siPrefixPick_.find(id); it != siPrefixPick_.end()) {
        const double m = std::fabs(*v) / Pow10(it->second);
        if (*v == 0.0 || (m >= 1e-3 && m < 1e6)) exp = it->second;
    }

    bool changed = false;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", *v / Pow10(exp));
    ImGui::SetNextItemWidth(width);
    if (ImGui::InputText("##si", buf, sizeof(buf))) {
        double mant = 0.0;
        int    pe   = kNoSiPrefix;
        if (ParseSi(buf, &mant, &pe)) {
            if (pe != kNoSiPrefix) {
                // A typed prefix wins and sticks, for the rest of this edit
                // and afterwards.
                *v = mant * Pow10(pe);
                siPrefixPick_[id] = pe;
                if (siEditId_ == id) siEditExp_ = pe;
            } else {
                *v = mant * Pow10(exp);
            }
            changed = true;
        }
    }
    if (ImGui::IsItemActivated()) {
        siEditId_  = id;
        siEditExp_ = exp;
    }
    if (ImGui::IsItemDeactivated() && siEditId_ == id) siEditId_ = 0;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("= %.9g\nType a number for the prefix beside it, or with its own\n"
                          "prefix: 800u, 0.8m, 0.8 mA, 8e-4 (all = 0.0008).",
                          *v);

    ImGui::SameLine(0.0f, 2.0f);
    ImGui::SetNextItemWidth(ImGui::CalcTextSize("mm").x + ImGui::GetFrameHeight() +
                            ImGui::GetStyle().FramePadding.x * 2.0f);
    const char* cur = SiSym(exp);
    if (ImGui::BeginCombo("##sip", *cur ? cur : "-")) {
        for (const SiPrefix& p : kSiPrefixes) {
            char row[32];
            std::snprintf(row, sizeof(row), "%-2s %s", *p.sym ? p.sym : "-", p.name);
            if (ImGui::Selectable(row, p.exp == exp) && p.exp != exp) {
                // The digits stay, the prefix changes: 800 u -> 800 m.
                *v = (*v / Pow10(exp)) * Pow10(p.exp);
                siPrefixPick_[id] = p.exp;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("SI prefix of the number on the left");
    if (unit && *unit) {   // the base unit right after its prefix: "m" "A"
        ImGui::SameLine(0.0f, 4.0f);
        ImGui::TextUnformatted(unit);
    }

    // The visible part of the label, as ImGui would have drawn it.
    const char* hash = std::strstr(label, "##");
    const std::string shown = hash ? std::string(label, hash) : std::string(label);
    if (!shown.empty()) {
        ImGui::SameLine();
        ImGui::TextUnformatted(shown.c_str());
    }
    ImGui::PopID();
    return changed;
}

// Shared editor for {key|topic, op, value} condition objects (an if step's
// params, a loop's "while"). Normalizes the object in place so a fresh
// condition is immediately valid.
bool App::DrawConditionEditor(nlohmann::json& cond)
{
    bool changed = false;
    if (!cond.is_object()) { cond = nlohmann::json::object(); changed = true; }
    if (!cond.contains("op")) { cond["op"] = "is_true"; changed = true; }
    if (!cond.contains("key") && !cond.contains("topic")) {
        cond["key"] = "";
        changed = true;
    }

    int src = cond.contains("topic") ? 1 : 0;
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::Combo("source", &src, "ctx key\0topic\0")) {
        const std::string old = src == 1 ? cond.value("key", std::string())
                                         : cond.value("topic", std::string());
        cond.erase("key");
        cond.erase("topic");
        cond[src == 1 ? "topic" : "key"] = old;
        changed = true;
    }
    const char* fk = cond.contains("topic") ? "topic" : "key";
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s", cond.value(fk, std::string()).c_str());
    if (ImGui::InputText(fk, buf, sizeof(buf))) {
        cond[fk] = std::string(buf);
        changed = true;
    }

    static const char* kOps[] = {">", "<", ">=", "<=", "==", "!=",
                                 "is_true", "is_false", "exists", "not_exists"};
    const std::string op = cond.value("op", std::string("is_true"));
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::BeginCombo("op", op.c_str())) {
        for (const char* o : kOps) {
            if (ImGui::Selectable(o, op == o)) {
                cond["op"] = o;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    const bool needsValue = op != "is_true" && op != "is_false" &&
                            op != "exists" && op != "not_exists";
    if (needsValue) {
        nlohmann::json v = cond.contains("value") ? cond["value"] : nlohmann::json(0.0);
        int t = v.is_string() ? 1 : 0;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::Combo("##ctype", &t, "number\0string\0")) {
            cond["value"] = t == 1 ? nlohmann::json("") : nlohmann::json(0.0);
            v             = cond["value"];
            changed       = true;
        }
        ImGui::SameLine();
        if (t == 1) {
            char vb[128];
            std::snprintf(vb, sizeof(vb), "%s",
                          v.is_string() ? v.get<std::string>().c_str() : "");
            if (ImGui::InputText("value", vb, sizeof(vb))) {
                cond["value"] = std::string(vb);
                changed = true;
            }
        } else {
            double d = v.is_number() ? v.get<double>() : 0.0;
            if (InputSiDouble("value", &d, 90.0f)) {
                cond["value"] = d;
                changed = true;
            }
        }
    }
    return changed;
}

} // namespace firedog
