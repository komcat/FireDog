// WndInterlocks.cpp : the floating Interlocks window (View menu) - READ ONLY
// live rule states, the last refusal and the trip - plus the Admin bypass, the
// trip modal and the menu-bar badges. Definitions of App members - see App.h.
// Design: docs/interlock-design.md.
//
// Nothing here edits a rule: that is the Interlock Editor page (Admin). This
// window answers the troubleshooting question "why will it not move?", so it
// is Technician (kRoleInterlockView). The bypass is the one control that
// changes anything, and it is Admin (kRoleInterlockBypass).
#include "App.h"

#include "imgui.h"

#include "core/Interlock.h"

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kIlRed(0.95f, 0.35f, 0.30f, 1.0f);
const ImVec4 kIlAmber(1.00f, 0.72f, 0.25f, 1.0f);
const ImVec4 kIlGreen(0.45f, 0.85f, 0.50f, 1.0f);
const ImVec4 kIlGrey(0.55f, 0.55f, 0.55f, 1.0f);

// A filled dot in the text flow: red = active, amber = active only because
// something could not be read (fail closed), green = inactive, grey = disabled.
void StateDot(const ImVec4& c)
{
    const float  h = ImGui::GetTextLineHeight();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + h * 0.5f, p.y + h * 0.5f),
                                                h * 0.32f, ImGui::GetColorU32(c));
    ImGui::Dummy(ImVec2(h, h));
}

ImVec4 RuleColor(const IlRuleStatus& st)
{
    if (!st.enabled) return kIlGrey;
    if (!st.active) return kIlGreen;
    return st.unknown ? kIlAmber : kIlRed;
}

} // namespace

void App::DrawInterlockMenuBadges()
{
    if (workflow_.InterlockTripActive())
        ImGui::TextColored(kIlRed, "  !! INTERLOCK TRIP !!");

    const int left = workflow_.InterlockBypassSecondsLeft();
    if (left > 0) {
        ImGui::TextColored(kIlRed, "  INTERLOCKS BYPASSED %d:%02d - by %s", left / 60, left % 60,
                           workflow_.InterlockBypassBy().c_str());
        if (CanUse(kRoleInterlockBypass)) {
            ImGui::SameLine();
            if (ImGui::SmallButton("End bypass")) workflow_.ClearInterlockBypass("ended by hand");
        }
    }

    // The last refusal, for five seconds: a Go button the gate refused would
    // otherwise look exactly like a button that did nothing.
    const Workflow::InterlockEvent ev = workflow_.LastInterlockRefusal();
    if (ev.seq == 0) return;
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - ev.at).count();
    if (age > 5.0) return;
    const float a = static_cast<float>(age > 4.0 ? 5.0 - age : 1.0);
    ImGui::TextColored(ImVec4(kIlAmber.x, kIlAmber.y, kIlAmber.z, a), "  Interlock: [%s] refused",
                       ev.rule.c_str());
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", ev.text.c_str());
}

void App::DrawInterlockTripModal()
{
    // The watch already stopped the device and the engines abort on the trip;
    // this only says what happened and takes the acknowledgement. Same shape
    // and reasoning as DrawSafetyAlertModal.
    if (workflow_.InterlockTripActive() && !ImGui::IsPopupOpen("Interlock Trip"))
        ImGui::OpenPopup("Interlock Trip");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Interlock Trip", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const Workflow::InterlockTrip t = workflow_.InterlockTripState();
        ImGui::TextColored(kIlRed, "Interlock [%s]", t.rule.c_str());
        ImGui::Separator();
        ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 460.0f);
        ImGui::Text("%s was stopped: axis %s was moving while the rule became active.",
                    t.device.c_str(), t.axis == "*" ? "(all - reference / buffer)" : t.axis.c_str());
        ImGui::Spacing();
        ImGui::TextUnformatted(("Because: " + t.reason).c_str());
        ImGui::Spacing();
        ImGui::TextColored(kIlRed, "Any run was aborted. New moves are blocked until acknowledged; "
                                   "after that the rule still refuses what it blocks.");
        ImGui::PopTextWrapPos();
        ImGui::Spacing();
        if (ImGui::Button("Acknowledge", ImVec2(160.0f, 0.0f))) {
            workflow_.AcknowledgeInterlockTrip();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void App::DrawInterlocksWindow()
{
    if (!CanUse(kRoleInterlockView)) { showInterlocks_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(720.0f, 460.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Interlocks", &showInterlocks_)) {
        ImGui::End();
        return;
    }

    const IlConfig cfg = workflow_.Interlocks();
    if (!cfg.loadError.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(kIlRed, "The interlock file cannot be read - EVERY move is refused: %s",
                           cfg.loadError.c_str());
        ImGui::PopTextWrapPos();
    } else if (!cfg.enabled) {
        ImGui::TextColored(kIlAmber, "Interlocks are switched OFF in the rule set.");
    } else {
        ImGui::Text("%zu rule(s), watch every %d ms", cfg.rules.size(), cfg.watchMs);
    }

    // ---- Bypass (Admin) ----
    const int left = workflow_.InterlockBypassSecondsLeft();
    if (left > 0) {
        ImGui::TextColored(kIlRed, "BYPASSED - %d:%02d left, by %s. The safety alert still applies.",
                           left / 60, left % 60, workflow_.InterlockBypassBy().c_str());
    }
    if (CanUse(kRoleInterlockBypass)) {
        if (left > 0) {
            if (ImGui::Button("End bypass")) workflow_.ClearInterlockBypass("ended by hand");
        } else {
            ImGui::SetNextItemWidth(90.0f);
            ImGui::SliderInt("min##ilbyp", &ilBypassMin_, 1, 30);
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.18f, 0.15f, 1.0f));
            if (ImGui::Button("Bypass interlocks"))
                workflow_.SetInterlockBypass(ilBypassMin_, user_.Label());
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Every rule stops refusing and the watch stops tripping until "
                                  "the time runs out, you end it, or you sign out.\n"
                                  "Every move a rule would have refused is logged as 'bypassed'.");
        }
    }

    // ---- Trip ----
    if (workflow_.InterlockTripActive()) {
        const Workflow::InterlockTrip t = workflow_.InterlockTripState();
        ImGui::TextColored(kIlRed, "TRIP: [%s] stopped %s", t.rule.c_str(), t.device.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Acknowledge")) workflow_.AcknowledgeInterlockTrip();
    }

    // ---- Last refusal ----
    const Workflow::InterlockEvent ev = workflow_.LastInterlockRefusal();
    if (ev.seq != 0) {
        const double age =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - ev.at).count();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("Last refusal (%.0f s ago): %s", age, ev.text.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::Separator();

    // ---- Rules, live ----
    const std::vector<IlRuleStatus> st = workflow_.InterlockStatus();
    if (st.empty()) ImGui::TextDisabled("No rules. An Admin adds them in View > Interlock Editor.");
    for (std::size_t i = 0; i < st.size() && i < cfg.rules.size(); ++i) {
        const IlRuleStatus& s = st[i];
        const IlRule&       r = cfg.rules[i];
        ImGui::PushID(static_cast<int>(i));
        StateDot(RuleColor(s));
        ImGui::SameLine();
        const char* state = !s.enabled ? "disabled"
                          : !s.active  ? "inactive"
                          : s.unknown  ? "ACTIVE (unreadable input)"
                                       : "ACTIVE";
        const bool open = ImGui::TreeNodeEx("##rule", ImGuiTreeNodeFlags_SpanAvailWidth |
                                                          (s.active ? ImGuiTreeNodeFlags_DefaultOpen : 0),
                                            "%s  -  %s", r.name.c_str(), state);
        if (open) {
            for (std::size_t c = 0; c < s.conds.size() && c < r.when.size(); ++c) {
                const IlCondStatus& cs = s.conds[c];
                StateDot(cs.unknown ? kIlAmber : cs.truth ? kIlRed : kIlGreen);
                ImGui::SameLine();
                ImGui::Text("%s", IlCondText(r.when[c]).c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("(%s)", cs.text.c_str());
            }
            for (const IlBlock& b : r.blocks)
                ImGui::BulletText("refuses %s", IlBlockText(b).c_str());
            if (!r.note.empty()) ImGui::TextDisabled("%s", r.note.c_str());
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    ImGui::End();
}

} // namespace firedog
