// PageLight.cpp : the CCS PD3 page (LED lighting controller on the LAN).
// Definitions of App members — see App.h.
//
// Every change goes through Workflow::Request* — queued for the driver's poll
// thread, which sends it within milliseconds — so a dragged slider never puts a
// TCP exchange on the UI thread. A slider SENDS on release, not per frame.
//
// Editing is locked while a run is active: a station or a recipe step sets the
// lighting it was taught at, and a hand on a slider mid-run would change the
// picture a detector is about to judge.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/LightCallout.h"
#include "core/Log.h"

#include <cfloat>
#include <cstdio>
#include <string>

namespace firedog {

namespace {

// A swatch for a channel's configured color; collimate and unknown are white.
ImVec4 SwatchColor(const std::string& color)
{
    if (color == "red")   return ImVec4(0.90f, 0.20f, 0.20f, 1.0f);
    if (color == "green") return ImVec4(0.20f, 0.80f, 0.30f, 1.0f);
    if (color == "blue")  return ImVec4(0.25f, 0.45f, 0.95f, 1.0f);
    return ImVec4(0.92f, 0.92f, 0.88f, 1.0f);
}

const CcsPd3::ChannelStatus* FindStatus(const CcsPd3::Status& s, int channel)
{
    for (const CcsPd3::ChannelStatus& c : s.ch) if (c.channel == channel) return &c;
    return nullptr;
}

} // namespace

void App::DrawLightListPanel()
{
    ImGui::TextUnformatted("Lighting");
    ImGui::Separator();
    if (lightId_.empty()) {
        ImGui::TextDisabled("no lighting controller configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.LightState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.LightName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectLight();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) workflow_.DisconnectLight();
    ImGui::EndDisabled();
    if (st == ConnState::Connected)
        ImGui::TextDisabled("disconnect leaves the lights as they are");

    CcsPd3* l = hardware_.Light();
    if (!l) return;
    const CcsPd3::Status s = l->GetStatus();
    if (st == ConnState::Connected && !s.linkUp) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Connecting));
        ImGui::TextWrapped("link lost - reconnecting");
        ImGui::PopStyleColor();
    }
    if (!s.warning.empty()) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Connecting));
        ImGui::TextWrapped("%s", s.warning.c_str());
        ImGui::PopStyleColor();
    }
    const std::string err = l->LastError();
    if (!err.empty() && (st == ConnState::Failed || !s.linkUp)) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
    }
}

// The color call-out block: light, %, one button per call-out, All off. Drawn
// by the Light page AND the Jog window's Camera tab, on the same App members
// (lightSel_, lightCallPct_, lightErr_), so the two surfaces are one control:
// a light or % picked on either shows on both. The button the controller's
// read-back matches is highlighted, so a click on either surface — or a run's
// light_color — shows on both as well. Locked while a run is active, like the
// rest of the page.
void App::DrawLightCalloutControls()
{
    const ConnState st = hardware_.LightState();
    CcsPd3*         l  = hardware_.Light();
    const bool connected = (st == ConnState::Connected) && l;
    const bool canEdit   = connected && !orch_.Busy();

    ImGui::SeparatorText("Color call-out");
    const std::vector<std::string> lights = workflow_.LightNames();
    if (lightSel_.empty() && !lights.empty()) lightSel_ = lights.front();

    // Stay on the line only if the next widget (w px) still fits, otherwise
    // wrap: the block lives in the wide Light page and the narrow Jog window.
    const auto sameLineIfFits = [](float w) {
        const float right = ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x;
        if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + w <= right)
            ImGui::SameLine();
    };
    const auto buttonW = [](const char* label) {
        return ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
    };

    std::string litColor;
    double      litPct = 0.0;
    const bool  lit = connected && workflow_.LitCallout(lightSel_, &litColor, &litPct);

    ImGui::BeginDisabled(!canEdit);
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("light", lightSel_.c_str())) {
        for (const std::string& n : lights)
            if (ImGui::Selectable(n.c_str(), n == lightSel_)) lightSel_ = n;
        ImGui::EndCombo();
    }
    sameLineIfFits(200.0f);
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("%##callpct", &lightCallPct_, 0.0f, 100.0f, "%.0f %%");

    bool first = true;
    for (const std::string& name : LightColorNames()) {
        LightColor c = LightColor::White;
        ParseLightColor(name, c);
        const std::string label = LightColorLabel(c);
        if (!first) sameLineIfFits(buttonW(label.c_str()));
        first = false;
        const bool active = lit && litColor == name;
        if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(label.c_str())) {
            lightErr_.clear();
            if (!workflow_.RequestLightCallout(lightSel_, name, lightCallPct_, &lightErr_))
                FD_WARN("light", "call-out refused: %s", lightErr_.c_str());
        }
        if (active) ImGui::PopStyleColor();
    }
    if (ImGui::Button("All off") && l) {
        lightErr_.clear();
        for (const CcsPd3::Channel& ch : l->Channels())
            workflow_.RequestLightChannel(ch.channel, 0.0);
    }
    ImGui::EndDisabled();

    // What the controller says the selected light is showing right now.
    if (connected) {
        if (!lit) {
            ImGui::TextDisabled("lit: (no call-out)");
        } else if (litColor == "off") {
            ImGui::TextDisabled("lit: off");
        } else {
            LightColor c = LightColor::White;
            ParseLightColor(litColor, c);
            ImGui::TextDisabled("lit: %s %.0f %%", LightColorLabel(c).c_str(), litPct);
        }
    }
    if (!lightErr_.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
        ImGui::TextWrapped("%s", lightErr_.c_str());
        ImGui::PopStyleColor();
    }
}

void App::DrawLightDetailPanel()
{
    const ConnState st = hardware_.LightState();
    CcsPd3*         l  = hardware_.Light();
    const bool connected = (st == ConnState::Connected) && l;
    const CcsPd3::Status s = l ? l->GetStatus() : CcsPd3::Status{};

    const bool running = orch_.Busy();
    const bool canEdit = connected && !running;

    // ---- color call-outs -------------------------------------------------
    DrawLightCalloutControls();

    // ---- per channel -----------------------------------------------------
    ImGui::SeparatorText("Channels");
    if (!l) {
        ImGui::TextDisabled("connect the lighting controller to operate it");
        return;
    }
    for (const CcsPd3::Channel& ch : l->Channels()) {
        ImGui::PushID(ch.channel);
        ImGui::ColorButton("##sw", SwatchColor(ch.color),
                           ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker,
                           ImVec2(14.0f, 14.0f));
        ImGui::SameLine();
        ImGui::Text("L%d  %-10s", ch.channel, ch.name.c_str());
        ImGui::SameLine(170.0f);

        const CcsPd3::ChannelStatus* rb = s.valid ? FindStatus(s, ch.channel) : nullptr;
        ImGui::BeginDisabled(!canEdit);
        float& pct = lightPct_[ch.channel];
        ImGui::SetNextItemWidth(220.0f);
        ImGui::SliderFloat("##pct", &pct, 0.0f, 100.0f, "%.0f %%");
        if (ImGui::IsItemDeactivatedAfterEdit())
            workflow_.RequestLightChannel(ch.channel, pct);
        // Not under the operator's hand: the slider shows what the controller
        // SAYS, so a run, a call-out or the front panel is never misreported.
        // An OFF channel keeps its last level, which is what ON brings back.
        else if (!ImGui::IsItemActive() && rb && rb->on)
            pct = static_cast<float>(CcsPd3::NativeToPercent(rb->intensity));
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("ON")) {
            if (pct <= 0.0f) pct = 100.0f;   // ON with nothing set means full
            workflow_.RequestLightChannel(ch.channel, pct);
        }
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
        if (ImGui::Button("OFF")) workflow_.RequestLightChannel(ch.channel, 0.0);
        ImGui::PopStyleColor();
        ImGui::EndDisabled();

        ImGui::SameLine();
        if (rb) {
            if (rb->on)
                ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "ON %.1f %%",
                                   CcsPd3::NativeToPercent(rb->intensity));
            else
                ImGui::TextDisabled("off");
        } else {
            ImGui::TextDisabled("-");
        }
        ImGui::PopID();
    }
    if (running)
        ImGui::TextDisabled("a run is active - lighting is set by the run");
    else if (!connected)
        ImGui::TextDisabled("connect the lighting controller to operate it");
}

void App::ApplyLightingNow(const AlignLighting& l)
{
    if (l.kind != "light") return;
    std::string why;
    if (!workflow_.RequestLightCallout(l.channel, l.color, l.value, &why))
        FD_WARN("light", "apply %s refused: %s", AlignLightingSummary(l).c_str(), why.c_str());
}

void App::DrawLightInfoPanel()
{
    ImGui::TextUnformatted("Controller info");
    ImGui::Separator();

    const ConnState st = hardware_.LightState();
    CcsPd3*         l  = hardware_.Light();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device");
    ImGui::TextWrapped("%s", l ? l->DeviceId().c_str() : "-");
    if (!l) return;

    const CcsPd3::Status s = l->GetStatus();
    ImGui::Spacing();
    ImGui::TextDisabled("Link");
    ImGui::TextUnformatted(s.linkUp ? "up" : "down");
    ImGui::TextDisabled("Unit");
    if (s.unitError) ImGui::TextColored(StateColor(ConnState::Failed), "overcurrent / fan stop");
    else             ImGui::TextUnformatted(s.valid ? "normal" : "-");

    for (const CcsPd3::Channel& ch : l->Channels()) {
        const CcsPd3::ChannelStatus* c = FindStatus(s, ch.channel);
        ImGui::Spacing();
        ImGui::TextDisabled("L%d %s%s%s", ch.channel, ch.name.c_str(),
                            ch.group.empty() ? "" : "  @", ch.group.c_str());
        if (!c || !s.valid) { ImGui::TextDisabled("no reading yet"); continue; }
        ImGui::Text("%s  F%03d  (%.1f %%)", c->on ? "ON " : "off", c->intensity,
                    CcsPd3::NativeToPercent(c->intensity));
        if (c->mode != "00")
            ImGui::TextColored(StateColor(ConnState::Connecting), "strobe mode S%s", c->mode.c_str());
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Polling");
    ImGui::TextUnformatted(l->IsPolling() ? "yes" : "no");
}

} // namespace firedog
