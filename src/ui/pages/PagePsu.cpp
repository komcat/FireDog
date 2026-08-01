// PagePsu.cpp : the UDP3305S page (UNI-T 3-channel DC power supply).
// Definitions of App members — see App.h. Setpoints seed from the device at
// connect (nothing to save back); disconnect forces every output OFF, which is
// bobcat's PSU safety habit and deliberately unlike the CLD.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <cstdio>

namespace firedog {

namespace {
// Datasheet channel limits: CH1/CH2 0..32 V / 0..5 A, CH3 0..6 V / 0..3 A.
float PsuMaxVolts(int ch) { return ch == 3 ? 6.0f : 32.0f; }
float PsuMaxAmps(int ch)  { return ch == 3 ? 3.0f : 5.0f; }
} // namespace

void App::DrawPsuListPanel()
{
    ImGui::TextUnformatted("Power supply");
    ImGui::Separator();
    if (psuId_.empty()) {
        ImGui::TextDisabled("no PSU configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.PsuState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.PsuName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectPsu();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectPsu();
        psuPulled_ = false;   // re-seed edits on the next connect
    }
    ImGui::EndDisabled();
    if (st == ConnState::Connected)
        ImGui::TextDisabled("disconnect forces all outputs OFF");

    if (st == ConnState::Failed && hardware_.Psu()) {
        const std::string err = hardware_.Psu()->LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawPsuDetailPanel()
{
    const ConnState st  = hardware_.PsuState();
    UDP3305S*       psu = hardware_.Psu();
    const bool connected = (st == ConnState::Connected) && psu;
    const UDP3305S::Status s = psu ? psu->GetStatus() : UDP3305S::Status{};
    const int channels = psu ? psu->Channels() : 3;

    if (connected && !psuPulled_) {   // seed edits from the device's setpoints
        for (int ch = 0; ch < channels; ++ch) {
            if (s.ch[ch].setV > 0.0) psuVSet_[ch] = static_cast<float>(s.ch[ch].setV);
            if (s.ch[ch].setI > 0.0) psuISet_[ch] = static_cast<float>(s.ch[ch].setI);
        }
        psuPulled_ = true;
    }
    if (!connected) psuPulled_ = false;

    for (int ch = 1; ch <= channels; ++ch) {
        const UDP3305S::ChannelStatus& c = s.ch[ch - 1];
        ImGui::PushID(ch);
        ImGui::Text("CH%d", ch);
        ImGui::SameLine();
        ImGui::TextDisabled("(0-%.0f V / 0-%.0f A)", PsuMaxVolts(ch), PsuMaxAmps(ch));
        ImGui::Separator();

        ImGui::BeginDisabled(!connected);
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("V##v", &psuVSet_[ch - 1], 0.0f, 0.0f, "%.3f");
        if (psuVSet_[ch - 1] < 0.0f)             psuVSet_[ch - 1] = 0.0f;
        if (psuVSet_[ch - 1] > PsuMaxVolts(ch))  psuVSet_[ch - 1] = PsuMaxVolts(ch);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("A##i", &psuISet_[ch - 1], 0.0f, 0.0f, "%.3f");
        if (psuISet_[ch - 1] < 0.0f)            psuISet_[ch - 1] = 0.0f;
        if (psuISet_[ch - 1] > PsuMaxAmps(ch))  psuISet_[ch - 1] = PsuMaxAmps(ch);
        ImGui::SameLine();
        if (ImGui::Button("Apply"))
            workflow_.SetPsuVI(ch, psuVSet_[ch - 1], psuISet_[ch - 1]);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("Output ON")) workflow_.PsuOutputOn(ch);
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
        if (ImGui::Button("Output OFF")) workflow_.PsuOutputOff(ch);
        ImGui::PopStyleColor();
        ImGui::EndDisabled();

        if (connected) {
            if (c.outputOn)
                ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f),
                                   "ON  |  %.3f V  %.3f A (set %.3f V / %.3f A)",
                                   c.measV, c.measI, c.setV, c.setI);
            else
                ImGui::TextDisabled("OFF  |  set %.3f V / %.3f A", c.setV, c.setI);
        }
        ImGui::Spacing();
        ImGui::PopID();
    }
    if (!connected) ImGui::TextDisabled("connect the power supply to operate it");
}

void App::DrawPsuInfoPanel()
{
    ImGui::TextUnformatted("Supply info");
    ImGui::Separator();

    const ConnState st  = hardware_.PsuState();
    UDP3305S*       psu = hardware_.Psu();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    ImGui::TextWrapped("%s", (psu && !psu->DeviceId().empty()) ? psu->DeviceId().c_str() : "-");

    if (psu) {
        const UDP3305S::Status s = psu->GetStatus();
        for (int ch = 1; ch <= psu->Channels(); ++ch) {
            const UDP3305S::ChannelStatus& c = s.ch[ch - 1];
            ImGui::Spacing();
            ImGui::TextDisabled("CH%d", ch);
            if (c.outputOn) ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "ON");
            else            ImGui::TextUnformatted("OFF");
            if (s.valid) ImGui::Text("%.3f V  %.3f A", c.measV, c.measI);
            else         ImGui::TextDisabled("no reading yet");
            ImGui::Text("set %.3f V / %.3f A", c.setV, c.setI);
        }
        ImGui::Spacing();
        ImGui::TextDisabled("Polling");
        ImGui::TextUnformatted(psu->IsPolling() ? "yes" : "no");
    }
}

} // namespace firedog
