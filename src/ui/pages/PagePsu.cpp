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

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstdio>

namespace firedog {

namespace {
// Datasheet channel limits: CH1/CH2 0..32 V / 0..5 A, CH3 0..6 V / 0..3 A.
float PsuMaxVolts(int ch) { return ch == 3 ? 6.0f : 32.0f; }
float PsuMaxAmps(int ch)  { return ch == 3 ? 3.0f : 5.0f; }
} // namespace

// The supply the middle and right panels operate: an index into the PSU fleet,
// clamped every frame (the fleet is fixed after Start, but a saved index from a
// machine with more supplies must not read past this one's).
std::size_t App::PsuSelected()
{
    if (psuSel_ < 0 || static_cast<std::size_t>(psuSel_) >= hardware_.PsuCount()) psuSel_ = 0;
    return static_cast<std::size_t>(psuSel_);
}

void App::DrawPsuListPanel()
{
    ImGui::TextUnformatted("Power supplies");
    ImGui::Separator();
    if (hardware_.PsuCount() == 0) {
        ImGui::TextDisabled("no PSU configured in hardware.json");
        return;
    }

    // One row per supply; the selected one is what the other two panels drive.
    const std::size_t sel = PsuSelected();
    for (std::size_t i = 0; i < hardware_.PsuCount(); ++i) {
        const ConnState rst = hardware_.PsuState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(rst), "%s", StateGlyph(rst));
        ImGui::SameLine();
        if (ImGui::Selectable(hardware_.PsuName(i).c_str(), i == sel) && i != sel) {
            psuSel_    = static_cast<int>(i);
            psuPulled_ = false;   // re-seed the edits from THIS supply's setpoints
        }
        ImGui::PopID();
    }
    ImGui::Separator();

    const std::size_t idx = PsuSelected();
    const std::string& name = hardware_.PsuName(idx);
    const ConnState st = hardware_.PsuState(idx);
    ImGui::TextUnformatted(name.c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectPsu(name);
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectPsu(name);
        psuPulled_ = false;   // re-seed edits on the next connect
    }
    ImGui::EndDisabled();
    if (st == ConnState::Connected)
        ImGui::TextDisabled("disconnect forces all outputs OFF");

    UDP3305S* dev = hardware_.PsuDevice(idx);
    if (st == ConnState::Failed && dev) {
        const std::string err = dev->LastError();
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
    if (hardware_.PsuCount() == 0) {
        ImGui::TextDisabled("no PSU configured in hardware.json");
        return;
    }
    ImGui::Text("%s", hardware_.PsuName(PsuSelected()).c_str());
    ImGui::Separator();
    DrawPsuChannelControls(false);
}

// The selected supply's channels: CV/CC, V/A edits, Apply, Output ON/OFF and
// the live line. Shared by the PSU page and the Jog window's PSU tab (compact)
// - one selection (psuSel_), one set of edit buffers, one apply path - the way
// the Camera tab shares the Camera page's exposure controls.
void App::DrawPsuChannelControls(bool compact)
{
    const std::size_t  idx  = PsuSelected();
    const std::string& name = hardware_.PsuName(idx);
    const ConnState st  = hardware_.PsuState(idx);
    UDP3305S*       psu = hardware_.PsuDevice(idx);
    const bool connected = (st == ConnState::Connected) && psu;
    const UDP3305S::Status s = psu ? psu->GetStatus() : UDP3305S::Status{};
    const int channels = psu ? psu->Channels() : 3;
    const float fieldW = compact ? 64.0f : 120.0f;

    if (connected && !psuPulled_) {   // seed edits from the device's setpoints
        for (int ch = 0; ch < channels; ++ch) {
            if (s.ch[ch].setV > 0.0) psuVSet_[ch] = static_cast<float>(s.ch[ch].setV);
            if (s.ch[ch].setI > 0.0) psuISet_[ch] = static_cast<float>(s.ch[ch].setI);
        }
        psuPulled_ = true;
    }
    if (!connected) psuPulled_ = false;

    std::array<int, UDP3305S::kMaxChannels>& modes = psuModes_[name];   // CV (0) until chosen
    for (int ch = 1; ch <= channels; ++ch) {
        const UDP3305S::ChannelStatus& c = s.ch[ch - 1];
        int& mode = modes[static_cast<std::size_t>(ch - 1)];
        ImGui::PushID(ch);
        ImGui::Text("CH%d", ch);
        ImGui::SameLine();
        if (!compact) {
            ImGui::TextDisabled("(0-%.0f V / 0-%.0f A)", PsuMaxVolts(ch), PsuMaxAmps(ch));
            ImGui::SameLine();
        }
        // CV / CC: which number is the target and which the limit. The supply
        // has no mode switch - the load decides - so this changes the labels
        // and what the status line holds the channel to, never a command.
        ImGui::RadioButton("CV", &mode, 0);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Constant voltage: V is the setpoint, A is the current limit.");
        ImGui::SameLine();
        ImGui::RadioButton("CC", &mode, 1);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Constant current: A is the setpoint, V is the voltage limit\n"
                              "(the most the supply may raise the voltage to push that current).");
        ImGui::Separator();

        ImGui::BeginDisabled(!connected);
        ImGui::SetNextItemWidth(fieldW);
        ImGui::InputFloat(mode == 0 ? "V###v" : "V limit###v", &psuVSet_[ch - 1], 0.0f, 0.0f, "%.3f");
        if (psuVSet_[ch - 1] < 0.0f)             psuVSet_[ch - 1] = 0.0f;
        if (psuVSet_[ch - 1] > PsuMaxVolts(ch))  psuVSet_[ch - 1] = PsuMaxVolts(ch);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(fieldW);
        ImGui::InputFloat(mode == 0 ? "A limit###i" : "A###i", &psuISet_[ch - 1], 0.0f, 0.0f, "%.3f");
        if (psuISet_[ch - 1] < 0.0f)            psuISet_[ch - 1] = 0.0f;
        if (psuISet_[ch - 1] > PsuMaxAmps(ch))  psuISet_[ch - 1] = PsuMaxAmps(ch);
        if (!compact) ImGui::SameLine();
        if (ImGui::Button("Apply"))
            workflow_.SetPsuVI(name, ch, psuVSet_[ch - 1], psuISet_[ch - 1]);
        // Page: fields + Apply, then ON/OFF below. Compact: fields, then
        // Apply ON OFF on one row.
        if (compact) ImGui::SameLine();

        // Compact: the output state IS the button colour, so the row reads at
        // a glance in a narrow window.
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button(compact ? "ON" : "Output ON")) workflow_.PsuOutputOn(name, ch);
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
        if (ImGui::Button(compact ? "OFF" : "Output OFF")) workflow_.PsuOutputOff(name, ch);
        ImGui::PopStyleColor();
        ImGui::EndDisabled();

        if (connected) {
            if (c.outputOn) {
                // What the LOAD has it doing, against what was asked for: a CV
                // channel at its current limit has not reached its voltage, a
                // CC channel short of its current is at its voltage limit.
                const bool inCC = UDP3305S::IsCurrentLimited(c);
                if (compact)
                    ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "ON %.3f V  %.3f A",
                                       c.measV, c.measI);
                else
                    ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f),
                                       "ON  |  %.3f V  %.3f A (set %.3f V / %.3f A)",
                                       c.measV, c.measI, c.setV, c.setI);
                ImGui::SameLine();
                if (inCC == (mode == 1))
                    ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "[%s]", inCC ? "CC" : "CV");
                else if (compact)
                    ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.25f, 1.0f), "[%s!]", inCC ? "CC" : "CV");
                else
                    ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.25f, 1.0f), "[%s - %s]",
                                       inCC ? "CC" : "CV",
                                       inCC ? "at the current limit, voltage not reached"
                                            : "at the voltage limit, current not reached");
                if (compact && inCC != (mode == 1) && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", inCC ? "at the current limit - voltage not reached"
                                                 : "at the voltage limit - current not reached");
            } else {
                ImGui::TextDisabled(compact ? "OFF  set %.3f V / %.3f A"
                                            : "OFF  |  set %.3f V / %.3f A", c.setV, c.setI);
            }
        }
        if (compact) ImGui::Separator(); else ImGui::Spacing();
        ImGui::PopID();
    }
    if (!connected) ImGui::TextDisabled("connect the power supply to operate it");
}

// The Jog PSU tab's three presets. A click puts the preset into the selected
// supply's channel - its CV/CC radio, both edits, and an Apply - through the
// same SetPsuVI the Apply button uses. It never switches an output: ON stays
// the operator's own press. Right-click edits a preset in a POPUP (never a
// modal, for the Pos tab's reason); the presets are a bench layout, saved in
// UiSettings, not product state.
void App::DrawPsuPresetRow()
{
    const std::size_t  idx  = PsuSelected();
    const std::string& name = hardware_.PsuName(idx);
    UDP3305S*          psu  = hardware_.PsuDevice(idx);
    const bool connected = hardware_.PsuState(idx) == ConnState::Connected && psu;
    const int  channels  = psu ? psu->Channels() : 3;

    const auto amps = [](double a, char* b, std::size_t n) {
        if (a < 1.0) std::snprintf(b, n, "%.0f mA", a * 1000.0);
        else         std::snprintf(b, n, "%.3f A", a);
    };

    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const float bw  = (ImGui::GetContentRegionAvail().x
                       - gap * static_cast<float>(kPsuPresetCount - 1))
                      / static_cast<float>(kPsuPresetCount);
    const float bh  = ImGui::GetTextLineHeight() * 3.0f + ImGui::GetStyle().FramePadding.y * 2.0f;

    for (int i = 0; i < kPsuPresetCount; ++i) {
        PsuPreset& p = psuPresets_[static_cast<std::size_t>(i)];
        p.channel = std::clamp(p.channel, 1, channels);
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);

        // Three lines: channel + mode, then the setpoint, then the limit.
        char a[24];
        amps(p.amps, a, sizeof(a));
        char label[96];
        if (p.mode == 1)
            std::snprintf(label, sizeof(label), "CH%d CC\n%s\n%.2f V###preset",
                          p.channel, a, p.volts);
        else
            std::snprintf(label, sizeof(label), "CH%d CV\n%.2f V\n%s###preset",
                          p.channel, p.volts, a);

        ImGui::BeginDisabled(!connected);
        if (ImGui::Button(label, ImVec2(bw, bh))) {
            const int ch = p.channel;
            const float v = std::clamp(static_cast<float>(p.volts), 0.0f, PsuMaxVolts(ch));
            const float c = std::clamp(static_cast<float>(p.amps), 0.0f, PsuMaxAmps(ch));
            psuModes_[name][static_cast<std::size_t>(ch - 1)] = p.mode;
            psuVSet_[ch - 1] = v;
            psuISet_[ch - 1] = c;
            workflow_.SetPsuVI(name, ch, v, c);
            FD_INFO("psu", "%s CH%d preset %d: %s %.3f V / %.3f A", name.c_str(), ch, i + 1,
                    p.mode == 1 ? "CC" : "CV", v, c);
        }
        ImGui::EndDisabled();

        // AllowWhenDisabled: the presets can be laid out before the supply is up.
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Preset %d: set %s CH%d to %s %.3f V / %.3f A\n"
                              "(%s). Output ON/OFF is not touched.\n"
                              "Right-click to edit.",
                              i + 1, name.c_str(), p.channel, p.mode == 1 ? "CC" : "CV",
                              p.volts, p.amps,
                              p.mode == 1 ? "A is the setpoint, V the limit"
                                          : "V is the setpoint, A the limit");
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) ImGui::OpenPopup("psupreset");
        }

        if (ImGui::BeginPopup("psupreset")) {
            ImGui::TextDisabled("Preset %d", i + 1);
            ImGui::Separator();
            bool changed = false;
            ImGui::TextUnformatted("Channel");
            for (int ch = 1; ch <= channels; ++ch) {
                ImGui::SameLine();
                char cl[8];
                std::snprintf(cl, sizeof(cl), "%d", ch);
                changed |= ImGui::RadioButton(cl, &p.channel, ch);
            }
            ImGui::TextUnformatted("Mode   ");
            ImGui::SameLine();
            changed |= ImGui::RadioButton("CV", &p.mode, 0);
            ImGui::SameLine();
            changed |= ImGui::RadioButton("CC", &p.mode, 1);
            ImGui::SetNextItemWidth(90.0f);
            ImGui::InputDouble(p.mode == 0 ? "V###pv" : "V limit###pv", &p.volts, 0.0, 0.0, "%.3f");
            changed |= ImGui::IsItemDeactivatedAfterEdit();
            ImGui::SetNextItemWidth(90.0f);
            ImGui::InputDouble(p.mode == 0 ? "A limit###pa" : "A###pa", &p.amps, 0.0, 0.0, "%.3f");
            changed |= ImGui::IsItemDeactivatedAfterEdit();
            p.volts = std::clamp(p.volts, 0.0, static_cast<double>(PsuMaxVolts(p.channel)));
            p.amps  = std::clamp(p.amps, 0.0, static_cast<double>(PsuMaxAmps(p.channel)));
            if (changed) SaveUiSettings();
            ImGui::EndPopup();
        }
        ImGui::PopID();
    }
}

void App::DrawPsuInfoPanel()
{
    ImGui::TextUnformatted("Supply info");
    ImGui::Separator();
    if (hardware_.PsuCount() == 0) return;

    const std::size_t idx = PsuSelected();
    const ConnState st  = hardware_.PsuState(idx);
    UDP3305S*       psu = hardware_.PsuDevice(idx);

    ImGui::TextDisabled("Name");
    ImGui::TextUnformatted(hardware_.PsuName(idx).c_str());
    ImGui::TextDisabled("Streams");
    ImGui::TextWrapped("psu/%s/ch<n>/v|i%s", hardware_.PsuName(idx).c_str(),
                       idx == 0 ? "  (+ psu/ch<n>/v|i)" : "");
    ImGui::Spacing();
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
