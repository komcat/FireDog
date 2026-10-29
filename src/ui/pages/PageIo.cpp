// PageIo.cpp : the IO page (Ezi-IO + WAGO) and the Pneumatics page (synthetic
// slides composed from IO pins). Definitions of App members — see App.h.
//
// Pin addressing is already normalized by the drivers (bit i == pin i for both
// inputs and outputs), so these panels index pins directly. Both pages gate
// their commands behind an armed checkbox — default ON, as an explicit kill
// switch rather than a speed bump.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <cstdint>
#include <cstdio>

namespace firedog {

void App::DrawIoListPanel()
{
    const size_t n = hardware_.IoCount();

    // Keep the current selection if still valid; otherwise fall back to the
    // first device (same convention as the Motion list).
    if (selectedIo_ < 0 || static_cast<size_t>(selectedIo_) >= n ||
        !hardware_.IoDevice(static_cast<size_t>(selectedIo_))) {
        selectedIo_ = n > 0 ? 0 : -1;
    }

    ImGui::TextUnformatted("IO devices");
    ImGui::Separator();

    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        IIoDevice* d = hardware_.IoDevice(i);
        if (!d) continue;
        any = true;
        const ConnState st = hardware_.IoState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        const std::string& name = hardware_.IoName(i);
        const ImVec2 sz = ImGui::CalcTextSize(name.c_str());
        if (ImGui::Selectable(name.c_str(), selectedIo_ == static_cast<int>(i),
                              ImGuiSelectableFlags_None, ImVec2(sz.x, 0.0f))) {
            selectedIo_ = static_cast<int>(i);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", d->Type().c_str());
        ImGui::SameLine();
        ImGui::TextColored(StateColor(st), "%s", StateText(st));
        ImGui::PopID();
    }
    if (!any) ImGui::TextDisabled("no IO devices configured");

    // ---- Background IO watch (input edge -> output action) ----
    IoWatch* w = hardware_.Watch();
    if (w) {
        ImGui::Spacing();
        ImGui::Separator();
        bool active = w->IsActive();
        if (ImGui::Checkbox("Watch bindings", &active)) w->SetActive(active);
        for (size_t i = 0; i < w->BindingCount(); ++i) {
            const IoWatch::Binding& b = w->BindingAt(i);
            ImGui::PushID(static_cast<int>(i));
            const bool on = w->InputOn(i);
            ImGui::TextColored(on ? StateColor(ConnState::Connected)
                                  : ImVec4(0.45f, 0.45f, 0.48f, 1.0f), "*");
            ImGui::SameLine();
            ImGui::TextUnformatted(b.name.c_str());
            const uint64_t hits = w->Hits(i);
            if (hits > 0) {
                ImGui::SameLine();
                ImGui::TextDisabled("(%llu)", static_cast<unsigned long long>(hits));
            }
            ImGui::PopID();
        }
    }
}

void App::DrawIoDetailPanel()
{
    const size_t n = hardware_.IoCount();
    IIoDevice* d = (selectedIo_ >= 0 && static_cast<size_t>(selectedIo_) < n)
                     ? hardware_.IoDevice(static_cast<size_t>(selectedIo_))
                     : nullptr;
    if (!d) {
        ImGui::TextDisabled("Select a device from the list.");
        return;
    }

    const size_t    idx = static_cast<size_t>(selectedIo_);
    const ConnState st  = hardware_.IoState(idx);

    ImGui::TextUnformatted(hardware_.IoName(idx).c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "[%s]", StateText(st));
    ImGui::Separator();
    ImGui::Text("Type : %s", d->Type().c_str());
    ImGui::Text("Pins : %d in / %d out", d->InputCount(), d->OutputCount());

    if (st == ConnState::Failed) {
        ImGui::Spacing();
        ImGui::TextColored(StateColor(ConnState::Failed), "error: %s", d->LastError().c_str());
    }

    // ---- Outputs (writes drive real actuators — gated behind arming) ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextUnformatted("Outputs");
    ImGui::SameLine();
    ImGui::Checkbox("Enable", &ioArmed_);
    ImGui::SameLine();
    if (ioArmed_) {
        ImGui::TextColored(StateColor(ConnState::Connecting), "LIVE - toggles drive hardware");
    } else {
        ImGui::TextDisabled("check Enable to activate the toggles");
    }

    const IIoDevice::IoStatus s = d->Status();
    const bool canWrite = ioArmed_ && st == ConnState::Connected;
    if (ImGui::BeginTable("iooutputs", 4,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Pin", ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("##toggle", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableHeadersRow();
        for (int pin = 0; pin < d->OutputCount(); ++pin) {
            const bool on = ((s.outputs >> pin) & 1ull) != 0;
            ImGui::TableNextRow();
            ImGui::PushID(pin);
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", pin);
            ImGui::TableSetColumnIndex(1);
            const std::string name = d->OutputPinName(pin);
            if (name.empty()) ImGui::TextDisabled("--");
            else              ImGui::TextUnformatted(name.c_str());
            ImGui::TableSetColumnIndex(2);
            if (on) ImGui::TextColored(StateColor(ConnState::Connected), "ON");
            else    ImGui::TextDisabled("OFF");
            ImGui::TableSetColumnIndex(3);
            ImGui::BeginDisabled(!canWrite);
            if (ImGui::SmallButton(on ? "Turn OFF" : "Turn ON")) {
                workflow_.SetIoOutput(idx, pin, !on);
            }
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (!s.valid && st == ConnState::Connected) {
        ImGui::TextDisabled("waiting for first poll...");
    }
}

void App::DrawIoInputPanel()
{
    // Right panel: live input pin states for the selected IO device.
    const size_t n = hardware_.IoCount();
    IIoDevice* d = (selectedIo_ >= 0 && static_cast<size_t>(selectedIo_) < n)
                     ? hardware_.IoDevice(static_cast<size_t>(selectedIo_))
                     : nullptr;

    ImGui::TextUnformatted("Inputs");
    ImGui::Separator();
    if (!d) { ImGui::TextDisabled("no selection"); return; }

    const IIoDevice::IoStatus s = d->Status();
    if (ImGui::BeginTable("ioinputs", 4,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Pin", ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("Latch", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();
        for (int pin = 0; pin < d->InputCount(); ++pin) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", pin);
            ImGui::TableSetColumnIndex(1);
            const std::string name = d->InputPinName(pin);
            if (name.empty()) ImGui::TextDisabled("--");
            else              ImGui::TextUnformatted(name.c_str());
            ImGui::TableSetColumnIndex(2);
            if ((s.inputs >> pin) & 1) ImGui::TextColored(StateColor(ConnState::Connected), "ON");
            else                       ImGui::TextDisabled("OFF");
            ImGui::TableSetColumnIndex(3);
            if ((s.latch >> pin) & 1) ImGui::TextColored(StateColor(ConnState::Connecting), "SET");
            else                      ImGui::TextDisabled("--");
        }
        ImGui::EndTable();
    }
    if (!s.valid) ImGui::TextDisabled("waiting for first poll...");

    ImGui::Spacing();
    if (ImGui::SmallButton("Clear latch")) {
        workflow_.ClearIoLatch(static_cast<size_t>(selectedIo_), ~0ull);
    }
}

// Position -> display color for the Pneumatics page.
static ImVec4 SlideColor(SlidePosition p)
{
    switch (p) {
        case SlidePosition::Extended:      return ImVec4(0.40f, 0.90f, 0.50f, 1.0f);
        case SlidePosition::Retracted:     return ImGui::GetStyleColorVec4(ImGuiCol_Text);
        case SlidePosition::Extending:
        case SlidePosition::Retracting:    return ImVec4(0.95f, 0.80f, 0.30f, 1.0f);
        case SlidePosition::HardwareError:
        case SlidePosition::Timeout:       return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
        default:                           return ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
    }
}

void App::DrawPneumaticListPanel()
{
    const size_t n = hardware_.SlideCount();
    if (selectedSlide_ < 0 || static_cast<size_t>(selectedSlide_) >= n) {
        selectedSlide_ = n > 0 ? 0 : -1;
    }

    ImGui::TextUnformatted("Pneumatic slides");
    ImGui::Separator();

    for (size_t i = 0; i < n; ++i) {
        PneumaticSlide* s = hardware_.Slide(i);
        const SlidePosition p = s->Position();
        ImGui::PushID(static_cast<int>(i));
        const ImVec2 sz = ImGui::CalcTextSize(s->Name().c_str());
        if (ImGui::Selectable(s->Name().c_str(), selectedSlide_ == static_cast<int>(i),
                              ImGuiSelectableFlags_None, ImVec2(sz.x, 0.0f))) {
            selectedSlide_ = static_cast<int>(i);
        }
        ImGui::SameLine();
        ImGui::TextColored(SlideColor(p), "%s", SlidePositionName(p));
        ImGui::PopID();
    }
    if (n == 0) ImGui::TextDisabled("no pneumatic slides configured");
}

void App::DrawPneumaticDetailPanel()
{
    const size_t n = hardware_.SlideCount();
    PneumaticSlide* s = (selectedSlide_ >= 0 && static_cast<size_t>(selectedSlide_) < n)
                          ? hardware_.Slide(static_cast<size_t>(selectedSlide_))
                          : nullptr;
    if (!s) {
        ImGui::TextDisabled("Select a slide from the list.");
        return;
    }

    const PneumaticSlide::Status st = s->GetStatus();
    const bool ioUp = s->IsIoConnected();

    ImGui::TextUnformatted(s->Name().c_str());
    ImGui::SameLine();
    ImGui::TextColored(SlideColor(st.position), "[%s]", SlidePositionName(st.position));
    ImGui::Separator();
    ImGui::Text("IO      : %s", ioUp ? "connected" : "not connected");
    ImGui::Text("Timeout : %u ms", s->TimeoutMs());
    if (st.moving) ImGui::Text("Moving  : %u ms elapsed", st.elapsedMs);
    else if (st.elapsedMs > 0) ImGui::Text("Last move : %u ms", st.elapsedMs);

    if (st.position == SlidePosition::Timeout || st.position == SlidePosition::HardwareError) {
        ImGui::Spacing();
        ImGui::TextColored(SlideColor(st.position), "error: %s",
                           st.position == SlidePosition::HardwareError
                               ? "both sensors active" : s->LastError().c_str());
    }

    // ---- Commands (drive real actuators — gated behind arming) ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextUnformatted("Control");
    ImGui::SameLine();
    ImGui::Checkbox("Enable", &pneuArmed_);
    ImGui::SameLine();
    if (pneuArmed_) {
        ImGui::TextColored(StateColor(ConnState::Connecting), "LIVE - buttons drive hardware");
    } else {
        ImGui::TextDisabled("check Enable to activate the buttons");
    }

    const size_t idx = static_cast<size_t>(selectedSlide_);
    ImGui::BeginDisabled(!pneuArmed_ || !ioUp);
    if (ImGui::Button("Extend"))  workflow_.ExtendSlide(idx);
    ImGui::SameLine();
    if (ImGui::Button("Retract")) workflow_.RetractSlide(idx);
    ImGui::SameLine();
    if (ImGui::Button("Stop"))    workflow_.StopSlide(idx);
    ImGui::EndDisabled();
}

void App::DrawPneumaticInfoPanel()
{
    // Right panel: the live output/sensor pin states for the selected slide.
    const size_t n = hardware_.SlideCount();
    PneumaticSlide* s = (selectedSlide_ >= 0 && static_cast<size_t>(selectedSlide_) < n)
                          ? hardware_.Slide(static_cast<size_t>(selectedSlide_))
                          : nullptr;

    ImGui::TextUnformatted("Slide IO");
    ImGui::Separator();
    if (!s) { ImGui::TextDisabled("no selection"); return; }

    const PneumaticSlide::Status st = s->GetStatus();
    if (ImGui::BeginTable("slideio", 2,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Signal");
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();
        auto row = [](const char* label, bool on) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            if (on) ImGui::TextColored(StateColor(ConnState::Connected), "ON");
            else    ImGui::TextDisabled("OFF");
        };
        row("Output (extend)",  st.outputOn);
        row("Extended sensor",  st.extendedOn);
        row("Retracted sensor", st.retractedOn);
        ImGui::EndTable();
    }
    if (!st.ioValid) ImGui::TextDisabled("waiting for first poll...");
}

} // namespace firedog
