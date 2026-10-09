// PageWatchEdit.cpp : the IO Watch editor (View menu, full-page takeover).
// Definitions of App members — see App.h.
//
// Edits a WORKING COPY of the io_watch config (watchCfg_): Apply rebuilds the
// live watch through Workflow::ApplyIoWatch, Save also writes
// config/hardware.json. Bindings reference pins BY NAME; Workflow resolves
// them once when the watch starts.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <cstdio>

namespace firedog {

// IO Watch editor — full-page main-window content (header/Close drawn by
// DrawMainWindow's editor-page branch).
void App::DrawWatchEditPanel()
{
    // Master settings + actions. Apply rebuilds the running watch from the
    // working copy; Save also persists it into config/hardware.json.
    ImGui::Checkbox("Enabled", &watchCfg_.enabled);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    ImGui::InputInt("Poll ms", &watchCfg_.pollingIntervalMs);
    ImGui::SameLine();
    if (ImGui::Button("+ Add binding")) watchCfg_.bindings.push_back({});
    ImGui::SameLine();
    if (ImGui::Button("Apply")) workflow_.ApplyIoWatch(watchCfg_);
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        workflow_.ApplyIoWatch(watchCfg_);
        if (HardwareConfig::SaveIoWatch(HardwareConfigPath(), watchCfg_))
            FD_INFO("app", "io_watch saved to %s", HardwareConfigPath().c_str());
        else
            FD_ERROR("app", "failed to save io_watch config");
    }
    ImGui::TextDisabled("Save writes config/hardware.json next to the exe; a rebuild overwrites it "
                        "(mirror durable changes into the repo config).");

    // Combo helpers: devices from the IO fleet; pins limited to NAMED pins
    // (bindings reference pins by name).
    auto ioDeviceByName = [this](const std::string& name) -> IIoDevice* {
        for (size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == name) return hardware_.IoDevice(i);
        return nullptr;
    };
    auto deviceCombo = [this](const char* id, std::string& dev) {
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::BeginCombo(id, dev.empty() ? "(device)" : dev.c_str())) {
            for (size_t i = 0; i < hardware_.IoCount(); ++i) {
                const std::string& n = hardware_.IoName(i);
                if (ImGui::Selectable(n.c_str(), n == dev)) dev = n;
            }
            ImGui::EndCombo();
        }
    };
    auto pinCombo = [&](const char* id, const std::string& dev, std::string& pin, bool output) {
        IIoDevice* d = ioDeviceByName(dev);
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo(id, pin.empty() ? "(pin)" : pin.c_str())) {
            if (d) {
                const int count = output ? d->OutputCount() : d->InputCount();
                for (int p = 0; p < count; ++p) {
                    const std::string n = output ? d->OutputPinName(p) : d->InputPinName(p);
                    if (n.empty()) continue;
                    if (ImGui::Selectable(n.c_str(), n == pin)) pin = n;
                }
            }
            ImGui::EndCombo();
        }
    };

    static const char* kEdges[]     = {"rising", "falling", "both"};
    static const char* kBehaviors[] = {"toggle", "set_on", "set_off", "momentary"};
    auto stringCombo = [](const char* id, std::string& val, const char* const* items,
                          int count, float width) {
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo(id, val.c_str())) {
            for (int i = 0; i < count; ++i) {
                if (ImGui::Selectable(items[i], val == items[i])) val = items[i];
            }
            ImGui::EndCombo();
        }
    };

    int deleteRow = -1;
    for (int i = 0; i < static_cast<int>(watchCfg_.bindings.size()); ++i) {
        IoWatchBindingConfig& b = watchCfg_.bindings[i];
        ImGui::PushID(i);
        ImGui::Separator();

        ImGui::Checkbox("##en", &b.enabled);
        ImGui::SameLine();
        char nameBuf[64];
        std::snprintf(nameBuf, sizeof(nameBuf), "%s", b.name.c_str());
        ImGui::SetNextItemWidth(260.0f);
        if (ImGui::InputText("##name", nameBuf, sizeof(nameBuf))) b.name = nameBuf;
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) deleteRow = i;

        ImGui::TextDisabled("in ");
        ImGui::SameLine();
        deviceCombo("##indev", b.inputDevice);
        ImGui::SameLine();
        pinCombo("##inpin", b.inputDevice, b.inputPin, false);
        ImGui::SameLine();
        stringCombo("##edge", b.trigger, kEdges, 3, 90.0f);
        ImGui::SameLine();
        ImGui::TextDisabled("-> out");
        ImGui::SameLine();
        deviceCombo("##outdev", b.outputDevice);
        ImGui::SameLine();
        pinCombo("##outpin", b.outputDevice, b.outputPin, true);
        ImGui::SameLine();
        stringCombo("##beh", b.behavior, kBehaviors, 4, 110.0f);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("ms", &b.debounceMs);

        // Optional indicator: an output (a switch LED) that mirrors the bound
        // output's level, whoever sets it.
        ImGui::TextDisabled("led");
        ImGui::SameLine();
        deviceCombo("##leddev", b.indicatorDevice);
        ImGui::SameLine();
        pinCombo("##ledpin", b.indicatorDevice, b.indicatorPin, true);
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear##led")) {
            b.indicatorDevice.clear();
            b.indicatorPin.clear();
        }

        ImGui::PopID();
    }
    if (deleteRow >= 0) {
        watchCfg_.bindings.erase(watchCfg_.bindings.begin() + deleteRow);
    }
    if (watchCfg_.bindings.empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("no bindings — use \"+ Add binding\"");
    }
}

} // namespace firedog
