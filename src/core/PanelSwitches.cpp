// PanelSwitches.cpp : the panel switch config loader. See PanelSwitches.h.
#include "core/PanelSwitches.h"

#include "core/Log.h"
#include "core/Paths.h"

#include <nlohmann/json.hpp>

#include <exception>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace firedog {

namespace {

std::string ConfigPath() { return MachinePath("config/panel_switches.local.json"); }

nlohmann::json ToJson(const PanelSwitchConfig& c)
{
    return {{"enabled", c.enabled},
            {"device", c.device},
            {"confirm", c.confirm},
            {"cancel", c.cancel},
            {"run", c.run},
            {"stop", c.stop},
            {"debounce_ms", c.debounceMs},
            {"min_shown_ms", c.minShownMs}};
}

} // namespace

PanelSwitchConfig LoadPanelSwitchConfig()
{
    PanelSwitchConfig c;
    const std::string path = ConfigPath();
    std::ifstream f(path);
    if (!f.is_open()) {
        // First start: leave an editable file behind, holding the defaults.
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
        std::ofstream o(path);
        if (o.is_open()) {
            o << ToJson(c).dump(2) << '\n';
            FD_INFO("panel", "wrote the default panel switch config to %s", path.c_str());
        }
        return c;
    }
    try {
        nlohmann::json j;
        f >> j;
        c.enabled    = j.value("enabled", c.enabled);
        c.device     = j.value("device", c.device);
        c.confirm    = j.value("confirm", c.confirm);
        c.cancel     = j.value("cancel", c.cancel);
        c.run        = j.value("run", c.run);
        c.stop       = j.value("stop", c.stop);
        c.debounceMs = j.value("debounce_ms", c.debounceMs);
        c.minShownMs = j.value("min_shown_ms", c.minShownMs);
    } catch (const std::exception& e) {
        FD_WARN("panel", "%s is malformed (%s) - using the defaults", path.c_str(), e.what());
        return PanelSwitchConfig{};
    }
    if (c.debounceMs < 0) c.debounceMs = 0;
    if (c.minShownMs < 0) c.minShownMs = 0;
    return c;
}

} // namespace firedog
