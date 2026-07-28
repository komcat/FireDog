// UiSettings.cpp
#include "ui/UiSettings.h"

#include "core/Log.h"

#include <nlohmann/json.hpp>

#include <exception>
#include <fstream>

namespace firedog {

UiSettings UiSettings::Load(const std::string& path)
{
    UiSettings s;
    std::ifstream f(path);
    if (!f.is_open()) return s;   // first run / no file — use defaults
    try {
        nlohmann::json j;
        f >> j;
        if (j.contains("font_index"))  s.fontIndex  = j["font_index"].get<int>();
        if (j.contains("font_size"))   s.fontSize   = j["font_size"].get<float>();
        if (j.contains("theme_index")) s.themeIndex = j["theme_index"].get<int>();
        if (j.contains("left_w"))     s.leftW     = j["left_w"].get<float>();
        if (j.contains("right_w"))    s.rightW    = j["right_w"].get<float>();
        if (j.contains("lower_h"))    s.lowerH    = j["lower_h"].get<float>();
        if (j.contains("show_chart"))    s.showChart   = j["show_chart"].get<bool>();
        if (j.contains("click_to_move")) s.clickToMove = j["click_to_move"].get<bool>();
        if (j.contains("last_product"))  s.lastProduct = j["last_product"].get<std::string>();
        FD_INFO("ui", "settings loaded from %s (font %d, %.0f px)",
                path.c_str(), s.fontIndex, s.fontSize);
    } catch (const std::exception& e) {
        FD_WARN("ui", "settings load failed (%s): using defaults", e.what());
        return UiSettings{};
    }
    return s;
}

bool UiSettings::Save(const std::string& path) const
{
    nlohmann::json j;
    j["font_index"]  = fontIndex;
    j["font_size"]   = fontSize;
    j["theme_index"] = themeIndex;
    j["left_w"]     = leftW;
    j["right_w"]    = rightW;
    j["lower_h"]    = lowerH;
    j["show_chart"]    = showChart;
    j["click_to_move"] = clickToMove;
    j["last_product"]  = lastProduct;

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("ui", "settings save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
