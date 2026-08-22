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
        if (j.contains("vision_param_w")) s.visionParamW = j["vision_param_w"].get<float>();
        if (j.contains("vision_result_w")) s.visionResultW = j["vision_result_w"].get<float>();
        if (j.contains("ed_inspect_w")) s.edInspectW = j["ed_inspect_w"].get<float>();
        if (j.contains("run_deck_w")) s.runDeckW = j["run_deck_w"].get<float>();
        if (j.contains("run_dut_w"))  s.runDutW  = j["run_dut_w"].get<float>();
        if (j.contains("run_scan_h")) s.runScanH = j["run_scan_h"].get<float>();
        if (j.contains("run_shot_h")) s.runShotH = j["run_shot_h"].get<float>();
        if (j.contains("org_lib_w"))  s.orgLibW  = j["org_lib_w"].get<float>();
        if (j.contains("show_chart"))    s.showChart   = j["show_chart"].get<bool>();
        if (j.contains("click_to_move")) s.clickToMove = j["click_to_move"].get<bool>();
        if (j.contains("show_vision_marks")) s.showVisionMarks = j["show_vision_marks"].get<bool>();
        if (j.contains("show_pos_labels")) s.showPosLabels = j["show_pos_labels"].get<bool>();
        if (j.contains("pos_labels_hidden")) s.posLabelsHidden = j["pos_labels_hidden"].get<bool>();
        if (j.contains("chart_scope"))   s.chartScope  = j["chart_scope"].get<bool>();
        if (j.contains("run_prog_uid")) s.runProgUid = j["run_prog_uid"].get<std::string>();
        if (j.contains("run_slot"))     s.runSlot    = j["run_slot"].get<int>();
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
    j["vision_param_w"]  = visionParamW;
    j["vision_result_w"] = visionResultW;
    j["ed_inspect_w"]    = edInspectW;
    j["run_deck_w"]      = runDeckW;
    j["run_dut_w"]       = runDutW;
    j["run_scan_h"]      = runScanH;
    j["run_shot_h"]      = runShotH;
    j["org_lib_w"]       = orgLibW;
    j["show_chart"]    = showChart;
    j["click_to_move"] = clickToMove;
    j["show_vision_marks"] = showVisionMarks;
    j["show_pos_labels"]   = showPosLabels;
    j["pos_labels_hidden"]  = posLabelsHidden;
    j["chart_scope"]   = chartScope;
    j["run_prog_uid"]  = runProgUid;
    j["run_slot"]      = runSlot;
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
