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
        if (j.contains("ed_palette_w")) s.edPaletteW = j["ed_palette_w"].get<float>();
        if (j.contains("ed_show_hidden")) s.edShowHidden = j["ed_show_hidden"].get<bool>();
        if (j.contains("ed_tag_filter"))  s.edTagFilter  = j["ed_tag_filter"].get<std::string>();
        if (j.contains("ed_tag_hide"))    s.edTagHide    = j["ed_tag_hide"].get<bool>();
        if (j.contains("run_deck_w")) s.runDeckW = j["run_deck_w"].get<float>();
        if (j.contains("run_dut_w"))  s.runDutW  = j["run_dut_w"].get<float>();
        if (j.contains("run_scan_h")) s.runScanH = j["run_scan_h"].get<float>();
        if (j.contains("run_shot_h")) s.runShotH = j["run_shot_h"].get<float>();
        if (j.contains("org_lib_w"))  s.orgLibW  = j["org_lib_w"].get<float>();
        if (j.contains("reg_ctl_w"))  s.regCtlW  = j["reg_ctl_w"].get<float>();
        if (j.contains("grip_ctl_w")) s.gripCtlW = j["grip_ctl_w"].get<float>();
        if (j.contains("tray_ctl_w")) s.trayCtlW = j["tray_ctl_w"].get<float>();
        if (j.contains("show_chart"))    s.showChart   = j["show_chart"].get<bool>();
        if (j.contains("click_to_move")) s.clickToMove = j["click_to_move"].get<bool>();
        if (j.contains("show_vision_marks")) s.showVisionMarks = j["show_vision_marks"].get<bool>();
        if (j.contains("show_pos_labels")) s.showPosLabels = j["show_pos_labels"].get<bool>();
        if (j.contains("pos_labels_hidden")) s.posLabelsHidden = j["pos_labels_hidden"].get<bool>();
        if (j.contains("nom_box_w")) s.nomBoxW = j["nom_box_w"].get<float>();
        if (j.contains("nom_box_h")) s.nomBoxH = j["nom_box_h"].get<float>();
        if (j.contains("show_gripper_marks")) s.showGripperMarks = j["show_gripper_marks"].get<bool>();
        if (j.contains("aux_panel_open")) s.auxPanelOpen = j["aux_panel_open"].get<bool>();
        if (j.contains("chart_scope"))   s.chartScope  = j["chart_scope"].get<bool>();
        if (j.contains("run_prog_uid")) s.runProgUid = j["run_prog_uid"].get<std::string>();
        if (j.contains("run_slot"))     s.runSlot    = j["run_slot"].get<int>();
        if (j.contains("win_x")) s.winX = j["win_x"].get<int>();
        if (j.contains("win_y")) s.winY = j["win_y"].get<int>();
        if (j.contains("win_w")) s.winW = j["win_w"].get<int>();
        if (j.contains("win_h")) s.winH = j["win_h"].get<int>();
        if (j.contains("win_max")) s.winMaximized = j["win_max"].get<bool>();
        if (j.contains("last_product"))  s.lastProduct = j["last_product"].get<std::string>();
        // Pos pad: tolerant on every axis — a non-object, a non-array value or
        // a short/long row all degrade to blank slots rather than throwing away
        // the whole settings file.
        // Readouts: an object per row; a bare string (the first version of the
        // field) is a topic with no limit.
        if (j.contains("run_readouts") && j["run_readouts"].is_array()) {
            for (const auto& v : j["run_readouts"]) {
                if (s.runReadouts.size() >= 4) break;
                RunReadout r;
                if (v.is_string()) {
                    r.topic = v.get<std::string>();
                } else if (v.is_object()) {
                    r.topic = v.value("topic", std::string());
                    r.mode  = v.value("mode", std::string("none"));
                    r.lo    = v.value("lo", 0.0);
                    r.hi    = v.value("hi", 0.0);
                }
                if (!r.topic.empty()) s.runReadouts.push_back(std::move(r));
            }
        }
        if (j.contains("pos_pad") && j["pos_pad"].is_object()) {
            for (auto it = j["pos_pad"].begin(); it != j["pos_pad"].end(); ++it) {
                if (!it.value().is_array()) continue;
                std::vector<std::string> slots;
                for (const auto& v : it.value())
                    slots.push_back(v.is_string() ? v.get<std::string>() : std::string());
                s.posPad[it.key()] = std::move(slots);
            }
        }
        // Quick IO rows: same tolerance - a malformed entry becomes a blank
        // row the operator can reassign, never a thrown-away file.
        if (j.contains("quick_io") && j["quick_io"].is_array()) {
            for (const auto& e : j["quick_io"]) {
                QuickIoRow r;
                if (e.is_object()) {
                    if (e.contains("device") && e["device"].is_string()) r.device  = e["device"].get<std::string>();
                    if (e.contains("pin") && e["pin"].is_string())       r.pin     = e["pin"].get<std::string>();
                    if (e.contains("ms") && e["ms"].is_number())         r.dwellMs = e["ms"].get<int>();
                    if (r.dwellMs < 0) r.dwellMs = 0;
                }
                s.quickIo.push_back(std::move(r));
            }
        }
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
    j["ed_palette_w"]    = edPaletteW;
    j["ed_show_hidden"]  = edShowHidden;
    j["ed_tag_filter"]   = edTagFilter;
    j["ed_tag_hide"]     = edTagHide;
    j["run_deck_w"]      = runDeckW;
    j["run_dut_w"]       = runDutW;
    j["run_scan_h"]      = runScanH;
    j["run_shot_h"]      = runShotH;
    j["org_lib_w"]       = orgLibW;
    j["reg_ctl_w"]       = regCtlW;
    j["grip_ctl_w"]      = gripCtlW;
    j["tray_ctl_w"]      = trayCtlW;
    j["show_chart"]    = showChart;
    j["click_to_move"] = clickToMove;
    j["show_vision_marks"] = showVisionMarks;
    j["show_pos_labels"]   = showPosLabels;
    j["pos_labels_hidden"]  = posLabelsHidden;
    j["nom_box_w"]          = nomBoxW;
    j["nom_box_h"]          = nomBoxH;
    j["show_gripper_marks"] = showGripperMarks;
    j["aux_panel_open"]     = auxPanelOpen;
    j["chart_scope"]   = chartScope;
    j["run_prog_uid"]  = runProgUid;
    j["run_slot"]      = runSlot;
    j["win_x"]   = winX;
    j["win_y"]   = winY;
    j["win_w"]   = winW;
    j["win_h"]   = winH;
    j["win_max"] = winMaximized;
    j["last_product"]  = lastProduct;
    nlohmann::json ro = nlohmann::json::array();
    for (const RunReadout& r : runReadouts)
        ro.push_back({{"topic", r.topic}, {"mode", r.mode}, {"lo", r.lo}, {"hi", r.hi}});
    j["run_readouts"]  = ro;
    nlohmann::json pad = nlohmann::json::object();
    for (const auto& kv : posPad) pad[kv.first] = kv.second;
    j["pos_pad"] = pad;
    nlohmann::json qio = nlohmann::json::array();
    for (const QuickIoRow& r : quickIo)
        qio.push_back({{"device", r.device}, {"pin", r.pin}, {"ms", r.dwellMs}});
    j["quick_io"] = qio;

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("ui", "settings save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
