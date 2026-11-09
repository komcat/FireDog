// UiSettings.cpp
#include "ui/UiSettings.h"

#include "core/Log.h"

#include <nlohmann/json.hpp>

#include <exception>
#include <fstream>

namespace firedog {

namespace {

// A view's live ROI filter (core ViewFilter) under its key prefix:
// <prefix>_roi_mode, <prefix>_binary, <prefix>_thr. One pair for every
// filtered view, so a new view adds a field and two calls, never a third
// spelling of the keys.
void LoadViewFilter(const nlohmann::json& j, const std::string& prefix, ViewFilter& s)
{
    if (j.contains(prefix + "_roi_mode")) s.roiMode = j[prefix + "_roi_mode"].get<int>();
    if (j.contains(prefix + "_binary"))   s.binary  = j[prefix + "_binary"].get<bool>();
    if (j.contains(prefix + "_thr"))      s.thr     = j[prefix + "_thr"].get<int>();
    s.Sanitize();
}

void SaveViewFilter(nlohmann::json& j, const std::string& prefix, const ViewFilter& s)
{
    j[prefix + "_roi_mode"] = s.roiMode;
    j[prefix + "_binary"]   = s.binary;
    j[prefix + "_thr"]      = s.thr;
}

} // namespace

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
        if (j.contains("vision_foot_h")) s.visionFootH = j["vision_foot_h"].get<float>();
        if (j.contains("ed_inspect_w")) s.edInspectW = j["ed_inspect_w"].get<float>();
        if (j.contains("ed_palette_w")) s.edPaletteW = j["ed_palette_w"].get<float>();
        if (j.contains("ed_keys_h")) s.edKeysH = j["ed_keys_h"].get<float>();
        if (j.contains("ed_show_hidden")) s.edShowHidden = j["ed_show_hidden"].get<bool>();
        if (j.contains("ed_loose_only")) s.edLooseOnly = j["ed_loose_only"].get<bool>();
        if (j.contains("ed_duration_sec")) s.edDurationSec = j["ed_duration_sec"].get<bool>();
        if (j.contains("ed_tag_filter"))  s.edTagFilter  = j["ed_tag_filter"].get<std::string>();
        if (j.contains("ed_tag_hide"))    s.edTagHide    = j["ed_tag_hide"].get<bool>();
        if (j.contains("run_deck_w")) s.runDeckW = j["run_deck_w"].get<float>();
        if (j.contains("run_dut_w"))  s.runDutW  = j["run_dut_w"].get<float>();
        if (j.contains("run_scan_h")) s.runScanH = j["run_scan_h"].get<float>();
        if (j.contains("run_shot_h")) s.runShotH = j["run_shot_h"].get<float>();
        if (j.contains("org_lib_w"))  s.orgLibW  = j["org_lib_w"].get<float>();
        if (j.contains("org_loose_only")) s.orgLooseOnly = j["org_loose_only"].get<bool>();
        if (j.contains("reg_ctl_w"))  s.regCtlW  = j["reg_ctl_w"].get<float>();
        LoadViewFilter(j, "reg", s.regFilter);
        if (j.contains("reg_exposure_us")) s.regExposureUs = j["reg_exposure_us"].get<double>();
        if (j.contains("reg_light_color")) s.regLightColor = j["reg_light_color"].get<std::string>();
        if (j.contains("reg_light_pct"))   s.regLightPct   = j["reg_light_pct"].get<double>();
        if (j.contains("grip_ctl_w")) s.gripCtlW = j["grip_ctl_w"].get<float>();
        if (j.contains("tray_ctl_w")) s.trayCtlW = j["tray_ctl_w"].get<float>();
        if (j.contains("scripts_list_w")) s.scriptsListW = j["scripts_list_w"].get<float>();
        if (j.contains("show_tray_win")) s.showTrayWin = j["show_tray_win"].get<bool>();
        if (j.contains("mosaic_left_w"))  s.mosaicLeftW  = j["mosaic_left_w"].get<float>();
        if (j.contains("mosaic_right_w")) s.mosaicRightW = j["mosaic_right_w"].get<float>();
        if (j.contains("show_chart"))    s.showChart   = j["show_chart"].get<bool>();
        if (j.contains("click_to_move")) s.clickToMove = j["click_to_move"].get<bool>();
        if (j.contains("show_vision_marks")) s.showVisionMarks = j["show_vision_marks"].get<bool>();
        if (j.contains("show_pos_labels")) s.showPosLabels = j["show_pos_labels"].get<bool>();
        if (j.contains("pos_labels_hidden")) s.posLabelsHidden = j["pos_labels_hidden"].get<bool>();
        if (j.contains("show_gripper_marks")) s.showGripperMarks = j["show_gripper_marks"].get<bool>();
        if (j.contains("aux_panel_open")) s.auxPanelOpen = j["aux_panel_open"].get<bool>();
        if (j.contains("aux_img_fit"))    s.auxImgFit    = j["aux_img_fit"].get<bool>();
        if (j.contains("aux_chart_compact")) s.auxChartCompact = j["aux_chart_compact"].get<bool>();
        if (j.contains("chart_scope"))   s.chartScope  = j["chart_scope"].get<bool>();
        if (j.contains("chart_smooth"))  s.chartSmooth = j["chart_smooth"].get<int>();
        if (j.contains("chart_y_bands_on")) s.chartYBands = j["chart_y_bands_on"].get<bool>();
        if (j.contains("chart_y_bands") && j["chart_y_bands"].is_array()) {
            // A saved list replaces the defaults whole; malformed rows are
            // dropped, and an empty result keeps the defaults.
            std::vector<ChartYBand> bands;
            for (const auto& v : j["chart_y_bands"]) {
                if (bands.size() >= static_cast<std::size_t>(kMaxChartYBands)) break;
                if (!v.is_object()) continue;
                ChartYBand b;
                b.loUa = v.value("lo_ua", 0.0);
                b.hiUa = v.value("hi_ua", 0.0);
                if (b.hiUa > b.loUa) bands.push_back(b);
            }
            if (!bands.empty()) s.chartYBandList = bands;
        }
        if (j.contains("chart_ch_default") && j["chart_ch_default"].is_number_integer()) {
            const int ch = j["chart_ch_default"].get<int>();
            s.chartChDefault = (ch >= 0 && ch <= 2) ? ch : 2;
        }
        if (j.contains("aux_tip_labels") && j["aux_tip_labels"].is_object()) {
            // Replaces the defaults whole, so a label cleared to "" stays clear.
            s.auxTipLabels.clear();
            for (auto it = j["aux_tip_labels"].begin(); it != j["aux_tip_labels"].end(); ++it)
                if (it.value().is_string())
                    s.auxTipLabels[it.key()] =
                        it.value().get<std::string>().substr(0, kAuxTipLabelMax);
        }
        if (j.contains("jog_reg_zoff_default") && j["jog_reg_zoff_default"].is_number())
            s.jogRegZOffDefault = j["jog_reg_zoff_default"].get<double>();
        if (j.contains("jog_reg_unverified_default"))
            s.jogRegUnverDefault = j["jog_reg_unverified_default"].get<bool>();
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
        if (j.contains("run_vals_user_only"))
            s.runValsUserOnly = j["run_vals_user_only"].get<bool>();
        if (j.contains("data_hidden_keys") && j["data_hidden_keys"].is_array())
            for (const auto& v : j["data_hidden_keys"])
                if (v.is_string()) s.dataHiddenKeys.push_back(v.get<std::string>());
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
        // PSU presets: per field, so a missing key keeps that preset's default.
        if (j.contains("psu_presets") && j["psu_presets"].is_array()) {
            const auto& a = j["psu_presets"];
            for (std::size_t i = 0; i < a.size() && i < s.psuPresets.size(); ++i) {
                if (!a[i].is_object()) continue;
                PsuPreset& p = s.psuPresets[i];
                p.channel = a[i].value("ch", p.channel);
                p.mode    = a[i].value("mode", p.mode) == 0 ? 0 : 1;
                p.volts   = a[i].value("v", p.volts);
                p.amps    = a[i].value("a", p.amps);
            }
        }
        if (j.contains("dyn_acc_per_vel") && j["dyn_acc_per_vel"].is_number())
            s.dynAccPerVel = j["dyn_acc_per_vel"].get<double>();
        if (j.contains("dyn_jerk_per_acc") && j["dyn_jerk_per_acc"].is_number())
            s.dynJerkPerAcc = j["dyn_jerk_per_acc"].get<double>();
        // Needle "clear first": the whole list, so an emptied one stays empty.
        if (j.contains("needle_clear_moves") && j["needle_clear_moves"].is_array()) {
            s.ndlClearMoves.clear();
            for (const auto& m : j["needle_clear_moves"]) {
                if (!m.is_object()) continue;
                s.ndlClearMoves.push_back({m.value("device", std::string()),
                                           m.value("position", std::string())});
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
    j["vision_foot_h"]   = visionFootH;
    j["ed_inspect_w"]    = edInspectW;
    j["ed_palette_w"]    = edPaletteW;
    j["ed_keys_h"]       = edKeysH;
    j["ed_show_hidden"]  = edShowHidden;
    j["ed_loose_only"]   = edLooseOnly;
    j["ed_duration_sec"] = edDurationSec;
    j["ed_tag_filter"]   = edTagFilter;
    j["ed_tag_hide"]     = edTagHide;
    j["run_deck_w"]      = runDeckW;
    j["run_dut_w"]       = runDutW;
    j["run_scan_h"]      = runScanH;
    j["run_shot_h"]      = runShotH;
    j["org_lib_w"]       = orgLibW;
    j["org_loose_only"]  = orgLooseOnly;
    j["reg_ctl_w"]       = regCtlW;
    SaveViewFilter(j, "reg", regFilter);
    j["reg_exposure_us"] = regExposureUs;
    j["reg_light_color"] = regLightColor;
    j["reg_light_pct"]   = regLightPct;
    j["grip_ctl_w"]      = gripCtlW;
    j["tray_ctl_w"]      = trayCtlW;
    j["scripts_list_w"]  = scriptsListW;
    j["show_tray_win"]   = showTrayWin;
    j["mosaic_left_w"]   = mosaicLeftW;
    j["mosaic_right_w"]  = mosaicRightW;
    j["show_chart"]    = showChart;
    j["click_to_move"] = clickToMove;
    j["show_vision_marks"] = showVisionMarks;
    j["show_pos_labels"]   = showPosLabels;
    j["pos_labels_hidden"]  = posLabelsHidden;
    j["show_gripper_marks"] = showGripperMarks;
    j["aux_panel_open"]     = auxPanelOpen;
    j["aux_img_fit"]        = auxImgFit;
    j["aux_chart_compact"]  = auxChartCompact;
    j["chart_scope"]   = chartScope;
    j["chart_smooth"]  = chartSmooth;
    j["chart_y_bands_on"] = chartYBands;
    nlohmann::json yb = nlohmann::json::array();
    for (const ChartYBand& b : chartYBandList)
        yb.push_back({{"lo_ua", b.loUa}, {"hi_ua", b.hiUa}});
    j["chart_y_bands"] = yb;
    j["chart_ch_default"]           = chartChDefault;
    nlohmann::json tl = nlohmann::json::object();
    for (const auto& kv : auxTipLabels) tl[kv.first] = kv.second;
    j["aux_tip_labels"] = tl;
    j["jog_reg_zoff_default"]       = jogRegZOffDefault;
    j["jog_reg_unverified_default"] = jogRegUnverDefault;
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
    j["run_vals_user_only"] = runValsUserOnly;
    j["data_hidden_keys"]   = dataHiddenKeys;
    nlohmann::json pad = nlohmann::json::object();
    for (const auto& kv : posPad) pad[kv.first] = kv.second;
    j["pos_pad"] = pad;
    nlohmann::json qio = nlohmann::json::array();
    for (const QuickIoRow& r : quickIo)
        qio.push_back({{"device", r.device}, {"pin", r.pin}, {"ms", r.dwellMs}});
    j["quick_io"] = qio;
    nlohmann::json pp = nlohmann::json::array();
    for (const PsuPreset& p : psuPresets)
        pp.push_back({{"ch", p.channel}, {"mode", p.mode}, {"v", p.volts}, {"a", p.amps}});
    j["psu_presets"] = pp;
    nlohmann::json cm = nlohmann::json::array();
    for (const NeedleClearMove& m : ndlClearMoves)
        cm.push_back({{"device", m.device}, {"position", m.position}});
    j["needle_clear_moves"] = cm;
    j["dyn_acc_per_vel"]    = dynAccPerVel;
    j["dyn_jerk_per_acc"]   = dynJerkPerAcc;

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("ui", "settings save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
