// WndPositionRecords.cpp : the floating Position Records window (View menu).
// Definitions of App members - see App.h.
//
// VIEW ONLY. The recipe step `record_position` appends one pose per unit run
// to <product data>/position_records/<name>.jsonl (core/PositionRecords.h);
// this window reads the tail of one name's file and shows where those poses
// fall: per-axis statistics, histograms, the trend over time (or unit), an X-Y
// scatter and the records themselves - over the last week, the last month,
// the last N units or everything loaded. Nothing here writes, moves or saves a
// position: the median it shows is a candidate for a person to judge, not a
// target. The rows are re-read when the product, the name or the append
// counter changes, so a run that records is visible on the next frame - never
// per frame. docs/position-records-design.md.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "core/PositionRecords.h"
#include "ui/Widgets.h"   // OpenFolderInExplorer

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <set>
#include <string>
#include <vector>

namespace firedog {

namespace {

// X Y Z U V W first, in that order; anything else a controller names after.
int AxisRank(const std::string& a)
{
    static const char* kOrder[] = {"X", "Y", "Z", "U", "V", "W"};
    for (int i = 0; i < 6; ++i)
        if (a == kOrder[i]) return i;
    return 6;
}

bool IsRotAxis(const std::string& a) { return a == "U" || a == "V" || a == "W"; }

const char* AxisUnit(const std::string& a)
{
    if (a == "X" || a == "Y" || a == "Z") return "mm";
    if (IsRotAxis(a)) return "deg";
    return "";
}

// Seconds to add to a UTC unix time so ImPlot's (UTC) time axis reads as local
// wall-clock time - without flipping ImPlot's global UseLocalTime style.
double LocalOffsetSeconds()
{
    const std::time_t now = std::time(nullptr);
    std::tm g{};
    gmtime_s(&g, &now);
    g.tm_isdst = -1;
    return static_cast<double>(now - std::mktime(&g));
}

const char* const kRangeNames[] = {"All loaded", "Last week", "Last month", "Last N units"};

} // namespace

void App::LoadPositionRecordsView(const std::string& dir)
{
    posRecReload_ = false;
    posRecDir_    = dir;
    posRecSeq_    = PositionRecordsSeq();   // before reading: a later append reloads again
    posRecNames_  = ListPositionRecordNames(dir);
    posRecRows_.clear();
    posRecSel_ = -1;
    posRecMsg_.clear();
    if (std::find(posRecNames_.begin(), posRecNames_.end(), posRecName_) == posRecNames_.end())
        posRecName_ = posRecNames_.empty() ? std::string() : posRecNames_.front();
    if (posRecName_.empty()) return;

    std::size_t bad = 0;
    std::string err;
    if (!LoadPositionRecords(dir, posRecName_, static_cast<std::size_t>(std::max(posRecTail_, 1)),
                             posRecRows_, &bad, &err)) {
        posRecMsg_ = "load failed: " + err;
        return;
    }
    if (bad > 0)
        posRecMsg_ = std::to_string(bad) + " unreadable line(s) skipped";
}

void App::DrawPositionRecordsWindow()
{
    // The menu item is gated, but the open bit outlives a page switch.
    if (!CanUse(kRolePosRecords)) { showPosRec_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(960.0f, 720.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Position Records", &showPosRec_)) {
        ImGui::End();
        return;
    }

    // Re-read when the product under the window changes, a record was appended
    // anywhere, or the toolbar asked for it - a view cache, not the document.
    const std::string dir = session_.ProductLoaded()
                                ? PositionRecordsDir(session_.Product().DataDir())
                                : std::string();
    if (dir != posRecDir_ || posRecSeq_ != PositionRecordsSeq() || posRecReload_)
        LoadPositionRecordsView(dir);

    if (dir.empty()) {
        ImGui::TextDisabled("No product loaded.");
        ImGui::End();
        return;
    }

    // ---- Toolbar ----
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::BeginCombo("Name", posRecName_.empty() ? "(none)" : posRecName_.c_str())) {
        for (const std::string& n : posRecNames_) {
            if (ImGui::Selectable(n.c_str(), n == posRecName_)) {
                posRecName_   = n;
                posRecReload_ = true;
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("One history per name - the name a record_position step was given.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(130.0f);
    ImGui::Combo("Range", &posRecRange_, kRangeNames, IM_ARRAYSIZE(kRangeNames));
    if (posRecRange_ == static_cast<int>(RecordRange::LastN)) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        if (ImGui::InputInt("units", &posRecN_, 10, 100)) posRecN_ = std::max(posRecN_, 1);
    }
    ImGui::SameLine();
    ImGui::Checkbox("Show U V W", &posRecUvw_);
    ImGui::SameLine();
    if (ImGui::Button("Reload")) posRecReload_ = true;
    ImGui::SameLine();
    if (ImGui::Button("Open folder")) OpenFolderInExplorer(dir);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", dir.c_str());

    ImGui::SetNextItemWidth(110.0f);
    if (ImGui::InputInt("Load last", &posRecTail_, 1000, 10000)) {
        posRecTail_   = std::clamp(posRecTail_, 10, 1000000);
        posRecReload_ = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How many records (newest) are read from the file.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    ImGui::SliderInt("Bins", &posRecBins_, 5, 100);

    if (posRecNames_.empty()) {
        ImGui::Separator();
        ImGui::TextWrapped("No position records for this product yet. Add a record_position "
                           "step after the step whose end pose should be kept (e.g. after an "
                           "xcan) - every unit run that passes it adds one record.");
        ImGui::End();
        return;
    }

    const std::vector<std::size_t> shown = FilterPositionRecords(
        posRecRows_, static_cast<RecordRange>(posRecRange_), posRecN_,
        static_cast<std::int64_t>(std::time(nullptr)));

    // The axes on show: every axis the shown records carry, U V W on request.
    std::set<std::string> devices;
    std::vector<std::string> axes;
    {
        std::set<std::string> seen;
        for (const std::size_t i : shown) {
            devices.insert(posRecRows_[i].device);
            for (const auto& [a, v] : posRecRows_[i].pose) seen.insert(a);
        }
        for (const std::string& a : seen)
            if (posRecUvw_ || !IsRotAxis(a)) axes.push_back(a);
        std::stable_sort(axes.begin(), axes.end(), [](const std::string& a, const std::string& b) {
            return AxisRank(a) < AxisRank(b);
        });
    }

    ImGui::Text("%zu shown of %zu loaded", shown.size(), posRecRows_.size());
    if (!devices.empty()) {
        ImGui::SameLine();
        std::string devs;
        for (const std::string& d : devices) devs += (devs.empty() ? "" : ", ") + d;
        if (devices.size() > 1)
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                               "  devices: %s  (MIXED - one name should be one device)",
                               devs.c_str());
        else
            ImGui::TextDisabled("  device: %s", devs.c_str());
    }
    if (!posRecMsg_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "  %s", posRecMsg_.c_str());
    }
    if (posRecSel_ >= static_cast<int>(posRecRows_.size())) posRecSel_ = -1;

    // Per-axis values of the shown records (records lacking an axis skip it).
    std::vector<std::vector<double>> vals(axes.size());
    std::vector<std::vector<double>> times(axes.size());
    std::vector<AxisStats> stats(axes.size());
    const double tz = LocalOffsetSeconds();
    for (std::size_t a = 0; a < axes.size(); ++a) {
        for (std::size_t k = 0; k < shown.size(); ++k) {
            const PositionRecord& r = posRecRows_[shown[k]];
            const auto it = r.pose.find(axes[a]);
            if (it == r.pose.end()) continue;
            vals[a].push_back(it->second);
            times[a].push_back(posRecTrendTime_ ? static_cast<double>(r.t) + tz
                                                : static_cast<double>(k));
        }
        stats[a] = ComputeAxisStats(vals[a]);
    }
    const PositionRecord* sel =
        posRecSel_ >= 0 ? &posRecRows_[static_cast<std::size_t>(posRecSel_)] : nullptr;

    // ---- Statistics ----
    ImGui::Separator();
    if (ImGui::BeginTable("##posrecstats", 8,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingFixedFit)) {
        for (const char* h : {"Axis", "n", "Median", "Mean", "Std dev", "Min", "Max", "Range"})
            ImGui::TableSetupColumn(h);
        ImGui::TableHeadersRow();
        for (std::size_t a = 0; a < axes.size(); ++a) {
            const AxisStats& s = stats[a];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s %s", axes[a].c_str(), AxisUnit(axes[a]));
            ImGui::TableNextColumn();
            ImGui::Text("%d", s.n);
            if (s.n == 0) continue;
            for (const double v : {s.median, s.mean, s.stdev, s.min, s.max, s.max - s.min}) {
                ImGui::TableNextColumn();
                ImGui::Text("%.4f", v);
            }
        }
        ImGui::EndTable();
    }
    if (ImGui::Button("Copy medians")) {
        std::string t;
        for (std::size_t a = 0; a < axes.size(); ++a) {
            if (stats[a].n == 0) continue;
            char b[64];
            std::snprintf(b, sizeof(b), "%s%s=%.4f", t.empty() ? "" : " ", axes[a].c_str(),
                          stats[a].median);
            t += b;
        }
        ImGui::SetClipboardText(t.c_str());
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Copy the per-axis medians of the shown records as text.");

    // ---- Charts / records ----
    const ImVec4 kSelCol(1.0f, 0.85f, 0.3f, 0.9f);
    const ImVec4 kMedCol(0.3f, 0.9f, 0.5f, 0.9f);
    if (ImGui::BeginTabBar("##posrectabs")) {
        if (ImGui::BeginTabItem("Histograms")) {
            const int n    = static_cast<int>(axes.size());
            const int cols = std::max(1, std::min(n, 3));
            const int rows = std::max(1, (n + cols - 1) / cols);
            if (n > 0 &&
                ImPlot::BeginSubplots("##posrechist", rows, cols, ImVec2(-1.0f, -1.0f))) {
                for (std::size_t a = 0; a < axes.size(); ++a) {
                    char title[48];
                    std::snprintf(title, sizeof(title), "%s (%s)##h%zu", axes[a].c_str(),
                                  AxisUnit(axes[a]), a);
                    if (!ImPlot::BeginPlot(title, ImVec2(-1.0f, -1.0f), ImPlotFlags_NoMenus))
                        continue;
                    ImPlot::SetupAxes(nullptr, "count", ImPlotAxisFlags_AutoFit,
                                      ImPlotAxisFlags_AutoFit);
                    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
                    if (!vals[a].empty()) {
                        ImPlotSpec hs;
                        hs.FillAlpha = 0.7f;
                        ImPlot::PlotHistogram("records", vals[a].data(),
                                              static_cast<int>(vals[a].size()), posRecBins_,
                                              1.0, ImPlotRange(), hs);
                        ImPlotSpec ms;
                        ms.LineColor  = kMedCol;
                        ms.LineWeight = 2.0f;
                        ImPlot::PlotInfLines("median", &stats[a].median, 1, ms);
                        if (sel) {
                            const auto it = sel->pose.find(axes[a]);
                            if (it != sel->pose.end()) {
                                ImPlotSpec ss;
                                ss.LineColor = kSelCol;
                                ImPlot::PlotInfLines("selected", &it->second, 1, ss);
                            }
                        }
                    }
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Trend")) {
            ImGui::Checkbox("Time axis", &posRecTrendTime_);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Off: one step per unit, oldest to newest.");
            const int n = static_cast<int>(axes.size());
            if (n > 0 && ImPlot::BeginSubplots("##posrectrend", n, 1, ImVec2(-1.0f, -1.0f),
                                               ImPlotSubplotFlags_LinkAllX)) {
                for (std::size_t a = 0; a < axes.size(); ++a) {
                    char title[48];
                    std::snprintf(title, sizeof(title), "##t%zu", a);
                    if (!ImPlot::BeginPlot(title, ImVec2(-1.0f, -1.0f), ImPlotFlags_NoMenus |
                                                                            ImPlotFlags_NoLegend))
                        continue;
                    char ylab[24];
                    std::snprintf(ylab, sizeof(ylab), "%s %s", axes[a].c_str(), AxisUnit(axes[a]));
                    ImPlot::SetupAxes(nullptr, ylab, ImPlotAxisFlags_AutoFit,
                                      ImPlotAxisFlags_AutoFit);
                    if (posRecTrendTime_) ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Time);
                    if (!vals[a].empty()) {
                        ImPlotSpec ls;
                        ls.Marker     = ImPlotMarker_Circle;
                        ls.MarkerSize = 3.0f;
                        ImPlot::PlotLine(axes[a].c_str(), times[a].data(), vals[a].data(),
                                         static_cast<int>(vals[a].size()), ls);
                        ImPlotSpec ms;
                        ms.LineColor = kMedCol;
                        ms.Flags     = ImPlotInfLinesFlags_Horizontal;
                        ImPlot::PlotInfLines("median", &stats[a].median, 1, ms);
                    }
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("X-Y")) {
            std::vector<double> xs, ys;
            for (const std::size_t i : shown) {
                const auto ix = posRecRows_[i].pose.find("X");
                const auto iy = posRecRows_[i].pose.find("Y");
                if (ix == posRecRows_[i].pose.end() || iy == posRecRows_[i].pose.end()) continue;
                xs.push_back(ix->second);
                ys.push_back(iy->second);
            }
            if (xs.empty()) {
                ImGui::TextDisabled("The shown records carry no X and Y.");
            } else if (ImPlot::BeginPlot("##posrecxy", ImVec2(-1.0f, -1.0f),
                                         ImPlotFlags_NoMenus | ImPlotFlags_Equal)) {
                ImPlot::SetupAxes("X mm", "Y mm", ImPlotAxisFlags_AutoFit,
                                  ImPlotAxisFlags_AutoFit);
                ImPlot::SetupLegend(ImPlotLocation_NorthEast);
                ImPlotSpec ps;
                ps.Marker     = ImPlotMarker_Circle;
                ps.MarkerSize = 3.0f;
                ps.FillAlpha  = 0.6f;
                ImPlot::PlotScatter("records", xs.data(), ys.data(), static_cast<int>(xs.size()), ps);
                const AxisStats sx = ComputeAxisStats(xs), sy = ComputeAxisStats(ys);
                ImPlotSpec ms;
                ms.Marker     = ImPlotMarker_Cross;
                ms.MarkerSize = 9.0f;
                ms.LineColor  = kMedCol;
                ms.LineWeight = 2.0f;
                ImPlot::PlotScatter("median", &sx.median, &sy.median, 1, ms);
                ImPlotSpec ln;
                ln.Marker     = ImPlotMarker_Diamond;
                ln.MarkerSize = 6.0f;
                ImPlot::PlotScatter("newest", &xs.back(), &ys.back(), 1, ln);
                if (sel) {
                    const auto ix = sel->pose.find("X");
                    const auto iy = sel->pose.find("Y");
                    if (ix != sel->pose.end() && iy != sel->pose.end()) {
                        ImPlotSpec ss;
                        ss.Marker     = ImPlotMarker_Square;
                        ss.MarkerSize = 6.0f;
                        ss.LineColor  = kSelCol;
                        ImPlot::PlotScatter("selected", &ix->second, &iy->second, 1, ss);
                    }
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Records")) {
            const int ncol = 4 + static_cast<int>(axes.size());
            if (ImGui::BeginTable("##posrecrows", ncol,
                                  ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX |
                                      ImGuiTableFlags_SizingFixedFit,
                                  ImVec2(-1.0f, -1.0f))) {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Time");
                ImGui::TableSetupColumn("Serial");
                ImGui::TableSetupColumn("Operator");
                ImGui::TableSetupColumn("Device");
                for (const std::string& a : axes) ImGui::TableSetupColumn(a.c_str());
                ImGui::TableHeadersRow();

                ImGuiListClipper clip;   // newest first
                clip.Begin(static_cast<int>(shown.size()));
                while (clip.Step()) {
                    for (int row = clip.DisplayStart; row < clip.DisplayEnd; ++row) {
                        const std::size_t i =
                            shown[shown.size() - 1 - static_cast<std::size_t>(row)];
                        const PositionRecord& r = posRecRows_[i];
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::PushID(static_cast<int>(i));
                        if (ImGui::Selectable(r.ts.c_str(), posRecSel_ == static_cast<int>(i),
                                              ImGuiSelectableFlags_SpanAllColumns))
                            posRecSel_ = posRecSel_ == static_cast<int>(i) ? -1 : static_cast<int>(i);
                        ImGui::PopID();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(r.serial.empty() ? "-" : r.serial.c_str());
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(r.operatorName.empty()
                                                   ? (r.operatorId.empty() ? "-" : r.operatorId.c_str())
                                                   : r.operatorName.c_str());
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(r.device.c_str());
                        for (const std::string& a : axes) {
                            ImGui::TableNextColumn();
                            const auto it = r.pose.find(a);
                            if (it == r.pose.end()) ImGui::TextDisabled("-");
                            else                    ImGui::Text("%.4f", it->second);
                        }
                    }
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

} // namespace firedog
