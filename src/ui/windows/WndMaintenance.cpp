// WndMaintenance.cpp : the maintenance history — every DOWN -> UP entry over a
// date range, with a free-text search, a summary (count, total, MTTR, longest,
// time by category) and CSV export. Read-only; opened from the OEE page.
// Definitions of App members — see App.h. Design: docs/oee-design.md.
//
// Same level as the OEE page (it reads the same log). The export's "Open
// folder" is Technician, like every Explorer shortcut: on an Operator's
// station it would be the one control that leaves the application.
#include "App.h"

#include "imgui.h"

#include "core/OeeTracker.h"
#include "core/Paths.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <system_error>
#include <vector>

namespace firedog {

namespace {

std::string Csv(const std::string& s)
{
    if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += '"';
        out += c;
    }
    return out + '"';
}

// Writes the rows to <data root>/oee/export/; returns the path, "" on failure.
std::string ExportCsv(const std::vector<OeeDowntime>& rows, OeeMs now)
{
    const std::string dir = DataPath("oee/export");
    std::error_code   ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path =
        dir + "/maintenance-history-" + OeeFormatLocal(now, "%Y%m%d-%H%M") + ".csv";
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) return std::string();
    f << "down_local,up_local,duration_min,category,reason,down_by,root_cause,action_taken,"
         "up_by,station\r\n";
    for (const OeeDowntime& d : rows) {
        char mins[32];
        std::snprintf(mins, sizeof(mins), "%.1f", static_cast<double>(d.Duration(now)) / 60000.0);
        f << OeeFormatLocal(d.start, "%Y-%m-%d %H:%M:%S") << ','
          << (d.Open() ? std::string() : OeeFormatLocal(d.end, "%Y-%m-%d %H:%M:%S")) << ','
          << mins << ',' << Csv(d.category) << ',' << Csv(d.reason) << ','
          << Csv(d.downByName) << ',' << Csv(d.rootCause) << ',' << Csv(d.actionTaken) << ','
          << Csv(d.upByName) << ',' << Csv(d.station) << "\r\n";
    }
    return f.good() ? path : std::string();
}

} // namespace

void App::DrawMaintenanceWindow()
{
    // The page is gated, but the open bit outlives a page switch and a role
    // never should.
    if (!CanAccess(Page::Oee)) { showMaint_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(1100.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Maintenance history", &showMaint_)) {
        ImGui::End();
        return;
    }

    const OeeMs now   = OeeNowMs();
    const OeeMs today = OeeLocalDayStart(now);

    // ---- Range + search ----
    const auto range = [&](const char* label, int days) {
        const bool on = (maintDays_ == days);
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        if (ImGui::Button(label)) maintDays_ = days;
        if (on) ImGui::PopStyleColor();
        ImGui::SameLine();
    };
    range("7 days", 7);
    range("30 days", 30);
    range("1 year", 365);
    ImGui::SetNextItemWidth(320.0f);
    ImGui::InputTextWithHint("##maintsearch", "search category / reason / cause / action / who",
                             maintSearch_, sizeof(maintSearch_));

    // In memory and small (one line per declaration), so it is filtered every
    // frame and the list can never disagree with the search box.
    const OeeMs from = OeeAddLocalDays(today, -(maintDays_ - 1));
    const OeeMs to   = OeeAddLocalDays(today, 1);
    const std::vector<OeeDowntime> rows = oee_.History(from, to, maintSearch_);

    // ---- Summary ----
    if (rows.empty()) {
        ImGui::TextDisabled("No downtime in this range.");
    } else {
        OeeMs total = 0, closedSum = 0, longest = 0;
        int   closed = 0, open = 0;
        std::map<std::string, std::pair<int, OeeMs>> byCat;
        for (const OeeDowntime& d : rows) {
            const OeeMs dur = d.Duration(now);
            total += dur;
            longest = std::max(longest, dur);
            if (d.Open()) ++open;
            else { ++closed; closedSum += dur; }
            auto& c = byCat[d.category.empty() ? std::string("(none)") : d.category];
            ++c.first;
            c.second += dur;
        }
        std::string summary = std::to_string(rows.size()) +
                              (rows.size() == 1 ? " downtime event" : " downtime events") +
                              "  -  total " + OeeHm(total);
        if (closed > 0) summary += "  -  MTTR " + OeeHm(closedSum / closed);
        summary += "  -  longest " + OeeHm(longest);
        if (open > 0) summary += "  -  1 still open";
        ImGui::TextUnformatted(summary.c_str());

        std::vector<std::pair<std::string, std::pair<int, OeeMs>>> cats(byCat.begin(), byCat.end());
        std::sort(cats.begin(), cats.end(),
                  [](const auto& a, const auto& b) { return a.second.second > b.second.second; });
        std::string line = "By category:";
        for (const auto& c : cats)
            line += "   " + c.first + " " + std::to_string(c.second.first) + "x (" +
                    OeeHm(c.second.second) + ")";
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", line.c_str());
        ImGui::PopTextWrapPos();
    }

    // ---- Export ----
    ImGui::BeginDisabled(rows.empty());
    if (ImGui::Button("Export CSV")) {
        const std::string path = ExportCsv(rows, now);
        maintMsg_ = path.empty() ? "Export failed: cannot write under " + DataPath("oee/export")
                                 : "Exported " + std::to_string(rows.size()) + " rows to " + path;
    }
    ImGui::EndDisabled();
    if (CanUse(UserRole::Technician)) {
        ImGui::SameLine();
        if (ImGui::Button("Open folder")) OpenFolderInExplorer(DataPath("oee/export"));
    }
    if (!maintMsg_.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", maintMsg_.c_str());
    }

    // ---- The entries ----
    if (ImGui::BeginTable("##mainthist", 9,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                              ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY |
                              ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Down", ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("Up", ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("Duration", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Category");
        ImGui::TableSetupColumn("Reason (at down)");
        ImGui::TableSetupColumn("Down by");
        ImGui::TableSetupColumn("Root cause");
        ImGui::TableSetupColumn("Action taken");
        ImGui::TableSetupColumn("Up by");
        ImGui::TableHeadersRow();
        for (const OeeDowntime& d : rows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeeFormatLocal(d.start, "%Y-%m-%d %H:%M").c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(d.Open() ? "(still down)"
                                            : OeeFormatLocal(d.end, "%Y-%m-%d %H:%M").c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeeHm(d.Duration(now)).c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.category.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.reason.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.downByName.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.rootCause.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.actionTaken.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.upByName.c_str());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace firedog
