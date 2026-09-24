// WndPositionHistory.cpp : the Position History window (View menu, and the
// Motion page's History / Hist buttons) — the saved-positions change journal
// (core/PositionHistory.h) beside the positions file in use, newest first,
// filtered by device and optionally one position, with Restore.
//
// Restore is not an undo: it writes the entry's old state back through
// ProductSession::RestorePosition, which saves like any edit and so appends
// its own entry ("restore of update @ <stamp>"). The journal therefore never
// loses the state it restored FROM, and a restore can itself be restored.
// Definitions of App members — see App.h.
#include "App.h"

#include "imgui.h"

#include "core/PositionHistory.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <system_error>
#include <vector>

namespace firedog {

namespace {

// "2026-09-24T03:11:03.120Z" -> "2026-09-24 10:11:03" in local time; anything
// that does not parse is shown as stored.
std::string LocalStamp(const std::string& iso)
{
    // Fixed layout "YYYY-MM-DDTHH:MM:SS..." — from_chars per field, like the
    // View Data window's bucket dates (no sscanf).
    auto field = [&](std::size_t at, std::size_t len, int& out) {
        if (iso.size() < at + len) return false;
        const char* b = iso.data() + at;
        return std::from_chars(b, b + len, out).ec == std::errc{};
    };
    std::tm tm{};
    if (!field(0, 4, tm.tm_year) || !field(5, 2, tm.tm_mon) || !field(8, 2, tm.tm_mday) ||
        !field(11, 2, tm.tm_hour) || !field(14, 2, tm.tm_min) || !field(17, 2, tm.tm_sec))
        return iso;
    tm.tm_year -= 1900;
    tm.tm_mon  -= 1;
    const time_t t = _mkgmtime(&tm);
    if (t == static_cast<time_t>(-1)) return iso;
    std::tm lt{};
    localtime_s(&lt, &t);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", lt.tm_year + 1900,
                  lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
    return buf;
}

// One line per axis that changed: "X -5.8140 -> -5.8000 (+0.0140)".
std::string DescribeChange(const PosChange& c)
{
    std::string out;
    char        b[96];
    if (c.action == "rename") {
        return "'" + c.nameBefore + "' -> '" + c.name + "'";
    }
    if (c.action == "update") {
        for (const auto& [axis, nv] : c.after) {
            const auto it = c.before.find(axis);
            const bool had = it != c.before.end();
            if (had && std::fabs(it->second - nv) <= 1e-6) continue;
            if (had)
                std::snprintf(b, sizeof(b), "%s %.4f -> %.4f (%+.4f)", axis.c_str(),
                              it->second, nv, nv - it->second);
            else
                std::snprintf(b, sizeof(b), "%s (new) -> %.4f", axis.c_str(), nv);
            if (!out.empty()) out += "\n";
            out += b;
        }
        for (const auto& [axis, ov] : c.before) {
            if (c.after.count(axis)) continue;
            std::snprintf(b, sizeof(b), "%s %.4f -> (dropped)", axis.c_str(), ov);
            if (!out.empty()) out += "\n";
            out += b;
        }
        return out;
    }
    const auto& vals = (c.action == "delete") ? c.before : c.after;
    for (const auto& [axis, v] : vals) {
        std::snprintf(b, sizeof(b), "%s %.4f", axis.c_str(), v);
        if (!out.empty()) out += "  ";
        out += b;
    }
    return out;
}

ImVec4 ActionColor(const std::string& a)
{
    if (a == "add")    return ImVec4(0.45f, 0.85f, 0.50f, 1.0f);
    if (a == "delete") return ImVec4(1.00f, 0.40f, 0.30f, 1.0f);
    if (a == "rename") return ImVec4(0.55f, 0.70f, 1.00f, 1.0f);
    return ImVec4(0.98f, 0.72f, 0.32f, 1.0f);   // update
}

} // namespace

void App::DrawPositionHistoryWindow()
{
    // The menu item is gated, but the open bit outlives a page switch and a
    // role never should.
    if (!CanUse(kRoleJog)) { showPosHist_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(980.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Position History", &showPosHist_)) {
        ImGui::End();
        return;
    }

    // Re-read when the journal grew (session version) or moved (product load).
    const std::string path = session_.PositionHistoryPath();
    if (path != posHistPath_ || posHistVer_ != session_.PositionHistoryVersion()) {
        posHistEntries_ = LoadPositionHistory(path);
        posHistPath_    = path;
        posHistVer_     = session_.PositionHistoryVersion();
    }

    // ---- Filters ----
    ImGui::TextUnformatted("Device");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("##phdev", posHistDevice_.empty() ? "(all)" : posHistDevice_.c_str())) {
        if (ImGui::Selectable("(all)", posHistDevice_.empty())) {
            posHistDevice_.clear();
            posHistUid_.clear();
        }
        for (const auto& [dev, rows] : session_.Positions()) {
            (void)rows;
            if (ImGui::Selectable(dev.c_str(), posHistDevice_ == dev)) {
                if (posHistDevice_ != dev) posHistUid_.clear();
                posHistDevice_ = dev;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Position");
    ImGui::SameLine();
    {
        // The current rows of that device; a uid that only the journal knows
        // (a deleted row) is reachable through its row's "filter" click below.
        std::string label = "(all)";
        if (!posHistUid_.empty()) {
            label = posHistUid_;
            const auto it = session_.Positions().find(posHistDevice_);
            if (it != session_.Positions().end())
                for (const SavedPos& sp : it->second)
                    if (sp.uid == posHistUid_) { label = sp.name; break; }
        }
        ImGui::SetNextItemWidth(220.0f);
        ImGui::BeginDisabled(posHistDevice_.empty());
        if (ImGui::BeginCombo("##phpos", label.c_str())) {
            if (ImGui::Selectable("(all)", posHistUid_.empty())) posHistUid_.clear();
            const auto it = session_.Positions().find(posHistDevice_);
            if (it != session_.Positions().end()) {
                for (const SavedPos& sp : it->second) {
                    if (sp.hidden) continue;   // never journalled
                    ImGui::PushID(sp.uid.c_str());
                    if (ImGui::Selectable(sp.name, posHistUid_ == sp.uid)) posHistUid_ = sp.uid;
                    ImGui::PopID();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Reload")) posHistPath_.clear();   // forces the re-read
    ImGui::SameLine();
    ImGui::TextDisabled("%s", path.c_str());

    // ---- Rows, newest first ----
    int shown = 0;
    for (const PosChange& c : posHistEntries_) {
        if (!posHistDevice_.empty() && c.device != posHistDevice_) continue;
        if (!posHistUid_.empty() && c.uid != posHistUid_) continue;
        ++shown;
    }
    if (shown == 0) {
        ImGui::TextDisabled(posHistEntries_.empty()
                                ? "No changes recorded yet - the journal starts with the next "
                                  "edit, teach, rename or delete."
                                : "No entries match the filter.");
    } else {
        ImGui::TextDisabled("%d entr%s", shown, shown == 1 ? "y" : "ies");
    }
    const bool busy = orch_.Busy();
    if (ImGui::BeginTable("##phrows", 8,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable,
                          ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("When",     ImGuiTableColumnFlags_WidthFixed, 140.0f);
        ImGui::TableSetupColumn("By",       ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Source",   ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableSetupColumn("Action",   ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("Device",   ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableSetupColumn("Change",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##restore", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        ImGui::TableHeadersRow();

        for (int i = static_cast<int>(posHistEntries_.size()) - 1; i >= 0; --i) {
            const PosChange& c = posHistEntries_[static_cast<std::size_t>(i)];
            if (!posHistDevice_.empty() && c.device != posHistDevice_) continue;
            if (!posHistUid_.empty() && c.uid != posHistUid_) continue;
            ImGui::TableNextRow();
            ImGui::PushID(i);
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(LocalStamp(c.at).c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s (UTC)", c.at.c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(c.by.empty() ? "-" : c.by.c_str());
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(c.source.c_str());
            if (ImGui::IsItemHovered() && c.source.size() > 24) ImGui::SetTooltip("%s", c.source.c_str());
            ImGui::TableSetColumnIndex(3);
            ImGui::TextColored(ActionColor(c.action), "%s", c.action.c_str());
            ImGui::TableSetColumnIndex(4);
            ImGui::TextUnformatted(c.device.c_str());
            ImGui::TableSetColumnIndex(5);
            // Click a name to narrow the view to that position — the one way
            // to follow a row the table no longer lists (deleted).
            if (ImGui::Selectable(c.name.c_str(), false, ImGuiSelectableFlags_None)) {
                posHistDevice_ = c.device;
                posHistUid_    = c.uid;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("uid: %s  (click to filter)", c.uid.c_str());
            ImGui::TableSetColumnIndex(6);
            ImGui::TextUnformatted(DescribeChange(c).c_str());
            ImGui::TableSetColumnIndex(7);
            const bool restorable = c.action == "update" || c.action == "rename" ||
                                    c.action == "delete";
            if (restorable) {
                ImGui::BeginDisabled(busy);
                if (ImGui::SmallButton("Restore")) {
                    if (!session_.RestorePosition(c))
                        FD_WARN("saved", "nothing to restore for '%s' (%s)", c.name.c_str(),
                                c.action.c_str());
                }
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    if (busy)
                        ImGui::SetTooltip("Not while a run is active");
                    else if (c.action == "delete")
                        ImGui::SetTooltip("Re-create the row under its old uid with these values");
                    else if (c.action == "rename")
                        ImGui::SetTooltip("Put the old name back");
                    else
                        ImGui::SetTooltip("Write the BEFORE values back (journalled as a restore)");
                }
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace firedog
