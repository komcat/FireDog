// PageDutView.cpp : the DUT Records viewer (View menu, full-page, READ-ONLY).
// Definitions of App members — see App.h.
//
// Scans the loaded product's data/ for serials and shows one record's
// append-only entry log plus its run history. Viewing only: every mutation goes
// through DutSession via the dut_* commands.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DutSession.h"
#include "core/Log.h"

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace firedog {

void App::LoadDutList()
{
    namespace fs = std::filesystem;
    dutList_.clear();
    dutSel_ = -1;
    dutRecSerial_.clear();
    dutRec_ = nlohmann::json();
    if (!session_.ProductLoaded()) return;
    std::error_code ec;
    for (const fs::directory_entry& e :
         fs::directory_iterator(session_.Product().DataDir(), ec)) {
        if (e.is_directory(ec) && fs::exists(e.path() / "dut.json", ec))
            dutList_.push_back(e.path().filename().string());
    }
    std::sort(dutList_.begin(), dutList_.end());
    if (!dutList_.empty()) dutSel_ = 0;
}

void App::DrawDutViewPanel()
{
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  leftW = 200.0f, rightW = 320.0f;

    // ---- Left: serial list ----
    ImGui::BeginChild("dvList", ImVec2(leftW, 0.0f), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Serials");
    ImGui::SameLine();
    if (ImGui::SmallButton("Reload")) LoadDutList();
    ImGui::Separator();
    if (!session_.ProductLoaded())
        ImGui::TextWrapped("Load a product first.");
    else if (dutList_.empty())
        ImGui::TextWrapped("No DUT records under %s.", session_.Product().DataDir().c_str());
    for (int i = 0; i < static_cast<int>(dutList_.size()); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(dutList_[i].c_str(), dutSel_ == i)) dutSel_ = i;
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // (Re)load the selected record — tolerant, view-only.
    if (dutSel_ >= 0 && dutSel_ < static_cast<int>(dutList_.size()) &&
        dutRecSerial_ != dutList_[dutSel_]) {
        dutRecSerial_ = dutList_[dutSel_];
        dutRec_       = nlohmann::json();
        try {
            std::ifstream f(session_.Product().DataDir() + "/" + dutRecSerial_ + "/dut.json");
            if (f.is_open()) f >> dutRec_;
        } catch (const std::exception& e) {
            FD_WARN("dut", "record '%s' unreadable: %s", dutRecSerial_.c_str(), e.what());
        }
    }
    const nlohmann::json entries =
        dutRec_.contains("entries") && dutRec_["entries"].is_array()
            ? dutRec_["entries"]
            : nlohmann::json::array();

    // ---- Middle: entry table ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    ImGui::BeginChild("dvEntries", ImVec2(midW, 0.0f), ImGuiChildFlags_Borders);
    if (dutRec_.is_object()) {
        if (ImGui::BeginTable("##dventries", 5,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableSetupColumn("time (UTC)", ImGuiTableColumnFlags_WidthFixed, 170.0f);
            ImGui::TableSetupColumn("key", ImGuiTableColumnFlags_WidthFixed, 120.0f);
            ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("value");
            ImGui::TableHeadersRow();
            for (const nlohmann::json& e : entries) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", static_cast<std::size_t>(e.value("seq", 0u)));
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(e.value("t", std::string()).c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(e.value("key", std::string()).c_str());
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(e.value("type", std::string()).c_str());
                ImGui::TableSetColumnIndex(4);
                const nlohmann::json v = e.contains("value") ? e["value"] : nlohmann::json();
                std::string shown = v.is_string() ? v.get<std::string>() : v.dump();
                if (shown.size() > 120) shown = shown.substr(0, 117) + "...";
                ImGui::TextUnformatted(shown.c_str());
                if (shown.size() >= 120 && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", v.dump(2).c_str());
            }
            ImGui::EndTable();
        }
    } else {
        ImGui::TextDisabled("Select a serial on the left.");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: record summary + run history ----
    ImGui::BeginChild("dvInfo", ImVec2(rightW, 0.0f), ImGuiChildFlags_Borders);
    if (dutRec_.is_object()) {
        ImGui::Text("Serial: %s", dutRec_.value("serial", std::string()).c_str());
        ImGui::Text("Product: %s", dutRec_.value("product", std::string()).c_str());
        ImGui::Text("Opened: %s", dutRec_.value("opened_at", std::string()).c_str());
        const std::string closed = dutRec_.value("closed_at", std::string());
        ImGui::Text("Closed: %s", closed.empty() ? "(open or interrupted)" : closed.c_str());
        const std::string status = dutRec_.value("status", std::string());
        if (status == "pass")
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f), "Status: PASS");
        else if (status == "fail")
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "Status: FAIL");
        else
            ImGui::TextDisabled("Status: (none)");
        ImGui::Text("Entries: %zu", entries.size());
        ImGui::Separator();
        ImGui::TextUnformatted("Runs");
        for (const nlohmann::json& e : entries) {
            if (e.value("key", std::string()) != "$run") continue;
            const nlohmann::json v = e.contains("value") ? e["value"] : nlohmann::json();
            if (!v.is_object()) continue;
            ImGui::BulletText("%s '%s' - %s", v.value("kind", std::string()).c_str(),
                              v.value("name", std::string()).c_str(),
                              v.value("result", std::string()).c_str());
        }
    }
    ImGui::EndChild();
}

} // namespace firedog
