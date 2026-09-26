// WndTray.cpp : the floating Waffle Tray window (View menu). Definitions of App
// members — see App.h.
//
// The Waffle Tray PAGE teaches a tray; this window is what an operator keeps
// open beside the Run page while parts come and go: pick the ACTIVE cell (the
// one a recipe's move_to_active_cell drives to), mark cavities used or full,
// and watch a run take them. It draws the same grid the page does, through
// App::DrawTrayGrid, so a click means one thing in both places. Operator
// level: selecting and marking are facts about the parts, not the machine;
// Drive here on the grid's menu is behind the jog arm like every move.
#include "App.h"

#include "imgui.h"

#include "core/WaffleTray.h"

#include <cstdio>
#include <string>
#include <vector>

namespace firedog {

void App::DrawTrayWindow()
{
    // The menu item is gated, but the open bit outlives a page switch and a
    // role never should.
    if (!CanUse(kRoleTray)) { showTray_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(520.0f, 420.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Waffle Tray", &showTray_)) {
        ImGui::End();
        return;
    }

    std::vector<WaffleTray>& list = session_.Trays();
    if (traySel_ >= static_cast<int>(list.size())) traySel_ = list.empty() ? -1 : 0;
    if (traySel_ < 0 && !list.empty()) traySel_ = 0;
    WaffleTray* t = (traySel_ >= 0) ? &list[static_cast<std::size_t>(traySel_)] : nullptr;

    // The same selection the page holds: one tray is "the tray" everywhere.
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::BeginCombo("##traywin", t ? t->name.c_str() : "(no trays)")) {
        for (int i = 0; i < static_cast<int>(list.size()); ++i) {
            const WaffleTray& row = list[static_cast<std::size_t>(i)];
            char lbl[160];
            std::snprintf(lbl, sizeof(lbl), "%s   (%d x %d, %s)##%d", row.name.c_str(),
                          row.rows, row.cols,
                          row.device.empty() ? "no device" : row.device.c_str(), i);
            if (ImGui::Selectable(lbl, i == traySel_)) {
                traySel_ = i;
                trayMsg_.clear();
            }
        }
        ImGui::EndCombo();
    }
    if (t) {
        TrayCell active{0, 0};
        ImGui::SameLine();
        if (workflow_.TrayActive(t->uid, &active))
            ImGui::Text("active (%d,%d)", active.first, active.second);
        else
            ImGui::TextDisabled("no active cell - click one");
    }
    if (!trayMsg_.empty()) ImGui::TextWrapped("%s", trayMsg_.c_str());
    ImGui::Separator();

    if (!t) {
        ImGui::TextDisabled("No tray. Teach one on the Waffle Tray page (F10).");
    } else {
        // Left strip: < / > step through the trays, wrapping at either end. It
        // moves the same traySel_ the combo and the page use - a view choice,
        // so it stays live during a run exactly like the combo.
        const int n = static_cast<int>(list.size());
        const float stripW = 44.0f;
        ImGui::BeginChild("traywinnav", ImVec2(stripW, 0.0f), ImGuiChildFlags_None);
        const ImVec2 btn(stripW - ImGui::GetStyle().WindowPadding.x * 2.0f, 40.0f);
        ImGui::BeginDisabled(n < 2);
        if (ImGui::Button("<", btn)) {
            traySel_ = (traySel_ + n - 1) % n;
            trayMsg_.clear();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Previous tray");
        if (ImGui::Button(">", btn)) {
            traySel_ = (traySel_ + 1) % n;
            trayMsg_.clear();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Next tray");
        ImGui::EndDisabled();
        ImGui::TextDisabled("%d/%d", traySel_ + 1, n);
        ImGui::EndChild();
        // Re-resolve: a click above changed the selection this frame.
        t = &list[static_cast<std::size_t>(traySel_)];

        ImGui::SameLine();
        ImGui::BeginChild("traywingrid", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        DrawTrayGrid(t, orch_.Busy());
        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace firedog
