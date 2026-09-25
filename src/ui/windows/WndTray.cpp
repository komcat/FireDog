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
        ImGui::BeginChild("traywingrid", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        DrawTrayGrid(t, orch_.Busy());
        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace firedog
