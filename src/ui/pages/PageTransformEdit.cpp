// PageTransformEdit.cpp : the axis-transform editor (View menu, full-page
// takeover). Definitions of App members — see App.h.
//
// Edits a working copy of ALL device transforms, including devices not in the
// current fleet (those are preserved on Save). Every edit hot-pushes through
// Workflow::SetTransforms so the jog panel reflects it immediately; Save writes
// config/motion_transforms.json.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <filesystem>

namespace firedog {

// Transforms editor — full-page main-window content (header/Close drawn by
// DrawMainWindow's editor-page branch). Rows = the 6 GLOBAL axes (X/Y/Z
// translation, U/V/W = theta x/y/z rotation), one column per motion device —
// same shape as the machine's global-axis table. Each cell picks the signed
// LOCAL axis that global axis maps to (one signed-permutation column of
// AxisTransform's m or r block). Every edit hot-applies to Workflow so the jog
// panel maps through it on the next press; Save writes config/motion_transforms.json.
void App::DrawTransformEditPanel()
{
    const size_t n = hardware_.MotionCount();
    if (n == 0) {
        ImGui::TextDisabled("no motion devices");
        return;
    }

    bool changed = false;

    if (ImGui::Button("Save")) {
        if (SaveMotionTransforms(MotionTransformsPath(), transformCfg_))
            FD_INFO("app", "transforms saved to %s", MotionTransformsPath().c_str());
        else
            FD_ERROR("app", "failed to save motion transforms");
    }
    ImGui::SameLine();
    if (ImGui::Button("Reload file")) {
        transformCfg_ = LoadMotionTransforms(MotionTransformsPath());
        changed = true;
        // Log the full path: the app reads the machine's copy, and "I edited
        // the repo file but Reload did nothing" is otherwise invisible.
        FD_INFO("app", "transforms reloaded from %s (%zu device(s))",
                MotionTransformsPath().c_str(), transformCfg_.size());
    }
    ImGui::TextDisabled("Edits apply to jog immediately (arm the Jog window to test). Save writes "
                        "this machine's %s; a build or a release never touches it (a change "
                        "every machine needs also goes into the repo's config/).",
                        MotionTransformsPath().c_str());

    // One combo per (global axis, device): the signed local axis this global
    // axis maps to. Writes one column of the selected block (m or r).
    auto cell = [&](AxisTransform& t, bool rot, int col, IMotionController* dev) {
        double (*blk)[3] = rot ? t.r : t.m;
        const std::vector<std::string> axes = dev->Axes();
        std::vector<std::string> names;   // local axis labels for this block
        if (rot) { for (size_t a = 3; a < axes.size() && a < 6; ++a) names.push_back(axes[a]); }
        else     { for (size_t a = 0; a < axes.size() && a < 3; ++a) names.push_back(axes[a]); }
        if (names.empty()) { ImGui::TextDisabled("n/a"); return; }

        // Current selection = dominant entry of this column.
        int ax = 0;
        for (int i = 1; i < 3; ++i)
            if (std::fabs(blk[i][col]) > std::fabs(blk[ax][col])) ax = i;
        const bool mapped = std::fabs(blk[ax][col]) > 1e-9 && ax < static_cast<int>(names.size());
        const bool pos    = blk[ax][col] >= 0.0;

        char cur[16];
        if (mapped) std::snprintf(cur, sizeof(cur), "%s %s", names[ax].c_str(), pos ? "+" : "-");
        else        std::snprintf(cur, sizeof(cur), "-");

        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::BeginCombo("##cell", cur)) {
            auto pick = [&](int i, double sign) {
                for (int r = 0; r < 3; ++r) blk[r][col] = 0.0;
                if (i >= 0) blk[i][col] = sign;
                changed = true;
            };
            for (int i = 0; i < static_cast<int>(names.size()); ++i) {
                char lbl[16];
                std::snprintf(lbl, sizeof(lbl), "%s +", names[i].c_str());
                if (ImGui::Selectable(lbl, mapped && ax == i && pos)) pick(i, +1.0);
                std::snprintf(lbl, sizeof(lbl), "%s -", names[i].c_str());
                if (ImGui::Selectable(lbl, mapped && ax == i && !pos)) pick(i, -1.0);
            }
            if (ImGui::Selectable("- (none)", !mapped)) pick(-1, 0.0);
            ImGui::EndCombo();
        }
    };

    static const char* kRows[6] = {"X", "Y", "Z", "U (\xce\xb8x)", "V (\xce\xb8y)", "W (\xce\xb8z)"};

    if (ImGui::BeginTable("xform", 1 + static_cast<int>(n),
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Global axis");
        for (size_t d = 0; d < n; ++d)
            ImGui::TableSetupColumn(hardware_.MotionName(d).c_str());
        ImGui::TableHeadersRow();

        for (int row = 0; row < 6; ++row) {
            const bool rot = row >= 3;
            const int  col = rot ? row - 3 : row;   // column of the m / r block
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(kRows[row]);
            for (size_t d = 0; d < n; ++d) {
                ImGui::TableSetColumnIndex(1 + static_cast<int>(d));
                IMotionController* dev = hardware_.MotionDevice(d);
                if (!dev) { ImGui::TextDisabled("-"); continue; }
                ImGui::PushID(row * 64 + static_cast<int>(d));
                AxisTransform& t = transformCfg_[dev->Name()];   // identity if new
                cell(t, rot, col, dev);
                ImGui::PopID();
            }
        }
        ImGui::EndTable();
    }

    if (changed) workflow_.SetTransforms(transformCfg_);   // hot reload
}

} // namespace firedog
