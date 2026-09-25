// PageTray.cpp : the Waffle Tray page (a full-page takeover, F10). Definitions
// of App members — see App.h.
//
// A tray is an R x C array of cavities, addressed by (row, col). Teaching every
// cavity as a saved position is what this page replaces: the operator jogs onto
// THREE of them — origin (1,1), row end (1,C), far corner (R,C) — types the
// counts, and every other cell is derived (core/WaffleTray.h, the one place the
// arithmetic lives; docs/waffle-tray-design.md).
//
// ---- What the page is for ----------------------------------------------------
//
// The moving is a recipe step (`move_to_cell`), so like the Gripped Object page
// this is not where the work happens. It exists for what a recipe cannot do:
//
//   * TEACHING the corners. Each one needs an operator jogging a tool into a
//     cavity by eye; there is nothing to automate, and a second way to create a
//     tray is a second answer to where its cells are.
//   * MASKING: a chipped row, a missing column, one broken cavity. A disabled
//     cell is never offered by `tray_next` and refused by `move_to_cell`.
//   * MARKING which cavities hold a part. A run's tray_next marks the cell it
//     took; an operator marks one by clicking it, or the whole tray with All
//     full / All empty. The set is saved WITH the tray, so a restart mid-tray
//     carries on at the next full cavity — the Workflow cursor a run reads is
//     its live mirror, and App::SetTrayCellsUsed writes both.
//
// The record is PRODUCT state and saves like every document: the Save button,
// with two exceptions that save on the spot because they are physical acts an
// operator must not lose to a forgotten click — teaching a corner, and changing
// the device (which clears the corners).
//
// Layout: the grid on the left (a button per cell, row and column headers that
// toggle a whole line, right-click drives there), the controls on the right —
// tray list, name, device, size, the three corner cards, the pitch readout.
#include "App.h"

#include "imgui.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid / NowStamp
#include "core/WaffleTray.h"
#include "hardware/motion/IMotionController.h"
#include "ui/Widgets.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);

// Cell button tints, one per state an operator reads off the grid at a glance:
// green holds a part, grey is empty (used), yellow is the one being worked on
// (the last tray_next), dark is switched off.
const ImVec4 kCellFull  (0.20f, 0.55f, 0.28f, 1.0f);
const ImVec4 kCellEmpty (0.45f, 0.45f, 0.47f, 1.0f);
const ImVec4 kCellActive(0.85f, 0.70f, 0.15f, 1.0f);
const ImVec4 kCellOff   (0.22f, 0.22f, 0.24f, 1.0f);   // disabled by row/col/cell

// Past this the grid is not something a person reads cell by cell, and a
// button per cell is a frame budget: the controls still work, the grid says so.
constexpr int kMaxDrawnCells = 2500;

// One line of "X 1.234  Y 5.678 ..." for a corner's readout.
std::string PoseLine(const std::map<std::string, double>& vals)
{
    std::string line;
    for (const auto& [axis, v] : vals) {
        char b[48];
        std::snprintf(b, sizeof(b), "%s%s %.3f", line.empty() ? "" : "  ", axis.c_str(), v);
        line += b;
    }
    return line;
}

} // namespace

// The one writer of a corner. Both the live capture and the taught-position
// picker resolve to COORDINATES before they get here, so the record never
// holds a position uid (a tray outlives the product's positions list the way
// a registration outlives the product). Saved on the spot: a teach is a
// physical act, and the next click is not where it should be lost.
void App::TeachTrayCorner(WaffleTray& t, TrayCornerSlot slot,
                          const std::map<std::string, double>& pose, const char* how)
{
    if (pose.empty()) {
        trayMsg_ = std::string(TrayCornerLabel(slot)) + ": the pose carries no axes";
        return;
    }
    TrayCorner& c = TrayCornerOf(t, slot);
    c.vals     = pose;
    c.have     = true;
    c.taughtAt = NowStamp();
    session_.SaveTrays();
    trayMsg_ = std::string(TrayCornerLabel(slot)) + " " + how;
    FD_INFO("tray", "'%s': %s %s on %s: %s", t.name.c_str(), TrayCornerLabel(slot), how,
            t.device.c_str(), PoseLine(pose).c_str());
}

void App::GoToTrayCell(const WaffleTray& t, int r, int c)
{
    char what[112];
    std::snprintf(what, sizeof(what), "cell (%d,%d) of '%s'", r, c, t.name.c_str());
    std::map<std::string, double> pose;
    if (!TrayCellPose(t, r, c, &pose)) {
        const std::string why = TrayValidate(t);
        trayMsg_ = std::string(what) + ": " + (why.empty() ? "outside the tray" : why);
        return;
    }
    if (!jogArmed_) {
        trayMsg_          = "arm jog first - this drives a device";
        trayOpenArmPopup_ = true;
        return;
    }
    if (orch_.Busy()) {
        trayMsg_ = "a run is active";
        return;
    }
    if (!StartWatchedMove(t.device, pose, what))
        trayMsg_ = "'" + t.device + "' refused the move - is it connected?";
    else
        trayMsg_.clear();
}

void App::SetTrayCellsUsed(const WaffleTray& t, const std::vector<TrayCell>& cells,
                           bool used, const char* why)
{
    // Cursor first (what a run reads), then the document (what survives).
    if (cells.empty()) {
        if (used) {
            std::set<TrayCell> all;
            for (int r = 1; r <= t.rows; ++r)
                for (int c = 1; c <= t.cols; ++c) all.insert(TrayCell{r, c});
            workflow_.TraySeed(t.uid, all);
        } else {
            workflow_.ClearTrayCursor(t.uid, why);
        }
    } else {
        for (const TrayCell& cell : cells) {
            if (used) workflow_.TrayConsume(t.uid, cell.first, cell.second);
            else      workflow_.TrayRelease(t.uid, cell.first, cell.second);
        }
    }
    session_.SetTrayCellsUsed(t.uid, cells, used);
    if (cells.size() == 1)
        FD_INFO("tray", "'%s': cell (%d,%d) %s", t.name.c_str(), cells[0].first,
                cells[0].second, why);
    else
        FD_INFO("tray", "'%s': %s", t.name.c_str(), why);
}

// The grid itself: the status lines, All full / All empty, the legend and a
// button per cell. Shared by the page and the floating Waffle Tray window,
// so the two cannot drift on what a click means.
void App::DrawTrayGrid(WaffleTray* t, bool busy)
{
    const std::string why = TrayValidate(*t);
    const std::set<TrayCell> taken = workflow_.TrayConsumed(t->uid);
    const std::size_t total = TrayCellCount(*t);
    std::size_t enabled = 0;
    for (int r = 1; r <= t->rows; ++r)
        for (int c = 1; c <= t->cols; ++c)
            if (TrayCellEnabled(*t, r, c)) ++enabled;

    ImGui::Text("%s  -  %d x %d on %s", t->name.c_str(), t->rows, t->cols,
                t->device.empty() ? "(no device)" : t->device.c_str());
    if (!why.empty())
        ImGui::TextColored(kWarn, "not usable yet: %s", why.c_str());
    else
        ImGui::TextColored(kGood, "usable: every cell has a pose");
    ImGui::Text("%zu of %zu cells enabled, %zu used (empty)", enabled, total,
                taken.size());
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || taken.empty());
    if (ImGui::SmallButton("All full")) {
        SetTrayCellsUsed(*t, {}, false, "all full on the Waffle Tray page");
        // A fresh tray starts at its first cell, like a tray_reset step.
        const std::vector<TrayCell> cells = TrayEnabledCells(*t, TrayOrder::RowMajor);
        if (!cells.empty()) workflow_.SetTrayActive(t->uid, cells.front());
        trayMsg_ = cells.empty() ? "every cell marked full"
                                 : "every cell marked full - the first cell is active";
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("A fresh tray was loaded: every cavity holds a part.\n"
                          "A recipe does the same with a tray_reset step.");
    ImGui::SameLine();
    ImGui::BeginDisabled(busy || taken.size() >= total);
    if (ImGui::SmallButton("All empty")) {
        SetTrayCellsUsed(*t, {}, true, "all empty on the Waffle Tray page");
        trayMsg_ = "every cell marked used - tray_next reports the tray empty";
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Every cavity is empty; then mark the ones that still hold "
                          "a part.");
    ImGui::TextDisabled("click a cell: make it the ACTIVE cell (move_to_active_cell goes "
                        "there) - cells before it become used, it and the rest full;  "
                        "right-click: mark used / full, drive there, disable;  a header: "
                        "the whole line on / off");
    ImGui::TextColored(kCellFull, "green");   ImGui::SameLine(); ImGui::TextDisabled("full");
    ImGui::SameLine(); ImGui::TextColored(kCellEmpty, "grey");   ImGui::SameLine(); ImGui::TextDisabled("empty (used)");
    ImGui::SameLine(); ImGui::TextColored(kCellActive, "yellow"); ImGui::SameLine(); ImGui::TextDisabled("active (selected, or last tray_next)");
    ImGui::SameLine(); ImGui::TextColored(kCellOff, "dark");     ImGui::SameLine(); ImGui::TextDisabled("disabled");
    ImGui::Separator();

    if (total > static_cast<std::size_t>(kMaxDrawnCells)) {
        ImGui::TextWrapped("%zu cells is more than this page draws (%d). The tray "
                           "still works: move_to_cell and tray_next address it by "
                           "index, and rows and columns can be disabled below.",
                           total, kMaxDrawnCells);
        // The line toggles without the grid, so a big tray is still maskable.
        ImGui::BeginDisabled(busy);
        ImGui::SetNextItemWidth(100.0f);
        ImGui::InputInt("row##big", &trayLineRow_);
        ImGui::SameLine();
        if (ImGui::SmallButton("toggle row")) {
            const int r = trayLineRow_;
            if (r >= 1 && r <= t->rows) {
                if (!t->disabledRows.erase(r)) t->disabledRows.insert(r);
            }
        }
        ImGui::SetNextItemWidth(100.0f);
        ImGui::InputInt("col##big", &trayLineCol_);
        ImGui::SameLine();
        if (ImGui::SmallButton("toggle col")) {
            const int c = trayLineCol_;
            if (c >= 1 && c <= t->cols) {
                if (!t->disabledCols.erase(c)) t->disabledCols.insert(c);
            }
        }
        ImGui::EndDisabled();
    } else {
        // Cell size from what fits, clamped so a 2 x 3 is not three
        // billboards and a 40 x 40 is still a target; past the clamp the
        // child scrolls.
        const ImVec2 region = ImGui::GetContentRegionAvail();
        const float  gap    = ImGui::GetStyle().ItemSpacing.x;
        const float  headW  = 30.0f;
        const float  fitW   = (region.x - headW - gap * t->cols) / static_cast<float>(t->cols);
        const float  fitH   = (region.y - headW - gap * t->rows) / static_cast<float>(t->rows);
        const float  sz     = std::clamp(std::floor(fminf(fitW, fitH)), 16.0f, 72.0f);
        const bool   labels = sz >= 34.0f;
        TrayCell active{0, 0};
        const bool haveActive = workflow_.TrayActive(t->uid, &active);

        ImGui::BeginChild("trayCells", region, ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar);
        // The headers and the menu's marks / disables are document edits and
        // lock while a run is on; the cell click (the active cell) does NOT,
        // because "pick the cell, then OK" on a prompt is the point of it.
        // Column headers: a whole column on or off.
        ImGui::Dummy(ImVec2(headW, sz * 0.6f));
        ImGui::BeginDisabled(busy);
        for (int c = 1; c <= t->cols; ++c) {
            ImGui::SameLine();
            const bool off = t->disabledCols.count(c) > 0;
            if (off) ImGui::PushStyleColor(ImGuiCol_Button, kCellOff);
            char lbl[32];
            std::snprintf(lbl, sizeof(lbl), "%d###ch%d", c, c);
            if (ImGui::Button(lbl, ImVec2(sz, sz * 0.6f))) {
                if (!t->disabledCols.erase(c)) t->disabledCols.insert(c);
            }
            if (off) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("column %d: %s", c, off ? "disabled" : "enabled");
        }
        ImGui::EndDisabled();
        for (int r = 1; r <= t->rows; ++r) {
            const bool rowOff = t->disabledRows.count(r) > 0;
            if (rowOff) ImGui::PushStyleColor(ImGuiCol_Button, kCellOff);
            char lbl[32];
            std::snprintf(lbl, sizeof(lbl), "%d###rh%d", r, r);
            ImGui::BeginDisabled(busy);
            if (ImGui::Button(lbl, ImVec2(headW, sz))) {
                if (!t->disabledRows.erase(r)) t->disabledRows.insert(r);
            }
            ImGui::EndDisabled();
            if (rowOff) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("row %d: %s", r, rowOff ? "disabled" : "enabled");

            for (int c = 1; c <= t->cols; ++c) {
                ImGui::SameLine();
                const bool cellOff = t->disabledCells.count(TrayCell{r, c}) > 0;
                const bool on      = TrayCellEnabled(*t, r, c);
                const bool took    = taken.count(TrayCell{r, c}) > 0;
                const bool isActive = haveActive && active == TrayCell{r, c};
                const ImVec4 tint = !on      ? kCellOff
                                  : isActive ? kCellActive
                                  : took     ? kCellEmpty
                                             : kCellFull;
                ImGui::PushStyleColor(ImGuiCol_Button, tint);
                const int pushed = 1;
                char cl[48];
                if (labels) std::snprintf(cl, sizeof(cl), "%d,%d###c%d_%d", r, c, r, c);
                else        std::snprintf(cl, sizeof(cl), "###c%d_%d", r, c);
                // Left click: pick the ACTIVE cell - what move_to_active_cell
                // drives to. Clicking the active one again clears it. The
                // marks and the setup acts sit on the right-click menu, so a
                // stray click changes nothing about the parts or the tray.
                if (ImGui::Button(cl, ImVec2(sz, sz))) {
                    if (isActive) {
                        workflow_.ClearTrayActive(t->uid);
                        trayMsg_ = "no active cell";
                    } else {
                        workflow_.SetTrayActive(t->uid, TrayCell{r, c});
                        // Picking a cell says "the work starts here": every
                        // cell before it in reading order (row by row) is
                        // used, this one and every one after it hold a part.
                        // One click describes the whole tray, so a run walks
                        // from the pick to the end and nowhere else.
                        std::vector<TrayCell> before, after;
                        for (int rr = 1; rr <= t->rows; ++rr)
                            for (int cc = 1; cc <= t->cols; ++cc) {
                                const TrayCell k{rr, cc};
                                const bool isBefore = (rr < r) || (rr == r && cc < c);
                                const bool used = t->usedCells.count(k) > 0;
                                if (isBefore && !used) before.push_back(k);
                                if (!isBefore && used) after.push_back(k);
                            }
                        if (!before.empty())
                            SetTrayCellsUsed(*t, before, true,
                                             "marked used before the active cell");
                        if (!after.empty())
                            SetTrayCellsUsed(*t, after, false,
                                             "marked full from the active cell on");
                        char b[160];
                        std::snprintf(b, sizeof(b), "active cell (%d,%d) - "
                                      "move_to_active_cell goes there; cells before it are "
                                      "used, it and every cell after it are full", r, c);
                        trayMsg_ = b;
                    }
                }
                if (pushed) ImGui::PopStyleColor(pushed);
                if (ImGui::BeginPopupContextItem()) {
                    ImGui::TextDisabled("cell (%d,%d)", r, c);
                    ImGui::BeginDisabled(busy);
                    if (ImGui::MenuItem(took ? "Mark full (holds a part)" : "Mark used (empty)"))
                        SetTrayCellsUsed(*t, {TrayCell{r, c}}, !took,
                                         took ? "marked full on the grid"
                                              : "marked used on the grid");
                    if (CanUse(kRoleJog) && ImGui::MenuItem("Drive here")) GoToTrayCell(*t, r, c);
                    if (ImGui::MenuItem(cellOff ? "Enable cell" : "Disable cell")) {
                        // The cell's OWN flag; a cell dark because its row
                        // is off stays dark, and the tooltip says which.
                        if (cellOff) t->disabledCells.erase(TrayCell{r, c});
                        else         t->disabledCells.insert(TrayCell{r, c});
                    }
                    ImGui::EndDisabled();
                    if (busy) ImGui::TextDisabled("(a run is active)");
                    ImGui::EndPopup();
                }
                if (ImGui::IsItemHovered()) {
                    std::map<std::string, double> pose;
                    std::string tip = "(" + std::to_string(r) + "," + std::to_string(c) + ")";
                    if (TrayCellPose(*t, r, c, &pose)) tip += "  " + PoseLine(pose);
                    if (cellOff) tip += "\ndisabled (this cell)";
                    if (rowOff)  tip += "\ndisabled (row " + std::to_string(r) + ")";
                    if (t->disabledCols.count(c))
                        tip += "\ndisabled (column " + std::to_string(c) + ")";
                    if (isActive) tip += "\nACTIVE - move_to_active_cell drives here";
                    tip += took ? "\nUSED - the cavity is empty" : "\nfull - holds a part";
                    tip += isActive ? "\nclick: clear the active cell"
                                    : "\nclick: make it the active cell (before it: used; it and after: full)";
                    tip += took ? "\nright-click: mark full, drive here, disable"
                                : "\nright-click: mark used, drive here, disable";
                    ImGui::SetTooltip("%s", tip.c_str());
                }
            }
        }
        ImGui::EndChild();
    }
}

void App::DrawTrayPanel()
{
    const bool busy = orch_.Busy();
    std::vector<WaffleTray>& list = session_.Trays();
    if (traySel_ >= static_cast<int>(list.size())) traySel_ = list.empty() ? -1 : 0;
    if (traySel_ < 0 && !list.empty()) traySel_ = 0;
    WaffleTray* t = (traySel_ >= 0) ? &list[static_cast<std::size_t>(traySel_)] : nullptr;

    const auto devConnected = [&](const std::string& name) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) != name) continue;
            const IMotionController* m = hardware_.MotionDevice(i);
            return m && m->IsConnected();
        }
        return false;
    };

    // ---- Layout: grid left, controls right, draggable splitter --------------
    constexpr float kSplitW  = 6.0f;
    constexpr float kMinGridW = 240.0f;
    constexpr float kMinCtlW  = 300.0f;

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (trayCtlW_ < kMinCtlW) trayCtlW_ = kMinCtlW;
    const float maxCtlW = avail.x - kSplitW - kMinGridW;
    if (trayCtlW_ > maxCtlW) trayCtlW_ = fmaxf(kMinCtlW, maxCtlW);
    const float rightW = trayCtlW_;
    const float gridW  = fmaxf(40.0f, avail.x - rightW - kSplitW);

    // ---- The grid -----------------------------------------------------------
    ImGui::BeginChild("trayGrid", ImVec2(gridW, avail.y), ImGuiChildFlags_Borders);
    if (!t) {
        ImGui::TextDisabled("No tray selected.");
    } else {
        DrawTrayGrid(t, busy);
    }
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##traysplit", true, kSplitW, avail.y)) {
        trayCtlW_ -= ImGui::GetIO().MouseDelta.x;
        trayCtlSplitting_ = true;
    } else if (trayCtlSplitting_) {
        trayCtlSplitting_ = false;
        SaveUiSettings();          // persist on drag END, like every other splitter
    }
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginChild("trayCtl", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);

    // ---- The trays ----------------------------------------------------------
    ImGui::SeparatorText("Trays");
    {
        ImGui::BeginDisabled(busy);
        if (ImGui::SmallButton("New")) {
            WaffleTray n;
            n.uid  = GenerateUid();
            n.name = "tray " + std::to_string(list.size() + 1);
            if (hardware_.MotionCount() > 0) n.device = hardware_.MotionName(0);
            list.push_back(std::move(n));
            traySel_ = static_cast<int>(list.size()) - 1;
            t = &list.back();
            trayMsg_.clear();
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!t);
        if (ImGui::SmallButton("Duplicate") && t) {
            WaffleTray copy = *t;         // by value: the push_back reallocates
            copy.uid   = GenerateUid();   // a copy is a NEW identity; corners and
            copy.name += " copy";         // masks come along (same fixture)
            list.push_back(std::move(copy));
            traySel_ = static_cast<int>(list.size()) - 1;
            t = &list.back();
            trayMsg_.clear();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete") && t) {
            workflow_.ClearTrayCursor(t->uid, "tray deleted");
            list.erase(list.begin() + traySel_);
            traySel_ = list.empty() ? -1 : 0;
            t = list.empty() ? nullptr : &list[0];
            trayMsg_.clear();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!session_.TraysDirty());
        // Stable id, changing label (the grip page says why).
        if (ImGui::SmallButton(session_.TraysDirty() ? "Save *###traysave"
                                                     : "Save###traysave"))
            session_.SaveTrays();
        ImGui::EndDisabled();
        ImGui::EndDisabled();

        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginListBox("##traylist", ImVec2(0.0f, 90.0f))) {
            for (int i = 0; i < static_cast<int>(list.size()); ++i) {
                ImGui::PushID(i);
                const WaffleTray& row = list[static_cast<std::size_t>(i)];
                char lbl[160];
                std::snprintf(lbl, sizeof(lbl), "%s   (%d x %d, %s)", row.name.c_str(),
                              row.rows, row.cols,
                              row.device.empty() ? "no device" : row.device.c_str());
                if (ImGui::Selectable(lbl, i == traySel_)) {
                    traySel_ = i;
                    trayMsg_.clear();
                }
                ImGui::PopID();
            }
            ImGui::EndListBox();
        }
    }

    if (!t) {
        ImGui::TextWrapped("No tray selected. A tray is an R x C array of cavities "
                           "taught from three corners; a recipe's move_to_cell step "
                           "drives to any cell of it by (row, col), and tray_next walks "
                           "the enabled cells one part at a time.");
        ImGui::EndChild();
        return;
    }

    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s", t->name.c_str());
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::InputText("##trayname", buf, sizeof(buf))) t->name = buf;
    }
    if (!trayMsg_.empty()) ImGui::TextWrapped("%s", trayMsg_.c_str());

    // ---- Device and size ----------------------------------------------------
    ImGui::SeparatorText("Device and size");
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(180.0f);
    if (ImGui::BeginCombo("device", t->device.empty() ? "(pick)" : t->device.c_str())) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string n = hardware_.MotionName(i);
            if (ImGui::Selectable(n.c_str(), n == t->device) && n != t->device) {
                // The corners are axis readings OF the old device: on another
                // controller the same numbers are a different place, so they
                // cannot follow. Cleared, said, and saved - like a teach.
                const bool hadCorners = t->origin.have || t->rowEnd.have || t->far.have;
                t->device = n;
                t->origin = TrayCorner{};
                t->rowEnd = TrayCorner{};
                t->far    = TrayCorner{};
                t->usedCells.clear();   // a re-taught tray starts full
                workflow_.ClearTrayCursor(t->uid, "tray device changed");
                session_.SaveTrays();
                trayMsg_ = hadCorners ? "device changed - the three corners were cleared, "
                                        "teach them again on " + n
                                      : "device set to " + n;
                FD_INFO("tray", "'%s': device -> %s%s", t->name.c_str(), n.c_str(),
                        hadCorners ? " (corners cleared)" : "");
            }
        }
        ImGui::EndCombo();
    }
    {
        int rows = t->rows, cols = t->cols;
        ImGui::SetNextItemWidth(110.0f);
        const bool r = ImGui::InputInt("rows", &rows);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        const bool c = ImGui::InputInt("cols", &cols);
        if (r || c) {
            t->rows = rows;
            t->cols = cols;
            TrayNormalize(*t);   // clamps, and drops masks the grid no longer has
        }
    }
    ImGui::EndDisabled();
    if (!t->device.empty() && !devConnected(t->device))
        ImGui::TextColored(kWarn, "'%s' is not connected", t->device.c_str());

    // ---- The three corners ----------------------------------------------------
    ImGui::SeparatorText("Corners");
    ImGui::TextWrapped("Jog into the cavity, then Teach here - or take a position the "
                       "product already has. Every axis the device reports is stored, "
                       "rotations included, so teach all three at one rotation.");
    const bool canTeach = !busy && !t->device.empty() && devConnected(t->device);
    const struct { TrayCornerSlot slot; const char* id; bool needed; } cards[] = {
        {TrayCornerSlot::Origin, "o", true},
        {TrayCornerSlot::RowEnd, "r", TrayNeedsRowEnd(*t)},
        {TrayCornerSlot::Far,    "f", TrayNeedsFar(*t)},
    };
    for (const auto& card : cards) {
        ImGui::PushID(card.id);
        const TrayCorner& corner = TrayCornerOf(*t, card.slot);
        if (card.needed)
            ImGui::TextUnformatted(TrayCornerLabel(card.slot));
        else
            ImGui::TextDisabled("%s - not needed for %d x %d", TrayCornerLabel(card.slot),
                                t->rows, t->cols);
        ImGui::BeginDisabled(!canTeach);
        if (ImGui::Button(corner.have ? "Re-teach here" : "Teach here", ImVec2(120.0f, 0.0f))) {
            std::map<std::string, double> pose;
            if (workflow_.DevicePose(t->device, pose))
                TeachTrayCorner(*t, card.slot, pose, "taught here");
            else
                trayMsg_ = "'" + t->device + "' is not connected";
        }
        ImGui::SameLine();
        if (const SavedPos* sp = TaughtPosePicker("##from", session_.Positions(), t->device))
            TeachTrayCorner(*t, card.slot, sp->vals,
                            (std::string("from taught position '") + sp->name + "'").c_str());
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || !corner.have);
        if (ImGui::Button("Go", ImVec2(50.0f, 0.0f))) {
            if (!jogArmed_) {
                trayMsg_          = "arm jog first - this drives a device";
                trayOpenArmPopup_ = true;
            } else if (!StartWatchedMove(t->device, corner.vals,
                                         std::string(TrayCornerLabel(card.slot)) + " of '" +
                                             t->name + "'")) {
                trayMsg_ = "'" + t->device + "' refused the move - is it connected?";
            } else {
                trayMsg_.clear();
            }
        }
        ImGui::EndDisabled();
        if (corner.have) {
            ImGui::TextDisabled("%s", PoseLine(corner.vals).c_str());
            ImGui::TextDisabled("taught %s", corner.taughtAt.empty() ? "(undated)"
                                                                     : corner.taughtAt.c_str());
        } else if (card.needed) {
            ImGui::TextColored(kWarn, "not taught");
        }
        ImGui::PopID();
    }

    // ---- What the corners say -------------------------------------------------
    ImGui::SeparatorText("Grid");
    {
        double colPitch = 0.0, rowPitch = 0.0;
        TrayPitch(*t, &colPitch, &rowPitch);
        if (TrayNeedsRowEnd(*t) && t->rowEnd.have)
            ImGui::Text("column pitch  %.4f mm  (%d columns)", colPitch, t->cols);
        if (TrayNeedsFar(*t) && t->far.have)
            ImGui::Text("row pitch     %.4f mm  (%d rows)", rowPitch, t->rows);
        const std::string why = TrayValidate(*t);
        if (why.empty())
            ImGui::TextColored(kGood, "every cell has a pose");
        else
            ImGui::TextColored(kWarn, "%s", why.c_str());
    }

    ImGui::SeparatorText("In a recipe");
    ImGui::TextWrapped("move_to_cell drives this tray's device to (row, col) - typed, or "
                       "read from ctx keys. move_to_active_cell drives to the cell "
                       "clicked on the grid (View > Waffle Tray window keeps the grid "
                       "beside the Run page). tray_next takes the next enabled cell that "
                       "still holds a part, marks it used and writes tray_row / "
                       "tray_col; put a move_to_cell with row_key = tray_row, col_key = "
                       "tray_col after it. Used cells are saved with the tray, so the "
                       "next run carries on where this one stopped. tray_reset, or All "
                       "full here, marks every cell full again.");

    // Said out loud: a Go with jog unarmed raises this alert. Safe as a modal:
    // every Go is disabled while orch_.Busy(), so no worker is behind a prompt.
    if (trayOpenArmPopup_) { ImGui::OpenPopup("Jog not armed##tray"); trayOpenArmPopup_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Jog not armed##tray", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored(kWarn, "Go drives '%s'.", t->device.c_str());
        ImGui::Text("Jog is not armed, so nothing moved.");
        ImGui::TextDisabled("Arm jog (the Arm checkbox in the Jog window, or the\n"
                            "button below), then click Go again.");
        ImGui::Spacing();
        if (ImGui::Button("Arm jog", ImVec2(120.0f, 0.0f))) {
            jogArmed_ = true;
            trayMsg_.clear();
            FD_INFO("tray", "jog armed from the Waffle Tray page's alert");
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(120.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    ImGui::EndChild();
}

} // namespace firedog
