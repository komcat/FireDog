// WaffleTray.h : a grid of positions taught from three corners — a tray of
// parts in an R x C cavity array, addressed by (row, col) instead of by a
// saved position per cavity. Design: docs/waffle-tray-design.md.
//
// The record is PRODUCT state (the grips model): the tray is part of how a
// product is loaded, so it lives in `{uid}.trays.json` beside the product's
// positions, with the config/trays.local.json -> trays.json fallback when no
// product is loaded. Explicit Save + dirty baseline, like every document.
//
// The three corners are COORDINATES in the device's own axis space (every axis
// the controller reported, exactly what a SavedPos holds) and never position
// uids: a cell pose is DERIVED, and `TrayCellPose` below is the one place the
// arithmetic happens — the page's preview, `move_to_cell` and the cursor all
// call it, so there is exactly one answer to where cell (r,c) is.
//
// Geometry: a parallelogram. origin = (1,1), rowEnd = (1,C), far = (R,C):
//     cell(r,c) = origin + (c-1)/(C-1) * (rowEnd - origin)
//                        + (r-1)/(R-1) * (far - rowStart)
// where rowStart is rowEnd when C > 1 and origin when C == 1 (a 1-column tray
// has no row end, and far is then (R,1)). Applied per axis, rotations
// included — the corners are taught at one rotation, so U/V/W interpolate to
// a constant. A 1 x N tray needs origin + rowEnd, N x 1 origin + far, 1 x 1
// origin alone. Three points describe a rotated tray, not a keystoned one.
//
// No ImGui, no Workflow: this header is shared by the session, the command
// layer and the page.
#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

struct TrayCorner {
    bool have = false;
    std::map<std::string, double> vals;   // axis -> value, device axis space
    std::string taughtAt;                 // NowStamp() of the teach
};

enum class TrayCornerSlot { Origin, RowEnd, Far };
enum class TrayOrder { RowMajor, ColMajor, Serpentine };

using TrayCell = std::pair<int, int>;   // 1-based (row, col)

struct WaffleTray {
    std::string uid;
    std::string name = "tray";
    std::string device;   // the motion device the corners were taught on
    int rows = 1, cols = 1;
    TrayCorner origin, rowEnd, far;
    // Disabled = never offered by tray_next and refused by move_to_cell. A
    // row or column switches off every cell in it; a cell on its own.
    std::set<int>      disabledRows, disabledCols;
    std::set<TrayCell> disabledCells;
    // USED = the cavity is empty: its part was taken (by a tray_next, or an
    // operator marked it). Persisted WITH the tray, because it describes the
    // fixture on the machine and a restart mid-tray must not start the next
    // recipe at (1,1) again. The Workflow cursor is this set's live,
    // thread-safe mirror: seeded from here at product load, written back
    // through one App function (the page) or the TakeTrayUsed drain (a run).
    std::set<TrayCell> usedCells;
};

// Two teaches of one spot. A pitch below this is not a tray, it is the same
// corner captured twice, and a grid built on it would put every cell on top
// of the origin.
inline constexpr double kTrayMinPitchMm = 0.010;

// The largest grid the record accepts. A mistyped count must be refused
// before anything is drawn or moved — same reason the program organizer caps
// its queue.
inline constexpr int kTrayMaxRows = 1000;
inline constexpr int kTrayMaxCols = 1000;

// ---- lookup ----------------------------------------------------------------
// Uid first, then exact name — the convention every document record follows.
const WaffleTray* FindWaffleTray(const std::vector<WaffleTray>& list,
                                 const std::string& uidOrName);
WaffleTray*       FindWaffleTray(std::vector<WaffleTray>& list,
                                 const std::string& uidOrName);

// ---- corners ---------------------------------------------------------------
const char*  TrayCornerLabel(TrayCornerSlot s);   // "origin (1,1)", ...
TrayCorner&       TrayCornerOf(WaffleTray& t, TrayCornerSlot s);
const TrayCorner& TrayCornerOf(const WaffleTray& t, TrayCornerSlot s);
bool TrayNeedsRowEnd(const WaffleTray& t);   // cols > 1
bool TrayNeedsFar(const WaffleTray& t);      // rows > 1

// ---- geometry --------------------------------------------------------------
// "" when the tray can yield a pose for every cell; else why not — rows/cols
// out of range, a corner the size needs untaught, a corner whose axis set is
// not the origin's, a pitch under kTrayMinPitchMm, or row and column
// directions that are parallel. One implementation: the page prints it and
// move_to_cell fails on it.
std::string TrayValidate(const WaffleTray& t);
bool TrayCellInRange(const WaffleTray& t, int r, int c);
bool TrayCellEnabled(const WaffleTray& t, int r, int c);   // in range && not disabled
bool TrayCellUsed(const WaffleTray& t, int r, int c);      // in usedCells
// The pose of cell (r,c). False when TrayValidate fails or the cell is out of
// range; `out` is only written on true.
bool TrayCellPose(const WaffleTray& t, int r, int c, std::map<std::string, double>* out);
// Distance between neighbouring cells along a row (colPitch) and between rows
// (rowPitch), over X/Y/Z, for the readout. 0 for a dimension of size 1.
bool TrayPitch(const WaffleTray& t, double* colPitchMm, double* rowPitchMm);
// Every enabled cell, in `order`. Serpentine is row-major with every second
// row reversed — the walk a pick-and-place makes without a long return leg.
std::vector<TrayCell> TrayEnabledCells(const WaffleTray& t, TrayOrder order);
std::size_t TrayCellCount(const WaffleTray& t);   // rows * cols
// Clamp rows/cols into range and drop disabled rows/cols/cells the grid no
// longer has. Called by whoever changed the size, never per frame.
void TrayNormalize(WaffleTray& t);

const char* TrayOrderName(TrayOrder o);
bool        TrayOrderFromName(const std::string& s, TrayOrder* out);

// ---- persistence -----------------------------------------------------------
nlohmann::json WaffleTrayToJson(const WaffleTray& t);
WaffleTray     WaffleTrayFromJson(const nlohmann::json& j);

// config/trays.local.json, falling back to config/trays.json — the no-product
// fallback, same rule as grips.
std::vector<WaffleTray> LoadWaffleTrays();

// The explicit-path variants the product system uses. `uidOut` receives the
// file's own uid so the caller can check it against the manifest.
std::vector<WaffleTray> LoadWaffleTraysFile(const std::string& path,
                                            std::string* uidOut = nullptr);
bool SaveWaffleTraysFile(const std::string& path, const std::vector<WaffleTray>& list,
                         const std::string& uid);

} // namespace firedog
