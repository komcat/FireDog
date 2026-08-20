// SavedPositions.h : saved/taught motion positions, persisted per device.
//
// One consolidated JSON for all devices (keyed by config device name), stored
// as `config/positions.local.json` next to the exe with a `positions.json`
// fallback for shipped defaults — the project's *.local.json convention.
// Tolerant load: a missing or malformed file yields an empty map, never throws.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace firedog {

// A saved/taught device position (name + per-axis values).
struct SavedPos {
    char name[32] = "Pos";                // display label — freely renameable
    std::string uid;                      // stable identity, never changes —
                                          // recipes/programs reference this
    std::map<std::string, double> vals;   // axis -> value
    bool hidden = false;                  // system position — not shown by default
};

// Device name -> ordered list of taught positions.
using SavedPosMap = std::map<std::string, std::vector<SavedPos>>;

// Reads config/positions.local.json, falling back to config/positions.json.
SavedPosMap LoadSavedPositions();

// Writes config/positions.local.json. Best-effort; false if unwritable.
bool SaveSavedPositions(const SavedPosMap& positions);

// The saved position closest to a live device pose — what turns four decimal
// places of axis readout into a name an operator recognises. Distance is
// Euclidean over the axes the position and the pose SHARE; a position that
// shares no axis with the pose is not a candidate, so a partial reading still
// ranks. `distMM` (if given) receives that distance. Null when nothing ranks.
//
// Hidden positions are candidates by DEFAULT: this answers "where is the
// stage", and withholding the true nearest would name a neighbour instead and
// call it home. `hidden` governs the recipe PICKER, not the truth. Pass
// includeHidden=false only where the system names (`algn_...`) are noise the
// reader cannot act on — and accept that the answer may then be a neighbour.
const SavedPos* NearestSavedPos(const std::vector<SavedPos>& positions,
                                const std::map<std::string, double>& pose,
                                double* distMM = nullptr,
                                bool includeHidden = true);

// Explicit-path variants used by the product system. `uidOut` (if given)
// receives the file's own "uid" field so the caller can check it against the
// product manifest; `uid` (if non-empty) is written into the file.
SavedPosMap LoadSavedPositionsFile(const std::string& path, std::string* uidOut = nullptr);
bool SaveSavedPositionsFile(const std::string& path, const SavedPosMap& positions,
                            const std::string& uid);

} // namespace firedog
