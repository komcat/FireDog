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

// Explicit-path variants used by the product system. `uidOut` (if given)
// receives the file's own "uid" field so the caller can check it against the
// product manifest; `uid` (if non-empty) is written into the file.
SavedPosMap LoadSavedPositionsFile(const std::string& path, std::string* uidOut = nullptr);
bool SaveSavedPositionsFile(const std::string& path, const SavedPosMap& positions,
                            const std::string& uid);

} // namespace firedog
