// PositionHistory.h : the change journal for saved positions.
//
// One JSON line per changed, added, renamed or deleted position, appended
// beside the positions file it describes ({uid}.positions.history.jsonl in
// the product folder, config/positions.history.local.jsonl before a product
// is loaded). Entries are produced by DIFFING the map about to be written
// against the map last written or loaded — ProductSession::SavePositions is
// the one choke point every writer already goes through (the Motion page's
// buttons, the teach drain, teach-mode saves), so no caller records a change
// itself and none can forget to. Hidden (system) positions are not
// journalled: they are the alignment stations' private poses, re-taught by a
// procedure and not something an engineer asks "who changed" about.
//
// Append-only, kept for ever, human-readable: an entry is ~200 bytes and a
// product accumulates a few thousand over its life. Restore is not a special
// file operation — writing the old values back produces an ordinary entry
// whose source says it was a restore and of what.
#pragma once

#include "core/SavedPositions.h"

#include <map>
#include <string>
#include <vector>

namespace firedog {

struct PosChange {
    std::string at;        // UTC ISO-8601 with ms
    std::string by;        // operator id ("" when nobody is signed in)
    std::string source;    // "hand" | "recipe <name>" | "script <name>" | "teach" | "restore ..."
    std::string action;    // "add" | "update" | "rename" | "delete"
    std::string device;
    std::string uid;
    std::string name;      // the name AFTER the change (before, for a delete)
    std::string nameBefore;   // rename only
    std::map<std::string, double> before;   // update / delete: the old axis values
    std::map<std::string, double> after;    // add / update: the new axis values
};

// Diff two maps into journal entries (stamped with `at`/`by`/`source`).
// A position is matched by uid within its device. Values are compared at
// 1e-6 mm: a rewrite of the same numbers is not a change. A row that moved
// AND was renamed yields two entries, so each is restorable on its own.
std::vector<PosChange> DiffPositions(const SavedPosMap& before, const SavedPosMap& after,
                                     const std::string& by, const std::string& source);

// Append entries (JSONL). Best-effort; false if the file is unwritable.
bool AppendPositionHistory(const std::string& path, const std::vector<PosChange>& entries);

// Read the whole journal, oldest first. Missing file -> empty; a malformed
// line is skipped with a warning, never fatal.
std::vector<PosChange> LoadPositionHistory(const std::string& path);

// The journal path for a positions file: ".json" -> ".history.jsonl"
// (so "{uid}.positions.json" -> "{uid}.positions.history.jsonl", and
// "config/positions.local.json" -> "config/positions.local.history.jsonl").
std::string PositionHistoryPathFor(const std::string& positionsPath);

} // namespace firedog
