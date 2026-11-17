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

// One POSITION VARIANT's own values for a saved position. A product made in
// models that are almost the same part keeps ONE position (one name, one uid,
// every recipe reference) and teaches only what differs per model. A run picks
// the variant (the program header / the editor's combo) and the values are
// swapped in ONCE, when its Env is built (ApplyPositionVariant), so no mover
// ever learns what a variant is. docs/position-variants-design.md.
struct SavedPosVariant {
    std::map<std::string, double> vals;   // axis -> value
    bool untaught = false;                // a placeholder, same meaning as SavedPos::untaught
};

// A saved/taught device position (name + per-axis values).
struct SavedPos {
    char name[32] = "Pos";                // display label — freely renameable
    std::string uid;                      // stable identity, never changes —
                                          // recipes/programs reference this
    std::map<std::string, double> vals;   // axis -> value
    bool hidden = false;                  // system position — not shown by default
    // A PLACEHOLDER: the row exists (a POR deploy added it because the deck
    // names it) but nobody has taught it on THIS machine, so `vals` are another
    // machine's numbers, kept for reference only. Every mover refuses it, every
    // Go is disabled on it, and a teach of it starts where the device already
    // stands. Any write of real values (a teach, Update, Edit) clears it.
    // docs/por-design.md §7.8.
    bool untaught = false;
    // Variant name -> that variant's own values. `vals` above IS the base
    // variant; a variant with no entry here falls back to it (EffectiveVals).
    std::map<std::string, SavedPosVariant> variants;
};

// Device name -> ordered list of taught positions.
using SavedPosMap = std::map<std::string, std::vector<SavedPos>>;

// How close (Euclidean, over the axes the pose and the record share) a device
// must sit to a taught position to be said to be AT it. One number for every
// readout that names where a device is (the Aux plate, the Jog box), so two
// panels never disagree about whether the stage has arrived.
inline constexpr double kSavedPosAtTolMM = 0.050;

// The values a position has under `variant` ("" = base): the variant's own
// entry when the row has one, else the base values. The ONE place a variant is
// resolved - the run overlay below and every hand-driven surface call it.
const std::map<std::string, double>& EffectiveVals(const SavedPos& p, const std::string& variant);
bool EffectiveUntaught(const SavedPos& p, const std::string& variant);
// True when the row carries its own entry for `variant` (never for base).
bool HasVariant(const SavedPos& p, const std::string& variant);

// The positions a run under `variant` sees: a copy whose every row holds its
// EFFECTIVE values and untaught flag, with `variants` emptied (the run copy is
// flat, so a teach inside it writes plain `vals`). "" flattens to base. Called
// by ProductSession::MakeRecipeEnv / MakeLuaEnv and by the views that preview
// a run (teach pick, Inspect Motion).
SavedPosMap ApplyPositionVariant(const SavedPosMap& positions, const std::string& variant);

// The one refusal text for driving to an untaught position, so a recipe step, a
// script and a page button say the same thing.
std::string UntaughtPositionMessage(const std::string& device, const SavedPos& p);

// Reads config/positions.local.json, falling back to config/positions.json.
// `variantsOut` (if given) receives the file's declared variant names.
SavedPosMap LoadSavedPositions(std::vector<std::string>* variantsOut = nullptr);

// Writes config/positions.local.json. Best-effort; false if unwritable.
// `variants` is the declared variant list, written at the top level.
bool SaveSavedPositions(const SavedPosMap& positions,
                        const std::vector<std::string>& variants = {});

// The saved position closest to a live device pose — what turns four decimal
// places of axis readout into a name an operator recognises. Distance is
// Euclidean over the axes the position and the pose SHARE; a position that
// shares no axis with the pose is not a candidate, so a partial reading still
// ranks. `distMM` (if given) receives that distance. Null when nothing ranks.
//
// Untaught positions are never candidates: their values are another machine's,
// so naming the stage by one would name a place this machine never stood.
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
// `variantsOut` / `variants`: the declared variant names (top-level
// "variants"; base is implicit and never listed).
SavedPosMap LoadSavedPositionsFile(const std::string& path, std::string* uidOut = nullptr,
                                   std::vector<std::string>* variantsOut = nullptr);
bool SaveSavedPositionsFile(const std::string& path, const SavedPosMap& positions,
                            const std::string& uid,
                            const std::vector<std::string>& variants = {});

} // namespace firedog
