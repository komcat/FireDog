// StepSummary.h : the one-line description of a recipe step, as a person reads it.
//
// "move_to_position hex-left Home", "delay 5 s", "run UV cure", "if power < 0.0015".
// The Recipe Editor's step tree shows it whenever a step has no nickname (and the
// inspector's "gen" button writes it into the nickname); the POR compare tool shows
// it on every row. ONE function, so the two describe a step identically.
//
// It reads the documents it is handed and nothing else — no App, no session, no UI —
// which is why it lives in firedog_cmd. A reference whose document is not given
// (a null pointer), or that does not resolve, is shown as the raw uid.
#pragma once

#include "command/Recipe.h"

#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

// The documents a summary resolves uids against. Every pointer may be null.
struct StepSummaryDocs {
    const CommandRegistry*          registry  = nullptr;   // null: every command reads as its type
    const std::vector<Recipe>*      recipes   = nullptr;   // sub_recipe uid -> name
    const SavedPosMap*              positions = nullptr;   // PositionUid -> name (within "device")
    const std::vector<AlignProc>*   aligns    = nullptr;   // NominalUid -> name (within "alignment")
    const std::vector<GripProfile>* grips     = nullptr;   // GripUid -> name
    const std::vector<WaffleTray>*  trays     = nullptr;   // TrayUid -> name
};

// The live label for a step: structural steps by their shape, commands by their
// type plus up to three non-plumbing params (timeouts, tolerances, poll rates are
// skipped), with uids resolved to names.
std::string StepSummary(const RecipeStep& s, const StepSummaryDocs& docs);

// A millisecond duration the way a person reads it: "750 ms", "5 s",
// "1 m 30.5 s", "1 h 2 m". Zero parts are left out. The editor's DurationMs
// readout and the step label both use it, so they read alike.
std::string FormatDurationMs(long long ms);

} // namespace firedog
