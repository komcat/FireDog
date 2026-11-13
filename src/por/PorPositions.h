// PorPositions.h : the Positions page of a POR compare (docs/por-design.md §6).
//
// A position matters in proportion to whether the deck drives to it, so the
// table starts from a TRACE of the master's deck: every program > slot > node >
// recipe (sub_recipe and on_fail followed) > every step, ENABLED OR NOT (a
// disabled step is one click from moving), collecting
//   * every PositionUid param, on the step's device;
//   * through an alignment: its two station poses and its nominals' camera poses;
//   * through a grip profile: its inspect pose;
//   * through a map: every node of it.
// A product script that finds a position by name (fd.positions) cannot be
// traced; such scripts are LISTED, so the table never claims to be complete.
//
// Positions are matched by device + name - except a private pose (named after
// its owner's uid), which is matched through its owner: the target's alignment
// or grip profile of the same name, and that owner's own pose.
#pragma once

#include "por/PorCompare.h"

#include <map>
#include <string>
#include <vector>

namespace firedog {

enum class PorPosTier {
    Deck,         // the master's deck drives to it
    Private,      // a pose owned by an alignment / grip the deck uses
    Unused,       // a visible master position the deck does not use
    TargetOnly,   // a visible target position with no master counterpart
};

struct PorPosRow {
    PorPosTier  tier = PorPosTier::Deck;
    std::string device;
    std::string name;      // the master's name (the target's, for TargetOnly)
    std::string owner;     // Private: "alignment Bench station 1", "grip Lens inspect"
    PorStatus   status = PorStatus::Same;
    bool        hasMaster = false, hasTarget = false;
    std::map<std::string, double> master, target;   // axis -> value
    double      dXyz   = 0.0;    // |Δ| over the X/Y/Z both sides have, mm
    double      dRot   = 0.0;    // max |Δ| over U/V/W, deg
    bool        axesDiffer    = false;
    bool        ambiguous     = false;   // the name is used twice on a side
    bool        overThreshold = false;
    std::vector<std::string> uses;       // "Siphog888 > PROD V2 > Load > 3.2 move_to_position"
    std::string note;
};

struct PorPositionsReport {
    std::vector<PorPosRow>   rows;            // Deck (first-use order), Private, Unused, TargetOnly
    std::vector<std::string> untraced;        // product scripts the deck runs, not followed
    std::vector<std::string> mapIssues;       // a required map missing on the target, ...
};

PorPositionsReport ComparePositions(const PorSide& master, const PorSide& target,
                                    const CommandRegistry& reg,
                                    double mmThreshold = 1.0, double degThreshold = 0.1);

} // namespace firedog
