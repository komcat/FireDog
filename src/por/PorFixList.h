// PorFixList.h : what stops the TARGET from running the MASTER's deck - the
// to-do list, as opposed to the differences (docs/por-design.md §6b).
//
// A difference says the target is not the same as the master; it may be fine,
// and a deploy can carry the structure across. An ERROR says the master's deck
// cannot run here as it stands, and almost every one is something a deploy will
// NEVER carry, because it is measured or machine data: a position, an
// alignment, a detection model, a taught point, a hardware.json name, a map.
// Those are taught or configured on this machine by hand, so they are listed
// as a fix list, grouped by what fixes them.
//
// The list answers "after the master's structure is here, what is still
// missing?", so it walks the MASTER's deck (every program > slot > node >
// recipe, sub_recipe and on_fail followed, disabled steps included) and checks
// each reference BY NAME on the target - exactly how a deploy would translate
// it. A recipe, a sub_recipe, a product script or a product picture the target
// lacks is NOT an error: the deploy brings it. A reference the MASTER cannot
// resolve is listed first: the plan of record itself is broken and must be
// fixed there before it is distributed.
#pragma once

#include "por/PorPositions.h"

#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

enum class PorFixKind {
    MasterBroken,   // the master's own reference does not resolve
    Teach,          // a position, alignment, model, grip, tray or point to teach HERE
    Configure,      // hardware.json, a map, a machine script: set this machine up
    Names,          // a name used twice, so a reference cannot be told apart
    UpdateExe,      // a command this FireDog build does not know
};
const char* PorFixKindTitle(PorFixKind k);

enum class PorFixJump { None, Recipe, Positions, Hardware };

struct PorFixItem {
    PorFixKind               kind = PorFixKind::Teach;
    std::string              what;   // the object, named
    std::string              fix;    // what to do about it
    std::vector<std::string> uses;   // "Siphog888 > PROD V2 > Load > 3.2 move_to_position"
    PorFixJump               jump = PorFixJump::None;
    std::string              recipe;    // Recipe: the first recipe that uses it
    std::string              section;   // Hardware: the section title
    // Teach only: a deploy can go ahead and leave this to be taught here - a
    // position (it becomes an untaught placeholder), a point or a step's own
    // detection model (written untaught), a private pose of an owner the target
    // has. False for a missing alignment, nominal, named model, grip or tray: a
    // deploy has nothing to translate the reference to (§7.2; phase 4d).
    bool                     teachAfterDeploy = false;
};

struct PorFixList {
    std::vector<PorFixItem> items;   // grouped by kind, in PorFixKind order
    int Count(PorFixKind k) const;
    int TargetCount() const;         // everything but MasterBroken
};

// `positions` is ComparePositions' report for the same two sides: the position
// errors come from its trace, so the two pages can never disagree.
PorFixList BuildFixList(const PorSide& master, const PorSide& target, const CommandRegistry& reg,
                        const PorPositionsReport& positions,
                        const std::string& shippedScriptsDir = "");

} // namespace firedog
