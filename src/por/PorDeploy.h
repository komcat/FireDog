// PorDeploy.h : the deploy PLAN - the master's structure translated onto the
// target's own objects, in memory (docs/por-design.md §7.2-§7.4, phase 4b).
//
// Nothing here writes a byte. PlanDeploy answers two questions:
//   * may a deploy run? Only when the Fix list holds nothing but what a deploy
//     leaves to be taught (§7.2) - everything else is a BLOCKER;
//   * what would it write? The target's recipe and program documents after the
//     deploy, its positions with the placeholders appended, and the product
//     files to copy - plus a summary of each for the preview.
//
// Translation (§7.4): every reference is rewritten from the master's uid to the
// target's uid of the same NAME, per ParamType, the way the compare resolves
// them. Measured values travel with the TARGET's step: recipes pair by name and
// their steps by PorPairSteps - the compare's own pairing - and a paired step's
// taught points and detect_object model are carried into the written step.
// Unpaired, a point becomes {} (the untaught point) and a model "". A position
// the target lacks becomes an untaught placeholder (SavedPos::untaught) with a
// fresh uid and the master's name, hidden flag and axis values.
#pragma once

#include "por/PorFixList.h"
#include "por/PorSide.h"

#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

struct PorDeployOptions {
    // Strict mirror: drop the target's recipes and programs the master does
    // not have. Off by default - a local experiment is work somebody did (§7.3).
    bool removeTargetOnly = false;
};

enum class PorPlanAction {
    Add,       // new on the target
    Replace,   // the target has it; the written one differs
    Same,      // the target has it; the written one is identical
    Keep,      // the target's own, the master has none: left as it is
    Remove,    // the target's own, dropped by the strict mirror
};
const char* PorPlanActionName(PorPlanAction a);

struct PorPlanItem {
    PorPlanAction action = PorPlanAction::Add;
    std::string   name;
    std::string   detail;
};

struct PorPlaceholder {
    std::string device, name, uid;
    std::vector<std::string> uses;   // "recipe 'Load' > 3 move_to_position"
};

struct PorDeployPlan {
    // Why a deploy may not run (§7.2): fix-list items a deploy cannot carry,
    // plus whole-side conditions (no product on the master, none on the target
    // yet - a first deploy comes in phase 4d).
    std::vector<PorFixItem>  blockers;
    // What the plan could not translate outside the deck (a recipe no program
    // runs, referring to something the target lacks): written as the master had
    // it, listed here so nobody is surprised.
    std::vector<std::string> warnings;

    // ---- The documents a deploy would write (the target's whole files) ----
    std::vector<Recipe>      recipes;
    std::vector<Program>     programs;
    SavedPosMap              positions;   // the target's, plus the placeholders
    std::vector<std::string> scripts;     // product script file names to copy from the master
    std::vector<std::string> images;      // product prompt pictures to copy

    // ---- The preview ----
    std::vector<PorPlanItem>    recipeItems, programItems, fileItems;
    std::vector<PorPlaceholder> placeholders;
    std::vector<std::string>    untaughtPoints;   // "recipe 'Load' > 2.1 detect_object: view_point"
    std::vector<std::string>    untaughtModels;
    int carriedPoints = 0, carriedModels = 0;

    bool Blocked() const { return !blockers.empty(); }
};

// `fixes` is BuildFixList's result for the same two sides; the blockers are
// taken from it, so the Fix list and the deploy window never disagree.
PorDeployPlan PlanDeploy(const PorSide& master, const PorSide& target, const CommandRegistry& reg,
                         const PorFixList& fixes, const PorDeployOptions& opt = {});

} // namespace firedog
