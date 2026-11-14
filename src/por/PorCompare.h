// PorCompare.h : the master-vs-target compare of a product's STRUCTURE — the
// deck (programs > slots > folders > recipe nodes) and the recipes (recipe >
// steps > params). Pure C++: two loaded sides and the command registry in, a
// tree of paired rows out; the tool's UI only draws it (docs/por-design.md §3-§4).
//
// Uids are minted per machine, so NOTHING is matched by uid:
//   * programs and recipes by name (a name used twice on a side is AMBIGUOUS
//     and left unpaired - guessing which "Home" was meant is the mistake this
//     tool exists to prevent);
//   * slots, folder/recipe nodes and steps by SEQUENCE ALIGNMENT, like a text
//     diff - steps have no identity at all, so a step pairs by (type, nickname)
//     and, between those anchors, with the next unpaired step of the same type;
//   * every reference inside a param is translated to a NAME on its own side
//     before the two are compared, and checked to exist on the target.
#pragma once

#include "por/PorSide.h"

#include <map>
#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

// Ascending severity: a row's `worst` is the max over itself and its children.
enum class PorStatus {
    Same,         // identical after normalisation
    Measured,     // differs only in a value measured on each machine (PointXyz, a private model)
    Unknown,      // a command this build does not know - compared as raw JSON
    Changed,      // a design value differs
    OnlyTarget,   // present on the target only
    OnlyMaster,   // present on the master only
    Ambiguous,    // a name used twice on a side, so it cannot be paired
    Missing,      // a reference that does not resolve (on the target, or on the master)
};
const char* PorStatusName(PorStatus s);

enum class PorParamClass {
    Design,      // a value the author chose; a difference is a difference
    Reference,   // names another object; compared by that object's NAME
    Measured,    // taught on the machine; shown, never judged
    Raw,         // a key the schema does not declare, or an unknown command's param
    Machine,     // this machine's identity (an IP, a serial, a VISA address): shown, never judged
};

struct PorParam {
    std::string   name;
    std::string   master, target;          // display text
    bool          hasMaster = false, hasTarget = false;
    PorParamClass cls    = PorParamClass::Design;
    PorStatus     status = PorStatus::Same;
    std::string   note;                    // "missing on target", "Δ 0.42 mm", ...
};

enum class PorRowKind {
    Program, Slot, Folder, RecipeNode, Recipe, Step, Else,   // the deck and the recipes
    Section, Device, Item,                                   // the hardware compare
};

struct PorRow {
    PorRowKind  kind   = PorRowKind::Step;
    PorStatus   status = PorStatus::Same;  // this row's own
    PorStatus   worst  = PorStatus::Same;  // this row and everything beneath it
    std::string master, target;            // one line each; "" = absent on that side
    bool        masterOff = false, targetOff = false;   // a disabled step
    std::string recipe;                    // RecipeNode / Recipe: the recipe's name
    bool        working = false;           // Recipe: reached from a deck on either side
    bool        used    = false;           // Device / Item: the master's deck uses it
    std::vector<PorParam> params;          // field-by-field, for an expanded row
    std::vector<PorRow>   children;
};

struct PorReport {
    std::vector<PorRow>      deck;      // one Program row each
    std::vector<PorRow>      recipes;   // one Recipe row each: master order, then target-only
    std::vector<std::string> notes;
};

// `shippedScriptsDir`: the config/scripts folder beside THIS tool's exe (the
// shipped copies a ScriptFile can resolve to); "" = not checked there.
PorReport ComparePor(const PorSide& master, const PorSide& target,
                     const CommandRegistry& reg, const std::string& shippedScriptsDir = "");

// ---- Shared with the positions compare -------------------------------------

// Align two key sequences like a text diff: the longest common subsequence of
// `a` and `b`, then - between two anchors - an unpaired a[i] pairs with the next
// unpaired b[j] whose `a2[i] == b2[j]` (empty a2/b2 = no second pass). Returns
// (i, j) pairs in display order; -1 marks the side that has nothing.
std::vector<std::pair<int, int>> PorAlign(const std::vector<std::string>& a,
                                          const std::vector<std::string>& b,
                                          const std::vector<std::string>& a2 = {},
                                          const std::vector<std::string>& b2 = {});

// THE step pairing (§3): two step lists lined up by (type, nickname), then -
// between those anchors - by type alone. The compare, the fix list and the
// deploy all call it, so "this master step is that target step" has one answer
// and the preview, the teach list and the written result cannot disagree.
std::vector<std::pair<int, int>> PorPairSteps(const std::vector<RecipeStep>& a,
                                              const std::vector<RecipeStep>& b);

// Every step of `a` (children and else branches too) mapped to the step of `b`
// PorPairSteps pairs it with; an unpaired step is absent from the map. The
// pointers point into `a` and `b`, which must outlive the map.
using PorStepTwins = std::map<const RecipeStep*, const RecipeStep*>;
void PorCollectTwins(const std::vector<RecipeStep>& a, const std::vector<RecipeStep>& b,
                     PorStepTwins* out);

// A position's name, or nullptr. `count` (optional) receives how many
// positions on that device carry the SAME name - more than one is ambiguous.
const SavedPos* PorFindPosByUid(const SavedPosMap& m, const std::string& device,
                                const std::string& uid);
const SavedPos* PorFindPosByName(const SavedPosMap& m, const std::string& device,
                                 const std::string& name, int* count = nullptr);

} // namespace firedog
