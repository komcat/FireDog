// Recipe.h : layer 2.5 — the Tier-1 orchestration artifact.
//
// A recipe is a NESTED list of steps (children arrays, not bobcat's flat list
// with begin/end markers — malformed nesting is impossible by construction).
// Leaf steps name a registered command; six structural types are interpreted
// by the engine itself: loop / if / group / parallel / sub_recipe / stop_recipe.
//
// Persistence follows the project conventions: one consolidated file
// (config/recipes.local.json -> recipes.json fallback), or the product
// manifest's "recipes" role when a product is loaded. Tolerant load: a
// missing or malformed file yields an empty list, never throws.
#pragma once

#include "core/AlignFrame.h"
#include "core/GripProfile.h"
#include "core/WaffleTray.h"
#include "vision/VisionModel.h"
#include "core/MotionMap.h"
#include "core/SavedPositions.h"
#include "command/CommandSpec.h"   // CommandOutputs (CollectStepOutputs)

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <cstdint>

namespace firedog {

class CommandRegistry;
class HardwareManager;

// Step nesting cap — guards the engine's recursive executor against stack
// overflow from hostile/hand-written JSON. Checked at validation.
inline constexpr int kMaxRecipeDepth = 32;
// Backstop for while-form loops that never converge.
inline constexpr int kDefaultMaxIterations = 10000;

struct RecipeStep {
    std::string    type;                              // command or structural type
    std::string    nickname;                          // optional display label
    bool           enabled = true;                    // disabled: skipped, kept
    nlohmann::json params  = nlohmann::json::object();
    // Per-VARIANT overrides of params that opted in (ParamSpec::variant):
    // {"65C": {"volts": 2.8, "amps": 0.3}}. `params` stays the base. Applied
    // once at run start (ApplyStepVariants), never read by a command. JSON
    // "variants", omitted when empty. docs/position-variants-design.md §6.
    nlohmann::json variants = nlohmann::json::object();

    std::vector<RecipeStep> children;                 // loop / if / group / sub_recipe
    std::vector<RecipeStep> elseChildren;             // if only

    enum class OnError { Abort, Continue, Retry };
    OnError onError    = OnError::Abort;
    int     retryCount = 0;                           // extra attempts when Retry
};

struct Recipe {
    std::string uid;
    std::string name = "recipe";
    std::string description;
    // Editor bookkeeping, persisted with the recipe. `hidden` withholds it
    // from the lists (the editor's, the organizer's library) and nothing else:
    // a program that references it by uid still runs it. `tags` are free
    // labels the lists filter on; a tag is a fact about how the author files
    // the recipe, not about what it does, so nothing else reads them.
    bool hidden = false;
    std::vector<std::string> tags;
    // On-fail recipe (uid into the library; "" = none, the default): what
    // RecipeEngine runs when THIS recipe, launched as a run, ends Failed - a
    // failed step or a failing stop_recipe. Meant for "get the tool out of the
    // part" (a pick fails half way: run LEFT home). Never on an operator Stop
    // or a safety abort - those mean "stop moving" - never for a teach run,
    // and never chained: the on-fail recipe's own on-fail is ignored. Only the
    // LAUNCHED recipe's is honoured; a sub_recipe is inlined and has none.
    std::string onFailUid;
    std::vector<RecipeStep> steps;
};

bool RecipeHasTag(const Recipe& r, const std::string& tag);
// "a, b ,c" -> {"a","b","c"}: trimmed, empties dropped, duplicates dropped.
std::vector<std::string> ParseTagList(const std::string& text);

// Structural step types interpreted by the engine (not registry commands).
bool IsStructuralStep(const std::string& type);

// Palette metadata for the structural types (the editor's add-step palette
// lists these alongside the registry commands).
struct StructuralStepInfo {
    const char* type;
    const char* description;
};
inline constexpr StructuralStepInfo kStructuralSteps[] = {
    {"group",       "Organizational container - runs its children in order"},
    {"parallel",    "One lane per device, lanes run AT ONCE, each lane's moves in "
                    "order; the block ends when every lane is done"},
    {"loop",        "Repeat children a fixed count, or while a condition holds"},
    {"if",          "Run children when a condition holds; else-branch otherwise"},
    {"sub_recipe",  "Run another recipe inline (referenced by uid)"},
    {"stop_recipe", "End the run early, as success or failure"},
};

// ---- Persistence ------------------------------------------------------------
// File shape: { "uid": "...", "recipes": [ {...}, ... ] }.
std::vector<Recipe> LoadRecipesFile(const std::string& path, std::string* uidOut = nullptr);
std::vector<Recipe> LoadRecipes();   // config/recipes.local.json -> recipes.json
bool SaveRecipesFile(const std::string& path, const std::vector<Recipe>& recipes,
                     const std::string& uid);

nlohmann::json RecipeToJson(const Recipe& r);
Recipe         RecipeFromJson(const nlohmann::json& j);   // tolerant
// One step (subtree included) — the editor's step clipboard round-trips
// through these. A step has no identity of its own (only a recipe carries a
// uid; a sub_recipe step's `uid` param names the recipe it CALLS), so a
// pasted copy needs nothing regenerated.
nlohmann::json StepToJson(const RecipeStep& s);
RecipeStep     StepFromJson(const nlohmann::json& j);     // tolerant

// ---- Validation -------------------------------------------------------------
// Numeric step-path convention (shared with the editor UI): a component
// >= kElseBranchBase addresses an if-step's ELSE branch (component -
// kElseBranchBase = index into elseChildren).
inline constexpr std::size_t kElseBranchBase = static_cast<std::size_t>(1) << 20;

struct RecipeIssue {
    std::string where;     // step path, e.g. "3.2.1 (move_to_position)"
    std::string message;
    std::vector<std::size_t> path;   // numeric path (empty = recipe-level issue)
};

// Structure + params + reference resolution. With requireConnected also checks
// that every referenced device/instrument is Connected (run-start validation —
// recipes never connect hardware; session setup is the operator's job).
// `library` resolves sub_recipe uids. Empty result = valid.
std::vector<RecipeIssue> ValidateRecipe(const Recipe& r, const CommandRegistry& reg,
                                        const HardwareManager& hw,
                                        const SavedPosMap& positions,
                                        const MotionMapSet& maps,
                                        const std::vector<AlignProc>& aligns,
                                        const std::vector<VisionModel>& visionModels,
                                        const std::vector<GripProfile>& grips,
                                        const std::vector<WaffleTray>& trays,
                                        const std::vector<Recipe>& library,
                                        bool requireConnected);

// Replace every sub_recipe step's children with the referenced recipe's steps
// (recursively, cycle-checked against the inline chain). False + err on an
// unknown uid, a cycle, or depth overflow.
bool InlineSubRecipes(Recipe& r, const std::vector<Recipe>& library,
                      std::string* err);

// ---- Declared outputs ------------------------------------------------------
// What the steps of a recipe declare they write (CommandSpec::outputs), in run
// order: depth-first, children before the next sibling, a then-branch before
// its else. Steps with no declaration are left out. With `before` set, the walk
// stops at that step path and returns only the steps that run BEFORE it - what
// a step there can rely on. A loop's later iterations are not modelled: a step
// inside a loop sees only what precedes it in the written order.
struct StepOutputs {
    std::vector<std::size_t> path;   // the step-path convention above
    std::string              type;
    std::string              nickname;
    bool                     enabled = true;   // false: the step (or a parent) is disabled
    CommandOutputs           outs;
};
std::vector<StepOutputs> CollectStepOutputs(const std::vector<RecipeStep>& steps,
                                            const CommandRegistry& reg,
                                            const std::vector<std::size_t>* before = nullptr);

// A step path as the editor numbers it: "3.2", "1.else.2" (1-based).
std::string StepPathLabel(const std::vector<std::size_t>& path);

// Count enabled leaf (command) steps for progress display. Fixed-count loops
// multiply; while-form loops count their body once (iterations unknown).
std::size_t CountLeafSteps(const std::vector<RecipeStep>& steps);

// The VERSION of what a run executes, for cycle-time history
// (docs/cycle-time-design.md §4): FNV-1a 64 over StepToJson of every step with
// `nickname` dropped - renaming a row changes nothing that runs; a param, an
// enabled flag, an on_error or a sub-recipe body does. Hash the INLINED steps.
std::uint64_t RecipeStepsHash(const std::vector<RecipeStep>& steps);
// A copy of `r` inlined against `library`, then hashed - what RecipeEngine::Start
// hashes for an ordinary run. 0 when the inline fails (that run is refused anyway).
std::uint64_t RecipeRunHash(const Recipe& r, const std::vector<Recipe>& library);

// The step half of a POSITION VARIANT run (docs/position-variants-design.md
// §6): every step's override for `variant` is written over its params - only
// for params that opted in (ParamSpec::variant; anything else is left for
// validation to have refused) - and every step's `variants` is then CLEARED,
// so what runs is flat and no command ever learns what a variant is. "" (base)
// only clears. Recursive (children, else). Applied by the engines AFTER
// sub-recipes are inlined, so the inlined bodies get it too, and after the run
// hash, which already covers the overrides. Returns how many params changed.
std::size_t ApplyStepVariants(std::vector<RecipeStep>& steps, const std::string& variant,
                              const CommandRegistry& reg);

} // namespace firedog
