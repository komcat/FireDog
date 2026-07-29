// Recipe.h : layer 2.5 — the Tier-1 orchestration artifact.
//
// A recipe is a NESTED list of steps (children arrays, not bobcat's flat list
// with begin/end markers — malformed nesting is impossible by construction).
// Leaf steps name a registered command; five structural types are interpreted
// by the engine itself: loop / if / group / sub_recipe / stop_recipe.
//
// Persistence follows the project conventions: one consolidated file
// (config/recipes.local.json -> recipes.json fallback), or the product
// manifest's "recipes" role when a product is loaded. Tolerant load: a
// missing or malformed file yields an empty list, never throws.
#pragma once

#include "core/MotionMap.h"
#include "core/SavedPositions.h"

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

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
    std::vector<RecipeStep> steps;
};

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
                                        const std::vector<Recipe>& library,
                                        bool requireConnected);

// Replace every sub_recipe step's children with the referenced recipe's steps
// (recursively, cycle-checked against the inline chain). False + err on an
// unknown uid, a cycle, or depth overflow.
bool InlineSubRecipes(Recipe& r, const std::vector<Recipe>& library,
                      std::string* err);

// Count enabled leaf (command) steps for progress display. Fixed-count loops
// multiply; while-form loops count their body once (iterations unknown).
std::size_t CountLeafSteps(const std::vector<RecipeStep>& steps);

} // namespace firedog
