// ProductSession.h : the application document — everything the operator's
// *product* consists of (saved positions, motion maps, recipes, programs,
// scripts) plus the manifest they belong to, and the ONE place that knows the
// product-file / config-fallback persistence convention.
//
// UI-THREAD ONLY. No locks, no ImGui, no Workflow/HardwareManager. The engines
// never read this live: they receive a COPY through Env at Start — copy-at-
// Start IS the concurrency contract — which is why mutable collection
// accessors are safe here and dirty tracking can stay snapshot-compare instead
// of observers.
//
// UI selection state (recipeSel_, progSel_, deckSlot_ …) deliberately does NOT
// live here; App resets it in thin wrappers around the load calls, so the
// session never needs a callback back into the UI.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "command/LuaEngine.h"   // ScriptInfo / ListScripts (no Lua headers leak)
#include "command/Program.h"
#include "command/Recipe.h"
#include "command/RecipeEngine.h"
#include "core/MotionMap.h"
#include "core/Product.h"
#include "core/SavedPositions.h"

namespace firedog {

class ProductSession {
public:
    // ---- collections (the UI edits these in place) ----
    SavedPosMap&        Positions()       { return positions_; }
    const SavedPosMap&  Positions() const { return positions_; }
    MotionMapSet&       Maps()            { return maps_; }
    const MotionMapSet& Maps()      const { return maps_; }

    std::vector<Recipe>&       Recipes()       { return recipes_; }
    const std::vector<Recipe>& Recipes() const { return recipes_; }

    std::vector<Program>&       Programs()       { return programs_; }
    const std::vector<Program>& Programs() const { return programs_; }

    // Scripts are scanned from disk, never edited here (authoring is external).
    const std::vector<ScriptInfo>& Scripts()        const { return scripts_; }
    const std::string&             ScriptsDirUsed() const { return scriptsDirUsed_; }

    // ---- product manifest ----
    const ProductManifest& Product() const { return product_; }
    bool ProductLoaded() const { return product_.Loaded(); }

    // Startup: config-fallback collections first, then the last product if one
    // was remembered. Returns false only when `lastProduct` was named and
    // could not be reloaded (already logged) — the session is usable either way.
    bool LoadInitial(const std::string& lastProduct);

    // File menu. Load/New re-point every collection at the product; Save runs
    // through the manifest entries, dispatching by role. All three also wire
    // DutSession to the product (which auto-closes any open DUT).
    bool LoadProduct(const std::string& name);
    void NewProduct(const char* name);
    void SaveProduct();

    // ---- load / save ----
    void LoadPositions();   // config fallback (startup, before any product)
    void LoadMaps();        // node-edge maps (Mapped Path tab)
    void SavePositions();   // product positions file when loaded, else config
    void SaveMaps();        // machine-global: maps reference stable position uids

    // Recipes: the product's recipes file when it has one with content, else
    // config/recipes.local.json -> recipes.json. Load/Save keep the dirty
    // baseline in step; RecipesDirty is a snapshot compare (no observers, so
    // the mutable Recipes() ref above stays safe).
    void LoadRecipesFromDisk();
    void SaveRecipes();
    // Product "Save through the manifest" pass: write to the manifest's own
    // path/uid, then adopt that uid and re-baseline.
    void SaveRecipesTo(const std::string& path, const std::string& uid);
    std::string RecipesDump() const;    // canonical serialization
    bool RecipesDirty() const { return RecipesDump() != recipesBaseline_; }

    // Programs (Run page) — same persistence conventions as recipes.
    void LoadProgramsFromDisk();
    void SavePrograms();
    void SaveProgramsTo(const std::string& path, const std::string& uid);
    std::string ProgramsDump() const;
    bool ProgramsDirty() const { return ProgramsDump() != programsBaseline_; }

    // Scripts: the product's scripts/ dir when it has one with content, else
    // config/scripts. ScriptsDirUsed() reports the dir ACTUALLY scanned.
    std::string ScriptsDir() const;
    void LoadScriptsList();

    // ---- engine hand-off ----
    // The ONE place a run's Env is built. Copy-at-Start is the concurrency
    // contract: the engines get a snapshot of the document and never read the
    // session again, so a run is immune to edits made while it executes.
    // Every launch site goes through these — hand-copying the three fields is
    // how a caller ends up forgetting the library and breaking sub_recipe.
    RecipeEngine::Env MakeRecipeEnv() const;
    LuaEngine::Env    MakeLuaEnv() const;

    // Teach hand-backs from save_position steps (queued on Workflow by engine
    // worker threads, drained on the UI thread): merge by name — the app-side
    // uid wins for existing names so recipe references stay stable — then
    // persist once per batch. No-op for an empty batch.
    void MergeTaughtPositions(const std::vector<std::pair<std::string, SavedPos>>& taught);

private:
    ProductManifest product_;
    SavedPosMap     positions_;
    MotionMapSet    maps_;

    std::vector<Recipe> recipes_;
    std::string         recipesUid_;        // file uid (product integrity check)
    std::string         recipesBaseline_;   // serialized state at load/save

    std::vector<Program> programs_;
    std::string          programsUid_;
    std::string          programsBaseline_;

    std::vector<ScriptInfo> scripts_;
    std::string             scriptsDirUsed_;   // the dir the list came from
};

} // namespace firedog
