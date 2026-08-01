#include "session/ProductSession.h"

#include "core/Log.h"

namespace firedog {

void ProductSession::LoadPositions()
{
    positions_ = LoadSavedPositions();   // config fallback (no product yet)
}

void ProductSession::LoadMaps()
{
    maps_ = LoadMotionMaps();
}

void ProductSession::SavePositions()
{
    // Product loaded -> autosave into the product's positions file (with its
    // uid); otherwise the machine-global config fallback. Warns if unwritable.
    if (const ManifestFile* e = product_.Entry("positions"))
        SaveSavedPositionsFile(product_.FilePath("positions"), positions_, e->uid);
    else
        SaveSavedPositions(positions_);
}

void ProductSession::SaveMaps()
{
    // Machine-global for now (config/maps.local.json) — maps reference
    // position uids, which stay stable across products.
    if (SaveMotionMaps(maps_))
        FD_INFO("map", "maps saved to config/maps.local.json");
    else
        FD_WARN("map", "could not save motion maps");
}

void ProductSession::LoadRecipesFromDisk()
{
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("recipes");
        if (e) {
            recipes_ = LoadRecipesFile(product_.FilePath("recipes"), &recipesUid_);
            if (!recipes_.empty()) {
                if (!recipesUid_.empty() && recipesUid_ != e->uid)
                    FD_WARN("product", "recipes uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), recipesUid_.c_str());
                recipesBaseline_ = RecipesDump();
                return;
            }
            // New products carry an empty seed file (no editor yet in Phase A) —
            // fall through to the config recipes so the runner has something.
            FD_INFO("recipe", "product '%s' has no recipes yet - using config recipes",
                    product_.name.c_str());
        } else {
            FD_WARN("product", "'%s' has no recipes entry; using config recipes",
                    product_.name.c_str());
        }
    }
    recipesUid_.clear();
    recipes_ = LoadRecipes();   // config/recipes.local.json -> recipes.json
    recipesBaseline_ = RecipesDump();
}

// Write the in-memory recipes to the product's recipes file when a product is
// loaded (manifest uid stamped), else config/recipes.local.json — the same
// precedence LoadRecipesFromDisk reads with.
void ProductSession::SaveRecipes()
{
    std::string path = "config/recipes.local.json";
    std::string uid  = recipesUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("recipes")) {
            path = product_.FilePath("recipes");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    recipesUid_ = uid;
    if (SaveRecipesFile(path, recipes_, uid)) {
        recipesBaseline_ = RecipesDump();
        FD_INFO("recipe", "saved %zu recipe(s) to %s", recipes_.size(), path.c_str());
    } else {
        FD_WARN("recipe", "could not save recipes to %s", path.c_str());
    }
}

void ProductSession::SaveRecipesTo(const std::string& path, const std::string& uid)
{
    SaveRecipesFile(path, recipes_, uid);
    recipesUid_      = uid;
    recipesBaseline_ = RecipesDump();
}

// Canonical serialization of the in-memory recipes, compared against the
// baseline captured at load/save to answer "are there unsaved edits?".
std::string ProductSession::RecipesDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const Recipe& r : recipes_) arr.push_back(RecipeToJson(r));
    return arr.dump();
}

// ---- Programs (Run page) — same persistence conventions as recipes ---------

void ProductSession::LoadProgramsFromDisk()
{
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("programs");
        if (e) {
            programs_ = LoadProgramsFile(product_.FilePath("programs"), &programsUid_);
            if (!programs_.empty()) {
                if (!programsUid_.empty() && programsUid_ != e->uid)
                    FD_WARN("product", "programs uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), programsUid_.c_str());
                programsBaseline_ = ProgramsDump();
                return;
            }
            FD_INFO("program", "product '%s' has no programs yet - using config programs",
                    product_.name.c_str());
        }
    }
    programsUid_.clear();
    programs_ = LoadPrograms();
    programsBaseline_ = ProgramsDump();
}

void ProductSession::SavePrograms()
{
    std::string path = "config/programs.local.json";
    std::string uid  = programsUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("programs")) {
            path = product_.FilePath("programs");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    programsUid_ = uid;
    if (SaveProgramsFile(path, programs_, uid)) {
        programsBaseline_ = ProgramsDump();
        FD_INFO("program", "saved %zu program(s) to %s", programs_.size(), path.c_str());
    } else {
        FD_WARN("program", "could not save programs to %s", path.c_str());
    }
}

void ProductSession::SaveProgramsTo(const std::string& path, const std::string& uid)
{
    SaveProgramsFile(path, programs_, uid);
    programsUid_      = uid;
    programsBaseline_ = ProgramsDump();
}

std::string ProductSession::ProgramsDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const Program& p : programs_) arr.push_back(ProgramToJson(p));
    return arr.dump();
}

void ProductSession::MergeTaughtPositions(
    const std::vector<std::pair<std::string, SavedPos>>& taught)
{
    if (taught.empty()) return;
    for (const auto& [dev, sp] : taught) {
        std::vector<SavedPos>& list = positions_[dev];
        bool found = false;
        for (SavedPos& e : list) {
            if (std::string(e.name) == sp.name) {
                e.vals = sp.vals;
                found  = true;
                break;
            }
        }
        if (!found) list.push_back(sp);
    }
    SavePositions();
}

} // namespace firedog
