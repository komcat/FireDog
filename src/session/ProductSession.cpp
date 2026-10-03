#include "session/ProductSession.h"

#include "command/ExecutionContext.h"   // MachineScriptsDir
#include "core/DutSession.h"
#include "core/Log.h"
#include "core/Paths.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace firedog {

// ---- Startup + product lifecycle -------------------------------------------

bool ProductSession::LoadInitial(const std::string& lastProduct)
{
    LoadPositions();   // config fallback (no product yet)
    LoadMaps();        // node-edge maps (Mapped Path tab)

    bool ok = true;
    if (!lastProduct.empty() && !LoadProduct(lastProduct)) {
        FD_WARN("product", "could not reload last product '%s'", lastProduct.c_str());
        ok = false;
    }
    if (!product_.Loaded()) LoadRecipesFromDisk();    // else LoadProduct did it
    if (!product_.Loaded()) LoadProgramsFromDisk();
    if (!product_.Loaded()) LoadAlignsFromDisk();
    if (!product_.Loaded()) LoadVisionFromDisk();
    if (!product_.Loaded()) LoadGripsFromDisk();
    if (!product_.Loaded()) LoadTraysFromDisk();
    LoadScriptsList();   // .lua programs (product dir, else the machine's + shipped)
    if (product_.Loaded())   // DUT records live under the product's data/
        DutSession::Instance().SetProduct(product_.name, product_.uid,
                                          product_.DataDir());
    return ok;
}

bool ProductSession::LoadProduct(const std::string& name)
{
    ProductManifest m = LoadProductManifest(name);
    if (!m.Loaded()) return false;

    const ManifestFile* e = m.Entry("positions");
    if (e) {
        std::string fileUid;
        positions_ = LoadSavedPositionsFile(m.FilePath("positions"), &fileUid);
        if (fileUid != e->uid)
            FD_WARN("product", "positions uid mismatch: manifest %s vs file '%s'",
                    e->uid.c_str(), fileUid.c_str());
    } else {
        FD_WARN("product", "'%s' has no positions entry; keeping current positions", name.c_str());
    }
    positionsBaseline_ = positions_;   // the journal starts from what the file holds
    product_ = std::move(m);
    // Products created before a role existed have no manifest entry for it, so
    // their saves would fall back to config/ forever. Backfill on load — this
    // is load time, never draw time.
    EnsureManifestRole(product_, "alignments");
    EnsureManifestRole(product_, "vision");
    EnsureManifestRole(product_, "grips");
    EnsureManifestRole(product_, "trays");
    LoadRecipesFromDisk();   // product recipes replace the config-fallback set
    LoadProgramsFromDisk();  // product programs likewise
    LoadAlignsFromDisk();    // product alignment procedures likewise
    LoadVisionFromDisk();    // product vision detection models likewise
    LoadGripsFromDisk();     // product gripped-object profiles likewise
    LoadTraysFromDisk();     // product waffle trays likewise
    LoadScriptsList();       // product scripts likewise (config fallback)
    DutSession::Instance().SetProduct(product_.name, product_.uid,
                                      product_.DataDir());   // auto-closes any DUT
    FD_INFO("product", "loaded '%s'", product_.name.c_str());
    return true;
}

void ProductSession::NewProduct(const char* name)
{
    ProductManifest m = CreateProduct(name);
    if (!m.Loaded()) return;
    // Where the current models' templates live, captured BEFORE the manifest
    // swap moves VisionDir() to the new product.
    const std::string oldVisionDir = VisionDir();
    const std::string oldHistory   = PositionHistoryPath();

    product_ = std::move(m);
    DutSession::Instance().SetProduct(product_.name, product_.uid,
                                      product_.DataDir());
    SavePositions();   // carry the current in-memory positions into the product
    // The positions carried over bring their journal with them: the new
    // product's rows ARE the old ones, and "who changed this" does not start
    // over because the folder did. Best-effort copy (the old side may have
    // none yet).
    {
        std::error_code ec;
        const std::string newHistory = PositionHistoryPath();
        if (newHistory != oldHistory && std::filesystem::exists(oldHistory, ec))
            std::filesystem::copy_file(oldHistory, newHistory,
                                       std::filesystem::copy_options::overwrite_existing, ec);
    }
    // Procedure DEFINITIONS carry over like recipes (their positions did too),
    // but a taught frame belongs to the part that was on the machine — keeping
    // it would silently give the new product the old part's origin.
    for (AlignProc& a : aligns_) a.result = AlignResult{};
    if (!aligns_.empty()) SaveAligns();

    // Detection models carry over too, but a model is useless without its
    // taught patch, so the .pgm files are COPIED rather than referenced across
    // products — the new product must stay self-contained when it is moved.
    if (!vision_.empty()) {
        CopyVisionTemplates(oldVisionDir, VisionDir());
        SaveVision();
    }

    // Grip profiles carry over with them: HOW to measure an object is a
    // parameter of the object, and the models they point at have just been
    // copied. What does NOT carry is the reference offset - it describes one
    // object in one gripper, and a new product is a new object until somebody
    // says otherwise. The measurement evidence goes with it; a stale "last
    // measured" would be read as this product's.
    for (GripProfile& g : grips_) {
        g.haveRef = false;
        g.refDx = g.refDy = 0.0;
        g.taughtAt.clear();
        g.lastDx = g.lastDy = 0.0;
        g.measuredAt.clear();
    }
    if (!grips_.empty()) SaveGrips();

    // Trays carry over intact. A tray's corners say where a fixture sits on
    // THIS machine, and its masks which cavities are usable — the new product
    // is loaded from the same fixture until somebody re-teaches it. Nothing
    // in the record describes one part the way a taught frame or a reference
    // offset does. (The consumed-cell cursor is runtime state on Workflow and
    // is cleared by App on a new product, like on a load.)
    if (!trays_.empty()) SaveTrays();
}

void ProductSession::SaveProduct()
{
    if (!product_.Loaded()) return;
    // "Save runs through the manifest": each entry is dispatched by role.
    for (const ManifestFile& e : product_.files) {
        const std::string path = product_.Dir() + "/" + e.file;
        if (e.role == "positions") {
            SaveSavedPositionsFile(path, positions_, e.uid);
        } else if (e.role == "recipes") {
            SaveRecipesTo(path, e.uid);
        } else if (e.role == "programs") {
            SaveProgramsTo(path, e.uid);
        } else if (e.role == "alignments") {
            SaveAlignsTo(path, e.uid);
        } else if (e.role == "vision") {
            SaveVisionTo(path, e.uid);
        } else if (e.role == "grips") {
            SaveGripsTo(path, e.uid);
        } else if (e.role == "trays") {
            SaveTraysTo(path, e.uid);
        } else {
            FD_WARN("product", "unknown manifest role '%s' (skipped)", e.role.c_str());
        }
    }
    SaveProductManifest(product_);
    FD_INFO("product", "saved '%s' (%zu manifest file(s))",
            product_.name.c_str(), product_.files.size());
}

// ---- Scripts (.lua programs; authored externally, scanned here) -------------

std::string ProductSession::ScriptsDir() const
{
    return product_.Loaded() ? product_.Dir() + "/scripts" : MachineScriptsDir();
}

void ProductSession::LoadScriptsList()
{
    scriptsDirUsed_ = ScriptsDir();
    scripts_ = ListScripts(scriptsDirUsed_);
    if (scripts_.empty() && product_.Loaded()) {
        // Product without a scripts folder — fall back like recipes do, and
        // show the directory ACTUALLY used, not the one we wished for.
        scriptsDirUsed_ = MachineScriptsDir();
        scripts_ = ListScripts(scriptsDirUsed_);
    }
    if (scriptsDirUsed_ != MachineScriptsDir()) return;

    // The machine folder holds only what was created or saved on this PC, so
    // the shipped scripts are listed beside it - a machine copy winning by
    // name, exactly as ResolveScript picks it.
    for (ScriptInfo& si : ListScripts("config/scripts")) {
        bool have = false;
        for (const ScriptInfo& m : scripts_) have = have || m.name == si.name;
        if (!have) scripts_.push_back(std::move(si));
    }
    std::sort(scripts_.begin(), scripts_.end(),
              [](const ScriptInfo& a, const ScriptInfo& b) { return a.name < b.name; });
}

void ProductSession::LoadPositions()
{
    positions_         = LoadSavedPositions();   // config fallback (no product yet)
    positionsBaseline_ = positions_;
}

void ProductSession::LoadMaps()
{
    maps_ = LoadMotionMaps();
}

void ProductSession::SavePositions(const std::string& source)
{
    // Product loaded -> autosave into the product's positions file (with its
    // uid); otherwise the machine-global config fallback. Warns if unwritable.
    if (const ManifestFile* e = product_.Entry("positions"))
        SaveSavedPositionsFile(product_.FilePath("positions"), positions_, e->uid);
    else
        SaveSavedPositions(positions_);
    // The journal: whatever changed since the last load / save, attributed.
    const std::vector<PosChange> changes =
        DiffPositions(positionsBaseline_, positions_, positionAuthor_, source);
    if (!changes.empty()) {
        AppendPositionHistory(PositionHistoryPath(), changes);
        ++positionHistoryVer_;
    }
    positionsBaseline_ = positions_;
}

std::string ProductSession::PositionHistoryPath() const
{
    if (product_.Entry("positions"))
        return PositionHistoryPathFor(product_.FilePath("positions"));
    return PositionHistoryPathFor(MachinePath("config/positions.local.json"));
}

bool ProductSession::RestorePosition(const PosChange& entry)
{
    std::vector<SavedPos>& list = positions_[entry.device];
    SavedPos* row = nullptr;
    for (SavedPos& e : list) if (e.uid == entry.uid) { row = &e; break; }
    const std::string source = "restore of " + entry.action + " @ " + entry.at;

    if (entry.action == "rename") {
        if (!row || entry.nameBefore.empty()) return false;
        std::snprintf(row->name, sizeof(row->name), "%s", entry.nameBefore.c_str());
    } else if (entry.action == "update") {
        if (!row || entry.before.empty()) return false;
        row->vals = entry.before;
    } else if (entry.action == "delete") {
        if (entry.before.empty()) return false;
        if (!row) {   // re-create under the same uid so old references resolve again
            SavedPos sp;
            std::snprintf(sp.name, sizeof(sp.name), "%s", entry.name.c_str());
            sp.uid = entry.uid;
            list.push_back(std::move(sp));
            row = &list.back();
        }
        row->vals   = entry.before;
        row->hidden = false;
    } else {
        return false;   // an add has nothing to put back (delete the row instead)
    }
    FD_INFO("saved", "restored '%s' (%s, %s) from the %s of %s", row->name,
            entry.device.c_str(), entry.uid.c_str(), entry.action.c_str(), entry.at.c_str());
    SavePositions(source);
    return true;
}

void ProductSession::SaveMaps()
{
    // Machine-global for now (<machine root>/config/maps.local.json) — maps reference
    // position uids, which stay stable across products.
    if (SaveMotionMaps(maps_))
        FD_INFO("map", "maps saved to %s", MachinePath("config/maps.local.json").c_str());
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
                RebaselineRecipes();
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
    RebaselineRecipes();
}

// Write the in-memory recipes to the product's recipes file when a product is
// loaded (manifest uid stamped), else config/recipes.local.json — the same
// precedence LoadRecipesFromDisk reads with.
void ProductSession::SaveRecipes()
{
    std::string path = MachinePath("config/recipes.local.json");
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
        RebaselineRecipes();
        FD_INFO("recipe", "saved %zu recipe(s) to %s", recipes_.size(), path.c_str());
    } else {
        FD_WARN("recipe", "could not save recipes to %s", path.c_str());
    }
}

void ProductSession::SaveRecipesTo(const std::string& path, const std::string& uid)
{
    SaveRecipesFile(path, recipes_, uid);
    recipesUid_      = uid;
    RebaselineRecipes();
}

// Both baselines at once, so "the file is dirty" and "this recipe is dirty"
// can never be measured against two different save moments.
void ProductSession::RebaselineRecipes()
{
    recipesBaseline_ = RecipesDump();
    recipeBaselineByUid_.clear();
    for (const Recipe& r : recipes_) recipeBaselineByUid_[r.uid] = RecipeToJson(r).dump();
}

bool ProductSession::RecipeDirty(const Recipe& r) const
{
    const auto it = recipeBaselineByUid_.find(r.uid);
    return it == recipeBaselineByUid_.end() || RecipeToJson(r).dump() != it->second;
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
    std::string path = MachinePath("config/programs.local.json");
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

// ---- Alignment procedures — same persistence conventions as recipes --------

void ProductSession::LoadAlignsFromDisk()
{
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("alignments");
        if (e) {
            aligns_ = LoadAlignProcsFile(product_.FilePath("alignments"), &alignsUid_);
            if (!aligns_.empty()) {
                if (!alignsUid_.empty() && alignsUid_ != e->uid)
                    FD_WARN("product", "alignments uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), alignsUid_.c_str());
                alignsBaseline_ = AlignsDump();
                return;
            }
            // A product with an empty seed file adopts the manifest uid anyway,
            // so the first Save lands in the product and not in config/.
            alignsUid_      = e->uid;
            aligns_.clear();
            alignsBaseline_ = AlignsDump();
            return;
        }
        FD_WARN("product", "'%s' has no alignments entry; using config alignments",
                product_.name.c_str());
    }
    alignsUid_.clear();
    aligns_ = LoadAlignProcs();   // config/alignments.local.json -> alignments.json
    alignsBaseline_ = AlignsDump();
}

void ProductSession::SaveAligns()
{
    std::string path = MachinePath("config/alignments.local.json");
    std::string uid  = alignsUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("alignments")) {
            path = product_.FilePath("alignments");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    alignsUid_ = uid;
    if (SaveAlignProcsFile(path, aligns_, uid)) {
        alignsBaseline_ = AlignsDump();
        FD_INFO("align", "saved %zu alignment procedure(s) to %s", aligns_.size(), path.c_str());
    } else {
        FD_WARN("align", "could not save alignments to %s", path.c_str());
    }
}

void ProductSession::SaveAlignsTo(const std::string& path, const std::string& uid)
{
    SaveAlignProcsFile(path, aligns_, uid);
    alignsUid_      = uid;
    alignsBaseline_ = AlignsDump();
}

std::string ProductSession::AlignsDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const AlignProc& p : aligns_) arr.push_back(AlignProcToJson(p));
    return arr.dump();
}

bool ProductSession::ApplyAlignResult(const std::string& uid, const AlignResult& result)
{
    AlignProc* p = FindAlignProc(aligns_, uid);
    if (!p) {
        FD_WARN("align", "taught frame for unknown procedure '%s' (dropped)", uid.c_str());
        return false;
    }
    p->result = result;
    // The history row is built HERE, from the procedure's own band and the
    // frame's own two points, so nothing has to carry a verdict across the
    // worker/UI boundary — and a band edited between the run and this call is
    // the one that judges it, which is the only reading that cannot go stale.
    const AlignRun row = MakeAlignRun(p->guard, result);
    p->runs.push_back(row);
    if (p->runs.size() > kAlignRunHistoryMax) p->runs.erase(p->runs.begin());
    SaveAligns();   // a taught frame is worthless if a crash loses it
    FD_INFO("align", "'%s': origin (%.4f, %.4f) mm, theta %.4f deg, span %.4f mm",
            p->name.c_str(), result.originX, result.originY, result.thetaDeg, result.spanMm);
    if (row.guarded && !row.ok) {
        FD_WARN("align", "'%s': OUT OF BAND - span %.4f (ref %.4f +/- %.4f), "
                         "theta %.4f (ref %.4f +/- %.4f)",
                p->name.c_str(), row.spanMm, p->guard.spanMm, p->guard.spanTolMm,
                row.thetaDeg, p->guard.thetaDeg, p->guard.thetaTolDeg);
    }
    return true;
}

// ---- Gripped-object profiles - the same document conventions again --------

void ProductSession::LoadGripsFromDisk()
{
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("grips");
        if (e) {
            grips_ = LoadGripProfilesFile(product_.FilePath("grips"), &gripsUid_);
            if (!grips_.empty()) {
                if (!gripsUid_.empty() && gripsUid_ != e->uid)
                    FD_WARN("product", "grips uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), gripsUid_.c_str());
                gripsBaseline_ = GripsDump();
                return;
            }
            // Same reason alignments do this: a product with an empty seed file
            // adopts the manifest uid, so the first Save lands in the product
            // and not in config/.
            gripsUid_      = e->uid;
            grips_.clear();
            gripsBaseline_ = GripsDump();
            return;
        }
        FD_WARN("product", "'%s' has no grips entry; using config grip profiles",
                product_.name.c_str());
    }
    gripsUid_.clear();
    grips_ = LoadGripProfiles();   // config/grips.local.json -> grips.json
    gripsBaseline_ = GripsDump();
}

void ProductSession::SaveGrips()
{
    std::string path = MachinePath("config/grips.local.json");
    std::string uid  = gripsUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("grips")) {
            path = product_.FilePath("grips");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    gripsUid_ = uid;
    if (SaveGripProfilesFile(path, grips_, uid)) {
        gripsBaseline_ = GripsDump();
        FD_INFO("grip", "saved %zu grip profile(s) to %s", grips_.size(), path.c_str());
    } else {
        FD_WARN("grip", "could not save grip profiles to %s", path.c_str());
    }
}

void ProductSession::SaveGripsTo(const std::string& path, const std::string& uid)
{
    SaveGripProfilesFile(path, grips_, uid);
    gripsUid_      = uid;
    gripsBaseline_ = GripsDump();
}

std::string ProductSession::GripsDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const GripProfile& g : grips_) arr.push_back(GripProfileToJson(g));
    return arr.dump();
}

// What the last run measured, by the route a taught frame takes. It is
// EVIDENCE and never a reference: adopting an offset is a decision an operator
// makes on the page, looking at the number, and a run that quietly promoted its
// own measurement to the standard would make every grip a good grip.
bool ProductSession::ApplyGripResult(const std::string& uid, double dx, double dy)
{
    GripProfile* g = FindGripProfile(grips_, uid);
    if (!g) {
        FD_WARN("grip", "measurement for unknown profile '%s' (dropped)", uid.c_str());
        return false;
    }
    g->lastDx     = dx;
    g->lastDy     = dy;
    g->measuredAt = NowStamp();
    SaveGrips();
    return true;
}

// ---- Waffle trays - the same document conventions once more ---------------

void ProductSession::LoadTraysFromDisk()
{
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("trays");
        if (e) {
            trays_ = LoadWaffleTraysFile(product_.FilePath("trays"), &traysUid_);
            if (!trays_.empty()) {
                if (!traysUid_.empty() && traysUid_ != e->uid)
                    FD_WARN("product", "trays uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), traysUid_.c_str());
                traysBaseline_ = TraysDump();
                return;
            }
            // An empty seed file adopts the manifest uid, so the first Save
            // lands in the product and not in config/ (same as grips).
            traysUid_      = e->uid;
            trays_.clear();
            traysBaseline_ = TraysDump();
            return;
        }
        FD_WARN("product", "'%s' has no trays entry; using config waffle trays",
                product_.name.c_str());
    }
    traysUid_.clear();
    trays_ = LoadWaffleTrays();   // config/trays.local.json -> trays.json
    traysBaseline_ = TraysDump();
}

void ProductSession::SaveTrays()
{
    std::string path = MachinePath("config/trays.local.json");
    std::string uid  = traysUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("trays")) {
            path = product_.FilePath("trays");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    traysUid_ = uid;
    if (SaveWaffleTraysFile(path, trays_, uid)) {
        traysBaseline_ = TraysDump();
        FD_INFO("tray", "saved %zu waffle tray(s) to %s", trays_.size(), path.c_str());
    } else {
        FD_WARN("tray", "could not save waffle trays to %s", path.c_str());
    }
}

void ProductSession::SaveTraysTo(const std::string& path, const std::string& uid)
{
    SaveWaffleTraysFile(path, trays_, uid);
    traysUid_      = uid;
    traysBaseline_ = TraysDump();
}

std::string ProductSession::TraysDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const WaffleTray& t : trays_) arr.push_back(WaffleTrayToJson(t));
    return arr.dump();
}

bool ProductSession::SetTrayCellsUsed(const std::string& uid,
                                      const std::vector<TrayCell>& cells, bool used)
{
    WaffleTray* t = FindWaffleTray(trays_, uid);
    if (!t) {
        FD_WARN("tray", "used-cell change for unknown tray '%s' (dropped)", uid.c_str());
        return false;
    }
    if (cells.empty()) {
        t->usedCells.clear();
        if (used)
            for (int r = 1; r <= t->rows; ++r)
                for (int c = 1; c <= t->cols; ++c) t->usedCells.insert(TrayCell{r, c});
    } else {
        for (const TrayCell& cell : cells) {
            if (!TrayCellInRange(*t, cell.first, cell.second)) continue;
            if (used) t->usedCells.insert(cell);
            else      t->usedCells.erase(cell);
        }
    }
    SaveTrays();
    return true;
}

// ---- Vision detection models — same conventions as alignment procedures ----

std::string ProductSession::VisionDir() const
{
    return product_.Loaded() ? product_.Dir() + "/vision" : MachinePath("config/vision");
}

void ProductSession::LoadVisionFromDisk()
{
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("vision");
        if (e) {
            vision_ = LoadVisionModelsFile(product_.FilePath("vision"), &visionUid_);
            if (!vision_.empty()) {
                if (!visionUid_.empty() && visionUid_ != e->uid)
                    FD_WARN("product", "vision uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), visionUid_.c_str());
                visionBaseline_ = VisionDump();
                return;
            }
            // A product with an empty seed file adopts the manifest uid anyway,
            // so the first Save lands in the product and not in config/.
            visionUid_      = e->uid;
            vision_.clear();
            visionBaseline_ = VisionDump();
            return;
        }
        FD_WARN("product", "'%s' has no vision entry; using config vision models",
                product_.name.c_str());
    }
    visionUid_.clear();
    vision_ = LoadVisionModels();   // config/vision.local.json -> vision.json
    visionBaseline_ = VisionDump();
}

void ProductSession::SaveVision()
{
    std::string path = MachinePath("config/vision.local.json");
    std::string uid  = visionUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("vision")) {
            path = product_.FilePath("vision");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    visionUid_ = uid;
    if (SaveVisionModelsFile(path, vision_, uid)) {
        visionBaseline_ = VisionDump();
        FD_INFO("vision", "saved %zu detection model(s) to %s", vision_.size(), path.c_str());
    } else {
        FD_WARN("vision", "could not save detection models to %s", path.c_str());
    }
}

void ProductSession::SaveVisionTo(const std::string& path, const std::string& uid)
{
    SaveVisionModelsFile(path, vision_, uid);
    visionUid_      = uid;
    visionBaseline_ = VisionDump();
}

std::string ProductSession::VisionDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const VisionModel& m : vision_) arr.push_back(VisionModelToJson(m));
    return arr.dump();
}

bool ProductSession::DeleteVisionModel(const std::string& uid)
{
    for (auto it = vision_.begin(); it != vision_.end(); ++it) {
        if (it->uid != uid) continue;
        if (!it->templateFile.empty()) {
            std::error_code ec;
            std::filesystem::remove(VisionDir() + "/" + it->templateFile, ec);
        }
        FD_INFO("vision", "deleted detection model '%s'", it->name.c_str());
        vision_.erase(it);
        return true;
    }
    return false;
}

void ProductSession::CopyVisionTemplates(const std::string& fromDir,
                                         const std::string& toDir) const
{
    if (fromDir == toDir) return;
    std::error_code ec;
    std::filesystem::create_directories(toDir, ec);
    for (const VisionModel& m : vision_) {
        if (m.templateFile.empty()) continue;
        std::filesystem::copy_file(fromDir + "/" + m.templateFile,
                                   toDir + "/" + m.templateFile,
                                   std::filesystem::copy_options::overwrite_existing, ec);
        if (ec)
            FD_WARN("vision", "could not copy template '%s' into %s (%s)",
                    m.templateFile.c_str(), toDir.c_str(), ec.message().c_str());
    }
}

// ---- Engine hand-off -------------------------------------------------------

RecipeEngine::Env ProductSession::MakeRecipeEnv() const
{
    RecipeEngine::Env env;
    env.positions = positions_;
    env.maps      = maps_;
    env.library   = recipes_;
    env.aligns    = aligns_;
    env.visionModels = vision_;
    env.grips        = grips_;
    env.trays        = trays_;
    env.visionDir    = VisionDir();
    // The dir actually SCANNED, not the ideal one: a product with no scripts
    // folder falls back to the machine's, and a run must resolve against the
    // same place the Scripts page lists.
    env.scriptsDir   = scriptsDirUsed_;
    return env;
}

LuaEngine::Env ProductSession::MakeLuaEnv() const
{
    LuaEngine::Env env;
    env.positions = positions_;
    env.maps      = maps_;
    env.library   = recipes_;   // fd.run_recipe / sub_recipe resolve against this
    env.aligns    = aligns_;
    env.visionModels = vision_;
    env.grips        = grips_;
    env.trays        = trays_;
    env.visionDir    = VisionDir();
    // The dir actually SCANNED, not the ideal one: a product with no scripts
    // folder falls back to the machine's, and a run must resolve against the
    // same place the Scripts page lists.
    env.scriptsDir   = scriptsDirUsed_;
    return env;
}

bool ProductSession::SeedHomePosition(const std::string& device,
                                      const std::vector<std::string>& axes)
{
    if (axes.empty()) return false;   // device not up yet — nothing to shape it from
    const auto it = positions_.find(device);
    if (it != positions_.end() && !it->second.empty()) return false;

    SavedPos home;
    std::snprintf(home.name, sizeof(home.name), "Home");
    home.uid = GenerateUid();
    for (const std::string& a : axes) home.vals[a] = 0.0;
    positions_[device].push_back(std::move(home));
    SavePositions();
    FD_INFO("position", "%s: seeded a Home position (no saved positions yet)",
            device.c_str());
    return true;
}

void ProductSession::MergeTaughtPositions(
    const std::vector<std::pair<std::string, SavedPos>>& taught, const std::string& source)
{
    if (taught.empty()) return;
    for (const auto& [dev, sp] : taught) {
        std::vector<SavedPos>& list = positions_[dev];
        bool found = false;
        // Uid first: a teach INTO a row (the movers' `teach` param) must land on
        // that row even if it was renamed while the run held its copy. The name
        // pass is save_position's contract - it may name a row it created, whose
        // fresh uid no session row carries yet.
        if (!sp.uid.empty()) {
            for (SavedPos& e : list) {
                if (e.uid == sp.uid) {
                    e.vals = sp.vals;
                    found  = true;
                    break;
                }
            }
        }
        if (!found) {
            for (SavedPos& e : list) {
                if (std::string(e.name) == sp.name) {
                    e.vals = sp.vals;
                    found  = true;
                    break;
                }
            }
        }
        if (!found) list.push_back(sp);
    }
    SavePositions(source);
}

} // namespace firedog
