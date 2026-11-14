// PorDeploy.cpp : see PorDeploy.h.
#include "por/PorDeploy.h"

#include "por/PorCompare.h"   // PorPairSteps, PorFindPos*

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "core/Product.h"     // GenerateUid

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <system_error>

namespace firedog {

const char* PorPlanActionName(PorPlanAction a)
{
    switch (a) {
    case PorPlanAction::Add:     return "add";
    case PorPlanAction::Replace: return "replace";
    case PorPlanAction::Same:    return "unchanged";
    case PorPlanAction::Keep:    return "keep (target only)";
    case PorPlanAction::Remove:  return "remove (target only)";
    }
    return "?";
}

namespace {

namespace fs = std::filesystem;

template <class T>
int CountName(const std::vector<T>& v, const std::string& name)
{
    int n = 0;
    for (const T& x : v) n += x.name == name;
    return n;
}

template <class T>
const T* ByName(const std::vector<T>& v, const std::string& name)
{
    for (const T& x : v) if (x.name == name) return &x;
    return nullptr;
}

bool IsTaughtPoint(const nlohmann::json& params, const std::string& name)
{
    const auto it = params.find(name);
    if (it == params.end() || !it->is_object()) return false;
    for (const char* ax : {"X", "Y", "Z"})
        if (!it->contains(ax) || !(*it)[ax].is_number()) return false;
    return true;
}

std::string ProductDir(const PorSide& s) { return s.root + "/products/" + s.product; }

// Byte-for-byte; a file that cannot be read is "different".
bool SameFile(const std::string& a, const std::string& b)
{
    std::ifstream fa(a, std::ios::binary), fb(b, std::ios::binary);
    if (!fa || !fb) return false;
    const std::string sa((std::istreambuf_iterator<char>(fa)), std::istreambuf_iterator<char>());
    const std::string sb((std::istreambuf_iterator<char>(fb)), std::istreambuf_iterator<char>());
    return sa == sb;
}

struct Translator {
    const PorSide&         m;
    const PorSide&         t;
    const CommandRegistry& reg;
    PorDeployPlan&         plan;

    std::map<std::string, std::string> recipeUid;    // master recipe uid -> written uid
    std::set<std::string> warned;
    std::string           where;                     // "recipe 'Load'"
    int                   recipePointsKept = 0, recipePointsUntaught = 0, recipeModelsUntaught = 0;

    void Warn(const std::string& s)
    {
        if (warned.insert(s).second) plan.warnings.push_back(s);
    }

    bool PosUidTaken(const std::string& uid) const
    {
        for (const auto& [dev, list] : plan.positions)
            for (const SavedPos& sp : list)
                if (sp.uid == uid) return true;
        return false;
    }

    // A private name (algn_<procUid>_..., grip_<uid>_..., vdet_<procUid>_...) re-pointed at the
    // target's owner OF THE SAME NAME. The name IS the ownership record (CLAUDE.md), so a
    // private object is matched by its owner, never by its own name.
    bool TranslatePrivate(const std::string& name, std::string* out) const
    {
        const auto split = [&](std::size_t plen, std::string* owner, std::string* tail) {
            const std::string rest = name.substr(plen);
            const std::size_t us   = rest.find('_');
            *owner = rest.substr(0, us);
            *tail  = us == std::string::npos ? "" : rest.substr(us);
        };
        std::string owner, tail;
        if (name.rfind("vdet_step_", 0) == 0) return false;   // a step's own: not a reference
        if (name.rfind("algn_", 0) == 0 || name.rfind("vdet_", 0) == 0) {
            const std::size_t plen = 5;
            split(plen, &owner, &tail);
            const AlignProc* ma = FindAlignProc(m.aligns, owner);
            if (!ma || CountName(t.aligns, ma->name) != 1) return false;
            *out = name.substr(0, plen) + ByName(t.aligns, ma->name)->uid + tail;
            return true;
        }
        if (name.rfind("grip_", 0) == 0) {
            split(5, &owner, &tail);
            const GripProfile* mg = FindGripProfile(m.grips, owner);
            if (!mg || CountName(t.grips, mg->name) != 1) return false;
            *out = "grip_" + ByName(t.grips, mg->name)->uid + tail;
            return true;
        }
        return false;
    }

    static bool IsPrivate(const std::string& n)
    {
        return n.rfind("algn_", 0) == 0 || n.rfind("grip_", 0) == 0 || n.rfind("vdet_", 0) == 0;
    }

    std::string Position(const std::string& device, const std::string& v, const std::string& use)
    {
        const SavedPos* mp = PorFindPosByUid(m.positions, device, v);
        if (!mp) {
            Warn(where + ": position uid " + v + " on " + device + " is not on the master - left as it is");
            return v;
        }
        std::string name = mp->name;
        if (mp->hidden && IsPrivate(name) && !TranslatePrivate(name, &name)) {
            Warn(where + ": private position '" + std::string(mp->name) +
                 "' - its owner is not on this machine; left as it is");
            return v;
        }
        int n = 0;
        if (const SavedPos* tp = PorFindPosByName(plan.positions, device, name, &n)) {
            if (n > 1) Warn(where + ": position '" + name + "' on " + device +
                            " is named twice here - the first was used");
            for (PorPlaceholder& ph : plan.placeholders)   // a placeholder made earlier
                if (ph.uid == tp->uid && std::find(ph.uses.begin(), ph.uses.end(), use) == ph.uses.end())
                    ph.uses.push_back(use);
            return tp->uid;
        }
        // A placeholder: the master's name, hidden flag and axes (a teach writes the axes
        // the row HAS), a fresh uid, and untaught - so nothing ever drives to it.
        SavedPos ph;
        std::snprintf(ph.name, sizeof(ph.name), "%s", name.c_str());
        do ph.uid = GenerateUid(); while (PosUidTaken(ph.uid));
        ph.vals     = mp->vals;
        ph.hidden   = mp->hidden;
        ph.untaught = true;
        plan.positions[device].push_back(ph);
        plan.placeholders.push_back({device, name, ph.uid, {use}});
        return ph.uid;
    }

    // An object referenced by uid OR name (AlignUid, GripUid, TrayUid, VisionUid): the
    // target's object of the master's object's name, written the way the master wrote it.
    template <class T, class Find>
    std::string Named(const std::vector<T>& mv, const std::vector<T>& tv, Find find,
                      const std::string& v, const char* what)
    {
        const T* mo = find(mv, v);
        if (!mo) {
            Warn(where + ": " + what + " '" + v + "' is not on the master - left as it is");
            return v;
        }
        std::string name = mo->name;
        if (IsPrivate(name) && !TranslatePrivate(name, &name)) {
            Warn(where + ": private " + what + " '" + mo->name +
                 "' - its owner is not on this machine; left as it is");
            return v;
        }
        const int n = CountName(tv, name);
        if (n == 0) {
            Warn(where + ": " + what + " '" + name + "' is not on this machine - left unresolved");
            return v;
        }
        if (n > 1) Warn(where + ": " + what + " '" + name + "' is named twice here - the first was used");
        const T* to = ByName(tv, name);
        return v == mo->uid ? to->uid : to->name;
    }

    std::string Nominal(const nlohmann::json& merged, const std::string& v)
    {
        const AlignProc* ma = FindAlignProc(m.aligns, merged.value("alignment", std::string()));
        const AlignNominal* mn = ma ? FindAlignNominal(*ma, v) : nullptr;
        if (!mn) {
            Warn(where + ": nominal '" + v + "' is not on the master - left as it is");
            return v;
        }
        const AlignProc* ta = CountName(t.aligns, ma->name) == 1 ? ByName(t.aligns, ma->name) : nullptr;
        const AlignNominal* tn = ta ? FindAlignNominal(*ta, mn->name) : nullptr;
        if (!tn) {
            Warn(where + ": nominal '" + mn->name + "' of alignment '" + ma->name +
                 "' is not on this machine - left unresolved");
            return v;
        }
        return v == mn->uid ? tn->uid : tn->name;
    }

    std::string RecipeRef(const std::string& v, const char* what)
    {
        const auto it = recipeUid.find(v);
        if (it != recipeUid.end()) return it->second;
        Warn(where + ": " + what + " names recipe uid " + v + ", not on the master - left as it is");
        return v;
    }

    std::vector<RecipeStep> Steps(const std::vector<RecipeStep>& ms, const std::vector<RecipeStep>* ts,
                                  std::vector<std::size_t>& path, std::size_t base)
    {
        static const std::vector<RecipeStep> kNone;
        const std::vector<RecipeStep>& B = ts ? *ts : kNone;
        std::vector<const RecipeStep*> twin(ms.size(), nullptr);
        for (const auto& [i, j] : PorPairSteps(ms, B))
            if (i >= 0 && j >= 0) twin[static_cast<std::size_t>(i)] = &B[static_cast<std::size_t>(j)];
        std::vector<RecipeStep> out;
        out.reserve(ms.size());
        for (std::size_t i = 0; i < ms.size(); ++i) {
            path.push_back(base + i);
            out.push_back(Step(ms[i], twin[i], path));
            path.pop_back();
        }
        return out;
    }

    RecipeStep Step(const RecipeStep& ms, const RecipeStep* ts, std::vector<std::size_t>& path)
    {
        const std::string label = where + " > " + StepPathLabel(path) + " " + ms.type;
        RecipeStep out = ms;
        out.children.clear();
        out.elseChildren.clear();

        if (ms.type == "sub_recipe" && ms.params.contains("uid") && ms.params["uid"].is_string())
            out.params["uid"] = RecipeRef(ms.params["uid"].get<std::string>(), "sub_recipe");

        if (const CommandSpec* spec = reg.Find(ms.type)) {
            const nlohmann::json merged = EffectiveParams(*spec, ms.params);
            const std::string device = merged.value("device", std::string());
            for (const ParamSpec& p : spec->params) {
                if (!ms.params.contains(p.name)) continue;   // only what the step stores
                const nlohmann::json& v = ms.params[p.name];

                // Measured: the TARGET's paired step's value, else untaught.
                if (p.type == ParamType::PointXyz) {
                    if (ts && IsTaughtPoint(ts->params, p.name)) {
                        out.params[p.name] = ts->params[p.name];
                        ++plan.carriedPoints;
                        ++recipePointsKept;
                    } else {
                        out.params[p.name] = nlohmann::json::object();   // {} = untaught
                        plan.untaughtPoints.push_back(label + ": " + p.name);
                        ++recipePointsUntaught;
                    }
                    continue;
                }
                if (ms.type == "detect_object" && p.name == "model") {
                    const std::string tm = ts ? ts->params.value("model", std::string()) : std::string();
                    if (!tm.empty() && FindVisionModel(t.vision, tm)) {
                        out.params["model"] = tm;
                        ++plan.carriedModels;
                    } else {
                        out.params["model"] = "";
                        plan.untaughtModels.push_back(label);
                        ++recipeModelsUntaught;
                    }
                    continue;
                }

                if (!v.is_string() || v.get<std::string>().empty()) continue;
                const std::string s = v.get<std::string>();
                switch (p.type) {
                case ParamType::PositionUid: out.params[p.name] = Position(device, s, label); break;
                case ParamType::AlignUid:
                    out.params[p.name] = Named(m.aligns, t.aligns,
                        [](const auto& l, const std::string& k) { return FindAlignProc(l, k); }, s, "alignment");
                    break;
                case ParamType::NominalUid: out.params[p.name] = Nominal(merged, s); break;
                case ParamType::VisionUid:
                    out.params[p.name] = Named(m.vision, t.vision,
                        [](const auto& l, const std::string& k) { return FindVisionModel(l, k); }, s,
                        "detection model");
                    break;
                case ParamType::GripUid:
                    out.params[p.name] = Named(m.grips, t.grips,
                        [](const auto& l, const std::string& k) { return FindGripProfile(l, k); }, s,
                        "grip profile");
                    break;
                case ParamType::TrayUid:
                    out.params[p.name] = Named(m.trays, t.trays,
                        [](const auto& l, const std::string& k) { return FindWaffleTray(l, k); }, s, "tray");
                    break;
                default:
                    break;   // device / pin / light / map / script / picture names cross as they are
                }
            }
        }
        out.children     = Steps(ms.children, ts ? &ts->children : nullptr, path, 0);
        out.elseChildren = Steps(ms.elseChildren, ts ? &ts->elseChildren : nullptr, path, kElseBranchBase);
        return out;
    }

    Recipe TranslateRecipe(const Recipe& mr, const Recipe* tr)
    {
        where = "recipe '" + mr.name + "'";
        recipePointsKept = recipePointsUntaught = recipeModelsUntaught = 0;
        Recipe out = mr;
        out.uid = recipeUid.at(mr.uid);
        if (!mr.onFailUid.empty()) out.onFailUid = RecipeRef(mr.onFailUid, "on_fail");
        std::vector<std::size_t> path;
        out.steps = Steps(mr.steps, tr ? &tr->steps : nullptr, path, 0);
        return out;
    }

    void Nodes(std::vector<ProgramNode>& items, const std::string& prog)
    {
        for (ProgramNode& n : items) {
            where = "program '" + prog + "'";
            if (n.IsFolder()) {
                if (!n.alignProcUid.empty())
                    n.alignProcUid = Named(m.aligns, t.aligns,
                        [](const auto& l, const std::string& k) { return FindAlignProc(l, k); },
                        n.alignProcUid, "alignment");
                Nodes(n.children, prog);
            } else {
                n.recipeUid = RecipeRef(n.recipeUid, "a node");
            }
        }
    }
};

std::string Detail(int steps, int kept, int untaught, int models)
{
    std::string d = std::to_string(steps) + " step" + (steps == 1 ? "" : "s");
    if (kept)     d += ", " + std::to_string(kept) + " point(s) kept from this machine";
    if (untaught) d += ", " + std::to_string(untaught) + " point(s) untaught";
    if (models)   d += ", " + std::to_string(models) + " model(s) untaught";
    return d;
}

PorFixItem Blocker(PorFixKind kind, const std::string& what, const std::string& fix)
{
    PorFixItem i;
    i.kind = kind;
    i.what = what;
    i.fix  = fix;
    return i;
}

} // namespace

PorDeployPlan PlanDeploy(const PorSide& master, const PorSide& target, const CommandRegistry& reg,
                         const PorFixList& fixes, const PorDeployOptions& opt)
{
    PorDeployPlan plan;
    const PorSide& m = master;
    const PorSide& t = target;

    // ---- Blockers (§7.2) ----
    if (!m.Loaded())
        plan.blockers.push_back(Blocker(PorFixKind::MasterBroken, "product '" + m.product + "' on the master",
                                        "the master root has no such product"));
    if (!t.Loaded())
        plan.blockers.push_back(Blocker(PorFixKind::Configure, "product '" + t.product + "' on this machine",
                                        "this machine has no such product yet - a first deploy comes "
                                        "in a later version (phase 4d); create the product here first"));
    for (const PorFixItem& f : fixes.items)
        if (!(f.kind == PorFixKind::Teach && f.teachAfterDeploy)) plan.blockers.push_back(f);
    const auto twice = [&](const auto& list, const char* what, bool onMaster) {
        std::map<std::string, int> n;
        for (const auto& x : list) ++n[x.name];
        for (const auto& [name, c] : n)
            if (c > 1)
                plan.blockers.push_back(Blocker(
                    onMaster ? PorFixKind::MasterBroken : PorFixKind::Names,
                    std::string(what) + " '" + name + "' (named " + std::to_string(c) + " times" +
                        (onMaster ? " on the master)" : " here)"),
                    onMaster ? "rename on the master so each name is one " + std::string(what)
                             : "rename on this machine so the name points at one " + std::string(what)));
    };
    twice(m.recipes, "recipe", true);
    twice(t.recipes, "recipe", false);
    twice(m.programs, "program", true);
    twice(t.programs, "program", false);

    // ---- Translate (even when blocked: the preview still says what it would be) ----
    plan.positions = t.positions;
    Translator tr{m, t, reg, plan, {}, {}, {}, 0, 0, 0};

    // Recipe uids first: a sub_recipe or on_fail may name a recipe later in the file.
    std::set<std::string> takenRecipe;
    for (const Recipe& r : t.recipes) takenRecipe.insert(r.uid);
    for (const Recipe& r : m.recipes) {
        if (CountName(t.recipes, r.name) == 1) { tr.recipeUid[r.uid] = ByName(t.recipes, r.name)->uid; continue; }
        std::string uid = r.uid;   // a recipe new here keeps the master's uid when it is free
        while (uid.empty() || takenRecipe.count(uid)) uid = GenerateUid();
        takenRecipe.insert(uid);
        tr.recipeUid[r.uid] = uid;
    }

    std::map<std::string, Recipe> written;   // by name
    std::map<std::string, std::string> detail;
    for (const Recipe& r : m.recipes) {
        if (written.count(r.name)) continue;   // named twice on the master: blocked above
        const Recipe* old = CountName(t.recipes, r.name) == 1 ? ByName(t.recipes, r.name) : nullptr;
        written[r.name] = tr.TranslateRecipe(r, old);
        detail[r.name]  = Detail(static_cast<int>(CountLeafSteps(r.steps)), tr.recipePointsKept,
                                 tr.recipePointsUntaught, tr.recipeModelsUntaught);
    }
    // The target's order is kept: a replaced recipe stays where it was, new ones are appended.
    std::set<std::string> placed;
    for (const Recipe& r : t.recipes) {
        const auto w = written.find(r.name);
        if (w != written.end() && !placed.count(r.name)) {
            const bool same = RecipeToJson(w->second) == RecipeToJson(r);
            plan.recipes.push_back(w->second);
            plan.recipeItems.push_back({same ? PorPlanAction::Same : PorPlanAction::Replace, r.name,
                                        detail[r.name]});
            placed.insert(r.name);
        } else if (w == written.end()) {
            if (!opt.removeTargetOnly) plan.recipes.push_back(r);
            plan.recipeItems.push_back({opt.removeTargetOnly ? PorPlanAction::Remove : PorPlanAction::Keep,
                                        r.name, ""});
        }
    }
    for (const Recipe& r : m.recipes) {
        if (placed.count(r.name)) continue;
        placed.insert(r.name);
        plan.recipes.push_back(written[r.name]);
        plan.recipeItems.push_back({PorPlanAction::Add, r.name, detail[r.name]});
    }

    // Programs: a program the target has keeps ITS uid (a DUT's $run names it).
    std::set<std::string> takenProg;
    for (const Program& p : t.programs) takenProg.insert(p.uid);
    std::map<std::string, Program> writtenProg;
    for (const Program& p : m.programs) {
        if (writtenProg.count(p.name)) continue;
        Program out = p;
        if (CountName(t.programs, p.name) == 1) {
            out.uid = ByName(t.programs, p.name)->uid;
        } else {
            while (out.uid.empty() || takenProg.count(out.uid)) out.uid = GenerateUid();
            takenProg.insert(out.uid);
        }
        for (ProgramSlot& s : out.slots) tr.Nodes(s.items, p.name);
        writtenProg[p.name] = std::move(out);
    }
    std::set<std::string> placedProg;
    for (const Program& p : t.programs) {
        const auto w = writtenProg.find(p.name);
        if (w != writtenProg.end() && !placedProg.count(p.name)) {
            const bool same = ProgramToJson(w->second) == ProgramToJson(p);
            plan.programs.push_back(w->second);
            plan.programItems.push_back({same ? PorPlanAction::Same : PorPlanAction::Replace, p.name,
                                         std::to_string(w->second.slots.size()) + " slot(s)"});
            placedProg.insert(p.name);
        } else if (w == writtenProg.end()) {
            if (!opt.removeTargetOnly) plan.programs.push_back(p);
            plan.programItems.push_back({opt.removeTargetOnly ? PorPlanAction::Remove : PorPlanAction::Keep,
                                         p.name, ""});
        }
    }
    for (const Program& p : m.programs) {
        if (placedProg.count(p.name)) continue;
        placedProg.insert(p.name);
        plan.programs.push_back(writtenProg[p.name]);
        plan.programItems.push_back({PorPlanAction::Add, p.name,
                                     std::to_string(writtenProg[p.name].slots.size()) + " slot(s)"});
    }

    // Product files: the master's are copied over; the target's own are kept (§7.3).
    const auto files = [&](const std::set<std::string>& mine, const std::set<std::string>& theirs,
                           const char* sub, std::vector<std::string>* copy) {
        for (const std::string& f : mine) {
            const std::string a = ProductDir(m) + "/" + sub + "/" + f;
            const std::string b = ProductDir(t) + "/" + sub + "/" + f;
            const bool has = theirs.count(f) > 0;
            const bool same = has && SameFile(a, b);
            if (!same) copy->push_back(f);
            plan.fileItems.push_back({!has ? PorPlanAction::Add : same ? PorPlanAction::Same
                                                                       : PorPlanAction::Replace,
                                      std::string(sub) + "/" + f, ""});
        }
        for (const std::string& f : theirs)
            if (!mine.count(f)) plan.fileItems.push_back({PorPlanAction::Keep, std::string(sub) + "/" + f, ""});
    };
    files(m.productScripts, t.productScripts, "scripts", &plan.scripts);
    files(m.productImages, t.productImages, "prompt_images", &plan.images);

    return plan;
}

} // namespace firedog
