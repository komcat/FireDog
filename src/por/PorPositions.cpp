// PorPositions.cpp : see PorPositions.h.
#include "por/PorPositions.h"

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "command/ExecutionContext.h"   // IsSystemScript

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>
#include <utility>

namespace firedog {

namespace {

// How a private pose is found again on the target: through its owner.
enum class OwnerKind { None, Station1, Station2, NominalCam, GripInspect };

struct Need {
    std::string device, uid;
    PorPosTier  tier = PorPosTier::Deck;
    OwnerKind   kind = OwnerKind::None;
    std::string ownerName;     // the alignment / grip profile's name
    std::string nominalName;   // NominalCam: the nominal's name within it
    std::string owner;         // display text
    std::vector<std::string> uses;
    std::set<std::string>    useSet;
};

struct Tracer {
    const PorSide&         m;
    const CommandRegistry& reg;
    std::map<std::pair<std::string, std::string>, Need> needs;   // (device, uid)
    std::vector<std::pair<std::string, std::string>>    order;   // first use
    std::set<std::string>                               untraced;
    std::set<std::pair<std::string, std::string>>       maps;    // (device, map name)
    std::set<std::string>                               stack;   // recipe uids being walked

    Need& Add(const std::string& device, const std::string& uid, PorPosTier tier,
              const std::string& use)
    {
        const auto key = std::make_pair(device, uid);
        auto it = needs.find(key);
        if (it == needs.end()) {
            Need n;
            n.device = device;
            n.uid    = uid;
            n.tier   = tier;
            it = needs.emplace(key, std::move(n)).first;
            order.push_back(key);
        } else if (tier == PorPosTier::Deck) {
            it->second.tier = PorPosTier::Deck;   // the deck naming it directly outranks
        }
        if (it->second.useSet.insert(use).second) it->second.uses.push_back(use);
        return it->second;
    }

    void Align(const std::string& ref, const std::string& use)
    {
        const AlignProc* ap = FindAlignProc(m.aligns, ref);
        if (!ap) return;
        const auto own = [&](const std::string& uid, OwnerKind k, const std::string& text,
                             const std::string& nominal) {
            if (uid.empty()) return;
            Need& n = Add(ap->device, uid, PorPosTier::Private, use);
            n.kind        = k;
            n.ownerName   = ap->name;
            n.nominalName = nominal;
            n.owner       = text;
        };
        own(ap->s1.posUid, OwnerKind::Station1, "alignment " + ap->name + " station 1", "");
        own(ap->s2.posUid, OwnerKind::Station2, "alignment " + ap->name + " station 2", "");
        for (const AlignNominal& nm : ap->nominals)
            own(nm.camUid, OwnerKind::NominalCam,
                "alignment " + ap->name + " nominal " + nm.name + " camera", nm.name);
    }

    void Grip(const std::string& ref, const std::string& use)
    {
        const GripProfile* g = FindGripProfile(m.grips, ref);
        if (!g || g->inspectUid.empty()) return;
        Need& n = Add(g->device, g->inspectUid, PorPosTier::Private, use);
        n.kind      = OwnerKind::GripInspect;
        n.ownerName = g->name;
        n.owner     = "grip " + g->name + " inspect";
    }

    void Steps(const std::vector<RecipeStep>& steps, const std::string& where,
               std::vector<std::size_t>& path, std::size_t base)
    {
        for (std::size_t i = 0; i < steps.size(); ++i) {
            const RecipeStep& s = steps[i];
            path.push_back(base + i);
            const std::string use = where + " > " + StepPathLabel(path) + " " + s.type;

            if (s.type == "sub_recipe") {
                if (const Recipe* r = RecipeByUid(m.recipes, s.params.value("uid", std::string())))
                    Walk(*r, use);
            }
            if (const CommandSpec* spec = reg.Find(s.type)) {
                const nlohmann::json merged = EffectiveParams(*spec, s.params);
                const std::string device = merged.value("device", std::string());
                for (const ParamSpec& p : spec->params) {
                    if (!merged.contains(p.name) || !merged[p.name].is_string()) continue;
                    const std::string v = merged[p.name].get<std::string>();
                    if (v.empty()) continue;
                    switch (p.type) {
                    case ParamType::PositionUid: Add(device, v, PorPosTier::Deck, use); break;
                    case ParamType::AlignUid:    Align(v, use); break;
                    case ParamType::GripUid:     Grip(v, use); break;
                    case ParamType::MapName: {
                        maps.insert({device, v});
                        const auto it = m.maps.find(device);
                        if (it == m.maps.end()) break;
                        for (const MotionMap& mm : it->second) {
                            if (mm.name != v) continue;
                            for (const MotionMapNode& node : mm.nodes)
                                Add(device, node.uid, PorPosTier::Deck, use + " (map " + v + ")");
                        }
                        break;
                    }
                    case ParamType::ScriptFile: {
                        const std::string file = std::filesystem::path(v).filename().string();
                        if (!IsSystemScript(file)) untraced.insert(file);
                        break;
                    }
                    default: break;
                    }
                }
            }
            Steps(s.children, where, path, 0);
            Steps(s.elseChildren, where, path, kElseBranchBase);
            path.pop_back();
        }
    }

    void Walk(const Recipe& r, const std::string& where)
    {
        if (!stack.insert(r.uid).second) return;   // a sub_recipe cycle: walked already
        const std::string here = where + " > " + r.name;
        std::vector<std::size_t> path;
        Steps(r.steps, here, path, 0);
        if (!r.onFailUid.empty())
            if (const Recipe* f = RecipeByUid(m.recipes, r.onFailUid)) Walk(*f, here + " on_fail");
        stack.erase(r.uid);
    }

    void Nodes(const std::vector<ProgramNode>& items, const std::string& where)
    {
        for (const ProgramNode& n : items) {
            if (n.IsFolder()) { Nodes(n.children, where + " > " + n.name); continue; }
            if (const Recipe* r = RecipeByUid(m.recipes, n.recipeUid)) Walk(*r, where);
        }
    }
};

void Deltas(PorPosRow& r, double mm, double deg)
{
    double s = 0.0;
    for (const char* ax : {"X", "Y", "Z"}) {
        const auto a = r.master.find(ax), b = r.target.find(ax);
        if (a != r.master.end() && b != r.target.end()) s += (a->second - b->second) * (a->second - b->second);
    }
    r.dXyz = std::sqrt(s);
    for (const char* ax : {"U", "V", "W"}) {
        const auto a = r.master.find(ax), b = r.target.find(ax);
        if (a != r.master.end() && b != r.target.end())
            r.dRot = std::max(r.dRot, std::fabs(a->second - b->second));
    }
    std::set<std::string> ka, kb;
    for (const auto& kv : r.master) ka.insert(kv.first);
    for (const auto& kv : r.target) kb.insert(kv.first);
    r.axesDiffer    = ka != kb;
    r.overThreshold = r.dXyz > mm || r.dRot > deg;
}

// The target's counterpart of a traced need. Sets the row's note on a miss.
const SavedPos* FindOnTarget(const PorSide& t, const Need& n, const SavedPos* mp,
                             PorPosRow& row, std::string* deviceOut)
{
    *deviceOut = n.device;
    switch (n.kind) {
    case OwnerKind::Station1:
    case OwnerKind::Station2:
    case OwnerKind::NominalCam: {
        const AlignProc* ap = FindAlignProc(t.aligns, n.ownerName);
        if (!ap) { row.note = "no alignment '" + n.ownerName + "' on the target"; return nullptr; }
        *deviceOut = ap->device;
        std::string uid;
        if (n.kind == OwnerKind::Station1)      uid = ap->s1.posUid;
        else if (n.kind == OwnerKind::Station2) uid = ap->s2.posUid;
        else {
            const AlignNominal* nm = FindAlignNominal(*ap, n.nominalName);
            if (!nm) { row.note = "no nominal '" + n.nominalName + "' on the target"; return nullptr; }
            uid = nm->camUid;
        }
        const SavedPos* sp = uid.empty() ? nullptr : PorFindPosByUid(t.positions, ap->device, uid);
        if (!sp) row.note = "not taught on the target";
        return sp;
    }
    case OwnerKind::GripInspect: {
        const GripProfile* g = FindGripProfile(t.grips, n.ownerName);
        if (!g) { row.note = "no grip profile '" + n.ownerName + "' on the target"; return nullptr; }
        *deviceOut = g->device;
        const SavedPos* sp = g->inspectUid.empty() ? nullptr
                                                   : PorFindPosByUid(t.positions, g->device, g->inspectUid);
        if (!sp) row.note = "not taught on the target";
        return sp;
    }
    case OwnerKind::None:
        break;
    }
    if (!mp) return nullptr;
    int count = 0;
    const SavedPos* sp = PorFindPosByName(t.positions, n.device, mp->name, &count);
    if (count > 1) row.ambiguous = true;
    if (!sp) row.note = "no position '" + std::string(mp->name) + "' on " + n.device + " on the target";
    return sp;
}

void Judge(PorPosRow& r, PorStatus whenMissing)
{
    if (!r.hasTarget || !r.hasMaster) { r.status = r.hasMaster ? whenMissing : PorStatus::OnlyTarget; return; }
    if (r.ambiguous)                         r.status = PorStatus::Ambiguous;
    else if (r.axesDiffer || r.overThreshold) r.status = PorStatus::Measured;
    else                                     r.status = PorStatus::Same;
}

} // namespace

PorPositionsReport ComparePositions(const PorSide& master, const PorSide& target,
                                    const CommandRegistry& reg,
                                    double mmThreshold, double degThreshold)
{
    PorPositionsReport rep;
    Tracer tr{master, reg, {}, {}, {}, {}, {}};
    for (const Program& p : master.programs)
        for (const ProgramSlot& s : p.slots) tr.Nodes(s.items, p.name + " > " + s.name);

    std::set<std::pair<std::string, std::string>> matchedTarget;   // (device, uid)
    std::set<std::pair<std::string, std::string>> traced;          // (device, uid) on the master

    // Deck first, then the private poses - each in first-use order.
    for (PorPosTier tier : {PorPosTier::Deck, PorPosTier::Private}) {
        for (const auto& key : tr.order) {
            const Need& n = tr.needs.at(key);
            if (n.tier != tier) continue;
            traced.insert(key);
            PorPosRow row;
            row.tier   = n.tier;
            row.device = n.device;
            row.owner  = n.owner;
            row.uses   = n.uses;
            const SavedPos* mp = PorFindPosByUid(master.positions, n.device, n.uid);
            if (mp) {
                row.hasMaster = true;
                row.name      = mp->name;
                row.master    = mp->vals;
                int count = 0;
                PorFindPosByName(master.positions, n.device, mp->name, &count);
                if (count > 1 && n.kind == OwnerKind::None) row.ambiguous = true;
            } else {
                row.name   = n.uid;
                row.status = PorStatus::Missing;
                row.note   = "the master's own reference does not resolve";
                rep.rows.push_back(std::move(row));
                continue;
            }
            std::string tdev;
            if (const SavedPos* tp = FindOnTarget(target, n, mp, row, &tdev)) {
                row.hasTarget = true;
                row.target    = tp->vals;
                matchedTarget.insert({tdev, tp->uid});
                Deltas(row, mmThreshold, degThreshold);
            }
            Judge(row, PorStatus::Missing);
            rep.rows.push_back(std::move(row));
        }
    }

    // The master's visible positions the deck does not use.
    for (const auto& [dev, list] : master.positions) {
        for (const SavedPos& mp : list) {
            if (mp.hidden || traced.count({dev, mp.uid})) continue;
            PorPosRow row;
            row.tier      = PorPosTier::Unused;
            row.device    = dev;
            row.name      = mp.name;
            row.hasMaster = true;
            row.master    = mp.vals;
            int cm = 0, ct = 0;
            PorFindPosByName(master.positions, dev, mp.name, &cm);
            if (const SavedPos* tp = PorFindPosByName(target.positions, dev, mp.name, &ct)) {
                row.hasTarget = true;
                row.target    = tp->vals;
                matchedTarget.insert({dev, tp->uid});
                Deltas(row, mmThreshold, degThreshold);
            }
            row.ambiguous = cm > 1 || ct > 1;
            Judge(row, PorStatus::OnlyMaster);
            rep.rows.push_back(std::move(row));
        }
    }

    // The target's visible positions nothing on the master corresponds to.
    for (const auto& [dev, list] : target.positions) {
        for (const SavedPos& tp : list) {
            if (tp.hidden || matchedTarget.count({dev, tp.uid})) continue;
            if (PorFindPosByName(master.positions, dev, tp.name)) continue;   // an ambiguous twin
            PorPosRow row;
            row.tier      = PorPosTier::TargetOnly;
            row.device    = dev;
            row.name      = tp.name;
            row.hasTarget = true;
            row.target    = tp.vals;
            row.status    = PorStatus::OnlyTarget;
            rep.rows.push_back(std::move(row));
        }
    }

    for (const auto& [dev, name] : tr.maps) {
        bool found = false;
        const auto it = target.maps.find(dev);
        if (it != target.maps.end())
            for (const MotionMap& mm : it->second) found = found || mm.name == name;
        if (!found) rep.mapIssues.push_back("map '" + name + "' on " + dev + " is missing on the target");
    }
    rep.untraced.assign(tr.untraced.begin(), tr.untraced.end());
    return rep;
}

} // namespace firedog
