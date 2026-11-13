// PorFixList.cpp : see PorFixList.h.
#include "por/PorFixList.h"

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "command/ExecutionContext.h"   // IsSystemScript
#include "core/LightCallout.h"          // ChannelInLight

#include <algorithm>
#include <filesystem>
#include <map>
#include <set>
#include <system_error>

namespace firedog {

const char* PorFixKindTitle(PorFixKind k)
{
    switch (k) {
    case PorFixKind::MasterBroken: return "The master itself is broken - fix it there before distributing it";
    case PorFixKind::Teach:        return "Teach on this machine";
    case PorFixKind::Configure:    return "Configure this machine";
    case PorFixKind::Names:        return "Names to resolve";
    case PorFixKind::UpdateExe:    return "Update FireDog on this machine";
    }
    return "";
}

int PorFixList::Count(PorFixKind k) const
{
    int n = 0;
    for (const PorFixItem& i : items) n += i.kind == k;
    return n;
}

int PorFixList::TargetCount() const
{
    return static_cast<int>(items.size()) - Count(PorFixKind::MasterBroken);
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

std::string StepName(const RecipeStep& s)
{
    if (!s.nickname.empty()) return s.nickname;
    const std::string prefix = s.params.value("ctx_prefix", std::string());
    return prefix.empty() ? s.type : s.type + " " + prefix;
}

struct Builder {
    const PorSide&         m;
    const PorSide&         t;
    const CommandRegistry& reg;
    std::string            shipped;
    std::map<std::string, std::size_t> index;   // kind|what -> item
    std::vector<PorFixItem>            items;
    std::set<std::string>              stack;   // recipe uids being walked

    std::size_t Add(PorFixKind kind, const std::string& what, const std::string& fix,
             const std::string& use, PorFixJump jump = PorFixJump::None,
             const std::string& recipe = "", const std::string& section = "")
    {
        const std::string key = std::to_string(static_cast<int>(kind)) + "|" + what;
        auto it = index.find(key);
        if (it == index.end()) {
            PorFixItem i;
            i.kind    = kind;
            i.what    = what;
            i.fix     = fix;
            i.jump    = jump;
            i.recipe  = recipe;
            i.section = section;
            it = index.emplace(key, items.size()).first;
            items.push_back(std::move(i));
        }
        std::vector<std::string>& uses = items[it->second].uses;
        if (!use.empty() && std::find(uses.begin(), uses.end(), use) == uses.end()) uses.push_back(use);
        return it->second;
    }

    bool Shipped(const std::string& file) const
    {
        std::error_code ec;
        return !shipped.empty() && fs::is_regular_file(shipped + "/" + file, ec);
    }

    // The target's step that a deploy would pair with `s` in the recipe of the
    // same name: same type and the same ctx_prefix (a detection's identity in
    // its recipe), else the same nickname. Null when it cannot be told.
    const RecipeStep* Twin(const std::string& recipeName, const RecipeStep& s) const
    {
        if (CountName(t.recipes, recipeName) != 1) return nullptr;
        const Recipe* r = ByName(t.recipes, recipeName);
        const std::string prefix = s.params.value("ctx_prefix", std::string());
        if (prefix.empty() && s.nickname.empty()) return nullptr;
        const RecipeStep* found = nullptr;
        const auto visit = [&](const auto& self, const std::vector<RecipeStep>& steps) -> void {
            for (const RecipeStep& x : steps) {
                if (found) return;
                if (x.type == s.type &&
                    (prefix.empty() ? x.nickname == s.nickname
                                    : x.params.value("ctx_prefix", std::string()) == prefix)) {
                    found = &x;
                    return;
                }
                self(self, x.children);
                self(self, x.elseChildren);
            }
        };
        visit(visit, r->steps);
        return found;
    }

    void Hardware(const ParamSpec& p, const std::string& v, const nlohmann::json& merged,
                  const std::string& use, const std::string& recipe)
    {
        const std::string device = merged.value("device", std::string());
        const auto cfg = [&](const std::string& what, const std::string& fix, const char* section) {
            Add(PorFixKind::Configure, what, fix, use, PorFixJump::Hardware, recipe, section);
        };
        switch (p.type) {
        case ParamType::DeviceMotion:
        case ParamType::DeviceHexapod: {
            const auto it = t.hw.motion.find(v);
            if (it == t.hw.motion.end()) {
                cfg("motion device '" + v + "'", "add it to this machine's hardware.json (motion_devices)",
                    "Motion devices");
                break;
            }
            // The same name driving different axes is a different machine.
            const MotionConfig* mc = nullptr;
            const MotionConfig* tc = nullptr;
            for (const MotionConfig& c : m.hwConfig.motionDevices) if (c.name == v) mc = &c;
            for (const MotionConfig& c : t.hwConfig.motionDevices) if (c.name == v) tc = &c;
            if (mc && tc && (mc->axes != tc->axes || mc->controllerType != tc->controllerType))
                cfg("motion device '" + v + "' is " + mc->controllerType + " '" + mc->axes +
                        "' on the master, " + tc->controllerType + " '" + tc->axes + "' here",
                    "check this machine's hardware.json - the deck's moves name these axes",
                    "Motion devices");
            break;
        }
        case ParamType::DeviceIo:
            if (!t.hw.io.count(v))
                cfg("IO device '" + v + "'", "add it to this machine's hardware.json (io_devices)",
                    "IO devices");
            break;
        case ParamType::DevicePsu:
            if (v.empty()) {
                if (t.hw.psus.empty())
                    cfg("a PSU (the step uses the first one)", "add a PSU to this machine's hardware.json",
                        "PSUs");
            } else if (std::find(t.hw.psus.begin(), t.hw.psus.end(), v) == t.hw.psus.end()) {
                cfg("PSU '" + v + "'", "add it to this machine's hardware.json (instruments)", "PSUs");
            }
            break;
        case ParamType::Slide:
            if (!t.hw.slides.count(v))
                cfg("pneumatic slide '" + v + "'",
                    "add it to this machine's hardware.json (pneumatic_slides)", "Pneumatic slides");
            break;
        case ParamType::LightName: {
            bool found = false;
            for (const LightChannelConfig& ch : t.hw.lightChannels)
                found = found || ChannelInLight(ch.group, ch.name, v);
            if (!found)
                cfg("light '" + v + "'",
                    "name a channel (or a channel group) so in this machine's hardware.json lights",
                    "Lights");
            break;
        }
        case ParamType::PinName: {
            const std::string ioDev = merged.value("io_device", device);
            const auto ti = t.hw.io.find(ioDev);
            if (ti == t.hw.io.end()) break;   // the device itself is listed
            const bool in  = p.pinDir != PinDir::Output && ti->second.inputs.count(v);
            const bool out = p.pinDir != PinDir::Input && ti->second.outputs.count(v);
            if (!in && !out) {
                cfg("pin '" + v + "' on " + ioDev,
                    "name the pin it is wired to in this machine's hardware.json (io_devices pins)",
                    "IO devices");
                break;
            }
            // Wired the same? The pin NUMBER behind the name, on each side.
            const auto pinOf = [&](const HardwareConfig& hc, bool input) -> int {
                for (const IoDeviceConfig& d : hc.ioDevices) {
                    if (d.name != ioDev) continue;
                    for (const auto& [pin, name] : input ? d.inputNames : d.outputNames)
                        if (name == v) return pin;
                }
                return -1;
            };
            const bool input = in;
            const int mp = pinOf(m.hwConfig, input), tp = pinOf(t.hwConfig, input);
            if (mp >= 0 && tp >= 0 && mp != tp) {
                const std::string dir = input ? "IN " : "OUT ";
                cfg("pin '" + v + "' on " + ioDev + " is " + dir + std::to_string(mp) +
                        " on the master, " + dir + std::to_string(tp) + " here",
                    "check the panel wiring, or correct the pin in this machine's hardware.json",
                    "IO devices");
            }
            break;
        }
        default:
            break;
        }
    }

    void Step(const RecipeStep& s, const std::string& use, const Recipe& owner)
    {
        const std::string label = use + (s.enabled ? "" : "  (disabled)");
        const auto broken = [&](const std::string& what) {
            Add(PorFixKind::MasterBroken, what,
                "the master's deck refers to something the master does not have", label,
                PorFixJump::Recipe, owner.name);
        };
        const auto teach = [&](const std::string& what, const std::string& fix) {
            Add(PorFixKind::Teach, what, fix, label, PorFixJump::Recipe, owner.name);
        };
        const auto names = [&](const std::string& what) {
            Add(PorFixKind::Names, what, "rename one of them on this machine so the name points at one",
                label, PorFixJump::Recipe, owner.name);
        };

        if (s.type == "sub_recipe") {
            const std::string uid = s.params.value("uid", std::string());
            if (const Recipe* r = RecipeByUid(m.recipes, uid)) Walk(*r, use);
            else broken("sub_recipe of '" + owner.name + "' calls recipe uid " + uid);
        }

        const CommandSpec* spec = reg.Find(s.type);
        if (!spec) {
            if (!IsStructuralStep(s.type))
                Add(PorFixKind::UpdateExe, "command '" + s.type + "'",
                    "this FireDog build does not know it: update FireDog here, or the master still "
                    "uses a retired command",
                    label, PorFixJump::Recipe, owner.name);
        } else {
            const nlohmann::json merged = EffectiveParams(*spec, s.params);
            for (const ParamSpec& p : spec->params) {
                if (!merged.contains(p.name)) continue;
                const nlohmann::json& jv = merged[p.name];

                // A point taught INTO the step is the master's coordinate. A
                // deploy keeps the target's own where the step pairs up.
                if (p.type == ParamType::PointXyz) {
                    if (jv.is_object() && !jv.empty() && !Twin(owner.name, s))
                        teach("point '" + p.name + "' of step '" + StepName(s) + "' in recipe '" +
                                  owner.name + "'",
                              "it is the master's coordinate: re-teach it with the step's Teach here");
                    continue;
                }
                if (!jv.is_string()) continue;
                const std::string v = jv.get<std::string>();
                if (v.empty() && p.type != ParamType::DevicePsu) continue;

                // detect_object's own detector: taught on each machine.
                if (s.type == "detect_object" && p.name == "model") {
                    if (!FindVisionModel(m.vision, v)) {
                        broken("detection model of step '" + StepName(s) + "' in '" + owner.name + "'");
                        continue;
                    }
                    const RecipeStep* tw = Twin(owner.name, s);
                    const std::string tm = tw ? tw->params.value("model", std::string()) : std::string();
                    if (tm.empty() || !FindVisionModel(t.vision, tm))
                        teach("detection model of step '" + StepName(s) + "' in recipe '" + owner.name + "'",
                              "teach it in the step's Vision detect helper on this machine");
                    continue;
                }

                switch (p.type) {
                case ParamType::AlignUid: {
                    const AlignProc* ap = FindAlignProc(m.aligns, v);
                    if (!ap) { broken("alignment '" + v + "'"); break; }
                    const int n = CountName(t.aligns, ap->name);
                    if (n == 0)
                        teach("alignment '" + ap->name + "'",
                              "create it on the Alignment page and teach its stations and detectors here");
                    else if (n > 1)
                        names("alignment '" + ap->name + "' (twice here)");
                    break;
                }
                case ParamType::NominalUid: {
                    const AlignProc* ap = FindAlignProc(m.aligns, merged.value("alignment", std::string()));
                    if (!ap) break;   // the alignment row says so
                    const AlignNominal* nm = FindAlignNominal(*ap, v);
                    if (!nm) { broken("nominal '" + v + "' of alignment '" + ap->name + "'"); break; }
                    if (CountName(t.aligns, ap->name) != 1) break;
                    if (!FindAlignNominal(*ByName(t.aligns, ap->name), nm->name))
                        teach("nominal '" + nm->name + "' of alignment '" + ap->name + "'",
                              "teach it through a live frame on the Alignment page here");
                    break;
                }
                case ParamType::VisionUid: {
                    const VisionModel* vm = FindVisionModel(m.vision, v);
                    if (!vm) { broken("detection model '" + v + "'"); break; }
                    if (vm->name.rfind("vdet_", 0) == 0) break;   // private: its owner is checked
                    const int n = CountName(t.vision, vm->name);
                    if (n == 0)
                        teach("detection model '" + vm->name + "'", "teach it on this machine's camera");
                    else if (n > 1)
                        names("detection model '" + vm->name + "' (twice here)");
                    break;
                }
                case ParamType::GripUid: {
                    const GripProfile* g = FindGripProfile(m.grips, v);
                    if (!g) { broken("grip profile '" + v + "'"); break; }
                    const int n = CountName(t.grips, g->name);
                    if (n == 0)
                        teach("grip profile '" + g->name + "'",
                              "create it on the Gripped Object page; teach its inspect pose and reference here");
                    else if (n > 1)
                        names("grip profile '" + g->name + "' (twice here)");
                    break;
                }
                case ParamType::TrayUid: {
                    const WaffleTray* tr = FindWaffleTray(m.trays, v);
                    if (!tr) { broken("tray '" + v + "'"); break; }
                    const int n = CountName(t.trays, tr->name);
                    if (n == 0)
                        teach("tray '" + tr->name + "'",
                              "create it on the Waffle Tray page and teach its three corners here");
                    else if (n > 1)
                        names("tray '" + tr->name + "' (twice here)");
                    break;
                }
                case ParamType::MapName: {
                    const std::string device = merged.value("device", std::string());
                    const auto has = [&](const PorSide& side) {
                        const auto it = side.maps.find(device);
                        if (it == side.maps.end()) return false;
                        for (const MotionMap& mm : it->second) if (mm.name == v) return true;
                        return false;
                    };
                    if (!has(m)) broken("map '" + v + "' on " + device);
                    else if (!has(t))
                        Add(PorFixKind::Configure, "map '" + v + "' on " + device,
                            "create it on the Motion page (Mapped Path) here - maps belong to the machine",
                            label, PorFixJump::Recipe, owner.name);
                    break;
                }
                case ParamType::ScriptFile: {
                    const std::string file = fs::path(v).filename().string();
                    if (IsSystemScript(file) || Shipped(file)) break;   // comes with the exe
                    if (!m.scripts.count(file)) { broken("script '" + file + "'"); break; }
                    if (m.productScripts.count(file)) break;            // the deploy brings it
                    if (!t.scripts.count(file))
                        Add(PorFixKind::Configure, "script '" + file + "'",
                            "copy it to <root>/config/scripts here - it is a machine script on the "
                            "master, not part of the product",
                            label, PorFixJump::Recipe, owner.name);
                    break;
                }
                case ParamType::ImageFile:
                    if (!m.promptImages.count(v)) { broken("prompt picture '" + v + "'"); break; }
                    if (m.productImages.count(v)) break;               // the deploy brings it
                    if (!t.promptImages.count(v))
                        Add(PorFixKind::Configure, "prompt picture '" + v + "'",
                            "copy it to <root>/config/prompt_images here (a machine picture on the master)",
                            label, PorFixJump::Recipe, owner.name);
                    break;
                case ParamType::DeviceMotion: case ParamType::DeviceHexapod: case ParamType::DeviceIo:
                case ParamType::DevicePsu:    case ParamType::Slide:         case ParamType::LightName:
                case ParamType::PinName:
                    if (t.hasHardware) Hardware(p, v, merged, label, owner.name);
                    break;
                default:
                    break;
                }
            }
        }
    }

    void Steps(const std::vector<RecipeStep>& steps, const std::string& where, const Recipe& owner,
               std::vector<std::size_t>& path, std::size_t base)
    {
        for (std::size_t i = 0; i < steps.size(); ++i) {
            const RecipeStep& s = steps[i];
            path.push_back(base + i);
            Step(s, where + " > " + StepPathLabel(path) + " " + s.type, owner);
            Steps(s.children, where, owner, path, 0);
            Steps(s.elseChildren, where, owner, path, kElseBranchBase);
            path.pop_back();
        }
    }

    void Walk(const Recipe& r, const std::string& where)
    {
        if (!stack.insert(r.uid).second) return;
        const std::string here = where + " > " + r.name;
        std::vector<std::size_t> path;
        Steps(r.steps, here, r, path, 0);
        if (!r.onFailUid.empty()) {
            if (const Recipe* f = RecipeByUid(m.recipes, r.onFailUid)) Walk(*f, here + " on_fail");
            else Add(PorFixKind::MasterBroken, "on_fail of '" + r.name + "' names recipe uid " + r.onFailUid,
                     "the master's deck refers to something the master does not have", here,
                     PorFixJump::Recipe, r.name);
        }
        stack.erase(r.uid);
    }

    void Nodes(const std::vector<ProgramNode>& items, const std::string& where)
    {
        for (const ProgramNode& n : items) {
            if (n.IsFolder()) { Nodes(n.children, where + " > " + n.name); continue; }
            if (const Recipe* r = RecipeByUid(m.recipes, n.recipeUid)) Walk(*r, where);
            else Add(PorFixKind::MasterBroken, "deck node names recipe uid " + n.recipeUid,
                     "the master's deck refers to a recipe the master does not have", where);
        }
    }
};

} // namespace

PorFixList BuildFixList(const PorSide& master, const PorSide& target, const CommandRegistry& reg,
                        const PorPositionsReport& positions, const std::string& shippedScriptsDir)
{
    Builder b{master, target, reg, shippedScriptsDir, {}, {}, {}};

    if (!target.hasHardware)
        b.Add(PorFixKind::Configure, "this machine's hardware.json",
              "there is none under the target root - no device or pin name can be checked", "",
              PorFixJump::Hardware, "", "Motion devices");

    for (const Program& p : master.programs)
        for (const ProgramSlot& s : p.slots) b.Nodes(s.items, p.name + " > " + s.name);

    // Positions: from the positions page's own trace, so the two agree.
    for (const PorPosRow& r : positions.rows) {
        if (r.tier != PorPosTier::Deck && r.tier != PorPosTier::Private) continue;
        const std::string where = r.owner.empty() ? "" : " (" + r.owner + ")";
        if (r.status == PorStatus::Missing && !r.hasMaster) {
            b.items[b.Add(PorFixKind::MasterBroken, "position uid " + r.name + " on " + r.device,
                          "the master's deck refers to a position the master does not have", "",
                          PorFixJump::Positions)].uses = r.uses;
        } else if (r.status == PorStatus::Missing) {
            b.items[b.Add(PorFixKind::Teach, "position '" + r.name + "' on " + r.device + where,
                          r.owner.empty() ? "teach it here: the Motion page, or the deck's Teach run once "
                                            "the recipe is on this machine"
                                          : r.note + " - teach it through its owner on this machine",
                          "", PorFixJump::Positions)].uses = r.uses;
        } else if (r.ambiguous) {
            b.items[b.Add(PorFixKind::Names,
                          "position '" + r.name + "' on " + r.device + " (name used twice)",
                          "rename one so the name points at one position", "",
                          PorFixJump::Positions)].uses = r.uses;
        }
    }

    PorFixList out;
    out.items = std::move(b.items);
    std::stable_sort(out.items.begin(), out.items.end(), [](const PorFixItem& a, const PorFixItem& c) {
        return static_cast<int>(a.kind) < static_cast<int>(c.kind);
    });
    return out;
}

} // namespace firedog
