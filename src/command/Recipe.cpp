// Recipe.cpp
#include "command/Recipe.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"   // CheckConditionShape
#include "command/MotionWaits.h"        // kTeachParamName (parallel refuses it)
#include "core/Log.h"
#include "core/Product.h"               // GenerateUid — backfill for recipes without one
#include "hardware/HardwareManager.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <set>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/recipes.local.json";
constexpr const char* kBasePath  = "config/recipes.json";
} // namespace

bool IsStructuralStep(const std::string& type)
{
    return type == "loop" || type == "if" || type == "group" || type == "parallel" ||
           type == "sub_recipe" || type == "stop_recipe";
}

// ---- JSON <-> model ---------------------------------------------------------

nlohmann::json StepToJson(const RecipeStep& s)
{
    nlohmann::json j;
    j["type"] = s.type;
    if (!s.nickname.empty()) j["nickname"] = s.nickname;
    if (!s.enabled) j["enabled"] = false;
    if (!s.params.empty()) j["params"] = s.params;
    switch (s.onError) {
    case RecipeStep::OnError::Abort:    break;   // default — omit
    case RecipeStep::OnError::Continue: j["on_error"] = "continue"; break;
    case RecipeStep::OnError::Retry:    j["on_error"] = {{"retry", s.retryCount}}; break;
    }
    if (!s.children.empty()) {
        nlohmann::json arr = nlohmann::json::array();
        for (const RecipeStep& c : s.children) arr.push_back(StepToJson(c));
        j["children"] = std::move(arr);
    }
    if (!s.elseChildren.empty()) {
        nlohmann::json arr = nlohmann::json::array();
        for (const RecipeStep& c : s.elseChildren) arr.push_back(StepToJson(c));
        j["else"] = std::move(arr);
    }
    return j;
}

RecipeStep StepFromJson(const nlohmann::json& j)
{
    RecipeStep s;
    s.type     = j.value("type", std::string());
    s.nickname = j.value("nickname", std::string());
    s.enabled  = j.value("enabled", true);
    if (j.contains("params") && j["params"].is_object()) s.params = j["params"];
    if (j.contains("on_error")) {
        const nlohmann::json& e = j["on_error"];
        if (e.is_string() && e.get<std::string>() == "continue") {
            s.onError = RecipeStep::OnError::Continue;
        } else if (e.is_object() && e.contains("retry") && e["retry"].is_number()) {
            s.onError    = RecipeStep::OnError::Retry;
            s.retryCount = e["retry"].get<int>();
            if (s.retryCount < 0) s.retryCount = 0;
        }
        // anything else (incl. "abort") = default Abort
    }
    if (j.contains("children") && j["children"].is_array()) {
        for (const nlohmann::json& c : j["children"]) {
            if (c.is_object()) s.children.push_back(StepFromJson(c));
        }
    }
    if (j.contains("else") && j["else"].is_array()) {
        for (const nlohmann::json& c : j["else"]) {
            if (c.is_object()) s.elseChildren.push_back(StepFromJson(c));
        }
    }
    return s;
}

nlohmann::json RecipeToJson(const Recipe& r)
{
    nlohmann::json j;
    j["uid"]  = r.uid;
    j["name"] = r.name;
    if (!r.description.empty()) j["description"] = r.description;
    // Both only when set, so a file written before they existed round-trips
    // byte for byte and stays clean in the dirty compare.
    if (r.hidden) j["hidden"] = true;
    if (!r.tags.empty()) j["tags"] = r.tags;
    nlohmann::json arr = nlohmann::json::array();
    for (const RecipeStep& s : r.steps) arr.push_back(StepToJson(s));
    j["steps"] = std::move(arr);
    return j;
}

Recipe RecipeFromJson(const nlohmann::json& j)
{
    Recipe r;
    r.uid         = j.value("uid", std::string());
    r.name        = j.value("name", std::string("recipe"));
    r.description = j.value("description", std::string());
    r.hidden      = j.value("hidden", false);
    if (j.contains("tags") && j["tags"].is_array()) {
        std::string joined;
        for (const nlohmann::json& t : j["tags"])
            if (t.is_string()) joined += t.get<std::string>() + ",";
        r.tags = ParseTagList(joined);   // the same trim/dedupe the editor applies
    }
    if (r.uid.empty()) r.uid = GenerateUid();
    if (j.contains("steps") && j["steps"].is_array()) {
        for (const nlohmann::json& s : j["steps"]) {
            if (s.is_object()) r.steps.push_back(StepFromJson(s));
        }
    }
    return r;
}

bool RecipeHasTag(const Recipe& r, const std::string& tag)
{
    for (const std::string& t : r.tags)
        if (t == tag) return true;
    return false;
}

std::vector<std::string> ParseTagList(const std::string& text)
{
    std::vector<std::string> out;
    std::string cur;
    const auto flush = [&]() {
        std::size_t b = 0, e = cur.size();
        while (b < e && std::isspace(static_cast<unsigned char>(cur[b]))) ++b;
        while (e > b && std::isspace(static_cast<unsigned char>(cur[e - 1]))) --e;
        const std::string tag = cur.substr(b, e - b);
        cur.clear();
        if (tag.empty()) return;
        for (const std::string& t : out)
            if (t == tag) return;
        out.push_back(tag);
    };
    for (const char c : text) {
        if (c == ',' || c == ';') flush();
        else cur += c;
    }
    flush();
    return out;
}

// ---- Persistence ------------------------------------------------------------

std::vector<Recipe> LoadRecipesFile(const std::string& path, std::string* uidOut)
{
    std::ifstream f(path);
    if (!f.is_open()) return {};
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", std::string());
        std::vector<Recipe> out;
        if (j.contains("recipes") && j["recipes"].is_array()) {
            for (const nlohmann::json& rj : j["recipes"]) {
                if (rj.is_object()) out.push_back(RecipeFromJson(rj));
            }
        }
        FD_INFO("recipe", "loaded %zu recipe(s) from %s", out.size(), path.c_str());
        return out;
    } catch (const std::exception& e) {
        FD_WARN("recipe", "recipes load failed (%s): %s", path.c_str(), e.what());
        return {};
    }
}

std::vector<Recipe> LoadRecipes()
{
    std::ifstream probe(kLocalPath);
    if (probe.is_open()) {
        probe.close();
        return LoadRecipesFile(kLocalPath);
    }
    return LoadRecipesFile(kBasePath);
}

bool SaveRecipesFile(const std::string& path, const std::vector<Recipe>& recipes,
                     const std::string& uid)
{
    nlohmann::json j;
    j["uid"] = uid;
    nlohmann::json arr = nlohmann::json::array();
    for (const Recipe& r : recipes) arr.push_back(RecipeToJson(r));
    j["recipes"] = std::move(arr);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("recipe", "recipes save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

// ---- Sub-recipe inlining ----------------------------------------------------

namespace {

const Recipe* FindByUid(const std::vector<Recipe>& library, const std::string& uid)
{
    for (const Recipe& r : library) {
        if (r.uid == uid) return &r;
    }
    return nullptr;
}

bool InlineSteps(std::vector<RecipeStep>& steps, const std::vector<Recipe>& library,
                 std::set<std::string>& chain, int depth, std::string* err)
{
    if (depth > kMaxRecipeDepth) {
        if (err) *err = "recipe nesting exceeds depth " + std::to_string(kMaxRecipeDepth);
        return false;
    }
    for (RecipeStep& s : steps) {
        if (s.type == "sub_recipe") {
            const std::string uid = s.params.value("uid", std::string());
            if (chain.count(uid) != 0) {
                if (err) *err = "sub-recipe cycle involving uid '" + uid + "'";
                return false;
            }
            const Recipe* sub = FindByUid(library, uid);
            if (!sub) {
                if (err) *err = "sub-recipe uid '" + uid + "' not found";
                return false;
            }
            s.children = sub->steps;   // inline a copy; executes like a group
            if (s.nickname.empty()) s.nickname = sub->name;
            chain.insert(uid);
            if (!InlineSteps(s.children, library, chain, depth + 1, err)) return false;
            chain.erase(uid);
            continue;   // don't recurse into children twice
        }
        if (!InlineSteps(s.children, library, chain, depth + 1, err)) return false;
        if (!InlineSteps(s.elseChildren, library, chain, depth + 1, err)) return false;
    }
    return true;
}

} // namespace

bool InlineSubRecipes(Recipe& r, const std::vector<Recipe>& library, std::string* err)
{
    std::set<std::string> chain;
    chain.insert(r.uid);
    return InlineSteps(r.steps, library, chain, 1, err);
}

// ---- Leaf counting ----------------------------------------------------------

std::size_t CountLeafSteps(const std::vector<RecipeStep>& steps)
{
    std::size_t n = 0;
    for (const RecipeStep& s : steps) {
        if (!s.enabled) continue;
        if (s.type == "loop") {
            std::size_t body = CountLeafSteps(s.children);
            if (s.params.contains("count") && s.params["count"].is_number()) {
                const double c = s.params["count"].get<double>();
                if (c > 0) body *= static_cast<std::size_t>(c);
            }
            n += body;
        } else if (s.type == "if") {
            // Unknown branch at plan time — count the larger one.
            n += std::max(CountLeafSteps(s.children), CountLeafSteps(s.elseChildren));
        } else if (s.type == "group" || s.type == "sub_recipe" || s.type == "parallel") {
            n += CountLeafSteps(s.children);   // a parallel lane is one leaf each
        } else if (s.type == "stop_recipe") {
            // structural, no leaf
        } else {
            n += 1;
        }
    }
    return n;
}

// ---- Validation -------------------------------------------------------------

namespace {

struct ValidateEnv {
    const CommandRegistry& reg;
    const HardwareManager& hw;
    const SavedPosMap&     positions;
    const MotionMapSet&    maps;
    const std::vector<AlignProc>& aligns;
    const std::vector<VisionModel>& visionModels;
    const std::vector<GripProfile>& grips;
    const std::vector<WaffleTray>&  trays;
    const std::vector<Recipe>& library;
    bool requireConnected = false;
    std::vector<RecipeIssue>* issues = nullptr;
};

void Issue(const ValidateEnv& env, const std::string& where,
           const std::vector<std::size_t>& path, std::string msg)
{
    env.issues->push_back({where, std::move(msg), path});
}

int MotionIdx(const HardwareManager& hw, const std::string& name)
{
    for (std::size_t i = 0; i < hw.MotionCount(); ++i) {
        if (hw.MotionName(i) == name) return static_cast<int>(i);
    }
    return -1;
}

int IoIdx(const HardwareManager& hw, const std::string& name)
{
    for (std::size_t i = 0; i < hw.IoCount(); ++i) {
        if (hw.IoName(i) == name) return static_cast<int>(i);
    }
    return -1;
}

bool SlideKnown(const HardwareManager& hw, const std::string& name)
{
    for (std::size_t i = 0; i < hw.SlideCount(); ++i) {
        if (hw.Slide(i) && hw.Slide(i)->Name() == name) return true;
    }
    return false;
}

// Semantic reference checks for one merged param set. The PositionUid /
// MapName / PinName types resolve against the step's "device" param (the
// vocabulary convention: commands with such params also carry a device param),
// and NominalUid against its "alignment" for the same reason.
void CheckReferences(const ValidateEnv& env, const std::string& where,
                     const std::vector<std::size_t>& path,
                     const CommandSpec& spec, const nlohmann::json& merged)
{
    const std::string device    = merged.value("device", std::string());
    const std::string alignment = merged.value("alignment", std::string());
    // A pin belongs to an IO device, and in most commands "device" IS that IO
    // device. The needle family is the exception: its "device" is the gantry, so
    // it names the IO fleet separately and the pin has to follow that one — or a
    // perfectly good pin would be reported unknown on a motion controller.
    const std::string ioDevice  = merged.value("io_device", device);
    for (const ParamSpec& p : spec.params) {
        if (!merged.contains(p.name) || !merged[p.name].is_string()) continue;
        const std::string v = merged[p.name].get<std::string>();
        if (v.empty() && !p.required) continue;   // optional and unset (e.g. clear_latch pin)
        switch (p.type) {
        case ParamType::DeviceMotion:
        case ParamType::DeviceHexapod: {
            const int idx = MotionIdx(env.hw, v);
            if (idx < 0) {
                Issue(env, where, path, "unknown motion device '" + v + "'");
            } else if (p.type == ParamType::DeviceHexapod &&
                       env.hw.MotionDevice(static_cast<std::size_t>(idx))->Type() != "PI") {
                Issue(env, where, path,
                      "'" + v + "' is not a PI hexapod (scan commands are PI-only)");
            } else if (env.requireConnected &&
                       env.hw.MotionState(static_cast<std::size_t>(idx)) != ConnState::Connected) {
                Issue(env, where, path, "motion device '" + v + "' is not connected");
            }
            break;
        }
        case ParamType::DeviceIo: {
            const int idx = IoIdx(env.hw, v);
            if (idx < 0) {
                Issue(env, where, path, "unknown io device '" + v + "'");
            } else if (env.requireConnected &&
                       env.hw.IoState(static_cast<std::size_t>(idx)) != ConnState::Connected) {
                Issue(env, where, path, "io device '" + v + "' is not connected");
            }
            break;
        }
        case ParamType::DevicePsu:
            // Only a NAMED supply reaches here (empty = the first, checked by
            // Needs::Psu below). Connection is checked there too, per device.
            if (env.hw.PsuIndex(v) < 0) Issue(env, where, path, "unknown PSU '" + v + "'");
            break;
        case ParamType::Slide:
            if (!SlideKnown(env.hw, v)) Issue(env, where, path, "unknown slide '" + v + "'");
            break;
        case ParamType::PositionUid: {
            const auto it = env.positions.find(device);
            bool found = false;
            if (it != env.positions.end()) {
                for (const SavedPos& sp : it->second) {
                    if (sp.uid == v) { found = true; break; }
                }
            }
            if (!found)
                Issue(env, where, path, "position uid '" + v + "' not found for device '" +
                                      device + "'");
            break;
        }
        case ParamType::AlignUid: {
            // Resolved by uid OR name (FindAlignProc), and NOT device-scoped:
            // the procedure carries its own device.
            if (!FindAlignProc(env.aligns, v))
                Issue(env, where, path, "alignment procedure '" + v + "' not found");
            break;
        }
        case ParamType::NominalUid: {
            // Scoped to the step's "alignment", the way PositionUid is scoped to
            // its "device". An unresolvable procedure is already reported by the
            // AlignUid row above, so say nothing more here — one broken pick
            // must not produce two errors.
            const AlignProc* ap = FindAlignProc(env.aligns, alignment);
            if (ap && !FindAlignNominal(*ap, v))
                Issue(env, where, path, "nominal '" + v + "' not found on alignment '" +
                                      ap->name + "'");
            break;
        }
        case ParamType::VisionUid: {
            // Resolved by uid OR name, like AlignUid. Existence only: whether
            // HALCON can actually RUN the model is a run-time question, and
            // failing validation on a licence would make an unlicensed machine
            // unable to even open a recipe that mentions vision.
            if (!FindVisionModel(env.visionModels, v))
                Issue(env, where, path, "detection model '" + v + "' not found");
            break;
        }
        case ParamType::GripUid: {
            // Uid or name, existence only - like VisionUid, and for a related
            // reason: whether the profile's GRIPPER is registered is a run-time
            // question about machine state, and a recipe must stay openable on a
            // machine whose registration has not been taught yet.
            if (!FindGripProfile(env.grips, v))
                Issue(env, where, path, "grip profile '" + v + "' not found");
            break;
        }
        case ParamType::TrayUid: {
            // Uid or name, existence only: whether the tray's corners are taught
            // is checked when the step runs (TrayValidate), so a recipe written
            // before the tray was taught still opens.
            if (!FindWaffleTray(env.trays, v))
                Issue(env, where, path, "waffle tray '" + v + "' not found");
            break;
        }
        case ParamType::MapName: {
            const auto it = env.maps.find(device);
            bool found = false;
            if (it != env.maps.end()) {
                for (const MotionMap& m : it->second) {
                    if (m.name == v) { found = true; break; }
                }
            }
            if (!found)
                Issue(env, where, path, "map '" + v + "' not found for device '" + device + "'");
            break;
        }
        case ParamType::ScriptFile: {
            if (!std::ifstream(v).is_open())
                Issue(env, where, path, "script file '" + v + "' not found");
            break;
        }
        case ParamType::PinName: {
            const int idx = IoIdx(env.hw, ioDevice);
            if (idx >= 0) {
                const IIoDevice* dev = env.hw.IoDevice(static_cast<std::size_t>(idx));
                bool ok = false;
                if (dev) {   // resolve against the declared pin direction
                    if (p.pinDir != PinDir::Output) ok = dev->InputPinByName(v) >= 0;
                    if (!ok && p.pinDir != PinDir::Input) ok = dev->OutputPinByName(v) >= 0;
                }
                if (dev && !ok) {
                    const char* dir = p.pinDir == PinDir::Input    ? "input "
                                      : p.pinDir == PinDir::Output ? "output "
                                                                   : "";
                    Issue(env, where, path, std::string(dir) + "pin '" + v +
                                          "' unknown on device '" + ioDevice + "'");
                }
            }
            break;
        }
        default:
            break;
        }
    }
    if (env.requireConnected) {
        const char* what = nullptr;
        ConnState st = ConnState::Connected;
        switch (spec.needs) {
        case CommandSpec::Needs::Camera:   what = "camera";   st = env.hw.CameraState();   break;
        case CommandSpec::Needs::Keithley: what = "Keithley"; st = env.hw.KeithleyState(); break;
        case CommandSpec::Needs::Cld:      what = "CLD101x";  st = env.hw.CldState();      break;
        case CommandSpec::Needs::Psu: {
            // The step's own supply ("device", "" = the first). A name that
            // resolves to nothing is already reported above.
            const int pi = env.hw.PsuIndex(device);
            what = "PSU";
            st   = pi >= 0 ? env.hw.PsuState(static_cast<std::size_t>(pi)) : ConnState::Disconnected;
            if (pi < 0 && !device.empty()) what = nullptr;
            break;
        }
        case CommandSpec::Needs::Light:    what = "lighting"; st = env.hw.LightState();    break;
        case CommandSpec::Needs::None:     break;
        }
        if (what && st != ConnState::Connected)
            Issue(env, where, path, std::string(what) + " is not connected");
    }
}

void ValidateSteps(const ValidateEnv& env, const std::vector<RecipeStep>& steps,
                   const std::string& pathPrefix,
                   const std::vector<std::size_t>& basePath, bool elseBranch,
                   int depth)
{
    if (depth > kMaxRecipeDepth) {
        Issue(env, pathPrefix.empty() ? "(root)" : pathPrefix, basePath,
              "nesting exceeds depth " + std::to_string(kMaxRecipeDepth));
        return;
    }
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const RecipeStep& s = steps[i];
        const std::string where =
            (pathPrefix.empty() ? "" : pathPrefix + ".") + std::to_string(i + 1) +
            " (" + s.type + ")";
        std::vector<std::size_t> path = basePath;
        path.push_back(elseBranch ? kElseBranchBase + i : i);
        if (!s.enabled) continue;

        if (s.type == "loop") {
            const bool hasCount = s.params.contains("count") && s.params["count"].is_number();
            const bool hasWhile = s.params.contains("while") && s.params["while"].is_object();
            if (hasCount == hasWhile) {
                Issue(env, where, path, "loop needs exactly one of \"count\" or \"while\"");
            } else if (hasCount && s.params["count"].get<double>() < 1) {
                Issue(env, where, path, "loop count must be >= 1");
            } else if (hasWhile) {
                const std::string e = CheckConditionShape(s.params["while"]);
                if (!e.empty()) Issue(env, where, path, e);
            }
            if (s.params.contains("index_ctx")) {
                if (!s.params["index_ctx"].is_string())
                    Issue(env, where, path, "loop index_ctx must be a ctx key (string)");
                else if (hasWhile)
                    Issue(env, where, path, "loop index_ctx only counts a \"count\" loop");
            }
            if (s.children.empty()) Issue(env, where, path, "loop has no children");
            ValidateSteps(env, s.children, where, path, false, depth + 1);
        } else if (s.type == "if") {
            const std::string e = CheckConditionShape(s.params);
            if (!e.empty()) Issue(env, where, path, e);
            if (s.children.empty() && s.elseChildren.empty())
                Issue(env, where, path, "if has no children");
            ValidateSteps(env, s.children, where, path, false, depth + 1);
            ValidateSteps(env, s.elseChildren, where, path, true, depth + 1);
        } else if (s.type == "group") {
            ValidateSteps(env, s.children, where, path, false, depth + 1);
        } else if (s.type == "parallel") {
            // The children are ordinary leaves and get the ordinary checks
            // (params, references, Connected) below; what is checked HERE is
            // what makes them safe to run at once. Every refusal is one that
            // would otherwise surface with two stages already moving.
            if (s.children.empty()) Issue(env, where, path, "parallel has no children");
            ValidateSteps(env, s.children, where, path, false, depth + 1);
            for (std::size_t k = 0; k < s.children.size(); ++k) {
                const RecipeStep& c = s.children[k];
                if (!c.enabled) continue;
                const std::string cw = where + "." + std::to_string(k + 1) + " (" + c.type + ")";
                std::vector<std::size_t> cp = path;
                cp.push_back(k);
                if (IsStructuralStep(c.type)) {
                    Issue(env, cw, cp, "a parallel block holds moves only, not " + c.type);
                    continue;
                }
                const CommandSpec* spec = env.reg.Find(c.type);
                if (!spec) continue;   // reported by the ordinary pass
                if (spec->parallelBy.empty()) {
                    Issue(env, cw, cp, c.type + " cannot run in a parallel block");
                    continue;
                }
                if (c.params.value(kTeachParamName, false))
                    Issue(env, cw, cp, "teach cannot run in a parallel block - the "
                                       "operator can only stand at one stage");
                // Two children on one device are fine: they form one lane and
                // run in order (StepRunner::ExecParallel).
            }
        } else if (s.type == "sub_recipe") {
            const std::string uid = s.params.value("uid", std::string());
            if (uid.empty()) {
                Issue(env, where, path, "sub_recipe needs a \"uid\" param");
            } else if (!FindByUid(env.library, uid)) {
                Issue(env, where, path, "sub-recipe uid '" + uid + "' not found");
            }
            // Its steps are validated when the referenced recipe is validated /
            // after inlining at run start; cycles are caught by InlineSubRecipes.
        } else if (s.type == "stop_recipe") {
            // params: optional ok(Bool)/message(String) — nothing to check
        } else {
            const CommandSpec* spec = env.reg.Find(s.type);
            if (!spec) {
                Issue(env, where, path, "unknown command '" + s.type + "'");
                continue;
            }
            nlohmann::json merged;
            const std::string e = ValidateParams(*spec, s.params, &merged);
            if (!e.empty()) {
                Issue(env, where, path, e);
                continue;
            }
            CheckReferences(env, where, path, *spec, merged);
            if (s.type == "wait_until") {
                // Its flat params form a condition — check the shape now so a
                // typo fails at load, not mid-run.
                nlohmann::json cond = nlohmann::json::object();
                if (!merged.value("topic", std::string()).empty()) cond["topic"] = merged["topic"];
                if (!merged.value("key", std::string()).empty())   cond["key"]   = merged["key"];
                cond["op"] = merged["op"];
                if (merged.contains("value")) cond["value"] = merged["value"];
                const std::string ce = CheckConditionShape(cond);
                if (!ce.empty()) Issue(env, where, path, ce);
            }
        }
    }
}

} // namespace

std::vector<RecipeIssue> ValidateRecipe(const Recipe& r, const CommandRegistry& reg,
                                        const HardwareManager& hw,
                                        const SavedPosMap& positions,
                                        const MotionMapSet& maps,
                                        const std::vector<AlignProc>& aligns,
                                        const std::vector<VisionModel>& visionModels,
                                        const std::vector<GripProfile>& grips,
                                        const std::vector<WaffleTray>& trays,
                                        const std::vector<Recipe>& library,
                                        bool requireConnected)
{
    std::vector<RecipeIssue> issues;
    ValidateEnv env{reg, hw, positions, maps, aligns, visionModels, grips, trays, library,
                    requireConnected, &issues};
    if (r.steps.empty()) issues.push_back({"(root)", "recipe has no steps", {}});
    ValidateSteps(env, r.steps, "", {}, false, 1);
    return issues;
}

} // namespace firedog
