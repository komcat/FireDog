// PorCompare.cpp : see PorCompare.h.
#include "por/PorCompare.h"

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "command/ExecutionContext.h"   // IsSystemScript
#include "command/StepSummary.h"
#include "core/LightCallout.h"          // ChannelInLight

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <system_error>

namespace firedog {

const char* PorStatusName(PorStatus s)
{
    switch (s) {
    case PorStatus::Same:       return "same";
    case PorStatus::Measured:   return "measured";
    case PorStatus::Unknown:    return "unknown command";
    case PorStatus::Changed:    return "changed";
    case PorStatus::OnlyTarget: return "only on target";
    case PorStatus::OnlyMaster: return "only on master";
    case PorStatus::Ambiguous:  return "ambiguous name";
    case PorStatus::Missing:    return "missing reference";
    }
    return "?";
}

// ---- Sequence alignment ------------------------------------------------------

std::vector<std::pair<int, int>> PorAlign(const std::vector<std::string>& a,
                                          const std::vector<std::string>& b,
                                          const std::vector<std::string>& a2,
                                          const std::vector<std::string>& b2)
{
    const int n = static_cast<int>(a.size());
    const int m = static_cast<int>(b.size());
    // L[i][j] = LCS length of a[i..] and b[j..].
    std::vector<std::vector<int>> L(n + 1, std::vector<int>(m + 1, 0));
    for (int i = n - 1; i >= 0; --i)
        for (int j = m - 1; j >= 0; --j)
            L[i][j] = a[i] == b[j] ? L[i + 1][j + 1] + 1 : std::max(L[i + 1][j], L[i][j + 1]);

    std::vector<std::pair<int, int>> anchors;
    for (int i = 0, j = 0; i < n && j < m;) {
        if (a[i] == b[j])                { anchors.emplace_back(i, j); ++i; ++j; }
        else if (L[i + 1][j] >= L[i][j + 1]) ++i;
        else                                 ++j;
    }
    anchors.emplace_back(n, m);   // sentinel: closes the last gap

    const bool second = a2.size() == a.size() && b2.size() == b.size() && !a2.empty();
    std::vector<std::pair<int, int>> out;
    int ia = 0, ja = 0;
    for (const auto& [ib, jb] : anchors) {
        // The gap a[ia, ib) x b[ja, jb): pair by the secondary key, in order.
        std::vector<int> pairOf(static_cast<std::size_t>(ib - ia), -1);
        if (second) {
            int lastJ = ja;
            for (int i = ia; i < ib; ++i) {
                for (int j = lastJ; j < jb; ++j) {
                    if (a2[i] == b2[j]) { pairOf[i - ia] = j; lastJ = j + 1; break; }
                }
            }
        }
        int jc = ja;
        for (int i = ia; i < ib; ++i) {
            const int pj = pairOf[i - ia];
            if (pj < 0) { out.emplace_back(i, -1); continue; }
            while (jc < pj) out.emplace_back(-1, jc++);
            out.emplace_back(i, pj);
            jc = pj + 1;
        }
        while (jc < jb) out.emplace_back(-1, jc++);
        if (ib < n && jb < m) out.emplace_back(ib, jb);   // the anchor itself
        ia = ib + 1;
        ja = jb + 1;
    }
    return out;
}

std::vector<std::pair<int, int>> PorPairSteps(const std::vector<RecipeStep>& a,
                                              const std::vector<RecipeStep>& b)
{
    std::vector<std::string> ka, kb, ta, tb;
    for (const RecipeStep& s : a) { ka.push_back(s.type + '\x1f' + s.nickname); ta.push_back(s.type); }
    for (const RecipeStep& s : b) { kb.push_back(s.type + '\x1f' + s.nickname); tb.push_back(s.type); }
    return PorAlign(ka, kb, ta, tb);
}

void PorCollectTwins(const std::vector<RecipeStep>& a, const std::vector<RecipeStep>& b,
                     PorStepTwins* out)
{
    for (const auto& [i, j] : PorPairSteps(a, b)) {
        if (i < 0 || j < 0) continue;
        (*out)[&a[i]] = &b[j];
        PorCollectTwins(a[i].children, b[j].children, out);
        PorCollectTwins(a[i].elseChildren, b[j].elseChildren, out);
    }
}

const SavedPos* PorFindPosByUid(const SavedPosMap& m, const std::string& device,
                                const std::string& uid)
{
    const auto it = m.find(device);
    if (it == m.end()) return nullptr;
    for (const SavedPos& sp : it->second)
        if (sp.uid == uid) return &sp;
    return nullptr;
}

const SavedPos* PorFindPosByName(const SavedPosMap& m, const std::string& device,
                                 const std::string& name, int* count)
{
    if (count) *count = 0;
    const auto it = m.find(device);
    if (it == m.end()) return nullptr;
    const SavedPos* first = nullptr;
    for (const SavedPos& sp : it->second) {
        if (name != sp.name) continue;
        if (!first) first = &sp;
        if (count) ++*count;
    }
    return first;
}

namespace {

namespace fs = std::filesystem;

std::string Num(double v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%g", v);
    return b;
}

std::string JsonText(const nlohmann::json& v)
{
    if (v.is_null())    return "";
    if (v.is_string())  return v.get<std::string>();
    if (v.is_boolean()) return v.get<bool>() ? "on" : "off";
    if (v.is_number())  return Num(v.get<double>());
    return v.dump();
}

bool JsonEqual(const nlohmann::json& a, const nlohmann::json& b)
{
    if (a.is_number() && b.is_number()) {
        const double x = a.get<double>(), y = b.get<double>();
        return std::fabs(x - y) <= 1e-9 * std::max({1.0, std::fabs(x), std::fabs(y)});
    }
    if (a.is_object() && b.is_object()) {
        if (a.size() != b.size()) return false;
        for (auto it = a.begin(); it != a.end(); ++it) {
            const auto jt = b.find(it.key());
            if (jt == b.end() || !JsonEqual(it.value(), *jt)) return false;
        }
        return true;
    }
    if (a.is_array() && b.is_array()) {
        if (a.size() != b.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (!JsonEqual(a[i], b[i])) return false;
        return true;
    }
    return a == b;
}

PorStatus Max(PorStatus a, PorStatus b) { return static_cast<int>(a) >= static_cast<int>(b) ? a : b; }

// A param's own presence difference reads as a CHANGED row: the step is there
// on both sides, one of them just says more.
PorStatus RowShare(PorStatus s)
{
    return (s == PorStatus::OnlyMaster || s == PorStatus::OnlyTarget) ? PorStatus::Changed : s;
}

void Finish(PorRow& r)
{
    r.worst = r.status;
    for (const PorRow& c : r.children) r.worst = Max(r.worst, c.worst);
}

bool IsPrivateName(const std::string& n)
{
    return n.rfind("vdet_", 0) == 0 || n.rfind("algn_", 0) == 0 || n.rfind("grip_", 0) == 0;
}

bool IsReferenceType(ParamType t)
{
    switch (t) {
    case ParamType::DeviceMotion: case ParamType::DeviceHexapod: case ParamType::DeviceIo:
    case ParamType::DevicePsu:    case ParamType::Slide:         case ParamType::LightName:
    case ParamType::PositionUid:  case ParamType::AlignUid:      case ParamType::NominalUid:
    case ParamType::VisionUid:    case ParamType::GripUid:       case ParamType::TrayUid:
    case ParamType::MapName:      case ParamType::PinName:       case ParamType::ScriptFile:
    case ParamType::ImageFile:
        return true;
    default:
        return false;
    }
}

// One reference, resolved on one side to the NAME the two sides are compared by.
struct Ref {
    std::string text;              // what the row shows
    std::string key;               // what the two sides are compared by
    bool        ok        = true;  // resolves on this side
    bool        ambiguous = false; // the name it resolves to is used twice
    bool        priv      = false; // a private object: matched by its owner, not its name
};

struct Ctx {
    const PorSide&         m;
    const PorSide&         t;
    const CommandRegistry& reg;
    std::string            shipped;   // the tool's own config/scripts, "" = none
    StepSummaryDocs        dm, dt;
    std::map<std::string, PorStatus> recipeWorst;   // by recipe name
    std::set<std::string>            recipeAmbiguous;
};

Ref Resolve(const Ctx& c, const PorSide& s, const ParamSpec& p, const nlohmann::json& merged,
            const nlohmann::json& v)
{
    Ref r;
    if (!v.is_string()) { r.text = JsonText(v); r.key = r.text; return r; }
    const std::string val = v.get<std::string>();
    r.text = val;
    r.key  = val;
    if (val.empty() && p.type != ParamType::DevicePsu) return r;   // optional and unset

    const std::string device = merged.value("device", std::string());
    const auto miss = [&r, &val](const std::string& why) {
        r.ok   = false;
        r.text = val + "  (" + why + ")";
        r.key  = "?" + val;
    };

    switch (p.type) {
    case ParamType::PositionUid: {
        const SavedPos* sp = PorFindPosByUid(s.positions, device, val);
        if (!sp) { miss("no such position on " + device); break; }
        int n = 0;
        PorFindPosByName(s.positions, device, sp->name, &n);
        r.text      = sp->name;
        r.key       = device + "/" + sp->name;
        r.ambiguous = n > 1;
        if (sp->hidden && IsPrivateName(sp->name)) { r.priv = true; r.key = "private position"; }
        break;
    }
    case ParamType::AlignUid:
        if (const AlignProc* ap = FindAlignProc(s.aligns, val)) r.text = r.key = ap->name;
        else miss("no such alignment");
        break;
    case ParamType::NominalUid: {
        const AlignProc* ap = FindAlignProc(s.aligns, merged.value("alignment", std::string()));
        const AlignNominal* nm = ap ? FindAlignNominal(*ap, val) : nullptr;
        if (nm) r.text = r.key = nm->name;
        else    miss("no such nominal");
        break;
    }
    case ParamType::VisionUid:
        if (const VisionModel* vm = FindVisionModel(s.vision, val)) {
            r.text = r.key = vm->name;
            if (IsPrivateName(vm->name)) { r.priv = true; r.key = "private model"; }
        } else {
            miss("no such detection model");
        }
        break;
    case ParamType::GripUid:
        if (const GripProfile* g = FindGripProfile(s.grips, val)) r.text = r.key = g->name;
        else miss("no such grip profile");
        break;
    case ParamType::TrayUid:
        if (const WaffleTray* tr = FindWaffleTray(s.trays, val)) r.text = r.key = tr->name;
        else miss("no such tray");
        break;
    case ParamType::MapName: {
        bool found = false;
        const auto it = s.maps.find(device);
        if (it != s.maps.end())
            for (const MotionMap& mm : it->second) found = found || mm.name == val;
        if (!found) miss("no such map on " + device);
        break;
    }
    case ParamType::DeviceMotion:
    case ParamType::DeviceHexapod: {
        if (!s.hasHardware) break;
        const auto it = s.hw.motion.find(val);
        if (it == s.hw.motion.end()) miss("no such motion device");
        else if (p.type == ParamType::DeviceHexapod && it->second != "PI") miss("not a PI hexapod");
        break;
    }
    case ParamType::DeviceIo:
        if (s.hasHardware && !s.hw.io.count(val)) miss("no such IO device");
        break;
    case ParamType::DevicePsu:
        if (val.empty()) {
            r.text = r.key = "(the first PSU)";
            if (s.hasHardware && s.hw.psus.empty()) { r.ok = false; r.text += "  (no PSU)"; }
        } else if (s.hasHardware &&
                   std::find(s.hw.psus.begin(), s.hw.psus.end(), val) == s.hw.psus.end()) {
            miss("no such PSU");
        }
        break;
    case ParamType::Slide:
        if (s.hasHardware && !s.hw.slides.count(val)) miss("no such slide");
        break;
    case ParamType::LightName: {
        if (!s.hasHardware) break;
        bool found = false;
        for (const LightChannelConfig& ch : s.hw.lightChannels)
            found = found || ChannelInLight(ch.group, ch.name, val);
        if (!found) miss(s.hw.hasLight ? "no such light" : "no lighting controller");
        break;
    }
    case ParamType::PinName: {
        // The pin follows "io_device" when the command names one (the needle
        // family), else "device" - CheckReferences' rule.
        const std::string ioDev = merged.value("io_device", device);
        const auto it = s.hw.io.find(ioDev);
        if (!s.hasHardware || it == s.hw.io.end()) break;   // the device row says so
        bool ok = false;
        if (p.pinDir != PinDir::Output) ok = it->second.inputs.count(val) > 0;
        if (!ok && p.pinDir != PinDir::Input) ok = it->second.outputs.count(val) > 0;
        if (!ok) miss("no such pin on " + ioDev);
        break;
    }
    case ParamType::ScriptFile: {
        const std::string file = fs::path(val).filename().string();
        r.text = r.key = file;
        if (IsSystemScript(file)) { r.text += "  (standard)"; break; }
        std::error_code ec;
        const bool shipped = !c.shipped.empty() && fs::is_regular_file(c.shipped + "/" + file, ec);
        if (!s.scripts.count(file) && !shipped) {
            r.ok   = false;
            r.text = file + "  (no such script)";
            r.key  = "?" + file;
        }
        break;
    }
    case ParamType::ImageFile:
        if (!s.promptImages.count(val)) miss("no such picture");
        break;
    default:
        break;
    }
    return r;
}

std::string PointText(const nlohmann::json& v)
{
    if (!v.is_object()) return JsonText(v);
    std::string t;
    for (const char* ax : {"X", "Y", "Z"}) {
        if (!v.contains(ax) || !v[ax].is_number()) continue;
        if (!t.empty()) t += ", ";
        t += std::string(ax) + " " + Num(v[ax].get<double>());
    }
    return t;
}

double PointDelta(const nlohmann::json& a, const nlohmann::json& b)
{
    double s = 0.0;
    for (const char* ax : {"X", "Y", "Z"}) {
        const double x = a.is_object() && a.contains(ax) && a[ax].is_number() ? a[ax].get<double>() : 0.0;
        const double y = b.is_object() && b.contains(ax) && b[ax].is_number() ? b[ax].get<double>() : 0.0;
        s += (x - y) * (x - y);
    }
    return std::sqrt(s);
}

std::string DesignText(const ParamSpec* p, const nlohmann::json& v)
{
    if (p && p->type == ParamType::IntChoice && v.is_number()) {
        const int idx = static_cast<int>(v.get<double>()) - static_cast<int>(p->minVal);
        if (idx >= 0 && idx < static_cast<int>(p->options.size())) return p->options[idx];
    }
    if (p && p->type == ParamType::DurationMs && v.is_number())
        return FormatDurationMs(std::llround(v.get<double>()));
    return JsonText(v);
}

// One param of a paired step. `p` null = a key the schema does not declare.
PorParam CompareParam(const Ctx& c, const std::string& cmd, const std::string& name,
                      const ParamSpec* p, const nlohmann::json& mm, const nlohmann::json& tm)
{
    PorParam out;
    out.name      = name;
    out.hasMaster = mm.contains(name);
    out.hasTarget = tm.contains(name);
    const nlohmann::json nul;
    const nlohmann::json& mv = out.hasMaster ? mm[name] : nul;
    const nlohmann::json& tv = out.hasTarget ? tm[name] : nul;

    // detect_object's model is the step's PRIVATE detector (a String, not a
    // VisionUid): taught on each machine, so only its existence is judged.
    if (cmd == "detect_object" && name == "model") {
        out.cls = PorParamClass::Measured;
        const auto exists = [](const PorSide& s, const nlohmann::json& v) {
            return v.is_string() && !v.get<std::string>().empty() &&
                   FindVisionModel(s.vision, v.get<std::string>()) != nullptr;
        };
        const bool okM = exists(c.m, mv), okT = exists(c.t, tv);
        out.master = okM ? "private model" : (out.hasMaster ? "(none)" : "");
        out.target = okT ? "private model" : (out.hasTarget ? "(none)" : "");
        if (okM && !okT) { out.status = PorStatus::Missing; out.note = "not taught on the target"; }
        else             { out.note = "taught on each machine"; }
        return out;
    }

    if (!out.hasMaster || !out.hasTarget) {
        out.cls    = p ? (IsReferenceType(p->type) ? PorParamClass::Reference : PorParamClass::Design)
                       : PorParamClass::Raw;
        out.master = out.hasMaster ? DesignText(p, mv) : "";
        out.target = out.hasTarget ? DesignText(p, tv) : "";
        out.status = out.hasMaster ? PorStatus::OnlyMaster : PorStatus::OnlyTarget;
        return out;
    }

    if (p && p->type == ParamType::PointXyz) {
        out.cls    = PorParamClass::Measured;
        out.master = PointText(mv);
        out.target = PointText(tv);
        if (!JsonEqual(mv, tv)) {
            out.status = PorStatus::Measured;
            out.note   = "Δ " + Num(PointDelta(mv, tv)) + " mm, taught on each machine";
        }
        return out;
    }

    if (p && IsReferenceType(p->type)) {
        out.cls = PorParamClass::Reference;
        const Ref rm = Resolve(c, c.m, *p, mm, mv);
        const Ref rt = Resolve(c, c.t, *p, tm, tv);
        out.master = rm.text;
        out.target = rt.text;
        if (!rt.ok || !rm.ok) {
            out.status = PorStatus::Missing;
            out.note   = !rt.ok ? (!rm.ok ? "missing on both" : "missing on the target")
                                : "missing on the master";
        } else if (rm.priv && rt.priv) {
            out.note = "private, matched by its owner";
        } else if (rm.key != rt.key) {
            out.status = PorStatus::Changed;
        } else if (rm.ambiguous || rt.ambiguous) {
            out.status = PorStatus::Ambiguous;
            out.note   = "the name is used twice on " +
                         std::string(rm.ambiguous ? (rt.ambiguous ? "both" : "the master")
                                                  : "the target");
        }
        return out;
    }

    out.cls    = p ? PorParamClass::Design : PorParamClass::Raw;
    out.master = DesignText(p, mv);
    out.target = DesignText(p, tv);
    if (!JsonEqual(mv, tv)) out.status = PorStatus::Changed;
    return out;
}

std::string OnErrorText(const RecipeStep& s)
{
    switch (s.onError) {
    case RecipeStep::OnError::Abort:    return "abort";
    case RecipeStep::OnError::Continue: return "continue";
    case RecipeStep::OnError::Retry:    return "retry " + std::to_string(s.retryCount);
    }
    return "?";
}

std::string StepLabel(const RecipeStep& s, const StepSummaryDocs& d)
{
    return s.nickname.empty() ? StepSummary(s, d) : s.nickname;
}

const Recipe* RecipeByUidIn(const PorSide& s, const std::string& uid)
{
    return RecipeByUid(s.recipes, uid);
}

std::vector<PorRow> CompareSteps(Ctx& c, const std::vector<RecipeStep>* a,
                                 const std::vector<RecipeStep>* b);

// A step present on ONE side, with its subtree (all one-sided too).
PorRow OneSidedStep(Ctx& c, const RecipeStep& s, bool master)
{
    PorRow r;
    r.kind   = PorRowKind::Step;
    r.status = master ? PorStatus::OnlyMaster : PorStatus::OnlyTarget;
    (master ? r.master : r.target) = StepLabel(s, master ? c.dm : c.dt);
    (master ? r.masterOff : r.targetOff) = !s.enabled;
    r.children = master ? CompareSteps(c, &s.children, nullptr) : CompareSteps(c, nullptr, &s.children);
    if (!s.elseChildren.empty()) {
        PorRow e;
        e.kind = PorRowKind::Else;
        e.status = r.status;
        (master ? e.master : e.target) = "else";
        e.children = master ? CompareSteps(c, &s.elseChildren, nullptr)
                            : CompareSteps(c, nullptr, &s.elseChildren);
        Finish(e);
        r.children.push_back(std::move(e));
    }
    Finish(r);
    return r;
}

PorRow PairedStep(Ctx& c, const RecipeStep& ms, const RecipeStep& ts)
{
    PorRow r;
    r.kind      = PorRowKind::Step;
    r.master    = StepLabel(ms, c.dm);
    r.target    = StepLabel(ts, c.dt);
    r.masterOff = !ms.enabled;
    r.targetOff = !ts.enabled;

    const auto field = [&r](const char* name, const std::string& a, const std::string& b) {
        PorParam p;
        p.name = name; p.master = a; p.target = b; p.hasMaster = p.hasTarget = true;
        if (a != b) p.status = PorStatus::Changed;
        r.params.push_back(std::move(p));
    };
    field("nickname", ms.nickname, ts.nickname);
    field("enabled", ms.enabled ? "on" : "off", ts.enabled ? "on" : "off");
    field("on_error", OnErrorText(ms), OnErrorText(ts));

    const CommandSpec* spec = c.reg.Find(ms.type);
    std::set<std::string> declared;
    if (spec) {
        const nlohmann::json mm = EffectiveParams(*spec, ms.params);
        const nlohmann::json tm = EffectiveParams(*spec, ts.params);
        for (const ParamSpec& p : spec->params) {
            declared.insert(p.name);
            r.params.push_back(CompareParam(c, ms.type, p.name, &p, mm, tm));
        }
    } else if (ms.type == "sub_recipe") {
        // The one structural reference: the recipe it calls, by NAME.
        declared.insert("uid");
        PorParam p;
        p.name = "recipe";
        p.cls  = PorParamClass::Reference;
        const Recipe* rm = RecipeByUidIn(c.m, ms.params.value("uid", std::string()));
        const Recipe* rt = RecipeByUidIn(c.t, ts.params.value("uid", std::string()));
        p.hasMaster = p.hasTarget = true;
        p.master = rm ? rm->name : ms.params.value("uid", std::string()) + "  (no such recipe)";
        p.target = rt ? rt->name : ts.params.value("uid", std::string()) + "  (no such recipe)";
        if (!rm || !rt)               { p.status = PorStatus::Missing; p.note = !rt ? "missing on the target" : "missing on the master"; }
        else if (rm->name != rt->name) p.status = PorStatus::Changed;
        r.params.push_back(std::move(p));
    }
    // Everything the schema does not declare (a structural step's params, an
    // unknown command's, a stray key): compared raw.
    std::set<std::string> extra;
    for (auto it = ms.params.begin(); it != ms.params.end(); ++it)
        if (!declared.count(it.key())) extra.insert(it.key());
    for (auto it = ts.params.begin(); it != ts.params.end(); ++it)
        if (!declared.count(it.key())) extra.insert(it.key());
    for (const std::string& k : extra)
        r.params.push_back(CompareParam(c, ms.type, k, nullptr, ms.params, ts.params));

    for (const PorParam& p : r.params) r.status = Max(r.status, RowShare(p.status));
    if (!spec && !IsStructuralStep(ms.type)) r.status = Max(r.status, PorStatus::Unknown);

    r.children = CompareSteps(c, &ms.children, &ts.children);
    if (!ms.elseChildren.empty() || !ts.elseChildren.empty()) {
        PorRow e;
        e.kind     = PorRowKind::Else;
        e.master   = "else";
        e.target   = "else";
        e.children = CompareSteps(c, &ms.elseChildren, &ts.elseChildren);
        Finish(e);
        r.children.push_back(std::move(e));
    }
    Finish(r);
    return r;
}

std::vector<PorRow> CompareSteps(Ctx& c, const std::vector<RecipeStep>* a,
                                 const std::vector<RecipeStep>* b)
{
    static const std::vector<RecipeStep> kNone;
    const std::vector<RecipeStep>& A = a ? *a : kNone;
    const std::vector<RecipeStep>& B = b ? *b : kNone;
    std::vector<PorRow> rows;
    for (const auto& [i, j] : PorPairSteps(A, B)) {
        if (i >= 0 && j >= 0) rows.push_back(PairedStep(c, A[i], B[j]));
        else if (i >= 0)      rows.push_back(OneSidedStep(c, A[i], true));
        else                  rows.push_back(OneSidedStep(c, B[j], false));
    }
    return rows;
}

// ---- Recipes -----------------------------------------------------------------

std::map<std::string, int> NameCounts(const std::vector<Recipe>& v)
{
    std::map<std::string, int> n;
    for (const Recipe& r : v) ++n[r.name];
    return n;
}

std::string TagsText(const Recipe& r)
{
    std::string t;
    for (const std::string& g : r.tags) t += (t.empty() ? "" : ", ") + g;
    return t;
}

PorRow OneSidedRecipe(Ctx& c, const Recipe& r, bool master, PorStatus st)
{
    PorRow row;
    row.kind   = PorRowKind::Recipe;
    row.status = st;
    row.recipe = r.name;
    (master ? row.master : row.target) = r.name;
    row.children = master ? CompareSteps(c, &r.steps, nullptr) : CompareSteps(c, nullptr, &r.steps);
    Finish(row);
    return row;
}

PorRow PairedRecipe(Ctx& c, const Recipe& m, const Recipe& t)
{
    PorRow row;
    row.kind   = PorRowKind::Recipe;
    row.recipe = m.name;
    row.master = m.name;
    row.target = t.name;
    const auto field = [&row](const char* name, const std::string& a, const std::string& b,
                              PorParamClass cls) {
        PorParam p;
        p.name = name; p.master = a; p.target = b; p.cls = cls;
        p.hasMaster = p.hasTarget = true;
        if (a != b) p.status = PorStatus::Changed;
        row.params.push_back(std::move(p));
    };
    field("description", m.description, t.description, PorParamClass::Design);
    field("hidden", m.hidden ? "on" : "off", t.hidden ? "on" : "off", PorParamClass::Design);
    field("tags", TagsText(m), TagsText(t), PorParamClass::Design);
    {
        PorParam p;
        p.name = "on_fail";
        p.cls  = PorParamClass::Reference;
        p.hasMaster = p.hasTarget = true;
        const Recipe* fm = m.onFailUid.empty() ? nullptr : RecipeByUidIn(c.m, m.onFailUid);
        const Recipe* ft = t.onFailUid.empty() ? nullptr : RecipeByUidIn(c.t, t.onFailUid);
        p.master = m.onFailUid.empty() ? "" : (fm ? fm->name : m.onFailUid + "  (no such recipe)");
        p.target = t.onFailUid.empty() ? "" : (ft ? ft->name : t.onFailUid + "  (no such recipe)");
        if ((!m.onFailUid.empty() && !fm) || (!t.onFailUid.empty() && !ft)) {
            p.status = PorStatus::Missing;
            p.note   = (!t.onFailUid.empty() && !ft) ? "missing on the target" : "missing on the master";
        } else if (p.master != p.target) {
            p.status = PorStatus::Changed;
        }
        row.params.push_back(std::move(p));
    }
    for (const PorParam& p : row.params) row.status = Max(row.status, p.status);
    row.children = CompareSteps(c, &m.steps, &t.steps);
    Finish(row);
    return row;
}

std::vector<PorRow> CompareRecipes(Ctx& c)
{
    const std::map<std::string, int> nm = NameCounts(c.m.recipes);
    const std::map<std::string, int> nt = NameCounts(c.t.recipes);
    const auto count = [](const std::map<std::string, int>& n, const std::string& k) {
        const auto it = n.find(k);
        return it == n.end() ? 0 : it->second;
    };
    const std::map<std::string, std::string> wm = WorkingRecipes(c.m.programs, c.m.recipes);
    const std::map<std::string, std::string> wt = WorkingRecipes(c.t.programs, c.t.recipes);

    std::vector<PorRow> rows;
    std::set<std::string> paired;
    for (const Recipe& m : c.m.recipes) {
        PorRow row;
        if (count(nm, m.name) > 1 || count(nt, m.name) > 1) {
            c.recipeAmbiguous.insert(m.name);
            row = OneSidedRecipe(c, m, true, PorStatus::Ambiguous);
        } else if (count(nt, m.name) == 1) {
            const Recipe* t = nullptr;
            for (const Recipe& r : c.t.recipes) if (r.name == m.name) { t = &r; break; }
            row = PairedRecipe(c, m, *t);
            paired.insert(m.name);
            row.working = wt.count(t->uid) > 0;
        } else {
            row = OneSidedRecipe(c, m, true, PorStatus::OnlyMaster);
        }
        row.working = row.working || wm.count(m.uid) > 0;
        c.recipeWorst[m.name] = Max(c.recipeWorst[m.name], row.worst);
        rows.push_back(std::move(row));
    }
    for (const Recipe& t : c.t.recipes) {
        if (paired.count(t.name)) continue;
        const bool amb = count(nm, t.name) > 1 || count(nt, t.name) > 1;
        if (amb) c.recipeAmbiguous.insert(t.name);
        PorRow row = OneSidedRecipe(c, t, false, amb ? PorStatus::Ambiguous : PorStatus::OnlyTarget);
        row.working = wt.count(t.uid) > 0;
        c.recipeWorst[t.name] = Max(c.recipeWorst[t.name], row.worst);
        rows.push_back(std::move(row));
    }
    return rows;
}

// ---- The deck ----------------------------------------------------------------

// A node's match key and label on its own side. An unresolved recipe keeps
// its uid in the key: it pairs with nothing, which is what it is.
std::string NodeName(const PorSide& s, const ProgramNode& n, bool* resolved = nullptr)
{
    if (resolved) *resolved = true;
    if (n.IsFolder()) return n.name;
    if (const Recipe* r = RecipeByUid(s.recipes, n.recipeUid)) return r->name;
    if (resolved) *resolved = false;
    return n.recipeUid + "  (no such recipe)";
}

std::string NodeKey(const PorSide& s, const ProgramNode& n)
{
    return (n.IsFolder() ? "F:" : "R:") + NodeName(s, n);
}

std::vector<PorRow> CompareNodes(Ctx& c, const std::vector<ProgramNode>* a,
                                 const std::vector<ProgramNode>* b);

PorRow NodeRow(Ctx& c, const ProgramNode* mn, const ProgramNode* tn)
{
    PorRow r;
    const ProgramNode& any = mn ? *mn : *tn;
    r.kind = any.IsFolder() ? PorRowKind::Folder : PorRowKind::RecipeNode;
    bool okM = true, okT = true;
    if (mn) r.master = NodeName(c.m, *mn, &okM);
    if (tn) r.target = NodeName(c.t, *tn, &okT);

    if (r.kind == PorRowKind::Folder) {
        if (mn && tn) {
            const auto field = [&r](const char* name, const std::string& a, const std::string& b) {
                PorParam p;
                p.name = name; p.master = a; p.target = b; p.hasMaster = p.hasTarget = true;
                if (a != b) p.status = PorStatus::Changed;
                r.params.push_back(std::move(p));
            };
            field("loop", std::to_string(mn->loop), std::to_string(tn->loop));
            field("dry_run", mn->dryRun ? "on" : "off", tn->dryRun ? "on" : "off");
            for (const PorParam& p : r.params) r.status = Max(r.status, p.status);
        } else {
            r.status = mn ? PorStatus::OnlyMaster : PorStatus::OnlyTarget;
        }
        r.children = CompareNodes(c, mn ? &mn->children : nullptr, tn ? &tn->children : nullptr);
    } else {
        r.recipe = mn ? r.master : r.target;
        if (!okM || !okT)        r.status = PorStatus::Missing;
        else if (!mn)            r.status = PorStatus::OnlyTarget;
        else if (!tn)            r.status = PorStatus::OnlyMaster;
        else if (c.recipeAmbiguous.count(r.recipe)) r.status = PorStatus::Ambiguous;
        else {
            // The node IS the recipe it runs: its state is that recipe's compare.
            const auto it = c.recipeWorst.find(r.recipe);
            r.status = it == c.recipeWorst.end() ? PorStatus::Same : it->second;
        }
    }
    Finish(r);
    return r;
}

std::vector<PorRow> CompareNodes(Ctx& c, const std::vector<ProgramNode>* a,
                                 const std::vector<ProgramNode>* b)
{
    static const std::vector<ProgramNode> kNone;
    const std::vector<ProgramNode>& A = a ? *a : kNone;
    const std::vector<ProgramNode>& B = b ? *b : kNone;
    std::vector<std::string> ka, kb;
    for (const ProgramNode& n : A) ka.push_back(NodeKey(c.m, n));
    for (const ProgramNode& n : B) kb.push_back(NodeKey(c.t, n));
    std::vector<PorRow> rows;
    for (const auto& [i, j] : PorAlign(ka, kb))
        rows.push_back(NodeRow(c, i >= 0 ? &A[i] : nullptr, j >= 0 ? &B[j] : nullptr));
    return rows;
}

// "3  UV cure": a cycle state as the deck reads it (top-level position + name).
std::string CycleText(const PorSide& s, const ProgramSlot& slot, const std::string& uid)
{
    if (uid.empty()) return "";
    for (std::size_t i = 0; i < slot.items.size(); ++i)
        if (slot.items[i].uid == uid)
            return std::to_string(i + 1) + "  " + NodeName(s, slot.items[i]);
    return "(not a top-level node)";
}

PorRow SlotRow(Ctx& c, const ProgramSlot* ms, const ProgramSlot* ts)
{
    PorRow r;
    r.kind = PorRowKind::Slot;
    if (ms) r.master = ms->name;
    if (ts) r.target = ts->name;
    if (ms && ts) {
        for (const char* which : {"cycle_start", "cycle_end"}) {
            const bool start = std::string(which) == "cycle_start";
            PorParam p;
            p.name = which;
            p.hasMaster = p.hasTarget = true;
            p.master = CycleText(c.m, *ms, start ? ms->startNodeUid : ms->endNodeUid);
            p.target = CycleText(c.t, *ts, start ? ts->startNodeUid : ts->endNodeUid);
            if (p.master != p.target) p.status = PorStatus::Changed;
            r.status = Max(r.status, p.status);
            r.params.push_back(std::move(p));
        }
    } else {
        r.status = ms ? PorStatus::OnlyMaster : PorStatus::OnlyTarget;
    }
    r.children = CompareNodes(c, ms ? &ms->items : nullptr, ts ? &ts->items : nullptr);
    Finish(r);
    return r;
}

PorRow ProgramRow(Ctx& c, const Program* mp, const Program* tp, PorStatus oneSided)
{
    PorRow r;
    r.kind = PorRowKind::Program;
    if (mp) r.master = mp->name;
    if (tp) r.target = tp->name;
    if (mp && tp) {
        PorParam p;
        p.name = "description"; p.master = mp->description; p.target = tp->description;
        p.hasMaster = p.hasTarget = true;
        if (p.master != p.target) p.status = PorStatus::Changed;
        r.status = p.status;
        r.params.push_back(std::move(p));
    } else {
        r.status = oneSided;
    }
    std::vector<std::string> ka, kb;
    if (mp) for (const ProgramSlot& s : mp->slots) ka.push_back(s.name);
    if (tp) for (const ProgramSlot& s : tp->slots) kb.push_back(s.name);
    for (const auto& [i, j] : PorAlign(ka, kb))
        r.children.push_back(SlotRow(c, i >= 0 ? &mp->slots[i] : nullptr,
                                        j >= 0 ? &tp->slots[j] : nullptr));
    Finish(r);
    return r;
}

std::vector<PorRow> CompareDeck(Ctx& c)
{
    std::map<std::string, int> nm, nt;
    for (const Program& p : c.m.programs) ++nm[p.name];
    for (const Program& p : c.t.programs) ++nt[p.name];
    const auto ambiguous = [&](const std::string& n) {
        return (nm.count(n) && nm.at(n) > 1) || (nt.count(n) && nt.at(n) > 1);
    };
    std::vector<PorRow> rows;
    std::set<std::string> paired;
    for (const Program& mp : c.m.programs) {
        if (ambiguous(mp.name)) { rows.push_back(ProgramRow(c, &mp, nullptr, PorStatus::Ambiguous)); continue; }
        const Program* tp = nullptr;
        for (const Program& p : c.t.programs) if (p.name == mp.name) { tp = &p; break; }
        if (tp) paired.insert(mp.name);
        rows.push_back(ProgramRow(c, &mp, tp, PorStatus::OnlyMaster));
    }
    for (const Program& tp : c.t.programs) {
        if (paired.count(tp.name)) continue;
        rows.push_back(ProgramRow(c, nullptr, &tp,
                                  ambiguous(tp.name) ? PorStatus::Ambiguous : PorStatus::OnlyTarget));
    }
    return rows;
}

StepSummaryDocs DocsOf(const PorSide& s, const CommandRegistry& reg)
{
    StepSummaryDocs d;
    d.registry  = &reg;
    d.recipes   = &s.recipes;
    d.positions = &s.positions;
    d.aligns    = &s.aligns;
    d.grips     = &s.grips;
    d.trays     = &s.trays;
    return d;
}

} // namespace

PorReport ComparePor(const PorSide& master, const PorSide& target,
                     const CommandRegistry& reg, const std::string& shippedScriptsDir)
{
    Ctx c{master, target, reg, shippedScriptsDir, DocsOf(master, reg), DocsOf(target, reg), {}, {}};
    PorReport rep;
    rep.recipes = CompareRecipes(c);   // first: a deck node reads its recipe's result
    rep.deck    = CompareDeck(c);
    if (!master.hasHardware)
        rep.notes.push_back("the master has no hardware.json - device names are checked on the target only");
    if (!target.hasHardware)
        rep.notes.push_back("the target has no hardware.json - device and pin names are NOT checked");
    return rep;
}

} // namespace firedog
