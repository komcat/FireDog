// Interlock.cpp : the interlock model's evaluation and its file. See Interlock.h.
#include "core/Interlock.h"

#include "core/Log.h"
#include "core/Paths.h"
#include "core/Product.h"   // GenerateUid

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <set>
#include <system_error>

namespace firedog {

namespace {

// Is an axis being MOVED (mm or deg)? Against an exact reference - what was
// last sent to it - anything but a restatement is motion, down to the 0.1 um
// jog step (a looser test would let a fine jog creep into a blocked zone one
// press at a time). Against only a measured value, the servo's jitter is not.
constexpr double kExactEps  = 1e-6;
constexpr double kJitterTol = 0.002;
// An in-flight target further than this from the measured position: the axis
// is still on its way (the same tolerance Workflow retires targets at).
constexpr double kFlightTol = 0.002;
// A last-sent target is trusted as the reference only while the axis sits
// within this of it - otherwise something else has moved it since.
constexpr double kLastCmdTrust = 0.01;

// The shipped default, beside the exe (a relative path is the exe folder).
constexpr const char* kShippedPath = "config/interlocks.json";

std::string Fmt(double v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%.3f", v);
    return b;
}

bool Compare(double a, IlOp op, double v)
{
    switch (op) {
    case IlOp::Gt: return a > v;
    case IlOp::Ge: return a >= v;
    case IlOp::Lt: return a < v;
    case IlOp::Le: return a <= v;
    }
    return true;
}

bool Covers(const IlBlock& b, const std::string& axis)
{
    return b.axes.empty() || std::find(b.axes.begin(), b.axes.end(), axis) != b.axes.end();
}

// Does a motion of `delta` match the block's direction filter?
bool DirMatches(const IlBlock& b, double delta)
{
    if (b.dir == 0) return true;
    return (b.dir > 0) ? (delta > 0.0) : (delta < 0.0);
}

IlCondStatus Unknown(std::string why)
{
    IlCondStatus s;
    s.truth   = true;    // FAIL CLOSED
    s.unknown = true;
    s.text    = std::move(why) + " - treated as active";
    return s;
}

IlCondStatus EvalAxis(const IlCond& c, const IlSnapshot& snap)
{
    const auto it = snap.motion.find(c.device);
    if (it == snap.motion.end()) return Unknown("no motion device '" + c.device + "'");
    const IlMotionState& m = it->second;
    if (!m.connected) return Unknown(c.device + " not connected");
    if (m.allBusy) return Unknown(c.device + " is referencing / running a buffer");
    const auto mi = m.measured.find(c.axis);
    if (mi == m.measured.end()) return Unknown(c.device + " reports no axis '" + c.axis + "'");

    IlCondStatus s;
    s.truth = Compare(mi->second, c.op, c.value);
    s.text  = c.device + " " + c.axis + " " + Fmt(mi->second);
    const auto ci = m.commanded.find(c.axis);
    if (ci != m.commanded.end() && std::fabs(ci->second - mi->second) > kFlightTol) {
        // TRUE AT EITHER END: where it is, or where it has been sent.
        s.truth = s.truth || Compare(ci->second, c.op, c.value);
        s.text += " -> " + Fmt(ci->second);
    }
    s.text += std::string(" ") + IlOpText(c.op) + " " + Fmt(c.value);
    return s;
}

IlCondStatus EvalSlide(const IlCond& c, const IlSnapshot& snap)
{
    std::vector<std::string> names = c.slides;
    if (names.empty())
        for (const auto& [n, p] : snap.slides) names.push_back(n);
    if (names.empty()) return Unknown("no pneumatic slide is configured");

    IlCondStatus s;
    std::string hits, unknowns;
    for (const std::string& n : names) {
        const auto it = snap.slides.find(n);
        if (it == snap.slides.end()) {
            s.truth = s.unknown = true;
            unknowns += (unknowns.empty() ? "" : ", ") + n + " (no such slide)";
            continue;
        }
        const IlSlidePos p = it->second;
        if (p == IlSlidePos::Unknown) {
            s.truth = s.unknown = true;
            unknowns += (unknowns.empty() ? "" : ", ") + n + " (sensors unreadable)";
            continue;
        }
        bool hit = false;
        switch (c.slideWant) {
        case IlSlideWant::Extended:     hit = (p == IlSlidePos::Extended || p == IlSlidePos::Moving); break;
        case IlSlideWant::NotRetracted: hit = (p != IlSlidePos::Retracted); break;
        case IlSlideWant::Retracted:    hit = (p == IlSlidePos::Retracted || p == IlSlidePos::Moving); break;
        }
        if (hit) {
            s.truth = true;
            const char* word = (p == IlSlidePos::Moving)    ? "moving"
                             : (p == IlSlidePos::Extended)  ? "extended"
                                                            : "retracted";
            hits += (hits.empty() ? "" : ", ") + n + " " + word;
        }
    }
    if (!hits.empty())          s.text = hits;
    if (!unknowns.empty())      s.text += (s.text.empty() ? "" : "; ") + unknowns + " - treated as active";
    if (s.text.empty())
        s.text = std::string("no slide ") + IlSlideWantText(c.slideWant);
    return s;
}

IlCondStatus EvalIo(const IlCond& c, const IlSnapshot& snap)
{
    const auto it = snap.io.find(IlIoKey(c.device, c.pin));
    if (it == snap.io.end()) return Unknown(c.device + "/" + c.pin + " not resolved");
    if (!it->second.known) return Unknown(it->second.why);
    IlCondStatus s;
    s.truth = (it->second.on == c.on);
    s.text  = c.device + "/" + c.pin + (it->second.on ? " ON" : " OFF");
    return s;
}

// A rule's live status on `snap`.
IlRuleStatus EvalRule(const IlRule& r, const IlSnapshot& snap)
{
    IlRuleStatus st;
    st.uid     = r.uid;
    st.name    = r.name;
    st.enabled = r.enabled;
    bool all = !r.when.empty();
    for (const IlCond& c : r.when) {
        IlCondStatus cs = IlEvaluateCond(c, snap);
        all        = all && cs.truth;
        st.unknown = st.unknown || cs.unknown;
        st.conds.push_back(std::move(cs));
    }
    st.active = r.enabled && all;
    return st;
}

std::string WhyActive(const IlRuleStatus& st)
{
    std::string s;
    for (const IlCondStatus& c : st.conds) s += (s.empty() ? "" : "; ") + c.text;
    return s;
}

// The state an intent would create: its targets become in-flight commanded
// ends, a reference makes the device unpredictable, an extend extends.
IlSnapshot Predict(const IlSnapshot& snap, const IlIntent& in)
{
    IlSnapshot p = snap;
    switch (in.kind) {
    case IlIntent::Kind::Move: {
        IlMotionState& m = p.motion[in.device];
        for (const auto& [axis, tgt] : in.targets) m.commanded[axis] = tgt;
        break;
    }
    case IlIntent::Kind::AllAxes:
        p.motion[in.device].allBusy = true;
        break;
    case IlIntent::Kind::SlideExtend:
        p.slides[in.device] = IlSlidePos::Extended;
        break;
    }
    return p;
}

// Where an axis moves FROM: its in-flight target, else the target last sent
// to it (while it still sits there), else where it is measured. *exact = the
// reference is a sent target, not a reading. False when nothing is known.
bool FromValue(const IlMotionState& m, const std::string& axis, double* out,
               bool* exact = nullptr)
{
    if (exact) *exact = true;
    const auto ci = m.commanded.find(axis);
    if (ci != m.commanded.end()) { *out = ci->second; return true; }
    const auto mi = m.measured.find(axis);
    const auto li = m.lastCmd.find(axis);
    if (li != m.lastCmd.end() &&
        (mi == m.measured.end() || std::fabs(li->second - mi->second) <= kLastCmdTrust)) {
        *out = li->second;
        return true;
    }
    if (exact) *exact = false;
    if (mi != m.measured.end()) { *out = mi->second; return true; }
    return false;
}

// Does sending `tgt` to `axis` move it? *delta = the signed motion (0 when
// nothing is known about the axis - then it counts as moving, either way).
bool AxisMoves(const IlMotionState& m, const std::string& axis, double tgt, double* delta)
{
    double from  = 0.0;
    bool   exact = false;
    if (!FromValue(m, axis, &from, &exact)) {
        *delta = 0.0;
        return true;
    }
    *delta = tgt - from;
    return std::fabs(*delta) > (exact ? kExactEps : kJitterTol);
}

// Is an in-flight target still being driven to? *delta = its signed motion.
bool InFlight(const IlMotionState& m, const std::string& axis, double cmd, double* delta)
{
    const auto meas = m.measured.find(axis);
    if (meas == m.measured.end()) {   // no reading: assume it moves, either way
        *delta = 0.0;
        return true;
    }
    *delta = cmd - meas->second;
    return std::fabs(*delta) > kFlightTol;
}

// A block's direction filter against a motion of `delta`; 0 = unknown
// direction, which matches every filter (fail closed).
bool DirHits(const IlBlock& b, double delta)
{
    return delta == 0.0 || DirMatches(b, delta);
}

const char* KindWhat(const IlIntent& in)
{
    if (!in.what.empty()) return in.what.c_str();
    switch (in.kind) {
    case IlIntent::Kind::Move:        return "move";
    case IlIntent::Kind::AllAxes:     return "reference";
    case IlIntent::Kind::SlideExtend: return "extend";
    }
    return "move";
}

} // namespace

std::string IlIoKey(const std::string& device, const std::string& pin)
{
    return device + "\n" + pin;
}

std::vector<std::pair<std::string, std::string>> IlIoPinsUsed(const IlConfig& cfg)
{
    std::set<std::pair<std::string, std::string>> seen;
    for (const IlRule& r : cfg.rules)
        for (const IlCond& c : r.when)
            if (c.kind == IlCondKind::IoInput) seen.emplace(c.device, c.pin);
    return {seen.begin(), seen.end()};
}

IlCondStatus IlEvaluateCond(const IlCond& c, const IlSnapshot& snap)
{
    switch (c.kind) {
    case IlCondKind::Axis:    return EvalAxis(c, snap);
    case IlCondKind::Slide:   return EvalSlide(c, snap);
    case IlCondKind::IoInput: return EvalIo(c, snap);
    }
    return Unknown("unknown condition type");
}

std::vector<IlRuleStatus> IlEvaluateAll(const IlConfig& cfg, const IlSnapshot& snap)
{
    std::vector<IlRuleStatus> out;
    out.reserve(cfg.rules.size());
    for (const IlRule& r : cfg.rules) {
        IlRuleStatus st = EvalRule(r, snap);
        if (!cfg.enabled) st.active = false;
        out.push_back(std::move(st));
    }
    return out;
}

IlVerdict IlCheckIntent(const IlConfig& cfg, const IlSnapshot& snap, const IlIntent& in)
{
    IlVerdict v;
    if (!cfg.enabled) return v;
    if (!cfg.loadError.empty()) {
        v.ok     = false;
        v.rule   = "(interlock file)";
        v.reason = std::string(KindWhat(in)) + " on " + in.device +
                   " refused: the interlock file cannot be read (" + cfg.loadError +
                   ") - fix it in the Interlock Editor, or bypass";
        return v;
    }

    const IlSnapshot pred = Predict(snap, in);
    for (const IlRule& r : cfg.rules) {
        if (!r.enabled) continue;
        const IlRuleStatus after = EvalRule(r, pred);
        if (!after.active) continue;

        auto refuse = [&](const std::string& what) {
            v.ok     = false;
            v.rule   = r.name;
            v.reason = "[" + r.name + "] refused " + what + ": " + WhyActive(after);
            return v;
        };

        // 1. The intent itself moves something this active rule blocks.
        for (const IlBlock& b : r.blocks) {
            if (in.kind == IlIntent::Kind::SlideExtend) {
                if (b.kind == IlBlockKind::SlideExtend &&
                    (b.device.empty() || b.device == in.device))
                    return refuse("extending " + in.device);
                continue;
            }
            if (b.kind != IlBlockKind::Axes || b.device != in.device) continue;
            if (in.kind == IlIntent::Kind::AllAxes)
                return refuse(std::string(KindWhat(in)) + " of " + in.device);

            const auto mi = snap.motion.find(in.device);
            const IlMotionState none;
            const IlMotionState& ms = (mi != snap.motion.end()) ? mi->second : none;
            std::string axes;
            for (const auto& [axis, tgt] : in.targets) {
                if (!Covers(b, axis)) continue;
                double d = 0.0;
                if (!AxisMoves(ms, axis, tgt, &d) || !DirHits(b, d)) continue;
                axes += (axes.empty() ? "" : ",") + axis;
            }
            if (!axes.empty())
                return refuse(std::string(KindWhat(in)) + " of " + in.device + " " + axes);
        }

        // 2. The intent would ACTIVATE this rule while one of its blocked axes
        //    is already moving - it would only trip the watch, so refuse it now.
        //    A rule that was active before the intent is the watch's business.
        if (EvalRule(r, snap).active) continue;
        for (const IlBlock& b : r.blocks) {
            if (b.kind != IlBlockKind::Axes) continue;
            const auto mi = snap.motion.find(b.device);
            if (mi == snap.motion.end() || !mi->second.connected) continue;
            const IlMotionState& m = mi->second;
            if (m.allBusy)
                return refuse(std::string(KindWhat(in)) + " on " + in.device + " while " +
                              b.device + " is referencing / running a buffer");
            for (const auto& [axis, cmd] : m.commanded) {
                if (!Covers(b, axis)) continue;
                // Retargeted by this very intent: case 1 judged the new target.
                if (in.kind == IlIntent::Kind::Move && b.device == in.device &&
                    std::any_of(in.targets.begin(), in.targets.end(),
                                [&](const auto& t) { return t.first == axis; }))
                    continue;
                double d = 0.0;
                if (!InFlight(m, axis, cmd, &d) || !DirHits(b, d)) continue;
                return refuse(std::string(KindWhat(in)) + " on " + in.device + " while " +
                              b.device + " " + axis + " is moving");
            }
        }
    }
    return v;
}

std::vector<IlViolation> IlFindViolations(const IlConfig& cfg, const IlSnapshot& snap)
{
    std::vector<IlViolation> out;
    if (!cfg.enabled) return out;
    for (const IlRule& r : cfg.rules) {
        if (!r.enabled) continue;
        const IlRuleStatus st = EvalRule(r, snap);
        if (!st.active) continue;
        for (const IlBlock& b : r.blocks) {
            if (b.kind != IlBlockKind::Axes) continue;
            const auto mi = snap.motion.find(b.device);
            if (mi == snap.motion.end() || !mi->second.connected) continue;
            const IlMotionState& m = mi->second;
            if (m.allBusy) {
                out.push_back({r.name, b.device, "*", WhyActive(st)});
                continue;
            }
            for (const auto& [axis, cmd] : m.commanded) {
                if (!Covers(b, axis)) continue;
                double d = 0.0;
                if (!InFlight(m, axis, cmd, &d) || !DirHits(b, d)) continue;
                out.push_back({r.name, b.device, axis, WhyActive(st)});
            }
        }
    }
    return out;
}

std::map<std::string, IlDirBlock> IlBlockedDirs(const IlConfig& cfg, const IlSnapshot& snap,
                                                const std::string& device)
{
    std::map<std::string, IlDirBlock> out;
    const auto mi = snap.motion.find(device);
    if (mi == snap.motion.end()) return out;
    constexpr double kStep = 1e-3;
    for (const std::string& axis : mi->second.axes) {
        double from = 0.0;
        if (!FromValue(mi->second, axis, &from)) continue;
        IlDirBlock d;
        for (int s : {+1, -1}) {
            IlIntent in;
            in.kind    = IlIntent::Kind::Move;
            in.device  = device;
            in.targets = {{axis, from + s * kStep}};
            in.what    = "jog";
            const IlVerdict v = IlCheckIntent(cfg, snap, in);
            if (v.ok) continue;
            if (s > 0) { d.plus = true;  d.rulePlus  = v.rule; }
            else       { d.minus = true; d.ruleMinus = v.rule; }
        }
        if (d.plus || d.minus) out[axis] = d;
    }
    return out;
}

// ---- Text ----------------------------------------------------------------------

const char* IlOpText(IlOp op)
{
    switch (op) {
    case IlOp::Gt: return ">";
    case IlOp::Ge: return ">=";
    case IlOp::Lt: return "<";
    case IlOp::Le: return "<=";
    }
    return "?";
}

const char* IlSlideWantText(IlSlideWant w)
{
    switch (w) {
    case IlSlideWant::Extended:     return "extended";
    case IlSlideWant::NotRetracted: return "not retracted";
    case IlSlideWant::Retracted:    return "retracted";
    }
    return "?";
}

std::string IlCondText(const IlCond& c)
{
    switch (c.kind) {
    case IlCondKind::Axis:
        return c.device + " " + c.axis + " " + IlOpText(c.op) + " " + Fmt(c.value);
    case IlCondKind::IoInput:
        return c.device + "/" + c.pin + (c.on ? " is ON" : " is OFF");
    case IlCondKind::Slide: {
        std::string who;
        for (const std::string& s : c.slides) who += (who.empty() ? "" : ", ") + s;
        return (who.empty() ? std::string("any slide") : who) + " " + IlSlideWantText(c.slideWant);
    }
    }
    return "?";
}

std::string IlBlockText(const IlBlock& b)
{
    if (b.kind == IlBlockKind::SlideExtend)
        return "extend " + (b.device.empty() ? std::string("any slide") : b.device);
    std::string axes;
    for (const std::string& a : b.axes) axes += (axes.empty() ? "" : ",") + a;
    std::string s = b.device + " " + (axes.empty() ? std::string("all axes") : axes);
    if (b.dir > 0) s += " (+ only)";
    if (b.dir < 0) s += " (- only)";
    return s;
}

// ---- JSON ----------------------------------------------------------------------

namespace {

IlOp ParseOp(const std::string& s)
{
    if (s == ">=") return IlOp::Ge;
    if (s == "<")  return IlOp::Lt;
    if (s == "<=") return IlOp::Le;
    return IlOp::Gt;
}

IlSlideWant ParseWant(const std::string& s)
{
    if (s == "extended")  return IlSlideWant::Extended;
    if (s == "retracted") return IlSlideWant::Retracted;
    return IlSlideWant::NotRetracted;
}

const char* WantKey(IlSlideWant w)
{
    switch (w) {
    case IlSlideWant::Extended:     return "extended";
    case IlSlideWant::NotRetracted: return "not_retracted";
    case IlSlideWant::Retracted:    return "retracted";
    }
    return "not_retracted";
}

nlohmann::json CondToJson(const IlCond& c)
{
    switch (c.kind) {
    case IlCondKind::Slide:
        return {{"type", "slide"}, {"slides", c.slides}, {"state", WantKey(c.slideWant)}};
    case IlCondKind::IoInput:
        return {{"type", "io"}, {"device", c.device}, {"pin", c.pin}, {"on", c.on}};
    case IlCondKind::Axis:
        return {{"type", "axis"}, {"device", c.device}, {"axis", c.axis},
                {"op", IlOpText(c.op)}, {"value", c.value}};
    }
    return {};
}

IlCond CondFromJson(const nlohmann::json& j)
{
    IlCond c;
    const std::string type = j.value("type", std::string("axis"));
    if (type == "slide") {
        c.kind      = IlCondKind::Slide;
        c.slides    = j.value("slides", std::vector<std::string>{});
        c.slideWant = ParseWant(j.value("state", std::string("not_retracted")));
    } else if (type == "io") {
        c.kind   = IlCondKind::IoInput;
        c.device = j.value("device", std::string());
        c.pin    = j.value("pin", std::string());
        c.on     = j.value("on", true);
    } else {
        // An unrecognised type reads as an axis condition on nothing - it
        // resolves to no device and so FAILS CLOSED rather than vanishing.
        c.kind   = IlCondKind::Axis;
        c.device = j.value("device", std::string());
        c.axis   = j.value("axis", std::string());
        c.op     = ParseOp(j.value("op", std::string(">")));
        c.value  = j.value("value", 0.0);
    }
    return c;
}

nlohmann::json BlockToJson(const IlBlock& b)
{
    if (b.kind == IlBlockKind::SlideExtend)
        return {{"type", "slide_extend"}, {"slide", b.device}};
    return {{"type", "axes"}, {"device", b.device}, {"axes", b.axes}, {"dir", b.dir}};
}

IlBlock BlockFromJson(const nlohmann::json& j)
{
    IlBlock b;
    if (j.value("type", std::string("axes")) == "slide_extend") {
        b.kind   = IlBlockKind::SlideExtend;
        b.device = j.value("slide", std::string());
        return b;
    }
    b.kind   = IlBlockKind::Axes;
    b.device = j.value("device", std::string());
    b.axes   = j.value("axes", std::vector<std::string>{});
    const int d = j.value("dir", 0);
    b.dir = (d > 0) ? 1 : (d < 0) ? -1 : 0;
    return b;
}

nlohmann::json ToJson(const IlConfig& cfg)
{
    nlohmann::json rules = nlohmann::json::array();
    for (const IlRule& r : cfg.rules) {
        nlohmann::json when = nlohmann::json::array(), blocks = nlohmann::json::array();
        for (const IlCond& c : r.when) when.push_back(CondToJson(c));
        for (const IlBlock& b : r.blocks) blocks.push_back(BlockToJson(b));
        rules.push_back({{"uid", r.uid}, {"name", r.name}, {"note", r.note},
                         {"enabled", r.enabled}, {"when", when}, {"blocks", blocks}});
    }
    return {{"enabled", cfg.enabled}, {"watch_ms", cfg.watchMs}, {"rules", rules}};
}

IlConfig FromJson(const nlohmann::json& j)
{
    IlConfig cfg;
    cfg.enabled = j.value("enabled", true);
    cfg.watchMs = std::clamp(j.value("watch_ms", 50), 10, 1000);
    if (j.contains("rules")) {
        for (const nlohmann::json& rj : j.at("rules")) {
            IlRule r;
            r.uid     = rj.value("uid", std::string());
            r.name    = rj.value("name", std::string());
            r.note    = rj.value("note", std::string());
            r.enabled = rj.value("enabled", true);
            if (r.uid.empty()) r.uid = GenerateUid();
            if (r.name.empty()) r.name = "rule " + r.uid;
            if (rj.contains("when"))
                for (const nlohmann::json& c : rj.at("when")) r.when.push_back(CondFromJson(c));
            if (rj.contains("blocks"))
                for (const nlohmann::json& b : rj.at("blocks")) r.blocks.push_back(BlockFromJson(b));
            cfg.rules.push_back(std::move(r));
        }
    }
    return cfg;
}

} // namespace

std::string InterlocksLocalPath() { return MachinePath("config/interlocks.local.json"); }

IlConfig LoadInterlocks(std::string* pathUsed)
{
    if (pathUsed) pathUsed->clear();
    const std::string local = InterlocksLocalPath();
    for (const std::string& path : {local, std::string(kShippedPath)}) {
        std::ifstream f(path);
        if (!f.is_open()) continue;
        if (pathUsed) *pathUsed = path;
        try {
            nlohmann::json j;
            f >> j;
            return FromJson(j);
        } catch (const std::exception& e) {
            // NOT a fall-through to the shipped (empty) default: that would
            // silently switch every interlock off. Refuse motion instead.
            FD_ERROR("interlock", "%s is malformed (%s) - every move is refused until it "
                                  "is fixed", path.c_str(), e.what());
            IlConfig broken;
            broken.loadError = path + ": " + e.what();
            return broken;
        }
    }
    return IlConfig{};
}

bool SaveInterlocks(const IlConfig& cfg, std::string* err)
{
    const std::string path = InterlocksLocalPath();
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    // Write a temp file and rename over: a half-written interlock file would
    // read as malformed and refuse every move.
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        if (!f.is_open()) {
            if (err) *err = "cannot open " + tmp;
            return false;
        }
        f << ToJson(cfg).dump(2) << '\n';
        if (!f.good()) {
            if (err) *err = "write failed: " + tmp;
            return false;
        }
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        if (err) *err = "cannot replace " + path + ": " + ec.message();
        return false;
    }
    return true;
}

} // namespace firedog
