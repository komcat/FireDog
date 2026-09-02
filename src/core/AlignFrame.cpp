// AlignFrame.cpp
#include "core/AlignFrame.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid / NowStamp

#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/alignments.local.json";
constexpr const char* kBasePath  = "config/alignments.json";

nlohmann::json LightingToJson(const AlignLighting& l)
{
    nlohmann::json j;
    j["kind"]    = l.kind;
    j["device"]  = l.device;
    j["channel"] = l.channel;
    j["value"]   = l.value;
    return j;
}

AlignLighting LightingFromJson(const nlohmann::json& j)
{
    AlignLighting l;
    if (!j.is_object()) return l;
    l.kind    = j.value("kind", "");
    l.device  = j.value("device", "");
    l.channel = j.value("channel", "");
    l.value   = j.value("value", 0.0);
    return l;
}

nlohmann::json StationToJson(const AlignStation& s)
{
    nlohmann::json j;
    j["pos_uid"]              = s.posUid;
    j["exposure_us"]          = s.exposureUs;
    j["mode"]                 = s.mode;
    j["detector"]             = s.detector;
    j["auto_fallback_manual"] = s.autoFallbackManual;
    j["lighting"]             = LightingToJson(s.lighting);
    return j;
}

AlignStation StationFromJson(const nlohmann::json& j)
{
    AlignStation s;
    if (!j.is_object()) return s;
    s.posUid             = j.value("pos_uid", "");
    s.exposureUs         = j.value("exposure_us", 0.0);
    s.mode               = j.value("mode", "manual");
    s.detector           = j.value("detector", "");
    s.autoFallbackManual = j.value("auto_fallback_manual", false);
    if (j.contains("lighting")) s.lighting = LightingFromJson(j["lighting"]);
    return s;
}

nlohmann::json PointToJson(const AlignPoint& p)
{
    nlohmann::json j;
    j["gx"] = p.gx; j["gy"] = p.gy;
    j["px"] = p.px; j["py"] = p.py;
    j["wx"] = p.wx; j["wy"] = p.wy;
    return j;
}

AlignPoint PointFromJson(const nlohmann::json& j)
{
    AlignPoint p;
    if (!j.is_object()) return p;
    p.gx = j.value("gx", 0.0); p.gy = j.value("gy", 0.0);
    p.px = j.value("px", 0.0); p.py = j.value("py", 0.0);
    p.wx = j.value("wx", 0.0); p.wy = j.value("wy", 0.0);
    return p;
}

nlohmann::json ResultToJson(const AlignResult& r)
{
    nlohmann::json j;
    j["valid"]       = r.valid;
    j["captured_at"] = r.capturedAt;
    j["e1"]          = PointToJson(r.e1);
    j["e2"]          = PointToJson(r.e2);
    j["origin_x"]    = r.originX;
    j["origin_y"]    = r.originY;
    j["theta_deg"]   = r.thetaDeg;
    j["span_mm"]     = r.spanMm;
    return j;
}

AlignResult ResultFromJson(const nlohmann::json& j)
{
    AlignResult r;
    if (!j.is_object()) return r;
    r.valid      = j.value("valid", false);
    r.capturedAt = j.value("captured_at", "");
    if (j.contains("e1")) r.e1 = PointFromJson(j["e1"]);
    if (j.contains("e2")) r.e2 = PointFromJson(j["e2"]);
    r.originX  = j.value("origin_x", 0.0);
    r.originY  = j.value("origin_y", 0.0);
    r.thetaDeg = j.value("theta_deg", 0.0);
    r.spanMm   = j.value("span_mm", 0.0);
    return r;
}

nlohmann::json GuardToJson(const AlignGuard& g)
{
    nlohmann::json j;
    j["enabled"]       = g.enabled;
    j["span_mm"]       = g.spanMm;
    j["span_tol_mm"]   = g.spanTolMm;
    j["theta_deg"]     = g.thetaDeg;
    j["theta_tol_deg"] = g.thetaTolDeg;
    return j;
}

AlignGuard GuardFromJson(const nlohmann::json& j)
{
    AlignGuard g;
    if (!j.is_object()) return g;
    g.enabled     = j.value("enabled", false);
    g.spanMm      = j.value("span_mm", 0.0);
    g.spanTolMm   = j.value("span_tol_mm", 0.2);
    g.thetaDeg    = j.value("theta_deg", 0.0);
    g.thetaTolDeg = j.value("theta_tol_deg", 5.0);
    return g;
}

nlohmann::json RunToJson(const AlignRun& r)
{
    nlohmann::json j;
    j["at"]        = r.at;
    j["e1x"]       = r.e1x;
    j["e1y"]       = r.e1y;
    j["e2x"]       = r.e2x;
    j["e2y"]       = r.e2y;
    j["dx"]        = r.dx;
    j["dy"]        = r.dy;
    j["span_mm"]   = r.spanMm;
    j["theta_deg"] = r.thetaDeg;
    j["guarded"]   = r.guarded;
    j["ok"]        = r.ok;
    return j;
}

AlignRun RunFromJson(const nlohmann::json& j)
{
    AlignRun r;
    if (!j.is_object()) return r;
    r.at       = j.value("at", "");
    r.e1x      = j.value("e1x", 0.0);
    r.e1y      = j.value("e1y", 0.0);
    r.e2x      = j.value("e2x", 0.0);
    r.e2y      = j.value("e2y", 0.0);
    r.dx       = j.value("dx", 0.0);
    r.dy       = j.value("dy", 0.0);
    r.spanMm   = j.value("span_mm", 0.0);
    r.thetaDeg = j.value("theta_deg", 0.0);
    r.guarded  = j.value("guarded", false);
    r.ok       = j.value("ok", true);
    return r;
}

nlohmann::json NominalToJson(const AlignNominal& n)
{
    nlohmann::json j;
    j["uid"]      = n.uid;
    j["name"]     = n.name;
    j["lx"]       = n.lx;
    j["ly"]       = n.ly;
    j["p1"]       = PointToJson(n.p1);
    j["p2"]       = PointToJson(n.p2);
    // Only when they mean something: a two-mark nominal writing two empty
    // points would read as a four-mark teach with two marks at the origin.
    if (n.points >= 4) {
        j["p3"]     = PointToJson(n.p3);
        j["p4"]     = PointToJson(n.p4);
        j["points"] = n.points;
    }
    j["mx"]       = n.mx;
    j["my"]       = n.my;
    j["span_mm"]  = n.spanMm;
    j["taught_at"] = n.taughtAt;
    j["frame_at"]  = n.frameAt;
    j["mode1"]     = n.mode1;
    j["mode2"]     = n.mode2;
    j["det1"]      = n.det1;
    j["det2"]      = n.det2;
    j["auto_fallback_manual"] = n.autoFallbackManual;
    j["view_uid"]  = n.viewUid;
    j["cam_uid"]   = n.camUid;
    return j;
}

AlignNominal NominalFromJson(const nlohmann::json& j)
{
    AlignNominal n;
    if (!j.is_object()) return n;
    n.uid  = j.value("uid", "");
    if (n.uid.empty()) n.uid = GenerateUid();   // hand-written row — assign once
    n.name = j.value("name", "nominal");
    n.lx   = j.value("lx", 0.0);
    n.ly   = j.value("ly", 0.0);
    if (j.contains("p1")) n.p1 = PointFromJson(j["p1"]);
    if (j.contains("p2")) n.p2 = PointFromJson(j["p2"]);
    if (j.contains("p3")) n.p3 = PointFromJson(j["p3"]);
    if (j.contains("p4")) n.p4 = PointFromJson(j["p4"]);
    // Absent = two, which is what every file written before four existed meant.
    n.points = j.value("points", 2);
    if (n.points != 4) n.points = 2;
    n.mx       = j.value("mx", 0.0);
    n.my       = j.value("my", 0.0);
    n.spanMm   = j.value("span_mm", 0.0);
    n.taughtAt = j.value("taught_at", "");
    n.frameAt  = j.value("frame_at", "");
    n.det1     = j.value("det1", "");
    n.det2     = j.value("det2", "");
    // Written before the mode existed, a named model WAS the mode — so a row
    // with a detector and no mode key loads as auto, which is what it did.
    n.mode1    = j.value("mode1", n.det1.empty() ? "manual" : "auto");
    n.mode2    = j.value("mode2", n.det2.empty() ? "manual" : "auto");
    n.autoFallbackManual = j.value("auto_fallback_manual", true);
    n.viewUid  = j.value("view_uid", "");
    n.camUid   = j.value("cam_uid", "");
    return n;
}

std::vector<AlignProc> ParseAlignProcs(std::ifstream& f, const std::string& path,
                                       std::string* uidOut)
{
    std::vector<AlignProc> out;
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", "");
        if (!j.contains("alignments") || !j["alignments"].is_array()) return out;
        for (const auto& row : j["alignments"]) {
            if (!row.is_object()) continue;
            out.push_back(AlignProcFromJson(row));
        }
        FD_INFO("align", "loaded %zu alignment procedure(s) from %s", out.size(), path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("align", "alignments load failed (%s): starting empty", e.what());
        return std::vector<AlignProc>{};
    }
    return out;
}

} // namespace

// ---- frame maths ---------------------------------------------------------

bool SolveAlignFrame(const AlignPoint& e1, const AlignPoint& e2,
                     AlignResult* out, std::string* err)
{
    const double dx = e2.wx - e1.wx;
    const double dy = e2.wy - e1.wy;
    const double span = std::sqrt(dx * dx + dy * dy);
    if (span < kAlignMinSpanMM) {
        if (err) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "E1 and E2 are only %.4f mm apart (need >= %.3f mm)",
                          span, kAlignMinSpanMM);
            *err = buf;
        }
        return false;
    }
    if (!out) return true;

    out->e1 = e1;
    out->e2 = e2;
    out->originX  = e1.wx;
    out->originY  = e1.wy;
    out->thetaDeg = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
    out->spanMm   = span;
    out->capturedAt = NowStamp();
    out->valid    = true;
    return true;
}

// ---- the guard band ------------------------------------------------------

void AlignSpanTheta(const AlignPoint& e1, const AlignPoint& e2,
                    double* spanMm, double* thetaDeg)
{
    const double dx = e2.wx - e1.wx;
    const double dy = e2.wy - e1.wy;
    if (spanMm)   *spanMm = std::sqrt(dx * dx + dy * dy);
    // Same expression SolveAlignFrame uses, deliberately: a readout that
    // disagreed with the frame by a sign convention would be worse than none.
    if (thetaDeg) *thetaDeg = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
}

bool AlignGuardCheck(const AlignGuard& g, double spanMm, double thetaDeg,
                     double* spanErr, double* thetaErr)
{
    if (!g.enabled) {
        if (spanErr)  *spanErr  = 0.0;
        if (thetaErr) *thetaErr = 0.0;
        return true;
    }
    const double ds = spanMm - g.spanMm;
    // Wrapped to (-180, 180]: a part at 179 deg and a reference at -179 are two
    // degrees apart, not 358. Without this a band straddling the wrap refuses
    // every part that crosses it.
    double dt = thetaDeg - g.thetaDeg;
    while (dt >  180.0) dt -= 360.0;
    while (dt <= -180.0) dt += 360.0;
    if (spanErr)  *spanErr  = ds;
    if (thetaErr) *thetaErr = dt;
    return std::fabs(ds) <= g.spanTolMm && std::fabs(dt) <= g.thetaTolDeg;
}

AlignRun MakeAlignRun(const AlignGuard& g, const AlignResult& r)
{
    AlignRun out;
    out.at       = r.capturedAt;
    out.e1x      = r.e1.wx;
    out.e1y      = r.e1.wy;
    out.e2x      = r.e2.wx;
    out.e2y      = r.e2.wy;
    out.dx       = r.e2.wx - r.e1.wx;
    out.dy       = r.e2.wy - r.e1.wy;
    out.spanMm   = r.spanMm;
    out.thetaDeg = r.thetaDeg;
    out.guarded  = g.enabled;
    out.ok       = AlignGuardCheck(g, r.spanMm, r.thetaDeg);
    return out;
}

AlignCpk ComputeAlignCpk(const std::vector<double>& values, double ref, double tol)
{
    AlignCpk out;
    out.n = static_cast<int>(values.size());
    if (values.empty()) return out;

    double sum = 0.0;
    out.minV = out.maxV = values.front();
    for (const double v : values) {
        sum += v;
        if (v < out.minV) out.minV = v;
        if (v > out.maxV) out.maxV = v;
    }
    out.mean = sum / static_cast<double>(out.n);
    if (out.n < 2 || tol <= 0.0) return out;   // valid stays false

    double ss = 0.0;
    for (const double v : values) ss += (v - out.mean) * (v - out.mean);
    out.sigma = std::sqrt(ss / static_cast<double>(out.n - 1));
    out.valid = true;

    // A spread below a part per million of the tolerance is the arithmetic
    // running out of digits, not a perfect process. Say so rather than printing
    // an index in the millions.
    if (out.sigma <= tol * 1e-6) { out.noSpread = true; return out; }

    out.cp  = tol / (3.0 * out.sigma);
    out.cpk = (tol - std::fabs(out.mean - ref)) / (3.0 * out.sigma);
    return out;
}

void AlignLocalToMachine(const AlignResult& r, double lx, double ly,
                         double* mx, double* my)
{
    if (!r.valid) { if (mx) *mx = lx; if (my) *my = ly; return; }
    const double th = r.thetaDeg * 3.14159265358979323846 / 180.0;
    const double c = std::cos(th), s = std::sin(th);
    if (mx) *mx = r.originX + lx * c - ly * s;
    if (my) *my = r.originY + lx * s + ly * c;
}

void AlignMachineToLocal(const AlignResult& r, double mx, double my,
                         double* lx, double* ly)
{
    if (!r.valid) { if (lx) *lx = mx; if (ly) *ly = my; return; }
    const double th = r.thetaDeg * 3.14159265358979323846 / 180.0;
    const double c = std::cos(th), s = std::sin(th);
    const double dx = mx - r.originX, dy = my - r.originY;
    if (lx) *lx =  dx * c + dy * s;   // R(-theta) — the transpose
    if (ly) *ly = -dx * s + dy * c;
}

bool SolveAlignNominalN(const AlignResult& frame, const AlignPoint* pts, int n,
                        AlignNominal* out, std::string* err)
{
    if (!frame.valid) {
        if (err) *err = "no valid local frame - teach the alignment first";
        return false;
    }
    if (!pts || (n != 2 && n != 4)) {
        if (err) *err = "a nominal is taught from 2 or 4 marks";
        return false;
    }
    if (!out) return true;

    double sx = 0.0, sy = 0.0;
    for (int i = 0; i < n; ++i) { sx += pts[i].wx; sy += pts[i].wy; }
    out->mx = sx / n;
    out->my = sy / n;

    // Two marks: the separation, which is what `spanMm` has always meant. Four:
    // twice the mean distance from the centre — the same figure for a ring that
    // the separation is for a pair, so one number reads the same either way.
    // It is the only number that can see a mark placed off its feature.
    if (n == 2) {
        const double dx = pts[1].wx - pts[0].wx;
        const double dy = pts[1].wy - pts[0].wy;
        out->spanMm = std::sqrt(dx * dx + dy * dy);
    } else {
        double sum = 0.0;
        for (int i = 0; i < n; ++i) {
            const double dx = pts[i].wx - out->mx;
            const double dy = pts[i].wy - out->my;
            sum += std::sqrt(dx * dx + dy * dy);
        }
        out->spanMm = 2.0 * sum / n;
    }

    out->points = n;
    out->p1 = pts[0];
    out->p2 = pts[1];
    // Left as they were on a two-mark teach: `points` says they are not real,
    // and a re-teach from four to two must not leave the old ring half-written.
    out->p3 = (n == 4) ? pts[2] : AlignPoint{};
    out->p4 = (n == 4) ? pts[3] : AlignPoint{};

    AlignMachineToLocal(frame, out->mx, out->my, &out->lx, &out->ly);
    out->taughtAt = NowStamp();
    out->frameAt  = frame.capturedAt;
    return true;
}

bool SolveAlignNominal(const AlignResult& frame, const AlignPoint& p1,
                       const AlignPoint& p2, AlignNominal* out, std::string* err)
{
    const AlignPoint pts[2] = {p1, p2};
    return SolveAlignNominalN(frame, pts, 2, out, err);
}

// ---- lookup --------------------------------------------------------------

const AlignProc* FindAlignProc(const std::vector<AlignProc>& list,
                               const std::string& uidOrName)
{
    if (uidOrName.empty()) return nullptr;
    for (const AlignProc& p : list) if (p.uid == uidOrName) return &p;
    for (const AlignProc& p : list) if (p.name == uidOrName) return &p;
    return nullptr;
}

AlignProc* FindAlignProc(std::vector<AlignProc>& list, const std::string& uidOrName)
{
    const std::vector<AlignProc>& c = list;
    return const_cast<AlignProc*>(FindAlignProc(c, uidOrName));
}

const AlignNominal* FindAlignNominal(const AlignProc& p, const std::string& uidOrName)
{
    if (uidOrName.empty()) return nullptr;
    for (const AlignNominal& n : p.nominals) if (n.uid == uidOrName) return &n;
    for (const AlignNominal& n : p.nominals) if (n.name == uidOrName) return &n;
    return nullptr;
}

AlignNominal* FindAlignNominal(AlignProc& p, const std::string& uidOrName)
{
    const AlignProc& c = p;
    return const_cast<AlignNominal*>(FindAlignNominal(c, uidOrName));
}

// ---- persistence ---------------------------------------------------------

nlohmann::json AlignProcToJson(const AlignProc& p)
{
    nlohmann::json j;
    j["uid"]      = p.uid;
    j["name"]     = p.name;
    j["device"]   = p.device;
    j["station1"] = StationToJson(p.s1);
    j["station2"] = StationToJson(p.s2);
    j["result"]   = ResultToJson(p.result);
    j["guard"]    = GuardToJson(p.guard);
    nlohmann::json noms = nlohmann::json::array();
    for (const AlignNominal& n : p.nominals) noms.push_back(NominalToJson(n));
    j["nominals"] = std::move(noms);
    nlohmann::json runs = nlohmann::json::array();
    for (const AlignRun& r : p.runs) runs.push_back(RunToJson(r));
    j["runs"] = std::move(runs);
    return j;
}

AlignProc AlignProcFromJson(const nlohmann::json& j)
{
    AlignProc p;
    if (!j.is_object()) return p;
    p.uid    = j.value("uid", "");
    if (p.uid.empty()) p.uid = GenerateUid();   // legacy row — assign once
    p.name   = j.value("name", "alignment");
    p.device = j.value("device", "gantry-main");
    if (j.contains("station1")) p.s1 = StationFromJson(j["station1"]);
    if (j.contains("station2")) p.s2 = StationFromJson(j["station2"]);
    if (j.contains("result"))   p.result = ResultFromJson(j["result"]);
    // Absent on every procedure authored before the band existed, and the
    // default is `enabled = false` — so an old procedure loads with no band and
    // behaves exactly as it did.
    if (j.contains("guard")) p.guard = GuardFromJson(j["guard"]);
    // Absent in every procedure authored before nominals existed — an empty
    // list is the right answer there, so no migration is needed.
    if (j.contains("nominals") && j["nominals"].is_array()) {
        for (const auto& row : j["nominals"]) {
            if (!row.is_object()) continue;
            p.nominals.push_back(NominalFromJson(row));
        }
    }
    // The history, oldest first. Truncated on load as well as on append: a
    // hand-edited file must not be able to grow the ring past its cap.
    if (j.contains("runs") && j["runs"].is_array()) {
        for (const auto& row : j["runs"]) {
            if (!row.is_object()) continue;
            p.runs.push_back(RunFromJson(row));
        }
        if (p.runs.size() > kAlignRunHistoryMax)
            p.runs.erase(p.runs.begin(),
                         p.runs.end() - static_cast<long>(kAlignRunHistoryMax));
    }
    return p;
}

std::vector<AlignProc> LoadAlignProcs()
{
    std::string path = kLocalPath;
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return std::vector<AlignProc>{};   // first run — start empty
    }
    return ParseAlignProcs(f, path, nullptr);
}

std::vector<AlignProc> LoadAlignProcsFile(const std::string& path, std::string* uidOut)
{
    if (uidOut) uidOut->clear();
    std::ifstream f(path);
    if (!f.is_open()) return std::vector<AlignProc>{};
    return ParseAlignProcs(f, path, uidOut);
}

bool SaveAlignProcsFile(const std::string& path, const std::vector<AlignProc>& procs,
                        const std::string& uid)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const AlignProc& p : procs) arr.push_back(AlignProcToJson(p));

    nlohmann::json j;
    if (!uid.empty()) j["uid"] = uid;
    j["alignments"] = std::move(arr);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("align", "alignments save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
