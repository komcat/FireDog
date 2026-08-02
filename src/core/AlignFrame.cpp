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

nlohmann::json NominalToJson(const AlignNominal& n)
{
    nlohmann::json j;
    j["uid"]      = n.uid;
    j["name"]     = n.name;
    j["lx"]       = n.lx;
    j["ly"]       = n.ly;
    j["p1"]       = PointToJson(n.p1);
    j["p2"]       = PointToJson(n.p2);
    j["mx"]       = n.mx;
    j["my"]       = n.my;
    j["span_mm"]  = n.spanMm;
    j["taught_at"] = n.taughtAt;
    j["frame_at"]  = n.frameAt;
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
    n.mx       = j.value("mx", 0.0);
    n.my       = j.value("my", 0.0);
    n.spanMm   = j.value("span_mm", 0.0);
    n.taughtAt = j.value("taught_at", "");
    n.frameAt  = j.value("frame_at", "");
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

bool SolveAlignNominal(const AlignResult& frame, const AlignPoint& p1,
                       const AlignPoint& p2, AlignNominal* out, std::string* err)
{
    if (!frame.valid) {
        if (err) *err = "no valid local frame - teach the alignment first";
        return false;
    }
    if (!out) return true;

    const double dx = p2.wx - p1.wx;
    const double dy = p2.wy - p1.wy;
    out->p1 = p1;
    out->p2 = p2;
    out->mx = (p1.wx + p2.wx) * 0.5;
    out->my = (p1.wy + p2.wy) * 0.5;
    out->spanMm = std::sqrt(dx * dx + dy * dy);
    AlignMachineToLocal(frame, out->mx, out->my, &out->lx, &out->ly);
    out->taughtAt = NowStamp();
    out->frameAt  = frame.capturedAt;
    return true;
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
    nlohmann::json noms = nlohmann::json::array();
    for (const AlignNominal& n : p.nominals) noms.push_back(NominalToJson(n));
    j["nominals"] = std::move(noms);
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
    // Absent in every procedure authored before nominals existed — an empty
    // list is the right answer there, so no migration is needed.
    if (j.contains("nominals") && j["nominals"].is_array()) {
        for (const auto& row : j["nominals"]) {
            if (!row.is_object()) continue;
            p.nominals.push_back(NominalFromJson(row));
        }
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
