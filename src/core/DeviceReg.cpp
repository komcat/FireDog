// DeviceReg.cpp — the fit, the prediction, and the file.
//
// See DeviceReg.h for why R is measured rather than taken from
// motion_transforms.json, and why the triad is never assumed right-handed.
#include "core/DeviceReg.h"

#include "core/Log.h"
#include "core/Product.h"   // NowStamp

#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {

constexpr const char* kLocalPath = "config/device_reg.local.json";
constexpr const char* kBasePath  = "config/device_reg.json";

constexpr double kRadToDeg = 57.295779513082320876798154814105;

RegVec3 Sub(const RegVec3& a, const RegVec3& b)
{
    return RegVec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

RegVec3 Scale(const RegVec3& a, double s)
{
    return RegVec3{a.x * s, a.y * s, a.z * s};
}

double Dot(const RegVec3& a, const RegVec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

RegVec3 Cross(const RegVec3& a, const RegVec3& b)
{
    return RegVec3{a.y * b.z - a.z * b.y,
                   a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x};
}

double Norm(const RegVec3& a) { return std::sqrt(Dot(a, a)); }

// Normalise, or return the zero vector when there is nothing to normalise. The
// callers all gate on the norm first, so this only has to be safe, not clever.
RegVec3 Unit(const RegVec3& a)
{
    const double n = Norm(a);
    if (n <= 0.0) return RegVec3{};
    return Scale(a, 1.0 / n);
}

// Angle between two unit vectors, in degrees. Clamped because a dot product of
// two hand-normalised vectors can land a hair outside [-1, 1] and acos would
// answer NaN — which then propagates into a refusal message nobody can read.
double AngleDeg(const RegVec3& a, const RegVec3& b)
{
    double d = Dot(a, b);
    if (d > 1.0)  d = 1.0;
    if (d < -1.0) d = -1.0;
    return std::acos(d) * kRadToDeg;
}

void SetColumn(DeviceReg& r, int j, const RegVec3& v)
{
    r.R[0][j] = v.x;
    r.R[1][j] = v.y;
    r.R[2][j] = v.z;
}

nlohmann::json Vec3ToJson(const RegVec3& v)
{
    nlohmann::json j;
    j["x"] = v.x;
    j["y"] = v.y;
    j["z"] = v.z;
    return j;
}

RegVec3 Vec3FromJson(const nlohmann::json& j)
{
    RegVec3 v;
    if (!j.is_object()) return v;
    v.x = j.value("x", 0.0);
    v.y = j.value("y", 0.0);
    v.z = j.value("z", 0.0);
    return v;
}

} // namespace

const char* DeviceRegLocalPath() { return kLocalPath; }

// ---- The maths -------------------------------------------------------------

RegVec3 DeviceRegPredict(const DeviceReg& r, const RegVec3& h)
{
    return RegVec3{
        r.R[0][0] * h.x + r.R[0][1] * h.y + r.R[0][2] * h.z + r.t.x,
        r.R[1][0] * h.x + r.R[1][1] * h.y + r.R[1][2] * h.z + r.t.y,
        r.R[2][0] * h.x + r.R[2][1] * h.y + r.R[2][2] * h.z + r.t.z};
}

RegVec3 DeviceRegSolve(const DeviceReg& r, const RegVec3& p)
{
    const RegVec3 d = Sub(p, r.t);
    // R^T, not an inverse: R is orthogonal by construction. That stays true when
    // it is IMPROPER (det = -1), which it often is — see the header.
    return RegVec3{
        r.R[0][0] * d.x + r.R[1][0] * d.y + r.R[2][0] * d.z,
        r.R[0][1] * d.x + r.R[1][1] * d.y + r.R[2][1] * d.z,
        r.R[0][2] * d.x + r.R[1][2] * d.y + r.R[2][2] * d.z};
}

std::string DeviceRegFit(DeviceReg& r, double jogMm, const RegVec3& h0,
                         const RegVec3& p0, const RegVec3& p1,
                         const RegVec3& p2, const RegVec3& p3)
{
    r.jogMm = jogMm;
    r.h0    = h0;
    r.p0    = p0;
    r.p1    = p1;
    r.p2    = p2;
    r.p3    = p3;

    // The jog length divides every column, so it is checked before it is used
    // rather than after a divide by something near zero.
    if (jogMm < kDeviceRegMinJogMm) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "jog of %.3f mm is below the %.1f mm minimum - under it the "
                      "pick noise dominates the direction and the angles mean nothing",
                      jogMm, kDeviceRegMinJogMm);
        return buf;
    }

    const RegVec3 c1 = Scale(Sub(p1, p0), 1.0 / jogMm);
    const RegVec3 c2 = Scale(Sub(p2, p0), 1.0 / jogMm);
    const RegVec3 c3 = Scale(Sub(p3, p0), 1.0 / jogMm);

    r.colNorm[0] = Norm(c1);
    r.colNorm[1] = Norm(c2);
    r.colNorm[2] = Norm(c3);

    // Every quality field is filled BEFORE the first refusal returns, so a
    // rejected fit still says which number was out. That is the whole reason the
    // measurement is kept on the record.
    const RegVec3 n1 = Unit(c1);
    const RegVec3 n2 = Unit(c2);
    const RegVec3 n3m = Unit(c3);

    r.orthoDeg[0] = AngleDeg(n1, n2) - 90.0;
    r.orthoDeg[1] = AngleDeg(n2, n3m) - 90.0;
    r.orthoDeg[2] = AngleDeg(n3m, n1) - 90.0;

    const RegVec3 perp = Unit(Cross(n1, n2));
    r.handed   = (Dot(perp, n3m) >= 0.0) ? 1.0 : -1.0;
    r.crossDeg = AngleDeg(n3m, Scale(perp, r.handed));

    // A column that did not travel the distance it was told to. This catches a
    // jog that hit a limit, a units mismatch, and a pick taken before the device
    // finished moving — all of which otherwise fit perfectly well and lie.
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(r.colNorm[i] - 1.0) <= kDeviceRegNormTol) continue;
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "axis %c moved %.4f mm for a %.3f mm command (scale %.4f, "
                      "tolerance %.3f) - the jog did not complete, or the two devices "
                      "disagree about what a millimetre is",
                      "XYZ"[i], r.colNorm[i] * jogMm, jogMm, r.colNorm[i],
                      kDeviceRegNormTol);
        return buf;
    }

    static const char* const kPair[3] = {"X and Y", "Y and Z", "Z and X"};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(r.orthoDeg[i]) <= kDeviceRegOrthoTolDeg) continue;
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "measured axes %s are %+.3f deg off perpendicular (tolerance "
                      "%.2f) - the fit is skewed, and a skewed fit misses by more the "
                      "further the tip travels from where it was taught",
                      kPair[i], r.orthoDeg[i], kDeviceRegOrthoTolDeg);
        return buf;
    }

    if (r.crossDeg > kDeviceRegCrossTolDeg) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "the measured Z axis is %.3f deg from where X and Y say it must "
                      "be (tolerance %.2f) - one pick or one focus pass is wrong, and "
                      "the handedness it was going to settle cannot be trusted",
                      r.crossDeg, kDeviceRegCrossTolDeg);
        return buf;
    }

    // Nearest orthogonal triad, by Gram-Schmidt off the two columns measured
    // laterally. The third is taken from the cross product rather than from its
    // own measurement — same direction to within crossDeg, and derived from the
    // better-conditioned pair — with the SIGN taken from the measurement, which
    // is the only thing the cross product cannot supply.
    const RegVec3 e1 = n1;
    const RegVec3 e2 = Unit(Sub(n2, Scale(e1, Dot(n2, e1))));
    const RegVec3 e3 = Scale(Unit(Cross(e1, e2)), r.handed);

    SetColumn(r, 0, e1);
    SetColumn(r, 1, e2);
    SetColumn(r, 2, e3);

    // t is whatever makes the fit pass through the point it was measured at:
    // t = p0 - R*h0. Rotate through a copy with a zero offset rather than
    // open-coding the multiply a second time.
    DeviceReg rotOnly = r;
    rotOnly.t = RegVec3{};
    r.t = Sub(p0, DeviceRegPredict(rotOnly, h0));

    r.valid = true;
    // A fresh fit has NOT been checked away from its own origin, and predicting
    // at h0 would return p0 by construction. Only a check pass at another pose
    // can set this.
    r.verified        = false;
    r.checkResidualMm = 0.0;
    r.checkedAt.clear();
    r.regAt = NowStamp();
    r.tipAt = r.regAt;
    return std::string();
}

std::string DeviceRegSetTip(DeviceReg& r, const RegVec3& h, const RegVec3& p)
{
    if (!r.valid)
        return "there is no registration to keep the rotation from - run the full "
               "registration first; a tip re-teach only moves t";

    DeviceReg tmp = r;
    tmp.t = RegVec3{};
    const RegVec3 rot = DeviceRegPredict(tmp, h);
    r.t = Sub(p, rot);
    r.tipAt = NowStamp();
    // The tip moved, so whatever the last check measured describes a tool that is
    // no longer on the machine. Saying "verified" about it would be a licence
    // issued for different tooling.
    r.verified        = false;
    r.checkResidualMm = 0.0;
    r.checkedAt.clear();
    return std::string();
}

bool DeviceRegRotationOk(const DeviceReg& r, const RegVec3& liveRot, std::string* why)
{
    if (!r.hasRot) return true;   // no U/V/W to disagree about
    const double du = std::fabs(liveRot.x - r.taughtRot.x);
    const double dv = std::fabs(liveRot.y - r.taughtRot.y);
    const double dw = std::fabs(liveRot.z - r.taughtRot.z);
    if (du <= kDeviceRegRotTolDeg && dv <= kDeviceRegRotTolDeg &&
        dw <= kDeviceRegRotTolDeg)
        return true;
    if (why) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "'%s' is rotated to (%.3f, %.3f, %.3f) but the registration was "
                      "taught at (%.3f, %.3f, %.3f) - the tip swings about the pivot "
                      "and this fit says nothing about a pose it never saw",
                      r.device.c_str(), liveRot.x, liveRot.y, liveRot.z,
                      r.taughtRot.x, r.taughtRot.y, r.taughtRot.z);
        *why = buf;
    }
    return false;
}

// ---- Persistence -----------------------------------------------------------

nlohmann::json DeviceRegToJson(const DeviceReg& r)
{
    nlohmann::json j;
    j["device"]     = r.device;
    j["ref_device"] = r.refDevice;
    j["valid"]      = r.valid;
    j["verified"]   = r.verified;

    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            j["rotation"][std::string("R") + static_cast<char>('1' + i) +
                          static_cast<char>('1' + k)] = r.R[i][k];
    j["offset"] = Vec3ToJson(r.t);

    j["measured"]["jog_mm"] = r.jogMm;
    j["measured"]["h0"]     = Vec3ToJson(r.h0);
    j["measured"]["p0"]     = Vec3ToJson(r.p0);
    j["measured"]["p1"]     = Vec3ToJson(r.p1);
    j["measured"]["p2"]     = Vec3ToJson(r.p2);
    j["measured"]["p3"]     = Vec3ToJson(r.p3);
    j["measured"]["col_norm"]  = {r.colNorm[0], r.colNorm[1], r.colNorm[2]};
    j["measured"]["ortho_deg"] = {r.orthoDeg[0], r.orthoDeg[1], r.orthoDeg[2]};
    j["measured"]["cross_deg"] = r.crossDeg;
    j["measured"]["handed"]    = r.handed;

    j["taught_rotation"]     = Vec3ToJson(r.taughtRot);
    j["has_rotation"]        = r.hasRot;

    j["check"]["residual_mm"] = r.checkResidualMm;
    j["check"]["pose"]        = Vec3ToJson(r.checkPose);
    j["check"]["at"]          = r.checkedAt;

    j["registered_at"] = r.regAt;
    j["tip_taught_at"] = r.tipAt;
    return j;
}

DeviceReg DeviceRegFromJson(const nlohmann::json& j)
{
    DeviceReg r;
    if (!j.is_object()) return r;
    r.device    = j.value("device", std::string());
    r.refDevice = j.value("ref_device", std::string("gantry-main"));
    r.valid     = j.value("valid", false);
    r.verified  = j.value("verified", false);

    if (j.contains("rotation") && j["rotation"].is_object()) {
        const nlohmann::json& m = j["rotation"];
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k) {
                const std::string key = std::string("R") + static_cast<char>('1' + i) +
                                        static_cast<char>('1' + k);
                r.R[i][k] = m.value(key, (i == k) ? 1.0 : 0.0);
            }
    }
    if (j.contains("offset")) r.t = Vec3FromJson(j["offset"]);

    if (j.contains("measured") && j["measured"].is_object()) {
        const nlohmann::json& m = j["measured"];
        r.jogMm = m.value("jog_mm", 0.0);
        if (m.contains("h0")) r.h0 = Vec3FromJson(m["h0"]);
        if (m.contains("p0")) r.p0 = Vec3FromJson(m["p0"]);
        if (m.contains("p1")) r.p1 = Vec3FromJson(m["p1"]);
        if (m.contains("p2")) r.p2 = Vec3FromJson(m["p2"]);
        if (m.contains("p3")) r.p3 = Vec3FromJson(m["p3"]);
        if (m.contains("col_norm") && m["col_norm"].is_array() &&
            m["col_norm"].size() == 3)
            for (int i = 0; i < 3; ++i) r.colNorm[i] = m["col_norm"][i].get<double>();
        if (m.contains("ortho_deg") && m["ortho_deg"].is_array() &&
            m["ortho_deg"].size() == 3)
            for (int i = 0; i < 3; ++i) r.orthoDeg[i] = m["ortho_deg"][i].get<double>();
        r.crossDeg = m.value("cross_deg", 0.0);
        r.handed   = m.value("handed", 1.0);
    }

    if (j.contains("taught_rotation")) r.taughtRot = Vec3FromJson(j["taught_rotation"]);
    r.hasRot = j.value("has_rotation", false);

    if (j.contains("check") && j["check"].is_object()) {
        r.checkResidualMm = j["check"].value("residual_mm", 0.0);
        if (j["check"].contains("pose")) r.checkPose = Vec3FromJson(j["check"]["pose"]);
        r.checkedAt = j["check"].value("at", std::string());
    }

    r.regAt = j.value("registered_at", std::string());
    r.tipAt = j.value("tip_taught_at", std::string());
    return r;
}

std::map<std::string, DeviceReg> LoadDeviceRegsFile(const std::string& path)
{
    std::map<std::string, DeviceReg> out;
    std::ifstream f(path);
    if (!f.is_open()) return out;
    try {
        nlohmann::json j;
        f >> j;
        // An object with a "registrations" array, so the file has room to grow a
        // version or a comment without every reader having to change shape.
        const nlohmann::json* arr = nullptr;
        if (j.is_array()) arr = &j;
        else if (j.is_object() && j.contains("registrations") &&
                 j["registrations"].is_array())
            arr = &j["registrations"];
        if (!arr) return out;
        for (const nlohmann::json& e : *arr) {
            const DeviceReg r = DeviceRegFromJson(e);
            if (r.device.empty()) continue;
            out[r.device] = r;
        }
    } catch (const std::exception& e) {
        FD_WARN("devreg", "device registrations %s are malformed (%s) - ignoring them",
                path.c_str(), e.what());
        return {};
    }
    return out;
}

std::map<std::string, DeviceReg> LoadDeviceRegs(std::string* pathUsed)
{
    if (pathUsed) pathUsed->clear();

    std::ifstream probe(kLocalPath);
    const bool haveLocal = probe.is_open();
    probe.close();

    const char* path = haveLocal ? kLocalPath : kBasePath;
    std::map<std::string, DeviceReg> regs = LoadDeviceRegsFile(path);
    if (pathUsed) *pathUsed = path;
    return regs;
}

bool SaveDeviceRegsFile(const std::string& path,
                        const std::map<std::string, DeviceReg>& regs)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("devreg", "device registration save failed: cannot open %s",
                path.c_str());
        return false;
    }
    nlohmann::json j;
    j["registrations"] = nlohmann::json::array();
    for (const auto& [name, r] : regs) {
        (void)name;
        j["registrations"].push_back(DeviceRegToJson(r));
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
