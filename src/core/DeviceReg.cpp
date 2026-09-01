// DeviceReg.cpp — the prediction, the two setters, and the file.
//
// See DeviceReg.h for where R comes from, why t is measured by imprint instead,
// and why the triad is never assumed right-handed.
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

RegVec3 Unit(const RegVec3& a)
{
    const double n = Norm(a);
    if (n <= 0.0) return RegVec3{};
    return Scale(a, 1.0 / n);
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
    // R^T, not an inverse: R is orthogonal by construction, and that stays true
    // when it is IMPROPER — which it usually is. See the header.
    return RegVec3{
        r.R[0][0] * d.x + r.R[1][0] * d.y + r.R[2][0] * d.z,
        r.R[0][1] * d.x + r.R[1][1] * d.y + r.R[2][1] * d.z,
        r.R[0][2] * d.x + r.R[1][2] * d.y + r.R[2][2] * d.z};
}

std::string DeviceRegSetRotation(DeviceReg& r, const RegVec3 cols[3],
                                 const std::string& source)
{
    if (!cols) return "no rotation columns";
    for (int j = 0; j < 3; ++j)
        if (Norm(cols[j]) <= 0.0) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "rotation column %d is zero - '%s' has no usable axis "
                          "mapping to the reference device", j + 1, r.device.c_str());
            return buf;
        }

    // Orthonormalise by Gram-Schmidt, and take the third column from the cross
    // product with the SIGN the input carried. The permutation matrices these
    // come from are already orthogonal, so this normally changes nothing — it is
    // here so that R^T stays a true inverse whatever arrives, and so a hand-
    // edited transforms file cannot make DeviceRegSolve quietly wrong.
    const RegVec3 e1 = Unit(cols[0]);
    RegVec3       e2 = Unit(Sub(cols[1], Scale(e1, Dot(cols[1], e1))));
    if (Norm(e2) <= 0.0)
        return "rotation columns 1 and 2 are parallel - the axis mapping is degenerate";
    const RegVec3 perp   = Unit(Cross(e1, e2));
    const double  handed = (Dot(perp, cols[2]) >= 0.0) ? 1.0 : -1.0;
    const RegVec3 e3     = Scale(perp, handed);

    SetColumn(r, 0, e1);
    SetColumn(r, 1, e2);
    SetColumn(r, 2, e3);
    r.rotSource = source;
    r.regAt     = NowStamp();
    // A rotation on its own predicts nothing: t is still whatever it was, and if
    // it was never measured the record must not read as usable.
    if (r.tipAt.empty()) r.valid = false;
    return std::string();
}

std::string DeviceRegSetTip(DeviceReg& r, const RegVec3& h, const RegVec3& p,
                            double spreadMm, int points)
{
    if (r.rotSource.empty())
        return "there is no rotation to hold fixed - adopt one for '" + r.device +
               "' first; an imprint measures where the tool touches, not how the "
               "device is turned";

    // t = p - R*h. Rotate through a copy with a zero offset rather than
    // open-coding the multiply a second time.
    DeviceReg rotOnly = r;
    rotOnly.t = RegVec3{};
    r.t = Sub(p, DeviceRegPredict(rotOnly, h));

    r.tipAt       = NowStamp();
    r.imprintPose = h;
    r.hasImprintPose = true;
    r.tipMeas     = p;
    r.tipSpreadMm = spreadMm;
    r.tipPoints   = points;
    r.valid       = true;

    // The tip moved, so whatever the last check measured described tooling that
    // may no longer be on the machine. Saying "verified" about it would be a
    // licence issued for a different tool.
    r.verified        = false;
    r.checkResidualMm = 0.0;
    r.checkedAt.clear();
    return std::string();
}

bool DeviceRegRotationOk(const DeviceReg& r, const RegVec3& liveRot, std::string* why)
{
    if (!r.hasRot) return true;   // no U/V/W to disagree about
    // The pivot is at the tip, so a rotation turns the tool about the very point
    // this record stores and does not move it. Nothing to guard. See the field's
    // comment in the header for why this is a switch and not an assumption.
    if (r.pivotAtTip) return true;
    const double du = std::fabs(liveRot.x - r.taughtRot.x);
    const double dv = std::fabs(liveRot.y - r.taughtRot.y);
    const double dw = std::fabs(liveRot.z - r.taughtRot.z);
    if (du <= kDeviceRegRotTolDeg && dv <= kDeviceRegRotTolDeg &&
        dw <= kDeviceRegRotTolDeg)
        return true;
    if (why) {
        // Zero prints as zero. An axis resting on zero reports a value either
        // side of it, and "%.3f" turns that into -0.000 half the time - which
        // reads as a real negative rotation to anybody who has not been told
        // otherwise, and made this message change from poll to poll.
        const auto tidy = [](double v) { return std::fabs(v) < 5e-4 ? 0.0 : v; };
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "'%s' is rotated to (%.3f, %.3f, %.3f) but the tip was taught "
                      "at (%.3f, %.3f, %.3f) - the tip swings about the pivot and "
                      "this record says nothing about a pose it never saw",
                      r.device.c_str(), tidy(liveRot.x), tidy(liveRot.y), tidy(liveRot.z),
                      tidy(r.taughtRot.x), tidy(r.taughtRot.y), tidy(r.taughtRot.z));
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
    j["rotation_source"] = r.rotSource;
    j["offset"]          = Vec3ToJson(r.t);

    j["taught_rotation"] = Vec3ToJson(r.taughtRot);
    j["has_rotation"]    = r.hasRot;
    j["pivot_at_tip"]    = r.pivotAtTip;

    j["poses"]["imprint"]     = Vec3ToJson(r.imprintPose);
    j["poses"]["has_imprint"] = r.hasImprintPose;
    j["poses"]["home"]        = Vec3ToJson(r.homePose);
    j["poses"]["has_home"]    = r.hasHomePose;
    j["poses"]["view"]        = Vec3ToJson(r.tipView);
    j["poses"]["has_view"]    = r.hasTipView;

    j["tip"]["measured"]  = Vec3ToJson(r.tipMeas);
    j["tip"]["spread_mm"] = r.tipSpreadMm;
    j["tip"]["points"]    = r.tipPoints;

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
    r.rotSource = j.value("rotation_source", std::string());
    if (j.contains("offset")) r.t = Vec3FromJson(j["offset"]);

    if (j.contains("taught_rotation")) r.taughtRot = Vec3FromJson(j["taught_rotation"]);
    r.hasRot = j.value("has_rotation", false);
    // Absent in files written before the field existed. TRUE is the right
    // default for those: they were written on a machine whose pivots ARE at
    // the tips, and the guard they predate was blanking their tips.
    r.pivotAtTip = j.value("pivot_at_tip", true);

    if (j.contains("poses") && j["poses"].is_object()) {
        const nlohmann::json& ps = j["poses"];
        if (ps.contains("imprint")) r.imprintPose = Vec3FromJson(ps["imprint"]);
        if (ps.contains("home"))    r.homePose    = Vec3FromJson(ps["home"]);
        if (ps.contains("view"))    r.tipView     = Vec3FromJson(ps["view"]);
        r.hasImprintPose = ps.value("has_imprint", false);
        r.hasHomePose    = ps.value("has_home", false);
        r.hasTipView     = ps.value("has_view", false);
    }

    if (j.contains("tip") && j["tip"].is_object()) {
        const nlohmann::json& tp = j["tip"];
        if (tp.contains("measured")) r.tipMeas = Vec3FromJson(tp["measured"]);
        r.tipSpreadMm = tp.value("spread_mm", 0.0);
        r.tipPoints   = tp.value("points", 0);
    }

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
