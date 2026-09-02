// GripProfile.cpp
#include "core/GripProfile.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid

#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/grips.local.json";
constexpr const char* kBasePath  = "config/grips.json";

std::vector<GripProfile> ParseGripProfiles(std::ifstream& f, const std::string& path,
                                           std::string* uidOut)
{
    std::vector<GripProfile> out;
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", "");
        if (!j.contains("grips") || !j["grips"].is_array()) return out;
        for (const auto& row : j["grips"]) {
            if (!row.is_object()) continue;
            out.push_back(GripProfileFromJson(row));
        }
        FD_INFO("grip", "loaded %zu gripped-object profile(s) from %s", out.size(),
                path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("grip", "grip profiles load failed (%s): starting empty", e.what());
        return std::vector<GripProfile>{};
    }
    return out;
}

} // namespace

// ---- lookup ----------------------------------------------------------------

const GripProfile* FindGripProfile(const std::vector<GripProfile>& list,
                                   const std::string& uidOrName)
{
    if (uidOrName.empty()) return nullptr;
    for (const GripProfile& g : list)
        if (g.uid == uidOrName) return &g;
    for (const GripProfile& g : list)
        if (g.name == uidOrName) return &g;
    return nullptr;
}

GripProfile* FindGripProfile(std::vector<GripProfile>& list, const std::string& uidOrName)
{
    const std::vector<GripProfile>& c = list;
    return const_cast<GripProfile*>(FindGripProfile(c, uidOrName));
}

// ---- the verdict -----------------------------------------------------------

bool GripVerdict(const GripProfile& g, double dx, double dy, std::string* why)
{
    // No reference means nothing has ever said what a good grip looks like for
    // this object. Reporting the offset is still useful; calling it a PASS is
    // not, and a default pass is how an unchecked profile ends up trusted.
    if (!g.haveRef) {
        if (why)
            *why = "no reference offset has been adopted for '" + g.name +
                   "' - measure a known-good grip on the Gripped Object page and "
                   "adopt it, or read the offset without a verdict";
        return false;
    }

    const double ex = dx - g.refDx;
    const double ey = dy - g.refDy;
    const double d  = std::sqrt(ex * ex + ey * ey);
    if (d > g.tolMm) {
        if (why) {
            char b[224];
            std::snprintf(b, sizeof(b),
                          "the object sits %.4f mm from where a good grip puts it "
                          "(%+.4f, %+.4f), tolerance %.4f",
                          d, ex, ey, g.tolMm);
            *why = b;
        }
        return false;
    }
    if (why) why->clear();
    return true;
}

// ---- persistence -----------------------------------------------------------

nlohmann::json GripProfileToJson(const GripProfile& g)
{
    nlohmann::json j;
    j["uid"]     = g.uid;
    j["name"]    = g.name;
    j["device"]  = g.device;
    j["inspect"] = g.inspectUid;

    j["mode"]                 = g.mode;
    j["detector"]             = g.detector;
    j["auto_fallback_manual"] = g.autoFallbackManual;

    nlohmann::json ref;
    ref["have"]   = g.haveRef;
    ref["dx"]     = g.refDx;
    ref["dy"]     = g.refDy;
    ref["tol_mm"] = g.tolMm;
    ref["at"]     = g.taughtAt;
    j["reference"] = std::move(ref);

    // The last measurement is evidence, not state a run reads: kept so "which
    // number is wrong" survives a re-teach, exactly like DeviceReg::tipMeas.
    nlohmann::json last;
    last["dx"]      = g.lastDx;
    last["dy"]      = g.lastDy;
    last["at"]      = g.measuredAt;
    j["last"] = std::move(last);
    return j;
}

GripProfile GripProfileFromJson(const nlohmann::json& j)
{
    GripProfile g;
    if (!j.is_object()) return g;
    g.uid = j.value("uid", "");
    if (g.uid.empty()) g.uid = GenerateUid();   // legacy row - assign once
    g.name       = j.value("name", "object");
    g.device     = j.value("device", "");
    g.inspectUid = j.value("inspect", "");

    g.mode               = j.value("mode", "manual");
    g.detector           = j.value("detector", "");
    g.autoFallbackManual = j.value("auto_fallback_manual", true);

    if (j.contains("reference") && j["reference"].is_object()) {
        const auto& r = j["reference"];
        g.haveRef  = r.value("have", false);
        g.refDx    = r.value("dx", 0.0);
        g.refDy    = r.value("dy", 0.0);
        g.tolMm    = r.value("tol_mm", 0.05);
        g.taughtAt = r.value("at", "");
    }
    if (j.contains("last") && j["last"].is_object()) {
        const auto& l = j["last"];
        g.lastDx     = l.value("dx", 0.0);
        g.lastDy     = l.value("dy", 0.0);
        g.measuredAt = l.value("at", "");
    }
    return g;
}

std::vector<GripProfile> LoadGripProfiles()
{
    std::string path = kLocalPath;
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return std::vector<GripProfile>{};   // first run - empty
    }
    return ParseGripProfiles(f, path, nullptr);
}

std::vector<GripProfile> LoadGripProfilesFile(const std::string& path, std::string* uidOut)
{
    if (uidOut) uidOut->clear();
    std::ifstream f(path);
    if (!f.is_open()) return std::vector<GripProfile>{};
    return ParseGripProfiles(f, path, uidOut);
}

bool SaveGripProfilesFile(const std::string& path, const std::vector<GripProfile>& list,
                          const std::string& uid)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const GripProfile& g : list) arr.push_back(GripProfileToJson(g));

    nlohmann::json j;
    if (!uid.empty()) j["uid"] = uid;
    j["grips"] = std::move(arr);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("grip", "grip profiles save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
