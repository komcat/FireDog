// MosaicConfig.cpp
#include "core/MosaicConfig.h"

#include "core/Log.h"
#include "core/Paths.h"
#include "core/Product.h"   // GenerateUid

#include <algorithm>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kBasePath = "config/mosaic.json";   // shipped default, beside the exe

// Pictures are production output, so an unset folder means the data root's.
std::string DefaultSaveDir() { return DataPath("mosaic"); }

nlohmann::json CornerToJson(const MosaicCorner& c)
{
    nlohmann::json j;
    j["have"] = c.have;
    j["x"]    = c.x;
    j["y"]    = c.y;
    j["z"]    = c.z;
    if (!c.taughtAt.empty()) j["taught_at"] = c.taughtAt;
    return j;
}

MosaicCorner CornerFromJson(const nlohmann::json& j)
{
    MosaicCorner c;
    if (!j.is_object()) return c;
    c.have     = j.value("have", false);
    c.x        = j.value("x", 0.0);
    c.y        = j.value("y", 0.0);
    c.z        = j.value("z", 0.0);
    c.taughtAt = j.value("taught_at", std::string());
    return c;
}
}   // namespace

const MosaicArea* FindMosaicArea(const std::vector<MosaicArea>& list, const std::string& uidOrName)
{
    if (uidOrName.empty()) return nullptr;
    for (const MosaicArea& a : list)
        if (a.uid == uidOrName) return &a;
    for (const MosaicArea& a : list)
        if (a.name == uidOrName) return &a;
    return nullptr;
}

MosaicArea* FindMosaicArea(std::vector<MosaicArea>& list, const std::string& uidOrName)
{
    return const_cast<MosaicArea*>(
        FindMosaicArea(static_cast<const std::vector<MosaicArea>&>(list), uidOrName));
}

const char* MosaicZModeName(MosaicZMode m)
{
    return m == MosaicZMode::Plane ? "plane" : "fixed";
}

bool MosaicZModeFromName(const std::string& s, MosaicZMode* out)
{
    if (s == "fixed") { *out = MosaicZMode::Fixed; return true; }
    if (s == "plane") { *out = MosaicZMode::Plane; return true; }
    return false;
}

const char* MosaicCornerLabel(int nCorners, int i)
{
    // Three corners follow the waffle tray: the fourth is derived from them, so
    // WHICH corner each one is matters. Four go round the edge in order.
    if (nCorners == 3) {
        switch (i) {
        case 0:  return "C1 origin";
        case 1:  return "C2 row end";
        case 2:  return "C3 far corner";
        default: return "C4 (derived)";
        }
    }
    switch (i) {
    case 0:  return "C1";
    case 1:  return "C2";
    case 2:  return "C3";
    default: return "C4";
    }
}

nlohmann::json MosaicAreaToJson(const MosaicArea& a)
{
    nlohmann::json j;
    j["uid"]       = a.uid;
    j["name"]      = a.name;
    j["n_corners"] = a.nCorners;
    nlohmann::json cs = nlohmann::json::array();
    for (int i = 0; i < kMosaicMaxCorners; ++i) cs.push_back(CornerToJson(a.corners[i]));
    j["corners"]      = cs;
    j["z_mode"]       = MosaicZModeName(a.zMode);
    j["z_fixed_mm"]   = a.zFixedMm;
    j["overlap_pct"]  = a.overlapPct;
    j["settle_ms"]    = a.settleMs;
    j["fresh_frames"] = a.freshFrames;
    j["velocity_mm_s"] = a.velocityMmS;
    return j;
}

MosaicArea MosaicAreaFromJson(const nlohmann::json& j)
{
    MosaicArea a;
    if (!j.is_object()) return a;
    a.uid      = j.value("uid", std::string());
    if (a.uid.empty()) a.uid = GenerateUid();
    a.name     = j.value("name", std::string("area"));
    a.nCorners = j.value("n_corners", 3) == 4 ? 4 : 3;
    if (j.contains("corners") && j["corners"].is_array()) {
        const nlohmann::json& cs = j["corners"];
        for (std::size_t i = 0; i < cs.size() && i < static_cast<std::size_t>(kMosaicMaxCorners); ++i)
            a.corners[i] = CornerFromJson(cs[i]);
    }
    MosaicZMode zm = MosaicZMode::Fixed;
    if (MosaicZModeFromName(j.value("z_mode", std::string("fixed")), &zm)) a.zMode = zm;
    a.zFixedMm    = j.value("z_fixed_mm", 0.0);
    a.overlapPct  = std::clamp(j.value("overlap_pct", 15.0), kMosaicMinOverlapPct,
                               kMosaicMaxOverlapPct);
    a.settleMs    = std::clamp(j.value("settle_ms", 0), 0, 10000);
    a.freshFrames = std::clamp(j.value("fresh_frames", 2), 0, 10);
    a.velocityMmS = std::max(0.0, j.value("velocity_mm_s", 0.0));
    return a;
}

nlohmann::json MosaicConfigToJson(const MosaicConfig& c)
{
    nlohmann::json j;
    // The default is stored as "" so it keeps following the data root if
    // FireDog.ini moves it; only a folder somebody chose is written out.
    j["save_dir"] = c.saveDir == DefaultSaveDir() ? std::string() : c.saveDir;
    nlohmann::json as = nlohmann::json::array();
    for (const MosaicArea& a : c.areas) as.push_back(MosaicAreaToJson(a));
    j["areas"] = as;
    nlohmann::json cal;
    cal["calibrated"]    = c.cal.calibrated;
    cal["mm_per_px"]     = {c.cal.m[0], c.cal.m[1], c.cal.m[2], c.cal.m[3]};
    cal["calibrated_at"] = c.cal.calibratedAt;
    cal["note"]          = c.cal.note;
    j["camera_cal"] = cal;
    return j;
}

MosaicConfig MosaicConfigFromJson(const nlohmann::json& j)
{
    MosaicConfig c;
    if (!j.is_object()) return c;
    c.saveDir = j.value("save_dir", c.saveDir);
    if (c.saveDir.empty()) c.saveDir = DefaultSaveDir();
    if (j.contains("areas") && j["areas"].is_array())
        for (const nlohmann::json& a : j["areas"]) c.areas.push_back(MosaicAreaFromJson(a));
    if (j.contains("camera_cal") && j["camera_cal"].is_object()) {
        const nlohmann::json& cal = j["camera_cal"];
        c.cal.calibrated = cal.value("calibrated", false);
        if (cal.contains("mm_per_px") && cal["mm_per_px"].is_array() &&
            cal["mm_per_px"].size() == 4) {
            for (std::size_t i = 0; i < 4; ++i)
                c.cal.m[i] = cal["mm_per_px"][i].is_number() ? cal["mm_per_px"][i].get<double>()
                                                             : 0.0;
        } else {
            c.cal.calibrated = false;   // a calibration without its matrix is none
        }
        c.cal.calibratedAt = cal.value("calibrated_at", std::string());
        c.cal.note         = cal.value("note", std::string());
    }
    return c;
}

// Machine state: under the machine root, never beside the exe (core/Paths.h).
std::string MosaicConfigLocalPath() { return MachinePath("config/mosaic.local.json"); }

MosaicConfig LoadMosaicConfig(std::string* pathUsed)
{
    if (pathUsed) pathUsed->clear();
    for (const std::string& path : {MosaicConfigLocalPath(), std::string(kBasePath)}) {
        std::ifstream f(path);
        if (!f.is_open()) continue;
        try {
            nlohmann::json j;
            f >> j;
            if (pathUsed) *pathUsed = path;
            return MosaicConfigFromJson(j);
        } catch (const std::exception& e) {
            FD_WARN("mosaic", "mosaic config %s is malformed (%s) - ignoring it", path.c_str(),
                    e.what());
        }
    }
    MosaicConfig c;
    c.saveDir = DefaultSaveDir();
    return c;
}

bool SaveMosaicConfig(const MosaicConfig& c)
{
    const std::string path = MosaicConfigLocalPath();
    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("mosaic", "mosaic config save failed: cannot open %s", path.c_str());
        return false;
    }
    f << MosaicConfigToJson(c).dump(2) << '\n';
    return true;
}

} // namespace firedog
