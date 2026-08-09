// NeedleCal.cpp
#include "core/NeedleCal.h"

#include "core/Log.h"

#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/needle_cal.local.json";
constexpr const char* kBasePath  = "config/needle_cal.json";
}   // namespace

const char* NeedleCalLocalPath() { return kLocalPath; }

nlohmann::json NeedleCalToJson(const NeedleCal& c)
{
    nlohmann::json j;
    j["valid"]     = c.valid;
    j["device"]    = c.device;
    j["slide"]     = c.slide;
    j["io_device"] = c.ioDevice;
    j["shot_pin"]  = c.shotPin;

    j["offset"]["dx_mm"] = c.dx;
    j["offset"]["dy_mm"] = c.dy;
    j["offset"]["dz_mm"] = c.dz;

    j["measured"]["camera"]["x"] = c.camX;
    j["measured"]["camera"]["y"] = c.camY;
    j["measured"]["camera"]["z"] = c.camZ;
    j["measured"]["needle"]["x"] = c.ndlX;
    j["measured"]["needle"]["y"] = c.ndlY;
    j["measured"]["needle"]["z"] = c.ndlZ;
    j["measured"]["imprint"]["x"] = c.dotX;
    j["measured"]["imprint"]["y"] = c.dotY;

    j["residual_mm"]   = c.residualMm;
    j["verified"]      = c.verified;
    j["calibrated_at"] = c.calibratedAt;
    return j;
}

NeedleCal NeedleCalFromJson(const nlohmann::json& j)
{
    NeedleCal c;
    if (!j.is_object()) return c;
    c.valid    = j.value("valid", false);
    c.device   = j.value("device", std::string("gantry-main"));
    c.slide    = j.value("slide", std::string("Dispenser_Head"));
    c.ioDevice = j.value("io_device", std::string("IOBottom"));
    c.shotPin  = j.value("shot_pin", std::string("Dispenser_Shot"));

    if (j.contains("offset") && j["offset"].is_object()) {
        const nlohmann::json& o = j["offset"];
        c.dx = o.value("dx_mm", 0.0);
        c.dy = o.value("dy_mm", 0.0);
        c.dz = o.value("dz_mm", 0.0);
    }
    if (j.contains("measured") && j["measured"].is_object()) {
        const nlohmann::json& m = j["measured"];
        if (m.contains("camera") && m["camera"].is_object()) {
            c.camX = m["camera"].value("x", 0.0);
            c.camY = m["camera"].value("y", 0.0);
            c.camZ = m["camera"].value("z", 0.0);
        }
        if (m.contains("needle") && m["needle"].is_object()) {
            c.ndlX = m["needle"].value("x", 0.0);
            c.ndlY = m["needle"].value("y", 0.0);
            c.ndlZ = m["needle"].value("z", 0.0);
        }
        if (m.contains("imprint") && m["imprint"].is_object()) {
            c.dotX = m["imprint"].value("x", 0.0);
            c.dotY = m["imprint"].value("y", 0.0);
        }
    }
    c.residualMm   = j.value("residual_mm", 0.0);
    c.verified     = j.value("verified", false);
    c.calibratedAt = j.value("calibrated_at", std::string());
    return c;
}

NeedleCal LoadNeedleCalFile(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) return NeedleCal{};
    try {
        nlohmann::json j;
        f >> j;
        return NeedleCalFromJson(j);
    } catch (const std::exception& e) {
        FD_WARN("needle", "needle calibration %s is malformed (%s) - ignoring it",
                path.c_str(), e.what());
        return NeedleCal{};
    }
}

NeedleCal LoadNeedleCal(std::string* pathUsed)
{
    if (pathUsed) pathUsed->clear();

    std::ifstream probe(kLocalPath);
    const bool haveLocal = probe.is_open();
    probe.close();

    const char* path = haveLocal ? kLocalPath : kBasePath;
    NeedleCal c = LoadNeedleCalFile(path);
    if (pathUsed) *pathUsed = path;
    return c;
}

bool SaveNeedleCalFile(const std::string& path, const NeedleCal& c)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("needle", "needle calibration save failed: cannot open %s", path.c_str());
        return false;
    }
    f << NeedleCalToJson(c).dump(2) << '\n';
    return true;
}

} // namespace firedog
