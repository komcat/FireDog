// PivotDefaults.cpp
#include "core/PivotDefaults.h"

#include "core/Log.h"
#include "core/Paths.h"

#include <nlohmann/json.hpp>

#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kBasePath = "config/pivot.json";   // shipped default, beside the exe
std::string LocalPath() { return MachinePath("config/pivot.local.json"); }
} // namespace

PivotDefaults LoadPivotDefaults()
{
    std::string   path = LocalPath();
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return PivotDefaults{};   // none taught yet
    }
    PivotDefaults out;
    try {
        nlohmann::json j;
        f >> j;
        if (j.contains("pivots") && j["pivots"].is_object()) {
            for (const auto& [dev, v] : j["pivots"].items()) {
                if (!v.is_object()) continue;
                out[dev] = PivotDefault{v.value("x", 0.0), v.value("y", 0.0), v.value("z", 0.0)};
            }
        }
        FD_INFO("pivot", "loaded %zu default pivot(s) from %s", out.size(), path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("pivot", "%s is malformed (%s) - no default pivots", path.c_str(), e.what());
        return PivotDefaults{};
    }
    return out;
}

bool SavePivotDefaults(const PivotDefaults& d)
{
    nlohmann::json j;
    j["pivots"] = nlohmann::json::object();
    for (const auto& [dev, p] : d)
        j["pivots"][dev] = {{"x", p.x}, {"y", p.y}, {"z", p.z}};
    const std::string path = LocalPath();
    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("pivot", "default pivot save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
