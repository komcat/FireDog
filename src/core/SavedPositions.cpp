// SavedPositions.cpp
#include "core/SavedPositions.h"

#include "core/Log.h"
#include "core/Paths.h"
#include "core/Product.h"   // GenerateUid — backfill for rows without one

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
std::string LocalPath() { return MachinePath("config/positions.local.json"); }
constexpr const char* kBasePath = "config/positions.json";   // shipped default, beside the exe

SavedPosMap ParsePositions(std::ifstream& f, const std::string& path, std::string* uidOut)
{
    SavedPosMap out;
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", "");
        if (!j.contains("positions") || !j["positions"].is_object()) return out;
        size_t total = 0;
        for (const auto& [dev, rows] : j["positions"].items()) {
            if (!rows.is_array()) continue;
            std::vector<SavedPos>& list = out[dev];
            for (const auto& row : rows) {
                if (!row.is_object()) continue;
                SavedPos p;
                const std::string name = row.value("name", "Pos");
                std::snprintf(p.name, sizeof(p.name), "%s", name.c_str());
                p.uid    = row.value("uid", "");
                if (p.uid.empty()) p.uid = GenerateUid();   // legacy row — assign once
                p.hidden = row.value("hidden", false);
                if (row.contains("axes") && row["axes"].is_object()) {
                    for (const auto& [axis, v] : row["axes"].items()) {
                        if (v.is_number()) p.vals[axis] = v.get<double>();
                    }
                }
                list.push_back(std::move(p));
                ++total;
            }
        }
        FD_INFO("saved", "loaded %zu position(s) for %zu device(s) from %s",
                total, out.size(), path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("saved", "positions load failed (%s): starting empty", e.what());
        return SavedPosMap{};
    }
    return out;
}

} // namespace

SavedPosMap LoadSavedPositions()
{
    std::string path = LocalPath();
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return SavedPosMap{};   // first run / no file — start empty
    }
    return ParsePositions(f, path, nullptr);
}

SavedPosMap LoadSavedPositionsFile(const std::string& path, std::string* uidOut)
{
    if (uidOut) uidOut->clear();
    std::ifstream f(path);
    if (!f.is_open()) return SavedPosMap{};
    return ParsePositions(f, path, uidOut);
}

bool SaveSavedPositionsFile(const std::string& path, const SavedPosMap& positions,
                            const std::string& uid)
{
    nlohmann::json devs = nlohmann::json::object();
    for (const auto& [dev, list] : positions) {
        nlohmann::json rows = nlohmann::json::array();
        for (const SavedPos& p : list) {
            nlohmann::json row;
            row["uid"]  = p.uid;
            row["name"] = p.name;
            if (p.hidden) row["hidden"] = true;
            row["axes"] = p.vals;
            rows.push_back(std::move(row));
        }
        devs[dev] = std::move(rows);
    }
    nlohmann::json j;
    if (!uid.empty()) j["uid"] = uid;
    j["positions"] = std::move(devs);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("saved", "positions save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

bool SaveSavedPositions(const SavedPosMap& positions)
{
    return SaveSavedPositionsFile(LocalPath(), positions, {});
}

const SavedPos* NearestSavedPos(const std::vector<SavedPos>& positions,
                                const std::map<std::string, double>& pose,
                                double* distMM, bool includeHidden)
{
    const SavedPos* best = nullptr;
    double bestD2 = 0.0;
    for (const SavedPos& p : positions) {
        if (p.hidden && !includeHidden) continue;
        double d2 = 0.0;
        int shared = 0;
        for (const auto& [axis, v] : p.vals) {
            const auto it = pose.find(axis);
            if (it == pose.end()) continue;
            const double d = v - it->second;
            d2 += d * d;
            ++shared;
        }
        if (shared == 0) continue;   // nothing in common — not a candidate
        if (!best || d2 < bestD2) { bestD2 = d2; best = &p; }
    }
    if (distMM) *distMM = best ? std::sqrt(bestD2) : 0.0;
    return best;
}

} // namespace firedog
