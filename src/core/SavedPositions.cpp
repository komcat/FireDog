// SavedPositions.cpp
#include "core/SavedPositions.h"

#include "core/Log.h"
#include "core/Paths.h"
#include "core/Product.h"   // GenerateUid — backfill for rows without one

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
std::string LocalPath() { return MachinePath("config/positions.local.json"); }
constexpr const char* kBasePath = "config/positions.json";   // shipped default, beside the exe

// "axes": {axis: number} -> map; non-numbers dropped.
void ParseAxes(const nlohmann::json& obj, std::map<std::string, double>& vals)
{
    if (!obj.contains("axes") || !obj["axes"].is_object()) return;
    for (const auto& [axis, v] : obj["axes"].items())
        if (v.is_number()) vals[axis] = v.get<double>();
}

SavedPosMap ParsePositions(std::ifstream& f, const std::string& path, std::string* uidOut,
                           std::vector<std::string>* variantsOut)
{
    SavedPosMap out;
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", "");
        if (variantsOut && j.contains("variants") && j["variants"].is_array())
            for (const auto& v : j["variants"])
                if (v.is_string() && !v.get<std::string>().empty() &&
                    std::find(variantsOut->begin(), variantsOut->end(), v.get<std::string>()) ==
                        variantsOut->end())
                    variantsOut->push_back(v.get<std::string>());
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
                p.hidden   = row.value("hidden", false);
                p.untaught = row.value("untaught", false);
                ParseAxes(row, p.vals);
                if (row.contains("variants") && row["variants"].is_object()) {
                    for (const auto& [vname, vrow] : row["variants"].items()) {
                        if (vname.empty() || !vrow.is_object()) continue;
                        SavedPosVariant var;
                        var.untaught = vrow.value("untaught", false);
                        ParseAxes(vrow, var.vals);
                        p.variants[vname] = std::move(var);
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

const std::map<std::string, double>& EffectiveVals(const SavedPos& p, const std::string& variant)
{
    if (!variant.empty()) {
        const auto it = p.variants.find(variant);
        if (it != p.variants.end()) return it->second.vals;
    }
    return p.vals;
}

bool EffectiveUntaught(const SavedPos& p, const std::string& variant)
{
    if (!variant.empty()) {
        const auto it = p.variants.find(variant);
        if (it != p.variants.end()) return it->second.untaught;
    }
    return p.untaught;
}

bool HasVariant(const SavedPos& p, const std::string& variant)
{
    return !variant.empty() && p.variants.count(variant) != 0;
}

SavedPosMap ApplyPositionVariant(const SavedPosMap& positions, const std::string& variant)
{
    SavedPosMap out;
    for (const auto& [dev, list] : positions) {
        std::vector<SavedPos>& dst = out[dev];
        dst.reserve(list.size());
        for (const SavedPos& p : list) {
            SavedPos q;
            std::snprintf(q.name, sizeof(q.name), "%s", p.name);
            q.uid      = p.uid;
            q.hidden   = p.hidden;
            q.vals     = EffectiveVals(p, variant);
            q.untaught = EffectiveUntaught(p, variant);
            dst.push_back(std::move(q));
        }
    }
    return out;
}

std::string UntaughtPositionMessage(const std::string& device, const SavedPos& p)
{
    return "position '" + std::string(p.name) + "' on " + device +
           " is not taught on this machine (a placeholder from a POR deploy) - teach it "
           "first: the deck's Teach run, or Update on the Motion page";
}

SavedPosMap LoadSavedPositions(std::vector<std::string>* variantsOut)
{
    if (variantsOut) variantsOut->clear();
    std::string path = LocalPath();
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return SavedPosMap{};   // first run / no file — start empty
    }
    return ParsePositions(f, path, nullptr, variantsOut);
}

SavedPosMap LoadSavedPositionsFile(const std::string& path, std::string* uidOut,
                                   std::vector<std::string>* variantsOut)
{
    if (uidOut) uidOut->clear();
    if (variantsOut) variantsOut->clear();
    std::ifstream f(path);
    if (!f.is_open()) return SavedPosMap{};
    return ParsePositions(f, path, uidOut, variantsOut);
}

bool SaveSavedPositionsFile(const std::string& path, const SavedPosMap& positions,
                            const std::string& uid, const std::vector<std::string>& variants)
{
    nlohmann::json devs = nlohmann::json::object();
    for (const auto& [dev, list] : positions) {
        nlohmann::json rows = nlohmann::json::array();
        for (const SavedPos& p : list) {
            nlohmann::json row;
            row["uid"]  = p.uid;
            row["name"] = p.name;
            if (p.hidden)   row["hidden"]   = true;
            if (p.untaught) row["untaught"] = true;
            row["axes"] = p.vals;
            if (!p.variants.empty()) {
                nlohmann::json vs = nlohmann::json::object();
                for (const auto& [vname, var] : p.variants) {
                    nlohmann::json vrow;
                    if (var.untaught) vrow["untaught"] = true;
                    vrow["axes"] = var.vals;
                    vs[vname] = std::move(vrow);
                }
                row["variants"] = std::move(vs);
            }
            rows.push_back(std::move(row));
        }
        devs[dev] = std::move(rows);
    }
    nlohmann::json j;
    if (!uid.empty()) j["uid"] = uid;
    if (!variants.empty()) j["variants"] = variants;
    j["positions"] = std::move(devs);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("saved", "positions save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

bool SaveSavedPositions(const SavedPosMap& positions, const std::vector<std::string>& variants)
{
    return SaveSavedPositionsFile(LocalPath(), positions, {}, variants);
}

const SavedPos* NearestSavedPos(const std::vector<SavedPos>& positions,
                                const std::map<std::string, double>& pose,
                                double* distMM, bool includeHidden)
{
    const SavedPos* best = nullptr;
    double bestD2 = 0.0;
    for (const SavedPos& p : positions) {
        if (p.hidden && !includeHidden) continue;
        if (p.untaught) continue;   // another machine's numbers
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
