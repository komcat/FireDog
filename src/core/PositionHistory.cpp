// PositionHistory.cpp
#include "core/PositionHistory.h"

#include "core/Log.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <exception>
#include <fstream>
#include <map>
#include <string>

namespace firedog {

namespace {

// UTC ISO-8601 with milliseconds (same shape as the DUT record's stamps).
std::string NowStamp()
{
    using namespace std::chrono;
    const auto   now = system_clock::now();
    const time_t t   = system_clock::to_time_t(now);
    const int    ms  = static_cast<int>(
        duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tm{};
    gmtime_s(&tm, &t);
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                  tm.tm_min, tm.tm_sec, ms);
    return buf;
}

bool SameVals(const std::map<std::string, double>& a, const std::map<std::string, double>& b)
{
    if (a.size() != b.size()) return false;
    for (const auto& [axis, v] : a) {
        const auto it = b.find(axis);
        if (it == b.end() || std::fabs(it->second - v) > 1e-6) return false;
    }
    return true;
}

nlohmann::json ValsToJson(const std::map<std::string, double>& v)
{
    nlohmann::json j = nlohmann::json::object();
    for (const auto& [axis, x] : v) j[axis] = x;
    return j;
}

std::map<std::string, double> ValsFromJson(const nlohmann::json& j)
{
    std::map<std::string, double> v;
    if (j.is_object()) {
        for (const auto& [axis, x] : j.items()) if (x.is_number()) v[axis] = x.get<double>();
    }
    return v;
}

} // namespace

std::vector<PosChange> DiffPositions(const SavedPosMap& before, const SavedPosMap& after,
                                     const std::string& by, const std::string& source)
{
    std::vector<PosChange> out;
    const std::string      at = NowStamp();
    auto stamp = [&](PosChange& c) {
        c.at     = at;
        c.by     = by;
        c.source = source;
    };

    // Adds, updates, renames: walk the new map.
    for (const auto& [dev, rows] : after) {
        const auto oldDev = before.find(dev);
        for (const SavedPos& np : rows) {
            if (np.hidden) continue;
            const SavedPos* op = nullptr;
            if (oldDev != before.end()) {
                for (const SavedPos& e : oldDev->second)
                    if (e.uid == np.uid) { op = &e; break; }
            }
            if (!op || op->hidden) {   // new, or just revealed: an add either way
                PosChange c;
                stamp(c);
                c.action = "add";
                c.device = dev;
                c.uid    = np.uid;
                c.name   = np.name;
                c.after  = np.vals;
                out.push_back(std::move(c));
                continue;
            }
            if (!SameVals(op->vals, np.vals)) {
                PosChange c;
                stamp(c);
                c.action = "update";
                c.device = dev;
                c.uid    = np.uid;
                c.name   = np.name;
                c.before = op->vals;
                c.after  = np.vals;
                out.push_back(std::move(c));
            }
            if (std::string(op->name) != np.name) {
                PosChange c;
                stamp(c);
                c.action     = "rename";
                c.device     = dev;
                c.uid        = np.uid;
                c.name       = np.name;
                c.nameBefore = op->name;
                out.push_back(std::move(c));
            }
        }
    }
    // Deletes: a visible row of the old map with no row of that uid now.
    for (const auto& [dev, rows] : before) {
        const auto newDev = after.find(dev);
        for (const SavedPos& op : rows) {
            if (op.hidden) continue;
            bool still = false;
            if (newDev != after.end()) {
                for (const SavedPos& e : newDev->second)
                    if (e.uid == op.uid) { still = !e.hidden; break; }
            }
            if (still) continue;
            PosChange c;
            stamp(c);
            c.action = "delete";
            c.device = dev;
            c.uid    = op.uid;
            c.name   = op.name;
            c.before = op.vals;
            out.push_back(std::move(c));
        }
    }
    return out;
}

bool AppendPositionHistory(const std::string& path, const std::vector<PosChange>& entries)
{
    if (entries.empty()) return true;
    std::ofstream f(path, std::ios::app | std::ios::binary);
    if (!f) {
        FD_WARN("saved", "cannot append position history to %s", path.c_str());
        return false;
    }
    for (const PosChange& c : entries) {
        nlohmann::json j;
        j["at"]     = c.at;
        j["by"]     = c.by;
        j["source"] = c.source;
        j["action"] = c.action;
        j["device"] = c.device;
        j["uid"]    = c.uid;
        j["name"]   = c.name;
        if (!c.nameBefore.empty()) j["name_before"] = c.nameBefore;
        if (!c.before.empty())     j["before"]      = ValsToJson(c.before);
        if (!c.after.empty())      j["after"]       = ValsToJson(c.after);
        f << j.dump() << '\n';
        FD_INFO("saved", "history: %s '%s' (%s, %s) by %s via %s", c.action.c_str(),
                c.name.c_str(), c.device.c_str(), c.uid.c_str(),
                c.by.empty() ? "-" : c.by.c_str(), c.source.c_str());
    }
    return static_cast<bool>(f);
}

std::vector<PosChange> LoadPositionHistory(const std::string& path)
{
    std::vector<PosChange> out;
    std::ifstream f(path, std::ios::binary);
    if (!f) return out;
    std::string line;
    int         lineNo = 0;
    while (std::getline(f, line)) {
        ++lineNo;
        if (line.empty() || line == "\r") continue;
        try {
            const nlohmann::json j = nlohmann::json::parse(line);
            PosChange c;
            c.at         = j.value("at", "");
            c.by         = j.value("by", "");
            c.source     = j.value("source", "");
            c.action     = j.value("action", "");
            c.device     = j.value("device", "");
            c.uid        = j.value("uid", "");
            c.name       = j.value("name", "");
            c.nameBefore = j.value("name_before", "");
            if (j.contains("before")) c.before = ValsFromJson(j["before"]);
            if (j.contains("after"))  c.after  = ValsFromJson(j["after"]);
            out.push_back(std::move(c));
        } catch (const std::exception& e) {
            FD_WARN("saved", "%s:%d: bad history line skipped (%s)", path.c_str(), lineNo,
                    e.what());
        }
    }
    return out;
}

std::string PositionHistoryPathFor(const std::string& positionsPath)
{
    const std::string tail = ".json";
    if (positionsPath.size() >= tail.size() &&
        positionsPath.compare(positionsPath.size() - tail.size(), tail.size(), tail) == 0)
        return positionsPath.substr(0, positionsPath.size() - tail.size()) + ".history.jsonl";
    return positionsPath + ".history.jsonl";
}

} // namespace firedog
