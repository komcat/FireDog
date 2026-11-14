// CycleTimes.cpp
#include "core/CycleTimes.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>

namespace firedog {

namespace {

// Tail of a recipe's file read at first use: enough for kCycleWindow runs of
// every version anyone still cares about, bounded whatever the file has grown to.
constexpr std::streamoff kTailBytes = 256 * 1024;
// Records kept in memory per recipe (all versions together).
constexpr std::size_t    kKeep      = 500;

std::string HashHex(std::uint64_t h)
{
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

std::uint64_t ParseHex(const std::string& s)
{
    std::uint64_t h = 0;
    for (const char c : s) {
        int d = -1;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        if (d < 0) return 0;
        h = (h << 4) | static_cast<std::uint64_t>(d);
    }
    return h;
}

// A recipe uid as a file stem: used as it is when it is plain
// (A-Z a-z 0-9 - _), else a stable hash of it - a uid is never trusted as a path.
std::string FileStem(const std::string& uid)
{
    bool plain = !uid.empty() && uid.size() <= 64;
    for (const char c : uid) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_';
        if (!ok) { plain = false; break; }
    }
    if (plain) return uid;
    std::uint64_t h = 1469598103934665603ULL;
    for (const char c : uid) {
        h ^= static_cast<unsigned char>(c);
        h *= 1099511628211ULL;
    }
    return "u" + HashHex(h);
}

nlohmann::json ToJson(const CycleTimeRecord& r)
{
    nlohmann::json j;
    j["t"]          = r.t;
    j["ts"]         = r.ts;
    j["uid"]        = r.uid;
    j["name"]       = r.name;
    j["hash"]       = HashHex(r.hash);
    j["wall_ms"]    = r.wallMs;
    j["wait_ms"]    = r.waitMs;
    j["machine_ms"] = r.MachineMs();   // derived; written for whoever reads the file
    j["steps"]      = r.steps;
    j["serial"]     = r.serial;
    j["operator"]   = r.operatorId;
    return j;
}

bool FromJson(const nlohmann::json& j, CycleTimeRecord& r)
{
    if (!j.is_object()) return false;
    const auto wall = j.find("wall_ms");
    if (wall == j.end() || !wall->is_number()) return false;
    r.t          = j.value("t", std::int64_t{0});
    r.ts         = j.value("ts", std::string());
    r.uid        = j.value("uid", std::string());
    r.name       = j.value("name", std::string());
    r.hash       = ParseHex(j.value("hash", std::string()));
    r.wallMs     = wall->get<std::int64_t>();
    r.waitMs     = j.value("wait_ms", std::int64_t{0});
    r.steps      = j.value("steps", std::size_t{0});
    r.serial     = j.value("serial", std::string());
    r.operatorId = j.value("operator", std::string());
    return r.hash != 0;
}

} // namespace

CycleEstimate EstimateCycle(const std::vector<CycleTimeRecord>& recs, std::uint64_t hash,
                            int window)
{
    CycleEstimate e;
    if (hash == 0 || window <= 0) return e;
    std::vector<std::int64_t> machine, wait;
    for (auto it = recs.rbegin(); it != recs.rend(); ++it) {
        if (it->hash != hash) continue;
        machine.push_back(it->MachineMs());
        wait.push_back(it->waitMs);
        if (static_cast<int>(machine.size()) >= window) break;
    }
    if (machine.empty()) return e;
    const auto median = [](std::vector<std::int64_t>& v) {
        std::sort(v.begin(), v.end());
        const std::size_t m = v.size() / 2;
        return (v.size() % 2) ? v[m] : (v[m - 1] + v[m]) / 2;
    };
    e.n         = static_cast<int>(machine.size());
    e.machineMs = median(machine);   // sorts machine
    e.waitMs    = median(wait);
    if (machine.size() < 5) {
        e.loMs = machine.front();
        e.hiMs = machine.back();
    } else {
        const std::size_t last = machine.size() - 1;
        e.loMs = machine[last / 10];
        e.hiMs = machine[last - last / 10];
    }
    return e;
}

std::string CycleTimesDir(const std::string& productDataDir)
{
    if (productDataDir.empty()) return std::string();
    return (std::filesystem::path(productDataDir) / "cycle_times").string();
}

std::string FormatClockMs(std::int64_t ms)
{
    if (ms < 0) ms = 0;
    const std::int64_t s = (ms + 500) / 1000;
    char buf[32];
    if (s >= 3600)
        std::snprintf(buf, sizeof(buf), "%lld:%02lld:%02lld",
                      static_cast<long long>(s / 3600), static_cast<long long>((s / 60) % 60),
                      static_cast<long long>(s % 60));
    else
        std::snprintf(buf, sizeof(buf), "%lld:%02lld", static_cast<long long>(s / 60),
                      static_cast<long long>(s % 60));
    return buf;
}

void CycleTimeStore::SetDir(const std::string& dir)
{
    if (dir == dir_) return;
    dir_ = dir;
    byUid_.clear();
    est_.clear();
    ++seq_;
}

const std::vector<CycleTimeRecord>& CycleTimeStore::Load(const std::string& uid)
{
    namespace fs = std::filesystem;
    if (const auto f = byUid_.find(uid); f != byUid_.end()) return f->second;
    std::vector<CycleTimeRecord>& out = byUid_[uid];   // a miss is cached as empty
    if (dir_.empty() || uid.empty()) return out;
    try {
        const fs::path file = fs::path(dir_) / (FileStem(uid) + ".jsonl");
        std::ifstream f(file, std::ios::binary);
        if (!f.is_open()) return out;   // no history yet - not an error
        f.seekg(0, std::ios::end);
        const std::streamoff size = f.tellg();
        if (size <= 0) return out;
        const std::streamoff from = size > kTailBytes ? size - kTailBytes : 0;
        f.seekg(from);
        std::string buf(static_cast<std::size_t>(size - from), '\0');
        f.read(buf.data(), size - from);
        std::size_t start = 0;
        if (from > 0) {   // the first piece is a partial line
            const std::size_t nl = buf.find('\n');
            start = nl == std::string::npos ? buf.size() : nl + 1;
        }
        while (start < buf.size()) {
            std::size_t nl = buf.find('\n', start);
            if (nl == std::string::npos) nl = buf.size();
            std::string line = buf.substr(start, nl - start);
            start = nl + 1;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
            CycleTimeRecord r;
            if (!j.is_discarded() && FromJson(j, r)) out.push_back(std::move(r));
        }
        if (out.size() > kKeep)
            out.erase(out.begin(), out.end() - static_cast<std::ptrdiff_t>(kKeep));
    } catch (const std::exception&) {
        out.clear();   // an unreadable history is no history; the run is unaffected
    }
    return out;
}

bool CycleTimeStore::Append(const CycleTimeRecord& rec, std::string* err)
{
    namespace fs = std::filesystem;
    if (dir_.empty()) {
        if (err) *err = "no product data folder";
        return false;
    }
    if (rec.uid.empty()) {
        if (err) *err = "recipe has no uid";
        return false;
    }
    Load(rec.uid);   // the cache must hold the file's tail BEFORE this line joins it
    try {
        std::error_code ec;
        fs::create_directories(dir_, ec);
        if (ec) {
            if (err) *err = "cannot create " + dir_ + ": " + ec.message();
            return false;
        }
        const fs::path file = fs::path(dir_) / (FileStem(rec.uid) + ".jsonl");
        std::ofstream f(file, std::ios::app | std::ios::binary);
        if (!f.is_open()) {
            if (err) *err = "cannot open " + file.string();
            return false;
        }
        f << ToJson(rec).dump() << '\n';
        f.flush();
        if (!f.good()) {
            if (err) *err = "write failed for " + file.string();
            return false;
        }
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
    std::vector<CycleTimeRecord>& v = byUid_[rec.uid];
    v.push_back(rec);
    if (v.size() > kKeep) v.erase(v.begin());
    est_.clear();
    ++seq_;
    return true;
}

CycleEstimate CycleTimeStore::Estimate(const std::string& uid, std::uint64_t hash)
{
    const auto key = std::make_pair(uid, hash);
    if (const auto f = est_.find(key); f != est_.end()) return f->second;
    const CycleEstimate e = EstimateCycle(Load(uid), hash);
    est_.emplace(key, e);
    return e;
}

} // namespace firedog
