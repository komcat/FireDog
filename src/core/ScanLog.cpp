// ScanLog.cpp
#include "core/ScanLog.h"

#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "core/Topics.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>

namespace firedog {

namespace {

// <data root>/xcan: production output (core/Paths.h).
std::string ScanDir() { return DataPath("xcan"); }

// Process-wide: the last scan id per device (prev_scan_id chains a stall to
// the retry that followed it) and the index append.
std::mutex                         g_mu;
std::map<std::string, std::string> g_lastScan;

// UTC ISO-8601 with milliseconds (same shape as the DUT record's stamps).
std::string UtcStamp()
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

// Local time as "YYYYMMDD_HHMMSS_mmm" (ids, sortable) and "YYYY-MM-DD" (the
// day folder — the same local day the daily log file uses).
void LocalStamps(std::string* id, std::string* day)
{
    using namespace std::chrono;
    const auto   now = system_clock::now();
    const time_t t   = system_clock::to_time_t(now);
    const int    ms  = static_cast<int>(
        duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[40];
    if (id) {
        std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03d",
                      tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                      tm.tm_min, tm.tm_sec, ms);
        *id = buf;
    }
    if (day) {
        std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d", tm.tm_year + 1900,
                      tm.tm_mon + 1, tm.tm_mday);
        *day = buf;
    }
}

std::string HostName()
{
    char*       v = nullptr;
    std::size_t n = 0;
    std::string out;
    if (_dupenv_s(&v, &n, "COMPUTERNAME") == 0 && v) out = v;
    std::free(v);
    return out;
}

// A device name as a file-name fragment.
std::string Safe(const std::string& s)
{
    std::string o;
    for (char c : s) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_';
        o += ok ? c : '_';
    }
    return o.empty() ? "dev" : o;
}

} // namespace

std::string ScanLog::NewGroupId(const std::string& caller)
{
    std::string id;
    LocalStamps(&id, nullptr);
    return Safe(caller) + "_" + id;
}

nlohmann::json ScanLog::PoseOf(const std::string& device)
{
    nlohmann::json pose = nlohmann::json::object();
    for (const char* axis : {"X", "Y", "Z", "U", "V", "W"}) {
        const DataStream* s = DataBus::Instance().Find(topics::MotionAxis(device, axis));
        if (s && s->Count() > 0) pose[axis] = s->LatestValue();
    }
    return pose;
}

std::string ScanLog::Begin(nlohmann::json header)
{
    if (active_) Finish(nlohmann::json::object(), "incomplete", "a second scan began");

    device_ = header.value("device", std::string());
    topic_  = header.value("topic", std::string());
    std::string stamp;
    LocalStamps(&stamp, nullptr);
    scanId_ = stamp + "_" + Safe(device_);

    std::string prev;
    {
        const std::scoped_lock lock(g_mu);
        prev = g_lastScan[device_];
        g_lastScan[device_] = scanId_;
    }

    header["type"]         = "header";
    header["scan_id"]      = scanId_;
    header["prev_scan_id"] = prev.empty() ? nlohmann::json() : nlohmann::json(prev);
    header["utc"]          = UtcStamp();
    header["host"]         = HostName();
    header_                = std::move(header);

    samples_.clear();
    haveBest_ = false;
    bestV_    = 0.0;
    bestPose_ = nlohmann::json();
    t0_       = std::chrono::steady_clock::now();
    active_   = true;
    return scanId_;
}

void ScanLog::Sample(nlohmann::json sample)
{
    if (!active_) return;
    sample["type"] = "sample";
    sample["seq"]  = samples_.size() + 1;
    sample["t"]    = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0_).count();
    const auto v = sample.find("v");
    if (v != sample.end() && v->is_number()) {
        const double x = v->get<double>();
        if (!haveBest_ || x > bestV_) {
            haveBest_ = true;
            bestV_    = x;
            bestPose_ = sample.value("pose", nlohmann::json());
        }
    }
    samples_.push_back(std::move(sample));
}

void ScanLog::Finish(nlohmann::json result, const std::string& status,
                     const std::string& error)
{
    if (!active_) return;
    active_ = false;

    if (!result.is_object()) result = nlohmann::json::object();
    result["type"]       = "result";
    result["status"]     = status;
    if (!error.empty()) result["error"] = error;
    result["n_samples"]  = samples_.size();
    result["duration_s"] = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - t0_).count();
    if (haveBest_) {
        result["best_v"]    = bestV_;
        result["best_pose"] = bestPose_;
    }

    try {
        namespace fs = std::filesystem;
        std::string day;
        LocalStamps(nullptr, &day);
        const fs::path dir = fs::path(ScanDir()) / day;
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) throw std::runtime_error("cannot create " + dir.string() + ": " + ec.message());

        const fs::path file = dir / (scanId_ + ".jsonl");
        const fs::path tmp  = dir / (scanId_ + ".jsonl.tmp");
        {
            std::ofstream f(tmp, std::ios::binary);
            if (!f.is_open()) throw std::runtime_error("cannot write " + tmp.string());
            f << header_.dump() << '\n';
            for (const nlohmann::json& s : samples_) f << s.dump() << '\n';
            f << result.dump() << '\n';
            if (!f.good()) throw std::runtime_error("write failed for " + tmp.string());
        }
        fs::rename(tmp, file, ec);
        if (ec) throw std::runtime_error("rename to " + file.string() + " failed: " + ec.message());

        // The index: enough to filter the dataset without opening every file.
        nlohmann::json idx;
        idx["scan_id"] = scanId_;
        idx["file"]    = file.filename().string();
        for (const char* k : {"prev_scan_id", "utc", "kind", "device", "topic", "axes",
                              "product", "serial", "context"}) {
            const auto it = header_.find(k);
            if (it != header_.end()) idx[k] = *it;
        }
        for (const char* k : {"status", "baseline", "final", "best_v", "exit_hit",
                              "n_samples", "duration_s"}) {
            const auto it = result.find(k);
            if (it != result.end()) idx[k] = *it;
        }
        {
            const std::scoped_lock lock(g_mu);
            std::ofstream f(dir / "index.jsonl", std::ios::app | std::ios::binary);
            if (f.is_open()) f << idx.dump() << '\n';
        }
    } catch (const std::exception& e) {
        FD_WARN("scanlog", "scan %s not saved: %s", scanId_.c_str(), e.what());
    }

    samples_.clear();
    header_ = nlohmann::json();
}

} // namespace firedog
