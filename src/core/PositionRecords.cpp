// PositionRecords.cpp
#include "core/PositionRecords.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>

namespace firedog {

namespace {

std::mutex                 g_mu;     // one append at a time, process-wide
std::atomic<std::uint64_t> g_seq{0};

nlohmann::json ToJson(const PositionRecord& r)
{
    nlohmann::json j;
    j["t"]      = r.t;
    j["ts"]     = r.ts;
    j["name"]   = r.name;
    j["device"] = r.device;
    nlohmann::json pose = nlohmann::json::object();
    for (const auto& [axis, v] : r.pose) pose[axis] = v;
    j["pose"]        = pose;
    j["product"]     = r.product;
    j["product_uid"] = r.productUid;
    j["serial"]      = r.serial.empty() ? nlohmann::json() : nlohmann::json(r.serial);
    if (r.operatorId.empty())
        j["operator"] = nullptr;
    else
        j["operator"] = {{"id", r.operatorId}, {"name", r.operatorName}};
    return j;
}

bool FromJson(const nlohmann::json& j, PositionRecord& r)
{
    if (!j.is_object()) return false;
    const auto pose = j.find("pose");
    if (pose == j.end() || !pose->is_object()) return false;
    r.t      = j.value("t", std::int64_t{0});
    r.ts     = j.value("ts", std::string());
    r.name   = j.value("name", std::string());
    r.device = j.value("device", std::string());
    for (const auto& [axis, v] : pose->items())
        if (v.is_number()) r.pose[axis] = v.get<double>();
    r.product    = j.value("product", std::string());
    r.productUid = j.value("product_uid", std::string());
    if (const auto s = j.find("serial"); s != j.end() && s->is_string())
        r.serial = s->get<std::string>();
    if (const auto o = j.find("operator"); o != j.end() && o->is_object()) {
        r.operatorId   = o->value("id", std::string());
        r.operatorName = o->value("name", std::string());
    }
    return true;
}

// The last `maxLines` lines of a file, read backwards in chunks so a long
// history costs only its tail.
bool ReadTail(const std::filesystem::path& file, std::size_t maxLines,
              std::vector<std::string>& lines, std::string* err)
{
    std::ifstream f(file, std::ios::binary);
    if (!f.is_open()) {
        if (err) *err = "cannot open " + file.string();
        return false;
    }
    f.seekg(0, std::ios::end);
    const std::streamoff size = f.tellg();
    constexpr std::streamoff kChunk = 64 * 1024;

    std::string    buf;         // the tail read so far
    std::streamoff pos = size;
    // +1: the newline that ends the last line is not a line boundary of its own.
    while (pos > 0) {
        const std::streamoff take = std::min(kChunk, pos);
        pos -= take;
        std::string chunk(static_cast<std::size_t>(take), '\0');
        f.seekg(pos);
        f.read(chunk.data(), take);
        buf.insert(0, chunk);
        if (static_cast<std::size_t>(std::count(buf.begin(), buf.end(), '\n')) > maxLines)
            break;
    }

    std::vector<std::string> all;
    std::size_t start = 0;
    while (start < buf.size()) {
        std::size_t nl = buf.find('\n', start);
        if (nl == std::string::npos) nl = buf.size();
        std::string line = buf.substr(start, nl - start);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) all.push_back(std::move(line));
        start = nl + 1;
    }
    // When the read stopped mid-file the first piece is a partial line.
    if (pos > 0 && !all.empty()) all.erase(all.begin());
    if (all.size() > maxLines) all.erase(all.begin(), all.end() - static_cast<std::ptrdiff_t>(maxLines));
    lines = std::move(all);
    return true;
}

} // namespace

bool PositionRecordNameOk(const std::string& s)
{
    if (s.empty() || s.size() > 64) return false;
    for (const char c : s) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || c == '.';
        if (!ok) return false;
    }
    return s != "." && s != "..";
}

std::string PositionRecordsDir(const std::string& productDataDir)
{
    if (productDataDir.empty()) return std::string();
    return (std::filesystem::path(productDataDir) / "position_records").string();
}

bool AppendPositionRecord(const std::string& dir, const PositionRecord& rec,
                          std::string* err)
{
    namespace fs = std::filesystem;
    if (dir.empty()) {
        if (err) *err = "no product data folder";
        return false;
    }
    if (!PositionRecordNameOk(rec.name)) {
        if (err) *err = "bad record name '" + rec.name + "' (use A-Z a-z 0-9 - _ .)";
        return false;
    }
    try {
        const std::string line = ToJson(rec).dump();
        const std::scoped_lock lock(g_mu);
        std::error_code ec;
        fs::create_directories(dir, ec);
        if (ec) {
            if (err) *err = "cannot create " + dir + ": " + ec.message();
            return false;
        }
        const fs::path file = fs::path(dir) / (rec.name + ".jsonl");
        std::ofstream f(file, std::ios::app | std::ios::binary);
        if (!f.is_open()) {
            if (err) *err = "cannot open " + file.string();
            return false;
        }
        f << line << '\n';
        f.flush();
        if (!f.good()) {
            if (err) *err = "write failed for " + file.string();
            return false;
        }
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
    g_seq.fetch_add(1, std::memory_order_relaxed);
    return true;
}

std::uint64_t PositionRecordsSeq() { return g_seq.load(std::memory_order_relaxed); }

std::vector<std::string> ListPositionRecordNames(const std::string& dir)
{
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    if (dir.empty()) return out;
    std::error_code ec;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        const fs::path& p = it->path();
        if (p.extension() != ".jsonl") continue;
        out.push_back(p.stem().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

bool LoadPositionRecords(const std::string& dir, const std::string& name,
                         std::size_t maxTail, std::vector<PositionRecord>& out,
                         std::size_t* badLines, std::string* err)
{
    namespace fs = std::filesystem;
    out.clear();
    if (badLines) *badLines = 0;
    if (dir.empty() || name.empty()) return true;
    const fs::path file = fs::path(dir) / (name + ".jsonl");
    std::error_code ec;
    if (!fs::exists(file, ec)) return true;

    std::vector<std::string> lines;
    if (!ReadTail(file, std::max<std::size_t>(maxTail, 1), lines, err)) return false;
    out.reserve(lines.size());
    for (const std::string& line : lines) {
        PositionRecord r;
        const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
        if (j.is_discarded() || !FromJson(j, r)) {
            if (badLines) ++*badLines;
            continue;
        }
        out.push_back(std::move(r));
    }
    return true;
}

std::vector<std::size_t> FilterPositionRecords(const std::vector<PositionRecord>& recs,
                                               RecordRange range, int n,
                                               std::int64_t nowUnix)
{
    std::vector<std::size_t> idx;
    const std::size_t count = recs.size();
    switch (range) {
    case RecordRange::LastN: {
        const std::size_t k = n > 0 ? std::min<std::size_t>(count, static_cast<std::size_t>(n)) : 0;
        for (std::size_t i = count - k; i < count; ++i) idx.push_back(i);
        break;
    }
    case RecordRange::LastWeek:
    case RecordRange::LastMonth: {
        const std::int64_t days  = range == RecordRange::LastWeek ? 7 : 30;
        const std::int64_t since = nowUnix - days * 86400;
        for (std::size_t i = 0; i < count; ++i)
            if (recs[i].t >= since) idx.push_back(i);
        break;
    }
    case RecordRange::All:
    default:
        for (std::size_t i = 0; i < count; ++i) idx.push_back(i);
        break;
    }
    return idx;
}

AxisStats ComputeAxisStats(std::vector<double> v)
{
    AxisStats s;
    s.n = static_cast<int>(v.size());
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    s.min = v.front();
    s.max = v.back();
    const std::size_t m = v.size() / 2;
    s.median = (v.size() % 2) ? v[m] : 0.5 * (v[m - 1] + v[m]);
    double sum = 0;
    for (const double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    if (v.size() > 1) {
        double ss = 0;
        for (const double x : v) ss += (x - s.mean) * (x - s.mean);
        s.stdev = std::sqrt(ss / static_cast<double>(v.size() - 1));
    }
    return s;
}

} // namespace firedog
