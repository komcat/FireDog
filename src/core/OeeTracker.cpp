// OeeTracker.cpp : the OEE record — JSON lines under <data root>/oee/.
// Design: docs/oee-design.md.
#include "core/OeeTracker.h"

#include "core/Log.h"
#include "core/Paths.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace firedog {

namespace {

constexpr OeeMs kHeartbeatMs = 60 * 1000;

// ---- UTC months: the file a record lives in ---------------------------------

std::tm UtcTm(OeeMs t)
{
    const std::time_t s = static_cast<std::time_t>(t / 1000);
    std::tm tm{};
    gmtime_s(&tm, &s);
    return tm;
}

std::string MonthKey(OeeMs t)
{
    const std::tm tm = UtcTm(t);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d", tm.tm_year + 1900, tm.tm_mon + 1);
    return buf;
}

// The first instant of the UTC month after t's.
OeeMs NextMonthStart(OeeMs t)
{
    std::tm tm = UtcTm(t);
    tm.tm_mday = 1;
    tm.tm_hour = tm.tm_min = tm.tm_sec = 0;
    tm.tm_mon += 1;   // _mkgmtime normalises December + 1
    return static_cast<OeeMs>(_mkgmtime(&tm)) * 1000;
}

OeeMs MonthStart(OeeMs t)
{
    std::tm tm = UtcTm(t);
    tm.tm_mday = 1;
    tm.tm_hour = tm.tm_min = tm.tm_sec = 0;
    return static_cast<OeeMs>(_mkgmtime(&tm)) * 1000;
}

const char* VerdictKey(OeeVerdict v)
{
    switch (v) {
        case OeeVerdict::Pass: return "pass";
        case OeeVerdict::Fail: return "fail";
        default:               return "aborted";
    }
}

OeeVerdict VerdictFrom(const std::string& s)
{
    if (s == "pass") return OeeVerdict::Pass;
    if (s == "fail") return OeeVerdict::Fail;
    return OeeVerdict::Aborted;
}

std::string Str(const nlohmann::json& j, const char* key)
{
    const auto it = j.find(key);
    return (it != j.end() && it->is_string()) ? it->get<std::string>() : std::string();
}

std::string Lower(std::string s)
{
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string ComputerName()
{
    char*       v = nullptr;
    std::size_t n = 0;
    std::string out;
    if (_dupenv_s(&v, &n, "COMPUTERNAME") == 0 && v) out = v;
    std::free(v);
    return out;
}

} // namespace

OeeTracker::~OeeTracker() { End(); }

void OeeTracker::Start()
{
    if (online_) return;
    dir_     = DataPath("oee");
    station_ = ComputerName();
    std::error_code ec;
    std::filesystem::create_directories(dir_, ec);
    if (ec) FD_WARN("oee", "cannot create %s (%s) - OEE is not recorded", dir_.c_str(),
                    ec.message().c_str());

    RecoverCrashed();
    LoadDowntimes();

    const OeeMs now = OeeNowMs();
    online_       = true;
    sessionStart_ = now;
    gapStart_     = now;
    running_      = false;
    WriteLive(now);
    ++rev_;
    if (const OeeDowntime* d = OpenDowntime())
        FD_WARN("oee", "station is DOWN since %s (declared by %s): %s",
                OeeFormatLocal(d->start, "%Y-%m-%d %H:%M").c_str(), d->downByName.c_str(),
                d->reason.c_str());
    FD_INFO("oee", "station online - OEE record in %s", dir_.c_str());
}

void OeeTracker::End()
{
    if (!online_) return;
    const OeeMs now = OeeNowMs();
    if (running_) AppendInterval("run", runStart_, now, runWhat_);
    AppendInterval("session", sessionStart_, now, std::string());
    running_ = false;
    online_  = false;
    std::error_code ec;
    std::filesystem::remove(dir_ + "/live.json", ec);
    ++rev_;
}

void OeeTracker::Tick()
{
    if (!online_) return;
    const OeeMs now = OeeNowMs();
    if (now - lastBeat_ >= kHeartbeatMs) WriteLive(now);
}

void OeeTracker::SetRunning(bool running, const std::string& what)
{
    if (!online_ || running == running_) return;
    const OeeMs now = OeeNowMs();
    if (running) {
        running_  = true;
        runStart_ = now;
        runWhat_  = what;
    } else {
        AppendInterval("run", runStart_, now, runWhat_);
        running_  = false;
        gapStart_ = now;
    }
    WriteLive(now);   // so a crash mid-run still knows when the run began
    ++rev_;
}

bool OeeTracker::GoDown(const std::string& by, const std::string& byName,
                        const std::string& category, const std::string& reason,
                        std::string* err)
{
    if (by.empty()) {
        if (err) *err = "Sign in first: a downtime is signed by an employee number.";
        return false;
    }
    if (reason.find_first_not_of(" \t\r\n") == std::string::npos) {
        if (err) *err = "A reason is required to put the station down.";
        return false;
    }
    if (const OeeDowntime* open = OpenDowntime()) {
        if (err)
            *err = "Already down since " + OeeFormatLocal(open->start, "%Y-%m-%d %H:%M") +
                   " (by " + open->downByName + ").";
        return false;
    }

    OeeDowntime d;
    d.start      = OeeNowMs();
    d.id         = "d" + std::to_string(d.start);
    d.downBy     = by;
    d.downByName = byName;
    d.category   = category;
    d.reason     = reason;
    d.station    = station_;
    const nlohmann::json j = {{"k", "down"},          {"id", d.id},
                              {"t", d.start},         {"by", d.downBy},
                              {"by_name", d.downByName}, {"category", d.category},
                              {"reason", d.reason},   {"station", d.station}};
    if (!AppendLine(dir_ + "/downtime.jsonl", j.dump())) {
        if (err) *err = "Cannot write " + dir_ + "/downtime.jsonl - the station was NOT put down.";
        return false;
    }
    downs_.push_back(d);
    ++rev_;
    FD_WARN("oee", "station DOWN - by %s [%s]: %s", d.downByName.c_str(), d.category.c_str(),
            d.reason.c_str());
    return true;
}

bool OeeTracker::GoUp(const std::string& by, const std::string& byName,
                      const std::string& rootCause, const std::string& actionTaken,
                      std::string* err)
{
    if (by.empty()) {
        if (err) *err = "Sign in first: a downtime is signed by an employee number.";
        return false;
    }
    const auto blank = [](const std::string& s) {
        return s.find_first_not_of(" \t\r\n") == std::string::npos;
    };
    if (blank(rootCause) || blank(actionTaken)) {
        if (err) *err = "Root cause and action taken are required to bring the station up.";
        return false;
    }
    OeeDowntime* d = nullptr;
    for (OeeDowntime& x : downs_)
        if (x.Open()) d = &x;
    if (!d) {
        if (err) *err = "The station is not down.";
        return false;
    }

    const OeeMs now = OeeNowMs();
    const nlohmann::json j = {{"k", "up"},           {"id", d->id},
                              {"t", now},            {"by", by},
                              {"by_name", byName},   {"root_cause", rootCause},
                              {"action", actionTaken}};
    if (!AppendLine(dir_ + "/downtime.jsonl", j.dump())) {
        if (err) *err = "Cannot write " + dir_ + "/downtime.jsonl - the station is still down.";
        return false;
    }
    d->end         = now;
    d->upBy        = by;
    d->upByName    = byName;
    d->rootCause   = rootCause;
    d->actionTaken = actionTaken;
    gapStart_      = now;
    ++rev_;
    FD_INFO("oee", "station UP - by %s (down %s, declared by %s). Root cause: %s. Action: %s.",
            byName.c_str(), OeeDuration(now - d->start).c_str(), d->downByName.c_str(),
            rootCause.c_str(), actionTaken.c_str());
    return true;
}

void OeeTracker::RecordUnit(const std::string& serial, const std::string& product, OeeVerdict v)
{
    if (!online_ || serial.empty()) return;
    OeeUnit u;
    u.at      = OeeNowMs();
    u.serial  = serial;
    u.product = product;
    u.verdict = v;
    const std::string key = MonthKey(u.at);
    const nlohmann::json j = {{"k", "unit"},          {"t", u.at},
                              {"serial", u.serial},   {"product", u.product},
                              {"verdict", VerdictKey(v)}};
    if (!AppendLine(dir_ + "/" + key + ".jsonl", j.dump())) return;
    if (const auto it = months_.find(key); it != months_.end()) it->second.units.push_back(u);
    ++rev_;
}

OeeTracker::Live OeeTracker::Status(OeeMs now) const
{
    if (const OeeDowntime* d = OpenDowntime()) return {OeeState::Down, d->start};
    if (!online_) return {OeeState::Offline, 0};
    if (running_) return {OeeState::Running, runStart_};
    return {now - gapStart_ >= kOeeIdleThresholdMs ? OeeState::Idle : OeeState::Standby,
            gapStart_};
}

const OeeDowntime* OeeTracker::OpenDowntime() const
{
    for (auto it = downs_.rbegin(); it != downs_.rend(); ++it)
        if (it->Open()) return &*it;
    return nullptr;
}

OeeReport OeeTracker::DayReport(OeeMs dayStart, OeeMs now)
{
    return Report(dayStart, OeeAddLocalDays(dayStart, 1), now);
}

OeeReport OeeTracker::Report(OeeMs from, OeeMs to, OeeMs now)
{
    const OeeMs extFrom = from - kOeeIdleThresholdMs;
    const OeeMs extTo   = to + kOeeIdleThresholdMs;

    std::vector<OeeInterval> sessions, runs, downs;
    OeeReport r;
    for (OeeMs m = MonthStart(extFrom); m < extTo; m = NextMonthStart(m)) {
        const Month& mo = LoadMonth(MonthKey(m));
        for (const OeeInterval& i : mo.sessions)
            if (i.start < extTo && i.end > extFrom) sessions.push_back(i);
        for (const OeeInterval& i : mo.runs)
            if (i.start < extTo && i.end > extFrom) runs.push_back(i);
        for (const OeeUnit& u : mo.units) {
            if (u.at < from || u.at >= to) continue;
            if (u.verdict == OeeVerdict::Pass)      ++r.passed;
            else if (u.verdict == OeeVerdict::Fail) ++r.failed;
            else                                    ++r.aborted;
        }
    }
    // What is still open lives in memory, not in a file yet.
    if (online_) sessions.push_back({sessionStart_, -1});
    if (running_) runs.push_back({runStart_, -1});
    for (const OeeDowntime& d : downs_) {
        downs.push_back({d.start, d.end});
        if (d.start < to && (d.Open() || d.end > from)) r.downtimes.push_back(d);
    }

    r.from          = from;
    r.to            = to;
    r.elapsedEnd    = std::min(to, now);
    r.idleThreshold = kOeeIdleThresholdMs;
    r.segments      = OeeClassify(from, to, now, sessions, runs, downs, kOeeIdleThresholdMs);
    return r;
}

std::vector<OeeDowntime> OeeTracker::History(OeeMs from, OeeMs to,
                                             const std::string& search) const
{
    const std::string needle = Lower(search);
    std::vector<OeeDowntime> out;
    for (auto it = downs_.rbegin(); it != downs_.rend(); ++it) {
        const OeeDowntime& d = *it;
        if (d.start >= to || (!d.Open() && d.end <= from)) continue;
        if (!needle.empty()) {
            const std::string hay =
                Lower(d.category + '\n' + d.reason + '\n' + d.rootCause + '\n' + d.actionTaken +
                      '\n' + d.downBy + '\n' + d.downByName + '\n' + d.upBy + '\n' + d.upByName);
            if (hay.find(needle) == std::string::npos) continue;
        }
        out.push_back(d);
    }
    return out;
}

const std::vector<std::string>& OeeTracker::Categories()
{
    static const std::vector<std::string> k = {
        "Mechanical",          "Electrical",       "Pneumatic",   "Motion / stage",
        "Optics / vision",     "Software / controls", "Material / parts", "Calibration",
        "Preventive maintenance", "Utilities (air / power)", "Other",
    };
    return k;
}

// ---- Files -------------------------------------------------------------------

OeeTracker::Month& OeeTracker::LoadMonth(const std::string& key)
{
    if (const auto it = months_.find(key); it != months_.end()) return it->second;
    Month& m = months_[key];
    std::ifstream f(dir_ + "/" + key + ".jsonl");
    if (!f.is_open()) return m;   // nothing happened that month (or no folder)
    std::string line;
    int bad = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        try {
            const nlohmann::json j = nlohmann::json::parse(line);
            const std::string k = Str(j, "k");
            if (k == "session" || k == "run") {
                const OeeInterval i{j.value("a", OeeMs{0}), j.value("b", OeeMs{0})};
                if (i.end > i.start) (k == "run" ? m.runs : m.sessions).push_back(i);
            } else if (k == "unit") {
                OeeUnit u;
                u.at      = j.value("t", OeeMs{0});
                u.serial  = Str(j, "serial");
                u.product = Str(j, "product");
                u.verdict = VerdictFrom(Str(j, "verdict"));
                m.units.push_back(u);
            }
        } catch (const std::exception&) {
            ++bad;   // a torn last line from a power cut: skip it, keep the rest
        }
    }
    if (bad > 0)
        FD_WARN("oee", "%s.jsonl: skipped %d unreadable line(s)", key.c_str(), bad);
    return m;
}

// Split at UTC month boundaries, each piece into the file of the month it is
// in — the invariant that lets a report read only the months it touches.
void OeeTracker::AppendInterval(const char* kind, OeeMs a, OeeMs b, const std::string& what)
{
    const bool isRun = std::string(kind) == "run";
    while (a < b) {
        const OeeMs pieceEnd = std::min(b, NextMonthStart(a));
        const std::string key = MonthKey(a);
        nlohmann::json j = {{"k", kind}, {"a", a}, {"b", pieceEnd}};
        if (isRun && !what.empty()) j["what"] = what;
        if (AppendLine(dir_ + "/" + key + ".jsonl", j.dump())) {
            if (const auto it = months_.find(key); it != months_.end())
                (isRun ? it->second.runs : it->second.sessions).push_back({a, pieceEnd});
        }
        a = pieceEnd;
    }
}

bool OeeTracker::AppendLine(const std::string& file, const std::string& line)
{
    std::ofstream f(file, std::ios::app | std::ios::binary);
    if (!f.is_open()) {
        FD_WARN("oee", "cannot append to %s", file.c_str());
        return false;
    }
    f << line << '\n';
    return f.good();
}

void OeeTracker::WriteLive(OeeMs now)
{
    lastBeat_ = now;
    nlohmann::json j = {{"session_start", sessionStart_}, {"last_seen", now}};
    if (running_) {
        j["run_start"] = runStart_;
        j["run_what"]  = runWhat_;
    }
    // temp + rename: a crash mid-write must leave the previous heartbeat, not
    // half of this one.
    const std::string path = dir_ + "/live.json";
    {
        std::ofstream f(path + ".tmp", std::ios::trunc | std::ios::binary);
        if (!f.is_open()) return;
        f << j.dump() << '\n';
        if (!f.good()) return;
    }
    std::error_code ec;
    std::filesystem::rename(path + ".tmp", path, ec);
}

// A live.json at Start is the last process's open session: it died without
// End(). Close it, and its run, at the last heartbeat it managed to write —
// that bounds the loss to one heartbeat period.
void OeeTracker::RecoverCrashed()
{
    const std::string path = dir_ + "/live.json";
    std::ifstream f(path);
    if (!f.is_open()) return;
    try {
        nlohmann::json j;
        f >> j;
        const OeeMs ss = j.value("session_start", OeeMs{0});
        const OeeMs ls = j.value("last_seen", OeeMs{0});
        if (ss > 0 && ls > ss) AppendInterval("session", ss, ls, std::string());
        if (j.contains("run_start")) {
            const OeeMs rs = j.value("run_start", OeeMs{0});
            if (rs > 0 && ls > rs) AppendInterval("run", rs, ls, Str(j, "run_what"));
        }
        FD_WARN("oee", "the last session ended without a clean exit - closed at its last "
                       "heartbeat (%s)",
                OeeFormatLocal(ls, "%Y-%m-%d %H:%M:%S").c_str());
    } catch (const std::exception& e) {
        FD_WARN("oee", "live.json is unreadable (%s) - the last session is lost", e.what());
    }
    f.close();
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

void OeeTracker::LoadDowntimes()
{
    downs_.clear();
    std::ifstream f(dir_ + "/downtime.jsonl");
    if (!f.is_open()) return;
    std::string line;
    int bad = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        try {
            const nlohmann::json j = nlohmann::json::parse(line);
            const std::string k = Str(j, "k");
            if (k == "down") {
                OeeDowntime d;
                d.id         = Str(j, "id");
                d.start      = j.value("t", OeeMs{0});
                d.downBy     = Str(j, "by");
                d.downByName = Str(j, "by_name");
                d.category   = Str(j, "category");
                d.reason     = Str(j, "reason");
                d.station    = Str(j, "station");
                downs_.push_back(d);
            } else if (k == "up") {
                const std::string id = Str(j, "id");
                for (OeeDowntime& d : downs_) {
                    if (d.id != id) continue;
                    d.end         = j.value("t", OeeMs{0});
                    d.upBy        = Str(j, "by");
                    d.upByName    = Str(j, "by_name");
                    d.rootCause   = Str(j, "root_cause");
                    d.actionTaken = Str(j, "action");
                }
            }
        } catch (const std::exception&) {
            ++bad;
        }
    }
    if (bad > 0) FD_WARN("oee", "downtime.jsonl: skipped %d unreadable line(s)", bad);
}

} // namespace firedog
