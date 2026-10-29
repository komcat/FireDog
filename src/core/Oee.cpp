// Oee.cpp : the pure half of OEE — classification, KPIs, local-day time.
// Design: docs/oee-design.md.
#include "core/Oee.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>

namespace firedog {

OeeMs OeeReport::TimeIn(OeeState s) const
{
    OeeMs t = 0;
    for (const OeeSegment& g : segments)
        if (g.state == s) t += g.end - g.start;
    return t;
}

double OeeReport::Ratio(OeeMs t) const
{
    const OeeMs e = Elapsed();
    return e > 0 ? static_cast<double>(t) / static_cast<double>(e) : 0.0;
}

double OeeReport::Availability() const
{
    return Elapsed() > 0 ? 1.0 - Ratio(TimeIn(OeeState::Down)) : 0.0;
}

std::optional<double> OeeReport::Quality() const
{
    if (passed + failed == 0) return std::nullopt;
    return static_cast<double>(passed) / static_cast<double>(passed + failed);
}

std::optional<double> OeeReport::Oee() const
{
    const std::optional<double> q = Quality();
    if (!q) return std::nullopt;
    return Utilization() * *q;
}

// Horizon walks the sorted boundary points and asks, for each elementary piece,
// which intervals contain its midpoint. The same answer comes from a sweep over
// +1/-1 edges, without the per-piece scan over every interval: a week of runs
// is a few thousand edges, and the page asks for eight reports.
std::vector<OeeSegment> OeeClassify(OeeMs from, OeeMs to, OeeMs now,
                                    const std::vector<OeeInterval>& sessions,
                                    const std::vector<OeeInterval>& runs,
                                    const std::vector<OeeInterval>& downs,
                                    OeeMs idleThreshold)
{
    const OeeMs end = std::min(to, now);
    if (end <= from) return {};

    // Widened so a gap is judged by its whole length, not the part inside the
    // window; never past now, which has not happened.
    const OeeMs extFrom = from - idleThreshold;
    const OeeMs extTo   = std::min(to + idleThreshold, now);

    enum Kind { kSession = 0, kRun = 1, kDown = 2, kEdge = 3 };
    struct Edge {
        OeeMs t;
        int   kind;
        int   delta;
    };
    std::vector<Edge> edges;
    edges.reserve((sessions.size() + runs.size() + downs.size()) * 2 + 2);
    const auto add = [&](const std::vector<OeeInterval>& v, int kind) {
        for (const OeeInterval& i : v) {
            const OeeMs a = std::max(i.start, extFrom);
            const OeeMs b = std::min(i.end < 0 ? now : i.end, extTo);
            if (b <= a) continue;
            edges.push_back({a, kind, +1});
            edges.push_back({b, kind, -1});
        }
    };
    add(sessions, kSession);
    add(runs, kRun);
    add(downs, kDown);
    edges.push_back({extFrom, kEdge, 0});
    edges.push_back({extTo, kEdge, 0});
    std::sort(edges.begin(), edges.end(),
              [](const Edge& x, const Edge& y) { return x.t < y.t; });

    // Elementary pieces with equal neighbours merged. "Gap" is provisional:
    // online and not running, Standby or Idle by its merged length.
    enum Piece { kOffline, kRunning, kGap, kDowned };
    struct Raw {
        OeeMs a, b;
        Piece p;
    };
    std::vector<Raw> raw;
    int count[3] = {0, 0, 0};
    std::size_t i = 0;
    while (i < edges.size()) {
        const OeeMs t = edges[i].t;
        for (; i < edges.size() && edges[i].t == t; ++i)
            if (edges[i].kind != kEdge) count[edges[i].kind] += edges[i].delta;
        if (i == edges.size()) break;
        const OeeMs next = edges[i].t;
        const Piece p = count[kDown] > 0      ? kDowned
                        : count[kRun] > 0     ? kRunning
                        : count[kSession] > 0 ? kGap
                                              : kOffline;
        if (!raw.empty() && raw.back().p == p && raw.back().b == t) raw.back().b = next;
        else raw.push_back({t, next, p});
    }

    std::vector<OeeSegment> out;
    out.reserve(raw.size());
    for (const Raw& r : raw) {
        OeeState s;
        switch (r.p) {
            case kDowned:  s = OeeState::Down; break;
            case kRunning: s = OeeState::Running; break;
            case kOffline: s = OeeState::Offline; break;
            default:
                s = (r.b - r.a >= idleThreshold) ? OeeState::Idle : OeeState::Standby;
                break;
        }
        const OeeMs a = std::max(r.a, from);
        const OeeMs b = std::min(r.b, end);
        if (b > a) out.push_back({a, b, s});
    }
    return out;
}

// ---- Time -------------------------------------------------------------------

OeeMs OeeNowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

static std::tm LocalTm(OeeMs t)
{
    const std::time_t s = static_cast<std::time_t>(t / 1000);
    std::tm tm{};
    localtime_s(&tm, &s);
    return tm;
}

// mktime normalises an out-of-range day and, with tm_isdst = -1, works out the
// offset of the day it lands on — which is what makes a DST day 23 or 25 h.
static OeeMs LocalMidnight(std::tm tm)
{
    tm.tm_hour  = 0;
    tm.tm_min   = 0;
    tm.tm_sec   = 0;
    tm.tm_isdst = -1;
    return static_cast<OeeMs>(std::mktime(&tm)) * 1000;
}

OeeMs OeeLocalDayStart(OeeMs t) { return LocalMidnight(LocalTm(t)); }

OeeMs OeeAddLocalDays(OeeMs dayStart, int days)
{
    std::tm tm = LocalTm(dayStart);
    tm.tm_mday += days;
    return LocalMidnight(tm);
}

std::string OeeFormatLocal(OeeMs t, const char* strftimeFmt)
{
    const std::tm tm = LocalTm(t);
    char buf[64];
    const std::size_t n = std::strftime(buf, sizeof(buf), strftimeFmt, &tm);
    return std::string(buf, n);
}

std::string OeeDuration(OeeMs ms)
{
    if (ms < 0) ms = 0;
    const long long s = ms / 1000;
    char buf[32];
    if (s >= 3600) std::snprintf(buf, sizeof(buf), "%lldh %02lldm", s / 3600, (s / 60) % 60);
    else           std::snprintf(buf, sizeof(buf), "%lldm %02llds", s / 60, s % 60);
    return buf;
}

std::string OeeHm(OeeMs ms)
{
    if (ms < 0) ms = 0;
    const long long m = ms / 60000;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%lld:%02lld", m / 60, m % 60);
    return buf;
}

std::string OeePct(double ratio)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f %%", ratio * 100.0);
    return buf;
}

const char* OeeStateLabel(OeeState s)
{
    switch (s) {
        case OeeState::Running: return "RUNNING";
        case OeeState::Standby: return "STANDBY";
        case OeeState::Idle:    return "IDLE";
        case OeeState::Down:    return "DOWN";
        default:                return "OFFLINE";
    }
}

} // namespace firedog
