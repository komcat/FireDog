// Oee.h : station utilisation (OEE) — the PURE half: the states, the
// classification of a time window into them, and the KPIs. No I/O and no
// clock: the caller passes `now`, so a report is a function of its inputs.
// The record is core/OeeTracker; the page is ui/pages/PageOee.cpp.
//
// Ported from Horizon (Horizon.Workflow/Oee, FAAM.UI's OEE page) so the two
// machines report the same numbers by the same rules. Design: docs/oee-design.md.
//
// Every instant is exactly ONE state, by priority:
//   Down    — declared by a technician; wins over everything, even while the
//             application is closed, until somebody brings the station UP.
//   Offline — FireDog was not running.
//   Running — something was executing (orch_.Busy()).
//   Standby — online and not executing for LESS than the idle threshold:
//             loading / handling between units. Counts as production.
//   Idle    — online and not executing for the threshold or MORE — the whole
//             gap, retroactively, once it reaches the threshold.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace firedog {

using OeeMs = std::int64_t;   // UTC unix milliseconds — every OEE time

enum class OeeState { Offline, Running, Standby, Idle, Down };

// A not-running stretch at least this long is Idle (all of it); shorter is
// Standby. Horizon's default; a constant until a station needs another.
inline constexpr OeeMs kOeeIdleThresholdMs = 30LL * 60 * 1000;

// [start, end); end < 0 = still open (runs to `now`).
struct OeeInterval {
    OeeMs start = 0;
    OeeMs end   = -1;
};

struct OeeSegment {
    OeeMs    start = 0, end = 0;
    OeeState state = OeeState::Offline;
};

// One down -> up bracket: the maintenance log entry. Down records who declared
// it (the signed-in employee number), a category and the reason as seen then;
// Up records who cleared it, the real root cause and the action taken.
struct OeeDowntime {
    std::string id;
    OeeMs       start = 0;
    OeeMs       end   = -1;   // -1 = still down
    std::string downBy, downByName, category, reason;
    std::string upBy, upByName, rootCause, actionTaken;
    std::string station;
    bool  Open() const { return end < 0; }
    OeeMs Duration(OeeMs now) const { return (end < 0 ? now : end) - start; }
};

// A closed DUT. Aborted = closed without a pass/fail verdict: not judged.
enum class OeeVerdict { Pass, Fail, Aborted };
struct OeeUnit {
    OeeMs       at = 0;
    std::string serial, product;
    OeeVerdict  verdict = OeeVerdict::Aborted;
};

// The picture of one window (normally one local calendar day). Durations are
// classified up to elapsedEnd (now, for today); the rest has not happened yet,
// so every ratio is over ELAPSED time — "today so far" reads sensibly, and a
// past day's elapsed is its whole 24 h (23 / 25 h across a DST change).
struct OeeReport {
    OeeMs from = 0, to = 0, elapsedEnd = 0;
    OeeMs idleThreshold = kOeeIdleThresholdMs;
    std::vector<OeeSegment>  segments;
    std::vector<OeeDowntime> downtimes;   // overlapping the window
    int passed = 0, failed = 0, aborted = 0;

    OeeMs Total() const { return to - from; }
    OeeMs Elapsed() const { return elapsedEnd > from ? elapsedEnd - from : 0; }
    OeeMs TimeIn(OeeState s) const;
    OeeMs Production() const { return TimeIn(OeeState::Running) + TimeIn(OeeState::Standby); }
    OeeMs Up() const { return Production() + TimeIn(OeeState::Idle); }

    double Uptime() const { return Ratio(Up()); }                          // up / elapsed
    double Availability() const;                                           // 1 - down / elapsed
    double Utilization() const { return Ratio(Production()); }             // production / elapsed
    std::optional<double> Quality() const;   // pass / (pass + fail); none with no judged unit
    std::optional<double> Oee() const;       // utilization x quality (performance not tracked)

    double Ratio(OeeMs t) const;
};

// Classify [from, to) into segments, up to min(to, now). The inputs are the raw
// records: online sessions, executions, downtimes. The window is widened by the
// threshold on both sides first, so a gap straddling midnight is judged by its
// real length. Pure — Horizon's OeeCalculator.Classify, as a sweep.
std::vector<OeeSegment> OeeClassify(OeeMs from, OeeMs to, OeeMs now,
                                    const std::vector<OeeInterval>& sessions,
                                    const std::vector<OeeInterval>& runs,
                                    const std::vector<OeeInterval>& downs,
                                    OeeMs idleThreshold = kOeeIdleThresholdMs);

// ---- Time, in the operator's terms ------------------------------------------
OeeMs OeeNowMs();
OeeMs OeeLocalDayStart(OeeMs t);                 // local midnight of t's day
OeeMs OeeAddLocalDays(OeeMs dayStart, int days); // DST-correct (23 / 25 h days)
std::string OeeFormatLocal(OeeMs t, const char* strftimeFmt);
std::string OeeDuration(OeeMs ms);               // "3h 05m" / "12m 40s"
std::string OeeHm(OeeMs ms);                     // "3:05"
std::string OeePct(double ratio);                // "87.5 %"

const char* OeeStateLabel(OeeState s);           // "RUNNING", "IDLE", ...

} // namespace firedog
