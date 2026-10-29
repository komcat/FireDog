// OeeTracker.h : station utilisation (OEE) — the RECORD. What the station was
// doing, written as it happens, read back as day reports for the OEE page.
// The pure half (states, classification, KPIs) is core/Oee.h.
// Design: docs/oee-design.md.
//
// Files, all under <data root>/oee/ (production output, like the DUT records):
//   YYYY-MM.jsonl   CLOSED intervals and units, one JSON object per line, filed
//                   by the UTC month they start in. An interval is split at a
//                   month boundary before it is written, so a month's file holds
//                   everything that happened in that month and a report reads
//                   exactly the months its window touches.
//                     {"k":"session","a":ms,"b":ms}            FireDog was running
//                     {"k":"run","a":ms,"b":ms,"what":"..."}   orch_.Busy()
//                     {"k":"unit","t":ms,"serial","product","verdict"}
//   downtime.jsonl  the maintenance log, all time — {"k":"down",...} when a
//                   technician declares the station down, {"k":"up",...} when
//                   it comes back. Small, read whole at Start: an open "down"
//                   with no "up" is a station that is STILL down, across a
//                   restart, which is the point of declaring it.
//   live.json       the OPEN session and run, rewritten every 60 s. A clean
//                   exit removes it, so finding one at Start means the last
//                   process died: its session and run are closed at its last
//                   heartbeat. Down is never closed by recovery.
//
// Idle and Standby are never stored — they are derived from the gaps.
//
// UI-THREAD ONLY, no locks — the same contract as ProductSession. Everything
// that feeds it is already on the UI thread (App::Update edges, the drained
// DUT-close queue, the page's DOWN/UP buttons). Appends are a line each; a
// month file is read once, on the first report that needs it.
#pragma once

#include "core/Oee.h"

#include <map>
#include <string>
#include <vector>

namespace firedog {

class OeeTracker {
public:
    OeeTracker() = default;
    ~OeeTracker();   // End(): a clean exit closes the open run and session
    OeeTracker(const OeeTracker&)            = delete;
    OeeTracker& operator=(const OeeTracker&) = delete;

    // Online: recovers what a crashed process left open, loads the maintenance
    // log, opens this session. Idempotent.
    void Start();
    // Offline: closes the open run and session and removes live.json.
    void End();
    // Every frame from App::Update: the 60 s heartbeat that bounds a crash.
    void Tick();

    // Running, edge-detected: the caller passes orch_.Busy() every frame and
    // what is running (for the record only). Ignored while offline.
    void SetRunning(bool running, const std::string& what);
    bool Running() const { return running_; }

    // The maintenance log. `by` / `byName` are the signed-in employee — the
    // caller's level check is the authority, this only refuses what cannot be
    // recorded: no name, no reason, already down / not down, a failed write.
    bool GoDown(const std::string& by, const std::string& byName,
                const std::string& category, const std::string& reason, std::string* err);
    bool GoUp(const std::string& by, const std::string& byName,
              const std::string& rootCause, const std::string& actionTaken, std::string* err);

    // A closed DUT (App drains DutSession's queue into this).
    void RecordUnit(const std::string& serial, const std::string& product, OeeVerdict v);

    // The live state and since when.
    struct Live {
        OeeState state = OeeState::Offline;
        OeeMs    since = 0;
    };
    Live Status(OeeMs now) const;
    const OeeDowntime* OpenDowntime() const;   // null when up

    // One local calendar day, and an arbitrary window. Non-const: the first
    // report touching a month reads its file.
    OeeReport DayReport(OeeMs dayStart, OeeMs now);
    OeeReport Report(OeeMs from, OeeMs to, OeeMs now);

    // Downtimes overlapping [from, to), newest first, filtered by a free-text
    // search (case-insensitive) over category / reason / root cause / action /
    // who.
    std::vector<OeeDowntime> History(OeeMs from, OeeMs to, const std::string& search) const;

    const std::string& Dir() const { return dir_; }
    // Bumped on every recorded change — the page's report cache key.
    unsigned Revision() const { return rev_; }

    // The DOWN form's category list (Horizon's presets).
    static const std::vector<std::string>& Categories();

private:
    struct Month {
        std::vector<OeeInterval> sessions, runs;
        std::vector<OeeUnit>     units;
    };
    Month& LoadMonth(const std::string& key);
    void   AppendInterval(const char* kind, OeeMs a, OeeMs b, const std::string& what);
    bool   AppendLine(const std::string& file, const std::string& line);
    void   WriteLive(OeeMs now);
    void   RecoverCrashed();
    void   LoadDowntimes();

    std::string dir_, station_;
    bool        online_       = false;
    OeeMs       sessionStart_ = 0;
    OeeMs       lastBeat_     = 0;
    bool        running_      = false;
    OeeMs       runStart_     = 0;
    std::string runWhat_;
    OeeMs       gapStart_     = 0;   // start of the current not-running stretch
    std::vector<OeeDowntime>     downs_;    // the whole log, oldest first
    std::map<std::string, Month> months_;   // by UTC "YYYY-MM", read on demand
    unsigned    rev_ = 0;
};

} // namespace firedog
