// CycleTimes.h : how long each recipe takes, kept per run, and the estimate
// the next run of the same recipe VERSION is shown against.
//
// One line per Completed, timed recipe run in
//
//     <product data dir>/cycle_times/<recipe uid>.jsonl
//
// written by App::UpdateCycleTimes alone (the engine queues the timing; the
// worker never touches the file). Wall time is split into machine time and
// operator time - a long prompt must not move the next fifty ETAs - and only
// runs of the same uid AND step hash (RecipeStepsHash) are compared.
//
// File IO + the pure statistics, and CycleTimeStore: a UI-thread-only cache in
// front of the files, like OeeTracker. docs/cycle-time-design.md.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

struct CycleTimeRecord {
    std::int64_t  t = 0;            // unix seconds (UTC)
    std::string   ts;               // "YYYY-MM-DDTHH:MM:SS" local, for reading
    std::string   uid, name;        // the recipe
    std::uint64_t hash = 0;         // its version (RecipeStepsHash)
    std::int64_t  wallMs = 0;       // Start to the end
    std::int64_t  waitMs = 0;       // of which blocked on the operator
    std::size_t   steps  = 0;       // leaf steps that ran
    std::string   serial;           // "" = no DUT
    std::string   operatorId;       // "" = nobody signed in
    std::int64_t MachineMs() const { return wallMs > waitMs ? wallMs - waitMs : 0; }
};

// How many matching runs an estimate looks back over.
inline constexpr int kCycleWindow = 20;

struct CycleEstimate {
    int          n = 0;              // matching runs used (0 = no history)
    std::int64_t machineMs = 0;      // median machine time
    std::int64_t waitMs    = 0;      // median operator time
    std::int64_t loMs = 0, hiMs = 0; // machine p10..p90 (min..max below 5 runs)
    bool Valid() const { return n > 0; }
    std::int64_t TypicalMs() const { return machineMs + waitMs; }
};

// The estimate over the last `window` records (oldest first) whose hash is
// `hash`. Pure.
CycleEstimate EstimateCycle(const std::vector<CycleTimeRecord>& recs, std::uint64_t hash,
                            int window = kCycleWindow);

// <productDataDir>/cycle_times ("" when productDataDir is "").
std::string CycleTimesDir(const std::string& productDataDir);

// "m:ss" or "h:mm:ss" (negative -> "0:00").
std::string FormatClockMs(std::int64_t ms);

// UI-thread only. Follows one folder; loads a recipe's file the first time it
// is asked about (its tail only) and caches each (uid, hash) estimate until
// the next append.
class CycleTimeStore {
public:
    // Switching folders (another product) drops every cache.
    void SetDir(const std::string& dir);
    const std::string& Dir() const { return dir_; }

    // Append to <dir>/<uid>.jsonl and to the cache. False + err on no folder,
    // an unusable uid or a failed write.
    bool Append(const CycleTimeRecord& rec, std::string* err);

    CycleEstimate Estimate(const std::string& uid, std::uint64_t hash);

    // Changes on every append and folder switch - lets a caller cache sums.
    std::uint64_t Seq() const { return seq_; }

private:
    const std::vector<CycleTimeRecord>& Load(const std::string& uid);

    std::string dir_;
    std::map<std::string, std::vector<CycleTimeRecord>> byUid_;   // loaded tails
    std::map<std::pair<std::string, std::uint64_t>, CycleEstimate> est_;
    std::uint64_t seq_ = 1;
};

} // namespace firedog
