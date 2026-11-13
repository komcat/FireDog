// PositionRecords.h : a product's history of named device poses.
//
// The recipe step `record_position` (command/CommandsRecords.cpp) appends one
// line per unit run to
//
//     <product data dir>/position_records/<name>.jsonl
//
// where <product data dir> is Product::DataDir() (<data root>/products/<p>/data)
// - production output, never beside the exe. The NAME is what says which device
// and which step: the author drops the step right after whatever ends at the
// pose worth keeping (an xcan peak, a climb, a manual adjust) and names it.
// Over many units the records cluster, and the cluster is where the next
// first-light / peak search should start. The Position Records window
// (ui/windows/WndPositionRecords.cpp) only VIEWS them.
//
// File IO here, plus the pure statistics both the window and any later
// consumer use - the one place they live. docs/position-records-design.md.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace firedog {

struct PositionRecord {
    std::int64_t  t = 0;            // unix seconds (UTC) - filtering by time
    std::string   ts;               // "YYYY-MM-DDTHH:MM:SS" local, for reading
    std::string   name;             // the record name (= the file stem)
    std::string   device;           // motion device whose pose this is
    std::map<std::string, double> pose;   // axis -> value, whatever it reports
    std::string   product, productUid;
    std::string   serial;           // "" = no DUT open
    std::string   operatorId, operatorName;
};

// A record name usable as a file stem: 1..64 of [A-Za-z0-9-_.] (the DUT
// serial rule).
bool PositionRecordNameOk(const std::string& name);

// <productDataDir>/position_records ("" when productDataDir is "").
std::string PositionRecordsDir(const std::string& productDataDir);

// Append one record to <dir>/<rec.name>.jsonl, creating the folder. Serialised
// process-wide; bumps PositionRecordsSeq() on success.
bool AppendPositionRecord(const std::string& dir, const PositionRecord& rec,
                          std::string* err);

// Changes every time a record is appended anywhere - an open viewer compares it
// to the value it loaded at and reloads when it moved.
std::uint64_t PositionRecordsSeq();

// The record names (file stems) in `dir`, sorted. Empty when dir is missing.
std::vector<std::string> ListPositionRecordNames(const std::string& dir);

// Load the LAST `maxTail` lines of <dir>/<name>.jsonl, oldest first. A line
// that does not parse is skipped and counted in *badLines. A missing file is
// an empty history, not an error.
bool LoadPositionRecords(const std::string& dir, const std::string& name,
                         std::size_t maxTail, std::vector<PositionRecord>& out,
                         std::size_t* badLines, std::string* err);

// ---- statistics (pure) -----------------------------------------------------

enum class RecordRange { All = 0, LastWeek, LastMonth, LastN };

// Indices into `recs` (oldest first) that fall in the range: the last 7 / 30
// days from `nowUnix`, or the last `n` records.
std::vector<std::size_t> FilterPositionRecords(const std::vector<PositionRecord>& recs,
                                               RecordRange range, int n,
                                               std::int64_t nowUnix);

struct AxisStats {
    int    n = 0;
    double mean = 0, median = 0, stdev = 0, min = 0, max = 0;
};
AxisStats ComputeAxisStats(std::vector<double> values);

} // namespace firedog
