// ScanLog.h : layer core — the silent per-scan dataset record (xcan).
//
// Every alignment scan writes ONE self-contained JSON Lines file, whether or
// not a DUT is open and whether or not anybody asked for a trace:
//
//   data/xcan/YYYY-MM-DD/<scan_id>.jsonl   header, one line per sample, result
//   data/xcan/YYYY-MM-DD/index.jsonl       one summary line per scan
//
// It is the training set for a learned peak finder / side-lobe detector, so it
// keeps what a model needs and cannot recover afterwards (the full 6-axis pose
// at every sample, the individual readings behind each measurement, the policy
// parameters, the part and the operator) and nothing it can (dv, dmove, the
// running best). Schema and the offline stall label: docs/xcan-dataset.md.
//
// Samples are buffered in memory and the file is written ONCE at Finish
// (temp + rename): no disk I/O inside the timed walk. A write failure is one
// warning and never fails the scan. The only writer is the fd.scanlog binding
// in RunLuaFile, which also finishes a scan the script never finished (error /
// Stop) — so an aborted scan still lands, labelled as such.
//
// One instance per RunLuaFile call, used by that worker thread only; the
// process-wide state (last scan id per device, the index append) is locked.
#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

class ScanLog {
public:
    // Open a record. `header` must carry "device"; ScanLog adds scan_id,
    // prev_scan_id (the last scan on that device in this process), utc and
    // host. Returns the scan id. A second Begin finishes the first as
    // "incomplete".
    std::string Begin(nlohmann::json header);

    // One measurement. ScanLog adds seq and t (s since Begin); a "v" and a
    // "pose" object are tracked for the result's best_v / best_pose.
    void Sample(nlohmann::json sample);

    // Close and write. ScanLog adds status, error (when non-empty), n_samples,
    // duration_s, best_v and best_pose to `result`.
    void Finish(nlohmann::json result, const std::string& status,
                const std::string& error = std::string());

    bool               Active() const { return active_; }
    const std::string& Device() const { return device_; }
    const std::string& Topic() const { return topic_; }

    // An id for a wrapper that runs several scans as one alignment
    // (xcan_to_target, xcan_two_lens): "<caller>_<local stamp>".
    static std::string NewGroupId(const std::string& caller);

    // Every position stream motion/<device>/{X,Y,Z,U,V,W} that exists, as
    // {axis: mm}. Absent axes are left out, never guessed.
    static nlohmann::json PoseOf(const std::string& device);

private:
    bool                                  active_ = false;
    std::string                           device_, topic_, scanId_;
    nlohmann::json                        header_;
    std::vector<nlohmann::json>           samples_;
    std::chrono::steady_clock::time_point t0_{};
    double                                bestV_ = 0.0;
    nlohmann::json                        bestPose_;
    bool                                  haveBest_ = false;
};

} // namespace firedog
