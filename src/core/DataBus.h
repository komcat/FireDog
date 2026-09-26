// DataBus.h : lightweight app-wide publish/subscribe for measurement streams.
//
// Publishers push (time, value) samples to a named topic; subscribers either
// register a push callback or pull a snapshot of recent history (the pattern
// the immediate-mode chart uses). One process-wide instance, thread-safe.
#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace firedog {

class DataStream {
public:
    using Callback = std::function<void(double t, double value)>;

    void Publish(double t, double value);

    // Retain only samples newer than (latest_t - seconds). <= 0 disables the
    // time window and falls back to the sample-count backstop only.
    void SetWindowSeconds(double seconds);

    // Copy recent history into t/v (as float, ready for ImPlot).
    void Snapshot(std::vector<float>& t, std::vector<float>& v) const;

    int  Subscribe(Callback cb);     // push subscription; returns handle
    void Unsubscribe(int id);

    double LatestValue() const;
    size_t Count() const;

    // The (t, value) samples published AFTER publish count `sinceCount` (the
    // counter Count() returns), oldest first, at most `maxN` of the newest.
    // Anything already evicted from the ring is simply absent. This is how a
    // scan log sees the individual readings behind one measurement.
    std::vector<std::pair<double, double>> Tail(size_t sinceCount, size_t maxN) const;

private:
    mutable std::mutex                       mutex_;
    std::deque<std::pair<double, double>>    ring_;      // (t, value)
    double                                   windowSec_ = 10.0;   // time retention
    size_t                                   capacity_ = 20000;   // hard backstop
    double                                   latest_   = 0.0;
    size_t                                   count_    = 0;
    std::unordered_map<int, Callback>        subs_;
    int                                      nextId_   = 1;
};

class DataBus {
public:
    static DataBus& Instance();

    // Get (creating if needed) the stream for a topic. The reference is stable.
    DataStream& Stream(const std::string& topic);

    // The stream for a topic if something already created it, else null —
    // for a reader that must not make a topic appear in Topics() by asking.
    DataStream* Find(const std::string& topic);

    std::vector<std::string> Topics() const;

private:
    DataBus() = default;
    mutable std::mutex                                          mutex_;
    std::unordered_map<std::string, std::unique_ptr<DataStream>> streams_;
};

} // namespace firedog
