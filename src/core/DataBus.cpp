// DataBus.cpp
#include "core/DataBus.h"

#include <memory>

namespace firedog {

void DataStream::Publish(double t, double value)
{
    std::vector<Callback> callbacks;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ring_.emplace_back(t, value);
        // Primary retention: drop anything older than the time window. The newest
        // timestamp is the reference, so this tolerates clock resets/pauses.
        if (windowSec_ > 0.0) {
            const double cutoff = t - windowSec_;
            while (!ring_.empty() && ring_.front().first < cutoff) ring_.pop_front();
        }
        // Backstop: bound memory if timestamps ever go backwards or stall.
        while (ring_.size() > capacity_) ring_.pop_front();
        latest_ = value;
        ++count_;

        callbacks.reserve(subs_.size());
        for (const auto& kv : subs_) callbacks.push_back(kv.second);
    }
    // Notify outside the lock so subscribers can't deadlock the publisher.
    for (const auto& cb : callbacks) cb(t, value);
}

void DataStream::SetWindowSeconds(double seconds)
{
    std::lock_guard<std::mutex> lock(mutex_);
    windowSec_ = seconds;
}

void DataStream::Snapshot(std::vector<float>& t, std::vector<float>& v) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    t.resize(ring_.size());
    v.resize(ring_.size());
    size_t i = 0;
    for (const auto& s : ring_) {
        t[i] = static_cast<float>(s.first);
        v[i] = static_cast<float>(s.second);
        ++i;
    }
}

int DataStream::Subscribe(Callback cb)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const int id = nextId_++;
    subs_.emplace(id, std::move(cb));
    return id;
}

void DataStream::Unsubscribe(int id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    subs_.erase(id);
}

double DataStream::LatestValue() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

size_t DataStream::Count() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
}

DataBus& DataBus::Instance()
{
    static DataBus instance;
    return instance;
}

DataStream& DataBus::Stream(const std::string& topic)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = streams_.find(topic);
    if (it == streams_.end()) {
        it = streams_.emplace(topic, std::make_unique<DataStream>()).first;
    }
    return *it->second;   // node-based map: reference stays valid across rehash
}

std::vector<std::string> DataBus::Topics() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    out.reserve(streams_.size());
    for (const auto& kv : streams_) out.push_back(kv.first);
    return out;
}

} // namespace firedog
