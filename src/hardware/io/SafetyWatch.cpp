// SafetyWatch.cpp
//
// Ported from bobcat_engine's SafetyWatchManager.cpp (PollingThreadFunction
// :283, ShouldTrigger :373, RaiseSafetyFlag :391): edge detection on cached
// inputs, one latched alert at a time, acknowledge to clear.
#include "hardware/io/SafetyWatch.h"

#include "core/Log.h"

#include <utility>

namespace firedog {
namespace {

bool ShouldTrigger(bool prev, bool curr, IoWatch::Edge edge)
{
    if (prev == curr) return false;
    switch (edge) {
        case IoWatch::Edge::Rising:  return !prev && curr;
        case IoWatch::Edge::Falling: return prev && !curr;
        case IoWatch::Edge::Both:    return true;
    }
    return false;
}

} // namespace

SafetyWatch::SafetyWatch(std::vector<Signal> signals, int intervalMs, Callback onTrigger)
    : intervalMs_(intervalMs > 0 ? intervalMs : 100), onTrigger_(std::move(onTrigger))
{
    slots_.reserve(signals.size());
    for (Signal& s : signals) {
        auto slot = std::make_unique<Slot>();
        slot->s = std::move(s);
        slots_.push_back(std::move(slot));
    }
}

SafetyWatch::~SafetyWatch() { Stop(); }

void SafetyWatch::Start()
{
    if (running_.exchange(true)) return;
    thread_ = std::thread(&SafetyWatch::Loop, this);
    FD_INFO("safety", "watching %zu safety signals every %d ms", slots_.size(), intervalMs_);
}

void SafetyWatch::Stop()
{
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
}

SafetyWatch::Alert SafetyWatch::ActiveAlert() const
{
    std::lock_guard<std::mutex> lock(mu_);
    return alert_;
}

void SafetyWatch::Acknowledge()
{
    if (raised_.exchange(false)) {
        critical_.store(false);
        FD_WARN("safety", "alert acknowledged — operations may resume");
    }
}

void SafetyWatch::Loop()
{
    auto nextPoll = std::chrono::steady_clock::now();
    while (running_.load()) {
        for (auto& sp : slots_) {
            Slot& s = *sp;
            const IIoDevice::IoStatus st = s.s.dev->Status();
            if (!st.valid) continue;   // device still connecting — no data yet
            const bool curr = ((st.inputs >> s.s.pin) & 1ull) != 0;
            s.inputOn.store(curr);

            if (!s.hasPrev) {   // first valid read seeds the edge detector —
                s.prev = curr;  // a signal already active at startup does not fire
                s.hasPrev = true;
                continue;
            }
            const bool prev = s.prev;
            s.prev = curr;
            if (curr == prev) continue;

            FD_INFO("safety", "%s:%s %s -> %s", s.s.device.c_str(), s.s.pinName.c_str(),
                    prev ? "ON" : "OFF", curr ? "ON" : "OFF");
            if (ShouldTrigger(prev, curr, s.s.trigger)) {
                Raise(s.s, prev, curr);
            }
        }
        nextPoll += std::chrono::milliseconds(intervalMs_);
        std::this_thread::sleep_until(nextPoll);
    }
}

void SafetyWatch::Raise(const Signal& s, bool prev, bool curr)
{
    // One alert at a time (bobcat semantics): further triggers are ignored
    // until the active one is acknowledged.
    if (raised_.load()) {
        FD_WARN("safety", "trigger on %s:%s ignored — alert already active",
                s.device.c_str(), s.pinName.c_str());
        return;
    }

    Alert a;
    a.title    = s.alertTitle;
    a.message  = s.alertMessage;
    a.critical = s.critical;
    {
        std::lock_guard<std::mutex> lock(mu_);
        alert_ = a;
    }
    critical_.store(s.critical);
    raised_.store(true);

    FD_ERROR("safety", "TRIGGERED %s:%s (%s -> %s): %s%s",
             s.device.c_str(), s.pinName.c_str(),
             prev ? "ON" : "OFF", curr ? "ON" : "OFF",
             s.alertTitle.c_str(), s.critical ? " [CRITICAL]" : "");

    if (onTrigger_) onTrigger_(a);   // Workflow stops motion for critical alerts
}

} // namespace firedog
