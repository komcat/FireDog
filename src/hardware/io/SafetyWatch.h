// SafetyWatch.h : first-priority safety-signal monitor, ported from
// bobcat_engine's SafetyWatchManager (dropping Qt). A background thread polls
// watched input pins (through the IO drivers' cached Status()) and, on a
// triggering edge, latches ONE active alert (further triggers are ignored
// until acknowledged) and fires a callback. For critical signals Workflow's
// callback stops all motion immediately; the UI shows a modal alert that must
// be acknowledged to resume.
//
// This is a background override with no dependency on procedures — it runs
// from process start to shutdown regardless of what the UI shows.
#pragma once

#include "hardware/io/IIoDevice.h"
#include "hardware/io/IoWatch.h"   // reuses IoWatch::Edge + parsing

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace firedog {

class SafetyWatch {
public:
    // Wiring resolved by Workflow from config names.
    struct Signal {
        std::string  device, pinName;    // for logging
        IIoDevice*   dev = nullptr;
        int          pin = -1;
        IoWatch::Edge trigger = IoWatch::Edge::Rising;
        std::string  alertTitle   = "Safety Alert";
        std::string  alertMessage = "A safety condition was triggered.";
        bool         critical = false;
    };

    struct Alert {
        std::string title, message;
        bool        critical = false;
    };

    // Fired ONCE per raise (not per edge), from the watch thread.
    using Callback = std::function<void(const Alert&)>;

    SafetyWatch(std::vector<Signal> signals, int intervalMs, Callback onTrigger);
    ~SafetyWatch();
    SafetyWatch(const SafetyWatch&) = delete;
    SafetyWatch& operator=(const SafetyWatch&) = delete;

    void Start();
    void Stop();
    bool IsRunning() const { return running_.load(); }

    bool  AlertActive() const { return raised_.load(); }
    bool  AlertCritical() const { return critical_.load(); }
    Alert ActiveAlert() const;         // copy under mutex
    void  Acknowledge();               // clear the latched alert

    size_t SignalCount() const { return slots_.size(); }
    const Signal& SignalAt(size_t i) const { return slots_[i]->s; }
    bool InputOn(size_t i) const { return slots_[i]->inputOn.load(); }

private:
    struct Slot {
        Signal s;
        bool   prev = false, hasPrev = false;
        std::atomic<bool> inputOn{false};
    };

    void Loop();
    void Raise(const Signal& s, bool prev, bool curr);

    std::vector<std::unique_ptr<Slot>> slots_;
    int               intervalMs_ = 100;
    Callback          onTrigger_;
    std::atomic<bool> running_{false};
    std::atomic<bool> raised_{false};
    std::atomic<bool> critical_{false};
    mutable std::mutex mu_;            // guards alert_
    Alert             alert_;
    std::thread       thread_;
};

} // namespace firedog
