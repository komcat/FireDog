// IoWatch.h : background input->output watchdog, ported from bobcat_engine's
// IoWatchManager (dropping Qt). A dedicated thread polls configured input pins
// (through the IO drivers' cached Status(), never hardware) and, on a
// triggering edge, executes an action on a bound output pin:
//   toggle (flip, default) | set_on | set_off | momentary (follow the input).
// Per-binding debounce suppresses switch bounce.
//
// A binding may also name an INDICATOR output (a switch's LED). Every tick it
// is driven to the bound output's level, read from the output device's cached
// Status() — so it shows the output's state whoever set it (this watch, the IO
// page, a recipe), and puts itself right if something else flips the LED. It
// keeps following while the watch is paused: a light is not an action.
//
// This is a pure background override — it does not touch procedures or
// safety logic. Effective edge resolution is bounded by the slower of the
// watch interval and the source device's own poll interval.
#pragma once

#include "hardware/io/IIoDevice.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace firedog {

class IoWatch {
public:
    enum class Edge { Rising, Falling, Both };
    enum class Behavior { Toggle, SetOn, SetOff, Momentary };

    static const char* EdgeName(Edge e);
    static const char* BehaviorName(Behavior b);
    static Edge     ParseEdge(const std::string& s);       // default Rising
    static Behavior ParseBehavior(const std::string& s);   // default Toggle

    // Wiring resolved by Workflow from config names. The referenced devices
    // are owned by HardwareManager and outlive the watch.
    struct Binding {
        std::string name;
        IIoDevice*  inputDev  = nullptr;  int inputPin  = -1;
        IIoDevice*  outputDev = nullptr;  int outputPin = -1;
        IIoDevice*  indicatorDev = nullptr;  int indicatorPin = -1;   // optional
        Edge       trigger    = Edge::Rising;
        Behavior    behavior   = Behavior::Toggle;
        int         debounceMs = 100;
    };

    IoWatch(std::vector<Binding> bindings, int intervalMs);
    ~IoWatch();
    IoWatch(const IoWatch&) = delete;
    IoWatch& operator=(const IoWatch&) = delete;

    void Start();
    void Stop();
    bool IsRunning() const { return running_.load(); }

    // Pause/resume evaluation without stopping the thread (UI kill switch).
    void SetActive(bool on) { active_.store(on); }
    bool IsActive() const { return active_.load(); }

    // Read-only view for the UI (atomics — safe against the watch thread).
    size_t BindingCount() const { return slots_.size(); }
    const Binding& BindingAt(size_t i) const { return slots_[i]->b; }
    bool     InputOn(size_t i) const { return slots_[i]->inputOn.load(); }
    uint64_t Hits(size_t i) const { return slots_[i]->hits.load(); }

private:
    // A binding + its edge/debounce runtime state (watch thread only, except
    // the atomics). unique_ptr keeps the atomics' addresses stable.
    struct Slot {
        Binding b;
        bool    prev    = false;
        bool    hasPrev = false;   // no edge until two valid reads
        std::chrono::steady_clock::time_point lastTrigger{};
        bool    hasTrigger = false;
        bool    ledFailed  = false;   // log a failing indicator once, not per tick
        std::atomic<bool>     inputOn{false};
        std::atomic<uint64_t> hits{0};
    };

    void Loop();
    void Execute(Slot& s, bool inputState);
    void MirrorIndicator(Slot& s);

    std::vector<std::unique_ptr<Slot>> slots_;
    int               intervalMs_ = 50;
    std::atomic<bool> running_{false};
    std::atomic<bool> active_{true};
    std::thread       thread_;
};

} // namespace firedog
