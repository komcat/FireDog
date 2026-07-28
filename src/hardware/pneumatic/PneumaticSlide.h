// PneumaticSlide.h : a synthetic device composed from IO pins — one output
// (ON = extend, OFF = spring/air return) and two position sensors. Ported from
// bobcat_engine's PneumaticSlide, dropping the Qt layers and the manager class.
//
// The core is passive: Update() advances the state machine from the IO
// drivers' cached Status() (never hardware), and Workflow ticks it every
// frame. Wiring is resolved once at startup by Workflow from config pin
// names; the referenced IIoDevice objects are owned by HardwareManager and
// outlive the slide.
#pragma once

#include "hardware/io/IIoDevice.h"

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace firedog {

enum class SlidePosition {
    Extended, Retracted, Extending, Retracting, Unknown, HardwareError, Timeout
};

const char* SlidePositionName(SlidePosition p);

class PneumaticSlide {
public:
    // Resolved by Workflow from config names (device name + pin name).
    struct Wiring {
        std::string name;
        IIoDevice*  outputDev    = nullptr;  int outputPin    = -1;
        IIoDevice*  extendedDev  = nullptr;  int extendedPin  = -1;
        IIoDevice*  retractedDev = nullptr;  int retractedPin = -1;
        uint32_t    timeoutMs    = 10000;
    };

    // One consistent snapshot for the UI.
    struct Status {
        SlidePosition position    = SlidePosition::Unknown;
        bool          outputOn    = false;
        bool          extendedOn  = false;
        bool          retractedOn = false;
        bool          ioValid     = false;   // underlying IO caches have data
        bool          moving      = false;
        uint32_t      elapsedMs   = 0;       // current move, or last completed
    };

    explicit PneumaticSlide(Wiring w);
    PneumaticSlide(const PneumaticSlide&) = delete;
    PneumaticSlide& operator=(const PneumaticSlide&) = delete;

    const std::string& Name() const { return w_.name; }
    uint32_t TimeoutMs() const { return w_.timeoutMs; }
    bool IsIoConnected() const;   // all three referenced devices connected

    bool Extend();    // output ON,  target Extended  (non-blocking)
    bool Retract();   // output OFF, target Retracted (non-blocking)
    bool Stop();      // abort tracking + output OFF (air-return safe state)

    void   Update();        // state-machine tick — Workflow calls each frame
    Status GetStatus() const;
    SlidePosition Position() const;

    // Blocking variants for procedures (bobcat WaitForPosition port). They
    // poll Update() on the caller's thread — never call from the UI thread.
    bool ExtendAndWait(int pollMs = 50);
    bool RetractAndWait(int pollMs = 50);

    std::string LastError() const;

private:
    // Internal helpers assume mu_ is held.
    SlidePosition UpdateInternal();
    SlidePosition Determine(bool extended, bool retracted) const;
    bool SetOutput(bool on);
    uint32_t ElapsedMs() const;
    bool WaitForPosition(SlidePosition target, int pollMs);

    Wiring             w_;
    mutable std::mutex mu_;
    SlidePosition      position_ = SlidePosition::Unknown;
    SlidePosition      target_   = SlidePosition::Unknown;   // Unknown = idle
    bool               timedOut_ = false;   // sticky until a new command/definite sensor
    std::chrono::steady_clock::time_point moveStart_{};
    uint32_t           lastMoveMs_ = 0;
    std::string        lastError_;
};

} // namespace firedog
