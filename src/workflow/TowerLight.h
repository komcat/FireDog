// TowerLight.h : the station's stack light follows the OEE state, so the floor
// reads the machine from across the room. Ported from Horizon's FAAM.UI
// TowerLight — same table, same seed. Design: docs/oee-design.md.
//
//   Mode                          Green       Yellow      Red         Buzzer
//   Running                       steady
//   Standby (< 30 min)            blink 0.5 s
//   Idle (>= 30 min)                          steady
//   Down (declared)                                       steady      3 s on the declaration
//   Waiting on the operator                   blink 0.5 s
//   Safety alert (until ack)                              blink       3 beeps on the trip
//   Operator notice (until OK)    (as the mode beneath)               its BuzzPattern
//   FireDog closed                off         off         off         off
//
// Priority: safety > down > waiting > running / standby / idle — a program
// blocked on a prompt shows yellow blinking, not green.
//
// Owned by Workflow and driven by ONE worker thread (a 250 ms pump), joined in
// the destructor, which then switches every lamp off — before the IO devices
// disconnect, because Workflow dies before HardwareManager. The worker writes
// straight to the IIoDevice rather than through Workflow::SetIoOutput: that
// logs every write, and a blinking lamp is two writes a second forever. Only
// pins that CHANGED are written; a failed write is logged and retried after
// 5 s. A manual toggle on the IO page therefore holds until the next change.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace firedog {

class HardwareManager;

// <machine root>/config/tower_light.local.json. There is no shipped default
// file: these values ARE the default (Horizon's faam_tower_light.json seed),
// written out on the first start so the machine has a file to edit.
struct TowerLightConfig {
    bool        enabled = true;
    std::string device  = "IOTower";
    std::string green   = "Tower_Green";
    std::string yellow  = "Tower_Yellow";
    std::string red     = "Tower_Red";
    std::string buzzer  = "Tower_Buzzer";   // empty = no buzzer
    // The IO device the BUZZER pin is on, when it is not the lamps' device
    // (a machine whose buzzer hangs off a switch panel). Empty = `device`.
    std::string buzzerDevice;
    int         safetyBeeps       = 3;      // on a safety trip (0 = silent)
    double      downBuzzerSeconds = 3.0;    // steady, on a DOWN declaration (0 = silent)
};
TowerLightConfig LoadTowerLightConfig();

// What the tower should show. App decides it — it alone sees both the OEE
// state and whether a run is waiting on the operator. Safety is not here: the
// worker reads the safety watch itself, so a trip shows even if the UI hangs.
enum class TowerMode : int { Off, Running, Standby, Idle, Down, Waiting };

// A buzzer pattern laid OVER whatever the lamps show, for as long as an
// operator notice (notify_operator) is up. A safety trip still wins: its own
// beeps replace it. 250 ms ticks:
//   Beep    0.5 s on / 0.5 s off
//   Double  two 250 ms beeps, then 1.5 s silence (2 s cycle)
//   Triple  three 250 ms beeps, then 1.5 s silence (3 s cycle)
//   Steady  on continuously
enum class BuzzPattern : int { None, Beep, Double, Triple, Steady };

class TowerLight {
public:
    TowerLight(HardwareManager& hw, std::function<bool()> safetyTripped);
    ~TowerLight();   // stop, join, all lamps off
    TowerLight(const TowerLight&)            = delete;
    TowerLight& operator=(const TowerLight&) = delete;

    // No-op (logged) when disabled or the device is not in hardware.json.
    void Start(const TowerLightConfig& cfg);
    void SetMode(TowerMode m) { mode_.store(static_cast<int>(m)); }
    // Any thread. None silences it at the next tick.
    void SetBuzz(BuzzPattern p) { buzz_.store(static_cast<int>(p)); }

private:
    struct Lamps {
        bool green = false, yellow = false, red = false, buzzer = false;
        bool operator==(const Lamps&) const = default;
    };
    void  Run();
    Lamps Next();
    void  Write(const Lamps& want);

    HardwareManager&      hw_;
    std::function<bool()> safetyTripped_;
    TowerLightConfig      cfg_;
    // One output: where it is and what was last written to it. Per PIN, not
    // per device, because the lamps and the buzzer may sit on two devices that
    // connect at different times - each pin is rewritten only when it changed
    // or its device came back, never every tick.
    struct Out {
        int         dev = -1, pin = -1;   // IO fleet index, output pin (-1 = none)
        std::string label;                // "IOTower.Tower_Green", for the log
        bool        known = false;        // false = unknown: write it next time
        bool        state = false;        // last value written
    };
    Out outG_, outY_, outR_, outB_;
    int dev_ = -1;   // the lamps' device (index into the IO fleet)

    std::atomic<int> mode_{static_cast<int>(TowerMode::Off)};
    std::atomic<int> buzz_{static_cast<int>(BuzzPattern::None)};

    std::mutex              mu_;
    std::condition_variable cv_;
    bool                    stop_ = false;
    std::thread             worker_;

    // Worker-thread only.
    std::chrono::steady_clock::time_point retryAt_{};
    int   tick_          = 0;
    bool  wasTripped_    = false;
    int   beepTicks_     = 0;
    int   wasDown_       = -1;    // -1 until the first tick: a Down carried over a restart does not buzz
    int   downBuzzTicks_ = 0;
    int   buzzWas_       = 0;     // the pattern last tick (a new one starts its cycle at 0)
    int   buzzTick_      = 0;     // position in the pattern's cycle
};

} // namespace firedog
