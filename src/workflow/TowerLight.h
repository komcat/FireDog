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
    int         safetyBeeps       = 3;      // on a safety trip (0 = silent)
    double      downBuzzerSeconds = 3.0;    // steady, on a DOWN declaration (0 = silent)
};
TowerLightConfig LoadTowerLightConfig();

// What the tower should show. App decides it — it alone sees both the OEE
// state and whether a run is waiting on the operator. Safety is not here: the
// worker reads the safety watch itself, so a trip shows even if the UI hangs.
enum class TowerMode : int { Off, Running, Standby, Idle, Down, Waiting };

class TowerLight {
public:
    TowerLight(HardwareManager& hw, std::function<bool()> safetyTripped);
    ~TowerLight();   // stop, join, all lamps off
    TowerLight(const TowerLight&)            = delete;
    TowerLight& operator=(const TowerLight&) = delete;

    // No-op (logged) when disabled or the device is not in hardware.json.
    void Start(const TowerLightConfig& cfg);
    void SetMode(TowerMode m) { mode_.store(static_cast<int>(m)); }

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
    int                   dev_ = -1;   // index into the IO fleet
    int pinG_ = -1, pinY_ = -1, pinR_ = -1, pinB_ = -1;

    std::atomic<int> mode_{static_cast<int>(TowerMode::Off)};

    std::mutex              mu_;
    std::condition_variable cv_;
    bool                    stop_ = false;
    std::thread             worker_;

    // Worker-thread only.
    Lamps written_;
    bool  haveWritten_ = false;   // false = unknown: rewrite every pin
    std::chrono::steady_clock::time_point retryAt_{};
    int   tick_          = 0;
    bool  wasTripped_    = false;
    int   beepTicks_     = 0;
    int   wasDown_       = -1;    // -1 until the first tick: a Down carried over a restart does not buzz
    int   downBuzzTicks_ = 0;
};

} // namespace firedog
