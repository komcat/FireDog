// Beacon.h : a temporary, named marker of a CAMERA pose - "the camera was
// looking here" - that a registered tool can be sent to later
// (move_tool_to_beacon) after the camera has moved on.
//
// Plain data. The store is Workflow's (SetBeacon / BeaconFor / ClearBeacons),
// for the live frame's reason: a program runs each queue item as its own engine
// Start and ctx is wiped between them.
//
// A beacon is the MEASURED pose of the camera's device after plant_beacon's own
// move to the plant location settled - never a commanded or typed number, so a
// beacon can only sit where the camera actually went. It lives in memory only:
// gone after kBeaconLifeMin minutes, at remove_beacon, at product load, or when
// the app closes. Program start and DUT open keep it. docs/beacon-design.md.
#pragma once

#include <chrono>
#include <string>

namespace firedog {

inline constexpr int kBeaconLifeMin = 30;

struct Beacon {
    std::string name;          // the step's name for it, e.g. "beacon1"
    // The motion device whose AXIS SPACE x / y / z are in: the camera's device.
    // A consumer refuses a registration referenced to any other device.
    std::string device;
    double x = 0.0, y = 0.0, z = 0.0;   // device axis mm, measured after settling
    std::string positionUid;   // the saved position it was planted from (record only)
    std::string at;            // wall clock, for the log
    std::chrono::steady_clock::time_point plantedAt{};
};

// Minutes this beacon has left (negative = expired), against `now`.
inline double BeaconMinutesLeft(const Beacon& b,
                                std::chrono::steady_clock::time_point now =
                                    std::chrono::steady_clock::now())
{
    const double age =
        std::chrono::duration<double, std::ratio<60>>(now - b.plantedAt).count();
    return static_cast<double>(kBeaconLifeMin) - age;
}

inline bool BeaconExpired(const Beacon& b,
                          std::chrono::steady_clock::time_point now =
                              std::chrono::steady_clock::now())
{
    return BeaconMinutesLeft(b, now) <= 0.0;
}

} // namespace firedog
