// IMotionController.h : layer 1 — common interface for a motion controller
// (PI hexapod, ACS gantry). HardwareManager owns a heterogeneous fleet of these
// behind this interface; Workflow (layer 2) drives their async connect.
//
// The concrete drivers (PIHexapod, ACSController) each keep their vendor SDK
// behind a PIMPL and fall back to a simulated stub when the SDK is absent, so
// the build is always green — same pattern as PylonCamera / Keithley6482.
#pragma once

#include "hardware/HardwareConfig.h"   // MotionConfig

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

class IMotionController {
public:
    virtual ~IMotionController() = default;

    virtual bool Connect()    = 0;   // blocking; opens the controller by IP:port
    virtual void Disconnect() = 0;
    virtual bool IsConnected() const = 0;

    virtual const std::string&        Name() const = 0;   // config display name
    virtual const std::string&        Type() const = 0;   // "PI" | "ACS"
    virtual std::vector<std::string>  Axes() const = 0;   // e.g. {X,Y,Z,U,V,W}

    // Thread-safe copy of the latest per-axis positions (axis -> value).
    virtual std::map<std::string, double> Positions() const = 0;

    // Minimal motion surface (layer 2 will use these for procedures).
    virtual bool MoveAbsolute(const std::string& axis, double position) = 0;
    // Coordinated absolute move of several axes in one command (e.g. gantry X+Y
    // together). An empty list is a no-op that returns true.
    virtual bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& targets) = 0;
    // Set jog/move velocity in mm/s. PI applies system velocity; ACS applies it
    // to every axis. Returns false if not connected.
    virtual bool SetVelocity(double mmPerSec) = 0;
    // True when the controller reports its motion profile finished. A position-
    // tolerance arrival check can LEAD the controller's own motion-complete
    // state by a moment (the slow tail of the profile); commands the controller
    // rejects while moving (PI VLS -> GCS error 1005 CONTROLLER_BUSY) must wait
    // for this. Default true: ACS accepts velocity changes mid-move, and the
    // sim is done as soon as positions read on target.
    virtual bool IsSettled() const { return true; }
    // True when the controller reports every axis ON TARGET — its own
    // settling-window logic (position within the settle window for the settle
    // time; PI qONT, tunable via SPA parameters). Profile-done (IsSettled) is
    // NOT this: the trajectory generator finishes while the platform can still
    // be ringing. Measurement-grade waits require both. Default true for
    // drivers without the signal (ACS, sims).
    virtual bool IsOnTarget() const { return true; }
    virtual bool StopAll() = 0;

    // Adaptive-polling demand signal. A consumer that needs realtime position
    // feedback (the PI/ACS page while open, an active-alignment procedure) calls
    // this repeatedly; the driver polls fast for a short window after each call
    // and drops to a slow idle rate when no one is asking. Cheap and thread-safe.
    virtual void RequestFastPoll() = 0;

    virtual std::string LastError() const = 0;
};

} // namespace firedog
