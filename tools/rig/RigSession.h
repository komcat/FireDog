// RigSession.h : one bring-up attempt — the rig's equivalent of App's hardware
// members.
//
// The point of the rig is that it uses the SAME path the client does:
// HardwareManager owns the devices, Workflow::Start builds them from config and
// drives the async connects. Nothing here re-implements a driver or a connect.
//
// Workflow::Start appends devices to the manager and is a once-per-object call,
// so a session is created fresh on every `connect` and destroyed on
// `disconnect` — that teardown is also what closes the VISA/TCP sessions.
#pragma once

#include "hardware/HardwareConfig.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <string>
#include <vector>

namespace firedog::rig {

struct DeviceRef;

// A copy of `full` with every device except `only` disabled. The io_watch and
// safety_watch sections are disabled too: their bindings name pins on devices
// that are not in a single-device fleet, so they could not resolve anyway.
HardwareConfig FilterToDevice(const HardwareConfig& full, const DeviceRef& only);

class RigSession {
public:
    // Starts the bring-up immediately (async, like the client's startup).
    // `connectSimFleet` mirrors App's HardwareManager::ConnectAllAsync — true
    // for a whole-config connect, false when probing a single device.
    RigSession(const HardwareConfig& cfg, bool connectSimFleet);

    RigSession(const RigSession&) = delete;
    RigSession& operator=(const RigSession&) = delete;

    // Tick Workflow for `seconds` (the slide state machines and the path
    // executor only advance while this runs), then return.
    void Settle(double seconds);

    // Tick until nothing is left in ConnState::Connecting, or the timeout ends.
    // False on timeout — a device that never answers.
    bool WaitForConnects(double timeoutSec);

    // Every device the config asked for reached ConnState::Connected.
    bool AllExpectedConnected() const;

    const HardwareConfig& Config() const { return cfg_; }
    HardwareManager&      Hw()           { return hw_; }
    const HardwareManager& Hw() const    { return hw_; }

private:
    bool AnyConnecting() const;

    // The config this session was started with — an edit made afterwards does
    // not apply until the next connect, and the probe reports against this.
    HardwareConfig  cfg_;

    // Declaration order matters and mirrors App.h: the manager owns the devices,
    // and Workflow holds the connect futures whose blocking destructors must
    // still see live devices. Workflow is destroyed first.
    HardwareManager hw_;
    Workflow        wf_{hw_};
};

// "Connected" / "Failed" / ... for printing.
const char* ConnStateName(ConnState s);

} // namespace firedog::rig
