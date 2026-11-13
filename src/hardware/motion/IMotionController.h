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

// The point a hexapod's U/V/W rotate about, in the controller's own
// coordinates (mm). `seq` changes every time the cached value is replaced
// (read, written or invalidated), so a consumer can tell a fresh read from
// the one it already showed.
struct PivotPoint {
    double   x = 0.0, y = 0.0, z = 0.0;
    bool     valid = false;   // false until one has been read off the controller
    unsigned seq   = 0;
};

// The motion profile a controller is set to, per axis, in the controller's own
// user units (ACS: VEL / ACC / DEC / JERK / KDEC, mm/s, mm/s^2, mm/s^3). Read
// only — `seq` changes on every read so a consumer can tell a fresh one, and
// `error` says why the last read failed (empty when it did not).
struct AxisProfile {
    std::string axis;
    double vel = 0.0, acc = 0.0, dec = 0.0, jerk = 0.0, kdec = 0.0;
    bool   valid = false;   // every value of this axis read back
};
struct MotionProfile {
    std::vector<AxisProfile> axes;   // in the configured axis order (index = ACS axis)
    bool        valid = false;       // a read has completed since connect
    unsigned    seq   = 0;
    std::string error;
    // The DEFAULT profile: the first complete read after connect, before
    // FireDog changed anything — what every ordinary move restores. Empty
    // until captured, and a dynamic profile is refused until it is.
    std::vector<AxisProfile> defaults;
    bool dynamic = false;            // the controller holds a DYNAMIC profile now
};

// A profile derived from each axis's own VEL: ACC = accPerVel * VEL,
// DEC = ACC, JERK = jerkPerAcc * ACC. KDEC (the kill deceleration) is never
// touched — it is the stop the safety path relies on.
struct DynamicProfile {
    double accPerVel  = 10.0;   // 1/s
    double jerkPerAcc = 10.0;   // 1/s
};

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
    // Read back the velocity the controller is set to (PI qVLS; ACS the first
    // axis, which SetVelocity keeps equal to the rest). Blocks for one
    // controller exchange. False when not connected or the driver cannot tell -
    // a caller that meant to restore it must not invent a value.
    virtual bool ReadVelocity(double* mmPerSec)
    {
        (void)mmPerSec;
        return false;
    }
    // True when the controller reports its motion profile finished. A position-
    // tolerance arrival check can LEAD the controller's own motion-complete
    // state by a moment (the slow tail of the profile); commands the controller
    // rejects while moving (PI VLS -> GCS error 1005 CONTROLLER_BUSY) must wait
    // for this, and so must a coordinated move on ACS (acsc_ToPointM on an axis
    // still in motion -> error 3065). Default true: the sims are done as soon
    // as positions read on target.
    virtual bool IsSettled() const { return true; }
    // True when the controller reports every axis ON TARGET — its own
    // settling-window logic (position within the settle window for the settle
    // time; PI qONT, tunable via SPA parameters). Profile-done (IsSettled) is
    // NOT this: the trajectory generator finishes while the platform can still
    // be ringing. Measurement-grade waits require both. Default true for
    // drivers without the signal (ACS, sims).
    virtual bool IsOnTarget() const { return true; }
    virtual bool StopAll() = 0;

    // Ask the CONTROLLER whether an absolute move to `targets` is possible,
    // WITHOUT moving (PI: qVMO, the "virtual move" - for a hexapod whether a
    // pose is reachable depends on all six axes at once, so only the controller
    // can say; axes not named keep their current commanded value). One answer
    // for the whole set. Unknown = this driver has no such query (ACS, the
    // sims) and *why says so; Error = the query was sent and failed, *why has
    // the controller's error. Blocks for one controller exchange.
    enum class Reach { Unknown, Yes, No, Error };
    virtual Reach CheckMove(const std::vector<std::pair<std::string, double>>& targets,
                            std::string* why)
    {
        (void)targets;
        if (why) *why = "this controller has no reachability query";
        return Reach::Unknown;
    }

    // Per-axis servo enable (ACS: acsc_Enable / acsc_Disable; the state is the
    // ENABLE bit of the motor-state word, polled with the positions). A driver
    // without the concept (PI) says so and refuses. An axis missing from
    // AxisEnabled() has no known state yet (not connected, not polled).
    virtual bool SupportsAxisEnable() const { return false; }
    virtual std::map<std::string, bool> AxisEnabled() const { return {}; }
    virtual bool SetAxisEnabled(const std::string& axis, bool on)
    {
        (void)axis;
        (void)on;
        return false;
    }

    // Controller-resident programs (ACS: ACSPL+ buffers, acsc_RunBuffer /
    // acsc_StopBuffer). Run starts the buffer at `label` - ACSPL+'s
    // `START 1, STARTUP` is RunBuffer(1, "STARTUP") - or at its first line when
    // the label is empty; the program must already be compiled on the controller. Both block for one controller
    // exchange and only say the command was accepted - the program itself runs
    // on the controller. A driver without the concept says so and refuses.
    virtual bool SupportsBuffers() const { return false; }
    virtual bool RunBuffer(int buffer, const std::string& label = {})
    {
        (void)buffer;
        (void)label;
        return false;
    }
    virtual bool StopBuffer(int buffer)
    {
        (void)buffer;
        return false;
    }
    // CACHED run state of a buffer this session started (polled with the
    // positions; set on an accepted Run so the start is never missed). A buffer
    // never started from here reads false.
    virtual bool BufferRunning(int buffer) const
    {
        (void)buffer;
        return false;
    }

    // Reference (homing) state. A PI hexapod forgets it on a controller restart
    // and refuses every MOV until it is referenced again (GCS FRF). Referenced()
    // is CACHED - read at connect, every few seconds, and continuously while a
    // reference runs - so a UI may call it every frame. StartReference returns
    // once the controller has ACCEPTED it; the platform then moves on its own.
    enum class RefState { Unknown, No, Referencing, Yes };
    virtual bool     SupportsReference() const { return false; }
    virtual RefState Referenced() const { return RefState::Unknown; }
    virtual bool     StartReference() { return false; }

    // Rotation pivot point (PI hexapods: GCS SPI / qSPI on the pivot
    // coordinates R S T). Pivot() is the CACHED value — read at connect, again
    // on RequestPivotRead() by the poll thread, and read back after every
    // SetPivot — so a UI may call it every frame without a bus round trip.
    // SetPivot blocks for one controller exchange; the C-887 accepts it only
    // with U = V = W = 0 and refuses otherwise (LastError carries the code).
    // A driver without the concept says so and refuses.
    virtual bool       SupportsPivot() const { return false; }
    virtual PivotPoint Pivot() const { return {}; }
    virtual void       RequestPivotRead() {}
    virtual bool SetPivot(double x, double y, double z)
    {
        (void)x;
        (void)y;
        (void)z;
        return false;
    }

    // Motion profile READ-BACK (ACS: acsc_GetVelocity / GetAcceleration /
    // GetDeceleration / GetJerk / GetKillDeceleration per axis). Profile() is
    // the CACHED value — read at connect, after every accepted SetVelocity, and
    // on RequestProfileRead() by the poll thread — so a UI may call it every
    // frame. Nothing here writes a profile.
    virtual bool          SupportsProfile() const { return false; }
    virtual MotionProfile Profile() const { return {}; }
    virtual void          RequestProfileRead() {}

    // DYNAMIC profile, opt-in per move (ACS). MoveAbsoluteDynamic sets every
    // axis's ACC/DEC/JERK from its own current VEL, then makes the same
    // coordinated move MoveAbsolute does. The contract that makes it safe to
    // offer: EVERY ordinary MoveAbsolute (jog, path, recipe, every other page)
    // first puts the DEFAULT profile back if a dynamic one is still on the
    // controller, and so does Disconnect — so a dynamic profile lives exactly
    // as long as the move that asked for it, and nothing inherits it.
    // RestoreDefaultProfile is that restore on its own (a no-op when nothing
    // is modified). A driver without the concept refuses and does not move.
    virtual bool MoveAbsoluteDynamic(const std::vector<std::pair<std::string, double>>& targets,
                                     const DynamicProfile& dyn)
    {
        (void)targets;
        (void)dyn;
        return false;
    }
    virtual bool RestoreDefaultProfile() { return true; }

    // Adaptive-polling demand signal. A consumer that needs realtime position
    // feedback (the PI/ACS page while open, an active-alignment procedure) calls
    // this repeatedly; the driver polls fast for a short window after each call
    // and drops to a slow idle rate when no one is asking. Cheap and thread-safe.
    virtual void RequestFastPoll() = 0;

    virtual std::string LastError() const = 0;
};

} // namespace firedog
