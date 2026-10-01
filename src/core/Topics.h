// Topics.h : shared DataBus topic names. The publisher (layer 2 wires the
// Keithley poll thread to these) and the consumer (the Aux chart pulls them)
// must agree on the strings — keep them here so neither side hard-codes them.
#pragma once

#include <string>

namespace firedog::topics {

inline constexpr const char* kKeithleyCh1 = "keithley/ch1";
inline constexpr const char* kKeithleyCh2 = "keithley/ch2";

// Camera sharpness score (Laplacian variance over the focus ROI), published by
// Workflow — continuously while focus sampling is on, and once per sample
// during an auto_focus sweep. Relative units: comparable only within one scene
// at one exposure, which is exactly the Z-sweep case.
inline constexpr const char* kCameraFocus = "camera/focus";

// Feature detection, published by Workflow::Detect on every run — the wizard's
// live tuning loop, a reliability sweep and an auto alignment station all land
// here. Scalar streams because that is what a DataStream carries and what a
// chart wants; Workflow::LatestDetection() holds the full record for anyone who
// needs the angle, the radius and the error text together.
//
// kVisionOk is 1/0 rather than a gate: a reliability run counts misses, so the
// misses have to be on the stream too.
inline constexpr const char* kVisionX     = "vision/x_px";
inline constexpr const char* kVisionY     = "vision/y_px";
inline constexpr const char* kVisionScore = "vision/score";
inline constexpr const char* kVisionAngle = "vision/angle_deg";
inline constexpr const char* kVisionOk    = "vision/ok";

// CLD101x laser controller streams: temperature in deg C, laser current in mA
// (display units — unlike the Keithley topics, which publish amps).
inline constexpr const char* kCldTemp    = "cld/temp";
inline constexpr const char* kCldCurrent = "cld/current";

// UDP3305S PSU per-channel measured streams (1-based channel), volts and amps.
// Every PSU publishes under its own name, "psu/<device>/ch1/v" / ".../i".
// The FIRST configured PSU ALSO publishes the original device-less names
// "psu/ch1/v" / "psu/ch1/i" - saved Run-page readouts and charts reference
// them, and renaming a topic silently empties every one of those.
inline std::string PsuVoltage(const std::string& device, int channel)
{
    return "psu/" + device + "/ch" + std::to_string(channel) + "/v";
}
inline std::string PsuCurrent(const std::string& device, int channel)
{
    return "psu/" + device + "/ch" + std::to_string(channel) + "/i";
}
inline std::string PsuVoltage(int channel) { return "psu/ch" + std::to_string(channel) + "/v"; }
inline std::string PsuCurrent(int channel) { return "psu/ch" + std::to_string(channel) + "/i"; }

// CCS PD3 lighting per-channel read-back (front-panel channel, 1..8), e.g.
// "light/ch2/pct" — the intensity setpoint in percent of full scale — and
// "light/ch2/on" (1/0). Both are what the controller SAYS, published by the
// driver's poll thread, never what was last requested.
inline std::string LightPct(int channel) { return "light/ch" + std::to_string(channel) + "/pct"; }
inline std::string LightOn(int channel)  { return "light/ch" + std::to_string(channel) + "/on"; }

// Per-axis motion position stream, e.g. "motion/hex-left/X". Publishers are the
// PI/ACS drivers' poll threads; any consumer (charts, alignment procedures,
// loggers, a future 3D view) subscribes or pulls a Snapshot on the same string.
inline std::string MotionAxis(const std::string& device, const std::string& axis)
{
    return "motion/" + device + "/" + axis;
}

// Where a registered device's TOOL TIP is, e.g. "tip/hex-left/X" — the live
// pose run through that device's DeviceReg, published in the REFERENCE device's
// axis space (the gantry the camera rides on), which is the space every taught
// point and every pick already lives in.
//
// Published only for a device with a valid registration, and dropped the moment
// its rotations leave the ones the fit was taught at: a prediction through a
// pose the fit never saw is a number, not a position. Absence is the signal —
// see core/DeviceReg.h.
inline std::string TipAxis(const std::string& device, const std::string& axis)
{
    return "tip/" + device + "/" + axis;
}

// Per-device IO bitmask streams, e.g. "io/IOBottom/in". The drivers' poll
// threads publish the pin bitmask (as a double) only when it changes; future
// procedures ("wait for UV_Head_Up") subscribe on the same strings.
inline std::string IoInputs(const std::string& device)  { return "io/" + device + "/in"; }
inline std::string IoOutputs(const std::string& device) { return "io/" + device + "/out"; }

} // namespace firedog::topics
