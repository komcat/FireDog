// Topics.h : shared DataBus topic names. The publisher (layer 2 wires the
// Keithley poll thread to these) and the consumer (the Aux chart pulls them)
// must agree on the strings — keep them here so neither side hard-codes them.
#pragma once

#include <string>

namespace firedog::topics {

inline constexpr const char* kKeithleyCh1 = "keithley/ch1";
inline constexpr const char* kKeithleyCh2 = "keithley/ch2";

// CLD101x laser controller streams: temperature in deg C, laser current in mA
// (display units — unlike the Keithley topics, which publish amps).
inline constexpr const char* kCldTemp    = "cld/temp";
inline constexpr const char* kCldCurrent = "cld/current";

// UDP3305S PSU per-channel measured streams (1-based channel), e.g.
// "psu/ch1/v" in volts and "psu/ch1/i" in amps.
inline std::string PsuVoltage(int channel) { return "psu/ch" + std::to_string(channel) + "/v"; }
inline std::string PsuCurrent(int channel) { return "psu/ch" + std::to_string(channel) + "/i"; }

// Per-axis motion position stream, e.g. "motion/hex-left/X". Publishers are the
// PI/ACS drivers' poll threads; any consumer (charts, alignment procedures,
// loggers, a future 3D view) subscribes or pulls a Snapshot on the same string.
inline std::string MotionAxis(const std::string& device, const std::string& axis)
{
    return "motion/" + device + "/" + axis;
}

// Per-device IO bitmask streams, e.g. "io/IOBottom/in". The drivers' poll
// threads publish the pin bitmask (as a double) only when it changes; future
// procedures ("wait for UV_Head_Up") subscribe on the same strings.
inline std::string IoInputs(const std::string& device)  { return "io/" + device + "/in"; }
inline std::string IoOutputs(const std::string& device) { return "io/" + device + "/out"; }

} // namespace firedog::topics
