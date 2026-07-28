// Topics.h : shared DataBus topic names. The publisher (layer 2 wires the
// Keithley poll thread to these) and the consumer (the Aux chart pulls them)
// must agree on the strings — keep them here so neither side hard-codes them.
#pragma once

#include <string>

namespace firedog::topics {

inline constexpr const char* kKeithleyCh1 = "keithley/ch1";
inline constexpr const char* kKeithleyCh2 = "keithley/ch2";

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
