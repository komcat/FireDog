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

} // namespace firedog::topics
