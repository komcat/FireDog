// PorHardware.h : the Hardware page of a POR compare — the master machine's
// hardware.json and motion_transforms.json against the target's
// (docs/por-design.md §6a).
//
// Two machines of one TYPE should be configured alike: the same devices under
// the same names, the same axes and transforms, the same pin NAME on the same
// pin NUMBER. The step compare cannot see a wiring difference - a recipe that
// names "switch_8" resolves on both machines even when switch_8 is input 3 on
// one and input 5 on the other - so this page lays the two files side by side,
// pin by pin, and says where a name moved.
//
// Fields are of two kinds:
//   * DESIGN   what the machine type fixes (axes, pin names, channel names,
//              counts, triggers, a transform) - a difference is a difference;
//   * MACHINE  this unit's identity (an IP, a port, a board id, a serial, a VISA
//              address) - shown, never judged.
// A camera's pixel->mm calibration is MEASURED: shown with the difference.
//
// Every device and pin the master's DECK uses is marked `used`, and a used one
// the target lacks is Missing - the rest of a difference is Changed.
#pragma once

#include "por/PorCompare.h"

#include <string>
#include <vector>

namespace firedog {

struct PorHardwareReport {
    // One Section row each: Motion devices, Motion transforms, IO devices (with
    // a pin row per named pin), Pneumatic slides, Cameras, Keithley, CLD101x,
    // PSUs, Lights (with a channel row per channel), IO watch, Safety watch.
    std::vector<PorRow>      sections;
    std::vector<std::string> notes;
};

PorHardwareReport CompareHardware(const PorSide& master, const PorSide& target,
                                  const CommandRegistry& reg);

} // namespace firedog
