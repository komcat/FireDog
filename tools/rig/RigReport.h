// RigReport.h : the machine-configuration handoff report.
//
// One function. It turns everything config/hardware.json describes — including
// the sections the rig cannot edit (pin name maps, pneumatic slides, io_watch,
// safety_watch, the Keithley's enabled channels) and the sibling
// motion_transforms.json — into one plain-text file, so a team that has neither
// the GUI nor a JSON editor in front of them can read, review and copy the
// machine's wiring and addressing.
//
// Read-only like the rest of the rig: it reports the loaded config, touches no
// hardware, and rewrites no config file. Reference: docs/rig-console.md.
#pragma once

#include <string>

namespace firedog::rig {

class ConfigStore;

// Default file name when the operator names none.
extern const char* const kDefaultReportFile;

// Write the whole configuration to `outPath` as text (overwriting it). Values
// the rig has edited but not saved are reported as edited and flagged, so a
// report never silently disagrees with the running machine.
//
// False with `err` filled if the file could not be written.
bool WriteConfigReport(const ConfigStore& store, const std::string& outPath,
                       std::string& err);

} // namespace firedog::rig
