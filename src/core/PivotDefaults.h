// PivotDefaults.h : the rotation pivot each PI hexapod is SUPPOSED to have.
//
// The C-887 holds its pivot (SPI R S T) in volatile memory: a controller
// restart puts it back to 0,0,0, and every U/V/W move after that silently
// rotates about the wrong point. The controller is the only place the pivot
// acts, so FireDog keeps the intended value here and writes it back whenever
// the controller reports something else (Workflow::TickPivotRestore).
//
// Machine state, keyed by device name: `config/pivot.local.json` under the
// machine root, falling back to a shipped `config/pivot.json`. Written by a
// successful pivot Write on the Motion page, and seeded once from the first
// non-zero pivot a device reports. Tolerant load: missing or malformed yields
// an empty map, never throws.
#pragma once

#include <map>
#include <string>

namespace firedog {

struct PivotDefault {
    double x = 0.0, y = 0.0, z = 0.0;   // pivot X Y Z (the controller's R S T), mm
};

using PivotDefaults = std::map<std::string, PivotDefault>;   // device name -> pivot

PivotDefaults LoadPivotDefaults();
bool          SavePivotDefaults(const PivotDefaults& d);   // best effort; false if unwritable

} // namespace firedog
