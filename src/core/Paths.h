// Paths.h : where FireDog's files live.
//
// Every file belongs to exactly ONE of three places, by who writes it:
//
//   the EXE folder    what a build or a release ships, and nothing else:
//                     config/scripts/, the config/*.json DEFAULTS, assets/.
//                     A rebuild or an unzip may replace any of it, so the app
//                     never writes here. Shipped paths stay relative
//                     ("config/recipes.json") - InitPaths makes the exe folder
//                     the working directory, however the exe was launched.
//
//   the MACHINE root  what this PC has learned: hardware.json and
//                     motion_transforms.json (edited in place), every
//                     *.local.json and its journal, config/vision/, products/
//                     (documents + the employee roster), logs/, imgui.ini.
//                     Default D:/ProgramData/FireDog. MachinePath(rel).
//
//   the DATA root     production output, the bulky part: DUT records under
//                     products/<name>/, xcan/, align_attempts/, mosaic/.
//                     Default <machine root>/data, separately redirectable so
//                     a machine can put it on a bigger disk. DataPath(rel).
//
// Both roots come from FireDog.ini beside the exe ([paths] root= / data=),
// read once. The ini is never shipped, so a release unzipped over an install
// cannot reset where the machine keeps its state; a missing ini means the
// defaults, and a commented template is written for the next reader.
//
// The FIRST start against an empty machine root copies (never moves) the old
// exe-side state across - see InitPaths. Copying leaves an older exe able to
// run from what it had.
//
// InitPaths must not log: the logger's own file lives under the machine root,
// so it asks this module first. What InitPaths decided is kept in PathNotes()
// for the caller to log once the logger is up.
#pragma once

#include <string>
#include <vector>

namespace firedog {

// Resolve the roots, make the exe folder the working directory, create the
// roots, run the first-start migration and seed the in-place machine files.
// Idempotent; every accessor below calls it, so order of first use is safe.
void InitPaths();

const std::string& ExeDir();        // the folder FireDog.exe is in
const std::string& MachineRoot();   // e.g. "D:/ProgramData/FireDog"
const std::string& DataRoot();      // e.g. "D:/ProgramData/FireDog/data"
const std::string& IniPath();       // "<exe>/FireDog.ini"

// "<machine root>/<rel>" and "<data root>/<rel>" (forward slashes).
std::string MachinePath(const std::string& rel);
std::string DataPath(const std::string& rel);

struct PathNote {
    bool        error = false;   // logged as ERROR (else INFO)
    std::string text;
};
// What InitPaths resolved, migrated and fell back from, in order.
const std::vector<PathNote>& PathNotes();

} // namespace firedog
