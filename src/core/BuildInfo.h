// BuildInfo.h : what this executable IS — the commit it was built from, the
// configuration, the compiler, and which vendor SDKs were compiled in.
//
// The first question about a misbehaving machine is "which build is on it?",
// and a version string typed into a header answers it wrongly the first time
// somebody forgets to bump it. So the git state is captured on every build
// (cmake/GitInfo.cmake), and the build TIME is the exe's own link stamp read at
// run time — a timestamp baked into a header would either recompile on every
// build or go stale.
#pragma once

#include <string>
#include <vector>

namespace firedog {

struct BuildSdk {
    const char* name;   // "Pylon", "VISA", ...
    bool        in;     // its HAS_* was defined for this build
};

struct BuildInfo {
    std::string describe;     // git describe --tags --dirty, e.g. "v0.12-6-g42f3cb8-dirty"
    std::string commit;       // short hash
    std::string branch;
    std::string commitDate;   // ISO 8601, the commit's (not the build's)
    bool        dirty = false;   // uncommitted changes were in the tree at build time

    std::string config;       // "Debug" / "Release"
    std::string arch;         // "x64" / "x86"
    std::string compiler;     // "MSVC 19.50.35717"
    std::string builtAt;      // the exe's last-write time, local, "YYYY-MM-DD HH:MM:SS"
    std::string exePath;

    std::vector<BuildSdk> sdks;
};

// Computed once, on first call.
const BuildInfo& GetBuildInfo();

// One line for logs: "v0.12-6-g42f3cb8-dirty (master) Release x64, built 2026-10-08 14:02:11".
std::string BuildSummary();

} // namespace firedog
