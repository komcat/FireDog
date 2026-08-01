// HalconEngine.h : MVTec HALCON availability — build, runtime and LICENSE.
//
// HALCON is optional in three independent ways, and all three have to be true
// before a HALCON-backed operator may run:
//
//   1. compiled in  — HAS_HALCON was defined at build time (the SDK was found).
//   2. runtime      — halcon.dll / halconcpp.dll actually load in this process.
//   3. licensed     — a licence file was found AND a licensed operator ran.
//
// Any of them can fail on a machine that builds fine: a dev box with the SDK
// but an expired .dat, a production PC with neither. This class answers "may I
// call HALCON?" with one flag and, when the answer is no, a reason string the
// UI can show verbatim. Nothing else in FireDog is allowed to call HALCON
// without asking here first — that is what keeps a missing licence a message
// instead of a crash.
//
// Why the DLLs are probed with LoadLibrary before any HALCON call: the SDK is
// linked with /DELAYLOAD, so the exe starts on a machine with no HALCON at all,
// but the first call to a missing delay-loaded import raises a Win32 SEH
// exception (0xC06D007E) that a C++ catch does NOT handle. Probing the module
// first means we never make that call.
//
// PIMPL, like every driver here: no HALCON header ever reaches our headers.
#pragma once

#include <string>
#include <vector>

namespace firedog {

struct HalconStatus {
    bool compiledIn    = false;   // HAS_HALCON at build time
    bool runtimeLoaded = false;   // halcon.dll + halconcpp.dll loaded
    bool licensed      = false;   // a licensed operator actually succeeded

    std::string root;             // HALCONROOT (build-time path if env is unset)
    std::string arch;             // HALCONARCH, e.g. "x64-win64"
    std::string version;          // reported by HALCON, e.g. "24.05"
    std::string licenseDir;       // where licence files were looked for
    std::vector<std::string> licenseFiles;   // *.dat found there

    std::string error;            // why it is unavailable ("" when Available)

    // True only when all three gates pass.
    bool Available() const { return compiledIn && runtimeLoaded && licensed; }
    // One line for logs and the UI: "HALCON 24.05 ready" / "HALCON unavailable: ...".
    std::string Summary() const;
};

class HalconEngine {
public:
    // Process-wide. First use probes; the result is cached because the licence
    // check costs a real operator call.
    static HalconEngine& Instance();

    const HalconStatus& Status() const { return status_; }
    bool Available() const { return status_.Available(); }

    // Re-run the probe. Cheap enough for a UI button: the point is that an
    // operator can drop a .dat into HALCONROOT/license and re-check without
    // restarting FireDog.
    const HalconStatus& Refresh();

private:
    HalconEngine();
    ~HalconEngine();
    HalconEngine(const HalconEngine&) = delete;
    HalconEngine& operator=(const HalconEngine&) = delete;

    HalconStatus status_;
};

} // namespace firedog
