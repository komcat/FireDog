// BuildInfo.cpp : see BuildInfo.h.
#include "core/BuildInfo.h"

#include "firedog_gitinfo.h"   // generated into the build tree by cmake/GitInfo.cmake

#ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
  #define NOMINMAX
#endif
#include <windows.h>

#include <cstdio>

namespace firedog {

namespace {

// The HAS_* defs are PUBLIC on firedog_hw, so this TU sees exactly what every
// driver was compiled with.
#ifdef HAS_PYLON
constexpr bool kPylon = true;
#else
constexpr bool kPylon = false;
#endif
#ifdef HAS_VISA
constexpr bool kVisa = true;
#else
constexpr bool kVisa = false;
#endif
#ifdef HAS_PI
constexpr bool kPi = true;
#else
constexpr bool kPi = false;
#endif
#ifdef HAS_ACS
constexpr bool kAcs = true;
#else
constexpr bool kAcs = false;
#endif
#ifdef HAS_EZIIO
constexpr bool kEziIo = true;
#else
constexpr bool kEziIo = false;
#endif
#ifdef HAS_HALCON
constexpr bool kHalcon = true;
#else
constexpr bool kHalcon = false;
#endif

// The exe's last-write time: when the linker produced it, which is the build
// time that matters (a TU's __TIME__ only moves when that TU recompiles).
std::string ExeWriteTime(const std::string& path)
{
    WIN32_FILE_ATTRIBUTE_DATA fa{};
    if (!GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fa)) return {};
    FILETIME   local{};
    SYSTEMTIME st{};
    if (!FileTimeToLocalFileTime(&fa.ftLastWriteTime, &local) ||
        !FileTimeToSystemTime(&local, &st))
        return {};
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u", st.wYear, st.wMonth,
                  st.wDay, st.wHour, st.wMinute, st.wSecond);
    return buf;
}

BuildInfo Make()
{
    BuildInfo b;
    b.describe   = FD_GIT_DESCRIBE;
    b.commit     = FD_GIT_COMMIT;
    b.branch     = FD_GIT_BRANCH;
    b.commitDate = FD_GIT_DATE;
    const std::string kDirty = "-dirty";
    b.dirty = b.describe.size() > kDirty.size() &&
              b.describe.compare(b.describe.size() - kDirty.size(), kDirty.size(), kDirty) == 0;

#ifdef NDEBUG
    b.config = "Release";
#else
    b.config = "Debug";
#endif
    b.arch = sizeof(void*) == 8 ? "x64" : "x86";
#ifdef _MSC_FULL_VER
    {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "MSVC %d.%d.%d", _MSC_FULL_VER / 10000000,
                      (_MSC_FULL_VER / 100000) % 100, _MSC_FULL_VER % 100000);
        b.compiler = buf;
    }
#else
    b.compiler = "unknown";
#endif

    char path[MAX_PATH] = {};
    if (GetModuleFileNameA(nullptr, path, MAX_PATH)) {
        b.exePath = path;
        b.builtAt = ExeWriteTime(b.exePath);
    }

    b.sdks = {{"Pylon", kPylon}, {"VISA", kVisa},     {"PI GCS2", kPi},
              {"ACS", kAcs},     {"Ezi-IO", kEziIo}, {"HALCON", kHalcon}};
    return b;
}

} // namespace

const BuildInfo& GetBuildInfo()
{
    static const BuildInfo info = Make();
    return info;
}

std::string BuildSummary()
{
    const BuildInfo& b = GetBuildInfo();
    std::string s = b.describe;
    if (b.branch != "unknown" && b.branch != "HEAD") s += " (" + b.branch + ")";
    s += " " + b.config + " " + b.arch;
    if (!b.builtAt.empty()) s += ", built " + b.builtAt;
    return s;
}

} // namespace firedog
