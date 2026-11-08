// ProcessMemory.cpp : see ProcessMemory.h.
#include "core/ProcessMemory.h"

#include <cstdio>

// Last, and narrowed, the Paths.cpp way: windows.h's min/max macros would
// otherwise break the standard headers above.
#ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
  #define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>   // PROCESS_MEMORY_COUNTERS_EX; K32GetProcessMemoryInfo is in kernel32

namespace firedog {

bool SampleProcessMemory(ProcessMemory* out)
{
    if (!out) return false;
    PROCESS_MEMORY_COUNTERS_EX pmc{};
    pmc.cb = sizeof(pmc);
    // The K32 entry point lives in kernel32 itself, so nothing extra to link.
    if (!K32GetProcessMemoryInfo(GetCurrentProcess(),
                                 reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pmc),
                                 sizeof(pmc)))
        return false;
    out->privateMB = static_cast<double>(pmc.PrivateUsage) / (1024.0 * 1024.0);
    out->workingMB = static_cast<double>(pmc.WorkingSetSize) / (1024.0 * 1024.0);
    return true;
}

std::string ProcessMemorySummary()
{
    ProcessMemory m;
    if (!SampleProcessMemory(&m)) return std::string();
    char buf[96];
    std::snprintf(buf, sizeof(buf), "private %.0f MB, working set %.0f MB", m.privateMB,
                  m.workingMB);
    return buf;
}

} // namespace firedog
