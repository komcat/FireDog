// ProcessMemory.h : how much memory this process holds right now - a probe
// for chasing growth (a leak, or a cache that never stops filling).
//
// Private bytes are what the process has committed for itself (heap, HALCON's
// caches, driver allocations made on its behalf): the number that climbs when
// something is never freed. The working set is what is resident in RAM, which
// Windows trims and refills, so it wanders even when nothing leaks.
#pragma once

#include <string>

namespace firedog {

struct ProcessMemory {
    double privateMB = 0.0;   // committed private bytes
    double workingMB = 0.0;   // resident working set
};

// False when Windows does not answer (never in practice).
bool SampleProcessMemory(ProcessMemory* out);

// "private 812 MB, working set 640 MB" - for a log line. "" on failure.
std::string ProcessMemorySummary();

} // namespace firedog
