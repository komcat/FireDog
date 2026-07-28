// IoSim.h : shared input pattern for the simulated IO paths (the EziIO stub
// build and WAGO's "simulated" config mode). Input pin p toggles every
// (p+1) seconds so each row blinks at its own rate in the UI.
#pragma once

#include <cstdint>

namespace firedog {

inline uint64_t SimIoInputs(double tSec, int count)
{
    uint64_t v = 0;
    for (int p = 0; p < count; ++p) {
        if (static_cast<long long>(tSec / (p + 1)) & 1) v |= (1ull << p);
    }
    return v;
}

} // namespace firedog
