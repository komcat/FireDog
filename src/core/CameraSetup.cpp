// CameraSetup.cpp
#include "core/CameraSetup.h"

#include <algorithm>
#include <cstdio>

namespace firedog {

void ViewFilter::Sanitize()
{
    if (roiMode < 0 || roiMode > 3) roiMode = 0;
    thr = std::clamp(thr, 0, 255);
}

std::string ViewFilterSummary(const ViewFilter& f)
{
    static const char* kRoi[4] = {"off", "1/3", "1/2", "2/3"};
    const int m = (f.roiMode >= 0 && f.roiMode <= 3) ? f.roiMode : 0;
    char buf[64];
    if (f.binary) std::snprintf(buf, sizeof(buf), "ROI %s binary %d", kRoi[m], f.thr);
    else          std::snprintf(buf, sizeof(buf), "ROI %s", kRoi[m]);
    return buf;
}

std::string CameraSetupSummary(const CameraSetup& s)
{
    std::string out;
    if (s.exposureUs > 0.0) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f us", s.exposureUs);
        out = buf;
    } else {
        out = "exposure as is";
    }
    out += ", ";
    out += s.lighting.Set() ? AlignLightingSummary(s.lighting) : std::string("light as is");
    out += ", ";
    out += ViewFilterSummary(s.filter);
    return out;
}

} // namespace firedog
