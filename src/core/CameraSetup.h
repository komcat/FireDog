// CameraSetup.h : how a camera view is looked at - acquisition + display.
//
// Two halves, applied differently, kept in one record because an operator
// tunes them together against one picture:
//
//   ACQUISITION  exposure + lighting. Physical: they change the frame every
//                consumer sees (a detector, a focus score, a saved image).
//   DISPLAY      a ViewFilter - an ROI square on the view's centre/crosshair
//                and a binary threshold drawn over it. Display ONLY: no mark,
//                click or measurement is ever taken on the filtered picture.
//
// Plain data, no ImGui, no device: a recipe command (worker thread), the
// recipe editor's helper and a page all hold the same struct. Applying it is
// Workflow::ApplyCameraSetup (worker, blocking light) or App::ApplyCameraSetupNow
// (UI thread, non-blocking light). docs/camera-setup-design.md.
#pragma once

#include "core/AlignLighting.h"

#include <string>

namespace firedog {

// The live ROI filter of one view. Computed by vision/RoiFilterOps, drawn by
// ui/RoiFilter. roiMode: 0 = off (whole frame), 1 = 1/3, 2 = 1/2, 3 = 2/3 of
// the frame's short side.
struct ViewFilter {
    int  roiMode = 0;
    bool binary  = false;
    int  thr     = 128;     // binary: grey >= thr is white

    // Anything to draw: an ROI outline or a binary picture.
    bool Active() const { return roiMode > 0 || binary; }
    void Sanitize();        // clamp a hand-edited file or an out-of-range param
    bool operator==(const ViewFilter&) const = default;
};

struct CameraSetup {
    double        exposureUs = 0.0;   // 0 = leave the camera's exposure as it is
    AlignLighting lighting;           // kind "" = leave the light as it is
    ViewFilter    filter;             // display only

    bool operator==(const CameraSetup&) const = default;
};

// One line for logs and step summaries:
// "4518 us, main_camera_light white 27 %, ROI 1/2 binary 94".
std::string ViewFilterSummary(const ViewFilter& f);
std::string CameraSetupSummary(const CameraSetup& s);

} // namespace firedog
