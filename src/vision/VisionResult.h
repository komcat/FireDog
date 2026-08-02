// VisionResult.h : the one result format every detector returns.
//
// A detection answers "where is the feature, how sure are we, and how big is
// it" — and the answer has to survive three consumers that never see each
// other: the teach wizard's overlay, the DataBus (so any future page can watch
// a detection stream), and `align_station`, which turns a pixel into a machine
// coordinate. One struct for all three, so a new detector cannot quietly grow a
// fourth shape.
//
// Two conventions that are easy to get wrong and are therefore fixed here:
//
//   * `xPx/yPx` are ALWAYS full-frame absolute pixels. A detector that ran on a
//     reduced ROI adds the ROI origin back before returning. bobcat_engine
//     patches its point list but not its measurement map, so half its numbers
//     are ROI-local and half are not; that split is not worth inheriting.
//   * `score` is 0..1 and comparable ACROSS methods, so a reliability run means
//     the same thing whatever the model is. A method whose native figure of
//     merit is an error (gray matching) or a shape feature (circularity) is
//     mapped into that range by the detector, not by the caller.
//
// Plain data — no SDK, no Halcon, so this header is safe everywhere.
#pragma once

#include <string>

namespace firedog {

struct VisionDetection {
    bool        ok = false;
    std::string error;              // why not, when !ok (empty when ok)
    std::string method;             // which detector produced it ("circle"/"ncc"/"corner")
    int         count = 0;          // candidates found; the BEST one is reported

    // Location — full-frame absolute camera pixels, anchor offset already applied.
    double xPx = 0.0, yPx = 0.0;
    // The same point as an offset from the image centre, in mm. Filled by layer 2
    // (Workflow owns the camera's pixel_to_mm calibration); 0 if uncalibrated.
    double dxMM = 0.0, dyMM = 0.0;

    // Pattern match: the match angle. Corner: the direction from the reported
    // corner towards the square's centre (screen convention, y down), which with
    // width/height below reconstructs the whole square from the point. Always 0
    // for a circle, which has no orientation.
    double angleDeg = 0.0;
    double score    = 0.0;          // 0..1, higher is better, cross-method
    double radiusPx = 0.0;          // circle fit
    // Feature size (bounding box). Corner: the two edges leaving the reported
    // corner, width at angleDeg-45 and height at angleDeg+45.
    double widthPx = 0.0, heightPx = 0.0;
    double lengthPx = 0.0;          // corner: the mean side. RESERVED otherwise

    unsigned long long frameSeq = 0;   // which camera frame produced this
    double elapsedMs = 0.0;            // detector wall time, for the ROI warning
};

} // namespace firedog
