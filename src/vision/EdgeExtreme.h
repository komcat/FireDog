// EdgeExtreme.h : stage 2 of a detection - the extreme point of an edge.
//
// Stage 1 (the model's method: a pattern match, a circle, a corner) only has to
// land NEAR the feature - a low NCC score is fine. Stage 2 looks in a box around
// that point, on the SAME pre-processed plane the method saw, and casts one ray
// per row (or column) from the named side of the box. Each ray stops at the
// first pixel that is an edge; the extreme of those stops is the answer:
//
//     side = left    rays run left -> right along every row, the reported
//                    point is the SMALLEST x reached (the leftmost edge point)
//     side = right   right -> left, the largest x
//     side = top     top -> bottom down every column, the smallest y
//     side = bottom  bottom -> top, the largest y
//
// That is the "apex" of a curved edge - a lens rim, a fibre end - which no
// template can pin: the template only says roughly where the rim is, the rays
// say where it actually turns. Along the ray the crossing is sub-pixel (linear
// interpolation to the threshold); across it, the reported coordinate is the
// mean of the rays within `bandPx` of the extreme, so a flat stretch reports
// its middle rather than whichever row happened to win.
//
// A speckle is not an edge: a ray's stop counts only when a NEIGHBOURING ray
// stopped within `continuityPx` of it (an edge is continuous across rays, a
// dot of noise is not).
//
// Plain C++, no SDK - like the focus metric, and for the same reason: it is
// arithmetic on a grey plane the caller already has. The pre-process chain in
// front of it is HALCON's (HalconDetector::Detect), and the chain is what makes
// "first pixel over the threshold" mean "the edge": a canny or binary output
// is 0 / 255.
#pragma once

#include <string>

namespace firedog {

// The parameters live on the model (VisionModel::edge); this is their shape.
struct EdgeExtremeParams {
    bool   enabled      = false;
    int    side         = 0;       // 0 left, 1 right, 2 top, 3 bottom
    int    boxW         = 200;     // search box, px, centred on stage 1's point + offset
    int    boxH         = 300;
    int    offX         = 0;       // box centre relative to stage 1's point, px
    int    offY         = 0;
    int    threshold    = 128;     // a ray stops at the first pixel past this grey
    int    polarity     = 0;       // 0 = edge is BRIGHT (>= threshold), 1 = DARK (<=)
    double bandPx       = 2.0;     // rays within this of the extreme form the apex
    double continuityPx = 3.0;     // a stop needs a neighbour ray within this
    int    minRays      = 5;       // fewer valid stops = no edge
};

struct EdgeExtremeResult {
    bool        ok = false;
    std::string error;
    double      x = 0.0, y = 0.0;          // the extreme point, plane px
    int         rays = 0, hits = 0;        // rays cast / valid stops
    int         apex = 0;                  // rays averaged into the reported point
    int         boxX = 0, boxY = 0, boxW = 0, boxH = 0;   // the clamped box, plane px
};

// `gray` is w*h bytes, row-major - the pre-processed plane. (cx, cy) is stage
// 1's point in the same plane's pixels.
EdgeExtremeResult FindEdgeExtreme(const unsigned char* gray, int w, int h,
                                  double cx, double cy, const EdgeExtremeParams& p);

const char* EdgeSideName(int side);   // "left" / "right" / "top" / "bottom"

} // namespace firedog
