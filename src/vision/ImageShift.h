// ImageShift.h : how far one picture of a scene moved against another — plain
// C++ normalised cross-correlation, no HALCON. Design: docs/mosaic-design.md.
//
// This is REGISTRATION, not detection: both images are of the same patch of
// the same part, a moment and a known stage move apart, and the only question
// is the pixel offset between them. It serves the mosaic twice - measuring the
// camera's pixel->gantry map (mosaic_calibrate) and tightening the seams
// between neighbouring tiles - and it must work on a machine with no vision
// licence for the reason auto-focus must: a licence problem may degrade
// inspection, never the machine's picture of itself. Feature DETECTION (find a
// fiducial, a corner, a circle) remains HALCON's, in HalconDetector.
//
// Inputs are 8-bit grey planes from RgbToGray8 (vision/FocusMetric), the one
// RGB->luma conversion.
#pragma once

namespace firedog {

struct ShiftResult {
    bool   ok = false;
    double dx = 0.0, dy = 0.0;   // pixels, sub-pixel
    double score = 0.0;          // zero-mean NCC at the peak, -1..1
};

// Match the template rect (tx, ty, tw, th) of `a` inside `b`, searching every
// integer offset d with |d - (cx, cy)| <= r per axis, and report the d for
// which b(p + d) best matches a(p). Offsets that would take the window outside
// `b` are skipped. Refined to sub-pixel by a parabola through the peak's
// neighbours. ok = false when no offset fits or the template is flat.
ShiftResult MatchTemplateNcc(const unsigned char* a, int aw, int ah, int tx, int ty, int tw, int th,
                             const unsigned char* b, int bw, int bh, double cx, double cy, int r);

// The shift d between two frames of the same size (b(p + d) ~ a(p)) for any
// |d| up to `maxShift` px per axis: the centre half of `a` searched
// exhaustively on a coarse pyramid level, then refined level by level. For
// shifts up to about a quarter of the frame.
ShiftResult EstimateFrameShift(const unsigned char* a, const unsigned char* b, int w, int h,
                               int maxShift);

} // namespace firedog
