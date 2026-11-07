// RoiFilterOps.h : the pixel maths of a ViewFilter (core/CameraSetup.h).
//
// Plain C++ over RgbToGray8 (the one luma conversion), so it needs no HALCON
// licence - the manual picks that use it fall under the auto-focus rule. It is
// NOT a detector's pre-process chain (HalconDetector): no detector reads its
// output; the view draws it, nothing else. Layer 1.5 so a worker (a recipe
// command's auto threshold) and the UI (ui/RoiFilter) compute the same picture.
#pragma once

#include "core/CameraSetup.h"     // ViewFilter
#include "vision/FocusMetric.h"   // FocusRoi

#include <cstddef>
#include <vector>

namespace firedog {

// The filter's ROI in camera px: a square of the mode's fraction of the short
// side, centred on (cx, cy) - the image centre when both are <= 0 (no
// crosshair placed) - and slid back inside the frame rather than shrunk at an
// edge. Mode 0 is the whole frame.
FocusRoi ViewFilterRect(int roiMode, int imgW, int imgH, float cx, float cy);

// Otsu's threshold over an 8-bit grey plane: the level that maximises the
// between-class variance of the two populations it splits.
int OtsuThreshold(const unsigned char* gray, std::size_t n);

// The filter's picture: the ROI cropped to grey into `out` (row-major,
// used.w * used.h), thresholded to 0/255 when `f.binary`. `asked` is
// ViewFilterRect's rectangle; the return is the CLAMPED one RgbToGray8 used,
// whose origin a caller adds back (every coordinate stays full-frame). A
// zero-size return means nothing was produced.
FocusRoi ViewFilterGray(const unsigned char* rgb, int imgW, int imgH, const ViewFilter& f,
                        const FocusRoi& asked, std::vector<unsigned char>& out);

// Otsu over the filter's ROI of a frame - what an "Auto" threshold means,
// wherever it is asked. -1 when the frame yields no pixels.
int ViewFilterAutoThreshold(const unsigned char* rgb, int imgW, int imgH, int roiMode,
                            float cx, float cy, std::vector<unsigned char>& scratch);

} // namespace firedog
