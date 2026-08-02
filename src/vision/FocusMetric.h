// FocusMetric.h : sharpness scoring for auto-focus — plain C++, no vendor SDK.
//
// Layer 1.5: pure compute over a frame buffer. No device access, no state, no
// HALCON/OpenCV — it runs identically whether or not any vision SDK is present,
// which is why auto-focus never depends on the HALCON license (see
// vision/HalconEngine.h). Ported from bobcat_engine's VisionAutoFocusCommands,
// whose cv::Laplacian + meanStdDev pair is exactly the variance below.
//
// It also owns the project's ONE RGB->luma conversion (RgbToGray8 below): every
// consumer of a camera frame that needs 8-bit grey goes through it, so a second
// detector cannot quietly pick different weights and disagree about what a
// pixel is worth.
//
// The measure is the VARIANCE OF THE LAPLACIAN: a sharp image has strong
// second derivatives (edges), a blurred one has weak ones. It is relative, not
// absolute — comparable only between frames of the SAME scene at the same
// exposure and gain, which is precisely the Z-sweep case.
#pragma once

#include <vector>

namespace firedog {

// Pixel rectangle in image coordinates. A zero/negative width or height means
// "whole frame"; the scorer clamps whatever it is given to the image bounds.
struct FocusRoi {
    int x = 0, y = 0, w = 0, h = 0;
};

// Centred square/rect ROI of `roiW` x `roiH` in an imgW x imgH image. Sizes
// larger than the image collapse to the whole frame.
FocusRoi CenteredRoi(int imgW, int imgH, int roiW, int roiH);

// Laplacian variance of an RGB8 buffer (`rgb` holds imgW*imgH*3 bytes, row
// major, no padding) over `roi`. Returns 0 for a null buffer or an ROI smaller
// than 3x3 (the 4-neighbour kernel needs a one-pixel border).
//
// Cost is O(roi.w * roi.h); a 256x256 ROI is well under a millisecond, which is
// what keeps this callable per frame from the poll path.
double FocusScoreRgb(const unsigned char* rgb, int imgW, int imgH, const FocusRoi& roi);

// Mean luma (0-255) over the same ROI. Cheap companion measure: a focus score
// is meaningless when the ROI is saturated or black, and the exposure that
// caused it is the usual reason an auto-focus sweep returns garbage.
double MeanLumaRgb(const unsigned char* rgb, int imgW, int imgH, const FocusRoi& roi);

// Copy `roi` out of an RGB8 buffer as a dense 8-bit grey plane (row major, no
// padding), using the same BT.601 luma as the scorer above. `roi` is clamped to
// the image; a zero/negative size means the whole frame.
//
// RETURNS THE ROI ACTUALLY USED (w = h = 0 on a null/empty buffer). That return
// is the important half of the contract: a detector must add the clamped
// ORIGIN back to whatever it finds to report a full-frame coordinate, and the
// origin it asked for is not necessarily the one it got.
//
// This is the hand-off to any real image processing: a detector reduces the
// search to an ROI here, then hands the dense plane to its SDK. Keeping the
// crop and the conversion in ONE pass is what makes a full-frame call cheap
// enough to sit in a live tuning loop.
FocusRoi RgbToGray8(const unsigned char* rgb, int imgW, int imgH, const FocusRoi& roi,
                    std::vector<unsigned char>& out);

} // namespace firedog
