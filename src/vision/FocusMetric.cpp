// FocusMetric.cpp : Laplacian-variance sharpness. See FocusMetric.h.
#include "vision/FocusMetric.h"

#include <vector>

namespace firedog {

namespace {

// ITU-R BT.601 luma. The Basler feed is mono promoted to RGB, so all three
// channels usually carry the same value — the weights cost nothing and keep the
// measure correct if a colour camera is ever fitted.
inline float Luma(const unsigned char* p)
{
    return 0.299f * p[0] + 0.587f * p[1] + 0.114f * p[2];
}

// Clamp an ROI to the image; an empty/oversized one becomes the whole frame.
FocusRoi ClampRoi(int imgW, int imgH, FocusRoi r)
{
    if (r.w <= 0 || r.h <= 0) { r.x = 0; r.y = 0; r.w = imgW; r.h = imgH; }
    if (r.x < 0) { r.w += r.x; r.x = 0; }
    if (r.y < 0) { r.h += r.y; r.y = 0; }
    if (r.x + r.w > imgW) r.w = imgW - r.x;
    if (r.y + r.h > imgH) r.h = imgH - r.y;
    if (r.w < 0) r.w = 0;
    if (r.h < 0) r.h = 0;
    return r;
}

// Copy the ROI's luma into a dense w*h plane. One pass over the source, so the
// Laplacian below reads a contiguous buffer instead of striding the RGB rows.
void ExtractLuma(const unsigned char* rgb, int imgW, const FocusRoi& r,
                 std::vector<float>& out)
{
    out.resize(static_cast<size_t>(r.w) * r.h);
    for (int y = 0; y < r.h; ++y) {
        const unsigned char* src = rgb + (static_cast<size_t>(r.y + y) * imgW + r.x) * 3;
        float*               dst = out.data() + static_cast<size_t>(y) * r.w;
        for (int x = 0; x < r.w; ++x, src += 3) dst[x] = Luma(src);
    }
}

} // namespace

FocusRoi CenteredRoi(int imgW, int imgH, int roiW, int roiH)
{
    FocusRoi r;
    if (roiW <= 0 || roiH <= 0 || roiW >= imgW || roiH >= imgH) {
        r.x = 0; r.y = 0; r.w = imgW; r.h = imgH;
        return r;
    }
    r.w = roiW;
    r.h = roiH;
    r.x = (imgW - roiW) / 2;
    r.y = (imgH - roiH) / 2;
    return r;
}

double FocusScoreRgb(const unsigned char* rgb, int imgW, int imgH, const FocusRoi& roi)
{
    if (!rgb || imgW <= 0 || imgH <= 0) return 0.0;
    const FocusRoi r = ClampRoi(imgW, imgH, roi);
    if (r.w < 3 || r.h < 3) return 0.0;   // the 4-neighbour kernel needs a border

    std::vector<float> luma;
    ExtractLuma(rgb, imgW, r, luma);

    // 4-neighbour Laplacian over the interior, accumulating sum and sum of
    // squares in one pass: variance = E[x^2] - E[x]^2. Doubles for the
    // accumulators — a 1 MP ROI of |lap| ~ 100 overflows float precision.
    double sum = 0.0, sumSq = 0.0;
    const int stride = r.w;
    for (int y = 1; y < r.h - 1; ++y) {
        const float* row  = luma.data() + static_cast<size_t>(y) * stride;
        const float* up   = row - stride;
        const float* down = row + stride;
        for (int x = 1; x < r.w - 1; ++x) {
            const double lap = 4.0 * row[x] - row[x - 1] - row[x + 1] - up[x] - down[x];
            sum   += lap;
            sumSq += lap * lap;
        }
    }
    const double n = static_cast<double>(r.w - 2) * (r.h - 2);
    if (n <= 0.0) return 0.0;
    const double mean = sum / n;
    const double var  = sumSq / n - mean * mean;
    return var > 0.0 ? var : 0.0;   // clamp the last-bit negative from rounding
}

double MeanLumaRgb(const unsigned char* rgb, int imgW, int imgH, const FocusRoi& roi)
{
    if (!rgb || imgW <= 0 || imgH <= 0) return 0.0;
    const FocusRoi r = ClampRoi(imgW, imgH, roi);
    if (r.w <= 0 || r.h <= 0) return 0.0;

    double sum = 0.0;
    for (int y = 0; y < r.h; ++y) {
        const unsigned char* src = rgb + (static_cast<size_t>(r.y + y) * imgW + r.x) * 3;
        for (int x = 0; x < r.w; ++x, src += 3) sum += Luma(src);
    }
    return sum / (static_cast<double>(r.w) * r.h);
}

} // namespace firedog
