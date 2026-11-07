// RoiFilterOps.cpp : see RoiFilterOps.h.
#include "vision/RoiFilterOps.h"

#include <algorithm>

namespace firedog {

FocusRoi ViewFilterRect(int roiMode, int imgW, int imgH, float cx, float cy)
{
    if (roiMode <= 0 || roiMode > 3) return FocusRoi{0, 0, imgW, imgH};
    static const double kFrac[4] = {1.0, 1.0 / 3.0, 0.5, 2.0 / 3.0};
    const int side = std::max(8, static_cast<int>((imgW < imgH ? imgW : imgH) * kFrac[roiMode]));
    if (cx <= 0.0f && cy <= 0.0f) { cx = imgW * 0.5f; cy = imgH * 0.5f; }   // not placed yet
    int x0 = static_cast<int>(cx) - side / 2;
    int y0 = static_cast<int>(cy) - side / 2;
    x0 = std::clamp(x0, 0, std::max(0, imgW - side));
    y0 = std::clamp(y0, 0, std::max(0, imgH - side));
    return FocusRoi{x0, y0, std::min(side, imgW), std::min(side, imgH)};
}

int OtsuThreshold(const unsigned char* g, std::size_t n)
{
    if (!g || n == 0) return 128;
    double hist[256] = {};
    for (std::size_t i = 0; i < n; ++i) hist[g[i]] += 1.0;
    const double total = static_cast<double>(n);
    double sumAll = 0.0;
    for (int i = 0; i < 256; ++i) sumAll += i * hist[i];
    double wB = 0.0, sumB = 0.0, best = -1.0;
    int    thr = 128;
    for (int t = 0; t < 256; ++t) {
        wB += hist[t];
        if (wB <= 0.0) continue;
        const double wF = total - wB;
        if (wF <= 0.0) break;
        sumB += t * hist[t];
        const double mB = sumB / wB, mF = (sumAll - sumB) / wF;
        const double between = wB * wF * (mB - mF) * (mB - mF);
        if (between > best) { best = between; thr = t + 1; }
    }
    return thr > 255 ? 255 : thr;
}

FocusRoi ViewFilterGray(const unsigned char* rgb, int imgW, int imgH, const ViewFilter& f,
                        const FocusRoi& asked, std::vector<unsigned char>& out)
{
    if (!rgb || imgW <= 0 || imgH <= 0) { out.clear(); return FocusRoi{}; }
    const FocusRoi got = RgbToGray8(rgb, imgW, imgH, asked, out);
    if (got.w <= 0 || got.h <= 0) return FocusRoi{};
    if (f.binary) {
        const std::size_t n = static_cast<std::size_t>(got.w) * static_cast<std::size_t>(got.h);
        const unsigned char t = static_cast<unsigned char>(std::clamp(f.thr, 0, 255));
        for (std::size_t i = 0; i < n; ++i) out[i] = out[i] >= t ? 255 : 0;
    }
    return got;
}

int ViewFilterAutoThreshold(const unsigned char* rgb, int imgW, int imgH, int roiMode,
                            float cx, float cy, std::vector<unsigned char>& scratch)
{
    if (!rgb || imgW <= 0 || imgH <= 0) return -1;
    const FocusRoi got = RgbToGray8(rgb, imgW, imgH,
                                    ViewFilterRect(roiMode, imgW, imgH, cx, cy), scratch);
    if (got.w <= 0 || got.h <= 0) return -1;
    return OtsuThreshold(scratch.data(),
                         static_cast<std::size_t>(got.w) * static_cast<std::size_t>(got.h));
}

} // namespace firedog
