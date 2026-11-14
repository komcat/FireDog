// EdgeExtreme.cpp : see EdgeExtreme.h.
#include "vision/EdgeExtreme.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace firedog {

const char* EdgeSideName(int side)
{
    switch (side) {
    case 1:  return "right";
    case 2:  return "top";
    case 3:  return "bottom";
    default: return "left";
    }
}

EdgeExtremeResult FindEdgeExtreme(const unsigned char* gray, int w, int h,
                                  double cx, double cy, const EdgeExtremeParams& p)
{
    EdgeExtremeResult r;
    if (!gray || w <= 0 || h <= 0) {
        r.error = "no image";
        return r;
    }

    // The box, centred on stage 1's point + the offset, clamped to the plane.
    const int bw = std::max(2, p.boxW), bh = std::max(2, p.boxH);
    const double bcx = cx + p.offX, bcy = cy + p.offY;
    int x0 = static_cast<int>(std::lround(bcx - bw * 0.5));
    int y0 = static_cast<int>(std::lround(bcy - bh * 0.5));
    int x1 = x0 + bw, y1 = y0 + bh;   // exclusive
    x0 = std::clamp(x0, 0, w);  x1 = std::clamp(x1, 0, w);
    y0 = std::clamp(y0, 0, h);  y1 = std::clamp(y1, 0, h);
    r.boxX = x0;  r.boxY = y0;  r.boxW = x1 - x0;  r.boxH = y1 - y0;
    if (r.boxW < 2 || r.boxH < 2) {
        r.error = "the search box is off the image";
        return r;
    }

    const int  side     = (p.side >= 0 && p.side <= 3) ? p.side : 0;
    const bool rows     = side == 0 || side == 1;            // rays along rows?
    const bool fromLow  = side == 0 || side == 2;            // start at x0 / y0?
    const double thr    = static_cast<double>(p.threshold);
    const auto  pass    = [&](double v) { return p.polarity == 0 ? v >= thr : v <= thr; };
    const auto  px      = [&](int x, int y) { return static_cast<double>(gray[y * w + x]); };

    // One ray per row (left/right) or column (top/bottom). `d` = where it
    // stopped along the ray, NaN = it crossed the whole box without an edge.
    const int across0 = rows ? y0 : x0, across1 = rows ? y1 : x1;
    const int along0  = rows ? x0 : y0, along1  = rows ? x1 : y1;
    const int n = across1 - across0;
    std::vector<double> d(static_cast<std::size_t>(n), std::nan(""));
    for (int i = 0; i < n; ++i) {
        const int a = across0 + i;
        const int len = along1 - along0;
        for (int k = 0; k < len; ++k) {
            const int s = fromLow ? along0 + k : along1 - 1 - k;
            const double v = rows ? px(s, a) : px(a, s);
            if (!pass(v)) continue;
            // Sub-pixel along the ray: where the line from the last non-edge
            // pixel to this one crosses the threshold. The first pixel of the
            // ray has no predecessor and stays whole.
            double pos = static_cast<double>(s);
            if (k > 0) {
                const int sp = fromLow ? s - 1 : s + 1;
                const double vp = rows ? px(sp, a) : px(a, sp);
                if (v != vp) {
                    const double t = std::clamp((thr - vp) / (v - vp), 0.0, 1.0);
                    pos = static_cast<double>(sp) + (fromLow ? t : -t);
                }
            }
            d[static_cast<std::size_t>(i)] = pos;
            break;
        }
    }
    r.rays = n;

    // Continuity: a stop counts only with a neighbour ray close to it.
    std::vector<char> valid(static_cast<std::size_t>(n), 0);
    for (int i = 0; i < n; ++i) {
        const double di = d[static_cast<std::size_t>(i)];
        if (std::isnan(di)) continue;
        for (int j : {i - 1, i + 1}) {
            if (j < 0 || j >= n) continue;
            const double dj = d[static_cast<std::size_t>(j)];
            if (!std::isnan(dj) && std::fabs(dj - di) <= p.continuityPx) {
                valid[static_cast<std::size_t>(i)] = 1;
                break;
            }
        }
        if (valid[static_cast<std::size_t>(i)]) ++r.hits;
    }
    if (r.hits < std::max(1, p.minRays)) {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "no %s edge in the box - %d of %d rays found one (needs %d); "
                      "check the threshold, the box and the filter",
                      EdgeSideName(side), r.hits, n, std::max(1, p.minRays));
        r.error = buf;
        return r;
    }

    // The extreme, then the rays that share it.
    double ext = fromLow ? 1e300 : -1e300;
    for (int i = 0; i < n; ++i) {
        if (!valid[static_cast<std::size_t>(i)]) continue;
        const double di = d[static_cast<std::size_t>(i)];
        ext = fromLow ? std::min(ext, di) : std::max(ext, di);
    }
    double sumA = 0.0;
    for (int i = 0; i < n; ++i) {
        if (!valid[static_cast<std::size_t>(i)]) continue;
        if (std::fabs(d[static_cast<std::size_t>(i)] - ext) > std::max(0.0, p.bandPx)) continue;
        sumA += static_cast<double>(across0 + i);
        ++r.apex;
    }
    const double across = sumA / static_cast<double>(r.apex);
    r.x  = rows ? ext : across;
    r.y  = rows ? across : ext;
    r.ok = true;
    return r;
}

} // namespace firedog
