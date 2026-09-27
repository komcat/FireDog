// ImageShift.cpp
#include "vision/ImageShift.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace firedog {

namespace {

struct Plane {
    std::vector<unsigned char> px;
    int w = 0, h = 0;
};

Plane Half(const unsigned char* p, int w, int h)
{
    Plane o;
    o.w = std::max(1, w / 2);
    o.h = std::max(1, h / 2);
    o.px.resize(static_cast<std::size_t>(o.w) * o.h);
    for (int y = 0; y < o.h; ++y) {
        const unsigned char* r0 = p + static_cast<std::size_t>(std::min(2 * y, h - 1)) * w;
        const unsigned char* r1 = p + static_cast<std::size_t>(std::min(2 * y + 1, h - 1)) * w;
        for (int x = 0; x < o.w; ++x) {
            const int x0 = std::min(2 * x, w - 1), x1 = std::min(2 * x + 1, w - 1);
            o.px[static_cast<std::size_t>(y) * o.w + x] =
                static_cast<unsigned char>((r0[x0] + r0[x1] + r1[x0] + r1[x1] + 2) / 4);
        }
    }
    return o;
}

// Zero-mean NCC of the template against b's window at offset (dx, dy). False
// when the window leaves b.
bool NccAt(const unsigned char* a, int aw, int tx, int ty, int tw, int th, double meanA,
           double normA, const unsigned char* b, int bw, int bh, int dx, int dy, double* out)
{
    const int bx = tx + dx, by = ty + dy;
    if (bx < 0 || by < 0 || bx + tw > bw || by + th > bh) return false;
    double sb = 0.0, sbb = 0.0, sab = 0.0;
    for (int y = 0; y < th; ++y) {
        const unsigned char* ra = a + static_cast<std::size_t>(ty + y) * aw + tx;
        const unsigned char* rb = b + static_cast<std::size_t>(by + y) * bw + bx;
        // Integer row sums: 8-bit samples, a row is far below 2^31 / 65025.
        long long rsb = 0, rsbb = 0, rsab = 0;
        for (int x = 0; x < tw; ++x) {
            const int va = ra[x], vb = rb[x];
            rsb += vb;
            rsbb += vb * vb;
            rsab += va * vb;
        }
        sb += static_cast<double>(rsb);
        sbb += static_cast<double>(rsbb);
        sab += static_cast<double>(rsab);
    }
    const double n    = static_cast<double>(tw) * th;
    const double varB = sbb - sb * sb / n;
    if (!(varB > 1e-6) || !(normA > 1e-6)) {
        *out = 0.0;
        return true;
    }
    *out = (sab - meanA * sb) / (normA * std::sqrt(varB));
    return true;
}

// Vertex of the parabola through (-1, l) (0, c) (1, r), when it is a peak.
double SubPixel(double l, double c, double r)
{
    const double den = l - 2.0 * c + r;
    if (!(den < -1e-12)) return 0.0;
    const double d = 0.5 * (l - r) / den;
    return (d > -1.0 && d < 1.0) ? d : 0.0;
}

}   // namespace

ShiftResult MatchTemplateNcc(const unsigned char* a, int aw, int ah, int tx, int ty, int tw, int th,
                             const unsigned char* b, int bw, int bh, double cx, double cy, int r)
{
    ShiftResult res;
    if (!a || !b || tw < 4 || th < 4 || tx < 0 || ty < 0 || tx + tw > aw || ty + th > ah || r < 0)
        return res;

    double sa = 0.0, saa = 0.0;
    for (int y = 0; y < th; ++y) {
        const unsigned char* ra = a + static_cast<std::size_t>(ty + y) * aw + tx;
        for (int x = 0; x < tw; ++x) {
            sa += ra[x];
            saa += static_cast<double>(ra[x]) * ra[x];
        }
    }
    const double n     = static_cast<double>(tw) * th;
    const double meanA = sa / n;
    const double normA = std::sqrt(std::max(0.0, saa - sa * sa / n));
    // A flat template matches everything equally: under one grey level of
    // spread there is nothing to register on.
    if (!(normA / std::sqrt(n) > 1.0)) return res;

    const int icx = static_cast<int>(std::lround(cx)), icy = static_cast<int>(std::lround(cy));
    const int span = 2 * r + 1;
    std::vector<double> score(static_cast<std::size_t>(span) * span, -2.0);
    double best = -2.0;
    int bxi = 0, byi = 0;
    for (int j = 0; j < span; ++j)
        for (int i = 0; i < span; ++i) {
            double s = 0.0;
            if (!NccAt(a, aw, tx, ty, tw, th, meanA, normA, b, bw, bh, icx - r + i, icy - r + j, &s))
                continue;
            score[static_cast<std::size_t>(j) * span + i] = s;
            if (s > best) {
                best = s;
                bxi  = i;
                byi  = j;
            }
        }
    if (best <= -1.5) return res;

    const auto at = [&](int i, int j) {
        if (i < 0 || j < 0 || i >= span || j >= span) return -2.0;
        return score[static_cast<std::size_t>(j) * span + i];
    };
    double fx = 0.0, fy = 0.0;
    if (at(bxi - 1, byi) > -1.5 && at(bxi + 1, byi) > -1.5)
        fx = SubPixel(at(bxi - 1, byi), best, at(bxi + 1, byi));
    if (at(bxi, byi - 1) > -1.5 && at(bxi, byi + 1) > -1.5)
        fy = SubPixel(at(bxi, byi - 1), best, at(bxi, byi + 1));
    res.ok    = true;
    res.dx    = icx - r + bxi + fx;
    res.dy    = icy - r + byi + fy;
    res.score = best;
    return res;
}

ShiftResult EstimateFrameShift(const unsigned char* a, const unsigned char* b, int w, int h,
                               int maxShift)
{
    ShiftResult res;
    if (!a || !b || w < 32 || h < 32) return res;

    // Pyramids down to ~160 px: small enough to search the whole range
    // exhaustively, large enough to keep some texture.
    std::vector<Plane> pa, pb;
    pa.push_back({std::vector<unsigned char>(a, a + static_cast<std::size_t>(w) * h), w, h});
    pb.push_back({std::vector<unsigned char>(b, b + static_cast<std::size_t>(w) * h), w, h});
    while (std::min(pa.back().w, pa.back().h) > 160 && pa.size() < 7) {
        pa.push_back(Half(pa.back().px.data(), pa.back().w, pa.back().h));
        pb.push_back(Half(pb.back().px.data(), pb.back().w, pb.back().h));
    }

    const int top = static_cast<int>(pa.size()) - 1;
    const Plane& ca = pa[static_cast<std::size_t>(top)];
    const Plane& cb = pb[static_cast<std::size_t>(top)];
    const int f = 1 << top;
    const int r = std::min(maxShift / f + 2, std::min(ca.w, ca.h) / 4);
    res = MatchTemplateNcc(ca.px.data(), ca.w, ca.h, ca.w / 4, ca.h / 4, ca.w / 2, ca.h / 2,
                           cb.px.data(), cb.w, cb.h, 0.0, 0.0, r);
    if (!res.ok) return res;
    for (int l = top - 1; l >= 0; --l) {
        const Plane& la = pa[static_cast<std::size_t>(l)];
        const Plane& lb = pb[static_cast<std::size_t>(l)];
        res = MatchTemplateNcc(la.px.data(), la.w, la.h, la.w / 4, la.h / 4, la.w / 2, la.h / 2,
                               lb.px.data(), lb.w, lb.h, res.dx * 2.0, res.dy * 2.0, 2);
        if (!res.ok) return res;
    }
    return res;
}

} // namespace firedog
