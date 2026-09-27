// MosaicLayout.cpp
#include "vision/MosaicLayout.h"

#include "vision/ImageShift.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>

namespace firedog {

namespace {

// A side shorter than this is one corner taught twice, not an edge.
constexpr double kMinSideMm = 0.010;
// Nor is an outline enclosing less than this an area.
constexpr double kMinAreaMm2 = 0.001;

double Cross(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

// Separating-axis test: does the axis-aligned rect [rx0,rx1] x [ry0,ry1]
// intersect the convex quad `q`?
bool RectTouchesQuad(double rx0, double ry0, double rx1, double ry1, const double q[4][2])
{
    double qx0 = q[0][0], qx1 = q[0][0], qy0 = q[0][1], qy1 = q[0][1];
    for (int k = 1; k < 4; ++k) {
        qx0 = std::min(qx0, q[k][0]); qx1 = std::max(qx1, q[k][0]);
        qy0 = std::min(qy0, q[k][1]); qy1 = std::max(qy1, q[k][1]);
    }
    if (rx1 <= qx0 || qx1 <= rx0 || ry1 <= qy0 || qy1 <= ry0) return false;   // the rect's axes

    const double rc[4][2] = {{rx0, ry0}, {rx1, ry0}, {rx1, ry1}, {rx0, ry1}};
    for (int k = 0; k < 4; ++k) {   // the quad's edge normals
        const double ex = q[(k + 1) % 4][0] - q[k][0];
        const double ey = q[(k + 1) % 4][1] - q[k][1];
        const double nx = -ey, ny = ex;
        double p0 = 1e300, p1 = -1e300, s0 = 1e300, s1 = -1e300;
        for (int i = 0; i < 4; ++i) {
            const double pq = q[i][0] * nx + q[i][1] * ny;
            const double pr = rc[i][0] * nx + rc[i][1] * ny;
            p0 = std::min(p0, pq); p1 = std::max(p1, pq);
            s0 = std::min(s0, pr); s1 = std::max(s1, pr);
        }
        if (s1 <= p0 || p1 <= s0) return false;
    }
    return true;
}

}   // namespace

// ---- Geometry ----------------------------------------------------------------------

bool MosaicGeomMake(const double mmPerPx[4], double g0x, double g0y, MosaicGeom* out)
{
    const double det = mmPerPx[0] * mmPerPx[3] - mmPerPx[1] * mmPerPx[2];
    const double scale = std::fabs(mmPerPx[0]) + std::fabs(mmPerPx[1]) +
                         std::fabs(mmPerPx[2]) + std::fabs(mmPerPx[3]);
    if (!(scale > 0.0) || !(std::fabs(det) > 1e-9 * scale * scale)) return false;
    MosaicGeom g;
    for (int i = 0; i < 4; ++i) g.m[i] = mmPerPx[i];
    g.inv[0] =  mmPerPx[3] / det;
    g.inv[1] = -mmPerPx[1] / det;
    g.inv[2] = -mmPerPx[2] / det;
    g.inv[3] =  mmPerPx[0] / det;
    g.g0x = g0x;
    g.g0y = g0y;
    *out = g;
    return true;
}

void MosaicCanvasToGantry(const MosaicGeom& g, double cx, double cy, double* gx, double* gy)
{
    *gx = g.g0x + g.m[0] * cx + g.m[1] * cy;
    *gy = g.g0y + g.m[2] * cx + g.m[3] * cy;
}

void MosaicGantryToCanvas(const MosaicGeom& g, double gx, double gy, double* cx, double* cy)
{
    const double dx = gx - g.g0x, dy = gy - g.g0y;
    *cx = g.inv[0] * dx + g.inv[1] * dy;
    *cy = g.inv[2] * dx + g.inv[3] * dy;
}

void MosaicDefaultM(double px2mmX, double px2mmY, double out[4])
{
    out[0] = px2mmX; out[1] = 0.0;
    out[2] = 0.0;    out[3] = px2mmY;
}

void MosaicEffectiveM(const MosaicCameraCal& cal, double px2mmX, double px2mmY, double out[4])
{
    if (cal.calibrated) {
        for (int i = 0; i < 4; ++i) out[i] = cal.m[i];
        return;
    }
    MosaicDefaultM(px2mmX, px2mmY, out);
}

// ---- The area --------------------------------------------------------------------

bool MosaicAreaQuad(const MosaicArea& a, double q[4][2])
{
    const int n = a.nCorners == 4 ? 4 : 3;
    for (int i = 0; i < n; ++i)
        if (!a.corners[i].have) return false;
    for (int i = 0; i < n; ++i) {
        q[i][0] = a.corners[i].x;
        q[i][1] = a.corners[i].y;
    }
    if (n == 3) {   // the waffle tray's parallelogram: origin, row end, far
        q[3][0] = a.corners[0].x + a.corners[2].x - a.corners[1].x;
        q[3][1] = a.corners[0].y + a.corners[2].y - a.corners[1].y;
    }
    return true;
}

bool MosaicPlaneFit(const MosaicArea& a, double* pa, double* pb, double* pc, std::string* why)
{
    const int n = a.nCorners == 4 ? 4 : 3;
    double mx = 0.0, my = 0.0, mz = 0.0;
    for (int i = 0; i < n; ++i) {
        if (!a.corners[i].have) {
            if (why) *why = std::string(MosaicCornerLabel(n, i)) + " is not taught";
            return false;
        }
        mx += a.corners[i].x; my += a.corners[i].y; mz += a.corners[i].z;
    }
    mx /= n; my /= n; mz /= n;
    double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
    for (int i = 0; i < n; ++i) {
        const double x = a.corners[i].x - mx, y = a.corners[i].y - my, z = a.corners[i].z - mz;
        sxx += x * x; sxy += x * y; syy += y * y; sxz += x * z; syz += y * z;
    }
    const double det = sxx * syy - sxy * sxy;
    // Relative to the spread, so a 1 mm and a 100 mm area are judged alike.
    if (!(det > 1e-9 * (sxx + syy) * (sxx + syy)) || !(sxx + syy > 0.0)) {
        if (why) *why = "the corners are in a line - no plane through them";
        return false;
    }
    const double b = (sxz * syy - syz * sxy) / det;
    const double c = (syz * sxx - sxz * sxy) / det;
    *pb = b;
    *pc = c;
    *pa = mz - b * mx - c * my;
    return true;
}

double MosaicTileZ(const MosaicArea& a, double x, double y)
{
    if (a.zMode == MosaicZMode::Plane) {
        double pa = 0.0, pb = 0.0, pc = 0.0;
        if (MosaicPlaneFit(a, &pa, &pb, &pc, nullptr)) {
            // A frame that only grazes the outline has its centre OUTSIDE it,
            // where the plane is an extrapolation. Held to the Zs the corners
            // were actually taught at: nothing the operator focused on says
            // the surface goes further than that.
            const int n = a.nCorners == 4 ? 4 : 3;
            double z0 = a.corners[0].z, z1 = a.corners[0].z;
            for (int i = 1; i < n; ++i) {
                z0 = std::min(z0, a.corners[i].z);
                z1 = std::max(z1, a.corners[i].z);
            }
            return std::clamp(pa + pb * x + pc * y, z0, z1);
        }
    }
    return a.zFixedMm;
}

std::string MosaicAreaValidate(const MosaicArea& a)
{
    if (a.nCorners != 3 && a.nCorners != 4) return "an area has 3 or 4 corners";
    for (int i = 0; i < a.nCorners; ++i)
        if (!a.corners[i].have)
            return std::string(MosaicCornerLabel(a.nCorners, i)) + " is not taught";

    double q[4][2];
    MosaicAreaQuad(a, q);
    double area2 = 0.0;
    int    pos = 0, neg = 0;
    for (int k = 0; k < 4; ++k) {
        const double* p0 = q[k];
        const double* p1 = q[(k + 1) % 4];
        const double* p2 = q[(k + 2) % 4];
        if (std::hypot(p1[0] - p0[0], p1[1] - p0[1]) < kMinSideMm) {
            char b[160];
            std::snprintf(b, sizeof(b), "corners %d and %d are the same point (< %.3f mm apart)",
                          k + 1, (k + 1) % 4 + 1, kMinSideMm);
            return b;
        }
        const double cr = Cross(p1[0] - p0[0], p1[1] - p0[1], p2[0] - p1[0], p2[1] - p1[1]);
        if (cr > 0.0) ++pos;
        if (cr < 0.0) ++neg;
        area2 += Cross(p0[0], p0[1], p1[0], p1[1]);
    }
    if (std::fabs(area2) * 0.5 < kMinAreaMm2)
        return "the corners enclose no area - they are in a line";
    if (pos != 0 && neg != 0)
        return a.nCorners == 4 ? "the four corners do not go round a convex outline - teach "
                                 "them in order round the edge"
                               : "the three corners do not make a parallelogram";

    if (!(a.overlapPct >= kMosaicMinOverlapPct && a.overlapPct <= kMosaicMaxOverlapPct)) {
        char b[96];
        std::snprintf(b, sizeof(b), "overlap %.1f %% is outside %.0f..%.0f %%", a.overlapPct,
                      kMosaicMinOverlapPct, kMosaicMaxOverlapPct);
        return b;
    }
    if (a.zMode == MosaicZMode::Plane) {
        double pa = 0.0, pb = 0.0, pc = 0.0;
        std::string why;
        if (!MosaicPlaneFit(a, &pa, &pb, &pc, &why)) return "Z plane: " + why;
    }
    return {};
}

// ---- The plan --------------------------------------------------------------------

MosaicPlan PlanMosaic(const MosaicArea& a, const double mmPerPx[4], int imgW, int imgH)
{
    MosaicPlan plan;
    plan.imgW = imgW;
    plan.imgH = imgH;
    plan.why  = MosaicAreaValidate(a);
    if (!plan.why.empty()) return plan;
    if (imgW <= 0 || imgH <= 0) {
        plan.why = "the camera frame size is unknown - connect the camera";
        return plan;
    }
    MosaicGeom unit;
    if (!MosaicGeomMake(mmPerPx, 0.0, 0.0, &unit)) {
        plan.why = "the camera's mm-per-pixel map is singular - check pixel_to_mm / the calibration";
        return plan;
    }

    // The outline in ABSOLUTE camera-frame pixels (g = M * p, no origin yet).
    double q[4][2], p[4][2];
    MosaicAreaQuad(a, q);
    double px0 = 1e300, px1 = -1e300, py0 = 1e300, py1 = -1e300;
    for (int k = 0; k < 4; ++k) {
        MosaicGantryToCanvas(unit, q[k][0], q[k][1], &p[k][0], &p[k][1]);
        px0 = std::min(px0, p[k][0]); px1 = std::max(px1, p[k][0]);
        py0 = std::min(py0, p[k][1]); py1 = std::max(py1, p[k][1]);
    }

    const double ov = a.overlapPct / 100.0;
    const double W = imgW, H = imgH;
    plan.stepX = W * (1.0 - ov);
    plan.stepY = H * (1.0 - ov);
    const double spanX = px1 - px0, spanY = py1 - py0;
    const double colsD = spanX <= W ? 1.0 : std::ceil((spanX - W) / plan.stepX) + 1.0;
    const double rowsD = spanY <= H ? 1.0 : std::ceil((spanY - H) / plan.stepY) + 1.0;
    if (colsD * rowsD > static_cast<double>(kMaxMosaicTiles) * 4.0) {   // before allocating
        char b[200];
        std::snprintf(b, sizeof(b), "%.0f x %.0f frames is far past the %zu-tile limit - "
                      "check the corners and the camera scale", rowsD, colsD, kMaxMosaicTiles);
        plan.why = b;
        return plan;
    }
    plan.cols = static_cast<int>(colsD);
    plan.rows = static_cast<int>(rowsD);

    // Centre the grid on the outline: the spare coverage is split evenly.
    const double covX = W + (plan.cols - 1) * plan.stepX;
    const double covY = H + (plan.rows - 1) * plan.stepY;
    const double sx = px0 - (covX - spanX) * 0.5;
    const double sy = py0 - (covY - spanY) * 0.5;
    double g0x = 0.0, g0y = 0.0;
    MosaicCanvasToGantry(unit, sx, sy, &g0x, &g0y);
    MosaicGeomMake(mmPerPx, g0x, g0y, &plan.geom);
    plan.canvasW = static_cast<int>(std::ceil(covX));
    plan.canvasH = static_cast<int>(std::ceil(covY));
    for (int k = 0; k < 4; ++k) {
        plan.quad[k][0] = p[k][0] - sx;
        plan.quad[k][1] = p[k][1] - sy;
    }

    for (int r = 0; r < plan.rows; ++r) {
        for (int i = 0; i < plan.cols; ++i) {
            const int c = (r % 2 == 0) ? i : plan.cols - 1 - i;   // serpentine
            const double x0 = c * plan.stepX, y0 = r * plan.stepY;
            if (!RectTouchesQuad(x0, y0, x0 + W, y0 + H, plan.quad)) continue;
            MosaicPlanTile t;
            t.row = r;
            t.col = c;
            t.cx  = x0 + W * 0.5;
            t.cy  = y0 + H * 0.5;
            MosaicCanvasToGantry(plan.geom, t.cx, t.cy, &t.gx, &t.gy);
            t.gz = MosaicTileZ(a, t.gx, t.gy);
            plan.tiles.push_back(t);
        }
    }
    if (plan.tiles.empty()) {
        plan.why = "no frame touches the outline";
        return plan;
    }
    if (plan.tiles.size() > kMaxMosaicTiles) {
        char b[160];
        std::snprintf(b, sizeof(b), "%zu tiles is past the %zu-tile limit - a smaller area, or "
                      "less overlap", plan.tiles.size(), kMaxMosaicTiles);
        plan.why = b;
        plan.tiles.clear();
        return plan;
    }
    plan.ok = true;
    return plan;
}

// ---- Composing -------------------------------------------------------------------

void ComposeMosaicRegion(const std::vector<MosaicTileRef>& tiles, int x0, int y0, int outW,
                         int outH, int scale,
                         const std::function<const unsigned char*(std::size_t)>& getTile,
                         std::vector<unsigned char>* out)
{
    if (scale < 1) scale = 1;
    out->assign(static_cast<std::size_t>(std::max(0, outW)) *
                    static_cast<std::size_t>(std::max(0, outH)) * 3u,
                0);
    if (outW <= 0 || outH <= 0) return;

    struct Rect {
        int    x, y, w, h;
        double cx, cy;
    };
    std::vector<Rect> rs;
    rs.reserve(tiles.size());
    for (const MosaicTileRef& t : tiles) {
        Rect r;
        r.x  = static_cast<int>(std::lround(t.x));
        r.y  = static_cast<int>(std::lround(t.y));
        r.w  = t.w;
        r.h  = t.h;
        r.cx = r.x + t.w * 0.5;
        r.cy = r.y + t.h * 0.5;
        rs.push_back(r);
    }

    // Each tile's pixels, fetched once per call and only if something needs them.
    std::vector<const unsigned char*> px(rs.size(), nullptr);
    std::vector<char> fetched(rs.size(), 0);
    const auto pixels = [&](std::size_t i) -> const unsigned char* {
        if (!fetched[i]) {
            fetched[i] = 1;
            px[i] = getTile(i);
        }
        return px[i];
    };

    const int sx0 = x0, sx1 = x0 + outW * scale;
    const double inv = 1.0 / (static_cast<double>(scale) * scale);
    std::vector<std::uint32_t> acc(static_cast<std::size_t>(outW) * 3u);
    std::vector<std::size_t> rowTiles, cand;
    std::vector<int> cuts;

    for (int oy = 0; oy < outH; ++oy) {
        std::fill(acc.begin(), acc.end(), 0u);
        for (int s = 0; s < scale; ++s) {
            const int y = y0 + oy * scale + s;
            rowTiles.clear();
            for (std::size_t i = 0; i < rs.size(); ++i) {
                const Rect& r = rs[i];
                if (y < r.y || y >= r.y + r.h) continue;
                if (r.x + r.w <= sx0 || r.x >= sx1) continue;
                if (!pixels(i)) continue;
                rowTiles.push_back(i);
            }
            if (rowTiles.empty()) continue;

            // Between two consecutive tile edges the set of tiles covering the
            // row is constant; only the nearest-centre choice varies inside.
            cuts.clear();
            cuts.push_back(sx0);
            cuts.push_back(sx1);
            for (std::size_t i : rowTiles) {
                cuts.push_back(std::clamp(rs[i].x, sx0, sx1));
                cuts.push_back(std::clamp(rs[i].x + rs[i].w, sx0, sx1));
            }
            std::sort(cuts.begin(), cuts.end());
            cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());

            for (std::size_t k = 0; k + 1 < cuts.size(); ++k) {
                const int a = cuts[k], b = cuts[k + 1];
                cand.clear();
                for (std::size_t i : rowTiles)
                    if (rs[i].x <= a && a < rs[i].x + rs[i].w) cand.push_back(i);
                if (cand.empty()) continue;
                for (int x = a; x < b; ++x) {
                    std::size_t own = cand[0];
                    if (cand.size() > 1) {
                        double best = 1e300;
                        for (std::size_t i : cand) {
                            const double dx = x + 0.5 - rs[i].cx, dy = y + 0.5 - rs[i].cy;
                            const double d  = dx * dx + dy * dy;
                            if (d < best) { best = d; own = i; }
                        }
                    }
                    const Rect& r = rs[own];
                    const unsigned char* src =
                        px[own] + (static_cast<std::size_t>(y - r.y) * static_cast<std::size_t>(r.w) +
                                   static_cast<std::size_t>(x - r.x)) * 3u;
                    std::uint32_t* dst = &acc[static_cast<std::size_t>((x - x0) / scale) * 3u];
                    dst[0] += src[0];
                    dst[1] += src[1];
                    dst[2] += src[2];
                }
            }
        }
        unsigned char* row = out->data() + static_cast<std::size_t>(oy) * outW * 3u;
        if (scale == 1) {
            for (std::size_t i = 0; i < acc.size(); ++i) row[i] = static_cast<unsigned char>(acc[i]);
        } else {
            for (std::size_t i = 0; i < acc.size(); ++i)
                row[i] = static_cast<unsigned char>(std::min(255.0, acc[i] * inv + 0.5));
        }
    }
}

// ---- Seam refinement ---------------------------------------------------------------

MosaicRefineResult RefineMosaicPlacement(
    const MosaicRefineInput& in,
    const std::function<bool(std::size_t, std::vector<unsigned char>*, int*, int*)>& gray,
    const std::function<bool()>& stop)
{
    MosaicRefineResult res;
    const std::size_t n = in.tiles.size();
    res.x.resize(n);
    res.y.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        res.x[i] = in.tiles[i].x;
        res.y[i] = in.tiles[i].y;
    }
    if (n < 2 || in.rowCol.size() != n) {
        res.why = "fewer than two tiles - nothing to refine";
        return res;
    }
    struct Pair {
        std::size_t i, j;
        double      ox, oy;   // measured P_j - P_i, full-res px
        double      w;
        bool        used;
    };
    std::vector<Pair> pairs;
    bool stopped = false;
    for (const MosaicPairMeasure& pm : MeasureMosaicPairs(in, gray, stop, &res.pairs, &stopped))
        pairs.push_back({pm.i, pm.j, pm.ox, pm.oy, pm.score, true});
    if (stopped) {
        res.why = "stopped";
        return res;
    }

    // Solve: every used pair pulls P_j - P_i toward its measurement, and each
    // tile is held to its stage position by a weak spring (lambda), which
    // anchors the whole and keeps a tile nobody matched where the stage put it.
    constexpr double kLambda = 0.02;
    constexpr double kOutlierPx = 3.0;
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<std::vector<std::pair<std::size_t, std::size_t>>> adj(n);   // (other, pair)
        for (std::size_t p = 0; p < pairs.size(); ++p) {
            if (!pairs[p].used) continue;
            adj[pairs[p].i].push_back({pairs[p].j, p});
            adj[pairs[p].j].push_back({pairs[p].i, p});
        }
        for (int it = 0; it < 5000; ++it) {
            double maxDelta = 0.0;
            for (std::size_t k = 0; k < n; ++k) {
                double sx = kLambda * in.tiles[k].x, sy = kLambda * in.tiles[k].y, sw = kLambda;
                for (const auto& [other, p] : adj[k]) {
                    const Pair& pr = pairs[p];
                    const double sgn = (pr.j == k) ? 1.0 : -1.0;   // P_j = P_i + o
                    sx += pr.w * (res.x[other] + sgn * pr.ox);
                    sy += pr.w * (res.y[other] + sgn * pr.oy);
                    sw += pr.w;
                }
                const double nx = sx / sw, ny = sy / sw;
                maxDelta = std::max(maxDelta, std::max(std::fabs(nx - res.x[k]), std::fabs(ny - res.y[k])));
                res.x[k] = nx;
                res.y[k] = ny;
            }
            if (maxDelta < 1e-3) break;
        }
        // One more pass without the pairs the solution disagrees with: a seam
        // that matched a repeated pattern one period off must not drag its
        // neighbours along.
        bool dropped = false;
        if (pass == 0)
            for (Pair& pr : pairs) {
                if (!pr.used) continue;
                const double ex = res.x[pr.j] - res.x[pr.i] - pr.ox;
                const double ey = res.y[pr.j] - res.y[pr.i] - pr.oy;
                if (std::hypot(ex, ey) > kOutlierPx) {
                    pr.used = false;
                    dropped = true;
                }
            }
        if (!dropped) break;
    }

    double ss = 0.0;
    for (const Pair& pr : pairs) {
        if (!pr.used) continue;
        ++res.used;
        const double ex = res.x[pr.j] - res.x[pr.i] - pr.ox;
        const double ey = res.y[pr.j] - res.y[pr.i] - pr.oy;
        ss += ex * ex + ey * ey;
    }
    if (res.used == 0) {
        for (std::size_t i = 0; i < n; ++i) {
            res.x[i] = in.tiles[i].x;
            res.y[i] = in.tiles[i].y;
        }
        res.why = "no seam matched well enough - too little texture in the overlaps?";
        return res;
    }
    res.rmsPx = std::sqrt(ss / res.used);
    for (std::size_t i = 0; i < n; ++i)
        res.maxMovePx = std::max(res.maxMovePx,
                                 std::hypot(res.x[i] - in.tiles[i].x, res.y[i] - in.tiles[i].y));
    res.ok = true;
    return res;
}

// ---- Pair measurement ----------------------------------------------------------------

std::vector<MosaicPairMeasure> MeasureMosaicPairs(
    const MosaicRefineInput& in,
    const std::function<bool(std::size_t, std::vector<unsigned char>*, int*, int*)>& gray,
    const std::function<bool()>& stop, int* candidates, bool* stopped)
{
    std::vector<MosaicPairMeasure> out;
    if (candidates) *candidates = 0;
    if (stopped) *stopped = false;
    const std::size_t n = in.tiles.size();
    if (n < 2 || in.rowCol.size() != n) return out;
    const int q  = std::max(1, in.scale);
    const int rq = std::max(2, (in.searchPx + q - 1) / q);

    // Neighbour pairs: right and below, in grid order so the grey planes can
    // be dropped a row behind the walk.
    std::map<std::pair<int, int>, std::size_t> cell;
    for (std::size_t i = 0; i < n; ++i) cell[in.rowCol[i]] = i;
    struct Pair {
        std::size_t i, j;
    };
    std::vector<Pair> pairs;
    for (const auto& [rc, i] : cell) {
        const auto right = cell.find({rc.first, rc.second + 1});
        const auto below = cell.find({rc.first + 1, rc.second});
        if (right != cell.end()) pairs.push_back({i, right->second});
        if (below != cell.end()) pairs.push_back({i, below->second});
    }
    if (candidates) *candidates = static_cast<int>(pairs.size());

    struct Plane {
        std::vector<unsigned char> px;
        int  w = 0, h = 0;
        bool ok = false;
    };
    std::map<std::size_t, Plane> planes;
    const auto plane = [&](std::size_t i) -> const Plane& {
        auto it = planes.find(i);
        if (it != planes.end()) return it->second;
        Plane p;
        p.ok = gray(i, &p.px, &p.w, &p.h) && p.w > 0 && p.h > 0;
        return planes.emplace(i, std::move(p)).first->second;
    };

    int lastRow = -1;
    for (const Pair& pr : pairs) {
        if (stop && stop()) {
            if (stopped) *stopped = true;
            return out;
        }
        const int row = in.rowCol[pr.i].first;
        if (row != lastRow) {   // drop what the walk has left behind
            for (auto it = planes.begin(); it != planes.end();)
                it = in.rowCol[it->first].first < row ? planes.erase(it) : std::next(it);
            lastRow = row;
        }
        const Plane& a = plane(pr.i);
        const Plane& b = plane(pr.j);
        if (!a.ok || !b.ok) continue;

        // Where the stage says j sits relative to i, at the planes' scale; the
        // template is i's share of the overlap, shrunk by the search radius so
        // every candidate window stays inside j.
        const double oqx = (in.tiles[pr.j].x - in.tiles[pr.i].x) / q;
        const double oqy = (in.tiles[pr.j].y - in.tiles[pr.i].y) / q;
        const double x0 = std::max(0.0, oqx), x1 = std::min<double>(a.w, oqx + b.w);
        const double y0 = std::max(0.0, oqy), y1 = std::min<double>(a.h, oqy + b.h);
        const int tx = static_cast<int>(std::ceil(x0)) + rq;
        const int ty = static_cast<int>(std::ceil(y0)) + rq;
        const int tw = static_cast<int>(std::floor(x1)) - rq - tx;
        const int th = static_cast<int>(std::floor(y1)) - rq - ty;
        if (tw < 12 || th < 12) continue;   // too thin a sliver to be evidence

        const ShiftResult m = MatchTemplateNcc(a.px.data(), a.w, a.h, tx, ty, tw, th, b.px.data(),
                                               b.w, b.h, -oqx, -oqy, rq);
        if (!m.ok || m.score < in.minScore) continue;
        // A peak on the edge of the search is a match that wanted to be
        // further away: not a measurement of this seam.
        if (std::fabs(m.dx + oqx) >= rq - 0.5 || std::fabs(m.dy + oqy) >= rq - 0.5) continue;
        MosaicPairMeasure pm;
        pm.i     = pr.i;
        pm.j     = pr.j;
        pm.ox    = -m.dx * q;
        pm.oy    = -m.dy * q;
        pm.score = m.score;
        out.push_back(pm);
    }
    return out;
}

// ---- The camera map from a mosaic --------------------------------------------------------

void MosaicMapDescribe(const double m[4], double* rotDeg, double* skewDeg, double* sx, double* sy)
{
    constexpr double kDeg = 180.0 / 3.14159265358979323846;
    // Image x runs along gantry (m0, m2), image y along (m1, m3).
    const double ax = std::atan2(m[2], m[0]), ay = std::atan2(m[3], m[1]);
    double skew = (ay - ax) * kDeg - 90.0;
    while (skew > 180.0) skew -= 360.0;
    while (skew <= -180.0) skew += 360.0;
    if (rotDeg)  *rotDeg  = ax * kDeg;
    if (skewDeg) *skewDeg = skew;
    if (sx)      *sx      = std::hypot(m[0], m[2]);
    if (sy)      *sy      = std::hypot(m[1], m[3]);
}

MosaicMapFit FitMosaicCameraMap(const std::vector<MosaicPairMeasure>& pairs,
                                const std::vector<std::pair<double, double>>& gantry,
                                int candidates)
{
    MosaicMapFit fit;
    fit.pairs = candidates;
    std::vector<char> use(pairs.size(), 1);
    for (std::size_t k = 0; k < pairs.size(); ++k)
        if (pairs[k].i >= gantry.size() || pairs[k].j >= gantry.size()) use[k] = 0;

    // Each pair says: the picture moved p for a gantry move D, and p = A D
    // with A = M^-1 (px per mm). Least squares over every pair gives A's two
    // rows independently; the pairs A disagrees with are dropped and it is
    // solved again, so a seam matched on a repeating pattern cannot tilt it.
    double A[4] = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> resid(pairs.size(), 0.0);
    for (int iter = 0; iter < 5; ++iter) {
        double sxx = 0.0, sxy = 0.0, syy = 0.0, bx0 = 0.0, by0 = 0.0, bx1 = 0.0, by1 = 0.0;
        int n = 0;
        for (std::size_t k = 0; k < pairs.size(); ++k) {
            if (!use[k]) continue;
            const MosaicPairMeasure& pm = pairs[k];
            const double dx = gantry[pm.j].first - gantry[pm.i].first;
            const double dy = gantry[pm.j].second - gantry[pm.i].second;
            const double w  = pm.score;
            sxx += w * dx * dx; sxy += w * dx * dy; syy += w * dy * dy;
            bx0 += w * dx * pm.ox; by0 += w * dy * pm.ox;
            bx1 += w * dx * pm.oy; by1 += w * dy * pm.oy;
            ++n;
        }
        const double det = sxx * syy - sxy * sxy;
        if (n < 3 || !(det > 1e-6 * (sxx + syy) * (sxx + syy))) {
            fit.why = n < 3 ? "fewer than three seams matched - too little texture in the overlaps?"
                            : "the matched seams run in one direction only - the fit needs both "
                              "rows and columns";
            return fit;
        }
        A[0] = (bx0 * syy - by0 * sxy) / det;
        A[1] = (by0 * sxx - bx0 * sxy) / det;
        A[2] = (bx1 * syy - by1 * sxy) / det;
        A[3] = (by1 * sxx - bx1 * sxy) / det;

        std::vector<double> live;
        for (std::size_t k = 0; k < pairs.size(); ++k) {
            const MosaicPairMeasure& pm = pairs[k];
            if (pm.i >= gantry.size() || pm.j >= gantry.size()) continue;
            const double dx = gantry[pm.j].first - gantry[pm.i].first;
            const double dy = gantry[pm.j].second - gantry[pm.i].second;
            resid[k] = std::hypot(A[0] * dx + A[1] * dy - pm.ox, A[2] * dx + A[3] * dy - pm.oy);
            if (use[k]) live.push_back(resid[k]);
        }
        std::nth_element(live.begin(), live.begin() + static_cast<std::ptrdiff_t>(live.size() / 2),
                         live.end());
        const double limit = std::max(2.0, 3.0 * live[live.size() / 2]);
        bool dropped = false;
        for (std::size_t k = 0; k < pairs.size(); ++k)
            if (use[k] && resid[k] > limit) {
                use[k]  = 0;
                dropped = true;
            }
        if (!dropped) break;
    }

    double ss = 0.0;
    for (std::size_t k = 0; k < pairs.size(); ++k)
        if (use[k]) {
            ++fit.used;
            ss += resid[k] * resid[k];
        }
    if (fit.used < 3) {
        fit.why = "too few seams agree with each other to fit a camera map";
        return fit;
    }
    fit.rmsPx = std::sqrt(ss / fit.used);
    const double detA = A[0] * A[3] - A[1] * A[2];
    if (!(std::fabs(detA) > 1e-12)) {
        fit.why = "the fitted map is singular";
        return fit;
    }
    fit.m[0] =  A[3] / detA;
    fit.m[1] = -A[1] / detA;
    fit.m[2] = -A[2] / detA;
    fit.m[3] =  A[0] / detA;
    MosaicMapDescribe(fit.m, &fit.rotDeg, &fit.skewDeg, &fit.scaleX, &fit.scaleY);
    fit.ok = true;
    return fit;
}

void DownsampleRgb(const unsigned char* rgb, int w, int h, int factor,
                   std::vector<unsigned char>* out, int* outW, int* outH)
{
    if (factor < 1) factor = 1;
    const int ow = std::max(1, (w + factor - 1) / factor);
    const int oh = std::max(1, (h + factor - 1) / factor);
    out->assign(static_cast<std::size_t>(ow) * oh * 3u, 0);
    *outW = ow;
    *outH = oh;
    if (!rgb || w <= 0 || h <= 0) return;
    std::vector<std::uint32_t> acc(static_cast<std::size_t>(ow) * 3u);
    std::vector<std::uint32_t> cnt(static_cast<std::size_t>(ow));
    for (int oy = 0; oy < oh; ++oy) {
        std::fill(acc.begin(), acc.end(), 0u);
        std::fill(cnt.begin(), cnt.end(), 0u);
        const int yEnd = std::min(h, (oy + 1) * factor);
        for (int y = oy * factor; y < yEnd; ++y) {
            const unsigned char* src = rgb + static_cast<std::size_t>(y) * w * 3u;
            for (int x = 0; x < w; ++x) {
                const std::size_t ox = static_cast<std::size_t>(x / factor);
                acc[ox * 3 + 0] += src[x * 3 + 0];
                acc[ox * 3 + 1] += src[x * 3 + 1];
                acc[ox * 3 + 2] += src[x * 3 + 2];
                ++cnt[ox];
            }
        }
        unsigned char* dst = out->data() + static_cast<std::size_t>(oy) * ow * 3u;
        for (int ox = 0; ox < ow; ++ox) {
            const std::uint32_t n = std::max<std::uint32_t>(1u, cnt[static_cast<std::size_t>(ox)]);
            for (int k = 0; k < 3; ++k)
                dst[ox * 3 + k] = static_cast<unsigned char>(
                    (acc[static_cast<std::size_t>(ox) * 3 + k] + n / 2) / n);
        }
    }
}

} // namespace firedog
