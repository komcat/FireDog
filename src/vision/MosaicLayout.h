// MosaicLayout.h : the arithmetic of a mosaic — which tiles an area needs,
// where the gantry goes for each one, where each tile lands on the stitched
// canvas, and how a region of that canvas is composed back out of the tiles.
// Plain C++, no HALCON, no SDK: stitching must work on a machine with no
// vision licence, for the same reason auto-focus does. Design:
// docs/mosaic-design.md.
//
// ---- The canvas is the CAMERA's pixel frame ------------------------------------
//
// With M the camera's 2x2 mm-per-pixel map (MosaicCameraCal), a pixel (u,v) of
// a frame taken with the gantry at g_i sits at
//     g = g_i + M * (u - W/2, v - H/2)
// and the canvas is defined by  g = g0 + M * c.  Every tile shares the camera's
// orientation, so in canvas pixels a tile is just its frame, translated so
// that its centre sits at  M^-1 * (g_i - g0).
// No tile is ever resampled - not to draw it, not to export it. A camera that
// is rotated against the gantry changes M, which moves where the tiles land and
// how a canvas point converts back to gantry mm, and nothing else.
//
// Placement uses the MEASURED pose of each tile (what the axes reported after
// settling), never the commanded one: the difference is exactly the error a
// commanded placement would stitch into the seams.
#pragma once

#include "core/MosaicConfig.h"

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

// The largest scan the planner accepts. A 2" x 2" area at 15 % overlap is
// ~120 tiles; four thousand is a whole wafer, and anything past it is a typo in
// a corner or the overlap, to be refused before the machine moves.
inline constexpr std::size_t kMaxMosaicTiles = 4000;

// Canvas pixels <-> gantry mm.
struct MosaicGeom {
    double m[4]   = {1.0, 0.0, 0.0, 1.0};   // mm per canvas px (row-major 2x2)
    double inv[4] = {1.0, 0.0, 0.0, 1.0};   // canvas px per mm
    double g0x = 0.0, g0y = 0.0;             // gantry XY of canvas pixel (0,0)
};

// False when `mmPerPx` is singular (a zero scale, or two parallel columns).
bool MosaicGeomMake(const double mmPerPx[4], double g0x, double g0y, MosaicGeom* out);
void MosaicCanvasToGantry(const MosaicGeom& g, double cx, double cy, double* gx, double* gy);
void MosaicGantryToCanvas(const MosaicGeom& g, double gx, double gy, double* cx, double* cy);

// diag(pixel_to_mm_x, pixel_to_mm_y): the uncalibrated M.
void MosaicDefaultM(double px2mmX, double px2mmY, double out[4]);
// The M a capture uses: the calibration when there is one, else the default.
void MosaicEffectiveM(const MosaicCameraCal& cal, double px2mmX, double px2mmY, double out[4]);

// ---- The area --------------------------------------------------------------------

// "" when the area can be planned; else why not — corners untaught, a
// degenerate or non-convex outline, an overlap out of range, a Z plane through
// collinear corners. One implementation: the page prints it and mosaic_capture
// fails on it.
std::string MosaicAreaValidate(const MosaicArea& a);

// The four outline corners in gantry XY, in order round the edge. For a
// 3-corner area the fourth is c0 + c2 - c1. False when a needed corner is
// untaught.
bool MosaicAreaQuad(const MosaicArea& a, double q[4][2]);

// z = pa + pb * x + pc * y through the taught corners (exact through three,
// least squares through four). False + *why when they are collinear.
bool MosaicPlaneFit(const MosaicArea& a, double* pa, double* pb, double* pc, std::string* why);

// The Z a tile at gantry (x, y) is shot at, by the area's Z mode. A plane is
// clamped to the range of Zs the corners were taught at, never extrapolated
// past it.
double MosaicTileZ(const MosaicArea& a, double x, double y);

// ---- The plan --------------------------------------------------------------------

struct MosaicPlanTile {
    int    row = 0, col = 0;        // 0-based grid cell
    double cx = 0.0, cy = 0.0;      // canvas px of the frame's centre (nominal)
    double gx = 0.0, gy = 0.0, gz = 0.0;   // where the gantry goes
};

struct MosaicPlan {
    bool        ok = false;
    std::string why;                // the refusal, when !ok
    MosaicGeom  geom;
    int         imgW = 0, imgH = 0; // camera frame size, px
    int         rows = 0, cols = 0; // the bounding grid (not every cell is shot)
    double      stepX = 0.0, stepY = 0.0;   // grid pitch, canvas px
    int         canvasW = 0, canvasH = 0;
    double      quad[4][2] = {};    // the area outline, canvas px
    std::vector<MosaicPlanTile> tiles;   // capture order: serpentine
};

// Lay the area out: a grid of frames at (1 - overlap) pitch covering the
// outline's bounding box, centred on it, keeping only the frames that touch the
// outline, walked serpentine (every second row reversed - no long return leg).
MosaicPlan PlanMosaic(const MosaicArea& a, const double mmPerPx[4], int imgW, int imgH);

// ---- Composing -------------------------------------------------------------------

// One captured tile on the canvas: its frame's top-left in canvas px, and size.
struct MosaicTileRef {
    double x = 0.0, y = 0.0;
    int    w = 0, h = 0;
};

// Fill `out` (outW * outH * 3, RGB) with the canvas rectangle whose top-left is
// (x0, y0), downsampled by the integer `scale` (a box average of scale x scale
// canvas pixels per output pixel). Where tiles overlap, the tile whose CENTRE is
// nearest owns the pixel - a Voronoi seam, sharp on both sides, where a blend
// would ghost anything that does not register to the pixel. Uncovered pixels
// are black. `getTile(i)` returns tile i's RGB (w * h * 3) or null (missing:
// treated as absent); the caller owns caching, because only it knows what
// decoding a tile costs.
void ComposeMosaicRegion(const std::vector<MosaicTileRef>& tiles, int x0, int y0, int outW,
                         int outH, int scale,
                         const std::function<const unsigned char*(std::size_t)>& getTile,
                         std::vector<unsigned char>* out);

// ---- Seam refinement ---------------------------------------------------------------
//
// The stage places every tile to within its repeatability and the camera map's
// accuracy - a few pixels, visible as a step along a seam. Refinement measures
// each neighbour pair's actual offset in its overlap (ImageShift NCC, on the
// quarter previews, or the full frames when they are small) and solves for the tile positions that agree best with all
// of them at once, held loosely to the stage positions so a tile with no
// textured overlap stays where the stage put it. Pairs that match poorly, or
// land on the edge of the search, are left out rather than trusted.

struct MosaicRefineInput {
    std::vector<MosaicTileRef>   tiles;    // stage placement, full-res canvas px
    std::vector<std::pair<int, int>> rowCol;   // grid cell of each tile
    int    scale = 4;          // the grey planes are 1/scale of the full frame
    int    searchPx = 24;      // how far a seam may be off, full-res px
    double minScore = 0.6;     // a weaker match is not evidence
};

struct MosaicRefineResult {
    bool        ok = false;
    std::string why;
    std::vector<double> x, y;  // refined top-left per tile, full-res canvas px
    int    pairs = 0, used = 0;
    double rmsPx = 0.0;        // residual of the used pairs after the solve
    double maxMovePx = 0.0;    // the largest correction applied to a tile
};

// `gray(i, &plane, &w, &h)` hands over tile i's grey plane at 1/scale; false
// = unavailable (the tile keeps its stage position). `stop()` is polled
// between pairs.
MosaicRefineResult RefineMosaicPlacement(
    const MosaicRefineInput& in,
    const std::function<bool(std::size_t, std::vector<unsigned char>*, int*, int*)>& gray,
    const std::function<bool()>& stop);

// ---- Pair measurement, and the camera map fitted to it ------------------------------------

// One neighbour pair's offset as the pictures show it: where tile j sits
// relative to tile i on the canvas (P_j - P_i), full-res px.
struct MosaicPairMeasure {
    std::size_t i = 0, j = 0;
    double      ox = 0.0, oy = 0.0;
    double      score = 0.0;   // NCC at the match
};

// Every right / below neighbour pair of `in`, matched in its overlap within
// +-in.searchPx of where the tiles are placed now. Only the pairs that matched
// (score >= minScore, peak not on the edge of the search) come back;
// *candidates is how many were tried, *stopped says `stop()` cut it short.
// The measurement both refinement and the camera-map fit are built on.
std::vector<MosaicPairMeasure> MeasureMosaicPairs(
    const MosaicRefineInput& in,
    const std::function<bool(std::size_t, std::vector<unsigned char>*, int*, int*)>& gray,
    const std::function<bool()>& stop, int* candidates, bool* stopped);

// The camera's mm-per-pixel map fitted to a mosaic: each matched pair is a
// gantry move D (the tiles' MEASURED poses, `gantry` index-aligned with the
// tiles) and the picture offset p it produced, p = M^-1 D. Least squares over
// all of them, the pairs the fit disagrees with dropped and the fit repeated.
// No extra motion: a capture already holds dozens of these, and they span the
// whole area where a four-step calibration samples one spot.
struct MosaicMapFit {
    bool        ok = false;
    std::string why;
    double      m[4] = {0.0, 0.0, 0.0, 0.0};   // mm per px, row-major
    int         pairs = 0, used = 0;           // tried, and agreeing with the fit
    double      rmsPx = 0.0;                   // residual of the used pairs
    double      rotDeg = 0.0, skewDeg = 0.0, scaleX = 0.0, scaleY = 0.0;
};
MosaicMapFit FitMosaicCameraMap(const std::vector<MosaicPairMeasure>& pairs,
                                const std::vector<std::pair<double, double>>& gantry,
                                int candidates);

// What a map says in words: the rotation of image x against gantry X, how far
// image y is from square to image x, and the mm-per-px along each image axis.
void MosaicMapDescribe(const double m[4], double* rotDeg, double* skewDeg, double* sx, double* sy);

// Box-downsample an RGB image by an integer factor (edge blocks are partial).
void DownsampleRgb(const unsigned char* rgb, int w, int h, int factor,
                   std::vector<unsigned char>* out, int* outW, int* outH);

} // namespace firedog
