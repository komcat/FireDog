// HalconDetector.cpp : circle/fiducial, gray NCC and square-corner detection.
// See the header.
//
// With vision/HalconEngine.cpp, one of only two translation units that include
// a HALCON header. Without HAS_HALCON it compiles the "built without the SDK"
// answer instead, so the build stays green and both exes still link.
#include "vision/HalconDetector.h"

#include "core/Log.h"
#include "vision/FocusMetric.h"     // RgbToGray8 — the one RGB->luma conversion
#include "vision/HalconEngine.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <vector>

#ifdef HAS_HALCON
  #include <halconcpp/HalconCpp.h>
#endif

namespace firedog {

namespace {

// Every failure path funnels through here so a miss and a licence problem are
// reported the same shape — a reliability run counts both as "not found".
bool Miss(VisionDetection* out, const std::string& why)
{
    if (out) { out->ok = false; out->error = why; }
    return false;
}

#ifdef HAS_HALCON

constexpr double kPi = 3.14159265358979323846;

// The search window as a FocusRoi, so RgbToGray8 does the crop and the clamp in
// the same pass it does the conversion. A disabled ROI is the whole frame.
FocusRoi SearchRoi(const VisionRoi& r)
{
    if (!r.enabled) return FocusRoi{};
    FocusRoi f;
    f.x = r.x; f.y = r.y; f.w = r.w; f.h = r.h;
    return f;
}

double NowMs()
{
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

// NCC settings for a synthesised corner template. `use_polarity` deliberately:
// the operator said which way round the contrast runs, and ignoring it would
// let a dark-on-light corner match a light-on-dark one 90 degrees away.
NccParams CornerNccParams(const CornerParams& c, int n)
{
    NccParams np;
    np.minScore          = c.minScore;
    np.maxMatches        = 1;
    np.numLevels         = 0;          // let HALCON pick the pyramid depth
    np.angleToleranceDeg = c.angleToleranceDeg;
    np.metric            = "use_polarity";
    np.useAnchor         = false;      // the vertex IS the template centre
    np.tmplW = np.tmplH  = n;
    return np;
}

#endif   // HAS_HALCON

} // namespace

// ===========================================================================
//  Build WITHOUT the SDK — structure only, so a model can still be authored
//  and a station still fails with a sentence instead of a link error.
// ===========================================================================
#ifndef HAS_HALCON

struct HalconDetector::Impl {};

HalconDetector::HalconDetector() : impl_(std::make_unique<Impl>()) {}
HalconDetector::~HalconDetector() = default;

bool HalconDetector::Available(std::string* why)
{
    if (why) *why = HalconEngine::Instance().Status().error;
    return false;
}

bool HalconDetector::Load(const VisionModel&, const std::string&, std::string* err)
{
    Available(err);
    return false;
}

bool HalconDetector::Teach(const VisionModel&, const unsigned char*, int, int,
                           const VisionRoi&, const std::string&,
                           int*, int*, std::string* err)
{
    Available(err);
    return false;
}

bool HalconDetector::Detect(const unsigned char*, int, int, VisionDetection* out)
{
    std::string why;
    Available(&why);
    return Miss(out, why);
}

// The chain is HALCON operators, so without the SDK there is no preview to
// give. The filter PARAMETERS are still editable and still persist (this whole
// file's absence changes nothing in VisionModel.cpp) — only the picture and the
// detection are gone, each saying so with the engine's own reason.
bool HalconDetector::Preprocess(const VisionModel&, const unsigned char*, int, int,
                                std::vector<unsigned char>* gray, FocusRoi* used,
                                std::string* err)
{
    if (gray) gray->clear();
    if (used) *used = FocusRoi{};
    Available(err);
    return false;
}

// ===========================================================================
//  Build WITH the SDK
// ===========================================================================
#else

using namespace HalconCpp;

namespace {

// ---- the pre-process chain -------------------------------------------------
// Three slots, applied in order to the ROI plane BEFORE any method looks at it.
// Ported operator-for-operator from Horizon's VisionFacet.ApplyOneFilter, so a
// chain tuned on the sibling machine behaves the same here.
//
// It is a plain fold: stage N's output IS stage N+1's input. A "none" slot is
// skipped rather than collapsing the list, so slots 1 and 3 with 2 empty is the
// two-stage chain [1,3] and the numbers on screen keep meaning what they say.

bool AnyFilter(const VisionModel& m)
{
    for (const VisionFilter& f : m.filters)
        if (f.kind != "none" && !f.kind.empty()) return true;
    return false;
}

HImage ApplyOneFilter(const HImage& g, const VisionFilter& f, int w, int h)
{
    if (f.kind == "gaussian")
        return g.GaussImage(static_cast<Hlong>(f.gaussMask >= 5 ? 5 : 3));

    if (f.kind == "binary") {
        // Foreground 255 / background 0, so what the next stage (and the
        // method's own threshold) sees is a clean two-level image.
        const HRegion region = g.Threshold(static_cast<double>(f.thresholdMin),
                                           static_cast<double>(f.thresholdMax));
        return region.RegionToBin(255, 0, static_cast<Hlong>(w), static_cast<Hlong>(h));
    }

    if (f.kind == "canny") {
        // The direction image is discarded: the chain carries one plane, and an
        // edge ANGLE is not something a downstream threshold or match can use.
        const double alpha = (f.alpha > 0.0) ? f.alpha : 1.0;
        HImage dir;
        const HImage amp = g.EdgesImage(&dir, HString("canny"), alpha, HString("nms"),
                                        static_cast<Hlong>(f.thresholdMin),
                                        static_cast<Hlong>(f.thresholdMax));
        return amp.ConvertImageType(HString("byte"));
    }

    return g;   // "none", or a kind written by a newer build than this one
}

// Fold the chain over `gray` IN PLACE. False + `err` on a HALCON failure; the
// buffer is left untouched, so a caller that ignores the failure detects on raw
// pixels rather than on half a chain.
//
// The empty chain returns before a HImage is even built: the common case must
// cost nothing, or an ROI-sized copy would be charged to every frame of every
// unfiltered model in the machine.
bool ApplyFilterChain(const VisionModel& m, std::vector<unsigned char>& gray,
                      int w, int h, std::string* err)
{
    if (!AnyFilter(m) || w <= 0 || h <= 0) return true;
    if (gray.size() < static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
        if (err) *err = "pre-process: the grey plane is smaller than its own size";
        return false;
    }
    try {
        // gen_image1 COPIES the pixels (gen_image1_extern is the one that
        // borrows), so `gray` may be overwritten from the result below.
        HImage cur;
        cur.GenImage1("byte", static_cast<Hlong>(w), static_cast<Hlong>(h),
                      static_cast<void*>(gray.data()));
        for (const VisionFilter& f : m.filters) {
            if (f.kind == "none" || f.kind.empty()) continue;
            cur = ApplyOneFilter(cur, f, w, h);
        }

        HString type;
        Hlong   ow = 0, oh = 0;
        const void* p = cur.GetImagePointer1(&type, &ow, &oh);
        if (!p || ow != static_cast<Hlong>(w) || oh != static_cast<Hlong>(h)) {
            if (err) *err = "pre-process: a filter changed the image size";
            return false;
        }
        const unsigned char* src = static_cast<const unsigned char*>(p);
        gray.assign(src, src + static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        return true;
    } catch (const HException& e) {
        if (err) *err = std::string("pre-process failed: ") + e.ErrorMessage().Text();
        return false;
    }
}

// Cut `box` (full-frame px) out of an ALREADY-converted plane whose own origin
// and size are `plane` (also full-frame px). Returns the rect actually taken,
// clamped to the plane — the same contract RgbToGray8 has, so a caller cannot
// tell which route produced its patch.
FocusRoi CropPlane(const std::vector<unsigned char>& src, const FocusRoi& plane,
                   const FocusRoi& box, std::vector<unsigned char>& out)
{
    out.clear();
    int x0 = box.x, y0 = box.y;
    int x1 = box.x + box.w, y1 = box.y + box.h;
    if (x0 < plane.x) x0 = plane.x;
    if (y0 < plane.y) y0 = plane.y;
    if (x1 > plane.x + plane.w) x1 = plane.x + plane.w;
    if (y1 > plane.y + plane.h) y1 = plane.y + plane.h;
    if (x1 <= x0 || y1 <= y0) return FocusRoi{};

    FocusRoi got;
    got.x = x0; got.y = y0; got.w = x1 - x0; got.h = y1 - y0;
    out.resize(static_cast<std::size_t>(got.w) * static_cast<std::size_t>(got.h));
    for (int y = 0; y < got.h; ++y) {
        const unsigned char* s =
            src.data() + static_cast<std::size_t>(y + got.y - plane.y) *
                             static_cast<std::size_t>(plane.w) +
            static_cast<std::size_t>(got.x - plane.x);
        std::memcpy(out.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(got.w),
                    s, static_cast<std::size_t>(got.w));
    }
    return got;
}

} // namespace

struct HalconDetector::Impl {
    VisionModel model;
    bool        loaded = false;

    HNCCModel ncc;
    bool      nccReady = false;

    // Build the NCC model from an already-grey patch. Two HALCON quirks, both
    // load-bearing:
    //   * AngleStep must go through the HTuple("auto") overload — the double
    //     overload rejects the sentinel.
    //   * NumLevels 0 means "choose the pyramid depth", which is what we want
    //     by default; a non-zero value is the operator overriding it.
    bool BuildNcc(const NccParams& p, const unsigned char* gray, int w, int h,
                  std::string* err)
    {
        nccReady = false;
        if (!gray || w < 4 || h < 4) {
            if (err) *err = "template patch is too small (need at least 4x4 px)";
            return false;
        }
        try {
            HImage tmpl;
            tmpl.GenImage1("byte", static_cast<Hlong>(w), static_cast<Hlong>(h),
                           static_cast<void*>(const_cast<unsigned char*>(gray)));
            const double aStart  = -p.angleToleranceDeg * kPi / 180.0;
            const double aExtent = 2.0 * p.angleToleranceDeg * kPi / 180.0;
            ncc.CreateNccModel(tmpl,
                               HTuple(static_cast<Hlong>(p.numLevels)),
                               aStart, aExtent, HTuple("auto"),
                               HString(p.metric.c_str()));
            nccReady = true;
            return true;
        } catch (const HException& e) {
            if (err) *err = std::string("create_ncc_model failed: ") + e.ErrorMessage().Text();
            return false;
        }
    }

    // ---- the two detectors -------------------------------------------------
    // Both receive an already-cropped grey plane and report in ROI-LOCAL
    // pixels; Detect() adds the ROI origin back exactly once, at the end.

    bool DetectCircle(const unsigned char* gray, int w, int h, VisionDetection* out)
    {
        const CircleParams& c = model.circle;
        try {
            HImage img;
            img.GenImage1("byte", static_cast<Hlong>(w), static_cast<Hlong>(h),
                          static_cast<void*>(const_cast<unsigned char*>(gray)));

            // Polarity 0 = a dark feature, taken as the [min..max] grey window.
            // Polarity 1 = a bright feature, taken as [max..255] — thresholdMax
            // doubles as the single lower bound there. Ported verbatim from
            // bobcat's robust plugin; changing it would silently re-tune every
            // profile written against it.
            HRegion region = (c.polarity == 0)
                ? img.Threshold(static_cast<double>(c.thresholdMin),
                                static_cast<double>(c.thresholdMax))
                : img.Threshold(static_cast<double>(c.thresholdMax), 255.0);

            HRegion closed    = region.ClosingCircle(c.closingRadius);
            HRegion connected = closed.Connection();
            HRegion byArea    = connected.SelectShape("area", "and",
                                                      static_cast<double>(c.minArea),
                                                      static_cast<double>(c.maxArea));
            HRegion byCirc    = byArea.SelectShape("circularity", "and",
                                                  c.minCircularity, 1.0);

            const Hlong n = byCirc.CountObj();
            out->count = static_cast<int>(n);
            if (n <= 0)
                return Miss(out, "no region passed the area/circularity filter");

            // More than one candidate: take the largest. A fiducial is the
            // biggest round thing in its ROI; if it is not, the ROI is wrong.
            HRegion best = (n > 1) ? byCirc.SelectShapeStd("max_area", 70) : byCirc;

            HXLDCont contour = best.GenContourRegionXld("border");
            if (contour.LengthXld().D() < 10.0)
                return Miss(out, "contour too short to fit a circle");

            // The HString overload is the one with HTuple outputs; the char*
            // overload returns bare doubles and would not compile here.
            HTuple row, col, radius, startPhi, endPhi, pointOrder;
            contour.FitCircleContourXld(
                HString(c.fitMethod == 0 ? "atukey" : "geotukey"),
                static_cast<Hlong>(-1),   // MaxNumPoints: use them all
                2.0,                      // MaxClosureDist
                static_cast<Hlong>(0),    // ClippingEndPoints
                static_cast<Hlong>(10),   // Iterations
                1.0,                      // ClippingFactor
                &row, &col, &radius, &startPhi, &endPhi, &pointOrder);
            if (radius.Length() <= 0 || radius.D() <= 0.0 || std::isnan(radius.D()))
                return Miss(out, "circle fit did not converge");

            out->xPx      = col.D();          // HALCON is (row, col) = (y, x)
            out->yPx      = row.D();
            out->radiusPx = radius.D();
            out->widthPx  = out->heightPx = 2.0 * radius.D();
            out->angleDeg = 0.0;              // a circle has no orientation
            // Circularity as the quality figure: it is already 0..1 and higher
            // is better, so it means the same thing as an NCC score and a
            // reliability run can compare the two.
            out->score = best.Circularity().D();
            out->ok    = true;
            return true;
        } catch (const HException& e) {
            return Miss(out, std::string("circle detection failed: ")
                             + e.ErrorMessage().Text());
        }
    }

    // A contrasting square, reported at ONE of its four corners.
    //
    // threshold -> close -> connect -> area window from the size slider ->
    // rectangularity gate -> largest survivor -> sub-pixel smallest_rectangle2
    // over its XLD border. The rectangle fit is what buys the sub-pixel corner:
    // the region itself is whole pixels, so taking a corner off it would quantise
    // every measurement to the grid the reliability run is trying to measure.
    //
    // The reported ANGLE is the direction from the corner towards the square's
    // centre (screen convention, y down), and `widthPx`/`heightPx` are the two
    // edges leaving that corner, ordered so that width lies at angle-45deg and
    // height at angle+45deg. That triple reconstructs the whole square from the
    // reported point alone, which is what lets the teach overlay draw exactly
    // what was found instead of a box centred on the wrong place.
    // Mode 1: the synthesised two-edge template. The match itself is ordinary
    // NCC — the whole difference is that the patch came from MakeCornerPatch
    // instead of off the frame, so DetectNcc does the work and this only
    // re-expresses the answer in the corner convention.
    bool DetectCornerTemplate(const unsigned char* gray, int w, int h, VisionDetection* out)
    {
        if (!nccReady)
            return Miss(out, "the corner template was not built - check the size");
        if (!DetectNcc(gray, w, h, out)) return false;

        // Direction from the vertex INTO the material, screen convention (x
        // right, y down, positive clockwise). The material fills the quadrant
        // OPPOSITE the reported corner, so a bottom-left corner points up-right.
        // The match angle turns it — HALCON's angle is counter-clockwise, this
        // one clockwise, hence the subtraction.
        static const double kBisector[4] = {-135.0, -45.0, 45.0, 135.0};   // TL TR BR BL
        const CornerParams& c = model.corner;
        const int q = (((c.corner >= 0 && c.corner < 4) ? c.corner : 0) + 2) % 4;
        const double half = 0.5 * static_cast<double>(model.ncc.tmplW);

        out->angleDeg = kBisector[q] - out->angleDeg;
        // The two edges leaving the vertex — half the patch each, which is
        // exactly the filled quadrant the overlay then draws.
        out->widthPx = out->heightPx = half;
        out->lengthPx = half;
        return true;
    }

    bool DetectCorner(const unsigned char* gray, int w, int h, VisionDetection* out)
    {
        const CornerParams& c = model.corner;
        if (c.mode == 1) return DetectCornerTemplate(gray, w, h, out);
        try {
            HImage img;
            img.GenImage1("byte", static_cast<Hlong>(w), static_cast<Hlong>(h),
                          static_cast<void*>(const_cast<unsigned char*>(gray)));

            // Auto is the default because a corner mark is defined by CONTRAST,
            // not by an absolute grey: max_separability re-splits per frame, so
            // the model survives the lamp ageing and the part changing finish.
            HRegion region;
            if (c.autoThreshold) {
                HTuple usedThr;
                region = img.BinaryThreshold(HString("max_separability"),
                                             HString(c.polarity == 0 ? "dark" : "light"),
                                             &usedThr);
            } else {
                const double t = static_cast<double>(c.threshold);
                region = (c.polarity == 0) ? img.Threshold(0.0, t)
                                           : img.Threshold(t, 255.0);
            }

            HRegion closed = (c.closingRadius > 0.0) ? region.ClosingCircle(c.closingRadius)
                                                     : region;
            HRegion connected = closed.Connection();

            // The size slider IS the search: an area window either side of
            // side^2 is what rejects the rest of the scene.
            double sMin = c.sizePx * (1.0 - c.sizeTolPct / 100.0);
            double sMax = c.sizePx * (1.0 + c.sizeTolPct / 100.0);
            if (sMin < 2.0)    sMin = 2.0;
            if (sMax <= sMin)  sMax = sMin + 1.0;

            HRegion byArea = connected.SelectShape("area", "and", sMin * sMin, sMax * sMax);
            // Rectangularity is 1.0 for a perfect rectangle at ANY rotation, so
            // this gate does not assume the part is square to the camera.
            HRegion byRect = byArea.SelectShape("rectangularity", "and",
                                                c.minRectangularity, 1.0);

            const Hlong n = byRect.CountObj();
            out->count = static_cast<int>(n);
            if (n <= 0)
                return Miss(out, "no square of that size passed the area/rectangularity filter");

            // More than one survivor: take the largest, as the circle path does.
            // The area window is already tight, so anything still standing is
            // the mark or a duplicate of it.
            HRegion best = (n > 1) ? byRect.SelectShapeStd("max_area", 70) : byRect;

            HXLDCont contour = best.GenContourRegionXld("border");
            HTuple row, col, phi, len1, len2;
            contour.SmallestRectangle2Xld(&row, &col, &phi, &len1, &len2);
            if (len1.Length() <= 0 || len1.D() <= 0.0 || len2.D() <= 0.0)
                return Miss(out, "could not fit a rectangle to the region");

            // HALCON: phi is counter-clockwise about a downward row axis, and
            // Length1/Length2 are HALF extents. Main axis in (x = col, y = row)
            // is therefore (cos phi, -sin phi), and its perpendicular (sin phi,
            // cos phi).
            const double cx = col.D(), cy = row.D(), p = phi.D();
            const double ux =  std::cos(p) * len1.D(), uy = -std::sin(p) * len1.D();
            const double vx =  std::sin(p) * len2.D(), vy =  std::cos(p) * len2.D();

            const double px[4] = {cx - ux - vx, cx + ux - vx, cx + ux + vx, cx - ux + vx};
            const double py[4] = {cy - uy - vy, cy + uy - vy, cy + uy + vy, cy - uy + vy};

            // Name the four in IMAGE terms, so "top-left" keeps meaning the same
            // thing when the part is rotated: x+y is smallest at the top-left and
            // largest at the bottom-right, x-y largest at the top-right.
            int iTL = 0, iTR = 0, iBR = 0, iBL = 0;
            for (int i = 1; i < 4; ++i) {
                if (px[i] + py[i] < px[iTL] + py[iTL]) iTL = i;
                if (px[i] + py[i] > px[iBR] + py[iBR]) iBR = i;
                if (px[i] - py[i] > px[iTR] - py[iTR]) iTR = i;
                if (px[i] - py[i] < px[iBL] - py[iBL]) iBL = i;
            }
            const int order[4] = {iTL, iTR, iBR, iBL};
            const int k = order[(c.corner >= 0 && c.corner < 4) ? c.corner : 0];

            const double qx = px[k], qy = py[k];
            // The two edges leaving this corner: the neighbours are the corners
            // that differ from it in exactly one of the two axis signs, which is
            // +-2*u and +-2*v about the corner.
            const double su = ((qx - cx) * ux + (qy - cy) * uy) >= 0.0 ? -1.0 : 1.0;
            const double sv = ((qx - cx) * vx + (qy - cy) * vy) >= 0.0 ? -1.0 : 1.0;
            const double e1x = su * 2.0 * ux, e1y = su * 2.0 * uy;   // along the main axis
            const double e2x = sv * 2.0 * vx, e2y = sv * 2.0 * vy;   // along the other

            const double bis = std::atan2((e1y + e2y), (e1x + e2x));   // towards the centre
            const double a1  = std::atan2(e1y, e1x) - bis;
            const double e1L = std::sqrt(e1x * e1x + e1y * e1y);
            const double e2L = std::sqrt(e2x * e2x + e2y * e2y);
            // Order the two edges by which side of the bisector they fall on, so
            // width is always the -45deg one. sin() rather than the raw
            // difference: the subtraction can land either side of +-pi.
            const bool e1First = std::sin(a1) < 0.0;

            out->xPx      = qx;
            out->yPx      = qy;
            out->angleDeg = bis * 180.0 / kPi;
            out->widthPx  = e1First ? e1L : e2L;
            out->heightPx = e1First ? e2L : e1L;
            out->lengthPx = 0.5 * (e1L + e2L);      // the mean side, for the readout
            // Rectangularity as the 0..1 quality figure, the same shape of
            // number the circle method reports as circularity.
            out->score = best.Rectangularity().D();
            out->ok    = true;
            return true;
        } catch (const HException& e) {
            return Miss(out, std::string("corner detection failed: ")
                             + e.ErrorMessage().Text());
        }
    }

    bool DetectNcc(const unsigned char* gray, int w, int h, VisionDetection* out)
    {
        if (!nccReady) return Miss(out, "no template has been taught for this model");
        const NccParams& p = model.ncc;
        try {
            HImage img;
            img.GenImage1("byte", static_cast<Hlong>(w), static_cast<Hlong>(h),
                          static_cast<void*>(const_cast<unsigned char*>(gray)));

            const double aStart  = -p.angleToleranceDeg * kPi / 180.0;
            const double aExtent = 2.0 * p.angleToleranceDeg * kPi / 180.0;

            HTuple row, col, angle, score;
            ncc.FindNccModel(img, aStart, aExtent, p.minScore,
                             static_cast<Hlong>(p.maxMatches),
                             0.5,        // MaxOverlap
                             "true",     // SubPixel refinement
                             static_cast<Hlong>(p.numLevels),
                             &row, &col, &angle, &score);

            out->count = static_cast<int>(score.Length());
            if (out->count <= 0)
                return Miss(out, "no match above the minimum score");

            // find_ncc_model returns matches best-first, so index 0 is the one.
            const double a  = angle[0].D();          // radians
            double       mx = col[0].D();
            double       my = row[0].D();

            // The anchor rides WITH the feature: rotate the template-relative
            // offset by the match angle before applying it, or a rotated part
            // puts the reported point somewhere the feature never was.
            if (p.useAnchor) {
                const double ca = std::cos(a), sa = std::sin(a);
                mx += p.anchorOffsetX * ca - p.anchorOffsetY * sa;
                my += p.anchorOffsetX * sa + p.anchorOffsetY * ca;
            }

            out->xPx      = mx;
            out->yPx      = my;
            out->angleDeg = a * 180.0 / kPi;
            out->score    = score[0].D();
            out->widthPx  = static_cast<double>(p.tmplW);
            out->heightPx = static_cast<double>(p.tmplH);
            out->ok       = true;
            return true;
        } catch (const HException& e) {
            return Miss(out, std::string("pattern match failed: ")
                             + e.ErrorMessage().Text());
        }
    }
};

HalconDetector::HalconDetector() : impl_(std::make_unique<Impl>()) {}
HalconDetector::~HalconDetector() = default;

bool HalconDetector::Available(std::string* why)
{
    const HalconStatus& s = HalconEngine::Instance().Status();
    if (why) *why = s.error;
    return s.Available();
}

bool HalconDetector::Load(const VisionModel& m, const std::string& modelDir,
                          std::string* err)
{
    impl_->loaded   = false;
    impl_->nccReady = false;
    if (!Available(err)) return false;

    impl_->model = m;

    if (m.method == "ncc") {
        if (m.templateFile.empty()) {
            if (err) *err = "model '" + m.name + "' has no taught template";
            return false;
        }
        const std::string path = modelDir + "/" + m.templateFile;
        std::vector<unsigned char> gray;
        int tw = 0, th = 0;
        if (!LoadGrayPgm(path, &gray, &tw, &th)) {
            if (err) *err = "cannot read template " + path;
            return false;
        }
        if (!impl_->BuildNcc(m.ncc, gray.data(), tw, th, err)) return false;
        // The stored size is what the overlay draws; a template edited outside
        // FireDog would otherwise draw a box the wrong size around a good match.
        impl_->model.ncc.tmplW = tw;
        impl_->model.ncc.tmplH = th;
    } else if (m.method == "corner") {
        if (m.corner.mode == 1) {
            // Synthesise the patch and build an NCC model from it, here at Load
            // rather than per detect: create_ncc_model is the expensive call,
            // and the parameters cannot change without a reload (Workflow
            // stamps the whole document, so an edit already forces one).
            int n = 0;
            const std::vector<unsigned char> patch = MakeCornerTemplate(m.corner, &n);
            const NccParams np = CornerNccParams(m.corner, n);
            // The corner method has no ncc block of its own; borrowing it is
            // what lets DetectNcc run unchanged over the synthetic template.
            impl_->model.ncc = np;
            if (!impl_->BuildNcc(np, patch.data(), n, n, err)) return false;
        }
        // Mode 0 carries no built state — it is all parameters, per detect.
    } else if (m.method != "circle") {
        if (err) *err = "unknown detection method '" + m.method + "'";
        return false;
    }

    impl_->loaded = true;
    return true;
}

bool HalconDetector::Teach(const VisionModel& m, const unsigned char* rgb, int w, int h,
                           const VisionRoi& patch, const std::string& pgmPath,
                           int* tmplW, int* tmplH, std::string* err)
{
    if (tmplW) *tmplW = 0;
    if (tmplH) *tmplH = 0;
    if (!Available(err)) return false;
    if (!rgb || w <= 0 || h <= 0) {
        if (err) *err = "no camera frame to teach from";
        return false;
    }

    FocusRoi want;
    want.x = patch.x; want.y = patch.y; want.w = patch.w; want.h = patch.h;
    std::vector<unsigned char> gray;
    FocusRoi got;

    if (!AnyFilter(m)) {
        // No chain: crop and convert in the one pass, as it always did.
        got = RgbToGray8(rgb, w, h, want, gray);
    } else {
        // A chain: the template MUST be cut from the filtered plane. Matching a
        // raw patch against a filtered live image scores nothing, and the
        // failure looks like a badly taught template rather than a wrong order.
        //
        // Filter the same plane a detect will filter — the search ROI — so the
        // patch carries the same border behaviour the match will see. A box that
        // reaches outside the ROI cannot be cut from it, so that case filters the
        // whole frame instead and says so; silently cropping to the ROI would
        // teach a template of the wrong size.
        std::vector<unsigned char> plane;
        FocusRoi used = RgbToGray8(rgb, w, h, SearchRoi(m.roi), plane);
        const bool inside = want.w > 0 && want.h > 0 &&
                            want.x >= used.x && want.y >= used.y &&
                            want.x + want.w <= used.x + used.w &&
                            want.y + want.h <= used.y + used.h;
        if (!inside) {
            used = RgbToGray8(rgb, w, h, FocusRoi{}, plane);
            FD_WARN("vision", "'%s': the template box reaches outside the search ROI - "
                              "pre-processing the whole frame to cut it",
                    m.name.c_str());
        }
        if (!ApplyFilterChain(m, plane, used.w, used.h, err)) return false;
        got = CropPlane(plane, used, want, gray);
    }

    if (got.w < 4 || got.h < 4) {
        if (err) *err = "the template box is too small (need at least 4x4 px)";
        return false;
    }

    if (!SaveGrayPgm(pgmPath, gray.data(), got.w, got.h)) {
        if (err) *err = "could not write the template to " + pgmPath;
        return false;
    }

    impl_->model    = m;
    impl_->loaded   = false;
    if (!impl_->BuildNcc(m.ncc, gray.data(), got.w, got.h, err)) return false;

    impl_->model.ncc.tmplW = got.w;
    impl_->model.ncc.tmplH = got.h;
    impl_->loaded = true;
    if (tmplW) *tmplW = got.w;
    if (tmplH) *tmplH = got.h;
    FD_INFO("vision", "'%s': taught a %dx%d template -> %s",
            m.name.c_str(), got.w, got.h, pgmPath.c_str());
    return true;
}

bool HalconDetector::Detect(const unsigned char* rgb, int w, int h, VisionDetection* out)
{
    if (!out) return false;
    *out = VisionDetection{};
    out->method = impl_->model.method;

    std::string why;
    if (!Available(&why))      return Miss(out, why);
    if (!impl_->loaded)        return Miss(out, "detection model is not loaded");
    if (!rgb || w <= 0 || h <= 0) return Miss(out, "no camera frame");

    const double t0 = NowMs();

    // One pass: crop to the search window AND convert to 8-bit grey. `used` is
    // the ROI that survived clamping — the origin added back below.
    std::vector<unsigned char> gray;
    const FocusRoi used = RgbToGray8(rgb, w, h, SearchRoi(impl_->model.roi), gray);
    if (used.w <= 0 || used.h <= 0) {
        out->elapsedMs = NowMs() - t0;
        return Miss(out, "the search ROI does not overlap the frame");
    }

    // Pre-processing, between the crop and the method — the ONE place it is
    // applied at run time. Every caller (align_station, place_mark,
    // vision_detect, vision_test, the teach page's Detect) funnels through
    // Workflow::Detect to here, so a chain cannot be honoured on one path and
    // forgotten on another.
    //
    // Note the difference from Horizon, which reduces the HALCON DOMAIN and
    // keeps the surrounding pixels: RgbToGray8 has already CROPPED, so a
    // gaussian or a canny sees the ROI edge as an image edge. That costs a
    // mask/2 rim and buys the coordinate handling below, which is already right.
    std::string ferr;
    if (!ApplyFilterChain(impl_->model, gray, used.w, used.h, &ferr)) {
        out->elapsedMs = NowMs() - t0;
        return Miss(out, ferr);
    }

    const bool found =
        (impl_->model.method == "ncc")      ? impl_->DetectNcc(gray.data(), used.w, used.h, out)
      : (impl_->model.method == "corner")   ? impl_->DetectCorner(gray.data(), used.w, used.h, out)
                                           : impl_->DetectCircle(gray.data(), used.w, used.h, out);

    // Report FULL-FRAME pixels whatever the ROI was. Exactly one place adds the
    // offset, so no consumer has to know whether an ROI was in play.
    if (found) {
        out->xPx += used.x;
        out->yPx += used.y;
    }
    out->elapsedMs = NowMs() - t0;
    return found;
}

// The preview's half of Detect: the same crop, the same chain, and then stop.
// Everything a method would go on to do is left out, so this runs before there
// is a template to build a model from — which is exactly when the chain is
// being tuned.
bool HalconDetector::Preprocess(const VisionModel& m, const unsigned char* rgb,
                                int w, int h, std::vector<unsigned char>* gray,
                                FocusRoi* used, std::string* err)
{
    if (gray) gray->clear();
    if (used) *used = FocusRoi{};
    if (!gray) return false;
    if (!Available(err)) return false;
    if (!rgb || w <= 0 || h <= 0) {
        if (err) *err = "no camera frame";
        return false;
    }

    const FocusRoi got = RgbToGray8(rgb, w, h, SearchRoi(m.roi), *gray);
    if (got.w <= 0 || got.h <= 0) {
        if (err) *err = "the search ROI does not overlap the frame";
        return false;
    }
    if (!ApplyFilterChain(m, *gray, got.w, got.h, err)) return false;
    if (used) *used = got;
    return true;
}

#endif   // HAS_HALCON

} // namespace firedog
