// HalconDetector.cpp : circle/fiducial and gray NCC detection. See the header.
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

// ===========================================================================
//  Build WITH the SDK
// ===========================================================================
#else

using namespace HalconCpp;

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
    const FocusRoi got = RgbToGray8(rgb, w, h, want, gray);
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

    const bool found = (impl_->model.method == "ncc")
        ? impl_->DetectNcc(gray.data(), used.w, used.h, out)
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

#endif   // HAS_HALCON

} // namespace firedog
