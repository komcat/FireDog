// VisionModel.cpp
#include "vision/VisionModel.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid

#include <cctype>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/vision.local.json";
constexpr const char* kBasePath  = "config/vision.json";

nlohmann::json RoiToJson(const VisionRoi& r)
{
    nlohmann::json j;
    j["enabled"] = r.enabled;
    j["x"] = r.x; j["y"] = r.y; j["w"] = r.w; j["h"] = r.h;
    return j;
}

VisionRoi RoiFromJson(const nlohmann::json& j)
{
    VisionRoi r;
    if (!j.is_object()) return r;
    r.enabled = j.value("enabled", false);
    r.x = j.value("x", 0); r.y = j.value("y", 0);
    r.w = j.value("w", 0); r.h = j.value("h", 0);
    return r;
}

nlohmann::json FilterToJson(const VisionFilter& f)
{
    nlohmann::json j;
    j["kind"]          = f.kind;
    j["threshold_min"] = f.thresholdMin;
    j["threshold_max"] = f.thresholdMax;
    j["gauss_mask"]    = f.gaussMask;
    j["alpha"]         = f.alpha;
    return j;
}

VisionFilter FilterFromJson(const nlohmann::json& j)
{
    VisionFilter f;
    if (!j.is_object()) return f;
    f.kind         = j.value("kind", "none");
    f.thresholdMin = j.value("threshold_min", 0);
    f.thresholdMax = j.value("threshold_max", 128);
    f.gaussMask    = j.value("gauss_mask", 3);
    f.alpha        = j.value("alpha", 1.0);
    return f;
}

nlohmann::json CircleToJson(const CircleParams& c)
{
    nlohmann::json j;
    j["threshold_min"]   = c.thresholdMin;
    j["threshold_max"]   = c.thresholdMax;
    j["closing_radius"]  = c.closingRadius;
    j["min_area"]        = c.minArea;
    j["max_area"]        = c.maxArea;
    j["min_circularity"] = c.minCircularity;
    j["polarity"]        = c.polarity;
    j["fit_method"]      = c.fitMethod;
    return j;
}

CircleParams CircleFromJson(const nlohmann::json& j)
{
    CircleParams c;
    if (!j.is_object()) return c;
    c.thresholdMin   = j.value("threshold_min", 0);
    c.thresholdMax   = j.value("threshold_max", 100);
    c.closingRadius  = j.value("closing_radius", 10.5);
    c.minArea        = j.value("min_area", 500);
    c.maxArea        = j.value("max_area", 999999);
    c.minCircularity = j.value("min_circularity", 0.7);
    c.polarity       = j.value("polarity", 0);
    c.fitMethod      = j.value("fit_method", 0);
    return c;
}

nlohmann::json NccToJson(const NccParams& n)
{
    nlohmann::json j;
    j["min_score"]       = n.minScore;
    j["max_matches"]     = n.maxMatches;
    j["num_levels"]      = n.numLevels;
    j["angle_tol_deg"]   = n.angleToleranceDeg;
    j["metric"]          = n.metric;
    j["use_anchor"]      = n.useAnchor;
    j["anchor_offset_x"] = n.anchorOffsetX;
    j["anchor_offset_y"] = n.anchorOffsetY;
    j["tmpl_w"]          = n.tmplW;
    j["tmpl_h"]          = n.tmplH;
    return j;
}

NccParams NccFromJson(const nlohmann::json& j)
{
    NccParams n;
    if (!j.is_object()) return n;
    n.minScore          = j.value("min_score", 0.7);
    n.maxMatches        = j.value("max_matches", 1);
    n.numLevels         = j.value("num_levels", 0);
    n.angleToleranceDeg = j.value("angle_tol_deg", 0.0);
    n.metric            = j.value("metric", "ignore_global_polarity");
    n.useAnchor         = j.value("use_anchor", false);
    n.anchorOffsetX     = j.value("anchor_offset_x", 0.0);
    n.anchorOffsetY     = j.value("anchor_offset_y", 0.0);
    n.tmplW             = j.value("tmpl_w", 0);
    n.tmplH             = j.value("tmpl_h", 0);
    return n;
}

nlohmann::json CornerToJson(const CornerParams& c)
{
    nlohmann::json j;
    j["mode"]               = c.mode;
    j["polarity"]           = c.polarity;
    j["min_score"]          = c.minScore;
    j["angle_tol_deg"]      = c.angleToleranceDeg;
    j["smooth_px"]          = c.smoothPx;
    j["auto_threshold"]     = c.autoThreshold;
    j["threshold"]          = c.threshold;
    j["size_px"]            = c.sizePx;
    j["size_tol_pct"]       = c.sizeTolPct;
    j["closing_radius"]     = c.closingRadius;
    j["min_rectangularity"] = c.minRectangularity;
    j["corner"]             = c.corner;
    return j;
}

CornerParams CornerFromJson(const nlohmann::json& j)
{
    CornerParams c;
    if (!j.is_object()) return c;
    c.mode              = j.value("mode", 1);
    c.polarity          = j.value("polarity", 0);
    c.minScore          = j.value("min_score", 0.5);
    c.angleToleranceDeg = j.value("angle_tol_deg", 5.0);
    c.smoothPx          = j.value("smooth_px", 1.0);
    c.autoThreshold     = j.value("auto_threshold", true);
    c.threshold         = j.value("threshold", 128);
    c.sizePx            = j.value("size_px", 40.0);
    c.sizeTolPct        = j.value("size_tol_pct", 40.0);
    c.closingRadius     = j.value("closing_radius", 2.0);
    c.minRectangularity = j.value("min_rectangularity", 0.75);
    c.corner            = j.value("corner", 0);
    return c;
}

nlohmann::json RelToJson(const VisionReliability& r)
{
    nlohmann::json j;
    j["runs"]       = r.runs;
    j["ok"]         = r.ok;
    j["mean_score"] = r.meanScore;
    j["mean_x_px"]  = r.meanXPx;
    j["mean_y_px"]  = r.meanYPx;
    j["std_x_px"]   = r.stdXPx;
    j["std_y_px"]   = r.stdYPx;
    j["max_dev_px"] = r.maxDevPx;
    j["tested_at"]  = r.testedAt;
    return j;
}

VisionReliability RelFromJson(const nlohmann::json& j)
{
    VisionReliability r;
    if (!j.is_object()) return r;
    r.runs      = j.value("runs", 0);
    r.ok        = j.value("ok", 0);
    r.meanScore = j.value("mean_score", 0.0);
    r.meanXPx   = j.value("mean_x_px", 0.0);
    r.meanYPx   = j.value("mean_y_px", 0.0);
    r.stdXPx    = j.value("std_x_px", 0.0);
    r.stdYPx    = j.value("std_y_px", 0.0);
    r.maxDevPx  = j.value("max_dev_px", 0.0);
    r.testedAt  = j.value("tested_at", "");
    return r;
}

std::vector<VisionModel> ParseVisionModels(std::ifstream& f, const std::string& path,
                                           std::string* uidOut)
{
    std::vector<VisionModel> out;
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", "");
        if (!j.contains("models") || !j["models"].is_array()) return out;
        for (const auto& row : j["models"]) {
            if (!row.is_object()) continue;
            out.push_back(VisionModelFromJson(row));
        }
        FD_INFO("vision", "loaded %zu detection model(s) from %s", out.size(), path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("vision", "vision models load failed (%s): starting empty", e.what());
        return std::vector<VisionModel>{};
    }
    return out;
}

// PGM headers allow comments and arbitrary whitespace between fields. Read one
// whitespace-delimited token, skipping '#' comment lines — the tolerant path,
// so a patch hand-exported from another tool still loads.
bool NextPgmToken(std::ifstream& f, std::string* tok)
{
    tok->clear();
    int c = f.get();
    for (;;) {
        while (c != EOF && std::isspace(static_cast<unsigned char>(c))) c = f.get();
        if (c == '#') { while (c != EOF && c != '\n') c = f.get(); continue; }
        break;
    }
    if (c == EOF) return false;
    while (c != EOF && !std::isspace(static_cast<unsigned char>(c))) {
        tok->push_back(static_cast<char>(c));
        c = f.get();
    }
    return !tok->empty();
}

} // namespace

// ---- the taught patch ----------------------------------------------------

bool SaveGrayPgm(const std::string& path, const unsigned char* gray, int w, int h)
{
    if (!gray || w <= 0 || h <= 0) return false;
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) {
        FD_WARN("vision", "template save failed: cannot open %s", path.c_str());
        return false;
    }
    f << "P5\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(gray),
            static_cast<std::streamsize>(w) * h);
    return f.good();
}

bool LoadGrayPgm(const std::string& path, std::vector<unsigned char>* gray,
                 int* w, int* h)
{
    if (!gray || !w || !h) return false;
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return false;

    std::string magic, sw, sh, smax;
    if (!NextPgmToken(f, &magic) || magic != "P5")   return false;
    if (!NextPgmToken(f, &sw) || !NextPgmToken(f, &sh) || !NextPgmToken(f, &smax))
        return false;

    int ww = 0, hh = 0, maxv = 0;
    try {
        ww = std::stoi(sw); hh = std::stoi(sh); maxv = std::stoi(smax);
    } catch (const std::exception&) { return false; }
    if (ww <= 0 || hh <= 0 || maxv != 255) return false;

    // NextPgmToken already consumed the single whitespace byte that separates
    // the maxval from the raster, so the stream is sitting on the first pixel.
    gray->assign(static_cast<std::size_t>(ww) * hh, 0);
    f.read(reinterpret_cast<char*>(gray->data()),
           static_cast<std::streamsize>(ww) * hh);
    if (f.gcount() != static_cast<std::streamsize>(ww) * hh) {
        FD_WARN("vision", "template %s is truncated", path.c_str());
        return false;
    }
    *w = ww; *h = hh;
    return true;
}

// ---- the synthesised corner template ---------------------------------------

int CornerTemplateSide(const CornerParams& c)
{
    int n = static_cast<int>(c.sizePx + 0.5);
    if (n < 8)   n = 8;
    if (n > 256) n = 256;
    if (n & 1)   ++n;      // even, so the vertex lands exactly on the patch centre
    return n;
}

std::vector<unsigned char> MakeCornerTemplate(const CornerParams& c, int* nOut)
{
    const int n    = CornerTemplateSide(c);
    const int half = n / 2;

    const unsigned char fg = (c.polarity == 0) ? 40 : 215;   // the material
    const unsigned char bg = (c.polarity == 0) ? 215 : 40;   // what surrounds it
    std::vector<unsigned char> p(static_cast<std::size_t>(n) * n, bg);

    // The material fills the quadrant OPPOSITE the named corner: the bottom-left
    // corner of a shape has the shape up-and-right of it.
    const int q  = (((c.corner >= 0 && c.corner < 4) ? c.corner : 0) + 2) % 4;
    const int x0 = (q == 0 || q == 3) ? 0 : half;
    const int y0 = (q == 0 || q == 1) ? 0 : half;
    for (int y = y0; y < y0 + half; ++y)
        for (int x = x0; x < x0 + half; ++x)
            p[static_cast<std::size_t>(y) * n + x] = fg;

    // Soften the edges. A razor-sharp synthetic step still correlates against a
    // real edge, but a real edge is a one-or-two pixel ramp, and matching a step
    // to a ramp is what makes the sub-pixel position land on a whole pixel more
    // often than it should. Two box passes ~ a small gaussian, and cost nothing
    // at these sizes.
    int r = static_cast<int>(c.smoothPx + 0.5);
    if (r > half / 2) r = half / 2;
    if (r > 0) {
        std::vector<unsigned char> t(p.size());
        const int wnd = 2 * r + 1;
        for (int pass = 0; pass < 2; ++pass) {
            for (int y = 0; y < n; ++y) {
                for (int x = 0; x < n; ++x) {
                    int sum = 0;
                    for (int k = -r; k <= r; ++k) {
                        int xx = x + k;
                        if (xx < 0)  xx = 0;
                        if (xx >= n) xx = n - 1;
                        sum += p[static_cast<std::size_t>(y) * n + xx];
                    }
                    t[static_cast<std::size_t>(y) * n + x] =
                        static_cast<unsigned char>(sum / wnd);
                }
            }
            for (int x = 0; x < n; ++x) {
                for (int y = 0; y < n; ++y) {
                    int sum = 0;
                    for (int k = -r; k <= r; ++k) {
                        int yy = y + k;
                        if (yy < 0)  yy = 0;
                        if (yy >= n) yy = n - 1;
                        sum += t[static_cast<std::size_t>(yy) * n + x];
                    }
                    p[static_cast<std::size_t>(y) * n + x] =
                        static_cast<unsigned char>(sum / wnd);
                }
            }
        }
    }

    if (nOut) *nOut = n;
    return p;
}

// ---- the pre-process chain, as one line ------------------------------------

std::string VisionFilterSummary(const VisionModel& m)
{
    std::string s;
    for (const VisionFilter& f : m.filters) {
        char buf[64];
        if (f.kind == "binary") {
            std::snprintf(buf, sizeof(buf), "binary[%d..%d]", f.thresholdMin, f.thresholdMax);
        } else if (f.kind == "gaussian") {
            std::snprintf(buf, sizeof(buf), "gauss[%d]", f.gaussMask);
        } else if (f.kind == "canny") {
            std::snprintf(buf, sizeof(buf), "canny[%d/%d a%.1f]",
                          f.thresholdMin, f.thresholdMax, f.alpha);
        } else {
            continue;   // "none", or a kind a newer build knows and we do not
        }
        if (!s.empty()) s += " > ";
        s += buf;
    }
    return s;
}

// ---- lookup --------------------------------------------------------------

const VisionModel* FindVisionModel(const std::vector<VisionModel>& list,
                                   const std::string& uidOrName)
{
    if (uidOrName.empty()) return nullptr;
    for (const VisionModel& m : list) if (m.uid == uidOrName) return &m;
    for (const VisionModel& m : list) if (m.name == uidOrName) return &m;
    return nullptr;
}

VisionModel* FindVisionModel(std::vector<VisionModel>& list, const std::string& uidOrName)
{
    const std::vector<VisionModel>& c = list;
    return const_cast<VisionModel*>(FindVisionModel(c, uidOrName));
}

// ---- persistence ---------------------------------------------------------

nlohmann::json VisionModelToJson(const VisionModel& m)
{
    nlohmann::json j;
    j["uid"]           = m.uid;
    j["name"]          = m.name;
    j["method"]        = m.method;
    j["roi"]           = RoiToJson(m.roi);
    j["exposure_us"]   = m.exposureUs;
    j["lighting"]      = AlignLightingToJson(m.lighting);
    // Always all three slots, in order — the position IS the stage number, so a
    // chain written with slot 2 empty must not come back as a two-element list.
    nlohmann::json filters = nlohmann::json::array();
    for (const VisionFilter& f : m.filters) filters.push_back(FilterToJson(f));
    j["filters"]       = std::move(filters);
    j["circle"]        = CircleToJson(m.circle);
    j["ncc"]           = NccToJson(m.ncc);
    j["corner"]        = CornerToJson(m.corner);
    j["template_file"] = m.templateFile;
    j["reliability"]   = RelToJson(m.rel);
    return j;
}

VisionModel VisionModelFromJson(const nlohmann::json& j)
{
    VisionModel m;
    if (!j.is_object()) return m;
    m.uid = j.value("uid", "");
    if (m.uid.empty()) m.uid = GenerateUid();   // legacy row — assign once
    m.name         = j.value("name", "detect");
    m.method       = j.value("method", "circle");
    m.exposureUs   = j.value("exposure_us", 0.0);
    if (j.contains("lighting"))    m.lighting = AlignLightingFromJson(j["lighting"]);
    m.templateFile = j.value("template_file", "");
    if (j.contains("roi"))         m.roi    = RoiFromJson(j["roi"]);
    // Tolerant both ways: a model written before the chain existed has no
    // "filters" at all (every slot stays "none"), and a file from a build with
    // more slots is read up to ours rather than rejected.
    if (j.contains("filters") && j["filters"].is_array()) {
        const nlohmann::json& arr = j["filters"];
        const std::size_t n = static_cast<std::size_t>(kVisionFilterSlots);
        for (std::size_t i = 0; i < arr.size() && i < n; ++i)
            m.filters[i] = FilterFromJson(arr[i]);
    }
    if (j.contains("circle"))      m.circle = CircleFromJson(j["circle"]);
    if (j.contains("ncc"))         m.ncc    = NccFromJson(j["ncc"]);
    if (j.contains("corner"))      m.corner = CornerFromJson(j["corner"]);
    if (j.contains("reliability")) m.rel    = RelFromJson(j["reliability"]);
    return m;
}

std::vector<VisionModel> LoadVisionModels()
{
    std::string path = kLocalPath;
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return std::vector<VisionModel>{};   // first run — start empty
    }
    return ParseVisionModels(f, path, nullptr);
}

std::vector<VisionModel> LoadVisionModelsFile(const std::string& path, std::string* uidOut)
{
    if (uidOut) uidOut->clear();
    std::ifstream f(path);
    if (!f.is_open()) return std::vector<VisionModel>{};
    return ParseVisionModels(f, path, uidOut);
}

bool SaveVisionModelsFile(const std::string& path,
                          const std::vector<VisionModel>& models,
                          const std::string& uid)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const VisionModel& m : models) arr.push_back(VisionModelToJson(m));

    nlohmann::json j;
    if (!uid.empty()) j["uid"] = uid;
    j["models"] = std::move(arr);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("vision", "vision models save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
