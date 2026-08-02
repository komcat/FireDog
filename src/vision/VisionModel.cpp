// VisionModel.cpp
#include "vision/VisionModel.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid

#include <cctype>
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
    j["circle"]        = CircleToJson(m.circle);
    j["ncc"]           = NccToJson(m.ncc);
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
    m.templateFile = j.value("template_file", "");
    if (j.contains("roi"))         m.roi    = RoiFromJson(j["roi"]);
    if (j.contains("circle"))      m.circle = CircleFromJson(j["circle"]);
    if (j.contains("ncc"))         m.ncc    = NccFromJson(j["ncc"]);
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
