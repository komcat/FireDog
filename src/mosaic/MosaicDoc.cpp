// MosaicDoc.cpp
#include "mosaic/MosaicDoc.h"

#include "core/Log.h"

#include <algorithm>
#include <exception>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace firedog {

namespace fs = std::filesystem;

namespace {

nlohmann::json Xyz(double x, double y, double z) { return {{"x", x}, {"y", y}, {"z", z}}; }

void ReadXyz(const nlohmann::json& j, const char* key, double* x, double* y, double* z)
{
    if (!j.contains(key) || !j[key].is_object()) return;
    const nlohmann::json& o = j[key];
    *x = o.value("x", 0.0);
    *y = o.value("y", 0.0);
    *z = o.value("z", 0.0);
}

bool ReadJsonFile(const fs::path& path, nlohmann::json* out, std::string* why)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        if (why) *why = "cannot open " + path.string();
        return false;
    }
    try {
        f >> *out;
        return true;
    } catch (const std::exception& e) {
        if (why) *why = path.string() + " is malformed (" + e.what() + ")";
        return false;
    }
}

// Temp file + rename: a reader sees the old file or the new one, never half.
bool WriteJsonFile(const fs::path& path, const nlohmann::json& j)
{
    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        if (!f.is_open()) {
            FD_WARN("mosaic", "cannot write %s", tmp.string().c_str());
            return false;
        }
        f << j.dump(1) << '\n';
        if (!f.good()) {
            FD_WARN("mosaic", "write to %s failed", tmp.string().c_str());
            return false;
        }
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);   // replaces an existing file on Windows too
    if (ec) {
        FD_WARN("mosaic", "cannot replace %s: %s", path.string().c_str(), ec.message().c_str());
        return false;
    }
    return true;
}

}   // namespace

std::string MosaicSafeName(const std::string& s)
{
    std::string out = s;
    for (char& c : out) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || c == '.';
        if (!ok) c = '_';
    }
    if (out.empty()) out = "_";
    return out;
}

nlohmann::json MosaicRecordToJson(const MosaicRecord& r)
{
    nlohmann::json j;
    j["format"]   = r.format;
    j["status"]   = r.status;
    if (!r.why.empty()) j["why"] = r.why;
    j["created"]  = r.created;
    j["finished"] = r.finished;
    j["device"]   = r.device;
    j["area"]     = MosaicAreaToJson(r.area);
    j["camera"]   = {{"w", r.imgW},
                     {"h", r.imgH},
                     {"mm_per_px", {r.geom.m[0], r.geom.m[1], r.geom.m[2], r.geom.m[3]}},
                     {"calibrated", r.calibrated}};
    j["canvas"]   = {{"g0", {r.geom.g0x, r.geom.g0y}},
                     {"w", r.canvasW},
                     {"h", r.canvasH},
                     {"rows", r.rows},
                     {"cols", r.cols},
                     {"step_x", r.stepX},
                     {"step_y", r.stepY}};
    nlohmann::json q = nlohmann::json::array();
    for (int k = 0; k < 4; ++k) q.push_back({r.quad[k][0], r.quad[k][1]});
    j["outline"] = q;
    j["planned"] = r.planned;
    nlohmann::json ts = nlohmann::json::array();
    for (const MosaicTileRec& t : r.tiles) {
        ts.push_back({{"file", t.file},
                      {"quarter", t.quarter},
                      {"row", t.row},
                      {"col", t.col},
                      {"cmd", Xyz(t.cmdX, t.cmdY, t.cmdZ)},
                      {"meas", Xyz(t.measX, t.measY, t.measZ)},
                      {"x", t.x},
                      {"y", t.y},
                      {"w", t.w},
                      {"h", t.h}});
    }
    j["tiles"] = ts;
    return j;
}

bool MosaicRecordFromJson(const nlohmann::json& j, MosaicRecord* out, std::string* why)
{
    if (!j.is_object()) {
        if (why) *why = "not a mosaic record";
        return false;
    }
    try {
        MosaicRecord r;
        r.format   = j.value("format", 1);
        r.status   = j.value("status", std::string("partial"));
        r.why      = j.value("why", std::string());
        r.created  = j.value("created", std::string());
        r.finished = j.value("finished", std::string());
        r.device   = j.value("device", std::string());
        if (j.contains("area")) r.area = MosaicAreaFromJson(j["area"]);

        double m[4] = {0.0, 0.0, 0.0, 0.0};
        double g0x = 0.0, g0y = 0.0;
        if (j.contains("camera") && j["camera"].is_object()) {
            const nlohmann::json& c = j["camera"];
            r.imgW       = c.value("w", 0);
            r.imgH       = c.value("h", 0);
            r.calibrated = c.value("calibrated", false);
            if (c.contains("mm_per_px") && c["mm_per_px"].is_array() && c["mm_per_px"].size() == 4)
                for (std::size_t i = 0; i < 4; ++i) m[i] = c["mm_per_px"][i].get<double>();
        }
        if (j.contains("canvas") && j["canvas"].is_object()) {
            const nlohmann::json& c = j["canvas"];
            if (c.contains("g0") && c["g0"].is_array() && c["g0"].size() == 2) {
                g0x = c["g0"][0].get<double>();
                g0y = c["g0"][1].get<double>();
            }
            r.canvasW = c.value("w", 0);
            r.canvasH = c.value("h", 0);
            r.rows    = c.value("rows", 0);
            r.cols    = c.value("cols", 0);
            r.stepX   = c.value("step_x", 0.0);
            r.stepY   = c.value("step_y", 0.0);
        }
        if (!MosaicGeomMake(m, g0x, g0y, &r.geom)) {
            if (why) *why = "the record's camera map is missing or singular";
            return false;
        }
        if (j.contains("outline") && j["outline"].is_array() && j["outline"].size() == 4)
            for (std::size_t k = 0; k < 4; ++k) {
                r.quad[k][0] = j["outline"][k][0].get<double>();
                r.quad[k][1] = j["outline"][k][1].get<double>();
            }
        r.planned = j.value("planned", 0);
        if (j.contains("tiles") && j["tiles"].is_array()) {
            for (const nlohmann::json& t : j["tiles"]) {
                MosaicTileRec tr;
                tr.file    = t.value("file", std::string());
                tr.quarter = t.value("quarter", std::string());
                tr.row     = t.value("row", 0);
                tr.col     = t.value("col", 0);
                ReadXyz(t, "cmd", &tr.cmdX, &tr.cmdY, &tr.cmdZ);
                ReadXyz(t, "meas", &tr.measX, &tr.measY, &tr.measZ);
                tr.x = t.value("x", 0.0);
                tr.y = t.value("y", 0.0);
                tr.w = t.value("w", r.imgW);
                tr.h = t.value("h", r.imgH);
                if (tr.file.empty() || tr.w <= 0 || tr.h <= 0) continue;
                r.tiles.push_back(std::move(tr));
            }
        }
        *out = std::move(r);
        return true;
    } catch (const std::exception& e) {
        if (why) *why = std::string("malformed mosaic record: ") + e.what();
        return false;
    }
}

bool SaveMosaicRecord(const std::string& dir, const MosaicRecord& r)
{
    return WriteJsonFile(fs::path(dir) / kMosaicRecordFile, MosaicRecordToJson(r));
}

bool LoadMosaicRecord(const std::string& dir, MosaicRecord* out, std::string* why)
{
    nlohmann::json j;
    if (!ReadJsonFile(fs::path(dir) / kMosaicRecordFile, &j, why)) return false;
    return MosaicRecordFromJson(j, out, why);
}

bool LoadMosaicAnnotations(const std::string& dir, MosaicAnnotations* out)
{
    *out = MosaicAnnotations{};
    const fs::path path = fs::path(dir) / kMosaicAnnotationsFile;
    std::error_code ec;
    if (!fs::exists(path, ec)) return true;   // none yet
    nlohmann::json j;
    std::string why;
    if (!ReadJsonFile(path, &j, &why)) {
        FD_WARN("mosaic", "annotations not loaded: %s", why.c_str());
        return false;
    }
    try {
        if (j.contains("markers") && j["markers"].is_array())
            for (const nlohmann::json& m : j["markers"]) {
                MosaicMarker mk;
                mk.uid  = m.value("uid", std::string());
                mk.name = m.value("name", std::string("marker"));
                mk.tag  = m.value("tag", std::string());
                mk.cx   = m.value("cx", 0.0);
                mk.cy   = m.value("cy", 0.0);
                mk.at   = m.value("at", std::string());
                out->markers.push_back(std::move(mk));
            }
        if (j.contains("labels") && j["labels"].is_array())
            for (const nlohmann::json& l : j["labels"]) {
                MosaicLabel lb;
                lb.uid  = l.value("uid", std::string());
                lb.name = l.value("name", std::string("label"));
                lb.tag  = l.value("tag", std::string());
                lb.x0   = l.value("x0", 0.0);
                lb.y0   = l.value("y0", 0.0);
                lb.x1   = l.value("x1", 0.0);
                lb.y1   = l.value("y1", 0.0);
                lb.at   = l.value("at", std::string());
                if (lb.x1 < lb.x0) std::swap(lb.x0, lb.x1);
                if (lb.y1 < lb.y0) std::swap(lb.y0, lb.y1);
                out->labels.push_back(std::move(lb));
            }
    } catch (const std::exception& e) {
        FD_WARN("mosaic", "annotations malformed (%s) - ignored", e.what());
        *out = MosaicAnnotations{};
        return false;
    }
    return true;
}

bool SaveMosaicAnnotations(const std::string& dir, const MosaicAnnotations& a)
{
    nlohmann::json j;
    nlohmann::json ms = nlohmann::json::array();
    for (const MosaicMarker& m : a.markers)
        ms.push_back({{"uid", m.uid}, {"name", m.name}, {"tag", m.tag}, {"cx", m.cx}, {"cy", m.cy},
                      {"at", m.at}});
    nlohmann::json ls = nlohmann::json::array();
    for (const MosaicLabel& l : a.labels)
        ls.push_back({{"uid", l.uid},
                      {"name", l.name},
                      {"tag", l.tag},
                      {"x0", l.x0},
                      {"y0", l.y0},
                      {"x1", l.x1},
                      {"y1", l.y1},
                      {"at", l.at}});
    j["markers"] = ms;
    j["labels"]  = ls;
    return WriteJsonFile(fs::path(dir) / kMosaicAnnotationsFile, j);
}

bool MosaicRecordExtent(const MosaicRecord& r, double* x0, double* y0, double* x1, double* y1,
                        bool useRefined)
{
    if (r.tiles.empty()) return false;
    double a0 = 1e300, b0 = 1e300, a1 = -1e300, b1 = -1e300;
    for (const MosaicTileRec& t : r.tiles) {
        const double tx = MosaicTileX(r, t, useRefined), ty = MosaicTileY(r, t, useRefined);
        a0 = std::min(a0, tx);
        b0 = std::min(b0, ty);
        a1 = std::max(a1, tx + t.w);
        b1 = std::max(b1, ty + t.h);
    }
    *x0 = a0; *y0 = b0; *x1 = a1; *y1 = b1;
    return true;
}

bool SaveMosaicPlacement(const std::string& dir, const std::vector<double>& x,
                         const std::vector<double>& y, const std::string& note)
{
    nlohmann::json ts = nlohmann::json::array();
    for (std::size_t i = 0; i < x.size() && i < y.size(); ++i) ts.push_back({x[i], y[i]});
    nlohmann::json j;
    j["tiles"] = ts;
    j["note"]  = note;
    return WriteJsonFile(fs::path(dir) / kMosaicPlacementFile, j);
}

void LoadMosaicPlacement(const std::string& dir, MosaicRecord* r)
{
    r->refined = false;
    r->refinedNote.clear();
    const fs::path path = fs::path(dir) / kMosaicPlacementFile;
    std::error_code ec;
    if (!fs::exists(path, ec)) return;
    nlohmann::json j;
    std::string why;
    if (!ReadJsonFile(path, &j, &why)) {
        FD_WARN("mosaic", "placement not loaded: %s", why.c_str());
        return;
    }
    try {
        if (!j.contains("tiles") || !j["tiles"].is_array() || j["tiles"].size() != r->tiles.size()) {
            FD_WARN("mosaic", "%s does not match the record's %zu tiles - ignored",
                    path.string().c_str(), r->tiles.size());
            return;
        }
        for (std::size_t i = 0; i < r->tiles.size(); ++i) {
            r->tiles[i].rx = j["tiles"][i][0].get<double>();
            r->tiles[i].ry = j["tiles"][i][1].get<double>();
        }
        r->refined     = true;
        r->refinedNote = j.value("note", std::string());
    } catch (const std::exception& e) {
        FD_WARN("mosaic", "placement malformed (%s) - ignored", e.what());
        r->refined = false;
    }
}

std::vector<MosaicTileRef> MosaicTileRefs(const MosaicRecord& r, bool useRefined)
{
    std::vector<MosaicTileRef> refs;
    refs.reserve(r.tiles.size());
    for (const MosaicTileRec& t : r.tiles) {
        MosaicTileRef ref;
        ref.x = MosaicTileX(r, t, useRefined);
        ref.y = MosaicTileY(r, t, useRefined);
        ref.w = t.w;
        ref.h = t.h;
        refs.push_back(ref);
    }
    return refs;
}

std::vector<std::string> ListMosaicFolders(const std::string& saveDir)
{
    std::vector<std::string> names;
    std::error_code ec;
    if (saveDir.empty() || !fs::is_directory(saveDir, ec)) return names;
    for (fs::directory_iterator it(saveDir, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_directory(ec)) continue;
        if (!fs::exists(it->path() / kMosaicRecordFile, ec)) continue;
        names.push_back(it->path().filename().string());
    }
    // "<area>_YYYYMMDD_HHMMSS": the last 15 characters are the stamp.
    const auto stamp = [](const std::string& n) {
        return n.size() >= 15 ? n.substr(n.size() - 15) : n;
    };
    std::sort(names.begin(), names.end(), [&](const std::string& a, const std::string& b) {
        const std::string sa = stamp(a), sb = stamp(b);
        return sa != sb ? sa > sb : a < b;
    });
    return names;
}

} // namespace firedog
