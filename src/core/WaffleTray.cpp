// WaffleTray.cpp
#include "core/WaffleTray.h"

#include "core/Log.h"
#include "core/Paths.h"
#include "core/Product.h"   // GenerateUid

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>

namespace firedog {

namespace {
std::string LocalPath() { return MachinePath("config/trays.local.json"); }
constexpr const char* kBasePath = "config/trays.json";   // shipped default, beside the exe

std::vector<WaffleTray> ParseWaffleTrays(std::ifstream& f, const std::string& path,
                                         std::string* uidOut)
{
    std::vector<WaffleTray> out;
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", "");
        if (!j.contains("trays") || !j["trays"].is_array()) return out;
        for (const auto& row : j["trays"]) {
            if (!row.is_object()) continue;
            out.push_back(WaffleTrayFromJson(row));
        }
        FD_INFO("tray", "loaded %zu waffle tray(s) from %s", out.size(), path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("tray", "waffle trays load failed (%s): starting empty", e.what());
        return std::vector<WaffleTray>{};
    }
    return out;
}

double AxisOr0(const std::map<std::string, double>& m, const char* axis)
{
    const auto it = m.find(axis);
    return (it != m.end()) ? it->second : 0.0;
}

// b - a over X/Y/Z, for the pitch and the parallel check. Missing axes read 0.
void XyzDelta(const std::map<std::string, double>& a, const std::map<std::string, double>& b,
              double v[3])
{
    v[0] = AxisOr0(b, "X") - AxisOr0(a, "X");
    v[1] = AxisOr0(b, "Y") - AxisOr0(a, "Y");
    v[2] = AxisOr0(b, "Z") - AxisOr0(a, "Z");
}

double Norm3(const double v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool SameAxes(const std::map<std::string, double>& a, const std::map<std::string, double>& b)
{
    if (a.size() != b.size()) return false;
    for (const auto& kv : a)
        if (b.find(kv.first) == b.end()) return false;
    return true;
}

// Where a row starts: the row end when there is one, else the origin (a
// one-column tray's far corner is (R,1)).
const TrayCorner& RowStart(const WaffleTray& t)
{
    return TrayNeedsRowEnd(t) ? t.rowEnd : t.origin;
}

nlohmann::json CornerToJson(const TrayCorner& c)
{
    nlohmann::json j;
    j["have"] = c.have;
    j["at"]   = c.taughtAt;
    nlohmann::json vals = nlohmann::json::object();
    for (const auto& kv : c.vals) vals[kv.first] = kv.second;
    j["vals"] = std::move(vals);
    return j;
}

TrayCorner CornerFromJson(const nlohmann::json& j)
{
    TrayCorner c;
    if (!j.is_object()) return c;
    c.have     = j.value("have", false);
    c.taughtAt = j.value("at", "");
    if (j.contains("vals") && j["vals"].is_object()) {
        for (const auto& kv : j["vals"].items())
            if (kv.value().is_number()) c.vals[kv.key()] = kv.value().get<double>();
    }
    if (c.vals.empty()) c.have = false;   // a taught corner with no axes is untaught
    return c;
}

} // namespace

// ---- lookup ----------------------------------------------------------------

const WaffleTray* FindWaffleTray(const std::vector<WaffleTray>& list,
                                 const std::string& uidOrName)
{
    if (uidOrName.empty()) return nullptr;
    for (const WaffleTray& t : list)
        if (t.uid == uidOrName) return &t;
    for (const WaffleTray& t : list)
        if (t.name == uidOrName) return &t;
    return nullptr;
}

WaffleTray* FindWaffleTray(std::vector<WaffleTray>& list, const std::string& uidOrName)
{
    const std::vector<WaffleTray>& c = list;
    return const_cast<WaffleTray*>(FindWaffleTray(c, uidOrName));
}

// ---- corners ---------------------------------------------------------------

const char* TrayCornerLabel(TrayCornerSlot s)
{
    switch (s) {
    case TrayCornerSlot::Origin: return "origin (1,1)";
    case TrayCornerSlot::RowEnd: return "row end (1,C)";
    case TrayCornerSlot::Far:    return "far corner (R,C)";
    }
    return "corner";
}

TrayCorner& TrayCornerOf(WaffleTray& t, TrayCornerSlot s)
{
    switch (s) {
    case TrayCornerSlot::Origin: return t.origin;
    case TrayCornerSlot::RowEnd: return t.rowEnd;
    case TrayCornerSlot::Far:    return t.far;
    }
    return t.origin;
}

const TrayCorner& TrayCornerOf(const WaffleTray& t, TrayCornerSlot s)
{
    return TrayCornerOf(const_cast<WaffleTray&>(t), s);
}

bool TrayNeedsRowEnd(const WaffleTray& t) { return t.cols > 1; }
bool TrayNeedsFar(const WaffleTray& t)    { return t.rows > 1; }

// ---- geometry --------------------------------------------------------------

std::string TrayValidate(const WaffleTray& t)
{
    char b[192];
    if (t.rows < 1 || t.cols < 1 || t.rows > kTrayMaxRows || t.cols > kTrayMaxCols) {
        std::snprintf(b, sizeof(b), "rows and columns must be 1..%d x 1..%d (have %d x %d)",
                      kTrayMaxRows, kTrayMaxCols, t.rows, t.cols);
        return b;
    }
    if (!t.origin.have || t.origin.vals.empty()) return "origin (1,1) is not taught";
    if (TrayNeedsRowEnd(t)) {
        if (!t.rowEnd.have) return "row end (1,C) is not taught";
        if (!SameAxes(t.origin.vals, t.rowEnd.vals))
            return "row end (1,C) does not carry the origin's axes - retaught on another device?";
    }
    if (TrayNeedsFar(t)) {
        if (!t.far.have) return "far corner (R,C) is not taught";
        if (!SameAxes(t.origin.vals, t.far.vals))
            return "far corner (R,C) does not carry the origin's axes - retaught on another device?";
    }

    double colV[3] = {0, 0, 0}, rowV[3] = {0, 0, 0};
    if (TrayNeedsRowEnd(t)) {
        XyzDelta(t.origin.vals, t.rowEnd.vals, colV);
        const double pitch = Norm3(colV) / (t.cols - 1);
        if (pitch < kTrayMinPitchMm) {
            std::snprintf(b, sizeof(b),
                          "column pitch is %.4f mm - the row end sits on the origin",
                          pitch);
            return b;
        }
    }
    if (TrayNeedsFar(t)) {
        XyzDelta(RowStart(t).vals, t.far.vals, rowV);
        const double pitch = Norm3(rowV) / (t.rows - 1);
        if (pitch < kTrayMinPitchMm) {
            std::snprintf(b, sizeof(b),
                          "row pitch is %.4f mm - the far corner sits on the row end",
                          pitch);
            return b;
        }
    }
    if (TrayNeedsRowEnd(t) && TrayNeedsFar(t)) {
        // |a x b| / (|a||b|) = sin of the angle between the two directions.
        const double cx = colV[1] * rowV[2] - colV[2] * rowV[1];
        const double cy = colV[2] * rowV[0] - colV[0] * rowV[2];
        const double cz = colV[0] * rowV[1] - colV[1] * rowV[0];
        const double cross[3] = {cx, cy, cz};
        const double sinA = Norm3(cross) / (Norm3(colV) * Norm3(rowV));
        if (sinA < 1.0e-3)
            return "row and column directions are parallel - the far corner lies on the first row";
    }
    return {};
}

bool TrayCellInRange(const WaffleTray& t, int r, int c)
{
    return r >= 1 && c >= 1 && r <= t.rows && c <= t.cols;
}

bool TrayCellEnabled(const WaffleTray& t, int r, int c)
{
    if (!TrayCellInRange(t, r, c)) return false;
    if (t.disabledRows.count(r) || t.disabledCols.count(c)) return false;
    return t.disabledCells.count(TrayCell{r, c}) == 0;
}

bool TrayCellUsed(const WaffleTray& t, int r, int c)
{
    return t.usedCells.count(TrayCell{r, c}) > 0;
}

bool TrayCellPose(const WaffleTray& t, int r, int c, std::map<std::string, double>* out)
{
    if (!TrayValidate(t).empty() || !TrayCellInRange(t, r, c)) return false;
    const double fc = TrayNeedsRowEnd(t) ? double(c - 1) / double(t.cols - 1) : 0.0;
    const double fr = TrayNeedsFar(t)    ? double(r - 1) / double(t.rows - 1) : 0.0;
    const TrayCorner& rowStart = RowStart(t);
    std::map<std::string, double> pose;
    for (const auto& kv : t.origin.vals) {
        const std::string& axis = kv.first;
        double v = kv.second;
        if (TrayNeedsRowEnd(t)) v += fc * (AxisOr0(t.rowEnd.vals, axis.c_str()) - kv.second);
        if (TrayNeedsFar(t))
            v += fr * (AxisOr0(t.far.vals, axis.c_str()) - AxisOr0(rowStart.vals, axis.c_str()));
        pose[axis] = v;
    }
    if (out) *out = std::move(pose);
    return true;
}

bool TrayPitch(const WaffleTray& t, double* colPitchMm, double* rowPitchMm)
{
    if (colPitchMm) *colPitchMm = 0.0;
    if (rowPitchMm) *rowPitchMm = 0.0;
    if (!t.origin.have) return false;
    double v[3];
    if (TrayNeedsRowEnd(t) && t.rowEnd.have && colPitchMm) {
        XyzDelta(t.origin.vals, t.rowEnd.vals, v);
        *colPitchMm = Norm3(v) / (t.cols - 1);
    }
    if (TrayNeedsFar(t) && t.far.have && rowPitchMm) {
        const TrayCorner& rs = RowStart(t);
        if (rs.have) {
            XyzDelta(rs.vals, t.far.vals, v);
            *rowPitchMm = Norm3(v) / (t.rows - 1);
        }
    }
    return true;
}

std::vector<TrayCell> TrayEnabledCells(const WaffleTray& t, TrayOrder order)
{
    std::vector<TrayCell> out;
    if (t.rows < 1 || t.cols < 1) return out;
    out.reserve(static_cast<std::size_t>(t.rows) * static_cast<std::size_t>(t.cols));
    switch (order) {
    case TrayOrder::RowMajor:
        for (int r = 1; r <= t.rows; ++r)
            for (int c = 1; c <= t.cols; ++c)
                if (TrayCellEnabled(t, r, c)) out.emplace_back(r, c);
        break;
    case TrayOrder::ColMajor:
        for (int c = 1; c <= t.cols; ++c)
            for (int r = 1; r <= t.rows; ++r)
                if (TrayCellEnabled(t, r, c)) out.emplace_back(r, c);
        break;
    case TrayOrder::Serpentine:
        for (int r = 1; r <= t.rows; ++r) {
            if (r % 2 == 1) {
                for (int c = 1; c <= t.cols; ++c)
                    if (TrayCellEnabled(t, r, c)) out.emplace_back(r, c);
            } else {
                for (int c = t.cols; c >= 1; --c)
                    if (TrayCellEnabled(t, r, c)) out.emplace_back(r, c);
            }
        }
        break;
    }
    return out;
}

std::size_t TrayCellCount(const WaffleTray& t)
{
    if (t.rows < 1 || t.cols < 1) return 0;
    return static_cast<std::size_t>(t.rows) * static_cast<std::size_t>(t.cols);
}

void TrayNormalize(WaffleTray& t)
{
    t.rows = std::clamp(t.rows, 1, kTrayMaxRows);
    t.cols = std::clamp(t.cols, 1, kTrayMaxCols);
    for (auto it = t.disabledRows.begin(); it != t.disabledRows.end();)
        it = (*it < 1 || *it > t.rows) ? t.disabledRows.erase(it) : std::next(it);
    for (auto it = t.disabledCols.begin(); it != t.disabledCols.end();)
        it = (*it < 1 || *it > t.cols) ? t.disabledCols.erase(it) : std::next(it);
    for (auto it = t.disabledCells.begin(); it != t.disabledCells.end();)
        it = TrayCellInRange(t, it->first, it->second) ? std::next(it)
                                                       : t.disabledCells.erase(it);
    for (auto it = t.usedCells.begin(); it != t.usedCells.end();)
        it = TrayCellInRange(t, it->first, it->second) ? std::next(it)
                                                       : t.usedCells.erase(it);
}

const char* TrayOrderName(TrayOrder o)
{
    switch (o) {
    case TrayOrder::RowMajor:   return "row_major";
    case TrayOrder::ColMajor:   return "col_major";
    case TrayOrder::Serpentine: return "serpentine";
    }
    return "row_major";
}

bool TrayOrderFromName(const std::string& s, TrayOrder* out)
{
    TrayOrder o;
    if      (s == "row_major")  o = TrayOrder::RowMajor;
    else if (s == "col_major")  o = TrayOrder::ColMajor;
    else if (s == "serpentine") o = TrayOrder::Serpentine;
    else return false;
    if (out) *out = o;
    return true;
}

// ---- persistence -----------------------------------------------------------

nlohmann::json WaffleTrayToJson(const WaffleTray& t)
{
    nlohmann::json j;
    j["uid"]     = t.uid;
    j["name"]    = t.name;
    j["device"]  = t.device;
    j["rows"]    = t.rows;
    j["cols"]    = t.cols;
    j["origin"]  = CornerToJson(t.origin);
    j["row_end"] = CornerToJson(t.rowEnd);
    j["far"]     = CornerToJson(t.far);

    nlohmann::json rows = nlohmann::json::array();
    for (int r : t.disabledRows) rows.push_back(r);
    nlohmann::json cols = nlohmann::json::array();
    for (int c : t.disabledCols) cols.push_back(c);
    nlohmann::json cells = nlohmann::json::array();
    for (const TrayCell& cell : t.disabledCells)
        cells.push_back(nlohmann::json::array({cell.first, cell.second}));
    nlohmann::json used = nlohmann::json::array();
    for (const TrayCell& cell : t.usedCells)
        used.push_back(nlohmann::json::array({cell.first, cell.second}));
    j["disabled_rows"]  = std::move(rows);
    j["disabled_cols"]  = std::move(cols);
    j["disabled_cells"] = std::move(cells);
    j["used_cells"]     = std::move(used);
    return j;
}

WaffleTray WaffleTrayFromJson(const nlohmann::json& j)
{
    WaffleTray t;
    if (!j.is_object()) return t;
    t.uid = j.value("uid", "");
    if (t.uid.empty()) t.uid = GenerateUid();   // legacy row - assign once
    t.name   = j.value("name", "tray");
    t.device = j.value("device", "");
    t.rows   = j.value("rows", 1);
    t.cols   = j.value("cols", 1);
    if (j.contains("origin"))  t.origin = CornerFromJson(j["origin"]);
    if (j.contains("row_end")) t.rowEnd = CornerFromJson(j["row_end"]);
    if (j.contains("far"))     t.far    = CornerFromJson(j["far"]);

    if (j.contains("disabled_rows") && j["disabled_rows"].is_array())
        for (const auto& v : j["disabled_rows"])
            if (v.is_number_integer()) t.disabledRows.insert(v.get<int>());
    if (j.contains("disabled_cols") && j["disabled_cols"].is_array())
        for (const auto& v : j["disabled_cols"])
            if (v.is_number_integer()) t.disabledCols.insert(v.get<int>());
    const auto readCells = [&](const char* key, std::set<TrayCell>& into) {
        if (!j.contains(key) || !j[key].is_array()) return;
        for (const auto& v : j[key])
            if (v.is_array() && v.size() == 2 && v[0].is_number_integer() &&
                v[1].is_number_integer())
                into.insert(TrayCell{v[0].get<int>(), v[1].get<int>()});
    };
    readCells("disabled_cells", t.disabledCells);
    readCells("used_cells", t.usedCells);
    TrayNormalize(t);
    return t;
}

std::vector<WaffleTray> LoadWaffleTrays()
{
    std::string path = LocalPath();
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return std::vector<WaffleTray>{};   // first run - empty
    }
    return ParseWaffleTrays(f, path, nullptr);
}

std::vector<WaffleTray> LoadWaffleTraysFile(const std::string& path, std::string* uidOut)
{
    if (uidOut) uidOut->clear();
    std::ifstream f(path);
    if (!f.is_open()) return std::vector<WaffleTray>{};
    return ParseWaffleTrays(f, path, uidOut);
}

bool SaveWaffleTraysFile(const std::string& path, const std::vector<WaffleTray>& list,
                         const std::string& uid)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const WaffleTray& t : list) arr.push_back(WaffleTrayToJson(t));

    nlohmann::json j;
    if (!uid.empty()) j["uid"] = uid;
    j["trays"] = std::move(arr);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("tray", "waffle trays save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

} // namespace firedog
