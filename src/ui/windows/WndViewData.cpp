// WndViewData.cpp : the View Data window (View menu, READ-ONLY).
//
// The DUT Records page shows ONE record at a time — its whole entry log, in
// order, which is the right view for "what happened to this part". This window
// is the other question: "how do the parts compare". It flattens every
// dut.json under the loaded product's data/ into one table — a row per serial,
// a column per recorded key — and charts one or two keys across the parts,
// either as a series in table order or, when a series is too crowded to read,
// as daily / weekly / monthly box plots.
//
// A window rather than a page so it can sit on a second monitor beside the Run
// page while parts go through. It writes nothing: every mutation of a record
// goes through DutSession via the dut_* commands, and the Explorer shortcut is
// the only thing here that leaves the application, gated like the DUT block's.
//
// The scan is synchronous on the UI thread — a few KB per record, so a few
// hundred parts is milliseconds — and runs on open, on product change and on
// Reload, never per frame.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/Widgets.h"   // OpenFolderInExplorer, StateColor
#include "core/Log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

// Fixed identity columns ahead of the per-key ones.
constexpr int kColSerial = 0;
constexpr int kColOpened = 1;
constexpr int kColClosed = 2;
constexpr int kColStatus = 3;
constexpr int kColRuns   = 4;
constexpr int kFixedCols = 5;

// ImGui's own ceiling is 512; a table wider than this is unreadable anyway,
// and the cut is reported in the window rather than silently applied.
constexpr int kMaxKeyCols = 200;

constexpr const char* kPlotModes[4] = {"Series", "Daily box", "Weekly box", "Monthly box"};
constexpr const char* kTimeSels[4]  = {"All time", "This month", "Last month", "Year to date"};

// The two series keep fixed colours so a legend entry means the same thing
// whichever keys are picked.
const ImVec4 kColA(0.93f, 0.55f, 0.25f, 1.0f);
const ImVec4 kColB(0.35f, 0.65f, 0.95f, 1.0f);

std::string CellText(const nlohmann::json& v)
{
    if (v.is_null())    return "-";
    if (v.is_string())  return v.get<std::string>();
    if (v.is_boolean()) return v.get<bool>() ? "true" : "false";
    if (v.is_number()) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", v.get<double>());
        return buf;
    }
    return v.dump();
}

// Missing values sort last in either direction; numbers sort ahead of text
// and against each other numerically, everything else by its rendered text.
int CompareValues(const nlohmann::json* a, const nlohmann::json* b)
{
    if (!a && !b) return 0;
    if (!a) return 1;
    if (!b) return -1;
    const bool na = a->is_number(), nb = b->is_number();
    if (na && nb) {
        const double x = a->get<double>(), y = b->get<double>();
        return x < y ? -1 : (x > y ? 1 : 0);
    }
    if (na != nb) return na ? -1 : 1;
    return CellText(*a).compare(CellText(*b));
}

// "YYYY-MM" for the UTC month `offset` months from now (0 = this, -1 = last).
// The records stamp UTC ISO strings, so a prefix compare IS the date filter —
// no parsing, and no local-time boundary that would move a part at midnight.
std::string MonthPrefix(int offset)
{
    const std::time_t now = std::time(nullptr);
    std::tm t{};
    gmtime_s(&t, &now);
    int y = t.tm_year + 1900, m = t.tm_mon + 1 + offset;
    while (m < 1)  { m += 12; --y; }
    while (m > 12) { m -= 12; ++y; }
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d-%02d", y, m);
    return buf;
}

std::string YearPrefix()
{
    const std::time_t now = std::time(nullptr);
    std::tm t{};
    gmtime_s(&t, &now);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%04d", t.tm_year + 1900);
    return buf;
}

// The box-plot bucket a record falls in, from its opened_at stamp: the day,
// the week (named by its Monday, so buckets still sort as text) or the month.
// Empty when the stamp is missing or malformed — such a record is left out of
// the chart rather than dropped into a bucket it may not belong to.
std::string BucketKey(const std::string& opened, int mode)
{
    if (opened.size() < 10) return std::string();
    switch (mode) {
        case 1: return opened.substr(0, 10);
        case 3: return opened.substr(0, 7);
        case 2: {
            int y = 0, m = 0, d = 0;
            if (std::sscanf(opened.c_str(), "%4d-%2d-%2d", &y, &m, &d) != 3) return std::string();
            using namespace std::chrono;
            const year_month_day ymd{year{y}, month{static_cast<unsigned>(m)},
                                     day{static_cast<unsigned>(d)}};
            if (!ymd.ok()) return std::string();
            const sys_days sd{ymd};
            const int wd = static_cast<int>(weekday{sd}.iso_encoding());   // Mon = 1 .. Sun = 7
            const year_month_day mon{sd - days{wd - 1}};
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%04d-%02u-%02u", static_cast<int>(mon.year()),
                          static_cast<unsigned>(mon.month()), static_cast<unsigned>(mon.day()));
            return buf;
        }
        default: return std::string();
    }
}

struct BoxStats {
    int    n = 0;
    double lo = 0, q1 = 0, med = 0, q3 = 0, hi = 0;
};

// Five-number summary; quartiles by linear interpolation between order
// statistics (the usual spreadsheet definition). Sorts its input.
BoxStats Summarise(std::vector<double>& v)
{
    BoxStats s;
    s.n = static_cast<int>(v.size());
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    auto q = [&](double p) {
        const double pos = p * static_cast<double>(v.size() - 1);
        const std::size_t i = static_cast<std::size_t>(pos);
        const double f = pos - static_cast<double>(i);
        return i + 1 < v.size() ? v[i] + (v[i + 1] - v[i]) * f : v[i];
    };
    s.lo  = v.front();
    s.q1  = q(0.25);
    s.med = q(0.5);
    s.q3  = q(0.75);
    s.hi  = v.back();
    return s;
}

// Text rotated a quarter turn anticlockwise, reading bottom-to-top, with its
// END at `top` (so a column of labels of unequal length lines up against the
// axis) and centred on `cx`. ImGui has no rotated text, and AddText culls
// glyphs against the clip rect BEFORE any vertex could be turned, so this
// places the glyph quads itself — the same thing ImPlot does for its own
// vertical axis label, without reaching into its internals.
void AddTextVertical(ImDrawList* dl, float cx, float top, ImU32 col, const char* text)
{
    ImFontBaked* baked = ImGui::GetFontBaked();
    if (!baked) return;
    const float scale = ImGui::GetFontSize() / baked->Size;
    const float lineH = ImGui::GetFontSize();
    const float len   = ImGui::CalcTextSize(text).x;
    // The unrotated origin: text runs from (ox, oy) rightwards; we map its
    // (dx, dy) to (dy, -dx) about that origin, so it runs upwards from there.
    const float ox = std::floor(cx - lineH * 0.5f);
    const float oy = std::floor(top + len);

    int count = 0;
    for (const char* s = text; *s; ++s) ++count;
    dl->PushTexture(ImGui::GetIO().Fonts->TexRef);
    dl->PrimReserve(count * 6, count * 4);
    int used = 0;
    float pen = 0.0f;
    for (const char* s = text; *s; ++s) {
        const ImFontGlyph* g = baked->FindGlyph(static_cast<ImWchar>(static_cast<unsigned char>(*s)));
        if (!g) continue;
        if (g->Visible) {
            const float x0 = pen + g->X0 * scale, x1 = pen + g->X1 * scale;
            const float y0 = g->Y0 * scale,       y1 = g->Y1 * scale;
            // (dx, dy) -> (ox + dy, oy - dx)
            const ImVec2 a(ox + y0, oy - x0), b(ox + y0, oy - x1);
            const ImVec2 c(ox + y1, oy - x1), d(ox + y1, oy - x0);
            dl->PrimQuadUV(a, b, c, d, ImVec2(g->U0, g->V0), ImVec2(g->U1, g->V0),
                           ImVec2(g->U1, g->V1), ImVec2(g->U0, g->V1), col);
            ++used;
        }
        pen += g->AdvanceX * scale;
    }
    if (used < count) dl->PrimUnreserve((count - used) * 6, (count - used) * 4);
    dl->PopTexture();
}

// The label strip under a plot: one vertical label per pixel column in `px`,
// thinned to every k-th so neighbours never overprint. `left..right` is the
// plot area's horizontal span; labels outside it (a zoomed plot) are skipped.
void DrawLabelStrip(const std::vector<float>& px, const std::vector<const char*>& labels,
                    float left, float right, float stripH)
{
    const ImVec2 top = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x, stripH));
    if (px.empty() || px.size() != labels.size()) return;

    float minGap = 1e9f;
    for (std::size_t i = 1; i < px.size(); ++i)
        minGap = std::min(minGap, std::fabs(px[i] - px[i - 1]));
    const float pitch = ImGui::GetFontSize() + 2.0f;
    const int   step  = minGap >= pitch ? 1 : static_cast<int>(std::ceil(pitch / std::max(minGap, 1.0f)));

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 col = ImGui::GetColorU32(ImGuiCol_Text);
    dl->PushClipRect(ImVec2(left, top.y), ImVec2(right, top.y + stripH), true);
    for (std::size_t i = 0; i < px.size(); i += static_cast<std::size_t>(step)) {
        if (px[i] < left - 1.0f || px[i] > right + 1.0f) continue;
        AddTextVertical(dl, px[i], top.y + 2.0f, col, labels[i]);
    }
    dl->PopClipRect();
}

} // namespace

void App::LoadDataRows()
{
    namespace fs = std::filesystem;
    // Keep the charted keys by NAME across a rescan — the index they had means
    // nothing once a new key appears earlier in the union.
    auto keyName = [&](int i) {
        return i >= 0 && i < static_cast<int>(dataKeys_.size())
                   ? dataKeys_[static_cast<std::size_t>(i)] : std::string();
    };
    const std::string keepA = keyName(dataKeySel_), keepB = keyName(dataKeySel2_);
    dataRows_.clear();
    dataKeys_.clear();
    dataKeySel_  = -1;
    dataKeySel2_ = -1;
    dataDir_     = session_.ProductLoaded() ? session_.Product().DataDir() : std::string();
    if (dataDir_.empty()) return;

    std::set<std::string> seen;
    std::error_code ec;
    for (const fs::directory_entry& e : fs::directory_iterator(dataDir_, ec)) {
        if (!e.is_directory(ec)) continue;
        const fs::path file = e.path() / "dut.json";
        if (!fs::exists(file, ec)) continue;

        nlohmann::json rec;
        try {
            std::ifstream f(file);
            if (f.is_open()) f >> rec;
        } catch (const std::exception& ex) {
            FD_WARN("dut", "record '%s' unreadable: %s",
                    e.path().filename().string().c_str(), ex.what());
            continue;
        }
        if (!rec.is_object()) continue;

        DutDataRow row;
        row.serial = rec.value("serial", e.path().filename().string());
        row.opened = rec.value("opened_at", std::string());
        row.closed = rec.value("closed_at", std::string());
        row.status = rec.value("status", std::string());
        if (rec.contains("entries") && rec["entries"].is_array()) {
            for (const nlohmann::json& en : rec["entries"]) {
                const std::string key = en.value("key", std::string());
                if (key.empty()) continue;
                const nlohmann::json v = en.contains("value") ? en["value"] : nlohmann::json();
                if (key == "$run") {
                    if (v.is_object())
                        row.runs.push_back(v.value("kind", std::string()) + " '" +
                                           v.value("name", std::string()) + "' - " +
                                           v.value("result", std::string()));
                    continue;
                }
                if (key[0] == '$') continue;   // $open / $close / $operator: not data
                if (seen.insert(key).second) dataKeys_.push_back(key);
                row.vals[key] = v;             // last write wins — see DutDataRow
            }
        }
        dataRows_.push_back(std::move(row));
    }

    for (int i = 0; i < static_cast<int>(dataKeys_.size()); ++i) {
        if (dataKeys_[static_cast<std::size_t>(i)] == keepA) dataKeySel_  = i;
        if (dataKeys_[static_cast<std::size_t>(i)] == keepB) dataKeySel2_ = i;
    }
    if (dataKeySel_ < 0 && !dataKeys_.empty()) dataKeySel_ = 0;

    SortDataRows();
    FD_INFO("dut", "view data: %zu records, %zu keys under %s", dataRows_.size(),
            dataKeys_.size(), dataDir_.c_str());
}

void App::SortDataRows()
{
    const int  col = dataSortCol_;
    const bool asc = dataSortAsc_;
    std::stable_sort(dataRows_.begin(), dataRows_.end(),
                     [&](const DutDataRow& a, const DutDataRow& b) {
        int c = 0;
        switch (col) {
            case kColSerial: c = a.serial.compare(b.serial); break;
            case kColOpened: c = a.opened.compare(b.opened); break;
            case kColClosed: c = a.closed.compare(b.closed); break;
            case kColStatus: c = a.status.compare(b.status); break;
            case kColRuns:
                c = a.runs.size() < b.runs.size() ? -1 : (a.runs.size() > b.runs.size() ? 1 : 0);
                break;
            default: {
                const int k = col - kFixedCols;
                if (k < 0 || k >= static_cast<int>(dataKeys_.size())) break;
                const std::string& key = dataKeys_[static_cast<std::size_t>(k)];
                const auto ia = a.vals.find(key), ib = b.vals.find(key);
                const nlohmann::json* va = ia == a.vals.end() ? nullptr : &ia->second;
                const nlohmann::json* vb = ib == b.vals.end() ? nullptr : &ib->second;
                // Missing stays last whichever way the column is flipped.
                if (!va || !vb) return va != nullptr && vb == nullptr;
                c = CompareValues(va, vb);
                break;
            }
        }
        return asc ? c < 0 : c > 0;
    });
}

void App::DrawViewDataWindow()
{
    // The menu item is gated, but the open bit outlives a page switch and a
    // role never should.
    if (!CanUse(kRoleData)) { showData_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(1000.0f, 700.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("View Data", &showData_)) {
        ImGui::End();
        return;
    }

    // Rescan when the product under the window changes (or on first open):
    // dataDir_ names what the rows came from, so no session callback is needed.
    const std::string want =
        session_.ProductLoaded() ? session_.Product().DataDir() : std::string();
    if (want != dataDir_) LoadDataRows();

    // ---- Toolbar ----
    if (ImGui::Button("Reload")) LoadDataRows();
    ImGui::SameLine();
    ImGui::BeginDisabled(dataDir_.empty());
    if (ImGui::Button("Open folder")) OpenFolderInExplorer(dataDir_);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Open the product data folder in Explorer");
    ImGui::EndDisabled();
    ImGui::SameLine();
    // The selected part, in full: the DUT Records page shows every entry of
    // one record, which is what a cell that looks wrong asks for next.
    const bool haveSel = !dataSelSerial_.empty() && CanAccess(Page::DutView);
    ImGui::BeginDisabled(!haveSel);
    const bool openDetails = ImGui::Button("Details");
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Open the selected serial in DUT Records (or double-click a row)");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputTextWithHint("##datafilter", "filter serial", dataFilter_, sizeof(dataFilter_));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(130.0f);
    ImGui::Combo("##datatime", &dataTimeSel_, kTimeSels, 4);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("By the record's opened_at stamp (UTC)");

    // The row filter: serial substring AND the time preset. One lambda so the
    // table, the chart and the count can never disagree about who is shown.
    const std::string filt = dataFilter_;
    std::string       datePrefix;
    switch (dataTimeSel_) {
        case 1:  datePrefix = MonthPrefix(0);  break;
        case 2:  datePrefix = MonthPrefix(-1); break;
        case 3:  datePrefix = YearPrefix();    break;
        default: break;
    }
    auto passes = [&](const DutDataRow& r) {
        if (!filt.empty() && r.serial.find(filt) == std::string::npos) return false;
        if (!datePrefix.empty() && r.opened.compare(0, datePrefix.size(), datePrefix) != 0)
            return false;
        return true;
    };
    // Indices into dataRows_ in sorted order; rebuilt after a header re-sort.
    std::vector<int> shown;
    auto rebuildShown = [&]() {
        shown.clear();
        for (int i = 0; i < static_cast<int>(dataRows_.size()); ++i)
            if (passes(dataRows_[static_cast<std::size_t>(i)])) shown.push_back(i);
    };
    rebuildShown();

    ImGui::SameLine();
    if (dataDir_.empty())
        ImGui::TextDisabled("Load a product first.");
    else
        ImGui::TextDisabled("%zu of %zu records, %zu keys - %s", shown.size(), dataRows_.size(),
                            dataKeys_.size(), dataDir_.c_str());
    ImGui::Separator();

    bool wantDetails = openDetails;

    // ---- Table ----
    const float chartH = 300.0f;   // controls row + plot + label strip
    const float tableH = std::max(120.0f, ImGui::GetContentRegionAvail().y - chartH -
                                              ImGui::GetStyle().ItemSpacing.y);
    const int keyCols = std::min(static_cast<int>(dataKeys_.size()), kMaxKeyCols);
    const ImGuiTableFlags tflags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollX |
        ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable | ImGuiTableFlags_Sortable |
        ImGuiTableFlags_Hideable | ImGuiTableFlags_SizingFixedFit;
    if (dataRows_.empty()) {
        ImGui::BeginChild("dataEmpty", ImVec2(0.0f, tableH), ImGuiChildFlags_Borders);
        if (dataDir_.empty())
            ImGui::TextDisabled("No product loaded.");
        else
            ImGui::TextWrapped("No DUT records under %s.", dataDir_.c_str());
        ImGui::EndChild();
    } else if (ImGui::BeginTable("##datatable", kFixedCols + keyCols, tflags,
                                 ImVec2(0.0f, tableH))) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("Serial", ImGuiTableColumnFlags_NoHide |
                                              ImGuiTableColumnFlags_DefaultSort, 80.0f);
        ImGui::TableSetupColumn("Opened (UTC)", ImGuiTableColumnFlags_None, 170.0f);
        ImGui::TableSetupColumn("Closed (UTC)", ImGuiTableColumnFlags_None, 170.0f);
        ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_None, 60.0f);
        ImGui::TableSetupColumn("Runs", ImGuiTableColumnFlags_None, 50.0f);
        for (int k = 0; k < keyCols; ++k)
            ImGui::TableSetupColumn(dataKeys_[static_cast<std::size_t>(k)].c_str());
        ImGui::TableHeadersRow();

        if (ImGuiTableSortSpecs* ss = ImGui::TableGetSortSpecs()) {
            if (ss->SpecsDirty && ss->SpecsCount > 0) {
                dataSortCol_ = ss->Specs[0].ColumnIndex;
                dataSortAsc_ = ss->Specs[0].SortDirection != ImGuiSortDirection_Descending;
                SortDataRows();
                ss->SpecsDirty = false;
                rebuildShown();   // the order changed under it
            }
        }

        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(shown.size()));
        while (clipper.Step()) {
            for (int s = clipper.DisplayStart; s < clipper.DisplayEnd; ++s) {
                const DutDataRow& r = dataRows_[static_cast<std::size_t>(shown[static_cast<std::size_t>(s)])];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(kColSerial);
                ImGui::PushID(s);
                if (ImGui::Selectable(r.serial.c_str(), dataSelSerial_ == r.serial,
                                      ImGuiSelectableFlags_SpanAllColumns |
                                          ImGuiSelectableFlags_AllowDoubleClick)) {
                    dataSelSerial_ = r.serial;
                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) wantDetails = true;
                }
                ImGui::PopID();
                if (ImGui::TableSetColumnIndex(kColOpened))
                    ImGui::TextUnformatted(r.opened.c_str());
                if (ImGui::TableSetColumnIndex(kColClosed))
                    ImGui::TextUnformatted(r.closed.empty() ? "(open or interrupted)"
                                                            : r.closed.c_str());
                if (ImGui::TableSetColumnIndex(kColStatus)) {
                    if (r.status == "pass")
                        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f), "PASS");
                    else if (r.status == "fail")
                        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "FAIL");
                    else
                        ImGui::TextDisabled("-");
                }
                if (ImGui::TableSetColumnIndex(kColRuns)) {
                    ImGui::Text("%zu", r.runs.size());
                    if (!r.runs.empty() && ImGui::IsItemHovered()) {
                        ImGui::BeginTooltip();
                        for (const std::string& line : r.runs) ImGui::BulletText("%s", line.c_str());
                        ImGui::EndTooltip();
                    }
                }
                for (int k = 0; k < keyCols; ++k) {
                    if (!ImGui::TableSetColumnIndex(kFixedCols + k)) continue;
                    const auto it = r.vals.find(dataKeys_[static_cast<std::size_t>(k)]);
                    if (it == r.vals.end()) { ImGui::TextDisabled("-"); continue; }
                    std::string shownText = CellText(it->second);
                    const bool  cut = shownText.size() > 40;
                    if (cut) shownText = shownText.substr(0, 37) + "...";
                    ImGui::TextUnformatted(shownText.c_str());
                    if (cut && ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", it->second.dump(2).c_str());
                }
            }
        }
        ImGui::EndTable();
    }
    if (static_cast<int>(dataKeys_.size()) > kMaxKeyCols)
        ImGui::TextColored(StateColor(ConnState::Failed),
                           "Only the first %d of %zu keys are shown.", kMaxKeyCols,
                           dataKeys_.size());

    // ---- Chart controls: two series (B optional), the plot mode ----
    auto keyCombo = [&](const char* id, int* sel, bool allowNone) {
        const char* label = *sel >= 0 && *sel < static_cast<int>(dataKeys_.size())
                                ? dataKeys_[static_cast<std::size_t>(*sel)].c_str()
                                : (allowNone ? "(none)" : "(no key)");
        ImGui::SetNextItemWidth(180.0f);
        if (ImGui::BeginCombo(id, label)) {
            if (allowNone && ImGui::Selectable("(none)", *sel < 0)) *sel = -1;
            for (int k = 0; k < static_cast<int>(dataKeys_.size()); ++k) {
                const bool s = *sel == k;
                if (ImGui::Selectable(dataKeys_[static_cast<std::size_t>(k)].c_str(), s)) *sel = k;
                if (s) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    };
    ImGui::TextColored(kColA, "A");
    ImGui::SameLine();
    keyCombo("##seriesA", &dataKeySel_, false);
    ImGui::SameLine();
    ImGui::TextColored(kColB, "B");
    ImGui::SameLine();
    keyCombo("##seriesB", &dataKeySel2_, true);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(130.0f);
    ImGui::Combo("##plotmode", &dataPlotMode_, kPlotModes, 4);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Series: one point per part in table order.\n"
                          "Box: min / Q1 / median / Q3 / max per day, week (from Monday) or month.");

    const bool haveA = dataKeySel_  >= 0 && dataKeySel_  < static_cast<int>(dataKeys_.size());
    const bool haveB = dataKeySel2_ >= 0 && dataKeySel2_ < static_cast<int>(dataKeys_.size());
    const std::string keyA = haveA ? dataKeys_[static_cast<std::size_t>(dataKeySel_)]  : std::string();
    const std::string keyB = haveB ? dataKeys_[static_cast<std::size_t>(dataKeySel2_)] : std::string();
    const char* yLabel = haveA && haveB ? "value" : (haveA ? keyA.c_str() : nullptr);
    auto numAt = [](const DutDataRow& r, const std::string& key, double* out) {
        const auto it = r.vals.find(key);
        if (it == r.vals.end() || !it->second.is_number()) return false;
        *out = it->second.get<double>();
        return true;
    };

    // The X labels are drawn by hand under the plot (vertical, thinned), so
    // the axis itself shows tick marks only. Pixel columns are captured inside
    // the plot, where the transform is defined, and consumed after EndPlot.
    const float stripH = 72.0f;
    const float plotH  = std::max(80.0f, ImGui::GetContentRegionAvail().y - stripH -
                                             ImGui::GetStyle().ItemSpacing.y);
    std::vector<float>       px;
    std::vector<const char*> labels;   // series: into dataRows_; box: into `names`
    std::vector<std::string> names;    // box bucket names — must outlive the strip draw
    float plotL = 0.0f, plotR = 0.0f;

    ImPlotSpec specA, specB;
    specA.LineColor = kColA; specA.Marker = ImPlotMarker_Circle; specA.LineWeight = 2.0f;
    specB.LineColor = kColB; specB.Marker = ImPlotMarker_Square; specB.LineWeight = 2.0f;

    if (dataPlotMode_ == 0) {
        // ---- Series: one point per shown part, in table order ----
        std::vector<double> xa, ya, xb, yb;
        double selX = -1.0;
        for (std::size_t s = 0; s < shown.size(); ++s) {
            const DutDataRow& r = dataRows_[static_cast<std::size_t>(shown[s])];
            double v = 0.0;
            if (haveA && numAt(r, keyA, &v)) { xa.push_back(static_cast<double>(s)); ya.push_back(v); }
            if (haveB && numAt(r, keyB, &v)) { xb.push_back(static_cast<double>(s)); yb.push_back(v); }
            if (r.serial == dataSelSerial_) selX = static_cast<double>(s);
            labels.push_back(r.serial.c_str());
        }
        if (ImPlot::BeginPlot("##viewdata", ImVec2(-1.0f, plotH), ImPlotFlags_NoMenus)) {
            ImPlot::SetupLegend(ImPlotLocation_NorthEast);
            ImPlot::SetupAxes(nullptr, yLabel,
                              ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoTickLabels,
                              ImPlotAxisFlags_AutoFit);
            if (!xa.empty())
                ImPlot::PlotLine(keyA.c_str(), xa.data(), ya.data(), static_cast<int>(xa.size()), specA);
            if (!xb.empty())
                ImPlot::PlotLine(keyB.c_str(), xb.data(), yb.data(), static_cast<int>(xb.size()), specB);
            if (selX >= 0.0) {
                ImPlotSpec sel;
                sel.LineColor = ImVec4(1.0f, 0.85f, 0.3f, 0.8f);
                ImPlot::PlotInfLines("##sel", &selX, 1, sel);
            }
            if (xa.empty() && xb.empty() && haveA)
                ImPlot::PlotText("no numeric values for this key", 0.5, 0.5);
            // Which part is under the mouse: the strip thins its labels, and
            // the hover answers for every count.
            if (ImPlot::IsPlotHovered()) {
                const int s = static_cast<int>(ImPlot::GetPlotMousePos().x + 0.5);
                if (s >= 0 && s < static_cast<int>(shown.size())) {
                    const DutDataRow& r =
                        dataRows_[static_cast<std::size_t>(shown[static_cast<std::size_t>(s)])];
                    ImGui::BeginTooltip();
                    ImGui::Text("%s  (%s)", r.serial.c_str(), r.opened.c_str());
                    double v = 0.0;
                    if (haveA) ImGui::TextColored(kColA, "%s = %s", keyA.c_str(),
                                                  numAt(r, keyA, &v) ? CellText(v).c_str() : "-");
                    if (haveB) ImGui::TextColored(kColB, "%s = %s", keyB.c_str(),
                                                  numAt(r, keyB, &v) ? CellText(v).c_str() : "-");
                    ImGui::EndTooltip();
                }
            }
            for (std::size_t s = 0; s < shown.size(); ++s)
                px.push_back(ImPlot::PlotToPixels(static_cast<double>(s), 0.0).x);
            plotL = ImPlot::GetPlotPos().x;
            plotR = plotL + ImPlot::GetPlotSize().x;
            ImPlot::EndPlot();
        }
    } else {
        // ---- Box plots: the shown parts bucketed by opened_at ----
        // Bucket keys are ISO-date prefixes, so the map's text order is the
        // calendar order. A bucket exists if EITHER series has a value in it.
        std::map<std::string, std::pair<std::vector<double>, std::vector<double>>> buckets;
        for (const int i : shown) {
            const DutDataRow& r = dataRows_[static_cast<std::size_t>(i)];
            const std::string key = BucketKey(r.opened, dataPlotMode_);
            if (key.empty()) continue;
            double v = 0.0;
            if (haveA && numAt(r, keyA, &v)) buckets[key].first.push_back(v);
            if (haveB && numAt(r, keyB, &v)) buckets[key].second.push_back(v);
        }
        std::vector<BoxStats>    statA, statB;
        double lo = 0.0, hi = 0.0;
        bool   any = false;
        for (auto& [name, pair] : buckets) {
            names.push_back(name);
            statA.push_back(Summarise(pair.first));
            statB.push_back(Summarise(pair.second));
            for (const BoxStats* s : {&statA.back(), &statB.back()}) {
                if (s->n == 0) continue;
                if (!any) { lo = s->lo; hi = s->hi; any = true; }
                lo = std::min(lo, s->lo);
                hi = std::max(hi, s->hi);
            }
        }
        for (const std::string& n : names) labels.push_back(n.c_str());
        const int    nb  = static_cast<int>(names.size());
        const double pad = (hi > lo ? hi - lo : std::max(std::fabs(hi), 1e-12)) * 0.08;

        if (ImPlot::BeginPlot("##viewdata", ImVec2(-1.0f, plotH), ImPlotFlags_NoMenus)) {
            ImPlot::SetupLegend(ImPlotLocation_NorthEast);
            ImPlot::SetupAxes(nullptr, yLabel,
                              ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoGridLines,
                              ImPlotAxisFlags_None);
            // Nothing is plotted through ImPlot's own items, so nothing
            // autofits: the limits are set from the summaries every frame.
            ImPlot::SetupAxisLimits(ImAxis_X1, -0.6, std::max(nb, 1) - 0.4, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, any ? lo - pad : 0.0, any ? hi + pad : 1.0,
                                    ImPlotCond_Always);
            if (haveA) ImPlot::PlotDummy(keyA.c_str(), specA);
            if (haveB) ImPlot::PlotDummy(keyB.c_str(), specB);

            ImDrawList* dl = ImPlot::GetPlotDrawList();
            ImPlot::PushPlotClipRect();
            auto drawBox = [&](const BoxStats& s, double cx, double hw, const ImVec4& col) {
                if (s.n == 0) return;
                const ImU32 line = ImGui::ColorConvertFloat4ToU32(col);
                const ImU32 fill = ImGui::ColorConvertFloat4ToU32(ImVec4(col.x, col.y, col.z, 0.35f));
                if (s.n == 1) {   // one part: a point, not a box of zero height
                    dl->AddCircleFilled(ImPlot::PlotToPixels(cx, s.med), 4.0f, line);
                    return;
                }
                const ImVec2 tl = ImPlot::PlotToPixels(cx - hw, s.q3);
                const ImVec2 br = ImPlot::PlotToPixels(cx + hw, s.q1);
                dl->AddRectFilled(tl, br, fill);
                dl->AddRect(tl, br, line, 0.0f, 0, 1.5f);
                const ImVec2 m0 = ImPlot::PlotToPixels(cx - hw, s.med), m1 = ImPlot::PlotToPixels(cx + hw, s.med);
                dl->AddLine(m0, m1, line, 2.5f);
                const ImVec2 top = ImPlot::PlotToPixels(cx, s.hi), bot = ImPlot::PlotToPixels(cx, s.lo);
                dl->AddLine(ImVec2(tl.x + (br.x - tl.x) * 0.5f, tl.y), top, line, 1.5f);
                dl->AddLine(ImVec2(tl.x + (br.x - tl.x) * 0.5f, br.y), bot, line, 1.5f);
                const float cap = (br.x - tl.x) * 0.25f;
                dl->AddLine(ImVec2(top.x - cap, top.y), ImVec2(top.x + cap, top.y), line, 1.5f);
                dl->AddLine(ImVec2(bot.x - cap, bot.y), ImVec2(bot.x + cap, bot.y), line, 1.5f);
            };
            const bool   both = haveA && haveB;
            const double hw   = both ? 0.17 : 0.30;
            for (int i = 0; i < nb; ++i) {
                if (haveA) drawBox(statA[static_cast<std::size_t>(i)], both ? i - 0.2 : i, hw, kColA);
                if (haveB) drawBox(statB[static_cast<std::size_t>(i)], both ? i + 0.2 : i, hw, kColB);
            }
            ImPlot::PopPlotClipRect();
            if (!any && haveA)
                ImPlot::PlotText("no numeric values in the shown records", 0.0, 0.5);

            if (ImPlot::IsPlotHovered()) {
                const int i = static_cast<int>(std::floor(ImPlot::GetPlotMousePos().x + 0.5));
                if (i >= 0 && i < nb) {
                    auto line = [&](const char* key, const BoxStats& s, const ImVec4& col) {
                        if (s.n == 0) { ImGui::TextColored(col, "%s: no values", key); return; }
                        ImGui::TextColored(col, "%s  n=%d", key, s.n);
                        ImGui::Text("  min %s   Q1 %s   med %s   Q3 %s   max %s",
                                    CellText(s.lo).c_str(), CellText(s.q1).c_str(),
                                    CellText(s.med).c_str(), CellText(s.q3).c_str(),
                                    CellText(s.hi).c_str());
                    };
                    ImGui::BeginTooltip();
                    ImGui::TextUnformatted(names[static_cast<std::size_t>(i)].c_str());
                    if (haveA) line(keyA.c_str(), statA[static_cast<std::size_t>(i)], kColA);
                    if (haveB) line(keyB.c_str(), statB[static_cast<std::size_t>(i)], kColB);
                    ImGui::EndTooltip();
                }
            }
            for (int i = 0; i < nb; ++i)
                px.push_back(ImPlot::PlotToPixels(static_cast<double>(i), 0.0).x);
            plotL = ImPlot::GetPlotPos().x;
            plotR = plotL + ImPlot::GetPlotSize().x;
            ImPlot::EndPlot();
        }
    }
    DrawLabelStrip(px, labels, plotL, plotR, stripH);

    ImGui::End();

    // Outside the window: SwitchPage may refuse, and the page's own list must
    // be the one the DUT Records page reads — a second scan there keeps the
    // two surfaces from ever disagreeing about which serials exist.
    if (wantDetails && haveSel) {
        LoadDutList();
        for (int i = 0; i < static_cast<int>(dutList_.size()); ++i)
            if (dutList_[static_cast<std::size_t>(i)] == dataSelSerial_) dutSel_ = i;
        SwitchPage(Page::DutView);
    }
}

} // namespace firedog
