// WndPreferences.cpp : the Preferences window - program-wide settings in the
// Visual Studio "Options" / Office shape: a category tree on the left, the
// selected page on the right, OK / Cancel / Apply along the bottom.
// Definitions of App members - see App.h.
//
// Opened from the Overview's Hardware header. Everything here is a STATION
// preference persisted in UiSettings - except Scanning, which edits the XCAN
// launch presets, targets and the quick-align band's slots, machine state in
// config/scan_presets.local.json (the same file the Run page's right-click
// modals write). Display and Charts only change how this bench draws, hence
// kRolePrefs (Operator); Motion holds the defaults the Jog window's Tool -> cam
// row starts from, so that page edits only at kRoleJog; Scanning edits only at
// kRoleScanPrefs.
//
// The dialog edits a working copy (prefsEdit_). Apply writes back only what
// differs from prefsBase_ (the values when it opened or last applied), so a
// field nobody touched here is never rewritten from a stale copy. NOT a modal:
// a run may be blocked on the always-on-top prompt window and a modal would
// lock it out.
#include "App.h"

#include "imgui.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "ui/Widgets.h"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace firedog {

namespace {
// The category tree. A group is a non-selectable parent; a page is a leaf with
// the id prefsCat_ holds. Add a page: a row here and a case in the switch.
enum PrefsPage { kPrefDisplay = 0, kPrefChartCurrent = 1, kPrefMotionRegMove = 2,
                 kPrefToolMarkers = 3, kPrefXcanPresets = 4, kPrefXcanTargets = 5,
                 kPrefQuickAlign = 6 };

struct PrefsGroup {
    const char* name;
    struct Leaf { const char* name; int id; const char* title; };
    std::vector<Leaf> leaves;
};

const std::vector<PrefsGroup>& PrefsTree()
{
    static const std::vector<PrefsGroup> tree = {
        {"Environment", {{"Display", kPrefDisplay, "Environment > Display"}}},
        {"Charts",      {{"Current (SMU)", kPrefChartCurrent, "Charts > Current (SMU)"}}},
        {"Motion",      {{"Tool -> cam", kPrefMotionRegMove, "Motion > Tool -> cam"}}},
        {"Graphic view", {{"Tool markers", kPrefToolMarkers, "Graphic view > Tool markers"}}},
        {"Scanning",    {{"XCAN presets", kPrefXcanPresets, "Scanning > XCAN presets"},
                         {"XCAN targets", kPrefXcanTargets, "Scanning > XCAN targets"},
                         {"Quick align", kPrefQuickAlign, "Scanning > Quick align"}}},
    };
    return tree;
}

const char* PrefsTitle(int id)
{
    for (const PrefsGroup& g : PrefsTree())
        for (const PrefsGroup::Leaf& l : g.leaves)
            if (l.id == id) return l.title;
    return "";
}

// Why a band list cannot be applied, or "" when it can.
std::string BandListError(const std::vector<ChartYBand>& bands)
{
    for (std::size_t i = 0; i < bands.size(); ++i) {
        if (!(bands[i].hiUa > bands[i].loUa)) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Range %zu: 'to' must be above 'from'.", i + 1);
            return buf;
        }
    }
    return {};
}

// A comma list of positive numbers ("10,5,2,1"), or false - what
// hexapod_align.lua's ladder parser would choke on.
bool ParseUmList(const std::string& s, std::vector<double>* out)
{
    std::stringstream ss(s);
    std::string tok;
    std::vector<double> v;
    while (std::getline(ss, tok, ',')) {
        const auto b = tok.find_first_not_of(" \t");
        if (b == std::string::npos) return false;
        const auto e = tok.find_last_not_of(" \t");
        tok = tok.substr(b, e - b + 1);
        char* end = nullptr;
        const double d = std::strtod(tok.c_str(), &end);
        if (end == tok.c_str() || *end != '\0' || !(d > 0.0)) return false;
        v.push_back(d);
    }
    if (v.empty()) return false;
    if (out) *out = v;
    return true;
}

// Why the XCAN pages cannot be applied, or "". Only ENABLED slots are judged:
// a spare switched off may hold anything until somebody turns it on.
std::string XcanPrefsError(const AppPrefs& e)
{
    char buf[160];
    for (int i = 0; i < kXcanPresetCount; ++i) {
        const XcanPreset& p = e.xcanPresets[static_cast<std::size_t>(i)];
        if (!p.enabled) continue;
        if (!ParseUmList(p.stepsUm, nullptr)) {
            std::snprintf(buf, sizeof(buf), "XCAN preset %d: steps must be positive numbers "
                          "separated by commas.", i + 1);
            return buf;
        }
        if (p.axes.find_first_not_of(" ,") == std::string::npos) {
            std::snprintf(buf, sizeof(buf), "XCAN preset %d: no axes.", i + 1);
            return buf;
        }
    }
    for (int i = 0; i < kXcanPresetCount; ++i) {
        const XcanTarget& t = e.xcanTargets[static_cast<std::size_t>(i)];
        if (!t.enabled) continue;
        if (!(t.targetUa > 0.0)) {
            std::snprintf(buf, sizeof(buf), "XCAN target %d: target must be above 0.", i + 1);
            return buf;
        }
        if (!(t.zoneLowUa > 0.0) || !(t.zoneHighUa > t.zoneLowUa)) {
            std::snprintf(buf, sizeof(buf), "XCAN target %d: zones need 0 < low < high.", i + 1);
            return buf;
        }
    }
    return {};
}

// One labelled row of a form: the label, a tooltip on it, then the cursor at
// the field column.
void FormLabel(const char* label, float labelW, const char* tip)
{
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
    ImGui::SameLine(labelW);
}
} // namespace

AppPrefs App::CurrentPrefs() const
{
    AppPrefs p;
    p.themeIndex     = themeIndex_;
    p.fontIndex      = fontIndex_;
    p.fontSize       = uiFontSize_;
    p.chartYBands    = chartYBands_;
    p.chartYBandList = chartYBandList_;
    p.chartChDefault = chartChDef_;
    p.auxTipLabels   = auxTipLabels_;
    p.jogRegZOff     = jogRegZOffDef_;
    p.jogRegUnver    = jogRegUnverDef_;
    for (int i = 0; i < kXcanPresetCount; ++i) {
        p.xcanPresets[static_cast<std::size_t>(i)] = xcanPresets_[i];
        p.xcanTargets[static_cast<std::size_t>(i)] = xcanTargets_[i];
    }
    p.quickAlign = quickAlign_;
    return p;
}

void App::OpenPreferences()
{
    // Re-seed only on open: a window already open keeps its pending edits.
    if (!showPrefs_) {
        prefsBase_ = CurrentPrefs();
        prefsEdit_ = prefsBase_;
        prefsMsg_.clear();
    }
    showPrefs_ = true;
    ImGui::SetWindowFocus("Preferences");
}

void App::ApplyPreferences()
{
    AppPrefs&       e = prefsEdit_;
    const AppPrefs& b = prefsBase_;

    if (e.themeIndex != b.themeIndex) {
        themeIndex_ = e.themeIndex;
        Theme::Apply(themeIndex_);
    }
    if (e.fontIndex != b.fontIndex || e.fontSize != b.fontSize) {
        fontIndex_  = e.fontIndex;
        uiFontSize_ = e.fontSize;
        host_.SetFont(kFonts[fontIndex_].path, uiFontSize_);
    }
    if (e.chartYBands != b.chartYBands) chartYBands_ = e.chartYBands;
    if (e.chartYBandList != b.chartYBandList) {
        // Sorted by where they start, so "the band holding a value" is the
        // lowest one and the list reads in order next time.
        std::stable_sort(e.chartYBandList.begin(), e.chartYBandList.end(),
                         [](const ChartYBand& x, const ChartYBand& y) { return x.loUa < y.loUa; });
        chartYBandList_ = e.chartYBandList;
        for (ChartBandState& s : chartBandSt_) s = ChartBandState{};   // indices changed
    }
    // A changed default also resets the Jog window's live field now - the
    // operator applied it to see it, not to wait for a restart.
    if (e.auxTipLabels != b.auxTipLabels) auxTipLabels_ = e.auxTipLabels;
    if (e.chartChDefault != b.chartChDefault) {
        chartChDef_ = e.chartChDefault;
        chartCh_    = chartChDef_;   // show it now, as the Jog defaults below do
    }
    if (e.jogRegZOff != b.jogRegZOff) {
        jogRegZOffDef_ = std::clamp(e.jogRegZOff, -100.0, 100.0);
        jogRegZOff_    = jogRegZOffDef_;
    }
    if (e.jogRegUnver != b.jogRegUnver) {
        jogRegUnverDef_ = e.jogRegUnver;
        jogRegUnver_    = jogRegUnverDef_;
    }
    // Scanning: slot by slot, so a slot edited meanwhile in a Run-page modal
    // and left alone here is not overwritten from this window's older copy.
    // The next launch reads them; a scan already running keeps what it got.
    bool scanChanged = false;
    for (int i = 0; i < kXcanPresetCount; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        if (!(e.xcanPresets[k] == b.xcanPresets[k])) {
            xcanPresets_[i] = e.xcanPresets[k];
            scanChanged = true;
        }
        if (!(e.xcanTargets[k] == b.xcanTargets[k])) {
            xcanTargets_[i] = e.xcanTargets[k];
            scanChanged = true;
        }
    }
    if (!(e.quickAlign == b.quickAlign)) {
        quickAlign_ = e.quickAlign;
        scanChanged = true;
    }
    if (scanChanged) SaveScanPresets();
    SaveUiSettings();
    prefsBase_ = CurrentPrefs();
    prefsEdit_ = prefsBase_;
}

void App::DrawPreferencesWindow()
{
    // The open bit outlives a sign-out; a role never should.
    if (!CanUse(kRolePrefs)) { showPrefs_ = false; return; }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(820.0f, 520.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Preferences", &showPrefs_, ImGuiWindowFlags_NoCollapse)) {
        ImGui::End();
        return;
    }

    const ImGuiStyle& st     = ImGui::GetStyle();
    const float       footH  = ImGui::GetFrameHeight() + st.ItemSpacing.y * 2.0f + 2.0f;
    const float       treeW  = (std::max)(200.0f, ImGui::CalcTextSize("Current (SMU)").x * 1.8f);
    const float       bodyH  = (std::max)(60.0f, ImGui::GetContentRegionAvail().y - footH);

    // ---- Category tree (left) ----
    ImGui::BeginChild("##prefsTree", ImVec2(treeW, bodyH), ImGuiChildFlags_Borders);
    for (const PrefsGroup& g : PrefsTree()) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNodeEx(g.name, ImGuiTreeNodeFlags_SpanAvailWidth)) {
            for (const PrefsGroup::Leaf& l : g.leaves) {
                const ImGuiTreeNodeFlags f = ImGuiTreeNodeFlags_Leaf |
                                             ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                             ImGuiTreeNodeFlags_SpanAvailWidth |
                                             (prefsCat_ == l.id ? ImGuiTreeNodeFlags_Selected : 0);
                ImGui::TreeNodeEx(l.name, f);
                if (ImGui::IsItemClicked()) prefsCat_ = l.id;
            }
            ImGui::TreePop();
        }
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Selected page (right) ----
    ImGui::BeginChild("##prefsPage", ImVec2(0.0f, bodyH), ImGuiChildFlags_Borders);
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.2f);
    ImGui::TextUnformatted(PrefsTitle(prefsCat_));
    ImGui::PopFont();
    ImGui::Separator();
    ImGui::Spacing();

    AppPrefs& e = prefsEdit_;
    switch (prefsCat_) {
    case kPrefDisplay: {
        const float labelW = ImGui::CalcTextSize("Font size").x + st.ItemSpacing.x * 3.0f;
        const float fieldW = 320.0f;

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Theme");
        ImGui::SameLine(labelW);
        ImGui::SetNextItemWidth(fieldW);
        if (ImGui::BeginCombo("##prefTheme", Theme::Name(e.themeIndex))) {
            for (int i = 0; i < Theme::Count(); ++i)
                if (ImGui::Selectable(Theme::Name(i), e.themeIndex == i)) e.themeIndex = i;
            ImGui::EndCombo();
        }

        const char* names[kFontCount];
        for (int i = 0; i < kFontCount; ++i) names[i] = kFonts[i].label;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Font");
        ImGui::SameLine(labelW);
        ImGui::SetNextItemWidth(fieldW);
        ImGui::Combo("##prefFont", &e.fontIndex, names, kFontCount);

        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Font size");
        ImGui::SameLine(labelW);
        ImGui::SetNextItemWidth(fieldW);
        ImGui::SliderFloat("##prefFontSize", &e.fontSize, 10.0f, 28.0f, "%.0f px");

        ImGui::Spacing();
        ImGui::TextDisabled("Applied with OK or Apply.");
        break;
    }
    case kPrefChartCurrent: {
        // The channel(s) shown when the app starts. The chart's own Ch1 /
        // Ch2 / Both buttons still switch it for the session.
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Show at startup");
        ImGui::SameLine();
        ImGui::RadioButton("Ch1##prefCh", &e.chartChDefault, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Ch2##prefCh", &e.chartChDefault, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Both##prefCh", &e.chartChDefault, 2);
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::Checkbox("Fixed Y ranges", &e.chartYBands);
        ImGui::TextWrapped("Auto-fit re-scales the Y axis on every new sample, so a flat but "
                           "noisy current makes the chart jump. With fixed ranges the axis "
                           "runs from the range holding the lowest visible value to the "
                           "range holding the highest, and stays put while the signal "
                           "does. A value inside no range falls back to auto-fit.");
        ImGui::Spacing();

        ImGui::BeginDisabled(!e.chartYBands);
        std::vector<ChartYBand>& bl = e.chartYBandList;
        int drop = -1;
        const ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                   ImGuiTableFlags_SizingFixedFit;
        if (ImGui::BeginTable("##prefBands", 4, tf)) {
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("From (uA)");
            ImGui::TableSetupColumn("To (uA)");
            ImGui::TableSetupColumn("Spans", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (int i = 0; i < static_cast<int>(bl.size()); ++i) {
                ChartYBand& b = bl[static_cast<std::size_t>(i)];
                ImGui::PushID(i);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%d", i + 1);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(130.0f);
                ImGui::InputDouble("##lo", &b.loUa, 0.0, 0.0, "%.4g");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(130.0f);
                ImGui::InputDouble("##hi", &b.hiUa, 0.0, 0.0, "%.4g");
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                if (b.hiUa > b.loUa)
                    ImGui::Text("%s .. %s", FormatCurrent(b.loUa * 1e-6).c_str(),
                                FormatCurrent(b.hiUa * 1e-6).c_str());
                else
                    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "to <= from");
                ImGui::SameLine();
                ImGui::BeginDisabled(bl.size() <= 1);
                if (ImGui::SmallButton("Remove")) drop = i;
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (drop >= 0) bl.erase(bl.begin() + drop);

        ImGui::BeginDisabled(static_cast<int>(bl.size()) >= kMaxChartYBands);
        if (ImGui::Button("+ Add range")) {
            // Continue upward a decade from the last one.
            const double from = bl.empty() ? 0.0 : bl.back().hiUa;
            bl.push_back({from, from > 0.0 ? from * 10.0 : 10.0});
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Reset to defaults")) bl = DefaultChartYBands();
        ImGui::EndDisabled();

        ImGui::Spacing();
        ImGui::TextDisabled("Applies to the Keithley page chart and the Aux window chart\n"
                            "(compact: per channel). Display only - recipes and scans\n"
                            "read the raw stream.");
        break;
    }
    case kPrefToolMarkers: {
        ImGui::TextWrapped("The text drawn beside each registered tool's tip on the Aux "
                           "view, and beside the triangle on the picture's edge that points "
                           "to a tip out of view. Up to %d letters; empty = the mark alone. "
                           "The colour is the device's own (its device picture).",
                           kAuxTipLabelMax);
        ImGui::Spacing();

        // A row per registered tool, plus any device already holding a label
        // (a registration not loaded right now keeps its entry).
        std::vector<std::string> devs;
        for (const auto& kv : workflow_.DeviceRegs())
            if (kv.second.valid) devs.push_back(kv.first);
        for (const auto& kv : e.auxTipLabels)
            if (std::find(devs.begin(), devs.end(), kv.first) == devs.end())
                devs.push_back(kv.first);
        std::sort(devs.begin(), devs.end());

        if (devs.empty()) {
            ImGui::TextDisabled("No registered tools.");
        } else if (ImGui::BeginTable("##prefTipLbl", 3,
                                     ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                     ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("Device");
            ImGui::TableSetupColumn("Label");
            ImGui::TableSetupColumn("Shown as", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (const std::string& d : devs) {
                ImGui::PushID(d.c_str());
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(d.c_str());

                ImGui::TableNextColumn();
                // An absent entry shows (and would draw) the full name; typing
                // creates the entry, clearing it to "" leaves the mark alone.
                const auto it  = e.auxTipLabels.find(d);
                const std::string cur = it != e.auxTipLabels.end() ? it->second : d;
                char buf[kAuxTipLabelMax + 1] = {};
                std::snprintf(buf, sizeof(buf), "%s", cur.c_str());
                ImGui::SetNextItemWidth(ImGui::CalcTextSize("MMMMMMMMMM").x +
                                        st.FramePadding.x * 2.0f);
                if (ImGui::InputText("##lbl", buf, sizeof(buf)))
                    e.auxTipLabels[d] = buf;

                ImGui::TableNextColumn();
                // A preview of the edge marker: the triangle in the device's
                // colour, pointing left, then the label.
                const ImU32  col = DeviceAccentColor(d, IM_COL32(255, 190, 70, 255));
                const float  h   = ImGui::GetFrameHeight();
                const ImVec2 p   = ImGui::GetCursorScreenPos();
                ImDrawList*  dl  = ImGui::GetWindowDrawList();
                const ImVec2 tip(p.x + 2.0f, p.y + h * 0.5f);
                const ImVec2 b1(p.x + h * 0.8f, p.y + h * 0.15f);
                const ImVec2 b2(p.x + h * 0.8f, p.y + h * 0.85f);
                dl->AddTriangleFilled(tip, b1, b2, col);
                dl->AddTriangle(tip, b1, b2, IM_COL32(0, 0, 0, 200), 1.5f);
                ImGui::Dummy(ImVec2(h, h));
                ImGui::SameLine();
                ImGui::AlignTextToFramePadding();
                const std::string shown = e.auxTipLabels.count(d) ? e.auxTipLabels[d] : d;
                if (shown.empty()) ImGui::TextDisabled("(mark only)");
                else ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%s", shown.c_str());
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        if (ImGui::Button("Reset to defaults##tipLbl")) e.auxTipLabels = DefaultAuxTipLabels();
        ImGui::SameLine();
        ImGui::TextDisabled("HL = hex-left, HR = hex-right");
        break;
    }
    case kPrefMotionRegMove: {
        ImGui::TextWrapped("What the Jog window's Tool -> cam row (a registered tool "
                           "sent to the point the camera looks at) starts with. Its "
                           "fields return to these at every start; an edit there "
                           "lasts until the app closes.");
        ImGui::Spacing();
        // These decide where a one-click move ends up, so they edit at the
        // level that may jog at all - the page is still readable below it.
        const bool canEdit = CanUse(kRoleJog);
        ImGui::BeginDisabled(!canEdit);
        const float labelW = ImGui::CalcTextSize("Allow unverified").x + st.ItemSpacing.x * 3.0f;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Z offset");
        ImGui::SameLine(labelW);
        ImGui::SetNextItemWidth(130.0f);
        ImGui::InputDouble("mm##prefRegZ", &e.jogRegZOff, 0.0, 0.0, "%.3f");
        e.jogRegZOff = std::clamp(e.jogRegZOff, -100.0, 100.0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("Allow unverified");
        ImGui::SameLine(labelW);
        ImGui::Checkbox("##prefRegUnver", &e.jogRegUnver);
        ImGui::EndDisabled();
        if (!canEdit) ImGui::TextDisabled("Technician and up may change these.");
        ImGui::Spacing();
        if (e.jogRegZOff == 0.0)
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                               "Z offset 0 = the tip goes ONTO the focused surface (a touch).");
        ImGui::TextDisabled("Offset from the camera's focused Z; NEGATIVE stands the tool\n"
                            "off the surface on this machine. Cam -> tool is XY only.");
        break;
    }
    case kPrefXcanPresets: {
        ImGui::TextWrapped("The XCAN launch presets: the step ladder and every "
                           "hexapod_align.lua tunable a launch hands the script. They are "
                           "the Run page's XCAN preset buttons and the ladders the quick-align "
                           "band presses. A running scan keeps what it started with.");
        ImGui::Spacing();
        const bool canEdit = CanUse(kRoleScanPrefs);
        if (ImGui::BeginTabBar("##xcanPresetTabs")) {
            for (int i = 0; i < kXcanPresetCount; ++i) {
                XcanPreset& p = e.xcanPresets[static_cast<std::size_t>(i)];
                char tab[64];
                std::snprintf(tab, sizeof(tab), "%d %s%s###xp%d", i + 1, p.label.c_str(),
                              p.enabled ? "" : " (off)", i);
                if (!ImGui::BeginTabItem(tab)) continue;
                ImGui::PushID(i);
                ImGui::BeginDisabled(!canEdit);
                const float labelW = ImGui::CalcTextSize("Consecutive limit").x +
                                     st.ItemSpacing.x * 4.0f;
                const float w = 160.0f;

                ImGui::SeparatorText("Button");
                FormLabel("Show button", labelW, "Off: the preset is a hidden spare.");
                ImGui::Checkbox("##en", &p.enabled);
                FormLabel("Label", labelW, nullptr);
                char lb[32];
                std::snprintf(lb, sizeof(lb), "%s", p.label.c_str());
                ImGui::SetNextItemWidth(w);
                if (ImGui::InputText("##label", lb, sizeof(lb))) p.label = lb;

                ImGui::SeparatorText("Ladder");
                FormLabel("Steps (um)", labelW,
                          "align_steps_um - coarse -> fine, comma separated. Each pass\n"
                          "walks every axis at one step size. With a target's adaptive\n"
                          "zones the same list is the zone steps (coarsest for the\n"
                          "lowest signal) and passes coarser than the zone are skipped.");
                char sb[96];
                std::snprintf(sb, sizeof(sb), "%s", p.stepsUm.c_str());
                ImGui::SetNextItemWidth(w);
                if (ImGui::InputText("##steps", sb, sizeof(sb))) p.stepsUm = sb;
                ImGui::SameLine();
                std::vector<double> ladder;
                if (ParseUmList(p.stepsUm, &ladder))
                    ImGui::TextDisabled("%zu passes", ladder.size());
                else
                    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "not a number list");
                FormLabel("Axes", labelW, "align_axes - coordinate-descent order, e.g. Z,Y,X.");
                char ab[48];
                std::snprintf(ab, sizeof(ab), "%s", p.axes.c_str());
                ImGui::SetNextItemWidth(w);
                if (ImGui::InputText("##axes", ab, sizeof(ab))) p.axes = ab;

                ImGui::SeparatorText("Measure");
                FormLabel("Settle (ms)", labelW,
                          "align_settle_ms - with fresh samples: the CEILING on the wait\n"
                          "for them; with fresh samples 0: a blind delay.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##settle", &p.settleMs, 0.0, 0.0, "%g");
                FormLabel("Settle floor (ms)", labelW,
                          "align_settle_min_ms - never measure sooner than this after a\n"
                          "move (slow receivers: photodiode / TIA). 0 = none.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##settlemin", &p.settleMinMs, 0.0, 0.0, "%g");
                FormLabel("Fresh samples", labelW,
                          "align_fresh_samples - NEW publishes on the feedback topic to\n"
                          "wait for per measure. 2 = the first fully post-move sample.\n"
                          "0 = the original blind settle sleep.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputInt("##fresh", &p.freshSamples);
                FormLabel("Average count", labelW,
                          "align_avg_count - samples averaged for the baseline.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputInt("##avg", &p.avgCount);

                ImGui::SeparatorText("Convergence");
                FormLabel("Consecutive limit", labelW,
                          "align_consec_limit - stop a direction after this many\n"
                          "non-improving steps.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputInt("##consec", &p.consecLimit);
                FormLabel("Signal drop (%)", labelW,
                          "align_sig_drop_pct - a relative drop this big ends the\n"
                          "direction at once. Noise dips reach ~6% here; true\n"
                          "post-peak drops are 50%+.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##drop", &p.sigDropPct, 0.0, 0.0, "%g");
                FormLabel("Improve (%)", labelW,
                          "align_improve_pct - the margin over which the scan goes back\n"
                          "to the global peak when the current signal fell off it.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##improve", &p.improvePct, 0.0, 0.0, "%g");
                FormLabel("Min signal (A)", labelW,
                          "align_min_signal - abort when |baseline| is below this,\n"
                          "feedback units (amps). 0 = never.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##minsig", &p.minSignal, 0.0, 0.0, "%g");

                ImGui::SeparatorText("Motion");
                FormLabel("Max travel (um)", labelW,
                          "align_max_travel_um - travel cap per direction.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##travel", &p.maxTravelUm, 0.0, 0.0, "%g");
                FormLabel("Velocity (mm/s)", labelW,
                          "align_velocity_mm_s - set once before the scan. 0 = keep.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##vel", &p.velocityMmS, 0.0, 0.0, "%g");
                FormLabel("Arrival tol (mm)", labelW, "align_tol_mm - move arrival tolerance.");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##tol", &p.tolMm, 0.0, 0.0, "%g");
                FormLabel("Move timeout (ms)", labelW, "align_move_timeout_ms");
                ImGui::SetNextItemWidth(w);
                ImGui::InputDouble("##mto", &p.moveTimeoutMs, 0.0, 0.0, "%g");

                ImGui::SeparatorText("Cycle-time trims");
                ImGui::Checkbox("Skip opposite", &p.skipOpposite);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("align_skip_opposite - a first direction that walked\n"
                                      "uphill already brackets the peak; skip the other.");
                ImGui::SameLine();
                ImGui::Checkbox("Direction memory", &p.dirMemory);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("align_dir_memory - each axis starts with the\n"
                                      "direction that last improved.");
                ImGui::SameLine();
                ImGui::Checkbox("Reuse sample", &p.reuseSample);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("align_reuse_sample - a point just measured is not\n"
                                      "measured again.");

                // Values the script would only meet as nonsense.
                p.freshSamples = std::clamp(p.freshSamples, 0, 20);
                p.avgCount     = std::clamp(p.avgCount, 1, 100);
                p.consecLimit  = std::clamp(p.consecLimit, 1, 100);
                p.settleMs     = (std::max)(p.settleMs, 0.0);
                p.settleMinMs  = (std::max)(p.settleMinMs, 0.0);

                ImGui::Spacing();
                if (ImGui::Button("Reset this preset to defaults")) p = DefaultXcanPreset(i);
                ImGui::EndDisabled();
                ImGui::PopID();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        if (!canEdit) ImGui::TextDisabled("Technician and up may change these.");
        break;
    }
    case kPrefXcanTargets: {
        ImGui::TextWrapped("The XCAN target slots - one per product power class. The "
                           "target ends the scan the moment the feedback reaches it; the "
                           "adaptive zones split [low, high] into the ladder's sections. "
                           "The ACTIVE slot (yellow on the Run page) goes into every XCAN "
                           "and Climb launch. Values in uA.");
        ImGui::Spacing();
        const bool canEdit = CanUse(kRoleScanPrefs);
        ImGui::BeginDisabled(!canEdit);
        if (ImGui::BeginTable("##xcanTargets", 6,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("Slot");
            ImGui::TableSetupColumn("Show");
            ImGui::TableSetupColumn("Target (uA)");
            ImGui::TableSetupColumn("Zone low (uA)");
            ImGui::TableSetupColumn("Zone high (uA)");
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (int i = 0; i < kXcanPresetCount; ++i) {
                XcanTarget& t = e.xcanTargets[static_cast<std::size_t>(i)];
                ImGui::PushID(i);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%d", i + 1);
                ImGui::TableNextColumn();
                ImGui::Checkbox("##en", &t.enabled);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(110.0f);
                ImGui::InputDouble("##tgt", &t.targetUa, 0.0, 0.0, "%g");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(110.0f);
                ImGui::InputDouble("##lo", &t.zoneLowUa, 0.0, 0.0, "%g");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(110.0f);
                ImGui::InputDouble("##hi", &t.zoneHighUa, 0.0, 0.0, "%g");
                ImGui::TableNextColumn();
                if (ImGui::SmallButton("Defaults")) t = DefaultXcanTarget(i);
                ImGui::SameLine();
                if (t.zoneHighUa <= t.zoneLowUa)
                    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "high <= low");
                else if (t.targetUa > t.zoneHighUa)
                    ImGui::TextDisabled("target above the zones");
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::EndDisabled();
        if (!canEdit) ImGui::TextDisabled("Technician and up may change these.");
        ImGui::Spacing();
        ImGui::TextDisabled("Zone low should sit at the raster handoff level.\n"
                            "A slot switched off is no Target button, and a quick-align\n"
                            "button using it is disabled.");
        break;
    }
    case kPrefQuickAlign: {
        ImGui::TextWrapped("Which slots the Run page's quick-align band presses. The "
                           "first-light button (the tall Left Coarse) runs at the LOW target; "
                           "the four production buttons (Left / Right x Coarse / Fine) run "
                           "at the HIGH target.");
        ImGui::Spacing();
        const bool canEdit = CanUse(kRoleScanPrefs);
        ImGui::BeginDisabled(!canEdit);
        const float labelW = ImGui::CalcTextSize("First-light target").x + st.ItemSpacing.x * 4.0f;
        const auto targetCombo = [&](const char* label, const char* id, int& slot) {
            FormLabel(label, labelW, nullptr);
            const XcanTarget& cur = e.xcanTargets[static_cast<std::size_t>(slot)];
            char prev[64];
            std::snprintf(prev, sizeof(prev), "%d: %g uA%s", slot + 1, cur.targetUa,
                          cur.enabled ? "" : " (off)");
            ImGui::SetNextItemWidth(260.0f);
            if (ImGui::BeginCombo(id, prev)) {
                for (int i = 0; i < kXcanPresetCount; ++i) {
                    const XcanTarget& t = e.xcanTargets[static_cast<std::size_t>(i)];
                    char it[96];
                    std::snprintf(it, sizeof(it), "%d: %g uA  (zones %g-%g)%s", i + 1,
                                  t.targetUa, t.zoneLowUa, t.zoneHighUa,
                                  t.enabled ? "" : " (off)");
                    if (ImGui::Selectable(it, slot == i)) slot = i;
                }
                ImGui::EndCombo();
            }
        };
        const auto presetCombo = [&](const char* label, const char* id, int& slot) {
            FormLabel(label, labelW, nullptr);
            const XcanPreset& cur = e.xcanPresets[static_cast<std::size_t>(slot)];
            char prev[96];
            std::snprintf(prev, sizeof(prev), "%d: %s%s", slot + 1, cur.label.c_str(),
                          cur.enabled ? "" : " (off)");
            ImGui::SetNextItemWidth(260.0f);
            if (ImGui::BeginCombo(id, prev)) {
                for (int i = 0; i < kXcanPresetCount; ++i) {
                    const XcanPreset& p = e.xcanPresets[static_cast<std::size_t>(i)];
                    char it[160];
                    std::snprintf(it, sizeof(it), "%d: %s  (%s um)%s", i + 1, p.label.c_str(),
                                  p.stepsUm.c_str(), p.enabled ? "" : " (off)");
                    if (ImGui::Selectable(it, slot == i)) slot = i;
                }
                ImGui::EndCombo();
            }
        };
        QuickAlignSlots& q = e.quickAlign;
        ImGui::SeparatorText("Targets");
        targetCombo("First-light target", "##qlo", q.targetLo);
        targetCombo("Production target", "##qhi", q.targetHi);
        ImGui::SeparatorText("Ladders");
        presetCombo("Coarse buttons", "##qco", q.coarse);
        presetCombo("Fine buttons", "##qfi", q.fine);
        ImGui::Spacing();
        if (ImGui::Button("Reset to defaults##quick")) q = QuickAlignSlots{};
        ImGui::EndDisabled();
        if (!canEdit) ImGui::TextDisabled("Technician and up may change these.");
        ImGui::Spacing();
        ImGui::TextDisabled("Edit the slots themselves on the XCAN presets / XCAN targets\n"
                            "pages.");
        break;
    }
    default:
        prefsCat_ = kPrefDisplay;
        break;
    }
    ImGui::EndChild();

    // ---- OK / Cancel / Apply (bottom right) ----
    std::string err = BandListError(e.chartYBandList);
    if (err.empty()) err = XcanPrefsError(e);
    const bool        dirty = !(e.themeIndex == prefsBase_.themeIndex &&
                                e.fontIndex == prefsBase_.fontIndex &&
                                e.fontSize == prefsBase_.fontSize &&
                                e.chartYBands == prefsBase_.chartYBands &&
                                e.chartYBandList == prefsBase_.chartYBandList &&
                                e.chartChDefault == prefsBase_.chartChDefault &&
                                e.auxTipLabels == prefsBase_.auxTipLabels &&
                                e.jogRegZOff == prefsBase_.jogRegZOff &&
                                e.jogRegUnver == prefsBase_.jogRegUnver &&
                                e.xcanPresets == prefsBase_.xcanPresets &&
                                e.xcanTargets == prefsBase_.xcanTargets &&
                                e.quickAlign == prefsBase_.quickAlign);
    ImGui::Spacing();
    const float bw    = ImGui::CalcTextSize("Cancel").x + st.FramePadding.x * 2.0f + 24.0f;
    const float total = bw * 3.0f + st.ItemSpacing.x * 2.0f;
    const float btnX  = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - total;
    bool said = false;
    if (!err.empty()) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "%s", err.c_str());
        said = true;
    } else if (dirty) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("unapplied changes");
        said = true;
    }
    if (said) ImGui::SameLine(btnX);
    else      ImGui::SetCursorPosX(btnX);
    ImGui::BeginDisabled(!err.empty());
    if (ImGui::Button("OK", ImVec2(bw, 0.0f))) {
        if (dirty) ApplyPreferences();
        showPrefs_ = false;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(bw, 0.0f))) {
        prefsEdit_ = prefsBase_;
        showPrefs_ = false;
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!dirty || !err.empty());
    if (ImGui::Button("Apply", ImVec2(bw, 0.0f))) ApplyPreferences();
    ImGui::EndDisabled();

    ImGui::End();
}

} // namespace firedog
