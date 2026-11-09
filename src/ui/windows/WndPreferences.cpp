// WndPreferences.cpp : the Preferences window - program-wide settings in the
// Visual Studio "Options" / Office shape: a category tree on the left, the
// selected page on the right, OK / Cancel / Apply along the bottom.
// Definitions of App members - see App.h.
//
// Opened from the Overview's Hardware header. Everything here is a STATION
// preference persisted in UiSettings. Display and Charts only change how this
// bench draws, hence kRolePrefs (Operator); Motion holds the defaults the Jog
// window's Tool -> cam row starts from, so that page edits only at kRoleJog.
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
#include <string>
#include <vector>

namespace firedog {

namespace {
// The category tree. A group is a non-selectable parent; a page is a leaf with
// the id prefsCat_ holds. Add a page: a row here and a case in the switch.
enum PrefsPage { kPrefDisplay = 0, kPrefChartCurrent = 1, kPrefMotionRegMove = 2 };

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
    p.jogRegZOff     = jogRegZOffDef_;
    p.jogRegUnver    = jogRegUnverDef_;
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
    default:
        prefsCat_ = kPrefDisplay;
        break;
    }
    ImGui::EndChild();

    // ---- OK / Cancel / Apply (bottom right) ----
    const std::string err   = BandListError(e.chartYBandList);
    const bool        dirty = !(e.themeIndex == prefsBase_.themeIndex &&
                                e.fontIndex == prefsBase_.fontIndex &&
                                e.fontSize == prefsBase_.fontSize &&
                                e.chartYBands == prefsBase_.chartYBands &&
                                e.chartYBandList == prefsBase_.chartYBandList &&
                                e.chartChDefault == prefsBase_.chartChDefault &&
                                e.jogRegZOff == prefsBase_.jogRegZOff &&
                                e.jogRegUnver == prefsBase_.jogRegUnver);
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
