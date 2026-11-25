// WndInspection.cpp : the floating Inspection View (View menu; it also opens
// itself when a collection starts). Definitions of App members - see App.h.
//
// The COLLECTOR's face. Every inspect_object verdict since the last Clear
// (core/Inspection.h, App::insp_, filled by App::Update from
// Workflow::TakeInspections) as a tree: one row per component (the step's
// ctx_prefix) with its runs, yield and last verdict, opening onto one row per
// metric - last value, mean, min, max, standard deviation, range, n and Cpk
// against the step's tolerance. Under it, the selected component's samples.
//
// Nothing here judges, moves or changes a step: the verdicts were the step's.
// What it writes is what the operator asks for - Save (two CSVs under a typed
// name into the product's data folder) or Copy CSV (the clipboard). Clear is
// the "cancel": the collection is memory only and lives until then.
// docs/inspection-view-design.md.
#include "App.h"

#include "imgui.h"

#include "core/Inspection.h"
#include "core/Paths.h"     // DataPath
#include "ui/Widgets.h"     // OpenFolderInExplorer

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kInspGood(0.40f, 0.85f, 0.45f, 1.0f);
const ImVec4 kInspBad(0.95f, 0.40f, 0.35f, 1.0f);
const ImVec4 kInspWarn(0.98f, 0.72f, 0.32f, 1.0f);

// Cpk colour: the usual thresholds - capable (>= 1.33), marginal (>= 1.0), not.
ImVec4 CpkColor(double cpk)
{
    if (cpk >= 1.33) return kInspGood;
    if (cpk >= 1.0)  return kInspWarn;
    return kInspBad;
}

bool WriteTextFile(const std::filesystem::path& p, const std::string& text, std::string* err)
{
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f) { *err = "cannot write " + p.string(); return false; }
    f << text;
    if (!f) { *err = "write failed: " + p.string(); return false; }
    return true;
}

} // namespace

std::string App::InspectionDir() const
{
    if (session_.ProductLoaded() && !session_.Product().DataDir().empty())
        return (std::filesystem::path(session_.Product().DataDir()) / "inspection").string();
    return DataPath("inspection");
}

void App::DrawInspectionWindow()
{
    if (!CanUse(kRoleInspection)) { showInspection_ = false; return; }

    ImGui::SetNextWindowSize(ImVec2(980.0f, 640.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Inspection View", &showInspection_)) {
        ImGui::End();
        return;
    }

    // ---- Toolbar: what is held; Clear; Save; Copy; Open folder -----------------
    const std::vector<std::string>& comps = insp_.Components();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%d result(s), %d component(s)", insp_.Total(), static_cast<int>(comps.size()));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Every inspect_object verdict since the last Clear - memory only.\n"
                          "Save writes it to the data folder; Clear drops it.");
    ImGui::SameLine();

    // Clear is the cancel - two clicks within 3 s, no modal (a modal would
    // block a prompt window a run may be waiting on).
    const double now   = ImGui::GetTime();
    const bool   armed = inspClearArmAt_ >= 0.0 && now - inspClearArmAt_ < 3.0;
    ImGui::BeginDisabled(insp_.Empty());
    if (armed) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button(armed ? "Click again to clear" : "Clear")) {
        if (armed) {
            FD_INFO("inspect", "inspection view cleared (%d result(s) dropped)", insp_.Total());
            insp_.Clear();
            inspSel_.clear();
            inspMsg_.clear();
            inspClearArmAt_ = -1.0;
        } else {
            inspClearArmAt_ = now;
        }
    }
    if (armed) ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Drop every collected result (not saved ones). The next\n"
                          "inspect_object result starts a new collection.");

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(200.0f);
    ImGui::InputTextWithHint("##inspName", "data name", inspName_, sizeof(inspName_));
    const std::string fname = InspectionFileName(inspName_);
    ImGui::SameLine();
    ImGui::BeginDisabled(insp_.Empty() || fname.empty());
    if (ImGui::Button("Save")) {
        namespace fs = std::filesystem;
        const fs::path dir   = InspectionDir();
        const fs::path rows  = dir / (fname + ".csv");
        const fs::path summ  = dir / (fname + "_summary.csv");
        std::error_code ec;
        std::string err;
        fs::create_directories(dir, ec);
        if (ec) {
            err = "cannot create " + dir.string() + ": " + ec.message();
        } else if (fs::exists(rows, ec) || fs::exists(summ, ec)) {
            // Never over an earlier save: a name is a record of one collection.
            err = "'" + fname + "' already exists in " + dir.string() + " - choose another name";
        } else if (WriteTextFile(rows, insp_.SamplesCsv(), &err) &&
                   WriteTextFile(summ, insp_.SummaryCsv(), &err)) {
            inspMsg_     = "saved " + rows.filename().string() + " + " +
                           summ.filename().string() + " to " + dir.string();
            inspMsgBad_  = false;
            inspLastDir_ = dir.string();
            FD_INFO("inspect", "inspection view saved: %s (%d result(s))", rows.string().c_str(),
                    insp_.Total());
        }
        if (!err.empty()) {
            inspMsg_    = err;
            inspMsgBad_ = true;
            FD_WARN("inspect", "inspection save: %s", err.c_str());
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Write <name>.csv (every result) and <name>_summary.csv\n"
                          "(statistics per component) to\n%s\nAn existing name is refused.",
                          InspectionDir().c_str());
    ImGui::SameLine();
    ImGui::BeginDisabled(insp_.Empty());
    if (ImGui::Button("Copy CSV")) {
        ImGui::SetClipboardText(insp_.SamplesCsv().c_str());
        inspMsg_    = "every result copied to the clipboard as CSV (paste into Excel)";
        inspMsgBad_ = false;
    }
    ImGui::EndDisabled();
    if (CanUse(kRoleData)) {   // leaves the application - the Explorer shortcut's level
        ImGui::SameLine();
        if (ImGui::Button("Open folder")) {
            std::error_code ec;
            std::filesystem::create_directories(InspectionDir(), ec);
            OpenFolderInExplorer(InspectionDir());
        }
    }
    if (!inspMsg_.empty())
        ImGui::TextColored(inspMsgBad_ ? kInspBad : kInspGood, "%s", inspMsg_.c_str());

    if (insp_.Empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("No results yet - every inspect_object step reports here when it runs.");
        ImGui::End();
        return;
    }

    // ---- The tree: component rows opening onto metric rows ---------------------
    const float treeH = ImGui::GetContentRegionAvail().y * 0.58f;
    const ImGuiTableFlags tf = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH |
                               ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable |
                               ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("inspTree", 9, tf, ImVec2(0.0f, treeH))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthStretch, 2.2f);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Mean", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Min", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Max", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Std dev", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Range", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("n", ImGuiTableColumnFlags_WidthFixed, 44.0f);
        ImGui::TableSetupColumn("Cpk", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();

        for (const std::string& name : comps) {
            const InspectionCollector::Counts c = insp_.CountsFor(name);
            const std::vector<InspectionSample>& ss = insp_.Samples(name);
            const InspectionSample& last = ss.back();

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGuiTreeNodeFlags nf = ImGuiTreeNodeFlags_SpanAvailWidth |
                                    ImGuiTreeNodeFlags_DefaultOpen |
                                    ImGuiTreeNodeFlags_OpenOnArrow;
            if (name == inspSel_) nf |= ImGuiTreeNodeFlags_Selected;
            const bool open = ImGui::TreeNodeEx(name.c_str(), nf, "%s", name.c_str());
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) inspSel_ = name;
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("datum '%s'  -  %d run(s), %d found, %d judged, %d passed\n"
                                  "Click to list its results below.",
                                  last.datum.c_str(), c.runs, c.found, c.judged, c.passed);
            ImGui::TableSetColumnIndex(1);
            if (!last.judged && last.found)
                ImGui::TextColored(kInspWarn, "no tol");
            else
                ImGui::TextColored(last.pass ? kInspGood : kInspBad, "%s",
                                   last.pass ? "PASS" : "FAIL");
            if (ImGui::IsItemHovered() && !last.why.empty())
                ImGui::SetTooltip("last: %s", last.why.c_str());
            ImGui::TableSetColumnIndex(2);
            if (c.runs > 0) {
                const double y = 100.0 * c.passed / c.runs;
                ImGui::TextColored(c.passed == c.runs ? kInspGood : kInspWarn, "yield %.1f%%", y);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%d of %d run(s) passed (a run with no tolerance set, or\n"
                                      "with nothing found, does not pass)",
                                      c.passed, c.runs);
            }
            ImGui::TableSetColumnIndex(3);
            if (c.found < c.runs)
                ImGui::TextColored(kInspBad, "%d not found", c.runs - c.found);
            ImGui::TableSetColumnIndex(7);
            ImGui::Text("%d", c.runs);

            if (!open) continue;
            for (const InspMetric m : InspMetrics()) {
                const MetricStats st = insp_.StatsFor(name, m);
                if (st.n == 0) continue;   // P not reported, or nothing found
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TreeNodeEx(InspMetricName(m),
                                  ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                      ImGuiTreeNodeFlags_SpanAvailWidth,
                                  "%s", InspMetricName(m));
                if (st.tol > 0.0 && ImGui::IsItemHovered())
                    ImGui::SetTooltip("tolerance %s%.4f mm (the latest result's)",
                                      st.oneSided ? "<= " : "+/-", st.tol);
                const bool score = m == InspMetric::Score;
                const char* fmt  = score ? "%.3f" : "%+.4f";
                ImGui::TableSetColumnIndex(1);
                // The last value against its own tolerance, when it has one.
                if (st.tol > 0.0) {
                    const bool in = st.oneSided ? st.last <= st.tol : std::fabs(st.last) <= st.tol;
                    ImGui::TextColored(in ? kInspGood : kInspBad, fmt, st.last);
                } else {
                    ImGui::Text(fmt, st.last);
                }
                ImGui::TableSetColumnIndex(2);
                ImGui::Text(fmt, st.mean);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text(fmt, st.min);
                ImGui::TableSetColumnIndex(4);
                ImGui::Text(fmt, st.max);
                ImGui::TableSetColumnIndex(5);
                if (st.n >= 2) ImGui::Text(score ? "%.3f" : "%.4f", st.stdev);
                else           ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(6);
                ImGui::Text(score ? "%.3f" : "%.4f", st.max - st.min);
                ImGui::TableSetColumnIndex(7);
                ImGui::Text("%d", st.n);
                ImGui::TableSetColumnIndex(8);
                if (st.hasCpk) {
                    ImGui::TextColored(CpkColor(st.cpk), "%.2f", st.cpk);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s against %s%.4f mm, sample std dev, %d result(s)\n"
                                          ">= 1.33 capable, >= 1.00 marginal",
                                          st.oneSided ? "Cpu (one-sided)" : "Cpk",
                                          st.oneSided ? "<= " : "+/-", st.tol, st.n);
                } else if (st.tol > 0.0) {
                    ImGui::TextDisabled("-");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("needs 2 or more results with some spread");
                }
            }
            ImGui::TreePop();
        }
        ImGui::EndTable();
    }

    // ---- The selected component's results, newest first ------------------------
    if (inspSel_.empty() && !comps.empty()) inspSel_ = comps.front();
    const std::vector<InspectionSample>& sel = insp_.Samples(inspSel_);
    ImGui::SeparatorText(("Results: " + inspSel_).c_str());
    const ImGuiTableFlags rf = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH |
                               ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable |
                               ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("inspRows", 9, rf, ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("time", ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableSetupColumn("serial", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("x", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("y", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("dX", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("dY", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("dR", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("verdict", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("why", ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableHeadersRow();
        ImGuiListClipper clip;
        clip.Begin(static_cast<int>(sel.size()));
        while (clip.Step()) {
            for (int r = clip.DisplayStart; r < clip.DisplayEnd; ++r) {
                const InspectionSample& s = sel[sel.size() - 1 - static_cast<std::size_t>(r)];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(s.at.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(s.serial.empty() ? "-" : s.serial.c_str());
                if (s.found) {
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%+.4f", s.lx);
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%+.4f", s.ly);
                    ImGui::TableSetColumnIndex(4);
                    ImGui::Text("%+.4f", s.dlx);
                    ImGui::TableSetColumnIndex(5);
                    ImGui::Text("%+.4f", s.dly);
                    ImGui::TableSetColumnIndex(6);
                    ImGui::Text("%.4f", s.dr);
                }
                ImGui::TableSetColumnIndex(7);
                if (s.found && !s.judged) ImGui::TextColored(kInspWarn, "no tol");
                else ImGui::TextColored(s.pass ? kInspGood : kInspBad, "%s",
                                        s.pass ? "PASS" : "FAIL");
                ImGui::TableSetColumnIndex(8);
                ImGui::TextUnformatted(s.why.c_str());
            }
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

} // namespace firedog
