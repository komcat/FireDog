// PorTool.cpp : see PorTool.h.
#include "PorTool.h"
#include "PorExport.h"

#include "command/CommandRegistry.h"
#include "core/Log.h"

#include "imgui.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <utility>

namespace firedog {

namespace {

ImVec4 StatusColor(PorStatus s)
{
    switch (s) {
    case PorStatus::Same:       return ImGui::GetStyleColorVec4(ImGuiCol_Text);
    case PorStatus::Measured:   return ImVec4(0.45f, 0.75f, 0.95f, 1.0f);
    case PorStatus::Unknown:    return ImVec4(0.75f, 0.60f, 0.95f, 1.0f);
    case PorStatus::Changed:    return ImVec4(1.00f, 0.82f, 0.30f, 1.0f);
    case PorStatus::OnlyTarget: return ImVec4(0.55f, 0.85f, 0.55f, 1.0f);
    case PorStatus::OnlyMaster: return ImVec4(1.00f, 0.62f, 0.25f, 1.0f);
    case PorStatus::Ambiguous:  return ImVec4(0.95f, 0.45f, 0.85f, 1.0f);
    case PorStatus::Missing:    return ImVec4(1.00f, 0.40f, 0.30f, 1.0f);
    }
    return ImGui::GetStyleColorVec4(ImGuiCol_Text);
}

// A faint band behind a row, in the row's own colour.
ImU32 Band(PorStatus s)
{
    ImVec4 c = StatusColor(s);
    c.w = 0.12f;
    return ImGui::GetColorU32(c);
}

bool Worse(PorStatus a, PorStatus b) { return static_cast<int>(a) > static_cast<int>(b); }

PorStatus WorstOf(const std::vector<PorRow>& rows)
{
    PorStatus w = PorStatus::Same;
    for (const PorRow& r : rows) if (Worse(r.worst, w)) w = r.worst;
    return w;
}

void CopyTo(char (&buf)[512], const std::string& s)
{
    std::snprintf(buf, sizeof(buf), "%s", s.c_str());
}

std::string AxesText(const std::map<std::string, double>& v)
{
    std::string t;
    char b[48];
    for (const auto& [ax, val] : v) {
        std::snprintf(b, sizeof(b), "%s%s %.3f", t.empty() ? "" : "  ", ax.c_str(), val);
        t += b;
    }
    return t;
}

const char* TierName(PorPosTier t)
{
    switch (t) {
    case PorPosTier::Deck:       return "Required by the deck (first-use order)";
    case PorPosTier::Private:    return "Private poses of the alignments and grips the deck uses";
    case PorPosTier::Unused:     return "Master positions the deck does not use";
    case PorPosTier::TargetOnly: return "Only on the target";
    }
    return "";
}

std::string RowLabel(const std::string& text, bool off, bool used, PorRowKind kind)
{
    if (text.empty()) return "\xE2\x80\x94";   // an em dash: absent on this side
    std::string l;
    if (kind == PorRowKind::Folder) l += "[+] ";
    l += text;
    if (off)  l += "   [off]";
    if (used) l += "   [deck]";
    return l;
}

void StatusCell(PorStatus own, PorStatus worst, const std::string& note)
{
    if (own != PorStatus::Same)
        ImGui::TextColored(StatusColor(own), "%s", PorStatusName(own));
    else if (worst != PorStatus::Same)
        ImGui::TextDisabled("below: %s", PorStatusName(worst));
    if (!note.empty()) {
        if (own != PorStatus::Same || worst != PorStatus::Same) ImGui::SameLine();
        ImGui::TextDisabled("%s", note.c_str());
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", note.c_str());
    }
}

} // namespace

PorTool::PorTool(const CommandRegistry& reg, RootsPreview roots, std::string localDir)
    : reg_(reg), roots_(std::move(roots)), localDir_(std::move(localDir))
{
    CopyTo(targetRoot_, roots_.root);
}

// ---- Settings (the tool's own, under %LOCALAPPDATA%) ------------------------------

void PorTool::LoadSettings()
{
    std::ifstream f(localDir_ + "/por_settings.json");
    if (!f.is_open()) return;
    try {
        nlohmann::json j;
        f >> j;
        CopyTo(masterRoot_, j.value("master_root", std::string()));
        CopyTo(targetRoot_, j.value("target_root", std::string(targetRoot_)));
        CopyTo(exportDir_, j.value("export_dir", std::string()));
        product_      = j.value("product", std::string());
        mmThreshold_  = j.value("mm_threshold", mmThreshold_);
        degThreshold_ = j.value("deg_threshold", degThreshold_);
        diffOnly_     = j.value("diff_only", diffOnly_);
        allParams_    = j.value("all_params", allParams_);
        workingOnly_  = j.value("working_only", workingOnly_);
        explorerW_    = j.value("explorer_w", explorerW_);
        if (j.contains("window")) {
            const nlohmann::json& w = j["window"];
            placement_.x         = w.value("x", 0);
            placement_.y         = w.value("y", 0);
            placement_.w         = w.value("w", 0);
            placement_.h         = w.value("h", 0);
            placement_.maximized = w.value("maximized", false);
        }
    } catch (const std::exception& e) {
        FD_WARN("por", "settings unreadable (%s) - defaults", e.what());
    }
    if (masterRoot_[0]) RefreshProducts();
}

void PorTool::SaveSettings() const
{
    nlohmann::json j;
    j["master_root"]   = masterRoot_;
    j["target_root"]   = targetRoot_;
    j["export_dir"]    = exportDir_;
    j["product"]       = product_;
    j["mm_threshold"]  = mmThreshold_;
    j["deg_threshold"] = degThreshold_;
    j["diff_only"]     = diffOnly_;
    j["all_params"]    = allParams_;
    j["working_only"]  = workingOnly_;
    j["explorer_w"]    = explorerW_;
    j["window"] = {{"x", placement_.x}, {"y", placement_.y}, {"w", placement_.w},
                   {"h", placement_.h}, {"maximized", placement_.maximized}};
    std::ofstream f(localDir_ + "/por_settings.json");
    if (f.is_open()) f << j.dump(2) << '\n';
}

// ---- Actions -------------------------------------------------------------------------

void PorTool::RefreshProducts()
{
    products_ = ListPorProducts(masterRoot_);
    if (!products_.empty() &&
        std::find(products_.begin(), products_.end(), product_) == products_.end())
        product_ = products_.front();
}

void PorTool::Compare()
{
    masterOk_  = LoadPorSide(masterRoot_, product_, &master_);
    targetOk_  = LoadPorSide(targetRoot_, product_, &target_);
    report_    = ComparePor(master_, target_, reg_, roots_.exeDir + "/config/scripts");
    positions_ = ComparePositions(master_, target_, reg_, mmThreshold_, degThreshold_);
    hardware_  = CompareHardware(master_, target_, reg_);
    compared_  = true;
    view_      = View::None;
    sel_ = sel2_ = -1;

    int recipesDiffer = 0;
    for (const PorRow& r : report_.recipes) recipesDiffer += r.worst != PorStatus::Same;
    int need = 0, missing = 0;
    for (const PorPosRow& p : positions_.rows) {
        if (p.tier != PorPosTier::Deck && p.tier != PorPosTier::Private) continue;
        ++need;
        missing += p.status == PorStatus::Missing;
    }
    char b[512];
    std::snprintf(b, sizeof(b),
                  "%s%s   deck: %s   recipes: %zu, %d differ   positions the deck needs: %d, %d missing"
                  "   hardware: %s",
                  masterOk_ ? "" : "MASTER HAS NO SUCH PRODUCT.  ",
                  targetOk_ ? "" : "TARGET HAS NO SUCH PRODUCT.  ",
                  PorStatusName(WorstOf(report_.deck)), report_.recipes.size(), recipesDiffer,
                  need, missing, PorStatusName(WorstOf(hardware_.sections)));
    summary_ = b;
    FD_INFO("por", "compared '%s': %s vs %s", product_.c_str(), masterRoot_, targetRoot_);
    SaveSettings();
}

void PorTool::ComparePositionsOnly()
{
    if (compared_) positions_ = ComparePositions(master_, target_, reg_, mmThreshold_, degThreshold_);
}

int PorTool::FindRecipe(const std::string& name) const
{
    for (std::size_t i = 0; i < report_.recipes.size(); ++i)
        if (report_.recipes[i].recipe == name) return static_cast<int>(i);
    return -1;
}

bool PorTool::Visible(const PorRow& r) const
{
    return !diffOnly_ || r.worst != PorStatus::Same;
}

int PorTool::VisibleParams(const PorRow& r) const
{
    if (allParams_) return static_cast<int>(r.params.size());
    int n = 0;
    for (const PorParam& p : r.params) n += p.status != PorStatus::Same || !p.note.empty();
    return n;
}

// ---- Frame -----------------------------------------------------------------------------

void PorTool::Draw()
{
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::SetNextWindowViewport(vp->ID);
    const ImGuiWindowFlags wf = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                ImGuiWindowFlags_NoSavedSettings |
                                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoDocking;
    if (ImGui::Begin("##por", nullptr, wf)) {
        DrawTopBar();
        ImGui::Separator();

        const float h = ImGui::GetContentRegionAvail().y;
        ImGui::BeginChild("explorer", ImVec2(explorerW_, h), ImGuiChildFlags_Borders);
        DrawExplorer();
        ImGui::EndChild();

        ImGui::SameLine(0.0f, 0.0f);
        ImGui::InvisibleButton("##split", ImVec2(6.0f, h));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive())
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        if (ImGui::IsItemActive())
            explorerW_ = std::clamp(explorerW_ + ImGui::GetIO().MouseDelta.x, 180.0f, 900.0f);

        ImGui::SameLine(0.0f, 0.0f);
        ImGui::BeginChild("content", ImVec2(0.0f, h), ImGuiChildFlags_Borders);
        DrawContent();
        ImGui::EndChild();
    }
    ImGui::End();

    if (showExport_) DrawExportWindow();
    if (jumpRecipe_ >= 0) {
        view_       = View::Recipe;
        sel_        = jumpRecipe_;
        jumpRecipe_ = -1;
    }
}

void PorTool::DrawTopBar()
{
    ImGui::SetNextItemWidth(460.0f);
    ImGui::InputText("Master root", masterRoot_, sizeof(masterRoot_));
    if (ImGui::IsItemDeactivatedAfterEdit()) RefreshProducts();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A folder laid out like a machine root: an Export master folder,\n"
                          "or another machine's root over the network (\\\\M1\\D$\\ProgramData\\FireDog).");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(240.0f);
    if (ImGui::BeginCombo("Product", product_.empty() ? "(none)" : product_.c_str())) {
        for (const std::string& p : products_)
            if (ImGui::Selectable(p.c_str(), p == product_)) product_ = p;
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Refresh")) RefreshProducts();
    ImGui::SameLine();
    if (ImGui::Button("Export master...")) showExport_ = true;

    ImGui::SetNextItemWidth(460.0f);
    ImGui::InputText("Target root", targetRoot_, sizeof(targetRoot_));
    ImGui::SameLine();
    if (ImGui::Button("This machine")) CopyTo(targetRoot_, roots_.root);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s\n%s", roots_.root.c_str(),
                          roots_.iniFound ? ("from " + roots_.ini).c_str()
                                          : "no FireDog.ini beside this exe - the default root");

    ImGui::BeginDisabled(product_.empty() || !masterRoot_[0] || !targetRoot_[0]);
    if (ImGui::Button("  Compare  ")) Compare();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::Checkbox("Differences only", &diffOnly_);
    ImGui::SameLine();
    ImGui::Checkbox("All params", &allParams_);
    ImGui::SameLine();
    ImGui::TextDisabled("|  position flag over");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    ImGui::InputFloat("mm", &mmThreshold_, 0.0f, 0.0f, "%.3f");
    if (ImGui::IsItemDeactivatedAfterEdit()) ComparePositionsOnly();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80.0f);
    ImGui::InputFloat("deg", &degThreshold_, 0.0f, 0.0f, "%.3f");
    if (ImGui::IsItemDeactivatedAfterEdit()) ComparePositionsOnly();

    if (compared_) ImGui::TextUnformatted(summary_.c_str());
    else           ImGui::TextDisabled("Read-only: nothing on either side is written.");
}

// One explorer entry with its badge. True when the node is open (non-leaf).
bool PorTool::ExplorerItem(const char* label, bool selected, bool leaf, PorStatus badge,
                           bool defaultOpen)
{
    ImGuiTreeNodeFlags f = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
                           ImGuiTreeNodeFlags_SpanAvailWidth;
    if (selected)    f |= ImGuiTreeNodeFlags_Selected;
    if (leaf)        f |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (defaultOpen) f |= ImGuiTreeNodeFlags_DefaultOpen;
    if (badge != PorStatus::Same) ImGui::PushStyleColor(ImGuiCol_Text, StatusColor(badge));
    const bool open = ImGui::TreeNodeEx(label, f);
    if (badge != PorStatus::Same) ImGui::PopStyleColor();
    if (badge != PorStatus::Same && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", PorStatusName(badge));
    return open && !leaf;
}

void PorTool::DrawExplorer()
{
    if (!compared_) {
        ImGui::TextWrapped("Set the master root and product and the target root, then Compare.");
        return;
    }
    const auto clicked = [] { return ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen(); };

    if (ExplorerItem("Deck", false, false, WorstOf(report_.deck), true)) {
        for (std::size_t i = 0; i < report_.deck.size(); ++i) {
            const PorRow& p = report_.deck[i];
            ImGui::PushID(static_cast<int>(i));
            const std::string label = p.master.empty() ? p.target : p.master;
            const bool open = ExplorerItem(label.c_str(), view_ == View::Program && sel_ == (int)i,
                                           p.children.empty(), p.worst, true);
            if (clicked()) { view_ = View::Program; sel_ = static_cast<int>(i); }
            if (open) {
                for (std::size_t j = 0; j < p.children.size(); ++j) {
                    const PorRow& s = p.children[j];
                    ImGui::PushID(static_cast<int>(j));
                    const std::string sl = s.master.empty() ? s.target : s.master;
                    ExplorerItem(sl.c_str(), view_ == View::Slot && sel_ == (int)i && sel2_ == (int)j,
                                 true, s.worst);
                    if (clicked()) { view_ = View::Slot; sel_ = static_cast<int>(i); sel2_ = static_cast<int>(j); }
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    const std::string recipesLabel = "Recipes (" + std::to_string(report_.recipes.size()) + ")";
    if (ExplorerItem(recipesLabel.c_str(), false, false, WorstOf(report_.recipes), true)) {
        ImGui::Checkbox("deck recipes only", &workingOnly_);
        for (std::size_t i = 0; i < report_.recipes.size(); ++i) {
            const PorRow& r = report_.recipes[i];
            if (workingOnly_ && !r.working) continue;
            if (diffOnly_ && r.worst == PorStatus::Same) continue;
            ImGui::PushID(static_cast<int>(i) + 100000);
            ExplorerItem(r.recipe.c_str(), view_ == View::Recipe && sel_ == (int)i, true, r.worst);
            if (clicked()) { view_ = View::Recipe; sel_ = static_cast<int>(i); }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    PorStatus posWorst = PorStatus::Same;
    for (const PorPosRow& p : positions_.rows)
        if ((p.tier == PorPosTier::Deck || p.tier == PorPosTier::Private) && Worse(p.status, posWorst))
            posWorst = p.status;
    if (!positions_.mapIssues.empty() && Worse(PorStatus::Missing, posWorst)) posWorst = PorStatus::Missing;
    ExplorerItem("Positions", view_ == View::Positions, true, posWorst);
    if (clicked()) view_ = View::Positions;

    if (ExplorerItem("Hardware", false, false, WorstOf(hardware_.sections), true)) {
        for (std::size_t i = 0; i < hardware_.sections.size(); ++i) {
            const PorRow& s = hardware_.sections[i];
            ImGui::PushID(static_cast<int>(i) + 200000);
            ExplorerItem(s.master.c_str(), view_ == View::Hardware && sel_ == (int)i, true, s.worst);
            if (clicked()) { view_ = View::Hardware; sel_ = static_cast<int>(i); }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    ExplorerItem("Notes", view_ == View::Notes, true, PorStatus::Same);
    if (clicked()) view_ = View::Notes;
}

// ---- The master | target table ---------------------------------------------------------

void PorTool::DrawContent()
{
    switch (view_) {
    case View::None:
        ImGui::TextDisabled(compared_ ? "Pick something in the explorer." : "");
        break;
    case View::Program:
        if (sel_ >= 0 && sel_ < (int)report_.deck.size()) {
            const PorRow& p = report_.deck[sel_];
            DrawRowsTable("##prog", p.children, &p.params);
        }
        break;
    case View::Slot:
        if (sel_ >= 0 && sel_ < (int)report_.deck.size() && sel2_ >= 0 &&
            sel2_ < (int)report_.deck[sel_].children.size()) {
            const PorRow& s = report_.deck[sel_].children[sel2_];
            ImGui::TextDisabled("Double-click a recipe row to open its compare.");
            DrawRowsTable("##slot", s.children, &s.params);
        }
        break;
    case View::Recipe:
        if (sel_ >= 0 && sel_ < (int)report_.recipes.size()) {
            const PorRow& r = report_.recipes[sel_];
            ImGui::Text("Recipe: %s%s", r.recipe.c_str(), r.working ? "   (the deck runs it)" : "");
            DrawRowsTable("##recipe", r.children, &r.params);
        }
        break;
    case View::Hardware:
        if (sel_ >= 0 && sel_ < (int)hardware_.sections.size()) {
            ImGui::TextDisabled("[deck] = the master's deck uses it.  Grey fields are this machine's own "
                                "(IP, port, serial) and are never judged.");
            DrawRowsTable("##hw", hardware_.sections[sel_].children, nullptr);
        }
        break;
    case View::Positions: DrawPositions(); break;
    case View::Notes:     DrawNotes();     break;
    }
}

void PorTool::DrawRowsTable(const char* id, const std::vector<PorRow>& rows,
                            const std::vector<PorParam>* header)
{
    const ImGuiTableFlags f = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable;
    if (!ImGui::BeginTable(id, 3, f)) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn(("MASTER   " + master_.root).c_str(), ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn(("TARGET   " + target_.root).c_str(), ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn("status", ImGuiTableColumnFlags_WidthStretch, 0.55f);
    ImGui::TableHeadersRow();
    if (header)
        for (const PorParam& p : *header) DrawParam(p);
    for (const PorRow& r : rows) DrawRow(r, 0);
    ImGui::EndTable();
}

void PorTool::DrawRow(const PorRow& r, int depth)
{
    if (!Visible(r)) return;
    ImGui::TableNextRow();
    if (r.status != PorStatus::Same) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, Band(r.status));
    ImGui::TableSetColumnIndex(0);
    ImGui::PushID(&r);

    const bool leaf = r.children.empty() && VisibleParams(r) == 0;
    ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_SpanAllColumns | ImGuiTreeNodeFlags_OpenOnArrow |
                            ImGuiTreeNodeFlags_OpenOnDoubleClick;
    if (leaf) fl |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (Worse(r.worst, PorStatus::Unknown) && depth < 4) fl |= ImGuiTreeNodeFlags_DefaultOpen;

    const bool colour = r.status != PorStatus::Same;
    if (colour) ImGui::PushStyleColor(ImGuiCol_Text, StatusColor(r.status));
    const std::string ml = RowLabel(r.master, r.masterOff, r.used, r.kind);
    const bool open = ImGui::TreeNodeEx("##row", fl, "%s", ml.c_str());
    if (colour) ImGui::PopStyleColor();
    if (r.kind == PorRowKind::RecipeNode && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Double-click: open the compare of recipe '%s'", r.recipe.c_str());
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) jumpRecipe_ = FindRecipe(r.recipe);
    }

    ImGui::TableSetColumnIndex(1);
    const std::string tl = RowLabel(r.target, r.targetOff, false, r.kind);
    if (r.target.empty())  ImGui::TextDisabled("%s", tl.c_str());
    else if (colour)       ImGui::TextColored(StatusColor(r.status), "%s", tl.c_str());
    else                   ImGui::TextUnformatted(tl.c_str());

    ImGui::TableSetColumnIndex(2);
    StatusCell(r.status, r.worst, "");

    if (open && !leaf) {
        for (const PorParam& p : r.params) DrawParam(p);
        for (const PorRow& c : r.children) DrawRow(c, depth + 1);
        ImGui::TreePop();
    }
    ImGui::PopID();
}

void PorTool::DrawParam(const PorParam& p)
{
    if (!allParams_ && p.status == PorStatus::Same && p.note.empty()) return;
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    const bool machine = p.cls == PorParamClass::Machine;
    const ImVec4 col = machine ? ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled) : StatusColor(p.status);
    const ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                  ImGuiTreeNodeFlags_Bullet;
    ImGui::PushStyleColor(ImGuiCol_Text, col);
    ImGui::TreeNodeEx(&p, fl, "%s:  %s", p.name.c_str(),
                      p.hasMaster ? p.master.c_str() : "\xE2\x80\x94");
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%s:  %s", p.name.c_str(), p.hasTarget ? p.target.c_str() : "\xE2\x80\x94");
    ImGui::PopStyleColor();
    ImGui::TableSetColumnIndex(2);
    StatusCell(p.status, PorStatus::Same, p.note);
}

// ---- Positions -------------------------------------------------------------------------

void PorTool::DrawPositions()
{
    for (const std::string& m : positions_.mapIssues)
        ImGui::TextColored(StatusColor(PorStatus::Missing), "%s", m.c_str());
    if (!positions_.untraced.empty()) {
        std::string t;
        for (const std::string& s : positions_.untraced) t += (t.empty() ? "" : ", ") + s;
        ImGui::TextDisabled("Not traced (a product script may find positions by name): %s", t.c_str());
    }

    const ImGuiTableFlags f = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable;
    if (!ImGui::BeginTable("##pos", 7, f)) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("device", ImGuiTableColumnFlags_WidthStretch, 0.45f);
    ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthStretch, 0.9f);
    ImGui::TableSetupColumn("uses", ImGuiTableColumnFlags_WidthStretch, 0.2f);
    ImGui::TableSetupColumn("MASTER", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("TARGET", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("difference", ImGuiTableColumnFlags_WidthStretch, 0.5f);
    ImGui::TableSetupColumn("status", ImGuiTableColumnFlags_WidthStretch, 0.8f);
    ImGui::TableHeadersRow();

    bool first = true;
    PorPosTier tier = PorPosTier::Deck;
    for (std::size_t i = 0; i < positions_.rows.size(); ++i) {
        const PorPosRow& r = positions_.rows[i];
        if (diffOnly_ && r.status == PorStatus::Same) continue;
        if (first || r.tier != tier) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextColored(StatusColor(PorStatus::Changed), "%s", TierName(r.tier));
            tier  = r.tier;
            first = false;
        }
        ImGui::PushID(static_cast<int>(i));
        ImGui::TableNextRow();
        if (r.status != PorStatus::Same) ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, Band(r.status));
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(r.device.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(r.name.c_str());
        if (!r.owner.empty()) { ImGui::SameLine(); ImGui::TextDisabled("(%s)", r.owner.c_str()); }
        ImGui::TableSetColumnIndex(2);
        if (!r.uses.empty()) {
            ImGui::Text("%zu", r.uses.size());
            if (ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();
                const std::size_t shown = std::min<std::size_t>(r.uses.size(), 30);
                for (std::size_t k = 0; k < shown; ++k) ImGui::TextUnformatted(r.uses[k].c_str());
                if (r.uses.size() > shown) ImGui::TextDisabled("... and %zu more", r.uses.size() - shown);
                ImGui::EndTooltip();
            }
        }
        ImGui::TableSetColumnIndex(3);
        if (r.hasMaster) ImGui::TextUnformatted(AxesText(r.master).c_str());
        else             ImGui::TextDisabled("\xE2\x80\x94");
        ImGui::TableSetColumnIndex(4);
        if (r.hasTarget) ImGui::TextUnformatted(AxesText(r.target).c_str());
        else             ImGui::TextDisabled("\xE2\x80\x94");
        ImGui::TableSetColumnIndex(5);
        if (r.hasMaster && r.hasTarget) {
            const ImVec4 c = r.overThreshold ? StatusColor(PorStatus::Measured)
                                             : ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
            if (r.dRot > 0.0) ImGui::TextColored(c, "%.3f mm  %.3f deg", r.dXyz, r.dRot);
            else              ImGui::TextColored(c, "%.3f mm", r.dXyz);
        }
        ImGui::TableSetColumnIndex(6);
        std::string note = r.note;
        if (r.axesDiffer) note += std::string(note.empty() ? "" : "; ") + "axes differ";
        if (r.ambiguous)  note += std::string(note.empty() ? "" : "; ") + "name used twice";
        if (r.overThreshold) note += std::string(note.empty() ? "" : "; ") + "over the threshold";
        StatusCell(r.status, PorStatus::Same, note);
        ImGui::PopID();
    }
    ImGui::EndTable();
}

// ---- Notes -------------------------------------------------------------------------------

void PorTool::DrawNotes()
{
    const auto list = [](const char* title, const std::vector<std::string>& v) {
        ImGui::SeparatorText(title);
        if (v.empty()) ImGui::TextDisabled("(nothing)");
        for (const std::string& s : v) ImGui::TextWrapped("%s", s.c_str());
    };
    ImGui::SeparatorText("This machine (resolved read-only, FireDog.ini not written)");
    ImGui::Text("exe   %s", roots_.exeDir.c_str());
    ImGui::Text("ini   %s%s", roots_.ini.c_str(), roots_.iniFound ? "" : "   (absent: defaults)");
    ImGui::Text("root  %s%s", roots_.root.c_str(), roots_.rootExists ? "" : "   (no config/ yet)");
    for (const std::string& s : roots_.notes) ImGui::TextWrapped("%s", s.c_str());
    list("Master: what was read", master_.notes);
    list("Target: what was read", target_.notes);
    list("Compare", report_.notes);
    list("Hardware", hardware_.notes);
}

// ---- Export master -------------------------------------------------------------------------

void PorTool::DrawExportWindow()
{
    ImGui::SetNextWindowSize(ImVec2(680.0f, 460.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Export master", &showExport_)) {
        ImGui::TextWrapped("Copies product '%s' from the master root, with that machine's "
                           "hardware.json, maps.local.json and motion_transforms.json, into an "
                           "EMPTY folder laid out like a machine root. Backups (*.bak) and the "
                           "position journal stay behind. The master is only read.",
                           product_.c_str());
        ImGui::SetNextItemWidth(-120.0f);
        ImGui::InputText("Destination", exportDir_, sizeof(exportDir_));
        ImGui::BeginDisabled(product_.empty() || !masterRoot_[0] || !exportDir_[0]);
        if (ImGui::Button("Export")) {
            exportOk_ = ExportMaster(masterRoot_, product_, exportDir_, &exportLog_, &exportMsg_);
            if (exportOk_) {
                exportMsg_ = "Exported " + std::to_string(exportLog_.size()) + " file(s) to " + exportDir_;
                FD_INFO("por", "%s", exportMsg_.c_str());
            } else {
                FD_WARN("por", "export failed: %s", exportMsg_.c_str());
            }
            SaveSettings();
        }
        ImGui::EndDisabled();
        if (!exportMsg_.empty())
            ImGui::TextColored(exportOk_ ? StatusColor(PorStatus::OnlyTarget) : StatusColor(PorStatus::Missing),
                               "%s", exportMsg_.c_str());
        ImGui::BeginChild("##exportlog", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
        for (const std::string& s : exportLog_) ImGui::TextUnformatted(s.c_str());
        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace firedog
