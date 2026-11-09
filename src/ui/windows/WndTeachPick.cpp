// WndTeachPick.cpp : the Teach positions window. Definitions of App members —
// see App.h.
//
// In teach mode the deck's "Teach run" button opens this instead of moving:
// it lists every saved position the run would stop at, in run order, and the
// operator ticks all of them or only some. Ticked positions open Manual Adjust
// as before; unticked ones are still DRIVEN to, without stopping, so the run
// follows the program exactly as written up to the next ticked one - and the
// queue ends after the last recipe holding a ticked position (App::StartDeckRun).
//
// The list comes from ListTeachTargets (command/TeachMode.h), which applies the
// SAME transform a teach run does to a copy, so it cannot disagree with what
// the run will do. Non-modal: the deck stays clickable and the list follows
// its selection. Nothing here writes the document.
#include "App.h"

#include "imgui.h"

#include "command/TeachMode.h"
#include "ui/Theme.h"   // kTeachOrange

#include <cstddef>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace firedog {

void App::OpenTeachPick()
{
    showTeachPick_ = true;
    teachPickRows_.clear();   // every opening starts with all positions ticked
    runStartErr_.clear();     // a refusal shown here belongs to THIS attempt
    RebuildTeachPick();
}

void App::RebuildTeachPick()
{
    const bool onNode = deckSlot_ >= 0 && !deckNode_.empty();
    teachPickProg_  = progSel_;
    teachPickSlot_  = deckSlot_;
    teachPickNode_  = deckNode_;
    teachPickScope_ = onNode ? RunScope::Node : RunScope::Slot;
    teachPickLabel_.clear();
    teachPickErr_.clear();

    // Ticks survive a rebuild by key: clicking another deck node and back must
    // not quietly re-tick what the operator just cleared.
    std::map<std::string, bool> was;
    for (const TeachPickRow& r : teachPickRows_) was[r.key] = r.checked;
    teachPickRows_.clear();

    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) {
        teachPickErr_ = "no program selected";
        return;
    }
    std::string err;
    const RunQueue q = BuildRunQueue(session_.Programs()[progSel_], teachPickScope_,
                                     deckSlot_, deckNode_, session_.Recipes(), &err);
    if (q.items.empty()) {
        teachPickErr_ = err.empty() ? "nothing to run" : err;
        return;
    }
    teachPickLabel_ = q.label;

    // A looped folder repeats its recipes in the queue; list each recipe once.
    std::map<std::string, std::vector<TeachTarget>> byRecipe;
    std::map<std::string, std::size_t> rowOf;
    for (const ProgramRunner::Item& it : q.items) {
        auto bi = byRecipe.find(it.recipe.uid);
        if (bi == byRecipe.end())
            bi = byRecipe.emplace(it.recipe.uid,
                                  ListTeachTargets(it.recipe, orch_.registry,
                                                   session_.Recipes())).first;
        for (const TeachTarget& t : bi->second) {
            const std::string key = TeachKey(t.device, t.uid);
            auto ri = rowOf.find(key);
            if (ri == rowOf.end()) {
                TeachPickRow row;
                row.key    = key;
                row.device = t.device;
                row.uid    = t.uid;
                row.name   = "(missing position)";
                const auto di = session_.Positions().find(t.device);
                if (di != session_.Positions().end())
                    for (const SavedPos& p : di->second)
                        if (p.uid == t.uid) { row.name = p.name; break; }
                const auto wi = was.find(key);
                row.checked = (wi == was.end()) ? true : wi->second;
                ri = rowOf.emplace(key, teachPickRows_.size()).first;
                teachPickRows_.push_back(std::move(row));
            }
            ++teachPickRows_[ri->second].stops;
        }
    }
}

void App::DrawTeachPickWindow()
{
    // Teach mode off (toggle, sign-out, level) closes it: it only means
    // anything as the first half of a teach run.
    if (!teachMode_ || !CanAccess(Page::Motion)) { showTeachPick_ = false; return; }

    // Follow the deck: what Start would run is whatever is selected NOW.
    {
        const bool onNode = deckSlot_ >= 0 && !deckNode_.empty();
        const RunScope scope = onNode ? RunScope::Node : RunScope::Slot;
        if (progSel_ != teachPickProg_ || deckSlot_ != teachPickSlot_ ||
            deckNode_ != teachPickNode_ || scope != teachPickScope_)
            RebuildTeachPick();
    }

    const float font = ImGui::GetStyle().FontSizeBase * 1.3f;
    const float k    = font / ImGui::GetStyle().FontSizeBase;
    ImGui::PushFont(nullptr, font);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(620.0f * k, 520.0f * k), ImGuiCond_Appearing);
    if (!ImGui::Begin("Teach positions", &showTeachPick_,
                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking |
                          ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        ImGui::PopFont();
        return;
    }

    ImGui::TextColored(kTeachOrange, "TEACH");
    ImGui::SameLine();
    ImGui::TextUnformatted(teachPickLabel_.empty() ? "(nothing selected)"
                                                   : teachPickLabel_.c_str());
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("Checked positions stop for Manual Adjust. Unchecked ones are "
                        "driven to WITHOUT stopping. IO, pneumatics and every other step "
                        "stay off, as in every teach run.");
    ImGui::PopTextWrapPos();

    int nChecked = 0;
    for (const TeachPickRow& r : teachPickRows_)
        if (r.checked) ++nChecked;
    const int nRows = static_cast<int>(teachPickRows_.size());

    if (ImGui::Button("Check all"))
        for (TeachPickRow& r : teachPickRows_) r.checked = true;
    ImGui::SameLine();
    if (ImGui::Button("Uncheck all"))
        for (TeachPickRow& r : teachPickRows_) r.checked = false;
    ImGui::SameLine();
    ImGui::TextDisabled("%d of %d checked", nChecked, nRows);

    const float footH = ImGui::GetFrameHeightWithSpacing() * 2.0f +
                        ImGui::GetTextLineHeightWithSpacing() * 2.0f;
    ImGui::BeginChild("tplist", ImVec2(0.0f, -footH), ImGuiChildFlags_Borders);
    if (!teachPickErr_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", teachPickErr_.c_str());
    } else if (teachPickRows_.empty()) {
        ImGui::TextDisabled("No moves to saved positions in this selection - "
                            "nothing to teach.");
    } else if (ImGui::BeginTable("tptab", 4,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                     ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("#");
        ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Device");
        ImGui::TableSetupColumn("Stops");
        ImGui::TableHeadersRow();
        for (int i = 0; i < nRows; ++i) {
            TeachPickRow& r = teachPickRows_[static_cast<std::size_t>(i)];
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d", i + 1);
            ImGui::TableNextColumn();
            ImGui::Checkbox(r.name.c_str(), &r.checked);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(r.device.c_str());
            ImGui::TableNextColumn();
            if (r.stops > 1) ImGui::Text("x%d", r.stops);
            else             ImGui::TextDisabled("1");
            if (ImGui::IsItemHovered() && r.stops > 1)
                ImGui::SetTooltip("The run arrives here %d times; a checked position "
                                  "stops every time.", r.stops);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();

    // Where the run will end, said before it moves.
    const TeachPickRow* lastPick = nullptr;
    for (const TeachPickRow& r : teachPickRows_)
        if (r.checked) lastPick = &r;
    if (nChecked > 0 && nChecked < nRows && lastPick)
        ImGui::TextDisabled("The run ends with the recipe that holds the last checked "
                            "position ('%s').", lastPick->name.c_str());
    else if (nChecked == nRows && nRows > 0)
        ImGui::TextDisabled("Every position stops - the same as a teach run without a pick.");
    else
        ImGui::TextDisabled("Check at least one position.");

    const bool busy = orch_.Busy();
    ImGui::BeginDisabled(busy || nChecked == 0 || !teachPickErr_.empty());
    ImGui::PushStyleColor(ImGuiCol_Button,        kTeachOrange);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kTeachOrangeHov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kTeachOrangeAct);
    char go[64];
    std::snprintf(go, sizeof(go), "Start teach (%d)", nChecked);
    if (ImGui::Button(go, ImVec2(220.0f * k, 0.0f))) {
        std::set<std::string> picked;
        for (const TeachPickRow& r : teachPickRows_)
            if (r.checked) picked.insert(r.key);
        // All ticked = the plain teach run: no pick, no trimmed queue.
        StartDeckRun(teachPickScope_, nChecked == nRows ? nullptr : &picked);
        if (runStartErr_.empty()) showTeachPick_ = false;   // refused: stay, say why
    }
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();
    if (busy && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("A run is active.");
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(140.0f * k, 0.0f))) showTeachPick_ = false;
    if (!runStartErr_.empty())
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "Not started: %s",
                           runStartErr_.c_str());

    ImGui::End();
    ImGui::PopFont();
}

} // namespace firedog
