// PageRecipeEdit.cpp : the Recipe Editor page (View menu, full-page) — recipe
// list, step tree, palette, toolbar and the run strip. The schema-driven forms
// live in RecipeForms.cpp. Definitions of App members — see App.h.
// Design: docs/recipe-editor-design.md.
//
// The step-path helpers below are the editor's addressing scheme into the
// nested step tree and belong with it. kElsePathBase MUST stay an alias of
// Recipe.h's kElseBranchBase: re-typing the literal would desync editor
// selection from the paths the validation layer emits.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"
#include "core/DutSession.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <string>

namespace firedog {

namespace {
// Step-path components >= kElsePathBase address an if-step's ELSE branch
// (component - kElsePathBase = index into elseChildren). Shared with the
// validation layer (Recipe.h kElseBranchBase) so issue paths select tree rows
// directly. The engine never emits else-marked paths (it addresses whichever
// branch ran by plain index), so run highlighting lands on the then-branch
// row — a documented quirk.
constexpr std::size_t kElsePathBase = kElseBranchBase;

const RecipeStep* FindStepByPath(const std::vector<RecipeStep>* list,
                                 const std::vector<std::size_t>& path)
{
    const RecipeStep* cur = nullptr;
    for (const std::size_t c : path) {
        const bool        inElse = c >= kElsePathBase;
        const std::size_t idx    = inElse ? c - kElsePathBase : c;
        if (inElse) {
            if (!cur) return nullptr;
            list = &cur->elseChildren;
        }
        if (!list || idx >= list->size()) return nullptr;
        cur  = &(*list)[idx];
        list = &cur->children;
    }
    return cur;
}

// Safe: the caller passes non-const storage; only the view was const.
RecipeStep* FindStepByPathMut(std::vector<RecipeStep>* list,
                              const std::vector<std::size_t>& path)
{
    return const_cast<RecipeStep*>(FindStepByPath(list, path));
}

// The list CONTAINING the addressed step, plus its index there — the handle
// needed for insert / move / delete.
std::vector<RecipeStep>* FindStepParent(std::vector<RecipeStep>* list,
                                        const std::vector<std::size_t>& path,
                                        std::size_t* idxOut)
{
    RecipeStep* cur = nullptr;
    for (std::size_t k = 0; k < path.size(); ++k) {
        const std::size_t c      = path[k];
        const bool        inElse = c >= kElsePathBase;
        const std::size_t idx    = inElse ? c - kElsePathBase : c;
        if (inElse) {
            if (!cur) return nullptr;
            list = &cur->elseChildren;
        }
        if (!list || idx >= list->size()) return nullptr;
        if (k + 1 == path.size()) {
            *idxOut = idx;
            return list;
        }
        cur  = &(*list)[idx];
        list = &cur->children;
    }
    return nullptr;
}

// A fresh step of `type`: structural types get a minimal valid shape,
// commands get their non-required params prefilled with the schema defaults
// (required ones are left for the form + validation to demand).
RecipeStep MakeStep(const std::string& type, const CommandRegistry& reg)
{
    RecipeStep s;
    s.type = type;
    if (type == "loop") {
        s.params["count"] = 1;
    } else if (type == "if") {
        s.params["key"] = "";
        s.params["op"]  = "is_true";
    } else if (type == "stop_recipe") {
        s.params["ok"] = true;
    } else if (const CommandSpec* spec = reg.Find(type)) {
        for (const ParamSpec& p : spec->params) {
            if (!p.required) s.params[p.name] = p.defaultValue;
        }
    }
    return s;
}

// Case-insensitive substring match for the palette search box ("" = all).
bool PaletteMatch(const std::string& hay, const char* needle)
{
    if (!needle[0]) return true;
    const auto lower = [](std::string s) {
        for (char& c : s)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };
    return lower(hay).find(lower(needle)) != std::string::npos;
}
} // namespace

void App::DrawRecipeEditPanel()
{
    const RecipeEngine::Status st = orch_.recipe.GetStatus();
    const bool running = orch_.recipe.Running();
    // What DISABLES the editor is the machine being busy, not this one engine.
    // A program run goes momentarily idle between queue items (the same reason
    // every Run surface gates on Busy), and an editor that comes back to life
    // in that gap lets an operator who only came here to read start editing,
    // reloading or deleting the recipe the next queue item is about to run.
    // `running` stays the engine's own state - it is what the gold executing
    // row means.
    const bool busy = orch_.Busy();
    const bool haveSel = recipeSel_ >= 0 && recipeSel_ < static_cast<int>(session_.Recipes().size());

    // Live author-time validation of the selected recipe (WITHOUT the
    // devices-Connected requirement — that stays a run-start check, so
    // authoring offline doesn't drown in noise).
    std::vector<RecipeIssue> issues;
    if (haveSel)
        issues = ValidateRecipe(session_.Recipes()[recipeSel_], orch_.registry, hardware_,
                                session_.Positions(), session_.Maps(), session_.Aligns(),
                                session_.VisionModels(), session_.Grips(),
                                session_.Recipes(), false);
    // The issues list lives in its own band at the BOTTOM of the page (the
    // run strip on top keeps only state/progress/errors).
    const std::size_t shownIssues = issues.size() < 3 ? issues.size() : 3;
    const float issuesH =
        issues.empty() ? 0.0f
                       : ImGui::GetTextLineHeightWithSpacing() *
                                 static_cast<float>(shownIssues + 1) + 10.0f;

    const float  stripH  = ImGui::GetFrameHeightWithSpacing() * 3.2f;
    const float  spacing = ImGui::GetStyle().ItemSpacing.y;
    const ImVec2 avail   = ImGui::GetContentRegionAvail();
    const float  paneH   = avail.y - stripH - issuesH -
                           spacing * (issues.empty() ? 1.0f : 2.0f);
    const float  leftW  = 230.0f;
    // The inspector is the one column whose natural width is not a constant: a
    // move_relative form is eight labelled fields, an XCAN step's is thirty, and
    // the operator's font size moves both. Dragged by the splitter below and
    // persisted, like the main window's panes.
    constexpr float kSplitW    = 6.0f;
    constexpr float kMinMidW   = 260.0f;
    constexpr float kMinRightW = 220.0f;
    if (edInspectW_ < kMinRightW) edInspectW_ = kMinRightW;
    const float maxRightW = avail.x - leftW - ImGui::GetStyle().ItemSpacing.x -
                            kSplitW - kMinMidW;
    if (edInspectW_ > maxRightW) edInspectW_ = maxRightW > kMinRightW ? maxRightW : kMinRightW;
    const float  rightW = edInspectW_;

    // ---- Top: run controls + state + progress (panes fill the rest) ----
    DrawRecipeRunStrip(stripH);

    // ---- Left: recipe list + CRUD ----
    ImGui::BeginChild("edRecipes", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Recipes");
    ImGui::SameLine();
    ImGui::BeginDisabled(busy);
    if (ImGui::SmallButton("Reload")) {
        if (session_.RecipesDirty()) {
            edConfirmAction_ = 2;   // guard: reload discards unsaved edits
        } else {
            LoadRecipesFromDisk();
            FD_INFO("recipe", "%zu recipe(s) loaded", session_.Recipes().size());
        }
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy);
    if (IconSmallButton(UiIcon("New"), "New")) {
        Recipe r;
        r.uid  = GenerateUid();
        r.name = "new recipe";
        session_.Recipes().push_back(std::move(r));
        recipeSel_ = static_cast<int>(session_.Recipes().size()) - 1;
        ClearStepSel();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveSel);
    if (ImGui::SmallButton("Dup")) {
        Recipe r = session_.Recipes()[recipeSel_];
        r.uid  = GenerateUid();
        r.name += " copy";
        session_.Recipes().push_back(std::move(r));
        recipeSel_ = static_cast<int>(session_.Recipes().size()) - 1;
        ClearStepSel();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Ren")) {
        std::snprintf(edNameBuf_, sizeof(edNameBuf_), "%s", session_.Recipes()[recipeSel_].name.c_str());
        edOpenRenameRecipe_ = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Del")) edOpenDeleteRecipe_ = true;
    ImGui::EndDisabled();
    // Clipboard round-trip: Copy puts the selected recipe's JSON on the OS
    // clipboard; Paste accepts a recipe object, a whole recipes file, or a
    // bare array, and mints a FRESH uid per pasted recipe so an in-place
    // copy never collides with the original's identity.
    ImGui::BeginDisabled(!haveSel);
    if (ImGui::SmallButton("Copy"))
        ImGui::SetClipboardText(RecipeToJson(session_.Recipes()[recipeSel_]).dump(2).c_str());
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Copy the selected recipe as JSON to the clipboard");
    ImGui::SameLine();
    if (ImGui::SmallButton("Paste")) {
        const char* clip = ImGui::GetClipboardText();
        nlohmann::json j = clip ? nlohmann::json::parse(clip, nullptr, false)
                                : nlohmann::json();
        nlohmann::json arr = nlohmann::json::array();
        if (j.is_array()) arr = j;
        else if (j.is_object() && j.contains("recipes") && j["recipes"].is_array())
            arr = j["recipes"];
        else if (j.is_object()) arr.push_back(j);
        int added = 0;
        for (const auto& rj : arr) {
            if (!rj.is_object() || !(rj.contains("steps") || rj.contains("name")))
                continue;   // not recipe-shaped — don't import junk
            Recipe r = RecipeFromJson(rj);
            r.uid = GenerateUid();   // pasted copy gets its OWN identity
            for (const Recipe& ex : session_.Recipes()) {
                if (ex.name == r.name) { r.name += " copy"; break; }
            }
            session_.Recipes().push_back(std::move(r));
            ++added;
        }
        if (added > 0) {
            recipeSel_ = static_cast<int>(session_.Recipes().size()) - 1;
            ClearStepSel();
            FD_INFO("recipe", "pasted %d recipe(s) from clipboard", added);
        } else {
            FD_WARN("recipe", "paste: clipboard has no recipe JSON");
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Paste recipe JSON from the clipboard (new uid)");
    ImGui::EndDisabled();
    ImGui::Separator();
    if (session_.Recipes().empty())
        ImGui::TextWrapped("No recipes yet - New creates one, or Reload from disk.");
    for (int i = 0; i < static_cast<int>(session_.Recipes().size()); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(session_.Recipes()[i].name.c_str(), recipeSel_ == i)) {
            recipeSel_ = i;
            ClearStepSel();
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Middle: searchable palette column + step tree ----
    // One ItemSpacing gap on the left (the recipe list's SameLine), then the
    // splitter, which is flush on both sides so the drag bar IS the border.
    // Floored: on a window too narrow for all three minimums ImGui would read a
    // negative width as "available minus this" and scramble the layout.
    const float midW = fmaxf(80.0f, avail.x - leftW - rightW -
                                        ImGui::GetStyle().ItemSpacing.x - kSplitW);
    // The palette's width is dragged (splitter between it and the tree) and
    // persisted: the long command names (grip_camera_to_object) want it wide,
    // a deep step tree wants it narrow, and no constant serves both.
    constexpr float kMinPalW  = 120.0f;
    constexpr float kMinTreeW = 120.0f;
    if (edPaletteW_ < kMinPalW) edPaletteW_ = kMinPalW;
    const float maxPalW = midW - kSplitW - kMinTreeW;
    if (edPaletteW_ > maxPalW) edPaletteW_ = maxPalW > kMinPalW ? maxPalW : kMinPalW;
    const float palW = edPaletteW_;
    ImGui::BeginChild("edMiddle", ImVec2(midW, paneH));
    if (haveSel) {
        Recipe& r = session_.Recipes()[recipeSel_];
        ImGui::BeginChild("edPalette", ImVec2(palW, 0.0f), ImGuiChildFlags_Borders);
        DrawRecipePalette(r, !busy);
        ImGui::EndChild();
        ImGui::SameLine(0.0f, 0.0f);
        if (Splitter("##edpalsplit", true, kSplitW, paneH)) {
            edPaletteW_ += ImGui::GetIO().MouseDelta.x;
            edPaletteSplitting_ = true;
        } else if (edPaletteSplitting_) {
            edPaletteSplitting_ = false;
            SaveUiSettings();   // persist on drag END, like the inspector's
        }
        ImGui::SameLine(0.0f, 0.0f);
        ImGui::BeginChild("edTree", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
        DrawRecipeTreeToolbar(r, !busy);
        ImGui::Separator();
        // Rows scroll in their own child so the ^/v/Delete toolbar above
        // stays pinned (same pattern as the palette's ##palList).
        ImGui::BeginChild("##edTreeRows");
        edTreeRows_.clear();
        std::vector<std::size_t> path;
        // While step-paused the gold row is the step ABOUT to run (Next).
        DrawRecipeTreeNodes(r.steps, path, false,
                            st.stepPaused ? st.nextPath : st.stepPath, running);
        // Arrow keys walk the visible rows (Up/Down; Left jumps to the
        // parent). Skipped while a text field or popup owns the keyboard.
        if (!ImGui::GetIO().WantTextInput &&
            !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId |
                                        ImGuiPopupFlags_AnyPopupLevel) &&
            !edTreeRows_.empty()) {
            const int d = (ImGui::IsKeyPressed(ImGuiKey_DownArrow) ? 1 : 0) -
                          (ImGui::IsKeyPressed(ImGuiKey_UpArrow) ? 1 : 0);
            if (d != 0) {
                int cur = -1;
                for (int k = 0; k < static_cast<int>(edTreeRows_.size()); ++k) {
                    if (edTreeRows_[k] == edStepSel_) { cur = k; break; }
                }
                const int next =
                    cur < 0 ? (d > 0 ? 0 : static_cast<int>(edTreeRows_.size()) - 1)
                            : cur + d;
                if (next >= 0 && next < static_cast<int>(edTreeRows_.size())) {
                    edStepSel_     = edTreeRows_[next];
                    edStepMulti_.clear();
                    edScrollToSel_ = true;
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow) && edStepSel_.size() > 1) {
                edStepSel_.pop_back();   // parent (also from an else-branch child)
                edStepMulti_.clear();
                edScrollToSel_ = true;
            }
        }
        // A Shift-click from this frame's draw: the range is every VISIBLE row
        // between the anchor and the clicked one, the anchor staying put. An
        // anchor inside a collapsed node is not in the list; the click then
        // just selects the row, like a plain one.
        if (edShiftPending_) {
            edShiftPending_ = false;
            int a = -1, b = -1;
            for (int k = 0; k < static_cast<int>(edTreeRows_.size()); ++k) {
                if (edTreeRows_[k] == edStepSel_) a = k;
                if (edTreeRows_[k] == edShiftTo_) b = k;
            }
            edStepMulti_.clear();
            if (a < 0 || b < 0) {
                edStepSel_ = edShiftTo_;
            } else {
                const int lo = a < b ? a : b, hi = a < b ? b : a;
                for (int k = lo; k <= hi; ++k)
                    if (k != a) edStepMulti_.push_back(edTreeRows_[k]);
            }
        }
        ImGui::EndChild();   // ##edTreeRows
        ImGui::EndChild();   // edTree
    } else {
        ImGui::TextDisabled("Select a recipe on the left.");
    }
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##edsplit", true, kSplitW, paneH)) {
        edInspectW_ -= ImGui::GetIO().MouseDelta.x;
        edInspectSplitting_ = true;
    } else if (edInspectSplitting_) {
        edInspectSplitting_ = false;
        SaveUiSettings();   // persist on drag END, like every other splitter here
    }
    ImGui::SameLine(0.0f, 0.0f);

    // ---- Right: inspector (step form, or recipe fields when no step) ----
    ImGui::BeginChild("edInspect", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    RecipeStep* sel = nullptr;
    if (haveSel && !edStepSel_.empty())
        sel = FindStepByPathMut(&session_.Recipes()[recipeSel_].steps, edStepSel_);
    if (sel) {
        DrawRecipeInspector(*sel, !busy);
    } else if (haveSel) {
        Recipe& r = session_.Recipes()[recipeSel_];
        ImGui::BeginDisabled(busy);
        char nbuf[64];
        std::snprintf(nbuf, sizeof(nbuf), "%s", r.name.c_str());
        if (ImGui::InputText("name", nbuf, sizeof(nbuf))) r.name = nbuf;
        char dbuf[512];
        std::snprintf(dbuf, sizeof(dbuf), "%s", r.description.c_str());
        if (ImGui::InputTextMultiline("description", dbuf, sizeof(dbuf),
                                      ImVec2(-1.0f, 60.0f)))
            r.description = dbuf;
        ImGui::EndDisabled();
        ImGui::TextDisabled("uid: %s", r.uid.c_str());
        ImGui::Separator();
        ImGui::TextDisabled("Select a step to edit it.");
    } else {
        ImGui::TextDisabled("Select a recipe first.");
    }
    ImGui::EndChild();

    // ---- Bottom: clickable validation-issues band (full width) ----
    if (!issues.empty()) {
        ImGui::BeginChild("edIssues", ImVec2(0.0f, issuesH), ImGuiChildFlags_Borders);
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "%zu issue(s) - click to select:", issues.size());
        ImGui::BeginChild("##valList");   // scrolls when more than the band shows
        for (std::size_t k = 0; k < issues.size(); ++k) {
            const RecipeIssue& is = issues[k];
            ImGui::PushID(static_cast<int>(k));
            if (ImGui::Selectable((is.where + ": " + is.message).c_str()) &&
                !is.path.empty()) {
                edStepSel_ = is.path;
                edStepMulti_.clear();
            }
            ImGui::PopID();
        }
        ImGui::EndChild();
        ImGui::EndChild();
    }

    // ---- Unsaved-changes guard (Close button / Reload raise the flag) ----
    if (edConfirmAction_ != 0 && !ImGui::IsPopupOpen("Unsaved Changes"))
        ImGui::OpenPopup("Unsaved Changes");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Unsaved Changes", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("There are unsaved recipe changes.");
        ImGui::Separator();
        if (ImGui::Button("Save", ImVec2(90.0f, 0.0f))) {
            session_.SaveRecipes();
            const int action = edConfirmAction_;
            edConfirmAction_ = 0;
            if (action == 1) SwitchPage(edLeaveTarget_);   // no longer dirty
            else             LoadRecipesFromDisk();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", ImVec2(90.0f, 0.0f))) {
            LoadRecipesFromDisk();   // throw away the in-memory edits
            const int action = edConfirmAction_;
            edConfirmAction_ = 0;
            if (action == 1) SwitchPage(edLeaveTarget_);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 0.0f))) {
            edConfirmAction_ = 0;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // ---- Editor modals (opened via the flags raised above) ----
    if (edOpenRenameRecipe_) { ImGui::OpenPopup("Rename Recipe"); edOpenRenameRecipe_ = false; }
    if (edOpenDeleteRecipe_) { ImGui::OpenPopup("Delete Recipe"); edOpenDeleteRecipe_ = false; }
    if (edOpenDeleteStep_)   { ImGui::OpenPopup("Delete Step");   edOpenDeleteStep_   = false; }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Rename Recipe", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::InputText("##recname", edNameBuf_, sizeof(edNameBuf_));
        if (ImGui::Button("OK", ImVec2(100.0f, 0.0f)) && haveSel) {
            session_.Recipes()[recipeSel_].name = edNameBuf_;   // label only; uid unchanged
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Recipe", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (haveSel)
            ImGui::Text("Delete recipe '%s'?", session_.Recipes()[recipeSel_].name.c_str());
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveSel) {
            session_.Recipes().erase(session_.Recipes().begin() + recipeSel_);
            recipeSel_ = -1;
            ClearStepSel();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Step", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        // The whole selection, not just the anchor: rows highlighted as
        // selected and left behind by Delete would be a lie.
        const std::vector<std::vector<std::size_t>> paths = SelectedStepPaths();
        const RecipeStep* ds = (haveSel && paths.size() == 1)
                                   ? FindStepByPath(&session_.Recipes()[recipeSel_].steps, paths[0])
                                   : nullptr;
        if (paths.size() > 1)
            ImGui::Text("Delete %zu steps (children included)?", paths.size());
        else
            ImGui::Text("Delete step '%s'%s?", ds ? ds->type.c_str() : "?",
                        ds && !ds->children.empty() ? " and its children" : "");
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveSel && !paths.empty()) {
            DeleteSelectedSteps(session_.Recipes()[recipeSel_]);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    DrawTeachPositionModal();
}

// Re-teach the position a step REFERENCES, in place. Raised by the "Teach here"
// button on a PositionUid form row (RecipeForms.cpp) through openTeachPos_,
// which is a flag rather than a direct OpenPopup because the row draws inside
// the inspector's child window and the popup has to live at panel level.
//
// This is the one surface in the editor that moves the document outside a
// recipe, so it asks. What it has to say is not "are you sure" but WHO ELSE is
// affected: the uid is preserved, so every other step pointing at this position
// keeps resolving and quietly follows it to the new pose.
void App::DrawTeachPositionModal()
{
    if (openTeachPos_) { ImGui::OpenPopup("Teach Position"); openTeachPos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("Teach Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;

    IMotionController* m = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i)
        if (hardware_.MotionName(i) == teachPosDev_) { m = hardware_.MotionDevice(i); break; }
    SavedPos*  tgt = nullptr;
    const auto it  = session_.Positions().find(teachPosDev_);
    if (it != session_.Positions().end())
        for (SavedPos& sp : it->second)
            if (sp.uid == teachPosUid_) { tgt = &sp; break; }

    if (!tgt || !m || !m->IsConnected()) {
        ImGui::TextUnformatted(!tgt ? "That saved position no longer exists."
                                    : "The device is not connected.");
        if (ImGui::Button("Close", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
    }

    // The driver idles at 1 Hz and nothing on this page asks otherwise, so a
    // pose read on the button click would be up to a second stale - at a taught
    // position that difference IS the measurement. Demand fast polls for as long
    // as the dialog is up and show what arrives: by the time anyone has read
    // this and pressed Teach the numbers are current, and they are the numbers
    // about to be stored.
    m->RequestFastPoll();
    const std::map<std::string, double> live = m->Positions();

    ImGui::Text("Re-teach '%s' from the live %s pose?", tgt->name, teachPosDev_.c_str());
    ImGui::Separator();
    if (ImGui::BeginTable("teachpos", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Axis");
        ImGui::TableSetupColumn("Saved");
        ImGui::TableSetupColumn("Live");
        ImGui::TableHeadersRow();
        for (const std::string& ax : m->Axes()) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(ax.c_str());
            ImGui::TableSetColumnIndex(1);
            const auto sv = tgt->vals.find(ax);
            if (sv != tgt->vals.end()) ImGui::Text("%.4f", sv->second);
            else                       ImGui::TextDisabled("--");
            ImGui::TableSetColumnIndex(2);
            const auto lv = live.find(ax);
            if (lv != live.end()) ImGui::Text("%.4f", lv->second);
            else                  ImGui::TextDisabled("--");
        }
        ImGui::EndTable();
    }
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                       "'%s' is a shared saved position.", tgt->name);
    ImGui::TextDisabled("uid %s is kept, so nothing breaks.", tgt->uid.c_str());
    ImGui::TextDisabled("Every other step, program and procedure that references it "
                        "moves here too.");
    ImGui::Spacing();
    if (ImGui::Button("Teach", ImVec2(100.0f, 0.0f))) {
        TeachPositionByUid(teachPosDev_, teachPosUid_);
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

// Overwrite an EXISTING saved position with the device's live pose, found by
// uid and updated in place. It never creates one: the uid came from a step that
// is already referencing it, so a missing row means a STALE reference, and
// inventing a position for it would bury exactly the error the picker draws in
// red. Same update-in-place rule as CapturePrivatePose (PageAlign.cpp), for the
// shared, named half of the table rather than the private half.
bool App::TeachPositionByUid(const std::string& device, const std::string& uid)
{
    IMotionController* m = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i)
        if (hardware_.MotionName(i) == device) { m = hardware_.MotionDevice(i); break; }
    if (!m || !m->IsConnected()) return false;

    const auto it = session_.Positions().find(device);
    if (it == session_.Positions().end()) return false;
    for (SavedPos& sp : it->second) {
        if (sp.uid != uid) continue;
        const std::map<std::string, double> live = m->Positions();
        for (const std::string& ax : m->Axes()) {
            const auto lv = live.find(ax);
            sp.vals[ax] = (lv != live.end()) ? lv->second : 0.0;
        }
        session_.SavePositions();
        FD_INFO("saved", "re-taught '%s' (uid %s, %s) from the live pose",
                sp.name, uid.c_str(), device.c_str());
        return true;
    }
    FD_WARN("saved", "teach: '%s' has no position uid %s", device.c_str(), uid.c_str());
    return false;
}

// Live-generated step label from the params: shown in the tree whenever the
// nickname is empty (so it always tracks edits), and materialized into the
// nickname field by the inspector's "gen" button. Skips plumbing params
// (timeouts, tolerances, poll rates) and resolves position uids to names.
std::string App::AutoNickname(const RecipeStep& s) const
{
    const auto num = [](double v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%g", v);
        return std::string(b);
    };
    const auto condText = [&num](const nlohmann::json& c) {
        std::string t = c.value("key", c.value("topic", std::string()));
        t += " " + c.value("op", std::string());
        if (c.contains("value")) {
            const nlohmann::json& v = c["value"];
            t += " " + (v.is_string() ? v.get<std::string>()
                                      : v.is_number() ? num(v.get<double>()) : v.dump());
        }
        return t;
    };

    if (s.type == "loop") {
        if (s.params.contains("while"))
            return "loop while " + condText(s.params["while"]);
        return "loop x" + num(s.params.value("count", 1.0));
    }
    if (s.type == "if") return "if " + condText(s.params);
    if (s.type == "group") return "group";
    if (s.type == "sub_recipe") {
        const std::string uid = s.params.value("uid", std::string());
        for (const Recipe& r : session_.Recipes())
            if (r.uid == uid) return "run " + r.name;
        return "sub_recipe";
    }
    if (s.type == "stop_recipe")
        return s.params.value("ok", true) ? "stop (ok)" : "stop (fail)";

    const CommandSpec* spec = orch_.registry.Find(s.type);
    if (!spec) return s.type;

    const auto skip = [](const std::string& n) {
        return n == "timeout_ms" || n == "leg_timeout_ms" || n == "tol_mm" ||
               n == "start_tol_mm" || n == "poll_ms" || n == "max_iterations" ||
               n == "velocity_mm_s" || n == "exposure_us" || n == "settle_min_ms" ||
               n == "pre_delay_ms" || n == "post_delay_ms";
    };
    std::string out = s.type;
    const std::string dev = s.params.value("device", std::string());
    int added = 0;
    for (const ParamSpec& p : spec->params) {
        if (added >= 3) break;
        if (skip(p.name) || !s.params.contains(p.name)) continue;
        const nlohmann::json& v = s.params[p.name];
        std::string piece;
        if (p.type == ParamType::PositionUid && v.is_string()) {
            piece = v.get<std::string>();
            const auto it = session_.Positions().find(dev);
            if (it != session_.Positions().end()) {
                for (const SavedPos& sp : it->second)
                    if (sp.uid == piece) { piece = sp.name; break; }
            }
        } else if (p.type == ParamType::NominalUid && v.is_string()) {
            // Same reason as PositionUid: the step stores a uid, and a uid in
            // the step tree tells the operator nothing.
            piece = v.get<std::string>();
            const AlignProc* ap =
                FindAlignProc(session_.Aligns(), s.params.value("alignment", std::string()));
            if (ap) {
                if (const AlignNominal* nm = FindAlignNominal(*ap, piece)) piece = nm->name;
            }
        } else if (p.type == ParamType::GripUid && v.is_string()) {
            piece = v.get<std::string>();
            if (const GripProfile* g = FindGripProfile(session_.Grips(), piece))
                piece = g->name;
        } else if (p.type == ParamType::IntChoice && v.is_number()) {
            // The choice's name, for the same reason: "align_station 2" makes
            // the reader look the numbering up.
            const int idx = static_cast<int>(v.get<double>()) - static_cast<int>(p.minVal);
            if (idx >= 0 && idx < static_cast<int>(p.options.size())) piece = p.options[idx];
            else piece = num(v.get<double>());
        } else if (p.type == ParamType::ScriptFile && v.is_string()) {
            piece = v.get<std::string>();   // show the file name, not the path
            const std::size_t cut = piece.find_last_of("/\\");
            if (cut != std::string::npos) piece = piece.substr(cut + 1);
        } else if (v.is_string()) {
            piece = v.get<std::string>();
        } else if (v.is_boolean()) {
            piece = v.get<bool>() ? "on" : "off";
        } else if (v.is_number()) {
            piece = num(v.get<double>());
        }
        if (piece.empty()) continue;
        out += " " + piece;
        ++added;
    }
    return out;
}

// One row per step; structural steps (except stop_recipe) are tree nodes.
// The executing step (engine stepPath) renders gold while a run is active.
void App::DrawRecipeTreeNodes(const std::vector<RecipeStep>& steps,
                              std::vector<std::size_t>& path, bool elseBranch,
                              const std::vector<std::size_t>& runPath, bool running)
{
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const RecipeStep& s = steps[i];
        path.push_back(elseBranch ? kElsePathBase + i : i);

        const bool structural = IsStructuralStep(s.type) && s.type != "stop_recipe";
        const bool anchor     = (path == edStepSel_);
        const bool selected   = anchor ||
            std::find(edStepMulti_.begin(), edStepMulti_.end(), path) != edStepMulti_.end();
        const bool executing  = running && path == runPath;

        // Every row is three pieces — number, icon, text — so the icon can sit
        // between the first two. The number gets a fixed column so the icons
        // line up vertically; a column the eye cannot run down is decoration.
        const std::string shown = s.nickname.empty() ? AutoNickname(s) : s.nickname;
        char text[160];
        std::snprintf(text, sizeof(text), "%s%s", shown.c_str(),
                      s.enabled ? "" : "  [off]");
        const float numW = ImGui::CalcTextSize("00").x +
                           ImGui::GetStyle().ItemInnerSpacing.x;

        const bool dim = !s.enabled && !executing;
        if (executing)
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.3f, 1.0f));
        else if (dim)
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);

        ImGui::PushID(static_cast<int>(path.back()));
        if (structural) {
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                       ImGuiTreeNodeFlags_SpanAvailWidth |
                                       ImGuiTreeNodeFlags_DefaultOpen;
            if (selected) flags |= ImGuiTreeNodeFlags_Selected;
            // The NUMBER is the node's own label — it has to sit after the
            // arrow, and only ImGui knows how wide that is. The icon and the
            // text are laid back over the row afterwards, same rewind as the
            // leaf branch, so `group` carries the glyph the palette gave it.
            const float lblX = ImGui::GetCursorPosX() +
                               ImGui::GetTreeNodeToLabelSpacing();
            const bool open = ImGui::TreeNodeEx("##node", flags, "%zu", i + 1);
            edTreeRows_.push_back(path);
            if (anchor && edScrollToSel_) {
                ImGui::SetScrollHereY(0.35f);
                edScrollToSel_ = false;
            }
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
                ClickStepRow(path);
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::SetCursorPosX(lblX + numW);
            DrawStepIconInline(StepIcon(s.type.c_str()));
            ImGui::TextUnformatted(text);
            if (open) {
                DrawRecipeTreeNodes(s.children, path, false, runPath, running);
                if (!s.elseChildren.empty()) {
                    ImGui::TextDisabled("else:");
                    DrawRecipeTreeNodes(s.elseChildren, path, true, runPath, running);
                }
                ImGui::TreePop();
            }
        } else {
            // One full-width Selectable carrying the whole row's click target,
            // with the number, the icon and the text laid back over it.
            const float x0 = ImGui::GetCursorPosX();
            if (ImGui::Selectable("##row", selected, ImGuiSelectableFlags_AllowOverlap))
                ClickStepRow(path);
            edTreeRows_.push_back(path);
            if (anchor && edScrollToSel_) {
                ImGui::SetScrollHereY(0.35f);
                edScrollToSel_ = false;
            }
            // Back onto the row via SameLine, NOT SetCursorScreenPos: the line
            // has to stay open so the last piece closes it. A hand-restored
            // cursor at the end of the last row trips ImGui's
            // "SetCursorPos used to extend parent boundaries" assert.
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::SetCursorPosX(x0);
            ImGui::Text("%zu", i + 1);
            ImGui::SameLine(0.0f, 0.0f);
            ImGui::SetCursorPosX(x0 + numW);
            DrawStepIconInline(StepIcon(s.type.c_str()));
            ImGui::TextUnformatted(text);
        }
        ImGui::PopID();

        if (executing || dim) ImGui::PopStyleColor();
        path.pop_back();
    }
}

// Tree toolbar: move within siblings, indent/outdent across container
// boundaries, delete. Adding steps lives in the palette column. Mutations
// address steps through the tree's path encoding, so selection follows the
// edited step.
void App::DrawRecipeTreeToolbar(Recipe& r, bool canEdit)
{
    std::size_t idx = 0;
    std::vector<RecipeStep>* parent =
        edStepSel_.empty() ? nullptr : FindStepParent(&r.steps, edStepSel_, &idx);

    // Indent target: the previous sibling, when it can hold children (same
    // rule as the palette's "into" mode).
    const RecipeStep* prev = (parent && idx > 0) ? &(*parent)[idx - 1] : nullptr;
    const bool canIndent =
        prev && IsStructuralStep(prev->type) && prev->type != "stop_recipe";
    const bool canOutdent = parent && edStepSel_.size() >= 2;
    // The four moves are single-row operations addressed through the anchor;
    // with more rows selected they would move one and leave the rest looking
    // as if they had gone too.
    const bool multi = !edStepMulti_.empty();

    ImGui::BeginDisabled(!canEdit);
    ImGui::BeginDisabled(multi);
    ImGui::BeginDisabled(!parent || idx == 0);
    if (ImGui::SmallButton("^")) {
        std::swap((*parent)[idx - 1], (*parent)[idx]);
        edStepSel_.back() -= 1;   // else-marker offset survives the +-1
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!parent || idx + 1 >= parent->size());
    if (ImGui::SmallButton("v")) {
        std::swap((*parent)[idx], (*parent)[idx + 1]);
        edStepSel_.back() += 1;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canIndent);
    if (ImGui::SmallButton(">")) {
        // Append to the previous sibling's children. The target reference
        // stays valid across the erase (it sits before the erased slot).
        RecipeStep& tgt = (*parent)[idx - 1];
        tgt.children.push_back(std::move((*parent)[idx]));
        parent->erase(parent->begin() + static_cast<std::ptrdiff_t>(idx));
        edStepSel_.back() -= 1;   // now addresses the container
        edStepSel_.push_back(tgt.children.size() - 1);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move into the loop/group/if directly above.\n"
                          "(An if's else branch: palette 'else' mode.)");
    ImGui::SameLine();
    ImGui::BeginDisabled(!canOutdent);
    if (ImGui::SmallButton("<")) {
        RecipeStep moved = std::move((*parent)[idx]);
        parent->erase(parent->begin() + static_cast<std::ptrdiff_t>(idx));
        edStepSel_.pop_back();    // now addresses the parent container
        std::size_t pIdx = 0;
        if (std::vector<RecipeStep>* gp =
                FindStepParent(&r.steps, edStepSel_, &pIdx)) {
            gp->insert(gp->begin() + static_cast<std::ptrdiff_t>(pIdx) + 1,
                       std::move(moved));
            edStepSel_.back() += 1;   // sibling AFTER the container
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Move out of the container, to just after it.");
    ImGui::EndDisabled();   // multi
    ImGui::SameLine();
    ImGui::BeginDisabled(!parent && !multi);
    if (ImGui::SmallButton("Delete")) edOpenDeleteStep_ = true;
    ImGui::EndDisabled();
    ImGui::EndDisabled();   // canEdit (reopened below Copy)
    ImGui::SameLine();
    // Copy is a read: it stays live while a run blocks editing, so steps can be
    // lifted out of a running recipe into another one.
    ImGui::BeginDisabled(!parent && !multi);
    if (ImGui::SmallButton("Copy")) CopySelectedSteps(r);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Copy the selected step(s) as JSON to the clipboard.\n"
                          "Ctrl-click adds a row, Shift-click takes a range.");
    ImGui::BeginDisabled(!canEdit);
    ImGui::SameLine();
    // No confirm on Cut, unlike Delete: the rows are on the clipboard and
    // Paste puts them back.
    ImGui::BeginDisabled(!parent && !multi);
    if (ImGui::SmallButton("Cut")) {
        CopySelectedSteps(r);
        DeleteSelectedSteps(r);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Copy the selected step(s) to the clipboard and remove them.");
    ImGui::SameLine();
    if (ImGui::SmallButton("Paste")) PasteSteps(r);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Paste steps from the clipboard at the palette's target\n"
                          "(after the selection / into it / its else branch).");
    ImGui::EndDisabled();   // canEdit
}

// Plain click: this row alone. Ctrl: toggle it in the multi set (the anchor
// hands over to the first extra row if it is the one toggled off). Shift:
// deferred to the end of the draw, where the visible rows are all known.
void App::ClickStepRow(const std::vector<std::size_t>& path)
{
    const ImGuiIO& io = ImGui::GetIO();
    if (io.KeyShift && !edStepSel_.empty()) {
        edShiftTo_      = path;
        edShiftPending_ = true;
    } else if (io.KeyCtrl && !edStepSel_.empty()) {
        if (path == edStepSel_) {
            if (!edStepMulti_.empty()) {
                edStepSel_ = edStepMulti_.front();
                edStepMulti_.erase(edStepMulti_.begin());
            }
            return;   // sole selected row: a Ctrl-click leaves it selected
        }
        const auto it = std::find(edStepMulti_.begin(), edStepMulti_.end(), path);
        if (it == edStepMulti_.end()) edStepMulti_.push_back(path);
        else                          edStepMulti_.erase(it);
    } else {
        edStepSel_ = path;
        edStepMulti_.clear();
    }
}

// Lexicographic order on the numeric path IS tree order: a container sorts
// before its children, and the else-branch offset puts an else row after
// every then row. A descendant of a kept row is dropped — the container's
// subtree already carries it.
std::vector<std::vector<std::size_t>> App::SelectedStepPaths() const
{
    std::vector<std::vector<std::size_t>> all;
    if (!edStepSel_.empty()) all.push_back(edStepSel_);
    all.insert(all.end(), edStepMulti_.begin(), edStepMulti_.end());
    std::sort(all.begin(), all.end());
    all.erase(std::unique(all.begin(), all.end()), all.end());
    std::vector<std::vector<std::size_t>> kept;
    for (const auto& p : all) {
        bool covered = false;
        for (const auto& k : kept) {
            if (k.size() < p.size() && std::equal(k.begin(), k.end(), p.begin())) {
                covered = true;
                break;
            }
        }
        if (!covered) kept.push_back(p);
    }
    return kept;
}

// The palette's one insert rule, for a batch: after the anchor as siblings,
// or appended to its children / else branch. What went in becomes the
// selection — first row the anchor, the rest in the multi set — so a paste
// reads back as exactly the rows it added.
void App::InsertStepsAtSelection(Recipe& r, std::vector<RecipeStep> steps)
{
    if (steps.empty()) return;
    const std::size_t n = steps.size();
    std::vector<std::vector<std::size_t>> added;
    if (edAddMode_ == 1) {          // children of the selected structural step
        RecipeStep* p = FindStepByPathMut(&r.steps, edStepSel_);
        if (!p) return;
        const std::size_t base = p->children.size();
        for (RecipeStep& s : steps) p->children.push_back(std::move(s));
        for (std::size_t k = 0; k < n; ++k) {
            added.push_back(edStepSel_);
            added.back().push_back(base + k);
        }
    } else if (edAddMode_ == 2) {   // else-branch of the selected if
        RecipeStep* p = FindStepByPathMut(&r.steps, edStepSel_);
        if (!p) return;
        const std::size_t base = p->elseChildren.size();
        for (RecipeStep& s : steps) p->elseChildren.push_back(std::move(s));
        for (std::size_t k = 0; k < n; ++k) {
            added.push_back(edStepSel_);
            added.back().push_back(kElsePathBase + base + k);
        }
    } else if (!edStepSel_.empty()) {   // siblings after the anchor
        std::size_t i = 0;
        std::vector<RecipeStep>* pl = FindStepParent(&r.steps, edStepSel_, &i);
        if (!pl) return;
        pl->insert(pl->begin() + static_cast<std::ptrdiff_t>(i) + 1,
                   std::make_move_iterator(steps.begin()),
                   std::make_move_iterator(steps.end()));
        for (std::size_t k = 0; k < n; ++k) {
            added.push_back(edStepSel_);
            added.back().back() += 1 + k;
        }
    } else {                        // nothing selected: append at root
        const std::size_t base = r.steps.size();
        for (RecipeStep& s : steps) r.steps.push_back(std::move(s));
        for (std::size_t k = 0; k < n; ++k) added.push_back({base + k});
    }
    edStepSel_ = added.front();
    edStepMulti_.assign(added.begin() + 1, added.end());
    edScrollToSel_ = true;
}

void App::CopySelectedSteps(const Recipe& r)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& p : SelectedStepPaths()) {
        if (const RecipeStep* s = FindStepByPath(&r.steps, p)) arr.push_back(StepToJson(*s));
    }
    if (arr.empty()) return;
    const std::size_t n = arr.size();
    nlohmann::json j;
    j["steps"] = std::move(arr);
    ImGui::SetClipboardText(j.dump(2).c_str());
    FD_INFO("recipe", "copied %zu step(s) to clipboard", n);
}

// Accepts the shape Copy writes ({"steps":[...]}), which is also a whole
// recipe's shape (so a recipe copied from the list pastes as its steps), a bare
// array of steps, or a single step object. Anything without a `type` is not a
// step and is skipped rather than pasted as an empty row.
void App::PasteSteps(Recipe& r)
{
    const char* clip = ImGui::GetClipboardText();
    nlohmann::json j = clip ? nlohmann::json::parse(clip, nullptr, false) : nlohmann::json();
    nlohmann::json arr = nlohmann::json::array();
    if (j.is_array()) arr = j;
    else if (j.is_object() && j.contains("steps") && j["steps"].is_array()) arr = j["steps"];
    else if (j.is_object() && j.contains("type")) arr.push_back(j);
    std::vector<RecipeStep> steps;
    for (const auto& sj : arr) {
        if (!sj.is_object() || !sj.contains("type")) continue;
        RecipeStep s = StepFromJson(sj);
        if (!s.type.empty()) steps.push_back(std::move(s));
    }
    if (steps.empty()) {
        FD_WARN("recipe", "paste: clipboard has no step JSON");
        return;
    }
    const std::size_t n = steps.size();
    InsertStepsAtSelection(r, std::move(steps));
    FD_INFO("recipe", "pasted %zu step(s) from clipboard", n);
}

// Last in tree order first: erasing a later row never shifts an earlier one,
// and SelectedStepPaths holds no ancestor of another.
std::size_t App::DeleteSelectedSteps(Recipe& r)
{
    const std::vector<std::vector<std::size_t>> paths = SelectedStepPaths();
    std::size_t gone = 0;
    for (auto it = paths.rbegin(); it != paths.rend(); ++it) {
        std::size_t idx = 0;
        if (std::vector<RecipeStep>* parent = FindStepParent(&r.steps, *it, &idx)) {
            parent->erase(parent->begin() + static_cast<std::ptrdiff_t>(idx));
            ++gone;
        }
    }
    ClearStepSel();
    return gone;
}

// Searchable add-step palette — its own scrolling column (the command list
// outgrew a popup). Insert target: after the selection, into a selected
// structural step, or a selected if-step's else branch.
void App::DrawRecipePalette(Recipe& r, bool canEdit)
{
    ImGui::BeginDisabled(!canEdit);
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##palSearch", "search...", edSearchBuf_,
                             sizeof(edSearchBuf_));

    const RecipeStep* sel =
        edStepSel_.empty() ? nullptr : FindStepByPath(&r.steps, edStepSel_);
    const bool canChild = sel && IsStructuralStep(sel->type) && sel->type != "stop_recipe";
    const bool canElse  = sel && sel->type == "if";
    if (!canChild && edAddMode_ == 1) edAddMode_ = 0;
    if (!canElse && edAddMode_ == 2) edAddMode_ = 0;
    if (ImGui::RadioButton("after", edAddMode_ == 0)) edAddMode_ = 0;
    ImGui::SameLine();
    ImGui::BeginDisabled(!canChild);
    if (ImGui::RadioButton("into", edAddMode_ == 1)) edAddMode_ = 1;
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canElse);
    if (ImGui::RadioButton("else", edAddMode_ == 2)) edAddMode_ = 2;
    ImGui::EndDisabled();
    ImGui::Separator();

    // Same target rule as Paste (InsertStepsAtSelection), one step at a time.
    const auto insert = [this, &r](const char* type) {
        std::vector<RecipeStep> one;
        one.push_back(MakeStep(type, orch_.registry));
        InsertStepsAtSelection(r, std::move(one));
    };

    ImGui::BeginChild("##palList");   // scrolls independently of the tree
    const bool searching = edSearchBuf_[0] != '\0';

    // Insert on DOUBLE-click only — a stray single click must not add steps
    // (same convention as the product list's double-click-to-load).
    //
    // The row is an EMPTY-labelled Selectable with the icon and the name laid
    // over it: an icon cannot go inside a Selectable's label, and the whole row
    // has to stay the double-click target. AllowOverlap so the overlaid text
    // does not steal the row's hover (the tooltip hangs off it).
    const auto drawEntry = [&](const char* name, const char* desc) {
        char id[96];
        std::snprintf(id, sizeof(id), "##%s", name);
        const float x0 = ImGui::GetCursorPosX();
        if (ImGui::Selectable(id, false, ImGuiSelectableFlags_AllowDoubleClick |
                                             ImGuiSelectableFlags_AllowOverlap) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            insert(name);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\n(double-click to insert)", desc);
        // Back onto the row just drawn, via SameLine so the LINE STAYS OPEN and
        // the last piece closes it normally. Rewinding with SetCursorScreenPos
        // instead leaves ImGui holding a hand-set cursor past the content
        // extents, and the final row of the child trips its
        // "SetCursorPos used to extend parent boundaries" assert.
        ImGui::SameLine(0.0f, 0.0f);
        ImGui::SetCursorPosX(x0);
        DrawStepIconInline(StepIcon(name));
        ImGui::TextUnformatted(name);
    };
    // One collapsible node per category (open by default; collapse state
    // persists for the session). While searching, matching categories are
    // forced open so results are always visible.
    const auto beginCat = [&](const char* cat) {
        if (searching) ImGui::SetNextItemOpen(true);
        return ImGui::TreeNodeEx(cat, ImGuiTreeNodeFlags_DefaultOpen |
                                          ImGuiTreeNodeFlags_SpanAvailWidth);
    };

    {   // Structure (engine-native step types)
        const bool catMatch = PaletteMatch("Structure", edSearchBuf_);
        bool any = catMatch;
        for (const StructuralStepInfo& t : kStructuralSteps) {
            if (any) break;
            any = PaletteMatch(t.type, edSearchBuf_);
        }
        if (any && beginCat("Structure")) {
            for (const StructuralStepInfo& t : kStructuralSteps) {
                if (catMatch || PaletteMatch(t.type, edSearchBuf_))
                    drawEntry(t.type, t.description);
            }
            ImGui::TreePop();
        }
    }

    // Registry commands, grouped by category (List() is category-ordered).
    const std::vector<const CommandSpec*> cmds = orch_.registry.List();
    for (std::size_t i = 0; i < cmds.size();) {
        const std::string& cat = cmds[i]->category;
        std::size_t end = i;
        while (end < cmds.size() && cmds[end]->category == cat) ++end;
        const bool catMatch = PaletteMatch(cat.c_str(), edSearchBuf_);
        bool any = catMatch;
        for (std::size_t k = i; k < end && !any; ++k)
            any = PaletteMatch(cmds[k]->name, edSearchBuf_);
        if (any && beginCat(cat.c_str())) {
            for (std::size_t k = i; k < end; ++k) {
                if (catMatch || PaletteMatch(cmds[k]->name, edSearchBuf_))
                    drawEntry(cmds[k]->name.c_str(), cmds[k]->description.c_str());
            }
            ImGui::TreePop();
        }
        i = end;
    }
    ImGui::EndChild();
    ImGui::EndDisabled();
}

// Top strip: Run/Stop/Save + state + executing step + progress (the
// validation-issues list is a separate band at the bottom of the page).
// Runs the recipe selected in the left panel through the engine
// (positions/maps copied at Start, so mid-run edits elsewhere in the UI
// can't disturb the run).
void App::DrawRecipeRunStrip(float height)
{
    const RecipeEngine::Status st = orch_.recipe.GetStatus();
    const bool running = orch_.recipe.Running();
    // The strip shares ONE engine with the Run page's program runner, so while
    // a program is running these controls are pointed at somebody else's run.
    // Step and Next would pause a program mid-recipe with no owner on screen to
    // release it - exactly how "I only changed page" ends as a failed
    // execution. Stop stays live (the operator must be able to stop the machine
    // from wherever they are) but it stops the PROGRAM, not one recipe out of
    // its queue.
    const bool progRun = orch_.programs.Active();
    ImGui::BeginChild("edRunStrip", ImVec2(0.0f, height), ImGuiChildFlags_Borders);

    // One row: state on the left, the run controls right-aligned. Errors get
    // their own line below so a long message never collides with the buttons.
    // Wider than the old 90 px: Run / Stop / Save carry an icon beside the
    // label now, and a fixed width that fitted the text alone would clip it.
    const ImGuiStyle& style = ImGui::GetStyle();
    const float btnW  = 110.0f;
    const float chkW  = ImGui::GetFrameHeight() + style.ItemInnerSpacing.x +
                        ImGui::CalcTextSize("Step").x;
    const float ctrlW = btnW * 5.0f + chkW + style.ItemSpacing.x * 5.0f;
    const float rightEdge =
        ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;

    ImGui::AlignTextToFramePadding();
    ImGui::Text("State: %s", RecipeEngine::StateName(st.state));
    {
        const DutSession::Info di = DutSession::Instance().GetInfo();
        ImGui::SameLine();
        ImGui::TextDisabled("| DUT: %s", di.open ? di.serial.c_str() : "none");
    }
    if (!st.currentStep.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("| %s", st.currentStep.c_str());
    }
    if (st.stepPaused) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.16f, 1.0f),
                           "| paused - next: %s", st.nextStep.c_str());
    }
    if (progRun) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.16f, 1.0f),
                           "| program run - drive it from the Run page (F1)");
    }

    ImGui::SameLine(rightEdge - ctrlW);
    // Step mode: Run starts paused at the selected step; Next executes one
    // leaf at a time. Toggling while a run is active pauses/resumes it live.
    ImGui::BeginDisabled(progRun);
    if (ImGui::Checkbox("Step", &edStepMode_) && running)
        orch_.recipe.SetStepMode(edStepMode_);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip(progRun
                              ? "A program run owns the engine (Run page, F1)"
                              : "Step mode: Run pauses at the selected step (or the\n"
                                "first step); Next executes one step at a time.\n"
                                "Uncheck to free-run from here; Stop exits.");
    ImGui::SameLine();
    const bool canRun = !orch_.Busy() && recipeSel_ >= 0 &&
                        recipeSel_ < static_cast<int>(session_.Recipes().size());
    ImGui::BeginDisabled(!canRun);
    if (IconButton(UiIcon("run_execute"), "Run", ImVec2(btnW, 0.0f))) {
        RecipeEngine::Env env = session_.MakeRecipeEnv();
        RecipeEngine::StartOpts opts;
        opts.stepMode = edStepMode_;
        if (edStepMode_) opts.startPath = edStepSel_;   // from the selection
        recipeStartErr_.clear();
        if (!orch_.recipe.Start(session_.Recipes()[recipeSel_], std::move(env),
                                 &recipeStartErr_, std::move(opts)))
            FD_WARN("recipe", "start rejected: %s", recipeStartErr_.c_str());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    // Run ONLY the selected row (children included), as a throwaway one-step
    // recipe through the same engine — same validation, stop, and progress.
    const RecipeStep* selStep =
        (canRun && !edStepSel_.empty())
            ? FindStepByPath(&session_.Recipes()[recipeSel_].steps, edStepSel_)
            : nullptr;
    ImGui::BeginDisabled(!selStep);
    if (ImGui::Button("Run Step", ImVec2(btnW, 0.0f)) && selStep) {
        Recipe one;
        one.uid  = "step-run";
        one.name = "step: " + (selStep->nickname.empty() ? AutoNickname(*selStep)
                                                         : selStep->nickname);
        one.steps.push_back(*selStep);
        one.steps.front().enabled = true;   // a disabled row can still be tested
        // Full Env, not just this step: sub_recipe references still resolve.
        RecipeEngine::Env env = session_.MakeRecipeEnv();
        recipeStartErr_.clear();
        if (!orch_.recipe.Start(one, std::move(env), &recipeStartErr_))
            FD_WARN("recipe", "step run rejected: %s", recipeStartErr_.c_str());
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Run only the selected step (its children included)");
    ImGui::SameLine();
    ImGui::BeginDisabled(!st.stepPaused || progRun);
    if (ImGui::Button("Next", ImVec2(btnW, 0.0f))) orch_.recipe.StepAdvance();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Execute the paused step, then pause again");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running && !progRun);
    if (IconButton(UiIcon("Stop"), "Stop", ImVec2(btnW, 0.0f))) {
        // Stopping ONE recipe out of a program's queue leaves the runner free
        // to start the next one against a machine the operator meant to halt.
        if (progRun) orch_.programs.Stop();
        else         orch_.recipe.RequestStop();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(orch_.Busy());
    if (IconButton(UiIcon("Save"), session_.RecipesDirty() ? "Save*" : "Save",
                   ImVec2(btnW, 0.0f)))
        session_.SaveRecipes();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered() && session_.RecipesDirty())
        ImGui::SetTooltip("Unsaved changes");
    if (!recipeStartErr_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           recipeStartErr_.c_str());
    } else if (!st.error.empty() && st.state != RecipeEngine::State::Running) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
    }
    if (st.leafTotal > 0) {
        float frac = static_cast<float>(st.leafDone) / static_cast<float>(st.leafTotal);
        if (frac > 1.0f) frac = 1.0f;   // while-loops can overshoot the estimate
        char overlay[48];
        std::snprintf(overlay, sizeof(overlay), "%zu / %zu", st.leafDone, st.leafTotal);
        ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), overlay);
    }
    ImGui::EndChild();
}

} // namespace firedog
