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
#include <cstdio>
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
    const bool haveSel = recipeSel_ >= 0 && recipeSel_ < static_cast<int>(session_.Recipes().size());

    // Live author-time validation of the selected recipe (WITHOUT the
    // devices-Connected requirement — that stays a run-start check, so
    // authoring offline doesn't drown in noise).
    std::vector<RecipeIssue> issues;
    if (haveSel)
        issues = ValidateRecipe(session_.Recipes()[recipeSel_], orch_.registry, hardware_,
                                session_.Positions(), session_.Maps(), session_.Aligns(),
                                session_.VisionModels(), session_.Recipes(), false);
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
    const float  rightW = 320.0f;

    // ---- Top: run controls + state + progress (panes fill the rest) ----
    DrawRecipeRunStrip(stripH);

    // ---- Left: recipe list + CRUD ----
    ImGui::BeginChild("edRecipes", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Recipes");
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::SmallButton("Reload")) {
        if (session_.RecipesDirty()) {
            edConfirmAction_ = 2;   // guard: reload discards unsaved edits
        } else {
            LoadRecipesFromDisk();
            FD_INFO("recipe", "%zu recipe(s) loaded", session_.Recipes().size());
        }
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(running);
    if (ImGui::SmallButton("New")) {
        Recipe r;
        r.uid  = GenerateUid();
        r.name = "new recipe";
        session_.Recipes().push_back(std::move(r));
        recipeSel_ = static_cast<int>(session_.Recipes().size()) - 1;
        edStepSel_.clear();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveSel);
    if (ImGui::SmallButton("Dup")) {
        Recipe r = session_.Recipes()[recipeSel_];
        r.uid  = GenerateUid();
        r.name += " copy";
        session_.Recipes().push_back(std::move(r));
        recipeSel_ = static_cast<int>(session_.Recipes().size()) - 1;
        edStepSel_.clear();
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
            edStepSel_.clear();
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
            edStepSel_.clear();
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Middle: searchable palette column + step tree ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    const float palW = 200.0f;
    ImGui::BeginChild("edMiddle", ImVec2(midW, paneH));
    if (haveSel) {
        Recipe& r = session_.Recipes()[recipeSel_];
        ImGui::BeginChild("edPalette", ImVec2(palW, 0.0f), ImGuiChildFlags_Borders);
        DrawRecipePalette(r, !running);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("edTree", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
        DrawRecipeTreeToolbar(r, !running);
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
                    edScrollToSel_ = true;
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow) && edStepSel_.size() > 1) {
                edStepSel_.pop_back();   // parent (also from an else-branch child)
                edScrollToSel_ = true;
            }
        }
        ImGui::EndChild();   // ##edTreeRows
        ImGui::EndChild();   // edTree
    } else {
        ImGui::TextDisabled("Select a recipe on the left.");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: inspector (step form, or recipe fields when no step) ----
    ImGui::BeginChild("edInspect", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    RecipeStep* sel = nullptr;
    if (haveSel && !edStepSel_.empty())
        sel = FindStepByPathMut(&session_.Recipes()[recipeSel_].steps, edStepSel_);
    if (sel) {
        DrawRecipeInspector(*sel, !running);
    } else if (haveSel) {
        Recipe& r = session_.Recipes()[recipeSel_];
        ImGui::BeginDisabled(running);
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
                !is.path.empty())
                edStepSel_ = is.path;
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
            edStepSel_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Step", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const RecipeStep* ds = (haveSel && !edStepSel_.empty())
                                   ? FindStepByPath(&session_.Recipes()[recipeSel_].steps, edStepSel_)
                                   : nullptr;
        ImGui::Text("Delete step '%s'%s?", ds ? ds->type.c_str() : "?",
                    ds && !ds->children.empty() ? " and its children" : "");
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveSel && ds) {
            std::size_t idx = 0;
            if (std::vector<RecipeStep>* parent =
                    FindStepParent(&session_.Recipes()[recipeSel_].steps, edStepSel_, &idx)) {
                parent->erase(parent->begin() + static_cast<std::ptrdiff_t>(idx));
                edStepSel_.clear();
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
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
        const bool selected   = (path == edStepSel_);
        const bool executing  = running && path == runPath;

        const std::string shown = s.nickname.empty() ? AutoNickname(s) : s.nickname;
        char label[160];
        std::snprintf(label, sizeof(label), "%zu  %s%s", i + 1, shown.c_str(),
                      s.enabled ? "" : "  [off]");

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
            const bool open = ImGui::TreeNodeEx("##node", flags, "%s", label);
            edTreeRows_.push_back(path);
            if (selected && edScrollToSel_) {
                ImGui::SetScrollHereY(0.35f);
                edScrollToSel_ = false;
            }
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
                edStepSel_ = path;
            if (open) {
                DrawRecipeTreeNodes(s.children, path, false, runPath, running);
                if (!s.elseChildren.empty()) {
                    ImGui::TextDisabled("else:");
                    DrawRecipeTreeNodes(s.elseChildren, path, true, runPath, running);
                }
                ImGui::TreePop();
            }
        } else {
            if (ImGui::Selectable(label, selected)) edStepSel_ = path;
            edTreeRows_.push_back(path);
            if (selected && edScrollToSel_) {
                ImGui::SetScrollHereY(0.35f);
                edScrollToSel_ = false;
            }
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

    ImGui::BeginDisabled(!canEdit);
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
    ImGui::SameLine();
    ImGui::BeginDisabled(!parent);
    if (ImGui::SmallButton("Delete")) edOpenDeleteStep_ = true;
    ImGui::EndDisabled();
    ImGui::EndDisabled();   // canEdit
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

    const auto insert = [this, &r](const char* type) {
        RecipeStep ns = MakeStep(type, orch_.registry);
        if (edAddMode_ == 1) {          // child of the selected structural step
            if (RecipeStep* p = FindStepByPathMut(&r.steps, edStepSel_)) {
                p->children.push_back(std::move(ns));
                edStepSel_.push_back(p->children.size() - 1);
            }
        } else if (edAddMode_ == 2) {   // else-branch of the selected if
            if (RecipeStep* p = FindStepByPathMut(&r.steps, edStepSel_)) {
                p->elseChildren.push_back(std::move(ns));
                edStepSel_.push_back(kElsePathBase + p->elseChildren.size() - 1);
            }
        } else if (!edStepSel_.empty()) {   // sibling after the selection
            std::size_t i = 0;
            if (std::vector<RecipeStep>* pl = FindStepParent(&r.steps, edStepSel_, &i)) {
                pl->insert(pl->begin() + static_cast<std::ptrdiff_t>(i) + 1,
                           std::move(ns));
                edStepSel_.back() += 1;
            }
        } else {                        // nothing selected: append at root
            r.steps.push_back(std::move(ns));
            edStepSel_ = {r.steps.size() - 1};
        }
    };

    ImGui::BeginChild("##palList");   // scrolls independently of the tree
    const bool searching = edSearchBuf_[0] != '\0';

    // Insert on DOUBLE-click only — a stray single click must not add steps
    // (same convention as the product list's double-click-to-load).
    const auto drawEntry = [&](const char* name, const char* desc) {
        if (ImGui::Selectable(name, false, ImGuiSelectableFlags_AllowDoubleClick) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            insert(name);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\n(double-click to insert)", desc);
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
    ImGui::BeginChild("edRunStrip", ImVec2(0.0f, height), ImGuiChildFlags_Borders);

    // One row: state on the left, the run controls right-aligned. Errors get
    // their own line below so a long message never collides with the buttons.
    const ImGuiStyle& style = ImGui::GetStyle();
    const float btnW  = 90.0f;
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

    ImGui::SameLine(rightEdge - ctrlW);
    // Step mode: Run starts paused at the selected step; Next executes one
    // leaf at a time. Toggling while a run is active pauses/resumes it live.
    if (ImGui::Checkbox("Step", &edStepMode_) && running)
        orch_.recipe.SetStepMode(edStepMode_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Step mode: Run pauses at the selected step (or the\n"
                          "first step); Next executes one step at a time.\n"
                          "Uncheck to free-run from here; Stop exits.");
    ImGui::SameLine();
    const bool canRun = !orch_.Busy() && recipeSel_ >= 0 &&
                        recipeSel_ < static_cast<int>(session_.Recipes().size());
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Run", ImVec2(90.0f, 0.0f))) {
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
    if (ImGui::Button("Run Step", ImVec2(90.0f, 0.0f)) && selStep) {
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
    ImGui::BeginDisabled(!st.stepPaused);
    if (ImGui::Button("Next", ImVec2(90.0f, 0.0f))) orch_.recipe.StepAdvance();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Execute the paused step, then pause again");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(90.0f, 0.0f))) orch_.recipe.RequestStop();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::Button(session_.RecipesDirty() ? "Save*" : "Save", ImVec2(90.0f, 0.0f)))
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
