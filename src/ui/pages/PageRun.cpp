// PageRun.cpp : the Run page (View menu, full-page) — the program deck, the
// execution view, the Scan band (Raster | XCAN) and the scan-preset modals.
// Definitions of App members — see App.h. Design: docs/run-page-design.md.
//
// Execution adds NO engine: StartDeckRun resolves one queue from the deck
// selection (program / slot / recipe) and ProgramRunner sequences ordinary
// RecipeEngine runs from it, so each recipe keeps its own validation,
// progress, prompts and $run DUT event. Ctx does NOT flow between recipes.
//
// The two scan launchers inject their form values as the scripts' ctx keys via
// LuaEngine::Env::initialCtx — the same channel recipe ctx_set steps use. Exit
// targets are entered in uA and multiplied by 1e-6 into feedback units here.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/DutSession.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace firedog {

// mode: 0 = whole program, 1 = selected slot, 2 = selected recipe.
// ---- Run page (View menu, full page) — programs = slots of recipes ----------
// Operator surface: deck on the left (slots + recipes, selectable), execution
// in the middle (queue with marks + progress), DUT on the right. Three run
// granularities fill the SAME runner queue; one Stop kills the whole chain.
// Design: docs/run-page-design.md.

// Raster first-light exit levels, in uA (feedback amps; injected x1e-6).
static constexpr double kRasterTargetsUa[] = {5.0, 20.0};

// Built-in XCAN target presets: the three product power classes (the
// zone_low must sit AT the raster handoff level — see the micro-lobe field
// note in hexapod_align.lua); slots 4-5 are spares the operator can switch
// on in the target settings modal (right-click a Target button).
static XcanTarget DefaultXcanTarget(int i)
{
    XcanTarget t;
    switch (i) {
    case 0: t.targetUa = 150.0;  t.zoneLowUa = 10.0;  t.zoneHighUa = 200.0;
            t.enabled = true; break;
    case 1: t.targetUa = 400.0;  t.zoneLowUa = 20.0;  t.zoneHighUa = 400.0;
            t.enabled = true; break;
    case 2: t.targetUa = 3400.0; t.zoneLowUa = 200.0; t.zoneHighUa = 3400.0;
            t.enabled = true; break;
    default: break;   // spare slot, struct defaults, hidden
    }
    return t;
}

// Target buttons are labeled by their value ("400 uA").
static std::string XcanTargetLabel(const XcanTarget& t)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g uA", t.targetUa);
    return buf;
}

// Built-in XCAN launch presets: slots 1/2 are bobcat's Coarse/Fine ladders
// (enabled), 3-5 are blank spares the operator can switch on in the
// settings modal (right-click any preset button).
static XcanPreset DefaultXcanPreset(int i)
{
    XcanPreset p;
    switch (i) {
    case 0:  p.label = "Coarse"; p.stepsUm = "10,5,2,1"; p.enabled = true; break;
    case 1:  p.label = "Fine";   p.stepsUm = "2,1,0.5";  p.enabled = true; break;
    default: p.label = "P" + std::to_string(i + 1);      p.enabled = false; break;
    }
    return p;
}

static nlohmann::json XcanPresetToJson(const XcanPreset& p)
{
    return nlohmann::json{
        {"label", p.label},
        {"enabled", p.enabled},
        {"steps_um", p.stepsUm},
        {"axes", p.axes},
        {"settle_ms", p.settleMs},
        {"settle_min_ms", p.settleMinMs},
        {"fresh_samples", p.freshSamples},
        {"avg_count", p.avgCount},
        {"consec_limit", p.consecLimit},
        {"sig_drop_pct", p.sigDropPct},
        {"max_travel_um", p.maxTravelUm},
        {"improve_pct", p.improvePct},
        {"min_signal", p.minSignal},
        {"tol_mm", p.tolMm},
        {"move_timeout_ms", p.moveTimeoutMs},
        {"velocity_mm_s", p.velocityMmS},
        {"skip_opposite", p.skipOpposite},
        {"dir_memory", p.dirMemory},
        {"reuse_sample", p.reuseSample},
    };
}

// Tolerant overlay: missing keys keep the preset's current (default) value.
static void XcanPresetFromJson(const nlohmann::json& j, XcanPreset& p)
{
    p.label         = j.value("label", p.label);
    p.enabled       = j.value("enabled", p.enabled);
    p.stepsUm       = j.value("steps_um", p.stepsUm);
    p.axes          = j.value("axes", p.axes);
    p.settleMs      = j.value("settle_ms", p.settleMs);
    p.settleMinMs   = j.value("settle_min_ms", p.settleMinMs);
    p.freshSamples  = j.value("fresh_samples", p.freshSamples);
    p.avgCount      = j.value("avg_count", p.avgCount);
    p.consecLimit   = j.value("consec_limit", p.consecLimit);
    p.sigDropPct    = j.value("sig_drop_pct", p.sigDropPct);
    p.maxTravelUm   = j.value("max_travel_um", p.maxTravelUm);
    p.improvePct    = j.value("improve_pct", p.improvePct);
    p.minSignal     = j.value("min_signal", p.minSignal);
    p.tolMm         = j.value("tol_mm", p.tolMm);
    p.moveTimeoutMs = j.value("move_timeout_ms", p.moveTimeoutMs);
    p.velocityMmS   = j.value("velocity_mm_s", p.velocityMmS);
    p.skipOpposite  = j.value("skip_opposite", p.skipOpposite);
    p.dirMemory     = j.value("dir_memory", p.dirMemory);
    p.reuseSample   = j.value("reuse_sample", p.reuseSample);
}

// Activatable preset button: theme color normally, yellow when active.
static bool PresetButton(const char* label, bool active, const ImVec2& size)
{
    if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.68f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.78f, 0.18f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.00f, 0.85f, 0.25f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.08f, 0.08f, 0.08f, 1.0f));
    }
    const bool clicked = ImGui::Button(label, size);
    if (active) ImGui::PopStyleColor(4);
    return clicked;
}

// SI-prefixed reading for the scan header (bobcat FormatSI: 2.09e-05 -> 20.888u).
static std::string FormatSI(double v)
{
    if (v == 0.0) return "0";
    const double a = std::fabs(v);
    const char*  suf;
    double       s;
    if      (a < 1e-9) { suf = "p"; s = v * 1e12; }
    else if (a < 1e-6) { suf = "n"; s = v * 1e9;  }
    else if (a < 1e-3) { suf = "u"; s = v * 1e6;  }
    else if (a < 1.0)  { suf = "m"; s = v * 1e3;  }
    else               { suf = "";  s = v;        }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f%s", s, suf);
    return buf;
}

void App::StartDeckRun(int mode)
{
    runStartErr_.clear();
    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) {
        runStartErr_ = "no program selected";
        return;
    }
    const Program& prog = session_.Programs()[progSel_];

    std::vector<ProgramRunner::Item> queue;
    const auto addRecipe = [&](const std::string& slotName, const std::string& uid) {
        const Recipe* r = RecipeByUid(session_.Recipes(), uid);
        if (!r) {
            runStartErr_ = "recipe uid '" + uid + "' not found (slot '" + slotName + "')";
            return false;
        }
        queue.push_back({slotName, *r});
        return true;
    };

    std::string label = prog.name;
    std::string kind  = "program";
    if (mode == 0) {
        for (const ProgramSlot& s : prog.slots)
            for (const std::string& uid : s.recipes)
                if (!addRecipe(s.name, uid)) return;
    } else {
        if (deckSlot_ < 0 || deckSlot_ >= static_cast<int>(prog.slots.size())) {
            runStartErr_ = "no slot selected";
            return;
        }
        const ProgramSlot& s = prog.slots[deckSlot_];
        if (mode == 1) {
            kind  = "slot";
            label = prog.name + " / " + s.name;
            for (const std::string& uid : s.recipes)
                if (!addRecipe(s.name, uid)) return;
        } else {
            if (deckRecipe_ < 0 || deckRecipe_ >= static_cast<int>(s.recipes.size())) {
                runStartErr_ = "no recipe selected";
                return;
            }
            kind = "recipe";
            if (!addRecipe(s.name, s.recipes[deckRecipe_])) return;
            label = prog.name + " / " + s.name + " / " + queue.front().recipe.name;
        }
    }

    RecipeEngine::Env env = session_.MakeRecipeEnv();
    if (!orch_.programs.Start(kind, label, prog.uid, std::move(queue), std::move(env),
                              &runStartErr_))
        FD_WARN("program", "run rejected: %s", runStartErr_.c_str());
}

void App::DrawRunPanel()
{
    const ProgramRunner::Status& rs = orch_.programs.GetStatus();
    const RecipeEngine::Status   es = orch_.recipe.GetStatus();
    const bool running = orch_.programs.Active();

    const ImGuiStyle& style  = ImGui::GetStyle();
    const float  stripH = ImGui::GetFrameHeight() + style.WindowPadding.y * 2.0f +
                          style.ChildBorderSize * 2.0f;
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  paneH  = avail.y - stripH - style.ItemSpacing.y;
    const float  leftW  = 300.0f;
    const float  rightW = 280.0f;

    const bool haveProg = progSel_ >= 0 && progSel_ < static_cast<int>(session_.Programs().size());

    // ---- Top strip: run controls ----
    ImGui::BeginChild("runStrip", ImVec2(0.0f, stripH), ImGuiChildFlags_Borders);
    const bool canRun = !orch_.Busy() && haveProg;
    // One Start button; granularity follows the deck selection
    // (recipe if one is selected, else slot, else the whole program).
    // While a run is active it becomes the Stop button.
    const bool onRecipe = deckSlot_ >= 0 && deckRecipe_ >= 0;
    const bool onSlot   = deckSlot_ >= 0 && deckRecipe_ < 0;
    if (running) {
        if (ImGui::Button("Stop", ImVec2(110.0f, 0.0f))) orch_.programs.Stop();
    } else {
        ImGui::BeginDisabled(!canRun);
        if (ImGui::Button("Start", ImVec2(110.0f, 0.0f)))
            StartDeckRun(onRecipe ? 2 : onSlot ? 1 : 0);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(onRecipe ? "Run the selected recipe"
                              : onSlot ? "Run the selected slot"
                                       : "Run the whole program");
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    ImGui::TextDisabled(onRecipe ? "recipe" : onSlot ? "slot" : "program");
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::Button(session_.ProgramsDirty() ? "Save*" : "Save", ImVec2(90.0f, 0.0f)))
        session_.SavePrograms();
    ImGui::EndDisabled();
    {
        const DutSession::Info di = DutSession::Instance().GetInfo();
        ImGui::SameLine();
        ImGui::TextDisabled("DUT: %s", di.open ? di.serial.c_str() : "none");
    }
    if (!runStartErr_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", runStartErr_.c_str());
    }
    ImGui::EndChild();

    // ---- Left: the deck ----
    ImGui::BeginChild("runDeck", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    ImGui::BeginDisabled(running);
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("##prog", haveProg ? session_.Programs()[progSel_].name.c_str()
                                             : "(program)")) {
        for (int i = 0; i < static_cast<int>(session_.Programs().size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(session_.Programs()[i].name.c_str(), progSel_ == i)) {
                progSel_    = i;
                deckSlot_   = -1;
                deckRecipe_ = -1;
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Edit", &runEdit_);
    // Active slot: mirrors the tree selection (deckSlot_, also the Start
    // granularity); in Edit mode the deck shows ONLY the active slot, so a
    // big program edits one slot at a time. "(all slots)" = whole program.
    {
        Program* prog = haveProg ? &session_.Programs()[progSel_] : nullptr;
        if (prog && deckSlot_ >= static_cast<int>(prog->slots.size()))
            deckSlot_ = -1;   // stale index (slot deleted) — back to all
        char prev[64];
        if (prog && deckSlot_ >= 0)
            std::snprintf(prev, sizeof(prev), "%d  %s", deckSlot_ + 1,
                          prog->slots[deckSlot_].name.c_str());
        else
            std::snprintf(prev, sizeof(prev), "(all slots)");
        ImGui::BeginDisabled(!haveProg);
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo("##actslot", prev)) {
            if (ImGui::Selectable("(all slots)", deckSlot_ < 0)) {
                deckSlot_   = -1;
                deckRecipe_ = -1;
            }
            for (int i = 0; prog && i < static_cast<int>(prog->slots.size()); ++i) {
                ImGui::PushID(i);
                char lbl[64];
                std::snprintf(lbl, sizeof(lbl), "%d  %s", i + 1,
                              prog->slots[i].name.c_str());
                if (ImGui::Selectable(lbl, deckSlot_ == i)) {
                    deckSlot_   = i;
                    deckRecipe_ = -1;
                }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("slot");
        ImGui::EndDisabled();
    }
    if (runEdit_) {
        if (ImGui::SmallButton("New")) {
            Program p;
            p.uid = GenerateUid();   // the traceability identity, minted once
            session_.Programs().push_back(std::move(p));
            progSel_    = static_cast<int>(session_.Programs().size()) - 1;
            deckSlot_   = -1;
            deckRecipe_ = -1;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!haveProg);
        if (ImGui::SmallButton("Del")) runOpenDelete_ = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("+ Slot")) {
            session_.Programs()[progSel_].slots.push_back({});
            // The new slot becomes active so Edit's filtered view shows it.
            deckSlot_   = static_cast<int>(session_.Programs()[progSel_].slots.size()) - 1;
            deckRecipe_ = -1;
        }
        ImGui::EndDisabled();
        if (haveProg) {
            Program& prog = session_.Programs()[progSel_];
            char nbuf[64];
            std::snprintf(nbuf, sizeof(nbuf), "%s", prog.name.c_str());
            ImGui::SetNextItemWidth(-40.0f);
            if (ImGui::InputText("name", nbuf, sizeof(nbuf))) prog.name = nbuf;
            ImGui::TextDisabled("uid: %s", prog.uid.c_str());
        }
    }
    ImGui::EndDisabled();
    ImGui::Separator();

    if (!haveProg) {
        ImGui::TextWrapped(session_.Programs().empty()
                               ? "No programs yet - Edit > New creates one."
                               : "Pick a program above.");
    } else {
        Program& prog = session_.Programs()[progSel_];
        for (int si = 0; si < static_cast<int>(prog.slots.size()); ++si) {
            // An active slot filters the tree to just that slot (Edit or
            // not); "(all slots)" in the combo shows the whole program.
            if (deckSlot_ >= 0 && si != deckSlot_) continue;
            ProgramSlot& slot = prog.slots[si];
            ImGui::PushID(si);
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                       ImGuiTreeNodeFlags_SpanAvailWidth |
                                       ImGuiTreeNodeFlags_DefaultOpen;
            if (deckSlot_ == si && deckRecipe_ < 0) flags |= ImGuiTreeNodeFlags_Selected;
            const bool open = ImGui::TreeNodeEx("##slot", flags, "%d  %s", si + 1,
                                                slot.name.c_str());
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
                deckSlot_   = si;
                deckRecipe_ = -1;
            }
            if (open) {
                if (runEdit_ && !running) {
                    char sbuf[48];
                    std::snprintf(sbuf, sizeof(sbuf), "%s", slot.name.c_str());
                    ImGui::SetNextItemWidth(120.0f);
                    if (ImGui::InputText("##sname", sbuf, sizeof(sbuf))) slot.name = sbuf;
                    ImGui::SameLine();
                    ImGui::BeginDisabled(si == 0);
                    if (ImGui::SmallButton("^")) {
                        std::swap(prog.slots[si - 1], prog.slots[si]);
                        // The active slot follows the moved item (the Edit
                        // view filters on it — don't let the view jump).
                        if (deckSlot_ == si) deckSlot_ = si - 1;
                        else if (deckSlot_ == si - 1) deckSlot_ = si;
                    }
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    ImGui::BeginDisabled(si + 1 >= static_cast<int>(prog.slots.size()));
                    if (ImGui::SmallButton("v")) {
                        std::swap(prog.slots[si], prog.slots[si + 1]);
                        if (deckSlot_ == si) deckSlot_ = si + 1;
                        else if (deckSlot_ == si + 1) deckSlot_ = si;
                    }
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    if (ImGui::SmallButton("x slot")) {
                        prog.slots.erase(prog.slots.begin() + si);
                        deckSlot_ = -1;
                        deckRecipe_ = -1;
                        ImGui::TreePop();
                        ImGui::PopID();
                        break;   // container changed; redraw next frame
                    }
                }
                for (int ri = 0; ri < static_cast<int>(slot.recipes.size()); ++ri) {
                    ImGui::PushID(ri);
                    const Recipe* r = RecipeByUid(session_.Recipes(), slot.recipes[ri]);
                    const bool missing = (r == nullptr);
                    const bool sel = deckSlot_ == si && deckRecipe_ == ri;
                    // Gold while this exact item runs.
                    const bool cur = running && rs.index < rs.items.size() &&
                                     rs.items[rs.index].slotName == slot.name &&
                                     !missing && rs.items[rs.index].recipeName == r->name;
                    if (cur)
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.3f, 1.0f));
                    else if (missing)
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.3f, 1.0f));
                    char row[96];
                    std::snprintf(row, sizeof(row), "   %s", missing
                                      ? (slot.recipes[ri] + " (missing!)").c_str()
                                      : r->name.c_str());
                    // AllowOverlap: the selectable's hit rect spans the full
                    // row, UNDER the trailing ^/v/x buttons — without this
                    // flag it claims hover first and the buttons never click.
                    if (ImGui::Selectable(row, sel, ImGuiSelectableFlags_AllowOverlap)) {
                        deckSlot_   = si;
                        deckRecipe_ = ri;
                    }
                    // Right-click shortcut: open this recipe in the Recipe Editor.
                    if (!missing && ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                        deckSlot_   = si;
                        deckRecipe_ = ri;
                        for (int k = 0; k < static_cast<int>(session_.Recipes().size()); ++k)
                            if (session_.Recipes()[k].uid == slot.recipes[ri]) { recipeSel_ = k; break; }
                        edStepSel_.clear();
                        prevPage_    = Page::Run;   // editor Close comes back here
                        currentPage_ = Page::RecipeEdit;
                    }
                    if (!missing && ImGui::IsItemHovered())
                        ImGui::SetTooltip("Right-click: open in Recipe Editor");
                    if (cur || missing) ImGui::PopStyleColor();
                    if (runEdit_ && !running) {
                        ImGui::SameLine();
                        ImGui::BeginDisabled(ri == 0);
                        if (ImGui::SmallButton("^"))
                            std::swap(slot.recipes[ri - 1], slot.recipes[ri]);
                        ImGui::EndDisabled();
                        ImGui::SameLine();
                        ImGui::BeginDisabled(ri + 1 >= static_cast<int>(slot.recipes.size()));
                        if (ImGui::SmallButton("v"))
                            std::swap(slot.recipes[ri], slot.recipes[ri + 1]);
                        ImGui::EndDisabled();
                        ImGui::SameLine();
                        if (ImGui::SmallButton("x")) {
                            slot.recipes.erase(slot.recipes.begin() + ri);
                            if (deckSlot_ == si) {
                                if (deckRecipe_ == ri) deckRecipe_ = -1;
                                else if (deckRecipe_ > ri) --deckRecipe_;
                            }
                            ImGui::PopID();
                            break;
                        }
                    }
                    ImGui::PopID();
                }
                if (runEdit_ && !running) {
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::BeginCombo("##addrcp", "+ recipe")) {
                        for (const Recipe& r : session_.Recipes()) {
                            if (ImGui::Selectable(r.name.c_str()))
                                slot.recipes.push_back(r.uid);
                        }
                        ImGui::EndCombo();
                    }
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Middle column: execution view / XCAN presets / log ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    ImGui::BeginChild("runMid", ImVec2(midW, paneH));
    const float scanH = ImGui::GetFrameHeightWithSpacing() * 6.0f +   // channel/target/tabs/3 content rows
                        ImGui::GetStyle().WindowPadding.y * 2.0f;
    const float logH  = 190.0f;
    float execH = ImGui::GetContentRegionAvail().y - scanH - logH -
                  ImGui::GetStyle().ItemSpacing.y * 2.0f;
    if (execH < 60.0f) execH = 60.0f;
    ImGui::BeginChild("runExec", ImVec2(0.0f, execH), ImGuiChildFlags_Borders);
    if (running || !rs.result.empty()) {
        ImGui::Text("%s: %s", rs.kind.c_str(), rs.label.c_str());
        if (running) {
            ImGui::Text("recipe %zu / %zu", rs.index + 1, rs.total);
            if (!es.currentStep.empty()) ImGui::TextDisabled("%s", es.currentStep.c_str());
            if (es.leafTotal > 0) {
                float frac = static_cast<float>(es.leafDone) /
                             static_cast<float>(es.leafTotal);
                if (frac > 1.0f) frac = 1.0f;
                ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f));
            }
        } else {
            const bool ok = rs.result.rfind("Completed", 0) == 0;
            ImGui::TextColored(ok ? ImVec4(0.4f, 1.0f, 0.5f, 1.0f)
                                  : ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                               "%s", rs.result.c_str());
        }
        ImGui::Separator();
        for (const ProgramRunner::ItemStatus& it : rs.items) {
            const char* mark = "  ";
            ImVec4 col = ImGui::GetStyle().Colors[ImGuiCol_TextDisabled];
            switch (it.mark) {
            case ProgramRunner::Mark::Running: mark = "> "; col = ImVec4(1.0f, 0.85f, 0.3f, 1.0f); break;
            case ProgramRunner::Mark::Done:    mark = "OK"; col = ImVec4(0.4f, 1.0f, 0.5f, 1.0f); break;
            case ProgramRunner::Mark::Failed:  mark = "X "; col = ImVec4(1.0f, 0.4f, 0.3f, 1.0f); break;
            case ProgramRunner::Mark::Stopped: mark = "! "; col = ImVec4(1.0f, 0.7f, 0.3f, 1.0f); break;
            case ProgramRunner::Mark::Pending: break;
            }
            ImGui::TextColored(col, "%s  %s / %s", mark, it.slotName.c_str(),
                               it.recipeName.c_str());
        }
    } else {
        ImGui::TextDisabled("Select a slot or recipe on the left, then Run.");
        if (haveProg && !session_.Programs()[progSel_].description.empty()) {
            ImGui::Separator();
            ImGui::TextWrapped("%s", session_.Programs()[progSel_].description.c_str());
        }
    }
    ImGui::EndChild();

    // Scan band: shared channel/Target/Trace header over Raster | XCAN tabs.
    ImGui::BeginChild("runScan", ImVec2(0.0f, scanH), ImGuiChildFlags_Borders);
    DrawRunScanBlock();
    ImGui::EndChild();

    // Log band (same viewer as the Overview lower panel).
    ImGui::BeginChild("runLog", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    DrawLogPanel();
    ImGui::EndChild();

    ImGui::EndChild();   // runMid
    ImGui::SameLine();

    // ---- Right: DUT (the operator flow is scan -> run -> pass/fail) ----
    ImGui::BeginChild("runDut", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    DrawDutBlock();
    ImGui::Separator();
    ImGui::Text("Engine: %s", RecipeEngine::StateName(es.state));
    if (!es.error.empty() && es.state != RecipeEngine::State::Running)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", es.error.c_str());
    ImGui::EndChild();

    // ---- Delete-program confirm ----
    if (runOpenDelete_) {
        ImGui::OpenPopup("Delete Program");
        runOpenDelete_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Program", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        if (haveProg)
            ImGui::Text("Delete program '%s'?", session_.Programs()[progSel_].name.c_str());
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveProg) {
            session_.Programs().erase(session_.Programs().begin() + progSel_);
            progSel_    = session_.Programs().empty() ? -1 : 0;
            deckSlot_   = -1;
            deckRecipe_ = -1;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void App::LoadScanPresets()
{
    for (int i = 0; i < kXcanPresetCount; ++i) {
        xcanPresets_[i] = DefaultXcanPreset(i);
        xcanTargets_[i] = DefaultXcanTarget(i);
    }
    std::ifstream f("config/scan_presets.local.json");
    if (!f.is_open()) f.open("config/scan_presets.json");
    if (!f.is_open()) return;   // first run — defaults stand
    const nlohmann::json j = nlohmann::json::parse(f, nullptr, false);
    if (!j.is_object()) return;
    if (j.contains("xcan_presets") && j["xcan_presets"].is_array()) {
        const nlohmann::json& arr = j["xcan_presets"];
        for (std::size_t i = 0;
             i < arr.size() && i < static_cast<std::size_t>(kXcanPresetCount); ++i) {
            if (arr[i].is_object())
                XcanPresetFromJson(arr[i], xcanPresets_[static_cast<int>(i)]);
        }
    }
    if (j.contains("xcan_targets") && j["xcan_targets"].is_array()) {
        const nlohmann::json& arr = j["xcan_targets"];
        for (std::size_t i = 0;
             i < arr.size() && i < static_cast<std::size_t>(kXcanPresetCount); ++i) {
            if (!arr[i].is_object()) continue;
            XcanTarget& t = xcanTargets_[static_cast<int>(i)];
            t.enabled    = arr[i].value("enabled", t.enabled);
            t.targetUa   = arr[i].value("target_ua", t.targetUa);
            t.zoneLowUa  = arr[i].value("zone_low_ua", t.zoneLowUa);
            t.zoneHighUa = arr[i].value("zone_high_ua", t.zoneHighUa);
        }
    }
}

void App::SaveScanPresets() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const XcanPreset& p : xcanPresets_) arr.push_back(XcanPresetToJson(p));
    nlohmann::json tarr = nlohmann::json::array();
    for (const XcanTarget& t : xcanTargets_) {
        tarr.push_back(nlohmann::json{
            {"enabled", t.enabled},
            {"target_ua", t.targetUa},
            {"zone_low_ua", t.zoneLowUa},
            {"zone_high_ua", t.zoneHighUa},
        });
    }
    nlohmann::json j;
    j["xcan_presets"] = std::move(arr);
    j["xcan_targets"] = std::move(tarr);
    std::ofstream f("config/scan_presets.local.json");
    if (f.is_open()) {
        f << j.dump(2) << "\n";
        FD_INFO("ui", "scan presets saved to config/scan_presets.local.json");
    } else {
        FD_WARN("ui", "could not save config/scan_presets.local.json");
    }
}

// XCAN preset launcher (bobcat Run page scan controls): hexapod_align.lua with
// the pressed launch preset (step ladder + tunables, right-click to edit) +
// the ACTIVE target preset (exit threshold + adaptive zone range). Uses the
// same product-script override and Env wiring as the Scanning tab.
void App::StartXcanPreset(const std::string& device, int presetIdx)
{
    std::string script = "config/scripts/hexapod_align.lua";
    for (const ScriptInfo& si : session_.Scripts()) {
        if (si.name == "hexapod_align") { script = si.path; break; }
    }
    const XcanPreset& p  = xcanPresets_[presetIdx];
    const XcanTarget& tp = xcanTargets_[xcanPresetSel_];
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"align_device", device},
        {"align_topic", std::string(runXcanTopic_)},
        {"align_steps_um", p.stepsUm},
        {"align_axes", p.axes},
        {"align_settle_ms", p.settleMs},
        {"align_settle_min_ms", p.settleMinMs},
        {"align_fresh_samples", static_cast<double>(p.freshSamples)},
        {"align_avg_count", static_cast<double>(p.avgCount)},
        {"align_consec_limit", static_cast<double>(p.consecLimit)},
        {"align_sig_drop_pct", p.sigDropPct},
        {"align_max_travel_um", p.maxTravelUm},
        {"align_improve_pct", p.improvePct},
        {"align_min_signal", p.minSignal},
        {"align_tol_mm", p.tolMm},
        {"align_move_timeout_ms", p.moveTimeoutMs},
        {"align_velocity_mm_s", p.velocityMmS},
        {"align_skip_opposite", p.skipOpposite ? 1.0 : 0.0},
        {"align_dir_memory", p.dirMemory ? 1.0 : 0.0},
        {"align_reuse_sample", p.reuseSample ? 1.0 : 0.0},
        {"align_trace", xcanTrace_ ? 1.0 : 0.0},
        {"align_exit_threshold", tp.targetUa * 1e-6},
        {"align_adaptive", 1.0},
        {"align_zone_low", tp.zoneLowUa * 1e-6},
        {"align_zone_high", tp.zoneHighUa * 1e-6},
    };
    scriptStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
        FD_WARN("script", "xcan preset '%s' rejected: %s", p.label.c_str(),
                scriptStartErr_.c_str());
}

// The Run page's Scan band: ONE shared header — feedback-channel picker +
// live value + Trace — over two tabs: Raster (first-light area scan) and
// XCAN (align hill climb). Each tab owns its target preset buttons.
void App::DrawRunScanBlock()
{
    // Shared header row: channel picker (live DataBus topics) + current value.
    ImGui::SetNextItemWidth(150.0f);
    if (ImGui::BeginCombo("##scanch", runXcanTopic_)) {
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (topics.empty()) ImGui::TextDisabled("(no channels published yet)");
        for (const std::string& t : topics) {
            if (ImGui::Selectable(t.c_str(), t == runXcanTopic_))
                std::snprintf(runXcanTopic_, sizeof(runXcanTopic_), "%s", t.c_str());
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    {
        std::vector<float> t, v;
        DataBus::Instance().Stream(runXcanTopic_).Snapshot(t, v);
        if (!v.empty()) {
            // Keithley topics publish amps; other topics get the bare SI prefix.
            const bool amps =
                std::string(runXcanTopic_).compare(0, 9, "keithley/") == 0;
            ImGui::Text("= %s%s", FormatSI(static_cast<double>(v.back())).c_str(),
                        amps ? "A" : "");
        } else {
            ImGui::TextDisabled("= (no data)");
        }
    }

    // Targets live per-tab now (activatable preset buttons); the header keeps
    // only the shared channel + Trace. Trace sits at a FIXED column so the
    // varying width of the live reading doesn't shove it around.
    ImGui::SameLine(150.0f + ImGui::GetStyle().ItemSpacing.x +
                    ImGui::CalcTextSize("= -888.888mA  ").x);
    ImGui::Checkbox("Trace", &xcanTrace_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Log every scan sample as 'xcan,'/'rast,' CSV lines\n"
                          "(also in the daily log file).");

    // One tab per scan method (Raster first: the physical workflow is
    // raster to first light, then XCAN to the peak).
    if (ImGui::BeginTabBar("##scantabs")) {
        if (ImGui::BeginTabItem("Raster")) {
            DrawRunRasterBlock();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("XCAN")) {
            DrawRunXcanBlock();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

// XCAN tab of the Scan band: target/zone preset buttons (one active, yellow)
// + bobcat step presets — Coarse (10,5,2,1 um) / Fine (2,1,0.5 um) per
// hexapod. Channel/Trace live in the shared Scan-band header.
void App::DrawRunXcanBlock()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    // Target preset row: exit target + adaptive zone range per product power
    // class; the ACTIVE (yellow) preset is injected into every launch.
    // Left-click activates, RIGHT-click edits in the target settings modal.
    ImGui::TextDisabled("Target");
    bool anyTarget = false;
    for (int i = 0; i < kXcanPresetCount; ++i) {
        const XcanTarget& t = xcanTargets_[i];
        if (!t.enabled) continue;
        anyTarget = true;
        ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(XcanTargetLabel(t).c_str(), xcanPresetSel_ == i,
                         ImVec2(76.0f, 0.0f)))
            xcanPresetSel_ = i;
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Exit target %g uA, adaptive zones %g-%g uA\n"
                              "right-click: settings",
                              t.targetUa, t.zoneLowUa, t.zoneHighUa);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                xcanTgtCfgSel_  = i;
                xcanTgtCfgOpen_ = true;
            }
        }
        ImGui::PopID();
    }
    if (!anyTarget) {   // escape hatch — no button left to right-click
        ImGui::SameLine();
        if (ImGui::SmallButton("targets...")) xcanTgtCfgOpen_ = true;
    }

    // Launch row: hex-left [presets...]   hex-right [presets...]   Stop.
    // Left-click launches on that hexapod; RIGHT-click opens the preset's
    // settings modal (works even while the button is disabled).
    const bool busy = orch_.Busy();
    bool anyPreset = false;
    for (const XcanPreset& p : xcanPresets_) anyPreset |= p.enabled;
    const auto presets = [&](const char* dev) {
        int idx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) == dev) { idx = static_cast<int>(i); break; }
        }
        IMotionController* m =
            idx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(idx)) : nullptr;
        ImGui::TextDisabled("%s", dev);
        ImGui::PushID(dev);
        ImGui::BeginDisabled(busy || !m || !m->IsConnected());
        for (int i = 0; i < kXcanPresetCount; ++i) {
            const XcanPreset& p = xcanPresets_[i];
            if (!p.enabled) continue;
            ImGui::SameLine();
            ImGui::PushID(i);
            if (ImGui::Button(p.label.c_str(), ImVec2(70.0f, 0.0f)))
                StartXcanPreset(dev, i);
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("steps: %s um\nright-click: settings",
                                  p.stepsUm.c_str());
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                    xcanCfgSel_  = i;
                    xcanCfgOpen_ = true;
                }
            }
            ImGui::PopID();
        }
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    presets("hex-left");
    ImGui::SameLine();
    ImGui::TextDisabled(" | ");
    ImGui::SameLine();
    presets("hex-right");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();
    if (!anyPreset) {   // escape hatch — no button left to right-click
        ImGui::SameLine();
        if (ImGui::SmallButton("presets...")) xcanCfgOpen_ = true;
    }

    // Status line: the running/last hexapod_align result (shared engine ctx).
    if (st.scriptName == "hexapod_align") {
        ImGui::Text("%s", LuaEngine::StateName(st.state));
        for (const auto& [key, val] : st.ctx) {
            if (key == "align_baseline" || key == "align_best" ||
                key == "align_gain" || key == "align_exit_hit") {
                ImGui::SameLine();
                ImGui::TextDisabled("%s=%s", key.c_str() + 6,   // strip "align_"
                                    CtxValueToString(val).c_str());
            }
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
        }
    } else if (!scriptStartErr_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           scriptStartErr_.c_str());
    } else {
        std::string s;
        for (const XcanPreset& p : xcanPresets_) {
            if (!p.enabled) continue;
            s += (s.empty() ? "" : "   ") + p.label + ": " + p.stepsUm + " um";
        }
        if (s.empty()) s = "no presets enabled";
        ImGui::TextDisabled("%s   (right-click to edit)", s.c_str());
    }

    DrawXcanPresetModal();
    DrawXcanTargetModal();
}

// Right-click settings for an XCAN target preset: exit target + the adaptive
// step-zone range (lower/upper limits, in uA). Save persists all five target
// slots (and the launch presets) to config/scan_presets.local.json.
void App::DrawXcanTargetModal()
{
    if (xcanTgtCfgOpen_) {
        ImGui::OpenPopup("XCAN Target");
        xcanTgtCfgOpen_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("XCAN Target", nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;

    // Slot selector: all five targets, the edited one yellow.
    for (int i = 0; i < kXcanPresetCount; ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(XcanTargetLabel(xcanTargets_[i]).c_str(),
                         xcanTgtCfgSel_ == i, ImVec2(76.0f, 0.0f)))
            xcanTgtCfgSel_ = i;
        ImGui::PopID();
    }
    XcanTarget& t = xcanTargets_[xcanTgtCfgSel_];
    ImGui::Separator();

    ImGui::Checkbox("show button", &t.enabled);
    const float w = 150.0f;
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("target uA (exit threshold)", &t.targetUa, 0.0, 0.0, "%g");
    ImGui::SeparatorText("adaptive step zones");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("zone low uA (lower limit)", &t.zoneLowUa, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("zone high uA (upper limit)", &t.zoneHighUa, 0.0, 0.0, "%g");
    if (t.zoneHighUa < t.zoneLowUa)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                           "zone high is below zone low");
    ImGui::TextDisabled("zone low should sit at the raster handoff level");

    ImGui::Separator();
    if (ImGui::Button("Save", ImVec2(90.0f, 0.0f))) {
        SaveScanPresets();
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Persist ALL presets to config/scan_presets.local.json");
    ImGui::SameLine();
    if (ImGui::Button("Close", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::SameLine();
    if (ImGui::Button("Defaults", ImVec2(90.0f, 0.0f)))
        t = DefaultXcanTarget(xcanTgtCfgSel_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset THIS target to its built-in values");
    ImGui::EndPopup();
}

// Right-click settings for an XCAN launch preset — the same schema-form idea
// as the recipe parameter inspector, over the hexapod_align.lua tunables.
// Save persists all five presets to config/scan_presets.local.json; Close
// keeps the edits for this session only.
void App::DrawXcanPresetModal()
{
    if (xcanCfgOpen_) {
        ImGui::OpenPopup("XCAN Preset");
        xcanCfgOpen_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("XCAN Preset", nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;

    // Slot selector: all five presets, the edited one yellow.
    for (int i = 0; i < kXcanPresetCount; ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(xcanPresets_[i].label.c_str(), xcanCfgSel_ == i,
                         ImVec2(76.0f, 0.0f)))
            xcanCfgSel_ = i;
        ImGui::PopID();
    }
    XcanPreset& p = xcanPresets_[xcanCfgSel_];
    ImGui::Separator();

    ImGui::Checkbox("show button", &p.enabled);
    const float w = 150.0f;
    char lb[32];
    std::snprintf(lb, sizeof(lb), "%s", p.label.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("label", lb, sizeof(lb))) p.label = lb;
    char sb[64];
    std::snprintf(sb, sizeof(sb), "%s", p.stepsUm.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("steps_um (coarse -> fine)", sb, sizeof(sb)))
        p.stepsUm = sb;
    char ab[32];
    std::snprintf(ab, sizeof(ab), "%s", p.axes.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("axes (descent order)", ab, sizeof(ab))) p.axes = ab;

    ImGui::SeparatorText("measure");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("settle_ms", &p.settleMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("settle_min_ms", &p.settleMinMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("fresh_samples", &p.freshSamples);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("avg_count", &p.avgCount);

    ImGui::SeparatorText("convergence");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("consec_limit", &p.consecLimit);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("sig_drop_pct", &p.sigDropPct, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("improve_pct", &p.improvePct, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("min_signal", &p.minSignal, 0.0, 0.0, "%g");

    ImGui::SeparatorText("motion");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("max_travel_um", &p.maxTravelUm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("velocity_mm_s", &p.velocityMmS, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("tol_mm", &p.tolMm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("move_timeout_ms", &p.moveTimeoutMs, 0.0, 0.0, "%g");

    ImGui::SeparatorText("heuristics");
    ImGui::Checkbox("skip_opposite", &p.skipOpposite);
    ImGui::SameLine();
    ImGui::Checkbox("dir_memory", &p.dirMemory);
    ImGui::SameLine();
    ImGui::Checkbox("reuse_sample", &p.reuseSample);

    ImGui::Separator();
    if (ImGui::Button("Save", ImVec2(90.0f, 0.0f))) {
        SaveScanPresets();
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Persist ALL five presets to config/scan_presets.local.json");
    ImGui::SameLine();
    if (ImGui::Button("Close", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::SameLine();
    if (ImGui::Button("Defaults", ImVec2(90.0f, 0.0f)))
        p = DefaultXcanPreset(xcanCfgSel_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset THIS preset to its built-in values");
    ImGui::EndPopup();
}

// Raster scan launcher: raster_scan.lua with the band's spans/steps + the
// tab's active first-light target preset; feedback channel and Trace are
// shared with the XCAN band. Axes stay at the script's defaults (slow = X
// stepping down from start, fast = Y sweeping centered).
void App::StartRasterScan(const std::string& device)
{
    std::string script = "config/scripts/raster_scan.lua";
    for (const ScriptInfo& si : session_.Scripts()) {
        if (si.name == "raster_scan") { script = si.path; break; }
    }
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"raster_device", device},
        {"raster_topic", std::string(runXcanTopic_)},
        {"raster_slow_span_um", rasterSlowSpanUm_},
        {"raster_slow_step_um", rasterSlowStepUm_},
        {"raster_fast_span_um", rasterFastSpanUm_},
        {"raster_fast_step_um", rasterFastStepUm_},
        {"raster_settle_ms", 150.0},
        {"raster_trace", xcanTrace_ ? 1.0 : 0.0},
        {"raster_exit_threshold", kRasterTargetsUa[rasterTgtSel_] * 1e-6},
    };
    scriptStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
        FD_WARN("script", "raster scan rejected: %s", scriptStartErr_.c_str());
}

// Raster tab of the Scan band: span/step knobs + per-hexapod Scan buttons.
// Channel/Target/Trace live in the shared Scan-band header (DrawRunScanBlock).
void App::DrawRunRasterBlock()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    // Knob row: X- span/step, Y span/step (um).
    ImGui::TextDisabled("X- span/step");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60.0f);
    ImGui::InputDouble("##rss", &rasterSlowSpanUm_, 0.0, 0.0, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputDouble("##rsp", &rasterSlowStepUm_, 0.0, 0.0, "%.1f");
    ImGui::SameLine();
    ImGui::TextDisabled("  Y+/- span/step");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60.0f);
    ImGui::InputDouble("##rfs", &rasterFastSpanUm_, 0.0, 0.0, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputDouble("##rfp", &rasterFastStepUm_, 0.0, 0.0, "%.1f");
    ImGui::SameLine();
    ImGui::TextDisabled("um");
    if (rasterSlowStepUm_ < 0.1) rasterSlowStepUm_ = 0.1;
    if (rasterFastStepUm_ < 0.1) rasterFastStepUm_ = 0.1;
    if (rasterSlowSpanUm_ < 0.0) rasterSlowSpanUm_ = 0.0;
    if (rasterFastSpanUm_ < 0.0) rasterFastSpanUm_ = 0.0;

    // Target preset row: first-light exit level, activatable (yellow = live).
    ImGui::TextDisabled("Target");
    for (int i = 0; i < static_cast<int>(std::size(kRasterTargetsUa)); ++i) {
        ImGui::SameLine();
        ImGui::PushID(i);
        char lbl[16];
        std::snprintf(lbl, sizeof(lbl), "%g uA", kRasterTargetsUa[i]);
        if (PresetButton(lbl, rasterTgtSel_ == i, ImVec2(64.0f, 0.0f)))
            rasterTgtSel_ = i;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Stop-and-stay the moment a grid node reaches %g uA",
                              kRasterTargetsUa[i]);
        ImGui::PopID();
    }

    // Launch row: one Scan per hexapod + Stop; channel and Trace come from
    // the shared Scan-band header.
    const bool busy = orch_.Busy();
    const auto scanBtn = [&](const char* dev) {
        int idx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) == dev) { idx = static_cast<int>(i); break; }
        }
        IMotionController* m =
            idx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(idx)) : nullptr;
        ImGui::TextDisabled("%s", dev);
        ImGui::SameLine();
        ImGui::PushID(dev);
        ImGui::BeginDisabled(busy || !m || !m->IsConnected());
        if (ImGui::Button("Scan", ImVec2(70.0f, 0.0f))) StartRasterScan(dev);
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    scanBtn("hex-left");
    ImGui::SameLine();
    ImGui::TextDisabled(" | ");
    ImGui::SameLine();
    scanBtn("hex-right");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();

    // Status line: the running/last raster_scan result (shared engine ctx).
    if (st.scriptName == "raster_scan") {
        ImGui::Text("%s", LuaEngine::StateName(st.state));
        for (const auto& [key, val] : st.ctx) {
            if (key == "raster_baseline" || key == "raster_best" ||
                key == "raster_points" || key == "raster_exit_hit" ||
                key == "raster_best_slow_um" || key == "raster_best_fast_um") {
                ImGui::SameLine();
                ImGui::TextDisabled("%s=%s", key.c_str() + 7,   // strip "raster_"
                                    CtxValueToString(val).c_str());
            }
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
        }
    } else {
        ImGui::TextDisabled("Serpentine area scan: X steps down from start, Y sweeps +/- half span.");
    }
}

} // namespace firedog
