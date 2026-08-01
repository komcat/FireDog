// PageMotion.cpp : the Motion page — device list, tabbed detail (Positions and
// Scanning; the Mapped Path tab lives in PageMotionMap.cpp) and the live-axis
// info panel. Definitions of App members — see App.h.
//
// The Scanning tab is a test bench over hexapod_align.lua and owns NO scan
// logic: its form values are injected via LuaEngine::Env::initialCtx, exactly
// as a recipe would inject them with ctx_set steps. Open motion views call
// RequestFastPoll every frame — it is a demand signal, not a mode.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <cstdio>
#include <map>

namespace firedog {

void App::DrawMotionListPanel()
{
    const size_t n = hardware_.MotionCount();

    // Keep the current selection if still valid; otherwise fall back to the
    // first device. The left panel draws before the middle/right, so this also
    // resolves the selection they read.
    if (selectedMotion_ < 0 || static_cast<size_t>(selectedMotion_) >= n ||
        !hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))) {
        selectedMotion_ = n > 0 ? 0 : -1;
    }

    ImGui::TextUnformatted("Motion devices");
    ImGui::Separator();

    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        IMotionController* m = hardware_.MotionDevice(i);
        if (!m) continue;
        any = true;
        const ConnState st = hardware_.MotionState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        const std::string& name = hardware_.MotionName(i);
        const ImVec2 sz = ImGui::CalcTextSize(name.c_str());
        if (ImGui::Selectable(name.c_str(), selectedMotion_ == static_cast<int>(i),
                              ImGuiSelectableFlags_None, ImVec2(sz.x, 0.0f))) {
            selectedMotion_ = static_cast<int>(i);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m->Type().c_str());
        ImGui::SameLine();
        ImGui::TextColored(StateColor(st), "%s", StateText(st));
        ImGui::PopID();
    }
    if (!any) ImGui::TextDisabled("no motion devices configured");
}

void App::DrawMotionDetailPanel()
{
    const size_t n = hardware_.MotionCount();
    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;
    if (!m) {
        ImGui::TextDisabled("Select a device from the list.");
        return;
    }
    // This page is showing this device live — ask it to poll fast while we're here.
    m->RequestFastPoll();

    const size_t    idx = static_cast<size_t>(selectedMotion_);
    const ConnState st  = hardware_.MotionState(idx);

    ImGui::TextUnformatted(hardware_.MotionName(idx).c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "[%s]", StateText(st));
    ImGui::Separator();

    const std::vector<std::string> axes = m->Axes();
    std::string axesLine;
    for (size_t i = 0; i < axes.size(); ++i) { if (i) axesLine += ' '; axesLine += axes[i]; }
    ImGui::Text("Type : %s", m->Type().c_str());
    ImGui::Text("Axes : %s", axesLine.c_str());

    if (st == ConnState::Failed) {
        ImGui::Spacing();
        ImGui::TextColored(StateColor(ConnState::Failed), "error: %s", m->LastError().c_str());
    }

    // ---- Tabbed content: saved positions | node-edge motion maps ----
    ImGui::Spacing();
    if (ImGui::BeginTabBar("motiontabs")) {
        if (ImGui::BeginTabItem("Positions")) {
            DrawMotionPositionsTab(idx, m);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Mapped Path")) {
            DrawMotionMapTab(idx, m);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Scanning")) {
            DrawMotionScanTab(idx, m);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

// Scanning tab — test bench for the script-level XCAN aligner: edit the
// algorithm parameters here, hit XCAN, and they are injected into
// hexapod_align.lua exactly as a recipe would inject them (Env.initialCtx ==
// ctx_set steps == fd.param sources). The script is the single algorithm
// implementation — this tab owns no scan logic.
void App::DrawMotionScanTab(std::size_t devIdx, IMotionController* m)
{
    (void)m;
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    ImGui::TextUnformatted("XCAN active alignment (config/scripts/hexapod_align.lua)");
    ImGui::Separator();

    // Device to scan: PI hexapods only (the align algorithm relies on PI's
    // verified settle — ACS reports settled immediately). Defaults to the
    // page's selected device when that is a PI, else the first PI.
    const auto isPi = [&](int i) {
        return i >= 0 && i < static_cast<int>(hardware_.MotionCount()) &&
               hardware_.MotionDevice(static_cast<std::size_t>(i))->Type() == "PI";
    };
    if (!isPi(xcanDevice_)) {
        xcanDevice_ = isPi(static_cast<int>(devIdx)) ? static_cast<int>(devIdx) : -1;
        for (std::size_t i = 0; xcanDevice_ < 0 && i < hardware_.MotionCount(); ++i)
            if (isPi(static_cast<int>(i))) xcanDevice_ = static_cast<int>(i);
    }
    if (xcanDevice_ < 0) {
        ImGui::TextDisabled("(no PI hexapod in the motion fleet)");
        return;
    }
    IMotionController* dev =
        hardware_.MotionDevice(static_cast<std::size_t>(xcanDevice_));

    ImGui::BeginDisabled(running);
    const float w = 180.0f;
    ImGui::SetNextItemWidth(w);
    if (ImGui::BeginCombo("device", hardware_.MotionName(
                                        static_cast<std::size_t>(xcanDevice_)).c_str())) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (!isPi(static_cast<int>(i))) continue;
            if (ImGui::Selectable(hardware_.MotionName(i).c_str(),
                                  xcanDevice_ == static_cast<int>(i)))
                xcanDevice_ = static_cast<int>(i);
        }
        ImGui::EndCombo();
    }
    // Feedback signal: only channels that actually exist on the DataBus.
    ImGui::SetNextItemWidth(w);
    if (ImGui::BeginCombo("feedback topic", xcanTopic_)) {
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (topics.empty())
            ImGui::TextDisabled("(no channels published yet)");
        for (const std::string& t : topics) {
            if (ImGui::Selectable(t.c_str(), t == xcanTopic_))
                std::snprintf(xcanTopic_, sizeof(xcanTopic_), "%s", t.c_str());
        }
        ImGui::EndCombo();
    }
    ImGui::SetNextItemWidth(w);
    ImGui::InputText("axes (descent order)", xcanAxes_, sizeof(xcanAxes_));
    ImGui::SetNextItemWidth(w);
    ImGui::InputText("step ladder (um)", xcanSteps_, sizeof(xcanSteps_));
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("settle (ms)", &xcanSettleMs_);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("consecutive decreases", &xcanConsec_);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("significant drop (%)", &xcanSigDropPct_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("max travel per dir (um)", &xcanTravelUm_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("peak recovery (%)", &xcanImprovePct_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("baseline averages", &xcanAvg_);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("min signal (abort below)", &xcanMinSignal_, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("exit threshold (0 = off)", &xcanExitThr_, 0.0, 0.0, "%g");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop scanning the moment a sample reaches this value\n"
                          "and STAY at that point (good-enough early exit).");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("move tolerance (mm)", &xcanTolMm_, 0.0, 0.0, "%g");
    ImGui::Checkbox("trace every sample", &xcanTrace_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("One 'xcan,...' CSV line per sample: time, phase, axis,\n"
                          "direction, step, positions, value, and delta vs the\n"
                          "previous sample. Persists in the daily log file:\n"
                          "  findstr \"xcan,\" logs\\firedog_*.log > scan.csv");
    ImGui::EndDisabled();

    ImGui::Spacing();
    const bool canRun = !orch_.Busy() && dev && dev->IsConnected();
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("XCAN", ImVec2(110.0f, 0.0f))) {
        // The product's script overrides the config one when present.
        std::string script = "config/scripts/hexapod_align.lua";
        for (const ScriptInfo& si : session_.Scripts()) {
            if (si.name == "hexapod_align") { script = si.path; break; }
        }
        LuaEngine::Env env = session_.MakeLuaEnv();
        env.initialCtx = {
            {"align_device", hardware_.MotionName(static_cast<std::size_t>(xcanDevice_))},
            {"align_topic", std::string(xcanTopic_)},
            {"align_axes", std::string(xcanAxes_)},
            {"align_steps_um", std::string(xcanSteps_)},
            {"align_settle_ms", static_cast<double>(xcanSettleMs_)},
            {"align_consec_limit", static_cast<double>(xcanConsec_)},
            {"align_sig_drop_pct", xcanSigDropPct_},
            {"align_max_travel_um", xcanTravelUm_},
            {"align_improve_pct", xcanImprovePct_},
            {"align_avg_count", static_cast<double>(xcanAvg_)},
            {"align_min_signal", xcanMinSignal_},
            {"align_exit_threshold", xcanExitThr_},
            {"align_tol_mm", xcanTolMm_},
            {"align_trace", xcanTrace_ ? 1.0 : 0.0},
        };
        scriptStartErr_.clear();
        if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
            FD_WARN("script", "xcan start rejected: %s", scriptStartErr_.c_str());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(110.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();
    if (!canRun && !running) {
        ImGui::SameLine();
        ImGui::TextDisabled(!dev || !dev->IsConnected() ? "(device not connected)"
                                                        : "(another run is active)");
    }
    if (!scriptStartErr_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           scriptStartErr_.c_str());
    }

    // ---- Live status + results (from the shared engine/ctx) ----
    ImGui::Separator();
    const bool thisScan = st.scriptName == "hexapod_align";
    ImGui::Text("State: %s", LuaEngine::StateName(st.state));
    if (running && thisScan && !st.lastCommand.empty()) {
        ImGui::SameLine();
        ImGui::TextDisabled("| %s", st.lastCommand.c_str());
    }
    if (thisScan) {
        for (const auto& [key, val] : st.ctx) {
            if (key == "align_baseline" || key == "align_best" ||
                key == "align_delta" || key == "align_gain" ||
                key == "align_exit_hit")
                ImGui::Text("%s: %s", key.c_str(), CtxValueToString(val).c_str());
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running)
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
    }
}

// Positions tab — the saved/taught position table (persisted; Move drives
// through Workflow).
void App::DrawMotionPositionsTab(std::size_t idx, IMotionController* m)
{
    const std::vector<std::string> axes = m->Axes();

    ImGui::TextUnformatted("Saved positions");
    ImGui::SameLine();

    const std::string      devName = hardware_.MotionName(idx);
    std::vector<SavedPos>& saved   = session_.Positions()[devName];
    if (saved.empty()) {                       // seed one placeholder row
        SavedPos home;
        std::snprintf(home.name, sizeof(home.name), "Home");
        home.uid = GenerateUid();
        for (const auto& a : axes) home.vals[a] = 0.0;
        saved.push_back(std::move(home));
    }

    if (ImGui::SmallButton("+ Add current")) {
        SavedPos p;
        std::snprintf(p.name, sizeof(p.name), "P%zu", saved.size());
        p.uid = GenerateUid();
        const std::map<std::string, double> pos = m->Positions();
        for (const auto& a : axes) { const auto it = pos.find(a); p.vals[a] = (it != pos.end()) ? it->second : 0.0; }
        saved.push_back(std::move(p));
        session_.SavePositions();
    }

    // Per-position visibility widget: system positions stay hidden from the
    // table unless revealed here. The hidden flag persists with the position.
    ImGui::SameLine();
    size_t hiddenCount = 0;
    for (const SavedPos& sp : saved) if (sp.hidden) ++hiddenCount;
    if (ImGui::SmallButton("Show/Hide...")) ImGui::OpenPopup("posvis");
    if (hiddenCount > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("(%zu hidden)", hiddenCount);
    }
    if (ImGui::BeginPopup("posvis")) {
        ImGui::TextDisabled("Checked positions are shown in the table.");
        ImGui::Checkbox("Reveal hidden rows (dimmed)", &showHiddenPos_);
        ImGui::Separator();
        for (size_t r = 0; r < saved.size(); ++r) {
            ImGui::PushID(static_cast<int>(r));
            bool visible = !saved[r].hidden;
            if (ImGui::Checkbox(saved[r].name, &visible)) {
                saved[r].hidden = !visible;
                session_.SavePositions();
            }
            ImGui::PopID();
        }
        ImGui::EndPopup();
    }

    const int cols = 5 + static_cast<int>(axes.size());
    if (ImGui::BeginTable("savedpos", cols,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
            ImVec2(0.0f, ImGui::GetContentRegionAvail().y))) {   // fill the panel
        ImGui::TableSetupColumn("##move", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##upd",  ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Name",   ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##ren",  ImGuiTableColumnFlags_WidthFixed);
        for (const auto& a : axes) ImGui::TableSetupColumn(a.c_str(), ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##del",  ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();

        for (size_t r = 0; r < saved.size(); ++r) {
            SavedPos& sp = saved[r];
            if (sp.hidden && !showHiddenPos_) continue;   // system position
            ImGui::TableNextRow();
            ImGui::PushID(static_cast<int>(r));
            if (sp.hidden)   // revealed hidden row — dim the whole row's text
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));

            ImGui::TableSetColumnIndex(0);
            if (ImGui::SmallButton("Move")) {   // drive to the saved target, all axes together
                std::vector<std::pair<std::string, double>> tgt;
                for (const auto& a : axes) {
                    const auto it = sp.vals.find(a);
                    if (it != sp.vals.end()) tgt.emplace_back(a, it->second);
                }
                if (workflow_.MoveToPosition(static_cast<size_t>(selectedMotion_), tgt))
                    FD_INFO("saved", "move %s -> '%s'", devName.c_str(), sp.name);
                else
                    FD_WARN("saved", "move %s -> '%s' failed", devName.c_str(), sp.name);
            }
            ImGui::TableSetColumnIndex(1);
            if (ImGui::SmallButton("Upd")) {    // capture the live position (real)
                const std::map<std::string, double> pos = m->Positions();
                for (const auto& a : axes) { const auto it = pos.find(a); if (it != pos.end()) sp.vals[a] = it->second; }
                FD_INFO("saved", "updated '%s' from current position", sp.name);
                session_.SavePositions();
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::TextUnformatted(sp.name);   // read-only — rename via the Edit dialog
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("uid: %s", sp.uid.c_str());
            ImGui::TableSetColumnIndex(3);
            if (ImGui::SmallButton("Edit")) {
                renameRow_ = static_cast<int>(r);
                std::snprintf(renameBuf_, sizeof(renameBuf_), "%s", sp.name);
                openRenamePos_ = true;
            }

            int c = 4;
            for (const auto& a : axes) {
                ImGui::TableSetColumnIndex(c++);
                const auto it = sp.vals.find(a);
                ImGui::Text("%.4f", (it != sp.vals.end()) ? it->second : 0.0);
            }
            ImGui::TableSetColumnIndex(c);
            if (ImGui::SmallButton("X")) {   // confirmed via dialog before erasing
                deleteRow_     = static_cast<int>(r);
                openDeletePos_ = true;
            }
            if (sp.hidden) ImGui::PopStyleColor();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    // ---- Delete confirmation dialog ----
    if (openDeletePos_) { ImGui::OpenPopup("Delete Position"); openDeletePos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = deleteRow_ >= 0 && static_cast<size_t>(deleteRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            const SavedPos& sp = saved[static_cast<size_t>(deleteRow_)];
            ImGui::Text("Delete position '%s' on %s?", sp.name, devName.c_str());
            ImGui::TextDisabled("uid %s — anything referencing it will break. This cannot be undone.",
                                sp.uid.c_str());
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
            if (ImGui::Button("Delete")) {
                FD_INFO("saved", "deleted '%s' (uid %s, %s)", sp.name, sp.uid.c_str(), devName.c_str());
                saved.erase(saved.begin() + deleteRow_);
                session_.SavePositions();
                if (renameRow_ == deleteRow_) renameRow_ = -1;   // row gone — drop pending rename
                deleteRow_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) { deleteRow_ = -1; ImGui::CloseCurrentPopup(); }
        }
        ImGui::EndPopup();
    }

    // ---- Rename dialog (name is a label; the uid never changes) ----
    if (openRenamePos_) { ImGui::OpenPopup("Rename Position"); openRenamePos_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Rename Position", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const bool rowValid = renameRow_ >= 0 && static_cast<size_t>(renameRow_) < saved.size();
        if (!rowValid) {
            ImGui::CloseCurrentPopup();
        } else {
            SavedPos& sp = saved[static_cast<size_t>(renameRow_)];
            ImGui::Text("Position uid %s (unchanged)", sp.uid.c_str());
            ImGui::SetNextItemWidth(320.0f);
            if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
            const bool submit = ImGui::InputText("##rename", renameBuf_, sizeof(renameBuf_),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
            const bool valid = renameBuf_[0] != '\0';
            ImGui::BeginDisabled(!valid);
            if (ImGui::Button("OK") || (submit && valid)) {
                FD_INFO("saved", "renamed '%s' -> '%s' (uid %s)", sp.name, renameBuf_, sp.uid.c_str());
                std::snprintf(sp.name, sizeof(sp.name), "%s", renameBuf_);
                session_.SavePositions();
                renameRow_ = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) { renameRow_ = -1; ImGui::CloseCurrentPopup(); }
        }
        ImGui::EndPopup();
    }
}

void App::DrawMotionInfoPanel()
{
    // Right panel: the live axis-position table for the selected device.
    const size_t n = hardware_.MotionCount();
    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;

    ImGui::TextUnformatted("Axis positions");
    ImGui::Separator();
    if (!m) { ImGui::TextDisabled("no selection"); return; }

    const std::vector<std::string>      axes = m->Axes();
    const std::map<std::string, double> pos  = m->Positions();
    if (ImGui::BeginTable("motionpos", 2,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Axis");
        ImGui::TableSetupColumn("Position");
        ImGui::TableHeadersRow();
        for (const std::string& ax : axes) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(ax.c_str());
            ImGui::TableSetColumnIndex(1);
            const auto it = pos.find(ax);
            if (it != pos.end()) ImGui::Text("%.4f", it->second);
            else                 ImGui::TextDisabled("--");
        }
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Separator();
    DrawVelocityCombo(static_cast<size_t>(selectedMotion_));
}

} // namespace firedog
