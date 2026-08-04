// PageScripts.cpp : the Scripts page (View menu, full-page) — list, read-only
// source view and the Lua debugger controls. Definitions of App members —
// see App.h.
//
// Authoring is external; this page lists .lua programs, highlights the
// executing line, and drives the lua_sethook debugger: click a source row to
// toggle a breakpoint (hot-swappable mid-run), Pause / Continue / Step, and the
// current function's locals while paused. Stop and safety aborts always release
// a pause.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "command/ExecutionContext.h"   // IsSystemScript / SystemScriptPath
#include "core/Log.h"
#include "core/DutSession.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

namespace firedog {

void App::LoadScriptsList()
{
    session_.LoadScriptsList();
    SyncScriptSelection();
}

void App::DrawScriptsPanel()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    const float  stripH = ImGui::GetFrameHeightWithSpacing() * 3.2f;
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  paneH  = avail.y - stripH;
    const float  leftW  = 230.0f;
    const float  rightW = 300.0f;

    // ---- Left: script list ----
    ImGui::BeginChild("scList", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Scripts");
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::SmallButton("Reload")) LoadScriptsList();
    ImGui::EndDisabled();
    ImGui::TextDisabled("%s", session_.ScriptsDirUsed().c_str());
    ImGui::Separator();
    if (session_.Scripts().empty())
        ImGui::TextWrapped("No .lua scripts found. Author them in an external "
                           "editor and hit Reload.");
    for (int i = 0; i < static_cast<int>(session_.Scripts().size()); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(session_.Scripts()[i].name.c_str(), scriptSel_ == i)) {
            scriptSel_      = i;
            scriptBpAnchor_ = -1;   // anchor is per-script
        }
        if (ImGui::IsItemHovered() && !session_.Scripts()[i].description.empty())
            ImGui::SetTooltip("%s", session_.Scripts()[i].description.c_str());
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Middle: read-only source, executing line highlighted ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    ImGui::BeginChild("scSrc", ImVec2(midW, paneH), ImGuiChildFlags_Borders);
    if (scriptSel_ >= 0 && scriptSel_ < static_cast<int>(session_.Scripts().size())) {
        const ScriptInfo& si = session_.Scripts()[scriptSel_];
        if (scriptSrcPath_ != si.path) {   // (re)load on selection change / Reload
            scriptSrcPath_ = si.path;
            scriptSrc_.clear();
            std::ifstream f(si.path);
            std::string line;
            while (std::getline(f, line)) {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                scriptSrc_.push_back(line);
            }
        }
        if (!si.description.empty()) ImGui::TextWrapped("%s", si.description.c_str());
        // A standard procedure is run from the shipped copy wherever it is
        // launched from. Say so on the file being read, because a product copy
        // opened here looks exactly like the one that runs, and editing it
        // would change nothing at all.
        if (IsSystemScript(si.path)) {
            const std::string shipped = SystemScriptPath(si.path);
            std::error_code   ec;
            if (!std::filesystem::equivalent(si.path, shipped, ec))
                ImGui::TextColored(ImVec4(0.98f, 0.72f, 0.32f, 1.0f),
                                   "STANDARD procedure - align_run/place_run and the "
                                   "page buttons are served from %s, never this copy",
                                   shipped.c_str());
            else
                ImGui::TextDisabled("standard procedure - kept in step with the "
                                    "align_/place_ commands; a product cannot fork it");
        }
        ImGui::TextDisabled("click: toggle breakpoint  |  shift+click: range from last click");
        ImGui::Separator();
        ImGui::BeginChild("##srclines");
        const bool thisRunning = running && st.scriptName == si.name;
        std::set<int>& bps = scriptBps_[si.path];
        for (int n = 0; n < static_cast<int>(scriptSrc_.size()); ++n) {
            const int  lineNo = n + 1;
            const bool bp     = bps.count(lineNo) != 0;
            const bool cur    = thisRunning && lineNo == st.currentLine;
            char row[512];
            std::snprintf(row, sizeof(row), "%c%4d  %s", bp ? '*' : ' ', lineNo,
                          scriptSrc_[n].c_str());
            if (cur) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.3f, 1.0f));
            else if (bp) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.45f, 1.0f));
            ImGui::PushID(n);
            if (ImGui::Selectable(row)) {   // toggle breakpoint (IDE gutter metaphor)
                if (ImGui::GetIO().KeyShift && scriptBpAnchor_ > 0) {
                    // Range from the anchor: set every line, or clear the
                    // whole range when it is already fully set.
                    const int lo = scriptBpAnchor_ < lineNo ? scriptBpAnchor_ : lineNo;
                    const int hi = scriptBpAnchor_ < lineNo ? lineNo : scriptBpAnchor_;
                    bool allSet = true;
                    for (int l = lo; l <= hi; ++l) {
                        if (bps.count(l) == 0) { allSet = false; break; }
                    }
                    for (int l = lo; l <= hi; ++l) {
                        if (allSet) bps.erase(l);
                        else        bps.insert(l);
                    }
                } else {   // plain or ctrl+click: individual toggle, new anchor
                    if (bp) bps.erase(lineNo);
                    else    bps.insert(lineNo);
                    scriptBpAnchor_ = lineNo;
                }
                if (thisRunning) orch_.lua.SetBreakpoints(bps);   // hot-swap live
            }
            ImGui::PopID();
            if (cur || bp) ImGui::PopStyleColor();
            if (cur && !st.paused) ImGui::SetScrollHereY(0.5f);   // follow execution
        }
        ImGui::EndChild();
    } else {
        ImGui::TextDisabled("Select a program on the left.");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: run info ----
    ImGui::BeginChild("scInfo", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    if (st.paused)
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "State: Paused @ line %d",
                           st.currentLine);
    else
        ImGui::Text("State: %s", LuaEngine::StateName(st.state));
    if (!st.scriptName.empty()) ImGui::Text("Script: %s", st.scriptName.c_str());
    if (running && st.currentLine > 0 && !st.paused)
        ImGui::Text("Line: %d", st.currentLine);
    if (!st.lastCommand.empty()) ImGui::Text("Last command: %s", st.lastCommand.c_str());
    if (st.paused && !st.locals.empty()) {
        ImGui::Separator();
        ImGui::TextUnformatted("Locals");
        if (ImGui::BeginTable("##sclocals", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            for (const auto& [name, val] : st.locals) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(name.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(val.c_str());
            }
            ImGui::EndTable();
        }
    }
    if (!st.error.empty() && st.state != LuaEngine::State::Running)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
    if (!st.ctx.empty()) {
        ImGui::Separator();
        ImGui::TextUnformatted("Context");
        if (ImGui::BeginTable("##scctx", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            for (const auto& [key, val] : st.ctx) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(key.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(CtxValueToString(val).c_str());
            }
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();

    // ---- Bottom strip: Run / Stop ----
    ImGui::BeginChild("scStrip", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    const bool canRun = !orch_.Busy() && scriptSel_ >= 0 &&
                        scriptSel_ < static_cast<int>(session_.Scripts().size());
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Run", ImVec2(90.0f, 0.0f))) {
        LuaEngine::Env env = session_.MakeLuaEnv();
        scriptStartErr_.clear();
        orch_.lua.SetBreakpoints(scriptBps_[session_.Scripts()[scriptSel_].path]);
        if (!orch_.lua.Start(session_.Scripts()[scriptSel_].path, std::move(env), &scriptStartErr_))
            FD_WARN("script", "start rejected: %s", scriptStartErr_.c_str());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (!st.paused) {
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Pause", ImVec2(90.0f, 0.0f))) orch_.lua.Pause();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Continue", ImVec2(90.0f, 0.0f))) orch_.lua.Resume();
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(90.0f, 0.0f))) orch_.lua.StepLine();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(90.0f, 0.0f))) orch_.lua.RequestStop();
    ImGui::EndDisabled();
    {
        const DutSession::Info di = DutSession::Instance().GetInfo();
        ImGui::SameLine();
        ImGui::TextDisabled("DUT: %s", di.open ? di.serial.c_str() : "none");
    }
    if (!scriptStartErr_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           scriptStartErr_.c_str());
    }
    ImGui::EndChild();
}

} // namespace firedog
