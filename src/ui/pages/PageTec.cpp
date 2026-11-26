// PageTec.cpp : the TEC page (the Arroyo TECSource fleet).
// Definitions of App members — see App.h. The page drives ONE TEC at a time
// (tecSel_, the PSU page's pattern). Every action goes through Workflow's
// Request* forms, so the page never waits on a serial port; what it shows is
// the driver's cached read-back, i.e. what the CONTROLLER says. Nothing is
// written at connect, so the output ON is always an explicit action here.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Topics.h"
#include "hardware/instruments/ArroyoTec.h"

#include <algorithm>
#include <cfloat>
#include <cstdio>

namespace firedog {

namespace {
const ImVec4 kTecOnColor   {0.30f, 0.85f, 0.40f, 1.0f};
const ImVec4 kTecWarnColor {1.00f, 0.45f, 0.20f, 1.0f};
constexpr unsigned kTecFaultBits = ArroyoTec::kCondSensorShort | ArroyoTec::kCondSensorOpen |
                                   ArroyoTec::kCondTecOpen | ArroyoTec::kCondRunaway;
} // namespace

std::size_t App::TecSelected()
{
    if (tecSel_ < 0 || static_cast<std::size_t>(tecSel_) >= hardware_.TecCount()) tecSel_ = 0;
    return static_cast<std::size_t>(tecSel_);
}

void App::DrawTecListPanel()
{
    ImGui::TextUnformatted("Arroyo TEC");
    ImGui::Separator();
    if (hardware_.TecCount() == 0) {
        ImGui::TextDisabled("no Arroyo TEC configured in hardware.json");
        ImGui::TextDisabled("(instruments[], type \"arroyo_tec\", port \"COMn\")");
        return;
    }

    // One row per TEC; the selected one is what the other two panels drive.
    const std::size_t sel = TecSelected();
    for (std::size_t i = 0; i < hardware_.TecCount(); ++i) {
        const ConnState rst = hardware_.TecState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(rst), "%s", StateGlyph(rst));
        ImGui::SameLine();
        if (ImGui::Selectable(hardware_.TecName(i).c_str(), i == sel) && i != sel) {
            tecSel_    = static_cast<int>(i);
            tecPulled_ = false;   // re-seed the edit from THIS controller's set point
        }
        ImGui::PopID();
    }
    ImGui::Separator();

    const std::size_t  idx  = TecSelected();
    const std::string& name = hardware_.TecName(idx);
    const ConnState    st   = hardware_.TecState(idx);
    ImGui::TextUnformatted(name.c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectTec(name);
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectTec(name);
        tecPulled_ = false;   // re-seed the edit on the next connect
    }
    ImGui::EndDisabled();
    ImGui::TextDisabled("Disconnect returns the front panel to local");
    ImGui::TextDisabled("and leaves the output as it is.");

    // A failed connect, or a dropped link the poll thread is reopening. A
    // refused set point is in the log and the Info panel's unit error.
    if (ArroyoTec* tec = hardware_.TecDevice(idx)) {
        const std::string err = tec->LastError();
        if (!err.empty() && (st == ConnState::Failed ||
                             (st == ConnState::Connected && !tec->GetStatus().linkUp))) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, st == ConnState::Failed ? StateColor(ConnState::Failed)
                                                                         : kTecWarnColor);
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawTecDetailPanel()
{
    if (hardware_.TecCount() == 0) {
        ImGui::TextDisabled("no Arroyo TEC configured");
        return;
    }
    const std::size_t  idx  = TecSelected();
    const std::string& name = hardware_.TecName(idx);
    const ConnState    st   = hardware_.TecState(idx);
    ArroyoTec*         tec  = hardware_.TecDevice(idx);
    const bool connected = (st == ConnState::Connected) && tec;
    const ArroyoTec::Status s = tec ? tec->GetStatus() : ArroyoTec::Status{};

    if (connected && s.valid && !tecPulled_) {   // seed the edit from the controller
        tecTempSet_ = static_cast<float>(s.setC);
        tecPulled_  = true;
    }
    if (!connected) tecPulled_ = false;

    ImGui::Text("Temperature control - %s", name.c_str());
    ImGui::Separator();
    // Requests go to the driver's poll thread; a run owns the TEC while it is busy.
    const bool busy = orch_.Busy();
    ImGui::BeginDisabled(!connected || busy || !s.linkUp);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("C##tecset", &tecTempSet_, 0.0f, 0.0f, "%.2f");
    if (s.limTHiC > s.limTLoC)
        tecTempSet_ = std::clamp(tecTempSet_, static_cast<float>(s.limTLoC), static_cast<float>(s.limTHiC));
    ImGui::SameLine();
    if (ImGui::Button("Apply##tecset")) workflow_.RequestTecTemperature(name, tecTempSet_);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
    if (ImGui::Button("Output ON")) workflow_.RequestTecOutput(name, true);
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button("Output OFF")) workflow_.RequestTecOutput(name, false);
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (connected && busy) ImGui::TextDisabled("a run is active - the TEC is the run's");

    if (!connected) {
        ImGui::TextDisabled("connect the controller to operate it");
        return;
    }
    if (!s.valid) {
        ImGui::TextDisabled("no reading yet");
        return;
    }

    ImGui::Spacing();
    char tval[48];
    if (s.tempValid) std::snprintf(tval, sizeof(tval), "%.3f C", s.tempC);
    else             std::snprintf(tval, sizeof(tval), "-- C (no sensor)");
    if (s.outputOn) {
        ImGui::TextColored(kTecOnColor, "OUTPUT ON  |  %s (set %.2f)", tval, s.setC);
        ImGui::SameLine();
        if (s.stable) ImGui::TextColored(kTecOnColor, "  STABLE");
        else          ImGui::TextColored(kTecWarnColor, "  settling");
    } else {
        ImGui::TextDisabled("output OFF  |  %s (set %.2f)", tval, s.setC);
    }
    ImGui::Text("I = %.2f A   V = %.2f V", s.currentA, s.voltageV);
    if (!s.linkUp)
        ImGui::TextColored(kTecWarnColor, "link down - reconnecting (values are the last read)");
    if (s.cond & kTecFaultBits)
        ImGui::TextColored(kTecWarnColor, "%s - the output will not stay on",
                           ArroyoTec::CondText(s.cond & kTecFaultBits).c_str());
    if (s.mode != "T" && !s.mode.empty())
        ImGui::TextColored(kTecWarnColor, "controller is in %s mode - the set point is a temperature "
                           "only in T mode (front panel menu)", s.mode.c_str());

    // Temperature history (the driver keeps ten minutes), set point in amber.
    ImGui::Spacing();
    DataBus::Instance().Stream(topics::TecTemp(name)).Snapshot(tecChartT_, tecChartV_);
    const ImVec2 size(-1.0f, std::max(160.0f, ImGui::GetContentRegionAvail().y));
    if (ImPlot::BeginPlot("##tectemp", size)) {
        ImPlot::SetupAxes("t (s)", "temperature (C)", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisFormat(ImAxis_X1, "%.0fs");
        ImPlot::SetupAxisLimits(ImAxis_X1, -600.0, 0.0, ImGuiCond_Always);
        if (!tecChartT_.empty()) {
            const float now = tecChartT_.back();
            for (float& t : tecChartT_) t -= now;   // relative time, 0 = newest
            const int n = static_cast<int>(tecChartT_.size());
            ImPlot::PlotLine("T", tecChartT_.data(), tecChartV_.data(), n);
            const double xs[2] = {-600.0, 0.0};
            const double ys[2] = {s.setC, s.setC};
            ImPlotSpec setSpec;
            setSpec.LineColor = ImVec4(0.9f, 0.7f, 0.2f, 0.8f);
            ImPlot::PlotLine("set", xs, ys, 2, setSpec);
        }
        ImPlot::EndPlot();
    }
}

void App::DrawTecInfoPanel()
{
    ImGui::TextUnformatted("Controller info");
    ImGui::Separator();
    if (hardware_.TecCount() == 0) {
        ImGui::TextDisabled("-");
        return;
    }

    const std::size_t idx = TecSelected();
    const ConnState   st  = hardware_.TecState(idx);
    ArroyoTec*        tec = hardware_.TecDevice(idx);

    ImGui::TextDisabled("Name");
    ImGui::TextUnformatted(hardware_.TecName(idx).c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    const std::string id = tec ? tec->DeviceId() : std::string();
    ImGui::TextWrapped("%s", id.empty() ? "-" : id.c_str());
    if (!tec) return;

    const ArroyoTec::Config& c = tec->GetConfig();
    ImGui::Spacing();
    ImGui::TextDisabled("Port");
    ImGui::Text("%s @ %d%s", c.port.c_str(), c.baud, c.simulated ? " (simulated)" : "");

    const ArroyoTec::Status s = tec->GetStatus();
    if (!s.valid) return;
    ImGui::Spacing();
    ImGui::TextDisabled("Mode");
    ImGui::TextUnformatted(s.mode.c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("Limits");
    ImGui::Text("T %.1f .. %.1f C", s.limTLoC, s.limTHiC);
    ImGui::Text("I %.2f A", s.limItA);
    ImGui::Spacing();
    ImGui::TextDisabled("Stable when");
    ImGui::Text("within +/-%.2f C for %.1f s", s.tolC, s.tolS);
    ImGui::Spacing();
    ImGui::TextDisabled("Condition");
    const std::string cond = ArroyoTec::CondText(s.cond);
    ImGui::TextWrapped("%u  %s", s.cond, cond.empty() ? "(clear)" : cond.c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("Last controller error");
    ImGui::TextWrapped("%s", s.lastUnitError.empty() ? "(none since connect)" : s.lastUnitError.c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("Polling");
    ImGui::TextUnformatted(tec->IsPolling() ? (s.linkUp ? "yes" : "yes - link down") : "no");
}

} // namespace firedog
