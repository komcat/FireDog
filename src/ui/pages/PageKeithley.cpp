// PageKeithley.cpp : the SMU page + the shared chart pipeline.
// Definitions of App members — see App.h.
//
// The instrument behind this page is whichever ISmu config picked — a 6482
// picoammeter or a 2602B SourceMeter — so every label that names a model comes
// from ISmu::ModelName(), never a literal. The page itself is model-agnostic:
// both answer bias, NPLC, range and two channels of amps.
//
// The chart is one pipeline with two consumers: UpdateKeithleyChartData fills
// the buffers once per frame from App::Update, and DrawKeithleyChart renders
// them in both the Aux overlay and this page. Sample rate is governed by NPLC,
// not the poll interval — see the Keithley notes in CLAUDE.md.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/UiSettings.h"
#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"
#include "hardware/instruments/ISmu.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace firedog {

namespace {
// Rebase absolute timestamps to relative time (t - now) so the newest sample
// sits at 0 s and older samples go negative — a fixed 0/-1/-2s X axis.
void ShiftToRelative(const std::vector<float>& t, double now, std::vector<float>& out) {
    out.resize(t.size());
    for (size_t i = 0; i < t.size(); ++i) out[i] = static_cast<float>(t[i] - now);
}
} // namespace

// Model name of the configured SMU, for the headers and the chart legend. The
// driver exists from Workflow::Start, so this is right before the first connect.
std::string App::KeithleyModelName()
{
    ISmu* kei = hardware_.Keithley();
    return kei ? kei->ModelName() : std::string("SMU");
}

void App::DrawKeithleyListPanel()
{
    ImGui::TextUnformatted(KeithleyModelName().c_str());
    ImGui::Separator();
    if (keithleyId_.empty()) {
        ImGui::TextDisabled("no instrument configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.KeithleyState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.KeithleyName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectKeithley();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectKeithley();
        keiPulled_ = false;   // re-seed edits on the next connect
    }
    ImGui::EndDisabled();

    ISmu* kei = hardware_.Keithley();
    ImGui::Spacing();
    if (kei && kei->IsPolling())
        ImGui::Text("polling @ %.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));
    else
        ImGui::TextDisabled("not polling");

    if (st == ConnState::Failed && kei) {
        const std::string err = kei->LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawKeithleyDetailPanel()
{
    const ConnState st  = hardware_.KeithleyState();
    ISmu*           kei = hardware_.Keithley();
    const bool connected = (st == ConnState::Connected) && kei;

    if (connected && !keiPulled_) {   // seed edits from the live driver once
        const SmuConfig cfg = kei->GetConfig();
        keiNplc_      = static_cast<float>(cfg.nplc);
        keiAutoRange_ = cfg.autoRange;
        if (cfg.fixedRange > 0.0) keiRangeIdx_ = KeiRangeIndexFor(cfg.fixedRange);
        const double bias = kei->BiasVolts();
        if (bias != 0.0) keiBias_ = static_cast<float>(bias);
        keiPulled_ = true;
    }
    if (!connected) keiPulled_ = false;

    // ---- Bias output (:SOUR:VOLT + :OUTP) ----
    ImGui::TextUnformatted("Bias output");
    ImGui::Separator();
    ImGui::BeginDisabled(!connected);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("V##bias", &keiBias_, 0.0f, 0.0f, "%.3f");
    if (keiBias_ >  10.0f) keiBias_ =  10.0f;   // photodiode-bias sanity clamp
    if (keiBias_ < -10.0f) keiBias_ = -10.0f;
    ImGui::SameLine();
    // Symmetric pair: ON applies the bias voltage AND enables the output.
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
    if (ImGui::Button("Output ON")) workflow_.SetKeithleyBias(keiBias_);
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button("Output OFF")) workflow_.KeithleyOutputOff();
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (connected) {
        if (kei->OutputOn())
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f),
                               "output ON @ %.3f V", kei->BiasVolts());
        else
            ImGui::TextDisabled("output OFF");
    } else {
        ImGui::TextDisabled("connect the instrument to control it");
    }

    ImGui::Spacing();
    // ---- Measurement settings (applied live; safe against the poll loop) ----
    ImGui::TextUnformatted("Measurement");
    ImGui::Separator();
    ImGui::BeginDisabled(!connected);

    ImGui::TextUnformatted("NPLC");
    static constexpr float kNplcPresets[] = {0.01f, 0.1f, 1.0f, 5.0f, 10.0f};
    for (int i = 0; i < 5; ++i) {
        if (i > 0) ImGui::SameLine();
        const float v   = kNplcPresets[i];
        const bool  sel = std::fabs(keiNplc_ - v) < v * 0.01f;
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        char lbl[16];
        std::snprintf(lbl, sizeof(lbl), "%g", v);
        if (ImGui::Button(lbl, ImVec2(52.0f, 0.0f))) {
            keiNplc_ = v;
            workflow_.SetKeithleyNplc(v);
        }
        if (sel) ImGui::PopStyleColor();
    }
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("##nplcIn", &keiNplc_, 0.0f, 0.0f, "%.3f");
    ImGui::SameLine();
    if (ImGui::Button("Apply##nplc")) workflow_.SetKeithleyNplc(keiNplc_);
    ImGui::TextDisabled("NPLC governs the sample rate: 0.1 = ~80 Hz, 1 = ~30 Hz, 5 = ~12 Hz");

    ImGui::Spacing();
    if (ImGui::Checkbox("Auto range", &keiAutoRange_)) {
        if (keiAutoRange_) workflow_.SetKeithleyRangeAuto();
        else               workflow_.SetKeithleyRangeFixed(kKeiRanges[keiRangeIdx_].amps);
    }
    if (!keiAutoRange_) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        if (ImGui::BeginCombo("##keirange", kKeiRanges[keiRangeIdx_].label)) {
            for (int i = 0; i < kKeiRangeCount; ++i) {
                if (ImGui::Selectable(kKeiRanges[i].label, keiRangeIdx_ == i)) {
                    keiRangeIdx_ = i;
                    workflow_.SetKeithleyRangeFixed(kKeiRanges[i].amps);
                }
            }
            ImGui::EndCombo();
        }
    }

    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputInt("Poll ms (0 = max)", &keiPollMs_);
    if (keiPollMs_ < 0) keiPollMs_ = 0;
    ImGui::SameLine();
    if (ImGui::Button("Apply##poll")) workflow_.SetKeithleyPollInterval(keiPollMs_);

    ImGui::EndDisabled();

    ImGui::Spacing();
    if (ImGui::Button("Save to config")) {
        const double fixed = keiAutoRange_ ? 0.0 : kKeiRanges[keiRangeIdx_].amps;
        if (HardwareConfig::SaveKeithleySettings("config/hardware.json", keithleyId_,
                                                 keiNplc_, keiAutoRange_, fixed,
                                                 keiPollMs_, keiBias_))
            FD_INFO("app", "Keithley settings saved to config/hardware.json");
        else
            FD_ERROR("app", "failed to save Keithley settings");
    }
    ImGui::TextDisabled("Save stores NPLC/range/poll/bias; a rebuild recopies the repo "
                        "config over the exe-side file.");

    ImGui::Spacing();
    // ---- Live chart (shared with the Aux overlay) ----
    ImGui::TextUnformatted("Live");
    ImGui::Separator();
    DrawKeithleyChart(ImVec2(-1, -1));   // fill the rest of the panel
}

void App::DrawKeithleyInfoPanel()
{
    ImGui::TextUnformatted("Instrument info");
    ImGui::Separator();

    const ConnState st  = hardware_.KeithleyState();
    ISmu*           kei = hardware_.Keithley();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    ImGui::TextWrapped("%s", (kei && !kei->DeviceId().empty()) ? kei->DeviceId().c_str() : "-");

    if (kei) {
        const SmuConfig cfg = kei->GetConfig();
        ImGui::TextDisabled("VISA");
        ImGui::TextWrapped("%s", cfg.visaResource.c_str());
        ImGui::Spacing();
        ImGui::TextDisabled("Settings");
        ImGui::Text("NPLC %.3g", cfg.nplc);
        if (cfg.autoRange) ImGui::TextUnformatted("range: auto");
        else               ImGui::Text("range: %.0e A", cfg.fixedRange);
        if (kei->OutputOn()) ImGui::Text("bias ON @ %.3f V", kei->BiasVolts());
        else                 ImGui::TextUnformatted("bias output OFF");
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Live");
    if (!kV1_.empty())
        ImGui::TextColored(ImVec4(0.00f, 0.85f, 1.00f, 1.0f), "Ch1 %s",
                           FormatCurrent(kV1_.back()).c_str());
    else ImGui::TextDisabled("Ch1 --");
    if (!kV2_.empty())
        ImGui::TextColored(ImVec4(1.00f, 0.55f, 0.10f, 1.0f), "Ch2 %s",
                           FormatCurrent(kV2_.back()).c_str());
    else ImGui::TextDisabled("Ch2 --");
    if (kei && kei->IsPolling())
        ImGui::Text("%.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));
}

// Per-frame pull of the Keithley streams + rolling-clock update. Called once
// from Update() so every chart view (Aux overlay + Keithley page) renders the
// same buffers without double-advancing the clock.
void App::UpdateKeithleyChartData(float dt)
{
    DataBus::Instance().Stream(topics::kKeithleyCh1).Snapshot(kT1_, kV1_);
    DataBus::Instance().Stream(topics::kKeithleyCh2).Snapshot(kT2_, kV2_);

    // chartClock_ is the rolling "now" the data is rebased against; it advances
    // by real frame time and re-syncs to the newest sample.
    const std::vector<float>& tref = kT1_.empty() ? kT2_ : kT1_;
    if (!tref.empty()) {
        const double last = tref.back();
        chartClock_ += dt;
        if (last > chartClock_ || chartClock_ - last > 0.5) chartClock_ = last;
    }
}

// Readout + channel selector + rolling chart — shared by the Aux overlay and
// the Keithley page (chartCh_/chartScope_ keep both views in sync). Two looks:
// classic (axes + ticks) and scope (bare dark canvas, thick trace, legend
// swatch top-left, big live value bottom-left — the bobcat scope style).
void App::DrawKeithleyChart(const ImVec2& size)
{
    const ImVec4 kCh1Col(0.00f, 0.85f, 1.00f, 1.0f);   // cyan
    const ImVec4 kCh2Col(1.00f, 0.55f, 0.10f, 1.0f);   // orange

    if (!chartScope_) {
        // Live readout row — in scope mode the value lives on the canvas instead.
        if (!kV1_.empty()) ImGui::TextColored(kCh1Col, "Ch1: %s", FormatCurrent(kV1_.back()).c_str());
        else               ImGui::TextDisabled("Ch1: --");
        ImGui::SameLine(0.0f, 16.0f);
        if (!kV2_.empty()) ImGui::TextColored(kCh2Col, "Ch2: %s", FormatCurrent(kV2_.back()).c_str());
        else               ImGui::TextDisabled("Ch2: --");
        ImGui::SameLine(0.0f, 16.0f);
        ImGui::Text("| %.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));
    }

    ImGui::RadioButton("Ch1", &chartCh_, 0); ImGui::SameLine();
    ImGui::RadioButton("Ch2", &chartCh_, 1); ImGui::SameLine();
    ImGui::RadioButton("Both", &chartCh_, 2); ImGui::SameLine(0.0f, 16.0f);
    if (ImGui::Checkbox("Scope", &chartScope_)) SaveUiSettings();
    if (chartScope_) {
        ImGui::SameLine(0.0f, 16.0f);
        ImGui::TextDisabled("%.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));
    }

    const bool scope = chartScope_;
    if (scope) {
        ImPlot::PushStyleColor(ImPlotCol_FrameBg,    ImVec4(0.16f, 0.17f, 0.19f, 1.0f));
        ImPlot::PushStyleColor(ImPlotCol_PlotBg,     ImVec4(0.16f, 0.17f, 0.19f, 1.0f));
        ImPlot::PushStyleColor(ImPlotCol_PlotBorder, ImVec4(0, 0, 0, 0));
        // Pad the Y auto-fit so the trace stays clear of the overlay text
        // (legend top-left, value readout bottom-left).
        ImPlot::PushStyleVar(ImPlotStyleVar_FitPadding, ImVec2(0.0f, 0.30f));
    }
    const ImPlotFlags pflags = scope ? ImPlotFlags_CanvasOnly : ImPlotFlags_None;
    if (ImPlot::BeginPlot("##keithley", size, pflags)) {
        // Y auto-fits to the visible data; X is a fixed relative-time window:
        // 0 s at the right edge (now), running back to -kChartWindowSec.
        if (scope) {
            ImPlot::SetupAxes(nullptr, nullptr,
                              ImPlotAxisFlags_NoDecorations,
                              ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit);
        } else {
            ImPlot::SetupAxes("t (s)", "current (A)",
                              ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisFormat(ImAxis_X1, "%.0fs");
        }
        ImPlot::SetupAxisLimits(ImAxis_X1, -kChartWindowSec, 0.0, ImGuiCond_Always);

        const float weight = scope ? 3.0f : 2.0f;
        if (chartCh_ != 1 && !kV1_.empty()) {
            ShiftToRelative(kT1_, chartClock_, kX1_);
            ImPlotSpec s1; s1.LineColor = kCh1Col; s1.LineWeight = weight;
            ImPlot::PlotLine("ch1", kX1_.data(), kV1_.data(), static_cast<int>(kV1_.size()), s1);
        }
        if (chartCh_ != 0 && !kV2_.empty()) {
            ShiftToRelative(kT2_, chartClock_, kX2_);
            ImPlotSpec s2; s2.LineColor = kCh2Col; s2.LineWeight = weight;
            ImPlot::PlotLine("ch2", kX2_.data(), kV2_.data(), static_cast<int>(kV2_.size()), s2);
        }

        if (scope) {
            // Overlay drawn straight on the plot canvas: per-channel legend
            // swatch top-left, live value(s) bottom-left in a larger size.
            ImDrawList*  dl = ImPlot::GetPlotDrawList();
            const ImVec2 p0 = ImPlot::GetPlotPos();
            const ImVec2 ps = ImPlot::GetPlotSize();
            ImPlot::PushPlotClipRect();
            const float fs  = ImGui::GetFontSize();
            const float pad = 8.0f;

            float ly = p0.y + pad;
            auto LegendRow = [&](const ImVec4& c, const char* label) {
                const float  box = fs * 0.55f;
                const ImVec2 b0(p0.x + pad, ly + (fs - box) * 0.5f);
                dl->AddRectFilled(b0, ImVec2(b0.x + box, b0.y + box),
                                  ImGui::ColorConvertFloat4ToU32(c));
                dl->AddText(ImVec2(b0.x + box + 6.0f, ly), IM_COL32(235, 235, 235, 255), label);
                ly += fs + 2.0f;
            };
            const std::string model = KeithleyModelName();
            char l1[64], l2[64];
            std::snprintf(l1, sizeof(l1), "%s Ch1", model.c_str());
            std::snprintf(l2, sizeof(l2), "%s Ch2", model.c_str());
            if (chartCh_ != 1) LegendRow(kCh1Col, l1);
            if (chartCh_ != 0) LegendRow(kCh2Col, l2);

            const float vfs = fs * 1.35f;
            float vy = p0.y + ps.y - pad - vfs;
            auto ValueRow = [&](const ImVec4& c, const std::vector<float>& v) {
                const std::string txt = v.empty() ? "--" : FormatCurrent(v.back());
                dl->AddText(ImGui::GetFont(), vfs, ImVec2(p0.x + pad, vy),
                            ImGui::ColorConvertFloat4ToU32(c), txt.c_str());
                vy -= vfs + 2.0f;
            };
            // Bottom-up: ch1 sits at the very bottom, ch2 stacks above it.
            if (chartCh_ != 1) ValueRow(kCh1Col, kV1_);
            if (chartCh_ != 0) ValueRow(kCh2Col, kV2_);
            ImPlot::PopPlotClipRect();
        }
        ImPlot::EndPlot();
    }
    if (scope) { ImPlot::PopStyleVar(); ImPlot::PopStyleColor(3); }
}

} // namespace firedog
