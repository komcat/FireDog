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

#include <algorithm>
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

// The chart's display low-pass presets: time constants (s) and their labels.
// Index 0 is off. A longer constant is a quieter trace and a later peak - an
// EMA delays a step by about tau, which is what the operator sees while jogging.
constexpr float       kChartSmoothTau[]   = {0.0f, 0.05f, 0.2f, 1.0f};
constexpr const char* kChartSmoothLabel[] = {"Off", "50 ms", "0.2 s", "1 s"};
constexpr int         kChartSmoothCount   = 4;

int ClampSmooth(int i) { return (i < 0 || i >= kChartSmoothCount) ? 0 : i; }

// Time-aware exponential moving average: alpha = 1 - exp(-dt / tau) per
// sample, so the corner frequency does not move with the poll rate (NPLC, the
// 6482 vs the 2602B). Seeded with the first sample; a gap of many tau (polling
// paused) simply lets the new sample through. tau <= 0 clears `out` (= off).
void SmoothEma(const std::vector<float>& t, const std::vector<float>& v, float tau,
               std::vector<float>& out)
{
    const size_t n = (std::min)(t.size(), v.size());
    if (tau <= 0.0f || n == 0) { out.clear(); return; }
    out.resize(n);
    float y = v[0];
    out[0] = y;
    for (size_t i = 1; i < n; ++i) {
        const float dt = t[i] - t[i - 1];
        const float a  = dt > 0.0f ? 1.0f - std::exp(-dt / tau) : 0.0f;
        y += a * (v[i] - y);
        out[i] = y;
    }
}
} // namespace

// The LP preset picker - on the chart's control row and on the Aux window's
// view strip (the compact chart has no controls). One setting, both places.
void App::DrawChartSmoothCombo()
{
    int sm = ClampSmooth(chartSmooth_);
    ImGui::SetNextItemWidth(ImGui::CalcTextSize("50 ms").x + ImGui::GetFrameHeight() * 1.6f);
    if (ImGui::Combo("LP", &sm, kChartSmoothLabel, kChartSmoothCount)) {
        chartSmooth_ = sm;
        SaveUiSettings();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Chart low-pass: smooths the trace and the printed\n"
                          "value (time constant shown). DISPLAY ONLY - recipes,\n"
                          "scans and the Keithley page's Live readout get the\n"
                          "raw samples. A longer constant lags a peak by about\n"
                          "that much. The instrument's own averaging is NPLC.");
}

std::string App::ChartSmoothTag() const
{
    const int i = ClampSmooth(chartSmooth_);
    return i == 0 ? std::string() : std::string("LP ") + kChartSmoothLabel[i];
}

// Fixed Y range by magnitude. The axis runs from the band holding the visible
// minimum to the band holding the visible maximum, so a flat signal sits still
// inside its band instead of being stretched to fill the plot on every sample
// (auto-fit), and a climb through a decade widens the axis at once.
//
// Narrowing back is held off until the data is 10 % of the narrower band clear
// of its edge: a signal hovering on 10 uA would otherwise flip between two
// axes every frame, which is the jumping this replaces. The window is seconds
// long, so a peak that has left still narrows the axis that much later.
//
// False = bands off, none configured, or a value inside no band -> the caller
// auto-fits as before (and the state resets, so the next band starts fresh).
bool App::ChartBandLimits(float dataLo, float dataHi, ChartBandState& st,
                          double& yLo, double& yHi) const
{
    const std::vector<ChartYBand>& b = chartYBandList_;
    const int n = static_cast<int>(b.size());
    auto bandOf = [&](double v) {
        for (int i = 0; i < n; ++i)
            if (v >= b[i].loUa * 1e-6 && v <= b[i].hiUa * 1e-6) return i;
        return -1;
    };
    int lo = chartYBands_ ? bandOf(dataLo) : -1;
    int hi = chartYBands_ ? bandOf(dataHi) : -1;
    if (lo < 0 || hi < 0) { st = ChartBandState{}; return false; }
    auto bLo = [&](int i) { return b[i].loUa * 1e-6; };
    auto bHi = [&](int i) { return b[i].hiUa * 1e-6; };
    if (lo > hi) std::swap(lo, hi);   // overlapping bands typed out of order

    // Hysteresis: keep last frame's wider end unless the data is well inside
    // the narrower one. A changed band list can leave stale indices - drop them.
    if (st.lo >= n || st.hi >= n) st = ChartBandState{};
    if (st.hi >= 0 && bHi(st.hi) > bHi(hi)) {
        const double margin = 0.10 * (bHi(hi) - bLo(hi));
        if (dataHi > bHi(hi) - margin) hi = st.hi;
    }
    if (st.lo >= 0 && bLo(st.lo) < bLo(lo)) {
        const double margin = 0.10 * (bHi(lo) - bLo(lo));
        if (dataLo < bLo(lo) + margin) lo = st.lo;
    }
    st.lo = lo;
    st.hi = hi;
    yLo = (std::min)(bLo(lo), bLo(hi));
    yHi = (std::max)(bHi(lo), bHi(hi));
    return yHi > yLo;
}

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
        if (HardwareConfig::SaveKeithleySettings(HardwareConfigPath(), keithleyId_,
                                                 keiNplc_, keiAutoRange_, fixed,
                                                 keiPollMs_, keiBias_))
            FD_INFO("app", "Keithley settings saved to %s", HardwareConfigPath().c_str());
        else
            FD_ERROR("app", "failed to save Keithley settings");
    }
    ImGui::TextDisabled("Save stores NPLC/range/poll/bias in this machine's config "
                        "(a build never overwrites it).");

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

    // Display low-pass, recomputed over the whole snapshot each frame (O(n),
    // and the buffer is seconds long): no filter state to go stale when the
    // stream restarts or the preset changes.
    const float tau = kChartSmoothTau[ClampSmooth(chartSmooth_)];
    SmoothEma(kT1_, kV1_, tau, kF1_);
    SmoothEma(kT2_, kV2_, tau, kF2_);

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
// the Keithley page (chartCh_/chartScope_/chartSmooth_ keep both views in
// sync). The trace AND the printed values are the smoothed series when the
// low-pass is on (ChartV1/2), so the number matches the line. Two looks:
// classic (axes + ticks) and scope (bare dark canvas, thick trace, legend
// swatch top-left, big live value bottom-left — the bobcat scope style).
void App::DrawKeithleyChart(const ImVec2& size)
{
    const ImVec4 kCh1Col(0.00f, 0.85f, 1.00f, 1.0f);   // cyan
    const ImVec4 kCh2Col(1.00f, 0.55f, 0.10f, 1.0f);   // orange

    const std::vector<float>& v1 = ChartV1();
    const std::vector<float>& v2 = ChartV2();
    const std::string lpTag = ChartSmoothTag();

    if (!chartScope_) {
        // Live readout row — in scope mode the value lives on the canvas instead.
        if (!v1.empty()) ImGui::TextColored(kCh1Col, "Ch1: %s", FormatCurrent(v1.back()).c_str());
        else             ImGui::TextDisabled("Ch1: --");
        ImGui::SameLine(0.0f, 16.0f);
        if (!v2.empty()) ImGui::TextColored(kCh2Col, "Ch2: %s", FormatCurrent(v2.back()).c_str());
        else               ImGui::TextDisabled("Ch2: --");
        ImGui::SameLine(0.0f, 16.0f);
        ImGui::Text("| %.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));
    }

    ImGui::RadioButton("Ch1", &chartCh_, 0); ImGui::SameLine();
    ImGui::RadioButton("Ch2", &chartCh_, 1); ImGui::SameLine();
    ImGui::RadioButton("Both", &chartCh_, 2); ImGui::SameLine(0.0f, 16.0f);
    if (ImGui::Checkbox("Scope", &chartScope_)) SaveUiSettings();
    ImGui::SameLine(0.0f, 16.0f);
    DrawChartSmoothCombo();
    if (chartScope_) {
        ImGui::SameLine(0.0f, 16.0f);
        ImGui::TextDisabled("%.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));
    }

    // The visible window's extent over the shown channel(s), for the fixed Y
    // bands. The same samples the trace draws: [now - window, now].
    double yLo = 0.0, yHi = 0.0;
    bool   fixedY = false;
    {
        const float tLo = static_cast<float>(chartClock_ - kChartWindowSec);
        float lo = FLT_MAX, hi = -FLT_MAX;
        auto extent = [&](const std::vector<float>& t, const std::vector<float>& v) {
            const std::size_t n = (std::min)(t.size(), v.size());
            for (std::size_t i = 0; i < n; ++i) {
                if (t[i] < tLo) continue;
                lo = (std::min)(lo, v[i]);
                hi = (std::max)(hi, v[i]);
            }
        };
        if (chartCh_ != 1) extent(kT1_, v1);
        if (chartCh_ != 0) extent(kT2_, v2);
        if (lo <= hi) fixedY = ChartBandLimits(lo, hi, chartBandSt_[0], yLo, yHi);
        else          chartBandSt_[0] = ChartBandState{};
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
        // Y is the fixed band range when one applies, else auto-fits to the
        // visible data; X is a fixed relative-time window: 0 s at the right
        // edge (now), running back to -kChartWindowSec.
        const ImPlotAxisFlags yFit = fixedY ? ImPlotAxisFlags_None : ImPlotAxisFlags_AutoFit;
        if (scope) {
            ImPlot::SetupAxes(nullptr, nullptr,
                              ImPlotAxisFlags_NoDecorations,
                              ImPlotAxisFlags_NoDecorations | yFit);
        } else {
            ImPlot::SetupAxes("t (s)", "current (A)",
                              ImPlotAxisFlags_None, yFit);
            ImPlot::SetupAxisFormat(ImAxis_X1, "%.0fs");
        }
        ImPlot::SetupAxisLimits(ImAxis_X1, -kChartWindowSec, 0.0, ImGuiCond_Always);
        if (fixedY) {
            // Headroom: the scope's overlay text (legend top-left, value
            // bottom-left) needs the room FitPadding gives an auto-fit.
            const double pad = (yHi - yLo) * (scope ? 0.20 : 0.03);
            ImPlot::SetupAxisLimits(ImAxis_Y1, yLo - pad, yHi + pad, ImGuiCond_Always);
        }

        const float weight = scope ? 3.0f : 2.0f;
        if (chartCh_ != 1 && !v1.empty()) {
            ShiftToRelative(kT1_, chartClock_, kX1_);
            const int n = static_cast<int>((std::min)(kX1_.size(), v1.size()));
            ImPlotSpec s1; s1.LineColor = kCh1Col; s1.LineWeight = weight;
            ImPlot::PlotLine("ch1", kX1_.data(), v1.data(), n, s1);
        }
        if (chartCh_ != 0 && !v2.empty()) {
            ShiftToRelative(kT2_, chartClock_, kX2_);
            const int n = static_cast<int>((std::min)(kX2_.size(), v2.size()));
            ImPlotSpec s2; s2.LineColor = kCh2Col; s2.LineWeight = weight;
            ImPlot::PlotLine("ch2", kX2_.data(), v2.data(), n, s2);
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
            const char* sep = lpTag.empty() ? "" : "  ";
            std::snprintf(l1, sizeof(l1), "%s Ch1%s%s", model.c_str(), sep, lpTag.c_str());
            std::snprintf(l2, sizeof(l2), "%s Ch2%s%s", model.c_str(), sep, lpTag.c_str());
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
            if (chartCh_ != 1) ValueRow(kCh1Col, v1);
            if (chartCh_ != 0) ValueRow(kCh2Col, v2);
            ImPlot::PopPlotClipRect();
        }
        ImPlot::EndPlot();
    }
    if (scope) { ImPlot::PopStyleVar(); ImPlot::PopStyleColor(3); }
}

} // namespace firedog
