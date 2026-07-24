// App.cpp : layer 3 (UI) implementation.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "core/DataBus.h"
#include "core/Topics.h"
#include "core/Log.h"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace firedog {

namespace {
constexpr int kWinW = 1280;
constexpr int kWinH = 800;

// User-adjustable UI prefs persist here (gitignored .local.json per convention),
// written next to the exe alongside the copied config/ folder.
constexpr const char* kUiSettingsPath = "config/ui.local.json";

// Selectable UI fonts. Paths are Windows system fonts (this is a Windows-only
// project); "" = ImGui's built-in font. Keep index 0 as the startup default.
struct FontOption { const char* label; const char* path; };
constexpr FontOption kFonts[] = {
    {"Segoe UI", "C:/Windows/Fonts/segoeui.ttf"},
    {"Tahoma",   "C:/Windows/Fonts/tahoma.ttf"},
    {"Verdana",  "C:/Windows/Fonts/verdana.ttf"},
    {"Consolas", "C:/Windows/Fonts/consola.ttf"},
    {"Built-in", ""},
};
constexpr int kFontCount = static_cast<int>(sizeof(kFonts) / sizeof(kFonts[0]));

// Jog step sizes (label + value in mm).
struct StepSize { const char* label; double mm; };
constexpr StepSize kSteps[] = {
    {"0.1um", 0.0001}, {"0.2um", 0.0002}, {"0.5um", 0.0005},
    {"1um",   0.001},  {"2um",   0.002},  {"5um",   0.005},
    {"10um",  0.01},   {"25um",  0.025},  {"50um",  0.05},
    {"100um", 0.1},    {"250um", 0.25},   {"500um", 0.5},
    {"1mm",   1.0},    {"2mm",   2.0},    {"5mm",   5.0},
};
constexpr int kStepCount = static_cast<int>(sizeof(kSteps) / sizeof(kSteps[0]));

// Jog velocities (mm/s) per controller type.
constexpr double kVelPI[]  = {1.0, 2.0, 5.0, 10.0};
constexpr double kVelACS[] = {1.0, 5.0, 10.0, 20.0, 40.0};

ImVec4 StateColor(ConnState s) {
    switch (s) {
        case ConnState::Connected:  return ImVec4(0.40f, 0.90f, 0.50f, 1.0f);
        case ConnState::Connecting: return ImVec4(0.95f, 0.80f, 0.30f, 1.0f);
        case ConnState::Failed:     return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
        default:                    return ImVec4(0.60f, 0.60f, 0.60f, 1.0f);
    }
}
const char* StateGlyph(ConnState s) {
    switch (s) {
        case ConnState::Connected:  return "[+]";
        case ConnState::Connecting: return "[~]";
        case ConnState::Failed:     return "[x]";
        default:                    return "[ ]";
    }
}
const char* StateText(ConnState s) {
    switch (s) {
        case ConnState::Connected:  return "connected";
        case ConnState::Connecting: return "connecting...";
        case ConnState::Failed:     return "failed";
        default:                    return "disconnected";
    }
}

ImVec4 LevelColor(LogLevel l) {
    switch (l) {
        case LogLevel::Trace: return ImVec4(0.55f, 0.55f, 0.58f, 1.0f);
        case LogLevel::Debug: return ImVec4(0.60f, 0.75f, 0.95f, 1.0f);
        case LogLevel::Info:  return ImVec4(0.85f, 0.88f, 0.90f, 1.0f);
        case LogLevel::Warn:  return ImVec4(0.95f, 0.80f, 0.30f, 1.0f);
        case LogLevel::Error: return ImVec4(0.97f, 0.45f, 0.45f, 1.0f);
    }
    return ImVec4(1, 1, 1, 1);
}

// A draggable splitter bar (a thin, invisible-until-hovered button). Returns
// true while being dragged; the caller applies io.MouseDelta to the adjacent
// panel size. `vertical` = a tall bar that resizes left/right; otherwise a wide
// bar that resizes up/down.
bool Splitter(const char* id, bool vertical, float thickness, float length) {
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.30f, 0.30f, 0.34f, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.45f, 0.45f, 0.55f, 0.6f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.55f, 0.55f, 0.70f, 0.9f));
    ImGui::Button(id, vertical ? ImVec2(thickness, length) : ImVec2(length, thickness));
    const bool active = ImGui::IsItemActive();
    if (active || ImGui::IsItemHovered())
        ImGui::SetMouseCursor(vertical ? ImGuiMouseCursor_ResizeEW : ImGuiMouseCursor_ResizeNS);
    ImGui::PopStyleColor(3);
    return active;
}

void DeviceStatusLine(ConnState s, const char* name) {
    ImGui::TextColored(StateColor(s), "%s", StateGlyph(s));
    ImGui::SameLine();
    ImGui::Text("%-12s %s", name, StateText(s));
}

// Format an amp value with an auto-scaled unit (mA / uA / nA / pA).
std::string FormatCurrent(double a) {
    const double m = std::fabs(a);
    const char* unit; double scale;
    if      (m >= 1e-3) { unit = "mA"; scale = 1e3;  }
    else if (m >= 1e-6) { unit = "uA"; scale = 1e6;  }
    else if (m >= 1e-9) { unit = "nA"; scale = 1e9;  }
    else                { unit = "pA"; scale = 1e12; }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f %s", a * scale, unit);
    return buf;
}

// Achieved sample rate from the samples in the last 1 s of a stream snapshot.
float MeasuredHz(const std::vector<float>& t) {
    if (t.size() < 2) return 0.0f;
    const float last = t.back();
    const float cut  = last - 1.0f;
    size_t i = t.size();
    while (i > 0 && t[i - 1] >= cut) --i;   // i = first index inside the window
    const size_t count = t.size() - i;
    const float  span  = last - t[i];
    if (span <= 0.0f || count < 2) return 0.0f;
    return static_cast<float>(count - 1) / span;
}

// Rebase absolute timestamps to relative time (t - now) so the newest sample
// sits at 0 s and older samples go negative — a fixed 0/-1/-2s X axis.
void ShiftToRelative(const std::vector<float>& t, double now, std::vector<float>& out) {
    out.resize(t.size());
    for (size_t i = 0; i < t.size(); ++i) out[i] = static_cast<float>(t[i] - now);
}
} // namespace

int App::Run()
{
    FD_INFO("app", "FireDog starting");
    Logger::Instance().SetMinLevel(static_cast<LogLevel>(logMinLevel_));
    if (!host_.Init("FireDog - Active Alignment", kWinW, kWinH)) {
        FD_ERROR("app", "GlHost init failed");
        return 1;
    }
    Theme::Apply();
    // Restore the saved font choice + size (defaults to Segoe UI 26 px on first
    // run), validated against the current font table / slider range.
    {
        const UiSettings ui = UiSettings::Load(kUiSettingsPath);
        fontIndex_  = (ui.fontIndex >= 0 && ui.fontIndex < kFontCount) ? ui.fontIndex : 0;
        uiFontSize_ = (ui.fontSize >= 8.0f && ui.fontSize <= 48.0f) ? ui.fontSize : 26.0f;
        // Panel sizes: keep saved px if present; 0 lets DrawMainWindow seed them.
        if (ui.leftW  > 0.0f) leftW_  = ui.leftW;
        if (ui.rightW > 0.0f) rightW_ = ui.rightW;
        if (ui.lowerH > 0.0f) lowerH_ = ui.lowerH;
        showChart_       = ui.showChart;
        enableClickMove_ = ui.clickToMove;
    }
    host_.SetFont(kFonts[fontIndex_].path, uiFontSize_);

    // Build a placeholder frame (checkerboard) for the main-window vision view.
    constexpr int IW = 640, IH = 480, CELL = 32;
    std::vector<unsigned char> px(static_cast<size_t>(IW) * IH * 4);
    for (int y = 0; y < IH; ++y) {
        for (int x = 0; x < IW; ++x) {
            const bool on = ((x / CELL) + (y / CELL)) & 1;
            const unsigned char v = on ? 90 : 55;
            const size_t i = (static_cast<size_t>(y) * IW + x) * 4;
            px[i] = v; px[i + 1] = v; px[i + 2] = static_cast<unsigned char>(v + 12); px[i + 3] = 255;
        }
    }
    viewportTex_ = host_.MakeTextureRGBA(IW, IH, px.data());

    objects_.push_back({ImVec2(120.0f,  90.0f), "Lens",  IM_COL32( 90, 200, 250, 255)});
    objects_.push_back({ImVec2(260.0f, 150.0f), "Fiber", IM_COL32(250, 190,  90, 255)});
    objects_.push_back({ImVec2(400.0f, 100.0f), "DUT",   IM_COL32(160, 250, 130, 255)});

    const HardwareConfig hw = HardwareConfig::Load("config/hardware.json");
    if (const CameraConfig* cam = hw.FirstEnabledCamera()) {
        pixelToMmX_ = static_cast<float>(cam->pixelToMmX);
        pixelToMmY_ = static_cast<float>(cam->pixelToMmY);
    }
    workflow_.SetTransforms(LoadMotionTransforms("config/motion_transforms.json"));
    hardware_.ConnectAllAsync();   // simulated instruments (motion/etc.)
    workflow_.Start(hw);           // layer 2 brings camera + Keithley online

    double prev = host_.Time();
    while (!host_.ShouldClose()) {
        const double now = host_.Time();
        Update(static_cast<float>(now - prev));
        prev = now;

        host_.BeginFrame();
        DrawMainWindow();
        if (showAux_) DrawAuxWindow();
        if (showJog_) DrawJogWindow();
        host_.EndFrame();
    }

    SaveUiSettings();   // backstop in case the last change wasn't flushed
    FD_INFO("app", "shutting down");
    host_.Shutdown();
    return 0;
}

void App::Update(float dt)
{
    workflow_.Update(dt);   // layer 2 (kept for future orchestration)
}

void App::DrawMainWindow()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(mv->Pos);
    ImGui::SetNextWindowSize(mv->Size);
    if (ImGui::Begin("Main", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_MenuBar)) {
        // ---- Menu bar placeholder (top strip) ----
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                ImGui::MenuItem("Exit", nullptr, false, false);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Hardware")) {
                if (ImGui::MenuItem("Overview", nullptr, currentPage_ == Page::Overview))
                    currentPage_ = Page::Overview;
                if (ImGui::MenuItem("PI",       nullptr, currentPage_ == Page::PI))
                    currentPage_ = Page::PI;
                if (ImGui::MenuItem("ACS",      nullptr, currentPage_ == Page::ACS))
                    currentPage_ = Page::ACS;
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Aux Window", nullptr, &showAux_);
                ImGui::MenuItem("Jog",        nullptr, &showJog_);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Help")) {
                ImGui::MenuItem("About", nullptr, false, false);
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        // ---- Panel layout (user-resizable via splitter bars) ----
        //   left | [ middle / lower ] | right. Sizes persist in leftW_/rightW_/
        //   lowerH_ (px): seeded from a proportion on the first frame, nudged by
        //   the splitter drags, and clamped every frame so window resizes and
        //   drags both stay sane. SameLine(0,0) keeps columns flush so the two
        //   vertical splitters are the only things between them (2*sp wide);
        //   the center column adds 2*ItemSpacing.y around its horizontal splitter.
        const ImVec2   avail = ImGui::GetContentRegionAvail();
        const float    sp    = 6.0f;                              // splitter thickness
        const float    gapY  = ImGui::GetStyle().ItemSpacing.y;   // vertical stack gap
        const ImGuiIO& io    = ImGui::GetIO();

        if (leftW_  <= 0.0f) leftW_  = avail.x * 0.20f;
        if (rightW_ <= 0.0f) rightW_ = avail.x * 0.15f;
        if (lowerH_ <= 0.0f) lowerH_ = avail.y * 0.15f;

        auto clampf = [](float v, float lo, float hi) {
            if (hi < lo) hi = lo;
            return v < lo ? lo : (v > hi ? hi : v);
        };
        constexpr float kMinSide = 80.0f, kMinCenter = 160.0f, kMinMid = 80.0f, kMinLower = 48.0f;
        leftW_  = clampf(leftW_,  kMinSide,  avail.x - rightW_ - 2.0f * sp - kMinCenter);
        rightW_ = clampf(rightW_, kMinSide,  avail.x - leftW_  - 2.0f * sp - kMinCenter);
        lowerH_ = clampf(lowerH_, kMinLower, avail.y - sp - 2.0f * gapY - kMinMid);

        const float centerW = avail.x - leftW_ - rightW_ - 2.0f * sp;
        const float midH    = avail.y - lowerH_ - sp - 2.0f * gapY;

        // Left column (content depends on the selected Hardware page)
        ImGui::BeginChild("LeftPanel", ImVec2(leftW_, avail.y), ImGuiChildFlags_Borders);
        switch (currentPage_) {
            case Page::PI:  DrawMotionListPanel("PI");  break;
            case Page::ACS: DrawMotionListPanel("ACS"); break;
            default:        DrawLeftPanel();            break;
        }
        ImGui::EndChild();

        ImGui::SameLine(0.0f, 0.0f);
        bool splitterActive = false;
        if (Splitter("##splitL", true, sp, avail.y)) { leftW_ += io.MouseDelta.x; splitterActive = true; }
        ImGui::SameLine(0.0f, 0.0f);

        // Center column: middle over lower, split by a horizontal bar
        ImGui::BeginGroup();
        ImGui::BeginChild("MiddlePanel", ImVec2(centerW, midH), ImGuiChildFlags_Borders);
        switch (currentPage_) {
            case Page::PI:  DrawMotionDetailPanel("PI");  break;
            case Page::ACS: DrawMotionDetailPanel("ACS"); break;
            default:        DrawMiddlePanel();            break;
        }
        ImGui::EndChild();
        if (Splitter("##splitM", false, sp, centerW)) { lowerH_ -= io.MouseDelta.y; splitterActive = true; }
        ImGui::BeginChild("LowerPanel", ImVec2(centerW, lowerH_), ImGuiChildFlags_Borders);
        DrawLowerPanel();
        ImGui::EndChild();
        ImGui::EndGroup();

        ImGui::SameLine(0.0f, 0.0f);
        if (Splitter("##splitR", true, sp, avail.y)) { rightW_ -= io.MouseDelta.x; splitterActive = true; }
        ImGui::SameLine(0.0f, 0.0f);

        // Right column (content depends on the selected Hardware page)
        ImGui::BeginChild("RightPanel", ImVec2(rightW_, avail.y), ImGuiChildFlags_Borders);
        switch (currentPage_) {
            case Page::PI:  DrawMotionInfoPanel("PI");  break;
            case Page::ACS: DrawMotionInfoPanel("ACS"); break;
            default:        DrawRightPanel();           break;
        }
        ImGui::EndChild();

        // Persist panel sizes once a splitter drag finishes (shared across pages).
        if (splitterWasActive_ && !splitterActive) SaveUiSettings();
        splitterWasActive_ = splitterActive;
    }
    ImGui::End();
}

void App::DrawLeftPanel()
{
    ImGui::TextUnformatted("Hardware");
    ImGui::Separator();
    for (const auto& d : hardware_.Snapshot()) {   // includes camera + Keithley
        DeviceStatusLine(d.state, d.name.c_str());
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Display")) {
        const char* names[kFontCount];
        for (int i = 0; i < kFontCount; ++i) names[i] = kFonts[i].label;

        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::Combo("##font", &fontIndex_, names, kFontCount)) {
            host_.SetFont(kFonts[fontIndex_].path, uiFontSize_);
            SaveUiSettings();                        // discrete change — save now
        }
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::SliderFloat("##fontsize", &uiFontSize_, 10.0f, 28.0f, "%.0f px")) {
            host_.SetFont(kFonts[fontIndex_].path, uiFontSize_);
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveUiSettings();   // save once on release
    }
}

void App::SaveUiSettings()
{
    UiSettings s;
    s.fontIndex = fontIndex_;
    s.fontSize  = uiFontSize_;
    s.leftW     = leftW_;
    s.rightW    = rightW_;
    s.lowerH    = lowerH_;
    s.showChart   = showChart_;
    s.clickToMove = enableClickMove_;
    s.Save(kUiSettingsPath);
}

void App::DrawMiddlePanel()
{
    DrawImageOverlay();
}

void App::DrawLowerPanel()
{
    DrawLogPanel();
}

void App::DrawRightPanel()
{
    ImGui::TextUnformatted("Overview");
    ImGui::Separator();
    ImGui::TextWrapped("Jog controls are in their own window — toggle from View > Jog.");
}

void App::DrawJogWindow()
{
    // Spawns near the top-right (visible); drag it out to make it its own OS
    // window (multi-viewport). &showJog_ gives it a close box that syncs the menu.
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + mv->Size.x - 262.0f, mv->Pos.y + 60.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250.0f, 640.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Jog", &showJog_)) {
        DrawJogPanel();
    }
    ImGui::End();
}

// Jog controls (its own window). Buttons + W/D·A/S·R/F keys route through the
// workflow, which maps global direction -> local axis via the device transform.
// Everything but Stop is gated by the Arm toggle.
void App::DrawJogPanel()
{
    const size_t n = hardware_.MotionCount();
    if (n == 0) { ImGui::TextDisabled("no motion devices"); return; }
    if (jogDevice_ < 0 || static_cast<size_t>(jogDevice_) >= n) jogDevice_ = 0;

    // --- Active device list ---
    ImGui::TextUnformatted("Device");
    ImGui::BeginChild("jogdevs", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * 3.4f),
                      ImGuiChildFlags_Borders);
    for (size_t i = 0; i < n; ++i) {
        IMotionController* d = hardware_.MotionDevice(i);
        if (!d) continue;
        const ConnState st = hardware_.MotionState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        if (ImGui::Selectable(hardware_.MotionName(i).c_str(), jogDevice_ == static_cast<int>(i)))
            jogDevice_ = static_cast<int>(i);
        ImGui::PopID();
    }
    ImGui::EndChild();

    IMotionController* dev = hardware_.MotionDevice(static_cast<size_t>(jogDevice_));
    if (!dev) { ImGui::TextDisabled("select a device"); return; }
    const std::string              devType = dev->Type();
    const std::vector<std::string> axes    = dev->Axes();
    const size_t                   di      = static_cast<size_t>(jogDevice_);
    dev->RequestFastPoll();   // keep positions fresh for accurate relative jogs

    // --- Arm toggle + Stop (Stop is always active) ---
    ImGui::Checkbox("Arm", &jogArmed_);
    ImGui::SameLine();
    if (jogArmed_) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "ARMED");
    else           ImGui::TextDisabled("disarmed");
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("STOP", ImVec2(-FLT_MIN, 30.0f))) workflow_.StopMotion(di);
    ImGui::PopStyleColor(2);
    ImGui::Separator();

    // --- Velocity combo (options depend on controller type) ---
    const double* vels = (devType == "ACS") ? kVelACS : kVelPI;
    const int     velN = (devType == "ACS") ? 5 : 4;
    if (jogVelIndex_ >= velN) jogVelIndex_ = 0;
    // Push the selected velocity to the (connected) device when it changes.
    if (jogVelDevice_ != jogDevice_ && dev->IsConnected()) {
        workflow_.SetJogVelocity(di, vels[jogVelIndex_]);
        jogVelDevice_ = jogDevice_;
    }
    ImGui::TextUnformatted("Velocity (mm/s)");
    ImGui::SetNextItemWidth(-FLT_MIN);
    char velPrev[16]; std::snprintf(velPrev, sizeof(velPrev), "%g", vels[jogVelIndex_]);
    if (ImGui::BeginCombo("##vel", velPrev)) {
        for (int i = 0; i < velN; ++i) {
            char b[16]; std::snprintf(b, sizeof(b), "%g", vels[i]);
            if (ImGui::Selectable(b, jogVelIndex_ == i)) {
                jogVelIndex_ = i;
                workflow_.SetJogVelocity(di, vels[i]);
                jogVelDevice_ = jogDevice_;
            }
        }
        ImGui::EndCombo();
    }

    // --- Step size button array ---
    ImGui::TextUnformatted("Step size");
    const float bw    = 54.0f;
    const float availw = ImGui::GetContentRegionAvail().x;
    int perRow = static_cast<int>(availw / (bw + 6.0f));
    if (perRow < 1) perRow = 1;
    for (int i = 0; i < kStepCount; ++i) {
        if (i % perRow != 0) ImGui::SameLine();
        const bool sel = (jogStepIndex_ == i);
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        if (ImGui::Button(kSteps[i].label, ImVec2(bw, 0.0f))) jogStepIndex_ = i;
        if (sel) ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // --- Translational cluster (global X/Y/Z), routed through the workflow ---
    // Left/Right = X-/X+ (A/D), In/Out = Y-/Y+ (W/S), Up/Down = Z-/Z+ (R/F). The
    // device transform (in Workflow) maps global direction -> local axis + sign.
    const double step = kSteps[jogStepIndex_].mm;
    const ImVec2 jb(58.0f, 0.0f);   // auto height -> fits the two-line "In\nW" labels
    ImGui::BeginDisabled(!jogArmed_);
    ImGui::Dummy(ImVec2(jb.x + 8.0f, 0.0f)); ImGui::SameLine();
    if (ImGui::Button("In\nW", jb))    workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);
    if (ImGui::Button("Left\nA", jb))  workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step); ImGui::SameLine();
    if (ImGui::Button("Out\nS", jb))   workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step); ImGui::SameLine();
    if (ImGui::Button("Right\nD", jb)) workflow_.JogGlobal(di, 1.0f,  0.0f, 0.0f, step);
    if (ImGui::Button("Up\nR", jb))    workflow_.JogGlobal(di, 0.0f,  0.0f, -1.0f, step); ImGui::SameLine();
    if (ImGui::Button("Down\nF", jb))  workflow_.JogGlobal(di, 0.0f,  0.0f, 1.0f, step);

    // --- Rotational cluster (local U/V/W...), only if the device has them ---
    const int rotCount = static_cast<int>(axes.size()) - 3;
    if (rotCount > 0) {
        if (jogRotAxis_ < 0 || jogRotAxis_ >= rotCount) jogRotAxis_ = 0;
        ImGui::Separator();
        ImGui::TextUnformatted("Rotational");
        ImGui::BeginGroup();
        for (int i = 0; i < rotCount; ++i) {
            const bool sel = (jogRotAxis_ == i);
            if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.55f, 0.10f, 1.0f));
            if (ImGui::Button(axes[3 + i].c_str(), ImVec2(jb.x, 0.0f))) jogRotAxis_ = i;
            if (sel) ImGui::PopStyleColor();
        }
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("+##rot", ImVec2(jb.x, 0.0f))) workflow_.JogLocalAxis(di, axes[3 + jogRotAxis_], +step);
        ImGui::PopStyleColor();
        char stepReadout[24];
        std::snprintf(stepReadout, sizeof(stepReadout), "%s##rotstep", kSteps[jogStepIndex_].label);
        ImGui::Button(stepReadout, ImVec2(jb.x, 0.0f));   // step readout (unique id)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("-##rot", ImVec2(jb.x, 0.0f))) workflow_.JogLocalAxis(di, axes[3 + jogRotAxis_], -step);
        ImGui::PopStyleColor();
        ImGui::EndGroup();
    }
    ImGui::EndDisabled();

    // --- Keyboard jog (armed + this window focused): one step per key press ---
    // A/D = Left/Right, W/S = In/Out, R/F = Up/Down.
    if (jogArmed_ && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
        if (ImGui::IsKeyPressed(ImGuiKey_A, false)) workflow_.JogGlobal(di, -1.0f, 0.0f, 0.0f, step);  // Left  X-
        if (ImGui::IsKeyPressed(ImGuiKey_D, false)) workflow_.JogGlobal(di,  1.0f, 0.0f, 0.0f, step);  // Right X+
        if (ImGui::IsKeyPressed(ImGuiKey_W, false)) workflow_.JogGlobal(di, 0.0f, -1.0f, 0.0f, step);  // In    Y-
        if (ImGui::IsKeyPressed(ImGuiKey_S, false)) workflow_.JogGlobal(di, 0.0f,  1.0f, 0.0f, step);  // Out   Y+
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) workflow_.JogGlobal(di, 0.0f, 0.0f, -1.0f, step);  // Up    Z-
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) workflow_.JogGlobal(di, 0.0f, 0.0f,  1.0f, step);  // Down  Z+
    }
}

// ---- PI / ACS pages (motion fleet) ---------------------------------------
// Read-only telemetry pulled through HardwareManager, mirroring how the UI
// already pulls camera frames — no device is *driven* from here (that stays in
// the workflow layer). `type` filters the fleet to "PI" or "ACS".

void App::DrawMotionListPanel(const char* type)
{
    const size_t n = hardware_.MotionCount();

    // Keep the current selection if it still matches this page's type; otherwise
    // fall back to the first device of this type. The left panel draws before the
    // middle/right, so this also resolves the selection they read.
    auto matches = [&](int i) {
        if (i < 0 || static_cast<size_t>(i) >= n) return false;
        IMotionController* d = hardware_.MotionDevice(static_cast<size_t>(i));
        return d && d->Type() == type;
    };
    if (!matches(selectedMotion_)) {
        selectedMotion_ = -1;
        for (size_t i = 0; i < n; ++i) {
            IMotionController* d = hardware_.MotionDevice(i);
            if (d && d->Type() == type) { selectedMotion_ = static_cast<int>(i); break; }
        }
    }

    ImGui::Text("%s devices", type);
    ImGui::Separator();

    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        IMotionController* m = hardware_.MotionDevice(i);
        if (!m || m->Type() != type) continue;
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
        ImGui::TextColored(StateColor(st), "%s", StateText(st));
        ImGui::PopID();
    }
    if (!any) ImGui::TextDisabled("no %s devices configured", type);
}

void App::DrawMotionDetailPanel(const char* type)
{
    const size_t n = hardware_.MotionCount();
    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;
    if (!m || m->Type() != type) {
        ImGui::TextDisabled("Select a %s device from the list.", type);
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

    ImGui::Spacing();
    ImGui::TextDisabled("Jog / move / stop run through the workflow layer (coming next).");
}

void App::DrawMotionInfoPanel(const char* type)
{
    // Right panel: the live axis-position table for the selected device.
    const size_t n = hardware_.MotionCount();
    IMotionController* m = (selectedMotion_ >= 0 && static_cast<size_t>(selectedMotion_) < n)
                             ? hardware_.MotionDevice(static_cast<size_t>(selectedMotion_))
                             : nullptr;

    ImGui::TextUnformatted("Axis positions");
    ImGui::Separator();
    if (!m || m->Type() != type) { ImGui::TextDisabled("no selection"); return; }

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
}

void App::DrawImageOverlay()
{
    ImGui::TextUnformatted("Vision");
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float w = 480.0f, h = 360.0f;
    ImGui::Image((ImTextureID)(intptr_t)viewportTex_, ImVec2(w, h));   // C-style cast: void*/ImU64 ImTextureID

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 c(origin.x + w * 0.5f, origin.y + h * 0.5f);
    dl->AddLine(ImVec2(c.x - 20, c.y), ImVec2(c.x + 20, c.y), IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddLine(ImVec2(c.x, c.y - 20), ImVec2(c.x, c.y + 20), IM_COL32(255, 80, 80, 255), 2.0f);
    dl->AddRect(ImVec2(origin.x + 120, origin.y + 90), ImVec2(origin.x + 300, origin.y + 240),
                IM_COL32(90, 250, 120, 255), 0.0f, 0, 2.0f);
    dl->AddText(ImVec2(origin.x + 122, origin.y + 72), IM_COL32(90, 250, 120, 255), "match 0.98");
}

void App::DrawObjectCanvas()
{
    ImGui::TextUnformatted("Layout (drag objects)");
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 size = ImGui::GetContentRegionAvail();   // fill the lower panel
    if (size.y < 60.0f) size.y = 60.0f;

    ImGui::InvisibleButton("canvas", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y), IM_COL32(30, 30, 34, 255));

    const float radius = 22.0f;
    const ImVec2 mouse = ImGui::GetIO().MousePos;

    if (ImGui::IsItemActive() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        for (int i = static_cast<int>(objects_.size()) - 1; i >= 0; --i) {
            const ImVec2 p(origin.x + objects_[i].pos.x, origin.y + objects_[i].pos.y);
            const float dx = mouse.x - p.x, dy = mouse.y - p.y;
            if (dx * dx + dy * dy <= radius * radius) { dragging_ = i; break; }
        }
    }
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) dragging_ = -1;
    if (dragging_ >= 0) {
        CanvasObject& o = objects_[dragging_];
        o.pos.x += ImGui::GetIO().MouseDelta.x;
        o.pos.y += ImGui::GetIO().MouseDelta.y;
        o.pos.x = fmaxf(radius, fminf(size.x - radius, o.pos.x));
        o.pos.y = fmaxf(radius, fminf(size.y - radius, o.pos.y));
    }

    for (const auto& o : objects_) {
        const ImVec2 p(origin.x + o.pos.x, origin.y + o.pos.y);
        dl->AddCircleFilled(p, radius, o.color);
        const ImVec2 ts = ImGui::CalcTextSize(o.label);
        dl->AddText(ImVec2(p.x - ts.x * 0.5f, p.y - ts.y * 0.5f), IM_COL32_BLACK, o.label);
    }
}

void App::DrawAuxWindow()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + mv->Size.x + 40.0f, mv->Pos.y + 40.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(700.0f, 760.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Aux - Camera + Signal", &showAux_)) {
        // Pull the newest camera frame into a GL texture.
        bool isNew = false;
        if (hardware_.Camera().GetLatestFrame(frameBuf_, cameraW_, cameraH_, isNew) &&
            cameraW_ > 0 && cameraH_ > 0) {
            if (isNew || cameraTex_ == 0) {
                cameraTex_ = host_.EnsureTextureRGB(cameraTex_, cameraW_, cameraH_, frameBuf_.data());
            }
        }

        // ---- Background: live camera fills the entire content region ----
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const ImVec2 avail  = ImGui::GetContentRegionAvail();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
        if (haveImg) {
            // "cover" fit: fill the box, preserve aspect, crop the overflow via UV.
            ImVec2 uv0(0.0f, 0.0f), uv1(1.0f, 1.0f);
            const float imgA = static_cast<float>(cameraW_) / static_cast<float>(cameraH_);
            const float boxA = avail.x / avail.y;
            if (boxA > imgA) { const float s = imgA / boxA; uv0.y = (1.0f - s) * 0.5f; uv1.y = 1.0f - uv0.y; }
            else             { const float s = boxA / imgA; uv0.x = (1.0f - s) * 0.5f; uv1.x = 1.0f - uv0.x; }
            ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, uv0, uv1);
        } else {
            dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), IM_COL32(18, 18, 22, 255));
            const char* msg = "waiting for camera feed...";
            const ImVec2 ts = ImGui::CalcTextSize(msg);
            dl->AddText(ImVec2(origin.x + (avail.x - ts.x) * 0.5f, origin.y + (avail.y - ts.y) * 0.5f),
                        IM_COL32(150, 150, 150, 255), msg);
            ImGui::Dummy(avail);   // reserve the region so layout below is correct
        }

        // ---- Interaction layer: crosshair + world-distance + click-to-move ----
        // An invisible button over the image captures hover/click. It's emitted
        // BEFORE the chart overlay child, so the chart (drawn later, on top) grabs
        // clicks in its corner and the stage won't move when you use the chart.
        ImGui::SetCursorScreenPos(origin);
        const bool imgClicked = ImGui::InvisibleButton("##imgInput", avail);
        const bool imgHovered = ImGui::IsItemHovered();
        const ImVec2 center(origin.x + avail.x * 0.5f, origin.y + avail.y * 0.5f);

        // Fixed center crosshair (the optical/stage reference).
        const ImU32 chair = IM_COL32(80, 230, 120, 200);
        dl->AddLine(ImVec2(origin.x, center.y), ImVec2(origin.x + avail.x, center.y), chair, 1.0f);
        dl->AddLine(ImVec2(center.x, origin.y), ImVec2(center.x, origin.y + avail.y), chair, 1.0f);
        dl->AddCircle(center, 6.0f, chair, 0, 1.5f);

        if (haveImg && imgHovered) {
            const ImVec2 m = ImGui::GetIO().MousePos;
            // Screen -> camera pixels. The image is "cover"-fit with a single
            // uniform scale, so 1 camera px = `scale` screen px on both axes.
            const float scale = fmaxf(avail.x / static_cast<float>(cameraW_),
                                      avail.y / static_cast<float>(cameraH_));
            const float dxPix = (m.x - center.x) / scale;         // camera px from center
            const float dyPix = (m.y - center.y) / scale;
            const float dxMM  = dxPix * pixelToMmX_;              // real-world offset
            const float dyMM  = dyPix * pixelToMmY_;

            // Line from center to cursor + a marker dot (red when click-to-move
            // is armed, so it's clear a click will command the gantry).
            const ImU32 markCol = enableClickMove_ ? IM_COL32(255, 90, 90, 255)
                                                   : IM_COL32(255, 220, 90, 255);
            dl->AddLine(center, m, IM_COL32(255, 220, 90, 160), 1.0f);
            dl->AddCircleFilled(m, 3.5f, markCol);

            // World-distance label near the cursor (kept inside the image).
            char lbl[96];
            std::snprintf(lbl, sizeof(lbl), "dX %+.3f mm   dY %+.3f mm", dxMM, dyMM);
            const ImVec2 lts = ImGui::CalcTextSize(lbl);
            ImVec2 lp(m.x + 14.0f, m.y + 8.0f);
            lp.x = fminf(lp.x, origin.x + avail.x - lts.x - 8.0f);
            lp.y = fminf(lp.y, origin.y + avail.y - lts.y - 6.0f);
            dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                              ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                              IM_COL32(0, 0, 0, 165), 3.0f);
            dl->AddText(lp, IM_COL32(255, 235, 150, 255), lbl);

            // Left-click -> move gantry-main (ACS) by this XY offset, via layer 2.
            // Only when armed via the "Click-to-move" checkbox (it moves real HW).
            if (enableClickMove_ && imgClicked) workflow_.MoveGantryRelativeMM(dxMM, dyMM);
        }

        // ---- Overlay: camera status, top-left (dark plate for readability) ----
        char camline[128];
        std::snprintf(camline, sizeof(camline), "%s  [%s]  %dx%d",
                      hardware_.CameraName().c_str(), StateText(hardware_.CameraState()),
                      cameraW_, cameraH_);
        const ImVec2 cts = ImGui::CalcTextSize(camline);
        dl->AddRectFilled(ImVec2(origin.x + 6.0f, origin.y + 6.0f),
                          ImVec2(origin.x + 14.0f + cts.x, origin.y + 12.0f + cts.y),
                          IM_COL32(0, 0, 0, 140), 4.0f);
        dl->AddText(ImVec2(origin.x + 10.0f, origin.y + 9.0f), IM_COL32(230, 230, 235, 255), camline);

        // ---- Overlay: view toggles, just under the status plate ----
        // A child window blocks the image input button beneath it, so ticking a
        // box never triggers a click-to-move.
        ImGui::SetCursorScreenPos(ImVec2(origin.x + 6.0f, origin.y + 12.0f + cts.y + 8.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.06f, 0.08f, 0.78f));
        ImGui::BeginChild("AuxControls", ImVec2(0.0f, 0.0f),
                          ImGuiChildFlags_AutoResizeX | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar);
        if (ImGui::Checkbox("Chart", &showChart_)) SaveUiSettings();
        ImGui::SameLine();
        if (ImGui::Checkbox("Click-to-move", &enableClickMove_)) SaveUiSettings();
        ImGui::EndChild();
        ImGui::PopStyleColor();

        // ---- Overlay: signal chart, pinned to the top-right corner ----
        if (showChart_) {
            const float margin = 10.0f;
            const float panelW = avail.x * 0.45f;
            const float panelH = avail.y * 0.40f;
            ImGui::SetCursorScreenPos(ImVec2(origin.x + avail.x - panelW - margin, origin.y + margin));
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.06f, 0.08f, 0.85f));
            ImGui::BeginChild("SignalOverlay", ImVec2(panelW, panelH), ImGuiChildFlags_Borders,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            DrawSignalOverlay();
            ImGui::EndChild();
            ImGui::PopStyleColor();
        }
    }
    ImGui::End();
}

void App::DrawSignalOverlay()
{
    // ---- Keithley current stream (subscribed from the DataBus) ----
    DataBus::Instance().Stream(topics::kKeithleyCh1).Snapshot(kT1_, kV1_);
    DataBus::Instance().Stream(topics::kKeithleyCh2).Snapshot(kT2_, kV2_);

    // Live readout: latest value per channel + achieved sample rate.
    if (!kV1_.empty()) ImGui::TextColored(ImVec4(0.00f, 0.85f, 1.00f, 1.0f), "Ch1: %s", FormatCurrent(kV1_.back()).c_str());
    else               ImGui::TextDisabled("Ch1: --");
    ImGui::SameLine(0.0f, 16.0f);
    if (!kV2_.empty()) ImGui::TextColored(ImVec4(1.00f, 0.55f, 0.10f, 1.0f), "Ch2: %s", FormatCurrent(kV2_.back()).c_str());
    else               ImGui::TextDisabled("Ch2: --");
    ImGui::SameLine(0.0f, 16.0f);
    ImGui::Text("| %.0f Hz", MeasuredHz(kT1_.empty() ? kT2_ : kT1_));

    ImGui::RadioButton("Ch1", &chartCh_, 0); ImGui::SameLine();
    ImGui::RadioButton("Ch2", &chartCh_, 1); ImGui::SameLine();
    ImGui::RadioButton("Both", &chartCh_, 2);

    if (ImPlot::BeginPlot("##keithley", ImVec2(-1, -1))) {
        // Y auto-fits to the visible data; X is a fixed relative-time window:
        // 0 s at the right edge (now), running back to -kChartWindowSec on the
        // left. chartClock_ is the rolling "now" that data is rebased against;
        // it advances by real frame time and re-syncs to the newest sample.
        ImPlot::SetupAxes("t (s)", "current (A)",
                          ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
        const std::vector<float>& tref = kT1_.empty() ? kT2_ : kT1_;
        if (!tref.empty()) {
            const double last = tref.back();
            chartClock_ += ImGui::GetIO().DeltaTime;
            if (last > chartClock_ || chartClock_ - last > 0.5) chartClock_ = last;
        }
        ImPlot::SetupAxisLimits(ImAxis_X1, -kChartWindowSec, 0.0, ImGuiCond_Always);
        ImPlot::SetupAxisFormat(ImAxis_X1, "%.0fs");

        if (chartCh_ != 1 && !kV1_.empty()) {
            ShiftToRelative(kT1_, chartClock_, kX1_);
            ImPlotSpec s1; s1.LineColor = ImVec4(0.00f, 0.85f, 1.00f, 1.0f); s1.LineWeight = 2.0f;   // cyan
            ImPlot::PlotLine("ch1", kX1_.data(), kV1_.data(), static_cast<int>(kV1_.size()), s1);
        }
        if (chartCh_ != 0 && !kV2_.empty()) {
            ShiftToRelative(kT2_, chartClock_, kX2_);
            ImPlotSpec s2; s2.LineColor = ImVec4(1.00f, 0.55f, 0.10f, 1.0f); s2.LineWeight = 2.0f;   // orange
            ImPlot::PlotLine("ch2", kX2_.data(), kV2_.data(), static_cast<int>(kV2_.size()), s2);
        }
        ImPlot::EndPlot();
    }
}

void App::DrawLogPanel()
{
    Logger& log = Logger::Instance();

    // ---- Controls (single row so the log gets the rest of the panel) ----
    static const char* kLevels[] = {"Trace", "Debug", "Info", "Warn", "Error"};
    ImGui::TextUnformatted("Log");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    if (ImGui::Combo("##loglevel", &logMinLevel_, kLevels, 5)) {
        log.SetMinLevel(static_cast<LogLevel>(logMinLevel_));   // gates capture going forward
    }
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &logAutoScroll_);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) { log.Clear(); logSeen_ = 0; logView_.clear(); }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu shown", logView_.size());

    // Refresh the cached snapshot only when the Logger reports new activity.
    const size_t total = log.Count();
    if (total != logSeen_) { logView_ = log.Snapshot(); logSeen_ = total; }

    const LogLevel minShown = static_cast<LogLevel>(logMinLevel_);
    ImGui::BeginChild("logscroll", ImVec2(0, 0), ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);
    for (const LogEntry& e : logView_) {
        if (e.level < minShown) continue;   // also filter already-buffered rows
        ImGui::TextColored(LevelColor(e.level), "%9.3f  %-5s  %s",
                           e.t, LevelName(e.level), e.text.c_str());
    }
    if (logAutoScroll_) ImGui::SetScrollY(ImGui::GetScrollMaxY());
    ImGui::EndChild();
}

} // namespace firedog
