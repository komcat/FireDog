// App.cpp : layer 3 (UI) implementation.
#include "App.h"

#include "imgui.h"
#include "implot.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "command/Commands.h"
#include "core/DataBus.h"
#include "core/DutSession.h"
#include "core/Topics.h"
#include "core/Log.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
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

// Keithley 6482 fixed current ranges (full-scale amps, decades).
struct KeiRange { const char* label; double amps; };
constexpr KeiRange kKeiRanges[] = {
    {"2 nA", 2e-9}, {"20 nA", 2e-8}, {"200 nA", 2e-7}, {"2 uA", 2e-6},
    {"20 uA", 2e-5}, {"200 uA", 2e-4}, {"2 mA", 2e-3}, {"20 mA", 2e-2},
};
constexpr int kKeiRangeCount = static_cast<int>(sizeof(kKeiRanges) / sizeof(kKeiRanges[0]));

// Closest table row for a configured fixed range (default: 2 mA).
int KeiRangeIndexFor(double amps) {
    int best = 6;
    if (amps <= 0.0) return best;
    double bestErr = 1e300;
    for (int i = 0; i < kKeiRangeCount; ++i) {
        const double err = std::fabs(std::log10(amps / kKeiRanges[i].amps));
        if (err < bestErr) { bestErr = err; best = i; }
    }
    return best;
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
    // Restore the saved theme + font choice + size (defaults to FireDog Dark /
    // Segoe UI 26 px on first run), validated against the current tables.
    std::string lastProduct;
    {
        const UiSettings ui = UiSettings::Load(kUiSettingsPath);
        fontIndex_  = (ui.fontIndex >= 0 && ui.fontIndex < kFontCount) ? ui.fontIndex : 0;
        uiFontSize_ = (ui.fontSize >= 8.0f && ui.fontSize <= 48.0f) ? ui.fontSize : 26.0f;
        themeIndex_ = (ui.themeIndex >= 0 && ui.themeIndex < Theme::Count()) ? ui.themeIndex : 0;
        // Panel sizes: keep saved px if present; 0 lets DrawMainWindow seed them.
        if (ui.leftW  > 0.0f) leftW_  = ui.leftW;
        if (ui.rightW > 0.0f) rightW_ = ui.rightW;
        if (ui.lowerH > 0.0f) lowerH_ = ui.lowerH;
        showChart_       = ui.showChart;
        enableClickMove_ = ui.clickToMove;
        chartScope_      = ui.chartScope;
        lastProduct      = ui.lastProduct;
    }
    Theme::Apply(themeIndex_);
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
    watchCfg_ = hw.ioWatch;   // working copy for the IO Watch editor
    if (const CameraConfig* cam = hw.FirstEnabledCamera()) {
        cameraId_   = cam->id;   // for saving settings back into hardware.json
        pixelToMmX_ = static_cast<float>(cam->pixelToMmX);
        pixelToMmY_ = static_cast<float>(cam->pixelToMmY);
    }
    if (const KeithleyConfig* kc = hw.FirstEnabledKeithley()) {
        keithleyId_   = kc->name;   // for saving settings back into hardware.json
        keiBias_      = static_cast<float>(kc->biasVoltage);
        keiNplc_      = static_cast<float>(kc->nplc);
        keiAutoRange_ = kc->autoRange;
        keiRangeIdx_  = KeiRangeIndexFor(kc->fixedRange);
        keiPollMs_    = kc->pollIntervalMs;
    }
    if (const Cld101xConfig* cc = hw.FirstEnabledCld()) {
        cldId_         = cc->name;
        cldTempSet_    = static_cast<float>(cc->defaultTemperature);
        cldCurrentSet_ = static_cast<float>(cc->defaultLaserCurrentMA);
    }
    if (const PsuConfig* pc = hw.FirstEnabledPsu()) {
        psuId_ = pc->name;
    }
    transformCfg_ = LoadMotionTransforms("config/motion_transforms.json");
    workflow_.SetTransforms(transformCfg_);   // transformCfg_ stays the editor's working copy
    savedPos_ = LoadSavedPositions();   // config fallback (no product)
    motionMaps_ = LoadMotionMaps();     // node-edge maps (Mapped Path tab)
    if (!lastProduct.empty() && !LoadProduct(lastProduct))
        FD_WARN("product", "could not reload last product '%s'", lastProduct.c_str());
    if (!product_.Loaded()) LoadRecipesFromDisk();   // else LoadProduct did it
    LoadScriptsList();             // .lua programs (product dir, else config/scripts)
    if (product_.Loaded())         // DUT records live under the product's data/
        DutSession::Instance().SetProduct(product_.name, product_.uid,
                                          product_.DataDir());
    hardware_.ConnectAllAsync();   // simulated instruments (motion/etc.)
    workflow_.Start(hw);           // layer 2 brings camera + Keithley online
    RegisterBuiltinCommands(cmdRegistry_, workflow_, hardware_);   // layer 2.5
    FD_INFO("command", "%zu command(s) registered", cmdRegistry_.Count());

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
    workflow_.Update(dt);        // layer 2 (kept for future orchestration)
    UpdateKeithleyChartData(dt); // shared chart buffers (Aux + Keithley page)
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
        // ---- Menu bar (top strip) ----
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("New Product..."))  openNewProduct_  = true;
                if (ImGui::MenuItem("Load Product...")) openLoadProduct_ = true;
                if (ImGui::MenuItem("Save Product", nullptr, false, product_.Loaded()))
                    SaveProduct();
                ImGui::Separator();
                if (product_.Loaded())
                    ImGui::MenuItem(("Product: " + product_.name).c_str(), nullptr, false, false);
                else
                    ImGui::MenuItem("Product: (none)", nullptr, false, false);
                ImGui::Separator();
                ImGui::MenuItem("Exit", nullptr, false, false);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Hardware")) {
                if (ImGui::MenuItem("Overview", nullptr, currentPage_ == Page::Overview))
                    currentPage_ = Page::Overview;
                if (ImGui::MenuItem("Camera",   nullptr, currentPage_ == Page::Camera))
                    currentPage_ = Page::Camera;
                if (ImGui::MenuItem("Keithley", nullptr, currentPage_ == Page::Keithley))
                    currentPage_ = Page::Keithley;
                if (ImGui::MenuItem("CLD101x",  nullptr, currentPage_ == Page::CLD))
                    currentPage_ = Page::CLD;
                if (ImGui::MenuItem("PSU",      nullptr, currentPage_ == Page::PSU))
                    currentPage_ = Page::PSU;
                if (ImGui::MenuItem("Motion",   nullptr, currentPage_ == Page::Motion))
                    currentPage_ = Page::Motion;
                if (ImGui::MenuItem("IO",       nullptr, currentPage_ == Page::IO))
                    currentPage_ = Page::IO;
                if (ImGui::MenuItem("Pneumatics", nullptr, currentPage_ == Page::Pneumatic))
                    currentPage_ = Page::Pneumatic;
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Aux Window", nullptr, &showAux_);
                ImGui::MenuItem("Jog",        nullptr, &showJog_);
                if (ImGui::BeginMenu("Theme")) {   // same picker as Display header
                    for (int i = 0; i < Theme::Count(); ++i) {
                        if (ImGui::MenuItem(Theme::Name(i), nullptr, themeIndex_ == i)) {
                            themeIndex_ = i;
                            Theme::Apply(themeIndex_);
                            SaveUiSettings();
                        }
                    }
                    ImGui::EndMenu();
                }
                ImGui::Separator();
                // Editor pages replace the main-window content; selecting one
                // remembers the current hardware page so Close returns to it.
                auto toggleEditor = [this](Page p) {
                    if (currentPage_ == p) { currentPage_ = prevPage_; return; }
                    if (currentPage_ != Page::WatchEdit && currentPage_ != Page::TransformEdit &&
                        currentPage_ != Page::RecipeEdit && currentPage_ != Page::Scripts &&
                        currentPage_ != Page::DutView)
                        prevPage_ = currentPage_;
                    currentPage_ = p;
                };
                if (ImGui::MenuItem("IO Watch Editor", nullptr, currentPage_ == Page::WatchEdit))
                    toggleEditor(Page::WatchEdit);
                if (ImGui::MenuItem("Transforms Editor", nullptr, currentPage_ == Page::TransformEdit))
                    toggleEditor(Page::TransformEdit);
                if (ImGui::MenuItem("Recipe Editor", nullptr, currentPage_ == Page::RecipeEdit))
                    toggleEditor(Page::RecipeEdit);
                if (ImGui::MenuItem("Scripts", nullptr, currentPage_ == Page::Scripts))
                    toggleEditor(Page::Scripts);
                if (ImGui::MenuItem("DUT Records", nullptr, currentPage_ == Page::DutView)) {
                    if (currentPage_ != Page::DutView) LoadDutList();
                    toggleEditor(Page::DutView);
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Help")) {
                ImGui::MenuItem("About", nullptr, false, false);
                ImGui::EndMenu();
            }
            if (workflow_.SafetyAlertActive()) {
                ImGui::TextColored(StateColor(ConnState::Failed), "  !! SAFETY ALERT !!");
            }
            ImGui::EndMenuBar();
        }
        DrawProductPopups();   // modals opened via the File-menu flags above
        DrawSafetyAlertModal();   // blocking safety alert (highest priority)
        DrawRecipePromptModal();  // global recipe prompt (any page, any time)

        // ---- Editor pages (View menu) replace the whole panel layout until
        // closed via the top-right Close button (back to the previous page).
        if (currentPage_ == Page::WatchEdit || currentPage_ == Page::TransformEdit ||
            currentPage_ == Page::RecipeEdit || currentPage_ == Page::Scripts ||
            currentPage_ == Page::DutView) {
            const char* title = currentPage_ == Page::WatchEdit       ? "IO Watch Editor"
                                : currentPage_ == Page::TransformEdit ? "Transforms Editor"
                                : currentPage_ == Page::RecipeEdit    ? "Recipe Editor"
                                : currentPage_ == Page::Scripts       ? "Scripts"
                                                                      : "DUT Records";
            ImGui::BeginChild("EditorPage", ImGui::GetContentRegionAvail(),
                              ImGuiChildFlags_Borders);
            ImGui::TextUnformatted(title);
            ImGui::SameLine();
            const float closeW = ImGui::CalcTextSize("Close").x +
                                 ImGui::GetStyle().FramePadding.x * 2.0f;
            ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - closeW);
            if (ImGui::Button("Close")) {
                // The recipe editor guards unsaved edits with a confirm modal.
                if (currentPage_ == Page::RecipeEdit && RecipesDirty())
                    edConfirmAction_ = 1;
                else
                    currentPage_ = prevPage_;
            }
            ImGui::Separator();
            if (currentPage_ == Page::WatchEdit)          DrawWatchEditPanel();
            else if (currentPage_ == Page::TransformEdit) DrawTransformEditPanel();
            else if (currentPage_ == Page::RecipeEdit)    DrawRecipeEditPanel();
            else if (currentPage_ == Page::Scripts)       DrawScriptsPanel();
            else                                          DrawDutViewPanel();
            ImGui::EndChild();
            ImGui::End();
            return;
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
            case Page::Camera:    DrawCameraListPanel();    break;
            case Page::Keithley:  DrawKeithleyListPanel();  break;
            case Page::CLD:       DrawCldListPanel();       break;
            case Page::PSU:       DrawPsuListPanel();       break;
            case Page::Motion:    DrawMotionListPanel();    break;
            case Page::IO:        DrawIoListPanel();        break;
            case Page::Pneumatic: DrawPneumaticListPanel(); break;
            default:              DrawLeftPanel();          break;
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
            case Page::Camera:    DrawCameraDetailPanel();    break;
            case Page::Keithley:  DrawKeithleyDetailPanel();  break;
            case Page::CLD:       DrawCldDetailPanel();       break;
            case Page::PSU:       DrawPsuDetailPanel();       break;
            case Page::Motion:    DrawMotionDetailPanel();    break;
            case Page::IO:        DrawIoDetailPanel();        break;
            case Page::Pneumatic: DrawPneumaticDetailPanel(); break;
            default:              DrawMiddlePanel();          break;
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
            case Page::Camera:    DrawCameraInfoPanel();    break;
            case Page::Keithley:  DrawKeithleyInfoPanel();  break;
            case Page::CLD:       DrawCldInfoPanel();       break;
            case Page::PSU:       DrawPsuInfoPanel();       break;
            case Page::Motion:    DrawMotionInfoPanel();    break;
            case Page::IO:        DrawIoInputPanel();       break;
            case Page::Pneumatic: DrawPneumaticInfoPanel(); break;
            default:              DrawRightPanel();         break;
        }
        ImGui::EndChild();

        // Persist panel sizes once a splitter drag finishes (shared across pages).
        if (splitterWasActive_ && !splitterActive) SaveUiSettings();
        splitterWasActive_ = splitterActive;
    }
    ImGui::End();
}

// ---- DUT Records viewer (View menu, full page, read-only) — Phase D2 -------
// Browses products/<name>/data/: serial list on the left, the record's entry
// log in the middle, header + run history on the right. Image entries show
// their linked path (in-app image preview is future work).

void App::LoadDutList()
{
    namespace fs = std::filesystem;
    dutList_.clear();
    dutSel_ = -1;
    dutRecSerial_.clear();
    dutRec_ = nlohmann::json();
    if (!product_.Loaded()) return;
    std::error_code ec;
    for (const fs::directory_entry& e :
         fs::directory_iterator(product_.DataDir(), ec)) {
        if (e.is_directory(ec) && fs::exists(e.path() / "dut.json", ec))
            dutList_.push_back(e.path().filename().string());
    }
    std::sort(dutList_.begin(), dutList_.end());
    if (!dutList_.empty()) dutSel_ = 0;
}

void App::DrawDutViewPanel()
{
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  leftW = 200.0f, rightW = 320.0f;

    // ---- Left: serial list ----
    ImGui::BeginChild("dvList", ImVec2(leftW, 0.0f), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Serials");
    ImGui::SameLine();
    if (ImGui::SmallButton("Reload")) LoadDutList();
    ImGui::Separator();
    if (!product_.Loaded())
        ImGui::TextWrapped("Load a product first.");
    else if (dutList_.empty())
        ImGui::TextWrapped("No DUT records under %s.", product_.DataDir().c_str());
    for (int i = 0; i < static_cast<int>(dutList_.size()); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(dutList_[i].c_str(), dutSel_ == i)) dutSel_ = i;
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // (Re)load the selected record — tolerant, view-only.
    if (dutSel_ >= 0 && dutSel_ < static_cast<int>(dutList_.size()) &&
        dutRecSerial_ != dutList_[dutSel_]) {
        dutRecSerial_ = dutList_[dutSel_];
        dutRec_       = nlohmann::json();
        try {
            std::ifstream f(product_.DataDir() + "/" + dutRecSerial_ + "/dut.json");
            if (f.is_open()) f >> dutRec_;
        } catch (const std::exception& e) {
            FD_WARN("dut", "record '%s' unreadable: %s", dutRecSerial_.c_str(), e.what());
        }
    }
    const nlohmann::json entries =
        dutRec_.contains("entries") && dutRec_["entries"].is_array()
            ? dutRec_["entries"]
            : nlohmann::json::array();

    // ---- Middle: entry table ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    ImGui::BeginChild("dvEntries", ImVec2(midW, 0.0f), ImGuiChildFlags_Borders);
    if (dutRec_.is_object()) {
        if (ImGui::BeginTable("##dventries", 5,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 40.0f);
            ImGui::TableSetupColumn("time (UTC)", ImGuiTableColumnFlags_WidthFixed, 170.0f);
            ImGui::TableSetupColumn("key", ImGuiTableColumnFlags_WidthFixed, 120.0f);
            ImGui::TableSetupColumn("type", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableSetupColumn("value");
            ImGui::TableHeadersRow();
            for (const nlohmann::json& e : entries) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", static_cast<std::size_t>(e.value("seq", 0u)));
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(e.value("t", std::string()).c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(e.value("key", std::string()).c_str());
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(e.value("type", std::string()).c_str());
                ImGui::TableSetColumnIndex(4);
                const nlohmann::json v = e.contains("value") ? e["value"] : nlohmann::json();
                std::string shown = v.is_string() ? v.get<std::string>() : v.dump();
                if (shown.size() > 120) shown = shown.substr(0, 117) + "...";
                ImGui::TextUnformatted(shown.c_str());
                if (shown.size() >= 120 && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", v.dump(2).c_str());
            }
            ImGui::EndTable();
        }
    } else {
        ImGui::TextDisabled("Select a serial on the left.");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: record summary + run history ----
    ImGui::BeginChild("dvInfo", ImVec2(rightW, 0.0f), ImGuiChildFlags_Borders);
    if (dutRec_.is_object()) {
        ImGui::Text("Serial: %s", dutRec_.value("serial", std::string()).c_str());
        ImGui::Text("Product: %s", dutRec_.value("product", std::string()).c_str());
        ImGui::Text("Opened: %s", dutRec_.value("opened_at", std::string()).c_str());
        const std::string closed = dutRec_.value("closed_at", std::string());
        ImGui::Text("Closed: %s", closed.empty() ? "(open or interrupted)" : closed.c_str());
        const std::string status = dutRec_.value("status", std::string());
        if (status == "pass")
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.5f, 1.0f), "Status: PASS");
        else if (status == "fail")
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "Status: FAIL");
        else
            ImGui::TextDisabled("Status: (none)");
        ImGui::Text("Entries: %zu", entries.size());
        ImGui::Separator();
        ImGui::TextUnformatted("Runs");
        for (const nlohmann::json& e : entries) {
            if (e.value("key", std::string()) != "$run") continue;
            const nlohmann::json v = e.contains("value") ? e["value"] : nlohmann::json();
            if (!v.is_object()) continue;
            ImGui::BulletText("%s '%s' - %s", v.value("kind", std::string()).c_str(),
                              v.value("name", std::string()).c_str(),
                              v.value("result", std::string()).c_str());
        }
    }
    ImGui::EndChild();
}

// Active-DUT status + open/close controls (Overview left panel). The operator
// path: type/scan a serial, Open (load-or-create). Recipes/scripts drive the
// same DutSession through the dut_* commands. Design: docs/dut-design.md.
void App::DrawDutBlock()
{
    const DutSession::Info di = DutSession::Instance().GetInfo();
    ImGui::TextUnformatted("DUT");
    ImGui::Separator();
    if (!di.productSet) {
        ImGui::TextDisabled("(load a product first)");
        return;
    }
    if (di.open) {
        ImGui::Text("%s", di.serial.c_str());
        ImGui::TextDisabled("%zu entrie(s)%s%s", di.entries,
                            di.status.empty() ? "" : " - ", di.status.c_str());
        std::string err;
        if (ImGui::SmallButton("Pass")) DutSession::Instance().Close("pass", &err);
        ImGui::SameLine();
        if (ImGui::SmallButton("Fail")) DutSession::Instance().Close("fail", &err);
        ImGui::SameLine();
        if (ImGui::SmallButton("Close")) DutSession::Instance().Close("", &err);
        if (!err.empty()) FD_WARN("dut", "close failed: %s", err.c_str());
    } else {
        ImGui::SetNextItemWidth(-64.0f);
        // Barcode scanners type + Enter, so Enter submits like the button.
        const bool enter = ImGui::InputTextWithHint(
            "##dutserial", "serial...", dutSerialBuf_, sizeof(dutSerialBuf_),
            ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        if ((ImGui::SmallButton("Open") || enter) && dutSerialBuf_[0] != '\0') {
            std::string err;
            if (!DutSession::Instance().Open(dutSerialBuf_, false, &err))
                FD_WARN("dut", "open failed: %s", err.c_str());
            else
                dutSerialBuf_[0] = '\0';
        }
    }
}

void App::DrawLeftPanel()
{
    ImGui::TextUnformatted("Hardware");
    ImGui::Separator();
    for (const auto& d : hardware_.Snapshot()) {   // includes camera + Keithley
        DeviceStatusLine(d.state, d.name.c_str());
    }

    ImGui::Spacing();
    DrawDutBlock();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Display")) {
        // Theme picker — hot-applies (mid-frame style changes are fine in
        // immediate mode) and persists like the font choice.
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::BeginCombo("##theme", Theme::Name(themeIndex_))) {
            for (int i = 0; i < Theme::Count(); ++i) {
                if (ImGui::Selectable(Theme::Name(i), themeIndex_ == i)) {
                    themeIndex_ = i;
                    Theme::Apply(themeIndex_);
                    SaveUiSettings();
                }
            }
            ImGui::EndCombo();
        }

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
    s.fontIndex  = fontIndex_;
    s.fontSize   = uiFontSize_;
    s.themeIndex = themeIndex_;
    s.leftW     = leftW_;
    s.rightW    = rightW_;
    s.lowerH    = lowerH_;
    s.showChart   = showChart_;
    s.clickToMove = enableClickMove_;
    s.chartScope  = chartScope_;
    s.lastProduct = product_.name;
    s.Save(kUiSettingsPath);
}

void App::SavePositions()
{
    // Product loaded -> autosave into the product's positions file (with its
    // uid); otherwise the machine-global config fallback. Warns if unwritable.
    if (const ManifestFile* e = product_.Entry("positions"))
        SaveSavedPositionsFile(product_.FilePath("positions"), savedPos_, e->uid);
    else
        SaveSavedPositions(savedPos_);
}

void App::SaveMaps()
{
    // Machine-global for now (config/maps.local.json) — maps reference
    // position uids, which stay stable across products.
    if (SaveMotionMaps(motionMaps_))
        FD_INFO("map", "maps saved to config/maps.local.json");
    else
        FD_WARN("map", "could not save motion maps");
}

bool App::LoadProduct(const std::string& name)
{
    ProductManifest m = LoadProductManifest(name);
    if (!m.Loaded()) return false;

    const ManifestFile* e = m.Entry("positions");
    if (e) {
        std::string fileUid;
        savedPos_ = LoadSavedPositionsFile(m.FilePath("positions"), &fileUid);
        if (fileUid != e->uid)
            FD_WARN("product", "positions uid mismatch: manifest %s vs file '%s'",
                    e->uid.c_str(), fileUid.c_str());
    } else {
        FD_WARN("product", "'%s' has no positions entry; keeping current positions", name.c_str());
    }
    product_ = std::move(m);
    LoadRecipesFromDisk();   // product recipes replace the config-fallback set
    LoadScriptsList();       // product scripts likewise (config fallback)
    DutSession::Instance().SetProduct(product_.name, product_.uid,
                                      product_.DataDir());   // auto-closes any DUT
    SaveUiSettings();   // remember as last product for the next startup
    FD_INFO("product", "loaded '%s'", product_.name.c_str());
    return true;
}

void App::LoadRecipesFromDisk()
{
    recipeSel_ = -1;
    recipeStartErr_.clear();
    if (product_.Loaded()) {
        const ManifestFile* e = product_.Entry("recipes");
        if (e) {
            recipes_ = LoadRecipesFile(product_.FilePath("recipes"), &recipesUid_);
            if (!recipes_.empty()) {
                if (!recipesUid_.empty() && recipesUid_ != e->uid)
                    FD_WARN("product", "recipes uid mismatch: manifest %s vs file '%s'",
                            e->uid.c_str(), recipesUid_.c_str());
                recipesBaseline_ = RecipesDump();
                return;
            }
            // New products carry an empty seed file (no editor yet in Phase A) —
            // fall through to the config recipes so the runner has something.
            FD_INFO("recipe", "product '%s' has no recipes yet - using config recipes",
                    product_.name.c_str());
        } else {
            FD_WARN("product", "'%s' has no recipes entry; using config recipes",
                    product_.name.c_str());
        }
    }
    recipesUid_.clear();
    recipes_ = LoadRecipes();   // config/recipes.local.json -> recipes.json
    recipesBaseline_ = RecipesDump();
}

// Canonical serialization of the in-memory recipes, compared against the
// baseline captured at load/save to answer "are there unsaved edits?".
std::string App::RecipesDump() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const Recipe& r : recipes_) arr.push_back(RecipeToJson(r));
    return arr.dump();
}

// ---- Recipe Editor (View menu, full page) — Phase B1: browse + run ----------
// The step tree and inspector are read-only; editing arrives in B2,
// persistence/validation in B3. See docs/recipe-editor-design.md.

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
    const RecipeEngine::Status st = recipeEngine_.GetStatus();
    const bool running = recipeEngine_.Running();
    const bool haveSel = recipeSel_ >= 0 && recipeSel_ < static_cast<int>(recipes_.size());

    // Live author-time validation of the selected recipe (WITHOUT the
    // devices-Connected requirement — that stays a run-start check, so
    // authoring offline doesn't drown in noise).
    std::vector<RecipeIssue> issues;
    if (haveSel)
        issues = ValidateRecipe(recipes_[recipeSel_], cmdRegistry_, hardware_,
                                savedPos_, motionMaps_, recipes_, false);
    const std::size_t shownIssues = issues.size() < 3 ? issues.size() : 3;
    const float issuesH =
        issues.empty() ? 0.0f
                       : ImGui::GetTextLineHeightWithSpacing() *
                                 static_cast<float>(shownIssues + 1) + 10.0f;

    const float  stripH = ImGui::GetFrameHeightWithSpacing() * 3.2f + issuesH;
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  paneH  = avail.y - stripH;
    const float  leftW  = 230.0f;
    const float  rightW = 320.0f;

    // ---- Left: recipe list + CRUD ----
    ImGui::BeginChild("edRecipes", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    ImGui::TextUnformatted("Recipes");
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::SmallButton("Reload")) {
        if (RecipesDirty()) {
            edConfirmAction_ = 2;   // guard: reload discards unsaved edits
        } else {
            LoadRecipesFromDisk();
            FD_INFO("recipe", "%zu recipe(s) loaded", recipes_.size());
        }
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(running);
    if (ImGui::SmallButton("New")) {
        Recipe r;
        r.uid  = GenerateUid();
        r.name = "new recipe";
        recipes_.push_back(std::move(r));
        recipeSel_ = static_cast<int>(recipes_.size()) - 1;
        edStepSel_.clear();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveSel);
    if (ImGui::SmallButton("Dup")) {
        Recipe r = recipes_[recipeSel_];
        r.uid  = GenerateUid();
        r.name += " copy";
        recipes_.push_back(std::move(r));
        recipeSel_ = static_cast<int>(recipes_.size()) - 1;
        edStepSel_.clear();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Ren")) {
        std::snprintf(edNameBuf_, sizeof(edNameBuf_), "%s", recipes_[recipeSel_].name.c_str());
        edOpenRenameRecipe_ = true;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Del")) edOpenDeleteRecipe_ = true;
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::Separator();
    if (recipes_.empty())
        ImGui::TextWrapped("No recipes yet - New creates one, or Reload from disk.");
    for (int i = 0; i < static_cast<int>(recipes_.size()); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(recipes_[i].name.c_str(), recipeSel_ == i)) {
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
        Recipe& r = recipes_[recipeSel_];
        ImGui::BeginChild("edPalette", ImVec2(palW, 0.0f), ImGuiChildFlags_Borders);
        DrawRecipePalette(r, !running);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("edTree", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
        DrawRecipeTreeToolbar(r, !running);
        ImGui::Separator();
        std::vector<std::size_t> path;
        DrawRecipeTreeNodes(r.steps, path, false, st.stepPath, running);
        ImGui::EndChild();
    } else {
        ImGui::TextDisabled("Select a recipe on the left.");
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: inspector (step form, or recipe fields when no step) ----
    ImGui::BeginChild("edInspect", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    RecipeStep* sel = nullptr;
    if (haveSel && !edStepSel_.empty())
        sel = FindStepByPathMut(&recipes_[recipeSel_].steps, edStepSel_);
    if (sel) {
        DrawRecipeInspector(*sel, !running);
    } else if (haveSel) {
        Recipe& r = recipes_[recipeSel_];
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

    DrawRecipeRunStrip(issues);

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
            SaveRecipes();
            if (edConfirmAction_ == 1) currentPage_ = prevPage_;
            else                       LoadRecipesFromDisk();
            edConfirmAction_ = 0;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard", ImVec2(90.0f, 0.0f))) {
            LoadRecipesFromDisk();   // throw away the in-memory edits
            if (edConfirmAction_ == 1) currentPage_ = prevPage_;
            edConfirmAction_ = 0;
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
            recipes_[recipeSel_].name = edNameBuf_;   // label only; uid unchanged
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Recipe", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (haveSel)
            ImGui::Text("Delete recipe '%s'?", recipes_[recipeSel_].name.c_str());
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveSel) {
            recipes_.erase(recipes_.begin() + recipeSel_);
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
                                   ? FindStepByPath(&recipes_[recipeSel_].steps, edStepSel_)
                                   : nullptr;
        ImGui::Text("Delete step '%s'%s?", ds ? ds->type.c_str() : "?",
                    ds && !ds->children.empty() ? " and its children" : "");
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveSel && ds) {
            std::size_t idx = 0;
            if (std::vector<RecipeStep>* parent =
                    FindStepParent(&recipes_[recipeSel_].steps, edStepSel_, &idx)) {
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
        for (const Recipe& r : recipes_)
            if (r.uid == uid) return "run " + r.name;
        return "sub_recipe";
    }
    if (s.type == "stop_recipe")
        return s.params.value("ok", true) ? "stop (ok)" : "stop (fail)";

    const CommandSpec* spec = cmdRegistry_.Find(s.type);
    if (!spec) return s.type;

    const auto skip = [](const std::string& n) {
        return n == "timeout_ms" || n == "leg_timeout_ms" || n == "tol_mm" ||
               n == "start_tol_mm" || n == "poll_ms" || n == "max_iterations" ||
               n == "velocity_mm_s" || n == "exposure_us";
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
            const auto it = savedPos_.find(dev);
            if (it != savedPos_.end()) {
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
        }
        ImGui::PopID();

        if (executing || dim) ImGui::PopStyleColor();
        path.pop_back();
    }
}

// Tree toolbar: move within siblings + delete. Adding steps lives in the
// palette column. Mutations address steps through the tree's path encoding,
// so selection follows the edited step.
void App::DrawRecipeTreeToolbar(Recipe& r, bool canEdit)
{
    std::size_t idx = 0;
    std::vector<RecipeStep>* parent =
        edStepSel_.empty() ? nullptr : FindStepParent(&r.steps, edStepSel_, &idx);

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
        RecipeStep ns = MakeStep(type, cmdRegistry_);
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
    bool anyStruct = false;
    for (const StructuralStepInfo& t : kStructuralSteps) {
        if (PaletteMatch(t.type, edSearchBuf_)) { anyStruct = true; break; }
    }
    // Insert on DOUBLE-click only — a stray single click must not add steps
    // (same convention as the product list's double-click-to-load).
    if (anyStruct) ImGui::TextDisabled("Structure");
    for (const StructuralStepInfo& t : kStructuralSteps) {
        if (!PaletteMatch(t.type, edSearchBuf_)) continue;
        if (ImGui::Selectable(t.type, false, ImGuiSelectableFlags_AllowDoubleClick) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            insert(t.type);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\n(double-click to insert)", t.description);
    }
    std::string lastCat;
    for (const CommandSpec* c : cmdRegistry_.List()) {
        if (!PaletteMatch(c->name, edSearchBuf_) &&
            !PaletteMatch(c->category, edSearchBuf_))
            continue;
        if (c->category != lastCat) {
            lastCat = c->category;
            ImGui::Separator();
            ImGui::TextDisabled("%s", lastCat.c_str());
        }
        if (ImGui::Selectable(c->name.c_str(), false,
                              ImGuiSelectableFlags_AllowDoubleClick) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            insert(c->name.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s\n(double-click to insert)", c->description.c_str());
    }
    ImGui::EndChild();
    ImGui::EndDisabled();
}

// Step inspector: common fields + the schema-driven parameter form. Editable
// unless a run is active (then everything renders disabled).
void App::DrawRecipeInspector(RecipeStep& s, bool canEdit)
{
    ImGui::BeginDisabled(!canEdit);
    ImGui::Text("Type: %s", s.type.c_str());
    char nbuf[64];
    std::snprintf(nbuf, sizeof(nbuf), "%s", s.nickname.c_str());
    if (ImGui::InputText("nickname", nbuf, sizeof(nbuf))) s.nickname = nbuf;
    ImGui::SameLine();
    if (ImGui::SmallButton("gen")) s.nickname = AutoNickname(s);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Fill the nickname from the params.\n"
                          "An EMPTY nickname already shows this text in the tree,\n"
                          "auto-updating as params change.");
    ImGui::Checkbox("enabled", &s.enabled);
    int oe = static_cast<int>(s.onError);
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::Combo("on error", &oe, "abort\0continue\0retry\0"))
        s.onError = static_cast<RecipeStep::OnError>(oe);
    if (s.onError == RecipeStep::OnError::Retry) {
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::InputInt("retries", &s.retryCount) && s.retryCount < 0)
            s.retryCount = 0;
    }
    ImGui::Separator();

    if (s.type == "loop") {
        const bool isWhile = s.params.contains("while");
        if (ImGui::RadioButton("count", !isWhile) && isWhile) {
            s.params.erase("while");
            s.params.erase("max_iterations");
            s.params["count"] = 1;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("while", isWhile) && !isWhile) {
            s.params.erase("count");
            s.params["while"] = nlohmann::json::object();
        }
        if (s.params.contains("count")) {
            int c = s.params["count"].is_number()
                        ? static_cast<int>(s.params["count"].get<double>()) : 1;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("count##loop", &c)) s.params["count"] = c < 1 ? 1 : c;
        } else if (s.params.contains("while")) {
            DrawConditionEditor(s.params["while"]);
            int mi = s.params.value("max_iterations", kDefaultMaxIterations);
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("max iterations", &mi))
                s.params["max_iterations"] = mi < 1 ? 1 : mi;
        }
    } else if (s.type == "if") {
        DrawConditionEditor(s.params);
        ImGui::TextDisabled("Palette 'else' mode adds else-branch steps.");
    } else if (s.type == "sub_recipe") {
        const std::string uid = s.params.value("uid", std::string());
        const Recipe* target = nullptr;
        for (const Recipe& rr : recipes_) {
            if (rr.uid == uid) { target = &rr; break; }
        }
        if (ImGui::BeginCombo("recipe", target ? target->name.c_str() : "(pick)")) {
            for (const Recipe& rr : recipes_) {
                // Obvious self-reference filtered; deeper cycles are caught by
                // InlineSubRecipes at run start.
                if (recipeSel_ >= 0 && rr.uid == recipes_[recipeSel_].uid) continue;
                if (ImGui::Selectable(rr.name.c_str(), target == &rr))
                    s.params["uid"] = rr.uid;
            }
            ImGui::EndCombo();
        }
    } else if (s.type == "stop_recipe") {
        bool ok = s.params.value("ok", true);
        if (ImGui::Checkbox("success", &ok)) s.params["ok"] = ok;
        char mbuf[128];
        std::snprintf(mbuf, sizeof(mbuf), "%s", s.params.value("message", std::string()).c_str());
        if (ImGui::InputText("message", mbuf, sizeof(mbuf)))
            s.params["message"] = std::string(mbuf);
    } else if (s.type == "group") {
        ImGui::TextDisabled("Group: organizational only.");
    } else if (const CommandSpec* spec = cmdRegistry_.Find(s.type)) {
        ImGui::TextWrapped("%s", spec->description.c_str());
        ImGui::Spacing();
        for (const ParamSpec& p : spec->params) DrawParamWidget(p, s.params);
        const std::string err = ValidateParams(*spec, s.params, nullptr);
        if (!err.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", err.c_str());
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "unknown command '%s'",
                           s.type.c_str());
    }
    ImGui::EndDisabled();
}

// One schema-driven form row: reads params[p.name] (schema default when
// absent), writes back on edit. The widget is chosen by ParamType — the whole
// reason the typed registry exists. Returns true when the value changed.
bool App::DrawParamWidget(const ParamSpec& p, nlohmann::json& params)
{
    bool changed = false;
    ImGui::PushID(p.name.c_str());
    nlohmann::json cur = params.contains(p.name) ? params[p.name] : p.defaultValue;
    const char* label = p.name.c_str();
    const float w = 170.0f;

    switch (p.type) {
    case ParamType::Bool: {
        bool v = cur.is_boolean() && cur.get<bool>();
        if (ImGui::Checkbox(label, &v)) { params[p.name] = v; changed = true; }
        break;
    }
    case ParamType::Int: {
        int v = cur.is_number() ? static_cast<int>(cur.get<double>()) : 0;
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputInt(label, &v)) {
            if (p.minVal < p.maxVal) {
                if (v < p.minVal) v = static_cast<int>(p.minVal);
                if (v > p.maxVal) v = static_cast<int>(p.maxVal);
            }
            params[p.name] = v;
            changed = true;
        }
        break;
    }
    case ParamType::Double: {
        double v = cur.is_number() ? cur.get<double>() : 0.0;
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputDouble(label, &v, 0.0, 0.0, "%g")) {
            if (p.minVal < p.maxVal) {
                if (v < p.minVal) v = p.minVal;
                if (v > p.maxVal) v = p.maxVal;
            }
            params[p.name] = v;
            changed = true;
        }
        break;
    }
    case ParamType::String: {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s",
                      cur.is_string() ? cur.get<std::string>().c_str() : "");
        if (ImGui::InputText(label, buf, sizeof(buf))) {
            params[p.name] = std::string(buf);
            changed = true;
        }
        break;
    }
    case ParamType::Value: {
        int t = cur.is_boolean() ? 0 : cur.is_number() ? 1 : 2;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::Combo("##vtype", &t, "bool\0number\0string\0")) {
            params[p.name] = t == 0   ? nlohmann::json(false)
                             : t == 1 ? nlohmann::json(0.0)
                                      : nlohmann::json("");
            cur     = params[p.name];
            changed = true;
        }
        ImGui::SameLine();
        if (t == 0) {
            bool v = cur.is_boolean() && cur.get<bool>();
            if (ImGui::Checkbox(label, &v)) { params[p.name] = v; changed = true; }
        } else if (t == 1) {
            double v = cur.is_number() ? cur.get<double>() : 0.0;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputDouble(label, &v, 0.0, 0.0, "%g")) {
                params[p.name] = v;
                changed = true;
            }
        } else {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%s",
                          cur.is_string() ? cur.get<std::string>().c_str() : "");
            if (ImGui::InputText(label, buf, sizeof(buf))) {
                params[p.name] = std::string(buf);
                changed = true;
            }
        }
        break;
    }
    case ParamType::Enum: {
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, v.c_str())) {
            for (const std::string& o : p.options) {
                if (ImGui::Selectable(o.c_str(), o == v)) {
                    params[p.name] = o;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::DeviceMotion:
    case ParamType::DeviceIo:
    case ParamType::Slide: {
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str())) {
            const auto item = [&](const std::string& name) {
                if (ImGui::Selectable(name.c_str(), name == v)) {
                    params[p.name] = name;
                    changed = true;
                }
            };
            if (p.type == ParamType::DeviceMotion) {
                for (std::size_t i = 0; i < hardware_.MotionCount(); ++i)
                    item(hardware_.MotionName(i));
            } else if (p.type == ParamType::DeviceIo) {
                for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
                    item(hardware_.IoName(i));
            } else {
                for (std::size_t i = 0; i < hardware_.SlideCount(); ++i)
                    if (hardware_.Slide(i)) item(hardware_.Slide(i)->Name());
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::PositionUid: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview   = v.empty() ? "(pick)" : v;
        const auto it = savedPos_.find(dev);
        if (it != savedPos_.end()) {
            for (const SavedPos& sp : it->second)
                if (sp.uid == v) { preview = sp.name; break; }
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, preview.c_str())) {
            if (it == savedPos_.end() || it->second.empty())
                ImGui::TextDisabled("(no saved positions for '%s')", dev.c_str());
            else
                for (const SavedPos& sp : it->second) {
                    if (ImGui::Selectable(sp.name, sp.uid == v)) {
                        params[p.name] = sp.uid;   // stores the uid, shows the name
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::MapName: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str())) {
            const auto it = motionMaps_.find(dev);
            if (it == motionMaps_.end() || it->second.empty())
                ImGui::TextDisabled("(no maps for '%s')", dev.c_str());
            else
                for (const MotionMap& m : it->second) {
                    if (ImGui::Selectable(m.name.c_str(), m.name == v)) {
                        params[p.name] = m.name;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::PinName: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const IIoDevice* d = nullptr;
        for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == dev) { d = hardware_.IoDevice(i); break; }
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str())) {
            if (!d) {
                ImGui::TextDisabled("(pick a device first)");
            } else {
                const auto pins = [&](bool inputs) {
                    const int n = inputs ? d->InputCount() : d->OutputCount();
                    for (int pin = 0; pin < n; ++pin) {
                        const std::string name =
                            inputs ? d->InputPinName(pin) : d->OutputPinName(pin);
                        if (name.empty()) continue;
                        if (ImGui::Selectable(name.c_str(), name == v)) {
                            params[p.name] = name;
                            changed = true;
                        }
                    }
                };
                if (p.pinDir != PinDir::Output) pins(true);    // inputs
                if (p.pinDir != PinDir::Input)  pins(false);   // outputs
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::TopicName: {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s",
                      cur.is_string() ? cur.get<std::string>().c_str() : "");
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputText(label, buf, sizeof(buf))) {
            params[p.name] = std::string(buf);
            changed = true;
        }
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (!topics.empty()) {
            ImGui::SameLine();
            if (ImGui::BeginCombo("##topics", "", ImGuiComboFlags_NoPreview)) {
                for (const std::string& t : topics) {
                    if (ImGui::Selectable(t.c_str())) {
                        params[p.name] = t;
                        changed = true;
                    }
                }
                ImGui::EndCombo();
            }
        }
        break;
    }
    case ParamType::ScriptFile: {
        // Picker over the same scanned list the Scripts page shows (product
        // scripts dir, else config/scripts). Stores the path; shows the name.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        for (const ScriptInfo& si : scripts_) {
            if (si.path == v) { preview = si.name; break; }
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, preview.c_str())) {
            if (scripts_.empty())
                ImGui::TextDisabled("(no scripts in %s)", scriptsDirUsed_.c_str());
            for (const ScriptInfo& si : scripts_) {
                if (ImGui::Selectable(si.name.c_str(), si.path == v)) {
                    params[p.name] = si.path;
                    changed = true;
                }
                if (ImGui::IsItemHovered() && !si.description.empty())
                    ImGui::SetTooltip("%s", si.description.c_str());
            }
            ImGui::EndCombo();
        }
        break;
    }
    }
    if (!p.help.empty() && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", p.help.c_str());
    ImGui::PopID();
    return changed;
}

// Shared editor for {key|topic, op, value} condition objects (an if step's
// params, a loop's "while"). Normalizes the object in place so a fresh
// condition is immediately valid.
bool App::DrawConditionEditor(nlohmann::json& cond)
{
    bool changed = false;
    if (!cond.is_object()) { cond = nlohmann::json::object(); changed = true; }
    if (!cond.contains("op")) { cond["op"] = "is_true"; changed = true; }
    if (!cond.contains("key") && !cond.contains("topic")) {
        cond["key"] = "";
        changed = true;
    }

    int src = cond.contains("topic") ? 1 : 0;
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::Combo("source", &src, "ctx key\0topic\0")) {
        const std::string old = src == 1 ? cond.value("key", std::string())
                                         : cond.value("topic", std::string());
        cond.erase("key");
        cond.erase("topic");
        cond[src == 1 ? "topic" : "key"] = old;
        changed = true;
    }
    const char* fk = cond.contains("topic") ? "topic" : "key";
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s", cond.value(fk, std::string()).c_str());
    if (ImGui::InputText(fk, buf, sizeof(buf))) {
        cond[fk] = std::string(buf);
        changed = true;
    }

    static const char* kOps[] = {">", "<", ">=", "<=", "==", "!=",
                                 "is_true", "is_false", "exists", "not_exists"};
    const std::string op = cond.value("op", std::string("is_true"));
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::BeginCombo("op", op.c_str())) {
        for (const char* o : kOps) {
            if (ImGui::Selectable(o, op == o)) {
                cond["op"] = o;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    const bool needsValue = op != "is_true" && op != "is_false" &&
                            op != "exists" && op != "not_exists";
    if (needsValue) {
        nlohmann::json v = cond.contains("value") ? cond["value"] : nlohmann::json(0.0);
        int t = v.is_string() ? 1 : 0;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::Combo("##ctype", &t, "number\0string\0")) {
            cond["value"] = t == 1 ? nlohmann::json("") : nlohmann::json(0.0);
            v             = cond["value"];
            changed       = true;
        }
        ImGui::SameLine();
        if (t == 1) {
            char vb[128];
            std::snprintf(vb, sizeof(vb), "%s",
                          v.is_string() ? v.get<std::string>().c_str() : "");
            if (ImGui::InputText("value", vb, sizeof(vb))) {
                cond["value"] = std::string(vb);
                changed = true;
            }
        } else {
            double d = v.is_number() ? v.get<double>() : 0.0;
            ImGui::SetNextItemWidth(130.0f);
            if (ImGui::InputDouble("value", &d, 0.0, 0.0, "%g")) {
                cond["value"] = d;
                changed = true;
            }
        }
    }
    return changed;
}

// Write the in-memory recipes to the product's recipes file when a product is
// loaded (manifest uid stamped), else config/recipes.local.json — the same
// precedence LoadRecipesFromDisk reads with.
void App::SaveRecipes()
{
    std::string path = "config/recipes.local.json";
    std::string uid  = recipesUid_;
    if (product_.Loaded()) {
        if (const ManifestFile* e = product_.Entry("recipes")) {
            path = product_.FilePath("recipes");
            uid  = e->uid;
        }
    }
    if (uid.empty()) uid = GenerateUid();
    recipesUid_ = uid;
    if (SaveRecipesFile(path, recipes_, uid)) {
        recipesBaseline_ = RecipesDump();
        FD_INFO("recipe", "saved %zu recipe(s) to %s", recipes_.size(), path.c_str());
    } else {
        FD_WARN("recipe", "could not save recipes to %s", path.c_str());
    }
}

// Bottom strip: Run/Stop/Save + state + executing step + progress + the
// clickable validation-issues list. Runs the recipe selected in the left
// panel through the engine (positions/maps copied at Start, so mid-run edits
// elsewhere in the UI can't disturb the run).
void App::DrawRecipeRunStrip(const std::vector<RecipeIssue>& issues)
{
    const RecipeEngine::Status st = recipeEngine_.GetStatus();
    const bool running = recipeEngine_.Running();
    ImGui::BeginChild("edRunStrip", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);

    const bool canRun = !OrchestrationBusy() && recipeSel_ >= 0 &&
                        recipeSel_ < static_cast<int>(recipes_.size());
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Run", ImVec2(90.0f, 0.0f))) {
        RecipeEngine::Env env;
        env.positions = savedPos_;
        env.maps      = motionMaps_;
        env.library   = recipes_;
        recipeStartErr_.clear();
        if (!recipeEngine_.Start(recipes_[recipeSel_], std::move(env), &recipeStartErr_))
            FD_WARN("recipe", "start rejected: %s", recipeStartErr_.c_str());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    // Run ONLY the selected row (children included), as a throwaway one-step
    // recipe through the same engine — same validation, stop, and progress.
    const RecipeStep* selStep =
        (canRun && !edStepSel_.empty())
            ? FindStepByPath(&recipes_[recipeSel_].steps, edStepSel_)
            : nullptr;
    ImGui::BeginDisabled(!selStep);
    if (ImGui::Button("Run Step", ImVec2(90.0f, 0.0f)) && selStep) {
        Recipe one;
        one.uid  = "step-run";
        one.name = "step: " + (selStep->nickname.empty() ? AutoNickname(*selStep)
                                                         : selStep->nickname);
        one.steps.push_back(*selStep);
        one.steps.front().enabled = true;   // a disabled row can still be tested
        RecipeEngine::Env env;
        env.positions = savedPos_;
        env.maps      = motionMaps_;
        env.library   = recipes_;   // sub_recipe references still resolve
        recipeStartErr_.clear();
        if (!recipeEngine_.Start(one, std::move(env), &recipeStartErr_))
            FD_WARN("recipe", "step run rejected: %s", recipeStartErr_.c_str());
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Run only the selected step (its children included)");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(90.0f, 0.0f))) recipeEngine_.RequestStop();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(running);
    if (ImGui::Button(RecipesDirty() ? "Save*" : "Save", ImVec2(90.0f, 0.0f)))
        SaveRecipes();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered() && RecipesDirty())
        ImGui::SetTooltip("Unsaved changes");
    ImGui::SameLine();
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
    if (!recipeStartErr_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           recipeStartErr_.c_str());
    } else if (!st.error.empty() && st.state != RecipeEngine::State::Running) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
    }
    if (st.leafTotal > 0) {
        float frac = static_cast<float>(st.leafDone) / static_cast<float>(st.leafTotal);
        if (frac > 1.0f) frac = 1.0f;   // while-loops can overshoot the estimate
        char overlay[48];
        std::snprintf(overlay, sizeof(overlay), "%zu / %zu", st.leafDone, st.leafTotal);
        ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), overlay);
    }
    if (!issues.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%zu issue(s) - click to select:",
                           issues.size());
        ImGui::BeginChild("##valList");   // scrolls when more than the strip shows
        for (std::size_t k = 0; k < issues.size(); ++k) {
            const RecipeIssue& is = issues[k];
            ImGui::PushID(static_cast<int>(k));
            if (ImGui::Selectable((is.where + ": " + is.message).c_str()) &&
                !is.path.empty())
                edStepSel_ = is.path;
            ImGui::PopID();
        }
        ImGui::EndChild();
    }
    ImGui::EndChild();
}

// Global orchestration prompt — drawn every frame from DrawMainWindow (like
// the safety modal), so a blocking recipe `prompt` step or script fd.prompt()
// surfaces no matter which page or window the operator is on. Only one engine
// runs at a time, so at most one prompt can be pending.
void App::DrawRecipePromptModal()
{
    const RecipeEngine::Status rst = recipeEngine_.GetStatus();
    const LuaEngine::Status    lst = luaEngine_.GetStatus();
    const bool fromScript = rst.pendingPrompt.empty() && !lst.pendingPrompt.empty();
    const std::string& pending = fromScript ? lst.pendingPrompt : rst.pendingPrompt;
    const bool wantsInput =
        fromScript ? lst.pendingPromptInput : rst.pendingPromptInput;

    if (!pending.empty() && !ImGui::IsPopupOpen("Recipe Prompt")) {
        promptInputBuf_[0] = '\0';
        ImGui::OpenPopup("Recipe Prompt");
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Recipe Prompt", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 420.0f);
        ImGui::TextUnformatted(pending.c_str());
        ImGui::PopTextWrapPos();
        const auto answer = [&](bool ok) {
            if (fromScript) luaEngine_.AnswerPrompt(ok, promptInputBuf_);
            else            recipeEngine_.AnswerPrompt(ok, promptInputBuf_);
        };
        bool submit = false;
        if (wantsInput) {
            // Barcode scanners type + Enter, so Enter submits like OK.
            if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
            ImGui::SetNextItemWidth(300.0f);
            submit = ImGui::InputText("##promptin", promptInputBuf_,
                                      sizeof(promptInputBuf_),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
        }
        ImGui::Separator();
        if (ImGui::Button("OK", ImVec2(100.0f, 0.0f)) || submit) {
            answer(true);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) {
            answer(false);
            ImGui::CloseCurrentPopup();
        }
        if (pending.empty()) ImGui::CloseCurrentPopup();   // stop released it
        ImGui::EndPopup();
    }
}

// ---- Programs page (View menu, full page) — Phase C1: list + run .lua -------
// Authoring happens in an external editor; this page lists, shows read-only
// source (executing line highlighted), and runs/stops programs.

std::string App::ScriptsDir() const
{
    return product_.Loaded() ? product_.Dir() + "/scripts" : "config/scripts";
}

void App::LoadScriptsList()
{
    scriptsDirUsed_ = ScriptsDir();
    scripts_ = ListScripts(scriptsDirUsed_);
    if (scripts_.empty() && product_.Loaded()) {
        // Product without a scripts folder — fall back like recipes do, and
        // show the directory ACTUALLY used, not the one we wished for.
        scriptsDirUsed_ = "config/scripts";
        scripts_ = ListScripts(scriptsDirUsed_);
    }
    if (scriptSel_ >= static_cast<int>(scripts_.size()))
        scriptSel_ = scripts_.empty() ? -1 : 0;
    scriptSrcPath_.clear();   // force a source re-read on next draw
    scriptSrc_.clear();
    scriptStartErr_.clear();
    scriptBpAnchor_ = -1;
}

void App::DrawScriptsPanel()
{
    const LuaEngine::Status st = luaEngine_.GetStatus();
    const bool running = luaEngine_.Running();

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
    ImGui::TextDisabled("%s", scriptsDirUsed_.c_str());
    ImGui::Separator();
    if (scripts_.empty())
        ImGui::TextWrapped("No .lua scripts found. Author them in an external "
                           "editor and hit Reload.");
    for (int i = 0; i < static_cast<int>(scripts_.size()); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(scripts_[i].name.c_str(), scriptSel_ == i)) {
            scriptSel_      = i;
            scriptBpAnchor_ = -1;   // anchor is per-script
        }
        if (ImGui::IsItemHovered() && !scripts_[i].description.empty())
            ImGui::SetTooltip("%s", scripts_[i].description.c_str());
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Middle: read-only source, executing line highlighted ----
    const float midW = avail.x - leftW - rightW - ImGui::GetStyle().ItemSpacing.x * 2.0f;
    ImGui::BeginChild("scSrc", ImVec2(midW, paneH), ImGuiChildFlags_Borders);
    if (scriptSel_ >= 0 && scriptSel_ < static_cast<int>(scripts_.size())) {
        const ScriptInfo& si = scripts_[scriptSel_];
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
                if (thisRunning) luaEngine_.SetBreakpoints(bps);   // hot-swap live
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
    const bool canRun = !OrchestrationBusy() && scriptSel_ >= 0 &&
                        scriptSel_ < static_cast<int>(scripts_.size());
    ImGui::BeginDisabled(!canRun);
    if (ImGui::Button("Run", ImVec2(90.0f, 0.0f))) {
        LuaEngine::Env env;
        env.positions = savedPos_;
        env.maps      = motionMaps_;
        env.library   = recipes_;
        scriptStartErr_.clear();
        luaEngine_.SetBreakpoints(scriptBps_[scripts_[scriptSel_].path]);
        if (!luaEngine_.Start(scripts_[scriptSel_].path, std::move(env), &scriptStartErr_))
            FD_WARN("script", "start rejected: %s", scriptStartErr_.c_str());
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (!st.paused) {
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Pause", ImVec2(90.0f, 0.0f))) luaEngine_.Pause();
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Continue", ImVec2(90.0f, 0.0f))) luaEngine_.Resume();
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(90.0f, 0.0f))) luaEngine_.StepLine();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop", ImVec2(90.0f, 0.0f))) luaEngine_.RequestStop();
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

void App::NewProduct(const char* name)
{
    ProductManifest m = CreateProduct(name);
    if (!m.Loaded()) return;
    product_ = std::move(m);
    DutSession::Instance().SetProduct(product_.name, product_.uid,
                                      product_.DataDir());
    SavePositions();     // carry the current in-memory positions into the product
    SaveUiSettings();
}

void App::SaveProduct()
{
    if (!product_.Loaded()) return;
    // "Save runs through the manifest": each entry is dispatched by role.
    // Future roles (recipes/programs) are skipped until they have real data.
    for (const ManifestFile& e : product_.files) {
        const std::string path = product_.Dir() + "/" + e.file;
        if (e.role == "positions") {
            SaveSavedPositionsFile(path, savedPos_, e.uid);
        } else if (e.role == "recipes") {
            SaveRecipesFile(path, recipes_, e.uid);
            recipesUid_      = e.uid;
            recipesBaseline_ = RecipesDump();
        } else if (e.role == "programs") {
            // placeholder — Lua programs arrive in Phase C
        } else {
            FD_WARN("product", "unknown manifest role '%s' (skipped)", e.role.c_str());
        }
    }
    SaveProductManifest(product_);
    FD_INFO("product", "saved '%s' (%zu manifest file(s))",
            product_.name.c_str(), product_.files.size());
}

void App::DrawProductPopups()
{
    // OpenPopup must run in this scope (not inside the menu), so the File menu
    // only raises flags. Center both modals on the main viewport.
    if (openLoadProduct_) { ImGui::OpenPopup("Load Product"); openLoadProduct_ = false; }
    if (openNewProduct_)  { ImGui::OpenPopup("New Product");  openNewProduct_  = false; }
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal("Load Product", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const std::vector<std::string> products = ListProducts();
        if (products.empty()) {
            ImGui::TextDisabled("No products found under products\\ - use New Product.");
        } else {
            ImGui::TextUnformatted("Double-click a product to load it:");
            if (ImGui::BeginListBox("##products", ImVec2(320.0f, 200.0f))) {
                for (const std::string& p : products) {
                    const bool current = (p == product_.name);
                    if (ImGui::Selectable((p + (current ? "  [loaded]" : "")).c_str(), current,
                                          ImGuiSelectableFlags_AllowDoubleClick)) {
                        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                            LoadProduct(p);
                            ImGui::CloseCurrentPopup();
                        }
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Double-click to load");
                }
                ImGui::EndListBox();
            }
        }
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("New Product", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Product name (folder under products\\):");
        ImGui::SetNextItemWidth(320.0f);
        const bool submit = ImGui::InputText("##newname", newProductName_, sizeof(newProductName_),
                                             ImGuiInputTextFlags_EnterReturnsTrue);
        // Valid: non-empty, no path separators, not already existing.
        std::string nm = newProductName_;
        const bool badChars = nm.find_first_of("\\/:*?\"<>|") != std::string::npos;
        bool exists = false;
        for (const std::string& p : ListProducts()) if (p == nm) { exists = true; break; }
        const bool valid = !nm.empty() && !badChars && !exists;
        if (badChars) ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "Name contains invalid characters.");
        if (exists)   ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "A product with this name already exists.");

        ImGui::BeginDisabled(!valid);
        if (ImGui::Button("Create") || (submit && valid)) {
            NewProduct(nm.c_str());
            newProductName_[0] = '\0';
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) { newProductName_[0] = '\0'; ImGui::CloseCurrentPopup(); }
        ImGui::EndPopup();
    }
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
    const std::vector<std::string> axes = dev->Axes();
    const size_t                   di   = static_cast<size_t>(jogDevice_);
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

    DrawVelocityCombo(di);

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

    // --- Rotational cluster (GLOBAL U/V/W = theta x/y/z), only if the device
    // has rotational axes. Routed through JogGlobalRot so the transform's
    // rotation block maps global -> signed local axis (test it live from the
    // Transforms editor). jogRotAxis_ selects the GLOBAL rotation axis.
    const int rotCount = static_cast<int>(axes.size()) - 3;
    if (rotCount > 0) {
        if (jogRotAxis_ < 0 || jogRotAxis_ >= 3) jogRotAxis_ = 0;
        static const char* kRotNames[3] = {"U \xce\xb8x", "V \xce\xb8y", "W \xce\xb8z"};

        // Mapped-target readout: dominant entry of the rotation block's column,
        // from the same working copy the Transforms editor hot-applies.
        std::string mapped = "unmapped";
        {
            AxisTransform t;   // identity if none configured
            const auto it = transformCfg_.find(dev->Name());
            if (it != transformCfg_.end()) t = it->second;
            int ax = 0;
            for (int i = 1; i < 3; ++i)
                if (std::fabs(t.r[i][jogRotAxis_]) > std::fabs(t.r[ax][jogRotAxis_])) ax = i;
            if (std::fabs(t.r[ax][jogRotAxis_]) > 1e-9 && 3 + ax < static_cast<int>(axes.size()))
                mapped = axes[3 + ax] + (t.r[ax][jogRotAxis_] >= 0.0 ? "+" : "-");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Rotational");
        ImGui::SameLine();
        ImGui::TextDisabled("-> local %s", mapped.c_str());
        ImGui::BeginGroup();
        for (int i = 0; i < 3; ++i) {
            const bool sel = (jogRotAxis_ == i);
            if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.55f, 0.10f, 1.0f));
            if (ImGui::Button(kRotNames[i], ImVec2(jb.x, 0.0f))) jogRotAxis_ = i;
            if (sel) ImGui::PopStyleColor();
        }
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        const float gu = (jogRotAxis_ == 0) ? 1.0f : 0.0f;
        const float gv = (jogRotAxis_ == 1) ? 1.0f : 0.0f;
        const float gw = (jogRotAxis_ == 2) ? 1.0f : 0.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("+##rot", ImVec2(jb.x, 0.0f))) workflow_.JogGlobalRot(di, gu, gv, gw, +step);
        ImGui::PopStyleColor();
        char stepReadout[24];
        std::snprintf(stepReadout, sizeof(stepReadout), "%s##rotstep", kSteps[jogStepIndex_].label);
        ImGui::Button(stepReadout, ImVec2(jb.x, 0.0f));   // step readout (unique id)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("-##rot", ImVec2(jb.x, 0.0f))) workflow_.JogGlobalRot(di, gu, gv, gw, -step);
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

// Pull the newest camera frame into the shared GL texture. Safe to call from
// several views in one frame — only a fresh frame re-uploads.
void App::UpdateCameraTexture()
{
    bool isNew = false;
    if (hardware_.Camera().GetLatestFrame(frameBuf_, cameraW_, cameraH_, isNew) &&
        cameraW_ > 0 && cameraH_ > 0) {
        if (isNew || cameraTex_ == 0) {
            cameraTex_ = host_.EnsureTextureRGB(cameraTex_, cameraW_, cameraH_, frameBuf_.data());
        }
    }
}

// ---- Camera page ----------------------------------------------------------
// Left: status + connect/disconnect. Middle: exposure + pixel->mm calibration
// + live preview. Right: identity/stream info. Commands go through Workflow;
// reads come from the HardwareManager accessors — same rule as every page.

void App::DrawCameraListPanel()
{
    ImGui::TextUnformatted("Camera");
    ImGui::Separator();
    if (cameraId_.empty()) {
        ImGui::TextDisabled("no camera configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.CameraState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.CameraName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectCamera();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectCamera();
        camExpoPulled_ = false;   // re-pull exposure on the next connect
    }
    ImGui::EndDisabled();

    if (st == ConnState::Failed) {
        const std::string err = hardware_.Camera().LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawCameraDetailPanel()
{
    const ConnState st  = hardware_.CameraState();
    PylonCamera&    cam = hardware_.Camera();

    // ---- Exposure (µs; log slider — useful ranges span decades) ----
    ImGui::TextUnformatted("Exposure");
    ImGui::Separator();
    if (st != ConnState::Connected) {
        ImGui::TextDisabled("connect the camera to adjust exposure");
        camExpoPulled_ = false;
    } else {
        if (!camExpoPulled_) {   // seed the edit value from the camera once
            const double cur = cam.ExposureUs();
            if (cur > 0.0) { camExposureUs_ = static_cast<float>(cur); camExpoPulled_ = true; }
        }
        double lo = 0.0, hi = 0.0;
        cam.ExposureRange(lo, hi);
        if (lo <= 0.0 || hi <= lo) { lo = 20.0; hi = 100000.0; }   // sane fallback

        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::SliderFloat("##expo", &camExposureUs_,
                           static_cast<float>(lo), static_cast<float>(hi),
                           "%.0f us", ImGuiSliderFlags_Logarithmic);
        if (ImGui::IsItemDeactivatedAfterEdit())          // apply once on release
            workflow_.SetCameraExposure(camExposureUs_);

        ImGui::SetNextItemWidth(140.0f);
        ImGui::InputFloat("us##expoIn", &camExposureUs_, 0.0f, 0.0f, "%.0f");
        ImGui::SameLine();
        if (ImGui::Button("Apply")) workflow_.SetCameraExposure(camExposureUs_);
        ImGui::TextDisabled("range %.0f - %.0f us", lo, hi);
    }

    ImGui::Spacing();
    // ---- Pixel -> mm calibration (drives Aux overlay + click-to-move live) ----
    ImGui::TextUnformatted("Pixel -> mm calibration");
    ImGui::Separator();
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputFloat("X mm/px", &pixelToMmX_, 0.0f, 0.0f, "%.5f");
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputFloat("Y mm/px", &pixelToMmY_, 0.0f, 0.0f, "%.5f");
    if (ImGui::Button("Save to config")) {
        if (HardwareConfig::SaveCameraSettings("config/hardware.json", cameraId_,
                                               pixelToMmX_, pixelToMmY_,
                                               camExpoPulled_ ? camExposureUs_ : 0.0))
            FD_INFO("app", "camera settings saved to config/hardware.json");
        else
            FD_ERROR("app", "failed to save camera settings");
    }
    ImGui::TextDisabled("Edits apply immediately (Aux overlay + click-to-move). Save also "
                        "stores the exposure; a rebuild recopies the repo config over the "
                        "exe-side file.");

    ImGui::Spacing();
    // ---- Live preview (fit to the panel, aspect preserved) ----
    ImGui::TextUnformatted("Preview");
    ImGui::Separator();
    UpdateCameraTexture();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (cameraTex_ && cameraW_ > 0 && cameraH_ > 0 && avail.x > 20.0f && avail.y > 20.0f) {
        float w = avail.x;
        float h = w * static_cast<float>(cameraH_) / static_cast<float>(cameraW_);
        if (h > avail.y) { h = avail.y; w = h * static_cast<float>(cameraW_) / static_cast<float>(cameraH_); }
        ImGui::Image((ImTextureID)(intptr_t)cameraTex_, ImVec2(w, h));   // C-style cast: void*/ImU64 ImTextureID
    } else {
        ImGui::TextDisabled("no frames yet");
    }
}

void App::DrawCameraInfoPanel()
{
    ImGui::TextUnformatted("Camera info");
    ImGui::Separator();

    const ConnState         st   = hardware_.CameraState();
    const PylonCamera::Info info = hardware_.Camera().GetInfo();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Model");
    ImGui::TextWrapped("%s", info.model.empty() ? "-" : info.model.c_str());
    ImGui::TextDisabled("Serial");
    ImGui::TextUnformatted(info.serial.empty() ? "-" : info.serial.c_str());
    ImGui::TextDisabled("IP");
    ImGui::TextUnformatted(info.ip.empty() ? "-" : info.ip.c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("Stream");
    if (info.grabbing) {
        if (info.width > 0) ImGui::Text("%d x %d px", info.width, info.height);
        else                ImGui::TextUnformatted("waiting for first frame");
        ImGui::Text("%.1f fps", info.fps);
    } else {
        ImGui::TextUnformatted("not grabbing");
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Calibration");
    ImGui::Text("X %.5f mm/px", pixelToMmX_);
    ImGui::Text("Y %.5f mm/px", pixelToMmY_);
}

// ---- Keithley page --------------------------------------------------------
// Left: status + connect/disconnect. Middle: bias output + live-adjustable
// measurement settings + the shared chart. Right: identity + current settings
// + live values. Commands go through Workflow; reads come from the driver's
// cheap state accessors (GetConfig/BiasVolts never touch VISA).

void App::DrawKeithleyListPanel()
{
    ImGui::TextUnformatted("Keithley 6482");
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

    Keithley6482* kei = hardware_.Keithley();
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
    Keithley6482*   kei = hardware_.Keithley();
    const bool connected = (st == ConnState::Connected) && kei;

    if (connected && !keiPulled_) {   // seed edits from the live driver once
        const Keithley6482::Config cfg = kei->GetConfig();
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
    Keithley6482*   kei = hardware_.Keithley();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    ImGui::TextWrapped("%s", (kei && !kei->DeviceId().empty()) ? kei->DeviceId().c_str() : "-");

    if (kei) {
        const Keithley6482::Config cfg = kei->GetConfig();
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

// ---- CLD101x page ---------------------------------------------------------
// Left: status + connect/disconnect. Middle: TEC (temperature setpoint +
// on/off + live reading) and Laser (current setpoint + on/off + live reading).
// Right: identity + live values. Turning the laser on is always an explicit
// button press — nothing is enabled automatically at connect.

void App::DrawCldListPanel()
{
    ImGui::TextUnformatted("CLD101x");
    ImGui::Separator();
    if (cldId_.empty()) {
        ImGui::TextDisabled("no CLD101x configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.CldState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.CldName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectCld();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectCld();
        cldPulled_ = false;   // re-seed edits on the next connect
    }
    ImGui::EndDisabled();

    if (st == ConnState::Failed && hardware_.Cld()) {
        const std::string err = hardware_.Cld()->LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawCldDetailPanel()
{
    const ConnState st  = hardware_.CldState();
    CLD101x*        cld = hardware_.Cld();
    const bool connected = (st == ConnState::Connected) && cld;
    const CLD101x::Status s = cld ? cld->GetStatus() : CLD101x::Status{};

    if (connected && !cldPulled_) {   // seed edits from the device's setpoints
        if (s.tempSetC   != 0.0) cldTempSet_    = static_cast<float>(s.tempSetC);
        if (s.laserSetMA != 0.0) cldCurrentSet_ = static_cast<float>(s.laserSetMA);
        cldPulled_ = true;
    }
    if (!connected) cldPulled_ = false;

    // ---- TEC (temperature control; run it before enabling the laser) ----
    ImGui::TextUnformatted("TEC");
    ImGui::Separator();
    ImGui::BeginDisabled(!connected);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("C##temp", &cldTempSet_, 0.0f, 0.0f, "%.2f");
    if (cldTempSet_ < 15.0f) cldTempSet_ = 15.0f;   // sane laser-diode window
    if (cldTempSet_ > 35.0f) cldTempSet_ = 35.0f;
    ImGui::SameLine();
    if (ImGui::Button("Apply##temp")) workflow_.SetCldTemperature(cldTempSet_);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
    if (ImGui::Button("TEC ON")) workflow_.CldTecOn();
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button("TEC OFF")) workflow_.CldTecOff();
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (connected) {
        char tval[32];
        if (s.tempValid) std::snprintf(tval, sizeof(tval), "%.2f C", s.tempC);
        else             std::snprintf(tval, sizeof(tval), "-- C (no sensor)");
        if (s.tecOn) ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f),
                                        "TEC ON  |  %s (set %.2f)", tval, s.tempSetC);
        else         ImGui::TextDisabled("TEC OFF  |  %s", tval);
        if (s.valid && !s.tempValid)
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.20f, 1.0f),
                               "temperature reads invalid - thermistor disconnected?");
    } else {
        ImGui::TextDisabled("connect the controller to operate it");
    }

    ImGui::Spacing();
    // ---- Laser (current control; ON is always an explicit action) ----
    ImGui::TextUnformatted("Laser");
    ImGui::Separator();
    ImGui::BeginDisabled(!connected);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("mA##ld", &cldCurrentSet_, 0.0f, 0.0f, "%.1f");
    if (cldCurrentSet_ < 0.0f)   cldCurrentSet_ = 0.0f;
    if (cldCurrentSet_ > 500.0f) cldCurrentSet_ = 500.0f;   // CLD101x LD range cap
    ImGui::SameLine();
    if (ImGui::Button("Apply##ld")) workflow_.SetCldLaserCurrentMA(cldCurrentSet_);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
    if (ImGui::Button("Laser ON")) workflow_.CldLaserOn();
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button("Laser OFF")) workflow_.CldLaserOff();
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (connected) {
        if (s.laserOn) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f),
                                          "LASER ON  |  %.1f mA (set %.1f)",
                                          s.laserActMA, s.laserSetMA);
        else           ImGui::TextDisabled("laser OFF  |  set %.1f mA", s.laserSetMA);
    }
    if (connected && s.laserOn && !s.tecOn) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.20f, 1.0f),
                           "warning: laser is ON with TEC OFF");
    }

    ImGui::Spacing();
    if (ImGui::Button("Save to config")) {
        if (HardwareConfig::SaveCldSettings("config/hardware.json", cldId_,
                                            cldTempSet_, cldCurrentSet_))
            FD_INFO("app", "CLD101x defaults saved to config/hardware.json");
        else
            FD_ERROR("app", "failed to save CLD101x defaults");
    }
    ImGui::TextDisabled("Save stores the setpoints as the config defaults (UI seeds — "
                        "never applied automatically); a rebuild recopies the repo config.");
}

void App::DrawCldInfoPanel()
{
    ImGui::TextUnformatted("Controller info");
    ImGui::Separator();

    const ConnState st  = hardware_.CldState();
    CLD101x*        cld = hardware_.Cld();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    ImGui::TextWrapped("%s", (cld && !cld->DeviceId().empty()) ? cld->DeviceId().c_str() : "-");

    if (cld) {
        const CLD101x::Status s = cld->GetStatus();
        ImGui::Spacing();
        ImGui::TextDisabled("TEC");
        ImGui::TextUnformatted(s.tecOn ? "ON" : "OFF");
        if (s.valid && s.tempValid) ImGui::Text("%.2f C (set %.2f)", s.tempC, s.tempSetC);
        else if (s.valid)           ImGui::TextDisabled("no sensor reading (thermistor?)");
        else                        ImGui::TextDisabled("no reading yet");
        ImGui::Spacing();
        ImGui::TextDisabled("Laser");
        if (s.laserOn) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "ON");
        else           ImGui::TextUnformatted("OFF");
        ImGui::Text("%.1f mA (set %.1f)", s.laserActMA, s.laserSetMA);
        ImGui::Spacing();
        ImGui::TextDisabled("Polling");
        ImGui::TextUnformatted(cld->IsPolling() ? "yes" : "no");
    }
}

// ---- PSU page (UNI-T UDP3305S) --------------------------------------------
// Left: status + connect/disconnect. Middle: per-channel V/I setpoints (the
// verified APPLy form) + output ON/OFF (read-back verified). Right: identity +
// live values. Turning an output on is always an explicit button press —
// nothing is enabled automatically at connect; disconnect forces all OFF.

namespace {
// Datasheet channel limits: CH1/CH2 0..32 V / 0..5 A, CH3 0..6 V / 0..3 A.
float PsuMaxVolts(int ch) { return ch == 3 ? 6.0f : 32.0f; }
float PsuMaxAmps(int ch)  { return ch == 3 ? 3.0f : 5.0f; }
} // namespace

void App::DrawPsuListPanel()
{
    ImGui::TextUnformatted("Power supply");
    ImGui::Separator();
    if (psuId_.empty()) {
        ImGui::TextDisabled("no PSU configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.PsuState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.PsuName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectPsu();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectPsu();
        psuPulled_ = false;   // re-seed edits on the next connect
    }
    ImGui::EndDisabled();
    if (st == ConnState::Connected)
        ImGui::TextDisabled("disconnect forces all outputs OFF");

    if (st == ConnState::Failed && hardware_.Psu()) {
        const std::string err = hardware_.Psu()->LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawPsuDetailPanel()
{
    const ConnState st  = hardware_.PsuState();
    UDP3305S*       psu = hardware_.Psu();
    const bool connected = (st == ConnState::Connected) && psu;
    const UDP3305S::Status s = psu ? psu->GetStatus() : UDP3305S::Status{};
    const int channels = psu ? psu->Channels() : 3;

    if (connected && !psuPulled_) {   // seed edits from the device's setpoints
        for (int ch = 0; ch < channels; ++ch) {
            if (s.ch[ch].setV > 0.0) psuVSet_[ch] = static_cast<float>(s.ch[ch].setV);
            if (s.ch[ch].setI > 0.0) psuISet_[ch] = static_cast<float>(s.ch[ch].setI);
        }
        psuPulled_ = true;
    }
    if (!connected) psuPulled_ = false;

    for (int ch = 1; ch <= channels; ++ch) {
        const UDP3305S::ChannelStatus& c = s.ch[ch - 1];
        ImGui::PushID(ch);
        ImGui::Text("CH%d", ch);
        ImGui::SameLine();
        ImGui::TextDisabled("(0-%.0f V / 0-%.0f A)", PsuMaxVolts(ch), PsuMaxAmps(ch));
        ImGui::Separator();

        ImGui::BeginDisabled(!connected);
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("V##v", &psuVSet_[ch - 1], 0.0f, 0.0f, "%.3f");
        if (psuVSet_[ch - 1] < 0.0f)             psuVSet_[ch - 1] = 0.0f;
        if (psuVSet_[ch - 1] > PsuMaxVolts(ch))  psuVSet_[ch - 1] = PsuMaxVolts(ch);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::InputFloat("A##i", &psuISet_[ch - 1], 0.0f, 0.0f, "%.3f");
        if (psuISet_[ch - 1] < 0.0f)            psuISet_[ch - 1] = 0.0f;
        if (psuISet_[ch - 1] > PsuMaxAmps(ch))  psuISet_[ch - 1] = PsuMaxAmps(ch);
        ImGui::SameLine();
        if (ImGui::Button("Apply"))
            workflow_.SetPsuVI(ch, psuVSet_[ch - 1], psuISet_[ch - 1]);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
        if (ImGui::Button("Output ON")) workflow_.PsuOutputOn(ch);
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
        if (ImGui::Button("Output OFF")) workflow_.PsuOutputOff(ch);
        ImGui::PopStyleColor();
        ImGui::EndDisabled();

        if (connected) {
            if (c.outputOn)
                ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f),
                                   "ON  |  %.3f V  %.3f A (set %.3f V / %.3f A)",
                                   c.measV, c.measI, c.setV, c.setI);
            else
                ImGui::TextDisabled("OFF  |  set %.3f V / %.3f A", c.setV, c.setI);
        }
        ImGui::Spacing();
        ImGui::PopID();
    }
    if (!connected) ImGui::TextDisabled("connect the power supply to operate it");
}

void App::DrawPsuInfoPanel()
{
    ImGui::TextUnformatted("Supply info");
    ImGui::Separator();

    const ConnState st  = hardware_.PsuState();
    UDP3305S*       psu = hardware_.Psu();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    ImGui::TextWrapped("%s", (psu && !psu->DeviceId().empty()) ? psu->DeviceId().c_str() : "-");

    if (psu) {
        const UDP3305S::Status s = psu->GetStatus();
        for (int ch = 1; ch <= psu->Channels(); ++ch) {
            const UDP3305S::ChannelStatus& c = s.ch[ch - 1];
            ImGui::Spacing();
            ImGui::TextDisabled("CH%d", ch);
            if (c.outputOn) ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "ON");
            else            ImGui::TextUnformatted("OFF");
            if (s.valid) ImGui::Text("%.3f V  %.3f A", c.measV, c.measI);
            else         ImGui::TextDisabled("no reading yet");
            ImGui::Text("set %.3f V / %.3f A", c.setV, c.setI);
        }
        ImGui::Spacing();
        ImGui::TextDisabled("Polling");
        ImGui::TextUnformatted(psu->IsPolling() ? "yes" : "no");
    }
}

// ---- PI / ACS pages (motion fleet) ---------------------------------------
// Read-only telemetry pulled through HardwareManager, mirroring how the UI
// already pulls camera frames — no device is *driven* from here (that stays in
// the workflow layer). `type` filters the fleet to "PI" or "ACS".

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
        ImGui::EndTabBar();
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
    std::vector<SavedPos>& saved   = savedPos_[devName];
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
        SavePositions();
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
                SavePositions();
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
                SavePositions();
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
                SavePositions();
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
                SavePositions();
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

// Mapped Path tab — node-edge motion maps. Nodes reference saved positions by
// uid (one node per position per map); edges are bidirectional. The canvas is
// a direct-manipulation editor: drag nodes, Edge mode links two clicked nodes,
// right-click adds/deletes. "Go" runs Dijkstra (XYZ distance weights) over the
// edges and executes the node chain through Workflow's path executor.
void App::DrawMotionMapTab(std::size_t devIdx, IMotionController* m)
{
    constexpr float  kNodeR       = 16.0f;   // canvas node radius (px)
    constexpr double kAtNodeTolMM = 0.05;    // "device is at this node" tolerance

    const std::string devName = hardware_.MotionName(devIdx);
    std::vector<MotionMap>& maps = motionMaps_[devName];

    if (mapSelDev_ != static_cast<int>(devIdx)) {   // device switched — reset state
        mapSelDev_ = static_cast<int>(devIdx);
        mapSel_ = 0;
        mapEdgeMode_ = false;
        mapEdgeFrom_.clear();
        mapDragNode_.clear();
        mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
        mapLastPath_.clear();
        mapFrom_ = 0;
        mapTo_   = -1;
    }
    if (mapSel_ >= static_cast<int>(maps.size())) mapSel_ = maps.empty() ? -1 : 0;

    const std::vector<SavedPos>& saved = savedPos_[devName];
    auto posOf = [&saved](const std::string& uid) -> const SavedPos* {
        for (const SavedPos& p : saved) {
            if (p.uid == uid) return &p;
        }
        return nullptr;
    };

    // ---- Map selector + New / Rename / Delete / Save ----
    ImGui::SetNextItemWidth(200.0f);
    const bool mapValid = mapSel_ >= 0 && mapSel_ < static_cast<int>(maps.size());
    if (ImGui::BeginCombo("##mapsel", mapValid ? maps[mapSel_].name.c_str() : "(no maps)")) {
        for (int i = 0; i < static_cast<int>(maps.size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(maps[i].name.c_str(), i == mapSel_)) {
                mapSel_ = i;
                mapEdgeFrom_.clear();
                mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
                mapLastPath_.clear();
                mapFrom_ = 0; mapTo_ = -1;
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("+ New")) { mapNameBuf_[0] = '\0'; openNewMap_ = true; }
    if (mapValid) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Rename")) {
            std::snprintf(mapNameBuf_, sizeof(mapNameBuf_), "%s", maps[mapSel_].name.c_str());
            openRenameMap_ = true;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) openDeleteMap_ = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("Save")) SaveMaps();   // edits also autosave
    }

    // ---- New / Rename / Delete map dialogs ----
    if (openNewMap_) { ImGui::OpenPopup("New Map"); openNewMap_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("New Map", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        const bool submit = ImGui::InputText("##mapname", mapNameBuf_, sizeof(mapNameBuf_),
                                             ImGuiInputTextFlags_EnterReturnsTrue);
        const bool valid = mapNameBuf_[0] != '\0';
        ImGui::BeginDisabled(!valid);
        if (ImGui::Button("Create") || (submit && valid)) {
            MotionMap mm;
            mm.name = mapNameBuf_;
            mm.uid  = GenerateUid();
            maps.push_back(std::move(mm));
            mapSel_ = static_cast<int>(maps.size()) - 1;
            mapLastPath_.clear();
            SaveMaps();
            FD_INFO("map", "created map '%s' for %s", mapNameBuf_, devName.c_str());
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (openRenameMap_) { ImGui::OpenPopup("Rename Map"); openRenameMap_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Rename Map", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (!mapValid) {
            ImGui::CloseCurrentPopup();
        } else {
            ImGui::SetNextItemWidth(280.0f);
            if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
            const bool submit = ImGui::InputText("##maprename", mapNameBuf_, sizeof(mapNameBuf_),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
            const bool valid = mapNameBuf_[0] != '\0';
            ImGui::BeginDisabled(!valid);
            if (ImGui::Button("OK") || (submit && valid)) {
                maps[mapSel_].name = mapNameBuf_;
                SaveMaps();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (openDeleteMap_) { ImGui::OpenPopup("Delete Map"); openDeleteMap_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Map", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (!mapValid) {
            ImGui::CloseCurrentPopup();
        } else {
            ImGui::Text("Delete map '%s' on %s?", maps[mapSel_].name.c_str(), devName.c_str());
            ImGui::TextDisabled("This cannot be undone.");
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
            if (ImGui::Button("Delete")) {
                FD_INFO("map", "deleted map '%s' (%s)", maps[mapSel_].name.c_str(), devName.c_str());
                maps.erase(maps.begin() + mapSel_);
                mapSel_ = maps.empty() ? -1 : 0;
                mapLastPath_.clear();
                SaveMaps();
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (!mapValid) {
        ImGui::Spacing();
        ImGui::TextDisabled("No maps for this device yet - create one with \"+ New\".");
        return;
    }
    MotionMap& map = maps[mapSel_];

    // ---- Edit toolbar ----
    if (ImGui::SmallButton("+ Add node")) ImGui::OpenPopup("mapaddnode");
    ImGui::SameLine();
    if (ImGui::Checkbox("Edge mode", &mapEdgeMode_) && !mapEdgeMode_) mapEdgeFrom_.clear();
    const bool edgeSelected = !mapSelEdgeA_.empty() && map.HasEdge(mapSelEdgeA_, mapSelEdgeB_);
    if (edgeSelected) {
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
        if (ImGui::SmallButton("Delete edge (Del)")) {
            map.RemoveEdge(mapSelEdgeA_, mapSelEdgeB_);
            mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
            mapLastPath_.clear();
            SaveMaps();
        }
        ImGui::PopStyleColor();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu node(s), %zu edge(s)  |  drag nodes; click an edge to select it; "
                        "right-click to add/delete",
                        map.nodes.size(), map.edges.size());
    if (ImGui::BeginPopup("mapaddnode")) {
        ImGui::TextDisabled("Add a saved position as a node:");
        ImGui::Separator();
        bool anyLeft = false;
        for (size_t i = 0; i < saved.size(); ++i) {
            if (map.FindNode(saved[i].uid)) continue;   // one node per position
            anyLeft = true;
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(saved[i].name)) {
                MotionMapNode n;
                n.uid = saved[i].uid;
                n.x   = 50.0f + static_cast<float>(map.nodes.size() % 5) * 120.0f;
                n.y   = 50.0f + static_cast<float>(map.nodes.size() / 5) * 90.0f;
                map.nodes.push_back(std::move(n));
                SaveMaps();
            }
            ImGui::PopID();
        }
        if (!anyLeft) ImGui::TextDisabled("(every saved position is already a node)");
        ImGui::EndPopup();
    }

    // ---- Canvas ----
    const float goH = ImGui::GetFrameHeightWithSpacing() * 2.5f;
    ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 cs(avail.x > 50.0f ? avail.x : 50.0f,
                    (avail.y - goH) > 140.0f ? (avail.y - goH) : 140.0f);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + cs.x, p0.y + cs.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, IM_COL32(20, 22, 27, 255), 4.0f);
    dl->AddRect(p0, p1, IM_COL32(70, 74, 82, 255), 4.0f);
    ImGui::InvisibleButton("mapcanvas", cs,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool  canvasHovered = ImGui::IsItemHovered();
    const ImVec2 mp = ImGui::GetIO().MousePos;
    const ImVec2 lm(mp.x - p0.x, mp.y - p0.y);   // mouse in canvas coords

    auto nodeCenter = [&p0](const MotionMapNode& n) { return ImVec2(p0.x + n.x, p0.y + n.y); };
    auto hitNode = [&]() -> std::string {
        for (auto it = map.nodes.rbegin(); it != map.nodes.rend(); ++it) {
            const float dx = lm.x - it->x, dy = lm.y - it->y;
            if (dx * dx + dy * dy <= kNodeR * kNodeR) return it->uid;
        }
        return {};
    };
    auto hitEdge = [&]() -> int {   // point-to-segment distance < 6 px
        for (size_t i = 0; i < map.edges.size(); ++i) {
            const MotionMapNode* a = map.FindNode(map.edges[i].a);
            const MotionMapNode* b = map.FindNode(map.edges[i].b);
            if (!a || !b) continue;
            const float abx = b->x - a->x, aby = b->y - a->y;
            const float len2 = abx * abx + aby * aby;
            float t = len2 > 0.0f ? ((lm.x - a->x) * abx + (lm.y - a->y) * aby) / len2 : 0.0f;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            const float dx = lm.x - (a->x + t * abx), dy = lm.y - (a->y + t * aby);
            if (dx * dx + dy * dy < 36.0f) return static_cast<int>(i);
        }
        return -1;
    };

    // Which node the device currently sits at (all saved axes within tol).
    std::string currentUid;
    if (m->IsConnected()) {
        const std::map<std::string, double> pos = m->Positions();
        for (const MotionMapNode& n : map.nodes) {
            const SavedPos* p = posOf(n.uid);
            if (!p || p->vals.empty()) continue;
            bool at = true;
            for (const auto& [axis, v] : p->vals) {
                const auto it = pos.find(axis);
                if (it == pos.end() || std::fabs(it->second - v) > kAtNodeTolMM) { at = false; break; }
            }
            if (at) { currentUid = n.uid; break; }
        }
    }

    auto edgeOnPath = [&](const std::string& a, const std::string& b) {
        for (size_t i = 0; i + 1 < mapLastPath_.size(); ++i) {
            if ((mapLastPath_[i] == a && mapLastPath_[i + 1] == b) ||
                (mapLastPath_[i] == b && mapLastPath_[i + 1] == a)) return true;
        }
        return false;
    };

    dl->PushClipRect(p0, p1, true);
    for (const MotionMapEdge& e : map.edges) {
        const MotionMapNode* a = map.FindNode(e.a);
        const MotionMapNode* b = map.FindNode(e.b);
        if (!a || !b) continue;
        const bool selected = (e.a == mapSelEdgeA_ && e.b == mapSelEdgeB_) ||
                              (e.a == mapSelEdgeB_ && e.b == mapSelEdgeA_);
        const bool onPath   = edgeOnPath(e.a, e.b);
        ImU32 col   = IM_COL32(130, 135, 145, 255);
        float width = 2.5f;
        if (onPath)   { col = IM_COL32(80, 220, 120, 255);  width = 4.0f; }
        if (selected) { col = IM_COL32(240, 160, 60, 255);  width = 4.0f; }
        dl->AddLine(nodeCenter(*a), nodeCenter(*b), col, width);
    }
    if (mapEdgeMode_ && !mapEdgeFrom_.empty()) {   // preview: picked node -> mouse
        if (const MotionMapNode* a = map.FindNode(mapEdgeFrom_))
            dl->AddLine(nodeCenter(*a), mp, IM_COL32(240, 200, 80, 180), 1.5f);
    }
    for (const MotionMapNode& n : map.nodes) {
        const SavedPos* p = posOf(n.uid);
        const ImVec2 c = nodeCenter(n);
        dl->AddCircleFilled(c, kNodeR, p ? IM_COL32(52, 120, 200, 255)
                                         : IM_COL32(150, 60, 60, 255));
        if (n.uid == currentUid)
            dl->AddCircle(c, kNodeR + 3.0f, IM_COL32(80, 220, 120, 255), 0, 3.0f);
        if (mapEdgeMode_ && n.uid == mapEdgeFrom_)
            dl->AddCircle(c, kNodeR + 3.0f, IM_COL32(240, 200, 80, 255), 0, 3.0f);
        const char*  label = p ? p->name : "(missing)";
        const ImVec2 ts    = ImGui::CalcTextSize(label);
        dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y + kNodeR + 3.0f),
                    IM_COL32(225, 228, 235, 255), label);
    }
    dl->PopClipRect();

    // ---- Canvas interactions ----
    const std::string hovered = canvasHovered ? hitNode() : std::string();
    if (canvasHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (mapEdgeMode_) {
            if (hovered.empty()) {
                mapEdgeFrom_.clear();
            } else if (mapEdgeFrom_.empty()) {
                mapEdgeFrom_ = hovered;
            } else if (hovered != mapEdgeFrom_) {
                if (!map.HasEdge(mapEdgeFrom_, hovered)) {
                    map.edges.push_back({mapEdgeFrom_, hovered});
                    SaveMaps();
                }
                mapEdgeFrom_ = hovered;   // chain: keep linking from here
            }
        } else if (!hovered.empty()) {
            mapDragNode_ = hovered;
            mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
        } else {
            const int ei = hitEdge();   // click an edge line to select it
            if (ei >= 0) {
                mapSelEdgeA_ = map.edges[ei].a;
                mapSelEdgeB_ = map.edges[ei].b;
            } else {
                mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
            }
        }
    }
    if (edgeSelected && !ImGui::GetIO().WantTextInput &&
        ImGui::IsKeyPressed(ImGuiKey_Delete)) {
        map.RemoveEdge(mapSelEdgeA_, mapSelEdgeB_);
        mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
        mapLastPath_.clear();
        SaveMaps();
    }
    if (!mapDragNode_.empty()) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            for (MotionMapNode& n : map.nodes) {
                if (n.uid != mapDragNode_) continue;
                n.x += ImGui::GetIO().MouseDelta.x;
                n.y += ImGui::GetIO().MouseDelta.y;
                n.x = n.x < kNodeR ? kNodeR : (n.x > cs.x - kNodeR ? cs.x - kNodeR : n.x);
                n.y = n.y < kNodeR ? kNodeR : (n.y > cs.y - kNodeR ? cs.y - kNodeR : n.y);
                break;
            }
        } else {
            mapDragNode_.clear();
            SaveMaps();   // persist the final spot once the drag ends
        }
    }
    if (canvasHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        if (mapEdgeMode_ && !mapEdgeFrom_.empty()) {
            mapEdgeFrom_.clear();   // cancel the pending edge pick (yellow line)
        } else {
            mapCtxNode_ = hovered;
            mapCtxEdge_ = hovered.empty() ? hitEdge() : -1;
            if (hovered.empty() && mapCtxEdge_ < 0) mapAddAt_ = lm;
            ImGui::OpenPopup("mapctx");
        }
    }
    if (ImGui::BeginPopup("mapctx")) {
        if (!mapCtxNode_.empty()) {
            const SavedPos* p = posOf(mapCtxNode_);
            ImGui::TextDisabled("node '%s'", p ? p->name : "(missing)");
            ImGui::Separator();
            if (ImGui::MenuItem("Delete node (and its edges)")) {
                map.RemoveNode(mapCtxNode_);
                mapLastPath_.clear();
                SaveMaps();
            }
        } else if (mapCtxEdge_ >= 0 && mapCtxEdge_ < static_cast<int>(map.edges.size())) {
            ImGui::TextDisabled("edge");
            ImGui::Separator();
            if (ImGui::MenuItem("Delete edge")) {
                map.edges.erase(map.edges.begin() + mapCtxEdge_);
                mapLastPath_.clear();
                SaveMaps();
            }
        } else {
            ImGui::TextDisabled("Add node here:");
            ImGui::Separator();
            bool anyLeft = false;
            for (size_t i = 0; i < saved.size(); ++i) {
                if (map.FindNode(saved[i].uid)) continue;
                anyLeft = true;
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::MenuItem(saved[i].name)) {
                    MotionMapNode n;
                    n.uid = saved[i].uid;
                    n.x   = mapAddAt_.x;
                    n.y   = mapAddAt_.y;
                    map.nodes.push_back(std::move(n));
                    SaveMaps();
                }
                ImGui::PopID();
            }
            if (!anyLeft) ImGui::TextDisabled("(every saved position is already a node)");
        }
        ImGui::EndPopup();
    }

    // ---- Move along path (node -> node via edges) ----
    struct GoNode { std::string uid; const char* label; };
    std::vector<GoNode> goNodes;
    for (const MotionMapNode& n : map.nodes) {
        if (const SavedPos* p = posOf(n.uid)) goNodes.push_back({n.uid, p->name});
    }
    if (mapTo_ >= static_cast<int>(goNodes.size()))       mapTo_   = -1;
    if (mapFrom_ > static_cast<int>(goNodes.size()))      mapFrom_ = 0;

    const Workflow::PathProgress pp = workflow_.PathStatus();
    const bool running = pp.active && pp.devIdx == devIdx;

    ImGui::TextUnformatted("Move");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("##gofrom", mapFrom_ == 0 ? "(nearest node)"
                                                    : goNodes[mapFrom_ - 1].label)) {
        if (ImGui::Selectable("(nearest node)", mapFrom_ == 0)) mapFrom_ = 0;
        for (int i = 0; i < static_cast<int>(goNodes.size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(goNodes[i].label, mapFrom_ == i + 1)) mapFrom_ = i + 1;
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("->");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("##goto", mapTo_ >= 0 ? goNodes[mapTo_].label : "(target)")) {
        for (int i = 0; i < static_cast<int>(goNodes.size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(goNodes[i].label, mapTo_ == i)) mapTo_ = i;
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(running || recipeEngine_.Running() || luaEngine_.Running() ||
                         !m->IsConnected() || goNodes.empty() || mapTo_ < 0);
    if (ImGui::Button("Go")) {
        std::string fromUid;
        if (mapFrom_ == 0) {   // nearest node to the live position
            const std::map<std::string, double> pos = m->Positions();
            double best = 0.0;
            for (const GoNode& g : goNodes) {
                const SavedPos* p = posOf(g.uid);
                double d2 = 0.0;
                for (const auto& [axis, v] : p->vals) {
                    const auto it = pos.find(axis);
                    if (it != pos.end()) d2 += (v - it->second) * (v - it->second);
                }
                if (fromUid.empty() || d2 < best) { best = d2; fromUid = g.uid; }
            }
        } else {
            fromUid = goNodes[mapFrom_ - 1].uid;
        }

        // Route planning is shared with the recipe engine's move_path command
        // (core/MotionMap::BuildMapRoute) — one planner, two executors.
        std::string routeErr;
        const std::vector<MapRouteLeg> legs = BuildMapRoute(
            map, saved, m->Axes(), fromUid, goNodes[mapTo_].uid, &routeErr);
        if (legs.empty()) {
            FD_WARN("path", "%s: %s", devName.c_str(), routeErr.c_str());
        } else {
            mapLastPath_.clear();
            std::vector<Workflow::PathWaypoint> wps;
            wps.reserve(legs.size());
            for (const MapRouteLeg& leg : legs) {
                mapLastPath_.push_back(leg.uid);
                wps.push_back({leg.label, leg.targets});
            }
            workflow_.StartPathMove(devIdx, std::move(wps));
        }
    }
    ImGui::EndDisabled();

    if (running) {
        ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "driving to '%s' (%zu/%zu)",
                           pp.label.c_str(), pp.current + 1, pp.total);
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) workflow_.CancelPathMove();
    } else if (!mapLastPath_.empty()) {
        std::string line = "last path: ";
        for (size_t i = 0; i < mapLastPath_.size(); ++i) {
            const SavedPos* p = posOf(mapLastPath_[i]);
            if (i) line += " -> ";
            line += p ? p->name : "?";
        }
        ImGui::TextDisabled("%s", line.c_str());
    }
}

void App::DrawVelocityCombo(std::size_t di)
{
    IMotionController* dev = hardware_.MotionDevice(di);
    if (!dev) return;
    const bool    acs  = dev->Type() == "ACS";
    const double* vels = acs ? kVelACS : kVelPI;
    const int     velN = acs ? 5 : 4;
    const int     key  = static_cast<int>(di);
    int& idx = velIndex_[key];
    if (idx < 0 || idx >= velN) idx = 0;

    // Deliberately NO velocity push at connect (and none at disconnect/exit):
    // the device keeps whatever velocity it already has until the operator —
    // or a recipe/script velocity override — explicitly sets one. Until then
    // the combo shows "(device)" instead of pretending to know the value.
    ImGui::TextUnformatted("Velocity (mm/s)");
    ImGui::SetNextItemWidth(-FLT_MIN);
    char prev[24];
    if (velCustomSet_[key] > 0.0)
        std::snprintf(prev, sizeof(prev), "%g (custom)", velCustomSet_[key]);
    else if (!velPushed_[key])
        std::snprintf(prev, sizeof(prev), "(device)");
    else
        std::snprintf(prev, sizeof(prev), "%g", vels[idx]);
    if (ImGui::BeginCombo("##vel", prev)) {
        for (int i = 0; i < velN; ++i) {
            char b[16]; std::snprintf(b, sizeof(b), "%g", vels[i]);
            if (ImGui::Selectable(b, velCustomSet_[key] <= 0.0 && idx == i)) {
                idx = i;
                velCustomSet_[key] = 0.0;   // back to a preset
                workflow_.SetJogVelocity(di, vels[i]);
                velPushed_[key] = true;
            }
        }
        ImGui::EndCombo();
    }

    // Custom velocity not in the preset list. Clamped to the preset ceiling for
    // this controller type (PI 10, ACS 40 mm/s) — raise kVel* to allow more.
    const double velMax = vels[velN - 1];
    double& custom = velCustomEdit_[key];
    if (custom <= 0.0) custom = vels[idx];   // seed the box on first show
    ImGui::SetNextItemWidth(-64.0f);
    ImGui::InputDouble("##velcustom", &custom, 0.0, 0.0, "%.3f");
    ImGui::SameLine();
    if (ImGui::SmallButton("Apply")) {
        if (custom < 0.001)  custom = 0.001;
        if (custom > velMax) custom = velMax;
        velCustomSet_[key] = custom;
        workflow_.SetJogVelocity(di, custom);
        velPushed_[key] = true;
        FD_INFO("motion", "%s custom velocity %.3f mm/s", dev->Name().c_str(), custom);
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

void App::DrawIoListPanel()
{
    const size_t n = hardware_.IoCount();

    // Keep the current selection if still valid; otherwise fall back to the
    // first device (same convention as the Motion list).
    if (selectedIo_ < 0 || static_cast<size_t>(selectedIo_) >= n ||
        !hardware_.IoDevice(static_cast<size_t>(selectedIo_))) {
        selectedIo_ = n > 0 ? 0 : -1;
    }

    ImGui::TextUnformatted("IO devices");
    ImGui::Separator();

    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        IIoDevice* d = hardware_.IoDevice(i);
        if (!d) continue;
        any = true;
        const ConnState st = hardware_.IoState(i);
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
        ImGui::SameLine();
        const std::string& name = hardware_.IoName(i);
        const ImVec2 sz = ImGui::CalcTextSize(name.c_str());
        if (ImGui::Selectable(name.c_str(), selectedIo_ == static_cast<int>(i),
                              ImGuiSelectableFlags_None, ImVec2(sz.x, 0.0f))) {
            selectedIo_ = static_cast<int>(i);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", d->Type().c_str());
        ImGui::SameLine();
        ImGui::TextColored(StateColor(st), "%s", StateText(st));
        ImGui::PopID();
    }
    if (!any) ImGui::TextDisabled("no IO devices configured");

    // ---- Background IO watch (input edge -> output action) ----
    IoWatch* w = hardware_.Watch();
    if (w) {
        ImGui::Spacing();
        ImGui::Separator();
        bool active = w->IsActive();
        if (ImGui::Checkbox("Watch bindings", &active)) w->SetActive(active);
        for (size_t i = 0; i < w->BindingCount(); ++i) {
            const IoWatch::Binding& b = w->BindingAt(i);
            ImGui::PushID(static_cast<int>(i));
            const bool on = w->InputOn(i);
            ImGui::TextColored(on ? StateColor(ConnState::Connected)
                                  : ImVec4(0.45f, 0.45f, 0.48f, 1.0f), "*");
            ImGui::SameLine();
            ImGui::TextUnformatted(b.name.c_str());
            const uint64_t hits = w->Hits(i);
            if (hits > 0) {
                ImGui::SameLine();
                ImGui::TextDisabled("(%llu)", static_cast<unsigned long long>(hits));
            }
            ImGui::PopID();
        }
    }
}

void App::DrawIoDetailPanel()
{
    const size_t n = hardware_.IoCount();
    IIoDevice* d = (selectedIo_ >= 0 && static_cast<size_t>(selectedIo_) < n)
                     ? hardware_.IoDevice(static_cast<size_t>(selectedIo_))
                     : nullptr;
    if (!d) {
        ImGui::TextDisabled("Select a device from the list.");
        return;
    }

    const size_t    idx = static_cast<size_t>(selectedIo_);
    const ConnState st  = hardware_.IoState(idx);

    ImGui::TextUnformatted(hardware_.IoName(idx).c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "[%s]", StateText(st));
    ImGui::Separator();
    ImGui::Text("Type : %s", d->Type().c_str());
    ImGui::Text("Pins : %d in / %d out", d->InputCount(), d->OutputCount());

    if (st == ConnState::Failed) {
        ImGui::Spacing();
        ImGui::TextColored(StateColor(ConnState::Failed), "error: %s", d->LastError().c_str());
    }

    // ---- Outputs (writes drive real actuators — gated behind arming) ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextUnformatted("Outputs");
    ImGui::SameLine();
    ImGui::Checkbox("Enable", &ioArmed_);
    ImGui::SameLine();
    if (ioArmed_) {
        ImGui::TextColored(StateColor(ConnState::Connecting), "LIVE - toggles drive hardware");
    } else {
        ImGui::TextDisabled("check Enable to activate the toggles");
    }

    const IIoDevice::IoStatus s = d->Status();
    const bool canWrite = ioArmed_ && st == ConnState::Connected;
    if (ImGui::BeginTable("iooutputs", 4,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Pin", ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("##toggle", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableHeadersRow();
        for (int pin = 0; pin < d->OutputCount(); ++pin) {
            const bool on = ((s.outputs >> pin) & 1ull) != 0;
            ImGui::TableNextRow();
            ImGui::PushID(pin);
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", pin);
            ImGui::TableSetColumnIndex(1);
            const std::string name = d->OutputPinName(pin);
            if (name.empty()) ImGui::TextDisabled("--");
            else              ImGui::TextUnformatted(name.c_str());
            ImGui::TableSetColumnIndex(2);
            if (on) ImGui::TextColored(StateColor(ConnState::Connected), "ON");
            else    ImGui::TextDisabled("OFF");
            ImGui::TableSetColumnIndex(3);
            ImGui::BeginDisabled(!canWrite);
            if (ImGui::SmallButton(on ? "Turn OFF" : "Turn ON")) {
                workflow_.SetIoOutput(idx, pin, !on);
            }
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (!s.valid && st == ConnState::Connected) {
        ImGui::TextDisabled("waiting for first poll...");
    }
}

void App::DrawIoInputPanel()
{
    // Right panel: live input pin states for the selected IO device.
    const size_t n = hardware_.IoCount();
    IIoDevice* d = (selectedIo_ >= 0 && static_cast<size_t>(selectedIo_) < n)
                     ? hardware_.IoDevice(static_cast<size_t>(selectedIo_))
                     : nullptr;

    ImGui::TextUnformatted("Inputs");
    ImGui::Separator();
    if (!d) { ImGui::TextDisabled("no selection"); return; }

    const IIoDevice::IoStatus s = d->Status();
    if (ImGui::BeginTable("ioinputs", 4,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Pin", ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("Latch", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();
        for (int pin = 0; pin < d->InputCount(); ++pin) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", pin);
            ImGui::TableSetColumnIndex(1);
            const std::string name = d->InputPinName(pin);
            if (name.empty()) ImGui::TextDisabled("--");
            else              ImGui::TextUnformatted(name.c_str());
            ImGui::TableSetColumnIndex(2);
            if ((s.inputs >> pin) & 1) ImGui::TextColored(StateColor(ConnState::Connected), "ON");
            else                       ImGui::TextDisabled("OFF");
            ImGui::TableSetColumnIndex(3);
            if ((s.latch >> pin) & 1) ImGui::TextColored(StateColor(ConnState::Connecting), "SET");
            else                      ImGui::TextDisabled("--");
        }
        ImGui::EndTable();
    }
    if (!s.valid) ImGui::TextDisabled("waiting for first poll...");

    ImGui::Spacing();
    if (ImGui::SmallButton("Clear latch")) {
        workflow_.ClearIoLatch(static_cast<size_t>(selectedIo_), ~0ull);
    }
}

// Position -> display color for the Pneumatics page.
static ImVec4 SlideColor(SlidePosition p)
{
    switch (p) {
        case SlidePosition::Extended:      return ImVec4(0.40f, 0.90f, 0.50f, 1.0f);
        case SlidePosition::Retracted:     return ImVec4(0.85f, 0.88f, 0.90f, 1.0f);
        case SlidePosition::Extending:
        case SlidePosition::Retracting:    return ImVec4(0.95f, 0.80f, 0.30f, 1.0f);
        case SlidePosition::HardwareError:
        case SlidePosition::Timeout:       return ImVec4(0.95f, 0.40f, 0.40f, 1.0f);
        default:                           return ImVec4(0.60f, 0.60f, 0.60f, 1.0f);
    }
}

void App::DrawPneumaticListPanel()
{
    const size_t n = hardware_.SlideCount();
    if (selectedSlide_ < 0 || static_cast<size_t>(selectedSlide_) >= n) {
        selectedSlide_ = n > 0 ? 0 : -1;
    }

    ImGui::TextUnformatted("Pneumatic slides");
    ImGui::Separator();

    for (size_t i = 0; i < n; ++i) {
        PneumaticSlide* s = hardware_.Slide(i);
        const SlidePosition p = s->Position();
        ImGui::PushID(static_cast<int>(i));
        const ImVec2 sz = ImGui::CalcTextSize(s->Name().c_str());
        if (ImGui::Selectable(s->Name().c_str(), selectedSlide_ == static_cast<int>(i),
                              ImGuiSelectableFlags_None, ImVec2(sz.x, 0.0f))) {
            selectedSlide_ = static_cast<int>(i);
        }
        ImGui::SameLine();
        ImGui::TextColored(SlideColor(p), "%s", SlidePositionName(p));
        ImGui::PopID();
    }
    if (n == 0) ImGui::TextDisabled("no pneumatic slides configured");
}

void App::DrawPneumaticDetailPanel()
{
    const size_t n = hardware_.SlideCount();
    PneumaticSlide* s = (selectedSlide_ >= 0 && static_cast<size_t>(selectedSlide_) < n)
                          ? hardware_.Slide(static_cast<size_t>(selectedSlide_))
                          : nullptr;
    if (!s) {
        ImGui::TextDisabled("Select a slide from the list.");
        return;
    }

    const PneumaticSlide::Status st = s->GetStatus();
    const bool ioUp = s->IsIoConnected();

    ImGui::TextUnformatted(s->Name().c_str());
    ImGui::SameLine();
    ImGui::TextColored(SlideColor(st.position), "[%s]", SlidePositionName(st.position));
    ImGui::Separator();
    ImGui::Text("IO      : %s", ioUp ? "connected" : "not connected");
    ImGui::Text("Timeout : %u ms", s->TimeoutMs());
    if (st.moving) ImGui::Text("Moving  : %u ms elapsed", st.elapsedMs);
    else if (st.elapsedMs > 0) ImGui::Text("Last move : %u ms", st.elapsedMs);

    if (st.position == SlidePosition::Timeout || st.position == SlidePosition::HardwareError) {
        ImGui::Spacing();
        ImGui::TextColored(SlideColor(st.position), "error: %s",
                           st.position == SlidePosition::HardwareError
                               ? "both sensors active" : s->LastError().c_str());
    }

    // ---- Commands (drive real actuators — gated behind arming) ----
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextUnformatted("Control");
    ImGui::SameLine();
    ImGui::Checkbox("Enable", &pneuArmed_);
    ImGui::SameLine();
    if (pneuArmed_) {
        ImGui::TextColored(StateColor(ConnState::Connecting), "LIVE - buttons drive hardware");
    } else {
        ImGui::TextDisabled("check Enable to activate the buttons");
    }

    const size_t idx = static_cast<size_t>(selectedSlide_);
    ImGui::BeginDisabled(!pneuArmed_ || !ioUp);
    if (ImGui::Button("Extend"))  workflow_.ExtendSlide(idx);
    ImGui::SameLine();
    if (ImGui::Button("Retract")) workflow_.RetractSlide(idx);
    ImGui::SameLine();
    if (ImGui::Button("Stop"))    workflow_.StopSlide(idx);
    ImGui::EndDisabled();
}

void App::DrawPneumaticInfoPanel()
{
    // Right panel: the live output/sensor pin states for the selected slide.
    const size_t n = hardware_.SlideCount();
    PneumaticSlide* s = (selectedSlide_ >= 0 && static_cast<size_t>(selectedSlide_) < n)
                          ? hardware_.Slide(static_cast<size_t>(selectedSlide_))
                          : nullptr;

    ImGui::TextUnformatted("Slide IO");
    ImGui::Separator();
    if (!s) { ImGui::TextDisabled("no selection"); return; }

    const PneumaticSlide::Status st = s->GetStatus();
    if (ImGui::BeginTable("slideio", 2,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Signal");
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();
        auto row = [](const char* label, bool on) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            if (on) ImGui::TextColored(StateColor(ConnState::Connected), "ON");
            else    ImGui::TextDisabled("OFF");
        };
        row("Output (extend)",  st.outputOn);
        row("Extended sensor",  st.extendedOn);
        row("Retracted sensor", st.retractedOn);
        ImGui::EndTable();
    }
    if (!st.ioValid) ImGui::TextDisabled("waiting for first poll...");
}

void App::DrawSafetyAlertModal()
{
    // First-priority safety alert: opened whenever a watched signal latched an
    // alert; motion is already stopped (Workflow's callback) and — for
    // critical alerts — blocked until acknowledged here.
    if (workflow_.SafetyAlertActive() && !ImGui::IsPopupOpen("Safety Alert")) {
        ImGui::OpenPopup("Safety Alert");
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Safety Alert", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const SafetyWatch::Alert a = workflow_.SafetyAlert();
        ImGui::TextColored(StateColor(ConnState::Failed), "%s", a.title.c_str());
        ImGui::Separator();
        ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 420.0f);
        ImGui::TextUnformatted(a.message.c_str());
        ImGui::PopTextWrapPos();
        if (a.critical) {
            ImGui::Spacing();
            ImGui::TextColored(StateColor(ConnState::Failed),
                               "All motion stopped. New moves are blocked until acknowledged.");
        }
        ImGui::Spacing();
        if (ImGui::Button("Acknowledge", ImVec2(160.0f, 0.0f))) {
            workflow_.AcknowledgeSafety();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// IO Watch editor — full-page main-window content (header/Close drawn by
// DrawMainWindow's editor-page branch).
void App::DrawWatchEditPanel()
{
    // Master settings + actions. Apply rebuilds the running watch from the
    // working copy; Save also persists it into config/hardware.json.
    ImGui::Checkbox("Enabled", &watchCfg_.enabled);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    ImGui::InputInt("Poll ms", &watchCfg_.pollingIntervalMs);
    ImGui::SameLine();
    if (ImGui::Button("+ Add binding")) watchCfg_.bindings.push_back({});
    ImGui::SameLine();
    if (ImGui::Button("Apply")) workflow_.ApplyIoWatch(watchCfg_);
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        workflow_.ApplyIoWatch(watchCfg_);
        if (HardwareConfig::SaveIoWatch("config/hardware.json", watchCfg_))
            FD_INFO("app", "io_watch saved to config/hardware.json");
        else
            FD_ERROR("app", "failed to save io_watch config");
    }
    ImGui::TextDisabled("Save writes config/hardware.json next to the exe; a rebuild overwrites it "
                        "(mirror durable changes into the repo config).");

    // Combo helpers: devices from the IO fleet; pins limited to NAMED pins
    // (bindings reference pins by name).
    auto ioDeviceByName = [this](const std::string& name) -> IIoDevice* {
        for (size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == name) return hardware_.IoDevice(i);
        return nullptr;
    };
    auto deviceCombo = [this](const char* id, std::string& dev) {
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::BeginCombo(id, dev.empty() ? "(device)" : dev.c_str())) {
            for (size_t i = 0; i < hardware_.IoCount(); ++i) {
                const std::string& n = hardware_.IoName(i);
                if (ImGui::Selectable(n.c_str(), n == dev)) dev = n;
            }
            ImGui::EndCombo();
        }
    };
    auto pinCombo = [&](const char* id, const std::string& dev, std::string& pin, bool output) {
        IIoDevice* d = ioDeviceByName(dev);
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo(id, pin.empty() ? "(pin)" : pin.c_str())) {
            if (d) {
                const int count = output ? d->OutputCount() : d->InputCount();
                for (int p = 0; p < count; ++p) {
                    const std::string n = output ? d->OutputPinName(p) : d->InputPinName(p);
                    if (n.empty()) continue;
                    if (ImGui::Selectable(n.c_str(), n == pin)) pin = n;
                }
            }
            ImGui::EndCombo();
        }
    };

    static const char* kEdges[]     = {"rising", "falling", "both"};
    static const char* kBehaviors[] = {"toggle", "set_on", "set_off", "momentary"};
    auto stringCombo = [](const char* id, std::string& val, const char* const* items,
                          int count, float width) {
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo(id, val.c_str())) {
            for (int i = 0; i < count; ++i) {
                if (ImGui::Selectable(items[i], val == items[i])) val = items[i];
            }
            ImGui::EndCombo();
        }
    };

    int deleteRow = -1;
    for (int i = 0; i < static_cast<int>(watchCfg_.bindings.size()); ++i) {
        IoWatchBindingConfig& b = watchCfg_.bindings[i];
        ImGui::PushID(i);
        ImGui::Separator();

        ImGui::Checkbox("##en", &b.enabled);
        ImGui::SameLine();
        char nameBuf[64];
        std::snprintf(nameBuf, sizeof(nameBuf), "%s", b.name.c_str());
        ImGui::SetNextItemWidth(260.0f);
        if (ImGui::InputText("##name", nameBuf, sizeof(nameBuf))) b.name = nameBuf;
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) deleteRow = i;

        ImGui::TextDisabled("in ");
        ImGui::SameLine();
        deviceCombo("##indev", b.inputDevice);
        ImGui::SameLine();
        pinCombo("##inpin", b.inputDevice, b.inputPin, false);
        ImGui::SameLine();
        stringCombo("##edge", b.trigger, kEdges, 3, 90.0f);
        ImGui::SameLine();
        ImGui::TextDisabled("-> out");
        ImGui::SameLine();
        deviceCombo("##outdev", b.outputDevice);
        ImGui::SameLine();
        pinCombo("##outpin", b.outputDevice, b.outputPin, true);
        ImGui::SameLine();
        stringCombo("##beh", b.behavior, kBehaviors, 4, 110.0f);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::InputInt("ms", &b.debounceMs);

        ImGui::PopID();
    }
    if (deleteRow >= 0) {
        watchCfg_.bindings.erase(watchCfg_.bindings.begin() + deleteRow);
    }
    if (watchCfg_.bindings.empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("no bindings — use \"+ Add binding\"");
    }
}

// Transforms editor — full-page main-window content (header/Close drawn by
// DrawMainWindow's editor-page branch). Rows = the 6 GLOBAL axes (X/Y/Z
// translation, U/V/W = theta x/y/z rotation), one column per motion device —
// same shape as the machine's global-axis table. Each cell picks the signed
// LOCAL axis that global axis maps to (one signed-permutation column of
// AxisTransform's m or r block). Every edit hot-applies to Workflow so the jog
// panel maps through it on the next press; Save writes config/motion_transforms.json.
void App::DrawTransformEditPanel()
{
    const size_t n = hardware_.MotionCount();
    if (n == 0) {
        ImGui::TextDisabled("no motion devices");
        return;
    }

    bool changed = false;

    if (ImGui::Button("Save")) {
        if (SaveMotionTransforms("config/motion_transforms.json", transformCfg_))
            FD_INFO("app", "transforms saved to config/motion_transforms.json");
        else
            FD_ERROR("app", "failed to save motion transforms");
    }
    ImGui::SameLine();
    if (ImGui::Button("Reload file")) {
        transformCfg_ = LoadMotionTransforms("config/motion_transforms.json");
        changed = true;
    }
    ImGui::TextDisabled("Edits apply to jog immediately (arm the Jog window to test). Save writes "
                        "config/motion_transforms.json next to the exe; a rebuild overwrites it "
                        "(mirror durable changes into the repo config).");

    // One combo per (global axis, device): the signed local axis this global
    // axis maps to. Writes one column of the selected block (m or r).
    auto cell = [&](AxisTransform& t, bool rot, int col, IMotionController* dev) {
        double (*blk)[3] = rot ? t.r : t.m;
        const std::vector<std::string> axes = dev->Axes();
        std::vector<std::string> names;   // local axis labels for this block
        if (rot) { for (size_t a = 3; a < axes.size() && a < 6; ++a) names.push_back(axes[a]); }
        else     { for (size_t a = 0; a < axes.size() && a < 3; ++a) names.push_back(axes[a]); }
        if (names.empty()) { ImGui::TextDisabled("n/a"); return; }

        // Current selection = dominant entry of this column.
        int ax = 0;
        for (int i = 1; i < 3; ++i)
            if (std::fabs(blk[i][col]) > std::fabs(blk[ax][col])) ax = i;
        const bool mapped = std::fabs(blk[ax][col]) > 1e-9 && ax < static_cast<int>(names.size());
        const bool pos    = blk[ax][col] >= 0.0;

        char cur[16];
        if (mapped) std::snprintf(cur, sizeof(cur), "%s %s", names[ax].c_str(), pos ? "+" : "-");
        else        std::snprintf(cur, sizeof(cur), "-");

        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::BeginCombo("##cell", cur)) {
            auto pick = [&](int i, double sign) {
                for (int r = 0; r < 3; ++r) blk[r][col] = 0.0;
                if (i >= 0) blk[i][col] = sign;
                changed = true;
            };
            for (int i = 0; i < static_cast<int>(names.size()); ++i) {
                char lbl[16];
                std::snprintf(lbl, sizeof(lbl), "%s +", names[i].c_str());
                if (ImGui::Selectable(lbl, mapped && ax == i && pos)) pick(i, +1.0);
                std::snprintf(lbl, sizeof(lbl), "%s -", names[i].c_str());
                if (ImGui::Selectable(lbl, mapped && ax == i && !pos)) pick(i, -1.0);
            }
            if (ImGui::Selectable("- (none)", !mapped)) pick(-1, 0.0);
            ImGui::EndCombo();
        }
    };

    static const char* kRows[6] = {"X", "Y", "Z", "U (\xce\xb8x)", "V (\xce\xb8y)", "W (\xce\xb8z)"};

    if (ImGui::BeginTable("xform", 1 + static_cast<int>(n),
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Global axis");
        for (size_t d = 0; d < n; ++d)
            ImGui::TableSetupColumn(hardware_.MotionName(d).c_str());
        ImGui::TableHeadersRow();

        for (int row = 0; row < 6; ++row) {
            const bool rot = row >= 3;
            const int  col = rot ? row - 3 : row;   // column of the m / r block
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(kRows[row]);
            for (size_t d = 0; d < n; ++d) {
                ImGui::TableSetColumnIndex(1 + static_cast<int>(d));
                IMotionController* dev = hardware_.MotionDevice(d);
                if (!dev) { ImGui::TextDisabled("-"); continue; }
                ImGui::PushID(row * 64 + static_cast<int>(d));
                AxisTransform& t = transformCfg_[dev->Name()];   // identity if new
                cell(t, rot, col, dev);
                ImGui::PopID();
            }
        }
        ImGui::EndTable();
    }

    if (changed) workflow_.SetTransforms(transformCfg_);   // hot reload
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
        UpdateCameraTexture();   // pull the newest camera frame into the GL texture

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
            if (chartCh_ != 1) LegendRow(kCh1Col, "Keithley 6482 Ch1");
            if (chartCh_ != 0) LegendRow(kCh2Col, "Keithley 6482 Ch2");

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

void App::DrawSignalOverlay()
{
    DrawKeithleyChart(ImVec2(-1, -1));   // fill the overlay child
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
