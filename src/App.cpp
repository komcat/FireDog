// App.cpp : layer 3 (UI) implementation.

#include "App.h"

#include "imgui.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "ui/Widgets.h"
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
    session_.LoadInitial(lastProduct);   // positions/maps/recipes/programs/scripts + DUT wiring
    ResetDocumentSelections();           // the UI side of that load
    LoadScanPresets();             // XCAN launch presets (machine-global local.json)
    hardware_.ConnectAllAsync();   // simulated instruments (motion/etc.)
    workflow_.Start(hw);           // layer 2 brings camera + Keithley online
    RegisterBuiltinCommands(orch_.registry, workflow_, hardware_);   // layer 2.5
    FD_INFO("command", "%zu command(s) registered", orch_.registry.Count());

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
    orch_.programs.Tick();       // sequences program/slot runs between frames
    UpdateKeithleyChartData(dt); // shared chart buffers (Aux + Keithley page)

    // Teach hand-backs from save_position steps: engine worker threads queue
    // them on Workflow, and THIS is the worker->UI hand-off point — the
    // session is UI-thread-only, so the drain must stay here in Update().
    session_.MergeTaughtPositions(workflow_.TakeSavedPositions());
}

namespace {
// Full-page takeovers (View menu) vs the Hardware pages.
bool IsTakeoverPage(Page p)
{
    return p == Page::WatchEdit || p == Page::TransformEdit ||
           p == Page::RecipeEdit || p == Page::Scripts ||
           p == Page::DutView || p == Page::Run;
}
} // namespace

// ALL page switches route here. Leaving the Recipe Editor with unsaved edits
// blocks: the editor stays up and its Unsaved Changes modal (Save / Discard /
// Cancel) completes the switch to edLeaveTarget_. Entering a takeover page
// from a hardware page remembers it so the editor's Close returns there.
void App::SwitchPage(Page p)
{
    if (currentPage_ == p) return;
    if (currentPage_ == Page::RecipeEdit && session_.RecipesDirty()) {
        edConfirmAction_ = 1;
        edLeaveTarget_   = p;
        return;
    }
    if (IsTakeoverPage(p) && !IsTakeoverPage(currentPage_))
        prevPage_ = currentPage_;
    currentPage_ = p;
}

// Selecting the active editor page again toggles back to the remembered page.
void App::ToggleEditorPage(Page p)
{
    SwitchPage(currentPage_ == p ? prevPage_ : p);
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
                if (ImGui::MenuItem("Save Product", nullptr, false, session_.ProductLoaded()))
                    session_.SaveProduct();
                ImGui::Separator();
                if (session_.ProductLoaded())
                    ImGui::MenuItem(("Product: " + session_.Product().name).c_str(), nullptr, false, false);
                else
                    ImGui::MenuItem("Product: (none)", nullptr, false, false);
                ImGui::Separator();
                // Routes through the same confirm modal as the window X.
                if (ImGui::MenuItem("Exit", "Alt+F4")) host_.SetShouldClose(true);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Hardware")) {
                if (ImGui::MenuItem("Overview", nullptr, currentPage_ == Page::Overview))
                    SwitchPage(Page::Overview);
                if (ImGui::MenuItem("Camera",   nullptr, currentPage_ == Page::Camera))
                    SwitchPage(Page::Camera);
                if (ImGui::MenuItem("Keithley", nullptr, currentPage_ == Page::Keithley))
                    SwitchPage(Page::Keithley);
                if (ImGui::MenuItem("CLD101x",  nullptr, currentPage_ == Page::CLD))
                    SwitchPage(Page::CLD);
                if (ImGui::MenuItem("PSU",      nullptr, currentPage_ == Page::PSU))
                    SwitchPage(Page::PSU);
                if (ImGui::MenuItem("Motion",   "F3", currentPage_ == Page::Motion))
                    SwitchPage(Page::Motion);
                if (ImGui::MenuItem("IO",       nullptr, currentPage_ == Page::IO))
                    SwitchPage(Page::IO);
                if (ImGui::MenuItem("Pneumatics", nullptr, currentPage_ == Page::Pneumatic))
                    SwitchPage(Page::Pneumatic);
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
                // Operator pages first (Run + Recipe Editor, with F-key
                // shortcuts), then the machine-setup editors.
                if (ImGui::MenuItem("Run", "F1", currentPage_ == Page::Run))
                    ToggleEditorPage(Page::Run);
                if (ImGui::MenuItem("Recipe Editor", "F2", currentPage_ == Page::RecipeEdit))
                    ToggleEditorPage(Page::RecipeEdit);
                ImGui::Separator();
                if (ImGui::MenuItem("IO Watch Editor", nullptr, currentPage_ == Page::WatchEdit))
                    ToggleEditorPage(Page::WatchEdit);
                if (ImGui::MenuItem("Transforms Editor", nullptr, currentPage_ == Page::TransformEdit))
                    ToggleEditorPage(Page::TransformEdit);
                if (ImGui::MenuItem("Scripts", nullptr, currentPage_ == Page::Scripts))
                    ToggleEditorPage(Page::Scripts);
                if (ImGui::MenuItem("DUT Records", nullptr, currentPage_ == Page::DutView)) {
                    if (currentPage_ != Page::DutView) LoadDutList();
                    ToggleEditorPage(Page::DutView);
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

        // Global page shortcuts: F1 Run, F2 Recipe Editor, F3 Motion. Activate
        // (not toggle) — pressing the key on its own page is a no-op.
        if (ImGui::IsKeyPressed(ImGuiKey_F1, false) && currentPage_ != Page::Run)
            ToggleEditorPage(Page::Run);
        if (ImGui::IsKeyPressed(ImGuiKey_F2, false) && currentPage_ != Page::RecipeEdit)
            ToggleEditorPage(Page::RecipeEdit);
        if (ImGui::IsKeyPressed(ImGuiKey_F3, false))
            SwitchPage(Page::Motion);

        // Quick-access strip (all pages): one-click jump to the operator pages.
        // Same guarded routing as the F-keys (activate, not toggle — clicking
        // the current page's button is a no-op); active page highlighted.
        {
            const auto pageBtn = [&](const char* label, Page p, bool takeover) {
                const bool active = (currentPage_ == p);
                if (active)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
                if (ImGui::Button(label) && !active) {
                    if (takeover) ToggleEditorPage(p);
                    else          SwitchPage(p);
                }
                if (active) ImGui::PopStyleColor();
            };
            pageBtn("Run (F1)", Page::Run, true);
            ImGui::SameLine();
            pageBtn("Recipe Editor (F2)", Page::RecipeEdit, true);
            ImGui::SameLine();
            pageBtn("Motion (F3)", Page::Motion, false);
            ImGui::Separator();
        }

        DrawProductPopups();   // modals opened via the File-menu flags above
        DrawExitConfirmModal();   // confirm before closing the application
        DrawSafetyAlertModal();   // blocking safety alert (highest priority)
        DrawRecipePromptWindow(); // global recipe prompt (any page, any time)
        DrawManualAdjustWindow(); // global manual_adjust jog window

        // ---- Editor pages (View menu) replace the whole panel layout until
        // closed via the top-right Close button (back to the previous page).
        if (currentPage_ == Page::WatchEdit || currentPage_ == Page::TransformEdit ||
            currentPage_ == Page::RecipeEdit || currentPage_ == Page::Scripts ||
            currentPage_ == Page::DutView || currentPage_ == Page::Run) {
            const char* title = currentPage_ == Page::WatchEdit       ? "IO Watch Editor"
                                : currentPage_ == Page::TransformEdit ? "Transforms Editor"
                                : currentPage_ == Page::RecipeEdit    ? "Recipe Editor"
                                : currentPage_ == Page::Scripts       ? "Scripts"
                                : currentPage_ == Page::DutView       ? "DUT Records"
                                                                      : "Run";
            ImGui::BeginChild("EditorPage", ImGui::GetContentRegionAvail(),
                              ImGuiChildFlags_Borders);
            ImGui::TextUnformatted(title);
            ImGui::SameLine();
            const float closeW = ImGui::CalcTextSize("Close").x +
                                 ImGui::GetStyle().FramePadding.x * 2.0f;
            ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - closeW);
            if (ImGui::Button("Close"))
                SwitchPage(prevPage_);   // dirty recipe edits raise the guard
            ImGui::Separator();
            if (currentPage_ == Page::WatchEdit)          DrawWatchEditPanel();
            else if (currentPage_ == Page::TransformEdit) DrawTransformEditPanel();
            else if (currentPage_ == Page::RecipeEdit)    DrawRecipeEditPanel();
            else if (currentPage_ == Page::Scripts)       DrawScriptsPanel();
            else if (currentPage_ == Page::DutView)       DrawDutViewPanel();
            else                                          DrawRunPanel();
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

// ---- DUT Records viewer (View menu, full page, read-only) — Phase D2 -------
// Browses products/<name>/data/: serial list on the left, the record's entry
// log in the middle, header + run history on the right. Image entries show
// their linked path (in-app image preview is future work).

// Active-DUT status + open/close controls (Overview left panel). The operator
// path: type/scan a serial, Open (load-or-create). Recipes/scripts drive the
// same DutSession through the dut_* commands. Design: docs/dut-design.md.
void App::DrawDutBlock()
{
    const DutSession::Info di = DutSession::Instance().GetInfo();
    ImGui::TextUnformatted("DUT");
    if (di.productSet) {
        // Top-right shortcut: the open DUT's record folder, else data/ root.
        const float bw = ImGui::CalcTextSize("Data").x +
                         ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SameLine(ImGui::GetContentRegionAvail().x +
                        ImGui::GetCursorPosX() - bw);
        if (ImGui::SmallButton("Data"))
            OpenFolderInExplorer(DutSession::Instance().BrowseDir());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(di.open ? "Open this DUT's folder in Explorer"
                                      : "Open the product data folder in Explorer");
    }
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
    s.lastProduct = session_.Product().name;
    s.Save(kUiSettingsPath);
}

// Re-point every UI selection at a freshly (re)loaded session document. The
// session never calls back into the UI, so each load wrapper does this here.
void App::ResetDocumentSelections()
{
    recipeSel_ = -1;
    recipeStartErr_.clear();
    progSel_    = session_.Programs().empty() ? -1 : 0;
    deckSlot_   = -1;
    deckRecipe_ = -1;
    runStartErr_.clear();
    SyncScriptSelection();
}

// Scripts page selection, after the session rescanned the scripts dir: keep a
// still-valid index, otherwise fall back to the first script (or none).
void App::SyncScriptSelection()
{
    if (scriptSel_ >= static_cast<int>(session_.Scripts().size()))
        scriptSel_ = session_.Scripts().empty() ? -1 : 0;
    scriptSrcPath_.clear();   // force a source re-read on next draw
    scriptSrc_.clear();
    scriptStartErr_.clear();
    scriptBpAnchor_ = -1;
}

bool App::LoadProduct(const std::string& name)
{
    if (!session_.LoadProduct(name)) return false;
    ResetDocumentSelections();
    SaveUiSettings();   // remember as last product for the next startup
    return true;
}

// Thin wrapper: the session owns the data + the file-precedence rules, the UI
// selection reset stays here (no session -> UI callbacks).
void App::LoadRecipesFromDisk()
{
    session_.LoadRecipesFromDisk();
    recipeSel_ = -1;
    recipeStartErr_.clear();
}

// Thin wrapper (like LoadRecipesFromDisk): session loads, App resets the deck
// selection — a freshly loaded set selects its first program.
void App::LoadProgramsFromDisk()
{
    session_.LoadProgramsFromDisk();
    progSel_    = session_.Programs().empty() ? -1 : 0;
    deckSlot_   = -1;
    deckRecipe_ = -1;
    runStartErr_.clear();
}

// ---- Recipe Editor (View menu, full page) — Phase B1: browse + run ----------
// The step tree and inspector are read-only; editing arrives in B2,
// persistence/validation in B3. See docs/recipe-editor-design.md.

// ---- Programs page (View menu, full page) — Phase C1: list + run .lua -------
// Authoring happens in an external editor; this page lists, shows read-only
// source (executing line highlighted), and runs/stops programs.

void App::NewProduct(const char* name)
{
    session_.NewProduct(name);
    SaveUiSettings();   // remember as last product for the next startup
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
                    const bool current = (p == session_.Product().name);
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

// ---- Camera page ----------------------------------------------------------
// Left: status + connect/disconnect. Middle: exposure + pixel->mm calibration
// + live preview. Right: identity/stream info. Commands go through Workflow;
// reads come from the HardwareManager accessors — same rule as every page.

// ---- Keithley page --------------------------------------------------------
// Left: status + connect/disconnect. Middle: bias output + live-adjustable
// measurement settings + the shared chart. Right: identity + current settings
// + live values. Commands go through Workflow; reads come from the driver's
// cheap state accessors (GetConfig/BiasVolts never touch VISA).

// ---- CLD101x page ---------------------------------------------------------
// Left: status + connect/disconnect. Middle: TEC (temperature setpoint +
// on/off + live reading) and Laser (current setpoint + on/off + live reading).
// Right: identity + live values. Turning the laser on is always an explicit
// button press — nothing is enabled automatically at connect.

// ---- PSU page (UNI-T UDP3305S) --------------------------------------------
// Left: status + connect/disconnect. Middle: per-channel V/I setpoints (the
// verified APPLy form) + output ON/OFF (read-back verified). Right: identity +
// live values. Turning an output on is always an explicit button press —
// nothing is enabled automatically at connect; disconnect forces all OFF.

// ---- PI / ACS pages (motion fleet) ---------------------------------------
// Read-only telemetry pulled through HardwareManager, mirroring how the UI
// already pulls camera frames — no device is *driven* from here (that stays in
// the workflow layer). `type` filters the fleet to "PI" or "ACS".

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

// Confirm-before-close: the OS close request (X button / Alt+F4) and File >
// Exit both set the GLFW should-close flag; it is swallowed here and turned
// into a modal so an accidental close can't drop unsaved edits or abandon a
// live run. Exit re-arms the flag with exitConfirmed_ set, ending the loop.
void App::DrawExitConfirmModal()
{
    if (host_.ShouldClose() && !exitConfirmed_) {
        host_.SetShouldClose(false);
        if (!ImGui::IsPopupOpen("Confirm Exit")) ImGui::OpenPopup("Confirm Exit");
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Confirm Exit", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Close the application?");
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "Unsaved changes may be lost.");
        // Point out what is actually at risk right now.
        if (session_.RecipesDirty())
            ImGui::BulletText("Recipe editor has unsaved edits");
        if (session_.ProgramsDirty())
            ImGui::BulletText("Program deck has unsaved edits");
        if (orch_.Busy())
            ImGui::BulletText("A recipe/script/path move is still running");
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
        if (ImGui::Button("Exit", ImVec2(120.0f, 0.0f))) {
            exitConfirmed_ = true;
            host_.SetShouldClose(true);
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
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
