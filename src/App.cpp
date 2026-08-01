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

// mode: 0 = whole program, 1 = selected slot, 2 = selected recipe.
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
                                session_.Positions(), session_.Maps(), session_.Recipes(), false);
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
        for (const Recipe& rr : session_.Recipes()) {
            if (rr.uid == uid) { target = &rr; break; }
        }
        if (ImGui::BeginCombo("recipe", target ? target->name.c_str() : "(pick)")) {
            for (const Recipe& rr : session_.Recipes()) {
                // Obvious self-reference filtered; deeper cycles are caught by
                // InlineSubRecipes at run start.
                if (recipeSel_ >= 0 && rr.uid == session_.Recipes()[recipeSel_].uid) continue;
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
    } else if (const CommandSpec* spec = orch_.registry.Find(s.type)) {
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

    // Device-scoped reference params (position uid / map / pin) go stale when
    // the operator hot-swaps the step's device: the stored value no longer
    // resolves for the new device, so the combo shows a bare uid (or a name
    // the device doesn't have). Render that loud — red text on a yellow frame,
    // plus a hint line — until the operator re-picks. Push before BeginCombo,
    // pop right after it so the dropdown's own items render normally.
    constexpr ImVec4 kStaleRed{0.85f, 0.08f, 0.08f, 1.0f};
    const auto pushStale = [&kStaleRed] {
        ImGui::PushStyleColor(ImGuiCol_Text,           kStaleRed);
        ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0.95f, 0.80f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(1.00f, 0.88f, 0.30f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(1.00f, 0.92f, 0.45f, 1.0f));
    };

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
    case ParamType::DeviceHexapod:
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
            if (p.type == ParamType::DeviceMotion ||
                p.type == ParamType::DeviceHexapod) {
                for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
                    if (p.type == ParamType::DeviceHexapod &&
                        hardware_.MotionDevice(i)->Type() != "PI")
                        continue;   // scan commands are PI-only
                    item(hardware_.MotionName(i));
                }
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
        bool resolved         = v.empty();   // empty = nothing to warn about yet
        const auto it = session_.Positions().find(dev);
        if (it != session_.Positions().end()) {
            for (const SavedPos& sp : it->second)
                if (sp.uid == v) { preview = sp.name; resolved = true; break; }
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (it == session_.Positions().end() || it->second.empty())
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
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a position of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::MapName: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const auto it = session_.Maps().find(dev);
        bool resolved = v.empty();
        if (!resolved && it != session_.Maps().end())
            for (const MotionMap& m : it->second)
                if (m.name == v) { resolved = true; break; }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (it == session_.Maps().end() || it->second.empty())
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
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a map of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::PinName: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const IIoDevice* d = nullptr;
        for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == dev) { d = hardware_.IoDevice(i); break; }
        bool resolved = v.empty();
        if (!resolved && d) {
            const auto has = [&](bool inputs) {
                const int n = inputs ? d->InputCount() : d->OutputCount();
                for (int pin = 0; pin < n; ++pin)
                    if ((inputs ? d->InputPinName(pin) : d->OutputPinName(pin)) == v)
                        return true;
                return false;
            };
            resolved = (p.pinDir != PinDir::Output && has(true)) ||
                       (p.pinDir != PinDir::Input && has(false));
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
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
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a pin of '%s' - re-pick", dev.c_str());
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
        for (const ScriptInfo& si : session_.Scripts()) {
            if (si.path == v) { preview = si.name; break; }
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, preview.c_str())) {
            if (session_.Scripts().empty())
                ImGui::TextDisabled("(no scripts in %s)", session_.ScriptsDirUsed().c_str());
            for (const ScriptInfo& si : session_.Scripts()) {
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
    std::vector<MotionMap>& maps = session_.Maps()[devName];

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

    const std::vector<SavedPos>& saved = session_.Positions()[devName];
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
        if (ImGui::SmallButton("Save")) session_.SaveMaps();   // edits also autosave
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
            session_.SaveMaps();
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
                session_.SaveMaps();
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
                session_.SaveMaps();
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
            session_.SaveMaps();
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
                session_.SaveMaps();
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
                    session_.SaveMaps();
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
        session_.SaveMaps();
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
            session_.SaveMaps();   // persist the final spot once the drag ends
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
                session_.SaveMaps();
            }
        } else if (mapCtxEdge_ >= 0 && mapCtxEdge_ < static_cast<int>(map.edges.size())) {
            ImGui::TextDisabled("edge");
            ImGui::Separator();
            if (ImGui::MenuItem("Delete edge")) {
                map.edges.erase(map.edges.begin() + mapCtxEdge_);
                mapLastPath_.clear();
                session_.SaveMaps();
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
                    session_.SaveMaps();
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
    ImGui::BeginDisabled(running || orch_.Busy() ||
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
