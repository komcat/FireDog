// App.cpp : layer 3 (UI) — the composition root.
//
// What lives here: the startup sequence and the frame loop (Run), the per-frame
// tick (Update), page switching, and the UI-side wrappers around
// ProductSession. Everything that DRAWS lives in ui/pages/ and ui/windows/,
// as out-of-line definitions of the same App members declared in App.h.
//
// Layer rule, unchanged by the split: the UI owns no device and runs no
// connect. It reads HardwareManager::Snapshot()/accessors and the DataBus, and
// issues every command through Workflow.
#include "App.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "ui/Widgets.h"
#include "command/Commands.h"
#include "core/DutSession.h"   // the DUT-open edge that drops taught frames
#include "core/Log.h"
#include "vision/HalconEngine.h"

#include <cstddef>
#include <string>
#include <vector>

namespace firedog {

namespace {
constexpr int kWinW = 1280;
constexpr int kWinH = 800;

// Full-page takeovers (View menu) vs the Hardware pages. Only SwitchPage
// below cares — it owns the takeover/prevPage_ bookkeeping.
bool IsTakeoverPage(Page p)
{
    return p == Page::WatchEdit || p == Page::TransformEdit ||
           p == Page::RecipeEdit || p == Page::Scripts ||
           p == Page::DutView || p == Page::Run || p == Page::Align ||
           p == Page::VisionTeach || p == Page::AlignNominal ||
           p == Page::AlignPlace;
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
        if (ui.visionParamW > 0.0f) visionParamW_ = ui.visionParamW;
        showChart_       = ui.showChart;
        enableClickMove_ = ui.clickToMove;
        showVisionMarks_ = ui.showVisionMarks;
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
    // Probe HALCON once at startup (build / runtime DLLs / licence) so its
    // state is in the log band from the first frame rather than only when
    // somebody opens the Camera page. It never blocks bring-up: auto-focus and
    // every non-HALCON path work regardless of the answer.
    HalconEngine::Instance();

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

    // Same hand-off for a frame taught by align_solve: the procedure document
    // is UI-thread state, so the worker queues the result and it is stored (and
    // persisted) here.
    for (const auto& [uid, frame] : workflow_.TakeAlignResults())
        session_.ApplyAlignResult(uid, frame);

    // A finished reliability run writes its statistics into the detection model
    // it tested. Here rather than in the teach window's draw for the same reason
    // as the two above: it is DERIVED state landing on the document, and a Draw
    // function must never mutate the document.
    DrainVisionTestResult();

    // Captured E points, for the Aux window's live overlay. Same worker->UI
    // route; see DrainAlignPoints for the idle fallback.
    DrainAlignPoints();

    // A NEW DUT is a new physical part, so any taught frame stops meaning
    // anything. Edge-detected off DutSession's own state rather than hooked
    // into the two Open call sites (the dut_open command and the Overview
    // button), so a third one cannot be added without this firing.
    {
        const DutSession::Info di = DutSession::Instance().GetInfo();
        const std::string key = di.open ? di.serial + "|" + di.openedAt : std::string();
        if (key != dutOpenKey_) {
            dutOpenKey_ = key;
            if (!key.empty()) workflow_.ClearLiveFrames("new DUT opened");
        }
    }

    // Age the last detection here rather than timing it in a Draw, so the value
    // and its age can never disagree within a frame. A new detection is one on
    // a DIFFERENT camera frame — the detector stamps the frame it used.
    {
        const VisionDetection d = workflow_.LatestDetection();
        if (d.frameSeq != auxDetSeq_) {
            auxDetSeq_ = d.frameSeq;
            auxDet_    = d;
            auxDetAge_ = 0.0f;
        } else {
            auxDetAge_ += dt;
        }
    }

    // A motion device with no saved positions gets a zeroed "Home" to start
    // from. It needs the device's axis list, which only exists once the async
    // connect finished — so it is checked here rather than at startup, and not
    // in a Draw function (which must never mutate the document). Cheap: a map
    // lookup per device, and a no-op the moment the device has any position.
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        IMotionController* m = hardware_.MotionDevice(i);
        if (m && m->IsConnected())
            session_.SeedHomePosition(hardware_.MotionName(i), m->Axes());
    }
}

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

// ---- UI-side of the application document ----------------------------------
// ProductSession owns the data and the persistence rules; App owns the
// selections that point into it. These wrappers are the seam: no session ->
// UI callbacks, so every load re-points the UI here.

void App::SaveUiSettings()
{
    UiSettings s;
    s.fontIndex  = fontIndex_;
    s.fontSize   = uiFontSize_;
    s.themeIndex = themeIndex_;
    s.leftW     = leftW_;
    s.rightW    = rightW_;
    s.lowerH    = lowerH_;
    s.visionParamW = visionParamW_;
    s.showChart   = showChart_;
    s.clickToMove = enableClickMove_;
    s.showVisionMarks = showVisionMarks_;
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
    alignSel_ = session_.Aligns().empty() ? -1 : 0;
    alignNameBuf_[0] = '\0';
    alignStartErr_.clear();
    // The vision models are a different document now, so the teach page's
    // selection is stale — leave the page rather than showing a uid that
    // belongs to the previous product. Cached detectors go with it.
    if (currentPage_ == Page::VisionTeach) SwitchPage(Page::Align);
    // Same for the nominal teach page: its procedure belonged to the previous
    // product, and its two marks were measured against a frame this load has
    // just cleared.
    if (currentPage_ == Page::AlignNominal) SwitchPage(Page::Align);
    nomProcUid_.clear();
    nomEditUid_.clear();
    nomHave_[0] = nomHave_[1] = false;
    nomErr_.clear();
    // The placement page likewise: its measurement was of an object that was on
    // the machine for the previous product.
    if (currentPage_ == Page::AlignPlace) SwitchPage(Page::Align);
    plcProcUid_.clear();
    plcNomUid_.clear();
    plcHave_[0] = plcHave_[1] = false;
    plcLastMove_.clear();
    visionSelUid_.clear();
    visionOwnerLabel_.clear();
    visionTmplKey_.clear();
    visionHaveDet_ = false;
    visionRelHave_ = false;
    visionRelUid_.clear();
    visionErr_.clear();
    // The E marks belong to the old product's procedure; DrainAlignPoints
    // re-seeds them from the new selection on the next frame.
    alignMarkHave_[0] = alignMarkHave_[1] = false;
    auxDetAge_ = 1.0e6f;
    // A different product is a different part: no taught frame survives it.
    // This also covers app start, since Run() calls us before the first frame.
    workflow_.ClearLiveFrames("product loaded");
    workflow_.InvalidateDetector("");
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

void App::NewProduct(const char* name)
{
    session_.NewProduct(name);
    SaveUiSettings();   // remember as last product for the next startup
}

} // namespace firedog
