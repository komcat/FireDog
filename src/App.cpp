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
#include "core/Employees.h"    // the roster behind the login gate
#include "core/Log.h"
#include "core/DeviceReg.h"    // SaveDeviceRegsFile (the registration hand-back)
#include "core/NeedleCal.h"    // SaveNeedleCalFile (the calibration hand-back)
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
           p == Page::AlignPlace || p == Page::Needle || p == Page::DeviceReg ||
           p == Page::Grip || p == Page::Tray;
}
} // namespace

int App::Run()
{
    FD_INFO("app", "FireDog starting");
    Logger::Instance().SetMinLevel(static_cast<LogLevel>(logMinLevel_));
    // Read the settings BEFORE the window exists: where it goes and how big it
    // is are arguments to its creation, not something to fix up afterwards —
    // a created-then-moved window flashes at the default spot first.
    // Defaults to FireDog Dark / Segoe UI 26 px on first run, and every index
    // is validated against the current tables.
    std::string lastProduct;
    const UiSettings ui = UiSettings::Load(kUiSettingsPath);
    {
        fontIndex_  = (ui.fontIndex >= 0 && ui.fontIndex < kFontCount) ? ui.fontIndex : 0;
        uiFontSize_ = (ui.fontSize >= 8.0f && ui.fontSize <= 48.0f) ? ui.fontSize : 26.0f;
        themeIndex_ = (ui.themeIndex >= 0 && ui.themeIndex < Theme::Count()) ? ui.themeIndex : 0;
        // Panel sizes: keep saved px if present; 0 lets DrawMainWindow seed them.
        if (ui.leftW  > 0.0f) leftW_  = ui.leftW;
        if (ui.rightW > 0.0f) rightW_ = ui.rightW;
        if (ui.lowerH > 0.0f) lowerH_ = ui.lowerH;
        if (ui.visionParamW  > 0.0f) visionParamW_  = ui.visionParamW;
        if (ui.visionResultW > 0.0f) visionResultW_ = ui.visionResultW;
        if (ui.edInspectW    > 0.0f) edInspectW_    = ui.edInspectW;
        if (ui.edPaletteW    > 0.0f) edPaletteW_    = ui.edPaletteW;
        edShowHidden_ = ui.edShowHidden;
        edTagFilter_  = ui.edTagFilter;
        edTagHide_    = ui.edTagHide;
        if (ui.runDeckW      > 0.0f) runDeckW_      = ui.runDeckW;
        if (ui.runDutW       > 0.0f) runDutW_       = ui.runDutW;
        if (ui.runScanH      > 0.0f) runScanH_      = ui.runScanH;
        if (ui.runShotH      > 0.0f) runShotPanelH_ = ui.runShotH;
        if (ui.orgLibW       > 0.0f) orgLibW_       = ui.orgLibW;
        if (ui.regCtlW       > 0.0f) regCtlW_       = ui.regCtlW;
        if (ui.gripCtlW      > 0.0f) gripCtlW_      = ui.gripCtlW;
        if (ui.trayCtlW      > 0.0f) trayCtlW_      = ui.trayCtlW;
        showTray_        = ui.showTrayWin;
        showChart_       = ui.showChart;
        enableClickMove_ = ui.clickToMove;
        showVisionMarks_ = ui.showVisionMarks;
        showPosLabels_   = ui.showPosLabels;
        posLabelsHidden_ = ui.posLabelsHidden;
        showGripperMarks_ = ui.showGripperMarks;
        auxPanelOpen_     = ui.auxPanelOpen;
        chartScope_      = ui.chartScope;
        runLastProgUid_  = ui.runProgUid;
        runLastSlot_     = ui.runSlot > 0 ? ui.runSlot : 0;
        lastProduct      = ui.lastProduct;
        posPad_          = ui.posPad;
        runReadouts_     = ui.runReadouts;
        if (runReadouts_.size() > 4) runReadouts_.resize(4);
        quickIo_         = ui.quickIo;
    }

    WindowPlacement place;
    place.x = ui.winX;  place.y = ui.winY;
    place.w = ui.winW;  place.h = ui.winH;
    place.maximized = ui.winMaximized;
    if (!host_.Init("FireDog - Active Alignment", kWinW, kWinH, place)) {
        FD_ERROR("app", "GlHost init failed");
        return 1;
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
    // The roster, read once. It lives under products/ rather than config/
    // because a rebuild recopies config/ and would replace every registration
    // made on this machine — see core/Employees.h.
    employees_ = LoadEmployees();
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
        // THE ACCESS GATE. While nobody is signed in the login screen draws
        // INSTEAD of the main window, so the menu bar, the F-keys, the quick
        // strip and every floating window do not exist yet — there is no
        // surface left to forget to gate. Update() above still runs, so the
        // hardware connects while the operator is still typing their number.
        //
        // The windows below carry their own level check as well as their open
        // bit: a role is not something a window may outlive.
        if (!user_.LoggedIn()) {
            DrawLoginScreen();
        } else {
            DrawMainWindow();
            if (showAux_   && CanUse(kRoleAux)) DrawAuxWindow();
            if (showJog_   && CanUse(kRoleJog)) DrawJogWindow();
            if (orgOpen_   && CanUse(kRoleOrg)) DrawProgramOrganizer();
            if (showData_  && CanUse(kRoleData)) DrawViewDataWindow();
            if (showPosHist_ && CanUse(kRoleJog)) DrawPositionHistoryWindow();
            if (showTray_  && CanUse(kRoleTray)) DrawTrayWindow();
            if (showUsers_) DrawEmployeeManager();   // checks Admin itself
        }
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
    // BEFORE programs.Tick(): the runner starts the next queue item the moment
    // it sees the engine idle, so a drain after it would miss the edge on every
    // recipe but the last. Ordering is the whole mechanism here.
    DrainCycleVars();
    orch_.programs.Tick();       // sequences program/slot runs between frames
    // The deck cycle, both halves of it. The marks are copied out of the
    // runner's per-item status every frame — which is what makes an accent
    // appear the moment a recipe finishes rather than only at the end of the
    // queue — and the runner going idle is edge-detected here, the same shape
    // as the dutOpenKey_ detector below. Design: docs/deck-cycle-design.md.
    DrainDeckMarks();
    {
        // A teach run proves nothing about a part: it neither marks nodes nor
        // advances the cycle (Status.teach outlives the run for this frame).
        const bool runActive = orch_.programs.Active();
        if (runWasActive_ && !runActive && !orch_.programs.GetStatus().teach)
            AdvanceDeckCycle();
        runWasActive_ = runActive;
    }
    UpdateKeithleyChartData(dt); // shared chart buffers (Aux + Keithley page)
    UpdateMoveWatch(dt);         // a UI move in flight (the frame's MOVING banner)

    // Teach hand-backs from save_position steps: engine worker threads queue
    // them on Workflow, and THIS is the worker->UI hand-off point — the
    // session is UI-thread-only, so the drain must stay here in Update().
    {
        // The journal names the run that taught: whichever engine owns the
        // worker that queued these (a program's slot is a recipe Start).
        std::vector<std::pair<std::string, SavedPos>> taught = workflow_.TakeSavedPositions();
        if (!taught.empty()) {
            std::string source = "teach";
            if (orch_.recipe.Running())   source = "recipe " + orch_.recipe.GetStatus().recipeName;
            else if (orch_.lua.Running()) source = "script " + orch_.lua.GetStatus().scriptName;
            session_.MergeTaughtPositions(taught, source);
        }
    }

    // Same hand-off for a frame taught by align_solve: the procedure document
    // is UI-thread state, so the worker queues the result and it is stored (and
    // persisted) here.
    for (const auto& [uid, frame] : workflow_.TakeAlignResults())
        session_.ApplyAlignResult(uid, frame);

    // And the same route for a freshly measured needle offset. Not a document
    // this time — it is machine state, so it goes back to Workflow and into its
    // own config file — but the reason for the hand-off is identical: a worker
    // thread must not write a config file, least of all one it might be halfway
    // through when a Stop lands.
    for (const NeedleCal& c : workflow_.TakeNeedleCals()) {
        workflow_.SetNeedleCal(c);
        if (SaveNeedleCalFile(NeedleCalLocalPath(), c))
            FD_INFO("needle", "needle calibration saved to %s", NeedleCalLocalPath());
    }

    // And a freshly measured device registration, by the same route for the same
    // reason. One difference: the file holds the WHOLE fleet, so the live map is
    // updated first and then written as a unit — writing only the record that
    // changed would drop every other device's registration on the first save.
    // What a grip run measured, by the same route and for the same reason as a
    // taught frame: the document is UI-thread state. Evidence only - the record
    // stores it as "last measured" and never as a new reference, because
    // promoting a measurement to the standard is an operator's decision.
    for (const Workflow::GripMeasurement& m : workflow_.TakeGripResults()) {
        session_.ApplyGripResult(m.uid, m.dx, m.dy);
        // ...and the Aux window's readout of it. The verdict is computed HERE,
        // against the profile the document holds, by the same GripVerdict the
        // command used - not shipped across the boundary as a second copy.
        const GripProfile* g = FindGripProfile(session_.Grips(), m.uid);
        if (!g) continue;   // already warned by ApplyGripResult
        AuxGripReadout r;
        r.have   = true;
        r.name   = g->name;
        r.device = g->device;
        r.cx     = m.cx;
        r.cy     = m.cy;
        r.dx     = m.dx;
        r.dy     = m.dy;
        r.ok     = GripVerdict(*g, m.dx, m.dy, &r.why);
        r.at     = g->measuredAt;
        auxGrip_ = std::move(r);
    }

    if (const std::vector<DeviceReg> regs = workflow_.TakeDeviceRegs(); !regs.empty()) {
        for (const DeviceReg& r : regs) workflow_.SetDeviceReg(r);
        if (SaveDeviceRegsFile(DeviceRegLocalPath(), workflow_.DeviceRegs()))
            FD_INFO("devreg", "device registrations saved to %s", DeviceRegLocalPath());
    }

    // A run's tray_next / tray_reset already moved the Workflow cursor; this
    // is the document catching up so the used cells survive a restart. The
    // cursor is not touched here — it is the worker's, and re-applying its
    // own change would be a second writer.
    for (const Workflow::TrayUsedEvent& ev : workflow_.TakeTrayUsed()) {
        std::vector<TrayCell> cells;
        if (!ev.all) cells.push_back(ev.cell);
        session_.SetTrayCellsUsed(ev.trayUid, cells, ev.used);
    }

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
            if (!key.empty()) {
                workflow_.ClearLiveFrames("new DUT opened");
                // And the picture, for the same reason: a still shows ONE
                // physical part. Left up, last part's serial would sit over
                // this part's run looking like a reading somebody just took.
                runShotW_ = runShotH_ = 0;
                runShotLabel_.clear();
                runShotStamp_.clear();
                runShotPath_.clear();
            }
        }
    }

    // The still a capture_view / dut_capture_image step took, turned into a
    // texture. Here for the same reason as every drain above: the command ran
    // on an engine worker, and uploading a texture is a GL call only the UI
    // thread may make.
    //
    // AFTER the DUT edge above, and that order is load-bearing: a recipe that
    // does dut_open and then dut_capture_image can land both inside one frame,
    // and draining first would have the open wipe the picture the capture just
    // delivered.
    DrainCapturedView();

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

// The pt1/pt2 diagrams shown over the live image while an object is being
// marked. Loaded lazily and ONCE (success or failure): the first surface that
// wants them runs long after GL is up, and a missing assets/ folder must cost a
// diagram, never a frame of retrying every draw.
void App::EnsureMarkIcons()
{
    if (markIconsTried_) return;
    markIconsTried_ = true;

    const char* paths[2] = {"assets/icons/pt1.png", "assets/icons/pt2.png"};
    for (int i = 0; i < 2; ++i) {
        std::vector<unsigned char> rgba;
        int w = 0, h = 0;
        if (!LoadPngRGBA(paths[i], &rgba, &w, &h)) {
            FD_WARN("ui", "mark icon '%s' not loaded - the step diagram is skipped",
                    paths[i]);
            continue;
        }
        markIcons_[i].tex = host_.MakeTextureRGBA(w, h, rgba.data());
        markIcons_[i].w   = w;
        markIcons_[i].h   = h;
    }
}

// One icon from assets/recipe_icons, by FILE stem. Same lazy-once rule as the
// mark icons, but keyed per file rather than per surface: several commands
// share a glyph, and loading it twice would be two textures of the same 32x32.
const MarkIcon* App::UiIcon(const char* file)
{
    if (!file || !*file) return nullptr;

    auto it = stepIcons_.find(file);
    if (it == stepIcons_.end()) {
        MarkIcon ic;
        const std::string path = std::string("assets/recipe_icons/") + file + ".png";
        std::vector<unsigned char> rgba;
        int w = 0, h = 0;
        if (LoadPngRGBA(path.c_str(), &rgba, &w, &h)) {
            ic.tex = host_.MakeTextureRGBA(w, h, rgba.data());
            ic.w   = w;
            ic.h   = h;
        } else {
            FD_WARN("ui", "recipe icon '%s' not loaded - drawn without it",
                    path.c_str());
        }
        it = stepIcons_.emplace(file, ic).first;   // cache the miss as well
    }
    return it->second.tex ? &it->second : nullptr;
}

// ALL page switches route here. Leaving the Recipe Editor with unsaved edits
// blocks: the editor stays up and its Unsaved Changes modal (Save / Discard /
// Cancel) completes the switch to edLeaveTarget_. Entering a takeover page
// from a hardware page remembers it so the editor's Close returns there.
//
// A RUN OUTRANKS THE GUARD. The guard is an ImGui MODAL, and a modal blocks
// input to every other window - including the always-on-top prompt /
// manual_adjust / pick windows an engine worker is blocked on. Raise it while
// a run is waiting on the operator and the machine is stuck behind a dialog
// about unsaved text: the prompt is still on screen, still un-clickable, and
// the run dies on the Stop that eventually follows. So while orch_ is busy the
// switch simply goes through - the edits stay in memory and stay dirty, and
// the guard does its job the next time the machine is idle. Changing pages
// must never be able to interfere with an execution in progress.
void App::SwitchPage(Page p)
{
    if (currentPage_ == p) return;
    // THE ACCESS CHOKE POINT. Every switch in the tree routes here, so a button
    // or shortcut that forgot to check still cannot land on a page its level
    // has no business on. The menu items and strip buttons hide themselves as
    // well, but that is courtesy; this is the enforcement.
    //
    // It warns rather than failing silently: the log band is on screen on the
    // operator pages, and "F5 did nothing" is the report you get otherwise.
    if (!CanAccess(p)) {
        FD_WARN("users", "%s (%s) may not open that page - %s level required",
                user_.id.c_str(), RoleName(user_.role), RoleName(MinRoleFor(p)));
        return;
    }
    if (currentPage_ == Page::RecipeEdit && session_.RecipesDirty() && !orch_.Busy()) {
        edConfirmAction_ = 1;
        edLeaveTarget_   = p;
        return;
    }
    // Same guard, same reasoning, for the script editor's buffer.
    if (currentPage_ == Page::Scripts && ScriptDirty() && !orch_.Busy()) {
        scConfirmAction_ = 2;
        scLeaveTarget_   = p;
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
    s.visionParamW  = visionParamW_;
    s.visionResultW = visionResultW_;
    s.edInspectW    = edInspectW_;
    s.edPaletteW    = edPaletteW_;
    s.edShowHidden  = edShowHidden_;
    s.edTagFilter   = edTagFilter_;
    s.edTagHide     = edTagHide_;
    s.runDeckW      = runDeckW_;
    s.runDutW       = runDutW_;
    s.runScanH      = runScanH_;
    s.runShotH      = runShotPanelH_;
    s.orgLibW       = orgLibW_;
    s.regCtlW       = regCtlW_;
    s.gripCtlW      = gripCtlW_;
    s.trayCtlW      = trayCtlW_;
    s.showTrayWin   = showTray_;
    s.showChart   = showChart_;
    s.clickToMove = enableClickMove_;
    s.showVisionMarks = showVisionMarks_;
    s.showPosLabels   = showPosLabels_;
    s.posLabelsHidden = posLabelsHidden_;
    s.showGripperMarks = showGripperMarks_;
    s.auxPanelOpen     = auxPanelOpen_;
    s.chartScope  = chartScope_;
    s.runProgUid  = runLastProgUid_;
    s.runSlot     = runLastSlot_;
    s.lastProduct = session_.Product().name;
    s.posPad      = posPad_;
    s.runReadouts = runReadouts_;
    s.quickIo     = quickIo_;
    // The window's own geometry comes from the host, which is the only thing
    // that knows it — App never tracks a size it does not set.
    const WindowPlacement& place = host_.Placement();
    s.winX = place.x;  s.winY = place.y;
    s.winW = place.w;  s.winH = place.h;
    s.winMaximized = place.maximized;
    s.Save(kUiSettingsPath);
}

// Re-point every UI selection at a freshly (re)loaded session document. The
// session never calls back into the UI, so each load wrapper does this here.
void App::ResetDocumentSelections()
{
    recipeSel_ = -1;
    recipeStartErr_.clear();
    RestoreDeckSelection();
    runStartErr_.clear();
    alignSel_ = session_.Aligns().empty() ? -1 : 0;
    // The grip profiles are a different document too, and the page's marks
    // describe a pick of the OLD product's part: clearing them is the same
    // rule that clears a live frame at a product load.
    gripSel_ = session_.Grips().empty() ? -1 : 0;
    gripMarkHave_[0] = gripMarkHave_[1] = false;
    gripMarkActive_  = 0;
    gripStartErr_.clear();
    gripTeachMsg_.clear();
    alignNameBuf_[0] = '\0';
    alignStartErr_.clear();
    alignView_ = 0;          // the step pane lands on E1 for a new document
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
    plcSamples_.clear();
    visionSelUid_.clear();
    visionOwnerLabel_.clear();
    // Every patch thumbnail belongs to the models that just went away.
    visionTmpl_.key.clear();
    alignTmpl_[0].key.clear();
    alignTmpl_[1].key.clear();
    pickTmpl_.key.clear();
    visionHaveDet_ = false;
    visionRelHave_ = false;
    visionRelUid_.clear();
    visionErr_.clear();
    // The E marks belong to the old product's procedure; DrainAlignPoints
    // re-seeds them from the new selection on the next frame.
    alignMarkHave_[0] = alignMarkHave_[1] = false;
    auxDetAge_ = 1.0e6f;
    // The grip readout names a profile of the old product.
    auxGrip_ = AuxGripReadout{};
    // A different product is a different part: no taught frame survives it.
    // This also covers app start, since Run() calls us before the first frame.
    workflow_.ClearLiveFrames("product loaded");
    // And a different product's trays are different trays: the cursor is
    // rebuilt from what THIS product's records say is used — the persisted
    // set is the truth, the cursor its live mirror.
    workflow_.ClearTrayCursor("", "product loaded");
    for (const WaffleTray& t : session_.Trays()) workflow_.TraySeed(t.uid, t.usedCells);
    traySel_ = session_.Trays().empty() ? -1 : 0;
    trayMsg_.clear();
    workflow_.InvalidateDetector("");
    SyncScriptSelection();
}

// Scripts page selection, after the session rescanned the scripts dir: keep
// the selection on the same FILE (a Save rescans to refresh the description
// and must not hop), otherwise a still-valid index, otherwise the first
// script (or none). The editor buffer is not touched here: a clean buffer
// whose file is no longer listed (product changed) is dropped by the page on
// its next draw, and a DIRTY one is kept and shown as an orphan, because a
// product load is not a licence to discard somebody's edits.
void App::SyncScriptSelection()
{
    int found = -1;
    for (int i = 0; i < static_cast<int>(session_.Scripts().size()); ++i) {
        if (session_.Scripts()[i].path == scriptEd_.path) { found = i; break; }
    }
    if (found >= 0)
        scriptSel_ = found;
    else if (scriptSel_ >= static_cast<int>(session_.Scripts().size()))
        scriptSel_ = session_.Scripts().empty() ? -1 : 0;
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

// Thin wrapper (like LoadRecipesFromDisk): session loads, App re-points the
// deck at the remembered program/slot (or the first, if this set has neither).
void App::LoadProgramsFromDisk()
{
    session_.LoadProgramsFromDisk();
    RestoreDeckSelection();
    runStartErr_.clear();
}

void App::NewProduct(const char* name)
{
    session_.NewProduct(name);
    // The trays carried over, used cells included: the fixture on the machine
    // did not change because the product folder did. NewProduct does not go
    // through ResetDocumentSelections, so the cursor is re-seeded here.
    workflow_.ClearTrayCursor("", "new product");
    for (const WaffleTray& t : session_.Trays()) workflow_.TraySeed(t.uid, t.usedCells);
    SaveUiSettings();   // remember as last product for the next startup
}

} // namespace firedog
