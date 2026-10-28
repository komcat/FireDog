// MainWindow.cpp : the Main window frame — menu bar, the three-panel layout
// with its draggable splitters, page dispatch, and the two global modals
// (safety alert, exit confirm). Definitions of App members — see App.h.
//
// Page switching: ALL switches route through App::SwitchPage (in App.cpp),
// which owns the takeover/prevPage bookkeeping and the dirty-recipe guard.
// Draw dispatch here must not assign currentPage_ directly.
#include "App.h"

#include "imgui.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "ui/Widgets.h"
#include "core/Log.h"
#include "update/AppUpdater.h"

#include <cmath>
#include <cstdio>

namespace firedog {

namespace {

// Arrival rule for a watched UI move, the same two things a move command waits
// for: every targeted axis inside tolerance, AND the controller reporting the
// profile finished and on target. The tolerance matches kMoveTolMM
// (command/MotionWaits.h) — this is a UI progress indicator, not a measurement
// gate, so it reads the same numbers rather than sharing the engine's blocking
// helper (which may only be called from a worker).
constexpr double kWatchTolMM   = 0.01;
constexpr float  kWatchTimeout = 60.0f;   // seconds before the banner gives up

bool WatchArrived(IMotionController* m,
                  const std::vector<std::pair<std::string, double>>& targets)
{
    const std::map<std::string, double> pos = m->Positions();
    for (const auto& [axis, tgt] : targets) {
        const auto it = pos.find(axis);
        if (it == pos.end() || std::fabs(it->second - tgt) > kWatchTolMM) return false;
    }
    return m->IsSettled() && m->IsOnTarget();
}

} // namespace

// Command a move to a saved position and start watching it. Addressed by uid
// (hidden rows included — a private station position is exactly what this is
// for); `what` is the phrase the banner shows ("station 1 of 'calibrate
// plate'"), so it reads as a place rather than a device index.
//
// False = nothing was commanded. The caller says why: the reasons (no position
// taught, device not connected, no shared axes) are all things its own button
// already knows about, and a banner is the wrong place to explain them.
bool App::StartWatchedMove(const std::string& device, const std::string& posUid,
                           const std::string& what)
{
    if (posUid.empty()) return false;
    const auto pit = session_.Positions().find(device);
    if (pit == session_.Positions().end()) return false;
    const SavedPos* pos = nullptr;
    for (const SavedPos& sp : pit->second)
        if (sp.uid == posUid) { pos = &sp; break; }
    if (!pos) return false;
    return StartWatchedMove(device, pos->vals, what);
}

bool App::StartWatchedMove(const std::string& device,
                           const std::map<std::string, double>& pose,
                           const std::string& what)
{
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) != device) continue;
        IMotionController* dev = hardware_.MotionDevice(i);
        if (!dev || !dev->IsConnected()) return false;

        std::vector<std::pair<std::string, double>> targets;
        for (const std::string& ax : dev->Axes()) {
            const auto it = pose.find(ax);
            if (it != pose.end()) targets.emplace_back(ax, it->second);
        }
        if (targets.empty()) return false;
        if (!workflow_.MoveToPosition(i, targets)) return false;

        moveWatch_.active  = true;
        moveWatch_.devIdx  = i;
        moveWatch_.device  = device;
        moveWatch_.what    = what;
        moveWatch_.targets = std::move(targets);
        moveWatch_.elapsed = 0.0f;
        FD_INFO("ui", "%s: driving to %s", device.c_str(), what.c_str());
        return true;
    }
    return false;
}

// Poll the watched move to completion. From Update(), never from a Draw: the
// banner and the arrival check must not disagree within a frame, and this is
// the only place the watch is cleared.
void App::UpdateMoveWatch(float dt)
{
    if (!moveWatch_.active) return;

    IMotionController* m = hardware_.MotionDevice(moveWatch_.devIdx);
    if (!m || !m->IsConnected()) {   // unplugged mid-move: the watch means nothing
        moveWatch_.active = false;
        return;
    }
    m->RequestFastPoll();            // check on fresh positions, like WaitArrive
    if (WatchArrived(m, moveWatch_.targets)) {
        moveWatch_.active = false;
        FD_INFO("ui", "%s: arrived at %s (%.1f s)", moveWatch_.device.c_str(),
                moveWatch_.what.c_str(), static_cast<double>(moveWatch_.elapsed));
        return;
    }
    // The move is NOT stopped on timeout: the axes may simply be slow, and a UI
    // progress banner has no business aborting motion an operator asked for.
    // It stops CLAIMING to know, which is the honest failure.
    moveWatch_.elapsed += dt;
    if (moveWatch_.elapsed > kWatchTimeout) {
        moveWatch_.active = false;
        FD_WARN("ui", "%s: still not at %s after %.0f s - no longer watching",
                moveWatch_.device.c_str(), moveWatch_.what.c_str(),
                static_cast<double>(kWatchTimeout));
    }
}

// One line under the quick-access strip, on EVERY page — including the takeover
// pages, which is the point: the teach page is opened by a move, and the
// operator lands on it while the gantry is still travelling.
void App::DrawMoveStatusBar()
{
    if (!moveWatch_.active) return;

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.78f, 0.30f, 1.0f));
    ImGui::Text("MOVING  %s -> %s   %.1f s", moveWatch_.device.c_str(),
                moveWatch_.what.c_str(), static_cast<double>(moveWatch_.elapsed));
    ImGui::PopStyleColor();
    ImGui::SameLine();
    if (ImGui::SmallButton("Stop##movewatch")) {
        workflow_.StopMotion(moveWatch_.devIdx);
        moveWatch_.active = false;
    }
    ImGui::Separator();
}

// The User menu plus the identity chip beside it. Called from the menu bar
// (see DrawMainWindow), so it must be inside a BeginMenuBar block.
//
// The chip is text in the bar rather than a click away on purpose: "who is this
// station signed in as" is the question a supervisor walking the line asks, and
// every DUT record written right now carries this answer.
void App::DrawUserMenu()
{
    if (ImGui::BeginMenu("User")) {
        ImGui::MenuItem(user_.Label().c_str(), nullptr, false, false);
        ImGui::MenuItem(RoleName(user_.role), nullptr, false, false);
        ImGui::Separator();
        // Registering a number, setting its level and setting its password are
        // one admin act on one surface — see WndEmployees.cpp.
        if (CanUse(UserRole::Admin)) {
            if (ImGui::MenuItem("Employee Numbers...")) showUsers_ = true;
            ImGui::Separator();
        }
        // Signing out mid-run would leave a program driving axes with nobody
        // attributable to it, and the record would name whoever signed in next.
        // Refused rather than queued: the honest answer is "stop the run".
        const bool busy = orch_.Busy();
        if (ImGui::MenuItem("Sign out", nullptr, false, !busy)) LogOut();
        if (busy && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("A run is in progress - stop it first");
        ImGui::EndMenu();
    }
    ImGui::TextDisabled("  %s (%s)", user_.id.c_str(), RoleName(user_.role));
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
        // Placement opens on whatever procedure the Alignment page has
        // selected, so all three ways in — the View menu, F6, the quick strip —
        // want the same two lines. It is NOT a plain page switch: OpenAlignPlace
        // resets the marks, so a page already open is left alone. The F-keys
        // activate rather than toggle anyway, and here that also protects a
        // measurement in progress from a stray key.
        //
        // The list is fetched on use, not captured: the File menu above can
        // replace the session's procedures wholesale.
        const auto openPlacement = [&]() {
            if (currentPage_ == Page::AlignPlace) return;
            // Checked here as well as in SwitchPage, because the "no procedure
            // selected" warning below would otherwise be the first thing an
            // operator pressing F6 is told — an answer to a question they are
            // not allowed to ask.
            if (!CanAccess(Page::AlignPlace)) {
                FD_WARN("users", "%s (%s) may not open Placement",
                        user_.id.c_str(), RoleName(user_.role));
                return;
            }
            std::vector<AlignProc>& list = session_.Aligns();
            if (alignSel_ < 0 || alignSel_ >= static_cast<int>(list.size())) {
                FD_WARN("ui", "Placement needs an alignment procedure selected "
                              "(Alignment, F5)");
                return;
            }
            AlignProc& a = list[static_cast<std::size_t>(alignSel_)];
            OpenAlignPlace(a, a.nominals.empty() ? std::string()
                                                 : a.nominals.front().uid);
        };

        // The Version page's menu item, in View and in Help: one definition so
        // the two cannot disagree. The label carries the news - a found release
        // says so where an Admin looks, without a popup over anybody's work.
        // Callers check CanAccess(Page::Version) first.
        const auto versionItem = [&]() {
            const AppUpdater::Status us = updater_.GetStatus();
            const bool news = us.state == AppUpdater::State::Available ||
                              us.state == AppUpdater::State::Ready;
            const std::string label =
                news ? "Version / Update  (" + us.latestVersion + " available)"
                     : std::string("Version / Update");
            if (ImGui::MenuItem(label.c_str(), nullptr, currentPage_ == Page::Version))
                ToggleEditorPage(Page::Version);
        };

        // ---- Menu bar (top strip) ----
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                // Creating a product defines what the machine builds, so it is
                // Admin. Loading and saving are Technician: the recipe editor
                // they already have writes into the product, and an editor whose
                // Save is out of reach is a trap.
                if (CanUse(UserRole::Admin) && ImGui::MenuItem("New Product..."))
                    openNewProduct_  = true;
                if (CanUse(UserRole::Technician)) {
                    if (ImGui::MenuItem("Load Product...")) openLoadProduct_ = true;
                    if (ImGui::MenuItem("Save Product", nullptr, false,
                                        session_.ProductLoaded()))
                        session_.SaveProduct();
                }
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
            // The hardware pages are all Technician, so the menu goes as a unit
            // rather than emptying itself item by item for an operator.
            if (CanUse(UserRole::Technician) && ImGui::BeginMenu("Hardware")) {
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
                if (ImGui::MenuItem("Lighting", nullptr, currentPage_ == Page::Light))
                    SwitchPage(Page::Light);
                if (ImGui::MenuItem("Motion",   "F3", currentPage_ == Page::Motion))
                    SwitchPage(Page::Motion);
                if (ImGui::MenuItem("IO",       nullptr, currentPage_ == Page::IO))
                    SwitchPage(Page::IO);
                if (ImGui::MenuItem("Pneumatics", nullptr, currentPage_ == Page::Pneumatic))
                    SwitchPage(Page::Pneumatic);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                if (CanUse(kRoleAux)) ImGui::MenuItem("Aux Window", nullptr, &showAux_);
                if (CanUse(kRoleJog)) ImGui::MenuItem("Jog",        nullptr, &showJog_);
                // Same bit as the Run strip's Edit checkbox — one window, two
                // ways in, never two answers about whether it is open.
                if (CanUse(kRoleOrg) &&
                    ImGui::MenuItem("Program Organizer", nullptr, &orgOpen_))
                    runEdit_ = orgOpen_;
                if (CanUse(kRoleData)) ImGui::MenuItem("View Data", nullptr, &showData_);
                if (CanUse(kRoleJog))  ImGui::MenuItem("Position History", nullptr, &showPosHist_);
                if (CanUse(kRoleTray)) ImGui::MenuItem("Waffle Tray window", nullptr, &showTray_);
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
                // shortcuts), then the machine-setup editors. Each item is
                // present only for a level that can open it: SwitchPage would
                // refuse anyway, but a menu full of doors that answer with a
                // warning is worse than a short menu.
                if (CanAccess(Page::Run) &&
                    ImGui::MenuItem("Run", "F1", currentPage_ == Page::Run))
                    ToggleEditorPage(Page::Run);
                if (CanAccess(Page::RecipeEdit) &&
                    ImGui::MenuItem("Recipe Editor", "F2", currentPage_ == Page::RecipeEdit))
                    ToggleEditorPage(Page::RecipeEdit);
                // Everything below this line is Technician or above, so for an
                // Operator the separator would be the last thing in the menu —
                // a rule under nothing, which reads as items that failed to
                // draw rather than as items that are not theirs.
                if (CanUse(UserRole::Technician)) ImGui::Separator();
                if (CanAccess(Page::WatchEdit) &&
                    ImGui::MenuItem("IO Watch Editor", nullptr, currentPage_ == Page::WatchEdit))
                    ToggleEditorPage(Page::WatchEdit);
                if (CanAccess(Page::TransformEdit) &&
                    ImGui::MenuItem("Transforms Editor", nullptr, currentPage_ == Page::TransformEdit))
                    ToggleEditorPage(Page::TransformEdit);
                if (CanAccess(Page::Align) &&
                    ImGui::MenuItem("Alignment", "F5", currentPage_ == Page::Align))
                    ToggleEditorPage(Page::Align);
                // Placement IS reachable from a nominal's Place... button, but
                // that is three clicks deep inside another page — findable only
                // by someone who already knows it exists. It opens on whatever
                // procedure the Alignment page has selected and picks that
                // procedure's own combo up from there, so the menu costs it
                // nothing: unlike the two teach pages, it carries no state that
                // only the caller could supply.
                if (CanAccess(Page::AlignPlace)) {
                    const bool haveProc =
                        alignSel_ >= 0 &&
                        alignSel_ < static_cast<int>(session_.Aligns().size());
                    if (ImGui::MenuItem("Placement", "F6",
                                        currentPage_ == Page::AlignPlace, haveProc))
                        openPlacement();
                    if (!haveProc && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                        ImGui::SetTooltip("Select an alignment procedure first (View > Alignment)");
                }
                if (CanAccess(Page::Needle) &&
                    ImGui::MenuItem("Needle", "F7", currentPage_ == Page::Needle))
                    ToggleEditorPage(Page::Needle);
                if (CanAccess(Page::DeviceReg) &&
                    ImGui::MenuItem("Registration", "F8", currentPage_ == Page::DeviceReg))
                    ToggleEditorPage(Page::DeviceReg);
                if (CanAccess(Page::Grip) &&
                    ImGui::MenuItem("Gripped object", "F9", currentPage_ == Page::Grip))
                    ToggleEditorPage(Page::Grip);
                if (CanAccess(Page::Tray) &&
                    ImGui::MenuItem("Waffle Tray", "F10", currentPage_ == Page::Tray))
                    ToggleEditorPage(Page::Tray);
                if (CanAccess(Page::Mosaic) &&
                    ImGui::MenuItem("Mosaic", "F11", currentPage_ == Page::Mosaic))
                    ToggleEditorPage(Page::Mosaic);
                if (CanAccess(Page::Scripts) &&
                    ImGui::MenuItem("Scripts", "F4", currentPage_ == Page::Scripts))
                    ToggleEditorPage(Page::Scripts);
                if (CanAccess(Page::DutView) &&
                    ImGui::MenuItem("DUT Records", nullptr, currentPage_ == Page::DutView)) {
                    if (currentPage_ != Page::DutView) LoadDutList();
                    ToggleEditorPage(Page::DutView);
                }
                if (CanAccess(Page::Version)) {
                    ImGui::Separator();
                    versionItem();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Help")) {
                if (CanAccess(Page::Version)) {
                    versionItem();
                } else {
                    ImGui::MenuItem("About", nullptr, false, false);
                }
                ImGui::EndMenu();
            }
            DrawUserMenu();   // who is signed in, sign out, manage the roster
            if (workflow_.SafetyAlertActive()) {
                ImGui::TextColored(StateColor(ConnState::Failed), "  !! SAFETY ALERT !!");
            }
            ImGui::EndMenuBar();
        }

        // Global page shortcuts: F1 Run, F2 Recipe Editor, F3 Motion, F4
        // Scripts, F5 Alignment, F6 Placement, F7 Needle, F8 Registration,
        // F9 Gripped object, F10 Waffle Tray, F11 Mosaic. Activate (not toggle)
        // — pressing the key on its own page is a no-op.
        if (ImGui::IsKeyPressed(ImGuiKey_F1, false) && currentPage_ != Page::Run)
            ToggleEditorPage(Page::Run);
        if (ImGui::IsKeyPressed(ImGuiKey_F2, false) && currentPage_ != Page::RecipeEdit)
            ToggleEditorPage(Page::RecipeEdit);
        if (ImGui::IsKeyPressed(ImGuiKey_F3, false))
            SwitchPage(Page::Motion);
        // Alt+F4 is the OS close request — never let it double as a page jump.
        if (ImGui::IsKeyPressed(ImGuiKey_F4, false) && !ImGui::GetIO().KeyAlt &&
            currentPage_ != Page::Scripts)
            ToggleEditorPage(Page::Scripts);
        if (ImGui::IsKeyPressed(ImGuiKey_F5, false) && currentPage_ != Page::Align)
            ToggleEditorPage(Page::Align);
        // F6 says so in the log when no procedure is selected: a key press has
        // no disabled state to look at, so a silent nothing reads as broken.
        if (ImGui::IsKeyPressed(ImGuiKey_F6, false))
            openPlacement();
        if (ImGui::IsKeyPressed(ImGuiKey_F7, false) && currentPage_ != Page::Needle)
            ToggleEditorPage(Page::Needle);
        if (ImGui::IsKeyPressed(ImGuiKey_F8, false) && currentPage_ != Page::DeviceReg)
            ToggleEditorPage(Page::DeviceReg);
        if (ImGui::IsKeyPressed(ImGuiKey_F9, false) && currentPage_ != Page::Grip)
            ToggleEditorPage(Page::Grip);
        if (ImGui::IsKeyPressed(ImGuiKey_F10, false) && currentPage_ != Page::Tray)
            ToggleEditorPage(Page::Tray);
        if (ImGui::IsKeyPressed(ImGuiKey_F11, false) && currentPage_ != Page::Mosaic)
            ToggleEditorPage(Page::Mosaic);

        // Quick-access strip (all pages): one-click jump to the operator pages.
        // Same guarded routing as the F-keys (activate, not toggle — clicking
        // the current page's button is a no-op); active page highlighted.
        // Buttons a level cannot use are ABSENT rather than disabled: an
        // operator's strip is one button wide and reads as their machine, not
        // as six locked doors. Each button owns its own SameLine so the strip
        // closes up with no gaps whichever ones are missing.
        {
            bool anyBtn = false;
            // Twice frame height, like the Run strip's Execute: this is the
            // row a gloved hand reaches for to change page, and a text-high
            // button is a target to aim at rather than hit.
            const ImVec2 stripBtn(0.0f, ImGui::GetFrameHeight() * 2.0f);
            const auto pageBtn = [&](const char* label, Page p, bool takeover) {
                if (!CanAccess(p)) return;
                if (anyBtn) ImGui::SameLine();
                anyBtn = true;
                const bool active = (currentPage_ == p);
                if (active)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
                if (ImGui::Button(label, stripBtn) && !active) {
                    if (takeover) ToggleEditorPage(p);
                    else          SwitchPage(p);
                }
                if (active) ImGui::PopStyleColor();
            };
            pageBtn("Run (F1)", Page::Run, true);
            pageBtn("Recipe Editor (F2)", Page::RecipeEdit, true);
            pageBtn("Motion (F3)", Page::Motion, false);
            pageBtn("Scripts (F4)", Page::Scripts, true);
            pageBtn("Alignment (F5)", Page::Align, true);
            // Not pageBtn: opening Placement is not a page switch but a switch
            // WITH a subject (the selected procedure), and with no procedure
            // there is nothing to open — hence the disabled state the other
            // buttons never need.
            if (CanAccess(Page::AlignPlace)) {
                if (anyBtn) ImGui::SameLine();
                anyBtn = true;
                const bool active = (currentPage_ == Page::AlignPlace);
                const bool haveProc =
                    alignSel_ >= 0 &&
                    alignSel_ < static_cast<int>(session_.Aligns().size());
                if (active)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
                ImGui::BeginDisabled(!haveProc);
                if (ImGui::Button("Placement (F6)", stripBtn)) openPlacement();
                ImGui::EndDisabled();
                if (active) ImGui::PopStyleColor();
                if (!haveProc && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Select an alignment procedure first (Alignment, F5)");
            }
            pageBtn("Needle (F7)", Page::Needle, true);
            // No Registration or Gripped object button: both are one-time
            // teach setups, reached from the View menu (or F8 / F9) only, so
            // the strip stays the row of pages an operator changes between
            // during a shift. The Waffle Tray page IS one of those: a fresh
            // tray means a Reset and a glance at which cells are taken.
            pageBtn("Waffle Tray (F10)", Page::Tray, true);
            if (anyBtn) ImGui::Separator();
        }

        // Above the page dispatch, so it is on screen whichever page the move
        // took the operator to — the whole reason it exists.
        DrawMoveStatusBar();

        DrawProductPopups();   // modals opened via the File-menu flags above
        DrawExitConfirmModal();   // confirm before closing the application
        DrawSafetyAlertModal();   // blocking safety alert (highest priority)
        DrawRecipePromptWindow(); // global recipe prompt (any page, any time)
        DrawManualAdjustWindow(); // global manual_adjust jog window
        DrawAlignPickWindow();    // global align_station crosshair pick

        // ---- Editor pages (View menu) replace the whole panel layout until
        // closed via the top-right Close button (back to the previous page).
        if (currentPage_ == Page::WatchEdit || currentPage_ == Page::TransformEdit ||
            currentPage_ == Page::RecipeEdit || currentPage_ == Page::Scripts ||
            currentPage_ == Page::DutView || currentPage_ == Page::Run ||
            currentPage_ == Page::Align || currentPage_ == Page::VisionTeach ||
            currentPage_ == Page::AlignNominal || currentPage_ == Page::AlignPlace ||
            currentPage_ == Page::Needle || currentPage_ == Page::DeviceReg ||
            currentPage_ == Page::Grip || currentPage_ == Page::Tray ||
            currentPage_ == Page::Mosaic || currentPage_ == Page::Version) {
            const char* title = currentPage_ == Page::WatchEdit       ? "IO Watch Editor"
                                : currentPage_ == Page::TransformEdit ? "Transforms Editor"
                                : currentPage_ == Page::RecipeEdit    ? "Recipe Editor"
                                : currentPage_ == Page::Scripts       ? "Scripts"
                                : currentPage_ == Page::DutView       ? "DUT Records"
                                : currentPage_ == Page::Align         ? "Alignment"
                                : currentPage_ == Page::VisionTeach   ? "Vision detection"
                                : currentPage_ == Page::AlignNominal  ? "Nominal target"
                                : currentPage_ == Page::AlignPlace    ? "Place on nominal"
                                : currentPage_ == Page::Needle        ? "Dispense needle"
                                : currentPage_ == Page::DeviceReg     ? "Device registration"
                                : currentPage_ == Page::Grip          ? "Gripped object"
                                : currentPage_ == Page::Tray          ? "Waffle tray"
                                : currentPage_ == Page::Mosaic        ? "Mosaic"
                                : currentPage_ == Page::Version       ? "Version and software update"
                                                                      : "Run";
            // Vision teach and the two alignment teach pages are only ever
            // reached FROM the Alignment page, and a takeover -> takeover switch
            // does not update prevPage_, so Close has to name Align explicitly
            // or it would jump to whatever hardware page happened to be open
            // before the Alignment page was.
            // Vision teach is shared by the alignment stations and the nominals'
            // object detectors, so it carries its own return page rather than
            // naming one here.
            const Page closeTo = (currentPage_ == Page::VisionTeach) ? visionReturnPage_
                                 : (currentPage_ == Page::AlignNominal ||
                                    currentPage_ == Page::AlignPlace)  ? Page::Align
                                                                       : prevPage_;
            ImGui::BeginChild("EditorPage", ImGui::GetContentRegionAvail(),
                              ImGuiChildFlags_Borders);
            ImGui::TextUnformatted(title);
            // No Close when there is nowhere to close TO. An Operator has only
            // the Run page, so closeTo resolves to the page they are already on
            // and SwitchPage returns early — a button that visibly does nothing
            // reads as a broken application rather than as a locked door.
            if (closeTo != currentPage_ && CanAccess(closeTo)) {
                ImGui::SameLine();
                const float closeW = ImGui::CalcTextSize("Close").x +
                                     ImGui::GetStyle().FramePadding.x * 2.0f;
                ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - closeW);
                if (ImGui::Button("Close"))
                    SwitchPage(closeTo);   // dirty recipe edits raise the guard
            }
            ImGui::Separator();
            if (currentPage_ == Page::WatchEdit)          DrawWatchEditPanel();
            else if (currentPage_ == Page::TransformEdit) DrawTransformEditPanel();
            else if (currentPage_ == Page::RecipeEdit)    DrawRecipeEditPanel();
            else if (currentPage_ == Page::Scripts)       DrawScriptsPanel();
            else if (currentPage_ == Page::DutView)       DrawDutViewPanel();
            else if (currentPage_ == Page::Align)         DrawAlignPanel();
            else if (currentPage_ == Page::VisionTeach)   DrawVisionTeachPanel();
            else if (currentPage_ == Page::AlignNominal)  DrawAlignNominalPanel();
            else if (currentPage_ == Page::AlignPlace)    DrawAlignPlacePanel();
            else if (currentPage_ == Page::Needle)        DrawNeedlePanel();
            else if (currentPage_ == Page::DeviceReg)     DrawDeviceRegPanel();
            else if (currentPage_ == Page::Grip)          DrawGripPanel();
            else if (currentPage_ == Page::Tray)          DrawTrayPanel();
            else if (currentPage_ == Page::Mosaic)        DrawMosaicPanel();
            else if (currentPage_ == Page::Version)       DrawVersionPanel();
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
            case Page::Light:     DrawLightListPanel();     break;
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
            case Page::Light:     DrawLightDetailPanel();     break;
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
            case Page::Light:     DrawLightInfoPanel();     break;
            case Page::Motion:    break;   // left panel carries the live axes
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
        if (ScriptDirty())
            ImGui::BulletText("Script editor has unsaved edits");
        if (orch_.Busy())
            ImGui::BulletText("A recipe/script/path move is still running");
        // Velopack's download cannot be cancelled; the process waits for it
        // after the window closes.
        if (updater_.GetStatus().state == AppUpdater::State::Downloading)
            ImGui::BulletText("A software update is downloading - exit waits for it");
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

} // namespace firedog
