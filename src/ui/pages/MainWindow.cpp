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

#include <cstdio>

namespace firedog {

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
                if (ImGui::MenuItem("Alignment", nullptr, currentPage_ == Page::Align))
                    ToggleEditorPage(Page::Align);
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
        DrawAlignPickWindow();    // global align_station crosshair pick

        // ---- Editor pages (View menu) replace the whole panel layout until
        // closed via the top-right Close button (back to the previous page).
        if (currentPage_ == Page::WatchEdit || currentPage_ == Page::TransformEdit ||
            currentPage_ == Page::RecipeEdit || currentPage_ == Page::Scripts ||
            currentPage_ == Page::DutView || currentPage_ == Page::Run ||
            currentPage_ == Page::Align || currentPage_ == Page::VisionTeach) {
            const char* title = currentPage_ == Page::WatchEdit       ? "IO Watch Editor"
                                : currentPage_ == Page::TransformEdit ? "Transforms Editor"
                                : currentPage_ == Page::RecipeEdit    ? "Recipe Editor"
                                : currentPage_ == Page::Scripts       ? "Scripts"
                                : currentPage_ == Page::DutView       ? "DUT Records"
                                : currentPage_ == Page::Align         ? "Alignment"
                                : currentPage_ == Page::VisionTeach   ? "Vision detection"
                                                                      : "Run";
            // Vision teach is only ever reached FROM the Alignment page, and a
            // takeover -> takeover switch does not update prevPage_, so Close
            // has to name Align explicitly or it would jump to whatever hardware
            // page happened to be open before the Alignment page was.
            const Page closeTo = (currentPage_ == Page::VisionTeach) ? Page::Align
                                                                     : prevPage_;
            ImGui::BeginChild("EditorPage", ImGui::GetContentRegionAvail(),
                              ImGuiChildFlags_Borders);
            ImGui::TextUnformatted(title);
            ImGui::SameLine();
            const float closeW = ImGui::CalcTextSize("Close").x +
                                 ImGui::GetStyle().FramePadding.x * 2.0f;
            ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - closeW);
            if (ImGui::Button("Close"))
                SwitchPage(closeTo);   // dirty recipe edits raise the guard
            ImGui::Separator();
            if (currentPage_ == Page::WatchEdit)          DrawWatchEditPanel();
            else if (currentPage_ == Page::TransformEdit) DrawTransformEditPanel();
            else if (currentPage_ == Page::RecipeEdit)    DrawRecipeEditPanel();
            else if (currentPage_ == Page::Scripts)       DrawScriptsPanel();
            else if (currentPage_ == Page::DutView)       DrawDutViewPanel();
            else if (currentPage_ == Page::Align)         DrawAlignPanel();
            else if (currentPage_ == Page::VisionTeach)   DrawVisionTeachPanel();
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

} // namespace firedog
