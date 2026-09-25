// PageOverview.cpp : the Overview page — the hardware-status left panel with
// its DUT block and Display controls, the vision middle panel, the right
// panel, and the shared log viewer. Definitions of App members — see App.h.
//
// DrawLogPanel is shared: the Overview lower strip and the Run page's log band
// both call it.
#include "App.h"

#include "imgui.h"

#include "ui/Theme.h"
#include "ui/UiSettings.h"
#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/DutSession.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <cfloat>
#include <cstdio>

namespace firedog {

namespace {
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

// Active-DUT status + open/close controls (Overview left panel). The operator
// path: type/scan a serial, Open (load-or-create). Recipes/scripts drive the
// same DutSession through the dut_* commands. Design: docs/dut-design.md.
void App::DrawDutBlock()
{
    const DutSession::Info di = DutSession::Instance().GetInfo();
    // A dot beside the label says whether a record is open: green with a DUT,
    // yellow without one - the one state an operator about to press Execute
    // needs to see from across the bench. Grey until a product is loaded.
    {
        const float  h  = ImGui::GetTextLineHeight();
        const ImVec2 p  = ImGui::GetCursorScreenPos();
        const ImU32  col = !di.productSet ? IM_COL32(110, 110, 115, 255)
                         : di.open        ? IM_COL32(70, 200, 100, 255)
                                          : IM_COL32(235, 190, 40, 255);
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + h * 0.5f, p.y + h * 0.5f),
                                                    h * 0.32f, col);
        ImGui::Dummy(ImVec2(h, h));
        ImGui::SameLine();
    }
    ImGui::TextUnformatted("DUT");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(!di.productSet ? "no product loaded"
                          : di.open      ? "DUT record open"
                                         : "no DUT open - enter a serial");
    // Technician and up. This block is shared with the Run page, so on an
    // Operator's station it is the ONE control that leaves the application
    // entirely — an Explorer window, and from there the whole filesystem. The
    // record it opens is written for them, not by them.
    if (di.productSet && CanUse(UserRole::Technician)) {
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
        // The serial is what an operator checks from across the bench: twice
        // the body size, shrunk to fit the row when a long one would overflow.
        {
            const float base = ImGui::GetStyle().FontSizeBase;
            const float maxW = ImGui::GetContentRegionAvail().x;
            float size = base * 2.0f;
            ImGui::PushFont(nullptr, size);
            const float tw = ImGui::CalcTextSize(di.serial.c_str()).x;
            ImGui::PopFont();
            if (tw > maxW && tw > 0.0f && maxW > 0.0f) {
                size *= maxW / tw;
                if (size < base * 0.75f) size = base * 0.75f;   // stay legible
            }
            ImGui::PushFont(nullptr, size);
            ImGui::TextUnformatted(di.serial.c_str());
            ImGui::PopFont();
        }
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
        ImGui::TextColored(LevelColor(e.level), "%s  %-5s  %s",
                           e.stamp, LevelName(e.level), e.text.c_str());
    }
    if (logAutoScroll_) ImGui::SetScrollY(ImGui::GetScrollMaxY());
    ImGui::EndChild();
}

} // namespace firedog
