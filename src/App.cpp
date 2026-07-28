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
    std::string lastProduct;
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
        lastProduct      = ui.lastProduct;
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
    savedPos_ = LoadSavedPositions();   // config fallback (no product)
    if (!lastProduct.empty() && !LoadProduct(lastProduct))
        FD_WARN("product", "could not reload last product '%s'", lastProduct.c_str());
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
                if (ImGui::MenuItem("Motion",   nullptr, currentPage_ == Page::Motion))
                    currentPage_ = Page::Motion;
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
        DrawProductPopups();   // modals opened via the File-menu flags above

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
            case Page::Motion: DrawMotionListPanel(); break;
            default:           DrawLeftPanel();       break;
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
            case Page::Motion: DrawMotionDetailPanel(); break;
            default:           DrawMiddlePanel();       break;
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
            case Page::Motion: DrawMotionInfoPanel(); break;
            default:           DrawRightPanel();      break;
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
    SaveUiSettings();   // remember as last product for the next startup
    FD_INFO("product", "loaded '%s'", product_.name.c_str());
    return true;
}

void App::NewProduct(const char* name)
{
    ProductManifest m = CreateProduct(name);
    if (!m.Loaded()) return;
    product_ = std::move(m);
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
        } else if (e.role == "recipes" || e.role == "programs") {
            // placeholder data — leave the uid-stub file as-is
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

    // ---- Saved / taught positions (persisted; Move drives via Workflow) ----
    ImGui::Spacing();
    ImGui::Separator();
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

    // Push the UI's velocity to the device once it is connected, so the combo
    // and the hardware agree even before the user touches it.
    if (dev->IsConnected() && !velPushed_[key]) {
        workflow_.SetJogVelocity(di, vels[idx]);
        velPushed_[key] = true;
    }

    ImGui::TextUnformatted("Velocity (mm/s)");
    ImGui::SetNextItemWidth(-FLT_MIN);
    char prev[24];
    if (velCustomSet_[key] > 0.0)
        std::snprintf(prev, sizeof(prev), "%g (custom)", velCustomSet_[key]);
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
