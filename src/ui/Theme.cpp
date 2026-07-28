// Theme.cpp
#include "ui/Theme.h"

#include "imgui.h"
#include "implot.h"

namespace firedog {

namespace {

// The header/button accent family, derived from one base + one hover color so
// each theme states its accent once and stays internally consistent.
void Accent(ImVec4* c, const ImVec4& base, const ImVec4& hover)
{
    c[ImGuiCol_TitleBgActive] = ImVec4(base.x, base.y, base.z, 1.00f);
    c[ImGuiCol_Header]        = ImVec4(base.x, base.y, base.z, 0.80f);
    c[ImGuiCol_HeaderHovered] = ImVec4(hover.x, hover.y, hover.z, 0.90f);
    c[ImGuiCol_HeaderActive]  = hover;
    c[ImGuiCol_Button]        = ImVec4(base.x, base.y, base.z, 0.70f);
    c[ImGuiCol_ButtonHovered] = ImVec4(hover.x, hover.y, hover.z, 0.90f);
    c[ImGuiCol_ButtonActive]  = hover;
}

void FireDogDark(ImGuiStyle& s)   // the original FireDog look (startup default)
{
    ImGui::StyleColorsDark(&s);
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.11f, 1.00f);
    Accent(c, ImVec4(0.16f, 0.29f, 0.40f, 1.0f), ImVec4(0.20f, 0.36f, 0.50f, 1.0f));
}

void DeepBlue(ImGuiStyle& s)      // navy backgrounds, brighter blue accents
{
    ImGui::StyleColorsDark(&s);
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]         = ImVec4(0.06f, 0.08f, 0.12f, 1.00f);
    c[ImGuiCol_ChildBg]          = ImVec4(0.08f, 0.10f, 0.15f, 1.00f);
    c[ImGuiCol_PopupBg]          = ImVec4(0.08f, 0.10f, 0.15f, 0.98f);
    c[ImGuiCol_FrameBg]          = ImVec4(0.12f, 0.17f, 0.26f, 1.00f);
    c[ImGuiCol_FrameBgHovered]   = ImVec4(0.16f, 0.23f, 0.35f, 1.00f);
    c[ImGuiCol_FrameBgActive]    = ImVec4(0.19f, 0.28f, 0.42f, 1.00f);
    c[ImGuiCol_CheckMark]        = ImVec4(0.30f, 0.70f, 1.00f, 1.00f);
    c[ImGuiCol_SliderGrab]       = ImVec4(0.28f, 0.60f, 0.90f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = ImVec4(0.35f, 0.72f, 1.00f, 1.00f);
    Accent(c, ImVec4(0.13f, 0.32f, 0.52f, 1.0f), ImVec4(0.18f, 0.42f, 0.66f, 1.0f));
}

void Graphite(ImGuiStyle& s)      // neutral grays, amber accents (industrial)
{
    ImGui::StyleColorsDark(&s);
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]         = ImVec4(0.12f, 0.12f, 0.12f, 1.00f);
    c[ImGuiCol_ChildBg]          = ImVec4(0.14f, 0.14f, 0.14f, 1.00f);
    c[ImGuiCol_FrameBg]          = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
    c[ImGuiCol_FrameBgHovered]   = ImVec4(0.26f, 0.26f, 0.26f, 1.00f);
    c[ImGuiCol_FrameBgActive]    = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
    c[ImGuiCol_CheckMark]        = ImVec4(0.95f, 0.65f, 0.15f, 1.00f);
    c[ImGuiCol_SliderGrab]       = ImVec4(0.85f, 0.55f, 0.12f, 1.00f);
    c[ImGuiCol_SliderGrabActive] = ImVec4(1.00f, 0.70f, 0.20f, 1.00f);
    Accent(c, ImVec4(0.45f, 0.31f, 0.10f, 1.0f), ImVec4(0.60f, 0.42f, 0.14f, 1.0f));
}

void Light(ImGuiStyle& s)         // bright-room / shop-floor visibility
{
    ImGui::StyleColorsLight(&s);
    ImVec4* c = s.Colors;
    Accent(c, ImVec4(0.55f, 0.68f, 0.82f, 1.0f), ImVec4(0.44f, 0.60f, 0.78f, 1.0f));
}

struct ThemeDef { const char* name; void (*apply)(ImGuiStyle&); };
constexpr ThemeDef kThemes[] = {
    {"FireDog Dark", &FireDogDark},
    {"Deep Blue",    &DeepBlue},
    {"Graphite",     &Graphite},
    {"Light",        &Light},
};
constexpr int kThemeCount = static_cast<int>(sizeof(kThemes) / sizeof(kThemes[0]));

} // namespace

int Theme::Count() { return kThemeCount; }

const char* Theme::Name(int index)
{
    if (index < 0 || index >= kThemeCount) index = 0;
    return kThemes[index].name;
}

void Theme::Apply(int index)
{
    if (index < 0 || index >= kThemeCount) index = 0;

    ImGuiStyle& s = ImGui::GetStyle();
    const float fontSize = s.FontSizeBase;   // owned by GlHost — survive the reset
    s = ImGuiStyle();                        // fresh baseline so themes don't stack
    s.FontSizeBase = fontSize;

    kThemes[index].apply(s);

    // Shared metrics (every theme).
    s.WindowRounding = 0.0f;   // keep 0 so detached multi-viewport OS windows look right
    s.FrameRounding  = 4.0f;
    s.GrabRounding   = 4.0f;
    s.WindowPadding  = ImVec2(10.0f, 10.0f);

    // Re-derive ImPlot's auto colors from the new ImGui style so the Aux chart
    // follows the theme (matters most for Light).
    ImPlot::StyleColorsAuto(&ImPlot::GetStyle());
}

} // namespace firedog
