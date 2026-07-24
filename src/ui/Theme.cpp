// Theme.cpp
#include "ui/Theme.h"

#include "imgui.h"

namespace firedog {

void Theme::Apply()
{
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 0.0f;   // keep 0 so detached multi-viewport OS windows look right
    s.FrameRounding  = 4.0f;
    s.GrabRounding   = 4.0f;
    s.WindowPadding  = ImVec2(10.0f, 10.0f);

    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]       = ImVec4(0.10f, 0.10f, 0.11f, 1.00f);
    c[ImGuiCol_TitleBgActive]  = ImVec4(0.16f, 0.29f, 0.40f, 1.00f);
    c[ImGuiCol_Header]         = ImVec4(0.16f, 0.29f, 0.40f, 0.80f);
    c[ImGuiCol_HeaderHovered]  = ImVec4(0.20f, 0.36f, 0.50f, 0.90f);
    c[ImGuiCol_Button]         = ImVec4(0.16f, 0.29f, 0.40f, 0.70f);
    c[ImGuiCol_ButtonHovered]  = ImVec4(0.20f, 0.36f, 0.50f, 0.90f);
}

} // namespace firedog
