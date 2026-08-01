// Widgets.cpp : the shared UI leaf helpers.
// windows.h first (lean, no min/max macros) so anything that defines APIENTRY
// later sees it already set; used only for ShellExecuteA (open-in-Explorer).
// This is the ONLY TU in the UI layer that pulls windows.h in — keep it so.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>

#include "ui/Widgets.h"

#include <cmath>
#include <cstdio>
#include <filesystem>

namespace firedog {

int KeiRangeIndexFor(double amps)
{
    int best = 6;
    if (amps <= 0.0) return best;
    double bestErr = 1e300;
    for (int i = 0; i < kKeiRangeCount; ++i) {
        const double err = std::fabs(std::log10(amps / kKeiRanges[i].amps));
        if (err < bestErr) { bestErr = err; best = i; }
    }
    return best;
}

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

void DeviceStatusLine(ConnState s, const char* name) {
    ImGui::TextColored(StateColor(s), "%s", StateGlyph(s));
    ImGui::SameLine();
    ImGui::Text("%-12s %s", name, StateText(s));
}

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

void OpenFolderInExplorer(const std::string& dir)
{
    if (dir.empty()) return;
    std::error_code ec;
    const std::filesystem::path abs = std::filesystem::absolute(dir, ec);
    if (ec) return;
    std::filesystem::create_directories(abs, ec);
    ShellExecuteA(nullptr, "open", abs.string().c_str(), nullptr, nullptr,
                  SW_SHOWNORMAL);
}

} // namespace firedog
