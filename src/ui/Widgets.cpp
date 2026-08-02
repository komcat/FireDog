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

CameraView MakeCameraView(const ImVec2& origin, const ImVec2& size,
                          int imgW, int imgH, float focusX, float focusY, float zoom)
{
    CameraView v;
    v.origin = origin;
    v.size   = size;
    v.imgW   = imgW;
    v.imgH   = imgH;
    if (imgW <= 0 || imgH <= 0) { v.scale = 1.0f; return v; }

    const float fit = fmaxf(size.x / static_cast<float>(imgW),
                            size.y / static_cast<float>(imgH));
    v.scale = fit * (zoom > 0.0f ? zoom : 1.0f);
    v.vw    = size.x / v.scale;
    v.vh    = size.y / v.scale;

    // Centre on the focus point, but never past the image edge: half a window
    // wider than the image would letterbox, so that case centres instead.
    const auto clamp = [](float c, float half, float extent) {
        return (half * 2.0f >= extent) ? extent * 0.5f
                                       : fminf(fmaxf(c, half), extent - half);
    };
    v.left = clamp(focusX, v.vw * 0.5f, static_cast<float>(imgW)) - v.vw * 0.5f;
    v.top  = clamp(focusY, v.vh * 0.5f, static_cast<float>(imgH)) - v.vh * 0.5f;
    return v;
}

namespace {

constexpr ImU32 kRefCol = IM_COL32(80, 230, 120, 140);   // image-centre reference
constexpr ImU32 kCurCol = IM_COL32(255, 200, 60, 255);   // the live crosshair
constexpr ImU32 kPtCol  = IM_COL32(120, 200, 255, 255);  // a latched mark
constexpr ImU32 kMidCol = IM_COL32(255, 90, 90, 255);    // their midpoint

// A labelled cross. `len` is what distinguishes the live crosshair from the
// latched marks at a glance, so the overlay needs no legend.
void DrawCross(ImDrawList* dl, const ImVec2& p, ImU32 col, float len, const char* label)
{
    dl->AddLine(ImVec2(p.x - len, p.y), ImVec2(p.x + len, p.y), col, 2.0f);
    dl->AddLine(ImVec2(p.x, p.y - len), ImVec2(p.x, p.y + len), col, 2.0f);
    dl->AddCircle(p, 5.0f, col, 0, 2.0f);
    if (label && label[0]) dl->AddText(ImVec2(p.x + 8.0f, p.y + 6.0f), col, label);
}

} // namespace

void DrawMarkPicker(const char* id, const ImVec2& box, const MarkPicker& mp)
{
    if (!mp.px || !mp.py || !mp.zoom) return;

    const bool haveImg = mp.tex && mp.imgW > 0 && mp.imgH > 0;
    const float footerH = ImGui::GetFrameHeightWithSpacing() + 8.0f;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 img = box;
    img.y = img.y > footerH + 40.0f ? img.y - footerH : 40.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const CameraView view = MakeCameraView(origin, img, mp.imgW, mp.imgH,
                                           *mp.px, *mp.py, *mp.zoom);

    if (haveImg) {
        ImGui::Image((ImTextureID)(intptr_t)mp.tex, img, view.Uv0(), view.Uv1());
    } else {
        dl->AddRectFilled(origin, ImVec2(origin.x + img.x, origin.y + img.y),
                          IM_COL32(18, 18, 22, 255));
        const char* msg = "no camera feed - cannot mark a point";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(origin.x + (img.x - ts.x) * 0.5f,
                           origin.y + (img.y - ts.y) * 0.5f),
                    IM_COL32(220, 140, 140, 255), msg);
        ImGui::Dummy(img);
    }

    ImGui::SetCursorScreenPos(origin);
    const bool clicked = ImGui::InvisibleButton(id, img);
    const bool hovered = ImGui::IsItemHovered();
    if (haveImg && clicked) view.ToCamera(ImGui::GetIO().MousePos, mp.px, mp.py);
    // Wheel zoom applies from the NEXT frame (the view above is already built) —
    // a frame of latency nobody can see.
    if (haveImg && hovered && ImGui::GetIO().MouseWheel != 0.0f) {
        *mp.zoom *= (ImGui::GetIO().MouseWheel > 0.0f) ? 1.25f : 0.8f;
        *mp.zoom = fminf(fmaxf(*mp.zoom, 1.0f), 12.0f);
    }
    // One press = one CAMERA pixel (hold to repeat, Shift for 10). Taken only
    // while the mouse is over this window, never globally: an arrow aimed at a
    // name field beside the image has to stay there.
    const bool keysHere =
        ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows |
                               ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
    if (haveImg && keysHere) {
        const float nudge = ImGui::GetIO().KeyShift ? 10.0f : 1.0f;
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))  *mp.px -= nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) *mp.px += nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))    *mp.py -= nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))  *mp.py += nudge;
        *mp.px = fminf(fmaxf(*mp.px, 0.0f), static_cast<float>(mp.imgW));
        *mp.py = fminf(fmaxf(*mp.py, 0.0f), static_cast<float>(mp.imgH));
    }

    if (haveImg) {
        // Zoomed in, a mark (or the image centre) can sit outside the visible
        // window entirely — clip rather than paint over the surrounding layout.
        dl->PushClipRect(origin, ImVec2(origin.x + img.x, origin.y + img.y), true);

        const ImVec2 ctr = view.ToScreen(static_cast<float>(mp.imgW) * 0.5f,
                                         static_cast<float>(mp.imgH) * 0.5f);
        dl->AddLine(ImVec2(origin.x, ctr.y), ImVec2(origin.x + img.x, ctr.y), kRefCol, 1.0f);
        dl->AddLine(ImVec2(ctr.x, origin.y), ImVec2(ctr.x, origin.y + img.y), kRefCol, 1.0f);

        // Machine mm -> camera px, through the pose given for THIS frame.
        const auto toPixel = [&](double wx, double wy, float* px, float* py) {
            *px = static_cast<float>((wx - mp.gx) / static_cast<double>(mp.pixelToMmX)) +
                  static_cast<float>(mp.imgW) * 0.5f;
            *py = static_cast<float>((wy - mp.gy) / static_cast<double>(mp.pixelToMmY)) +
                  static_cast<float>(mp.imgH) * 0.5f;
        };

        if (mp.havePose && mp.pixelToMmX > 0.0f && mp.pixelToMmY > 0.0f) {
            for (int i = 0; i < mp.markCount; ++i) {
                if (!mp.marks || !mp.marks[i].have) continue;
                float px = 0.0f, py = 0.0f;
                toPixel(mp.marks[i].wx, mp.marks[i].wy, &px, &py);
                DrawCross(dl, view.ToScreen(px, py), kPtCol, 16.0f, mp.marks[i].label);
            }
            if (mp.midpointLabel && mp.marks && mp.markCount >= 2 &&
                mp.marks[0].have && mp.marks[1].have) {
                float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f, cx = 0.0f, cy = 0.0f;
                toPixel(mp.marks[0].wx, mp.marks[0].wy, &ax, &ay);
                toPixel(mp.marks[1].wx, mp.marks[1].wy, &bx, &by);
                toPixel((mp.marks[0].wx + mp.marks[1].wx) * 0.5,
                        (mp.marks[0].wy + mp.marks[1].wy) * 0.5, &cx, &cy);
                dl->AddLine(view.ToScreen(ax, ay), view.ToScreen(bx, by), kMidCol, 1.0f);
                DrawCross(dl, view.ToScreen(cx, cy), kMidCol, 22.0f, mp.midpointLabel);
            }
        }

        DrawCross(dl, view.ToScreen(*mp.px, *mp.py), kCurCol, 22.0f, nullptr);
        dl->PopClipRect();
    }

    ImGui::TextDisabled("click to place (recentres), arrow keys nudge 1 px "
                        "(Shift = 10 px), wheel zooms");
    ImGui::SameLine();
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(110.0f);
    ImGui::SliderFloat("##zoom", mp.zoom, 1.0f, 12.0f, "x%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) *mp.zoom = 1.0f;
    ImGui::PopID();
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
