// Widgets.cpp : the shared UI leaf helpers.
// windows.h first (lean, no min/max macros) so anything that defines APIENTRY
// later sees it already set; used only for ShellExecuteA (open-in-Explorer).
// This is the ONLY TU in the UI layer that pulls windows.h in — keep it so.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>

#include "ui/Widgets.h"

// The PNG decoder, implemented here because this TU already owns the "talks to
// the outside world" jobs (ShellExecute, the filesystem). stb_image_write's
// implementation lives in CommandsDut.cpp; the two headers are independent.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <cmath>
#include <cstddef>
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

bool LoadPngRGBA(const char* path, std::vector<unsigned char>* rgba,
                 int* width, int* height)
{
    if (!path || !rgba || !width || !height) return false;
    int w = 0, h = 0, ch = 0;
    stbi_uc* px = stbi_load(path, &w, &h, &ch, 4);   // force RGBA
    if (!px) return false;
    rgba->assign(px, px + static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u);
    stbi_image_free(px);
    *width  = w;
    *height = h;
    return true;
}

void DrawWorldMark(ImDrawList* dl, const WorldMarkView& wv, const CameraMark& m,
                   float crossLen)
{
    if (!dl || !m.have || wv.pixelToMmX <= 0.0f || wv.pixelToMmY <= 0.0f) return;
    const ImU32 col = m.col ? m.col : kPtCol;

    // Machine mm -> camera px, through the pose the image centre sits at.
    const float px = static_cast<float>((m.wx - wv.gx) / static_cast<double>(wv.pixelToMmX)) +
                     static_cast<float>(wv.imgW) * 0.5f;
    const float py = static_cast<float>((m.wy - wv.gy) / static_cast<double>(wv.pixelToMmY)) +
                     static_cast<float>(wv.imgH) * 0.5f;
    const ImVec2 s = wv.view.ToScreen(px, py);

    const float pad = 2.0f;
    if (s.x >= wv.boxMin.x - pad && s.x <= wv.boxMax.x + pad &&
        s.y >= wv.boxMin.y - pad && s.y <= wv.boxMax.y + pad) {
        DrawCross(dl, s, col, crossLen, m.label);
        return;
    }

    // Off-screen: an arrow on the edge, pointing at it, with how far away it is.
    const ImVec2 c((wv.boxMin.x + wv.boxMax.x) * 0.5f,
                   (wv.boxMin.y + wv.boxMax.y) * 0.5f);
    float dx = s.x - c.x, dy = s.y - c.y;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-3f) return;
    dx /= len;
    dy /= len;

    // How far along that ray the box edge is, inset so the arrow and its label
    // stay wholly inside. Guard the zero component: a purely horizontal
    // direction never crosses the top or bottom edge.
    const float inset = 30.0f;
    const float halfW = (wv.boxMax.x - wv.boxMin.x) * 0.5f - inset;
    const float halfH = (wv.boxMax.y - wv.boxMin.y) * 0.5f - inset;
    if (halfW <= 0.0f || halfH <= 0.0f) return;
    const float tx = (std::fabs(dx) > 1e-4f) ? halfW / std::fabs(dx) : 1e9f;
    const float ty = (std::fabs(dy) > 1e-4f) ? halfH / std::fabs(dy) : 1e9f;
    const float t  = tx < ty ? tx : ty;
    const ImVec2 tip(c.x + dx * t, c.y + dy * t);

    // A triangle pointing along the ray: tip, then two points swung off the
    // perpendicular behind it.
    const float aLen = 16.0f, aHalf = 9.0f;
    const ImVec2 back(tip.x - dx * aLen, tip.y - dy * aLen);
    const ImVec2 p1(back.x - dy * aHalf, back.y + dx * aHalf);
    const ImVec2 p2(back.x + dy * aHalf, back.y - dx * aHalf);
    dl->AddTriangleFilled(tip, p1, p2, col);

    const double distMM = std::sqrt((m.wx - wv.gx) * (m.wx - wv.gx) +
                                    (m.wy - wv.gy) * (m.wy - wv.gy));
    char lbl[64];
    std::snprintf(lbl, sizeof(lbl), "%s %.2f mm", m.label ? m.label : "", distMM);
    const ImVec2 ts = ImGui::CalcTextSize(lbl);
    // Label behind the arrow, then clamped into the box: near a corner the
    // "behind" position can still fall outside.
    ImVec2 lp(tip.x - dx * (aLen + 6.0f) - ts.x * 0.5f,
              tip.y - dy * (aLen + 6.0f) - ts.y * 0.5f);
    lp.x = fminf(fmaxf(lp.x, wv.boxMin.x + 4.0f), wv.boxMax.x - ts.x - 4.0f);
    lp.y = fminf(fmaxf(lp.y, wv.boxMin.y + 4.0f), wv.boxMax.y - ts.y - 4.0f);
    dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                      ImVec2(lp.x + ts.x + 4.0f, lp.y + ts.y + 2.0f),
                      IM_COL32(0, 0, 0, 170), 3.0f);
    dl->AddText(lp, col, lbl);
}

void DrawMarkIconsInline(const MarkIcon icons[2], int active, float height)
{
    if (!icons || height <= 0.0f) return;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    for (int i = 0; i < 2; ++i) {
        if (!icons[i].tex || icons[i].w <= 0 || icons[i].h <= 0) continue;
        const float w = height * static_cast<float>(icons[i].w) /
                                 static_cast<float>(icons[i].h);
        const ImVec2 p = ImGui::GetCursorScreenPos();
        // 30 % on the one that is not being asked for. Dimming rather than
        // hiding it keeps the pair readable as "1 of 2", which is the thing an
        // operator actually needs to know.
        const ImU32 tint = (i == active) ? IM_COL32(255, 255, 255, 255)
                                         : IM_COL32(255, 255, 255, 77);
        dl->AddImage((ImTextureID)(intptr_t)icons[i].tex, p,
                     ImVec2(p.x + w, p.y + height),
                     ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f), tint);
        ImGui::Dummy(ImVec2(w, height));
        ImGui::SameLine();
    }
}

MarkPickerResult DrawMarkPicker(const char* id, const ImVec2& box, const MarkPicker& mp)
{
    MarkPickerResult out;
    if (!mp.px || !mp.py || !mp.zoom) return out;

    const bool haveImg = mp.tex && mp.imgW > 0 && mp.imgH > 0;
    // The footer row is as tall as the diagrams when there are any, so reserve
    // for them — otherwise the image eats the row they sit on.
    const bool haveIcons = mp.icons[0].tex != 0 || mp.icons[1].tex != 0;
    const float footerH = (haveIcons ? ImGui::GetFrameHeight() * 2.0f
                                     : ImGui::GetFrameHeight()) +
                          ImGui::GetStyle().ItemSpacing.y + 8.0f;
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

    // RIGHT click = drive there. The offset from the image centre in mm is the
    // move that brings the clicked feature under the centre — the same
    // arithmetic a mark uses, and the same convention the Aux window's
    // click-to-move established.
    //
    // The crosshair jumps to the centre with it, because that is where the
    // clicked feature is about to be. Leaving it on its old PIXEL would be
    // actively wrong: after the stage moves, that pixel looks at a different
    // physical place, and a mark taken there would be off by the whole move.
    if (haveImg && hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
        mp.pixelToMmX > 0.0f && mp.pixelToMmY > 0.0f) {
        float cx = 0.0f, cy = 0.0f;
        view.ToCamera(ImGui::GetIO().MousePos, &cx, &cy);
        out.moveRequested = true;
        out.dxMM = static_cast<double>(cx - static_cast<float>(mp.imgW) * 0.5f) *
                   static_cast<double>(mp.pixelToMmX);
        out.dyMM = static_cast<double>(cy - static_cast<float>(mp.imgH) * 0.5f) *
                   static_cast<double>(mp.pixelToMmY);
        *mp.px = static_cast<float>(mp.imgW) * 0.5f;
        *mp.py = static_cast<float>(mp.imgH) * 0.5f;
    }
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

        WorldMarkView wv;
        wv.view       = view;
        wv.boxMin     = origin;
        wv.boxMax     = ImVec2(origin.x + img.x, origin.y + img.y);
        wv.gx         = mp.gx;
        wv.gy         = mp.gy;
        wv.pixelToMmX = mp.pixelToMmX;
        wv.pixelToMmY = mp.pixelToMmY;
        wv.imgW       = mp.imgW;
        wv.imgH       = mp.imgH;

        // Machine mm -> camera px, for the midpoint line below.
        const auto toPixel = [&](double wx, double wy, float* px, float* py) {
            *px = static_cast<float>((wx - mp.gx) / static_cast<double>(mp.pixelToMmX)) +
                  static_cast<float>(mp.imgW) * 0.5f;
            *py = static_cast<float>((wy - mp.gy) / static_cast<double>(mp.pixelToMmY)) +
                  static_cast<float>(mp.imgH) * 0.5f;
        };

        if (mp.havePose && mp.pixelToMmX > 0.0f && mp.pixelToMmY > 0.0f) {
            // Marks that have left the field of view become edge arrows rather
            // than vanishing — the stage moves between marks, and a mark you
            // cannot see is indistinguishable from one you never took.
            for (int i = 0; i < mp.markCount; ++i) {
                if (!mp.marks) break;
                DrawWorldMark(dl, wv, mp.marks[i], 16.0f);
            }
            if (mp.midpointLabel && mp.marks && mp.markCount >= 2 &&
                mp.marks[0].have && mp.marks[1].have) {
                float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
                toPixel(mp.marks[0].wx, mp.marks[0].wy, &ax, &ay);
                toPixel(mp.marks[1].wx, mp.marks[1].wy, &bx, &by);
                dl->AddLine(view.ToScreen(ax, ay), view.ToScreen(bx, by), kMidCol, 1.0f);
                CameraMark mid;
                mid.wx    = (mp.marks[0].wx + mp.marks[1].wx) * 0.5;
                mid.wy    = (mp.marks[0].wy + mp.marks[1].wy) * 0.5;
                mid.have  = true;
                mid.label = mp.midpointLabel;
                mid.col   = kMidCol;
                DrawWorldMark(dl, wv, mid, 22.0f);
            }
        }

        DrawCross(dl, view.ToScreen(*mp.px, *mp.py), kCurCol, 22.0f, nullptr);
        dl->PopClipRect();
    }

    // Footer: the step diagrams first, then the hint and the zoom control. They
    // sit here rather than over the image because over the image they cover the
    // feature being marked.
    DrawMarkIconsInline(mp.icons, mp.activeMark, ImGui::GetFrameHeight() * 2.0f);
    ImGui::TextDisabled("left click places, RIGHT click drives there, arrows nudge "
                        "1 px (Shift = 10), wheel zooms");
    ImGui::SameLine();
    ImGui::PushID(id);
    ImGui::SetNextItemWidth(110.0f);
    ImGui::SliderFloat("##zoom", mp.zoom, 1.0f, 12.0f, "x%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) *mp.zoom = 1.0f;
    ImGui::PopID();
    return out;
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
