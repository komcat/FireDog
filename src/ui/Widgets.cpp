// Widgets.cpp : the shared UI leaf helpers.
// windows.h first (lean, no min/max macros) so anything that defines APIENTRY
// later sees it already set; used only for ShellExecuteA (open-in-Explorer).
// This is the ONLY TU in the UI layer that pulls windows.h in — keep it so.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>

#include "ui/Widgets.h"

#include "vision/VisionModel.h"   // the patch helpers: LoadGrayPgm + MakeCornerTemplate

// The PNG decoder, implemented here because this TU already owns the "talks to
// the outside world" jobs (ShellExecute, the filesystem). stb_image_write's
// implementation lives in CommandsDut.cpp; the two headers are independent.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
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

void DrawCameraViewImage(ImDrawList* dl, unsigned int tex, const CameraView& view)
{
    const ImVec2 boxMax(view.origin.x + view.size.x, view.origin.y + view.size.y);
    dl->AddRectFilled(view.origin, boxMax, IM_COL32(18, 18, 22, 255));
    if (!tex || view.imgW <= 0 || view.imgH <= 0) return;

    // The visible window intersected with the image, in camera px: at zoom
    // >= 1 this is the window itself, below 1 the image inside the letterbox.
    const float x0 = fmaxf(view.left, 0.0f);
    const float y0 = fmaxf(view.top, 0.0f);
    const float x1 = fminf(view.left + view.vw, static_cast<float>(view.imgW));
    const float y1 = fminf(view.top + view.vh, static_cast<float>(view.imgH));
    if (x1 <= x0 || y1 <= y0) return;

    dl->AddImage((ImTextureID)(intptr_t)tex, view.ToScreen(x0, y0), view.ToScreen(x1, y1),
                 ImVec2(x0 / static_cast<float>(view.imgW), y0 / static_cast<float>(view.imgH)),
                 ImVec2(x1 / static_cast<float>(view.imgW), y1 / static_cast<float>(view.imgH)));
}

void DrawZoomBadge(ImDrawList* dl, const CameraView& view, float zoom)
{
    char txt[48];
    std::snprintf(txt, sizeof(txt), "zoom x%.2f", zoom);
    const ImVec2 ts  = ImGui::CalcTextSize(txt);
    const ImVec2 pad(6.0f, 3.0f);
    const ImVec2 p0(view.origin.x + 6.0f, view.origin.y + 6.0f);
    const ImVec2 p1(p0.x + ts.x + pad.x * 2.0f, p0.y + ts.y + pad.y * 2.0f);
    dl->AddRectFilled(p0, p1, IM_COL32(0, 0, 0, 150), 3.0f);
    dl->AddText(ImVec2(p0.x + pad.x, p0.y + pad.y), IM_COL32(235, 235, 235, 255), txt);
}

namespace {

constexpr ImU32 kRefCol = IM_COL32(80, 230, 120, 140);   // image-centre reference
constexpr ImU32 kCurCol = IM_COL32(255, 200, 60, 255);   // the live crosshair
constexpr ImU32 kPtCol  = IM_COL32(120, 200, 255, 255);  // a latched mark
constexpr ImU32 kMidCol = IM_COL32(255, 90, 90, 255);    // their midpoint
constexpr ImU32 kEnvCol = IM_COL32(255, 90, 90, 110);    // the span they bound
constexpr ImU32 kToolCol = IM_COL32(255, 190, 70, 255);  // a registered tool tip
constexpr ImU32 kToolBox = IM_COL32(255, 190, 70, 190);  // ...and its footprint
// Grab radius for the footprint box's corners, screen px. Generous: the
// corner is a handle on a live camera image, not a window edge.
constexpr float kBoxGrabPx = 12.0f;
// Below this on-screen half-size a DrawWorldBox outline is not a size, it is a
// slightly fatter dot beside the cross's own 5 px ring - and two glyphs that
// mean different things at the same scale is worse than one.
constexpr float kBoxMinPx = 7.0f;
// A read-only outline the picker is handed with no colour of its own.
constexpr ImU32 kObjBox = IM_COL32(90, 220, 235, 200);

// A labelled cross. `len` is what distinguishes the live crosshair from the
// latched marks at a glance, so the overlay needs no legend. `ring` is off for
// the live crosshair: the circle is the size of the feature being aimed at, so
// it hides exactly the pixels the operator is trying to line the cross up with.
void DrawCross(ImDrawList* dl, const ImVec2& p, ImU32 col, float len, const char* label,
               bool ring = true)
{
    dl->AddLine(ImVec2(p.x - len, p.y), ImVec2(p.x + len, p.y), col, 2.0f);
    dl->AddLine(ImVec2(p.x, p.y - len), ImVec2(p.x, p.y + len), col, 2.0f);
    if (ring) dl->AddCircle(p, 5.0f, col, 0, 2.0f);
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

namespace {

// A world box on screen: its centre, half-sizes and spin, from the anchor and
// the offset. ONE place, because DrawWorldBox draws from it and the picker's
// drag handling hit-tests and edits against it - a second copy of this
// arithmetic is how a corner handle ends up beside the corner it moves.
//
// mm -> screen goes through pixelToMm on each axis SEPARATELY, because the two
// calibrations differ: a square drawn with one of them comes out a rectangle.
// The anchor offset is applied in SCREEN space, after the same conversion and
// the same spin as the corners, so the point stays on the rim it was measured
// at whatever the axes' scales.
struct WorldBoxGeom {
    ImVec2 c{};                 // centre, screen
    float  rx = 0.0f, ry = 0.0f; // half-sizes, screen px
    float  cs = 1.0f, sn = 0.0f; // the spin
    float  sx = 0.0f, sy = 0.0f; // screen px per mm, each axis
    // Box frame (px) -> screen, and back.
    ImVec2 Turn(float x, float y) const { return ImVec2(x * cs - y * sn, x * sn + y * cs); }
    ImVec2 Unturn(float x, float y) const { return ImVec2(x * cs + y * sn, -x * sn + y * cs); }
    ImVec2 Corner(int k) const {
        const float kx = (k == 1 || k == 2) ? 1.0f : -1.0f;
        const float ky = (k >= 2) ? 1.0f : -1.0f;
        const ImVec2 t = Turn(kx * rx, ky * ry);
        return ImVec2(c.x + t.x, c.y + t.y);
    }
};

bool ResolveWorldBox(const WorldMarkView& wv, double ax, double ay, double adx, double ady,
                     double wMm, double lMm, double deg, WorldBoxGeom* g)
{
    if (wMm <= 0.0 || lMm <= 0.0 || wv.pixelToMmX <= 0.0f || wv.pixelToMmY <= 0.0f)
        return false;
    g->sx = wv.view.scale / wv.pixelToMmX;
    g->sy = wv.view.scale / wv.pixelToMmY;
    g->rx = static_cast<float>(wMm) * 0.5f * g->sx;
    g->ry = static_cast<float>(lMm) * 0.5f * g->sy;

    // The anchor, through the same projection DrawWorldMark uses - a second
    // copy of it would put the outline off the cross it belongs to.
    const float px = static_cast<float>((ax - wv.gx) / static_cast<double>(wv.pixelToMmX)) +
                     static_cast<float>(wv.imgW) * 0.5f;
    const float py = static_cast<float>((ay - wv.gy) / static_cast<double>(wv.pixelToMmY)) +
                     static_cast<float>(wv.imgH) * 0.5f;
    const ImVec2 a = wv.view.ToScreen(px, py);

    const float th = static_cast<float>(deg) * 3.14159265358979f / 180.0f;
    g->cs = cosf(th);
    g->sn = sinf(th);
    // The centre: the anchor less its offset, the offset turned as the box is.
    const ImVec2 od = g->Turn(static_cast<float>(adx) * g->sx, static_cast<float>(ady) * g->sy);
    g->c = ImVec2(a.x - od.x, a.y - od.y);
    return true;
}

} // namespace

void DrawWorldBox(ImDrawList* dl, const WorldMarkView& wv, double ax, double ay,
                  double adx, double ady, double wMm, double lMm, double deg,
                  unsigned int col, float handlePx)
{
    WorldBoxGeom g;
    if (!dl || !ResolveWorldBox(wv, ax, ay, adx, ady, wMm, lMm, deg, &g)) return;
    if (g.rx < kBoxMinPx && g.ry < kBoxMinPx) return;

    // Wholly outside the box: nothing to draw. Tested on the turned box's
    // bounding radius, so a corner that swings into view is not culled.
    const float rr = sqrtf(g.rx * g.rx + g.ry * g.ry);
    if (g.c.x + rr < wv.boxMin.x || g.c.x - rr > wv.boxMax.x ||
        g.c.y + rr < wv.boxMin.y || g.c.y - rr > wv.boxMax.y) return;

    // Unlike the cross, this has EXTENT, and a box wider than the field of view
    // would otherwise paint across whatever sits beside the image. Clipped to
    // the image box, which is what it is a statement about.
    dl->PushClipRect(wv.boxMin, wv.boxMax, true);
    const ImVec2 q[4] = {g.Corner(0), g.Corner(1), g.Corner(2), g.Corner(3)};
    dl->AddQuad(q[0], q[1], q[2], q[3], col, 1.5f);
    // Corner handles, so a box that CAN be dragged looks it - and a read-only
    // one does not invite a drag it will not answer.
    if (handlePx > 0.0f)
        for (const ImVec2& k : q)
            dl->AddRectFilled(ImVec2(k.x - handlePx, k.y - handlePx),
                              ImVec2(k.x + handlePx, k.y + handlePx), col);
    dl->PopClipRect();
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

void DrawWorldBand(ImDrawList* dl, const WorldMarkView& wv, const WorldBand& b)
{
    if (!dl || !b.have || wv.pixelToMmX <= 0.0f || wv.pixelToMmY <= 0.0f) return;
    if (b.rMax <= 0.0 || b.aMax <= b.aMin) return;

    // Machine mm -> screen, the same projection DrawWorldMark uses.
    const auto toScreen = [&](double wx, double wy) {
        const float px = static_cast<float>((wx - wv.gx) / static_cast<double>(wv.pixelToMmX)) +
                         static_cast<float>(wv.imgW) * 0.5f;
        const float py = static_cast<float>((wy - wv.gy) / static_cast<double>(wv.pixelToMmY)) +
                         static_cast<float>(wv.imgH) * 0.5f;
        return wv.view.ToScreen(px, py);
    };

    const ImU32 line = b.col ? b.col : IM_COL32(90, 230, 130, 200);
    const ImU32 fill = (line & 0x00FFFFFFu) | (40u << 24);

    const double rIn  = b.rMin > 0.0 ? b.rMin : 0.0;
    const double rOut = b.rMax;
    const double a0   = b.aMin * 3.14159265358979323846 / 180.0;
    const double a1   = b.aMax * 3.14159265358979323846 / 180.0;

    // Enough segments that the arc reads as an arc at the zoom the pick window
    // uses, and few enough that a band spanning 90 degrees is not thousands of
    // quads. The arcs are nearly straight in a small field of view anyway.
    int steps = static_cast<int>((b.aMax - b.aMin) * 1.5) + 16;
    if (steps > 96) steps = 96;

    // Anything this far outside the box is not going to be seen; skipping it
    // keeps a centre 50 mm away from producing quads with vast coordinates.
    const float cullPad = (wv.boxMax.x - wv.boxMin.x) * 4.0f + 1000.0f;
    const auto  offBox  = [&](const ImVec2& p) {
        return p.x < wv.boxMin.x - cullPad || p.x > wv.boxMax.x + cullPad ||
               p.y < wv.boxMin.y - cullPad || p.y > wv.boxMax.y + cullPad;
    };

    std::vector<ImVec2> inner, outer;
    inner.reserve(static_cast<std::size_t>(steps) + 1);
    outer.reserve(static_cast<std::size_t>(steps) + 1);
    for (int i = 0; i <= steps; ++i) {
        const double t = a0 + (a1 - a0) * static_cast<double>(i) / static_cast<double>(steps);
        const double c = std::cos(t), s = std::sin(t);
        inner.push_back(toScreen(b.cx + rIn  * c, b.cy + rIn  * s));
        outer.push_back(toScreen(b.cx + rOut * c, b.cy + rOut * s));
    }

    for (int i = 0; i < steps; ++i) {
        const std::size_t k = static_cast<std::size_t>(i);
        if (offBox(inner[k]) && offBox(outer[k]) &&
            offBox(inner[k + 1]) && offBox(outer[k + 1])) continue;
        dl->AddQuadFilled(inner[k], outer[k], outer[k + 1], inner[k + 1], fill);
    }
    // The two arcs bright, the two limit rays dimmer: which way to move the
    // crosshair is a question about distance far more often than about angle.
    dl->AddPolyline(inner.data(), steps + 1, line, 0, 1.5f);
    dl->AddPolyline(outer.data(), steps + 1, line, 0, 1.5f);
    dl->AddLine(inner.front(), outer.front(), line, 1.0f);
    dl->AddLine(inner.back(),  outer.back(),  line, 1.0f);
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

const char* StepIconFile(const char* type)
{
    if (!type || !*type) return nullptr;

    // Exact matches first, prefix families below.
    struct Row { const char* type; const char* file; };
    static constexpr Row kExact[] = {
        // A relative move and a move to a taught place are different acts —
        // one is "from wherever we are", the other "to a known point" — and
        // telling those two apart at a glance is the whole reason a step list
        // gets icons at all.
        {"move_relative",        "move_relative"},
        {"move_relative_global", "move_relative"},
        // A block of steps rather than one step. Only `group` — giving `loop`
        // and `if` the same glyph would say "these rows are the same kind of
        // thing", which is the opposite of what the column is for.
        {"group",                "GROUP"},
        // A parallel block is a block of steps too — what differs is HOW its
        // rows run, and the row text says "parallel". Reusing the glyph beats
        // naming a picture nobody drew.
        {"parallel",             "GROUP"},
        // The machine has stopped and is waiting for a person.
        {"prompt",               "prompt"},
        {"prompt_input",         "prompt"},
        // A pneumatic slide has its own two glyphs. It IS a synthetic device
        // over the IO fleet (one output pin, two sensor pins), but which WAY it
        // is about to travel is the thing an operator scanning the list wants,
        // and the shared IO glyph cannot say it.
        {"extend_slide",         "Extend"},
        {"retract_slide",        "Retract"},
        {"set_output",           "IO"},
        {"pulse_output",         "IO"},
        {"wait_input",           "IO"},
        {"clear_latch",          "IO"},
        // Structure: a loop and an if each get their OWN glyph (and neither
        // shares GROUP's), for the reason given above.
        {"loop",                 "Loop_command"},
        {"if",                   "IF icon"},
        // The frame- and tray-relative moves take their family's glyph rather
        // than move_default: WHERE the target comes from (a taught frame, a
        // tray cell) is what sets them apart from a move to a saved position.
        // Listed here because the move_ prefix below would claim them.
        {"move_local",           "align command"},
        {"move_to_nominal",      "align command"},
        {"move_to_cell",         "tray command"},
        {"move_to_active_cell",  "tray command"},
    };
    for (const Row& r : kExact)
        if (std::strcmp(r.type, type) == 0) return r.file;

    // Prefix families: everything that drives an axis, and everything that
    // writes the part's record. Matched by PREFIX so a command added later to
    // either family is covered without an edit here — and without silently
    // naming a picture nobody drew.
    if (std::strncmp(type, "move_", 5) == 0) return "move_default";
    if (std::strncmp(type, "dut_",  4) == 0) return "data";
    if (std::strncmp(type, "align_",   6) == 0) return "align command";
    if (std::strncmp(type, "nominal_", 8) == 0) return "align command";
    if (std::strncmp(type, "tray_",    5) == 0) return "tray command";
    return nullptr;
}

void DrawStepIconInline(const MarkIcon* icon)
{
    const float  h = ImGui::GetTextLineHeight();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(h, h));   // reserves the column whether or not it draws
    if (icon && icon->tex)
        ImGui::GetWindowDrawList()->AddImage((ImTextureID)(intptr_t)icon->tex, p,
                                             ImVec2(p.x + h, p.y + h));
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
}

bool IconButton(const MarkIcon* icon, const char* label, const ImVec2& size)
{
    if (!icon || !icon->tex) return ImGui::Button(label, size);

    const ImGuiStyle& st = ImGui::GetStyle();
    const float  ih = ImGui::GetTextLineHeight();
    const ImVec2 ts = ImGui::CalcTextSize(label);
    const float  contentW = ih + st.ItemInnerSpacing.x + ts.x;

    ImVec2 sz = size;
    if (sz.x <= 0.0f) sz.x = contentW + st.FramePadding.x * 2.0f;
    if (sz.y <= 0.0f) sz.y = ImGui::GetFrameHeight();

    // An EMPTY-labelled button with both pieces drawn over it: Button() centres
    // ONE label and has no two-part form. The id still comes from `label`, so a
    // caller that flips "Save" to "Save*" behaves exactly as it did before.
    char id[128];
    std::snprintf(id, sizeof(id), "##%s", label);
    const ImVec2 p       = ImGui::GetCursorScreenPos();
    const bool   clicked = ImGui::Button(id, sz);

    // GetColorU32 folds in the style alpha, so both pieces dim together inside
    // a BeginDisabled block — an icon that stayed bright there would read as
    // the one live control on a dead row.
    ImDrawList* dl   = ImGui::GetWindowDrawList();
    const ImU32 tint = ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
    const float x    = p.x + (sz.x - contentW) * 0.5f;
    const float iy   = p.y + (sz.y - ih) * 0.5f;
    dl->AddImage((ImTextureID)(intptr_t)icon->tex, ImVec2(x, iy),
                 ImVec2(x + ih, iy + ih), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f), tint);
    dl->AddText(ImVec2(x + ih + st.ItemInnerSpacing.x, p.y + (sz.y - ts.y) * 0.5f),
                ImGui::GetColorU32(ImGuiCol_Text), label);
    return clicked;
}

bool IconSmallButton(const MarkIcon* icon, const char* label)
{
    if (!icon || !icon->tex) return ImGui::SmallButton(label);
    // What SmallButton itself does: the ordinary button with its vertical
    // frame padding taken away.
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,
                        ImVec2(ImGui::GetStyle().FramePadding.x, 0.0f));
    const bool clicked = IconButton(icon, label);
    ImGui::PopStyleVar();
    return clicked;
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
        // Draw-list, not ImGui::Image: below zoom 1 the frame is letterboxed
        // and only the part of the box that IS image may sample the texture.
        DrawCameraViewImage(dl, mp.tex, view);
        ImGui::Dummy(img);
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

    // ---- The tool's footprint box, and its corner handles -------------------
    // Resolved BEFORE the crosshair click below, because a press that grabbed a
    // corner must not also place a mark: the box is dragged with the same button
    // that marks a feature, and on a live camera image the operator cannot see
    // which one they hit.
    //
    // The box is CENTRED on the tip, every frame, and that is the whole design.
    // It is not a rectangle you position; it is the tool's perimeter, and the
    // tool is wherever the registration says. So a corner drag can only change
    // the SIZE - there is no way to leave the box describing a place the tool
    // is not.
    const bool  haveTool = haveImg && mp.toolHave && mp.havePose &&
                           mp.pixelToMmX > 0.0f && mp.pixelToMmY > 0.0f;
    ImVec2      toolScr(0.0f, 0.0f);
    bool        toolInBox = false;
    float       boxHalfW = 0.0f, boxHalfH = 0.0f;
    const bool  wasDragging = mp.boxDragging && *mp.boxDragging;
    bool        boxGrabbed  = wasDragging;
    if (haveTool) {
        const float tpx = static_cast<float>((mp.toolWx - mp.gx) /
                                             static_cast<double>(mp.pixelToMmX)) +
                          static_cast<float>(mp.imgW) * 0.5f;
        const float tpy = static_cast<float>((mp.toolWy - mp.gy) /
                                             static_cast<double>(mp.pixelToMmY)) +
                          static_cast<float>(mp.imgH) * 0.5f;
        toolScr   = view.ToScreen(tpx, tpy);
        toolInBox = toolScr.x >= origin.x && toolScr.x <= origin.x + img.x &&
                    toolScr.y >= origin.y && toolScr.y <= origin.y + img.y;

        if (mp.boxWmm && mp.boxHmm && toolInBox) {
            // mm -> screen px: camera px per mm, times the view's magnification.
            boxHalfW = 0.5f * *mp.boxWmm / mp.pixelToMmX * view.scale;
            boxHalfH = 0.5f * *mp.boxHmm / mp.pixelToMmY * view.scale;
        }
        // The corners answer the mouse only for a caller that owns the size.
        if (mp.boxWmm && mp.boxHmm && mp.boxDragging && toolInBox) {
            // The mouse in the BOX's frame: un-spun about the tip, so a turned
            // box is grabbed at its real corners and a drag reads its real
            // width and length rather than the screen-axis distances.
            const ImVec2 m0 = ImGui::GetIO().MousePos;
            const float  th = -mp.boxDeg * 3.14159265358979f / 180.0f;
            const float  cs = cosf(th), sn = sinf(th);
            const float  ux = m0.x - toolScr.x, uy = m0.y - toolScr.y;
            const float  dx = fabsf(ux * cs - uy * sn), dy = fabsf(ux * sn + uy * cs);
            const bool   onCorner = fabsf(dx - boxHalfW) <= kBoxGrabPx &&
                                    fabsf(dy - boxHalfH) <= kBoxGrabPx;
            if (*mp.boxDragging) {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    // Both halves follow the cursor: the corner IS the size.
                    // Floored rather than allowed to reach zero, so a box
                    // dragged shut can still be grabbed and opened again.
                    *mp.boxWmm = fmaxf(0.02f, 2.0f * dx / view.scale * mp.pixelToMmX);
                    *mp.boxHmm = fmaxf(0.02f, 2.0f * dy / view.scale * mp.pixelToMmY);
                } else {
                    *mp.boxDragging = false;
                }
            } else if (hovered && onCorner &&
                       ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                *mp.boxDragging = true;
                boxGrabbed      = true;
            }
            if (onCorner || *mp.boxDragging)
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNWSE);
        }
    }

    // ---- Editable outlines: drag the body to move, a corner to resize -------
    // Resolved before the crosshair click for the reason the tool box is. A
    // body drag keeps the size and moves the box - the anchor POINT stays where
    // it was measured, so what changes is the offset between them. A corner
    // drag holds the OPPOSITE corner and pulls this one to the object's edge,
    // which changes both the size and, since the centre moved, the offset.
    // Everything is done in the box's own frame in screen px and converted
    // back to mm by the same per-axis scale the box was drawn with.
    if (haveImg && mp.havePose && mp.pixelToMmX > 0.0f && mp.pixelToMmY > 0.0f) {
        WorldMarkView wvd;
        wvd.view       = view;
        wvd.boxMin     = origin;
        wvd.boxMax     = ImVec2(origin.x + img.x, origin.y + img.y);
        wvd.gx         = mp.gx;
        wvd.gy         = mp.gy;
        wvd.pixelToMmX = mp.pixelToMmX;
        wvd.pixelToMmY = mp.pixelToMmY;
        wvd.imgW       = mp.imgW;
        wvd.imgH       = mp.imgH;
        for (int i = 0; i < mp.boxCount && mp.boxes; ++i) {
            const MarkPicker::WorldBox& b = mp.boxes[i];
            if (!b.editW || !b.editL || !b.editAdx || !b.editAdy || !b.drag) continue;
            WorldBoxGeom g;
            if (!ResolveWorldBox(wvd, b.ax, b.ay, b.adx, b.ady, b.wMm, b.lMm, b.deg, &g))
                continue;
            const bool wasDrag = *b.drag >= 0;
            if (wasDrag) boxGrabbed = true;   // the release is the click ImGui reports

            // What the mouse is over: a corner first (they sit on the body),
            // then the body.
            const ImVec2 m = ImGui::GetIO().MousePos;
            int over = -1;
            for (int k = 0; k < 4 && over < 0; ++k) {
                const ImVec2 q = g.Corner(k);
                if (fabsf(m.x - q.x) <= kBoxGrabPx && fabsf(m.y - q.y) <= kBoxGrabPx) over = k;
            }
            if (over < 0) {
                const ImVec2 l = g.Unturn(m.x - g.c.x, m.y - g.c.y);
                if (fabsf(l.x) <= g.rx && fabsf(l.y) <= g.ry) over = 4;
            }

            if (wasDrag) {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    if (*b.drag == 4) {
                        // The box moves by the mouse's delta; the anchor does
                        // not, so the offset shrinks by the same amount.
                        const ImVec2 d = ImGui::GetIO().MouseDelta;
                        const ImVec2 dl_ = g.Unturn(d.x, d.y);
                        *b.editAdx -= static_cast<double>(dl_.x / g.sx);
                        *b.editAdy -= static_cast<double>(dl_.y / g.sy);
                    } else {
                        // The opposite corner is fixed; the box is whatever
                        // rectangle spans from it to the mouse. Floored rather
                        // than allowed to reach zero, so a box dragged shut can
                        // still be grabbed and opened again.
                        const int   k  = *b.drag;
                        const float kx = (k == 1 || k == 2) ? 1.0f : -1.0f;
                        const float ky = (k >= 2) ? 1.0f : -1.0f;
                        const ImVec2 o(-kx * g.rx, -ky * g.ry);             // box frame
                        const ImVec2 l = g.Unturn(m.x - g.c.x, m.y - g.c.y); // the mouse
                        const float  minPx = 0.01f;
                        const float hx = fmaxf(fabsf(l.x - o.x) * 0.5f, minPx * g.sx);
                        const float hy = fmaxf(fabsf(l.y - o.y) * 0.5f, minPx * g.sy);
                        // The new centre, in the box frame: where it moved to
                        // is what the anchor offset loses.
                        const float ncx = o.x + kx * hx, ncy = o.y + ky * hy;
                        *b.editW   = static_cast<double>(2.0f * hx / g.sx);
                        *b.editL   = static_cast<double>(2.0f * hy / g.sy);
                        *b.editAdx -= static_cast<double>(ncx / g.sx);
                        *b.editAdy -= static_cast<double>(ncy / g.sy);
                    }
                } else {
                    *b.drag = -1;
                }
            } else if (hovered && over >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                *b.drag    = over;
                boxGrabbed = true;
            }
            const int cue = (*b.drag >= 0) ? *b.drag : over;
            if (cue == 4)      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            else if (cue >= 0) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNWSE);
        }
    }

    // A click that belonged to the box is not a mark. `wasDragging` covers the
    // RELEASE, which is the edge ImGui reports as the click.
    if (haveImg && clicked && !boxGrabbed) view.ToCamera(ImGui::GetIO().MousePos, mp.px, mp.py);

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
        *mp.zoom = fminf(fmaxf(*mp.zoom, mp.zoomMin), mp.zoomMax);
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
            // The centre of every LATCHED mark, and the box they bound. Two
            // marks make it a midpoint and four make it the centre of a ring;
            // the arithmetic is the same average either way, which is the whole
            // reason four marks needed no second code path here.
            int nHave = 0;
            for (int i = 0; i < mp.markCount && mp.marks; ++i)
                if (mp.marks[i].have) ++nHave;
            if (mp.midpointLabel && mp.marks && nHave >= 2) {
                double cwx = 0.0, cwy = 0.0;
                ImVec2 lo(FLT_MAX, FLT_MAX), hi(-FLT_MAX, -FLT_MAX);
                ImVec2 sa(0.0f, 0.0f), sb(0.0f, 0.0f);
                int    seen = 0;
                for (int i = 0; i < mp.markCount; ++i) {
                    if (!mp.marks[i].have) continue;
                    cwx += mp.marks[i].wx;
                    cwy += mp.marks[i].wy;
                    float mx = 0.0f, my = 0.0f;
                    toPixel(mp.marks[i].wx, mp.marks[i].wy, &mx, &my);
                    const ImVec2 sp = view.ToScreen(mx, my);
                    lo.x = fminf(lo.x, sp.x); lo.y = fminf(lo.y, sp.y);
                    hi.x = fmaxf(hi.x, sp.x); hi.y = fmaxf(hi.y, sp.y);
                    if (seen == 0)      sa = sp;
                    else if (seen == 1) sb = sp;
                    ++seen;
                }
                cwx /= seen;
                cwy /= seen;
                // The footprint the two corners bound. Both marks are usually
                // off screen together at working zoom, and a bare diagonal
                // between two edge arrows says nothing about how big the part
                // is; the box does, and it is what the operator is really
                // checking when the midpoint looks wrong. Machine-axis aligned
                // unless `envelopeDeg` spins it — this is the span the marks
                // define, not the taught frame.
                const ImVec2 c((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f);
                // Screen space, not camera px: the rotation stays a rotation
                // only where the scale is the same on both axes, and the view's
                // magnification is (px->screen is uniform, mm->px is not).
                const float th = mp.envelopeDeg * 3.14159265358979f / 180.0f;
                const float cs = cosf(th), sn = sinf(th);
                const auto spin = [&](float x, float y) {
                    const float ux = x - c.x, uy = y - c.y;
                    return ImVec2(c.x + ux * cs - uy * sn, c.y + ux * sn + uy * cs);
                };
                dl->AddQuad(spin(lo.x, lo.y), spin(hi.x, lo.y),
                            spin(hi.x, hi.y), spin(lo.x, hi.y), kEnvCol, 1.0f);
                // The diagonal only means something for a PAIR: across four
                // marks it is a line between two arbitrary corners of the ring.
                if (seen == 2) dl->AddLine(sa, sb, kMidCol, 1.0f);
                CameraMark mid;
                mid.wx    = cwx;
                mid.wy    = cwy;
                mid.have  = true;
                mid.label = mp.midpointLabel;
                mid.col   = kMidCol;
                DrawWorldMark(dl, wv, mid, 22.0f);
            }
        }

        // The tool tip and its footprint, on top of the marks (it is what the
        // operator is comparing them against) and under the live crosshair
        // (which is what they are placing).
        if (haveTool) {
            if (toolInBox && mp.boxWmm && mp.boxHmm && boxHalfW > 0.0f) {
                const ImVec2 lo(toolScr.x - boxHalfW, toolScr.y - boxHalfH);
                const ImVec2 hi(toolScr.x + boxHalfW, toolScr.y + boxHalfH);
                // Spun about the tip in screen space, the way the envelope is:
                // the rotation stays a rotation only where the scale is the
                // same on both axes.
                const float th = mp.boxDeg * 3.14159265358979f / 180.0f;
                const float cs = cosf(th), sn = sinf(th);
                const auto spin = [&](float x, float y) {
                    const float ux = x - toolScr.x, uy = y - toolScr.y;
                    return ImVec2(toolScr.x + ux * cs - uy * sn, toolScr.y + ux * sn + uy * cs);
                };
                const ImVec2 q[4] = {spin(lo.x, lo.y), spin(hi.x, lo.y),
                                     spin(hi.x, hi.y), spin(lo.x, hi.y)};
                dl->AddQuad(q[0], q[1], q[2], q[3], kToolBox, 1.6f);
                // Corner handles, so a box that CAN be dragged looks it - and a
                // read-only one does not invite a drag it will not answer.
                if (mp.boxDragging) {
                    const float g = 5.0f;
                    for (const ImVec2& k : q)
                        dl->AddRectFilled(ImVec2(k.x - g, k.y - g),
                                          ImVec2(k.x + g, k.y + g), kToolBox);
                }
                char bl[64];
                std::snprintf(bl, sizeof(bl), "%.3f x %.3f mm", *mp.boxWmm, *mp.boxHmm);
                const ImVec2 ts = ImGui::CalcTextSize(bl);
                // Above the box's highest corner, whichever one that is once
                // it has turned.
                float top = q[0].y;
                for (const ImVec2& k : q) top = fminf(top, k.y);
                const ImVec2 at(toolScr.x + boxHalfW - ts.x, top - ts.y - 4.0f);
                dl->AddRectFilled(ImVec2(at.x - 4.0f, at.y - 2.0f),
                                  ImVec2(at.x + ts.x + 4.0f, at.y + ts.y + 2.0f),
                                  IM_COL32(0, 0, 0, 165), 3.0f);
                dl->AddText(at, kToolBox, bl);
            }
            CameraMark tm;
            tm.wx    = mp.toolWx;
            tm.wy    = mp.toolWy;
            tm.have  = true;
            tm.label = mp.toolLabel;
            tm.col   = kToolCol;
            DrawWorldMark(dl, wv, tm, 14.0f);
        }

        // The caller's read-only outlines, under the live crosshair like
        // everything else that is not being placed.
        for (int i = 0; i < mp.boxCount && mp.boxes; ++i) {
            const MarkPicker::WorldBox& b = mp.boxes[i];
            const bool editable = b.editW && b.editL && b.editAdx && b.editAdy && b.drag;
            DrawWorldBox(dl, wv, b.ax, b.ay, b.adx, b.ady, b.wMm, b.lMm, b.deg,
                         b.col ? b.col : kObjBox, editable ? 5.0f : 0.0f);
        }

        DrawCross(dl, view.ToScreen(*mp.px, *mp.py), kCurCol, 22.0f, nullptr, false);
        dl->PopClipRect();
        // Last, so nothing paints over it: the one overlay allowed on the
        // picture, because it says what scale the picture is at.
        DrawZoomBadge(dl, view, *mp.zoom);
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
    ImGui::SliderFloat("##zoom", mp.zoom, mp.zoomMin, mp.zoomMax, "x%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) *mp.zoom = 1.0f;
    ImGui::PopID();
    return out;
}

// ---- What a detection model is looking for ---------------------------------

std::string VisionModelPatchKey(const VisionModel& m)
{
    char buf[224];
    if (m.method == "ncc") {
        // The anchor is IN the key: it decides where the cross goes, and an
        // operator dragging it wants to watch the thumbnail follow.
        std::snprintf(buf, sizeof(buf), "|ncc|%s|%dx%d|%d|%.3f,%.3f",
                      m.templateFile.c_str(), m.ncc.tmplW, m.ncc.tmplH,
                      m.ncc.useAnchor ? 1 : 0, m.ncc.anchorOffsetX, m.ncc.anchorOffsetY);
    } else if (m.method == "corner" && m.corner.mode == 1) {
        std::snprintf(buf, sizeof(buf), "|corner|%d|%d|%.2f|%.2f", m.corner.corner,
                      m.corner.polarity, m.corner.sizePx, m.corner.smoothPx);
    } else {
        std::snprintf(buf, sizeof(buf), "|none|%s", m.method.c_str());
    }
    // The pre-process chain too: a re-teach writes the SAME filename at the same
    // size, so without this a patch cut through a new threshold would keep
    // showing the picture it replaced — on this page, on the station card and in
    // the manual-pick window at the same time.
    return m.uid + buf + "|" + VisionFilterSummary(m);
}

bool VisionModelPatch(const VisionModel& m, const std::string& visionDir,
                      std::vector<unsigned char>* gray, int* w, int* h,
                      float* rx, float* ry)
{
    if (!gray || !w || !h) return false;
    gray->clear();
    *w = *h = 0;

    if (m.method == "ncc") {
        if (m.templateFile.empty()) return false;
        if (!LoadGrayPgm(visionDir + "/" + m.templateFile, gray, w, h)) return false;
        // The anchor is an offset from the patch CENTRE, so the reported point
        // is the centre plus it — and the centre itself when there is none.
        if (rx) *rx = static_cast<float>(*w) * 0.5f +
                      (m.ncc.useAnchor ? static_cast<float>(m.ncc.anchorOffsetX) : 0.0f);
        if (ry) *ry = static_cast<float>(*h) * 0.5f +
                      (m.ncc.useAnchor ? static_cast<float>(m.ncc.anchorOffsetY) : 0.0f);
        return true;
    }
    if (m.method == "corner" && m.corner.mode == 1) {
        int n = 0;
        *gray = MakeCornerTemplate(m.corner, &n);
        if (n <= 0 || gray->size() < static_cast<std::size_t>(n) * static_cast<std::size_t>(n)) {
            gray->clear();
            return false;
        }
        *w = *h = n;
        // The side is forced even so the vertex lands exactly on the centre;
        // that IS the reported point, which is why this method needs no anchor.
        if (rx) *rx = static_cast<float>(n) * 0.5f;
        if (ry) *ry = static_cast<float>(n) * 0.5f;
        return true;
    }
    return false;
}

ImVec2 DrawPatchThumb(const PatchTex& pt, float maxW, float maxH)
{
    if (!pt.tex || pt.w <= 0 || pt.h <= 0 || maxW <= 0.0f || maxH <= 0.0f)
        return ImVec2(0.0f, 0.0f);

    const float sc = fminf(maxW / static_cast<float>(pt.w),
                           maxH / static_cast<float>(pt.h));
    const ImVec2 sz(static_cast<float>(pt.w) * sc, static_cast<float>(pt.h) * sc);
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImGui::Image((ImTextureID)(intptr_t)pt.tex, sz);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRect(o, ImVec2(o.x + sz.x, o.y + sz.y), IM_COL32(120, 120, 130, 255));
    // Same amber as the live crosshair in the pick window, on purpose: the
    // operator is being asked to put THAT cross on THIS point.
    const ImU32 col = IM_COL32(255, 200, 60, 255);
    const ImVec2 c(o.x + pt.rx * sc, o.y + pt.ry * sc);
    dl->AddLine(ImVec2(c.x - 9.0f, c.y), ImVec2(c.x + 9.0f, c.y), col, 1.5f);
    dl->AddLine(ImVec2(c.x, c.y - 9.0f), ImVec2(c.x, c.y + 9.0f), col, 1.5f);
    dl->AddCircle(c, 3.5f, col, 0, 1.5f);
    return sz;
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

void DrawBigLabel(const char* text, const ImVec2& box, const ImVec4& color)
{
    const float base = ImGui::GetStyle().FontSizeBase;
    float size = box.y * 0.62f;
    if (size < base) size = base;
    ImGui::PushFont(nullptr, size);
    ImVec2 ts = ImGui::CalcTextSize(text);
    ImGui::PopFont();
    const float maxW = box.x - 6.0f;
    if (ts.x > maxW && ts.x > 0.0f) {
        size = size * maxW / ts.x;
        if (size < base) size = base;
        ImGui::PushFont(nullptr, size);
        ts = ImGui::CalcTextSize(text);
        ImGui::PopFont();
    }
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    ImGui::PushFont(nullptr, size);
    ImGui::SetCursorScreenPos(ImVec2(pos.x + (box.x - ts.x) * 0.5f, pos.y + (box.y - ts.y) * 0.5f));
    ImGui::TextColored(color, "%s", text);
    ImGui::PopFont();
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

const SavedPos* TaughtPosePicker(const char* id, const SavedPosMap& all,
                                 const std::string& device)
{
    const SavedPos* picked = nullptr;
    ImGui::SetNextItemWidth(140.0f);
    if (!ImGui::BeginCombo(id, "or taught...")) return nullptr;

    const auto it = all.find(device);
    if (it == all.end() || it->second.empty()) {
        ImGui::TextDisabled("no taught positions for '%s'", device.c_str());
    } else {
        bool any = false;
        for (const SavedPos& sp : it->second) {
            if (sp.hidden) continue;
            any = true;
            if (ImGui::Selectable(sp.name)) picked = &sp;
        }
        if (!any) ImGui::TextDisabled("only system positions taught");
    }
    ImGui::EndCombo();
    return picked;
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
