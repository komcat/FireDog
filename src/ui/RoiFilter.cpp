// RoiFilter.cpp : the live ROI filter's UI half - see RoiFilter.h.
#include "ui/RoiFilter.h"

#include "imgui.h"

#include "ui/GlHost.h"
#include "ui/Widgets.h"            // CameraView
#include "vision/RoiFilterOps.h"   // ViewFilterRect / ViewFilterGray / auto threshold

#include <cmath>
#include <cstdint>

namespace firedog {

bool DrawRoiFilterToolbar(const char* id, ViewFilter& f, RoiFilterState& st,
                          const RoiFilterFrame& frame, float cx, float cy)
{
    bool save = false;
    ImGui::PushID(id);

    // Row 1 - the ROI.
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("ROI");
    static const char* kRoiLbl[4] = {"off", "1/3", "1/2", "2/3"};
    for (int i = 0; i < 4; ++i) {
        ImGui::SameLine();
        if (ImGui::RadioButton(kRoiLbl[i], &f.roiMode, i)) save = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A square on the crosshair, this fraction of the frame's\n"
                          "short side. The binary view is applied inside it.");

    // Row 2 - the binary threshold.
    if (ImGui::Checkbox("binary", &f.binary)) save = true;
    ImGui::SameLine();
    ImGui::BeginDisabled(!f.binary);
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderInt("##thr", &f.thr, 0, 255, "threshold %d");
    if (ImGui::IsItemDeactivatedAfterEdit()) save = true;
    ImGui::EndDisabled();
    ImGui::SameLine();
    // Otsu over what the ROI holds now: the level that best splits it into two
    // populations - the part and its background, when the ROI is on the part.
    ImGui::BeginDisabled(!frame.Valid());
    if (ImGui::Button("Auto")) {
        const int t = ViewFilterAutoThreshold(frame.rgb, frame.w, frame.h, f.roiMode, cx, cy,
                                              st.gray);
        if (t >= 0) {
            f.thr    = t;
            f.binary = true;
            save     = true;
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Pick the threshold that best splits the ROI into dark and\n"
                          "bright (Otsu), and switch the binary view on.");

    ImGui::PopID();
    return save;
}

RoiOverlay UpdateRoiFilter(GlHost& host, const ViewFilter& f, RoiFilterState& st,
                           const RoiFilterFrame& frame, float cx, float cy)
{
    RoiOverlay ov;
    if (!frame.Valid()) return ov;

    const FocusRoi r = ViewFilterRect(f.roiMode, frame.w, frame.h, cx, cy);
    ov.outline = f.roiMode > 0;
    ov.x0 = static_cast<float>(r.x);
    ov.y0 = static_cast<float>(r.y);
    ov.x1 = static_cast<float>(r.x + r.w);
    ov.y1 = static_cast<float>(r.y + r.h);
    if (!f.binary) return ov;   // an outline needs no picture

    // Rebuilt on a new frame or a changed ROI/filter only - a 2/3 ROI on a
    // 5 MP frame is ~2 MP, cheap once per frame but not worth doing at no change.
    const bool sameKey = st.tex && st.haveBuilt && st.built == f && r.x == st.ax &&
                         r.y == st.ay && r.w == st.aw && r.h == st.ah;
    if (!sameKey || frame.seq != st.seq) {
        const FocusRoi got = ViewFilterGray(frame.rgb, frame.w, frame.h, f, r, st.gray);
        if (got.w <= 0 || got.h <= 0) return ov;
        const std::size_t n = static_cast<std::size_t>(got.w) * static_cast<std::size_t>(got.h);
        st.rgb.resize(n * 3);
        for (std::size_t i = 0; i < n; ++i)
            st.rgb[i * 3] = st.rgb[i * 3 + 1] = st.rgb[i * 3 + 2] = st.gray[i];
        st.tex = host.EnsureTextureRGB(st.tex, got.w, got.h, st.rgb.data());
        st.ax = r.x; st.ay = r.y; st.aw = r.w; st.ah = r.h;
        st.gx = got.x; st.gy = got.y; st.gw = got.w; st.gh = got.h;
        st.built     = f;
        st.haveBuilt = true;
        st.seq       = frame.seq;
    }
    if (st.tex && st.gw > 0 && st.gh > 0) {
        ov.tex = st.tex;
        ov.tx0 = static_cast<float>(st.gx);
        ov.ty0 = static_cast<float>(st.gy);
        ov.tx1 = static_cast<float>(st.gx + st.gw);
        ov.ty1 = static_cast<float>(st.gy + st.gh);
    }
    return ov;
}

void DrawRoiOverlay(ImDrawList* dl, const CameraView& view, const RoiOverlay& ov)
{
    if (!dl || view.imgW <= 0 || view.imgH <= 0) return;

    // The visible window intersected with the image, in camera px.
    const float wx0 = fmaxf(view.left, 0.0f);
    const float wy0 = fmaxf(view.top, 0.0f);
    const float wx1 = fminf(view.left + view.vw, static_cast<float>(view.imgW));
    const float wy1 = fminf(view.top + view.vh, static_cast<float>(view.imgH));

    if (ov.tex && ov.tx1 > ov.tx0 && ov.ty1 > ov.ty0) {
        const float vx0 = fmaxf(wx0, ov.tx0), vy0 = fmaxf(wy0, ov.ty0);
        const float vx1 = fminf(wx1, ov.tx1), vy1 = fminf(wy1, ov.ty1);
        if (vx1 > vx0 && vy1 > vy0) {
            const float rw = ov.tx1 - ov.tx0, rh = ov.ty1 - ov.ty0;
            dl->AddImage((ImTextureID)(intptr_t)ov.tex,   // C-style cast: void*/ImU64 ImTextureID
                         view.ToScreen(vx0, vy0), view.ToScreen(vx1, vy1),
                         ImVec2((vx0 - ov.tx0) / rw, (vy0 - ov.ty0) / rh),
                         ImVec2((vx1 - ov.tx0) / rw, (vy1 - ov.ty0) / rh));
        }
    }
    if (ov.outline && ov.x1 > ov.x0 && ov.y1 > ov.y0) {
        dl->PushClipRect(view.origin,
                         ImVec2(view.origin.x + view.size.x, view.origin.y + view.size.y), true);
        dl->AddRect(view.ToScreen(ov.x0, ov.y0), view.ToScreen(ov.x1, ov.y1),
                    IM_COL32(255, 200, 60, 230), 0.0f, 0, 1.5f);
        dl->PopClipRect();
    }
}

} // namespace firedog
