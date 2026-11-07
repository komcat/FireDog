// RoiFilter.h : a live ROI filter (core ViewFilter) on a camera image view.
//
// The UI half of a ViewFilter: the toolbar that edits one, the per-view
// texture its picture is cached in, and the overlay drawn over the frame. The
// pixel maths is vision/RoiFilterOps (shared with any worker); the data is
// core/CameraSetup.h (shared with recipe commands). DISPLAY ONLY - no mark,
// click or measurement is ever taken on the filtered picture.
//
// Reuse, per view - two App members and up to three calls:
//   ViewFilter       what to show       (persist it in UiSettings with
//                                        LoadViewFilter/SaveViewFilter under the
//                                        view's own prefix - or not at all, for
//                                        a view a run drives)
//   RoiFilterState   the view's texture + scratch (never shared: the texture
//                    IS that view's picture)
//   DrawRoiFilterToolbar -> the two toolbar rows, true when a setting changed
//                           (optional: a view a recipe drives has no toolbar)
//   UpdateRoiFilter      -> rebuilds on a new frame / changed setting only,
//                           returns the RoiOverlay to draw
//   DrawRoiOverlay       -> into any CameraView; MarkPicker takes it as `roi`
//
// Deliberately light: no imgui.h here.
#pragma once

#include "core/CameraSetup.h"   // ViewFilter

#include <cstddef>
#include <vector>

struct ImDrawList;

namespace firedog {

class GlHost;
struct CameraView;

// The frame the filter is computed from: packed RGB8 and its FrameSeq (the
// freshness counter - never `isNew`, which the first consumer eats).
struct RoiFilterFrame {
    const unsigned char* rgb = nullptr;
    std::size_t bytes = 0;        // size of the buffer behind `rgb`
    int  w = 0, h = 0;
    unsigned long long seq = 0;
    bool Valid() const {
        return rgb && w > 0 && h > 0 &&
               bytes >= static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3;
    }
};

// What a view draws, in CAMERA px. `outline` borders [x0,x1) x [y0,y1); a
// non-zero `tex` is the processed picture of exactly [tx0,tx1) x [ty0,ty1).
struct RoiOverlay {
    bool         outline = false;
    unsigned int tex     = 0;
    float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
    float tx0 = 0.0f, ty0 = 0.0f, tx1 = 0.0f, ty1 = 0.0f;
};

// The cache behind one view. Rebuilt only when the frame, the rectangle or the
// filter moved; the ASKED rectangle is the key, the overlay is placed on the
// one the crop actually returned.
struct RoiFilterState {
    unsigned int tex = 0;
    int ax = 0, ay = 0, aw = 0, ah = 0;   // asked
    int gx = 0, gy = 0, gw = 0, gh = 0;   // got (clamped)
    ViewFilter built;
    bool       haveBuilt = false;
    unsigned long long seq = 0;
    std::vector<unsigned char> gray, rgb;
};

// The toolbar, two rows: ROI size, then binary + threshold + Auto (Otsu over
// the ROI, which also switches binary on). Returns true when a setting changed
// - the caller saves. `id` scopes the widgets so two views never share one.
bool DrawRoiFilterToolbar(const char* id, ViewFilter& f, RoiFilterState& st,
                          const RoiFilterFrame& frame, float cx, float cy);

// Bring the view's texture up to date and say what to draw. Cheap when nothing
// moved. Returns no texture rather than a stale one (a picture of the previous
// ROI or threshold must not be stretched over the new one).
RoiOverlay UpdateRoiFilter(GlHost& host, const ViewFilter& f, RoiFilterState& st,
                           const RoiFilterFrame& frame, float cx, float cy);

// Draw the overlay into a view: the texture where it came from, clipped to the
// part of the box that is image, then the border. Draw-list only.
void DrawRoiOverlay(ImDrawList* dl, const CameraView& view, const RoiOverlay& ov);

} // namespace firedog
