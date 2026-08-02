// PageVisionTeach.cpp : the Vision Detection teach page (a full-page takeover,
// reached only from an alignment station card).
//
// Tune a detection model against the live camera until it finds the feature
// reliably. Definitions of App members — see App.h.
//
// Why a takeover page and not a panel on the Alignment page: the job needs a BIG
// live image (an ROI is dragged on it, a template is cut out of it, and the
// result is judged by looking at the overlay), and the Alignment page's middle
// column is a narrow strip of station cards. Why not a floating window either:
// teaching a detector is a task you do FOR a station and then leave, the
// operator is not meant to be jogging underneath it, and the other authoring
// surfaces in this app (recipes, transforms, watch, scripts) are all takeovers —
// one exit convention beats two.
//
// It shares the crosshair pick window's camera-view mapping
// (ui/Widgets.h CameraView) so a pixel means the same thing in both surfaces.
//
// Threading, and why it is not a violation of "no blocking call on the UI
// thread": `Detect once` is a bounded one-shot compute with no wait in it — the
// same class as MeasureFocus and the texture upload. The RELIABILITY run, which
// sleeps between runs and must be stoppable, goes to the engine as a one-step
// ad-hoc recipe, exactly like the Autofocus buttons.
#include "App.h"

#include "imgui.h"

#include "core/Log.h"
#include "core/Product.h"          // GenerateUid
#include "ui/Widgets.h"
#include "vision/HalconDetector.h"
#include "vision/HalconEngine.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace firedog {

namespace {

constexpr ImU32 kRoiCol    = IM_COL32(80, 200, 255, 220);   // search window
constexpr ImU32 kBoxCol    = IM_COL32(255, 210, 70, 230);   // template box
constexpr ImU32 kHitCol    = IM_COL32(80, 240, 130, 255);   // a good detection
constexpr ImU32 kAnchorCol = IM_COL32(255, 120, 200, 255);  // the reported point

const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);
const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);

// A detect that takes this long makes the Live checkbox feel broken. Warning
// once beats silently being slow: the fix (a tighter ROI, fewer pyramid levels)
// is one the operator has right in front of them.
constexpr double kSlowDetectMs = 150.0;

// Explanatory paragraph under a control group. TextDisabled does NOT wrap, so
// hand-broken lines would spill out of a narrowed parameter column — which is
// exactly the column the operator is allowed to narrow. Wrap instead.
void HelpText(const char* s)
{
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
    ImGui::TextWrapped("%s", s);
    ImGui::PopStyleColor();
}

// Width for a LABELLED widget: ImGui draws the label to the right of the field,
// so -FLT_MIN (stretch to the edge) pushes every label off the panel. A negative
// width means "available minus this", so reserving a share of the column keeps
// the labels readable at any splitter position.
float LabelledItemWidth()
{
    const float avail = ImGui::GetContentRegionAvail().x;
    float reserve = avail * 0.45f;
    if (reserve < 90.0f)  reserve = 90.0f;
    if (reserve > 180.0f) reserve = 180.0f;
    return -reserve;
}

// A detection model belongs to exactly ONE alignment station, and its name is
// the ownership record — the same device the private station positions use
// (`algn_<procUid>_<station>`, PageAlign.cpp). There is nothing to share: the
// ROI, the threshold and the taught patch only mean anything at the pose,
// exposure and magnification they were tuned at, so a model that "fits" a second
// station is a coincidence, not a reuse. That is why the window has no model
// list, no New/Duplicate/Delete and no name field: naming a thing that can only
// have one owner is a decision nobody should have to make.
//
// Kept in this TU because it also owns the two maintenance calls (drop with the
// procedure, copy with a duplicate) — one place knows the convention.
std::string StationDetectorName(const std::string& procUid, int station)
{
    return "vdet_" + procUid + "_" + std::to_string(station);
}

// Normalise a rubber-band drag into a positive-extent rectangle clamped to the
// image. Dragging up-left is as natural as down-right, and a negative width
// would reach HALCON as an empty region.
VisionRoi RectFromDrag(float x0, float y0, float x1, float y1, int imgW, int imgH)
{
    VisionRoi r;
    float lx = x0 < x1 ? x0 : x1, hx = x0 < x1 ? x1 : x0;
    float ly = y0 < y1 ? y0 : y1, hy = y0 < y1 ? y1 : y0;
    if (lx < 0.0f) lx = 0.0f;
    if (ly < 0.0f) ly = 0.0f;
    if (hx > static_cast<float>(imgW)) hx = static_cast<float>(imgW);
    if (hy > static_cast<float>(imgH)) hy = static_cast<float>(imgH);
    r.x = static_cast<int>(lx + 0.5f);
    r.y = static_cast<int>(ly + 0.5f);
    r.w = static_cast<int>(hx - lx + 0.5f);
    r.h = static_cast<int>(hy - ly + 0.5f);
    r.enabled = true;
    return r;
}

} // namespace

// ---- Opening / model lifecycle ---------------------------------------------

const VisionModel* App::StationDetector(const AlignProc& a, int station) const
{
    return FindVisionModel(session_.VisionModels(), StationDetectorName(a.uid, station));
}

void App::OpenVisionTeach(AlignProc& a, int station)
{
    visionErr_.clear();
    visionHaveDet_    = false;
    visionLive_       = false;
    visionDragMode_   = 0;
    visionDragging_   = false;
    visionSlowWarned_ = false;

    // Address the model by its canonical private NAME, not by whatever uid the
    // station happens to hold: the name is the ownership record, so this is the
    // one lookup that cannot end up pointing at another station's model.
    const std::string name = StationDetectorName(a.uid, station);
    VisionModel* m = FindVisionModel(session_.VisionModels(), name);
    if (!m) {
        VisionModel fresh;
        fresh.uid  = GenerateUid();
        fresh.name = name;
        session_.VisionModels().push_back(std::move(fresh));
        m = &session_.VisionModels().back();
        session_.SaveVision();   // the station's reference below must survive a crash
    }
    visionSelUid_ = m->uid;

    AlignStation& st = (station == 1) ? a.s1 : a.s2;
    st.detector = m->uid;   // recipes and align_station resolve by uid

    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s  -  station %d (E%d)",
                  a.name.c_str(), station, station);
    visionOwnerLabel_ = buf;

    // Centre the view on the frame; the ROI takes over once one is set.
    visionFocusX_ = static_cast<float>(cameraW_) * 0.5f;
    visionFocusY_ = static_cast<float>(cameraH_) * 0.5f;
    visionTmplKey_.clear();   // force a thumbnail reload for the new model

    SwitchPage(Page::VisionTeach);   // every page switch routes through here
}

// Deleting a procedure takes its private detectors with it. Without this they
// would accumulate invisibly: nothing lists them, so an orphan is something
// nobody would ever go looking for — the same reasoning as
// DropAlignStationPositions.
void App::DropAlignStationDetectors(const AlignProc& a)
{
    int dropped = 0;
    for (int s = 1; s <= 2; ++s) {
        const VisionModel* m =
            FindVisionModel(session_.VisionModels(), StationDetectorName(a.uid, s));
        if (!m) continue;
        const std::string uid = m->uid;
        if (visionSelUid_ == uid) visionSelUid_.clear();   // do not leave the page on a corpse
        workflow_.InvalidateDetector(uid);
        if (session_.DeleteVisionModel(uid)) ++dropped;
    }
    if (dropped > 0) {
        session_.SaveVision();
        FD_INFO("vision", "'%s': dropped %d private detection model(s)",
                a.name.c_str(), dropped);
    }
}

// A duplicated procedure gets its OWN copies, patch file included — sharing
// rows the original's Delete would take away is how a duplicate silently breaks
// later. Mirrors the private-station-position copy beside it.
void App::CopyAlignStationDetectors(const AlignProc& src, AlignProc& dst)
{
    bool any = false;
    for (int s = 1; s <= 2; ++s) {
        const VisionModel* sm =
            FindVisionModel(session_.VisionModels(), StationDetectorName(src.uid, s));
        if (!sm) continue;

        VisionModel copy = *sm;   // copy BEFORE the push_back invalidates `sm`
        copy.uid  = GenerateUid();
        copy.name = StationDetectorName(dst.uid, s);
        if (!copy.templateFile.empty()) {
            std::vector<unsigned char> gray;
            int tw = 0, th = 0;
            if (LoadGrayPgm(session_.VisionDir() + "/" + sm->templateFile, &gray, &tw, &th)) {
                copy.templateFile = copy.uid + ".pgm";
                SaveGrayPgm(session_.VisionDir() + "/" + copy.templateFile,
                            gray.data(), tw, th);
            } else {
                copy.templateFile.clear();   // no patch is better than a dangling one
            }
        }
        (s == 1 ? dst.s1 : dst.s2).detector = copy.uid;
        session_.VisionModels().push_back(std::move(copy));
        any = true;
    }
    if (any) session_.SaveVision();
}

void App::RefreshVisionTemplateTex(const VisionModel& m)
{
    // Key on uid + filename + size so a re-teach (which keeps the filename but
    // changes the content) still reloads: the size is what changes with it.
    const std::string key = m.uid + "|" + m.templateFile + "|" +
                            std::to_string(m.ncc.tmplW) + "x" + std::to_string(m.ncc.tmplH);
    if (key == visionTmplKey_) return;
    visionTmplKey_ = key;
    visionTmplW_ = visionTmplH_ = 0;
    if (m.templateFile.empty()) return;

    std::vector<unsigned char> gray;
    int w = 0, h = 0;
    if (!LoadGrayPgm(session_.VisionDir() + "/" + m.templateFile, &gray, &w, &h)) return;

    // The GL upload path is RGB8, so expand the grey plane. A taught patch is
    // small (tens of kB), and this runs only when the model changes.
    std::vector<unsigned char> rgb(static_cast<std::size_t>(w) * h * 3);
    for (std::size_t i = 0; i < gray.size(); ++i) {
        rgb[i * 3 + 0] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = gray[i];
    }
    visionTmplTex_ = host_.EnsureTextureRGB(visionTmplTex_, w, h, rgb.data());
    visionTmplW_   = w;
    visionTmplH_   = h;
}

void App::StartVisionReliabilityRun(const VisionModel& m)
{
    visionErr_.clear();
    visionRelHave_ = false;   // a new run supersedes an unaccepted old one

    RecipeStep step;
    step.type     = "vision_test";
    step.nickname = "Reliability: " + m.name;
    step.params   = {
        {"model", m.uid},
        {"runs", visionTestRuns_},
        {"interval_ms", 100},
        {"trace", true},          // one CSV line per run into the daily log
    };

    Recipe one;
    one.uid  = "vision-test";
    one.name = kVisionTestRunName;
    one.steps.push_back(std::move(step));

    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &visionErr_))
        FD_WARN("vision", "reliability run not started: %s", visionErr_.c_str());
}

// Called from App::Update — see App.h. The run happens on a worker over a COPY
// of the document, so its numbers come back through Workflow's queue and are
// PARKED here; the page's Accept button is what writes them onto the model.
//
// The engine's Status ctx snapshot is NOT usable for this: RecipeEngine clears
// runCtx_ in its terminal block before the state becomes Completed, so a UI
// waiting for Completed and then reading Status.ctx always reads an empty map.
void App::DrainVisionTestResult()
{
    for (const auto& [uid, rel] : workflow_.TakeVisionReliability()) {
        visionRelUid_     = uid;
        visionRelPending_ = rel;
        visionRelHave_    = true;
    }
}

// The E-point marks the Aux overlay draws. Two sources, and which one wins is
// the whole design:
//
//   * a RUN owns them while it is executing — that is live feedback, and it has
//     to appear as each station is captured rather than only after the solve;
//   * otherwise they mirror the SELECTED procedure's taught frame, so the same
//     overlay doubles as a check on what is actually stored (drive to local
//     (0,0) and E1's mark should sit under the centre crosshair).
//
// The two agree by construction once a run finishes: align_solve persists the
// same points the queue delivered.
void App::DrainAlignPoints()
{
    bool got = false;
    for (const Workflow::AlignPointMark& mk : workflow_.TakeAlignPoints()) {
        const int i = (mk.station == 2) ? 1 : 0;
        // A fresh E1 means a NEW run: drop the previous E2 rather than leaving
        // last run's second point on screen next to this run's first.
        if (i == 0) alignMarkHave_[1] = false;
        alignMarkE_[i]    = mk.point;
        alignMarkHave_[i] = true;
        got = true;
    }
    if (got || orch_.Busy()) return;   // a live run owns the marks

    const std::vector<AlignProc>& list = session_.Aligns();
    const bool sel = alignSel_ >= 0 && alignSel_ < static_cast<int>(list.size());
    if (sel && list[alignSel_].result.valid) {
        alignMarkE_[0] = list[alignSel_].result.e1;
        alignMarkE_[1] = list[alignSel_].result.e2;
        alignMarkHave_[0] = alignMarkHave_[1] = true;
    } else {
        alignMarkHave_[0] = alignMarkHave_[1] = false;
    }
}

// ---- The image band ---------------------------------------------------------

void App::DrawVisionTeachImage(VisionModel& m)
{
    UpdateCameraTexture();
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;

    // Follow the ROI when there is one: at high zoom the search window is the
    // only part of the frame that matters, and losing it off-screen while
    // tuning a threshold is maddening.
    if (m.roi.enabled && m.roi.w > 0 && m.roi.h > 0) {
        visionFocusX_ = static_cast<float>(m.roi.x) + static_cast<float>(m.roi.w) * 0.5f;
        visionFocusY_ = static_cast<float>(m.roi.y) + static_cast<float>(m.roi.h) * 0.5f;
    } else if (haveImg && visionFocusX_ <= 0.0f && visionFocusY_ <= 0.0f) {
        visionFocusX_ = static_cast<float>(cameraW_) * 0.5f;
        visionFocusY_ = static_cast<float>(cameraH_) * 0.5f;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.x < 40.0f) avail.x = 40.0f;
    if (avail.y < 40.0f) avail.y = 40.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    const CameraView view = MakeCameraView(origin, avail, cameraW_, cameraH_,
                                           visionFocusX_, visionFocusY_, visionZoom_);

    if (haveImg) {
        ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, view.Uv0(), view.Uv1());
    } else {
        dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y),
                          IM_COL32(18, 18, 22, 255));
        const char* msg = "no camera feed - connect the camera on the Camera page";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(origin.x + (avail.x - ts.x) * 0.5f,
                           origin.y + (avail.y - ts.y) * 0.5f),
                    IM_COL32(220, 140, 140, 255), msg);
        ImGui::Dummy(avail);
    }

    ImGui::SetCursorScreenPos(origin);
    ImGui::InvisibleButton("##visionImg", avail,
                           ImGuiButtonFlags_MouseButtonLeft);
    const bool hovered = ImGui::IsItemHovered();

    // Rubber-band drag, armed by one of the two "drag it on the image" buttons.
    // Arming rather than drag-anywhere keeps an accidental click from silently
    // reshaping the search window while somebody is judging a result.
    if (visionDragMode_ != 0 && haveImg) {
        if (ImGui::IsItemActivated()) {
            visionDragging_ = true;
            view.ToCamera(ImGui::GetIO().MousePos, &visionDragX0_, &visionDragY0_);
            visionDragX1_ = visionDragX0_;
            visionDragY1_ = visionDragY0_;
        }
        if (visionDragging_ && ImGui::IsItemActive())
            view.ToCamera(ImGui::GetIO().MousePos, &visionDragX1_, &visionDragY1_);
        if (visionDragging_ && ImGui::IsItemDeactivated()) {
            visionDragging_ = false;
            const VisionRoi r = RectFromDrag(visionDragX0_, visionDragY0_,
                                             visionDragX1_, visionDragY1_,
                                             cameraW_, cameraH_);
            if (r.w >= 4 && r.h >= 4) {
                if (visionDragMode_ == 1) {
                    m.roi = r;
                } else if (visionDragMode_ == 2) {
                    int tw = 0, th = 0;
                    std::string err;
                    if (workflow_.TeachDetector(m, session_.VisionDir(), r, &tw, &th, &err)) {
                        m.templateFile = m.uid + ".pgm";
                        m.ncc.tmplW    = tw;
                        m.ncc.tmplH    = th;
                        m.method       = "ncc";
                        // A fresh template invalidates the old anchor: it was an
                        // offset from a DIFFERENT patch centre, so keeping it
                        // would point at nothing in particular.
                        m.ncc.anchorOffsetX = m.ncc.anchorOffsetY = 0.0;
                        m.ncc.useAnchor     = false;
                        visionTmplKey_.clear();
                        visionErr_.clear();
                        visionHaveDet_ = false;
                        // The .pgm is already on disk, so save the document too
                        // rather than leaving a patch file with no model row
                        // pointing at it. (Parameter edits stay explicit-Save.)
                        session_.SaveVision();
                    } else {
                        visionErr_ = err;
                    }
                }
            } else {
                visionErr_ = "that box is too small - drag at least 4x4 px";
            }
            visionDragMode_ = 0;
        }
    }

    // Wheel zooms; applies from the next frame (the view is already computed).
    if (haveImg && hovered && ImGui::GetIO().MouseWheel != 0.0f) {
        visionZoom_ *= (ImGui::GetIO().MouseWheel > 0.0f) ? 1.25f : 0.8f;
        visionZoom_ = fminf(fmaxf(visionZoom_, 1.0f), 16.0f);
    }

    if (!haveImg) return;

    // ---- Overlay. Clipped to the image box: zoomed in, the ROI or the
    // detection can sit entirely outside the visible window.
    dl->PushClipRect(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), true);

    const auto rectOnScreen = [&](const VisionRoi& r, ImU32 col, float thick) {
        const ImVec2 a = view.ToScreen(static_cast<float>(r.x), static_cast<float>(r.y));
        const ImVec2 b = view.ToScreen(static_cast<float>(r.x + r.w),
                                       static_cast<float>(r.y + r.h));
        dl->AddRect(a, b, col, 0.0f, 0, thick);
    };

    if (m.roi.enabled && m.roi.w > 0 && m.roi.h > 0) rectOnScreen(m.roi, kRoiCol, 1.5f);

    if (visionDragging_) {
        const VisionRoi live = RectFromDrag(visionDragX0_, visionDragY0_,
                                            visionDragX1_, visionDragY1_,
                                            cameraW_, cameraH_);
        rectOnScreen(live, visionDragMode_ == 1 ? kRoiCol : kBoxCol, 2.0f);
    }

    if (visionHaveDet_ && visionDet_.ok) {
        const ImVec2 c = view.ToScreen(static_cast<float>(visionDet_.xPx),
                                       static_cast<float>(visionDet_.yPx));
        if (visionDet_.radiusPx > 0.0) {
            dl->AddCircle(c, static_cast<float>(visionDet_.radiusPx) * view.scale,
                          kHitCol, 0, 2.0f);
        } else if (visionDet_.widthPx > 0.0 && visionDet_.heightPx > 0.0) {
            const float hw = static_cast<float>(visionDet_.widthPx) * 0.5f * view.scale;
            const float hh = static_cast<float>(visionDet_.heightPx) * 0.5f * view.scale;
            dl->AddRect(ImVec2(c.x - hw, c.y - hh), ImVec2(c.x + hw, c.y + hh),
                        kHitCol, 0.0f, 0, 2.0f);
        }
        // The reported point, drawn separately from the shape: with an anchor
        // these are deliberately NOT the same place, and seeing the gap is the
        // only way to confirm the anchor is doing what was intended.
        const float mh = 18.0f;
        dl->AddLine(ImVec2(c.x - mh, c.y), ImVec2(c.x + mh, c.y), kAnchorCol, 2.0f);
        dl->AddLine(ImVec2(c.x, c.y - mh), ImVec2(c.x, c.y + mh), kAnchorCol, 2.0f);

        char lbl[160];
        std::snprintf(lbl, sizeof(lbl),
                      "%.1f, %.1f px   dX %+.4f  dY %+.4f mm   score %.3f",
                      visionDet_.xPx, visionDet_.yPx, visionDet_.dxMM, visionDet_.dyMM,
                      visionDet_.score);
        const ImVec2 lts = ImGui::CalcTextSize(lbl);
        ImVec2 lp(c.x + 14.0f, c.y + 10.0f);
        lp.x = fminf(lp.x, origin.x + avail.x - lts.x - 8.0f);
        lp.y = fminf(lp.y, origin.y + avail.y - lts.y - 6.0f);
        dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                          ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                          IM_COL32(0, 0, 0, 170), 3.0f);
        dl->AddText(lp, IM_COL32(200, 255, 220, 255), lbl);
    }

    dl->PopClipRect();
}

// ---- The parameter column ---------------------------------------------------

void App::DrawVisionTeachParams(VisionModel& m)
{
    std::string why;
    const bool halcon = HalconDetector::Available(&why);
    const bool busy   = orch_.Busy();
    const float itemW = LabelledItemWidth();

    // ---- 1. Camera ----
    ImGui::SeparatorText("Camera");
    float expo = static_cast<float>(m.exposureUs);
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::InputFloat("exposure (us)", &expo, 0.0f, 0.0f, "%.0f"))
        m.exposureUs = expo < 0.0f ? 0.0 : static_cast<double>(expo);
    ImGui::SameLine();
    ImGui::BeginDisabled(m.exposureUs <= 0.0);
    if (ImGui::Button("Apply##visexpo")) workflow_.SetCameraExposure(m.exposureUs);
    ImGui::EndDisabled();
    HelpText("0 = leave the camera alone. A model is tuned AT an exposure, so it "
             "carries the one it was taught with.");

    const Workflow::FocusSample fs = workflow_.LatestFocus();
    if (fs.ok) {
        const bool badLuma = fs.meanLuma < 5.0 || fs.meanLuma > 250.0;
        ImGui::TextColored(badLuma ? kWarn : ImGui::GetStyle().Colors[ImGuiCol_Text],
                           "focus %.0f   luma %.0f", fs.score, fs.meanLuma);
        if (badLuma) {
            ImGui::SameLine();
            ImGui::TextColored(kWarn, "(fix exposure first)");
        }
    }
    double mmX = 0.0, mmY = 0.0;
    workflow_.CameraPixelToMm(mmX, mmY);
    ImGui::TextDisabled("%.5f / %.5f mm per px (Camera page)", mmX, mmY);

    // ---- 2. Search ROI ----
    ImGui::SeparatorText("Search ROI");
    ImGui::Checkbox("limit the search to a region", &m.roi.enabled);
    ImGui::BeginDisabled(!m.roi.enabled);
    int roi[4] = {m.roi.x, m.roi.y, m.roi.w, m.roi.h};
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::DragInt4("##roi", roi, 1.0f, 0, 100000, "%d")) {
        m.roi.x = roi[0]; m.roi.y = roi[1]; m.roi.w = roi[2]; m.roi.h = roi[3];
    }
    if (ImGui::Button(visionDragMode_ == 1 ? "drag it on the image..." : "Set ROI by drag"))
        visionDragMode_ = (visionDragMode_ == 1) ? 0 : 1;
    ImGui::SameLine();
    if (ImGui::Button("Full frame")) {
        m.roi = VisionRoi{};
        visionFocusX_ = static_cast<float>(cameraW_) * 0.5f;
        visionFocusY_ = static_cast<float>(cameraH_) * 0.5f;
    }
    ImGui::EndDisabled();
    HelpText("x / y / w / h in camera pixels. An ROI is the cheapest speed-up there "
             "is, and usually the difference between a detector that can run live "
             "and one that cannot.");

    // ---- 3. Method ----
    ImGui::SeparatorText("Method");
    int method = (m.method == "ncc") ? 1 : 0;
    if (ImGui::RadioButton("Circle / fiducial", method == 0)) { m.method = "circle"; visionHaveDet_ = false; }
    ImGui::SameLine();
    if (ImGui::RadioButton("Pattern (NCC)", method == 1)) { m.method = "ncc"; visionHaveDet_ = false; }

    if (m.method == "circle") {
        CircleParams& c = m.circle;
        const char* pol[] = {"dark feature", "bright feature"};
        ImGui::SetNextItemWidth(itemW);
        ImGui::Combo("polarity", &c.polarity, pol, 2);
        ImGui::SetNextItemWidth(itemW);
        ImGui::DragInt("grey max", &c.thresholdMax, 1.0f, 1, 255);
        if (c.polarity == 0) {
            ImGui::SetNextItemWidth(itemW);
            ImGui::DragInt("grey min", &c.thresholdMin, 1.0f, 0, 254);
        } else {
            HelpText("bright: the feature is grey max .. 255");
        }
        float cr = static_cast<float>(c.closingRadius);
        ImGui::SetNextItemWidth(itemW);
        if (ImGui::DragFloat("closing radius", &cr, 0.1f, 0.0f, 60.0f, "%.1f"))
            c.closingRadius = static_cast<double>(cr);
        ImGui::SetNextItemWidth(itemW);
        ImGui::DragInt("min area px", &c.minArea, 10.0f, 1, 10000000);
        ImGui::SetNextItemWidth(itemW);
        ImGui::DragInt("max area px", &c.maxArea, 100.0f, 1, 100000000);
        float mc = static_cast<float>(c.minCircularity);
        ImGui::SetNextItemWidth(itemW);
        if (ImGui::SliderFloat("min circularity", &mc, 0.0f, 1.0f, "%.2f"))
            c.minCircularity = static_cast<double>(mc);
        const char* fits[] = {"atukey (robust)", "geotukey"};
        ImGui::SetNextItemWidth(itemW);
        ImGui::Combo("fit", &c.fitMethod, fits, 2);
        HelpText("Threshold -> close -> connect -> area & circularity filter -> "
                 "largest -> robust circle fit. Start with the grey window, then "
                 "widen min area until the feature survives.");
    } else {
        NccParams& n = m.ncc;
        ImGui::BeginDisabled(!halcon);
        if (ImGui::Button(visionDragMode_ == 2 ? "drag the template box..."
                                               : (m.templateFile.empty() ? "Teach template"
                                                                         : "Re-teach template")))
            visionDragMode_ = (visionDragMode_ == 2) ? 0 : 2;
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("%s", m.templateFile.empty()
                                      ? "(none yet)"
                                      : (std::to_string(n.tmplW) + " x " +
                                         std::to_string(n.tmplH) + " px").c_str());

        RefreshVisionTemplateTex(m);
        if (visionTmplTex_ && visionTmplW_ > 0 && visionTmplH_ > 0) {
            // Show the patch at a legible size and let a click on it place the
            // anchor. Anchoring on the THUMBNAIL rather than the live image is
            // deliberate: the offset is defined relative to the template, and
            // this is the only view where that is literally what is on screen.
            const float maxW = ImGui::GetContentRegionAvail().x;
            const float sc   = fminf(fminf(maxW / static_cast<float>(visionTmplW_), 4.0f),
                                     160.0f / static_cast<float>(visionTmplH_));
            const ImVec2 sz(static_cast<float>(visionTmplW_) * sc,
                            static_cast<float>(visionTmplH_) * sc);
            const ImVec2 tOrigin = ImGui::GetCursorScreenPos();
            ImGui::Image((ImTextureID)(intptr_t)visionTmplTex_, sz);
            ImGui::SetCursorScreenPos(tOrigin);
            if (ImGui::InvisibleButton("##anchorPick", sz)) {
                const ImVec2 mp = ImGui::GetIO().MousePos;
                n.anchorOffsetX = static_cast<double>((mp.x - tOrigin.x) / sc) -
                                  static_cast<double>(visionTmplW_) * 0.5;
                n.anchorOffsetY = static_cast<double>((mp.y - tOrigin.y) / sc) -
                                  static_cast<double>(visionTmplH_) * 0.5;
                // Match bobcat: a sub-tenth-pixel offset IS the centre, so the
                // flag derives from the numbers instead of being a third thing
                // to keep in step with them.
                n.useAnchor = std::fabs(n.anchorOffsetX) > 0.1 ||
                              std::fabs(n.anchorOffsetY) > 0.1;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Click to place the anchor - the point this model\n"
                                  "REPORTS. It rides with the match, rotating by the\n"
                                  "match angle, so it stays on the feature.");
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const ImVec2 ac(tOrigin.x + sz.x * 0.5f +
                                static_cast<float>(n.anchorOffsetX) * sc,
                            tOrigin.y + sz.y * 0.5f +
                                static_cast<float>(n.anchorOffsetY) * sc);
            dl->AddLine(ImVec2(ac.x - 8.0f, ac.y), ImVec2(ac.x + 8.0f, ac.y), kAnchorCol, 1.5f);
            dl->AddLine(ImVec2(ac.x, ac.y - 8.0f), ImVec2(ac.x, ac.y + 8.0f), kAnchorCol, 1.5f);
            ImGui::TextDisabled("anchor %+.1f, %+.1f px%s",
                                n.anchorOffsetX, n.anchorOffsetY,
                                n.useAnchor ? "" : " (centre)");
            ImGui::SameLine();
            if (ImGui::SmallButton("Centre")) {
                n.anchorOffsetX = n.anchorOffsetY = 0.0;
                n.useAnchor = false;
            }
        }

        float ms = static_cast<float>(n.minScore);
        ImGui::SetNextItemWidth(itemW);
        if (ImGui::SliderFloat("min score", &ms, 0.1f, 1.0f, "%.2f"))
            n.minScore = static_cast<double>(ms);
        float at = static_cast<float>(n.angleToleranceDeg);
        ImGui::SetNextItemWidth(itemW);
        if (ImGui::DragFloat("angle tol (deg)", &at, 0.5f, 0.0f, 180.0f, "%.1f"))
            n.angleToleranceDeg = static_cast<double>(at);
        ImGui::SetNextItemWidth(itemW);
        ImGui::DragInt("pyramid levels", &n.numLevels, 0.1f, 0, 6, "%d (0 = auto)");
        ImGui::SetNextItemWidth(itemW);
        ImGui::DragInt("max matches", &n.maxMatches, 0.1f, 1, 20);
        int metric = (n.metric == "use_polarity") ? 1 : 0;
        const char* metrics[] = {"ignore global polarity", "use polarity"};
        ImGui::SetNextItemWidth(itemW);
        if (ImGui::Combo("metric", &metric, metrics, 2))
            n.metric = metric == 1 ? "use_polarity" : "ignore_global_polarity";
        HelpText("Angle tolerance 0 means no rotation search, which is much the "
                 "fastest. Raise it only if the part really turns.");
    }

    // ---- 4. Detect ----
    ImGui::SeparatorText("Detect");
    ImGui::BeginDisabled(!halcon);
    if (ImGui::Button("Detect once", ImVec2(120.0f, 0.0f))) {
        workflow_.Detect(m, session_.VisionDir(), &visionDet_);
        visionHaveDet_ = true;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Live", &visionLive_);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Re-detect on every new camera frame. Costs a detection\n"
                          "per frame - turn it off when you stop turning knobs.");

    if (!halcon) {
        ImGui::TextColored(kBad, "%s", why.c_str());
        if (ImGui::SmallButton("Re-check")) HalconEngine::Instance().Refresh();
        ImGui::SameLine();
        ImGui::TextDisabled("(drop a licence .dat in and re-check - no restart)");
    }

    if (visionHaveDet_) {
        if (visionDet_.ok) {
            ImGui::TextColored(kGood, "found %d  score %.3f  %.1f ms",
                               visionDet_.count, visionDet_.score, visionDet_.elapsedMs);
            ImGui::Text("x %.2f  y %.2f px", visionDet_.xPx, visionDet_.yPx);
            ImGui::Text("dX %+.4f  dY %+.4f mm", visionDet_.dxMM, visionDet_.dyMM);
            if (m.method == "ncc")
                ImGui::Text("angle %.3f deg", visionDet_.angleDeg);
            else
                ImGui::Text("radius %.2f px  (%.4f mm)", visionDet_.radiusPx,
                            visionDet_.radiusPx * mmX);
            if (visionDet_.elapsedMs > kSlowDetectMs)
                ImGui::TextColored(kWarn, "slow - tighten the ROI or drop a pyramid level");
        } else {
            ImGui::TextColored(kBad, "%s", visionDet_.error.c_str());
        }
    }

    // ---- 5. Reliability ----
    ImGui::SeparatorText("Reliability");
    ImGui::SetNextItemWidth(90.0f);
    ImGui::DragInt("runs", &visionTestRuns_, 1.0f, 2, 500);
    ImGui::SameLine();
    ImGui::BeginDisabled(!halcon || busy);
    if (ImGui::Button("Run test")) StartVisionReliabilityRun(m);
    ImGui::EndDisabled();
    if (busy) {
        ImGui::SameLine();
        ImGui::TextDisabled("another run is active");
    }

    // Report only on OUR run: the engine is shared with the Recipe Editor, the
    // Run page and the programs.
    const RecipeEngine::Status rs = orch_.recipe.GetStatus();
    if (rs.recipeName == kVisionTestRunName) {
        if (rs.state == RecipeEngine::State::Running) {
            ImGui::TextUnformatted(rs.currentStep.c_str());
            if (ImGui::Button("Stop##vistest")) orch_.StopAll();
        } else if (!rs.error.empty() && (rs.state == RecipeEngine::State::Failed ||
                                         rs.state == RecipeEngine::State::Stopped)) {
            ImGui::TextColored(kBad, "%s", rs.error.c_str());
        }
    }

    // A finished run waits here to be ACCEPTED. Whether 20/20 at 0.05 px is good
    // enough is a judgement about this part and this feature; the person who
    // just watched the run makes it, and until they do the model keeps whatever
    // figure it already had.
    const bool pending = visionRelHave_ && visionRelUid_ == m.uid;
    if (pending) {
        const VisionReliability& r = visionRelPending_;
        const bool clean = r.runs > 0 && r.ok == r.runs;
        ImGui::TextColored(clean ? kGood : kWarn,
                           "%d/%d ok   sigma %.3f / %.3f px   worst %.3f px",
                           r.ok, r.runs, r.stdXPx, r.stdYPx, r.maxDevPx);
        ImGui::TextDisabled("mean score %.3f   %s", r.meanScore, r.testedAt.c_str());

        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.18f, 0.55f, 0.28f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.24f, 0.68f, 0.36f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.14f, 0.45f, 0.22f, 1.0f));
        const bool accept = ImGui::Button("Accept & save");
        ImGui::PopStyleColor(3);
        ImGui::SameLine();
        const bool discard = ImGui::SmallButton("Discard");
        if (accept) {
            m.rel = r;
            session_.SaveVision();   // a figure a crash loses is a test to sit through again
            visionRelHave_ = false;
            FD_INFO("vision", "'%s': accepted reliability %d/%d, sigma %.3f/%.3f px",
                    m.name.c_str(), r.ok, r.runs, r.stdXPx, r.stdYPx);
        } else if (discard) {
            visionRelHave_ = false;
        }
    }

    // The figure currently ON the model, which is what the station card shows
    // and what an operator later trusts. Kept separate from the pending block
    // above so "what I just measured" and "what is recorded" never blur.
    if (m.rel.runs > 0) {
        const bool solid = m.rel.ok == m.rel.runs && m.rel.maxDevPx < 2.0;
        ImGui::TextDisabled("recorded:");
        ImGui::SameLine();
        ImGui::TextColored(solid ? kGood : kWarn,
                           "%d/%d ok   sigma %.3f / %.3f px   worst %.3f px",
                           m.rel.ok, m.rel.runs, m.rel.stdXPx, m.rel.stdYPx, m.rel.maxDevPx);
        ImGui::TextDisabled("mean score %.3f   %s", m.rel.meanScore, m.rel.testedAt.c_str());
    } else if (!pending) {
        HelpText("Never tested. One good detection proves nothing - what matters is "
                 "how far the reported point wanders when nothing has moved.");
    }
}

// ---- The page --------------------------------------------------------------
// A full-page takeover like the other editors, not a floating window: teaching
// a detector is a task you do FOR a station and then leave, and the operator is
// not meant to be jogging or watching another page underneath. The shared
// editor frame in MainWindow.cpp draws the title and the Close button (which
// returns to the Alignment page); everything below is the body.

void App::DrawVisionTeachPanel()
{
    VisionModel* m = FindVisionModel(session_.VisionModels(), visionSelUid_);
    if (!m) {
        // The model went away under us (a procedure delete, or a product load).
        ImGui::TextColored(kBad, "this detection model no longer exists");
        if (ImGui::Button("Back to Alignment")) SwitchPage(Page::Align);
        return;
    }

    // ---- Owner bar ----
    // No model list, no New/Duplicate/Delete, no name field: a model belongs to
    // exactly one station and is created, named and destroyed with it. What the
    // bar has to answer is "which station am I tuning?", not "which model".
    ImGui::TextUnformatted(visionOwnerLabel_.c_str());
    ImGui::SameLine();
    const bool dirty = session_.VisionDirty();
    if (ImGui::Button(dirty ? "Save *" : "Save")) {
        session_.SaveVision();
        workflow_.InvalidateDetector(m->uid);   // next detect rebuilds from the saved model
    }
    ImGui::SameLine();
    // The one-button exit, because leaving is what you do when you are done
    // tuning and Save-then-Close is the only sequence that ever makes sense.
    // (The frame's plain Close is still there for abandoning a bad experiment.)
    if (ImGui::Button("Save & close")) {
        session_.SaveVision();
        workflow_.InvalidateDetector(m->uid);
        SwitchPage(Page::Align);
        return;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%s", m->name.c_str());

    if (!visionErr_.empty()) ImGui::TextColored(kBad, "%s", visionErr_.c_str());
    if (visionDragMode_ != 0)
        ImGui::TextColored(kWarn, "%s",
                           visionDragMode_ == 1
                               ? "drag a rectangle on the image to set the search ROI"
                               : "drag a rectangle around the feature to cut the template");
    ImGui::Separator();

    // ---- Body: big live image | draggable splitter | parameter column ----
    // The column is resizable because its natural width is not a constant: a
    // circle model's form is short and narrow, an NCC model's carries a
    // thumbnail and a longer label set, and the operator's font size moves both.
    constexpr float kSplitW    = 6.0f;
    constexpr float kMinImageW = 240.0f;
    constexpr float kMinParamW = 200.0f;

    const float bodyH  = ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing();
    const float totalW = ImGui::GetContentRegionAvail().x;
    // Clamp every frame, not just on drag: the main window is resizable too, and
    // a width saved on a wide monitor must not push the image off a narrow one.
    const float maxParamW = totalW - kSplitW - kMinImageW;
    if (visionParamW_ > maxParamW) visionParamW_ = maxParamW;
    if (visionParamW_ < kMinParamW) visionParamW_ = kMinParamW;

    ImGui::BeginChild("visimg", ImVec2(totalW - visionParamW_ - kSplitW, bodyH),
                      ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollWithMouse);
    DrawVisionTeachImage(*m);
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##vissplit", true, kSplitW, bodyH)) {
        visionParamW_ -= ImGui::GetIO().MouseDelta.x;
        visionParamSplitting_ = true;
    } else if (visionParamSplitting_) {
        visionParamSplitting_ = false;
        SaveUiSettings();   // persist on drag END, like the main window's splitters
    }
    ImGui::SameLine(0.0f, 0.0f);

    ImGui::BeginChild("visparams", ImVec2(visionParamW_, bodyH), ImGuiChildFlags_Borders);
    DrawVisionTeachParams(*m);
    ImGui::EndChild();

    // ---- Footer ----
    ImGui::TextUnformatted("zoom");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    ImGui::SliderFloat("##viszoom", &visionZoom_, 1.0f, 16.0f, "x%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) visionZoom_ = 1.0f;
    ImGui::SameLine();
    ImGui::TextDisabled("wheel over the image zooms  |  models are saved with the product");

    // Live re-detect, gated on a NEW frame so a still feed costs nothing. Done
    // after the draw so the overlay above and this detection stay one frame
    // apart at most — and never inside the parameter form, where a mid-edit
    // detect would use half-applied values.
    if (visionLive_ && HalconDetector::Available(nullptr)) {
        const unsigned long long seq = workflow_.CameraFrameSeq();
        if (seq != visionSeqSeen_) {
            visionSeqSeen_ = seq;
            workflow_.Detect(*m, session_.VisionDir(), &visionDet_);
            visionHaveDet_ = true;
            if (visionDet_.elapsedMs > kSlowDetectMs && !visionSlowWarned_) {
                visionSlowWarned_ = true;
                FD_WARN("vision", "'%s': a detect takes %.0f ms - live tuning will feel "
                                  "slow; tighten the ROI or reduce the pyramid levels",
                        m->name.c_str(), visionDet_.elapsedMs);
            }
        }
    }
}

} // namespace firedog
