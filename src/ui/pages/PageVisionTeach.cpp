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

// The same convention one level down, for the two marks that locate an OBJECT
// being placed on a nominal. Private to that nominal for the same reason a
// station's is private to the station: the ROI, the threshold and the taught
// patch only mean anything at the pose and exposure they were tuned at, and a
// nominal is viewed from its own place.
//
// "vdet_" + 8 + "_" + 8 + "_p1" = 25 chars.
std::string NominalDetectorName(const std::string& procUid, const std::string& nomUid,
                                int index)
{
    return "vdet_" + procUid + "_" + nomUid + "_p" + std::to_string(index);
}

// The corner form's two halves, plus the diagram they share. File-scope helpers
// rather than more branches inline: they hold no state (every value is a
// CornerParams field), and the two modes share nothing below the reported corner
// — mode 1 builds its template out of numbers, mode 0 runs a blob chain.

// What mode 1 is actually looking for, drawn at diagram scale: the material
// quadrant filled the way MakeCornerTemplate fills it, with the reported point
// crossed at the vertex. Click a quadrant to name that corner.
//
// It exists because "which corner" is a picture, not a word: naming the filled
// quadrant instead of the corner was read backwards by everyone who tried it,
// and a diagram that cannot disagree with the search is the fix that sticks.
void CornerDiagram(CornerParams& c)
{
    constexpr float kS = 84.0f;
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##cornerdiag", ImVec2(kS, kS));
    if (ImGui::IsItemClicked()) {
        const ImVec2 mp = ImGui::GetIO().MousePos;
        const bool right = (mp.x - o.x) > kS * 0.5f;
        const bool low   = (mp.y - o.y) > kS * 0.5f;
        // The click names a CORNER — the corner of the material that sits at the
        // centre of the diagram, which is the quadrant diagonally opposite.
        c.corner = low ? (right ? 0 : 1) : (right ? 3 : 2);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click the quadrant your material sits in.\n"
                          "The cross is what the detector reports: the corner\n"
                          "where the two edges meet.");

    const ImU32 fg = (c.polarity == 0) ? IM_COL32(40, 40, 44, 255)
                                       : IM_COL32(215, 215, 215, 255);
    const ImU32 bg = (c.polarity == 0) ? IM_COL32(215, 215, 215, 255)
                                       : IM_COL32(30, 30, 34, 255);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(o, ImVec2(o.x + kS, o.y + kS), bg);

    const int   q = (((c.corner >= 0 && c.corner < 4) ? c.corner : 0) + 2) % 4;
    const float h = kS * 0.5f;
    const ImVec2 a(o.x + ((q == 0 || q == 3) ? 0.0f : h),
                   o.y + ((q == 0 || q == 1) ? 0.0f : h));
    dl->AddRectFilled(a, ImVec2(a.x + h, a.y + h), fg);
    dl->AddRect(o, ImVec2(o.x + kS, o.y + kS), IM_COL32(120, 120, 130, 255));

    const ImVec2 ctr(o.x + h, o.y + h);   // the vertex, always the diagram centre
    dl->AddLine(ImVec2(ctr.x - 12.0f, ctr.y), ImVec2(ctr.x + 12.0f, ctr.y), kAnchorCol, 2.0f);
    dl->AddLine(ImVec2(ctr.x, ctr.y - 12.0f), ImVec2(ctr.x, ctr.y + 12.0f), kAnchorCol, 2.0f);
}

void DrawCornerEdgeForm(CornerParams& c, float itemW, double mmX)
{
    CornerDiagram(c);
    ImGui::SameLine();
    ImGui::TextWrapped("The cross is the reported point. Click the shaded quadrant "
                       "to say where your material is.");

    float sz = static_cast<float>(c.sizePx);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::SliderFloat("edge length px", &sz, 8.0f, 256.0f, "%.0f",
                           ImGuiSliderFlags_Logarithmic))
        c.sizePx = static_cast<double>(sz);
    if (mmX > 0.0) ImGui::TextDisabled("%.0f px = %.3f mm", c.sizePx, c.sizePx * mmX);

    float ms = static_cast<float>(c.minScore);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::SliderFloat("min score", &ms, 0.1f, 1.0f, "%.2f"))
        c.minScore = static_cast<double>(ms);
    float at = static_cast<float>(c.angleToleranceDeg);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::DragFloat("angle tol (deg)", &at, 0.5f, 0.0f, 180.0f, "%.1f"))
        c.angleToleranceDeg = static_cast<double>(at);
    float sm = static_cast<float>(c.smoothPx);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::DragFloat("edge softness px", &sm, 0.1f, 0.0f, 8.0f, "%.1f"))
        c.smoothPx = static_cast<double>(sm);

    HelpText("The template is BUILT from these numbers - there is nothing to teach and "
             "nothing on disk. A patch of this size with one quadrant filled is matched "
             "by NCC, and the vertex is its centre. Keep the edge length inside the "
             "shortest straight run either edge has: for the end of a bar, no more than "
             "the bar's thickness. Softness matches the synthetic step to a real edge's "
             "ramp - 1 px suits most optics, raise it if the image is soft.");
    HelpText("A low score usually means the size is wrong (the patch is reaching past the "
             "end of one edge) or the material side is the wrong one of the four.");
}

void DrawCornerBlobForm(CornerParams& c, float itemW, double mmX)
{
    ImGui::Checkbox("auto threshold", &c.autoThreshold);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Re-splits light from dark on every frame\n"
                          "(max_separability). A corner mark is defined by\n"
                          "CONTRAST, not by an absolute grey, so this is what\n"
                          "survives a lamp ageing or a change of finish.");
    ImGui::BeginDisabled(c.autoThreshold);
    ImGui::SetNextItemWidth(itemW);
    ImGui::DragInt("grey threshold", &c.threshold, 1.0f, 1, 254);
    ImGui::EndDisabled();

    // The size IS the search: the area window is derived from it, so it gets the
    // slider and the derived window is printed underneath.
    float sz = static_cast<float>(c.sizePx);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::SliderFloat("square size px", &sz, 4.0f, 800.0f, "%.0f",
                           ImGuiSliderFlags_Logarithmic))
        c.sizePx = static_cast<double>(sz);
    float tol = static_cast<float>(c.sizeTolPct);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::SliderFloat("size tolerance", &tol, 5.0f, 90.0f, "+/- %.0f %%"))
        c.sizeTolPct = static_cast<double>(tol);
    const double lo = c.sizePx * (1.0 - c.sizeTolPct / 100.0);
    const double hi = c.sizePx * (1.0 + c.sizeTolPct / 100.0);
    ImGui::TextDisabled("accepts sides %.0f .. %.0f px  (area %.0f .. %.0f px2)",
                        lo, hi, lo * lo, hi * hi);
    if (mmX > 0.0) ImGui::TextDisabled("%.0f px = %.3f mm", c.sizePx, c.sizePx * mmX);

    float cr = static_cast<float>(c.closingRadius);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::DragFloat("closing radius", &cr, 0.1f, 0.0f, 30.0f, "%.1f"))
        c.closingRadius = static_cast<double>(cr);
    float mr = static_cast<float>(c.minRectangularity);
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::SliderFloat("min rectangularity", &mr, 0.0f, 1.0f, "%.2f"))
        c.minRectangularity = static_cast<double>(mr);

    HelpText("Threshold -> close -> connect -> area window from the size -> "
             "rectangularity gate -> largest -> sub-pixel rectangle fit, then the corner "
             "you picked. The corners are named in IMAGE terms and classified after the "
             "fit, so a rotated part still reports the one you asked for. Rectangularity "
             "is 1.0 for a perfect rectangle at any angle - drop it only if the mark has "
             "rounded corners.");
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
    visionAnchorOpen_ = false;   // never inherit the last station's open picker

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
    visionTmpl_.key.clear();   // force a thumbnail reload for the new model

    // Set every time, not just once: the page is shared with the nominals'
    // object detectors, and a station teach must not inherit their exit.
    visionReturnPage_ = Page::Align;
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
    // The procedure's nominals die with it, so their object detectors do too.
    for (const AlignNominal& n : a.nominals) DropNominalDetectors(a, n);
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

        const std::string uid = CopyVisionModelAs(*sm, StationDetectorName(dst.uid, s));
        if (uid.empty()) continue;
        (s == 1 ? dst.s1 : dst.s2).detector = uid;
        any = true;
    }
    // A duplicate's NOMINALS came along with the plain copy, so their private
    // object detectors have to as well — otherwise the copy would silently fall
    // back to asking an operator for every placement.
    for (std::size_t i = 0; i < dst.nominals.size() && i < src.nominals.size(); ++i) {
        for (int k = 1; k <= 2; ++k) {
            const VisionModel* sm = FindVisionModel(
                session_.VisionModels(),
                NominalDetectorName(src.uid, src.nominals[i].uid, k));
            if (!sm) continue;
            const std::string newName =
                NominalDetectorName(dst.uid, dst.nominals[i].uid, k);
            const std::string uid = CopyVisionModelAs(*sm, newName);
            if (uid.empty()) continue;
            (k == 1 ? dst.nominals[i].det1 : dst.nominals[i].det2) = uid;
            any = true;
        }
    }
    if (any) session_.SaveVision();
}

// Copy one model under a new private name, patch file included. Returns the new
// uid, or "" if the copy could not be made. Factored out when the nominals'
// object detectors needed the identical operation — a second copy of the .pgm
// handling is how one of them ends up sharing a patch file with its original.
std::string App::CopyVisionModelAs(const VisionModel& src, const std::string& newName)
{
    VisionModel copy = src;   // copy BEFORE any push_back invalidates `src`
    copy.uid  = GenerateUid();
    copy.name = newName;
    if (!copy.templateFile.empty()) {
        std::vector<unsigned char> gray;
        int tw = 0, th = 0;
        if (LoadGrayPgm(session_.VisionDir() + "/" + src.templateFile, &gray, &tw, &th)) {
            copy.templateFile = copy.uid + ".pgm";
            SaveGrayPgm(session_.VisionDir() + "/" + copy.templateFile, gray.data(), tw, th);
        } else {
            copy.templateFile.clear();   // no patch is better than a dangling one
        }
    }
    const std::string uid = copy.uid;
    session_.VisionModels().push_back(std::move(copy));
    return uid;
}

// ---- A nominal's object detectors ------------------------------------------

const VisionModel* App::NominalDetector(const AlignProc& a, const AlignNominal& n,
                                        int index) const
{
    return FindVisionModel(session_.VisionModels(),
                           NominalDetectorName(a.uid, n.uid, index));
}

// Same page, same model lifecycle, different owner: Close returns to the
// Placement page rather than to Alignment, which is what visionReturnPage_
// carries (a takeover -> takeover switch does not update prevPage_).
void App::OpenVisionTeachForNominal(AlignProc& a, AlignNominal& n, int index)
{
    visionErr_.clear();
    visionHaveDet_    = false;
    visionLive_       = false;
    visionDragMode_   = 0;
    visionDragging_   = false;
    visionSlowWarned_ = false;
    visionAnchorOpen_ = false;

    const std::string name = NominalDetectorName(a.uid, n.uid, index);
    VisionModel* m = FindVisionModel(session_.VisionModels(), name);
    if (!m) {
        VisionModel fresh;
        fresh.uid  = GenerateUid();
        fresh.name = name;
        session_.VisionModels().push_back(std::move(fresh));
        m = &session_.VisionModels().back();
        session_.SaveVision();   // the nominal's reference below must survive a crash
    }
    visionSelUid_ = m->uid;

    (index == 1 ? n.det1 : n.det2)   = m->uid;   // place_mark resolves by uid
    (index == 1 ? n.mode1 : n.mode2) = "auto";   // teaching one is asking to use it
    session_.SaveAligns();                       // taught by hand; persist like the rest

    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s  -  nominal '%s'  -  object point %d",
                  a.name.c_str(), n.name.c_str(), index);
    visionOwnerLabel_ = buf;

    visionFocusX_ = static_cast<float>(cameraW_) * 0.5f;
    visionFocusY_ = static_cast<float>(cameraH_) * 0.5f;
    visionTmpl_.key.clear();

    visionReturnPage_ = Page::AlignPlace;
    SwitchPage(Page::VisionTeach);
}

// Deleting a nominal takes its object detectors with it, exactly as deleting a
// procedure takes its station detectors.
void App::DropNominalDetectors(const AlignProc& a, const AlignNominal& n)
{
    int dropped = 0;
    for (int k = 1; k <= 2; ++k) {
        const VisionModel* m = NominalDetector(a, n, k);
        if (!m) continue;
        const std::string uid = m->uid;
        if (visionSelUid_ == uid) visionSelUid_.clear();
        workflow_.InvalidateDetector(uid);
        if (session_.DeleteVisionModel(uid)) ++dropped;
    }
    if (dropped > 0) {
        session_.SaveVision();
        FD_INFO("vision", "'%s' nominal '%s': dropped %d object detector(s)",
                a.name.c_str(), n.name.c_str(), dropped);
    }
}

// The key carries everything the picture and the reported point depend on, so a
// re-teach (which keeps the filename but changes the content), an anchor drag or
// a corner change all reload — and a Draw that runs every frame uploads nothing.
bool App::EnsurePatchTex(PatchTex& pt, const VisionModel& m)
{
    const std::string key = VisionModelPatchKey(m);
    if (key != pt.key) {
        pt.key = key;
        pt.w = pt.h = 0;

        std::vector<unsigned char> gray;
        int w = 0, h = 0;
        float rx = 0.0f, ry = 0.0f;
        if (VisionModelPatch(m, session_.VisionDir(), &gray, &w, &h, &rx, &ry)) {
            // The GL upload path is RGB8, so expand the grey plane. A patch is
            // small (tens of kB), and this runs only when the key changes.
            std::vector<unsigned char> rgb(static_cast<std::size_t>(w) * h * 3);
            for (std::size_t i = 0; i < gray.size(); ++i) {
                rgb[i * 3 + 0] = rgb[i * 3 + 1] = rgb[i * 3 + 2] = gray[i];
            }
            pt.tex = host_.EnsureTextureRGB(pt.tex, w, h, rgb.data());
            pt.w   = w;
            pt.h   = h;
            pt.rx  = rx;
            pt.ry  = ry;
        }
    }
    return pt.tex != 0 && pt.w > 0 && pt.h > 0;
}

void App::OpenVisionAnchorPicker()
{
    visionAnchorOpen_ = true;
    visionAnchorZoom_ = 0.0f;   // refit — the patch size is whatever was just cut
}

// ---- The anchor pick -------------------------------------------------------
// Its own window, and the ONLY place the anchor can be set. The offset is a
// sub-pixel property of the template, so it has to be picked on a view where a
// template pixel is a visible thing: the parameter column's thumbnail packs the
// patch into ~200 px, where one click covers several pixels and a stray one
// silently moves the point every later measurement is reported at.
//
// Not a modal: the camera keeps running underneath, Live detect keeps drawing
// the reported point on the big image, and watching that mark move as the
// anchor is dragged is the confirmation that the pick was right.
void App::DrawVisionAnchorWindow(VisionModel& m)
{
    if (!visionAnchorOpen_) return;
    if (m.method != "ncc" || m.templateFile.empty()) { visionAnchorOpen_ = false; return; }

    // Keyed, so this is a no-op once loaded.
    if (!EnsurePatchTex(visionTmpl_, m)) {
        visionAnchorOpen_ = false;
        return;
    }

    NccParams& n  = m.ncc;
    const float tw = static_cast<float>(visionTmpl_.w);
    const float th = static_cast<float>(visionTmpl_.h);

    if (visionAnchorZoom_ <= 0.0f) {
        const float fit = 420.0f / (tw > th ? tw : th);
        visionAnchorZoom_ = fminf(fmaxf(fit, 1.0f), 32.0f);
    }
    const float z = visionAnchorZoom_;

    // A sub-tenth-pixel offset IS the centre (bobcat's rule), so the flag is
    // derived from the numbers rather than being a third thing to keep in step.
    // Clamped to the patch: an anchor outside the template is not a point on
    // the feature, it is a typo.
    const auto commit = [&]() {
        const double hx = static_cast<double>(tw) * 0.5, hy = static_cast<double>(th) * 0.5;
        if (n.anchorOffsetX < -hx) n.anchorOffsetX = -hx;
        if (n.anchorOffsetX >  hx) n.anchorOffsetX =  hx;
        if (n.anchorOffsetY < -hy) n.anchorOffsetY = -hy;
        if (n.anchorOffsetY >  hy) n.anchorOffsetY =  hy;
        n.useAnchor = std::fabs(n.anchorOffsetX) > 0.1 || std::fabs(n.anchorOffsetY) > 0.1;
    };

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("Anchor point", &visionAnchorOpen_, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Click (or drag) the point this model should REPORT.");
        HelpText("The anchor rides with the match and rotates by the match angle, so it "
                 "stays on the feature. Arrow keys nudge by 1 px, shift+arrow by 0.1 px.");

        // Bounded box with scrollbars rather than an unbounded auto-resize: at
        // x32 a 200 px patch is 6400 px wide and the window would leave the screen.
        const float dispW = tw * z, dispH = th * z;
        const ImVec2 box(fminf(dispW, 900.0f) + 2.0f, fminf(dispH, 640.0f) + 2.0f);
        ImGui::BeginChild("anchimg", box, ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_HorizontalScrollbar);
        const ImVec2 o = ImGui::GetCursorScreenPos();
        ImGui::Image((ImTextureID)(intptr_t)visionTmpl_.tex, ImVec2(dispW, dispH));
        ImGui::SetCursorScreenPos(o);
        ImGui::InvisibleButton("##anchorpick", ImVec2(dispW, dispH),
                               ImGuiButtonFlags_MouseButtonLeft);
        // Track while held, not just on release: dragging the crosshair onto
        // the feature is how a sub-pixel pick actually gets made.
        if (ImGui::IsItemActive()) {
            const ImVec2 mp = ImGui::GetIO().MousePos;
            n.anchorOffsetX = static_cast<double>((mp.x - o.x) / z) - static_cast<double>(tw) * 0.5;
            n.anchorOffsetY = static_cast<double>((mp.y - o.y) / z) - static_cast<double>(th) * 0.5;
            commit();
        }

        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 c0(o.x + dispW * 0.5f, o.y + dispH * 0.5f);   // the patch centre
        dl->AddLine(ImVec2(o.x, c0.y), ImVec2(o.x + dispW, c0.y), IM_COL32(255, 255, 255, 40), 1.0f);
        dl->AddLine(ImVec2(c0.x, o.y), ImVec2(c0.x, o.y + dispH), IM_COL32(255, 255, 255, 40), 1.0f);
        const ImVec2 ac(c0.x + static_cast<float>(n.anchorOffsetX) * z,
                        c0.y + static_cast<float>(n.anchorOffsetY) * z);
        dl->AddLine(ImVec2(ac.x - 14.0f, ac.y), ImVec2(ac.x + 14.0f, ac.y), kAnchorCol, 1.5f);
        dl->AddLine(ImVec2(ac.x, ac.y - 14.0f), ImVec2(ac.x, ac.y + 14.0f), kAnchorCol, 1.5f);
        dl->AddCircle(ac, 4.0f, kAnchorCol, 0, 1.5f);
        ImGui::EndChild();

        // Focused = this window or its child, so the nudge works whether the
        // last click landed on the image or on a control below it.
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) {
            const double step = ImGui::GetIO().KeyShift ? 0.1 : 1.0;
            bool moved = false;
            if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))  { n.anchorOffsetX -= step; moved = true; }
            if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) { n.anchorOffsetX += step; moved = true; }
            if (ImGui::IsKeyPressed(ImGuiKey_UpArrow))    { n.anchorOffsetY -= step; moved = true; }
            if (ImGui::IsKeyPressed(ImGuiKey_DownArrow))  { n.anchorOffsetY += step; moved = true; }
            if (moved) commit();
        }

        ImGui::Text("anchor %+.2f, %+.2f px%s", n.anchorOffsetX, n.anchorOffsetY,
                    n.useAnchor ? "" : "   (centre)");
        ImGui::TextDisabled("template %d x %d px", visionTmpl_.w, visionTmpl_.h);

        ImGui::SetNextItemWidth(160.0f);
        ImGui::SliderFloat("zoom", &visionAnchorZoom_, 1.0f, 32.0f, "x%.1f",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SameLine();
        if (ImGui::Button("Fit")) visionAnchorZoom_ = 0.0f;

        if (ImGui::Button("Centre")) {
            n.anchorOffsetX = n.anchorOffsetY = 0.0;
            n.useAnchor = false;
        }
        ImGui::SameLine();
        // Save here, not only on the page's Save: the pick is the whole reason
        // this window exists, and an anchor lost to a crash is a re-teach.
        if (ImGui::Button("Done")) {
            session_.SaveVision();
            workflow_.InvalidateDetector(m.uid);   // next detect rebuilds with the new anchor
            visionAnchorOpen_ = false;
        }
    }
    ImGui::End();
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
                        // The file name and the size can both come back
                        // unchanged from a re-teach, so the key alone would not
                        // notice. Drop every thumbnail keyed on this patch —
                        // the Alignment page's and the pick window's included,
                        // or they keep showing the patch that was just replaced.
                        visionTmpl_.key.clear();
                        alignTmpl_[0].key.clear();
                        alignTmpl_[1].key.clear();
                        pickTmpl_.key.clear();
                        visionErr_.clear();
                        visionHaveDet_ = false;
                        // Straight into the anchor pick: the anchor is the
                        // point the model REPORTS, and a patch cut around a
                        // feature has just had it reset to the box centre.
                        OpenVisionAnchorPicker();
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
        if (visionDet_.method == "corner" && visionDet_.widthPx > 0.0) {
            // The detected square, drawn from the reported corner outwards: the
            // angle points at the centre and the two edges leave at +-45 from
            // it. Not the generic centred box below — for a corner that box
            // would straddle the point and show a square that is not there.
            const double b = visionDet_.angleDeg * 3.14159265358979323846 / 180.0;
            const double a1 = b - 0.78539816339744831, a2 = b + 0.78539816339744831;
            const ImVec2 e1(static_cast<float>(std::cos(a1) * visionDet_.widthPx) * view.scale,
                            static_cast<float>(std::sin(a1) * visionDet_.widthPx) * view.scale);
            const ImVec2 e2(static_cast<float>(std::cos(a2) * visionDet_.heightPx) * view.scale,
                            static_cast<float>(std::sin(a2) * visionDet_.heightPx) * view.scale);
            const ImVec2 quad[4] = {c,
                                    ImVec2(c.x + e1.x, c.y + e1.y),
                                    ImVec2(c.x + e1.x + e2.x, c.y + e1.y + e2.y),
                                    ImVec2(c.x + e2.x, c.y + e2.y)};
            dl->AddPolyline(quad, 4, kHitCol, ImDrawFlags_Closed, 2.0f);
        } else if (visionDet_.radiusPx > 0.0) {
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
    int method = (m.method == "ncc") ? 1 : (m.method == "corner") ? 2 : 0;
    const char* methods[] = {"Circle / fiducial", "Pattern (NCC)", "Square corner"};
    ImGui::SetNextItemWidth(itemW);
    if (ImGui::Combo("method", &method, methods, 3)) {
        m.method = (method == 1) ? "ncc" : (method == 2) ? "corner" : "circle";
        visionHaveDet_    = false;
        visionAnchorOpen_ = false;   // the picker belongs to the NCC form
    }

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
    } else if (m.method == "corner") {
        CornerParams& c = m.corner;
        // Mode first: it decides which half of this form even applies, and
        // "two edges" is the one that works on a corner attached to something
        // (a bar end, a die edge) rather than an isolated square.
        int cmode = (c.mode == 0) ? 1 : 0;
        const char* modes[] = {"two edges (no teaching)", "solid square (blob)"};
        ImGui::SetNextItemWidth(itemW);
        if (ImGui::Combo("shape", &cmode, modes, 2)) {
            c.mode = (cmode == 0) ? 1 : 0;
            visionHaveDet_ = false;
        }

        const char* pol[] = {"dark material on light", "bright material on dark"};
        ImGui::SetNextItemWidth(itemW);
        ImGui::Combo("contrast", &c.polarity, pol, 2);

        // Shared by both modes, and it means the same thing in each: which
        // corner OF THE FEATURE comes back as the measurement.
        const char* corners[] = {"top-left", "top-right", "bottom-right", "bottom-left"};
        ImGui::SetNextItemWidth(itemW);
        ImGui::Combo("report corner", &c.corner, corners, 4);

        if (c.mode == 1) DrawCornerEdgeForm(c, itemW, mmX);
        else             DrawCornerBlobForm(c, itemW, mmX);
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

        if (EnsurePatchTex(visionTmpl_, m)) {
            // A PREVIEW of the patch and where the anchor sits on it. Picking
            // happens in DrawVisionAnchorWindow, not here: at this scale one
            // screen pixel is several template pixels, so a click was a coarse
            // guess dressed up as a measurement — and a stray one silently
            // moved the point the model reports.
            const float maxW = ImGui::GetContentRegionAvail().x;
            const float sc   = fminf(fminf(maxW / static_cast<float>(visionTmpl_.w), 4.0f),
                                     160.0f / static_cast<float>(visionTmpl_.h));
            const ImVec2 sz(static_cast<float>(visionTmpl_.w) * sc,
                            static_cast<float>(visionTmpl_.h) * sc);
            const ImVec2 tOrigin = ImGui::GetCursorScreenPos();
            ImGui::Image((ImTextureID)(intptr_t)visionTmpl_.tex, sz);
            ImGui::SetCursorScreenPos(tOrigin);
            if (ImGui::InvisibleButton("##anchorPick", sz)) OpenVisionAnchorPicker();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Click to open the anchor picker - the anchor is the\n"
                                  "point this model REPORTS. It rides with the match,\n"
                                  "rotating by the match angle, so it stays on the feature.");
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
            if (ImGui::Button("Set anchor...")) OpenVisionAnchorPicker();
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
            else if (m.method == "corner")
                ImGui::Text("sides %.1f x %.1f px  (%.4f mm)  corner->centre %.1f deg",
                            visionDet_.widthPx, visionDet_.heightPx,
                            visionDet_.lengthPx * mmX, visionDet_.angleDeg);
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
        if (ImGui::Button("Back")) SwitchPage(visionReturnPage_);
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
        SwitchPage(visionReturnPage_);   // whoever opened this page owns the exit
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

    // Floating, so it is drawn outside the two body children — a window begun
    // inside a child would inherit its clip rect.
    DrawVisionAnchorWindow(*m);

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
