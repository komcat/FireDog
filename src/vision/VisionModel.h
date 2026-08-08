// VisionModel.h : a taught feature detector, as a document.
//
// A detection model is everything needed to find one feature again: which part
// of the frame to look in, which method to use, that method's tuning, the
// taught template (pattern matching only) and a record of how reliably it last
// performed. It is named and uid'd so an alignment station — or, later, a
// recipe step — references it instead of carrying a copy of the settings.
//
// Deliberately a DOCUMENT, not autosaved state: explicit Save plus a dirty
// baseline, exactly like recipes and alignment procedures. Tuning a detector is
// an authoring act, and half-tuned parameters must not reach the next run.
//
// Persistence follows core/AlignFrame.{h,cpp} exactly — the product's
// {uid}.vision.json when a product is loaded, else config/vision.local.json
// with a config/vision.json fallback. Tolerant load: a missing or malformed
// file yields an empty vector, never throws.
//
// No vendor SDK reaches this header. The Halcon-backed detector that CONSUMES a
// model lives in vision/HalconDetector.{h,cpp}; a model can be listed, edited
// and saved on a machine with no HALCON at all.
#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

// The search window, in full-frame camera pixels. Disabled = search the whole
// frame. An ROI is the cheapest speed-up there is and usually the difference
// between a detector that can run live and one that cannot.
struct VisionRoi {
    bool enabled = false;
    int  x = 0, y = 0, w = 0, h = 0;
};

// ---- pre-processing ------------------------------------------------------
// One stage of the pre-process chain: a transform applied to the search ROI
// BEFORE any method looks at it. Some features only separate after a threshold,
// a smooth or an edge transform, and no amount of method tuning substitutes for
// that. Ported from Horizon's FAAM.UI vision profile, which is the same idea in
// production on the sibling machine.
//
// `kind` is a string like every other discriminator in this file ("method",
// "metric"), so an unknown value from a newer build reads as a no-op rather
// than a crash. The parameter fields are SHARED across kinds — min/max is the
// grey band for "binary" and the hysteresis pair for "canny" — which is why the
// UI relabels them rather than showing five dead boxes.
struct VisionFilter {
    std::string kind = "none";   // "none" | "binary" | "gaussian" | "canny"
    int    thresholdMin = 0;     // binary: min grey   | canny: hysteresis low
    int    thresholdMax = 128;   // binary: max grey   | canny: hysteresis high
    int    gaussMask    = 3;     // gaussian only: 3 or 5
    double alpha        = 1.0;   // canny only: smoothing, 0.2 .. 5.0
};

// Three slots, fixed. The count IS the feature — enough to threshold, smooth
// and take edges in one pass, few enough that the whole chain is on screen —
// and a fixed slot gives each row a stable identity across a re-order.
inline constexpr int kVisionFilterSlots = 3;

// Circle / fiducial, the blob-then-robust-fit chain. Ported from
// bobcat_engine's HalconCircleDetectionRobustPlugin, which is the variant in
// production use there (the sub-pixel-edge variant does not filter on
// circularity and picks up far more junk).
struct CircleParams {
    int    thresholdMin   = 0;      // grey window for the feature ...
    int    thresholdMax   = 100;    // ... polarity decides which side is used
    double closingRadius  = 10.5;   // morphological close, fills speckle gaps
    int    minArea        = 500;
    int    maxArea        = 999999;
    double minCircularity = 0.7;
    int    polarity       = 0;      // 0 = dark feature on light, 1 = bright on dark
    int    fitMethod      = 0;      // 0 = atukey, 1 = geotukey (both robust)
};

// Gray normalised-cross-correlation pattern match, plus an operator-placed
// anchor. The anchor is what makes a match usable as a coordinate: the point of
// interest is rarely the centre of a legible template.
struct NccParams {
    double      minScore   = 0.7;
    int         maxMatches = 1;
    int         numLevels  = 0;     // 0 = let HALCON choose the pyramid depth
    double      angleToleranceDeg = 0.0;   // 0 = no rotation search (fastest)
    std::string metric     = "ignore_global_polarity";   // | "use_polarity"

    // Anchor offset RELATIVE TO THE TEMPLATE CENTRE, in template pixels. It is
    // rotated by the match angle before being applied, so it stays attached to
    // the feature rather than to the screen.
    bool   useAnchor = false;
    double anchorOffsetX = 0.0, anchorOffsetY = 0.0;

    int tmplW = 0, tmplH = 0;       // taught patch size, for the UI and sanity checks
};

// A 90-degree corner, in two flavours — because "a corner" in a real scene is
// usually NOT a free-standing square.
//
// MODE 1, TWO EDGES (the default). The template is SYNTHESISED IN MEMORY from
// these parameters: a patch with one quadrant filled, matched by NCC. Nothing is
// taught and nothing is stored on disk — a corner is a geometric primitive, and
// needing a photograph of one is a maintenance cost with no information in it.
// This is the mode for the end of a bar, the corner of a die, the meeting of two
// straight edges: the feature only has to be locally right, so a region that
// runs off the ROI (which defeats any blob search) is fine.
//   * `sizePx` is the template size, i.e. how much of each edge is matched. Keep
//     it inside the shortest straight run either edge has: for a bar end, no
//     more than the bar's thickness.
//
// MODE 0, SOLID SQUARE. The blob chain, for a printed or machined square mark
// that really is isolated: threshold -> close -> connect -> an area window of
// side^2 over [1-tol .. 1+tol] -> rectangularity (1.0 for a perfect rectangle at
// ANY rotation, so it does not assume the part is square to the camera) ->
// sub-pixel rectangle fit, then the named corner. The four are classified in
// image terms AFTER the fit, so a rotated part still reports the one picked.
//
// `corner` means the SAME THING in both modes: which corner of the feature the
// detector reports. In mode 1 the material therefore fills the OPPOSITE quadrant
// of the synthesised patch — the bottom-left corner of a shape has the shape
// up-and-right of it. Naming the filled quadrant instead was the first cut, and
// it was wrong the way an inverted axis is wrong: every operator read
// "bottom-left" as the corner they were marking, which is the diagonal opposite
// of the quadrant that fills.
struct CornerParams {
    int    mode          = 1;      // 1 = two-edge template, 0 = solid square blob
    int    polarity      = 0;      // 0 = dark material on light, 1 = bright on dark
    double sizePx        = 40.0;   // template size (mode 1) / square side (mode 0), px
    int    corner        = 0;      // reported corner: 0 TL, 1 TR, 2 BR, 3 BL

    // ---- mode 1 -----------------------------------------------------------
    double minScore      = 0.5;    // NCC score floor
    double angleToleranceDeg = 5.0;   // rotation search either way; 0 = none (fastest)
    double smoothPx      = 1.0;    // soften the synthetic edges to match a real one

    // ---- mode 0 -----------------------------------------------------------
    bool   autoThreshold = true;   // per-frame max_separability split
    int    threshold     = 128;    // used only when autoThreshold is off
    double sizeTolPct    = 40.0;   // +/- on the side; the area window derives from it
    double closingRadius = 2.0;    // fills speckle before the shape filters
    double minRectangularity = 0.75;
};

// The mode-1 template, synthesised from the parameters: a square patch with the
// quadrant OPPOSITE `corner` filled, edges softened, so the named corner lands
// exactly at the patch centre and the match reports it with no anchor. `n`
// receives the side actually used (even, clamped to 8..256).
//
// Here rather than in HalconDetector because the teach page draws the same thing
// as a diagram: two implementations of "which quadrant fills" is exactly how the
// picture and the search end up disagreeing.
int CornerTemplateSide(const CornerParams& c);
std::vector<unsigned char> MakeCornerTemplate(const CornerParams& c, int* n);

// What the last reliability run measured. A RECORD, never a gate: the run tells
// an operator whether the model is trustworthy, and refusing to save a model
// that scored badly would just mean tuning it somewhere else.
struct VisionReliability {
    int    runs = 0, ok = 0;
    double meanScore = 0.0;
    double meanXPx = 0.0, meanYPx = 0.0;
    double stdXPx  = 0.0, stdYPx  = 0.0;
    double maxDevPx = 0.0;          // worst distance from the mean point
    std::string testedAt;           // "2026-08-01T10:22:31"
};

struct VisionModel {
    std::string uid;
    std::string name   = "detect";
    std::string method = "circle";     // "circle" | "ncc" | "corner"

    VisionRoi roi;
    double    exposureUs = 0.0;        // 0 = leave the camera exposure alone

    // Applied in slot order to the ROI before the method runs; "none" slots are
    // skipped, so 1 and 3 set with 2 empty is the two-stage chain [1,3]. Part of
    // the MODEL, not of the page that edits it: the same chain has to be in
    // force when a recipe's align_station detects, or a model taught through a
    // threshold would find nothing at run time.
    VisionFilter filters[kVisionFilterSlots];

    // All three sets are carried whatever the method is, so switching method to
    // compare two approaches does not throw the other one's tuning away.
    CircleParams circle;
    NccParams    ncc;
    CornerParams corner;

    // Taught template patch, "<uid>.pgm", RELATIVE to the model directory.
    // Relative on purpose: bobcat stores an absolute path, which makes a
    // product folder non-relocatable the moment it is copied to another PC.
    std::string templateFile;

    VisionReliability rel;
};

// The active chain as one short line: "gauss[3] > canny[20/80 a1.0]", or "" when
// no slot is enabled. One formatter, three consumers — the log, the Alignment
// page's station card, and the thumbnail cache key — so what an operator reads
// on the card is exactly what the detector will run.
std::string VisionFilterSummary(const VisionModel& m);

// ---- the taught patch ----------------------------------------------------
// 8-bit binary PGM (P5). Plain C++, ~20 lines: FireDog carries no image codec,
// and a format any viewer opens means a taught template can still be inspected
// on a machine with no HALCON licence. `gray` is w*h bytes, row-major.

bool SaveGrayPgm(const std::string& path, const unsigned char* gray, int w, int h);
bool LoadGrayPgm(const std::string& path, std::vector<unsigned char>* gray,
                 int* w, int* h);

// ---- lookup --------------------------------------------------------------
// Resolve by uid first, then by exact name — stations store the uid (so a
// rename is safe), operators and scripts use the name.
const VisionModel* FindVisionModel(const std::vector<VisionModel>& list,
                                   const std::string& uidOrName);
VisionModel*       FindVisionModel(std::vector<VisionModel>& list,
                                   const std::string& uidOrName);

// ---- persistence ---------------------------------------------------------
// Round-trip helpers, also used for the session's dirty baseline (a canonical
// dump compared against the state at load/save — the recipes convention).
nlohmann::json VisionModelToJson(const VisionModel& m);
VisionModel    VisionModelFromJson(const nlohmann::json& j);

// Reads config/vision.local.json, falling back to config/vision.json.
std::vector<VisionModel> LoadVisionModels();

// Explicit-path variants used by the product system. `uidOut` (if given)
// receives the file's own "uid" field so the caller can check it against the
// product manifest; `uid` (if non-empty) is written into the file.
std::vector<VisionModel> LoadVisionModelsFile(const std::string& path,
                                              std::string* uidOut = nullptr);
bool SaveVisionModelsFile(const std::string& path,
                          const std::vector<VisionModel>& models,
                          const std::string& uid);

} // namespace firedog
