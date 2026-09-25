// AlignFrame.h : the Alignment Procedure document and the local frame it teaches.
//
// A part is never loaded twice in exactly the same place, so a machine
// coordinate taught against one placement is wrong for the next. An alignment
// procedure fixes the *reference* instead of the positions: the operator marks
// two features on the real part — E1 and E2 — at two gantry stations, and the
// pair defines a rigid 2-D frame O1:
//
//     origin = E1,  local X+ = E1 -> E2,  local Y+ = X+ rotated +90 degrees
//
// Coordinates expressed in O1 stay valid whatever the part's actual placement:
// run the procedure once per part, then every downstream move is precise.
//
// The frame is deliberately rigid — origin + rotation about Z, no scale and no
// tilt. The measured E1->E2 span is recorded as a diagnostic so a mis-pick is
// visible rather than absorbed.
//
// Persistence follows core/SavedPositions.{h,cpp} exactly: the product's
// {uid}.alignments.json when a product is loaded, else
// config/alignments.local.json with an alignments.json fallback. Tolerant load
// — a missing or malformed file yields an empty vector, never throws.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

// Illumination for a station. PLACEHOLDER: no illumination device exists in the
// hardware config yet, so an empty `kind` (the default) means "leave lighting
// alone" and the run applies nothing. The shape is here so a procedure taught
// today keeps its settings when a real light is wired up.
struct AlignLighting {
    std::string kind;             // "" (none) | "io" | "psu"
    std::string device;           // io device name, or "" for the PSU
    std::string channel;          // io pin name, or PSU channel number as text
    double      value = 0.0;      // io: non-zero = on;  psu: volts
};

// One of the two stations the procedure visits.
struct AlignStation {
    std::string   posUid;                  // SavedPos uid on the procedure's device
    double        exposureUs = 0.0;        // 0 = leave the camera exposure alone
    AlignLighting lighting;
    std::string   mode = "manual";         // "manual" (crosshair) | "auto" (detector)
    std::string   detector;                // auto placeholder: model/pattern name
    bool          autoFallbackManual = false;   // auto unavailable -> ask the operator
};

// One captured E point. The world point is what the frame is built from; the
// gantry pose and the raw pixel are kept so a capture can be audited later.
struct AlignPoint {
    double gx = 0.0, gy = 0.0;    // gantry XY at capture (mm, machine frame)
    double px = 0.0, py = 0.0;    // crosshair position, absolute camera pixels
    double wx = 0.0, wy = 0.0;    // world point = gantry + pixel offset (mm)
    // Gantry Z at capture, AXIS space, read straight off the device - the
    // height the camera was focused at when this point was marked. `hasZ` is
    // false for a device with no Z and for every point written before Z was
    // recorded; a missing Z must never read as Z = 0.
    double gz = 0.0;
    bool   hasZ = false;
};

// The taught frame. `valid` is false until a solve succeeds.
struct AlignResult {
    bool        valid = false;
    std::string capturedAt;                // "2026-08-01T10:22:31"
    AlignPoint  e1, e2;
    double      originX = 0.0, originY = 0.0;   // == e1.wx / e1.wy
    double      thetaDeg = 0.0;                 // E1 -> E2 direction
    double      spanMm = 0.0;                   // |E1 -> E2|, diagnostic
    // The frame's working height: the MEAN gantry Z of E1 and E2, in the
    // device's own axis space (never through AxisTransform - gantry-main
    // inverts Z). The frame is 2-D, so this is not transformed by it; it is the
    // focus height the part was measured at, which is what move_to_nominal
    // drives to by default. `hasZ` only when BOTH points carried one.
    double      z = 0.0;
    bool        hasZ = false;
};

// ---- the guard band, and the record of what each run measured -------------
// A solved frame is always geometrically valid — two points that are not on top
// of each other always define an origin and a direction. What `SolveAlignFrame`
// cannot know is whether they are the RIGHT two points: E2 marked on the wrong
// feature yields a frame wrong by the size of that feature, published as LIVE,
// and every move_local after it wrong by the same amount with nothing on screen
// to say so.
//
// The band is the missing sanity check, and it is deliberately TWO numbers of
// different character:
//
//   span  is a fact about the PART — the distance between the two features is
//         on the drawing, it is the same on every part, so its tolerance is
//         tight and a breach means the wrong feature was marked;
//   theta is a fact about the LOADING — how the part happens to sit differs
//         every time, legitimately. Its reference is a nominal orientation
//         (usually 0: square to the machine), not a measurement, and its band
//         is wide. It catches a part in backwards or a slipped clamp.
//
// Off by default, so every procedure authored before this behaves exactly as it
// did. An enabled band never REFUSES a frame by itself: the check surfaces
// while E2 is being marked, where the crosshair can still be moved, and an
// operator who accepts a mark outside it gets a flagged run rather than a
// blocked machine.
struct AlignGuard {
    bool   enabled     = false;
    double spanMm      = 0.0;   // reference E1->E2 distance
    double spanTolMm   = 0.2;
    double thetaDeg    = 0.0;   // nominal orientation, not a measured one
    double thetaTolDeg = 5.0;
};

// One solved frame, kept as a row. E1 and E2 are machine mm — the same numbers
// the frame panel shows — and dx/dy is E2-E1, which is what a band breach is
// actually about. `guarded` records whether a band was on AT THE TIME: an `ok`
// that nothing checked must not read like one that passed.
struct AlignRun {
    std::string at;                                 // == AlignResult::capturedAt
    double e1x = 0.0, e1y = 0.0, e2x = 0.0, e2y = 0.0;
    double dx = 0.0, dy = 0.0;
    double spanMm = 0.0, thetaDeg = 0.0;
    bool   guarded = false;
    bool   ok      = true;
};

// The most marks a nominal record can hold (p1..p4). The teach page asks for at
// most kAlignNominalTeachMax; the fourth slot exists for older files.
inline constexpr int kAlignNominalMarksMax = 4;
inline constexpr int kAlignNominalTeachMax = 3;

// One taught NOMINAL: a target point on the part, held in LOCAL (O1)
// coordinates so it travels with the part instead of with the machine. Teach it
// once against a part whose frame was just taught, and every later part gets
// the same point at its own placement, because the frame moved and the local
// coordinate did not.
//
// It is taught as the AVERAGE of 1, 2 or 3 crosshair marks. One mark is the
// point itself. Two or three are there because the interesting point is often
// the CENTRE of something — a die, a pocket, a footprint — with no feature under
// it to aim at: mark corners and the centre is arithmetic, not judgement. The
// same number of marks on ONE feature averages the pick noise down instead.
//
// Only `lx`/`ly` are consumed. Everything below them is the record of how the
// point was taught, kept for the same reason AlignResult keeps its E points: a
// suspicious number has to be auditable without re-teaching.
struct AlignNominal {
    std::string uid;
    std::string name = "nominal";
    double      lx = 0.0, ly = 0.0;   // LOCAL (O1) mm — the only field a run uses
    // The marks it was taught from, machine mm. The page asks for 1..3; `p4`
    // and `points == 4` survive only because files taught on the older 2-or-4
    // page carry them, and a file must go on reading.
    AlignPoint  p1, p2, p3, p4;
    int         points = 2;           // 1..4 — how many of the above are real
    double      mx = 0.0, my = 0.0;   // CENTRE, machine mm, AT TEACH TIME
    // One mark: 0. Two: the separation. Three or more: twice the mean distance
    // from the centre, so one number means the same thing either way — a width
    // across the marks. Diagnostic, like AlignResult::spanMm.
    double      spanMm = 0.0;
    std::string taughtAt;             // "2026-08-01T10:22:31"
    std::string frameAt;              // capturedAt of the frame it was converted through

    // Where the CAMERA stands, focused, to look at this nominal ITSELF — the
    // bare target on the substrate, with nothing gripped over it. A private
    // position like `viewUid`, named `algn_<procUid>_<nomUid>_c`.
    //
    // It is NOT `viewUid` and must never be merged with it. That one is the view
    // of a gripped OBJECT hovering above the point: a different XY, and a
    // different Z, because the height that focuses a part held in the jaws does
    // not focus the substrate under it. Teaching one from the other is exactly
    // the mistake `viewUid`'s tooltip already warns about, and merging the two
    // fields would make that mistake unavoidable rather than merely easy.
    //
    // Its Z is the whole point of storing it: the local frame is 2-D, so
    // local->machine hands back XY and nothing else. The focused Z is a
    // measurement of where the substrate is at this point, and the only way to
    // get it back is to have written it down when it was in focus.
    std::string camUid;

    // How the OBJECT's two marks are found when something is placed here.
    // "manual" = ask the operator; "auto" = run the taught detection model in
    // `detN`, named `vdet_<procUid>_<nomUid>_p<N>` on the same private-ownership
    // rule as an alignment station's detector.
    //
    // The mode is its own field, exactly like AlignStation::mode, rather than
    // being read off `detN` being non-empty: a taught patch and its parameters
    // cost real time at the microscope, so choosing manual for one part must not
    // destroy them. Manual keeps the model on the shelf; detect picks it back up.
    //
    // The fallback defaults ON here, unlike a station's, because a placement is
    // a per-part operation an operator is usually standing over: failing the
    // whole run beats nothing, but asking beats both. It is never SILENT — a
    // fallback logs a warning naming what failed, so a procedure cannot quietly
    // claim to be automatic while a human places every point.
    std::string mode1 = "manual", mode2 = "manual";
    std::string det1, det2;
    bool        autoFallbackManual = true;

    // Where the CAMERA has to stand to see the object being placed here. A
    // saved-position uid on the procedure's own device, private to this nominal
    // and named `algn_<procUid>_<nomUid>_v` on the same ownership rule as a
    // station's position.
    //
    // It is NOT the nominal, and usually shares no coordinate with it: the
    // object is picked up somewhere else, and the height that focuses on a
    // gripped part is not the height that focuses on the substrate. Without it
    // every placement needed a hand-driven camera or an extra move_to_position
    // step in front of it, and the nominal — the thing that knows where its
    // object is looked at — knew nothing about it.
    //
    // Empty = not taught, and `place_view` then leaves the camera alone: a
    // procedure written before this field must keep working.
    std::string viewUid;
};

// The document: a named, uid'd, reusable procedure.
struct AlignProc {
    std::string  uid;
    std::string  name = "alignment";
    std::string  device = "gantry-main";   // motion device the stations belong to
    AlignStation s1, s2;
    AlignResult  result;
    AlignGuard   guard;                    // what a solved frame is checked against
    std::vector<AlignNominal> nominals;    // taught targets in this frame
    // The last kAlignRunHistoryMax solved frames, oldest first. A RECORD, not a
    // licence: unlike `result` it survives every boundary that clears a frame,
    // because the question it answers ("what did the last hundred parts
    // measure?") is exactly the one asked when one of them comes out wrong.
    std::vector<AlignRun> runs;
};

// ---- frame maths ---------------------------------------------------------
// The only geometry code in the project: AxisTransform (hardware/HardwareConfig.h)
// is a signed permutation and cannot hold an arbitrary angle or an offset.

// Smallest E1->E2 separation that yields a trustworthy angle. Below this the
// pixel-pick noise dominates the direction, so a solve is refused rather than
// producing a frame that is quietly rotated by degrees.
inline constexpr double kAlignMinSpanMM = 0.05;

// How many solved frames a procedure remembers. Bounded because it rides the
// product document — and set to the longest repeatability sweep the page will
// run, so a whole sweep's rows are still there to compute a Cpk from when it
// finishes. Anything smaller silently drops the beginning of the evidence.
inline constexpr std::size_t kAlignRunHistoryMax = 100;

// The repeatability sweep's bounds. Below three there is no spread to speak of
// and a Cpk would be arithmetic dressed up as evidence; above a hundred the
// sweep outlives both the operator's attention and the ring above.
inline constexpr int kAlignRepeatMin     = 3;
inline constexpr int kAlignRepeatMax     = 100;
inline constexpr int kAlignRepeatDefault = 30;

// Fill `out` (origin, angle, span, timestamp, valid) from the two marked
// points. False + `err` when the two points are too close together.
bool SolveAlignFrame(const AlignPoint& e1, const AlignPoint& e2,
                     AlignResult* out, std::string* err = nullptr);

// ---- the guard band ------------------------------------------------------
// The span and direction of E1->E2, without the refusal or the timestamp
// SolveAlignFrame carries. It exists so a CANDIDATE point — a crosshair the
// operator is still moving — can be measured against the band without pretending
// to be a frame. Either out may be null.
void AlignSpanTheta(const AlignPoint& e1, const AlignPoint& e2,
                    double* spanMm, double* thetaDeg);

// True when (spanMm, thetaDeg) is inside `g`. `spanErr`/`thetaErr` come back as
// SIGNED deviations from the reference (measured - reference), because "0.4 mm
// too long" and "0.4 mm too short" are different mistakes. Both outs may be null.
//
// A DISABLED band returns true and zero deviations: an inactive band never
// refuses anything, and every caller can therefore ask unconditionally.
bool AlignGuardCheck(const AlignGuard& g, double spanMm, double thetaDeg,
                     double* spanErr = nullptr, double* thetaErr = nullptr);

// The history row for a solved frame, verdict included. One function so the
// page, the command and the session cannot disagree about what a run measured.
AlignRun MakeAlignRun(const AlignGuard& g, const AlignResult& r);

// What a series of measurements did against one half of the band. The band's
// tolerance IS the specification limit, which is the whole reason a capability
// figure belongs here rather than in a spreadsheet: the operator already
// decided what a good part may vary by when they set the band.
//
//   Cp  = tol / (3 sigma)                  — the spread alone, if it were centred
//   Cpk = (tol - |mean - ref|) / (3 sigma) — the spread AND where it sits
//
// Cp is the capability the process could have; Cpk is the one it has. Reporting
// only Cp hides a tight distribution sitting against a limit — which is exactly
// what a detector locked onto the wrong edge looks like.
//
// sigma is the SAMPLE deviation (n-1). Vision reliability uses the population
// form for a different job (describing one sitting); a capability index is an
// estimate about future parts from a sample, so it takes the sample form.
struct AlignCpk {
    int    n     = 0;
    double mean  = 0.0;
    double sigma = 0.0;
    double minV  = 0.0, maxV = 0.0;
    double cp    = 0.0, cpk = 0.0;
    bool   valid = false;   // n >= 2 and tol > 0
    // Every sample identical to the last digit. Cp/Cpk are infinite rather than
    // huge, and a printed 1e9 would read as a measurement rather than as the
    // resolution limit it is.
    bool   noSpread = false;
};

AlignCpk ComputeAlignCpk(const std::vector<double>& values, double ref, double tol);

// Local (O1) <-> machine XY. Both are no-ops when the frame is not valid, so a
// caller that forgets to check gets the identity, never a silent garbage move.
void AlignLocalToMachine(const AlignResult& r, double lx, double ly,
                         double* mx, double* my);
void AlignMachineToLocal(const AlignResult& r, double mx, double my,
                         double* lx, double* ly);

// Midpoint of two marks, converted through `frame` into local (O1), plus the
// record fields. `out`'s uid and name are left alone — the caller owns identity.
//
// An INVALID frame is refused rather than passed through: AlignMachineToLocal
// is deliberately an identity in that case, so a nominal taught through one
// would silently be a machine coordinate wearing a local coordinate's name, and
// would then be re-projected by the next frame into somewhere no part has ever
// been. There is no minimum span here (unlike the frame): a nominal is a point,
// so two coincident marks are merely a point, not a meaningless direction.
// The N-mark form: 1..kAlignNominalMarksMax marks, whose AVERAGE is the target.
// The teach page offers 1, 2 or 3; 4 is accepted because older files hold it.
bool SolveAlignNominalN(const AlignResult& frame, const AlignPoint* pts, int n,
                        AlignNominal* out, std::string* err);

// The two-mark form, kept because most callers have exactly two and because
// every file written before four existed says two. Delegates.
bool SolveAlignNominal(const AlignResult& frame, const AlignPoint& p1,
                       const AlignPoint& p2, AlignNominal* out,
                       std::string* err = nullptr);

// ---- lookup --------------------------------------------------------------
// Resolve by uid first, then by exact name — recipes reference the uid (rename-
// safe), scripts and operators use the name.
const AlignProc* FindAlignProc(const std::vector<AlignProc>& list,
                               const std::string& uidOrName);
AlignProc*       FindAlignProc(std::vector<AlignProc>& list,
                               const std::string& uidOrName);

// Same rule one level down: a nominal is addressed by uid or by name, within
// the procedure that owns it.
const AlignNominal* FindAlignNominal(const AlignProc& p, const std::string& uidOrName);
AlignNominal*       FindAlignNominal(AlignProc& p, const std::string& uidOrName);

// ---- persistence ---------------------------------------------------------
// Round-trip helpers, also used for the session's dirty baseline (a canonical
// dump compared against the state at load/save — the recipes convention).
nlohmann::json AlignProcToJson(const AlignProc& p);
AlignProc      AlignProcFromJson(const nlohmann::json& j);

// Reads config/alignments.local.json, falling back to config/alignments.json.
std::vector<AlignProc> LoadAlignProcs();

// Explicit-path variants used by the product system. `uidOut` (if given)
// receives the file's own "uid" field so the caller can check it against the
// product manifest; `uid` (if non-empty) is written into the file.
std::vector<AlignProc> LoadAlignProcsFile(const std::string& path,
                                          std::string* uidOut = nullptr);
bool SaveAlignProcsFile(const std::string& path, const std::vector<AlignProc>& procs,
                        const std::string& uid);

} // namespace firedog
