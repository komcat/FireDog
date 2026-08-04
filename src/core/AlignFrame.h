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
};

// The taught frame. `valid` is false until a solve succeeds.
struct AlignResult {
    bool        valid = false;
    std::string capturedAt;                // "2026-08-01T10:22:31"
    AlignPoint  e1, e2;
    double      originX = 0.0, originY = 0.0;   // == e1.wx / e1.wy
    double      thetaDeg = 0.0;                 // E1 -> E2 direction
    double      spanMm = 0.0;                   // |E1 -> E2|, diagnostic
};

// One taught NOMINAL: a target point on the part, held in LOCAL (O1)
// coordinates so it travels with the part instead of with the machine. Teach it
// once against a part whose frame was just taught, and every later part gets
// the same point at its own placement, because the frame moved and the local
// coordinate did not.
//
// It is taught as the MIDPOINT of two operator-marked features. Two marks
// rather than one because the interesting point is usually the CENTRE of
// something — a die, a pocket, a footprint — and a centre is a place with no
// feature under it to aim at. Marking two opposite corners puts the crosshair
// on something real both times; the midpoint is then arithmetic, not judgement.
//
// Only `lx`/`ly` are consumed. Everything below them is the record of how the
// point was taught, kept for the same reason AlignResult keeps its E points: a
// suspicious number has to be auditable without re-teaching.
struct AlignNominal {
    std::string uid;
    std::string name = "nominal";
    double      lx = 0.0, ly = 0.0;   // LOCAL (O1) mm — the only field a run uses
    AlignPoint  p1, p2;               // the two marks, machine mm
    double      mx = 0.0, my = 0.0;   // midpoint, machine mm, AT TEACH TIME
    double      spanMm = 0.0;         // |p1->p2| — diagnostic, like AlignResult::spanMm
    std::string taughtAt;             // "2026-08-01T10:22:31"
    std::string frameAt;              // capturedAt of the frame it was converted through

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
    std::vector<AlignNominal> nominals;    // taught targets in this frame
};

// ---- frame maths ---------------------------------------------------------
// The only geometry code in the project: AxisTransform (hardware/HardwareConfig.h)
// is a signed permutation and cannot hold an arbitrary angle or an offset.

// Smallest E1->E2 separation that yields a trustworthy angle. Below this the
// pixel-pick noise dominates the direction, so a solve is refused rather than
// producing a frame that is quietly rotated by degrees.
inline constexpr double kAlignMinSpanMM = 0.05;

// Fill `out` (origin, angle, span, timestamp, valid) from the two marked
// points. False + `err` when the two points are too close together.
bool SolveAlignFrame(const AlignPoint& e1, const AlignPoint& e2,
                     AlignResult* out, std::string* err = nullptr);

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
