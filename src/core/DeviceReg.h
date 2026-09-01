// DeviceReg.h : where another device's tool tip is, in the camera device's space.
//
// The needle offset (core/NeedleCal.h) solves this problem INSIDE one device:
// the camera and the needle ride the same gantry, so the delta between them
// crosses no frame and three numbers describe it. A gripper on a hexapod is the
// same question across a device boundary, and it needs a rotation as well:
//
//     p_ref = R * h + t          predict: where the tip is, from the pose
//     h     = R^T * (p_ref - t)  command: what pose puts the tip there
//
// `h` is the registered device's raw axis XYZ; `p_ref` is the REFERENCE device's
// raw axis XYZ — the gantry the camera rides on. Both stay in axis space for
// exactly the reason NeedleCal does: CaptureWorldPoint already reports
// `wx = gx + pick.dxMM` there, read_position reads it, move_axis_to commands it,
// and gantry-main's configured transform inverts Z, so a number that took a
// detour through AxisTransform would descend where it meant to retreat.
//
// The two halves come from different places and go stale for different reasons,
// and keeping that straight is most of the design:
//
//   * R IS TAKEN FROM config/motion_transforms.json — the permutation composed
//     as M_ref * M_dev^T. It is a hand-typed ±1 matrix: exact for a device
//     bolted square, and approximate for one that is not. `rotSource` records
//     where it came from so nothing downstream can mistake it for a measurement,
//     and the CHECK is what puts a number on what it costs — a seeded rotation
//     predicts perfectly at the pose the tip was taught and drifts with distance
//     from it. That is the deal, and it is a fair one for an APPROACH.
//
//   * t IS MEASURED, by imprint. Touch the tool onto a coupon, mark the corners
//     of the mark it leaves, and the centre of that mark is the tip in XY while
//     the focused camera Z over it is the tip in Z — because the tip was resting
//     on the surface now in focus. A pick on a curved tool viewed from above
//     lands on its SILHOUETTE, which is not where it touches; an imprint is the
//     contact point by definition, and a flat mark on a flat surface is the best
//     case there is for both the focus metric and the crosshair.
//
// R belongs to the MOUNT; t belongs to the mount AND the tool. Change a gripper
// and the tip moves while the axis directions do not, so that costs one imprint
// and not a re-registration — `regAt` and `tipAt` are stamped separately so the
// page can say which half is old.
//
// Like NeedleCal, and unlike a taught frame, THE PERSISTED RECORD IS THE LICENCE
// TO MOVE: a frame describes one physical part that may have been swapped since,
// this describes tooling that has not. It is machine-level, not per-product, and
// lives in config/device_reg.local.json with a config/device_reg.json seed.
//
// What it is NOT good enough for is the final placement — a seeded rotation and
// one measured point get a tip close, nowhere near the 20 µm a placement
// converges to. placement_procedure.lua keeps its measure/correct/re-measure
// loop; this only means the loop lands on pass 1 instead of pass 2. See
// docs/device-registration-design.md.
#pragma once

#include <map>
#include <string>

#include <nlohmann/json.hpp>

namespace firedog {

// A point or a direction in one device's raw axis space. Local to this header:
// nothing else in the codebase carries a 3-vector, and this is the only place
// that multiplies one.
struct RegVec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

// How far the live rotations may sit from the ones the tip was taught at before
// a prediction refuses. The record describes one rotation and says nothing about
// any other — a hexapod's tip swings about its pivot.
inline constexpr double kDeviceRegRotTolDeg = 0.05;

struct DeviceReg {
    // Both halves present: a rotation AND a measured tip. Either alone predicts
    // nothing.
    bool valid = false;
    // A check pass re-measured the tip AWAY from the pose it was taught at and
    // agreed. With a seeded rotation this is the only thing that has ever put a
    // number on the mount error, so it matters more here than it would for a
    // measured one.
    bool verified = false;

    std::string device;                    // "hex-left" — the registered device
    std::string refDevice = "gantry-main"; // whose axis space p lives in

    // Columns are the registered device's axis directions expressed in the
    // reference device's axis space: R[i][j] is component i of column j.
    double  R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    // Empty until a rotation has been adopted. "axis-transforms" is the only
    // source today; the field exists so a measured one could arrive later
    // without every reader having to guess which it is holding.
    std::string rotSource;
    RegVec3 t;                             // tip position at device pose (0,0,0)

    // The rotations the fit was pinned to. Not compensated for — refused past
    // kDeviceRegRotTolDeg, because the record says nothing about a pose it never
    // saw. UNLESS the pivot is at the tip; see below.
    RegVec3 taughtRot;
    bool    hasRot = false;   // false when the device reports no U/V/W at all

    // The controller's pivot point coincides with the tool tip, so rotating
    // U/V/W turns the tool WITHOUT translating the point this record stores.
    // With that true, `t` describes every rotation and the guard above has
    // nothing to protect: refusing at 0.05 deg would blank a perfectly good tip
    // the moment somebody nudged a rotation axis, which is what it did.
    //
    // It is a property of how the hexapod is SET UP, not of the registration —
    // a PI pivot is a controller setting — so it lives here as a stated
    // assumption with a switch, rather than being hard-coded either way. Turn
    // it off for a device whose pivot is somewhere else, and the tip swings
    // again: then a record taken at one rotation really does describe no other.
    //
    // Defaults TRUE: both grippers on this machine are set up that way, and the
    // guard's failure mode is silent absence (no marker, refused moves) which
    // reads as a broken overlay rather than as a stated refusal.
    bool pivotAtTip = true;

    // ---- The teachable poses -------------------------------------------------
    // Three positions the operator sets once and the page drives back to, so a
    // re-teach after a tool change repeats the geometry instead of hunting for
    // the coupon and the mark from scratch.
    //
    // They live HERE and not in the product's saved positions, deliberately:
    // saved positions belong to a product document and this is machine state, so
    // a pose held there would vanish whenever a different product — or none —
    // was loaded. Same argument that keeps the employee roster out of `config/`.
    RegVec3 imprintPose;                 // device axes: tool touching the coupon
    bool    hasImprintPose = false;
    RegVec3 homePose;                    // device axes: clear of the coupon
    bool    hasHomePose = false;
    RegVec3 tipView;                     // reference axes: camera over the mark
    bool    hasTipView = false;

    // ---- The measurement behind t --------------------------------------------
    // Kept for the reason NeedleCal keeps its three poses: when a tip lands
    // somewhere surprising, "which number is wrong" is the only useful question,
    // and re-teaching to find out throws the evidence away.
    RegVec3 tipMeas;                // the measured tip itself, reference space
    // The measured width of the mark, and how many corner picks were averaged.
    // Kept for the reason `spanMm` is kept on a frame: the average of N corner
    // picks is the centre only if they are placed symmetrically, and a pick that
    // lands off the mark shifts the centre by half its error while looking
    // exactly like a good one. A width that does not match the tool you know you
    // have is the only thing that shows it.
    double  tipSpreadMm = 0.0;
    int     tipPoints   = 0;

    // ---- The daily check ----------------------------------------------------
    double      checkResidualMm = 0.0;
    RegVec3     checkPose;          // where the check was taken (device axes)
    std::string checkedAt;

    std::string regAt;   // when the rotation was adopted
    std::string tipAt;   // when the tip was last measured
};

// ---- The maths -------------------------------------------------------------

// p_ref = R * h + t
RegVec3 DeviceRegPredict(const DeviceReg& r, const RegVec3& h);

// h = R^T * (p_ref - t). Exact for any orthogonal R, proper or not — and it is
// often NOT proper: every matrix in the shipped transforms file has det = -1,
// because controller axis-sign conventions are not consistently handed. Forcing
// a proper rotation here would invert one axis of every answer.
RegVec3 DeviceRegSolve(const DeviceReg& r, const RegVec3& p);

// Adopt a rotation from three columns (column j = where the reference device's
// axes see the registered device move for +1 mm on its axis j). Orthonormalises
// them, preserving handedness, and stamps `regAt` and `rotSource`. Does NOT make
// the record valid: a rotation without a tip predicts nothing.
std::string DeviceRegSetRotation(DeviceReg& r, const RegVec3 cols[3],
                                 const std::string& source);

// Teach t from one measured tip at a known pose, keeping R. Refuses without a
// rotation — there would be nothing to hold fixed. Stamps `tipAt`, records
// `tipMeas`, makes the record valid, and clears `verified`: the tip moved, so
// whatever the last check measured described tooling that may no longer be on
// the machine.
//
// The imprint's CENTRE and its measured width are computed by the page that owns
// the corner picks, not here — that is where the N points exist, and a second
// copy of the formula would be a second opinion about where the tool touches.
// `spreadMm` is twice the mean distance from the centre: the plain separation
// for two opposite picks, a diameter for four, so one number means the same
// thing however many were taken.
std::string DeviceRegSetTip(DeviceReg& r, const RegVec3& h, const RegVec3& p,
                            double spreadMm, int points);

// Whether a live pose is close enough to the taught rotations to predict
// through. `why` receives the reason when it is not.
bool DeviceRegRotationOk(const DeviceReg& r, const RegVec3& liveRot, std::string* why);

// ---- Persistence -----------------------------------------------------------
// One file for the whole fleet, keyed by device name: the runtime-writable pair
// config/device_reg.local.json -> config/device_reg.json, same rule as every
// other file of its kind.
nlohmann::json DeviceRegToJson(const DeviceReg& r);
DeviceReg      DeviceRegFromJson(const nlohmann::json& j);

std::map<std::string, DeviceReg> LoadDeviceRegs(std::string* pathUsed = nullptr);
std::map<std::string, DeviceReg> LoadDeviceRegsFile(const std::string& path);
bool SaveDeviceRegsFile(const std::string& path,
                        const std::map<std::string, DeviceReg>& regs);

// The one place the writable path is spelled, so a command, the UI and the
// loader cannot disagree about which file a registration lands in.
const char* DeviceRegLocalPath();

} // namespace firedog
