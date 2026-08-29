// DeviceReg.h : where another device's tool tip is, in the camera device's space.
//
// The needle offset (core/NeedleCal.h) solves this problem INSIDE one device:
// the camera and the needle ride the same gantry, so the delta between them
// crosses no frame and three numbers describe it. A gripper on a hexapod is the
// same question across a device boundary, and it is a strictly bigger object:
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
// Three things about this are easy to get wrong, and each has a wrong version
// that looks right:
//
//   * R MUST BE MEASURED, NOT INHERITED. config/motion_transforms.json already
//     carries a matrix per device, but it is a hand-typed permutation of +-1:
//     somebody worked out which gantry direction each hexapod axis points along.
//     That is exact for a hexapod bolted square and approximate otherwise, and
//     the error is invisible — a mount 2 deg off predicts perfectly at the point
//     the offset was taught and misses by 1 mm 30 mm away. It degrades smoothly
//     with distance, so it reads as "the calibration is drifting" rather than as
//     "the rotation was never measured". The typed matrix is still useful as a
//     SEED to aim the measurement with; it is never the answer.
//
//   * THE AXIS TRIAD IS NOT ASSUMED RIGHT-HANDED. Every matrix in the shipped
//     transforms file has det = -1, because controller axis-sign conventions are
//     not consistently handed. So c3 = c1 x c2 only up to a sign, `handed` takes
//     that sign from the measurement, and R is orthogonal but may be improper.
//     R^T is still its inverse either way. Forcing a proper rotation here would
//     invert one axis of every answer.
//
//   * R AND t GO STALE FOR DIFFERENT REASONS. R is a property of the MOUNT; t is
//     a property of the mount AND the tool. Change a gripper and the tip moves
//     while the axis directions do not — so that costs ONE pick (DeviceRegSetTip)
//     and not the whole four-pick pass. `regAt` and `tipAt` are stamped
//     separately so the page can say which half is old.
//
// Like NeedleCal, and unlike a taught frame, THE PERSISTED RECORD IS THE LICENCE
// TO MOVE: a frame describes one physical part that may have been swapped since,
// this describes tooling that has not. It is machine-level, not per-product, and
// lives in config/device_reg.local.json with a config/device_reg.json seed.
//
// What it is NOT good enough for is the final placement. It is a four-point fit
// with one column measured through autofocus — good for arriving inside the
// placement loop's capture range, nowhere near the 20 um that loop converges to.
// placement_procedure.lua keeps its measure/correct/re-measure loop; this only
// means the loop lands on pass 1 instead of pass 2. See
// docs/device-registration-design.md.
#pragma once

#include <map>
#include <string>

#include <nlohmann/json.hpp>

namespace firedog {

// A point or a direction in one device's raw axis space. Local to this header:
// nothing else in the codebase carries a 3-vector, and the fit is the only place
// that multiplies one.
struct RegVec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

// ---- Refusal thresholds ----------------------------------------------------
// These are refusals, not warnings. A registration that is quietly wrong is
// worse than none at all, because everything downstream reads as working.
inline constexpr double kDeviceRegNormTol     = 0.005;   // |c| away from 1
inline constexpr double kDeviceRegOrthoTolDeg = 1.0;     // columns off 90 deg
inline constexpr double kDeviceRegCrossTolDeg = 2.0;     // measured c3 vs c1 x c2
inline constexpr double kDeviceRegMinJogMm    = 1.0;     // below this, pick noise wins
// How far the live rotations may sit from the ones the fit was taught at before
// a prediction refuses. The record is fitted at one rotation and says nothing
// about any other; see the U/V/W note in the design doc.
inline constexpr double kDeviceRegRotTolDeg   = 0.05;

struct DeviceReg {
    // False until a fit has actually been stored. Unlike NeedleCal there is no
    // typed seed here — the aiming rotation comes from motion_transforms.json,
    // which is already on disk, so there is nothing for an operator to guess.
    bool valid = false;
    // A check pass re-measured the tip AWAY from the pose the fit was taught at
    // and agreed. A fit that has only ever been evaluated at its own origin has
    // tested the measurement, not the rotation.
    bool verified = false;

    std::string device;                    // "hex-left" — the registered device
    std::string refDevice = "gantry-main"; // whose axis space p lives in

    // Columns are the registered device's axis directions expressed in the
    // reference device's axis space: R[i][j] is component i of column j.
    double  R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    RegVec3 t;                             // tip position at device pose (0,0,0)

    // ---- The measurement behind the numbers --------------------------------
    // Kept for the reason NeedleCal keeps its three poses: when a tip lands
    // somewhere surprising, "which pick is wrong" is the only useful question,
    // and re-teaching to find out throws the evidence away.
    double  jogMm = 0.0;      // L, the per-axis jog the columns were divided by
    RegVec3 h0;               // registered device pose at P0
    RegVec3 p0, p1, p2, p3;   // measured tip, ref space: rest, +X, +Y, +Z

    double colNorm[3]  = {0, 0, 0};   // |c_i| BEFORE normalising; want 1.000
    double orthoDeg[3] = {0, 0, 0};   // c1^c2, c2^c3, c3^c1 minus 90 deg
    double crossDeg    = 0.0;         // measured c3 against +-(c1 x c2)
    double handed      = 1.0;         // the sign the Z jog resolved

    // The rotations the fit was taught at. Not compensated for — refused past
    // kDeviceRegRotTolDeg, because a fit says nothing about a pose it never saw.
    RegVec3 taughtRot;
    bool    hasRot = false;   // false when the device reports no U/V/W at all

    // ---- The daily check ----------------------------------------------------
    double      checkResidualMm = 0.0;
    RegVec3     checkPose;          // where the check was taken (device axes)
    std::string checkedAt;

    std::string regAt;   // the full four-pick fit
    std::string tipAt;   // the last time t alone was re-measured
};

// ---- The maths -------------------------------------------------------------

// p_ref = R * h + t
RegVec3 DeviceRegPredict(const DeviceReg& r, const RegVec3& h);

// h = R^T * (p_ref - t). Exact for any orthogonal R, proper or not.
RegVec3 DeviceRegSolve(const DeviceReg& r, const RegVec3& p);

// Fit R and t from the four measured tip points and the pose P0 was taken at.
// Fills every quality field on `r` whether it accepts or refuses, so a refusal
// can say which number was out. Returns "" on success, else the reason.
//
// `jogMm` is the distance each axis was commanded, and it is what the columns
// are divided by — so a jog that did not complete shows up as a column norm
// below 1, which is one of the refusals.
std::string DeviceRegFit(DeviceReg& r, double jogMm, const RegVec3& h0,
                         const RegVec3& p0, const RegVec3& p1,
                         const RegVec3& p2, const RegVec3& p3);

// Re-teach t alone from one measured tip at a known pose, keeping R. This is the
// gripper-change path: the tool moved, the mount did not. Stamps `tipAt`.
// Refuses on an invalid record — without an R there is nothing to hold fixed.
std::string DeviceRegSetTip(DeviceReg& r, const RegVec3& h, const RegVec3& p);

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
