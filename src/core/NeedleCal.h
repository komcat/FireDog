// NeedleCal.h : where the dispense needle is, relative to what the camera sees.
//
// The camera and the needle ride the same gantry, a hand's width apart — on the
// real machine about (-104, +13) mm. So "dispense on the feature I am looking
// at" is not a move to the feature: it is a move to the feature PLUS that lever
// arm, and the lever arm is a physical fact about the tooling that has to be
// measured. Nothing can dispense where the camera points until it has been.
//
// It is measured the only way a needle offset can be: put a dot on a coupon,
// look at the dot, and subtract. The gantry pose that made the imprint minus the
// world position of the imprint IS the vector, because both are read in the same
// axis space on the same device.
//
//     dx/dy : add to a CAMERA pose to get the pose that puts the NEEDLE there
//     dz    : add to a FOCUSED camera Z to get the Z at which the needle touches
//
// Three deliberate choices, each of which has a wrong version that looks right:
//
//   * MACHINE-LEVEL, not per-product. The needle is a fixture; it does not
//     change because a different part was loaded. It lives beside the hardware
//     config (config/needle_cal.local.json, falling back to needle_cal.json),
//     not in the product document, and it is re-taught when the needle is
//     changed — not when the product is.
//
//   * AXIS SPACE, never global. Camera and needle are on the SAME device, so
//     the delta between them never crosses a frame and must never be pushed
//     through AxisTransform. gantry-main's transform inverts Z (M33 = -1); a
//     number captured from IMotionController::Positions() and then issued as a
//     global delta would descend when it meant to retreat.
//
//   * dz is an OFFSET from a focused camera Z, not an absolute height. The
//     sibling project stored the touch height as a machine coordinate and then
//     needed a separate taught file for every single dispense point, because an
//     absolute Z only describes the one surface it was taught on. Paired with
//     auto-focus, one offset serves every point.
//
// The Z DIRECTION is not assumed anywhere. Which way "down" is on a given
// gantry is whatever the measurement said: sign(dz). A hardcoded convention is
// how a needle gets driven into a part.
#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace firedog {

struct NeedleCal {
    // False until a calibration has actually been run. A shipped seed is
    // deliberately invalid: it is close enough to aim the FIRST imprint into the
    // field of view, and nowhere near good enough to dispense through.
    bool valid = false;

    // The fleet this was measured on. Everything below is in this device's raw
    // axis coordinates.
    std::string device   = "gantry-main";
    std::string slide    = "Dispenser_Head";   // pneumatic slide, needle down/up
    std::string ioDevice = "IOBottom";
    std::string shotPin  = "Dispenser_Shot";   // the glue valve

    double dx = 0.0, dy = 0.0;   // camera pose -> needle pose (axis mm)
    double dz = 0.0;             // focused camera Z -> needle touch Z (axis mm)

    // The measurement behind the number. Kept for the same reason AlignResult
    // keeps its E points: a suspicious offset has to be auditable without
    // re-teaching it, and "which of the three poses is wrong" is the only
    // question worth asking when a dot lands somewhere surprising.
    double camX = 0.0, camY = 0.0, camZ = 0.0;   // camera, focused on the coupon
    double ndlX = 0.0, ndlY = 0.0, ndlZ = 0.0;   // pose when the needle imprinted
    double dotX = 0.0, dotY = 0.0;               // the imprint, world XY from the pick

    // How far the second pass had to move the answer. 0 with `verified` false
    // means nobody checked — which is not the same as zero error, and the UI
    // says so.
    double residualMm = 0.0;
    bool   verified   = false;

    std::string calibratedAt;   // "2026-08-09T14:02:11"
};

// Where the needle has to be driven to reach what the camera is centred on.
// Trivial, and here rather than at three call sites so the sign is written down
// exactly once.
inline void NeedleCameraToNeedle(const NeedleCal& c, double camX, double camY,
                                 double* ndlX, double* ndlY)
{
    if (ndlX) *ndlX = camX + c.dx;
    if (ndlY) *ndlY = camY + c.dy;
}

// +1 or -1: which way the axis has to move for the needle to go DOWN, taken
// from the measurement instead of from a convention. dz == 0 (never calibrated)
// answers +1, but every caller that would act on it is already refusing an
// invalid calibration.
inline double NeedleDownSign(const NeedleCal& c)
{
    return (c.dz >= 0.0) ? 1.0 : -1.0;
}

// ---- persistence ---------------------------------------------------------
// The runtime-writable pair: config/needle_cal.local.json is what a calibration
// writes and what loads first; config/needle_cal.json is the seed that ships.
nlohmann::json NeedleCalToJson(const NeedleCal& c);
NeedleCal      NeedleCalFromJson(const nlohmann::json& j);

// Tolerant: a missing or malformed file yields a default (invalid) record and
// logs, never throws. `pathUsed` receives the file actually read, so a log line
// can say whether the machine is running on its own calibration or on the seed.
NeedleCal LoadNeedleCal(std::string* pathUsed = nullptr);
NeedleCal LoadNeedleCalFile(const std::string& path);
bool      SaveNeedleCalFile(const std::string& path, const NeedleCal& c);

// The one place the writable path is spelled, so a command, the UI and the
// loader cannot disagree about which file a calibration lands in. Under the
// machine root (core/Paths.h).
std::string NeedleCalLocalPath();

} // namespace firedog
