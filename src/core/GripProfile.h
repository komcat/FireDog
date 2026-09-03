// GripProfile.h : where the object a gripper is HOLDING sits, relative to that
// gripper's own tip.
//
// The device registration (core/DeviceReg.h) answers "where is the tool tip",
// which is a fact about the machine and goes stale only when the tool changes.
// This answers the question one level out: a lens is picked from a tray and it
// lands in the jaws a little differently every time, so where the OBJECT is, is
// a fact about this pick and about nothing else.
//
//     object   = the centre measured through the camera        (reference axis mm)
//     tip      = DeviceRegTip(device)                          (reference axis mm)
//     offset   = object - tip                                  <- what a run returns
//
// Both terms are in the REFERENCE device's axis space — the gantry the camera
// rides on — for the reason NeedleCal and DeviceReg both are: CaptureWorldPoint
// reports `wx = gx + pick.dxMM` there, and gantry-main's transform inverts Z, so
// a number that took a detour through AxisTransform would come back describing a
// different direction. The subtraction cancels the origin, which is what makes
// the result transportable to the device holding the object.
//
// WHY A PROFILE AND NOT A MEASUREMENT. Every field here except the evidence at
// the bottom is a parameter of HOW to measure: where to hold the object, how to
// find it, and how far off it may sit. Those are properties of an
// object type — of "the 8 mm lens" — and they are worth naming and re-using. The
// measurement itself is per-pick and belongs in ctx, which is exactly where a
// run leaves it; `lastDx`/`lastDy` are kept only as EVIDENCE, the
// way DeviceReg keeps tipMeas, so "which number is wrong" stays answerable
// without re-teaching and throwing the answer away.
//
// It is PRODUCT state, not machine state: a lens is part of a product, its
// detection model is cut from a picture of that product's part, and both travel
// with the product folder. That is the opposite of DeviceReg's argument, and
// deliberately so — one describes the tool, the other describes what the tool
// picked up.
//
// TWO DATES, for the reason DeviceReg has two: `taughtAt` stamps the reference
// offset (this object, in these jaws) and `measuredAt` stamps the last run.
// Change the tool or the object and the reference is void while the parameters
// are not, so the page can say which half is old.
//
// What it deliberately does NOT hold:
//
//   * THE REFERENCE DEVICE. It is `DeviceRegFor(device).refDevice` — the
//     registration already names the space this offset lives in, and a second
//     copy is a second thing that can disagree.
//   * A dz. The offset is XY, in the plane the camera measures. A Z from a focus
//     sweep would describe the object's HEIGHT, which is a different question
//     asked by a different instrument, and folding it in here would let a
//     mis-focus read as a mis-grip.
//   * A saved pose in the product's positions. The inspect pose IS one of those
//     (hidden, named after this profile) — the opposite of DeviceReg's rule,
//     because this record lives in the same document the positions do.
//
// See docs/gripped-object-design.md.
#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

// How many marks a mode needs. Manual takes two — their midpoint is the object's
// reference point. Auto takes one: the detector reports the point.
inline constexpr int kGripManualMarks = 2;

struct GripProfile {
    std::string uid;
    std::string name = "object";

    // The gripper. It must carry a valid DeviceReg: without a tip there is
    // nothing to measure the object against, and the page refuses with the
    // registration's own reason rather than inventing one.
    std::string device;

    // A hidden private position on `device`, named grip_<uid>_i. Where to hold
    // the object so the camera can see it.
    //
    // It carries the ROTATIONS as well as XYZ (CapturePrivatePose stores every
    // axis the device reports), and that is what makes a reference offset
    // comparable between runs: the offset is measured in the camera's frame, so
    // a gripper turned 5 degrees since the teach would swing the object and read
    // as a mis-grip. Driving here first restores the pose the reference was
    // taught at. A profile with no inspect pose measures wherever it finds the
    // tool — legitimate, and the page says so.
    std::string inspectUid;

    // ---- how the object is found ---------------------------------------------
    // The mode is its own field and NOT inferred from `detector` being set, for
    // the reason a station's is: a taught patch costs real time at the
    // microscope, and choosing manual for one product must not destroy it.
    std::string mode = "manual";        // "manual" | "auto"
    std::string detector;               // vision model uid, auto mode
    // Defaults ON, like a nominal's and unlike a station's: measuring a grip is
    // a per-pick operation with an operator usually standing over it, and a
    // failed detection that asks is better than a run that stops. It is never
    // silent — the fallback logs a warning naming what failed.
    bool autoFallbackManual = true;

    // ---- outline -------------------------------------------------------------
    // The object's footprint as a rectangle: width along the reference device's
    // X and length along its Y at the inspect pose, in mm. DRAWN ONLY - on the
    // page while it is taught and on the Aux window as the gripper carries it -
    // and never judged: see the note on size just below. 0 x 0 draws nothing.
    //
    // The MEASURED point (two marks' midpoint, or the detection) is the ANCHOR.
    // It may sit anywhere on the rectangle - a lens found by its edge, a die
    // by one corner - and `anchorDx/Dy` says where, as point minus centre in
    // the rectangle's own frame. The outline is hung off that point wherever
    // the point is, which is how it follows the tool: the point is tip + offset,
    // and the tip moves with the device.
    double outWmm = 0.0, outLmm = 0.0;
    double anchorDx = 0.0, anchorDy = 0.0;

    // ---- what counts as right -------------------------------------------------
    // NO SIZE. The two manual marks have a separation and a circle model has a
    // radius, but neither is the object's real width: a mark lands on whatever
    // edge the operator can see and a detector on whatever contrast it was
    // taught on, so a "measured width" would be checked against a number that
    // describes the picture, not the part. The only thing this profile judges
    // is the offset.

    // The offset a GOOD grip has. Adopted from a measurement on the page, never
    // typed: nobody knows in advance where a gripper's jaws take hold. Until it
    // is adopted a run still reports its offset and simply has no verdict to
    // give — `ok` is false and the reason says why, rather than a pass by
    // default.
    bool   haveRef = false;
    double refDx = 0.0, refDy = 0.0;
    double tolMm = 0.05;

    // ---- evidence -------------------------------------------------------------
    double      lastDx = 0.0, lastDy = 0.0;
    std::string measuredAt;   // the last run
    std::string taughtAt;     // when the reference offset was adopted
};

// ---- lookup ----------------------------------------------------------------
// Uid first, then exact name — the convention every document record follows:
// recipes reference the uid (rename-safe), scripts and operators use the name.
const GripProfile* FindGripProfile(const std::vector<GripProfile>& list,
                                   const std::string& uidOrName);
GripProfile*       FindGripProfile(std::vector<GripProfile>& list,
                                   const std::string& uidOrName);

// ---- the outline -----------------------------------------------------------
// Whether there is a rectangle to draw. Both sides positive: a width with no
// length is a line, and a line is not the shape of anything in the jaws.
bool GripHasOutline(const GripProfile& g);

// ---- the verdict -----------------------------------------------------------
// Whether a measured offset is acceptable for this profile, with the reason
// when it is not. ONE implementation, because the page prints this verdict and
// grip_apply fails a run on it, and two of them would eventually disagree about
// the same pick.
bool GripVerdict(const GripProfile& g, double dx, double dy, std::string* why);

// ---- persistence -----------------------------------------------------------
nlohmann::json GripProfileToJson(const GripProfile& g);
GripProfile    GripProfileFromJson(const nlohmann::json& j);

// config/grips.local.json, falling back to config/grips.json — the no-product
// fallback, same rule as alignments.
std::vector<GripProfile> LoadGripProfiles();

// The explicit-path variants the product system uses. `uidOut` receives the
// file's own uid so the caller can check it against the manifest.
std::vector<GripProfile> LoadGripProfilesFile(const std::string& path,
                                              std::string* uidOut = nullptr);
bool SaveGripProfilesFile(const std::string& path,
                          const std::vector<GripProfile>& list,
                          const std::string& uid);

} // namespace firedog
