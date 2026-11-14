// GrippedObject.h : an object REGISTERED to the gripper that holds it - where
// the thing in the jaws sits relative to that gripper's registered tip, in XYZ.
//
// Plain data. The store is Workflow's (SetGrippedObject / GrippedObjectFor /
// ClearGrippedObjects), for the live frame's reason: a program runs each queue
// item as its own engine Start and ctx is wiped between them.
//
// It is written by ONE command, grip_register_object: the camera has been
// brought onto the object (detect_object's center_after, or any move), and the
// object point - an object result's XY, or the camera centre, plus the camera's
// FOCUSED Z - is subtracted from the gripper's predicted tip. Both are in the
// camera device's axis space, so the origin cancels and what is kept is a
// vector. It is standalone: no GripProfile, no reference, no verdict.
//
// It has a BEACON's lifetime (core/Beacon.h), not an object result's: good for
// kGrippedObjectLifeMin minutes after registering, then gone - and gone too at
// grip_release_object, the Aux window's Clear, product load and app close.
// Program start and DUT open KEEP it, so a lens registered in one program can
// be placed by the next. An expired object reads as absent everywhere;
// ExpireGrippedObjects (App::Update) erases it and logs that once.
// grip_object_to_camera moves by
// it (tip target = camera - d); the Aux window draws it as a diamond at the
// LIVE tip plus (dx, dy), so it rides with the gripper.
// docs/gripped-object-design.md §8.
#pragma once

#include <chrono>
#include <string>

namespace firedog {

// The same 30 minutes a beacon gets, and for the same reason: long enough for a
// pick-and-place cycle, short enough that a forgotten record does not outlive
// the part.
inline constexpr int kGrippedObjectLifeMin = 30;

struct GrippedObject {
    std::string name;      // "lens" - the step's name for it
    std::string gripper;   // the registered device holding it ("hex-left")
    // The motion device whose AXIS SPACE every point below is in: the camera's
    // device, which is the gripper's DeviceReg::refDevice.
    std::string device;

    double ox = 0.0, oy = 0.0, oz = 0.0;   // the object, as measured
    double tx = 0.0, ty = 0.0, tz = 0.0;   // the gripper's predicted tip, then
    double dx = 0.0, dy = 0.0, dz = 0.0;   // object - tip            <- the answer
    double hx = 0.0, hy = 0.0, hz = 0.0;   // the same vector in the GRIPPER's axes
    // false: the Z half was not measured, and dz = hz = 0.
    bool hasZ = false;

    // The gripper's U when it was registered. A record only: the Aux window
    // says "U moved" when the live U is elsewhere, because a rotation about the
    // tip swings the object and nothing here compensates for it.
    double gripU = 0.0;
    bool   hasU  = false;

    std::string result;   // the object result it came from ("" = camera centre)
    std::string at;       // wall clock, for the log and the label
    std::chrono::steady_clock::time_point registeredAt{};
};

// Minutes this object has left (negative = expired), against `now`.
inline double GrippedObjectMinutesLeft(const GrippedObject& o,
                                       std::chrono::steady_clock::time_point now =
                                           std::chrono::steady_clock::now())
{
    const double age =
        std::chrono::duration<double, std::ratio<60>>(now - o.registeredAt).count();
    return static_cast<double>(kGrippedObjectLifeMin) - age;
}

inline bool GrippedObjectExpired(const GrippedObject& o,
                                 std::chrono::steady_clock::time_point now =
                                     std::chrono::steady_clock::now())
{
    return GrippedObjectMinutesLeft(o, now) <= 0.0;
}

} // namespace firedog
