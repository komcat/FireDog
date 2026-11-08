// ObjectResult.h : a detected object's LIVE result - where one physical part
// was found, by name, readable by any recipe of the program that found it.
//
// Plain data. The store that holds these is Workflow's (SetObjectResult /
// ObjectResultFor / ClearObjectResults), for the reason the live taught frame
// lives there: a program runs each queue item as its own engine Start, and ctx
// is wiped between them, so a result in ctx cannot reach the recipe after the
// one that measured it. See docs/vision-detection-design.md §7a.
//
// A result describes ONE PART, exactly like a taught frame, so it has the
// frame's lifetime: cleared at program start, DUT open and product load. A
// consumer that moved against the previous part's result would look exactly
// like one that arrived.
#pragma once

#include <string>

namespace firedog {

struct ObjectResult {
    std::string name;          // e.g. "det_sled" - the producer step's ctx_prefix
    // The motion device whose AXIS SPACE x / y are in: the camera's device. A
    // coordinate on one device is a different place on every other, so it is
    // part of the record and a consumer refuses any other device.
    std::string device;
    // Only what the producer was asked for: a step ticked for X alone stores
    // no Y, and a move then drives X alone.
    bool   hasX = false, hasY = false, hasP = false;
    double x = 0.0, y = 0.0;   // device axis mm
    double p = 0.0;            // deg, clockwise positive as the camera picture shows it
    // The operator put the crosshair on it after the detector missed
    // (detect_object's manual_fallback). P is then 0 - a crosshair has no
    // rotation - and the log and the editor say "manual".
    bool   manual = false;

    // Evidence, for the log and the Recipe Editor's live list.
    std::string        model;        // detection model name
    double             score = 0.0;
    unsigned long long frameSeq = 0;
    std::string        at;           // when it was measured
};

} // namespace firedog
