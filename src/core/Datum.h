// Datum.h : a part's local frame taught by ONE recipe step (datum_reference)
// from two or three detected points - the alignment frame packaged as a step.
//
// Plain data. The store is Workflow's (SetDatum / DatumFor / ClearDatums), for
// the live frame's reason: a program runs each queue item as its own engine
// Start and ctx is wiped between them, so a frame in ctx could never reach the
// recipe after the one that measured it.
//
// The frame itself is an AlignResult, transformed by AlignLocalToMachine, so
// move_local drives a datum exactly as it drives an alignment.
//
// Lifetime (memory only): kDatumLifeMin after the solve, datum_clear, DUT
// close, product load, app close - and, for a datum solved with NO DUT open,
// the next program start (on a station without DUTs nothing else would say the
// part changed). docs/datum-reference-design.md §4.
#pragma once

#include "core/AlignFrame.h"

#include <chrono>
#include <string>

namespace firedog {

inline constexpr int kDatumLifeMin = 30;

struct Datum {
    std::string name;          // the step's `name`, e.g. "datum1"
    // The motion device whose AXIS SPACE the points and the frame are in: the
    // camera's device. move_local defaults to it.
    std::string device;
    int         points = 2;    // 2 or 3
    // origin / theta / span / z, and E1 / E2 as MEASURED. For a 3-point datum
    // the origin is the FITTED E1 (where local 0,0 lands), not the raw mark.
    AlignResult frame;
    AlignPoint  e3;            // measured, when points == 3
    // 3-point only: how far the fitted frame puts each point's reference from
    // where it was measured (mm), and the worst of the three.
    double      resid[3] = {0.0, 0.0, 0.0};
    double      maxResid = 0.0;
    // A DUT was open at the solve. A datum without one dies at the next
    // program start (Workflow::ClearDatumsWithoutDut).
    bool        withDut = false;
    std::string at;            // wall clock, for the log and the overlay
    std::chrono::steady_clock::time_point madeAt{};
};

// Every run's RAW measurement under a datum's name, published BEFORE the solve
// or the guards can refuse it - what the helper's "Use as reference" adopts.
// Never a licence to move: no consumer reads it.
struct DatumMeasure {
    std::string name;
    int         points = 2;
    double      spanMm = 0.0;            // |E1 -> E2| as measured
    bool        hasE3 = false;
    double      e3lx = 0.0, e3ly = 0.0;  // E3 in the E1->E2 two-point frame
    std::string at;
};

inline double DatumMinutesLeft(const Datum& d,
                               std::chrono::steady_clock::time_point now =
                                   std::chrono::steady_clock::now())
{
    const double age =
        std::chrono::duration<double, std::ratio<60>>(now - d.madeAt).count();
    return static_cast<double>(kDatumLifeMin) - age;
}

inline bool DatumExpired(const Datum& d,
                         std::chrono::steady_clock::time_point now =
                             std::chrono::steady_clock::now())
{
    return DatumMinutesLeft(d, now) <= 0.0;
}

} // namespace firedog
