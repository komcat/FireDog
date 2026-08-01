// RigProbe.h : the read-back report.
//
// Strictly read-only. Every call below is a getter or a cached snapshot — the
// rig never issues a setpoint, an output switch, a move or a slide command, so
// pointing it at a live machine cannot move anything.
#pragma once

namespace firedog::rig {

class RigSession;

// One line per device: name -> ConnState, straight from HardwareManager::Snapshot.
void PrintStatus(const RigSession& s);

// Identity + live values for everything the session brought up: axis positions,
// IO pin tables with their config names, instrument readings, camera stats.
void PrintProbe(RigSession& s);

} // namespace firedog::rig
