// MotionWaits.h : the arrival/settle contract shared by every command that
// moves an axis. Defined in CommandsMotion.cpp — declared here only because
// CommandsVision.cpp's auto-focus sweep must wait exactly the same way.
//
// Do not reimplement these in a new command file. The two-signal settle rule
// (profile done AND on-target) is what makes a scan sample trustworthy; a
// second, slightly different copy is how a scan starts measuring a ringing
// stage. See docs/pi-hexapod-motion-design.md.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "command/CommandSpec.h"
#include "command/ExecutionContext.h"

namespace firedog {

class IMotionController;

// Default arrival tolerance, same rule as Workflow::TickPath.
inline constexpr double kMoveTolMM = 0.01;

// Poll the driver's cached positions until every targeted axis is within tolMM.
// Keeps the fast-poll window open so the check runs on fresh positions.
ExecutionContext::WaitResult WaitArrive(
    ExecutionContext& ctx, IMotionController* m,
    const std::vector<std::pair<std::string, double>>& targets, double tolMM,
    int timeoutMs);

// Wait for the controller to finish its motion profile AND report on-target.
// Stops the device and fails on timeout, so an abort leaves a defined state.
CommandResult WaitSettle(ExecutionContext& ctx, IMotionController* m, int idx);

// Resolve a device name to a CONNECTED controller; null + *err otherwise.
IMotionController* ResolveMotion(ExecutionContext& ctx, const std::string& device,
                                 int* idxOut, std::string* err);

} // namespace firedog
