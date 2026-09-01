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
#include "core/SavedPositions.h"   // SavedPos (TargetsOf below)

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

// A saved position's values, restricted to the axes this device actually has.
// Shared for the reason everything else here is: a second copy would be a
// second answer to which axes a position commands, and a position that shares
// no axis with the device has to fail the same way from every caller. Three
// families call it now - motion, align and registration - and the align one
// carried its own identical copy (`StationTargets`) until this replaced it.
std::vector<std::pair<std::string, double>> TargetsOf(const SavedPos& p,
                                                      const IMotionController* m);

// ---- the two-speed move ----------------------------------------------------
// Drive to `targets` and wait, optionally covering the last stretch slowly:
// travel at the step's own velocity to a point `approach_mm` (or `approach_pct`
// of the distance) short of the target, then finish at approach_velocity_mm_s.
// A stage that arrives at full speed overshoots and rings, and the settle wait
// pays for it; slowing only the last stretch costs almost no cycle time.
//
// THE ONE IMPLEMENTATION. Every point-to-point move command goes through here —
// a second copy would drift on the ordering below, which is load-bearing:
//   * the travel leg arrives on a LOOSER tolerance (max(tol_mm, 0.05)): its
//     exact spot does not matter, only that we get there and slow down;
//   * WaitSettle runs BEFORE the velocity change, because PI rejects VLS while
//     a profile is still finishing (GCS 1005 CONTROLLER_BUSY);
//   * velocity_mm_s is re-applied afterwards, so the device is left at the
//     speed the step declared rather than crawling into the next one.
//
// With no approach configured this is exactly the plain
// MoveToPosition -> WaitArrive -> WaitSettle it replaced. Configured HALF way
// (a distance with no velocity, or the reverse) is a failure, not a no-op: a
// distance with no slow velocity is a full-speed arrival wearing the label of a
// careful one.
//
// The caller sets the TRAVEL velocity itself (ApplyPreMove, or an inline
// SetJogVelocity) before calling — this reads velocity_mm_s only to restore it.
// `what` names the destination in the error messages ("'safe'", "the placement
// correction").
CommandResult MoveWithApproach(ExecutionContext& ctx, IMotionController* m, int idx,
                               const std::vector<std::pair<std::string, double>>& targets,
                               const ParamValues& p, const std::string& what);

// The three parameters MoveWithApproach reads, as a shared schema fragment —
// two TUs register them, so they are here rather than file-local like
// CommandsMotion's VelocityParam/ExposureParam. All three carry the "Approach"
// group, which folds them away in the editor until one is set.
ParamSpec ApproachVelocityParam(bool required = false);
ParamSpec ApproachMmParam();
ParamSpec ApproachPctParam();

} // namespace firedog
