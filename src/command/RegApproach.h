// RegApproach.h : the two halves every "drive a registered device to a point"
// command shares, so a second family can move through a registration without
// restating its refusals.
//
// Defined in CommandsDeviceReg.cpp, which is where the argument for them lives.
// They are exposed for exactly one reason: grip_move_to_camera (CommandsGrip.cpp)
// is move_tool_to_camera with the OBJECT's offset subtracted, and a copy of the
// verified gate, the rotation guard, the travel ceiling and the two-speed
// approach would drift off the originals - the failure this family exists to
// not have.
#pragma once

#include "command/CommandSpec.h"
#include "core/DeviceReg.h"

#include <string>

namespace firedog {

class ExecutionContext;
class IMotionController;
class ParamValues;

// The refusals: resolve `device`, require its registration, check the space a
// caller claims its point is in (`expectedRef`, empty trusts the record), the
// verified gate (`requireVerified`, honouring `allow_unverified`) and the
// rotation guard against the LIVE pose. Fills the record, the controller and
// its index on success.
CommandResult PrepareRegMove(ExecutionContext& ctx, const ParamValues& p,
                             const std::string& device,
                             const std::string& expectedRef, DeviceReg* regOut,
                             IMotionController** mOut, int* idxOut,
                             bool requireVerified);

// The move: a point in the device's OWN axis space, through the `axes` /
// `max_mm` / `velocity_mm_s` / approach parameters and MoveWithApproach.
// `ceiling` is the caller's account of where the point came from, appended to
// the refusal when the travel exceeds max_mm.
CommandResult ApproachAxisPoint(ExecutionContext& ctx, const ParamValues& p,
                                const std::string& device, IMotionController* m,
                                int idx, const RegVec3& h, const char* ceiling,
                                const char* what);

// The solve in front of it: a point in the REFERENCE device's axis space,
// handed to a registered device so its tip lands there.
CommandResult ApproachRefPoint(ExecutionContext& ctx, const ParamValues& p,
                               const std::string& device, const DeviceReg& r,
                               IMotionController* m, int idx, const RegVec3& target);

} // namespace firedog
