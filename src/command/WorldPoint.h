// WorldPoint.h : a camera pixel turned into a machine coordinate - the device
// the camera rides on, plus where in the frame the feature is.
//
// Defined in CommandsAlign.cpp, where it was written for align_station and
// pick_point. Exposed so detect_object (CommandsVision.cpp) measures with the
// SAME code: the pose is read AFTER the capture, because that is the pose the
// pixel offset is relative to, and a second copy that read it in a different
// order would be wrong in a way no test shows - only a part placed 20 um out.
#pragma once

#include "command/CommandSpec.h"
#include "command/ExecutionContext.h"
#include "core/AlignFrame.h"   // AlignPoint

namespace firedog {

class IMotionController;

// Fills `e`: gx/gy (and gz when the device has a Z) read live off `m`, the
// pick's pixel, and wx/wy = gx/gy + the pick's mm offset from the image
// centre. Waits ~120 ms for a fresh pose poll; fails on Stop or a device that
// reports no X/Y.
CommandResult CaptureWorldPoint(ExecutionContext& ctx, IMotionController* m,
                                const ExecutionContext::PickResult& pick,
                                AlignPoint* e);

} // namespace firedog
