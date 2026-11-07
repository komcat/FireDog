// RoiFilterView.h : roi_filter_view_setup's params <-> the CameraSetup it applies.
//
// The ONE mapping between the step and the record: the command builds its
// CameraSetup through it, and the ROI Filter view setup helper window previews
// a step and writes an edit back through it. Two hand-written mappings is how
// the helper's "Try" and the run start disagreeing about what a step does.
// docs/camera-setup-design.md.
#pragma once

#include "core/CameraSetup.h"

#include <nlohmann/json.hpp>

namespace firedog {

class ParamValues;

// From validated params (defaults merged). light "" = leave the light; then
// color and percent are kept in the step but not applied.
CameraSetup RoiFilterViewSetupFromParams(const ParamValues& p);

// Into a step's params, in place. A setup that leaves the light writes
// light = "" but still writes a color / percent it carries (else keeps the
// step's), so switching the light back on returns the call-out the operator
// had chosen.
void RoiFilterViewSetupToParams(const CameraSetup& s, nlohmann::json& params);

} // namespace firedog
