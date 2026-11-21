// StepDetectors.h : which params of a recipe step hold its PRIVATE detection
// models (vdet_step_<uid>, kStepDetectorPrefix) - the one table every owner of
// that question reads: ProductSession::SyncStepDetectors (a copied step gets its
// own model, an unreferenced one is deleted), the Vision detect helper's owner
// count, the POR compare / fix list / deploy.
//
// A command whose step owns a model by any other param is a command whose
// models are deleted as orphans on the next save - so a new owner goes HERE,
// and nowhere else. docs/vision-detection-design.md §7a,
// docs/datum-reference-design.md §6.
#pragma once

#include <string>
#include <vector>

namespace firedog {

inline const std::vector<std::string>& StepDetectorParams(const std::string& commandType)
{
    static const std::vector<std::string> kNone;
    static const std::vector<std::string> kDetect = {"model"};
    static const std::vector<std::string> kDatum  = {"model1", "model2", "model3"};
    if (commandType == "detect_object")   return kDetect;
    if (commandType == "datum_reference") return kDatum;
    return kNone;
}

// True when `param` of a `commandType` step holds a private model.
inline bool IsStepDetectorParam(const std::string& commandType, const std::string& param)
{
    for (const std::string& p : StepDetectorParams(commandType))
        if (p == param) return true;
    return false;
}

} // namespace firedog
