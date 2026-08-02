// HalconDetector.h : the feature detectors — circle/fiducial, gray NCC pattern
// matching and the square-corner finder, all backed by MVTec HALCON.
//
// This is the one place in FireDog where an image is actually ANALYSED.
// Everything else in src/vision either scores sharpness (FocusMetric, plain
// C++) or answers "is HALCON usable?" (HalconEngine). Ported from
// bobcat_engine's HalconCircleDetectionRobustPlugin and
// HalconPatternMatchingNCCPlugin, minus OpenCV — the frame arrives as the RGB8
// buffer the camera already publishes and is reduced with vision/FocusMetric's
// RgbToGray8.
//
// HALCON IS REQUIRED, AND ITS ABSENCE IS A MESSAGE, NEVER A CRASH. Every entry
// point asks HalconEngine::Instance().Available() first and returns false with
// that engine's reason string when the answer is no. That check is not
// defensive politeness: the SDK is /DELAYLOAD-ed, so the first call into a
// missing import raises a Win32 SEH exception (0xC06D007E) that a C++ catch
// does NOT handle — the LoadLibrary probe inside HalconEngine has to have
// passed before any HALCON symbol is touched.
//
// Nothing here is on the alignment critical path by accident: the manual
// crosshair pick and auto-focus stay plain C++, so an expired licence degrades
// automatic detection and never blocks an operator teaching a frame.
//
// PIMPL, like every driver in this project: no HALCON type reaches this header,
// so it is safe to include from the workflow and UI layers.
#pragma once

#include <memory>
#include <string>

#include "vision/VisionModel.h"
#include "vision/VisionResult.h"

namespace firedog {

class HalconDetector {
public:
    HalconDetector();
    ~HalconDetector();
    HalconDetector(const HalconDetector&) = delete;
    HalconDetector& operator=(const HalconDetector&) = delete;

    // True when a detector may run at all, with the reason when it may not
    // (HalconEngine's, verbatim — the UI shows it and the command fails with it,
    // so there is exactly one wording for a licence problem).
    static bool Available(std::string* why = nullptr);

    // Adopt `m`. For "ncc" this reads <modelDir>/<templateFile> and builds the
    // NCC model, which is the expensive part — keep the detector alive across
    // detects rather than reloading per frame.
    //
    // Parameter-only edits still need a reload: HALCON bakes the angle range
    // and the pyramid depth INTO the model, so a "cheap" partial update would
    // silently ignore half the form.
    bool Load(const VisionModel& m, const std::string& modelDir, std::string* err);

    // Teach: crop `patch` out of the frame, write it to `pgmPath`, and build the
    // NCC model from it using `m`'s ncc parameters. `tmplW`/`tmplH` report the
    // patch size actually taken (the crop is clamped to the frame), which the
    // caller writes back into the model along with the relative filename.
    // On success the detector is left LOADED with the freshly taught model, so
    // a teach can be tested immediately without a reload.
    bool Teach(const VisionModel& m, const unsigned char* rgb, int w, int h,
               const VisionRoi& patch, const std::string& pgmPath,
               int* tmplW, int* tmplH, std::string* err);

    // Run the loaded model over one RGB8 frame (w*h*3 bytes, row major).
    // ALWAYS fills `out`: a miss is `ok = false` plus a reason, not a failure —
    // "the feature is not there" is a legitimate answer that a reliability run
    // has to be able to count. Returns out->ok for call-site convenience.
    bool Detect(const unsigned char* rgb, int w, int h, VisionDetection* out);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace firedog
