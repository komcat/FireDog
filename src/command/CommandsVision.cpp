// CommandsVision.cpp : vision commands — auto-focus today, HALCON-backed
// inspection later.
//
// Runs on the RecipeEngine / Lua worker thread, never the UI thread. Every wait
// goes through ExecutionContext, so Stop interrupts the sweep within one poll
// interval and a timeout leaves the stage in a defined state.
//
// The focus measure is vision/FocusMetric (Laplacian variance, plain C++) —
// deliberately NOT HALCON, so auto-focus keeps working on a machine with no
// vision licence. HALCON availability is a separate question, answered by
// vision/HalconEngine and reported by `halcon_status`.
//
// Ported from bobcat_engine's VisionAutoFocusCommands (cv::Laplacian +
// meanStdDev), with three changes that matter on this machine:
//   * frame freshness is proven by the camera's frame counter, not a fixed
//     sleep — the same evidence-over-delay idea as the XCAN scan's publish gate
//     (docs/xcan-scan-design.md §4.3),
//   * the peak is refined by parabolic interpolation between its neighbours, so
//     the result is not quantised to z_step_mm,
//   * arrival/settle use the shared move waits, so a sample is never taken off
//     a ringing stage.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"
#include "command/RoiFilterView.h"
#include "command/WorldPoint.h"     // CaptureWorldPoint - align_station's own
#include "core/LightCallout.h"  // LightColorNames (roi_filter_view_setup)
#include "core/Log.h"
#include "core/ProcessMemory.h"   // the memory probe on every detect_object
#include "core/Product.h"   // NowStamp (reliability timestamp)
#include "hardware/HardwareManager.h"
#include "vision/HalconEngine.h"
#include "vision/VisionModel.h"     // DetectionImageRotation (detect_object P)
#include "hardware/motion/IMotionController.h"
#include "workflow/Workflow.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

// A runaway z_range/z_step combination is a typo, not a plan: 2000 moves would
// take an hour. Refuse it up front with a message that names both numbers.
constexpr int kMaxSweepPoints = 400;

struct FocusPoint {
    double z     = 0.0;
    double score = 0.0;
    double luma  = 0.0;
};

// Drive ONE axis to an absolute target and wait until the controller is both
// arrived and settled. Everything else in the sweep assumes this succeeded.
CommandResult MoveAxisTo(ExecutionContext& ctx, IMotionController* m, int idx,
                         const std::string& axis, double z, double tolMM, int timeoutMs)
{
    const std::vector<std::pair<std::string, double>> target = {{axis, z}};
    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), target)) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "move to %s=%.4f rejected", axis.c_str(), z);
        return CommandResult::Fail(buf);
    }
    const auto r = WaitArrive(ctx, m, target, tolMM, timeoutMs);
    if (r != ExecutionContext::WaitResult::Ok) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
        char buf[128];
        std::snprintf(buf, sizeof(buf), "timeout driving %s to %.4f", axis.c_str(), z);
        return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped ? "stopped" : buf);
    }
    return WaitSettle(ctx, m, idx);
}

// Score the scene at the current position, once the camera has produced frames
// that are certainly post-move.
//
// `freshFrames = 2` for the same reason the XCAN scan asks for 2 publishes:
// frame #1 may have been integrating while the stage was still travelling, so
// it is a mixed exposure; frame #2 is wholly from the new position. The wait is
// evidence-based and self-scaling — at 30 fps it costs ~66 ms, at 5 fps ~400 ms
// — where a fixed sleep is either wasteful or wrong.
bool SampleFocus(ExecutionContext& ctx, int roiPx, int freshFrames, int settleMs,
                 int frameTimeoutMs, bool& warnedStale, Workflow::FocusSample& out)
{
    if (settleMs > 0 && !ctx.Sleep(settleMs)) return false;   // stopped

    if (freshFrames > 0) {
        const unsigned long long seq0 = ctx.Wf().CameraFrameSeq();
        const auto r = ctx.WaitUntil(
            [&] {
                return ctx.Wf().CameraFrameSeq() >=
                       seq0 + static_cast<unsigned long long>(freshFrames);
            },
            frameTimeoutMs, 10);
        if (r == ExecutionContext::WaitResult::Stopped) return false;
        if (r == ExecutionContext::WaitResult::Timeout && !warnedStale) {
            warnedStale = true;   // once per sweep: a stalled feed says it once, loudly
            FD_WARN("vision", "auto_focus: camera produced no new frame within %d ms - "
                              "scoring the latest frame instead (feed stalled?)",
                    frameTimeoutMs);
        }
    }

    out = ctx.Wf().MeasureFocus(roiPx);
    return true;
}

// Sub-step peak from the three samples around the maximum. Equal spacing, so
// the vertex of the parabola through (-1,y0) (0,y1) (1,y2) sits at
//   d = 0.5 * (y0 - y2) / (y0 - 2*y1 + y2)
// which is only a peak when the denominator is negative (concave). Anything
// else — a flat or noisy triple — falls back to the sampled Z.
//
// The equal-spacing assumption is CHECKED, not assumed: a point whose frame
// never arrived is dropped from `pts`, which would leave a 2-step gap next to
// the peak and silently place the interpolated Z in the wrong place.
bool ParabolicPeak(const std::vector<FocusPoint>& pts, size_t i, double step, double& zOut)
{
    if (i == 0 || i + 1 >= pts.size()) return false;
    const double tol = step * 0.01;
    if (std::fabs((pts[i].z - pts[i - 1].z) - step) > tol) return false;   // gap below
    if (std::fabs((pts[i + 1].z - pts[i].z) - step) > tol) return false;   // gap above

    const double y0 = pts[i - 1].score, y1 = pts[i].score, y2 = pts[i + 1].score;
    const double den = y0 - 2.0 * y1 + y2;
    if (den >= -1e-12) return false;                 // not concave: no interior peak
    double d = 0.5 * (y0 - y2) / den;
    if (!(d > -1.0 && d < 1.0)) return false;        // NaN-safe bounds check
    zOut = pts[i].z + d * step;
    return true;
}

CommandResult ExecAutoFocus(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string axis   = p.GetString("axis");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    // The axis must exist on this device — "Z" on a device that calls it
    // something else would otherwise fail deep inside the first move.
    const std::vector<std::string> axes = m->Axes();
    bool haveAxis = false;
    for (const std::string& a : axes) if (a == axis) { haveAxis = true; break; }
    if (!haveAxis) return CommandResult::Fail("device '" + device + "' has no axis '" + axis + "'");

    const double rangeMM  = p.GetDouble("z_range_mm");
    const double stepMM   = p.GetDouble("z_step_mm");
    const std::string dir = p.GetString("direction");
    const int    roiPx    = p.GetInt("roi_px");
    const int    fresh    = p.GetInt("fresh_frames");
    const int    settleMs = p.GetInt("settle_ms");
    const int    frameTo  = p.GetInt("frame_timeout_ms");
    const int    moveTo   = p.GetInt("timeout_ms");
    const double tolMM    = p.GetDouble("tol_mm");
    const double minScore = p.GetDouble("min_score");
    const bool   interp   = p.GetBool("interpolate");
    const bool   trace    = p.GetBool("trace");
    const std::string prefix = p.GetString("ctx_prefix");

    // Swept interval. `both` straddles the start; `minus` / `plus` are one-sided
    // — the case where you already know which way focus lies (the Camera tab's
    // Autofocus button sweeps Z− from wherever the operator parked the gantry).
    // The walk itself is ALWAYS in increasing Z whatever the interval, so every
    // sample is approached from the same side and backlash cannot bias one end
    // of the curve against the other.
    const double spanMM = (dir == "both") ? 2.0 * rangeMM : rangeMM;

    // Snap the interval count to the nearest whole number when it is within a
    // hair of one, instead of flooring. A UI that stores the step as a float
    // sends 0.025f = 0.02500000037..., which makes 0.5/step = 19.9999997 — a
    // plain floor drops the last sample, and the last sample is the operator's
    // current position. The tolerance is RELATIVE because the error grows with
    // the interval count.
    const double raw     = spanMM / stepMM;
    const double snapped = std::round(raw);
    const int intervals  = static_cast<int>(
        (std::fabs(raw - snapped) <= 1e-6 * (snapped > 1.0 ? snapped : 1.0)) ? snapped
                                                                             : std::floor(raw));
    const int points = intervals + 1;
    if (points > kMaxSweepPoints) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "z_range_mm %.4f at z_step_mm %.4f would need %d moves (max %d) - "
                      "coarsen the step or narrow the range",
                      rangeMM, stepMM, points, kMaxSweepPoints);
        return CommandResult::Fail(buf);
    }

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    // Start position of the swept axis: the sweep is centred on it, and it is
    // where we go back to if the sweep finds nothing.
    const std::map<std::string, double> pos0 = m->Positions();
    const auto itZ = pos0.find(axis);
    if (itZ == pos0.end()) return CommandResult::Fail("no live position for axis '" + axis + "'");
    const double startZ = itZ->second;

    // Lowest sampled Z: the sweep always walks upward from here.
    const double zLow = (dir == "plus") ? startZ : startZ - rangeMM;

    ctx.CtxSet(prefix + ".start_z_mm", startZ);
    ctx.CtxSet(prefix + ".success", false);   // set true only on a clean finish

    FD_INFO("vision", "auto_focus: %s %s sweep %.4f .. %.4f mm (%s %.4f), step %.4f "
                      "(%d points), ROI %d px",
            device.c_str(), axis.c_str(), zLow, zLow + spanMM, dir.c_str(), rangeMM,
            stepMM, points, roiPx > 0 ? roiPx : 0);

    std::vector<FocusPoint> pts;
    pts.reserve(static_cast<size_t>(points));
    bool warnedStale = false;

    for (int i = 0; i < points; ++i) {
        const double z = zLow + i * stepMM;

        const CommandResult mv = MoveAxisTo(ctx, m, idx, axis, z, tolMM, moveTo);
        if (!mv.ok) {
            // A rejected/timed-out move mid-sweep is fatal: the remaining
            // samples would be at unknown positions.
            (void)MoveAxisTo(ctx, m, idx, axis, startZ, tolMM, moveTo);
            return mv;
        }

        Workflow::FocusSample s;
        if (!SampleFocus(ctx, roiPx, fresh, settleMs, frameTo, warnedStale, s))
            return CommandResult::Fail("stopped");
        if (!s.ok) {
            FD_WARN("vision", "auto_focus: no camera frame at %s=%.4f - point skipped",
                    axis.c_str(), z);
            continue;
        }

        pts.push_back({z, s.score, s.meanLuma});
        if (trace)
            FD_INFO("vision", "autofocus,%d,%.5f,%.4f,%.2f", i, z, s.score, s.meanLuma);
    }

    if (pts.empty()) {
        (void)MoveAxisTo(ctx, m, idx, axis, startZ, tolMM, moveTo);
        return CommandResult::Fail("auto_focus: no usable focus samples (camera grabbing?)");
    }

    size_t best = 0;
    for (size_t i = 1; i < pts.size(); ++i)
        if (pts[i].score > pts[best].score) best = i;

    if (pts[best].score < minScore) {
        (void)MoveAxisTo(ctx, m, idx, axis, startZ, tolMM, moveTo);
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "auto_focus: best score %.4f is below min_score %.4f - returned to start",
                      pts[best].score, minScore);
        return CommandResult::Fail(buf);
    }

    // A peak on the first or last sample means the true focus is outside the
    // swept range: the answer is a range problem, not a focus position, so say
    // so rather than quietly parking on the edge.
    const bool edge = (best == 0 || best + 1 == pts.size());
    if (edge)
        FD_WARN("vision", "auto_focus: peak sits at the edge of the sweep (%s=%.4f) - "
                          "widen z_range_mm, the real focus is probably outside it",
                axis.c_str(), pts[best].z);

    double bestZ = pts[best].z;
    bool   used  = false;
    if (interp && !edge) used = ParabolicPeak(pts, best, stepMM, bestZ);

    const CommandResult mv = MoveAxisTo(ctx, m, idx, axis, bestZ, tolMM, moveTo);
    if (!mv.ok) return mv;

    // Re-measure where we actually parked: the number a later step reads should
    // describe the stage's real state, not the sweep's best guess.
    Workflow::FocusSample finalS;
    if (!SampleFocus(ctx, roiPx, fresh, settleMs, frameTo, warnedStale, finalS))
        return CommandResult::Fail("stopped");

    ctx.CtxSet(prefix + ".best_z_mm", bestZ);
    ctx.CtxSet(prefix + ".best_score", pts[best].score);
    ctx.CtxSet(prefix + ".final_score", finalS.ok ? finalS.score : 0.0);
    ctx.CtxSet(prefix + ".mean_luma", pts[best].luma);
    ctx.CtxSet(prefix + ".delta_mm", bestZ - startZ);
    ctx.CtxSet(prefix + ".samples", static_cast<double>(pts.size()));
    ctx.CtxSet(prefix + ".interpolated", used);
    ctx.CtxSet(prefix + ".edge", edge);
    ctx.CtxSet(prefix + ".success", true);

    FD_INFO("vision", "auto_focus: best %s=%.5f mm (%s), score %.4f -> final %.4f, "
                      "%zu samples, d=%+.5f mm",
            axis.c_str(), bestZ, used ? "interpolated" : "sampled", pts[best].score,
            finalS.ok ? finalS.score : 0.0, pts.size(), bestZ - startZ);
    return CommandResult::Success();
}

// Report HALCON availability into ctx (and the log). Cheap, no hardware — its
// point is that a recipe can BRANCH on the licence instead of failing halfway:
//   halcon_status -> if ctx halcon.available is_false -> skip the vision block.
CommandResult ExecHalconStatus(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string prefix = p.GetString("ctx_prefix");
    const HalconStatus& s = p.GetBool("refresh") ? HalconEngine::Instance().Refresh()
                                                 : HalconEngine::Instance().Status();
    ctx.CtxSet(prefix + ".available", s.Available());
    ctx.CtxSet(prefix + ".compiled_in", s.compiledIn);
    ctx.CtxSet(prefix + ".runtime_loaded", s.runtimeLoaded);
    ctx.CtxSet(prefix + ".licensed", s.licensed);
    ctx.CtxSet(prefix + ".version", s.version);
    ctx.CtxSet(prefix + ".summary", s.Summary());

    FD_INFO("vision", "%s", s.Summary().c_str());
    if (p.GetBool("require") && !s.Available())
        return CommandResult::Fail(s.Summary());
    return CommandResult::Success();
}

// ---- Feature detection ------------------------------------------------------
// Unlike auto-focus, these DO require HALCON — see vision/HalconDetector.h for
// why that asymmetry is deliberate. The detector reports "not found" and "no
// licence" through the same field, so both are counted honestly by a
// reliability run instead of one of them looking like a crash.

// Wait until the camera has produced `freshFrames` frames since now, then run
// the model. Same evidence-over-delay gate as SampleFocus above, and the same
// degradation on a stalled feed: warn once, detect on the latest frame.
bool SampleDetect(ExecutionContext& ctx, const VisionModel& m, int freshFrames,
                  int frameTimeoutMs, bool& warnedStale, VisionDetection& out)
{
    if (freshFrames > 0) {
        const unsigned long long seq0 = ctx.Wf().CameraFrameSeq();
        const auto r = ctx.WaitUntil(
            [&] {
                return ctx.Wf().CameraFrameSeq() >=
                       seq0 + static_cast<unsigned long long>(freshFrames);
            },
            frameTimeoutMs, 10);
        if (r == ExecutionContext::WaitResult::Stopped) return false;
        if (r == ExecutionContext::WaitResult::Timeout && !warnedStale) {
            warnedStale = true;
            FD_WARN("vision", "vision_detect: camera produced no new frame within %d ms - "
                              "using the latest frame instead (feed stalled?)",
                    frameTimeoutMs);
        }
    }
    ctx.Wf().Detect(m, ctx.VisionDir(), &out);
    return true;
}

// Flatten a detection into ctx. One writer, so the wizard, a recipe step and an
// auto alignment station all describe a result with the same key names.
void WriteDetectCtx(ExecutionContext& ctx, const std::string& prefix,
                    const VisionDetection& d)
{
    ctx.CtxSet(prefix + ".ok", d.ok);
    ctx.CtxSet(prefix + ".count", static_cast<double>(d.count));
    ctx.CtxSet(prefix + ".x_px", d.xPx);
    ctx.CtxSet(prefix + ".y_px", d.yPx);
    ctx.CtxSet(prefix + ".dx_mm", d.dxMM);
    ctx.CtxSet(prefix + ".dy_mm", d.dyMM);
    ctx.CtxSet(prefix + ".angle_deg", d.angleDeg);
    ctx.CtxSet(prefix + ".score", d.score);
    ctx.CtxSet(prefix + ".radius_px", d.radiusPx);
    ctx.CtxSet(prefix + ".elapsed_ms", d.elapsedMs);
    ctx.CtxSet(prefix + ".error", d.error);
}

CommandResult ExecVisionDetect(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string name   = p.GetString("model");
    const std::string prefix = p.GetString("ctx_prefix");

    const VisionModel* m = ctx.FindVisionModel(name);
    if (!m) return CommandResult::Fail("detection model '" + name + "' not found");

    // The light the model was taught under, FIRST (then the exposure - the
    // order align_station uses). A model that carries one and cannot get it
    // FAILS: a detection under the wrong light is a different picture, not a
    // slightly worse one.
    if (p.GetBool("apply_lighting")) {
        std::string why;
        if (!ctx.Wf().ApplyLighting(m->lighting, &why))
            return CommandResult::Fail("'" + m->name + "': " + why);
    }
    const double expo = p.GetDouble("exposure_us");
    // The model's own exposure is the default; the step may override it. A
    // detector is tuned AT an exposure, so carrying it with the model is what
    // makes a taught model reproducible.
    const double useExpo = expo > 0.0 ? expo : m->exposureUs;
    if (useExpo > 0.0) ctx.Wf().SetCameraExposure(useExpo);

    VisionDetection d;
    bool warnedStale = false;
    if (!SampleDetect(ctx, *m, p.GetInt("fresh_frames"), p.GetInt("frame_timeout_ms"),
                      warnedStale, d))
        return CommandResult::Fail("stopped");

    WriteDetectCtx(ctx, prefix, d);

    if (d.ok) {
        FD_INFO("vision", "'%s': (%.2f, %.2f) px  dx %+.4f dy %+.4f mm  "
                          "angle %.3f deg  score %.3f  (%.1f ms)",
                m->name.c_str(), d.xPx, d.yPx, d.dxMM, d.dyMM, d.angleDeg,
                d.score, d.elapsedMs);
    } else {
        FD_WARN("vision", "'%s': %s", m->name.c_str(), d.error.c_str());
    }

    // `require` is what turns a miss into a failed step. Off by default so a
    // recipe can branch on `<prefix>.ok` instead — the halcon_status pattern.
    if (p.GetBool("require") && !d.ok)
        return CommandResult::Fail("'" + m->name + "': " + d.error);
    return CommandResult::Success();
}

// ---- detect_object -----------------------------------------------------------
//
// One comprehensive step, in three phases:
//
//   1 PRE-DETECTION  the light and the exposure the step's model carries, THEN
//     the move to the camera view taught into the step (`view_point`, the
//     helper's Teach here): with `z_lift` (default) Z goes to `safe_z_mm`
//     first, XY across, Z down to the taught height last; without it, one
//     direct XYZ move. No view_point = detect where the camera stands.
//   2 DETECTION  with the STEP'S OWN model. On a miss, `manual_fallback` asks the
//     operator to put the crosshair on the object (the alignment pick window)
//     and that point IS the result, P = 0, marked manual; otherwise `require`
//     decides between a failed step and <prefix>_ok = false.
//   3 POST-DETECTION  optionally (`center_after`) the camera XY onto the result,
//     Z unchanged, through MoveToObjectResult - move_to_result's own drive.
//
// X / Y are machine mm on the camera's device - its axis pose at the frame
// centre (the crosshair) plus the feature's pixel offset, through the
// CaptureWorldPoint align_station uses - and P the feature's rotation from its
// taught orientation as the camera picture shows it (degrees, clockwise
// positive). The result goes to Workflow's live object store under the
// prefix's name (any later recipe of the program can move to it) with ctx
// mirrors for this recipe; only the ticked axes are in either.

// The one parameter set the step's own moves run with: the arrival rule every
// mover uses, the step's velocity, no approach (the moves are camera travel,
// not a part closing on a part).
ParamValues DetectMoveParams(const ParamValues& p)
{
    return ParamValues(nlohmann::json{{"tol_mm", kMoveTolMM},
                                      {"timeout_ms", 60000},
                                      {"velocity_mm_s", p.GetDouble("velocity_mm_s")}});
}

// Phase 1's move. Three legs with the lift, so the camera never travels across
// the work at the height it focuses at; one coordinated leg without it.
CommandResult MoveCameraToView(ExecutionContext& ctx, IMotionController* cam, int idx,
                               double x, double y, double z, bool zLift, double safeZ,
                               const ParamValues& mv)
{
    const CommandResult pre = ApplyPreMove(ctx, idx, mv);
    if (!pre.ok) return pre;
    if (!zLift)
        return MoveWithApproach(ctx, cam, idx, {{"X", x}, {"Y", y}, {"Z", z}}, mv,
                                "the taught camera view");
    CommandResult r = MoveWithApproach(ctx, cam, idx, {{"Z", safeZ}}, mv, "the safe Z");
    if (!r.ok) return r;
    r = MoveWithApproach(ctx, cam, idx, {{"X", x}, {"Y", y}}, mv, "the view's X/Y");
    if (!r.ok) return r;
    return MoveWithApproach(ctx, cam, idx, {{"Z", z}}, mv, "the view's Z");
}

CommandResult ExecDetectObject(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string prefix = p.GetString("ctx_prefix");
    const bool takeX = p.GetBool("take_x"), takeY = p.GetBool("take_y"),
               takeP = p.GetBool("take_p");
    // Written FIRST, so a recipe branching on <prefix>_ok can read it on every
    // failing path below too.
    ctx.CtxSet(prefix + "_ok", false);
    ctx.CtxSet(prefix + "_manual", false);
    // And the live result under this name goes FIRST too: from here on it is
    // this run's measurement or nothing. A miss, a refusal or a Stop that left
    // the previous part's result standing would hand a later move_to_result a
    // place this part has never been.
    ctx.Wf().EraseObjectResult(prefix);

    if (!takeX && !takeY && !takeP)
        return CommandResult::Fail("detect_object reports nothing - tick X, Y and/or P "
                                   "in the step's Vision detect helper");
    const std::string uid = p.GetString("model");
    if (uid.empty())
        return CommandResult::Fail("detect_object has no detection model - create and "
                                   "teach it in the step's Vision detect helper");
    const VisionModel* m = ctx.FindVisionModel(uid);
    if (!m)
        return CommandResult::Fail("detect_object: its detection model (" + uid +
                                   ") is not in this product - re-create it in the helper");

    // P is a property of the METHOD, so a P nobody can measure is refused
    // before anything moves rather than reported as a 0 that looks real. (A
    // MANUAL fallback reports 0 by agreement - but only after the detector,
    // which could measure it, missed.)
    if (takeP) {
        std::string why;
        if (!DetectionImageRotation(*m, VisionDetection{}, nullptr, &why))
            return CommandResult::Fail("detect_object: P is ticked but " + why);
    }

    std::string err;
    int idx = -1;
    IMotionController* cam = ResolveMotion(ctx, p.GetString("camera_device"), &idx, &err);
    if (!cam) return CommandResult::Fail(err);
    const ParamValues mv = DetectMoveParams(p);

    // ---- 1 pre-detection: camera, light, then the view ------------------------
    // Light, then exposure, BEFORE the move, so the picture is ready on arrival
    // - the order align_station uses. A model whose light cannot be set FAILS:
    // a detection under the wrong light is a different picture.
    if (p.GetBool("apply_lighting")) {
        std::string why;
        if (!ctx.Wf().ApplyLighting(m->lighting, &why))
            return CommandResult::Fail("detect_object: " + why);
    }
    if (m->exposureUs > 0.0 && !ctx.Wf().SetCameraExposure(m->exposureUs))
        return CommandResult::Fail("detect_object: camera exposure change rejected "
                                   "(camera connected?)");
    if (p.Has("view_point")) {
        const nlohmann::json& vp = p.Raw().at("view_point");
        if (!vp.is_object() || !vp.contains("X") || !vp.contains("Y") || !vp.contains("Z"))
            return CommandResult::Fail("detect_object: view_point is not an X/Y/Z pose - "
                                       "Teach here again");
        const double vx = vp.at("X").get<double>(), vy = vp.at("Y").get<double>(),
                     vz = vp.at("Z").get<double>();
        const CommandResult mvr =
            MoveCameraToView(ctx, cam, idx, vx, vy, vz, p.GetBool("z_lift"),
                             p.GetDouble("safe_z_mm"), mv);
        if (!mvr.ok) return mvr;
        FD_INFO("vision", "detect_object '%s': at the taught view (%.4f, %.4f, %.4f)%s",
                prefix.c_str(), vx, vy, vz, p.GetBool("z_lift") ? " via the safe Z" : "");
        // The stage has arrived and settled (MoveWithApproach's two signals),
        // but what sits on it may still be ringing: a fixed pause before the
        // fresh-frame gate starts counting. Only after a move - a step that
        // detects where the camera stands has nothing to settle from.
        const int settle = p.GetInt("settle_ms");
        if (settle > 0 && !ctx.Sleep(settle)) return CommandResult::Fail("stopped");
    }

    // ---- 2 detection ------------------------------------------------------------
    VisionDetection d;
    bool warnedStale = false;
    if (!SampleDetect(ctx, *m, p.GetInt("fresh_frames"), p.GetInt("frame_timeout_ms"),
                      warnedStale, d))
        return CommandResult::Fail("stopped");

    ExecutionContext::PickResult pick;
    bool manual = false;
    if (d.ok) {
        pick.px   = d.xPx;
        pick.py   = d.yPx;
        pick.dxMM = d.dxMM;
        pick.dyMM = d.dyMM;
    } else if (p.GetBool("manual_fallback")) {
        // The alignment pick window, with the model and the reason it missed,
        // so the operator knows WHICH point the run wanted - never silently.
        FD_WARN("vision", "detect_object '%s': %s - asking the operator", prefix.c_str(),
                d.error.c_str());
        ExecutionContext::PickRequest req;
        req.alignName   = "detect_object";
        req.label       = "locate '" + prefix + "'";
        req.message     = "The detector found nothing (" + d.error + "). Put the "
                          "crosshair on the object, then OK - that point is the result.";
        req.autoError   = d.error;
        req.detectorUid = m->uid;
        if (!ctx.AwaitPick(req, &pick)) return CommandResult::Fail("cancelled by operator");
        manual = true;
    } else {
        ctx.CtxSet(prefix + "_score", d.score);
        ctx.CtxSet(prefix + "_error", d.error);
        FD_WARN("vision", "detect_object '%s': %s", prefix.c_str(), d.error.c_str());
        if (p.GetBool("require"))
            return CommandResult::Fail("detect_object: nothing found - " + d.error);
        return CommandResult::Success();
    }

    // The pose AFTER the frame (or the pick): CaptureWorldPoint's order.
    AlignPoint e;
    const CommandResult cap = CaptureWorldPoint(ctx, cam, pick, &e);
    if (!cap.ok) return cap;

    double pDeg = 0.0;   // a manual pick reports P = 0: a crosshair has no rotation
    if (takeP && !manual) {
        std::string why;
        if (!DetectionImageRotation(*m, d, &pDeg, &why))
            return CommandResult::Fail("detect_object: " + why);
    }

    // The result, into the live store: the one place a LOCATION crosses steps
    // (and recipes). Typed, with the device as a field.
    ObjectResult res;
    res.name     = prefix;
    res.device   = cam->Name();
    res.hasX     = takeX;
    res.hasY     = takeY;
    res.hasP     = takeP;
    res.x        = e.wx;
    res.y        = e.wy;
    res.p        = pDeg;
    res.manual   = manual;
    res.model    = m->name;
    res.score    = manual ? 0.0 : d.score;
    res.frameSeq = d.frameSeq;
    res.at       = NowStamp();
    ctx.Wf().SetObjectResult(res);

    // The same numbers in ctx, for THIS recipe - written in the same breath.
    ctx.CtxSet(prefix + "_score", res.score);
    ctx.CtxSet(prefix + "_x_px", pick.px);
    ctx.CtxSet(prefix + "_y_px", pick.py);
    ctx.CtxSet(prefix + "_error", manual ? d.error : std::string());
    if (takeX) ctx.CtxSet(prefix + "_x", e.wx);
    if (takeY) ctx.CtxSet(prefix + "_y", e.wy);
    if (takeP) ctx.CtxSet(prefix + "_p", pDeg);
    ctx.CtxSet(prefix + "_manual", manual);
    ctx.CtxSet(prefix + "_ok", true);

    char got[128] = "";
    int  n = 0;
    const auto room = [&] { return sizeof(got) - static_cast<std::size_t>(n); };
    if (takeX) n += std::snprintf(got + n, room(), " X %.4f", e.wx);
    if (takeY) n += std::snprintf(got + n, room(), " Y %.4f", e.wy);
    if (takeP) std::snprintf(got + n, room(), " P %+.3f deg", pDeg);
    FD_INFO("vision", "detect_object '%s'%s:%s  (%s at %.4f, %.4f + %.4f, %.4f mm; %s)",
            prefix.c_str(), manual ? " MANUAL" : "", got, cam->Name().c_str(), e.gx, e.gy,
            pick.dxMM, pick.dyMM, manual ? "operator pick" : "detected");
    // The memory probe, one line per detection: chasing growth across a loop
    // of detections needs a number per detection, not a Task Manager glance.
    // DEBUG - silent at the default level; lower the log level to see it.
    // (2026-11-07: 60 detections showed a cache plateau at ~1 GB private,
    // working set flat at ~430 MB - no leak.)
    FD_DEBUG("mem", "after detect_object '%s': %s", prefix.c_str(),
            ProcessMemorySummary().c_str());

    // ---- 3 post-detection: the camera onto the result --------------------------
    if (p.GetBool("center_after") && (takeX || takeY)) {
        const int pause = p.GetInt("pre_center_ms");
        if (pause > 0 && !ctx.Sleep(pause)) return CommandResult::Fail("stopped");
        const CommandResult c = MoveToObjectResult(ctx, cam, idx, cam->Name(), res, takeX,
                                                   takeY, 0.0, 0.0,
                                                   p.GetDouble("center_max_mm"), mv);
        if (!c.ok) return c;
    }
    return CommandResult::Success();
}

// Run a model N times and report the spread. This is the honest answer to "is
// this detector trustworthy?": a single successful detect proves nothing, and
// the number that matters is how far the reported point wanders when nothing
// has moved.
CommandResult ExecVisionTest(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string name   = p.GetString("model");
    const std::string prefix = p.GetString("ctx_prefix");
    const int runs     = p.GetInt("runs");
    const int interval = p.GetInt("interval_ms");
    const bool trace   = p.GetBool("trace");

    const VisionModel* m = ctx.FindVisionModel(name);
    if (!m) return CommandResult::Fail("detection model '" + name + "' not found");

    {   // light, then exposure - the order align_station uses
        std::string why;
        if (!ctx.Wf().ApplyLighting(m->lighting, &why))
            return CommandResult::Fail("'" + m->name + "': " + why);
    }
    if (m->exposureUs > 0.0) ctx.Wf().SetCameraExposure(m->exposureUs);

    std::vector<VisionDetection> hits;
    hits.reserve(static_cast<std::size_t>(runs));
    int attempted = 0;
    bool warnedStale = false;

    for (int i = 0; i < runs; ++i) {
        VisionDetection d;
        if (!SampleDetect(ctx, *m, p.GetInt("fresh_frames"),
                          p.GetInt("frame_timeout_ms"), warnedStale, d))
            return CommandResult::Fail("stopped");
        ++attempted;
        if (d.ok) hits.push_back(d);
        if (trace)
            FD_INFO("vision", "vtest,%d,%d,%.4f,%.4f,%.4f,%.2f",
                    i, d.ok ? 1 : 0, d.xPx, d.yPx, d.score, d.elapsedMs);
        if (i + 1 < runs && interval > 0 && !ctx.Sleep(interval))
            return CommandResult::Fail("stopped");
    }

    const std::size_t n = hits.size();
    double meanX = 0, meanY = 0, meanS = 0, sdX = 0, sdY = 0, maxDev = 0;
    if (n > 0) {
        for (const VisionDetection& d : hits) {
            meanX += d.xPx; meanY += d.yPx; meanS += d.score;
        }
        meanX /= static_cast<double>(n);
        meanY /= static_cast<double>(n);
        meanS /= static_cast<double>(n);
        for (const VisionDetection& d : hits) {
            const double ex = d.xPx - meanX, ey = d.yPx - meanY;
            sdX += ex * ex;
            sdY += ey * ey;
            const double dev = std::sqrt(ex * ex + ey * ey);
            if (dev > maxDev) maxDev = dev;
        }
        // Population sigma, not sample sigma: this is the spread of the runs we
        // actually did, not an estimate of some larger population.
        sdX = std::sqrt(sdX / static_cast<double>(n));
        sdY = std::sqrt(sdY / static_cast<double>(n));
    }

    const double okPct = attempted > 0
        ? 100.0 * static_cast<double>(n) / static_cast<double>(attempted) : 0.0;

    ctx.CtxSet(prefix + ".runs", static_cast<double>(attempted));
    ctx.CtxSet(prefix + ".ok", static_cast<double>(n));
    ctx.CtxSet(prefix + ".ok_pct", okPct);
    ctx.CtxSet(prefix + ".mean_score", meanS);
    ctx.CtxSet(prefix + ".mean_x_px", meanX);
    ctx.CtxSet(prefix + ".mean_y_px", meanY);
    ctx.CtxSet(prefix + ".std_x_px", sdX);
    ctx.CtxSet(prefix + ".std_y_px", sdY);
    ctx.CtxSet(prefix + ".max_dev_px", maxDev);

    // Hand the same numbers back to the UI thread through Workflow's queue, NOT
    // through the engine's Status ctx snapshot: RecipeEngine clears runCtx_ in
    // its terminal block BEFORE the state becomes Completed, so a UI waiting for
    // Completed and then reading Status.ctx always reads an empty map. This is
    // the taught-position / taught-frame route, and it is the only one that
    // survives the end of a run.
    VisionReliability rel;
    rel.runs      = attempted;
    rel.ok        = static_cast<int>(n);
    rel.meanScore = meanS;
    rel.meanXPx   = meanX;
    rel.meanYPx   = meanY;
    rel.stdXPx    = sdX;
    rel.stdYPx    = sdY;
    rel.maxDevPx  = maxDev;
    rel.testedAt  = NowStamp();
    ctx.Wf().QueueVisionReliability(m->uid, rel);

    FD_INFO("vision", "'%s' reliability: %zu/%d ok (%.0f%%)  score %.3f  "
                      "sigma %.3f/%.3f px  worst %.3f px",
            m->name.c_str(), n, attempted, okPct, meanS, sdX, sdY, maxDev);

    if (n == 0) return CommandResult::Fail("'" + m->name + "': not detected in any run");
    return CommandResult::Success();
}

// Grab the frame the camera is showing right now and park it for the Run
// page's image box.
//
// Deliberately NOT the same command as dut_capture_image, and deliberately with
// no DUT requirement. The use case is reading a serial off the part: drive the
// camera to the label, capture, and the operator types what they can now see
// into the DUT field. That happens BEFORE a DUT is open — it is how you learn
// which DUT to open — so a command that failed without one would be useless for
// exactly the job it exists to do. It records nothing; dut_capture_image is
// what records.
CommandResult ExecCaptureView(ExecutionContext& ctx, const ParamValues& p)
{
    Workflow::CapturedView v;
    bool isNew = false;   // a still, not a stream: the current frame will do
    if (!ctx.Hw().Camera().GetLatestFrame(v.rgb, v.w, v.h, isNew) || v.w <= 0 ||
        v.h <= 0)
        return CommandResult::Fail("no camera frame available (camera grabbing?)");

    v.label = p.GetString("label");
    v.stamp = NowStamp();
    FD_INFO("vision", "captured %dx%d view%s%s", v.w, v.h,
            v.label.empty() ? "" : " - ", v.label.c_str());
    ctx.Wf().SetCapturedView(std::move(v));
    return CommandResult::Success();
}

} // namespace

// ---- roi_filter_view_setup -------------------------------------------------
// The step <-> CameraSetup mapping (command/RoiFilterView.h). The command runs
// it, the helper window previews and writes back through it.

CameraSetup RoiFilterViewSetupFromParams(const ParamValues& p)
{
    CameraSetup s;
    s.exposureUs = p.GetDouble("exposure_us");
    const std::string light = p.GetString("light");
    if (!light.empty()) {
        s.lighting.kind    = "light";
        s.lighting.channel = light;
        s.lighting.color   = p.GetString("color");
        s.lighting.value   = p.GetDouble("percent");
    }
    s.filter.roiMode = p.GetInt("roi_mode");
    s.filter.binary  = p.GetBool("binary");
    s.filter.thr     = p.GetInt("threshold");
    s.filter.Sanitize();
    return s;
}

void RoiFilterViewSetupToParams(const CameraSetup& s, nlohmann::json& params)
{
    if (!params.is_object()) params = nlohmann::json::object();
    params["exposure_us"] = s.exposureUs > 0.0 ? s.exposureUs : 0.0;
    // light "" = leave the lighting. The call-out is still written when the
    // setup carries one, so switching the light back on returns what the
    // operator had chosen rather than a default.
    params["light"] = s.lighting.kind == "light" ? s.lighting.channel : std::string();
    if (!s.lighting.color.empty()) {
        params["color"]   = s.lighting.color;
        params["percent"] = std::clamp(s.lighting.value, 0.0, 100.0);
    }
    ViewFilter f = s.filter;
    f.Sanitize();
    params["roi_mode"]  = f.roiMode;
    params["binary"]    = f.binary;
    params["threshold"] = f.thr;
}

namespace {

// Set the camera up for what follows: exposure, the light, and the live view's
// ROI filter. Workflow::ApplyCameraSetup is the whole of it - never a second
// exposure or light path. It fails rather than continue under a setup the
// camera or the light refused.
CommandResult ExecRoiFilterViewSetup(ExecutionContext& ctx, const ParamValues& p)
{
    const CameraSetup s = RoiFilterViewSetupFromParams(p);
    std::string why;
    if (!ctx.Wf().ApplyCameraSetup(s, &why))
        return CommandResult::Fail("roi_filter_view_setup: " + why);
    return CommandResult::Success();
}

} // namespace

void RegisterVisionCommands(CommandRegistry& reg, Workflow& /*wf*/, HardwareManager& /*hw*/)
{
    using PT = ParamType;

    // ROI Filter view setup: how the camera view is looked at, as one step. All
    // of its params are edited in the helper window, against a live picture.
    reg.Register({"roi_filter_view_setup", "Vision",
                  "Set the camera view up: exposure, light, and the live ROI filter "
                  "(display only)",
                  {
                      {"exposure_us", PT::Double, false, 0.0, 0.0, 1.0e7, {},
                       "camera exposure (us); 0 = leave it as it is",
                       PinDir::Any, kRoiFilterViewHelper},
                      {"light", PT::LightName, false, "", 0, 0, {},
                       "light to set (a channel group or a channel's name); "
                       "empty = leave the lighting as it is",
                       PinDir::Any, kRoiFilterViewHelper},
                      {"color", PT::Enum, false, "white", 0, 0, LightColorNames(),
                       "the call-out on that light",
                       PinDir::Any, kRoiFilterViewHelper},
                      {"percent", PT::Double, false, 50.0, 0.0, 100.0, {},
                       "intensity (%); 0 = hold the light dark",
                       PinDir::Any, kRoiFilterViewHelper},
                      {"roi_mode", PT::IntChoice, false, 0, 0, 3, {"off", "1/3", "1/2", "2/3"},
                       "ROI square on the view's centre, this fraction of the frame's "
                       "short side; off = the whole frame",
                       PinDir::Any, kRoiFilterViewHelper},
                      {"binary", PT::Bool, false, false, 0, 0, {},
                       "draw the ROI as a binary threshold (display only - nothing "
                       "measures on it)",
                       PinDir::Any, kRoiFilterViewHelper},
                      {"threshold", PT::Int, false, 128, 0, 255, {},
                       "binary: grey at or above this is white",
                       PinDir::Any, kRoiFilterViewHelper},
                  },
                  CommandSpec::Needs::Camera, ExecRoiFilterViewSetup,
                  /*parallelBy*/ "", /*helper*/ kRoiFilterViewHelper});

    // Its undo for the DISPLAY half only: the live view back on the raw frame.
    // Exposure and light are left as they are - those are physical settings a
    // later step may rely on, and a step that put them back would have to know
    // what they were before. Needs nothing: clearing a picture aid touches no
    // device.
    reg.Register({"roi_filter_view_clear", "Vision",
                  "Clear the live view's ROI filter (back to the raw picture; exposure "
                  "and light unchanged)",
                  {},
                  CommandSpec::Needs::None,
                  [](ExecutionContext& ctx, const ParamValues&) {
                      ctx.Wf().SetLiveViewFilter(ViewFilter{});
                      FD_INFO("camera", "live view filter cleared");
                      return CommandResult::Success();
                  }});

    reg.Register({"capture_view", "Vision",
                  "Show the current camera frame in the Run page's image box",
                  {
                      {"label", PT::String, false, "", 0, 0, {},
                       "caption under the picture, e.g. 'serial number'"},
                  },
                  CommandSpec::Needs::Camera, ExecCaptureView});

    reg.Register({"auto_focus", "Vision",
                  "Sweep one axis, score sharpness per step, move to the sharpest Z",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device carrying the optic"},
                      {"axis", PT::String, false, "Z", 0, 0, {}, "local axis to sweep"},
                      {"z_range_mm", PT::Double, false, 0.2, 0.001, 50.0, {},
                       "sweep distance (see direction)"},
                      {"direction", PT::Enum, false, "both", 0, 0,
                       {"both", "minus", "plus"},
                       "both = +/- range around the start; minus = start-range..start; "
                       "plus = start..start+range"},
                      {"z_step_mm", PT::Double, false, 0.02, 0.0005, 5.0, {},
                       "sweep step (peak is refined below this by interpolation)"},
                      {"roi_px", PT::Int, false, 256, 0, 4096, {},
                       "centred square ROI scored; 0 = whole frame"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "new frames required after each move (0 = use the latest, blind)"},
                      {"settle_ms", PT::Int, false, 0, 0, 5000, {},
                       "precautionary delay before sampling (0 = rely on the frame gate)"},
                      {"frame_timeout_ms", PT::Int, false, 2000, 50, 30000, {},
                       "ceiling on the fresh-frame wait; then the latest frame is used"},
                      {"timeout_ms", PT::Int, false, 30000, 100, 600000, {}, "per-move arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      {"min_score", PT::Double, false, 0.0, 0, 0, {},
                       "fail (and return to start) if the best score is below this"},
                      {"interpolate", PT::Bool, false, true, 0, 0, {},
                       "refine the peak between samples (parabolic)"},
                      {"trace", PT::Bool, false, false, 0, 0, {},
                       "log one CSV line per sample: autofocus,i,z,score,luma"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional sweep velocity; 0 = keep current"},
                      {"ctx_prefix", PT::String, false, "autofocus", 0, 0, {},
                       "ctx key prefix for the results"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::Camera, ExecAutoFocus});

    reg.Register({"halcon_status", "Vision",
                  "Report HALCON build/runtime/licence state into ctx (optionally require it)",
                  {
                      {"refresh", PT::Bool, false, false, 0, 0, {},
                       "re-probe instead of using the cached result"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when HALCON is unavailable"},
                      {"ctx_prefix", PT::String, false, "halcon", 0, 0, {},
                       "ctx key prefix for the results"},
                  },
                  CommandSpec::Needs::None, ExecHalconStatus});

    reg.Register({"vision_detect", "Vision",
                  "Locate a taught feature with a detection model; results into ctx",
                  {
                      {"model", PT::VisionUid, true, {}, 0, 0, {},
                       "detection model (uid or name)"},
                      {"exposure_us", PT::Double, false, 0.0, 0.0, 1.0e7, {},
                       "override the model's exposure; 0 = use the model's"},
                      {"apply_lighting", PT::Bool, false, true, 0, 0, {},
                       "set the light the model was taught under first (no-op if it has none)"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "new frames required before detecting (0 = use the latest)"},
                      {"frame_timeout_ms", PT::Int, false, 2000, 50, 30000, {},
                       "ceiling on the fresh-frame wait; then the latest frame is used"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when nothing is detected (else branch on <prefix>.ok)"},
                      {"ctx_prefix", PT::String, false, "vision", 0, 0, {},
                       "results go to <prefix>.ok/.x_px/.y_px/.dx_mm/.dy_mm/.angle_deg/"
                       ".score/.radius_px/.count/.elapsed_ms/.error"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::Camera, ExecVisionDetect});

    // Find an object with the step's PRIVATE model and report its X / Y / P.
    // The model and the three ticks are the helper's: they mean something only
    // against the picture the model was taught on.
    CommandSpec detectSpec{"detect_object", "Vision",
                  "Find an object with this step's own detection model; report its "
                  "X / Y (machine mm) and/or P (deg) into ctx",
                  {
                      {"model", PT::String, false, "", 0, 0, {},
                       "this step's private detection model (uid) - created and "
                       "taught in the helper",
                       PinDir::Any, kVisionDetectHelper},
                      {"take_x", PT::Bool, false, true, 0, 0, {},
                       "report X: the camera device's X at the crosshair + the "
                       "feature's offset, mm",
                       PinDir::Any, kVisionDetectHelper},
                      {"take_y", PT::Bool, false, true, 0, 0, {},
                       "report Y, the same way",
                       PinDir::Any, kVisionDetectHelper},
                      {"take_p", PT::Bool, false, false, 0, 0, {},
                       "report P: the feature's rotation from its taught orientation, "
                       "deg, clockwise positive on the camera picture (ncc / corner "
                       "models with an angle tolerance)",
                       PinDir::Any, kVisionDetectHelper},
                      {"view_point", PT::PointXyz, false, {}, 0, 0, {},
                       "the camera pose to detect from, in the camera device's axis space - "
                       "taught in the helper (Teach here); empty = detect where it stands",
                       PinDir::Any, kVisionDetectHelper},
                      {"settle_ms", PT::Int, false, 100, 0, 10000, {},
                       "after arriving at view_point, wait this long before the frames "
                       "are taken (ms; 0 = none)",
                       PinDir::Any, kVisionDetectHelper},
                      {"pre_center_ms", PT::Int, false, 100, 0, 10000, {},
                       "after the detection, wait this long before the centring move "
                       "(ms; 0 = none; only with center_after)",
                       PinDir::Any, kVisionDetectHelper},
                      {"z_lift", PT::Bool, false, true, 0, 0, {},
                       "travel to view_point via safe_z_mm (Z first, XY, then Z down); "
                       "off = one direct XYZ move",
                       PinDir::Any, kVisionDetectHelper},
                      {"safe_z_mm", PT::Double, false, 25.0, 0, 0, {},
                       "the Z travelled at when z_lift is on - a camera-device AXIS value",
                       PinDir::Any, kVisionDetectHelper},
                      {"manual_fallback", PT::Bool, false, false, 0, 0, {},
                       "nothing found: ask the operator to put the crosshair on the object "
                       "(that point is the result, P = 0) instead of failing",
                       PinDir::Any, kVisionDetectHelper},
                      {"center_after", PT::Bool, false, false, 0, 0, {},
                       "after a result: move the camera X / Y onto it (Z unchanged)",
                       PinDir::Any, kVisionDetectHelper},
                      {"center_max_mm", PT::Double, false, 5.0, 0.0, 1000.0, {},
                       "refuse a centring move longer than this (0 = no ceiling)",
                       PinDir::Any, kVisionDetectHelper},
                      {"camera_device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the device the camera rides on: X / Y are ITS axis coordinates"},
                      VelocityParam(),
                      {"apply_lighting", PT::Bool, false, true, 0, 0, {},
                       "set the light the model was taught under first (no-op if it has none)"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "new frames required before detecting (0 = use the latest)"},
                      {"frame_timeout_ms", PT::Int, false, 2000, 50, 30000, {},
                       "ceiling on the fresh-frame wait; then the latest frame is used"},
                      {"require", PT::Bool, false, true, 0, 0, {},
                       "fail the step when nothing is found (off: branch on <prefix>_ok)"},
                      {"ctx_prefix", PT::String, false, "det", 0, 0, {},
                       "NAMES the object result (the live store move_to_result reads, "
                       "any recipe of the program) and prefixes the ctx keys: _ok, "
                       "the ticked _x / _y / _p, _score / _x_px / _y_px / _error"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::Camera, ExecDetectObject,
                  /*parallelBy*/ "", /*helper*/ kVisionDetectHelper};
    // What it writes, from the ticks and the prefix: ONE object result named
    // after the prefix, on the camera's device, carrying the ticked axes - and
    // the ctx mirrors for this recipe.
    detectSpec.outputs = [](const nlohmann::json& p) {
        const std::string pre = p.value("ctx_prefix", std::string("det"));
        const std::string dev = p.value("camera_device", std::string("gantry-main"));
        const bool tx = p.value("take_x", true), ty = p.value("take_y", true),
                   tp = p.value("take_p", false);
        CommandOutputs o;
        std::string axes;
        if (tx) axes += "X";
        if (ty) axes += "Y";
        if (tp) axes += "P";
        if (!axes.empty())
            o.results.push_back({pre, dev, axes, "the detected object, '" + dev + "' axis mm"});
        o.keys.push_back({pre + "_ok", "bool", "the object was found"});
        if (p.value("manual_fallback", false))
            o.keys.push_back({pre + "_manual", "bool",
                              "the operator located it (the detector missed); P = 0"});
        if (tx) o.keys.push_back({pre + "_x", "mm", "object X, '" + dev + "' axis mm"});
        if (ty) o.keys.push_back({pre + "_y", "mm", "object Y, '" + dev + "' axis mm"});
        if (tp) o.keys.push_back({pre + "_p", "deg",
                                  "object rotation from the taught orientation, deg, "
                                  "clockwise positive on the picture"});
        o.keys.push_back({pre + "_score", "number", "detection score 0..1"});
        o.keys.push_back({pre + "_x_px", "number", "object X in the picture, px"});
        o.keys.push_back({pre + "_y_px", "number", "object Y in the picture, px"});
        o.keys.push_back({pre + "_error", "text", "why nothing was found"});
        return o;
    };
    reg.Register(std::move(detectSpec));

    reg.Register({"vision_test", "Vision",
                  "Run a detection model N times and report the hit rate and spread",
                  {
                      {"model", PT::VisionUid, true, {}, 0, 0, {},
                       "detection model (uid or name)"},
                      {"runs", PT::Int, false, 20, 1, 500, {}, "how many detections to take"},
                      {"interval_ms", PT::Int, false, 100, 0, 10000, {},
                       "pause between runs"},
                      {"fresh_frames", PT::Int, false, 1, 0, 10, {},
                       "new frames required per run (1 = never score the same frame twice)"},
                      {"frame_timeout_ms", PT::Int, false, 2000, 50, 30000, {},
                       "ceiling on the fresh-frame wait"},
                      {"trace", PT::Bool, false, false, 0, 0, {},
                       "log one CSV line per run: vtest,i,ok,x_px,y_px,score,ms"},
                      {"ctx_prefix", PT::String, false, "vision_test", 0, 0, {},
                       "results go to <prefix>.runs/.ok/.ok_pct/.mean_score/.mean_x_px/"
                       ".mean_y_px/.std_x_px/.std_y_px/.max_dev_px"},
                  },
                  CommandSpec::Needs::Camera, ExecVisionTest});
}

} // namespace firedog
