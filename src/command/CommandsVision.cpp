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
#include "core/Log.h"
#include "core/Product.h"   // NowStamp (reliability timestamp)
#include "hardware/HardwareManager.h"
#include "vision/HalconEngine.h"
#include "workflow/Workflow.h"

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

void RegisterVisionCommands(CommandRegistry& reg, Workflow& /*wf*/, HardwareManager& /*hw*/)
{
    using PT = ParamType;

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
