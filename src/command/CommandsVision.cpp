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

} // namespace

void RegisterVisionCommands(CommandRegistry& reg, Workflow& /*wf*/, HardwareManager& /*hw*/)
{
    using PT = ParamType;

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
}

} // namespace firedog
