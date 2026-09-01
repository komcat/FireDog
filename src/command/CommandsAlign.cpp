// CommandsAlign.cpp : the Alignment Procedure command family — teach a local
// coordinate system (O1) from two operator-marked features, then use it.
//
// align_station drives to a station, applies its exposure/lighting and gets the
// marked point; align_solve turns the two points into a frame and hands it back
// to the UI thread to persist; align_apply and move_local consume the frame.
// align_run is the typed one-step wrapper over alignment_procedure.lua, the
// same shape as `xcan` — the script IS the procedure, so it stays editable.
//
// Why a MANUAL pick answer arrives in millimetres: the operator's click happens
// in the UI, which is where the on-screen pixel mapping already lives, so it
// converts and the worker just adds the offset to the gantry pose. An AUTO
// station has no UI in the loop, so Workflow::Detect does the same conversion
// on the worker side with the same camera calibration — the two routes fill the
// same PickResult and everything downstream is identical.
// See docs/alignment-procedure-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"      // RunLuaFile (align_run)
#include "command/MotionWaits.h"    // WaitArrive / WaitSettle / ResolveMotion
#include "core/AlignFrame.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "vision/HalconDetector.h"   // Available() — the licence gate for auto mode
#include "workflow/Workflow.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

// Illumination for a station. No illumination device exists in the hardware
// config yet, so an empty `kind` (the default) is a no-op and the whole thing
// is a placeholder with a real call path already wired.
CommandResult ApplyLighting(ExecutionContext& ctx, const AlignLighting& l)
{
    if (l.kind.empty()) return CommandResult::Success();
    if (l.kind == "io") {
        if (!ctx.Wf().SetIoOutputByName(l.device, l.channel, l.value != 0.0))
            return CommandResult::Fail("lighting: IO output '" + l.device + "/" +
                                       l.channel + "' rejected");
        return CommandResult::Success();
    }
    if (l.kind == "psu") {
        const int ch = std::atoi(l.channel.c_str());
        if (ch <= 0) return CommandResult::Fail("lighting: bad PSU channel '" + l.channel + "'");
        if (!ctx.Wf().SetPsuVI(ch, l.value, 0.0) || !ctx.Wf().PsuOutputOn(ch))
            return CommandResult::Fail("lighting: PSU channel rejected");
        return CommandResult::Success();
    }
    return CommandResult::Fail("lighting: unknown kind '" + l.kind + "'");
}

// ctx key for one component of a captured point: "<prefix>_e1_wx" etc.
std::string PointKey(const std::string& prefix, int station, const char* field)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s_e%d_%s", prefix.c_str(), station, field);
    return buf;
}

double CtxNumber(ExecutionContext& ctx, const std::string& key, bool* found)
{
    const auto v = ctx.CtxGet(key);
    if (!v || !std::holds_alternative<double>(*v)) { if (found) *found = false; return 0.0; }
    if (found) *found = true;
    return std::get<double>(*v);
}

// The world point of an already-captured station, straight out of ctx. False
// when that station has not been captured in this run under this prefix.
bool CtxWorldPoint(ExecutionContext& ctx, const std::string& prefix, int station,
                   double* wx, double* wy)
{
    bool fx = false, fy = false;
    const double x = CtxNumber(ctx, PointKey(prefix, station, "wx"), &fx);
    const double y = CtxNumber(ctx, PointKey(prefix, station, "wy"), &fy);
    if (!fx || !fy) return false;
    if (wx) *wx = x;
    if (wy) *wy = y;
    return true;
}

// The guard band for THIS pick, when there is one to show: E2 of a procedure
// whose band is on, with E1 already captured. Everything else leaves it empty
// and the window draws nothing.
//
// It is filled HERE rather than in the UI because only the worker knows E1 — it
// was captured at the other station, one move ago, and the pick window sees
// nothing but the current field of view.
void FillPickBand(ExecutionContext& ctx, const AlignProc& proc, int station,
                  const std::string& prefix, ExecutionContext::PickBand* band)
{
    if (!band || station != 2 || !proc.guard.enabled) return;
    double e1x = 0.0, e1y = 0.0;
    if (!CtxWorldPoint(ctx, prefix, 1, &e1x, &e1y)) return;
    band->have     = true;
    band->cx       = e1x;
    band->cy       = e1y;
    band->spanRef  = proc.guard.spanMm;
    band->spanTol  = proc.guard.spanTolMm;
    band->thetaRef = proc.guard.thetaDeg;
    band->thetaTol = proc.guard.thetaTolDeg;
}

// Turn a captured pick into a world point: where the device is, plus where in
// the frame the feature was marked. Read the pose AFTER the capture — nothing
// moved, but this is the pose the pixel offset is relative to.
//
// Shared by align_station and place_mark. One copy on purpose: a second one
// that read the pose in a different order would be wrong in a way no test
// would show, only a part placed 20 um out.
CommandResult CaptureWorldPoint(ExecutionContext& ctx, IMotionController* m,
                                const ExecutionContext::PickResult& pick,
                                AlignPoint* e)
{
    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> live = m->Positions();
    const auto itx = live.find("X"), ity = live.find("Y");
    if (itx == live.end() || ity == live.end())
        return CommandResult::Fail("device '" + m->Name() + "' does not report X/Y");

    e->gx = itx->second;
    e->gy = ity->second;
    e->px = pick.px;
    e->py = pick.py;
    e->wx = e->gx + pick.dxMM;
    e->wy = e->gy + pick.dyMM;
    return CommandResult::Success();
}

// ---- align_station -------------------------------------------------------

CommandResult ExecAlignStation(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");
    const int station = p.GetInt("station");
    if (station != 1 && station != 2) return CommandResult::Fail("station must be 1 or 2");
    const AlignStation& st = (station == 1) ? proc->s1 : proc->s2;
    if (st.posUid.empty())
        return CommandResult::Fail("station has no saved position assigned");

    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, proc->device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const SavedPos* pos = ctx.FindPosition(proc->device, st.posUid);
    if (!pos) return CommandResult::Fail("station position uid not found");
    const auto targets = TargetsOf(*pos, m);
    if (targets.empty()) return CommandResult::Fail("station position shares no axes with device");

    // Resolve the detector BEFORE moving, so a mis-typed model name costs no
    // travel — but do not act on the failure yet: whether it is fatal depends on
    // auto_fallback_manual, and that decision belongs beside the capture.
    const VisionModel* model = nullptr;
    std::string autoErr;
    if (st.mode == "auto") {
        model = ctx.FindVisionModel(st.detector);
        if (!model) {
            autoErr = st.detector.empty()
                ? "station " + std::to_string(station) + " is set to auto but names "
                  "no detection model"
                : "station " + std::to_string(station) + " names no detection model '" +
                  st.detector + "'";
        } else if (std::string why; !HalconDetector::Available(&why)) {
            autoErr = "cannot run detection model '" + model->name + "': " + why;
        }
    }

    // Exposure BEFORE the move, so the sensor settles while the axes travel.
    // The station's own setting wins; a model carries the exposure it was tuned
    // at, which is what makes a taught detector reproducible on its own.
    const double expoUs = st.exposureUs > 0.0
        ? st.exposureUs
        : (model ? model->exposureUs : 0.0);
    if (expoUs > 0.0 && !ctx.Wf().SetCameraExposure(expoUs))
        return CommandResult::Fail("camera exposure change rejected (camera connected?)");
    const CommandResult light = ApplyLighting(ctx, st.lighting);
    if (!light.ok) return light;

    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("move to station " + std::to_string(station) + " rejected");
    const auto arrived = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                                    p.GetInt("timeout_ms"));
    if (arrived != ExecutionContext::WaitResult::Ok) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));   // defined state on any abort
        return CommandResult::Fail(arrived == ExecutionContext::WaitResult::Stopped
                                       ? "stopped"
                                       : "timeout driving to station " + std::to_string(station));
    }
    const CommandResult settled = WaitSettle(ctx, m, idx);
    if (!settled.ok) return settled;

    // Where the E point comes from. Both routes fill the SAME PickResult — the
    // pixel and the millimetre offset from the image centre — so everything
    // downstream is identical whether a detector or an operator produced it.
    ExecutionContext::PickResult pick;
    bool manual = (st.mode != "auto");
    // The world point, and whether it is already in hand. An auto capture has to
    // be turned into one EARLY, because the guard band is a statement about
    // where E2 sits relative to E1 and a pixel offset cannot answer it. Kept so
    // the happy path captures the pose exactly once.
    AlignPoint e;
    bool       haveE  = false;
    const std::string prefix = p.GetString("ctx_prefix");

    if (!manual && autoErr.empty()) {
        // Prove the frame is post-move before believing it: frame #1 may have
        // been integrating while the stage travelled, frame #2 is wholly from
        // the station. Same gate auto_focus uses.
        const int fresh = p.GetInt("fresh_frames");
        if (fresh > 0) {
            const unsigned long long seq0 = ctx.Wf().CameraFrameSeq();
            const auto fr = ctx.WaitUntil(
                [&] {
                    return ctx.Wf().CameraFrameSeq() >=
                           seq0 + static_cast<unsigned long long>(fresh);
                },
                2000, 10);
            if (fr == ExecutionContext::WaitResult::Stopped)
                return CommandResult::Fail("stopped");
            if (fr == ExecutionContext::WaitResult::Timeout)
                FD_WARN("align", "'%s' station %d: no new frame within 2000 ms - "
                                 "detecting on the latest frame (feed stalled?)",
                        proc->name.c_str(), station);
        }

        VisionDetection det;
        ctx.Wf().Detect(*model, ctx.VisionDir(), &det);
        if (det.ok) {
            pick.px   = det.xPx;
            pick.py   = det.yPx;
            pick.dxMM = det.dxMM;
            pick.dyMM = det.dyMM;
            FD_INFO("align", "'%s' E%d: '%s' found at (%.2f, %.2f) px, score %.3f",
                    proc->name.c_str(), station, model->name.c_str(),
                    det.xPx, det.yPx, det.score);

            // Is the point the detector found where E2 is ALLOWED to be? A
            // detector that locks onto the wrong instance of a repeated feature
            // succeeds with a high score and a frame wrong by the pitch between
            // them — the score says the match was good, not that it was the
            // right one. Out of band it is treated exactly like a detection
            // failure, so the existing rule decides what happens: ask the
            // operator if the station allows a fallback, fail if it does not.
            const CommandResult cap = CaptureWorldPoint(ctx, m, pick, &e);
            if (!cap.ok) return cap;
            haveE = true;
            double e1x = 0.0, e1y = 0.0;
            if (proc->guard.enabled && station == 2 &&
                CtxWorldPoint(ctx, prefix, 1, &e1x, &e1y)) {
                AlignPoint e1;
                e1.wx = e1x;
                e1.wy = e1y;
                double span = 0.0, theta = 0.0;
                AlignSpanTheta(e1, e, &span, &theta);
                if (!AlignGuardCheck(proc->guard, span, theta)) {
                    char buf[224];
                    std::snprintf(buf, sizeof(buf),
                                  "'%s' found a point OUT OF BAND: span %.4f mm "
                                  "(%.4f +/- %.4f), theta %.3f deg (%.3f +/- %.3f)",
                                  model->name.c_str(), span, proc->guard.spanMm,
                                  proc->guard.spanTolMm, theta, proc->guard.thetaDeg,
                                  proc->guard.thetaTolDeg);
                    autoErr = buf;
                    haveE   = false;   // the operator's mark replaces it
                }
            }
        } else {
            autoErr = "'" + model->name + "': " + det.error;
        }
    }

    // A failure anywhere in the auto path lands here. Falling back is opt-in
    // BECAUSE a silent fallback would let a procedure claim to be automatic
    // while a human placed every point.
    if (!manual && !autoErr.empty()) {
        if (!st.autoFallbackManual)
            return CommandResult::Fail(autoErr +
                " (set auto_fallback_manual or switch the station to manual)");
        FD_WARN("align", "'%s' station %d: %s - asking the operator",
                proc->name.c_str(), station, autoErr.c_str());
        manual = true;
    }

    if (manual) {
        ExecutionContext::PickRequest req;
        req.alignName = proc->name;
        req.station   = station;
        // Only set on the FALLBACK path (autoErr non-empty): a station that was
        // manual by design has no taught point to show, and showing one would
        // claim a precision the procedure never asked for.
        req.autoError   = autoErr;
        if (!autoErr.empty() && model) req.detectorUid = model->uid;
        req.message   = ctx.Substitute(p.GetString("message"));
        if (req.message.empty()) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Mark E%d on the live image, then OK", station);
            req.message = buf;
        }
        // Where E2 is allowed to land, drawn on the image the operator is
        // marking. This is the whole re-mark loop: the crosshair moves until it
        // is inside the band, and accepting one outside takes its own button.
        FillPickBand(ctx, *proc, station, prefix, &req.band);
        if (!ctx.AwaitPick(req, &pick))
            return CommandResult::Fail("cancelled by operator");
        haveE = false;   // the operator's mark, not whatever the detector said
    }

    // The world point: where the gantry is, plus where in the frame the feature
    // was marked (by hand or by the detector). Already in hand when an auto
    // capture had to be measured against the band.
    if (!haveE) {
        const CommandResult captured = CaptureWorldPoint(ctx, m, pick, &e);
        if (!captured.ok) return captured;
    }

    // Feedback to the live overlays, the moment the point exists. align_solve
    // only fires after BOTH stations, so without this the Aux window would have
    // nothing to show while station 2 is being marked — which is exactly when
    // an operator wants to see that E1 landed where they expected.
    ctx.Wf().QueueAlignPoint(proc->uid, station, e);

    ctx.CtxSet(PointKey(prefix, station, "gx"), e.gx);
    ctx.CtxSet(PointKey(prefix, station, "gy"), e.gy);
    ctx.CtxSet(PointKey(prefix, station, "px"), e.px);
    ctx.CtxSet(PointKey(prefix, station, "py"), e.py);
    ctx.CtxSet(PointKey(prefix, station, "wx"), e.wx);
    ctx.CtxSet(PointKey(prefix, station, "wy"), e.wy);

    FD_INFO("align", "'%s' E%d (%s): gantry (%.4f, %.4f) + mark (%+.4f, %+.4f) "
                     "= (%.4f, %.4f) mm",
            proc->name.c_str(), station, manual ? "manual" : "auto",
            e.gx, e.gy, pick.dxMM, pick.dyMM, e.wx, e.wy);
    return CommandResult::Success();
}

// ---- align_solve ---------------------------------------------------------

CommandResult ExecAlignSolve(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");

    const std::string prefix = p.GetString("ctx_prefix");
    AlignPoint e1, e2;
    bool ok = true, f = false;
    e1.gx = CtxNumber(ctx, PointKey(prefix, 1, "gx"), &f); ok = ok && f;
    e1.gy = CtxNumber(ctx, PointKey(prefix, 1, "gy"), &f); ok = ok && f;
    e1.px = CtxNumber(ctx, PointKey(prefix, 1, "px"), &f);
    e1.py = CtxNumber(ctx, PointKey(prefix, 1, "py"), &f);
    e1.wx = CtxNumber(ctx, PointKey(prefix, 1, "wx"), &f); ok = ok && f;
    e1.wy = CtxNumber(ctx, PointKey(prefix, 1, "wy"), &f); ok = ok && f;
    e2.gx = CtxNumber(ctx, PointKey(prefix, 2, "gx"), &f); ok = ok && f;
    e2.gy = CtxNumber(ctx, PointKey(prefix, 2, "gy"), &f); ok = ok && f;
    e2.px = CtxNumber(ctx, PointKey(prefix, 2, "px"), &f);
    e2.py = CtxNumber(ctx, PointKey(prefix, 2, "py"), &f);
    e2.wx = CtxNumber(ctx, PointKey(prefix, 2, "wx"), &f); ok = ok && f;
    e2.wy = CtxNumber(ctx, PointKey(prefix, 2, "wy"), &f); ok = ok && f;
    if (!ok)
        return CommandResult::Fail("no captured E1/E2 in ctx under prefix '" + prefix +
                                   "' - run align_station for both stations first");

    AlignResult r;
    std::string err;
    if (!SolveAlignFrame(e1, e2, &r, &err)) {
        ctx.CtxSet(prefix + "_valid", false);
        return CommandResult::Fail(err);
    }

    ctx.CtxSet(prefix + "_origin_x", r.originX);
    ctx.CtxSet(prefix + "_origin_y", r.originY);
    ctx.CtxSet(prefix + "_theta_deg", r.thetaDeg);
    ctx.CtxSet(prefix + "_span_mm", r.spanMm);
    ctx.CtxSet(prefix + "_valid", true);

    // The guard band's verdict, into ctx beside the frame. It is REPORTED here,
    // not enforced: by the time a solve runs, an out-of-band point has already
    // been shown to the operator on the image and consciously accepted, or an
    // auto station without a fallback has already failed. Failing the step now
    // would only throw away a frame somebody deliberately kept.
    //
    // The ctx keys are what let a recipe act on it anyway — `dut_record` writes
    // the verdict onto the part's record, and a branch can refuse to carry on.
    double spanErr = 0.0, thetaErr = 0.0;
    const bool inBand = AlignGuardCheck(proc->guard, r.spanMm, r.thetaDeg,
                                        &spanErr, &thetaErr);
    ctx.CtxSet(prefix + "_guard_ok", inBand);
    ctx.CtxSet(prefix + "_span_err_mm", spanErr);
    ctx.CtxSet(prefix + "_theta_err_deg", thetaErr);
    if (!inBand) {
        FD_WARN("align", "'%s' solved OUT OF BAND: span %.4f (%.4f +/- %.4f), "
                         "theta %.4f (%.4f +/- %.4f)",
                proc->name.c_str(), r.spanMm, proc->guard.spanMm, proc->guard.spanTolMm,
                r.thetaDeg, proc->guard.thetaDeg, proc->guard.thetaTolDeg);
    }

    // The LIVE frame — what align_apply / move_local will actually use. Set
    // unconditionally, even when `save` is off: `save` decides whether the
    // frame is RECORDED, not whether this session may move against the part it
    // just measured. This is also what makes the frame outlive the run that
    // taught it, so a program can align in one slot and work in the next.
    ctx.Wf().SetLiveFrame(proc->uid, r);

    if (p.GetBool("save")) {
        // The document is UI-thread state: hand the frame over and let
        // App::Update store and persist it. That copy is a RECORD only — no run
        // ever consumes it (see Workflow::SetLiveFrame).
        ctx.Wf().QueueAlignResult(proc->uid, r);
    }
    FD_INFO("align", "'%s' solved: origin (%.4f, %.4f) mm, theta %.4f deg, span %.4f mm",
            proc->name.c_str(), r.originX, r.originY, r.thetaDeg, r.spanMm);
    return CommandResult::Success();
}

// ---- align_apply ---------------------------------------------------------

// Resolve the procedure AND its LIVE frame — the one check every consumer of a
// frame owes the operator.
//
// The frame comes from Workflow, never from `proc->result`. The document copy
// is a record of the LAST teach, which may have been against a different part
// on a different day; moving against it would look exactly like a correct move
// and be wrong by however far the part has shifted. Only a frame taught in this
// session counts, and it is cleared whenever the part might have changed.
const AlignProc* LiveProc(ExecutionContext& ctx, const std::string& name,
                          AlignResult* frame, std::string* err)
{
    const AlignProc* proc = ctx.FindAlign(name);
    if (!proc) { *err = "alignment procedure not found: '" + name + "'"; return nullptr; }
    if (!ctx.Wf().LiveFrame(proc->uid, frame)) {
        *err = "alignment '" + proc->name + "' has no live frame - run it in THIS "
               "program first (a saved frame is a record of the last teach, not a "
               "licence to move)";
        return nullptr;
    }
    return proc;
}

CommandResult ExecAlignApply(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err);
    if (!proc) return CommandResult::Fail(err);

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(frame, p.GetDouble("x_mm"), p.GetDouble("y_mm"), &mx, &my);
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_x", mx);
    ctx.CtxSet(prefix + "_y", my);
    return CommandResult::Success();
}

// ---- nominal_point -------------------------------------------------------
//
// Where a nominal IS, in machine XY, with nothing marked and nothing moved.
//
// `align_apply` one function up already converts a local coordinate through the
// frame — but it takes two raw numbers, and the only thing that knew a
// NOMINAL's numbers was SolvePlacement, which needs the object's two marks
// first. That is precisely the measurement a registered device makes
// unnecessary for the FIRST approach: to send a gripper to n1, something has to
// be able to say where n1 is before anything has looked at the object.
//
// There is deliberately no `_z`. The frame is 2-D rigid and an AlignNominal
// stores lx/ly and nothing else (core/AlignFrame.h), so a Z reported here would
// be invented, and an invented Z on a gripper approach is a lens driven into a
// substrate. A caller that needs one takes it from a focused camera reading or
// from a stored placement offset — see docs/device-registration-design.md.
CommandResult ExecNominalPoint(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err);
    if (!proc) return CommandResult::Fail(err);

    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    if (!nom)
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(frame, nom->lx, nom->ly, &mx, &my);

    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_x", mx);
    ctx.CtxSet(prefix + "_y", my);
    // The local pair travels with it. A nominal taught against the wrong feature
    // shows up here as two numbers that do not match the drawing; the machine
    // pair alone can never show that, because it is supposed to change with
    // every part.
    ctx.CtxSet(prefix + "_lx", nom->lx);
    ctx.CtxSet(prefix + "_ly", nom->ly);
    ctx.CtxSet(prefix + "_name", nom->name);

    FD_INFO("align", "'%s': nominal '%s' local (%.4f, %.4f) -> machine (%.4f, %.4f)",
            proc->name.c_str(), nom->name.c_str(), nom->lx, nom->ly, mx, my);
    return CommandResult::Success();
}

// ---- move_local ----------------------------------------------------------

CommandResult ExecMoveLocal(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err);
    if (!proc) return CommandResult::Fail(err);

    const std::string device = p.GetString("device").empty() ? proc->device
                                                             : p.GetString("device");
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(frame, p.GetDouble("x_mm"), p.GetDouble("y_mm"), &mx, &my);

    std::vector<std::pair<std::string, double>> targets = {{"X", mx}, {"Y", my}};
    if (p.Has("z_mm")) targets.emplace_back("Z", p.GetDouble("z_mm"));

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    return MoveWithApproach(ctx, m, idx, targets, p, "the local coordinate");
}

// ---- placement -----------------------------------------------------------
// Put an object that some OTHER device is holding onto a taught nominal. The
// UI twin of these three is ui/pages/PageAlignPlace.cpp; see
// docs/alignment-procedure-design.md §6c for why only a delta can cross
// between two devices whose origins have never been related.

// ctx key for one component of a marked object point: "<prefix>_p1_wx" etc.
std::string ObjKey(const std::string& prefix, int index, const char* field)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s_p%d_%s", prefix.c_str(), index, field);
    return buf;
}

CommandResult ExecPlaceMark(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");
    const int index = p.GetInt("index");
    if (index != 1 && index != 2) return CommandResult::Fail("index must be 1 or 2");

    // The pose device is the one carrying the CAMERA — the alignment's own
    // device — never the one holding the object. A mark is a measurement
    // through the lens, so it is relative to where the lens is.
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, proc->device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    // How the mark is found. A nominal may carry a taught model per mark; that
    // is what makes a placement unattended. Resolve it BEFORE anything else so a
    // stale uid costs nothing, and mirror align_station's auto path exactly —
    // the detector fills the SAME PickResult the crosshair fills, so everything
    // downstream is identical either way.
    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    const VisionModel*  model = nullptr;
    std::string autoErr;
    if (nom) {
        // The point's MODE decides, not whether a model happens to exist: a
        // nominal may keep a taught model while the operator marks this part by
        // hand, exactly as a manual station keeps its detector.
        const std::string mode = (index == 1) ? nom->mode1 : nom->mode2;
        if (mode == "auto") {
            const std::string det = (index == 1) ? nom->det1 : nom->det2;
            model = det.empty() ? nullptr : ctx.FindVisionModel(det);
            if (!model)
                autoErr = det.empty()
                    ? "point " + std::to_string(index) + " is set to detect but names "
                      "no detection model"
                    : "point " + std::to_string(index) + " names no detection "
                      "model '" + det + "'";
            else if (std::string why; !HalconDetector::Available(&why))
                autoErr = "cannot run detection model '" + model->name + "': " + why;
        }
    } else if (!p.GetString("nominal").empty()) {
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");
    }

    ExecutionContext::PickResult pick;
    bool manual = (model == nullptr && autoErr.empty());   // no model asked for at all

    if (model && autoErr.empty()) {
        // Prove the frame is post-move before believing it — the same gate
        // align_station and auto_focus use.
        const int fresh = p.GetInt("fresh_frames");
        if (fresh > 0) {
            const unsigned long long seq0 = ctx.Wf().CameraFrameSeq();
            const auto fr = ctx.WaitUntil(
                [&] {
                    return ctx.Wf().CameraFrameSeq() >=
                           seq0 + static_cast<unsigned long long>(fresh);
                },
                2000, 10);
            if (fr == ExecutionContext::WaitResult::Stopped)
                return CommandResult::Fail("stopped");
            if (fr == ExecutionContext::WaitResult::Timeout)
                FD_WARN("align", "object point %d: no new frame within 2000 ms - "
                                 "detecting on the latest frame (feed stalled?)", index);
        }

        VisionDetection det;
        ctx.Wf().Detect(*model, ctx.VisionDir(), &det);
        if (det.ok) {
            pick.px   = det.xPx;
            pick.py   = det.yPx;
            pick.dxMM = det.dxMM;
            pick.dyMM = det.dyMM;
            FD_INFO("align", "object point %d: '%s' found at (%.2f, %.2f) px, score %.3f",
                    index, model->name.c_str(), det.xPx, det.yPx, det.score);
        } else {
            autoErr = "'" + model->name + "': " + det.error;
        }
    }

    // Detection asked for and not delivered. Falling back is the nominal's
    // choice and defaults ON — but never silently: the warning names what
    // failed, so nobody can believe a run was automatic when it was not.
    if (!autoErr.empty()) {
        if (nom && !nom->autoFallbackManual)
            return CommandResult::Fail(autoErr + " (tick the nominal's fallback, or "
                                       "re-teach the model)");
        FD_WARN("align", "object point %d: %s - asking the operator", index,
                autoErr.c_str());
        manual = true;
    }

    // Nothing drives here: the object has to be in view already, which the
    // script arranges with move_to_position. Keeping the move out means the two
    // marks may come from two different views without this command growing a
    // second station concept.
    ExecutionContext::PickRequest req;
    req.alignName = proc->name;
    req.station   = index;
    // The fallback's evidence, exactly as align_station sets it: the taught
    // patch and the reason, so the operator marks the point the model wanted
    // rather than their own reading of "the corner".
    req.autoError   = autoErr;
    if (!autoErr.empty() && model) req.detectorUid = model->uid;
    // The window TITLE. A caller that repeats the pair (a verify pass) can say
    // so here — without it, the second round is indistinguishable from the
    // first and reads as a second object being placed.
    req.label = ctx.Substitute(p.GetString("label"));
    if (req.label.empty()) {
        char lbl[64];
        std::snprintf(lbl, sizeof(lbl), "point %d of 2 on the object", index);
        req.label = lbl;
    }
    // Two opposite corners of a part is exactly what this command asks for, so
    // this is the one caller the diagrams belong to.
    req.stepIcons = true;

    // What to show the operator alongside the crosshair. Both are computed
    // here, because only the worker has the live frame and the run context.
    //
    // The NOMINAL first: it is the thing the marks are being measured against,
    // and seeing it — or an arrow saying it is 5.6 mm off to the left — is what
    // turns "mark the object" into "mark THIS object".
    if (nom) {
        AlignResult frame;
        if (ctx.Wf().LiveFrame(proc->uid, &frame) && frame.valid) {
            ExecutionContext::PickHint h;
            AlignLocalToMachine(frame, nom->lx, nom->ly, &h.wx, &h.wy);
            h.label = nom->name;
            req.hints.push_back(std::move(h));
        }
    }
    // ...then the mark already taken, so the second one can be placed relative
    // to the first rather than from memory.
    for (int prev = 1; prev < index; ++prev) {
        bool fx = false, fy = false;
        ExecutionContext::PickHint h;
        h.wx = CtxNumber(ctx, ObjKey(p.GetString("ctx_prefix"), prev, "wx"), &fx);
        h.wy = CtxNumber(ctx, ObjKey(p.GetString("ctx_prefix"), prev, "wy"), &fy);
        if (!fx || !fy) continue;
        h.label = "pt" + std::to_string(prev);
        req.hints.push_back(std::move(h));
    }

    req.message = ctx.Substitute(p.GetString("message"));
    if (req.message.empty()) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "Mark point %d of 2 on the object (their MIDPOINT is its "
                      "reference point), then OK", index);
        req.message = buf;
    }

    if (manual && !ctx.AwaitPick(req, &pick))
        return CommandResult::Fail("cancelled by operator");

    AlignPoint e;
    const CommandResult captured = CaptureWorldPoint(ctx, m, pick, &e);
    if (!captured.ok) return captured;

    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(ObjKey(prefix, index, "gx"), e.gx);
    ctx.CtxSet(ObjKey(prefix, index, "gy"), e.gy);
    ctx.CtxSet(ObjKey(prefix, index, "px"), e.px);
    ctx.CtxSet(ObjKey(prefix, index, "py"), e.py);
    ctx.CtxSet(ObjKey(prefix, index, "wx"), e.wx);
    ctx.CtxSet(ObjKey(prefix, index, "wy"), e.wy);

    FD_INFO("align", "object point %d (%s): %s (%.4f, %.4f) + mark (%+.4f, %+.4f) = "
                     "(%.4f, %.4f) mm",
            index, manual ? "manual" : "auto", proc->device.c_str(), e.gx, e.gy,
            pick.dxMM, pick.dyMM, e.wx, e.wy);
    return CommandResult::Success();
}

// ---- pick_point ------------------------------------------------------------
//
// The crosshair pick, with no alignment procedure behind it: show the operator
// the live image, take the mark, and report where that mark IS in machine mm.
//
// It lives in this file rather than in a new one for a single reason —
// CaptureWorldPoint. That function's comment says a second copy reading the pose
// in a different order would be wrong in a way only a part placed 20 um out
// would show, and a needle offset measured through a second copy would be wrong
// the same way. Same maths, same file, one implementation.
//
// Nothing moves here, exactly as in align_station and place_mark. The feature has
// to be in view already; the pick window's own right-click-drive is how an
// operator brings a stray one in.
CommandResult ExecPickPoint(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    ExecutionContext::PickRequest req;
    req.label     = ctx.Substitute(p.GetString("label"));
    if (req.label.empty()) req.label = "the point";
    req.alignName = ctx.Substitute(p.GetString("title"));
    if (req.alignName.empty()) req.alignName = device;
    req.message   = ctx.Substitute(p.GetString("message"));
    if (req.message.empty())
        req.message = "Put the crosshair on " + req.label + ", then OK";
    // stepIcons stays false: this is one mark, not two corners of a part.

    // An optional aim point, drawn as the pick window's target — and, when it is
    // outside the field of view, as an edge arrow with its distance. For a needle
    // imprint that is the difference between "the dot is 4 mm to the left" and a
    // blank grey picture the operator has to hunt across.
    if (p.GetBool("hint")) {
        ExecutionContext::PickHint h;
        h.wx    = p.GetDouble("hint_x_mm");
        h.wy    = p.GetDouble("hint_y_mm");
        h.label = req.label;
        req.hints.push_back(std::move(h));
    }

    // The live residual readout. Separate from the hint on purpose: a hint is
    // where to LOOK, and is drawn on the image; this is how far off the mark
    // still is, in numbers, including a Z the picture cannot show. The device
    // travels with it because the window has to read that Z from somewhere and
    // guessing "the gantry" would be wrong the day a second camera is added.
    if (p.GetBool("target")) {
        req.target.have   = true;
        req.target.x      = p.GetDouble("target_x_mm");
        req.target.y      = p.GetDouble("target_y_mm");
        req.target.z      = p.GetDouble("target_z_mm");
        req.target.tolMm  = p.GetDouble("target_tol_mm");
        req.target.device = device;
    }

    ExecutionContext::PickResult pick;
    if (!ctx.AwaitPick(req, &pick)) return CommandResult::Fail("cancelled by operator");

    AlignPoint e;
    const CommandResult captured = CaptureWorldPoint(ctx, m, pick, &e);
    if (!captured.ok) return captured;

    // Underscore keys, not the dots CommandsVision uses: this is the align
    // family's handshake and its siblings all spell a captured point this way.
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_gx", e.gx);
    ctx.CtxSet(prefix + "_gy", e.gy);
    ctx.CtxSet(prefix + "_px", e.px);
    ctx.CtxSet(prefix + "_py", e.py);
    ctx.CtxSet(prefix + "_dx_mm", pick.dxMM);
    ctx.CtxSet(prefix + "_dy_mm", pick.dyMM);
    ctx.CtxSet(prefix + "_wx", e.wx);
    ctx.CtxSet(prefix + "_wy", e.wy);

    FD_INFO("align", "picked %s: %s (%.4f, %.4f) + mark (%+.4f, %+.4f) = (%.4f, %.4f) mm",
            req.label.c_str(), device.c_str(), e.gx, e.gy, pick.dxMM, pick.dyMM,
            e.wx, e.wy);
    return CommandResult::Success();
}

// ---- place_view ------------------------------------------------------------
//
// Drive the CAMERA to the pose a nominal was taught to be looked at from. It is
// the nominal's own position (`algn_<procUid>_<nomUid>_v`, hidden), so this
// command needs no position parameter and cannot be pointed at an unrelated
// pose — which is exactly what kept a viewing move out of this family until the
// nominal owned one.
//
// An untaught view is NOT an error: every procedure written before the field
// existed drove there with a move_to_position step, and those must keep working.
// A nominal owns TWO camera poses and they are not interchangeable: `_v` looks
// at a gripped OBJECT hovering over the point, `_c` looks at the point ITSELF,
// focused on the substrate. They differ by the height of the grip. Two commands,
// therefore, so a recipe says which one it means and cannot be read as the
// other — over one body, so the wait, the settle and the failure text are the
// same wherever a camera is sent to a nominal.
//
// `focusPose` picks the field; `what` and `button` name it in the messages,
// because "no camera view taught" pointing at the wrong button to press is
// worse than no message at all.
CommandResult DriveToNominalPose(ExecutionContext& ctx, const ParamValues& p,
                                 bool focusPose, double offX, double offY, double offZ)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");
    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    if (!nom)
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");

    const std::string uid    = focusPose ? nom->camUid : nom->viewUid;
    const char* const what   = focusPose ? "camera focus position" : "camera view";
    const char* const button = focusPose ? "Save cam focus" : "Teach view";

    if (uid.empty()) {
        if (p.GetBool("required"))
            return CommandResult::Fail("nominal '" + nom->name + "' has no " + what +
                                       " taught (Alignment page: " + button + ")");
        FD_INFO("align", "'%s': nominal '%s' has no %s - leaving %s where it is",
                proc->name.c_str(), nom->name.c_str(), what, proc->device.c_str());
        return CommandResult::Success();
    }

    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, proc->device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const SavedPos* pos = ctx.FindPosition(proc->device, uid);
    if (!pos)
        return CommandResult::Fail("nominal '" + nom->name + "' names a " + what +
                                   " that no longer exists - re-teach it");
    auto targets = TargetsOf(*pos, m);
    if (targets.empty())
        return CommandResult::Fail(std::string(what) + " shares no axes with '" +
                                   proc->device + "'");

    // The offsets, added to the taught pose in the DEVICE'S OWN AXIS SPACE -
    // the same thing move_to_position_offset does to a saved position, and for
    // the same reason: "that pose, but 0.5 mm higher" is a step somebody writes
    // constantly, and the alternative is a second taught pose per variation.
    //
    // Axis space, not global: this pose was captured axis-by-axis off the
    // device, and gantry-main's transform inverts Z, so a number that took a
    // detour through the global frame would descend where it meant to retreat.
    // An axis the pose does not command is not invented here either - offsetting
    // an axis that was never taught would be a move to a place nobody chose.
    if (offX != 0.0 || offY != 0.0 || offZ != 0.0) {
        bool any = false;
        for (auto& [axis, v] : targets) {
            if (axis == "X")      { v += offX; any = any || offX != 0.0; }
            else if (axis == "Y") { v += offY; any = any || offY != 0.0; }
            else if (axis == "Z") { v += offZ; any = any || offZ != 0.0; }
        }
        if (!any)
            return CommandResult::Fail(std::string(what) + " commands none of the axes "
                                       "the offsets name - check X/Y/Z against '" +
                                       proc->device + "'");
        FD_INFO("align", "'%s': %s offset by (%+.4f, %+.4f, %+.4f) in '%s' axis space",
                proc->name.c_str(), what, offX, offY, offZ, proc->device.c_str());
    }

    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail(std::string("move to the ") + what + " rejected");
    const auto arrived = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                                    p.GetInt("timeout_ms"));
    if (arrived != ExecutionContext::WaitResult::Ok) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));   // defined state on any abort
        return CommandResult::Fail(arrived == ExecutionContext::WaitResult::Stopped
                                       ? "stopped"
                                       : std::string("timeout driving to the ") + what);
    }
    // Settle before anything looks through the lens: a mark taken while the
    // stage is still ringing carries that ring into the placement.
    const CommandResult settled = WaitSettle(ctx, m, idx);
    if (!settled.ok) return settled;

    FD_INFO("align", "'%s': %s at the %s for '%s'", proc->name.c_str(),
            proc->device.c_str(), what, nom->name.c_str());
    return CommandResult::Success();
}

// No offsets: a placement's view is the pose its detection models were taught
// through, and a model matched at a different standoff is matched against a
// picture it never saw. If a placement ever needs one, it needs a re-teach.
CommandResult ExecPlaceView(ExecutionContext& ctx, const ParamValues& p)
{
    return DriveToNominalPose(ctx, p, false, 0.0, 0.0, 0.0);
}

// The focused view of the nominal ITSELF. Its Z is the reason it exists: the
// local frame is 2-D, so a nominal converted through it gives an XY and no
// height at all — this is the one place the height that focuses this point on
// this substrate was ever written down.
CommandResult ExecNominalView(ExecutionContext& ctx, const ParamValues& p)
{
    return DriveToNominalPose(ctx, p, true,
                              p.GetDouble("offset_x_mm"), p.GetDouble("offset_y_mm"),
                              p.GetDouble("offset_z_mm"));
}

// The shared arithmetic behind place_apply and place_move: read the two marks,
// take their midpoint, and difference it against the nominal expressed on THIS
// part. Everything lands in ctx either way, so a dry run and a real one leave
// the same trail.
struct PlaceSolution {
    double objMx = 0.0, objMy = 0.0, objLx = 0.0, objLy = 0.0;
    double nomMx = 0.0, nomMy = 0.0;
    double dMx = 0.0, dMy = 0.0;     // machine-frame correction VECTOR
    double dLx = 0.0, dLy = 0.0;     // the same thing in local (O1) terms
    double dist = 0.0;
};

CommandResult SolvePlacement(ExecutionContext& ctx, const ParamValues& p,
                             const AlignProc** procOut, PlaceSolution* out)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err);
    if (!proc) return CommandResult::Fail(err);
    *procOut = proc;

    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    if (!nom)
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");

    const std::string prefix = p.GetString("ctx_prefix");
    bool ok = true, f = false;
    const double p1x = CtxNumber(ctx, ObjKey(prefix, 1, "wx"), &f); ok = ok && f;
    const double p1y = CtxNumber(ctx, ObjKey(prefix, 1, "wy"), &f); ok = ok && f;
    const double p2x = CtxNumber(ctx, ObjKey(prefix, 2, "wx"), &f); ok = ok && f;
    const double p2y = CtxNumber(ctx, ObjKey(prefix, 2, "wy"), &f); ok = ok && f;
    if (!ok)
        return CommandResult::Fail("no marked object points in ctx under prefix '" +
                                   prefix + "' - run place_mark for 1 and 2 first");

    out->objMx = (p1x + p2x) * 0.5;
    out->objMy = (p1y + p2y) * 0.5;
    AlignMachineToLocal(frame, out->objMx, out->objMy, &out->objLx, &out->objLy);
    // The nominal on THIS part: local coordinates through the frame just taught.
    AlignLocalToMachine(frame, nom->lx, nom->ly, &out->nomMx, &out->nomMy);

    // The correction. Taken in MACHINE mm as nominal - object: the frame's
    // origin cancels, which is what makes the answer meaningful to a device
    // that has never been related to that origin. The rotation does NOT cancel,
    // and must not — both points came through the same live frame.
    out->dMx  = out->nomMx - out->objMx;
    out->dMy  = out->nomMy - out->objMy;
    out->dLx  = nom->lx - out->objLx;
    out->dLy  = nom->ly - out->objLy;
    out->dist = std::sqrt(out->dMx * out->dMx + out->dMy * out->dMy);

    ctx.CtxSet(prefix + "_obj_x",  out->objMx);
    ctx.CtxSet(prefix + "_obj_y",  out->objMy);
    ctx.CtxSet(prefix + "_obj_lx", out->objLx);
    ctx.CtxSet(prefix + "_obj_ly", out->objLy);
    ctx.CtxSet(prefix + "_nom_x",  out->nomMx);
    ctx.CtxSet(prefix + "_nom_y",  out->nomMy);
    ctx.CtxSet(prefix + "_dx",     out->dMx);
    ctx.CtxSet(prefix + "_dy",     out->dMy);
    ctx.CtxSet(prefix + "_dlx",    out->dLx);
    ctx.CtxSet(prefix + "_dly",    out->dLy);
    ctx.CtxSet(prefix + "_dist",   out->dist);
    return CommandResult::Success();
}

CommandResult ExecPlaceApply(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = nullptr;
    PlaceSolution s;
    const CommandResult solved = SolvePlacement(ctx, p, &proc, &s);
    if (!solved.ok) return solved;

    FD_INFO("align", "'%s' -> '%s': object (%.4f, %.4f), nominal (%.4f, %.4f), "
                     "delta (%+.4f, %+.4f) = %.4f mm",
            proc->name.c_str(), p.GetString("nominal").c_str(), s.objMx, s.objMy,
            s.nomMx, s.nomMy, s.dMx, s.dMy, s.dist);
    return CommandResult::Success();
}

CommandResult ExecPlaceMove(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = nullptr;
    PlaceSolution s;
    const CommandResult solved = SolvePlacement(ctx, p, &proc, &s);
    if (!solved.ok) return solved;

    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, p.GetString("device"), &idx, &err);
    if (!m) return CommandResult::Fail(err);

    // The ceiling the page puts on screen, as a hard failure here: unattended,
    // there is nobody to notice that a mis-taught frame turned a 0.2 mm nudge
    // into a 20 mm lunge. A correction bigger than this is a broken input, not
    // a big correction.
    const double maxMM = p.GetDouble("max_mm");
    if (s.dist > maxMM) {
        char buf[192];
        std::snprintf(buf, sizeof(buf),
                      "correction %.4f mm exceeds max_mm %.4f - check the frame and "
                      "the marks (object (%.4f, %.4f), nominal (%.4f, %.4f))",
                      s.dist, maxMM, s.objMx, s.objMy, s.nomMx, s.nomMy);
        return CommandResult::Fail(buf);
    }

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    // Layer 2 maps the machine-frame vector onto this device's own axes and adds
    // it to the live pose. The targets are worked out BEFORE anything moves,
    // because a two-speed approach has to split the distance and cannot split a
    // move already under way — and because the arrival wait then watches exactly
    // what was commanded rather than recomputing from a moving pose.
    std::vector<std::pair<std::string, double>> targets;
    if (!ctx.Wf().GlobalDeltaTargets(static_cast<std::size_t>(idx), s.dMx, s.dMy, 0.0,
                                     &targets))
        return CommandResult::Fail("placement move rejected");

    const CommandResult moved =
        MoveWithApproach(ctx, m, idx, targets, p, "the placement correction");
    if (!moved.ok) return moved;

    // The marks describe where the object WAS. Drop them so a second
    // place_apply cannot silently reuse them and report a correction that has
    // already been spent — the same reason the page clears them.
    const std::string prefix = p.GetString("ctx_prefix");
    for (int i = 1; i <= 2; ++i)
        for (const char* fld : {"gx", "gy", "px", "py", "wx", "wy"})
            ctx.CtxErase(ObjKey(prefix, i, fld));

    FD_INFO("align", "'%s': placement move of %.4f mm issued on %s - re-mark to "
                     "measure the residual",
            proc->name.c_str(), s.dist, p.GetString("device").c_str());
    return CommandResult::Success();
}

} // namespace

void RegisterAlignCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // every action goes through ExecutionContext::Wf() at run time
    (void)hw;
    using PT = ParamType;

    reg.Register({"align_station", "Align",
                  "Drive to an alignment station and capture its E point",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"station", PT::IntChoice, true, {}, 1, 2,
                       {"E1 - origin", "E2 - X+"},
                       "1 = E1 (origin), 2 = E2 (X+)"},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction shown to the operator ({key} substitutes ctx values)"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "captured point goes to <prefix>_e<N>_gx/gy/px/py/wx/wy"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "auto stations only: new frames required after arrival "
                       "before detecting (0 = use the latest, blind)"},
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecAlignStation});

    reg.Register({"align_solve", "Align",
                  "Build the local frame (O1) from the captured E1/E2 and store it",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "reads <prefix>_e1_*/<prefix>_e2_*, writes <prefix>_origin_x/_origin_y/"
                       "_theta_deg/_span_mm/_valid"},
                      {"save", PT::Bool, false, true, 0, 0, {},
                       "persist the taught frame into the procedure document"},
                  },
                  CommandSpec::Needs::None, ExecAlignSolve});

    reg.Register({"align_apply", "Align",
                  "Convert a local (O1) coordinate to machine XY into the run context",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"x_mm", PT::Double, true, {}, 0, 0, {}, "local X (mm from E1 along E1->E2)"},
                      {"y_mm", PT::Double, true, {}, 0, 0, {}, "local Y (mm, X rotated +90 deg)"},
                      {"ctx_prefix", PT::String, false, "local", 0, 0, {},
                       "result goes to <prefix>_x / <prefix>_y"},
                  },
                  CommandSpec::Needs::None, ExecAlignApply});

    // align_apply for a nominal instead of two typed numbers. Registered here,
    // beside it, because it is the same conversion — what it adds is that the
    // caller does not have to already know the coordinates of the thing it is
    // asking about, which is what stood between a taught nominal and a
    // registered device's first approach.
    reg.Register({"nominal_point", "Align",
                  "Where a nominal IS in machine XY, through the live frame - no "
                  "marks and no motion (<prefix>_x/_y/_lx/_ly/_name)",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "the nominal to resolve (uid or name)"},
                      {"ctx_prefix", PT::String, false, "nom", 0, 0, {},
                       "result goes to <prefix>_x / _y / _lx / _ly / _name"},
                  },
                  CommandSpec::Needs::None, ExecNominalPoint});

    reg.Register({"move_local", "Align",
                  "Move to a coordinate expressed in an alignment's local frame",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"device", PT::DeviceMotion, false, "", 0, 0, {},
                       "motion device; empty = the procedure's own device"},
                      {"x_mm", PT::Double, true, {}, 0, 0, {}, "local X (mm)"},
                      {"y_mm", PT::Double, true, {}, 0, 0, {}, "local Y (mm)"},
                      {"z_mm", PT::Double, false, {}, 0, 0, {},
                       "optional machine Z (the frame is 2-D; Z is not transformed)"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveLocal});

    // ---- placement ------------------------------------------------------
    reg.Register({"place_view", "Align",
                  "Drive the camera to a nominal's taught viewing pose",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure - its device is the one carrying the camera"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "nominal whose camera view to drive to"},
                      {"required", PT::Bool, false, false, 0, 0, {},
                       "fail when the nominal has no view taught; off = leave the "
                       "camera where it is (a run that moves it some other way)"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecPlaceView});

    // Its sibling, and registered right beside it so nobody picks one meaning
    // the other: place_view looks at the OBJECT above the point, this looks at
    // the POINT. Under Align rather than a placement group, because looking at
    // a nominal is not a placement step - it is what a check, a measurement or
    // an operator wants.
    reg.Register({"nominal_view", "Align",
                  "Drive the camera to a nominal's taught FOCUS pose - the point "
                  "itself, in focus, including the Z - plus an optional XYZ offset",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure - its device is the one carrying the camera"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "nominal whose camera FOCUS position to drive to (Alignment "
                       "page: Centre camera, focus, Save cam focus)"},
                      {"required", PT::Bool, false, false, 0, 0, {},
                       "fail when the nominal has no focus pose taught; off = leave "
                       "the camera where it is"},
                      {"offset_x_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the taught pose's X, in the DEVICE'S OWN axis space "
                       "(not global) - the same thing move_to_position_offset does"},
                      {"offset_y_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the taught pose's Y, device axis space"},
                      {"offset_z_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the taught pose's Z, device axis space - a standoff "
                       "from the focused height. Which way +Z counts is a property of "
                       "the machine (gantry-main's transform inverts Z), so try a "
                       "small value and watch before trusting a large one"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecNominalView});

    reg.Register({"place_mark", "Align",
                  "Ask the operator to mark one point on a gripped object",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure - its device is the one carrying the camera"},
                      {"index", PT::IntChoice, true, {}, 1, 2,
                       {"pt1 - first corner", "pt2 - opposite corner"},
                       "1 or 2; the MIDPOINT is the point"},
                      {"nominal", PT::NominalUid, false, "", 0, 0, {},
                       "nominal whose taught detector finds this mark; empty = always ask"},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction shown to the operator ({key} substitutes ctx values)"},
                      {"label", PT::String, false, "", 0, 0, {},
                       "window title text; say which pass this is when repeating"},
                      {"ctx_prefix", PT::String, false, "place", 0, 0, {},
                       "marked point goes to <prefix>_p<N>_gx/gy/px/py/wx/wy"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "detected marks only: new frames required before detecting "
                       "(0 = use the latest, blind)"},
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecPlaceMark});

    reg.Register({"place_apply", "Align",
                  "Compute the correction from the marked object to a nominal (no motion)",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "nominal on that procedure (uid or name)"},
                      {"ctx_prefix", PT::String, false, "place", 0, 0, {},
                       "reads <prefix>_p1_*/<prefix>_p2_*, writes <prefix>_obj_x/_obj_y/"
                       "_obj_lx/_obj_ly/_nom_x/_nom_y/_dx/_dy/_dlx/_dly/_dist"},
                  },
                  CommandSpec::Needs::None, ExecPlaceApply});

    reg.Register({"place_move", "Align",
                  "Move the holding device so the marked object lands on a nominal",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "nominal on that procedure (uid or name)"},
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device HOLDING the object (not the camera's)"},
                      {"max_mm", PT::Double, false, 2.0, 0.0001, 50.0, {},
                       "refuse a correction larger than this - unattended, a "
                       "mis-taught frame has nobody to notice it"},
                      {"ctx_prefix", PT::String, false, "place", 0, 0, {},
                       "same prefix place_mark wrote; the marks are DROPPED after the move"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecPlaceMove});

    // Registered under Vision rather than Align: it belongs to no procedure, and
    // grouping it with the frame commands would suggest it teaches one.
    reg.Register({"pick_point", "Vision",
                  "Ask the operator to mark a point on the live image; report it in "
                  "machine mm as <prefix>_wx/_wy",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device the camera rides on - its pose is what the mark is "
                       "measured against"},
                      {"label", PT::String, false, "", 0, 0, {},
                       "what the operator is marking, e.g. \"the needle imprint\""},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction line; empty builds one from the label"},
                      {"title", PT::String, false, "", 0, 0, {},
                       "window subtitle; empty uses the device name"},
                      {"hint", PT::Bool, false, false, 0, 0, {},
                       "draw an aim point at hint_x_mm/hint_y_mm - shown as an edge "
                       "arrow with its distance when it is outside the field of view"},
                      {"hint_x_mm", PT::Double, false, 0.0, 0, 0, {}, "aim point X, machine mm"},
                      {"hint_y_mm", PT::Double, false, 0.0, 0, 0, {}, "aim point Y, machine mm"},
                      {"target", PT::Bool, false, false, 0, 0, {},
                       "show the LIVE residual against target_x/y/z_mm while the "
                       "window is open - dX/dY from the crosshair, dZ from the "
                       "device's own Z, so a hand focus can be watched in"},
                      {"target_x_mm", PT::Double, false, 0.0, 0, 0, {}, "predicted X, machine mm"},
                      {"target_y_mm", PT::Double, false, 0.0, 0, 0, {}, "predicted Y, machine mm"},
                      {"target_z_mm", PT::Double, false, 0.0, 0, 0, {}, "predicted Z, machine mm"},
                      {"target_tol_mm", PT::Double, false, 0.0, 0, 0, {},
                       "what counts as on target; 0 shows the number with no verdict"},
                      {"ctx_prefix", PT::String, false, "pick", 0, 0, {},
                       "ctx prefix for the captured point"},
                  },
                  CommandSpec::Needs::Camera, ExecPickPoint});

    // Typed one-step wrapper over the procedure script — the `xcan` pattern, so
    // the procedure stays an editable .lua while a recipe runs it as one step.
    CommandRegistry* rp = &reg;
    reg.Register({"align_run", "Align",
                  "Run a whole alignment procedure (alignment_procedure.lua)",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "ctx prefix the script captures and solves under"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/alignment_procedure.lua", 0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file - point it at a different name "
                       "to run something else"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("align_name", p.GetString("alignment"));
                      c.CtxSet("align_prefix", p.GetString("ctx_prefix"));
                      // A product's own alignment_procedure.lua wins over the
                      // shipped one — the Alignment page's Run button already
                      // prefers it, and "the same procedure" must not mean a
                      // different file depending on which surface launched it.
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});

    // The placement equivalent, and the reason the whole thing is a script: the
    // procedure is a LOOP — measure, correct, measure again, stop when the
    // residual is inside tolerance — and a loop with an operator in it belongs
    // somewhere editable, not compiled into a step.
    //
    // It DOES drive to a viewing position now, when the nominal owns one: that
    // is `place_view` inside the script, and it needs no parameter here because
    // the pose belongs to the nominal. What is still refused is a free-floating
    // `position` parameter — a raw uid in a text box, and the second command in
    // this family pretending to be a mover. A nominal with no view taught keeps
    // the old contract: get the object in front of the camera with a
    // `move_to_position` step above this one.
    reg.Register({"place_run", "Align",
                  "Run a whole placement (placement_procedure.lua)",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "nominal to place the object on"},
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device HOLDING the object"},
                      {"align_first", PT::Bool, false, true, 0, 0, {},
                       "re-teach the frame before placing; off when the program "
                       "already aligned this part and is placing several objects"},
                      {"go_to_view", PT::Bool, false, true, 0, 0, {},
                       "drive the camera to the nominal's taught view first; a "
                       "nominal without one never moves it either way"},
                      {"tries", PT::Int, false, 2, 1, 5, {},
                       "measure/correct attempts before the residual must be inside tol"},
                      {"tol_mm", PT::Double, false, 0.02, 0.0001, 5.0, {},
                       "residual that counts as placed"},
                      {"max_mm", PT::Double, false, 2.0, 0.0001, 50.0, {},
                       "largest single correction allowed"},
                      {"ctx_prefix", PT::String, false, "place", 0, 0, {},
                       "ctx prefix the script marks and solves under"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/placement_procedure.lua", 0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file - point it at a different name "
                       "to run something else"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("place_alignment", p.GetString("alignment"));
                      c.CtxSet("place_nominal",   p.GetString("nominal"));
                      c.CtxSet("place_device",    p.GetString("device"));
                      c.CtxSet("place_align_first", p.GetBool("align_first"));
                      c.CtxSet("place_go_to_view",  p.GetBool("go_to_view"));
                      c.CtxSet("place_tries",     static_cast<double>(p.GetInt("tries")));
                      c.CtxSet("place_tol_mm",    p.GetDouble("tol_mm"));
                      c.CtxSet("place_max_mm",    p.GetDouble("max_mm"));
                      c.CtxSet("place_prefix",    p.GetString("ctx_prefix"));
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});
}

} // namespace firedog
