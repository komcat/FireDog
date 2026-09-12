// CommandsMotion.cpp : motion + pneumatic-slide commands.
//
// All of these run on the RecipeEngine worker thread. Blocking moves issue the
// Workflow command, then wait for arrival by polling the driver's cached
// positions through ExecutionContext::WaitUntil — so a stop request interrupts
// within one poll interval, and a timeout stops the device (defined state).
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace firedog {

// Same arrival rule as Workflow::TickPath. Declared in MotionWaits.h as
// kMoveTolMM; the local alias keeps the schema tables below unchanged.
constexpr double kDefaultTolMM = kMoveTolMM;

// ---- Shared move waits (declared in MotionWaits.h; CommandsVision uses them) --

// Wait until every targeted axis is within tolMM of its target. Keeps the
// driver's fast-poll window open so the check runs on fresh positions.
ExecutionContext::WaitResult WaitArrive(
    ExecutionContext& ctx, IMotionController* m,
    const std::vector<std::pair<std::string, double>>& targets, double tolMM,
    int timeoutMs)
{
    return ctx.WaitUntil(
        [&] {
            m->RequestFastPoll();
            const std::map<std::string, double> pos = m->Positions();
            for (const auto& [axis, tgt] : targets) {
                const auto it = pos.find(axis);
                if (it == pos.end() || std::fabs(it->second - tgt) > tolMM) return false;
            }
            return true;
        },
        timeoutMs, 50);
}

// After position arrival the controller can still be finishing the tail of
// its motion profile — WaitArrive's tolerance LEADS the controller's own
// motion-complete state, and the PI hexapod rejects VLS (so the next step's
// velocity change) with GCS error 1005 until the profile is done. Every move
// command ends with this settle-wait so the following step starts on a truly
// idle controller. Two signals, both required: profile done (IsMoving false)
// AND on-target (qONT — position inside the controller's settling window for
// the settling time; profile-done alone can lead mechanical settle while the
// platform is still ringing, which poisoned scan measurements). Drivers that
// can't tell (or don't care — ACS) report both immediately.
CommandResult WaitSettle(ExecutionContext& ctx, IMotionController* m, int idx)
{
    const auto r = ctx.WaitUntil(
        [&] { return m->IsSettled() && m->IsOnTarget(); }, 5000, 50);
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped" : "timeout waiting for motion settle");
}

// The two-speed move — see MotionWaits.h for the contract and why the ordering
// below is load-bearing. Lifted out of what used to be move_to_position_approach
// so every point-to-point command shares one implementation.
CommandResult MoveWithApproach(ExecutionContext& ctx, IMotionController* m, int idx,
                               const std::vector<std::pair<std::string, double>>& targets,
                               const ParamValues& p, const std::string& what)
{
    if (targets.empty()) return CommandResult::Fail("nothing to move: no targets");

    const double tol       = p.GetDouble("tol_mm");
    const int    timeoutMs = p.GetInt("timeout_ms");
    const double approachV = p.GetDouble("approach_velocity_mm_s");
    const double approachMm  = p.GetDouble("approach_mm");
    const double approachPct = p.GetDouble("approach_pct");
    const bool   wantZone    = approachMm > 0.0 || approachPct > 0.0;

    // Half configured is an authoring mistake with a dangerous failure mode: a
    // zone with no slow velocity would arrive at full speed while the recipe
    // claims a careful approach. Name the missing half instead.
    if (approachV > 0.0 && !wantZone)
        return CommandResult::Fail("approach_velocity_mm_s is set but no zone - "
                                   "set approach_mm or approach_pct");
    if (approachV <= 0.0 && wantZone)
        return CommandResult::Fail("an approach zone is set but no approach velocity - "
                                   "set approach_velocity_mm_s");

    // The plain move, byte for byte what every one of these commands did before
    // the approach existed. No live-position read, no extra leg, no cost.
    if (approachV <= 0.0) {
        if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
            return CommandResult::Fail("move to " + what + " rejected");
        const auto r = WaitArrive(ctx, m, targets, tol, timeoutMs);
        if (r == ExecutionContext::WaitResult::Ok) return WaitSettle(ctx, m, idx);
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
        return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                       ? "stopped" : "timeout driving to " + what);
    }

    m->RequestFastPoll();
    const std::map<std::string, double> cur = m->Positions();
    double dist2 = 0.0;
    for (const auto& [axis, tgt] : targets) {
        const auto it = cur.find(axis);
        if (it == cur.end())
            return CommandResult::Fail("no live position for axis '" + axis + "'");
        dist2 += (tgt - it->second) * (tgt - it->second);
    }
    const double dist = std::sqrt(dist2);
    // Already there. Not an error and not worth two legs — a step that re-issues
    // a move it has already made must still succeed, and a percentage zone of
    // zero distance is zero.
    if (dist < 1e-9) {
        if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
            return CommandResult::Fail("move to " + what + " rejected");
        return WaitSettle(ctx, m, idx);
    }
    const double approach = approachMm > 0.0 ? approachMm : dist * approachPct / 100.0;
    if (approach <= 0.0)
        return CommandResult::Fail("the approach zone works out to zero");

    // Put the device back on the velocity the STEP declared, whatever happens
    // below. There is no read-back on IMotionController, so a step that left
    // velocity_mm_s at 0 ("keep current") has nothing to restore to — say so
    // rather than silently handing the next move a crawl.
    const double travelV = p.GetDouble("velocity_mm_s");
    bool slowed = false;
    const auto restore = [&]() {
        if (!slowed) return;
        if (travelV > 0.0) {
            ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), travelV);
        } else {
            FD_WARN("motion", "%s: left at the approach velocity %.3f mm/s - set "
                              "velocity_mm_s on this step to restore a travel speed",
                    m->Name().c_str(), approachV);
        }
        slowed = false;
    };

    // Travel leg, skipped when we are already inside the approach zone. Its
    // arrival check is deliberately looser: the intermediate point is not a
    // place anyone cares about, it is just where the slowing down starts.
    if (approach < dist) {
        const double frac = (dist - approach) / dist;
        std::vector<std::pair<std::string, double>> inter;
        inter.reserve(targets.size());
        for (const auto& [axis, tgt] : targets) {
            const double c = cur.at(axis);
            inter.emplace_back(axis, c + (tgt - c) * frac);
        }
        if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), inter))
            return CommandResult::Fail("travel leg to " + what + " rejected");
        const auto r = WaitArrive(ctx, m, inter, tol > 0.05 ? tol : 0.05, timeoutMs);
        if (r != ExecutionContext::WaitResult::Ok) {
            ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
            return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                           ? "stopped" : "timeout on the travel leg");
        }
        // The velocity change below is rejected while the controller is still
        // finishing this leg (PI VLS -> GCS 1005), so settle before asking.
        const CommandResult s = WaitSettle(ctx, m, idx);
        if (!s.ok) return s;
    }

    if (!ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), approachV))
        return CommandResult::Fail("approach velocity change rejected");
    slowed = true;

    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets)) {
        restore();
        return CommandResult::Fail("approach leg to " + what + " rejected");
    }
    const auto r = WaitArrive(ctx, m, targets, tol, timeoutMs);
    if (r != ExecutionContext::WaitResult::Ok) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
        restore();
        return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                       ? "stopped" : "timeout driving to " + what);
    }
    const CommandResult settled = WaitSettle(ctx, m, idx);
    restore();
    return settled;
}

ParamSpec ApproachVelocityParam(bool required)
{
    nlohmann::json def;              // null = no default, i.e. the param is required
    if (!required) def = 0.0;        // 0 = no approach leg at all
    return {"approach_velocity_mm_s", ParamType::Double, required, def, 0.0, 100.0, {},
            "velocity for the final approach leg; 0 = arrive at the travel speed",
            PinDir::Any, "Approach"};
}

ParamSpec ApproachMmParam()
{
    return {"approach_mm", ParamType::Double, false, 0.0, 0, 0, {},
            "approach zone as mm before the target (wins over approach_pct)",
            PinDir::Any, "Approach"};
}

ParamSpec ApproachPctParam()
{
    return {"approach_pct", ParamType::Double, false, 0.0, 0.0, 100.0, {},
            "approach zone as % of the total distance",
            PinDir::Any, "Approach"};
}

// Resolve device + connected controller, or fail with a message.
IMotionController* ResolveMotion(ExecutionContext& ctx, const std::string& device,
                                 int* idxOut, std::string* err)
{
    const int idx = ctx.MotionIndexByName(device);
    if (idx < 0) {
        *err = "unknown motion device '" + device + "'";
        return nullptr;
    }
    IMotionController* m = ctx.Hw().MotionDevice(static_cast<std::size_t>(idx));
    if (!m || !m->IsConnected()) {
        *err = "motion device '" + device + "' is not connected";
        return nullptr;
    }
    if (idxOut) *idxOut = idx;
    return m;
}

// Position uid -> targets restricted to the device's axes. Declared in
// MotionWaits.h: the registration family drives a saved position too, and two
// copies of this loop would be two answers to which axes a position commands.
std::vector<std::pair<std::string, double>> TargetsOf(const SavedPos& p,
                                                      const IMotionController* m)
{
    std::vector<std::pair<std::string, double>> targets;
    for (const std::string& a : m->Axes()) {
        const auto it = p.vals.find(a);
        if (it != p.vals.end()) targets.emplace_back(a, it->second);
    }
    return targets;
}

ParamSpec TeachParam()
{
    return {kTeachParamName, ParamType::Bool, false, false, 0, 0, {},
            "after arriving, open Manual Adjust so the operator can fine-tune and "
            "Save this position (Skip keeps the saved value)"};
}

CommandResult TeachAfterMove(ExecutionContext& ctx, IMotionController* m,
                             const std::string& device, const SavedPos& pos,
                             const std::map<std::string, double>& subtract)
{
    ExecutionContext::AdjustRequest req;
    req.message = "Teach '" + std::string(pos.name) + "' on " + device +
                  ": jog onto the exact spot, then Save. Skip keeps the saved value.";
    req.device       = device;
    req.stepUm       = 10.0;
    req.teach        = true;
    req.positionName = pos.name;
    req.target       = pos.vals;
    if (!subtract.empty()) {
        // Show the point the operator is actually standing on (base + offset),
        // not the base they are teaching - that is where the crosshair is.
        for (const auto& [axis, off] : subtract) {
            const auto it = req.target.find(axis);
            if (it != req.target.end()) it->second += off;
        }
        req.message += " (offset move: the base position is saved, offset removed)";
    }

    const bool save = ctx.AwaitAdjust(req);
    if (ctx.ShouldStop()) return CommandResult::Fail("stopped");
    if (!save) {
        FD_INFO("recipe", "teach '%s' on %s: skipped, saved value kept", pos.name,
                device.c_str());
        return CommandResult::Success();
    }

    // Same recipe as save_position: reopen the fast-poll window and outlast one
    // cycle, so the pose read is the one the last jog left, not a stale cache.
    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> live = m->Positions();
    std::map<std::string, double> vals;
    for (const auto& kv : pos.vals) {
        const std::string& axis = kv.first;
        const auto li = live.find(axis);
        if (li == live.end()) continue;   // axis the device does not report
        double v = li->second;
        const auto si = subtract.find(axis);
        if (si != subtract.end()) v -= si->second;
        vals[axis] = v;
    }
    if (vals.empty()) return CommandResult::Fail("teach: device reports none of the position's axes");
    if (!ctx.RecordPositionByUid(device, pos.uid, vals))
        return CommandResult::Fail("teach: position '" + std::string(pos.name) +
                                   "' is no longer in the run's positions");
    return CommandResult::Success();
}

// ---- File-local from here on ------------------------------------------------
namespace {

// Optional pre-move actions shared by EVERY motion command: a device velocity
// override (persists on the device — same semantics as the Jog panel; there
// is no read-back to restore from) and a camera exposure change issued BEFORE
// the move, so the sensor settles while the axes travel and vision is ready
// on arrival.
CommandResult ApplyPreMove(ExecutionContext& ctx, int idx, const ParamValues& p)
{
    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");
    const double expo = p.GetDouble("exposure_us");
    if (expo > 0.0 && !ctx.Wf().SetCameraExposure(expo))
        return CommandResult::Fail("camera exposure change rejected (camera connected?)");
    return CommandResult::Success();
}

// The two shared optional params, appended to every motion command's schema.
ParamSpec VelocityParam()
{
    return {"velocity_mm_s", ParamType::Double, false, 0.0, 0, 0, {},
            "optional velocity for this move; 0 = keep current (persists on the device)"};
}
ParamSpec ExposureParam()
{
    return {"exposure_us", ParamType::Double, false, 0.0, 0, 0, {},
            "optional camera exposure set BEFORE moving (vision-ready on arrival); 0 = leave"};
}

CommandResult ExecMoveToPosition(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const SavedPos* pos = ctx.FindPosition(device, p.GetString("position"));
    if (!pos) return CommandResult::Fail("position uid not found");
    const auto targets = TargetsOf(*pos, m);
    if (targets.empty()) return CommandResult::Fail("position shares no axes with device");

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;
    const CommandResult mv =
        MoveWithApproach(ctx, m, idx, targets, p, "'" + std::string(pos->name) + "'");
    if (!mv.ok || !p.GetBool(kTeachParamName)) return mv;
    return TeachAfterMove(ctx, m, device, *pos);
}

CommandResult ExecMovePath(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const MotionMap* map = ctx.FindMap(device, p.GetString("map"));
    if (!map) return CommandResult::Fail("map not found");
    const std::string fromUid = p.GetString("from");
    const SavedPos*   fromPos = ctx.FindPosition(device, fromUid);
    if (!fromPos) return CommandResult::Fail("'from' position uid not found");

    // STRICT START (design decision): the device must already be at `from` on
    // every axis the position defines — no nearest-node snapping, no implicit
    // move. Reach the start with an explicit move_to_position step.
    const double startTol = p.GetDouble("start_tol_mm");
    m->RequestFastPoll();
    const std::map<std::string, double> live = m->Positions();
    for (const auto& [axis, v] : fromPos->vals) {
        const auto it = live.find(axis);
        if (it == live.end()) continue;   // axis the device doesn't report
        if (std::fabs(it->second - v) > startTol) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                          "not at start '%s': axis %s is %.4f, expected %.4f (tol %.4f)",
                          fromPos->name, axis.c_str(), it->second, v, startTol);
            return CommandResult::Fail(buf);
        }
    }

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;

    const auto posIt = ctx.Positions().find(device);
    if (posIt == ctx.Positions().end())
        return CommandResult::Fail("device has no saved positions");
    std::string routeErr;
    const std::vector<MapRouteLeg> legs = BuildMapRoute(
        *map, posIt->second, m->Axes(), fromUid, p.GetString("to"), &routeErr);
    if (legs.empty()) return CommandResult::Fail(routeErr);

    const int legTimeout = p.GetInt("leg_timeout_ms");
    for (std::size_t i = 0; i < legs.size(); ++i) {
        const MapRouteLeg& leg = legs[i];
        if (leg.targets.empty()) continue;   // node with no shared axes (first = start)
        if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), leg.targets))
            return CommandResult::Fail("move to '" + leg.label + "' rejected");
        const auto r = WaitArrive(ctx, m, leg.targets, kDefaultTolMM, legTimeout);
        if (r != ExecutionContext::WaitResult::Ok) {
            ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
            return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                           ? "stopped"
                                           : "timeout driving to '" + leg.label + "'");
        }
        FD_INFO("recipe", "%s: reached '%s' (%zu/%zu)", device.c_str(),
                leg.label.c_str(), i + 1, legs.size());
    }
    const CommandResult settled = WaitSettle(ctx, m, idx);
    if (!settled.ok || !p.GetBool(kTeachParamName)) return settled;
    // Only the destination is taught: the intermediate nodes are the map's,
    // each reached by its own step somewhere else.
    const SavedPos* toPos = ctx.FindPosition(device, p.GetString("to"));
    if (!toPos) return CommandResult::Fail("'to' position uid not found");
    return TeachAfterMove(ctx, m, device, *toPos);
}

CommandResult ExecMoveRelativeGlobal(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const double dx = p.GetDouble("dx_mm");
    const double dy = p.GetDouble("dy_mm");
    const double dz = p.GetDouble("dz_mm");
    if (dx == 0.0 && dy == 0.0 && dz == 0.0) return CommandResult::Success();

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;

    // One JogGlobal per non-zero global axis: the transform maps it to the
    // device's signed local axes; consecutive calls accumulate onto one
    // evolving target (JogTrack), which is exactly what we want here.
    if (dx != 0.0 && !ctx.Wf().JogGlobal(static_cast<std::size_t>(idx), 1, 0, 0, dx))
        return CommandResult::Fail("global X move rejected (unmapped axis?)");
    if (dy != 0.0 && !ctx.Wf().JogGlobal(static_cast<std::size_t>(idx), 0, 1, 0, dy))
        return CommandResult::Fail("global Y move rejected (unmapped axis?)");
    if (dz != 0.0 && !ctx.Wf().JogGlobal(static_cast<std::size_t>(idx), 0, 0, 1, dz))
        return CommandResult::Fail("global Z move rejected (unmapped axis?)");

    // Settle-based arrival: the command doesn't know the local targets (the
    // transform does), so wait until positions stop changing — 3 consecutive
    // stable polls after an initial motion-start grace.
    if (!ctx.Sleep(300)) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
        return CommandResult::Fail("stopped");
    }
    std::map<std::string, double> prev = m->Positions();
    int stable = 0;
    const auto r = ctx.WaitUntil(
        [&] {
            m->RequestFastPoll();
            const std::map<std::string, double> cur = m->Positions();
            bool same = true;
            for (const auto& [axis, v] : cur) {
                const auto it = prev.find(axis);
                if (it == prev.end() || std::fabs(it->second - v) > 0.005) {
                    same = false;
                    break;
                }
            }
            prev = cur;
            stable = same ? stable + 1 : 0;
            return stable >= 3;
        },
        p.GetInt("timeout_ms"), 100);
    if (r == ExecutionContext::WaitResult::Ok) return WaitSettle(ctx, m, idx);
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped" : "timeout waiting for settle");
}

// Single-axis LOCAL relative move: target = current + delta, absolute move,
// wait for arrival (precise, unlike the settle-based global variant).
CommandResult ExecMoveRelative(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const std::string axis  = p.GetString("axis");
    const double      delta = p.GetDouble("delta_mm");
    if (delta == 0.0) return CommandResult::Success();
    m->RequestFastPoll();
    const std::map<std::string, double> cur = m->Positions();
    const auto it = cur.find(axis);
    if (it == cur.end())
        return CommandResult::Fail("device does not report axis '" + axis + "'");

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;
    // One axis, so the helper's Euclidean distance is simply |delta| — the
    // approach zone means the same thing here as on a coordinated move.
    const std::vector<std::pair<std::string, double>> targets = {
        {axis, it->second + delta}};
    return MoveWithApproach(ctx, m, idx, targets, p, "the relative target");
}

// Read the device's live pose into ctx as <prefix>_<axis>, lowercase — the same
// spelling manual_adjust already uses, so a script that jogs and a script that
// merely looks name the pose identically.
//
// This exists because until now the ONLY way a script could see a coordinate was
// as a side effect of asking the operator to jog: save_position writes the pose
// to the document but hands nothing back, and the needle procedures need to do
// arithmetic on a pose nobody touched.
CommandResult ExecReadPosition(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    // Same freshness rule as save_position: the idle poll cache can be a second
    // old, and a pose read one second after a move is a pose from before it.
    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> pos = m->Positions();
    if (pos.empty())
        return CommandResult::Fail("no live positions for '" + device + "'");

    const std::string prefix = p.GetString("ctx_prefix");
    for (const auto& [axis, val] : pos) {
        std::string key = prefix + "_";
        for (const char c : axis)
            key += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        ctx.CtxSet(key, val);
    }
    return CommandResult::Success();
}

// Coordinated absolute move to explicit AXIS coordinates — the frameless twin of
// move_local, for a caller that computed the numbers itself.
//
// `axes` is REQUIRED and has no default on purpose. Every other coordinate here
// could sensibly default to 0, and a step that meant "descend Z" but silently
// also commanded X=0 and Y=0 would drive the head across the machine at speed.
// Naming the axes is one word, and it is the word that makes the omission
// impossible.
CommandResult ExecMoveAxisTo(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const std::string axes = p.GetString("axes");
    if (axes.empty()) return CommandResult::Fail("axes is empty - name the axes to command");

    // Axes(), not Positions(): the question is whether this device HAS the axis,
    // which is a fact about the device, not about whether it has polled a value
    // for it yet.
    const std::vector<std::string> have = m->Axes();
    std::vector<std::pair<std::string, double>> targets;
    for (const char raw : axes) {
        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(raw)));
        const std::string axis(1, c);
        double val = 0.0;
        if (c == 'X')      val = p.GetDouble("x_mm");
        else if (c == 'Y') val = p.GetDouble("y_mm");
        else if (c == 'Z') val = p.GetDouble("z_mm");
        else return CommandResult::Fail(std::string("axes: '") + c +
                                        "' is not one of X, Y, Z");
        bool known = false;
        for (const std::string& a : have)
            if (a == axis) { known = true; break; }
        if (!known)
            return CommandResult::Fail("device '" + device + "' has no axis " + axis);
        for (const auto& t : targets)
            if (t.first == axis)
                return CommandResult::Fail("axes names " + axis + " twice");
        targets.emplace_back(axis, val);
    }

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;
    return MoveWithApproach(ctx, m, idx, targets, p, "the axis target");
}

// Move to a saved position PLUS per-axis offsets (e.g. "safe" with Z -5 mm).
// Offsets apply only to axes the position defines — a nonzero offset on an
// undefined axis is an authoring error, not a silent skip.
CommandResult ExecMoveToPositionOffset(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const SavedPos* pos = ctx.FindPosition(device, p.GetString("position"));
    if (!pos) return CommandResult::Fail("position uid not found");
    auto targets = TargetsOf(*pos, m);
    if (targets.empty()) return CommandResult::Fail("position shares no axes with device");

    const std::pair<const char*, double> offsets[] = {
        {"X", p.GetDouble("offset_x_mm")},
        {"Y", p.GetDouble("offset_y_mm")},
        {"Z", p.GetDouble("offset_z_mm")},
    };
    for (const auto& [axis, off] : offsets) {
        if (off == 0.0) continue;
        bool applied = false;
        for (auto& [a, tgt] : targets) {
            if (a == axis) {
                tgt += off;
                applied = true;
                break;
            }
        }
        if (!applied)
            return CommandResult::Fail(std::string("offset on axis ") + axis +
                                       " but the position does not define it");
    }

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;
    const CommandResult mv = MoveWithApproach(ctx, m, idx, targets, p,
                                              "'" + std::string(pos->name) + "' + offset");
    if (!mv.ok || !p.GetBool(kTeachParamName)) return mv;
    // The operator stands at base + offset; the BASE is what gets saved.
    std::map<std::string, double> sub;
    for (const auto& [axis, off] : offsets)
        if (off != 0.0) sub[axis] = off;
    return TeachAfterMove(ctx, m, device, *pos, sub);
}

// Kept because recipes on disk name it, and because a REQUIRED approach velocity
// still says something a set of optional parameters does not: this step exists to
// arrive slowly. Otherwise it is now exactly move_to_position — same body, same
// helper, no second copy of the two-leg logic to drift.
CommandResult ExecMoveToPositionApproach(ExecutionContext& ctx, const ParamValues& p)
{
    return ExecMoveToPosition(ctx, p);
}

CommandResult ExecStopMotion(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    int idx = -1;
    if (!ResolveMotion(ctx, p.GetString("device"), &idx, &err))
        return CommandResult::Fail(err);
    return ctx.Wf().StopMotion(static_cast<std::size_t>(idx))
               ? CommandResult::Success()
               : CommandResult::Fail("stop rejected");
}

// ---- Pneumatic slides -------------------------------------------------------

CommandResult ExecSlideMove(ExecutionContext& ctx, const ParamValues& p, bool extend)
{
    const std::string name = p.GetString("slide");
    const int idx = ctx.SlideIndexByName(name);
    if (idx < 0) return CommandResult::Fail("unknown slide '" + name + "'");
    PneumaticSlide* s = ctx.Hw().Slide(static_cast<std::size_t>(idx));
    if (!s->IsIoConnected()) return CommandResult::Fail("slide IO not connected");

    const bool issued = extend ? ctx.Wf().ExtendSlide(static_cast<std::size_t>(idx))
                               : ctx.Wf().RetractSlide(static_cast<std::size_t>(idx));
    if (!issued) return CommandResult::Fail(extend ? "extend rejected" : "retract rejected");

    // Workflow ticks the slide state machine every frame; we watch its status.
    // timeout_ms 0 = the slide's own configured timeout + margin.
    int timeout = p.GetInt("timeout_ms");
    if (timeout <= 0) timeout = static_cast<int>(s->TimeoutMs()) + 2000;
    const SlidePosition want = extend ? SlidePosition::Extended : SlidePosition::Retracted;
    bool hwFail = false;
    const auto r = ctx.WaitUntil(
        [&] {
            const PneumaticSlide::Status st = s->GetStatus();
            if (st.position == SlidePosition::HardwareError ||
                st.position == SlidePosition::Timeout) {
                hwFail = true;
                return true;   // stop waiting — report below
            }
            return st.position == want;
        },
        timeout, 50);
    if (r == ExecutionContext::WaitResult::Stopped) {
        s->Stop();   // output OFF — air-return safe state
        return CommandResult::Fail("stopped");
    }
    if (hwFail) return CommandResult::Fail("slide reported " +
                                           std::string(SlidePositionName(s->Position())));
    if (r == ExecutionContext::WaitResult::Timeout)
        return CommandResult::Fail("timeout waiting for slide");
    return CommandResult::Success();
}

} // namespace

void RegisterMotionCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;
    (void)hw;   // captured through ExecutionContext at run time
    using PT = ParamType;

    reg.Register({"set_velocity", "Motion",
                  "Set the device move velocity (persists on the device until changed)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"velocity_mm_s", PT::Double, true, {}, 0.001, 100.0, {},
                       "velocity in mm/s"},
                  },
                  CommandSpec::Needs::None,
                  [](ExecutionContext& ctx, const ParamValues& p) {
                      int idx = -1;
                      std::string err;
                      if (!ResolveMotion(ctx, p.GetString("device"), &idx, &err))
                          return CommandResult::Fail(err);
                      if (!ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx),
                                                   p.GetDouble("velocity_mm_s")))
                          return CommandResult::Fail("velocity change rejected");
                      return CommandResult::Success();
                  }});

    reg.Register({"save_position", "Motion",
                  "Save the device's CURRENT position into a named saved position "
                  "(updates by name keeping its uid; creates it if new)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"name", PT::String, true, {}, 0, 0, {},
                       "saved-position name to update or create"},
                  },
                  CommandSpec::Needs::None,
                  [](ExecutionContext& ctx, const ParamValues& p) {
                      int idx = -1;
                      std::string err;
                      IMotionController* m =
                          ResolveMotion(ctx, p.GetString("device"), &idx, &err);
                      if (!m) return CommandResult::Fail(err);
                      // The poll cache can be a second stale at the idle rate:
                      // demand fast polling and wait out one fast cycle so the
                      // taught values are the live post-move position.
                      m->RequestFastPoll();
                      if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
                      const std::map<std::string, double> pos = m->Positions();
                      if (pos.empty())
                          return CommandResult::Fail("no live positions for '" +
                                                     p.GetString("device") + "'");
                      ctx.RecordPosition(p.GetString("device"), p.GetString("name"), pos);
                      return CommandResult::Success();
                  }});

    reg.Register({"read_position", "Motion",
                  "Read the device's CURRENT pose into ctx as <prefix>_x/_y/_z "
                  "(every axis it reports, lowercase)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"ctx_prefix", PT::String, false, "pos", 0, 0, {},
                       "ctx prefix for the axis values"},
                  },
                  CommandSpec::Needs::None, ExecReadPosition});

    reg.Register({"move_axis_to", "Motion",
                  "Coordinated absolute move to explicit axis coordinates; axes not "
                  "named in 'axes' are left where they are",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"axes", PT::String, true, {}, 0, 0, {},
                       "which axes to command, e.g. \"XYZ\", \"XY\", \"Z\" - required, "
                       "so a Z-only move can never also command X and Y to zero"},
                      {"x_mm", PT::Double, false, 0.0, 0, 0, {}, "X target (used when axes names X)"},
                      {"y_mm", PT::Double, false, 0.0, 0, 0, {}, "Y target (used when axes names Y)"},
                      {"z_mm", PT::Double, false, 0.0, 0, 0, {}, "Z target (used when axes names Z)"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveAxisTo});

    reg.Register({"move_to_position", "Motion",
                  "Coordinated absolute move to a saved position, wait for arrival "
                  "(optionally with a slower final approach)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"position", PT::PositionUid, true, {}, 0, 0, {}, "saved position uid"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                      TeachParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToPosition});

    reg.Register({"move_to_position_offset", "Motion",
                  "Move to a saved position PLUS per-axis offsets (e.g. 'safe' with Z -5 mm)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"position", PT::PositionUid, true, {}, 0, 0, {}, "saved position uid"},
                      {"offset_x_mm", PT::Double, false, 0.0, 0, 0, {}, "X offset from the position"},
                      {"offset_y_mm", PT::Double, false, 0.0, 0, 0, {}, "Y offset from the position"},
                      {"offset_z_mm", PT::Double, false, 0.0, 0, 0, {}, "Z offset from the position"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                      TeachParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToPositionOffset});

    // Now a spelling of move_to_position with the approach velocity REQUIRED.
    // Kept registered because recipes on disk name it, and because a step that
    // must arrive slowly is worth saying out loud in the step list.
    reg.Register({"move_to_position_approach", "Motion",
                  "Move to a saved position with a slower final approach (last N mm or % of distance)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"position", PT::PositionUid, true, {}, 0, 0, {}, "saved position uid"},
                      ApproachVelocityParam(true),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "per-leg timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "final tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                      TeachParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToPositionApproach});

    reg.Register({"move_path", "Motion",
                  "Move along a mapped path; the device must already be AT 'from'",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"map", PT::MapName, true, {}, 0, 0, {}, "motion map name"},
                      {"from", PT::PositionUid, true, {}, 0, 0, {}, "start node (must be here)"},
                      {"to", PT::PositionUid, true, {}, 0, 0, {}, "destination node"},
                      {"start_tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 5.0, {},
                       "max distance from 'from' to accept the start"},
                      {"leg_timeout_ms", PT::Int, false, 120000, 100, 600000, {}, "per-leg timeout"},
                      VelocityParam(),
                      ExposureParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                      TeachParam(),
                  },
                  CommandSpec::Needs::None, ExecMovePath});

    reg.Register({"move_relative", "Motion",
                  "Relative move on ONE local axis (target = current + delta), wait for "
                  "arrival (optionally with a slower final approach)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"axis", PT::String, true, {}, 0, 0, {}, "local axis name (X, Y, Z, U...)"},
                      {"delta_mm", PT::Double, true, {}, 0, 0, {}, "signed delta (mm or deg)"},
                      {"timeout_ms", PT::Int, false, 30000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveRelative});

    reg.Register({"move_relative_global", "Motion",
                  "Relative move along GLOBAL axes (via the device transform), wait for settle",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"dx_mm", PT::Double, false, 0.0, 0, 0, {}, "global X delta (mm)"},
                      {"dy_mm", PT::Double, false, 0.0, 0, 0, {}, "global Y delta (mm)"},
                      {"dz_mm", PT::Double, false, 0.0, 0, 0, {}, "global Z delta (mm)"},
                      {"timeout_ms", PT::Int, false, 30000, 100, 600000, {}, "settle timeout"},
                      VelocityParam(),
                      ExposureParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveRelativeGlobal});

    reg.Register({"stop_motion", "Motion", "Stop all motion on one device",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                  },
                  CommandSpec::Needs::None, ExecStopMotion});

    reg.Register({"extend_slide", "Pneumatic",
                  "Extend a pneumatic slide and wait for the sensor",
                  {
                      {"slide", PT::Slide, true, {}, 0, 0, {}, "slide name"},
                      {"timeout_ms", PT::Int, false, 0, 0, 0, {}, "0 = slide's configured timeout"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return ExecSlideMove(c, p, true);
                  }});

    reg.Register({"retract_slide", "Pneumatic",
                  "Retract a pneumatic slide and wait for the sensor",
                  {
                      {"slide", PT::Slide, true, {}, 0, 0, {}, "slide name"},
                      {"timeout_ms", PT::Int, false, 0, 0, 0, {}, "0 = slide's configured timeout"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return ExecSlideMove(c, p, false);
                  }});
}

} // namespace firedog
