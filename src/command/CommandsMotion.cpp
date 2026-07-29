// CommandsMotion.cpp : motion + pneumatic-slide commands.
//
// All of these run on the RecipeEngine worker thread. Blocking moves issue the
// Workflow command, then wait for arrival by polling the driver's cached
// positions through ExecutionContext::WaitUntil — so a stop request interrupts
// within one poll interval, and a timeout stops the device (defined state).
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <cmath>
#include <cstdio>

namespace firedog {

namespace {

constexpr double kDefaultTolMM = 0.01;   // same arrival rule as Workflow::TickPath

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

// Position uid -> targets restricted to the device's axes.
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
    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("move to '" + std::string(pos->name) + "' rejected");

    const auto r = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                              p.GetInt("timeout_ms"));
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));   // defined state on any abort
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped"
                                   : "timeout driving to '" + std::string(pos->name) + "'");
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
    return CommandResult::Success();
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
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
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
    const std::vector<std::pair<std::string, double>> targets = {
        {axis, it->second + delta}};
    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("relative move rejected");
    const auto r = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                              p.GetInt("timeout_ms"));
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped" : "timeout on relative move");
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
    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("move to '" + std::string(pos->name) + "' + offset rejected");
    const auto r = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                              p.GetInt("timeout_ms"));
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped"
                                   : "timeout driving to '" + std::string(pos->name) + "' + offset");
}

// Two-speed move to a saved position: travel at the (optional) main velocity
// to an intermediate point `approach` short of the target, then finish at the
// slower approach velocity. Approach distance = approach_mm, or approach_pct
// of the full distance (approach_mm wins when both are set).
CommandResult ExecMoveToPositionApproach(ExecutionContext& ctx, const ParamValues& p)
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

    const double approachMm  = p.GetDouble("approach_mm");
    const double approachPct = p.GetDouble("approach_pct");
    double approach = approachMm > 0.0 ? approachMm : dist * approachPct / 100.0;
    if (approach <= 0.0)
        return CommandResult::Fail("set approach_mm or approach_pct (> 0)");

    const CommandResult pre = ApplyPreMove(ctx, idx, p);   // travel velocity + exposure
    if (!pre.ok) return pre;

    const double tol       = p.GetDouble("tol_mm");
    const int    timeoutMs = p.GetInt("timeout_ms");

    // Fast leg to the intermediate point (skipped when already within the
    // approach zone). Intermediate arrival check is deliberately looser —
    // its exact spot doesn't matter, only that we're near it and slow.
    if (approach < dist) {
        const double frac = (dist - approach) / dist;
        std::vector<std::pair<std::string, double>> inter;
        inter.reserve(targets.size());
        for (const auto& [axis, tgt] : targets) {
            const double c = cur.at(axis);
            inter.emplace_back(axis, c + (tgt - c) * frac);
        }
        if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), inter))
            return CommandResult::Fail("approach travel leg rejected");
        const auto r = WaitArrive(ctx, m, inter, tol > 0.05 ? tol : 0.05, timeoutMs);
        if (r != ExecutionContext::WaitResult::Ok) {
            ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
            return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                           ? "stopped" : "timeout on travel leg");
        }
    }

    if (!ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx),
                                 p.GetDouble("approach_velocity_mm_s")))
        return CommandResult::Fail("approach velocity change rejected");
    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("approach leg rejected");
    const auto r = WaitArrive(ctx, m, targets, tol, timeoutMs);
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped"
                                   : "timeout driving to '" + std::string(pos->name) + "'");
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

    reg.Register({"move_to_position", "Motion",
                  "Coordinated absolute move to a saved position, wait for arrival",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"position", PT::PositionUid, true, {}, 0, 0, {}, "saved position uid"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
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
                  },
                  CommandSpec::Needs::None, ExecMoveToPositionOffset});

    reg.Register({"move_to_position_approach", "Motion",
                  "Move to a saved position with a slower final approach (last N mm or % of distance)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"position", PT::PositionUid, true, {}, 0, 0, {}, "saved position uid"},
                      {"approach_velocity_mm_s", PT::Double, true, {}, 0.001, 100.0, {},
                       "velocity for the final approach leg"},
                      {"approach_mm", PT::Double, false, 0.0, 0, 0, {},
                       "approach zone as mm before the target (wins over approach_pct)"},
                      {"approach_pct", PT::Double, false, 0.0, 0.0, 100.0, {},
                       "approach zone as % of the total distance"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "per-leg timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "final tolerance"},
                      VelocityParam(),
                      ExposureParam(),
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
                  },
                  CommandSpec::Needs::None, ExecMovePath});

    reg.Register({"move_relative", "Motion",
                  "Relative move on ONE local axis (target = current + delta), wait for arrival",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "motion device name"},
                      {"axis", PT::String, true, {}, 0, 0, {}, "local axis name (X, Y, Z, U...)"},
                      {"delta_mm", PT::Double, true, {}, 0, 0, {}, "signed delta (mm or deg)"},
                      {"timeout_ms", PT::Int, false, 30000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kDefaultTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
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
                  },
                  CommandSpec::Needs::None,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return ExecSlideMove(c, p, false);
                  }});
}

} // namespace firedog
