// CommandsAlign.cpp : the Alignment Procedure command family — teach a local
// coordinate system (O1) from two operator-marked features, then use it.
//
// align_station drives to a station, applies its exposure/lighting and gets the
// marked point; align_solve turns the two points into a frame and hands it back
// to the UI thread to persist; align_apply and move_local consume the frame.
// align_run is the typed one-step wrapper over alignment_procedure.lua, the
// same shape as `xcan` — the script IS the procedure, so it stays editable.
//
// Why the pick answer arrives in millimetres: only the UI knows the live
// pixel_to_mm calibration, so it converts and the worker just adds the offset
// to the gantry pose. See docs/alignment-procedure-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"      // RunLuaFile (align_run)
#include "command/MotionWaits.h"    // WaitArrive / WaitSettle / ResolveMotion
#include "core/AlignFrame.h"
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "workflow/Workflow.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

// A station's saved position, restricted to the axes the device actually has.
std::vector<std::pair<std::string, double>> StationTargets(const SavedPos& p,
                                                           const IMotionController* m)
{
    std::vector<std::pair<std::string, double>> targets;
    for (const std::string& a : m->Axes()) {
        const auto it = p.vals.find(a);
        if (it != p.vals.end()) targets.emplace_back(a, it->second);
    }
    return targets;
}

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
    const auto targets = StationTargets(*pos, m);
    if (targets.empty()) return CommandResult::Fail("station position shares no axes with device");

    // Exposure BEFORE the move, so the sensor settles while the axes travel.
    if (st.exposureUs > 0.0 && !ctx.Wf().SetCameraExposure(st.exposureUs))
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

    // Auto mode has no detector behind it yet — say so instead of pretending.
    bool manual = (st.mode != "auto");
    if (!manual) {
        if (!st.autoFallbackManual)
            return CommandResult::Fail(
                "station " + std::to_string(station) +
                " is set to auto, but feature detection is not implemented "
                "(set auto_fallback_manual or switch the station to manual)");
        FD_WARN("align", "'%s' station %d: auto unavailable - asking the operator",
                proc->name.c_str(), station);
        manual = true;
    }

    ExecutionContext::PickResult pick;
    if (manual) {
        ExecutionContext::PickRequest req;
        req.alignName = proc->name;
        req.station   = station;
        req.message   = ctx.Substitute(p.GetString("message"));
        if (req.message.empty()) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Mark E%d on the live image, then OK", station);
            req.message = buf;
        }
        if (!ctx.AwaitPick(req, &pick))
            return CommandResult::Fail("cancelled by operator");
    }

    // The world point: where the gantry is, plus where in the frame the
    // operator marked. Read the pose AFTER the pick — nothing moved, but this
    // is the pose the pixel offset is relative to.
    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> live = m->Positions();
    const auto itx = live.find("X"), ity = live.find("Y");
    if (itx == live.end() || ity == live.end())
        return CommandResult::Fail("device '" + proc->device + "' does not report X/Y");

    AlignPoint e;
    e.gx = itx->second;
    e.gy = ity->second;
    e.px = pick.px;
    e.py = pick.py;
    e.wx = e.gx + pick.dxMM;
    e.wy = e.gy + pick.dyMM;

    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(PointKey(prefix, station, "gx"), e.gx);
    ctx.CtxSet(PointKey(prefix, station, "gy"), e.gy);
    ctx.CtxSet(PointKey(prefix, station, "px"), e.px);
    ctx.CtxSet(PointKey(prefix, station, "py"), e.py);
    ctx.CtxSet(PointKey(prefix, station, "wx"), e.wx);
    ctx.CtxSet(PointKey(prefix, station, "wy"), e.wy);

    FD_INFO("align", "'%s' E%d: gantry (%.4f, %.4f) + pick (%+.4f, %+.4f) = (%.4f, %.4f) mm",
            proc->name.c_str(), station, e.gx, e.gy, pick.dxMM, pick.dyMM, e.wx, e.wy);
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

    if (p.GetBool("save")) {
        // The document is UI-thread state: hand the frame over and let
        // App::Update store and persist it.
        ctx.Wf().QueueAlignResult(proc->uid, r);
    }
    FD_INFO("align", "'%s' solved: origin (%.4f, %.4f) mm, theta %.4f deg, span %.4f mm",
            proc->name.c_str(), r.originX, r.originY, r.thetaDeg, r.spanMm);
    return CommandResult::Success();
}

// ---- align_apply ---------------------------------------------------------

// Resolve the procedure AND require a taught frame — the one check every
// consumer of a frame owes the operator.
const AlignProc* SolvedProc(ExecutionContext& ctx, const std::string& name,
                            std::string* err)
{
    const AlignProc* proc = ctx.FindAlign(name);
    if (!proc) { *err = "alignment procedure not found: '" + name + "'"; return nullptr; }
    if (!proc->result.valid) {
        *err = "alignment '" + proc->name + "' has no taught frame yet - run it first";
        return nullptr;
    }
    return proc;
}

CommandResult ExecAlignApply(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    const AlignProc* proc = SolvedProc(ctx, p.GetString("alignment"), &err);
    if (!proc) return CommandResult::Fail(err);

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(proc->result, p.GetDouble("x_mm"), p.GetDouble("y_mm"), &mx, &my);
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_x", mx);
    ctx.CtxSet(prefix + "_y", my);
    return CommandResult::Success();
}

// ---- move_local ----------------------------------------------------------

CommandResult ExecMoveLocal(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    const AlignProc* proc = SolvedProc(ctx, p.GetString("alignment"), &err);
    if (!proc) return CommandResult::Fail(err);

    const std::string device = p.GetString("device").empty() ? proc->device
                                                             : p.GetString("device");
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(proc->result, p.GetDouble("x_mm"), p.GetDouble("y_mm"), &mx, &my);

    std::vector<std::pair<std::string, double>> targets = {{"X", mx}, {"Y", my}};
    if (p.Has("z_mm")) targets.emplace_back("Z", p.GetDouble("z_mm"));

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("move rejected");
    const auto arrived = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                                    p.GetInt("timeout_ms"));
    if (arrived == ExecutionContext::WaitResult::Ok) return WaitSettle(ctx, m, idx);
    ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
    return CommandResult::Fail(arrived == ExecutionContext::WaitResult::Stopped
                                   ? "stopped"
                                   : "timeout driving to the local coordinate");
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
                      {"station", PT::Int, true, {}, 1, 2, {}, "1 = E1 (origin), 2 = E2 (X+)"},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction shown to the operator ({key} substitutes ctx values)"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "captured point goes to <prefix>_e<N>_gx/gy/px/py/wx/wy"},
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
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveLocal});

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
                       "config/scripts/alignment_procedure.lua", 0, 0, {}, "script override"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("align_name", p.GetString("alignment"));
                      c.CtxSet("align_prefix", p.GetString("ctx_prefix"));
                      const std::string err =
                          RunLuaFile(p.GetString("file"), *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});
}

} // namespace firedog
