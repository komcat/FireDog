// CommandsDatum.cpp : datum_reference / datum_clear - the alignment frame as
// ONE recipe step that owns its two or three detection points.
//
// Each point is the actual detect_object, run through RunRegistered with the
// step's model<i> / view_point<i> and its shared run options - so there is no
// second copy of drive / light / detect / manual fallback, and each point is
// also an object result (<name>_E<i>) like any detection. Then the frame:
// SolveAlignFrame for two points, FitAlignFrame3 (a rigid least-squares fit
// onto a TAUGHT reference shape) for three; the guards; and the datum into
// Workflow's store, where move_local {datum} reads it. Nothing is persisted.
//
// docs/datum-reference-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"   // ResolveMotion, VelocityParam
#include "core/AlignFrame.h"
#include "core/Datum.h"
#include "core/DutSession.h"       // withDut: was a DUT open at the solve
#include "core/Log.h"
#include "core/Product.h"          // NowStamp
#include "hardware/motion/IMotionController.h"
#include "workflow/Workflow.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <variant>

namespace firedog {

namespace {

using PT = ParamType;

std::string PointPrefix(const std::string& name, int i)
{
    return name + "_E" + std::to_string(i);
}

bool CtxDouble(ExecutionContext& ctx, const std::string& key, double* out)
{
    const std::optional<CtxValue> v = ctx.CtxGet(key);
    if (!v || !std::holds_alternative<double>(*v)) return false;
    *out = std::get<double>(*v);
    return true;
}

// The signed difference a - b folded into (-180, 180].
double WrapDeg(double d)
{
    while (d > 180.0)   d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}

// "" when point i has its model and its view; else why not, in the datum's own
// words. Checked for EVERY point before the first moves: a 3-point step with
// an untaught E3 must not drive to E1 and E2 first.
std::string PointUntaught(const ParamValues& p, int i)
{
    const std::string tag = "E" + std::to_string(i);
    if (p.GetString(("model" + std::to_string(i)).c_str()).empty())
        return tag + " has no detection model - create and teach it in the step's Vision "
                     "detect helper";
    if (!p.Has(("view_point" + std::to_string(i)).c_str()))
        return tag + ": the camera view is not taught - Teach here on " + tag +
               " in the helper";
    return std::string();
}

// One point: detect_object itself, then the AlignPoint the frame is built from.
// The pose is read AFTER the detection - the camera has not moved since (no
// centring is asked for), and the Z it stands at is the focus height.
CommandResult MeasurePoint(ExecutionContext& ctx, const CommandRegistry& reg,
                           const ParamValues& p, int i, AlignPoint* out)
{
    const std::string name   = p.GetString("name");
    const std::string vpKey  = "view_point" + std::to_string(i);
    const std::string mdKey  = "model" + std::to_string(i);
    const std::string prefix = PointPrefix(name, i);
    char tag[8];
    std::snprintf(tag, sizeof(tag), "E%d", i);

    nlohmann::json args = {
        {"model", p.GetString(mdKey.c_str())},
        {"view_point", p.Raw().at(vpKey)},
        {"take_x", true},
        {"take_y", true},
        {"take_p", false},
        {"require", true},
        {"center_after", false},
        {"ctx_prefix", prefix},
    };
    // The run options the step shares with detect_object, handed on unchanged.
    for (const char* k : {"camera_device", "velocity_mm_s", "z_lift", "safe_z_mm", "settle_ms",
                          "manual_fallback", "apply_lighting", "fresh_frames",
                          "frame_timeout_ms"})
        if (p.Has(k)) args[k] = p.Raw().at(k);

    FD_INFO("datum", "'%s': measuring %s", name.c_str(), tag);
    const CommandResult r = RunRegistered(ctx, reg, "detect_object", args);
    if (!r.ok) return CommandResult::Fail(std::string(tag) + ": " + r.error);

    ObjectResult res;
    if (!ctx.Wf().ObjectResultFor(prefix, &res) || !res.hasX || !res.hasY)
        return CommandResult::Fail(std::string(tag) + ": detect_object left no X/Y result");

    std::string err;
    int idx = -1;
    IMotionController* cam = ResolveMotion(ctx, p.GetString("camera_device"), &idx, &err);
    if (!cam) return CommandResult::Fail(err);
    const std::map<std::string, double> pose = cam->Positions();

    AlignPoint e;
    e.wx = res.x;
    e.wy = res.y;
    if (const auto it = pose.find("X"); it != pose.end()) e.gx = it->second;
    if (const auto it = pose.find("Y"); it != pose.end()) e.gy = it->second;
    if (const auto it = pose.find("Z"); it != pose.end()) { e.gz = it->second; e.hasZ = true; }
    CtxDouble(ctx, prefix + "_x_px", &e.px);
    CtxDouble(ctx, prefix + "_y_px", &e.py);
    *out = e;
    return CommandResult::Success();
}

CommandResult ExecDatumReference(ExecutionContext& ctx, const ParamValues& p,
                                 const CommandRegistry& reg)
{
    const std::string name = p.GetString("name");
    if (name.empty()) return CommandResult::Fail("datum name is empty");
    const int n = p.GetString("mode") == "3pt" ? 3 : 2;

    // From here on it is this run's datum or nothing: a miss, a refusal, a
    // failed guard or a Stop must not leave the previous part's frame for a
    // later move_local.
    ctx.CtxSet(name + "_ok", false);
    ctx.Wf().EraseDatum(name);

    // 3-point needs its reference shape - but it is checked AFTER measuring,
    // so a step without one still leaves the measurement to adopt.
    const double refSpan = p.GetDouble("ref_span_mm");
    const double refE3x  = p.GetDouble("ref_e3_lx");
    const double refE3y  = p.GetDouble("ref_e3_ly");
    const bool   haveRef3 = refSpan > 0.0 && (refE3x != 0.0 || refE3y != 0.0);

    for (int i = 1; i <= n; ++i)
        if (const std::string why = PointUntaught(p, i); !why.empty())
            return CommandResult::Fail("datum '" + name + "': " + why);

    // ---- 1 the points ----------------------------------------------------------
    // Each found point goes to the progress slot at once, so the Aux view shows
    // E1 while E2 is still being looked for. Erased on EVERY exit by the guard:
    // a half-measured datum must not sit on the picture after the step.
    DatumProgress prog;
    prog.name   = name;
    prog.device = p.GetString("camera_device");
    prog.points = n;
    struct ProgressGuard {
        Workflow&   wf;
        std::string name;
        ~ProgressGuard() { wf.EraseDatumProgress(name); }
    } progressGuard{ctx.Wf(), name};
    ctx.Wf().SetDatumProgress(prog);

    AlignPoint pts[3];
    for (int i = 1; i <= n; ++i) {
        const CommandResult r = MeasurePoint(ctx, reg, p, i, &pts[i - 1]);
        if (!r.ok) return r;
        prog.pts[i - 1] = pts[i - 1];
        prog.have       = i;
        ctx.Wf().SetDatumProgress(prog);
    }

    // ---- the raw measurement, for the helper's Use as reference -----------------
    // Published before the solve and the guards: a 3-point step with no
    // reference fails below, and this is what its engineer adopts.
    DatumMeasure meas;
    meas.name   = name;
    meas.points = n;
    meas.at     = NowStamp();
    {
        const double dx = pts[1].wx - pts[0].wx, dy = pts[1].wy - pts[0].wy;
        meas.spanMm = std::sqrt(dx * dx + dy * dy);
        AlignResult two;
        if (n == 3 && SolveAlignFrame(pts[0], pts[1], &two, nullptr)) {
            AlignMachineToLocal(two, pts[2].wx, pts[2].wy, &meas.e3lx, &meas.e3ly);
            meas.hasE3 = true;
        }
    }
    ctx.Wf().SetDatumMeasure(meas);
    ctx.CtxSet(name + "_span_mm", meas.spanMm);
    if (meas.hasE3) {
        ctx.CtxSet(name + "_e3_lx", meas.e3lx);
        ctx.CtxSet(name + "_e3_ly", meas.e3ly);
    }

    // ---- 2 the frame ------------------------------------------------------------
    Datum d;
    d.name   = name;
    d.device = p.GetString("camera_device");
    d.points = n;
    std::string err;
    if (n == 2) {
        if (!SolveAlignFrame(pts[0], pts[1], &d.frame, &err))
            return CommandResult::Fail("datum '" + name + "': " + err);
    } else {
        // Never a quiet 2-point frame from a 3-point step: a step that says
        // three points and fits two is the "claims automatic, isn't" failure.
        if (!haveRef3)
            return CommandResult::Fail("datum '" + name + "': 3-point mode has no reference "
                                       "shape - this run's measurement is ready: open the "
                                       "helper and press Use as reference");
        if (!FitAlignFrame3(pts[0], pts[1], pts[2], refSpan, refE3x, refE3y, &d.frame,
                            d.resid, &err))
            return CommandResult::Fail("datum '" + name + "': " + err);
        d.e3       = pts[2];
        d.maxResid = std::max({d.resid[0], d.resid[1], d.resid[2]});
    }

    // ---- the guards: refuse rather than publish a frame off the wrong feature ---
    // Unlike an alignment station nobody looked at these marks before the
    // solve, so an out-of-band datum FAILS and publishes nothing.
    const double spanTol = p.GetDouble("span_tol_mm");
    if (spanTol > 0.0) {
        if (refSpan <= 0.0)
            return CommandResult::Fail("datum '" + name + "': span_tol_mm is set but there is "
                                       "no reference span - Use as reference in the helper");
        const double e = d.frame.spanMm - refSpan;
        if (std::fabs(e) > spanTol) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          "datum '%s': E1-E2 span %.4f mm is %+.4f from the reference %.4f "
                          "(tolerance %.4f) - a point is on the wrong feature?",
                          name.c_str(), d.frame.spanMm, e, refSpan, spanTol);
            return CommandResult::Fail(buf);
        }
    }
    const double thTol = p.GetDouble("theta_tol_deg");
    if (thTol > 0.0) {
        const double e = WrapDeg(d.frame.thetaDeg - p.GetDouble("theta_ref_deg"));
        if (std::fabs(e) > thTol) {
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                          "datum '%s': theta %.4f deg is %+.4f from %.4f (tolerance %.4f) - "
                          "the part is turned or loaded backwards?",
                          name.c_str(), d.frame.thetaDeg, e, p.GetDouble("theta_ref_deg"),
                          thTol);
            return CommandResult::Fail(buf);
        }
    }
    const double resTol = p.GetDouble("resid_tol_mm");
    if (n == 3 && resTol > 0.0 && d.maxResid > resTol) {
        char buf[220];
        std::snprintf(buf, sizeof(buf),
                      "datum '%s': fit residuals E1 %.4f  E2 %.4f  E3 %.4f mm - the worst "
                      "exceeds %.4f (a point is on the wrong feature?)",
                      name.c_str(), d.resid[0], d.resid[1], d.resid[2], resTol);
        return CommandResult::Fail(buf);
    }

    // ---- 3 publish ----------------------------------------------------------------
    d.withDut = DutSession::Instance().GetInfo().open;
    d.at      = NowStamp();
    d.madeAt  = std::chrono::steady_clock::now();
    ctx.Wf().SetDatum(d);

    ctx.CtxSet(name + "_origin_x", d.frame.originX);
    ctx.CtxSet(name + "_origin_y", d.frame.originY);
    ctx.CtxSet(name + "_theta_deg", d.frame.thetaDeg);
    if (n == 3) ctx.CtxSet(name + "_max_resid_mm", d.maxResid);
    ctx.CtxSet(name + "_ok", true);

    if (n == 3)
        FD_INFO("datum", "'%s' (3 pt) on %s: origin (%.4f, %.4f) mm, theta %.4f deg, span "
                         "%.4f mm, residuals %.4f / %.4f / %.4f mm - good for %d min%s",
                name.c_str(), d.device.c_str(), d.frame.originX, d.frame.originY,
                d.frame.thetaDeg, d.frame.spanMm, d.resid[0], d.resid[1], d.resid[2],
                kDatumLifeMin, d.withDut ? " or until the DUT closes"
                                         : " or until the next program start (no DUT open)");
    else
        FD_INFO("datum", "'%s' (2 pt) on %s: origin (%.4f, %.4f) mm, theta %.4f deg, span "
                         "%.4f mm - good for %d min%s",
                name.c_str(), d.device.c_str(), d.frame.originX, d.frame.originY,
                d.frame.thetaDeg, d.frame.spanMm, kDatumLifeMin,
                d.withDut ? " or until the DUT closes"
                          : " or until the next program start (no DUT open)");
    return CommandResult::Success();
}

// ---- inspect_object --------------------------------------------------------
//
// Where an object sits on the PART: detect_object itself (the step's own model
// and view, through RunRegistered - no second copy of drive / light / detect /
// fallback), then its X / Y through the named datum into local coordinates, and
// the criteria: local offset from the nominal against per-axis and radial
// tolerances. The datum is checked before anything moves (live, and measured on
// the camera's own device - another device's axis space would turn a correct
// measurement into a wrong number). docs/datum-reference-design.md §5a.
CommandResult ExecInspectObject(ExecutionContext& ctx, const ParamValues& p,
                                const CommandRegistry& reg)
{
    const std::string prefix = p.GetString("ctx_prefix");
    // Written FIRST: every failing path below leaves a recipe branching on
    // <prefix>_pass reading false, never the previous part's verdict.
    ctx.CtxSet(prefix + "_pass", false);
    ctx.CtxSet(prefix + "_why", std::string("not inspected"));

    // An inspection that cannot pass is a VERDICT, not a broken step: by
    // default it is reported (<prefix>_pass = false, _why) and the program goes
    // on - the part failed, the machine did not. fail_on_reject opts into
    // failing the step instead. A Stop, a refused move or a camera error is
    // NOT a verdict and still fails (detect_object's own result, below).
    const auto reject = [&](const std::string& why) {
        ctx.CtxSet(prefix + "_why", why);
        FD_INFO("datum", "inspect '%s': FAIL - %s", prefix.c_str(), why.c_str());
        return p.GetBool("fail_on_reject") ? CommandResult::Fail("inspect '" + prefix + "': " + why)
                                           : CommandResult::Success();
    };

    const std::string dn  = p.GetString("datum");
    const std::string cam = p.GetString("camera_device");
    if (dn.empty()) return reject("no datum named");
    Datum d;
    std::string err;
    if (!ctx.Wf().DatumFor(dn, &d, &err)) return reject(err);
    if (d.device != cam)
        return reject("datum '" + dn + "' was measured on '" + d.device +
                      "', the camera here rides on '" + cam + "' - another axis space");

    nlohmann::json args = {
        {"model", p.GetString("model")},
        {"take_x", true},
        {"take_y", true},
        {"take_p", p.GetBool("take_p")},
        // A miss is judged here (a verdict), so detect_object never fails on
        // one; `require` decides what this step does with it.
        {"require", false},
        {"center_after", false},
        {"ctx_prefix", prefix},
    };
    // The detection options the step shares with detect_object, handed on.
    for (const char* k : {"view_point", "use_current_position", "camera_device",
                          "velocity_mm_s", "z_lift", "safe_z_mm", "settle_ms",
                          "manual_fallback", "apply_lighting", "fresh_frames",
                          "frame_timeout_ms"})
        if (p.Has(k)) args[k] = p.Raw().at(k);

    const CommandResult r = RunRegistered(ctx, reg, "detect_object", args);
    if (!r.ok) return CommandResult::Fail("inspect '" + prefix + "': " + r.error);

    ObjectResult res;
    if (!ctx.Wf().ObjectResultFor(prefix, &res) || !res.hasX || !res.hasY) {
        // Nothing found: detect_object said so in <prefix>_ok / _error.
        // Nothing to judge - and not a pass.
        ctx.CtxSet(prefix + "_why", std::string("object not found"));
        FD_INFO("datum", "inspect '%s': FAIL - object not found", prefix.c_str());
        return p.GetBool("require")
                   ? CommandResult::Fail("inspect '" + prefix + "': object not found")
                   : CommandResult::Success();
    }
    // The detection moved and waited: the datum may have expired meanwhile.
    if (!ctx.Wf().DatumFor(dn, &d, &err)) return reject(err);

    double lx = 0.0, ly = 0.0;
    AlignMachineToLocal(d.frame, res.x, res.y, &lx, &ly);
    const double dlx = lx - p.GetDouble("nominal_lx");
    const double dly = ly - p.GetDouble("nominal_ly");
    const double dr  = std::sqrt(dlx * dlx + dly * dly);
    ctx.CtxSet(prefix + "_lx", lx);
    ctx.CtxSet(prefix + "_ly", ly);
    ctx.CtxSet(prefix + "_dlx", dlx);
    ctx.CtxSet(prefix + "_dly", dly);
    ctx.CtxSet(prefix + "_dr", dr);

    // ---- the criteria: every tolerance set (> 0) must hold ---------------------
    const double tx = p.GetDouble("tol_x_mm"), ty = p.GetDouble("tol_y_mm"),
                 tr = p.GetDouble("tol_r_mm");
    std::string why;
    const auto check = [&](double tol, double v, const char* what) {
        if (tol <= 0.0 || std::fabs(v) <= tol) return;
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s%s %+.4f outside +/-%.4f", why.empty() ? "" : "; ",
                      what, v, tol);
        why += buf;
    };
    check(tx, dlx, "dX");
    check(ty, dly, "dY");
    check(tr, dr, "dR");
    const bool judged = tx > 0.0 || ty > 0.0 || tr > 0.0;
    // No tolerance set = report only, and NOT a pass: a default pass is how an
    // unchecked inspection ends up trusted.
    const bool pass = judged && why.empty();
    if (!judged) why = "no tolerance set - report only";
    ctx.CtxSet(prefix + "_pass", pass);
    ctx.CtxSet(prefix + "_why", pass ? std::string() : why);

    FD_INFO("datum", "inspect '%s' in datum '%s': local (%.4f, %.4f) mm, nominal (%.4f, %.4f), "
                     "d (%+.4f, %+.4f) r %.4f - %s%s%s",
            prefix.c_str(), dn.c_str(), lx, ly, p.GetDouble("nominal_lx"),
            p.GetDouble("nominal_ly"), dlx, dly, dr,
            pass ? "PASS" : (judged ? "REJECT" : "not judged"), pass ? "" : ": ",
            pass ? "" : why.c_str());

    if (judged && !pass && p.GetBool("fail_on_reject"))
        return CommandResult::Fail("inspect '" + prefix + "': " + why);
    return CommandResult::Success();
}

CommandResult ExecDatumClear(ExecutionContext& ctx, const ParamValues& p)
{
    if (p.GetBool("all")) {
        ctx.Wf().ClearDatums("datum_clear all");
        return CommandResult::Success();
    }
    const std::string name = p.GetString("name");
    if (ctx.Wf().EraseDatum(name))
        FD_INFO("datum", "datum '%s' cleared", name.c_str());
    else
        FD_INFO("datum", "datum_clear: no datum '%s' (nothing to do)", name.c_str());
    return CommandResult::Success();
}

} // namespace

void RegisterDatumCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // every action goes through ExecutionContext::Wf() at run time
    (void)hw;
    CommandRegistry* rp = &reg;
    const char* H = kVisionDetectHelper;

    CommandSpec spec{"datum_reference", "Align",
        "Find 2 or 3 points with this step's own detectors and make a datum: a local "
        "frame (origin E1, X+ toward E2) for move_local - good for 30 min or until the "
        "DUT closes",
        {
            {"name", PT::String, false, "datum1", 0, 0, {},
             "the datum's name - move_local {datum} and datum_clear use it; its points "
             "are also object results <name>_E1 ..."},
            {"mode", PT::Enum, false, "2pt", 0, 0, {"2pt", "3pt"},
             "2pt = origin E1, X+ toward E2; 3pt = a rigid least-squares fit of the taught "
             "reference shape onto E1/E2/E3 (E3 away from the E1-E2 line adds precision)",
             PinDir::Any, H},
            {"model1", PT::String, false, "", 0, 0, {},
             "E1's private detection model (uid) - created and taught in the helper",
             PinDir::Any, H},
            {"view_point1", PT::PointXyz, true, {}, 0, 0, {},
             "the camera pose E1 is detected from (camera device axis space)",
             PinDir::Any, H},
            {"model2", PT::String, false, "", 0, 0, {},
             "E2's private detection model (uid)", PinDir::Any, H},
            {"view_point2", PT::PointXyz, true, {}, 0, 0, {},
             "the camera pose E2 is detected from", PinDir::Any, H},
            {"model3", PT::String, false, "", 0, 0, {},
             "E3's private detection model (uid; 3pt only)", PinDir::Any, H},
            {"view_point3", PT::PointXyz, false, {}, 0, 0, {},
             "the camera pose E3 is detected from (3pt only)", PinDir::Any, H},
            {"ref_span_mm", PT::Double, false, 0.0, 0.0, 10000.0, {},
             "reference |E1-E2| (mm; 0 = none) - Use as reference in the helper",
             PinDir::Any, H},
            {"ref_e3_lx", PT::Double, false, 0.0, 0, 0, {},
             "reference E3, local X (mm) - Use as reference", PinDir::Any, H},
            {"ref_e3_ly", PT::Double, false, 0.0, 0, 0, {},
             "reference E3, local Y (mm) - Use as reference", PinDir::Any, H},
            {"span_tol_mm", PT::Double, false, 0.0, 0.0, 100.0, {},
             "fail when |E1-E2| differs from ref_span_mm by more than this (0 = off)",
             PinDir::Any, H},
            {"theta_ref_deg", PT::Double, false, 0.0, -180.0, 180.0, {},
             "the expected X+ direction, machine deg (with theta_tol_deg)", PinDir::Any, H},
            {"theta_tol_deg", PT::Double, false, 0.0, 0.0, 180.0, {},
             "fail when theta differs from theta_ref_deg by more than this (0 = off)",
             PinDir::Any, H},
            {"resid_tol_mm", PT::Double, false, 0.0, 0.0, 100.0, {},
             "3pt: fail when any point's fit residual exceeds this (0 = off)",
             PinDir::Any, H},
            {"camera_device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
             "the device the camera rides on: the datum is in ITS axis space"},
            {"z_lift", PT::Bool, false, true, 0, 0, {},
             "travel to each view via safe_z_mm (Z first, XY, then Z down)", PinDir::Any, H},
            {"safe_z_mm", PT::Double, false, 25.0, 0, 0, {},
             "the Z travelled at when z_lift is on - a camera-device AXIS value",
             PinDir::Any, H},
            {"settle_ms", PT::Int, false, 100, 0, 10000, {},
             "after arriving at a view, wait this long before the frames (ms)",
             PinDir::Any, H},
            {"manual_fallback", PT::Bool, false, false, 0, 0, {},
             "a point not found: ask the operator for the crosshair instead of failing",
             PinDir::Any, H},
            VelocityParam(),
            {"apply_lighting", PT::Bool, false, true, 0, 0, {},
             "set the light each model was taught under first"},
            {"fresh_frames", PT::Int, false, 2, 0, 10, {},
             "new frames required before each detection (0 = use the latest)"},
            {"frame_timeout_ms", PT::Int, false, 2000, 50, 30000, {},
             "ceiling on the fresh-frame wait; then the latest frame is used"},
            PreDelayParam(),
            PostDelayParam(),
        },
        CommandSpec::Needs::Camera,
        [rp](ExecutionContext& c, const ParamValues& p) {
            return ExecDatumReference(c, p, *rp);
        },
        /*parallelBy*/ "", /*helper*/ H};
    spec.outputs = [](const nlohmann::json& p) {
        const std::string n   = p.value("name", std::string("datum1"));
        const std::string dev = p.value("camera_device", std::string("gantry-main"));
        const bool three = p.value("mode", std::string("2pt")) == "3pt";
        CommandOutputs o;
        // The datum itself - what move_local's datum picker offers.
        o.datums.push_back({n, dev, std::string(three ? "3-point" : "2-point") +
                                        " local frame (origin E1, X+ toward E2), '" + dev +
                                        "' axis mm"});
        for (int i = 1; i <= (three ? 3 : 2); ++i)
            o.results.push_back({PointPrefix(n, i), dev, "XY",
                                 "datum point E" + std::to_string(i) + ", '" + dev +
                                     "' axis mm"});
        o.keys.push_back({n + "_ok", "bool", "the datum was made (and passed its guards)"});
        o.keys.push_back({n + "_origin_x", "mm", "where local (0,0) is, '" + dev + "' X"});
        o.keys.push_back({n + "_origin_y", "mm", "where local (0,0) is, '" + dev + "' Y"});
        o.keys.push_back({n + "_theta_deg", "deg", "the local X+ direction, machine deg"});
        o.keys.push_back({n + "_span_mm", "mm", "|E1-E2| as measured"});
        if (three) {
            o.keys.push_back({n + "_max_resid_mm", "mm", "the worst point's fit residual"});
            o.keys.push_back({n + "_e3_lx", "mm", "E3 measured in the E1-E2 frame, X"});
            o.keys.push_back({n + "_e3_ly", "mm", "E3 measured in the E1-E2 frame, Y"});
        }
        return o;
    };
    reg.Register(std::move(spec));

    // ---- inspect_object: detect_object + the datum's local frame + criteria ----
    // The helper params share detect_object's NAMES, so the Vision detect
    // helper edits them with the same functions; it never centres.
    const char* C = "Criteria";
    CommandSpec insp{"inspect_object", "Vision",
        "Find an object with this step's own detection model and report where it sits in a "
        "datum's local frame; judge it against a nominal and tolerances",
        {
            {"datum", PT::DatumName, false, "", 0, 0, {},
             "the datum (datum_reference) the object is measured against - REQUIRED; it must "
             "be live and measured on camera_device"},
            {"ctx_prefix", PT::String, false, "insp", 0, 0, {},
             "NAMES the object result and prefixes the ctx keys: _lx / _ly (local mm), "
             "_dlx / _dly / _dr (from the nominal), _pass, _why, plus detect_object's own"},
            {"model", PT::String, false, "", 0, 0, {},
             "this step's private detection model (uid) - created and taught in the helper",
             PinDir::Any, H},
            {"take_p", PT::Bool, false, false, 0, 0, {},
             "also report P (rotation from the taught orientation, deg, clockwise positive "
             "on the picture - NOT corrected by the datum's rotation)",
             PinDir::Any, H},
            {"use_current_position", PT::Bool, false, false, 0, 0, {},
             "detect where the camera stands now (no move to view_point)", PinDir::Any, H},
            {"view_point", PT::PointXyz, false, {}, 0, 0, {},
             "the camera pose to detect from (camera device axis space) - Teach here in the "
             "helper; required unless use_current_position",
             PinDir::Any, H},
            {"settle_ms", PT::Int, false, 100, 0, 10000, {},
             "wait this long before the frames are taken (ms)", PinDir::Any, H},
            {"z_lift", PT::Bool, false, true, 0, 0, {},
             "travel to view_point via safe_z_mm (Z first, XY, then Z down)", PinDir::Any, H},
            {"safe_z_mm", PT::Double, false, 25.0, 0, 0, {},
             "the Z travelled at when z_lift is on - a camera-device AXIS value",
             PinDir::Any, H},
            {"manual_fallback", PT::Bool, false, false, 0, 0, {},
             "nothing found: ask the operator for the crosshair instead of failing",
             PinDir::Any, H},
            {"camera_device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
             "the device the camera rides on - must be the datum's"},
            VelocityParam(),
            {"apply_lighting", PT::Bool, false, true, 0, 0, {},
             "set the light the model was taught under first"},
            {"fresh_frames", PT::Int, false, 2, 0, 10, {},
             "new frames required before detecting (0 = use the latest)"},
            {"frame_timeout_ms", PT::Int, false, 2000, 50, 30000, {},
             "ceiling on the fresh-frame wait; then the latest frame is used"},
            // Off: a miss is a failed INSPECTION, reported - not a failed
            // program (2026-11-24).
            {"require", PT::Bool, false, false, 0, 0, {},
             "fail the STEP when nothing is found; off (default) = report it "
             "(<prefix>_ok / _pass = false) and go on"},
            {"nominal_lx", PT::Double, false, 0.0, 0, 0, {},
             "where the object should be, datum-local X (mm) - Use as nominal in the helper",
             PinDir::Any, C},
            {"nominal_ly", PT::Double, false, 0.0, 0, 0, {},
             "where the object should be, datum-local Y (mm)", PinDir::Any, C},
            {"tol_x_mm", PT::Double, false, 0.0, 0.0, 1000.0, {},
             "|local X - nominal| must be within this (0 = not checked)", PinDir::Any, C},
            {"tol_y_mm", PT::Double, false, 0.0, 0.0, 1000.0, {},
             "|local Y - nominal| must be within this (0 = not checked)", PinDir::Any, C},
            {"tol_r_mm", PT::Double, false, 0.0, 0.0, 1000.0, {},
             "the radial distance from the nominal must be within this (0 = not checked)",
             PinDir::Any, C},
            {"fail_on_reject", PT::Bool, false, false, 0, 0, {},
             "a reject (out of tolerance, or no live datum) FAILS the step; off (default) "
             "= report it in <prefix>_pass / _why and go on",
             PinDir::Any, C},
            PreDelayParam(),
            PostDelayParam(),
        },
        CommandSpec::Needs::Camera,
        [rp](ExecutionContext& c, const ParamValues& p) { return ExecInspectObject(c, p, *rp); },
        /*parallelBy*/ "", /*helper*/ H};
    // Rules BETWEEN params, refused at Start: a datum, and a taught view unless
    // the step detects where the camera stands.
    insp.check = [](const nlohmann::json& m) -> std::string {
        if (m.value("datum", std::string()).empty())
            return "no datum - pick the datum_reference this object is measured against";
        if (!m.value("use_current_position", false) &&
            (!m.contains("view_point") || m["view_point"].is_null()))
            return "point 'view_point' not taught - Teach here in the Vision detect helper, "
                   "or tick Use current position";
        return std::string();
    };
    insp.outputs = [](const nlohmann::json& p) {
        const std::string pre = p.value("ctx_prefix", std::string("insp"));
        const std::string dev = p.value("camera_device", std::string("gantry-main"));
        const std::string dn  = p.value("datum", std::string());
        const bool tp = p.value("take_p", false);
        CommandOutputs o;
        o.results.push_back({pre, dev, tp ? "XYP" : "XY",
                             "the inspected object, '" + dev + "' axis mm"});
        o.keys.push_back({pre + "_lx", "mm", "object X in datum '" + dn + "' (local)"});
        o.keys.push_back({pre + "_ly", "mm", "object Y in datum '" + dn + "' (local)"});
        o.keys.push_back({pre + "_dlx", "mm", "local X - nominal_lx"});
        o.keys.push_back({pre + "_dly", "mm", "local Y - nominal_ly"});
        o.keys.push_back({pre + "_dr", "mm", "distance from the nominal"});
        o.keys.push_back({pre + "_pass", "bool",
                          "inside every set tolerance (false when none is set)"});
        o.keys.push_back({pre + "_why", "text", "why not a pass"});
        o.keys.push_back({pre + "_ok", "bool", "the object was found"});
        o.keys.push_back({pre + "_x", "mm", "object X, '" + dev + "' axis mm"});
        o.keys.push_back({pre + "_y", "mm", "object Y, '" + dev + "' axis mm"});
        if (tp) o.keys.push_back({pre + "_p", "deg", "rotation from the taught orientation"});
        o.keys.push_back({pre + "_score", "number", "detection score 0..1"});
        return o;
    };
    reg.Register(std::move(insp));

    reg.Register({"datum_clear", "Align",
                  "Cancel a datum made by datum_reference (or every datum)",
                  {
                      {"name", PT::String, false, "datum1", 0, 0, {}, "the datum to clear"},
                      {"all", PT::Bool, false, false, 0, 0, {},
                       "clear every datum (name is ignored)"},
                  },
                  CommandSpec::Needs::None, ExecDatumClear});
}

} // namespace firedog
