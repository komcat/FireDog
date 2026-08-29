// CommandsDeviceReg.cpp : the device-registration command family.
//
// Registering a device means relating ITS axis origin to the camera device's,
// so a gripper tip has a known position at every moment instead of only just
// after somebody looked at it. The needle family solved the same problem inside
// one device; see core/DeviceReg.h for why this one is bigger (a rotation as
// well as an offset, and a handedness that cannot be assumed).
//
//   device_reg             put the registration — or the aiming seed — into ctx
//   device_reg_save        take a freshly measured four-point fit back
//   device_reg_tip_save    re-teach t alone, keeping R (the gripper changed)
//   device_reg_check_save  record what the check pass measured
//   move_to_point          command a reference-space point to a registered device
//   device_reg_run         run the registration procedure as one recipe step
//   device_reg_check_run   run the daily check as one recipe step
//
// As with the needle, the procedures themselves are Lua
// (config/scripts/device_registration.lua, config/scripts/device_reg_check.lua)
// and these commands do almost nothing: read the record, write the record, and
// launch the two scripts. The arithmetic that has to be right — the fit and its
// refusals — is in core/DeviceReg.cpp, because a refusal that only exists in a
// script is a refusal a fork can delete.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"     // RunLuaFile (the two script launchers)
#include "command/MotionWaits.h"   // ResolveMotion, MoveWithApproach, kMoveTolMM
#include "core/DeviceReg.h"
#include "core/Log.h"
#include "core/Product.h"          // NowStamp
#include "hardware/motion/IMotionController.h"
#include "workflow/Workflow.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

using PT = ParamType;

RegVec3 Vec3Param(const ParamValues& p, const char* xk, const char* yk, const char* zk)
{
    return RegVec3{p.GetDouble(xk), p.GetDouble(yk), p.GetDouble(zk)};
}

// ---- device_reg ------------------------------------------------------------
// Reports the registration AND, when there is none, the aiming columns derived
// from config/motion_transforms.json. Both go into ctx under the same keys with
// `_seeded` telling them apart, because the registration script's aiming maths
// is identical either way — what differs is only how much the answer is worth,
// and that is a decision for the script, not a second set of keys.

CommandResult ExecDeviceReg(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string ref    = p.GetString("ref_device");
    const std::string prefix = p.GetString("ctx_prefix");

    const DeviceReg r = ctx.Wf().DeviceRegFor(device);

    RegVec3 cols[3];
    bool    seeded = true;
    if (!ctx.Wf().DeviceRegAimColumns(device, ref, cols, &seeded))
        return CommandResult::Fail("no motion device named '" + device + "' or '" +
                                   ref + "'");

    // Write every key FIRST, valid or not: a script that branches on <prefix>_ok
    // has to be able to read it on the failing path too, and `require` is only a
    // shortcut for that check.
    ctx.CtxSet(prefix + "_ok", r.valid);
    ctx.CtxSet(prefix + "_verified", r.verified);
    ctx.CtxSet(prefix + "_seeded", seeded);
    ctx.CtxSet(prefix + "_ref", r.valid ? r.refDevice : ref);
    ctx.CtxSet(prefix + "_tx", r.t.x);
    ctx.CtxSet(prefix + "_ty", r.t.y);
    ctx.CtxSet(prefix + "_tz", r.t.z);
    for (int j = 0; j < 3; ++j) {
        const std::string c = prefix + "_c" + static_cast<char>('1' + j);
        ctx.CtxSet(c + "x", cols[j].x);
        ctx.CtxSet(c + "y", cols[j].y);
        ctx.CtxSet(c + "z", cols[j].z);
    }
    ctx.CtxSet(prefix + "_jog_mm", r.jogMm);
    ctx.CtxSet(prefix + "_h0x", r.h0.x);
    ctx.CtxSet(prefix + "_h0y", r.h0.y);
    ctx.CtxSet(prefix + "_h0z", r.h0.z);
    ctx.CtxSet(prefix + "_at", r.regAt);
    ctx.CtxSet(prefix + "_tip_at", r.tipAt);
    ctx.CtxSet(prefix + "_checked_at", r.checkedAt);
    ctx.CtxSet(prefix + "_check_mm", r.checkResidualMm);

    if (!r.valid && p.GetBool("require"))
        return CommandResult::Fail(
            "'" + device + "' has no registration - run the device registration "
            "procedure (Registration page, or a device_reg_run step) first. Until "
            "then nothing knows where its tip is relative to the camera.");

    if (r.valid)
        FD_INFO("devreg", "'%s' registered to '%s': t=(%.4f, %.4f, %.4f) mm%s",
                device.c_str(), r.refDevice.c_str(), r.t.x, r.t.y, r.t.z,
                r.verified ? "" : " (UNVERIFIED - never checked away from its origin)");
    else
        FD_INFO("devreg", "'%s' is not registered - aiming with the seed from the "
                          "axis transforms, which is good to a degree or two",
                device.c_str());
    return CommandResult::Success();
}

// ---- device_reg_save -------------------------------------------------------

CommandResult ExecDeviceRegSave(ExecutionContext& ctx, const ParamValues& p)
{
    DeviceReg r;
    r.device    = p.GetString("device");
    r.refDevice = p.GetString("ref_device");
    if (r.device.empty())  return CommandResult::Fail("device is empty");
    if (r.device == r.refDevice)
        return CommandResult::Fail("device and ref_device are both '" + r.device +
                                   "' - a device cannot be registered to itself; "
                                   "ref_device names the gantry the CAMERA rides on");

    r.taughtRot = Vec3Param(p, "rot_u_deg", "rot_v_deg", "rot_w_deg");
    r.hasRot    = p.GetBool("has_rotation");

    // The fit does the refusing, and it fills every quality field before it
    // returns — so a rejection can name the number that was out instead of
    // saying the measurement failed.
    const std::string why = DeviceRegFit(
        r, p.GetDouble("jog_mm"),
        Vec3Param(p, "h0_x_mm", "h0_y_mm", "h0_z_mm"),
        Vec3Param(p, "p0_x_mm", "p0_y_mm", "p0_z_mm"),
        Vec3Param(p, "p1_x_mm", "p1_y_mm", "p1_z_mm"),
        Vec3Param(p, "p2_x_mm", "p2_y_mm", "p2_z_mm"),
        Vec3Param(p, "p3_x_mm", "p3_y_mm", "p3_z_mm"));
    if (!why.empty())
        return CommandResult::Fail("registration of '" + r.device + "' refused: " + why);

    const DeviceReg old = ctx.Wf().DeviceRegFor(r.device);

    // The engine worker does NOT write the file — same rule as a taught frame
    // and the needle offset. A run stopped mid-way must not be able to leave a
    // half-written registration behind.
    ctx.Wf().QueueDeviceReg(r);

    if (old.valid)
        FD_INFO("devreg", "'%s' re-registered: t (%.4f, %.4f, %.4f) -> (%.4f, %.4f, "
                          "%.4f) mm (moved %.4f mm); columns %.4f/%.4f/%.4f, "
                          "perpendicularity %+.3f/%+.3f/%+.3f deg, handed %+.0f",
                r.device.c_str(), old.t.x, old.t.y, old.t.z, r.t.x, r.t.y, r.t.z,
                std::sqrt((r.t.x - old.t.x) * (r.t.x - old.t.x) +
                          (r.t.y - old.t.y) * (r.t.y - old.t.y) +
                          (r.t.z - old.t.z) * (r.t.z - old.t.z)),
                r.colNorm[0], r.colNorm[1], r.colNorm[2],
                r.orthoDeg[0], r.orthoDeg[1], r.orthoDeg[2], r.handed);
    else
        FD_INFO("devreg", "'%s' registered to '%s': t=(%.4f, %.4f, %.4f) mm; columns "
                          "%.4f/%.4f/%.4f, perpendicularity %+.3f/%+.3f/%+.3f deg, "
                          "handed %+.0f",
                r.device.c_str(), r.refDevice.c_str(), r.t.x, r.t.y, r.t.z,
                r.colNorm[0], r.colNorm[1], r.colNorm[2],
                r.orthoDeg[0], r.orthoDeg[1], r.orthoDeg[2], r.handed);
    FD_WARN("devreg", "'%s' is registered but UNVERIFIED - run the check to measure "
                      "it away from the pose it was fitted at; predicting at that "
                      "pose returns the measurement by construction and tests nothing",
            r.device.c_str());
    return CommandResult::Success();
}

// ---- device_reg_tip_save ---------------------------------------------------

CommandResult ExecDeviceRegTipSave(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    DeviceReg r = ctx.Wf().DeviceRegFor(device);

    const RegVec3 h = Vec3Param(p, "h_x_mm", "h_y_mm", "h_z_mm");
    const RegVec3 q = Vec3Param(p, "p_x_mm", "p_y_mm", "p_z_mm");
    const RegVec3 before = DeviceRegPredict(r, h);

    const std::string why = DeviceRegSetTip(r, h, q);
    if (!why.empty())
        return CommandResult::Fail("tip re-teach for '" + device + "' refused: " + why);

    const double moved = std::sqrt((q.x - before.x) * (q.x - before.x) +
                                   (q.y - before.y) * (q.y - before.y) +
                                   (q.z - before.z) * (q.z - before.z));
    ctx.Wf().QueueDeviceReg(r);
    FD_INFO("devreg", "'%s' tip re-taught: the tip is %.4f mm from where the old "
                      "record put it; the rotation is unchanged and the registration "
                      "is now unverified until the check runs",
            device.c_str(), moved);
    return CommandResult::Success();
}

// ---- device_reg_check_save -------------------------------------------------

CommandResult ExecDeviceRegCheckSave(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    DeviceReg r = ctx.Wf().DeviceRegFor(device);
    if (!r.valid)
        return CommandResult::Fail("'" + device + "' has no registration to check");

    r.checkResidualMm = p.GetDouble("residual_mm");
    r.checkPose       = Vec3Param(p, "pose_x_mm", "pose_y_mm", "pose_z_mm");
    r.verified        = p.GetBool("verified");
    r.checkedAt       = NowStamp();

    ctx.Wf().QueueDeviceReg(r);
    if (r.verified)
        FD_INFO("devreg", "'%s' checked: tip landed %.4f mm from prediction - verified",
                device.c_str(), r.checkResidualMm);
    else
        FD_WARN("devreg", "'%s' checked: tip landed %.4f mm from prediction - NOT "
                          "verified. A constant shift is a tool that moved (re-teach "
                          "the tip); a residual that grows with distance from the "
                          "fitted pose is the rotation (re-register).",
                device.c_str(), r.checkResidualMm);
    return CommandResult::Success();
}

// ---- move_to_point ---------------------------------------------------------

CommandResult ExecMoveToPoint(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const DeviceReg r = ctx.Wf().DeviceRegFor(device);
    if (!r.valid)
        return CommandResult::Fail(
            "'" + device + "' has no registration, so a point in another device's "
            "space means nothing to it - only a DELTA crosses between unregistered "
            "devices. Register it, or use a relative move.");

    // The point is stated in SOME device's axis space, and which one is not a
    // detail the caller may leave to chance: a gantry coordinate interpreted
    // against a registration fitted to a different reference is wrong by the
    // whole distance between two origins.
    const std::string ref = p.GetString("ref_device");
    if (!ref.empty() && ref != r.refDevice)
        return CommandResult::Fail("'" + device + "' is registered against '" +
                                   r.refDevice + "', but the point was given in '" +
                                   ref + "' space");

    if (!r.verified && !p.GetBool("allow_unverified"))
        return CommandResult::Fail(
            "'" + device + "' is registered but not verified - run the check first, "
            "or set allow_unverified. A fit that has only been evaluated at its own "
            "origin has tested the measurement, not the rotation.");

    // Rotations, before anything is computed from them. A hexapod's tip swings
    // about its pivot, so a fit taken at one rotation describes no other.
    const std::map<std::string, double> live = m->Positions();
    if (r.hasRot) {
        const auto iu = live.find("U"), iv = live.find("V"), iw = live.find("W");
        if (iu != live.end() && iv != live.end() && iw != live.end()) {
            std::string why;
            if (!DeviceRegRotationOk(r, RegVec3{iu->second, iv->second, iw->second},
                                     &why))
                return CommandResult::Fail(why);
        }
    }

    const RegVec3 target = Vec3Param(p, "x_mm", "y_mm", "z_mm");
    const RegVec3 h      = DeviceRegSolve(r, target);

    const std::string axes = p.GetString("axes");
    if (axes.empty()) return CommandResult::Fail("axes is empty - name the axes to command");

    const std::vector<std::string> have = m->Axes();
    std::vector<std::pair<std::string, double>> targets;
    double travelSq = 0.0;
    for (const char raw : axes) {
        const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(raw)));
        const std::string axis(1, c);
        double val = 0.0;
        if (c == 'X')      val = h.x;
        else if (c == 'Y') val = h.y;
        else if (c == 'Z') val = h.z;
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
        if (const auto it = live.find(axis); it != live.end()) {
            const double d = val - it->second;
            travelSq += d * d;
        }
        targets.emplace_back(axis, val);
    }

    // The ceiling. Unattended there is nobody to notice that a stale record
    // turned a short approach into a lunge across the travel, and a registration
    // is exactly the kind of number that goes wrong silently.
    const double maxMM = p.GetDouble("max_mm");
    const double travel = std::sqrt(travelSq);
    if (maxMM > 0.0 && travel > maxMM) {
        char buf[224];
        std::snprintf(buf, sizeof(buf),
                      "move of %.4f mm exceeds max_mm %.4f - the point (%.4f, %.4f, "
                      "%.4f) in '%s' space solves to (%.4f, %.4f, %.4f) on '%s', "
                      "which is a long way from where it is",
                      travel, maxMM, target.x, target.y, target.z,
                      r.refDevice.c_str(), h.x, h.y, h.z, device.c_str());
        return CommandResult::Fail(buf);
    }

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    FD_INFO("devreg", "'%s' approaching (%.4f, %.4f, %.4f) in '%s' space -> pose "
                      "(%.4f, %.4f, %.4f), %.4f mm of travel",
            device.c_str(), target.x, target.y, target.z, r.refDevice.c_str(),
            h.x, h.y, h.z, travel);
    return MoveWithApproach(ctx, m, idx, targets, p, "the registered point");
}

}   // namespace

void RegisterDeviceRegCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // every action goes through ExecutionContext::Wf() at run time
    (void)hw;

    reg.Register({"device_reg", "Registration",
                  "Read a device's tip registration into ctx (<prefix>_ok/_tx.._c3z); "
                  "with none, reports the aiming columns the axis transforms imply",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device whose tool tip is being placed"},
                      {"ref_device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the device the CAMERA rides on - the space the tip is "
                       "reported in"},
                      {"ctx_prefix", PT::String, false, "reg", 0, 0, {},
                       "ctx prefix for the registration"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when there is no registration, instead of "
                       "reporting <prefix>_ok = false"},
                  },
                  CommandSpec::Needs::None, ExecDeviceReg});

    reg.Register({"device_reg_save", "Registration",
                  "Store a freshly measured four-point fit as THE registration for a "
                  "device (refuses a fit that fails its quality gates)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device being registered"},
                      {"ref_device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the device the camera rides on"},
                      {"jog_mm", PT::Double, true, {}, 0, 0, {},
                       "how far each axis was jogged between P0 and P1/P2/P3; every "
                       "column is divided by it, so a jog that did not complete shows "
                       "up as a column scale below 1 and is refused"},
                      {"h0_x_mm", PT::Double, true, {}, 0, 0, {}, "device pose X at P0"},
                      {"h0_y_mm", PT::Double, true, {}, 0, 0, {}, "device pose Y at P0"},
                      {"h0_z_mm", PT::Double, true, {}, 0, 0, {}, "device pose Z at P0"},
                      {"p0_x_mm", PT::Double, true, {}, 0, 0, {},
                       "tip at rest, reference X", PinDir::Any, "Measured"},
                      {"p0_y_mm", PT::Double, true, {}, 0, 0, {}, "tip at rest, reference Y", PinDir::Any, "Measured"},
                      {"p0_z_mm", PT::Double, true, {}, 0, 0, {}, "tip at rest, reference Z", PinDir::Any, "Measured"},
                      {"p1_x_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +X jog, reference X", PinDir::Any, "Measured"},
                      {"p1_y_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +X jog, reference Y", PinDir::Any, "Measured"},
                      {"p1_z_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +X jog, reference Z", PinDir::Any, "Measured"},
                      {"p2_x_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +Y jog, reference X", PinDir::Any, "Measured"},
                      {"p2_y_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +Y jog, reference Y", PinDir::Any, "Measured"},
                      {"p2_z_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +Y jog, reference Z", PinDir::Any, "Measured"},
                      {"p3_x_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +Z jog, reference X", PinDir::Any, "Measured"},
                      {"p3_y_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +Z jog, reference Y", PinDir::Any, "Measured"},
                      {"p3_z_mm", PT::Double, true, {}, 0, 0, {}, "tip after the +Z jog, reference Z", PinDir::Any, "Measured"},
                      {"has_rotation", PT::Bool, false, false, 0, 0, {},
                       "the device reports U/V/W; the fit is then refused at any other "
                       "rotation than the one below",
                       PinDir::Any, "Rotation"},
                      {"rot_u_deg", PT::Double, false, 0.0, 0, 0, {},
                       "U the fit was taught at", PinDir::Any, "Rotation"},
                      {"rot_v_deg", PT::Double, false, 0.0, 0, 0, {},
                       "V the fit was taught at", PinDir::Any, "Rotation"},
                      {"rot_w_deg", PT::Double, false, 0.0, 0, 0, {},
                       "W the fit was taught at", PinDir::Any, "Rotation"},
                  },
                  CommandSpec::Needs::None, ExecDeviceRegSave});

    reg.Register({"device_reg_tip_save", "Registration",
                  "Re-teach the tip offset alone, keeping the measured rotation - the "
                  "one-pick path for a changed gripper",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "the registered device"},
                      {"h_x_mm", PT::Double, true, {}, 0, 0, {}, "device pose X at the measurement"},
                      {"h_y_mm", PT::Double, true, {}, 0, 0, {}, "device pose Y"},
                      {"h_z_mm", PT::Double, true, {}, 0, 0, {}, "device pose Z"},
                      {"p_x_mm", PT::Double, true, {}, 0, 0, {}, "measured tip, reference X"},
                      {"p_y_mm", PT::Double, true, {}, 0, 0, {}, "measured tip, reference Y"},
                      {"p_z_mm", PT::Double, true, {}, 0, 0, {}, "measured tip, reference Z"},
                  },
                  CommandSpec::Needs::None, ExecDeviceRegTipSave});

    reg.Register({"device_reg_check_save", "Registration",
                  "Record what the check pass measured, and whether it agreed",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "the registered device"},
                      {"residual_mm", PT::Double, true, {}, 0, 0, {},
                       "how far the tip landed from where the record predicted"},
                      {"verified", PT::Bool, false, false, 0, 0, {},
                       "the residual was inside tolerance AND the check pose was away "
                       "from the pose the fit was taught at"},
                      {"pose_x_mm", PT::Double, false, 0.0, 0, 0, {}, "device pose X at the check"},
                      {"pose_y_mm", PT::Double, false, 0.0, 0, 0, {}, "device pose Y at the check"},
                      {"pose_z_mm", PT::Double, false, 0.0, 0, 0, {}, "device pose Z at the check"},
                  },
                  CommandSpec::Needs::None, ExecDeviceRegCheckSave});

    // The thing that is impossible without a registration: an absolute point,
    // measured through the camera, commanded to a device that has never shared
    // an origin with it. Deliberately named and documented as an APPROACH — the
    // fit is worth tens of microns and a placement is worth twenty, so the
    // measure/correct loop still lands the part. See docs/device-registration-design.md.
    reg.Register({"move_to_point", "Registration",
                  "APPROACH a point given in the reference device's axis space, on a "
                  "registered device - gets the tip close; it does not place it",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the registered device to move"},
                      {"x_mm", PT::Double, true, {}, 0, 0, {},
                       "target X, in the REFERENCE device's axis space"},
                      {"y_mm", PT::Double, true, {}, 0, 0, {}, "target Y, reference space"},
                      {"z_mm", PT::Double, true, {}, 0, 0, {}, "target Z, reference space"},
                      {"axes", PT::String, false, "XYZ", 0, 0, {},
                       "which axes to command; the others are left where they are"},
                      {"ref_device", PT::DeviceMotion, false, "", 0, 0, {},
                       "the space the point is given in; empty trusts the "
                       "registration, a name is checked against it"},
                      {"max_mm", PT::Double, false, 10.0, 0.0, 500.0, {},
                       "refuse a move longer than this; 0 disables the ceiling. A "
                       "stale registration turns a short approach into a lunge, and "
                       "there is nobody watching an unattended run"},
                      {"allow_unverified", PT::Bool, false, false, 0, 0, {},
                       "move through a registration whose check has not been run"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToPoint});

    // The two script launchers, same shape as needle_cal_run: set the ctx the
    // script reads with fd.param, resolve the file, run it inline on this worker.
    CommandRegistry* rp = &reg;

    reg.Register({"device_reg_run", "Registration",
                  "Run the whole device registration (device_registration.lua)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device carrying the tool tip"},
                      {"ref_device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the device the camera rides on"},
                      {"jog_mm", PT::Double, false, 10.0, 1.0, 100.0, {},
                       "per-axis jog between measurements. Longer resolves the angle "
                       "better - 10 mm picked to 5 um is about 0.03 deg - and is "
                       "bounded by travel and clearance, not by the field of view"},
                      {"autofocus", PT::Bool, false, true, 0, 0, {},
                       "focus on the tip at every pose; the focused reference Z IS "
                       "the tip's height measurement"},
                      {"focus_range_mm", PT::Double, false, 1.0, 0.01, 20.0, {},
                       "focus sweep after a jog. Wider than a normal focus on "
                       "purpose: the seed aims to a degree or two, which over the "
                       "jog is several hundred microns of Z"},
                      {"v_travel", PT::Double, false, 0.0, 0.0, 100.0, {},
                       "velocity for the reference device's moves; 0 = leave as set"},
                      {"v_jog", PT::Double, false, 0.0, 0.0, 100.0, {},
                       "velocity for the registered device's jogs; 0 = leave as set"},
                      {"settle_ms", PT::Int, false, 300, 0, 60000, {},
                       "pause after each jog before looking"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/device_registration.lua", 0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("reg_device", p.GetString("device"));
                      c.CtxSet("reg_ref", p.GetString("ref_device"));
                      c.CtxSet("reg_jog_mm", p.GetDouble("jog_mm"));
                      c.CtxSet("reg_autofocus", p.GetBool("autofocus"));
                      c.CtxSet("reg_focus_range_mm", p.GetDouble("focus_range_mm"));
                      c.CtxSet("reg_v_travel", p.GetDouble("v_travel"));
                      c.CtxSet("reg_v_jog", p.GetDouble("v_jog"));
                      c.CtxSet("reg_settle_ms", static_cast<double>(p.GetInt("settle_ms")));
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});

    reg.Register({"device_reg_check_run", "Registration",
                  "Check a registration against one measured tip, and optionally "
                  "adopt it as a new tip offset (device_reg_check.lua)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "the registered device"},
                      {"check_dx_mm", PT::Double, false, 5.0, -100.0, 100.0, {},
                       "check pose, X offset from where the fit was taught. Zero on "
                       "all three axes checks only that the measurement repeats - it "
                       "cannot see a rotation error at all"},
                      {"check_dy_mm", PT::Double, false, 5.0, -100.0, 100.0, {},
                       "check pose, Y offset from the fitted pose"},
                      {"check_dz_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "check pose, Z offset from the fitted pose"},
                      {"tol_mm", PT::Double, false, 0.05, 0.001, 5.0, {},
                       "residual that counts as verified"},
                      {"adopt_tip", PT::Bool, false, false, 0, 0, {},
                       "re-teach t from this measurement, keeping the rotation. The "
                       "gripper-change path - only correct when you know WHY the tip "
                       "moved, because it zeroes the residual either way"},
                      {"autofocus", PT::Bool, false, true, 0, 0, {}, "focus on the tip"},
                      {"focus_range_mm", PT::Double, false, 0.5, 0.01, 20.0, {},
                       "focus sweep at the check pose"},
                      {"v_travel", PT::Double, false, 0.0, 0.0, 100.0, {},
                       "velocity for the reference device's moves; 0 = leave as set"},
                      {"v_jog", PT::Double, false, 0.0, 0.0, 100.0, {},
                       "velocity for the registered device's move; 0 = leave as set"},
                      {"settle_ms", PT::Int, false, 300, 0, 60000, {},
                       "pause after the move before looking"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/device_reg_check.lua", 0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("reg_device", p.GetString("device"));
                      c.CtxSet("reg_check_dx_mm", p.GetDouble("check_dx_mm"));
                      c.CtxSet("reg_check_dy_mm", p.GetDouble("check_dy_mm"));
                      c.CtxSet("reg_check_dz_mm", p.GetDouble("check_dz_mm"));
                      c.CtxSet("reg_tol_mm", p.GetDouble("tol_mm"));
                      c.CtxSet("reg_adopt_tip", p.GetBool("adopt_tip"));
                      c.CtxSet("reg_autofocus", p.GetBool("autofocus"));
                      c.CtxSet("reg_focus_range_mm", p.GetDouble("focus_range_mm"));
                      c.CtxSet("reg_v_travel", p.GetDouble("v_travel"));
                      c.CtxSet("reg_v_jog", p.GetDouble("v_jog"));
                      c.CtxSet("reg_settle_ms", static_cast<double>(p.GetInt("settle_ms")));
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});
}

} // namespace firedog
