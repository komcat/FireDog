// CommandsDeviceReg.cpp : the device-registration command family.
//
// Registering a device means relating ITS axis origin to the camera device's, so
// a gripper tip has a known position at every moment instead of only just after
// somebody looked at it. See core/DeviceReg.h for where the rotation comes from
// and why the tip is measured by imprint.
//
//   device_reg             put the registration into ctx
//   device_reg_check_save  record what the check pass measured
//   move_to_point          command a reference-space point to a registered device
//   device_reg_check_run   run the daily check as one recipe step
//
// TEACHING IS NOT HERE. Adopting the rotation and measuring the tip both live on
// the Registration page (ui/pages/PageDeviceReg.cpp), because both need an
// operator at the machine placing a coupon, jogging a tool into contact and
// marking the corners of the mark it left — there is nothing for a recipe step
// to automate, and a second way to create a registration is a second answer to
// where the tool is. What a recipe needs from all this is the CONSUMING end:
// read the record, move through it, and check it.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"     // RunLuaFile (the check launcher)
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

CommandResult ExecDeviceReg(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string prefix = p.GetString("ctx_prefix");

    const DeviceReg r = ctx.Wf().DeviceRegFor(device);

    // Write every key FIRST, valid or not: a script that branches on <prefix>_ok
    // has to be able to read it on the failing path too, and `require` is only a
    // shortcut for that check.
    ctx.CtxSet(prefix + "_ok", r.valid);
    ctx.CtxSet(prefix + "_verified", r.verified);
    ctx.CtxSet(prefix + "_ref", r.refDevice);
    ctx.CtxSet(prefix + "_rot_source", r.rotSource);
    ctx.CtxSet(prefix + "_tx", r.t.x);
    ctx.CtxSet(prefix + "_ty", r.t.y);
    ctx.CtxSet(prefix + "_tz", r.t.z);
    for (int j = 0; j < 3; ++j) {
        const std::string c = prefix + "_c" + static_cast<char>('1' + j);
        ctx.CtxSet(c + "x", r.R[0][j]);
        ctx.CtxSet(c + "y", r.R[1][j]);
        ctx.CtxSet(c + "z", r.R[2][j]);
    }
    // The pose the tip was taught at. The check drives to an OFFSET from this,
    // because predicting at the taught pose returns the measurement by
    // construction and tests nothing.
    ctx.CtxSet(prefix + "_tip_x", r.imprintPose.x);
    ctx.CtxSet(prefix + "_tip_y", r.imprintPose.y);
    ctx.CtxSet(prefix + "_tip_z", r.imprintPose.z);
    ctx.CtxSet(prefix + "_home_x", r.homePose.x);
    ctx.CtxSet(prefix + "_home_y", r.homePose.y);
    ctx.CtxSet(prefix + "_home_z", r.homePose.z);
    ctx.CtxSet(prefix + "_has_home", r.hasHomePose);
    ctx.CtxSet(prefix + "_spread_mm", r.tipSpreadMm);
    ctx.CtxSet(prefix + "_at", r.regAt);
    ctx.CtxSet(prefix + "_tip_at", r.tipAt);
    ctx.CtxSet(prefix + "_checked_at", r.checkedAt);
    ctx.CtxSet(prefix + "_check_mm", r.checkResidualMm);

    if (!r.valid && p.GetBool("require"))
        return CommandResult::Fail(
            "'" + device + "' has no registration - adopt a rotation and teach its "
            "tip on the Registration page first. Until then nothing knows where its "
            "tool is relative to the camera.");

    if (r.valid) {
        // A check that ran and disagreed is not the same state as one that never
        // ran, and the log is where a stale registration is usually noticed.
        char state[128] = "";
        if (!r.verified && !r.checkedAt.empty())
            snprintf(state, sizeof(state),
                     " (check FAILED %.4f mm on %s - not verified)",
                     r.checkResidualMm, r.checkedAt.c_str());
        else if (!r.verified)
            snprintf(state, sizeof(state),
                     " (UNVERIFIED - never checked away from the pose the tip was "
                     "taught at)");
        FD_INFO("devreg", "'%s' registered to '%s': t=(%.4f, %.4f, %.4f) mm, rotation "
                          "from %s%s",
                device.c_str(), r.refDevice.c_str(), r.t.x, r.t.y, r.t.z,
                r.rotSource.c_str(), state);
    }
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
                          "the tip by imprint); a residual that grows with distance "
                          "from the taught pose is the rotation, which is only as "
                          "good as the axis transforms it came from.",
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
    // against a registration taught to a different reference is wrong by the
    // whole distance between two origins.
    const std::string ref = p.GetString("ref_device");
    if (!ref.empty() && ref != r.refDevice)
        return CommandResult::Fail("'" + device + "' is registered against '" +
                                   r.refDevice + "', but the point was given in '" +
                                   ref + "' space");

    if (!r.verified && !p.GetBool("allow_unverified"))
        return CommandResult::Fail(
            "'" + device + "' is registered but not verified - run the check first, "
            "or set allow_unverified. Its rotation came from the axis transforms, "
            "so until something has measured the tip away from where it was taught, "
            "nothing has put a number on what that costs.");

    // Rotations, before anything is computed from them. A hexapod's tip swings
    // about its pivot, so a record taken at one rotation describes no other.
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
    // turned a short approach into a lunge, and a registration is exactly the
    // kind of number that goes wrong silently.
    const double maxMM  = p.GetDouble("max_mm");
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
                  "Read a device's tip registration into ctx "
                  "(<prefix>_ok/_tx.._c3z/_tip_x..)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device whose tool tip is being placed"},
                      {"ctx_prefix", PT::String, false, "reg", 0, 0, {},
                       "ctx prefix for the registration"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when there is no registration, instead of "
                       "reporting <prefix>_ok = false"},
                  },
                  CommandSpec::Needs::None, ExecDeviceReg});

    reg.Register({"device_reg_check_save", "Registration",
                  "Record what the check pass measured, and whether it agreed",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "the registered device"},
                      {"residual_mm", PT::Double, true, {}, 0, 0, {},
                       "how far the tip landed from where the record predicted"},
                      {"verified", PT::Bool, false, false, 0, 0, {},
                       "the residual was inside tolerance AND the check pose was away "
                       "from the pose the tip was taught at"},
                      {"pose_x_mm", PT::Double, false, 0.0, 0, 0, {}, "device pose X at the check"},
                      {"pose_y_mm", PT::Double, false, 0.0, 0, 0, {}, "device pose Y at the check"},
                      {"pose_z_mm", PT::Double, false, 0.0, 0, 0, {}, "device pose Z at the check"},
                  },
                  CommandSpec::Needs::None, ExecDeviceRegCheckSave});

    // The thing that is impossible without a registration: an absolute point,
    // measured through the camera, commanded to a device that has never shared
    // an origin with it. Deliberately named and documented as an APPROACH — the
    // rotation is a typed permutation and the tip is one measured point, so this
    // gets a tool close and the measure/correct loop still lands the part. See
    // docs/device-registration-design.md.
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

    CommandRegistry* rp = &reg;

    reg.Register({"device_reg_check_run", "Registration",
                  "Check a registration against one measured tip, and optionally "
                  "adopt it as a new tip offset (device_reg_check.lua)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {}, "the registered device"},
                      {"check_dx_mm", PT::Double, false, 5.0, -100.0, 100.0, {},
                       "check pose, X offset from where the tip was taught. Zero on "
                       "all three axes checks only that the measurement repeats - it "
                       "cannot see a rotation error at all"},
                      {"check_dy_mm", PT::Double, false, 5.0, -100.0, 100.0, {},
                       "check pose, Y offset from the taught pose"},
                      {"check_dz_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "check pose, Z offset from the taught pose"},
                      {"tol_mm", PT::Double, false, 0.05, 0.001, 5.0, {},
                       "residual that counts as verified"},
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
