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
//   move_tool_to_camera    ...that point being wherever the camera is looking now
//   move_camera_to_tool    the inverse: drive the camera over a tool's tip
//   tool_travel_to_camera  lift off, cross, then move_tool_to_camera
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
#include "command/RegApproach.h"   // the shared halves, declared for CommandsGrip
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

}   // namespace

// ---- the two halves both point commands share -------------------------------
//
// A registration may only be driven through under conditions the record
// itself states, and there must be exactly one statement of them: a second
// command that forgot the rotation guard would produce a plausible pose that
// is simply wrong, which is the failure this whole record exists to avoid.
//
// Outside the anonymous namespace, declared in command/RegApproach.h: the grip
// family's grip_move_to_camera is this approach with an offset subtracted, and
// it must not carry a second copy of these refusals.
//
// `expectedRef` is the space a caller CLAIMS its point is in; empty trusts
// the record. A gantry coordinate interpreted against a registration taught
// to a different reference is wrong by the whole distance between two origins,
// and that is not a detail a recipe author may leave to chance.
//
// `requireVerified` is the ONE knob, passed explicitly by every caller rather
// than defaulted, because it is the one refusal that is not about whether the
// record can be predicted through at all: it is about whether a record nothing
// has ever checked may MOVE A TOOL. Sending the camera to look at a tip is not
// that, and gating it would hide the very disagreement the picture exists to
// show - see move_camera_to_tool.
CommandResult PrepareRegMove(ExecutionContext& ctx, const ParamValues& p,
                             const std::string& device,
                             const std::string& expectedRef, DeviceReg* regOut,
                             IMotionController** mOut, int* idxOut,
                             bool requireVerified)
{
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

    if (!expectedRef.empty() && expectedRef != r.refDevice)
        return CommandResult::Fail("'" + device + "' is registered against '" +
                                   r.refDevice + "', but the point was given in '" +
                                   expectedRef + "' space");

    if (requireVerified && !r.verified && !p.GetBool("allow_unverified"))
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

    *regOut = r;
    *mOut   = m;
    *idxOut = idx;
    return CommandResult::Success();
}

// The other half: drive a device to a point in ITS OWN axis space, once the
// record has been found and its refusals have passed. Every command in this
// family ends here, whichever device it is moving, so the travel ceiling, the
// two-speed approach and the log line are the same wherever the point came
// from - a second implementation of 'drive a device to a point' is precisely
// what this family exists to not have.
//
// `ceiling` is the caller's account of where the point CAME FROM, appended to
// the refusal when the move is too long: 'a stale registration turned a short
// approach into a lunge' is only actionable if the message says which numbers
// produced it, and only the caller knows whether that was a solve, a camera
// pose or a predicted tip.
CommandResult ApproachAxisPoint(ExecutionContext& ctx, const ParamValues& p,
                                const std::string& device, IMotionController* m,
                                int idx, const RegVec3& h, const char* ceiling,
                                const char* what)
{
    const std::map<std::string, double> live = m->Positions();

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
        char buf[320];
        std::snprintf(buf, sizeof(buf), "move of %.4f mm exceeds max_mm %.4f - %s",
                      travel, maxMM, ceiling);
        return CommandResult::Fail(buf);
    }

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    FD_INFO("devreg", "'%s' approaching pose (%.4f, %.4f, %.4f), %.4f mm of travel",
            device.c_str(), h.x, h.y, h.z, travel);
    return MoveWithApproach(ctx, m, idx, targets, p, what);
}

// Drive a REGISTERED device so its tip lands on a point in the reference
// device's axis space: the solve, and then the shared approach above.
CommandResult ApproachRefPoint(ExecutionContext& ctx, const ParamValues& p,
                               const std::string& device, const DeviceReg& r,
                               IMotionController* m, int idx, const RegVec3& target)
{
    const RegVec3 h = DeviceRegSolve(r, target);
    char ceiling[288];
    std::snprintf(ceiling, sizeof(ceiling),
                  "the point (%.4f, %.4f, %.4f) in '%s' space solves to (%.4f, %.4f, "
                  "%.4f) on '%s', which is a long way from where it is",
                  target.x, target.y, target.z, r.refDevice.c_str(), h.x, h.y, h.z,
                  device.c_str());
    FD_INFO("devreg", "'%s' aiming at (%.4f, %.4f, %.4f) in '%s' space",
            device.c_str(), target.x, target.y, target.z, r.refDevice.c_str());
    return ApproachAxisPoint(ctx, p, device, m, idx, h, ceiling, "the registered point");
}

namespace {

// ---- move_to_point ---------------------------------------------------------

CommandResult ExecMoveToPoint(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    const CommandResult ready = PrepareRegMove(ctx, p, device, p.GetString("ref_device"),
                                               &r, &m, &idx, true);
    if (!ready.ok) return ready;

    return ApproachRefPoint(ctx, p, device, r, m, idx,
                            Vec3Param(p, "x_mm", "y_mm", "z_mm"));
}

// ---- move_tool_to_camera ----------------------------------------------------
//
// The point the camera is looking at RIGHT NOW, handed to a registered tool.
// It is move_to_point with the target read off the reference device instead of
// typed into the step, and that is the whole difference - which makes it the
// one worth having, because the coordinates under a camera are not something a
// recipe author can write down in advance: they are wherever the operator, an
// alignment or a detection has just left the stage.
//
// Z IS AN OFFSET FROM THE FOCUSED PLANE, NOT A HEIGHT - the same argument the
// needle's dz makes. The reference Z of what the camera is looking at is 'the
// gantry Z that focuses on it', so 0 would send the tip INTO the surface in
// focus. The default is -2 mm, which on this machine stands the tool 2 mm off
// it: negative counts away from the surface, because gantry-main's transform
// inverts Z. That is a measured fact about this machine (2026-09-01), stated
// in the parameter help and dated there, not a rule - re-check it after any
// edit to config/motion_transforms.json.
//
// Still an APPROACH, exactly like move_to_point: a typed rotation and one
// measured tip get the tool close, and a placement still measures and corrects.
CommandResult ExecMoveToolToCamera(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    const CommandResult ready = PrepareRegMove(ctx, p, device, "", &r, &m, &idx, true);
    if (!ready.ok) return ready;

    // Where the camera stands. Read through layer 2 by the record's OWN
    // reference name: the registration says which device it was taught
    // against, and nothing here may second-guess it.
    std::map<std::string, double> refPose;
    if (!ctx.Wf().DevicePose(r.refDevice, refPose))
        return CommandResult::Fail("'" + r.refDevice + "' is not connected - there "
                                   "is no camera point to move to");
    const auto ix = refPose.find("X"), iy = refPose.find("Y"), iz = refPose.find("Z");
    if (ix == refPose.end() || iy == refPose.end() || iz == refPose.end())
        return CommandResult::Fail("'" + r.refDevice + "' does not report X/Y/Z");

    const RegVec3 target{ix->second, iy->second,
                         iz->second + p.GetDouble("z_offset_mm")};
    FD_INFO("devreg", "'%s' to the camera point: '%s' is at (%.4f, %.4f, %.4f), "
                      "z_offset %+.4f",
            device.c_str(), r.refDevice.c_str(), ix->second, iy->second, iz->second,
            p.GetDouble("z_offset_mm"));
    return ApproachRefPoint(ctx, p, device, r, m, idx, target);
}

// ---- move_camera_to_tool ----------------------------------------------------
//
// The other direction, and the cheap one: leave the tool where it is and drive
// the CAMERA until its centre lands on the tip. Same record, same refusals, read
// backwards - p = R*h + t predicts where the tip is from the device's live pose,
// and the reference device's own X/Y is by definition the point its camera is
// looking at, which is what makes the inverse a plain move rather than a solve.
//
// It is worth having for two different reasons:
//
//   * LOOKING IS NOT PLACING. Every other command here moves a tool through a
//     registration that is a typed rotation plus one measured point, so each one
//     is an approach with a ceiling and an unverified refusal. This one moves the
//     camera over its own workspace to take a picture; when the record is wrong
//     the picture is of the wrong place, and that is the whole point - it is how
//     an operator SEES what the registration costs, next to the overlay's cross.
//   * It is the step a vision check needs. 'Photograph the gripper' had no way to
//     say where the gripper was, so it needed a taught position per tool per
//     station - which describes where the tool was PARKED, not where it is.
//
// Z DEFAULTS TO BEING LEFT ALONE (axes 'XY'). The tip's reference Z is by
// construction the gantry Z that FOCUSES on the tip, so commanding it is the
// useful thing - and it is also the leg that drives the camera down toward a
// tool of unknown height, so it is opt-in and never a side effect of asking to
// look at something.
CommandResult ExecMoveCameraToTool(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    // The registered device is not moved, but the refusals about whether this
    // record may be PREDICTED through are exactly the question being asked, so
    // they are shared rather than restated - a second opinion about that is how
    // two surfaces start disagreeing about where a tool is. The verified gate is
    // the one that does not apply: it asks whether an unchecked record may move
    // a tool, and this moves a camera. The Registration page's 'Look at the
    // tool' button makes the identical judgement, which is the point - one
    // answer, on both surfaces.
    const CommandResult ready = PrepareRegMove(ctx, p, device, "", &r, &m, &idx, false);
    if (!ready.ok) return ready;

    const std::map<std::string, double> live = m->Positions();
    const auto ix = live.find("X"), iy = live.find("Y"), iz = live.find("Z");
    if (ix == live.end() || iy == live.end() || iz == live.end())
        return CommandResult::Fail("'" + device + "' does not report X/Y/Z, so there "
                                   "is no pose to predict its tip from");
    const RegVec3 tip = DeviceRegPredict(r, RegVec3{ix->second, iy->second, iz->second});

    // The camera is what moves here, so the camera is what has to be resolvable
    // - and it is named by the RECORD, never assumed to be some device called
    // "gantry-main".
    std::string err;
    int         refIdx = -1;
    IMotionController* rm = ResolveMotion(ctx, r.refDevice, &refIdx, &err);
    if (!rm) return CommandResult::Fail(err);

    const RegVec3 target{tip.x, tip.y, tip.z + p.GetDouble("z_offset_mm")};
    char ceiling[288];
    std::snprintf(ceiling, sizeof(ceiling),
                  "'%s' tip is predicted at (%.4f, %.4f, %.4f) in '%s' space, which is "
                  "a long way from where the camera is standing",
                  device.c_str(), tip.x, tip.y, tip.z, r.refDevice.c_str());
    FD_INFO("devreg", "'%s' to the '%s' tip at (%.4f, %.4f, %.4f), z_offset %+.4f",
            r.refDevice.c_str(), device.c_str(), tip.x, tip.y, tip.z,
            p.GetDouble("z_offset_mm"));
    return ApproachAxisPoint(ctx, p, r.refDevice, rm, refIdx, target, ceiling,
                             "the predicted tip");
}

// ---- tool_travel_to_camera --------------------------------------------------
//
// The convenience wrapper, and the reason it is a command rather than three
// steps an author is trusted to remember: move_tool_to_camera is ONE
// coordinated move from wherever the tool happens to be, straight through
// whatever is between it and the camera. Retreat, cross, descend is the
// discipline dispense_procedure.lua spells out for the same reason, and a
// path that only exists in a recipe author's head is the one that gets left
// out of the next recipe.
//
//   1  LIFT OFF   a relative move along GLOBAL Z, through the device's own
//                 transform, so one number means the same thing on a hexapod
//                 and on a gantry whose axis Z is inverted. Positive is up on
//                 this machine; 0 skips the leg for a tool already clear.
//   2  CROSS      an optional taught position - the waypoint that keeps the
//                 traverse out of the fixtures. Optional because a station
//                 with nothing in the way should not have to invent one. It
//                 goes through MoveWithApproach like every other
//                 point-to-point move, so a configured approach zone slows
//                 the crossing too: that is the price of having exactly one
//                 implementation, and it is cheap next to a second copy
//                 drifting off the settle rule.
//   3  DESCEND    move_tool_to_camera, with its own z_offset.
//
// Each leg waits for arrival and settle before the next is issued: they are
// three moves, not one interpolated path, and a Stop between them leaves the
// tool in a defined place.
CommandResult ExecToolTravelToCamera(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    // The registration is checked FIRST, before anything moves: a lift off
    // followed by a refusal to descend leaves the tool parked mid-air for a
    // fault that was knowable before the first millimetre.
    const CommandResult ready = PrepareRegMove(ctx, p, device, "", &r, &m, &idx, true);
    if (!ready.ok) return ready;

    // One velocity for the whole trip, set once. MoveWithApproach only RESTORES
    // velocity_mm_s after its approach leg - the caller is what applies it - so
    // without this the lift and the crossing would run at whatever the last step
    // left on the device.
    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    const double lift = p.GetDouble("lift_mm");
    if (lift != 0.0) {
        std::vector<std::pair<std::string, double>> issued;
        if (!ctx.Wf().MoveRelativeGlobal(static_cast<std::size_t>(idx), 0.0, 0.0,
                                         lift, &issued))
            return CommandResult::Fail("lift off rejected - is global Z mapped for '" +
                                       device + "'?");
        // Wait on exactly what was SENT: the transform decided which local axes
        // those millimetres landed on, and recomputing them from a pose that has
        // since moved is how a wait starts passing early.
        const auto arrived = WaitArrive(ctx, m, issued, kMoveTolMM,
                                        p.GetInt("timeout_ms"));
        if (arrived != ExecutionContext::WaitResult::Ok) {
            ctx.Wf().StopMotion(static_cast<std::size_t>(idx));
            return CommandResult::Fail(arrived == ExecutionContext::WaitResult::Stopped
                                           ? "stopped"
                                           : "timeout lifting off");
        }
        const CommandResult settled = WaitSettle(ctx, m, idx);
        if (!settled.ok) return settled;
        FD_INFO("devreg", "'%s' lifted %+.3f mm along global Z", device.c_str(), lift);
    }

    const std::string via = p.GetString("via_position");
    if (!via.empty()) {
        const SavedPos* pos = ctx.FindPosition(device, via);
        if (!pos)
            return CommandResult::Fail("via_position uid not found for '" + device + "'");
        const auto targets = TargetsOf(*pos, m);
        if (targets.empty())
            return CommandResult::Fail("via_position shares no axes with '" + device + "'");
        const CommandResult crossed =
            MoveWithApproach(ctx, m, idx, targets, p, "'" + std::string(pos->name) + "'");
        if (!crossed.ok) return crossed;
    }

    return ExecMoveToolToCamera(ctx, p);
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

    // The same approach, aimed at the one point a recipe author cannot write
    // down: wherever the camera is looking when the step runs.
    reg.Register({"move_tool_to_camera", "Registration",
                  "APPROACH the point the camera is looking at now with a registered "
                  "tool - z_offset_mm stands it off the focused plane",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the registered device whose TIP is being sent to the camera"},
                      {"z_offset_mm", PT::Double, false, -2.0, -100.0, 100.0, {},
                       "offset from the camera's own Z, which is the Z that FOCUSES "
                       "on what is under it - so 0 sends the tip onto that surface. "
                       "NEGATIVE stands the tool off it (higher), positive drives it "
                       "in: measured on this machine 2026-09-01, where gantry-main's "
                       "transform inverts Z. Re-check after editing "
                       "config/motion_transforms.json"},
                      {"axes", PT::String, false, "XYZ", 0, 0, {},
                       "which axes to command; the others are left where they are"},
                      {"max_mm", PT::Double, false, 10.0, 0.0, 500.0, {},
                       "refuse a move longer than this; 0 disables the ceiling"},
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
                  CommandSpec::Needs::None, ExecMoveToolToCamera});

    // The inverse, and the only command in this family that does not move a
    // TOOL: the camera goes to look at one. Everything about it follows from
    // that - a wider ceiling, because a camera crossing to a parked tool is a
    // longer trip than a tool descending to the point beneath it; XY only,
    // because Z is what drives the lens toward a tool of unknown height; and NO
    // allow_unverified, because there is no verified gate to opt out of. That
    // gate asks whether an unchecked record may move a tool, and looking is how
    // an operator finds out what an unchecked record costs.
    reg.Register({"move_camera_to_tool", "Registration",
                  "Drive the REFERENCE device so its camera centre lands on a "
                  "registered tool's tip - the inverse of move_tool_to_camera",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the registered device whose TIP the camera is being sent to. "
                       "It does not move; the reference device named by its "
                       "registration does"},
                      {"axes", PT::String, false, "XY", 0, 0, {},
                       "which axes of the REFERENCE device to command. XY leaves the "
                       "camera's height alone; adding Z drives it to the height that "
                       "FOCUSES on the tip, which is also the leg that moves the lens "
                       "toward a tool of unknown height"},
                      {"z_offset_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to that focused Z, and only used when axes names Z. 0 "
                       "focuses on the tip itself; negative stands the camera off it "
                       "on this machine, where gantry-main's transform inverts Z"},
                      {"max_mm", PT::Double, false, 60.0, 0.0, 500.0, {},
                       "refuse a move longer than this; 0 disables the ceiling. Wider "
                       "than the tool commands' 10 mm because this crosses the "
                       "workspace to a parked tool, not the last few mm to a point"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveCameraToTool});

    // Retreat, cross, descend - one step, because a path that lives only in the
    // author's head is the one the next recipe leaves out.
    reg.Register({"tool_travel_to_camera", "Registration",
                  "Lift off, optionally cross via a taught position, then approach "
                  "the camera point (move_tool_to_camera)",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the registered device to bring to the camera"},
                      {"lift_mm", PT::Double, false, 4.0, -100.0, 100.0, {},
                       "retreat along GLOBAL Z before crossing, through the device's "
                       "own transform - positive is up on this machine. 0 skips the "
                       "leg for a tool that is already clear"},
                      {"via_position", PT::PositionUid, false, "", 0, 0, {},
                       "optional taught waypoint to cross through, so the traverse "
                       "misses the fixtures; empty goes straight to the camera "
                       "point. An approach zone, if set, slows this leg as well "
                       "as the final descent"},
                      {"z_offset_mm", PT::Double, false, -2.0, -100.0, 100.0, {},
                       "offset from the camera's focused Z on arrival. NEGATIVE "
                       "stands the tool off the surface (higher) on this machine - "
                       "see move_tool_to_camera"},
                      {"axes", PT::String, false, "XYZ", 0, 0, {},
                       "which axes the final approach commands"},
                      {"max_mm", PT::Double, false, 10.0, 0.0, 500.0, {},
                       "ceiling on the FINAL approach; 0 disables it"},
                      {"allow_unverified", PT::Bool, false, false, 0, 0, {},
                       "move through a registration whose check has not been run"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {},
                       "arrival timeout, per leg"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance for the crossing and the approach"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "velocity for every leg; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                  },
                  CommandSpec::Needs::None, ExecToolTravelToCamera});

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
                      {"autofocus", PT::Bool, false, false, 0, 0, {},
                       "sweep for focus at the check pose. OFF by default: the tip Z in "
                       "the record is a HAND-focused Z from the teach, so a check that "
                       "focuses by sweep measures how far the two METHODS disagree and "
                       "reports it as a registration error"},
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
