// CommandsGrip.cpp : the gripped-object command family.
//
// Where the object in a gripper's jaws sits, relative to that gripper's own
// registered tip. See core/GripProfile.h for the record and
// docs/gripped-object-design.md for why it is a measurement and not a constant.
//
//   grip_profile   put a profile into ctx (and where its tool is)
//   grip_mark      one point on the object - detected, or marked by hand
//   grip_apply     centre - tip = the offset, plus the verdict
//   grip_detect    every mark the profile's mode needs, then apply; the gripper
//                  to its inspect pose and the camera onto the tool first only
//                  when asked (go_to_inspect, look)
//   grip_measure   drive to the object, look at it, measure it (grip_measure.lua)
//   grip_move_to_camera   put the OBJECT under the camera: move_tool_to_camera
//                         with that offset subtracted from the tip's target
//   grip_camera_to_object the other direction: the CAMERA over the object,
//                         move_camera_to_tool with that offset added to the tip
//
// TEACHING IS NOT HERE, for the reason it is not in the device-registration
// family either: adopting a reference offset means an operator looking at a
// known-good grip and saying so. A run that promoted its own measurement to the
// standard would make every grip a good grip. What a recipe gets is the
// consuming end - measure, judge, report.
//
// The measurement is the same one the alignment family makes, which is why the
// pieces are shared rather than re-derived: `Workflow::Detect` for the auto
// path, `ExecutionContext::AwaitPick` for the manual one, and both filling the
// same PickResult so everything downstream is identical either way.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"     // RunLuaFile (the grip_measure launcher)
#include "command/MotionWaits.h"   // ResolveMotion, kMoveTolMM, the approach params
#include "command/RegApproach.h"   // PrepareRegMove / ApproachRefPoint
#include "core/DeviceReg.h"
#include "core/GripProfile.h"
#include "core/Log.h"
#include "hardware/motion/IMotionController.h"
#include "vision/HalconDetector.h"
#include "workflow/Workflow.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace firedog {

namespace {

using PT = ParamType;

// ctx key for one component of a marked point: "<prefix>_p1_wx".
// Deliberately the SAME shape place_mark uses (CommandsAlign.cpp): an operator
// reading two runs side by side should not have to learn two spellings of the
// same idea.
std::string MarkKey(const std::string& prefix, int index, const char* field)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s_p%d_%s", prefix.c_str(), index, field);
    return buf;
}

double CtxNumber(ExecutionContext& ctx, const std::string& key, bool* found)
{
    const auto v = ctx.CtxGet(key);
    if (!v || !std::holds_alternative<double>(*v)) { if (found) *found = false; return 0.0; }
    if (found) *found = true;
    return std::get<double>(*v);
}

// How many marks this profile's mode needs. One place, because the script asks
// it to decide how many grip_mark steps to issue and grip_apply asks it to
// decide what to read back — and those two disagreeing is a run that measures
// one point and averages two.
int MarksFor(const GripProfile& g)
{
    return (g.mode == "auto") ? 1 : kGripManualMarks;
}

// The profile, the gripper's registration, and the controller that carries the
// CAMERA. Every command here needs the same three and refuses on the same
// grounds, so there is one statement of them.
//
// The camera device is the one the measurement is relative to — a mark is a
// feature seen through a lens, so it is relative to where the lens is — and it
// is named by the REGISTRATION, never assumed to be some device called
// "gantry-main".
CommandResult PrepareGrip(ExecutionContext& ctx, const std::string& profile,
                          const GripProfile** gOut, DeviceReg* regOut,
                          IMotionController** camOut)
{
    const GripProfile* g = ctx.FindGrip(profile);
    if (!g)
        return CommandResult::Fail("grip profile not found: '" + profile + "'");
    if (g->device.empty())
        return CommandResult::Fail("grip profile '" + g->name + "' names no gripper - "
                                   "pick one on the Gripped Object page");

    const DeviceReg r = ctx.Wf().DeviceRegFor(g->device);
    if (!r.valid)
        return CommandResult::Fail(
            "'" + g->device + "' has no registration, so nothing knows where its tool "
            "is - and an object's position is only meaningful as an offset from that "
            "tool. Teach it on the Registration page (F8) first.");

    std::string err;
    int idx = -1;
    IMotionController* cam = ResolveMotion(ctx, r.refDevice, &idx, &err);
    if (!cam) return CommandResult::Fail(err);

    if (gOut)   *gOut   = g;
    if (regOut) *regOut = r;
    if (camOut) *camOut = cam;
    return CommandResult::Success();
}

// ---- grip_profile ----------------------------------------------------------

CommandResult ExecGripProfile(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string prefix = p.GetString("ctx_prefix");
    const GripProfile* g = ctx.FindGrip(p.GetString("profile"));

    // Write the keys FIRST, resolvable or not, for the reason device_reg does:
    // a script that branches on <prefix>_ok has to be able to read it on the
    // failing path too.
    ctx.CtxSet(prefix + "_ok", g != nullptr);
    if (!g) {
        if (p.GetBool("require"))
            return CommandResult::Fail("grip profile not found: '" +
                                       p.GetString("profile") + "'");
        return CommandResult::Success();
    }

    ctx.CtxSet(prefix + "_uid", g->uid);
    ctx.CtxSet(prefix + "_name", g->name);
    ctx.CtxSet(prefix + "_device", g->device);
    ctx.CtxSet(prefix + "_inspect", g->inspectUid);
    ctx.CtxSet(prefix + "_has_inspect", !g->inspectUid.empty());
    ctx.CtxSet(prefix + "_mode", g->mode);
    ctx.CtxSet(prefix + "_marks", static_cast<double>(MarksFor(*g)));
    ctx.CtxSet(prefix + "_has_ref", g->haveRef);
    ctx.CtxSet(prefix + "_ref_dx", g->refDx);
    ctx.CtxSet(prefix + "_ref_dy", g->refDy);
    ctx.CtxSet(prefix + "_tol_mm", g->tolMm);
    ctx.CtxSet(prefix + "_taught_at", g->taughtAt);

    // The camera device, because the script needs to name it in a move and
    // must not guess: it is a property of the gripper's registration.
    const DeviceReg r = ctx.Wf().DeviceRegFor(g->device);
    ctx.CtxSet(prefix + "_ref_device", r.refDevice);
    ctx.CtxSet(prefix + "_registered", r.valid);
    return CommandResult::Success();
}

// ---- grip_mark -------------------------------------------------------------
//
// One point on the object: detected when the profile says auto, marked with the
// crosshair when it says manual, and marked by hand as a FALLBACK when a
// detection was asked for and did not arrive. A near-twin of place_mark, and
// deliberately so - the two answer the same question about different things,
// and an operator who has learned one window has learned both.
//
// Nothing drives here. The object has to be in view already, which grip_measure
// arranges above this step; keeping the move out is what lets the two marks come
// from two different views without this command growing a station concept.
CommandResult ExecGripMark(ExecutionContext& ctx, const ParamValues& p)
{
    const GripProfile* g   = nullptr;
    DeviceReg          reg;
    IMotionController* cam = nullptr;
    const CommandResult ready = PrepareGrip(ctx, p.GetString("profile"), &g, &reg, &cam);
    if (!ready.ok) return ready;

    const int index = p.GetInt("index");
    const int marks = MarksFor(*g);
    if (index < 1 || index > marks) {
        char b[160];
        std::snprintf(b, sizeof(b), "index %d, but '%s' is in %s mode and takes %d mark(s)",
                      index, g->name.c_str(), g->mode.c_str(), marks);
        return CommandResult::Fail(b);
    }

    // Resolve the model BEFORE anything else, and do not act on the failure
    // yet: the same order align_station uses, so a mis-typed name costs no time
    // and the operator is asked for the point rather than shown a bare error.
    const VisionModel* model = nullptr;
    std::string        autoErr;
    if (g->mode == "auto") {
        model = g->detector.empty() ? nullptr : ctx.FindVisionModel(g->detector);
        if (!model)
            autoErr = g->detector.empty()
                          ? "'" + g->name + "' is set to detect but names no detection model"
                          : "'" + g->name + "' names no detection model '" + g->detector + "'";
        else if (std::string why; !HalconDetector::Available(&why))
            autoErr = "cannot run detection model '" + model->name + "': " + why;
    }

    ExecutionContext::PickResult pick;
    bool manual = (model == nullptr && autoErr.empty());

    if (model && autoErr.empty()) {
        // Prove the frame is post-move before believing it - the same gate
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
                FD_WARN("grip", "grip mark %d: no new frame within 2000 ms - detecting "
                                "on the latest frame (feed stalled?)", index);
        }

        VisionDetection det;
        ctx.Wf().Detect(*model, ctx.VisionDir(), &det);
        if (det.ok) {
            pick.px   = det.xPx;
            pick.py   = det.yPx;
            pick.dxMM = det.dxMM;
            pick.dyMM = det.dyMM;
            FD_INFO("grip", "grip mark %d: '%s' found at (%.2f, %.2f) px, score %.3f",
                    index, model->name.c_str(), det.xPx, det.yPx, det.score);
        } else {
            autoErr = "'" + model->name + "': " + det.error;
        }
    }

    // Detection asked for and not delivered. Falling back is the profile's
    // choice and defaults ON - but never silently: the warning names what
    // failed, so nobody can believe a run was automatic when it was not.
    if (!autoErr.empty()) {
        if (!g->autoFallbackManual)
            return CommandResult::Fail(autoErr + " (tick the profile's fallback, or "
                                       "re-teach the model)");
        FD_WARN("grip", "grip mark %d: %s - asking the operator", index, autoErr.c_str());
        manual = true;
    }

    ExecutionContext::PickRequest req;
    req.alignName   = g->name;
    req.station     = index;
    req.autoError   = autoErr;
    if (!autoErr.empty() && model) req.detectorUid = model->uid;
    req.label = ctx.Substitute(p.GetString("label"));
    if (req.label.empty()) {
        char lbl[96];
        if (marks == 1) std::snprintf(lbl, sizeof(lbl), "%s: mark the object", g->name.c_str());
        else std::snprintf(lbl, sizeof(lbl), "%s: point %d of %d on the object",
                           g->name.c_str(), index, marks);
        req.label = lbl;
    }
    // Two opposite edges of an object is exactly what the two-mark mode asks
    // for, so the corner diagrams belong here as they do on place_mark.
    req.stepIcons = (marks == 2);

    // Where the tool is, as a hint. This is the point the whole measurement is
    // relative to, and seeing it next to the crosshair is what turns "mark the
    // object" into "mark the object THIS tool is holding" - on a picture with
    // two grippers in it, that is not a rhetorical difference.
    {
        RegVec3     tip;
        std::string why;
        if (ctx.Wf().DeviceRegTip(g->device, tip, &why)) {
            ExecutionContext::PickHint h;
            h.wx    = tip.x;
            h.wy    = tip.y;
            h.label = g->device + " tip";
            req.hints.push_back(std::move(h));
        }
    }
    // ...then the mark already taken, so the second can be placed opposite the
    // first rather than from memory.
    for (int prev = 1; prev < index; ++prev) {
        bool fx = false, fy = false;
        ExecutionContext::PickHint h;
        h.wx = CtxNumber(ctx, MarkKey(p.GetString("ctx_prefix"), prev, "wx"), &fx);
        h.wy = CtxNumber(ctx, MarkKey(p.GetString("ctx_prefix"), prev, "wy"), &fy);
        if (!fx || !fy) continue;
        h.label = "pt" + std::to_string(prev);
        req.hints.push_back(std::move(h));
    }

    req.message = ctx.Substitute(p.GetString("message"));
    if (req.message.empty()) {
        char buf[192];
        if (marks == 1)
            std::snprintf(buf, sizeof(buf),
                          "Mark the centre of the object in the jaws, then OK");
        else
            std::snprintf(buf, sizeof(buf),
                          "Mark point %d of %d on the object (their MIDPOINT is its "
                          "centre), then OK", index, marks);
        req.message = buf;
    }

    if (manual && !ctx.AwaitPick(req, &pick))
        return CommandResult::Fail("cancelled by operator");

    // Where the camera is, plus where in the frame the feature was marked. The
    // pose is read AFTER the pick because that is the pose the pixel offset is
    // relative to - nothing moved, but the order is the load-bearing part.
    cam->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> live = cam->Positions();
    const auto ix = live.find("X"), iy = live.find("Y");
    if (ix == live.end() || iy == live.end())
        return CommandResult::Fail("device '" + cam->Name() + "' does not report X/Y");

    const double wx = ix->second + pick.dxMM;
    const double wy = iy->second + pick.dyMM;

    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(MarkKey(prefix, index, "wx"), wx);
    ctx.CtxSet(MarkKey(prefix, index, "wy"), wy);
    ctx.CtxSet(MarkKey(prefix, index, "px"), pick.px);
    ctx.CtxSet(MarkKey(prefix, index, "py"), pick.py);

    FD_INFO("grip", "grip mark %d (%s): %s (%.4f, %.4f) + mark (%+.4f, %+.4f) = "
                    "(%.4f, %.4f) mm",
            index, manual ? "manual" : "auto", cam->Name().c_str(), ix->second,
            iy->second, pick.dxMM, pick.dyMM, wx, wy);
    return CommandResult::Success();
}

// ---- grip_apply ------------------------------------------------------------
//
// The arithmetic, and nothing else moves.
//
//     centre = the midpoint of the marks (or the single detection)
//     tip    = DeviceRegTip(device)          - where the TOOL is, right now
//     offset = centre - tip
//
// Both terms are in the reference device's axis space, so the origin cancels and
// what is left is a vector - which is the only thing that crosses to a device
// whose origin has never been related to the camera's. The same vector is also
// published in the GRIPPER's own axes (R^T applied, no `t`: a delta needs no
// translation), because that is the form a correction move takes.
CommandResult ExecGripApply(ExecutionContext& ctx, const ParamValues& p)
{
    const GripProfile* g   = nullptr;
    DeviceReg          reg;
    const CommandResult ready = PrepareGrip(ctx, p.GetString("profile"), &g, &reg, nullptr);
    if (!ready.ok) return ready;

    const std::string prefix = p.GetString("ctx_prefix");
    const int         marks  = MarksFor(*g);

    double sx = 0.0, sy = 0.0;
    for (int i = 1; i <= marks; ++i) {
        bool fx = false, fy = false;
        const double wx = CtxNumber(ctx, MarkKey(prefix, i, "wx"), &fx);
        const double wy = CtxNumber(ctx, MarkKey(prefix, i, "wy"), &fy);
        if (!fx || !fy)
            return CommandResult::Fail("no mark " + std::to_string(i) + " under prefix '" +
                                       prefix + "' - run grip_mark for 1.." +
                                       std::to_string(marks) + " first");
        sx += wx;
        sy += wy;
    }
    // The midpoint, and nothing about the distance between the marks: that
    // separation is where the operator could see an edge, not the object's
    // width, and a check against a typed size would refuse good grips on
    // lighting. See GripProfile.h.
    const double cx = sx / marks;
    const double cy = sy / marks;

    // The tool, right now. Through layer 2's one predictor, refusals and all -
    // a rotated hexapod has no tip to be measured against, and a second
    // prediction here would be a second answer to where the tool is.
    RegVec3     tip;
    std::string why;
    if (!ctx.Wf().DeviceRegTip(g->device, tip, &why))
        return CommandResult::Fail("no tip for '" + g->device + "': " + why);

    const double dx = cx - tip.x;
    const double dy = cy - tip.y;

    // The same vector in the gripper's own axes, which is the form a correction
    // move takes. Two solves and a subtraction rather than a hand-rolled R^T:
    // `t` cancels between them (a DIFFERENCE needs no translation - applying one
    // would count the origin twice), and DeviceRegSolve stays the only place in
    // the codebase that multiplies a matrix. Z is 0 in both, so it stays 0 here:
    // this measurement is XY, in the plane the camera works in.
    const RegVec3 hObj = DeviceRegSolve(reg, RegVec3{cx, cy, 0.0});
    const RegVec3 hTip = DeviceRegSolve(reg, RegVec3{tip.x, tip.y, 0.0});
    const RegVec3 h{hObj.x - hTip.x, hObj.y - hTip.y, hObj.z - hTip.z};

    std::string reason;
    const bool  ok = GripVerdict(*g, dx, dy, &reason);

    ctx.CtxSet(prefix + "_x", cx);
    ctx.CtxSet(prefix + "_y", cy);
    ctx.CtxSet(prefix + "_tip_x", tip.x);
    ctx.CtxSet(prefix + "_tip_y", tip.y);
    ctx.CtxSet(prefix + "_dx", dx);
    ctx.CtxSet(prefix + "_dy", dy);
    ctx.CtxSet(prefix + "_dist", std::sqrt(dx * dx + dy * dy));
    ctx.CtxSet(prefix + "_hx", h.x);
    ctx.CtxSet(prefix + "_hy", h.y);
    ctx.CtxSet(prefix + "_hz", h.z);
    ctx.CtxSet(prefix + "_ok", ok);
    ctx.CtxSet(prefix + "_why", reason);
    // The error against the reference, which is the number an operator watches
    // drift. Zero and meaningless without a reference, and _ok already says so.
    ctx.CtxSet(prefix + "_err_x", g->haveRef ? dx - g->refDx : 0.0);
    ctx.CtxSet(prefix + "_err_y", g->haveRef ? dy - g->refDy : 0.0);

    // Evidence back to the document, by the route a taught frame takes: the
    // worker queues, App::Update stores. Never a new reference - see
    // ProductSession::ApplyGripResult.
    ctx.Wf().QueueGripResult(Workflow::GripMeasurement{g->uid, dx, dy, cx, cy});

    if (ok) {
        FD_INFO("grip", "'%s': object at (%.4f, %.4f), tip at (%.4f, %.4f), offset "
                        "(%+.4f, %+.4f) mm - OK",
                g->name.c_str(), cx, cy, tip.x, tip.y, dx, dy);
    } else {
        FD_WARN("grip", "'%s': offset (%+.4f, %+.4f) mm - %s",
                g->name.c_str(), dx, dy, reason.c_str());
    }

    // Reporting and failing are different jobs. The default fails, because an
    // unattended run that measured a bad grip and carried on would place a
    // part with it; `require_ok = false` is for a recipe that wants to branch
    // on <prefix>_ok itself.
    if (!ok && p.GetBool("require_ok"))
        return CommandResult::Fail("'" + g->name + "': " + reason);
    return CommandResult::Success();
}

// ---- grip_detect -----------------------------------------------------------
//
// Find the object, with the camera already on it. Every mark the profile's MODE
// asks for - one detection in auto, two crosshair picks in manual - then the
// arithmetic, in one step. No motion.
//
// It exists because the choice of mode is SAVED IN THE PROFILE, on the Gripped
// Object page, and a recipe that has to spell out "grip_mark 1, grip_mark 2,
// grip_apply" has copied that choice into the recipe: switch the profile to
// detect and the second grip_mark step fails on its index. grip_measure hides
// the same thing, but it also drives the gripper and the camera - this is the
// step for a recipe that has framed the object itself, or that wants to look
// again without moving anything.
//
// It is a composite of the two commands, not a third reading of them: the marks
// go through ExecGripMark (the fallback, the hints, the pick window) and the
// verdict through ExecGripApply, with the same ctx keys, so what a single
// grip_mark step writes and what this writes cannot drift apart.
//
// `look` is the one move it can make, and it is opt-in: the camera onto the
// tool's tip first, through the registry's own move_camera_to_tool with that
// command's defaults (the 60 mm ceiling, the tolerance) rather than a copy of
// them. Z stays where it is unless `focus_z` says otherwise, for the reason
// that command's axes default to XY.
// Run another registered command with ITS defaults for everything not given:
// the ceilings, tolerances and timeouts belong to that command, and a copy of
// them here would be the one that goes stale.
static CommandResult RunRegistered(ExecutionContext& ctx, const CommandRegistry& reg,
                            const char* name, const nlohmann::json& args)
{
    const CommandSpec* spec = reg.Find(name);
    if (!spec) return CommandResult::Fail(std::string(name) + " is not registered");
    nlohmann::json merged;
    const std::string ve = ValidateParams(*spec, args, &merged);
    if (!ve.empty()) return CommandResult::Fail(std::string(name) + ": " + ve);
    const CommandResult r = spec->execute(ctx, ParamValues(std::move(merged)));
    if (!r.ok) return CommandResult::Fail(std::string(name) + ": " + r.error);
    return r;
}

CommandResult ExecGripDetect(ExecutionContext& ctx, const ParamValues& p,
                             const CommandRegistry& reg)
{
    const GripProfile* g = nullptr;
    const CommandResult ready = PrepareGrip(ctx, p.GetString("profile"), &g, nullptr, nullptr);
    if (!ready.ok) return ready;

    const std::string profile = p.GetString("profile");
    const std::string prefix  = p.GetString("ctx_prefix");
    const int         marks   = MarksFor(*g);

    FD_INFO("grip", "grip detect: '%s' in %s mode - %s%s%d mark(s), then apply",
            g->name.c_str(), g->mode.c_str(),
            p.GetBool("go_to_inspect") ? "gripper to its inspect pose, " : "",
            p.GetBool("look") ? "camera onto the tool, " : "", marks);

    // The gripper to its inspect pose first, when asked. That pose carries the
    // ROTATIONS the reference offset was taught at, which is what makes two
    // measurements comparable; a profile without one is measured where it
    // stands, and says so - the same leg, and the same words, as
    // grip_measure.lua.
    if (p.GetBool("go_to_inspect")) {
        if (g->inspectUid.empty()) {
            FD_WARN("grip", "'%s' has no inspect pose taught - measuring %s where it "
                            "stands. A reference offset taught at another rotation will "
                            "not compare.",
                    g->name.c_str(), g->device.c_str());
        } else {
            const CommandResult r = RunRegistered(
                ctx, reg, "move_to_position",
                {{"device", g->device}, {"position", g->inspectUid}});
            if (!r.ok) return r;
        }
    }

    if (p.GetBool("look")) {
        // Z is commanded when asked for by name OR by an offset: an offset
        // typed against a move that then leaves Z alone is a number that did
        // nothing, silently. The height is the tip's focused Z plus it.
        const double zoff  = p.GetDouble("z_offset_mm");
        const bool   takeZ = p.GetBool("focus_z") || zoff != 0.0;
        if (takeZ)
            FD_INFO("grip", "grip detect: camera Z to the tip's focused height %+.4f mm",
                    zoff);
        const CommandResult r = RunRegistered(
            ctx, reg, "move_camera_to_tool",
            {{"device", g->device}, {"axes", takeZ ? "XYZ" : "XY"}, {"z_offset_mm", zoff}});
        if (!r.ok) return r;
    }

    // Settle after whichever moves were made, before looking.
    if (p.GetBool("go_to_inspect") || p.GetBool("look")) {
        const int settle = p.GetInt("settle_ms");
        if (settle > 0 && !ctx.Sleep(settle)) return CommandResult::Fail("stopped");
    }

    // Every key the sub-command reads is supplied: a hand-built ParamValues
    // carries no spec defaults, and a missing one reads as 0 / "".
    for (int i = 1; i <= marks; ++i) {
        nlohmann::json m = {
            {"profile", profile},
            {"index", i},
            {"ctx_prefix", prefix},
            {"fresh_frames", p.GetInt("fresh_frames")},
            {"label", p.GetString("label")},
            {"message", p.GetString("message")},
        };
        const CommandResult r = ExecGripMark(ctx, ParamValues(std::move(m)));
        if (!r.ok) return r;
    }

    nlohmann::json a = {
        {"profile", profile},
        {"ctx_prefix", prefix},
        {"require_ok", p.GetBool("require_ok")},
    };
    return ExecGripApply(ctx, ParamValues(std::move(a)));
}

// ---- grip_move_to_camera ---------------------------------------------------
//
// move_tool_to_camera, for the OBJECT instead of the tip. The camera is looking
// at a point and the gripper must put what it is holding there; what it is
// holding sits `offset` from the tip, so the tip is sent to the camera point
// MINUS the offset and the object lands on it.
//
//     tip target = camera - (dx, dy)               (reference axis mm)
//
// THE OFFSET COMES FROM CTX - what grip_measure / grip_apply left under
// <prefix>_dx/_dy in THIS run. Never from the profile's `last*` (an earlier run,
// of a different pick) and never from the engine's copy of the document (taken
// at Start, before this pick existed). A run that has not measured this grip has
// no offset, and the step says so rather than moving on an assumed one.
//
// Z ASSUMES THE OBJECT SITS AT THE TIP'S HEIGHT. The offset is XY, measured in
// the plane the camera works in, so Z is exactly move_tool_to_camera's: the
// camera's focused Z plus z_offset_mm, applied to the TIP. An object that hangs
// below the jaws is that much closer to the surface than z_offset_mm says; a
// recipe that knows the height folds it into that parameter.
//
// Still an APPROACH, through the same refusals and the same ceiling as the tool
// commands: a typed rotation plus one measured tip gets the object close, and a
// placement still measures and corrects from there.
CommandResult ExecGripMoveToCamera(ExecutionContext& ctx, const ParamValues& p)
{
    const GripProfile* g = nullptr;
    const CommandResult ready = PrepareGrip(ctx, p.GetString("profile"), &g, nullptr, nullptr);
    if (!ready.ok) return ready;

    const std::string prefix = p.GetString("ctx_prefix");
    bool fx = false, fy = false;
    const double dx = CtxNumber(ctx, prefix + "_dx", &fx);
    const double dy = CtxNumber(ctx, prefix + "_dy", &fy);
    if (!fx || !fy)
        return CommandResult::Fail(
            "no grip offset under prefix '" + prefix + "' - run grip_detect (or "
            "grip_measure) for '" + g->name + "' earlier in this recipe. The step "
            "moves on what THIS run measured, never on an earlier one");

    // The registration's own refusals, verified gate included: this MOVES a
    // tool, unlike the look the measurement takes.
    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    const CommandResult reg = PrepareRegMove(ctx, p, g->device, "", &r, &m, &idx, true);
    if (!reg.ok) return reg;

    // Where the camera stands, read through layer 2 by the record's OWN
    // reference name - the same read move_tool_to_camera makes.
    std::map<std::string, double> refPose;
    if (!ctx.Wf().DevicePose(r.refDevice, refPose))
        return CommandResult::Fail("'" + r.refDevice + "' is not connected - there "
                                   "is no camera point to move to");
    const auto ix = refPose.find("X"), iy = refPose.find("Y"), iz = refPose.find("Z");
    if (ix == refPose.end() || iy == refPose.end() || iz == refPose.end())
        return CommandResult::Fail("'" + r.refDevice + "' does not report X/Y/Z");

    const RegVec3 target{ix->second - dx, iy->second - dy,
                         iz->second + p.GetDouble("z_offset_mm")};
    FD_INFO("grip", "'%s' to the camera point: '%s' is at (%.4f, %.4f, %.4f), object "
                    "offset (%+.4f, %+.4f), so the '%s' tip aims at (%.4f, %.4f), "
                    "z_offset %+.4f",
            g->name.c_str(), r.refDevice.c_str(), ix->second, iy->second, iz->second,
            dx, dy, g->device.c_str(), target.x, target.y, p.GetDouble("z_offset_mm"));
    return ApproachRefPoint(ctx, p, g->device, r, m, idx, target);
}

}   // namespace

// ---- grip_camera_to_object -------------------------------------------------
//
// The cheap direction, as move_camera_to_tool is for the tip: leave the gripper
// where it is and drive the CAMERA until its centre lands on the OBJECT in the
// jaws - the predicted tip plus the grip offset, both in the reference device's
// axis space, which is by definition the point its camera looks at. Looking is
// not placing, so the verified gate does not apply (the rotation guard and the
// travel ceiling still do), and a wrong record shows as the wrong picture -
// which is what the picture is for.
//
// WHICH offset is the choice, and it is explicit rather than defaulted through
// a fallback: `this_run` is what a grip_detect / grip_measure step measured
// earlier in THIS recipe, the same key grip_move_to_camera moves on;
// `reference` is the profile's adopted good grip, the EXPECTED place before
// anything has been measured; `last` is an earlier run's number, a different
// pick, kept as evidence. Missing is a failure that names the source, never a
// silent slide to the next one - a look at the wrong offset is a picture of
// the wrong place with nothing on it saying so.
CommandResult ExecGripCameraToObject(ExecutionContext& ctx, const ParamValues& p)
{
    const GripProfile* g = nullptr;
    const CommandResult ready = PrepareGrip(ctx, p.GetString("profile"), &g, nullptr, nullptr);
    if (!ready.ok) return ready;

    double            dx = 0.0, dy = 0.0;
    const std::string src = p.GetString("offset");
    if (src == "this_run") {
        const std::string prefix = p.GetString("ctx_prefix");
        bool fx = false, fy = false;
        dx = CtxNumber(ctx, prefix + "_dx", &fx);
        dy = CtxNumber(ctx, prefix + "_dy", &fy);
        if (!fx || !fy)
            return CommandResult::Fail(
                "no grip offset under prefix '" + prefix + "' - run grip_detect (or "
                "grip_measure) earlier in this recipe, or ask for the 'reference' offset");
    } else if (src == "reference") {
        if (!g->haveRef)
            return CommandResult::Fail("'" + g->name + "' has no reference offset adopted - "
                                       "adopt one on the Gripped Object page, or measure "
                                       "first and use 'this_run'");
        dx = g->refDx;
        dy = g->refDy;
    } else if (src == "last") {
        if (g->measuredAt.empty())
            return CommandResult::Fail("'" + g->name + "' has never been measured, so there "
                                       "is no 'last' offset");
        dx = g->lastDx;
        dy = g->lastDy;
    } else {
        return CommandResult::Fail("offset must be this_run, reference or last");
    }

    // The tip, through the registration's own refusals - the same judgement
    // move_camera_to_tool makes, unverified allowed for the same reason.
    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    const CommandResult reg = PrepareRegMove(ctx, p, g->device, "", &r, &m, &idx, false);
    if (!reg.ok) return reg;

    const std::map<std::string, double> live = m->Positions();
    const auto ix = live.find("X"), iy = live.find("Y"), iz = live.find("Z");
    if (ix == live.end() || iy == live.end() || iz == live.end())
        return CommandResult::Fail("'" + g->device + "' does not report X/Y/Z, so there "
                                   "is no pose to predict its tip from");
    const RegVec3 tip = DeviceRegPredict(r, RegVec3{ix->second, iy->second, iz->second});

    std::string        err;
    int                refIdx = -1;
    IMotionController* rm = ResolveMotion(ctx, r.refDevice, &refIdx, &err);
    if (!rm) return CommandResult::Fail(err);

    // The object is assumed at the tip's height: the offset is XY, in the plane
    // the camera measures, and z_offset_mm is the operator's word on the rest.
    const RegVec3 target{tip.x + dx, tip.y + dy, tip.z + p.GetDouble("z_offset_mm")};
    char ceiling[320];
    std::snprintf(ceiling, sizeof(ceiling),
                  "'%s' is predicted at (%.4f, %.4f) in '%s' space - the '%s' tip plus "
                  "the %s offset (%+.4f, %+.4f) - which is a long way from where the "
                  "camera is standing",
                  g->name.c_str(), target.x, target.y, r.refDevice.c_str(),
                  g->device.c_str(), src.c_str(), dx, dy);
    FD_INFO("grip", "'%s' to '%s': tip (%.4f, %.4f, %.4f) + %s offset (%+.4f, %+.4f), "
                    "z_offset %+.4f",
            r.refDevice.c_str(), g->name.c_str(), tip.x, tip.y, tip.z, src.c_str(), dx, dy,
            p.GetDouble("z_offset_mm"));
    return ApproachAxisPoint(ctx, p, r.refDevice, rm, refIdx, target, ceiling,
                             "the gripped object");
}

void RegisterGripCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // every action goes through ExecutionContext::Wf() at run time
    (void)hw;

    reg.Register({"grip_profile", "Grip",
                  "Read a gripped-object profile into ctx "
                  "(<prefix>_device/_mode/_marks/_ref_dx..)",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile to read"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "ctx prefix for the profile's fields"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when the profile is gone, instead of reporting "
                       "<prefix>_ok = false"},
                  },
                  CommandSpec::Needs::None, ExecGripProfile});

    reg.Register({"grip_mark", "Grip",
                  "Mark ONE point on the object in the jaws - detected when the "
                  "profile says auto, a crosshair pick when it says manual",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile being measured"},
                      {"index", PT::IntChoice, true, 1, 1, 2,
                       {"pt1 - first edge", "pt2 - opposite edge"},
                       "which mark this is. Auto mode takes one (the detector "
                       "reports the centre); manual takes two, whose midpoint is the "
                       "centre"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "writes <prefix>_p<N>_wx/_wy/_px/_py"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "camera frames to wait for before detecting, so the picture "
                       "is post-move; 0 detects on whatever is there"},
                      {"label", PT::String, false, "", 0, 0, {},
                       "title for the pick window; empty names the profile and the point"},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction shown with the crosshair; empty uses the default"},
                  },
                  CommandSpec::Needs::None, ExecGripMark});

    reg.Register({"grip_apply", "Grip",
                  "Object centre minus the registered tip - the grip offset, with a "
                  "verdict. No motion",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile being measured"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "reads <prefix>_p<N>_*, writes <prefix>_x/_y/_tip_x/_tip_y/"
                       "_dx/_dy/_dist/_hx/_hy/_hz/_ok/_why/_err_x/_err_y"},
                      {"require_ok", PT::Bool, false, true, 0, 0, {},
                       "fail the step on a bad grip. Off reports the verdict in "
                       "<prefix>_ok and lets the recipe decide - an unattended run "
                       "that carries on would place the part with that grip"},
                  },
                  CommandSpec::Needs::None, ExecGripApply});

    // The other direction: the camera over the object. Same parameters as
    // move_camera_to_tool less the device (the profile names it) plus WHICH
    // offset, so an author who knows one knows both.
    reg.Register({"grip_camera_to_object", "Grip",
                  "Drive the CAMERA so its centre lands on the object in the jaws - "
                  "move_camera_to_tool with the grip offset added. The gripper does "
                  "not move",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile; its gripper's tip is the base"},
                      {"offset", PT::Enum, false, "this_run", 0, 0,
                       {"this_run", "reference", "last"},
                       "which grip offset to add to the tip: this_run = what a "
                       "grip_detect / grip_measure step measured earlier in THIS "
                       "recipe (<prefix>_dx/_dy); reference = the profile's adopted "
                       "good grip, the expected place before measuring; last = an "
                       "earlier run's evidence. A missing one fails, never slides to "
                       "the next"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "the prefix this_run reads the offset from"},
                      {"axes", PT::String, false, "XY", 0, 0, {},
                       "which axes of the camera device to command. XY leaves its "
                       "height alone; adding Z drives it to the height that FOCUSES "
                       "on the tip, plus z_offset_mm"},
                      {"z_offset_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the tip's focused Z, only when axes names Z. The "
                       "object is assumed to sit at the tip's height; this is the "
                       "operator's word on the rest. NEGATIVE raises the camera on "
                       "this machine"},
                      {"max_mm", PT::Double, false, 60.0, 0.0, 500.0, {},
                       "refuse a move longer than this; 0 disables the ceiling"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                  },
                  CommandSpec::Needs::None, ExecGripCameraToObject});

    CommandRegistry* rp = &reg;

    // The composite that finds: every mark the profile's saved mode needs, then
    // the verdict - and, on request, the camera onto the tool first. A recipe
    // that has framed the object itself uses this bare; one that wants the
    // gripper posed as well uses grip_measure.
    reg.Register({"grip_detect", "Grip",
                  "Find the object in the jaws - one detection or two crosshair marks, "
                  "per the profile's saved mode - and apply. Moves nothing unless "
                  "'go_to_inspect' or 'look' is on",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile; its mode (manual / detect) decides "
                       "how the object is found"},
                      {"go_to_inspect", PT::Bool, false, false, 0, 0, {},
                       "first drive the GRIPPER to the profile's taught inspect pose "
                       "(move_to_position, with its defaults). That pose carries the "
                       "rotations the reference offset was taught at. A profile with "
                       "no pose taught is measured where it stands, with a warning"},
                      {"look", PT::Bool, false, false, 0, 0, {},
                       "first drive the camera onto the tool's tip (move_camera_to_tool, "
                       "with its defaults). Off expects the object to be in view already"},
                      {"focus_z", PT::Bool, false, false, 0, 0, {},
                       "let that move command Z as well, to the height that focuses on "
                       "the tip. Off by default: it is the leg that drives the lens "
                       "toward a tool of unknown height"},
                      {"z_offset_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "manual offset for the camera's Z on that move, added to the "
                       "tip's focused height - for an object that sits above or below "
                       "the tip. Non-zero commands Z even with focus_z off. NEGATIVE "
                       "raises the camera on this machine (gantry-main's Z is inverted)"},
                      {"settle_ms", PT::Int, false, 300, 0, 60000, {},
                       "pause after the moves before looking; nothing when neither "
                       "is on"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "writes <prefix>_p<N>_* for the marks and everything grip_apply "
                       "writes (_dx/_dy/_hx/_hy/_ok/_why ..)"},
                      {"require_ok", PT::Bool, false, true, 0, 0, {},
                       "fail the step on a bad grip (see grip_apply)"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "camera frames to wait for before detecting, so the picture "
                       "is post-move; 0 detects on whatever is there"},
                      {"label", PT::String, false, "", 0, 0, {},
                       "title for the pick window; empty names the profile and the point"},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction shown with the crosshair; empty uses the default"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      return ExecGripDetect(c, p, *rp);
                  }});

    // The consuming move: the object under the camera. Same parameters as
    // move_tool_to_camera less the device (the profile names it) plus the ctx
    // prefix the offset is read from, so an author who knows one knows both.
    reg.Register({"grip_move_to_camera", "Grip",
                  "APPROACH the point the camera is looking at with the OBJECT in the "
                  "jaws - move_tool_to_camera with this run's grip offset applied",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile; its gripper is what moves"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "reads <prefix>_dx/_dy, the offset a grip_detect or grip_measure "
                       "step left earlier in THIS recipe. Fails when there is none: "
                       "the step never moves on an earlier run's number"},
                      {"z_offset_mm", PT::Double, false, -2.0, -100.0, 100.0, {},
                       "offset from the camera's focused Z, applied to the TIP: the "
                       "object is assumed to sit at the tip's height, so an object "
                       "hanging below the jaws is that much closer than this says. "
                       "NEGATIVE stands the tool off the surface on this machine - "
                       "see move_tool_to_camera"},
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
                  CommandSpec::Needs::None, ExecGripMoveToCamera});

    // The composite: drive to the object, look at it, measure it. One step,
    // because the path is not something a recipe author should have to
    // remember - the same argument tool_travel_to_camera makes, and the same
    // launcher the Gripped Object page's Measure button uses, so the taught
    // procedure and the production one are one code path.
    reg.Register({"grip_measure", "Grip",
                  "Drive the gripper to its inspect pose, put the camera on the tool, "
                  "and measure where the object sits (grip_measure.lua)",
                  {
                      {"profile", PT::GripUid, true, {}, 0, 0, {},
                       "the gripped-object profile to measure against"},
                      {"go_to_inspect", PT::Bool, false, true, 0, 0, {},
                       "drive the gripper to the profile's taught inspect pose first. "
                       "That pose carries the ROTATIONS the reference offset was "
                       "taught at, so a run that skips it may measure a turned tool"},
                      {"look", PT::Bool, false, true, 0, 0, {},
                       "drive the camera onto the tool's tip (move_camera_to_tool). "
                       "Off leaves the camera where it is - for a recipe that has "
                       "already framed the object itself"},
                      {"focus_z", PT::Bool, false, false, 0, 0, {},
                       "let that move command Z as well, to the height that focuses "
                       "on the tip. Off by default: it is the leg that drives the "
                       "lens toward a tool of unknown height"},
                      {"z_offset_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "manual offset for the camera's Z on that move, added to the "
                       "tip's focused height. Non-zero commands Z even with focus_z "
                       "off. NEGATIVE raises the camera on this machine"},
                      {"autofocus", PT::Bool, false, false, 0, 0, {},
                       "sweep for focus once the camera is over the object"},
                      {"focus_range_mm", PT::Double, false, 0.5, 0.01, 20.0, {},
                       "focus sweep range, when autofocus is on"},
                      {"settle_ms", PT::Int, false, 300, 0, 60000, {},
                       "pause after the moves before looking"},
                      {"require_ok", PT::Bool, false, true, 0, 0, {},
                       "fail the step on a bad grip (see grip_apply)"},
                      {"ctx_prefix", PT::String, false, "grip", 0, 0, {},
                       "prefix for every key the run writes"},
                      {"file", PT::ScriptFile, false, "config/scripts/grip_measure.lua",
                       0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("grip_profile_uid", p.GetString("profile"));
                      c.CtxSet("grip_go_to_inspect", p.GetBool("go_to_inspect"));
                      c.CtxSet("grip_look", p.GetBool("look"));
                      c.CtxSet("grip_focus_z", p.GetBool("focus_z"));
                      c.CtxSet("grip_z_offset_mm", p.GetDouble("z_offset_mm"));
                      c.CtxSet("grip_autofocus", p.GetBool("autofocus"));
                      c.CtxSet("grip_focus_range_mm", p.GetDouble("focus_range_mm"));
                      c.CtxSet("grip_settle_ms", static_cast<double>(p.GetInt("settle_ms")));
                      c.CtxSet("grip_require_ok", p.GetBool("require_ok"));
                      c.CtxSet("grip_prefix", p.GetString("ctx_prefix"));
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});
}

} // namespace firedog
