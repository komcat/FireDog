// CommandsNeedle.cpp : the dispense-needle command family.
//
// Four commands, and between them they do almost nothing — which is the point.
// The needle calibration and the dispense sequence are LUA procedures
// (config/scripts/needle_calibration.lua, config/scripts/dispense_procedure.lua),
// built out of the motion, IO, pneumatic and prompt commands that already exist.
// What was missing was not a routine; it was a place to keep the one number
// those routines need, and two typed steps to launch them from a recipe.
//
//   needle_offset        put the calibrated lever arm into ctx
//   needle_offset_save   take a freshly measured one back
//   needle_cal_run       run the calibration procedure as one recipe step
//   dispense_run         run the dispense procedure as one recipe step
//
// The sibling project (bobcat_engine) built both procedures as C++ plugins
// emitting primitive actions, and paid for it in an unusual way: its sequence
// language had no arithmetic on literals, so a step that meant
// "previous_z minus 1 mm" silently computed "previous_z minus 0" and drove the
// needle onto the surface at 5 mm/s instead of parking a millimetre above it.
// Lua has real numbers. Keeping the arithmetic in the script is not a stylistic
// preference here — it is where that entire class of bug went.
//
// See core/NeedleCal.h for why the offset is machine-level, why it lives in axis
// space, and why nothing in this family assumes which way Z points.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"   // RunLuaFile (needle_cal_run / dispense_run)
#include "core/Log.h"
#include "core/NeedleCal.h"
#include "core/Product.h"        // NowStamp
#include "workflow/Workflow.h"

#include <cmath>
#include <string>

namespace firedog {

namespace {

using PT = ParamType;

// ---- needle_offset ---------------------------------------------------------

CommandResult ExecNeedleOffset(ExecutionContext& ctx, const ParamValues& p)
{
    const NeedleCal c = ctx.Wf().Needle();
    const std::string prefix = p.GetString("ctx_prefix");

    // Write the keys FIRST, valid or not: a script that checks <prefix>_ok has
    // to be able to read it even on the failing path, and `require` is only a
    // shortcut for the check.
    ctx.CtxSet(prefix + "_ok", c.valid);
    ctx.CtxSet(prefix + "_dx", c.dx);
    ctx.CtxSet(prefix + "_dy", c.dy);
    ctx.CtxSet(prefix + "_dz", c.dz);
    ctx.CtxSet(prefix + "_down", NeedleDownSign(c));
    ctx.CtxSet(prefix + "_device", c.device);
    ctx.CtxSet(prefix + "_slide", c.slide);
    ctx.CtxSet(prefix + "_io", c.ioDevice);
    ctx.CtxSet(prefix + "_pin", c.shotPin);
    ctx.CtxSet(prefix + "_at", c.calibratedAt);
    ctx.CtxSet(prefix + "_verified", c.verified);
    ctx.CtxSet(prefix + "_residual_mm", c.residualMm);
    ctx.CtxSet(prefix + "_z_residual_mm", c.zResidualMm);
    ctx.CtxSet(prefix + "_z_checked", c.zChecked);

    if (!c.valid && p.GetBool("require"))
        return CommandResult::Fail(
            "no needle calibration - run the needle calibration procedure "
            "(Needle page, or a needle_cal_run step) before dispensing. Until "
            "then nothing knows where the needle is relative to the camera.");

    if (c.valid)
        FD_INFO("needle", "needle offset (%.4f, %.4f, %.4f) mm on %s%s",
                c.dx, c.dy, c.dz, c.device.c_str(),
                c.verified      ? ""
                : c.needsZCheck ? " (UNVERIFIED - checked in XY only, re-calibrate to check dZ)"
                                : " (UNVERIFIED - single-pass calibration)");
    return CommandResult::Success();
}

// ---- needle_offset_save ----------------------------------------------------

CommandResult ExecNeedleOffsetSave(ExecutionContext& ctx, const ParamValues& p)
{
    NeedleCal c;
    c.valid    = true;
    c.device   = p.GetString("device");
    c.slide    = p.GetString("slide");
    c.ioDevice = p.GetString("io_device");
    c.shotPin  = p.GetString("shot_pin");

    c.dx = p.GetDouble("dx_mm");
    c.dy = p.GetDouble("dy_mm");
    c.dz = p.GetDouble("dz_mm");

    c.camX = p.GetDouble("cam_x_mm");
    c.camY = p.GetDouble("cam_y_mm");
    c.camZ = p.GetDouble("cam_z_mm");
    c.ndlX = p.GetDouble("ndl_x_mm");
    c.ndlY = p.GetDouble("ndl_y_mm");
    c.ndlZ = p.GetDouble("ndl_z_mm");
    c.dotX = p.GetDouble("dot_x_mm");
    c.dotY = p.GetDouble("dot_y_mm");

    c.residualMm  = p.GetDouble("residual_mm");
    c.zResidualMm = p.GetDouble("z_residual_mm");
    c.zChecked    = p.GetBool("z_checked");
    // Verified means BOTH halves were re-measured. A caller claiming it
    // without a Z check would write a record that reloads as unverified - so
    // it is not verified now either.
    c.verified   = p.GetBool("verified") && c.zChecked;
    if (p.GetBool("verified") && !c.zChecked)
        FD_WARN("needle", "needle_offset_save: verified without z_checked - stored "
                          "UNVERIFIED (a verify pass must check dZ as well)");
    c.calibratedAt = NowStamp();

    // A zero dz means the needle touches at the camera's own focal height, which
    // no dispenser does. It is what an uninitialised variable looks like, and it
    // is also what makes NeedleDownSign meaningless — so refuse it here rather
    // than let a two-stage descent pick a direction by coin toss.
    if (c.dz == 0.0)
        return CommandResult::Fail("dz_mm is 0 - the needle cannot touch at the "
                                   "camera's focal plane; the Z touch-off was not "
                                   "captured");

    // Sanity, not policy: the point of the calibration is that the needle is
    // somewhere ELSE. An offset of a few microns means the two poses came from
    // the same reading, which is a script bug, not a very well aligned machine.
    if (std::sqrt(c.dx * c.dx + c.dy * c.dy) < 0.05)
        return CommandResult::Fail("the camera and needle poses are within 50 um of "
                                   "each other - one of them was not actually captured");

    const NeedleCal old = ctx.Wf().Needle();

    // The engine worker does NOT write the file. Same rule as a taught frame: the
    // UI thread owns persistence, so a run that is stopped mid-way cannot leave a
    // half-written calibration behind.
    ctx.Wf().QueueNeedleCal(c);

    if (old.valid)
        FD_INFO("needle", "needle offset %s: (%.4f, %.4f, %.4f) -> (%.4f, %.4f, %.4f) mm "
                          "(delta %+.4f, %+.4f, %+.4f)",
                c.verified ? "re-calibrated" : "re-calibrated, UNVERIFIED",
                old.dx, old.dy, old.dz, c.dx, c.dy, c.dz,
                c.dx - old.dx, c.dy - old.dy, c.dz - old.dz);
    else
        FD_INFO("needle", "needle offset calibrated: (%.4f, %.4f, %.4f) mm on %s%s",
                c.dx, c.dy, c.dz, c.device.c_str(),
                c.verified ? "" : " (UNVERIFIED)");
    return CommandResult::Success();
}

}   // namespace

void RegisterNeedleCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;
    (void)hw;

    reg.Register({"needle_offset", "Needle",
                  "Read the calibrated camera->needle lever arm into ctx "
                  "(<prefix>_ok/_dx/_dy/_dz/_down)",
                  {
                      {"ctx_prefix", PT::String, false, "needle", 0, 0, {},
                       "ctx prefix for the offset"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when no calibration exists, instead of "
                       "reporting <prefix>_ok = false"},
                  },
                  CommandSpec::Needs::None, ExecNeedleOffset});

    reg.Register({"needle_offset_save", "Needle",
                  "Store a freshly measured camera->needle lever arm as THE machine "
                  "calibration",
                  {
                      {"dx_mm", PT::Double, true, {}, 0, 0, {},
                       "add to a camera X to get the pose that puts the needle there"},
                      {"dy_mm", PT::Double, true, {}, 0, 0, {}, "the same for Y"},
                      {"dz_mm", PT::Double, true, {}, 0, 0, {},
                       "add to a FOCUSED camera Z to get the needle's touch Z; its "
                       "sign is what tells every later descent which way is down"},
                      {"device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the device these axis coordinates belong to"},
                      {"slide", PT::Slide, false, "Dispenser_Head", 0, 0, {},
                       "pneumatic slide that puts the needle down"},
                      {"io_device", PT::DeviceIo, false, "IOBottom", 0, 0, {},
                       "IO device carrying the glue valve"},
                      {"shot_pin", PT::PinName, false, "Dispenser_Shot", 0, 0, {},
                       "output pin that fires a shot", PinDir::Output},
                      {"verified", PT::Bool, false, false, 0, 0, {},
                       "a second pass re-measured this and agreed in XY AND Z "
                       "(ignored without z_checked)",
                       PinDir::Any, "Record"},
                      {"residual_mm", PT::Double, false, 0.0, 0, 0, {},
                       "how far the verify pass had to move the XY answer",
                       PinDir::Any, "Record"},
                      {"z_checked", PT::Bool, false, false, 0, 0, {},
                       "a verify pass drove to the predicted touch Z and measured "
                       "how far off it was",
                       PinDir::Any, "Record"},
                      {"z_residual_mm", PT::Double, false, 0.0, 0, 0, {},
                       "real touch Z minus predicted, signed along down (+ = touched "
                       "lower than predicted)",
                       PinDir::Any, "Record"},
                      {"cam_x_mm", PT::Double, false, 0.0, 0, 0, {}, "camera pose X", PinDir::Any, "Record"},
                      {"cam_y_mm", PT::Double, false, 0.0, 0, 0, {}, "camera pose Y", PinDir::Any, "Record"},
                      {"cam_z_mm", PT::Double, false, 0.0, 0, 0, {}, "camera pose Z (focused)", PinDir::Any, "Record"},
                      {"ndl_x_mm", PT::Double, false, 0.0, 0, 0, {}, "imprint pose X", PinDir::Any, "Record"},
                      {"ndl_y_mm", PT::Double, false, 0.0, 0, 0, {}, "imprint pose Y", PinDir::Any, "Record"},
                      {"ndl_z_mm", PT::Double, false, 0.0, 0, 0, {}, "imprint pose Z (touching)", PinDir::Any, "Record"},
                      {"dot_x_mm", PT::Double, false, 0.0, 0, 0, {}, "the imprint, world X", PinDir::Any, "Record"},
                      {"dot_y_mm", PT::Double, false, 0.0, 0, 0, {}, "the imprint, world Y", PinDir::Any, "Record"},
                  },
                  CommandSpec::Needs::None, ExecNeedleOffsetSave});

    // The two script launchers. Same shape as align_run: set the ctx
    // the script reads with fd.param, resolve the file, run it inline on this
    // worker. One step in a recipe, one button on a page, one file.
    CommandRegistry* rp = &reg;

    reg.Register({"needle_cal_run", "Needle",
                  "Run the whole needle calibration (needle_calibration.lua)",
                  {
                      {"device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the gantry carrying both the camera and the needle"},
                      {"slide", PT::Slide, false, "Dispenser_Head", 0, 0, {},
                       "pneumatic slide that puts the needle down"},
                      {"io_device", PT::DeviceIo, false, "IOBottom", 0, 0, {},
                       "IO device carrying the glue valve"},
                      {"shot_pin", PT::PinName, false, "Dispenser_Shot", 0, 0, {},
                       "output pin that fires a shot", PinDir::Output},
                      {"shot_ms", PT::Int, false, 300, 1, 60000, {},
                       "valve open time for the calibration imprint"},
                      {"settle_ms", PT::Int, false, 500, 0, 60000, {},
                       "pause after the slide extends, before touching down"},
                      {"clear_mm", PT::Double, false, 1.0, 0.0, 20.0, {},
                       "safety height: on a verify pass the fast approach stops this "
                       "far above the gap and v_final takes over"},
                      {"first_clear_mm", PT::Double, false, 3.0, 0.0, 20.0, {},
                       "pass 1 parks this far above the PREVIOUS touch height before "
                       "the operator jogs down - the needle may have been changed"},
                      {"verify_gap_mm", PT::Double, false, 0.1, 0.0, 2.0, {},
                       "a verify pass stops this far above the PREDICTED touch "
                       "height; the operator closes the gap, and how far they jog is "
                       "the Z residual"},
                      {"lift_mm", PT::Double, false, 1.5, 0.0, 20.0, {},
                       "after the shot, lift Z this far off the dot before the slide "
                       "comes up"},
                      {"autofocus", PT::Bool, false, true, 0, 0, {},
                       "focus the camera on the coupon, so dz is measured from a "
                       "focal plane and not from wherever Z happened to be"},
                      {"passes", PT::Int, false, 2, 1, 5, {},
                       "1 imprints once and cannot check itself; 2+ drives to the "
                       "calibrated touch height and checks XY and Z"},
                      {"tol_mm", PT::Double, false, 0.05, 0.001, 5.0, {},
                       "XY residual a verify pass has to land inside"},
                      {"z_tol_mm", PT::Double, false, 0.05, 0.001, 5.0, {},
                       "Z residual a verify pass has to land inside"},
                      {"v_traverse", PT::Double, false, 40.0, 0.001, 100.0, {},
                       "XY traverse: needle over the spot, camera back over the dot"},
                      {"v_approach", PT::Double, false, 20.0, 0.001, 100.0, {},
                       "Z velocity down to the safety height"},
                      {"v_final", PT::Double, false, 5.0, 0.001, 100.0, {},
                       "Z velocity from the safety height to the verify gap"},
                      {"v_liftoff", PT::Double, false, 5.0, 0.001, 100.0, {},
                       "Z velocity lifting off the dot after the shot"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/needle_calibration.lua", 0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file - point it at a different name "
                       "to run something else"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("ndl_device", p.GetString("device"));
                      c.CtxSet("ndl_slide", p.GetString("slide"));
                      c.CtxSet("ndl_io", p.GetString("io_device"));
                      c.CtxSet("ndl_pin", p.GetString("shot_pin"));
                      c.CtxSet("ndl_shot_ms", static_cast<double>(p.GetInt("shot_ms")));
                      c.CtxSet("ndl_settle_ms", static_cast<double>(p.GetInt("settle_ms")));
                      c.CtxSet("ndl_clear_mm", p.GetDouble("clear_mm"));
                      c.CtxSet("ndl_first_clear_mm", p.GetDouble("first_clear_mm"));
                      c.CtxSet("ndl_verify_gap_mm", p.GetDouble("verify_gap_mm"));
                      c.CtxSet("ndl_lift_mm", p.GetDouble("lift_mm"));
                      c.CtxSet("ndl_autofocus", p.GetBool("autofocus"));
                      c.CtxSet("ndl_passes", static_cast<double>(p.GetInt("passes")));
                      c.CtxSet("ndl_tol_mm", p.GetDouble("tol_mm"));
                      c.CtxSet("ndl_z_tol_mm", p.GetDouble("z_tol_mm"));
                      c.CtxSet("ndl_v_traverse", p.GetDouble("v_traverse"));
                      c.CtxSet("ndl_v_approach", p.GetDouble("v_approach"));
                      c.CtxSet("ndl_v_final", p.GetDouble("v_final"));
                      c.CtxSet("ndl_v_liftoff", p.GetDouble("v_liftoff"));
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});

    // Deliberately NOT a system script, unlike the calibration above. Where the
    // needle is, is a fact about the machine and every product must measure it
    // the same way. How much glue goes where is a fact about the PRODUCT — dot
    // count, dwell, whether the operator confirms — so a product's own
    // dispense_procedure.lua has to keep winning over the shipped one.
    reg.Register({"dispense_run", "Needle",
                  "Dispense at a taught camera position (dispense_procedure.lua)",
                  {
                      // Two ways to say where, and a step holds exactly one:
                      // the position picker is the choice (its empty entry is
                      // "custom point"), the editor clears the other when
                      // either is set, and the step fails before moving if it
                      // holds both or none.
                      {"position", PT::PositionUid, false, "", 0, 0, {},
                       "where the CAMERA stands over the dispense point: a saved "
                       "position (when several steps share it), or 'custom point' "
                       "to teach the pose into this step below"},
                      {"point", PT::PointXyz, false, {}, 0, 0, {},
                       "the custom camera pose, taught into this step (Teach here) - "
                       "the needle offset is what turns it into a needle move"},
                      {"device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                       "the gantry carrying both the camera and the needle"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/dispense_procedure.lua", 0, 0, {},
                       "script override; a product's own copy of this name wins"},
                      // Everything below is edited in the Dispense helper,
                      // against the needle motion drawing - in the order of the
                      // motion: 1 traverse, 2-4 down, 5-7 shot and up, 8 back.
                      {"v_travel", PT::Double, false, 0.0, 0.0, 100.0, {},
                       "1, 8: XY travel velocity; 0 = leave the device where it is set",
                       PinDir::Any, kDispenseHelper},
                      {"adjust", PT::Bool, false, true, 0, 0, {},
                       "after arriving, pause on a manual adjust (X/Y only) so the "
                       "operator can centre the camera on the point; the dot goes "
                       "where they leave it. This run only - the taught point is "
                       "not rewritten",
                       PinDir::Any, kDispenseHelper},
                      {"autofocus", PT::Bool, false, false, 0, 0, {},
                       "focus at the point and take the dispense height from THAT "
                       "focal plane instead of the position's taught Z",
                       PinDir::Any, kDispenseHelper},
                      {"confirm", PT::Bool, false, true, 0, 0, {},
                       "ask the operator before extending (2), before firing (5) and "
                       "whether the dot looks right (8)",
                       PinDir::Any, kDispenseHelper},
                      {"v_approach", PT::Double, false, 20.0, 0.001, 100.0, {},
                       "3: Z velocity down to the safety height",
                       PinDir::Any, kDispenseHelper},
                      {"safety_mm", PT::Double, false, 1.0, 0.0, 20.0, {},
                       "height above the target where the fast descent hands over to "
                       "the slow one",
                       PinDir::Any, kDispenseHelper},
                      {"v_final", PT::Double, false, 5.0, 0.001, 100.0, {},
                       "4: Z velocity for the last stretch onto the part",
                       PinDir::Any, kDispenseHelper},
                      {"gap_mm", PT::Double, false, 0.0, 0.0, 10.0, {},
                       "dispense standoff: how far ABOVE the calibrated touch height "
                       "the needle stops, letting the bead bridge the gap. Distinct "
                       "from z_trim_mm, which corrects the calibration itself",
                       PinDir::Any, kDispenseHelper},
                      {"z_trim_mm", PT::Double, false, 0.0, -5.0, 5.0, {},
                       "adjust the dispense height: + moves the needle CLOSER to the "
                       "part, whichever way Z counts on this gantry",
                       PinDir::Any, kDispenseHelper},
                      {"before_ms", PT::Int, false, 0, 0, 60000, {},
                       "settle after arriving at height, before the valve opens",
                       PinDir::Any, kDispenseHelper},
                      {"shot_ms", PT::Int, false, 300, 1, 60000, {},
                       "5: valve open time - this IS the shot size; there is no "
                       "pressure control in the loop",
                       PinDir::Any, kDispenseHelper},
                      {"after_ms", PT::Int, false, 0, 0, 60000, {},
                       "dwell after the valve closes, before lifting (string break)",
                       PinDir::Any, kDispenseHelper},
                      {"lift_mm", PT::Double, false, 1.5, 0.0, 20.0, {},
                       "6: after the shot, lift Z this far off the dot BEFORE the "
                       "slide comes up (0 = slide first)",
                       PinDir::Any, kDispenseHelper},
                      {"v_liftoff", PT::Double, false, 5.0, 0.001, 100.0, {},
                       "6, 8: Z velocity lifting off the dot and climbing away; "
                       "v_final is for arriving on a part, not for leaving one",
                       PinDir::Any, kDispenseHelper},
                      {"verify", PT::Bool, false, true, 0, 0, {},
                       "8: bring the camera back over the dot afterwards",
                       PinDir::Any, kDispenseHelper},
                      {"capture", PT::Bool, false, true, 0, 0, {},
                       "attach an image of the result to the open DUT",
                       PinDir::Any, kDispenseHelper},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      const bool hasPos   = !p.GetString("position").empty();
                      const bool hasPoint = p.Has("point");
                      if (hasPos && hasPoint)
                          return CommandResult::Fail(
                              "dispense_run holds both a taught point and a saved "
                              "position - clear one");
                      if (!hasPos && !hasPoint)
                          return CommandResult::Fail(
                              "dispense_run has no dispense point - pick a saved "
                              "position, or 'custom point' and Teach here");
                      // Every key set on every run, point or not: a recipe
                      // shares one ctx, and a previous step's disp_x must not
                      // survive into a step that names a position.
                      c.CtxSet("disp_point", hasPoint);
                      const nlohmann::json pt =
                          hasPoint ? p.Raw().at("point") : nlohmann::json::object();
                      c.CtxSet("disp_x", hasPoint ? pt.at("X").get<double>() : 0.0);
                      c.CtxSet("disp_y", hasPoint ? pt.at("Y").get<double>() : 0.0);
                      c.CtxSet("disp_z", hasPoint ? pt.at("Z").get<double>() : 0.0);
                      c.CtxSet("disp_position", p.GetString("position"));
                      c.CtxSet("disp_device", p.GetString("device"));
                      c.CtxSet("disp_shot_ms", static_cast<double>(p.GetInt("shot_ms")));
                      c.CtxSet("disp_before_ms", static_cast<double>(p.GetInt("before_ms")));
                      c.CtxSet("disp_after_ms", static_cast<double>(p.GetInt("after_ms")));
                      c.CtxSet("disp_z_trim_mm", p.GetDouble("z_trim_mm"));
                      c.CtxSet("disp_gap_mm", p.GetDouble("gap_mm"));
                      c.CtxSet("disp_safety_mm", p.GetDouble("safety_mm"));
                      c.CtxSet("disp_adjust", p.GetBool("adjust"));
                      c.CtxSet("disp_autofocus", p.GetBool("autofocus"));
                      c.CtxSet("disp_confirm", p.GetBool("confirm"));
                      c.CtxSet("disp_verify", p.GetBool("verify"));
                      c.CtxSet("disp_capture", p.GetBool("capture"));
                      c.CtxSet("disp_v_travel", p.GetDouble("v_travel"));
                      c.CtxSet("disp_v_approach", p.GetDouble("v_approach"));
                      c.CtxSet("disp_v_final", p.GetDouble("v_final"));
                      c.CtxSet("disp_v_liftoff", p.GetDouble("v_liftoff"));
                      c.CtxSet("disp_lift_mm", p.GetDouble("lift_mm"));
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  },
                  /*parallelBy*/ "", /*helper*/ kDispenseHelper});
}

} // namespace firedog
