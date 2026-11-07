-- Device registration teach pose: the tool at device ZERO, the camera over it.
--
-- One button on the Registration page's Teach Pose tab runs this. Three steps:
--
--   1  the operator confirms the registered device may move, and it is driven
--      to axis zero - X Y Z U V W all 0;
--   2  the operator jogs the camera's device (gantry-main) until the tool is in
--      the picture and FOCUSED on it;
--   3  device_reg_teach_save latches both poses into the record: the device
--      pose (zero) as the pose the tip is taught at, the camera pose as the
--      view. The focused camera Z IS the tip's height.
--
-- Marking the tool's corners and computing the tip is the NEXT tab (Capture
-- tool points). The device must still be at zero when that is done - the page
-- refuses to compute a tip from a device that has moved off its taught pose.
--
-- Why zero: the record's `t` is defined as the tip at device pose (0,0,0), so a
-- tip measured there IS t. Nothing is carried through the rotation, which is a
-- typed permutation, and every station has the same pose to come back to after
-- a tool change.
--
-- Parameters (fd.param -> ctx keys, injected by the Registration page):
--   reg_device     the registered device              (required)
--   reg_ref        the camera's device                (default: the record's)
--   reg_v_jog      registered device velocity         (default 0 = as set)
--   reg_step_um    first jog step for the camera      (default 50)
--   reg_settle_ms  pause after the move to zero       (default 300)

local DEV    = fd.param("reg_device", "")
local REF    = fd.param("reg_ref", "")
local VJOG   = fd.param("reg_v_jog", 0.0)
local STEP   = fd.param("reg_step_um", 50.0)
local SETTLE = math.floor(fd.param("reg_settle_ms", 300))

if DEV == "" then
    error("reg_device is not set - name the device to teach. Launch this from the " ..
          "Registration page.")
end
if REF == "" then
    fd.device_reg{ device = DEV, ctx_prefix = "reg" }
    REF = fd.ctx.reg_ref
    if REF == nil or REF == "" then REF = "gantry-main" end
end
if REF == DEV then
    error("a device cannot be registered to itself")
end

-- ---- 1  device to zero ------------------------------------------------------
fd.read_position{ device = DEV, ctx_prefix = "h0" }
local function n(v) return v or 0.0 end
fd.logf("teach: '%s' now at X %.4f  Y %.4f  Z %.4f  U %.4f  V %.4f  W %.4f",
        DEV, n(fd.ctx.h0_x), n(fd.ctx.h0_y), n(fd.ctx.h0_z),
        n(fd.ctx.h0_u), n(fd.ctx.h0_v), n(fd.ctx.h0_w))

if not fd.prompt(string.format(
        "About to move '%s' to ZERO: X Y Z = 0 and U V W = 0.\n\n" ..
        "It is now at X %.3f  Y %.3f  Z %.3f  U %.3f  V %.3f  W %.3f.\n\n" ..
        "Check the gripper has room to get there - nothing under it, and '%s' " ..
        "is clear of it. OK to move.",
        DEV, n(fd.ctx.h0_x), n(fd.ctx.h0_y), n(fd.ctx.h0_z),
        n(fd.ctx.h0_u), n(fd.ctx.h0_v), n(fd.ctx.h0_w), REF), 0, "warning") then
    error("cancelled by operator")
end

-- U/V/W only if the device has them; a device without rotations still has a
-- zero, and move_axis_to refuses an axis the device does not report.
local axes = "XYZ"
if fd.ctx.h0_u ~= nil and fd.ctx.h0_v ~= nil and fd.ctx.h0_w ~= nil then
    axes = "XYZUVW"
end
fd.move_axis_to{ device = DEV, axes = axes, x_mm = 0, y_mm = 0, z_mm = 0,
                 u_deg = 0, v_deg = 0, w_deg = 0, velocity_mm_s = VJOG }
if SETTLE > 0 then fd.sleep(SETTLE) end
fd.logf("teach: '%s' at zero (%s)", DEV, axes)

-- ---- 2  camera over the tool -------------------------------------------------
-- Cancel fails the step, which ends the run with nothing saved - the record
-- keeps whatever pose it had.
fd.manual_adjust{
    device  = REF,
    step_um = STEP,
    message = string.format(
        "'%s' is at zero. Jog '%s' until the gripper tip is in the picture, " ..
        "then FOCUS on it - the focused Z is the tip's height. OK saves this " ..
        "camera pose.", DEV, REF),
}

-- ---- 3  save -----------------------------------------------------------------
fd.device_reg_teach_save{ device = DEV, ref_device = REF, ctx_prefix = "teach" }
fd.logf("teach: camera view (%.4f, %.4f, %.4f) on '%s' saved for '%s' - now mark the " ..
        "tool on the Capture tool points tab, then Compute and save tip",
        fd.ctx.teach_view_x, fd.ctx.teach_view_y, fd.ctx.teach_view_z, REF, DEV)
