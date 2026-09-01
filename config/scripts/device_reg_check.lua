-- Device registration check: does the record still say where the tip is?
--
-- One measurement. Drive the registered device to a pose, ask the record where
-- its tip should therefore be, drive the camera there, and look.
--
--     residual = | measured tip - t - R * h_check |
--
-- THE CHECK POSE MUST NOT BE THE POSE THE TIP WAS TAUGHT AT. t is derived from
-- the imprint measured at that pose, so predicting there returns the measurement
-- by construction: the residual is about how well a pick repeats and says nothing
-- whatever about the rotation. Hence the offsets below, and hence a check with
-- all three left at zero refuses to call anything verified however small the
-- number comes out.
--
-- This matters more here than it would with a measured rotation, because the
-- rotation is NOT measured - it is the permutation composed out of
-- config/motion_transforms.json, exact only for a device bolted square. This
-- check is the one thing on the machine that ever puts a number on what that
-- assumption costs, and it can only do it at a distance from the taught pose.
--
-- Reading the residual is the whole point, so it is worth knowing what it means:
--
--   a CONSTANT shift, the same wherever you check    -> the TOOL moved.
--        Somebody changed the gripper. Re-teach the tip by imprint on the
--        Registration page: one touch, keeping the rotation.
--   a residual that GROWS with distance from the taught pose -> the ROTATION is
--        wrong, which here means the device is not square to the axes the
--        transforms file claims. t alone cannot absorb an angle; either correct
--        the mount or keep every approach short.
--
-- That split is why the record carries two dates. R belongs to the mount; t
-- belongs to the mount AND the tool, and only one of them changes when a gripper
-- is swapped. See core/DeviceReg.h.
--
-- Parameters (fd.param -> ctx keys, injected by the Registration page's Check
-- button or a device_reg_check_run recipe step):
--   reg_device           the registered device            (required)
--   reg_check_dx/dy/dz_mm  check pose, offset from the taught pose (5, 5, 0)
--   reg_tol_mm           residual that counts as verified (default 0.05)
--   reg_autofocus        focus on the tip                 (default false)
--   reg_focus_range_mm   focus sweep at the check pose    (default 0.5)
--   reg_v_travel         reference device velocity        (default 0 = as set)
--   reg_v_jog            registered device velocity       (default 0 = as set)
--   reg_settle_ms        pause after the move             (default 300)

local DEV    = fd.param("reg_device", "")
local DX     = fd.param("reg_check_dx_mm", 5.0)
local DY     = fd.param("reg_check_dy_mm", 5.0)
local DZ     = fd.param("reg_check_dz_mm", 0.0)
local TOL    = fd.param("reg_tol_mm", 0.05)
local FOCUS  = fd.param("reg_autofocus", false)
local FRANGE = fd.param("reg_focus_range_mm", 0.5)
local VTRAV  = fd.param("reg_v_travel", 0.0)
local VJOG   = fd.param("reg_v_jog", 0.0)
local SETTLE = math.floor(fd.param("reg_settle_ms", 300))

if DEV == "" then
    error("reg_device is not set - name the registered device to check. Launch " ..
          "this from the Registration page or a device_reg_check_run step.")
end

-- require = true: there is nothing to check without a registration, and failing
-- here costs no travel.
fd.device_reg{ device = DEV, ctx_prefix = "reg", require = true }
local REF = fd.ctx.reg_ref
local t   = { x = fd.ctx.reg_tx, y = fd.ctx.reg_ty, z = fd.ctx.reg_tz }
local C = {
    { x = fd.ctx.reg_c1x, y = fd.ctx.reg_c1y, z = fd.ctx.reg_c1z },
    { x = fd.ctx.reg_c2x, y = fd.ctx.reg_c2y, z = fd.ctx.reg_c2z },
    { x = fd.ctx.reg_c3x, y = fd.ctx.reg_c3y, z = fd.ctx.reg_c3z },
}
-- The pose the tip was taught at. The check offsets are measured FROM it, which
-- is the only way this can see a rotation error at all.
local h0 = { x = fd.ctx.reg_tip_x, y = fd.ctx.reg_tip_y, z = fd.ctx.reg_tip_z }

-- p = t + R*h, written as the columns the record handed back. Same arithmetic
-- the C++ side does; here because the script is what decides where to drive.
local function predict(h)
    return { x = t.x + h.x * C[1].x + h.y * C[2].x + h.z * C[3].x,
             y = t.y + h.x * C[1].y + h.y * C[2].y + h.z * C[3].y,
             z = t.z + h.x * C[1].z + h.y * C[2].z + h.z * C[3].z }
end

local offset = math.sqrt(DX * DX + DY * DY + DZ * DZ)
local tests_rotation = offset > 0.0
if not tests_rotation then
    fd.log("WARNING: the check offsets are all zero, so this checks the tip at the " ..
           "very pose its tip was taught at. That measures whether a pick repeats, " ..
           "not whether the rotation is right, and it cannot verify anything - set " ..
           "check_dx/dy/dz to move away from the taught pose.")
end

local hc = { x = h0.x + DX, y = h0.y + DY, z = h0.z + DZ }
local pred = predict(hc)
fd.logf("check: '%s' to (%.4f, %.4f, %.4f), %.3f mm from where its tip was taught; the " ..
        "record puts its tip at (%.4f, %.4f, %.4f) in '%s' space",
        DEV, hc.x, hc.y, hc.z, offset, pred.x, pred.y, pred.z, REF)

if not fd.prompt(string.format(
        "About to move '%s' to (%.3f, %.3f, %.3f) and follow with '%s' to (%.3f, " ..
        "%.3f, %.3f). Check both have room. OK to move.",
        DEV, hc.x, hc.y, hc.z, REF, pred.x, pred.y, pred.z)) then
    error("cancelled by operator")
end

fd.move_axis_to{ device = DEV, axes = "XYZ", x_mm = hc.x, y_mm = hc.y, z_mm = hc.z,
                 velocity_mm_s = VJOG }
if SETTLE > 0 then fd.sleep(SETTLE) end

fd.move_axis_to{ device = REF, axes = "XYZ", x_mm = pred.x, y_mm = pred.y, z_mm = pred.z,
                 velocity_mm_s = VTRAV }
if FOCUS then
    fd.auto_focus{ device = REF, axis = "Z", z_range_mm = FRANGE }
end

-- With the autofocus off, the operator sets Z by hand and the window's live
-- readout is what they set it against - so the message has to say so. An
-- autofocus that ran already put Z where the sweep peaked and the pick is only
-- about XY.
local how
if FOCUS then
    how = "Put the crosshair on the tool tip, then OK."
else
    how = "Focus on the tool tip by hand (jog Z, tick the live focus score), " ..
          "watch dZ close, then put the crosshair on the tip and OK."
end

-- The hint is the prediction, so an operator who cannot see the tip is told how
-- far off it is and in which direction - which IS the measurement, before it is
-- taken. The TARGET is the same point again, and it is not the same thing: the
-- hint is drawn on the picture, this is the residual in numbers, Z included -
-- and Z is the half a camera cannot show. Without it a hand focus is blind.
fd.pick_point{ device = REF, ctx_prefix = "tip",
               label = string.format("the tool tip of '%s'", DEV),
               message = how,
               hint = true, hint_x_mm = pred.x, hint_y_mm = pred.y,
               target = true, target_x_mm = pred.x, target_y_mm = pred.y,
               target_z_mm = pred.z, target_tol_mm = TOL }
fd.read_position{ device = REF, ctx_prefix = "g" }
local meas = { x = fd.ctx.tip_wx, y = fd.ctx.tip_wy, z = fd.ctx.g_z }

local ex, ey, ez = meas.x - pred.x, meas.y - pred.y, meas.z - pred.z
local residual = math.sqrt(ex * ex + ey * ey + ez * ez)
fd.logf("check: tip measured at (%.4f, %.4f, %.4f) - %.4f mm from prediction " ..
        "(%+.4f, %+.4f, %+.4f)", meas.x, meas.y, meas.z, residual, ex, ey, ez)

-- Home before writing anything. If the write is refused, the device is still
-- left where the operator set it up rather than out at the check pose.
fd.move_axis_to{ device = DEV, axes = "XYZ", x_mm = h0.x, y_mm = h0.y, z_mm = h0.z,
                 velocity_mm_s = VJOG }

local verified = tests_rotation and (residual <= TOL)
fd.device_reg_check_save{ device = DEV, residual_mm = residual, verified = verified,
                          pose_x_mm = hc.x, pose_y_mm = hc.y, pose_z_mm = hc.z }

if verified then
    fd.logf("'%s' verified: %.4f mm at %.3f mm from the taught pose, inside %.4f mm",
            DEV, residual, offset, TOL)
    return
end

if not tests_rotation then
    -- Recorded, deliberately not failed: the operator asked for a repeatability
    -- measurement and got one. Refusing work that was explicitly requested would
    -- be wrong; letting it read as a verification would be worse.
    fd.logf("WARNING: '%s' measured %.4f mm at the very pose its tip was taught at. That is a " ..
            "repeatability figure, not a verification - it is recorded as such and " ..
            "the registration stays unverified.", DEV, residual)
    return
end

error(string.format(
    "'%s' is %.4f mm out at %.3f mm from where its tip was taught, outside the " ..
    "%.4f mm tolerance. A shift of the same size wherever you check is the TOOL - " ..
    "re-teach the tip by imprint. A miss that grows with distance is the ROTATION, " ..
    "which came from the axis transforms and is only as square as the mount. " ..
    "Loosening the tolerance fixes neither.",
    DEV, residual, offset, TOL))
