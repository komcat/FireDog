-- Device registration: measure where another device's tool tip is, in camera space
--
-- The needle calibration measures a lever arm INSIDE one device. This measures a
-- relationship BETWEEN two: the gripper on a hexapod and the camera on the
-- gantry have never shared an origin, which is why nothing today can say where a
-- gripper is except immediately after looking at it.
--
--     p_ref = R * h + t
--
-- `h` is the registered device's axis XYZ, `p_ref` the reference (camera) device's.
-- Four measurements give both:
--
--     P0 = the tip at rest                     c1 = (P1 - P0) / L
--     jog the device +L on X  -> P1            c2 = (P2 - P0) / L
--     jog the device +L on Y  -> P2            c3 = (P3 - P0) / L
--     jog the device +L on Z  -> P3            t  = P0 - R * h0
--
-- Differences kill t, so each jog hands back that axis's direction directly.
-- That is the same trick the needle procedure uses to cancel the unknown; here
-- it is what makes a rotation measurable at all.
--
-- Four things about this file are deliberate:
--
--  1. R IS MEASURED, NOT TAKEN FROM config/motion_transforms.json. That file's
--     matrices are hand-typed permutations of +-1 - exact for a device bolted
--     square, and quietly wrong for one that is not, in a way that predicts
--     perfectly where it was taught and misses by a millimetre 30 mm away. The
--     typed matrix is used here only to AIM the camera between jogs, which is
--     all a couple of degrees is good for.
--
--  2. THE CAMERA FOLLOWS THE TIP, and the prediction is what lets it. After each
--     jog the tip is L away - far outside a 6 mm field of view - so the script
--     drives the reference device to where the aiming columns say the tip went,
--     and the operator only has to click it. That is why L is bounded by travel
--     and clearance rather than by the field of view, and why longer is better:
--     10 mm picked to 5 um resolves about 0.03 deg.
--
--  3. NOTHING IS ASSUMED ABOUT HANDEDNESS OR DIRECTION. Every matrix in the
--     shipped transforms file has det = -1. The fit takes the sign from the
--     measurement; this file just measures.
--
--  4. THE REFUSALS ARE IN C++, NOT HERE. device_reg_save runs the fit and
--     refuses a column that did not travel its commanded distance, a skewed
--     triad, or a Z that disagrees with X cross Y. A gate that lived in this
--     file would be a gate a forked copy could delete - which is also why this
--     file is a STANDARD procedure and a product copy of it is ignored.
--
-- Parameters (fd.param -> ctx keys, injected by the Registration page's Register
-- button or a device_reg_run recipe step):
--   reg_device           the device carrying the tip      (required)
--   reg_ref              the device carrying the camera   (default "gantry-main")
--   reg_jog_mm           per-axis jog between poses       (default 10)
--   reg_autofocus        focus on the tip at every pose   (default true)
--   reg_focus_range_mm   focus sweep after a jog          (default 1.0)
--   reg_v_travel         reference device velocity        (default 0 = as set)
--   reg_v_jog            registered device velocity       (default 0 = as set)
--   reg_settle_ms        pause after each jog             (default 300)

local DEV    = fd.param("reg_device", "")
local REF    = fd.param("reg_ref", "gantry-main")
local L      = fd.param("reg_jog_mm", 10.0)
local FOCUS  = fd.param("reg_autofocus", true)
local FRANGE = fd.param("reg_focus_range_mm", 1.0)
local VTRAV  = fd.param("reg_v_travel", 0.0)
local VJOG   = fd.param("reg_v_jog", 0.0)
local SETTLE = math.floor(fd.param("reg_settle_ms", 300))

if DEV == "" then
    error("reg_device is not set - name the device whose tool tip is being " ..
          "registered. Launch this from the Registration page or a device_reg_run step.")
end
if DEV == REF then
    error("reg_device and reg_ref are both '" .. DEV .. "' - a device cannot be " ..
          "registered to itself; reg_ref names the device the CAMERA rides on")
end
if L < 1.0 then
    error(string.format("reg_jog_mm of %.3f is too small - under a millimetre the " ..
                        "pick noise dominates the direction and the angles are " ..
                        "meaningless", L))
end

-- The aiming columns. With a registration these ARE the measured rotation; with
-- none they come from the axis transforms and are good to a degree or two, which
-- over a 10 mm jog is a few hundred microns - well inside the field of view.
fd.device_reg{ device = DEV, ref_device = REF, ctx_prefix = "aim" }
local seeded = fd.ctx.aim_seeded
local C = {
    { x = fd.ctx.aim_c1x, y = fd.ctx.aim_c1y, z = fd.ctx.aim_c1z },
    { x = fd.ctx.aim_c2x, y = fd.ctx.aim_c2y, z = fd.ctx.aim_c2z },
    { x = fd.ctx.aim_c3x, y = fd.ctx.aim_c3y, z = fd.ctx.aim_c3z },
}
if seeded then
    fd.logf("registration: no previous fit for '%s' - aiming with the permutation " ..
            "from the axis transforms", DEV)
else
    fd.logf("registration: re-measuring '%s', aiming with its own previous fit", DEV)
end

-- The device pose we start from, and the rotations the whole fit will be pinned
-- to. Read BEFORE anything moves: everything below is an offset from here, and a
-- pose read after the first jog would put the origin one jog out.
fd.read_position{ device = DEV, ctx_prefix = "h" }
local h0 = { x = fd.ctx.h_x, y = fd.ctx.h_y, z = fd.ctx.h_z }
local ru, rv, rw = fd.ctx.h_u, fd.ctx.h_v, fd.ctx.h_w
local hasRot = (ru ~= nil) and (rv ~= nil) and (rw ~= nil)
if not hasRot then ru, rv, rw = 0.0, 0.0, 0.0 end
fd.logf("'%s' starts at (%.4f, %.4f, %.4f)%s", DEV, h0.x, h0.y, h0.z,
        hasRot and string.format(", rotations (%.3f, %.3f, %.3f)", ru, rv, rw) or "")

if hasRot and (math.abs(ru) > 0.001 or math.abs(rv) > 0.001 or math.abs(rw) > 0.001) then
    -- Not refused: a machine may legitimately sit at a non-zero rotation. But the
    -- fit describes THIS rotation and no other, and every later prediction will
    -- refuse anywhere else - so say it out loud now rather than at the first
    -- puzzled move_to_point.
    if not fd.prompt(string.format(
            "'%s' is rotated to (%.3f, %.3f, %.3f). The registration will be pinned " ..
            "to exactly that - the tip swings about the pivot, so nothing will " ..
            "predict at any other rotation. Continue?", DEV, ru, rv, rw)) then
        error("cancelled by operator")
    end
end

if VJOG > 0.0 then fd.logf("registration: jogging '%s' at %.3f mm/s", DEV, VJOG) end

-- ---- Measuring one tip position --------------------------------------------
-- Focus, then pick. The focused REFERENCE Z is the tip's height: there is no
-- other way for one camera to measure a height, and it is why this column is the
-- noisiest of the three.
local function measure(label, hint)
    if FOCUS then
        fd.auto_focus{ device = REF, axis = "Z", z_range_mm = FRANGE }
    end
    local pick = { device = REF, ctx_prefix = "tip", label = label,
                   message = "Put the crosshair on the tool tip, then OK." }
    if hint then
        -- Where the tip should be if the aiming columns were perfect. Drawn as a
        -- target, or as an edge arrow with its distance when it is out of frame -
        -- the difference between "it is 0.4 mm left" and a blank picture.
        pick.hint      = true
        pick.hint_x_mm = hint.x
        pick.hint_y_mm = hint.y
    end
    fd.pick_point(pick)
    fd.read_position{ device = REF, ctx_prefix = "g" }
    return { x = fd.ctx.tip_wx, y = fd.ctx.tip_wy, z = fd.ctx.g_z }
end

-- ---- P0: find the tip by hand ----------------------------------------------
-- The one measurement nothing can aim: t is exactly what is unknown, so there is
-- no pose to drive to. Every measurement after this one is predicted.
if not fd.prompt(string.format(
        "Registering '%s' against '%s'.\n\n" ..
        "Bring the tool tip somewhere the camera can see it and where it has %.1f mm " ..
        "of clear travel in +X, +Y and +Z. This procedure WILL jog '%s' by that " ..
        "much on each axis. OK when it is safe.", DEV, REF, L, DEV)) then
    error("cancelled by operator")
end

fd.manual_adjust{ device = REF, step_um = 100, enable_x = true, enable_y = true,
                  enable_z = true,
                  message = "Get the tool tip into the picture and roughly in focus, " ..
                            "then OK." }
local P = {}
P[0] = measure("the tool tip at rest")
fd.logf("P0: tip at (%.4f, %.4f, %.4f) in '%s' space", P[0].x, P[0].y, P[0].z, REF)

-- ---- P1..P3: jog one axis, follow, measure ---------------------------------
local AXIS = { "X", "Y", "Z" }
for i = 1, 3 do
    local a = AXIS[i]
    local target = { x = h0.x, y = h0.y, z = h0.z }
    if a == "X" then target.x = h0.x + L
    elseif a == "Y" then target.y = h0.y + L
    else target.z = h0.z + L end

    -- Predict where the tip will land, and take the camera there. Without this
    -- the tip is L millimetres outside a six-millimetre field and the operator
    -- is hunting; with it, the tip lands within the aiming error of the middle
    -- of the picture.
    local pred = { x = P[0].x + L * C[i].x,
                   y = P[0].y + L * C[i].y,
                   z = P[0].z + L * C[i].z }

    if not fd.prompt(string.format(
            "About to jog '%s' %+.3f mm on %s, and follow with '%s' to (%.3f, %.3f, " ..
            "%.3f). Check both have room. OK to move.",
            DEV, L, a, REF, pred.x, pred.y, pred.z)) then
        error("cancelled by operator")
    end

    fd.move_axis_to{ device = DEV, axes = a,
                     x_mm = target.x, y_mm = target.y, z_mm = target.z,
                     velocity_mm_s = VJOG }
    if SETTLE > 0 then fd.sleep(SETTLE) end

    fd.move_axis_to{ device = REF, axes = "XYZ",
                     x_mm = pred.x, y_mm = pred.y, z_mm = pred.z,
                     velocity_mm_s = VTRAV }

    P[i] = measure(string.format("the tool tip after the %+.1f mm %s jog", L, a), pred)

    local dx, dy, dz = P[i].x - P[0].x, P[i].y - P[0].y, P[i].z - P[0].z
    local moved = math.sqrt(dx * dx + dy * dy + dz * dz)
    -- The scale check, in the log, before the fit refuses on it. A number that
    -- is not L is the single most informative thing in this whole procedure.
    fd.logf("P%d: %s jog of %.3f mm moved the tip %.4f mm (scale %.4f) along " ..
            "(%+.4f, %+.4f, %+.4f)",
            i, a, L, moved, moved / L, dx / L, dy / L, dz / L)
end

-- ---- Home the device -------------------------------------------------------
-- Before the save, not after: if the fit is refused, the device is still left
-- where the operator set it up rather than 10 mm out on three axes.
fd.move_axis_to{ device = DEV, axes = "XYZ",
                 x_mm = h0.x, y_mm = h0.y, z_mm = h0.z,
                 velocity_mm_s = VJOG }

-- ---- Fit and store ---------------------------------------------------------
-- The command does the arithmetic and owns the refusals; see core/DeviceReg.cpp.
fd.device_reg_save{
    device = DEV, ref_device = REF, jog_mm = L,
    h0_x_mm = h0.x, h0_y_mm = h0.y, h0_z_mm = h0.z,
    p0_x_mm = P[0].x, p0_y_mm = P[0].y, p0_z_mm = P[0].z,
    p1_x_mm = P[1].x, p1_y_mm = P[1].y, p1_z_mm = P[1].z,
    p2_x_mm = P[2].x, p2_y_mm = P[2].y, p2_z_mm = P[2].z,
    p3_x_mm = P[3].x, p3_y_mm = P[3].y, p3_z_mm = P[3].z,
    has_rotation = hasRot, rot_u_deg = ru, rot_v_deg = rv, rot_w_deg = rw,
}

-- Deliberately not "done". A fresh fit predicts P0 at h0 by construction, so
-- nothing here has tested the rotation - only the check pass, at a pose the fit
-- was NOT taught at, can do that. Saying "registered" and stopping is how an
-- unverified fit ends up carrying a placement.
fd.logf("'%s' is registered to '%s' and NOT yet verified. Run the check (with a " ..
        "non-zero offset) to measure it away from the pose it was fitted at - " ..
        "until then move_to_point will refuse unless it is told not to.", DEV, REF)
