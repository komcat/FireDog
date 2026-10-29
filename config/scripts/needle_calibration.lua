-- Needle calibration: measure where the dispense needle is, relative to the camera
--
-- The camera and the needle ride the same gantry, a hand's width apart. Nothing
-- can dispense on the feature the camera is looking at until that lever arm has
-- been measured, and it can only be measured the way you would measure it by
-- hand: put a dot on a coupon, look at the dot, subtract.
--
--   dx, dy  add to a CAMERA pose to get the pose that puts the NEEDLE there
--   dz      add to a FOCUSED camera Z to reach the height the needle touches at
--
-- Pass 1 MEASURES: the operator jogs the needle down onto the coupon by hand.
-- Pass 2+ VERIFIES: the machine drives the needle to where the calibration says
-- it touches, stops `verify_gap_mm` short, and the operator only closes that
-- gap. How far they had to jog is the Z residual; where the dot lands is the XY
-- residual. A calibration is VERIFIED only when both are inside tolerance -
-- dispense_procedure.lua drives to cam_z + dz with nobody's hand on the jog, so
-- dz is the number that most needs checking, and an XY-only check never did.
--
-- The motion of one pass, in order (the needle-calibration illustration):
--   traverse  needle over the spot, at the camera's Z        v_traverse
--   slide     pneumatic down
--   approach  Z down to the safety height (clear_mm short)   v_approach
--   final     Z the last stretch (verify passes only)        v_final
--   touch     the operator's jog; the shot
--   lift off  Z up lift_mm, off the dot                      v_liftoff
--   slide     pneumatic up
--   traverse  camera back over the dot to inspect it         v_traverse
--
-- Three things about this file are deliberate and worth reading before editing:
--
--  1. THE ARITHMETIC IS HERE. The sibling machine built this as a C++ plugin
--     emitting steps into a sequence language with no numeric literals, so a
--     step meaning "previous_z minus 1 mm" silently computed "previous_z minus
--     nothing" and drove the needle onto the coupon at 5 mm/s instead of parking
--     a millimetre clear. Lua has real numbers. That is most of why this is a
--     script.
--
--  2. NOTHING ASSUMES WHICH WAY Z POINTS. `dir` comes out of the measurement,
--     and the lift-off goes TOWARD the camera's Z - the height the needle came
--     down from - so it is right even before anything has been measured.
--
--  3. THE OFFSET IS CARRIED IN LOCALS between passes, not re-read through
--     fd.needle_offset. A saved calibration reaches Workflow via the UI thread
--     one frame later, so a second pass that re-read it could easily get the
--     value from before the first pass. Read once, compute onwards.
--
-- Parameters (fd.param -> ctx keys, injected by the Needle page's Calibrate
-- button or a needle_cal_run recipe step):
--   ndl_device         the gantry carrying both       (default "gantry-main")
--   ndl_slide          slide that puts the needle down (default "Dispenser_Head")
--   ndl_io             IO device with the glue valve   (default "IOBottom")
--   ndl_pin            the valve output pin            (default "Dispenser_Shot")
--   ndl_shot_ms        valve open time for the imprint (default 300)
--   ndl_settle_ms      pause after the slide extends   (default 500)
--   ndl_clear_mm       safety height: the fast approach stops this far short (1.0)
--   ndl_first_clear_mm pass 1 parks this far above the PREVIOUS touch (3.0)
--   ndl_verify_gap_mm  a verify pass stops this far above the predicted touch (0.1)
--   ndl_lift_mm        lift-off after the shot, before the slide comes up (1.5)
--   ndl_autofocus      focus on the coupon first       (default true)
--   ndl_passes         1 measures, 2+ verifies         (default 2)
--   ndl_tol_mm         XY residual a verify pass must hit (default 0.05)
--   ndl_z_tol_mm       Z residual a verify pass must hit  (default 0.05)
--   ndl_v_traverse     XY traverse, mm/s               (default 40)
--   ndl_v_approach     Z down to the safety height     (default 20)
--   ndl_v_final        Z safety height -> verify gap   (default 5)
--   ndl_v_liftoff      Z lift-off from the dot         (default 5)
--   ndl_clear_moves    "device=uid;..." moved to, in order, before anything
--                      else - the Needle page's "Clear first" list (default "")

local DEV      = fd.param("ndl_device", "gantry-main")
local SLIDE    = fd.param("ndl_slide", "Dispenser_Head")
local IO       = fd.param("ndl_io", "IOBottom")
local PIN      = fd.param("ndl_pin", "Dispenser_Shot")
local SHOT     = math.floor(fd.param("ndl_shot_ms", 300))
local SETTLE   = math.floor(fd.param("ndl_settle_ms", 500))
local CLEAR    = fd.param("ndl_clear_mm", 1.0)
local FIRSTCLR = fd.param("ndl_first_clear_mm", 3.0)
local GAP      = fd.param("ndl_verify_gap_mm", 0.1)
local LIFT     = fd.param("ndl_lift_mm", 1.5)
local FOCUS    = fd.param("ndl_autofocus", true)
local PASSES   = math.floor(fd.param("ndl_passes", 2))
local TOL      = fd.param("ndl_tol_mm", 0.05)
local ZTOL     = fd.param("ndl_z_tol_mm", 0.05)
local VTRAV    = fd.param("ndl_v_traverse", 40.0)
local VAPPR    = fd.param("ndl_v_approach", 20.0)
local VFINAL   = fd.param("ndl_v_final", 5.0)
local VLIFT    = fd.param("ndl_v_liftoff", 5.0)

if DEV == "" then error("ndl_device is not set - name the gantry to calibrate") end
if PASSES < 1 then PASSES = 1 end

-- Out of the camera's way first (the hexapods), before the coupon prompt.
for dev, uid in fd.param("ndl_clear_moves", ""):gmatch("([^;=]+)=([^;]+)") do
    fd.logf("needle calibration: clearing %s first", dev)
    fd.move_to_position{ device = dev, position = uid }
end

-- The offset as it stands right now. Read ONCE (see note 3 above); everything
-- after this is arithmetic on locals.
fd.needle_offset{ ctx_prefix = "prev" }
local have = fd.ctx.prev_ok
local dx   = fd.ctx.prev_dx
local dy   = fd.ctx.prev_dy
local dz   = fd.ctx.prev_dz
-- `have` covers a real calibration. A seed typed on the Needle page arrives as
-- ok = false with non-zero dx/dy, and it is enough to AIM with even though it is
-- not enough to dispense through - which is exactly what it is for.
local aim  = have or (dx ~= 0.0) or (dy ~= 0.0)
local aimZ = have or (dz ~= 0.0)

if have then
    fd.logf("needle calibration: starting from (%.4f, %.4f, %.4f) mm", dx, dy, dz)
elseif aim then
    fd.logf("needle calibration: no calibration yet, aiming with the seed (%.4f, %.4f)", dx, dy)
else
    fd.log("needle calibration: nothing to aim with - you will jog the needle over " ..
           "the coupon by hand, and hunt for the imprint afterwards")
end

-- Which way the axis moves to take the needle DOWN. Only meaningful once
-- something has measured it; until then nothing uses it.
local function down_sign(v) if v >= 0.0 then return 1.0 else return -1.0 end end

-- LIFT mm from `z` toward `z_up` (never past it). The camera's Z is where the
-- needle came down from, so "toward it" is up whichever way the axis counts,
-- and before any dz exists to say so.
local function lift_toward(z, z_up)
    local d = z_up - z
    local s = (d >= 0.0) and 1.0 or -1.0
    return z + s * math.min(LIFT, math.abs(d))
end

local residual, zres, verified = 0.0, 0.0, false

for pass = 1, PASSES do
    -- Pass 2+ checks the answer the pass before it produced: the machine goes
    -- where that says, and the operator reports how far off it was.
    local verify = pass > 1

    -- ---- 1. A clean spot on the coupon ------------------------------------
    if pass == 1 then
        if not fd.prompt("Put a flat, matte coupon under the camera - something an " ..
                         "epoxy dot will show up on. OK when it is in place.") then
            error("cancelled by operator")
        end
    else
        -- Pass 2 re-imprints. On the SAME spot it would land in the first dot
        -- and there would be nothing to measure.
        if not fd.prompt(string.format(
                "Pass %d of %d (VERIFY): move the coupon to a CLEAN spot (or slide a " ..
                "fresh one in), then OK.", pass, PASSES)) then
            error("cancelled by operator")
        end
    end

    fd.manual_adjust{ device = DEV, step_um = 100, enable_x = true, enable_y = true,
                      enable_z = true,
                      message = "Centre the camera on a clean part of the coupon, " ..
                                "roughly in focus, then OK." }

    -- ---- 2. The camera reference ------------------------------------------
    -- Focus FIRST. dz is measured down from a focal plane; measured down from
    -- wherever Z happened to be left, it is not an offset at all and the next
    -- dispense at a different height would be wrong by the difference.
    if FOCUS then
        fd.auto_focus{ device = DEV, axis = "Z" }
    end
    fd.read_position{ device = DEV, ctx_prefix = "cam" }
    local cam_x, cam_y, cam_z = fd.ctx.cam_x, fd.ctx.cam_y, fd.ctx.cam_z
    fd.logf("pass %d: camera at (%.4f, %.4f, %.4f)", pass, cam_x, cam_y, cam_z)

    -- ---- 3. Traverse: the needle over that spot ----------------------------
    -- At the CAMERA's Z, not the needle's: this is a traverse of a hundred
    -- millimetres or so, and it happens at whatever height the camera focuses
    -- at, which is well clear of the coupon with the slide up.
    if aim then
        fd.move_axis_to{ device = DEV, axes = "XYZ",
                         x_mm = cam_x + dx, y_mm = cam_y + dy, z_mm = cam_z,
                         velocity_mm_s = VTRAV }
    end

    if not fd.prompt(string.format(
            "About to extend '%s'. Check the needle has room to come down, then OK.",
            SLIDE)) then
        error("cancelled by operator")
    end

    -- Everything from here to the lift-off happens with the needle DOWN, and a
    -- failure in the middle of it (a Cancel on the jog or the shot) must not
    -- leave it standing on the coupon. It runs under pcall; any failure but an
    -- operator STOP - which means "no more motion", and whose hook re-fires on
    -- the next command anyway - lifts the needle off and brings the slide up
    -- before the error goes on.
    local tip_x, tip_y, tip_z
    local function needle_down()
        fd.extend_slide{ slide = SLIDE }
        fd.sleep(SETTLE)

        -- ---- 4. Touch down --------------------------------------------------
        -- When nothing aimed the needle it is hanging over wherever the lever
        -- arm put it, a hundred millimetres from the spot the camera was
        -- centred on. So coarse XY by eye first - and the message says to slide
        -- the COUPON, because covering that distance at ten microns a keypress
        -- is not a procedure.
        if not aim then
            fd.manual_adjust{ device = DEV, step_um = 100,
                              enable_x = true, enable_y = true, enable_z = false,
                              message = "Get the needle over a clean part of the coupon - " ..
                                        "jog XY, or just slide the coupon under it. Watch " ..
                                        "the needle, not the screen. OK when it is over it." }
        end

        local dir = down_sign(dz)
        local pz  = cam_z + dz          -- where the calibration says it touches
        if verify then
            -- The check: approach fast to the safety height, slow to verify_gap_mm
            -- short of the predicted touch, and let the operator close the gap.
            local target = pz - dir * GAP
            local safety = target - dir * CLEAR
            fd.move_axis_to{ device = DEV, axes = "Z", z_mm = safety, velocity_mm_s = VAPPR }
            fd.move_axis_to{ device = DEV, axes = "Z", z_mm = target, velocity_mm_s = VFINAL }
            fd.logf("pass %d: stopped %.3f mm short of the predicted touch Z %.4f", pass,
                    GAP, pz)
            -- No ctx_prefix here on purpose: manual_adjust reports the pose from
            -- the poll cache, read_position demands a fast poll and waits one
            -- cycle out. At a touch height the difference is the whole point.
            fd.manual_adjust{ device = DEV, step_um = 10,
                              enable_x = false, enable_y = false, enable_z = true,
                              message = string.format(
                                  "VERIFY: the needle stopped %.3f mm above where the " ..
                                  "calibration says it touches. Jog Z DOWN until it " ..
                                  "just touches - barely, not pressing. If it is " ..
                                  "ALREADY pressing, jog UP until it just touches. " ..
                                  "Then OK.", GAP) }
        else
            -- Pass 1 measures. The previous touch height may belong to a needle
            -- that has since been CHANGED - the usual reason to calibrate - so
            -- park a wider first_clear_mm above it, not the safety height, and
            -- hand the whole last stretch to the operator.
            if aimZ then
                fd.move_axis_to{ device = DEV, axes = "Z",
                                 z_mm = pz - dir * FIRSTCLR, velocity_mm_s = VAPPR }
                fd.logf("pass %d: parked %.3f mm short of the previous touch height",
                        pass, FIRSTCLR)
            end
            fd.manual_adjust{ device = DEV, step_um = 100,
                              enable_x = false, enable_y = false, enable_z = true,
                              message = "Jog Z DOWN until the needle just touches the " ..
                                        "coupon - barely, not pressing. Coarse steps " ..
                                        "first, 10 um for the last few tenths. Then OK." }
        end
        fd.read_position{ device = DEV, ctx_prefix = "tip" }
        tip_x, tip_y, tip_z = fd.ctx.tip_x, fd.ctx.tip_y, fd.ctx.tip_z

        -- ---- 5. The imprint -------------------------------------------------
        if not fd.prompt(string.format("Fire a %d ms shot to make the imprint?", SHOT)) then
            error("cancelled by operator")
        end
        fd.pulse_output{ device = IO, pin = PIN, ms = SHOT }

        -- ---- 6. Lift off, THEN the slide -------------------------------------
        -- Z first, by lift_mm, so the needle comes straight up off the dot it
        -- just made under Z's control; only then does the slide travel. The
        -- dispense procedure retreats in the same order (the Needle page's
        -- motion drawing covers both).
        fd.move_axis_to{ device = DEV, axes = "Z", z_mm = lift_toward(tip_z, cam_z),
                         velocity_mm_s = VLIFT }
        fd.retract_slide{ slide = SLIDE }
    end

    local ok, err = pcall(needle_down)
    if not ok then
        if not tostring(err):find("stopped", 1, true) then
            pcall(function()
                fd.read_position{ device = DEV, ctx_prefix = "abort" }
                fd.move_axis_to{ device = DEV, axes = "Z",
                                 z_mm = lift_toward(fd.ctx.abort_z, cam_z),
                                 velocity_mm_s = VLIFT }
                fd.retract_slide{ slide = SLIDE }
            end)
        end
        error(err, 0)
    end

    -- ---- 7. Traverse: the camera back over the dot ---------------------------
    -- The camera sees the imprint from gantry pose (imprint world position), and
    -- with an offset to aim by that is exactly where the camera was standing
    -- when this pass started - so drive back there and the dot lands within the
    -- calibration error of the middle of the picture. The slide is up and Z only
    -- climbs on the way, so the move is one coordinated traverse.
    --
    -- With NOTHING to aim by there is no such pose to compute, and driving back
    -- to the camera spot would be a hundred-millimetre round trip to a place the
    -- dot is not. Climb back to the camera's Z and hunt from here instead.
    -- Jogging during the hunt is harmless either way: the pick reports the dot's
    -- WORLD position and does not care where the camera stood when it was clicked.
    if aim then
        fd.move_axis_to{ device = DEV, axes = "XYZ",
                         x_mm = cam_x, y_mm = cam_y, z_mm = cam_z,
                         velocity_mm_s = VTRAV }
    else
        fd.move_axis_to{ device = DEV, axes = "Z", z_mm = cam_z, velocity_mm_s = VTRAV }
        fd.manual_adjust{ device = DEV, step_um = 100, enable_x = true, enable_y = true,
                          enable_z = false,
                          message = "Find the imprint: it is roughly one needle-to-lens " ..
                                    "distance from here, in the direction the lens sits " ..
                                    "from the needle. Jog until the dot is in the " ..
                                    "picture, then OK." }
    end

    local pick = { device = DEV, ctx_prefix = "dot", label = "the needle imprint",
                   message = "Put the crosshair on the CENTRE of the imprint, then OK." }
    if aim then
        -- Where the dot should be if the current offset were perfect. Shown as a
        -- target, or as an edge arrow with its distance when it is out of frame -
        -- which is the difference between "it is 4 mm left" and a blank picture.
        pick.hint      = true
        pick.hint_x_mm = tip_x - dx
        pick.hint_y_mm = tip_y - dy
    end
    fd.pick_point(pick)
    local dot_x, dot_y = fd.ctx.dot_wx, fd.ctx.dot_wy

    -- ---- 8. Subtract --------------------------------------------------------
    local ndx = tip_x - dot_x
    local ndy = tip_y - dot_y
    local ndz = tip_z - cam_z

    if verify then
        residual = math.sqrt((ndx - dx) ^ 2 + (ndy - dy) ^ 2)
        -- Signed along `down`: + = it touched further DOWN than predicted (a
        -- dispense would stop short), - = higher (a dispense would press in).
        zres = (ndz - dz) * down_sign(dz)
        fd.logf("pass %d: (%.4f, %.4f, %.4f) mm - XY residual %.4f mm, Z residual " ..
                "%+.4f mm", pass, ndx, ndy, ndz, residual, zres)
    elseif aim then
        fd.logf("pass %d: (%.4f, %.4f, %.4f) mm - moved the XY answer by %.4f mm",
                pass, ndx, ndy, ndz, math.sqrt((ndx - dx) ^ 2 + (ndy - dy) ^ 2))
    else
        fd.logf("pass %d: (%.4f, %.4f, %.4f) mm - first measurement", pass, ndx, ndy, ndz)
    end

    dx, dy, dz = ndx, ndy, ndz
    aim, aimZ  = true, true
    -- A pass counts as verified when it re-measured the pass before it and
    -- agreed in BOTH halves. Pass 1 is never that, however small its numbers.
    verified = verify and (residual <= TOL) and (math.abs(zres) <= ZTOL)

    -- Save every pass. A pass-2 abort must not throw away a good pass-1
    -- measurement, and the value is carried in locals anyway, so re-reading it
    -- is not something anything downstream depends on.
    fd.needle_offset_save{
        dx_mm = dx, dy_mm = dy, dz_mm = dz,
        device = DEV, slide = SLIDE, io_device = IO, shot_pin = PIN,
        verified = verified, residual_mm = residual,
        z_checked = verify, z_residual_mm = zres,
        cam_x_mm = cam_x, cam_y_mm = cam_y, cam_z_mm = cam_z,
        ndl_x_mm = tip_x, ndl_y_mm = tip_y, ndl_z_mm = tip_z,
        dot_x_mm = dot_x, dot_y_mm = dot_y,
    }

    if verified then
        fd.logf("pass %d agreed to within %.4f mm XY, %+.4f mm Z - done", pass,
                residual, zres)
        break
    end
end

-- Two different endings, and which one it is matters.
--
-- passes = 1 asked for a single measurement: it was taken, it was saved, and
-- nothing checked it. That SUCCEEDS - refusing work the caller explicitly asked
-- for would be wrong - but it says so loudly, and the Needle page keeps saying
-- so afterwards. An unverified calibration must never read like a verified one.
--
-- passes >= 2 means a pass DID re-measure and disagreed by more than a
-- tolerance. The offset is saved regardless, because it is still a better
-- number than the one before it, and then this fails: a measurement that will
-- not repeat points at the marking, at the touch, or at a needle that moves
-- when it touches down, and none of those is fixed by loosening the tolerance.
if PASSES == 1 then
    fd.logf("WARNING: needle offset (%.4f, %.4f, %.4f) mm saved UNVERIFIED " ..
            "(passes = 1) - set passes >= 2 to re-measure and check it", dx, dy, dz)
    return
end

if not verified then
    error(string.format(
        "needle calibration did not repeat: the last verify pass was off by " ..
        "%.4f mm XY (tolerance %.4f) and %+.4f mm Z (tolerance %.4f). The offset " ..
        "(%.4f, %.4f, %.4f) mm was saved anyway - it is better than what was there " ..
        "- but check the imprint quality, the touch, and that the needle is not " ..
        "deflecting on contact before dispensing through it.",
        residual, TOL, zres, ZTOL, dx, dy, dz))
end

fd.logf("needle calibrated: (%.4f, %.4f, %.4f) mm on %s, verified to %.4f mm XY, " ..
        "%+.4f mm Z", dx, dy, dz, DEV, residual, zres)
