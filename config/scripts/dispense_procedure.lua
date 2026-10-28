-- Dispense procedure: put a metered dot of epoxy where the camera is looking
--
-- The dispense point is a CAMERA POSE over the target: a point taught into the
-- dispense_run step itself (Teach here), or a saved position when several steps
-- share one. The whole trick: a point is taught by driving the camera to the
-- feature and looking at it, which an operator can do and check, and the needle
-- calibration is what turns "where the camera stands" into "where the needle
-- stands". Nothing here holds a needle coordinate, so nothing here goes stale
-- when the needle is changed - re-calibrate and every taught point still works.
--
-- Shot size is the valve open time and nothing else. There is no pressure
-- control in this loop, no purge, no bead: the external regulator sets the
-- flow, `shot_ms` sets how long it flows. Do not read `shot_ms` as a volume
-- without knowing what the regulator is at.
--
-- THIS FILE IS NOT A STANDARD PROCEDURE, unlike needle_calibration.lua next to
-- it, and the split is on purpose. Where the needle is, is a fact about the
-- machine and every product must measure it identically. How much glue goes
-- where is a fact about the PRODUCT - so a product's own copy of this file wins
-- over the shipped one, and is meant to.
--
-- Z SIGN: never assumed. `dir` comes from the sign of the calibrated dz, so
-- "one millimetre above the part" is one millimetre above it on either
-- convention.
--
-- Parameters (fd.param -> ctx keys, injected by a dispense_run recipe step):
--   disp_point       true = the step carries a taught camera point:
--   disp_x/_y/_z       its X, Y, Z in the device's axis space
--   disp_position    else a saved position with the CAMERA over the point
--                    (exactly one of the two; dispense_run refuses both/none)
--   disp_device      the gantry carrying both       (default "gantry-main")
--   disp_shot_ms     valve open time = shot size    (default 300)
--   disp_before_ms   settle at height before firing (default 0)
--   disp_after_ms    dwell after firing, string break (default 0)
--   disp_z_trim_mm   + moves the needle CLOSER to the part (default 0)
--   disp_gap_mm      standoff ABOVE the touch height (default 0)
--   disp_safety_mm   fast -> slow handover height   (default 1.0)
--   disp_adjust      after arriving, let the operator nudge X/Y onto the
--                    point; the dot goes where they leave it (default true)
--   disp_autofocus   take the height from a focus here instead of the
--                    position's taught Z            (default false)
--   disp_confirm     ask before extending and before firing (default true)
--   disp_verify      bring the camera back over the dot    (default true)
--   disp_verify_ms   confirm OFF: the dot prompt continues by itself after
--                    this long; 0 = no prompt              (default 3000)
--   disp_capture     attach an image to the open DUT       (default true)
--   disp_v_travel / disp_v_approach / disp_v_final / disp_v_liftoff
--                    velocities, mm/s

local POS     = fd.param("disp_position", "")
local POINT   = fd.param("disp_point", false)
local PX      = fd.param("disp_x", 0.0)
local PY      = fd.param("disp_y", 0.0)
local PZ      = fd.param("disp_z", 0.0)
local DEV     = fd.param("disp_device", "gantry-main")
local SHOT    = math.floor(fd.param("disp_shot_ms", 300))
local BEFORE  = math.floor(fd.param("disp_before_ms", 0))
local AFTER   = math.floor(fd.param("disp_after_ms", 0))
local TRIM    = fd.param("disp_z_trim_mm", 0.0)
local GAP     = fd.param("disp_gap_mm", 0.0)
local SAFETY  = fd.param("disp_safety_mm", 1.0)
local ADJUST  = fd.param("disp_adjust", true)
local FOCUS   = fd.param("disp_autofocus", false)
local CONFIRM = fd.param("disp_confirm", true)
local VERIFY  = fd.param("disp_verify", true)
local VERIFY_MS = math.floor(fd.param("disp_verify_ms", 3000))
local CAPTURE = fd.param("disp_capture", true)
local VTRAVEL = fd.param("disp_v_travel", 0.0)
local VAPPR   = fd.param("disp_v_approach", 20.0)
local VFINAL  = fd.param("disp_v_final", 5.0)
local VLIFT   = fd.param("disp_v_liftoff", 5.0)

if POS == "" and not POINT then
    error("no dispense point (disp_point / disp_position) - the camera pose over " ..
          "the point. Launch this from a dispense_run recipe step - running the " ..
          "file directly from the Scripts page injects no parameters.")
end

-- ---- 1. The lever arm ------------------------------------------------------
-- First, and hard. Everything below is a move computed from these three
-- numbers; without them the only honest thing to do is stop before anything
-- has travelled anywhere.
-- Prefix "arm", not "ndl": a recipe shares one ctx across its steps, and
-- needle_cal_run injects ndl_* parameters into that same ctx. Two families of
-- key with one prefix is how a later step quietly reads an earlier step's
-- setting instead of its own.
fd.needle_offset{ ctx_prefix = "arm", require = true }
local dx, dy, dz = fd.ctx.arm_dx, fd.ctx.arm_dy, fd.ctx.arm_dz
local SLIDE = fd.ctx.arm_slide
local IO    = fd.ctx.arm_io
local PIN   = fd.ctx.arm_pin
local dir   = fd.ctx.arm_down           -- +1 or -1: which way the axis goes DOWN

if not fd.ctx.arm_verified then
    fd.log("WARNING: dispensing through an UNVERIFIED needle calibration - it was " ..
           "measured once and never re-checked (Needle page: calibrate with passes >= 2)")
end

-- The velocity the gantry is set to BEFORE anything here touches it. Velocity
-- persists on the controller, and every leg below sets its own; this is what
-- the lift-off hands back, so the next step travels at what the operator (or
-- the step before) chose rather than at whatever the last leg here left.
fd.read_velocity{ device = DEV, ctx_prefix = "disp_start" }
local VSTART = fd.ctx.disp_start_mm_s

-- 0.001 is set_velocity's own floor: below it the command fails validation,
-- and failing here - with the glue already down - would report a dispense that
-- actually worked as a failed step.
local function restore_velocity()
    if VSTART >= 0.001 then
        fd.set_velocity{ device = DEV, velocity_mm_s = VSTART }
    else
        fd.logf("note: %s read %.4f mm/s at the start - not restored", DEV, VSTART)
    end
end

-- ---- 2. Camera over the point ----------------------------------------------
if POINT then
    fd.move_axis_to{ device = DEV, axes = "XYZ", x_mm = PX, y_mm = PY, z_mm = PZ,
                     velocity_mm_s = VTRAVEL }
else
    fd.move_to_position{ device = DEV, position = POS, velocity_mm_s = VTRAVEL }
end

-- The taught pose gets the camera NEAR the point; this part sits wherever it
-- sits today. X/Y only: Z is the focus, and the touch height is computed from
-- it below - a hand-jogged Z would quietly move every dot's height with it.
-- Before the auto_focus, so the focus is taken over the spot actually chosen.
-- Nothing is written back: the next part needs its own nudge, and a run that
-- re-taught its own point would walk the point off one part at a time.
if ADJUST then
    fd.read_position{ device = DEV, ctx_prefix = "taught" }
    fd.manual_adjust{ device = DEV, step_um = 10,
                      enable_x = true, enable_y = true, enable_z = false,
                      message = "Centre the camera crosshair on the dispense point " ..
                                "(X/Y only), then OK. The dot goes where you leave it." }
end

-- Focusing here rather than trusting the taught Z is the option that makes dz
-- an offset worth having: the part's actual height today, plus the arm, is the
-- touch height today. Off by default because it costs a sweep per dot and a
-- taught Z is usually good enough.
if FOCUS then
    fd.auto_focus{ device = DEV, axis = "Z" }
end

fd.read_position{ device = DEV, ctx_prefix = "cam" }
local cam_x, cam_y, cam_z = fd.ctx.cam_x, fd.ctx.cam_y, fd.ctx.cam_z
if ADJUST then
    fd.logf("dispense: operator adjusted X %+.4f, Y %+.4f mm from the taught pose",
            cam_x - fd.ctx.taught_x, cam_y - fd.ctx.taught_y)
end

-- Two Z corrections, and they are not the same thing.
--   TRIM corrects the CALIBRATION: + brings the needle closer to the part,
--   because the measured touch height was slightly off on this machine.
--   GAP is a deliberate STANDOFF: the needle is meant to stop short and let the
--   bead bridge the gap. It is a property of the glue and the dot size, not of
--   the machine, which is why it is its own number - trimming a gap away would
--   also lose the calibration correction it was hiding.
-- Both are applied along `dir`, so neither assumes which way Z counts.
local touch_z  = cam_z + dz + dir * TRIM      -- needle ON the part
local target_z = touch_z - dir * GAP          -- where it actually stops
local safety_z = target_z - dir * SAFETY
fd.logf("dispense: camera (%.4f, %.4f, %.4f), touch Z %.4f, gap %.3f -> " ..
        "needle target Z %.4f (safety %.4f)",
        cam_x, cam_y, cam_z, touch_z, GAP, target_z, safety_z)

-- ---- 3. Needle over the point ----------------------------------------------
-- Still at the camera's height. This leg is the length of the lever arm - tens
-- of millimetres - and it happens well clear of the part.
fd.move_axis_to{ device = DEV, axes = "XYZ",
                 x_mm = cam_x + dx, y_mm = cam_y + dy, z_mm = cam_z,
                 velocity_mm_s = VTRAVEL }

if CONFIRM then
    if not fd.prompt(string.format(
            "Needle is over the point. Extend '%s' and dispense?", SLIDE)) then
        error("cancelled by operator")
    end
end
fd.extend_slide{ slide = SLIDE }

-- ---- 4. Two-speed descent ---------------------------------------------------
-- Fast to a millimetre short, slow for the last millimetre. The slow leg is
-- what stops the needle arriving with momentum it has to shed into the part;
-- the fast leg is what stops that costing a second per dot.
fd.move_axis_to{ device = DEV, axes = "Z", z_mm = safety_z, velocity_mm_s = VAPPR }
fd.move_axis_to{ device = DEV, axes = "Z", z_mm = target_z, velocity_mm_s = VFINAL }

-- ---- 5. The shot -------------------------------------------------------------
if CONFIRM then
    if not fd.prompt(string.format("At dispense height. Fire a %d ms shot?", SHOT)) then
        -- Retreat before failing: leaving the needle sitting over the part
        -- because the operator said no would be a worse outcome than the
        -- dispense. Same order as the normal retreat below - slide first.
        fd.retract_slide{ slide = SLIDE }
        fd.move_axis_to{ device = DEV, axes = "Z", z_mm = cam_z, velocity_mm_s = VLIFT }
        restore_velocity()
        error("cancelled by operator")
    end
end

if BEFORE > 0 then fd.sleep(BEFORE) end
-- pulse_output closes the valve even when a Stop interrupts the wait, which is
-- the only reason this is one command instead of on / delay / off.
fd.pulse_output{ device = IO, pin = PIN, ms = SHOT }
if AFTER > 0 then fd.sleep(AFTER) end

-- ---- 6. Retreat ---------------------------------------------------------------
-- The SLIDE comes up first. It lifts the needle straight off the dot on the
-- cylinder's own stroke - faster than Z can, and it breaks the string while the
-- head is still stationary over the point. Only then does Z climb, and at the
-- lift-off velocity rather than v_final: v_final exists to arrive on a part
-- gently, and there is nothing left to arrive at once the needle is clear.
-- XY comes last either way, so the needle never travels sideways while it is
-- still down near the dot it just placed.
-- Once clear, the gantry goes back to the velocity it had when this started.
fd.retract_slide{ slide = SLIDE }
fd.move_axis_to{ device = DEV, axes = "Z", z_mm = cam_z, velocity_mm_s = VLIFT }
restore_velocity()

-- ---- 7. Look at the result ------------------------------------------------
if VERIFY then
    fd.move_axis_to{ device = DEV, axes = "XYZ",
                     x_mm = cam_x, y_mm = cam_y, z_mm = cam_z,
                     velocity_mm_s = VTRAVEL }
    if CAPTURE and fd.dut.is_open() then
        fd.dut_capture_image{ key = "dispense" }
    end
    -- With confirm on, the run waits for an answer. With it off, the operator
    -- still gets a look: the prompt continues by itself after VERIFY_MS, and a
    -- Cancel inside that window still rejects the dot.
    local ok = true
    if CONFIRM then
        ok = fd.prompt("Does the dot look right?")
    elseif VERIFY_MS > 0 then
        ok = fd.prompt("Check the dot. Cancel if it is wrong.", VERIFY_MS)
    end
    if not ok then
        restore_velocity()
        error("dispense rejected by operator")
    end
end

-- ---- 8. Leave the device as it was found ---------------------------------
-- Velocity persists on the controller, and this is the single most common way
-- to lose minutes to a machine that looks broken: whatever the LAST move here
-- set is what the next recipe step - and the next RUN of this one - travels at.
-- The lift-off already restored the starting velocity; the verify leg above
-- may have changed it again (v_travel), so put it back once more.
if VERIFY and VTRAVEL > 0 then
    restore_velocity()
end

fd.logf("dispensed %d ms at (%.4f, %.4f), needle Z %.4f (%.3f mm above touch)",
        SHOT, cam_x, cam_y, target_z, GAP)
