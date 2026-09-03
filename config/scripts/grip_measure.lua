-- Grip measurement: where the object in a gripper's jaws actually sits
--
-- A gripper picks a lens and it lands in the jaws a little differently every
-- time. The device registration says where the gripper's TIP is; this drives the
-- tool where the camera can see it, finds the object, and reports the difference
-- between the two. That difference is the grip error, and a placement that knows
-- it lands on its first pass instead of its second.
--
--   1  INSPECT   send the gripper to its taught inspect pose. That pose carries
--                the ROTATIONS the reference offset was taught at, which is what
--                makes two measurements comparable -- a gripper turned 5 degrees
--                swings the object and reads as a mis-grip. A profile with no
--                inspect pose skips this leg and measures the tool where it is.
--   2  LOOK      drive the camera onto the tip (move_camera_to_tool). Nothing
--                here knows how tall the tool is, so Z is opt-in.
--   3  FOCUS     an optional sweep, once the camera is over the object.
--   4  MARK      one detection, or two crosshair marks, per the profile's mode.
--   5  APPLY     centre - tip = the offset, judged against the profile.
--
-- What it leaves in ctx (<prefix>_dx/_dy, and _hx/_hy in the gripper's own
-- axes) is what a later `grip_move_to_camera` step moves on: the tip is sent
-- to the camera point minus that offset, so the OBJECT lands under the camera.
--
-- It is the same file the Gripped Object page's Measure button runs, which is
-- the point: the procedure an operator taught and the one a recipe runs at three
-- in the morning have to be one thing.
--
-- Parameters (fd.param -> ctx keys). Two surfaces inject them: a `grip_measure`
-- recipe step and that button. Running this file straight from the Scripts page
-- fails on the first parameter, by design -- nothing there says which object.
--   grip_profile_uid    the profile, uid or name         (required)
--   grip_go_to_inspect  drive to the inspect pose first  (default true)
--   grip_look           drive the camera onto the tip    (default true)
--   grip_focus_z        let that move command Z too      (default false)
--   grip_z_offset_mm    manual Z offset on that move, added to the tip's
--                       focused height; non-zero commands Z by itself (default 0)
--   grip_autofocus      sweep for focus after the move   (default false)
--   grip_focus_range_mm sweep range                      (default 0.5)
--   grip_settle_ms      pause before looking             (default 300)
--   grip_require_ok     fail the run on a bad grip       (default true)
--   grip_prefix         ctx prefix for every key         (default "grip")

local PROFILE = fd.param("grip_profile_uid", "")
local GOTO    = fd.param("grip_go_to_inspect", true)
local LOOK    = fd.param("grip_look", true)
local FOCUSZ  = fd.param("grip_focus_z", false)
local ZOFF    = fd.param("grip_z_offset_mm", 0)
local AF      = fd.param("grip_autofocus", false)
local AFRANGE = fd.param("grip_focus_range_mm", 0.5)
local SETTLE  = fd.param("grip_settle_ms", 300)
local NEEDOK  = fd.param("grip_require_ok", true)
local PREFIX  = fd.param("grip_prefix", "grip")

if PROFILE == "" then
    error("grip_profile_uid is not set (which object is being measured). Launch " ..
          "this from the Gripped Object page's Measure button or a grip_measure " ..
          "recipe step - running the file directly from the Scripts page injects " ..
          "no parameters.")
end

-- Everything the moves need comes out of the record, including WHICH device
-- carries the camera: that is a property of the gripper's registration, and a
-- script that guessed "gantry-main" would be wrong on the first machine that
-- had two.
fd.grip_profile{ profile = PROFILE, ctx_prefix = "gp", require = true }

local DEV    = fd.ctx.gp_device
local CAM    = fd.ctx.gp_ref_device
local MARKS  = math.floor(fd.ctx.gp_marks)
local NAME   = fd.ctx.gp_name

fd.logf("grip: measuring '%s' held by %s, through %s (%d mark(s), %s mode)",
        NAME, DEV, CAM, MARKS, fd.ctx.gp_mode)

-- 1  The inspect pose. Skipping it is legitimate and stated, not silent: a
-- recipe that has already posed the tool should not pay for a second move.
if GOTO then
    if fd.ctx.gp_has_inspect then
        fd.move_to_position{ device = DEV, position = fd.ctx.gp_inspect }
    else
        fd.logf("grip: '%s' has no inspect pose taught - measuring %s where it stands. " ..
                "A reference offset taught at another rotation will not compare.",
                NAME, DEV)
    end
end

-- 2  The camera onto the tool. XY only unless asked otherwise - by name, or by
-- an offset, which would otherwise be a number that silently did nothing.
if LOOK then
    local takeZ = FOCUSZ or ZOFF ~= 0
    fd.move_camera_to_tool{ device = DEV, axes = takeZ and "XYZ" or "XY",
                            z_offset_mm = ZOFF }
end

-- 3  Focus. After the crossing, never before it: a sweep at the old pose
-- measures the focus of a scene this run is about to leave.
if AF then
    fd.auto_focus{ device = CAM, axis = "Z", z_range_mm = AFRANGE }
end

if SETTLE > 0 then fd.sleep(SETTLE) end

-- 4 + 5  The marks and the verdict. One mark in auto mode (a detector reports
-- the centre), two in manual (their midpoint is the centre), then the
-- arithmetic - grip_detect reads the count off the profile, so neither this
-- script nor a recipe using that step alone can disagree with grip_apply
-- about it.
fd.grip_detect{ profile = PROFILE, ctx_prefix = PREFIX, require_ok = NEEDOK }

fd.logf("grip: '%s' sits (%+.4f, %+.4f) mm from the %s tip - %s",
        NAME, fd.ctx[PREFIX .. "_dx"], fd.ctx[PREFIX .. "_dy"], DEV,
        fd.ctx[PREFIX .. "_ok"] and "OK" or fd.ctx[PREFIX .. "_why"])
