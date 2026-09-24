-- XCAN two lens: left coarse to target, one signal-banded left follow-up, then one right coarse pass
--
-- The two-lens sequence as the operator drives it from the Run page's XCAN
-- band (6800 uA target, Left Coarse), written down so a recipe runs it the
-- same way every time:
--
--   1. LEFT coarse ladder, adaptive over [zone_low, target], exit at target.
--      If the target was not reached, the feedback after the pass picks ONE
--      follow-up on the left hexapod:
--         below `coarse_band_ua`             nothing - not enough light to climb on
--         [coarse_band_ua, fine_band_ua)     the coarse ladder once more
--         at or above `fine_band_ua`         the fine ladder once
--   2. Still short of the target: ONE coarse pass on the RIGHT hexapod.
--   3. Exit as is. The stage stays wherever the last pass left it, and the
--      result goes to ctx; the step fails only when `fail_if_missed` is set.
--
-- Every pass is fd.xcan (hexapod_align.lua), so the scan has one
-- implementation; this file only decides which hexapod, which ladder and
-- how many times. "Reached" is the pass's own exit flag OR a fresh reading
-- at or above the target after it.
--
-- Parameters (fd.param -> ctx keys; a recipe sets them with ctx_set steps
-- before its run_script step, or leaves the defaults):
--   xtl_left             left hexapod                        (default "hex-left")
--   xtl_right            right hexapod                       (default "hex-right")
--   xtl_topic            feedback signal                     (default "keithley/ch1")
--   xtl_axes             XYZ | XY | XZ | YZ                  (default "XYZ")
--   xtl_target_ua        exit threshold, uA                  (default 6800)
--   xtl_zone_low_ua      adaptive zone lower limit, uA       (default 200)
--   xtl_zone_high_ua     adaptive zone upper limit, uA       (default 0 = the target)
--   xtl_coarse_steps_um  the Coarse ladder                   (default "10,5,2,1")
--   xtl_fine_steps_um    the Fine ladder                     (default "2,1,0.5")
--   xtl_coarse_band_ua   left follow-up is coarse from here  (default 800)
--   xtl_fine_band_ua     left follow-up is fine from here    (default 1500)
--   xtl_settle_min_ms    settle floor before each sample     (default 0)
--   xtl_velocity_mm_s    scan velocity (0 = keep current)    (default 1)
--   xtl_trace            per-sample CSV trace, 1/0           (default 0)
--   xtl_fail_if_missed   error when the target is not reached (default 0: exit as is)
--
-- Results (ctx, for dut_record / if steps):
--   xtl_reached    true when a pass reached the target
--   xtl_passes     how many passes ran
--   xtl_last       which pass ran last: "left coarse" | "left coarse 2" | "left fine" | "right coarse"
--   xtl_best_ua    the last pass's final reading, uA

local function flag(v) return (v == true) or ((tonumber(v) or 0) ~= 0) end

local P = {
  left           = fd.param("xtl_left", "hex-left"),
  right          = fd.param("xtl_right", "hex-right"),
  topic          = fd.param("xtl_topic", "keithley/ch1"),
  axes           = fd.param("xtl_axes", "XYZ"),
  target_ua      = fd.param("xtl_target_ua", 6800),
  zone_low_ua    = fd.param("xtl_zone_low_ua", 200),
  zone_high_ua   = fd.param("xtl_zone_high_ua", 0),
  coarse_steps   = fd.param("xtl_coarse_steps_um", "10,5,2,1"),
  fine_steps     = fd.param("xtl_fine_steps_um", "2,1,0.5"),
  coarse_band_ua = fd.param("xtl_coarse_band_ua", 800),
  fine_band_ua   = fd.param("xtl_fine_band_ua", 1500),
  settle_min_ms  = math.floor(tonumber(fd.param("xtl_settle_min_ms", 0)) or 0),
  velocity_mm_s  = fd.param("xtl_velocity_mm_s", 1),
  trace          = flag(fd.param("xtl_trace", 0)),
  fail_if_missed = flag(fd.param("xtl_fail_if_missed", 0)),
}
if P.zone_high_ua <= 0 then P.zone_high_ua = P.target_ua end

if P.target_ua <= 0 then
  error("xtl_target_ua must be > 0 - the script has nothing to reach")
end
if P.coarse_band_ua >= P.fine_band_ua then
  error(string.format("xtl_coarse_band_ua (%g) must be below xtl_fine_band_ua (%g)",
                      P.coarse_band_ua, P.fine_band_ua))
end
if P.zone_low_ua >= P.zone_high_ua then
  error(string.format("xtl_zone_low_ua (%g) must be below the zone high (%g)",
                      P.zone_low_ua, P.zone_high_ua))
end

-- The bus carries amps; the recipe-facing numbers are uA (same convention as
-- the xcan step's exit_target_ua).
local function live_ua()
  local v = fd.read(P.topic)
  if v == nil then error("no feedback stream on " .. P.topic) end
  return v * 1e6
end

local passes = 0

-- One climb. Returns reached, live reading (uA).
local function climb(label, device, steps)
  passes = passes + 1
  fd.logf("xcan_two_lens: %s on %s, ladder %s um, adaptive [%g, %g] uA, target %g uA",
          label, device, steps, P.zone_low_ua, P.zone_high_ua, P.target_ua)
  fd.xcan{
    device         = device,
    topic          = P.topic,
    axes           = P.axes,
    steps_um       = steps,
    exit_target_ua = P.target_ua,
    adaptive       = true,
    zone_low_ua    = P.zone_low_ua,
    zone_high_ua   = P.zone_high_ua,
    settle_min_ms  = P.settle_min_ms,
    velocity_mm_s  = P.velocity_mm_s,
    trace          = P.trace,
  }
  local hit  = flag(fd.ctx.align_exit_hit)
  local best = (tonumber(fd.ctx.align_best) or 0) * 1e6
  local now  = live_ua()
  fd.ctx.xtl_last    = label
  fd.ctx.xtl_best_ua = best
  fd.logf("xcan_two_lens: %s done - best %.4g uA, live %.4g uA, exit %s",
          label, best, now, hit and "hit" or "not hit")
  return hit or now >= P.target_ua, now
end

-- 1. Left coarse, then one banded follow-up on the left.
local reached, now = climb("left coarse", P.left, P.coarse_steps)

if not reached then
  if now >= P.fine_band_ua then
    reached, now = climb("left fine", P.left, P.fine_steps)
  elseif now >= P.coarse_band_ua then
    reached, now = climb("left coarse 2", P.left, P.coarse_steps)
  else
    fd.logf("xcan_two_lens: %.4g uA is below the %g uA coarse band - no left follow-up",
            now, P.coarse_band_ua)
  end
end

-- 2. Still short: one coarse pass on the right hexapod.
if not reached then
  reached, now = climb("right coarse", P.right, P.coarse_steps)
end

-- 3. Exit as is.
fd.ctx.xtl_reached = reached
fd.ctx.xtl_passes  = passes

if reached then
  fd.logf("xcan_two_lens: target %g uA reached after %d pass(es) (%s)",
          P.target_ua, passes, fd.ctx.xtl_last)
else
  local msg = string.format("xcan_two_lens: target %g uA NOT reached after %d pass(es) - last %.4g uA (%s)",
                            P.target_ua, passes, now, fd.ctx.xtl_last)
  if P.fail_if_missed then error(msg) end
  fd.log(msg)
end
