-- XCAN to target: one coarse ladder, then up to N fine ladders until the feedback reaches the target
--
-- Pass 1 is the ordinary hill climb (hexapod_align.lua through fd.xcan) with
-- the coarse ladder and the adaptive zones, exiting the moment a sample
-- reaches the target. If the feedback is still under the target afterwards,
-- the climb is repeated with the FINE ladder, up to `retries` times, and the
-- script returns as soon as any pass reaches it.
--
-- Every pass is the same fd.xcan a recipe step runs, so the scan itself has
-- exactly one implementation: this file only decides how many times to ask
-- and with which ladder. "Reached" is judged on the pass's own exit flag OR
-- a fresh reading of the topic after it, whichever says yes - the exit flag
-- is the authoritative one, the reading catches a peak that settled above
-- the target after the last sample.
--
-- Parameters (fd.param -> ctx keys; a recipe sets them with ctx_set steps
-- before its run_script step, or leaves the defaults):
--   xcan_device          hexapod to scan                     (default "hex-left")
--   xcan_topic           feedback signal                     (default "keithley/ch1")
--   xcan_axes            XYZ | XY | XZ | YZ                  (default "XYZ")
--   xcan_target_ua       exit threshold, uA                  (default 800)
--   xcan_steps_um        pass-1 ladder, coarse -> fine       (default "10,5,2,1")
--   xcan_adaptive        pass-1 signal-zoned ladder, 1/0     (default 1)
--   xcan_zone_low_ua     adaptive zone lower limit, uA       (default 1)
--   xcan_zone_high_ua    adaptive zone upper limit, uA       (default 800)
--   xcan_fine_steps_um   retry ladder                        (default "2,1,0.5")
--   xcan_fine_adaptive   retry passes signal-zoned, 1/0      (default 0: run every rung)
--   xcan_retries         fine passes after pass 1            (default 3)
--   xcan_settle_min_ms   settle floor before each sample     (default 0)
--   xcan_velocity_mm_s   scan velocity (0 = keep current)    (default 1)
--   xcan_trace           per-sample CSV trace, 1/0           (default 0)
--   xcan_fail_if_missed  error when no pass reaches target   (default 1)
--
-- Results (ctx, for dut_record / if steps):
--   xcan_reached   true when a pass reached the target
--   xcan_passes    how many passes ran (1 + fine passes)
--   xcan_best_ua   the last pass's final reading, uA

local function flag(v) return (v == true) or ((tonumber(v) or 0) ~= 0) end

local P = {
  device         = fd.param("xcan_device", "hex-left"),
  topic          = fd.param("xcan_topic", "keithley/ch1"),
  axes           = fd.param("xcan_axes", "XYZ"),
  target_ua      = fd.param("xcan_target_ua", 800),
  steps_um       = fd.param("xcan_steps_um", "10,5,2,1"),
  adaptive       = flag(fd.param("xcan_adaptive", 1)),
  zone_low_ua    = fd.param("xcan_zone_low_ua", 1),
  zone_high_ua   = fd.param("xcan_zone_high_ua", 800),
  fine_steps_um  = fd.param("xcan_fine_steps_um", "2,1,0.5"),
  fine_adaptive  = flag(fd.param("xcan_fine_adaptive", 0)),
  retries        = math.floor(tonumber(fd.param("xcan_retries", 3)) or 3),
  settle_min_ms  = math.floor(tonumber(fd.param("xcan_settle_min_ms", 0)) or 0),
  velocity_mm_s  = fd.param("xcan_velocity_mm_s", 1),
  trace          = flag(fd.param("xcan_trace", 0)),
  fail_if_missed = flag(fd.param("xcan_fail_if_missed", 1)),
}

if P.target_ua <= 0 then
  error("xcan_target_ua must be > 0 - the script has nothing to reach")
end
if P.retries < 0 or P.retries > 20 then
  error(string.format("xcan_retries is %d - it must be between 0 and 20", P.retries))
end

-- The bus carries amps; the recipe-facing numbers are uA (same convention as
-- the xcan step's exit_target_ua).
local function live_ua()
  local v = fd.read(P.topic)
  if v == nil then error("no feedback stream on " .. P.topic) end
  return v * 1e6
end

-- Scan-log tags (docs/xcan-dataset.md): every pass's record carries the same
-- group, its pass number and stage, so the dataset sees one alignment, not N
-- unrelated scans. Cleared on every exit, error included, so a later plain
-- xcan in the same run is not filed under this group.
local GROUP = fd.scanlog.new_group("xcan_to_target")
local function tag(pass, stage)
  fd.ctx.scanlog_caller    = "xcan_to_target"
  fd.ctx.scanlog_group     = GROUP
  fd.ctx.scanlog_pass      = pass
  fd.ctx.scanlog_stage     = stage
  fd.ctx.scanlog_target_ua = P.target_ua
end
local function untag()
  for _, k in ipairs({ "caller", "group", "pass", "stage", "target_ua" }) do
    fd.ctx["scanlog_" .. k] = ""
  end
end

-- One climb. Returns true when the target was reached.
local function climb(label, steps, adaptive)
  fd.logf("xcan_to_target: %s on %s, ladder %s um%s, target %g uA",
          label, P.device, steps, adaptive and " (adaptive)" or "", P.target_ua)
  fd.xcan{
    device         = P.device,
    topic          = P.topic,
    axes           = P.axes,
    steps_um       = steps,
    exit_target_ua = P.target_ua,
    adaptive       = adaptive,
    zone_low_ua    = P.zone_low_ua,
    zone_high_ua   = P.zone_high_ua,
    settle_min_ms  = P.settle_min_ms,
    velocity_mm_s  = P.velocity_mm_s,
    trace          = P.trace,
  }
  local hit  = flag(fd.ctx.align_exit_hit)
  local best = (tonumber(fd.ctx.align_best) or 0) * 1e6
  local now  = live_ua()
  fd.ctx.xcan_best_ua = best
  fd.logf("xcan_to_target: %s done - best %.4g uA, live %.4g uA, exit %s",
          label, best, now, hit and "hit" or "not hit")
  return hit or now >= P.target_ua
end

local passes  = 1
local reached = false

local ok, err = pcall(function()
  tag(1, "coarse")
  reached = climb("pass 1 (coarse)", P.steps_um, P.adaptive)

  for i = 1, P.retries do
    if reached then break end
    passes = passes + 1
    tag(passes, "fine")
    reached = climb(string.format("retry %d of %d (fine)", i, P.retries),
                    P.fine_steps_um, P.fine_adaptive)
  end
end)
untag()
if not ok then error(err, 0) end

fd.ctx.xcan_reached = reached
fd.ctx.xcan_passes  = passes

if reached then
  fd.logf("xcan_to_target: target %g uA reached after %d pass(es)", P.target_ua, passes)
else
  local msg = string.format("xcan_to_target: target %g uA NOT reached after %d pass(es) - last %.4g uA",
                            P.target_ua, passes, fd.ctx.xcan_best_ua)
  if P.fail_if_missed then error(msg) end
  fd.log(msg)
end
