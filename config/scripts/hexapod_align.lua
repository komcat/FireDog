-- Active alignment: coordinate-descent hill climb, coarse-to-fine (bobcat XCAN port)
-- Port of bobcat_engine's CoarseToFineStrategy: for each step size (coarse->fine),
-- scan each axis in turn - probe BOTH directions with a ratchet walk, return to
-- the best point, recover to the global peak if the current signal fell off.
-- Parameters come from the recipe via ctx (fd.param = ctx value or default);
-- results land back in ctx (align_*) for dut_record / conditionals.
--
-- Cycle-time trims on top of the bobcat behaviour (all default ON, set the
-- align_* param to 0 to restore the original two-sided/always-remeasure scan):
--   align_skip_opposite - a first direction that walked uphill already brackets
--                         the peak; the opposite direction is skipped.
--   align_dir_memory    - each axis starts with the direction that last improved.
--   align_reuse_sample  - a point that was just measured is not measured again
--                         (direction starts, axis-done checks).
--   align_fresh_samples - measure() waits for this many NEW publishes on the
--                         feedback topic instead of sleeping settle_ms blind:
--                         the move commands already guarantee the mechanics
--                         are settled (arrival tolerance + PI motion-profile
--                         done), so the wait only has to outlast the sample
--                         that was integrating WHILE the stage moved. 2 =
--                         first fully-post-move sample; settle_ms becomes the
--                         timeout ceiling, not a delay. 0 = original blind sleep.

-- ---- Parameters (inject from the recipe with ctx_set "align_..." steps) ----
local P = {
  device         = fd.param("align_device", "hex-left"),
  topic          = fd.param("align_topic", "keithley/ch1"),   -- feedback signal
  axes           = fd.param("align_axes", "Z,Y,X"),           -- coordinate-descent order
  steps_um       = fd.param("align_steps_um", "10,5,2,1"),    -- coarse -> fine ladder
  settle_ms      = fd.param("align_settle_ms", 300),          -- fresh-sample ceiling (legacy: blind delay)
  settle_min_ms  = fd.param("align_settle_min_ms", 0),        -- floor for slow receivers (photodiode/TIA)
  fresh_samples  = fd.param("align_fresh_samples", 2),        -- new publishes per measure (0 = blind sleep)
  consec_limit   = fd.param("align_consec_limit", 3),   -- stop dir after N non-improvements
  sig_drop_pct   = fd.param("align_sig_drop_pct", 15),  -- hard break on a big relative drop
                                                        -- (bobcat used 5, but on this device
                                                        -- noise dips reach ~6% while true
                                                        -- post-peak drops are 50%+ — 5 ended
                                                        -- productive walks early)
  max_travel_um  = fd.param("align_max_travel_um", 200),-- per-direction travel cap
  improve_pct    = fd.param("align_improve_pct", 1),    -- global-peak recovery threshold
  avg_count      = fd.param("align_avg_count", 3),      -- baseline averaging samples
  min_signal     = fd.param("align_min_signal", 0),     -- abort if |baseline| below this
  exit_threshold = fd.param("align_exit_threshold", 0), -- stop when a sample >= this (0 = off)
  tol_mm         = fd.param("align_tol_mm", 0.001),     -- move arrival tolerance
  move_timeout   = fd.param("align_move_timeout_ms", 10000),
  velocity_mm_s  = fd.param("align_velocity_mm_s", 1),  -- scan velocity, set once
                                                        -- up front so settle is
                                                        -- deterministic (0 = keep)
  -- Adaptive step (signal-zoned ladder): the [zone_low, zone_high] signal
  -- range is split into #steps equal LOG sections (Gaussian: distance from
  -- peak ~ sqrt(log ratio)), and steps_um doubles as the zone steps, coarsest
  -- for the lowest zone. Passes coarser than the current zone's step are
  -- skipped, so a strong start (e.g. after a raster handoff) goes straight
  -- to fine steps. 0 = original fixed ladder (every pass runs).
  adaptive       = fd.param("align_adaptive", 0),
  zone_low       = fd.param("align_zone_low", 1e-6),    -- feedback units
  zone_high      = fd.param("align_zone_high", 400e-6), -- feedback units
  trace          = fd.param("align_trace", 0),          -- 1 = per-sample CSV trace
  scanlog        = fd.param("align_scanlog", 0),        -- 1 = ML record to data/xcan/
                                                        -- (paused 2026-10: off by default)
  -- Cycle-time trims (1 = on, 0 = original bobcat behaviour)
  skip_opposite  = fd.param("align_skip_opposite", 1),  -- skip -dir after +dir improved
  dir_memory     = fd.param("align_dir_memory", 1),     -- last winning direction first
  reuse_sample   = fd.param("align_reuse_sample", 1),   -- no re-measure of unmoved points
}

local function flag(v) return (v == true) or ((tonumber(v) or 0) ~= 0) end
local SKIP_OPP = flag(P.skip_opposite)
local DIR_MEM  = flag(P.dir_memory)
local REUSE    = flag(P.reuse_sample)

-- Early exit (bobcat XCAN Extended): the moment any measurement reaches
-- exit_threshold, the whole scan unwinds and STAYS at that point.
local exit_hit = false

-- ---- Helpers ----------------------------------------------------------------
local function split(s)
  local out = {}
  for w in string.gmatch(s, "[^,%s]+") do out[#out + 1] = w end
  return out
end

local AXES  = split(P.axes)
local STEPS = {}
for _, s in ipairs(split(P.steps_um)) do STEPS[#STEPS + 1] = tonumber(s) / 1000.0 end
assert(#AXES > 0, "align_axes is empty")
assert(#STEPS > 0, "align_steps_um is empty")

-- Adaptive zoning: signal -> the zone's step (mm). Zones are equal log slices
-- of [zone_low, zone_high]; below the range = coarsest, above = finest.
local ADAPTIVE = (P.adaptive == true) or ((tonumber(P.adaptive) or 0) ~= 0)
if ADAPTIVE then
  assert(P.zone_high > P.zone_low and P.zone_low > 0,
         "align_zone_low/high must satisfy 0 < low < high")
end
local function zone_step(v)
  if v <= P.zone_low  then return STEPS[1] end
  if v >= P.zone_high then return STEPS[#STEPS] end
  local frac = math.log(v / P.zone_low) / math.log(P.zone_high / P.zone_low)
  local idx  = math.floor(frac * #STEPS) + 1
  if idx > #STEPS then idx = #STEPS end
  return STEPS[idx]
end

local function pos(axis)
  local v = fd.read("motion/" .. P.device .. "/" .. axis)
  if v == nil then error("no position stream for " .. P.device .. "/" .. axis) end
  return v
end

-- Move time since the last logged sample (ms), for the scan log: what each
-- sample cost in motion (a recovery is several moves, so it accumulates).
local MOVE_MS = 0

local function move_abs(axis, target)
  local t0 = fd.clock()
  fd.move_relative{ device = P.device, axis = axis, delta_mm = target - pos(axis),
                    tol_mm = P.tol_mm, timeout_ms = P.move_timeout }
  MOVE_MS = MOVE_MS + (fd.clock() - t0) * 1000.0
end

local function move_rel(axis, delta)
  local t0 = fd.clock()
  fd.move_relative{ device = P.device, axis = axis, delta_mm = delta,
                    tol_mm = P.tol_mm, timeout_ms = P.move_timeout }
  MOVE_MS = MOVE_MS + (fd.clock() - t0) * 1000.0
end

-- One sample per measure (bobcat: single sample in the loop, no averaging).
-- With fresh_samples > 0 the settle is verified, not guessed: fd.read returns
-- value, count (monotonic publish counter), and we wait until fresh_samples
-- NEW publishes landed after this call started. The mechanics are already
-- settled when the move command returns (WaitArrive + PI motion-profile done);
-- the only stale thing is the instrument sample that integrated during the
-- move — publish #fresh_samples is guaranteed post-move. settle_ms caps the
-- wait (stalled stream degrades to the old behaviour: latest value).
local FRESH = tonumber(P.fresh_samples) or 0
local settle_warned = false

-- How the LAST measure() went, for the scan log: the publish count it started
-- from (n0), the fresh publishes it got, how long it waited, and whether the
-- settle_ms ceiling cut the wait short (a possibly stale sample).
local M = { n0 = 0, got = 0, wait_ms = 0, timeout = false }

local function measure()
  local mt0 = fd.clock()
  local _, n0 = fd.read(P.topic)
  M.n0, M.timeout = n0, false
  if FRESH <= 0 then
    fd.sleep(P.settle_ms)                        -- original blind settle
  else
    if P.settle_min_ms > 0 then fd.sleep(P.settle_min_ms) end
    local t0 = fd.clock()
    while true do
      local _, n = fd.read(P.topic)
      if n >= n0 + FRESH then break end
      if (fd.clock() - t0) * 1000.0 >= P.settle_ms then
        M.timeout = true
        if not settle_warned then
          settle_warned = true
          fd.logf("measure: only %d/%d fresh samples on %s within %g ms - using latest",
                  n - n0, FRESH, P.topic, P.settle_ms)
        end
        break
      end
      fd.sleep(10)
    end
  end
  local t0 = fd.clock()
  while true do
    local v, n = fd.read(P.topic)
    if v ~= nil then
      M.got, M.wait_ms = n - n0, (fd.clock() - mt0) * 1000.0
      if P.exit_threshold ~= 0 and v >= P.exit_threshold and not exit_hit then
        exit_hit = true
        fd.logf("exit threshold reached: %.4g >= %.4g", v, P.exit_threshold)
      end
      return v
    end
    if fd.clock() - t0 > 5.0 then error("no signal on " .. P.topic) end
    fd.sleep(50)
  end
end

local function snapshot()          -- current position of every scanned axis
  local s = {}
  for _, a in ipairs(AXES) do s[a] = pos(a) end
  return s
end

local function goto_snapshot(s)
  for _, a in ipairs(AXES) do move_abs(a, s[a]) end
end

-- ---- Per-sample record: scan log (align_scanlog=1) + CSV trace (align_trace=1)
-- With align_scanlog=1 every measured sample goes to fd.scanlog - the dataset
-- under data/xcan/ (docs/xcan-dataset.md); without it begin is never called,
-- so sample/finish are no-ops. C++ adds the full pose and the raw
-- readings after M.n0. With align_trace=1 it is ALSO one CSV line:
--   xcan,seq,t_s,phase,axis,dir,step_um,<axis>=pos...,v=value,dv=delta,dmove,note
-- dv / dmove are the change vs the PREVIOUS sample (value and per-axis um),
-- so the whole walk can be replayed. Lines land in the log band AND the
-- persistent daily log - extract with:  findstr "xcan," logs\firedog_*.log
local TRACE = (P.trace == true) or ((tonumber(P.trace) or 0) ~= 0)
local T  = { n = 0, t0 = fd.clock(), v = nil, pos = nil }   -- previous sample
local TC = { phase = "base", axis = "-", dir = 0, step = 0 } -- current context
local function trace(v, note)
  fd.scanlog.sample{ phase = TC.phase, axis = TC.axis, dir = TC.dir,
                     step_um = TC.step * 1000, v = v, note = note or "",
                     n0 = M.n0, fresh_req = FRESH, fresh_got = M.got,
                     wait_ms = M.wait_ms, settle_timeout = M.timeout,
                     move_ms = MOVE_MS }
  MOVE_MS = 0
  if not TRACE then return end
  T.n = T.n + 1
  local p    = snapshot()
  local cols = {}
  for _, a in ipairs(AXES) do cols[#cols + 1] = string.format("%s=%.5f", a, p[a]) end
  local dmove = {}
  if T.pos then
    for _, a in ipairs(AXES) do
      local d = (p[a] - T.pos[a]) * 1000.0   -- um
      if math.abs(d) > 0.01 then dmove[#dmove + 1] = string.format("d%s=%+.2f", a, d) end
    end
  end
  fd.logf("xcan,%d,%.2f,%s,%s,%+d,%g,%s,v=%.6g,dv=%+.4g,%s,%s",
          T.n, fd.clock() - T.t0, TC.phase, TC.axis, TC.dir, TC.step * 1000,
          table.concat(cols, ","), v, T.v and (v - T.v) or 0,
          #dmove > 0 and table.concat(dmove, " ") or "dmove=0", note or "")
  T.v, T.pos = v, p
end

-- Global peak over the whole alignment (value + full position snapshot).
local gpeak = { v = -math.huge, pos = nil }
local function update_gpeak(v)
  if v > gpeak.v then gpeak.v = v; gpeak.pos = snapshot() end
end

-- ---- The ratchet walk: one axis, one direction ------------------------------
-- v0: a fresh sample already taken at the current point (align_reuse_sample)
-- so the redundant "start" measurement is skipped. Returns maxV, maxPos,
-- improved (walked uphill at least once) and curV - the value measured at the
-- point the stage is left on, for the caller to reuse.
local function scan_direction(axis, step, dir, v0)
  TC.phase, TC.axis, TC.dir, TC.step = "scan", axis, dir, step
  local maxV = v0
  if maxV == nil then
    maxV = measure()
    trace(maxV, "start")
  end
  local maxPos = pos(axis)
  update_gpeak(maxV)
  local prev, consec, dist, movedFromMax = maxV, 0, 0.0, false
  local improved = false
  while not exit_hit and consec < P.consec_limit and
        dist < P.max_travel_um / 1000.0 do
    move_rel(axis, dir * step)
    local v = measure()
    dist = dist + step
    if v > maxV then
      maxV, maxPos = v, pos(axis)
      consec, movedFromMax, improved = 0, false, true
      update_gpeak(v)
      trace(v, "better")
    else
      consec = consec + 1
      movedFromMax = true
      if prev > 0 and (prev - v) / prev > P.sig_drop_pct / 100.0 then
        trace(v, "drop-break")
        break
      end
      trace(v, string.format("worse(%d/%d)", consec, P.consec_limit))
    end
    prev = v
  end
  local curV = maxV
  -- On threshold exit we STAY at the point that hit it - no walk-back.
  if movedFromMax and not exit_hit then
    move_abs(axis, maxPos)
    local v = measure()
    trace(v, "return-to-best")
    if v > maxV then maxV = v end
    curV = v
  end
  return maxV, maxPos, improved, curV
end

-- ---- One axis at one step size: probe both directions, keep the better -----
-- With align_dir_memory the direction that last improved this axis goes first;
-- with align_skip_opposite an uphill first direction ends the axis - the peak
-- is bracketed, the other way can only walk back down. Returns bestV plus,
-- when known, a fresh sample at the point the stage is left on.
local dirMem = {}

local function scan_axis(axis, step, v0)
  local p0 = pos(axis)
  local d1 = (DIR_MEM and dirMem[axis]) or 1
  local v1, p1, imp1, cur1 = scan_direction(axis, step, d1, REUSE and v0 or nil)
  if exit_hit then return v1, nil end     -- stay at the threshold point
  if imp1 then
    dirMem[axis] = d1
    if SKIP_OPP then
      fd.logf("  %s @ %gum: best=%.4g (one-sided)", axis, step * 1000, v1)
      return v1, cur1
    end
  end
  -- A no-improvement walk already returned to p0, so cur1 is a fresh sample
  -- there and the second direction can start on it.
  move_abs(axis, p0)
  local v2, p2, imp2, cur2 =
      scan_direction(axis, step, -d1, (REUSE and not imp1) and cur1 or nil)
  if exit_hit then return v2, nil end
  if imp2 then dirMem[axis] = -d1 end
  local bestV, bestP = v1, p1
  if v2 > v1 then bestV, bestP = v2, p2 end
  local curV = nil
  if math.abs(bestP - p2) <= P.tol_mm then
    curV = cur2                           -- already standing on the best point
  else
    move_abs(axis, bestP)
  end
  fd.logf("  %s @ %gum: best=%.4g", axis, step * 1000, bestV)
  return bestV, curV
end

-- ---- Main -------------------------------------------------------------------
fd.logf("hexapod_align: %s on %s, axes %s, steps %s um", P.device, P.topic,
        P.axes, P.steps_um)

-- Deterministic scan velocity: set ONCE up front (persists on the device),
-- so every micro-move has the same profile and settle behaviour.
if (tonumber(P.velocity_mm_s) or 0) > 0 then
  fd.set_velocity{ device = P.device, velocity_mm_s = P.velocity_mm_s }
end

if ADAPTIVE then
  local bounds = {}
  for i = 1, #STEPS - 1 do
    bounds[#bounds + 1] = string.format("%.3g",
        P.zone_low * (P.zone_high / P.zone_low) ^ (i / #STEPS))
  end
  fd.logf("adaptive ladder: %d log zones over [%.3g, %.3g] (bounds %s), steps %s um",
          #STEPS, P.zone_low, P.zone_high, table.concat(bounds, "/"), P.steps_um)
end

if TRACE then
  fd.logf("xcan,header: seq,t_s,phase,axis,dir,step_um,pos(%s),value,dv,dmove_um,note",
          P.axes)
end

-- Open the scan record: P is the whole resolved policy (off-policy learning
-- needs the parameters that produced the trajectory).
if flag(P.scanlog) then
  fd.scanlog.begin{ kind = "xcan", device = P.device, topic = P.topic,
                    axes = P.axes, params = P }
end
local passes_run, passes_skipped = 0, 0

-- Averaged baseline (bobcat averages ONLY the baseline).
local carryV = nil   -- fresh sample at the current point, reused as next start
local sum = 0
for i = 1, P.avg_count do
  local v = measure()
  trace(v, "baseline")
  sum = sum + v
  carryV = v
  if i < P.avg_count then fd.sleep(50) end
end
local baseline = sum / P.avg_count
fd.logf("baseline = %.4g", baseline)
if math.abs(baseline) < P.min_signal then
  error(string.format("baseline %.3g below min_signal %.3g - is the beam on?",
                      baseline, P.min_signal))
end
update_gpeak(baseline)

for _, step in ipairs(STEPS) do
  -- Adaptive: skip rungs coarser than what the signal zone calls for. The
  -- reference is the best reachable signal (current point or global peak) —
  -- a single noisy sample must not bounce the zone back to coarse.
  local skip = false
  if ADAPTIVE then
    local ref = math.max(carryV or 0, gpeak.v)
    local zs  = zone_step(ref)
    if step > zs * 1.001 then
      fd.logf("pass @ %g um skipped (adaptive: signal %.4g -> zone step %g um)",
              step * 1000, ref, zs * 1000)
      skip = true
    end
  end
  if skip then passes_skipped = passes_skipped + 1 end
  if not skip then
  passes_run = passes_run + 1
  fd.logf("pass @ %g um", step * 1000)
  for _, axis in ipairs(AXES) do
    local _, curV = scan_axis(axis, step, carryV)
    if exit_hit then goto scan_done end
    -- Global-peak recovery: if the signal here is worse than the best ever
    -- seen by more than improve_pct, go back to that best point.
    TC.phase, TC.axis, TC.dir = "check", axis, 0
    local cur = REUSE and curV or nil     -- the return-to-best sample IS the check
    if cur == nil then
      fd.sleep(100)
      cur = measure()
      trace(cur, "axis-done")
      if exit_hit then goto scan_done end
    end
    carryV = cur
    if cur > 0 and gpeak.pos ~= nil and
       (gpeak.v - cur) / cur > P.improve_pct / 100.0 then
      fd.logf("  recovering to global peak (%.4g > %.4g)", gpeak.v, cur)
      goto_snapshot(gpeak.pos)
      local rv = measure()
      trace(rv, "recovered")
      carryV = rv
    end
  end
  end   -- if not skip (adaptive rung gate)
end
::scan_done::

-- Finish: at the threshold point when the early exit fired, else at the best
-- point ever seen.
TC.phase, TC.axis, TC.dir, TC.step = "final", "-", 0, 0
local finalV = measure()
trace(finalV, "final")
if not exit_hit and gpeak.pos ~= nil and gpeak.v > finalV then
  goto_snapshot(gpeak.pos)
  finalV = measure()
  trace(finalV, "final@gpeak")
end

fd.scanlog.finish{ baseline = baseline, final = finalV, exit_hit = exit_hit,
                   exit_threshold = P.exit_threshold,
                   gain = (baseline > 0 and finalV > 0) and (finalV / baseline) or 0,
                   passes_run = passes_run, passes_skipped = passes_skipped }

fd.ctx.align_baseline = baseline
fd.ctx.align_best     = finalV
fd.ctx.align_delta    = finalV - baseline
fd.ctx.align_exit_hit = exit_hit
-- A ratio only means something when both readings are real positive signal;
-- on dark-noise runs it would be a ratio of noise samples.
fd.ctx.align_gain = (baseline > 0 and finalV > 0) and (finalV / baseline) or 0
fd.logf("hexapod_align done: baseline=%.4g best=%.4g delta=%+.4g gain=%s",
        baseline, finalV, fd.ctx.align_delta,
        fd.ctx.align_gain > 0 and string.format("%.2fx", fd.ctx.align_gain)
                               or "n/a (signal not positive)")
