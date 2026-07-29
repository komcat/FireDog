-- Active alignment: coordinate-descent hill climb, coarse-to-fine (bobcat XCAN port)
-- Port of bobcat_engine's CoarseToFineStrategy: for each step size (coarse->fine),
-- scan each axis in turn - probe BOTH directions with a ratchet walk, return to
-- the best point, recover to the global peak if the current signal fell off.
-- Parameters come from the recipe via ctx (fd.param = ctx value or default);
-- results land back in ctx (align_*) for dut_record / conditionals.

-- ---- Parameters (inject from the recipe with ctx_set "align_..." steps) ----
local P = {
  device         = fd.param("align_device", "hex-left"),
  topic          = fd.param("align_topic", "keithley/ch1"),   -- feedback signal
  axes           = fd.param("align_axes", "Z,Y,X"),           -- coordinate-descent order
  steps_um       = fd.param("align_steps_um", "10,5,2,1"),    -- coarse -> fine ladder
  settle_ms      = fd.param("align_settle_ms", 300),          -- after-move settle
  consec_limit   = fd.param("align_consec_limit", 3),   -- stop dir after N non-improvements
  sig_drop_pct   = fd.param("align_sig_drop_pct", 5),   -- hard break on a big relative drop
  max_travel_um  = fd.param("align_max_travel_um", 200),-- per-direction travel cap
  improve_pct    = fd.param("align_improve_pct", 1),    -- global-peak recovery threshold
  avg_count      = fd.param("align_avg_count", 3),      -- baseline averaging samples
  min_signal     = fd.param("align_min_signal", 0),     -- abort if |baseline| below this
  tol_mm         = fd.param("align_tol_mm", 0.001),     -- move arrival tolerance
  move_timeout   = fd.param("align_move_timeout_ms", 10000),
}

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

local function pos(axis)
  local v = fd.read("motion/" .. P.device .. "/" .. axis)
  if v == nil then error("no position stream for " .. P.device .. "/" .. axis) end
  return v
end

local function move_abs(axis, target)
  fd.move_relative{ device = P.device, axis = axis, delta_mm = target - pos(axis),
                    tol_mm = P.tol_mm, timeout_ms = P.move_timeout }
end

local function move_rel(axis, delta)
  fd.move_relative{ device = P.device, axis = axis, delta_mm = delta,
                    tol_mm = P.tol_mm, timeout_ms = P.move_timeout }
end

-- One sample after settle (bobcat: single sample in the loop, no averaging).
local function measure()
  fd.sleep(P.settle_ms)
  local t0 = fd.clock()
  while true do
    local v = fd.read(P.topic)
    if v ~= nil then return v end
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

-- Global peak over the whole alignment (value + full position snapshot).
local gpeak = { v = -math.huge, pos = nil }
local function update_gpeak(v)
  if v > gpeak.v then gpeak.v = v; gpeak.pos = snapshot() end
end

-- ---- The ratchet walk: one axis, one direction ------------------------------
local function scan_direction(axis, step, dir)
  local maxV, maxPos = measure(), pos(axis)
  update_gpeak(maxV)
  local prev, consec, dist, movedFromMax = maxV, 0, 0.0, false
  while consec < P.consec_limit and dist < P.max_travel_um / 1000.0 do
    move_rel(axis, dir * step)
    local v = measure()
    dist = dist + step
    if v > maxV then
      maxV, maxPos = v, pos(axis)
      consec, movedFromMax = 0, false
      update_gpeak(v)
    else
      consec = consec + 1
      movedFromMax = true
      if prev > 0 and (prev - v) / prev > P.sig_drop_pct / 100.0 then break end
    end
    prev = v
  end
  if movedFromMax then
    move_abs(axis, maxPos)
    local v = measure()
    if v > maxV then maxV = v end
  end
  return maxV, maxPos
end

-- ---- One axis at one step size: probe both directions, keep the better -----
local function scan_axis(axis, step)
  local p0 = pos(axis)
  local vP, pP = scan_direction(axis, step, 1)
  move_abs(axis, p0)
  local vN, pN = scan_direction(axis, step, -1)
  local bestV, bestP = vP, pP
  if vN > vP then bestV, bestP = vN, pN end
  move_abs(axis, bestP)
  fd.logf("  %s @ %gum: best=%.4g", axis, step * 1000, bestV)
  return bestV
end

-- ---- Main -------------------------------------------------------------------
fd.logf("hexapod_align: %s on %s, axes %s, steps %s um", P.device, P.topic,
        P.axes, P.steps_um)

-- Averaged baseline (bobcat averages ONLY the baseline).
local sum = 0
for i = 1, P.avg_count do
  sum = sum + measure()
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
  fd.logf("pass @ %g um", step * 1000)
  for _, axis in ipairs(AXES) do
    scan_axis(axis, step)
    fd.sleep(100)
    -- Global-peak recovery: if the signal here is worse than the best ever
    -- seen by more than improve_pct, go back to that best point.
    local cur = measure()
    if cur > 0 and gpeak.pos ~= nil and
       (gpeak.v - cur) / cur > P.improve_pct / 100.0 then
      fd.logf("  recovering to global peak (%.4g > %.4g)", gpeak.v, cur)
      goto_snapshot(gpeak.pos)
    end
  end
end

-- Finish at the best point ever seen, and report.
local finalV = measure()
if gpeak.pos ~= nil and gpeak.v > finalV then
  goto_snapshot(gpeak.pos)
  finalV = measure()
end

fd.ctx.align_baseline = baseline
fd.ctx.align_best     = finalV
fd.ctx.align_delta    = finalV - baseline
-- A ratio only means something when both readings are real positive signal;
-- on dark-noise runs it would be a ratio of noise samples.
fd.ctx.align_gain = (baseline > 0 and finalV > 0) and (finalV / baseline) or 0
fd.logf("hexapod_align done: baseline=%.4g best=%.4g delta=%+.4g gain=%s",
        baseline, finalV, fd.ctx.align_delta,
        fd.ctx.align_gain > 0 and string.format("%.2fx", fd.ctx.align_gain)
                               or "n/a (signal not positive)")
