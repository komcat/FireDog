-- Raster (area) scan: serpentine grid first-light search (bobcat GridScan port)
--
-- The move, defined: the grid hangs off the STARTING pose. The slow axis
-- (default X) begins at its start position X0 and only steps DOWN
-- (raster_slow_dir = -1), one row per step, covering slow_span_um total.
-- The fast axis (default Y) sweeps CENTERED on its start position Y0, from
-- Y0 - fast_span_um/2 to Y0 + fast_span_um/2, sampling every fast_step_um.
-- Rows alternate sweep direction (serpentine / bobcat snakePattern - no
-- flyback move, less backlash):
--
--   X0        :  Y-half ->  ->  ->  ->  Y+half
--   X0-step   :  Y+half <-  <-  <-  <-  Y-half
--   X0-2*step :  Y-half ->  ->  ->  ->  Y+half   ... etc
--
-- Every node is an ABSOLUTE target computed from the start pose (no drift
-- accumulation); one measurement per node (same fresh-sample settle as
-- hexapod_align). If raster_exit_threshold > 0 the scan stops-and-stays the
-- moment a sample reaches it (first light found - run XCAN next); otherwise
-- the scan runs and the stage returns to the best node seen.
--
-- Two cycle-time cuts (both gated on REAL signal - a row peak reaching
-- baseline * raster_signal_factor - so they can't fire on dark noise):
--   row stop  - a single spot means row peaks rise -> max -> fall as the slow
--               axis walks past it; after raster_row_stop consecutive rows
--               raster_row_drop_pct below the best row, the rest are dark and
--               the scan ends early (0 = always run the full grid).
--   tracking  - once a row saw signal, the next row sweeps only a
--               raster_track_span_um window centered on that row's peak
--               (clamped to the span; a row that loses the signal puts the
--               next one back on the full span).
-- Results land in ctx (raster_*) for dut_record / conditionals.

-- ---- Parameters (inject from a recipe with ctx_set "raster_..." steps) -----
local P = {
  device         = fd.param("raster_device", "hex-left"),
  topic          = fd.param("raster_topic", "keithley/ch1"),  -- feedback signal
  slow_axis      = fd.param("raster_slow_axis", "X"),
  slow_dir       = fd.param("raster_slow_dir", -1),     -- -1 = step down from start
  slow_span_um   = fd.param("raster_slow_span_um", 40), -- one-sided total travel
  slow_step_um   = fd.param("raster_slow_step_um", 4),  -- row pitch
  fast_axis      = fd.param("raster_fast_axis", "Y"),
  fast_span_um   = fd.param("raster_fast_span_um", 40), -- CENTERED: +/- half around start
  fast_step_um   = fd.param("raster_fast_step_um", 4),  -- sample pitch along a row
  settle_ms      = fd.param("raster_settle_ms", 300),   -- fresh-sample ceiling
  settle_min_ms  = fd.param("raster_settle_min_ms", 0), -- floor for slow receivers
  fresh_samples  = fd.param("raster_fresh_samples", 2), -- new publishes per measure
  exit_threshold = fd.param("raster_exit_threshold", 0),-- stop-and-stay target (0 = off)
  avg_count      = fd.param("raster_avg_count", 3),     -- baseline averaging samples
  -- Early finish: a single coupling spot means row peaks rise -> max -> fall
  -- as the slow axis walks past it; rows beyond the fall are dark.
  row_stop       = fd.param("raster_row_stop", 2),       -- stop after N declining rows (0 = full grid)
  row_drop_pct   = fd.param("raster_row_drop_pct", 30),  -- row is declining when this % below best row
  signal_factor  = fd.param("raster_signal_factor", 5),  -- peak >= baseline*factor = real signal
  -- Fast-axis tracking: once a row saw real signal, the next row only sweeps
  -- a narrow window centered on that row's peak (falls back to the full span
  -- the moment a tracked row loses the signal; always clamped to the span).
  track          = fd.param("raster_track", 1),          -- 0 = every row sweeps the full span
  track_span_um  = fd.param("raster_track_span_um", 16), -- tracked window width
  tol_mm         = fd.param("raster_tol_mm", 0.001),    -- move arrival tolerance
  move_timeout   = fd.param("raster_move_timeout_ms", 10000),
  velocity_mm_s  = fd.param("raster_velocity_mm_s", 1), -- scan velocity, set once
                                                        -- up front (0 = keep)
  trace          = fd.param("raster_trace", 0),         -- 1 = per-sample CSV trace
}

assert(P.slow_step_um > 0, "raster_slow_step_um must be > 0")
assert(P.fast_step_um > 0, "raster_fast_step_um must be > 0")
P.slow_dir = ((tonumber(P.slow_dir) or -1) >= 0) and 1 or -1   -- normalize to +/-1

local exit_hit = false

-- ---- Helpers (same shapes as hexapod_align) ---------------------------------
local function pos(axis)
  local v = fd.read("motion/" .. P.device .. "/" .. axis)
  if v == nil then error("no position stream for " .. P.device .. "/" .. axis) end
  return v
end

local function move_abs(axis, target)
  local delta = target - pos(axis)
  if math.abs(delta) < 1e-6 then return end   -- already there (sub-nm)
  fd.move_relative{ device = P.device, axis = axis, delta_mm = delta,
                    tol_mm = P.tol_mm, timeout_ms = P.move_timeout }
end

-- One sample per node, fresh-publish gated (see hexapod_align for rationale:
-- mechanics are settled when the move returns; only the instrument sample
-- that integrated during the move is stale - wait out fresh_samples publishes).
local FRESH = tonumber(P.fresh_samples) or 0
local settle_warned = false

local function measure()
  if FRESH <= 0 then
    fd.sleep(P.settle_ms)
  else
    local _, n0 = fd.read(P.topic)
    if P.settle_min_ms > 0 then fd.sleep(P.settle_min_ms) end
    local t0 = fd.clock()
    while true do
      local _, n = fd.read(P.topic)
      if n >= n0 + FRESH then break end
      if (fd.clock() - t0) * 1000.0 >= P.settle_ms then
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
    local v = fd.read(P.topic)
    if v ~= nil then
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

-- ---- Optional per-sample trace (raster_trace=1) -----------------------------
--   rast,seq,t_s,row,col,<slow>=pos,<fast>=pos,v=value,dv=delta,note
-- Extract with:  findstr "rast," logs\firedog_*.log
local TRACE = (P.trace == true) or ((tonumber(P.trace) or 0) ~= 0)
local T = { n = 0, t0 = fd.clock(), v = nil }
local function trace(v, row, col, note)
  if not TRACE then return end
  T.n = T.n + 1
  fd.logf("rast,%d,%.2f,%d,%d,%s=%.5f,%s=%.5f,v=%.6g,dv=%+.4g,%s",
          T.n, fd.clock() - T.t0, row, col,
          P.slow_axis, pos(P.slow_axis), P.fast_axis, pos(P.fast_axis),
          v, T.v and (v - T.v) or 0, note or "")
  T.v = v
end

-- ---- Main -------------------------------------------------------------------
-- Deterministic scan velocity: set ONCE up front (persists on the device),
-- so every grid move has the same profile and settle behaviour.
if (tonumber(P.velocity_mm_s) or 0) > 0 then
  fd.set_velocity{ device = P.device, velocity_mm_s = P.velocity_mm_s }
end

local s0 = pos(P.slow_axis)   -- start pose = grid origin
local f0 = pos(P.fast_axis)

local rows = math.floor(P.slow_span_um / P.slow_step_um + 0.5)   -- rows beyond row 0
local cols = math.floor(P.fast_span_um / P.fast_step_um + 0.5)   -- steps per sweep
local half = P.fast_span_um / 2.0

fd.logf("raster_scan: %s on %s, %s %s%g um (dir %+d, %d rows) x %s +/-%g um (%d cols), step %g/%g um = up to %d points",
        P.device, P.topic, P.slow_axis, P.slow_dir < 0 and "-" or "+",
        P.slow_span_um, P.slow_dir, rows + 1, P.fast_axis, half, cols + 1,
        P.slow_step_um, P.fast_step_um, (rows + 1) * (cols + 1))
if TRACE then
  fd.logf("rast,header: seq,t_s,row,col,%s,%s,value,dv,note", P.slow_axis, P.fast_axis)
end

-- Averaged baseline at the start pose (dark level - raster usually runs
-- with no signal yet, so unlike hexapod_align there is no min-signal abort).
local sum = 0
for i = 1, P.avg_count do
  local v = measure()
  trace(v, 0, 0, "baseline")
  sum = sum + v
  if i < P.avg_count then fd.sleep(50) end
end
local baseline = sum / P.avg_count
fd.logf("baseline = %.4g", baseline)

-- The start pose is itself a valid grid node (its value = the baseline), so
-- seed best with it: a scan that finds nothing better returns to the start.
local best   = { v = baseline, s = s0, f = f0, row = 0, col = 0 }
local points = 0

-- "Real signal" arming level for row-stop and tracking. A non-positive
-- baseline would let pure noise arm them, so both stay off in that case.
local TRACK    = (P.track == true) or ((tonumber(P.track) or 0) ~= 0)
local armLevel = baseline > 0 and baseline * P.signal_factor or math.huge
if armLevel == math.huge and (TRACK or P.row_stop > 0) then
  fd.logf("  baseline <= 0: row-stop / tracking disarmed (signal_factor gate unusable)")
end

local bestRowV, decline = -math.huge, 0
local seen      = false     -- any row with real signal yet
local prevArmed = false     -- previous row saw real signal
local prevPeakF = f0        -- previous row's peak position (fast axis, mm)

for r = 0, rows do
  if exit_hit then break end
  local sTgt = s0 + P.slow_dir * r * P.slow_step_um / 1000.0
  move_abs(P.slow_axis, sTgt)

  -- Fast window: full span, or - once the previous row saw real signal -
  -- a narrow window centered on that row's peak, clamped to the span.
  local lo, hi = -half, half                     -- offsets from f0, um
  if TRACK and prevArmed then
    local center = (prevPeakF - f0) * 1000.0
    lo = math.max(-half, center - P.track_span_um / 2.0)
    hi = math.min(half, center + P.track_span_um / 2.0)
  end
  local nCols = math.floor((hi - lo) / P.fast_step_um + 1e-9)

  local rowV, rowF = -math.huge, f0
  for c = 0, nCols do
    -- Serpentine: even rows sweep lo -> hi, odd rows come back.
    local k    = (r % 2 == 0) and c or (nCols - c)
    local fTgt = f0 + (lo + k * P.fast_step_um) / 1000.0
    move_abs(P.fast_axis, fTgt)
    local v = measure()
    points = points + 1
    if v > rowV then rowV, rowF = v, fTgt end
    if v > best.v then
      best = { v = v, s = sTgt, f = fTgt, row = r, col = k }
      trace(v, r, k, "peak")
    else
      trace(v, r, k, "")
    end
    if exit_hit then break end   -- stop-and-stay at the threshold node
  end
  if exit_hit then break end

  -- Row bookkeeping: arm on real signal; once seen, stop when row peaks have
  -- fallen row_drop_pct below the best row for row_stop consecutive rows -
  -- the spot is behind us, every remaining row is dark.
  prevArmed, prevPeakF = (rowV >= armLevel), rowF
  if prevArmed then seen = true end
  if P.row_stop > 0 and seen then
    if rowV > bestRowV then
      bestRowV, decline = rowV, 0
    elseif rowV < bestRowV * (1.0 - P.row_drop_pct / 100.0) then
      decline = decline + 1
      if decline >= P.row_stop then
        fd.logf("  row peaks declining %d rows past best (%.4g < %.4g) - stopping at row %d/%d",
                decline, rowV, bestRowV, r + 1, rows + 1)
        break
      end
    else
      decline = 0
    end
  end
end

-- Finish: stay put on a threshold exit, else return to the best node.
local finalV
if exit_hit then
  finalV = fd.read(P.topic)
else
  move_abs(P.slow_axis, best.s)
  move_abs(P.fast_axis, best.f)
  finalV = measure()
  trace(finalV, best.row, best.col, "final@peak")
end

fd.ctx.raster_baseline     = baseline
fd.ctx.raster_best         = best.v
fd.ctx.raster_final        = finalV
fd.ctx.raster_points       = points
fd.ctx.raster_exit_hit     = exit_hit
fd.ctx.raster_best_slow_um = (best.s - s0) * 1000.0   -- offset from start, um
fd.ctx.raster_best_fast_um = (best.f - f0) * 1000.0
fd.logf("raster_scan done: %d points, baseline=%.4g best=%.4g at %s%+.1fum %s%+.1fum%s",
        points, baseline, best.v, P.slow_axis, fd.ctx.raster_best_slow_um,
        P.fast_axis, fd.ctx.raster_best_fast_um,
        exit_hit and " (exit threshold hit - staying)" or "")
