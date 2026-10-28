-- UV advanced: two UV cure loops with fixed-step drift correction on a hexapod
-- (a port of Horizon's UV_SAA5)
--
-- Every pulse: output OFF 100 ms -> ON 250 ms -> OFF 150 ms, then the loop's
-- cure wait (16 s). pulse_output drives the ON leg, so the output goes OFF even
-- when Stop lands inside it.
--
-- Loop 1 - UV_PLC1, 6 pulses. Each pulse reads T0 before the trigger and T1
-- after the wait. When T1 is uva_l1_drop_pct (0.5 %) or more below T0, an X/Y
-- ladder of fixed 0.5 um steps, 0.5 s settle each, from the start pose:
--   1) X+0.5 um: better than T1 -> stay, done
--   2) else X-0.5 um: better than T1 -> stay, done
--   3) else back to the start, then the same on Y (Y+0.5 um, then Y-0.5 um)
--   4) nothing better -> back at the start pose
-- Each pulse is logged against the previous one and the loop start, with how
-- much of the drop the correction won back, and a summary table after the loop.
--
-- Loop 2 - UV_PLC2, 10 pulses. Records T2.0 before the first pulse and T2.i
-- after each. For pulses 1..uva_l2_correct_n (all 10 by default), when T2.i is
-- uva_l2_drop_pct (1 %) or more below the previous reading, an X-only
-- correction from X0, each 0.5 um shift made as 5 x 0.1 um (300 ms each) and
-- then a 1 s settle:
--   1) X+0.5 um: better than X0 -> stay
--   2) else back to X0, then X-0.5 um: better than X0 -> stay
--   3) else back to X0
--
-- The outputs are NOT switched off at the end: every pulse already leaves its
-- output OFF.
--
-- Signal: the MAGNITUDE of the feedback stream (keithley/ch1, in uA), averaged
-- over uva_samples reads. A photocurrent can read negative; "better" always
-- means more light. A read with no fresh sample (stream not publishing) is n/a,
-- and an n/a reading never triggers a correction.
--
-- Moves are ABSOLUTE targets off the pose read at the start of each correction
-- (move_axis_to), so "back to the start" lands on the start and steps never
-- stack up error from a stale position cache.
--
-- If the script is stopped mid-correction, the hexapod is deliberately left
-- where it stopped (no return to the start / X0).
--
-- Parameters (fd.param -> ctx keys; a recipe sets them with ctx_set steps
-- before its run_script step, or leaves the defaults):
--   uva_io             IO device                            (default "IOBottom")
--   uva_device         hexapod to correct                   (default "hex-left")
--   uva_topic          feedback signal                      (default "keithley/ch1")
--   uva_samples        reads averaged per measurement       (default 5)
--   uva_sample_ms      wait between reads, ms               (default 100)
--   uva_off_ms / uva_on_ms / uva_after_ms
--                      pulse shape: OFF, ON, OFF, ms        (default 100 / 250 / 150)
--   uva_step_um        correction step, um                  (default 0.5)
--   uva_l1_pin         loop 1 output                        (default "UV_PLC1")
--   uva_l1_n           loop 1 pulses                        (default 6)
--   uva_l1_wait_ms     loop 1 cure wait after each pulse    (default 16000)
--   uva_l1_drop_pct    correct when T1 is this % below T0   (default 0.5)
--   uva_l1_settle_ms   settle after each X/Y step, ms       (default 500)
--   uva_l2_pin         loop 2 output                        (default "UV_PLC2")
--   uva_l2_n           loop 2 pulses                        (default 10)
--   uva_l2_wait_ms     loop 2 cure wait after each pulse    (default 16000)
--   uva_l2_correct_n   pulses 1..N may correct              (default 10)
--   uva_l2_drop_pct    correct when this % below previous   (default 1.0)
--   uva_l2_substeps    each X shift made in this many steps (default 5)
--   uva_l2_substep_ms  wait after each sub-step, ms         (default 300)
--   uva_l2_settle_ms   settle after each shift, ms          (default 1000)
--
-- Results (ctx, for dut_record / if steps):
--   uva_base_ua, uva_final_ua         before loop 1 / after loop 2
--   uva_l1_t0_ua_<i>, uva_l1_t1_ua_<i>, uva_l1_end_ua_<i>, uva_l1_corr_<i>
--                                     loop 1 pulse i ("X+", "Y-", "none", "-", "n/a")
--   uva_l2_ua_<i>                     loop 2 reading after pulse i (uva_l2_ua_0 = before)
--   uva_corrections                   corrections run, both loops

local function num(key, default)
  return tonumber(fd.param(key, default)) or default
end
local function int(key, default)
  return math.floor(num(key, default))
end

local P = {
  io          = fd.param("uva_io", "IOBottom"),
  dev         = fd.param("uva_device", "hex-left"),
  topic       = fd.param("uva_topic", "keithley/ch1"),
  samples     = int("uva_samples", 5),
  sample_ms   = int("uva_sample_ms", 100),
  off_ms      = int("uva_off_ms", 100),
  on_ms       = int("uva_on_ms", 250),
  after_ms    = int("uva_after_ms", 150),
  step_mm     = num("uva_step_um", 0.5) / 1000.0,

  l1_pin      = fd.param("uva_l1_pin", "UV_PLC1"),
  l1_n        = int("uva_l1_n", 6),
  l1_wait     = int("uva_l1_wait_ms", 16000),
  l1_drop     = -math.abs(num("uva_l1_drop_pct", 0.5)),
  l1_settle   = int("uva_l1_settle_ms", 500),

  l2_pin      = fd.param("uva_l2_pin", "UV_PLC2"),
  l2_n        = int("uva_l2_n", 10),
  l2_wait     = int("uva_l2_wait_ms", 16000),
  l2_correct  = int("uva_l2_correct_n", 10),
  l2_drop     = -math.abs(num("uva_l2_drop_pct", 1.0)),
  l2_substeps = math.max(1, int("uva_l2_substeps", 5)),
  l2_sub_ms   = int("uva_l2_substep_ms", 300),
  l2_settle   = int("uva_l2_settle_ms", 1000),
}
local MIN_GAIN_PCT = 0.0   -- a loop 1 direction must beat T1 by more than this

if P.samples < 1 then error("uva_samples must be >= 1") end
if P.step_mm <= 0 then error("uva_step_um must be > 0") end

local corrections = 0

-- fd.ctx takes a bool, number or string - never nil - so an n/a reading leaves
-- its key unset instead of stopping the cure.
local function put(key, v)
  if v ~= nil then fd.ctx[key] = v end
end

-- ---- Signal ------------------------------------------------------------------
-- Average magnitude, uA, of the samples read; nil when the stream published
-- nothing new across the whole measurement (a frozen value is no measurement).
local function read_ua()
  local _, c0 = fd.read(P.topic)
  local sum, ok = 0, 0
  for _ = 1, P.samples do
    local v = fd.read(P.topic)
    if v ~= nil then sum, ok = sum + math.abs(v) * 1e6, ok + 1 end
    fd.sleep(P.sample_ms)
  end
  local _, c1 = fd.read(P.topic)
  if ok == 0 or c1 == c0 then return nil end
  return sum / ok
end

local function fmt(v)
  if v == nil then return "n/a" end
  return string.format("%.4f", v)
end

-- Percent change of v vs ref; nil if either is missing.
local function pct(v, ref)
  if v == nil or ref == nil then return nil end
  if math.abs(ref) < 1e-12 then return 0 end
  return (v - ref) / math.abs(ref) * 100
end

local function fmtPct(p)
  if p == nil then return "n/a" end
  return string.format("%.2f %%", p)
end

-- ---- Output ------------------------------------------------------------------
local function pulse(pin)
  fd.set_output{ device = P.io, pin = pin, on = false }
  fd.sleep(P.off_ms)
  fd.pulse_output{ device = P.io, pin = pin, ms = P.on_ms }
  fd.sleep(P.after_ms)
end

-- ---- Motion ------------------------------------------------------------------
local function pose()
  fd.read_position{ device = P.dev, ctx_prefix = "uva_pos" }
  local x, y = fd.ctx.uva_pos_x, fd.ctx.uva_pos_y
  if x == nil or y == nil then error(P.dev .. " reports no X/Y position") end
  return x, y
end

-- Absolute single-axis move ("X" or "Y").
local function move_to(axis, v)
  if axis == "X" then
    fd.move_axis_to{ device = P.dev, axes = "X", x_mm = v }
  else
    fd.move_axis_to{ device = P.dev, axes = "Y", y_mm = v }
  end
end

-- ---- Loop 1 correction: X then Y, fixed steps --------------------------------
-- Try start+step, then start-step, on one axis (reading r0 at the start). A
-- direction that beats r0 is kept: returns its reading and name. Otherwise the
-- stage goes back to the start and nil is returned.
local function try_axis(tag, axis, start, r0)
  move_to(axis, start + P.step_mm)
  fd.sleep(P.l1_settle)
  local rPlus = read_ua()
  local gPlus = pct(rPlus, r0)
  fd.logf("%s  %s+%.1fum: %s uA  (%s vs start)", tag, axis, P.step_mm * 1000, fmt(rPlus), fmtPct(gPlus))
  if gPlus ~= nil and gPlus > MIN_GAIN_PCT then
    fd.logf("%s  %s+ improved, staying there", tag, axis)
    return rPlus, axis .. "+"
  end

  move_to(axis, start - P.step_mm)
  fd.sleep(P.l1_settle)
  local rMinus = read_ua()
  local gMinus = pct(rMinus, r0)
  fd.logf("%s  %s-%.1fum: %s uA  (%s vs start)", tag, axis, P.step_mm * 1000, fmt(rMinus), fmtPct(gMinus))
  if gMinus ~= nil and gMinus > MIN_GAIN_PCT then
    fd.logf("%s  %s- improved, staying there", tag, axis)
    return rMinus, axis .. "-"
  end

  move_to(axis, start)
  fd.sleep(P.l1_settle)
  return nil
end

-- The first direction that improves ends the correction. Returns the reading
-- where the stage ends up and the direction kept ("none" = back at the start).
local function correct_xy(tag, r0)
  corrections = corrections + 1
  local x0, y0 = pose()
  fd.logf("%s  correcting %s from X %.5f Y %.5f, start %s uA", tag, P.dev, x0, y0, fmt(r0))
  local r, dir = try_axis(tag, "X", x0, r0)
  if r ~= nil then return r, dir end
  r, dir = try_axis(tag, "Y", y0, r0)
  if r ~= nil then return r, dir end
  r = read_ua()
  fd.logf("%s  no direction improved, back at start: %s uA", tag, fmt(r))
  return r, "none"
end

-- Share of the T0 -> T1 drop the correction won back (tc = after correction).
local function recovery(t0, t1, tc)
  if t0 == nil or t1 == nil or tc == nil or t0 - t1 <= 1e-12 then return nil end
  return (tc - t1) / (t0 - t1) * 100
end

local function loop1()
  local pin, n = P.l1_pin, P.l1_n
  fd.logf("uv_advanced: loop 1 - %s x %d, wait %d ms, correct on a drop of %.2f %%",
          pin, n, P.l1_wait, -P.l1_drop)
  local rows = {}
  local ref, prevT0, prevT1 = nil, nil, nil
  for i = 1, n do
    local tag = string.format("%s %d/%d", pin, i, n)
    local t0 = read_ua()
    if ref == nil then ref = t0 end
    pulse(pin)
    fd.sleep(P.l1_wait)
    local t1 = read_ua()
    local d1 = pct(t1, t0)
    fd.logf("%s  T0=%s  T1=%s uA  (%s)", tag, fmt(t0), fmt(t1), fmtPct(d1))
    if i > 1 then
      fd.logf("%s  vs previous: T0 %s, T1 %s  |  T1 vs loop start %s", tag,
              fmtPct(pct(t0, prevT0)), fmtPct(pct(t1, prevT1)), fmtPct(pct(t1, ref)))
    end

    local tEnd, corr = t1, "-"
    if d1 == nil then
      fd.logf("%s  reading n/a, no correction", tag)
      corr = "n/a"
    elseif d1 <= P.l1_drop then
      local tc, dir = correct_xy(tag, t1)
      tEnd, corr = tc, dir
      fd.logf("%s  after correction: %s uA  (%s vs T1, %s vs T0, recovered %s of the drop)",
              tag, fmt(tc), fmtPct(pct(tc, t1)), fmtPct(pct(tc, t0)),
              fmtPct(recovery(t0, t1, tc)))
    end

    rows[i] = { t0 = t0, t1 = t1, d1 = d1, corr = corr, tEnd = tEnd }
    put("uva_l1_t0_ua_" .. i, t0)
    put("uva_l1_t1_ua_" .. i, t1)
    put("uva_l1_end_ua_" .. i, tEnd)
    put("uva_l1_corr_" .. i, corr)
    prevT0, prevT1 = t0, t1
  end

  fd.logf("%s summary (loop start T0=%s uA):", pin, fmt(ref))
  fd.log("  #  T0        T1        T1vsT0    corr  end       end vs start")
  for i, r in ipairs(rows) do
    fd.logf("  %d  %s  %s  %s  %s  %s  %s", i, fmt(r.t0), fmt(r.t1), fmtPct(r.d1),
            r.corr, fmt(r.tEnd), fmtPct(pct(r.tEnd, ref)))
  end
end

-- ---- Loop 2 correction: X only, in sub-steps ---------------------------------
-- Walk X from `from` to `to` in P.l2_substeps steps, then settle; returns the
-- reading there.
local function walk_x(from, to)
  for k = 1, P.l2_substeps do
    move_to("X", from + (to - from) * k / P.l2_substeps)
    fd.sleep(P.l2_sub_ms)
  end
  fd.sleep(P.l2_settle)
  return read_ua()
end

-- X correction from X0 (reading r0 there). Returns the reading where X ends up.
local function correct_x(tag, r0)
  corrections = corrections + 1
  local x0 = pose()
  local s = P.step_mm
  fd.logf("%s  correcting %s X from X0 %.5f: %s uA", tag, P.dev, x0, fmt(r0))

  local rPlus = walk_x(x0, x0 + s)
  fd.logf("%s  X+%.1fum: %s uA  (%s vs X0)", tag, s * 1000, fmt(rPlus), fmtPct(pct(rPlus, r0)))
  if rPlus ~= nil and rPlus > r0 then
    fd.logf("%s  X+ improved, staying there", tag)
    return rPlus
  end

  local rBack = walk_x(x0 + s, x0)
  fd.logf("%s  back at X0: %s uA  (%s vs X0)", tag, fmt(rBack), fmtPct(pct(rBack, r0)))
  local rMinus = walk_x(x0, x0 - s)
  fd.logf("%s  X-%.1fum: %s uA  (%s vs X0)", tag, s * 1000, fmt(rMinus), fmtPct(pct(rMinus, r0)))
  if rMinus ~= nil and rMinus > r0 then
    fd.logf("%s  X- improved, staying there", tag)
    return rMinus
  end

  local rEnd = walk_x(x0 - s, x0)
  fd.logf("%s  neither direction improved, back at X0: %s uA  (%s vs X0)",
          tag, fmt(rEnd), fmtPct(pct(rEnd, r0)))
  return rEnd
end

local function loop2()
  local pin, n = P.l2_pin, P.l2_n
  fd.logf("uv_advanced: loop 2 - %s x %d, wait %d ms, correct pulses 1..%d on a drop of %.2f %%",
          pin, n, P.l2_wait, math.min(P.l2_correct, n), -P.l2_drop)
  local readings = {}
  local prev = read_ua()
  put("uva_l2_ua_0", prev)
  fd.logf("%s  T2.0=%s uA", pin, fmt(prev))
  for i = 1, n do
    local tag = string.format("%s %d/%d", pin, i, n)
    pulse(pin)
    fd.sleep(P.l2_wait)
    local t = read_ua()
    readings[i] = fmt(t)
    put("uva_l2_ua_" .. i, t)
    local d = pct(t, prev)
    fd.logf("%s  T2.%d=%s uA  (%s vs previous %s)", tag, i, fmt(t), fmtPct(d), fmt(prev))

    if i <= P.l2_correct then
      if d == nil then
        fd.logf("%s  reading n/a, no correction", tag)
      elseif d <= P.l2_drop then
        local after = correct_x(tag, t)
        if after ~= nil then t = after end   -- the next pulse compares against where X ended up
      end
    end
    if t ~= nil then prev = t end
  end
  fd.logf("%s  T2.1..%d: %s", pin, n, table.concat(readings, ", "))
end

-- ---- Run ---------------------------------------------------------------------
local base = read_ua()
put("uva_base_ua", base)
fd.logf("uv_advanced: baseline %s uA on %s, correcting %s", fmt(base), P.topic, P.dev)

loop1()
loop2()

local final = read_ua()
put("uva_final_ua", final)
fd.ctx.uva_corrections = corrections
fd.logf("uv_advanced: UV sequence completed - final %s uA (%s vs baseline %s uA), %d correction(s)",
        fmt(final), fmtPct(pct(final, base)), fmt(base), corrections)
