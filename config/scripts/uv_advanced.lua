-- UV advanced: pulse UV_PLC2 for N rounds, re-align when the signal drops
--
-- Each round:
--   1. UV_PLC2 OFF, dwell 150 ms
--   2. UV_PLC2 ON  for 150 ms, then OFF (pulse_output: OFF even on Stop)
--   3. dwell 150 ms
--   4. wait 13 s (the UV cure)
-- for 10 rounds.
--
-- On the first 2 rounds only, Keithley ch1 is read before the trigger (T0,
-- the baseline) and again after the 13 s wait (T1). When
-- (T1 - T0) / T0 < -2 % the lens is re-aligned with an xcan (0.5 um ladder,
-- XYZ, 2000 ms settle floor) before the next round starts, so that round runs
-- longer than 13 s. Rounds 3..10 only pulse and wait.
--
-- Parameters (fd.param -> ctx keys; a recipe sets them with ctx_set steps
-- before its run_script step, or leaves the defaults):
--   uva_io               IO device                           (default "IOBottom")
--   uva_pin              UV trigger output                   (default "UV_PLC2")
--   uva_rounds           how many rounds                     (default 10)
--   uva_dwell_ms         each OFF / ON / OFF dwell, ms       (default 150)
--   uva_wait_ms          wait after the trigger, ms          (default 13000)
--   uva_check_rounds     rounds that read T0/T1              (default 2)
--   uva_drop_pct         xcan when T1 vs T0 is below -this % (default 2)
--   uva_topic            feedback signal                     (default "keithley/ch1")
--   uva_xcan_device      hexapod to xcan                     (default "hex-left")
--   uva_xcan_axes        XYZ | XY | XZ | YZ                  (default "XYZ")
--   uva_xcan_steps_um    xcan ladder                         (default "0.5")
--   uva_xcan_settle_ms   settle floor before each sample, ms (default 2000)
--   uva_xcan_velocity    xcan velocity mm/s (0 = keep)       (default 1)
--
-- Results (ctx, for dut_record / if steps), per checked round r:
--   uva_t0_ua_<r>, uva_t1_ua_<r>   the two readings, uA
--   uva_delta_pct_<r>              (T1 - T0) / T0 * 100
--   uva_xcan_<r>                   true when that round re-aligned
--   uva_xcan_count                 how many xcans ran in total

local P = {
  io            = fd.param("uva_io", "IOBottom"),
  pin           = fd.param("uva_pin", "UV_PLC2"),
  rounds        = math.floor(tonumber(fd.param("uva_rounds", 10)) or 10),
  dwell_ms      = math.floor(tonumber(fd.param("uva_dwell_ms", 150)) or 150),
  wait_ms       = math.floor(tonumber(fd.param("uva_wait_ms", 13000)) or 13000),
  check_rounds  = math.floor(tonumber(fd.param("uva_check_rounds", 2)) or 2),
  drop_pct      = tonumber(fd.param("uva_drop_pct", 2)) or 2,
  topic         = fd.param("uva_topic", "keithley/ch1"),
  xcan_device   = fd.param("uva_xcan_device", "hex-left"),
  xcan_axes     = fd.param("uva_xcan_axes", "XYZ"),
  xcan_steps    = fd.param("uva_xcan_steps_um", "0.5"),
  xcan_settle   = math.floor(tonumber(fd.param("uva_xcan_settle_ms", 2000)) or 2000),
  xcan_velocity = tonumber(fd.param("uva_xcan_velocity", 1)) or 1,
}

if P.rounds < 1 then error("uva_rounds must be >= 1") end

-- The bus carries amps; the numbers here are uA, like the xcan step's.
local function live_ua()
  local v = fd.read(P.topic)
  if v == nil then error("no feedback stream on " .. P.topic) end
  return v * 1e6
end

-- OFF, ON, OFF with a dwell after each. pulse_output drives the ON leg so the
-- output goes OFF even when Stop lands inside it.
local function trigger_uv()
  fd.set_output{ device = P.io, pin = P.pin, on = false }
  fd.sleep(P.dwell_ms)
  fd.pulse_output{ device = P.io, pin = P.pin, ms = P.dwell_ms }
  fd.sleep(P.dwell_ms)
end

local xcans = 0
fd.logf("uv_advanced: %d rounds on %s/%s, wait %d ms, check rounds 1..%d (drop > %g %%)",
        P.rounds, P.io, P.pin, P.wait_ms, P.check_rounds, P.drop_pct)

for r = 1, P.rounds do
  local check = r <= P.check_rounds
  local t0
  if check then
    t0 = live_ua()
    fd.ctx["uva_t0_ua_" .. r] = t0
  end

  fd.logf("uv_advanced: round %d/%d - trigger", r, P.rounds)
  trigger_uv()
  fd.sleep(P.wait_ms)

  if check then
    local t1 = live_ua()
    fd.ctx["uva_t1_ua_" .. r] = t1
    fd.ctx["uva_xcan_" .. r] = false
    if t0 == 0 then
      fd.logf("uv_advanced: round %d - T0 is 0 uA, no delta to judge; xcan skipped", r)
    else
      local delta = (t1 - t0) / t0 * 100
      fd.ctx["uva_delta_pct_" .. r] = delta
      fd.logf("uv_advanced: round %d - T0 %.4g uA, T1 %.4g uA, delta %.2f %%", r, t0, t1, delta)
      if delta < -P.drop_pct then
        fd.ctx["uva_xcan_" .. r] = true
        fd.logf("uv_advanced: round %d - drop below -%g %%, xcan %s (%s um, %s, settle %d ms)",
                r, P.drop_pct, P.xcan_device, P.xcan_steps, P.xcan_axes, P.xcan_settle)
        fd.xcan{
          device        = P.xcan_device,
          topic         = P.topic,
          axes          = P.xcan_axes,
          steps_um      = P.xcan_steps,
          settle_min_ms = P.xcan_settle,
          velocity_mm_s = P.xcan_velocity,
        }
        xcans = xcans + 1
        fd.logf("uv_advanced: round %d - xcan %s done, live %.4g uA", r, P.xcan_device, live_ua())
      end
    end
  end
end

fd.ctx.uva_xcan_count = xcans
fd.logf("uv_advanced: done - %d rounds, %d xcan(s)", P.rounds, xcans)
