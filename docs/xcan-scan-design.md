# XCAN scan: coordinate-descent alignment, adaptive step ladder, and zero-wait timing

The XCAN active-alignment scan is a Lua script — `config/scripts/hexapod_align.lua` — run
by `LuaEngine` through bindings generated from the `CommandRegistry`. Port of
bobcat_engine's `CoarseToFineStrategy` (XCAN), plus cycle-time work that is FireDog-specific.

Three launch surfaces, one script:

| Surface | Entry | Parameter source |
|---|---|---|
| Recipe / program step | `xcan` command (`command/CommandsSystem.cpp`) | step form params → ctx keys → `fd.param` |
| Run page, Scan band → XCAN tab | `App::StartXcanPreset` | `XcanPreset` + `XcanTarget` → `LuaEngine::Env::initialCtx` |
| Motion page → Scanning tab | same `initialCtx` mechanism | on-screen form |

Every parameter reaches the script the same way: `fd.param("align_x", default)` reads ctx
key `align_x` if present, else the default. Results go back into ctx as `align_baseline`,
`align_best`, `align_delta`, `align_gain`, `align_exit_hit`.

---

## 1. Algorithm

Coordinate descent with a coarse→fine step ladder. Feedback = one DataBus topic
(`keithley/ch1` by default), maximised.

```
set_velocity(1 mm/s)                       -- once, for deterministic settle
baseline = mean(measure() x avg_count)     -- the ONLY averaging in the scan
abort if |baseline| < min_signal

for step in STEPS (e.g. 10, 5, 2, 1 um):   -- adaptive mode may skip rungs (§3)
    for axis in AXES (default Z, Y, X):
        scan_axis(axis, step)
        global-peak recovery check
final: measure(); if the global peak is better, return to it
```

### `scan_direction(axis, step, dir)` — the ratchet walk

Repeat while not `exit_hit`:

1. `move_rel(axis, dir * step)` (blocking; arrival + settle guaranteed — see
   `docs/pi-hexapod-motion-design.md` §5)
2. `v = measure()`
3. `v > maxV` → new best, remember position, `consec = 0`, `improved = true`
   else → `consec += 1`

Stop conditions:

| Condition | Parameter | Default |
|---|---|---|
| N consecutive non-improvements | `align_consec_limit` | 3 |
| Relative drop `(prev - v)/prev` exceeds | `align_sig_drop_pct` | **15 %** |
| Travel cap in this direction | `align_max_travel_um` | 200 µm |
| Sample reached the exit target | `align_exit_threshold` | 0 = off |

On exit the stage walks back to the best point and re-measures ("return-to-best") — unless
the threshold exit fired, in which case it **stops and stays** on the point that hit it.

> `sig_drop_pct` is 15, not bobcat's 5: on this device noise dips reach ~6 % while true
> post-peak drops are 50 %+, so 5 killed productive walks (a −5.8 % dip ended a good 2 µm
> Z walk).

### `scan_axis(axis, step)` — both directions

Probes direction `d1`, then `-d1`, keeps the better peak. Two structural trims (both
default ON; set the param to 0 for original bobcat behaviour):

| Trim | Param | Rule |
|---|---|---|
| Skip opposite | `align_skip_opposite` | a first direction that walked uphill already **brackets** the peak; the other way can only walk downhill → axis done |
| Direction memory | `align_dir_memory` | each axis starts with the direction that last improved it |
| Reuse sample | `align_reuse_sample` | a point that was just measured is not measured again (direction starts, axis-done checks, return-to-best doubles as the check sample) |

### Global-peak recovery

Every measurement updates `gpeak = {value, full position snapshot}`. After each axis, if
the current signal is worse than `gpeak` by more than `align_improve_pct` (default 1 %),
the scan drives back to the peak snapshot. This is the guard against a fine pass wandering
off a peak found during a coarse pass.

### Early exit

`align_exit_threshold` > 0 (entered in the UI/step form as **µA**, injected ×1e-6): the
first sample ≥ threshold sets `exit_hit`, every loop unwinds, and the scan **stays where it
is** — no walk-back, no return to `gpeak`.

---

## 2. Trace format (`align_trace = 1`)

One CSV line per measured sample, into the log band **and** the daily log
(`logs/firedog_YYYY-MM-DD.log`). Extract with `findstr "xcan," logs\firedog_*.log`.

```
xcan,seq,t_s,phase,axis,dir,step_um,<axis>=pos...,v=value,dv=delta,dmove_um,note
```

`phase` ∈ `base | scan | check | final`; `note` ∈ `start | better | worse(n/N) |
drop-break | return-to-best | axis-done | recovered | baseline | final | final@gpeak`.
`dv`/`dmove` are deltas vs the previous sample, so a whole walk is replayable offline.

---

## 3. Adaptive step ladder (`align_adaptive`)

Rationale: the signal level tells you roughly how far you are from the peak. For a
Gaussian coupling profile, distance from peak ∝ √(log(peak/value)) — so **log** signal
zones map cleanly onto step sizes.

The range `[align_zone_low, align_zone_high]` (feedback units; the UI takes µA and injects
×1e-6) is split into `#STEPS` **equal log sections**. Zone *i* owns `STEPS[i]`, coarsest for
the lowest zone. Zone boundaries:

```
bound_i = zone_low * (zone_high / zone_low)^(i / #STEPS)     i = 1 .. #STEPS-1
zone_step(v): v <= zone_low -> STEPS[1];  v >= zone_high -> STEPS[#STEPS]
              idx = floor( log(v/zone_low) / log(zone_high/zone_low) * #STEPS ) + 1
```

Gate: a pass whose step is **coarser** than `zone_step(ref) * 1.001` is skipped and logged.
`ref = max(carryV, gpeak.v)` — the best *reachable* signal, deliberately not a single raw
sample, so one noisy reading cannot bounce the ladder back to coarse.

Effect: a strong start (e.g. handed off from `raster_scan.lua`) goes straight to fine steps.

> **Tuning rule learned on hardware:** `zone_low_ua` must equal the **raster handoff
> level** (5 µA in the qualified pipeline), never below it. With `zone_low = 1` the first
> boundary lands at 4.47 µA — under the handoff — so the coarse 5 µm pass is skipped and
> the 2 µm walk gets captured by a micro-lobe at ~6.7 µA. **The coarse pass is the lobe
> immunity; adaptive must not gate it off below the handoff level.**

Command defaults: `adaptive = false`, `zone_low_ua = 1`, `zone_high_ua = 400`. The Run
page's XCAN band always launches with `align_adaptive = 1` and takes both zone bounds from
the active target slot.

---

## 4. Timing — the zero-wait settle concept

### 4.1 What the wait used to be

Original bobcat behaviour: after each move, sleep a fixed settle time (300 ms), then read
the latest value. That delay was paying for two different things at once — mechanical
settle **and** measurement freshness — blind, with no evidence either had happened. At
~90–110 samples per scan it dominated the cycle time.

### 4.2 Why the mechanical part is already zero

Nothing is left to wait for mechanically when a move returns:

* `WaitArrive` — every targeted axis within `tol_mm` (0.001 mm in scans), verified from
  live 20 Hz positions.
* `WaitSettle` — `PI_IsMoving` false **and** `PI_qONT` true (the controller's own settling
  window / settling time).
* Velocity fixed at 1 mm/s up front, so every micro-move has the same profile.

So the correct mechanical settle delay in the script is **zero**. (See
`docs/pi-hexapod-motion-design.md` §5.)

### 4.3 What actually remains stale: the instrument sample

The Keithley integrates continuously. The sample latched at the moment the move finished
may have been integrating **while the stage was travelling** — it is a physically mixed
sample, not a measurement of the new position. That is the only real staleness left, and it
is a *sample-count* problem, not a *time* problem.

**Fresh-sample gating** (`align_fresh_samples`, default 2) solves it with evidence instead
of a delay. `fd.read(topic)` returns `(value, count)` where `count` is the DataBus topic's
monotonic publish counter, read **before** the value (so a racing publish can only make the
value fresher than the gate, never staler):

```lua
local _, n0 = fd.read(P.topic)                  -- publish counter at the start
if P.settle_min_ms > 0 then fd.sleep(P.settle_min_ms) end   -- precautionary floor
local t0 = fd.clock()
while true do
  local _, n = fd.read(P.topic)
  if n >= n0 + FRESH then break end             -- 2 new publishes landed
  if (fd.clock() - t0)*1000 >= P.settle_ms then ... break end   -- ceiling, warn once
  fd.sleep(10)
end
return fd.read(P.topic)                         -- the fresh value
```

* `fresh_samples = 2` — publish #1 may straddle the move; **publish #2 is guaranteed
  fully post-move**.
* `align_settle_ms` (default 150 ms in the Run presets, 300 ms script default) is no longer
  a delay — it is the **timeout ceiling**. If the stream stalls, the wait degrades to the
  old "use the latest value" behaviour and logs a one-shot warning.
* `fresh_samples = 0` restores the original blind sleep.

The wait therefore self-scales with the instrument: at NPLC 0.1 (~80 Hz) it costs ~25 ms;
at NPLC 1 (~110 ms bench cadence) ~220 ms. It never costs *more* than the signal actually
needs, and it is never *shorter* than one complete integration.

### 4.4 The precautionary floor — `settle_min_ms`

**Unit: milliseconds** (`settle_min_ms`, integer, range 0–5000; script param
`align_settle_min_ms`). Script/command default **0** — the pure zero-wait path.

Production setting is **50 ms**, and it is deliberately a *precaution*, not a requirement:
the fresh-sample gate proves the Keithley integration is post-move, but it cannot see the
analogue chain in front of it (photodiode + TIA / slow receiver front-end), where a genuine
electrical time constant can outlive the mechanical settle. 50 ms buys 100 ms-grade signal
quality for +0.5 s on a full scan. Measured on the bench (4×10 scans, hex-left, qONT +
1 mm/s build):

| `settle_min_ms` | Mean scan time | Stalls / 10 |
|---|---|---|
| 0 | 14.9 s | 3 |
| **50 (production)** | **15.4 s** | 2 |
| 100 | 18.3 s | 1 |
| 200 | 26.5 s | 2 |

The stall rate (~20 %) is **settle-independent** — stalls are the coarse pass ratcheting
onto a repeatable side lobe (~44, ~78, ~127 µA against a 400–470 µA main peak), which no
amount of settle changes. So there is no timing justification for going above 50 ms; the
fix for stalls is a recipe retry guard (§6), not a longer wait.

### 4.5 Cycle-time result

| Build | Scan time | Samples |
|---|---|---|
| Original (blind 300 ms settle, two-sided always, always re-measure) | 31.6 s | 114 |
| + `skip_opposite`, `dir_memory`, `reuse_sample`, `fresh_samples = 2` | **18.2 s** | 91 |

Coarse preset, hex-left, `keithley/ch1`, 10/5/2/1 µm, NPLC 1 (~110 ms stationary cadence).
Confirmed on hardware. Raster → adaptive-XCAN pipeline: ~17 s combined, 9/9 exit hits.

### 4.6 Fixed waits that remain in the script (and why)

| Site | Wait | Reason |
|---|---|---|
| Fresh-sample poll | `fd.sleep(10)` | polling granularity of the count gate |
| Baseline averaging, between samples | `fd.sleep(50)` | decorrelate the `avg_count` baseline samples |
| Axis-done check when no reusable sample exists | `fd.sleep(100)` | only when `reuse_sample = 0` |
| "no signal on topic" guard | `fd.sleep(50)`, 5 s cap | stream-dead detection, then `error()` |

Every one of these goes through `ExecutionContext::Sleep`, which slices at 20 ms and
returns false on stop — so **Stop is honoured inside every wait**, and the Lua `count`/`line`
hook kills pure-Lua loops independently.

---

## 5. Parameter reference

| ctx key | Command/preset param | Default | Meaning |
|---|---|---|---|
| `align_device` | `device` | — | hexapod name |
| `align_topic` | `topic` | `keithley/ch1` | feedback stream |
| `align_axes` | preset `axes` | `Z,Y,X` | coordinate-descent order |
| `align_steps_um` | `steps_um` | `10,5,2,1` | ladder (adaptive: the zone steps) |
| `align_exit_threshold` | `exit_target_ua` ×1e-6 | 0 | stop-and-stay target |
| `align_adaptive` | `adaptive` | false (Run band: on) | signal-zoned ladder |
| `align_zone_low` / `align_zone_high` | `zone_low_ua` / `zone_high_ua` ×1e-6 | 1 / 400 µA | zone range |
| `align_fresh_samples` | preset `freshSamples` | 2 | new publishes per measure (0 = blind sleep) |
| `align_settle_ms` | preset `settleMs` | 150 (preset) / 300 (script) | fresh-sample **ceiling** |
| `align_settle_min_ms` | `settle_min_ms` | 0 (production: 50 ms) | precautionary floor |
| `align_avg_count` | preset `avgCount` | 3 | baseline averaging only |
| `align_consec_limit` | preset `consecLimit` | 3 | non-improvements before a direction ends |
| `align_sig_drop_pct` | preset `sigDropPct` | 15 | hard break on a relative drop |
| `align_max_travel_um` | preset `maxTravelUm` | 200 | per-direction travel cap |
| `align_improve_pct` | preset `improvePct` | 1 | global-peak recovery threshold |
| `align_min_signal` | preset `minSignal` | 0 | abort if the baseline is below this |
| `align_tol_mm` | preset `tolMm` | 0.001 | move arrival tolerance |
| `align_move_timeout_ms` | preset `moveTimeoutMs` | 10000 | per-move timeout |
| `align_velocity_mm_s` | `velocity_mm_s` | 1.0 | set once up front; 0 = keep |
| `align_trace` | `trace` | false | per-sample CSV trace |
| `align_skip_opposite` / `align_dir_memory` / `align_reuse_sample` | preset toggles | 1 | cycle-time trims; 0 = bobcat behaviour |

Outputs: `align_baseline`, `align_best`, `align_delta`, `align_gain` (0 unless both
readings are positive), `align_exit_hit`.

---

## 6. Known issues / operating guidance

1. **Side lobes.** ~20 % of coarse runs settle on a repeatable lobe (~6.7, 44, 78, 127 µA).
   Mitigation is a recipe retry guard, not tuning: `loop while ctx align_best < 0.0002`,
   `max_iterations 3`, with a move back to the start position inside the loop before the
   `xcan` step.
2. **Exit threshold latches on ONE sample.** The final re-measure can land ~7 % below the
   trip value (e.g. trip 409 µA → final 381 µA). A confirm-on-exit (require N consecutive
   samples ≥ threshold) is discussed but **not implemented**.
3. **Adaptive `zone_low` must sit at or above the handoff signal level** (§3).
4. **Concurrency.** `App::OrchestrationBusy()` keeps the Lua engine, the recipe engine and
   the UI path executor mutually exclusive — an XCAN run blocks a recipe start and vice versa.

Related: `docs/pi-hexapod-motion-design.md` (arrival/settle contract),
`docs/keithley-polling-design.md` (sample cadence, publish counter),
`docs/lua-tier-design.md` (bindings, stop/debug hooks).
