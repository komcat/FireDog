# Climb Z + XY raster: the gap-closing search between first light and the peak

`config/scripts/climb_raster.lua`, run by `LuaEngine` through bindings generated
from the `CommandRegistry`. Port of Horizon's `RasterClimbEngine`
(`src/Horizon.Workflow/RasterClimb/`, C#), restructured onto FireDog's script
tier so it shares the measure/settle contract with `hexapod_align.lua` and
`raster_scan.lua` instead of reimplementing it.

Two launch surfaces, one script — the same shape `docs/xcan-scan-design.md`
describes:

| Surface | Entry | Parameter source |
|---|---|---|
| Recipe / program step | `climb_raster` command (`command/CommandsSystem.cpp`) | step form params → ctx keys → `fd.param` |
| Run page, Scan band → Climb tab | `App::StartClimbPreset` | `ClimbPreset` + the active `XcanTarget` → `LuaEngine::Env::initialCtx` |

Every parameter reaches the script the same way: `fd.param("climb_z_step_um",
default)` reads ctx key `climb_z_step_um` if present, else the default. Results
go back into ctx as `climb_*`.

**Where it sits in the pipeline.** `raster_scan` finds first light in a plane at
a fixed gap. `xcan` polishes a peak with a coordinate-descent ladder. Neither
closes the *gap* — the Z distance between the FAU and the part — because both
treat their axes symmetrically, and Z is not symmetric: coupling rises
monotonically as the gap closes until the parts touch. This search is the
in-between: **drive Z in, and only search XY when driving Z stops paying.**

```
raster_scan  →  climb_raster  →  xcan
first light     close the gap     peak
```

---

## 1. Algorithm

One iteration = **step Z, probe, and — unless the probe says otherwise — search
the XY plane and move to its maximum.**

```
baseline = mean(measure() x avg_count)      -- the ONLY averaging in the climb
repeat up to floor(max_z_um / z_step_um) times:
    move_rel(Z, z_dir * z_step_um)
    probe = measure()                       -- where the stage already stands
    if probe >= exit_level        -> stop: target
    if probe still gaining        -> skip the plane, step Z again
    plane = scan_plane(seeded with the probe)
    move to the plane's best
    if plane best >= exit_level   -> stop: target
    if plane best collapsed       -> stop: contact  (retreat to the peak)
park on the best point of the whole run; measure once to confirm
```

**A plane's verdict only exists once its scan has finished.** No exit is taken
part-way through a plane, because near first light the signal at a given Z is
not knowable until the plane has been searched — a single mid-scan point says
nothing about the plane.

### 1.1 The Z probe — the fast path

After a Z step, one sample is taken at the XY the stage is already on. The XY
has not moved since the previous plane's best (or the previous probe), so this
compares like with like: **the same point, one Z step deeper.**

While that step alone gains more than `climb_z_probe_pct` (5 % by default), the
*gap* — not the alignment — is what limits coupling, and the plane search would
only re-find the same spot. So the plane is skipped and Z steps again. The XY
plane is searched once that gain flattens out or the signal falls, which is
where XY misalignment starts to dominate.

Two rules keep the fast path honest:

* **It can only end the run by reaching the target.** A measured point at the
  exit level is the exit criterion whatever the search was doing. It never stops
  on a drop — a fall is exactly the case that hands over to a full plane scan,
  still the only thing entitled to declare a suspected contact.
* **It only compares against a positive reference.** Near first light the signal
  can sit at or below zero, where a relative gain is meaningless (and would flip
  sign); those planes are always rastered.

`climb_z_probe_pct = 0` disables the probe entirely — every plane is searched.

### 1.2 Plane scan: `cross` (default) vs `spiral`

| | `cross` | `spiral` |
|---|---|---|
| Path | line scan along A, move to that axis's best, then along B | square spiral outward from the plane centre |
| Points | ~10–20 (`1 + 8N` worst case per pass) | up to `(2N+1)²` |
| Early stop | `climb_peak_drop` consecutive non-improvements end a direction | `climb_rings_stop` consecutive rings without improvement stop the expansion |
| Assumes | the peak is roughly axis-separable | nothing about the peak's shape |

Cross is the default because fibre-to-chip coupling separates well enough and a
plane costs a handful of points instead of 169. Spiral is the pass to reach for
when a part's peak is tilted or double-lobed, and it starts at the middle (where
the signal is most likely) with unit steps, so it wastes no travel getting there.

### 1.3 Exit criteria

The run stops on the first of:

| Stop | `climb_stop` | Meaning |
|---|---|---|
| Target reached | `target` | a plane best (or a probe) reached `climb_target_pct` of `climb_target` — the intended exit |
| Suspected contact | `contact` | a completed plane's best fell more than `climb_contact_drop_pct` below the run peak |
| Budget spent | `z_budget` | `climb_max_z_um` of travel without reaching the target |
| Stalled | `stall` | `climb_stall_planes` planes produced no new peak (off by default) |

**`climb_target` is mandatory and must be > 0.** Unlike a raster (bounded by its
grid) or XCAN (bounded by its ladder), this search has no intrinsic budget other
than the travel cap — the target is what ends it. The script asserts on a
missing target rather than inventing one.

### 1.4 The contact guard, and why it retreats

A real climb's signal rises or plateaus as the gap closes. A sustained collapse
means the FAU is touching something, and continuing to step Z+ would push into
it. So a plane whose best falls `climb_contact_drop_pct` below the run peak ends
the run and the stage **retreats to the peak** — the last Z at which the part
was demonstrably not being pushed.

Three details are load-bearing:

* The drop is judged **before** the plane is folded into the run best, or the
  plane would be compared against itself.
* It needs a **positive** peak: a relative drop means nothing while the signal is
  at or below zero.
* Planes the probe skipped count as prior measurement — they left a real sample
  behind — so a collapse on the first plane rastered after a run of skips is
  still judged.

**The retreat happens on every stop, including an error.** The climb body runs
under `pcall`; a failure parks at the run best before re-raising. A search that
drives toward a part must not leave the stage wherever it broke.

---

## 2. Trace format (`climb_trace = 1`)

One CSV line per measured sample, into the log band **and** the daily log
(`logs/firedog_YYYY-MM-DD.log`). Extract with
`findstr "clmb," logs\firedog_*.log`.

```
clmb,seq,t_s,phase,plane,z_um,ia,ib,<A>=pos,<B>=pos,<Z>=pos,v=value,dv=delta,note
```

`phase` ∈ `base | probe | plane | final`; `note` ∈ `baseline | probe |
probe-skip | probe-capped | probe-target | plane-best | final@peak` (or empty
for an ordinary plane point). `ia`/`ib` are the sample's cell offsets from the
plane centre, so a plane can be replotted offline exactly as it was walked.

---

## 3. Parameter reference

| ctx key | Command param | Default | Meaning |
|---|---|---|---|
| `climb_device` | `device` | — | hexapod name |
| `climb_topic` | `topic` | `keithley/ch1` | feedback stream |
| `climb_target` | `target_ua` ×1e-6 | — (**required**) | the signal being driven at |
| `climb_target_pct` | `target_pct` | 85 | fraction of it that counts as reached |
| `climb_z_axis` | `z_axis` | `Z` | the climb axis |
| `climb_plane_axes` | `plane_axes` | `X,Y` | the two searched axes (A,B) |
| `climb_z_dir` | — | +1 | sign of the climb direction |
| `climb_z_step_um` | `z_step_um` | 1.0 | travel per climb step |
| `climb_max_z_um` | `max_z_um` | 20 | total travel budget |
| `climb_max_z_steps` | — | 2000 | absolute step cap, whatever the step size |
| `climb_z_probe_pct` | `z_probe_pct` | 5 | gain that means "Z alone is still paying" (0 = off) |
| `climb_max_skips` | `max_skips` | 5 | forced plane search after N skips (0 = unlimited) |
| `climb_plane_scan` | `plane_scan` | `cross` | `cross` or `spiral` |
| `climb_xy_step_um` | `xy_step_um` | 0.5 | plane grid pitch |
| `climb_xy_steps` | `xy_steps` | 6 | grid half-width in cells (13 × 13) |
| `climb_peak_drop` | preset `peakDrop` | 2 | cross: non-improvements that end a direction |
| `climb_cross_passes` | — | 1 | cross: A→B passes per plane |
| `climb_rings_stop` | preset `ringsStop` | 2 | spiral: rings without improvement (0 = full grid) |
| `climb_improve_pct` | preset `improvePct` | 2 | noise margin on "is this better?" |
| `climb_contact_drop_pct` | `contact_drop_pct` | 10 | plane best this far under the peak = contact (0 = off) |
| `climb_stall_planes` | preset `stallPlanes` | 0 | planes without a new peak before stopping (0 = off) |
| `climb_raster_start` | — | 0 | search the starting plane before the first Z step |
| `climb_settle_ms` | preset `settleMs` | 150 (preset) / 300 (script) | fresh-sample **ceiling** |
| `climb_settle_min_ms` | `settle_min_ms` | 0 (production: 50 ms) | precautionary floor |
| `climb_z_settle_ms` | preset `zSettleMs` | 200 | extra wait after a Z step |
| `climb_fresh_samples` | preset `freshSamples` | 2 | new publishes per measure (0 = blind sleep) |
| `climb_avg_count` | preset `avgCount` | 3 | baseline averaging only |
| `climb_tol_mm` | preset `tolMm` | 0.001 | move arrival tolerance |
| `climb_move_timeout_ms` | preset `moveTimeoutMs` | 10000 | per-move timeout |
| `climb_velocity_mm_s` | `velocity_mm_s` | 1.0 | set once up front; 0 = keep |
| `climb_trace` | `trace` | false | per-sample CSV trace |

Outputs: `climb_baseline`, `climb_best` (peak seen), `climb_final` (confirmed
after the park), `climb_target`, `climb_target_frac`, `climb_gain`,
`climb_stop`, `climb_exit_hit`, `climb_z_um`, `climb_best_z_um`,
`climb_best_a_um`, `climb_best_b_um`, `climb_planes`, `climb_skipped`,
`climb_points`.

---

## 4. Timing

Measurement is the **same** contract as `hexapod_align.lua` §4 — one sample per
point, gated on `climb_fresh_samples` new publishes of the feedback topic rather
than a blind sleep, with `climb_settle_ms` as the timeout ceiling and
`climb_settle_min_ms` as the precautionary floor for a slow analogue front end.
That decision, its rationale and its bench numbers live in
`docs/xcan-scan-design.md` §4 and are not repeated here.

The one addition is `climb_z_settle_ms` (200 ms): a Z step on a hexapod is a
different mechanical event from an in-plane micro-step — it loads the flexures
along the stiff axis — so the climb pays one extra settle per plane, not per
point.

---

## 5. Differences from the Horizon original

Deliberate, and each fixes something found reviewing `RasterClimbEngine`:

1. **The probe sample seeds the plane.** Horizon's probe measures the plane
   centre, then `ScanPlaneCross`/`ScanPlaneSpiral` immediately measures the same
   XY again — same point, no motion between them. The two values can disagree,
   and whichever wins decides the plane's initial best. Here the probe *is* the
   centre sample.
2. **`climb_max_skips` caps consecutive skips.** The fast path assumes XY does
   not drift as Z advances. On a hexapod with a pivot offset it does: a Z move
   has real XY cross-coupling, so a long run of skips accumulates XY error with
   nothing but the gain test watching — and the gain test stays satisfied by the
   gap closing. After `max_skips` skips the plane is searched regardless. It can
   only ever *add* measurements, never end a run early.
3. **Each line-scan direction is judged against the line origin.** Horizon seeds
   both directions' `lineBest` from `plane.BestV`, which the first direction may
   have just raised — so the second direction starts already "failing to
   improve" and a noise spike on one side truncates the other side to
   `peak_drop` samples. Both directions here are judged against the value
   measured *at the line origin*.
4. **One sample per point, not an N-sample average.** FireDog's house measure
   (fresh-publish gating) makes per-point averaging redundant; averaging is kept
   for the baseline only, as in the other two scan scripts.
5. **A confirming measurement after the final park**, reported separately as
   `climb_final`. `climb_best` is the peak the search saw; a target exit latches
   on one sample, and the two can differ.
6. **`climb_stall_planes`**, off by default. Horizon has no "converged but short
   of target" stop, so a part that physically cannot reach the exit level runs
   the full Z budget every time — finding its peak early, then stepping past it.
   The contact guard catches a *collapse*, not a slow decline.

Not ported: Horizon's SLED guard and Auto-W chaining (`RasterClimbPanel.xaml.cs`)
are SiPhOG-specific and belong to that machine's process layer, not to the search.

---

## 6. Known issues / operating guidance

1. **Contact is declared only after a full plane scan completes.** The probe
   sees the drop and hands over to the plane, which then performs up to
   `PointsPerPlane` XY moves *while in contact* before the collapse is judged.
   Stepping Z into a part is one failure mode; rastering XY against it is shear.
   Until that is addressed, keep `climb_z_step_um` small enough that a single
   step cannot go from clear to hard contact, and keep the contact guard on.
2. **The target must match the product power class.** The Run page's Climb tab
   drives at the *active XCAN Target slot* deliberately — one product, one
   target — so switching the Target button changes what the climb aims at.
3. **Concurrency.** `orch_.Busy()` keeps the Lua engine, the recipe engine, the
   program runner and the UI path executor mutually exclusive — a climb blocks a
   recipe start and vice versa.

Related: `docs/xcan-scan-design.md` (the measure/settle contract this shares),
`docs/pi-hexapod-motion-design.md` (arrival/settle), `docs/run-page-design.md`
(the Scan band), `docs/lua-tier-design.md` (bindings, stop/debug hooks).
