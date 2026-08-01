# Vision layer — focus metric, auto-focus, HALCON gating

*Reference for `src/vision/`, the `auto_focus` / `halcon_status` commands, and how
FireDog behaves when HALCON is absent or unlicensed. Summary lives in CLAUDE.md.*

The layer has two halves that are deliberately independent:

| Half | Files | Needs a vendor SDK? |
|---|---|---|
| Focus measure | `vision/FocusMetric.{h,cpp}` | **No** — plain C++ |
| HALCON availability | `vision/HalconEngine.{h,cpp}` | Optional (`HAS_HALCON`) |

**Auto-focus must never depend on HALCON.** Focus is the one vision operation
the machine needs to run a product, and a licence file is machine-local state
that can expire. Keeping the metric to ~100 lines of plain C++ means a licence
problem degrades inspection, never alignment.

`src/vision/` compiles into **`firedog_hw`** (not the exe), so the rig gets it
too — see `docs/build-and-sdks.md`.

---

## 1. The focus measure

**Variance of the Laplacian** over a centred ROI, on the RGB8 buffer the camera
already publishes. A sharp image has strong second derivatives; a blurred one
has weak ones. Ported from bobcat_engine's `VisionAutoFocusCommands`, whose
`cv::Laplacian` + `meanStdDev` pair computes the same number — FireDog just does
not carry OpenCV to get it.

```
score = Var( 4*c - up - down - left - right )   over the ROI, on BT.601 luma
```

Two properties worth knowing before trusting a number:

* **Relative, not absolute.** Comparable only between frames of the same scene at
  the same exposure and gain. That is exactly the Z-sweep case, and exactly not
  the "compare two products" case.
* **ROI-bound cost.** O(roi²). 256×256 is well under a millisecond, which is what
  makes it legal to score every frame on the UI thread.

`MeanLumaRgb` rides along as the guard rail: a saturated or black ROI makes the
score meaningless, and exposure is the fix. The UI turns the readout amber
outside luma 5–250.

### Where it is sampled — `Workflow::MeasureFocus(roiPx)`

Layer 2 owns the sampling: it copies the newest frame, scores it, publishes
`topics::kCameraFocus` (`camera/focus`) and caches the result. Thread-safe by
construction — the auto-focus worker and the UI readout both call it.

`SetFocusStreaming(on, roiPx)` makes `Workflow::Update` score every new frame, so
the number can be watched live while jogging Z by hand (Jog window → Camera tab
→ **Live score**). Off by default; it costs a score per frame.

### Frame freshness — why `PylonCamera::FrameSeq()` exists

`GetLatestFrame`'s `isNew` flag is **one-shot**: the first caller of a frame
consumes it. With two consumers (UI texture upload, focus sampler) that races,
and the loser concludes "no new frame". So layer 1 also exposes a **monotonic
frame counter** (`FrameSeq()` / `GetLatestFrameSeq`), which nobody consumes and
any number of readers can compare against their own last-seen value.

This is the camera analogue of the DataBus publish counter that the XCAN scan
uses (`docs/xcan-scan-design.md` §4.3), and it is what makes a focus sample
*provably* post-move rather than hopefully post-move.

---

## 2. `auto_focus` (command, category **Vision**)

`src/command/CommandsVision.cpp`. Runs on the engine worker thread; every wait
goes through `ExecutionContext`, so Stop interrupts the sweep within one poll
interval.

```
for z in the swept interval (see `direction`) step z_step_mm:
    move the axis to z          -- Workflow::MoveToPosition, one axis
    WaitArrive + WaitSettle     -- the SHARED waits (command/MotionWaits.h)
    wait for `fresh_frames` new frames, then score
peak  = argmax(score)
bestZ = parabolic refinement of the peak between its neighbours
move to bestZ, re-measure, write ctx
```

Three differences from the bobcat port, each earning its keep:

1. **Freshness is proven, not slept for.** `fresh_frames = 2`: frame #1 may have
   been integrating while the stage travelled (a mixed exposure); frame #2 is
   wholly from the new position. The wait self-scales with frame rate — ~66 ms at
   30 fps, ~400 ms at 5 fps — where a fixed sleep is either wasteful or wrong.
   `frame_timeout_ms` is the ceiling: on a stalled feed it degrades to "score the
   latest frame" and warns **once** per sweep. `fresh_frames = 0` restores the
   blind behaviour.
2. **The peak is refined below the step.** Parabolic vertex through the three
   samples around the maximum, so the answer is not quantised to `z_step_mm`.
   Guarded: it needs a concave triple, an interior peak, and genuinely
   equally-spaced neighbours (a dropped sample leaves a 2-step gap, which would
   otherwise place the interpolated Z wrongly).
3. **Arrival and settle are the shared move waits.** `WaitArrive` +
   `WaitSettle` from `command/MotionWaits.h` — the same two-signal rule every
   motion command uses (profile done AND on-target). A second, slightly
   different copy is how a scan starts measuring a ringing stage.

### Parameters

| Param | Default | Meaning |
|---|---|---|
| `device` | — | motion device carrying the optic |
| `axis` | `Z` | local axis to sweep (validated against the device's axes) |
| `z_range_mm` | 0.2 | sweep distance (see `direction`) |
| `direction` | `both` | `both` = ±range around the start; `minus` = `start-range .. start`; `plus` = `start .. start+range` |
| `z_step_mm` | 0.02 | sweep step |
| `roi_px` | 256 | centred square ROI; 0 = whole frame |
| `fresh_frames` | 2 | new frames required after each move (0 = blind) |
| `settle_ms` | 0 | precautionary delay before sampling |
| `frame_timeout_ms` | 2000 | ceiling on the fresh-frame wait |
| `timeout_ms` / `tol_mm` | 30000 / 0.01 | per-move arrival timeout / tolerance |
| `min_score` | 0 | fail (and return to start) below this |
| `interpolate` | true | parabolic peak refinement |
| `trace` | false | one CSV line per sample: `autofocus,i,z,score,luma` |
| `velocity_mm_s` | 0 | optional sweep velocity; 0 = keep |
| `ctx_prefix` | `autofocus` | ctx key prefix |

`z_range_mm` / `z_step_mm` combinations needing more than **400** moves are
rejected up front, naming both numbers — that combination is a typo, not a plan.

Whatever the interval, the walk itself is **always in increasing Z**, so every
sample is approached from the same side and backlash cannot bias one end of the
curve against the other.

### The Autofocus buttons (Jog window → Camera tab, and the Aux window)

Two launchers, one sweep: both call `App::StartQuickAutoFocus` over the same App
state, so they cannot disagree about what a sweep does. Defaults are
**gantry-main Z downward** (`direction = minus`), 0.5 mm range in 0.025 mm steps,
scoring with the ROI already selected for the live readout. `trace` is on, so the
whole curve lands in the log as `autofocus,i,z,score,luma` lines.

* **Camera tab** owns the numbers — range, step, direction — because that is
  where you are already looking at the score and the exposure behind it.
* **Aux window**, beside the Z arrows, carries the button and the *direction*
  only. That is where an operator actually is (watching the picture) when they
  decide focus is off, and direction is the knob an edge-peak result tells you to
  change.

Both run the command as a **one-step ad-hoc recipe** on the engine worker — the
same mechanism as the Recipe Editor's *Run Step*. Two reasons, both rules rather
than preferences: the sweep blocks for tens of seconds and must never do that on
the UI thread, and validation (device known, camera Connected, params in range)
stays the engine's single copy. They gate on `jogArmed_` (real motion) and on
`orch_.Busy()` (one orchestration owner), and each reports progress **only for
its own run** — matched by recipe name against `kQuickAutoFocusName`, since the
engine is shared with the Recipe Editor, the Run page and the programs — with
Stop inline.

### Results (ctx, under `ctx_prefix`)

`.success` `.best_z_mm` `.best_score` `.final_score` `.mean_luma` `.delta_mm`
`.samples` `.interpolated` `.edge` `.start_z_mm`

`.edge` is the one to branch on: a peak on the first or last sample means the
true focus lies **outside** the swept range, so the result is a range problem,
not a focus position. It is logged as a warning and still returned.

### Failure behaviour

| Situation | Result |
|---|---|
| Move rejected / timed out mid-sweep | device stopped, return to start, step fails |
| No usable samples (camera not grabbing) | return to start, step fails |
| Best score below `min_score` | return to start, step fails |
| Stop requested | fails with `stopped`, wherever it is |

---

## 3. HALCON — build, runtime, licence

`vision/HalconEngine.{h,cpp}` is the **only** TU that includes a HALCON header.
Everything else asks `HalconEngine::Instance().Available()` and gets a bool plus
a reason string.

Three independent gates, because they fail independently and need different
fixes:

| Gate | Meaning | Typical fix |
|---|---|---|
| `compiledIn` | `HAS_HALCON` was defined — CMake found the SDK | install the SDK, re-configure |
| `runtimeLoaded` | `halcon.dll` + `halconcpp.dll` actually load | install the HALCON runtime |
| `licensed` | a licensed operator really ran | drop a valid `*.dat` in the licence dir |

### Detection (CMake)

No vendored copy — HALCON is licensed per machine, so the SDK stays where it was
installed. CMake reads the environment the installer sets:

```
HALCONROOT  -> <root>/include/halconcpp/HalconCpp.h, <root>/lib/$HALCONARCH/halconcpp.lib
HALCONARCH  -> x64-win64 (default when unset)
```

Gating is `HAS_HALCON` PUBLIC on `firedog_hw`, plus `FIREDOG_HALCON_ROOT` baked
in as a build-time fallback path for the DLL probe (a service account may not
have `HALCONROOT` in its environment).

### Why /DELAYLOAD, and why LoadLibrary before any call

The libs are linked with `/DELAYLOAD:halconcpp.dll /DELAYLOAD:halcon.dll`
(INTERFACE on `firedog_hw`, so both exes inherit it), so an exe built **with**
the SDK still starts on a machine that has **no** HALCON.

The catch: the first call into a missing delay-loaded import raises a Win32 SEH
exception (`0xC06D007E`) that a C++ `catch` does not handle. So `Refresh()`
probes the modules with `LoadLibrary` **first** and only calls HALCON if they
loaded. Version comes from `GetSystem("version")` (no licence needed, so it is
readable even on an expired box); the licence gate is a real
`gen_image_const` + `mean_image`, because a `.dat` on disk can still be expired
or issued for another host id.

### Surfaces

* **Camera page → right panel** — the three gates, version, licence files found,
  the error text, and a **Re-check** button (dropping in a `.dat` should not cost
  a restart).
* **Startup log** — probed once in `App::Run`, so the state is in the log band
  from the first frame.
* **`FireDogRig` → `probe`** — a `vision HALCON` block. Read-only, like the rest
  of the rig; "is the licence on this PC valid?" is a bring-up question.
* **`halcon_status` command** — writes `halcon.available` / `.licensed` /
  `.version` / `.summary` into ctx so a recipe can **branch** on the licence
  instead of failing halfway through a vision block. `require = true` turns it
  into a hard gate.

Related: `docs/build-and-sdks.md` (SDK gating rules), `docs/command-layer.md`
(the registry), `docs/xcan-scan-design.md` §4.3 (the freshness idea this borrows).
