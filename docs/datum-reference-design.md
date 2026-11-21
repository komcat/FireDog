# Datum reference: the alignment frame as one recipe step

*A part's local frame, taught by ONE step that owns its two or three detection
points, kept for the part's life, consumed by `move_local`, drawn on the Aux
view. Code: `core/Datum.h`, `core/AlignFrame` (`FitAlignFrame3`),
`command/CommandsDatum.cpp`, `Workflow::SetDatum` …, the Vision detect helper
(`ui/windows/WndVisionDetect.cpp`), `App::DrawAuxDatums` (`ui/windows/WndAux.cpp`).*

---

## 1. What it is, and what it is not

The Alignment Procedure (`docs/alignment-procedure-design.md`) already teaches a
local frame from two features: origin = E1, X+ = E1 → E2, Y+ = X+ turned +90°.
It is a PAGE: a procedure document with stations, tiers, history, a
repeatability sweep and nominals, run from a recipe by `align_run`.

A **datum** is the same frame packaged as one recipe step,
`datum_reference`. The step owns everything it needs - its points' private
detection models and taught views - and nothing lives on another page. It is
for the recipe author who wants "find the part, then work in its coordinates"
without a procedure document.

| | Alignment procedure | Datum |
|---|---|---|
| Where it lives | a procedure in the product file | the step's own params |
| Points | E1, E2 | E1, E2, optional E3 |
| Detectors | private to a station (`vdet_<proc>_<station>`) | private to the step (`vdet_step_<uid>`) |
| Frame maths | `SolveAlignFrame` | `SolveAlignFrame` (2 pt), `FitAlignFrame3` (3 pt) |
| Record | `AlignProc::result` + run history | none - memory only |
| Lifetime | live frame: program start, DUT open, product load | 30 min, DUT close, product load (§4) |
| Consumer | `move_local`, `align_apply`, nominals | `move_local {datum}` |

The frame record is the same `AlignResult`, transformed by the same
`AlignLocalToMachine`, so `move_local` drives a datum exactly as it drives an
alignment. What differs is only where the frame comes from and how long it lives.

## 2. Two points or three

**2-point** is the alignment frame exactly: `SolveAlignFrame(E1, E2)`.

**3-point** is a least-squares RIGID fit (origin + rotation, no scale, no shear)
of a taught reference shape onto the three measured points. The reference is
where each point belongs in the local frame:

```
E1 = (0, 0)      E2 = (ref_span_mm, 0)      E3 = (ref_e3_lx, ref_e3_ly)
```

`FitAlignFrame3` is the 2-D Procrustes solution: with both point sets centred on
their centroids, `theta = atan2(sum(dx*my - dy*mx), sum(dx*mx + dy*my))` and the
origin is the measured centroid minus the turned reference centroid. The result
is an `AlignResult` like any other - `originX/Y` is where local (0, 0) lands,
which is the FITTED E1, not the raw E1 mark.

### Why it is more precise - and when it is not

Rotation noise is pick noise divided by the points' spread about their centroid:
`sigma_theta ~ sigma_pick / sqrt(sum r_i^2)`.

| Layout | sum r^2 | theta noise vs 2-point |
|---|---|---|
| 2 points, span L | 0.5 L^2 | 1 |
| E3 at the square corner (0, L) | 1.33 L^2 | ~0.61 |
| E3 on the E1-E2 line, mid-way | 0.5 L^2 | **1 - no gain** (only the origin averages) |

So E3 earns its keep only when it adds LEVER: placed away from the E1-E2 line,
ideally the far corner. A longer E1-E2 span buys the same thing with no third
point. The helper says so when E3's reference sits near the line.

Scale was rejected for the same reason the alignment frame rejects it: a fitted
scale absorbs a mis-pick into a plausible number. A shear/affine fit would do the
same with three points. Rigid, always.

### The residuals are the check

Three points give 6 measured numbers for 3 unknowns, so each point has a
RESIDUAL - how far the fitted frame puts its reference from where it was
measured. `resid_tol_mm` (0 = off) fails the step on the worst one: a wrong
feature at ANY of the three shows up there. Two points have one check only, the
span against `ref_span_mm` (`span_tol_mm`).

### The reference is taught, not typed

Every run records its raw measurement - the E1-E2 span, and E3's coordinates in
the E1-E2 frame - in a slot beside the datum store (`Workflow::DatumMeasureFor`).
The helper's **Use as reference** copies the last one into the step
(`ref_span_mm`, `ref_e3_lx`, `ref_e3_ly`). So: run the step on a good part, look
at it, press the button.

A 3-point step with no reference **fails** (after measuring, so the measurement
is there to adopt). It never quietly falls back to a 2-point frame: a step that
says 3-point and fits two is the "claims automatic, isn't" failure. A 2-point
step needs no reference; with one, the span guard can use it.

## 3. The step

`datum_reference` - one step, three phases:

1. **Each point** runs the actual `detect_object` through `RunRegistered`, with
   the step's `model<i>` / `view_point<i>` and its shared run options (camera
   device, Z lift, safe Z, settle, manual fallback, light, frames, velocity).
   There is no second copy of the drive-detect-fallback logic. Each point is
   therefore ALSO an object result, `<name>_E1` …, which `move_to_result` can
   drive to - with an object result's lifetime, not the datum's.
2. **Solve**: 2 pt `SolveAlignFrame`, 3 pt `FitAlignFrame3`; then the guards
   (`span_tol_mm` against `ref_span_mm`, `theta_tol_deg` against
   `theta_ref_deg`, `resid_tol_mm`). An out-of-band datum FAILS and publishes
   nothing - unlike an alignment station, nobody looked at the mark first.
3. **Publish**: the datum into `Workflow`'s store under `name`, and ctx mirrors
   (`<name>_ok`, `_origin_x`, `_origin_y`, `_theta_deg`, `_span_mm`,
   `_max_resid_mm`) that no move reads.

The name is ERASED before the first point is measured, so a miss, a refusal, a
failed guard or a Stop leaves no previous part's datum for a later move.

`datum_clear {name | all}` removes one or every datum; a missing name is a no-op.

Not a teach mover (no position of its own to write), not a parallel mover
(shared `Workflow` state), not in the dry-run allowlist (it detects).

## 4. Lifetime

A datum describes ONE part, but a program may measure it in one recipe and use
it in the next, so it lives in `Workflow` (ctx is wiped between queue items),
memory only:

- **30 min** (`kDatumLifeMin`) from the solve. An expired datum reads as absent
  everywhere; `App::Update` → `ExpireDatums` erases and logs it once.
- **`datum_clear`**.
- **DUT close** - edge-detected in `App::Update` off `DutSession::GetInfo()`
  beside the DUT-open edge: the DUT that was open is no longer (closed, or
  replaced by another). Every datum goes. A closed DUT is a finished part.
- **Program start, for a datum made with NO DUT open.** On a station that never
  opens a DUT, the 30 minutes alone would let the next part, loaded inside that
  window, move against the previous part's datum with nothing on screen to say
  so. `Datum::withDut` records whether a DUT was open at the solve;
  `ClearDatumsWithoutDut("program start")` drops the rest.
- **Product load, app close.**

Program start KEEPS a datum made with a DUT open - the beacon's rule - so a
`move_local` in a later program needs no `use_kept_frame`: the step names the
datum, and the DUT boundary is what says the part changed.

## 5. `move_local {datum}`

`move_local` takes `alignment` OR `datum` - exactly one, refused before moving
otherwise (the `dispense_run` point/position shape). With a datum, an empty
`device` means the datum's device. Everything after the frame lookup is the
existing body (`MoveToLocalPoint`): the same transform, Z rule, velocity and
`MoveWithApproach`. `use_kept_frame` means nothing for a datum and is ignored.

## 6. The helper

The datum step names `kVisionDetectHelper`: the SAME Vision detect helper, with
a point strip (E1 / E2 / E3) at the top. The selected point's model and view are
what the helper's camera-position section, trial detections and teach panels
work on (`App::datumPointSel_`), through the same functions `detect_object`
uses, pointed at `model<i>` / `view_point<i>`. Below the strip: the mode, the
reference (+ Use as reference), and the guards.

The Recipe Editor's step form shows each point's template and view thumbnail
(read-only, one row per point, `App::stepDatumTmplTex_` / `stepDatumViewTex_`),
the same pair a `detect_object` form shows.

**Ownership.** A step's private models are found through ONE table,
`StepDetectorParams(type)` (`command/StepDetectors.h`): `detect_object` →
`model`; `datum_reference` → `model1`, `model2`, `model3`. `SyncStepDetectors`,
the helper's owner count and the POR compare / fix list / deploy all read it,
so a copied datum step gets its own three models on save and a deleted one
leaves no orphans.

## 7. The Aux overlay

`App::DrawAuxDatums`, always on (it is what a run will move against), only for a
datum on the device posing the picture:

- E1 / E2 / E3 as labelled crosses (`<name> E1` …) at their MEASURED points,
  through `DrawWorldMark`, so off-screen points become edge arrows with the
  distance - the usual case, since a span is normally larger than the field of
  view;
- the **X axis** - a line through the frame origin along theta, the positive
  ray solid and labelled `X+` where it leaves the picture, the negative ray
  faint - and the **Y axis** the same along theta + 90°. The lines are what stay
  useful when no point is on screen: the X line crossing the view shows the
  part's angle wherever the camera is.

**While the step measures**, the points found so far are drawn too, labelled
`(measuring k/n)` and with no axes - there is no frame until the last point.
They come from a progress slot (`Workflow::SetDatumProgress`, `core/Datum.h`
`DatumProgress`) the command fills point by point and erases on EVERY exit, so
a half-measured datum never stays on the picture. The alignment's
`QueueAlignPoint` reason: the moment an operator wants to see E1 landed is
while E2 is still being looked for.

Projection is `WorldToScreen` (`ui/Widgets`), the one machine-mm → screen
function `DrawWorldMark` and `DrawWorldBox` also use.

## 8. Future: the template outline (not built)

A template is a set of shapes (outline, pads, keep-outs) in LOCAL coordinates.
Drawn through the live datum - local → `AlignLocalToMachine` → `WorldToScreen` -
it shows where things are supposed to be on this part. It needs nothing the
datum does not already have: the frame is held in machine mm and the projection
exists. Display only, like every outline on the Aux view: nothing would judge or
move by it.

## 9. Known limits

- One camera device per datum: all points are measured on `camera_device`.
- The frame is 2-D. Z is the mean focus height of E1 and E2, as for an
  alignment, and is not transformed.
- No run history. A recipe that wants the numbers on the part's record writes
  the ctx mirrors with `dut_record_all {prefix = <name>_}`.
