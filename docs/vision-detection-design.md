# Vision detection: taught models, the teach wizard, auto stations

*Why detection is a named document rather than a station field, why it is
HALCON-only when auto-focus deliberately is not, and where each half lives.
Reference material for the shipped code: `src/vision/VisionModel.{h,cpp}`,
`src/vision/VisionResult.h`, `src/vision/HalconDetector.{h,cpp}`,
`src/command/CommandsVision.cpp`, `src/ui/pages/PageVisionTeach.cpp`.*

---

## 1. What changed

Before this, `src/vision/` analysed no images at all: `FocusMetric` scored
sharpness and `HalconEngine` only answered "is HALCON installed and licensed?".
The Alignment Procedure's **auto** mode was structure with nothing behind it —
`align_station` failed with *"feature detection is not implemented"*.

Now an alignment station can locate E1/E2 without an operator. Two methods,
both ported from `bobcat_engine`'s HALCON plugins minus OpenCV:

| Method | Chain | Returns |
|---|---|---|
| `circle` | threshold → closing → connection → area & circularity filter → largest → `fit_circle_contour_xld` (Tukey) | XY, radius, circularity as the score |
| `ncc` | `create_ncc_model` / `find_ncc_model` over a taught patch, plus an operator-placed anchor | XY, angle, score |

Shape-based matching and edge/line measurement are **not** in this build.
`VisionDetection` reserves `lengthPx` for the latter so adding it does not change
the result shape.

## 2. A separate document, owned by one station

A model is its own uid-stamped collection — manifest role `vision`, file
`{uid}.vision.json`, with the `config/vision.local.json` → `vision.json`
fallback — following `core/AlignFrame` line for line. Two reasons it is not just
a few more fields inside `AlignStation`:

1. A taught NCC model is a **file**, not a number. Something has to own that
   file's lifetime, and a struct embedded in another document is a bad owner.
2. `vision_detect` should be usable from a recipe step and from Lua, neither of
   which has an alignment station to read parameters out of.

But a separate document is **not** a shared one. Each model belongs to exactly
one station:

* it is auto-named **`vdet_<procUid>_<station>`**, and that name IS the
  ownership record — the same device the private station positions use, so no
  schema change was needed and an orphan is always traceable;
* the station's existing `detector` field caches the uid, so `align_station` and
  a recipe step resolve it the way they resolve everything else;
* the teach window therefore has **no model list, no New/Duplicate/Delete and no
  name field**. Naming a thing that can only have one owner is a decision nobody
  should have to make.

The first draft did offer a shared library with a picker, on the theory that one
fiducial might be marked by several stations. That is a coincidence, not a
reuse: the ROI, the grey window and the taught patch only mean anything at the
pose, exposure and magnification they were tuned at, and two stations look at
different places. A picker would mostly be a way to attach the wrong detector.

Maintenance follows the ownership, in the same two places the station positions
use: **`DropAlignStationDetectors`** on procedure delete (nothing lists these
models, so an orphan is something nobody would ever go looking for) and
**`CopyAlignStationDetectors`** on duplicate, patch file included, so a
duplicate is self-contained rather than sharing rows the original's Delete would
take away.

### The `.pgm`, and why not PNG

bobcat saves the taught patch as a PNG through `cv::imwrite` and rebuilds the
HALCON model from it on load — never `write_ncc_model`. That rebuild-on-load
idea is right (a model file is version-locked to the SDK that wrote it; a PNG is
not), and it is kept. The format is not: FireDog carries no image codec, so the
patch is an **8-bit binary PGM**, twenty lines of plain C++ in `VisionModel.cpp`.

Two things fall out of that choice, both wanted: any viewer opens a `.pgm`, and
the teach window can show the taught patch **without HALCON**, so a model is
still inspectable on an unlicensed box.

The filename stored in the JSON is **relative** to `ProductSession::VisionDir()`.
bobcat stores an absolute path, which quietly breaks the moment a product folder
is copied to another PC — the one thing a product folder exists to allow.

Ownership is explicit because the file is invisible: `DeleteVisionModel` removes
the `.pgm` with the row, Duplicate **copies** it under the new uid rather than
sharing it, and `NewProduct` copies every patch into the new product's folder.
Same reasoning that makes deleting an alignment procedure drop its private
station positions.

## 3. Why HALCON-only, when auto-focus is deliberately not

`docs/vision-layer.md` states the rule that auto-focus must never depend on
HALCON, and the manual crosshair pick is under it too. Detection is
**deliberately outside** it.

The distinction is what a licence failure costs. Focus and the manual pick are
how a human keeps working when the machine's licensing is broken — they must
never be blocked. Automatic detection is a *convenience over* the manual pick:
when it is unavailable, the manual path is still right there, and the operator
loses time rather than capability. Writing a second, plain-C++ NCC matcher and
blob finder to close that gap would mean two detectors to keep honest and two
sets of tuning parameters that behave subtly differently — a permanent cost to
avoid a temporary one.

So: `HalconDetector::Available()` forwards `HalconEngine`'s three gates and its
reason string, and **that one string** is what the station card shows, what the
teach window shows, and what the failed step reports. One wording for one
problem.

`HalconDetector.cpp` is the second (and last) TU allowed to include a HALCON
header. The `/DELAYLOAD` rule binds it unchanged: it may not touch a HALCON
symbol until `HalconEngine`'s `LoadLibrary` probe has passed, because the first
call into a missing delay-loaded import raises SEH `0xC06D007E`, which no C++
`catch` handles.

## 4. The result format

One struct, `VisionDetection` (`vision/VisionResult.h`), filled by the detector,
cached by `Workflow`, published to the DataBus, and flattened into ctx by
`vision_detect`. Three conventions are fixed there rather than per caller:

* **`xPx`/`yPx` are always FULL-FRAME.** A detector that ran on a reduced ROI
  adds the clamped origin back before returning — `Workflow::Detect` does it in
  exactly one place. bobcat patches its point list but *not* its measurement
  map, so half its numbers are ROI-local; that split is not worth inheriting.
* **`score` is 0..1 and comparable ACROSS methods.** The circle detector reports
  the selected region's circularity, which is already in that range and already
  higher-is-better, so a reliability run means the same thing whichever method
  is under test.
* **A miss is `ok = false` plus a reason, not a failure.** "The feature is not
  there" is a legitimate answer that a reliability run has to be able to count.
  A licence problem arrives through the same field, so neither looks like a
  crash.

### Pub-sub

`Workflow::Detect` publishes `vision/x_px`, `vision/y_px`, `vision/score`,
`vision/angle_deg` and `vision/ok` (`core/Topics.h`). Scalar streams because
that is what a `DataStream` carries and what a chart wants; `vision/ok` is on
the bus as 1/0 **including the misses**, since suppressing them would flatter
every model. `Workflow::LatestDetection()` holds the full record for anyone who
needs the angle, the radius and the error text together.

Publishing happens **outside** `detectMu_`, so a subscriber can never deadlock
against a detect.

### Detector caching

Building an NCC model is the expensive part, so `Workflow` keeps one
`HalconDetector` per model uid. The cache is keyed on the model's **canonical
JSON dump**, not on a dirty flag: HALCON bakes the angle range and the pyramid
depth *into* the model, so a stale detector would silently ignore half the
parameter form, and comparing the whole document means no edit path can forget
to invalidate.

## 5. The teach wizard

`src/ui/pages/PageVisionTeach.cpp`, a **full-page takeover** (`Page::VisionTeach`)
opened from an auto station's **Teach detection…** button.

Not a panel on the Alignment page, because the job needs a big live image — an
ROI is dragged on it, a template is cut out of it, and the result is judged by
looking at the overlay — and that page's middle column is a narrow strip of
station cards. Not a floating window either, though it started as one: teaching
a detector is a task you do *for* a station and then leave, the operator is not
meant to be jogging underneath it, and every other authoring surface here
(recipes, transforms, IO watch, scripts) is a takeover — one exit convention
beats two.

Deliberately **not in the View menu**: it always belongs to one station, so the
only way in is that station's button. That has one consequence worth knowing:
`SwitchPage` only records `prevPage_` on a non-takeover → takeover switch, so
Alignment → VisionTeach leaves it pointing at whatever hardware page preceded
the Alignment page. Close therefore names `Page::Align` explicitly rather than
using `prevPage_`. Beside the frame's Close sits **Save & close**, because
save-then-leave is the only sequence that makes sense when tuning is finished;
plain Close is what abandons a bad experiment.

It shares the crosshair pick window's view mapping, hoisted into
`ui/Widgets.h` as **`CameraView`** when the second consumer appeared: cover fit
× zoom, with the visible window following a focus point so a zoomed view can
never hide what is being marked. Two copies of that arithmetic would drift the
moment one of them gained a feature, and a pixel has to mean the same thing in
both surfaces.

Its header bar answers "which station am I tuning?" — the procedure name and
station — not "which model", because the model is decided by the button that
opened the page. `OpenVisionTeach(proc, station)` addresses it by its
canonical private **name**, creates it on first use, and writes the uid back
into the station; that is the one lookup that cannot end up pointing at another
station's model.

Bands, in the order the work happens: camera (exposure + the focus/luma guard
rail) → search ROI → method + parameters → **Detect once / Live** → reliability
→ Save.

The two panes are split by a **draggable `Splitter`**, with the width persisted
as `UiSettings::visionParamW` (its own field, not the main window's `rightW`:
a takeover page replaces that five-region layout entirely, and this column has
to hold a whole parameter form). The column's natural width is genuinely not a constant —
a circle model's form is short, an NCC model's carries a thumbnail and longer
labels, and the operator's font size moves both. Two consequences in the code:
labelled widgets take a **negative** item width so ImGui still has room to draw
the label to their right (`-FLT_MIN` would push every label off the panel), and
the explanatory paragraphs go through `TextWrapped`, since hand-broken lines
would spill out of the very column the operator is allowed to narrow.

A few decisions worth keeping:

* **The anchor is placed on the THUMBNAIL, not the live image.** The offset is
  defined relative to the template centre, and the thumbnail is the only view
  where that is literally what is on screen. `useAnchor` derives from the
  numbers (`|offset| > 0.1 px`) rather than being a third thing to keep in step
  with them — bobcat's rule, and it removes a state.
* **A re-teach clears the anchor.** It was an offset from a *different* patch
  centre; carrying it over would point at nothing in particular.
* **Rubber-band drags are armed by a button**, not drag-anywhere. An accidental
  click must not reshape the search window while somebody is judging a result.
* **The detection point is drawn separately from the fitted shape.** With an
  anchor these are deliberately not the same place, and seeing the gap is the
  only way to confirm the anchor does what was intended.
* **A teach saves the document immediately.** The `.pgm` is already on disk;
  leaving a patch file with no model row pointing at it is how orphans start.
  Parameter edits stay explicit-Save — a half-tuned detector must not reach the
  next run.

### Threading

`Detect once` and `Live` call `Workflow::Detect` **on the UI thread**. That is
not a breach of "no blocking call on the UI thread": it is a bounded one-shot
compute with no wait in it, the same class as `MeasureFocus` and the texture
upload. A detect over `kSlowDetectMs` (150 ms) warns once, on the page and in
the log, and names the two fixes the operator has in front of them (tighter ROI,
fewer pyramid levels).

The **reliability run** goes to the engine as a one-step ad-hoc `vision_test`
recipe, exactly like the Autofocus buttons, because it sleeps between runs and
must be stoppable.

**Its statistics come back through `Workflow`'s queue** (`QueueVisionReliability`
/ `TakeVisionReliability`), drained in `App::Update` — the taught-position and
taught-frame route, not the engine's ctx.

That is not a preference, it is the only thing that works, and the first version
got it wrong: `RecipeEngine::ThreadMain` sets `runCtx_ = nullptr` in its terminal
block **before** the state becomes `Completed`, and `GetStatus` only fills
`Status.ctx` `if (runCtx_)`. So a UI that waits for `Completed` and then reads
`Status.ctx` always reads an **empty map** — the run visibly succeeded, the log
showed the numbers, and the page still said "never tested". `Status.ctx` is a
*live* display snapshot; anything a command must hand back **after** the run has
to be queued.

### Why reliability is accepted, not auto-recorded

A single successful detect proves nothing; the number that matters is how far
the reported point wanders when nothing has moved. So `vision_test` runs N times
and reports hit rate, mean score, per-axis sigma and the worst deviation.

A finished run is **parked, not applied**. The page shows it with a green
**Accept & save** (and a Discard), and only that button writes `m.rel` and
persists. Whether 20/20 at 0.05 px is good enough is a judgement about this part
and this feature, and the person who just watched the run is the one qualified
to make it; until they decide, the model keeps whatever figure it already had.
The page shows the pending result and the `recorded:` one separately, so "what I
just measured" and "what is stored" never blur.

It is deliberately **not** a save gate on the model itself. Refusing to save a
model that scored badly would just mean tuning it somewhere the number is not
being recorded. The station card surfaces the recorded figure
(`18/20 ok, sigma 0.4 px`) so a bad one is visible where it matters.

## 6. The Aux window overlay

The Aux window (the operator's watching surface, often on a second monitor)
draws two kinds of mark over the live image, behind a **Vision marks** checkbox
persisted in `UiSettings`. It defaults **on**, unlike click-to-move, because it
is a readout and changes nothing.

**The detection** — `Workflow::LatestDetection()`, in camera pixels. Its box is
exactly **20×20 camera pixels**, a true footprint, so its on-screen size says how
big the feature is relative to the frame; at the Aux window's cover fit that is
only a few screen pixels on a large sensor, which is why the crosshair arms
extend a fixed distance *beyond* it. The box is the measurement, the arms are
what makes it findable. A **miss** is drawn too, with its reason: "it looked and
found nothing" is different information from "it never ran", and only one of them
means the parameters need work.

Freshness is handled in `App::Update`, not in the draw: a new detection is one
carrying a different `frameSeq`, and the age accumulates from `dt`. It fades out
over the last half second of a 2.5 s hold. A marker that lingers over a scene the
stage has since left is worse than no marker.

**E1 and E2** are the interesting half. They are stored in **machine
millimetres**, not pixels, because the two points are captured at two different
gantry poses — a stored pixel becomes a lie the moment the stage moves. They are
projected back through the live gantry pose, the exact inverse of the capture:

```
capture:  wx = gx + (px - w/2) * pixel_to_mm_x
overlay:  screen_x = centre + ((wx - gx) / pixel_to_mm_x) * scale
```

drawn only when the result lands inside the frame — out of view, the overlay says
nothing rather than guessing at an edge. `Workflow::GantryXY` supplies the pose,
because layer 2 is what knows *which* device the gantry is; a UI overlay must not
go hunting for a device called "gantry-main".

Two sources feed those marks, and which wins is the design:

* **A run owns them while it executes.** `align_station` queues each point the
  moment it captures it (`Workflow::QueueAlignPoint`), because `align_solve` only
  fires after *both* stations — without the queue there would be nothing to show
  while station 2 is being marked, which is exactly when an operator wants to see
  that E1 landed where they expected. A fresh E1 drops the previous E2, so last
  run's second point never sits beside this run's first.
* **Otherwise they mirror the selected procedure's taught frame**, which makes
  the same overlay a check on what is actually stored: drive to local (0, 0) and
  E1's mark should sit under the centre crosshair.

The two agree by construction once a run finishes — `align_solve` persists the
same points the queue delivered.

## 7. The commands

| Command | Role |
|---|---|
| `vision_detect` | locate the feature once; `<prefix>.ok/.x_px/.y_px/.dx_mm/.dy_mm/.angle_deg/.score/.radius_px/.count/.elapsed_ms/.error` |
| `vision_test` | run N times; `<prefix>.runs/.ok/.ok_pct/.mean_score/.mean_x_px/.mean_y_px/.std_x_px/.std_y_px/.max_dev_px` |

`require = false` by default on `vision_detect`, so a recipe branches on
`vision.ok` instead of dying halfway — the `halcon_status` pattern.

`ParamType::VisionUid` copies `AlignUid` exactly: resolved by uid *or* name, not
device-scoped (a model belongs to the camera, not a stage), validated by
`CheckReferences`, rendered as a picker by `DrawParamWidget`. Validation checks
**existence only** — whether HALCON can actually run it is a run-time question,
and failing validation on a licence would leave an unlicensed machine unable to
even open a recipe that mentions vision.

Freshness reuses the auto-focus gate (`fresh_frames = 2`): frame #1 may have
been integrating while the stage travelled, frame #2 is wholly from the station.
On a stalled feed it degrades to "detect on the latest frame" and warns once.

## 8. `align_station` in auto mode

```
resolve the model BEFORE moving        -- a typo costs no travel
apply exposure (station's wins, else the model's)
move, WaitArrive, WaitSettle
wait fresh_frames, Workflow::Detect
pick.px/py/dxMM/dyMM  <-  det.xPx/yPx/dxMM/dyMM
```

Both routes fill the **same `PickResult`**, so everything downstream —
`e.wx = gx + pick.dxMM`, the ctx writes, `align_solve`, `QueueAlignResult` — is
byte-identical whether a detector or an operator produced the point. There is
still exactly one way a taught frame comes into existence.

The failure resolution is deliberate: a missing model, an unlicensed HALCON and
a genuine miss all land in one `autoErr` string, checked in one place. Falling
back to the crosshair is opt-in (`auto_fallback_manual`) **because** a silent
fallback would let a procedure claim to be automatic while a human placed every
point.

### The pixel→mm calibration moved to layer 2

`docs/alignment-procedure-design.md` §3 says the conversion happens in the UI,
because only the UI held the live `pixel_to_mm` values. An auto station has no
UI in the loop, so `Workflow` now owns them (`CameraPixelToMm` /
`SetCameraPixelToMm` over the `camCfg_` it already captured at Start) and the
Camera page's calibration fields write through it. The manual pick still
converts UI-side, where the operator's click already is; the two read the same
numbers.

## 9. Known limits

1. **No shape-based (edge) matching.** NCC needs texture and does not tolerate
   scale change; a low-contrast edge-defined feature has no good method yet.
2. **No edge/length measurement.** `lengthPx` is reserved and nothing fills it.
3. **`maxMatches > 1` reports only the best.** The count is returned so a recipe
   can notice ambiguity, but there is no multi-result carrier.
4. **Calibration is still two scalars.** `pixel_to_mm_x/y` carry no rotation or
   skew, so a camera rotated relative to the stage biases a detected point
   exactly as it biases a manual one (`alignment-procedure-design.md` §8.2).
5. **Lighting is still a placeholder** — no illumination device exists in
   `config/hardware.json`, so a model's exposure is the only optical setting
   that does anything.
6. **The reliability run does not move anything.** It measures repeatability at
   one pose, not reproducibility across a load/unload cycle.

Related: `docs/vision-layer.md` (focus metric, HALCON gating),
`docs/alignment-procedure-design.md` (the frame the detector feeds),
`docs/state-and-config.md` (the collection and its fallback rules),
`docs/command-layer.md` (registry → editor → Lua bindings),
`docs/ui-file-layout.md` (where the window lives).
