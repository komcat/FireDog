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

Now an alignment station can locate E1/E2 without an operator. Three methods —
the first two ported from `bobcat_engine`'s HALCON plugins minus OpenCV, the
third written here:

| Method | Chain | Returns |
|---|---|---|
| `circle` | threshold → closing → connection → area & circularity filter → largest → `fit_circle_contour_xld` (Tukey) | XY, radius, circularity as the score |
| `ncc` | `create_ncc_model` / `find_ncc_model` over a taught patch, plus an operator-placed anchor | XY, angle, score |
| `corner`, **two edges** (default) | a template **synthesised in memory** from the parameters — one quadrant filled, softened — matched by `find_ncc_model` | XY of the vertex, the material direction, NCC score |
| `corner`, solid square | threshold (auto or fixed) → closing → connection → **area window from the size slider** → rectangularity gate → largest → `smallest_rectangle2_xld` → the named corner | XY of one corner, the two edges, rectangularity as the score |

Shape-based matching and edge/line measurement are **not** in this build.

### Why `corner` is its own method and not a tuned `circle`

A machined or printed **square corner mark** is the third thing a station is
routinely asked to find, and neither existing method locates one:

* `circle` fits a circle — a square fails its circularity gate by construction,
  and its *centre* is not the point of interest anyway;
* `ncc` can match a corner, but only after somebody teaches a patch of *that*
  corner at *that* magnification and exposure, and then re-teaches it when any
  of that changes. A corner is a geometric primitive; needing a taught image of
  one is a maintenance cost with no information in it.

### The default is a SYNTHESISED template, not a blob search

The first cut of `corner` only had the blob chain, and it fails on the corner
people actually mark: the **end of a bar**. That region runs off the ROI, so its
area is nothing like `side²` and no area window can contain it — the detector
correctly reported *"no square of that size passed the area/rectangularity
filter"* while the corner sat in plain view.

So mode 1 turns the problem round. A corner is two straight edges meeting at 90°,
which is a **template you can compute**: a square patch with one quadrant filled,
softened by a couple of box-blur passes, matched by ordinary NCC. Consequences:

* **Nothing is taught and nothing is stored.** `MakeCornerTemplate`
  (`vision/VisionModel.cpp`, plain C++ so the teach page can draw the same thing)
  builds the patch at `Load()` from the parameters. There is no `.pgm`, nothing to copy with a
  duplicated procedure, and nothing that can go stale against the numbers beside
  it — the failure mode the taught NCC path has to spend a `templateFile` and a
  patch-lifetime rule on.
* **The feature only has to be locally right.** Whatever the region does outside
  the patch is irrelevant, which is exactly the property the blob chain lacks.
* **The vertex is the patch centre**, so the match reports it directly and
  `useAnchor` stays off — the anchor exists for a taught patch whose point of
  interest is somewhere off-centre, and a synthesised one has no such problem.
* **One `corner` parameter, meaning the same thing in both modes**: which corner
  *of the feature* is reported. The synthesised patch therefore fills the
  quadrant **diagonally opposite** — the bottom-left corner of a shape has the
  shape up-and-right of it. The first cut named the filled quadrant instead
  (`material side`), and it was wrong the way an inverted axis is wrong: every
  reading of "bottom-left" meant the corner being marked, so the vertex came back
  at the far end of the patch from where it was wanted. The teach page draws the
  patch as a **clickable diagram** built from the same `MakeCornerTemplate`, so
  the picture cannot disagree with the search.
* `metric` is `use_polarity` deliberately: the operator said which way the
  contrast runs, and ignoring it would let a dark-on-light corner match a
  light-on-dark one 90° away.
* **The edges are softened before the model is built.** A razor-sharp synthetic
  step still correlates against a real edge, but a real edge is a one-or-two
  pixel ramp, and matching a step to a ramp is what makes the "sub-pixel"
  position land on a whole pixel more often than it should.

The one number that needs care is the size: it is how much of each edge is
matched, so it must stay inside the shortest straight run either edge has — for a
bar end, no more than the bar's thickness, or the filled quadrant reaches past
the far edge and the score collapses.

### The solid-square mode

Kept for a printed or machined square mark that really is isolated. The size is a
**slider** there too: the same mark is a different number of pixels at every
magnification, and the size is what the search is made of — the accepted area
window is `side²` over `[1-tol … 1+tol]`, and that window is what rejects the
rest of the scene.

Three decisions inside it:

* **Sub-pixel comes from the XLD rectangle fit**, not from the region. The
  region is whole pixels, so reading a corner off it would quantise every
  measurement to the grid a reliability run exists to measure.
* **Auto threshold (`max_separability`) is the default.** A corner mark is
  defined by contrast, not by an absolute grey, so a per-frame split is what
  survives a lamp ageing or a change of part finish. The fixed threshold stays
  for a scene where the auto split has something else to latch onto.
* **Which corner is a parameter, and it is named in image terms.** The four are
  classified *after* the fit (`x+y` smallest at top-left, `x−y` largest at
  top-right), so a rotated part still reports the corner the operator picked
  rather than "whichever one HALCON listed first".

`angleDeg` is repurposed rather than expanded: for a corner it is the direction
from the reported point **into the material** — the square's centre in mode 0,
the filled quadrant turned by the match angle in mode 1 — and `widthPx`/
`heightPx` are the two edges leaving that corner, ordered at `angle−45°` and
`angle+45°`. That triple reconstructs the square from the reported point, which is what
lets the teach overlay draw exactly what was found — a *centred* box, the shape
the other methods use, would straddle the corner and show a square that is not
there. `lengthPx` carries the mean side for the readout. **No new fields**:
`VisionResult`'s whole point is that a new detector cannot grow a new shape.

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

### Opening the page drives to the station

`OpenVisionTeach` **commands the move to the station's position before it
switches page**. A model is tuned at one pose — that is the whole reason it is
private to the station — so a page opened wherever the gantry happened to be
standing invites an ROI, a threshold and a template cut out of the wrong scene.

The move is *watched*, never waited on: `App::StartWatchedMove`
(`ui/pages/MainWindow.cpp`) issues `Workflow::MoveToPosition` and parks the
targets in `moveWatch_`; `UpdateMoveWatch` polls arrival from `App::Update`
using the same two signals a move command waits for (every axis within 0.01 mm,
then `IsSettled() && IsOnTarget()`), and `DrawMoveStatusBar` draws a **MOVING**
line with a Stop button under the quick-access strip — above the page dispatch,
so it follows the operator onto this page. Nothing blocks the UI thread, and the
teach page's **Live** re-detect is suppressed while the watch is active: a
detection on a mid-travel frame reports a point on a scene the station will
never see, and the overlay makes it look measured.

A refused move (no position assigned, device not connected, a safety alert) is
logged and the page still opens — the parameter form is worth reading with no
gantry — but it is said out loud, because a page that silently did not move
looks exactly like one that did.

### Three columns

Ported from the sibling **Horizon** project's `FAAM.UI` *Vision Profile* page
(`src/FAAM.UI/Views/VisionProfileView.xaml`), which is the same job on the same
kind of machine. The old two-pane layout queued camera, ROI, method, detect and
reliability into one narrow strip; the work has three parts, so it now has three
columns:

| Column | Contents |
|---|---|
| **image** | the live frame with the ROI/template drags and the filter overlay, and a two-row camera foot (state, the model's exposure + Apply, focus/luma, zoom + Fit, **Overlay**, mm/px) |
| **method + chain** | `detection method` → **pre-process filters** + the filtered picture → the method's own form → search ROI → Detect once / Live → reliability |
| **results** | the matched scene beside the taught patch, the detection as a key/value table, and a one-line status |

Above them a top strip: which station this model belongs to, **Move to** (drives
the owner's position through `StartWatchedMove` — the same watched move opening
the page performs), `Save`, `Save & close`.

Deliberately **not** ported: Horizon's `Vision profile` combo and its
`New / Load / Save / History` buttons. A profile there is a free-standing
document; here a model is private to one station and auto-named, so there is
nothing to pick, name or duplicate. Lighting is left out for the same class of
reason — it has no device behind it and it belongs to `AlignStation`, not to the
model.

The camera foot moved out of the parameter column because those controls are
properties of the *picture*: reading a luma warning two columns away from the
image it describes never worked. The detection numbers moved *into* the result
column for the mirror reason — a detection is a result, and printing it in both
places gives an operator two readings of one thing.

The three columns are separated by two **draggable `Splitter`s**, persisted as
`UiSettings::visionParamW` and `visionResultW` (their own fields, not the main
window's `leftW`/`rightW`: a takeover page replaces that five-region layout
entirely). Both are clamped every frame, and when the window is too narrow for
both they shrink **proportionally** rather than one collapsing whichever happens
to be clamped second. Neither natural width is a constant — a circle model's form
is short, an NCC model's carries a thumbnail and longer labels, and the
operator's font size moves both. Two consequences in the code: labelled widgets
take a **negative** item width so ImGui still has room to draw the label to their
right (`-FLT_MIN` would push every label off the panel), and the explanatory
paragraphs go through `TextWrapped`, since hand-broken lines would spill out of
the very column the operator is allowed to narrow.

### Pre-processing: three stacked filters

Some features only separate after a threshold, a smooth or an edge transform, and
no amount of method tuning substitutes for that. `VisionModel::filters` is a
fixed array of **three `VisionFilter` slots**, each
`none | binary | gaussian | canny`, applied **in slot order** to the ROI plane
before any method looks at it. A stage's output is the next stage's input; a
`none` slot is skipped rather than collapsing the list, so slots 1 and 3 with 2
empty is the chain `[1,3]` and the numbers on screen keep meaning what they say.

The parameter fields are shared across kinds, exactly as Horizon shares them:
`thresholdMin`/`thresholdMax` are the grey band for `binary` and the hysteresis
pair for `canny` (the UI relabels them Min/Max ↔ Low/High), `gaussMask` is 3 or 5,
`alpha` is Canny's smoothing.

| kind | HALCON, in `HalconDetector.cpp` |
|---|---|
| `binary` | `threshold(min,max)` → `region_to_bin(255, 0, w, h)` |
| `gaussian` | `gauss_image(3\|5)` |
| `canny` | `edges_image(…,"canny",alpha,"nms",low,high)`, direction discarded → `convert_image_type("byte")` |

**One seam, two call sites.** The chain is applied in `HalconDetector::Detect`,
between `RgbToGray8`'s crop and the `DetectNcc`/`DetectCorner`/`DetectCircle`
dispatch — and every run-time caller (`align_station`, `place_mark`,
`vision_detect`, `vision_test`) plus the teach page's Detect funnels through
`Workflow::Detect` to exactly there, so a chain cannot be honoured on one path
and forgotten on another. The second site is `HalconDetector::Teach`: **the
template is cut from the filtered plane**, because matching a raw patch against a
filtered live image scores nothing and the failure looks like a badly taught
template rather than a wrong order. Teach filters the same plane a detect will —
the search ROI — and falls back to the whole frame, saying so, when the template
box reaches outside it; silently cropping to the ROI would teach a template of
the wrong size.

**Accepted difference from Horizon:** it reduces the HALCON *domain*
(`reduce_domain`) and keeps the surrounding pixels; FireDog *crops* in
`RgbToGray8`, so a gaussian or a canny sees the ROI edge as an image edge. That
costs a `mask/2` rim and buys the coordinate handling, which is already right.

**The preview is the same code.** `HalconDetector::Preprocess` (static — the
chain is stateless and is usually tuned before there is a template to build a
model from) runs the identical crop and fold and hands the plane back;
`Workflow::PreprocessPreview` wraps it under `detectMu_`. It feeds both the
picture in the filter band and the translucent overlay on the live image (pixels
≥ 128 tinted, drawn at the ROI's own screen rect so a filtered pixel lands on the
pixel it came from). It recomputes on exactly three triggers — the `Preview`
button, a change to the chain or the ROI, or a new frame **while Live is on** —
and never mid-travel.

Two consequences elsewhere: `VisionModelPatchKey` includes the chain summary
(a re-teach writes the same filename at the same size, so without it the
thumbnail would keep showing the patch it replaced), and the Alignment page's
station card prints `VisionFilterSummary` — a pre-processed model behaves nothing
like a raw one and the chain is otherwise invisible from there. `Workflow::Detect`
needs no change: it already stamps the whole model JSON, so a filter edit forces
a detector rebuild.

Without HALCON the filter parameters still edit and still persist —
`VisionModel.cpp` carries no SDK — but the preview and the detection are gone,
each saying so with `HalconEngine`'s reason verbatim.

One thing the help text has to say out loud: **the method's own thresholds now
see the filtered image.** After a binary filter the plane is 0 and 255, so the
circle method's default grey window of 0..100 selects the background.

A few decisions worth keeping:

* **The anchor is placed on the TEMPLATE, not the live image.** The offset is
  defined relative to the template centre, and the patch is the only view where
  that is literally what is on screen. `useAnchor` derives from the numbers
  (`|offset| > 0.1 px`) rather than being a third thing to keep in step with
  them — bobcat's rule, and it removes a state.
* **The pick has its own window** (`DrawVisionAnchorWindow`), and it is the only
  place the anchor can be set. On the parameter column's thumbnail the patch is
  packed into ~200 px, so one click covers several template pixels — a coarse
  guess dressed up as a measurement, and a stray one silently moves the point
  every later measurement is reported at. The window magnifies the patch
  (fit … x32), tracks a drag, and takes arrow-key nudges of 1 px (0.1 px with
  shift). The thumbnail keeps drawing the crosshair and now *opens* the picker.
  Deliberately not modal: Live detect keeps drawing the reported point on the
  big image, and watching that mark move with the anchor is the confirmation.
* **A re-teach clears the anchor and opens the picker.** The old offset was
  measured from a *different* patch centre, so it is reset to the centre —
  which is exactly the state that needs a pick, so the window comes up with it.
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

### The fallback carries the taught picture with it

A fallback pick sets two more fields on the `PickRequest`: `detectorUid` (the
model that was supposed to find the point) and `autoError` (why it did not).
Both are empty on a station that was manual by design — this is evidence about a
*failure*, not decoration on the ordinary pick.

The window resolves the uid and shows the model's **patch with the reported
point crossed on it**, beside the reason. It exists because the operator at the
machine is rarely the person who tuned the detector, and until this the run said
only "mark E1". Nothing on a live image distinguishes one corner of a feature
from the corner the model was taught on — and a mark on the wrong one is not a
visible mistake, it is a frame wrong by the size of the feature, with every move
made through it wrong by the same amount.

`Widgets`' `VisionModelPatch` is the one place a model becomes a picture: the
taught `.pgm` for NCC (cross at the anchor, else the centre), the **synthesised**
`MakeCornerTemplate` patch for a two-edge corner (cross at the centre, which is
where that method reports), and nothing for the blob chains — a circle or a
solid square is matched as a shape, so the window falls back to naming the point
in words. The Alignment page's station panel and the Placement page's object
points draw the same thumbnail from the same helper, so what an operator is
shown mid-run is what the procedure was authored against.

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
