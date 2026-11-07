# Camera setup — exposure, light and the live view filter

How a camera view is LOOKED AT, as one record a recipe step can carry and an
operator can tune against one picture. First filter: binary. First consumer
that is not a page: the Aux window's live view.

Status (2026-11-07): the code layer (sections 1–4), the recipe command
`roi_filter_view_setup` and its editor helper (section 5) are in. Nothing on
the Aux window edits the filter — by design; a run or the helper sets it.

## 1. The record — `core/CameraSetup.h`

```cpp
struct ViewFilter  { int roiMode; bool binary; int thr; };   // display
struct CameraSetup { double exposureUs; AlignLighting lighting; ViewFilter filter; };
```

Two halves with different natures, kept in one record because they are tuned
together against one picture:

| Half | Fields | Nature |
|---|---|---|
| Acquisition | `exposureUs` (0 = leave), `lighting` (kind "" = leave) | PHYSICAL — changes the frame every consumer sees: a detector, a focus score, a saved image |
| Display | `filter`: ROI square (off, 1/3, 1/2, 2/3 of the short side) + binary threshold | DISPLAY ONLY — drawn over the live picture, read by nothing |

Plain data in `core/` because three layers hold it: a recipe command (worker
thread), the recipe editor's helper (UI), and a page's persisted settings
(`UiSettings`, `LoadViewFilter`/`SaveViewFilter` under the view's own prefix).

## 2. The pixels — `vision/RoiFilterOps`

`ViewFilterRect` (the square, centred on a crosshair or the image centre, slid
inside the frame), `ViewFilterGray` (crop to grey through `RgbToGray8`, then
threshold), `OtsuThreshold`, `ViewFilterAutoThreshold` (what *Auto* means,
wherever it is asked). Plain C++: no HALCON licence, because the manual picks
that use it fall under the auto-focus rule. It is **not** a detector
pre-process chain — `HalconDetector`'s chain is the only thing a detector
reads, and nothing reads this.

## 3. The view — `ui/RoiFilter`

Per view: a `ViewFilter` and a `RoiFilterState` (its texture + scratch, never
shared — the texture IS that view's picture). `DrawRoiFilterToolbar` (optional:
a view a run drives has none), `UpdateRoiFilter` (rebuilds on a new `FrameSeq`
or a changed setting only; never returns a stale texture), `DrawRoiOverlay`
into any `CameraView` (`MarkPicker::roi` for a `DrawMarkPicker` page). The
main camera's frame is `App::MainCameraFrame()`. Users: the Device
Registration page (toolbar, persisted as `reg_*`), the Aux view (no toolbar,
runtime only).

## 4. Applying a setup — two entry points, one meaning

| Who | Call | Light | Filter goes to |
|---|---|---|---|
| A recipe command (worker) | `Workflow::ApplyCameraSetup(s, &why)` | `ApplyLighting` — **blocking**, the one implementation, any kind (light / io / psu) | `Workflow::SetLiveViewFilter` → drained in `App::Update` into `auxFilter_` |
| The UI (helper / page) | `App::ApplyCameraSetupNow(s)` | `ApplyLightingNow` — **queued**, "light" kind only (no socket I/O on the UI thread) | `auxFilter_` directly |

`ApplyCameraSetup` FAILS — and hands no filter on — when the camera refuses
the exposure or the light refuses its call-out: a picture taken under the wrong
setup looks exactly like one taken under the right one, the same reason a
detection model whose light cannot be set fails.

The live-view filter is a **slot, not a queue** (only the newest matters),
drained in `App::Update` rather than in the Aux Draw so a closed window still
shows what the run asked for when opened. `auxFilter_` is **runtime only,
never persisted**: a binary picture left over from yesterday's run would hide
the part an operator watches. An inactive `ViewFilter` (ROI off, binary off)
clears it. While one is active the Aux view says so on the picture
("view filter: ROI 1/2 binary 94"), so a filtered view is never read as the raw
frame.

## 5. The `roi_filter_view_setup` command and its helper

A recipe step that sets the camera view up for what follows (a manual pick, a
detection, a picture for the record).

**Command `roi_filter_view_setup`** ("ROI Filter view setup", Vision,
`CommandsVision.cpp`) — named for what the operator sets up: the view the next
step is looked at through.

| Param | Type | Default | Meaning |
|---|---|---|---|
| `exposure_us` | double | 0 | main_camera exposure; 0 = leave it |
| `light` | light name | "" | channel group / channel to set; "" = leave the lighting |
| `color` | enum | white | the call-out on that light |
| `percent` | double | 50 | intensity; 0 = hold the light dark |
| `roi_mode` | int choice | off | ROI square: off / 1/3 / 1/2 / 2/3 of the short side, on the view's centre |
| `binary` | bool | false | draw the ROI as a binary threshold (display only) |
| `threshold` | int | 128 | binary: grey >= this is white |

It builds a `CameraSetup` and calls `Workflow::ApplyCameraSetup` — nothing
else, never a second exposure or light path — and fails when that refuses.
`Needs::Camera`. Not a mover: a teach-mode run disables it, and a dry-run folder
skips it (it is not on `kDryRunCommands`, and does not belong there - it is
neither motion nor IO).

**The mapping is ONE pair of functions** (`command/RoiFilterView.h`):
`RoiFilterViewSetupFromParams` (the command runs it, the helper previews with
it, the step tree's label `view: ...` is drawn from it) and
`RoiFilterViewSetupToParams` (the helper writes an edit back through it). A
step whose light is "" still keeps its colour / percent, so switching the light
back on returns what the operator chose. The UI reads a whole step through
`EffectiveParams` (stored values + schema defaults, `command/CommandSpec.h`).

**Helper window "ROI Filter view setup"** (`ui/windows/WndRoiFilterView.cpp`).
Every param carries `group = kRoiFilterViewHelper` and the command names it as
its `helper`, so the step form shows one *Open ROI Filter view setup...* button
and a summary line instead of seven rows. Like the Dispense helper it edits the
Recipe Editor's SELECTED step in place (`EditorSelectedStep`) and its Save is
the editor's Save; it writes the step only on an edit, never because it looked.

* Left: the live main camera, the WHOLE frame letterboxed, with the step's ROI
  filter drawn on it (the helper's own `rfvFilterSt_` — never the Aux view's,
  which shows what a RUN asked for), centred on the image centre with the
  centre crosshair, where the Aux view centres it too.
* Right: exposure ("leave as it is", a log slider applied on release, *Read*
  takes the camera's current value), the `LightCalloutPicker` (a click lights it
  at once, QUEUED), the shared ROI filter toolbar, the step's summary, then
  *Try on camera* (`ApplyCameraSetupNow` — exposure, light and the Aux view's
  filter, as a run would) and *Clear the Aux view filter*.
* Everything that edits or applies is disabled while `orch_.Busy()`: the run
  owns the camera, and holds its own copy of the recipe.

## 6. Clearing the live view's filter

A filter a run set stays on the Aux view until something clears it. Four
things do, and each clears the DISPLAY half only (exposure and light are
physical settings a later step may rely on, and putting them back would need
to know what they were):

* **`roi_filter_view_clear`** — a recipe step, no params, `Needs::None`
  (`SetLiveViewFilter(ViewFilter{})`). The undo a recipe writes after the steps
  that wanted the filter.
* **Right-click on the Aux picture** — the window's one control over the
  filter; the plate says so ("... (right-click to clear)"). Right, because
  left is click-to-move. No level and no arm: it only removes a display aid.
* **The helper's *Clear the Aux view filter*.**
* **An app restart** — `auxFilter_` is never persisted.

A new `roi_filter_view_setup` replaces it. Open: whether the end of a run
should clear it as well (today it does not).
