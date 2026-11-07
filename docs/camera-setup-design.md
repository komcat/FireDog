# Camera setup — exposure, light and the live view filter

How a camera view is LOOKED AT, as one record a recipe step can carry and an
operator can tune against one picture. First filter: binary. First consumer
that is not a page: the Aux window's live view.

Status (2026-11-07): the **code layer is in** (sections 1–4). The recipe
command `roi_filter_view_setup` and its editor helper (section 5) are the next step and are not
written yet. Nothing on the Aux window edits the filter — by design.

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

## 5. Next — the ROI Filter view setup command and its helper (planned)

A recipe step that sets the camera up for what follows (a manual pick, a
detection, a picture for the record):

* **Command `roi_filter_view_setup`** ("ROI Filter view setup") — named for
  what the operator sets up: the view the next step is looked at through.
  Params `exposure_us` (0 = leave), `light` (name; "" = leave), `color`,
  `percent`, `roi_mode` (off / 1/3 / 1/2 / 2/3), `binary`, `threshold`. It builds a `CameraSetup` and calls
  `Workflow::ApplyCameraSetup` — never a second exposure/light path. Needs the
  camera (and the light when one is named). Not a mover: a teach-mode run
  disables it, a dry-run folder skips it (not on `kDryRunCommands`).
* **Helper window "ROI Filter view setup"** — `CommandSpec::helper` names it
  (a `kRoiFilterViewHelper` constant beside `kDispenseHelper`), the params
  share its `group`, so the step form shows one button + a summary line
  (`CameraSetupSummary`). The helper edits the Recipe Editor's SELECTED step in
  place (`EditorSelectedStep`, never a path of its own), over a live
  main-camera view carrying the shared ROI filter toolbar, an exposure slider
  and the `LightCalloutPicker`. *Try* applies it through `ApplyCameraSetupNow`;
  the saved parameters are the step's — the recipe file is the persistence.
* Open: whether a run's view filter is cleared when the run ends, or stays until
  the next setup (today: it stays until the next setup or an app restart).
