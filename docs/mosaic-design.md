# Mosaic: large-area capture, stitch, annotate

One camera field of view is a few millimetres (2448 × 2048 px at 0.00248 mm/px ≈ 6 × 5 mm).
A part is inches. The Mosaic page (**F11**, Technician) photographs a whole area by walking
the gantry camera over it frame by frame, and shows the frames as one picture that can be
panned, zoomed, marked up and exported.

A 2" × 2" area at 15 % overlap is ~120 tiles and a ~20k × 20k px canvas (~1.3 GB of RGB).
That one number drives most of what follows: the tiles stay on disk, the viewer never makes
one giant texture, and the full-resolution export is written band by band.

## Files

| File | Role |
|---|---|
| `core/MosaicConfig` | the areas, the save folder, the camera map: `config/mosaic.local.json` |
| `vision/MosaicLayout` | the plan (which tiles, where the gantry goes), canvas ↔ gantry, compositing, seam refinement |
| `vision/ImageShift` | plain-C++ NCC: how far one picture moved against another |
| `command/CommandsMosaic` | `mosaic_capture`, `mosaic_calibrate` |
| `mosaic/MosaicDoc` | one mosaic folder on disk: record, annotations, refined placement |
| `mosaic/MosaicWorker` | the page's background thread: tile decoding, exports, seam refinement |
| `ui/pages/PageMosaic` | the page |

## Machine state, not product state

A mosaic surveys whatever sits on the machine, taken by the camera on the gantry. It belongs
to no product and no DUT. So the areas live in `config/mosaic.local.json` (falling back to the
shipped `config/mosaic.json`), like the needle calibration. The pictures go to a folder the
operator sets once on the page (default `C:\mosaic`, created on demand).

The page is the only writer of the config and saves on every change. The capture reads the
**file** at Start, and every editing surface is disabled while `orch_.Busy()`, so the page
and a running capture never write and read it at the same time.

## The area

An area has 3 or 4 corners. Each corner is a gantry **axis** reading (X, Y, Z), taught with
the camera over the corner and in focus.

- **3 corners** make a parallelogram, using the waffle tray's convention: C1 origin, C2 along
  one edge, C3 the far corner. C4 is derived as C1 + C3 − C2.
- **4 corners** make any convex quadrilateral, taught in order round its edge. A self-crossing
  or collinear outline is refused by `MosaicAreaValidate`, which is the one place the page and
  the command both get their refusal from.

Z has two modes:

- **Fixed** (the default): every tile is shot at one Z.
- **Plane through the corners**: z = a + b·x + c·y, exact through 3 corners and least squares
  through 4. Teaching each corner in focus is what focuses the whole area, at no cost per
  tile. A tile whose centre falls outside the outline is **clamped** to the range of Zs the
  corners were taught at. The plane is never extrapolated past what the operator focused on.

There is no per-tile autofocus. Where a part needs it, it is a later addition to the walk.

## The canvas is the camera's pixel frame

With **M** the camera's 2×2 mm-per-pixel map, pixel (u, v) of a frame taken with the gantry
at gᵢ sits at

    g = gᵢ + M · (u − W/2, v − H/2)

and the canvas is defined by g = g₀ + M·c. Every tile shares the camera's orientation, so on
the canvas a tile is its frame translated so that its centre sits at M⁻¹(gᵢ − g₀). No tile is
ever resampled, whether it is being drawn or exported. A camera turned against the gantry
changes M. That moves where the tiles land and how a canvas point converts back to gantry mm,
and nothing else.

**Placement uses the measured pose**: what the axes reported after settling, never what they
were commanded. The difference is exactly the error a commanded placement would stitch into
the seams. Both poses are kept in the record.

Uncalibrated, M is `diag(pixel_to_mm_x, pixel_to_mm_y)` from `hardware.json`. That is the Aux
window's click-to-move convention: image +x = gantry +X, image +y = gantry +Y, no rotation.
`mosaic_calibrate` measures all four terms (next section).

## The walk: `mosaic_capture`

A registered command, not an engine. The page launches it as a one-step recipe, exactly like
the Jog window's quick autofocus. It therefore inherits `orch_.Busy()`, Stop, the safety abort
and the worker, and a recipe or a Lua script can survey a part too. It is not a teach mover
and not a parallel mover (it owns the camera for the whole walk).

Its only parameter is the area (by name or uid), plus the usual move fragments. The overlap,
settle, fresh frames and Z mode are all taken from the area record. There is one source of
truth for what "the area" means.

1. **Plan** (`PlanMosaic`). A grid at (1 − overlap) pitch covers the outline's bounding box and
   is centred on it. Only frames that touch the outline are kept (a separating-axis test).
   The walk is serpentine. Anything invalid is refused here, before the machine moves: the
   area, a singular M, more than `kMaxMosaicTiles`, or a disk too small for the estimate.
2. **Folder** `<save_dir>/<area>_<YYYYMMDD_HHMMSS>/`. `mosaic.json` is written before the
   first move.
3. **Per tile**:
   - `MoveWithApproach` (the one two-speed move).
   - The settle, then the fresh-frame gate. This is the auto-focus rule: frames wholly after
     the move, proven by `CameraFrameSeq`, never by a sleep.
   - The frame is filed as `tiles/rRR_cCC.png` and `tiles/rRR_cCC_q.png`. Both PNGs use stb
     at compression level 1: at the default level a 5 MP tile costs most of a second, which
     is most of the walk.
   - `mosaic.json` is rewritten, so a stop or a crash still leaves a usable partial record.
   - The quarter preview is queued (`Workflow::QueueMosaicTile`) and the page fills the
     picture in while the gantry walks.
4. **End**: status `complete` or `partial` (with the reason), plus `QueueMosaicDone`.

## One writer per file

| File | Written by |
|---|---|
| `mosaic.json` (capture record) | `mosaic_capture` only |
| `tiles/*.png` | `mosaic_capture` only |
| `annotations.json` (markers, labels) | the page only (UI thread) |
| `placement.json` (refined positions) | `MosaicWorker` only |
| `export/…` | `MosaicWorker` only |

That is the whole concurrency story: no file has two writers, so nothing needs a lock or a
merge.

Annotations are stored in **canvas pixels only**. A marker's gantry coordinate is derived
through the record's own M every time it is shown. Storing both would be two answers to where
a marker is.

## The viewer

The picture is drawn tile by tile, each at the resolution the zoom needs:

- **1/16**: made on the UI thread from the quarter. It is always resident once seen and is
  the whole picture at fit zoom.
- **1/4**: the preview filed with every frame. Paged, capped at 400 textures.
- **full frame**: paged least-recently-used, capped at 16 textures and at 6 decodes in
  flight.

`MosaicWorker` decodes the PNGs off the UI thread. `App::UpdateMosaic` uploads at most a few
per frame and deletes the least recently drawn textures past the caps (`GlHost::DeleteTexture`).
A texture drawn last frame is never evicted.

Each tile is cropped halfway into its overlap with any neighbour, so the on-screen seam falls
where the export's does.

- **Zoom** with the wheel, about the cursor. **Pan** with a middle-drag, the Pan tool, or
  Space + drag. There is also **Fit** and **1:1**.
- The **Marker** tool drops a named point. The **Crop label** tool drags out a named
  rectangle.
- **Right-click** a marker or label for its menu. **Delete** removes the selected one.
- **The right panel** has two plain lists, markers and crop labels; a click selects the item
  and centres the view on it. The selected item is edited in one place below the lists:
  name, **tag**, Go to, Centre, Delete.
- **Filtering**: a name search (case-blind substring) and a tag filter narrow both lists. The
  same predicate (`MosaicAnnShown`) fades the filtered-out marks in the picture (and makes
  them unclickable) and picks what **Export crops** writes.
- **Tags**: a new mark takes the tag being filtered on. A crop exports into
  `crops/<tag>/<name>.png`, or `crops/` when untagged, so a tagged set comes out as a sorted
  dataset.
- **Tiles mode** (the *Tiles* checkbox) outlines every frame and highlights the one under the
  cursor (the nearest centre, i.e. whose pixel it is).
  - With the Pan tool, a click that is not a drag selects it; the arrow keys step to a
    neighbour.
  - **Go to tile** (right panel, or right-click) drives the gantry to the pose that frame was
    **shot at** (its measured X/Y/Z), so the live camera shows what the filed frame shows.
- **Go to** drives the camera centre onto a marker (or a label's centre) at the Z its nearest
  tile was shot at. It needs the jog arm and an idle machine. The coordinate describes where
  things were **when the mosaic was taken**. A part that moved since has moved.

## Exports

These run on `MosaicWorker`. Loads are still serviced between bands, so zooming stays live.

- **Crops**: one full-resolution PNG per label, `export/crops/<name>.png`. A repeated name gets
  a `_2` suffix rather than overwriting.
- **Mosaic** at 1:1, 1:2, 1:4 or 1:8 (a box average), `export/mosaic_1toN_<stamp>.png`.
  - Up to 40 MP it is composed in memory and written by stb, compressed.
  - Past that it is composed **band by band** into a PNG whose zlib stream is made of stored
    (uncompressed) deflate blocks. This is valid PNG, needs no compressor, and keeps memory to
    one band. The price is a file about as big as the raw pixels (a 2" × 2" 1:1 export is
    ~1.3 GB).

Where tiles overlap, the tile whose **centre is nearest** owns the pixel (`ComposeMosaicRegion`).
That is a Voronoi seam, sharp on both sides. A blend would ghost anything that does not
register to the pixel.

## Accuracy

### Camera map: `mosaic_calibrate` (Admin)

A gantry move D shifts the picture by d = −M⁻¹D. The command:

1. Steps +s and −s along X and measures each shift against the start frame with
   `EstimateFrameShift`.
2. Diffs the two shifts. That gives the first column of M⁻¹; the start frame's own error cancels
   in the difference, and so does backlash that is the same both ways.
3. Does the same along Y for the second column.
4. Inverts, so M carries scale, rotation and skew.

It refuses:

- a step that would move the picture past a quarter of the frame;
- a match under `min_score` (nothing textured under the camera);
- a scale more than 30 % from `pixel_to_mm` (a repeating pattern matched one period off).

The result is a **proposal**, never the map in use. It travels `Workflow::QueueMosaicCal` →
`App::Update` → `App::ProposeMosaicCal`, and the Camera map section shows it beside the map in
use: rotation, skew and mm/px for both, plus how many pixels the change moves a seam. An Admin
clicks **Apply** (only then is `config/mosaic.local.json` written) or **Discard**. A calibration
that matched the wrong thing has to be caught by a person reading the numbers, not by the next
capture. It does **not** touch `hardware.json`'s `pixel_to_mm`, which click-to-move and the
alignment still use.

### Camera map from a mosaic (Admin: *Fit camera map to this mosaic*)

A capture already holds the evidence, and more of it than four steps at one spot: every
neighbour pair is a gantry move D (the tiles' **measured** poses) and a picture offset p it
produced, with p = M⁻¹D. `MosaicWorker`'s FitMap job:

1. Measures every pair with `MeasureMosaicPairs`, the same measurement seam refinement uses.
   The search is wider than refinement's, bounded by the overlap, because the map in the
   record may be off by a degree.
2. Fits M⁻¹ by least squares over all pairs (`FitMosaicCameraMap`).
3. Drops pairs off by more than max(2 px, 3 × the median residual) and fits again.
4. Refuses a scale more than 30 % from the one the capture used.

It needs rows **and** columns, because a single row says nothing about the Y axis. It costs no
motion. It lands as the same proposal as the stepping calibration.

On the first real capture (1280 × 1024 frames, 9 × 6 tiles) it gave:

- rotation +0.603°, skew +0.286°, 0.002478 mm/px;
- 48 of 93 seams used, 2.2 px RMS;
- 36 s on the worker.

That agrees with the per-pair angles measured by hand (−0.62° along X, −0.89° along Y for the
picture motion).

Applying a map changes the **next** captures. A capture already on disk keeps the map it was
taken with; *Refine seams* closes its seams.

### Seam refinement (the page's *Refine seams*)

The stage places each tile to within its repeatability. Refinement then:

1. Measures every right and down neighbour pair's actual offset in its overlap, with NCC on
   the quarter previews (the full frames when frames are ≤ 1280 px wide, where a quarter of
   the overlap is too thin).
2. Leaves out pairs that match under 0.6 or peak on the edge of the ±24 px search.
3. Solves for the positions that agree with every remaining pair at once. Each tile is held to
   its stage position by a weak spring, so a tile with no textured overlap stays put.
4. Drops pairs the solution disagrees with by more than 3 px and solves again.

It always starts from the stage positions, so a second run re-measures and never compounds
the first. The result goes to `placement.json`. The stage positions stay in `mosaic.json`, and
*use refined* switches between them.

On synthetic tiles with ±6 px of stage error, the relative error went from 12.5 px to 0.3 px.

### Why plain C++ and not HALCON

`ImageShift` is **registration**, not detection. Both images are of the same patch of the same
part, a known stage move apart, and the only question is the offset between them. It must work
on a machine with no vision licence for the reason auto-focus must: a licence problem may
degrade inspection, never the machine's picture of itself. Feature detection stays HALCON's,
in `HalconDetector`. `RgbToGray8` stays the one RGB→luma conversion.
