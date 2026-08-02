# Alignment procedure: a taught local coordinate system (O1)

*Why the frame is two points and rigid, where the crosshair pick lives, and how
an auto station gets the same point from a detector. Reference material for the
shipped code:
`src/core/AlignFrame.{h,cpp}`, `src/command/CommandsAlign.cpp`,
`src/ui/pages/PageAlign.cpp`, `config/scripts/alignment_procedure.lua`.*

---

## 1. The problem

A part is never loaded twice in exactly the same place. Every taught position in
FireDog is a raw machine coordinate, so the part's placement error lands
one-for-one on every downstream move, and the only remedy is re-teaching
positions per part.

The alignment procedure fixes the **reference** instead of the positions. The
operator marks two features on the real part — **E1** and **E2** — at two
stations:

```
origin = E1        local X+ = E1 -> E2        local Y+ = X+ rotated +90 deg
```

Coordinates expressed in that frame (**O1**) stay valid whatever the placement.
Run the procedure once per part, then `move_local` is precise.

## 2. Why two points, and why rigid

Two points is the minimum that fixes both **where** the part is and **how it is
turned** — one point gives only a translation. It is also the most an operator
can mark reliably without a detector.

The frame is **2-D rigid**: origin + rotation about Z. No scale term, and no
tilt. Scale was rejected on purpose: dividing the measured E1→E2 span by a
nominal span would absorb a mis-pick into a plausible-looking scale factor and
hide it. Instead the measured span is **recorded** (`span_mm`) and shown on the
page, so a bad pick is visible as a number that does not match the drawing.

`SolveAlignFrame` refuses a span below `kAlignMinSpanMM` (0.05 mm): below that,
pick noise dominates the direction and the angle is meaningless. Refusing beats
emitting a frame that is quietly rotated by degrees.

There was nothing to reuse for this. `AxisTransform`
(`hardware/HardwareConfig.h`) is a signed permutation — it cannot hold an
arbitrary angle or an offset — and `MotionMap` is a graph over positions, not a
geometry. `core/AlignFrame` is the project's only frame maths.

## 3. How a point becomes a coordinate

Manual mode marks a point by **placing a crosshair on the live image**. Nothing
moves during a pick, so it costs no travel and no settle:

```
E = ( gantry_X + (px - width/2)  * pixel_to_mm_x,
      gantry_Y + (py - height/2) * pixel_to_mm_y )
```

Sign convention: camera +X is screen-right and camera +Y is screen-**down**, and
screen-down is machine **+Y** — the convention `Workflow::MoveGantryRelativeMM`
already established for the Aux window's click-to-move. The pick window uses the
same "cover" fit and the same screen↔pixel mapping as `WndAux.cpp`, so a pixel
means the same thing in both surfaces.

**The pixel→mm conversion happens in the UI, not in the command.** Only the UI
holds the live `pixelToMmX_/Y_` calibration, so `PickResult` carries both the raw
pixel (for the record) and the offset in millimetres (for the maths), and the
worker never needs a camera-config accessor.

The crosshair starts at the image centre. OK without clicking therefore means
"the feature is dead centre" — offset (0, 0) — which is the correct answer when
the operator jogged the feature under the centre crosshair first. Arrow keys
nudge the mark **1 camera pixel** (hold to repeat, Shift: 10 px).

The view is the old cover fit times a **zoom**, default **1.5× (in 50 %)** so a
small feature is legible; the wheel over the image and the footer slider change
it, `Fit` returns to 1.0, and the value is kept across picks so one setting
serves both stations. Two consequences of zooming worth knowing:

* the visible window **follows the mark** (clamped to the image), so a zoomed
  view can never hide the crosshair and the nudge always stays on screen — a
  click therefore recentres on what was clicked;
* the image-centre reference crosshair can fall outside the visible window, so
  the overlay is drawn inside a `PushClipRect`.

Because zoom ≥ 1 and the base fit is a *cover* fit, the visible window is always
inside the image — there is no letterboxing case to handle.

Two things the nudge depends on, both easy to get wrong:
`BringWindowToDisplayFront` changes **draw order only**, so the window needs an
explicit `SetNextWindowFocus()` when it appears or the keyboard still belongs to
whatever had it; and the keys are taken only while the window is focused *or
hovered*, never globally, because the window is non-modal and an arrow press
aimed at another surface has to stay there.

## 4. The pick handshake

The capture is an operator-in-the-loop wait on an engine worker thread, which is
exactly the shape `manual_adjust` already had — so it is the same handshake, not
a second invention:

```
command            ctx.AwaitPick(req, &result)
  -> engine        AwaitPickInternal: cv_.wait_for in 100 ms slices
                   (so RequestStop always gets through)
  -> Status        pendingPick + pickReq, polled by the UI
  -> UI            App::DrawAlignPickWindow (WndPrompts.cpp), non-modal,
                   BringWindowToDisplayFront
  -> engine        AnswerPick(ok, result) -> notify_all
```

Both engines implement it, and the window routes to whichever has it pending —
identical to the prompt and adjust windows beside it. Cancel fails the step; a
Stop releases the wait and ends the run.

The **result** travels the other way through the taught-position route:
`align_solve` calls `Workflow::QueueAlignResult`, and `App::Update` drains it,
stores it against the procedure uid and persists — because `ProductSession` is
UI-thread-only and the engine holds a *copy* of the document.

## 5. Auto mode

An `auto` station locates its feature with a **taught detection model** —
circle/fiducial or gray NCC pattern match, both HALCON-backed. The station's
`detector` field holds the model's uid; it was put there precisely so a
procedure authored before detectors existed would survive their arrival, and
uid-or-name resolution means an old free-text value still resolves.

The capture is the only thing that differs. `Workflow::Detect` fills the **same
`PickResult`** the crosshair window fills — pixel plus millimetre offset from
the image centre — so `e.wx = gx + pick.dxMM`, the ctx writes, `align_solve` and
`QueueAlignResult` are byte-identical either way. There is still exactly one way
a taught frame comes into existence.

Failure is loud on purpose. A missing model, an unlicensed HALCON and a genuine
miss all resolve to one message, and the step **fails** with it unless
`auto_fallback_manual` is set — in which case it logs a warning and asks the
operator. A silent fallback would let a procedure claim to be automatic while a
human placed every point.

**The manual pick still never depends on HALCON** — it is ImGui over a Pylon
frame, so a licence problem cannot stop an operator teaching a frame. Detection
is deliberately outside that rule; see `docs/vision-detection-design.md` §3.

Each captured point is also queued for the UI the moment it exists
(`Workflow::QueueAlignPoint`), so the **Aux window** can mark E1 while station 2
is still being captured — `align_solve` only fires after both, so without that
queue there would be nothing to show at precisely the moment an operator wants
to confirm E1 landed correctly. The marks are stored in machine mm and projected
back through the live gantry pose, so they stay on the feature as the stage
moves; detail in `docs/vision-detection-design.md` §6.

One thing moved because of this: the **pixel→mm calibration now lives in
`Workflow`** (`CameraPixelToMm`/`SetCameraPixelToMm` over the `camCfg_` captured
at Start), because an auto station has no UI in the loop to do the conversion.
The manual pick still converts UI-side as §3 describes; both read the same
numbers, and the Camera page's fields write through to layer 2.

## 5b. The frame's lifetime — live vs recorded

A taught frame describes where **one physical part** sat. The moment that part
could have changed, the frame is not merely stale but actively dangerous:
`move_local` would drive to coordinates derived from it without complaining, and
the resulting move looks exactly like a correct one — it is simply wrong by
however far the part shifted.

So the frame exists in two places with two different jobs:

| | Where | Job |
|---|---|---|
| **Live** | `Workflow` (`SetLiveFrame` / `LiveFrame`) | the only thing `align_apply`, `move_local` and *Move here* will consume |
| **Recorded** | `AlignProc::result`, in the product file | history: the Alignment page shows it, the Aux overlay draws its E1/E2, traceability keeps it |

**No run ever consumes the recorded copy.** `align_solve` sets the live frame
unconditionally — `save` decides whether the frame is *recorded*, not whether
this session may move against the part it just measured.

The live frame is cleared at every boundary where the part might have changed:

* **program start** (`App::StartDeckRun`) — every deck run teaches its own frame
  or fails loudly;
* **DUT open** — edge-detected in `App::Update` off `DutSession::GetInfo()`
  rather than hooked into the two `Open` call sites, so a third one cannot be
  added without the reset firing;
* **product load / app start** (`App::ResetDocumentSelections`).

Why `Workflow` and not the engine's `Env`: **a program runs each slot as its own
engine `Start`**, so per-run state would be wiped between the recipe that aligns
and the recipe that works. The lifetime has to be the program, and `Workflow` is
the layer that spans runs.

This is also what makes the one-run pattern work at all. `align_solve` cannot
write into `Env::aligns` (a `const` copy-at-Start), so before this the sequence

```lua
fd.align_run{ alignment = "calibrate plate" }
fd.move_local{ alignment = "calibrate plate", x_mm = 3.5, y_mm = 0 }
```

would have moved against the *previous* run's frame. Now the first line
publishes to `Workflow` and the second reads it back — same run, same program,
any number of slots later.

The Alignment page shows the two states separately (`LIVE taught …` versus
`NO LIVE FRAME` over the record), and *Move here* is gated on the live one:
a button that would drive against last week's record is the same bug with a
friendlier face.

## 6. Storage

A procedure is product data, so it lives with the product: manifest role
`alignments`, file `{uid}.alignments.json`, with the usual
`config/alignments.local.json` → `alignments.json` fallback when no product is
loaded. Persistence is copied verbatim from the recipes block in
`ProductSession` — explicit Save plus a dirty baseline, because a procedure is a
**document**, not autosaved state like positions and maps.

Products created before this feature have no `alignments` manifest entry, and
without one their saves would fall back to `config/` forever. `LoadProduct`
therefore calls `EnsureManifestRole(product_, "alignments")` — a one-shot
backfill that mints a uid and rewrites the manifest at **load** time, never in a
Draw function.

The one thing that does *not* wait for Save is the taught frame itself:
`ApplyAlignResult` persists immediately, because a frame lost to a crash means
re-teaching the part.

### Private station positions

A station can point at any saved position, but the natural workflow is *jog
until it looks right, then teach* — so the page's **Teach here** button captures
the device's current pose as a position **private to the procedure**:

* named `algn_<procUid>_<station>` — the name IS the ownership record, so no
  schema change was needed and an orphan is always traceable to its owner;
* `hidden = true`, which already keeps it off the Motion page's table and now
  also out of the recipe editor's `PositionUid` picker (that filter was missing
  and applies to every system position, not just these).

Three consequences that are deliberate:

1. **Re-teaching updates in place.** The uid stays stable, so a station — or
   anything else referencing it — survives a re-teach.
2. **Deleting a procedure deletes its positions** (`DropAlignStationPositions`).
   Hidden rows are somewhere nobody looks; an orphan would linger forever.
   Duplicate copies them under the new owner rather than sharing rows the
   original's Delete would take away.
3. **Hidden is withheld from the picker, not blocked at validation.** An
   existing reference still resolves — only the *offer* is withdrawn. Blocking
   by uid would force the recipe validator to know about alignment ownership,
   and a hand-written recipe would fail confusingly.

The Motion page's "Reveal hidden rows (dimmed)" escape hatch keeps working on
purpose: when a station is 2 mm off, somebody has to be able to see the number.
The station card shows the coordinates too, since a private position is not on
the Motion table.

### Private detection models

An **auto** station's detector follows the identical pattern, for the identical
reason. It is named `vdet_<procUid>_<station>`, created on first use by the
teach window, dropped by `DropAlignStationDetectors` when the procedure is
deleted, and copied (patch file included) by `CopyAlignStationDetectors` when it
is duplicated. There is no picker and no rename: an ROI, a grey window and a
taught patch only mean anything at the pose and exposure they were tuned at, so
a model that happens to suit a second station is a coincidence rather than a
reuse. Detail in `docs/vision-detection-design.md` §2.

## 6b. Nominals — targets expressed in the frame

A frame on its own only says where the part is. The thing an operator actually
wants to name is a **place on the part**: where a die goes, where a fibre must
land, the centre of a footprint. Held as a machine coordinate that place is
wrong for the next part by exactly the placement error the frame exists to
absorb; held in **local (O1)** coordinates it is right for every part, because
the frame moves and the coordinate does not.

So a procedure carries a **named list of nominals** (`AlignProc::nominals`),
each of them just `lx`/`ly` plus the record of how it was taught. A list rather
than a single point because one procedure's frame typically serves several
targets, and because a name is how anything downstream will address one.

### Two marks, not one

A nominal is taught as the **midpoint of two marks**. The reason is the same one
that makes a centre hard to teach at all: the interesting point is usually the
*middle* of something, and a middle has no feature under it to put a crosshair
on. Marking two opposite corners puts the crosshair on something real both
times and leaves the centre to arithmetic.

This is deliberately *not* the frame's two-point rule. E1/E2 fix an origin **and
a direction**, so `SolveAlignFrame` refuses a short span — below `kAlignMinSpanMM`
the angle is noise. A nominal is a point: two coincident marks give a valid (if
pointless) answer, so `SolveAlignNominal` imposes no minimum and records the
span as a diagnostic instead.

What it *does* refuse is an **invalid frame**. `AlignMachineToLocal` is an
identity when the frame is not valid, so a nominal taught through one would be a
machine coordinate wearing a local coordinate's name — and the next frame would
faithfully re-project it to somewhere no part has ever been.

### The teach page

`Page::AlignNominal` (`ui/pages/PageAlignNominal.cpp`) is a takeover reached
only from the frame panel's *Teach nominal…*, on exactly the argument
`PageVisionTeach` makes: marking a feature needs a big live image, and the
Alignment page's columns are strips. It shares `CameraView` with the crosshair
pick window and the vision teach page, so a pixel means the same thing in all
three.

Three things about it are worth stating outright:

* **Nothing moves.** It reads the live image and the device's pose and writes a
  document field, so there is no arming checkbox — the pick window's argument.
* **Each mark latches its own pose.** `wx = gx + (px - w/2) * pixel_to_mm_x`, the
  same arithmetic §3 describes, evaluated at the moment the operator presses
  *Set*. Jogging between the two marks is therefore expected, not a hazard, and
  the two may come from opposite ends of the travel. Captured marks are drawn by
  projecting back through the **current** pose, so they stay on their features
  as the stage moves — the Aux window's E marks again.
* **It needs a LIVE frame** (§5b), not the record. A nominal is *stored* in
  local coordinates, so the conversion that produces them has to come from a
  frame describing the part under the camera right now. Teaching against an old
  record would produce numbers that describe nothing on this part — and, unlike
  a bad move, would then be inherited by every part after it.

Saving persists immediately (`SaveAligns`), like a taught frame and for the same
reason: two hand-placed marks are expensive to lose. Re-teach keeps the uid so
existing references survive; a duplicate name inside one procedure is refused,
because the name is how a nominal is addressed and two of them would make that
lookup a coin toss.

Duplicating a procedure **keeps** its nominals while clearing its `result`. That
asymmetry is the whole idea in one line: the frame is per-part, the nominals are
per-design.

### What consumes one

The frame panel's *Use* button loads a nominal into the local→machine probe
fields, so *Move here* drives the **procedure's own device** to it — still
behind the live-frame gate and the arm checkbox.

## 6c. Placement — putting a gripped object on a nominal

The interesting case is the other one: the object is held by **hex-left**, and
the nominal is a point the **gantry's** camera measured. That is not a different
coordinate, it is a different *kind of move*.

**Only a delta can cross between the two devices.** A hexapod's origin has never
been measured against the gantry's, and `AxisTransform`
(`hardware/HardwareConfig.h`) is a signed permutation with **no offset** — it can
carry a direction between frames but never a position. An absolute machine XY
therefore cannot be commanded to hex-left at all. What can:

```
object   = midpoint of two marks           (machine mm, through the camera)
nominal  = LocalToMachine(live frame, n)   (machine mm, at THIS part's pose)
delta    = nominal - object                (machine mm — a vector)
per-axis = transform * delta               (the mover's own axes)  -> relative move
```

The frame's **origin cancels** in that subtraction, which is exactly what makes
the result meaningful to a device that knows nothing about it. The **rotation
does not cancel**, and must not: both points go through the *same* live frame,
so the vector already carries the part's actual orientation.

Two things that would be easy to get wrong:

* **`JogGlobal` is not the mapping to reuse.** It collapses a direction to its
  single dominant axis — right for a jog button, and it would silently drop half
  of a diagonal correction here. `Workflow::GlobalDeltaToAxes` keeps every
  component; it is layer 2 owning the kinematics, so the page that *prints* the
  correction and the move that will eventually *command* it cannot drift apart.
* **The object must be measured, not assumed.** Where a gripper picked the part
  up is exactly the error being corrected, so a placement that skips the
  measurement is an open-loop move wearing a closed loop's name.

`Page::AlignPlace` (`ui/pages/PageAlignPlace.cpp`, reached from a nominal's
*Place…*) is that arithmetic with every intermediate value on screen: object
machine and local, delta local and machine, the distance, the per-axis relative
move, and the mover's resulting absolute pose so travel limits can be eyeballed.

*Move object to nominal* issues it through `Workflow::MoveRelativeGlobal`, which
re-derives the per-axis deltas from the same transform, adds them to the live
pose and sends one coordinated command — so what the page prints and what the
machine is told cannot drift apart. The page hands over the machine-frame vector
and nothing else; layer 2 owns the kinematics, as it does for every other move.

Three guards, because the destination is **computed** — the same risk class as
click-to-move, and unlike a station's *Go* there is no human-taught pose behind
it:

1. **The shared arming flag** (`jogArmed_`, the one the Jog window arms), plus
   the usual `orch_.Busy()` gate.
2. **A visible ceiling** on the correction (default 2 mm, editable on the page).
   A mis-taught frame does not produce an implausible number — it produces an
   ordinary one that happens to be huge — so the limit is on screen where it can
   be judged and raised deliberately, not a constant only a rebuild can change.
3. **A move clears the marks.** They describe where the object was *before* it,
   so leaving them would display a correction already spent, and a second press
   would apply it twice. Clearing forces the re-measure, which is what makes this
   a closed loop at all: the residual delta is the accuracy number, and nothing
   about the transform can promise it in advance. The last issued move stays on
   screen to compare against.

### Knowing what a residual means

After a correction the object is measured again, and the leftover delta is the
accuracy number. But a leftover of 0.1 mm means nothing until you know what two
measurements of a *stationary* object disagree by — that scatter is the marking
error, and it is the floor under every placement.

So the page keeps a **repeatability sample set**: mark, *Log measure*, mark
again, and it reports the spread in X and Y plus the worst distance from the
mean. Logging clears the marks, because an independent sample means re-marking;
a **move clears the whole set**, because samples taken before it describe a
different position and would report the move itself as scatter.

Read it as the discriminator it is: a residual comfortably inside the spread is
the marking, not a miss, and a correction smaller than the spread is noise being
chased. It also separates the two ways a placement can disappoint — a bad
transform shows up as a residual much *larger* than the spread and in a
consistent direction, while sloppy marking shows up as both numbers being
similar and neither improving with another move.

Nothing here is persisted: it measures this operator, these optics and this
part, right now.

Both mark pickers are the same widget (`DrawMarkPicker`, `ui/Widgets.h`),
hoisted out of the nominal page the moment this one needed it: two copies of the
machine-mm↔pixel projection would drift, and the point of the shared
`CameraView` is that a pixel means the same thing everywhere.

## 7. Commands and the script

| Command | Role |
|---|---|
| `align_station` | drive to the station, apply exposure/lighting, capture E1 or E2 into ctx (crosshair or detector) |
| `align_solve` | build the frame from the two ctx points, publish it to ctx, persist it |
| `align_apply` | local (O1) → machine XY into ctx; fails unless a **live** frame was taught this session (§5b) |
| `move_local` | `align_apply` + one coordinated move (arrival + settle via `MotionWaits.h`) |

The Alignment page's right panel is the interactive twin of the last two: type a
local X/Y, read the machine XY, and **Move here** drives to it (XY only — the
frame is 2-D, so Z is left where it is rather than invented). That button sits
behind the shared `jogArmed_` flag, unlike the station cards' `Go`: `Go` drives
to a pose a human taught, while this destination is *computed*, so a mis-taught
frame sends the stage somewhere unexpected — the same risk class as
click-to-move.
| `align_run` | typed one-step wrapper over `alignment_procedure.lua` |

`align_run` is the **`xcan` pattern**: a real parameter form whose values become
ctx keys, then `RunLuaFile`. The script *is* the procedure — editable, and
re-runnable from the Scripts page, from a recipe step, from Lua
(`fd.align_run{...}`, since every registered command gets an `fd.` binding), and
from the Alignment page's Run button, all through the same code path.

**Which file it runs**: `ExecutionContext::ResolveScript` prefers the product's
own copy of that filename (under `ScriptsDir()`, the dir the session actually
scanned) over the shipped `config/scripts/` one. The Alignment page's Run button
already did this; without the same rule in the command, "run the alignment
procedure" would mean a *different file* depending on which surface launched it —
a recipe would silently run the shipped script while the page ran the edited one.
The page expresses the rule over its already-scanned list because it launches the
script directly rather than through a command; the two must stay in step.

`run_script`, `xcan` and `raster_scan` still take their `file` literally. That is
a known inconsistency, left alone deliberately: changing it would re-point any
existing product that happens to carry a stale copy of `hexapod_align.lua`.

That last point is the design's centre of gravity: **the page does not capture
points itself.** Run launches the script; the pick window surfaces; the frame
comes back through `QueueAlignResult`. There is exactly one way a frame can come
into existence, so teaching and a scripted re-run cannot drift apart.

`alignment` is a `ParamType::AlignUid` — resolved by uid *or* name (recipes
store the uid, so renaming is safe), validated by `CheckReferences`, and
rendered as a picker in the recipe editor. It is **not** device-scoped like
`PositionUid`: a procedure carries its own device.

## 8. Known limits

1. **No zoom in the pick window.** Precision is bounded by the on-screen scale
   plus the arrow-key nudge.
2. **Calibration is two scalars.** `pixel_to_mm_x/y` carry no rotation or skew
   term, so a camera rotated relative to the stage biases every pick — manual or
   detected alike. The frame cannot correct for that — it is measured through
   the same lens.
3. **Z is untouched.** `move_local` passes `z_mm` straight through as a machine
   coordinate; the frame is 2-D.
4. **Lighting is a placeholder.** No illumination device exists in
   `config/hardware.json`; `kind = ""` is the only setting that does anything
   today, and the io/psu paths are wired but unexercised.
5. **Local-frame saved positions are not implemented.** Tagging a `SavedPos`
   with an alignment so `move_to_position` auto-retargets is the obvious next
   phase; it touches the positions schema, the validator and every move path,
   so it was kept out of this one.
6. **A nominal has no command yet.** It is document data plus two pages and the
   frame panel's *Use*; no recipe step reads one.
7. **Placement is a UI action only** (§6c). There is no `place_local` command,
   so a recipe cannot do it: measuring the object needs an operator marking two
   features, which is the same reason an `auto` alignment station had to wait
   for a detector. The move itself is already layer 2
   (`Workflow::MoveRelativeGlobal`), so a command would be the marking, not the
   motion.
8. **Placement corrects translation only.** A nominal is a point, so the
   correction is a 2-D shift: if the gripper picked the object up rotated, the
   page cannot see it and the shift will not fix it. The information is
   *available* — the object's two marks have a direction, and storing the
   nominal's mark direction alongside its midpoint would give a Δθ to command on
   the holder's W — but a nominal deliberately stores a point today, so that is
   a schema change, not a calculation.

Related: `docs/state-and-config.md` (the collection and its fallback rules),
`docs/command-layer.md` (registry → editor → Lua bindings),
`docs/vision-layer.md` (why nothing here needs HALCON),
`docs/ui-file-layout.md` (where the page and the window live).
