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
circle/fiducial, gray NCC pattern match or square corner, all HALCON-backed. The station's
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

When it does ask, the request carries the failed detector's uid and the reason
(`PickRequest::detectorUid` / `autoError`), and the pick window shows that
model's taught patch with the point it reports crossed on it. Asking a human for
"E1" without showing which point of the feature the detector was taught on gets a
plausible mark on the wrong corner, and that is a frame wrong by the size of the
feature — see `docs/vision-detection-design.md` §8.

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

## 5c. The guard band, and the record of every run

`SolveAlignFrame` can only refuse two marks that are on top of each other
(§2). Everything else it accepts, because two points that are apart *do* define
an origin and a direction — it has no way to know they are the **right** two
points. E2 marked on the wrong instance of a repeated feature yields a frame
wrong by the pitch between them, published as LIVE, and every `move_local` after
it wrong by the same amount with nothing on screen saying so.

The **guard band** (`AlignGuard`, on the procedure) is the missing check, and it
is deliberately two numbers of different kinds:

| | what it is | reference | band |
|---|---|---|---|
| **span** | a fact about the **part** — the distance between the two features is on the drawing and is the same on every part | taken from a good run (*Set from last run*) | tight; a breach means the wrong feature was marked |
| **theta** | a fact about the **fixture** — where it holds a part, which the part itself then varies around | 0 (square to the machine), or taken from a run when the fixture sits at an angle | wide; it catches a part in backwards or a slipped clamp |

*Set from last run* fills **both** references, because a fixture that holds
every part at 0.55° is a fact worth capturing rather than retyping. What it
never fills is a **tolerance**: that is a judgement about how much a good part
may vary, and the mistake this table exists to prevent is reading theta's band
as a measurement tolerance and tightening it to whatever today's part happened
to measure.

It is **off by default**, so every procedure authored before it behaves exactly
as it did, and `AlignGuardCheck` on a disabled band returns *in band* with zero
deviations — an inactive band never refuses anything, and every caller can
therefore ask unconditionally.

### The re-mark loop is the crosshair

An enabled band does **not** refuse a frame. It surfaces where the mistake can
still be undone: while E2 is being marked.

`align_station` fills `PickRequest::band` for station 2 — E1 in machine mm, plus
the span and theta limits — and the pick window projects it exactly as it
projects a `PickHint` (`DrawWorldBand`, `ui/Widgets.cpp`, through the same
`WorldMarkView`). The band is an annulus **sector** centred on E1, and only part
of it is ever on screen: at a 7 mm span in a 3 mm field of view the operator
sees two arcs and the two limit rays crossing the view, and moves the crosshair
between them. The verdict is recomputed every frame from the mark, so **moving
the mark is the retry** — there is no loop to run, no attempt counter and no
second question to answer.

The band is filled by the worker rather than the UI for the reason every hint is:
only the worker knows E1. It was captured at the other station, one move ago,
and the window sees nothing but the current field of view.

Out of band, `OK (Mark here)` becomes **`Accept out of band`** — different
label, different colour, and an `FD_WARN` naming both deviations. The band is a
sanity check, not an interlock: the person at the machine can see the part and
the software cannot, and refusing them outright would only teach them to switch
the band off. What it must not do is let that acceptance pass for an ordinary
OK.

An **auto** station gets the same treatment through the path that already
exists. A detection that succeeds but lands outside the band is treated as a
detection *failure*: the station falls back to a manual pick if
`auto_fallback_manual` is set — with the band drawn and the miss in the
message — and fails loudly if it is not. A high match score says the match was
good, not that it was the right feature.

`align_solve` therefore **reports** rather than enforces: by the time it runs,
an out-of-band point has been consciously accepted at the window, or an auto
station without a fallback has already failed. Failing there would only throw
away a frame somebody deliberately kept. It writes `<prefix>_guard_ok`,
`<prefix>_span_err_mm` and `<prefix>_theta_err_deg` into ctx, which is what lets
a recipe act on the verdict anyway — `dut_record` puts it on the part's record,
and a branch can refuse to carry on.

### The run history

Each procedure keeps its last `kAlignRunHistoryMax` (100) solved frames as
`AlignRun` rows: the timestamp, E1 and E2 in machine mm, the E2−E1 vector, span,
theta, whether a band was in force, and the verdict. The row is built in
`ProductSession::ApplyAlignResult` — the one writer of `result`, which already
persists immediately — from the procedure's own band and the frame's own two
points, so no verdict has to cross the worker/UI boundary.

Two lifetimes, on purpose, and they are the same distinction as §5b one level
down: `result` is a **licence** and is cleared at every boundary where the part
might have changed; `runs` is a **record** and is cleared by nothing, because
the question it answers — *what did the last hundred parts measure?* — is exactly
the one asked when one of them comes out wrong. The ring is bounded because it
rides the product document; the cap is applied on load as well as on append, so
a hand-edited file cannot grow it.

The ring's cap is not an arbitrary round number: it is `kAlignRepeatMax`, the
longest repeatability sweep the page will run, so a whole sweep's rows are still
there to compute a capability figure from when it finishes.

### The repeatability sweep, and the capability figure

The band says what a good part may vary by. Whether the procedure can actually
hold it is a different question, and *Run alignment × N* on the Frame check step
answers it: 3 to 100 runs (default 30), each one a **whole** alignment through
`fd.align_run` — travel, settle, exposure and detection together, exactly as a
recipe runs it. There is deliberately no shortcut that re-captures points
without moving; a sweep that skipped the move would report the detector's noise
and call it the machine's.

The loop lives in `config/scripts/alignment_repeat.lua`, a **system** script
(§7), and not in `App::Update`. That is a correctness point, not a style one:
one `orch_.lua.Start` covers the whole sweep, so `orch_.Busy()` stays true from
first run to last and Stop ends the lot. A UI-side loop that relaunched between
iterations would go momentarily idle between them — and every Run/Go surface in
the app gates on `Busy()`, so a second run could start in the gap. It is a
system script for a third reason beyond the two §7 gives: it is the source of a
capability number, and a forked copy that quietly changed what one iteration
does would still produce one.

A failed iteration **ends** the sweep. A Cpk computed over the runs that
happened to succeed is a lie by omission — the failures are the part of the
distribution that matters most.

The rows it leaves are ordinary history rows; nothing about the sweep is stored
separately. The pane computes, over the last N of them (N being the same
spinbox — a figure computed over a different span than the sweep answers a
question nobody asked):

```
Cp  = tol / (3 sigma)                    the spread alone, if it were centred
Cpk = (tol - |mean - ref|) / (3 sigma)   the spread AND where it sits
```

against **both** halves of the band, with the tolerance as the specification
limit — which is the whole reason the figure belongs here rather than in a
spreadsheet: the operator already decided what a good part may vary by when they
set the band. `sigma` is the **sample** deviation (n−1), unlike vision
reliability's population form: a capability index is an estimate about future
parts made from a sample.

Both are shown because the pair is diagnostic. High Cp with low Cpk is a tight
distribution sitting off the reference — a detector locked onto the wrong edge,
or a reference taken from a different part — and the pane says so in words when
it sees it. Cpk is coloured at the usual 1.33 (capable) and 1.00 (the band is
exactly three sigma wide) lines, because the number is useless to somebody who
does not know the convention. Zero measurable spread reports as `inf` rather
than an index in the millions: that is the arithmetic running out of digits, not
a perfect process.

The page shows all of it on the **Frame check** step, whose dot is the only red
one on the selector: green the last run passed the band, red it did not, orange
nothing has been checked. Editing a tolerance never rewrites a stored verdict —
where the two disagree the pane says which band was in force at the time.

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

### The middle panel is five steps, one at a time

The middle panel is a **header, a step selector, and a swappable pane**
(`DrawAlignDetailPanel`). The header holds the two facts that survive every
step — the procedure's name and the device its stations are poses on — and the
pane draws exactly *one* of five things: the E1 card, the E2 card, Run, the
frame check (§5c), or the nominals.

Stacked, as they were, they were far taller than the page. Run — the thing the
whole page exists to reach — sat below the fold, and the two station cards,
which differ only in a heading, had to be told apart by reading. The selector
answers that by *reporting* as well as choosing: E1, E2, Run and Frame check
each carry a status dot, and every label lines up because the row without one
spends the same width on a `Dummy`.

The dots answer different questions per row, which is the point of having them
where the choice is made. A station's says whether it is taught and how; **Run's
is about this SESSION** — green means a LIVE frame exists, the only state in
which anything may move against it, so the record left by last week's part is
deliberately not green; **Frame check's is about the last measurement** (§5c).

The dot has three states, and what makes one green is deliberately stricter than
"has a position":

| dot | station state |
|---|---|
| **green** | taught, `mode = auto`, and a detection model is taught behind it |
| **cyan** | taught, `mode = manual` — the point is marked by hand |
| **orange** | no station position **or** an auto station with no model (an NCC one with no patch counts as none) |

An auto station with no model has everything a run needs except the one thing it
will actually use; it fails at run time, on the machine, with a part loaded. A
green dot there would be a promise the station cannot keep, so it reads exactly
as orange as an untaught one — and the card beside it says which of the two it
is.

**The selector never gates Run.** Both stations taught is what enables the *Run
alignment* button inside the pane, not the button that opens the pane: a
selector that refuses to open is a selector that cannot explain itself, and the
reason (`assign a saved position to both stations first`) is written under the
button it disables.

`alignView_` is an index, not a uid — unlike every selection that names a
document object — because it addresses a fixed set of panes that no edit can
reorder (named `kAlignViewE1..kAlignViewCheck`, so the switch, the selector and
the resets cannot drift apart). It resets to **E1** whenever the selected procedure changes (New,
Duplicate, Delete, a click in the list, and a product load): a pane left on
*Run* while a different, untaught procedure is selected reads as a licence the
new one has not earned.

### Card layout

Each station card is a **two-column table**: *where* on the left (the private
position, its pose, exposure, lighting), *how* on the right (Manual crosshair /
Auto detect, and the detector's status block). They answer independent
questions, and the detector block — button, licence line, filter chain,
reliability figures, thumbnail, fallback checkbox — is by far the tallest thing
on the page. Stacked, it pushed the second station and the Run button off the
bottom of the panel. The columns are stretch-weighted rather than fixed, so both
cards line up at any panel width, and the divider is draggable.

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

### Two marks, not one — or four

A nominal is taught as the **centre of two or four marks**. The reason is the
same one that makes a centre hard to teach at all: the interesting point is
usually the *middle* of something, and a middle has no feature under it to put a
crosshair on. Marking corners puts the crosshair on something real every time and
leaves the centre to arithmetic.

Two or four is a real choice. Two opposite corners give the centre in one stroke
and **cannot check themselves**: a mark that lands off its feature moves the
centre by half its error and looks exactly like a good mark. Four give the same
centre as the average of a ring, and `spanMm` — twice the mean distance from the
centre — is then a *width across the marks* that either matches the
part or does not. `SolveAlignNominalN` takes 2 or 4 and refuses anything else: a
three-mark ring has no centre worth the name, and averaging one anyway would put
a nominal somewhere nobody chose. `points` on the record says which; files
written before four existed have no key and load as two. `SolveAlignNominal`
(two marks) stays, delegating, because most callers have exactly two.

The picker needed no second code path for it: the centre it draws is the average
of every *latched* mark and the envelope is their bounding box, which is the
midpoint and the diagonal box when there are two. Only the diagonal LINE is
pair-only — across four marks it joins two arbitrary corners of the ring.

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
only from the nominal list's *New nominal…* and a nominal's *Re-teach* — one
call, and the uid it carries (or does not) is the whole difference, which is why
the two buttons are no longer both called "teach" — on exactly the argument
`PageVisionTeach` makes: marking a feature needs a big live image, and the
Alignment page's columns are strips. It shares `CameraView` with the crosshair
pick window and the vision teach page, so a pixel means the same thing in all
three.

It can also draw a **registered tool's tip** (a combo of the devices registered
against this image's gantry — a tip in another origin, projected through this
pose, is off by the distance between the two) and a **resizable box centred on
it**. Drag a corner on the image to size it; it re-centres on the tip every
frame, so a drag can only change the size and there is no way to leave the box
describing a place the tool is not. Its size persists in `UiSettings` and
**nothing about it is stored on the nominal** — a nominal is a point on the part,
and how wide the gripper is says nothing about where that point belongs. What it
answers is the question an operator actually has while teaching one: *will the
tool clear what is beside it*, which no number on the panel answers. A tip that
cannot be predicted draws nothing and the panel prints the reason in full.

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

### The four steps, numbered on the buttons

A nominal's row is eight equal-looking `SmallButton`s and nothing about them
says which comes first — least of all that *3. Use* and *4. Place…* are not two
ways to do the same thing. So the four that **advance** carry their number:

| | Button | What it does |
|---|---|---|
| 1 | *Teach nominal…* | mark two features; their midpoint becomes the local X/Y |
| 2 | *Teach view* | save the current pose as where the camera watches this point from |
| 3 | *Use* | load it into the local→machine probe, so *Move here* can drive there and check it |
| 4 | *Place…* | mark a gripped object and compute the correction onto it |

*Go view*, *Clear view*, *Re-teach* and *Delete* stay **unnumbered**, behind a
24 px gap on their row. They check or undo a step rather than advancing one, and
a number on them would claim otherwise.

### What consumes one

The nominal list's *Use* button loads a nominal into the local→machine probe
fields directly above it, so *Move here* drives the **procedure's own device**
to it — still behind the live-frame gate and the arm checkbox.

### A nominal owns two camera poses, not one

`viewUid` (`algn_<procUid>_<nomUid>_v`) is where the camera stands to watch a
**gripped object** hovering over this point. `camUid` (`…_c`) is where it stands,
**focused, looking at the nominal itself** — the bare target on the substrate.

They are separate fields and must stay separate. The Z that focuses a part held
in the jaws does not focus the substrate under it: they differ by the height of
the grip, which is exactly the mistake `viewUid`'s tooltip already warns about.
Merging them would make that mistake unavoidable rather than merely easy.

The focus pose is worth storing for its **Z**. The local frame is 2-D:
`AlignLocalToMachine` hands back an XY and says nothing about height, so the
height that focuses this point on this substrate exists nowhere else once the
stage has moved. Writing it down while it is in focus is the only way to have it.

Teaching one is three steps on the nominal's row, in the order they are used:

| Button | What it does |
|---|---|
| *Centre camera* | local → machine through the **live** frame, XY only, then drives there. **Armed**, and gated on a live frame, exactly like *Move here*: a computed destination out of a stale frame goes somewhere nobody chose. Z is left where it is — the focus is the operator's to set next. |
| *Save cam focus* | stores the pose the device is at now, **X, Y and the focused Z**, as `algn_<procUid>_<nomUid>_c` (hidden, private to the nominal). |
| *Go focus* / *Clear focus* | a plain taught-pose move, and the delete. |

It follows every rule the object view follows, because it is the same kind of
thing: the name is the ownership record, deleting the nominal drops it
(`DropAlignNominalCamPos` beside `DropAlignNominalView`), duplicating the
procedure copies it, and the procedure-level prefix sweep catches it because
`IsOwnedAlignPos` matches on `algn_<procUid>_` rather than enumerating suffixes.

`nominal_view` is what drives there from a recipe, registered beside
`place_view` and sharing one body (`DriveToNominalPose`) so the arrival, the
settle and the failure text cannot drift apart. Two commands rather than a flag
on one, because a step has to *say* which pose it means: reading `place_view` as
"look at the nominal" is exactly the grip-height error above, wearing a name that
sounds right. Neither fails on an untaught pose unless `required` is set — a
procedure older than the field drives there some other way.

`nominal_view` also takes **`offset_x/y/z_mm`**, added to the taught pose in the
device's **own axis space** — the same thing `move_to_position_offset` does to a
saved position, and for the same reason: *that pose, but 0.5 mm higher* is a step
people write constantly, and the alternative is a second taught pose per
variation. Axis space and not global, because the pose was captured axis-by-axis
off the device and `gantry-main`'s transform inverts Z, so a number routed
through the global frame would descend where it meant to retreat; which way +Z
counts is a property of the machine, and the parameter help says to try a small
value first. An offset naming only axes the pose does not command **fails** —
offsetting an axis that was never taught is a move to a place nobody chose.

`place_view` deliberately takes none: a placement's view is the pose its
detection models were taught through, and a model matched at a different standoff
is matched against a picture it never saw.

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

### Showing the operator what they are aiming at

Marking is done through a ~3 mm field of view against nominals that can be
centimetres apart, so the surfaces draw three things over the image, all
projected from machine mm through the **live** pose so they stay on their
features as the stage moves:

* the **nominal** being aimed at (red), computed from the live frame every
  frame — it is what the marks are measured against;
* **marks already taken**, so the second point is placed relative to the first
  rather than from memory;
* the **pt1/pt2 step diagrams** in the footer row, the inactive one at 30 %.

The important case is the one that does not fit: a point outside the view
becomes an **arrow on the box edge, labelled with its distance**
(`DrawWorldMark`). Without it, "the target is 5.6 mm off to the left" and "the
target is where I am looking" are the same picture — which is exactly how a run
measures one feature and aims at another, and only finds out when `max_mm`
refuses the correction.

The run-time path carries these as `PickRequest::hints`: the *worker* computes
them, because only it has the live frame and the run context, and the UI only
projects. The pages compute their own from the same numbers.

Once an arrow says the target is 6 mm away, the operator needs to *get* there,
so a **right click drives the gantry** to put the clicked point under the centre
— on every marking surface, the pick window included. Two consequences worth
stating:

* it is behind the shared **arming flag**, because it is click-to-move by
  another name, and marking is otherwise safe with jog disarmed. There is no
  `orch_.Busy()` gate: the pick window exists only *during* a run, and being
  able to bring a feature into view is the whole point of asking a human —
  `manual_adjust` jogs mid-run on the same argument;
* the **crosshair jumps to the centre with it**. Leaving it on its old pixel
  would be wrong in a way nothing on screen shows: after the stage moves that
  pixel looks at a different physical place, and OK would capture a point off by
  the whole move. `DrawMarkPicker` reports the request rather than performing it
  — it has no device access — and the page issues it through layer 2.

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

### Running a placement from a recipe

The page is the bench surface; a production run needs the same thing as a step.
That is four commands and a script, mirroring the alignment family exactly —
`place_mark` (one point, one prompt), `place_apply` (the arithmetic, no motion),
`place_move` (apply + one relative move), and `place_run` over
`placement_procedure.lua`.

**The script exists because the procedure is a LOOP.** One correction lands
within the accuracy of the marking and the transform; re-measuring is what turns
that into a number instead of a hope, and the second pass usually removes what
the first one left. So:

```lua
for attempt = 1, TRIES do
    fd.place_mark{ alignment = ALIGN, index = 1, ctx_prefix = PREFIX }
    fd.place_mark{ alignment = ALIGN, index = 2, ctx_prefix = PREFIX }
    fd.place_apply{ alignment = ALIGN, nominal = NOM, ctx_prefix = PREFIX }
    if fd.ctx[PREFIX .. "_dist"] <= TOL then placed = true break end
    fd.place_move{ alignment = ALIGN, nominal = NOM, device = DEV, max_mm = MAXMM }
end
if not placed then error("...") end   -- a silent 0.3 mm miss is worse than none
```

A loop with an operator inside it is exactly what belongs in an editable file
rather than compiled into a step — the same argument `align_run` makes.

**It teaches the frame first, by default** (`align_first`). A nominal is stored
in local coordinates, so without a live frame there is nothing to convert it
through — and the live frame is cleared at every program start, DUT open and
product load (§5b). A placement that merely *assumed* one would therefore fail
on its first step about half the times anybody used it. Turning it off is for
the case it was reserved for: a program that has already aligned this part and
is placing several objects on it, because the frame belongs to the part, not to
each placement.

**Two surfaces launch it, one code path.** A `place_run` step is the production
route; the Placement page's *Run guided placement* button is the bench one, and
it injects the same keys through `LuaEngine::Env::initialCtx` exactly as the
Alignment page's Run button does. Running the file bare from the Scripts page
fails on its first parameter, which is correct and worth keeping: nothing on
that page says which object is going onto what. The button exists because
without it the script would be reachable only by first authoring a recipe.

The guided run is armed like the manual button — a scripted correction is still
a computed destination. That arming checkbox is duplicated into the guided
block, because the manual one sits below the correction readout, which does not
render until both marks exist; without a copy the run could never be armed.

### Making it unattended

The two prompts are the only human left in the loop, and they go the same way an
`auto` alignment station's did: a **taught detection model per mark**, private to
the nominal and named `vdet_<procUid>_<nomUid>_p<N>`. `Workflow::Detect` fills
the same `PickResult` the crosshair fills, so the midpoint, the correction, the
move and the verify loop are byte-identical whether a detector or an operator
produced the point.

Each mark carries its own **mode**, `AlignNominal::mode1/mode2`, exactly as a
station carries `AlignStation::mode`: *manual* is the crosshair pick, *detect*
runs the model. The mode is a field rather than "does `detN` name something",
because the first version read it that way and teaching a model was therefore
irreversible — there was no way back to manual short of deleting a patch that
cost real time at the microscope. Manual now leaves the model on the shelf, and
`place_mark` branches on the mode, not on the uid. A row written before the
field existed loads as `auto` when it names a detector, which is what it did.

They are taught on the **same page the station detectors use** — the Placement
page's *Set up detection…*, under the point set to *detect*, opens
`PageVisionTeach` with the nominal as the owner. One difference was needed: that
page used to `Close` to Alignment unconditionally, so it now carries
`visionReturnPage_` and goes back to whoever opened it.

Ownership is per **nominal**, not per procedure, on the rule the whole vision
layer already follows: an ROI, a threshold and a taught patch only mean anything
at the pose and exposure they were tuned at, and each nominal is viewed from its
own place. A nominal's delete drops its models (`DropNominalDetectors`), a
procedure's delete drops every nominal's, and a duplicate copies them with their
patch files — `CopyVisionModelAs` was factored out of the station copy so the
`.pgm` handling exists once.

**The fallback defaults ON here**, unlike a station's. A station teaches a frame
that everything downstream depends on, so a silent human substitution there
would corrupt the record; a placement is a per-part operation an operator is
usually standing over, where asking beats failing the run. It is never *silent*
either way: a fallback logs a warning naming the model and the reason, and each
captured point logs `(manual)` or `(auto)`, so nobody can believe a run was
automatic when it was not. Untick it on a nominal that must never quietly become
hand-placed.

Four things the commands do that the page's habits made obvious:

* **The prompt is the existing pick window.** `PickRequest` grew a `label`, so
  the same window and the same `AwaitPick` handshake say "point 1 of 2 on the
  object" instead of "mark E1". One window, two vocabularies.
* **No command in the family takes a viewing position as a parameter.**
  `place_run` briefly had one; it was removed on discovering what it would
  actually be — a raw position **uid** typed into a text box, because
  `ExecutionContext::FindPosition` resolves by uid only and the editor's picker
  is scoped to the step's `device`, which here is the *holder*, not the camera.
  The camera does get driven now, but by `place_view`, off a pose the **nominal
  owns** (below) — which is the same thing `align_station` does and needs no
  parameter at all. `place_mark` still never moves anything, so the two marks
  may still come from two different views.
* **`max_mm` is a hard failure, not a clamp.** The page shows its ceiling to a
  human who can judge it; a recipe has nobody watching, and a correction bigger
  than the ceiling is a broken input rather than a big correction.
* **`place_move` erases the marks it consumed** (`ExecutionContext::CtxErase`),
  so the next `place_apply` must be given fresh ones. Without that, a second
  pass would re-derive and re-apply a correction that has already been made —
  the ctx equivalent of the page's cleared marks.

### The nominal's camera view

A nominal says where the object must **end up**. It says nothing about where the
camera has to stand to **watch the gripped object get there** — and those are never the same
pose: the object is gripped somewhere else, and the height that focuses on a
part held above the substrate is not the height that focuses on the substrate.
Until this existed, every placement paid for that gap twice: an operator jogged
the gantry by hand before pressing *Run guided placement*, and a recipe carried a
`move_to_position` step in front of `place_run` that had to be re-taught whenever
the view changed.

It is a view of the **object**, and teaching it means gripping a real one and
bringing it to where it will hover before the camera is jogged — the Z that
focuses the substrate does not focus a part held above it by the gripper
standoff plus its own thickness. Teach it against an empty gripper and every
`place_mark` comes up blurred, and so does any detection model cut from that
frame, which turns an `auto` station into a fallback manual pick that looks like
a licensing problem and is not. The XY only has to FRAME the object: both marks
inside the picture, not the nominal's own coordinates.

So the nominal owns one: `AlignNominal::viewUid`, a saved position on the
**procedure's** device, named `algn_<procUid>_<nomUid>_v` and `hidden` — the
same private-ownership rule the station positions follow, one level down.
`CapturePrivatePose` is the single writer of such a row (factored out of
`TeachAlignStation` when this became its second caller), so an update-in-place
keeps the uid and a re-teach cannot orphan a reference.

`place_view` drives there, and it is the reason a viewing move was finally
allowed into this family: it takes no position, only the alignment and the
nominal, so it cannot be pointed at an unrelated pose. The script calls it once,
**after** `align_run` (which parks the camera at station 2, so an earlier view
would be thrown away) and **outside** the measure/correct loop — nothing moves
the camera between passes, and re-driving would only add settle time to a
re-measure.

An untaught view is **not** an error. `place_view` logs that it is leaving the
camera alone and succeeds, so every procedure written before the field keeps the
old contract: get the object in view with a `move_to_position` step above the
run. `required = true` flips that for a recipe that wants the omission caught.

The guided page gets there the same way: `OpenAlignPlace` drives the camera to
the nominal's view before it switches. The first thing that page asks for is a
mark on the gripped object, and a mark can only be made through the camera —
opening it from wherever the gantry happened to be left is opening it looking at
the wrong place, and the operator's first move was always to press *Go to view*
anyway. Same no-op rule as `place_view`: no view taught, or the device down, and
nothing moves. It sits **after** the `CanAccess` gate, so a refused switch never
leaves a machine that moved for a page nobody reached.

Lifecycle is the rest of the pattern, and all three parts are needed: deleting a
nominal drops its view (`DropAlignNominalView`), deleting a procedure drops
every one it owns (`DropAlignStationPositions` now sweeps by the
`algn_<procUid>_` prefix rather than enumerating the two stations, so a nominal
deleted earlier cannot have left a hidden row behind), and duplicating copies
them under the new owner.

`CaptureWorldPoint` is shared with `align_station`: pose-after-pick, then
`w = pose + offset`. One copy, because a second one that read the pose in a
different order would be wrong in a way nothing would show except a part placed
20 µm out.

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

The **Local → machine** block under *Run alignment* is the interactive twin of
the last two: type a local X/Y, read the machine XY, and **Move here** drives to
it (XY only — the frame is 2-D, so Z is left where it is rather than invented).
That button sits behind the shared `jogArmed_` flag, unlike the station cards'
`Go`: `Go` drives to a pose a human taught, while this destination is
*computed*, so a mis-taught frame sends the stage somewhere unexpected — the
same risk class as click-to-move.

The nominal list is a **list on the left, one nominal's controls on the right**. It began as one block of six buttons per nominal, which read
tolerably with one point taught and not at all with four: twenty-four buttons,
and the numbered sequence — the only thing on the page that says what to do
first — repeated four times over. Now the sequence appears once, and the list
answers the question no arrangement of buttons could: which point the next press
acts on. Selection is held as `alignNomSel_`, a **uid** and not an index, for the
reason indexes are refused everywhere else in this codebase — the vector is
appended to and erased from while the pane is open. A stale or empty selection
falls back to the first nominal rather than leaving a pane of controls with no
subject, and Delete is **deferred** past `EndChild`: erasing from the vector the
pane is reading invalidates the pointer mid-draw. A row carries one fact beyond
its name, `(no view)`, because that is the one that changes what step 4 does.

That block and the nominals live in the **middle** panel — never in the
right-hand frame panel where they started. The split between them is by what a
thing *is*: the right column reports the frame, a record of what was taught; the
middle carries what you do with it. `DrawAlignFrameActions` therefore re-resolves
the frame itself rather than taking one as a parameter — the live frame is the
only one that may move the stage, and a caller handing in `a.result` would be
exactly the bug §5b exists to prevent.

| `align_run` | typed one-step wrapper over `alignment_procedure.lua` |
| `place_view` | drive the camera to a nominal's taught viewing pose (no-op when it has none, unless `required`) |
| `place_mark` | ask the operator to mark one point on a gripped object (index 1 or 2) |
| `place_apply` | marks + nominal → the correction, into ctx. **No motion** |
| `place_move` | `place_apply` + one relative move on the holding device, `max_mm` guarded |
| `place_run` | typed one-step wrapper over `placement_procedure.lua` |

`align_run` is the **`xcan` pattern**: a real parameter form whose values become
ctx keys, then `RunLuaFile`. The script *is* the procedure — editable, and
re-runnable from the Scripts page, from a recipe step, from Lua
(`fd.align_run{...}`, since every registered command gets an `fd.` binding), and
from the Alignment page's Run button, all through the same code path.

**Which file it runs**: `alignment_procedure.lua`, `alignment_repeat.lua` and
`placement_procedure.lua` are **standard procedures** — `IsSystemScript` names them, and both
`ExecutionContext::ResolveScript` and the two page buttons resolve them to the
shipped `config/scripts/` copy, always. A file of that name in a product's
scripts folder is ignored and logged, and the Scripts page says so on the copy
being read.

The reason is `place_view`. These two scripts are edited in step with the
commands they call, so a product that forked one goes on running the version it
forked: the day the placement procedure grew its camera-view step, every forked
product would have kept skipping it, and the symptom — a camera that simply does
not move — points at the machine, not at a stale file nobody remembers copying.
"The placement procedure" has to mean one file on every machine.

They briefly worked the other way (product copy wins, the rule the page's
scanned-list launcher already used). What that solved — one surface running a
different file from another — is solved better by there being only one file.

Product-specific scripts keep the override, and `run_script`, `xcan` and
`raster_scan` still take their `file` literally: a scan's parameters are a
product's business, and re-pointing an existing product that carries its own
`hexapod_align.lua` would break it for no gain.

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
   nominal list's *Use*; no recipe step reads one.
7. **An unattended placement needs a model taught per nominal per mark** — two
   teach sessions before a nominal can place anything without a human, and they
   only hold at the pose and exposure they were tuned at. Nothing shares a model
   between nominals, by design; if the same object is genuinely viewed the same
   way at several of them, that is real duplicated work.
8. **A nominal is referenced by plain text.** `place_apply`/`place_move` take
   `nominal` as a `String`, so the recipe editor shows a text field and
   `CheckReferences` cannot validate it — unlike `alignment`, which is an
   `AlignUid`. A rename inside a procedure therefore breaks a recipe silently
   until it runs.
9. **Placement corrects translation only.** A nominal is a point, so the
   correction is a 2-D shift: if the gripper picked the object up rotated,
   neither the page nor the script can see it and the shift will not fix it. The information is
   *available* — the object's two marks have a direction, and storing the
   nominal's mark direction alongside its midpoint would give a Δθ to command on
   the holder's W — but a nominal deliberately stores a point today, so that is
   a schema change, not a calculation.

Related: `docs/state-and-config.md` (the collection and its fallback rules),
`docs/command-layer.md` (registry → editor → Lua bindings),
`docs/vision-layer.md` (why nothing here needs HALCON),
`docs/ui-file-layout.md` (where the page and the window live).
