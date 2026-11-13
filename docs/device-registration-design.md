# Device registration: where hex-left and hex-right actually are

*Relating a hexapod's axis space to the gantry camera's, so a gripper tip has a
known position at every moment instead of only after it has been measured.
Companion to `docs/needle-dispense-design.md` (the same idea inside one device)
and `docs/alignment-procedure-design.md` (the same idea about a part).*

**Status: built.** Core `src/core/DeviceReg.{h,cpp}`, commands
`src/command/CommandsDeviceReg.cpp`, the check procedure
`config/scripts/device_reg_check.lua` (STANDARD), the teach and the page
`src/ui/pages/PageDeviceReg.cpp` (View > Registration, F8), record
`config/device_reg.local.json`.

---

## 1. The problem

`placement_procedure.lua` puts an object held by a hexapod onto a nominal by
measuring it, correcting by a **relative** move, and measuring again. That works,
and it will keep working — but it only knows where the gripper is *immediately
after looking at it*. Between placements the tip's position is unknown, which
costs three things:

* the first correction is a guess, so the loop usually needs a second pass;
* nothing on screen can say where a gripper is, and nothing can check that two
  of them are not about to occupy the same space;
* a point taught through the camera cannot be handed to a hexapod at all. There
  is no way to say "put the gripper *there*", only "move it a bit that way".

The needle solved the identical problem for the dispense head by measuring one
lever arm once. This is that idea across a device boundary — where it needs a
rotation as well as an offset.

## 2. The two halves, and where each comes from

```
p_ref = R * h + t          predict: where the tip is, from the pose
h     = R^T * (p_ref - t)  command: what pose puts the tip there
```

`h` is the registered device's raw axis XYZ; `p_ref` is the reference device's —
the gantry the camera rides on. Both stay in **axis space**, the same choice
`NeedleCal` makes and for the same reason: `CaptureWorldPoint` already reports
`wx = gx + pick.dxMM` there, `read_position` reads it, `move_axis_to` commands
it, and `gantry-main`'s configured transform inverts Z, so a number that took a
detour through `AxisTransform` would descend where it meant to retreat.

**`R` is taken from `config/motion_transforms.json`**, composed as
`M_ref * M_dev^T` — the permutation implied by the two devices' entries.

```
hex-left    M = [[0,0,1],[0,1,0],[1,0,0]]      global X -> hex Z,  Y -> Y,  Z -> X
hex-right   M = [[0,0,1],[0,-1,0],[-1,0,0]]
gantry-main M = [[1,0,0],[0,1,0],[0,0,-1]]
```

These are hand-typed ±1 matrices: somebody worked out which gantry direction each
hexapod axis points along and wrote it down. That is **exact for a device bolted
square and approximate for one that is not**, and the error is the awkward kind —
it vanishes at the point the tip was taught and grows with distance from it, so
it reads as drift rather than as an untested assumption.

That is a fair deal for an *approach*, and it is why `rotSource` is on the record:
nothing measured this, every consumer must be able to tell, and the **check** (§6)
is the one thing on the machine that ever puts a number on what it costs.

Two properties of that file it would be easy to get wrong:

* **It has no offset.** `AxisTransform` is `m` and `r` and nothing else, and
  `GlobalDeltaToAxes` multiplies a *vector*. That is the whole reason for the
  standing rule that only a delta may cross between devices. `t` is the missing
  translation.
* **Every one of those matrices is improper** — `det = -1` for all three, because
  controller axis-sign conventions are not consistently handed. `DeviceRegSetRotation`
  orthonormalises while *preserving* the handedness it was given, and `R^T` is a
  true inverse either way. Forcing a proper rotation would invert one axis of
  every answer.

**`t` is measured, wherever the device stands** — §4.

## 3. What is stored

`core/DeviceReg.h`, one record **per device**, machine-level, in the
runtime-writable pair `config/device_reg.local.json` → `config/device_reg.json`.

| Field | Why |
|---|---|
| `R`, `rotSource` | the rotation and where it came from; `"axis-transforms"` today |
| `t` | the tip at device pose (0,0,0) |
| `imprintPose`, `homePose`, `tipView` | the pose the tip was taught at (read, never set), an old imprint-teach pose, and where the camera looked from — §5 |
| `tipMeas`, `tipPoints` | the measurement behind `t` |
| `toolWmm`, `toolLmm` | the jaws' footprint, typed, drawn around the tip and never acted on |
| `taughtRot`, `hasRot` | the rotations `t` is pinned to (whatever U/V/W the device stood at); refused past 0.05° unless `pivotAtTip` |
| `checkResidualMm`, `checkPose`, `checkedAt` | the last check |
| `regAt`, `tipAt` | **two dates, not one** — §7 |
| `valid`, `verified` | both halves present; and a check agreed |

Like `NeedleCal` and unlike a taught frame, **the persisted record is the licence
to move**: a frame describes one physical part that may have been swapped since,
this describes tooling that has not. It is not cleared at program start, DUT open
or product load, and it does not belong in the product document.

Ownership needs no new mechanism: `Workflow` holds the live records mutex-guarded
beside `Needle()`, writers hand a record back through `QueueDeviceReg`, and **only
`App::Update` writes the file**. The drain differs from the needle's in one way
worth knowing — the file holds the **whole fleet**, so `App::Update` updates the
live map first and writes it as a unit. Writing only the record that changed
would drop every other device's registration on the first save.

## 4. Teaching the tip: wherever the device stands

*(Changed twice on 2026-11-07. The tip used to be taught by IMPRINT — the tool
touched a coupon and the mark was measured. It was then taught at device ZERO,
behind a run that drove the device there; that was dropped the same day, for the
reason in point 2 below.)*

Leave the registered device where it is, put the camera over the tool and focus
on it, then mark the tool's corners on the live image. The centre of the marks is
the tip in XY; the **focused reference Z** is the tip in Z; the device's pose `h`
(X Y Z, and U V W) is **read** at the first mark. Then `t = tip − R·h`
(`DeviceRegSetTip`).

1. **The operator never moves the device and never has to know where it is.**
   The pose is half of the measurement, so the page reads it rather than asking
   anybody to set it. The teach moves nothing at all — there is no confirm,
   because there is nothing to confirm.
2. **Where it is taught buys no accuracy by itself.** A prediction at pose `q`
   is `tip + R·(q − h)`: the typed rotation's error is zero at `h` and grows with
   `|q − h|`, wherever `h` is. Teaching at zero only helped a device that works
   near zero (the argument that "no `R·h` term carries the error into `t`" was
   about the stored number, not about a single prediction). **The best place to
   teach is where the device works.**
3. **The rotations are whatever the device stood at.** `taughtRot` takes the live
   U/V/W at the first mark; with `pivotAtTip` off, predicting past
   `kDeviceRegRotTolDeg` from them is refused, exactly as before.

What the imprint bought, and what this gives up: an imprint is the contact point
by definition, where a pick on a curved tool viewed from above lands on its
*silhouette*. Marking the tool directly makes "which bit of the tip is the tip"
the operator's judgement again — the corners are placed symmetrically around
the tool's end as it appears in focus. The check (§6) is what measures the
consequence.

**There is no `dz` here, unlike the needle.** The needle needs one because camera
and needle share a device and there is no other way to relate their heights. Here
the reference Z of a point is *already defined* as the gantry Z that focuses on
it, so the focused view hands the tip's Z straight back. It goes into the same
measured point as X and Y — no extra field, no second concept.

### No width check — a typed footprint instead

An earlier version measured the mark's width — twice the mean distance of the
corner picks from their centre — and refused the teach when it disagreed with a
typed tool width, on the argument that a pick off the mark shifts the centre by
half its error while looking exactly like a good one. It was removed. The width
of an *imprint* is a fact about the coupon and the ink, not about the tool: a
gripper pressed into a soft coupon leaves a mark wider than its jaws, and a
light touch leaves one narrower, so the refusal fired on good picks and was
turned off at the first station. The corner picks are the operator's judgement
and the record stores how many were averaged, nothing more.

What the record holds instead is the tool's **footprint**: `toolWmm` along the
reference device's X and `toolLmm` along its Y, typed under *Registration* on
the page and part of the machine-state record like the tip it is centred on.
It is only ever **drawn** — a rectangle around the predicted tip, turned by the
device's live **U** so it follows the jaws, on the page's own image (where its
corners can be dragged to resize it, in the box's own frame), on the Gripped
Object page, and on the Aux view. Nothing refuses on it and nothing moves by it;
`0` draws nothing. The spin is screen-space, the same drawing convention the
picker's envelope uses; if a station's camera is mounted so that +U reads the
other way on screen, the sign lives in exactly two `spin` lambdas
(`Widgets.cpp`, `WndAux.cpp`). Its purpose is the question a cross cannot answer: *is the
gripper standing on the part?*

The centre is computed on the page, where the picks are. A second copy in C++
would be a second opinion about where the tool touches.

## 5. The teach: the page, one tab

All of it is on the page's *Capture tool points* tab — that is where the picks
exist on the image, and the centre is computed where they are. It is not a
recipe step and not a script: a second launcher is a second way to create a
registration.

| | Step | What it captures |
|---|---|---|
| 1 | **Camera on the tool** | the operator jogs the reference device over the tool and **focuses**; its Z is the tip's height |
| 2 | **Corners** | 2–4 marks on the live image, in machine mm, through `DrawMarkPicker`, each with the camera pose it was taken at (`gx`, `gy`, `gz`). The FIRST also latches the registered device's live pose (X Y Z U V W) and both device names |
| 3 | **Compute** | centre + mean focused Z → the tip at the latched pose → `t`, via `DeviceRegSetRotation` (once) and `DeviceRegSetTip`; `imprintPose` = the latched pose, `taughtRot` = its U/V/W, `tipView` = the first mark's camera pose with the mean Z |

Things about it that are load-bearing:

* **Compute refuses a device that has moved since the first mark** (0.01 mm,
  0.01°). The marks were taken against the tool where it stood and the tip is
  computed as if it still stands there. The tab warns as soon as it moves.
* **Compute refuses a set whose camera Z spread is over 0.05 mm.** One focus for
  the whole set: a small refocus between corners is the same surface and is
  averaged, a large one means a corner was marked on something else.
* **A mark set belongs to one device pair.** Capturing a corner after the device
  or reference combo changed starts a new set; Compute refuses a set latched for
  another pair.
* **The steps are green when the record answers them**: a tip computed by the
  teach predicts, at `imprintPose`, exactly `tipMeas`. Steps 1–2 also go green
  from marks taken this session.
* **The poses live in the record, not in saved positions.** Saved positions
  belong to the product document; a registration is machine state. The old
  "or taught..." pickers and the home pose are gone from the page with the
  imprint teach (`homePose` stays in the record and in `device_reg`'s ctx for
  old records).
* **Marks are latched with the pose they were taken at** (`wx = gx + offset`), so
  the operator may jog the camera between corners and a mark still means the same
  place — the same rule the nominal teach page follows.
* **Seeing the tool: an image toolbar and a kept camera view.** Above the image,
  an **ROI** square on the crosshair (off, 1/3, 1/2 or 2/3 of the frame's short
  side) and a **binary** threshold view inside it (slider, or *Auto* = Otsu over
  the ROI) — the shared live ROI filter, `ui/RoiFilter` (`docs/ui-file-layout.md`).
  Display only — plain C++ through `RgbToGray8`, drawn by `DrawMarkPicker` as an
  overlay (`MarkPicker::roi`); the marks and the click are still on the camera
  frame. Under step 3, the **camera view**: main_camera's
  exposure and the `main_camera_light` call-out, HARD-CODED to that camera and
  light group, applied on an edit or by *Apply saved view* (never on page open),
  and kept with the toolbar settings in UiSettings for the next teach.
* **Corners can be dragged after they are captured, and every mark has
  reference lines.** The picker draws a full-width and full-height line through
  each latched corner and through the live crosshair (`MarkPicker::markLines`),
  so corners line up against each other and against an edge. A press on a corner
  grabs it (`MarkPicker::markDrag`); the widget reports the new place and the
  page re-latches the corner against the pose it is seen through NOW, focused Z
  included — exactly a fresh capture. The device pose latched at the first corner
  stays: a drag adjusts a pick, it does not start a set. Both are opt-in on the
  shared widget; only this page turns them on.

## 6. The check

`device_reg_check.lua` (STANDARD) drives the device to `imprintPose + (dx,dy,dz)`,
predicts the tip from the record, drives the camera there, and measures the miss.

**The check pose must not be the pose the tip was taught at.** `t` is derived from
the tip measured there, so predicting there returns the measurement by
construction: the residual is about whether a pick repeats and says nothing about
the rotation. Hence the offsets, defaulting to (5, 5, 0) mm, and hence `verified`
stays false when all three are zero however small the number comes out. The page
says so beside the fields; the script says so in the log.

This matters more here than it would with a measured rotation. **The check is the
only thing on the machine that ever measures what the typed permutation costs.**

Reading the residual is the point:

* a **constant** shift, the same wherever you check → the **tool** moved. Re-teach
  the tip: one teach (camera on the tool, mark, compute), keeping the rotation.
* a residual that **grows with distance** from the taught pose → the **rotation**.
  The device is not square to the axes the transforms file claims. `t` cannot
  absorb an angle; correct the mount, or keep every approach short.

### Focus the check the way the tip was taught

`reg_autofocus` defaults to **false**, and that is not a convenience. The tip's Z
is "the gantry Z that focuses on it", and step 2 sets it with an operator focusing
by eye. Let the check set Z with a sweep instead and the residual reports the
disagreement between the two *methods* — measured at 0.15 mm on a flat coupon,
three times the default tolerance, reproducible to 2 um, and indistinguishable in
the log from a tool that moved. Focus the check the same way the teach was
focused, or focus both with the sweep; never one of each.

With the autofocus off, the operator sets Z by hand while the pick window is open,
and the pick window is what they set it against: `pick_point`'s **target**
(`target_x/y/z_mm`, `target_tol_mm`) draws a live residual — dX and dY from the
crosshair against the live pose, **dZ from the device's own Z** — recomputed every
frame, plus the live focus score so the number being maximised is on the same
window. A hint says where to look; a target says how far off you still are, and Z
is the half the picture cannot show. Without it a hand focus is blind and the
distance only exists in the log, after the run has already failed.

### Three states, not two

`verified` is a bool, but a check that **ran and disagreed** is not the same thing
as a check that was **never run**: one has put a number on what the rotation costs
and the other has measured nothing. The record already distinguishes them —
`checkedAt` is stamped on a failed check too — so the page and the `device_reg`
log line say *CHECK FAILED 0.1504 mm at 2.000 mm from the taught pose* rather than
"nothing has measured the tip". Telling an operator who just watched the check
drive, focus and pick that nothing measured anything reads as the software having
missed it.

## 7. Two dates, not one

`R` is a property of the **mount**; `t` is a property of the mount **and the
tool**. Change a gripper and the tip moves while the axis directions do not.

| Event | What is invalid | Cost |
|---|---|---|
| gripper changed / tool replaced | `t` | **one teach** — steps 1–4, keeping `R` |
| device moved, remounted, crashed | `R` and `t` | re-check the transforms entry, then re-teach |

`regAt` and `tipAt` are stamped separately so the page can say which half is old.
Teaching a tip clears `verified`: the tip moved, so whatever the last check
measured described tooling that may no longer be on the machine.

## 8. What consumes it — and what must not

**Consumes it:**

* a live tip position, published every frame per device by `Workflow::PublishTips`
  on `topics::TipAxis` (`tip/<device>/X|Y|Z`, `core/Topics.h` — never hardcoded).
  A tip that *cannot* be predicted publishes **nothing** rather than a stale
  value: absence is the signal, and a readout frozen at the last good number is
  how a rotated hexapod goes on looking placed. The reason is logged on the edge,
  not once per frame;
* a soft envelope check — two tips, or a tip and the gantry, converging;
* `move_to_point{device, x_mm, y_mm, z_mm}`, which commands a reference-space
  point to a hexapod through `R^T`. This is the thing that was impossible before.
  It refuses an unregistered device, an unverified one (unless
  `allow_unverified`), a point declared in a different reference space, a live
  rotation away from the taught one, and — via `max_mm`, default 10 mm — a move
  longer than an approach has any business being. Unattended, nobody notices that
  a stale record turned a short approach into a lunge.
  **Every registration move runs one precheck before anything moves.** It lives in
  `ApproachAxisPoint`, the shared helper where all of them end.
  - It logs the travel and the per-axis deltas from the live pose, then applies the
    `max_mm` ceiling.
  - It asks the CONTROLLER whether the target can be reached at all, with
    `IMotionController::CheckMove` (PI: `VMO?`, a "virtual move"). A hexapod's
    workspace depends on all six axes together: a pose can be inside every axis's
    range and still be unreachable, and only the controller knows.
  - **No** fails the step with "nothing moved", instead of a `PI_MOV` error
    mid-approach after the travel leg was already issued.
  - **A query that failed also fails.** A check that could not run is not a pass.
  - A controller with no such query (ACS, the sims) answers **Unknown**. That is
    logged and the move goes on, with the controller's own MOV check the only one.
  - Only the final pose is asked. The approach leg's intermediate point lies on the
    straight line to it.
* `move_tool_to_camera{device, z_offset_mm}` — the same approach aimed at the one
  point a recipe author cannot write down: wherever the camera is looking when
  the step runs. It reads the reference device's live pose, so the target comes
  from whatever left the stage there (an operator, an alignment, a detection),
  and it goes through the identical refusals and the identical ceiling because
  it ends in the identical helper. **`z_offset_mm` is an offset from the FOCUSED
  plane, not a height** — the same argument the needle's `dz` makes: the
  reference Z of what is under the camera is "the gantry Z that focuses on it",
  so 0 would send the tip into that surface. It defaults to −2 mm, and negative
  is the direction *away* from the surface on this machine (measured 2026-09-01;
  `gantry-main`'s transform inverts Z), stated and dated in the parameter help
  rather than asserted as a rule;
* `move_camera_to_tool{device, axes, z_offset_mm}` — the inverse, and the only
  command in the family that does not move a tool: the **camera** goes to look at
  one. `p = R*h + t` predicts the tip from the device's live pose, and the
  reference device's own X/Y is by definition the point its camera is looking at,
  so the inverse is a plain move rather than a solve. Two things make it worth
  having: it is how an operator *sees* what a registration costs — the tool
  itself, next to the overlay's cross that claims to be on it — and it is what a
  vision check of a gripper needed, since "photograph the tool" previously had no
  way to say where the tool was and fell back on a taught position per tool per
  station, which describes where it was *parked*. **Z defaults to being left
  alone** (`axes` = `XY`): the tip's reference Z is by construction the height
  that focuses on it, so commanding it is the useful thing *and* the leg that
  drives the lens toward a tool of unknown height, which must never be a side
  effect of asking to look at something. Its ceiling defaults wider (60 mm) for
  the same reason its risk is lower — it crosses the workspace to a parked tool
  rather than descending the last few mm onto a point — and it takes **no**
  `allow_unverified`, because it has no verified gate to opt out of: that gate
  asks whether an unchecked record may move a *tool*, and looking is how you find
  out what an unchecked record costs. `PrepareRegMove` therefore takes
  `requireVerified` explicitly from every caller rather than defaulting it, so
  that one exception is stated at the call site instead of hidden;
* `tool_travel_to_camera{device, lift_mm, via_position, z_offset_mm}` — retreat,
  cross, descend, as one step. It exists because of the "not in scope" bullet
  below: `move_tool_to_camera` is ONE coordinated move straight through whatever
  is in the way, and a path that lives only in a recipe author's head is the one
  the next recipe leaves out. Leg 1 is a relative move along **global** Z through
  the device's own transform (default 4 mm; positive is up on this machine, 0
  skips it); leg 2 is an optional taught waypoint; leg 3 is
  `move_tool_to_camera`. Each leg waits for arrival *and* settle before the next
  is issued — three moves, not one interpolated path, so a Stop between them
  leaves the tool somewhere defined. The registration is checked **before the
  first millimetre**: a lift-off followed by a refusal to descend parks the tool
  mid-air over a fault that was knowable up front.

**Must not consume it: the final placement.** A typed rotation and one measured
point get a tool close — nowhere near `place_tol_mm`'s 20 µm.
`placement_procedure.lua` keeps its measure/correct/re-measure loop exactly as it
is; the registration only means the loop lands on pass 1 instead of pass 2. A
`move_to_point` is an **approach**, and it says so in its name, its help, its log
line and on the page.

### Seeing it on the picture

The Aux window's **Grippers** toggle draws every registered tip on the live image,
projected through the gantry pose exactly as the E marks are, in the device's own
colour (`DeviceAccentColor`, the badge of its device picture: hex-left teal-green,
hex-right orange) and labelled with a short name from Preferences > Graphic view
(`aux_tip_labels`, default HL / HR, at most 10 letters, empty = the mark alone).
A tip outside the frame is an outlined triangle on the edge pointing at it, with
that label and no distance (`CameraMark::edgeDistance = false`; a nominal keeps
its "name N.NN mm" arrow). `dZ` against the plane in focus goes on the Z gauge —
because two tips 3 mm apart in Z land on the same pixel. The gauge names each
pointer by the same short label (HL / HR; the full name only when the label is
empty) and prints whole MICRONS (`+12990`, ticks 1k / 5k / 10k), which keeps the
plate narrow; the tape itself still scales in mm. It is a readout
and nothing acts on it, but it inherits both of the record's refusals verbatim: a
tip that cannot be predicted (no registration, not connected, rotated away from
where it was taught) draws **no position**, and a tip registered against a
different reference is not drawn at all, since a point in another origin
projected through this pose is off by the whole distance between the two.

The Z half is a **gauge** in the bottom-right corner, not a line of text: one
vertical tape with the camera's focused plane across its middle, the left
hexapod's pointer on the left of the tape and the right one's on the right
(by name; an unnamed device takes a free side, a third one keeps a text line).
The reading is the tip's **height above focus** — `dZ` times
`Workflow::GantryAxisZUpSign()`, global Z+ (the Jog window's Up; the global
frame is Z-up) pushed through the gantry's transform — on gantry-main
(`M33 = −1`, raw Z 0 at the top of travel) that is axis Z−. So up the tape is
up at the machine
without anything assuming which way gantry Z counts; when the sign cannot be
derived the tape shows the raw axis `dZ` and says `+Z` / `-Z` at its ends. The
scale is logarithmic (bends at 0.1 mm, ends at 10 mm): the last tenth of an
approach gets as much tape as the 5 mm before it, and a tip further off pins at
the end in red with its true number beside it. Within 0.05 mm it turns green
and reads **0.000**, pointer ON the focus line — a residual below what the
label means would only make the pointer twitch. The gauge is a **fixed size**
(a 200 px tape; each side reserves a label slot for its name and a worst-case
number), so nothing on it moves but the pointers. Like the cross, it is a
belief, and its title says `reg`.

What a refusal *does* draw is one dim `no tip: <reason>` line in the corner.
Absence stays the signal — nothing invents a position — but silent absence answers
no question: the first operator to look for a gripper that was 1.4° off its
taught rotation could not tell a refusal from a broken overlay, and the log says
it once per session. The line is the reason string with its explanatory tail
elided, not a second wording of it. The colour and the word
`reg` on the plate exist to stop it reading as a detection: one is what the camera
saw, the other is what the machine believes.

Around each cross is the tool's **footprint**, drawn to scale: a rectangle of
`toolWmm` × `toolLmm` in machine mm, so it grows and shrinks with the picture
under it instead of staying a screen-sized glyph. A cross answers *where is the
gripper* and never *is it standing on the part*, and at a 3 mm field of view
those are the same cross. The size drawn is the one **typed** on the Registration
page — the jaws as the operator knows them — so a registration with no
footprint draws no outline at all. It fades when the tip is off the focused
plane: both the position and the size go through a pixel->mm calibration that
belongs to *that* plane, and the cross never had a size to be wrong about.

### Trying it from the page

The page's *Verify and test* tab is two buttons, each the record read one way —
and on the page, nothing else: the check runs from a recipe step
(`device_reg_check_run`), so a record taught on the page stays UNVERIFIED until
one does.

**Move tool to camera** drives the registered device so its tip lands where the
reference device is looking *now*, through the same solve `move_to_point`
performs and the same rotation guard (`kDeviceRegRotTolDeg`, unless
`pivotAtTip`). There is no verified gate on the button — the operator watching
the tool arrive under the cross IS the check it would have asked for — and the
popup says when the record is unverified. A signed **Z offset** in the reference
device's *axis* Z stands the tip off: typed rather than derived, because which
way that counts is a property of the machine, and 0 sends the tip to the focused
surface itself — a touch, which the tab says in orange. On this machine
**negative lifts the tool away from the focused surface** (measured 2026-09-01 on
`gantry-main`, whose transform inverts Z); the tooltip says so and dates it,
because it is a measurement and not a rule — re-check it after any edit to the
transforms file.

**Move camera to tool** leaves the tool where it is and drives the *camera* until
its centre lands on the predicted tip — `move_camera_to_tool` from a button. The
tip comes from `Workflow::DeviceRegTip`, the same predictor the overlay and the tip
topics use, so there is no second prediction on this page. **Z is opt-in**
(*focus Z too*), for the reason the command's `axes` defaults to `XY`.

**A move longer than 3 mm (`App::kRegConfirmMm`) asks first**, in a popup that
holds the move exactly as it was computed at the click — device, targets and
travel — and *Move* issues that, so the operator approves the numbers they read.
It replaces the old per-button travel ceilings. The popup is raised only by a
click while nothing runs (both buttons gate on `orch_.Busy()`) and closes itself
if a run starts anyway: a modal must never sit over a prompt window a worker is
blocked on. Both buttons are behind the jog arm. They are still an APPROACH —
nothing here closes a loop.

## 9. Composing with a placement: nominal → gripper

The point of all this in production. A program aligns the part, then wants to put
a lens held by `hex-left` on nominal `n1`:

```
align_run{alignment}                     teaches the frame on THIS part
nominal_point{alignment, nominal = n1}   -> nom_x, nom_y   (machine XY)
   ... a Z from somewhere (below) ...
move_to_point{device = hex-left, x = nom_x, y = nom_y, z = ...}   the APPROACH
place_mark x2 -> place_apply -> place_move                        the PLACEMENT
```

The link that makes this work at all, and that is easy to assume rather than
check: **`AlignLocalToMachine`'s output and `move_to_point`'s input are the same
space.** A frame's origin comes from `AlignPoint.wx = gx + pick.dxMM`, gantry-main
axis XY; a registration's `p_ref` is gantry-main axis XY. No conversion sits
between them, and none should be added.

`nominal_point` (in `CommandsAlign.cpp`, beside `align_apply`) exists because
nothing else could answer "where is n1" without first marking the object —
`align_apply` takes two raw numbers and `SolvePlacement` needs the marks. Skipping
that first measurement is the whole value of a registered device, so something had
to name the target before anything had looked at the object. It goes through
`LiveProc`, so it refuses a saved frame exactly as `move_local` does.

**Three things this composition does not give you:**

* **A Z.** The frame is 2-D rigid and a nominal stores `lx`/`ly` only, so
  `nominal_point` reports no `_z` — an invented Z on a gripper approach is a lens
  driven into a substrate. Take it from a focused camera reading at the target
  plus a per-product standoff for lens thickness and bond line. Nothing stores
  that standoff yet; it is the same shape as the needle's `dz`.
* **The lens centre.** The imprint measured where the *tool* touches. A gripped
  lens's centre is somewhere else — off by how it seats in the jaws, plus grip
  repeatability. Either teach the tip with a reference part gripped (then `t` is
  per part type, which is what `tipAt` is for) or let the placement loop absorb
  it. Do not assume it.
* **A path**, for `move_to_point` and `move_tool_to_camera` alike: each is one
  coordinated move from wherever the gripper is to the target, straight through
  whatever is in the way. Retreat to a safe height, traverse, descend — the same
  discipline `dispense_procedure.lua` spells out, and for the same reason.
  `tool_travel_to_camera` packages exactly that for the camera point and nothing
  more: it is three ordinary moves in a fixed order, not a planner, and it knows
  nothing about what is actually on the table. The waypoint is still the
  author's to teach.

## 10. The rule this changed

CLAUDE.md used to say, under **Vision**:

> Across devices, only a DELTA transforms. […] an absolute machine XY must never
> be commanded to a hexapod.

The premise — that the two origins have never been related — is what this removes.
The rule was **rewritten, not deleted**: delete it and the next person places a
part open-loop through a record that was never meant to carry it.

## 10b. The pivot, and when the rotation guard is nothing but harm

`DeviceRegRotationOk` refuses to predict a tip more than `kDeviceRegRotTolDeg`
(0.05°) from the rotations it was taught at, because a tip that swings about a
pivot somewhere else describes no rotation but its own. That is right — *when the
pivot is somewhere else*.

A PI hexapod's pivot is a **controller setting**, and both grippers here have it
on the tool tip. Rotating U/V/W then turns the tool about the very point the
record stores and does not translate it, so `t` describes every rotation and the
guard protects nothing. What it does instead is blank a perfectly good tip: at
U = 0 against a tip taught at U = −1.4° the marker vanished from the Aux view,
every `move_to_point` refused, and the reason arrived as one log line an operator
reading a camera image never sees. Absence looked like a broken overlay.

So the assumption became a field — `DeviceReg::pivotAtTip`, persisted as
`pivot_at_tip`, with a checkbox on the Registration page — rather than either a
hard-coded refusal or a hard-coded bypass. **It defaults to true**, and files
written before the field existed load as true: they were written on this machine,
by this pivot, and the guard they predate was blanking their tips. Turn it off
for a device whose pivot is not on the tool, and the refusal returns exactly as
it was — the page then also prints the rotations the tip was taught at, because
that is the number you need to get back to.

One switch, read by `DeviceRegRotationOk`, so the overlay, the tip topics and
every move command agree about it by construction.

## 11. Sketch of the pieces

| Piece | Where |
|---|---|
| record, predict/solve, the two setters, the pivot switch, JSON, load/save | `src/core/DeviceReg.{h,cpp}` |
| live records, queue, seed columns, tip publish | `src/workflow/Workflow.{h,cpp}` |
| `device_reg`, `device_reg_check_save`, `move_to_point`, `move_tool_to_camera`, `move_camera_to_tool`, `tool_travel_to_camera`, `device_reg_check_run` | `src/command/CommandsDeviceReg.cpp` (`PrepareRegMove` is the one shared refusal; `ApproachAxisPoint` the one approach, whichever device is moving, with `ApproachRefPoint` the solve in front of it — all three declared in `command/RegApproach.h` so `grip_move_to_camera` can use them rather than copy them) |
| the four-step teach and the page | `src/ui/pages/PageDeviceReg.cpp` (F8) |
| the check (STANDARD) | `config/scripts/device_reg_check.lua` |
| `nominal_point` | `src/command/CommandsAlign.cpp` |
| the file write, draining the queue | `App::Update` |
| the empty shipped seed and its notes | `config/device_reg.json` |

Page access is **Admin** — this teaches the machine, which is the line
`MinRoleFor` already draws.

## 12. Deliberately not in scope

* **U/V/W compensation**, in the general case. See *The pivot* below for the
  case that is not general — and is the one this machine is in. Where the pivot
  is somewhere else, the record
  is pinned to the rotations it was taught at, stores them, and **refuses to
  predict** past `kDeviceRegRotTolDeg`. Silently predicting through a rotation the
  record never saw is the failure this avoids. If rotations are ever commanded,
  the record grows a tool vector from the pivot and prediction becomes
  `t + R*(h + Rot(u,v,w)*r_tool)` — the PI controller's own pivot setting is the
  natural source for `r_tool`.
* **Measuring `R`.** A jog-and-measure pass on three axes would give a rotation
  that carries the real mounting error, and an earlier version of this design had
  one. It was removed as more procedure than the accuracy justified for an
  approach: the check already bounds what the typed matrix costs, and if that
  number is ever too large the honest fix is the mount, not a fitted angle
  compensating for it. `rotSource` is the seam it would come back through.
* **Registering the hexapods to each other.** Both register to gantry-main, so
  hex-to-hex falls out by composition.
