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

**`t` is measured, by imprint** — §4.

## 3. What is stored

`core/DeviceReg.h`, one record **per device**, machine-level, in the
runtime-writable pair `config/device_reg.local.json` → `config/device_reg.json`.

| Field | Why |
|---|---|
| `R`, `rotSource` | the rotation and where it came from; `"axis-transforms"` today |
| `t` | the tip at device pose (0,0,0) |
| `imprintPose`, `homePose`, `tipView` | **the three teachable poses**, §5 |
| `tipMeas`, `tipSpreadMm`, `tipPoints` | the measurement behind `t`, and its width |
| `taughtRot`, `hasRot` | the rotations `t` is pinned to; refused past 0.05° |
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

## 4. Teaching the tip: the imprint

Touch the tool onto a marking coupon, then measure the mark it leaves. The mark's
centre is the tip in XY; the **focused reference Z over the coupon** is the tip in
Z, because the tool was resting on the surface now in focus.

Two reasons this beats picking the tool itself in mid-air, which is what an
earlier version of this design did:

1. **An imprint is the contact point by definition.** A curved tool viewed from
   above shows you its *silhouette*, which is not where it touches, and "which bit
   of the tip is the tip" is otherwise a matter of opinion.
2. **It is a flat mark on a flat surface** — the best case there is for both the
   focus metric and the crosshair, where a shiny curved tool is close to the worst.

**There is no `dz` here, unlike the needle.** The needle needs one because camera
and needle share a device and there is no other way to relate their heights. Here
the reference Z of a point is *already defined* as the gantry Z that focuses on
it, so the focused view hands the tip's Z straight back. It goes into the same
measured point as X and Y — no extra field, no second concept.

### The width is recorded, and it is a refusal

`tipSpreadMm` is **twice the mean distance from the centre** — the plain
separation for two diametrically opposite picks, a diameter for four, so one
number means the same thing however many were taken. It exists because the
average of N corner picks is the centre only if they are placed symmetrically,
and a pick that lands off the mark **shifts the centre by about half its error
while looking exactly like a good one**. The measured width is the only thing that
can see it.

With the tool's real width entered, a disagreement is a **refusal before anything
is stored** — not a warning, because there is nothing about a quietly shifted
centre for a warning to catch later. With it left at 0 the page says, in orange,
that nothing checked.

The centre and the width are computed on the page, where the picks are. A second
copy in C++ would be a second opinion about where the tool touches.

## 5. Why the teach is a page and not a procedure

Every step needs an operator at the machine: placing a coupon, jogging a tool into
contact by eye, driving the camera until the mark is in frame, and marking its
corners. There is nothing for a recipe to automate. A prompt-driven script would
be this page with its buttons hidden inside modal windows — and a second way to
create a registration is a second answer to where the tool is.

What a recipe gets is the **consuming** end: `device_reg`, `move_to_point`,
`device_reg_check_run`.

The four steps:

| | Step | What it captures |
|---|---|---|
| 1 | **Imprint pose** | the device pose with the tool marking the coupon — plus `homePose`, somewhere clear, so the tool gets out of the camera's way |
| 2 | **Camera view** | the reference pose over the mark, **focused**; its Z is the tip's height |
| 3 | **Corners** | 2–4 marks on the live image, in machine mm, through `DrawMarkPicker` |
| 4 | **Complete** | centre + focused Z → `t`, via `DeviceRegSetRotation` (once) and `DeviceRegSetTip` |

Three things about it that are load-bearing:

* **The three poses live in the record, not in saved positions.** Saved positions
  belong to the product document; a registration is machine state, and one that
  depended on whichever product happened to be loaded would break when none was.
  Same argument that keeps the employee roster out of `config/`. Storing them is
  what makes a re-teach after a tool change a repeat of the same geometry rather
  than a fresh hunt for the coupon and the mark.

  Each pose can nonetheless be **sourced** from a position the product already
  has taught — a machine has a home for each device long before anybody registers
  one, and standing the machine there again to make a second one is busywork.
  The picker resolves the position to **coordinates on the spot** and stores
  those; the record never holds a position uid, so the convenience costs nothing
  in ownership. Hidden positions are withheld from the picker (they are the
  alignment system's private `algn_…` poses), and a position that does not define
  all of X, Y and Z is refused rather than read as zero on the missing axes —
  which on a hexapod is a real place, and not the one anybody meant.

  One caveat the page states next to the view: a taught position carries whatever
  Z it was saved at, and the camera view's Z **is** the tip's height measurement.
  Adopting one is a fine way to get there; focusing and re-capturing is what turns
  it into a number.
* **`StoreDeviceRegPose` is the one writer.** Capture and adopt both go through
  it, so the two cannot drift apart, and the rotation latch — which only the
  imprint slot wants — is stated once. Those rotations are read **live** from the
  device whichever way the pose arrived: naming a pose turns nothing, and nothing
  on this machine commands U/V/W, so the device's current rotations are what the
  tool will be at when it makes the mark.
* **Marks are latched with the pose they were taken at** (`wx = gx + offset`), so
  the operator may jog between corners and a mark still means the same place —
  the same rule the nominal teach page follows.
* **The "go to" buttons are gated on `jogArmed_`.** Driving to a pose the
  operator taught is still driving, and layer 2 issues it
  (`Workflow::MoveToPosition`).

## 6. The check

`device_reg_check.lua` (STANDARD) drives the device to `imprintPose + (dx,dy,dz)`,
predicts the tip from the record, drives the camera there, and measures the miss.

**The check pose must not be the pose the tip was taught at.** `t` is derived from
the imprint measured there, so predicting there returns the measurement by
construction: the residual is about whether a pick repeats and says nothing about
the rotation. Hence the offsets, defaulting to (5, 5, 0) mm, and hence `verified`
stays false when all three are zero however small the number comes out. The page
says so beside the fields; the script says so in the log.

This matters more here than it would with a measured rotation. **The check is the
only thing on the machine that ever measures what the typed permutation costs.**

Reading the residual is the point:

* a **constant** shift, the same wherever you check → the **tool** moved. Re-teach
  the tip: one imprint, keeping the rotation.
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
| gripper changed / tool replaced | `t` | **one imprint** — steps 1–4, keeping `R` |
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
projected through the gantry pose exactly as the E marks are, with an edge arrow
and a distance when the tip is outside the frame and `dZ` against the plane in
focus — because two tips 3 mm apart in Z land on the same pixel. It is a readout
and nothing acts on it, but it inherits both of the record's refusals verbatim: a
tip that cannot be predicted (no registration, not connected, rotated away from
where it was taught) draws **no position**, and a tip registered against a
different reference is not drawn at all, since a point in another origin
projected through this pose is off by the whole distance between the two.

What a refusal *does* draw is one dim `no tip: <reason>` line in the corner.
Absence stays the signal — nothing invents a position — but silent absence answers
no question: the first operator to look for a gripper that was 1.4° off its
taught rotation could not tell a refusal from a broken overlay, and the log says
it once per session. The line is the reason string with its explanatory tail
elided, not a second wording of it. The colour and the word
`reg` on the plate exist to stop it reading as a detection: one is what the camera
saw, the other is what the machine believes.

### Trying it from the page

The Registration page's **Send tool here** drives the registered device so its tip
lands where the reference device is standing *now* — the point under the camera —
through the same solve `move_to_point` performs, with the same three refusals in
the same order: no registration, unverified (unless *allow unverified* is ticked,
which logs the move as such), rotations outside `kDeviceRegRotTolDeg`. It adds a
travel ceiling, because a stale record turns a short approach into a lunge, and a
signed **Z offset** in the reference device's *axis* Z — signed and typed rather
than derived, because which way that counts is a property of the machine and 0
sends the tip to the focused surface itself, which is a touch. On this machine
**negative lifts the tool away from the focused surface** (measured 2026-09-01 on
`gantry-main`, whose transform inverts Z); the tooltip says so and dates it,
because it is a measurement and not a rule — re-check it after any edit to the
transforms file.

It exists because a registration that has never been driven through has never been
*seen* to work; reading a residual off a log is not the same as watching the tool
arrive. It is still an APPROACH, and the button says so — nothing here closes a
loop.

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
| `device_reg`, `device_reg_check_save`, `move_to_point`, `move_tool_to_camera`, `tool_travel_to_camera`, `device_reg_check_run` | `src/command/CommandsDeviceReg.cpp` (`PrepareRegMove` + `ApproachRefPoint` are the one shared refusal/approach pair) |
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
