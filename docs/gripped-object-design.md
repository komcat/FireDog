# Gripped objects: where the thing in the jaws actually sits

*The record is `core/GripProfile.h`; the page is `ui/pages/PageGrip.cpp`; the
commands are `command/CommandsGrip.cpp` and `config/scripts/grip_measure.lua`.*

## 1. The question

`docs/device-registration-design.md` answers **where is the tool tip**. That is a
fact about the machine: it changes when somebody changes the gripper, and not
otherwise.

This answers the question one step out. A gripper picks a lens off a tray, and
the lens lands in the jaws a little differently every time — a few tens of
microns of it, which is most of what a placement's correction loop spends its
first pass removing. Where the *object* is, is a fact about **this pick** and
about nothing else.

```
object   =  the centre measured through the camera      (reference axis mm)
tip      =  Workflow::DeviceRegTip(device)              (reference axis mm)
offset   =  object - tip                                <- what a run returns
```

Both terms are in the **reference device's axis space** — the gantry the camera
rides on — for the reason `NeedleCal` and `DeviceReg` both are: `CaptureWorldPoint`
reports `wx = gx + pick.dxMM` there, `move_axis_to` commands it there, and
`gantry-main`'s transform inverts Z, so a number that took a detour through
`AxisTransform` would come back describing a different direction.

The subtraction is what makes the answer useful: **the origin cancels**, so what
is left is a vector, and a vector is the only thing that crosses to a device
whose origin has never been related to the camera's. `grip_apply` publishes it
twice — once in the reference frame and once in the gripper's own axes, the form
a correction move takes — and the second is `DeviceRegSolve(centre) −
DeviceRegSolve(tip)` rather than a hand-rolled `R^T`, because `t` cancels between
them and there is then still exactly one place in the codebase that multiplies a
matrix.

## 2. Why it is a *profile*

Every field on the record except the last three is a parameter of **how to
measure**: where to hold the object, how to find it, how wide it should be, how
far off it may sit. Those describe an object *type* — "the 8 mm lens" — and they
are worth naming and re-using, which is what the recipe step references.

The measurement itself is per-pick and belongs in ctx, which is where a run
leaves it. `lastDx`/`lastDy` are kept only as **evidence**, the way
`DeviceReg` keeps `tipMeas`: when a grip lands somewhere surprising, "which
number is wrong" is the only useful question, and re-measuring to find out throws
the answer away.

### It is PRODUCT state, not machine state

The opposite of `DeviceReg`'s argument, and deliberately. A lens is part of a
product; its detection model is cut from a picture of that product's part; both
travel with the product folder. So a profile lives in `{uid}.grips.json` under
the product (manifest role `grips`, config fallback
`config/grips.local.json` → `grips.json`), and its private inspect pose lives in
the product's own positions — where a registration's three poses deliberately do
**not**, because those describe the machine and would vanish at the next product
load.

`NewProduct` carries the profiles over — how to measure is a parameter — and
clears every reference offset. A new product is a new object until somebody says
otherwise.

### Two dates

`taughtAt` stamps the reference offset; `measuredAt` stamps the last run. Change
the tool or the object and the reference is void while the parameters are not, so
the page can say which half is old. Same split, same reason, as `DeviceReg`'s
`regAt` and `tipAt`.

## 3. What it does not hold

* **The reference device.** It is `DeviceRegFor(device).refDevice`. The
  registration already names the space the offset lives in, and a second copy is
  a second thing that can disagree.
* **A `dz`.** The offset is XY, in the plane the camera measures. A Z from a
  focus sweep would describe the object's *height* — a different question asked
  by a different instrument — and folding it in would let a mis-focus read as a
  mis-grip.
* **A rotation guard.** The inspect pose carries the rotations (`CapturePrivatePose`
  stores every axis the device reports), so driving there restores the pose the
  reference was taught at. That is a stronger guarantee than a tolerance check,
  and it costs a move the run was making anyway. A profile with **no** inspect
  pose measures the tool where it finds it — legitimate, and both the page and
  the script say so out loud rather than refusing.

## 4. Auto and manual, and why there is no size check

The mode is its own field and is **not** inferred from a detector existing — the
rule `AlignStation` set. A taught patch costs real time at the microscope, and
choosing manual for one run must not destroy it.

| mode | marks | centre |
|---|---|---|
| `manual` | 2 | their midpoint |
| `auto` | 1 | the detection |

**There is no width.** An earlier draft carried a `sizeMm` with a tolerance, on
the argument `DeviceReg::tipSpreadMm` makes: a mark off the object shifts the
midpoint by half its error and the width is the only number that can see it.
It was dropped because the number it would be compared against is not real.
The two manual marks land on whatever edge the operator can *see* — a lens has
a bevel, a shadow and a reflection, and each of them is an edge — and a circle
model reports whatever contrast it was taught on. Both describe the picture,
not the part, so a check against the part's real width refuses good grips on
lighting and passes bad ones by luck. What the profile judges is the offset,
against a reference adopted from the same kind of picture, which is the one
comparison that cancels the picture out.

**No reference means no pass.** A profile whose offset has never been adopted
still reports its measurement; `ok` is false and `_why` says why. A default pass
is how an unchecked profile ends up trusted.

### A constant registration error cancels

Worth knowing, because it decides how much the registration's `verified` flag
matters here. The reference offset and every later measurement both subtract the
**same** predicted tip, at the **same** inspect pose, through the **same**
record. A registration that is off by a constant — which is exactly what a typed
rotation gives you near the pose its tip was taught at — shifts both by the same
amount and vanishes from the difference.

So an unverified registration costs far less on this page than it costs a move,
and the page says `UNVERIFIED` rather than refusing. What does **not** cancel is
a registration that changed between the teach and the run: re-teaching a tip
invalidates every reference offset measured against the old one, and nothing
here can detect that. It is the reason `tipAt` is on screen.

## 5. The commands

| Command | Does |
|---|---|
| `grip_profile` | the record into ctx, including which device carries the camera — a script must not guess that |
| `grip_mark` | one point: detected, or a crosshair pick, or a crosshair pick as a **logged** fallback from a failed detection. A near-twin of `place_mark`, deliberately: an operator who has learned one window has learned both |
| `grip_apply` | the arithmetic and the verdict. No motion. Queues the measurement back to the document as evidence |
| `grip_measure` | the composite a recipe uses: inspect pose → camera onto the tool → optional focus → marks → apply |
| `grip_move_to_camera` | the consuming move: `move_tool_to_camera` with the offset subtracted from the tip's target, so the **object** lands under the camera |

`grip_measure` runs `config/scripts/grip_measure.lua`, which is a **STANDARD**
script (`IsSystemScript`). That list is not a policy about machine state: *what*
this measures is per-product to its bones — the lens, the gripper and the
tolerance all live in the product's own profile — but *how* it measures is not.
A product that forked the file would keep whatever order of legs it was copied
at, and the failure would look like a bad grip rather than like a missing step.
The product's business is entirely in the record the script reads.

Two defaults are worth stating. `grip_apply`'s `require_ok` defaults **true**,
because an unattended run that measured a bad grip and carried on would place a
part with it; and `move_camera_to_tool`'s `axes` defaults to `XY`, because Z is
the leg that drives the lens toward a tool of unknown height.

### `grip_move_to_camera`

The step the measurement exists for. The camera is looking at a point — wherever
an operator, an alignment or a detection left the stage — and the gripper must
put what it is holding there:

```
tip target = camera − (dx, dy)        (reference axis mm)
```

Three things about it are deliberate.

* **The offset comes from ctx**, `<prefix>_dx/_dy`, which is what `grip_measure`
  or `grip_apply` left earlier in the *same* run. Not the profile's `last*`:
  that is an earlier run's number for a different pick. Not the engine's copy
  of the document either: it was taken at Start, before this pick existed. A
  run that has not measured this grip has no offset, and the step fails rather
  than moving on an assumed one.
* **Z assumes the object sits at the tip's height.** The offset is XY — it is
  measured in the plane the camera works in — so Z is exactly
  `move_tool_to_camera`'s: the camera's focused Z plus `z_offset_mm`, applied to
  the *tip*. An object hanging below the jaws is that much closer to the surface
  than the parameter says; a recipe that knows the height folds it in.
* **It is the registration's approach, not a copy of it.** `PrepareRegMove`
  and `ApproachRefPoint` are declared in `command/RegApproach.h` for exactly
  this caller, so the verified gate, the rotation guard, the travel ceiling and
  the two-speed approach are the same code `move_tool_to_camera` runs. A second
  statement of those refusals would drift off the first.

Still an approach, in the sense §7 gives the word: a typed rotation plus one
measured tip gets the object close, and a placement measures and corrects from
there.

## 6. The page

`Page::Grip`, F9, a full-page takeover, Admin by default like every teach page
(`MinRoleFor`'s `default:`).

The measuring is a recipe step, so the page is not where the work happens. It
exists for the two things a recipe cannot do:

* **Teaching.** Adopting a reference offset means an operator looking at a grip
  they believe is good and saying so. A run that promoted its own measurement to
  the standard would make every grip a good grip, and the number would drift a
  little further every shift with nothing to notice it. Hence
  `ProductSession::ApplyGripResult` writes `last*` and **never** `ref*`.
* **Seeing.** The image carries the two marks, their midpoint *and* the
  registered tip, projected through the same pose (`MarkPicker::toolHave`). Half
  a millimetre of grip error is a number in a log and a glaring picture here. The
  tip is drawn differently from a mark on purpose: a mark is a thing somebody
  measured, a tip is a thing the machine computed out of a registration.

Its **Measure** button launches the same shipped script the recipe step runs —
the rule the Alignment, Placement and Registration pages all follow — with one
difference it states on the page: `require_ok` is **false** there. Looking at a
measurement that may be out is the entire point of running it by hand.

Ownership follows the convention the alignment procedures set. The private
inspect pose is `grip_<uid>_i` and `hidden`; the private detector is
`vdet_grip_<uid>`. The name **is** the ownership record, so no schema changed and
an orphan is always traceable: Delete calls `DropGripPositions` +
`DropGripDetector`, and Duplicate re-parents the pose and copies the model with
its `.pgm`, so a copy is self-contained rather than sharing a row the original's
Delete would take away.

## 7. What it is not for

It is **not** a placement. A grip offset plus a registration still adds up to a
typed rotation and one measured point, nowhere near `place_tol_mm`'s 20 µm.
`placement_procedure.lua` keeps its measure/correct/re-measure loop exactly as it
is. What this buys is that the loop starts from a **known** grip instead of an
assumed one — which is the difference between landing on pass 1 and landing on
pass 2, and, when the grip is bad enough, between a correction and a scrapped
part.
