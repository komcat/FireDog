# Device registration: where hex-left and hex-right actually are

*Relating a hexapod's axis space to the gantry camera's, so a gripper tip has a
known position at every moment instead of only after it has been measured.
Companion to `docs/needle-dispense-design.md` (the same idea inside one device)
and `docs/alignment-procedure-design.md` (the same idea about a part).*

**Status: built.** Core `src/core/DeviceReg.{h,cpp}`, commands
`src/command/CommandsDeviceReg.cpp`, procedures `config/scripts/device_registration.lua`
and `config/scripts/device_reg_check.lua` (both STANDARD), page
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
lever arm once. This is that idea across a device boundary.

## 2. What exists today, and why it is not enough

`config/motion_transforms.json` already carries a matrix per device:

```
hex-left    M = [[0,0,1],[0,1,0],[1,0,0]]      global X -> hex Z,  Y -> Y,  Z -> X
hex-right   M = [[0,0,1],[0,-1,0],[-1,0,0]]
gantry-main M = [[1,0,0],[0,1,0],[0,0,-1]]
```

Three properties of that file matter here.

**It has no offset.** `AxisTransform` is `m` and `r` and nothing else, and
`GlobalDeltaToAxes` multiplies a *vector*. That is the entire reason for the
standing rule that only a delta may cross between devices: the two origins have
never been related, so there is no absolute answer to convert. This design
supplies the missing translation.

**Its rotation is typed, not measured.** Every entry is a permutation of ±1 —
somebody worked out which gantry direction each hexapod axis points along and
wrote it down. That is exact if the hexapod is bolted square and approximate
otherwise, and the error is not visible: a mount 2° off predicts correctly at the
point the offset was taught and misses by 1 mm 30 mm away. It degrades smoothly
with distance, so it reads as "the calibration is drifting" rather than as "the
rotation was never measured". **The registration must measure its own rotation.**

**Every one of those matrices is improper** — `det = -1` for all three, because
controller axis-sign conventions are not consistently handed. So the measured
triad must never be assumed right-handed, and `c3 = c1 x c2` is only true up to
a sign that has to be taken from the measurement. A registration that silently
forced a proper rotation would invert one axis of the answer.

## 3. What is stored

`core/DeviceReg.h`, one record **per device**, machine-level, in the same
runtime-writable pair every file of its kind uses
(`config/device_reg.local.json`, seeded by `config/device_reg.json`).

```
R[3][3]   hexapod axis direction -> gantry-main axis direction
t[3]      gantry-main axis position of the tip when the hexapod reads (0,0,0)

predict:  p_gantry = R * h + t
command:  h        = R^T * (p_gantry - t)
```

`p` is in **gantry-main raw axis space**, not global. This is the same choice
`NeedleCal` makes and for the same reason: `CaptureWorldPoint` already returns
`wx = gx + pick.dxMM` in that space, `read_position` reads it and `move_axis_to`
commands it, so the number never passes through a transform that could invert its
Z. Nothing here goes through `AxisTransform`.

Kept beside it, for the same reason `NeedleCal` keeps its three poses and
`AlignResult` keeps its E points — a suspicious registration has to be auditable
without re-teaching it:

| Field | Why it is worth keeping |
|---|---|
| the four measured tip points and the hexapod pose at each | the raw measurement; "which pick is wrong" is the only useful question |
| `colNorm[3]` | each measured column's length before normalisation. Should be 1.000 |
| `orthoDeg[3]` | how far from perpendicular the measured columns were |
| `handed` | the sign the Z jog resolved, `+1` or `-1` |
| `regAt`, `tipAt` | **two dates, not one** — see §7 |
| `checkResidualMm`, `checkedAt` | the last daily check |
| `valid`, `verified` | a seed may aim; only a verified record may be moved through |

`R` is stored **orthonormalised** (nearest rotation to the measured columns,
sign-matched to `handed`), with the raw columns kept alongside. Normalising
throws away information that is worth refusing on, so the refusal happens first
and the clean-up second — never the other way round.

## 4. How it is measured

Every primitive this needs already exists and is device-generic:
`read_position{device}`, `move_axis_to{device, axes}`, `auto_focus` and
`pick_point`. The procedure is a Lua script, exactly like
`needle_calibration.lua`, and for the same reason.

The trick is the one the needle procedure already uses: **subtract two poses read
in the same space and the unknown offset cancels.** Applied to a jog, that means
each hexapod axis hands you its own direction in gantry space directly.

```
P0 = measure the tip                    (drive there, auto_focus, pick_point, read_position)
jog hex X by +L, re-measure -> P1       c1 = (P1 - P0) / L
jog hex Y by +L, re-measure -> P2       c2 = (P2 - P0) / L
jog hex Z by +L, re-measure -> P3       c3 = (P3 - P0) / L
                                        t  = P0 - R * h0
```

Four measurements. Some notes that are not obvious:

* **The camera does not have to keep the tip in a 6 mm field.** The gantry
  follows the tip between jogs and its pose is re-read at every pick, so `L` is
  limited by hexapod travel and by clearance, not by the field of view. Longer is
  better: 10 mm measured to 5 µm resolves the angle to ~0.03°, and the angular
  error is what dominates far from the taught point.
* **Every pose re-focuses.** The hex Z jog moves the tip out of the focal plane
  by design, and `dz`-style reasoning does not apply here — the focused gantry Z
  *is* the measurement of the tip's height.
* **The Z column is measured through a worse channel than the other two.** XY
  comes from a pixel pick; Z comes from autofocus. So the stored `c3` is
  `handed * (e1 x e2)` — derived entirely from the better-conditioned pair — and
  the measured Z jog's job is to resolve **which sign** (§2) and to produce
  `crossDeg`, the disagreement between the two, which is one of the refusals. The
  measurement decides the handedness; the cross product decides the direction.
* **The whole pass happens at a taught safe calibration pose** with bounded jogs
  and an operator confirmation before the first move. A hexapod jogging 10 mm
  with a gripper on it, underneath a gantry that is also moving, is the one part
  of this that can break hardware.
* **The camera follows the tip, and the typed matrix is what lets it.** After a
  jog the tip is `L` away, far outside the field of view. `device_reg` reports
  three **aiming columns** — the measured rotation when the device has one, else
  `R_seed = M_ref * M_dev^T` composed from `motion_transforms.json` — and the
  script drives the reference device to `P0 + L * c_i` before asking for the
  pick. `_seeded` says which it got. That is the only job the typed permutation
  has here, and a couple of degrees is plenty for it: over a 10 mm jog it puts
  the tip a few hundred microns from the middle of the picture. It is also why
  `focus_range_mm` defaults wider than a normal focus — a few hundred microns of
  Z is more than a 0.2 mm sweep can find.
* **P0 is the one measurement nothing can aim.** `t` is exactly what is unknown,
  so the first find is a `manual_adjust` and every measurement after it is
  predicted. Same bootstrap shape as the needle's first imprint, minus the tape
  measure: the seed is already on disk.

The script is **STANDARD** (`IsSystemScript`, `command/ExecutionContext.h`).
Where a gripper is, is a fact about the machine; a product fork of the file would
be five products with five answers and no way to tell which is right — the same
argument that makes `needle_calibration.lua` standard.

## 5. What it refuses

Refusals, not warnings. A registration that is quietly wrong is worse than none,
because everything downstream reads as working:

| Condition | Why it is fatal |
|---|---|
| a column norm outside ~1 ± 0.005 | the axis moved a different distance than it was told to — a units or encoder problem. This is the job `span_mm` does for the align frame |
| columns more than ~1° from perpendicular | the fit is skewed; the miss grows with distance travelled |
| measured `c3` more than ~2° from `±(c1 x c2)` | one pick or one focus pass is bad, and the handedness it was going to resolve cannot be trusted |
| a jog shorter than `kDeviceRegMinJogMm` (1 mm) | below it, pick noise dominates the direction — the same reason `SolveAlignFrame` refuses a span under 0.05 mm |

The thresholds are `kDeviceRegNormTol` / `OrthoTolDeg` / `CrossTolDeg` / `MinJogMm`
in `core/DeviceReg.h`, and `DeviceRegFit` fills **every** quality field before the
first refusal returns — so a rejected fit still names the number that was out
instead of saying the measurement failed. They live in C++ rather than in the
procedure because a gate that only exists in a script is a gate a fork can
delete.

## 6. What consumes it — and what must not

**Consumes it:**

* a live tip position, published every frame per device by `Workflow::PublishTips`
  on `topics::TipAxis` (`tip/<device>/X|Y|Z`, `core/Topics.h` — never hardcoded),
  so the UI can simply read where a gripper is. A tip that *cannot* be predicted
  publishes **nothing** rather than a stale value: absence is the signal, and a
  readout frozen at the last good number is how a rotated hexapod goes on looking
  placed. The reason is logged on the edge, not once per frame;
* a soft envelope check — two tips, or a tip and the gantry, converging;
* `move_to_point{device, x_mm, y_mm, z_mm}`, which commands a gantry-space point
  to a hexapod through `R^T`. This is the thing that was impossible before. It
  refuses an unregistered device, an unverified one (unless `allow_unverified`),
  a point declared in a different reference space than the fit was taught in, a
  live rotation away from the taught one, and — via `max_mm`, default 10 mm — a
  move longer than an approach has any business being. Unattended, nobody
  notices that a stale record turned a short approach into a lunge.

**Must not consume it: the final placement.** Its accuracy is a four-point fit
with one column measured through focus — tens of microns, good enough to arrive
inside the measure loop's capture range, nowhere near `place_tol_mm`'s 0.02 mm.
`placement_procedure.lua` keeps its measure/correct/re-measure loop exactly as
it is; the registration only means the loop converges on pass 1 instead of pass 2.
A `move_to_point` is an **approach**, and it says so in its name, its one-line
help, its log line and on the page — the split is the same one `NeedleCal` draws
between a seed that may aim and a verified record that may dispense.

## 7. Lifetime: two dates, not one

This is the part that differs most from the needle, and it is worth the extra
field.

**`R` is a property of the mount. `t` is a property of the mount *and the
tool*.** Change the gripper and the tip moves; the hexapod's axis directions have
not. So:

| Event | What is invalid | Cost to restore |
|---|---|---|
| gripper changed / tip replaced | `t` only | **one pick** — re-measure the tip at the calibration pose, keep `R` |
| hexapod moved, remounted, or crashed hard | `R` and `t` | the full four-pick pass |

`regAt` and `tipAt` are stamped separately so the Registration page can say which
half is old, and a tip re-teach is a cheap enough operation that nobody is tempted
to skip it and "let the loop absorb it".

**Daily check** is one pick and is exactly the needle's second pass: drive to the
check pose, predict the tip from the record, measure it, store the residual and
`checkedAt`. Inside tolerance it stamps `verified`; outside it, the failure names
both repairs and how to tell them apart — a constant shift is the tool, a residual
that grows with distance from the fitted pose is the rotation.

Two things about the check are load-bearing:

* **The check pose must not be the pose the fit was taught at.** `t` is derived
  from `P0` at `h0`, so predicting at `h0` returns `P0` by construction: the
  residual there measures whether a pick repeats and says nothing whatever about
  the rotation. Hence `check_dx/dy/dz`, defaulting to (5, 5, 0) mm, and hence
  `verified` stays false when all three are zero however small the number comes
  out. The page says so beside the fields; the script says so in the log.
* **`adopt_tip` is off by default.** It re-teaches `t` from the check measurement
  and keeps `R` — the gripper-change path, one pick — and it zeroes the residual
  *whatever caused it*. Adopting a shift you have not explained is how a rotation
  error gets quietly relabelled as a tip offset and carried forward. Adopting also
  clears `verified`, because the record now describes tooling nothing has checked;
  the confirmation is a second run with adopt off.

Like `NeedleCal` and unlike a taught frame, **the persisted record is the licence
to move.** A frame describes one physical part that may have been swapped since;
this describes tooling that has not. It is not cleared at program start, DUT open
or product load, and it does not belong in the product document.

Ownership follows the established route with no new mechanism: `Workflow` holds
the live records mutex-guarded beside `Needle()`, a worker hands a result back
with a queue, and **only `App::Update` writes the file** — an engine thread must
not be halfway through a config file when a Stop lands.

## 8. The rule this changes

CLAUDE.md currently says, under **Vision**:

> Across devices, only a DELTA transforms. Placing an object held by a hexapod on
> a gantry-measured nominal is a **relative** move: `AxisTransform` has no offset
> and the two origins have never been related, so an absolute machine XY must
> never be commanded to a hexapod.

The premise in the middle of that sentence is precisely what this removes. The
rule was **rewritten, not deleted** — delete it and the next person places a part
open-loop through a fit that was never meant to carry it. What CLAUDE.md says
now:

> **Across devices, an absolute position crosses only through a VERIFIED
> `DeviceReg`, and only as an approach.** Without one, `AxisTransform` has no
> offset and the origins are unrelated, so only a delta transforms. With one, a
> gantry-space point may be commanded to a hexapod via `move_to_point` to get the
> tip close — the final placement is still the measured delta the placement loop
> computes, because the registration is a four-point fit and the tolerance is
> 20 µm. Mapping a delta stays `Workflow::GlobalDeltaToAxes`, never `JogGlobal`.

## 9. Sketch of the pieces

| Piece | Where |
|---|---|
| `DeviceReg` record, predict/solve helpers, JSON, load/save | `src/core/DeviceReg.{h,cpp}` |
| live records, queue from worker, DataBus tip publish | `src/workflow/Workflow.{h,cpp}` |
| `device_reg`, `device_reg_save`, `device_reg_tip_save`, `device_reg_check_save`, `move_to_point`, `device_reg_run`, `device_reg_check_run` | `src/command/CommandsDeviceReg.cpp` |
| the four-pick procedure (STANDARD) | `config/scripts/device_registration.lua` |
| the check, with the optional tip adopt (STANDARD) | `config/scripts/device_reg_check.lua` |
| the page: record, both dates, residual, quality numbers, run/check, gated on `orch_.Busy()` | `src/ui/pages/PageDeviceReg.cpp` (View > Registration, F8) |
| the file write, draining the queue | `App::Update` |
| the empty shipped seed and its notes | `config/device_reg.json` |

The queue drain differs from the needle's in one way worth knowing: the file
holds the **whole fleet**, so `App::Update` updates the live map first and writes
it as a unit. Writing only the record that changed would drop every other
device's registration on the first save.

Page access is **Admin** — this teaches the machine, which is the line
`MinRoleFor` already draws.

## 10. Deliberately not in scope

* **U/V/W compensation.** The grippers are used at zero rotation, so the record
  is fitted there, stores the angles it was taught at, and **refuses to predict**
  when the live rotations differ beyond a threshold. Silently predicting through
  a rotation the fit never saw is the failure this avoids. If rotations are ever
  commanded, the record grows a tool vector from the pivot and prediction becomes
  `t + R*(h + Rot(u,v,w)*r_tool)` — the PI controller's own pivot setting is the
  natural source for `r_tool`.
* **Registering the hexapods to each other.** Both register to gantry-main, so
  hex-to-hex falls out by composition and needs no third calibration.
* **Replacing `motion_transforms.json`.** The measured `R` is strictly better
  than the typed permutation, but the transform file feeds `GlobalDeltaToAxes`
  for jogging and a great deal already depends on it. Leave it alone; a later
  step can offer "adopt the measured rotation" as an explicit, reviewable action
  on the Transforms page.
