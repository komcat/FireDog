# Needle calibration and dispensing

Ported from `bobcat_engine`'s **Calibrate Camera-to-Needle Offset** and
**Dispense** process plugins. The algorithms are the same; almost none of the
code is, because the two procedures are Lua scripts here and were C++ plugins
there.

---

## The problem

The camera and the dispense needle ride the same gantry, a hand's width apart —
on the reference machine about **(−104, +13) mm**. So "dispense on the feature I
am looking at" is not a move to the feature: it is a move to the feature *plus*
that lever arm. Until the arm has been measured, nothing can dispense where the
camera points, and there is no way to derive it from the config — it is a
physical fact about how the tooling was bolted on.

## What is stored

`core/NeedleCal.h`, one record, machine-level:

```
dx, dy   add to a CAMERA pose to get the pose that puts the NEEDLE there
dz       add to a FOCUSED camera Z to reach the height the needle touches at
```

plus the three poses it was subtracted from (camera, needle-at-touch, imprint),
the residual of the verify pass, and when it was taken.

**Machine-level, not per-product.** The needle is a fixture. It does not change
because a different part was loaded, and asking every product to re-teach the
same physical constant is how five products end up with five different answers
and nobody knows which one is right. It lives in `config/needle_cal.local.json`
with a `config/needle_cal.json` seed — the runtime-writable pair, same rule as
every other file of its kind. It is re-taught when the **needle** changes.

**Axis space, never global.** Camera and needle are on the same device, so the
delta between them never crosses a frame and must never go through
`AxisTransform`. `gantry-main`'s transform inverts Z (`M33 = −1`), so a number
read from `IMotionController::Positions()` and then issued as a global delta
would descend when it meant to retreat. Everything — the stored offset, the
`read_position` reads, the `move_axis_to` targets — stays in raw axis
coordinates of the named device.

**dz is an offset, not a height.** bobcat stored the needle's touch height as an
absolute gantry coordinate, and then needed a separate taught file for every
dispense point (12 of them on the live machine) because an absolute Z only
describes the one surface it was taught on. Paired with `auto_focus`, one offset
serves every point: focus at the target, add `dz`, that is the touch height
*there*.

**The Z direction is never assumed.** `NeedleDownSign` is `sign(dz)`, taken from
the measurement. bobcat hardcoded "+Z is down", which is right on that machine
and a crash on the next one.

## Ownership

`Workflow` holds the live record (mutex-guarded, like `LiveFrame`), loads it in
`StartNeedleCal()`, and exposes `Needle()` to worker threads. A calibration run
hands its result back with `QueueNeedleCal` and `App::Update` drains it — the
same worker→UI route a taught frame takes, for the same reason: an engine thread
must not write a config file, least of all one it might be halfway through when
a Stop lands.

**There is no live-vs-saved distinction here, and that is deliberate.** An
alignment frame describes one physical part, which may have been swapped since,
so a run must never consume the saved copy. The needle offset describes the
tooling, which has not been swapped. The persisted record *is* the licence to
move. It goes stale for exactly one reason — somebody changed the needle — and
`calibratedAt` on the Needle page is what says so.

## Commands

Four of them are generic primitives that were simply missing:

| Command | What it adds |
|---|---|
| `read_position{device, ctx_prefix}` | a script can see a pose it did not ask an operator to jog to. Before this, the only pose a script could read was a side effect of `manual_adjust`. |
| `read_velocity{device, ctx_prefix}` | the velocity the controller is set to, as `<prefix>_mm_s`, so a script that changes it can put it back. |
| `move_axis_to{device, axes, x_mm, y_mm, z_mm, …}` | absolute per-axis coordinated move through the shared `MoveWithApproach`. `axes` is **required**: a step meaning "descend Z" that silently also commanded X=0 and Y=0 would cross the machine at speed. |
| `pick_point{device, label, hint…, ctx_prefix}` | the crosshair pick with no alignment procedure behind it. Reuses `CaptureWorldPoint`, so the mark→world-mm maths has one implementation. |

And four in the family proper (`command/CommandsNeedle.cpp`):
`needle_offset`, `needle_offset_save`, `needle_cal_run`, `dispense_run`. Between
them they do almost nothing — read the record, write the record, and launch the
two scripts. That is the point.

## Why the procedures are scripts

bobcat built both as `IProcessPlugin::BuildSequence()` bodies emitting primitive
actions into a sequence language, and paid for it in a way worth recording. That
language resolved *both* operands of its `Calculate` action as context keys, with
no path for a numeric literal — so a step meaning `previous_z − 1.0` looked up a
key named `"1.0"`, found nothing, and computed `previous_z − 0`. The "park one
millimetre above the last touch height" approach drove straight onto the coupon
at 5 mm/s, and the prompt that came up afterwards said the needle was a
millimetre clear.

Lua has real numbers. Keeping the arithmetic in the script is not a stylistic
preference; it is where that whole class of defect went.

## `needle_calibration.lua` — STANDARD

On `IsSystemScript`, so every product runs the shipped copy and a product fork is
ignored with a warning. It measures the **machine**; there is no such thing as
one product's version of the answer.

Per pass:

1. Prompt for a coupon (pass ≥ 2: *a clean spot* — a second imprint on the first
   one has nothing to measure).
2. `manual_adjust` 100 µm — centre the camera on it.
3. `auto_focus` — so `dz` is measured down from a focal plane.
4. `read_position` → the camera reference.
5. With an offset to aim by, `move_axis_to` the needle over that spot **at the
   camera's Z**. The traverse is the length of the lever arm and happens well
   clear of the coupon.
6. Confirm → `extend_slide` → settle.
7. With a known `dz`, park `clear_mm` short of the last touch height; then
   `manual_adjust` 10 µm on Z (plus XY when nothing has aimed it) → the operator
   brings it down to contact. `read_position` takes the pose, not
   `manual_adjust`'s `ctx_prefix`: the latter reads the poll cache, the former
   demands a fast poll and waits a cycle out, and at a touch height that
   difference is the measurement.
8. Confirm → `pulse_output` fires the shot.
9. Retreat **Z first, then the slide, then XY** — the needle never travels
   sideways while it is down near the dot it just made. (The *dispense*
   procedure retracts before lifting; here Z must clear first, because the
   operator has just jogged the tip into contact and the imprint is what is
   being measured.)
10. `pick_point` on the imprint, with the expected position as a hint (an edge
    arrow with its distance when it is out of frame).
11. `dx = tip_x − dot_wx`, `dy = tip_y − dot_wy`, `dz = tip_z − cam_z`.

Two passes by default. The second re-measures and the difference is the
residual; inside `tol_mm` the record is marked `verified`, outside it the offset
is still saved (it is better than what was there) and the script then fails,
because a measurement that will not repeat points at the marking or at a needle
that deflects on contact, and neither is fixed by loosening the tolerance.
`passes = 1` succeeds with a loud unverified warning — the same bargain
`placement_procedure.lua` strikes, and the Needle page keeps saying so
afterwards.

The offset is **carried in Lua locals between passes**, not re-read through
`needle_offset`. A saved calibration reaches `Workflow` via the UI thread one
frame later, so a second pass that re-read it could get the value from before the
first pass.

### Bootstrapping

With no offset at all the first imprint lands ~104 mm outside a ~6 mm field of
view and the "click the dot" step has nothing to show. The Needle page takes a
typed tape-measure **seed**, stored with `valid: false` — enough to aim with,
never enough to dispense through. Failing that, the script has the operator jog
the needle over the coupon by hand and hunt for the imprint afterwards; jogging
the camera during the hunt is harmless, because `pick_point` reports the dot's
world position and does not care where the camera stood.

## `dispense_procedure.lua` — NOT standard

Deliberately absent from `IsSystemScript`. Where the needle is, is a fact about
the machine; how much glue goes where is a fact about the **product**, so a
product's own copy of this file wins over the shipped one and is meant to.

The dispense point is a **camera pose over the target**, held one of two ways,
and a step holds exactly one:

- **`point`** (`ParamType::PointXyz`, `{"X","Y","Z"}` in the device's axis
  space) — taught INTO the step by the editor's *Teach here*. This is the usual
  case: a dot is one step's business, and bobcat needed one
  `navigation_<uid>.json` side file per recipe step (18 on the live machine) to
  say the same thing. No saved position to create or name, no confirmation —
  nothing else references it.
- **`position`** — a saved position, for when several steps share one pose.

The `position` picker is the choice: its empty entry reads *custom point —
taught in this step*, and only then does the `point` row (with its own Go /
Teach here / Clear) appear. Picking a position clears the point and hides the
row; teaching a point clears the position;
`dispense_run` fails before anything moves if it holds both or neither. Nothing
in the recipe holds a needle coordinate either way, so nothing goes stale when
the needle is changed: re-calibrate and every taught point still works.

1. `needle_offset{require = true}` — fail before anything travels.
2. `move_axis_to` the point (or `move_to_position`) — camera over it. With
   `adjust` (default on), a `manual_adjust` with **X/Y only** lets the operator
   centre the crosshair on this part's point; everything below starts from where
   they leave it. Z stays locked because the touch height is computed from the
   camera's Z. The adjustment applies to **this run only**. Nothing is written
   back into the step or the position: the next part needs its own nudge, and a
   run that re-taught its own point would drift the point one part at a time.
   Then the optional `auto_focus`, over the spot the operator chose.
3. `read_position` → `cam_*`; `touch_z = cam_z + dz + dir*trim`,
   `target_z = touch_z − dir*gap`, `safety_z = target_z − dir*safety`.
   **`z_trim_mm` and `gap_mm` are not the same correction and must stay
   separate.** Trim corrects the *calibration* — the measured touch height was
   slightly off on this machine. Gap is a deliberate *standoff*: the needle is
   meant to stop short and let the bead bridge it, which is a property of the
   glue and the dot size. Folding one into the other means a product that later
   wants no gap also throws away the machine correction it was hiding.
4. `move_axis_to` the needle over the point, still at the camera's Z.
5. Confirm → `extend_slide`.
6. Two-speed descent: `v_approach` to `safety_z`, `v_final` to `target_z`.
7. Confirm → `before_ms` → `pulse_output` → `after_ms`. A cancel here retreats
   before failing: leaving the needle resting on the part would be worse than
   the dispense.
8. Retreat: **retract the slide first**, then Z at `v_liftoff`, then XY. The
   cylinder lifts the needle off the dot on its own stroke — straighter and
   faster than Z can, and it breaks the string while the head is still
   stationary over the point. Climbing at `v_final` instead would crawl away
   from a part there is nothing left to arrive at. XY stays last either way, so
   the needle never travels sideways while it is down near the dot. Once Z is
   clear, the gantry is set back to the velocity `read_velocity` read off the
   controller before the first move.
9. Optionally bring the camera back, capture an image to the DUT, and ask.
   With `confirm` on the run waits for the answer. With it off, `verify_ms`
   (default 3000) still puts the dot in front of the operator: the prompt
   counts down and continues by itself (`prompt`'s `auto_ok_ms`), and a Cancel
   inside the window rejects the dot. `0` skips the prompt.

`pulse_output` rather than on/delay/off because it closes the valve **even when
a Stop interrupts the wait** (`CommandsIo.cpp`). A stop that left
`Dispenser_Shot` energised would keep pumping.

**Velocity persists on the controller, and this bites hardest here.** The
descent runs at `v_final`, so whatever the last move sets is what the *next*
recipe step — and the next run of this one — travels at; the symptom is a head
that appears not to move for seconds and then a `move_axis_to` timeout. So the
script reads the gantry's velocity **off the controller** before it moves
(`read_velocity`: PI `qVLS`, ACS `acsc_GetVelocity`) and sets it back right
after the lift-off — and again at the end if the verify leg travelled at
`v_travel`. A gantry at 40 mm/s before the step is at 40 mm/s after it. Read,
not remembered: the jog page's combo shows "(device)" until somebody sets one,
so the only honest source of "what it was" is the controller. A device that
cannot report its velocity fails the step before anything has moved.

**There is no pressure control, no purge, no bead.** bobcat has none either. The
external regulator sets the flow and `shot_ms` sets how long it flows; `shot_ms`
is not a volume unless you know what the regulator is at.

## The UI

`Page::Needle` (F7, `ui/pages/PageNeedle.cpp`): the current offset and its date,
the three poses behind it under a fold, the hardware selectors, the run
parameters, Run/Stop gated on `orch_.Busy()`, and the seed row.

Nothing on it actuates. The calibration fires a shot, but from inside the
procedure behind the procedure's own confirmations on an engine thread — which
is why there is no test-dispense button and no arm checkbox: there is nothing to
arm.

## Hardware it expects

From `config/hardware.json` as shipped: `gantry-main` (ACS, X/Y/Z),
`Dispenser_Head` as a pneumatic slide (output `IOBottom/Dispenser_Head`, sensors
`IOTop/Dispenser_Head_Down` and `_Up`), and `IOBottom/Dispenser_Shot` as the
valve output. The IO watch also binds `switch6 → Dispenser_Shot`, a physical
button on the operator panel; it is rising-edge from an input, so it does not
contend with a scripted shot.

## Notes for whoever ports the next piece

The reference implementation carries several defects that are easy to inherit by
reading it too faithfully:

- `context/Calculate` has no numeric literals (above) — `CalibrateCameraToNeedleOffset.cpp:352`.
- `CapturePosition` narrows `double` → `float` in its context store.
- `Extend`/`Retract` ignore the `debounce_delay_ms` and `timeout_ms` the plugins
  pass them; the real timeout is in `pneumatic_slides.json`.
- `SetOutput{pinName}` and `SetOutputPin{pin}` are two different commands used
  inconsistently between the two plugins, surviving only on fallback logic.
- `CalibrationProcesses.cpp:258` emits a `MoveToAbs` with only X and Y, which the
  handler rejects at run time.
- `docs/calibration-system-overview.md`, `config/calibration/README.md` and the
  whole `config/calibration/` tree describe a layout the code abandoned; live
  code uses `config/SYSTEM/` only.
