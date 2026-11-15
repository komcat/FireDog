# Motion interlocks — conditional refusal of moves across devices

Status: BUILT (2026-11-14). Model `src/core/Interlock.{h,cpp}`, gate and watch in
`src/workflow/Workflow.cpp` (section *Motion interlocks*), editor
`src/ui/pages/PageInterlocks.cpp`, status window `src/ui/windows/WndInterlocks.cpp`.

## 1. Why

`SafetyWatch` is the only guard FireDog had: a latched, E-Stop-style alert that
blocks **every** move. What it cannot say is the machine's geometry:

- *If any pneumatic slide is down AND gantry-main Z > 50, gantry-main may not move
  X/Y, and hex-left / hex-right may not move X/Y/Z.*
- *If the stage's X < −5, the hexapods may not move* (the stage is under the grippers).

A rule like this must hold for every way the machine moves: the jog, every Go
button, recipes, Lua, `parallel` lanes, mapped paths, ACS buffers, references.
It must also catch a rule that becomes true **while** a move runs: a slide
dropped from the IO page during a gantry XY move.

## 2. The rule

```
WHEN  every condition holds          (AND; write OR as two rules)
THEN  refuse every listed motion
```

| Condition | True when |
|---|---|
| `slide` | ANY of the named slides (all slides when none named) is `extended` / `not retracted` / `retracted` |
| `io` | `device`/`pin` (an input, by name) reads ON / OFF |
| `axis` | `device` `axis` `>` `>=` `<` `<=` `value` — in **axis space** |

| Block | Refuses |
|---|---|
| `axes` | the named axes of a motion device (empty = every axis), either way or one direction (`dir` +1 / −1) |
| `slide_extend` | extending one slide, or any slide |

Retracting a slide is never refused: it is the return to the safe state.

## 3. Three properties

**Fail closed.** These count as **true**, and the reason says so:
- a disconnected device, or an IO cache with no reading yet
- a name that resolves to nothing
- a slide whose sensors show neither end, or both
- an axis on a device that is referencing or running a buffer

A typo makes a rule always active, never inert. The editor draws every unresolved
name in red. A `interlocks.local.json` that exists but cannot be parsed does **not**
fall back to the shipped (empty) default, because that would silently switch every
rule off. Every move is refused until the file is fixed or an Admin bypasses.

**True at either end.** An axis condition holds if any of these satisfies it:
- the measured value
- an in-flight commanded target
- the target of the move being checked

Positions are cached (1 Hz on an idle controller), and a Z still travelling to 60
reads 30. Workflow remembers what it sent each device (`ilCmd_`), and that
commanded end closes the gap.

**Predicted.** A move is judged on the state it would create, with its own targets
applied as commanded ends. "Z to 60 and XY together" is refused by the rule that
blocks XY while Z > 50; it is not allowed just because Z happens to start low.

**Axis space**, not global. `gantry-main`'s transform inverts Z (axis Z− is up),
so a rule is written against what the Motion page shows for the device. The
editor puts the live value beside every threshold and has an `= now` button.

## 4. The gate

`IlCheckIntent(cfg, snapshot, intent)` refuses an intent if a rule that is active
in the **predicted** state:

1. blocks an axis the intent moves, in the direction it moves it. Reference and
   Buffer are judged as moving every axis. **or**
2. would become active **because of this intent** while one of its blocked axes is
   already in flight on another device. Extending a slide while gantry XY runs at
   Z 62 would only trip the watch, so the gate refuses it up front. A rule that was
   already active is the watch's business, not the gate's.

**What counts as moving an axis.** A saved position names every axis, so "XY at
the same place, Z down" must not read as an XY move just because the servo jitters
around the measured value. Workflow keeps the **last target sent** per axis
(`ilLastCmd_`), and the reference for a new target is chosen in this order:

- the in-flight target, else
- the last-sent target, while the axis still sits within 0.01 of it, else
- the measured value.

Against a sent target, anything but an exact restatement is motion (down to the
0.1 µm jog step; a looser test would let a fine jog creep into a blocked zone one
press at a time). Against only a reading, 2 µm of jitter is not. With no
reference at all, the axis counts as moving, in either direction. Stopping a
device, a reference, a buffer and a disconnect all drop its last-sent targets.

Workflow calls it in **every** method that reaches a controller, right after the
safety check, holding `ilGateMu_` across check + issue + `NoteCommanded`. Lanes,
workers and the UI jog race each other, so a check that is not atomic with its
issue is no check. The methods are:
- `MoveToPosition`, `MoveToPositionDynamic`
- `MoveRelativeGlobal`, `MoveGantryRelativeMM`
- `JogLocalAxis` (every jog funnels through it). A refused press does not advance
  the jog track. The lock also serialises `jogTracks_`.
- `StartPathMove`, and every later leg in `TickPath`
- `RunBuffer`, `StartReference`
- `ExtendSlide`, and `SetIoOutput` switching a slide's valve ON. Otherwise the IO
  page is a way round the rules.

Every command, Lua call, lane and UI surface goes through these methods: no driver
move call exists outside `Workflow.cpp`. **A new move method must gate the same
way.**

A refusal is logged under `interlock`, recorded for the UI banner
(`LastInterlockRefusal`), and left in a **thread-local** (`Workflow::LastMoveRefusal`).
Commands append that through `MoveRefusedWhy()` (`command/MotionWaits.h`), so a
refused step reads `move to 'P1' rejected - [Slide down + Z high] refused move of
gantry-main X,Y: UV_Head extended; gantry-main Z 62.013 > 50.000`. It is an ordinary
step failure, so `on_error` and the on-fail recipe apply.

## 5. The watch

One thread owned by `Workflow`, started by `Start`. `~Workflow` joins it first,
after the tower light. Every `watch_ms` (default 50) it does three things:

1. **Retires in-flight targets.** A target is retired when it is reached
   (≤ 0.002), or when the move is ≥ 1 s old and the controller reports settled
   (not referencing; the buffer it started has ended). `IsSettled` is a controller
   query, so it is asked at most every 250 ms per device and only for devices with
   something in flight. It keeps `RequestFastPoll` open while anything is in flight.
2. **Ends a bypass that ran out.**
3. **Looks for violations** (`IlFindViolations`): an active rule whose blocked axis
   is in flight. For each one it does `StopAll` on **that device only**, forgets
   its commanded end, raises `pathCancelReq_` / `bufFollowCancelReq_`, and latches
   a **trip**.

A trip behaves like a critical safety alert:
- the engines abort through `Workflow::MotionAbortActive()`. The run ends Stopped
  with "interlock trip", and the on-fail recipe does not run.
- every move method refuses (`MotionInhibited`)
- the tower light alarms
- the trip modal waits for Acknowledge

After the acknowledgement the rule still refuses what it blocks.

`StopMotion` / `StopAllMotion` forget a device's commanded end: a stopped device
has none.

**Known limits.** Motion started outside FireDog is invisible to both halves. An IO
Watch binding that switches a slide's valve writes the driver directly, so the gate
cannot refuse it. The watch still stops any blocked axis that is moving.

## 6. Bypass

Admin only (`kRoleInterlockBypass`), from the Interlocks window: 1–30 min. Gate and
watch pass and log what they would have done (`bypassed: …`). A red menu-bar banner
counts down and names who started it. It ends when:
- it runs out
- someone presses *End bypass*
- **any sign-out** (`App::LogOut`, the jog arm's rule)

It never covers the safety alert.

## 7. Files and access

| What | Where | Level |
|---|---|---|
| rules, machine copy | `<machine root>/config/interlocks.local.json` (Save writes temp + rename) | — |
| rules, shipped default | `config/interlocks.json` (empty) | — |
| Interlock Editor page | View > Interlock Editor (`Page::Interlocks`) | Admin (the `MinRoleFor` default) |
| Interlocks window | View > Interlocks, read only | Technician (`kRoleInterlockView`) |
| bypass | the Interlocks window | Admin |
| rig | `interlocks` prints every rule's live state; it evaluates, never acts | — |

Interlocks are **machine** state, because they describe the machine's geometry.
Like `hardware.json`, a change every machine needs is made on each machine.

The editor evaluates the **working copy** on the live snapshot. Every condition
row shows TRUE/false with its reason, and every rule shows a dot (red = active,
amber = active only because something is unreadable, green = inactive, grey =
disabled). An edit can be judged against the machine before it is applied.

The Jog window greys each direction button a rule refuses right now
(`InterlockBlockedDirs`: a ±0.001 step through the same `IlCheckIntent`), with the
rule in the tooltip. It is advisory only; the gate decides.

## 8. JSON

```json
{
  "enabled": true,
  "watch_ms": 50,
  "rules": [
    { "uid": "3f9a01c2", "name": "Slide down + gantry Z high", "enabled": true,
      "note": "the grippers sit under the UV head",
      "when": [ { "type": "slide", "slides": [], "state": "not_retracted" },
                { "type": "axis", "device": "gantry-main", "axis": "Z", "op": ">", "value": 50.0 } ],
      "blocks": [ { "type": "axes", "device": "gantry-main", "axes": ["X", "Y"], "dir": 0 },
                  { "type": "axes", "device": "hex-left",  "axes": ["X", "Y", "Z"], "dir": 0 },
                  { "type": "axes", "device": "hex-right", "axes": ["X", "Y", "Z"], "dir": 0 },
                  { "type": "slide_extend", "slide": "" } ] }
  ]
}
```

`state` is `extended` | `not_retracted` | `retracted`. An `io` condition is
`{ "type": "io", "device": "IOTop", "pin": "Door_Closed", "on": false }`. An
unknown condition `type` loads as an axis condition on nothing, so it fails closed
rather than vanishing.

## 9. Follow-ups

- POR: compare `interlocks.local.json` rule by rule (by name), like `hardware.json`.
- A rule could carry "allow the move that clears it" automatically. Today the
  direction filter says it explicitly (`dir`).
