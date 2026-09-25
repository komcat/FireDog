# Waffle Tray — Design

Status: **implemented** (September 2026). Code: `core/WaffleTray.{h,cpp}`,
`command/CommandsTray.cpp`, `ui/pages/PageTray.cpp`, the `trays` role in
`session/ProductSession`, the cursor on `Workflow`.

## 1. The question

A saved position is a fixed point. A tray of parts — lenses in a 4 × 6 cavity
array, dies on a waffle pack — is a few hundred points that differ by a pitch,
and teaching each one as a position is both the work and the error source: one
mistyped cavity is a crash. What the machine needs is one record that yields a
pose for **any** cell `(row, col)`, so a recipe can say "the next lens" instead
of naming a position.

## 2. The record: three corners, a parallelogram

A tray is a named, uid-stamped record on a motion device with `rows × cols` and
three taught corners:

| Corner | Cell | Needed when |
|---|---|---|
| `origin` | (1, 1) | always |
| `rowEnd` | (1, C) | `cols > 1` |
| `far` | (R, C) | `rows > 1` |

Each corner is **every axis the controller reported** — X/Y/Z and U/V/W on a
hexapod — exactly what a `SavedPos` holds, in the device's own axis space. The
pose of a cell is linear in both directions:

```
rowStart  = rowEnd      when cols > 1, else origin      (a 1-column tray's far is (R,1))
cell(r,c) = origin + (c-1)/(C-1) * (rowEnd - origin)
                   + (r-1)/(R-1) * (far    - rowStart)
```

applied per axis. Rotations interpolate to a constant when the three corners
were taught at one rotation, which the page tells the operator to do. Three
points describe a **rotated** tray, not a keystoned one; a bilinear four-corner
form is the obvious upgrade if a real tray ever needs it, and it would be one
more `TrayCorner` plus one line in `TrayCellPose`.

`TrayCellPose` (`core/WaffleTray.cpp`) is the ONE place this arithmetic lives.
The page's tooltip, `move_to_cell` and the cursor's walk all call it, so there is
exactly one answer to where a cell is.

### Why coordinates and not position uids

The grip profile stores its inspect pose as a hidden private position, because
`grip_detect` drives there with the registry's own `move_to_position` and the
Jog pad can offer it. A tray's cells are **derived**: nothing ever moves to a
corner through a position command, and a record whose three corners were rows
in another file would be two files to keep in step for no consumer. The
`or taught...` picker (`TaughtPosePicker`, `ui/Widgets`) still lets an operator
source a corner from a position the product has — it resolves to coordinates on
the spot, the way `AdoptDeviceRegPose` does.

### Product state

A tray is part of how a product is loaded, so it lives with the product:
manifest role `trays`, file `{uid}.trays.json`, `config/trays.local.json` →
`trays.json` when no product is loaded, explicit Save + dirty baseline — the
grips model, line for line. `NewProduct` carries trays over intact: the corners
say where a fixture sits on this machine and the masks which cavities work,
and nothing in the record describes one part the way a taught frame or a
reference offset does.

### What refuses (`TrayValidate`)

`rows`/`cols` outside 1..1000 · a corner the size needs untaught · a corner
whose axis set is not the origin's (re-taught on another device) · a pitch under
`kTrayMinPitchMm` (10 µm: two teaches of one spot) · row and column directions
parallel (the far corner lies on the first row). One implementation: the page
prints it, `move_to_cell` fails on it. Changing the tray's device on the page
**clears all three corners** and says so — the same numbers on another
controller are a different place.

## 3. Masks

`disabledRows`, `disabledCols`, `disabledCells`. A cell is enabled when it is in
range and none of the three names it. `TrayNormalize` prunes what a shrunk grid
no longer has, in the click handler that changed the size — never per frame. A
disabled cell is never offered by `tray_next` and **fails** `move_to_cell`
unless the step says `skip_disabled`, in which case `<prefix>_moved` is false:
a move that silently did nothing looks exactly like one that arrived.

## 4. The cursor: runtime state on Workflow

`tray_next` walks the enabled cells and hands out the next one a run has not
taken. Which cells are taken is **runtime** state, never persisted: it
describes the parts on ONE physical tray, exactly like a live frame describes
one part, and a persisted "taken" would read as a licence to skip a cavity
after a restart nobody remembers.

It lives on `Workflow` (`TrayNext` / `TrayConsumed` / `ClearTrayCursor`, under
`trayMu_`) for the live frame's reason: a program runs each slot as its own
engine Start, so per-run state would be wiped between one part and the next —
and the whole point is that slot N takes the cell after slot N−1. Hence,
**unlike** a frame, it is NOT cleared at program start or DUT open. It is
cleared at product load and new product (`App`), by the page's *Reset taken*,
by a `tray_reset` step, and for one tray when it is deleted or its device
changes. Every clear says so in the log.

`TrayNext` picks and consumes under ONE lock, so two lanes of a `parallel`
block could not take the same cell even though `move_to_cell` is not a parallel
mover (see §5). `wrap` releases exactly the cells the walk covers and starts
over — a tray reloaded without a reset — and a cell disabled since stays as it
was.

## 5. The commands

| Step | Does |
|---|---|
| `move_to_cell` | tray + `row`/`col` (typed) or `row_key`/`col_key` (ctx) → `TrayCellPose` → optional stand-off (`offset_axis` + `offset_mm`, both or neither) → `ApplyPreMove` → `MoveWithApproach`. Writes `<prefix>_row/_col/_moved` (`ctx_prefix`, default `cell`). |
| `tray_next` | next enabled, untaken cell in `order` (`row_major` / `col_major` / `serpentine`) → `<prefix>_row/_col/_index/_remaining/_ok` (default prefix `tray`). Exhausted: `_ok = false`, Success unless `require`. |
| `tray_reset` | release every taken cell of the tray. |

A missing `row_key` **fails** rather than falling back to the typed value: a
loop that forgot to set its index would otherwise drive every pass to the same
cell.

**Not a teach mover.** A teach writes the live pose into the position the step
drove to, by uid; a cell has no row to write into, and re-teaching a corner on
the page is how a tray moves. So `move_to_cell` declares no `teach` param and a
teach-mode run disables it, like every non-mover.

**Not a parallel mover.** `parallel` deals lanes by the VALUE of the param
`parallelBy` names, which must be a device; this step's device is inside the
tray record. Naming `tray` would make two trays on one controller two lanes on
one device. The validator refuses it inside a block.

### Getting the index into the step

Params are literals: nothing substitutes `{key}` into an Int. Three ways in,
each a ctx key the step reads itself (the `grip_move_to_camera` precedent):

1. `tray_next` → `move_to_cell row_key=tray_row col_key=tray_col`. One part per
   run, across recipes and program slots (the cursor).
2. `loop count=R index_ctx=r { loop count=C index_ctx=c { move_to_cell row_key=r col_key=c } }`.
   The `loop` step's new optional `index_ctx` writes the 1-based pass number
   before each pass (`StepRunner`); it counts a `count` loop only.
3. Lua: `fd.move_to_cell{tray="lens tray A", row=r, col=c}` in any loop —
   integral Lua numbers already pass as Int params.

## 6. The page

`Page::Tray`, a takeover on the strip and F10, **Technician**: a corner is a
raw axis reading into the product document — a Motion-page saved position,
which a Technician already teaches. Every Go sits behind the jog arm and
`orch_.Busy()`, and an unarmed click raises the same alert the Grip page uses.

Left: the grid — a button per cell, headers that toggle a whole row or column,
consumed cells tinted, right-click drives there through `StartWatchedMove` (the
pose overload; a cell has no uid). Past 2500 cells the grid is not drawn and
line toggles take its place. Right: the tray list (New / Duplicate / Delete /
Save), name, device, size, the three corner cards (Teach here · or taught... ·
Go · values · date), the pitch readout, the validation line, *Reset taken*.

Two edits save on the spot instead of waiting for Save: teaching a corner and
changing the device. Both are physical acts an operator must not lose to a
forgotten click.
