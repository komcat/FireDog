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

## 4. Used cells: product state, with a live cursor on Workflow

`tray_next` walks the enabled cells and hands out the next one that still
holds a part, marking it **used** (the cavity is empty). Which cells are used
is saved WITH the tray (`usedCells`, `used_cells` in the file): it describes
the fixture on the machine, an operator marks cavities by hand — one by
clicking it, all with *All full* / *All empty* — and a restart mid-tray must
not start the next recipe at (1,1) again. (The first version kept it as
session-only state, like a live frame; the shift-change case is what changed
it.)

A run cannot write the document (engines hold a copy, UI thread only), so the
live truth a worker reads is a cursor on `Workflow` (`TrayNext` /
`TrayConsumed` / `TraySeed` / `ClearTrayCursor`, under `trayMu_`), held there
for the live frame's reason: a program runs each slot as its own engine Start,
and the whole point is that slot N takes the cell after slot N−1. Hence,
**unlike** a frame, it is NOT cleared at program start or DUT open. The two
stay in step by construction:

- **Product load / new product**: `App` clears the cursor and re-seeds it from
  every tray's `usedCells` — the persisted set is the truth.
- **A run**: `TrayNext` marks the cursor and queues a `TrayUsedEvent` under
  the same lock; `tray_reset` clears the cursor and queues "all full".
  `App::Update` drains `TakeTrayUsed` into `ProductSession::SetTrayCellsUsed`,
  which saves on the spot (a grip measurement's route). The drain never
  touches the cursor: that is the worker's, and a second writer would race it.
- **The page**: `App::SetTrayCellsUsed` writes the cursor and then the
  document. Nothing else writes either.

`TrayNext` picks, consumes and queues under ONE lock, so two lanes of a
`parallel` block could not take the same cell even though `move_to_cell` is
not a parallel mover (see §5). `wrap` marks every cell full and starts over —
a tray reloaded without a reset. Delete drops the cursor with the record; a
device change clears the corners and the used set together (a re-taught tray
starts full).

## 5. The commands

| Step | Does |
|---|---|
| `move_to_cell` | tray + `row`/`col` (typed) or `row_key`/`col_key` (ctx) → `TrayCellPose` → optional stand-off (`offset_axis` + `offset_mm`, both or neither) → `ApplyPreMove` → `MoveWithApproach`. Writes `<prefix>_row/_col/_moved` (`ctx_prefix`, default `cell`). |
| `tray_next` | next enabled cell still holding a part, in `order` (`row_major` / `col_major` / `serpentine`), marked used → `<prefix>_row/_col/_index/_remaining/_ok` (default prefix `tray`). Tray empty: `_ok = false`, Success unless `require`. |
| `move_to_active_cell` | the same move to the tray's ACTIVE cell — the one an operator clicked on the grid, or the last one `tray_next` handed out. Fails when none is set. |
| `tray_mark_used` | the step after the work: mark the ACTIVE cell used and (`advance`, default on) make the next full enabled cell after it in `order` the active one; `<prefix>_row/_col` name the new active cell, `_ok = false` and the active cell cleared when the tray is done (fail with `require`). |
| `tray_reset` | mark every cell of the tray full again and make the first enabled cell (in `order`) the active one, so the run-per-part recipe needs no click after a reset; `<prefix>_row/_col/_ok`. The page's *All full* does the same. |

The run-per-part recipe this gives: the operator clicks the first cell once,
then every run is `move_to_active_cell` → the work → `tray_mark_used`, and the
next run carries on from the next full cell with nobody touching the grid.

Both movers take `mark_used` (default off): arriving at a cavity is not taking
the part from it, so a "go and look" step leaves the tray as it was, and a
sweep that does take parts says so on the step. When set, the arrival marks the
cell through the same two writes `tray_next` makes.

### The active cell

Runtime state on `Workflow` beside the cursor (`TrayActive` / `SetTrayActive` /
`ClearTrayActive`): the cell a recipe's `move_to_active_cell` drives to. Two
writers — a click on the grid (the page or the floating window) and
`tray_next`, which makes the cell it hands out active. A click also describes
the whole tray — "the work starts here": every cell before the pick in reading
order (row by row) is marked used, the pick and every cell after it full,
through `SetTrayCellsUsed`. So a run walks from the pick to the end and
nowhere else, and a complete tray STOPS: `tray_mark_used` on the last cell
clears the active cell, and the next `move_to_active_cell` fails on "no active
cell", which ends the run — and the program, since a failed recipe finishes
the program as Failed. Nothing executes further until an operator loads a tray
and clicks its first cell. (`require` on `tray_mark_used` moves that stop into
the same run, at the cost of the last part's run reading as failed.) Cleared with the cursor
(product load, All full / All empty, `tray_reset`). Not persisted: a restart
has no part in progress, and an operator picks one in a second. The click
stays live while a run is blocked on a prompt, which is what makes
`prompt "pick the cell, then OK"` → `move_to_active_cell` a usable recipe; the
marks and the disables on the same grid lock while `orch_.Busy()`, like every
other document edit.

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

Left: the grid (`App::DrawTrayGrid`) — a button per cell, green full, grey
empty (used), yellow active, dark disabled. A click picks the ACTIVE cell
(again: clears it); the right-click menu carries *Mark used* / *Mark full*,
*Drive here* (through `StartWatchedMove`'s pose overload; a cell has no uid;
`kRoleJog`) and *Disable* / *Enable*, so a stray click changes nothing about
the parts. Headers toggle a whole row or column; *All full* / *All empty* mark
the tray. Past 2500 cells the grid is not drawn and line toggles take its
place. Right: the tray list (New / Duplicate / Delete / Save), name, device,
size, the three corner cards (Teach here · or taught... · Go · values · date),
the pitch readout, the validation line.

**The floating Waffle Tray window** (`ui/windows/WndTray.cpp`, View menu,
`kRoleTray` = Operator) is the same grid — the very same `DrawTrayGrid`, so a
click means one thing in both places — with a tray combo, kept open beside the
Run page while parts come and go: pick the active cell, mark cavities, watch a
run take them. The page stays Technician because it teaches; the window is
Operator because selecting and marking are facts about the parts, and its
Drive here is behind the jog level like every move.

Three edits save on the spot instead of waiting for Save: teaching a corner,
changing the device, and marking cells used or full. All are facts about the
fixture an operator must not lose to a forgotten click.
