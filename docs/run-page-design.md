# Run Page & Programs — Design (draft for approval)

Status: **approved — R1 implemented** (`command/Program`, `command/ProgramRunner`, Run page).
**Superseded in part** by `docs/program-organizer-design.md`: a slot is now a
TREE of nodes (recipe | folder with a loop count), the deck is READ-ONLY, and
every edit lives in the Program Organizer window. The editing bullets below are
kept as the history of why the shape moved.
This fills the product manifest's reserved `programs` role and gives operators
a production surface: the **Run page**, driving **programs** = ordered decks
of **slots**, each slot an ordered bundle of recipes.

## 1. The model — program → slots → recipes

```
Program  { uid, name, description, slots: [Slot] }
Slot     { name, recipes: [recipe uid, ...] }        // ordered   <- SUPERSEDED
```

*(A slot now holds `items: [Node]`, a tree — see
`docs/program-organizer-design.md`. The rest of this section still stands.)*

- A **slot** ("deck position") bundles recipes that belong together as one
  operator action — e.g. slot "Load" = [open gripper, move to load, close],
  slot "Align" = [coarse align, XCAN, verify], slot "Cure" = [UV on, wait,
  UV off]. Slots are ordered; recipes within a slot are ordered.
- A **program** gets a **uid at creation** (`GenerateUid`) — the stable
  identity that traceability records reference. Renaming changes only the
  label (the SavedPositions/recipes convention).
- Recipes are referenced by **uid** (rename-safe). A missing uid is a
  validation error shown on the page, not a silent skip.
- This is bobcat's Program level minus the DAG — the original Phase-A survey
  showed bobcat's DAG executed sequentially in practice, so FireDog uses an
  honest ordered list. Conditional flow belongs inside recipes (`if`/
  `stop_recipe`) or scripts, not between them.

## 2. Persistence

Same conventions as recipes:

- Product loaded: the manifest's `programs` role file, `{uid}.programs.json`
  (`App::SaveProduct` fills the placeholder; uid stamped, baseline for dirty
  tracking). No product: `config/programs.local.json` → `programs.json`.
- File shape: `{ "uid": ..., "programs": [ {uid, name, description,
  slots: [{name, recipes: [uids]}]} ] }`. Tolerant load.
- `core`-level model + load/save in `command/Program.{h,cpp}` (thin — no
  engine of its own).

## 3. Execution — the ProgramRunner (no new engine)

A program run is **a sequence of ordinary RecipeEngine runs** — one per
recipe, so every recipe keeps its own `$run` DUT event, its own validation,
progress, prompts, and stop behavior. The sequencing lives in a small
App-level state machine (`ProgramRunner`, ticked from `App::Update` — the
path-executor precedent):

```
Start(queue):   // ONE code path for all three granularities:
                //   Run Program -> every (slot, recipe) in order
                //   Run Slot    -> that slot's recipes
                //   Run Recipe  -> the single selected recipe
    refuse if orch_.Busy();  resolve all uids up front (fail fast)
Tick():
    if recipe engine Idle-after-run:
        Completed  -> advance to the next queue item; start it
        Failed     -> run FAILED at (slot, recipe); stop sequencing
        Stopped    -> run STOPPED
    when the last item completes -> COMPLETED
Stop():         // ONE stop, propagated:
    mark the queue dead (no further recipes start), then
    RequestStop() on the running recipe engine — which the engine already
    propagates into every blocking command wait and (via run_script) into
    any Lua sub-run. One button, the whole chain dies.
```

- Between recipes the runner starts the next engine run with a fresh Env
  (positions/maps/recipes snapshot taken once at program start — a program
  runs against one consistent world).
- **Ctx does NOT flow between recipes** (each engine run has a fresh ctx —
  existing behavior, unchanged). Data that must cross recipe boundaries goes
  through the DUT record or DataBus. Revisit only if real programs demand it.
- **Traceability**: at the end of a program or slot run (any terminal state)
  the runner writes a `$run` event `{kind:"program"|"slot", name, uid, result,
  recipes_done}` — `DutSession::RecordRun` gains the optional `uid`/details.
  A single-recipe run adds no extra event (the recipe's own `$run` already
  covers it). Combined, a DUT record shows the full production story: which
  program, which recipes, in what order, each result.
- Safety/mutual exclusion: the runner counts as the orchestration owner
  (`orch_.Busy()` extended); a critical safety alert aborts the current
  recipe (existing) and the runner stops sequencing.

## 4. The Run page (View menu, full-page like the others)

Operator-facing: minimal reading, big controls, DUT-first flow.

```
+--------------------------------------------------------------------------+
| [Execute/Stop] scope  Siphog888 [x]Edit  DUT 2087      [Save] [Load]    |
|  ^ green = ready, red = running                                          |
+------------------+--------------------------------------+----------------+
| [slot combo]     |  RUNNING: Align > XCAN fine          | DUT            |
| Deck             |  step 3.2 move_to_position ...       |  serial 2087   |
|  v 2  Align      |  [########----------] 12/28          |  14 entries    |
|      coarse  OK  |  slot: recipe 1/3                    |  [Pass][Fail]  |
|      xcan    ... |                                      |----------------|
|                  |  (idle: slot/recipe summary,         | last results   |
|                  |   validation issues)                 |  align_best... |
+------------------+--------------------------------------+----------------+
```

- **Left — the deck**: the **active-slot combo**, then that ONE slot expanded
  to its recipes (names resolved from uids; missing uid in red). The deck
  deliberately shows a single slot rather than the whole program: an operator
  is at one station, and a long program's other slots are noise they can
  mis-click. The slot row and individual recipes are click-selectable — the
  selection drives both Execute's scope and the middle panel's idle view.
  Live per-recipe status marks during a run (done / running / pending /
  failed), whichever granularity started it. **Recipe rows are 2.25 text-lines
  tall** (`kDeckRowScale`, scaled off the text line so the font picker keeps
  working): this is the list an operator picks from between parts, often with a
  glove on, so the row is a touch target first and a line of text second. The
  row carries an empty label and draws its name centred separately — a
  Selectable renders text at the top-left of its box, which in a tall row
  leaves the name floating above the band it names — and the hover/right-click
  results are captured immediately after the Selectable, before the label
  becomes the current item.
- **The (program, slot) pair persists** (`UiSettings run_prog_uid` /
  `run_slot`, program by UID so a rename keeps it) and is re-applied on every
  product load that still contains that uid. An operator working one station
  across a shift comes back to that station after a restart, not to the top of
  the program. `App::SelectDeckProgram` / `SelectDeckSlot` /
  `RememberDeckSelection` / `RestoreDeckSelection` are the only writers —
  nothing else assigns `progSel_`/`deckSlot_`.
- **Middle — execution**: the running recipe's name, current step, recipe
  progress bar (engine status, as the recipe editor shows), plus program-level
  progress (slot i/N, recipe j/M). Idle: selected slot's recipe list +
  validation summary (every referenced recipe validated with the editor's
  author-time validation). Below it (post-R1): the **Scan band** — one shared
  header (feedback-channel picker with live value, Trace) over a tab per scan
  method, in the order the machine uses them: **Raster** (first light,
  raster_scan.lua), **Climb** (close the gap, climb_raster.lua — see
  `docs/climb-raster-design.md`) and **XCAN** (the peak, hexapod_align.lua).
  Each tab is hex-left / hex-right × launch presets, right-click to edit, with
  inline results. The XCAN and Climb tabs share one **Target** row (the
  product power classes; the active slot is injected into every launch), and
  the XCAN row ends in **MAX**: no exit threshold at all — the ladder runs
  every rung with the adaptive zones off and parks on the best point, the
  manual maximisation for a part whose class is not known yet. MAX is a mode
  over the slot, not a slot: a click on any target (or a Quick launch, which
  names one) turns it off, and Climb, whose target is mandatory, refuses to
  run under it and says so. Its height is dragged by the splitter above it and persisted
  (`runScanH`), because the tabs carry parameter forms and a fixed row count
  crops whichever tab needs the most. Below that, a **log band** (the shared
  Overview log viewer).
- **Right — DUT**: the same DutSession info + open/close controls as the
  Overview block (shared drawing code), because the operator flow is
  scan serial → run → pass/fail. Below it, a small ctx snapshot of the last
  finished recipe (align results etc.).
- **Top strip** (originally at the bottom): ONE **Execute** button whose scope
  follows the deck selection — selected recipe runs alone, otherwise the shown
  slot runs (a disabled label beside the button names the scope; originally
  three separate Run Program/Slot/Recipe buttons, collapsed post-R1). While
  running it toggles to **Stop**, which propagates through the whole chain
  (queue → recipe engine → command waits → Lua sub-runs). Progress (recipe
  bar + recipe j/M) displays identically for both granularities — same runner,
  same status. Prompts surface via the existing global modal. The strip's
  controls are **twice frame height** so a gloved hand finds Execute/Stop
  without aiming, and the button is **colour-coded: green ready, red running**
  — the colour is the part read from across the bench, since the label is only
  legible once you are standing at the screen and by then you already know.
  Disabled-green (nothing selected, or another engine busy) dims through
  `BeginDisabled`, so "not ready" never looks like "go"; it is deliberately the
  only saturated colour in the strip. It was briefly moved down beside the
  progress bar and moved back: the strip is where the operator's hand already
  goes. Beside it live **Save**, a **Load** button opening the program list as
  a modal, the loaded program's name, the **Edit** toggle and a rejected Start
  (`runStartErr_`, which belongs next to the press that produced it) — which
  program is up and whether it can be edited are questions about the whole
  page, so they belong in the strip the operator already reads, not buried in
  the deck column.
- **There is NO whole-program scope** (`RunScope` is `Slot | Node`, enforced
  in `BuildRunQueue`, not just hidden in the UI). A multi-slot program is a
  station sequence an operator walks — part in, run this slot, part moves on —
  so one press launching every slot back-to-back would drive the machine
  through stations whose parts have not arrived. Removing the scope from the
  resolver rather than the button is what keeps a later UI from re-offering it.
- **Light program editing on the page** (Phase R1) — *superseded*. It was
  buttons growing inside the deck behind an "Edit" toggle. They are gone: the
  deck is read-only and "Edit" now opens the **Program Organizer** window
  (`docs/program-organizer-design.md`). The reasoning that killed it is the same
  one that made the rows three lines tall — the deck is what an operator reads
  while running a station, and an editing control in it is one mis-click away
  from changing the program under a part already in the machine. Save, dirty
  tracking and the unsaved-changes guard are unchanged.

## 4b. The image box (added 2026-08-21)

Top of the right column, above the DUT block, because that is the order the
operator works in: **look at the part, read what it says, then type the serial
and open the record.**

One still, not a stream. The Aux window already shows what the camera sees right
now; this shows what it was *pointed at when a step said so*, and it stays put
while the stage drives on to the next station. A live view cannot answer the
second question, which is the whole reason the box exists — the motivating case
is driving the camera to a serial-number label, capturing, and letting the
operator read it off a picture that no longer moves.

Two commands fill it:

| | Requires a DUT | Writes a file | For |
|---|---|---|---|
| `capture_view` | no | no | look at something |
| `dut_capture_image` | yes | yes | file it as evidence |

`capture_view` deliberately does **not** require an open DUT. Reading the serial
is how you learn *which* DUT to open, so a command that failed without one would
be useless for exactly the job it exists to do. `dut_capture_image` fills the
same box as a side effect after its record write succeeds: the frame is already
paid for, and the operator watching the run is the one person who can notice
that the camera photographed the wrong thing.

### The hand-off is a slot, not a queue

`Workflow::CapturedView` is **one** parked frame, latest wins. A 5 MP RGB frame
is ~15 MB, the box can show exactly one, and the one worth showing is the last
one asked for — a queue would hold megabytes of pictures nobody will ever see.
The UI *takes* the frame (moves the pixels out) rather than copying it: the
bytes exist only to become a texture, and the GPU has them afterwards.

Same worker→UI rule as every other hand-back: the command runs on an engine
worker and `EnsureTextureRGB` is a GL call, so `DrainCapturedView` runs in
`App::Update`.

**Its position in `App::Update` is load-bearing.** It sits *after* the DUT-open
edge detector, which clears the box for the same reason it drops live frames — a
still shows one physical part. A recipe that does `dut_open` and then
`dut_capture_image` can land both inside one frame, and draining first would let
the open wipe the picture the capture just delivered.

The box height is a dragged, persisted splitter (`run_shot_h`) rather than a
constant: how tall it has to be to read a serial depends on the lens, not on the
layout. Hovering the picture shows it at 640 px in a tooltip.

## 5. Implementation order

1. **R1 — model + runner + page**: `command/Program.{h,cpp}` (model, load/
   save, validation), `ProgramRunner` (App-level, ticked from Update),
   `DutSession::RecordRun` uid/details extension, the Run page with deck,
   execution view, DUT panel, run controls, and light editing. `SaveProduct`
   fills the `programs` role.
2. **R2 (usage-driven)**: per-slot on-failure policy (abort program vs skip
   to next slot), operator lockout of Edit mode, run-history panel (from the
   DUT viewer's `$run` extraction), auto-open-DUT prompt at program start.

## 6. Open questions (decide at review)

1. **Run Slot scope** — drafted: runs ONLY the selected slot (station
   operation). Alternative: "run from this slot to the end". Both are cheap;
   drafted slot-only + a "Run from here" as R2 if wanted.
2. **Failure policy v1** — drafted: any recipe Failure stops the program
   (loudly, at the failing slot/recipe). Per-slot continue-on-fail is R2.
3. **Menu placement** — drafted: View menu with the other full pages. If the
   Run page becomes the operator's home screen, promoting it to a top-level
   menu entry (or startup page) is a two-line change later.
