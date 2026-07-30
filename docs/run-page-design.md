# Run Page & Programs — Design (draft for approval)

Status: **approved — R1 implemented** (`command/Program`, `command/ProgramRunner`, Run page).
This fills the product manifest's reserved `programs` role and gives operators
a production surface: the **Run page**, driving **programs** = ordered decks
of **slots**, each slot an ordered bundle of recipes.

## 1. The model — program → slots → recipes

```
Program  { uid, name, description, slots: [Slot] }
Slot     { name, recipes: [recipe uid, ...] }        // ordered
```

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
    refuse if OrchestrationBusy();  resolve all uids up front (fail fast)
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
  (`OrchestrationBusy()` extended); a critical safety alert aborts the current
  recipe (existing) and the runner stops sequencing.

## 4. The Run page (View menu, full-page like the others)

Operator-facing: minimal reading, big controls, DUT-first flow.

```
+--------------------------------------------------------------------------+
| [Start/Stop]  scope label  [Save]  DUT serial                            |
+------------------+--------------------------------------+----------------+
| [program combo]  |  RUNNING: Align > XCAN fine          | DUT            |
| Deck             |  step 3.2 move_to_position ...       |  serial 2087   |
|  > Load     [>]  |  [########----------] 12/28          |  14 entries    |
|  v Align    [>]  |  program: slot 2/4, recipe 1/3       |  [Pass][Fail]  |
|      coarse  OK  |                                      |----------------|
|      xcan    ... |  (idle: slot/recipe summary,         | last results   |
|  > Cure     [>]  |   validation issues)                 |  align_best... |
|  > Inspect  [>]  |                                      |                |
+------------------+--------------------------------------+----------------+
```

- **Left — the deck**: program selector, then the slot list; each slot
  expandable to its recipes (names resolved from uids; missing uid in red).
  **Slots AND individual recipes are click-selectable** — the selection drives
  both the run buttons' scope and the middle panel's idle view. Per-slot
  **run button** (station-by-station operation); a selected recipe runs alone
  via Run Recipe. Live per-recipe status marks during any run (done /
  running / pending / failed), whichever granularity started it.
- **Middle — execution**: the running recipe's name, current step, recipe
  progress bar (engine status, as the recipe editor shows), plus program-level
  progress (slot i/N, recipe j/M). Idle: selected slot's recipe list +
  validation summary (every referenced recipe validated with the editor's
  author-time validation). Below it (post-R1): an **XCAN band** — bobcat's
  Run-page scan controls (hex-left / hex-right × Coarse 10,5,2,1 µm /
  Fine 2,1,0.5 µm presets over hexapod_align.lua, feedback-channel picker
  with live value, Stop, inline results) — and a **log band** (the shared
  Overview log viewer).
- **Right — DUT**: the same DutSession info + open/close controls as the
  Overview block (shared drawing code), because the operator flow is
  scan serial → run → pass/fail. Below it, a small ctx snapshot of the last
  finished recipe (align results etc.).
- **Top strip** (originally at the bottom): ONE **Start** button whose scope
  follows the deck
  selection — selected recipe runs alone, selected slot runs that slot,
  no selection runs the whole program (a disabled label beside the button
  names the scope; originally three separate Run Program/Slot/Recipe
  buttons, collapsed post-R1). While running it toggles to **Stop**, which
  propagates through the whole chain (queue → recipe engine → command
  waits → Lua sub-runs). Progress (recipe bar + program slot i/N, recipe
  j/M) displays identically for all three granularities — same runner,
  same status. Prompts surface via the existing global modal.
- **Light program editing on the page** (Phase R1, gated behind an "Edit"
  toggle so the operator view stays clean): new/rename/delete program, add/
  rename/remove/reorder slot, add (recipe combo)/remove/reorder recipes in a
  slot, Save. Dirty tracking + unsaved-changes guard reuse the recipe
  editor's pattern. A fancier drag-drop program editor can come later if
  needed — the model is trivial enough that list buttons suffice.

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
