# Cycle time — every run timed silently, an ETA from the runs before it

## 1. What it is for

A recipe's progress bar counted LEAF STEPS (`leafDone / leafTotal`) and a program's
counted QUEUE ITEMS. Both are wrong in the same way: a `log` step and a 40-second
`xcan` are one step each, so the bar reads 90 % with most of the time still ahead.
And nothing remembered how long a run took, so nobody could say what the station's
cycle time is, or whether it is getting slower.

So every real run is timed, without asking, and the time is kept per recipe. The
next run of the same recipe uses that history to show where it is in TIME and when
it will be done:

```
3 / 12   0:42 of ~1:30            <- the bar is time, the overlay still counts steps
ETA 0:48 - machine 1:22-1:31 (7 runs) - operator 0:05
```

It is an AID, exactly like the deck cycle: nothing refuses, nothing waits, nothing
moves on it.

## 2. What is measured — machine time and operator time, apart

The one thing that would ruin it: an operator who takes five minutes over a prompt.
Counted into the run, one coffee break moves the median and every ETA after it lies.
So a run's wall clock is split in two:

- **operator time** — the engine is blocked on a person:
  - every handshake the engine owns: `prompt` / `prompt_input` / a choice (incl.
    one with auto-OK — a prompt nobody answers is still a prompt), `manual_adjust`,
    the crosshair pick, and the step / Pause gate (`AwaitStepInternal`, only while
    it actually holds);
  - the two commands that wait on a person through `Workflow` rather than a hook:
    `notify_operator` (until OK or its timeout) and `xcan_operator` (while it waits
    for a press or Done — NOT while a scan it was asked for runs). Both bracket the
    wait with `ExecutionContext::OperatorWaitScope`, which calls the `operatorWait`
    hook (LAST in `Hooks`; the Lua engine leaves it empty).
- **machine time** = wall − operator.

The engine keeps the bracket itself (`waitSince_` / `waitAcc_`, under its mutex, with
a depth count so nested brackets cannot double-count), so the split is exact to the
clock, not sampled per frame.

A new command that waits on a person through anything but the hooks must take an
`OperatorWaitScope` around that wait, or its operator time is booked as machine time.

## 3. Which runs are recorded

Only a run that says something about how long the recipe takes:

| run | recorded |
|---|---|
| Completed, launched from the Recipe page or as a program/slot queue item | yes |
| Failed / Stopped | no — a partial run is not a cycle |
| teach run, dry run | no — same rule as `$run`, the deck marks, `record_position` |
| started from a step (Run from here) | no — the part before it never ran |
| step mode / Pause | yes — the gate is operator time, the steps are real |

The Lua engine (`fd.*` scripts launched on their own) is not timed: it has no step
structure to estimate against. A script run INSIDE a recipe is part of that recipe's
time.

## 4. Which runs are comparable — the recipe VERSION

A recipe that was edited is a different recipe as far as time goes. Every run carries
`hash` = FNV-1a 64 over the INLINED steps (`RecipeStepsHash`, `command/Recipe.h`):
`StepToJson` of every step with `nickname` dropped — a rename of a row is not a change
of what runs, a param, an `enabled` flag, an `on_error` or a sub-recipe's body is.
Because it is taken after `InlineSubRecipes`, editing a sub-recipe re-versions every
recipe that calls it.

The estimate uses only runs with the SAME uid AND hash. An edited recipe therefore
starts "learning" again, which is right: the old numbers describe steps that no longer
exist.

## 5. The file

```
<data root>/products/<product>/data/cycle_times/<recipe uid>.jsonl
```

Production output under the product's `DataDir()` (as `position_records/`), reached
through `DutSession::ProductDataDir()`. One file per recipe, so an estimate reads only
its own history. One JSON line per run:

```json
{"t":1794470400,"ts":"2026-11-14T14:20:05","uid":"r_3f9a","name":"align lens 1",
 "hash":"9c1e0f3b77a2d401","wall_ms":91234,"wait_ms":5120,"machine_ms":86114,
 "steps":12,"serial":"SN0001","operator":"1234"}
```

`serial` is the DUT open when the run ENDED (else the one open when it started; `""`
none). `operator` is the signed-in employee number when App wrote the line.

**One writer: `App::UpdateCycleTimes`.** The engine pushes a `RunTiming` into its own
mutex-guarded queue at the terminal boundary (`RecipeEngine::TakeRunTimings`, capped);
`App::Update` drains it and appends — the `TakeSavedPositions` shape. The worker never
touches the file. A failed write is logged and dropped: a missing history line must
never fail a run that already completed.

## 6. The estimate

`CycleTimeStore` (`core/CycleTimes`) is UI-thread only, like `OeeTracker`. It follows
the product's data dir (checked every frame — a string compare), loads a recipe's
file lazily the first time it is asked for (the last 256 KB, ≤ 500 runs kept), and
caches each `(uid, hash)` estimate until the next append.

`EstimateCycle` over the last **20** matching runs: `n`, the MEDIAN machine time, the
median operator time, and the p10–p90 machine range (min–max below 5 runs). A median
because retries, `if` branches and a scan that climbs three times are real and
occasional; a mean lets one of them steer the next fifty ETAs.

**A recipe run** (`App::cycleEta_`, computed in `Update`, Draw only reads it):

- `frac` = machine elapsed / median machine, capped at 0.99 — the bar FREEZES while
  the operator is being waited on, because no work is being done;
- remaining = max(0, median machine − machine elapsed) + max(0, median operator −
  operator so far) — the typical operator time is still expected, a long one is not
  projected forward;
- past the median the line says `+0:12 over typical` instead of an ETA of zero.

No history for this version → the bar falls back to steps and the line says
`learning`. A teach or dry run shows elapsed time only.

**A program / slot run** sums the queue: done items at their ACTUAL time (kept per
item in `ProgramRunner::ItemStatus`), the running item as above, every pending item at
its median machine + operator time. Each queue item is hashed once at
`ProgramRunner::Start` (`RecipeRunHash`, one inline per distinct recipe) so pending
items find their history. A pending item with no history makes the total a lower
bound (`ETA ≥ 2:10, 2 recipes not timed yet`) and the program bar falls back to
items. The queue sums are recomputed when the item index or the store changes, never
per frame — a ×1000 folder is a 1000-item queue.

## 7. Display

- **Recipe Editor**: the run bar is time-based when there is history; overlay still
  `done / total` steps plus `elapsed of ~typical` (typical = median machine + median
  operator); one line under it (ETA, the p10–p90 machine range and run count, operator
  time so far / "waiting for operator"). After the run: `last run 1:28 (operator 0:05)`.
- **Run page**: a queue bar (`recipe 3 / 8   2:10`, machine-time when every item has
  history, else items) and `cycle ~3:40 - ETA 1:30 - operator 0:32`, then the running
  recipe's bar and line as above. The finished result says `took 3:42 (operator 0:32)`.
- Code: `ui/RunTiming.cpp` (`UpdateCycleTimes`, `DrawRecipeTimeLine`,
  `DrawProgramTimeLine`, `RecipeRunBarFrac/Text`).

## 8. What it is not

- Not an interlock and not a judge: a run that is slower than typical is not failed,
  warned or stopped.
- Not OEE: OEE asks whether the station was running; this asks how long one recipe
  takes when it runs. They share no state.
- Not a second result channel: the timing crosses threads only through
  `TakeRunTimings`, never through `Status.lastCtx`.

## 9. Later (not built)

- **Per-step weighting**: time each leaf by its step path and weight the bar by the
  median duration of the steps done. Better inside one long recipe; needs a per-step
  file shape. Build when a whole-recipe ETA proves too coarse on the machine.
- A cycle-time view (trend per recipe, the slow step) beside Position Records.
- Slot-level records (the station takt time) — today it is the sum of its recipes.
