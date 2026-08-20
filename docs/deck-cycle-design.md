# Deck cycle — start/end states and per-node status

Status: **implemented** (`ProgramSlot::start/endNodeUid`, `App::DrainDeckMarks` /
`AdvanceDeckCycle` / `DeckCycleRange` / `DeckMarkOf`, deck accents, organizer
right-click).
Extends `docs/run-page-design.md` and `docs/program-organizer-design.md`.

An operator running a part today presses Execute, reads the log to see whether
it passed, picks the next recipe by hand, and remembers where they got to. This
turns the slot into a **cycle**: two of its nodes are marked *start* and *end*,
each finished node carries a green or yellow accent, and pressing Execute on the
start node wipes the accents for the next part.

It adds **no engine, no thread, and no change to `ProgramRunner` or
`BuildRunQueue`.** Every part of it is deck state and one drain in `App::Update`.

---

## 1. The model — two uids on the slot

```cpp
struct ProgramSlot {
    std::string name;
    std::vector<ProgramNode> items;
    std::string startNodeUid;   // "" = no cycle defined
    std::string endNodeUid;
};
```

JSON: `"cycle_start"` / `"cycle_end"` on the slot object, both optional, absent
when empty. Tolerant load like everything else.

- **On the slot, not the program.** The deck shows one slot, Execute's scope is
  one slot, and a cycle is what an operator walks at one station. A program-level
  cycle would have to cross stations that different people are standing at.
- **Uids, not indices** — the same rule the rest of the tree runs on. A reorder
  in the organizer must not silently move the start state.
- **Both must be TOP-LEVEL nodes.** A start buried inside a folder is not a
  thing that can be run on its own: the folder is the unit Execute launches. So
  the cycle is resolved as two indices into `slot.items` and nothing deeper, and
  a uid that no longer resolves at the top level makes the cycle **undefined**
  rather than half-defined.

### What "in the cycle" means

The cycle is the run of top-level nodes from `startNodeUid`'s index through
`endNodeUid`'s index, **inclusive**. Nodes above the start and below the end —
`Init SMU and TEC`, `Initial Machine`, the `Manual Troubleshooting` folder — are
not chain members: nothing auto-advances to or from them. They can still be run,
and they still get accents when they are.

An `end` that sits **above** `start` is not a wrapping cycle, it is a mistake:
the cycle reads as undefined and the organizer says so.

---

## 2. Status — runtime only, keyed by node uid

```cpp
enum class NodeMark { None, Passed, Failed };
std::map<std::string, NodeMark> deckMarks_;   // App member
```

**Never written to the program file.** A status describes one physical part, so
it belongs to the session exactly the way a live frame does — and the same rule
applies for the same reason: a saved green tick would be a record of a part that
has since left the machine, read as a licence to skip a step.

Filled by one drain in `App::Update`, right after `orch_.programs.Tick()`:

```cpp
for (const ProgramRunner::ItemStatus& it : orch_.programs.GetStatus().items) {
    if (it.mark == Mark::Done)                             deckMarks_[it.nodeUid] = Passed;
    else if (it.mark == Mark::Failed || it.mark == Stopped) deckMarks_[it.nodeUid] = Failed;
}
```

Three things fall out of that being *the whole implementation*:

- **Accents appear live**, mid-run, not only at the end — the runner already
  marks each item as it completes.
- **Loops behave.** A `x100` folder puts the same node uid in the queue a
  hundred times; the drain walks in queue order and `Pending` writes nothing, so
  a pass 3 failure that stopped the run leaves the node yellow rather than being
  overwritten by the passes that never happened.
- **A Stop is yellow.** Not-passed is the useful distinction to an operator
  looking at a part; "how it failed to pass" is what the log band is for.

**A folder's accent is derived, not stored.** Only recipe leaves become queue
items, so a folder shows green when every leaf beneath it passed, yellow if any
failed, and nothing otherwise. Deriving it means a folder can never disagree
with its contents.

### What clears it

Only **running the start node** — `deckMarks_.clear()` in `App::StartDeckRun`,
beside the existing `ClearLiveFrames("program start")`, which is already the
place this kind of boundary lives and already carries the comment saying why it
is in App and not in layer 2.5.

Two presses qualify, and only these two:

- Execute with the **start node** selected.
- Execute with **slot scope** — a slot run necessarily contains the start node.

Nothing else touches the map. Not a page switch, not a restart, not a new DUT.
That is deliberate and it is what was asked for: the accents are the operator's
record of where this part got to, and the only thing that means "new part" is
them starting the cycle again.

---

## 3. Advance on pass, hold on fail

The runner going idle is edge-detected in `App::Update`, the same shape as the
existing `dutOpenKey_` detector:

```cpp
const bool active = orch_.programs.Active();
if (runWasActive_ && !active) AdvanceDeckCycle();
runWasActive_ = active;
```

`AdvanceDeckCycle()` moves the selection only when **all** of these hold:

| Condition | Why |
|---|---|
| the run was **node** scope | a slot run already ran the whole chain; there is nowhere to advance to |
| `Status.result` starts with `Completed` | a failure must leave the selection on the failing step, so the same press retries it |
| the node that RAN is a top-level cycle member | running `Manual Troubleshooting` mid-cycle must not move the operator's place |

**The node that ran, not the current selection.** `StartDeckRun` records
`runNodeUid_` at launch. The deck stays clickable during a run, so reading
`deckNode_` afterwards would advance from wherever the operator happened to
click while watching.

At the **end** node, the selection wraps back to **start** — ready for the next
part, with the finished cycle's accents still on screen until that start press
clears them.

### The cycle never blocks a run

Execute still runs whatever is selected, in any order, cycle or not. This is an
aid, not an interlock: an operator who needs to re-run one step, or drop into
Manual Troubleshooting halfway, must not be fought by the UI. The armed-checkbox
rule covers actions that can damage the machine; running a recipe out of order
is not one of them.

---

## 4. What it looks like

```
  1  Production
 ┃                                          <- accent gutter
 ┃   Init SMU and TEC
 ┃
 ┃   Initial Machine
 ┃
▐█   Probe SLED and PIC              start ✓   green
▐█   PP Left Lens SIPHOG                   ✓   green
▐█   PP Right Lens SIPHOG                  ✗   yellow, SELECTED  <- Execute retries
 ┃   UV                                end
 ┃ ▸ Manual Troubleshooting
```

- **A colour bar down the left edge of the row** is the part read from across
  the bench — same argument as Execute's green/red. The row is already three
  text-lines tall and already draws its own label, so the bar and the mark are
  two `AddRectFilled` / `AddLine` calls in `DrawDeckNodes`.
- The tick and cross are **drawn**, not typed: the deck font is a fixed
  monospace and a missing glyph would silently become a box. The execution
  list's `OK` / `X` text stays as it is — it is scanned, not glanced at.
- `start` / `end` are dim right-aligned captions on their rows. No gutter
  bracket spanning the range: it would have to route around folders, and the two
  captions answer the same question.
- The two colours are file-scope constants in `PageRun.cpp`, next to the gold
  running row and the red missing one — **not** `ui/Theme`. Theme owns how the
  app is styled; these say what a RUN did, which is the page's business.

### Setting the two states — in the organizer

Right-click a **top-level** node → `Set as cycle start` / `Set as cycle end` /
`Clear cycle`. The items are hidden on nodes inside a folder, per §1.

Under the organizer's toolbar, one line naming the current cycle and a Clear
button:

```
cycle:  Probe SLED and PIC  ->  UV                        [Clear]
cycle:  end is above start - not a cycle                  [Clear]   (red)
cycle:  start node no longer in this slot                 [Clear]   (red)
```

The organizer is the only surface that writes these, exactly as it is the only
surface that writes anything else about a program.

---

## 5. Files

| File | Change |
|---|---|
| `command/Program.h/.cpp` | two uids on `ProgramSlot`, `cycle_start`/`cycle_end` JSON |
| `App.h` | `deckMarks_`, `runNodeUid_`, `runWasActive_`; `DrainDeckMarks` / `AdvanceDeckCycle` / `DeckCycleRange` |
| `App.cpp` | `Update`: the drain + the run-finished edge |
| `ui/pages/PageRun.cpp` | `StartDeckRun` (clear + record); `DrawDeckNodes` (bar, mark, captions); folder-derived status helper |
| `ui/windows/WndProgramOrganizer.cpp` | right-click set/clear, the cycle line |
| docs | this file; a line in `CLAUDE.md`; cross-refs in the two Run docs |

No changes to `ProgramRunner`, `BuildRunQueue`, `RecipeEngine`, or any file
below layer 3 other than the two model uids.

---

---

## 5b. The Values panel — what the part measured

Recipes write measurements into their run ctx (`keithley_read` → `DarkCurrent`,
`read_topic` → `smu_ch1`, `prompt_input` → `serial`, the UV recipe's
`DRY_VALUE` / `UV_VALUE1` …). Those values are the part's story, and until now
they were visible only in the log.

Two problems had to be solved, both of them lifetime problems:

1. **`Status.ctx` is empty by the time a run is `Completed`.** The engine drops
   its `runCtx_` pointer before the state goes terminal. So `RecipeEngine` now
   takes `ctx.CtxSnapshot()` into **`Status.lastCtx`** at that exact boundary,
   under the same lock. Polling `.ctx` from the UI instead would drop a value
   written by the recipe's final step, since a run can end between two frames.
   `lastCtx` is **display only** and cleared at `Start`; a command handing data
   back still goes through a `Workflow` queue.
2. **Ctx does not flow between recipes.** Each run gets a fresh one, which is
   deliberate and stays that way. The accumulation is the UI's job:
   `App::DrainCycleVars` merges each finished run's `lastCtx` into
   `cycleVars_` on the **engine-idle edge** — one recipe finishing, not one
   queue finishing.

That edge is why the drain sits **before `orch_.programs.Tick()`** in
`App::Update`: the runner starts the next queue item the instant it sees the
engine idle, so a drain placed after it would catch only the last recipe of a
slot run.

`cycleVars_` is cleared on the **same boundary as `deckMarks_`** — running the
start node. The ticks and the numbers must never describe two different parts.

The panel lives in the Run page's right column, under the DUT block it belongs
beside, with the engine line pinned below it so a long list cannot push that off
the panel. A running recipe's live keys are merged over the accumulated ones and
drawn gold, so there is one list to read rather than two, and the recipe that
wrote each value is a hover tooltip rather than a third column in a narrow
panel.

**Engine scratch is filtered out** (`IsOperatorCtxKey`): `align_*`, `place_*`
and anything starting with `_`. The alignment and placement scripts push a dozen
bookkeeping values through the same ctx a recipe measures into, and burying
`DarkCurrent` under them makes the panel useless at a glance. The underscore is
the escape hatch for a recipe author who wants a key kept out of the operator's
sight.

This is **not** traceability — `dut_record` is, and the UV recipe already uses
it. The panel is the operator's live view of the part in front of them.

---

## 6. Open, and deliberately deferred

1. **DUT coupling.** The cycle finishing is, in practice, the part finishing —
   so `DutSession::Close("pass"|"fail")` is a natural thing to prompt at the
   wrap. Left out of v1 because it ties the deck to the DUT lifecycle, and the
   answer to "was the part good" is not always "did every recipe complete".
2. **Cycle-scoped Execute** ("run start→end in one press"). This is the
   whole-program scope argument again: the chain exists because the operator
   does something between steps. If it is ever wanted, it is a slot run with a
   different range, not a new code path.
3. **Per-node retry counts** (how many times this part failed here) — cheap to
   add on top of `deckMarks_`, worth nothing until somebody asks for the number.
