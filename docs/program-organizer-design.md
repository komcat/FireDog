# Program Organizer & folders — Design

Status: **implemented** (`command/Program` node tree, `BuildRunQueue` expansion,
`ui/windows/WndProgramOrganizer.cpp`, the Run deck turned read-only).
Extends `docs/run-page-design.md`, which still describes the Run page itself.

A program's slot used to be a **flat list of recipe uids**, edited by buttons
that grew inside the operator's deck whenever *Edit* was ticked. That was enough
to order a station's recipes and nothing more. This adds the two things a real
production deck needs — **repetition** and **grouping** — and moves every edit
out of the deck into a window of its own.

---

## 1. The model — a slot is a TREE

```
Program  { uid, name, description, slots: [Slot] }
Slot     { name, items: [Node] }
Node     = Recipe { uid, recipeUid }
         | Folder { uid, name, loop, alignProcUid, items: [Node] }
```

- **A folder is the unit of repetition.** `loop` is how many passes over its
  contents. There is deliberately **no "repeat rows 1–3" range loop**: a range is
  a second way to say what a folder already says, and it breaks the moment
  somebody reorders a row. To loop recipes 1–3 you put them in a folder and set
  its count — which also gives the repetition a *name*, so the deck can say
  `Left Lens x100` instead of `rows 1-3, 100 times`.
- **Every node carries its own uid**, minted at creation (`MakeRecipeNode` /
  `MakeFolderNode`). That uid is the identity the organizer selects by, the deck
  highlights by, and the runner marks against. An **index** cannot do any of
  those jobs any more: an index into a tree means nothing the moment a folder
  above it opens, closes, or is reordered.
- Recipes are still referenced by **uid** (rename-safe), and a missing uid is
  still a validation error shown in red, not a silent skip.
- `alignProcUid` is **reserved and inert** — see §5.

### Persistence

Same file, one new shape. A slot writes `items` (nodes); the loader prefers
`items` and falls back to the pre-folder `recipes: [uid, ...]`, turning each
string into a recipe node. That fallback stays **forever**: saved programs are
production documents, and reading the old shape costs one branch against a file
nobody can re-author from memory.

```json
{"name": "Production", "items": [
  {"kind": "recipe", "uid": "…", "recipe": "…"},
  {"kind": "folder", "uid": "…", "name": "Left Lens", "loop": 100, "items": [ … ]}
]}
```

---

## 2. Loops are expanded at Start, not at run time

`BuildRunQueue` walks the tree and emits **plain queue items**, multiplying each
folder's `loop` through as it goes. `ProgramRunner` never learns what a folder
is: it stays a flat sequence of ordinary `RecipeEngine` runs.

That is the whole point. A loop that lived inside the runner would be a second
execution path with its own stop behaviour, its own progress arithmetic and its
own traceability story — three chances to disagree with the one that already
works. Expanded up front, `Stop` still kills the chain at the first recipe
boundary, progress is still `recipe i / N`, and every pass still writes its own
`$run` event.

The cost is honest and bounded:

- The queue length **is** the expanded length, so `kMaxQueueItems` (20 000)
  refuses a mistyped loop count **before the machine moves** rather than after
  forty minutes of it.
- `kMaxDepth` (8) refuses a hand-edited file nested deep enough to recurse the
  UI thread off its stack.
- The organizer prints the expanded number next to every folder and at the foot
  of the slot (`… expanding to 1 200 recipe run(s)`), because *how many* is the
  question an operator has before pressing Execute on a `x100`.

Each item carries `path` (the folder breadcrumb), `pass` and `passes` — the
**innermost** enclosing loop, which is the one being counted by whoever is
watching. The deck prints `pass 7/100` on the folder whose direct child is
running; the execution list prints `Production / Left Lens [7/100] / PP Left`.

### Run scopes

`RunScope` is now **`Slot | Node`** (it was `Slot | Recipe`), and the deck
selection is a node uid rather than an index. Selecting a **folder** and pressing
Execute runs that folder *including its loops* — the only way to run one repeated
group without running the slot around it. `kind` in the `$run` event follows:
`"slot"`, `"folder"` or `"recipe"` (the last still writes no extra event, since
the recipe's own `$run` covers it).

There is still **no whole-program scope**, for the reason `run-page-design.md`
gives: a multi-slot program is a station sequence an operator walks.

---

## 3. The organizer is a WINDOW, and the deck is read-only

*Edit* used to sprout `^ v x` buttons beside every recipe in the deck and a
`+ recipe` combo underneath. Two problems with that, and they are the reason
this exists:

1. **The deck is a running instrument.** It is what an operator reads between
   parts, with three-line-tall touch targets. An editing control living in it is
   one mis-click away from changing the program under a part already in the
   machine.
2. **A tree does not fit in a 300-px column.** Folders, loop counts, a recipe
   library to pick from and a place to drop things need room the deck does not
   have and should not take from the execution view.

So the deck is now **read-only** — it renders the tree, selects a node, and
right-click still opens a recipe in the Recipe Editor — and *Edit* opens the
**Program Organizer**, its own ImGui window (multi-viewport: drag it to a second
monitor and reshape a program while watching the deck it will drive).

```
+ Program Organizer ------------------------------------------------+
| [New] [Delete]  program [Siphog888____]  uid: da744c55            |
| 1st slot: [+ Slot] [Dup Slot] [^][v][x Slot] [x] 2nd panel                    |
+------------------------+------------------------------------------+
| Recipes                | [+ Folder] [^][v][>][<][Remove]          |
| [filter______]         | folder [Left Lens___]  loop [100]        |
|  Init SMU and TEC  add |------------------------------------------|
|  Initial Machine   add |     Init SMU and TEC                     |
|  PP Left Lens …    add | [+] Left Lens   x100   (300 runs)        |
|  Reject Left       add |     PP Left Lens SIPHOG                  |
|  …                     |     Reject Left                          |
|                        |     LEFT HOME                            |
|                        |     UV                                   |
+------------------------+------------------------------------------+
| 5 item(s) in this slot, expanding to 302 recipe run(s).           |
| [Save*] [Close]   Edits live in memory until Save.                |
+-------------------------------------------------------------------+
```

- **The Run strip's Edit checkbox, the View menu entry and the window's close
  box are ONE bit** (`runEdit_` mirrors `orgOpen_`). Three ways in, never two
  answers about whether it is open.
- **The organizer's slot selection is its own** (`orgSlot_`, seeded from the
  deck when the window opens). Reorganising slot 3 must not drag the operator's
  deck off the station they are standing at.
- **Two ways to place, on purpose.** Double-click (or `add`) inserts at the
  selection — inside it if a folder is selected, after it otherwise — which is
  what makes clicking down the library build a slot in order. Drag-and-drop
  aims: onto a folder row it goes **inside**, onto a recipe row it lands
  **after**, onto the tail strip it appends at the top level. `> / <` are
  indent/outdent for anyone who would rather not drag.
- **A folder always moves with its children.** Every edit goes through the
  `command/Program` tree helpers (`ExtractNode` / `InsertAfter` / `InsertInto` /
  `MoveNodeUp|Down` / `Indent` / `Outdent`) rather than being hand-rolled in the
  UI: "move this row" is only ever correct if the subtree comes with it.

### Two panels, one program (September 2026)

The tree column became **two**: the *1st panel* is the slot being organized
(the top row's `+ Slot / Dup Slot / ^ v / x Slot` act on it; the
library's `add` and double-click land in whichever panel its "add to" radio names, the 1st by default), and the *2nd panel* shows any slot of
the same program — or the same slot again — with an **aisle** between them:

```
| Recipes      | 1st: [1  PROD Tall A v]     |        | 2nd: [2  PROD Tall B v] |
| …            | [+ Folder][^][v][>][<]      | Copy > | [+ Folder][^][v][>][<]  |
|              | [Remove][Cut][Copy][Paste][Rename] | Move > | [Remove][Cut][Copy][Paste][Rename]
|              | cycle: …                    |        | cycle: …                |
|              |     Init SMU and TEC        | < Copy |     Probe SLED          |
|              | [+] Left Lens x100 (…)      | < Move |     UV                  |
```

- **Each panel owns its slot and its selection** (`orgSlot_`/`orgSel_`,
  `orgSlot2_`/`orgSel2_`), and every helper takes `panel` (0 or 1) rather than
  a slot reference alone, so the two selections, the two drop targets and the
  clipboard never disagree about which tree they mean. A drop is recorded
  with its panel and applied by that panel.
- **The aisle moves or copies the selection across**, landing by the same
  rule a paste does (inside a selected folder, after a selected row, else at
  the end). *Move* keeps the node's uid — it is the same node, elsewhere — and
  clears it from the source slot's cycle states; *Copy* lands a clone. A drag
  from one panel into the other is the same act as *Move*. The same slot on
  both sides is allowed: it is then a second view, and Move is an in-slot move.
- **Cut / Copy / Paste** on either toolbar work a one-subtree clipboard
  (`orgClip_`). Paste inserts a **clone**, so one clip can be pasted twice.
- **Every copy goes through `CloneNode` / `CloneSlot` / `CloneProgram`**
  (`command/Program`), which mint **fresh uids all the way down**: a uid is an
  identity (selection, run marks, the cycle states, the DUT's program
  reference), so a copy that kept them would be two nodes answering to one
  name. `CloneSlot` re-points its cycle uids onto the copies; `CloneProgram`
  mints a new program uid. *Duplicate* (program) and *Dup Slot* are these two,
  and the copy becomes what you are looking at. `+ Slot` asks for the new slot's name in a popup; *Rename* on each panel's toolbar renames that panel's slot the same way (the top row carries no permanent name field).
- The 2nd panel is a checkbox (`orgTwo_`, on by default, not persisted); with
  it off the window is the single-tree layout it had before.

### Two invariants the UI must keep

**Drops are DEFERRED.** `DrawOrgNodeDropTarget` only *records* the landing
(`orgDrop*` members); `ApplyOrgDrop` runs it after the tree draw. Inserting on
the spot reallocates the very vector the recursion is walking, leaving the loop
holding a dangling `ProgramNode&`. It is also what makes the tree walk a plain
range-for.

**A failed insert must not eat the subtree.** The inserts take the node **by
value**, so `ApplyOrgDrop` holds a copy until one succeeds and pushes the copy
back at the top level if none does. A folder full of recipes lost to a mis-aimed
drop is not something an operator can undo.

**A run outranks editing.** The window's editing surfaces are wrapped in
`BeginDisabled(orch_.Busy())` — the whole orchestrator, never one engine, since
the program runner goes momentarily idle between queue items. It does not close
itself and it raises **no modal**: the delete-program confirm is a plain popup,
because this window can be open on a second monitor while an engine worker is
blocked on the always-on-top prompt window.

---

*(A slot also carries the deck cycle's two uids — see
`docs/deck-cycle-design.md`. They are set from this window's right-click menu,
which keeps it the only surface that writes anything about a program.)*

## 4. What did NOT change

- `ProgramRunner` — same queue, same `Tick`, same `Stop`, same `$run` writing.
- Ctx still does not flow between recipes. Two passes of a loop are two
  independent recipe runs; data that must cross goes through the DUT or DataBus.
- The deck still shows exactly **one slot**, still persists `(program, slot)` by
  uid, still gates Execute on `orch_.Busy()`.
- `ClearLiveFrames("program start")` still fires on every deck run.

---

## 5. Folder-as-a-system — reserved, not implemented

The field `Folder::alignProcUid` exists and round-trips through JSON. Nothing
reads it at run time yet, and **the organizer deliberately shows no picker for
it**: a control that silently does nothing is worse than an absent one.

The agreed semantics, for whoever implements it:

> A folder naming an alignment procedure **re-teaches that frame at the start of
> every loop pass** — `align_run` → live frame → the pass's recipes consume it
> through `move_local` → cleared when the pass ends.

Per-pass, not per-folder, because **each pass handles a different physical
part**. That is the same rule the rest of the alignment layer already runs on
(`CLAUDE.md`): a saved frame is a record, never a licence to move; a live frame
describes the one part it was measured on and is cleared at every boundary. A
folder simply adds a boundary — the pass — to the list that already contains
program start, DUT open and product load.

Implementation notes for that pass:

- The frame is **Workflow** state, and layer 2.5 does not own it. `StartDeckRun`
  is where `ClearLiveFrames` lives today for exactly this reason; a per-pass
  clear needs an equivalent hook the runner can call *into App*, not a
  `Workflow` reference smuggled into `ProgramRunner`.
- The align run itself should be a real queue item (the shipped
  `alignment_procedure.lua` behind a recipe), not a special case inside the
  runner — same argument as §2: one execution path, or the loop grows its own.
