# Recipe Editor — Phase B Design (draft for approval)

Status: **draft** — implementation starts after this document is approved.
Prerequisite: Phase A (`src/command/`, commit 0908a65) — the engine, registry,
and recipe model are done and verified on hardware. This phase is **pure UI**
plus a handful of small, listed core tweaks; the engine's contract does not
change.

## 1. Goals and non-goals

**Goals**

- A graphical recipe editor: operators build and maintain recipes without
  hand-editing JSON.
- **Schema-driven forms**: the parameter panel is generated from the
  registry's `ParamSpec`s — a new command registered in Phase A code appears
  in the editor with a working form, zero editor changes. This is the payoff
  of the typed registry.
- Author-time validation surfaced live (the same `ValidateRecipe` the engine
  runs at Start), with click-to-navigate issues.
- Run integration: run the recipe being edited, watch the executing step
  highlight in the tree, prompt/stop handled globally.

**Non-goals (Phase B)**

- No undo/redo (note: the working-copy model makes "Revert to saved" cheap —
  that is the v1 safety net).
- No drag-and-drop step reordering (buttons first; DnD is additive later).
- No recipe diff/versioning, no step copy/paste between recipes (later).
- No Lua anything (Phase C).

## 2. Placement — a full-page editor, View menu

Follows the established editor-page pattern (`WatchEdit` / `TransformEdit`):
**View ▸ Recipe Editor** takes over the main window's panel layout until its
top-right Close button returns to `prevPage_`. New `Page::RecipeEdit` enum
value; all state lives in `App` (UI layer) — the editor is a *client* of
`src/command/`, which stays UI-free.

Inside the page, the familiar three-panel split (shared splitter sizes):

```
+----------------+--------------------------------+----------------------+
| Recipes        | Steps (tree)                   | Inspector            |
|  - list        |  > 1 set_output   vacuum on [x]|  [selected step's    |
|  - New/Dup/    |  v 2 loop (3x)             [x] |   schema-driven      |
|    Ren/Del     |      2.1 move_to_position  [x] |   parameter form]    |
|                |      2.2 read_topic        [x] |                      |
|                |  > 3 if pd > 1e-6          [x] |  nickname, enabled,  |
|                |  + Add step  ^ v  Del          |  on_error, params    |
+----------------+--------------------------------+----------------------+
| Validation issues (click -> select step)   [Validate] [Save] [Run/Stop] |
+-------------------------------------------------------------------------+
```

- **Left — recipe list.** The working copy (`editRecipes_`). New (fresh uid via
  `GenerateUid`), Duplicate (new uid — uids are identity), Rename (label only),
  Delete (confirm modal). A `*` marks unsaved changes.
- **Middle — step tree.** The nested model rendered as an indented tree
  (`ImGui::TreeNode` per structural step, selectable leaves). Per row: index
  path, type or nickname, a compact param summary (e.g. `move_to_position ->
  "PosA" (hex-left)`), and the `enabled` checkbox. Row buttons / context menu:
  Add sibling, Add child (structural only), Move up/down (within siblings),
  Delete. **Add step** opens the palette: registry commands grouped by
  category (`CommandRegistry::List()`) plus the six structural types.
- **Row icons.** A small picture sits after the step number in the tree and
  before the name in the palette — the SAME picture in both, from one lookup
  (`StepIconFile`, `ui/Widgets`), because the glyph learned while picking a
  command has to be the glyph met in the recipe it lands in. It answers "what
  does this row do to the machine?" before the text is read, which is what a
  twenty-step recipe needs: `move_relative` and `move_to_position` differ by
  four characters mid-name and by everything in what they do. The families are
  *move* (`move_*`, `place_move`, with relative moves on their own glyph),
  *IO* (the four IO commands), *extend* / *retract* (the pneumatic slides get
  their own pair rather than the shared IO glyph — a slide IS an output pin and
  two sensor pins, but which WAY it is about to travel is what the reader wants
  and IO cannot say it), *data* (`dut_*`), *prompt* (the machine has stopped
  and is waiting for a person) and *group* (a block of steps). The mapping is by
  command name, not category — Align and Motion both contain moves — and `loop`
  and `if` deliberately do NOT share the group glyph, since saying "these rows
  are the same kind of thing" is the opposite of what the column is for. Rows
  with no icon still reserve the column, so the names stay in one vertical line.
  Icons are cosmetic by construction: a missing PNG leaves a gap and logs once.
  The same set also fronts the page's **buttons** — New, Save, Run, Stop — via
  `IconButton`, which does NOT reserve space when an icon is missing (see
  `docs/ui-file-layout.md`). The Run page uses the identical four, so Execute
  and Stop mean the same thing on both surfaces.
- **Right — inspector.** The selected step's form: nickname, enabled,
  `on_error` (abort / continue / retry N), then one widget per `ParamSpec`
  (see §3). Structural steps get their structural form: loop → count-or-while
  radio + condition editor; if → condition editor; sub_recipe → recipe picker
  (by name, stores uid); stop_recipe → ok flag + message.
- **Bottom strip — validation + actions.** Issues list from `ValidateRecipe`
  (without the Connected requirement — that stays a run-time check); clicking
  an issue selects the offending step. Save, Validate, Run/Stop, and run
  progress (bar + state), replacing the Phase-A Recipe Runner window.

## 3. Schema → widget mapping

One function, `DrawParamWidget(const ParamSpec&, json& value)`, switched on
`ParamType` — the whole reason `ParamSpec` exists:

| ParamType | Widget |
|---|---|
| `Bool` | checkbox when optional; **ON / OFF radio pair when required** — an unticked box and a never-answered one are the same pixels, and on a required bool the second is a validation error the operator cannot see. Neither radio is lit until a pick is made, and either pick stores the value |
| `Int` / `Double` | drag/input scalar, clamped to `minVal..maxVal` when active |
| `String` | text input (`{key}` substitution documented in the help text) |
| `Value` | small type combo (bool / number / string) + matching field |
| `Enum` | combo over `options` |
| `DeviceMotion` / `DeviceIo` / `Slide` | combo over the HardwareManager fleet names |
| `PositionUid` | combo of the *device param's* saved positions — shows names, stores uids; re-filtered when the device param changes; plus **Go** and **Teach here** (below) |
| `MapName` | combo of the device's maps |
| `PinName` | combo of the device's pin names (see direction hint below) |
| `TopicName` | combo of `DataBus::Instance().Topics()` + free-text entry (topics create on demand) |

### `Go` / `Teach here` on a position row

A saved position is a taught pose, and the step that uses it is where an
operator notices it has drifted. Sending them to the Motion page to fix it is
how a recipe ends up pointing at a position nobody dared re-teach, so both live
on the form row.

**Teach writes the position IN PLACE — same uid.** Nothing breaks, because every
reference still resolves; and that is precisely why it raises a confirmation
naming the position and saying what else moves with it. The dialog is not an
"are you sure", it is the only place the sharing is visible: `P24` on one step
is the same `P24` on every other. `App::TeachPositionByUid` never *creates* a
row — a uid that resolves to nothing is a stale reference, which the picker
already draws in red, and inventing a position for it would bury that.

The dialog holds `RequestFastPoll()` open and shows saved-vs-live side by side
while it is up. Without that the pose read on the click would be up to a second
stale (the ACS driver idles at 1 Hz and nothing on this page asks otherwise),
and at a taught position that second *is* the measurement — the same reason
`record_position` demands a fast poll and waits a cycle out.

Both buttons sit inside the inspector's `BeginDisabled(!canEdit)`, i.e.
`!orch_.Busy()`: a **Go** from here mid-run would be a second hand on the
machine. This also means the confirm modal can never appear while an engine
worker is blocked on the always-on-top prompt window.

`help` renders as a tooltip on the label. Unknown/malformed existing values
(hand-edited JSON) render as a highlighted raw-text fallback rather than
crashing the form — tolerant, like every loader.

**Condition editor** (shared by `if`, `loop while`, and `wait_until`'s form):
source toggle (ctx key | topic), key/topic field (topic uses the TopicName
combo), op combo (the 10 ops), value field shown only for comparison ops.

## 4. Run integration

- Run executes the **working copy's** selected recipe (`Env.library` = working
  copy, so sub-recipe edits take effect without saving). Stop = `RequestStop`.
- While running, the editor goes **read-only** (forms disabled, tree still
  navigable) and the executing step row highlights.
- **The prompt modal moves out of the runner window to `DrawMainWindow`**
  (drawn every frame like `DrawSafetyAlertModal`). Phase-A bug being fixed
  here: today the prompt only appears while the Recipe Runner window is open.
  Global = it shows regardless of which page/window is visible, centered on
  the main window.
- The **Recipe Runner window is retired** — the editor's bottom strip covers
  select+run+stop+progress, and the window was documented as throwaway.

## 5. Editing model and persistence

- **Working copy.** Opening the editor copies `recipes_` → `editRecipes_`.
  Edits touch only the copy; a dirty flag per recipe drives the `*` and the
  close-with-unsaved-changes confirm. **Revert** re-copies from `recipes_`.
- **Save** validates first (issues don't block saving — half-built recipes are
  legitimate — but the user sees them), then writes through the existing
  Phase-A persistence: the product's `{uid}.recipes.json` (manifest uid
  stamped) when a product is loaded, else `config/recipes.local.json`. After
  save, `recipes_` = the saved copy, and the Reload-from-disk path stays
  untouched. `App::SaveProduct`'s `recipes` placeholder role is now real:
  it writes the current `recipes_` (mirroring the positions role).
- Recipes reference positions/maps by uid/name exactly as Phase A defined —
  the editor never invents a new addressing scheme.
- **Filing: `hidden` and `tags` live ON the recipe; the filter lives in
  UiSettings.** Hide/Show and Tag on the list's toolbar edit the selected
  recipe (dirty like a rename, saved with it, `hidden` and `tags` written
  only when set so older files round-trip unchanged). Hidden withholds the
  recipe from the LISTS — the editor's and the organizer's library — and from
  nothing else: a program that names it by uid still runs it, exactly like a
  hidden position resolving at validation. The list's filter (`show hidden`,
  one tag, show-only-that-tag or hide-it) is how one operator looks at the
  library, so it is persisted per station in UiSettings and never in the
  recipes file. A recipe the filter would drop still draws while SELECTED,
  dimmed: the editor beside the list is showing it. `App::RecipeListed` is
  the one predicate; the organizer applies the hidden half and lets its text
  filter match a tag as well as the name.

## 6. Small core changes required (the complete list)

1. **`ParamSpec.pinDir`** — `enum class PinDir { Any, Input, Output }`,
   default `Any`. The PinName combo must list input pins for `wait_input` /
   `clear_latch` and output pins for `set_output` / `pulse_output`; validation
   also tightens from "resolves as either" to the declared direction. Touches
   `CommandSpec.h`, the four pin-using command registrations, and
   `Recipe.cpp`'s `CheckReferences`.
2. **Structural palette metadata** — a tiny constant table in `Recipe.h`
   (`{type, description}` for the six structural types) so the palette
   doesn't hardcode strings in the UI.
3. **`RecipeEngine::Status.stepPath`** — add `std::vector<std::size_t>` (the
   numeric path of the executing step, e.g. `{1,0,2}`) alongside the existing
   display string, so the tree can highlight without parsing "3.2.1". Engine
   already tracks the path as a string; carrying the vector is mechanical.
4. **Prompt modal relocation** — from `DrawRecipeRunnerWindow` to
   `DrawMainWindow` (App-only change, listed for completeness).

Nothing else in `src/command/` changes — that is the acceptance test for the
Phase-A layering.

## 7. Implementation order

1. **B1 — browse + run**: page + three panels, read-only tree, read-only
   inspector, bottom strip with run/stop/progress, global prompt modal,
   running-step highlight. (Engine tweaks #3 and #4 land here.) Retire the
   runner window.
2. **B2 — edit**: palette + add/delete/move/enable/nickname/on_error, the
   schema-driven forms, condition editor. (Core tweaks #1 and #2 land here.)
3. **B3 — persist + validate**: working-copy dirty tracking, Save/Revert,
   product role wiring in `SaveProduct`, validation strip with
   click-to-navigate.

Each sub-phase builds clean and is usable on its own; B1 alone already
replaces the runner window.

## 8. Open questions (decide at review)

1. **Where does Run live long-term?** Drafted: in the editor's bottom strip
   (author-run-fix loop). If operators need a simplified "pick and run"
   surface later, that's a small dedicated page reading the same engine —
   deferred until asked for.
2. **Step param summaries in the tree** — drafted: yes, one-line compact
   summary per row (device/position names resolved). Costs a little per-frame
   lookup; can be cached per edit if it ever shows in profiling.
3. **`Value`-typed param editing** — drafted: explicit type combo. Alternative
   is free text with inference (`"true"` → bool?), rejected as too magical for
   a machine controller.
