# Position variants

A product sometimes comes in models that are *almost* the same part: most taught
positions are identical, a few sit slightly off (model B's pick point is 0.2 mm
over). Without variants the only answer is a second set of positions under new
names and a second copy of every recipe that names them — two documents that
drift apart the first time one is edited.

A **position variant** keeps ONE position — one name, one uid, every recipe,
map and program reference — and lets it carry extra value sets, one per variant.
A run says which variant it moves by; a position taught for that variant moves
to those values, every other position to its base values.

## 1. The data

```cpp
struct SavedPosVariant { std::map<std::string,double> vals; bool untaught; };
struct SavedPos { ...; std::map<std::string,double> vals;            // the BASE
                       std::map<std::string, SavedPosVariant> variants; };
```

On disk (the product's `{uid}.positions.json`, or `positions.local.json`):

```json
{ "uid": "...", "variants": ["B", "C"],
  "positions": { "gantry-main": [
    { "uid": "...", "name": "pick", "axes": {"X": 10.0, "Y": 5.0, "Z": -3.0},
      "variants": { "B": { "axes": {"X": 10.2, "Y": 5.0, "Z": -3.0} } } } ] } }
```

- **Base is implicit.** `vals` IS the base variant; it is never listed and the
  name `base` cannot be declared.
- **The list is DECLARED per product** (`"variants"`, `ProductSession::PositionVariants`).
  A program header, the working-variant combo and a teach can only name a
  declared one, so a typo never invents a variant. Declared and deleted on the
  Motion page (*Variants...*).
- Both keys are written only when non-empty: a file without variants is
  byte-for-byte what an older build wrote. **An older build that SAVES the
  file drops both** — a station must not run a pre-variant exe once variants
  are taught.

## 2. Resolution: overlay once, at Env build

`EffectiveVals(pos, variant)` / `EffectiveUntaught` (`core/SavedPositions`) are
the ONE place a variant is resolved: the variant's entry when the row has one,
else the base.

A run never sees the structure. `ProductSession::MakeRecipeEnv(variant)` /
`MakeLuaEnv(variant)` build the run's positions with `ApplyPositionVariant`: a
FLAT copy whose every row already holds its effective values and untaught
flag. So `ExecutionContext::FindPosition`, every mover, `BuildMapRoute`, Lua
`fd.*`, `ValidateRecipe` and the untaught refusals are unchanged — the same
shape as the teach and dry-run transforms: one transform at Start, nothing
downstream learns. `Env::posVariant` records which variant was applied; it is
read only by the teach hand-off and the DUT record.

## 3. Who picks the variant

| Surface | Variant |
|---|---|
| Run page deck (Execute, Teach run) | the PROGRAM header's `Program::posVariant` (JSON `"variant"`), set in the Program Organizer |
| Recipe Editor Run / Run Step / Run From, Teach here, the form's Go | the WORKING variant (`App::posVariant_`), the run strip's combo |
| Motion page positions table, Update / Edit / Move, *Use base* | the working variant (the same member; its combo sits by the title) |
| Jog pad Go, its "at" readout and hold + Upd; `StartWatchedMove` callers | the working variant |
| Everything else (quick autofocus, mosaic, align / grip / needle / scripts pages, XCAN / raster) | base |

The working variant is runtime only (like teach mode) and cleared on product
load. The Run page never reads it: what a deck runs is a property of the
program, not of whoever last touched the Motion page. A program naming a
variant the product no longer declares is **refused at Execute** (red on the
strip), before the frames or the cycle are touched.

## 4. Teaching a variant

A teach is how a variant gets its values — "keep the name, teach it for B":

- **In a run** (a teach-mode run, a mover's `teach`, `save_position`): the run
  copy is flat, so a teach updates its `vals` exactly as before. The hand-off
  (`ExecutionContext::QueueTaught`) tags the row: under a variant it is queued
  with ONE `variants` entry holding the values, and
  `ProductSession::MergeTaughtPositions` writes that entry, never the base. The
  first teach of B on a row creates its B entry; base is untouched. A row the
  run CREATES (`save_position` of a new name) gets the values as its base —
  there is nothing to differ from yet.
- A teach-mode run under B of a position with no B entry starts from the base
  values (the fallback), Manual Adjust opens there, Save creates the B entry.
- **By hand**: Motion page Update / Edit, the editor's Teach here and the Jog
  box's Upd write the working variant through `SetPositionVariantVals`
  (starting from what it showed, so an axis the row never had stays out). *Use
  base* (`RevertPositionVariant`) drops the row's entry. Add always creates a
  base row.

## 5. Journal

`PosChange::variant` (JSON `"variant"`) says which variant an entry is about.
`DiffPositions` diffs the base AND each variant entry: an entry appearing is an
`add` (restore removes it — back to base), changing an `update`, disappearing a
`delete` (restore re-creates it). A deleted row also journals each of its
variants, so after restoring the row each variant can be restored. A rename is
always of the row.

## 6. Out of scope (v1)

- **POR** compares and deploys base values only. Export master copies the file
  whole, so variants travel with it; a deploy placeholder does not yet copy
  variant entries (they should arrive `untaught`), and the compare does not
  judge them.
- **Cycle times** are keyed by the recipe's version hash, not the variant.
- No variant rename (declare the new name, re-teach, delete the old).
- Private positions (`algn_*`, grip inspect poses) are never offered a variant;
  `CapturePrivatePose` writes base only.
