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

## 6. Step values per variant (PSU first)

Some models need different supply settings, not different places. The same
idea reaches recipe STEPS: a step keeps its params as the base and carries
per-variant overrides of the params that opted in.

```json
{ "type": "psu_startup",
  "params":   { "device": "psu1", "channel": 1, "mode": "CV", "volts": 3.3, "amps": 0.5 },
  "variants": { "65C": { "volts": 2.8, "amps": 0.3 } } }
```

- **Opt-in per PARAM** (`ParamSpec::variant`, set after construction like
  `siUnit`), the same shape as a command opting into teach or `parallel`. A
  variant may change a value that is a property of the part — a supply's
  volts for this model — never WHICH device, channel or position a step
  drives. Opted in today: `psu_startup` `volts` `amps` `tol_v` `tol_a`,
  `psu_set_vi` `volts` `amps` (`WithVariantParams`, `CommandsInstruments.cpp`).
  A light %, an exposure or a Keithley bias opts in with one line.
- **Applied once, after inlining** — `ApplyStepVariants(steps, variant, reg)`
  in `RecipeEngine::Start` (after `InlineSubRecipes` and the run hash, so the
  sub-recipe bodies get it; the on-fail recipe too) and in Lua's
  `fd.run_recipe` (the ctx's variant). It writes the override over the param
  and CLEARS every step's `variants`: what runs is flat and no command ever
  reads an override. Base only clears.
- **Every variant is validated**, whichever one a run uses: `ValidateRecipe`
  checks each override set with `ValidateParams` (`variant 65C: volts ...`)
  and refuses a key whose param did not opt in. Validation is per step, so
  the editor's red marks and Start's refusal cover all models at once.
- **Edited in the step form under the working variant** (the Recipe Editor's
  combo): an opted-in row is drawn over a view holding the variant's value
  and written back into the override (`App::DrawStepParam`), violet when the
  variant has its own, with a `base` button to drop it. Base editing is
  unchanged.
- The overrides are part of the step's JSON, so they are in the recipe's
  dirty state, its copy/paste and its cycle-time version hash (editing a
  65C value re-starts learning for every model of that recipe).
- Deleting a declared variant also drops its step overrides
  (`DeletePositionVariant`), leaving the recipes DIRTY for an explicit Save.

## 7. POR (FireDogPor)

- **Deck**: a program's `variant` is a compared param; the target's value is
  judged Missing when the target does not declare it (Execute would refuse).
- **Steps**: for every variant either side overrides, `<param> @<variant>`
  compares the value that variant RUNS with on each side (override, else the
  base) — so an override equal to the other side's base is Same.
- **Positions** (deck / private tiers): values stay measured and are never
  compared per variant; COVERAGE is. A variant the master has taught for a
  position that the target has not (none, or an untaught one) is Missing —
  that model would drive to the base place there — and becomes a Teach item
  on the Fix list (`teachAfterDeploy`). A variant only the target has is noted.
- **Declared lists** (`PorSide::posVariants`): a variant a master program runs
  that the target does not declare is a Missing issue and a Configure item
  (a deploy BLOCKER); one declared on one side only is listed, not judged.
- **Deploy plan**: a placeholder carries the master's variant entries, every
  one `untaught`. Programs and steps carry their `variant` / `variants` as
  written. Export master copies the files whole, so variants travel.

## 8. Out of scope (v1)

- **Cycle times** are keyed by the recipe's version hash, not the variant.
- No variant rename (declare the new name, re-teach, delete the old).
- Private positions (`algn_*`, grip inspect poses) are never offered a variant;
  `CapturePrivatePose` writes base only.
