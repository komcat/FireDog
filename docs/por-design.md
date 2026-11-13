# POR — a master product compared with, then deployed to, another machine

*Status: DESIGN (2026-11-12). Nothing here is built yet. Working name of the tool:
`FireDogPor`.*

## 1. What it is for

An engineer gets a product's programs working on one machine — the **master**. The
other machines should run the same plan of record (POR): the same deck, the same
recipes, the same steps. Today the only way across is copying the product folder by
hand, which carries the master's MEASURED data with it — positions, view points,
detection models taught on another camera — and a position that is right on one
machine is a crash on the next.

So a separate executable, run by an engineer on purpose:

1. **Compare** (phase 3, read-only): show, row by row, how the target's copy of a
   product differs from the master's — the deck, every recipe, every step and its
   params, and every position the deck needs.
2. **Deploy** (phase 4): copy the master's STRUCTURE onto the target, translated to
   the target's own objects, and say what still has to be taught there.

It is a separate exe, not a FireDog page, for the reasons FireDogRig is: it has one
narrow job, it never reaches out to a network, and nothing in the client can pull a
plan behind an operator's back. **The repository is PUBLIC** — a POR never goes to
GitHub Releases; it travels as a folder (USB stick, plant share).

## 2. What is structure and what is measured

A product folder mixes the two. The line:

| Structure — the same on every machine | Measured — belongs to one machine |
|---|---|
| programs: slots, folders, loops, dry-run flags, the deck cycle | saved positions (raw axis readings) |
| recipes: steps, order, nesting, params, nicknames, `on_fail` | `PointXyz` params stored IN a step (`detect_object` / `vision_detect` `view_point`, `dispense_run` `point`) |
| product scripts, prompt images | alignment station poses + nominal camera poses (`algn_<uid>_…`) |
| alignment procedure layout + nominals (local frame) | detection models + `.pgm` (taught at one pose, exposure, light, camera) |
| grip profile settings, tray rows/cols/masks | grip inspect pose + reference offset, tray corners + used cells |

And outside the product, machine-level and never touched: `hardware.json` (device,
PSU, slide, light and pin NAMES the recipes refer to), `maps.local.json` (named maps
whose nodes are position uids), needle cal, device registration, transforms, pivots.

**The compare shows both kinds; it judges only structure.** A measured value is shown
with its difference, in its own colour, as information — two machines built to one
drawing should agree within a few mm, and one that does not is worth a look, but it
is never a "difference to fix".

## 3. Matching — uids mean nothing across machines

Every uid is minted per machine (`GenerateUid`, 8 hex), so the same recipe, node or
position has a different uid on each. Everything is matched by NAME and STRUCTURE:

| What | Matched by |
|---|---|
| program, slot, folder | name (within its parent) |
| recipe node in a slot/folder | the NAME of the recipe it points at (each side resolves its own `recipeUid`) |
| recipe | name |
| step | **nothing** — steps have no identity (`Recipe.h`); sequence-aligned per nesting level on `(type, nickname)`, recursing into `children` and `else` |
| position | device + name |
| alignment / grip / tray / vision model | name; a nominal by name WITHIN its procedure (duplicated procedures share nominal uids) |
| private objects (`algn_<uid>_…`, `vdet_<uid>_…`, `vdet_step_<uid>`, `grip_<uid>_i`) | through their OWNER (procedure + station, step, profile) — never by the literal name, which embeds a uid |

**Rows align like a text diff** (longest common subsequence on the match key) in every
list: a slot's items, a folder's items, a recipe's steps, a block's children. Each
pair comes out as one of: **same**, **changed**, **only master**, **only target**.

**Names are not enforced unique** (positions, recipes, programs, alignments, grips,
trays). A name that occurs twice on either side is reported **ambiguous** and its rows
are not paired — guessing which of two "Home"s is meant is exactly the mistake this
tool exists to prevent.

## 4. Comparing a step's params

Both sides are compared AFTER normalisation, so the diff shows what an engineer would
call a difference:

1. **Defaults filled in** on both sides (`EffectiveParams`) — a key left out is not a
   difference from the same value written down.
2. **References translated to names** before comparing, each through its `ParamType`
   (`CommandSpec.h`), so a command added later is compared without touching the tool:
   - `PositionUid` → `device / name`; `sub_recipe.uid`, `on_fail` → recipe name;
     `AlignUid`, `NominalUid`, `VisionUid`, `GripUid`, `TrayUid` → name;
     `detect_object.model` (a `String` holding the step's private model) → "private model".
   - Each translated reference then carries its status ON THE TARGET: found, or
     **missing** (a position, recipe, alignment, map, device, pin, slide or light the
     target does not have).
   - Names already (`DeviceMotion/Io/Psu/Hexapod`, `Slide`, `LightName`, `MapName`,
     `PinName`) are checked against the target's `hardware.json` / maps.
3. **Measured params** (`PointXyz`) are shown with their delta, coloured "measured",
   never "changed".
4. **Unknown commands** (a step whose `type` is not in this build's registry — e.g. the
   retired `place_run` in old hidden recipes) compare by raw JSON and are flagged, never
   abort the compare.

## 5. The tool

### 5.1 Inputs
- **Master** — a folder: the product folder plus the master machine's
  `config/hardware.json` and `config/maps.local.json`. The tool's *Export master*
  writes exactly that to a chosen folder (read-only on the master; it writes only to
  the destination) with a small `por_export.json` (product, source machine name, date,
  exe version, file hashes).
- **Target** — this machine's root, resolved from `FireDog.ini` READ-ONLY, or any
  folder of the same shape (so an engineer can compare two exports on a desk PC).

### 5.2 Read-only by construction
The compare must not write a byte to the target. So the tool does **not** use
`ProductSession` (its `LoadProduct` can rewrite `manifest.json` via
`EnsureManifestRole`) and does **not** call `InitPaths` (it can write `FireDog.ini`,
run the first-start migration and seed files) — it loads each side with the plain
`Load*` functions from explicit paths and logs to its own file, never
`firedog.log`. The command specs come from `RegisterBuiltinCommands` over a
`HardwareManager` + `Workflow` that are constructed and never `Start`ed (no threads, no
connects; Pylon is initialised, which is why the tool ships beside `FireDog.exe`).

### 5.3 Layout — three columns
```
┌ Explorer ────────┬ MASTER (siphog888 @ M1) ─────┬ TARGET (siphog888 @ M3) ─────┐
│ ▾ Deck           │ 1 move_to_position  Home     │ 1 move_to_position  Home     │
│   ▾ Siphog888    │ 2 detect_object  Sled  ◆     │ 2 detect_object  Sled  ◆     │ measured
│     ▸ PROD V2  ⚠ │ 3 prompt "Power < 1500"      │ 3 prompt "Power < 1000"      │ changed
│     ▸ ENGINEER   │ 4 sub_recipe  UV cure        │   ─                          │ only master
│ ▾ Recipes (47)   │   ─                          │ 4 wait 500 ms                │ only target
│   ● PP Left    ⚠ │ 5 move_path  ProcessFlow     │ 5 move_path  ✖ map missing   │ missing ref
│ ▾ Positions    ✖ │                              │                              │
└──────────────────┴──────────────────────────────┴──────────────────────────────┘
```
- **Explorer** (left): Deck (programs → slots → folders → recipe nodes), Recipes,
  Positions (§6). Every item carries the WORST status below it, so the problem is found
  without opening anything.
- **Master | Target** (middle, right) are ONE `ImGui::BeginTable` with two columns in
  one scroll region — paired rows line up and scroll together by construction (there is
  no synced-scroll code in FireDog to reuse, and two children would drift).
- A row expands to its params: `name: master | target`, one line each, coloured as §4.
- A step row's text is the shared step description (§8, phase 1c) — the tool and the
  Recipe Editor describe a step identically.
- Colours from `ui/Theme`: red = missing/invalid, gold = changed, `TextDisabled` = only
  one side, a "measured" colour for §4.3, `kDryRunCyan` for dry-run folders.

## 6. The Positions page

A position matters in proportion to whether the deck drives to it, so the page starts
there.

### 6.1 What the deck requires — the trace
From every program → slot → node → recipe (`WorkingRecipes`: nodes, `sub_recipe`,
`on_fail`) → every step (enabled AND disabled — a disabled step is one click from
moving):
- every `PositionUid` param (`move_to_position*`, `move_lift_off`, `check_position`,
  `move_path` `from`/`to`, `align_run`, `dispense_run`, `tool_travel_to_camera`
  `via_position`, `plant_beacon`, …);
- through `align_run`/`alignment` → that procedure's station poses and its nominals'
  camera poses;
- through a grip profile → its inspect pose;
- through `move_path` → the map, and the map's nodes on the route's device.

**Not traceable, and said so:** a product script that finds a position by name
(`fd.positions(device)`) — the page lists such scripts as "not traced" rather than
pretending the list is complete.

### 6.2 Order and rows
1. **Required by the deck** — grouped by device, in order of FIRST USE (slot order,
   then step order), so the list reads like the teach order of a cycle.
2. **Private poses** of the required procedures and grips, under their owner.
3. **Not used by the deck** — the rest of the master's visible positions.
4. **Only on the target.**

Each row: device · name · used-by count (expands to `slot › recipe › step` paths) ·
master axes · target axes · Δ per axis and |ΔXYZ|. Statuses:
- **missing on target** (red) — the deck cannot run there; for phase 4 this is a
  placeholder to teach.
- **ambiguous** — the name occurs twice on a side.
- **axes differ** — one side has an axis the other does not.
- **Δ over threshold** (user-set, default 1 mm / 0.1°, persisted with the tool's
  settings) — highlighted, never an error.
- **map** — a required map missing on the target, or a node of it pointing at a
  position the target lacks.

## 7. Deploy (phase 4 — direction, to be designed on phase-3 results)

- Refused while `FireDog.exe` is running (its session holds the product in memory and
  its next Save would overwrite the deploy), and while the target has a device / pin /
  slide / light / map the deck needs missing, or a command this build lacks.
- Back up the target's product folder (dated copy) before writing anything.
- Write programs, recipes, product scripts, prompt images — **translated**: every
  master uid replaced by the target's uid of the same name. A matched step keeps the
  TARGET's `PointXyz` and private model; an unmatched one is flagged "re-teach".
- A required position the target lacks becomes a placeholder row with a fresh target
  uid and the master's name, flagged **untaught**. This needs FireDog to learn one
  boolean: movers refuse an untaught position, and teaching one opens Manual Adjust
  where the device stands instead of driving to the stored value first
  (`RecordPositionByUid` already refuses to create a row).
- Never writes positions that exist, alignment poses, models, grip offsets, tray
  corners, or anything under `<root>/config/`.
- Stamps `por.json` in the product folder (rev, source machine, date, hash per deployed
  file): FireDog shows "POR rev N" / "modified locally", every `$run` carries the rev,
  and the next compare can say what changed on the target since.

## 8. Phases

Each numbered item is its own commit; foundations change no behaviour.

**Phase 0 — this document.**

**Phase 1 — foundations in FireDog (no behaviour change)**
- 1a. `firedog_cmd` static library: `src/command/`, `src/session/`,
  `src/mosaic/MosaicDoc.*` move out of the exe glob. It links `firedog_hw`, `lua_static`,
  `sol2`, the stb include dir; `SOL_ALL_SAFETIES_ON` PRIVATE. `MosaicWorker`, `update/`,
  `ui/`, `App` stay in the exe. Build-only commit; CLAUDE.md + `docs/build-and-sdks.md`
  updated to four targets.
- 1b. `GlHost::Init` takes the `imgui.ini` path (FireDog passes today's
  `MachinePath("imgui.ini")`).
- 1c. `App::AutoNickname` becomes a free function over a small resolver interface
  (recipes, positions, aligns, grips, trays by uid → name), output byte-identical; the
  editor calls it through an adapter over `session_`.
- 1d. A path-taking product loader (`LoadProductManifestAt(dir)`) beside the
  name-taking one, and a read-only `FireDog.ini` reader that resolves the root without
  `InitPaths`' side effects.

**Phase 2 — the compare engine (plain C++, no UI, `src/por/` in `firedog_cmd`)**
- 2a. `PorSide`: one side loaded read-only from a folder — manifest, positions,
  recipes, programs, aligns, grips, trays, vision index, hardware names, maps.
- 2b. Name resolution + ambiguity detection; param normalisation (§4) driven by
  `ParamType`.
- 2c. Sequence alignment (LCS) for slots/folders/steps → a diff tree of paired rows with
  statuses; the worst-status roll-up for the explorer.
- 2d. The position trace (§6.1) and the positions table (§6.2).

**Phase 3 — `FireDogPor` (read-only)**
- 3a. Target + shell: `tools/por/`, links `firedog_cmd` + imgui/implot/glfw, own icon
  `.rc`, own `imgui.ini` and log, `firedog_stage_runtime`; pick master folder / target.
- 3b. Explorer + Deck/Recipes compare (§5.3).
- 3c. Positions page (§6).
- 3d. *Export master* (§5.1).
- Release: `package-release.ps1` already packs every exe; the `/release` skill's
  verify step learns the third exe.

**Phase 4 — deploy** (§7), designed in detail once phase 3 has been run against two
real machines: the untaught flag + teach-in-place in FireDog first, then the
translated write, backup, `por.json`, and the rev on `$run`.

## 9. Open questions
- The tool's name (`FireDogPor` is a working name).
- Which level may run it — it is an engineer's tool, but it has no login of its own;
  deploy (phase 4) probably wants the roster's Admin.
- First deploy to a machine that lacks the product entirely: structure + all
  placeholders and a full teach is the safe default; seeding the master's values as
  untaught starting points is a possible option.
