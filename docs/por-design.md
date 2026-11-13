# POR — a master product compared with, then deployed to, another machine

*Status: phases 1-3 are in (2026-11-13): the foundations, the compare engine
(`src/por/`) and the read-only tool `FireDogPor` (`tools/por/`), released in v0.24.
Phase 4 (deploy) is designed in §7 and not built. Working name of the tool: `FireDogPor`.*

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
- **Master** — a ROOT folder laid out like a machine root: an export, or another
  machine's root over the network (`\\M1\D$\ProgramData\FireDog`). The tool's *Export
  master* (`tools/por/PorExport`) copies `products/<name>/` (minus `*.bak` and the
  `*.history.jsonl` journal) and `config/hardware.json`, `maps.local.json`,
  `motion_transforms.json` into an EMPTY folder the engineer names, plus a
  `por_export.json` (product, source root, date, the tool's build, the file list). It
  refuses a non-empty destination or one inside the product folder; the master is
  only read.
- **Target** — this machine's root, resolved from `FireDog.ini` READ-ONLY
  (`ResolveRootsReadOnly`, the *This machine* button), or any folder of the same shape
  (so an engineer can compare two exports on a desk PC). A target that lacks the
  product still loads its hardware, maps and transforms, so a first compare shows the
  hardware page and every master row as only-on-master.
- The tool's own state — the roots last used, thresholds, the window, `imgui.ini`, the
  log — is under `%LOCALAPPDATA%\FireDogPor`: never beside the exe, never a machine root.

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
  Positions (§6), Hardware (§6a). Every item carries the WORST status below it, so the problem is found
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

## 6a. The Hardware page

Two machines of one TYPE should be configured alike, and the step compare cannot see
a wiring difference: a step that names `switch_8` resolves on both machines even when
`switch_8` is input 3 on one and input 5 on the other. So the master's
`hardware.json` and `motion_transforms.json` are compared with the target's, section
by section (`por/PorHardware`):

| Section | Matched by | Rows beneath |
|---|---|---|
| Motion devices | name | — |
| Motion transforms | device name | — |
| IO devices | name | one per named pin, `IN n` / `OUT n` |
| Pneumatic slides | name | — |
| Cameras | display name, else id | — |
| Keithley · CLD101x · PSUs | name | — |
| Lights | name | one per channel `Ln` |
| IO watch · Safety watch | (one each) | bindings by name · signals by device + pin |

- **Pins are compared by NUMBER**, both sides' names on the same pin side by side
  (pin numbers are normalised, bit i == pin i, `CLAUDE.md` Hardware). A name that sits
  on another pin on the other side is **Changed — "wired differently"**, with the pin
  it is on there. A name the target lacks is Missing when the deck uses it.
- **Fields have a class.** DESIGN (axes, controller type, counts, pin and channel
  names, triggers, transforms) is judged. MACHINE (IP, port, board id, serial, VISA
  address) is this unit's identity: shown dimmed, never judged. A camera's pixel→mm is
  MEASURED: shown with its difference.
- **`used`**: every device, pin, slide and light channel the master's deck reaches
  (the working recipes' device/pin/slide/light params, a slide's own pins, a command's
  `needs`) is marked; a used one missing on the target is Missing, the rest of a
  difference Changed.
- The page compares and nothing else: a fix is a hand edit of the target's
  `<root>/config/hardware.json`, the same as today (`CLAUDE.md`, State & documents).

## 6b. The Fix list - errors, not differences

Two questions, kept apart:

- **Differences** (the tree, §5.3-§6a): is the target the same as the master? A
  difference may be intended, and a deploy can carry the structure across.
- **Errors** (the Fix list, `por/PorFixList`, the first page): can the MASTER's deck
  run on THIS machine? Almost every error is something a deploy never carries,
  because it is measured or machine data - so it is a to-do list, not a diff.

The list walks the master's deck (every program > slot > node > recipe, `sub_recipe`
and `on_fail` followed, disabled steps included) and checks each reference BY NAME on
the target, exactly as a deploy would translate it. A recipe, a sub_recipe, a product
script or a product picture the target lacks is not an error: the deploy brings it.
Positions come from the Positions page's own trace, so the two pages cannot disagree.

| Group | What lands in it | Fixed by |
|---|---|---|
| The master itself is broken | a reference the MASTER cannot resolve: a deck node, sub_recipe or on_fail uid, a position, alignment, model, grip, tray, map, script or picture | fixing the master before it is distributed |
| Teach on this machine | a missing position (or a private pose, through its owner), alignment, nominal, detection model, grip profile, tray; a `PointXyz` or `detect_object` model whose step has no twin here (same recipe, same type, same `ctx_prefix`, else the same nickname) | teaching it here |
| Configure this machine | a device, PSU, slide, light or pin not in this `hardware.json`; a pin NAME on another pin NUMBER than on the master; a motion device with other axes; a missing map; a machine-level script or picture | editing this machine's `hardware.json`, the Motion page's maps, `<root>/config/scripts` |
| Names to resolve | a name used twice here, so a reference cannot be told apart | renaming |
| Update FireDog | a command this build does not know | an update (or the master still uses a retired command) |

Each item carries its uses (`slot > recipe > step`) and a Go to the recipe, the
Positions page or the Hardware section. The summary line leads with the count to fix.

## 7. Deploy (phase 4) - design, 2026-11-13

Deploy makes the target's STRUCTURE the master's, translated onto the target's own
objects, and leaves everything measured or machine-owned where it is. Afterwards the
Fix list is the target's teach list and nothing else.

### 7.1 Where and when it may run

- **On the target machine, into its own root only.** The target root must be what
  `ResolveRootsReadOnly()` says this machine's root is; a network or export root is
  never a deploy target. That is what makes the next check meaningful.
- **Refused while `FireDog.exe` runs on this PC** (process list). Its open session holds
  the product in memory and its next save would overwrite the deploy - and a run in
  progress must never have its recipes swapped under it.
- **Signed by an Admin of THIS machine's roster.** The tool has no session, so the
  deploy window asks for an employee number and password, checked with `Authenticate`
  against `<root>/products/employees.json` (a path-taking `LoadEmployeesFile`; the
  built-in admin works, as on the login gate). The id signs the journal and `por.json`.
- **A master export from a newer build is refused.** The tool's loaders and savers are
  this build's (it ships with this FireDog), so a field a newer master wrote would be
  read, not understood, and dropped on save. `por_export.json`'s `tool` is compared
  with this build; a master read live over the network has no such record and gets a
  warning instead.

### 7.2 Preflight - the Fix list decides

A deploy runs only when the Fix list (§6b) holds nothing but things a deploy turns into
**placeholders**. Every other group blocks, because the deck it would leave behind could
not run, or the translation would have to guess:

| Fix list group | Deploy |
|---|---|
| The master itself is broken | **blocks** - fix the master, export again |
| Configure this machine (hardware names, wiring, axes, maps, machine scripts) | **blocks** - configure first |
| Names to resolve | **blocks** - a name used twice cannot be translated |
| Update FireDog | **blocks** - update first |
| Teach: positions, private poses, taught points, `detect_object` models | allowed - become untaught placeholders (§7.4) |
| Teach: alignments, nominals, grip profiles, trays, named models | allowed from phase 4d (§7.5); blocks before it |

The deploy window shows the blockers with the Fix list's Go buttons, and the plan
(§7.7) only when there are none.

### 7.3 What is written - and what is never touched

| Product file | Deploy |
|---|---|
| recipes | master's recipes, translated (§7.4); target-only recipes kept |
| programs | master's programs, translated; target-only programs kept |
| `scripts/`, `prompt_images/` | master's files copied over; target-only files kept |
| positions | **only additions**: untaught placeholders for required positions the target lacks. Existing rows are never changed |
| alignments, grips, trays | phase 4d: missing ones created DESIGN-ONLY (§7.5); existing ones never changed |
| vision (models + `.pgm`) | never - every detector is taught per camera |
| manifest | kept; created only on a first deploy (§7.6) |

Never touched: anything under `<root>/config/` (hardware, transforms, maps, needle cal,
device registration, panel switches, ...), the data root (DUT records, position
records, mosaics), the roster.

**Target-only recipes and programs are kept** by default - a local experiment is work
somebody did. The deploy window lists them, and offers *Remove what the master does not
have* (off by default) for a strict mirror.

### 7.4 Translation - every uid, by name, through the same pairing the compare uses

The master's documents are copied and every reference rewritten from the master's uid
to the target's uid of the same NAME, per `ParamType` - the compare's own resolution
(§4), now writing instead of judging:

| Reference | Becomes |
|---|---|
| recipe (program node, `sub_recipe`, `on_fail`) | the target's recipe of that name; a recipe new to the target keeps the MASTER's uid when it is free here, else a fresh one |
| program | the target's program of that name keeps ITS uid (DUT `$run`s reference it); a new one keeps the master's uid when free |
| program node uids, slot cycle states | the master's, as written (unique within the file; only runtime state refers to them) |
| `PositionUid` | the target's position of `device` + name; none -> an untaught placeholder (below) |
| `AlignUid`, `GripUid`, `TrayUid`, named `VisionUid` | the target's object of that name |
| `NominalUid` | the nominal of that name within the translated alignment |
| private `VisionUid` (`vdet_<procUid>_...`) | the same station's model of the target's alignment of that name |
| device / IO / PSU / slide / light / map / pin names, scripts, pictures | unchanged (names already; the preflight proved they exist) |

**Measured values travel with the TARGET's step, never the master's.** Recipes are
paired by name and their steps by `PorAlign` - exactly the compare's pairing (§3), so the
preview and the result cannot disagree. Where a master step pairs with a target step,
the target step's `PointXyz` values and `detect_object` model are carried into the
written step. Where it does not, they are made **untaught**:

- a `PointXyz` becomes `{}` - the untaught-point marker (§7.8). FireDog's validation
  refuses a run whose step holds `{}`, so it never silently becomes "no point" (for
  `detect_object` that would mean *detect where the camera stands*);
- a `detect_object` model becomes `""` - creating one is a button in the Vision detect
  helper, and a step without one is refused at Start.

The Fix list's own `Twin` heuristic (same type + `ctx_prefix` / nickname) is replaced by
this `PorAlign` pairing first (phase 4b), so "teach this point" and "this point was
carried" are one answer.

**Placeholders.** A required position the target lacks (from the Positions page's trace:
deck positions and map nodes, and in 4d the private poses of created owners) is added
with a FRESH target uid, the master's name and `hidden` flag, the master's axis values,
and **`untaught: true`**. The values are kept only as what the master had - every mover
refuses an untaught position (§7.8), so they can never be driven to.

### 7.5 Missing alignments, grips and trays - design-only copies (phase 4d)

Each carries its design and has every measured part emptied, so it exists to be taught:

| Object | Copied | Emptied |
|---|---|---|
| alignment | name, device, guard, nominals (`lx`/`ly`: the local frame is design), station modes | station poses and nominal camera poses -> untaught placeholders `algn_<newUid>_...`; detectors -> none; `result`, `runs` -> empty |
| grip profile | name, device, find mode, tolerance, outline | inspect pose -> untaught placeholder `grip_<newUid>_i`; detector -> none; reference and last offsets -> none (no reference = no PASS) |
| tray | name, device, rows x cols, masks | corners -> untaught; `usedCells` -> none (it describes one physical tray) |

Until 4d, a missing alignment, grip or tray blocks (§7.2) and is created by hand.

### 7.6 First deploy - the target has no such product

A new product folder is created the way `CreateProduct` does (fresh manifest uid, one
uid-named file per role) but by path, not through `MachinePath`. Everything the deck
needs becomes a placeholder, so the first Fix list after it is the machine's complete
teach list. DUT records start fresh under `<data>/products/<name>/` - the product NAME
is the link to history, which is why a deploy never renames.

### 7.7 Apply - preview, backup, write, journal, stamp

1. **Plan** (no write): load both sides fresh, run the preflight, translate everything
   in memory. The window shows the plan - recipes replaced / added / kept, programs
   likewise, placeholders by device, points and models made untaught, files copied - and
   the master's label (§7.9).
2. **Backup** the target's whole product folder to
   `<root>/por_backups/<product>/<stamp>/` - outside `products/`, where `ListProducts`
   would mistake it for a product. *Restore backup* in the tool copies one back.
3. **Write** each file to a temp name and rename it into place, after the backup and
   only once the whole plan is in memory.
4. **Journal** the placeholders through the existing diff - `DiffPositions(before,
   after, <admin id>, "deploy POR <label>")` + `AppendPositionHistory` - so the position
   rule (a change is recorded by diff, never at a call site) holds for the tool too.
5. **Stamp** `<product>/por.json` (§7.9).
6. **Re-compare** and open the Fix list: it should now hold the teach list and nothing
   else, and the differences only what the target kept.

### 7.8 What FireDog must learn first (phase 4a)

- **`SavedPos::untaught`** (persisted `"untaught": true`, absent = taught):
  - movers refuse it at VALIDATION, before anything moves - `ValidateRecipe`'s
    `PositionUid` check, `FindPosition` as the run-time backstop, `move_path` for a
    route through an untaught node;
  - the UI's Go buttons (Motion page, the Jog window's Pos pad, the alignment station
    and grip inspect drives) are disabled on it, and the row is drawn as untaught;
  - **teaching it does not move first**: a teach mover whose position is untaught skips
    the approach and opens Manual Adjust where the device stands - the operator jogs
    to the spot and saves, which clears the flag. A teach run's validation allows an
    untaught position on a mover that has `teach = true`, and only there;
  - the Motion page's *Teach here* / Save clears it too;
  - `DiffPositions` journals the flag clearing, even when the values did not change.
- **The untaught point `{}`**: `ValidateParams` refuses a `PointXyz` that is an object
  without X/Y/Z, with "point not taught on this machine - Teach here in the step"; the
  editor's *Teach here* fills it as today.
- **`por.json`**: read with the product (read-only); Help > Version shows the POR label,
  where from, when, by whom, and *modified locally since* when a structure file's hash
  no longer matches; every `$run` carries `por` = the label and a short hash, so a unit
  says which plan of record built it.

### 7.9 POR identity - the label and `por.json`

The master has no revision counter, so the identity is made at **export**: *Export
master* asks for a **label** (`UAA2 2026-11-13`, free text) and records it in
`por_export.json` with a hash of each structure file (recipes, programs, scripts,
prompt images). Deploy copies that into `<product>/por.json`:

```json
{ "label": "UAA2 2026-11-13", "master": "D:/master_uaa2", "exported": "...",
  "deployed": "...", "by": "EN1023", "tool": "v0.25",
  "files": { "recipes": "<sha256>", "programs": "<sha256>", "scripts/x.lua": "<sha256>" } }
```

A deploy straight from a network root has no export record: the deploy window asks for
the label there and hashes the master's files itself. The hashes are SHA-256 through
Windows CNG (`bcrypt`), one helper in `core/`, used by the tool and FireDog alike.

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
- 1c. `App::AutoNickname`'s body becomes `StepSummary(step, StepSummaryDocs)`
  (`command/StepSummary.h`): the docs are pointers to the registry, recipes, positions,
  aligns, grips and trays (null = show the uid), output byte-identical;
  `AutoNickname` is the editor's adapter over `session_`. `FormatDurationMs` moved
  there from `ui/Widgets` with it.
- 1d. Reading a machine without touching it:
  - `LoadProductManifestAt(dir)` (`core/Product`): a manifest from ANY folder, named
    by the folder, `Dir()`/`FilePath()` answering under it (`ProductManifest::folder`).
    One parse shared with `LoadProductManifest(name)`; it never backfills a role.
  - `LoadMotionMapsFile(path)` (`core/MotionMap`): the maps parse without the
    machine fallback. Every other document already had a `Load*File(path)`.
  - `ResolveRootsReadOnly()` (`core/Paths`): where `InitPaths` WOULD put the roots,
    with no chdir, template ini, folder, migration or seed.
  - `Logger::SetFileBeforeFirstUse(path)`: the tool's own log file. Without it the
    first `FD_*` line would resolve `MachinePath("logs/...")` and so run `InitPaths`.
  - The tool's rule that follows: call nothing that resolves `MachinePath`/`DataPath`
    (the no-`File` loaders, `ProductSession`, `DutSession`, `HardwareConfigPath`).

**Phase 2 — the compare engine (plain C++, no UI, `src/por/` in `firedog_cmd`)**
- 2a. `PorSide` (`por/PorSide`): one side loaded read-only from a ROOT folder laid out
  like a machine root (`products/<name>/`, `config/hardware.json`,
  `config/maps.local.json`) — so the target is its machine root and a master export
  is a folder of the same shape. A role file that is empty falls back to
  `<root>/config/<role>.local.json`, as the app does. Hardware is reduced to the
  NAMES a recipe may use (enabled motion / IO + pins / PSUs / slides / light
  channels); scripts and prompt pictures to file names.
- 2b/2c. `ComparePor` (`por/PorCompare`): recipes first (by name; a name used twice
  on a side is Ambiguous), then the deck, whose recipe nodes take their recipe's
  result. `PorAlign` is the one sequence aligner (LCS, then same-type pairing in each
  gap) for slots, nodes and steps. Params go through `EffectiveParams` and are judged
  per `ParamType`: references resolved to names on each side (`Missing` when one
  does not resolve), `PointXyz` and `detect_object.model` as Measured, the rest by
  value. A row's `worst` rolls up for the explorer's badges.
- 2d. `ComparePositions` (`por/PorPositions`): the §6 trace and table — Deck tier in
  first-use order, Private poses matched through their owner, Unused, TargetOnly;
  untraced product scripts and missing required maps listed beside it.
- 2e. `CompareHardware` (`por/PorHardware`): §6a. `PorSide` keeps the whole
  `HardwareConfig` and `motion_transforms.json` for it.

**Phase 3 — `FireDogPor` (read-only)**
- 3a. Target + shell: `tools/por/`, links `firedog_cmd` + imgui/implot/glfw, own icon
  `.rc`, own `imgui.ini` and log, `firedog_stage_runtime`; pick master folder / target.
- 3b. Explorer + Deck/Recipes compare (§5.3).
- 3c. Positions page (§6) and Hardware page (§6a).
- 3d. *Export master* (§5.1).
- 3e. The Fix list (§6b): `por/PorFixList` + the first explorer page.
- Release: `package-release.ps1` already packs every exe; the `/release` skill's
  verify step learns the third exe.

**Phase 4 - deploy** (§7). FireDog learns the placeholders before anything can make one.
- 4a. FireDog: `SavedPos::untaught` (validation refusal, UI Go disabled, teach-in-place,
  journalled clearing) and the untaught point `{}` (§7.8).
- 4b. The plan, read-only: the Fix list's twin check moves onto `PorAlign`; `PorDeploy`
  translates both sides into the documents a deploy would write and the tool's Deploy
  window previews them, with the blockers. Nothing is written yet.
- 4c. Apply: process check, Admin sign-in against the target's roster, backup, temp +
  rename writes, journal, re-compare. *Restore backup*.
- 4d. Design-only copies of missing alignments, grip profiles and trays (§7.5); first
  deploy into a machine without the product (§7.6).
- 4e. POR identity: the export label + SHA-256 hashes, `por.json`, FireDog's Help >
  Version line and `por` on every `$run` (§7.9).

## 9. Open questions
- **Strict mirror by default?** Keeping target-only recipes and programs is the default
  (§7.3). A plant that wants every machine IDENTICAL would flip it - one checkbox
  either way.
- **Design params that are really per-machine.** Everything a step holds that is not a
  reference or a `PointXyz` is design and a deploy overwrites it: a relative-move delta,
  an approach zone, an exposure a machine was tuned to. If one of those turns out to be
  per-machine in practice, it should become a taught value or machine config - not an
  exception in the deploy. The compare shows each one before a deploy, so the first
  real deploys will say which.
- **Whether a placeholder keeps the master's axis values** (§7.4). They are never driven
  to; they only let the teach list say how far the taught value landed from the
  master's. Dropping them is simpler; keeping them costs nothing at run time.
