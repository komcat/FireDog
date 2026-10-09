# The machine root: where FireDog keeps its files

FireDog writes nothing beside its own exe. Everything a machine learns — its
wiring, its calibrations, its products, its operators, its production records
— lives under a **machine root** that a build, a release zip and a deleted
build folder never touch. `core/Paths.{h,cpp}` is the one place that decides
where that is.

## Why

Before this, every path in the code was a cwd-relative literal
(`"config/x.json"`, `"products"`, `"logs/…"`, `"data/xcan"`), and the cwd was
the exe folder. So a machine's state lived *inside the build tree*:

- a fresh build tree or an unzipped release started the machine over (taught
  positions, the roster, the needle calibration, every DUT record);
- `hardware.json` and `motion_transforms.json` are edited **in place** — they
  have no `xxx.local.json` pair — so a build that copied the repo's `config/`
  silently reverted the machine's wiring. `cmake/StageConfig.cmake` learned
  "copy only when missing" to stop that, which is a patch on the symptom;
- the GUI never set its own cwd (only the rig did), so a shortcut with another
  "Start in" read the wrong config and created an empty `products/` wherever.

## Three places, one owner each

| Place | Holds | Who writes it |
|---|---|---|
| **exe folder** | `config/scripts/`, the `config/*.json` **defaults**, `assets/`, the DLLs | the build / the release. The app never writes here (one exception: the `FireDog.ini` template below, written only when missing). |
| **machine root** (default `D:\ProgramData\FireDog`) | `config/hardware.json`, `config/motion_transforms.json`, every `config/*.local.json` and its `*.history.jsonl` journal, `config/vision/`, `config/scripts/` (machine copies), `products/<name>/` documents, `products/employees.json`, `logs/firedog_*.log`, `imgui.ini`, `firedog-config-report.txt`, `migrated_from.txt` | the app (UI thread / the documented hand-offs), the rig's `--save` |
| **data root** (default `<root>\data`) | `products/<name>/data` + `images` (DUT records), `xcan/`, `align_attempts/`, `mosaic/` (when the Mosaic page's folder is left at its default) | the app — production output, the bulky part |

`MachinePath(rel)` and `DataPath(rel)` build the two; shipped files keep their
relative spelling (`"config/recipes.json"`) because `InitPaths` makes the exe
folder the working directory.

The **`xxx.local.json` → `xxx.json` fallback is unchanged in meaning**: the
local file is now `<root>/config/xxx.local.json`, the default is still the
exe's `config/xxx.json`. "Look at the machine root first" is exactly that
fallback.

`HardwareConfigPath()` / `MotionTransformsPath()` (`hardware/HardwareConfig.h`)
are the one spelling of the two in-place files; the pages, `App` and the rig
all go through them.

## `FireDog.ini`

Beside the exe:

```ini
[paths]
root = D:\ProgramData\FireDog
data = E:\FireDogData        ; optional; default <root>\data
```

- Read once, at startup. A relative value is taken from the exe folder;
  `%VARIABLES%` expand.
- **Never shipped** — `package-release.ps1` does not put one in the zip — so
  unzipping a new version over an install cannot reset where the machine keeps
  its state. When none exists the app writes a commented template (all
  defaults) for the next reader, best effort.
- A dev who wants a sandbox drops one into a build tree: `root = state`.

**With no `root` in the ini, the default depends on the PC**: `D:\ProgramData\FireDog`
when `D:` is a FIXED disk, else `%ProgramData%\FireDog` (normally `C:`) — an
info note, not an error, since a C:-only PC is a normal machine. Removable
media does not count as a D: drive: a USB stick present on some boots would
flip the root between starts, and each flip would look like a first start
against an empty folder (and migrate the exe-side state again into it).

**If the root cannot be created** (an ini `root` on a missing drive, a
read-only folder), the app falls back to
`%ProgramData%\FireDog` and logs an ERROR naming the ini to fix; if even that
fails, it keeps state beside the exe (the old behaviour). It never refuses to
start — the login gate and the hardware still come up, and the log says why
the state is not where it should be.

## First start: copy, never move

`InitPaths` treats a machine root without `config/` as a first start and
**copies** the old exe-side state into it (`skip_existing`, never deleting a
source):

| From (exe folder) | To |
|---|---|
| `config/*.local.json`, `config/*.history.jsonl`, `config/vision/` | `<root>/config/` |
| `products/<name>/…` except its data/images folders (named by the manifest) | `<root>/products/<name>/` |
| `products/<name>/data`, `images` | `<data>/products/<name>/` |
| `products/employees.json`, anything else in `products/` | `<root>/products/` |
| `logs/*.log` | `<root>/logs/` |
| `logs/align_attempts/` | `<data>/align_attempts/` |
| `data/xcan/` (and anything else in `data/`) | `<data>/` |
| `imgui.ini` | `<root>/` |

Then, on **every** start, the two in-place files are **seeded** when missing:
`<root>/config/hardware.json` and `motion_transforms.json` are copied from the
exe folder. On an existing machine that copy IS the machine's wiring
(StageConfig never overwrote it); on a fresh install it is the repo default.

`migrated_from.txt` in the root lists what was copied. The originals stay, so
an older exe still runs from what it had — delete them once the machine runs
well. Only the FIRST build tree started against an empty root is migrated; a
second tree's state is not merged in.

**`StageConfig.cmake` keeps "copy only when missing".** It no longer protects
anything the new exe reads, but it does protect the *migration*: a machine
that builds the new version before running it would otherwise have its
exe-side `hardware.json` replaced by the repo's, and the first start would seed
the root from the wrong copy.

## Scripts

Three folders, consulted in order by `ExecutionContext::ResolveScript`:

1. the product's `scripts/` (when a product is loaded and has any),
2. the machine's `<root>/config/scripts/` (`MachineScriptsDir()`),
3. the path the caller named — normally the shipped `config/scripts/<file>`.

**Standard procedures (`IsSystemScript`) skip all of this** and always run the
shipped copy, as before.

**A shipped spelling whose file is missing beside the exe falls back to the
machine root.** `LocateShippedScript` turns `config/scripts/<file>` into
`<root>/config/scripts/<file>` when the exe's copy is absent and the root's
exists — for an install whose exe folder holds only the exe (its `config/`
renamed away after the migration). `SystemScriptPath`, `LuaEngine::Start` and
`RunLuaFile` all apply it, so a standard procedure, a typed scan's default
`file` and a UI launcher's literal default all find it. The exe's copy still
wins whenever it exists — it is the one built with the commands it calls — so a
rebuild, which restages `config/scripts/`, takes over from the root's copy.

Without a product the Scripts page lists the machine folder *and* the shipped
one (a machine copy wins by name). Saving a shipped, non-standard script writes
the machine's copy instead (`SaveScriptEditor`), which then runs; New Script
creates in the machine folder. Editing a standard procedure still writes the
shipped file, to be mirrored into the repo.

## Mosaic

`MosaicConfig::saveDir` defaults to `<data>/mosaic`. The default is stored as
`""` in `mosaic.local.json` so it follows the data root if the ini moves it; a
folder somebody chose is stored as typed. A machine whose file already says
`C:\mosaic` keeps it.

## Rules

- **A new runtime-written file goes under `MachinePath` or `DataPath`** — never
  a bare relative path, which now means "beside the exe".
- **`InitPaths` never logs.** The logger's own file is `MachinePath("logs/…")`,
  so a log call inside the `call_once` would deadlock. It keeps `PathNotes()`;
  `App::Run` and the rig print them once the logger is up.
- **Copy, never move**, in the migration. A rollback must stay possible.
- **The ini is never shipped.**
