# UI file layout (layer 3)

*Landed 2026-07-31 as phase 3 of `app-refactor-plan.md`. This is the reference
for where UI code goes and why it is arranged this way — read it before adding
a page, a window, or a shared widget.*

App.cpp was **6,825 lines**. It is now a **237-line composition root**, with
every drawing function living in a per-surface translation unit. Nothing is over
~1,150 lines, and the two biggest files (`PageRun`, `PageRecipeEdit`) are the
two richest operator surfaces.

---

## The mechanism: member definitions across translation units

Every `Draw*` function is still a **member of `App`**, declared in `App.h`. The
page files just define them out of line:

```cpp
// src/ui/pages/PageCamera.cpp
#include "App.h"
namespace firedog {
void App::DrawCameraDetailPanel() { … }
}
```

Same multi-TU-one-unit shape as `command/Commands*.cpp`. **No context structs,
no friend declarations, no renames.**

### Why not a `UiContext` struct

The Motion page alone touches ~35 App members plus five services. A per-page
reference bundle would be a second name for every member, edited twice for every
member added. Immediate-mode UI wants direct state access; the bundle buys
nothing and costs a permanent synchronisation chore.

### Why not per-page classes

Much of the apparently page-local state is **not** local:

| State | Shared by |
|---|---|
| `xcan*` knobs | Motion Scanning tab **and** the Run page's Scan band |
| `vel*` maps | Jog window **and** Motion page |
| Keithley chart buffers | Keithley page **and** the Aux window |
| `leftW_` / `rightW_` / `lowerH_` | every page, plus UiSettings persistence |

Page classes would have to re-centralise all of it into a shared bag — i.e.
back to where we started, with an extra indirection.

### Costs accepted

`App.h` stays the single declaration header, so touching it recompiles ~20 small
TUs. Those build in parallel, so wall-clock is better than the one 6,825-line TU
it replaced. No PIMPL is needed: `App.h` contains no vendor headers.

---

## Rules for working in these files

1. **Page state stays an `App` member.** New file-scope *mutable* state in a
   page TU is a bug: `UiSettings` persistence and cross-page sharing both read
   App members. File-scope `constexpr` tables and `static`/anonymous-namespace
   *helper functions* are fine.
2. **A helper lives in the file that uses it.** Only hoist to `ui/Widgets` when
   a second TU genuinely needs it — see the next section.
3. **Never assign `currentPage_` directly.** All page switches route through
   `App::SwitchPage`, which owns the takeover/`prevPage_` bookkeeping and the
   dirty-recipe guard. The one documented exception is the Run deck's
   right-click-to-edit shortcut.
4. **The layer rule is unchanged.** A page reads `HardwareManager::Snapshot()`
   / accessors and the DataBus (pull-`Snapshot` only, no subscriptions), and
   issues every command through `Workflow`. No page constructs a device or runs
   a connect.
5. **Gate every Run/Go surface on `orch_.Busy()`** — never on a subset of the
   engines. See `command/Orchestration.h`.
6. **Document data goes through `session_`**, never through a page-local copy.
   Build engine `Env`s with `session_.MakeRecipeEnv()` / `MakeLuaEnv()`.

### Moving code between these files

ImGui IDs derive from the **runtime** stack, not source location, so relocating
a function is behaviour-preserving *as long as* window/child/label strings and
call order stay byte-identical. Do the move and the improvement in separate
commits, and verify with `git diff --color-moved`.

---

## `ui/Widgets.{h,cpp}` — the shared leaf helpers

Stateless helpers and lookup tables with **more than one consumer**:
`StateColor`/`Glyph`/`Text`, `DeviceStatusLine`, `Splitter`, `FormatCurrent`,
`MeasuredHz`, `KeiRangeIndexFor`, and the `kFonts` / `kSteps` / `kKeiRanges` /
`kUiSettingsPath` tables. Tables are `inline constexpr` so they are one entity
rather than a per-TU copy.

Two hard constraints:

- **`Widgets.cpp` is the only UI TU that includes `windows.h`** (it backs
  `OpenFolderInExplorer`, the sole `ShellExecuteA` user). Keep it that way — no
  page file should have to deal with the macro fallout.
- **`WndPrompts.cpp` is the only file that includes `imgui_internal.h`**, for
  `BringWindowToDisplayFront`.

Deliberately *not* hoisted, because each has exactly one home: `kWinW`/`kWinH`
(App.cpp), `kVelPI`/`kVelACS` (WndJog), `LevelColor` (PageOverview),
`ShiftToRelative` (PageKeithley).

---

## The file map

| File | Contents | LOC |
|---|---|---|
| `src/App.cpp` | **composition root**: `Run()` (startup + frame loop), `Update()`, `SwitchPage`/`ToggleEditorPage`, the ProductSession UI wrappers | 237 |
| `src/ui/Widgets.{h,cpp}` | shared leaf helpers + tables; sole `windows.h` user | 110 |
| `src/ui/pages/MainWindow.cpp` | menu bar, three-panel layout + splitters, page dispatch, safety-alert + exit modals | 351 |
| `src/ui/pages/PageOverview.cpp` | left panel (status + DUT block + Display), vision middle, right panel, product popups, shared `DrawLogPanel` | 245 |
| `src/ui/pages/PageCamera.cpp` | Camera page + `UpdateCameraTexture` | 171 |
| `src/ui/pages/PageKeithley.cpp` | Keithley page + the whole chart pipeline (two consumers: page and Aux) | 364 |
| `src/ui/pages/PageCld.cpp` / `PagePsu.cpp` | CLD101x / UDP3305S pages | 179 / 160 |
| `src/ui/pages/PageMotion.cpp` | device list, tabbed detail, Positions tab, Scanning test bench, info panel | 489 |
| `src/ui/pages/PageMotionMap.cpp` | the Mapped Path graph editor (was the single largest function in App.cpp) | 511 |
| `src/ui/pages/PageIo.cpp` | IO page + Pneumatics page (slides are composed from IO pins) | 330 |
| `src/ui/pages/PageRecipeEdit.cpp` | editor page, step tree, palette, toolbar, run strip, step-path helpers | 879 |
| `src/ui/pages/RecipeForms.cpp` | `DrawRecipeInspector` / `DrawParamWidget` / `DrawConditionEditor` | 495 |
| `src/ui/pages/PageRun.cpp` | deck, execution view, Scan band (Raster \| XCAN), both launchers, preset modals | 1144 |
| `src/ui/pages/PageAlign.cpp` | Alignment page: procedures \| stations + Run \| the taught frame | 320 |
| `src/ui/pages/PageScripts.cpp` | Scripts page + Lua debugger controls | 215 |
| `src/ui/pages/PageDutView.cpp` | DUT Records viewer (read-only) | 150 |
| `src/ui/pages/PageWatchEdit.cpp` / `PageTransformEdit.cpp` | the two full-page config editors | 135 / 131 |
| `src/ui/windows/WndJog.cpp` | Jog window + the shared velocity combo | 240 |
| `src/ui/windows/WndAux.cpp` | Aux window (camera + chart, own OS window) | 170 |
| `src/ui/windows/WndPrompts.cpp` | global `prompt` + `manual_adjust` + alignment crosshair-pick windows; sole `imgui_internal.h` user | 360 |

CMake needs no edits: `GLOB_RECURSE … CONFIGURE_DEPENDS` picks up new files.

---

## Two invariants worth restating

**`kElsePathBase` (PageRecipeEdit.cpp) must stay an alias of `kElseBranchBase`
(`command/Recipe.h`).** Step-path components at or above it address an if-step's
else branch. Re-typing the literal would desync editor selection from the paths
the validation layer emits.

**Engine `Status` is read per frame, never cached across frames.** The prompt
and manual_adjust windows edge-detect a *new* pending request off those
per-frame reads.
