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
| `jogDevice_` / `jogArmed_` | the Jog window's Jog tab **and** its Pos tab (and the Aux window's Z arrows) |
| `ioArmed_` | the IO page's output toggles **and** the Jog window's IO tab |
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
   Note that `prevPage_` is only updated on a **non-takeover → takeover** switch,
   so a takeover that opens another takeover must name its return page itself.
   `Page::VisionTeach` is the case: it is reached only from an alignment station
   card, and its Close targets `Page::Align` explicitly.
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
`MeasuredHz`, `KeiRangeIndexFor`, `CameraView`/`MakeCameraView`, and the
`kFonts` / `kSteps` / `kKeiRanges` / `kUiSettingsPath` tables. Tables are
`inline constexpr` so they are one entity rather than a per-TU copy.

`CameraView` is the worked example of the hoist rule: the zoomable
cover-fit-plus-follow-the-mark mapping between camera and screen pixels lived in
`WndPrompts.cpp` until `PageVisionTeach.cpp` needed the *same* mapping. A pixel
has to mean the same thing in both surfaces, and two copies of that arithmetic
would drift the moment one gained a feature.

`Widgets.cpp` also implements **stb_image** (`LoadPngRGBA`), the project's only
PNG *reader* — `stb_image_write`'s implementation lives in `CommandsDut.cpp` and
the two headers are independent. It sits here because this TU already owns the
jobs that talk to the outside world (ShellExecute, the filesystem). Texture
lifetime is state, so the decode is a leaf helper and the GL texture belongs to
`App` (`EnsureMarkIcons`, `App.cpp`) — loaded once on first use, since a missing
`assets/` must cost a diagram rather than a retry every frame.

The **recipe icons** (`assets/recipe_icons/`) follow the same split.
`App::UiIcon` caches one texture per FILE stem — per file, not per caller,
because several commands share a glyph — and caches the failures too.
`StepIconFile` (Widgets) maps a command NAME to a stem, and `App::StepIcon` is
just those two composed. Adding an icon is a PNG plus one line in
`StepIconFile`: CMake copies the whole `assets/` tree, and `move_*` / `dut_*`
already fall through to their family glyph.

The set serves two shapes, and they differ on purpose:

- **Rows** — `DrawStepIconInline`, used by the palette and the step tree. It
  always RESERVES the column, so a type with no icon leaves a gap rather than
  shifting its neighbours out of line: a column the eye cannot run down is
  decoration. Both surfaces call the same lookup, which is the point — the
  glyph an operator learns while picking a command is the glyph they meet in
  the recipe it lands in.
- **Buttons** — `IconButton` / `IconSmallButton` (New, Save, run_execute, Stop
  on the Recipe Editor and the Run page). These degrade to a plain
  `ImGui::Button` when the icon is missing: a button is read where it sits, not
  scanned down a column, so a reserved blank square would only make it wider.
  Both draw an empty-labelled button and lay the icon + text over it centred —
  `Button()` centres ONE label and has no two-part form — and tint through
  `GetColorU32` so the icon dims with the text inside `BeginDisabled`.

`DrawMarkPicker` (`CameraMark` / `MarkPicker`) is the rule applied a second
time, one level up: the whole *mark a feature* surface — crosshair, click, wheel
zoom, arrow nudge, and latched marks projected from machine mm back through the
current pose — lived in `PageAlignNominal.cpp` until `PageAlignPlace.cpp` needed
it. It still holds no state: the crosshair and zoom are pointers to App members,
per the no-state rule above.

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
| `src/ui/pages/MainWindow.cpp` | menu bar, three-panel layout + splitters, page dispatch, safety-alert + exit modals, the watched-move banner (`StartWatchedMove` / `UpdateMoveWatch` / `DrawMoveStatusBar`) | 351 |
| `src/ui/pages/PageOverview.cpp` | left panel (status + DUT block + Display), vision middle, right panel, product popups, shared `DrawLogPanel` | 245 |
| `src/ui/pages/PageCamera.cpp` | Camera page + `UpdateCameraTexture` | 171 |
| `src/ui/pages/PageKeithley.cpp` | Keithley page + the whole chart pipeline (two consumers: page and Aux) | 364 |
| `src/ui/pages/PageCld.cpp` / `PagePsu.cpp` | CLD101x / UDP3305S pages | 179 / 160 |
| `src/ui/pages/PageLight.cpp` | CCS PD3 lighting page + `App::ApplyLightingNow` (what a `LightCalloutPicker` click lights) | ~250 |
| `src/ui/pages/PageMotion.cpp` | device list + live axes, tabbed detail, Positions tab (one selected row by uid, `posSelUid_`; Move / Update / Edit / Rename / Hist / Delete on a toolbar above the table, double-click = Move — every action still lands on its confirmation dialog), Scanning test bench | 489 |
| `src/ui/pages/PageMotionMap.cpp` | the Mapped Path graph editor (was the single largest function in App.cpp) | 511 |
| `src/ui/pages/PageIo.cpp` | IO page + Pneumatics page (slides are composed from IO pins) | 330 |
| `src/ui/pages/PageRecipeEdit.cpp` | editor page, step tree, palette, toolbar, run strip, step-path helpers | 879 |
| `src/ui/pages/RecipeForms.cpp` | `DrawRecipeInspector` / `DrawParamWidget` / `DrawConditionEditor` | 495 |
| `src/ui/pages/PageRun.cpp` | deck, execution view, Scan band (Raster \| XCAN), both launchers, preset modals | 1144 |
| `src/ui/pages/PageAlign.cpp` | Alignment page: procedures \| stations + Run \| the taught frame + its nominals | 400 |
| `src/ui/pages/PageVisionTeach.cpp` | Vision detection teach page (takeover, reached only from a station card): three columns — image + camera foot \| pre-process chain + method form + Detect/Live + reliability \| matched crop + results table + status | 1000 |
| `src/ui/pages/PageAlignNominal.cpp` | Nominal teach page (takeover, reached only from the frame panel): two crosshair marks, their midpoint converted through the live frame into local (O1) | 250 |
| `src/ui/pages/PageAlignPlace.cpp` | Placement page (takeover; **View menu** or a nominal's *Place…*): measures a gripped object the same way, computes the relative correction onto its holding device's axes and issues it behind the arming flag + a visible ceiling | 340 |
| `src/ui/pages/PageScripts.cpp` | Scripts page + Lua debugger controls | 215 |
| `src/ui/pages/PageDutView.cpp` | DUT Records viewer (read-only) | 150 |
| `src/ui/windows/WndViewData.cpp` | View Data window (View menu, read-only): every `dut.json` of the loaded product flattened into ONE table — a row per serial, a column per recorded key (last value wins), sortable, serial-filtered and time-preset filtered (this month / last month / YTD by `opened_at`) — over a chart of one or two keys: a series in table order, or daily / weekly / monthly box plots drawn by hand on the plot draw list. X labels are vertical text placed glyph-by-glyph under the plot (ImGui has no rotated text and `AddText` culls before a vertex could be turned), thinned so they never overprint. Double-click / Details opens the serial in DUT Records | 640 |
| `src/ui/pages/PageWatchEdit.cpp` / `PageTransformEdit.cpp` | the two full-page config editors | 135 / 131 |
| `src/ui/pages/PageNeedle.cpp` | Needle page (takeover, F7): the camera→needle lever arm, its measurement, the run parameters and the bootstrap seed. Launches `needle_calibration.lua`; actuates nothing itself | 302 |
| `src/ui/pages/PageDeviceReg.cpp` | Registration page (View menu / F8 only — no strip button, it is a one-time setup): a device's tip in camera space. Live image + `DrawMarkPicker` on the left; on the right the device pair over **four tabs** — Registration (record, both dates, footprint, fleet), Teach Pose (steps 1–2), Capture tool points (steps 3–4), Verify and test (check, Send tool here, Look at the tool). Steps and tabs turn green when the RECORD answers them. The teach is here rather than in a script because every step needs an operator at the machine; the page launches only `device_reg_check.lua` | 1220 |
| `src/ui/pages/PageGrip.cpp` | Gripped Object page (View menu / F9 only — no strip button, it is a one-time setup): where the object in a gripper's jaws sits, as an offset from that gripper's registered tip. Live image + `DrawMarkPicker` (zoom 0.5–3×, ratio badge top-left) with the TIP drawn beside the marks; on the right the profile list over **four tabs** — Profile (gripper, outline), Teach Pose (inspect pose, look at it), Find the object (marks/detector, reference), Measure. Steps and tabs turn green when the PROFILE answers them. It measures nothing a recipe cannot — `grip_measure` does that — it TEACHES what a good grip is, and shows the marks against the tip | 640 |
| `src/ui/pages/PageTray.cpp` | Waffle Tray page (strip button / F10, Technician): a grid of positions taught from three corners. The grid on the left — a button per cell: click marks it used / full (saved with the tray, mirrored to the Workflow cursor through `App::SetTrayCellsUsed`), the right-click menu drives there (the pose overload of `StartWatchedMove`) or disables it, headers toggle a whole line, *All full* / *All empty*; the controls on the right — tray list, name, device, size, three corner cards (Teach here · `TaughtPosePicker` · Go), pitch, validation. The moving is a recipe step (`move_to_cell`); the page teaches, masks and shows | 480 |
| `src/ui/windows/WndTray.cpp` | Waffle Tray window (View menu, Operator): the selected tray's grid through the page's own `DrawTrayGrid` — pick the active cell, mark cavities used / full, watch a run take them — kept open beside the Run page | 80 |
| `src/ui/windows/WndMotionInspect.cpp` | Inspect Motion window (Run strip button, Technician): read-only table of every move in what Execute would run (selected item, else the slot; sub-recipes inlined) — device, target, velocity, approach, exposure, with values a step leaves at 0 carried forward from the step that set them | 330 |
| `src/ui/pages/PageVersion.cpp` | Version page (takeover, Help menu, Admin): this build + the software update — Check / Download / Restart and install, release notes. Talks only to `AppUpdater` (owned by `main()`); see `docs/software-update-design.md` | 120 |
| `src/ui/windows/WndJog.cpp` | Jog window (Jog / Camera / **Pos** / **IO** tabs) + the shared velocity combo | 885 |
| `src/ui/windows/WndAux.cpp` | Aux window (camera + chart, own OS window) | 170 |
| `src/ui/windows/WndProgramOrganizer.cpp` | Program Organizer window (Run strip's Edit / View menu): recipe library \| slot tree with folders, loop counts, drag-drop; the ONLY program-editing surface | 520 |
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
