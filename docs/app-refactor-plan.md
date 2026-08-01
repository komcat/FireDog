# App.cpp / App.h refactoring plan

*Prepared 2026-07-30 (overnight review; three analysis passes: structure/coupling map,
UI-module split design, state/ownership split design). Decision-ready — review, adjust,
then execute phase by phase. Every step compiles green and changes zero behavior unless
flagged.*

App.cpp is **6,330 lines** (5,927 non-blank), App.h 449. The two biggest clusters are
the **Recipe editor (1,149 L)** and the **Motion page (1,015 L)**; Run page + Scan band
add 724 L. `DrawMotionMapTab` alone is 481 lines — the single largest function.

---

## The recommendation in three sentences

1. **Split mechanism:** keep every `Draw*` function a **member of `App`**, but move the
   *definitions* into ~20 per-page translation units (`src/ui/pages/*`, `src/ui/windows/*`)
   — the same multi-TU-one-unit shape as `command/Commands*.cpp`. No context structs, no
   friends, no renames; diffs are pure moves (`git diff --color-moved`).
2. **State split:** before moving UI files, extract two owner objects — a UI-thread-only
   **`ProductSession`** (positions / maps / recipes / programs / product lifecycle /
   scripts list + all their load/save/dirty logic + `MakeRecipeEnv()/MakeLuaEnv()`) and an
   **`Orchestration`** aggregate (registry + both engines + ProgramRunner + `Busy()` +
   `StopAll()`), preserving the destruction-order discipline in one member slot instead
   of four.
3. **Order:** Orchestration → ProductSession → Widgets extraction → page-file moves →
   optional layer cleanups. State-first because the persistence cluster (C7, 278 L)
   otherwise gets moved into a new UI file and then immediately dissolved into the
   session — double churn on the same lines.

---

## What the mapping pass found (beyond size)

**Coupling core** (what every page touches): `hardware_` (37 functions), `workflow_` (24),
`savedPos_` (16), `product_` (16), `recipes_` (14), `motionMaps_` (11), the engines (17
combined). 72 members are used by exactly ONE function; whole clusters (map editor state,
scan-tab knobs, jog state…) are trivially movable with their page.

**Bugs / dead code surfaced (worth fixing regardless of the refactor):**

- **Gate drift bug:** the Motion page Go button gates on `recipeEngine_.Running() ||
  luaEngine_.Running()` but not `programRunner_.Active()` — between queue items a
  program run is momentarily "idle", so Go can start a path move that then makes the
  runner's next recipe start fail. Unifying every gate on `Orchestration::Busy()` fixes
  it (flag as deliberate behavior fix in that commit).
- **Dead code:** `DrawObjectCanvas` (App.cpp ~6005) has **no caller**; `objects_`,
  `dragging_`, `CanvasObject` exist only for it. Delete in phase 0.
- **Six duplicated Env-construction sites** hand-copy `savedPos_/motionMaps_/recipes_`
  into `RecipeEngine::Env`/`LuaEngine::Env` — the "forgot to copy library" failure mode.
  `session.MakeRecipeEnv()/MakeLuaEnv()` collapses them.
- **Two independent launchers** of the same two Lua scripts (Run-page Scan band vs
  Motion-page Scanning tab) duplicate initialCtx assembly — candidate for a shared
  launcher on the session/orchestration boundary later (not urgent).
- Cross-cluster leaks to be aware of (not bugs): the Run page writes `recipeSel_` /
  `edStepSel_` / `currentPage_` directly (documented exception); `DrawKeithleyChart`
  writes UiSettings (Scope checkbox); `StopMachine` lives in the Run cluster but is
  called from the Jog panel.

---

## Decision 1 — split mechanism: member definitions across TUs

Each new file does `#include "App.h"` and defines `void App::DrawMotionListPanel() {…}`
out of line. Rationale (against the alternatives):

- **vs. a `UiContext` struct:** the Motion page alone touches ~35 members + 5 services;
  a per-page reference bundle is a second name for every member, edited twice per member
  addition. Immediate-mode UI wants direct state access.
- **vs. per-page classes:** much of the "page-local" state is *not* local —
  `xcan*` knobs are shared by the Scanning tab and the Run band; `vel*` by Jog and
  Motion; chart buffers by Keithley page and Aux; splitter sizes by all pages +
  UiSettings persistence. Page classes would re-centralize into a shared bag anyway.
- **Costs accepted:** App.h stays the single declaration header (touching it recompiles
  ~20 small TUs — parallel, so wall-clock improves vs. one 6,330-line TU). No PIMPL
  needed: App.h already contains no vendor headers.
- **CMake:** `GLOB_RECURSE … CONFIGURE_DEPENDS` picks up new files — zero build edits.

Rules during moves (behavior safety): window/child/label strings and call order stay
byte-identical (ImGui IDs derive from the runtime stack, not source location); no
"improvements" mid-move; new page TUs must NOT grow file-scope mutable state (page state
stays App members so UiSettings persistence keeps working). There are currently **no
mutable file-scope statics** in App.cpp — keep it that way.

## Decision 2 — state & ownership

### 2a. `Orchestration` aggregate (src/command or src/App.h-local)

```cpp
struct Orchestration {
    CommandRegistry registry;    // declared first — engines hold const ref
    RecipeEngine    recipe;
    LuaEngine       lua;
    ProgramRunner   programs;
    bool Busy() const;           // recipe || lua || programs.Active() || wf.PathActive()
    void StopAll();              // current App::StopMachine body
};
// In App.h — SAME slot as today, AFTER workflow_ (destruction-order comment moves too):
Orchestration orch_{workflow_, hardware_};
```

One member slot to protect instead of four; ~60 mechanical call-site renames. Do NOT
give it a ProductSession reference — Env stays caller-built (copy-at-Start is the
concurrency contract). Prompt/adjust window state (`promptWasPending_` etc.) stays UI
state and moves with the prompt windows file.

### 2b. `ProductSession` (new `src/session/ProductSession.{h,cpp}`)

UI-thread-only application document. **No locks, no ImGui, no Workflow/HardwareManager
includes.** Owns: `product_`, `savedPos_`, `motionMaps_`, `recipes_` (+uid/baseline),
`programs_` (+uid/baseline), `scripts_` list + scripts dir. API: mutable collection
refs (`Positions()`, `Recipes()`, …), load/save per collection with the product-file /
`config/*.local.json` fallback in ONE place, `RecipesDirty()/ProgramsDirty()`
(snapshot-compare, unchanged), `LoadProduct/NewProduct/SaveProduct/LoadInitial`,
`MakeRecipeEnv()/MakeLuaEnv()`, and `MergeTaughtPositions()` (body of the teach-queue
merge; **the call stays in `App::Update`** — that's the worker→UI hand-off point).

Why it's cheap here: engines copy state via Env at Start (zero thread safety needed);
dirty tracking is snapshot-compare (mutable refs keep working, no observers); the
fallback convention is currently smeared over five functions.

UI-selection resets on load (`recipeSel_ = …`) stay in thin App wrappers — no
session→UI callbacks.

### 2c. Explicitly skipped (over-engineering)

- Generic document/undo/observer framework; setter-based dirty tracking.
- Thread-safe session (copy-at-Start IS the model).
- Moving `UiSettings` (user/machine prefs, saved from ~10 interaction sites) or
  `watchCfg_`/`transformCfg_` (hardware-config working copies — they belong to their
  editor pages).

## Decision 3 — target file layout (after all phases)

| File | Contents | ~LOC |
|---|---|---|
| src/App.cpp | Run(), Update() (+teach drain call), SwitchPage/Toggle, UpdateCameraTexture | ~300 |
| src/session/ProductSession.{h,cpp} | §2b | ~450 |
| (Orchestration) | §2a — small; header-mostly | ~80 |
| src/ui/Widgets.{h,cpp} | cross-TU helpers: StateColor/Glyph/Text, DeviceStatusLine, Splitter, FormatCurrent, MeasuredHz, kSteps, kFonts, kKeiRanges+KeiRangeIndexFor, OpenFolderInExplorer (sole windows.h), kUiSettingsPath | ~290 |
| src/ui/pages/MainWindow.cpp | DrawMainWindow, safety + exit modals | ~560 |
| src/ui/pages/PageOverview.cpp | left panel, DUT block, overview panels, log panel | ~330 |
| src/ui/pages/PageCamera.cpp / PageCld.cpp / PagePsu.cpp | instrument pages | 150–170 ea |
| src/ui/pages/PageKeithley.cpp | page + chart pipeline (UpdateKeithleyChartData/DrawKeithleyChart) | ~430 |
| src/ui/pages/PageMotion.cpp | list/detail/scan-tab/positions-tab, DrawVelocityCombo, info | ~560 |
| src/ui/pages/PageMotionMap.cpp | DrawMotionMapTab | ~490 |
| src/ui/pages/PageIo.cpp | IO + Pneumatic panels | ~320 |
| src/ui/pages/PageRecipeEdit.cpp | editor + step-path anon-ns helpers + run strip | ~780 |
| src/ui/pages/RecipeForms.cpp | DrawRecipeInspector/DrawParamWidget/DrawConditionEditor | ~480 |
| src/ui/pages/PageRun.cpp | deck + Scan band + launchers (kXcanTargets/PresetButton/FormatSI stay local) | ~730 |
| src/ui/pages/PageScripts.cpp / PageDutView.cpp / PageWatchEdit.cpp / PageTransformEdit.cpp | one each | 105–215 ea |
| src/ui/windows/WndJog.cpp / WndAux.cpp / WndPrompts.cpp | floating windows (WndPrompts is the sole imgui_internal.h user) | 170–200 ea |

Nothing over ~800 lines. Note: no `AppPersist.cpp` — that cluster dissolves into
ProductSession (this is the concrete reason state goes first).

## Phase plan (each numbered item = one compile-green commit)

**Phase 0 — free wins** — ✅ DONE (commit `47364c3`)
1. ~~Delete dead `DrawObjectCanvas` + `objects_`/`dragging_`/`CanvasObject`.~~

**Phase 1 — Orchestration** — ✅ DONE
2. ~~Aggregate registry+engines+runner into `orch_` (same member slot, comment moved);
   move `OrchestrationBusy()`/`StopMachine()` in; unify ALL gates on `Busy()`
   (⚠ includes the Go-button behavior fix — call it out); rename ~60 call sites.~~
   Landed as `src/command/Orchestration.{h,cpp}`; `orch_.registry/recipe/lua/programs`
   + `Busy()`/`StopAll()`; 62 call sites; Go-button gate fixed. CLAUDE.md +
   run-page-design.md updated.

**Phase 2 — ProductSession** (all renames inside the still-single App.cpp) — ✅ DONE
3. ~~Skeleton: positions + maps (+ teach-merge method; call stays in Update()).~~
   Also took `product_` — `SavePositions` can't resolve its target path without it.
4. ~~Recipes + dirty (SwitchPage guard → `session_.RecipesDirty()`).~~
5. ~~Programs.~~
6. ~~Product lifecycle + scripts + DutSession wiring; `Run()` startup collapses to
   `session_.LoadInitial(lastProduct)`.~~ UI half kept in App as
   `ResetDocumentSelections()` + `SyncScriptSelection()`.
7. ~~`MakeRecipeEnv/MakeLuaEnv` replace the six Env sites.~~ There were **seven**.

**Phase 3 — UI file split** (pure moves; verify `--color-moved`) — ✅ DONE
8. ~~Widgets.{h,cpp} extraction.~~
9. ~~PageCamera, PageCld, PagePsu.~~
10. ~~PageKeithley + chart cluster.~~
11. ~~PageIo (+ pneumatic).~~
12. ~~WndJog, WndAux, WndPrompts.~~
13. ~~PageWatchEdit, PageTransformEdit, PageScripts, PageDutView.~~
14. ~~PageMotion + PageMotionMap.~~
15. ~~PageRecipeEdit + RecipeForms, then PageRun.~~
16. ~~MainWindow.cpp; prune App.cpp includes.~~ App.cpp is **237 L**. Result documented
    in `docs/ui-file-layout.md`; CLAUDE.md Layer 3 points at it.

⚠ **Correction to the phase-3 method.** The plan assumed each page commit would be
compile-green on its own. It was not: several pages reference file-scope `static`
helpers (`SlideColor`, `PresetButton`, `FormatSI`, the XCAN preset JSON helpers…)
that the per-function carve left behind in App.cpp until commit 16, and the moved
TUs needed includes App.cpp had provided. **Only the phase-3 end state builds.**
If phase 3 is ever re-run or bisected, carve each page's file-scope statics in the
SAME commit as its members, and re-check includes per TU.

**Phase 4 — optional layer cleanups (small, independent)** — ✅ DONE
17. ~~`NearestMapNodeUid` (Go button geometry) → core/MotionMap (one planner spirit).~~
18. ~~`BuildRunQueue` (StartDeckRun resolution) → command/Program.~~ Landed in
    **command/ProgramRunner**, not Program: the result *is* `ProgramRunner::Item`, so
    Program.h would have had to depend on its own executor. `int mode` → `RunScope`.
19. ~~Seed-"Home" placeholder out of `DrawMotionPositionsTab` draw path → session.~~
    Driven from `App::Update`, not startup — the row is shaped from the device axis
    list, which does not exist until the async connect finishes (the reason it was in
    the draw path at all). Fixed a latent bug on the way: the seeded row was never
    persisted, so its uid changed every launch and anything referencing it broke.

---

## Outcome

`App.cpp` **6,330 → ~250 lines** (it had grown to 6,825 by the time execution
started). Three owners now exist where `App` had everything:

| | |
|---|---|
| `command/Orchestration.{h,cpp}` | registry + both engines + runner, one member slot, one `Busy()` gate |
| `session/ProductSession.{h,cpp}` | the product document + all its persistence rules + the Env factories |
| `src/ui/pages/`, `src/ui/windows/` | ~20 per-surface TUs — see `docs/ui-file-layout.md` |

Behavior changes were deliberate and are called out in their commits: the
Motion-page Go gate (phase 1) and the Home-seed persistence (phase 4).

## Risk register

- **Teach queue (commit 3):** producers are engine worker threads, consumer must remain
  `App::Update` on the UI thread. ProductSession documented UI-thread-only.
- **Engine status:** `GetStatus()` per-frame reads are safe; do NOT cache a Status in
  Orchestration across frames (prompt windows edge-detect on per-frame reads).
- **DataBus:** layer 3 uses pull-`Snapshot` only — no subscriber lifetimes involved.
- **ImGui IDs:** byte-identical labels/order during moves (see Decision 1 rules).
- **windows.h:** only Widgets.cpp keeps it (WIN32_LEAN_AND_MEAN/NOMINMAX before it);
  `imgui_internal.h` only in WndPrompts.cpp.
- **`kElsePathBase`** stays the alias of `kElseBranchBase` (Recipe.h) next to the
  step-path helpers — a re-typed literal would desync editor selection from validation
  paths.
- Build verification per commit: configure ok; **user builds and runs** (project rule);
  commits 4 and 6 deserve a careful manual pass (dirty-guard modal, last-product
  auto-reload, product-fallback save paths).

## Later / out of scope

- Per-page state structs inside App.h (cosmetic declutter, per-cluster, optional).
- Unifying the two scan launchers (Run band vs Scanning tab) over one helper.
- A `docs/` design-doc + CLAUDE.md section update should land with Phase 2 (session)
  and Phase 3 (pages) per the project convention.
