# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**This file is the map and the rules.** The detail lives in `docs/`, one topic per
file — open the relevant one *before* changing that subsystem. What is kept here
is what prevents a mistake when nothing else has been read.

## What FireDog Is

A **lightweight C++20 controller for a universal active-alignment machine**. Real drivers exist for: Basler camera (Pylon), Keithley 6482 (VISA), Thorlabs CLD101x laser/TEC (VISA), UNI-T UDP3305S 3-channel PSU (VISA), PI hexapods + ACS gantry (motion), FASTECH Ezi-IO + WAGO Modbus (digital IO), plus synthetic pneumatic slides, an IO watch, and a safety watch built on the IO fleet. Still a simulated placeholder: SMU.

Sibling projects under `..\komcat\` are the reference implementations for the same domain: **`bobcat_engine`** (mature Qt6 version, richest domain map) and **`BobDog`** (a raylib+rlImGui version — useful for how the PI/ACS/Pylon SDKs get wired into CMake). FireDog deliberately does **not** use raylib.

Entry point is `FireDog.cpp` → `firedog::App::Run()`, whose main loop is `while (!host_.ShouldClose())` over the GLFW `GlHost`. Everything is in namespace `firedog`.

## Build

```bash
cmake --preset x64-debug             # configure (presets: x64/x86 × debug/release)
out/build/x64-debug/FireDog.exe      # run the client
out/build/x64-debug/FireDogRig.exe   # console hardware bring-up rig
```

Three targets: **`firedog_hw`** (static lib — `src/core` + `src/hardware` +
`src/vision` + `src/workflow`, and where all vendor-SDK gating lives),
**`FireDog`** (the GUI client), **`FireDogRig`** (the console rig, sources in
`tools/rig/`).

CMake + Ninja + MSVC, **Windows only**. Deps come from `FetchContent` (first configure needs network). Vendor SDKs are optional, each gated by a compile def (`HAS_PYLON`/`HAS_VISA`/`HAS_PI`/`HAS_ACS`/`HAS_EZIIO`/`HAS_HALCON`) with a simulated stub fallback, so the build always stays green — details in `docs/build-and-sdks.md`.

**The user compiles and tests themselves** (global preference). Do **not** run `cmake --build …` or launch the app — `cmake --preset` is fine; leave building and running to the user.

## Where things are

| Layer | Path | Role | Topic doc |
|---|---|---|---|
| 1 — Hardware | `src/hardware/` | drivers + interfaces; `HardwareManager` owns every device | `docs/hardware-layer.md` |
| 1.5 — Vision | `src/vision/` | focus metric (plain C++), HALCON availability/licence, detection models + detectors | `docs/vision-layer.md`, `docs/vision-detection-design.md` |
| 2 — Workflow | `src/workflow/` | builds devices from config, drives connect/poll, jog kinematics, motion/IO commands, safety | `docs/architecture.md` |
| 2.5 — Commands | `src/command/` | `CommandRegistry`, `RecipeEngine`, `LuaEngine`, `Orchestration` | `docs/command-layer.md` |
| — Document | `src/session/` | `ProductSession`: product + positions + maps + recipes + programs + scripts + alignments | `docs/state-and-config.md` |
| 3 — UI | `src/App.*`, `src/ui/` | GLFW+OpenGL3 host, ImGui pages, ImPlot charts | `docs/ui-layer.md`, `docs/ui-file-layout.md` |
| — Tools | `tools/rig/` | `FireDogRig`, the console bring-up rig (layers 1–2, no UI) | `docs/rig-console.md` |

### Topic docs (`docs/`)

**Reference — how it works now:**
`architecture.md` (layering + ownership) · `build-and-sdks.md` (**targets + SDK gating**) · `hardware-layer.md` (per-device drivers) · `vision-layer.md` (focus metric, auto-focus, HALCON) · `state-and-config.md` (config files, persistence, DataBus) · `command-layer.md` (registry, recipes, Lua) · `run-and-dut.md` (programs, Run page, traceability) · `ui-layer.md` · `ui-file-layout.md` (**where UI code goes**) · `rig-console.md` (the FireDogRig console)

**Design rationale — why it is that way:**
`recipe-core-design.md` · `recipe-editor-design.md` · `lua-tier-design.md` · `run-page-design.md` · `dut-design.md` · `xcan-scan-design.md` (the align algorithm) · `alignment-procedure-design.md` (the taught local frame O1) · `vision-detection-design.md` (detection models, the teach wizard, auto stations) · `pi-hexapod-motion-design.md` · `keithley-polling-design.md` · `app-refactor-plan.md` (the 2026-07 App.cpp decomposition, done)

## Rules

Breaking any of these is a defect, not a style choice.

### Layering
- **Dependency direction is one-way: UI → workflow → hardware.** The UI never touches a device directly.
- **Layer 1 owns, layer 2 drives, layer 3 renders.** `HardwareManager` is the sole owner of every device. Do **not** move device ownership or connect logic into `App`.
- Connection state is `ConnState` — never reintroduce a parallel int-status encoding.
- Exception, deliberate: **`SafetyWatch` is owned by `Workflow`**, because its callback stops motion through the Workflow object and so must die first.

### Threading
- **No blocking call on the UI thread.** Engine waits go through `ExecutionContext::Sleep/WaitUntil` (every wait has a timeout) so Stop interrupts within one poll interval. `PneumaticSlide::ExtendAndWait`/`RetractAndWait` block — never call them from the UI.
- **No detached threads, ever.** Each engine runs ONE worker; the dtor stops and joins it.
- `ProductSession` is **UI-thread only, no locks** — engines receive a *copy* through `Env` at Start. That copy-at-Start IS the concurrency contract.
- The session never calls back into the UI; `App` re-points its own selections after a load.

### State & documents
- Build a run's `Env` **only** via `session_.MakeRecipeEnv()` / `MakeLuaEnv()` — hand-copying the fields is how a caller forgets the library and breaks `sub_recipe`.
- **A Draw function must never mutate the document.** Seeding/derivation belongs in `App::Update` or the session.
- Runtime-writable JSON follows the `xxx.local.json` → `xxx.json` fallback (local files gitignored).
- A rebuild recopies the repo's `config/` over the exe-side copy — mirror durable edits back into the repo file.
- Never hard-code DataBus topic strings; they live in `core/Topics.h`.

### Orchestration
- **Every Run/Go surface gates on `orch_.Busy()`** — never on a subset of the engines. The program runner goes momentarily idle between queue items, so checking the engines alone lets a second run slip in.
- Recipes never connect hardware; validation at Start requires devices already `Connected`.
- Ctx does **not** flow between recipes — cross-recipe data goes through the DUT or the DataBus.
- `$`-prefixed DUT keys are reserved for events.

### Hardware
- Port hardware code from **`bobcat_engine`**, not BobDog. Read the matching adapter in `..\komcat\bobcat_engine\include\` and port its actual SDK calls.
- All drivers are PIMPL: **vendor headers never leak into ours**, and each has a same-API simulated stub for when the SDK is absent.
- **IO pin addressing is normalized: bit i == pin i for BOTH inputs and outputs.** Vendor bit layouts are translated inside the driver; a shifted value must never cross the interface.
- **Keithley 6482:** read with `:READ?` each iteration. Do **not** set `:INIT:CONT ON` or read with `:FETC?` (every `:READ?` then raises -213 and the stream stalls at 0 Hz), and do not add a fixed inter-read sleep. Sample rate is governed by NPLC, not the poll interval.

### Vision
- **Auto-focus must never depend on HALCON.** The focus measure is plain C++ (`vision/FocusMetric`); a licence problem degrades inspection, never alignment. The Alignment Procedure's manual crosshair pick is under the same rule — it is ImGui over a Pylon frame and nothing more.
- **Feature detection DOES need HALCON, and says so.** `vision/HalconDetector` (circle/fiducial, gray NCC with an anchor, square corner) is the only image analysis in the build. Without a licence the teach window is disabled with `HalconEngine`'s reason shown verbatim and an `auto` station fails loudly at run time; it must never silently become a manual pick unless the procedure asked for that fallback.
- **`RgbToGray8` (`vision/FocusMetric`) is the one RGB→luma conversion.** A second detector must not pick its own weights. It returns the CLAMPED ROI, and a detector must add that origin back — every `VisionDetection` coordinate is full-frame.
- **`RecipeEngine::Status.ctx` is a LIVE snapshot, not a result channel.** The engine clears `runCtx_` before the state becomes `Completed`, so a UI that waits for Completed and then reads `Status.ctx` gets an empty map. Anything a command hands back after a run goes through a `Workflow` queue drained in `App::Update` (`TakeSavedPositions` / `TakeAlignResults` / `TakeVisionReliability`).
- **A detection model is PRIVATE to one alignment station and auto-named `vdet_<procUid>_<station>`** — the name is the ownership record, exactly like the private station positions. There is no model picker, no rename, no New/Duplicate/Delete: parameters and a taught patch only mean anything at the pose and exposure they were tuned at. Deleting a procedure must drop them (`DropAlignStationDetectors`), duplicating must copy them (`CopyAlignStationDetectors`).
- **A detection model owns its `.pgm`** (`<modelUid>.pgm` under `ProductSession::VisionDir()`). Deleting a model drops the file (`DeleteVisionModel`); duplicating copies it. The filename in the JSON is **relative** — an absolute path makes a product folder non-relocatable.
- **A taught frame can only be created one way**: `align_solve` → `Workflow::QueueAlignResult` → `App::Update` → `ProductSession::ApplyAlignResult`. The Alignment page runs the same script a recipe's `align_run` step does — do not add a second capture path in the UI.
- **A SAVED frame is a record, never a licence to move.** `align_apply`, `move_local` and the Alignment page's *Move here* consume the **live** frame (`Workflow::SetLiveFrame`/`LiveFrame`), which exists only for the session that taught it and is cleared at **program start, DUT open and product load**. A frame describes where one physical part sat; moving against a stale one looks exactly like a correct move and is wrong by however far the part shifted. Never make a run read `AlignProc::result`.
- **Across devices, only a DELTA transforms.** Placing an object held by a hexapod on a gantry-measured nominal is a **relative** move: `AxisTransform` has no offset and the two origins have never been related, so an absolute machine XY must never be commanded to a hexapod. Measure the object, subtract in the frame (the origin cancels, the rotation must not), then map the vector with `Workflow::GlobalDeltaToAxes` — **not** `JogGlobal`, which collapses a direction to its dominant axis and would drop half a diagonal.
- **A nominal is LOCAL-ONLY, and can only be taught through a live frame.** `AlignNominal` stores `lx`/`ly` (O1); the machine midpoint beside it is a record. `AlignMachineToLocal` is an identity on an invalid frame, so teaching through one silently stores a machine coordinate that the next frame re-projects to nowhere — `SolveAlignNominal` refuses instead. Duplicating a procedure keeps its nominals and clears its `result`: the frame is per-part, the nominals are per-design.
- **A procedure's private positions are named `algn_<procUid>_…` and `hidden`** — `_<station>` for the two stations, `_<nomUid>_v` for a nominal's camera view. The name is the ownership record: deleting a procedure drops every one of them (`DropAlignStationPositions`, a prefix sweep), deleting a nominal drops its view (`DropAlignNominalView`), duplicating copies both — or hidden orphans accumulate where nobody looks. `CapturePrivatePose` is the only writer, so a re-teach keeps the uid. `hidden` is withheld from the recipe *picker*, never blocked at validation — an existing reference must still resolve.
- **A nominal's camera view is where the camera stands to WATCH a placement, never the target.** `place_view` consumes it; an untaught view is a no-op, not a failure, because procedures older than the field drive there with a `move_to_position` step above the run.
- **`alignment_procedure.lua` and `placement_procedure.lua` are STANDARD** (`IsSystemScript`, `command/ExecutionContext.h`): every surface runs the shipped `config/scripts/` copy, and a product copy of either name is ignored with a warning. They change with the commands they call — a fork keeps silently skipping the newest step. Do not add a launcher that resolves them off `session_.Scripts()`.
- **`vision/Halcon*.cpp` are the only TUs that include a HALCON header** (`HalconEngine.cpp`, `HalconDetector.cpp`), and nothing else may call HALCON without asking `HalconEngine::Instance().Available()` first. Probe the DLLs with `LoadLibrary` **before** any HALCON call — the SDK is `/DELAYLOAD`ed, and a missing delay-loaded import raises an SEH exception that `catch` does not handle.
- **Frame freshness uses `PylonCamera::FrameSeq()`, never `isNew`** — `isNew` is one-shot, so the first of two consumers eats it.
- Move waits live in `command/MotionWaits.h`. Do **not** write a second arrival/settle helper in a new command file; the two-signal settle rule is what makes a scan sample trustworthy.

### UI
- **Page state stays an `App` member.** New file-scope *mutable* state in a page TU is a bug — UiSettings persistence and cross-page sharing both read App members.
- A helper lives in the file that uses it; it moves to `ui/Widgets.{h,cpp}` only when a second TU needs it.
- **`Widgets.cpp` is the only UI TU that includes `windows.h`**; **`WndPrompts.cpp` the only one that includes `imgui_internal.h`**. Keep both that way.
- **All page switches route through `App::SwitchPage`** (it owns the takeover/`prevPage_` bookkeeping and the dirty-recipe guard). Don't assign `currentPage_` directly.
- When moving UI code between files, keep window/child/label strings and call order **byte-identical** (ImGui IDs derive from the runtime stack, not source location), and never mix a move with an improvement in one commit.
- Dangerous actions stay gated behind their armed checkbox (IO outputs, pneumatics, jog, click-to-move).
- Put global style/color tweaks in `ui/Theme.{h,cpp}`, not in individual panels.
- **Do not reintroduce raylib/rlImGui.** It is single-window by design, which makes a real second OS window impossible and blocks ImGui multi-viewport. Owning the GLFW host removes that ceiling.

### Project
- Windows-only, MSVC, C++20. Aim for `/W4`-clean.
- **Vendor-SDK gating belongs on the `firedog_hw` library, PUBLIC** — never on an executable, or the second exe silently loses a `HAS_*` def. New executables call `firedog_stage_runtime(<target>)` for the config/DLL staging.
- **`FireDogRig` is read-only.** It connects and reports; it must never gain a command that moves an axis, writes an IO output, or switches an output. Actuation stays in the client behind its armed checkboxes.
- **There is no automated test suite.** Verify by building clean, then running (simulated hardware today). Never claim "tests pass" or invent a test command.
