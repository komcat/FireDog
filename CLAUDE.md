# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**This file is the map and the rules.** The detail lives in `docs/`, one topic per
file — open the relevant one *before* changing that subsystem. What is kept here
is what prevents a mistake when nothing else has been read.

## What FireDog Is

A **lightweight C++20 controller for a universal active-alignment machine**. Real drivers exist for: Basler camera (Pylon), Keithley 6482 picoammeter and Keithley 2602B SMU (VISA, both behind `ISmu`), Thorlabs CLD101x laser/TEC (VISA), UNI-T UDP3305S 3-channel PSU (VISA), PI hexapods + ACS gantry (motion), FASTECH Ezi-IO + WAGO Modbus (digital IO), plus synthetic pneumatic slides, an IO watch, and a safety watch built on the IO fleet. Nothing real is still simulated.

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
| — Access | `src/core/Employees.*`, `src/ui/pages/PageLogin.cpp` | employee roster, levels, the login gate | `docs/user-access-design.md` |
| — Tools | `tools/rig/` | `FireDogRig`, the console bring-up rig (layers 1–2, no UI) | `docs/rig-console.md` |

### Topic docs (`docs/`)

**Reference — how it works now:**
`architecture.md` (layering + ownership) · `build-and-sdks.md` (**targets + SDK gating**) · `hardware-layer.md` (per-device drivers) · `vision-layer.md` (focus metric, auto-focus, HALCON) · `state-and-config.md` (config files, persistence, DataBus) · `command-layer.md` (registry, recipes, Lua) · `run-and-dut.md` (programs, Run page, traceability) · `ui-layer.md` · `ui-file-layout.md` (**where UI code goes**) · `rig-console.md` (the FireDogRig console)

**Design rationale — why it is that way:**
`recipe-core-design.md` · `recipe-editor-design.md` · `lua-tier-design.md` · `run-page-design.md` · `program-organizer-design.md` (**slots are node TREES: folders + loops; the deck is read-only**) · `deck-cycle-design.md` (start/end states + per-node pass/fail accents) · `dut-design.md` · `xcan-scan-design.md` (the align algorithm) · `climb-raster-design.md` (the gap-closing Z + XY raster) · `alignment-procedure-design.md` (the taught local frame O1) · `user-access-design.md` (**the employee roster, the three levels, the login gate**) · `vision-detection-design.md` (detection models, the teach wizard, auto stations) · `needle-dispense-design.md` (the camera→needle lever arm and the dispense procedure) · `device-registration-design.md` (**the camera→gripper registration: R measured not typed, two dates, an approach and not a placement**) · `pi-hexapod-motion-design.md` · `keithley-polling-design.md` (the SMU layer: 6482 SCPI + 2602B TSP behind `ISmu`) · `app-refactor-plan.md` (the 2026-07 App.cpp decomposition, done)

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
- **A slot is a TREE of nodes (recipe | folder), and a FOLDER is the unit of repetition.** Loops are expanded into the flat queue by `BuildRunQueue` at Start — `ProgramRunner` never learns what a folder is, so Stop, progress and `$run` keep exactly one implementation. Hence the ceilings (`kMaxQueueItems`, `kMaxDepth`): a mistyped loop count must be refused before the machine moves. There is no "repeat rows 1-3" range loop — a range is a second way to say what a folder says, and it breaks on the first reorder.
- **A node is addressed by its UID, never by an index.** An index into a tree means nothing once a folder above it opens, closes or moves; `deckNode_`, the run marks and the organizer's selection are all uids.
- **A deck cycle's node marks are RUNTIME state, cleared only by running the start node.** `deckMarks_` never reaches the program file or UiSettings: a green tick describes one physical part, exactly like a live frame, and a persisted one would read as a licence to skip a step. `AdvanceDeckCycle` steps off `runNodeUid_` — what was LAUNCHED — never off `deckNode_`, because the deck stays clickable during a run. The cycle never blocks a run: it is an aid, not an interlock.
- **Program editing lives ONLY in the Program Organizer window** (`ui/windows/WndProgramOrganizer.cpp`); the Run deck is read-only. Its drops are DEFERRED to `ApplyOrgDrop` — inserting inside `DrawOrgTree` reallocates the vector the recursion is walking. Its editing surfaces gate on `orch_.Busy()` and it raises no modal: it can be open on a second monitor while a worker is blocked on the always-on-top prompt window.
- **Every Run/Go surface gates on `orch_.Busy()`** — never on a subset of the engines. The program runner goes momentarily idle between queue items, so checking the engines alone lets a second run slip in.
- Recipes never connect hardware; validation at Start requires devices already `Connected`.
- Ctx does **not** flow between recipes — cross-recipe data goes through the DUT or the DataBus.
- `$`-prefixed DUT keys are reserved for events.

### Dispense needle
- **The camera→needle offset lives in `gantry-main` AXIS space and never goes through `AxisTransform`.** Camera and needle ride the same device, so the delta between them crosses no frame — and `gantry-main`'s transform inverts Z (`M33 = -1`), so an axis value issued as a global delta descends when it meant to retreat. `read_position` reads axis, `move_axis_to` commands axis, `NeedleCal` stores axis.
- **Which way Z counts is derived, never assumed.** `NeedleDownSign` is `sign(dz)` from the measurement. bobcat hardcoded "+Z is down"; that is correct on one machine and a needle crash on the next.
- **`dz` is an offset from a FOCUSED camera Z, not a height.** An absolute touch height only describes the surface it was taught on, which is why bobcat needed a `dispense_z_<uid>.json` per point. With `auto_focus`, one offset serves every point.
- **The calibration is machine-level and its persisted record IS the licence to move** — unlike a taught frame, which describes one part and is cleared at every boundary. It goes stale for exactly one reason: the needle was changed. A worker queues it (`Workflow::QueueNeedleCal`); only `App::Update` writes `config/needle_cal.local.json`.
- **`needle_calibration.lua` is STANDARD; `dispense_procedure.lua` is deliberately NOT.** Where the needle is, is a fact about the machine and every product must measure it identically. How much glue goes where is a fact about the product, so its own copy must keep winning.

### Device registration
- **`R` is MEASURED; the matrix in `motion_transforms.json` is a typed guess and only ever a SEED to aim with.** Those entries are permutations of ±1 — exact for a device bolted square, and wrong for one that is not in a way that predicts perfectly where it was taught and misses by a millimetre 30 mm away. Copying them into a registration is how "the calibration is drifting" gets written in the log instead of "the rotation was never measured".
- **The axis triad is never assumed right-handed.** Every matrix in the shipped transforms file has `det = −1`; controller sign conventions are not consistently handed. `handed` takes the sign from the Z jog, `R` may be improper, and `R^T` is still its inverse either way. Forcing a proper rotation inverts one axis of every answer.
- **`R` belongs to the MOUNT, `t` belongs to the mount AND the tool — hence two dates.** A changed gripper invalidates `t` alone and costs ONE pick (`device_reg_tip_save`); only a moved or crashed device costs the four-pick pass. `regAt` and `tipAt` are stamped separately so the page can say which half is old.
- **A fit is not verified by predicting at the pose it was fitted at** — that returns the measurement by construction. The check drives to `h0 + (dx,dy,dz)` and `verified` stays false when all three are zero, however small the residual comes out. `adopt_tip` zeroes the residual whatever caused it, so it is off by default and clears `verified`.
- **The quality gates live in `DeviceRegFit` (C++), not in the procedure**, and they are refusals: a column scale off 1, a skewed triad, a measured Z that disagrees with `c1 × c2`. A gate that only exists in a Lua file is a gate a fork can delete. `DeviceRegFit` fills every quality field *before* the first refusal returns, so a rejection names the number that was out.
- **A tip that cannot be predicted publishes NOTHING** on `tip/<device>/…` — no stale value, no guess. A readout frozen at the last good number is how a rotated hexapod goes on looking placed. The reason is logged on the edge, never per frame.
- **Both procedures are STANDARD** (`device_registration.lua`, `device_reg_check.lua`): where a gripper is, is a fact about the machine, and five product copies would be five answers with no way to tell which is right.

### Users & access
- **An employee number is an identity; a level is a property of that number.** The number, its level and its password are registered together by an admin in the User Manager — there is no shared per-level password and no self-service password change, because a credential that can be lent names a person who was never there.
- **The roster lives in `products/employees.json`, NEVER in `config/`.** A rebuild recopies `config/` over the exe-side copy, so a roster there is wiped by every build. For the same reason it takes no `xxx.local.json` pair: there is no shipped default to fall back to.
- **`MinRoleFor` defaults to Admin.** A page not named in the table is locked until somebody grants it — the only default that cannot fail open. The split is by what a page WRITES: Technician uses what the machine knows (motion, recipes, diagnostics), Admin teaches it (frames, detectors, needle, device registration, transforms, scripts).
- **`SwitchPage` is the enforcement; hidden menu items are courtesy.** Every switch already routes through it, so a surface that forgets to check still cannot land above its level. Gate a click *early* only when it mutates state before switching (the Run deck's right-click, and the four `Open*` teach helpers, which set up a model/marks/selection first).
- **A page gate does not cover a WINDOW or a shared block.** The Aux window is `kRoleAux` (Operator) because it monitors — but everything on it that MOVES (Arm jog, Z arrows, click-to-move, Autofocus) is `kRoleJog`, or it is a way around the Jog gate. Click-to-move is checked at the checkbox AND at the click handler because `enableClickMove_` is persisted in UiSettings. `DrawDutBlock` is shared by Overview and Run, so its Explorer shortcut is Technician: on an Operator's station it is the one control that leaves the application.
- **`LogOut` clears `jogArmed_`.** An armed jog inherited by whoever signs in next is the opposite of what the arm is for. `manual_adjust`'s jog stays ungated on purpose — a recipe step asked *this* operator to nudge the part and the run is blocked until they do.
- **The login gate is not a `Page`** — it draws instead of the whole main window, so no menu, F-key, strip button or floating window exists until somebody signs in. `Update` still runs, so hardware connects during login.
- **Sign out is refused while `orch_.Busy()`**, which is also what stops the gate appearing mid-run and blocking the always-on-top prompt window — the same reason this layer raises no modal.
- **`App::LogIn` is the ONE writer of `DutSession::SetOperator`.** The record carries the operator top-level AND as a `$operator` entry at open/handover AND on every `$run`: one record can span a shift change.
- Passwords are **plaintext, on purpose** (a roster repairable with Notepad beats resistance to someone who already owns the PC). The built-in `admin` / `iamyourfather` is checked before the roster so a corrupt file is never a lockout, and cannot be registered over.

### Hardware
- Port hardware code from **`bobcat_engine`**, not BobDog. Read the matching adapter in `..\komcat\bobcat_engine\include\` and port its actual SDK calls.
- All drivers are PIMPL: **vendor headers never leak into ours**, and each has a same-API simulated stub for when the SDK is absent.
- **IO pin addressing is normalized: bit i == pin i for BOTH inputs and outputs.** Vendor bit layouts are translated inside the driver; a shifted value must never cross the interface.
- **The current stream has TWO drivers behind one interface: `ISmu`.** `keithley_6482` (SCPI, GPIB/USB) and `keithley_2602b` (TSP, LAN) are picked by `type` in `hardware.json`, and `Workflow::LaunchKeithleyConnect` is the ONE place a concrete meter is constructed. Everything above layer 1 — the `keithley_*` recipe commands, the `keithley/ch1|ch2` topics, the page, the chart — stays model-agnostic; adding a third meter must not add a second code path up there. Do not rename the commands or topics to match a model: saved recipes reference them by name.
- **Keithley 6482:** read with `:READ?` each iteration. Do **not** set `:INIT:CONT ON` or read with `:FETC?` (every `:READ?` then raises -213 and the stream stalls at 0 Hz), and do not add a fixed inter-read sleep. Sample rate is governed by NPLC, not the poll interval.
- **Keithley 2602B is a SOURCE-measure unit, so its output must be ON to measure.** A 6482 measures with its V source off; an SMU with its output off reads ~0 A, which looks exactly like a dark photodiode. `SetVoltageBias` therefore switches **every enabled channel** on (ch1 at the bias, ch2 at 0 V) — never make it bias ch1 alone to "match the 6482". It is TSP, not SCPI: `localnode.prompts = 0` must go out **before the first query** or `TSP>` tokens interleave and every read is one reply out of step, and NPLC is clamped to 0.001..25 because the instrument silently ignores an out-of-window value.

### Vision
- **Auto-focus must never depend on HALCON.** The focus measure is plain C++ (`vision/FocusMetric`); a licence problem degrades inspection, never alignment. The Alignment Procedure's manual crosshair pick is under the same rule — it is ImGui over a Pylon frame and nothing more.
- **Feature detection DOES need HALCON, and says so.** `vision/HalconDetector` (circle/fiducial, gray NCC with an anchor, square corner) is the only image analysis in the build. Without a licence the teach window is disabled with `HalconEngine`'s reason shown verbatim and an `auto` station fails loudly at run time; it must never silently become a manual pick unless the procedure asked for that fallback.
- **The pre-process chain is applied in exactly TWO places**, both in `HalconDetector.cpp`: `Detect` (between the `RgbToGray8` crop and the method dispatch — every run-time caller funnels through `Workflow::Detect` to there) and `Teach` (the template is cut from the **filtered** plane, or NCC matches a raw patch against a filtered image and scores nothing). The teach page's preview and overlay go through the static `HalconDetector::Preprocess`; a second implementation of the three filters is how the picture starts disagreeing with the detector. The chain lives on the model, so a filter edit already invalidates the cached detector via the JSON stamp — but `VisionModelPatchKey` must carry `VisionFilterSummary` or a re-teach keeps showing the old thumbnail.
- **`RgbToGray8` (`vision/FocusMetric`) is the one RGB→luma conversion.** A second detector must not pick its own weights. It returns the CLAMPED ROI, and a detector must add that origin back — every `VisionDetection` coordinate is full-frame.
- **The Run page's image box is ONE parked frame (`Workflow::CapturedView`), a slot and not a queue** — a 5 MP frame is ~15 MB and the box shows one. `DrainCapturedView` must stay **after** the DUT-open edge in `App::Update`: that edge clears the box (a still shows one physical part), and a recipe doing `dut_open` then `dut_capture_image` lands both in one frame, so draining first lets the open wipe the picture the capture just delivered. `capture_view` deliberately needs **no** open DUT — reading the serial is how you learn which DUT to open.
- **`RecipeEngine::Status.ctx` is a LIVE snapshot, not a result channel.** The engine clears `runCtx_` before the state becomes `Completed`, so a UI that waits for Completed and then reads `Status.ctx` gets an empty map. Anything a command hands back after a run goes through a `Workflow` queue drained in `App::Update` (`TakeSavedPositions` / `TakeAlignResults` / `TakeVisionReliability`). **`Status.lastCtx` is the snapshot taken at that terminal boundary and is DISPLAY ONLY** (the Run page's Values panel): it exists because polling `.ctx` per frame misses a value written by the recipe's final step. Never make a command read it — two result channels will disagree about what a run produced. Its drain (`DrainCycleVars`) must stay **before** `orch_.programs.Tick()` in `App::Update`, or the runner starts the next queue item first and the edge is missed on every recipe but the last.
- **A detection model is PRIVATE to one alignment station and auto-named `vdet_<procUid>_<station>`** — the name is the ownership record, exactly like the private station positions. There is no model picker, no rename, no New/Duplicate/Delete: parameters and a taught patch only mean anything at the pose and exposure they were tuned at. Deleting a procedure must drop them (`DropAlignStationDetectors`), duplicating must copy them (`CopyAlignStationDetectors`).
- **A detection model owns its `.pgm`** (`<modelUid>.pgm` under `ProductSession::VisionDir()`). Deleting a model drops the file (`DeleteVisionModel`); duplicating copies it. The filename in the JSON is **relative** — an absolute path makes a product folder non-relocatable.
- **A taught frame can only be created one way**: `align_solve` → `Workflow::QueueAlignResult` → `App::Update` → `ProductSession::ApplyAlignResult`. The Alignment page runs the same script a recipe's `align_run` step does — do not add a second capture path in the UI.
- **A SAVED frame is a record, never a licence to move.** `align_apply`, `move_local` and the Alignment page's *Move here* consume the **live** frame (`Workflow::SetLiveFrame`/`LiveFrame`), which exists only for the session that taught it and is cleared at **program start, DUT open and product load**. A frame describes where one physical part sat; moving against a stale one looks exactly like a correct move and is wrong by however far the part shifted. Never make a run read `AlignProc::result`.
- **Across devices, an absolute position crosses ONLY through a verified `DeviceReg`, and only as an approach.** Without one, `AxisTransform` has no offset and the two origins have never been related, so **only a DELTA transforms**: measure the object, subtract in the frame (the origin cancels, the rotation must not), then map the vector with `Workflow::GlobalDeltaToAxes` — **not** `JogGlobal`, which collapses a direction to its dominant axis and would drop half a diagonal. With a registration, `move_to_point` may command a gantry-space point to a hexapod to get the tip **close**; the final placement is still the measured delta `placement_procedure.lua` computes, because a registration is a four-point fit and `place_tol_mm` is 20 µm. Never let a `move_to_point` stand in for the measure/correct loop.
- **A nominal is LOCAL-ONLY, and can only be taught through a live frame.** `AlignNominal` stores `lx`/`ly` (O1); the machine midpoint beside it is a record. `AlignMachineToLocal` is an identity on an invalid frame, so teaching through one silently stores a machine coordinate that the next frame re-projects to nowhere — `SolveAlignNominal` refuses instead. Duplicating a procedure keeps its nominals and clears its `result`: the frame is per-part, the nominals are per-design.
- **A procedure's private positions are named `algn_<procUid>_…` and `hidden`** — `_<station>` for the two stations, `_<nomUid>_v` for a nominal's camera view. The name is the ownership record: deleting a procedure drops every one of them (`DropAlignStationPositions`, a prefix sweep), deleting a nominal drops its view (`DropAlignNominalView`), duplicating copies both — or hidden orphans accumulate where nobody looks. `CapturePrivatePose` is the only writer, so a re-teach keeps the uid. `hidden` is withheld from the recipe *picker*, never blocked at validation — an existing reference must still resolve.
- **A nominal's camera view is where the camera stands to WATCH a placement, never the target.** `place_view` consumes it; an untaught view is a no-op, not a failure, because procedures older than the field drive there with a `move_to_position` step above the run.
- **`alignment_procedure.lua` and `placement_procedure.lua` are STANDARD** (`IsSystemScript`, `command/ExecutionContext.h`): every surface runs the shipped `config/scripts/` copy, and a product copy of either name is ignored with a warning. They change with the commands they call — a fork keeps silently skipping the newest step. Do not add a launcher that resolves them off `session_.Scripts()`.
- **`vision/Halcon*.cpp` are the only TUs that include a HALCON header** (`HalconEngine.cpp`, `HalconDetector.cpp`), and nothing else may call HALCON without asking `HalconEngine::Instance().Available()` first. Probe the DLLs with `LoadLibrary` **before** any HALCON call — the SDK is `/DELAYLOAD`ed, and a missing delay-loaded import raises an SEH exception that `catch` does not handle.
- **Frame freshness uses `PylonCamera::FrameSeq()`, never `isNew`** — `isNew` is one-shot, so the first of two consumers eats it.
- Move waits live in `command/MotionWaits.h`. Do **not** write a second arrival/settle helper in a new command file; the two-signal settle rule is what makes a scan sample trustworthy.
- **The two-speed move has ONE implementation: `MoveWithApproach`** (`command/MotionWaits.h`), and every point-to-point move command goes through it. Its ordering is load-bearing and a second copy will drift off it: the travel leg arrives on a LOOSER tolerance, `WaitSettle` runs BEFORE the velocity change (PI rejects `VLS` mid-profile with GCS 1005), and `velocity_mm_s` is re-applied afterwards so the device is not left crawling into the next step. A half-configured approach (a zone with no velocity, or the reverse) **fails** — it is a full-speed arrival wearing the label of a careful one, and `ValidateParams` is per-parameter so nothing catches it at author time.

### UI
- **Page state stays an `App` member.** New file-scope *mutable* state in a page TU is a bug — UiSettings persistence and cross-page sharing both read App members.
- A helper lives in the file that uses it; it moves to `ui/Widgets.{h,cpp}` only when a second TU needs it.
- **`Widgets.cpp` is the only UI TU that includes `windows.h`**; **`WndPrompts.cpp` the only one that includes `imgui_internal.h`**. Keep both that way.
- **All page switches route through `App::SwitchPage`** (it owns the takeover/`prevPage_` bookkeeping and the dirty-recipe guard). Don't assign `currentPage_` directly.
- **A run outranks navigation, and no page may drive a run it did not start.** Changing pages mid-run must not raise a MODAL: a modal blocks input to every other window, including the always-on-top prompt / `manual_adjust` / align-pick windows an engine worker is blocked on — the operator then sees a prompt they cannot click while the machine waits. Hence `SwitchPage` skips the dirty-recipe guard while `orch_.Busy()` (the edits stay in memory and stay dirty). For the same reason the Recipe Editor gates its editing surfaces on `orch_.Busy()` — **not** `recipe.Running()`, which blinks false between a program's queue items — disables Step/Next while `programs.Active()`, and makes its Stop stop the *program*, never one recipe out of the queue.
- When moving UI code between files, keep window/child/label strings and call order **byte-identical** (ImGui IDs derive from the runtime stack, not source location), and never mix a move with an improvement in one commit.
- Dangerous actions stay gated behind their armed checkbox (IO outputs, pneumatics, jog, click-to-move).
- Put global style/color tweaks in `ui/Theme.{h,cpp}`, not in individual panels.
- **Do not reintroduce raylib/rlImGui.** It is single-window by design, which makes a real second OS window impossible and blocks ImGui multi-viewport. Owning the GLFW host removes that ceiling.

### Project
- Windows-only, MSVC, C++20. Aim for `/W4`-clean.
- **Vendor-SDK gating belongs on the `firedog_hw` library, PUBLIC** — never on an executable, or the second exe silently loses a `HAS_*` def. New executables call `firedog_stage_runtime(<target>)` for the config/DLL staging.
- **`FireDogRig` is read-only.** It connects and reports; it must never gain a command that moves an axis, writes an IO output, or switches an output. Actuation stays in the client behind its armed checkboxes.
- **There is no automated test suite.** Verify by building clean, then running (simulated hardware today). Never claim "tests pass" or invent a test command.
