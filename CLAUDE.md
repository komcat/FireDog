# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What FireDog Is

A **lightweight C++20 controller for a universal active-alignment machine**. Real drivers exist for: Basler camera (Pylon), Keithley 6482 (VISA), PI hexapods + ACS gantry (motion), FASTECH Ezi-IO + WAGO Modbus (digital IO), plus synthetic pneumatic slides, an IO watch, and a safety watch built on the IO fleet. Still simulated placeholders: CLD101x, SMU, PSU.

Sibling projects under `..\komcat\` are the reference implementations for the same domain: **`bobcat_engine`** (mature Qt6 version, richest domain map) and **`BobDog`** (a raylib+rlImGui version — useful for how the PI/ACS/Pylon SDKs get wired into CMake). FireDog deliberately does **not** use raylib (see the UI section for why).

## Build System

CMake + Ninja + MSVC (`cl.exe`), **Windows only**. Dependencies are fetched at configure time via `FetchContent` (GLFW 3.4, Dear ImGui *docking* branch, ImPlot, nlohmann/json) — first configure needs network access and takes a while. Optional vendor SDKs, each gated by a compile def and with a stub fallback so the build always stays green:
- **Basler Pylon** (`HAS_PYLON`) — camera. Looks for `external/pylon 8`, then a system install (`PYLON_DEV_DIR` env).
- **NI-VISA** (`HAS_VISA`) — GPIB instruments (Keithley). Looks for `external/visa`, then `C:/Program Files/IVI Foundation/VISA/Win64`.
- **PI GCS2** (`HAS_PI`) — hexapods. Vendored in `external/PI` (header + import lib; DLL copied next to the exe).
- **ACS ACSC** (`HAS_ACS`) — gantry. Vendored in `external/ACS`; links `ws2_32`.
- **FASTECH EziMOTIONPlusE** (`HAS_EZIIO`) — Ezi-IO boards. Vendored in `external/eziio` (DLL copied next to the exe).

WAGO needs no SDK — it speaks Modbus TCP over raw winsock.

```bash
cmake --preset x64-debug          # configure (presets: x64/x86 × debug/release)
out/build/x64-debug/FireDog.exe   # run
```

**The user compiles and tests themselves** (global preference). Do **not** run `cmake --build ...` or launch the app — configure (`cmake --preset`) is fine; leave building and running to the user.

## Architecture — Three Layers

`readme.txt` defines the layering; the `src/` tree follows it. **Keep the dependency direction one-way: UI → workflow → hardware.** The UI never touches a device directly.

| Layer | Path | Role |
|-------|------|------|
| 1 — Hardware connection | `src/hardware/` | Device drivers + interfaces; `HardwareManager` owns them all |
| 2 — Workflow | `src/workflow/` | `Workflow` builds devices from config, drives connect/poll, owns jog kinematics, motion/IO commands, safety |
| 3 — UI | `src/App.*`, `src/ui/` | GLFW+OpenGL3 host + Dear ImGui panels + ImPlot charts |

Entry point is `FireDog.cpp` → `firedog::App::Run()`, whose main loop is `while (!host_.ShouldClose())` over the GLFW `GlHost` (no raylib — see "Why not raylib"). Everything is in namespace `firedog`.

**Layer binding (single ownership, one-way drive).** Keep it: **layer 1 owns, layer 2 drives, layer 3 renders.** `HardwareManager` is the *sole owner* of every device — the `IDevice` fleet, the two direct-SDK drivers (`Camera()`, `Keithley()`), the motion fleet, the IO fleet, the pneumatic slides, and the IoWatch. Each owned device/slot carries a `ConnState` atomic (the async-connect hand-off between Workflow's connect threads and the UI read) that `Snapshot()` folds into one status list. `Workflow::Start(hw)` owns the *sequence*: per-subsystem `StartCamera/StartKeithley/StartMotion/StartIO/StartPneumatics/StartIoWatch/StartSafetyWatch` (config→device, async connect, bias, `StartPolling` topic wiring), holding the connect futures. `App` (UI) constructs no device, runs no `std::async`, and holds no per-device state — it reads `Snapshot()`, the accessors, and the DataBus, and issues every command through `Workflow` (`JogGlobal`, `MoveToPosition`, `SetIoOutput`, `ExtendSlide`, …). **Do not** move device ownership or connect logic back into `App`, and don't reintroduce a parallel int-status encoding — use `ConnState`.

One deliberate exception: **`SafetyWatch` is owned by `Workflow`, not `HardwareManager`** — its callback stops motion via the Workflow object, so it must die first (declared last; `~Workflow` also stops it explicitly).

### Layer 1 — Hardware

**Truth source:** all hardware-connection code is ported from **`bobcat_engine`** (the battle-ready production controller), NOT BobDog. When adding a module, read the matching adapter in `..\komcat\bobcat_engine\include\` and port its actual SDK calls, dropping the Qt/abstraction layers.

Two driver shapes coexist, both PIMPL (vendor headers never leak into ours), both with a same-API simulated stub when the SDK compile def is absent:
- **Direct-SDK singletons** (camera, Keithley) — one concrete class, no interface.
- **Fleets behind an interface** (`IMotionController` for PI/ACS, `IIoDevice` for Ezi-IO/WAGO) — multiple heterogeneous devices; Workflow builds each from config and hands it to `HardwareManager` (`AddMotion`/`AddIo`), which stores it in a slot with its own `ConnState` atomic (slots held by `unique_ptr` so the atomics stay put).

Modules:
- **Camera** — `hardware/camera/PylonCamera.{h,cpp}`. Direct Pylon SDK. Ported from bobcat_engine: GigE heartbeat + max packet size, `GrabStrategy_LatestImageOnly` + `GrabLoop_ProvidedByUser`, a grab thread, `CImageFormatConverter` → RGB8, mutex-guarded latest frame. Exposure control via SFNC `ExposureTime` (legacy `ExposureTimeAbs` fallback), `ExposureAuto` forced Off at connect; `GetInfo()` snapshots identity + measured fps. The UI pulls frames via `App::UpdateCameraTexture` (shared texture; fresh-frame gated, so Aux + Camera page can both draw it). The **Camera page** drives connect/disconnect/exposure through `Workflow` (`ConnectCamera`/`DisconnectCamera`/`SetCameraExposure`; the config is captured at `Start` for reconnects) and edits the pixel→mm calibration live, saved back into the camera's `hardware.json` entry via `HardwareConfig::SaveCameraSettings` (read-modify-write; `exposure_us` is also applied after every connect).
- **Keithley 6482** — `hardware/instruments/Keithley6482.{h,cpp}`. Direct VISA. SCPI from bobcat_engine's `Keithley6482Adapter`: `*IDN?`, `*RST`/`*CLS`, `:SYST:ZCH OFF`, `:FORM:ELEM CURR1,CURR2` (dual read), NPLC + range, `:SOUR:VOLT`/`:OUTP ON` bias, `:READ?` parse. A `std::thread` poll loop publishes both channels to the DataBus; `Workflow::Start` connects it async and sets **ch1 bias = 1 V**.
  - **Sample rate** is governed by `measurement_nplc` (integration time), not the poll interval: NPLC 0.1 ≈ 80 Hz, NPLC 1 ≈ 30 Hz, NPLC 5 ≈ 12 Hz. `poll_interval_ms: 0` means "as fast as the instrument allows". The poll loop reads with **`:READ?`** each iteration (INIT+FETCh in one transaction) — the proven 6482 method (see `komcat/komcat_CMakeProject2/serverPython/Keithley6482/hardware/smart_multimeter.py`). Do **not** configure `:INIT:CONT ON` or read with `:FETC?`: on the 6482 continuous-init makes every `:READ?` raise -213 "Init ignored" and the stream stalls at 0 Hz. Don't add a fixed inter-read sleep either.
- **Motion** — `hardware/motion/`: `IMotionController` + `PIHexapod` (GCS2) and `ACSController` (ACSC over TCP). Poll threads publish per-axis positions to `motion/<device>/<axis>` and use **adaptive polling**: `RequestFastPoll()` is a demand signal — consumers that need realtime positions (open motion page, active procedure) call it repeatedly; the driver polls fast for a short window and drops to a slow idle rate otherwise.
- **Digital IO** — `hardware/io/`: `IIoDevice` + `EziIoDevice` (FAS lib) and `WagoIoDevice` (Modbus TCP; `simulated: true` gives an in-memory sim with no sockets). **Pin addressing is normalized: bit i == pin i for BOTH inputs and outputs** — any vendor bit layout (EZI-IO packs outputs at bits 8+/16+) is translated inside the driver; a shifted value must never cross the interface. Latch semantics differ: EZI-IO latches input edges in hardware; WAGO synthesizes rising edges in software per poll cycle (misses pulses shorter than the interval). Pins have config names, resolvable both ways (`InputPinByName` etc.) — pneumatic slides, IO watch, and safety watch all reference pins **by name**, resolved once at startup by Workflow.
- **Pneumatic slides** — `hardware/pneumatic/PneumaticSlide.{h,cpp}`. Synthetic devices composed from IO pins (one output = extend, two position sensors). The core is **passive**: `Update()` advances the state machine from the IO drivers' cached `Status()` (never hardware), and Workflow ticks it every frame. No `ConnState` — a slide's "connection" is its underlying IO devices. `ExtendAndWait`/`RetractAndWait` block — never call from the UI thread.
- **IoWatch** — `hardware/io/IoWatch.{h,cpp}`. Background thread mapping input edges to output actions (toggle / set_on / set_off / momentary) with per-binding debounce. Editable at runtime via the IO Watch editor window; `Workflow::ApplyIoWatch` swaps the running watch.
- **SafetyWatch** — `hardware/io/SafetyWatch.{h,cpp}`. First-priority: polls safety inputs (E-Stop etc.), latches ONE alert until acknowledged, fires a callback from the watch thread. For **critical** signals Workflow's callback calls `StopAllMotion()` immediately, and new motion commands are blocked (`SafetyBlocked()`) until the UI's modal alert is acknowledged.
- `IDevice` / `SimulatedDevice` — placeholders for the not-yet-ported instruments (CLD101x, SMU, PSU).

### Config

- **`config/hardware.json`** — consolidated device config, sections: `cameras`, `instruments` (Keithley), `motion_devices` (PI/ACS), `io_devices` (eziio/wago, with per-pin `pins` name maps), `pneumatic_slides`, `io_watch`, `safety_watch`. Loaded by `HardwareConfig::Load` (tolerant/nlohmann; struct-per-section in `HardwareConfig.h`). Copied next to the exe at build time; the app reads it relative to cwd. `HardwareConfig::SaveIoWatch` writes ONLY the `io_watch` section back (read-modify-write) — but a rebuild recopies the repo's `config/` over the exe-side copy, so mirror durable edits into the repo file too.
- **`config/motion_transforms.json`** — per-device global→local `AxisTransform` (identity default): a 3×3 `Matrix` for translation and an optional 3×3 `Rotation` block (global U/V/W = θx/θy/θz → local rotational axes). Both are signed-permutation columns (column j = where global axis j goes). Used by Workflow's jog kinematics (`JogGlobal`/`JogGlobalRot`); editable live in the **Transforms Editor** page (View menu) — every edit hot-applies via `SetTransforms`, Save writes the file (same exe-side/rebuild caveat as `SaveIoWatch`).
- **Saved positions** — `core/SavedPositions.{h,cpp}`: one JSON for all devices keyed by device name, `config/positions.local.json` with `positions.json` fallback (the project's `*.local.json` convention — local files gitignored). Positions carry a stable `uid` (recipes/programs will reference it; renaming changes only the label) and can be `hidden` (system positions).
- **Products** — `core/Product.{h,cpp}`: a product is a folder under `products/` (next to the exe) with a `manifest.json` naming uid-stamped core files (`{uid}.positions.json` / recipes / programs) plus `data/` and `images/` subfolders. When a product is loaded (File menu; last product auto-reloads at startup via `UiSettings`), saved positions persist into the product's positions file instead of `config/positions.local.json`.
- **UI prefs** — `ui/UiSettings.{h,cpp}`: font, panel splitter sizes, view toggles, last product — a gitignored `*.local.json` next to the exe. All loaders are tolerant: missing/malformed files yield defaults, never throw.

### Pub-sub — `DataBus` (`src/core/DataBus.{h,cpp}`)
Process-wide, thread-safe publish/subscribe for measurement streams. `DataBus::Instance().Stream("<topic>")` returns a stable `DataStream` reference; publishers `Publish(t, value)`, consumers push-`Subscribe(callback)` or pull `Snapshot(t, v)` (what the immediate-mode Aux chart does each frame). Topic names live in `core/Topics.h` — never hard-code the strings: `keithley/ch1|ch2`, `motion/<device>/<axis>`, `io/<device>/in|out` (pin bitmask as double, published only on change). Add new device streams by publishing to a new topic — no wiring changes needed on the consumer side.

### Layer 2 — Workflow notes
- **Jog kinematics live here** (not the UI, not the driver): `JogGlobal` maps a global direction through the device's `AxisTransform` to a signed local relative move; rapid presses accumulate onto one evolving per-(device,axis) target (`JogTrack`) so the latest press overrides rather than queues, re-seeding from the live position after an idle gap.
- `MoveToPosition` issues one coordinated multi-axis absolute move (saved-positions Move button); `MoveGantryRelativeMM` backs the Aux window's click-to-move.
- All motion commands are inhibited while a critical safety alert is unacknowledged.

### Layer 3 — UI notes
- **`Theme`** (`src/ui/Theme.{h,cpp}`) — centralized ImGui styling as a table of selectable themes (FireDog Dark / Deep Blue / Graphite / Light). `Theme::Apply(index)` rebuilds the style from a fresh baseline (preserving `FontSizeBase`, which GlHost owns) and re-derives ImPlot's auto colors; the Display header's theme combo hot-switches it and the index persists in `UiSettings`. Every theme keeps `WindowRounding = 0` (multi-viewport OS windows) — put all global style/color tweaks here rather than scattering them across panels.
- **`GlHost`** (`src/ui/GlHost.h`) is our own thin host over GLFW + OpenGL3 + Dear ImGui. It owns the main OS window, the GL context, and the ImGui/ImPlot lifecycle. `Init` enables **both** `DockingEnable` and `ViewportsEnable` (multi-viewport = real extra OS windows). `EndFrame` renders the main window, then `UpdatePlatformWindows()/RenderPlatformWindowsDefault()` for detached viewport windows, restoring the GL context after. It exposes `MakeTextureRGBA()` (GL 3.0, via `opengl32`) so we don't depend on raylib for textures.
- **Fonts** — `GlHost::SetFont(ttfPath, sizePx)` / `SetFontSize(sizePx)` change the UI typeface/size (ImGui **1.92 dynamic fonts** — the backend owns the atlas texture via `ImGuiBackendFlags_RendererHasTextures`, so there is **no** `Create/DestroyFontsTexture`). Changes are **deferred to the next `BeginFrame`** (safe to call mid-draw): size is just `style.FontSizeBase`, and the atlas is only reloaded when the *typeface path* actually changes. A missing/bad path falls back to the built-in scalable font. Default is **Segoe UI 26 px**; the Display header offers a picker (`kFonts` table in `App.cpp`), persisted via `UiSettings`.
- **Main window** is paged (`Page` enum, Hardware menu): **Overview** (status + vision + log), **Camera** (connect/disconnect / exposure + pixel→mm calibration + preview / identity + stream info), **Motion** (PI + ACS: device list / detail + saved positions / live axes), **IO** (device list / output toggles / live inputs), **Pneumatic** (slide list / Extend-Retract-Stop / sensor states). Every page uses the same three-panel layout (left list, middle detail, right live info) with draggable splitters whose pixel sizes persist in `UiSettings` and are shared across pages. The **IO Watch editor** and **Transforms editor** (View menu) are full-page takeovers of the main window — they replace the panel layout until their top-right Close button returns to the previous page (`prevPage_`); each edits a working copy (watch: Apply rebuilds the live watch, Save also writes `config/hardware.json`; transforms: edits hot-apply, Save writes `config/motion_transforms.json`). Separate windows: **Jog** (View menu; armed via checkbox, W/D·A/S·R/F keys), the blocking **safety alert modal**, and the **Aux window** — camera image + Keithley ImPlot chart, positioned outside the main viewport on first use so multi-viewport spawns it as its own OS window (drag to a second monitor).
- **Dangerous actions are gated**: IO output toggles and pneumatic commands sit behind an armed checkbox (default ON as a kill switch); jog and Aux click-to-move (image click → `MoveGantryRelativeMM` via the `pixelToMm` calibration) default OFF and must be explicitly armed.
- **Image + overlay**: upload frame via `GlHost` textures, show with `ImGui::Image`, then draw vision overlays with `ImGui::GetWindowDrawList()` in the same screen coords. Keep textures in windows on the **main** GL context; ImPlot charts are pure vector geometry so they render in any viewport.
- **Log viewer** (`core/Log.{h,cpp}`): `Logger` ringbuffer, snapshotted by the UI with level filter + autoscroll.

**Why not raylib:** raylib is single-window by design (`InitWindow` once, one global GL context), so a real second OS window is impossible and ImGui multi-viewport can't be enabled through rlImGui. Owning a GLFW host removes that ceiling. Do not reintroduce raylib/rlImGui.

## Conventions (carry over from siblings)
- Windows-only, MSVC, C++20. Aim for `/W4`-clean.
- Runtime-writable JSON follows the `xxx.local.json` → `xxx.json` fallback convention (local files gitignored): saved positions, UI settings.
- No automated test suite — verify by building clean, then running (simulated hardware today). Never claim "tests pass" or invent a test command.
