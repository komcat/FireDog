# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What FireDog Is

A **lightweight C++20 controller for a universal active-alignment machine**. Hardware: PI hexapod, ACS motion controller, Keithley current measurement, CLD101x, SMU, PSU. Early-stage — the scaffold runs, but the hardware layer is still simulated.

Sibling projects under `..\komcat\` are the reference implementations for the same domain: **`bobcat_engine`** (mature Qt6 version, richest domain map) and **`BobDog`** (a raylib+rlImGui version — useful for how the PI/ACS/Pylon SDKs get wired into CMake). FireDog deliberately does **not** use raylib (see the UI section for why).

## Build System

CMake + Ninja + MSVC (`cl.exe`), **Windows only**. Dependencies are fetched at configure time via `FetchContent` (GLFW 3.4, Dear ImGui *docking* branch, ImPlot, nlohmann/json) — first configure needs network access and takes a while. Two optional vendor SDKs, each gated by a compile def and with a stub fallback so the build always stays green:
- **Basler Pylon** (`HAS_PYLON`) — camera. Looks for `external/pylon 8`, then a system install (`PYLON_DEV_DIR` env).
- **NI-VISA** (`HAS_VISA`) — GPIB instruments (Keithley). Looks for `external/visa`, then `C:/Program Files/IVI Foundation/VISA/Win64`.

```bash
cmake --preset x64-debug          # configure (presets: x64/x86 × debug/release)
out/build/x64-debug/FireDog.exe   # run
```

**The user compiles and tests themselves** (global preference). Do **not** run `cmake --build ...` or launch the app — configure (`cmake --preset`) is fine; leave building and running to the user.

## Architecture — Three Layers

`readme.txt` defines the layering; the `src/` tree follows it. **Keep the dependency direction one-way: UI → workflow → hardware.** The UI never touches a device directly.

| Layer | Path | Role |
|-------|------|------|
| 1 — Hardware connection | `src/hardware/` | `IDevice` interface; per-instrument adapters; `HardwareManager` owns them all |
| 2 — Workflow | `src/workflow/` | `Workflow` orchestrates hardware collaboration (alignment procedures) |
| 3 — UI | `src/App.*`, `src/ui/` | GLFW+OpenGL3 host + Dear ImGui panels + ImPlot charts |

Entry point is `FireDog.cpp` → `firedog::App::Run()`, whose main loop is `while (!host_.ShouldClose())` over the GLFW `GlHost` (no raylib — see "Why not raylib"). Everything is in namespace `firedog`.

### Layer 1 — Hardware

**Truth source:** all hardware-connection code is ported from **`bobcat_engine`** (the battle-ready production controller), NOT BobDog. When adding a module, read the matching adapter in `..\komcat\bobcat_engine\include\` and port its actual SDK calls, dropping the Qt/abstraction layers.

- **Consolidated config** — `config/hardware.json` holds all device settings (cameras first; `motion`/`instruments` sections come as those modules land). Loaded by `HardwareConfig::Load` (`hardware/HardwareConfig.{h,cpp}`, tolerant/nlohmann). Copied next to the exe at build time; the app reads `config/hardware.json` relative to cwd.
- **Camera** — `hardware/camera/PylonCamera.{h,cpp}`. A **direct Pylon SDK** class (no `ICameraHardware`/adapter/factory stack — deliberately). PIMPL keeps all Pylon/Windows headers out of the header so UI code stays clean. Ported SDK details from bobcat_engine: GigE heartbeat + max packet size, `GrabStrategy_LatestImageOnly` + `GrabLoop_ProvidedByUser`, a grab thread, `CImageFormatConverter` → RGB8, mutex-guarded latest frame. Without `HAS_PYLON` it compiles an animated test-pattern stub with the same API. The camera object is **owned by `HardwareManager`** (`hardware_.Camera()`) and connected async by `Workflow::Start` from config; the UI only pulls frames — the live RGB frame streams into the **Aux window** via `GlHost::EnsureTextureRGB`.
- **Keithley 6482** — `hardware/instruments/Keithley6482.{h,cpp}`. A **direct VISA** driver (no `ISMUDevice`/adapter/Qt — deliberately), PIMPL keeping `<visa.h>` out of the header. Ported SCPI from bobcat_engine's `Keithley6482Adapter`: `*IDN?`, `*RST`/`*CLS`, `:SYST:ZCH OFF`, `:FORM:ELEM CURR1,CURR2` (dual read), NPLC + range, `:SOUR:VOLT`/`:OUTP ON` bias, `:READ?` parse. A `std::thread` poll loop (not QTimer) reads both channels with `:READ?` each iteration (see the sample-rate note below for why), **publishing to the DataBus**. Without `HAS_VISA` it compiles a simulated dual-channel stub. The driver is **owned by `HardwareManager`** (`hardware_.Keithley()`, built lazily via `SetKeithley`); `Workflow::Start` connects it async, sets **ch1 bias = 1 V**, then polls into topics `keithley/ch1` and `keithley/ch2` (names in `core/Topics.h`, shared with the UI consumer).

**Sample rate** is governed by `measurement_nplc` (integration time), not the poll interval: NPLC 0.1 ≈ 80 Hz, NPLC 1 ≈ 30 Hz, NPLC 5 ≈ 12 Hz (higher NPLC = slower but less noisy). `poll_interval_ms: 0` means "as fast as the instrument allows"; a positive value is an optional rate cap. The poll loop reads with **`:READ?`** each iteration (INIT+FETCh in one transaction) — this is the proven 6482 method (see `komcat/komcat_CMakeProject2/serverPython/Keithley6482/hardware/smart_multimeter.py`, which uses `:READ?` on every path). Do **not** configure `:INIT:CONT ON` or read with `:FETC?`: on the 6482 continuous-init makes every `:READ?` raise -213 "Init ignored" and the stream stalls at 0 Hz. Don't add a fixed inter-read sleep either.
- `IDevice` / `SimulatedDevice` / `HardwareManager` — still simulate the *other* instruments (PI, ACS, CLD101x, SMU, PSU) as placeholders until each is ported from bobcat_engine. `HardwareManager::ConnectAllAsync()` connects every simulated device on its own `std::async` thread; UI reads a thread-safe `Snapshot()`.

**Layer binding (single ownership, one-way drive).** Keep it: **layer 1 owns, layer 2 drives, layer 3 renders.** `HardwareManager` is the *sole owner* of every device — the `IDevice` fleet **and** the two direct-SDK drivers (`Camera()`, `Keithley()`), each with a `ConnState` atomic and name that `Snapshot()` folds into one status list. `Workflow::Start(hw)` owns the *sequence* (config→device, async connect, 1 V bias, `StartPolling` topic wiring) and holds the connect futures. `App` (UI) constructs no device, runs no `std::async`, and holds no per-device state — it reads `Snapshot()`, `hardware_.Camera()`, and the DataBus. **Do not** move device ownership or connect logic back into `App`, and don't reintroduce a parallel int-status encoding — use `ConnState`.

### Pub-sub — `DataBus` (`src/core/DataBus.{h,cpp}`)
Process-wide, thread-safe publish/subscribe for measurement streams — the "data available throughout the application" requirement. `DataBus::Instance().Stream("<topic>")` returns a `DataStream` (stable reference). Publishers call `Publish(t, value)` (e.g. the Keithley poll thread); consumers either register a push `Subscribe(callback)` or pull `Snapshot(t, v)` (what the immediate-mode Aux chart does each frame). The Aux window's ImPlot chart subscribes to `keithley/ch1`/`keithley/ch2` with a Ch1/Ch2/Both selector. Add new device streams by publishing to a new topic — no wiring changes needed on the consumer side.

### Layer 3 — UI notes
- **`Theme`** (`src/ui/Theme.{h,cpp}`) — centralized ImGui styling. `Theme::Apply()` is called once during `App` setup (`App.cpp`); put all global style/color tweaks here rather than scattering them across panels.
- **`GlHost`** (`src/ui/GlHost.h`) is our own thin host over GLFW + OpenGL3 + Dear ImGui. It owns the main OS window, the GL context, and the ImGui/ImPlot lifecycle. `Init` enables **both** `DockingEnable` and `ViewportsEnable` (multi-viewport = real extra OS windows). `EndFrame` renders the main window, then `UpdatePlatformWindows()/RenderPlatformWindowsDefault()` for detached viewport windows, restoring the GL context after. It exposes `MakeTextureRGBA()` (GL 3.0, via `opengl32`) so we don't depend on raylib for textures.
- **Fonts** — `GlHost::SetFont(ttfPath, sizePx)` / `SetFontSize(sizePx)` change the UI typeface/size (ImGui **1.92 dynamic fonts** — the backend owns the atlas texture via `ImGuiBackendFlags_RendererHasTextures`, so there is **no** `Create/DestroyFontsTexture`). Changes are **deferred to the next `BeginFrame`** (safe to call mid-draw): size is just `style.FontSizeBase` (no atlas work — slider-drag is cheap), and the atlas is only `Clear()`-and-reloaded when the *typeface path* actually changes. A missing/bad path falls back to the built-in scalable font (`AddFontDefault`); the loader probes the file first to avoid ImGui's debug `IM_ASSERT` on a bad path. `App::Run` sets **Segoe UI 26 px** at startup; the Main window's **Display** header offers a font picker + size slider (`kFonts` table in `App.cpp`, Windows system fonts).
- **Main window** = hardware status + vision image/overlay + drag-drop layout canvas. **Aux window** = realtime ImPlot chart, positioned *outside* the main viewport on first use so multi-viewport spawns it as its **own OS window** at startup (drag to a second monitor).
- Both windows currently show a **temporary `SetWindowFontScale` identifier banner** ("MAIN WINDOW" / "AUX WINDOW") — marked `TEMP` in `App.cpp`, to be removed.
- **Image + overlay**: upload frame via `GlHost::MakeTextureRGBA`, show with `ImGui::Image`, then draw vision overlays with `ImGui::GetWindowDrawList()` in the same screen coords (`DrawImageOverlay`). Real vision output replaces the checkerboard texture. Keep textures in the **Main** window (main GL context); the Aux chart is pure vector geometry so it renders in any viewport.
- **Drag-and-drop objects**: `DrawObjectCanvas` — manual hit-test against `objects_` + `MouseDelta`. Immediate-mode pattern for movable canvas items.
- **Realtime chart**: rolling `chartT_/chartV_` buffers fed each frame, plotted with ImPlot.

**Why not raylib:** raylib is single-window by design (`InitWindow` once, one global GL context), so a real second OS window is impossible and ImGui multi-viewport can't be enabled through rlImGui. Owning a GLFW host removes that ceiling. Do not reintroduce raylib/rlImGui.

## Conventions (carry over from siblings)
- Windows-only, MSVC, C++20. Aim for `/W4`-clean.
- Per-device settings will be JSON with a `xxx.local.json` → `xxx.json` local-override fallback (local files gitignored). Not present yet.
- No automated test suite — verify by building clean, then running (simulated hardware today). Never claim "tests pass" or invent a test command.
