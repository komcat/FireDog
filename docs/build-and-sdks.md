# Build & vendor SDKs

*Reference for CMake, presets and the optional hardware SDKs. Summary lives in CLAUDE.md.*

## Build System

CMake + Ninja + MSVC (`cl.exe`), **Windows only**. Dependencies are fetched at configure time via `FetchContent` (GLFW 3.4, Dear ImGui *docking* branch, ImPlot, nlohmann/json, Lua 5.4 via walterschell/Lua + sol2 v3.3.0) — first configure needs network access and takes a while. Optional vendor SDKs, each gated by a compile def and with a stub fallback so the build always stays green:
- **Basler Pylon** (`HAS_PYLON`) — camera. Looks for `external/pylon 8`, then a system install (`PYLON_DEV_DIR` env).
- **NI-VISA** (`HAS_VISA`) — GPIB instruments (Keithley). Looks for `external/visa`, then `C:/Program Files/IVI Foundation/VISA/Win64`.
- **PI GCS2** (`HAS_PI`) — hexapods. Vendored in `external/PI` (header + import lib; DLL copied next to the exe).
- **ACS ACSC** (`HAS_ACS`) — gantry. Vendored in `external/ACS`; links `ws2_32`.
- **FASTECH EziMOTIONPlusE** (`HAS_EZIIO`) — Ezi-IO boards. Vendored in `external/eziio` (DLL copied next to the exe).

WAGO needs no SDK — it speaks Modbus TCP over raw winsock.

```bash
cmake --preset x64-debug             # configure (presets: x64/x86 × debug/release)
out/build/x64-debug/FireDog.exe      # run the client
out/build/x64-debug/FireDogRig.exe   # run the console bring-up rig
```

## Targets

Three, from the one root `CMakeLists.txt`:

- **`firedog_hw`** (STATIC) — the UI-free layers: `src/core`, `src/hardware`,
  `src/workflow`. **All vendor-SDK gating lives here**, as PUBLIC compile
  definitions / include dirs / link libraries, because `HardwareManager.h` pulls
  `PylonCamera.h` & co into the UI translation units and the `HAS_*` defs must
  reach every consumer. Add a new SDK by extending the detection block *and* the
  matching `if(..._FOUND)` block on this library — not on an executable.
- **`FireDog`** — the GUI client. Globs `src/**` and filters out the three
  directories above (they arrive via `firedog_hw`), adds GLFW/ImGui/ImPlot/Lua.
- **`FireDogRig`** — the console bring-up rig (`tools/rig/`, see
  `docs/rig-console.md`). Links `firedog_hw` and nothing else.

`firedog_stage_runtime(<target>)` performs the POST_BUILD staging both exes need
— `config/` next to the exe plus the Pylon/PI/EziIO runtime DLLs. Call it for any
new executable; the copies are `copy_if_different`, so the second call is a no-op
when both targets land in the same output directory.

**The user compiles and tests themselves** (global preference). Do **not** run `cmake --build ...` or launch the app — configure (`cmake --preset`) is fine; leave building and running to the user.
