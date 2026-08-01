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
cmake --preset x64-debug          # configure (presets: x64/x86 × debug/release)
out/build/x64-debug/FireDog.exe   # run
```

**The user compiles and tests themselves** (global preference). Do **not** run `cmake --build ...` or launch the app — configure (`cmake --preset`) is fine; leave building and running to the user.
