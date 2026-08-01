# Architecture — the three layers

*The layering contract and who owns what. The one-way dependency rule and the
`ConnState` convention are repeated in CLAUDE.md because breaking them is easy
and expensive; the detail lives here.*

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

## Layer 2 — Workflow notes
- **Jog kinematics live here** (not the UI, not the driver): `JogGlobal` maps a global direction through the device's `AxisTransform` to a signed local relative move; rapid presses accumulate onto one evolving per-(device,axis) target (`JogTrack`) so the latest press overrides rather than queues, re-seeding from the live position after an idle gap.
- `MoveToPosition` issues one coordinated multi-axis absolute move (saved-positions Move button); `MoveGantryRelativeMM` backs the Aux window's click-to-move.
- All motion commands are inhibited while a critical safety alert is unacknowledged.
