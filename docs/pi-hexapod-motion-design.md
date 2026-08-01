# PI hexapod (GCS2): connection, motion commands, and arrival/on-target waiting

Covers `src/hardware/motion/PIHexapod.{h,cpp}` (layer 1, PI GCS2 over TCP/IP),
`IMotionController` (the fleet interface), `Workflow` (layer 2 gating), and the blocking
wait helpers in `src/command/CommandsMotion.cpp` that every recipe/Lua move goes through.

Two hexapods are configured (`hex-left` @ 192.168.0.10:50000, `hex-right` @
192.168.0.30:50000), axes `X Y Z U V W`. Everything below is per-instance.

---

## 1. Module structure

```
config/hardware.json "motion_devices"  ──► MotionConfig {name, controller_type, axes, ip, port}
        ▼
Workflow::StartMotion       one PIHexapod per PI entry -> HardwareManager::AddMotion (owner)
        │                   ConnState Connecting, then std::async Connect() per device
        ▼
PIHexapod (PIMPL over PI_GCS2_DLL.h)
        │  mtx     : positions map + ALL SDK I/O
        │  pollMtx : fastUntilNs + poll condition_variable
        ▼  poll thread: PI_qPOS -> positions[] -> DataBus motion/<device>/<axis>
Commands (CommandsMotion.cpp)  ── Workflow::MoveToPosition ──► PI_MOV
        │  WaitArrive (positions, tolerance)
        └► WaitSettle (PI_IsMoving == false AND PI_qONT == true)
```

`HAS_PI` absent → a simulated Impl with the same API (targets snap to position,
`IsSettled`/`IsOnTarget` always true).

---

## 2. Connection

### `PIHexapod::Connect()`

| # | GCS2 call | Notes |
|---|---|---|
| 1 | `PI_ConnectTCPIP(ip, port)` → `id` | `id < 0` = failure; reason from `PI_GetInitError()`, stored in `lastError` |
| 2 | `PI_INI(id, NULL)` | initialise all axes |
| 3 | — | `connected = true`, one `DataBus` stream resolved per axis (`topics::MotionAxis(name, axis)`) |
| 4 | — | poll thread started |

The connect call is **blocking**; `Workflow::StartMotion` launches every device
concurrently on its own `std::async` thread and folds the result into
`ConnState::Connected | Failed`, which the UI status list renders. No referencing after
`AddMotion` — the manager owns the device, the index is stable.

### `Disconnect()`

Order matters and is enforced: flip `polling` under `pollMtx` → `pollCv.notify_one()`
(wakes the loop out of `wait_for` immediately, no 1 s tail) → `join` → then under `mtx`
`PI_STP(id)` (stop any motion) → `PI_CloseConnection(id)` → `id = -1`. `~Impl` calls it.

---

## 3. Position polling (adaptive)

The poll loop holds `mtx` only for `PI_qPOS(id, "X Y Z U V W", buf)`; publishing to the
DataBus happens **outside** the lock so a subscriber callback cannot deadlock against
`Positions()`.

| State | Period | Rate |
|---|---|---|
| Fast (a consumer is asking) | 50 ms | 20 Hz |
| Idle | 1000 ms | 1 Hz |

`RequestFastPoll()` is a **demand signal, not a mode**: it sets
`fastUntilNs = now + 500 ms`; the loop picks the fast period while `now < fastUntilNs`. A
consumer that needs realtime feedback (open Motion page, an active scan, `WaitArrive`)
calls it **repeatedly**. If the loop was idle, the call also notifies the CV so the switch
to fast is immediate instead of waiting out the 1 s idle sleep.

Sim driver: 50 Hz fast / 2 Hz idle, same window.

`Positions()` returns a locked copy of the map — safe from any thread, and it is the
source of truth for arrival checks (never a fresh SDK query from the caller's thread).

---

## 4. Motion command reference

| API | GCS2 | Semantics |
|---|---|---|
| `MoveAbsolute(axis, pos)` | `PI_MOV(id, "Z", &v)` | single axis, non-blocking (returns as soon as the controller accepts) |
| `MoveAbsolute(vector<axis,pos>)` | `PI_MOV(id, "X Y Z", vals)` | **one** command → axes move **together** (coordinated), not sequenced |
| `SetVelocity(mm/s)` | `PI_VLS(id, v)` | system (vectorial) velocity, persists on the device |
| `IsSettled()` | `PI_IsMoving(id, axes, BOOL[])` | true when **no** configured axis is executing a profile |
| `IsOnTarget()` | `PI_qONT(id, axes, BOOL[])` | true when **every** axis is inside the controller's settling window for the settling time (SPA-tunable) |
| `StopAll()` | `PI_STP(id)` | |
| error detail | `PI_qERR(id, &err)` | queried after any failed call, formatted into `lastError` |

All of these take `mtx`, so they serialise against the poll thread's `PI_qPOS`.

### `PI_VLS` and CONTROLLER_BUSY (1005)

The C-887 **rejects** a velocity change while the platform is still executing a motion
profile. `SetVelocity` therefore retries on error 1005: 100 ms steps up to 2000 ms, and it
**drops `mtx` between attempts** so the poll thread keeps running. Any other error, or the
2 s cap, fails the call. This is why scans set velocity once up front rather than per move.

### Layer-2 gate

Commands never call the driver directly — they call `Workflow::MoveToPosition(devIdx,
targets)`, which refuses while a critical safety alert is unacknowledged
(`SafetyBlocked()`), then issues the coordinated `MoveAbsolute`.
`Workflow::SetJogVelocity` clamps to the PI/ACS velocity limits before `PI_VLS`.

---

## 5. Waiting for completion — the two-stage rule

`PI_MOV` returns immediately. Every blocking command (`move_to_position`,
`move_relative`, `move_to_position_offset`, `move_path` legs, `move_to_position_approach`)
uses the same two-stage wait, in this order:

### Stage 1 — `WaitArrive` (position tolerance)

```
ctx.WaitUntil([&]{
      m->RequestFastPoll();                 // keep the 20 Hz window open
      pos = m->Positions();                 // cached, from the poll thread
      every targeted axis: |pos[axis] - target| <= tol_mm
  }, timeout_ms, /*pollMs=*/50);
```

| Parameter | Default | Range |
|---|---|---|
| `tol_mm` | 0.01 mm (`kDefaultTolMM`, same rule as `Workflow::TickPath`) | 0.0001 – 1.0 |
| `timeout_ms` | 30000 (`move_relative`) / 60000 (`move_to_position`) / 120000 per leg (`move_path`) | 100 – 600000 |
| poll granularity | 50 ms | — |

Scans tighten this: the XCAN script passes `tol_mm = 0.001` and `timeout_ms = 10000`.

### Stage 2 — `WaitSettle` (controller truth)

```
ctx.WaitUntil([&]{ return m->IsSettled() && m->IsOnTarget(); }, 5000, 50);
```

Both signals are required, because they mean different things:

* **`IsSettled` (`PI_IsMoving` false)** — the trajectory generator finished. A
  position-tolerance check can *lead* this (the profile's slow tail is inside tolerance),
  which is exactly what triggers `PI_VLS` error 1005 on the next step.
* **`IsOnTarget` (`PI_qONT`)** — the controller's own settling logic: position inside the
  settling window for the settling time. Profile-done alone can lead mechanical settle
  while the platform is still ringing — that ringing poisoned scan measurements before
  `qONT` was added to the predicate.

**Contract for callers:** when a move command returns success, the mechanics are settled.
Downstream code (the scan scripts) does not add its own mechanical settle delay — see
`docs/xcan-scan-design.md` §4.

### Failure paths

| Outcome | Action |
|---|---|
| `WaitResult::Stopped` (stop requested) | `Workflow::StopMotion(idx)`, step fails with `"stopped"` |
| `WaitResult::Timeout` (arrival) | `StopMotion(idx)` → **defined state**, step fails `"timeout …"` |
| `WaitResult::Timeout` (settle, 5 s) | `StopMotion(idx)`, `"timeout waiting for motion settle"` |
| Driver can't tell (`PI_IsMoving`/`PI_qONT` call itself fails) | returns **true** — never wedge the caller; the arrival tolerance still gates the step |

`ExecutionContext::WaitUntil` checks the stop flag **before** the predicate on every
iteration and clamps the deadline to `max(timeoutMs, pollMs)` — **no wait is ever
unbounded**, and Stop is honoured within one 50 ms poll.

---

## 6. Timing budget for one micro-move (measured context: 1 mm/s, ≤10 µm steps)

| Phase | Cost |
|---|---|
| `PI_MOV` accept | ~1 GCS transaction |
| Physical travel (10 µm @ 1 mm/s) | ~10 ms + profile ramp |
| `WaitArrive` detect | ≤ 50 ms (poll granularity) + up to 50 ms position staleness |
| `WaitSettle` (`IsMoving` + `qONT`) | ≤ 50 ms per check, typically 1–2 checks |

Two consequences that drove the scan design:
1. The 50 ms poll granularity is the floor of any per-move latency — pushing move
   timeouts lower buys nothing.
2. Because arrival is verified *and* settle is verified, the only stale thing left after a
   move is the **instrument sample that was integrating while the stage travelled** — not
   the mechanics.

Deterministic profiles matter: scans call `set_velocity` once before starting (1 mm/s),
so every micro-move has the same ramp and the same settle behaviour. Without it the settle
time varies with whatever velocity the last jog left on the device.
