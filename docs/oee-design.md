# OEE — station uptime / utilization tracking

How the station spends each 24 h day, recorded as it happens and shown on the
**OEE** page (strip button / View › OEE, Operator). Ported from Horizon
(`Horizon.Workflow/Oee`, FAAM.UI's OEE page and tower light) so the two
machines report the same numbers by the same rules. Total available time = 24 h
(a local calendar day; 23 / 25 h across a DST change).

## States

Every instant is exactly one state, by priority:

| State | Meaning |
|---|---|
| **Down** | A technician declared the station down. Wins over everything — counts even while FireDog is closed, until someone brings it back UP. |
| **Offline** | FireDog was not running. |
| **Running** | `orch_.Busy()`: a recipe, program, script or path move executing. |
| **Standby** | Online, nothing executing, for **less than 30 min** — loading / handling between units. Counts as production. |
| **Idle** | Online, nothing executing for **30 min or more** — the *whole* gap is idle (it turns from Standby to Idle retroactively once it reaches 30 min). |

Production = Running + Standby; Up = Production + Idle.

**Running is `orch_.Busy()` and nothing else** — the same answer every Run
surface gates on. Horizon brackets each engine with a production token; FireDog
already has one definition of "something is driving the machine", and a program
going momentarily idle between queue items must not record a gap. The login
screen counts as online: a machine waiting for someone to sign in is standing by.

## KPIs (over elapsed time — the full day for a past day, "so far" for today)

- **Uptime** = Up ÷ elapsed
- **Availability** = 1 − Down ÷ elapsed
- **Utilization** = Production ÷ elapsed
- **Quality** = Pass ÷ (Pass + Fail) of closed DUTs. A DUT closed without a
  verdict (Close with no status, or auto-closed by the next open / a product
  change) is *aborted* and not judged.
- **OEE** = Utilization × Quality — performance needs an ideal cycle time and is
  not tracked (taken as 100 %).

## The pure half — `core/Oee.{h,cpp}`

States, `OeeReport` (the KPIs), and `OeeClassify`, Horizon's
`OeeCalculator.Classify` written as a +1/−1 edge sweep instead of a midpoint
containment scan per piece (a week of runs is thousands of edges and the page
asks for eight reports). The window is widened by the idle threshold on both
sides, so a gap straddling midnight is judged by its real length. No I/O, no
clock — `now` is passed in. The sweep was checked against a literal port of
Horizon's algorithm on 20 000 random cases with zero mismatches.

## The record — `core/OeeTracker.{h,cpp}`

JSON lines under `<data root>/oee/` (production output, beside the DUT records;
Horizon's SQLite `oee.db` became Notepad-repairable files, like the roster):

| File | Holds |
|---|---|
| `YYYY-MM.jsonl` | CLOSED intervals and units, filed by the UTC month they **start** in: `{"k":"session","a","b"}`, `{"k":"run","a","b","what"}`, `{"k":"unit","t","serial","product","verdict"}` |
| `downtime.jsonl` | the maintenance log, all time: `{"k":"down","id","t","by","by_name","category","reason","station"}` then `{"k":"up","id","t","by","by_name","root_cause","action"}` |
| `live.json` | the OPEN session and run + `last_seen`, rewritten every 60 s and on each run edge |

- **An interval is split at UTC month boundaries before it is written**, so a
  month's file holds exactly what happened in it and a report reads only the
  months its window touches. Month files are read on the first report that
  needs them and cached.
- **Crash recovery**: a clean exit removes `live.json`. Finding one at Start
  means the last process died; its session and run are closed at its last
  heartbeat (≤ 60 s lost). **Down is never closed by recovery** — an open
  `down` with no `up` is a station that is still down, across any restart.
- Idle and Standby are never stored — they are derived from the gaps.
- **UI-thread only, no locks** (the `ProductSession` contract). Everything that
  feeds it already is: `App::UpdateOee` (from `Update`) edge-detects
  `orch_.Busy()`, drains `DutSession::TakeClosedUnits()` (Close runs on engine
  workers too, so it is a queue), and ticks the heartbeat. A torn last line
  from a power cut is skipped with a warning; the rest of the file stands.

## The page — `ui/pages/PageOee.cpp`, `ui/windows/WndMaintenance.cpp`

Live state badge (colour + since), the DOWN / UP form, a day picker, a 24 h
timeline (hover a span for its times), KPI tiles, time breakdown, the day's
downtime log, the last 7 days, and the definitions with the data folder.
Reports are rebuilt by `App::RefreshOeeReports` from `Update` — when the record
changes, the day changes, or once a second while today is shown — never inside
a Draw.

**DOWN / UP** (`kRoleDowntime` = Technician):
- **Signed by the signed-in employee**, not a typed name: the login already
  says who this is, and an employee number is an identity.
- DOWN asks for a **category** (Horizon's presets: Mechanical, Electrical,
  Pneumatic, Motion / stage, Optics / vision, Software / controls, Material /
  parts, Calibration, Preventive maintenance, Utilities, Other) and the
  **reason** (required). UP shows what was recorded and asks for the **root
  cause** and the **action taken** (both required).
- **Inline on the page, never a modal** — a declaration may be made while a run
  is blocked on the always-on-top prompt window. `LogOut` clears a half-written
  form: it is signed by whoever confirms it.
- A failed write refuses the change rather than going down (or up) in memory
  only.

**Maintenance history** (the page's *Maintenance history…* button): every
down → up entry over 7 / 30 / 365 days, a live text filter over category /
reason / root cause / action / who, a summary (count, total, MTTR, longest,
time by category) and *Export CSV* to `<data root>/oee/export/`. *Open folder*
is Technician, like every Explorer shortcut.

**The strip mascot follows the station state too**: a declared DOWN outranks
everything but a safety alert (red, ears down, a blinking `!`), and Idle — no
run for 30 min — puts it to sleep (faded, eye shut, rising z's).

## Tower light — `workflow/TowerLight.{h,cpp}`

| Station state | Green | Yellow | Red | Buzzer |
|---|---|---|---|---|
| Running | steady | | | |
| Standby (< 30 min) | blink 0.5 s | | | |
| Idle (≥ 30 min) | | steady | | |
| Down (declared) | | | steady | steady 3 s on the declaration, then off |
| Waiting on the operator (prompt, manual adjust, crosshair pick, step pause) | | blink 0.5 s | | |
| Safety alert (until acknowledged) | | | blink | 3 beeps on the trip |
| FireDog closed | off | off | off | off |

Priority: safety > down > waiting > running / standby / idle.

- **Owned by `Workflow`, one pump thread (250 ms), joined in the destructor**,
  which then switches every lamp off — `Workflow` dies before
  `HardwareManager`, so the IO devices are still connected. `~Workflow` resets
  it first: the pump reads the safety watch.
- **Started by the client, not by `Workflow::Start`** (`App::Run` calls
  `StartTowerLight`). FireDogRig runs the same `Start` and must never write an
  IO output.
- `App::UpdateOee` decides the mode every frame (`SetTowerMode`); the pump
  reads the safety watch itself, so a trip shows even if the UI stalls. "Waiting
  on the operator" is `App::OperatorWanted()` — the same answer the strip
  mascot hops on.
- Writes go straight to `IIoDevice::SetOutput`, not `Workflow::SetIoOutput`,
  which logs every write: a blinking lamp is two writes a second, forever. Only
  changed pins are written, so a manual toggle on the IO page holds until the
  next change; a failed write is logged and retried after 5 s; a reconnect
  rewrites every pin. A Down carried over a restart does not buzz; declaring UP
  silences the buzzer at once.
- Config: `<machine root>/config/tower_light.local.json`, **written with
  Horizon's seed on the first start** (`IOTower` / `Tower_Green` /
  `Tower_Yellow` / `Tower_Red` / `Tower_Buzzer`, `safety_beeps: 3`,
  `down_buzzer_seconds: 3`, `enabled: true`). There is no shipped default file:
  the compiled defaults are it, and the release zip ships no repo `config/*.json`
  anyway. A device not in `hardware.json` logs once and leaves the tower off; a
  missing pin name leaves that lamp dark.
