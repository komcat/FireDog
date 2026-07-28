# Recipe Core — Design (draft for approval)

Status: **approved — Phase A implemented** (`src/command/`, July 2026).
Scope: the **engine only** (layer 2.5). The recipe *editor* UI is a later, separate
phase; the only UI in scope here is a minimal run/stop/status hook for testing.

## 1. Goals and non-goals

**Goals**

- A **command registry**: every scriptable machine action declared once — name,
  typed parameter schema, execute function — driving `Workflow` (layer 2).
- A **recipe model + executor**: JSON recipes (flat data, no UI types) run on a
  worker thread with cooperative stop, per-step error policy, and simple
  control flow (loop / if / sub-recipe / wait-until).
- Both future frontends hang off the same registry: the graphical recipe editor
  (Tier 1) renders forms from the parameter schemas; the Lua engine (Tier 2)
  binds each command as a Lua function. Neither is built in this phase, but
  nothing in the core may preclude them.

**Non-goals (this phase)**

- No recipe editor UI, no Lua, no program-level DAG (bobcat's DAG executes
  sequentially anyway — an ordered list with conditions covers it later).
- No debugger/breakpoints yet (design leaves room: the engine already pauses at
  step boundaries, so step-mode is an additive feature).
- No new hardware capability — commands only compose what `Workflow` exposes.

## 2. Placement and dependency rules

New directory **`src/command/`** — layer **2.5**, between workflow and UI:

```
UI (App, pages)  →  command (registry, recipe engine)  →  workflow  →  hardware
```

- `src/command/` includes **no ImGui/UI headers**. The UI observes the engine
  through value-type snapshots (same philosophy as `ConnState` /
  `HardwareManager::Snapshot()`) and issues Run/Stop calls.
- Commands call **only `Workflow` methods and read-side state** (DataBus
  snapshots, `HardwareManager` status). Never a device directly.
- **Ownership**: `App` owns `CommandRegistry` and `RecipeEngine`, declared
  *after* `workflow_` so they are destroyed first (`~RecipeEngine` joins the
  worker thread while `Workflow` is still alive — same discipline as
  `SafetyWatch`).

Files:

| File | Contents |
|---|---|
| `command/CommandSpec.h` | `ParamSpec`, `CommandSpec`, `ParamValues` (validation + typed access) |
| `command/CommandRegistry.{h,cpp}` | registration, lookup, enumeration |
| `command/CommandsMotion.cpp` | motion + pneumatic command implementations |
| `command/CommandsIo.cpp` | IO + wait commands |
| `command/CommandsInstruments.cpp` | Keithley / CLD / PSU / camera commands |
| `command/CommandsSystem.cpp` | delay, log, ctx ops, read-topic |
| `command/ExecutionContext.{h,cpp}` | stop token, ctx store, interruptible waits |
| `command/Recipe.{h,cpp}` | recipe/step model, JSON load/save, validation |
| `command/RecipeEngine.{h,cpp}` | worker-thread executor + status snapshot |

### 2.1 Coding rules — RAII, mission-critical discipline

This layer commands real machine motion, lasers, and pneumatics, so it is held
to a stricter standard than the rest of the codebase — in the spirit of
mission-critical C++ guidelines (MISRA C++ / JSF AV C++), enforced pragmatically
rather than by full rulebook adoption:

- **RAII for every resource, no exceptions.** No naked `new`/`delete`
  (`std::unique_ptr` only), no manual `lock()`/`unlock()`
  (`std::scoped_lock`/`std::lock_guard`/`std::unique_lock` only), files via
  fstream. The worker thread is `std::jthread` (or a join-in-destructor
  wrapper): **`~RecipeEngine` requests stop and joins — always.** Detached
  threads are forbidden; bobcat's zombie-thread watchdog pattern must not be
  ported.
- **Every exit path leaves a defined state.** The step loop catches all
  exceptions at the step boundary (a throwing command = a failed step, run →
  `Failed`); no exception ever crosses the thread boundary or kills the
  process. Destructors are `noexcept`. On any run end (Completed / Failed /
  Stopped / engine destruction) the engine transitions state under the mutex
  and logs the terminal cause — there is no path that leaves the engine
  "Running" with no thread behind it.
- **Bounded everything.** Every wait has a timeout (`WaitUntil` requires one);
  `while`-loops carry `max_iterations` (default 10000); step nesting depth is
  capped (guard at load, e.g. 32) so the recursive executor cannot overflow
  the stack; sub-recipe inlining is cycle-checked. No unbounded recursion
  anywhere in the engine.
- **No silent failure.** Every `CommandResult` error, abort cause, and
  validation failure is logged; a step cannot fail without a log line saying
  which step and why.
- **Hardware-off is never implicit.** The engine stops *itself* (and motion,
  via the existing safety path) but never switches instruments or outputs as a
  side effect of abort — de-energizing is either an authored recipe step or
  the operator's/safety layer's explicit action (the same principle as the
  CLD driver's connect/disconnect behavior).
- `/W4`-clean like the rest of the project; the recipe core should also be
  clean under `/analyze` (MSVC static analysis) since it is the layer most
  worth the extra scrutiny.

## 3. Command registry

### 3.1 Parameter schema

Typed schemas are the deliberate upgrade over bobcat's `map<string,string>`:
they buy load-time validation now, and auto-generated editor forms + Lua
argument checking later.

```cpp
enum class ParamType {
    Bool, Int, Double, String,
    DeviceMotion,   // motion fleet device name
    DeviceIo,       // IO fleet device name
    Slide,          // pneumatic slide name
    PositionUid,    // SavedPositions uid (stable across renames)
    MapName,        // MotionMap name
    PinName,        // IO pin name (resolved against DeviceIo param)
    TopicName,      // DataBus topic
    Enum,           // one of options
};

struct ParamSpec {
    std::string    name;
    ParamType      type;
    bool           required = true;
    nlohmann::json defaultValue;          // used when !required
    double         minVal = 0, maxVal = 0; // numeric bounds (minVal<maxVal → active)
    std::vector<std::string> options;     // for Enum
    std::string    help;                  // one line, future editor tooltip
};
```

The `DeviceMotion`/`PositionUid`/`PinName`/… types are strings on the wire;
their point is *semantic*: validation resolves them against live config, and
the future editor renders the right picker widget instead of a text box.

### 3.2 Command spec

```cpp
struct CommandResult { bool ok = true; std::string error; };

struct CommandSpec {
    std::string name;        // stable id, snake_case: "move_to_position"
    std::string category;    // "Motion", "IO", "Instruments", "System", ...
    std::string description; // one line
    std::vector<ParamSpec> params;
    // Blocking is allowed and expected; implementations MUST poll
    // ctx.ShouldStop() inside any wait loop.
    std::function<CommandResult(ExecutionContext&, const ParamValues&)> execute;
};
```

`CommandRegistry` is a name→spec map with `Register`, `Find`, and `List`
(sorted, for the future editor palette and Lua binding loop). Built-in commands
are registered by free functions per domain file
(`RegisterMotionCommands(reg, wf)` …) called once from `App` after
`Workflow::Start`. The execute lambdas capture `Workflow&` — the registry
itself has no back-pointer to anything.

`ParamValues` wraps the step's `nlohmann::json` params after validation:
`p.GetDouble("volts")`, `p.GetString("device")`, etc. — defaults already
substituted, types already checked, so implementations read without guarding.

### 3.3 Initial command vocabulary (v1)

Grounded 1:1 in today's `Workflow` API — nothing speculative.

**Threading rule (hard requirement): no motion — and no blocking command of
any kind — ever runs on the UI thread.** Every command executes on the
`RecipeEngine` worker thread; blocking commands (marked ⏳) wait there via the
`ExecutionContext` helpers. The UI thread only issues Run/Stop and polls
`GetStatus()` — a running recipe can never freeze or stutter the interface.

**Motion**
- `move_to_position` ⏳ — device + `PositionUid`; issues
  `Workflow::MoveToPosition` with the position's axis targets, then polls live
  positions (`motion/<dev>/<axis>` DataBus topics) until all targeted axes are
  within tolerance (same 0.01 mm rule as `TickPath`) or `timeout_ms`.
- `move_path` ⏳ — device + `MapName` + `from` `PositionUid` + `to`
  `PositionUid`. **Strict start**: the step first verifies the device's live
  position matches `from` on every axis the position defines (within
  `start_tol_mm`, default the executor's 0.01 mm); if not, the step **fails
  immediately** — no nearest-node snapping, no implicit move to the start
  node. Recipes reach the start explicitly (`move_to_position from` as the
  preceding step), so every motion in a run is authored and auditable; the
  nearest-node convenience remains a Motion-page-only feature. Then
  `FindMapPath(from → to)` and the command executes the waypoints on the
  recipe worker thread: issue leg → wait-arrived → next leg, with stop-token
  checks in every wait. It does **not** use `StartPathMove`/`TickPath` (that
  executor's state is UI-thread-only, see `Workflow.h`).
  Prerequisite refactor: the route-building logic currently inline in the
  Motion page's Go button (`App.cpp` ~2217–2273: XYZ edge-cost lambda,
  uid→`PathWaypoint` targets restricted to the device's axes) moves into a
  shared helper (e.g. `BuildMapRoute` beside `FindMapPath` in
  `core/MotionMap`), which both the UI Go button and this command call — one
  implementation of route planning, two executors (UI `TickPath`, recipe leg
  loop).
- `move_relative_global` ⏳ — device + global dx/dy/dz mm (via `JogGlobal`
  with a single step), wait-arrived. For small programmed offsets.
- `stop_motion` — device; `Workflow::StopMotion`.

**IO & pneumatics**
- `set_output` — `DeviceIo` + `PinName` + on/off (`SetIoOutputByName`).
- `pulse_output` ⏳ — set on, interruptible sleep `ms`, set off.
- `wait_input` ⏳ — `DeviceIo` + `PinName` + wanted state + `timeout_ms`;
  polls the driver's cached `Status()` through a `Workflow` read accessor.
- `clear_latch` — device + mask (`ClearIoLatch`).
- `extend_slide` ⏳ / `retract_slide` ⏳ — slide name; calls the non-blocking
  `ExtendSlide`/`RetractSlide`, then waits on the slide state machine reaching
  the sensor-confirmed position (timeout from the slide's config).

**Instruments**
- `keithley_bias` (`SetKeithleyBias`), `keithley_output_off`,
  `keithley_nplc`, `keithley_range_auto` / `keithley_range_fixed`.
- `cld_tec` on/off, `cld_laser` on/off, `cld_set_temperature`,
  `cld_set_laser_current`.
- `wait_for_temperature` ⏳ — target °C ± tolerance on `cld/temp`, timeout.
- `psu_set_vi` (channel, V, I), `psu_output` (channel, on/off).
- `camera_exposure` (`SetCameraExposure`).

**System / data**
- `delay` ⏳ — interruptible sleep `ms`.
- `log` — level + message (with `{ctx.key}` substitution).
- `ctx_set` — key + value (number/string/bool) into the run context.
- `read_topic` — sample a `TopicName` via `DataBus::Snapshot` into a ctx key.
  This is the generic instrument→ctx bridge (`keithley/ch1` → `"pd_current"`).
- `wait_until` ⏳ — the workhorse: `TopicName` *or* ctx key, comparison op,
  threshold, `timeout_ms`, poll interval. Covers "wait for photocurrent above
  X", "wait for temperature settle", etc. without bespoke commands.
- `prompt` ⏳ — message; blocks until the operator acknowledges (engine
  exposes the pending prompt in its status snapshot; the UI shows a modal and
  calls `AnswerPrompt`). OK/Cancel — Cancel fails the step.

Deliberately **excluded**: connect/disconnect commands (session setup is the
operator's job; a recipe assumes its devices are up — validation checks this
at run start), jog (interactive by nature), IO-watch/safety mutation.

## 4. Execution context

One instance per run, passed to every command:

```cpp
using CtxValue = std::variant<bool, double, std::string>;

class ExecutionContext {
public:
    // Cooperative cancellation — the only stop mechanism commands see.
    bool ShouldStop() const;                       // atomic flag
    bool Sleep(int ms);                            // false if stopped early
    // Poll pred every pollMs until true / timeout / stop.
    enum class WaitResult { Ok, Timeout, Stopped };
    WaitResult WaitUntil(std::function<bool()> pred, int timeoutMs, int pollMs = 20);

    // String-keyed run state (bobcat's SequenceContext, typed).
    void                    CtxSet(const std::string& key, CtxValue v);
    std::optional<CtxValue> CtxGet(const std::string& key) const;

    void Log(LogLevel lvl, const std::string& msg);  // -> firedog::Logger

    Workflow& Wf();   // commands issue every action through this
};
```

Rules:

- Every blocking command waits **only** through `Sleep`/`WaitUntil`, so Stop
  interrupts any wait within one poll interval. **No zombie-thread watchdog**
  (bobcat's known wart): FireDog's `Workflow` calls are themselves bounded, so
  cooperative checking suffices.
- The ctx store is mutex-guarded (the UI status snapshot may read it while a
  command writes).
- A **critical safety alert aborts the run**: the engine checks
  `Workflow::SafetyAlertActive()` between steps, and motion commands already
  fail fast via `SafetyBlocked()` inside `Workflow`. Hard axis stop remains
  `SafetyWatch` → `StopAllMotion()` — decoupled from graceful cancellation,
  exactly like bobcat.

## 5. Recipe model

### 5.1 JSON shape — nested steps

```json
{
  "uid": "rcp-7f3a9c",
  "name": "Attach fiber v1",
  "description": "",
  "steps": [
    { "type": "set_output", "nickname": "vacuum on", "enabled": true,
      "params": { "device": "wago-main", "pin": "vacuum" , "on": true },
      "on_error": "abort" },

    { "type": "loop", "params": { "count": 3 },
      "children": [
        { "type": "move_to_position",
          "params": { "device": "hex-left", "position": "pos-a41b22", "timeout_ms": 30000 },
          "on_error": { "retry": 2 } },
        { "type": "read_topic", "params": { "topic": "keithley/ch1", "ctx": "pd" } }
      ] },

    { "type": "if", "params": { "key": "pd", "op": ">", "value": 1e-6 },
      "children":  [ { "type": "log", "params": { "level": "info", "message": "aligned, pd={pd}" } } ],
      "else":      [ { "type": "stop_recipe", "params": { "message": "no light" } } ] }
  ]
}
```

- **Nested `children`, not bobcat's flat list with begin/end markers.** The
  executor recurses naturally, malformed nesting is impossible by construction,
  and the future editor renders it as an indented list (flat view is a
  rendering concern, not a storage one).
- Common step fields: `type`, `params`, optional `nickname`, `enabled`
  (default true — disabled steps are skipped but kept, bobcat's most-used
  editing feature), `on_error`: `"abort"` (default) | `"continue"` |
  `{"retry": N}` (N re-executions, then abort).

### 5.2 Control flow is engine-native, not registry commands

The executor itself interprets five structural step types; the registry holds
only leaf actions. This keeps the registry's contract simple (a command = one
parameterized action) and keeps flow semantics in exactly one place:

- `loop` — `count: N` **or** `while: {key|topic, op, value}` (condition
  re-evaluated each iteration; `max_iterations` guard, default 10000). The
  while-form is the deliberate fix to bobcat's fixed-count-only loops.
- `if` / `else` — condition `{key|topic, op, value}`; ops:
  `> < >= <= == != is_true is_false exists not_exists` (bobcat's proven set).
  A full expression parser (AND/OR/parens) is a later, additive upgrade.
- `group` — organizational only (nickname + children), emits nothing.
- `sub_recipe` — `uid` of another recipe; inlined at load with
  cycle detection (bobcat's self-reference check, generalized).
- `stop_recipe` — end the run early; `ok: true|false` variant for
  success-stop vs. fail-stop.

### 5.3 Validation

`Recipe::Validate(registry, resolveCtx)` runs at load and again at run start,
returning a list of `{stepPath, message}`:

- unknown command type; missing/mistyped/out-of-range params (per `ParamSpec`);
- unresolvable references: device names against the fleets, `PositionUid`
  against `SavedPositions`, pin names against the IO config, map names;
- at run start additionally: every referenced device **Connected** (recipes
  don't connect hardware — see §3.3).

Load is tolerant like every other FireDog loader (bad file → error list, never
throw).

### 5.4 Persistence

Follows the product/`*.local.json` conventions that already exist:

- No product loaded: `config/recipes.local.json` (fallback `recipes.json`) —
  one file, array of recipes, keyed by `uid`.
- Product loaded: `products/<dir>/{uid}.recipes.json` via the manifest role
  `"recipes"` that `Product.h` already reserves (`App.cpp` currently skips it
  pending real data — this fills it in).
- Recipes reference positions by **uid**, so position renames never break
  recipes (the existing SavedPositions design contract).

## 6. Recipe engine (executor)

```cpp
class RecipeEngine {
public:
    RecipeEngine(Workflow& wf, CommandRegistry& reg);
    ~RecipeEngine();                    // Stop() + join

    bool Start(Recipe recipe);          // false if a run is active / validation fails
    void RequestStop();                 // cooperative; honored within one poll interval
    void AnswerPrompt(bool ok);         // resolves a blocking `prompt` step

    enum class State { Idle, Running, Stopping, Completed, Failed, Stopped };
    struct Status {
        State       state = State::Idle;
        std::string recipeName;
        std::string currentStep;        // "3.2.1  move_to_position (retry 1/2)"
        std::size_t leafDone = 0, leafTotal = 0;   // progress over enabled leaves
        std::string error;              // set when Failed
        std::string pendingPrompt;      // non-empty while a prompt blocks
        std::vector<std::pair<std::string, CtxValue>> ctx;  // snapshot for display
    };
    Status GetStatus() const;           // mutex-guarded copy, cheap, UI-safe
};
```

- **One worker thread**, one run at a time (matches the machine: one
  orchestration owner; the path executor / jog remain available when idle, and
  `Start` refuses while `Workflow::PathActive()`).
- The step loop: check stop flag + safety → resolve/execute step → apply
  `on_error` policy → update status. Every status mutation goes through one
  mutex; the UI polls `GetStatus()` per frame like every other snapshot in the
  app.
- Step results and errors also go to `firedog::Logger`, so the existing log
  viewer is the run log — no new UI needed to see what happened.
- Pause/step-mode/breakpoints: not in v1, but the design point where they slot
  in is the step-boundary check (a condition_variable wait, as bobcat does).

## 7. Minimal test hook (not the editor)

To exercise the core before any editor exists, one temporary addition to the
**View menu**: a small "Recipe Runner" window — recipe picker (from the loaded
recipe file), Run / Stop buttons, status line + progress from `GetStatus()`,
prompt modal when `pendingPrompt` is set. Hand-written JSON recipes drive
testing (simulated IO/instruments make this workable without hardware). This
window is throwaway; the real editor replaces it in the next phase.

## 8. Phasing

1. **Phase A (this design)** — `src/command/` core: registry + specs, execution
   context, recipe model + validation + persistence, engine, v1 vocabulary,
   test-hook window. Verified by building clean + running hand-written recipes
   against simulated devices.
2. **Phase B** — recipe editor UI (schema-driven forms from `ParamSpec`,
   indented step list, per-step enable/disable, run controls) — designed and
   approved separately after Phase A lands.
3. **Phase C** — Lua tier: sol2 via FetchContent, bindings generated from the
   registry (`fd.move_to_position{device=..., position=...}` plus `ctx`, `log`,
   raw read accessors), debug-hook stop polling, `run_recipe` command both ways
   (Lua→recipe as a call; recipe→Lua as a `run_script` step type).

## 9. Open questions (decide at review)

1. ~~**`move_path` destination form**~~ — **RESOLVED**: explicit `from` + `to`
   with strict start verification (step fails if the device is not already at
   `from`); no nearest-node auto-start in recipes. See §3.3.
2. **`ctx` initial values** — should a recipe declare `"inputs"` (name,
   type, default) that the runner/prompt fills before start? Drafted: no for
   v1; `ctx_set` steps cover it, and inputs become an editor-phase feature.
3. **Progress granularity** — leaf-count progress (drafted) vs. per-step
   duration estimates. Drafted: leaf count; estimates need history we don't
   have yet.
