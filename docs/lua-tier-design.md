# Lua Tier (Programs) — Phase C Design (draft for approval)

Status: **draft** — implementation starts after this document is approved.
Prerequisites: Phase A (command core) and Phase B (recipe editor) — both done.
This phase delivers the Tier-2 "sophisticated program" surface promised by the
original two-tier decision: a real language over the SAME command registry.

## 1. Goals and non-goals

**Goals**

- Run **Lua 5.4 programs** that orchestrate the machine through the exact
  command vocabulary recipes use — bindings generated from `CommandRegistry`,
  so a command added in C++ appears in recipes, editor forms, AND Lua with no
  extra work (one registry, three clients — completing the design).
- Real control flow the recipe tier deliberately lacks: while-loops over
  measurements, arithmetic, functions, tables, early returns, error handling
  (`pcall`).
- Same operational contract as the recipe engine: one worker thread, one
  orchestration owner at a time, cooperative stop honored inside any wait or
  loop, safety-alert abort, exceptions/errors contained, no UI-thread blocking.
- Programs are **plain `.lua` files**; FireDog lists, runs, stops, and
  monitors them. (Phase C1 shipped them view-only; the in-app editor came
  later as C4, below — the file on disk is still the program.)

**Non-goals (this phase)**

- No in-app text editing of scripts (view + reload only — ImGui is the wrong
  tool for code editing; bobcat's operators author in external editors too).
  *Superseded by C4 (implemented 2026-09) — see §8.*
- No Lua debugger (breakpoints/step) — bobcat proves it's buildable on
  `lua_sethook` later; the hook infrastructure this phase adds is the same one
  a debugger would extend.
- No sandbox hardening beyond disabling obviously dangerous stdlib parts —
  scripts are trusted engineer artifacts living in the product folder, not
  untrusted input.
- No program-level DAG (bobcat's was sequential in practice; a Lua script IS
  the program sequencer).

## 2. Build integration

- **Lua 5.4** via FetchContent from a CMake-wrapped mirror
  (`walterschell/Lua`, static lib target, compiles as C — trivial under MSVC).
- **sol2 v3.3.0** via FetchContent (header-only). Binding layer is ~300 lines
  because the registry already describes every command; we do NOT hand-roll
  the C API like bobcat's 2000-line `ScriptEngine`.
- Both fetched at configure time like GLFW/ImGui — first configure needs
  network, build stays green offline afterwards.

## 3. LuaEngine (`src/command/LuaEngine.{h,cpp}`)

Mirror of `RecipeEngine`, same RAII rules (worker `std::thread`, dtor requests
stop + joins, no detached threads, every exit path lands in a terminal state):

```cpp
class LuaEngine {
public:
    LuaEngine(Workflow& wf, HardwareManager& hw, const CommandRegistry& reg);
    ~LuaEngine();

    struct Env { SavedPosMap positions; MotionMapSet maps; std::vector<Recipe> library; };

    bool Start(const std::string& scriptPath, Env env, std::string* err = nullptr);
    void RequestStop();
    void AnswerPrompt(bool ok);

    enum class State { Idle, Running, Completed, Failed, Stopped };
    struct Status {
        State       state = State::Idle;
        std::string scriptName;
        int         currentLine = 0;     // from the debug hook (progress display)
        std::string lastCommand;         // last fd.* call (what is it doing?)
        std::string error;
        std::string pendingPrompt;
        std::vector<std::pair<std::string, CtxValue>> ctx;
    };
    Status GetStatus() const;
    bool   Running() const;
};
```

- **Stop** is two-layered, both cooperative: (1) every `fd.*` command runs
  through the same `ExecutionContext`, so blocking waits abort within one poll
  interval exactly like recipes; (2) a **count hook** (`lua_sethook`, every N
  instructions) checks the stop flag and raises a Lua error for pure-Lua
  runaway loops (`while true do end` dies too — bobcat's proven mechanism).
  The hook also records the current line for the status display.
- A Lua error that isn't caught by the script's own `pcall` fails the run
  (state `Failed`, message = the Lua traceback's first line, full traceback to
  the Logger). C++ exceptions never cross into Lua or vice versa (sol2
  protected handlers + the engine's outer catch).
- **Mutual exclusion**: `Start` refuses while the RecipeEngine runs, the path
  executor is active, or another script runs; App enforces the converse on
  the recipe side (both Run buttons check both engines). One orchestration
  owner, always.

## 4. The `fd` API (generated bindings)

One registration pass over `CommandRegistry::List()` builds the `fd` table:

```lua
-- every registry command, named args, validated against the ParamSpec schema:
fd.move_to_position{ device="hex-left", position="pos-a41b22", velocity_mm_s=5 }
fd.set_output{ device="IOBottom", pin="vacuum", on=true }
fd.keithley_read{ channel=1, ctx="pd" }
```

- Lua table → `nlohmann::json` params → the SAME `ValidateParams` → execute.
  Bad args raise a Lua error naming the parameter — schema checking for free.
- **A failed command raises a Lua error** (run aborts unless the script
  `pcall`s it) — mirrors the recipe default `on_error: abort`; `pcall` is the
  script's `on_error: continue/retry`.

Hand-written conveniences (small, fixed):

```lua
fd.read("keithley/ch1")        -- latest DataBus sample or nil (no data yet);
                               -- 2nd return = monotonic publish count, for
                               -- fresh-sample gating: local v, n = fd.read(t)
fd.ctx.pd                      -- ctx read (metatable); fd.ctx.pd = 1.5 writes
fd.sleep(500)                  -- interruptible; raises on stop
fd.log("aligned, pd=%.3g", pd) -- printf-style to the Logger ("script" source)
print(...)                     -- redirected to the Logger too
fd.prompt("Continue?")         -- same operator modal; returns false on Cancel
fd.positions("hex-left")       -- name->uid table of saved positions
fd.run_recipe("<uid or name>") -- run a recipe inline (see §5)
```

Stdlib: `base, math, string, table` open; `os`/`io` NOT opened (no file/exec
access from scripts — the engine provides everything a program should touch);
`os.clock`/`os.time` re-exposed individually since timing is legitimate.

## 5. Cross-calls: `run_recipe` and `run_script`

The step-execution core moves out of `RecipeEngine` into a shared internal
runner (`command/StepRunner.{h,cpp}`: the ExecSteps/ExecLeaf logic operating
on an `ExecutionContext` + progress callbacks). Then:

- `RecipeEngine` = worker thread + status plumbing around StepRunner (no
  behavior change).
- **`fd.run_recipe(x)`** = StepRunner invoked synchronously on the Lua worker
  thread with the same `ExecutionContext` — shared ctx, same stop token, sub
  recipes inline as usual. Progress surfaces as `lastCommand`.
- **`run_script` recipe step** (new registered command, `System` category:
  `{file: String}`) = a bare Lua state created on the recipe worker thread,
  same `fd` bindings over the same context. Recursion is capped: a script
  started by a recipe cannot itself `run_recipe` (depth 1 — enough for
  bobcat's proven "escape hatch" pattern, avoids re-entrancy analysis).

## 6. Program files and persistence

- **Location**: the shipped `config/scripts/*.lua` beside the exe (refreshed every build),
  the machine's `<root>/config/scripts/` (what was saved or created on this PC; wins by
  name - `docs/machine-root-design.md`), or the product's
  `products/<name>/scripts/` when a product is loaded — same precedence as
  recipes. First line comment `-- <description>` shows in the list.
- The manifest's `programs` role stays reserved: it will hold program
  *metadata* (uid ↔ file mapping) when programs need stable uids for
  cross-referencing. Phase C1 ships with folder scanning only — files are the
  truth, no duplicate bookkeeping.
- A sample `config/scripts/demo_align.lua` ships as the smoke test (loop over
  `keithley_read` + `move_relative`, threshold exit — exercises commands,
  ctx, control flow, and stop).

## 7. UI — Programs page (View menu, full-page like the others)

Left: script list (scan + Reload; description line under the name). Middle:
**read-only source view** (monospace, line numbers, the executing line
highlighted from `Status.currentLine`) — authoring happens in VS Code; a
"Reload" picks up saved changes instantly. Right: run info — state,
last command, ctx table. Bottom strip: Run / Stop, error display (first
traceback line; full in the Log panel). The global prompt modal already
handles `fd.prompt` (same engine-side pending-prompt pattern).

## 8. Implementation order

1. **C1 — engine + bindings**: CMake fetch, LuaEngine, generated `fd` API +
   conveniences (no `run_recipe`), scripts folder scan, Programs page, demo
   script. Verified: demo runs/stops/fails correctly against sim + hardware.
2. **C2 — cross-calls**: StepRunner extraction (pure refactor, recipes must
   behave identically), `fd.run_recipe`, `run_script` command + palette entry.
3. **C3 — debugger (IMPLEMENTED)**: line breakpoints on the existing hook
   (click a source row; hot-swappable mid-run), Pause / Continue / Step-line,
   and locals inspection while paused (the Lua worker blocks inside the hook;
   pauses land at line boundaries, so a blocking fd.* command finishes first;
   Stop and safety aborts always release a pause). Still usage-driven and NOT
   built: script-local `positions` teaching helpers.
4. **C4 — in-app script editor (IMPLEMENTED 2026-09)**. The Scripts page's
   source view is a real editor: Lua syntax colour, line numbers + the
   breakpoint gutter, auto-indent, Tab indent, find, completion, a compile-only
   syntax check and lint, Save / Revert / New, and the Recipe Editor's
   unsaved-changes guard (`SwitchPage`, list clicks, Reload, the exit
   confirm). State: `ScriptEditorState` (`ui/ScriptEditor.h`, one `App`
   member); code: `ui/pages/PageScripts.cpp`.

   Approach — NOT an embedded editor widget. The ImGuiColorTextEdit family
   was rejected for 1.92+ dynamic fonts and for being a second text-editing
   implementation to maintain. Instead ImGui's own `InputTextMultiline` does
   the editing (caret, selection, clipboard, undo) with its text colour set to
   TRANSPARENT, and the page paints a syntax-coloured rendering of the same
   buffer into the parent draw list BEFORE the widget (a parent's draw list
   renders before its children, so the colour sits under the widget's
   selection highlight and caret — the caret is `ImGuiCol_InputTextCursor`,
   not the text colour). The widget is sized to the WHOLE text and never
   scrolls itself; the child around it scrolls, so gutter, colour, find marks
   and caret share one scroll offset. Two ImGui facts are load-bearing:
   (1) Escape inside an InputText REVERTS the whole edit session — the edit
   callback keeps a `shadow` copy and the page restores it on that
   deactivation, so reaching for Escape to close the completion popup cannot
   lose the typing; (2) ImGui fires ONE callback per frame (Completion, else
   Edit, else Always), so the key handling is one function both events call.
   The completion popup is a window on the TOOLTIP layer (`Begin` with
   `ImGuiWindowFlags_Tooltip | NoFocusOnAppearing`): drawn above everything
   without taking the keyboard, which is what lets Up/Down/Tab/Enter reach
   the editor; a click on a row takes focus, so the editor asks for it back
   and applies the row on its next callback.

   The completion database is `CommandRegistry::List()` (name, params,
   description) plus `LuaFdBuiltins()` in `LuaEngine.cpp` — the hand-written
   bindings listed once, next to the code that binds them, so the editor never
   carries a second copy. The same set is what the lint checks `fd.<name>`
   against; the syntax check is `CheckLuaSyntax` (`luaL_loadbufferx` on a
   bare scratch state: nothing runs, no header leaves the engine TU),
   debounced 0.6 s behind the last keystroke.

   **The engine still runs the FILE.** Run is disabled while the buffer is
   dirty and the editor is read-only while `orch_.Busy()`, so the line the
   debugger reports is always a line of the text on screen. Save preserves the
   file's line endings and rescans the list (the description is the first
   line), keeping the selection by path. The two standard procedures can be
   edited here like any file, and the page keeps saying which copy runs.

## 9. Open questions (decide at review)

1. **Script location naming** — `scripts/` (drafted) vs `programs/`. Drafted
   scripts/ to match bobcat's vocabulary; the manifest role keeps the
   "programs" name for the metadata file.
2. **`fd` command style** — named-args table (drafted,
   `fd.move_to_position{device=..., ...}`) vs positional. Drafted named: it
   matches the schema, reads like the recipe JSON, and survives new optional
   params.
3. **print → Logger level** — drafted Info. Debug would hide script output by
   default; Info matches "the log is the run record".
