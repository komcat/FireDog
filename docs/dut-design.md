# DUT Traceability — Design (draft for approval)

Status: **draft** — implementation starts after this document is approved.
Reference: bobcat_engine's DUT system (`include\dut\DeviceUnderTest`,
`include\sequence\dut\*`, `include\recipe\processes\DUTProcesses.cpp`) — the
lifecycle and record shape are ported; its surveyed defects are deliberately
fixed, not carried over.

## 1. What a DUT is in FireDog

The part being tested/produced. Identity = **(product, serial number)** — the
product comes from the loaded Product (never typed), the serial from the
operator (keyboard or barcode scanner) or a recipe/script. **One active DUT
process-wide** (bobcat's proven model), held by a thread-safe `DutSession`.

**Lifecycle** (bobcat semantics, kept):
- **Open** (default *load-or-create*): resumes an existing record by appending,
  or creates a fresh one. An explicit `new=true` starts a fresh record — never
  the default (in bobcat the raw `create` verb silently truncated). It no
  longer destroys anything: the old `dut.json` is COPIED to
  `dut.replaced-<UTC stamp>.json` beside it first (the open fails if the copy
  does), and the new record carries a `$replaced` event naming that file.
  Readers (View Data, the uploader) only open `dut.json`. Until 2026-10 it was a
  real wipe, and a post-bake probe with `new=true` erased the assembly records
  of ~900 parts; `tools/dut/dut_recover_from_logs.py` rebuilds what the log
  still holds. A re-test of the same part wants `new=false`.
- **Override**: opening a new DUT while one is open **auto-closes** the
  previous — but only *after* the new identity validates, so a failed open
  never loses the current DUT (bobcat's ordering, kept).
- **Close**: appends a close event with an end timestamp and an optional
  **pass/fail status**, then clears the active session. (Bobcat's close wrote
  nothing — you couldn't tell a record was finished, let alone how it ended.
  Fixed.)
- Product unload/switch auto-closes the active DUT.
- Crash safety: every insert flushes (bobcat's behavior, kept), but via
  **write-temp-then-rename** so a crash mid-write can't truncate the record
  (bobcat's live-path `ofstream` could).

## 2. Storage — under the Product, atomically

Everything for one part lives in one folder (bobcat split the JSON and the
images folder; unified here), inside the product's already-reserved `data/`:

```
products/<name>/data/<serial>/dut.json
products/<name>/data/<serial>/images/     (future camera-capture commands)
```

A **loaded product is required** to open a DUT — the product folder *is* the
traceability container. (Bobcat wrote to a CWD-relative `data/` that its
backup page resolved differently — a latent mismatch we don't inherit.)

`dut.json` (append-only entry log, bobcat's shape + fixes):

```json
{
  "product": "siphog888", "product_uid": "ab12cd34",
  "serial": "2087",
  "opened_at": "2026-07-28T19:31:04.120Z",
  "closed_at": "",                  // set on close
  "status": "",                     // "" | "pass" | "fail" (set on close)
  "entries": [
    { "seq": 1, "t": "...", "key": "$open",  "type": "event",  "value": "load_or_create" },
    { "seq": 2, "t": "...", "key": "$run",   "type": "object",
      "value": { "kind": "recipe", "name": "Attach fiber v1", "result": "Completed" } },
    { "seq": 3, "t": "...", "key": "pd_final", "type": "double", "value": 1.23e-6 },
    { "seq": 4, "t": "...", "key": "$close", "type": "event",  "value": "pass" }
  ]
}
```

- Keys stay **non-unique / append-only** (it's a log of what happened, bobcat's
  good instinct), but every entry gets a monotonic `seq` so rows have identity
  beyond a timestamp.
- **`$run` events are automatic**: when a recipe or script run reaches its
  terminal state while a DUT is open, the engine appends `{kind, name,
  result}`. This fixes bobcat's biggest traceability hole — its records never
  said *which recipe produced them*.
- Booleans stay booleans (bobcat's context bridge stringified them).

## 3. `core/DutSession` (new, layer core)

Process-wide singleton like `DataBus`/`Logger` (bobcat used a raw global
pointer; same singularity, done safely). Thread-safe — recipes, scripts, and
the UI all touch it.

```cpp
class DutSession {
public:
    static DutSession& Instance();

    // App wires these on product load/unload; switching products auto-closes.
    void SetProduct(const std::string& name, const std::string& uid,
                    const std::string& dataDir);   // "" name = no product
    struct Info {
        bool open = false;
        std::string product, serial, openedAt, status;
        std::size_t entries = 0;
    };
    Info GetInfo() const;                          // UI + fd.dut accessors

    // Lifecycle. Open validates BEFORE auto-closing the previous DUT.
    bool Open(const std::string& serial, bool destructiveNew, std::string* err);
    bool Close(const std::string& status, std::string* err);   // ""|pass|fail

    // Appends (each one flushes atomically). Fail hard when no DUT is open —
    // no silent no-ops (bobcat's InsertAllFromDataStore soft-passed).
    bool Insert(const std::string& key, const CtxValue& v, std::string* err);
    bool InsertObject(const std::string& key, const nlohmann::json& obj,
                      std::string* err);
    void RecordRun(const std::string& kind, const std::string& name,
                   const std::string& result);     // no-op when closed (by design)
};
```

`RecordRun` is the one deliberate soft path: engines call it unconditionally
at terminal state; testing without a DUT shouldn't fail runs.

## 4. Commands (registry → recipes, editor forms, and Lua for free)

New **DUT** category:

| Command | Params | Notes |
|---|---|---|
| `dut_open` | `serial` String (optional), `new` Bool (default false) | empty serial falls back to ctx key `serial` (set by `prompt_input` or a script); fails before touching the open DUT |
| `dut_close` | `status` Enum `none\|pass\|fail` (default none) | |
| `dut_set` | `key` String, `value` Value | typed literal entry |
| `dut_record` | `key` String, `ctx` String | the ctx→DUT bridge: measure into ctx, record from ctx |
| `dut_record_all` | `prefix` String (default "") | all ctx values (prefix-filtered, prefix stripped) as ONE object entry — bobcat's bulk insert, without its string-to-number guessing |

Plus the missing input primitive this exposes:

- **`prompt_input`** (System): `{message, ctx}` — a modal with a TEXT FIELD;
  the answer lands in the ctx key (Cancel fails the step). This is how an
  operator types or **barcode-scans** a serial mid-recipe (scanners emit
  keystrokes + Enter). Engine side: the prompt handshake gains a kind
  (ok/input) and an answer string; the global modal renders the field.
  (Bobcat's AskForInput auto-accepted defaults in loop mode — ours always
  asks.)

Lua additionally gets read accessors (hand-written, like `fd.read`):
`fd.dut.serial()`, `fd.dut.product()`, `fd.dut.is_open()` — so a script can
reuse the open DUT without re-prompting (the exact gap that broke bobcat's
scripted path, where scripts had a private context and no serial).

## 5. UI (minimal v1)

- **Overview left panel — DUT block** (under the hardware list): current state
  (`DUT: 2087 — 14 entries` or `no DUT`), a serial input + **Open** (load-or-
  create) + **Close** buttons, pass/fail buttons on close. Disabled with no
  product loaded ("load a product first").
- The recipe/scripts **run strips show the active DUT serial** so it's always
  visible what part a run will be attributed to.
- A record viewer (browse `data/`, plot entries) is explicitly **future work**
  — bobcat ended up with a separate explorer tool; FireDog can grow a page
  later. The JSON is human-readable meanwhile.

## 6. Implementation order

1. **D1 — core + commands**: `DutSession` (atomic persistence, lifecycle,
   events), the five `dut_*` commands, `prompt_input` + modal upgrade,
   engine `$run` events, Overview DUT block + run-strip serial display.
2. **D2 (IMPLEMENTED)**: `dut_capture_image` — latest camera frame → PNG in
   the record's `images/` (stb_image_write; filename `<key>_<UTC stamp>.png`,
   fixing bobcat's unlinked date-less names) + a `type="image"` entry with the
   record-relative path. **DUT Records** viewer (View menu, read-only): serial
   list from the product's `data/`, entry table, header + run history.
   Still future: in-viewer image preview, retention/backup.

## 7. Open questions (decide at review)

1. **Auto-open prompt?** When a recipe hits `dut_open` with no serial anywhere,
   drafted: the step FAILS with a clear message (author adds `prompt_input`
   before it). Alternative: `dut_open` implicitly prompts. Drafted explicit —
   implicit prompts inside commands made bobcat's loop mode misbehave.
2. **`$run` granularity** — drafted: one event per recipe/script run (kind,
   name, result). Alternative adds per-step detail; rejected as noise — the
   Logger already has step detail.
3. **Serial charset** — drafted: sanitize to filesystem-safe (alnum, dash,
   underscore, dot); reject others at Open rather than mangling silently.
