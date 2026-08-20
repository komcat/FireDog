# SMU polling subsystem (Keithley 6482 / 2602B)

Two channels of current on the DataBus. This is the **feedback signal source** for active
alignment: the XCAN / raster scripts read `keithley/ch1`, so the poll loop's sample
cadence directly sets the scan cycle time.

Two instruments answer that, chosen by `type` in `config/hardware.json`:

| `type` | Instrument | Transport | Command set |
|---|---|---|---|
| `keithley_6482` | dual picoammeter + V source | GPIB / USB | SCPI |
| `keithley_2602b` | dual-channel SourceMeter SMU | LAN | TSP (Lua on the instrument) |

Both implement **`ISmu`** (`src/hardware/instruments/ISmu.h`), so everything above layer 1
— Workflow's command surface, the page, the chart, the recipe commands, the topics — is
written once and does not know which one is attached. §1–§5 describe the 6482 and the
shared machinery; **§6 is what is different about the 2602B.**

Files: `src/hardware/instruments/ISmu.h`, `Keithley6482.{h,cpp}`, `Keithley2602B.{h,cpp}`
(layer 1), `src/workflow/Workflow.cpp` (layer 2 bring-up + command surface),
`src/ui/pages/PageKeithley.cpp` (layer 3).

---

## 1. Module structure

```
config/hardware.json  "instruments"[type=keithley_6482]
        │  HardwareConfig::Load -> KeithleyConfig
        ▼
Workflow::StartKeithley  ──► keiCfg_ (kept for reconnects)
        │                     hw_.SetKeithleyName()
        ▼
Workflow::LaunchKeithleyConnect
        │  type contains "2602" ? Keithley2602B : Keithley6482
        │  -> HardwareManager::SetKeithley(unique_ptr<ISmu>) (sole owner)
        │  std::async: Connect() -> SetVoltageBias(bias) -> StartPolling(interval, t1, t2)
        │              ConnState: Connecting -> Connected | Failed
        ▼
Keithley6482 (PIMPL)                     poll thread
        │  visaMutex : all VISA I/O      ── :READ? ──► ParseDual ──►
        │  cfgMutex  : runtime cfg                                  │
        │  atomics   : biasVolts, outputOn, connected, polling      │
        ▼                                                           ▼
                                            DataBus  keithley/ch1 , keithley/ch2
                                                    │            │
                                    UI chart / Aux overlay      fd.read() in Lua scans
```

**Ownership** follows the project rule: `HardwareManager` owns the driver, `Workflow`
owns the sequence, `App` only reads. The UI never calls the driver directly — every page
button goes through `Workflow::ConnectKeithley / SetKeithleyNplc / SetKeithleyBias /
SetKeithleyRangeAuto / SetKeithleyRangeFixed / SetKeithleyPollInterval`, each of which
resolves through `ConnectedKeithley()` (null unless `ConnState::Connected`).

### Class surface (`ISmu.h` — both drivers)

| Group | Method | Notes |
|---|---|---|
| Lifecycle | `Connect()` / `Disconnect()` / `IsConnected()` | `Connect` is blocking — Workflow runs it on a `std::async` thread |
| Source | `SetVoltageBias(volts)` , `OutputOff()` | read-back verified against `:OUTP?` |
| Measure cfg | `SetNplc(n)` , `SetRangeAuto()` , `SetRangeFixed(amps)` | safe while polling (same mutex as `:READ?`) |
| Cheap UI reads | `BiasVolts()` , `OutputOn()` , `GetConfig()` | atomics / `cfgMutex` — never block on VISA |
| Streaming | `StartPolling(ms, topicCh1, topicCh2)` , `StopPolling()` , `IsPolling()` | one `std::thread` |
| Diagnostics | `DeviceId()` (`*IDN?` string), `LastError()` | |

PIMPL: `<visa.h>` never leaves the .cpp. Without `HAS_VISA` a **simulated Impl** with the
identical surface compiles (sine currents, rate derived from NPLC), so the DataBus →
chart → script pipeline is exercisable with no instrument attached.

### Config keys (`config/hardware.json`, section `instruments`)

| Key | Default | Effect |
|---|---|---|
| `type` | `keithley_6482` | **selects the driver** — anything containing `2602` picks the 2602B |
| `visa_resource` | `GPIB0::1::INSTR` | `viOpen` target (`TCPIP0::<ip>::INSTR` for the 2602B) |
| `enabled_channels` | `[1,2]` | which channels get a DataBus stream |
| `measurement_nplc` | 5.0 (repo file: **0.1**) | integration time → **the sample rate** |
| `auto_range` / `fixed_range` | true / 0.0 | `:RANG:AUTO ON` or `:RANG <A>` |
| `poll_interval_ms` | 100 (repo file: **0**) | 0 = "as fast as the instrument allows" |
| `bias_voltage` | 1.0 | applied to ch1 after every connect |

---

## 2. SCPI command reference (6482)

### 2.1 Connect / init sequence (`Impl::Connect`, under `visaMutex`)

| # | Call | Purpose |
|---|---|---|
| 1 | `viOpenDefaultRM` , `viOpen(resource, …, 5000)` | open session, 5 s open timeout |
| 2 | `viSetAttribute(VI_ATTR_TMO_VALUE, 5000)` | 5 s I/O timeout |
| 3 | `*IDN?` | identity; **failure aborts the connect** and closes both sessions |
| 4 | `*RST` , `*CLS` | known state, clear status |
| 5 | `:SYST:ZCH OFF` | zero-check off (input connected) |
| 6 | `:FORM:ELEM CURR1,CURR2` | one read returns **both** channels: `"1.2e-6,5.6e-7"` |
| 7 | `:SENS1:CURR:NPLC <n>` , `:SENS2:CURR:NPLC <n>` | integration time per channel |
| 8 | `:SENS1|2:CURR:RANG:AUTO ON` *or* `:SENS1|2:CURR:RANG <A>` | ranging |
| 9 | `:SENS1:CURR:AVER:STAT OFF` , `:SENS2:CURR:AVER:STAT OFF` | no filter — speed |
| 10 | `:SYST:AZER OFF` | no auto-zero — speed |
| 11 | `:DISP:ENAB OFF` | front-panel refresh throttles reads; restored on disconnect |
| 12 | `*CLS` | clean error queue before streaming |

### 2.2 Steady state (poll thread)

| Call | Contract |
|---|---|
| `:READ?` | INIT + FETCh in ONE transaction; returns `ch1,ch2` per `:FORM:ELEM` |

`ParseDual` splits on `,`; unparsable fields stay at the sentinel `-999.0` and are **not**
published (a bad token can't poison the chart or a scan sample).

### 2.3 Runtime commands (UI / Workflow, all under `visaMutex`)

| API | SCPI issued | Verification |
|---|---|---|
| `SetVoltageBias(v)` | `:SOUR:VOLT <v>` then `:OUTP ON` | `:OUTP?` read-back → `outputOn`; returns true only if the instrument confirms ON |
| `OutputOff()` | `:OUTP OFF` | `:OUTP?` read-back |
| `SetNplc(n)` | `:SENS1:CURR:NPLC n` , `:SENS2:…` | writes `cfg.nplc` under `cfgMutex` on success |
| `SetRangeAuto()` | `:SENS1|2:CURR:RANG:AUTO ON` | |
| `SetRangeFixed(a)` | `:SENS1|2:CURR:RANG <a>` (scientific) | |

`VerifyOutputInternal()` also drains the SCPI error queue: up to 5× `:SYST:ERR?` until a
reply starting `"0,"` (empty). Every non-empty entry is logged `FD_WARN("keithley", …)`
and stored in `lastError`, so a silently rejected command surfaces in the log band.

### 2.4 Disconnect

`StopPolling()` (joins the thread) → `:OUTP OFF` → `:SYST:ZCH ON` → `:DISP:ENAB ON` →
`viClose(inst)` , `viClose(rm)`. Bias is **always** dropped on disconnect.

---

## 3. Threading model

| Thread | Touches | Lock |
|---|---|---|
| UI (main) | `BiasVolts()`, `OutputOn()`, `GetConfig()`, `IsConnected/IsPolling` | atomics / `cfgMutex` only — never `visaMutex` |
| UI → Workflow commands | bias / NPLC / range | takes `visaMutex` (waits at most one `:READ?`) |
| Connect thread (`std::async`, Workflow) | `Connect` + first bias + `StartPolling` | `visaMutex` |
| Poll thread | `:READ?` + `DataStream::Publish` | `visaMutex` for the transaction only |

Rules that keep this safe:
* Two mutexes on purpose — a per-frame UI read must never queue behind a VISA transaction.
* `StopPolling()` is idempotent (`polling.exchange(false)` + `join`) and is called by
  `Disconnect()` and by `~Impl` — no detached threads.
* The poll loop sleeps **1 ms** even in continuous mode, purely so a waiting UI command
  can acquire `visaMutex` between reads instead of starving.

---

## 4. Timing

**NPLC governs the rate, not `poll_interval_ms`.** One PLC = 1/mains ≈ 16.7 ms, charged
per channel, plus GPIB transaction overhead.

| `measurement_nplc` | Approx. published rate | Approx. period |
|---|---|---|
| 0.1 | ~80 Hz | ~12 ms |
| 1 | ~30 Hz (bench: ~110 ms stationary cadence incl. auto-range hunting) | ~33 ms |
| 5 | ~12 Hz | ~85 ms |

`poll_interval_ms` semantics:
* `> 0` — **rate cap**: the loop sleeps `interval − elapsed`, so it never publishes faster
  than that period. Use to throttle a fast instrument.
* `0`/negative — **continuous**: GPIB + integration latency paces the loop (1 ms yield only).

Auto-range remains the dominant per-read cost once `:DISP:ENAB OFF` is set; a fixed range
via `SetRangeFixed` removes the range-hunt jitter when the signal band is known.

Consumer side: `DataStream` keeps a 10 s time window (20000-sample backstop) and a
**monotonic publish counter** `Count()`. That counter is what the scan scripts gate on —
see `docs/xcan-scan-design.md` §4.

---

## 5. Known traps (do not "fix" these)

1. **Never `:INIT:CONT ON` + `:FETC?`.** With continuous init the 6482 answers every
   `:READ?` with error −213 "Init ignored" and the stream stalls at 0 Hz. `:READ?` per
   iteration is the proven path (matches the Python `smart_multimeter.py` reference).
2. **No fixed inter-read sleep.** The instrument already paces itself through NPLC.
3. **Bias truth comes from the instrument.** `SetVoltageBias` returns
   `ok && outputOn.load()`; do not report success from the write alone.
4. A failed bias at bring-up **warns but does not abort** — polling still starts, because
   an unbiased measurement is still useful for diagnosis.

---

## 6. The 2602B — same contract, different everything else

`Keithley2602B.{h,cpp}`. The class surface, the two-mutex threading model (§3), the
`poll_interval_ms` semantics and the "NPLC governs the rate" rule (§4) are all identical —
that is the point of `ISmu`. What differs is the wire and one piece of physics.

### 6.1 TSP is not SCPI

Every line sent is **Lua executed on the instrument**. Assignments are silent
(`smua.measure.nplc = 0.1`); bytes come back only when the chunk calls `print()`. So there
is no `:READ?` and no `?`-suffix query form — a "query" here is `print(<expr>)`.

Two lines go out **before the first query**, and the order is load-bearing:

```
localnode.prompts = 0      -- else "TSP>" tokens interleave and every read is one reply late
localnode.showerrors = 0   -- else error text lands between a print() and its value
```

### 6.2 Connect / init sequence

| # | Call | Purpose |
|---|---|---|
| 1 | `viOpen(resource, …, 5000)` , `TMO 5000` | open session |
| 2 | `VI_ATTR_TERMCHAR_EN = TRUE` , `TERMCHAR = '\n'` | **required** for a `::SOCKET` session — a raw socket has no END bit to end a read. Harmless on VXI-11 |
| 3 | `viClear` | drop what a previous session left buffered |
| 4 | `localnode.prompts = 0` , `localnode.showerrors = 0` | see §6.1 |
| 5 | `*IDN?` | identity; **failure aborts the connect** |
| 6 | `errorqueue.clear()` , `format.data = format.ASCII` , `format.asciiprecision = 8` | |
| 7 | `smuX.reset()` | channel defaults: DC-volts source, 0 V, 100 mA compliance, output OFF |
| 8 | `smuX.source.func = smuX.OUTPUT_DCVOLTS` , `smuX.source.levelv = 0` | |
| 9 | `smuX.measure.nplc = <n>` | integration time — same meaning as the 6482 |
| 10 | `smuX.measure.autozero = smuX.AUTOZERO_OFF` , `smuX.measure.filter.enable = smuX.FILTER_OFF` | speed, same two knobs as §2.1 steps 9–10 |
| 11 | `smuX.measure.autorangei = smuX.AUTORANGE_ON` *or* `smuX.measure.rangei = <A>` | ranging |
| 12 | `display.smuX.measure.func = display.MEASURE_DCAMPS` | front panel shows what the driver is reading |

`smuX` walks only the **enabled** channels (`smua` ← ch1, `smub` ← ch2), so a one-channel
config never touches the other SMU.

### 6.3 Steady state

One `print()` returns both channels in one round trip, matching the 6482's `:FORM:ELEM`
trick:

```lua
print(string.format("%.6e,%.6e", smua.measure.i(), smub.measure.i()))
```

Built once in `StartPolling`. A **disabled** channel is padded with a literal `0` rather
than dropped, so the reply always has two comma-separated fields and `ParseDual` (§2.2) is
byte-for-byte the same parser.

### 6.4 The one real behavioural difference: output ON to measure

A 6482 is an ammeter — it measures with its voltage source off. **An SMU measures nothing
with its output off**: it reads ~0 A, which on the chart is indistinguishable from a dark
photodiode. So `SetVoltageBias` switches **every enabled channel ON** — ch1 at the bias
voltage, ch2 at 0 V — and `OutputOff` drops them all. `OutputOn()` is true only when every
enabled channel reads back on.

This is why the bias at bring-up matters more here than on the 6482: without it there is
no signal at all, not merely an unbiased one.

### 6.5 NPLC and range windows differ

* **NPLC is 0.001 … 25** on the 2600B (the 6482 allows more). Out-of-window values are
  *silently ignored* by the instrument — it queues an error and keeps the old value — so
  `SetNplc` **clamps** and stores the clamped value in `cfg`, and the page shows what the
  instrument is actually doing rather than what was typed.
* **The UI's fixed-range table is the 6482's decades** (2 nA … 20 mA); the 2602B has
  different ranges and its floor is 100 nA. It does not reject an in-between request — it
  selects the smallest range that *contains* it — so `SetRangeFixed` reads
  `smuX.measure.rangei` back and stores the range the instrument actually chose.

### 6.6 Errors and disconnect

`DrainErrorsInternal` queries `print(errorqueue.count)` and then `print(errorqueue.next())`
up to 10 times, logging each `FD_WARN("keithley", …)` and keeping the last in `lastError` —
the TSP equivalent of the 6482's `:SYST:ERR?` drain, and it runs after every runtime setter,
not only after an output change.

Disconnect: `StopPolling()` → `smuX.source.output = smuX.OUTPUT_OFF` for every enabled
channel → `viClose`. Outputs are **always** dropped, same as the 6482.

### 6.7 VISA resource forms

```
TCPIP0::192.168.0.111::INSTR          VXI-11 — try this first
TCPIP0::192.168.0.111::5025::SOCKET   raw SCPI socket — needs the termchar (§6.2 step 2)
```

Verify with `*IDN?` in NI MAX before launching the client; a failed `viOpen` or `*IDN?`
shows verbatim in the page's error band via `LastError()`.
