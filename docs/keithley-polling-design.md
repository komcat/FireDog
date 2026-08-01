# Keithley 6482 polling subsystem

Dual-channel picoammeter + voltage source. This is the **feedback signal source** for
active alignment: the XCAN / raster scripts read `keithley/ch1` off the DataBus, so the
poll loop's sample cadence directly sets the scan cycle time.

Files: `src/hardware/instruments/Keithley6482.{h,cpp}` (layer 1),
`src/workflow/Workflow.cpp` (layer 2 bring-up + command surface),
`src/App.cpp` Keithley page (layer 3).

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
        │  builds Keithley6482 -> HardwareManager::SetKeithley (sole owner)
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

### Class surface (`Keithley6482.h`)

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
| `visa_resource` | `GPIB0::1::INSTR` | `viOpen` target |
| `enabled_channels` | `[1,2]` | which channels get a DataBus stream |
| `measurement_nplc` | 5.0 (repo file: **0.1**) | integration time → **the sample rate** |
| `auto_range` / `fixed_range` | true / 0.0 | `:RANG:AUTO ON` or `:RANG <A>` |
| `poll_interval_ms` | 100 (repo file: **0**) | 0 = "as fast as the instrument allows" |
| `bias_voltage` | 1.0 | applied to ch1 after every connect |

---

## 2. SCPI command reference

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
