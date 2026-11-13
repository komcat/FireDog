# Position records — a named pose kept on every unit run

## 1. What it is for

On one product, the pose a device ends at after a given step (the peak after an
`xcan`, the end of a `climb_raster`, where an operator finished a manual adjust)
clusters from unit to unit. That cluster is the known place to start the next
first-light / peak search. Nothing kept it: `fd.scanlog` (`data/xcan/`) is paused by
default, machine-wide, Lua-only, and its index carries no pose; a DUT record holds
one part.

So a recipe says, explicitly, "keep this pose": a **`record_position`** step placed
right after the step that left the device where it matters. Every unit run that
passes it appends one record. The **Position Records** window (View menu) shows the
history. Neither predicts nor moves anything — the median the window shows is a
candidate for a person to judge.

## 2. The step

`record_position` (`command/CommandsRecords.cpp`, category DUT):

| param | | |
|---|---|---|
| `device` | required | the motion device whose pose is recorded |
| `name` | required | the history it goes into — **the name says which device and which step** (`lens1_peak`, `fiber_first_light`); 1–64 of `A-Z a-z 0-9 - _ .` |
| `record_without_dut` | default off | record even with no DUT open |

- Reads the pose with `read_position`'s freshness rule (`RequestFastPoll`, 120 ms,
  `Positions()`) — every axis the controller reports, so a hexapod records X Y Z U V W.
- **No product loaded → fails.** There is nowhere to keep it.
- **No DUT open → skipped** (logged, the step succeeds) unless `record_without_dut`.
  A test run without a part must neither fail nor pollute the history.
- **A write failure fails the step.** The history was asked for; a silent gap is
  worse than a stopped run.
- Not a mover: no `teach`, no `parallelBy`, and **not in the dry-run allowlist** — a
  dry or teach run never writes a record.
- Lua: `fd.record_position{device=..., name=...}`, generated from the registry.

## 3. The file

```
<data root>/products/<product>/data/position_records/<name>.jsonl
```

`Product::DataDir()` is production output (`DataPath`, never beside the exe), reached
from the worker through `DutSession::ProductDataDir()`. One file per name: a product
aligning two parts keeps two histories, and the window reads one name without
filtering a shared file. One JSON line per record, appended under a process-wide
mutex:

```json
{"t":1794470400,"ts":"2026-11-12T14:20:05","name":"lens1_peak","device":"hexapod-1",
 "pose":{"X":0.1234,"Y":-0.0567,"Z":3.2101,"U":0.01,"V":-0.02,"W":0.0},
 "product":"PRD-A","product_uid":"...","serial":"SN0001",
 "operator":{"id":"1234","name":"..."}}
```

`t` is unix seconds (UTC) — what the time filters use; `ts` is local time for reading.
`serial` is `null` when recorded without a DUT, `operator` `null` when nobody is
signed in.

## 4. The window

`ui/windows/WndPositionRecords.cpp`, **Technician** (`kRolePosRecords`, like View
Data), closed at sign-out, not persisted. View only.

- **Name** picks the file; **Load last** bounds how many newest lines are read (the
  file is read from its end, so a long history costs only its tail).
- **Range**: all loaded, last week, last month, or the last N units.
- **Statistics** per axis: n, median, mean, std dev, min, max, range; *Copy medians*.
- **Histograms** per axis with the median (and the selected record) marked.
- **Trend** per axis against time (local) or unit index, the median as a line.
- **X–Y** scatter with the median and the newest record marked.
- **Records** newest first: time, serial, operator, device, axes. Selecting one marks
  it in the charts.
- X Y Z by default; *Show U V W* adds the rotations. A name whose records come from
  more than one device is flagged — one name should be one device.

The window re-reads when the product's folder changes, when `PositionRecordsSeq()`
moved (any append), or on Reload — never per frame.

## 5. Where the statistics live

`core/PositionRecords` holds the file IO **and** `FilterPositionRecords` /
`ComputeAxisStats`. A later step that drives to a predicted start point must call
these rather than derive its own median — the window and the move must agree about
what "the known location" is.
