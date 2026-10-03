# XCAN dataset: the silent per-scan record

An XCAN scan can write one self-contained, machine-readable record. The `xcan` step,
every pass of `xcan_to_target` / `xcan_two_lens`, the Run page presets and the Motion
page all run `hexapod_align.lua`, and that script logs through `fd.scanlog`.

**Recording is PAUSED (2026-10): `align_scanlog` defaults to 0.** Set the ctx param
`align_scanlog` to 1 (a `ctx_set` step before the scan) to record again; change the
default in `hexapod_align.lua` to turn it back on everywhere. With it off, `begin` is
never called, so `sample` / `finish` do nothing and no file is written.

The goal is a training set for two models:

1. **Faster peak finding.** Predict the peak, or the next move, from the samples seen
   so far (a surrogate model, Bayesian optimisation, or a learned step policy).
2. **Stall / side-lobe detection.** Recognise early that a walk is climbing a side lobe
   (the ~20 % stalls at ~6.7/44/78/127 µA, see `xcan-scan-design.md` §6).

The record keeps what a model needs and cannot get back afterwards. It leaves out
anything that can be recomputed: `dv`, `dmove` and the running best are derived offline.

## Files

```
data/xcan/YYYY-MM-DD/<scan_id>.jsonl   header line, one line per sample, result line
data/xcan/YYYY-MM-DD/index.jsonl       one summary line per scan (filter without opening every file)
```

- **Location.** `data/` sits beside `logs/`, in the process working directory.
- **Day folder.** Named for the local calendar day, the same day as the daily log.
- **`scan_id`.** `<local YYYYMMDD_HHMMSS_mmm>_<device>`.
- **Where records go.** They are written whether or not a DUT is open, and they are
  machine data, not product data. There is no pruning. A record is about 40–60 KB.

## Record schema

### `header` (first line)

| field | meaning |
|---|---|
| `type` | `"header"` |
| `scan_id`, `prev_scan_id` | `prev_scan_id` is the previous scan on the same device in this process, or `null`. It chains a stall to its retry. |
| `utc`, `host` | Start time as UTC ISO-8601 in ms, and `COMPUTERNAME`. |
| `kind`, `script` | `kind` is `"xcan"`. `script` is the script path that ran. |
| `device`, `topic`, `axes` | The hexapod, the maximised stream, and the descent order (e.g. `"Z,Y,X"`). |
| `params` | Every resolved `P` value from `hexapod_align.lua`: ladder, zones, adaptive, settle, fresh samples, consec, drop %, travel, velocity, tol, trims, exit threshold. This is the policy that produced the trajectory. |
| `context` | Tags set by a wrapper as `scanlog_*` ctx keys: `caller`, `group`, `pass`, `stage`, `target_ua`, and `lens` (two-lens only). Empty for a plain `xcan`. |
| `product`, `product_uid`, `dut_open`, `serial`, `operator` | The part and the person. `serial` is `null` without an open DUT. `operator` is `{id, name, level}` or `null`. |
| `start_pose` | Every existing `motion/<device>/{X,Y,Z,U,V,W}` stream, in mm (and deg for the rotations as the controller reports them). |
| `smu` | `{model, nplc, auto_range, fixed_range_a, bias_v, output_on}`, or `null` when the SMU is not connected. |
| `aux` | Latest `ch1_a`, `ch2_a` (A), `laser_ma` (mA) and `tec_c` (°C). Each is `null` when its stream does not exist. Used to normalise for source drift. |

### `sample` (one line per `measure()`)

| field | meaning |
|---|---|
| `seq`, `t` | 1-based order, and seconds since the header (steady clock). |
| `phase`, `axis`, `dir`, `step_um`, `note` | The algorithm's state and decision. `phase` is one of `base\|scan\|check\|final`. `note` is one of `baseline\|start\|better\|worse(n/N)\|drop-break\|return-to-best\|axis-done\|recovered\|final\|final@gpeak`. These are the action labels. |
| `v` | The value the algorithm used, in topic units (A). |
| `pose` | All six axes, read right after the sample. This is the *x* of *f(x)*. |
| `raw` | `[[t_dev, v], ...]`: every stream reading published after the measurement began, plus the one just before it, capped at 64. `t_dev` is the publisher's clock. Gives per-point noise σ and shows the reading that integrated during the move. |
| `n0` | The topic's publish count when the measurement began. |
| `fresh_req`, `fresh_got` | Fresh readings demanded and obtained. |
| `wait_ms`, `settle_timeout` | Measurement wall time, and `true` when the `settle_ms` ceiling cut the wait short (the sample may be stale). |
| `move_ms` | Motion time since the previous sample. A recovery sums several moves. |

### `result` (last line, always written)

| field | meaning |
|---|---|
| `status` | See below. |
| `error` | The script's error text, when there is one. |
| `baseline`, `final`, `gain`, `exit_hit`, `exit_threshold` | The outcome, from the script (status `ok` only). |
| `passes_run`, `passes_skipped` | Ladder rungs run and rungs skipped by the adaptive gate (status `ok` only). |
| `best_v`, `best_pose` | The highest sample and its full pose, tracked in C++ over the samples. This is the peak-location target. |
| `final_pose` | Pose when the record closed. |
| `n_samples`, `duration_s` | Cost. |

`status` takes one of these values:

- `ok`: the script called `finish`.
- `error`: a script error, e.g. baseline below `min_signal`.
- `stopped`: the operator pressed Stop, or the safety stop fired.
- `incomplete`: the script ended without finishing, or a second `begin` arrived.

## The stall label is computed OFFLINE

A scan does not know whether it stalled. Its peak might be the real one, and a guess
written into the record would be a label nobody can trust. Derive it when building the
training set:

1. Chain the scans of one alignment. Use `context.group` for wrapper passes. Otherwise
   follow `prev_scan_id` with the same `serial` + `device`, within a short time gap.
2. `chain_best` is the maximum `final` over the chain. A scan is a **stall** when its
   `final` is well below `chain_best` (e.g. under 0.8×). The later scan that reached
   `chain_best` is the recovery.
3. Samples before a stall's `best_pose` are the "climbing a lobe" sequence.

## Labelling: `tools/xcan/xcan_label.py`

The steps above are automated by `tools/xcan/xcan_label.py`, a Python script (the standard
library only; `plot` also needs matplotlib). It can be run from any folder:

```
python xcan_label.py [ui]      the review window (xcan_label_ui.py, tkinter + matplotlib)
python xcan_label.py label     the export without a window
python xcan_label.py plot <scan_id> [--chain] [--save out.png]
common: [--stall-ratio 0.8] [--min-signal 1e-8] [--max-gap-s 300] [--data] [--out]
```

**Labelling is automatic.** Every scan gets its label from the chain rules as soon as it
is loaded. The window is for reviewing those labels and overruling the wrong ones:

- **Scan list.** Colour shows the label; a yellow row has been overridden.
- **Plot.** Signal on top, X/Y/Z below. By default it shows the whole chain.
- **Keys.**
  - `1`–`6` set a label, and the selection then moves to the next scan.
  - `0` returns the scan to its automatic label.
  - `C` toggles the chain plot.
  - `F5` reloads from disk, because FireDog keeps writing while the window is open.
- **Parameters.** The top bar holds the stall ratio, min signal and max gap.
  **Relabel** reruns the rules with those values.
- **Export** writes the output described below.

It reads `data/xcan` in the working directory if there is one, else FireDog's default
`D:\ProgramData\FireDog\data\xcan` (the data root, `docs/machine-root-design.md`). Otherwise it reads the
newest `out/build/<preset>/data/xcan` in the repo (where builds before the machine root
wrote). `--data` overrides all three.

It writes to `xcan_labeled/`, next to the data folder. Everything there is derived, so you can delete the
folder and rerun at any time.

- **`xcan_labeled/<label>/`** holds a COPY of each raw record, sorted by its current
  label. The raw day folders are never moved, for three reasons:
  - A label is not final. A later retry that joins the chain can turn `single` into
    `stall`, retuning the ratio changes it, and so does an override.
  - FireDog keeps appending to `data/xcan/<day>/` and its `index.jsonl`.
  - The raw data is the only copy that cannot be regenerated.

  Every export rebuilds the label folders from scratch.

- **`scans.csv`**: one row per scan, with `chain_id`, `chain_best`, `frac_of_best`, and
  `label`. `label` is one of:
  - `stall`: `final` is below `stall-ratio` × `chain_best`.
  - `recovery`: the later scan that reached `chain_best` after a stall.
  - `peak`: the scan reached `chain_best`, with no earlier stall.
  - `single`: no second scan in the chain to compare against, so the scan cannot be
    judged.
  - `no_signal`: `chain_best` is below `--min-signal`. This is a dark or noise chain,
    and a ratio of noise is not a label.
  - `excluded`: the status is not `ok`.

  A stall also gets `recovered_by`, the scan that recovered it.
- **`samples.csv`**: one row per sample of every `ok` scan that is not `excluded`.
  - **Derived features:** `dv`, `dmove_um`, `run_best`, and `raw_n` / `raw_sigma`.
  - **Goal 1 targets:**
    - `next_d*`: the move the policy took.
    - `to_scan_peak_*`: the offset to this scan's own `best_pose`.
    - `to_chain_peak_*`: the offset to the chain's true peak, which is the `best_pose`
      of the scan with the highest `final`. For a stall, this is the recovery's peak.

    X/Y/Z are in µm; U/V/W are in degrees.
  - **Goal 2 target, `climb_lobe`:**
    - `1` for a stall's samples up to its best sample.
    - `0` for every sample of a `peak` or `recovery` scan.
    - Empty when the scan is not labelled.

**Manual correction.** Put `data/xcan/overrides.csv` (columns `scan_id,label,note`) next
to the day folders. An override always wins over the automatic label and is marked
`label_source=manual`. It lives with the data, not with the output, because it is the
only human input and must survive a rerun. Use `plot --chain` to look at a chain before
you overrule it.

## Loading

```python
import json, glob, pandas as pd
recs = []
for f in glob.glob("data/xcan/*/2*.jsonl"):
    lines = [json.loads(l) for l in open(f)]
    h, s, r = lines[0], lines[1:-1], lines[-1]
    recs.append((h, pd.json_normalize(s), r))
index = pd.concat(pd.read_json(f, lines=True) for f in glob.glob("data/xcan/*/index.jsonl"))
```

## Rules

- **One writer: `fd.scanlog` (`core/ScanLog`, bound in `RunLuaFile`).** Never add a
  second dataset writer in a command or a page. Two writers would describe one scan
  two ways.
- **No disk I/O inside the walk.** Samples are buffered and the file is written once, at
  `finish` or when the script ends (temp file then rename). Timing must not change
  because the scan is being recorded.
- **Silent.** A write failure is one `scanlog` warning in the log and never fails a scan.
- **C++ adds what the script cannot see:** pose, raw readings, DUT, operator, SMU and
  aux. The script says only what it decided. Another scan (raster, climb) joins the
  dataset by calling `fd.scanlog.begin/sample/finish` with its own `kind`.
- **A wrapper tags its passes through `scanlog_*` ctx keys and clears them on every
  exit**, errors included (`pcall`, then re-raise). Otherwise a later plain `xcan` in
  the same run would be filed under the wrapper's group. A tag cleared to `""` is
  treated as absent.
