# Inspection View: the inspection collector

*Every `inspect_object` verdict, kept per component until the operator clears
it, shown as a statistics tree and saved as CSV on request. Code:
`core/Inspection.{h,cpp}` (samples, statistics, CSV - pure),
`Workflow::QueueInspection` / `TakeInspections`, `App::insp_` (drained in
`App::Update`), `ui/windows/WndInspection.cpp` (`App::DrawInspectionWindow`).*

---

## 1. The flow

1. `inspect_object` (`command/CommandsDatum.cpp`, `docs/datum-reference-design.md`
   §5a) posts one `InspectionSample` per VERDICT - pass, reject, object not
   found, no live datum - through `Workflow::QueueInspection`. A machine
   failure (Stop, a refused move, a camera error) is not a verdict and posts
   nothing. The step's ctx keys are unchanged; the queue is the result channel
   (never `Status.ctx`, `docs/command-layer.md`).
2. `App::Update` drains `TakeInspections` into `insp_`, an
   `InspectionCollector` - UI thread only, like `ProductSession`.
3. The window draws `insp_`. Nothing in it judges, moves or edits a step.

## 2. Lifetime: until Clear

The collection is memory only and lives until the operator presses **Clear**
(two clicks within 3 s - no modal, a run may be waiting on a prompt window),
or the app closes. It survives a page switch, a sign-out, a product load and
program starts: it is the operator's tally, not a part's state. **Save** writes
it; Clear drops it.

The window opens itself when a collection STARTS (a sample into an empty
collector). Closed mid-collection it stays closed until the next Clear - a
window that pops back after every part is a window people stop closing. It is
also on View > Inspection View (`kRoleInspection`, Operator).

## 3. The tree

One row per component (the step's `ctx_prefix`), in first-seen order:
the last verdict (PASS / FAIL, or "no tol" when the step judges nothing),
yield (passed / runs - a run with no tolerance or nothing found does not
pass), how many were not found, and n. It opens onto one row per metric:

| Metric | |
|---|---|
| x, y (local) | the datum-local position |
| dX, dY | from the nominal, two-sided tolerance `tol_x_mm` / `tol_y_mm` |
| dR | distance from the nominal, one-sided `tol_r_mm` |
| P | only when the step reports it |
| score | the detection score |

Columns: last value (green / red against its tolerance), mean, min, max,
sample standard deviation, range, n, **Cpk**. Statistics use FOUND results only.
Cpk is computed only when the metric has a tolerance, n >= 2 and the spread is
not zero: `min(tol - mean, mean + tol) / 3s` two-sided, `(tol - mean) / 3s` for
dR (Cpu). It uses the LATEST result's tolerance - an edited step judges from
then on. Colours: >= 1.33 green, >= 1.00 amber, else red.

Under the tree, the selected component's results, newest first.

## 4. Saving

- **Save** (a typed name, made file-safe by `InspectionFileName`) writes
  `<name>.csv` (every result, one row each) and `<name>_summary.csv` (per
  component: counts and yield; per metric: the statistics) into
  `<product data>/inspection/`, or `<data root>/inspection/` with no product.
  An existing name is REFUSED - a name is the record of one collection.
- **Copy CSV** puts every result on the clipboard (no OS file dialog: it would
  block the UI thread, and with it any prompt a run waits on).
- **Open folder** (`kRoleData`, it leaves the application) opens the folder.

## 5. Known limits

- No persistence between sessions other than Save.
- P is the picture rotation from the taught orientation, not corrected by the
  datum, and not judged (§5a of the datum doc).
