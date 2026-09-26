"""Offline labeler for the XCAN dataset (docs/xcan-dataset.md).

The scan record never says whether it stalled; that label is derived here, from the
chain of scans that aligned one part. Two tables come out, one per ML goal:

  scans.csv    one row per scan: its chain, the stall / peak / recovery label
  samples.csv  one row per sample: derived features (dv, dmove, running best, noise)
               plus the targets: the chain's true peak pose relative to this sample
               (goal 1, faster peak finding) and climb_lobe (goal 2, stall detection)

Labelling is automatic. A person corrects a label through data/xcan/overrides.csv
(scan_id,label,note), normally from the review window; an override always wins and is
marked label_source=manual. The export also COPIES every raw record into
xcan_labeled/<label>/ (the raw day folders are never moved).

  python xcan_label.py [ui]          review window (xcan_label_ui.py)
  python xcan_label.py label         export without a window
  python xcan_label.py plot <scan_id> [--chain] [--save out.png]
  common: [--data <dir>] [--out <dir>] [--stall-ratio 0.8] [--min-signal 1e-8]

The data folder is data/xcan in the working directory if there is one, else the newest
out/build/<preset>/data/xcan in this repo; --data overrides. Output goes to
xcan_labeled/ beside it.
Everything written is derived: delete the output folder and rerun at any time.
"""

import argparse
import csv
import glob
import json
import math
import os
import shutil
import statistics
import sys
from datetime import datetime, timedelta

AXES = ("X", "Y", "Z", "U", "V", "W")
LIN = ("X", "Y", "Z")  # mm in the record, um in the output
LABELS = ("stall", "peak", "recovery", "single", "no_signal", "excluded")


# ---------------------------------------------------------------- loading

def parse_utc(s):
    return datetime.fromisoformat(s.replace("Z", "+00:00")) if s else None


def load_scan(path):
    lines = []
    with open(path, encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                lines.append(json.loads(ln))
    if not lines or lines[0].get("type") != "header":
        return None
    header = lines[0]
    result = lines[-1] if lines[-1].get("type") == "result" else {"status": "truncated"}
    samples = [l for l in lines[1:] if l.get("type") == "sample"]
    start = parse_utc(header.get("utc"))
    dur = result.get("duration_s")
    if dur is None:
        dur = samples[-1]["t"] if samples else 0.0
    return {
        "id": header["scan_id"],
        "file": os.path.relpath(path),
        "path": os.path.abspath(path),
        "h": header,
        "r": result,
        "s": samples,
        "start": start,
        "end": start + timedelta(seconds=dur) if start else None,
    }


def load_all(data_dir):
    scans = {}
    for path in sorted(glob.glob(os.path.join(data_dir, "*", "2*.jsonl"))):
        try:
            sc = load_scan(path)
        except (OSError, json.JSONDecodeError) as e:
            print(f"skip {path}: {e}", file=sys.stderr)
            continue
        if sc:
            scans[sc["id"]] = sc
    return scans


def load_overrides(data_dir):
    path = os.path.join(data_dir, "overrides.csv")
    out = {}
    if not os.path.exists(path):
        return out
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            sid = (row.get("scan_id") or "").strip()
            lab = (row.get("label") or "").strip()
            if not sid:
                continue
            if lab not in LABELS:
                print(f"overrides.csv: {sid}: unknown label '{lab}' (use {', '.join(LABELS)})",
                      file=sys.stderr)
                continue
            out[sid] = (lab, (row.get("note") or "").strip())
    return out


# ---------------------------------------------------------------- chaining

def ctx(sc, key):
    v = (sc["h"].get("context") or {}).get(key)
    return None if v in (None, "") else v


def build_chains(scans, max_gap_s):
    """Rule 1: a wrapper's passes share context.group (keyed per device).
    Rule 2: otherwise follow prev_scan_id when device, serial, product and topic all
    match and the gap between the two scans is at most max_gap_s."""
    parent = {sid: sid for sid in scans}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    by_group = {}
    for sid, sc in scans.items():
        g = ctx(sc, "group")
        if g is not None:
            by_group.setdefault((g, sc["h"].get("device")), []).append(sid)
    for members in by_group.values():
        for m in members[1:]:
            union(members[0], m)

    same = ("device", "serial", "product", "topic")
    for sid, sc in scans.items():
        prev = scans.get(sc["h"].get("prev_scan_id"))
        if prev is None or ctx(sc, "group") is not None or ctx(prev, "group") is not None:
            continue
        if any(sc["h"].get(k) != prev["h"].get(k) for k in same):
            continue
        if sc["start"] and prev["end"] and (sc["start"] - prev["end"]).total_seconds() > max_gap_s:
            continue
        union(prev["id"], sid)

    chains = {}
    for sid in scans:
        chains.setdefault(find(sid), []).append(sid)
    out = {}
    for members in chains.values():
        members.sort(key=lambda s: (scans[s]["start"] or datetime.min, s))
        out[members[0]] = members
    return out


# ---------------------------------------------------------------- labelling

def label_chain(chain_id, members, scans, ratio, min_signal):
    ok = [s for s in members if scans[s]["r"].get("status") == "ok"
          and scans[s]["r"].get("final") is not None]
    chain_best = max((scans[s]["r"]["final"] for s in ok), default=None)
    peak_scan = max(ok, key=lambda s: scans[s]["r"]["final"]) if ok else None

    rows = {}
    for pos, sid in enumerate(members):
        sc = scans[sid]
        final = sc["r"].get("final")
        row = {"chain_id": chain_id, "chain_pos": pos + 1, "chain_len": len(members),
               "chain_best": chain_best, "peak_scan": peak_scan,
               "frac_of_best": None, "recovered_by": None, "reason": ""}
        if sid not in ok:
            row["label"], row["reason"] = "excluded", f"status={sc['r'].get('status')}"
        elif chain_best is None or chain_best < min_signal:
            row["label"], row["reason"] = "no_signal", f"chain_best below {min_signal:g}"
        elif len(ok) < 2:
            row["label"], row["reason"] = "single", "no second scan to compare against"
        else:
            row["frac_of_best"] = final / chain_best
            row["label"] = "stall" if final < ratio * chain_best else "peak"
        rows[sid] = row

    # The later scan that reached chain_best is the recovery of every stall before it.
    stalls = [s for s in members if rows[s]["label"] == "stall"]
    if peak_scan and stalls and rows[peak_scan]["label"] == "peak":
        earlier = [s for s in stalls if members.index(s) < members.index(peak_scan)]
        if earlier:
            rows[peak_scan]["label"] = "recovery"
            for s in earlier:
                rows[s]["recovered_by"] = peak_scan
    return rows


def best_seq(sc):
    best, seq = None, None
    for smp in sc["s"]:
        v = smp.get("v")
        if v is not None and (best is None or v > best):
            best, seq = v, smp["seq"]
    return seq


def pose_delta(a, b, ax):
    """b - a for one axis: um for X/Y/Z, deg for U/V/W; None when either is missing."""
    if not a or not b or a.get(ax) is None or b.get(ax) is None:
        return None
    d = b[ax] - a[ax]
    return d * 1000.0 if ax in LIN else d


def unit(ax):
    return "um" if ax in LIN else "deg"


def sample_rows(sc, srow, scans):
    label = srow["label"]
    chain_peak = scans[srow["peak_scan"]]["r"].get("best_pose") if srow["peak_scan"] else None
    scan_peak = sc["r"].get("best_pose")
    bseq = best_seq(sc)
    out, prev_v, prev_pose, run_best = [], None, None, None
    ss = sc["s"]
    for i, smp in enumerate(ss):
        v, pose = smp.get("v"), smp.get("pose") or {}
        raw = [p[1] for p in (smp.get("raw") or []) if len(p) > 1 and p[1] is not None]
        run_best = v if run_best is None or (v is not None and v > run_best) else run_best
        dmove = None
        if prev_pose:
            parts = [pose_delta(prev_pose, pose, a) for a in LIN]
            if all(p is not None for p in parts):
                dmove = math.sqrt(sum(p * p for p in parts))
        nxt = ss[i + 1].get("pose") if i + 1 < len(ss) else None

        if label == "stall":
            climb = 1 if bseq is not None and smp["seq"] <= bseq else None
        elif label in ("peak", "recovery"):
            climb = 0
        else:
            climb = None

        row = {
            "scan_id": sc["id"], "chain_id": srow["chain_id"], "scan_label": label,
            "seq": smp.get("seq"), "t": smp.get("t"), "phase": smp.get("phase"),
            "axis": smp.get("axis"), "dir": smp.get("dir"), "step_um": smp.get("step_um"),
            "note": smp.get("note"), "v": v,
            "dv": (v - prev_v) if v is not None and prev_v is not None else None,
            "run_best": run_best,
            "dmove_um": dmove,
            "raw_n": len(raw),
            "raw_sigma": statistics.pstdev(raw) if len(raw) > 1 else None,
            "fresh_req": smp.get("fresh_req"), "fresh_got": smp.get("fresh_got"),
            "wait_ms": smp.get("wait_ms"), "settle_timeout": smp.get("settle_timeout"),
            "move_ms": smp.get("move_ms"),
            "climb_lobe": climb,
        }
        for ax in AXES:
            row[f"pose_{ax}"] = pose.get(ax)
        for ax in AXES:
            row[f"next_d{ax}_{unit(ax)}"] = pose_delta(pose, nxt, ax)
        for ax in AXES:
            row[f"to_scan_peak_{ax}_{unit(ax)}"] = pose_delta(pose, scan_peak, ax)
        for ax in AXES:
            row[f"to_chain_peak_{ax}_{unit(ax)}"] = pose_delta(pose, chain_peak, ax)
        out.append(row)
        prev_v, prev_pose = v, pose
    return out


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def save_overrides(data_dir, overrides):
    """The one writer of overrides.csv (the CLI only reads it; the UI edits it)."""
    path = os.path.join(data_dir, "overrides.csv")
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["scan_id", "label", "note"])
        for sid in sorted(overrides):
            w.writerow([sid, overrides[sid][0], overrides[sid][1]])
    os.replace(tmp, path)


def compute(scans, overrides, ratio, min_signal, max_gap_s):
    """Automatic labels for every scan, then the overrides on top.
    Returns (chains, labels, scan_rows); labels[sid] keeps auto_label beside label."""
    chains = build_chains(scans, max_gap_s)
    labels = {}
    for cid, members in chains.items():
        labels.update(label_chain(cid, members, scans, ratio, min_signal))

    scan_rows = []
    for sid in sorted(scans, key=lambda s: (scans[s]["start"] or datetime.min, s)):
        sc, row = scans[sid], labels[sid]
        row["auto_label"] = row["label"]
        row["label_source"] = "auto"
        row["override_note"] = ""
        if sid in overrides:
            row["label"], row["override_note"] = overrides[sid]
            row["label_source"] = "manual"
        h, r = sc["h"], sc["r"]
        scan_rows.append({
            "scan_id": sid, "chain_id": row["chain_id"], "chain_pos": row["chain_pos"],
            "chain_len": row["chain_len"], "utc": h.get("utc"), "device": h.get("device"),
            "product": h.get("product"), "serial": h.get("serial"), "topic": h.get("topic"),
            "caller": ctx(sc, "caller"), "group": ctx(sc, "group"), "pass": ctx(sc, "pass"),
            "stage": ctx(sc, "stage"), "lens": ctx(sc, "lens"),
            "status": r.get("status"), "baseline": r.get("baseline"), "final": r.get("final"),
            "best_v": r.get("best_v"), "gain": r.get("gain"), "chain_best": row["chain_best"],
            "frac_of_best": row["frac_of_best"], "label": row["label"],
            "auto_label": row["auto_label"],
            "label_source": row["label_source"], "recovered_by": row["recovered_by"],
            "reason": row["reason"], "override_note": row["override_note"],
            "n_samples": r.get("n_samples", len(sc["s"])), "duration_s": r.get("duration_s"),
            "settle_timeouts": sum(1 for s in sc["s"] if s.get("settle_timeout")),
            "file": sc["file"],
        })
    return chains, labels, scan_rows


def export(scans, labels, scan_rows, out):
    """scans.csv + samples.csv, and a COPY of every raw record under <out>/<label>/.
    The raw day folders are never touched: a label can change when a later retry joins
    the chain, when the ratio is retuned or when someone overrides it, and FireDog keeps
    appending to data/xcan/<day>/. The label folders are rebuilt from scratch each time."""
    sample_out = []
    for row in scan_rows:
        sid = row["scan_id"]
        if row["label"] != "excluded" and row["status"] == "ok":
            sample_out.extend(sample_rows(scans[sid], labels[sid], scans))

    os.makedirs(out, exist_ok=True)
    write_csv(os.path.join(out, "scans.csv"), scan_rows)
    write_csv(os.path.join(out, "samples.csv"), sample_out)

    for lab in LABELS:
        d = os.path.join(out, lab)
        if os.path.isdir(d):
            shutil.rmtree(d)
    for row in scan_rows:
        d = os.path.join(out, row["label"])
        os.makedirs(d, exist_ok=True)
        shutil.copy2(scans[row["scan_id"]]["path"], d)
    return len(sample_out)


def summary(scan_rows):
    counts = {}
    for r in scan_rows:
        counts[r["label"]] = counts.get(r["label"], 0) + 1
    return ", ".join(f"{k} {counts[k]}" for k in LABELS if k in counts)


def cmd_label(a):
    scans = load_all(a.data)
    if not scans:
        sys.exit(f"no scans under {a.data}")
    overrides = load_overrides(a.data)
    for sid in sorted(set(overrides) - set(scans)):
        print(f"overrides.csv: {sid} is not in the dataset", file=sys.stderr)
    chains, labels, scan_rows = compute(scans, overrides, a.stall_ratio, a.min_signal,
                                        a.max_gap_s)
    n = export(scans, labels, scan_rows, a.out)
    print(f"{len(scan_rows)} scans in {len(chains)} chains, {n} samples -> {a.out}")
    print("  " + summary(scan_rows))
    if overrides:
        print(f"  {len(set(overrides) & set(scans))} manual overrides applied")


# ---------------------------------------------------------------- plotting

def draw_scans(fig, scans, ids, lab, title_id, chain):
    """Signal on top (dotted line at each scan's best sample), X/Y/Z below in um
    relative to each scan's start. Shared by `plot` and the UI."""
    fig.clear()
    ax_v = fig.add_subplot(2, 1, 1)
    ax_p = fig.add_subplot(2, 1, 2, sharex=ax_v)
    offset = 0
    for sid in ids:
        sc = scans[sid]
        seq = [offset + s["seq"] for s in sc["s"]]
        v = [s.get("v") for s in sc["s"]]
        ax_v.plot(seq, v, ".-", lw=1 if sid != title_id else 1.8,
                  label=f"{sid} [{lab.get(sid, '?')}]")
        b = best_seq(sc)
        if b is not None:
            ax_v.axvline(offset + b, ls=":", c="gray")
        p0 = sc["s"][0]["pose"] if sc["s"] else {}
        for ax in LIN:
            ax_p.plot(seq, [pose_delta(p0, s.get("pose"), ax) for s in sc["s"]],
                      label=ax if sid == ids[0] else None)
        offset = seq[-1] if seq else offset
    ax_v.set_ylabel(scans[ids[0]]["h"].get("topic", "v"))
    ax_v.legend(fontsize=8)
    ax_v.grid(alpha=0.3)
    ax_p.set_ylabel("pose - scan start (um)")
    ax_p.set_xlabel("sample (chained)" if chain else "sample")
    ax_p.legend(fontsize=8)
    ax_p.grid(alpha=0.3)
    fig.suptitle(title_id + (" chain" if chain else ""), fontsize=10)
    fig.tight_layout()


def cmd_plot(a):
    import matplotlib.pyplot as plt

    scans = load_all(a.data)
    if a.scan_id not in scans:
        sys.exit(f"{a.scan_id} not found under {a.data}")
    ids = [a.scan_id]
    if a.chain:
        chains = build_chains(scans, a.max_gap_s)
        ids = next(m for m in chains.values() if a.scan_id in m)

    lab = {}
    lpath = os.path.join(a.out, "scans.csv")
    if os.path.exists(lpath):
        with open(lpath, newline="", encoding="utf-8") as f:
            lab = {r["scan_id"]: r["label"] for r in csv.DictReader(f)}

    fig = plt.figure(figsize=(11, 7))
    draw_scans(fig, scans, ids, lab, a.scan_id, a.chain)
    if a.save:
        fig.savefig(a.save, dpi=120)
        print(f"saved {a.save}")
    else:
        plt.show()


def find_data_dir():
    """data/xcan in the working directory, else the newest one under the repo's
    out/build/<preset>/ (FireDog writes beside the exe it ran as)."""
    here = os.path.join("data", "xcan")
    if os.path.isdir(here):
        return here
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    found = [d for d in glob.glob(os.path.join(repo, "out", "build", "*", "data", "xcan"))
             if os.path.isdir(d)]
    if not found:
        sys.exit(f"no data/xcan in {os.getcwd()} or under {repo}\\out\\build\\*; pass --data")

    def newest(d):
        return max((os.path.getmtime(f) for f in glob.glob(os.path.join(d, "*", "*.jsonl"))),
                   default=os.path.getmtime(d))
    found.sort(key=newest, reverse=True)
    if len(found) > 1:
        print("several datasets, using the newest (pass --data to pick):", file=sys.stderr)
        for d in found:
            print(f"  {d}", file=sys.stderr)
    return found[0]


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("cmd", nargs="?", choices=("ui", "label", "plot"), default="ui",
                   help="ui (default) opens the review window; label writes the export "
                        "without one; plot draws one scan")
    p.add_argument("scan_id", nargs="?", help="plot: the scan to draw")
    p.add_argument("--data", help="the xcan folder (default: found automatically)")
    p.add_argument("--out", help="output folder (default: xcan_labeled beside --data)")
    p.add_argument("--max-gap-s", type=float, default=300.0,
                   help="longest pause between two scans of one chain (rule 2)")
    p.add_argument("--stall-ratio", type=float, default=0.8,
                   help="label: final below this x chain_best is a stall")
    p.add_argument("--min-signal", type=float, default=1e-8,
                   help="label: chain_best below this (topic units, A) is no_signal")
    p.add_argument("--chain", action="store_true", help="plot: the whole chain")
    p.add_argument("--save", help="plot: write a PNG instead of opening a window")

    a = p.parse_args()
    a.data = a.data or find_data_dir()
    a.out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.data)), "xcan_labeled")
    print(f"data: {os.path.abspath(a.data)}")
    if a.cmd == "plot":
        if not a.scan_id:
            p.error("plot needs a scan_id")
        cmd_plot(a)
    elif a.cmd == "ui":
        import xcan_label_ui
        xcan_label_ui.run(a)
    else:
        cmd_label(a)


if __name__ == "__main__":
    main()
