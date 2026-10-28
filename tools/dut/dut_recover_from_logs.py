"""Rebuild DUT records that a `dut_open new=true` wiped, from the FireDog logs.

Until 2026-10 `dut_open new=true` truncated an existing dut.json. The post-bake
probe recipe used it, so every part probed after bake lost its assembly record
(XCAN peaks, UV readings, the run history). The log still holds part of it:

  [dut] DUT '<serial>' open (create|reopen, ...)    session start
  [dut] DUT '<serial>' closed (<status>, N entrie(s)) session end (N = what it held)
  [recipe] Xcan_Left_Peak (align_best) = <v>, ...    -> Xcan_Left_Peak
  [recipe] Xcan_Right_Peak (align_best) = <v>, ...   -> Xcan_Right_Peak
  [recipe] post_uv_value = <v>   inside 'Probe after Bake test'
                                                     -> After Bake measurement
  [recipe]/[script] run '<name>' finished: <result>  -> $run
  [users] <id> - <name> logged in as <Level>         -> $operator / run operator

NOT recoverable: pre_uv_value / post_uv_value from the UV recipe (keithley_read
never logs its value). The uploader's sheet / kumgit backup still has those.

For each serial the record on disk is matched to the logged session that
created it (its first $open). Every logged session of that serial BEFORE it was
wiped and is rebuilt; sessions from it on are on disk, and only logged values
missing from them are added. Each added entry carries "recovered": "<log>:<line>".
A post_uv_value entry written inside a 'Probe after Bake test' run is the
after-bake reading misfiled under the UV key; it is relabeled to
"After Bake measurement" with "relabeled_from".

  python dut_recover_from_logs.py                 write merged copies to data_recovered/
  python dut_recover_from_logs.py --apply         write in place (dut.json.pre-recovery.bak kept)
  common: [--root C:/ProgramData/FireDog] [--product siphog888] [--serial S ...]
          [--data <dir>] [--logs <dir>] [--out <dir>]

Run it with FireDog closed (a DUT open in the app would be overwritten by the
app's next write). Re-running is harmless: a merged record matches its first
logged session, so nothing is added twice.
"""

import argparse
import csv
import datetime as dt
import glob
import json
import os
import re
import shutil
import sys

AFTER_BAKE_RECIPE = "Probe after Bake test"
AFTER_BAKE_KEY = "After Bake measurement"

RE_LINE = re.compile(r"^\[(\d\d:\d\d:\d\d\.\d{3})\] \[[^\]]*\] \[([^\]]+)\] (.*)$")
RE_LOGNAME = re.compile(r"firedog_(\d{4}-\d{2}-\d{2})\.log$")
RE_OPEN = re.compile(r"^DUT '(.+)' open \((\w+), ")
RE_CLOSED = re.compile(r"^DUT '(.+)' closed \((\w+), (\d+) entrie")
RE_PRODUCT = re.compile(r"^loaded '(.+)'$")
RE_LOGIN = re.compile(r"^(\S+) - (.+) logged in as (\w+)$")
RE_RUN_STARTED = re.compile(r"^run started: '(.+)'$")
RE_RUN_FINISHED = re.compile(r"^run '(.+)' finished: (\w+)")
RE_XCAN = re.compile(r"^(Xcan_(?:Left|Right)_Peak) \(align_best\) = ([-+0-9.eE]+)")
RE_AFTER_BAKE = re.compile(r"^post_uv_value = ([-+0-9.eE]+)$")


def utc_stamp(day, hms):
    """Local log time -> the record's UTC ISO stamp (2026-10-27T22:11:56.448Z)."""
    local = dt.datetime.strptime(f"{day} {hms}", "%Y-%m-%d %H:%M:%S.%f")
    u = local.astimezone(dt.timezone.utc)   # naive = this PC's local zone, DST-aware
    return u.strftime("%Y-%m-%dT%H:%M:%S.") + f"{u.microsecond // 1000:03d}Z"


def parse_stamp(s):
    return dt.datetime.strptime(s, "%Y-%m-%dT%H:%M:%S.%fZ")


def parse_logs(log_dir):
    """Walk the dated logs in order. Returns (sessions, after_bake_windows)."""
    files = []
    for p in glob.glob(os.path.join(log_dir, "firedog_*.log")):
        m = RE_LOGNAME.search(os.path.basename(p))
        if m:
            files.append((m.group(1), p))
    files.sort()

    sessions = []
    windows = []            # (start, end) of every after-bake run, UTC stamps
    cur = None              # open session
    product = None
    operator = None         # {"id","name","level"}
    run_name = None
    run_start = None

    def end_session(close_t=None, status=None, n=None):
        nonlocal cur
        if cur is not None:
            cur["close_t"], cur["status"], cur["logged_n"] = close_t, status, n
            sessions.append(cur)
            cur = None

    for day, path in files:
        src = os.path.basename(path)
        with open(path, encoding="utf-8", errors="replace") as f:
            for ln, line in enumerate(f, 1):
                m = RE_LINE.match(line.rstrip("\r\n"))
                if not m:
                    continue
                hms, cat, msg = m.groups()
                t = None

                def T():
                    nonlocal t
                    if t is None:
                        t = utc_stamp(day, hms)
                    return t

                where = f"{src}:{ln}"
                if cat == "app" and msg.endswith(" starting"):
                    end_session()               # the app went down with it open
                    product = operator = run_name = None
                elif cat == "product" and (mm := RE_PRODUCT.match(msg)):
                    product = mm.group(1)
                elif cat == "users":
                    if mm := RE_LOGIN.match(msg):
                        operator = {"id": mm.group(1), "name": mm.group(2),
                                    "level": mm.group(3).lower()}
                    elif "logged out" in msg:
                        operator = None
                elif cat == "dut" and (mm := RE_OPEN.match(msg)):
                    end_session()
                    cur = {"serial": mm.group(1), "mode": mm.group(2), "open_t": T(),
                           "open_src": where, "product": product,
                           "operator": operator, "items": []}
                elif cat == "dut" and (mm := RE_CLOSED.match(msg)):
                    if cur is not None and cur["serial"] == mm.group(1):
                        cur["close_src"] = where
                        end_session(T(), mm.group(2), int(mm.group(3)))
                elif cat == "recipe" and (mm := RE_RUN_STARTED.match(msg)):
                    run_name, run_start = mm.group(1), T()
                elif cat in ("recipe", "script") and (mm := RE_RUN_FINISHED.match(msg)):
                    if cat == "recipe":
                        if mm.group(1) == AFTER_BAKE_RECIPE and run_start:
                            windows.append((run_start, T()))
                        run_name = None
                    if cur is not None:
                        v = {"kind": cat, "name": mm.group(1), "result": mm.group(2)}
                        if operator:
                            v["operator"] = operator["id"]
                        cur["items"].append(("$run", "object", v, T(), where))
                elif cat == "recipe" and cur is not None:
                    if mm := RE_XCAN.match(msg):
                        val = to_float(mm.group(2))
                        if val is not None:
                            cur["items"].append((mm.group(1), "double", val, T(), where))
                    elif (mm := RE_AFTER_BAKE.match(msg)) and run_name == AFTER_BAKE_RECIPE:
                        val = to_float(mm.group(1))
                        if val is not None:
                            cur["items"].append((AFTER_BAKE_KEY, "double", val, T(), where))
    end_session()
    return sessions, windows


def to_float(s):
    try:
        return float(s)
    except ValueError:
        return None


def entry(key, typ, value, t, where):
    return {"seq": 0, "t": t, "key": key, "type": typ, "value": value, "recovered": where}


def near(a, b, secs):
    return abs((parse_stamp(a) - parse_stamp(b)).total_seconds()) <= secs


def in_window(t, start, end, pad=1.0):
    p = parse_stamp(t)
    return (parse_stamp(start) - dt.timedelta(seconds=pad) <= p
            <= parse_stamp(end) + dt.timedelta(seconds=pad))


def merge(serial, rec, sessions, windows, stats):
    """Return (new_record or None, report row)."""
    row = {"serial": serial, "status": "", "lost_sessions": 0, "logged_lost_entries": 0,
           "recovered": 0, "relabeled": 0, "added_to_kept": 0, "keys": "", "note": ""}
    entries = rec.get("entries", [])
    opens = [e for e in entries if e.get("key") == "$open"]
    if not opens:
        row["status"], row["note"] = "skip", "record has no $open"
        return None, row
    first_open = opens[0]["t"]
    mine = [s for s in sessions if s["serial"] == serial
            and s["product"] in (None, rec.get("product"))]
    # The session that created the record on disk; everything logged before it
    # was wiped. Without it in the log, the record's own $open is the boundary.
    match = next((s for s in mine if near(s["open_t"], first_open, 2.0)), None)
    if match is None:
        row["note"] = f"record opened {first_open} is not in the logs; boundary = that time"
    boundary = parse_stamp(match["open_t"] if match else first_open) - dt.timedelta(seconds=2)
    lost = [s for s in mine if parse_stamp(s["open_t"]) < boundary]
    kept = [s for s in mine if parse_stamp(s["open_t"]) >= boundary]

    new_entries = []
    keys = {}
    for s in lost:
        new_entries.append(entry("$open", "event", s["mode"], s["open_t"], s["open_src"]))
        if s["operator"]:
            new_entries.append(entry("$operator", "object", s["operator"], s["open_t"],
                                     s["open_src"]))
        for key, typ, val, t, where in s["items"]:
            new_entries.append(entry(key, typ, val, t, where))
            if not key.startswith("$"):
                keys[key] = keys.get(key, 0) + 1
        if s["close_t"]:
            new_entries.append(entry("$close", "event", s["status"], s["close_t"],
                                     s.get("close_src", s["open_src"])))
        row["logged_lost_entries"] += s["logged_n"] or 0
    row["lost_sessions"] = len(lost)
    row["recovered"] = len(new_entries)

    # On-disk entries: relabel the after-bake reading misfiled as post_uv_value.
    kept_entries = []
    for e in entries:
        e = dict(e)
        if e.get("key") == "post_uv_value" and any(in_window(e["t"], a, b) for a, b in windows):
            e["key"], e["relabeled_from"] = AFTER_BAKE_KEY, "post_uv_value"
            row["relabeled"] += 1
        kept_entries.append(e)

    # Kept sessions: add logged values the record lacks; check the ones it has.
    for s in kept:
        end = s["close_t"] or "9999-12-31T00:00:00.000Z"
        for key, typ, val, t, where in s["items"]:
            if key.startswith("$"):
                continue
            have = [e for e in kept_entries if e.get("key") == key
                    and in_window(e["t"], s["open_t"], end)]
            if have:
                v = have[0].get("value")
                if isinstance(v, (int, float)) and val != 0:
                    stats["checked"] += 1
                    if abs(v - val) > 1e-5 * abs(val):
                        stats["mismatch"].append(f"{serial} {key}: file {v} log {val}")
                continue
            kept_entries.append(entry(key, typ, val, t, where))
            row["added_to_kept"] += 1
            keys[key] = keys.get(key, 0) + 1

    if not lost and not row["relabeled"] and not row["added_to_kept"]:
        row["status"] = "unchanged"
        return None, row

    merged = sorted(new_entries + kept_entries, key=lambda e: e["t"])  # stable
    for i, e in enumerate(merged, 1):
        e["seq"] = i
    out = dict(rec)
    out["entries"] = merged
    if lost:   # a reopened record keeps its first creation time, as DutSession does
        out["opened_at"] = min(rec.get("opened_at") or first_open, lost[0]["open_t"])
    row["status"] = "merged"
    row["keys"] = "; ".join(f"{k}={n}" for k, n in sorted(keys.items()))
    return out, row


def write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
        f.write("\n")
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default="C:/ProgramData/FireDog", help="FireDog machine root")
    ap.add_argument("--product", default="siphog888")
    ap.add_argument("--data", help="the product's DUT folder (default <root>/data/products/<product>/data)")
    ap.add_argument("--logs", help="log folder (default <root>/logs)")
    ap.add_argument("--out", help="output folder (default data_recovered/ beside --data)")
    ap.add_argument("--serial", nargs="*", help="only these serials")
    ap.add_argument("--apply", action="store_true",
                    help="write in place; the original is kept as dut.json.pre-recovery.bak")
    a = ap.parse_args()

    data_dir = a.data or os.path.join(a.root, "data", "products", a.product, "data")
    log_dir = a.logs or os.path.join(a.root, "logs")
    out_dir = a.out or os.path.join(os.path.dirname(os.path.normpath(data_dir)), "data_recovered")
    if not os.path.isdir(data_dir) or not os.path.isdir(log_dir):
        sys.exit(f"missing folder: {data_dir if not os.path.isdir(data_dir) else log_dir}")

    sessions, windows = parse_logs(log_dir)
    print(f"logs: {len(sessions)} DUT session(s), {len(windows)} '{AFTER_BAKE_RECIPE}' run(s)")

    stats = {"checked": 0, "mismatch": []}
    rows = []
    for d in sorted(os.listdir(data_dir)):
        f = os.path.join(data_dir, d, "dut.json")
        if not os.path.isfile(f) or (a.serial and d not in a.serial):
            continue
        try:
            with open(f, encoding="utf-8") as fh:
                rec = json.load(fh)
        except (OSError, json.JSONDecodeError) as e:
            rows.append({"serial": d, "status": "skip", "note": f"unreadable: {e}"})
            continue
        merged, row = merge(d, rec, sessions, windows, stats)
        rows.append(row)
        if merged is None:
            continue
        if a.apply:
            if not rec.get("closed_at"):
                row["status"], row["note"] = "skip", "record is open (closed_at empty)"
                continue
            bak = f + ".pre-recovery.bak"
            if not os.path.exists(bak):          # keep the FIRST original on a re-run
                shutil.copy2(f, bak)
            write_json(f, merged)
        else:
            write_json(os.path.join(out_dir, d, "dut.json"), merged)

    os.makedirs(out_dir, exist_ok=True)
    report = os.path.join(out_dir, "recovery_report.csv")
    cols = ["serial", "status", "lost_sessions", "logged_lost_entries", "recovered",
            "relabeled", "added_to_kept", "keys", "note"]
    with open(report, "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})

    count = lambda s: sum(1 for r in rows if r.get("status") == s)
    totals = {}
    for r in rows:
        for kv in filter(None, (r.get("keys") or "").split("; ")):
            k, n = kv.rsplit("=", 1)
            totals[k] = totals.get(k, 0) + int(n)
    print(f"records: {len(rows)}  merged {count('merged')}  unchanged {count('unchanged')}"
          f"  skipped {count('skip')}")
    print(f"values recovered: {totals or 'none'}")
    print(f"relabeled after-bake readings: {sum(r.get('relabeled', 0) for r in rows)}")
    print(f"parser check: {stats['checked']} logged value(s) compared with intact entries,"
          f" {len(stats['mismatch'])} mismatch(es)")
    for m in stats["mismatch"][:10]:
        print("  MISMATCH", m)
    print(("applied in place under " + data_dir) if a.apply else ("merged copies in " + out_dir))
    print("report: " + report)


if __name__ == "__main__":
    main()
