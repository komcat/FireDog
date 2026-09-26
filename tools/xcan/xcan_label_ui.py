"""Review UI for the XCAN labeler (xcan_label.py ui, or run this file).

Labelling is automatic: every scan gets its label from the chain rules the moment the
data is loaded. This window is for LOOKING at those labels and overruling the ones that
are wrong. An override goes to data/xcan/overrides.csv on the spot; Export writes
scans.csv, samples.csv and the per-label copies into xcan_labeled/.

Keys: Up/Down select, 1..6 set a label, 0 back to automatic, C toggles the chain plot,
F5 reloads from disk (FireDog keeps writing while this is open).
"""

import os
import sys
import tkinter as tk
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import xcan_label as xl

COLORS = {
    "stall": "#c0392b", "recovery": "#1e8449", "peak": "#1f5fa8",
    "single": "#7f6000", "no_signal": "#888888", "excluded": "#555555",
}
COLUMNS = (
    ("utc", "Time (UTC)", 150), ("device", "Device", 80), ("chain", "Chain", 60),
    ("status", "Status", 70), ("final", "Final", 90), ("frac", "x best", 60),
    ("auto", "Auto", 75), ("label", "Label", 75), ("src", "Src", 55),
)


def fmt(v, spec="{:.3e}"):
    return "" if v is None else spec.format(v)


class App:
    def __init__(self, root, a):
        self.root, self.a = root, a
        self.scans, self.labels, self.rows, self.chains = {}, {}, [], {}
        self.overrides = {}
        root.title(f"XCAN labels - {os.path.abspath(a.data)}")
        root.geometry("1500x850")

        # ---- top bar: parameters, actions, filter
        top = ttk.Frame(root, padding=4)
        top.pack(side=tk.TOP, fill=tk.X)
        self.v_ratio = tk.StringVar(value=str(a.stall_ratio))
        self.v_min = tk.StringVar(value=str(a.min_signal))
        self.v_gap = tk.StringVar(value=str(a.max_gap_s))
        for text, var in (("Stall ratio", self.v_ratio), ("Min signal (A)", self.v_min),
                          ("Max gap (s)", self.v_gap)):
            ttk.Label(top, text=text).pack(side=tk.LEFT, padx=(8, 2))
            ttk.Entry(top, textvariable=var, width=9).pack(side=tk.LEFT)
        ttk.Button(top, text="Relabel", command=self.relabel).pack(side=tk.LEFT, padx=8)
        ttk.Button(top, text="Reload (F5)", command=self.reload).pack(side=tk.LEFT)
        ttk.Button(top, text="Export", command=self.export).pack(side=tk.LEFT, padx=8)
        ttk.Label(top, text="Show").pack(side=tk.LEFT, padx=(16, 2))
        self.v_filter = tk.StringVar(value="all")
        cb = ttk.Combobox(top, textvariable=self.v_filter, state="readonly", width=12,
                          values=("all", "manual") + xl.LABELS)
        cb.pack(side=tk.LEFT)
        cb.bind("<<ComboboxSelected>>", lambda e: self.fill_tree())
        self.v_status = tk.StringVar()
        ttk.Label(top, textvariable=self.v_status).pack(side=tk.RIGHT, padx=8)

        pane = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        pane.pack(fill=tk.BOTH, expand=True)

        # ---- left: the scan list
        left = ttk.Frame(pane)
        self.tree = ttk.Treeview(left, columns=[c[0] for c in COLUMNS], show="headings",
                                 selectmode="browse")
        for key, text, w in COLUMNS:
            self.tree.heading(key, text=text)
            self.tree.column(key, width=w, anchor=tk.W, stretch=False)
        for lab, c in COLORS.items():
            self.tree.tag_configure(lab, foreground=c)
        self.tree.tag_configure("manual", background="#fff3c4")
        sb = ttk.Scrollbar(left, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.LEFT, fill=tk.Y)
        self.tree.bind("<<TreeviewSelect>>", lambda e: self.show_selected())
        pane.add(left, weight=2)

        # ---- right: plot, details, label buttons
        right = ttk.Frame(pane)
        self.fig = Figure(figsize=(7, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        NavigationToolbar2Tk(self.canvas, right).update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        opts = ttk.Frame(right)
        opts.pack(fill=tk.X)
        self.v_chain = tk.BooleanVar(value=True)
        ttk.Checkbutton(opts, text="Show whole chain (C)", variable=self.v_chain,
                        command=self.show_selected).pack(side=tk.LEFT)

        self.info = tk.Text(right, height=7, wrap=tk.WORD, relief=tk.FLAT,
                            background=root.cget("background"))
        self.info.pack(fill=tk.X, padx=4)

        btns = ttk.LabelFrame(right, text="Set label", padding=4)
        btns.pack(fill=tk.X, padx=4, pady=4)
        for i, lab in enumerate(xl.LABELS):
            tk.Button(btns, text=f"{i + 1}  {lab}", fg=COLORS[lab], width=12,
                      command=lambda l=lab: self.set_label(l)).pack(side=tk.LEFT, padx=2)
        tk.Button(btns, text="0  auto", width=8,
                  command=lambda: self.set_label(None)).pack(side=tk.LEFT, padx=(12, 2))
        nf = ttk.Frame(right)
        nf.pack(fill=tk.X, padx=4, pady=(0, 6))
        ttk.Label(nf, text="Note").pack(side=tk.LEFT)
        self.v_note = tk.StringVar()
        self.note_entry = ttk.Entry(nf, textvariable=self.v_note)
        self.note_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.note_entry.bind("<Return>", lambda e: self.save_note())
        ttk.Button(nf, text="Save note", command=self.save_note).pack(side=tk.LEFT)
        pane.add(right, weight=3)

        for k in range(len(xl.LABELS)):
            root.bind(str(k + 1), lambda e, l=xl.LABELS[k]: self.key(lambda: self.set_label(l)))
        root.bind("0", lambda e: self.key(lambda: self.set_label(None)))
        root.bind("c", lambda e: self.key(self.toggle_chain))
        root.bind("<F5>", lambda e: self.reload())

        self.reload()

    # ---- keys typed into the note box must not relabel the scan
    def key(self, fn):
        if self.root.focus_get() is not self.note_entry:
            fn()

    def toggle_chain(self):
        self.v_chain.set(not self.v_chain.get())
        self.show_selected()

    def params(self):
        try:
            return float(self.v_ratio.get()), float(self.v_min.get()), float(self.v_gap.get())
        except ValueError:
            messagebox.showerror("XCAN labels", "Stall ratio, min signal and max gap must be numbers")
            return None

    # ---- data
    def reload(self):
        self.scans = xl.load_all(self.a.data)
        self.overrides = xl.load_overrides(self.a.data)
        self.relabel()

    def relabel(self):
        p = self.params()
        if p is None:
            return
        if not self.scans:
            self.rows, self.labels, self.chains = [], {}, {}
        else:
            self.chains, self.labels, self.rows = xl.compute(self.scans, self.overrides, *p)
        self.fill_tree()

    def fill_tree(self):
        keep = self.selected()
        self.tree.delete(*self.tree.get_children())
        f = self.v_filter.get()
        for r in self.rows:
            if f == "manual" and r["label_source"] != "manual":
                continue
            if f not in ("all", "manual") and r["label"] != f:
                continue
            tags = [r["label"]] + (["manual"] if r["label_source"] == "manual" else [])
            self.tree.insert("", tk.END, iid=r["scan_id"], tags=tags, values=(
                r["utc"], r["device"], f"{r['chain_pos']}/{r['chain_len']}", r["status"],
                fmt(r["final"]), fmt(r["frac_of_best"], "{:.2f}"), r["auto_label"],
                r["label"], r["label_source"]))
        self.v_status.set(f"{len(self.rows)} scans, {len(self.chains)} chains | "
                          + xl.summary(self.rows))
        items = self.tree.get_children()
        if keep in items:
            target = keep
        elif items:
            target = items[0]
        else:
            self.fig.clear()
            self.canvas.draw_idle()
            return
        self.tree.selection_set(target)
        self.tree.see(target)
        self.tree.focus(target)

    def selected(self):
        sel = self.tree.selection()
        return sel[0] if sel else None

    def row(self, sid):
        return next((r for r in self.rows if r["scan_id"] == sid), None)

    def show_selected(self):
        sid = self.selected()
        if not sid:
            return
        r, lab = self.row(sid), self.labels[sid]
        ids = next(m for m in self.chains.values() if sid in m) if self.v_chain.get() else [sid]
        names = {x["scan_id"]: x["label"] for x in self.rows}
        xl.draw_scans(self.fig, self.scans, ids, names, sid, self.v_chain.get())
        self.canvas.draw_idle()

        h = self.scans[sid]["h"]
        ctx = {k: v for k, v in (h.get("context") or {}).items() if v not in (None, "")}
        lines = [
            f"{sid}   label {r['label']} ({r['label_source']}), auto {r['auto_label']}",
            f"product {r['product']}   serial {r['serial']}   topic {r['topic']}   "
            f"context {ctx or '-'}",
            f"baseline {fmt(r['baseline'])}   final {fmt(r['final'])}   "
            f"best_v {fmt(r['best_v'])}   chain_best {fmt(r['chain_best'])}",
            f"samples {r['n_samples']}   {fmt(r['duration_s'], '{:.1f}')} s   "
            f"settle timeouts {r['settle_timeouts']}",
        ]
        if lab.get("reason"):
            lines.append(f"reason: {lab['reason']}")
        if lab.get("recovered_by"):
            lines.append(f"recovered by {lab['recovered_by']}")
        if self.scans[sid]["r"].get("error"):
            lines.append(f"error: {self.scans[sid]['r']['error']}")
        self.info.configure(state=tk.NORMAL)
        self.info.delete("1.0", tk.END)
        self.info.insert("1.0", "\n".join(lines))
        self.info.configure(state=tk.DISABLED)
        self.v_note.set(r["override_note"])

    # ---- edits
    def set_label(self, lab):
        sid = self.selected()
        if not sid:
            return
        if lab is None:
            self.overrides.pop(sid, None)
        else:
            self.overrides[sid] = (lab, self.v_note.get().strip())
        self.commit(sid)

    def save_note(self):
        sid = self.selected()
        if not sid:
            return
        lab = self.overrides.get(sid, (self.row(sid)["label"], ""))[0]
        self.overrides[sid] = (lab, self.v_note.get().strip())
        self.commit(sid)

    def commit(self, sid):
        xl.save_overrides(self.a.data, self.overrides)
        self.relabel()
        items = self.tree.get_children()
        if sid in items:  # a filter may have hidden it; then stay where the list starts
            nxt = self.tree.next(sid)
            if nxt:  # move on, so labelling a run of scans is one key per scan
                self.tree.selection_set(nxt)
                self.tree.see(nxt)
                self.tree.focus(nxt)

    def export(self):
        if not self.rows:
            return
        n = xl.export(self.scans, self.labels, self.rows, self.a.out)
        messagebox.showinfo("XCAN labels",
                            f"{len(self.rows)} scans, {n} samples\n{xl.summary(self.rows)}\n\n"
                            f"-> {os.path.abspath(self.a.out)}")


def run(a):
    root = tk.Tk()
    App(root, a)
    root.mainloop()


if __name__ == "__main__":
    sys.argv.insert(1, "ui")
    xl.main()
