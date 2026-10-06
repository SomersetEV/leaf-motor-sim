"""Compare a tractor log and a bench log of the same scenario.

Plan section 9, "Metrics", and section 10. Computes the same numbers for
both logs, prints a table, and saves overlay plots:

  * overshoot above revlim, and time to settle within 1 % of revlim
  * steady-state droop below revlim under load
  * peak-to-peak rpm ripple per plunger stroke
  * recovery time after a load step
  * peak integral term, and time with the integral clamped

    python compare_runs.py tractor.csv bench.csv [--revlim 2000] [--plot cmp.png]

revlim is read from the 0x4A2 frames when present. Bench load steps are found
from 0x4B2 load events; give tractor step times with --step-tractor.
"""

from __future__ import annotations

import argparse
import csv
import sys

import numpy as np

import loglib
import metrics as mt


def step_times_from_events(log: loglib.Log):
    col = loglib.SIM_EVENT
    if not log.has(col):
        return []
    ev = log.grid[col].to_numpy()
    t = log.grid.index.to_numpy()
    return [float(x) for x in t[ev == 2]]


def compute(log: loglib.Log, revlim, stroke_s=None, steps=None, align=0.0) -> dict:
    g = log.grid
    t = g.index.to_numpy() - align
    v = np.abs(loglib.speed_rpm(log).to_numpy(dtype=float))
    if revlim is None:
        if not log.has(loglib.REVLIM):
            raise ValueError(f"{log.source}: no revlim in the log; pass --revlim")
        revlim = float(np.nanmedian(g[loglib.REVLIM]))
    governing = v > 0.9 * revlim
    binding = g[loglib.GOV_BINDING].to_numpy() > 0.5 if log.has(loglib.GOV_BINDING) else None
    if binding is not None and not binding.any():
        binding = None  # governor never bound: fall back to speed near the limit
    period = stroke_s if stroke_s else mt.dominant_period_s(t, v)
    out = {
        "revlim_rpm": revlim,
        "overshoot_rpm": mt.overshoot_rpm(v, revlim),
        "settle_s": mt.settle_time_s(t, v, revlim),
        "droop_rpm": mt.droop_rpm(v, revlim, binding if binding is not None else governing),
        "stroke_period_s": period,
        "ripple_p2p_rpm": mt.ripple_p2p_rpm(t, v, period, governing),
    }
    steps = steps if steps is not None else [s - align for s in step_times_from_events(log)]
    rec = [mt.recovery_time_s(t, v, s, 0.01 * revlim) for s in steps]
    out["recovery_s"] = float(np.nanmean(rec)) if rec and np.isfinite(rec).any() else float("nan")
    out["peak_integral_pct"] = mt.peak_abs(g[loglib.GOV_INT]) if log.has(loglib.GOV_INT) else float("nan")
    out["int_clamped_s"] = mt.time_true_s(g[loglib.GOV_CLAMPED]) if log.has(loglib.GOV_CLAMPED) else float("nan")
    return out


def alignment(log: loglib.Log, revlim, mode: str) -> float:
    if mode == "start":
        return 0.0
    v = np.abs(loglib.speed_rpm(log).to_numpy(dtype=float))
    t0 = mt.first_crossing(log.grid.index.to_numpy(), v, 0.5 * revlim)
    return t0 or 0.0


def plot(logs, labels, aligns, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    have_int = any(lg.has(loglib.GOV_INT) for lg in logs)
    fig, axes = plt.subplots(3 if have_int else 2, 1, figsize=(12, 9 if have_int else 6), sharex=True)
    for lg, lab, a in zip(logs, labels, aligns):
        t = lg.grid.index.to_numpy() - a
        axes[0].plot(t, loglib.speed_rpm(lg), lw=0.8, label=lab)
        if lg.has(loglib.REQ_RAW):
            axes[1].plot(t, lg.grid[loglib.REQ_RAW] * 100 / 2047, lw=0.8, label=lab)
        if have_int and lg.has(loglib.GOV_INT):
            axes[2].plot(t, lg.grid[loglib.GOV_INT], lw=0.8, label=lab)
    axes[0].set_ylabel("speed [rpm]")
    axes[1].set_ylabel("torque request [%]")
    if have_int:
        axes[2].set_ylabel("governor integral [%]")
    axes[-1].set_xlabel("time from alignment point [s]")
    for ax in axes:
        ax.legend(loc="upper right")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tractor")
    ap.add_argument("bench")
    ap.add_argument("--revlim", type=float, help="rpm (default: from 0x4A2)")
    ap.add_argument("--stroke-ms", type=float, help="plunger period (default: dominant oscillation)")
    ap.add_argument("--step-tractor", type=float, nargs="*", default=None, help="load step times, s")
    ap.add_argument("--step-bench", type=float, nargs="*", default=None,
                    help="load step times, s (default: 0x4B2 load events)")
    ap.add_argument("--align", choices=["cross", "start"], default="cross",
                    help="cross: first time speed reaches half of revlim")
    ap.add_argument("--plot", default="compare.png")
    ap.add_argument("--csv", help="also write the table as CSV")
    ap.add_argument("--dbc", default=str(loglib.DEFAULT_DBC))
    a = ap.parse_args(argv)

    logs = [loglib.load(a.tractor, a.dbc), loglib.load(a.bench, a.dbc)]
    for lg in logs:
        print(loglib.summary(lg))
    revlim = a.revlim
    if revlim is None:
        for lg in logs:
            if lg.has(loglib.REVLIM):
                revlim = float(np.nanmedian(lg.grid[loglib.REVLIM]))
                break
    if revlim is None:
        print("error: no revlim in either log; pass --revlim", file=sys.stderr)
        return 2
    stroke = a.stroke_ms / 1000 if a.stroke_ms else None
    aligns = [alignment(lg, revlim, a.align) for lg in logs]
    steps = [a.step_tractor, a.step_bench]
    res = [compute(lg, revlim, stroke, None if s is None else [x - al for x in s], al)
           for lg, s, al in zip(logs, steps, aligns)]

    rows = []
    print(f"\n| metric | tractor | bench | bench vs tractor |\n|---|---|---|---|")
    for k in res[0]:
        tr, be = res[0][k], res[1][k]
        diff = (be - tr) / abs(tr) * 100 if np.isfinite(tr) and np.isfinite(be) and tr != 0 else float("nan")
        print(f"| {k} | {tr:.3f} | {be:.3f} | {diff:+.1f} % |")
        rows.append([k, tr, be, diff])
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "tractor", "bench", "bench_vs_tractor_pct"])
            w.writerows(rows)
    plot(logs, ["tractor", "bench"], aligns, a.plot)
    print(f"\nplot written to {a.plot}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
