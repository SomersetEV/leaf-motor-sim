"""Burst-wise fit of J, Tc and b from a bursty tractor log of runs R1, R2 and R4.

    python analyse_r1r2r4.py LOG OUTDIR [--r12 T0 T1] [--r4 T0 T1]

Written for the 13 Nov 2026 captures (docs/tractor/R1R2R4_2026-11-13.md). The
telematics logger kept only about 0.23 s of every 2 s, so extract_params.py
cannot be used: its 5 Hz filter needs 0.3 s of clean data either side of a
sample. Here each sample inside a burst gets a speed slope from a 70 ms
straight-line fit, and the fit is

    T_reported = J dw/dt + Tc + b w          (w > 300 rpm)

on reported torque (0x1DA), not the request, because in R4 the inverter
delivered 40-70 % of the request and the accelerations follow the reported
value. Times are seconds from the first frame (loglib's t). Needs the tools/
requirements; run from anywhere.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import lsq_linear

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
import loglib  # noqa: E402

RPM = np.pi / 30
REP = "INV_Status_1DA.ReportedTorque"
POT = "GOV_Speed_4A0.PotUser"
ERR = "GOV_Speed_4A0.GovSpeedErr"
HALF = 3  # slope window half-width, ticks


def slope_rows(g):
    w = g[loglib.SPEED].to_numpy() * RPM
    rep = g[REP].to_numpy()
    req = g[loglib.REQ_RAW].to_numpy() * 0.25
    t = g.index.to_numpy()
    x = np.arange(-HALF, HALF + 1) * loglib.GRID_S
    rows = []
    for i in range(HALF, len(w) - HALF):
        sl = slice(i - HALF, i + HALF + 1)
        if np.isnan(w[sl]).any() or np.isnan(rep[sl]).any():
            continue
        rows.append((t[i], w[i], np.polyfit(x, w[sl], 1)[0], rep[sl].mean(), np.nanmean(req[sl])))
    return pd.DataFrame(rows, columns=["t", "w", "a", "rep", "req"])


def fit(d, col="rep"):
    A = np.column_stack([d.a, np.ones(len(d)), d.w])
    y = d[col].to_numpy()
    r = lsq_linear(A, y, bounds=(0, np.inf))
    res = y - A @ r.x
    return {"J": r.x[0], "Tc": r.x[1], "b": r.x[2], "n": int(len(d)), "rms_nm": float(np.sqrt(np.mean(res ** 2)))}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("outdir")
    ap.add_argument("--r12", nargs=2, type=float, default=[206.0, 372.0], help="R1/R2 window, s")
    ap.add_argument("--r4", nargs=2, type=float, default=[383.0, 447.0], help="R4 window, s")
    a = ap.parse_args()
    out = Path(a.outdir)
    out.mkdir(parents=True, exist_ok=True)

    log = loglib.load(a.log)
    g = log.grid
    print(loglib.summary(log))
    d = slope_rows(g)
    d = d[d.w > 300 * RPM]
    r12 = d[(d.t > a.r12[0]) & (d.t < a.r12[1])]
    r4 = d[(d.t > a.r4[0]) & (d.t < a.r4[1])]
    both = pd.concat([r12, r4])
    fits = {"R1/R2 reported": fit(r12), "R1/R2 requested": fit(r12.dropna(), "req"),
            "R4 reported": fit(r4), "R1/R2 + R4 reported": fit(both)}
    for k, f in fits.items():
        print(f"{k:22s} n={f['n']:4d}  J={f['J']:.4f} kg m2  Tc={f['Tc']:.2f} Nm  "
              f"b={f['b']:.4f} Nm s/rad  rms={f['rms_nm']:.2f} Nm")
    (out / "fit.json").write_text(json.dumps(fits, indent=2))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # 1. overview
    fig, ax = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    ax[0].plot(g.index, g[loglib.SPEED], "k.", ms=1)
    ax[0].plot(g.index, g[loglib.REVLIM].ffill(), "r-", lw=0.8, label="revlim")
    ax[0].set_ylabel("rpm")
    ax[0].set_ylim(-500, 3700)
    ax[0].legend(loc="upper left")
    ax[1].plot(g.index, g[POT], "C0.", ms=1)
    ax[1].set_ylabel("pedal %")
    ax[2].plot(g.index, g[loglib.REQ_RAW] * 0.25, "C0.", ms=1, label="request")
    ax[2].plot(g.index, g[REP], "C2.", ms=1, label="reported")
    ax[2].set_ylabel("Nm")
    ax[2].set_xlabel("t, s (log has 0.23 s of every ~2 s)")
    ax[2].legend(loc="upper left")
    for x in (a.r12[0], a.r4[0]):
        for axx in ax:
            axx.axvline(x, color="grey", lw=0.5, ls="--")
    ax[0].set_title("R1/R2 (revlim 3000, regen 0) then R4 (revlim 2200, Kp 11, Ki 0, govFlt 4)")
    fig.tight_layout()
    fig.savefig(out / "overview.png", dpi=100)
    plt.close(fig)

    # 2. friction: speed-holds (|accel| small) against the fitted line
    f = fits["R1/R2 + R4 reported"]
    hold = r12[np.abs(r12.a) < 3]
    fig, axx = plt.subplots(figsize=(7, 4.5))
    axx.plot(hold.w / RPM, hold.rep, ".", ms=3, alpha=0.5, label="R1/R2 holds, reported torque")
    rpm = np.linspace(0, 2500, 50)
    axx.plot(rpm, f["Tc"] + f["b"] * rpm * RPM, "k-", label=f"fit: {f['Tc']:.1f} Nm + {f['b'] * RPM * 1000:.2f} Nm per 1000 rpm")
    axx.set_xlabel("rpm")
    axx.set_ylabel("Nm")
    axx.set_title(f"Friction in neutral, PTO out (J fitted {f['J']:.3f} kg m2)")
    axx.legend()
    axx.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "friction.png", dpi=100)
    plt.close(fig)

    # 3. R4 bursts
    sp = g.loc[a.r4[0]:a.r4[1], loglib.SPEED]
    ok = sp[sp.notna() & (sp > 300)].index.to_numpy()
    cuts = np.where(np.diff(ok) > 0.05)[0]
    segs = [(s, e) for s, e in zip(np.r_[ok[0], ok[cuts + 1]], np.r_[ok[cuts], ok[-1]]) if e - s > 0.1]
    filt = g[loglib.REVLIM].ffill() - g[ERR]
    fig, axes = plt.subplots(2, len(segs), figsize=(1.9 * len(segs), 6), sharey="row",
                             gridspec_kw=dict(wspace=0.08), squeeze=False)
    for k, (s0, s1) in enumerate(segs):
        s = g.loc[s0:s1]
        ax0, ax1 = axes[0, k], axes[1, k]
        ax0.plot(s.index, s[loglib.SPEED], "k.-", ms=2, lw=0.8, label="speed")
        ax0.plot(s.index, filt.loc[s0:s1], "C1-", lw=1, label="governor filtered")
        ax0.axhline(2200, color="r", lw=0.6, ls="--")
        ax0.set_title(f"{s0:.1f} s", fontsize=8)
        ax1.plot(s.index, s[loglib.REQ_RAW] * 0.25, "C0.-", ms=2, lw=0.8, label="request")
        ax1.plot(s.index, s[REP], "C2.-", ms=2, lw=0.8, label="reported")
        ax1.axhline(0, color="grey", lw=0.4)
        for axx in (ax0, ax1):
            axx.tick_params(labelsize=6)
            axx.set_xticks([])
    axes[0, 0].set_ylabel("rpm")
    axes[1, 0].set_ylabel("Nm")
    axes[0, 0].legend(fontsize=7)
    axes[1, 0].legend(fontsize=7)
    fig.suptitle("R4 bursts, 0.23 s each: revlim 2200, Kp 11, Ki 0, govFlt 4", fontsize=10)
    fig.savefig(out / "r4_bursts.png", dpi=100, bbox_inches="tight")
    plt.close(fig)
    print("written", out)


if __name__ == "__main__":
    main()
