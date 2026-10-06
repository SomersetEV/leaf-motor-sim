"""Fit inertia and friction from no-load acceleration and coast-down logs.

Plan section 9 (runs R1, R2, and R3 for the implement). Solves, by bounded
linear least squares over every log given,

    T_m = J dw/dt + sgn(w) (Tc + b|w| + c w^2)

where T_m is the 0x1D4 request through the motor envelope and torque lag,
and w is the 0x1DA speed. Both sides are smoothed with the same zero-phase
filter before fitting, so torque steps do not bias the derivative.

    python extract_params.py r1.csv r2.csv -o params.json [--plot fit.png]

Limits, stated in the output too:
  * From speed and torque request alone only J / kT is identifiable. That
    ratio sets the loop behaviour, so the bench stays valid; absolute values
    need reported torque (0x1DA, plan V3) or DC power from a shunt.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

import loglib
import simmodel as sm

NOTE = ("Only J/kT is identifiable from speed and torque request alone; absolute J, Tc, b, c "
        "scale with the assumed kT (plan V1). The ratio is what sets loop behaviour.")


def fit_columns(log: loglib.Log, m: sm.MotorParams, cutoff_hz: float, w_min: float,
                delay_ticks: int):
    """Regression rows for one log: returns (A, y, t, w, t_m) with masked rows removed."""
    g = log.grid
    if not log.has(loglib.REQ_RAW):
        raise ValueError(f"{log.source}: no 0x1D4 torque requests")
    w = loglib.speed_rpm(log).to_numpy(dtype=float) * sm.RPM_TO_RAD_S
    valid = ~np.isnan(w)
    w = np.where(valid, w, 0.0)
    t_req = sm.requested_torque(g[loglib.REQ_RAW].to_numpy(), m.kT, delay_ticks)
    t_m = sm.motor_torque_tick_avg(t_req, w, m)

    # Friction averaged over each tick, assuming speed changes linearly within it
    w_prev = np.concatenate([[w[0]], w[:-1]])
    w_mid = 0.5 * (w + w_prev)
    sgn = np.sign(w_mid)
    mean_sq = (w * w + w * w_prev + w_prev * w_prev) / 3.0
    cols = [sm.backward_derivative(w), sgn, sgn * np.abs(w_mid), sgn * mean_sq]
    A = np.column_stack([sm.zero_phase(c, cutoff_hz) for c in cols])
    y = sm.zero_phase(t_m, cutoff_hz)

    # Exclude standstill (friction holds the shaft, the equation does not apply),
    # missing data, and the filter's reach around them.
    bad = (np.abs(w) < w_min) | ~valid
    reach = int(round(1.5 / (cutoff_hz * sm.DT))) if cutoff_hz > 0 else 1
    bad = np.convolve(bad.astype(float), np.ones(2 * reach + 1), mode="same") > 0
    keep = ~bad
    keep[: reach + 1] = False
    keep[-reach - 1:] = False
    return A[keep], y[keep], g.index.to_numpy()[keep], w[keep], t_m[keep]


def fit(logs, m: sm.MotorParams, cutoff_hz=5.0, w_min=20.0, delay_ticks=None):
    """Fits J, Tc, b, c. delay_ticks None tries -2..2 and keeps the best fit."""
    if delay_ticks is None:
        best = None
        for d in range(-2, 3):
            r = fit(logs, m, cutoff_hz, w_min, d)
            if best is None or r[0]["fit"]["rms_residual_nm"] < best[0]["fit"]["rms_residual_nm"]:
                best = r
        return best
    parts = [fit_columns(lg, m, cutoff_hz, w_min, delay_ticks) for lg in logs]
    A = np.vstack([p[0] for p in parts])
    y = np.concatenate([p[1] for p in parts])
    if len(y) < 50:
        raise ValueError("too few moving samples to fit; need acceleration and coast-down")
    res = lsq_linear(A, y, bounds=(0, np.inf))
    J, Tc, b, c = res.x
    pred = A @ res.x
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    return {
        "J": float(J), "Tc": float(Tc), "b": float(b), "c": float(c),
        "fit": {"samples": int(len(y)), "rms_residual_nm": float(np.sqrt(ss_res / len(y))),
                "r2": 1 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
                "delay_ticks": int(delay_ticks)},
    }, parts, res.x


def plot(parts, x, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(parts), 1, figsize=(11, 3.2 * len(parts)), squeeze=False)
    for ax, (A, y, t, w, _) in zip(axes[:, 0], parts):
        ax.plot(t, y, ".", ms=2, label="motor torque (filtered)")
        ax.plot(t, A @ x, ".", ms=2, label="fitted J dw/dt + friction")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("torque [Nm]")
        ax.legend(loc="upper right")
        ax2 = ax.twinx()
        ax2.plot(t, w / sm.RPM_TO_RAD_S, "k.", ms=0.8, alpha=0.4)  # points: fit rows only
        ax2.set_ylabel("speed [rpm]")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="R1 and R2 logs (or R3 for the implement)")
    ap.add_argument("-o", "--out", default="params.json")
    ap.add_argument("--plot", help="PNG of the fit (default: next to --out)")
    ap.add_argument("--dbc", default=str(loglib.DEFAULT_DBC))
    ap.add_argument("--kt", type=float, default=0.25, help="Nm per request bit (plan V1)")
    ap.add_argument("--tau", type=float, default=30.0, help="torque lag, ms")
    ap.add_argument("--tmax", type=float, default=280.0, help="motor torque limit, Nm (plan V2)")
    ap.add_argument("--pmax", type=float, default=80.0, help="motor power limit, kW (plan V2)")
    ap.add_argument("--cutoff", type=float, default=5.0, help="zero-phase filter cutoff, Hz")
    ap.add_argument("--w-min", type=float, default=20.0, help="ignore speeds below this, rad/s")
    ap.add_argument("--delay-ticks", type=int, default=None,
                    help="shift requests later by N ticks (default: best of -2..2)")
    a = ap.parse_args(argv)

    m = sm.MotorParams(a.kt, a.tau, a.tmax, a.pmax)
    logs = [loglib.load(p, a.dbc) for p in a.logs]
    for lg in logs:
        print(loglib.summary(lg))
    result, parts, x = fit(logs, m, a.cutoff, a.w_min, a.delay_ticks)
    result.update({"kT": a.kt, "tau": a.tau, "Tmax": a.tmax, "Pmax": a.pmax,
                   "J_over_kT": result["J"] / a.kt, "sources": a.logs, "note": NOTE})
    Path(a.out).write_text(json.dumps(result, indent=2))
    png = a.plot or str(Path(a.out).with_suffix(".png"))
    plot(parts, x, png)

    print(f"\nJ  = {result['J']:.4f} kg.m2   (J/kT = {result['J_over_kT']:.4f})")
    print(f"Tc = {result['Tc']:.3f} Nm")
    print(f"b  = {result['b']:.5f} Nm.s/rad")
    print(f"c  = {result['c']:.3e} Nm.s2/rad2")
    print(f"fit: {result['fit']['samples']} samples, rms {result['fit']['rms_residual_nm']:.2f} Nm, "
          f"R2 {result['fit']['r2']:.4f}, delay {result['fit']['delay_ticks']} ticks")
    print(f"\n{NOTE}\nwritten {a.out} and {png}")
    print("Simulator console: " + " ".join(
        f"set {k} {result[k]:.6g}" for k in ("J", "Tc", "b", "c")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
