"""Checks a bench log from scripts/m5b_report.yaml (plan items I6 and I7).

0x4B0 carries the model's own speed and 0x1DA the speed the Zombie is given,
in the same tick. Their difference is exactly what the simulator added:
the report delay, and the measurement noise.

  - Marks 1 to 2: the lag of 0x1DA behind 0x4B0, in ticks, should be 2
    (rptdelay 20 ms).
  - Marks 2 to 3: band noise (noise 11, noise_fc 6, noise_bw 4). Its SD, the
    frequency of its spectral peak, and what passes the governor filter at
    shifts 4 and 2, against what tools/fit_noise.py predicts.
  - Marks 4 to 5: white noise of the same SD, for comparison.

    python m5b_check.py m5b_report.log
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from scipy.signal import lfilter

import fit_noise
import loglib

MODEL = "SIM_Fast_4B0.ModelSpeed"


def window(log, a: int, b: int):
    ev = log.raw["SIM_Event_4B2"]
    marks = {int(v): float(t) for e, v, t in zip(ev["Event"], ev["EventValue"], ev["t"]) if int(e) == 1}
    t = log.grid.index.to_numpy()
    return (t >= marks[a] + 0.5) & (t < marks[b])


def gov_pass(x, shift: int) -> float:
    k = 1.0 / (1 << shift)
    return float(lfilter([k], [1, -(1 - k)], x)[100:].std())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    a = ap.parse_args(argv)
    log = loglib.load(a.log)
    rep = log.grid[loglib.SPEED].to_numpy(dtype=float)
    mod = log.grid[MODEL].to_numpy(dtype=float)
    ok = True

    w = window(log, 1, 2)
    errs = {lag: float(np.nanstd(rep[w][lag:] - mod[w][:len(mod[w]) - lag])) for lag in range(0, 6)}
    lag = min(errs, key=errs.get)
    moved = float(np.nanmax(mod[w]) - np.nanmin(mod[w]))
    ok &= lag == 2 and moved > 20
    print(f"rptdelay 20: 0x1DA lags the model by {lag} ticks ({lag * 10} ms); "
          f"the speed moved {moved:.0f} rpm. Residual by lag: "
          + ", ".join(f"{k}: {v:.2f}" for k, v in errs.items()))

    for (m0, m1), name, fc in (((2, 3), "band", 6.0), ((4, 5), "white", 0.0)):
        w = window(log, m0, m1)
        n = rep[w] - mod[w]
        n = n[np.isfinite(n)]
        spec = np.abs(np.fft.rfft((n - n.mean()) * np.hanning(len(n)))) ** 2
        f = np.fft.rfftfreq(len(n), 0.01)
        sm = np.convolve(spec, np.ones(25) / 25, mode="same")
        peak = float(f[1:][np.argmax(sm[1:])])
        line = f"{name} noise: sd {n.std():.2f} rpm (set 11)"
        if fc:
            line += f", spectral peak {peak:.1f} Hz (set 6)"
            ok &= abs(n.std() - 11) < 1.5 and 4.5 <= peak <= 7.5
        else:
            ok &= abs(n.std() - 11) < 1.5
        for k in (4, 2):
            got, want = gov_pass(n, k), fit_noise.true_passed(fc, 4.0, 11.0, k)
            line += f"; shift {k} passes {got:.2f} (model {want:.2f})"
            ok &= abs(got / want - 1) < 0.2
        print(line)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
