"""Governor performance metrics, computed the same way for tractor and bench.

Plan section 9, "Metrics". All functions take plain arrays on a uniform grid
(t in seconds, speed in rpm) so they can be tested without logs.
"""

from __future__ import annotations

import numpy as np


def first_crossing(t, speed, level):
    """Time speed first reaches level, or None."""
    idx = np.flatnonzero(np.asarray(speed) >= level)
    return float(t[idx[0]]) if len(idx) else None


def overshoot_rpm(speed, revlim):
    """Highest speed above revlim (negative if never reached)."""
    return float(np.nanmax(speed) - revlim)


def settle_time_s(t, speed, revlim, band=0.01, window_s=10.0):
    """Time from first reaching revlim (less band) until speed stays within
    +/- band of revlim, judged over window_s after that first crossing."""
    t = np.asarray(t)
    speed = np.asarray(speed)
    t0 = first_crossing(t, speed, revlim * (1 - band))
    if t0 is None:
        return float("nan")
    sel = (t >= t0) & (t <= t0 + window_s)
    outside = np.abs(speed[sel] - revlim) > band * revlim
    if not outside.any():
        return 0.0
    last_out = t[sel][np.flatnonzero(outside)[-1]]
    if last_out >= t[sel][-1]:
        return float("nan")  # still outside at the end of the window
    return float(last_out - t0 + (t[1] - t[0]))


def droop_rpm(speed, revlim, mask=None):
    """revlim minus the median speed while governing under load."""
    speed = np.asarray(speed)
    if mask is None:
        mask = speed > 0.9 * revlim
    sel = np.asarray(mask, bool) & ~np.isnan(speed)
    return float(revlim - np.median(speed[sel])) if sel.any() else float("nan")


def dominant_period_s(t, speed, f_lo=0.3, f_hi=5.0):
    """Period of the strongest oscillation between f_lo and f_hi."""
    x = np.asarray(speed, dtype=float)
    x = x[~np.isnan(x)]
    if len(x) < 64:
        return float("nan")
    dt = float(t[1] - t[0])
    x = x - np.mean(x)
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    f = np.fft.rfftfreq(len(x), dt)
    band = (f >= f_lo) & (f <= f_hi)
    if not band.any():
        return float("nan")
    return float(1.0 / f[band][np.argmax(spec[band])])


def ripple_p2p_rpm(t, speed, period_s, mask=None):
    """Median peak-to-peak speed over consecutive windows of one stroke."""
    t = np.asarray(t)
    speed = np.asarray(speed, dtype=float)
    if not np.isfinite(period_s) or period_s <= 0:
        return float("nan")
    n = max(2, int(round(period_s / (t[1] - t[0]))))
    sel = np.ones(len(speed), bool) if mask is None else np.asarray(mask, bool)
    p2p = []
    for i in range(0, len(speed) - n + 1, n):
        if sel[i:i + n].all():
            w = speed[i:i + n]
            if not np.isnan(w).any():
                p2p.append(w.max() - w.min())
    return float(np.median(p2p)) if p2p else float("nan")


def recovery_time_s(t, speed, t_step, band_rpm, pre_s=1.0, hold_s=1.0, max_s=20.0):
    """Time after a load step until speed returns within band_rpm of its
    pre-step mean and stays there for hold_s."""
    t = np.asarray(t)
    speed = np.asarray(speed, dtype=float)
    pre = speed[(t >= t_step - pre_s) & (t < t_step)]
    if len(pre) == 0:
        return float("nan")
    ref = np.nanmean(pre)
    sel = (t >= t_step) & (t <= t_step + max_s)
    ts, vs = t[sel], speed[sel]
    inside = np.abs(vs - ref) <= band_rpm
    dt = t[1] - t[0]
    need = int(round(hold_s / dt))
    run = 0
    for i, ok in enumerate(inside):
        run = run + 1 if ok else 0
        if run >= need:
            return float(ts[i - need + 1] - t_step)
    return float("nan")


def peak_abs(x):
    x = np.asarray(x, dtype=float)
    return float(np.nanmax(np.abs(x))) if np.isfinite(x).any() else float("nan")


def time_true_s(flag, dt=0.01):
    f = np.nan_to_num(np.asarray(flag, dtype=float))
    return float(np.sum(f > 0.5) * dt)
