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


# --- Plan item I8: loop metrics and one cost for sweeps, harness and search ---

def iae_rpm_s(t, speed, ref, mask=None):
    """Integrated absolute error, rpm·s, over mask (default all samples)."""
    t = np.asarray(t, dtype=float)
    e = np.abs(np.asarray(speed, dtype=float) - ref)
    sel = np.isfinite(e) if mask is None else (np.asarray(mask, bool) & np.isfinite(e))
    return float(np.sum(e[sel]) * (t[1] - t[0])) if sel.any() else float("nan")


def torque_rms_nm(torque, mask=None):
    """RMS of the torque about its own mean, Nm: how hard the governor works
    the motor beyond carrying the mean load (actuator effort)."""
    x = np.asarray(torque, dtype=float)
    sel = np.isfinite(x) if mask is None else (np.asarray(mask, bool) & np.isfinite(x))
    return float(np.std(x[sel])) if sel.sum() > 1 else float("nan")


def ring_after_step(t, speed, t_step, window_s=5.0, settle_s=1.0, min_rpm=2.0):
    """Ring period and decay after a step at t_step, on a clean speed signal.

    The speed is taken about its median over the last settle_s of the window.
    Each half-cycle between crossings of that level (with hysteresis) gives
    one peak. Returns (period_s, decay, n_peaks): period from the spacing of
    the first peaks, and decay as the size of each peak over the one a full
    cycle before it (the plan's "each cycle keeps 0.65 of the last"). With a
    single peak and no return swing, the decay is 0 and the period NaN.
    """
    t = np.asarray(t, dtype=float)
    v = np.asarray(speed, dtype=float)
    sel = (t >= t_step) & (t <= t_step + window_s) & np.isfinite(v)
    ts, vs = t[sel], v[sel]
    if len(vs) < 10:
        return float("nan"), float("nan"), 0
    ref = float(np.median(vs[ts >= ts[-1] - settle_s]))
    e = vs - ref
    h = max(min_rpm, 0.05 * float(np.max(np.abs(e))))
    peaks, times = [], []
    sign, best, best_t = 0, 0.0, 0.0
    for ti, ei in zip(ts, e):
        s = 1 if ei > h else (-1 if ei < -h else 0)
        if s and s != sign:
            if sign:
                peaks.append(best)
                times.append(best_t)
            sign, best, best_t = s, ei, ti
        elif s == sign and abs(ei) > abs(best):
            best, best_t = ei, ti
    if sign:
        peaks.append(best)
        times.append(best_t)
    n = len(peaks)
    if n < 2:
        return float("nan"), (0.0 if n == 1 else float("nan")), n
    use = min(n, 6)
    period = float(2.0 * np.median(np.diff(times[:use])))
    if n < 3:
        return period, 0.0, n
    ratios = [abs(peaks[i + 2]) / abs(peaks[i]) for i in range(min(n - 2, 4))]
    return period, float(np.median(ratios)), n


def phase_margin_deg(decay):
    """Phase margin estimated from the per-cycle decay of the ring, for a loop
    whose closed-loop response is dominated by one second-order pair.
    Decay 0.65 gives about 8 degrees and 0.35 about 19 (M4 bench)."""
    if not np.isfinite(decay):
        return float("nan")
    if decay <= 0.0:
        return 90.0
    if decay >= 1.0:
        return 0.0
    ld = np.log(decay)
    zeta = -ld / np.sqrt(4 * np.pi ** 2 + ld ** 2)
    return float(np.degrees(np.arctan(2 * zeta / np.sqrt(np.sqrt(1 + 4 * zeta ** 4) - 2 * zeta ** 2))))


# Weights for cost(): rpm of mean absolute error, rpm of per-stroke ripple and
# Nm of torque RMS count about equally on the placeholder baler. A starting
# point, to be set with Ben; the hard limits (ring decay, overshoot) are
# checked separately, not folded in here.
COST_WEIGHTS = {"load_mae_rpm": 1.0, "stroke_p2p_rpm": 0.25, "load_treq_rms_nm": 0.5}


def cost(m: dict, weights: dict | None = None) -> float:
    """One number to rank runs by, lower is better, shared by sweep.py, the
    software-in-the-loop harness and the gain search (plan section 16). A
    weighted sum of the metrics in m named by weights; NaN if any is missing.
    Ripple only counts when the run has strokes: without a period it is 0."""
    w = COST_WEIGHTS if weights is None else weights
    total = 0.0
    for k, wk in w.items():
        v = m.get(k, float("nan"))
        if k == "stroke_p2p_rpm" and not np.isfinite(m.get("stroke_period_s", float("nan"))):
            v = 0.0
        if v is None or not np.isfinite(v):
            return float("nan")
        total += wk * float(v)
    return float(total)
