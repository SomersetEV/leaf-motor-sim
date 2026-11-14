"""Loop calculator for the Zombie's RPM governor against the simulated shaft
(plan section 16, "Loop model").

A linear, discrete-time model of the loop exactly as the bench and the
software-in-the-loop harness run it, tick by tick:

  plant    ten 1 ms sub-steps per 10 ms tick, as lib/plant: the torque lag
           (exact for a held command), then the inertia with viscous drag
           linearised at revlim (b + 2 c w). Tc has no effect on small signals.
  sensor   the end-of-tick speed, rptdelay ms older (to the nearest 1 ms).
  delay    3 ticks from a speed sample to the tick that applies the torque
           worked out from it: the Zombie uses the speed it read at the end of
           its previous tick, sends the result in its next tick's 0x1D4, and
           the simulator latches it at its following tick (measured on the
           3 Nov bench logs; at some clock phases it is 4, so try --delay 10).
  governor Throttle::SpeedLimitCommand: speed filter shift govFlt, P on the
           filtered error, I summed each tick, D on the filtered slope (fixed
           shift 4), in percent; percent x 2047/100 x kT is newton metres.

It ignores everything nonlinear: the torque limit, regenmax, the integral's
reset, freeze and clamp, whole-rpm rounding, and the pedal. So it describes
the governor while it is the binding limit and the torque stays inside
regenmax and Tmax.

It prints the crossover, the phase and gain margins of the loop gain, and the
dominant closed-loop pole as a ring period and the share of each cycle that
survives into the next (the "keep" that sweep.py measures as ring_decay).

    python loopcalc.py --kp 11 --flt 4 --J 0.3
    python loopcalc.py --kp 5,10,15,20,25 --flt 4,2 --J 2.5      # a table
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys

import numpy as np
from numpy.polynomial import polynomial as P

TICK_S = 0.01
SUBSTEPS = 10
DT_S = TICK_S / SUBSTEPS
LOOP_TICKS = 3                 # speed sample to the tick applying its torque (bench, 3 Nov)
RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)
PCT_TO_RAW = 2047.0 / 100.0    # LeafINV::SetTorque
SLOPE_SHIFT = 4                # the D-term's slope filter, fixed in SpeedLimitCommand
DEFAULTS = dict(kp=11.0, ki=0.0, kd=0.0, flt=4, J=0.3, tau=30.0, delay=0.0, revlim=2200.0,
                b=0.02, c=0.0, kt=0.25)


# ---- unit conversions (all of them, here) -------------------------------------

def kp_nm_per_rad_s(kp: float, kt: float = 0.25) -> float:
    """govKp as entered (0.01 % per rpm) in Nm per rad/s."""
    return kp * 0.01 * PCT_TO_RAW * kt * RPM_PER_RAD_S


def ti_s(kp: float, ki: float) -> float:
    """Integral time: govKp x 0.01 % per rpm over govKi x 0.001 % per rpm per tick."""
    return float("inf") if ki <= 0 else (kp * 0.01) / (ki * 0.001) * TICK_S


def filter_lag_s(shift: int) -> float:
    """Time constant of IIRFILTERF with weight 1/2^shift per tick."""
    if shift <= 0:
        return 0.0
    return -TICK_S / math.log(1.0 - 1.0 / (1 << shift))


# ---- the model ------------------------------------------------------------------

def _plant_substep(J, tau_ms, beta):
    """One lib/plant sub-step, linearised: x = [w rad/s, t_motor Nm], u = Nm."""
    d = 1.0 - math.exp(-DT_S * 1000.0 / tau_ms) if tau_ms > 0 else 1.0
    a = np.array([[1.0 - beta * DT_S / J, DT_S * (1.0 - d) / J], [0.0, 1.0 - d]])
    b = np.array([DT_S * d / J, d])
    return a, b


def _plant_tf(J, tau_ms, beta, m):
    """Torque held over tick k to the speed (rpm) after m sub-steps of tick k,
    as polynomials in x = 1/z: returns (num, den), ascending powers."""
    a1, b1 = _plant_substep(J, tau_ms, beta)
    A = np.linalg.matrix_power(a1, SUBSTEPS)
    B = sum(np.linalg.matrix_power(a1, i) @ b1 for i in range(SUBSTEPS))
    Am = np.linalg.matrix_power(a1, m)
    Bm = sum((np.linalg.matrix_power(a1, i) @ b1 for i in range(m)), np.zeros(2))
    c = np.array([RPM_PER_RAD_S, 0.0])
    den = np.array([1.0, -np.trace(A), np.linalg.det(A)])
    # adj(I - A x) = [[1 - a22 x, a12 x], [a21 x, 1 - a11 x]]
    adj = [[np.array([1.0, -A[1, 1]]), np.array([0.0, A[0, 1]])],
           [np.array([0.0, A[1, 0]]), np.array([1.0, -A[0, 0]])]]
    v = c @ Am  # row vector
    num = np.zeros(1)
    for i in range(2):
        for j in range(2):
            num = P.polyadd(num, v[i] * B[j] * adj[i][j])
    num = P.polymul(num, [0.0, 1.0])  # the x (= 1/z) in A_m x (I - A x)^-1 B
    num = P.polyadd(num, (c @ Bm) * den)
    return num, den


def _governor_tf(kp, ki, kd, flt):
    """Percent of torque per rpm of speed (negative feedback), in x = 1/z."""
    a = 1.0 / (1 << flt) if flt > 0 else 1.0
    a4 = 1.0 / (1 << SLOPE_SHIFT)
    f = (np.array([a]), np.array([1.0, -(1.0 - a)]))                 # speed filter
    terms = [(np.array([kp * 0.01]), np.array([1.0]))]
    if ki > 0:
        terms.append((np.array([ki * 0.001]), np.array([1.0, -1.0])))
    if kd > 0:
        terms.append((kd * a4 * np.array([1.0, -1.0]), np.array([1.0, -(1.0 - a4)])))
    num, den = np.zeros(1), np.array([1.0])
    for n, d in terms:  # sum of fractions
        num = P.polyadd(P.polymul(num, d), P.polymul(n, den))
        den = P.polymul(den, d)
    return P.polymul(f[0], num), P.polymul(f[1], den)


def loop_tf(p: dict):
    """The loop gain L as (num, den) in x = 1/z."""
    beta = p["b"] + 2.0 * p["c"] * p["revlim"] / RPM_PER_RAD_S
    d_sub = int(round(p["delay"] / (DT_S * 1000.0)))
    q, r = divmod(d_sub, SUBSTEPS)
    pn, pd = _plant_tf(p["J"], p["tau"], beta, SUBSTEPS - r)
    gn, gd = _governor_tf(p["kp"], p["ki"], p["kd"], int(p["flt"]))
    k = PCT_TO_RAW * p["kt"]
    delay = np.zeros(LOOP_TICKS + q + 1)
    delay[-1] = 1.0
    num = k * P.polymul(P.polymul(gn, pn), delay)
    den = P.polymul(gd, pd)
    return num, den


def analyse(**kw) -> dict:
    p = {**DEFAULTS, **kw}
    num, den = loop_tf(p)
    w = np.linspace(1e-3, math.pi / TICK_S, 20000)  # rad/s up to Nyquist
    x = np.exp(-1j * w * TICK_S)
    L = P.polyval(x, num) / P.polyval(x, den)
    mag, ph = np.abs(L), np.unwrap(np.angle(L))
    out = {k: p[k] for k in ("kp", "ki", "kd", "flt", "J", "tau", "delay")}
    cross = np.where((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
    if len(cross):
        i = cross[0]
        out["crossover_rad_s"] = float(w[i])
        out["phase_margin_deg"] = float(math.degrees(ph[i]) + 180.0)
    else:
        out["crossover_rad_s"] = out["phase_margin_deg"] = float("nan")
    m180 = np.where((ph[:-1] > -math.pi) & (ph[1:] <= -math.pi))[0]
    out["gain_margin_db"] = float(-20 * math.log10(mag[m180[0]])) if len(m180) else float("inf")
    # Closed loop: 1 + L = 0, i.e. den + num = 0 in x; poles z = 1/x
    roots = P.polyroots(P.polyadd(den, num))
    roots = roots[np.abs(roots) > 1e-9]
    z = 1.0 / roots
    osc = z[(z.imag > 1e-6)]
    out["stable"] = bool(np.all(np.abs(z) < 1.0))
    if len(osc):
        zd = osc[np.argmax(np.abs(osc))]
        theta = float(np.angle(zd))
        out["ring_period_s"] = 2 * math.pi / theta * TICK_S
        out["ring_keep"] = float(abs(zd) ** (2 * math.pi / theta))
    else:
        out["ring_period_s"] = out["ring_keep"] = float("nan")
    real = z[np.abs(z.imag) <= 1e-6].real
    slow = real[real > 0]
    out["slowest_real_s"] = float(-TICK_S / math.log(slow.max())) if len(slow) and slow.max() < 1 else float("nan")
    out["kp_nm_per_rad_s"] = kp_nm_per_rad_s(p["kp"], p["kt"])
    out["ti_s"] = ti_s(p["kp"], p["ki"])
    return out


def load_step_response(load_nm: float, ticks: int, **kw) -> np.ndarray:
    """Speed change in rpm, tick by tick, after a load torque step of load_nm
    (negative for a load taken off) on the linear loop at steady state: the
    same equations as loop_tf, in the time domain. The load acts on the
    shaft directly, not through the torque lag, as in lib/plant."""
    p = {**DEFAULTS, **kw}
    beta = p["b"] + 2.0 * p["c"] * p["revlim"] / RPM_PER_RAD_S
    a1, b1 = _plant_substep(p["J"], p["tau"], beta)
    d_sub = int(round(p["delay"] / (DT_S * 1000.0)))
    a = 1.0 / (1 << int(p["flt"])) if p["flt"] > 0 else 1.0
    a4 = 1.0 / (1 << SLOPE_SHIFT)
    k_nm = PCT_TO_RAW * p["kt"]
    x = np.zeros(2)                      # [w rad/s, t_motor Nm], deviations
    hist = [0.0] * (d_sub + 1)           # sub-step speeds, newest last, rpm
    f = f_prev = slope = integ = 0.0
    queue = [0.0] * LOOP_TICKS           # torque (Nm) for the next ticks
    out = np.zeros(ticks)
    for k in range(ticks):
        u = queue.pop(0)
        for _ in range(SUBSTEPS):
            x = a1 @ x + b1 * u
            x[0] -= load_nm / p["J"] * DT_S
            hist = hist[1:] + [x[0] * RPM_PER_RAD_S]
        out[k] = x[0] * RPM_PER_RAD_S
        y = hist[0]                      # the reported speed, d_sub ms old
        f += a * (y - f)
        slope += a4 * ((f - f_prev) - slope)
        f_prev = f
        integ += -f * p["ki"] * 0.001
        pct = -f * p["kp"] * 0.01 + integ - slope * p["kd"]
        queue.append(pct * k_nm)
    return out


COLS = [("kp", "govKp", "{:g}"), ("ki", "govKi", "{:g}"), ("kd", "govKd", "{:g}"), ("flt", "govFlt", "{:g}"),
        ("J", "J", "{:g}"), ("delay", "+ms", "{:g}"), ("crossover_rad_s", "wc rad/s", "{:.1f}"),
        ("phase_margin_deg", "PM deg", "{:.1f}"), ("gain_margin_db", "GM dB", "{:.1f}"),
        ("ring_period_s", "ring s", "{:.3f}"), ("ring_keep", "keep", "{:.2f}"), ("stable", "stable", "{}")]


def _floats(s: str):
    return [float(v) for v in s.split(",")]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for k, unit in [("kp", "govKp as entered, 0.01 %/rpm"), ("ki", "govKi as entered, 0.001 %/rpm/tick"),
                    ("kd", "govKd, % per rpm/tick"), ("flt", "govFlt shift"), ("J", "inertia, kg m2"),
                    ("tau", "motor torque lag, ms"), ("delay", "extra delay (rptdelay), ms"),
                    ("revlim", "rpm"), ("b", "viscous drag, Nm s/rad"), ("c", "quadratic drag, Nm s2/rad2"),
                    ("kt", "Nm per request bit")]:
        ap.add_argument(f"--{k}", type=_floats, default=[DEFAULTS[k]],
                        help=f"{unit.replace('%', '%%')} (default {DEFAULTS[k]:g}); a comma list makes a table")
    a = ap.parse_args(argv)
    names = list(DEFAULTS)
    grid = itertools.product(*[getattr(a, n) for n in names])
    rows = [analyse(**dict(zip(names, vals))) for vals in grid]
    print("  ".join(f"{h:>8}" for _, h, _ in COLS))
    for r in rows:
        print("  ".join(f"{f.format(r[k]):>8}" for k, _, f in COLS))
    if len(rows) == 1:
        r = rows[0]
        print(f"\ngovKp {r['kp']:g} is {r['kp_nm_per_rad_s']:.2f} Nm per rad/s; "
              f"integral time {r['ti_s']:.2f} s; speed filter lag {filter_lag_s(int(r['flt'])) * 1000:.0f} ms; "
              f"loop delay {LOOP_TICKS * 10 + r['delay']:g} ms plus the 10 ms hold")
    return 0


if __name__ == "__main__":
    sys.exit(main())
