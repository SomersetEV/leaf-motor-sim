"""Replays a bench log through the software-in-the-loop harness and compares
the two runs (plan section 16, M5c acceptance).

From the bench log it takes what the harness cannot know: the pedal
(PotUser in 0x4A0), the Zombie's governor settings (0x4A2, 0x4A3) and the
model inertia (0x4B1). It starts the harness at rest at the first moment the
bench shaft is also at rest with GovLog on, and drives it tick by tick with
the same inputs. Load commands are not in the log, so the bench run must be
at no load (preset default, which was noload until 14 Nov 2026), as the M4 pedal tests were.

It then finds every approach to revlim (the speed rising through 98 % of it
from below half of it) and measures the ring that follows, in both runs, with
metrics.ring_after_step on the model speed (0x4B0). Ring period within 10 %
is the acceptance test.

    python sil_replay.py "M4 second logs.csv" --out m4_sil.log
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import loglib
import metrics as mt
from silport import SilError, SilPort

POT = "GOV_Speed_4A0.PotUser"
MODEL = "SIM_Fast_4B0.ModelSpeed"
GAINS = {"revlim": "GOV_Gains_4A2.Revlim", "govKp": "GOV_Gains_4A2.GovKp",
         "govKi": "GOV_Gains_4A2.GovKi", "govKd": "GOV_Gains_4A2.GovKd",
         "govImax": "GOV_State_4A3.GovImax", "govFlt": "GOV_State_4A3.GovFlt"}
INERTIA = "SIM_Slow_4B1.Inertia"
RING_WINDOW_S = 4.0
PERIOD_TOL = 0.10


def _first_tick_using(bench: loglib.Log, name: str, new: float, old: float, t_seen: float):
    """The slow frames (0x4A2, 0x4A3) report a change up to 100 ms late. For
    govFlt and govKp the fast frames show the exact tick: the filter output
    (revlim - GovSpeedErr) follows its shift, and GovP = error x govKp / 100.
    Returns the time of the first 0x4A0 that used the new value, or t_seen."""
    a0, a1 = bench.raw.get("GOV_Speed_4A0"), bench.raw.get("GOV_Terms_4A1")
    if a0 is None or a1 is None or name not in ("govFlt", "govKp"):
        return t_seen
    n = min(len(a0), len(a1))
    t = a0["t"].to_numpy()[:n]
    sel = np.where((t > t_seen - 0.2) & (t <= t_seen + 0.01))[0]
    err = a0["GovSpeedErr"].to_numpy(dtype=float)[:n]
    speed_in = a0["GovSpeedIn"].to_numpy(dtype=float)[:n]
    revlim = float(np.interp(t_seen, bench.grid.index, bench.grid[GAINS["revlim"]].ffill().bfill()))
    for k in sel[1:]:
        if name == "govFlt":
            f_prev, f_now = revlim - err[k - 1], revlim - err[k]
            x = speed_in[k]
            fits = {s: abs(f_prev + (x - f_prev) / (1 << int(s)) - f_now) for s in (new, old)}
            if abs(x - f_prev) > 8 and fits[new] < 0.3 and fits[old] > 0.3:
                return float(t[k])
        else:
            p = float(a1["GovP"].iloc[k])
            if abs(err[k]) > 20 and abs(p / err[k] * 100 - new) < abs(p / err[k] * 100 - old):
                return float(t[k])
    return t_seen


def replay(bench: loglib.Log, out_path: str, zombie=None, log=print) -> float:
    """Runs the harness on the bench log's inputs; writes its bus log to
    out_path. Returns the bench time of the harness's tick 0."""
    g = bench.grid
    t = g.index.to_numpy()
    pot = g[POT].ffill().to_numpy(dtype=float)
    gains = {k: g[c].ffill().to_numpy(dtype=float, copy=True) for k, c in GAINS.items() if c in g}
    for k, arr in gains.items():  # move each change to the tick it took effect
        for i in np.where(np.diff(arr) != 0)[0] + 1:
            if not (np.isfinite(arr[i]) and np.isfinite(arr[i - 1])):
                continue
            te = _first_tick_using(bench, k, arr[i], arr[i - 1], float(t[i]))
            j = int(np.searchsorted(t, te - 0.005))
            if j < i:
                log(f"  {k} {arr[i - 1]:g} -> {arr[i]:g} took effect at {te:.3f} s, not {t[i]:.3f} s")
                arr[j:i] = arr[i]
    model = np.abs(g[MODEL].to_numpy(dtype=float))
    ok = np.isfinite(pot) & np.isfinite(model) & (model < 1.0)
    for c in gains.values():
        ok &= np.isfinite(c)
    if not ok.any():
        raise SilError("no moment with GovLog on and the shaft at rest to start from")
    i0 = int(np.argmax(ok))
    log(f"replaying from bench t={t[i0]:.2f} s to {t[-1]:.2f} s")
    with SilPort(zombie, pedal=0.0, warmup_s=0.0, log=log) as port, open(out_path, "w") as f:
        log(port.banner)

        def cmd(c):
            port.write((c + "\n").encode())
            for line in list(port.q):
                if not line.startswith(b"("):
                    text = line.decode().strip()
                    if text.startswith("err"):
                        raise SilError(f"'{c}': {text}")
            drain()

        def drain():
            while port.q:
                line = port.q.popleft()
                if line.startswith(b"("):
                    f.write(line.decode())

        if INERTIA in g and np.isfinite(g[INERTIA].iloc[i0:]).any():
            j = float(g[INERTIA].iloc[i0:].dropna().iloc[0])
            cmd(f"set J {j:g}")
        cmd("zset GovLog 1")  # as on the bench: governor frames in the log
        cmd("canlog 1")
        last = {}
        for i in range(i0, len(t)):
            for k, arr in gains.items():
                v = float(arr[i])
                if np.isfinite(v) and last.get(k) != v:
                    cmd(f"zset {k} {v:g}")
                    last[k] = v
            if np.isfinite(pot[i]) and last.get("pedal") != round(pot[i], 2):
                last["pedal"] = round(pot[i], 2)
                cmd(f"pedal {last['pedal']:g}")
            port.sleep(0.01)
            drain()
        cmd("canlog 0")
    return float(t[i0])


def approaches(t, v, revlim):
    """Times the speed rises through 98 % of revlim after being below half."""
    out, armed = [], False
    for ti, vi in zip(t, v):
        if not np.isfinite(vi):
            continue
        if vi < revlim / 2:
            armed = True
        elif armed and vi >= 0.98 * revlim:
            out.append(float(ti))
            armed = False
    return out


def compare(bench: loglib.Log, sil: loglib.Log, t0: float, log=print) -> bool:
    gb, gs = bench.grid, sil.grid
    tb = gb.index.to_numpy()
    ts = gs.index.to_numpy() + t0
    vb = np.abs(gb[MODEL].to_numpy(dtype=float))
    vs = np.abs(gs[MODEL].to_numpy(dtype=float))
    revlim = float(gb[GAINS["revlim"]].dropna().iloc[-1])
    flt = gb[GAINS["govFlt"]].ffill().to_numpy(dtype=float)
    # Whole-run agreement on the common grid
    common = np.interp(ts, tb, vb)
    d = vs - common
    log(f"speed, harness minus bench: RMS {np.sqrt(np.nanmean(d ** 2)):.1f} rpm over {ts[-1] - ts[0]:.0f} s")
    ok = True
    rows = 0
    log(f"{'approach':>9} {'govFlt':>6}   {'bench period':>12} {'sil period':>10} {'diff':>6}"
        f"   {'bench keep':>10} {'sil keep':>8}")
    for ta in approaches(tb[tb >= t0], vb[tb >= t0], revlim):
        k = int(np.searchsorted(tb, ta))
        pb, db, nb = mt.ring_after_step(tb, vb, ta, window_s=RING_WINDOW_S)
        sil_cross = approaches(ts[(ts >= ta - 0.5)], vs[(ts >= ta - 0.5)], revlim)
        if not sil_cross:
            log(f"{ta:9.2f}  the harness never reached revlim here")
            ok = False
            continue
        p_s, d_s, n_s = mt.ring_after_step(ts, vs, sil_cross[0], window_s=RING_WINDOW_S)
        if not (np.isfinite(pb) and nb >= 3):
            continue  # no ring on the bench to compare (pedal let go too soon)
        rows += 1
        diff = p_s / pb - 1
        ok &= abs(diff) <= PERIOD_TOL
        log(f"{ta:9.2f} {flt[k]:6.0f}   {pb:10.3f} s {p_s:8.3f} s {diff:+6.1%}   {db:10.2f} {d_s:8.2f}")
    if not rows:
        log("no ring on the bench to compare")
        return False
    log("PASS" if ok else "FAIL", f"(ring period within {PERIOD_TOL:.0%})")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="bench log with GovLog frames, at no load")
    ap.add_argument("--out", required=True, help="the harness's bus log (candump)")
    ap.add_argument("--zombie", help="Stm32-vcu tree (default ZOMBIE_DIR or ../Stm32-vcu)")
    a = ap.parse_args(argv)
    try:
        bench = loglib.load(a.log)
        t0 = replay(bench, a.out, a.zombie)
        return 0 if compare(bench, loglib.load(a.out), t0) else 1
    except SilError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
