"""Re-runs bench sweeps through the software-in-the-loop harness and compares
the tables (plan section 16, M5c acceptance: stroke ripple within 15 %).

Each argument is a folder from docs/bench/results/: its sweep YAML is run
through the harness with the pedal at the bench's mean pedal_pct, and the
harness's metrics.csv is compared, set by set, with the bench's
metrics_rescored.csv (or metrics.csv). Bench sets whose speed noise differs
are compared but not judged: the bench firmware before M5b did not restart
its noise generator on preset, so its noise sequence cannot be reproduced.

    python sil_check.py ../docs/bench/results/2026-11-03_sweep_*_baler --out D:/runs/sil_check

With --loopcalc it instead checks tools/loopcalc.py against the harness: for
each case in LOOPCALC_CASES, a 20 Nm load step at no load is put on and taken
off, and the ring after it comes off is measured on the model speed. The
cases are P and PD only. With govKi above 0 the governor is not linear at
revlim: over the limit it clamps its output to zero or below, so an integral
carrying load turns into a relay there and the loop settles into a small
limit cycle that no linear model describes (see docs/bench/M5c_sil.md).

    python sil_check.py --loopcalc --out D:/runs/sil_loopcalc
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import loglib
import loopcalc
import metrics as mt
import sweep
from silport import SilError, SilPort

COLUMNS = ["stroke_p2p_rpm", "treq_stroke_p2p_nm", "noload_p2p_rpm", "load_rms_err_rpm",
           "peak_integral_pct", "cost"]
JUDGED = "stroke_p2p_rpm"
TOL = 0.15


def bench_table(folder: Path) -> pd.DataFrame:
    for name in ("metrics_rescored.csv", "metrics.csv"):
        if (folder / name).exists():
            return pd.read_csv(folder / name)
    raise SilError(f"{folder}: no metrics.csv")


def run_folder(folder: Path, out: Path, zombie=None, log=print) -> pd.DataFrame:
    yamls = sorted(folder.glob("*.yaml"))
    if not yamls:
        raise SilError(f"{folder}: no sweep YAML")
    s = sweep.load_sweep(yamls[0].read_text())
    bench = bench_table(folder)
    pedal = float(np.nanmean(bench["pedal_pct"])) if "pedal_pct" in bench else 100.0
    # The bench's loop delay depended on the clock phase (loop_ticks: 3, or 4,
    # or between when the phase drifted through the boundary during the run).
    # The harness is always 3; each extra tick is 10 ms of rptdelay.
    if "loop_ticks" in bench and len(bench) == len(s["sets"]):
        for st, lt in zip(s["sets"], bench["loop_ticks"]):
            extra = float(np.clip(round((lt - 3.0) * 10.0), 0, 50)) if np.isfinite(lt) else 0.0
            if extra > 0:
                st["sim"] = {**(st.get("sim") or {}), "rptdelay": extra}
    out.mkdir(parents=True, exist_ok=True)
    quiet = lambda msg: None  # noqa: E731
    with SilPort(zombie, pedal=pedal, log=log) as port:
        rows = sweep.run_sweep(s, sweep.Console(port, log=quiet, clock=port.clock, sleep=port.sleep),
                               out, log=quiet)
    sweep.write_table(rows, out / "metrics.csv")
    (out / "sil_info.txt").write_text(port.banner + "\n")
    return pd.DataFrame(rows)


RING_TOL = 0.10
RESEEDING_FROM = "2026-11-10"  # M5b: the bench restarts its noise on preset and mark from here


def compare(name: str, bench: pd.DataFrame, sil: pd.DataFrame, log=print) -> bool:
    """Judges stroke ripple (within TOL) on every set, and the ring period
    after the load comes off (within RING_TOL) wherever the bench measured a
    ring that keeps at least RING_KEEP_MIN of each cycle. Sets with speed
    noise are judged only for runs from RESEEDING_FROM on (by folder date)."""
    ok = True
    log(f"\n{name}")
    gains = [c for c in ("govKp", "govKi", "govKd", "govFlt", "loop_ticks") if c in bench]
    noisy = [c for c in bench if c.startswith("sim_noise")]
    noise_repeatable = name[:10] >= RESEEDING_FROM
    cols = COLUMNS + [c for c in ("ring_period_s", "ring_decay") if c in bench and bench[c].notna().any()]
    head = f"{'set':>3} " + " ".join(f"{g[:6]:>6}" for g in gains) + "".join(f"  {c[:18]:>21}" for c in cols)
    log(head)
    for i in range(min(len(bench), len(sil))):
        b, s = bench.iloc[i], sil.iloc[i]
        has_noise = any(pd.notna(b.get(c)) and b.get(c) not in ("", 0) for c in noisy)
        judged = noise_repeatable or not has_noise
        cells = []
        for c in cols:
            x, y = float(b.get(c, np.nan)), float(s.get(c, np.nan))
            fmt = "8.3f" if c.startswith("ring") else "8.1f"
            cells.append(f"  {x:{fmt}}/{y:{fmt}} {(y / x - 1) if x else np.nan:+5.0%}")
        x, y = float(b.get(JUDGED, np.nan)), float(s.get(JUDGED, np.nan))
        good = bool(np.isfinite(x) and np.isfinite(y) and abs(y / x - 1) <= TOL)
        rb = float(b.get("ring_period_s", np.nan))
        rs = float(s.get("ring_period_s", np.nan))
        kb = float(b.get("ring_decay", np.nan))
        if np.isfinite(rb) and np.isfinite(kb) and kb >= RING_KEEP_MIN:
            good &= bool(np.isfinite(rs) and abs(rs / rb - 1) <= RING_TOL)
        if judged:
            ok &= good
        mark = ("" if good else "  <-") if judged else "  (noise before M5b: not judged)"
        log(f"{int(b['set']):>3} " + " ".join(f"{b[g]:>6.3g}" for g in gains) + "".join(cells) + mark)
    return ok


# (govKp, govKd, govFlt, J, extra delay ms): no-load P and PD loops across
# the inertia range and the bench's two loop delays
LOOPCALC_CASES = [(11, 0, 4, 0.3, 0), (11, 0, 3, 0.3, 0), (11, 0, 2, 0.3, 0), (11, 0, 4, 0.3, 10),
                  (6, 0, 2, 0.3, 0), (11, 0, 4, 0.9, 0), (11, 0, 2, 0.9, 0), (25, 0, 4, 2.5, 0),
                  (40, 0, 2, 2.5, 0), (11, 1, 2, 0.3, 0)]
STEP_NM = 20.0


def harness_ring(port: SilPort, case, out: Path, log=print):
    kp, kd, flt, j, delay = case
    con = sweep.Console(port, log=lambda m: None, clock=port.clock, sleep=port.sleep)
    con.cmd("preset default")
    con.cmd(f"set J {j:g}")
    con.cmd(f"set rptdelay {delay:g}")
    for n, v in (("govKi", 0), ("govKp", kp), ("govKd", kd), ("govFlt", flt), ("GovLog", 1)):
        con.cmd(f"zset {n} {v:g}")
    # With govKi 0 the Zombie keeps whatever integral it had (plan G1); one
    # left over puts the loop on the relay at revlim. Clear it as sweep.py does.
    imax = con.zget("govImax")
    con.cmd("zset govImax 0")
    con.pump(sweep.INT_CLEAR_S)
    con.cmd(f"zset govImax {imax:g}")
    con.pump(4.0)
    with open(out, "w") as f:
        con.canlog = f
        con.cmd("canlog 1")
        con.cmd(f"load step {STEP_NM:g}")
        con.pump(LOADED_S)  # long enough for the lightest-damped case to settle
        con.cmd("mark 1")
        con.cmd("load off")
        con.pump(5.0)
        con.cmd("canlog 0")
        con.canlog = None
    lg = loglib.load(out)
    t = lg.grid.index.to_numpy()
    v = lg.grid[sweep.SIM_SPEED].to_numpy(dtype=float)
    ev = lg.raw["SIM_Event_4B2"]
    t_off = float(ev["t"][ev["Event"] == 1].iloc[-1])
    i0 = int(np.searchsorted(t, t_off))
    base = float(np.mean(v[(t >= t_off - 0.5) & (t < t_off - 0.05)]))  # loaded steady state
    dv = v[i0 - 2:i0 + RESPONSE_TICKS] - base  # from 2 ticks early, for the alignment search
    return dv, mt.ring_after_step(t, v, t_off, window_s=4.5)


RESPONSE_TICKS = 300
LOADED_S = 25.0
SHAPE_TOL = 0.10   # RMS difference over the peak deviation
RING_KEEP_MIN = 0.25  # below this a ring has too few peaks for a period


def response_shape(dv, lin) -> float:
    """RMS difference between the harness's response (dv, from two ticks
    before the step) and loopcalc's (lin), over lin's peak. The log's grid
    may sit a tick or two either side of the model's, so the best of five
    alignments is taken."""
    errs = [float(np.sqrt(np.mean((dv[s:s + len(lin)] - lin) ** 2)))
            for s in range(5) if s + len(lin) <= len(dv)]
    return min(errs) / float(np.max(np.abs(lin)))


def check_loopcalc(out: Path, zombie=None, log=print) -> bool:
    """Each case: the harness's response to the load coming off against
    loopcalc's linear response (shape), and where there is a ring to measure
    (keep at least RING_KEEP_MIN) its period against loopcalc's pole."""
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    log(f"{'govKp':>5} {'govKd':>5} {'govFlt':>6} {'J':>4} {'+ms':>3}   {'harness ring':>12} {'keep':>5}"
        f"   {'loopcalc':>8} {'keep':>5}   {'period':>6}   {'shape':>5}")
    with SilPort(zombie, log=log) as port:
        for i, case in enumerate(LOOPCALC_CASES):
            dv, (period, keep, n) = harness_ring(port, case, out / f"loopcalc_{i:02d}_bus.log")
            kp, kd, flt, j, delay = case
            lc = loopcalc.analyse(kp=kp, kd=kd, flt=flt, J=j, delay=delay)
            lin = loopcalc.load_step_response(-STEP_NM, RESPONSE_TICKS, kp=kp, kd=kd, flt=flt, J=j, delay=delay)
            shape = response_shape(dv, lin)
            good = shape <= SHAPE_TOL
            judged = lc["ring_keep"] >= RING_KEEP_MIN
            diff = period / lc["ring_period_s"] - 1
            if judged:
                good &= bool(np.isfinite(diff) and abs(diff) <= 0.10)
            ok &= good
            log(f"{kp:>5g} {kd:>5g} {flt:>6g} {j:>4g} {delay:>3g}   {period:10.3f} s {keep:5.2f}"
                f"   {lc['ring_period_s']:6.3f} s {lc['ring_keep']:5.2f}   "
                + (f"{diff:+6.1%}" if judged else f"{'-':>6}") + f"   {shape:5.1%}" + ("" if good else "  <-"))
    log(f"\n{'PASS' if ok else 'FAIL'}: response within {SHAPE_TOL:.0%} RMS of loopcalc's in every case, "
        f"and ring period within 10 % wherever each cycle keeps at least {RING_KEEP_MIN}")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="*", help="docs/bench/results/<date>_<sweep>/ folders")
    ap.add_argument("--loopcalc", action="store_true", help="check loopcalc.py against the harness instead")
    ap.add_argument("--out", required=True, help="folder for the harness's logs and tables")
    ap.add_argument("--zombie", help="Stm32-vcu tree (default ZOMBIE_DIR or ../Stm32-vcu)")
    a = ap.parse_args(argv)
    if a.loopcalc:
        try:
            return 0 if check_loopcalc(Path(a.out), a.zombie) else 1
        except (SilError, sweep.SweepError) as e:
            print(f"error: {e}", file=sys.stderr)
            return 1
    if not a.folders:
        ap.error("give result folders, or --loopcalc")
    ok = True
    try:
        for f in map(Path, a.folders):
            sil = run_folder(f, Path(a.out) / f.name, a.zombie)
            ok &= compare(f.name, bench_table(f), sil)
    except (SilError, sweep.SweepError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"\n{'PASS' if ok else 'FAIL'}: {JUDGED} within {TOL:.0%} of the bench on every judged set "
          "(each cell: bench/harness and the difference)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
