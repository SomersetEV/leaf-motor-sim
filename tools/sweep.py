"""Run a list of Zombie governor gain sets against a load, unattended (plan M5a).

For each set: put the load back to no-load, start the bus log (canlog), write
the gains with zset, hold no-load for settle_s, apply the load (a preset or
an SD profile) for run_s, and score both phases: no-load peak and stability,
then mean, droop, range and per-stroke ripple under load. The gains
the Zombie had at the start are written back at the end, also after an
error. GovLog is turned on for the sweep (it is needed for scoring) and
restored with the gains. Nothing is saved on the Zombie: a power cycle
restores its setup.

A set may also hold `sim:` model parameters for the simulator, such as
`sim: {noise: 5}` for speed noise on 0x1DA. Presets reset the model, so they
are applied again after every load change, and put back by preset noload.

The pedal must be held at a fixed position for the whole sweep (a clamp or a
fixed pot). Each row reports the mean pedal position and the share of time
the governor was the binding limit, so a slipped pedal shows up.

    python sweep.py sweep.yaml --port COM3 --out D:/runs/sweep1

Example sweep.yaml:

    load: {preset: baler}          # or {profile: baler_r3}  (SD /profiles/)
    settle_s: 8                    # no-load time before each run
    run_s: 30
    base: {govKi: 0, govKd: 0}     # applied before each set's own values
    sets:
      - {govKp: 8}
      - {govKp: 11}
      - {govKp: 14}
      - {govKp: 11, govKi: 2}
      - {govKp: 11, sim: {noise: 5}}   # simulator 'set' parameters
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import yaml

import loglib
import metrics as mt
from run_script import LinePort

ZPARAMS = ("revlim", "govKp", "govKi", "govKd", "govImax", "govFlt")  # gains a set may change
POT_USER = "GOV_Speed_4A0.PotUser"
SIM_TREQ = "SIM_Fast_4B0.TorqueReq"
SIM_SPEED = "SIM_Fast_4B0.ModelSpeed"  # shaft speed before any reported-speed noise


class SweepError(Exception):
    pass


class Console:
    """The simulator's console: commands with ok/err replies, while canlog
    and stream lines go to whichever files are open."""

    def __init__(self, port, log=print):
        self.port = port
        self.log = log
        self.canlog = None

    def _line(self):
        raw = self.port.readline()
        if not raw:
            return None
        line = raw.decode("ascii", errors="backslashreplace").strip()
        if not line:
            return None
        if line[0] == "(":
            if self.canlog:
                self.canlog.write(line + "\n")
            return None
        if line[0].isdigit() or line.startswith("t_ms,"):
            return None  # CSV stream, not used by the sweep
        return line

    def pump(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._line() is None:
                time.sleep(0.002)

    def cmd(self, c: str, timeout_s: float = 3.0) -> str:
        self.port.write((c + "\n").encode())
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            line = self._line()
            if line and (line.startswith("ok") or line.startswith("err")):
                self.log(f"    > {c}  < {line[:100]}")
                if line.startswith("err"):
                    raise SweepError(f"'{c}': {line}")
                return line
        raise SweepError(f"'{c}': no reply")

    def zget(self, name: str) -> float:
        return float(self.cmd(f"zget {name}").split("=", 1)[1])


def load_sweep(text: str) -> dict:
    s = yaml.safe_load(text) or {}
    if not s.get("sets"):
        raise SweepError("sweep needs a non-empty 'sets' list")
    load = s.get("load") or {}
    if not (("preset" in load) ^ ("profile" in load)):
        raise SweepError("load needs exactly one of 'preset' or 'profile'")
    for i, g in enumerate([s.get("base") or {}] + list(s["sets"])):
        bad = [k for k in g if k not in ZPARAMS and k != "sim"]
        if bad:
            raise SweepError(f"set {i}: not a governor parameter: {', '.join(bad)}")
    return s


def apply_load(con: Console, load: dict):
    if "preset" in load:
        con.cmd(f"preset {load['preset']}")
    else:
        con.cmd(f"profile load {load['profile']}", timeout_s=30.0)
        con.cmd("profile start")


def apply_sim(con: Console, sim: dict):
    for n, v in sim.items():
        con.cmd(f"set {n} {v}")


def clear_load(con: Console, load: dict):
    if "profile" in load:
        try:
            con.cmd("profile stop")
        except SweepError:
            pass  # already finished
    con.cmd("preset noload")


LOAD_START_EVENTS = (3, 5)  # 0x4B2 ProfileStart, Preset
LOAD_SKIP_S = 5.0            # load-phase metrics start this long after the load is applied
NOLOAD_TAIL_S = 3.0          # no-load stability is judged over the last few seconds


def _stroke_p2p(t, v, period_s):
    """Median peak-to-peak speed over consecutive one-stroke windows."""
    if not np.isfinite(period_s) or period_s <= 0 or len(t) < 2:
        return float("nan")
    n = max(2, int(round(period_s / (t[1] - t[0]))))
    p2p = [np.ptp(v[i:i + n]) for i in range(0, len(v) - n + 1, n) if np.isfinite(v[i:i + n]).all()]
    return float(np.median(p2p)) if p2p else float("nan")


def score(bus_log: Path, revlim: float) -> dict:
    """Scores one run in two phases: no-load after the gain change, and the load
    from LOAD_SKIP_S after it was applied. compare_runs' tractor metrics are
    not used directly: their 90 % of revlim mask drops every baler stroke."""
    log = loglib.load(bus_log)
    g = log.grid
    t = g.index.to_numpy()
    v = np.abs(loglib.speed_rpm(log).to_numpy(dtype=float))
    ev = log.raw.get("SIM_Event_4B2")
    starts = ev["t"][ev["Event"].isin(LOAD_START_EVENTS)].to_numpy() if ev is not None else []
    t_load = float(starts[-1]) if len(starts) else 0.0

    nl = (t >= t_load - NOLOAD_TAIL_S) & (t < t_load) & np.isfinite(v)
    nl_all = (t < t_load) & np.isfinite(v)
    ld = (t >= t_load + LOAD_SKIP_S) & np.isfinite(v)
    m = {"revlim_rpm": revlim}
    m["noload_peak_rpm"] = float(v[nl_all].max()) if nl_all.any() else float("nan")
    m["noload_p2p_rpm"] = float(np.ptp(v[nl])) if nl.any() else float("nan")
    if log.has(SIM_TREQ) and nl.any():
        m["noload_treq_sd_nm"] = float(np.nanstd(g[SIM_TREQ].to_numpy(dtype=float)[nl]))
    if ld.any():
        tl, vl = t[ld], v[ld]
        period = mt.dominant_period_s(tl, vl)
        m["load_mean_rpm"] = float(vl.mean())
        m["droop_rpm"] = revlim - m["load_mean_rpm"]
        m["load_min_rpm"] = float(vl.min())
        m["load_max_rpm"] = float(vl.max())
        m["stroke_period_s"] = period
        m["stroke_p2p_rpm"] = _stroke_p2p(tl, vl, period)
        if log.has(SIM_TREQ):
            treq = g[SIM_TREQ].to_numpy(dtype=float)[ld]
            m["treq_stroke_p2p_nm"] = _stroke_p2p(tl, treq, period)
        if log.has(SIM_SPEED):
            ws = np.abs(g[SIM_SPEED].to_numpy(dtype=float))[ld]
            m["model_stroke_p2p_rpm"] = _stroke_p2p(tl, ws, period)
    m["peak_integral_pct"] = mt.peak_abs(g[loglib.GOV_INT]) if log.has(loglib.GOV_INT) else float("nan")
    m["pedal_pct"] = float(np.nanmean(g[POT_USER].to_numpy(dtype=float)[ld])) if log.has(POT_USER) else float("nan")
    m["binding_frac"] = (float(np.nanmean(g[loglib.GOV_BINDING].to_numpy(dtype=float)[ld] > 0.5))
                         if log.has(loglib.GOV_BINDING) else float("nan"))
    m["dropped_4a1"] = log.dropped_4a1
    return m


def run_sweep(s: dict, con: Console, out: Path, log=print) -> list[dict]:
    load = s["load"]
    settle_s = float(s.get("settle_s", 8.0))
    run_s = float(s.get("run_s", 30.0))
    base = dict(s.get("base") or {})
    base_sim = base.pop("sim", None) or {}
    names = sorted({k for g in [base] + list(s["sets"]) for k in g if k != "sim"})
    sim_names = sorted({k for g in [{"sim": base_sim}] + list(s["sets"]) for k in (g.get("sim") or {})})

    con.cmd("stream 0")
    con.cmd("canlog 0")
    original = {n: con.zget(n) for n in sorted(set(names) | {"revlim", "GovLog"})}
    log(f"Zombie at start: {original}")
    rows = []
    try:
        con.cmd("zset GovLog 1")  # governor frames 0x4A0-0x4A3, needed for scoring
        for i, overrides in enumerate(s["sets"], 1):
            overrides = dict(overrides)
            sim = {**base_sim, **(overrides.pop("sim", None) or {})}
            gains = {**base, **overrides}
            log(f"set {i}/{len(s['sets'])}: {gains}" + (f" sim {sim}" if sim else ""))
            clear_load(con, load)
            apply_sim(con, sim)
            con.pump(NOLOAD_TAIL_S)  # let the previous load's removal die away
            bus = out / f"run_{i:02d}_bus.log"
            with open(bus, "w") as f:
                con.canlog = f
                con.cmd("canlog 1")
                con.cmd(f"mark {i}")
                for n, v in gains.items():
                    con.cmd(f"zset {n} {v}")
                con.pump(settle_s)  # no-load phase, scored for stability
                apply_load(con, load)
                apply_sim(con, sim)
                con.pump(run_s)
                con.cmd("canlog 0")
                con.pump(0.2)  # lines already queued
                con.canlog = None
            clear_load(con, load)
            row = {"set": i, **{n: gains.get(n, original[n]) for n in names},
                   **{f"sim_{n}": sim.get(n, "") for n in sim_names}}
            try:
                row.update(score(bus, float(gains.get("revlim", original["revlim"]))))
            except Exception as e:  # keep the sweep going; the log is kept
                log(f"    scoring failed: {e}")
            if not row.get("pedal_pct", 0) >= 5.0:
                log(f"    WARNING: pedal at {row.get('pedal_pct', float('nan')):.1f} %; "
                    "this run is not meaningful (is the pedal held?)")
            rows.append(row)
            log("    " + ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                                   for k, v in row.items() if k not in names and k != "set" and not k.startswith("sim_")))
    finally:
        con.canlog = None
        if sim_names:
            try:
                clear_load(con, load)  # preset noload puts the model back
            except SweepError as e:
                log(f"WARNING: could not reset the model: {e}")
        for n, v in original.items():
            try:
                con.cmd(f"zset {n} {v}")
            except SweepError as e:
                log(f"WARNING: could not restore {n}={v}: {e}")
    return rows


def write_table(rows: list[dict], path: Path):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sweep")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", required=True, help="folder for bus logs and metrics.csv")
    a = ap.parse_args(argv)

    s = load_sweep(open(a.sweep).read())
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    import serial  # pyserial

    sp = serial.Serial(baudrate=a.baud, timeout=0.02)
    sp.port = a.port
    sp.dtr = False  # released, so opening cannot reset the ESP32
    sp.rts = False
    sp.open()
    with sp:
        sp.reset_input_buffer()
        try:
            rows = run_sweep(s, Console(LinePort(sp)), out)
        except SweepError as e:
            print(f"stopped: {e}", file=sys.stderr)
            return 1
    write_table(rows, out / "metrics.csv")
    print(f"done: {len(rows)} sets, table in {out / 'metrics.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
