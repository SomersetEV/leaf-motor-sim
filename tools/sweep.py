"""Run a list of Zombie governor gain sets against a load, unattended (plan M5a).

For each set: put the load back to no-load, start the bus log (canlog), write
the gains with zset, hold no-load for settle_s, apply the load (a preset or
an SD profile) for run_s, then take it off (load off, or profile stop) and
hold for tail_s. Each phase is scored: no-load peak and stability; mean,
droop, range, per-stroke ripple, error and torque effort under load; and the
ring after the load comes off (period, decay per cycle, and the phase margin
they imply), with metrics.cost() over the lot (plan item I8). The gains
the Zombie had at the start are written back at the end, also after an
error. GovLog is turned on for the sweep (it is needed for scoring) and
restored with the gains. Nothing is saved on the Zombie: a power cycle
restores its setup.

The governor integral is cleared at the start of each set by setting govImax
to 0 for a moment (the Zombie clamps the integral to +-govImax while the
governor is binding). Otherwise a set with govKi 0 keeps whatever integral
the previous set left, since only speed below revlim/2 resets it.

A set may also hold `sim:` model parameters for the simulator, such as
`sim: {noise: 5}` for speed noise on 0x1DA. Presets reset the model, so they
are applied again after every load change, and put back by preset noload.

`repeats: N` runs every set N times. metrics.csv then has one row per run,
and metrics_summary.csv the mean and standard deviation of each metric per
set (plan item I5). The simulator restarts its noise on preset and mark, so
repeats differ only by timing.

The pedal must be held at a fixed position for the whole sweep (a clamp or a
fixed pot). Each row reports the mean pedal position and the share of time
the governor was the binding limit, so a slipped pedal shows up.

    python sweep.py sweep.yaml --port COM3 --out D:/runs/sweep1

Example sweep.yaml:

    load: {preset: baler}          # or {profile: baler_r3}  (SD /profiles/)
    # A load can also run console commands after the preset (cmds) and ramp
    # the signed gradient torque along [seconds from load start, Nm] points:
    #   load: {preset: noload, cmds: ["load noise 3 2"],
    #          grad: [[0, 20], [10, 20], [25, 60], [40, 60]]}
    settle_s: 8                    # no-load time before each run
    run_s: 30
    tail_s: 4                      # after the load comes off; 0 = no tail
    repeats: 1
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
    and stream lines go to whichever files are open.

    send() writes without waiting. Its reply is counted off when it arrives,
    and cmd() first waits for every such reply, so a late 'ok' can never be
    taken as the answer to the next command."""

    def __init__(self, port, log=print, clock=time.monotonic, sleep=time.sleep):
        self.port = port
        self.log = log
        self.clock = clock
        self.sleep = sleep
        self.canlog = None
        self.unacked = 0  # replies still due for send()

    def _line(self, raw=None):
        raw = self.port.readline() if raw is None else raw
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
        if self.unacked and (line.startswith("ok") or line.startswith("err")):
            self.unacked -= 1
            if line.startswith("err"):
                self.log(f"    WARNING: unchecked command failed: {line[:100]}")
            return None
        return line

    def pump(self, seconds: float):
        end = self.clock() + seconds
        while self.clock() < end:
            raw = self.port.readline()
            if raw:
                self._line(raw)
            else:
                self.sleep(0.002)  # only when idle: canlog runs at ~500 lines/s

    def send(self, c: str):
        """Writes a command without waiting for its reply."""
        self.port.write((c + "\n").encode())
        self.unacked += 1

    def drain(self, timeout_s: float = 3.0):
        """Waits until every send() has had its reply."""
        end = self.clock() + timeout_s
        while self.unacked and self.clock() < end:
            raw = self.port.readline()
            if raw:
                self._line(raw)
            else:
                self.sleep(0.002)
        if self.unacked:
            n, self.unacked = self.unacked, 0
            raise SweepError(f"{n} replies missing")

    def cmd(self, c: str, timeout_s: float = 3.0) -> str:
        self.drain(timeout_s)
        self.port.write((c + "\n").encode())
        end = self.clock() + timeout_s
        while self.clock() < end:
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
    if int(s.get("repeats", 1)) < 1:
        raise SweepError("repeats must be at least 1")
    return s


def apply_load(con: Console, load: dict):
    if "preset" in load:
        con.cmd(f"preset {load['preset']}")
    else:
        con.cmd(f"profile load {load['profile']}", timeout_s=30.0)
        con.cmd("profile start")
    for c in load.get("cmds") or []:
        con.cmd(c)


def run_load(con: Console, load: dict, run_s: float):
    """Holds the load for run_s, ramping the gradient if the load has one."""
    pts = load.get("grad")
    if not pts:
        con.pump(run_s)
        return
    ts = [float(p[0]) for p in pts]
    nm = [float(p[1]) for p in pts]
    t0 = con.clock()
    last = None
    while (now := con.clock() - t0) < run_s:
        v = round(float(np.interp(now, ts, nm)), 1)
        if last is None:
            con.cmd(f"load grad {v}")
        elif v != last:
            con.send(f"load grad {v}")  # replies are counted off by the console
        last = v
        con.pump(min(GRAD_STEP_S, run_s - now))
    con.drain()


def remove_load(con: Console, load: dict, mark: int):
    """Takes the load off for the tail phase and leaves the model as it is.
    The mark (the set's own number again) tells score() when."""
    con.cmd(f"mark {mark}")
    if "profile" in load:
        try:
            con.cmd("profile stop")
        except SweepError:
            pass  # already finished
    con.cmd("load off")


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


GRAD_STEP_S = 0.1            # gradient ramps are sent as 'load grad' this often
LOAD_START_EVENTS = (3, 5)   # 0x4B2 ProfileStart, Preset
EV_MARKER = 1                # 0x4B2 marker: the set's number, at its start and at load removal
LOAD_SKIP_S = 5.0            # load-phase metrics start this long after the load is applied
NOLOAD_TAIL_S = 3.0          # no-load stability is judged over the last few seconds
INT_CLEAR_S = 0.5            # govImax held at 0 this long to clear the integral
TAIL_S = 4.0                 # default time after the load comes off, for the ring metrics


def _stroke_p2p(t, v, period_s):
    """Median peak-to-peak speed over consecutive one-stroke windows."""
    if not np.isfinite(period_s) or period_s <= 0 or len(t) < 2:
        return float("nan")
    n = max(2, int(round(period_s / (t[1] - t[0]))))
    p2p = [np.ptp(v[i:i + n]) for i in range(0, len(v) - n + 1, n) if np.isfinite(v[i:i + n]).all()]
    return float(np.median(p2p)) if p2p else float("nan")


def score(bus_log: Path, revlim: float) -> dict:
    """Scores one run in three phases: no-load after the gain change, the load
    from LOAD_SKIP_S after it was applied until it comes off, and the ring
    after it comes off. compare_runs' tractor metrics are not used directly:
    their 90 % of revlim mask drops every baler stroke."""
    log = loglib.load(bus_log)
    g = log.grid
    t = g.index.to_numpy()
    v = np.abs(loglib.speed_rpm(log).to_numpy(dtype=float))
    ev = log.raw.get("SIM_Event_4B2")
    starts = ev["t"][ev["Event"].isin(LOAD_START_EVENTS)].to_numpy() if ev is not None else []
    t_load = float(starts[-1]) if len(starts) else 0.0
    marks = ev["t"][(ev["Event"] == EV_MARKER) & (ev["t"] > t_load)].to_numpy() if ev is not None else []
    t_off = float(marks[0]) if len(marks) else float("inf")

    nl = (t >= t_load - NOLOAD_TAIL_S) & (t < t_load) & np.isfinite(v)
    nl_all = (t < t_load) & np.isfinite(v)
    ld = (t >= t_load + LOAD_SKIP_S) & (t < t_off) & np.isfinite(v)
    m = {"revlim_rpm": revlim}
    m["noload_peak_rpm"] = float(v[nl_all].max()) if nl_all.any() else float("nan")
    m["noload_p2p_rpm"] = float(np.ptp(v[nl])) if nl.any() else float("nan")
    if log.has(SIM_TREQ) and nl.any():
        treq_nl = g[SIM_TREQ].to_numpy(dtype=float)[nl]
        m["noload_treq_sd_nm"] = float(np.nanstd(treq_nl))
        m["noload_treq_tick_rms_nm"] = float(np.sqrt(np.nanmean((np.diff(treq_nl) / np.sqrt(2)) ** 2)))
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
        err = vl - revlim
        m["load_rms_err_rpm"] = float(np.sqrt(np.mean(err ** 2)))
        m["load_mae_rpm"] = float(np.mean(np.abs(err)))
        m["load_iae_rpm_s"] = mt.iae_rpm_s(t, v, revlim, ld)
        m["load_max_over_rpm"] = float(err.max())
        m["load_max_under_rpm"] = float(-err.min())
        if log.has(SIM_TREQ):
            treq = g[SIM_TREQ].to_numpy(dtype=float)
            d = np.diff(treq[ld]) / np.sqrt(2)
            m["treq_tick_rms_nm"] = float(np.sqrt(np.nanmean(d ** 2)))
            m["load_treq_rms_nm"] = mt.torque_rms_nm(treq, ld)
    if np.isfinite(t_off) and t[-1] > t_off + 1.0:
        # The model's own speed, where logged, has no measurement noise to
        # cross the settling level and fake a peak
        ring = np.abs(g[SIM_SPEED].to_numpy(dtype=float)) if log.has(SIM_SPEED) else v
        period, decay, n = mt.ring_after_step(t, ring, t_off, window_s=float(t[-1] - t_off))
        m["ring_period_s"] = period
        m["ring_decay"] = decay
        m["ring_pm_deg"] = mt.phase_margin_deg(decay)
        m["ring_peaks"] = n
    m["peak_integral_pct"] = mt.peak_abs(g[loglib.GOV_INT]) if log.has(loglib.GOV_INT) else float("nan")
    m["pedal_pct"] = float(np.nanmean(g[POT_USER].to_numpy(dtype=float)[ld])) if log.has(POT_USER) else float("nan")
    m["binding_frac"] = (float(np.nanmean(g[loglib.GOV_BINDING].to_numpy(dtype=float)[ld] > 0.5))
                         if log.has(loglib.GOV_BINDING) else float("nan"))
    m["dropped_4a1"] = log.dropped_4a1
    m["cost"] = mt.cost(m)
    return m


def run_sweep(s: dict, con: Console, out: Path, log=print) -> list[dict]:
    load = s["load"]
    settle_s = float(s.get("settle_s", 8.0))
    run_s = float(s.get("run_s", 30.0))
    tail_s = float(s.get("tail_s", TAIL_S))
    repeats = int(s.get("repeats", 1))
    base = dict(s.get("base") or {})
    base_sim = base.pop("sim", None) or {}
    names = sorted({k for g in [base] + list(s["sets"]) for k in g if k != "sim"})
    sim_names = sorted({k for g in [{"sim": base_sim}] + list(s["sets"]) for k in (g.get("sim") or {})})

    con.cmd("stream 0")
    con.cmd("canlog 0")
    original = {n: con.zget(n) for n in sorted(set(names) | {"revlim", "GovLog", "govImax"})}
    log(f"Zombie at start: {original}")
    rows = []
    try:
        con.cmd("zset GovLog 1")  # governor frames 0x4A0-0x4A3, needed for scoring
        for i, overrides in enumerate(s["sets"], 1):
            overrides = dict(overrides)
            sim = {**base_sim, **(overrides.pop("sim", None) or {})}
            gains = {**base, **overrides}
            for r in range(1, repeats + 1):
                rows.append(run_set(con, load, i, r, gains, sim, original, names, sim_names,
                                    settle_s, run_s, tail_s, repeats, len(s["sets"]), out, log))
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


def run_set(con, load, i, r, gains, sim, original, names, sim_names,
            settle_s, run_s, tail_s, repeats, n_sets, out, log) -> dict:
    """One run of one set: clear, log, gains, no-load, load, tail, score."""
    log(f"set {i}/{n_sets}" + (f" run {r}/{repeats}" if repeats > 1 else "")
        + f": {gains}" + (f" sim {sim}" if sim else ""))
    clear_load(con, load)
    apply_sim(con, sim)
    con.cmd("zset govImax 0")  # clears the integral the previous set left
    con.pump(INT_CLEAR_S)
    con.cmd(f"zset govImax {gains.get('govImax', original['govImax'])}")
    con.pump(NOLOAD_TAIL_S)  # let the previous load's removal die away
    bus = out / (f"run_{i:02d}_bus.log" if repeats == 1 else f"run_{i:02d}_{r}_bus.log")
    with open(bus, "w") as f:
        con.canlog = f
        con.cmd("canlog 1")
        con.cmd(f"mark {i}")
        for n, v in gains.items():
            con.cmd(f"zset {n} {v}")
        con.pump(settle_s)  # no-load phase, scored for stability
        apply_load(con, load)
        apply_sim(con, sim)
        run_load(con, load, run_s)
        if tail_s > 0:
            remove_load(con, load, i)
            con.pump(tail_s)  # scored for the ring after the step
        con.cmd("canlog 0")
        con.pump(0.2)  # lines already queued
        con.canlog = None
    clear_load(con, load)
    row = {"set": i, **({"run": r} if repeats > 1 else {}),
           **{n: gains.get(n, original[n]) for n in names},
           **{f"sim_{n}": sim.get(n, "") for n in sim_names}}
    try:
        row.update(score(bus, float(gains.get("revlim", original["revlim"]))))
    except Exception as e:  # keep the sweep going; the log is kept
        log(f"    scoring failed: {e}")
    if not row.get("pedal_pct", 0) >= 5.0:
        log(f"    WARNING: pedal at {row.get('pedal_pct', float('nan')):.1f} %; "
            "this run is not meaningful (is the pedal held?)")
    log("    " + ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                           for k, v in row.items()
                           if k not in names and k not in ("set", "run") and not k.startswith("sim_")))
    return row


def write_table(rows: list[dict], path: Path):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def summarise(rows: list[dict]) -> list[dict]:
    """One row per set: its gains and sim parameters, the number of runs, then
    the mean and standard deviation over its runs of every measured number
    (the columns from revlim_rpm on)."""
    out = []
    for i in dict.fromkeys(r["set"] for r in rows):
        runs = [r for r in rows if r["set"] == i]
        keys = list(dict.fromkeys(k for r in runs for k in r))
        first_metric = keys.index("revlim_rpm") if "revlim_rpm" in keys else len(keys)
        row = {"set": i, "runs": len(runs)}
        for j, k in enumerate(keys):
            if k in ("set", "run"):
                continue
            if j < first_metric:
                row[k] = runs[0].get(k, "")
                continue
            a = np.array([r.get(k, np.nan) for r in runs], dtype=float)
            ok = np.isfinite(a)
            row[f"{k}_mean"] = float(a[ok].mean()) if ok.any() else float("nan")
            row[f"{k}_sd"] = float(a[ok].std(ddof=1)) if ok.sum() > 1 else float("nan")
        out.append(row)
    return out


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
    print(f"done: {len(rows)} runs, table in {out / 'metrics.csv'}")
    if int(s.get("repeats", 1)) > 1:
        write_table(summarise(rows), out / "metrics_summary.csv")
        print(f"mean and standard deviation per set in {out / 'metrics_summary.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
