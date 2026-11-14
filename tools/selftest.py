"""Self-test for the PC tools (plan section 9, "Tool self-test").

Builds tools/synth/synth_log.cpp with g++ from the firmware's own plain C++
libraries, generates logs with known parameters, and checks:

  1. extract_params recovers J, Tc, b, c within 5 % (the plan's criterion)
  2. make_profile recovers a known baler load, and the profile it writes
     parses and replays through the firmware's profile code unchanged
  3. loglib counts dropped 0x4A1 frames and reads candump and telematics logs
  4. the metrics give the expected values on known signals, including the
     loop metrics and cost() of plan item I8
  5. the Python mirror of the simulator's speed noise (lib/sensor) passes the
     governor filter like the C++ does (plan item I6)
  6. compare_runs gives identical numbers for identical logs
  7. run_script sends commands on time and checks the replies
  8. sweep.py, against a fake console: replies matched to their commands with
     canlog lines and late replies mixed in, the integral cleared, gains
     restored after an error, repeats summarised (plan item I4)
  9. loopcalc's conversions, and its rings against the M4 bench
 10. the software-in-the-loop harness (M5c): the Zombie's golden test
     through it, a 3 Nov bench run reproduced, and loopcalc against it.
     Needs the Stm32-vcu tree (ZOMBIE_DIR or ../Stm32-vcu); skipped without

    python selftest.py [--keep DIR]
"""

from __future__ import annotations

import argparse
import io
import json
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

TOOLS = Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import compare_runs  # noqa: E402
import extract_params  # noqa: E402
import loglib  # noqa: E402
import make_profile  # noqa: E402
import metrics as mt  # noqa: E402
import run_script  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")


def build_synth(out_dir: Path) -> Path:
    gxx = shutil.which("g++")
    if not gxx:
        raise SystemExit("g++ not found on PATH; it is needed to build the synthetic log generator")
    exe = out_dir / ("synth_log.exe" if sys.platform == "win32" else "synth_log")
    libs = ["leafcodec", "plant", "loads", "sensor", "simproto", "simcore"]
    srcs = [TOOLS / "synth" / "synth_log.cpp", ROOT / "lib/leafcodec/leafcodec.cpp",
            ROOT / "lib/plant/plant.cpp", ROOT / "lib/loads/loads.cpp",
            ROOT / "lib/loads/profile.cpp", ROOT / "lib/simproto/telemetry.cpp",
            ROOT / "lib/sensor/speed_report.cpp"]
    cmd = [gxx, "-O2", "-std=c++17", *[f"-I{ROOT / 'lib' / l}" for l in libs],
           *map(str, srcs), "-o", str(exe)]
    subprocess.run(cmd, check=True)
    return exe


def synth(exe: Path, *args):
    subprocess.run([str(exe), *map(str, args)], check=True)


def test_fit(exe, d):
    sets = [dict(J=0.42, Tc=6.5, b=0.025, c=4e-5), dict(J=0.9, Tc=10.0, b=0.04, c=1e-4)]
    for i, p in enumerate(sets):
        log_path = d / f"accel_coast_{i}.csv"
        synth(exe, "accel_coast", log_path, *[f"{k}={v}" for k, v in p.items()])
        res, _, _ = extract_params.fit([loglib.load(log_path)], extract_params.sm.MotorParams())
        errs = {k: (res[k] - v) / v * 100 for k, v in p.items()}
        worst = max(abs(e) for e in errs.values())
        check(f"extract_params set {i + 1} within 5 %", worst <= 5.0,
              " ".join(f"{k} {e:+.2f}%" for k, e in errs.items()))


def test_profile(exe, d):
    p = dict(J=0.9, Tc=10.0, b=0.03, c=0.0)
    synth(exe, "accel_coast", d / "ac_baler.csv", *[f"{k}={v}" for k, v in p.items()])
    res, _, _ = extract_params.fit([loglib.load(d / "ac_baler.csv")], extract_params.sm.MotorParams())
    params_path = d / "params.json"
    params_path.write_text(json.dumps(res | {"kT": 0.25, "tau": 30.0, "Tmax": 280.0, "Pmax": 80.0}))

    synth(exe, "baler", d / "baler.csv", d / "baler_truth.csv", "seconds=40",
          *[f"{k}={v}" for k, v in p.items()])
    prof = d / "baler_profile.csv"
    make_profile.main([str(d / "baler.csv"), str(params_path), "-o", str(prof)])
    got = pd.read_csv(prof)
    truth = pd.read_csv(d / "baler_truth.csv")
    check("profile rows exactly 10 ms apart from 0",
          bool((got.t_ms.to_numpy() == np.arange(len(got)) * 10).all()))
    a, b = got.torque_nm.to_numpy(), truth.load_nm.to_numpy()
    n = min(len(a), len(b)) - 20
    best = min(((lag, float(np.sqrt(np.mean((a[10 + lag:10 + lag + n] - b[10:10 + n]) ** 2))))
                for lag in range(-3, 4)), key=lambda x: x[1])
    corr = float(np.corrcoef(a[10 + best[0]:10 + best[0] + n], b[10:10 + n])[0, 1])
    peak = float(b.max())
    check("make_profile recovers the baler load", best[1] < 0.05 * peak and corr > 0.99,
          f"rms {best[1]:.2f} Nm on a {peak:.0f} Nm peak, correlation {corr:.4f}, lag {best[0]}")

    replay = d / "baler_replay.csv"
    synth(exe, "replay", prof, replay)
    r = pd.read_csv(replay)
    diff = float(np.abs(r.torque_nm.to_numpy() - got.torque_nm.to_numpy()).max())
    check("profile replays through the firmware profile code", len(r) == len(got) and diff <= 0.051,
          f"{len(r)} rows, max difference {diff:.3f} Nm (storage is 0.1 Nm)")
    return d / "baler.csv"


def test_loglib(exe, d):
    synth(exe, "accel_coast", d / "dropped.csv", "drop=3")
    log = loglib.load(d / "dropped.csv")
    check("loglib counts dropped 0x4A1 frames", log.dropped_4a1 == 3, f"counted {log.dropped_4a1}")
    check("counter gaps across wrap", loglib.counter_gaps([253, 254, 255, 0, 2, 3]) == 1)

    # The same frames as a candump -l log must decode identically
    sav = loglib.read_frames(d / "dropped.csv")
    dump = d / "dropped.log"
    with open(dump, "w") as f:
        for t, cid, data in zip(sav.t[:3000], sav.can_id[:3000], sav.data[:3000]):
            f.write(f"({1700000000 + t:.6f}) can0 {cid:03X}#{data.hex().upper()}\n")
    a = loglib.load(dump).grid[loglib.SPEED].to_numpy()
    b = log.grid[loglib.SPEED].to_numpy()[:len(a)]
    ok = len(a) > 100 and np.allclose(a[:-5], b[:len(a) - 5], equal_nan=True)
    check("candump log decodes like SavvyCAN CSV", ok)

    # The same frames as a telematics session file: boot-time ms rows, then
    # the TIME_SYNC row and unix ms, with trip markers in between
    tele = d / "session_0001.csv"
    boot0_ms, unix_at_boot_ms, n_boot = 60_000, 1_794_000_000_000, 1500
    with open(tele, "w") as f:
        f.write("Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n")
        for i, (t, cid, data) in enumerate(zip(sav.t[:3000], sav.can_id[:3000], sav.data[:3000])):
            ms = boot0_ms + round(t * 1000)
            if i == n_boot:
                f.write(f"TIME_SYNC,{unix_at_boot_ms + ms},{ms},,,,,,,,,,\nTRIP_START,,,,,,,,,,,,\n")
            stamp = ms if i < n_boot else unix_at_boot_ms + ms
            f.write(f"{stamp},0x{cid:03X},false,0,{len(data)},"
                    + ",".join(f"{x:02X}" for x in data.ljust(8, b"\0")) + "\n")
        f.write("TRIP_END,30,-0.1,-0.03,50,50,40.0,,,,,,\n")
    tlog = loglib.load(tele)
    c = tlog.grid[loglib.SPEED].to_numpy()
    ok = len(c) > 100 and np.allclose(c[:len(a) - 5], b[:len(a) - 5], atol=1.0, equal_nan=True)
    start_ok = tlog.start_unix_s is not None and abs(tlog.start_unix_s - (unix_at_boot_ms + boot0_ms) / 1000) < 0.002
    check("telematics log decodes like SavvyCAN CSV, across TIME_SYNC", ok and start_ok,
          f"start {tlog.start_unix_s}")


def test_metrics():
    t = np.arange(0, 20, 0.01)
    rev = 2000.0
    # Rise to 2100, decay back to 2000 with 1 s time constant
    v = np.where(t < 1, 2100 * t, 2000 + 100 * np.exp(-(t - 1)))
    check("overshoot", abs(mt.overshoot_rpm(v, rev) - 100) < 0.5)
    st = mt.settle_time_s(t, v, rev)
    exp_st = (math.log(100 / 20) + 1) - (0.99 * rev / 2100)  # 1 % band = 20 rpm
    check("settle time", abs(st - exp_st) < 0.02, f"{st:.3f} s, expected {exp_st:.3f} s")
    rip = 2000 - 30 + 30 * np.sin(2 * np.pi * t / 0.7)
    check("droop", abs(mt.droop_rpm(rip, rev, np.ones_like(t, bool)) - 30) < 0.5)
    check("stroke period", abs(mt.dominant_period_s(t, rip) - 0.7) < 0.03,
          f"{mt.dominant_period_s(t, rip):.3f} s")
    check("ripple peak-to-peak", abs(mt.ripple_p2p_rpm(t, rip, 0.7) - 60) < 1.0,
          f"{mt.ripple_p2p_rpm(t, rip, 0.7):.2f} rpm")
    step = np.where(t < 5, 2000, 2000 - 150 * np.exp(-(t - 5) / 0.5))
    rec = mt.recovery_time_s(t, step, 5.0, 20)
    exp_rec = 0.5 * math.log(150 / 20)
    check("recovery time", abs(rec - exp_rec) < 0.02, f"{rec:.3f} s, expected {exp_rec:.3f} s")
    check("time with flag set", abs(mt.time_true_s(np.r_[np.zeros(100), np.ones(250)]) - 2.5) < 1e-9)


def test_metrics_i8():
    t = np.arange(0, 10, 0.01)
    check("IAE of a constant error", abs(mt.iae_rpm_s(t, np.full_like(t, 2210.0), 2200.0, t < 2) - 20.0) < 1e-6)
    tq = 50 + 30 * np.sin(2 * np.pi * t / 0.7)
    check("torque RMS about the mean", abs(mt.torque_rms_nm(tq) - 30 / math.sqrt(2)) < 0.1,
          f"{mt.torque_rms_nm(tq):.2f} Nm")
    # A ring that keeps 0.65 of itself each 0.63 s cycle, as on the M4 bench at shift 4
    period, keep, t0 = 0.63, 0.65, 1.0
    sig = -math.log(keep) / period
    v = np.where(t < t0, 2200.0, 2200 + 100 * np.exp(-sig * (t - t0)) * np.cos(2 * np.pi * (t - t0) / period))
    p, dec, n = mt.ring_after_step(t, v, t0, window_s=8.0)
    check("ring period and decay after a step", abs(p - period) < 0.02 and abs(dec - keep) < 0.04 and n >= 4,
          f"{p:.3f} s, keeps {dec:.3f} per cycle, {n} peaks")
    p1, d1, n1 = mt.ring_after_step(t, np.where(t < t0, 2200.0, 2200 + 100 * np.exp(-(t - t0) / 0.2)), t0)
    check("no ring after a smooth recovery", d1 == 0.0 and n1 == 1, f"decay {d1}, {n1} peaks")
    pm65, pm35 = mt.phase_margin_deg(0.65), mt.phase_margin_deg(0.35)
    check("phase margin from decay (plan section 16: about 7 and 17-21 degrees)",
          6 <= pm65 <= 9 and 16 <= pm35 <= 21, f"{pm65:.1f} and {pm35:.1f} degrees")
    m = {"load_mae_rpm": 70.0, "stroke_p2p_rpm": 240.0, "stroke_period_s": 0.7, "load_treq_rms_nm": 60.0}
    want = sum(mt.COST_WEIGHTS[k] * m[k] for k in mt.COST_WEIGHTS)
    worse = dict(m, stroke_p2p_rpm=300.0)
    check("cost is the weighted sum, higher for more ripple, NaN when a term is missing",
          abs(mt.cost(m) - want) < 1e-9 and mt.cost(worse) > mt.cost(m)
          and math.isnan(mt.cost({"load_mae_rpm": 1.0})), f"{mt.cost(m):.1f}")


def test_speed_noise(exe, d):
    """The Python mirror of lib/sensor gives the governor the same noise."""
    import fit_noise
    import simmodel as sm
    from scipy.signal import lfilter
    for fc, bw in ((0.0, 4.0), (6.0, 4.0)):
        out = d / f"noise_{fc:g}.csv"
        synth(exe, "speednoise", out, "noise=10", f"fc={fc}", f"bw={bw}", "ticks=200000")
        c = pd.read_csv(out).noise_rpm.to_numpy()
        p = sm.speed_noise(200000, 10.0, fc, bw, np.random.default_rng(3))
        rows = []
        ok = abs(c.std() - 10) < 0.3 and abs(p.std() - 10) < 0.3
        for k in (4, 2):
            a = 1.0 / (1 << k)
            fc_, fp = (lfilter([a], [1, -(1 - a)], x)[500:].std() for x in (c, p))
            want = fit_noise.true_passed(fc, bw, 10.0, k)
            ok &= abs(fc_ / want - 1) < 0.05 and abs(fp / want - 1) < 0.05
            rows.append(f"shift {k}: C++ {fc_:.2f}, Python {fp:.2f}, expected {want:.2f}")
        check(f"speed noise {'white' if fc == 0 else f'band {fc:g} Hz'}: C++ and Python mirror agree",
              ok, f"sd {c.std():.2f}/{p.std():.2f}; " + "; ".join(rows))


class FakeSim:
    """Stands in for the simulator console and the Zombie behind it, for
    sweep.py. Replies in order, like the real console, with 'load grad'
    answered late. canlog lines are mixed in while canlog is on. Plain 'ok'
    replies carry a tag, so a test can see which command each one answered."""

    def __init__(self, now, fail_on=None, late_s=0.3):  # longer than the 0.2 s the old sweep waited
        self.now = now
        self.fail_on = fail_on
        self.late_s = late_s
        self.z = {"revlim": 2200.0, "govKp": 11.0, "govKi": 0.0, "govKd": 0.0, "govImax": 100.0,
                  "govFlt": 4.0, "GovLog": 0.0}
        self.queue = []
        self.sent = []
        self.tags = {}
        self.canlog = False
        self.next_frame = 0.0

    def write(self, data: bytes):
        c = data.decode().strip()
        self.sent.append(c)
        parts = c.split()
        if c == self.fail_on:
            reply = "err Zombie refused (abort 0x06090030)"
        elif parts[0] == "zget":
            reply = f"ok {parts[1]}={self.z[parts[1]]:g}"
        elif parts[0] == "zset":
            old = self.z[parts[1]]
            self.z[parts[1]] = float(parts[2])
            reply = f"ok {parts[1]}={float(parts[2]):g} was={old:g}"
        else:
            reply = f"ok #{len(self.sent)}"
            self.tags[len(self.sent)] = c
        if parts[0] == "canlog":
            self.canlog = parts[1] == "1"
        due = self.now[0] + (self.late_s if c.startswith("load grad") else 0.002)
        self.queue.append((max(due, self.queue[-1][0] if self.queue else 0.0), (reply + "\n").encode()))

    def readline(self):
        self.now[0] += 0.001
        if self.canlog and self.now[0] >= self.next_frame:
            self.next_frame = self.now[0] + 0.002
            return f"({self.now[0]:.6f}) can0 4B0#0000000000000000\n".encode()
        if self.queue and self.queue[0][0] <= self.now[0]:
            return self.queue.pop(0)[1]
        return b""


def test_sweep(d):
    import sweep
    spec = sweep.load_sweep("load: {preset: default, grad: [[0, 10], [1, 30]]}\n"
                            "settle_s: 0.5\nrun_s: 1.0\ntail_s: 0.3\n"
                            "base: {govKp: 11}\nsets: [{govKi: 0}, {govKi: 1, govFlt: 2}]\n")
    now = [0.0]

    def sleep(s):
        now[0] += s

    def run(sim, out):
        lines = []
        con = sweep.Console(sim, log=lines.append, clock=lambda: now[0], sleep=sleep)
        out.mkdir(exist_ok=True)
        try:
            return sweep.run_sweep(spec, con, out, log=lines.append), lines, None
        except sweep.SweepError as e:
            return None, lines, e

    sim = FakeSim(now)
    rows, lines, err = run(sim, d / "sweep_ok")
    answered = [(c, int(r.split("#")[1])) for c, r in
                (ln.strip()[2:].split("  < ") for ln in lines if ln.strip().startswith("> ")) if "#" in r]
    wrong = [(c, sim.tags[n]) for c, n in answered if sim.tags[n] != c]
    grads = sum(c.startswith("load grad") for c in sim.sent)
    check("sweep: every reply goes to its own command, with late grad replies", err is None and not wrong
          and grads > 5 and len(answered) >= 15, f"{len(answered)} checked, {grads} grad commands, mismatched {wrong[:2]}")
    bus = (d / "sweep_ok" / "run_01_bus.log").read_text().splitlines()
    check("sweep: canlog lines go to the bus log, not the reply reader",
          len(bus) > 100 and all(ln.startswith("(") for ln in bus), f"{len(bus)} lines")
    seq = [c for c in sim.sent if c.startswith("zset govImax") or c == "canlog 1"]
    want = ["zset govImax 0", "zset govImax 100.0", "canlog 1"] * 2
    check("sweep: integral cleared before each set (govImax 0, then back, then the log)",
          seq[:6] == want, " | ".join(seq[:6]))
    check("sweep: tail phase marks and removes the load",
          sim.sent.count("load off") == 2 and sim.sent.count("mark 1") == 2, "")
    check("sweep: gains restored at the end", sim.z == FakeSim(now).z, str(sim.z))

    sim = FakeSim(now, fail_on="zset govFlt 2")
    _, lines, err = run(sim, d / "sweep_err")
    tail = {c.split()[1]: float(c.split()[2]) for c in sim.sent[sim.sent.index("zset govFlt 2") + 1:]
            if c.startswith("zset ")}
    orig = FakeSim(now).z
    check("sweep: gains restored after an error", err is not None and sim.z == orig
          and all(tail.get(n) == orig[n] for n in ("govKi", "GovLog", "govFlt", "govImax", "revlim")),
          f"stopped with: {err}")

    rows = [{"set": 1, "run": r, "govKp": 11, "revlim_rpm": 2200.0, "stroke_p2p_rpm": v}
            for r, v in ((1, 230.0), (2, 240.0), (3, 235.0))]
    s = sweep.summarise(rows)
    check("sweep: repeats summarised as mean and standard deviation",
          len(s) == 1 and s[0]["runs"] == 3 and s[0]["govKp"] == 11
          and abs(s[0]["stroke_p2p_rpm_mean"] - 235) < 1e-9 and abs(s[0]["stroke_p2p_rpm_sd"] - 5) < 1e-9, str(s[0]))


def test_compare(baler_log):
    log = loglib.load(baler_log)
    a = compare_runs.compute(log, 2000.0)
    b = compare_runs.compute(loglib.load(baler_log), 2000.0)
    same = all((math.isnan(a[k]) and math.isnan(b[k])) or a[k] == b[k] for k in a)
    check("compare_runs is repeatable on identical logs", same)
    check("compare_runs finds the 0.7 s plunger stroke", abs(a["stroke_period_s"] - 0.7) < 0.05,
          f"{a['stroke_period_s']:.3f} s, ripple {a['ripple_p2p_rpm']:.1f} rpm")


class FakePort:
    """Answers each command with 'ok', or 'err' for commands starting 'bad'."""

    def __init__(self, clock):
        self.clock = clock
        self.sent = []
        self.pending = [b"12,0,0.00,0.00,5.00,0.0,1\n"]

    def write(self, data: bytes):
        cmd = data.decode().strip()
        self.sent.append((round(self.clock(), 3), cmd))
        self.pending.append(b"err nope\n" if cmd.startswith("bad") else b"ok\n")
        self.pending.append(b"22,0,0.00,0.00,5.00,0.0,1\n")

    def readline(self):
        return self.pending.pop(0) if self.pending else b""


def test_run_script():
    now = [0.0]
    clock = lambda: now[0]  # noqa: E731

    def sleep(s):
        now[0] += s

    def readline_advance(port):
        orig = port.readline

        def rl():
            now[0] += 0.001
            return orig()
        port.readline = rl

    script = run_script.load_script(
        "steps:\n  - {at: 0, cmd: preset baler}\n  - {at: 1.5, cmd: mark 1}\n"
        "  - {at: 3, cmd: bad thing, allow_err: true}\n")
    port = FakePort(clock)
    readline_advance(port)
    stream = io.StringIO()
    fails = run_script.run(script, port, log=lambda m: None, stream_out=stream, clock=clock, sleep=sleep)
    times = [t for t, _ in port.sent]
    on_time = all(abs(t - want) < 0.03 for t, want in zip(times, [0, 1.5, 3]))
    check("run_script sends on schedule", on_time and len(times) == 3, f"sent at {times}")
    check("run_script counts err replies", fails == 1)
    check("run_script saves stream lines", stream.getvalue().count("\n") >= 3)
    try:
        run_script.run(run_script.load_script("steps:\n  - {at: 0, cmd: bad}\n"), FakePort(clock),
                       log=lambda m: None, clock=clock, sleep=sleep)
        check("run_script stops on err without allow_err", False)
    except run_script.ScriptError:
        check("run_script stops on err without allow_err", True)


def test_loopcalc():
    """loopcalc's conversions, and its predictions against the M4 bench rings
    and the 3 Nov govKp sweep (plan section 16)."""
    import loopcalc as lc
    check("loopcalc: govKp 1 is 0.489 Nm per rad/s", abs(lc.kp_nm_per_rad_s(1.0) - 0.489) < 0.001,
          f"{lc.kp_nm_per_rad_s(1.0):.4f}")
    check("loopcalc: shift 4 filter lag about 155 ms", abs(lc.filter_lag_s(4) - 0.155) < 0.002)
    r4, r2 = lc.analyse(kp=11, flt=4, J=0.3), lc.analyse(kp=11, flt=2, J=0.3)
    check("loopcalc: M4 bench ring at shift 4 (0.64 s, each cycle keeps 0.65)",
          abs(r4["ring_period_s"] / 0.640 - 1) < 0.05 and abs(r4["ring_keep"] - 0.65) < 0.1,
          f"{r4['ring_period_s']:.3f} s, keep {r4['ring_keep']:.2f}")
    check("loopcalc: M4 bench ring at shift 2 (0.37 s, keeps 0.35)",
          abs(r2["ring_period_s"] / 0.370 - 1) < 0.05 and abs(r2["ring_keep"] - 0.35) < 0.1,
          f"{r2['ring_period_s']:.3f} s, keep {r2['ring_keep']:.2f}")
    s14, s18 = lc.analyse(kp=14, flt=4, J=0.3)["stable"], lc.analyse(kp=18, flt=4, J=0.3)["stable"]
    check("loopcalc: govKp 14 stable and 18 not, as in the 3 Nov sweep", s14 and not s18)
    # A P-only loop settles at the load over (Kp in Nm/rpm + viscous drag)
    y = lc.load_step_response(20.0, 1500, kp=11, flt=2, J=0.3)
    k = 11 * 0.01 * lc.PCT_TO_RAW * 0.25 + 0.02 / lc.RPM_PER_RAD_S
    check("loopcalc: P-only droop under a load step", abs(y[-1] + 20.0 / k) < 0.2,
          f"{y[-1]:.2f} rpm, expected {-20.0 / k:.2f}")


def test_sil(d: Path):
    """The software-in-the-loop harness (tools/sil, M5c): the Zombie's golden
    test through it, a bench run reproduced, and loopcalc against it. Needs
    the Stm32-vcu tree (ZOMBIE_DIR or ../Stm32-vcu); skipped without it."""
    import loopcalc as lc
    import sil_check
    import silport
    import sweep
    try:
        zdir = silport.zombie_dir()
    except silport.SilError as e:
        print(f"SKIP  sil harness checks: {e}")
        return
    check("sil: the Zombie's governor golden test passes through the harness",
          silport.golden(zdir, log=lambda m: None))
    # 3 Nov govFlt sweep, set 3 (Kp 11, Ki 1, shift 2, baler, pedal 89.7 %):
    # bench stroke_p2p_rpm 237.68, treq_stroke_p2p_nm 160.8
    s = sweep.load_sweep("load: {preset: baler}\nsettle_s: 8\nrun_s: 30\ntail_s: 4\n"
                         "base: {govKp: 11, govKi: 1, govKd: 0}\nsets:\n  - {govFlt: 2}\n")
    with silport.SilPort(pedal=89.7, log=lambda m: None) as port:
        con = sweep.Console(port, log=lambda m: None, clock=port.clock, sleep=port.sleep)
        row = sweep.run_sweep(s, con, d, log=lambda m: None)[0]
        check("sil: 3 Nov bench run reproduced (stroke ripple within 1 %)",
              abs(row["stroke_p2p_rpm"] / 237.68 - 1) < 0.01 and abs(row["treq_stroke_p2p_nm"] / 160.8 - 1) < 0.01,
              f"stroke {row['stroke_p2p_rpm']:.1f} rpm, torque {row['treq_stroke_p2p_nm']:.1f} Nm per stroke")
        check("sil: loop delay is 3 ticks, as on the bench", abs(row["loop_ticks"] - 3.0) < 0.01,
              f"{row['loop_ticks']:.2f}")
        check("sil: the simulated clock ran (no wall-clock waits)", port.clock() > 40.0)
        case = (11, 0, 4, 0.3, 0)
        dv, (period, keep, n) = sil_check.harness_ring(port, case, d / "sil_ring_bus.log")
    lin = lc.load_step_response(-sil_check.STEP_NM, sil_check.RESPONSE_TICKS, kp=11, flt=4, J=0.3)
    shape = sil_check.response_shape(dv, lin)
    want = lc.analyse(kp=11, flt=4, J=0.3)["ring_period_s"]
    check("sil: loopcalc and the harness agree (ring period within 10 %, response within 10 %)",
          abs(period / want - 1) < 0.10 and shape < 0.10,
          f"ring {period:.3f} s against {want:.3f} s, response RMS difference {shape:.1%}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--keep", help="keep generated files in this directory")
    a = ap.parse_args(argv)
    d = Path(a.keep) if a.keep else Path(tempfile.mkdtemp(prefix="leafsim_selftest_"))
    d.mkdir(parents=True, exist_ok=True)
    try:
        exe = build_synth(d)
        test_fit(exe, d)
        baler = test_profile(exe, d)
        test_loglib(exe, d)
        test_metrics()
        test_metrics_i8()
        test_speed_noise(exe, d)
        test_compare(baler)
        test_run_script()
        test_sweep(d)
        test_loopcalc()
        test_sil(d)
    finally:
        if not a.keep:
            shutil.rmtree(d, ignore_errors=True)
    passed = sum(RESULTS)
    print(f"\n{passed}/{len(RESULTS)} checks passed")
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
