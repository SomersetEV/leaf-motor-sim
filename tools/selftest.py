"""Self-test for the PC tools (plan section 9, "Tool self-test").

Builds tools/synth/synth_log.cpp with g++ from the firmware's own plain C++
libraries, generates logs with known parameters, and checks:

  1. extract_params recovers J, Tc, b, c within 5 % (the plan's criterion)
  2. make_profile recovers a known baler load, and the profile it writes
     parses and replays through the firmware's profile code unchanged
  3. loglib counts dropped 0x4A1 frames and reads candump logs
  4. the metrics give the expected values on known signals
  5. compare_runs gives identical numbers for identical logs
  6. run_script sends commands on time and checks the replies

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
    libs = ["leafcodec", "plant", "loads", "simproto", "simcore"]
    srcs = [TOOLS / "synth" / "synth_log.cpp", ROOT / "lib/leafcodec/leafcodec.cpp",
            ROOT / "lib/plant/plant.cpp", ROOT / "lib/loads/loads.cpp",
            ROOT / "lib/loads/profile.cpp", ROOT / "lib/simproto/telemetry.cpp"]
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
        test_compare(baler)
        test_run_script()
    finally:
        if not a.keep:
            shutil.rmtree(d, ignore_errors=True)
    passed = sum(RESULTS)
    print(f"\n{passed}/{len(RESULTS)} checks passed")
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
