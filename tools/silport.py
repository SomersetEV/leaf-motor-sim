"""The software-in-the-loop harness (tools/sil, plan section 16) as a console
port, so sweep.py and run_script.py drive it with --port sil as they drive
the board.

The harness runs in simulated time. SilPort gives the tools a clock and a
sleep that advance it: sleep(dt) runs the ticks that fall in dt, and a
readline() with nothing waiting counts as 1 ms of waiting, as on a real port.
Replies arrive at once, so a sweep runs as fast as the PC can tick.

build() compiles the harness with g++ from this repo's lib/ and the Zombie
tree, into tools/sil/build/, and only when a source has changed. The Zombie
tree comes from --zombie or ZOMBIE_DIR (default: ../Stm32-vcu next to this
repo). Its commit, with '+dirty' for uncommitted changes to its sources, is
passed to the harness and printed in its first line.

    python silport.py --golden        # the Zombie's governor golden test
    python silport.py                 # interactive console, for trying things
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from collections import deque
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
ROOT = TOOLS.parent
SIL = TOOLS / "sil"
BUILD = SIL / "build"
TICK_S = 0.01
WARMUP_S = 10.0  # spin-up at the starting gains before the first command

OUR_LIBS = ["leafcodec", "loads", "plant", "sensor", "simcore", "simengine", "simproto"]
OUR_SRCS = ["lib/simengine/sim_engine.cpp", "lib/plant/plant.cpp", "lib/loads/loads.cpp",
            "lib/loads/profile.cpp", "lib/sensor/speed_report.cpp", "lib/leafcodec/leafcodec.cpp",
            "lib/simcore/cmd_latch.cpp", "lib/simcore/json_flat.cpp", "lib/simcore/param_table.cpp",
            "lib/simcore/presets.cpp", "lib/simproto/commands.cpp", "lib/simproto/console_parse.cpp",
            "lib/simproto/telemetry.cpp", "lib/simproto/zombie_sdo.cpp", "tools/sil/sil_main.cpp"]
ZOMBIE_SRCS = ["src/throttle.cpp", "src/govlog.cpp", "libopeninv/src/params.cpp"]
ZOMBIE_C_SRCS = ["libopeninv/src/my_string.c"]


class SilError(Exception):
    pass


def zombie_dir(arg: str | None = None) -> Path:
    d = Path(arg or os.environ.get("ZOMBIE_DIR") or ROOT.parent / "Stm32-vcu")
    if not (d / "src" / "throttle.cpp").exists():
        raise SilError(f"no Zombie tree at {d} (use --zombie or ZOMBIE_DIR)")
    if not (d / "libopeninv" / "src" / "params.cpp").exists():
        raise SilError(f"{d}/libopeninv is empty: clone Stm32-vcu with --recurse-submodules")
    return d


def zombie_commit(zdir: Path) -> str:
    try:
        h = subprocess.run(["git", "-C", str(zdir), "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(zdir), "diff", "--quiet", "HEAD", "--",
                                "src", "include", "libopeninv"]).returncode != 0
        return h + ("+dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _compiler(name: str) -> str:
    exe = shutil.which(name)
    if not exe:
        raise SilError(f"{name} not found on PATH (see CLAUDE.md: WinLibs on Windows)")
    return exe


def build(zdir: Path, log=print) -> Path:
    """Compiles the harness if any source is newer than it. Returns its path."""
    gxx, gcc = _compiler("g++"), _compiler("gcc")
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / ("sil.exe" if os.name == "nt" else "sil")
    stamp = BUILD / "zombie_dir.txt"
    ours = [ROOT / s for s in OUR_SRCS] + [SIL / "zombie_side.cpp"]
    zsrc = [zdir / s for s in ZOMBIE_SRCS + ZOMBIE_C_SRCS]
    headers = [p for d in [ROOT / "include", SIL, *[ROOT / "lib" / l for l in OUR_LIBS],
                           zdir / "include", zdir / "libopeninv" / "include", zdir / "test"]
               for p in d.glob("*.h")] + [ROOT / "src" / "sim_defaults.h"]
    newest = max(p.stat().st_mtime for p in ours + zsrc + headers)
    if exe.exists() and exe.stat().st_mtime >= newest and stamp.exists() \
            and stamp.read_text() == str(zdir.resolve()):
        return exe
    log(f"building the sil harness against {zdir} ...")
    our_inc = [f"-I{ROOT / 'include'}"] + [f"-I{ROOT / 'lib' / l}" for l in OUR_LIBS] + [f"-I{SIL}"]
    z_inc = [f"-I{zdir / 'include'}", f"-I{zdir / 'libopeninv' / 'include'}", f"-I{zdir / 'test'}"]
    objs = []

    def cc(tool, src, flags, out):
        r = subprocess.run([tool, "-O2", *flags, "-c", str(src), "-o", str(out)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            raise SilError(f"compiling {src} failed:\n{r.stderr[-3000:]}")
        objs.append(out)

    for s in OUR_SRCS:
        cc(gxx, ROOT / s, ["-std=c++17", "-Wall", *our_inc], BUILD / (Path(s).stem + ".o"))
    cc(gxx, SIL / "zombie_side.cpp", z_inc, BUILD / "zombie_side.o")
    for s in ZOMBIE_SRCS:
        cc(gxx, zdir / s, z_inc, BUILD / ("z_" + Path(s).stem + ".o"))
    for s in ZOMBIE_C_SRCS:
        cc(gcc, zdir / s, ["-std=c99", *z_inc], BUILD / ("z_" + Path(s).stem + ".o"))
    r = subprocess.run([gxx, *map(str, objs), "-o", str(exe)], capture_output=True, text=True)
    if r.returncode != 0:
        raise SilError(f"linking the harness failed:\n{r.stderr[-3000:]}")
    stamp.write_text(str(zdir.resolve()))
    return exe


def golden(zdir: Path, log=print) -> bool:
    exe = build(zdir, log)
    r = subprocess.run([str(exe), "--golden"], capture_output=True, text=True)
    log(r.stdout.strip() or r.stderr.strip())
    return r.returncode == 0


class SilPort:
    """A harness process with the LinePort interface (write, readline) and a
    simulated clock (clock, sleep) for the tools' Console and run()."""

    def __init__(self, zombie: str | None = None, sd: str | None = None, pedal: float | None = None,
                 log=print, warmup_s: float = WARMUP_S):
        zdir = zombie_dir(zombie)
        exe = build(zdir, log)
        self.commit = zombie_commit(zdir)
        args = ["--zombie-commit", self.commit]
        if sd:
            args += ["--sd", str(sd)]
        if pedal is not None:
            args += ["--pedal", str(pedal)]
        self.proc = subprocess.Popen([str(exe), *args], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.t = 0.0
        self.ticks = 0
        self.q: deque[bytes] = deque()
        self.banner = b"\n".join(self._collect()).decode()
        # On the bench the motor is already turning at revlim when a script
        # starts. Spin up from rest at the starting gains (P only), and let
        # the overshoot die away, before the tools take over.
        self.sleep(warmup_s)
        self.q.clear()

    def _collect(self) -> list[bytes]:
        out = []
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise SilError(f"the harness exited (code {self.proc.poll()})")
            line = line.rstrip(b"\r\n")
            if line == b"#end":
                return out
            out.append(line)

    def _send(self, line: str):
        self.proc.stdin.write((line + "\n").encode())
        self.proc.stdin.flush()
        self.q.extend(x + b"\n" for x in self._collect())

    # LinePort interface
    def write(self, data: bytes):
        for line in data.decode().splitlines():
            if line.strip():
                self._send(line.strip())
        return len(data)

    def readline(self) -> bytes:
        if self.q:
            return self.q.popleft()
        self.sleep(0.001)  # nothing waiting: a millisecond passes, as on a port
        return self.q.popleft() if self.q else b""

    # Simulated time
    def clock(self) -> float:
        return self.t

    def sleep(self, dt: float):
        self.t += max(0.0, dt)
        n = int(self.t / TICK_S + 1e-9) - self.ticks
        if n > 0:
            self._send(f"#run {n}")
            self.ticks += n

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=10)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zombie", help="Stm32-vcu tree (default ZOMBIE_DIR or ../Stm32-vcu)")
    ap.add_argument("--golden", action="store_true", help="run the governor golden test and exit")
    ap.add_argument("--sd", help="folder standing in for the SD card (presets/, profiles/)")
    a = ap.parse_args(argv)
    try:
        if a.golden:
            return 0 if golden(zombie_dir(a.zombie)) else 1
        with SilPort(a.zombie, a.sd) as p:
            print(p.banner)
            for line in sys.stdin:
                p.write(line.encode())
                while p.q:
                    print(p.q.popleft().decode().rstrip())
    except SilError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
