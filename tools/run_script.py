"""Send timed console commands to the simulator, so bench tests repeat exactly.

Plan section 9 and 10. A YAML script lists commands with their start time
in seconds; each must get an "ok" reply. CSV stream lines arriving in the
meantime can be saved with --stream-out, and canlog bus frames (candump -l
format, readable by loglib) with --canlog-out.

    python run_script.py test.yaml [--port COM7] [--dry-run] [--log run.txt]

Example script:

    port: COM7            # optional; --port overrides
    baud: 921600
    steps:
      - {at: 0.0,  cmd: "preset baler"}
      - {at: 0.5,  cmd: "stream 1 1"}
      - {at: 1.0,  cmd: "mark 1"}
      - {at: 20.0, cmd: "load step 60 500"}
      - {at: 40.0, cmd: "load off"}
      - {at: 45.0, cmd: "stream 0"}
"""

from __future__ import annotations

import argparse
import sys
import time

import yaml


class ScriptError(Exception):
    pass


def load_script(text: str) -> dict:
    s = yaml.safe_load(text) or {}
    steps = s.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ScriptError("script needs a non-empty 'steps' list")
    last = -1.0
    for i, st in enumerate(steps):
        if not isinstance(st, dict) or "at" not in st or "cmd" not in st:
            raise ScriptError(f"step {i + 1}: needs 'at' and 'cmd'")
        at = float(st["at"])
        if at < last:
            raise ScriptError(f"step {i + 1}: times must not go backwards")
        last = at
    return s


def run(script: dict, port, log=print, stream_out=None, reply_timeout_s=None,
        clock=time.monotonic, sleep=time.sleep, canlog_out=None) -> int:
    """Runs the steps against port (write(bytes), readline() -> bytes with a
    short timeout). Returns the number of failed commands."""
    timeout = reply_timeout_s or float(script.get("reply_timeout_s", 2.0))
    failures = 0
    t0 = clock()

    def handle_line(raw: bytes):
        line = raw.decode("ascii", errors="backslashreplace").strip()
        if not line:
            return None
        if line[0] == "(":  # canlog frame, candump -l format
            if canlog_out:
                canlog_out.write(line + "\n")
            return None
        if line[0].isdigit() or line.startswith("t_ms,"):
            if stream_out:
                stream_out.write(line + "\n")
            return None
        return line

    for st in script["steps"]:
        at = float(st["at"])
        while clock() - t0 < at:
            raw = port.readline()  # keep draining while waiting
            line = handle_line(raw)
            if line:
                log(f"{clock() - t0:8.3f}  < {line}")
            remaining = at - (clock() - t0)
            if not raw and remaining > 0.05:  # sleep only when the port is idle
                sleep(min(0.01, remaining))
        cmd = str(st["cmd"]).strip()
        log(f"{clock() - t0:8.3f}  > {cmd}")
        port.write((cmd + "\n").encode())
        deadline = clock() + timeout
        reply = None
        while clock() < deadline:
            line = handle_line(port.readline())
            if line is None:
                continue
            log(f"{clock() - t0:8.3f}  < {line}")
            if line.startswith("ok") or line.startswith("err"):
                reply = line
                break
        if reply is None or reply.startswith("err"):
            failures += 1
            log(f"          {'no reply' if reply is None else 'error'} for '{cmd}'")
            if not st.get("allow_err", False):
                raise ScriptError(f"'{cmd}' failed: {reply or 'no reply'}")
    return failures


class LinePort:
    """Line reader over a pyserial port that reads whatever is waiting in one
    call. pyserial's own readline reads a byte at a time, too slow for canlog
    at several hundred lines a second."""

    def __init__(self, port):
        self.port = port
        self.buf = b""
        self.lines = []

    def readline(self) -> bytes:
        if not self.lines:
            chunk = self.port.read(self.port.in_waiting or 1)  # waits up to the timeout
            if chunk:
                parts = (self.buf + chunk).split(b"\n")
                self.buf = parts.pop()
                self.lines = [p + b"\n" for p in parts]
        return self.lines.pop(0) if self.lines else b""

    def write(self, data: bytes):
        return self.port.write(data)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("script")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--dry-run", action="store_true", help="print the schedule only")
    ap.add_argument("--log", help="also write the console transcript here")
    ap.add_argument("--stream-out", help="write streamed CSV lines here")
    ap.add_argument("--canlog-out", help="write canlog frames here (candump -l format)")
    ap.add_argument("--zombie", help="--port sil: Stm32-vcu tree (default ZOMBIE_DIR or ../Stm32-vcu)")
    ap.add_argument("--sd", help="--port sil: folder standing in for the SD card")
    ap.add_argument("--pedal", type=float, help="--port sil: the Zombie's pedal in percent (default 100)")
    a = ap.parse_args(argv)

    script = load_script(open(a.script).read())
    if a.dry_run:
        for st in script["steps"]:
            print(f"{float(st['at']):8.3f}  {st['cmd']}")
        return 0

    port_name = a.port or script.get("port")
    if not port_name:
        print("error: no serial port (use --port or 'port:' in the script)", file=sys.stderr)
        return 2

    logf = open(a.log, "w") if a.log else None
    streamf = open(a.stream_out, "w") if a.stream_out else None
    canlogf = open(a.canlog_out, "w") if a.canlog_out else None

    def log(msg):
        print(msg)
        if logf:
            logf.write(msg + "\n")

    if str(port_name).lower() == "sil":  # the PC harness, in simulated time
        from silport import SilError, SilPort
        try:
            with SilPort(a.zombie, a.sd, a.pedal) as sil:
                log(sil.banner)
                failures = run(script, sil, log, streamf, canlog_out=canlogf,
                               clock=sil.clock, sleep=sil.sleep)
                log(f"done, {failures} command(s) failed")
        except (SilError, ScriptError) as e:
            log(f"stopped: {e}")
            return 1
        finally:
            for f in (logf, streamf, canlogf):
                if f:
                    f.close()
        return 1 if failures else 0

    import serial  # pyserial

    # Open with DTR and RTS released: asserting them on open can pulse the
    # ESP32's auto-reset circuit and reboot the simulator mid-test.
    port = serial.Serial(baudrate=a.baud or int(script.get("baud", 921600)), timeout=0.02)
    port.port = port_name
    port.dtr = False
    port.rts = False
    port.open()
    with port:
        port.reset_input_buffer()
        try:
            failures = run(script, LinePort(port), log, streamf, canlog_out=canlogf)
            log(f"done, {failures} command(s) failed")
        except ScriptError as e:
            log(f"stopped: {e}")
            return 1
        finally:
            if logf:
                logf.close()
            if streamf:
                streamf.close()
            if canlogf:
                canlogf.close()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
