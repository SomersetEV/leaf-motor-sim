"""Send timed console commands to the simulator, so bench tests repeat exactly.

Plan section 9 and 10. A YAML script lists commands with their start time
in seconds; each must get an "ok" reply. CSV stream lines arriving in the
meantime can be saved with --stream-out.

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
        clock=time.monotonic, sleep=time.sleep) -> int:
    """Runs the steps against port (write(bytes), readline() -> bytes with a
    short timeout). Returns the number of failed commands."""
    timeout = reply_timeout_s or float(script.get("reply_timeout_s", 2.0))
    failures = 0
    t0 = clock()

    def handle_line(raw: bytes):
        line = raw.decode(errors="replace").strip()
        if not line:
            return None
        if line[0].isdigit() or line.startswith("t_ms,"):
            if stream_out:
                stream_out.write(line + "\n")
            return None
        return line

    for st in script["steps"]:
        at = float(st["at"])
        while clock() - t0 < at:
            line = handle_line(port.readline())  # keep draining while waiting
            if line:
                log(f"{clock() - t0:8.3f}  < {line}")
            remaining = at - (clock() - t0)
            if remaining > 0.05:
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("script")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--dry-run", action="store_true", help="print the schedule only")
    ap.add_argument("--log", help="also write the console transcript here")
    ap.add_argument("--stream-out", help="write streamed CSV lines here")
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
    import serial  # pyserial

    logf = open(a.log, "w") if a.log else None
    streamf = open(a.stream_out, "w") if a.stream_out else None

    def log(msg):
        print(msg)
        if logf:
            logf.write(msg + "\n")

    with serial.Serial(port_name, a.baud or int(script.get("baud", 921600)), timeout=0.02) as port:
        port.reset_input_buffer()
        try:
            failures = run(script, port, log, streamf)
        except ScriptError as e:
            log(f"stopped: {e}")
            return 1
        finally:
            if logf:
                logf.close()
            if streamf:
                streamf.close()
    log(f"done, {failures} command(s) failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
