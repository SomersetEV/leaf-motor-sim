"""Read raw CAN logs and decode them into named signals on a 10 ms grid.

Plan section 9. Reads SavvyCAN CSV (GVRET native format) and candump logs,
decodes them with tools/dbc/leaf_sim.dbc, and reports dropped Zombie log
frames from the 0x4A1 rolling counter.

    python loglib.py LOG [--dbc FILE] [--out grid.csv]
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cantools
import numpy as np
import pandas as pd

DEFAULT_DBC = Path(__file__).resolve().parent / "dbc" / "leaf_sim.dbc"
GRID_S = 0.01

# Column names used by the other tools
SPEED = "INV_Status_1DA.MotorSpeed"
SPEED_GOV = "GOV_Speed_4A0.GovSpeedIn"
REQ_RAW = "VCM_TorqueCmd_1D4.TorqueRequestRaw"
REVLIM = "GOV_Gains_4A2.Revlim"
GOV_INT = "GOV_Terms_4A1.GovInt"
GOV_CLAMPED = "GOV_Terms_4A1.GovIntClamped"
GOV_BINDING = "GOV_Terms_4A1.GovBinding"
GOV_COUNTER = "GOV_Terms_4A1.GovCounter"
SIM_LOAD = "SIM_Fast_4B0.TorqueLoad"
SIM_EVENT = "SIM_Event_4B2.Event"


@dataclass
class Frames:
    t: np.ndarray        # seconds from the first frame
    can_id: np.ndarray   # int
    data: list           # bytes per frame
    source: str = ""


@dataclass
class Log:
    grid: pd.DataFrame                   # index t (s), one column per "Message.Signal"
    raw: dict = field(default_factory=dict)  # message name -> DataFrame at its own times
    frames: int = 0
    dropped_4a1: int = 0
    duration_s: float = 0.0
    source: str = ""

    def has(self, col: str) -> bool:
        return col in self.grid.columns and self.grid[col].notna().any()


# ---- readers ----------------------------------------------------------------

def _read_savvycan(path: Path) -> Frames:
    df = pd.read_csv(path, dtype=str, skipinitialspace=True)
    cols = {c.strip().lower(): c for c in df.columns}
    tcol = cols.get("time stamp") or cols.get("timestamp")
    if tcol is None or "id" not in cols:
        raise ValueError(f"{path}: not a SavvyCAN CSV (need 'Time Stamp' and 'ID' columns)")
    dcols = [cols[f"d{i}"] for i in range(1, 9) if f"d{i}" in cols]
    lencol = cols.get("len")
    t = pd.to_numeric(df[tcol], errors="coerce").to_numpy(dtype=float)
    # SavvyCAN writes microseconds; fall back to seconds if values look small
    t = t / 1e6 if np.nanmax(t) > 1e5 else t
    ids = df[cols["id"]].map(lambda s: int(str(s).replace("0x", ""), 16)).to_numpy()
    lens = pd.to_numeric(df[lencol], errors="coerce").fillna(8).astype(int).to_numpy() if lencol else np.full(len(df), 8)
    raw = df[dcols].fillna("0").to_numpy()
    data = [bytes(int(x, 16) for x in row[:n]) for row, n in zip(raw, lens)]
    ok = ~np.isnan(t)
    return Frames(t[ok] - t[ok][0], ids[ok], [d for d, k in zip(data, ok) if k], str(path))


_CANDUMP_LOG = re.compile(r"\(([\d.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")
_CANDUMP_TXT = re.compile(r"^\s*(?:\(([\d.]+)\)\s+)?\S+\s+([0-9A-Fa-f]+)\s+\[(\d)\]\s+((?:[0-9A-Fa-f]{2}\s*)*)")


def _read_candump(path: Path) -> Frames:
    ts, ids, data = [], [], []
    with open(path) as f:
        for line in f:
            m = _CANDUMP_LOG.search(line)
            if m:
                ts.append(float(m.group(1)))
                ids.append(int(m.group(2), 16))
                data.append(bytes.fromhex(m.group(3)))
                continue
            m = _CANDUMP_TXT.match(line)
            if m and m.group(1):
                ts.append(float(m.group(1)))
                ids.append(int(m.group(2), 16))
                data.append(bytes.fromhex(m.group(4).replace(" ", "")))
    if not ts:
        raise ValueError(f"{path}: no timestamped candump frames (use candump -l or -ta)")
    t = np.array(ts)
    return Frames(t - t[0], np.array(ids), data, str(path))


def read_frames(path) -> Frames:
    path = Path(path)
    with open(path) as f:
        head = f.readline()
    if "," in head and ("time" in head.lower() or "id" in head.lower()):
        return _read_savvycan(path)
    return _read_candump(path)


# ---- decoding ---------------------------------------------------------------

def decode(frames: Frames, db) -> dict:
    """Decodes every known message. Returns name -> DataFrame (column 't' + signals)."""
    rows: dict[str, list] = {}
    known = {m.frame_id: m for m in db.messages}
    for t, cid, d in zip(frames.t, frames.can_id, frames.data):
        msg = known.get(int(cid))
        if msg is None or len(d) < msg.length:
            continue
        try:
            sig = msg.decode(d, decode_choices=False, scaling=True)
        except Exception:
            continue
        sig["t"] = t
        rows.setdefault(msg.name, []).append(sig)
    return {name: pd.DataFrame(r) for name, r in rows.items()}


def counter_gaps(values, modulo: int = 256) -> int:
    """Frames missing from a rolling counter sequence."""
    v = np.asarray(values, dtype=int)
    if len(v) < 2:
        return 0
    steps = np.mod(np.diff(v), modulo)
    return int(np.sum(np.where(steps == 0, 0, steps - 1)))


# Continuous signals are interpolated between their own timestamps. Holding
# the last value instead would, with frames jittering around grid points,
# repeat or skip samples and put spikes into any derivative.
INTERPOLATED = {SPEED, SPEED_GOV, "SIM_Fast_4B0.ModelSpeed"}


def to_grid(raw: dict, duration_s: float, period_s: float = GRID_S,
            hold_periods: dict | None = None) -> pd.DataFrame:
    """Signals on a fixed grid: speeds interpolated, everything else the most
    recent value. Points further than three frame periods from data are NaN."""
    grid_t = np.round(np.arange(0.0, duration_s + period_s / 2, period_s), 6)
    out = pd.DataFrame({"t": grid_t})
    for name, df in raw.items():
        if df.empty:
            continue
        df = df.sort_values("t")
        period = float(np.median(np.diff(df["t"]))) if len(df) > 2 else 1.0
        tol = max(3 * period, 3 * period_s)
        if hold_periods and name in hold_periods:
            tol = hold_periods[name]
        renamed = df.rename(columns={c: f"{name}.{c}" for c in df.columns if c != "t"})
        merged = pd.merge_asof(out[["t"]], renamed, on="t", direction="backward", tolerance=tol)
        ft = df["t"].to_numpy()
        # Distance to the nearest frame, for marking interpolated gaps
        pos = np.clip(np.searchsorted(ft, grid_t), 1, len(ft) - 1) if len(ft) > 1 else None
        for c in merged.columns:
            if c == "t":
                continue
            if c in INTERPOLATED and len(ft) > 1:
                v = np.interp(grid_t, ft, renamed[c].to_numpy(dtype=float))
                gap = ft[pos] - ft[pos - 1]
                v[(gap > tol) | (grid_t < ft[0]) | (grid_t > ft[-1])] = np.nan
                out[c] = v
            else:
                out[c] = merged[c].to_numpy()
    return out.set_index("t")


def load(path, dbc=DEFAULT_DBC) -> Log:
    db = cantools.database.load_file(str(dbc), strict=True)
    frames = read_frames(path)
    raw = decode(frames, db)
    duration = float(frames.t[-1]) if len(frames.t) else 0.0
    # Events are instants, not states: never hold them on the grid
    grid = to_grid(raw, duration, hold_periods={"SIM_Event_4B2": GRID_S / 2})
    dropped = counter_gaps(raw["GOV_Terms_4A1"]["GovCounter"]) if "GOV_Terms_4A1" in raw else 0
    return Log(grid=grid, raw=raw, frames=len(frames.t), dropped_4a1=dropped,
               duration_s=duration, source=str(path))


def speed_rpm(log: Log) -> pd.Series:
    """Best available motor speed in rpm (signed where possible)."""
    if log.has(SPEED):
        return log.grid[SPEED]
    if log.has(SPEED_GOV):
        return log.grid[SPEED_GOV]
    raise ValueError(f"{log.source}: no speed signal (0x1DA or 0x4A0)")


def summary(log: Log) -> str:
    lines = [f"{log.source}: {log.frames} frames, {log.duration_s:.2f} s"]
    for name, df in sorted(log.raw.items()):
        if len(df) > 1:
            dt = np.diff(df["t"].to_numpy())
            lines.append(f"  {name:22s} {len(df):7d} frames, period {np.median(dt) * 1000:7.2f} ms "
                         f"(min {dt.min() * 1000:.2f}, max {dt.max() * 1000:.2f})")
        else:
            lines.append(f"  {name:22s} {len(df):7d} frames")
    if "GOV_Terms_4A1" in log.raw:
        lines.append(f"  dropped 0x4A1 frames (rolling counter): {log.dropped_4a1}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--dbc", default=str(DEFAULT_DBC))
    ap.add_argument("--out", help="write the 10 ms grid as CSV")
    a = ap.parse_args(argv)
    log = load(a.log, a.dbc)
    print(summary(log))
    if a.out:
        log.grid.to_csv(a.out)
        print(f"grid written to {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
