"""Turn a working log into a load profile for the simulator's SD card.

Plan section 9 (runs R5 steady work, R6 baling, R7 PTO steps). The load is
the motor torque minus the inertial torque, less the base friction the
simulator already applies, clamped at zero:

    T_prof = max(0, T_m - J dw/dt - Tc - b|w| - c w^2)

Rows are written exactly 10 ms apart from t_ms = 0, as the firmware indexes
rows and never interpolates.

    python make_profile.py r6.csv params.json -o baler.csv [--start 30 --end 330]

Note: the profile is the load at the speed the tractor actually ran. When
testing gains that let speed move far from that, use the simulator's speed
exponent (profile exp <n> <w_ref_rpm>).
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np

import loglib
import simmodel as sm

MAX_ROWS = 60000


def profile_from_log(log: loglib.Log, params: dict, m: sm.MotorParams, cutoff_hz: float,
                     delay_ticks: int) -> np.ndarray:
    """Load torque per grid row (Nm, >= 0)."""
    g = log.grid
    if not log.has(loglib.REQ_RAW):
        raise ValueError(f"{log.source}: no 0x1D4 torque requests")
    w = loglib.speed_rpm(log).to_numpy(dtype=float) * sm.RPM_TO_RAD_S
    missing = np.isnan(w)
    if missing.all():
        raise ValueError(f"{log.source}: no speed samples")
    # Bridge gaps for the arithmetic; the load near them is zeroed below
    idx = np.arange(len(w))
    w = np.interp(idx, idx[~missing], w[~missing])
    reach = int(round(1.5 / (cutoff_hz * sm.DT))) if cutoff_hz > 0 else 1
    missing = np.convolve(missing.astype(float), np.ones(2 * reach + 1), mode="same") > 0
    t_req = sm.requested_torque(g[loglib.REQ_RAW].to_numpy(), m.kT, delay_ticks)
    t_m = sm.motor_torque_tick_avg(t_req, w, m)
    w_prev = np.concatenate([[w[0]], w[:-1]])
    w_mid = 0.5 * (w + w_prev)
    inertial = params["J"] * sm.backward_derivative(w)
    fric = sm.friction(w_mid, params["Tc"], params["b"], params["c"])
    load = sm.zero_phase(t_m - inertial - fric, cutoff_hz)
    # At standstill the request does not move the shaft; the load is unknown
    load = np.where(np.abs(w_mid) < 1.0, 0.0, load)
    load[missing] = 0.0
    return np.maximum(load, 0.0)


def write_profile(path, torque: np.ndarray, inertia: float | None = None) -> None:
    with open(path, "w", newline="\n") as f:
        f.write("t_ms,torque_nm" + (",inertia_kgm2" if inertia else "") + "\n")
        for i, v in enumerate(torque):
            f.write(f"{i * 10},{v:.2f}" + (f",{inertia:.4f}" if inertia else "") + "\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("params", help="params.json from extract_params.py")
    ap.add_argument("-o", "--out", required=True, help="profile CSV for /profiles/ on the SD card")
    ap.add_argument("--start", type=float, default=0.0, help="start time in the log, s")
    ap.add_argument("--end", type=float, default=None, help="end time in the log, s")
    ap.add_argument("--cutoff", type=float, default=10.0, help="zero-phase filter cutoff, Hz")
    ap.add_argument("--delay-ticks", type=int, default=None,
                    help="request shift in ticks (default: the value extract_params chose)")
    ap.add_argument("--with-inertia", action="store_true", help="add the fitted J as an inertia column")
    ap.add_argument("--dbc", default=str(loglib.DEFAULT_DBC))
    a = ap.parse_args(argv)

    params = json.load(open(a.params))
    m = sm.MotorParams(params.get("kT", 0.25), params.get("tau", 30.0),
                       params.get("Tmax", 280.0), params.get("Pmax", 80.0))
    delay = a.delay_ticks if a.delay_ticks is not None else params.get("fit", {}).get("delay_ticks", 0)
    log = loglib.load(a.log, a.dbc)
    print(loglib.summary(log))
    load = profile_from_log(log, params, m, a.cutoff, delay)
    t = log.grid.index.to_numpy()
    sel = (t >= a.start) & ((t <= a.end) if a.end is not None else True)
    load = load[sel]
    if len(load) > MAX_ROWS:
        print(f"warning: {len(load)} rows, truncated to {MAX_ROWS} (10 minutes)")
        load = load[:MAX_ROWS]
    write_profile(a.out, load, params["J"] if a.with_inertia else None)
    print(f"\n{len(load)} rows ({len(load) / 100:.1f} s), mean {load.mean():.1f} Nm, "
          f"peak {load.max():.1f} Nm, written {a.out}")
    print("The profile is the load at the speed the tractor ran; use 'profile exp' "
          "when testing gains that move speed far from it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
