"""Model equations shared by the PC tools, matching lib/plant (plan section 6).

Signals are on the 10 ms grid from loglib. Which request row drove which
speed change depends on where the frames fall relative to the grid and on
the inverter's latency, so the tools shift the request by delay_ticks
(positive = later) and extract_params picks the shift that fits best.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, filtfilt

RPM_TO_RAD_S = 2 * np.pi / 60
DT = 0.01


@dataclass
class MotorParams:
    kT: float = 0.25        # Nm per 0x1D4 bit (plan V1, VERIFY)
    tau_ms: float = 30.0    # torque lag
    tmax_nm: float = 280.0  # plan V2, VERIFY
    pmax_kw: float = 80.0   # plan V2, VERIFY


def requested_torque(raw: np.ndarray, kT: float, delay_ticks: int = 0) -> np.ndarray:
    """Request in Nm per row; missing requests (no 0x1D4) count as zero."""
    t = np.nan_to_num(np.asarray(raw, dtype=float)) * kT
    if delay_ticks > 0:
        t = np.concatenate([np.zeros(delay_ticks), t[:-delay_ticks]])
    elif delay_ticks < 0:
        t = np.concatenate([t[-delay_ticks:], np.zeros(-delay_ticks)])
    return t


SUBSTEPS = 10


def motor_torque_tick_avg(t_req: np.ndarray, w: np.ndarray, m: MotorParams) -> np.ndarray:
    """Average motor torque over each tick: envelope, then first-order lag.

    Reproduces lib/plant: ten 1 ms sub-steps per tick, each updating the lag
    and then applying the new value for the whole sub-step. The envelope uses
    the speed at the start of the tick.
    """
    t_req = np.asarray(t_req, dtype=float)
    w = np.nan_to_num(np.asarray(w, dtype=float))
    pmax = m.pmax_kw * 1000.0
    out = np.zeros_like(t_req)
    tm = 0.0
    a = 1.0 - np.exp(-(DT / SUBSTEPS) / (m.tau_ms / 1000.0)) if m.tau_ms > 0 else 1.0
    # After n sub-steps from tm towards tc: tc + (tm - tc) * (1 - a)^n
    r = (1.0 - a) ** np.arange(1, SUBSTEPS + 1)
    r_mean, r_end = r.mean(), r[-1]
    for k in range(len(t_req)):
        aw = abs(w[k - 1]) if k > 0 else 0.0
        lim = m.tmax_nm if aw * m.tmax_nm <= pmax else pmax / aw
        tc = min(max(t_req[k], -lim), lim)
        out[k] = tc + (tm - tc) * r_mean
        tm = tc + (tm - tc) * r_end
    return out


def zero_phase(x: np.ndarray, cutoff_hz: float) -> np.ndarray:
    """Zero-phase low-pass (Butterworth, filtfilt) on a 10 ms grid."""
    if cutoff_hz <= 0 or cutoff_hz >= 0.5 / DT:
        return np.asarray(x, dtype=float)
    b, a = butter(2, cutoff_hz / (0.5 / DT))
    return filtfilt(b, a, np.asarray(x, dtype=float))


def backward_derivative(w: np.ndarray) -> np.ndarray:
    d = np.zeros_like(w, dtype=float)
    d[1:] = np.diff(w) / DT
    return d


def friction(w: np.ndarray, tc: float, b: float, c: float) -> np.ndarray:
    """Signed resistive torque Tc + b|w| + c w^2, opposing rotation."""
    aw = np.abs(w)
    return np.sign(w) * (tc + b * aw + c * aw * aw)
