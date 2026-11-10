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
    kT: float = 0.25        # Nm per 0x1D4 bit (plan V1, settled)
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


# --- Reported speed noise, mirroring lib/sensor/speed_report.cpp (plan I6) ---

def design_bandpass(fc_hz: float, bw_hz: float, fs_hz: float = 1 / DT):
    """Band-pass biquad (RBJ, 0 dB peak gain) as in lib/sensor: returns
    (b, a, out_sd), with out_sd the output SD for unit white input."""
    w0 = 2 * np.pi * fc_hz / fs_hz
    alpha = np.sin(w0) / (2 * fc_hz / bw_hz)
    a0 = 1 + alpha
    b0, a1, a2 = alpha / a0, -2 * np.cos(w0) / a0, (1 - alpha) / a0
    g0 = (1 + a2) / ((1 - a2) * ((1 + a2) ** 2 - a1 ** 2))
    g1 = -a1 * g0 / (1 + a2)
    g2 = -a1 * g1 - a2 * g0
    out_sd = float(np.sqrt(b0 * b0 * 2 * (g0 - g2)))
    return np.array([b0, 0.0, -b0]), np.array([1.0, a1, a2]), out_sd


def speed_noise(n: int, sd_rpm: float, fc_hz: float = 0.0, bw_hz: float = 4.0,
                rng: np.random.Generator | None = None) -> np.ndarray:
    """n ticks of the simulator's reported-speed noise: white when fc_hz is 0
    (or at or above Nyquist), otherwise band-limited, scaled to sd_rpm."""
    from scipy.signal import lfilter
    rng = rng or np.random.default_rng(0x5EED1DA1)
    x = rng.standard_normal(n)
    if sd_rpm <= 0:
        return np.zeros(n)
    if fc_hz <= 0 or bw_hz <= 0 or fc_hz >= 0.5 / DT:
        return sd_rpm * x
    b, a, out_sd = design_bandpass(fc_hz, bw_hz)
    return sd_rpm * lfilter(b, a, x) / out_sd


def gov_filter_gain(f_hz, shift: int) -> np.ndarray:
    """|H(f)| of the Zombie governor's speed filter, IIRFILTERF with weight
    1/2^shift per 10 ms tick."""
    k = 1.0 / (1 << shift)
    z = np.exp(-2j * np.pi * np.asarray(f_hz, dtype=float) * DT)
    return np.abs(k / (1 - (1 - k) * z))
