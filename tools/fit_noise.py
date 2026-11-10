"""Fit the simulator's reported-speed noise to the ripple in tractor logs (plan I6).

The tractor's reported speed carries a ripple of roughly 6-10 Hz on top of
the real shaft motion. The governor sees it as noise, and its speed filter
passes a different share of it than of white noise, so white noise cannot
match the tractor at more than one govFlt. This fits the simulator's
band-limited noise (set noise / noise_fc / noise_bw) to the logs instead.

Method: the logs hold bursts of consecutive 10 ms 0x1DA frames. Each burst
is cut into segments of SEG ticks, and each segment has a quadratic fit
removed, which takes out the shaft motion (and with it anything below about
3 Hz). The averaged spectrum of what is left is the target. The model's
noise goes through exactly the same steps, so the two are compared like for
like, and the fit picks the band centre, bandwidth and level that match the
target best on a log scale.

The check is the noise that gets through the governor filter: the spectrum
weighted by |H(f)|^2 of IIRFILTERF at each shift, for the tractor and the
fitted band noise, and for the band noise again without the detrend. The
plan's target is the band noise within 20 % of the tractor at shifts 4 and 2,
with one setting. The last column shows why white noise cannot do that: the
white level the governor needs to see the tractor's figure differs by shift.
Below about 3 Hz the logs cannot tell ripple from shaft motion, so the figures
assume the tractor has no extra noise hidden there.

    python fit_noise.py ../logs/session_0011.csv ../logs/session_0012.csv
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import loglib
import simmodel as sm

SEG = 30            # ticks per segment; most logger bursts are 30-35 frames
MIN_RPM = 500.0     # segments slower than this are left out
MAX_STEP_RPM = 300  # a jump bigger than this in one tick is a glitch
SHIFTS = (4, 3, 2, 1)
TARGET = 0.20       # plan I6: within 20 % at shifts 4 and 2


def tractor_segments(paths, seg: int = SEG) -> list[np.ndarray]:
    out = []
    for p in paths:
        log = loglib.load(p)
        fr = log.raw.get("INV_Status_1DA")
        if fr is None:
            continue
        t = fr["t"].to_numpy()
        v = fr["MotorSpeed"].to_numpy(dtype=float)
        for s in np.split(np.arange(len(t)), np.flatnonzero(np.diff(t) > 0.015) + 1):
            for i in range(0, len(s) - seg + 1, seg):
                x = v[s[i:i + seg]]
                if np.abs(x).mean() >= MIN_RPM and np.abs(np.diff(x)).max() <= MAX_STEP_RPM:
                    out.append(x)
    return out


def detrended_psd(segs) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Frequencies, mean spectrum (rpm^2 per bin, summing to the residual
    variance) and the per-segment residual RMS."""
    segs = np.atleast_2d(np.asarray(segs, dtype=float))
    n = segs.shape[1]
    k = np.arange(n)
    basis = np.vander(k, 3)
    coef, *_ = np.linalg.lstsq(basis, segs.T, rcond=None)
    r = segs - (basis @ coef).T
    w = np.hanning(n)
    spec = np.abs(np.fft.rfft(r * w, axis=1)) ** 2
    spec[:, 1:-1] *= 2  # one-sided
    spec /= n * np.sum(w ** 2)
    return np.fft.rfftfreq(n, sm.DT), spec.mean(axis=0), r.std(axis=1)


def model_psd(fc: float, bw: float, sd: float = 1.0, nseg: int = 4000, seg: int = SEG, seed: int = 1):
    x = sm.speed_noise(nseg * seg, sd, fc, bw, np.random.default_rng(seed))
    return detrended_psd(x.reshape(nseg, seg))[1]


def passed(f, psd, shift: int) -> float:
    """RMS that gets through the governor filter at shift, from a spectrum."""
    return float(np.sqrt(np.sum(psd[1:] * sm.gov_filter_gain(f[1:], shift) ** 2)))


def features(f, psd) -> np.ndarray:
    """What the fit matches: the residual RMS, and the RMS through the
    governor filter at each shift."""
    return np.array([np.sqrt(psd[1:].sum())] + [passed(f, psd, k) for k in SHIFTS])


def true_passed(fc: float, bw: float, sd: float, shift: int) -> float:
    """RMS of the model's noise through the governor filter, with no detrend:
    what the governor would really see from the simulator."""
    from scipy.integrate import trapezoid
    from scipy.signal import freqz
    f = np.linspace(0.0, 0.5 / sm.DT, 4001)
    if fc > 0:
        b, a, _ = sm.design_bandpass(fc, bw)
        h2 = np.abs(freqz(b, a, worN=f, fs=1 / sm.DT)[1]) ** 2
    else:
        h2 = np.ones_like(f)
    g2 = sm.gov_filter_gain(f, shift) ** 2
    return float(sd * np.sqrt(trapezoid(h2 * g2, f) / trapezoid(h2, f)))


def fit(f, target):
    """Grid search over the band shape, on a log scale, with the level as the
    best common offset for each shape. It matches the tractor's features()
    twice: through the same detrend, and with the model's own noise
    undetrended (true_passed). The detrend leaves the logs blind below about
    3 Hz, and without the second term a band centred there fits the visible
    part just as well while handing the governor twice the tractor's noise
    at shift 4. Matching the spectrum bin by bin instead lets the flat floor
    above about 13 Hz, which the governor filter removes anyway, pull the
    band far too wide."""
    lt = np.log(features(f, target))
    best = None
    for fc in np.arange(3.0, 12.01, 0.25):
        for bw in np.arange(1.0, 20.01, 0.5):
            lm = np.log(features(f, model_psd(fc, bw, nseg=1500)))
            lu = np.log([true_passed(fc, bw, 1.0, k) for k in SHIFTS])
            r = np.r_[lt - lm, lt[1:] - lu]
            c = float(np.mean(r))
            err = float(np.sqrt(np.mean((r - c) ** 2)))
            if best is None or err < best[0]:
                best = (err, fc, bw, float(np.exp(c)))
    err, fc, bw, sd = best
    return {"noise": round(sd, 1), "noise_fc": float(fc), "noise_bw": float(bw), "log_rms_err": err}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="tractor logs with 0x1DA (telematics, SavvyCAN or candump)")
    ap.add_argument("--json", help="write the fitted parameters and checks here")
    a = ap.parse_args(argv)

    segs = tractor_segments(a.logs)
    if len(segs) < 20:
        print(f"only {len(segs)} usable {SEG}-tick segments; need at least 20", file=sys.stderr)
        return 1
    f, tgt, rms = detrended_psd(segs)
    print(f"{len(segs)} segments of {SEG} ticks; residual RMS median {np.median(rms):.1f} rpm, "
          f"90th percentile {np.percentile(rms, 90):.1f} rpm")

    p = fit(f, tgt)
    band = model_psd(p["noise_fc"], p["noise_bw"], p["noise"], seed=2)
    print(f"fit: set noise {p['noise']}, noise_fc {p['noise_fc']}, noise_bw {p['noise_bw']} "
          f"(RMS log error {p['log_rms_err']:.3f})")

    print("\nspectrum after the detrend, rpm^2 per bin\n  Hz   tractor   band model")
    for i in range(1, len(f)):
        print(f"{f[i]:5.1f}  {tgt[i]:8.2f}  {band[i]:10.2f}")

    checks = {}
    print("\nnoise through the governor filter, rpm RMS. 'tractor' and 'band' have the same")
    print("detrend; 'band raw' is the fitted noise as the governor gets it, with no detrend.")
    print("The last column is the white 'set noise' that gives the governor the tractor's figure.")
    print("shift  tractor   band  ratio   band raw  ratio   white to match")
    for k in SHIFTS:
        t, b = passed(f, tgt, k), passed(f, band, k)
        raw = true_passed(p["noise_fc"], p["noise_bw"], p["noise"], k)
        white_sd = t / true_passed(0.0, 0.0, 1.0, k)
        checks[k] = {"tractor": t, "band": b, "band_raw": raw, "white_sd_to_match": white_sd}
        print(f"  {k}    {t:6.2f}  {b:6.2f}  {b / t:5.2f}   {raw:6.2f}   {raw / t:5.2f}   {white_sd:6.1f}")

    ok = all(abs(checks[k]["band"] / checks[k]["tractor"] - 1) <= TARGET for k in (4, 2))
    print(f"\nband noise within {TARGET:.0%} of the tractor at shifts 4 and 2: {'yes' if ok else 'NO'}")
    if a.json:
        Path(a.json).write_text(json.dumps({"fit": p, "segments": len(segs), "checks": checks,
                                            "pass": ok}, indent=2))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
