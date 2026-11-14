# Road runs D1–D3: plan, 14 Nov 2026

For now Ben can only run the tractor on the road. He can run implements empty when he has them. These road runs keep M7 moving until then. Field sheet: `M7_road_D1_D3_field_sheet.pdf` (`make_road_sheet.py`). PTO ratio: 1800 motor rpm is 540 rpm at the PTO.

## Why road runs are worth doing

The loop model's central claim (plan section 16) is that the usable `govKp` scales with inertia: about 10.5 × J for 45° of phase margin at `govFlt` 4. Only one inertia is measured so far: the neutral shaft at J 0.055 (13 Nov). On the road, the inertia the motor sees is the tractor's mass through the gearbox, so it changes with gear:

J ≈ m · (v / ω)², where v is road speed and ω is motor speed. For 2.5 t, with v the speed at 1800 rpm:

| km/h at 1800 rpm | J kg·m² | Kp 11, shift 4: phase margin | Each ring cycle keeps |
| --- | --- | --- | --- |
| 25 | 3.4 | 73° | 0.00 |
| 15 | 1.2 | 50° | 0.04 |
| 10 | 0.54 | 26° | 0.26 |
| 8 | 0.35 | 12° | 0.55 |
| 6 | 0.20 | −5° (unstable) | 1.23 |
| 4 | 0.09 | −28° | 3.2 |

Three gears give three points on that curve. They span the stability boundary at today's gains, and they bracket the PTO case, whose inertia is still unknown (R3). Agreement on the road means the model can be trusted for PTO work before any implement is measured.

The 7 Nov road logs gave J 2.7–3.8 from a rough fit between logger bursts, gear unknown. That is consistent with a road gear at about 25 km/h.

## The runs

| Run | What | Gives |
| --- | --- | --- |
| D1 | Fixed pedal from about 500 to 1800 rpm, then coast in gear (`regenmax` 0 if the road allows). Gears A (road) and B (about 10 km/h at 1800), twice each way. | J per gear, which also checks the noted gear speeds; rolling plus driveline drag. Works on bursty logs. |
| D2 | Pedal step to the limit and hold 10 s. Gear A at full pedal, B at about 25 %, C (6–8 km/h) at about 12 %. Three each. Abort above 2700 rpm or on a hard surge. | Overshoot and hunting at three inertias; validation step 2 on the road. Needs continuous logs. |
| D3 | Gear A, full pedal at the limit, 20 s up and 20 s down a steady hill. Twice each way. | Droop and regen at the limit; a gradient load profile for the bench road sweep (`sweep_road.yaml`). Needs continuous logs. |
| R4 at 6 % (optional, parked) | As on the R1/R2/R4 sheet | V9 (inverter torque in fast transients) |

## Harness predictions used for the pedal steps

The Zombie's governor (`a05e984`) was run against the `noload` shaft with J set per gear and 20 Nm of rolling load: `revlim` 2200, Kp 11, Ki 0, `govFlt` 4, pedal stepped from rest.

| km/h at 1800 | J | Pedal | Peak rpm | Held, 8–12 s (rpm) |
| --- | --- | --- | --- | --- |
| 25 | 3.39 | 100 | 2138 (P-only droop) | 2138 |
| 15 | 1.22 | 100 | 2322 | 2138 |
| 10 | 0.54 | 100 / 25 | 2771 / 2384 | 2138 |
| 8 | 0.35 | 100 / 25 / 12 | 3166 / 2555 / 2256 | about 2138 |
| 6 | 0.20 | 25 / 12 | 2913 / 2364 | hunts 1588–2635 / 1982–2279 |

The model gives the full requested torque. In neutral the real inverter delivered 40–70 % of it in fast transients (V9), so these are upper bounds. The model also has no tyre or shaft compliance; if the tractor hunts where the single-inertia model doesn't, that is the two-mass case in plan section 10.

## What the logs will be used for

1. **D1:** J and drag per gear, with the burst-wise fit (`analyse_r1r2r4.py`, generalised). Then a road preset to replace the guesses in `sweep_road.yaml` (J 2.5, rolling 20 Nm).
2. **D2:** each run repeated in the harness and on the bench with the logged gains and pedal. Compare overshoot, ring period and hunting per gear; plan section 10 validation step 2 sets the tolerances.
3. **D3:** `make_profile.py` gives the gradient load; then re-run the road sweep against it.
4. **With these fitted, R3** (PTO engaged, implement empty) needs only its own inertia to place the PTO case on the same curve.
