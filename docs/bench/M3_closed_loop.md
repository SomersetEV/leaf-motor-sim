# M3 bench test: closed loop

**Goal (plan section 11, M3):** the Zombie completes precharge and reaches run mode. Throttle raises the Zombie's `speed`. Releasing it lets speed fall. The governor holds `revlim`.

Do the M2 bench test first ([M2_can_bringup.md](M2_can_bringup.md)). M3 uses the same wiring and adds the driver inputs.

## What the simulator does now

Each 10 ms tick, the simulator turns the Zombie's `0x1D4` torque request into Nm (raw × 0.25). It clips that to 280 Nm or 80 kW, lags it by 30 ms, and drives a 0.30 kg·m² inertia against 5 Nm of constant friction plus 0.02 Nm·s/rad viscous drag. The resulting speed goes back to the Zombie in `0x1DA`. With no valid `0x1D4` for 100 ms, the requested torque drops to zero.

All of these numbers are placeholders until the tractor logs (M7) replace them. With them, expect:

| Behaviour | Placeholder model |
| --- | --- |
| Smallest request that turns the shaft | Above 5 Nm, about 20 counts, about 1 % torque |
| Full torque | 280 Nm, reached at about 55 % torque request (above that the request is clipped, `clip=1`) |
| No-load acceleration at full torque | About 9000 rpm/s, so 0 to 2000 rpm in about a quarter of a second |
| Power limit starts at | About 2700 rpm |
| Coast from 2000 rpm to stop | About 9 s |

No-load acceleration is very fast with this inertia. Expect a large overshoot the first time the governor limit is hit. That shows the model is working, not that it matches the tractor.

## 1. Wire

As in M2, plus:

| Item | Connection |
| --- | --- |
| Throttle pot | Zombie throttle 1 input, as on the tractor |
| Ignition, start, direction, brake switches | Zombie digital inputs at 12 V |
| Contactor and precharge outputs | Not connected (indicator lamps optional) |

## 2. Zombie parameters

Start from the tractor's parameter file and the M2 settings, then set:

| Parameter | Value | Why |
| --- | --- | --- |
| `udcsw` | 300 | Below the simulated 360 V, so precharge completes |
| `udcmin` | 250 | Below 360 V, so there is no voltage derate |
| `udclim` | 400 | Above 360 V |
| `revlim` | 1500 to start, then the tractor's working value | Governor setpoint |

Precharge needs `udc >= udcsw`, the throttle released, and `udc < udclim`. The simulator reports 360 V from power-up, so precharge should finish as soon as the Zombie starts it.

Plan item V7 is still open: which inputs the tractor's vehicle class needs before it allows run mode. If the Zombie will not leave precharge or standby, note `opmode` and the input states in the web interface.

## 3. Flash

```text
pio run -e tcan485 -t upload
pio device monitor -e tcan485
```

The banner should read `leaf-motor-sim M3 closed loop`. Type `params` to confirm the model values above.

## 4. Test steps

1. **Reach run mode.** Ignition on, brake on, select direction, start. Check:
   - The web interface shows `opmode` = Run.
   - `stat` shows `rx1d4` increasing, `rx_timeout=0`, `hv=7` and `rej1d4=0`.
   - SavvyCAN shows `0x1D4` and `0x11A` every 10 ms.
2. **Throttle raises speed.** Press the throttle gently. The Zombie's `speed` and SavvyCAN's `MotorSpeed` rise, and `stat` shows `t_req` and `t_motor` positive.
3. **Release lets speed fall.** Release the throttle. Speed falls smoothly to zero in roughly 9 s from 2000 rpm. If the Zombie applies regen, it falls faster. Speed must not go negative or oscillate around zero.
4. **Governor holds `revlim`.** Press the throttle fully. Speed rises, overshoots, then settles near `revlim`. In `stat`, `t_req` drops back as the governor takes over. Hold for 30 s.
5. **Reverse (optional).** Select reverse and press the throttle. Speed should go negative, unless `reversemotor` flips it.
6. **Timeout.** With the shaft turning, switch the Zombie off or leave run mode. Within 100 ms, `rx_timeout=1` and `t_req=0`, and the speed coasts down.

Throughout, `overruns` and `out_of_tol` stay at 0 and `jitter_1s_us` stays below 500.

## 5. If it fails

| Symptom | Likely cause |
| --- | --- |
| Zombie stays in precharge, then shows `MOD_PCHFAIL` | `udcsw` above 360, `udclim` below 360, or the throttle is not released. Check `udc` = 360 in the web interface. |
| `rx1d4` stays at 0 in run mode | `InverterCan` is set to the other bus. |
| `rej1d4` increasing | Corrupted frames. Check wiring and termination, and send me a SavvyCAN capture. |
| Speed rises with the throttle released | The Zombie is still sending positive torque. Check `req_raw` in `stat` and the Zombie's `potnom`. |
| Speed hunts around `revlim` | Possibly real governor behaviour with placeholder model values. Capture it and send it to me. Tuning comes after validation (M8). |

Send back: a SavvyCAN capture (CSV) of steps 2 to 4, a `stat` line taken during step 4, and the Zombie's governor parameters (`revlim`, `govKp`, `govKi`, `govKd`, `govImax`).
