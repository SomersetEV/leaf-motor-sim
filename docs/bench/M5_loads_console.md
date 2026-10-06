# M5 bench test: loads, presets, console, telemetry, events

**Goal (plan section 11, M5):** `load step` and `load pulse` visibly disturb speed. `0x4B0` to `0x4B2` decode through the DBC. Jitter stays inside ±0.5 ms with streaming on.

Run this after M3 passes. The wiring is the same as M3.

## What is new

- **Loads**, from the console or presets:
  - `load step <Nm> [ramp_ms] [dJ]`
  - `load pulse <Nm> <period_ms> <duty_pct>` (half-sine)
  - `load grad <Nm>` (signed; can drive the shaft)
  - `load noise <sd_Nm> [corner_hz]`
  - `load off`
- **Parameters:** `set <name> <value>` for any parameter in `params`, and `save` to keep them in flash across power cycles.
- **Presets:** `preset noload | topper | baler | pto_step`. The numbers are placeholders until M7.
- **Telemetry:** `0x4B0` (model state, 10 ms), `0x4B1` (status, 100 ms), and `0x4B2` (an event for every load, preset, profile, `mark` or `set` command).
- **Streaming:** `stream 1 [divider]` prints CSV `t_ms,raw_req,t_req_nm,t_motor_nm,t_load_nm,rpm,flags`.
- **Speed noise:** `set noise <rpm>` adds noise to the speed reported in `0x1DA` only. The model and `0x4B0` stay clean.
- **Status LED:**

  | Colour | Meaning |
  | --- | --- |
  | Blue | No valid `0x1D4` |
  | Green | Torque requests arriving |
  | Yellow | Profile replay in progress |
  | Red | Bus-off, or a tick overrun or out-of-tolerance tick, in the last second |

Commands are applied by the 10 ms tick at the start of its next tick, so a disturbance starts on a tick boundary. Its `0x4B2` event carries the time.

## Steps

Flash, open the console, and put the Zombie in Run mode with the governor holding `revlim`.

1. **Telemetry.** In SavvyCAN, with `tools/dbc/leaf_sim.dbc` loaded:
   - `SIM_Fast_4B0.ModelSpeed` matches `INV_Status_1DA.MotorSpeed`.
   - `TorqueReq` and `TorqueMotor` track the request.
   - `SIM_Slow_4B1.Inertia` = 0.300.
2. **Step.** Type `mark 1`, then `load step 60 500`. Speed dips, then the governor recovers. SavvyCAN shows a `0x4B2` event (`Load`, value 60), and `TorqueLoad` rises by 60 Nm over 0.5 s. Then `load step 0`: speed overshoots, then settles.
3. **Pulse.** `load pulse 150 700 40`: a 0.7 s ripple appears on speed. Then `load off`.
4. **Presets.** `preset baler`, then `stat`: `preset=baler pulse=on J=0.900`. Then `preset pto_step`: the inertia steps to 0.6 kg·m² once the 500 ms ramp ends. Then `preset noload`.
5. **Parameters.** `set J 0.5`, then `params` shows `J=0.5`, and SavvyCAN shows a `ParamChange` event. `save`, power-cycle the LilyGo, and check `params` still shows `J=0.5`. `set J 0.3`, then `save` again.
6. **Streaming and jitter.** `stream 1`: CSV lines arrive at 100 per second. Leave it for 10 minutes with `preset baler`, then `stream 0` and `stat`:
   - `overruns=0`, `out_of_tol=0`, and `jitter_max_us` below 500;
   - `stream_dropped` is 0, or small if the PC's serial terminal stalled.
7. **Fault.** `fault inv 1`: the Zombie reports an inverter error. `fault inv 0` clears it.
8. **LED.** Blue when the Zombie is out of Run mode, green when it is in Run mode.

Send back: the `stat` line from step 6 and a SavvyCAN capture of steps 2 to 4.
