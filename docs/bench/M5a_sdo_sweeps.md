# M5a bench test: governor gains over CAN SDO, scripted sweeps

**Goal (plan section 11, M5a):** the simulator reads and writes the Zombie's governor parameters over CAN SDO, and `tools/sweep.py` runs a list of gain sets against a load unattended, writing a metrics table.

Same wiring as M3, with the Due **unpowered**: it sends error frames even in listen-only mode. The simulator's `canlog` replaces SavvyCAN, and Claude can run all of this over COM3.

## What is new

| Item | Detail |
| --- | --- |
| `zget <name>` | Reads a Zombie parameter: `ok govKp=11`. |
| `zset <name> <value>` | Reads the old value, writes, reads back: `ok govKp=14 was=11`. Sends a `0x4B2` `ZParam` event, value = parameter ID. |
| Whitelist | `revlim`, `govKp`, `govKi`, `govKd`, `govImax`, `govFlt`, `GovLog`. Anything else is refused by the simulator, and out-of-range values are refused by the Zombie (`abort 0x06090030`). |
| No saving | The simulator never sends the SDO save command. A Zombie power cycle restores the setup saved in the web interface. |
| `tools/sweep.py` | For each gain set: no-load settle, `zset` the gains, apply the load, log the bus for `run_s`, score it. Writes `metrics.csv` plus one bus log per run. Turns `GovLog` on, and writes back every original value at the end, including after an error. |

## Steps

1. **Zombie in Run, pedal clamped** at a fixed position, high enough that the governor holds `revlim` under the load (100 % is simplest).
2. **Read and write.** `zget govKp` matches the web interface. `zset govKp 12.5` returns `was=` the old value, and the web interface shows 12.5 after a refresh. `zset govKp` back to the original.
3. **Refusals.** `zset govFlt 9` returns `err Zombie refused (abort 0x06090030)`. `zset throtmax 50` returns `err not allowed`.
4. **Sweep.** From `tools/`:

   ```text
   ..\.venv\Scripts\python sweep.py scripts\sweep_kp_baler.yaml --port COM3 --out <folder>
   ```

   That runs four `govKp` values, P-only, against the placeholder `baler` preset, in about 3 minutes. Each row should show `pedal_pct` near your clamp position and `binding_frac` above 0. A `WARNING: pedal at ...` line means the pedal slipped.
5. **Restored.** After the sweep, the web interface shows the original gains, and `GovLog` back as it was.

Send back the `metrics.csv`, or let Claude run it and read it directly.

## Done so far (3 Nov 2026, Claude over COM3)

- Steps 2 and 3 pass: reads, writes, read-back, refusals, and `0x4B2` events, about 0.5 ms per SDO exchange.
- The sweep plumbing passes with no pedal: two sets, gains and `GovLog` restored. The metrics were empty, correctly flagged by `pedal_pct=0`.
- Sweep with the pedal clamped, `sweep_kp_baler.yaml`, passes: four sets, pedal steady at 91 %, governor binding throughout, no dropped `0x4A1` frames, and the gains restored. The logs are in `sweep_kp_baler_4`. On the placeholder baler, higher `govKp` cuts the droop from 129 to 58 rpm, but the per-stroke ripple rises from 232 to 380 rpm and the torque swing from 52 to 203 Nm. At no-load, Kp 14 starts to ring (45 rpm peak to peak) and Kp 18 is unstable (1350 rpm peak to peak).
