# SD card layout

Copy `presets/` and `profiles/` to the root of a FAT32 microSD card for the T-CAN485.

## Presets: `/presets/<name>.json`

A preset is a flat JSON object of numbers. A file with the same name as a built-in preset (`noload`, `topper`, `baler`, `pto_step`) overrides that preset. Any key you leave out keeps the built-in value. A file with any other name starts from the default parameters with no loads.

The console command `preset <name>` checks the SD card first, then the built-in presets.

| Key | Meaning |
| --- | --- |
| `J`, `Tc`, `b`, `c`, `tau`, `Tmax`, `Pmax`, `kT`, `udc`, `noise`, `motor_c`, `inv_c` | Model parameters, as for `set` |
| `step_nm`, `step_ramp_ms`, `step_dj` | Resistive step (on if `step_nm` > 0) |
| `pulse_nm`, `pulse_period_ms`, `pulse_duty_pct` | Half-sine pulse (on if `pulse_nm` > 0) |
| `noise_nm`, `noise_hz` | Filtered draft noise (on if `noise_nm` > 0) |
| `grad_nm` | Signed active torque |

## Profiles: `/profiles/<file>.csv`

Profiles are written by `tools/make_profile.py`. The header is `t_ms,torque_nm` with an optional `inertia_kgm2` column, and there is one row every 10 ms starting at 0. The firmware rejects any other spacing. The limit is 60,000 rows (10 minutes).

```text
profile load example     (".csv" is added if there is no extension)
profile loop 1
profile exp 1.5 2000     (optional: torque scales with (rpm / 2000)^1.5)
profile start
```
