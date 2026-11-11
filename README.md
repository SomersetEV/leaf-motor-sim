# leaf-motor-sim

Nissan Leaf inverter and motor simulator for a LilyGo T-CAN485, used to test the ZombieVerter RPM governor on the bench. The full plan is in [docs/SIMULATOR_PLAN.md](docs/SIMULATOR_PLAN.md).

## Build and test

```text
pio test -e native          # unit tests on the PC (lib/)
pio run -e tcan485          # firmware
pio run -e tcan485 -t upload
pio device monitor -e tcan485
python tools/selftest.py    # PC tools, see tools/README.md
```

Run these from PowerShell or the PlatformIO terminal in VS Code. The pioarduino platform's tool installer refuses to run under Git Bash (MSYS). It also fails if `IDF_PATH` points to an ESP-IDF install that does not exist.

## Layout

| Path | Contents |
| --- | --- |
| `lib/leafcodec` | `0x1D4` decode, `0x1DA` and `0x55A` encode, Nissan CRC |
| `lib/plant` | Single-inertia motor model: torque envelope, torque lag, friction, standstill hold |
| `lib/loads` | Step, pulse, noise, gradient and profile loads; profile storage and CSV parser |
| `lib/sensor` | Reported speed in `0x1DA`: delay (`rptdelay`) and white or band-limited noise |
| `lib/simengine` | The 10 ms tick itself (commands, loads, model, frames), shared by the firmware and the PC harness |
| `lib/simcore` | `0x1D4` latch, tick statistics, parameter table, presets, flat JSON, ring buffer |
| `lib/simproto` | Console parsing, command parser, `0x4B0`–`0x4B2` packing, CSV stream lines, Zombie SDO |
| `src/` | ESP32 code: TWAI driver, 10 ms tick, console, `canlog`, SD files, NVS, status LED |
| `test/` | Unity tests for `lib/`, run with `pio test -e native` |
| `tools/` | Python log tools, synthetic log generator and self-test ([tools/README.md](tools/README.md)) |
| `tools/dbc/leaf_sim.dbc` | DBC: Leaf frames, Zombie `0x4A0`–`0x4A3`, simulator `0x4B0`–`0x4B2` |
| `sd_card/` | Example SD card contents and format ([sd_card/README.md](sd_card/README.md)) |
| `docs/bench/` | Bench test procedure for each milestone; `results/` holds sweep tables |
| `.github/workflows/` | CI: native tests, tool self-test and firmware build on every push |

Everything in `lib/` is plain C++ with no Arduino or ESP-IDF includes (plan rule 3).

## Console

One command per line at 921600 baud. Every reply starts with `ok` or `err`. Type `help` for the list:

```text
stat | params | set <name> <value> | save
preset <name> | presets
load step <Nm> [ramp_ms] [dJ] | load pulse <Nm> <period_ms> <duty_pct>
load grad <Nm> | load noise <sd_Nm> [corner_hz] | load off
profile load <file> | profile start | profile stop | profile loop <0|1>
profile exp <n> <w_ref_rpm> | profile list
mark <n> | fault inv <0|1> | stream <0|1> [divider] | canlog <0|1> [all]
zget <name> | zset <name> <value>   (Zombie governor params over SDO)
```

`params` lists every model parameter with its unit and range. Besides the plant (`J`, `Tc`, `b`, `c`, `tau`, `Tmax`, `Pmax`, `kT`), there are `udc`, the temperatures, and the reported speed's `noise` (rpm), `noise_fc` and `noise_bw` (Hz; `noise_fc` 0 is white noise) and `rptdelay` (ms). The tractor's speed ripple is `set noise 11`, `set noise_fc 6`, `set noise_bw 4`. A `preset` or `mark` restarts the noise, so a script repeats exactly.

## Status

| Milestone | State |
| --- | --- |
| M1 Skeleton, `leafcodec`, native tests | Done |
| M2 CAN bring-up | Bench tested 3 Nov 2026: [docs/bench/M2_can_bringup.md](docs/bench/M2_can_bringup.md) |
| M3 Plant with base friction, closed loop | Bench tested 3 Nov 2026: [docs/bench/M3_closed_loop.md](docs/bench/M3_closed_loop.md) |
| M4 Zombie governor logging | Bench tested 3 Nov 2026; now on `Tractor-RPM-limiting-testing` of Stm32-vcu: [docs/bench/M4_gov_logging.md](docs/bench/M4_gov_logging.md) |
| M5 Loads, presets, console, telemetry, events | Bench tested 3 Nov 2026: [docs/bench/M5_loads_console.md](docs/bench/M5_loads_console.md) |
| M5a Gains over SDO, scripted sweeps | Bench tested 3 Nov 2026: [docs/bench/M5a_sdo_sweeps.md](docs/bench/M5a_sdo_sweeps.md), results in [docs/bench/results/](docs/bench/results/) |
| M5b Review improvements (plan section 15) | Built 10 Nov 2026, awaiting bench test: [docs/bench/M5b_review_improvements.md](docs/bench/M5b_review_improvements.md) |
| M5c Loop calculator and software-in-the-loop (plan section 16) | Built 11 Nov 2026; reproduces the 3 Nov bench runs: [docs/bench/M5c_sil.md](docs/bench/M5c_sil.md) |
| M5d Gain search | Planned (plan section 16) |
| M6 SD profiles and PC tools | Bench tested 3 Nov 2026: [docs/bench/M6_sd_profiles_tools.md](docs/bench/M6_sd_profiles_tools.md) |
| M7 Tractor capture, runs R1 to R7 | Ben |
| M8, M9 Validation and tuning | Need M7 logs |
