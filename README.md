# leaf-motor-sim

Nissan Leaf inverter and motor simulator for a LilyGo T-CAN485, used to test the ZombieVerter RPM governor on the bench. The full plan is in [docs/SIMULATOR_PLAN.md](docs/SIMULATOR_PLAN.md).

## Build and test

```text
pio test -e native          # unit tests on the PC (lib/)
pio run -e tcan485          # firmware
pio run -e tcan485 -t upload
pio device monitor -e tcan485
```

Run these from PowerShell or the PlatformIO terminal in VS Code. The pioarduino platform's tool installer refuses to run under Git Bash (MSYS). It also fails if `IDF_PATH` points to an ESP-IDF install that does not exist.

## Layout

| Path | Contents |
| --- | --- |
| `lib/leafcodec` | `0x1D4` decode, `0x1DA` and `0x55A` encode, Nissan CRC. Plain C++. |
| `lib/simcore` | `0x1D4` latch with the 100 ms receive timeout, tick timing statistics. Plain C++. |
| `lib/simproto` | Console line reader and tokeniser. Plain C++. |
| `lib/plant` | Single-inertia motor model: torque envelope, torque lag, friction, standstill hold. Plain C++. |
| `src/` | ESP32 code: TWAI driver, 10 ms simulation task, serial console. |
| `test/` | Unity tests for `lib/`, run with `pio test -e native`. |
| `tools/dbc/leaf_sim.dbc` | DBC for SavvyCAN. |
| `docs/bench/` | Bench test procedure for each milestone. |

## Status

| Milestone | State |
| --- | --- |
| M1 Skeleton, `leafcodec`, native tests | Done |
| M2 CAN bring-up | Built, awaiting bench test: [docs/bench/M2_can_bringup.md](docs/bench/M2_can_bringup.md) |
| M3 Plant with base friction, closed loop | Built, awaiting bench test: [docs/bench/M3_closed_loop.md](docs/bench/M3_closed_loop.md) |
