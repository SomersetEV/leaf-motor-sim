# CLAUDE.md

Context for Claude Code. Read this first, then [docs/SIMULATOR_PLAN.md](docs/SIMULATOR_PLAN.md), which is the authoritative plan.

## What this is

A bench rig for Ben's Leyland 255 electric tractor. A LilyGo T-CAN485 (ESP32) impersonates a Nissan Leaf inverter and motor on CAN, against a real ZombieVerter VCU, so the Zombie's RPM governor can be tuned without the tractor. There are two codebases:

| Part | Where |
| --- | --- |
| Simulator firmware, PC tools, docs | This repo, `SomersetEV/leaf-motor-sim`, branch `main` |
| Zombie logging changes (plan Part B) | `SomersetEV/Stm32-vcu`, branch `gov-logging`, based on `Tractor-RPM-limiting-testing` @ `d85e756`. Clone with `--recurse-submodules`. |

## Working rules (plan section 2, plus Ben's preferences)

- Build in milestone order. Stop after each milestone so Ben can test it on hardware; nothing here can be verified on the board from the editor.
- Every milestone ships a bench procedure in `docs/bench/` covering what to wire, what to type and what Ben should see.
- `lib/` is plain C++ with no Arduino or ESP-IDF includes, so it builds and unit-tests on the PC. Hardware access lives only in `src/`.
- Nothing in the 10 ms tick may block: no serial, no SD, no heap. The console posts `SimCmd`s on a queue, and the tick applies them at the start of its next tick.
- Interfaces use rpm, Nm, kg·m², V and ms. The model uses rad/s internally.
- Items marked VERIFY in the plan get a named constant and a comment pointing to plan section 12.
- Zombie Part B changes are logging only, bit for bit. Ask before touching control logic, existing defaults, or anything out of phase 1 scope.
- Commit or push only when Ben asks. Ben is happy with a commit per milestone.

## Status (6 Oct 2026)

| Milestone | State |
| --- | --- |
| M1 codec and native tests | Done |
| M2 CAN bring-up | Built, not bench tested |
| M3 plant, closed loop | Built, not bench tested |
| M4 Zombie `gov-logging` | Built, pushed, not bench tested. Host tests pass, firmware builds. |
| M5 loads, presets, console, telemetry | Built, unit tested, not bench tested |
| M6 SD profiles and PC tools | Tools done and self-tested; SD replay not bench tested |
| M7 tractor captures R1–R7 | Ben's job |
| M8, M9 validation and tuning | Need M7 logs. Analysis uses `tools/`. |

**Next:** Ben runs the bench procedures in order: `docs/bench/M2_*`, `M3_*`, `M4_*`, `M5_*`, `M6_*`. Fix whatever they turn up before building anything further. When tractor logs arrive, follow plan section 10: `extract_params.py`, then `make_profile.py`, then `compare_runs.py`.

## Build and test

```text
pio test -e native             # 60 Unity tests for lib/
pio run -e tcan485             # firmware (pioarduino "stable": Arduino-ESP32 3.3.0, IDF 5.5)
python tools/selftest.py       # 21 checks; needs g++ on PATH and tools/requirements.txt
```

Zombie, on the `gov-logging` branch: `cd test && make && ./test_vcu` for host tests (18 pass, including `TestGovernorGolden`). Run `make` at the root for the firmware, using the GNU Arm toolchain. On Windows, build libopencm3 once with `make -C libopencm3 TARGETS=stm32/f1 PYTHON="py -3"`.

### Environment pitfalls (found on Ben's PC; the laptop may differ)

- Run `pio` from PowerShell or the VS Code PlatformIO terminal. pioarduino's `idf_tools.py` refuses to run under Git Bash (MSYS).
- A global `IDF_PATH` (for example from an ESP-IDF install) can make pioarduino's tool install fail. The symptom is `idf_tools.py installation failed`, then missing `xtensa-esp32-elf-g++`. Clear `IDF_PATH` in that shell before running `pio`. This project does not use a separate ESP-IDF install.
- If `pio` is not on `PATH`, it is at `~/.platformio/penv/Scripts/pio.exe`.
- On Windows, `python` may be only the Store stub; use `py -3`.
- Unity's `TEST_ASSERT_*` macros evaluate their arguments more than once. Never pass calls with side effects, such as `tick()`, directly into them.

## Decisions and facts already settled

- **CAN pins:** TX 27, RX 26, from Ben's working telematics firmware. Plan item V4 is settled by Ben; the plan and LilyGO's example have them swapped.
- **Zombie facts checked at `d85e756`:**
  - `nissan_crc`: polynomial 0x85, byte 7 treated as 0.
  - `0x1D4` torque: 12-bit two's complement in b2 and the upper nibble of b3. Out-of-range requests are sent as 0. HV status in the b4 low nibble, counter in the b4 top two bits.
  - `0x1DA` as decoded by the Zombie: reverse speeds read 1 rpm high (not compensated, on purpose); `udc` used only below 420 V with `ShuntType` 0; error = `(b6 & 0xB0) != 0`.
  - Governor: the speed filter is shift 4, about 155 ms, not the 30 ms the old comment claimed (plan V5). The comment is corrected in `gov-logging`, and `govFlt` makes the shift a parameter.
  - Parameter IDs: `govFlt` 162, `GovLog` 163, spot values 2126–2129. Next free IDs are 164 and 2130.
- **Tests compare against the Zombie's own code.** `test/test_leafcodec` contains verbatim copies of the Zombie encode and decode functions. On the Zombie side, the governor golden test was recorded from the unmodified code, and the first commit on `gov-logging` contains only that test.
- **PC tools must mirror `lib/plant` exactly.** `tools/simmodel.py` reproduces the firmware's 1 ms lag sub-steps; the continuous-time lag biased b and c. `loglib` interpolates speeds onto the 10 ms grid, because holding the last value creates derivative spikes. `extract_params` picks the request-to-speed delay from −2 to +2 ticks automatically.
- **Profiles** are stored in 8 kB chunks, capped at 60,000 rows, because the ESP32 often lacks a contiguous 120 kB block. Inertia is stored as change points.
- **Preset numbers are placeholders.** `noload`, `topper`, `baler` and `pto_step` were invented by Claude until M7 data exists. Files at `/presets/<name>.json` on SD override them.

## Open items (plan section 12)

| Item | State |
| --- | --- |
| V1 `kT` = 0.25 Nm per bit | Open: needs a tractor log |
| V2 motor 280 Nm, 80 kW | Open: Ben to confirm the motor and inverter generation |
| V3 reported torque layout in `0x1DA` | Open: behind `SIM_REPORT_TORQUE`, default off |
| V4 CAN pins | Settled |
| V5 governor filter shift | Ben decides after bench runs at 4, 3 and 2 |
| V6 raw 10 ms CAN logging on the tractor | Open |
| V7 run-mode inputs for the vehicle class | Open: needs the tractor's `Vehicle` parameter, then read its `Ready()`/`Start()` in Stm32-vcu |

## Ben's safety note for tractor work

Turn `GovLog` on with the tractor stationary first, and confirm the MG charger's DC-DC output stays up. DC-DC dropouts on this machine have coincided with extra CAN traffic before.
