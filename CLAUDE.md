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
| M2 CAN bring-up | Bench tested (3 Nov 2026) |
| M3 plant, closed loop | Bench tested (3 Nov 2026). Bench setup needs `BMS_Mode` 0; see `docs/bench/M3_*`. |
| M4 Zombie `gov-logging` | Bench tested (3 Nov 2026), second capture. The first capture failed on the ID 162 clash below. |
| M5 loads, presets, console, telemetry | Bench tested 3 Nov 2026, all steps pass (step 7, the inverter error, confirmed by Ben). Claude drove the console over COM3 with `tools/scripts/m5_part*.yaml`. Tick jitter (about 0.68 ms late wakes) was fixed in `55de8ac`; the worst is now 7–8 µs. Bus errors came from the powered Due, and the sim's `canlog` replaces SavvyCAN. |
| M5a SDO gain control and sweeps | Built 3 Nov 2026: `zset`/`zget` bench-tested against the Zombie, and `tools/sweep.py` runs end to end. Four sweeps run with the pedal clamped (Kp, Ki, `govFlt`, Kd; results in `docs/bench/M5a_*`). On the placeholder baler, the best so far is Kp 11, Ki 1, `govFlt` 2, Kd 0–1; Kd 2 and above is unstable at no-load. Ben can't find the baler logs, so sweeps use the placeholder `baler` preset for now. The telematics SD card had only one short raw log (`session_0052.csv`, 100 kB). |
| M6 SD profiles and PC tools | SD replay bench tested (3 Nov 2026): a profile built from a bench log replays to 0.09 Nm, and the closed loop reproduces its source run to 4.8 rpm. `selftest.py` 21/21 and native tests 64/64 on the laptop. Done. |
| M7 tractor captures R1–R7 | Ben's job |
| M8, M9 validation and tuning | Need M7 logs. Analysis uses `tools/`. |

**Next:** re-run `tools/scripts/sweep_road.yaml` (road theory test, no baler: full pedal, J 2.5, slow gradient ramps from +60 to −15 Nm). Results so far are not valid. The first run lost log chunks to a `sweep.py` pump bug, now fixed. The second run's Ki 0 sets kept an 8 % integral from the previous sweep, because with Ki 0 the Zombie freezes the integral rather than clearing it. `sweep.py` now clears it at each set by pulsing `govImax` to 0. That change is not yet bench-run; check that `peak_integral_pct` is 0 on the Ki 0 sets. Also, `topper.json` (test only) to come off the sim's SD card. Then M7 tractor captures. `set noise <rpm>` adds noise to the reported speed only; `sweep.py` sets it per run with `sim:`. SD transfer over serial (`sd ls`/`sd get`, `tools/sd_pull.py`) is parked in a git stash, because `sd ls` rebooted the sim. Fix whatever they turn up before building anything further. When tractor logs arrive, follow plan section 10: `extract_params.py`, then `make_profile.py`, then `compare_runs.py`.

**Phase 2 (planned, do not start yet):** speed-mode throttle, plan sections 13 and 14, milestones M10–M13. Starts only after M9 is signed off and the sim is working on the bench. It changes Zombie control logic on a new `speed-mode` branch, so the logging-only rule above does not apply to it, but confirm scope with Ben before starting. Next free Zombie parameter IDs are still 164 and 2130 unless Phase 1 uses them.

## Build and test

```text
pio test -e native             # 64 Unity tests for lib/
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
  - **Known clash, left as is by Ben (3 Nov 2026):** `Tractor-RPM-limiting-testing` `c1ad315` (19 Oct) also uses ID 162, for `MGChgLimRaw` (default 2046). Flash load skips range checks, so going from that firmware to `gov-logging` loads `govFlt` = 2046. The filter then goes NaN and the governor commands `regenmax` regardless of pedal until reboot. After any switch between the two, set `govFlt` = 4 (or `MGChgLimRaw` = 2046), save and power-cycle. `gov-logging` is not to be changed for this yet.
- **Tractor governor setup (Ben, 3 Nov 2026):** `revlim` 2200, `govKp` 11, `govKi` 0, `govKd` 0, `govImax` 100, `regenmax` −30, `throtramp` 10. It is P-only today.
- **Tests compare against the Zombie's own code.** `test/test_leafcodec` contains verbatim copies of the Zombie encode and decode functions. On the Zombie side, the governor golden test was recorded from the unmodified code, and the first commit on `gov-logging` contains only that test.
- **PC tools must mirror `lib/plant` exactly.** `tools/simmodel.py` reproduces the firmware's 1 ms lag sub-steps; the continuous-time lag biased b and c. `loglib` interpolates speeds onto the 10 ms grid, because holding the last value creates derivative spikes. `extract_params` picks the request-to-speed delay from −2 to +2 ticks automatically.
- **Profiles** are stored in 8 kB chunks, capped at 60,000 rows, because the ESP32 often lacks a contiguous 120 kB block. Inertia is stored as change points.
- **Bench capture is the sim's `canlog`, not SavvyCAN** (3 Nov 2026). Ben's Due with GVRET sends about 120 error frames a second even in listen-only mode, and it dropped Zombie frames from its captures; keep it unpowered. To capture, run `tools/run_script.py ... --canlog-out bus.log` with `canlog 1` in the script. On the laptop, the sim is on COM3 (CH9102) and the Due on COM5. Claude can flash and drive the sim from here (`pio run -e tcan485 -t upload --upload-port COM3`; a first connect attempt sometimes fails, so retry).
- **Tick timing:** the tick has core 1 to itself. The UART and TWAI drivers are installed from a core-0 init task, and `can_rx_task` runs on core 0. With the TWAI ISR or `can_rx_task` on core 1, the errata reset on each bus error held off the tick by about 0.6 ms (M5 bench).
- **Laptop tooling:** `.venv` (Python 3.11 from `~/.platformio/python3`) has `tools/requirements.txt` installed. `g++` is WinLibs GCC 16.2 (`winget` `BrechtSanders.WinLibs.POSIX.UCRT`, per user, 3 Nov 2026). If a shell doesn't find it, put `%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64in` first on `PATH`. Native tests and `selftest.py` both pass with it.
- **Preset numbers are placeholders.** `noload`, `topper`, `baler` and `pto_step` were invented by Claude until M7 data exists. Files at `/presets/<name>.json` on SD override them.

## Open items (plan section 12)

| Item | State |
| --- | --- |
| V1 `kT` = 0.25 Nm per bit | Open: needs a tractor log |
| V2 motor 280 Nm, 80 kW | Open: Ben to confirm the motor and inverter generation |
| V3 reported torque layout in `0x1DA` | Open: behind `SIM_REPORT_TORQUE`, default off |
| V4 CAN pins | Settled |
| V5 governor filter shift | Ben decides after bench runs at 4, 3 and 2. M4, `noload`, P-only (Kp 11): shift 4 rings at about 630 ms, and each cycle keeps about 0.65 of the last one's size; shift 2 rings at about 370 ms and keeps about 0.35. M5a sweeps (placeholder baler, Kp 11, Ki 1), no-load ring / stroke ripple: shift 4 gives 90 / 322 rpm, 3 gives 60 / 293, and 2 gives 37 / 238, but torque swing per stroke rises from 124 to 161 Nm. Speed noise (`set noise` up to 15 rpm, Gaussian) does not change that: at shift 2 it adds about 2 Nm RMS of tick-to-tick torque jitter at no-load, and true-shaft ripple stays 242–268 rpm against 322 at shift 4. Shift 2 was on the tractor briefly in July (`c54f5bd7`, reverted in `d9edbf39`), but other problems were going on, so Ben says to treat that trial as no information. |
| V6 raw 10 ms CAN logging on the tractor | Open |
| V7 run-mode inputs for the vehicle class | Open: needs the tractor's `Vehicle` parameter, then read its `Ready()`/`Start()` in Stm32-vcu |

## Ben's safety note for tractor work

Turn `GovLog` on with the tractor stationary first, and confirm the MG charger's DC-DC output stays up. DC-DC dropouts on this machine have coincided with extra CAN traffic before.
