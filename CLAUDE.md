# CLAUDE.md

Context for Claude Code. Read this first, then [docs/SIMULATOR_PLAN.md](docs/SIMULATOR_PLAN.md), which is the authoritative plan.

## What this is

A bench rig for Ben's Leyland 255 electric tractor. A LilyGo T-CAN485 (ESP32) impersonates a Nissan Leaf inverter and motor on CAN, against a real ZombieVerter VCU, so the Zombie's RPM governor can be tuned without the tractor. There are two codebases:

| Part | Where |
| --- | --- |
| Simulator firmware, PC tools, docs | This repo, `SomersetEV/leaf-motor-sim`, branch `main` |
| Zombie logging changes (plan Part B) | `SomersetEV/Stm32-vcu`. Built on branch `gov-logging` (`4b57091`, based on `Tractor-RPM-limiting-testing` @ `d85e756`), and since carried into `Tractor-RPM-limiting-testing` itself (`7ea0fa6`, clang-format `a05e984`, 8 Nov). Clone with `--recurse-submodules`. |

## Working rules (plan section 2, plus Ben's preferences)

- Build in milestone order. Stop after each milestone so Ben can test it on hardware; nothing here can be verified on the board from the editor.
- Every milestone ships a bench procedure in `docs/bench/` covering what to wire, what to type and what Ben should see.
- `lib/` is plain C++ with no Arduino or ESP-IDF includes, so it builds and unit-tests on the PC. Hardware access lives only in `src/`.
- Nothing in the 10 ms tick may block: no serial, no SD, no heap. The console posts `SimCmd`s on a queue, and the tick applies them at the start of its next tick.
- Interfaces use rpm, Nm, kg·m², V and ms. The model uses rad/s internally.
- Items marked VERIFY in the plan get a named constant and a comment pointing to plan section 12.
- Zombie Part B changes are logging only, bit for bit. Ask before touching control logic, existing defaults, or anything out of phase 1 scope.
- Commit or push only when Ben asks. Ben is happy with a commit per milestone.

## Status (11 Nov 2026)

| Milestone | State |
| --- | --- |
| M1 codec and native tests | Done |
| M2 CAN bring-up | Bench tested (3 Nov 2026) |
| M3 plant, closed loop | Bench tested (3 Nov 2026). Bench setup needs `BMS_Mode` 0; see `docs/bench/M3_*`. |
| M4 Zombie `gov-logging` | Bench tested (3 Nov 2026), second capture. The first capture failed on the ID 162 clash below. |
| M5 loads, presets, console, telemetry | Bench tested 3 Nov 2026, all steps pass (step 7, the inverter error, confirmed by Ben). Claude drove the console over COM3 with `tools/scripts/m5_part*.yaml`. Tick jitter (about 0.68 ms late wakes) was fixed in `55de8ac`; the worst is now 7–8 µs. Bus errors came from the powered Due, and the sim's `canlog` replaces SavvyCAN. |
| M5a SDO gain control and sweeps | Built 3 Nov 2026: `zset`/`zget` bench-tested against the Zombie, and `tools/sweep.py` runs end to end. Four sweeps run with the pedal clamped (Kp, Ki, `govFlt`, Kd; results in `docs/bench/M5a_*`, tables in `docs/bench/results/`). On the placeholder baler, the best so far is Kp 11, Ki 1, `govFlt` 2, Kd 0–1; Kd 2 and above is unstable at no-load. Ben can't find the baler logs, so sweeps use the placeholder `baler` preset for now. The telematics SD card had only one short raw log (`session_0052.csv`, 100 kB). |
| M5b review improvements (plan section 15) | Built 10 Nov 2026, awaiting bench: `docs/bench/M5b_review_improvements.md`. Per-item state is in plan section 15, "Progress". I9 was done by Ben (logger fixed); I10 (`sd ls` reboot) needs the board. Committed (`2dc92c4`). |
| M5c loop calculator and software-in-the-loop (plan section 16) | Built 11 Nov 2026, all three acceptance tests pass on the 3 Nov bench logs: `docs/bench/M5c_sil.md`. The bench part waits for the M5b sweeps (ring periods on their tails) and Ben's `regenramp` (V8). |
| M6 SD profiles and PC tools | SD replay bench tested (3 Nov 2026): a profile built from a bench log replays to 0.09 Nm, and the closed loop reproduces its source run to 4.8 rpm. Done. |
| M7 tractor captures R1–R7 | Ben's job |
| M8, M9 validation and tuning | Need M7 logs. Analysis uses `tools/`. |

**Next:** the M5b bench procedure, `docs/bench/M5b_review_improvements.md`. In short: flash, check `params` and the repeatability of one set (`repeats: 3`, I5), then re-run `tools/scripts/sweep_road.yaml` (road theory test, no baler: full pedal, J 2.5, slow gradient ramps from +60 to −15 Nm). Since 10 Nov it uses one band-limited noise setting fitted to the 7 Nov tractor logs (`noise` 11, `noise_fc` 6, `noise_bw` 4, from `tools/fit_noise.py`) in place of the per-`govFlt` white-noise levels. It adds sets at `govKp` 20–30 on shift 2 (plan section 16) and a 4 s tail with the load off, for ring metrics. Earlier road results are not valid: the first run lost log chunks to a pump bug, and the second run's Ki 0 sets kept an 8 % integral. `sweep.py` now clears the integral at each set by pulsing `govImax` to 0; check that `peak_integral_pct` is 0 on the Ki 0 sets. Then I10: apply the git stash (`sd ls`/`sd get`, `tools/sd_pull.py`), capture the backtrace from the `sd ls` reboot and fix it, and take `topper.json` (test only) off the sim's SD card. Then push for the first CI run (I3); CI now also checks out `SomersetEV/Stm32-vcu` for the harness checks. Then run `sil_check.py` on the new M5b result folders (`docs/bench/M5c_sil.md`, bench steps). Then M5d, the gain search over the harness. When Ben's new tractor logs arrive (logger fixed 10 Nov), check them for `0x4A1` gaps and follow plan section 10: `extract_params.py`, then `make_profile.py`, then `compare_runs.py`.

**Added 10 Nov 2026 (plan sections 15–17):** a repo review gave milestones M5b (eleven improvements, built 10 Nov), M5c (loop calculator and software-in-the-loop harness) and M5d (gain search). None needs tractor data. Section 17 is Phase 3, governor auto-tune (M14–M17): do not start without Ben. The Zombie facts below were re-checked on 10 Nov (item I1).

**Phase 2 (planned, do not start yet):** speed-mode throttle, plan sections 13 and 14, milestones M10–M13. Starts only after M9 is signed off and the sim is working on the bench. It changes Zombie control logic on a new `speed-mode` branch, so the logging-only rule above does not apply to it, but confirm scope with Ben before starting. Next free Zombie IDs are 165 (parameters) and 2130 (values) unless Phase 1 uses them.

## Build and test

```text
pio test -e native             # 79 Unity tests for lib/
pio run -e tcan485             # firmware (pioarduino "stable": Arduino-ESP32 3.3.0, IDF 5.5)
python tools/selftest.py       # 48 checks; needs g++ on PATH and tools/requirements.txt.
                               # The 5 harness checks need the Zombie tree (ZOMBIE_DIR or ../Stm32-vcu), else skipped
```

GitHub Actions (`.github/workflows/ci.yml`) runs all three on every push, with the Zombie tree checked out for the harness checks.

Software-in-the-loop (M5c): `tools/silport.py --golden`, then `sweep.py ... --port sil` or `run_script.py ... --port sil`. `tools/silport.py` builds the harness into `tools/sil/build/` (gitignored) when a source changes. `loopcalc.py` gives margins and the ring from a linear model.

Zombie, on `Tractor-RPM-limiting-testing` (or `gov-logging`): `cd test && make && ./test_vcu` for host tests (18 pass, including `TestGovernorGolden`; re-checked on the tractor branch 10 Nov). On Windows, `mingw32-make` from WinLibs works where `make` is missing. Run `make` at the root for the firmware, using the GNU Arm toolchain. On Windows, build libopencm3 once with `make -C libopencm3 TARGETS=stm32/f1 PYTHON="py -3"`.

### Environment pitfalls (found on Ben's PC; the laptop may differ)

- Run `pio` from PowerShell or the VS Code PlatformIO terminal. pioarduino's `idf_tools.py` refuses to run under Git Bash (MSYS).
- A global `IDF_PATH` (for example from an ESP-IDF install) can make pioarduino's tool install fail. The symptom is `idf_tools.py installation failed`, then missing `xtensa-esp32-elf-g++`. Clear `IDF_PATH` in that shell before running `pio`. This project does not use a separate ESP-IDF install.
- If `pio` is not on `PATH`, it is at `~/.platformio/penv/Scripts/pio.exe`.
- On Windows, `python` may be only the Store stub; use `py -3`.
- Unity's `TEST_ASSERT_*` macros evaluate their arguments more than once. Never pass calls with side effects, such as `tick()`, directly into them.

## Decisions and facts already settled

- **CAN pins:** TX 27, RX 26, from Ben's working telematics firmware. Plan item V4 is settled by Ben; the plan and LilyGO's example have them swapped.
- **Zombie facts checked at `d85e756`, re-checked 10 Nov 2026 at `a05e984` (`Tractor-RPM-limiting-testing`); none had changed except the ID clash:**
  - `nissan_crc`: polynomial 0x85, byte 7 treated as 0.
  - `0x1D4` torque: 12-bit two's complement in b2 and the upper nibble of b3. Out-of-range requests are sent as 0. HV status in the b4 low nibble, counter in the b4 top two bits.
  - `0x1DA` as decoded by the Zombie: reverse speeds read 1 rpm high (not compensated, on purpose); `udc` used only below 420 V with `ShuntType` 0; error = `(b6 & 0xB0) != 0`.
  - Governor: the speed filter is shift 4, about 155 ms, not the 30 ms the old comment claimed (plan V5). The comment is corrected in `gov-logging`, and `govFlt` makes the shift a parameter.
  - Parameter IDs: `govFlt` 162, `GovLog` 163, `MGChgLimRaw` 164, spot values 2126–2129. Next free IDs are 165 and 2130.
  - **ID 162 clash, fixed in the source on 8 Nov (`28d53d0` moved `MGChgLimRaw` to 164).** `c1ad315` (19 Oct) had `MGChgLimRaw` (default 2046) on ID 162, the same as `govFlt`. Flash load skips range checks, so a Zombie whose saved setup came from `c1ad315`-era firmware still loads `govFlt` = 2046 when it first boots `gov-logging` or `a05e984`. The filter then goes NaN and the governor commands `regenmax` regardless of pedal until reboot. After any such switch, set `govFlt` = 4 (and `MGChgLimRaw` = 2046), save and power-cycle.
- **Tractor governor setup (Ben, 3 Nov 2026):** `revlim` 2200, `govKp` 11, `govKi` 0, `govKd` 0, `govImax` 100, `regenmax` −30, `throtramp` 10. It is P-only today.
- **Tests compare against the Zombie's own code.** `test/test_leafcodec` contains verbatim copies of the Zombie encode and decode functions. On the Zombie side, the governor golden test was recorded from the unmodified code, and the first commit on `gov-logging` contains only that test.
- **The harness runs the firmware's own tick.** `lib/simengine` is the sim tick, shared by `src/sim_task.cpp` and `tools/sil`; change the tick there, never in one copy. The Zombie glue in `tools/sil/zombie_side.cpp` is line-for-line copies of `Ms10Task`, `ProcessThrottle`, `SendGovLog`, `SetTorque`, `DecodeCAN` and the 0x1D4 part of `NissLeafMng::Task10Ms`. Re-check them when the Zombie branch moves.
- **Bench loop delay (measured 11 Nov 2026 from the 3 Nov logs):** three sim ticks from a 0x1DA speed sample to the tick applying its torque, because each 0x1D4 carries the previous tick's request (`Task10Ms` runs before `SetTorque`). It is four ticks while the drifting clock phase (about 1.7 ms a minute) puts the 0x1D4 just after the sim's tick. Sweep rows record it as `loop_ticks`; compare runs at equal values, and in the harness use `rptdelay 10` for four.
- **Governor relay at `revlim` (M5c finding 3):** with `govKi` above 0, `MIN(0, P + I + D)` over the limit cuts torque to zero, so the loop hunts at `revlim` (no-load p2p 37–90 rpm on the bench). `loopcalc.py` describes P and PD only there. Candidate change G5 is for the harness only, with Ben's sign-off.
- **PC tools must mirror `lib/plant` exactly.** `tools/simmodel.py` reproduces the firmware's 1 ms lag sub-steps; the continuous-time lag biased b and c. `loglib` interpolates speeds onto the 10 ms grid, because holding the last value creates derivative spikes. `extract_params` picks the request-to-speed delay from −2 to +2 ticks automatically.
- **Profiles** are stored in 8 kB chunks, capped at 60,000 rows, because the ESP32 often lacks a contiguous 120 kB block. Inertia is stored as change points.
- **Bench capture is the sim's `canlog`, not SavvyCAN** (3 Nov 2026). Ben's Due with GVRET sends about 120 error frames a second even in listen-only mode, and it dropped Zombie frames from its captures; keep it unpowered. To capture, run `tools/run_script.py ... --canlog-out bus.log` with `canlog 1` in the script. On the laptop, the sim is on COM3 (CH9102) and the Due on COM5. Claude can flash and drive the sim from here (`pio run -e tcan485 -t upload --upload-port COM3`; a first connect attempt sometimes fails, so retry).
- **Tick timing:** the tick has core 1 to itself. The UART and TWAI drivers are installed from a core-0 init task, and `can_rx_task` runs on core 0. With the TWAI ISR or `can_rx_task` on core 1, the errata reset on each bus error held off the tick by about 0.6 ms (M5 bench).
- **Laptop tooling:** `.venv` (Python 3.11 from `~/.platformio/python3`) has `tools/requirements.txt` installed. `g++` is WinLibs GCC 16.2 (`winget` `BrechtSanders.WinLibs.POSIX.UCRT`, per user, 3 Nov 2026). If a shell doesn't find it, put `%LOCALAPPDATA%\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64in` first on `PATH`. Native tests and `selftest.py` both pass with it.
- **Preset numbers are placeholders.** `noload`, `topper`, `baler` and `pto_step` were invented by Claude until M7 data exists. Files at `/presets/<name>.json` on SD override them.

## Open items (plan section 12)

| Item | State |
| --- | --- |
| V1 `kT` = 0.25 Nm per bit | Settled 8 Nov 2026 from telematics logs `session_0011`/`0012` (7 Nov): reported torque is 0.249–0.250 Nm per request bit, and shunt DC power gives 84–86 % motoring efficiency |
| V2 motor 280 Nm, 80 kW | Open: Ben to confirm the motor and inverter generation |
| V3 reported torque layout in `0x1DA` | Settled 8 Nov 2026, same logs: tracks the request with r = 0.998 at 20–30 ms lag. The sim now reports its motor torque there (`SIM_REPORT_TORQUE`, default on) |
| V4 CAN pins | Settled: TX 27, RX 26 (plan section 4 updated 10 Nov) |
| V5 governor filter shift | Ben decides after bench runs at 4, 3 and 2. M4, `noload`, P-only (Kp 11): shift 4 rings at about 630 ms, and each cycle keeps about 0.65 of the last one's size; shift 2 rings at about 370 ms and keeps about 0.35. M5a sweeps (placeholder baler, Kp 11, Ki 1), no-load ring / stroke ripple: shift 4 gives 90 / 322 rpm, 3 gives 60 / 293, and 2 gives 37 / 238, but torque swing per stroke rises from 124 to 161 Nm. Speed noise (`set noise` up to 15 rpm, Gaussian) does not change that: at shift 2 it adds about 2 Nm RMS of tick-to-tick torque jitter at no-load, and true-shaft ripple stays 242–268 rpm against 322 at shift 4. Shift 2 was on the tractor briefly in July (`c54f5bd7`, reverted in `d9edbf39`), but other problems were going on, so Ben says to treat that trial as no information. |
| V6 raw 10 ms CAN logging on the tractor | Ben fixed the telematics logger on 10 Nov (it kept about 15 % of frames); new logs to come |
| V7 run-mode inputs for the vehicle class | Open: needs the tractor's `Vehicle` parameter, then read its `Ready()`/`Start()` in Stm32-vcu |
| V8 tractor `regenramp` | Open: not logged. The bench Zombie's is at least 13.6 %/10 ms (M4 log), not the default 1; the harness uses 100. Ben to give the value |

## Ben's safety note for tractor work

Turn `GovLog` on with the tractor stationary first, and confirm the MG charger's DC-DC output stays up. DC-DC dropouts on this machine have coincided with extra CAN traffic before.
