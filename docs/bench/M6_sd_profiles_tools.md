# M6 test: SD profiles and PC tools

**Goal (plan section 11, M6):** the tool self-test recovers known parameters within 5 %. A generated profile replays from SD and matches its source torque trace in `0x4B0`.

## 1. PC tools (no hardware, Ben's PC)

```text
cd tools
python -m pip install -r requirements.txt
python selftest.py
```

It should finish with `21/21 checks passed`. It needs `g++` on `PATH`. WinLibs or MSYS2 MinGW both work on Windows. The laptop has neither: its LLVM `clang++` finds MSVC but not the Windows SDK headers. The run here gave the following:
- `J`, `Tc`, `b` and `c` recovered to within 0.11 %;
- a synthetic baler load recovered with an RMS error of 1.8 Nm on a 150 Nm peak;
- the profile replayed through the firmware's profile code to within 0.05 Nm (storage is 0.1 Nm).

## 2. Test profile from a bench log (done, 3 Nov 2026)

`sd_card/profiles/baler_bench.csv` was made by `make_profile.py` from the M5a Kd sweep's first run (`sweep_kd_baler/run_01_bus.log`: placeholder `baler` preset, Kp 11, Ki 1, `govFlt` 2), using the sim's own baler parameters (J 0.9, Tc 10, b 0.02), delay 0 ticks, cutoff 20 Hz:

```text
python make_profile.py run_01_bus.log scripts\sim_baler_params.json -o ..\sd_card\profiles\baler_bench.csv --start 8.08 --cutoff 20 --delay-ticks 0
```

Against the load the sim itself reported in `0x4B0`, less the base friction, it is within 3.8 Nm RMS on a 150 Nm peak, with no bias (0.1 Nm). Delay 0 fits best. A delay of +1 tick gives 10 Nm RMS. It is 2,999 rows (30 s), with a mean of 38 Nm and a peak of 147 Nm.

## 3. SD card

1. Format a microSD card FAT32. Copy `sd_card/presets` and `sd_card/profiles` to its root.
2. Also copy the **contents** of `sd_card/m6_test/` to the root, so `topper.json` joins `/presets` and `bad_spacing.csv` joins `/profiles`. These are for this test only. Delete them from the card afterwards, as `topper.json` changes the topper preset.
3. Insert the card and reset the LilyGo. The banner shows `SD card found`, and `stat` ends `sd=1`.

## 4. Replay on the bench

Zombie in Run, pedal held at any fixed position so the shaft turns, Due unpowered. Claude runs this over COM3:

```text
cd tools
..\.venv\Scripts\python run_script.py scripts\m6_replay.yaml --port COM3 --log m6.txt --canlog-out m6_bus.log
```

It takes about 80 s. Ben watches the LED.

| Time | Command | Expected |
| --- | --- | --- |
| 0–1 s | `stat`, `presets`, `profile list` | `sd=1`; the card's presets and profiles are listed |
| 1.5 s | `profile load baler_bench` | `ok rows=2999 seconds=29.99 inertia=0`, plus the load time |
| 6 s | `profile start` | LED turns yellow. `0x4B2` `ProfileStart` event; `SIM_Slow_4B1.ProfileState` = Running |
| 11 s | `profile load baler_bench` again | `err stop the profile first` |
| 36 s | (the profile ends) | `ProfileStop` event, `ProfileState` = Finished, and `stat` shows `profile=finished` |
| 39–72 s | `profile loop 1`, `profile start` | Runs past its 30 s end without stopping |
| 75–76 s | `profile load nosuch` | `err file not found` |
|  | `profile load bad_spacing` | `err rows must be 10 ms apart from 0 at line 4` |
|  | `preset nosuch` | `err unknown preset, try presets` |
| 76.5 s | `preset topper`, `stat` | `J=0.750 noise=on`: the card file overrides J and keeps the built-in noise |

**Match check (Claude, from `m6_bus.log`):** `SIM_Fast_4B0.TorqueLoad`, less the base friction (`Tc` + `b`|ω|, from the `params` line), follows `baler_bench.csv` row for row, at most one tick late.

Send back: the LED colour during the replay. Claude reads `m6.txt` and `m6_bus.log` directly.

**Closed-loop check:** `scripts/m6_reproduce.yaml` replays the profile with the model and Zombie gains of the run it came from: J 0.9, Tc 10, Kp 11, Ki 1, `govFlt` 2. The speed trace should then match that run's. Afterwards it restores `govKi` 0, `govFlt` 4 and `GovLog` 0. That matches the tractor setup, not necessarily what the Zombie had before.

A profile holds only the load above the base friction of the parameters it was made with, and no inertia unless it was made with `--with-inertia`. Replay it with the same J, Tc, b and c, or the speed swings differently. With `noload`'s J 0.3 and Tc 5 instead, the same profile swung the speed between 981 and 3116 rpm on the tractor gains.

## Results (3 Nov 2026, Claude over COM3)

Logs are in `D:\Downloads\leaf sim testing logs\m6`.

- **Section 4: every reply as expected.**
  - The card mounted (`sd=1`), and both directories were listed.
  - `profile load` read 2,999 rows in 525 ms.
  - `ProfileStart` and `ProfileStop` events came 30.0 s apart, and the state went Running, then Finished.
  - Loop ran 33 s, and `row` wrapped to 251.
  - All four error replies came back word for word.
  - `preset topper` from the card gave `J=0.750 noise=on`.
- **Match check: pass.** `TorqueLoad` minus `Tc` + `b`|ω| follows the file to within 0.09 Nm RMS (0.21 Nm at worst) on both passes, with no lag. A lag of ±1 tick gives 7 Nm.
- **Closed-loop check: pass.** The speed trace matches the source run to within 4.8 rpm RMS (correlation 0.999).

  | Metric | Source run | Replay |
  | --- | --- | --- |
  | Lowest dip (rpm) | 2040 | 2040 |
  | Highest peak (rpm) | 2290 | 2280 |
  | Ripple per stroke (rpm) | 237 | 235 |
  | Torque swing per stroke (Nm) | 161 | 164 |

- **LED:** yellow while the profile ran (Ben).
- **Still to do:** section 1 (`selftest.py`, Ben's PC).
