# M5c: loop calculator and software-in-the-loop harness

**Goal (plan section 16, M5c):** a PC model of the governor loop that is good enough to tune on. There are two parts:

- **Harness.** `tools/sil/` runs the Zombie's own governor code in closed loop with the simulator's own tick, in simulated time.
- **Calculator.** `tools/loopcalc.py` gives the loop's margins and ring from a linear model.

Acceptance has three parts:

- the Zombie's golden test passes through the harness;
- three bench runs reproduce, with ring period within 10 % and stroke ripple within 15 %;
- the calculator and the harness agree on ring period within 10 %.

All three pass, on the bench logs from 3 Nov.

Nothing here needs the board. The bench steps at the end are for after the M5b sweeps.

## What was built

| Part | What it does |
| --- | --- |
| `lib/simengine` | The simulator's 10 ms tick, moved out of `src/sim_task.cpp` into plain C++. The tick applies console commands, runs the loads and the model, and builds every frame. The firmware and the harness both run this one class, so the harness's plant is the firmware's, tick for tick. Four native tests. |
| `tools/sil/zombie_side.cpp` | The Zombie side of the harness. It runs the Zombie's own `throttle.cpp`, `govlog.cpp` and `params.cpp`, compiled from the Stm32-vcu tree. Around them, line-for-line copies of `Ms10Task`, `ProcessThrottle`, `SendGovLog`, `LeafINV::SetTorque` and `DecodeCAN`, and the 0x1D4 part of `NissLeafMng::Task10Ms`. Left out: the pedal map and the voltage, current and temperature derates. |
| `tools/sil/sil_main.cpp` | The harness itself. It speaks the simulator's console on stdin and stdout, adds `#run <ticks>`, `pedal <pct>` and `zparam <name> <value>`, and writes `canlog` lines in candump format. `--golden` runs the Zombie's governor golden test. |
| `tools/silport.py` | Builds the harness with g++, and only when a source has changed. It presents the harness as a console port with a simulated clock. The Zombie tree comes from `--zombie`, `ZOMBIE_DIR` or `../Stm32-vcu`. The first line of every run records the Zombie commit. |
| `sweep.py --port sil`, `run_script.py --port sil` | The same sweeps and scripts as on the bench, against the harness, in seconds. `--pedal` sets the Zombie's pedal (default 100 %). `--sd DIR` stands in for the SD card. |
| `tools/loopcalc.py` | The loop calculator, an exact linearisation of the same loop. It uses the plant's 1 ms sub-steps, the measured three-tick delay, and the Zombie's filters, P, I and D. It prints crossover, phase and gain margins, and the dominant pole as ring period and keep per cycle. Comma lists give a table. |
| `tools/sil_check.py` | Re-runs result folders from `docs/bench/results/` through the harness and judges stroke ripple and ring period. `--loopcalc` checks the calculator against the harness. |
| `tools/sil_replay.py` | Replays a bench log's pedal and Zombie settings through the harness from rest, and compares the rings. It is for no-load pedal tests like M4. |
| `loglib.loop_delay_ticks`, `loop_ticks` | The bench's loop delay, measured from the bus log. It is now a column in every sweep table. See finding 1. |

## Results

### Golden test

`python tools/silport.py --golden` prints `golden PASS: 500 ticks bit for bit`. The Zombie code is built at `-O2` against `a05e984`.

### Bench sweeps reproduced (`sil_check.py`)

All five 3 Nov sweeps were run from their folders in `docs/bench/results/`, 22 judged sets in all. The harness reproduces each set's stroke ripple to within 0.5 %. Most other columns agree to within 1 %: torque per stroke, error, peak integral and cost. The unstable sets (Kd 2 and 4, Ki 4, Kp 18) are included.

| Sweep, set | Stroke ripple, bench / harness (rpm) | Torque per stroke, bench / harness (Nm) |
| --- | --- | --- |
| `govFlt` 4, 3, 2 | 322.1 / 321.8, 293.0 / 293.0, 237.7 / 237.7 | 124.0 / 124.0, 145.5 / 145.5, 160.8 / 160.8 |
| `govKd` 2, 4 (unstable) | 234.5 / 234.2, 263.9 / 263.3 | 238.3 / 238.3, 400.5 / 400.3 |
| `govKp` 8, 18 | 232.4 / 231.3, 379.8 / 379.8 | 52.0 / 51.7, 202.8 / 202.8 |

Two kinds of set are shown but not judged.

- **Sets with speed noise.** Before M5b the bench did not restart its noise on `preset`, so its noise sequence cannot be reproduced. Even so, they agree to within 4 %.
- **No-load p2p on marginal sets.** This depends on what the previous set left, and the harness and bench part company there.

### Rings reproduced (`sil_replay.py`, M4 pedal tests)

`M4 second logs.csv` is Ben's no-load pedal test: Kp 11, P only, J 0.3. Replayed from rest with the logged pedal, the harness stays within 3.6 rpm RMS of the bench over 86 s.

| Approach to `revlim` | `govFlt` | Ring, bench / harness | Keep per cycle, bench / harness |
| --- | --- | --- | --- |
| 55.3 s | 4 | 0.640 / 0.640 s | 0.65 / 0.65 |
| 98.2 s | 4, then 2 at 98.79 s | 0.660 / 0.660 s | 0.36 / 0.36 |

Two settings are not logged, and the replay needed them to match:

- **`regenramp`:** finding 2.
- **The tick the shift changed:** the 0x4A3 frame showed it 52 ms late. `sil_replay` finds the true tick from the filter output logged in 0x4A0.

### The calculator against the harness (`sil_check.py --loopcalc`)

Each case is a 20 Nm load taken off at no load, P and PD only. Two things are compared:

- **Response:** the harness's whole response against the calculator's, as RMS difference over the peak.
- **Ring period:** judged wherever each cycle keeps at least 0.25. With fewer peaks than that, a period isn't well defined.

| `govKp` | `govKd` | `govFlt` | J | Extra delay | Harness ring | Calculator ring | Response difference |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 11 | 0 | 4 | 0.3 | 0 | 0.640 s, keeps 0.70 | 0.636 s, keeps 0.70 | 1.4 % |
| 11 | 0 | 3 | 0.3 | 0 | 0.480 s, 0.54 | 0.477 s, 0.54 | 1.1 % |
| 11 | 0 | 2 | 0.3 | 0 | 0.380 s, 0.36 | 0.376 s, 0.34 | 1.0 % |
| 11 | 0 | 4 | 0.3 | 10 ms | 0.640 s, 0.91 | 0.649 s, 0.91 | 2.3 % |
| 11 | 1 | 2 | 0.3 | 0 | 0.300 s, 0.69 | 0.303 s, 0.69 | 5.2 % |
| 6, 11, 25, 40 | 0 | 2 or 4 | 0.3 to 2.5 | 0 | keeps 0.00 to 0.16 | keeps 0.00 to 0.09 | 0.6 to 1.8 % |

The calculator also gets the M4 bench and the 3 Nov `govKp` sweep right. At shift 4 it gives 0.636 s and 0.70, against the bench's 0.640 s and 0.65, and at shift 2 it gives 0.376 s and 0.34, against 0.37 s and 0.35. At shift 4 and J 0.3, `govKp` 14 has 0.3° of margin and 18 is unstable. On the bench those sets ran at no-load p2p of 45 and 1349 rpm.

## Findings

1. **The loop delay is three ticks, not about two.**

   A speed sample reaches the model as torque three simulator ticks later, so 30 ms plus the 10 ms hold.

   - The Zombie's governor uses the speed it read at the end of its previous tick.
   - `Ms10Task` sends 0x1D4 (`selectedInverter->Task10Ms()`) before `SetTorque` stores this tick's request, so every 0x1D4 carries the previous tick's torque. All 3,800 frames in each bench log checked show this.
   - The simulator then applies the frame at its next tick.

   When the 0x1D4 arrives within about 2 ms after the simulator's tick, the delay is four ticks. The two 10 ms clocks drift by about 1.7 ms a minute, so a bench session drifts through that window every few minutes. The 3 Nov runs show `loop_ticks` 3.0, 4.0, or in between where the phase crossed during a run.

   The extra tick explains the bench's biggest run-to-run differences. Noise sweep set 4 and `govFlt` sweep set 3 had the same settings, but the bench gave 268.5 and 237.7 rpm of stroke ripple. They ran at 4 ticks and 3 ticks, and the harness gives 268.3 and 237.7 at those delays.

   `loop_ticks` is now in every sweep row. In the harness the delay is always 3. Use `set rptdelay 10` for 4; `sil_check` does this per set from the bench table.

2. **The bench Zombie's `regenramp` is at least 13.6 %/10 ms**, not the default 1. It is not logged. In the M4 log it never limited a step, and the largest step was 13.6 %. The harness uses 100, the parameter's maximum. With the default of 1, the M4 overshoot came out wrong by 190 rpm.

   A harness study at 1, 2, 3, 5, 10, 20 and 100 found three things:

   - **While governing, it hardly matters.** For a load dropping off or a sudden overrun, the governor asks for regen slower than 1 % a tick, so even the default never limits it.
   - **It matters for a fast run-up to `revlim` at low inertia.** At shift 4, 1 % keeps the shaft more than 50 rpm over the limit for 2.07 s, and 3 and above for 1.51 s.
   - **Nothing above 5 changes anything.**

   Ben will set the tractor to at least 5 (11 Nov 2026), which is still half as harsh a torque reversal as `throtramp` 10 on the drive side. The harness keeps 100, which behaves the same.

3. **With `govKi` above 0, the governor hunts at `revlim` like a relay.** Over the limit, `SpeedLimitCommand` clamps its output to `MIN(0, P + I + D)`. When the integral carries load (14 % for the no-load friction), a speed one rpm over `revlim` cuts the torque straight to zero. Back under, the torque returns at once. The loop settles into a small limit cycle that no linear model describes: about 0.45 s, 40 rpm peak to peak, at J 0.9.

   This is the no-load ripple the 3 Nov sweeps found: with `govKi` 0, no-load p2p was 1–2 rpm, and with `govKi` 1 it was 37–90 rpm. The bench and the harness agree. The calculator applies to P and PD only, and to PI loops while they stay below `revlim`.

   This is plan section 16's candidate change G5. Over the limit, it would stop throwing away a positive `P + I` when the integral is carrying load. It is for the harness only, behind a parameter, and needs Ben's sign-off before any Zombie branch.

4. **Sending this tick's request would buy 5–8° of margin at low inertia.** This is G6. If `SetTorque` ran before the inverter's `Task10Ms`, the loop would be two ticks. At J 0.3, `govKp` 11, the calculator gives 13° of margin at shift 4, against 7.5° today, and 31.5° at shift 2, against 23°. This also changes control logic, so it is for the harness only, with Ben's sign-off.

5. **Starting gains for 45° of margin, P only, three-tick loop:**

   | `govFlt` | J 0.3 | J 0.9 | J 2.5 | Per unit J |
   | --- | --- | --- | --- | --- |
   | 4 | 3.2 | 9.4 | 26 | 10.5 |
   | 3 | 4.7 | 14 | 39 | 15.5 |
   | 2 | 6.5 | 19 | 54 | 21 |
   | 1 | 8.4 | 25 | 69 | 28 |

   This confirms plan section 16's hand estimate of about 10×J at shift 4 and 20×J at shift 2. At four ticks these values are 5–13 % lower.

## How to run

From `tools/`, with g++ on `PATH` (CLAUDE.md, laptop tooling) and the Zombie tree at `../Stm32-vcu`, or set `ZOMBIE_DIR`:

```text
..\.venv\Scripts\python silport.py --golden
..\.venv\Scripts\python sweep.py scripts\sweep_road.yaml --port sil --out D:\runs\sil_road
..\.venv\Scripts\python run_script.py scripts\m5b_report.yaml --port sil --canlog-out sil_m5b.log
..\.venv\Scripts\python sil_check.py ..\docs\bench\results\2026-11-03_sweep_*_baler --out D:\runs\sil_check
..\.venv\Scripts\python sil_check.py --loopcalc --out D:\runs\sil_loopcalc
..\.venv\Scripts\python sil_replay.py "D:\Downloads\leaf sim testing logs\M4 second logs.csv" --out m4_sil.log
..\.venv\Scripts\python loopcalc.py --kp 5,10,15,20,25 --flt 4,2 --J 2.5
```

The tool self-test runs the golden test, one bench reproduction and one calculator case through the harness. It skips them if the Zombie tree is missing. CI checks the tree out from GitHub.

## Bench steps (after the M5b sweeps)

M5c needs nothing new on the bench. Once M5b steps 3 and 4 have produced their results folders:

1. **Ring periods.** Run `sil_check.py docs/bench/results/<date>_sweep_repeat_baler docs/bench/results/<date>_sweep_road --out <folder>`. Those sweeps end each set with the load off, so ring period is judged as well as stroke ripple (within 10 %). Noise sets are judged too, because the bench now restarts its noise on `preset`. It should print `PASS`. Look at `loop_ticks` for each set: a set at 4 is run in the harness with 10 ms of `rptdelay`.
2. **`regenramp`.** Set the tractor to at least 5 (finding 2). Check the bench Zombie reads 5 or more too, in the web interface.
3. **Optional: a fresh pedal test**, if Claude asks for one: no load, P only, pedal pressed from rest to past `revlim` a few times at shifts 4 and 2, GovLog on, `canlog` running. `sil_replay.py` then compares the rings.
