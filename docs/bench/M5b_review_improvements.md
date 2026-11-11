# M5b bench test: review improvements

**Goal (plan section 15, M5b):** check on the bench the items that changed the firmware or the sweeps: report delay (I7), band-limited speed noise (I6), repeatable runs (I5), and the sweep changes (I4, I8) in a real road sweep. Then I10, the `sd ls` reboot, which Claude does with the board connected. The per-item state is in plan section 15, "Progress".

Same wiring as M5a: the Zombie in Run mode, the pedal clamped (100 % is simplest), and the Due **unpowered**. Claude can run steps 2 to 5 over COM3.

## What is new

| Item | Detail |
| --- | --- |
| `rptdelay` | `set rptdelay <ms>`, 0–50: the speed in `0x1DA` is that many milliseconds old, to the nearest 1 ms sub-step. `0x4B0` still carries the model's own speed. |
| `noise_fc`, `noise_bw` | Shape of the `set noise` speed noise. `noise_fc` 0 (the default) is white per tick, as before. Otherwise it is band-limited around `noise_fc` Hz with a −3 dB bandwidth of `noise_bw` Hz, and `noise` stays its standard deviation. The tractor's ripple, fitted from the 7 Nov logs by `tools/fit_noise.py`, is `noise 11`, `noise_fc 6`, `noise_bw 4`. |
| Repeatable runs | `preset` and `mark` restart both noise sources (speed and load), and `preset` restarts the pulse phase. A script run twice should give the same run, apart from timing. |
| Sweeps | `repeats: N` runs each set N times and writes `metrics_summary.csv`. Each set now ends with the load taken off for `tail_s` (default 4 s), and is scored for ring period, decay per cycle and phase margin there, plus `cost`. Replies to the gradient commands are counted, so a late `ok` can no longer be taken for the next command's reply. |
| NVS | The saved-parameter layout changed. **On the first boot after flashing, any parameters saved with `save` are ignored and the defaults load.** Re-enter and `save` anything you rely on. |

## Steps

1. **Flash and check the parameters.** `pio run -e tcan485 -t upload --upload-port COM3`, then `params`. The list ends with `noise_fc=0 noise_bw=4 rptdelay=0`, and the ranges show `noise_fc 0 .. 45`, `noise_bw 0.5 .. 50`, `rptdelay 0 .. 50`. `set rptdelay 51` returns `err value out of range`.

2. **Report delay and noise.** With the governor holding `revlim`, from `tools/`:

   ```text
   ..\.venv\Scripts\python run_script.py scripts\m5b_report.yaml --port COM3 --canlog-out m5b_report.log
   ..\.venv\Scripts\python m5b_check.py m5b_report.log
   ```

   It takes about 75 s at J 2.5, so the extra delay cannot make the loop ring. `m5b_check.py` should print `PASS`, with:
   - `0x1DA lags the model by 2 ticks (20 ms)` and the speed moving at least 20 rpm on the load step;
   - band noise with an SD of about 11 rpm, a spectral peak near 6 Hz, and shifts 4 and 2 passing about 1.9 and 6.2 rpm (within 20 %);
   - white noise of the same SD passing about 2.0 and 4.2 rpm.

3. **Repeatability (I5).**

   ```text
   ..\.venv\Scripts\python sweep.py scripts\sweep_repeat_baler.yaml --port COM3 --out <folder>
   ```

   Three runs of Kp 11, Ki 1, `govFlt` 2 on the baler, about 3 minutes. In `metrics.csv`, the three `stroke_p2p_rpm` values should be within 5 rpm of each other (before M5b they varied by about ±15). Each row now also has `ring_period_s`, `ring_decay`, `ring_pm_deg` and `cost`.

   Compare only runs with the same `loop_ticks` (added in M5c). The Zombie's and the simulator's clocks drift by about 1.7 ms a minute, and while the drift puts the Zombie's 0x1D4 just after the simulator's tick, the loop has 4 ticks of delay instead of 3. In the 3 Nov sweeps, the same settings gave 238 rpm at 3 ticks and 268 rpm at 4 (`docs/bench/M5c_sil.md`, finding 1). If the three runs straddle the boundary, run the sweep again.

4. **Road sweep.**

   ```text
   ..\.venv\Scripts\python sweep.py scripts\sweep_road.yaml --port COM3 --out <folder>
   ```

   Nine sets at under 2 minutes each, about 16 minutes. Check:
   - `peak_integral_pct` is 0 on the `govKi` 0 sets (the integral clear at each set);
   - `dropped_4a1` is 0, and the console never shows `replies missing`;
   - `ring_*` columns are filled on every row.

   The headline numbers are `load_rms_err_rpm`, `load_max_under_rpm` and `load_max_over_rpm` (the climb and the overrun), `ring_pm_deg`, and `cost`.

5. **Results into the repo.** For steps 3 and 4, copy `metrics.csv` (and `metrics_summary.csv`) with the sweep's YAML into `docs/bench/results/<date>_<sweep name>/`, as in `docs/bench/results/README.md`. Keep the bus logs outside git.

6. **SD over serial (I10), Claude with the board connected.** Apply the parked stash (`git stash list`: "SD card pull over serial"), rebuild, and run `sd ls` with the serial monitor capturing the panic backtrace. Decode it with the firmware's `.elf`, fix the cause (suspects: console task stack, or SD access from the wrong task), then use `sd get` or `tools/sd_pull.py` to take `topper.json` off the card.

7. **CI (I3).** After the next push, the Actions tab on GitHub shows the `CI` workflow green on `main`: native tests, tool self-test, firmware build.

Send back the folders from steps 3 and 4, or let Claude run them and read them directly.
