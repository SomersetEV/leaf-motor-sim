# PC tools

These turn tractor CAN logs into simulator parameters and load profiles, compare tractor and bench runs, and script bench tests. See plan section 9.

```text
python -m pip install -r requirements.txt
python selftest.py          # needs g++ on PATH; checks every tool against synthetic logs
```

| Script | Input | Output |
| --- | --- | --- |
| `loglib.py` | SavvyCAN CSV, candump log (`-l` or `-ta`), or telematics `session_XXXX.csv` (detected; `--format` overrides) | Named signals on a 10 ms grid; reports dropped `0x4A1` frames and gaps in each message's stream |
| `extract_params.py` | R1 and R2 logs (or R3 for the implement) | `params.json` with `J`, `Tc`, `b`, `c`, and a fit plot |
| `make_profile.py` | R5, R6 or R7 log, plus `params.json` | Profile CSV for `/profiles/` on the SD card |
| `compare_runs.py` | A tractor log and a bench log of the same scenario | Metrics table and overlay plot |
| `run_script.py` | YAML script and the simulator's serial port | Timed console commands. `--stream-out` saves the CSV stream and `--canlog-out` the bus log (`canlog 1`, candump format, readable by `loglib`). Bench scripts are in `scripts/` |
| `sweep.py` | Sweep YAML and the simulator's serial port | M5a: runs Zombie gain sets over SDO (`zset`) against a preset or SD profile, logs each run with `canlog`, and writes `metrics.csv`, scoring the no-load and loaded phases separately. A set can also set simulator model parameters (`sim: {noise: 5}`). Restores the original gains at the end. Examples: `scripts/sweep_*_baler.yaml` |

## Typical flow

```text
python extract_params.py r1_*.csv r2_*.csv -o params.json
python make_profile.py r6_baling.csv params.json -o ../sd_card/profiles/baling.csv --start 20
python compare_runs.py r4_tractor.csv r4_bench.csv --plot r4.png --csv r4.csv
```

`extract_params.py` prints the `set` commands for the simulator console. Type them, then `save`.

## What the fit can and cannot tell you

- From speed and torque request alone, only J / kT is identifiable. That ratio sets loop behaviour, so the bench stays valid. Reported torque (`0x1DA`) and shunt DC power in tractor logs have since confirmed kT = 0.25 Nm per bit (plan V1 and V3, 8 Nov 2026), so absolute values hold with the default `--kt`.
- The motor model uses `--kt`, `--tau`, `--tmax` and `--pmax`. `--kt` is confirmed, and `--tau` 30 ms agrees with the 20–30 ms lag seen between request and reported torque. `--tmax` and `--pmax` are still placeholders (plan V2).
- The torque-to-speed alignment depends on log timing, so `extract_params.py` tries delays of −2 to +2 ticks and keeps the best fit. The choice is recorded in `params.json`, and `make_profile.py` reuses it.
- A profile is the load at the speed the tractor actually ran. When testing gains that move speed far from it, use `profile exp`.

## Self-test

`selftest.py` compiles `synth/synth_log.cpp` together with the firmware's plain C++ libraries (`lib/`). It generates logs from the simulator's own plant with known parameters, then checks the following:
- the fit recovers `J`, `Tc`, `b` and `c` within 5 % (the plan's criterion; it currently achieves about 0.1 %);
- a generated baler profile matches the true load;
- the profile parses and replays through the firmware's profile code;
- dropped-frame counting, candump and telematics parsing, the metrics, and `run_script` timing.

## Telematics logs

The LilyGo telematics logger writes `session_XXXX.csv` with millisecond timestamps: ms since boot until the phone sets the clock, then unix ms, with a `TIME_SYNC` row giving the mapping. `loglib` converts the boot-time rows with it, so `t = 0` is the first frame and the summary prints the wall-clock start. Check the gap counts in the summary before fitting: as of November 2026 the logger keeps only about 15 % of frames in run mode (0.3 s bursts about every 2 s, plan V6), which is too little for `extract_params` or `make_profile`.
