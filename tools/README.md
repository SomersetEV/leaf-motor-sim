# PC tools

These turn tractor CAN logs into simulator parameters and load profiles, compare tractor and bench runs, and script bench tests. See plan section 9.

```text
python -m pip install -r requirements.txt
python selftest.py          # needs g++ on PATH; checks every tool against synthetic logs
```

| Script | Input | Output |
| --- | --- | --- |
| `loglib.py` | SavvyCAN CSV or candump log (`-l` or `-ta`) | Named signals on a 10 ms grid; reports dropped `0x4A1` frames |
| `extract_params.py` | R1 and R2 logs (or R3 for the implement) | `params.json` with `J`, `Tc`, `b`, `c`, and a fit plot |
| `make_profile.py` | R5, R6 or R7 log, plus `params.json` | Profile CSV for `/profiles/` on the SD card |
| `compare_runs.py` | A tractor log and a bench log of the same scenario | Metrics table and overlay plot |
| `run_script.py` | YAML script and the simulator's serial port | Timed console commands; `scripts/baler_example.yaml` is an example |

## Typical flow

```text
python extract_params.py r1_*.csv r2_*.csv -o params.json
python make_profile.py r6_baling.csv params.json -o ../sd_card/profiles/baling.csv --start 20
python compare_runs.py r4_tractor.csv r4_bench.csv --plot r4.png --csv r4.csv
```

`extract_params.py` prints the `set` commands for the simulator console. Type them, then `save`.

## What the fit can and cannot tell you

- From speed and torque request alone, only J / kT is identifiable. That ratio sets loop behaviour, so the bench stays valid. Absolute values need reported torque (`0x1DA`, plan V3) or DC power from a shunt.
- The motor model uses `--kt`, `--tau`, `--tmax` and `--pmax`. Their defaults are the plan's placeholders, which still need checking (plan V1 and V2).
- The torque-to-speed alignment depends on log timing, so `extract_params.py` tries delays of −2 to +2 ticks and keeps the best fit. The choice is recorded in `params.json`, and `make_profile.py` reuses it.
- A profile is the load at the speed the tractor actually ran. When testing gains that move speed far from it, use `profile exp`.

## Self-test

`selftest.py` compiles `synth/synth_log.cpp` together with the firmware's plain C++ libraries (`lib/`). It generates logs from the simulator's own plant with known parameters, then checks the following:
- the fit recovers `J`, `Tc`, `b` and `c` within 5 % (the plan's criterion; it currently achieves about 0.1 %);
- a generated baler profile matches the true load;
- the profile parses and replays through the firmware's profile code;
- dropped-frame counting, candump parsing, the metrics, and `run_script` timing.
