# Bench sweep results

One folder per sweep, named `<date>_<sweep name>`, so every number quoted in `docs/bench/` traces to a file (plan item I2). Each folder holds:

| File | What it is |
| --- | --- |
| `<sweep>.yaml` | The sweep exactly as run, copied from `tools/scripts/` |
| `metrics.csv` | The table `tools/sweep.py` wrote at the time |
| `metrics_rescored.csv` | The same bus logs scored again by the current `sweep.score()`, with the later metrics (no-load torque jitter, error, torque effort, `cost`) |
| `metrics_summary.csv` | Only for sweeps run with `repeats`: the mean and standard deviation per set |

The bus logs (`run_NN_bus.log`, a few MB each) stay out of git. For the 3 Nov 2026 sweeps they are on Ben's PC in `D:/Downloads/leaf sim testing logs/`, in the folder with the sweep's name. The one exception is `sweep_kp_baler`, whose logs are in `sweep_kp_baler_4`: folders `_1` to `_3` were plumbing runs and are not results. To re-score them:

```text
cd tools
..\.venv\Scripts\python -c "import sweep; print(sweep.score(r'<folder>\run_01_bus.log', 2200))"
```

## What is here

| Folder | Sweep | Quoted in |
| --- | --- | --- |
| `2026-11-03_sweep_kp_baler` | `govKp` 8–18, P only, placeholder baler | `M5a_sdo_sweeps.md` |
| `2026-11-03_sweep_ki_baler` | `govKi` 0–4 at `govKp` 11 | `M5a_sdo_sweeps.md` |
| `2026-11-03_sweep_flt_baler` | `govFlt` 4, 3, 2 at `govKp` 11, `govKi` 1 | `M5a_sdo_sweeps.md`, plan V5 |
| `2026-11-03_sweep_kd_baler` | `govKd` 0–4 at shift 2 | `M5a_sdo_sweeps.md` |
| `2026-11-03_sweep_noise_baler` | White speed noise 0, 5 and 15 rpm at shifts 4 and 2 | `M5a_sdo_sweeps.md`, plan V5. The no-load torque jitter figures are `noload_treq_tick_rms_nm` in the re-scored table. |

The first road sweeps (`sweep_road.yaml`, from 8 Nov) are not here. Their results are not valid (see `CLAUDE.md`), and the sweep has changed since.
