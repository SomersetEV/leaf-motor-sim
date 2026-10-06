# M4 bench test: Zombie governor logging

**Goal (plan section 11, M4):** the golden test passes before and after the change. With `GovLog` off, no new frames appear. With it on, `0x4A0` and `0x4A1` appear every 10 ms and decode sensibly through the DBC.

The firmware is on branch `gov-logging` of `SomersetEV/Stm32-vcu`, based on `Tractor-RPM-limiting-testing` at `d85e756`. The local working copy is `github/Stm32-vcu-gov-logging`, a git worktree of your existing clone.

## What changed in the Zombie

| Item | Detail |
| --- | --- |
| `GovLog` (param 163, Governor) | `Off` by default. When `On` and in Run mode, the Zombie sends `0x4A0` and `0x4A1` every 10 ms and `0x4A2` and `0x4A3` every 100 ms on the `CanMapCan` bus. |
| `govFlt` (param 162, Governor) | Speed filter shift, 0 to 6, default 4, which is today's filter. Shift 4 is a time constant of about 155 ms, 3 about 75 ms, 2 about 35 ms. Takes effect as soon as it is set. |
| Spot values | `govErr` (rpm), `govP`, `govD` and `potuser` (%), alongside the existing `govOut` and `govInt`. |
| Governor arithmetic | Unchanged. The host golden test checks 500 ticks bit for bit against the original code. |

The source comment that claimed the speed filter had been cut to about 30 ms has been corrected. The filter at shift 4 has always been about 155 ms.

## 1. Build and flash

From `Stm32-vcu-gov-logging`, with the GNU Arm toolchain on `PATH`:

```text
make get-deps     (first time only; builds libopencm3)
make
```

On Windows, if `make get-deps` fails because `python` is not found, run `make -C libopencm3 TARGETS=stm32/f1 PYTHON="py -3"` instead. That is how it was built here.

Host tests: `cd test && make && ./test_vcu`. The run should end with `All tests passed`, including `TestGovernorGolden`.

Flash `stm32_vcu.bin` or `stm32_vcu.hex` the way you normally update the Zombie. Saved parameters are kept. Afterwards the Governor category shows `govFlt = 4` and `GovLog = Off`.

## 2. Bench setup

Use the M3 bench setup: the simulator running, the Zombie in Run mode, and SavvyCAN logging the bench bus with `tools/dbc/leaf_sim.dbc` loaded. `CanMapCan` must be the bench bus.

## 3. Checks

1. **`GovLog` off.** In Run mode, SavvyCAN shows no frames with IDs `0x4A0` to `0x4A3`.
2. **`GovLog` on.** Set `GovLog = On`, no reboot needed. Check:
   - `0x4A0` and `0x4A1` arrive every 10 ms, and `0x4A2` and `0x4A3` every 100 ms.
   - None of them appear outside Run mode.
3. **Decode through the DBC.** With the throttle held so the governor holds `revlim`:
   - `GOV_Speed_4A0.GovSpeedIn` follows the simulator's speed, one tick behind.
   - `GovSpeedErr` is near zero, and `PotUser` shows your pedal position.
   - `GOV_Terms_4A1`: `GovP + GovD + GovInt` matches `govOut` in the web interface. `GovBinding` = 1 while the governor holds the speed down.
   - `GovCounter` steps by 1 every frame with no gaps.
   - `GOV_Gains_4A2`: `Revlim`, `GovKp`, `GovKi` and `GovKd` equal the values entered in the web interface.
   - `GOV_State_4A3`: `Opmode` = Run, and `GovFlt` = 4.
4. **Pedal is the limit.** Hold the pedal low so speed stays below `revlim`. `GovIntFrozen` = 1 and `GovBinding` = 0.
5. **`govFlt` change.** Set `govFlt = 2`. `GOV_State_4A3.GovFlt` follows, and the governor responds faster. Set it back to 4.
6. **Bus load.** SavvyCAN's bus load rises by about 6 % with `GovLog` on.

## 4. First use on the tractor

Turn `GovLog` on with the tractor stationary. Confirm the MG charger's DC-DC output stays up for a few minutes before doing any field work. DC-DC dropouts on this machine have coincided with extra CAN traffic before. If the DC-DC drops out, turn `GovLog` off and tell me.

Send back: a SavvyCAN capture (CSV) covering checks 1 to 5.
