# M2 bench test: CAN bring-up

**Goal (plan section 11, M2):** SavvyCAN shows `0x1DA` every 10 ms ± 0.5 ms and `0x55A` every 100 ms. The Zombie web interface shows `udc` at 360 V, `speed` 0, and both temperatures at 40 °C.

In this build the simulator holds speed at zero and ignores the torque request. The motor model arrives in M3.

## 1. Wire

| From | To |
| --- | --- |
| LilyGo CAN H / CAN L | Bench bus CAN H / CAN L |
| Zombie inverter CAN H / CAN L | Bench bus |
| USB-CAN adapter | Bench bus, SavvyCAN in listen-only |
| LilyGo USB | PC (power and serial) |
| Zombie | 12 V bench supply |

- **Termination:** the bus needs exactly two 120 Ω. With everything unpowered, measure CAN H to CAN L: it should read about 60 Ω. About 120 Ω means one is missing; about 40 Ω means there are three.
- **Power the Zombie** for every test. SavvyCAN in listen-only mode does not acknowledge frames. If the Zombie is off, nothing acknowledges the simulator's frames and the LilyGo keeps retrying.

## 2. Zombie parameters

Load the tractor's parameter file, then set:

| Parameter | Value |
| --- | --- |
| `Inverter` | `Leaf_Gen1` |
| `ShuntType` | `None` (0) |
| `BMS_Mode` | `Off` |
| `InverterCan`, `CanMapCan` | The bus wired to the bench |

## 3. Flash and open the console

From the `leaf-motor-sim` folder:

```text
pio run -e tcan485 -t upload
pio device monitor -e tcan485
```

You should see:

```text
leaf-motor-sim M2 CAN bring-up
ok running, type help
```

If it prints `err CAN driver failed to start`, stop and send me the serial output.

## 4. Check

**SavvyCAN.** Load `tools/dbc/leaf_sim.dbc`. Then:

- `0x1DA` arrives every 10 ms. Check the interval spread over at least a minute: every interval should be between 9.5 and 10.5 ms.
- `0x55A` arrives every 100 ms.
- `INV_Status_1DA`: `Voltage` = 360 V, `MotorSpeed` = 0 rpm, `ErrorBits` = 0.
- `INV_Temps_55A`: `MotorTemp` and `InverterTemp` ≈ 40 °C.

**Zombie web interface:** `udc` = 360, `speed` = 0, both inverter and motor temperatures = 40.

**Simulator console.** Type `stat` a few times over a minute. Expected:

| Field | Expected |
| --- | --- |
| `jitter_1s_us`, `jitter_max_us` | Below 500 |
| `overruns`, `out_of_tol` | 0 |
| `tx_failed` | 0, or not increasing |
| `can` | `running` |
| `tec`, `rec` | 0, or low single digits |
| `rx50b` | Increasing (the Zombie is awake on the bus) |
| `rx_timeout` | 1 unless the Zombie is in run mode |

If the Zombie reaches run mode (`opmode` = Run), `rx1d4` starts increasing, `rx_timeout` goes to 0, and `rej1d4` should stay at 0. Getting this far is a bonus. It is not required for M2.

## 5. If it fails

| Symptom | Likely cause |
| --- | --- |
| No `0x1DA` in SavvyCAN, `tx_failed` rising, `tec` 128 or above | Nothing is acknowledging the frames: Zombie off, CAN H/L swapped, wrong bus, or termination missing. |
| `can=bus_off`, `bus_off` count rising | Wiring fault or wrong bit rate. The firmware recovers automatically. |
| Frames in SavvyCAN but the Zombie shows `udc` 0 | `ShuntType` is not 0, or `InverterCan` is set to the other bus. |
| `rx50b` stays at 0 although the Zombie is on | The simulator receives nothing: check the CAN pins (TX 27 / RX 26, from the telematics firmware). |

Send back: a SavvyCAN capture of at least one minute (CSV), a `stat` line taken at the end, and a screenshot of the Zombie spot values.
