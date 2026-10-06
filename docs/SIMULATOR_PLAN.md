# Leaf Motor Simulator: Implementation Plan

Oct 6, 2026 · @Ben

## 1. Purpose and scope

The goal is a bench rig where a real ZombieVerter runs against a simulated Nissan Leaf inverter and motor, so the RPM governor can be tested and tuned without the tractor.

The work splits into two parts:

- **Part A, simulator firmware.** New firmware for a LilyGo T-CAN485 (ESP32). It impersonates the Leaf inverter on CAN and runs a motor and load model in closed loop with the Zombie.
- **Part B, Zombie logging changes.** Small additions to the tractor branch of the Zombie firmware that expose governor internals as spot values, so they can be logged over CAN on the tractor and on the bench.

A third, smaller piece is PC tooling in Python that turns tractor CAN logs into model parameters and load profiles for the simulator.

**Phase 1 target.** Reproduce on the bench two behaviours already seen on the Leyland 255, then tune against them:

- Hitting the RPM limit quickly at no load gives a dip, then a slow creep back up to the limit.
- During baling, the governor integral winds up across a plunger stroke and the governor loses regulation between strokes.

Tuning covers `govKp`, `govKi`, `govKd`, `govImax` and the speed filter shift.

**Out of scope for phase 1.** These are later extensions and must not be built yet: throttle DAC output, driving the Zombie digital inputs, BMS, shunt or charger simulation, a two-mass driveline model, and automated gain sweeps over CAN SDO.

## 2. Working rules for Claude in VS Code

Build in the milestone order of section 11 and stop at each milestone for a hardware test by Ben. Nothing here can be verified on real hardware from the editor.

**Source of truth.** Facts about the Zombie firmware in this plan were read from `SomersetEV/Stm32-vcu`, branch `Tractor-RPM-limiting-testing`, commit `d85e756` (15 Sep 2026). Re-check each one against the working tree before relying on it. Where the tree differs, the tree wins and the difference gets reported.

**Repositories.**

| Work | Location |
| --- | --- |
| Simulator firmware and PC tools | New repo `leaf-motor-sim`, PlatformIO project |
| Zombie logging changes | New branch `gov-logging` off `Tractor-RPM-limiting-testing` in `SomersetEV/Stm32-vcu` |
| This plan | `docs/SIMULATOR_PLAN.md` in `leaf-motor-sim`, exported from this doc as Markdown |
|  |  |

**Rules.**

1. Part B changes are logging only. Governor behaviour must be bit-for-bit unchanged with the new parameters at their defaults.
2. Ask before changing any control logic, any existing parameter default, or anything outside phase 1 scope.
3. Keep all simulator logic that is not hardware access in plain C++ with no Arduino or ESP-IDF includes, so it builds and unit-tests on the PC.
4. Nothing in the 10 ms simulation loop may block: no SD access, no serial writes that can stall, no heap allocation.
5. Interfaces use rpm, Nm, kg·m², volts and milliseconds. The model uses rad/s internally.
6. Every milestone delivers a short bench test procedure: what to wire, what to type, what Ben should see.
7. Items marked **VERIFY** are assumptions. Do not hard-code them without a named constant and a comment pointing at section 12.

## 3. Bench architecture

Three nodes share one 500 kbit/s CAN bus: the real Zombie, the LilyGo simulator, and a USB-CAN adapter that only listens. There is no high voltage anywhere on the bench.

| Node | Real or simulated | Connections |
| --- | --- | --- |
| ZombieVerter VCU | Real hardware, real firmware (device under test) | 12 V bench supply. Inverter CAN to the bench bus. |
| LilyGo T-CAN485 | Simulated Leaf inverter, motor and load | USB to PC for power and serial. CAN H/L to the bench bus. |
| USB-CAN adapter with SavvyCAN | Real, independent logger | Bench bus, listen-only. |
| Throttle pot | Real | Zombie throttle 1 input, as on the tractor. |
| Ignition, start, direction, brake switches | Real | Zombie digital inputs at 12 V. |
| Contactor and precharge outputs | Not connected | Optional indicator lamps. |

The bus needs exactly two 120 Ω terminations. Check which of the three nodes already carry one before adding any.

### How the Zombie starts against the simulator

These points come from the source and drive the simulator design:

- `LeafINV::Task10Ms()` only runs in `MOD_RUN`, so the Zombie sends no `0x11A` or `0x1D4` frames during precharge.
- With `ShuntType = 0`, `udc` is taken only from byte 0 of `0x1DA`, and only when the decoded value is below 420 V.
- Precharge completes when `udc >= udcsw`, the throttle is released, and `udc < udclim`.

The simulator must therefore send `0x1DA` from power-up without waiting for any frame from the Zombie. Otherwise precharge times out and the Zombie enters `MOD_PCHFAIL`.

### Zombie bench parameters

Load the tractor's saved parameter file onto the bench Zombie first, so throttle maps, ramps and governor settings match the machine. Then set only these:

| Parameter | Bench value | Reason |
| --- | --- | --- |
| `Inverter` | `Leaf_Gen1` | Uses `leafinv.cpp` and `NissLeafMng.cpp`. |
| `ShuntType` | `None` (0) | Makes `udc` come from the simulated `0x1DA`. |
| `BMS_Mode` | `Off` | No BMS on the bench. |
| `udcsw` | Below simulator voltage | Lets precharge complete. |
| `udcmin`, `udclim` | Below and above simulator voltage | Avoids voltage derate hiding governor behaviour. |
| `InverterCan`, `CanMapCan` | The bus wired to the bench | Puts control and log frames on the logged bus. |
| `revlim` | As on the tractor for the test case | Governor setpoint. |

Set the simulator's reported voltage to the tractor's nominal pack voltage, so the voltage limits behave as they do on the machine.

## 4. Part A: hardware, toolchain and project layout

The simulator is a PlatformIO project for the LilyGo T-CAN485, using the Arduino-ESP32 framework with the ESP-IDF TWAI driver (`driver/twai.h`) and FreeRTOS tasks.

### Pins

These values are from LilyGO's own example `config.h` for the board ([source](https://raw.githubusercontent.com/Xinyuan-LilyGO/T-CAN485/main/example/Arduino/CAN/config.h)).

| Signal | GPIO | Note |
| --- | --- | --- |
| `PIN_5V_EN` | 16 | Drive high to power the transceiver supply. |
| `CAN_TX_PIN` | 26 | **VERIFY** against Ben's working telematics firmware. |
| `CAN_RX_PIN` | 27 | **VERIFY**, some published pinouts show TX and RX swapped. |
| `CAN_SE_PIN` | 23 | Drive low to take the transceiver out of standby. |
| SD MISO, MOSI, SCLK, CS | 2, 15, 14, 13 | SPI microSD. |
| `WS2812_PIN` | 4 | Status LED. |

Put every pin in `include/pins.h`. If Ben's telematics project for this board is available, copy its CAN pin definitions, because they are proven on his hardware.

### PlatformIO environments

| Environment | Purpose |
| --- | --- |
| `tcan485` | Firmware build for the board. Serial monitor at 921600 baud. |
| `native` | PC build that runs the unit tests for codecs, plant and loads (Unity). |

### Layout

```text
leaf-motor-sim/
  platformio.ini
  docs/SIMULATOR_PLAN.md
  include/
    pins.h            board pins
    config.h          CAN ids, periods, default model parameters
  lib/                pure C++, no Arduino includes
    leafcodec/        decode 0x1D4 and 0x11A, encode 0x1DA and 0x55A, Nissan CRC
    plant/            motor torque envelope, torque lag, inertia integration
    loads/            load components, presets, profile player
    simproto/         telemetry frame packing, console command parser
  src/                hardware-facing code
    main.cpp          setup, task creation
    can_io.cpp        TWAI init, receive task, transmit helper, bus-off recovery
    sim_task.cpp      the 10 ms loop
    console.cpp       serial commands and CSV streaming
    sd_profiles.cpp   load profile and preset files into RAM
    status_led.cpp
  test/
    test_leafcodec/  test_plant/  test_loads/  test_simproto/
  tools/              PC-side Python, see section 9
    dbc/leaf_sim.dbc
```

## 5. Part A: CAN interface specification

The simulator needs to decode one frame properly (`0x1D4`) and transmit two (`0x1DA`, `0x55A`). The bus is 500 kbit/s with 11-bit identifiers.

### Frames received from the Zombie

| ID | Zombie period | Simulator use |
| --- | --- | --- |
| `0x1D4` | 10 ms, only in `MOD_RUN` | Torque request and HV status. The only frame that drives the model. |
| `0x11A` | 10 ms, only in `MOD_RUN` | Gear byte and on/off byte, recorded for telemetry only. |
| `0x50B` | 100 ms while awake | Presence only. |
| `0x1F2`, `0x1DB`, `0x1DC`, `0x55B`, `0x59E`, `0x5BC` | 10 or 100 ms | Ignored. Charger and battery spoof frames. |

Decode of `0x1D4`, mirroring `NissLeafMng::Task10Ms`:

```cpp
// bytes 2..3: signed 12-bit torque request, left-justified
int16_t raw = (int16_t)((b[2] << 4) | (b[3] >> 4));
if (raw & 0x800) raw -= 0x1000;          // sign-extend from bit 11
// Zombie side: raw = torquePercent * 2047 / 100  (LeafINV::SetTorque)
float torque_req_nm = raw * K_T;         // K_T = 0.25 Nm per bit, VERIFY
uint8_t hv_status = b[4] & 0x0F;         // 7 = HV on, 3 = precharge, 2 = off
uint8_t counter   = b[4] >> 6;           // 0..3, increments each frame
bool    crc_ok    = nissan_crc(b) == b[7];   // polynomial 0x85
```

Port `nissan_crc` exactly from `NissLeafMng.cpp`. Frames with a bad CRC are ignored and counted, as a real inverter would do.

If no valid `0x1D4` arrives for 100 ms, the model applies zero motor torque and sets the timeout flag. This is the normal state whenever the Zombie is not in run mode.

**Torque scaling matters for tuning.** At 0.25 Nm per bit, 100 % throttle is 2047 counts, about 512 Nm. That is well above what the motor can produce, so the real inverter clips. On a 280 Nm motor, everything above roughly 55 % commands no extra torque. The model's torque envelope in section 6 must reproduce this, because it changes the effective governor gain.

### Frames transmitted by the simulator

`0x1DA`, DLC 8, every 10 ms from power-up, matching `LeafINV::DecodeCAN`:

```cpp
b[0] = (uint8_t)lroundf(udc_volts / 2.0f);   // Zombie: voltage = b[0] * 2
b[1] = 0;
b[2] = 0;  b[3] = 0;                         // reported torque, see below
int16_t raw = (int16_t)lroundf(rpm * 2.0f);  // 0.5 rpm per bit, signed, clamp to +/-16383 rpm
b[4] = (uint8_t)(raw >> 8);
b[5] = (uint8_t)(raw & 0xFF);
b[6] = inverter_error ? 0x10 : 0x00;         // Zombie: error = (b[6] & 0xB0) != 0
b[7] = 0;
```

The Zombie decodes speed as `(b[4] << 7) | (b[5] >> 1)` and subtracts `0x7FFF` when the result exceeds `0x3FFF`. It therefore sees whole rpm, and reverse speeds read 1 rpm off. Encode as the real inverter does and do not compensate.

The Zombie does not check a counter or CRC on `0x1DA`, and has no receive timeout for it.

A real inverter also reports its actual torque in `0x1DA`. The believed layout is 11-bit signed at 0.5 Nm per bit in `((b[2] & 0x07) << 8) | b[3]`. **VERIFY** this from a tractor capture before filling it. Keep it behind a `SIM_REPORT_TORQUE` config flag, default off.

`0x55A`, DLC 8, every 100 ms:

| Byte | Content | Default |
| --- | --- | --- |
| 1 | Motor temperature in °F | 104 (40 °C) |
| 2 | Inverter temperature in °F | 104 (40 °C) |
| others | Zero |  |

### Simulator telemetry frames

These are the simulator's own frames, so that model internals land in the same SavvyCAN log as everything else. All multi-byte values are little-endian. IDs are constants in `config.h`.

| ID | Period | Bytes | Signal | Type and scale |
| --- | --- | --- | --- | --- |
| `0x4B0` | 10 ms | 0–1 | Model speed | int16, 0.5 rpm |
|  |  | 2–3 | Requested torque as decoded | int16, 0.1 Nm |
|  |  | 4–5 | Applied motor torque after envelope and lag | int16, 0.1 Nm |
|  |  | 6–7 | Load torque | int16, 0.1 Nm |
| `0x4B1` | 100 ms | 0–1 | Inertia | uint16, 0.001 kg·m² |
|  |  | 2 | Active preset id | uint8 |
|  |  | 3 | Profile state: 0 idle, 1 loaded, 2 running, 3 finished | uint8 |
|  |  | 4 | Flags: bit0 rx timeout, bit1 envelope clipping, bit2 noise on, bit3 fault injected, bit4 bus error seen | uint8 |
|  |  | 5 | Count of rejected `0x1D4` frames, wrapping | uint8 |
|  |  | 6 | Count of 10 ms tick overruns, wrapping | uint8 |
|  |  | 7 | Worst tick jitter in the last second | uint8, 0.1 ms |
| `0x4B2` | On event | 0 | Event: 1 marker, 2 load step, 3 profile start, 4 profile stop, 5 preset change, 6 parameter change | uint8 |
|  |  | 1–2 | Event value or marker number | uint16 |
|  |  | 3–6 | Simulator time | uint32, ms |

No ID from `0x4A0` to `0x4BF` is sent or registered anywhere in the Zombie source at the reference commit. The Zombie log frames in section 8 use `0x4A0`–`0x4A3`. Both ranges have lower bus priority than the 10 ms Leaf control frames.

Write `tools/dbc/leaf_sim.dbc` covering `0x11A`, `0x1D4`, `0x1DA`, `0x55A`, `0x4A0`–`0x4A3` and `0x4B0`–`0x4B2`, so SavvyCAN can graph every signal by name.

## 6. Part A: plant and load model

The model is a single rotating inertia driven by a torque-limited, lagged motor and opposed by a configurable load. It lives in `lib/plant` and `lib/loads` as plain C++.

### Equations

Motor torque is the request, clipped to the motor's envelope, then lagged:

```latex
T_{cmd} = \operatorname{clamp}\left(T_{req},\ \pm\min\left(T_{max},\ \frac{P_{max}}{|\omega|}\right)\right)
\qquad
\tau\,\frac{dT_m}{dt} = T_{cmd} - T_m
```

The shaft accelerates on the difference between motor torque and load:

```latex
J\,\frac{d\omega}{dt} = T_m - T_{act} - \operatorname{sgn}(\omega)\,T_{res}
```

Loads fall into two classes. Resistive loads always oppose rotation and cannot turn the shaft. Active loads are signed and can drive it, which is the overrun case.

```latex
T_{res} = T_c + b\,|\omega| + c\,\omega^2 + T_{step}(t) + T_{pulse}(t) + T_{noise}(t) + T_{prof}(t)\left(\frac{|\omega|}{\omega_{ref}}\right)^{n}
\qquad
T_{act} = T_{grad}
```

### Integration

Each 10 ms tick runs ten 1 ms sub-steps, which keeps the integration stable with small inertias. Standstill needs care so that friction holds the shaft instead of oscillating it:

```cpp
// one 1 ms sub-step, dt = 0.001, w in rad/s
float t_drive = t_motor - t_active;                    // net of signed loads
float t_res   = resistive_torque(fabsf(w), t);         // magnitude, >= 0
if (fabsf(w) < W_EPS && fabsf(t_drive) <= t_res) {
    w = 0.0f;                                          // held at standstill
} else {
    float dir   = (fabsf(w) >= W_EPS) ? copysignf(1.0f, w) : copysignf(1.0f, t_drive);
    float w_new = w + (t_drive - dir * t_res) / J * dt;
    if (fabsf(w) >= W_EPS && w_new * w < 0.0f) w_new = 0.0f;   // friction cannot reverse the shaft
    w = w_new;
}
```

Measurement noise is added to the rpm reported in `0x1DA` only, never to the model state.

### Model parameters

Every default below is a placeholder until the identification runs in section 9 replace it. All are settable at run time from the console and saved to NVS.

| Name | Meaning | Placeholder | Real value comes from |
| --- | --- | --- | --- |
| `J` | Total inertia referred to the motor shaft | 0.30 kg·m² | No-load acceleration run |
| `Tc` | Constant friction torque | 5 Nm | Coast-down |
| `b` | Viscous coefficient | 0.02 Nm·s/rad | Coast-down |
| `c` | Quadratic coefficient | 0 | Coast-down |
| `tau` | Motor torque lag | 30 ms | Requested against reported torque in logs |
| `Tmax` | Motor torque limit | 280 Nm | **VERIFY** motor type on the tractor |
| `Pmax` | Motor power limit | 80 kW | **VERIFY** motor type on the tractor |
| `kT` | Torque per request bit | 0.25 Nm | **VERIFY** from logs |
| `udc` | Reported bus voltage | 360 V | Tractor pack nominal voltage |
| `noise` | Speed noise, standard deviation | 0 rpm | Steady no-load log |

### Load components

| Component | Class | Parameters | Represents |
| --- | --- | --- | --- |
| Constant | Resistive | `Tc` | Friction, hydraulic pump at fixed pressure |
| Viscous | Resistive | `b` | Gearbox oil churn |
| Quadratic | Resistive | `c` | Fans, topper or mower blades |
| Step | Resistive | Torque, ramp time, optional inertia change | PTO engagement, relief valve, sudden load loss |
| Pulse | Resistive | Peak torque, period, duty, half-sine shape | Baler plunger |
| Noise | Resistive | Standard deviation, corner frequency | Draft work |
| Profile | Resistive | File, loop flag, speed exponent `n`, `w_ref` | Replay of a real job from a tractor log |
| Gradient | Active | Signed torque | Slope, overrunning load |

PTO engagement is a resistive torque ramp, with any inertia change applied at the end of the ramp. Clutch-slip dynamics are out of scope for phase 1.

### Presets and profiles

A preset is a named set of model parameters and load components. Compile in `noload`, `topper`, `baler` and `pto_step` with placeholder numbers. Presets on the SD card in `/presets/<name>.json` override them.

A profile is a CSV file in `/profiles/` with a header and one row per 10 ms:

```csv
t_ms,torque_nm,inertia_kgm2
0,12.5,0.30
10,12.9,0.30
20,14.1,0.30
```

The PC tool always writes rows at exactly 10 ms spacing, so the firmware indexes rows and never interpolates. The inertia column is optional. Load the whole file into RAM as int16 at 0.1 Nm before a run starts, and cap it at 60,000 rows, which is 10 minutes.

## 7. Part A: firmware architecture

A steady 10 ms tick is the one hard requirement. The Zombie governor's gains and filters are per tick, so jitter in the speed frame is equivalent to changing the tuning.

### Tasks

| Task | Core | Priority | Job |
| --- | --- | --- | --- |
| `sim_task` | 1 | Highest | The 10 ms loop, timed with `vTaskDelayUntil` at a 1 kHz tick rate. |
| `can_rx_task` | 1 | High | Blocks on `twai_receive`, decodes frames, updates a shared latest-command struct with a timestamp. |
| `console_task` | 0 | Low | Serial commands, CSV streaming, SD file loading. |

WiFi and Bluetooth stay off in phase 1.

### The 10 ms tick, in order

1. Copy the latest decoded `0x1D4` command under a short critical section. Apply the 100 ms receive timeout.
2. Advance load components and the profile index by one tick.
3. Run ten 1 ms model sub-steps.
4. Encode and queue `0x1DA`, then `0x4B0`.
5. Every tenth tick, queue `0x55A` and `0x4B1`.
6. Push one sample into a lock-free ring buffer for the console task to stream.
7. Record the tick period in microseconds for jitter statistics.

Target: tick period within 10 ms ± 0.5 ms, with zero overruns over a 10 minute run. Report worst-case jitter in `0x4B1` and in the `stat` command.

### CAN driver

Configure TWAI at 500 kbit/s in normal mode with an accept-all filter and a transmit queue of at least 16 frames. Transmit with a zero timeout so a full queue never blocks the tick. Enable alerts for bus-off and recover automatically with `twai_initiate_recovery`. Set the bus-error flag in `0x4B1` when it happens.

### Serial console

Line-based commands at 921600 baud. Every command replies with one line starting `ok` or `err`.

| Command | Action |
| --- | --- |
| `help` | List commands. |
| `stat` | Speed, torques, timeout state, jitter, overruns, CAN error counters. |
| `params` | Print all model parameters. |
| `set <name> <value>` | Set a model parameter from section 6. |
| `save` | Store parameters to NVS. |
| `preset <name>` | Apply a preset. |
| `load step <Nm> [ramp_ms] [dJ]` | Start a resistive torque step. |
| `load pulse <Nm> <period_ms> <duty_pct>` | Start a periodic pulse load. |
| `load grad <Nm>` | Set the signed active torque. |
| `load off` | Remove step, pulse, gradient and noise loads. |
| `profile load <file>` | Read a profile into RAM. |
| `profile start`, `profile stop`, `profile loop <0\|1>` | Control replay. |
| `mark <n>` | Send a marker event in `0x4B2`. |
| `fault inv <0\|1>` | Set or clear the inverter error bit in `0x1DA`. |
| `stream <0\|1> [divider]` | CSV streaming on or off, optionally every Nth tick. |

Streamed CSV columns: `t_ms,raw_req,t_req_nm,t_motor_nm,t_load_nm,rpm,flags`. If the ring buffer overflows, drop samples and count them. Never block the tick.

Every `load`, `preset`, `profile start` and `profile stop` command also emits a `0x4B2` event, so the CAN log shows exactly when each disturbance began.

### Status LED

| Colour | Meaning |
| --- | --- |
| Blue | Running, no valid `0x1D4` (Zombie not in run mode). |
| Green | Running, receiving valid torque requests. |
| Yellow | Profile replay in progress. |
| Red | CAN bus-off or tick overrun in the last second. |

## 8. Part B: Zombie firmware logging changes

The Zombie needs one real addition: governor internals sent on CAN every 10 ms. Today CAN-mapped spot values go out every 100 ms, because `canMap->SendAll()` is called from `Ms100Task`. That gives three to five samples across a baler plunger stroke, which is too few to tune against.

### What already exists at the reference commit

| Item | Where | Detail |
| --- | --- | --- |
| Governor | `Throttle::SpeedLimitCommand` in `src/throttle.cpp` | Float speed filter, derivative on measured speed, conditional integration, regen clamp at `regenmax`. |
| Call path | `Ms10Task` → `utils::ProcessThrottle` → `SpeedLimitCommand` | Runs every 10 ms in `MOD_RUN`, using the absolute speed from the previous tick. |
| Parameters | `include/param_prj.h` | `revlim` (15), `govKp` (157, scaled ×0.01), `govKi` (158, scaled ×0.001), `govImax` (159), `govKd` (160). |
| Spot values | `include/param_prj.h` | `govOut` (2124), `govInt` (2125). |
| Host tests | `test/test_throttle.cpp` | Existing PC-side tests for throttle functions. |

### Changes

**B1. Expose governor internals.** Move the function-static governor state in `SpeedLimitCommand` to static members of `Throttle`, and add members for the values below. The arithmetic must not change.

- Speed error in rpm, P term, D term, both in percent.
- User throttle before any limiting, captured in `ProcessThrottle` straight after `GetUserThrottleCommand()`.
- A flags byte: bit0 governor is the binding limit this tick, bit1 over the limit, bit2 integral reset branch taken, bit3 integral frozen, bit4 integral clamped at `govImax`.

Add spot values `govErr`, `govP`, `govD` and `potuser` so they also plot in the web interface.

**B2. Fast governor log frames.** Add parameter `GovLog` in `CAT_GOVERNOR` (`0=Off, 1=On`, default Off). When on and in `MOD_RUN`, send the frames below from `Ms10Task`, straight after `ProcessThrottle`, on `canInterface[Param::GetInt(Param::CanMapCan)]`. All values are little-endian and saturate instead of wrapping.

| ID | Period | Bytes | Signal | Type and scale |
| --- | --- | --- | --- | --- |
| `0x4A0` | 10 ms | 0–1 | Speed passed to the governor | uint16, 1 rpm |
|  |  | 2–3 | Speed error, `speedLimit` minus filtered speed | int16, 0.125 rpm |
|  |  | 4–5 | User throttle before limits | int16, 0.01 % |
|  |  | 6–7 | `potnom`, final value after all limits and ramp | int16, 0.01 % |
| `0x4A1` | 10 ms | 0–1 | Integral term `govInt` | int16, 0.01 % |
|  |  | 2–3 | P term | int16, 0.01 % |
|  |  | 4–5 | D term | int16, 0.01 % |
|  |  | 6 | Flags from B1 | uint8 |
|  |  | 7 | Rolling tick counter | uint8 |
| `0x4A2` | 100 ms | 0–1 | `revlim` | uint16, 1 rpm |
|  |  | 2–3 | `govKp` as entered | uint16, raw fixed point (value × 32) |
|  |  | 4–5 | `govKi` as entered | uint16, raw fixed point (value × 32) |
|  |  | 6–7 | `govKd` as entered | uint16, raw fixed point (value × 32) |
| `0x4A3` | 100 ms | 0 | `govImax` | uint8, 1 % |
|  |  | 1 | `govFlt` from B3 | uint8 |
|  |  | 2 | `opmode` | uint8 |
|  |  | 3 | `dir` | int8 |
|  |  | 4 | `TorqDerate` | uint8 |
|  |  | 5 | `regenmax` | int8, 1 % |
|  |  | 6 | `throtramp` | uint8 |

Governor output is not sent because it equals P term plus D term plus integral. The gains travel in the log itself, so every capture records the tuning it was made with. The rolling counter lets the analysis detect dropped frames.

The added traffic is about 220 frames per second, roughly 6 % of a 500 kbit/s bus.

**B3. Make the governor speed filter a parameter.** Add `govFlt` in `CAT_GOVERNOR`, range 0 to 6, default 4. Use it in place of the literal `4` in the speed `IIRFILTERF` call. Leave the slope filter's literal `4` alone. Default 4 keeps today's behaviour.

This also resolves a mismatch in the source. The comment above the filter says the time constant was cut to about 30 ms. The macro is `IIRFILTERF(l,n,c) = (n + l*((1<<c)-1)) / (1<<c)`, so `c = 4` is 1/16 per tick, about 160 ms, the same as the old integer filter. Correct the comment to match the code. Do not change the value. Ben decides it on the bench.

**B4. Parameter ID housekeeping.** At the reference commit the highest parameter ID in use is 161 and the highest spot value ID is 2125. The `Next param id` comment says 160, which is stale. Compute the next free IDs from the file, then update both comments.

**B5. Host tests.** Extend `test/test_throttle.cpp`:

1. Before B1, add a golden test: a scripted sequence of speed and throttle inputs with the recorded `finalSpnt` outputs.
2. After B1 to B3, the golden test must pass unchanged with `govFlt = 4`.
3. Add cases for integral reset below half the limit, regen clamp at `regenmax` above the limit, and integral freeze while the throttle is the binding limit.

### What Part B must not do

- Change the rate of `canMap->SendAll()`.
- Touch `0x11A` or `0x1D4` generation.
- Change governor logic, gains, or defaults, or add gain scheduling.

### First use on the tractor

Turn `GovLog` on with the tractor stationary and confirm the MG charger's DC-DC output stays up before doing field work. DC-DC dropouts on this machine have previously coincided with extra CAN traffic, and the cause was never confirmed.

## 9. Log capture and PC tooling

Tractor logs must be raw CAN frames with timestamps of 1 ms or better, taken on the inverter bus with `GovLog` on. Summarised or 1 Hz telematics data cannot be used for model fitting.

### Capture runs on the tractor

Save each run as its own file in SavvyCAN CSV or candump format, with a text note of implement, gear, PTO state and conditions.

| Run | What to do | What it gives |
| --- | --- | --- |
| R1 No-load acceleration | PTO out, neutral, `revlim` set high. Step the throttle to a fixed low value, about 20 %, and let speed rise to about 2000 rpm. Three repeats. | Inertia. |
| R2 Coast-down | From about 2000 rpm, release the throttle and let it run down. Set regen to zero for this run if practical. | Friction terms `Tc`, `b`, `c`. |
| R3 Implement empty | Repeat R1 and R2 with the PTO engaged and the implement running empty. | Added inertia and friction of the implement. |
| R4 Fast hit on the limit | Working `revlim`, no load, throttle snapped fully open. Three repeats. | Reference for overshoot and the dip-and-creep behaviour. |
| R5 Steady work | Five minutes of topping or mowing at governed speed. | Steady load profile. |
| R6 Baling | Five minutes including material starting and stopping. | Pulse load profile and governor wind-up reference. |
| R7 PTO engage and disengage | At governed speed, engage and disengage the PTO several times. | Step response reference. |

### Python tools

Python 3 with `pandas`, `numpy`, `scipy`, `matplotlib`, `cantools` and `python-can`, listed in `tools/requirements.txt`.

| Script | Input | Output |
| --- | --- | --- |
| `loglib.py` | SavvyCAN CSV or candump file, plus the DBC | A DataFrame of named signals on a 10 ms grid. Reports dropped ticks from the rolling counter. |
| `extract_params.py` | R1, R2, R3 logs | `params.json` with fitted `J`, `Tc`, `b`, `c`, and fit plots. |
| `make_profile.py` | R5, R6, R7 logs and `params.json` | Profile CSV files for the SD card. |
| `compare_runs.py` | A tractor log and a bench log of the same scenario | Overlay plots and a metrics table. |
| `run_script.py` | A YAML test script and the simulator serial port | Sends timed console commands, so bench tests are repeatable. |

### Fitting method

`extract_params.py` smooths speed with a zero-phase filter, differentiates it, and solves this by linear least squares:

```latex
T_m = J\,\frac{d\omega}{dt} + T_c + b\,\omega + c\,\omega^2
```

`make_profile.py` computes total load as motor torque minus inertial torque, then subtracts the base friction the simulator already adds, and clamps the result at zero:

```latex
T_{prof}(t) = \max\left(0,\ T_m - J\,\frac{d\omega}{dt} - T_c - b\,|\omega| - c\,\omega^2\right)
```

Two limits to state in the tool output:

- From speed and torque request alone, only the ratio of inertia to `kT` can be identified. That ratio is what sets loop behaviour, so the bench stays valid. Absolute values need reported torque from `0x1DA` or DC power from a shunt.
- A profile extracted this way is the load at the speed the tractor actually ran. Use the speed exponent `n` when testing gains that let speed move far from the logged value.

### Metrics

`compare_runs.py` computes the same numbers for tractor and bench logs:

- Overshoot above `revlim` in rpm, and time to settle within 1 % of `revlim`.
- Steady-state droop below `revlim` under load.
- Peak-to-peak rpm ripple per plunger stroke.
- Recovery time after a load step.
- Peak integral term, and time spent with the integral clamped.

### Tool self-test

Before any tractor data exists, generate a synthetic log from `lib/plant` with known parameters and confirm `extract_params.py` recovers them within 5 %.

## 10. Validation and tuning workflow

No gain is tuned on the bench until the bench reproduces what the tractor did with the gains the tractor had. The gains recorded in `0x4A2` and `0x4A3` of each tractor log make that comparison exact.

### Validation steps

1. **Open loop.** Set `revlim` high and repeat R1 on the bench with the fitted parameters. The bench speed trace should track the tractor's R1 trace within 5 % at every point.
2. **Fast hit on the limit.** Load the gains from the R4 log and repeat R4 on the bench. Overshoot should match within 20 % or 30 rpm, whichever is larger. The dip and creep should be present with similar duration.
3. **Baling.** Replay the R6 profile with the R6 gains. Peak-to-peak rpm ripple and the shape of the integral trace should match within 25 %.
4. **PTO step.** Replay R7 with the R7 gains. Recovery time should match within 25 %.

If a step fails, adjust in this order and record what changed: torque lag `tau`, inertia, speed noise, load speed exponent. If the bench still cannot reproduce hunting seen on the tractor, stop and raise the two-mass driveline model with Ben. Do not tune model parameters per scenario to force a match.

### Tuning procedure

Change one setting at a time and run every scenario for each change. Use `run_script.py` so each test is identical, and a marker number per run so logs can be matched to settings.

| Order | Setting | Start from | What to look for |
| --- | --- | --- | --- |
| 1 | `govFlt` | 4, then 3, then 2 | Less dip-and-creep in R4 without no-load hunting. |
| 2 | `govKp` | Tractor value | Lower ripple in R6 without oscillation in R4. |
| 3 | `govKd` | 0 | Damping of overshoot, watching for noise amplification with realistic speed noise. |
| 4 | `govKi` | Tractor value | Droop removed in R5 without wind-up between strokes in R6. |
| 5 | `govImax` | 100 | Keep high enough to carry the full steady load. Do not use it to hide wind-up. |

Score every run with the metrics in section 9. Pick two or three candidate settings, run them on the tractor with `GovLog` on, and compare tractor against bench again. Disagreement at this stage is the most useful input for improving the model.

## 11. Milestones and acceptance criteria

Nine milestones, each ending in a test Ben can run. M4 does not depend on M1 to M3 and can be built first if tractor logging is the priority. M7 needs M4 and M6.

| # | Milestone | Built by | Acceptance |
| --- | --- | --- | --- |
| M1 | Project skeleton, `leafcodec`, native tests | Claude | `pio test -e native` passes. Tests cover `0x1D4` decode for every raw value from −2048 to 2047, `0x1DA` speed encode checked against the Zombie's decode expression, and `nissan_crc` against a frame built by the Zombie's algorithm. |
| M2 | CAN bring-up on the board | Claude, Ben tests | SavvyCAN shows `0x1DA` at 10 ms ± 0.5 ms and `0x55A` at 100 ms. The Zombie web interface shows `udc` at the configured voltage, `speed` 0, and both temperatures at 40 °C. |
| M3 | Plant with base friction, closed loop | Claude, Ben tests | The Zombie completes precharge and reaches run mode. Throttle raises the Zombie's `speed`. Releasing it lets speed fall. The governor holds `revlim`. |
| M4 | Part B firmware on branch `gov-logging` | Claude, Ben tests | Golden test passes before and after. With `GovLog` off, no new frames appear. With it on, `0x4A0` and `0x4A1` appear at 10 ms and decode sensibly through the DBC. |
| M5 | Loads, presets, console, telemetry, events | Claude, Ben tests | `load step` and `load pulse` visibly disturb speed. `0x4B0` to `0x4B2` decode through the DBC. Jitter stays inside ± 0.5 ms with streaming on. |
| M6 | SD profiles and PC tools | Claude | Tool self-test recovers known parameters within 5 %. A generated profile replays from SD and matches its source torque trace in `0x4B0`. |
| M7 | Tractor capture, runs R1 to R7 | Ben | Seven sets of raw logs with `GovLog` on. `extract_params.py` and `make_profile.py` run on them without manual editing. |
| M8 | Validation, section 10 | Ben, Claude analyses | All four validation steps inside tolerance, or a written list of what does not match and why. |
| M9 | Tuning, section 10 | Ben, Claude analyses | A metrics table across settings and scenarios, and two or three candidate settings for tractor trials. |

### Later extensions, not to be built now

- Software-in-the-loop on the PC: link `lib/plant` and `lib/loads` with the Zombie's `throttle.cpp` in a host build, to sweep hundreds of gain sets in seconds before any bench run.
- Throttle output from an I2C DAC and driven digital inputs, for fully scripted driver input.
- Automated gain sweeps over CAN SDO.
- SavvyCAN connection through the simulator itself using the GVRET protocol.
- Two-mass driveline with shaft compliance and backlash.
- Road vehicle presets, and shunt or BMS simulation.

## 12. Open items and risks

Seven assumptions need checking and three risks need watching. None blocks M1 to M6.

### To verify

| # | Assumption | Why it matters | How to settle it |
| --- | --- | --- | --- |
| V1 | Torque request scale `kT` is 0.25 Nm per bit. | Sets where the motor envelope clips, which changes effective governor gain. | Compare request against reported torque in a tractor log, or against DC power from a shunt. |
| V2 | Tractor motor limits are 280 Nm and 80 kW. | Same as V1. | Ben confirms which Leaf motor and inverter generation is fitted. |
| V3 | Reported torque in `0x1DA` is 11-bit signed at 0.5 Nm in bytes 2 and 3. | Needed to fit torque lag and to separate inertia from `kT`. | Decode a tractor log and check it tracks the request. |
| V4 | T-CAN485 CAN pins are TX 26 and RX 27. | Published pinouts disagree between board revisions. | Copy from Ben's working telematics firmware. |
| V5 | Governor speed filter is meant to be shift 4. | The source comment says about 30 ms, the code gives about 160 ms. | Ben decides after bench runs with `govFlt` at 4, 3 and 2. |
| V6 | The tractor can record raw 10 ms CAN frames. | Summarised telematics data cannot fit the model. | Ben confirms the logger, or uses a USB-CAN adapter with SavvyCAN in the cab. |
| V7 | The bench Zombie reaches run mode with ignition, start, direction and brake inputs only. | Start conditions depend on the selected vehicle type. | Read the `Ready()` and `Start()` logic of the vehicle class in the tractor's parameter file. |

### Risks

| Risk | Effect | Response |
| --- | --- | --- |
| A single inertia cannot reproduce driveline oscillation. | Bench looks more stable than the tractor. | Caught by validation step 2 or 3. Escalate to the two-mass model. |
| The real inverter has its own torque ramping and derating that the model lacks. | Bench results near the torque or power limit are optimistic. | Fit `tau` from logs. Treat results where the clipping flag is set as indicative only. |
| Gains that look best on the bench are tuned to model error. | Poor behaviour back on the tractor. | Always finish with tractor trials of two or three candidates and compare again. |
