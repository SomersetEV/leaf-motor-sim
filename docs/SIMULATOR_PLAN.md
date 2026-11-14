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

**Out of scope for phase 1.** These are later extensions and must not be built yet: throttle DAC output, driving the Zombie digital inputs, BMS, shunt or charger simulation, and a two-mass driveline model. Gain control over CAN SDO was moved into phase 1 as M5a by Ben on 3 Nov 2026.

## 2. Working rules for Claude in VS Code

Build in the milestone order of section 11 and stop at each milestone for a hardware test by Ben. Nothing here can be verified on real hardware from the editor.

**Source of truth.** Facts about the Zombie firmware in this plan were read from `SomersetEV/Stm32-vcu`, branch `Tractor-RPM-limiting-testing`, commit `d85e756` (15 Sep 2026). Re-check each one against the working tree before relying on it. Where the tree differs, the tree wins and the difference gets reported.

Re-checked 10 Nov 2026 (item I1) against `Tractor-RPM-limiting-testing` at `a05e984` (8 Nov), which now carries the governor logging commits from `gov-logging`, and `MGChgLimRaw` moved to ID 164 in `28d53d0`. The Leaf codecs (`leafinv.cpp`, `NissLeafMng.cpp`) are unchanged since `d85e756`. `throttle.cpp` matches `gov-logging`, apart from formatting. The next free IDs are 165 (parameters) and 2130 (values). The host tests pass on the tractor branch.

**Repositories.**

| Work | Location |
| --- | --- |
| Simulator firmware and PC tools | New repo `leaf-motor-sim`, PlatformIO project |
| Zombie logging changes | Branch `gov-logging` off `Tractor-RPM-limiting-testing` in `SomersetEV/Stm32-vcu`, since carried into `Tractor-RPM-limiting-testing` itself (`7ea0fa6`, `a05e984`) |
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

Two nodes share one 500 kbit/s CAN bus: the real Zombie and the LilyGo simulator, which also logs the bus (`canlog`). There is no high voltage anywhere on the bench.

| Node | Real or simulated | Connections |
| --- | --- | --- |
| ZombieVerter VCU | Real hardware, real firmware (device under test) | 12 V bench supply. Inverter CAN to the bench bus. |
| LilyGo T-CAN485 | Simulated Leaf inverter, motor and load | USB to PC for power and serial. CAN H/L to the bench bus. |
| USB-CAN adapter with SavvyCAN | Not used since 3 Nov 2026 | The Due with GVRET sent error frames even in listen-only mode and dropped Zombie frames. Keep it unpowered. The simulator's `canlog` logs the bus instead (section 7). |
| Throttle pot | Real | Zombie throttle 1 input, as on the tractor. |
| Ignition, start, direction, brake switches | Real | Zombie digital inputs at 12 V. |
| Contactor and precharge outputs | Not connected | Optional indicator lamps. |

The bus needs exactly two 120 Ω terminations. Check which nodes already carry one before adding any.

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

These values are from LilyGO's own example `config.h` for the board ([source](https://raw.githubusercontent.com/Xinyuan-LilyGO/T-CAN485/main/example/Arduino/CAN/config.h)), except the CAN pins, which come from Ben's working telematics firmware (V4, settled). LilyGO's example has them swapped.

| Signal | GPIO | Note |
| --- | --- | --- |
| `PIN_5V_EN` | 16 | Drive high to power the transceiver supply. |
| `CAN_TX_PIN` | 27 | From Ben's telematics firmware, proven on his hardware. |
| `CAN_RX_PIN` | 26 | Same. |
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
    leafcodec/        decode 0x1D4, encode 0x1DA and 0x55A, Nissan CRC
    plant/            motor torque envelope, torque lag, inertia integration
    loads/            load components, profile storage and CSV parser
    sensor/           reported speed: delay and noise (items I6, I7)
    simcore/          0x1D4 latch, tick statistics, parameter table, presets, flat JSON
    simproto/         telemetry frame packing, console and command parsing, Zombie SDO
  src/                hardware-facing code
    main.cpp          setup, task creation
    can_io.cpp        TWAI init, receive task, transmit helper, bus-off recovery, canlog
    sim_task.cpp      the 10 ms loop
    console.cpp       serial commands, CSV streaming, zget and zset
    sd_files.cpp      profile and preset files from SD into RAM
    params_store.cpp  parameters in NVS
    status_led.cpp
  test/
    test_leafcodec/  test_plant/  test_loads/  test_sensor/  test_simcore/  test_simproto/
  tools/              PC-side Python, see section 9
    dbc/leaf_sim.dbc
    scripts/          bench scripts and sweep YAML
  docs/bench/         bench procedure per milestone; results/ holds sweep tables
  sd_card/            example SD card contents
  .github/workflows/  CI: native tests, tool self-test, firmware build (item I3)
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
b[2] = (tq >> 8) & 0x07;  b[3] = tq & 0xFF; // reported torque, see below
int16_t raw = (int16_t)lroundf(rpm * 2.0f);  // 0.5 rpm per bit, signed, clamp to +/-16383 rpm
b[4] = (uint8_t)(raw >> 8);
b[5] = (uint8_t)(raw & 0xFF);
b[6] = inverter_error ? 0x10 : 0x00;         // Zombie: error = (b[6] & 0xB0) != 0
b[7] = 0;
```

The Zombie decodes speed as `(b[4] << 7) | (b[5] >> 1)` and subtracts `0x7FFF` when the result exceeds `0x3FFF`. It therefore sees whole rpm, and reverse speeds read 1 rpm off. Encode as the real inverter does and do not compensate.

The Zombie does not check a counter or CRC on `0x1DA`, and has no receive timeout for it.

A real inverter also reports its actual torque in `0x1DA`, 11-bit signed at 0.5 Nm per bit in `((b[2] & 0x07) << 8) | b[3]`. Tractor logs of 7 Nov 2026 confirm the layout (V3, settled), so the simulator fills it with the model's motor torque after the envelope and lag. The `SIM_REPORT_TORQUE` config flag, default on, can turn it off. The Zombie does not read these bits. The real inverter also sets bits 3 and 4 of `b[2]` while running; the simulator leaves them clear.

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
| `kT` | Torque per request bit | 0.25 Nm | Settled from tractor logs (V1) |
| `udc` | Reported bus voltage | 360 V | Tractor pack nominal voltage |
| `noise` | Speed noise, standard deviation | 0 rpm | Tractor logs, `tools/fit_noise.py`: 11 rpm |
| `noise_fc` | Speed noise band centre; 0 is white per tick | 0 Hz | Same fit: 6 Hz (item I6) |
| `noise_bw` | Speed noise bandwidth, −3 dB | 4 Hz | Same fit: 4 Hz |
| `rptdelay` | Extra delay on the speed in `0x1DA`, 0–50 ms, to the nearest 1 ms sub-step | 0 ms | Delay-margin tests (item I7) |
| `motor_c`, `inv_c` | Reported temperatures in `0x55A` | 40 °C | Not needed |

The speed noise and its seed, and the load noise, restart on every `preset` and `mark`, so a script repeats exactly (item I5).

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
| `can_rx_task` | 0 | High | Blocks on `twai_receive`, decodes frames, updates a shared latest-command struct with a timestamp. |
| `console_task` | 0 | Low | Serial commands, CSV streaming, SD file loading. |

The UART and TWAI drivers are installed from a short init task on core 0, so their interrupts run on core 0 too. The M5 bench showed why: on each bus error, the TWAI ISR's errata workaround resets the controller while holding the driver lock. A task on core 1 waiting for that lock held off the tick by about 0.6 ms.

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
| `load noise <sd_Nm> [corner_hz]` | Low-pass filtered random load torque. |
| `load off` | Remove step, pulse, gradient and noise loads. |
| `presets` | List built-in presets and those on SD. |
| `profile load <file>` | Read a profile into RAM. |
| `profile start`, `profile stop`, `profile loop <0\|1>` | Control replay. |
| `profile exp <n> <w_ref_rpm>`, `profile list` | Scale profile torque with speed; list profiles on SD. |
| `mark <n>` | Send a marker event in `0x4B2`, and restart both noise sources. |
| `zget <name>`, `zset <name> <value>` | Read or write a whitelisted Zombie governor parameter over SDO (M5a). |
| `fault inv <0\|1>` | Set or clear the inverter error bit in `0x1DA`. |
| `stream <0\|1> [divider]` | CSV streaming on or off, optionally every Nth tick. |
| `canlog <0\|1> [all]` | Bus log on or off: received and transmitted frames as candump `-l` lines, `(s.us) can0 ID#DATA`. By default it logs only the IDs in `CANLOG_IDS`; `all` logs every frame. A full queue drops frames and counts them in `stat` as `canlog_dropped`. |

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

Runs D1–D3 were added on 14 Nov 2026, while the tractor can only be run on the road. They measure the loop at three gear inertias, which brackets the PTO case. The reasoning and harness predictions are in `docs/tractor/road_runs_plan.md`, and the field sheet is `docs/tractor/M7_road_D1_D3_field_sheet.pdf`. 1800 motor rpm is 540 rpm at the PTO.

| Run | What to do | What it gives |
| --- | --- | --- |
| R1 No-load acceleration | PTO out, neutral, `revlim` set high. Hold the throttle at about 2.5, 3 and 3.5 % in turn (one each), and let speed settle by itself for about 5 s. The neutral shaft is light (J 0.055 kg·m², 13 Nov 2026), so 20 % would run away. | Inertia, and friction at three speeds. |
| R2 Coast-down | From about 2000 rpm, release the throttle and let it run down. Set regen to zero for this run if practical. | Friction terms `Tc`, `b`, `c`. |
| R3 Implement empty | Repeat R1 and R2 with the PTO engaged and the implement running empty. | Added inertia and friction of the implement. |
| R4 Fast hit on the limit | Working `revlim`, no load, throttle stepped quickly to about 6 % and held 5 s. Release at once above `revlim` + 500. Never full throttle in neutral: 18 % overshot to 3460 rpm on 13 Nov 2026. Three repeats. | Reference for overshoot, hunting and the dip-and-creep behaviour. |
| R5 Steady work | Five minutes of topping or mowing at governed speed. | Steady load profile. |
| R6 Baling | Five minutes including material starting and stopping. | Pulse load profile and governor wind-up reference. |
| R7 PTO engage and disengage | At governed speed, engage and disengage the PTO several times. | Step response reference. |
| D1 Road acceleration and coast | In gear on a flat road (gears A, road, and B, about 10 km/h at 1800 rpm): a fixed pedal from about 500 to 1800 rpm, then coast in gear, `regenmax` 0 if the road allows. Twice each way. | Inertia per gear, rolling and driveline drag. |
| D2 Road hit on the limit | In gear on the flat: step to the limit and hold 10 s. Gear A at full pedal, B at about 25 %, C (6–8 km/h) at about 12 %. Release above 2700 rpm or on a hard surge. Three each. | Overshoot and hunting at three inertias, either side of the stability boundary at today's gains. |
| D3 Road gradient at the limit | Gear A, full pedal, 20 s up and 20 s down a steady hill. Twice each way. | Droop and regen at the limit; gradient load profile for the road sweep. |

### Python tools

Python 3 with `pandas`, `numpy`, `scipy`, `matplotlib`, `cantools` and `python-can`, listed in `tools/requirements.txt`.

| Script | Input | Output |
| --- | --- | --- |
| `loglib.py` | SavvyCAN CSV or candump file, plus the DBC | A DataFrame of named signals on a 10 ms grid. Reports dropped ticks from the rolling counter. |
| `extract_params.py` | R1, R2, R3 logs | `params.json` with fitted `J`, `Tc`, `b`, `c`, and fit plots. |
| `make_profile.py` | R5, R6, R7 logs and `params.json` | Profile CSV files for the SD card. |
| `compare_runs.py` | A tractor log and a bench log of the same scenario | Overlay plots and a metrics table. |
| `run_script.py` | A YAML test script and the simulator serial port | Sends timed console commands, so bench tests are repeatable. |
| `sweep.py` | A sweep YAML and the simulator serial port | Runs Zombie gain sets over SDO against a load and scores each run (M5a, items I4, I5, I8). |
| `fit_noise.py` | Tractor logs with `0x1DA` | The band-limited speed noise that matches the tractor through the governor filter (item I6). |

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
2. **Fast hit on the limit.** Load the gains from the R4 log and repeat R4 on the bench. Overshoot should match within 20 % or 30 rpm, whichever is larger. The dip and creep should be present with similar duration. Do the same for each gear of D2 (road), at that gear's fitted inertia, with the same tolerances; the gear that hunts should hunt at a similar period.
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

Each milestone ends in a test Ben can run. M4 does not depend on M1 to M3 and can be built first if tractor logging is the priority. M7 needs M4 and M6. M5b to M5d were added on 10 Nov 2026 and need no tractor data. Phase 2 milestones M10 to M13 are in section 14, and Phase 3 milestones M14 to M17 in section 17.

| # | Milestone | Built by | Acceptance |
| --- | --- | --- | --- |
| M1 | Project skeleton, `leafcodec`, native tests | Claude | `pio test -e native` passes. Tests cover `0x1D4` decode for every raw value from −2048 to 2047, `0x1DA` speed encode checked against the Zombie's decode expression, and `nissan_crc` against a frame built by the Zombie's algorithm. |
| M2 | CAN bring-up on the board | Claude, Ben tests | SavvyCAN shows `0x1DA` at 10 ms ± 0.5 ms and `0x55A` at 100 ms. The Zombie web interface shows `udc` at the configured voltage, `speed` 0, and both temperatures at 40 °C. |
| M3 | Plant with base friction, closed loop | Claude, Ben tests | The Zombie completes precharge and reaches run mode. Throttle raises the Zombie's `speed`. Releasing it lets speed fall. The governor holds `revlim`. |
| M4 | Part B firmware on branch `gov-logging` | Claude, Ben tests | Golden test passes before and after. With `GovLog` off, no new frames appear. With it on, `0x4A0` and `0x4A1` appear at 10 ms and decode sensibly through the DBC. |
| M5 | Loads, presets, console, telemetry, events | Claude, Ben tests | `load step` and `load pulse` visibly disturb speed. `0x4B0` to `0x4B2` decode through the DBC. Jitter stays inside ± 0.5 ms with streaming on. |
| M5a | Zombie governor gains over CAN SDO, scripted sweeps | Claude, Ben tests | `zset` and `zget` change and read `govKp`, `govKi`, `govKd`, `govImax`, `govFlt` and `revlim` on the bench Zombie, and the web interface agrees. Each write appears as a `0x4B2` event. A sweep script runs a list of gain sets against a preset or profile unattended and writes a metrics table. `canlog` captures the bench bus with no dropped frames, and `loglib` decodes it. |
| M5b | Review improvements, section 15 | Claude, Ben answers the ASK items | Every row of the section 15 table meets its "Done when". |
| M5c | Loop calculator and software-in-the-loop, section 16 | Claude | The Zombie golden test passes through the harness. Three bench runs reproduce: ring period within 10 %, stroke ripple within 15 %. |
| M5d | Gain search, section 16 | Claude, Ben tests | A ranked table from the search, and its top three confirmed on the bench in the same order. |
| M6 | SD profiles and PC tools | Claude | Tool self-test recovers known parameters within 5 %. A generated profile replays from SD and matches its source torque trace in `0x4B0`. |
| M7 | Tractor capture, runs R1 to R7 | Ben | Seven sets of raw logs with `GovLog` on. `extract_params.py` and `make_profile.py` run on them without manual editing. |
| M8 | Validation, section 10 | Ben, Claude analyses | All four validation steps inside tolerance, or a written list of what does not match and why. |
| M9 | Tuning, section 10 | Ben, Claude analyses | A metrics table across settings and scenarios, and two or three candidate settings for tractor trials. |

### M5a: gain control over CAN SDO

The Zombie's SDO server is node 3 on CAN1 (`sdo.SetNodeId(3)` in `stm32_vcu.cpp`). Requests go out as `0x603` and replies come back as `0x583`. A parameter is addressed by its unique ID: index `0x2100 | (id >> 8)`, sub-index `id & 0xFF`. The value is fixed point, ×32. `Param::Set` range-checks the value and calls `Param::Change`, so the new value takes effect at once.

| Parameter | ID |
| --- | --- |
| `revlim` | 15 |
| `govKp` | 157 |
| `govKi` | 158 |
| `govImax` | 159 |
| `govKd` | 160 |
| `govFlt` | 162 |

- **Console:** `zset <name> <value>` and `zget <name>`. The console task sends the request and waits up to 100 ms for the reply, outside the tick. The CAN receive task hands `0x583` replies to the console.
- **Whitelist:** only the six parameters above, plus `GovLog` (163) so a sweep can turn on the governor frames, can be written. The simulator never sends the SDO save command (index `0x5002`), so a Zombie power cycle restores its saved setup.
- **Events:** each accepted write emits a `0x4B2` event (`ZParam`, value = parameter ID), and the console logs the old and new values.
- **Sweeps:** `tools/sweep.py` reads a list of gain sets and runs each one: `zset` the gains, run a preset or SD profile for a fixed time, and score the stream with `tools/metrics.py`. It writes one metrics row per gain set. For baler tuning it uses profiles made from Ben's baler logs with `make_profile.py` (M6).
- **Pedal:** a sweep needs the pedal held at a fixed position for its whole length, so Ben needs a clamp or a fixed pot. A throttle DAC stays out of scope.
- **The Zombie side is unchanged.** SDO parameter access is standard libopeninv.
- **Bench capture comes from the simulator, not SavvyCAN** (Ben, 3 Nov 2026). The Due running GVRET sent error frames even in listen-only mode, and its capture dropped Zombie frames. `canlog` (section 7) logs every frame the simulator sees, and `run_script.py --canlog-out` saves it in candump format, which `loglib` reads like any other log. Sweeps record the bus log and the CSV stream together. Keep the Due unpowered during runs that matter.

### Later extensions, not to be built now

- Software-in-the-loop on the PC: moved into the plan as M5c on 10 Nov 2026, see section 16.
- Throttle output from an I2C DAC and driven digital inputs, for fully scripted driver input.
- SavvyCAN connection through the simulator itself using the GVRET protocol.
- Two-mass driveline with shaft compliance and backlash.
- Road vehicle presets, and shunt or BMS simulation.

## 12. Open items and risks

Nine assumptions needed checking and three risks need watching. None blocks M1 to M6. V1, V3 and V4 are settled.

### To verify

| # | Assumption | Why it matters | How to settle it |
| --- | --- | --- | --- |
| V1 | Torque request scale `kT` is 0.25 Nm per bit. | Sets where the motor envelope clips, which changes effective governor gain. | **Settled 8 Nov 2026.** In two tractor logs (telematics `session_0011` and `0012`, 7 Nov), reported torque is 0.249–0.250 Nm per request bit. With `kT` 0.25, mechanical power is 84–86 % of the shunt's DC power when motoring, and DC power 78–82 % of mechanical on regen. |
| V2 | Tractor motor limits are 280 Nm and 80 kW. | Same as V1. | Ben confirms which Leaf motor and inverter generation is fitted. |
| V3 | Reported torque in `0x1DA` is 11-bit signed at 0.5 Nm in bytes 2 and 3. | Needed to fit torque lag and to separate inertia from `kT`. | **Settled 8 Nov 2026.** In the same logs it tracks the request with r = 0.998, best aligned at 20–30 ms lag, consistent with `tau` 30 ms. The simulator now reports it (`SIM_REPORT_TORQUE`, default on). |
| V4 | T-CAN485 CAN pins are TX 26 and RX 27. | Published pinouts disagree between board revisions. | **Settled by Ben.** His working telematics firmware uses TX 27 and RX 26, the other way round from LilyGO's example, and `include/pins.h` follows it. |
| V5 | Governor speed filter is meant to be shift 4. | The source comment says about 30 ms, the code gives about 160 ms. | Ben decides after bench runs with `govFlt` at 4, 3 and 2. |
| V6 | The tractor can record raw 10 ms CAN frames. | Summarised telematics data cannot fit the model. | The telematics logger kept only about 15 % of frames in run mode (7 Nov logs). Ben has fixed it (10 Nov 2026, item I9) and will take new logs. Check them for gaps in the `0x4A1` counter. |
| V7 | The bench Zombie reaches run mode with ignition, start, direction and brake inputs only. | Start conditions depend on the selected vehicle type. | Read the `Ready()` and `Start()` logic of the vehicle class in the tractor's parameter file. |
| V8 | The tractor's `regenramp` is 100 %/10 ms (no regen ramp), as the harness assumes. | It sets how fast the governor can brake an overspeed. With the default of 1, the M4 overshoot is wrong by 190 rpm. | **Decided 11 Nov 2026:** Ben will set at least 5. The M4 bench log shows the bench Zombie at 13 or more (M5c finding 2). In the harness, values of 3 and above all behave the same, so the harness keeps 100. |
| V9 | The inverter delivers the requested torque after a 30 ms lag, as the simulator models it. | Sets overshoot and hunting at no load. With the inverter's shortfall, the tractor's R4 was much milder than the harness's (18 % pedal: 3460 rpm against 4164). | **Open, found 13 Nov 2026** (`docs/tractor/R1R2R4_2026-11-13.md`, section 4). In fast no-load transients the tractor delivered and reported 40–70 % of the request, and about 0 Nm on small regen requests, always opposing the change in speed. At low steady torque it matches. The cause (inverter damping or rate limit) needs a continuous log of R4. |

### Risks

| Risk | Effect | Response |
| --- | --- | --- |
| A single inertia cannot reproduce driveline oscillation. | Bench looks more stable than the tractor. | Caught by validation step 2 or 3. Escalate to the two-mass model. |
| The bench loop delay depends on the phase between the Zombie's and the simulator's clocks, which drifts by about 1.7 ms a minute (M5c). | The same settings give different results at three and four ticks: 238 and 268 rpm of stroke ripple on 3 Nov. | Every sweep row records `loop_ticks`. Compare runs with the same value, and use `rptdelay 10` in the harness for four. |
| The real inverter has its own torque ramping and derating that the model lacks. | Bench results near the torque or power limit are optimistic. | Fit `tau` from logs. Treat results where the clipping flag is set as indicative only. |
| Gains that look best on the bench are tuned to model error. | Poor behaviour back on the tractor. | Always finish with tractor trials of two or three candidates and compare again. |

## 13. Phase 2: speed-mode throttle firmware

Phase 2 starts once M9 is signed off and the Leaf sim is functional. It adds a second throttle mode: pedal % sets an rpm setpoint from 0 to `spdmax`, and a PI loop on measured speed produces the torque request. The aim is a hydrostatic-style feel for the tractor and telehandler work: lift off and the machine slows to the commanded speed, and it holds speed under PTO load or downhill with a trailer.

Torque mode stays the default and its code path is untouched. Speed mode is selected by a parameter and can only be entered at standstill. Build it on a new branch `speed-mode` off the Phase 1 governor branch.

### Signal chain

Each 10 ms step in `ProcessThrottle`, when `throtmode` = 1:

1. Pedal % from the existing pot calibration and deadband, unchanged.
2. Setpoint: `rpmTarget` = pedal % × `spdmax` (`spdmaxrev` in reverse), clamped to `revlim`.
3. Setpoint ramp: `rpmSp` moves towards `rpmTarget` at no more than `spdrampup` or `spdrampdn` rpm/s. This does most of the smoothing; the PI should not be tuned to do it.
4. Speed filter: the same IIR as the governor, coefficient `spdflt`.
5. PI plus feed-forward: output = `spdkp` × error + integral + `spdff` × `rpmSp`, in torque %.
6. Output clamp: between the speed-tapered regen limit (`regenmax`, faded below `regenrpm` to zero at `regenendrpm`, as today) and `throtmax`.
7. Existing chain, unchanged: Udc, Idc and temperature derates, `SpeedLimitCommand` as the hard backstop, then `RampThrottle`.
8. Anti-windup by back-calculation: the integrator is corrected by the difference between the PI output and the final `potnom` after step 7, so a derate cannot wind it up.

Reuse the Phase 1 governor PID code with its own gains if it fits, or libopeninv's `PiController`. **VERIFY** which is cleaner once the Phase 1 code is final.

### Parameters

New category `CAT_SPEEDMODE`. Defaults are placeholders for the first bench run, not tuned values. Assign ids per B4.

| Name | Unit | Range | Placeholder | Purpose |
| --- | --- | --- | --- | --- |
| `throtmode` | — | 0–1 | 0 | 0 = Torque, 1 = Speed |
| `spdmax` | rpm | 0–10000 | 3000 | Setpoint at 100 % pedal, forward |
| `spdmaxrev` | rpm | 0–10000 | 1000 | Setpoint at 100 % pedal, reverse |
| `spdrampup` | rpm/s | 10–10000 | 1500 | Setpoint rise rate |
| `spdrampdn` | rpm/s | 10–10000 | 2000 | Setpoint fall rate |
| `spdkp` | %/100 rpm | 0–50 | 2 | Proportional gain |
| `spdki` | %/(100 rpm·s) | 0–50 | 1 | Integral gain |
| `spdff` | %/1000 rpm | 0–20 | 0 | Feed-forward on setpoint |
| `spdflt` | — | 0–8 | 4 | Speed filter coefficient |
| `spdzero` | rpm | 0–500 | 150 | Below this setpoint with pedal at 0, fade to coast or normal regen |
| `spdzerofade` | ms | 0–2000 | 300 | Fade time for `spdzero` |

### Zero speed, exits and mode changes

- **Near zero.** With pedal at 0 and `rpmSp` below `spdzero`, the PI output fades to the normal torque-mode off-pedal behaviour over `spdzerofade`. The loop never fights to hold 0 rpm, which would chatter through driveline backlash.
- **Brake.** Integrator reset, output hands to the existing brake regen path.
- **Pedal fault, stale `0x1DA` (no frame for 50 ms), direction change, neutral, or leaving `MOD_RUN`.** Integrator reset and zero torque within one 10 ms step.
- **Mode change.** `throtmode` changes only take effect when speed is below 50 rpm and pedal is at 0. Otherwise the change is held and a flag is set.

### Logging

With `GovLog` on and `throtmode` = 1, add two frames to the Part B set. Both are inside the free `0x4A0`–`0x4BF` range; add them to `leaf_sim.dbc`.

| ID | Period | Bytes | Signal | Type and scale |
| --- | --- | --- | --- | --- |
| `0x4A4` | 10 ms | 0–1 | `rpmTarget` | uint16, 1 rpm |
| | | 2–3 | `rpmSp` after ramp | uint16, 1 rpm |
| | | 4–5 | Integral term | int16, 0.01 % |
| | | 6–7 | PI + feed-forward output before derates | int16, 0.01 % |
| `0x4A5` | 100 ms | 0–1 | `spdkp` as entered | uint16, value × 32 |
| | | 2–3 | `spdki` as entered | uint16, value × 32 |
| | | 4–5 | `spdff` as entered | uint16, value × 32 |
| | | 6 | `throtmode` | uint8 |
| | | 7 | Speed-mode flags: zero fade, exit reason, windup active, mode change held | uint8 bitfield |

### Host tests

Extend `test/test_throttle.cpp`:

1. Torque-mode golden test passes unchanged with `throtmode` = 0.
2. Setpoint ramp: a pedal step gives a setpoint that rises at exactly `spdrampup`.
3. Clamp: output never exceeds `throtmax` or goes below the tapered regen limit.
4. Anti-windup: hold a derate at 20 % for 5 s with a large error, release it, and the integrator is no larger than the clamp allows.
5. Exits: each exit condition resets the integrator and gives zero torque in one step.
6. Mode change refused above 50 rpm, accepted at standstill with no output step.

## 14. Phase 2: speed-mode bench tests

Speed mode is proven on the bench in ten scripted tests before it goes near the tractor. The rig, plant model and load presets from Phase 1 carry over. Four simulator additions are needed first, because speed mode depends on driver input and driveline behaviour that Phase 1 could leave out.

### Simulator additions

- **Scripted pedal.** Promote the I2C DAC from Later extensions to required (two MCP4725 channels for dual-pot checking). Console commands `pedal <pct>`, `pedal ramp <from> <to> <ms>` and `pedal fault`, which drives the second channel out of range. Brake becomes a driven digital input with `brake on|off`.
- **Backlash.** Promote the two-mass driveline from Later extensions, with a dead angle `backlash` referred to the motor shaft (placeholder 3°, from a slow-reversal log on the tractor). Without it, test S6 means nothing.
- **Report delay.** Parameter `rptdelay`, 0–50 ms, holds back each `0x1DA` speed value by that much, to test robustness to CAN and inverter latency.
- **Overrun load.** Allow a negative constant load torque, so the load drives the motor as a trailer does downhill.

Add frame `0x4B3` at 10 ms with commanded pedal % (uint16, 0.01 %), brake state, `rptdelay` and the backlash state (in contact or in the gap), so every speed-mode run is fully described by its log.

### Test cases

Pass criteria are starting targets for Ben to adjust after the first runs. Each test runs from a `run_script.py` script, with `GovLog` on.

| ID | Test | Script | Pass criteria |
| --- | --- | --- | --- |
| S1 | Setpoint step, no load | `noload`, pedal 20 % → 30 % | Overshoot ≤ 3 % of the step; within ± 25 rpm of setpoint in 1.5 s after the ramp ends |
| S2 | Pedal ramp tracking | `noload`, pedal 0 → 80 % over 5 s | Tracking error ≤ 100 rpm during the ramp, no oscillation after |
| S3 | Load step at steady pedal | `pto_step` at 50 % pedal | Sag ≤ 150 rpm; back within ± 25 rpm in 1 s; no more than 2 cycles of oscillation |
| S4 | Overrun | negative load, 40 % pedal | Holds setpoint within ± 50 rpm on regen; regen never beyond the tapered limit |
| S5 | Derate windup | sim raises motor temperature in `0x55A` until `tmpmmax` derates for 5 s, then drops it | Overshoot after release ≤ 5 % of setpoint; windup flag set during the derate |
| S6 | Zero crossing and backlash | lift off from 1500 rpm to 0; then pedal 2 % ↔ 4 % at 1 Hz | Torque changes sign at most once per lift-off; fades to coast below `spdzero`; no chatter at low setpoint |
| S7 | Safety exits | brake on; `pedal fault`; sim stops `0x1DA`; direction change | Zero torque or brake regen path within 20 ms; integrator reset in `0x4A4`; exit reason in `0x4A5` |
| S8 | Latency margin | repeat S1 and S3 at `rptdelay` 20 ms and 40 ms | Stable at 20 ms; behaviour at 40 ms recorded as the margin |
| S9 | Mode change | change `throtmode` at 1000 rpm, then at standstill | Refused and flagged while moving; no torque step when accepted |
| S10 | Baler pulse comparison | `baler` preset, same pedal, torque mode then speed mode | Speed ripple in speed mode lower than torque mode with the governor, at equal or lower peak torque |

Extend `compare_runs.py` with overshoot, settling time, sag, recovery time, integrated absolute error and torque sign changes per run, so every test produces the same metrics row.

### Tuning order

1. Set `spdrampup` and `spdrampdn` for feel first, with conservative gains, on S1 and S2.
2. Raise `spdkp` on S1 until overshoot appears, then back off by a third.
3. Raise `spdki` on S3 until sag recovery meets the target without extra cycles.
4. Add `spdff` only if S2 tracking still lags.
5. Check every candidate against S5 to S8 before calling it a candidate.

### Milestones

| # | Milestone | Built by | Acceptance |
| --- | --- | --- | --- |
| M10 | Speed mode on branch `speed-mode` | Claude, Ben tests | Host tests in section 13 pass; torque-mode golden test unchanged; `0x4A4` and `0x4A5` decode through the DBC |
| M11 | Simulator additions | Claude, Ben tests | Pedal, brake, backlash, `rptdelay` and overrun all work from the console and show in `0x4B3` |
| M12 | Test suite S1–S10 scripted | Claude | All ten run unattended from `run_script.py` and produce a metrics table |
| M13 | Tuned candidates and first tractor trial | Ben, Claude analyses | Two or three settings pass S1–S10; first yard trial at low `spdmax` only after S7 passes |

## 15. Review of 10 Nov 2026: improvements

A review of the repo on 10 Nov 2026 found eleven things worth fixing before more tuning work. They form milestone M5b. None needs tractor data.

Section 2 still applies. Work down the table and stop at the end of the milestone for Ben. Items marked **ASK** need Ben's answer before any work on them.

| # | Item | What to do | Done when |
| --- | --- | --- | --- |
| I1 | Documents out of date | Re-check every Zombie fact in `CLAUDE.md` against the Stm32-vcu working tree. On 10 Nov the local `Tractor-RPM-limiting-testing` carried the governor logging commits, and `28d53d0` (8 Nov) had moved `MGChgLimRaw` to ID 164. If that holds, the ID 162 clash is gone and the next free parameter ID is 165. Also fix the two test counts in `CLAUDE.md` (64 and 66, 21 and 22), the README status table and console list, and this plan: section 3 (SavvyCAN node), section 4 (CAN pins, file layout) and V4 in section 12. | No statement in the three files contradicts the two trees. |
| I2 | Untracked data | **ASK** Ben. Proposal: commit `docs/tractor/`, keep `logs/` out of git with `.gitignore`, and commit each sweep's `metrics.csv` and YAML under `docs/bench/results/<date>_<name>/`. | `git status` is clean, and each number in `docs/bench/M5a_*` traces to a file. |
| I3 | Continuous integration | GitHub Actions on push: `pio test -e native`, `python tools/selftest.py` with Linux g++, and `pio run -e tcan485`. | A green run on `main`. |
| I4 | Tests for `sweep.py` | Both sweep bugs so far were in this file, and it has no test. Add a fake console to `selftest.py`, as `run_script` has. Cover canlog lines mixed with replies, a late `ok` after a gradient ramp, the integral clear, and gains restored after an error. | New checks pass in `selftest.py`. |
| I5 | Repeatable sweeps | The same settings vary by about ±15 rpm between runs. Reset both noise seeds and the pulse phase on `preset` and on `mark`. Add `repeats: N` to the sweep YAML, and report the mean and standard deviation of each metric. | One set run three times differs by under 5 rpm in `stroke_p2p_rpm`. |
| I6 | Coloured speed noise | The simulator's speed noise is white per tick. The tractor's is a ripple at about 6–10 Hz, so `sweep_road.yaml` uses a different noise level for each `govFlt`. Add a parameter `noise_hz` (0 keeps white noise) for band-limited noise centred on that frequency, in `lib/` and mirrored in the PC tools. Fit the level and band from `logs/session_0011.csv` and `session_0012.csv`. Then remove the per-`govFlt` levels from `sweep_road.yaml`. | With one setting, the noise that passes the governor filter matches the tractor within 20 % at shifts 4 and 2. |
| I7 | Report delay | Bring `rptdelay` forward from section 14: 0–50 ms of delay on the speed in `0x1DA`. It is needed for the delay-margin checks in sections 16 and 17. | A unit test passes, and `params` shows it. |
| I8 | Metrics | Add to `metrics.py`: integrated absolute error, ring period and decay ratio after a step (which give a phase-margin estimate), and torque RMS as a measure of actuator effort. Add one `cost()` function, shared by the sweep, the software-in-the-loop harness and the gain search. | Self-test on known signals. |
| I9 | Tractor logger (V6) | M7 is blocked: the telematics logger keeps about 15 % of frames in run mode. **ASK** Ben which route to take: fix that logger, or add a listen-only logger build of this firmware (`TWAI_MODE_LISTEN_ONLY`, the filtered `canlog` IDs, binary records written to SD from a core-0 task, about 8 kB/s). | A 10-minute bench log with no gaps in the `0x4A1` counter. |
| I10 | SD transfer over serial | Apply the stash, capture the panic backtrace from `sd ls`, and fix it. Suspects are the console task's stack size and SD access from the wrong task. | `sd ls` and `sd get` work, and `topper.json` is off the card. |
| I11 | Two small code checks | In `src/sim_task.cpp`, `apply()` ignores a failed `param_set`. In `src/can_io.cpp`, the `0x1D4` latch decodes with `K_T_DEFAULT`, while the tick uses the `kT` parameter. Confirm each is harmless, or fix it. | A note in the commit message. |

### Progress (10 Nov 2026)

| # | State |
| --- | --- |
| I1 | Done. The Zombie facts hold, apart from the ID 162 clash, which is gone: see section 2. `CLAUDE.md`, the README and sections 3, 4, 6, 7, 9 and 12 are updated. |
| I2 | Done as proposed (Ben, 10 Nov). `logs/` is in `.gitignore`. The five 3 Nov sweeps are in `docs/bench/results/`, each with its YAML, the table as run, and the same bus logs re-scored by today's `sweep.py`; `docs/tractor/` goes in with the commit. |
| I3 | Built: `.github/workflows/ci.yml`. A green run needs a push. |
| I4 | Done: seven `sweep.py` checks in `selftest.py`, against a fake console. While writing them, the fixed 0.2 s wait after a gradient ramp was replaced: the console now counts the replies to unchecked commands, and each checked command waits for them first. |
| I5 | Built: both noise sources restart on `preset` and `mark`, and the pulse phase on `preset` and `load pulse`, but not on `mark`, which `baler_example.yaml` sends mid-stroke. `repeats: N` writes `metrics_summary.csv`. The "under 5 rpm" check needs the bench. |
| I6 | Built and fitted: `lib/sensor`, parameters `noise_fc` and `noise_bw`, mirrored in `tools/simmodel.py` and checked against the C++ in `selftest.py`. `tools/fit_noise.py` on the 7 Nov logs gives `noise` 11, `noise_fc` 6, `noise_bw` 4: within 5 % of the tractor through the filter at shifts 4, 3, 2 and 1 (white noise would need 11 at shift 4 but 16 at shift 2). `sweep_road.yaml` uses that one setting. |
| I7 | Built: `rptdelay`, with unit tests; `params` lists it. |
| I8 | Done: `iae_rpm_s`, `torque_rms_nm`, `ring_after_step`, `phase_margin_deg` and `cost()` in `metrics.py`, with self-test checks. `sweep.py` ends each set with the load taken off (`tail_s`, default 4 s) and scores the ring there. |
| I9 | Ben fixed the telematics logger himself (10 Nov); new logs to come. |
| I10 | Waiting for the bench: the board must be connected to capture the backtrace. |
| I11 | Both harmless. The console range-checks before posting `set`, so `param_set` cannot fail; `apply()` now sends the event only for a value it took. `Cmd1D4::torque_nm` from the latch is never read: the tick scales `raw` by `kT` itself. Commented in `can_io.cpp`. |

## 16. Tuning improvements

Tuning so far has been by bench sweep, at 30 to 90 seconds per gain set. This section adds a model of the loop that says where to look, a PC harness that runs the Zombie's own governor code against the plant in milliseconds, and a search over that harness. The bench then confirms a short list instead of exploring.

### Loop model

The plant is an integrator: motor torque into one inertia. Around it sit three lags:

| Lag | Size |
| --- | --- |
| Governor speed filter | 155 ms at `govFlt` 4, 75 ms at 3, 35 ms at 2, 14 ms at 1 |
| Motor torque lag `tau` | 30 ms |
| Transport | Three ticks, 30 ms, plus the 10 ms hold (measured in M5c): the governor uses the speed it read last tick, and `Ms10Task` sends that tick's 0x1D4 before `SetTorque` stores the new request. Four ticks while the clock phase puts the 0x1D4 just after the simulator's tick. |

In physical units, `govKp` 1 (0.01 % per rpm) is 0.489 Nm per rad/s at `kT` 0.25. With P only, the loop gain is

```latex
L(j\omega) = \frac{K_p}{J\,j\omega}\cdot\frac{1}{1+j\omega\tau_f}\cdot\frac{1}{1+j\omega\tau}\cdot e^{-j\omega\theta}
```

where τ_f is the filter lag and θ the transport delay. Stability therefore depends on `govKp` divided by inertia, and the usable `govKp` is proportional to inertia.

A hand calculation for `govKp` 11 and J 0.30 kg·m² matches the M4 bench:

| `govFlt` | Predicted crossover | Predicted margin and ring | Measured ring (M4) |
| --- | --- | --- | --- |
| 4 | 9.5 rad/s | About 7°, 660 ms | 630 ms, each cycle 0.65 of the last (about 7°) |
| 2 | 14.6 rad/s | About 21°, 400 ms | 370 ms, each cycle 0.35 of the last (about 17°) |

Inertia is not one number on this machine. The placeholder presets and the 7 Nov tractor logs span about ten to one: 0.30 at no load, 0.90 for the baler, and 2.5 to 3.5 on the road. No fixed `govKp` suits that range. At shift 4, `govKp` 11 is near the stability limit at J 0.3, and two to three times softer than it could be at J 2.5.

The model gives these starting values. They are predictions from placeholder inertias, to be confirmed in the harness and then on the bench:

| `govFlt` | `govKp` for about 45° margin | At J 0.3 | At J 0.9 | At J 2.5 |
| --- | --- | --- | --- | --- |
| 4 | About 10 × J | 3 | 9 | 25 |
| 2 | About 20 × J | 6 | 18 | 50 |

- Shift 2 about doubles the usable `govKp`. Try shift 1 once coloured noise (I6) is in.
- Set `govKi` from `govKp`. The integral time Ti is `govKp` / (10 × `govKi`) seconds, so `govKp` 11 with `govKi` 1 is 1.1 s. Aim for Ti of about 4 divided by the crossover in rad/s.
- Try `govImax` at about 60. Above about 55 % the motor is already at its 280 Nm limit, so a larger integral only adds wind-up.
- Add road sets at `govKp` 20 to 30 with shift 2 to `sweep_road.yaml`. Done 10 Nov 2026: `govKp` 20 and 30 P-only, and 25 with `govKi` 3.

Build `tools/loopcalc.py`: from `govKp`, `govKi`, `govKd`, `govFlt`, J, `tau` and delay, it prints the crossover, the phase and gain margins and the predicted ring period. Keep the unit conversions in that one file.

Done 11 Nov 2026 as an exact discrete-time linearisation: the plant's 1 ms sub-steps, the measured three-tick delay, and the Zombie's filters. It matches the harness to within 1.4 % on ring period and the M4 bench to within 2 %. With that delay, the table above holds: 45° of margin needs about 10.5×J at shift 4 and 21×J at shift 2 (`docs/bench/M5c_sil.md`, finding 5). It does not describe PI loops at `revlim` (finding 3).

### Software-in-the-loop harness (M5c)

`tools/sil/` holds a PC program that links the Zombie's `throttle.cpp` and `govlog.cpp`, as `Stm32-vcu/test/Makefile` already does, with this repo's `lib/plant`, `lib/loads` and `lib/leafcodec`. The Zombie path comes from `--zombie` or `ZOMBIE_DIR`, and the Zombie commit hash is written to the output.

Each 10 ms step, in order:

1. Decode the speed from the previous tick's `0x1DA` with the Zombie's own expression, so the governor sees whole rpm one tick late.
2. Call `SpeedLimitCommand`, then `RampThrottle`.
3. Convert percent to the raw request, encode `0x1D4` and decode it.
4. Run ten plant sub-steps.
5. Encode `0x1DA` with the speed noise and report delay.

The voltage, current and temperature derates are left out, and the output says so.

The harness reads the same sweep YAML as `sweep.py`. It writes a candump log with `0x4A0`–`0x4A3` and `0x4B0`–`0x4B2`, so `loglib` and `sweep.score()` work on it unchanged. `sweep.py --port sil` runs a sweep against it.

Acceptance:

- The Zombie golden test inputs give the golden outputs through the harness.
- Three existing bench runs reproduce: ring period within 10 % and stroke ripple within 15 %. If the bench logs cannot be found, run them again, which needs Ben's pedal clamp.
- `loopcalc.py` and the harness agree on ring period within 10 %.

**Progress (11 Nov 2026): built, and all three acceptance tests pass.** The details are in `docs/bench/M5c_sil.md`.

- **Golden test:** passes bit for bit.
- **Bench sweeps:** all five 3 Nov sweeps reproduce, 22 judged sets, with stroke ripple within 0.5 %.
- **Rings:** the M4 pedal-test rings reproduce exactly (0.640 / 0.640 s and 0.660 / 0.660 s).
- **Calculator against harness:** agree to within 1.4 % on ring period, and to within 0.6–5.2 % RMS on the whole response.

The harness differs from the steps above in three ways.

- The simulator's tick became `lib/simengine`, shared with the firmware, so the plant cannot drift from it.
- The order follows the bench, which was measured from the 3 Nov logs: each 0x1D4 carries the previous tick's request. That makes three ticks from a speed sample to its torque, not two.
- `regenramp` stands at 100, not the default 1. The bench Zombie's regen was never ramp-limited (V8).

`sweep.py`, `run_script.py`, `sil_check.py` and `sil_replay.py` drive the harness through `tools/silport.py`, in simulated time. Every sweep row now also has `loop_ticks`, the bench's measured loop delay.

### Gain search (M5d)

`tools/tune_search.py` searches over the harness.

| Part | Choice |
| --- | --- |
| Variables | `govKp`, `govKi`, `govKd`, `govFlt`, `govImax` |
| Scenarios | Fast hit on the limit, baler, road gradient ramp, PTO step with an inertia change |
| Each candidate must hold across | J from 0.7 to 1.4 times nominal, `tau` 20–45 ms, extra delay 0–20 ms, tractor-level speed noise |
| Hard limits | No-load ring decays to under 0.25 per cycle in every case; an overshoot limit set by Ben |
| Then minimise | `cost()` from I8: speed error, stroke ripple and torque effort |
| Method | Coarse grid, then refinement with `scipy` |

The output is a ranked table and a sweep YAML holding the top three, for confirmation on the bench. If the bench ranks them differently, report the difference and treat it as a model fault to find.

### Governor changes to evaluate in the harness only

These four change control logic. None goes into a Zombie branch without Ben's sign-off, and each must sit behind a parameter whose default leaves today's governor unchanged, bit for bit.

| # | Change | Reason |
| --- | --- | --- |
| G1 | Clear the integral when `govKi` is 0 | Today it stays frozen at its last value, which spoiled the second road sweep. |
| G2 | Make the D-term slope filter shift a parameter | It is fixed at shift 4 (about 155 ms) whatever `govFlt` is, so `govKd` lags at the stroke frequency. |
| G3 | Schedule `govKp` on inertia | Section 17. |
| G4 | Feed-forward from a load-torque observer | Keep it only if it beats G3 at the same stability margin. |
| G5 | Over `revlim`, stop forcing the output to zero or below while the integral carries load | Today `MIN(0, P + I + D)` cuts the torque to zero one rpm over the limit, so any `govKi` above 0 hunts at `revlim` like a relay: 37–90 rpm at no load on the bench (M5c finding 3). |
| G6 | Call `SetTorque` before the inverter's `Task10Ms` in `Ms10Task` | The 0x1D4 carries the previous tick's request, an extra 10 ms in the loop. Two ticks instead of three gives 13° of margin at shift 4 and J 0.3, against 7.5° today (M5c finding 4). |

## 17. Phase 3: governor auto-tune

Phase 3 lets the governor find its own gains for whatever is on the shaft: first on demand, then continuously while working. Do not start it without Ben. It changes Zombie control logic, so the logging-only rule does not cover it, and it needs M5c and M5d first.

The tuner lives in the Zombie firmware, on a new branch `gov-autotune`. It is not an external device writing SDO to a moving tractor. All of its logic is one plain C++ module, `govtune`, with no hardware includes, so the same code runs in host tests, in the harness and on the Zombie.

### Three parts

**Tune now.** On request at governed speed, a relay replaces the governor output: the mean torque plus or minus `govTuneAmp` %, switched on the sign of the speed error with hysteresis, for about ten cycles. The speed then oscillates with amplitude a and period Tu. The ultimate gain is Ku = 4d / (π a), where d is the relay amplitude. Start from `govKp` = 0.4 Ku and an integral time of Tu. This measures inertia and every lag together, with no model. The test aborts on pedal lift, brake, speed leaving a window, or torque clipping.

**Continuous inertia estimate.** While the governor is the binding limit, add a small sine to its output. Start with 3 Hz at ±2 %, which moves speed by about 6 rpm at J 0.9, less than the ripple already there. Demodulate both the total torque command and the raw speed at the probe frequency, over whole cycles. Their ratio is the plant response at that frequency. Load does not bias it, because the probe is uncorrelated with the load. The response gives the effective inertia J_est. Then:

- `govKp` = Kp_ref × J_est / J_ref, with `govKi` following to keep the integral time;
- both clamped to a range and changed by no more than about 10 % per second;
- the estimate frozen whenever the governor is not binding.

**Oscillation watchdog.** Whenever gains are being applied, sustained ringing near the crossover frequency cuts `govKp` by 30 % and holds it there, with a flag in the log.

### Modes and logging

Parameter `govTune`:

| Value | Behaviour |
| --- | --- |
| 0 Off | Today's governor, bit for bit. The golden test must pass. Default. |
| 1 Estimate | The probe and the estimate run and are logged. The gains are not touched. |
| 2 Apply | Gains follow the estimate, with the watchdog on. |

Parameter IDs start from the next free one found in I1. With `GovLog` on, a new frame `0x4A6` carries the inertia estimate, the scheduled `govKp`, the probe amplitude, the tuner state and its flags. Add it to `leaf_sim.dbc`.

### Milestones

| # | Milestone | Built by | Acceptance |
| --- | --- | --- | --- |
| M14 | `govtune` in the harness | Claude | The relay result is within 20 % of the gain search's best `govKp` at J 0.3, 0.9 and 2.5. The probe estimate is within 15 % in 3 s, with baler pulses and tractor-level noise. The watchdog recovers from a three-to-one drop in inertia within 2 s. |
| M15 | Zombie `gov-autotune`, Estimate only | Claude, Ben tests | Golden test unchanged at `govTune` 0. On the bench, the estimate follows `set J` and `load step … dJ` changes on the simulator. |
| M16 | Apply on the bench | Claude, Ben tests | With inertia stepped 0.3 → 0.9 → 2.5 → 0.3, ring decay stays under 0.25 per cycle, and ripple is no worse than the best fixed gain for each inertia. The same holds at `rptdelay` 20 ms. |
| M17 | Tractor | Ben, Claude analyses | Estimate mode logged first: stationary with the PTO running, then working. The DC-DC check in section 8 applies. Apply mode only after Ben has reviewed those logs. |

### A bench-only first step

Before M15, and without touching the Zombie, `tools/autotune_bench.py` can do a version of this from the PC. It applies a load step, reads the ring period and decay from `canlog`, and moves `govKp` over `zset` toward a target decay. That proves the idea and the metrics on real hardware.

The same estimator can later schedule `spdkp` in Phase 2 speed mode.
