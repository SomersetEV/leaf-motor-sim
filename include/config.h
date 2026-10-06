// CAN ids, periods and default model parameters.
// See docs/SIMULATOR_PLAN.md sections 5 and 6.
#pragma once

#include <stdint.h>

// --- CAN bus ---------------------------------------------------------------
constexpr uint32_t CAN_BITRATE = 500000;

// Frames received from the Zombie
constexpr uint32_t ID_LEAF_TORQUE_CMD = 0x1D4; // torque request, HV status
constexpr uint32_t ID_LEAF_VCM_GEAR   = 0x11A; // gear and on/off, telemetry only
constexpr uint32_t ID_LEAF_VCM_ALIVE  = 0x50B; // presence only

// Frames transmitted by the simulator as the inverter
constexpr uint32_t ID_LEAF_INV_STATUS = 0x1DA; // voltage, speed, error
constexpr uint32_t ID_LEAF_INV_TEMPS  = 0x55A; // motor and inverter temperature

// Simulator telemetry frames
constexpr uint32_t ID_SIM_FAST   = 0x4B0; // 10 ms
constexpr uint32_t ID_SIM_SLOW   = 0x4B1; // 100 ms
constexpr uint32_t ID_SIM_EVENT  = 0x4B2; // on event

// --- Timing ----------------------------------------------------------------
constexpr uint32_t TICK_MS            = 10;  // simulation tick, 0x1DA period
constexpr uint32_t SLOW_PERIOD_TICKS  = 10;  // 0x55A and 0x4B1 every 100 ms
constexpr uint32_t SUBSTEPS_PER_TICK  = 10;  // 1 ms model sub-steps
constexpr uint32_t RX_TIMEOUT_MS      = 100; // no valid 0x1D4 -> zero torque
constexpr uint32_t TICK_TOLERANCE_US  = 500; // target 10 ms +/- 0.5 ms
constexpr uint32_t JITTER_WINDOW_TICKS = 100; // worst jitter per second

// --- Tasks -----------------------------------------------------------------
// The tick and CAN receive run on core 1; the console runs on core 0.
constexpr int SIM_TASK_CORE      = 1;
constexpr int SIM_TASK_PRIO      = 24; // configMAX_PRIORITIES - 1
constexpr int CAN_RX_TASK_CORE   = 1;
constexpr int CAN_RX_TASK_PRIO   = 23;
constexpr int CONSOLE_TASK_CORE  = 0;
constexpr int CONSOLE_TASK_PRIO  = 1;
constexpr uint32_t TASK_STACK_BYTES = 4096;
constexpr uint32_t CONSOLE_STACK_BYTES = 6144; // snapshots and SD buffers

constexpr uint32_t CAN_TX_QUEUE_LEN = 32; // plan: at least 16
constexpr uint32_t CAN_RX_QUEUE_LEN = 32;

// --- Console, streaming, files -----------------------------------------------
constexpr uint32_t SIM_CMD_QUEUE_LEN   = 8;    // console -> tick commands
constexpr uint32_t STREAM_RING_LEN     = 256;  // samples, power of two
constexpr uint32_t STATUS_WINDOW_MS    = 1000; // "in the last second" (LED, flags)
constexpr uint32_t PRESET_JSON_MAX     = 1024; // bytes
constexpr const char *SD_PROFILE_DIR   = "/profiles/";
constexpr const char *SD_PRESET_DIR    = "/presets/";
constexpr uint8_t  LED_LEVEL           = 24;   // WS2812 brightness, 0-255
constexpr uint32_t SPEED_NOISE_SEED    = 0x5EED1DA1u;
constexpr uint32_t LOAD_NOISE_SEED     = 0x10ADu;

// --- Default model parameters (placeholders until section 9 fits) ---------

// VERIFY (plan section 12, V1): torque request scale in Nm per 0x1D4 bit.
constexpr float K_T_DEFAULT = 0.25f;

// VERIFY (plan section 12, V3): reported torque layout in 0x1DA bytes 2-3.
// Off until confirmed from a tractor capture.
#define SIM_REPORT_TORQUE 0

// Plant (plan section 6). Placeholders until the identification runs.
constexpr float J_DEFAULT_KGM2   = 0.30f;  // no-load acceleration run
constexpr float TC_DEFAULT_NM    = 5.0f;   // coast-down
constexpr float B_DEFAULT_NMS    = 0.02f;  // coast-down, Nm·s/rad
constexpr float C_DEFAULT_NMS2   = 0.0f;   // coast-down, Nm·s²/rad²
constexpr float TAU_DEFAULT_MS   = 30.0f;  // request against reported torque in logs
// VERIFY (plan section 12, V2): motor type on the tractor.
constexpr float TMAX_DEFAULT_NM  = 280.0f;
constexpr float PMAX_DEFAULT_KW  = 80.0f;

constexpr float UDC_DEFAULT_V        = 360.0f; // tractor pack nominal
constexpr float MOTOR_TEMP_DEFAULT_C = 40.0f;
constexpr float INV_TEMP_DEFAULT_C   = 40.0f;
