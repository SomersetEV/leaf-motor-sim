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

constexpr uint32_t CAN_TX_QUEUE_LEN = 32; // plan: at least 16
constexpr uint32_t CAN_RX_QUEUE_LEN = 32;

// --- Default model parameters (placeholders until section 9 fits) ---------

// VERIFY (plan section 12, V1): torque request scale in Nm per 0x1D4 bit.
constexpr float K_T_DEFAULT = 0.25f;

// VERIFY (plan section 12, V3): reported torque layout in 0x1DA bytes 2-3.
// Off until confirmed from a tractor capture.
#define SIM_REPORT_TORQUE 0

constexpr float UDC_DEFAULT_V        = 360.0f; // tractor pack nominal
constexpr float MOTOR_TEMP_DEFAULT_C = 40.0f;
constexpr float INV_TEMP_DEFAULT_C   = 40.0f;
