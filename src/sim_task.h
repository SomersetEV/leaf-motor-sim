// The 10 ms simulation tick. See docs/SIMULATOR_PLAN.md section 7.
#pragma once

#include <stdint.h>

#include "leafcodec.h"
#include "tick_stats.h"

struct SimParams {
    float udc_v;      // reported bus voltage
    float motor_c;    // reported motor temperature
    float inv_c;      // reported inverter temperature
    float k_t;        // Nm per 0x1D4 request bit
    bool  inv_fault;  // inverter error bit in 0x1DA
};

// Copy of the tick's state for the console. Taken under a lock.
struct SimSnapshot {
    uint32_t          sim_ms;
    float             rpm;
    leafcodec::Cmd1D4 cmd;
    bool              cmd_fresh;  // false: no valid 0x1D4 within the timeout
    SimParams         params;
    simcore::TickStats stats;
};

void sim_start_task();
SimSnapshot sim_get_snapshot();
