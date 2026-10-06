// The 10 ms simulation tick. See docs/SIMULATOR_PLAN.md section 7.
#pragma once

#include <stdint.h>

#include "leafcodec.h"
#include "plant.h"
#include "tick_stats.h"

struct SimParams {
    plant::PlantParams plant;
    float k_t;        // Nm per 0x1D4 request bit
    float udc_v;      // reported bus voltage
    float motor_c;    // reported motor temperature
    float inv_c;      // reported inverter temperature
    bool  inv_fault;  // inverter error bit in 0x1DA
};

// Copy of the tick's state for the console. Taken under a lock.
struct SimSnapshot {
    uint32_t          sim_ms;
    leafcodec::Cmd1D4 cmd;
    bool              cmd_fresh;  // false: no valid 0x1D4 within the timeout
    float             t_req_nm;   // decoded request, zero on timeout
    float             t_motor_nm; // applied motor torque after envelope and lag
    float             t_load_nm;  // signed load torque on the shaft
    float             rpm;        // model speed
    bool              clipping;   // request exceeded the motor envelope
    SimParams         params;
    simcore::TickStats stats;
};

void sim_start_task();
SimSnapshot sim_get_snapshot();
