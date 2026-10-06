// The 10 ms simulation tick. See docs/SIMULATOR_PLAN.md section 7.
//
// The tick owns the model, the loads and the profile replay. Other tasks
// change them only by posting a SimCmd, which the tick applies at the start
// of its next tick, and read them through a snapshot copied under a lock.
#pragma once

#include <stdint.h>

#include "commands.h"
#include "leafcodec.h"
#include "loads.h"
#include "sim_params.h"
#include "telemetry.h"
#include "tick_stats.h"

struct SimSnapshot {
    uint32_t          sim_ms;
    leafcodec::Cmd1D4 cmd;
    bool              cmd_fresh;    // false: no valid 0x1D4 within the timeout
    float             t_req_nm;     // decoded request, zero on timeout
    float             t_motor_nm;   // applied motor torque after envelope and lag
    float             t_load_nm;    // signed load torque on the shaft
    float             rpm;          // model speed (no measurement noise)
    float             inertia_kgm2; // inertia in use this tick
    bool              clipping;
    bool              inv_fault;
    uint8_t           flags;        // simproto::FLAG_*
    SimParams         params;
    uint8_t           preset_id;
    char              preset_name[16];
    loads::LoadConfig loads;
    float             step_level_nm;
    loads::ProfileState profile_state;
    const loads::Profile *profile;
    uint32_t          profile_row;
    bool              profile_loop;
    float             profile_exp_n;
    float             profile_wref_rpm;
    simcore::TickStats stats;
};

void sim_start_task(const SimParams &initial);

// Queues a command for the next tick. Never blocks; false if the queue is full.
bool sim_post(const simproto::SimCmd &c);

SimSnapshot sim_get_snapshot();

// CSV streaming: the tick pushes one sample every divider ticks while on.
void sim_set_stream(bool on, uint32_t divider);
bool sim_stream_pop(simproto::StreamSample &s);
void sim_stream_clear();
uint32_t sim_stream_dropped();
