// The simulator's 10 ms tick without the hardware: applies console commands,
// advances the loads and the model, and builds every frame the simulator
// sends. The firmware's sim task (src/sim_task.cpp) and the PC
// software-in-the-loop harness (tools/sil) both run this one class, so the
// harness's plant is the firmware's plant, tick for tick (plan section 16).
//
// Plain C++, no heap. Frames go out through a FrameSink, which the firmware
// points at the CAN driver and the harness at its candump writer.
#pragma once

#include <stdint.h>

#include "commands.h"
#include "leafcodec.h"
#include "loads.h"
#include "plant.h"
#include "sim_params.h"
#include "speed_report.h"

namespace simengine {

class FrameSink {
public:
    virtual void send(uint32_t id, const uint8_t *data, uint8_t dlc) = 0;

protected:
    ~FrameSink() = default;
};

// Fixed settings, from include/config.h in both builds.
struct Config {
    uint32_t tick_ms;
    uint32_t substeps;           // model sub-steps per tick
    uint32_t slow_period_ticks;  // 0x55A and 0x4B1 every this many ticks
    uint32_t load_noise_seed;
    uint32_t speed_noise_seed;
    bool     report_torque;      // motor torque in 0x1DA (plan V3)
    uint32_t id_inv_status;      // 0x1DA
    uint32_t id_inv_temps;       // 0x55A
    uint32_t id_sim_fast;        // 0x4B0
    uint32_t id_sim_slow;        // 0x4B1
    uint32_t id_sim_event;       // 0x4B2
};

// What the tick needs from outside the model.
struct TickIn {
    uint32_t          now_ms;
    bool              fresh;           // a valid 0x1D4 within the timeout
    leafcodec::Cmd1D4 cmd;             // the latest one, zero torque when not fresh
    bool              bus_error;       // a bus error within the status window
    uint32_t          rejected_1d4;
    uint32_t          overruns;
    uint32_t          worst_jitter_us;
};

struct TickOut {
    float   t_req_nm;      // request applied this tick
    float   rpm;           // model speed, no measurement noise
    float   rpm_reported;  // speed sent in 0x1DA
    float   inertia_kgm2;
    uint8_t flags;         // simproto::FLAG_*
};

class Engine {
public:
    Engine(const Config &cfg, const SimParams &initial);

    // Applies one console command; runs at the start of a tick.
    void apply(const simproto::SimCmd &c, uint32_t now_ms, FrameSink &out);

    // One tick: loads, ten model sub-steps, then 0x1DA and 0x4B0, and on every
    // slow_period_ticks-th tick (the first included) 0x55A and 0x4B1.
    TickOut tick(const TickIn &in, FrameSink &out);

    const SimParams &params() const { return params_; }
    const plant::Plant &plant() const { return plant_; }
    const loads::Loads &loads() const { return loads_; }
    uint8_t preset_id() const { return preset_id_; }
    const char *preset_name() const { return preset_name_; }
    bool inv_fault() const { return inv_fault_; }
    uint32_t ticks() const { return tick_; }

private:
    void send_event(uint8_t ev, uint16_t value, uint32_t now_ms, FrameSink &out);
    void reseed();

    Config cfg_;
    SimParams params_;
    plant::Plant plant_;
    loads::Loads loads_;
    sensor::SpeedReport report_;
    bool inv_fault_ = false;
    uint8_t preset_id_ = 0;
    char preset_name_[16] = "";
    uint32_t tick_ = 0;
};

} // namespace simengine
