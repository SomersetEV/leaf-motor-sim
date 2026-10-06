// Motor and driveline model: a single rotating inertia driven by a
// torque-limited, lagged motor and opposed by friction and loads.
// See docs/SIMULATOR_PLAN.md section 6.
//
// Plain C++. Interfaces use rpm, Nm, kg·m², kW and ms; the model state is in
// rad/s.
#pragma once

#include <stdint.h>

namespace plant {

struct PlantParams {
    float j_kgm2;    // total inertia referred to the motor shaft
    float tc_nm;     // constant friction torque
    float b_nms;     // viscous coefficient, Nm·s/rad
    float c_nms2;    // quadratic coefficient, Nm·s²/rad²
    float tau_ms;    // motor torque lag time constant
    float tmax_nm;   // motor torque limit
    float pmax_kw;   // motor power limit
};

// Loads beyond base friction, held constant across the sub-steps of a tick.
// Filled in by lib/loads from M5.
struct TickLoads {
    float t_res_nm    = 0.0f;  // extra resistive torque, >= 0, always opposes rotation
    float t_active_nm = 0.0f;  // signed active torque (gradient); positive opposes forward
};

class Plant {
public:
    static constexpr float W_EPS = 0.01f;     // rad/s, below this the shaft can be held
    static constexpr float J_MIN = 0.001f;    // kg·m², guards against divide by zero

    explicit Plant(const PlantParams &p) { set_params(p); }
    void set_params(const PlantParams &p);
    const PlantParams &params() const { return p_; }

    // Advances the model by dt_s with torque request t_req_nm.
    void substep(float t_req_nm, const TickLoads &loads, float dt_s);

    void reset();

    float w_rad_s() const { return w_; }
    float rpm() const;
    float t_cmd_nm() const { return t_cmd_; }      // request after the envelope
    float t_motor_nm() const { return t_motor_; }  // after the lag
    float t_load_nm() const { return t_load_; }    // signed total load on the shaft
    bool  clipping() const { return clipping_; }   // request exceeded the envelope

    // Torque limit at speed w: min(Tmax, Pmax / |w|).
    float torque_limit_nm(float w_rad_s) const;

private:
    PlantParams p_{};
    float w_       = 0.0f;
    float t_cmd_   = 0.0f;
    float t_motor_ = 0.0f;
    float t_load_  = 0.0f;
    bool  clipping_ = false;
};

} // namespace plant
