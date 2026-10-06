// Load components added to the plant's base friction.
// See docs/SIMULATOR_PLAN.md section 6, "Load components".
//
// Resistive loads (step, pulse, noise, profile) always oppose rotation and
// cannot turn the shaft. The gradient is an active, signed load and can
// drive the shaft (overrun). Advanced once per 10 ms tick.
//
// Plain C++, no heap.
#pragma once

#include <stdint.h>

#include "gauss.h"
#include "plant.h"
#include "profile.h"

namespace loads {

enum class ProfileState : uint8_t { Idle = 0, Loaded = 1, Running = 2, Finished = 3 };

struct LoadConfig {
    // Step: ramps from its current value to step_nm; dj applied at ramp end.
    bool  step_on       = false;
    float step_nm       = 0.0f;
    float step_ramp_ms  = 0.0f;
    float step_dj_kgm2  = 0.0f;
    // Pulse: half-sine of peak pulse_nm for duty % of each period.
    bool  pulse_on        = false;
    float pulse_nm        = 0.0f;
    float pulse_period_ms = 1000.0f;
    float pulse_duty_pct  = 50.0f;
    // Noise: Gaussian, low-pass filtered at noise_hz, standard deviation noise_nm.
    bool  noise_on  = false;
    float noise_nm  = 0.0f;
    float noise_hz  = 2.0f;
    // Gradient: signed active torque; positive opposes forward rotation.
    float grad_nm = 0.0f;
};

struct LoadOutput {
    plant::TickLoads tick;       // for Plant::substep
    float inertia_delta_kgm2;    // step dJ in force (0 if none)
    bool  profile_inertia;       // profile_inertia_kgm2 replaces the model inertia
    float profile_inertia_kgm2;
};

class Loads {
public:
    explicit Loads(uint32_t seed = 0x2545F491u) : gauss_(seed) {}

    // Applies a whole configuration (presets). Steps jump to their target.
    void configure(const LoadConfig &c);
    const LoadConfig &config() const { return cfg_; }

    void start_step(float nm, float ramp_ms, float dj_kgm2);
    void start_pulse(float peak_nm, float period_ms, float duty_pct);
    void set_noise(float sd_nm, float corner_hz);
    void set_grad(float nm) { cfg_.grad_nm = nm; }
    // Removes step, pulse, gradient and noise. The profile is unaffected.
    void off();

    // Profile replay. The profile must outlive its use here; pass nullptr
    // before freeing it.
    void set_profile(const Profile *p);
    const Profile *profile() const { return profile_; }
    bool profile_start();      // from row 0; false if no profile
    void profile_stop();
    void set_profile_loop(bool loop) { loop_ = loop; }
    bool profile_loop() const { return loop_; }
    // Profile torque scales with (|w| / w_ref)^n; n = 0 disables.
    void set_profile_exponent(float n, float w_ref_rpm) { exp_n_ = n; exp_wref_rpm_ = w_ref_rpm; }
    float profile_exponent() const { return exp_n_; }
    float profile_wref_rpm() const { return exp_wref_rpm_; }
    ProfileState profile_state() const { return state_; }
    uint32_t profile_row() const { return row_; }

    // Advances every component by one tick of dt_s at shaft speed w_rad_s.
    LoadOutput tick(float w_rad_s, float dt_s);

    float step_level_nm() const { return step_level_; }

private:
    LoadConfig cfg_;
    Gauss gauss_;

    float step_level_ = 0.0f;   // current step torque
    float step_start_ = 0.0f;   // torque at the start of the ramp
    float step_t_ms_  = 0.0f;   // time into the ramp
    float step_dj_in_force_ = 0.0f;

    float pulse_t_ms_ = 0.0f;
    float noise_y_ = 0.0f;

    const Profile *profile_ = nullptr;
    ProfileState state_ = ProfileState::Idle;
    uint32_t row_ = 0;
    bool  loop_ = false;
    float exp_n_ = 0.0f;
    float exp_wref_rpm_ = 0.0f;
};

} // namespace loads
