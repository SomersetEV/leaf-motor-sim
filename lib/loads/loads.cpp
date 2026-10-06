#include "loads.h"

#include <math.h>

namespace loads {

static constexpr float PI_F = 3.14159265358979f;
static constexpr float RPM_TO_RAD_S = 2.0f * PI_F / 60.0f;

void Loads::configure(const LoadConfig &c) {
    cfg_ = c;
    step_level_ = c.step_on ? c.step_nm : 0.0f;
    step_start_ = step_level_;
    step_t_ms_  = c.step_ramp_ms;  // ramp complete
    step_dj_in_force_ = c.step_on ? c.step_dj_kgm2 : 0.0f;
    pulse_t_ms_ = 0.0f;
    noise_y_    = 0.0f;
}

void Loads::start_step(float nm, float ramp_ms, float dj_kgm2) {
    cfg_.step_on      = true;
    cfg_.step_nm      = nm < 0.0f ? 0.0f : nm;
    cfg_.step_ramp_ms = ramp_ms < 0.0f ? 0.0f : ramp_ms;
    cfg_.step_dj_kgm2 = dj_kgm2;
    step_start_ = step_level_;
    step_t_ms_  = 0.0f;
}

void Loads::start_pulse(float peak_nm, float period_ms, float duty_pct) {
    cfg_.pulse_on        = true;
    cfg_.pulse_nm        = peak_nm < 0.0f ? 0.0f : peak_nm;
    cfg_.pulse_period_ms = period_ms < 10.0f ? 10.0f : period_ms;
    cfg_.pulse_duty_pct  = duty_pct < 0.0f ? 0.0f : (duty_pct > 100.0f ? 100.0f : duty_pct);
    pulse_t_ms_ = 0.0f;
}

void Loads::set_noise(float sd_nm, float corner_hz) {
    cfg_.noise_on = sd_nm > 0.0f;
    cfg_.noise_nm = sd_nm < 0.0f ? 0.0f : sd_nm;
    cfg_.noise_hz = corner_hz > 0.0f ? corner_hz : 2.0f;
}

void Loads::off() {
    LoadConfig c;
    c.pulse_period_ms = cfg_.pulse_period_ms;
    c.noise_hz = cfg_.noise_hz;
    cfg_ = c;
    step_level_ = step_start_ = 0.0f;
    step_t_ms_ = 0.0f;
    step_dj_in_force_ = 0.0f;
    noise_y_ = 0.0f;
}

void Loads::set_profile(const Profile *p) {
    profile_ = p;
    row_ = 0;
    state_ = (p && p->rows() > 0) ? ProfileState::Loaded : ProfileState::Idle;
}

bool Loads::profile_start() {
    if (!profile_ || profile_->rows() == 0) return false;
    row_ = 0;
    state_ = ProfileState::Running;
    return true;
}

void Loads::profile_stop() {
    if (state_ == ProfileState::Running) state_ = ProfileState::Loaded;
}

LoadOutput Loads::tick(float w, float dt_s) {
    float dt_ms = dt_s * 1000.0f;
    float res = 0.0f;
    LoadOutput out{};

    // Step with linear ramp; dJ takes effect at the end of the ramp.
    if (cfg_.step_on) {
        if (step_t_ms_ < cfg_.step_ramp_ms) {
            step_t_ms_ += dt_ms;
            float f = step_t_ms_ >= cfg_.step_ramp_ms ? 1.0f : step_t_ms_ / cfg_.step_ramp_ms;
            step_level_ = step_start_ + (cfg_.step_nm - step_start_) * f;
        } else {
            step_level_ = cfg_.step_nm;
        }
        if (step_t_ms_ >= cfg_.step_ramp_ms) step_dj_in_force_ = cfg_.step_dj_kgm2;
        res += step_level_;
    }

    // Half-sine pulse during the first duty % of each period.
    if (cfg_.pulse_on) {
        float on_ms = cfg_.pulse_period_ms * cfg_.pulse_duty_pct / 100.0f;
        if (pulse_t_ms_ < on_ms && on_ms > 0.0f)
            res += cfg_.pulse_nm * sinf(PI_F * pulse_t_ms_ / on_ms);
        pulse_t_ms_ += dt_ms;
        if (pulse_t_ms_ >= cfg_.pulse_period_ms) pulse_t_ms_ -= cfg_.pulse_period_ms;
    }

    // First-order low-pass filtered white noise, scaled to noise_nm at the output.
    if (cfg_.noise_on) {
        float a = 1.0f - expf(-2.0f * PI_F * cfg_.noise_hz * dt_s);
        float sd_in = cfg_.noise_nm * sqrtf((2.0f - a) / a);
        noise_y_ += a * (sd_in * gauss_.next() - noise_y_);
        res += noise_y_;
    }

    // Profile replay, one row per tick.
    if (state_ == ProfileState::Running && profile_) {
        float t = profile_->torque_nm(row_);
        if (exp_n_ != 0.0f && exp_wref_rpm_ > 0.0f)
            t *= powf(fabsf(w) / (exp_wref_rpm_ * RPM_TO_RAD_S), exp_n_);
        res += t > 0.0f ? t : 0.0f;
        if (profile_->has_inertia()) {
            out.profile_inertia = true;
            out.profile_inertia_kgm2 = profile_->inertia_kgm2(row_);
        }
        if (++row_ >= profile_->rows()) {
            if (loop_) row_ = 0;
            else state_ = ProfileState::Finished;
        }
    }

    // Resistive loads only oppose rotation, so their sum cannot go negative.
    out.tick.t_res_nm = res > 0.0f ? res : 0.0f;
    out.tick.t_active_nm = cfg_.grad_nm;
    out.inertia_delta_kgm2 = step_dj_in_force_;
    return out;
}

} // namespace loads
