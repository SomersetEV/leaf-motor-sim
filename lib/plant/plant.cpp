#include "plant.h"

#include <math.h>

namespace plant {

static constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * 3.14159265358979f);

void Plant::set_params(const PlantParams &p) {
    p_ = p;
    if (p_.j_kgm2 < J_MIN) p_.j_kgm2 = J_MIN;
    if (p_.tau_ms < 0.0f) p_.tau_ms = 0.0f;
    if (p_.tmax_nm < 0.0f) p_.tmax_nm = 0.0f;
    if (p_.pmax_kw < 0.0f) p_.pmax_kw = 0.0f;
}

void Plant::reset() {
    w_ = t_cmd_ = t_motor_ = t_load_ = 0.0f;
    clipping_ = false;
}

float Plant::rpm() const { return w_ * RAD_S_TO_RPM; }

float Plant::torque_limit_nm(float w) const {
    float aw = fabsf(w);
    float pmax_w = p_.pmax_kw * 1000.0f;
    if (aw * p_.tmax_nm <= pmax_w) return p_.tmax_nm;
    return pmax_w / aw;
}

void Plant::substep(float t_req, const TickLoads &loads, float dt) {
    // Torque envelope
    float lim = torque_limit_nm(w_);
    clipping_ = fabsf(t_req) > lim;
    t_cmd_ = t_req > lim ? lim : (t_req < -lim ? -lim : t_req);

    // First-order torque lag, exact for a constant command over dt
    if (p_.tau_ms > 0.0f)
        t_motor_ += (t_cmd_ - t_motor_) * (1.0f - expf(-dt * 1000.0f / p_.tau_ms));
    else
        t_motor_ = t_cmd_;

    // Inertia, with friction able to hold the shaft at standstill (section 6)
    float aw      = fabsf(w_);
    float t_drive = t_motor_ - loads.t_active_nm;
    float t_res   = p_.tc_nm + p_.b_nms * aw + p_.c_nms2 * aw * aw + loads.t_res_nm;
    if (aw < W_EPS && fabsf(t_drive) <= t_res) {
        w_ = 0.0f;
        t_load_ = loads.t_active_nm + t_drive;  // friction balances the drive
    } else {
        float dir   = (aw >= W_EPS) ? copysignf(1.0f, w_) : copysignf(1.0f, t_drive);
        float w_new = w_ + (t_drive - dir * t_res) / p_.j_kgm2 * dt;
        if (aw >= W_EPS && w_new * w_ < 0.0f) w_new = 0.0f;  // friction cannot reverse the shaft
        w_ = w_new;
        t_load_ = loads.t_active_nm + dir * t_res;
    }
}

} // namespace plant
