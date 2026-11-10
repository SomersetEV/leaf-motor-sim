// Default model parameters from config.h, as a SimParams.
#pragma once

#include "config.h"
#include "sim_params.h"

inline SimParams default_sim_params() {
    SimParams p;
    p.plant = {J_DEFAULT_KGM2, TC_DEFAULT_NM, B_DEFAULT_NMS, C_DEFAULT_NMS2,
               TAU_DEFAULT_MS, TMAX_DEFAULT_NM, PMAX_DEFAULT_KW};
    p.k_t = K_T_DEFAULT;
    p.udc_v = UDC_DEFAULT_V;
    p.noise_rpm = 0.0f;
    p.motor_c = MOTOR_TEMP_DEFAULT_C;
    p.inv_c = INV_TEMP_DEFAULT_C;
    p.noise_fc_hz = NOISE_FC_DEFAULT_HZ;
    p.noise_bw_hz = NOISE_BW_DEFAULT_HZ;
    p.rptdelay_ms = 0.0f;
    return p;
}
