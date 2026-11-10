// Simulator model parameters, settable from the console and saved to NVS.
// See docs/SIMULATOR_PLAN.md section 6, "Model parameters".
#pragma once

#include "plant.h"

struct SimParams {
    plant::PlantParams plant;
    float k_t;        // Nm per 0x1D4 request bit
    float udc_v;      // reported bus voltage
    float noise_rpm;  // speed noise on the reported rpm only, standard deviation
    float motor_c;    // reported motor temperature
    float inv_c;      // reported inverter temperature
    // Shape and delay of the reported speed (lib/sensor, plan items I6 and I7)
    float noise_fc_hz;  // speed noise band centre; 0 = white per tick
    float noise_bw_hz;  // speed noise bandwidth, -3 dB
    float rptdelay_ms;  // extra delay on the speed in 0x1DA
};
