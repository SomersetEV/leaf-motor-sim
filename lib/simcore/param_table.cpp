#include "param_table.h"

#include <math.h>
#include <string.h>

namespace simcore {

// Names follow plan section 6; temperatures are extra, for the 0x55A frame.
const ParamInfo PARAMS[PARAM_COUNT] = {
    {"J",       "kg.m2",       0.001f, 50.0f},
    {"Tc",      "Nm",          0.0f,   500.0f},
    {"b",       "Nm.s/rad",    0.0f,   10.0f},
    {"c",       "Nm.s2/rad2",  0.0f,   1.0f},
    {"tau",     "ms",          0.0f,   1000.0f},
    {"Tmax",    "Nm",          0.0f,   1000.0f},
    {"Pmax",    "kW",          0.0f,   500.0f},
    {"kT",      "Nm/bit",      0.01f,  2.0f},
    {"udc",     "V",           0.0f,   510.0f},
    {"noise",   "rpm",         0.0f,   500.0f},
    {"motor_c", "degC",        -40.0f, 150.0f},
    {"inv_c",   "degC",        -40.0f, 150.0f},
};

int param_index(const char *name) {
    for (int i = 0; i < PARAM_COUNT; i++)
        if (strcmp(PARAMS[i].name, name) == 0) return i;
    return -1;
}

float &param_ref(SimParams &p, int index) {
    switch (index) {
    case 0:  return p.plant.j_kgm2;
    case 1:  return p.plant.tc_nm;
    case 2:  return p.plant.b_nms;
    case 3:  return p.plant.c_nms2;
    case 4:  return p.plant.tau_ms;
    case 5:  return p.plant.tmax_nm;
    case 6:  return p.plant.pmax_kw;
    case 7:  return p.k_t;
    case 8:  return p.udc_v;
    case 9:  return p.noise_rpm;
    case 10: return p.motor_c;
    default: return p.inv_c;
    }
}

float param_get(const SimParams &p, int index) {
    return param_ref(const_cast<SimParams &>(p), index);
}

bool param_set(SimParams &p, int index, float value) {
    if (index < 0 || index >= PARAM_COUNT) return false;
    if (!(value >= PARAMS[index].min && value <= PARAMS[index].max)) return false;  // also rejects NaN
    param_ref(p, index) = value;
    return true;
}

bool params_valid(const SimParams &p) {
    for (int i = 0; i < PARAM_COUNT; i++) {
        float v = param_get(p, i);
        if (!(v >= PARAMS[i].min && v <= PARAMS[i].max)) return false;
    }
    return true;
}

} // namespace simcore
