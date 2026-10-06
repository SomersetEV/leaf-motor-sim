// Named, range-checked access to SimParams for the console (set, params)
// and preset files.
#pragma once

#include "sim_params.h"

namespace simcore {

struct ParamInfo {
    const char *name;
    const char *unit;
    float min;
    float max;
};

constexpr int PARAM_COUNT = 12;
extern const ParamInfo PARAMS[PARAM_COUNT];

// Index of the parameter called name (case-sensitive), or -1.
int param_index(const char *name);

float &param_ref(SimParams &p, int index);
float param_get(const SimParams &p, int index);

// Sets parameter index to value if it is within range.
bool param_set(SimParams &p, int index, float value);

// True if every parameter is within range (e.g. after loading from NVS).
bool params_valid(const SimParams &p);

} // namespace simcore
