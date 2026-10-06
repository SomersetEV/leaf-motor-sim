// Model parameters in NVS (the ESP32's flash key-value store).
#pragma once

#include "sim_params.h"

// Loads saved parameters into p. Returns false, leaving p unchanged, if none
// are saved or the saved set is from another layout or out of range.
bool params_load(SimParams &p);
bool params_save(const SimParams &p);
