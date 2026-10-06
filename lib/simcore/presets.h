// Presets: a named set of model parameters and load components.
// See docs/SIMULATOR_PLAN.md section 6, "Presets and profiles".
//
// Compiled-in presets noload, topper, baler and pto_step carry placeholder
// numbers until the identification runs replace them. A file
// /presets/<name>.json on the SD card overrides a compiled preset of the same
// name, or defines a new one; keys it leaves out keep their compiled (or base)
// values.
#pragma once

#include <stdint.h>

#include "loads.h"
#include "sim_params.h"

namespace simcore {

struct Preset {
    char name[16];
    uint8_t id;          // reported in 0x4B1; 0 = no preset applied
    SimParams params;
    loads::LoadConfig loads;
};

constexpr uint8_t PRESET_ID_FILE = 200;  // a preset that exists only on SD
constexpr int BUILTIN_PRESET_COUNT = 4;
extern const char *const BUILTIN_PRESET_NAMES[BUILTIN_PRESET_COUNT];

// base: the default parameters (config.h) the presets are built on.
bool builtin_preset(const char *name, const SimParams &base, Preset &out);

// Builds a preset from a JSON file's text. Starts from the compiled preset
// of the same name if there is one, else from base with no loads.
// Returns nullptr on success, otherwise an error message; bad_key (if not
// null) receives the offending key.
const char *preset_from_json(const char *name, const char *json, const SimParams &base,
                             Preset &out, char *bad_key = nullptr, int bad_key_len = 0);

} // namespace simcore
