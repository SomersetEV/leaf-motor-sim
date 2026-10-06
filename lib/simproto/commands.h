// Console commands that change simulator state. The console parses them and
// queues a SimCmd; the 10 ms tick applies it at the start of its next tick,
// so the model never sees a half-applied change.
// See docs/SIMULATOR_PLAN.md section 7, "Serial console".
#pragma once

#include <stdint.h>

#include "presets.h"
#include "profile.h"

namespace simproto {

enum class CmdType : uint8_t {
    SetParam,      // i = parameter index, f[0] = value
    Preset,        // preset
    LoadStep,      // f = Nm, ramp ms, dJ
    LoadPulse,     // f = peak Nm, period ms, duty %
    LoadGrad,      // f[0] = Nm
    LoadNoise,     // f = sd Nm, corner Hz
    LoadOff,
    ProfileSet,    // profile (nullptr clears)
    ProfileStart,
    ProfileStop,
    ProfileLoop,   // i = 0/1
    ProfileExp,    // f = n, w_ref rpm
    Mark,          // i = marker number
    FaultInv,      // i = 0/1
};

struct SimCmd {
    CmdType type;
    int32_t i;
    float f[3];
    const loads::Profile *profile;
    simcore::Preset preset;
};

// Parses set, load, profile start/stop/loop/exp, mark and fault. Commands
// needing files (preset, profile load) are built by the console itself.
// Returns nullptr on success, otherwise an error message for an "err" reply.
const char *parse_sim_command(int argc, char *const argv[], SimCmd &out);

// Strict number parsing: the whole token must be a finite number.
bool parse_float(const char *s, float &out);
bool parse_int(const char *s, int32_t &out);

} // namespace simproto
