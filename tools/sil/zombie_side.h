// The Zombie half of the software-in-the-loop harness (plan section 16).
//
// zombie_side.cpp is compiled against the Zombie tree (Stm32-vcu include/,
// libopeninv/include) and linked with the Zombie's own throttle.cpp,
// govlog.cpp and params.cpp. This header has no Zombie includes, so the
// harness's main file can include it next to this repo's headers.
#pragma once

#include <stdint.h>

namespace zside {

// Loads the Zombie's parameter defaults, then the tractor's governor setup
// (CLAUDE.md, 3 Nov 2026) and run mode. See zombie_side.cpp for the list.
void init();

// Writes a parameter by name with the raw fixed-point value an SDO write
// carries (value x 32), then runs the Zombie's parameter-change mapping.
// Returns nullptr, or an error for an unknown name or out-of-range value.
const char *set_raw(const char *name, int32_t raw);
bool get(const char *name, float &value);

struct Ms10Out {
    uint8_t f1d4[8];    // 0x1D4 as NissLeafMng::Task10Ms builds it
    bool    govlog;     // GovLog on: 0x4A0 and 0x4A1 were built
    bool    slow;       // 0x4A2 and 0x4A3 were built too
    uint8_t f4a0[8], f4a1[8], f4a2[8], f4a3[8];
    float   potnom;     // percent, after the governor and the ramp
};

// One Ms10Task in run mode with the pedal at pedal_pct, using the speed the
// previous call read. Ends, as the Zombie does, by reading the motor speed
// from the latest 0x1DA passed to receive_1da().
Ms10Out ms10(float pedal_pct);

// LeafINV::DecodeCAN for 0x1DA: stores the speed for the next read.
void receive_1da(const uint8_t d[8]);

// Runs the Zombie's own governor golden test inputs (test/test_throttle.cpp,
// GoldenInput) through SpeedLimitCommand as linked here, against
// test/golden_governor.h. Must run before any other governor call in the
// process. Returns the number of mismatching ticks (first in *first).
int golden_mismatches(int *first);

} // namespace zside
