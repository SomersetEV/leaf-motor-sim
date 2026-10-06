// Holds the latest valid 0x1D4 torque request and applies the receive timeout.
// See docs/SIMULATOR_PLAN.md section 5.
//
// Plain C++. Not thread safe: the caller guards it (src/can_io.cpp).
#pragma once

#include <stdint.h>

#include "leafcodec.h"

namespace simcore {

class CmdLatch {
public:
    explicit CmdLatch(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

    // Decodes one received 0x1D4 frame. Frames with the wrong length or a bad
    // CRC are ignored and counted, as a real inverter would do.
    // Returns true if the frame was accepted.
    bool accept(const uint8_t *data, uint8_t dlc, uint32_t now_ms, float k_t);

    // Copies the latest accepted command into out. Returns false, with out
    // holding zero torque, if nothing valid arrived within the timeout.
    bool current(uint32_t now_ms, leafcodec::Cmd1D4 &out) const;

    uint32_t accepted() const { return accepted_; }
    uint32_t rejected() const { return rejected_; }

private:
    uint32_t timeout_ms_;
    leafcodec::Cmd1D4 latest_{};
    uint32_t latest_ms_ = 0;
    bool     have_ = false;
    uint32_t accepted_ = 0;
    uint32_t rejected_ = 0;
};

} // namespace simcore
