#include "cmd_latch.h"

namespace simcore {

bool CmdLatch::accept(const uint8_t *data, uint8_t dlc, uint32_t now_ms, float k_t) {
    if (dlc != 8) {
        rejected_++;
        return false;
    }
    leafcodec::Cmd1D4 c = leafcodec::decode_1d4(data, k_t);
    if (!c.crc_ok) {
        rejected_++;
        return false;
    }
    latest_    = c;
    latest_ms_ = now_ms;
    have_      = true;
    accepted_++;
    return true;
}

bool CmdLatch::current(uint32_t now_ms, leafcodec::Cmd1D4 &out) const {
    // Unsigned subtraction handles the millisecond counter wrapping.
    if (!have_ || (uint32_t)(now_ms - latest_ms_) >= timeout_ms_) {
        out = leafcodec::Cmd1D4{};
        return false;
    }
    out = latest_;
    return true;
}

} // namespace simcore
