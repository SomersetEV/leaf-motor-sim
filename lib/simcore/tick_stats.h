// Timing statistics for the 10 ms simulation tick.
// See docs/SIMULATOR_PLAN.md section 7: period within 10 ms +/- 0.5 ms and
// zero overruns, with worst jitter reported per one-second window.
//
// Plain C++. Written only by the tick; readers take a copy under a lock.
#pragma once

#include <stdint.h>

namespace simcore {

class TickStats {
public:
    TickStats(uint32_t nominal_us = 10000, uint32_t tolerance_us = 500, uint32_t window_ticks = 100)
        : nominal_us_(nominal_us), tolerance_us_(tolerance_us), window_ticks_(window_ticks) {}

    // period_us: time since the previous tick started.
    // overrun: the tick missed its deadline (the delay did not block).
    void record(uint32_t period_us, bool overrun);

    uint32_t ticks() const { return ticks_; }
    uint32_t last_period_us() const { return last_period_us_; }
    uint32_t overruns() const { return overruns_; }
    uint32_t out_of_tolerance() const { return out_of_tol_; }
    uint32_t worst_jitter_us() const { return worst_total_us_; }
    // Worst jitter over the last completed window (one second at 100 ticks).
    uint32_t window_worst_jitter_us() const { return window_published_us_; }
    // Overruns or out-of-tolerance ticks in the last completed window.
    bool window_bad() const { return window_bad_published_; }

private:
    uint32_t nominal_us_;
    uint32_t tolerance_us_;
    uint32_t window_ticks_;

    uint32_t ticks_ = 0;
    uint32_t last_period_us_ = 0;
    uint32_t overruns_ = 0;
    uint32_t out_of_tol_ = 0;
    uint32_t worst_total_us_ = 0;

    uint32_t window_count_ = 0;
    uint32_t window_worst_us_ = 0;
    bool     window_bad_ = false;
    uint32_t window_published_us_ = 0;
    bool     window_bad_published_ = false;
};

} // namespace simcore
