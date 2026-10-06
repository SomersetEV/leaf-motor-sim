#include "tick_stats.h"

namespace simcore {

void TickStats::record(uint32_t period_us, bool overrun) {
    uint32_t jitter = period_us > nominal_us_ ? period_us - nominal_us_
                                              : nominal_us_ - period_us;
    bool bad = overrun || jitter > tolerance_us_;

    ticks_++;
    last_period_us_ = period_us;
    if (overrun) overruns_++;
    if (jitter > tolerance_us_) out_of_tol_++;
    if (jitter > worst_total_us_) worst_total_us_ = jitter;

    if (jitter > window_worst_us_) window_worst_us_ = jitter;
    if (bad) window_bad_ = true;
    if (++window_count_ >= window_ticks_) {
        window_published_us_  = window_worst_us_;
        window_bad_published_ = window_bad_;
        window_count_    = 0;
        window_worst_us_ = 0;
        window_bad_      = false;
    }
}

} // namespace simcore
