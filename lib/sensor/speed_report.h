// The speed the inverter reports in 0x1DA: the model's shaft speed, delayed
// and with measurement noise added. See docs/SIMULATOR_PLAN.md section 15
// (items I6 and I7).
//
// The noise is white per tick when noise_fc_hz is 0. Otherwise it is white
// noise through a two-pole band-pass centred on noise_fc_hz with a -3 dB
// bandwidth of noise_bw_hz, which is the shape of the speed ripple in the
// tractor logs of 7 Nov 2026 (tools/fit_noise.py). Either way noise_rpm is
// the standard deviation of what is added. The noise never touches the model.
//
// tools/simmodel.py mirrors the band-pass and its scaling.
//
// Plain C++, no heap.
#pragma once

#include <stdint.h>

#include "gauss.h"

namespace sensor {

struct ReportParams {
    float noise_rpm   = 0.0f;  // standard deviation of the added noise
    float noise_fc_hz = 0.0f;  // band centre; 0 = white per tick
    float noise_bw_hz = 4.0f;  // -3 dB bandwidth when noise_fc_hz > 0
    float delay_ms    = 0.0f;  // extra delay on the reported speed
};

// Band-pass biquad (RBJ cookbook, 0 dB peak gain), normalised so a0 = 1:
// y[n] = b0 (x[n] - x[n-2]) - a1 y[n-1] - a2 y[n-2].
struct BandPass {
    float b0 = 0.0f, a1 = 0.0f, a2 = 0.0f;
    float out_sd = 1.0f;  // output standard deviation for unit white input
};

// fc_hz must be above 0 and below fs_hz / 2; bw_hz above 0.
BandPass design_bandpass(float fc_hz, float bw_hz, float fs_hz);

class SpeedReport {
public:
    static constexpr int   HISTORY      = 64;     // samples kept, power of two
    static constexpr float DELAY_MAX_MS = 50.0f;  // plan item I7

    // sample_ms: time between push() calls (the model sub-step).
    // tick_s: time between report() calls (one 0x1DA frame).
    SpeedReport(float sample_ms, float tick_s, uint32_t seed);

    // Restarts the noise sequence from seed and clears the band-pass state,
    // so a run can be repeated exactly. The delay history is kept.
    void reseed(uint32_t seed);

    // Fills the delay history with rpm, e.g. after a model reset.
    void fill(float rpm);

    // Records the model speed; call once per sub-step.
    void push(float rpm);

    // Speed delay_ms ago, to the nearest sample, at most DELAY_MAX_MS.
    float delayed_rpm(float delay_ms) const;

    // One noise sample; call once per tick.
    float noise(const ReportParams &p);

    // Delayed speed plus noise; call once per tick, after the sub-steps.
    float report(const ReportParams &p) { return delayed_rpm(p.delay_ms) + noise(p); }

private:
    float sample_ms_;
    float fs_hz_;
    loads::Gauss gauss_;
    float hist_[HISTORY];
    uint32_t head_ = 0;  // index of the newest sample

    BandPass bp_;
    float bp_fc_ = -1.0f, bp_bw_ = -1.0f;  // design the band-pass was made for
    float x1_ = 0.0f, x2_ = 0.0f, y1_ = 0.0f, y2_ = 0.0f;
};

} // namespace sensor
