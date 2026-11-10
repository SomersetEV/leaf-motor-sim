#include "speed_report.h"

#include <math.h>

namespace sensor {

static constexpr float PI_F = 3.14159265358979f;

BandPass design_bandpass(float fc_hz, float bw_hz, float fs_hz) {
    BandPass bp;
    float w0 = 2.0f * PI_F * fc_hz / fs_hz;
    float q = fc_hz / bw_hz;
    float alpha = sinf(w0) / (2.0f * q);
    float a0 = 1.0f + alpha;
    bp.b0 = alpha / a0;
    bp.a1 = -2.0f * cosf(w0) / a0;
    bp.a2 = (1.0f - alpha) / a0;

    // Output variance for unit white input. The all-pole part
    // 1 / (1 + a1 z^-1 + a2 z^-2) has autocovariances g0, g1, g2; the
    // numerator b0 (1 - z^-2) then gives b0^2 (2 g0 - 2 g2).
    double a1 = bp.a1, a2 = bp.a2;
    double g0 = (1.0 + a2) / ((1.0 - a2) * ((1.0 + a2) * (1.0 + a2) - a1 * a1));
    double g1 = -a1 * g0 / (1.0 + a2);
    double g2 = -a1 * g1 - a2 * g0;
    double var = (double)bp.b0 * bp.b0 * 2.0 * (g0 - g2);
    bp.out_sd = var > 0.0 ? (float)sqrt(var) : 1.0f;
    return bp;
}

SpeedReport::SpeedReport(float sample_ms, float tick_s, uint32_t seed)
    : sample_ms_(sample_ms > 0.0f ? sample_ms : 1.0f),
      fs_hz_(tick_s > 0.0f ? 1.0f / tick_s : 100.0f),
      gauss_(seed) {
    fill(0.0f);
}

void SpeedReport::reseed(uint32_t seed) {
    gauss_ = loads::Gauss(seed);
    x1_ = x2_ = y1_ = y2_ = 0.0f;
}

void SpeedReport::fill(float rpm) {
    for (float &h : hist_) h = rpm;
}

void SpeedReport::push(float rpm) {
    head_ = (head_ + 1) & (HISTORY - 1);
    hist_[head_] = rpm;
}

float SpeedReport::delayed_rpm(float delay_ms) const {
    if (!(delay_ms > 0.0f)) return hist_[head_];
    if (delay_ms > DELAY_MAX_MS) delay_ms = DELAY_MAX_MS;
    uint32_t n = (uint32_t)lroundf(delay_ms / sample_ms_);
    if (n > HISTORY - 1) n = HISTORY - 1;
    return hist_[(head_ - n) & (HISTORY - 1)];
}

float SpeedReport::noise(const ReportParams &p) {
    if (!(p.noise_rpm > 0.0f)) return 0.0f;
    float x = gauss_.next();
    if (!(p.noise_fc_hz > 0.0f) || !(p.noise_bw_hz > 0.0f) || p.noise_fc_hz >= 0.5f * fs_hz_)
        return p.noise_rpm * x;

    if (p.noise_fc_hz != bp_fc_ || p.noise_bw_hz != bp_bw_) {
        bp_ = design_bandpass(p.noise_fc_hz, p.noise_bw_hz, fs_hz_);
        bp_fc_ = p.noise_fc_hz;
        bp_bw_ = p.noise_bw_hz;
    }
    float y = bp_.b0 * (x - x2_) - bp_.a1 * y1_ - bp_.a2 * y2_;
    x2_ = x1_;
    x1_ = x;
    y2_ = y1_;
    y1_ = y;
    return p.noise_rpm * y / bp_.out_sd;
}

} // namespace sensor
