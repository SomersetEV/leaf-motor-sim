// Tests for lib/sensor: report delay (plan item I7) and the speed noise,
// white or band-limited (plan item I6).

#include <math.h>
#include <unity.h>

#include "speed_report.h"

using namespace sensor;

static constexpr float SUB_MS = 1.0f;   // model sub-step
static constexpr float TICK_S = 0.01f;  // one 0x1DA frame
static constexpr float PI_F = 3.14159265358979f;

void setUp() {}
void tearDown() {}

static void push_ramp(SpeedReport &r, int n) {
    for (int i = 1; i <= n; i++) r.push((float)i);  // newest sample is n
}

static void test_delay_picks_the_sample_that_many_ms_ago() {
    SpeedReport r(SUB_MS, TICK_S, 1);
    push_ramp(r, 100);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.delayed_rpm(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(93.0f, r.delayed_rpm(7.0f));
    TEST_ASSERT_EQUAL_FLOAT(98.0f, r.delayed_rpm(2.4f));   // nearest sample
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.delayed_rpm(50.0f));
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.delayed_rpm(80.0f));  // clamped to DELAY_MAX_MS
    TEST_ASSERT_EQUAL_FLOAT(100.0f, r.delayed_rpm(NAN));
}

static void test_delay_history_starts_filled() {
    SpeedReport r(SUB_MS, TICK_S, 1);
    r.fill(1500.0f);
    r.push(1501.0f);
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, r.delayed_rpm(30.0f));
    ReportParams p;  // no noise
    p.delay_ms = 1.0f;
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, r.report(p));
    p.delay_ms = 0.0f;
    TEST_ASSERT_EQUAL_FLOAT(1501.0f, r.report(p));
}

static void test_no_noise_by_default() {
    SpeedReport r(SUB_MS, TICK_S, 1);
    ReportParams p;
    for (int i = 0; i < 100; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, r.noise(p));
}

// Mean and standard deviation of n samples of noise(p)
static void noise_stats(SpeedReport &r, const ReportParams &p, int n, float &mean, float &sd) {
    double s = 0.0, s2 = 0.0;
    for (int i = 0; i < n; i++) {
        double x = r.noise(p);
        s += x;
        s2 += x * x;
    }
    mean = (float)(s / n);
    sd = (float)sqrt(s2 / n - (s / n) * (s / n));
}

static void test_white_noise_has_the_set_sd() {
    SpeedReport r(SUB_MS, TICK_S, 0x5EED1DA1u);
    ReportParams p;
    p.noise_rpm = 10.0f;
    float mean, sd;
    noise_stats(r, p, 40000, mean, sd);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, mean);
    TEST_ASSERT_FLOAT_WITHIN(0.25f, 10.0f, sd);
}

static void test_bandpass_sd_formula_matches_impulse_response() {
    const float cases[][2] = {{7.0f, 4.0f}, {2.0f, 0.5f}, {20.0f, 30.0f}, {45.0f, 5.0f}};
    for (const auto &c : cases) {
        BandPass bp = design_bandpass(c[0], c[1], 100.0f);
        double x1 = 0, x2 = 0, y1 = 0, y2 = 0, e = 0;
        for (int n = 0; n < 20000; n++) {
            double x = n == 0 ? 1.0 : 0.0;
            double y = bp.b0 * (x - x2) - bp.a1 * y1 - bp.a2 * y2;
            x2 = x1; x1 = x; y2 = y1; y1 = y;
            e += y * y;
        }
        TEST_ASSERT_FLOAT_WITHIN(0.002f * bp.out_sd, (float)sqrt(e), bp.out_sd);
    }
}

// Power of x at f_hz (single-bin DFT), for a 100 Hz sample rate
static float power_at(const float *x, int n, float f_hz) {
    double re = 0, im = 0;
    for (int i = 0; i < n; i++) {
        double a = 2.0 * PI_F * f_hz * i * TICK_S;
        re += x[i] * cos(a);
        im -= x[i] * sin(a);
    }
    return (float)((re * re + im * im) / n);
}

static void test_band_noise_has_the_set_sd_and_peaks_at_fc() {
    SpeedReport r(SUB_MS, TICK_S, 0x5EED1DA1u);
    ReportParams p;
    p.noise_rpm = 10.0f;
    p.noise_fc_hz = 7.0f;
    p.noise_bw_hz = 4.0f;
    float mean, sd;
    noise_stats(r, p, 40000, mean, sd);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, mean);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 10.0f, sd);

    // Averaged over many blocks, the spectrum at fc stands well above the
    // spectrum at 1 Hz and at 30 Hz, where white noise would be flat.
    static float x[512];
    double at_fc = 0, at_1 = 0, at_30 = 0;
    for (int b = 0; b < 40; b++) {
        for (float &v : x) v = r.noise(p);
        at_fc += power_at(x, 512, 7.0f);
        at_1 += power_at(x, 512, 1.0f);
        at_30 += power_at(x, 512, 30.0f);
    }
    TEST_ASSERT_TRUE(at_fc > 10.0 * at_1);
    TEST_ASSERT_TRUE(at_fc > 10.0 * at_30);
}

static void test_fc_at_or_above_nyquist_falls_back_to_white() {
    SpeedReport a(SUB_MS, TICK_S, 7), b(SUB_MS, TICK_S, 7);
    ReportParams pw, pb;
    pw.noise_rpm = pb.noise_rpm = 5.0f;
    pb.noise_fc_hz = 50.0f;
    for (int i = 0; i < 50; i++) {
        float wa = a.noise(pw);
        float wb = b.noise(pb);
        TEST_ASSERT_EQUAL_FLOAT(wa, wb);
    }
}

static void test_reseed_repeats_the_sequence() {
    SpeedReport r(SUB_MS, TICK_S, 0x1234u);
    ReportParams p;
    p.noise_rpm = 8.0f;
    p.noise_fc_hz = 7.0f;
    p.noise_bw_hz = 4.0f;
    float first[64];
    r.reseed(0xABCDu);
    for (float &v : first) v = r.noise(p);
    for (int i = 0; i < 1000; i++) r.noise(p);  // wander off
    r.reseed(0xABCDu);
    for (float v : first) {
        float again = r.noise(p);
        TEST_ASSERT_EQUAL_FLOAT(v, again);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_delay_picks_the_sample_that_many_ms_ago);
    RUN_TEST(test_delay_history_starts_filled);
    RUN_TEST(test_no_noise_by_default);
    RUN_TEST(test_white_noise_has_the_set_sd);
    RUN_TEST(test_bandpass_sd_formula_matches_impulse_response);
    RUN_TEST(test_band_noise_has_the_set_sd_and_peaks_at_fc);
    RUN_TEST(test_fc_at_or_above_nyquist_falls_back_to_white);
    RUN_TEST(test_reseed_repeats_the_sequence);
    return UNITY_END();
}
