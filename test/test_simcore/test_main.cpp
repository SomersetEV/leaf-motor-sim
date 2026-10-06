// Tests for lib/simcore: 0x1D4 latch with receive timeout, and tick statistics.

#include <stdint.h>
#include <unity.h>

#include "cmd_latch.h"
#include "config.h"
#include "leafcodec.h"
#include "tick_stats.h"

using namespace simcore;

void setUp() {}
void tearDown() {}

// A valid 0x1D4 frame for torque request raw with HV on.
static void make_1d4(int16_t raw, uint8_t ctr, uint8_t b[8]) {
    b[0] = 0xF7;
    b[1] = 0x07;
    b[2] = (uint8_t)(((raw < 0) ? 0x80 : 0) | ((raw >> 4) & 0x7F));
    b[3] = (uint8_t)((raw << 4) & 0xF0);
    b[4] = (uint8_t)(leafcodec::HV_ON | (ctr << 6));
    b[5] = 0x44;
    b[6] = 0x30;
    b[7] = leafcodec::nissan_crc(b);
}

// ---- CmdLatch -------------------------------------------------------------

static void test_latch_empty_is_timed_out() {
    CmdLatch l(RX_TIMEOUT_MS);
    leafcodec::Cmd1D4 c;
    c.raw = 99;
    TEST_ASSERT_FALSE(l.current(0, c));
    TEST_ASSERT_EQUAL_INT16(0, c.raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.torque_nm);
}

static void test_latch_timeout_at_100ms() {
    CmdLatch l(RX_TIMEOUT_MS);
    uint8_t b[8];
    make_1d4(400, 1, b);
    TEST_ASSERT_TRUE(l.accept(b, 8, 1000, K_T_DEFAULT));
    leafcodec::Cmd1D4 c;
    TEST_ASSERT_TRUE(l.current(1000, c));
    TEST_ASSERT_EQUAL_INT16(400, c.raw);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, c.torque_nm);
    TEST_ASSERT_TRUE(l.current(1099, c));
    TEST_ASSERT_FALSE(l.current(1100, c));
    TEST_ASSERT_EQUAL_INT16(0, c.raw);
}

static void test_latch_timeout_across_ms_wrap() {
    CmdLatch l(RX_TIMEOUT_MS);
    uint8_t b[8];
    make_1d4(-200, 0, b);
    uint32_t t0 = 0xFFFFFFF0u;
    TEST_ASSERT_TRUE(l.accept(b, 8, t0, K_T_DEFAULT));
    leafcodec::Cmd1D4 c;
    TEST_ASSERT_TRUE(l.current(t0 + 50, c));  // wrapped past zero
    TEST_ASSERT_EQUAL_INT16(-200, c.raw);
    TEST_ASSERT_FALSE(l.current(t0 + 100, c));
}

static void test_latch_rejects_bad_crc_and_length() {
    CmdLatch l(RX_TIMEOUT_MS);
    uint8_t b[8];
    make_1d4(300, 2, b);
    TEST_ASSERT_TRUE(l.accept(b, 8, 0, K_T_DEFAULT));

    uint8_t bad[8];
    make_1d4(900, 3, bad);
    bad[7] ^= 0x01;
    TEST_ASSERT_FALSE(l.accept(bad, 8, 10, K_T_DEFAULT));
    make_1d4(900, 3, bad);
    TEST_ASSERT_FALSE(l.accept(bad, 7, 10, K_T_DEFAULT));

    TEST_ASSERT_EQUAL_UINT32(1, l.accepted());
    TEST_ASSERT_EQUAL_UINT32(2, l.rejected());
    // Rejected frames neither replace the command nor refresh the timeout.
    leafcodec::Cmd1D4 c;
    TEST_ASSERT_TRUE(l.current(99, c));
    TEST_ASSERT_EQUAL_INT16(300, c.raw);
    TEST_ASSERT_FALSE(l.current(100, c));
}

// ---- TickStats ------------------------------------------------------------

static void test_tick_stats_jitter_and_tolerance() {
    TickStats s(10000, 500, 100);
    s.record(10000, false);
    s.record(10300, false);
    s.record(9600, false);
    TEST_ASSERT_EQUAL_UINT32(3, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(400, s.worst_jitter_us());
    TEST_ASSERT_EQUAL_UINT32(0, s.out_of_tolerance());
    s.record(10501, false);
    TEST_ASSERT_EQUAL_UINT32(1, s.out_of_tolerance());
    TEST_ASSERT_EQUAL_UINT32(501, s.worst_jitter_us());
    TEST_ASSERT_EQUAL_UINT32(10501, s.last_period_us());
}

static void test_tick_stats_overrun_counted() {
    TickStats s(10000, 500, 100);
    s.record(10000, true);
    TEST_ASSERT_EQUAL_UINT32(1, s.overruns());
    TEST_ASSERT_EQUAL_UINT32(0, s.out_of_tolerance());
}

static void test_tick_stats_window_published_and_reset() {
    TickStats s(10000, 500, 100);
    // Nothing published until the first window completes.
    for (int i = 0; i < 99; i++) s.record(i == 10 ? 10700 : 10000, false);
    TEST_ASSERT_EQUAL_UINT32(0, s.window_worst_jitter_us());
    TEST_ASSERT_FALSE(s.window_bad());
    s.record(10000, false);
    TEST_ASSERT_EQUAL_UINT32(700, s.window_worst_jitter_us());
    TEST_ASSERT_TRUE(s.window_bad());
    // A clean second window replaces it; the all-time worst is kept.
    for (int i = 0; i < 100; i++) s.record(10050, false);
    TEST_ASSERT_EQUAL_UINT32(50, s.window_worst_jitter_us());
    TEST_ASSERT_FALSE(s.window_bad());
    TEST_ASSERT_EQUAL_UINT32(700, s.worst_jitter_us());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_latch_empty_is_timed_out);
    RUN_TEST(test_latch_timeout_at_100ms);
    RUN_TEST(test_latch_timeout_across_ms_wrap);
    RUN_TEST(test_latch_rejects_bad_crc_and_length);
    RUN_TEST(test_tick_stats_jitter_and_tolerance);
    RUN_TEST(test_tick_stats_overrun_counted);
    RUN_TEST(test_tick_stats_window_published_and_reset);
    return UNITY_END();
}
