// Tests for lib/simcore: 0x1D4 latch with receive timeout, and tick statistics.

#include <stdint.h>
#include <unity.h>

#include <math.h>
#include <string.h>

#include "cmd_latch.h"
#include "config.h"
#include "json_flat.h"
#include "leafcodec.h"
#include "param_table.h"
#include "presets.h"
#include "spsc_ring.h"
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

// ---- Parameter table -------------------------------------------------------

static SimParams defaults() {
    return SimParams{{J_DEFAULT_KGM2, TC_DEFAULT_NM, B_DEFAULT_NMS, C_DEFAULT_NMS2, TAU_DEFAULT_MS,
                      TMAX_DEFAULT_NM, PMAX_DEFAULT_KW},
                     K_T_DEFAULT, UDC_DEFAULT_V, 0.0f, MOTOR_TEMP_DEFAULT_C, INV_TEMP_DEFAULT_C};
}

static void test_param_table_names_and_set() {
    SimParams p = defaults();
    TEST_ASSERT_TRUE(params_valid(p));
    int j = param_index("J");
    TEST_ASSERT_EQUAL_INT(0, j);
    TEST_ASSERT_EQUAL_FLOAT(J_DEFAULT_KGM2, param_get(p, j));
    TEST_ASSERT_TRUE(param_set(p, j, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, p.plant.j_kgm2);
    TEST_ASSERT_FALSE(param_set(p, j, 0.0f));   // below min
    TEST_ASSERT_FALSE(param_set(p, j, NAN));
    TEST_ASSERT_EQUAL_INT(-1, param_index("j"));  // case-sensitive
    // Every name in the table maps to a distinct field
    for (int i = 0; i < PARAM_COUNT; i++) {
        SimParams q = defaults();
        float probe = PARAMS[i].min + (PARAMS[i].max - PARAMS[i].min) * 0.37f;
        TEST_ASSERT_TRUE(param_set(q, i, probe));
        for (int k = 0; k < PARAM_COUNT; k++)
            if (k != i) TEST_ASSERT_EQUAL_FLOAT(param_get(defaults(), k), param_get(q, k));
        TEST_ASSERT_EQUAL_INT(i, param_index(PARAMS[i].name));
    }
}

// ---- JSON and presets ------------------------------------------------------

struct Collected {
    int n;
    char keys[8][32];
    float vals[8];
};

static bool collect(const char *k, float v, void *ctx) {
    Collected &c = *(Collected *)ctx;
    strncpy(c.keys[c.n], k, 31);
    c.vals[c.n++] = v;
    return true;
}

static void test_json_flat() {
    Collected c{};
    TEST_ASSERT_NULL(json_flat_parse(" { \"J\": 0.9,\n \"pulse_nm\" : -1.5e2 } \n", collect, &c));
    TEST_ASSERT_EQUAL_INT(2, c.n);
    TEST_ASSERT_EQUAL_STRING("pulse_nm", c.keys[1]);
    TEST_ASSERT_EQUAL_FLOAT(-150.0f, c.vals[1]);
    Collected e{};
    TEST_ASSERT_NULL(json_flat_parse("{}", collect, &e));
    TEST_ASSERT_NOT_NULL(json_flat_parse("{\"a\": \"text\"}", collect, &e));
    TEST_ASSERT_NOT_NULL(json_flat_parse("{\"a\": 1,}", collect, &e));
    TEST_ASSERT_NOT_NULL(json_flat_parse("{\"a\": {\"b\": 1}}", collect, &e));
    TEST_ASSERT_NOT_NULL(json_flat_parse("[1]", collect, &e));
    TEST_ASSERT_NOT_NULL(json_flat_parse("{\"a\": 1} x", collect, &e));
}

static void test_builtin_presets() {
    SimParams base = defaults();
    Preset p;
    for (int i = 0; i < BUILTIN_PRESET_COUNT; i++) {
        TEST_ASSERT_TRUE(builtin_preset(BUILTIN_PRESET_NAMES[i], base, p));
        TEST_ASSERT_EQUAL_UINT8(i + 1, p.id);
        TEST_ASSERT_TRUE(params_valid(p.params));
    }
    TEST_ASSERT_TRUE(builtin_preset("baler", base, p));
    TEST_ASSERT_TRUE(p.loads.pulse_on);
    TEST_ASSERT_FALSE(builtin_preset("nonsense", base, p));
}

static void test_preset_json_overrides() {
    SimParams base = defaults();
    Preset p;
    // Overrides one field of a compiled preset; the rest stays compiled.
    TEST_ASSERT_NULL(preset_from_json("baler", "{\"pulse_nm\": 200, \"J\": 1.2}", base, p));
    TEST_ASSERT_EQUAL_UINT8(3, p.id);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, p.loads.pulse_nm);
    TEST_ASSERT_EQUAL_FLOAT(700.0f, p.loads.pulse_period_ms);
    TEST_ASSERT_EQUAL_FLOAT(1.2f, p.params.plant.j_kgm2);
    // A new name starts from base with no loads
    TEST_ASSERT_NULL(preset_from_json("mower", "{\"c\": 0.0005, \"noise_nm\": 2}", base, p));
    TEST_ASSERT_EQUAL_UINT8(PRESET_ID_FILE, p.id);
    TEST_ASSERT_EQUAL_STRING("mower", p.name);
    TEST_ASSERT_TRUE(p.loads.noise_on);
    TEST_ASSERT_FALSE(p.loads.pulse_on);
    TEST_ASSERT_EQUAL_FLOAT(TC_DEFAULT_NM, p.params.plant.tc_nm);
    // Errors name the key
    char bad[32];
    TEST_ASSERT_EQUAL_STRING("unknown key",
                             preset_from_json("x", "{\"Jx\": 1}", base, p, bad, sizeof(bad)));
    TEST_ASSERT_EQUAL_STRING("Jx", bad);
    TEST_ASSERT_EQUAL_STRING("value out of range",
                             preset_from_json("x", "{\"J\": 0}", base, p, bad, sizeof(bad)));
    TEST_ASSERT_EQUAL_STRING("J", bad);
}

// ---- Ring buffer -----------------------------------------------------------

static void test_spsc_ring() {
    SpscRing<int, 4> r;
    int v;
    TEST_ASSERT_FALSE(r.pop(v));
    for (int i = 0; i < 4; i++) TEST_ASSERT_TRUE(r.push(i));
    TEST_ASSERT_FALSE(r.push(99));  // full: dropped and counted
    TEST_ASSERT_EQUAL_UINT32(1, r.dropped());
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(r.pop(v));
        TEST_ASSERT_EQUAL_INT(i, v);
    }
    TEST_ASSERT_FALSE(r.pop(v));
    // Indices keep working past wrap of the buffer
    for (int i = 0; i < 1000; i++) {
        TEST_ASSERT_TRUE(r.push(i));
        TEST_ASSERT_TRUE(r.pop(v));
        TEST_ASSERT_EQUAL_INT(i, v);
    }
    r.push(1);
    r.push(2);
    r.clear();
    TEST_ASSERT_FALSE(r.pop(v));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_param_table_names_and_set);
    RUN_TEST(test_json_flat);
    RUN_TEST(test_builtin_presets);
    RUN_TEST(test_preset_json_overrides);
    RUN_TEST(test_spsc_ring);
    RUN_TEST(test_latch_empty_is_timed_out);
    RUN_TEST(test_latch_timeout_at_100ms);
    RUN_TEST(test_latch_timeout_across_ms_wrap);
    RUN_TEST(test_latch_rejects_bad_crc_and_length);
    RUN_TEST(test_tick_stats_jitter_and_tolerance);
    RUN_TEST(test_tick_stats_overrun_counted);
    RUN_TEST(test_tick_stats_window_published_and_reset);
    return UNITY_END();
}
