// Tests for lib/simengine, the tick shared by the firmware and the PC harness.
#include <math.h>
#include <string.h>
#include <unity.h>

#include "config.h"
#include "param_table.h"
#include "sim_engine.h"
#include "telemetry.h"

using simengine::Engine;

struct Frame {
    uint32_t id;
    uint8_t d[8];
};

class Recorder : public simengine::FrameSink {
public:
    static constexpr int MAX = 64;
    Frame f[MAX];
    int n = 0;
    void send(uint32_t id, const uint8_t *data, uint8_t dlc) override {
        if (n < MAX) {
            f[n].id = id;
            memcpy(f[n].d, data, dlc > 8 ? 8 : dlc);
            n++;
        }
    }
    int count(uint32_t id) const {
        int c = 0;
        for (int i = 0; i < n; i++) c += f[i].id == id;
        return c;
    }
    const Frame *last(uint32_t id) const {
        for (int i = n - 1; i >= 0; i--)
            if (f[i].id == id) return &f[i];
        return nullptr;
    }
};

static simengine::Config cfg() {
    return simengine::Config{TICK_MS, SUBSTEPS_PER_TICK, SLOW_PERIOD_TICKS, LOAD_NOISE_SEED,
                             SPEED_NOISE_SEED, true, ID_LEAF_INV_STATUS, ID_LEAF_INV_TEMPS,
                             ID_SIM_FAST, ID_SIM_SLOW, ID_SIM_EVENT};
}

static SimParams defaults() {
    return SimParams{{J_DEFAULT_KGM2, TC_DEFAULT_NM, B_DEFAULT_NMS, C_DEFAULT_NMS2, TAU_DEFAULT_MS,
                      TMAX_DEFAULT_NM, PMAX_DEFAULT_KW},
                     K_T_DEFAULT, UDC_DEFAULT_V, 0.0f, MOTOR_TEMP_DEFAULT_C, INV_TEMP_DEFAULT_C,
                     NOISE_FC_DEFAULT_HZ, NOISE_BW_DEFAULT_HZ, 0.0f};
}

static simengine::TickIn input(uint32_t now_ms, bool fresh, int16_t raw) {
    simengine::TickIn in{};
    in.now_ms = now_ms;
    in.fresh = fresh;
    in.cmd.raw = fresh ? raw : 0;
    return in;
}

// Zombie decode of the 0x1DA speed (plan section 5)
static int16_t speed_1da(const uint8_t *b) {
    int16_t s = (b[4] << 7) | b[5] >> 1;
    if (s > 0x3fff) s -= 0x7fff;
    return s;
}

void setUp() {}
void tearDown() {}

static void test_frames_each_tick_and_slow_period() {
    Engine e(cfg(), defaults());
    for (uint32_t k = 0; k < 21; k++) {
        Recorder r;
        e.tick(input(k * 10, true, 0), r);
        TEST_ASSERT_EQUAL_INT(1, r.count(ID_LEAF_INV_STATUS));
        TEST_ASSERT_EQUAL_INT(1, r.count(ID_SIM_FAST));
        int slow = (k % 10 == 0) ? 1 : 0;
        TEST_ASSERT_EQUAL_INT(slow, r.count(ID_LEAF_INV_TEMPS));
        TEST_ASSERT_EQUAL_INT(slow, r.count(ID_SIM_SLOW));
    }
    TEST_ASSERT_EQUAL_UINT32(21, e.ticks());
}

static void test_request_drives_the_model_and_timeout_zeroes_it() {
    Engine e(cfg(), defaults());
    Recorder r;
    simengine::TickOut o{};
    for (int k = 0; k < 50; k++) o = e.tick(input(k * 10, true, 400), r);  // 100 Nm
    TEST_ASSERT_EQUAL_FLOAT(400 * K_T_DEFAULT, o.t_req_nm);
    TEST_ASSERT_TRUE(o.rpm > 100.0f);
    TEST_ASSERT_EQUAL_UINT8(0, o.flags & simproto::FLAG_RX_TIMEOUT);
    Recorder r2;
    o = e.tick(input(500, false, 400), r2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.t_req_nm);
    TEST_ASSERT_TRUE(o.flags & simproto::FLAG_RX_TIMEOUT);
    // 0x1DA carries the model speed to the nearest rpm when there is no noise
    const Frame *f = r2.last(ID_LEAF_INV_STATUS);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_INT_WITHIN(1, (int)lroundf(o.rpm), speed_1da(f->d));
}

static void test_set_param_sends_event_and_takes_effect() {
    Engine e(cfg(), defaults());
    simproto::SimCmd c{};
    c.type = simproto::CmdType::SetParam;
    c.i = simcore::param_index("rptdelay");
    c.f[0] = 20.0f;
    Recorder r;
    e.apply(c, 1234, r);
    TEST_ASSERT_EQUAL_INT(1, r.count(ID_SIM_EVENT));
    TEST_ASSERT_EQUAL_UINT8(simproto::EV_PARAM, r.f[0].d[0]);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, e.params().rptdelay_ms);
    // An out-of-range value is not taken and sends nothing
    c.f[0] = 99.0f;
    Recorder r2;
    e.apply(c, 1244, r2);
    TEST_ASSERT_EQUAL_INT(0, r2.n);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, e.params().rptdelay_ms);
}

// Mark restarts the speed noise, so two engines that ran different lengths
// before the mark report the same noise after it (plan item I5).
static void test_mark_restarts_the_noise() {
    SimParams p = defaults();
    p.noise_rpm = 10.0f;
    Engine a(cfg(), p), b(cfg(), p);
    Recorder r;
    for (int k = 0; k < 7; k++) a.tick(input(k * 10, true, 0), r);
    for (int k = 0; k < 19; k++) b.tick(input(k * 10, true, 0), r);
    simproto::SimCmd m{};
    m.type = simproto::CmdType::Mark;
    m.i = 3;
    a.apply(m, 0, r);
    b.apply(m, 0, r);
    int differ = 0;
    for (int k = 0; k < 30; k++) {
        simengine::TickOut oa = a.tick(input(0, true, 0), r);
        simengine::TickOut ob = b.tick(input(0, true, 0), r);
        differ += oa.rpm_reported != ob.rpm_reported;
    }
    TEST_ASSERT_EQUAL_INT(0, differ);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_frames_each_tick_and_slow_period);
    RUN_TEST(test_request_drives_the_model_and_timeout_zeroes_it);
    RUN_TEST(test_set_param_sends_event_and_takes_effect);
    RUN_TEST(test_mark_restarts_the_noise);
    return UNITY_END();
}
