// Tests for lib/plant against closed-form results.

#include <math.h>
#include <unity.h>

#include "plant.h"

using namespace plant;

static constexpr float DT = 0.001f;  // 1 ms sub-step, as in the firmware
static constexpr float RPM_PER_RAD_S = 60.0f / (2.0f * 3.14159265358979f);

void setUp() {}
void tearDown() {}

// No friction, no lag, large limits: a pure inertia.
static PlantParams ideal(float j) {
    return PlantParams{j, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 1000.0f};
}

static void run(Plant &p, float t_req, float seconds, const TickLoads &l = {}) {
    int n = (int)lroundf(seconds / DT);
    for (int i = 0; i < n; i++) p.substep(t_req, l, DT);
}

static void test_pure_inertia_acceleration() {
    Plant p(ideal(0.3f));
    run(p, 30.0f, 1.0f);  // 30 Nm / 0.3 kg·m² = 100 rad/s² for 1 s
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, p.w_rad_s());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f * RPM_PER_RAD_S, p.rpm());
    TEST_ASSERT_FALSE(p.clipping());
}

static void test_negative_direction_symmetric() {
    Plant p(ideal(0.3f));
    run(p, -30.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -100.0f, p.w_rad_s());
}

static void test_torque_lag_time_constant() {
    PlantParams pp = ideal(1000.0f);  // huge inertia, speed barely moves
    pp.tau_ms = 30.0f;
    Plant p(pp);
    run(p, 100.0f, 0.030f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f * (1.0f - expf(-1.0f)), p.t_motor_nm());
    run(p, 100.0f, 0.270f);  // 10 time constants in total
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, p.t_motor_nm());
}

static void test_torque_limit_at_standstill() {
    PlantParams pp = ideal(1000.0f);
    pp.tmax_nm = 280.0f;
    pp.pmax_kw = 80.0f;
    Plant p(pp);
    p.substep(512.0f, {}, DT);  // 100 % request at 0.25 Nm per bit
    TEST_ASSERT_TRUE(p.clipping());
    TEST_ASSERT_EQUAL_FLOAT(280.0f, p.t_cmd_nm());
    p.substep(-512.0f, {}, DT);
    TEST_ASSERT_EQUAL_FLOAT(-280.0f, p.t_cmd_nm());
}

static void test_power_limit_at_speed() {
    PlantParams pp = ideal(0.05f);
    pp.tmax_nm = 280.0f;
    pp.pmax_kw = 10.0f;
    Plant p(pp);
    // Corner speed 10 kW / 280 Nm = 35.7 rad/s; at 200 rad/s the limit is 50 Nm.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, p.torque_limit_nm(200.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, p.torque_limit_nm(-200.0f));
    TEST_ASSERT_EQUAL_FLOAT(280.0f, p.torque_limit_nm(30.0f));
    run(p, 280.0f, 2.0f);
    TEST_ASSERT_TRUE(p.clipping());
    TEST_ASSERT_FLOAT_WITHIN(20.0f, 10000.0f, p.t_cmd_nm() * p.w_rad_s());
    TEST_ASSERT_TRUE(p.t_cmd_nm() * p.w_rad_s() <= 10000.0f * 1.001f);
}

static void test_friction_holds_shaft_below_breakaway() {
    PlantParams pp = ideal(0.3f);
    pp.tc_nm = 5.0f;
    Plant p(pp);
    for (int i = 0; i < 1000; i++) {
        p.substep(4.9f, {}, DT);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, p.w_rad_s());
    }
    for (int i = 0; i < 1000; i++) {
        p.substep(-4.9f, {}, DT);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, p.w_rad_s());
    }
}

static void test_breakaway_above_friction() {
    PlantParams pp = ideal(0.3f);
    pp.tc_nm = 5.0f;
    Plant p(pp);
    run(p, 8.0f, 1.0f);  // (8 - 5) / 0.3 = 10 rad/s²
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, p.w_rad_s());
}

static void test_coast_down_stops_without_reversing() {
    PlantParams pp = ideal(0.3f);
    pp.tc_nm = 5.0f;
    pp.b_nms = 0.02f;
    Plant p(pp);
    run(p, 60.0f, 1.0f);
    TEST_ASSERT_TRUE(p.w_rad_s() > 100.0f);
    // Release: the shaft must reach exactly zero and stay there.
    bool stopped = false;
    for (int i = 0; i < 60000; i++) {
        p.substep(0.0f, {}, DT);
        TEST_ASSERT_TRUE(p.w_rad_s() >= 0.0f);
        if (stopped) TEST_ASSERT_EQUAL_FLOAT(0.0f, p.w_rad_s());
        if (p.w_rad_s() == 0.0f) stopped = true;
    }
    TEST_ASSERT_TRUE(stopped);
}

static void test_viscous_decay() {
    PlantParams pp = ideal(0.3f);
    pp.b_nms = 0.03f;  // time constant J / b = 10 s
    Plant p(pp);
    run(p, 30.0f, 1.0f);  // about 100 rad/s; viscous drag slows it slightly
    float w0 = p.w_rad_s();
    run(p, 0.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(w0 * 0.01f, w0 * expf(-1.0f), p.w_rad_s());
}

static void test_steady_state_torque_balance() {
    PlantParams pp = ideal(0.03f);
    pp.tc_nm = 5.0f;
    pp.b_nms = 0.02f;
    pp.c_nms2 = 0.0001f;
    pp.tau_ms = 30.0f;
    Plant p(pp);
    run(p, 10.0f, 30.0f);
    // 10 = 5 + 0.02 w + 0.0001 w², so w = (-0.02 + sqrt(0.0024)) / 0.0002 = 144.95 rad/s
    float w = p.w_rad_s();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 144.95f, w);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, p.t_load_nm());
}

static void test_extra_resistive_load() {
    PlantParams pp = ideal(0.3f);
    Plant p(pp);
    TickLoads l;
    l.t_res_nm = 10.0f;
    run(p, 40.0f, 1.0f, l);  // (40 - 10) / 0.3 = 100 rad/s²
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, p.w_rad_s());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, p.t_load_nm());
}

static void test_active_load_drives_shaft() {
    PlantParams pp = ideal(0.3f);
    pp.tc_nm = 5.0f;
    Plant p(pp);
    TickLoads l;
    l.t_active_nm = -20.0f;  // overrunning load pushing forward
    run(p, 0.0f, 1.0f, l);   // (20 - 5) / 0.3 = 50 rad/s²
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, p.w_rad_s());
    // A smaller active load than friction cannot move it.
    Plant q(pp);
    l.t_active_nm = -4.0f;
    run(q, 0.0f, 1.0f, l);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, q.w_rad_s());
}

static void test_reset_and_param_guards() {
    PlantParams pp = ideal(0.0f);  // zero inertia is clamped
    pp.tau_ms = -5.0f;
    Plant p(pp);
    TEST_ASSERT_EQUAL_FLOAT(Plant::J_MIN, p.params().j_kgm2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.params().tau_ms);
    run(p, 1.0f, 0.1f);
    TEST_ASSERT_TRUE(isfinite(p.w_rad_s()));
    p.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.w_rad_s());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.t_motor_nm());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_pure_inertia_acceleration);
    RUN_TEST(test_negative_direction_symmetric);
    RUN_TEST(test_torque_lag_time_constant);
    RUN_TEST(test_torque_limit_at_standstill);
    RUN_TEST(test_power_limit_at_speed);
    RUN_TEST(test_friction_holds_shaft_below_breakaway);
    RUN_TEST(test_breakaway_above_friction);
    RUN_TEST(test_coast_down_stops_without_reversing);
    RUN_TEST(test_viscous_decay);
    RUN_TEST(test_steady_state_torque_balance);
    RUN_TEST(test_extra_resistive_load);
    RUN_TEST(test_active_load_drives_shaft);
    RUN_TEST(test_reset_and_param_guards);
    return UNITY_END();
}
