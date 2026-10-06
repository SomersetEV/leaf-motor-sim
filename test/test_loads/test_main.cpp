// Tests for lib/loads: load components, profile storage and CSV parsing.

#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "loads.h"
#include "profile.h"

using namespace loads;

static constexpr float DT = 0.01f;  // one 10 ms tick
static constexpr float RPM_TO_RAD_S = 2.0f * 3.14159265358979f / 60.0f;

void setUp() {}
void tearDown() {}

// ---- Step ------------------------------------------------------------------

static void test_step_ramps_then_adds_inertia() {
    Loads l;
    l.start_step(100.0f, 500.0f, 0.3f);
    LoadOutput o{};
    for (int i = 0; i < 25; i++) o = l.tick(0.0f, DT);  // 250 ms: half way
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, o.tick.t_res_nm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.inertia_delta_kgm2);
    for (int i = 0; i < 25; i++) o = l.tick(0.0f, DT);  // 500 ms: ramp done
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, o.tick.t_res_nm);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, o.inertia_delta_kgm2);
}

static void test_step_instant_and_load_loss() {
    Loads l;
    l.start_step(80.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, l.tick(0.0f, DT).tick.t_res_nm);
    l.start_step(0.0f, 0.0f, 0.0f);  // sudden load loss
    TEST_ASSERT_EQUAL_FLOAT(0.0f, l.tick(0.0f, DT).tick.t_res_nm);
}

static void test_step_ramps_from_current_level() {
    Loads l;
    l.start_step(40.0f, 0.0f, 0.0f);
    l.tick(0.0f, DT);
    l.start_step(80.0f, 100.0f, 0.0f);
    LoadOutput o{};
    for (int i = 0; i < 5; i++) o = l.tick(0.0f, DT);  // 50 of 100 ms
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, o.tick.t_res_nm);
}

// ---- Pulse -----------------------------------------------------------------

static void test_pulse_half_sine() {
    Loads l;
    l.start_pulse(100.0f, 1000.0f, 50.0f);  // 500 ms half-sine, then 500 ms off
    float peak = 0.0f;
    float values[100];
    for (int i = 0; i < 100; i++) {
        values[i] = l.tick(0.0f, DT).tick.t_res_nm;
        if (values[i] > peak) peak = values[i];
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, values[0]);                       // sin(0)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, values[25]);            // 250 ms: peak
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f * sinf(3.14159265f * 0.2f), values[10]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, peak);
    for (int i = 50; i < 100; i++) TEST_ASSERT_EQUAL_FLOAT(0.0f, values[i]);
    // Next period repeats
    LoadOutput o{};
    for (int i = 0; i <= 10; i++) o = l.tick(0.0f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, values[10], o.tick.t_res_nm);
}

// ---- Noise -----------------------------------------------------------------

static void test_noise_statistics() {
    Loads l(12345);
    l.set_noise(5.0f, 2.0f);
    const int n = 200000;
    double sum = 0, sum2 = 0;
    LoadConfig c = l.config();
    TEST_ASSERT_TRUE(c.noise_on);
    // Resistive output is clamped at zero, so check the raw component via a
    // large constant step that keeps the sum positive.
    l.start_step(100.0f, 0.0f, 0.0f);
    for (int i = 0; i < n; i++) {
        double v = l.tick(0.0f, DT).tick.t_res_nm - 100.0;
        sum += v;
        sum2 += v * v;
    }
    double mean = sum / n;
    double sd = sqrt(sum2 / n - mean * mean);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, (float)mean);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 5.0f, (float)sd);  // within 10 %
}

static void test_noise_deterministic_for_seed() {
    Loads a(7), b(7);
    a.set_noise(3.0f, 5.0f);
    b.set_noise(3.0f, 5.0f);
    a.start_step(50.0f, 0.0f, 0.0f);
    b.start_step(50.0f, 0.0f, 0.0f);
    for (int i = 0; i < 100; i++) {
        float va = a.tick(0.0f, DT).tick.t_res_nm;  // Unity macros evaluate args twice
        float vb = b.tick(0.0f, DT).tick.t_res_nm;
        TEST_ASSERT_EQUAL_FLOAT(va, vb);
    }
}

static void test_resistive_sum_never_negative() {
    Loads l(99);
    l.set_noise(20.0f, 10.0f);
    for (int i = 0; i < 10000; i++) TEST_ASSERT_TRUE(l.tick(0.0f, DT).tick.t_res_nm >= 0.0f);
}

// ---- Gradient and off ------------------------------------------------------

static void test_grad_is_active_and_off_clears() {
    Loads l;
    l.set_grad(-25.0f);
    l.start_step(30.0f, 0.0f, 0.2f);
    l.start_pulse(50.0f, 100.0f, 50.0f);
    LoadOutput o = l.tick(0.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(-25.0f, o.tick.t_active_nm);
    l.off();
    o = l.tick(0.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.tick.t_active_nm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.tick.t_res_nm);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.inertia_delta_kgm2);
}

static void test_configure_applies_step_immediately() {
    Loads l;
    LoadConfig c;
    c.step_on = true;
    c.step_nm = 60.0f;
    c.step_ramp_ms = 500.0f;
    c.step_dj_kgm2 = 0.3f;
    l.configure(c);
    LoadOutput o = l.tick(0.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, o.tick.t_res_nm);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, o.inertia_delta_kgm2);
}

// ---- Profile storage and replay -------------------------------------------

static void fill(Profile &p, int rows) {
    for (int i = 0; i < rows; i++) TEST_ASSERT_TRUE(p.append(i * 0.1f));
}

static void test_profile_storage_across_chunks() {
    Profile p;
    fill(p, Profile::CHUNK_ROWS * 2 + 10);
    TEST_ASSERT_EQUAL_UINT32(Profile::CHUNK_ROWS * 2 + 10, p.rows());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, (Profile::CHUNK_ROWS + 3) * 0.1f, p.torque_nm(Profile::CHUNK_ROWS + 3));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, p.torque_nm(p.rows()));  // past the end
    p.clear();
    TEST_ASSERT_EQUAL_UINT32(0, p.rows());
}

static void test_profile_row_cap() {
    Profile p;
    for (uint32_t i = 0; i < Profile::MAX_ROWS; i++) p.append(1.0f);
    TEST_ASSERT_FALSE(p.append(1.0f));
    TEST_ASSERT_EQUAL_UINT32(Profile::MAX_ROWS, p.rows());
}

static void test_profile_replay_finish_and_loop() {
    Profile p;
    p.append(10.0f);
    p.append(20.0f);
    p.append(30.0f);
    Loads l;
    TEST_ASSERT_TRUE(l.profile_state() == ProfileState::Idle);
    TEST_ASSERT_FALSE(l.profile_start());
    l.set_profile(&p);
    TEST_ASSERT_TRUE(l.profile_state() == ProfileState::Loaded);
    TEST_ASSERT_TRUE(l.profile_start());
    TEST_ASSERT_EQUAL_FLOAT(10.0f, l.tick(0.0f, DT).tick.t_res_nm);
    TEST_ASSERT_EQUAL_FLOAT(20.0f, l.tick(0.0f, DT).tick.t_res_nm);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, l.tick(0.0f, DT).tick.t_res_nm);
    TEST_ASSERT_TRUE(l.profile_state() == ProfileState::Finished);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, l.tick(0.0f, DT).tick.t_res_nm);

    l.set_profile_loop(true);
    l.profile_start();
    for (int i = 0; i < 3; i++) l.tick(0.0f, DT);
    TEST_ASSERT_TRUE(l.profile_state() == ProfileState::Running);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, l.tick(0.0f, DT).tick.t_res_nm);
    l.profile_stop();
    TEST_ASSERT_TRUE(l.profile_state() == ProfileState::Loaded);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, l.tick(0.0f, DT).tick.t_res_nm);
}

static void test_profile_speed_exponent() {
    Profile p;
    for (int i = 0; i < 10; i++) p.append(40.0f);
    Loads l;
    l.set_profile(&p);
    l.set_profile_exponent(2.0f, 1000.0f);
    l.profile_start();
    float w = 500.0f * RPM_TO_RAD_S;  // half of w_ref
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, l.tick(w, DT).tick.t_res_nm);  // 40 * 0.25
    l.set_profile_exponent(0.0f, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 40.0f, l.tick(w, DT).tick.t_res_nm);
}

static void test_profile_inertia_change_points() {
    Profile p;
    ProfileCsvParser csv(p);
    TEST_ASSERT_TRUE(csv.line("t_ms,torque_nm,inertia_kgm2"));
    TEST_ASSERT_TRUE(csv.line("0,1,0.30"));
    TEST_ASSERT_TRUE(csv.line("10,1,0.30"));
    TEST_ASSERT_TRUE(csv.line("20,1,0.60"));
    TEST_ASSERT_TRUE(csv.line("30,1,0.60"));
    TEST_ASSERT_TRUE(csv.finish());
    TEST_ASSERT_TRUE(p.has_inertia());
    TEST_ASSERT_EQUAL_FLOAT(0.30f, p.inertia_kgm2(1));
    TEST_ASSERT_EQUAL_FLOAT(0.60f, p.inertia_kgm2(2));
    TEST_ASSERT_EQUAL_FLOAT(0.60f, p.inertia_kgm2(3));

    Loads l;
    l.set_profile(&p);
    l.profile_start();
    LoadOutput o = l.tick(0.0f, DT);
    TEST_ASSERT_TRUE(o.profile_inertia);
    TEST_ASSERT_EQUAL_FLOAT(0.30f, o.profile_inertia_kgm2);
    l.tick(0.0f, DT);
    o = l.tick(0.0f, DT);
    TEST_ASSERT_EQUAL_FLOAT(0.60f, o.profile_inertia_kgm2);
}

// ---- Profile CSV parser ----------------------------------------------------

static void test_csv_accepts_plan_example() {
    Profile p;
    ProfileCsvParser csv(p);
    TEST_ASSERT_TRUE(csv.line("t_ms,torque_nm,inertia_kgm2\r\n"));
    TEST_ASSERT_TRUE(csv.line("0,12.5,0.30"));
    TEST_ASSERT_TRUE(csv.line("10,12.9,0.30"));
    TEST_ASSERT_TRUE(csv.line("20,14.1,0.30\r"));
    TEST_ASSERT_TRUE(csv.line(""));
    TEST_ASSERT_TRUE(csv.finish());
    TEST_ASSERT_EQUAL_UINT32(3, p.rows());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.9f, p.torque_nm(1));
}

static void test_csv_rejects() {
    {
        Profile p;
        ProfileCsvParser csv(p);
        TEST_ASSERT_FALSE(csv.line("time,torque"));
    }
    {
        Profile p;
        ProfileCsvParser csv(p);
        csv.line("t_ms,torque_nm");
        TEST_ASSERT_TRUE(csv.line("0,1"));
        TEST_ASSERT_FALSE(csv.line("15,1"));  // not 10 ms spacing
    }
    {
        Profile p;
        ProfileCsvParser csv(p);
        csv.line("t_ms,torque_nm");
        TEST_ASSERT_FALSE(csv.line("10,1"));  // must start at 0
    }
    {
        Profile p;
        ProfileCsvParser csv(p);
        csv.line("t_ms,torque_nm");
        TEST_ASSERT_FALSE(csv.line("0,abc"));
        TEST_ASSERT_EQUAL_UINT32(2, csv.line_number());
    }
    {
        Profile p;
        ProfileCsvParser csv(p);
        csv.line("t_ms,torque_nm");
        TEST_ASSERT_FALSE(csv.line("0,1,0.3"));  // extra column
    }
    {
        Profile p;
        ProfileCsvParser csv(p);
        csv.line("t_ms,torque_nm");
        TEST_ASSERT_FALSE(csv.finish());  // no rows
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_step_ramps_then_adds_inertia);
    RUN_TEST(test_step_instant_and_load_loss);
    RUN_TEST(test_step_ramps_from_current_level);
    RUN_TEST(test_pulse_half_sine);
    RUN_TEST(test_noise_statistics);
    RUN_TEST(test_noise_deterministic_for_seed);
    RUN_TEST(test_resistive_sum_never_negative);
    RUN_TEST(test_grad_is_active_and_off_clears);
    RUN_TEST(test_configure_applies_step_immediately);
    RUN_TEST(test_profile_storage_across_chunks);
    RUN_TEST(test_profile_row_cap);
    RUN_TEST(test_profile_replay_finish_and_loop);
    RUN_TEST(test_profile_speed_exponent);
    RUN_TEST(test_profile_inertia_change_points);
    RUN_TEST(test_csv_accepts_plan_example);
    RUN_TEST(test_csv_rejects);
    return UNITY_END();
}
