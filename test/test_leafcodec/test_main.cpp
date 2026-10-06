// Tests for lib/leafcodec against the Zombie's own encode and decode code.
//
// The zombie_* functions below are copied from SomersetEV/Stm32-vcu at commit
// d85e756 (branch Tractor-RPM-limiting-testing). Keep them verbatim apart from
// the wrapping, so the tests check the codec against what the Zombie really does.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "config.h"
#include "leafcodec.h"

using namespace leafcodec;

// ---- Zombie reference code ------------------------------------------------

// d85e756 src/NissLeafMng.cpp:497, NissLeafMng::nissan_crc
static void zombie_nissan_crc(uint8_t *data, uint8_t polynomial) {
  // We want to process 8 bytes with the 8th byte being zero
  data[7] = 0;
  uint8_t crc = 0;
  for (int b = 0; b < 8; b++) {
    for (int i = 7; i >= 0; i--) {
      uint8_t bit = ((data[b] & (1 << i)) > 0) ? 1 : 0;
      if (crc >= 0x80)
        crc = (uint8_t)(((crc << 1) + bit) ^ polynomial);
      else
        crc = (uint8_t)((crc << 1) + bit);
    }
  }
  data[7] = crc;
}

// d85e756 src/NissLeafMng.cpp:108-250, NissLeafMng::Task10Ms, 0x1D4 frame.
// hv_status stands in for the opmode branch (7 run/charge, 3 precharge, 2 off).
static void zombie_build_1d4(int16_t final_torque_request, uint8_t mprun10,
                             uint8_t hv_status, uint8_t bytes[8]) {
  bytes[0] = 0xF7;
  bytes[1] = 0x07;
  if (final_torque_request >= -2048 && final_torque_request <= 2047) {
    bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) |
               ((final_torque_request >> 4) & 0x7f);
    bytes[3] = (final_torque_request << 4) & 0xf0;
  } else {
    bytes[2] = 0x00;
    bytes[3] = 0x00;
  }
  bytes[4] = hv_status;
  bytes[4] = bytes[4] | (mprun10 << 6);
  bytes[5] = 0x44;
  bytes[6] = 0x30; // brake applied heavilly.
  zombie_nissan_crc(bytes, 0x85);
}

// d85e756 src/leafinv.cpp, LeafINV::DecodeCAN, id 0x1DA
static void zombie_decode_1da(const uint8_t *bytes, uint16_t &voltage,
                              int16_t &speed, bool &error) {
  voltage = (bytes[0] * 2);
  int16_t parsed_speed = (bytes[4] << 7) | bytes[5] >> 1;
  if (parsed_speed > 0x3fff)
    parsed_speed -= 0x7fff; // 15 bit signed conversion
  speed = parsed_speed;
  error = (bytes[6] & 0xb0) != 0x00;
}

// d85e756 src/leafinv.cpp, LeafINV::fahrenheit_to_celsius
static int8_t zombie_fahrenheit_to_celsius(uint16_t fahrenheit) {
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if (result < -128)
    return -128;
  if (result > 127)
    return 127;
  return result;
}

// ---- 0x1D4 ----------------------------------------------------------------

void setUp() {}
void tearDown() {}

static void test_1d4_every_raw_and_counter() {
    uint8_t b[8];
    for (int raw = -2048; raw <= 2047; raw++) {
        for (uint8_t ctr = 0; ctr < 4; ctr++) {
            zombie_build_1d4((int16_t)raw, ctr, HV_ON, b);
            Cmd1D4 c = decode_1d4(b, K_T_DEFAULT);
            TEST_ASSERT_EQUAL_INT16(raw, c.raw);
            TEST_ASSERT_EQUAL_FLOAT(raw * 0.25f, c.torque_nm);
            TEST_ASSERT_TRUE(c.crc_ok);
            TEST_ASSERT_EQUAL_UINT8(ctr, c.counter);
            TEST_ASSERT_EQUAL_UINT8(HV_ON, c.hv_status);
        }
    }
}

static void test_1d4_crc_matches_zombie() {
    uint8_t b[8];
    for (int raw = -2048; raw <= 2047; raw += 37) {
        zombie_build_1d4((int16_t)raw, (uint8_t)(raw & 3), HV_ON, b);
        uint8_t zombie_crc = b[7];
        b[7] = 0x5A; // our CRC must ignore byte 7
        TEST_ASSERT_EQUAL_HEX8(zombie_crc, nissan_crc(b));
    }
}

static void test_1d4_single_bit_errors_rejected() {
    const int16_t raws[] = {-2048, -1, 0, 1, 512, 2047};
    uint8_t b[8];
    for (int16_t raw : raws) {
        for (int bit = 0; bit < 64; bit++) {
            zombie_build_1d4(raw, 2, HV_ON, b);
            b[bit / 8] ^= (uint8_t)(1 << (bit % 8));
            TEST_ASSERT_FALSE(decode_1d4(b, K_T_DEFAULT).crc_ok);
        }
    }
}

static void test_1d4_hv_status_values() {
    const uint8_t states[] = {HV_OFF, HV_PRECHARGE, HV_ON};
    uint8_t b[8];
    for (uint8_t hv : states) {
        for (uint8_t ctr = 0; ctr < 4; ctr++) {
            zombie_build_1d4(100, ctr, hv, b);
            Cmd1D4 c = decode_1d4(b, K_T_DEFAULT);
            TEST_ASSERT_EQUAL_UINT8(hv, c.hv_status);
            TEST_ASSERT_EQUAL_UINT8(ctr, c.counter);
            TEST_ASSERT_TRUE(c.crc_ok);
        }
    }
}

// ---- 0x1DA ----------------------------------------------------------------

static void test_1da_speed_every_rpm() {
    uint8_t b[8];
    uint16_t v;
    int16_t speed;
    bool err;
    for (int rpm = -16383; rpm <= 16383; rpm++) {
        encode_1da(UDC_DEFAULT_V, (float)rpm, false, b);
        zombie_decode_1da(b, v, speed, err);
        // The Zombie reads reverse speeds 1 rpm high; not compensated.
        int expected = rpm < 0 ? rpm + 1 : rpm;
        TEST_ASSERT_EQUAL_INT16(expected, speed);
    }
}

static void test_1da_speed_clamped() {
    uint8_t b[8];
    uint16_t v;
    int16_t speed;
    bool err;
    encode_1da(UDC_DEFAULT_V, 20000.0f, false, b);
    zombie_decode_1da(b, v, speed, err);
    TEST_ASSERT_EQUAL_INT16(16383, speed);
    encode_1da(UDC_DEFAULT_V, -20000.0f, false, b);
    zombie_decode_1da(b, v, speed, err);
    TEST_ASSERT_EQUAL_INT16(-16382, speed);
}

static void test_1da_voltage_and_error() {
    uint8_t b[8];
    uint16_t v;
    int16_t speed;
    bool err;
    encode_1da(360.0f, 0.0f, false, b);
    zombie_decode_1da(b, v, speed, err);
    TEST_ASSERT_EQUAL_UINT16(360, v);
    TEST_ASSERT_FALSE(err);
    TEST_ASSERT_EQUAL_HEX8(0, b[2]);
    TEST_ASSERT_EQUAL_HEX8(0, b[3]);

    encode_1da(360.0f, 0.0f, true, b);
    zombie_decode_1da(b, v, speed, err);
    TEST_ASSERT_TRUE(err);

    encode_1da(1000.0f, 0.0f, false, b);
    zombie_decode_1da(b, v, speed, err);
    TEST_ASSERT_EQUAL_UINT16(510, v);
}

// ---- 0x55A ----------------------------------------------------------------

static void test_55a_temperatures() {
    uint8_t b[8];
    encode_55a(MOTOR_TEMP_DEFAULT_C, INV_TEMP_DEFAULT_C, b);
    TEST_ASSERT_EQUAL_UINT8(104, b[1]);
    TEST_ASSERT_EQUAL_UINT8(104, b[2]);
    TEST_ASSERT_EQUAL_INT8(40, zombie_fahrenheit_to_celsius(b[1]));
    TEST_ASSERT_EQUAL_INT8(40, zombie_fahrenheit_to_celsius(b[2]));

    encode_55a(80.0f, 60.0f, b);
    TEST_ASSERT_EQUAL_INT8(80, zombie_fahrenheit_to_celsius(b[1]));
    TEST_ASSERT_EQUAL_INT8(60, zombie_fahrenheit_to_celsius(b[2]));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_1d4_every_raw_and_counter);
    RUN_TEST(test_1d4_crc_matches_zombie);
    RUN_TEST(test_1d4_single_bit_errors_rejected);
    RUN_TEST(test_1d4_hv_status_values);
    RUN_TEST(test_1da_speed_every_rpm);
    RUN_TEST(test_1da_speed_clamped);
    RUN_TEST(test_1da_voltage_and_error);
    RUN_TEST(test_55a_temperatures);
    return UNITY_END();
}
