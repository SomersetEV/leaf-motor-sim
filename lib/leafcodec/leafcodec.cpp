#include "leafcodec.h"

#include <math.h>
#include <string.h>

namespace leafcodec {

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Ported from NissLeafMng::nissan_crc (d85e756, src/NissLeafMng.cpp:497),
// reading byte 7 as zero instead of overwriting it.
uint8_t nissan_crc(const uint8_t b[8]) {
    uint8_t crc = 0;
    for (int n = 0; n < 8; n++) {
        uint8_t byte = (n == 7) ? 0 : b[n];
        for (int i = 7; i >= 0; i--) {
            uint8_t bit = ((byte & (1 << i)) > 0) ? 1 : 0;
            if (crc >= 0x80)
                crc = (uint8_t)(((crc << 1) + bit) ^ NISSAN_CRC_POLY);
            else
                crc = (uint8_t)((crc << 1) + bit);
        }
    }
    return crc;
}

Cmd1D4 decode_1d4(const uint8_t b[8], float k_t) {
    Cmd1D4 c;
    // bytes 2..3: signed 12-bit torque request, left-justified
    int16_t raw = (int16_t)((b[2] << 4) | (b[3] >> 4));
    if (raw & 0x800) raw -= 0x1000;
    c.raw       = raw;
    c.torque_nm = raw * k_t;
    c.hv_status = b[4] & 0x0F;
    c.counter   = b[4] >> 6;
    c.crc_ok    = nissan_crc(b) == b[7];
    return c;
}

void encode_1da(float udc_v, float rpm, bool inv_error, uint8_t out[8]) {
    memset(out, 0, 8);
    out[0] = (uint8_t)lroundf(clampf(udc_v, 0.0f, UDC_LIMIT_V) / 2.0f);
    // bytes 2..3: reported torque, left zero until SIM_REPORT_TORQUE (plan V3)
    int16_t raw = (int16_t)lroundf(clampf(rpm, -RPM_LIMIT, RPM_LIMIT) * 2.0f);
    out[4] = (uint8_t)((uint16_t)raw >> 8);
    out[5] = (uint8_t)(raw & 0xFF);
    out[6] = inv_error ? 0x10 : 0x00;
}

static uint8_t celsius_to_fahrenheit(float c) {
    return (uint8_t)lroundf(clampf(c * 9.0f / 5.0f + 32.0f, 0.0f, 255.0f));
}

void encode_55a(float motor_c, float inv_c, uint8_t out[8]) {
    memset(out, 0, 8);
    out[1] = celsius_to_fahrenheit(motor_c);
    out[2] = celsius_to_fahrenheit(inv_c);
}

} // namespace leafcodec
