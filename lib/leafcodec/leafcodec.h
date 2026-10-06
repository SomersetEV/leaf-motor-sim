// Nissan Leaf inverter CAN codec: decodes the Zombie's 0x1D4 torque request
// and encodes the inverter's 0x1DA and 0x55A frames.
//
// Plain C++, no Arduino or ESP-IDF includes, so it builds and tests on the PC.
// Byte layouts mirror SomersetEV/Stm32-vcu at commit d85e756
// (src/NissLeafMng.cpp and src/leafinv.cpp). See docs/SIMULATOR_PLAN.md section 5.
#pragma once

#include <stdint.h>

namespace leafcodec {

constexpr uint8_t NISSAN_CRC_POLY = 0x85;

// HV status values in the low nibble of 0x1D4 byte 4
constexpr uint8_t HV_OFF       = 0x02;
constexpr uint8_t HV_PRECHARGE = 0x03;
constexpr uint8_t HV_ON        = 0x07;

constexpr float RPM_LIMIT     = 16383.0f; // 0x1DA speed range, 0.5 rpm per bit
constexpr float UDC_LIMIT_V   = 510.0f;   // 0x1DA byte 0, 2 V per bit

// CRC over bytes 0..6 with byte 7 treated as zero, as the Zombie computes it.
// Does not modify the frame.
uint8_t nissan_crc(const uint8_t b[8]);

struct Cmd1D4 {
    int16_t raw;        // signed 12-bit torque request, -2048..2047
    float   torque_nm;  // raw * k_t
    uint8_t hv_status;  // HV_OFF, HV_PRECHARGE or HV_ON
    uint8_t counter;    // 0..3, increments each frame
    bool    crc_ok;     // byte 7 matches nissan_crc
};

Cmd1D4 decode_1d4(const uint8_t b[8], float k_t);

// Inverter status: bus voltage, motor speed and error flag. Speed is encoded
// as the real inverter does; the Zombie's 1 rpm offset on reverse speeds is
// not compensated.
void encode_1da(float udc_v, float rpm, bool inv_error, uint8_t out[8]);

// Motor and inverter temperatures, sent on the bus in degrees Fahrenheit.
void encode_55a(float motor_c, float inv_c, uint8_t out[8]);

} // namespace leafcodec
