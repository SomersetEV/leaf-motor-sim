// Simulator telemetry frames and CSV stream lines.
// See docs/SIMULATOR_PLAN.md section 5, "Simulator telemetry frames", and
// section 7, "Serial console". Multi-byte values are little-endian and
// saturate instead of wrapping.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace simproto {

// 0x4B1 byte 4 and the CSV flags column
constexpr uint8_t FLAG_RX_TIMEOUT = 0x01;
constexpr uint8_t FLAG_CLIPPING   = 0x02;
constexpr uint8_t FLAG_NOISE      = 0x04;
constexpr uint8_t FLAG_FAULT      = 0x08;
constexpr uint8_t FLAG_BUS_ERROR  = 0x10;

// 0x4B2 byte 0
enum EventType : uint8_t {
    EV_MARKER = 1,
    EV_LOAD = 2,
    EV_PROFILE_START = 3,
    EV_PROFILE_STOP = 4,
    EV_PRESET = 5,
    EV_PARAM = 6,
};

// 0x4B0, every 10 ms: model speed 0.5 rpm, torques 0.1 Nm
void pack_4b0(float rpm, float t_req_nm, float t_motor_nm, float t_load_nm, uint8_t out[8]);

struct SlowTelemetry {
    float    inertia_kgm2;
    uint8_t  preset_id;
    uint8_t  profile_state;  // 0 idle, 1 loaded, 2 running, 3 finished
    uint8_t  flags;          // FLAG_*
    uint32_t rejected_1d4;   // sent modulo 256
    uint32_t overruns;       // sent modulo 256
    uint32_t worst_jitter_us;
};

// 0x4B1, every 100 ms
void pack_4b1(const SlowTelemetry &s, uint8_t out[8]);

// 0x4B2, on event
void pack_4b2(uint8_t event, uint16_t value, uint32_t sim_ms, uint8_t out[8]);

struct StreamSample {
    uint32_t t_ms;
    int16_t  raw_req;
    float    t_req_nm;
    float    t_motor_nm;
    float    t_load_nm;
    float    rpm;
    uint8_t  flags;
};

constexpr const char *CSV_HEADER = "t_ms,raw_req,t_req_nm,t_motor_nm,t_load_nm,rpm,flags";

// Formats one CSV line (no newline). Returns the length, as snprintf.
int format_csv(const StreamSample &s, char *buf, size_t len);

} // namespace simproto
