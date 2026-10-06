#include "telemetry.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace simproto {

static int16_t sat16(float v) {
    if (!(v == v)) return 0;
    if (v >= 32767.0f) return 32767;
    if (v <= -32768.0f) return -32768;
    return (int16_t)lroundf(v);
}

static void put16(uint8_t *d, uint16_t v) {
    d[0] = (uint8_t)(v & 0xFF);
    d[1] = (uint8_t)(v >> 8);
}

void pack_4b0(float rpm, float t_req, float t_motor, float t_load, uint8_t out[8]) {
    put16(out + 0, (uint16_t)sat16(rpm * 2.0f));
    put16(out + 2, (uint16_t)sat16(t_req * 10.0f));
    put16(out + 4, (uint16_t)sat16(t_motor * 10.0f));
    put16(out + 6, (uint16_t)sat16(t_load * 10.0f));
}

void pack_4b1(const SlowTelemetry &s, uint8_t out[8]) {
    memset(out, 0, 8);
    float j = s.inertia_kgm2 * 1000.0f;
    j = j < 0.0f ? 0.0f : (j > 65535.0f ? 65535.0f : j);
    put16(out, (uint16_t)lroundf(j));
    out[2] = s.preset_id;
    out[3] = s.profile_state;
    out[4] = s.flags;
    out[5] = (uint8_t)s.rejected_1d4;
    out[6] = (uint8_t)s.overruns;
    uint32_t jit = (s.worst_jitter_us + 50) / 100;  // 0.1 ms
    out[7] = jit > 255 ? 255 : (uint8_t)jit;
}

void pack_4b2(uint8_t event, uint16_t value, uint32_t sim_ms, uint8_t out[8]) {
    memset(out, 0, 8);
    out[0] = event;
    put16(out + 1, value);
    out[3] = (uint8_t)(sim_ms & 0xFF);
    out[4] = (uint8_t)((sim_ms >> 8) & 0xFF);
    out[5] = (uint8_t)((sim_ms >> 16) & 0xFF);
    out[6] = (uint8_t)(sim_ms >> 24);
}

int format_csv(const StreamSample &s, char *buf, size_t len) {
    return snprintf(buf, len, "%lu,%d,%.2f,%.2f,%.2f,%.1f,%u", (unsigned long)s.t_ms,
                    s.raw_req, s.t_req_nm, s.t_motor_nm, s.t_load_nm, s.rpm, s.flags);
}

} // namespace simproto
