#include "zombie_sdo.h"

#include <math.h>
#include <string.h>

namespace simproto {

constexpr uint16_t SDO_INDEX_PARAM_UID = 0x2100;

const ZParam *zparam_find(const char *name) {
    for (const ZParam &p : ZPARAMS)
        if (strcmp(p.name, name) == 0) return &p;
    return nullptr;
}

static void put_header(uint8_t cmd, uint16_t id, uint8_t out[8]) {
    uint16_t index = SDO_INDEX_PARAM_UID | (id >> 8);
    out[0] = cmd;
    out[1] = (uint8_t)index;
    out[2] = (uint8_t)(index >> 8);
    out[3] = (uint8_t)id;
}

void sdo_write_request(uint16_t id, float value, uint8_t out[8]) {
    put_header(SDO_CMD_WRITE, id, out);
    uint32_t fixed = (uint32_t)(int32_t)lroundf(value * 32.0f);
    for (int i = 0; i < 4; i++) out[4 + i] = (uint8_t)(fixed >> (8 * i));
}

void sdo_read_request(uint16_t id, uint8_t out[8]) {
    put_header(SDO_CMD_READ, id, out);
    memset(out + 4, 0, 4);
}

SdoResult sdo_decode_reply(const uint8_t *d, uint8_t dlc) {
    SdoResult r{SdoReply::Bad, 0, 0.0f, 0};
    if (dlc < 8) return r;
    uint16_t index = (uint16_t)(d[1] | (d[2] << 8));
    if ((index & 0xFF00) != SDO_INDEX_PARAM_UID) return r;
    r.id = (uint16_t)(((index & 0xFF) << 8) | d[3]);
    uint32_t raw = (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) |
                   ((uint32_t)d[7] << 24);
    switch (d[0]) {
    case SDO_CMD_READ_REPLY:  r.kind = SdoReply::ReadOk;  break;
    case SDO_CMD_WRITE_REPLY: r.kind = SdoReply::WriteOk; break;
    case SDO_CMD_ABORT:       r.kind = SdoReply::Abort; r.abort_code = raw; return r;
    default: return r;
    }
    r.value = (float)(int32_t)raw / 32.0f;
    return r;
}

} // namespace simproto
