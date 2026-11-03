// Zombie (openinverter VCU) parameter access over CANopen SDO, for M5a.
// See docs/SIMULATOR_PLAN.md section 11, "M5a". The Zombie's SDO server is
// node 3 on CAN1 (sdo.SetNodeId(3) in stm32_vcu.cpp). A parameter is addressed
// by its unique id: index 0x2100 | (id >> 8), sub-index id & 0xFF. Values are
// libopeninv fixed point, x32. Frames follow libopeninv's cansdo.h.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace simproto {

constexpr uint8_t  ZOMBIE_NODE_ID = 3;
constexpr uint32_t SDO_REQ_ID = 0x600 + ZOMBIE_NODE_ID;  // 0x603, simulator to Zombie
constexpr uint32_t SDO_REP_ID = 0x580 + ZOMBIE_NODE_ID;  // 0x583, Zombie to simulator

// cansdo.h command bytes
constexpr uint8_t SDO_CMD_WRITE       = 0x23;  // expedited download, 4 bytes
constexpr uint8_t SDO_CMD_READ        = 0x40;
constexpr uint8_t SDO_CMD_WRITE_REPLY = 0x60;
constexpr uint8_t SDO_CMD_READ_REPLY  = 0x43;
constexpr uint8_t SDO_CMD_ABORT       = 0x80;

struct ZParam {
    const char *name;
    uint16_t    id;  // Zombie parameter unique id (param_prj.h)
};

// The only Zombie parameters the simulator reads or writes: the governor,
// plus GovLog so a sweep can turn the governor frames on. It never sends
// the SDO save command, so a Zombie power cycle restores its saved setup.
constexpr ZParam ZPARAMS[] = {
    {"revlim", 15},   {"govKp", 157}, {"govKi", 158}, {"govImax", 159},
    {"govKd", 160},   {"govFlt", 162}, {"GovLog", 163},
};
constexpr int ZPARAM_COUNT = sizeof(ZPARAMS) / sizeof(ZPARAMS[0]);

// nullptr if name is not in ZPARAMS. Case-sensitive, as the Zombie's names.
const ZParam *zparam_find(const char *name);

void sdo_write_request(uint16_t id, float value, uint8_t out[8]);
void sdo_read_request(uint16_t id, uint8_t out[8]);

enum class SdoReply : uint8_t { WriteOk, ReadOk, Abort, Bad };

struct SdoResult {
    SdoReply kind;
    uint16_t id;          // parameter id the reply refers to
    float    value;       // ReadOk: the value; WriteOk: the value written
    uint32_t abort_code;  // Abort: e.g. 0x06090030 out of range, 0x06020000 no such index
};

// Decodes a 0x583 frame. Bad if it is short or not a parameter reply.
SdoResult sdo_decode_reply(const uint8_t *data, uint8_t dlc);

} // namespace simproto
