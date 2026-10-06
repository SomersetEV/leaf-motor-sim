// TWAI (CAN) driver wrapper: init, receive task, non-blocking transmit,
// bus-off recovery. See docs/SIMULATOR_PLAN.md section 7, "CAN driver".
#pragma once

#include <stdint.h>

#include "leafcodec.h"

struct CanStats {
    uint32_t rx_1d4;          // valid 0x1D4 frames
    uint32_t rej_1d4;         // 0x1D4 frames rejected (bad CRC or length)
    uint32_t rx_11a;
    uint32_t rx_50b;
    uint32_t rx_other;
    uint8_t  gear_11a;        // 0x11A byte 0, last seen
    uint8_t  onoff_11a;       // 0x11A byte 1, last seen
    uint32_t tx_queued;
    uint32_t tx_failed;       // queue full or bus off; frame dropped
    uint32_t bus_off_count;
    uint32_t last_bus_error_ms; // 0 if never
    uint32_t tec;             // transmit error counter
    uint32_t rec;             // receive error counter
    uint8_t  state;           // twai_state_t
};

// Powers the transceiver, installs and starts TWAI at 500 kbit/s.
bool can_init();

// Starts the receive task. Call after can_init().
void can_start_rx_task();

// Queues one frame with zero timeout. Never blocks; a failure is counted.
bool can_send(uint32_t id, const uint8_t *data, uint8_t dlc);

// Latest valid 0x1D4. Returns false, with zero torque in out, on timeout.
bool can_get_cmd(uint32_t now_ms, leafcodec::Cmd1D4 &out);

CanStats can_get_stats();
