// WS2812 status LED. See docs/SIMULATOR_PLAN.md section 7, "Status LED".
//   Red:    CAN bus-off, or a tick overrun or out-of-tolerance tick, in the last second
//   Yellow: profile replay in progress
//   Green:  receiving valid torque requests
//   Blue:   running, no valid 0x1D4 (Zombie not in run mode)
#pragma once

#include "sim_task.h"

void status_led_update(const SimSnapshot &s, bool can_bus_off_recent);
