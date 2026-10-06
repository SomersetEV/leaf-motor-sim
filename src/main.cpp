// Leaf motor simulator firmware entry point. See docs/SIMULATOR_PLAN.md.
// M3: closed loop. The Zombie's 0x1D4 torque request drives a single-inertia
// plant with base friction; 0x1DA reports its speed every 10 ms from power-up.

#include <Arduino.h>

#include "can_io.h"
#include "console.h"
#include "sim_task.h"

void setup() {
    Serial.begin(921600);
    Serial.println("leaf-motor-sim M3 closed loop");

    if (!can_init()) {
        Serial.println("err CAN driver failed to start");
        return;
    }
    can_start_rx_task();
    sim_start_task();
    console_start_task();
    Serial.println("ok running, type help");
}

void loop() {
    vTaskDelete(nullptr);  // all work runs in the tasks started in setup()
}
