// Leaf motor simulator firmware entry point. See docs/SIMULATOR_PLAN.md.
// M2: CAN bring-up. Sends 0x1DA every 10 ms and 0x55A every 100 ms from
// power-up, with speed held at zero. The plant model arrives in M3.

#include <Arduino.h>

#include "can_io.h"
#include "console.h"
#include "sim_task.h"

void setup() {
    Serial.begin(921600);
    Serial.println("leaf-motor-sim M2 CAN bring-up");

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
