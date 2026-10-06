// Leaf motor simulator firmware entry point. See docs/SIMULATOR_PLAN.md.
// The Zombie's 0x1D4 torque request drives a single-inertia plant with
// friction and configurable loads; 0x1DA reports its speed every 10 ms from
// power-up, with simulator telemetry in 0x4B0-0x4B2.

#include <Arduino.h>

#include "can_io.h"
#include "console.h"
#include "params_store.h"
#include "sd_files.h"
#include "sim_defaults.h"
#include "sim_task.h"

void setup() {
    Serial.begin(921600);
    Serial.println("leaf-motor-sim M6");

    SimParams params = default_sim_params();
    bool saved = params_load(params);
    bool sd = sd_begin();
    Serial.printf("params from %s, SD card %s\n", saved ? "NVS" : "defaults",
                  sd ? "found" : "not found");

    if (!can_init()) {
        Serial.println("err CAN driver failed to start");
        return;
    }
    can_start_rx_task();
    sim_start_task(params);
    console_start_task();
    Serial.println("ok running, type help");
}

void loop() {
    vTaskDelete(nullptr);  // all work runs in the tasks started in setup()
}
