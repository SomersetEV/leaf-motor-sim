// Leaf motor simulator firmware entry point. See docs/SIMULATOR_PLAN.md.
// The Zombie's 0x1D4 torque request drives a single-inertia plant with
// friction and configurable loads; 0x1DA reports its speed every 10 ms from
// power-up, with simulator telemetry in 0x4B0-0x4B2.

#include <Arduino.h>

#include "can_io.h"
#include "config.h"
#include "console.h"
#include "params_store.h"
#include "sd_files.h"
#include "sim_defaults.h"
#include "sim_task.h"

static SimParams s_params;
static bool s_can_ok = false;
static SemaphoreHandle_t s_init_done;

// Drivers are installed from core 0 so their interrupts run there: an ESP32
// interrupt stays on the core that allocated it, and the UART and TWAI ISRs
// on core 1 held off the 10 ms tick (M5 bench, about 0.6 ms late wakes).
static void init_drivers(void *) {
    Serial.begin(921600);
    Serial.println("leaf-motor-sim M6");

    s_params = default_sim_params();
    bool saved = params_load(s_params);
    bool sd = sd_begin();
    Serial.printf("params from %s, SD card %s\n", saved ? "NVS" : "defaults",
                  sd ? "found" : "not found");

    s_can_ok = can_init();
    xSemaphoreGive(s_init_done);
    vTaskDelete(nullptr);
}

void setup() {
    s_init_done = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(init_drivers, "init", INIT_STACK_BYTES, nullptr, INIT_TASK_PRIO,
                            nullptr, DRIVER_CORE);
    xSemaphoreTake(s_init_done, portMAX_DELAY);

    if (!s_can_ok) {
        Serial.println("err CAN driver failed to start");
        return;
    }
    can_start_rx_task();
    sim_start_task(s_params);
    console_start_task();
    Serial.println("ok running, type help");
}

void loop() {
    vTaskDelete(nullptr);  // all work runs in the tasks started in setup()
}
