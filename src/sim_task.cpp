#include "sim_task.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "can_io.h"
#include "config.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static SimSnapshot s_snap = {
    0, 0.0f, {}, false,
    {UDC_DEFAULT_V, MOTOR_TEMP_DEFAULT_C, INV_TEMP_DEFAULT_C, K_T_DEFAULT, false},
    simcore::TickStats(TICK_MS * 1000, TICK_TOLERANCE_US, JITTER_WINDOW_TICKS),
};

// Nothing in here may block: no serial, no SD, no heap (plan rule 4).
static void sim_task(void *) {
    simcore::TickStats stats(TICK_MS * 1000, TICK_TOLERANCE_US, JITTER_WINDOW_TICKS);
    SimParams p = s_snap.params;
    TickType_t wake = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    uint32_t tick = 0;
    uint8_t b[8];

    for (;;) {
        BaseType_t delayed = xTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
        int64_t now_us = esp_timer_get_time();
        if (tick > 0) stats.record((uint32_t)(now_us - prev_us), delayed == pdFALSE);
        prev_us = now_us;
        uint32_t now_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());

        // 1. Latest torque request, with the 100 ms receive timeout
        leafcodec::Cmd1D4 cmd;
        bool fresh = can_get_cmd(now_ms, cmd);

        // 2-3. Loads and plant arrive in M3; speed is held at zero
        float rpm = 0.0f;

        // 4. Inverter status every tick, from power-up
        leafcodec::encode_1da(p.udc_v, rpm, p.inv_fault, b);
        can_send(ID_LEAF_INV_STATUS, b, 8);

        // 5. Temperatures every tenth tick
        if (tick % SLOW_PERIOD_TICKS == 0) {
            leafcodec::encode_55a(p.motor_c, p.inv_c, b);
            can_send(ID_LEAF_INV_TEMPS, b, 8);
        }

        portENTER_CRITICAL(&s_mux);
        s_snap.sim_ms    = now_ms;
        s_snap.rpm       = rpm;
        s_snap.cmd       = cmd;
        s_snap.cmd_fresh = fresh;
        s_snap.stats     = stats;
        portEXIT_CRITICAL(&s_mux);

        tick++;
    }
}

void sim_start_task() {
    xTaskCreatePinnedToCore(sim_task, "sim", TASK_STACK_BYTES, nullptr,
                            SIM_TASK_PRIO, nullptr, SIM_TASK_CORE);
}

SimSnapshot sim_get_snapshot() {
    portENTER_CRITICAL(&s_mux);
    SimSnapshot s = s_snap;
    portEXIT_CRITICAL(&s_mux);
    return s;
}
