#include "console.h"

#include <Arduino.h>
#include <string.h>

#include "can_io.h"
#include "config.h"
#include "console_parse.h"
#include "sim_task.h"

static const char *twai_state_name(uint8_t s) {
    switch (s) {
    case 0: return "stopped";
    case 1: return "running";
    case 2: return "bus_off";
    case 3: return "recovering";
    default: return "unknown";
    }
}

static void cmd_help() {
    Serial.println("ok commands: help, stat, params");
}

static void cmd_params() {
    SimSnapshot s = sim_get_snapshot();
    const SimParams &p = s.params;
    Serial.printf("ok J=%.3f Tc=%.2f b=%.4f c=%.6f tau=%.1f Tmax=%.1f Pmax=%.1f kT=%.3f"
                  " udc=%.0f motor_c=%.0f inv_c=%.0f fault=%d\n",
                  p.plant.j_kgm2, p.plant.tc_nm, p.plant.b_nms, p.plant.c_nms2,
                  p.plant.tau_ms, p.plant.tmax_nm, p.plant.pmax_kw, p.k_t, p.udc_v,
                  p.motor_c, p.inv_c, p.inv_fault ? 1 : 0);
}

static void cmd_stat() {
    SimSnapshot s = sim_get_snapshot();
    CanStats c = can_get_stats();
    const simcore::TickStats &t = s.stats;

    Serial.printf("ok t_ms=%lu rpm=%.1f req_raw=%d t_req=%.2f t_motor=%.2f t_load=%.2f"
                  " clip=%d hv=%u rx_timeout=%d udc=%.0f",
                  (unsigned long)s.sim_ms, s.rpm, s.cmd.raw, s.t_req_nm, s.t_motor_nm,
                  s.t_load_nm, s.clipping ? 1 : 0, s.cmd.hv_status,
                  s.cmd_fresh ? 0 : 1, s.params.udc_v);
    Serial.printf(" ticks=%lu period_us=%lu jitter_1s_us=%lu jitter_max_us=%lu"
                  " overruns=%lu out_of_tol=%lu",
                  (unsigned long)t.ticks(), (unsigned long)t.last_period_us(),
                  (unsigned long)t.window_worst_jitter_us(),
                  (unsigned long)t.worst_jitter_us(), (unsigned long)t.overruns(),
                  (unsigned long)t.out_of_tolerance());
    Serial.printf(" rx1d4=%lu rej1d4=%lu rx11a=%lu gear=0x%02X onoff=0x%02X rx50b=%lu"
                  " rx_other=%lu",
                  (unsigned long)c.rx_1d4, (unsigned long)c.rej_1d4,
                  (unsigned long)c.rx_11a, c.gear_11a, c.onoff_11a,
                  (unsigned long)c.rx_50b, (unsigned long)c.rx_other);
    Serial.printf(" tx_queued=%lu tx_failed=%lu can=%s tec=%lu rec=%lu bus_off=%lu"
                  " last_bus_err_ms=%lu\n",
                  (unsigned long)c.tx_queued, (unsigned long)c.tx_failed,
                  twai_state_name(c.state), (unsigned long)c.tec,
                  (unsigned long)c.rec, (unsigned long)c.bus_off_count,
                  (unsigned long)c.last_bus_error_ms);
}

static void handle_line(char *line) {
    char *argv[8];
    int argc = simproto::tokenize(line, argv, 8);
    if (argc == 0) return;
    if (strcmp(argv[0], "help") == 0) cmd_help();
    else if (strcmp(argv[0], "stat") == 0) cmd_stat();
    else if (strcmp(argv[0], "params") == 0) cmd_params();
    else Serial.printf("err unknown command '%s', try help\n", argv[0]);
}

static void console_task(void *) {
    simproto::LineReader reader;
    char line[simproto::LineReader::MAX_LINE + 1];
    for (;;) {
        while (Serial.available() > 0) {
            if (!reader.feed((char)Serial.read())) continue;
            if (reader.overflowed()) {
                Serial.println("err line too long");
                continue;
            }
            strcpy(line, reader.line());
            handle_line(line);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void console_start_task() {
    xTaskCreatePinnedToCore(console_task, "console", TASK_STACK_BYTES, nullptr,
                            CONSOLE_TASK_PRIO, nullptr, CONSOLE_TASK_CORE);
}
