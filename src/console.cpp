#include "console.h"

#include <Arduino.h>
#include <string.h>

#include "can_io.h"
#include "commands.h"
#include "config.h"
#include "console_parse.h"
#include "param_table.h"
#include "params_store.h"
#include "presets.h"
#include "sd_files.h"
#include "sim_defaults.h"
#include "sim_task.h"
#include "status_led.h"

using simproto::CmdType;
using simproto::SimCmd;

static loads::Profile s_profile;  // the one profile in RAM, owned here
static bool s_streaming = false;
static SimCmd s_cmd;              // large (holds a preset), so not on the stack

static const char *twai_state_name(uint8_t s) {
    switch (s) {
    case 0: return "stopped";
    case 1: return "running";
    case 2: return "bus_off";
    case 3: return "recovering";
    default: return "unknown";
    }
}

static const char *profile_state_name(loads::ProfileState s) {
    switch (s) {
    case loads::ProfileState::Idle: return "idle";
    case loads::ProfileState::Loaded: return "loaded";
    case loads::ProfileState::Running: return "running";
    default: return "finished";
    }
}

static void reply_ok() { Serial.println("ok"); }
static void reply_err(const char *msg) { Serial.printf("err %s\n", msg); }

static void post(const SimCmd &c) {
    if (sim_post(c)) reply_ok();
    else reply_err("busy, try again");
}

static void cmd_help() {
    Serial.println("ok commands:");
    Serial.println("  stat | params | set <name> <value> | save");
    Serial.println("  preset <name> | presets");
    Serial.println("  load step <Nm> [ramp_ms] [dJ] | load pulse <Nm> <period_ms> <duty_pct>");
    Serial.println("  load grad <Nm> | load noise <sd_Nm> [corner_hz] | load off");
    Serial.println("  profile load <file> | profile start | profile stop | profile loop <0|1>");
    Serial.println("  profile exp <n> <w_ref_rpm> | profile list");
    Serial.println("  mark <n> | fault inv <0|1> | stream <0|1> [divider]");
}

static void cmd_params() {
    SimSnapshot s = sim_get_snapshot();
    Serial.print("ok");
    for (int i = 0; i < simcore::PARAM_COUNT; i++)
        Serial.printf(" %s=%g", simcore::PARAMS[i].name, simcore::param_get(s.params, i));
    Serial.println();
    for (int i = 0; i < simcore::PARAM_COUNT; i++)
        Serial.printf("  %-8s %-11s %g .. %g\n", simcore::PARAMS[i].name, simcore::PARAMS[i].unit,
                      simcore::PARAMS[i].min, simcore::PARAMS[i].max);
}

static void cmd_stat() {
    SimSnapshot s = sim_get_snapshot();
    CanStats c = can_get_stats();
    const simcore::TickStats &t = s.stats;
    const loads::LoadConfig &l = s.loads;

    Serial.printf("ok t_ms=%lu rpm=%.1f req_raw=%d t_req=%.2f t_motor=%.2f t_load=%.2f"
                  " J=%.3f clip=%d hv=%u rx_timeout=%d fault=%d",
                  (unsigned long)s.sim_ms, s.rpm, s.cmd.raw, s.t_req_nm, s.t_motor_nm,
                  s.t_load_nm, s.inertia_kgm2, s.clipping ? 1 : 0, s.cmd.hv_status,
                  s.cmd_fresh ? 0 : 1, s.inv_fault ? 1 : 0);
    Serial.printf(" preset=%s step=%.1f pulse=%s grad=%.1f noise=%s profile=%s row=%lu",
                  s.preset_id ? s.preset_name : "-", s.step_level_nm,
                  l.pulse_on ? "on" : "off", l.grad_nm, l.noise_on ? "on" : "off",
                  profile_state_name(s.profile_state), (unsigned long)s.profile_row);
    Serial.printf(" ticks=%lu period_us=%lu jitter_1s_us=%lu jitter_max_us=%lu"
                  " overruns=%lu out_of_tol=%lu stream_dropped=%lu",
                  (unsigned long)t.ticks(), (unsigned long)t.last_period_us(),
                  (unsigned long)t.window_worst_jitter_us(), (unsigned long)t.worst_jitter_us(),
                  (unsigned long)t.overruns(), (unsigned long)t.out_of_tolerance(),
                  (unsigned long)sim_stream_dropped());
    Serial.printf(" rx1d4=%lu rej1d4=%lu rx11a=%lu gear=0x%02X onoff=0x%02X rx50b=%lu"
                  " rx_other=%lu",
                  (unsigned long)c.rx_1d4, (unsigned long)c.rej_1d4, (unsigned long)c.rx_11a,
                  c.gear_11a, c.onoff_11a, (unsigned long)c.rx_50b, (unsigned long)c.rx_other);
    Serial.printf(" tx_queued=%lu tx_failed=%lu can=%s tec=%lu rec=%lu bus_off=%lu"
                  " last_bus_err_ms=%lu sd=%d\n",
                  (unsigned long)c.tx_queued, (unsigned long)c.tx_failed, twai_state_name(c.state),
                  (unsigned long)c.tec, (unsigned long)c.rec, (unsigned long)c.bus_off_count,
                  (unsigned long)c.last_bus_error_ms, sd_ready() ? 1 : 0);
}

static void cmd_save() {
    SimSnapshot s = sim_get_snapshot();
    if (params_save(s.params)) reply_ok();
    else reply_err("NVS write failed");
}

static void cmd_preset(int argc, char **argv) {
    if (argc != 2) return reply_err("usage: preset <name>");
    s_cmd = SimCmd{};
    s_cmd.type = CmdType::Preset;
    static char json[PRESET_JSON_MAX];
    const char *err = sd_read_preset(argv[1], json, sizeof(json));
    if (!err) {
        char bad[32] = "";
        err = simcore::preset_from_json(argv[1], json, default_sim_params(), s_cmd.preset, bad,
                                        sizeof(bad));
        if (err) {
            Serial.printf("err %s.json: %s%s%s\n", argv[1], err, bad[0] ? " " : "", bad);
            return;
        }
    } else if (strcmp(err, "missing") != 0) {
        return reply_err(err);
    } else if (!simcore::builtin_preset(argv[1], default_sim_params(), s_cmd.preset)) {
        return reply_err("unknown preset, try presets");
    }
    post(s_cmd);
}

static void cmd_presets() {
    Serial.print("ok built in:");
    for (int i = 0; i < simcore::BUILTIN_PRESET_COUNT; i++)
        Serial.printf(" %s", simcore::BUILTIN_PRESET_NAMES[i]);
    Serial.println();
    if (sd_ready()) {
        Serial.printf("  on SD %s:\n", SD_PRESET_DIR);
        sd_list(SD_PRESET_DIR);
    }
}

// Detaches the profile from the tick before the RAM is reused.
static bool detach_profile() {
    s_cmd = SimCmd{};
    s_cmd.type = CmdType::ProfileSet;
    s_cmd.profile = nullptr;
    if (!sim_post(s_cmd)) return false;
    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (sim_get_snapshot().profile == nullptr) return true;
    }
    return false;
}

static void cmd_profile_load(const char *file) {
    if (sim_get_snapshot().profile_state == loads::ProfileState::Running)
        return reply_err("stop the profile first");
    if (!detach_profile()) return reply_err("busy, try again");
    uint32_t t0 = millis();
    uint32_t line = 0;
    const char *err = sd_load_profile(file, s_profile, line);
    if (err) {
        if (line) Serial.printf("err %s at line %lu\n", err, (unsigned long)line);
        else reply_err(err);
        return;
    }
    s_cmd = SimCmd{};
    s_cmd.type = CmdType::ProfileSet;
    s_cmd.profile = &s_profile;
    if (!sim_post(s_cmd)) return reply_err("busy, try again");
    Serial.printf("ok rows=%lu seconds=%.2f inertia=%d load_ms=%lu\n",
                  (unsigned long)s_profile.rows(), s_profile.rows() * 0.01f,
                  s_profile.has_inertia() ? 1 : 0, (unsigned long)(millis() - t0));
}

static void cmd_stream(int argc, char **argv) {
    int32_t on = 0, div = 1;
    if (argc < 2 || argc > 3 || !simproto::parse_int(argv[1], on) || (on != 0 && on != 1) ||
        (argc == 3 && (!simproto::parse_int(argv[2], div) || div < 1 || div > 1000)))
        return reply_err("usage: stream <0|1> [divider 1-1000]");
    sim_set_stream(false, 1);
    sim_stream_clear();
    s_streaming = on != 0;
    reply_ok();
    if (s_streaming) {
        Serial.println(simproto::CSV_HEADER);
        sim_set_stream(true, (uint32_t)div);
    }
}

static void handle_line(char *line) {
    char *argv[8];
    int argc = simproto::tokenize(line, argv, 8);
    if (argc == 0) return;
    const char *c = argv[0];

    if (strcmp(c, "help") == 0) return cmd_help();
    if (strcmp(c, "stat") == 0) return cmd_stat();
    if (strcmp(c, "params") == 0) return cmd_params();
    if (strcmp(c, "save") == 0) return cmd_save();
    if (strcmp(c, "preset") == 0) return cmd_preset(argc, argv);
    if (strcmp(c, "presets") == 0) return cmd_presets();
    if (strcmp(c, "stream") == 0) return cmd_stream(argc, argv);
    if (strcmp(c, "profile") == 0 && argc >= 2 && strcmp(argv[1], "load") == 0) {
        if (argc != 3) return reply_err("usage: profile load <file>");
        return cmd_profile_load(argv[2]);
    }
    if (strcmp(c, "profile") == 0 && argc == 2 && strcmp(argv[1], "list") == 0) {
        if (!sd_ready()) return reply_err("no SD card");
        Serial.printf("ok %s:\n", SD_PROFILE_DIR);
        return sd_list(SD_PROFILE_DIR);
    }

    s_cmd = SimCmd{};
    const char *err = simproto::parse_sim_command(argc, argv, s_cmd);
    if (err) return reply_err(err);
    post(s_cmd);
}

static void console_task(void *) {
    simproto::LineReader reader;
    char line[simproto::LineReader::MAX_LINE + 1];
    char csv[96];
    uint32_t last_led = 0;
    uint32_t last_bus_off_count = 0, bus_off_at = 0;
    bool bus_off_seen = false;

    for (;;) {
        while (Serial.available() > 0) {
            if (!reader.feed((char)Serial.read())) continue;
            if (reader.overflowed()) {
                reply_err("line too long");
                continue;
            }
            strcpy(line, reader.line());
            handle_line(line);
        }

        // Stream: a bounded number of lines per pass so commands stay responsive
        simproto::StreamSample smp;
        for (int i = 0; s_streaming && i < 16 && sim_stream_pop(smp); i++) {
            simproto::format_csv(smp, csv, sizeof(csv));
            Serial.println(csv);
        }

        uint32_t now = millis();
        if (now - last_led >= 100) {
            last_led = now;
            CanStats c = can_get_stats();
            if (c.bus_off_count != last_bus_off_count) {
                last_bus_off_count = c.bus_off_count;
                bus_off_at = now;
                bus_off_seen = true;
            }
            bool bus_off = c.state == 2 || c.state == 3 ||
                           (bus_off_seen && now - bus_off_at < STATUS_WINDOW_MS);
            status_led_update(sim_get_snapshot(), bus_off);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void console_start_task() {
    xTaskCreatePinnedToCore(console_task, "console", CONSOLE_STACK_BYTES, nullptr, CONSOLE_TASK_PRIO, nullptr,
                            CONSOLE_TASK_CORE);
}
