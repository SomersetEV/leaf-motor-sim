// Software-in-the-loop harness (plan section 16, M5c): the Zombie's own
// governor code (zombie_side.cpp) in closed loop with the simulator's own
// tick (lib/simengine), on the PC, in simulated time.
//
// It speaks the simulator's serial console on stdin and stdout, so
// tools/sweep.py and tools/run_script.py drive it as they drive the board
// (--port sil, through tools/silport.py). Two additions, for the driver only:
//
//   #run <n>              advance n ticks of 10 ms; bus log lines are printed if on
//   pedal <pct>           the Zombie's pedal, in percent after its pedal map (default 100)
//   zparam <name> <value> set any Zombie parameter (zset allows only the governor's)
//
// Every input line's output ends with a line "#end". Bus log lines use the
// candump format of the simulator's canlog, with the same filter, so loglib
// and sweep.score() read them unchanged.
//
// Each 10 ms tick runs the two nodes in the order and phase measured on the
// bench (3 Nov sweep logs; see docs/bench/M5c_sil.md):
//
//   +1.0 ms  Zombie Ms10Task: governor on the speed it read last tick,
//            0x4A0/0x4A1, then 0x1D4 carrying last tick's request, then it
//            reads the speed of the latest 0x1DA, sent the tick before.
//   +4.2 ms  Simulator tick: queued console commands, the 0x1D4 latched
//            above, loads, model, 0x1DA and telemetry.
//
// That gives the bench's three ticks from a speed sample to the torque it
// causes. At some clock phases the bench has four; set rptdelay 10 for those.
//
//   sil [--sd DIR] [--pedal PCT] [--zombie-commit HASH]
//   sil --golden      runs the Zombie's governor golden test and exits
//
// DIR stands in for the SD card: DIR/presets/<name>.json and DIR/profiles/.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <deque>
#include <string>

#include "cmd_latch.h"
#include "commands.h"
#include "config.h"
#include "console_parse.h"
#include "param_table.h"
#include "presets.h"
#include "profile.h"
#include "sim_engine.h"
#include "telemetry.h"
#include "zombie_sdo.h"
#include "zombie_side.h"
#include "../../src/sim_defaults.h"

using simproto::CmdType;
using simproto::SimCmd;

static const uint64_t T0_US = 1000000000ull;  // log time of tick 0, 1000 s
static const uint32_t Z_PHASE_US = 1000;     // Zombie's tick after the 10 ms boundary
static const uint32_t S_PHASE_US = 4200;     // simulator's tick

static std::string g_sd;
static float g_pedal = 100.0f;
static bool g_canlog = false;
static bool g_canlog_all = false;
static uint64_t g_now_us = T0_US;            // time stamp for the next logged frame
static uint32_t g_tick = 0;

static void log_frame(uint32_t id, const uint8_t *d, uint8_t dlc) {
    if (!g_canlog) return;
    if (!g_canlog_all) {
        bool wanted = false;
        for (uint32_t w : CANLOG_IDS) wanted |= (w == id);
        if (!wanted) return;
    }
    simproto::CanLogFrame f;
    f.t_us = g_now_us;
    f.id = id;
    f.dlc = dlc;
    memcpy(f.data, d, dlc);
    char line[simproto::CANDUMP_LINE_MAX];
    simproto::format_candump(f, line, sizeof(line));
    puts(line);
    g_now_us += 12;  // frames in one burst a few microseconds apart, in order
}

// The simulator's frames: logged, and 0x1DA delivered to the Zombie
class Bus : public simengine::FrameSink {
public:
    void send(uint32_t id, const uint8_t *data, uint8_t dlc) override {
        log_frame(id, data, dlc);
        if (id == ID_LEAF_INV_STATUS) zside::receive_1da(data);
    }
};

static simengine::Config engine_config() {
    return simengine::Config{TICK_MS, SUBSTEPS_PER_TICK, SLOW_PERIOD_TICKS, LOAD_NOISE_SEED,
                             SPEED_NOISE_SEED, SIM_REPORT_TORQUE != 0, ID_LEAF_INV_STATUS,
                             ID_LEAF_INV_TEMPS, ID_SIM_FAST, ID_SIM_SLOW, ID_SIM_EVENT};
}

static simengine::Engine *g_engine;
static simcore::CmdLatch g_latch(RX_TIMEOUT_MS);
static Bus g_bus;
static std::deque<SimCmd> g_queue;     // console commands for the next tick
static loads::Profile g_profile;
static SimCmd g_cmd;                   // large (holds a preset), so not on the stack

static uint32_t now_ms() { return g_tick * TICK_MS; }

static void reply_ok() { puts("ok"); }
static void reply_err(const char *msg) { printf("err %s\n", msg); }

static void run_ticks(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        uint64_t base = T0_US + (uint64_t)g_tick * TICK_MS * 1000;

        // Zombie Ms10Task
        g_now_us = base + Z_PHASE_US;
        zside::Ms10Out z = zside::ms10(g_pedal);
        if (z.govlog) {
            log_frame(0x4A0, z.f4a0, 8);
            log_frame(0x4A1, z.f4a1, 8);
        }
        log_frame(ID_LEAF_TORQUE_CMD, z.f1d4, 8);
        if (z.slow) {
            log_frame(0x4A2, z.f4a2, 8);
            log_frame(0x4A3, z.f4a3, 8);
        }
        g_latch.accept(z.f1d4, 8, now_ms(), g_engine->params().k_t);

        // Simulator tick
        g_now_us = base + S_PHASE_US;
        while (!g_queue.empty()) {
            g_engine->apply(g_queue.front(), now_ms(), g_bus);
            g_queue.pop_front();
        }
        simengine::TickIn in{};
        in.now_ms = now_ms();
        in.fresh = g_latch.current(now_ms(), in.cmd);
        in.rejected_1d4 = g_latch.rejected();
        g_engine->tick(in, g_bus);
        g_tick++;
    }
    fflush(stdout);
}

static void post(const SimCmd &c) {
    g_queue.push_back(c);
    reply_ok();
}

static bool read_file(const std::string &path, std::string &out) {
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return false;
    char buf[4096];
    size_t n;
    out.clear();
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    fclose(f);
    return true;
}

static bool safe_name(const char *name) {
    return name[0] && !strchr(name, '/') && !strchr(name, '\\') && !strstr(name, "..");
}

static void cmd_preset(int argc, char **argv) {
    if (argc != 2) return reply_err("usage: preset <name>");
    if (!safe_name(argv[1])) return reply_err("bad preset name");
    g_cmd = SimCmd{};
    g_cmd.type = CmdType::Preset;
    std::string json;
    if (!g_sd.empty() && read_file(g_sd + "/presets/" + argv[1] + ".json", json)) {
        if (json.size() >= PRESET_JSON_MAX) return reply_err("preset file too large");
        char bad[32] = "";
        const char *err = simcore::preset_from_json(argv[1], json.c_str(), default_sim_params(),
                                                    g_cmd.preset, bad, sizeof(bad));
        if (err) {
            printf("err %s.json: %s%s%s\n", argv[1], err, bad[0] ? " " : "", bad);
            return;
        }
    } else if (!simcore::builtin_preset(argv[1], default_sim_params(), g_cmd.preset)) {
        return reply_err("unknown preset, try presets");
    }
    post(g_cmd);
}

static void cmd_profile_load(const char *file) {
    if (g_engine->loads().profile_state() == loads::ProfileState::Running)
        return reply_err("stop the profile first");
    if (g_sd.empty()) return reply_err("no SD card");
    if (!safe_name(file)) return reply_err("bad file name");
    // Detach before the rows are reused, as the firmware does
    SimCmd det{};
    det.type = CmdType::ProfileSet;
    det.profile = nullptr;
    g_engine->apply(det, now_ms(), g_bus);
    std::string path = g_sd + "/profiles/" + file + (strchr(file, '.') ? "" : ".csv");
    FILE *f = fopen(path.c_str(), "r");
    if (!f) return reply_err("file not found");
    loads::ProfileCsvParser csv(g_profile);
    char buf[96];
    const char *err = nullptr;
    uint32_t line = 0;
    while (fgets(buf, sizeof(buf), f)) {
        size_t n = strlen(buf);
        if (n == sizeof(buf) - 1 && buf[n - 1] != '\n') {
            err = "line too long";
            line = csv.line_number() + 1;
            break;
        }
        while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = '\0';
        if (!csv.line(buf)) {
            err = csv.error();
            line = csv.line_number();
            break;
        }
    }
    fclose(f);
    if (!err && !csv.finish()) err = csv.error();
    if (err) {
        g_profile.clear();
        if (line) printf("err %s at line %lu\n", err, (unsigned long)line);
        else reply_err(err);
        return;
    }
    g_cmd = SimCmd{};
    g_cmd.type = CmdType::ProfileSet;
    g_cmd.profile = &g_profile;
    g_queue.push_back(g_cmd);
    printf("ok rows=%lu seconds=%.2f inertia=%d load_ms=0\n", (unsigned long)g_profile.rows(),
           g_profile.rows() * 0.01f, g_profile.has_inertia() ? 1 : 0);
}

static void cmd_params() {
    const SimParams &p = g_engine->params();
    printf("ok");
    for (int i = 0; i < simcore::PARAM_COUNT; i++)
        printf(" %s=%g", simcore::PARAMS[i].name, simcore::param_get(p, i));
    printf("\n");
    for (int i = 0; i < simcore::PARAM_COUNT; i++)
        printf("  %-8s %-11s %g .. %g\n", simcore::PARAMS[i].name, simcore::PARAMS[i].unit,
               simcore::PARAMS[i].min, simcore::PARAMS[i].max);
}

static void cmd_stat() {
    const plant::Plant &m = g_engine->plant();
    printf("ok t_ms=%lu rpm=%.1f t_motor=%.2f t_load=%.2f J=%.3f preset=%s sil=1\n",
           (unsigned long)now_ms(), m.rpm(), m.t_motor_nm(), m.t_load_nm(), m.params().j_kgm2,
           g_engine->preset_id() ? g_engine->preset_name() : "-");
}

static const simproto::ZParam *zparam_arg(const char *name) {
    const simproto::ZParam *p = simproto::zparam_find(name);
    if (!p) reply_err("not allowed (revlim govKp govKi govKd govImax govFlt GovLog)");
    return p;
}

static void cmd_zget(int argc, char **argv) {
    if (argc != 2) return reply_err("usage: zget <name>");
    const simproto::ZParam *p = zparam_arg(argv[1]);
    if (!p) return;
    float v = 0;
    zside::get(p->name, v);
    printf("ok %s=%g\n", p->name, v);
}

// As the firmware: reads the old value, writes with the SDO's fixed-point
// value, reads back, then posts the 0x4B2 event for the next tick.
static void cmd_zset(int argc, char **argv) {
    float v = 0;
    if (argc != 3 || !simproto::parse_float(argv[2], v)) return reply_err("usage: zset <name> <value>");
    const simproto::ZParam *p = zparam_arg(argv[1]);
    if (!p) return;
    float old = 0, now = 0;
    zside::get(p->name, old);
    uint8_t req[8];
    simproto::sdo_write_request(p->id, v, req);
    int32_t raw = (int32_t)((uint32_t)req[4] | (uint32_t)req[5] << 8 | (uint32_t)req[6] << 16 |
                            (uint32_t)req[7] << 24);
    const char *err = zside::set_raw(p->name, raw);
    if (err) return reply_err(err);
    zside::get(p->name, now);
    g_cmd = SimCmd{};
    g_cmd.type = CmdType::ZParamEvent;
    g_cmd.i = p->id;
    g_queue.push_back(g_cmd);
    printf("ok %s=%g was=%g\n", p->name, now, old);
}

static void handle_line(char *line) {
    char *argv[8];
    int argc = simproto::tokenize(line, argv, 8);
    if (argc == 0) return;
    const char *c = argv[0];
    int32_t n = 0;
    float f = 0;

    if (strcmp(c, "#run") == 0) {
        if (argc != 2 || !simproto::parse_int(argv[1], n) || n < 0) return reply_err("usage: #run <ticks>");
        return run_ticks((uint32_t)n);
    }
    if (strcmp(c, "zparam") == 0) {  // any Zombie parameter, for what-ifs (sil only)
        if (argc != 3 || !simproto::parse_float(argv[2], f)) return reply_err("usage: zparam <name> <value>");
        const char *err = zside::set_raw(argv[1], (int32_t)lroundf(f * 32.0f));
        if (err) return reply_err(err);
        float now = 0;
        zside::get(argv[1], now);
        return (void)printf("ok %s=%g\n", argv[1], now);
    }
    if (strcmp(c, "pedal") == 0) {
        if (argc != 2 || !simproto::parse_float(argv[1], f) || f < -100 || f > 100)
            return reply_err("usage: pedal <-100..100>");
        g_pedal = f;
        return reply_ok();
    }
    if (strcmp(c, "help") == 0) return (void)puts("ok sil: the simulator console, plus #run <ticks> and pedal <pct>");
    if (strcmp(c, "stat") == 0) return cmd_stat();
    if (strcmp(c, "params") == 0) return cmd_params();
    if (strcmp(c, "save") == 0) return reply_ok();
    if (strcmp(c, "preset") == 0) return cmd_preset(argc, argv);
    if (strcmp(c, "presets") == 0) {
        printf("ok built in:");
        for (int i = 0; i < simcore::BUILTIN_PRESET_COUNT; i++) printf(" %s", simcore::BUILTIN_PRESET_NAMES[i]);
        printf("\n");
        return;
    }
    if (strcmp(c, "stream") == 0) {
        if (argc >= 2 && strcmp(argv[1], "0") == 0) return reply_ok();
        return reply_err("stream is not available in sil; use canlog");
    }
    if (strcmp(c, "canlog") == 0) {
        bool all = argc == 3 && strcmp(argv[2], "all") == 0;
        if (argc < 2 || argc > 3 || (argc == 3 && !all) || !simproto::parse_int(argv[1], n) ||
            (n != 0 && n != 1))
            return reply_err("usage: canlog <0|1> [all]");
        g_canlog = n == 1;
        g_canlog_all = all;
        return reply_ok();
    }
    if (strcmp(c, "zget") == 0) return cmd_zget(argc, argv);
    if (strcmp(c, "zset") == 0) return cmd_zset(argc, argv);
    if (strcmp(c, "profile") == 0 && argc >= 2 && strcmp(argv[1], "load") == 0) {
        if (argc != 3) return reply_err("usage: profile load <file>");
        return cmd_profile_load(argv[2]);
    }
    if (strcmp(c, "profile") == 0 && argc == 2 && strcmp(argv[1], "list") == 0)
        return reply_err("profile list is not available in sil");

    g_cmd = SimCmd{};
    const char *err = simproto::parse_sim_command(argc, argv, g_cmd);
    if (err) return reply_err(err);
    post(g_cmd);
}

int main(int argc, char **argv) {
    const char *commit = "unknown";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--golden") == 0) {
            int first = -1;
            int bad = zside::golden_mismatches(&first);
            if (bad) printf("golden FAIL: %d ticks differ, first at tick %d\n", bad, first);
            else printf("golden PASS: 500 ticks bit for bit\n");
            return bad ? 1 : 0;
        } else if (strcmp(argv[i], "--sd") == 0 && i + 1 < argc) {
            g_sd = argv[++i];
        } else if (strcmp(argv[i], "--pedal") == 0 && i + 1 < argc) {
            g_pedal = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--zombie-commit") == 0 && i + 1 < argc) {
            commit = argv[++i];
        } else {
            fprintf(stderr, "usage: sil [--sd DIR] [--pedal PCT] [--zombie-commit HASH] | sil --golden\n");
            return 2;
        }
    }
    zside::init();
    static simengine::Engine engine(engine_config(), default_sim_params());
    g_engine = &engine;

    printf("#sil zombie=%s pedal=%g sd=%s left_out=pedal_map,UdcLimit,IdcLimit,TemperatureDerate\n",
           commit, g_pedal, g_sd.empty() ? "-" : g_sd.c_str());
    puts("#end");
    fflush(stdout);

    simproto::LineReader reader;
    char line[simproto::LineReader::MAX_LINE + 1];
    int ch;
    while ((ch = getchar()) != EOF) {
        if (!reader.feed((char)ch)) continue;
        if (reader.overflowed()) reply_err("line too long");
        else {
            strcpy(line, reader.line());
            handle_line(line);
        }
        puts("#end");
        fflush(stdout);
    }
    return 0;
}
