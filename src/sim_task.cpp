#include "sim_task.h"

#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>
#include <freertos/queue.h>
#include <string.h>

#include "can_io.h"
#include "config.h"
#include "sim_engine.h"
#include "spsc_ring.h"

using simproto::SimCmd;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static SimSnapshot s_snap;
static QueueHandle_t s_cmd_queue;

static simcore::SpscRing<simproto::StreamSample, STREAM_RING_LEN> s_ring;
static std::atomic<bool> s_stream_on{false};
static std::atomic<uint32_t> s_stream_div{1};

static SimParams s_initial;  // parameters at power-up, until the task builds the engine

// Frames from the engine go straight to the CAN driver, which also copies
// them to the bus log.
class CanSink : public simengine::FrameSink {
public:
    void send(uint32_t id, const uint8_t *data, uint8_t dlc) override { can_send(id, data, dlc); }
};

static simengine::Config engine_config() {
    simengine::Config c;
    c.tick_ms = TICK_MS;
    c.substeps = SUBSTEPS_PER_TICK;
    c.slow_period_ticks = SLOW_PERIOD_TICKS;
    c.load_noise_seed = LOAD_NOISE_SEED;
    c.speed_noise_seed = SPEED_NOISE_SEED;
    c.report_torque = SIM_REPORT_TORQUE != 0;
    c.id_inv_status = ID_LEAF_INV_STATUS;
    c.id_inv_temps = ID_LEAF_INV_TEMPS;
    c.id_sim_fast = ID_SIM_FAST;
    c.id_sim_slow = ID_SIM_SLOW;
    c.id_sim_event = ID_SIM_EVENT;
    return c;
}

// Nothing in here may block: no serial, no SD, no heap (plan rule 4). The
// tick itself is lib/simengine, which the PC harness (tools/sil) also runs.
static void sim_task(void *) {
    simcore::TickStats stats(TICK_MS * 1000, TICK_TOLERANCE_US, JITTER_WINDOW_TICKS);
    static simengine::Engine engine(engine_config(), s_initial);  // static: off the task stack
    CanSink sink;
    TickType_t wake = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    SimCmd cmd_in;

    for (;;) {
        BaseType_t delayed = xTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
        int64_t now_us = esp_timer_get_time();
        if (engine.ticks() > 0) stats.record((uint32_t)(now_us - prev_us), delayed == pdFALSE);
        prev_us = now_us;
        uint32_t now_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());

        // Console commands, applied between ticks
        while (xQueueReceive(s_cmd_queue, &cmd_in, 0) == pdTRUE) engine.apply(cmd_in, now_ms, sink);

        // The latest torque request (zero torque after the 100 ms timeout),
        // then the tick: loads, model, 0x1DA, telemetry
        simengine::TickIn in;
        in.now_ms = now_ms;
        in.fresh = can_get_cmd(now_ms, in.cmd);
        in.bus_error = can_bus_error_within(now_ms, STATUS_WINDOW_MS);
        in.rejected_1d4 = can_rejected_1d4();
        in.overruns = stats.overruns();
        in.worst_jitter_us = stats.window_worst_jitter_us();
        uint32_t tick = engine.ticks();
        simengine::TickOut o = engine.tick(in, sink);
        const plant::Plant &model = engine.plant();
        const loads::Loads &loads = engine.loads();

        // Sample for CSV streaming; dropped (and counted) if the console lags
        uint32_t div = s_stream_div.load(std::memory_order_relaxed);
        if (s_stream_on.load(std::memory_order_relaxed) && tick % div == 0) {
            simproto::StreamSample smp{now_ms, in.cmd.raw, o.t_req_nm, model.t_motor_nm(),
                                       model.t_load_nm(), o.rpm, o.flags};
            s_ring.push(smp);
        }

        // Snapshot for the console (tick period was recorded at the top)
        portENTER_CRITICAL(&s_mux);
        s_snap.sim_ms        = now_ms;
        s_snap.cmd           = in.cmd;
        s_snap.cmd_fresh     = in.fresh;
        s_snap.t_req_nm      = o.t_req_nm;
        s_snap.t_motor_nm    = model.t_motor_nm();
        s_snap.t_load_nm     = model.t_load_nm();
        s_snap.rpm           = o.rpm;
        s_snap.inertia_kgm2  = o.inertia_kgm2;
        s_snap.clipping      = model.clipping();
        s_snap.inv_fault     = engine.inv_fault();
        s_snap.flags         = o.flags;
        s_snap.params        = engine.params();
        s_snap.preset_id     = engine.preset_id();
        memcpy(s_snap.preset_name, engine.preset_name(), sizeof(s_snap.preset_name));
        s_snap.loads         = loads.config();
        s_snap.step_level_nm = loads.step_level_nm();
        s_snap.profile_state = loads.profile_state();
        s_snap.profile       = loads.profile();
        s_snap.profile_row   = loads.profile_row();
        s_snap.profile_loop  = loads.profile_loop();
        s_snap.profile_exp_n = loads.profile_exponent();
        s_snap.profile_wref_rpm = loads.profile_wref_rpm();
        s_snap.stats         = stats;
        portEXIT_CRITICAL(&s_mux);
    }
}

void sim_start_task(const SimParams &initial) {
    s_initial = initial;
    s_cmd_queue = xQueueCreate(SIM_CMD_QUEUE_LEN, sizeof(SimCmd));
    s_snap = SimSnapshot{};
    s_snap.params = initial;
    s_snap.inertia_kgm2 = initial.plant.j_kgm2;
    s_snap.profile_state = loads::ProfileState::Idle;
    s_snap.stats = simcore::TickStats(TICK_MS * 1000, TICK_TOLERANCE_US, JITTER_WINDOW_TICKS);
    xTaskCreatePinnedToCore(sim_task, "sim", TASK_STACK_BYTES, nullptr,
                            SIM_TASK_PRIO, nullptr, SIM_TASK_CORE);
}

bool sim_post(const SimCmd &c) {
    return xQueueSend(s_cmd_queue, &c, 0) == pdTRUE;
}

SimSnapshot sim_get_snapshot() {
    portENTER_CRITICAL(&s_mux);
    SimSnapshot s = s_snap;
    portEXIT_CRITICAL(&s_mux);
    return s;
}

void sim_set_stream(bool on, uint32_t divider) {
    s_stream_div.store(divider ? divider : 1, std::memory_order_relaxed);
    s_stream_on.store(on, std::memory_order_relaxed);
}

bool sim_stream_pop(simproto::StreamSample &s) { return s_ring.pop(s); }
void sim_stream_clear() { s_ring.clear(); }
uint32_t sim_stream_dropped() { return s_ring.dropped(); }
