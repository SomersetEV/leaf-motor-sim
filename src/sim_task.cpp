#include "sim_task.h"

#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>
#include <freertos/queue.h>
#include <string.h>

#include "can_io.h"
#include "config.h"
#include "gauss.h"
#include "param_table.h"
#include "spsc_ring.h"

using simproto::CmdType;
using simproto::SimCmd;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static SimSnapshot s_snap;
static QueueHandle_t s_cmd_queue;

static simcore::SpscRing<simproto::StreamSample, STREAM_RING_LEN> s_ring;
static std::atomic<bool> s_stream_on{false};
static std::atomic<uint32_t> s_stream_div{1};

// State owned by the tick task
static SimParams s_params;
static plant::Plant *s_plant;
static loads::Loads *s_loads;
static bool s_inv_fault = false;
static uint8_t s_preset_id = 0;
static char s_preset_name[16] = "";

static void send_event(uint8_t ev, uint16_t value, uint32_t now_ms) {
    uint8_t b[8];
    simproto::pack_4b2(ev, value, now_ms, b);
    can_send(ID_SIM_EVENT, b, 8);
}

static uint16_t nm_value(float nm) { return (uint16_t)(int16_t)lroundf(nm); }

// Applies one queued command. Runs at the start of a tick.
static void apply(const SimCmd &c, uint32_t now_ms) {
    switch (c.type) {
    case CmdType::SetParam:
        simcore::param_set(s_params, c.i, c.f[0]);
        s_plant->set_params(s_params.plant);
        send_event(simproto::EV_PARAM, (uint16_t)c.i, now_ms);
        break;
    case CmdType::Preset:
        s_params = c.preset.params;
        s_plant->set_params(s_params.plant);
        s_loads->configure(c.preset.loads);
        s_preset_id = c.preset.id;
        memcpy(s_preset_name, c.preset.name, sizeof(s_preset_name));
        send_event(simproto::EV_PRESET, c.preset.id, now_ms);
        break;
    case CmdType::LoadStep:
        s_loads->start_step(c.f[0], c.f[1], c.f[2]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms);
        break;
    case CmdType::LoadPulse:
        s_loads->start_pulse(c.f[0], c.f[1], c.f[2]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms);
        break;
    case CmdType::LoadGrad:
        s_loads->set_grad(c.f[0]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms);
        break;
    case CmdType::LoadNoise:
        s_loads->set_noise(c.f[0], c.f[1]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms);
        break;
    case CmdType::LoadOff:
        s_loads->off();
        send_event(simproto::EV_LOAD, 0, now_ms);
        break;
    case CmdType::ProfileSet:
        s_loads->set_profile(c.profile);
        break;
    case CmdType::ProfileStart:
        if (s_loads->profile_start()) send_event(simproto::EV_PROFILE_START, 0, now_ms);
        break;
    case CmdType::ProfileStop:
        if (s_loads->profile_state() == loads::ProfileState::Running) {
            uint32_t row = s_loads->profile_row();
            s_loads->profile_stop();
            send_event(simproto::EV_PROFILE_STOP, (uint16_t)(row > 65535 ? 65535 : row), now_ms);
        }
        break;
    case CmdType::ProfileLoop:
        s_loads->set_profile_loop(c.i != 0);
        break;
    case CmdType::ProfileExp:
        s_loads->set_profile_exponent(c.f[0], c.f[1]);
        break;
    case CmdType::Mark:
        send_event(simproto::EV_MARKER, (uint16_t)c.i, now_ms);
        break;
    case CmdType::FaultInv:
        s_inv_fault = c.i != 0;
        break;
    }
}

// Nothing in here may block: no serial, no SD, no heap (plan rule 4).
static void sim_task(void *) {
    simcore::TickStats stats(TICK_MS * 1000, TICK_TOLERANCE_US, JITTER_WINDOW_TICKS);
    plant::Plant model(s_params.plant);
    loads::Loads loads(LOAD_NOISE_SEED);
    loads::Gauss speed_noise(SPEED_NOISE_SEED);
    s_plant = &model;
    s_loads = &loads;

    const float dt_tick = TICK_MS / 1000.0f;
    const float dt_sub  = dt_tick / SUBSTEPS_PER_TICK;
    TickType_t wake = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    uint32_t tick = 0;
    uint8_t b[8];
    SimCmd cmd_in;

    for (;;) {
        BaseType_t delayed = xTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));
        int64_t now_us = esp_timer_get_time();
        if (tick > 0) stats.record((uint32_t)(now_us - prev_us), delayed == pdFALSE);
        prev_us = now_us;
        uint32_t now_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());

        // Console commands, applied between ticks
        while (xQueueReceive(s_cmd_queue, &cmd_in, 0) == pdTRUE) apply(cmd_in, now_ms);

        // 1. Latest torque request; zero torque after the 100 ms timeout
        leafcodec::Cmd1D4 cmd;
        bool fresh = can_get_cmd(now_ms, cmd);
        float t_req = fresh ? cmd.raw * s_params.k_t : 0.0f;

        // 2. Loads and profile advance one tick; inertia follows them
        loads::ProfileState prof_before = loads.profile_state();
        loads::LoadOutput lo = loads.tick(model.w_rad_s(), dt_tick);
        float j = lo.profile_inertia ? lo.profile_inertia_kgm2
                                     : s_params.plant.j_kgm2 + lo.inertia_delta_kgm2;
        model.set_inertia(j);
        if (prof_before == loads::ProfileState::Running &&
            loads.profile_state() == loads::ProfileState::Finished) {
            uint32_t rows = loads.profile() ? loads.profile()->rows() : 0;
            send_event(simproto::EV_PROFILE_STOP, (uint16_t)(rows > 65535 ? 65535 : rows), now_ms);
        }

        // 3. Ten 1 ms model sub-steps
        for (uint32_t i = 0; i < SUBSTEPS_PER_TICK; i++) model.substep(t_req, lo.tick, dt_sub);
        float rpm = model.rpm();
        // Measurement noise goes on the reported speed only, never the model
        float rpm_reported = rpm;
        if (s_params.noise_rpm > 0.0f) rpm_reported += s_params.noise_rpm * speed_noise.next();

        // 4. Inverter status, then fast telemetry
        leafcodec::encode_1da(s_params.udc_v, rpm_reported, s_inv_fault, b);
        can_send(ID_LEAF_INV_STATUS, b, 8);
        simproto::pack_4b0(rpm, t_req, model.t_motor_nm(), model.t_load_nm(), b);
        can_send(ID_SIM_FAST, b, 8);

        uint8_t flags = 0;
        if (!fresh) flags |= simproto::FLAG_RX_TIMEOUT;
        if (model.clipping()) flags |= simproto::FLAG_CLIPPING;
        if (s_params.noise_rpm > 0.0f || loads.config().noise_on) flags |= simproto::FLAG_NOISE;
        if (s_inv_fault) flags |= simproto::FLAG_FAULT;
        if (can_bus_error_within(now_ms, STATUS_WINDOW_MS)) flags |= simproto::FLAG_BUS_ERROR;

        // 5. Every tenth tick: temperatures and slow telemetry
        if (tick % SLOW_PERIOD_TICKS == 0) {
            leafcodec::encode_55a(s_params.motor_c, s_params.inv_c, b);
            can_send(ID_LEAF_INV_TEMPS, b, 8);
            simproto::SlowTelemetry st{j, s_preset_id, (uint8_t)loads.profile_state(), flags,
                                       can_rejected_1d4(), stats.overruns(),
                                       stats.window_worst_jitter_us()};
            simproto::pack_4b1(st, b);
            can_send(ID_SIM_SLOW, b, 8);
        }

        // 6. Sample for CSV streaming; dropped (and counted) if the console lags
        uint32_t div = s_stream_div.load(std::memory_order_relaxed);
        if (s_stream_on.load(std::memory_order_relaxed) && tick % div == 0) {
            simproto::StreamSample smp{now_ms, cmd.raw, t_req, model.t_motor_nm(),
                                       model.t_load_nm(), rpm, flags};
            s_ring.push(smp);
        }

        // 7. Snapshot for the console (tick period was recorded at the top)
        portENTER_CRITICAL(&s_mux);
        s_snap.sim_ms        = now_ms;
        s_snap.cmd           = cmd;
        s_snap.cmd_fresh     = fresh;
        s_snap.t_req_nm      = t_req;
        s_snap.t_motor_nm    = model.t_motor_nm();
        s_snap.t_load_nm     = model.t_load_nm();
        s_snap.rpm           = rpm;
        s_snap.inertia_kgm2  = j;
        s_snap.clipping      = model.clipping();
        s_snap.inv_fault     = s_inv_fault;
        s_snap.flags         = flags;
        s_snap.params        = s_params;
        s_snap.preset_id     = s_preset_id;
        memcpy(s_snap.preset_name, s_preset_name, sizeof(s_snap.preset_name));
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

        tick++;
    }
}

void sim_start_task(const SimParams &initial) {
    s_params = initial;
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
