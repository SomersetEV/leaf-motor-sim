#include "sim_engine.h"

#include <math.h>
#include <string.h>

#include "param_table.h"
#include "telemetry.h"

namespace simengine {

using simproto::CmdType;

static uint16_t nm_value(float nm) { return (uint16_t)(int16_t)lroundf(nm); }

Engine::Engine(const Config &cfg, const SimParams &initial)
    : cfg_(cfg),
      params_(initial),
      plant_(initial.plant),
      loads_(cfg.load_noise_seed),
      report_(cfg.tick_ms / 1000.0f / cfg.substeps * 1000.0f, cfg.tick_ms / 1000.0f,
              cfg.speed_noise_seed) {}

void Engine::send_event(uint8_t ev, uint16_t value, uint32_t now_ms, FrameSink &out) {
    uint8_t b[8];
    simproto::pack_4b2(ev, value, now_ms, b);
    out.send(cfg_.id_sim_event, b, 8);
}

// Both noise sources restart on preset and mark, so the same script gives the
// same run (plan item I5). The pulse phase restarts on preset (configure) and
// load pulse, but not on mark, which scripts also send mid-stroke.
void Engine::reseed() {
    loads_.reseed(cfg_.load_noise_seed);
    report_.reseed(cfg_.speed_noise_seed);
}

void Engine::apply(const simproto::SimCmd &c, uint32_t now_ms, FrameSink &out) {
    switch (c.type) {
    case CmdType::SetParam:
        // The console range-checks before posting, so this cannot fail today;
        // the event is only sent for a value that was taken.
        if (!simcore::param_set(params_, c.i, c.f[0])) break;
        plant_.set_params(params_.plant);
        send_event(simproto::EV_PARAM, (uint16_t)c.i, now_ms, out);
        break;
    case CmdType::Preset:
        params_ = c.preset.params;
        plant_.set_params(params_.plant);
        loads_.configure(c.preset.loads);
        reseed();
        preset_id_ = c.preset.id;
        memcpy(preset_name_, c.preset.name, sizeof(preset_name_));
        send_event(simproto::EV_PRESET, c.preset.id, now_ms, out);
        break;
    case CmdType::LoadStep:
        loads_.start_step(c.f[0], c.f[1], c.f[2]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms, out);
        break;
    case CmdType::LoadPulse:
        loads_.start_pulse(c.f[0], c.f[1], c.f[2]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms, out);
        break;
    case CmdType::LoadGrad:
        loads_.set_grad(c.f[0]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms, out);
        break;
    case CmdType::LoadNoise:
        loads_.set_noise(c.f[0], c.f[1]);
        send_event(simproto::EV_LOAD, nm_value(c.f[0]), now_ms, out);
        break;
    case CmdType::LoadOff:
        loads_.off();
        send_event(simproto::EV_LOAD, 0, now_ms, out);
        break;
    case CmdType::ProfileSet:
        loads_.set_profile(c.profile);
        break;
    case CmdType::ProfileStart:
        if (loads_.profile_start()) send_event(simproto::EV_PROFILE_START, 0, now_ms, out);
        break;
    case CmdType::ProfileStop:
        if (loads_.profile_state() == loads::ProfileState::Running) {
            uint32_t row = loads_.profile_row();
            loads_.profile_stop();
            send_event(simproto::EV_PROFILE_STOP, (uint16_t)(row > 65535 ? 65535 : row), now_ms, out);
        }
        break;
    case CmdType::ProfileLoop:
        loads_.set_profile_loop(c.i != 0);
        break;
    case CmdType::ProfileExp:
        loads_.set_profile_exponent(c.f[0], c.f[1]);
        break;
    case CmdType::Mark:
        reseed();
        send_event(simproto::EV_MARKER, (uint16_t)c.i, now_ms, out);
        break;
    case CmdType::FaultInv:
        inv_fault_ = c.i != 0;
        break;
    case CmdType::ZParamEvent:
        send_event(simproto::EV_ZPARAM, (uint16_t)c.i, now_ms, out);
        break;
    }
}

TickOut Engine::tick(const TickIn &in, FrameSink &out) {
    const float dt_tick = cfg_.tick_ms / 1000.0f;
    const float dt_sub = dt_tick / cfg_.substeps;
    uint8_t b[8];
    TickOut o;

    // 1. The latest torque request; zero torque after the timeout
    o.t_req_nm = in.fresh ? in.cmd.raw * params_.k_t : 0.0f;

    // 2. Loads and profile advance one tick; inertia follows them
    loads::ProfileState prof_before = loads_.profile_state();
    loads::LoadOutput lo = loads_.tick(plant_.w_rad_s(), dt_tick);
    o.inertia_kgm2 = lo.profile_inertia ? lo.profile_inertia_kgm2
                                        : params_.plant.j_kgm2 + lo.inertia_delta_kgm2;
    plant_.set_inertia(o.inertia_kgm2);
    if (prof_before == loads::ProfileState::Running &&
        loads_.profile_state() == loads::ProfileState::Finished) {
        uint32_t rows = loads_.profile() ? loads_.profile()->rows() : 0;
        send_event(simproto::EV_PROFILE_STOP, (uint16_t)(rows > 65535 ? 65535 : rows), in.now_ms, out);
    }

    // 3. Model sub-steps, each recorded for the report delay
    for (uint32_t i = 0; i < cfg_.substeps; i++) {
        plant_.substep(o.t_req_nm, lo.tick, dt_sub);
        report_.push(plant_.rpm());
    }
    o.rpm = plant_.rpm();
    // Delay and measurement noise go on the reported speed only, never the model
    sensor::ReportParams rp;
    rp.noise_rpm = params_.noise_rpm;
    rp.noise_fc_hz = params_.noise_fc_hz;
    rp.noise_bw_hz = params_.noise_bw_hz;
    rp.delay_ms = params_.rptdelay_ms;
    o.rpm_reported = report_.report(rp);

    // 4. Inverter status, then fast telemetry
    float t_reported = cfg_.report_torque ? plant_.t_motor_nm() : 0.0f;  // after envelope and lag
    leafcodec::encode_1da(params_.udc_v, o.rpm_reported, t_reported, inv_fault_, b);
    out.send(cfg_.id_inv_status, b, 8);
    simproto::pack_4b0(o.rpm, o.t_req_nm, plant_.t_motor_nm(), plant_.t_load_nm(), b);
    out.send(cfg_.id_sim_fast, b, 8);

    o.flags = 0;
    if (!in.fresh) o.flags |= simproto::FLAG_RX_TIMEOUT;
    if (plant_.clipping()) o.flags |= simproto::FLAG_CLIPPING;
    if (params_.noise_rpm > 0.0f || loads_.config().noise_on) o.flags |= simproto::FLAG_NOISE;
    if (inv_fault_) o.flags |= simproto::FLAG_FAULT;
    if (in.bus_error) o.flags |= simproto::FLAG_BUS_ERROR;

    // 5. Every slow period: temperatures and slow telemetry
    if (tick_ % cfg_.slow_period_ticks == 0) {
        leafcodec::encode_55a(params_.motor_c, params_.inv_c, b);
        out.send(cfg_.id_inv_temps, b, 8);
        simproto::SlowTelemetry st{o.inertia_kgm2, preset_id_, (uint8_t)loads_.profile_state(),
                                   o.flags, in.rejected_1d4, in.overruns, in.worst_jitter_us};
        simproto::pack_4b1(st, b);
        out.send(cfg_.id_sim_slow, b, 8);
    }
    tick_++;
    return o;
}

} // namespace simengine
