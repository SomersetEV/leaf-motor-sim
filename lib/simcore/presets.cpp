#include "presets.h"

#include <string.h>

#include "json_flat.h"
#include "param_table.h"

namespace simcore {

const char *const BUILTIN_PRESET_NAMES[BUILTIN_PRESET_COUNT] = {
    "noload", "topper", "baler", "pto_step"};

static void set_name(Preset &p, const char *name) {
    strncpy(p.name, name, sizeof(p.name) - 1);
    p.name[sizeof(p.name) - 1] = '\0';
}

// Placeholder numbers (plan section 6): replace once tractor logs are fitted.
bool builtin_preset(const char *name, const SimParams &base, Preset &out) {
    out = Preset{};
    out.params = base;
    out.loads = loads::LoadConfig{};
    set_name(out, name);

    if (strcmp(name, "noload") == 0) {
        out.id = 1;  // base model, no loads
    } else if (strcmp(name, "topper") == 0) {
        out.id = 2;  // blades: added inertia, quadratic drag, draft noise
        out.params.plant.j_kgm2 = 0.60f;
        out.params.plant.c_nms2 = 0.0004f;
        out.loads.noise_on = true;
        out.loads.noise_nm = 3.0f;
        out.loads.noise_hz = 2.0f;
    } else if (strcmp(name, "baler") == 0) {
        out.id = 3;  // plunger: periodic half-sine pulse
        out.params.plant.j_kgm2 = 0.90f;
        out.params.plant.tc_nm = 10.0f;
        out.loads.pulse_on = true;
        out.loads.pulse_nm = 150.0f;
        out.loads.pulse_period_ms = 700.0f;
        out.loads.pulse_duty_pct = 40.0f;
    } else if (strcmp(name, "pto_step") == 0) {
        out.id = 4;  // PTO engagement: torque ramp, inertia added at its end
        out.loads.step_on = true;
        out.loads.step_nm = 60.0f;
        out.loads.step_ramp_ms = 500.0f;
        out.loads.step_dj_kgm2 = 0.30f;
    } else {
        return false;
    }
    return true;
}

struct JsonCtx {
    Preset *p;
    const char *bad;
    char bad_key[32];
};

static bool apply_member(const char *key, float v, void *vctx) {
    JsonCtx &c = *(JsonCtx *)vctx;
    loads::LoadConfig &l = c.p->loads;
    int idx = param_index(key);
    bool ok = true;
    if (idx >= 0) {
        ok = param_set(c.p->params, idx, v);
        if (!ok) c.bad = "value out of range";
    } else if (strcmp(key, "step_nm") == 0) {
        l.step_nm = v; l.step_on = v > 0.0f;
    } else if (strcmp(key, "step_ramp_ms") == 0) {
        l.step_ramp_ms = v;
    } else if (strcmp(key, "step_dj") == 0) {
        l.step_dj_kgm2 = v;
    } else if (strcmp(key, "pulse_nm") == 0) {
        l.pulse_nm = v; l.pulse_on = v > 0.0f;
    } else if (strcmp(key, "pulse_period_ms") == 0) {
        l.pulse_period_ms = v;
    } else if (strcmp(key, "pulse_duty_pct") == 0) {
        l.pulse_duty_pct = v;
    } else if (strcmp(key, "noise_nm") == 0) {
        l.noise_nm = v; l.noise_on = v > 0.0f;
    } else if (strcmp(key, "noise_hz") == 0) {
        l.noise_hz = v;
    } else if (strcmp(key, "grad_nm") == 0) {
        l.grad_nm = v;
    } else {
        ok = false;
        c.bad = "unknown key";
    }
    if (!ok) {
        strncpy(c.bad_key, key, sizeof(c.bad_key) - 1);
        c.bad_key[sizeof(c.bad_key) - 1] = '\0';
    }
    return ok;
}

const char *preset_from_json(const char *name, const char *json, const SimParams &base,
                             Preset &out, char *bad_key, int bad_key_len) {
    if (!builtin_preset(name, base, out)) {
        out = Preset{};
        out.params = base;
        out.loads = loads::LoadConfig{};
        out.id = PRESET_ID_FILE;
        set_name(out, name);
    }
    JsonCtx ctx{&out, nullptr, {0}};
    const char *err = json_flat_parse(json, apply_member, &ctx);
    if (err && ctx.bad) err = ctx.bad;
    if (err && bad_key && bad_key_len > 0) {
        strncpy(bad_key, ctx.bad_key, bad_key_len - 1);
        bad_key[bad_key_len - 1] = '\0';
    }
    return err;
}

} // namespace simcore
