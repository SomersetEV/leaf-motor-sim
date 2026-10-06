#include "commands.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "param_table.h"

namespace simproto {

bool parse_float(const char *s, float &out) {
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || !isfinite(v)) return false;
    out = (float)v;
    return true;
}

bool parse_int(const char *s, int32_t &out) {
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    out = (int32_t)v;
    return true;
}

static bool is(const char *a, const char *b) { return strcmp(a, b) == 0; }

static const char *parse_bool(const char *s, int32_t &out) {
    if (!parse_int(s, out) || (out != 0 && out != 1)) return "expected 0 or 1";
    return nullptr;
}

// Reads argv[first..] into f[], with defaults for missing optional values.
static const char *parse_floats(int argc, char *const argv[], int first, int required,
                                int total, SimCmd &out) {
    int n = argc - first;
    if (n < required) return "missing value";
    if (n > total) return "too many values";
    for (int k = 0; k < n; k++)
        if (!parse_float(argv[first + k], out.f[k])) return "bad number";
    return nullptr;
}

const char *parse_sim_command(int argc, char *const argv[], SimCmd &out) {
    out.i = 0;
    out.f[0] = out.f[1] = out.f[2] = 0.0f;
    out.profile = nullptr;
    if (argc < 1) return "empty command";
    const char *c = argv[0];

    if (is(c, "set")) {
        if (argc != 3) return "usage: set <name> <value>";
        int idx = simcore::param_index(argv[1]);
        if (idx < 0) return "unknown parameter, try params";
        float v;
        if (!parse_float(argv[2], v)) return "bad number";
        const simcore::ParamInfo &pi = simcore::PARAMS[idx];
        if (v < pi.min || v > pi.max) return "value out of range";
        out.type = CmdType::SetParam;
        out.i = idx;
        out.f[0] = v;
        return nullptr;
    }

    if (is(c, "load")) {
        if (argc < 2) return "usage: load step|pulse|grad|noise|off ...";
        const char *s = argv[1];
        const char *err;
        if (is(s, "step")) {
            if ((err = parse_floats(argc, argv, 2, 1, 3, out))) return err;
            if (out.f[0] < 0.0f || out.f[1] < 0.0f) return "torque and ramp must be >= 0";
            out.type = CmdType::LoadStep;
        } else if (is(s, "pulse")) {
            if ((err = parse_floats(argc, argv, 2, 3, 3, out))) return err;
            if (out.f[0] < 0.0f || out.f[1] < 10.0f || out.f[2] < 0.0f || out.f[2] > 100.0f)
                return "need Nm >= 0, period >= 10 ms, duty 0-100";
            out.type = CmdType::LoadPulse;
        } else if (is(s, "grad")) {
            if ((err = parse_floats(argc, argv, 2, 1, 1, out))) return err;
            out.type = CmdType::LoadGrad;
        } else if (is(s, "noise")) {
            out.f[1] = 2.0f;
            if ((err = parse_floats(argc, argv, 2, 1, 2, out))) return err;
            if (out.f[0] < 0.0f || out.f[1] <= 0.0f) return "need sd >= 0 and corner > 0";
            out.type = CmdType::LoadNoise;
        } else if (is(s, "off")) {
            if (argc != 2) return "too many values";
            out.type = CmdType::LoadOff;
        } else {
            return "usage: load step|pulse|grad|noise|off ...";
        }
        return nullptr;
    }

    if (is(c, "profile")) {
        if (argc < 2) return "usage: profile load|start|stop|loop|exp ...";
        const char *s = argv[1];
        if (is(s, "start") && argc == 2) { out.type = CmdType::ProfileStart; return nullptr; }
        if (is(s, "stop") && argc == 2) { out.type = CmdType::ProfileStop; return nullptr; }
        if (is(s, "loop") && argc == 3) {
            out.type = CmdType::ProfileLoop;
            return parse_bool(argv[2], out.i);
        }
        if (is(s, "exp") && argc == 4) {
            if (!parse_float(argv[2], out.f[0]) || !parse_float(argv[3], out.f[1]))
                return "bad number";
            if (out.f[1] <= 0.0f) return "w_ref must be > 0 rpm";
            out.type = CmdType::ProfileExp;
            return nullptr;
        }
        return "usage: profile start|stop|loop <0|1>|exp <n> <w_ref_rpm>";
    }

    if (is(c, "mark")) {
        if (argc != 2 || !parse_int(argv[1], out.i) || out.i < 0 || out.i > 65535)
            return "usage: mark <0-65535>";
        out.type = CmdType::Mark;
        return nullptr;
    }

    if (is(c, "fault")) {
        if (argc != 3 || !is(argv[1], "inv")) return "usage: fault inv <0|1>";
        out.type = CmdType::FaultInv;
        return parse_bool(argv[2], out.i);
    }

    return "unknown command, try help";
}

} // namespace simproto
