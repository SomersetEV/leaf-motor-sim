// Synthetic CAN logs from the simulator's own model, for testing the PC tools
// before any tractor data exists (plan section 9, "Tool self-test").
//
// Builds on the PC from the firmware's plain C++ libraries; selftest.py
// compiles it with g++. Writes SavvyCAN CSV. A crude PI loop stands in for
// the Zombie where a scenario needs closed-loop control.
//
//   synth_log accel_coast <out.csv> [key=value ...]
//   synth_log baler <out.csv> <truth.csv> [key=value ...]
//   synth_log replay <profile.csv> <out.csv>
//
// Keys: J Tc b c tau Tmax Pmax kT (model), drop=<n> (drop n 0x4A1 frames),
// pulse_nm pulse_period_ms pulse_duty_pct (baler load), seconds.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "leafcodec.h"
#include "loads.h"
#include "plant.h"
#include "profile.h"
#include "telemetry.h"

struct Opts {
    plant::PlantParams p{0.30f, 5.0f, 0.02f, 0.0f, 30.0f, 280.0f, 80.0f};
    float kt = 0.25f;
    int drop = 0;
    float pulse_nm = 150.0f, pulse_period_ms = 700.0f, pulse_duty_pct = 40.0f;
    float seconds = 60.0f;
};

static bool set_opt(Opts &o, const char *kv) {
    const char *eq = strchr(kv, '=');
    if (!eq) return false;
    char key[32];
    size_t n = (size_t)(eq - kv);
    if (n >= sizeof(key)) return false;
    memcpy(key, kv, n);
    key[n] = '\0';
    float v = (float)atof(eq + 1);
    if (!strcmp(key, "J")) o.p.j_kgm2 = v;
    else if (!strcmp(key, "Tc")) o.p.tc_nm = v;
    else if (!strcmp(key, "b")) o.p.b_nms = v;
    else if (!strcmp(key, "c")) o.p.c_nms2 = v;
    else if (!strcmp(key, "tau")) o.p.tau_ms = v;
    else if (!strcmp(key, "Tmax")) o.p.tmax_nm = v;
    else if (!strcmp(key, "Pmax")) o.p.pmax_kw = v;
    else if (!strcmp(key, "kT")) o.kt = v;
    else if (!strcmp(key, "drop")) o.drop = (int)v;
    else if (!strcmp(key, "pulse_nm")) o.pulse_nm = v;
    else if (!strcmp(key, "pulse_period_ms")) o.pulse_period_ms = v;
    else if (!strcmp(key, "pulse_duty_pct")) o.pulse_duty_pct = v;
    else if (!strcmp(key, "seconds")) o.seconds = v;
    else return false;
    return true;
}

// SavvyCAN "GVRET native" CSV, timestamps in microseconds
static void frame(FILE *f, uint64_t t_us, uint32_t id, const uint8_t d[8]) {
    fprintf(f, "%llu,%08X,false,Rx,0,8", (unsigned long long)t_us, (unsigned)id);
    for (int i = 0; i < 8; i++) fprintf(f, ",%02X", d[i]);
    fprintf(f, "\n");
}

// 0x1D4 exactly as NissLeafMng::Task10Ms builds it in run mode (d85e756)
static void zombie_1d4(int16_t req, uint8_t ctr, uint8_t b[8]) {
    b[0] = 0xF7;
    b[1] = 0x07;
    if (req >= -2048 && req <= 2047) {
        b[2] = (uint8_t)(((req < 0) ? 0x80 : 0) | ((req >> 4) & 0x7F));
        b[3] = (uint8_t)((req << 4) & 0xF0);
    } else {
        b[2] = b[3] = 0;
    }
    b[4] = (uint8_t)(0x07 | (ctr << 6));
    b[5] = 0x44;
    b[6] = 0x30;
    b[7] = leafcodec::nissan_crc(b);
}

static void put16(uint8_t *d, int v) {
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    d[0] = (uint8_t)(v & 0xFF);
    d[1] = (uint8_t)((v >> 8) & 0xFF);
}

struct Sim {
    Opts o;
    plant::Plant model;
    loads::Loads ld;
    FILE *out;
    uint64_t tick = 0;
    uint8_t ctr = 0, gov_ctr = 0;
    int dropped = 0;

    Sim(const Opts &opts, FILE *f) : o(opts), model(opts.p), ld(1234), out(f) {}

    // One 10 ms tick with request raw; returns the resistive load added.
    float step(int16_t raw, float gov_int = 0, float gov_p = 0) {
        uint64_t t_us = tick * 10000ULL;
        uint8_t b[8];
        loads::LoadOutput lo = ld.tick(model.w_rad_s(), 0.01f);
        float t_req = raw * o.kt;
        for (int i = 0; i < 10; i++) model.substep(t_req, lo.tick, 0.001f);

        leafcodec::encode_1da(360.0f, model.rpm(), false, b);
        frame(out, t_us + 300, 0x1DA, b);
        simproto::pack_4b0(model.rpm(), t_req, model.t_motor_nm(), model.t_load_nm(), b);
        frame(out, t_us + 500, 0x4B0, b);
        zombie_1d4(raw, ctr, b);
        ctr = (ctr + 1) & 3;
        frame(out, t_us + 2000, 0x1D4, b);

        // Zombie governor log frames
        memset(b, 0, 8);
        put16(b, (int)lroundf(fabsf(model.rpm())));
        frame(out, t_us + 2200, 0x4A0, b);
        memset(b, 0, 8);
        put16(b, (int)lroundf(gov_int * 100));
        put16(b + 2, (int)lroundf(gov_p * 100));
        b[7] = gov_ctr++;
        bool drop_this = dropped < o.drop && tick > 100 && tick % 97 == 0;
        if (drop_this) dropped++;
        else frame(out, t_us + 2300, 0x4A1, b);
        tick++;
        return lo.tick.t_res_nm;
    }
    float rpm() const { return model.rpm(); }
};

static int accel_coast(const Opts &o, const char *path) {
    FILE *f = fopen(path, "w");
    if (!f) return 1;
    fprintf(f, "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n");
    Sim s(o, f);
    const int16_t levels[2] = {300, 500};
    const float targets[2] = {2500.0f, 3000.0f};
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < 50; i++) s.step(0);                // 0.5 s idle
        while (s.rpm() < targets[k]) s.step(levels[k]);        // R1: fixed request
        while (s.rpm() > 0.0f) s.step(0);                      // R2: coast-down
    }
    for (int i = 0; i < 50; i++) s.step(0);
    fclose(f);
    return 0;
}

static int baler(const Opts &o, const char *path, const char *truth_path) {
    FILE *f = fopen(path, "w");
    FILE *t = fopen(truth_path, "w");
    if (!f || !t) return 1;
    fprintf(f, "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n");
    fprintf(t, "t_ms,load_nm\n");
    Sim s(o, f);
    // Crude PI speed loop standing in for the Zombie governor, 2000 rpm
    float integ = 0;
    int n = (int)(o.seconds * 100);
    for (int i = 0; i < n; i++) {
        if (i == 300) s.ld.start_pulse(o.pulse_nm, o.pulse_period_ms, o.pulse_duty_pct);
        float err = 2000.0f - s.rpm();
        float p = err * 0.3f;
        if (fabsf(err) < 1000.0f) integ += err * 0.004f;
        integ = integ < -20 ? -20 : (integ > 60 ? 60 : integ);
        float pct = p + integ;
        pct = pct < -10 ? -10 : (pct > 100 ? 100 : pct);
        int16_t raw = (int16_t)(pct * 2047.0f / 100.0f);
        float load = s.step(raw, integ, p);
        fprintf(t, "%d,%.3f\n", i * 10, load);
    }
    fclose(f);
    fclose(t);
    return 0;
}

// Parses a profile CSV with the firmware's parser and replays it through
// the firmware's load code, writing the torque applied each tick.
static int replay(const char *profile_path, const char *out_path) {
    FILE *in = fopen(profile_path, "r");
    FILE *out = fopen(out_path, "w");
    if (!in || !out) return 1;
    static loads::Profile prof;
    loads::ProfileCsvParser csv(prof);
    char line[128];
    while (fgets(line, sizeof(line), in)) {
        if (!csv.line(line)) {
            fprintf(stderr, "line %u: %s\n", (unsigned)csv.line_number(), csv.error());
            return 2;
        }
    }
    if (!csv.finish()) {
        fprintf(stderr, "%s\n", csv.error());
        return 2;
    }
    loads::Loads ld;
    ld.set_profile(&prof);
    ld.profile_start();
    fprintf(out, "t_ms,torque_nm\n");
    for (uint32_t i = 0; i < prof.rows(); i++)
        fprintf(out, "%u,%.2f\n", (unsigned)(i * 10), ld.tick(0.0f, 0.01f).tick.t_res_nm);
    fclose(in);
    fclose(out);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: synth_log accel_coast|baler|replay ...\n");
        return 1;
    }
    Opts o;
    const char *mode = argv[1];
    int first_kv = !strcmp(mode, "baler") || !strcmp(mode, "replay") ? 4 : 3;
    for (int i = first_kv; i < argc; i++) {
        if (!set_opt(o, argv[i])) {
            fprintf(stderr, "bad option %s\n", argv[i]);
            return 1;
        }
    }
    if (!strcmp(mode, "accel_coast")) return accel_coast(o, argv[2]);
    if (!strcmp(mode, "baler") && argc >= 4) return baler(o, argv[2], argv[3]);
    if (!strcmp(mode, "replay") && argc >= 4) return replay(argv[2], argv[3]);
    fprintf(stderr, "unknown mode\n");
    return 1;
}
