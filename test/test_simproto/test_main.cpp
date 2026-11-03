// Tests for lib/simproto console line handling.

#include <string.h>
#include <unity.h>

#include <math.h>

#include "commands.h"
#include "console_parse.h"
#include "param_table.h"
#include "telemetry.h"
#include "zombie_sdo.h"

using namespace simproto;

void setUp() {}
void tearDown() {}

// Feeds s and returns how many complete lines it produced; the last is in r.
static int feed_all(LineReader &r, const char *s) {
    int lines = 0;
    for (; *s; s++)
        if (r.feed(*s)) lines++;
    return lines;
}

static void test_line_endings() {
    LineReader r;
    TEST_ASSERT_EQUAL_INT(1, feed_all(r, "stat\r\n"));
    TEST_ASSERT_EQUAL_STRING("stat", r.line());
    TEST_ASSERT_EQUAL_INT(1, feed_all(r, "help\n"));
    TEST_ASSERT_EQUAL_STRING("help", r.line());
    TEST_ASSERT_EQUAL_INT(1, feed_all(r, "a b\r"));
    TEST_ASSERT_EQUAL_STRING("a b", r.line());
    TEST_ASSERT_EQUAL_INT(0, feed_all(r, "\r\n\n"));  // blank lines ignored
}

static void test_overlong_line_discarded() {
    LineReader r;
    char longline[LineReader::MAX_LINE + 10];
    memset(longline, 'x', sizeof(longline) - 1);
    longline[sizeof(longline) - 1] = '\0';
    feed_all(r, longline);
    TEST_ASSERT_TRUE(r.feed('\n'));
    TEST_ASSERT_TRUE(r.overflowed());
    TEST_ASSERT_EQUAL_STRING("", r.line());
    // The next line is read normally.
    TEST_ASSERT_EQUAL_INT(1, feed_all(r, "stat\n"));
    TEST_ASSERT_FALSE(r.overflowed());
    TEST_ASSERT_EQUAL_STRING("stat", r.line());
}

static void test_max_length_line_kept() {
    LineReader r;
    char line[LineReader::MAX_LINE + 1];
    memset(line, 'y', LineReader::MAX_LINE);
    line[LineReader::MAX_LINE] = '\0';
    feed_all(r, line);
    TEST_ASSERT_TRUE(r.feed('\n'));
    TEST_ASSERT_FALSE(r.overflowed());
    TEST_ASSERT_EQUAL_STRING(line, r.line());
}

static void test_tokenize() {
    char line[] = "  load  step\t120 500 ";
    char *argv[8];
    int n = tokenize(line, argv, 8);
    TEST_ASSERT_EQUAL_INT(4, n);
    TEST_ASSERT_EQUAL_STRING("load", argv[0]);
    TEST_ASSERT_EQUAL_STRING("step", argv[1]);
    TEST_ASSERT_EQUAL_STRING("120", argv[2]);
    TEST_ASSERT_EQUAL_STRING("500", argv[3]);
}

static void test_tokenize_limits() {
    char empty[] = "   ";
    char *argv[2];
    TEST_ASSERT_EQUAL_INT(0, tokenize(empty, argv, 2));

    char many[] = "a bb ccc";
    TEST_ASSERT_EQUAL_INT(2, tokenize(many, argv, 2));
    TEST_ASSERT_EQUAL_STRING("a", argv[0]);
    TEST_ASSERT_EQUAL_STRING("bb", argv[1]);
}

// ---- Commands --------------------------------------------------------------

// Tokenises a command line and parses it.
static const char *parse(const char *text, SimCmd &out) {
    static char line[128];
    strncpy(line, text, sizeof(line) - 1);
    char *argv[8];
    int argc = tokenize(line, argv, 8);
    return parse_sim_command(argc, argv, out);
}

static void test_cmd_set() {
    SimCmd c;
    TEST_ASSERT_NULL(parse("set J 0.45", c));
    TEST_ASSERT_TRUE(c.type == CmdType::SetParam);
    TEST_ASSERT_EQUAL_INT(simcore::param_index("J"), c.i);
    TEST_ASSERT_EQUAL_FLOAT(0.45f, c.f[0]);
    TEST_ASSERT_NOT_NULL(parse("set J 0", c));      // out of range
    TEST_ASSERT_NOT_NULL(parse("set Jx 1", c));     // unknown
    TEST_ASSERT_NOT_NULL(parse("set J 1abc", c));   // not a number
    TEST_ASSERT_NOT_NULL(parse("set J", c));
}

static void test_cmd_load() {
    SimCmd c;
    TEST_ASSERT_NULL(parse("load step 80", c));
    TEST_ASSERT_TRUE(c.type == CmdType::LoadStep);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, c.f[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.f[1]);
    TEST_ASSERT_NULL(parse("load step 60 500 0.3", c));
    TEST_ASSERT_EQUAL_FLOAT(0.3f, c.f[2]);
    TEST_ASSERT_NOT_NULL(parse("load step -5", c));
    TEST_ASSERT_NOT_NULL(parse("load step 1 2 3 4", c));
    TEST_ASSERT_NULL(parse("load pulse 150 700 40", c));
    TEST_ASSERT_TRUE(c.type == CmdType::LoadPulse);
    TEST_ASSERT_NOT_NULL(parse("load pulse 150 700", c));
    TEST_ASSERT_NOT_NULL(parse("load pulse 150 700 120", c));
    TEST_ASSERT_NULL(parse("load grad -20", c));
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, c.f[0]);
    TEST_ASSERT_NULL(parse("load noise 3", c));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, c.f[1]);  // default corner
    TEST_ASSERT_NULL(parse("load off", c));
    TEST_ASSERT_TRUE(c.type == CmdType::LoadOff);
    TEST_ASSERT_NOT_NULL(parse("load spin 3", c));
}

static void test_cmd_profile_mark_fault() {
    SimCmd c;
    TEST_ASSERT_NULL(parse("profile start", c));
    TEST_ASSERT_TRUE(c.type == CmdType::ProfileStart);
    TEST_ASSERT_NULL(parse("profile loop 1", c));
    TEST_ASSERT_EQUAL_INT(1, c.i);
    TEST_ASSERT_NOT_NULL(parse("profile loop 2", c));
    TEST_ASSERT_NULL(parse("profile exp 1.5 540", c));
    TEST_ASSERT_EQUAL_FLOAT(540.0f, c.f[1]);
    TEST_ASSERT_NOT_NULL(parse("profile exp 1 0", c));
    TEST_ASSERT_NULL(parse("mark 42", c));
    TEST_ASSERT_TRUE(c.type == CmdType::Mark);
    TEST_ASSERT_EQUAL_INT(42, c.i);
    TEST_ASSERT_NOT_NULL(parse("mark -1", c));
    TEST_ASSERT_NULL(parse("fault inv 1", c));
    TEST_ASSERT_TRUE(c.type == CmdType::FaultInv);
    TEST_ASSERT_NOT_NULL(parse("fault motor 1", c));
    TEST_ASSERT_NOT_NULL(parse("frobnicate", c));
}

// ---- Telemetry -------------------------------------------------------------

static int16_t s16(const uint8_t *d) { return (int16_t)(d[0] | (d[1] << 8)); }

static void test_4b0_scaling_and_saturation() {
    uint8_t d[8];
    pack_4b0(-1234.5f, 101.23f, -50.0f, 7.04f, d);
    TEST_ASSERT_EQUAL_INT16(-2469, s16(d));
    TEST_ASSERT_EQUAL_INT16(1012, s16(d + 2));
    TEST_ASSERT_EQUAL_INT16(-500, s16(d + 4));
    TEST_ASSERT_EQUAL_INT16(70, s16(d + 6));
    pack_4b0(20000.0f, 1e6f, -1e6f, NAN, d);
    TEST_ASSERT_EQUAL_INT16(32767, s16(d));
    TEST_ASSERT_EQUAL_INT16(32767, s16(d + 2));
    TEST_ASSERT_EQUAL_INT16(-32768, s16(d + 4));
    TEST_ASSERT_EQUAL_INT16(0, s16(d + 6));
}

static void test_4b1_and_4b2() {
    uint8_t d[8];
    SlowTelemetry s{0.9f, 3, 2, FLAG_RX_TIMEOUT | FLAG_FAULT, 258, 3, 1234};
    pack_4b1(s, d);
    TEST_ASSERT_EQUAL_UINT16(900, (uint16_t)s16(d));
    TEST_ASSERT_EQUAL_UINT8(3, d[2]);
    TEST_ASSERT_EQUAL_UINT8(2, d[3]);
    TEST_ASSERT_EQUAL_HEX8(0x09, d[4]);
    TEST_ASSERT_EQUAL_UINT8(2, d[5]);    // 258 wraps
    TEST_ASSERT_EQUAL_UINT8(3, d[6]);
    TEST_ASSERT_EQUAL_UINT8(12, d[7]);   // 1.2 ms in 0.1 ms
    s.worst_jitter_us = 100000;
    pack_4b1(s, d);
    TEST_ASSERT_EQUAL_UINT8(255, d[7]);  // saturates

    pack_4b2(EV_LOAD, 0xBEEF, 0x01020304, d);
    TEST_ASSERT_EQUAL_UINT8(EV_LOAD, d[0]);
    TEST_ASSERT_EQUAL_HEX8(0xEF, d[1]);
    TEST_ASSERT_EQUAL_HEX8(0xBE, d[2]);
    TEST_ASSERT_EQUAL_HEX8(0x04, d[3]);
    TEST_ASSERT_EQUAL_HEX8(0x01, d[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, d[7]);
}

static void test_csv_line() {
    StreamSample s{1230, -40, -10.0f, -9.5f, 12.25f, 1501.04f, 5};
    char buf[96];
    format_csv(s, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("1230,-40,-10.00,-9.50,12.25,1501.0,5", buf);
}

static void test_candump_line() {
    CanLogFrame f{12345678ull, 0x1D4, 8, {0xF7, 0x07, 0xD9, 0xA0, 0x47, 0x44, 0x30, 0x1F}};
    char buf[CANDUMP_LINE_MAX];
    format_candump(f, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("(12.345678) can0 1D4#F707D9A04744301F", buf);

    CanLogFrame g{5ull, 0x55A, 0, {}};
    format_candump(g, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("(0.000005) can0 55A#", buf);

    CanLogFrame h{4294967295000000ull, 0x7FF, 2, {0xAB, 0xCD}};  // longest timestamp
    format_candump(h, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("(4294967295.000000) can0 7FF#ABCD", buf);
}

static void test_zparam_whitelist() {
    const ZParam *kp = zparam_find("govKp");
    TEST_ASSERT_NOT_NULL(kp);
    TEST_ASSERT_EQUAL(157, kp->id);
    const ZParam *flt = zparam_find("govFlt");
    TEST_ASSERT_NOT_NULL(flt);
    TEST_ASSERT_EQUAL(162, flt->id);
    const ZParam *gl = zparam_find("GovLog");
    TEST_ASSERT_NOT_NULL(gl);
    TEST_ASSERT_EQUAL(163, gl->id);
    TEST_ASSERT_EQUAL(7, ZPARAM_COUNT);
    TEST_ASSERT_NULL(zparam_find("throtmax"));  // not writable from the simulator
    TEST_ASSERT_NULL(zparam_find("govkp"));     // names are case-sensitive
}

static void test_sdo_requests() {
    uint8_t b[8];
    sdo_write_request(157, 14.0f, b);  // govKp = 14 -> 448 = 0x1C0
    const uint8_t w[8] = {0x23, 0x00, 0x21, 0x9D, 0xC0, 0x01, 0x00, 0x00};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(w, b, 8);

    sdo_write_request(15, 2200.0f, b);  // revlim, id below 256
    const uint8_t r15[8] = {0x23, 0x00, 0x21, 0x0F, 0x00, 0x13, 0x01, 0x00};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(r15, b, 8);

    sdo_write_request(157, 0.03125f, b);  // smallest step, 1 count
    TEST_ASSERT_EQUAL_HEX8(0x01, b[4]);

    sdo_write_request(300, -1.0f, b);  // id above 255 goes in the index; negative value
    const uint8_t neg[8] = {0x23, 0x01, 0x21, 0x2C, 0xE0, 0xFF, 0xFF, 0xFF};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(neg, b, 8);

    sdo_read_request(158, b);
    const uint8_t rd[8] = {0x40, 0x00, 0x21, 0x9E, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rd, b, 8);
}

static void test_sdo_replies() {
    const uint8_t rd[8] = {0x43, 0x00, 0x21, 0x9D, 0x60, 0x01, 0x00, 0x00};  // 352 = 11.0
    SdoResult r = sdo_decode_reply(rd, 8);
    TEST_ASSERT_TRUE(r.kind == SdoReply::ReadOk);
    TEST_ASSERT_EQUAL(157, r.id);
    TEST_ASSERT_EQUAL_FLOAT(11.0f, r.value);

    const uint8_t wr[8] = {0x60, 0x00, 0x21, 0x9D, 0xC0, 0x01, 0x00, 0x00};
    r = sdo_decode_reply(wr, 8);
    TEST_ASSERT_TRUE(r.kind == SdoReply::WriteOk);
    TEST_ASSERT_EQUAL_FLOAT(14.0f, r.value);

    const uint8_t ab[8] = {0x80, 0x00, 0x21, 0xA2, 0x30, 0x00, 0x09, 0x06};
    r = sdo_decode_reply(ab, 8);
    TEST_ASSERT_TRUE(r.kind == SdoReply::Abort);
    TEST_ASSERT_EQUAL(162, r.id);
    TEST_ASSERT_EQUAL_HEX32(0x06090030u, r.abort_code);

    r = sdo_decode_reply(rd, 7);  // short
    TEST_ASSERT_TRUE(r.kind == SdoReply::Bad);
    const uint8_t other[8] = {0x43, 0x00, 0x50, 0x00, 0, 0, 0, 0};  // not a parameter index
    r = sdo_decode_reply(other, 8);
    TEST_ASSERT_TRUE(r.kind == SdoReply::Bad);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_cmd_set);
    RUN_TEST(test_cmd_load);
    RUN_TEST(test_cmd_profile_mark_fault);
    RUN_TEST(test_4b0_scaling_and_saturation);
    RUN_TEST(test_4b1_and_4b2);
    RUN_TEST(test_csv_line);
    RUN_TEST(test_candump_line);
    RUN_TEST(test_zparam_whitelist);
    RUN_TEST(test_sdo_requests);
    RUN_TEST(test_sdo_replies);
    RUN_TEST(test_line_endings);
    RUN_TEST(test_overlong_line_discarded);
    RUN_TEST(test_max_length_line_kept);
    RUN_TEST(test_tokenize);
    RUN_TEST(test_tokenize_limits);
    return UNITY_END();
}
