// Tests for lib/simproto console line handling.

#include <string.h>
#include <unity.h>

#include <math.h>

#include "commands.h"
#include "console_parse.h"
#include "param_table.h"
#include "telemetry.h"

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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_cmd_set);
    RUN_TEST(test_cmd_load);
    RUN_TEST(test_cmd_profile_mark_fault);
    RUN_TEST(test_4b0_scaling_and_saturation);
    RUN_TEST(test_4b1_and_4b2);
    RUN_TEST(test_csv_line);
    RUN_TEST(test_line_endings);
    RUN_TEST(test_overlong_line_discarded);
    RUN_TEST(test_max_length_line_kept);
    RUN_TEST(test_tokenize);
    RUN_TEST(test_tokenize_limits);
    return UNITY_END();
}
