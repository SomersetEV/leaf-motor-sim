// Tests for lib/simproto console line handling.

#include <string.h>
#include <unity.h>

#include "console_parse.h"

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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_line_endings);
    RUN_TEST(test_overlong_line_discarded);
    RUN_TEST(test_max_length_line_kept);
    RUN_TEST(test_tokenize);
    RUN_TEST(test_tokenize_limits);
    return UNITY_END();
}
