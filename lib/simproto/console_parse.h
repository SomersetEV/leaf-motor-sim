// Serial console line assembly and tokenising. No heap allocation.
// See docs/SIMULATOR_PLAN.md section 7, "Serial console".
#pragma once

#include <stddef.h>

namespace simproto {

// Collects characters into lines. CR, LF or CRLF end a line. A line longer
// than the buffer is discarded whole and reported as overflowed.
class LineReader {
public:
    static constexpr size_t MAX_LINE = 127;

    // Returns true when a complete line is ready in line().
    bool feed(char c);
    const char *line() const { return buf_; }
    bool overflowed() const { return overflowed_; }

private:
    char   buf_[MAX_LINE + 1] = {};
    size_t len_ = 0;
    bool   discarding_ = false;
    bool   overflowed_ = false;
};

// Splits line in place on spaces and tabs. Returns the number of tokens,
// at most max_tokens; extra tokens are dropped.
int tokenize(char *line, char *argv[], int max_tokens);

} // namespace simproto
