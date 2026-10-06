#include "console_parse.h"

namespace simproto {

bool LineReader::feed(char c) {
    if (c == '\r' || c == '\n') {
        bool was_discarding = discarding_;
        discarding_ = false;
        if (was_discarding) {
            overflowed_ = true;
            len_ = 0;
            buf_[0] = '\0';
            return true;
        }
        if (len_ == 0) return false;  // blank line, or LF after CR
        buf_[len_] = '\0';
        len_ = 0;
        overflowed_ = false;
        return true;
    }
    if (discarding_) return false;
    if (len_ >= MAX_LINE) {
        discarding_ = true;
        return false;
    }
    buf_[len_++] = c;
    return false;
}

static bool is_space(char c) { return c == ' ' || c == '\t'; }

int tokenize(char *line, char *argv[], int max_tokens) {
    int n = 0;
    char *p = line;
    while (*p && n < max_tokens) {
        while (is_space(*p)) *p++ = '\0';
        if (!*p) break;
        argv[n++] = p;
        while (*p && !is_space(*p)) p++;
    }
    if (*p) *p = '\0';  // cut off tokens beyond max_tokens
    return n;
}

} // namespace simproto
