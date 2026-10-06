#include "json_flat.h"

#include <stdlib.h>

namespace simcore {

static const char *skip_ws(const char *p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

const char *json_flat_parse(const char *text, JsonMemberFn fn, void *ctx) {
    const char *p = skip_ws(text);
    if (*p != '{') return "expected {";
    p = skip_ws(p + 1);
    if (*p == '}') return *skip_ws(p + 1) ? "text after }" : nullptr;

    for (;;) {
        if (*p != '"') return "expected \"key\"";
        char key[32];
        int n = 0;
        p++;
        while (*p && *p != '"') {
            if (*p == '\\') return "escapes not supported";
            if (n >= (int)sizeof(key) - 1) return "key too long";
            key[n++] = *p++;
        }
        if (*p != '"') return "unterminated key";
        key[n] = '\0';
        p = skip_ws(p + 1);
        if (*p != ':') return "expected :";
        p = skip_ws(p + 1);
        char *end;
        double v = strtod(p, &end);
        if (end == p) return "values must be numbers";
        p = skip_ws(end);
        if (!fn(key, (float)v, ctx)) return "rejected member";
        if (*p == ',') {
            p = skip_ws(p + 1);
            continue;
        }
        if (*p == '}') return *skip_ws(p + 1) ? "text after }" : nullptr;
        return "expected , or }";
    }
}

} // namespace simcore
