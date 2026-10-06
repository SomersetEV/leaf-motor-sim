#include "profile.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace loads {

void Profile::clear() {
    for (uint32_t i = 0; i < MAX_CHUNKS; i++) {
        free(chunks_[i]);
        chunks_[i] = nullptr;
    }
    rows_ = 0;
    n_j_  = 0;
}

bool Profile::append(float torque_nm) {
    if (rows_ >= MAX_ROWS) return false;
    uint32_t c = rows_ / CHUNK_ROWS;
    if (!chunks_[c]) {
        chunks_[c] = (int16_t *)malloc(CHUNK_ROWS * sizeof(int16_t));
        if (!chunks_[c]) return false;
    }
    float v = torque_nm * 10.0f;
    v = v > 32767.0f ? 32767.0f : (v < -32768.0f ? -32768.0f : v);
    chunks_[c][rows_ % CHUNK_ROWS] = (int16_t)lroundf(v);
    rows_++;
    return true;
}

bool Profile::set_inertia_from_next_row(float j_kgm2) {
    if (n_j_ > 0 && j_changes_[n_j_ - 1].row == rows_) {
        j_changes_[n_j_ - 1].j = j_kgm2;
        return true;
    }
    if (n_j_ >= MAX_J_CHANGES) return false;
    j_changes_[n_j_++] = {rows_, j_kgm2};
    return true;
}

float Profile::torque_nm(uint32_t row) const {
    if (row >= rows_) return 0.0f;
    return chunks_[row / CHUNK_ROWS][row % CHUNK_ROWS] * 0.1f;
}

float Profile::inertia_kgm2(uint32_t row) const {
    // Last change point at or before row (binary search).
    uint32_t lo = 0, hi = n_j_;
    while (hi - lo > 1) {
        uint32_t mid = (lo + hi) / 2;
        if (j_changes_[mid].row <= row) lo = mid;
        else hi = mid;
    }
    return n_j_ ? j_changes_[lo].j : 0.0f;
}

// Parses a number and advances p past it and one following comma.
static bool next_number(const char *&p, double &v) {
    while (*p == ' ' || *p == '\t') p++;
    char *end;
    v = strtod(p, &end);
    if (end == p) return false;
    p = end;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == ',') p++;
    return true;
}

static bool at_end(const char *p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return *p == '\0';
}

bool ProfileCsvParser::line(const char *text) {
    line_no_++;
    if (at_end(text)) return true;  // blank line

    if (!header_done_) {
        if (strncmp(text, "t_ms,torque_nm", 14) != 0)
            return fail("header must start t_ms,torque_nm");
        const char *rest = text + 14;
        if (strncmp(rest, ",inertia_kgm2", 13) == 0) {
            has_j_ = true;
            rest += 13;
        }
        if (!at_end(rest)) return fail("unknown header column");
        header_done_ = true;
        return true;
    }

    const char *p = text;
    double t_ms, torque, j = 0;
    if (!next_number(p, t_ms) || !next_number(p, torque)) return fail("bad number");
    if (has_j_ && !next_number(p, j)) return fail("missing inertia");
    if (!at_end(p)) return fail("too many columns");

    double expected = (double)out_.rows() * Profile::TICK_MS;
    if (fabs(t_ms - expected) > 0.001) return fail("rows must be 10 ms apart from 0");

    if (has_j_ && (float)j != last_j_) {
        if (j <= 0) return fail("inertia must be positive");
        if (!out_.set_inertia_from_next_row((float)j)) return fail("too many inertia changes");
        last_j_ = (float)j;
    }
    if (!out_.append((float)torque)) {
        return fail(out_.rows() >= Profile::MAX_ROWS ? "more than 60000 rows" : "out of memory");
    }
    return true;
}

bool ProfileCsvParser::finish() {
    if (!header_done_) return fail("empty file");
    if (out_.rows() == 0) return fail("no rows");
    return true;
}

} // namespace loads
