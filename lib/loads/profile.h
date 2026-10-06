// Load profile storage and CSV parsing. See docs/SIMULATOR_PLAN.md section 6,
// "Presets and profiles".
//
// A profile is one row per 10 ms tick: torque in 0.1 Nm (int16) and an
// optional inertia column. Torque is stored in fixed-size chunks so a long
// profile does not need one large contiguous block of RAM. Inertia is stored
// as change points, since it only changes at events such as PTO engagement.
//
// Plain C++. Allocation happens only while loading, never in the tick.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace loads {

class Profile {
public:
    static constexpr uint32_t MAX_ROWS     = 60000;  // 10 minutes at 10 ms
    static constexpr uint32_t CHUNK_ROWS   = 4096;   // 8 KB per chunk
    static constexpr uint32_t MAX_CHUNKS   = (MAX_ROWS + CHUNK_ROWS - 1) / CHUNK_ROWS;
    static constexpr uint32_t MAX_J_CHANGES = 256;
    static constexpr uint32_t TICK_MS      = 10;

    Profile() = default;
    ~Profile() { clear(); }
    Profile(const Profile &) = delete;
    Profile &operator=(const Profile &) = delete;

    void clear();

    // Appends one row. Returns false when full or out of memory.
    bool append(float torque_nm);
    // Records the inertia from the current row onwards. Returns false when
    // the change-point table is full.
    bool set_inertia_from_next_row(float j_kgm2);

    uint32_t rows() const { return rows_; }
    float torque_nm(uint32_t row) const;
    bool  has_inertia() const { return n_j_ > 0; }
    // Inertia in force at row; only valid when has_inertia().
    float inertia_kgm2(uint32_t row) const;

private:
    int16_t *chunks_[MAX_CHUNKS] = {};
    uint32_t rows_ = 0;
    struct JChange { uint32_t row; float j; };
    JChange  j_changes_[MAX_J_CHANGES] = {};
    uint32_t n_j_ = 0;
};

// Parses a profile CSV one line at a time:
//   t_ms,torque_nm[,inertia_kgm2]
//   0,12.5,0.30
//   10,12.9,0.30
// Rows must be exactly 10 ms apart starting at 0, as make_profile.py writes
// them; the firmware indexes rows and never interpolates.
class ProfileCsvParser {
public:
    explicit ProfileCsvParser(Profile &out) : out_(out) { out_.clear(); }

    // Returns false on error; error() then describes it.
    bool line(const char *text);
    // Call after the last line. Returns false if no rows were read.
    bool finish();

    const char *error() const { return err_; }
    uint32_t line_number() const { return line_no_; }

private:
    bool fail(const char *msg) { err_ = msg; return false; }

    Profile &out_;
    bool header_done_ = false;
    bool has_j_ = false;
    float last_j_ = -1.0f;
    uint32_t line_no_ = 0;
    const char *err_ = "";
};

} // namespace loads
