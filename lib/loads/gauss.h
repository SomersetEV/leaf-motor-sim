// Small deterministic Gaussian noise source (xorshift32 + Box-Muller).
// Plain C++, no heap; the same seed gives the same sequence on PC and ESP32.
#pragma once

#include <math.h>
#include <stdint.h>

namespace loads {

class Gauss {
public:
    explicit Gauss(uint32_t seed = 0x2545F491u) : s_(seed ? seed : 1u) {}

    // Standard normal sample, mean 0, standard deviation 1.
    float next() {
        if (have_spare_) {
            have_spare_ = false;
            return spare_;
        }
        float u1 = uniform();
        float u2 = uniform();
        float r  = sqrtf(-2.0f * logf(u1));
        float a  = 6.28318530718f * u2;
        spare_ = r * sinf(a);
        have_spare_ = true;
        return r * cosf(a);
    }

private:
    // Uniform in (0, 1], never zero so logf is safe.
    float uniform() {
        s_ ^= s_ << 13;
        s_ ^= s_ >> 17;
        s_ ^= s_ << 5;
        return ((s_ >> 8) + 1u) * (1.0f / 16777216.0f);
    }

    uint32_t s_;
    float    spare_ = 0.0f;
    bool     have_spare_ = false;
};

} // namespace loads
