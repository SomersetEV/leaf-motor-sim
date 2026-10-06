// Lock-free single-producer, single-consumer ring buffer.
// The 10 ms tick pushes, the console task pops. push() never blocks: when
// the ring is full the sample is dropped and counted.
#pragma once

#include <atomic>
#include <stdint.h>

namespace simcore {

template <typename T, uint32_t N>
class SpscRing {
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

public:
    bool push(const T &v) {
        uint32_t h = head_.load(std::memory_order_relaxed);
        uint32_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= N) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    bool pop(T &v) {
        uint32_t t = tail_.load(std::memory_order_relaxed);
        uint32_t h = head_.load(std::memory_order_acquire);
        if (t == h) return false;
        v = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    // Consumer side only: discards everything queued.
    void clear() { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }

    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    T buf_[N];
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> dropped_{0};
};

} // namespace simcore
