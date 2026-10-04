// ps_audio/spsc.h - lock-free single-producer / single-consumer containers
// (ps_audio A7, out/audio_plan.md section 4 "Thread model"): a fixed-capacity
// queue of trivially copyable items (the EngineInput queue) and a float ring
// (the Runner's output). No locks, no allocation after construction; one
// producer thread and one consumer thread each, acquire/release ordering on
// the two indices only.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace ps_audio {

// Capacity N must be a power of two; holds at most N items.
template <class T, std::size_t N>
class SpscQueue {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "SpscQueue capacity must be a power of two");
    static_assert(std::is_trivially_copyable_v<T>, "SpscQueue items must be trivially copyable");

public:
    // Producer. false when full (the item is dropped; the caller decides).
    bool push(const T& v) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        const std::size_t h = head_.load(std::memory_order_acquire);
        if (t - h >= N) return false;
        buf_[t & (N - 1)] = v;
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }
    // Consumer. false when empty.
    bool pop(T& out) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        if (h == t) return false;
        out = buf_[h & (N - 1)];
        head_.store(h + 1, std::memory_order_release);
        return true;
    }
    [[nodiscard]] std::size_t size_approx() const {
        return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
    }
    static constexpr std::size_t capacity() { return N; }

private:
    T buf_[N]{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};

// Float ring of `capacity` samples (rounded up to a power of two), sized once.
class SpscFloatRing {
public:
    explicit SpscFloatRing(std::size_t capacity) {
        std::size_t c = 2;
        while (c < capacity) c <<= 1;
        buf_.assign(c, 0.0f);
        mask_ = c - 1;
    }
    [[nodiscard]] std::size_t capacity() const { return mask_ + 1; }
    // Producer: writes all n or nothing; returns false when there is no room.
    bool write(const float* src, std::size_t n) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        const std::size_t h = head_.load(std::memory_order_acquire);
        if (capacity() - (t - h) < n) return false;
        for (std::size_t i = 0; i < n; ++i) buf_[(t + i) & mask_] = src[i];
        tail_.store(t + n, std::memory_order_release);
        return true;
    }
    // Consumer: reads up to n samples, returns how many.
    std::size_t read(float* dst, std::size_t n) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        const std::size_t avail = t - h;
        const std::size_t k = avail < n ? avail : n;
        for (std::size_t i = 0; i < k; ++i) dst[i] = buf_[(h + i) & mask_];
        head_.store(h + k, std::memory_order_release);
        return k;
    }
    [[nodiscard]] std::size_t readable() const {
        return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
    }
    // Bounded diagnostic snapshot for a third observer. Reading head first
    // avoids unsigned underflow if both producer and consumer advance.
    [[nodiscard]] std::size_t readable_snapshot() const {
        const auto head=head_.load(std::memory_order_acquire);
        const auto count=tail_.load(std::memory_order_acquire)-head;
        return count<capacity() ? count : capacity();
    }
    [[nodiscard]] std::size_t writable() const { return capacity() - readable(); }

private:
    std::vector<float> buf_;
    std::size_t mask_ = 0;
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};

} // namespace ps_audio
