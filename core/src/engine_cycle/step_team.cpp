// engine_cycle/step_team.cpp - see ps/engine_cycle/step_team.h.

#include "ps/engine_cycle/step_team.h"

#include <cstddef>

namespace ps::engine_cycle {

namespace {

inline void cpu_relax() {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    _mm_pause();
#elif defined(__aarch64__) || defined(_M_ARM64)
    // ARM64 spin-wait hint (the counterpart of x86 PAUSE). `yield` is a NOP on
    // cores without SMT but is the architecturally defined hint; GCC and clang
    // take the asm form, MSVC its intrinsic.
#if defined(_MSC_VER) && defined(_M_ARM64)
    __yield();
#else
    __asm__ __volatile__("yield" ::: "memory");
#endif
#else
    std::this_thread::yield();
#endif
}

// Spin this long before blocking on the generation counter. A pause is a few
// tens of ns on x86, so ~4096 keeps a step that follows closely (the common
// live case) in cache without burning a core for long when the voice is idle.
constexpr int kSpinLimit = 4096;

}  // namespace

StepTeam::StepTeam(int threads) : worker_count_(threads > 1 ? threads - 1 : 0) {
    threads_.reserve(static_cast<std::size_t>(worker_count_));
    for (int i = 0; i < worker_count_; ++i) {
        threads_.emplace_back([this, i] { worker_loop(i); });
    }
    // Do not return until every worker has read its starting generation. A
    // worker that reads gen_ only after the first run() has bumped it would
    // then wait on that same value and miss the notify (run() has already
    // returned) - the first step would hang.
    while (started_.load(std::memory_order_acquire) < worker_count_) {
        std::this_thread::yield();
    }
}

StepTeam::~StepTeam() {
    shutdown_.store(true, std::memory_order_release);
    gen_.fetch_add(1, std::memory_order_acq_rel);  // release any blocked waiter
    gen_.notify_all();
    for (std::thread& t : threads_) t.join();
}

void StepTeam::worker_loop(int id) {
    std::uint64_t seen = gen_.load(std::memory_order_acquire);
    started_.fetch_add(1, std::memory_order_release);
    for (;;) {
        // Observe the new generation as a value and only adopt it after it is
        // seen to differ - adopting it first and then loading would let a run()
        // that bumps and notifies between those two steps be missed (the worker
        // would wait on the generation it just read, with no further notify).
        std::uint64_t g = seen;
        int spins = 0;
        for (;;) {
            if (shutdown_.load(std::memory_order_acquire)) return;
            g = gen_.load(std::memory_order_acquire);
            if (g != seen) break;
            if (spins++ < kSpinLimit) {
                cpu_relax();
            } else {
                gen_.wait(seen);  // blocks until the generation advances
                if (shutdown_.load(std::memory_order_acquire)) return;
            }
        }
        seen = g;
        // The destructor bumps gen_ only to wake blocked workers; a spinning
        // worker that saw that bump must not treat it as a job (job_ may
        // already be null).
        if (shutdown_.load(std::memory_order_acquire)) return;
        if (job_ == nullptr) return;
        // job_ was published (a plain store) before the releasing gen_ bump
        // that produced g, so it is visible here.
        job_->run(id + 1, worker_count_ + 1);
        done_.fetch_add(1, std::memory_order_acq_rel);
    }
}

void StepTeam::run(StepJob& job) {
    if (worker_count_ == 0) {
        job.run(0, 1);
        return;
    }
    job_ = &job;
    done_.store(0, std::memory_order_relaxed);
    const std::uint64_t next = gen_.load(std::memory_order_relaxed) + 1;
    gen_.store(next, std::memory_order_release);
    gen_.notify_all();
    job.run(0, worker_count_ + 1);  // the caller's own share (worker 0)
    int spins = 0;
    while (done_.load(std::memory_order_acquire) < worker_count_) {
        if (spins++ < kSpinLimit) {
            cpu_relax();
        } else {
            std::this_thread::yield();
        }
    }
    job_ = nullptr;
}

}  // namespace ps::engine_cycle
