// ps/jobs/job_system.h — work-stealing job pool (PLAN.md D8).
//
// Owns every worker thread in the process; Jolt is handed a bridge
// (core/src/jobs/jolt_job_system.h, internal - not part of this public
// header) that forwards onto this same pool, so there is exactly one pool.
// Nothing else in the core may create threads (PLAN.md hard rule).
//
// Determinism contract this type exists to support (D7/D8): callers using
// parallel_for/parallel_batches must follow scatter -> compute -> ordered
// gather themselves (write only to a pre-allocated per-index slot inside
// the callback, then combine results in a later, sequential, fixed-order
// pass) - the job system guarantees every submitted chunk runs exactly
// once and that the call blocks until all of them finish, but it makes no
// guarantee about *which* worker runs *which* chunk or in what order, by
// design, so that result independence from scheduling is provable rather
// than incidental.

#pragma once

#include "ps/jobs/job_fn.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace ps::jobs {

// Physical core count, best-effort per-platform (core/src/jobs/topology.cpp).
// Falls back to hardware_concurrency()/2 (SMT heuristic) or 1.
int physical_core_count();

class JobSystem {
public:
    // A WaitGroup ties a batch of submitted jobs to a countdown a caller can
    // wait on (see wait() below for how). Stack-allocated per call in
    // parallel_for/parallel_batches; never shared across unrelated batches.
    struct WaitGroup {
        std::atomic<long> remaining{0};
    };

    // num_workers == 0 selects PLAN.md's default: physical cores - 2 (room
    // for the caller's own main/render threads), clamped to >= 1 so a
    // single-core / debugging configuration still works.
    explicit JobSystem(int num_workers = 0);
    ~JobSystem();
    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    [[nodiscard]] int worker_count() const { return static_cast<int>(deques_.size()); }

    // Submits one job. If group is non-null, group.remaining is incremented
    // (by this call, before the job becomes visible to any worker) and
    // decremented again once the job body returns. Race free as long as all
    // submits for a group happen before the first wait() call on it (the
    // pattern every helper below follows).
    void submit(JobFn fn, WaitGroup* group = nullptr);

    // Blocks the calling thread until group.remaining reaches 0. The
    // calling thread helps drain the pool's queues while waiting (whether
    // or not it is itself one of this pool's own worker threads), polling
    // the atomic remaining counter directly rather than parking on a
    // condition variable tied to the group - see job_system.cpp's comment
    // above wait() for why (a flaky hang, release-build only, traced to
    // this).
    void wait(WaitGroup& group);

    // Splits [0, count) into chunks of size min_grain (last chunk may be
    // smaller), runs fn(begin, end) once per chunk across the pool, blocks
    // until all chunks are done. fn must be safe to call concurrently from
    // multiple threads; per D8 it must only write to its own pre-allocated
    // output slots.
    template <typename Fn>
    void parallel_for(std::size_t count, std::size_t min_grain, Fn&& fn) {
        if (count == 0) return;
        if (min_grain == 0) min_grain = 1;
        const std::size_t num_chunks = (count + min_grain - 1) / min_grain;
        WaitGroup group;
        for (std::size_t c = 0; c < num_chunks; ++c) {
            const std::size_t begin = c * min_grain;
            const std::size_t end = std::min(count, begin + min_grain);
            submit(JobFn([&fn, begin, end]() { fn(begin, end); }), &group);
        }
        wait(group);
    }

    // Runs fn(batch_index) once per batch in [0, num_batches), same barrier
    // semantics as parallel_for.
    template <typename Fn>
    void parallel_batches(std::size_t num_batches, Fn&& fn) {
        parallel_for(num_batches, 1, [&fn](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) fn(i);
        });
    }

    // True if called from one of this pool's own worker threads.
    [[nodiscard]] bool on_worker_thread() const;

    // Blocks until every job this pool has ever been handed - including
    // ones with no WaitGroup, i.e. fire-and-forget submissions such as the
    // Jolt job-system bridge's (core/src/jobs/jolt_job_system.h) - has
    // actually finished executing, not merely been queued. Needed before
    // tearing down anything a still-in-flight fire-and-forget job's lambda
    // might reference (see JoltRigidBackend's destructor for why: a Jolt
    // Update() call can leave a job running slightly past the point its own
    // barrier reports "done", a genuine intermittent use-after-free found
    // via a WSL/Linux-only, native-speed-only, ~60%-reproducible SIGSEGV in
    // the determinism stress test, root-caused from a post-mortem core dump
    // - a worker thread's stack was inside
    // JoltJobSystemBridge::QueueJob(...)::lambda calling job->Execute() on
    // a Job already freed by that point, because JoltRigidBackend's bridge
    // (and its backing JPH::FixedSizeFreeList<Job>) had already been
    // destroyed).
    void drain_idle();

    // Per-worker busy time accumulated since the last reset_worker_stats()
    // call (profiling only - PLAN.md's per-stage timing / worker
    // utilisation telemetry channels; see World::stage_timing_channels_).
    [[nodiscard]] std::uint64_t worker_busy_ns(int worker_index) const;
    void reset_worker_stats();

private:
    struct JobEntry {
        JobFn fn;
        WaitGroup* group = nullptr;
    };

    // Fixed-capacity double-ended queue backing every worker's own deque and
    // the injector queue (PHYS-009 Step 3): a std::deque<JobEntry> here
    // allocated a new internal node every time push_back crossed a block
    // boundary that pop_front (a thief, or an injector-draining worker) had
    // just vacated - proven with a debug_alloc backtrace landing in
    // std::deque<JobEntry>::_M_push_back_aux, called from JobSystem::submit
    // while running on a worker thread (i.e. exactly the "parallel_for
    // called from inside a running job" nested-submission case). Because
    // push_back (owner) and pop_front (thieves/injector consumers) act on
    // opposite ends concurrently, this is not a one-time warm-up cost but a
    // permanent per-block churn - reproduced identically by a no-op nested
    // parallel_for with no articulation/World code involved at all. A
    // fixed-size ring buffer, sized once at JobSystem construction time (on
    // the constructing thread, never inside a job) and never resized again,
    // removes the allocation entirely rather than merely amortising it.
    // Capacity is generous relative to any known chunk count (parallel_for
    // submits roughly 4x worker_count chunks by default, or as many as
    // count/min_grain for an explicit small grain - bench_force_elements'
    // 2000/20 = 100 is the largest in this codebase) so overflow should
    // never happen from real workloads; if it ever does, push() fails
    // loudly (fail_queue_full()) rather than silently allocating.
    class RingQueue {
    public:
        explicit RingQueue(std::size_t capacity) : buf_(capacity) {}

        [[nodiscard]] bool push_back(JobEntry&& e) {
            if (size_ >= buf_.size()) return false;
            buf_[(head_ + size_) % buf_.size()] = std::move(e);
            ++size_;
            return true;
        }
        bool pop_back(JobEntry& out) {
            if (size_ == 0) return false;
            --size_;
            out = std::move(buf_[(head_ + size_) % buf_.size()]);
            return true;
        }
        bool pop_front(JobEntry& out) {
            if (size_ == 0) return false;
            out = std::move(buf_[head_]);
            head_ = (head_ + 1) % buf_.size();
            --size_;
            return true;
        }
        [[nodiscard]] bool empty() const { return size_ == 0; }

    private:
        std::vector<JobEntry> buf_; // sized once at construction, never resized
        std::size_t head_ = 0;
        std::size_t size_ = 0;
    };

    struct WorkerQueue {
        explicit WorkerQueue(std::size_t capacity) : q(capacity) {}
        std::mutex m;
        RingQueue q;
    };

    // Never exceeded by any workload in this codebase (see RingQueue's
    // comment); kept as one named constant so every queue - worker-owned or
    // the shared injector - agrees on the same headroom.
    static constexpr std::size_t kQueueCapacity = 4096;

    [[noreturn]] static void fail_queue_full(const char* which);

    void worker_loop(int index);
    bool try_pop_own(int index, JobEntry& out);
    bool try_steal(int index, JobEntry& out);
    bool try_pop_injector(JobEntry& out);
    void run_job(JobEntry& job);

    std::vector<std::thread> workers_;
    // unique_ptr indirection (not WorkerQueue directly): WorkerQueue holds a
    // std::mutex, which is neither copyable nor movable, so a plain
    // std::vector<WorkerQueue> cannot grow via reserve()/emplace_back() (the
    // container must remain instantiable even though, in practice, every
    // element is constructed once at JobSystem startup and the vector never
    // reallocates after that). This indirection is itself an allocation,
    // but - like the RingQueue buffers - only ever at JobSystem construction
    // time, never inside a job.
    std::vector<std::unique_ptr<WorkerQueue>> deques_;
    std::mutex injector_m_;
    RingQueue injector_{kQueueCapacity};
    std::mutex wake_m_;
    std::condition_variable wake_cv_;
    std::atomic<long> pending_{0};
    std::atomic<bool> shutdown_{false};

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324) // structure was padded due to alignment specifier - intentional (cache-line padding to avoid false sharing between workers)
#endif
    struct alignas(64) WorkerStats { std::atomic<std::uint64_t> busy_ns{0}; };
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    std::vector<WorkerStats> stats_;
};

} // namespace ps::jobs
