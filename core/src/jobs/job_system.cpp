#include "ps/jobs/job_system.h"
#include "ps/jobs/debug_alloc.h"

#include <cassert>
#include <chrono>

namespace ps::jobs {

namespace {
thread_local int t_worker_index = -1; // -1: not a worker thread
}

JobSystem::JobSystem(int num_workers) {
    if (num_workers <= 0) {
        const int physical = physical_core_count();
        num_workers = std::max(1, physical - 2);
    }
    deques_ = std::vector<WorkerQueue>(static_cast<std::size_t>(num_workers));
    stats_ = std::vector<WorkerStats>(static_cast<std::size_t>(num_workers));
    workers_.reserve(static_cast<std::size_t>(num_workers));
    for (int i = 0; i < num_workers; ++i) {
        workers_.emplace_back([this, i] { worker_loop(i); });
    }
}

JobSystem::~JobSystem() {
    shutdown_.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(wake_m_);
    }
    wake_cv_.notify_all();
    for (auto& t : workers_) {
        if (t.joinable()) t.join();
    }
}

bool JobSystem::on_worker_thread() const { return t_worker_index >= 0; }

void JobSystem::submit(JobFn fn, WaitGroup* group) {
    if (group) group->remaining.fetch_add(1, std::memory_order_acq_rel);
    JobEntry entry{std::move(fn), group};

    if (t_worker_index >= 0) {
        auto& own = deques_[static_cast<std::size_t>(t_worker_index)];
        {
            std::lock_guard<std::mutex> lock(own.m);
            own.q.push_back(std::move(entry));
        }
    } else {
        std::lock_guard<std::mutex> lock(injector_m_);
        injector_.push_back(std::move(entry));
    }
    pending_.fetch_add(1, std::memory_order_acq_rel);
    wake_cv_.notify_all();
}

bool JobSystem::try_pop_own(int index, JobEntry& out) {
    auto& own = deques_[static_cast<std::size_t>(index)];
    std::lock_guard<std::mutex> lock(own.m);
    if (own.q.empty()) return false;
    out = std::move(own.q.back());
    own.q.pop_back();
    return true;
}

bool JobSystem::try_steal(int index, JobEntry& out) {
    const int n = static_cast<int>(deques_.size());
    for (int off = 1; off < n; ++off) {
        const int victim = (index + off) % n;
        auto& q = deques_[static_cast<std::size_t>(victim)];
        std::lock_guard<std::mutex> lock(q.m);
        if (!q.q.empty()) {
            out = std::move(q.q.front());
            q.q.pop_front();
            return true;
        }
    }
    return false;
}

bool JobSystem::try_pop_injector(JobEntry& out) {
    std::lock_guard<std::mutex> lock(injector_m_);
    if (injector_.empty()) return false;
    out = std::move(injector_.front());
    injector_.pop_front();
    return true;
}

void JobSystem::run_job(JobEntry& job) {
    const auto t0 = std::chrono::steady_clock::now();
    debug_alloc::enter_job();
    job.fn();
    debug_alloc::exit_job();
    const auto t1 = std::chrono::steady_clock::now();
    if (t_worker_index >= 0) {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        stats_[static_cast<std::size_t>(t_worker_index)].busy_ns.fetch_add(static_cast<std::uint64_t>(ns), std::memory_order_relaxed);
    }
    pending_.fetch_sub(1, std::memory_order_acq_rel);
    if (job.group) {
        job.group->remaining.fetch_sub(1, std::memory_order_acq_rel);
    }
}

void JobSystem::worker_loop(int index) {
    t_worker_index = index;
    JobEntry job;
    while (!shutdown_.load(std::memory_order_acquire)) {
        if (try_pop_own(index, job) || try_steal(index, job) || try_pop_injector(job)) {
            run_job(job);
            continue;
        }
        std::unique_lock<std::mutex> lock(wake_m_);
        wake_cv_.wait_for(lock, std::chrono::milliseconds(1), [this] {
            return shutdown_.load(std::memory_order_acquire) || pending_.load(std::memory_order_acquire) > 0;
        });
    }
}

// wait() previously parked non-worker callers on a per-group
// condition_variable, notified from run_job() once remaining.fetch_sub()
// returned 1. That is a textbook-correct pattern on paper (the notifier
// takes the same mutex the waiter's predicate check + block transition is
// serialized against), but it went into an intermittent, release-build-only
// hang under this project's actual usage: World::step() drives ~9 stages
// per tick through exactly this path, on the thread calling World::step()
// (never one of JobSystem's own workers), and one stage
// (backend_step) runs JoltPhysics' own PhysicsSystem::Update(), which
// itself creates and queues a variable, data-dependent number of further
// jobs through the very same pool while that stage's single outer job is
// still executing. Across the ~1700-2700 ticks a validation test runs,
// that combination reproduced a hang in roughly half of Release-build
// runs (never observed in Debug, where different timing apparently never
// hit the same window) - see PHYS/TOOL vault tickets for the fuller
// writeup. Rather than keep chasing the exact interleaving in a
// notify/wakeup design, wait() now has every caller - worker or not -
// actively drain the pool's queues while polling the group's atomic
// counter directly. Progress then depends only on that atomic (whose
// semantics are unambiguous) instead of a notify happening to reach an
// already-registered waiter, and it has the side benefit that a calling
// thread which is not one of the pool's own workers (e.g. the sim thread
// running World::step()) can still help execute nested work - including
// Jolt's own bridged jobs - instead of sitting idle.
void JobSystem::wait(WaitGroup& group) {
    const int self = t_worker_index;

    if (self < 0) {
        // Caller is not one of this pool's own workers (the common case:
        // World::step() runs on the sim thread, never a worker). It has no
        // deadlock risk to avoid by helping (only a worker blocking inside
        // a nested wait() could starve the pool), so it just polls the
        // group's atomic counter directly rather than also contending on
        // the same deque mutexes the real workers use (measured directly:
        // having this thread also call try_pop_injector()/steal in the
        // poll loop measurably hurt the worker-count scaling benchmark via
        // lock contention with nothing useful to steal here). Plain
        // yield() only, deliberately no sleep_for() fallback: on Windows,
        // sleep_for() durations shorter than the OS timer resolution
        // (default ~15.6 ms) round UP to that full resolution unless a
        // process opts into a finer timer - confirmed by reproducing it
        // (an earlier version of this loop escalated to
        // sleep_for(20us) after ~1000 spins, which collapsed the
        // benchmark to a flat ~15.5 ms regardless of worker count, i.e.
        // one full Windows timer tick per wait() call). A tight tick
        // budget (PLAN.md D3: 240 Hz) cannot afford that, so this stays a
        // pure spin - acceptable because these waits are expected to
        // resolve in well under a millisecond.
        while (group.remaining.load(std::memory_order_acquire) > 0) {
            std::this_thread::yield();
        }
        return;
    }

    // Caller is a worker (nested submission, e.g. World's force-element
    // stage calling parallel_for from within a job already running on this
    // pool): help drain the pool instead of idly polling, so nested work
    // cannot starve waiting for a free worker.
    JobEntry job;
    while (group.remaining.load(std::memory_order_acquire) > 0) {
        if (try_pop_own(self, job) || try_steal(self, job) || try_pop_injector(job)) {
            run_job(job);
        } else {
            std::this_thread::yield();
        }
    }
}

void JobSystem::drain_idle() {
    while (pending_.load(std::memory_order_acquire) > 0) {
        std::this_thread::yield();
    }
}

std::uint64_t JobSystem::worker_busy_ns(int worker_index) const {
    return stats_[static_cast<std::size_t>(worker_index)].busy_ns.load(std::memory_order_relaxed);
}

void JobSystem::reset_worker_stats() {
    for (auto& s : stats_) s.busy_ns.store(0, std::memory_order_relaxed);
}

} // namespace ps::jobs
