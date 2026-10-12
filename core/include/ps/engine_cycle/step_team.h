// engine_cycle/step_team.h - ps_audio A9b: the small persistent worker team
// that runs one physics step (the PipeNetwork pipe loop, or the live engine's
// cylinder loop) across a fixed number of threads. Part of target
// ps_engine_cycle, never links ps_core.
//
// Why a hand-rolled team and not ps::jobs::JobSystem: ps_engine_cycle must
// never link ps_core (ps_engine_cycle_standalone guards this), and the live
// voice steps one small, fixed, contiguous workload every sample at 48 kHz -
// a global task-graph scheduler would add queueing and allocation per step for
// no gain. This team spawns `threads - 1` worker threads once, forks each
// step with a generation-counter barrier, allocates nothing per step, and
// blocks idle workers after a short spin (the live case calls it back within
// tens of microseconds; a bench or a paused voice should not burn a core).
//
// Determinism (A9b's hard requirement): the fork/join here only decides WHICH
// worker touches WHICH pipe / cylinder. The result must be bit-identical for
// every thread count >= 1, so the callers must keep every floating-point sum
// in a fixed order independent of the split (PipeNetwork folds its per-face
// exchange in pipe/sub-cycle order after the barrier; the cylinder loop is
// per-cylinder independent). This header has no shared floating-point state.
#pragma once

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#elif defined(_M_ARM64) && defined(_MSC_VER)
#include <intrin.h>  // __yield() (cpu_relax in step_team.cpp); aarch64 GCC/clang use inline asm instead
#endif

namespace ps::engine_cycle {

// One unit of parallel work. The caller and every worker call run() exactly
// once per step with a distinct `worker` id in [0, workers): id 0 is the
// calling thread, 1..workers-1 the spawned ones. An implementation must only
// touch memory owned by the ids it is given (pipe_groups_ / a cylinder's own
// stepper); shared accumulators are left to a serial fold after run() returns.
struct StepJob {
    virtual ~StepJob() = default;
    virtual void run(int worker, int workers) = 0;
};

class StepTeam {
public:
    // `threads` >= 1; 1 creates no worker thread (run() then just calls the
    // caller's share), so a serial engine can own a team of size 1.
    explicit StepTeam(int threads);
    ~StepTeam();
    StepTeam(const StepTeam&) = delete;
    StepTeam& operator=(const StepTeam&) = delete;

    // Total workers: the calling thread (id 0) plus the spawned ones, >= 1.
    [[nodiscard]] int size() const { return worker_count_ + 1; }

    // Run `job` on all size() workers and return when they are done. `job`
    // runs on the caller (id 0) too. Not re-entrant.
    void run(StepJob& job);

private:
    void worker_loop(int id);

    int worker_count_ = 0;
    std::vector<std::thread> threads_;
    std::atomic<std::uint64_t> gen_{0};   // work generation; bumped per run()
    std::atomic<int> done_{0};            // workers finished the current run()
    std::atomic<int> started_{0};         // workers that have entered worker_loop
    std::atomic<bool> shutdown_{false};
    StepJob* job_ = nullptr;              // published before gen_ is bumped
};

}  // namespace ps::engine_cycle
