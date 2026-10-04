// adapters/godot/src/sim_thread.h — steps a ps::World on its own thread at
// a fixed rate, independent of Godot's frame rate (PLAN.md D3: "The adapter
// runs the sim on its own thread and reads interpolated snapshots (lock-free
// triple buffer)"; P2 acceptance: "the sim keeps 240 Hz with Godot at any
// frame rate").
//
// Godot-free (only ps::World + std::thread) - this is a wall-clock-driven
// scheduler, which is exactly the adapter's job (PLAN.md's "no wall clock in
// the core" targets World::step()'s own logic, not the thing calling it at
// a fixed cadence from outside).

#pragma once

#include "ps/world/world.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

#if defined(_WIN32)
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

namespace ps_godot {

namespace detail {

// Sleeps the calling thread until `target`, more precisely than a plain
// sleep_until: coarse-sleeps down to within `spin_margin` of the deadline,
// then busy-spins the remainder. Needed on Windows: measured (see this
// adapter's PLAN.md P2 report) that std::this_thread::sleep_until alone,
// even with SimThread's own timeBeginPeriod(1) active, systematically
// overshoots a ~4.17 ms (240 Hz) deadline enough that the achieved rate
// settles at roughly half the target under sustained load - every
// individual overshoot permanently pushes the next tick later, since there
// is no headroom left in a 4.17 ms period to absorb multi-millisecond
// timer-granularity jitter. Spinning the last ~1-2 ms costs one CPU core
// but is the standard fixed-timestep technique for exactly this problem.
inline void precise_sleep_until(std::chrono::steady_clock::time_point target) {
    using namespace std::chrono;
    constexpr auto kSpinMargin = microseconds(1500);
    auto now = steady_clock::now();
    while (target - now > kSpinMargin) {
        std::this_thread::sleep_for(milliseconds(1));
        now = steady_clock::now();
    }
    while (steady_clock::now() < target) {
        // Busy-spin for the last ~1.5 ms.
    }
}

} // namespace detail

class SimThread {
public:
    explicit SimThread(ps::World& world) : world_(world) {}
    ~SimThread() { stop(); }
    SimThread(const SimThread&) = delete;
    SimThread& operator=(const SimThread&) = delete;

    // Called synchronously on the sim thread, right after world_.step()
    // returns and before the next tick starts - the one moment nothing else
    // touches the backend, so a callback here may safely read
    // World::link_frame/backend state that the public snapshot triple
    // buffer does not carry (PLAN.md D10/P1.5a articulation links - see
    // triple_buffer.h's file comment). Must not block for long: it runs on
    // the fixed-rate tick's own budget. Not thread-safe to change while
    // running() - set it before start().
    void set_post_step(std::function<void()> post_step) { post_step_ = std::move(post_step); }

    // Called synchronously on the sim thread right BEFORE world_.step()
    // runs each tick (PLAN.md M1: control-channel hand-off - see
    // ps_simulation.h's set_control doc comment for why this needs to be a
    // sim-thread-only write into ps::World's control channels rather than
    // the Godot main thread calling World::set_control directly). Not
    // thread-safe to change while running() - set it before start(), same
    // rule as set_post_step above.
    void set_pre_step(std::function<void()> pre_step) { pre_step_ = std::move(pre_step); }

    void start() {
        if (running_.exchange(true)) return; // already running
#if defined(_WIN32)
        // Raises the whole process's timer resolution from Windows' default
        // (~15.6 ms) to 1 ms - reduces, but on its own does not eliminate,
        // std::this_thread::sleep's overshoot (see detail::precise_sleep_until
        // above for why the hybrid spin-wait is still needed).
        timeBeginPeriod(1);
#endif
        thread_ = std::thread([this] { loop(); });
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (thread_.joinable()) thread_.join();
#if defined(_WIN32)
        timeEndPeriod(1);
#endif
    }

    [[nodiscard]] bool running() const { return running_.load(std::memory_order_relaxed); }

    // Total ticks this SimThread has run since start() - a plain relaxed
    // atomic counter, safe to read from the Godot main thread for the
    // "measure and report" tick-rate-stability check (PLAN.md P2
    // acceptance).
    [[nodiscard]] std::uint64_t ticks_run() const { return ticks_run_.load(std::memory_order_relaxed); }
    // Diagnostics (any thread): longest world step + post_step wall time since
    // the previous call (exchange-reset), and wake-ups that had to run more
    // than one tick back-to-back (cumulative).
    [[nodiscard]] std::uint64_t take_max_step_ns() { return max_step_ns_.exchange(0, std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t catch_up_wakeups() const { return catch_up_wakeups_.load(std::memory_order_relaxed); }

private:
    void loop() {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(world_.dt()));
        // Bounded catch-up: if a single wake-up is due for more than one
        // tick (scheduler jitter, a slow post_step, or a real stall), run
        // up to this many ticks back-to-back before re-checking the clock,
        // rather than either (a) losing rate permanently by only ever
        // running one tick per wake-up (the bug precise_sleep_until's
        // comment describes) or (b) an unbounded burst after a long stall
        // (e.g. a paused debugger) trying to fully replay lost wall time.
        constexpr int kMaxCatchUpTicksPerWakeup = 8;

        auto next = clock::now() + period;
        while (running_.load(std::memory_order_relaxed)) {
            detail::precise_sleep_until(next);

            int ran = 0;
            while (running_.load(std::memory_order_relaxed) && clock::now() >= next && ran < kMaxCatchUpTicksPerWakeup) {
                const auto step_start = clock::now();
                if (pre_step_) pre_step_();
                world_.step();
                ticks_run_.fetch_add(1, std::memory_order_relaxed);
                if (post_step_) post_step_();
                const auto step_ns = static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - step_start).count());
                std::uint64_t prev_max = max_step_ns_.load(std::memory_order_relaxed);
                while (step_ns > prev_max && !max_step_ns_.compare_exchange_weak(prev_max, step_ns, std::memory_order_relaxed)) {}
                next += period;
                ++ran;
            }
            if (ran > 1) catch_up_wakeups_.fetch_add(1, std::memory_order_relaxed);
            if (clock::now() >= next) {
                // Still behind after the bounded catch-up - a genuine stall
                // (e.g. the process was paused). Resync to "now" instead of
                // an ever-growing backlog; a fixed 240 Hz cadence resuming
                // late is the intended behaviour, not a catch-up spiral.
                next = clock::now() + period;
            }
        }
    }

    ps::World& world_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> ticks_run_{0};
    std::atomic<std::uint64_t> max_step_ns_{0};
    std::atomic<std::uint64_t> catch_up_wakeups_{0};
    std::function<void()> post_step_;
    std::function<void()> pre_step_;
};

} // namespace ps_godot
