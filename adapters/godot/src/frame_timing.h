// adapters/godot/src/frame_timing.h — adapter main-thread time-per-frame
// accounting (PLAN.md P2 front-end performance target: "the adapter's
// per-frame main-thread work must fit far inside a 4.2 ms frame ... Measure
// it: adapter main-thread time per frame ... target < 1 ms").
//
// The adapter's own per-frame CPU cost is spread across several Godot
// nodes' _process callbacks (PsBodyNode3D per body/link, PsInputMap,
// PsDebugDraw, plus the demo's own GDScript: hud.gd, free_cam.gd/
// chase_cam.gd) - all called on Godot's single main thread, in the order
// those nodes were added to the tree, within the same frame. Rather than
// timing one node and calling it "the adapter", every one of those
// callbacks wraps itself with ScopedAdapterTimer (C++) or measures itself
// with Time.get_ticks_usec() and reports via PsSimulation::add_adapter_
// time_us (GDScript), so the total genuinely covers every piece of adapter
// work in that frame. hud.gd (added last in main.gd, so its _process runs
// last among these) calls PsSimulation::consume_adapter_frame_time_us()
// once per frame to read the frame's total and reset it for the next one.
//
// A single process-wide counter (not per-PsSimulation-instance) is
// deliberate and harmless: this demo only ever runs one PsSimulation, and
// the accounting is presentation/telemetry, never simulation state (same
// spirit as World's own stage-timing telemetry channels - PLAN.md D7's "no
// wall clock in the core" targets deterministic stepping logic, not
// profiling). No locking needed: every writer and the one reader all run
// on Godot's main thread, never the sim thread.

#pragma once

#include <chrono>
#include <cstdint>

namespace ps_godot {

class FrameTiming {
public:
    static void add_us(std::int64_t us) { total_us_ += us; }

    // Returns the accumulated total since the last call, then resets it.
    static std::int64_t consume_us() {
        const std::int64_t v = total_us_;
        total_us_ = 0;
        return v;
    }

private:
    static inline std::int64_t total_us_ = 0;
};

// RAII scoped timer: construct at the top of a _process/_ready body, and
// its destructor folds the elapsed wall time into FrameTiming.
class ScopedAdapterTimer {
public:
    ScopedAdapterTimer() : start_(std::chrono::steady_clock::now()) {}
    ~ScopedAdapterTimer() {
        const auto end = std::chrono::steady_clock::now();
        FrameTiming::add_us(std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count());
    }
    ScopedAdapterTimer(const ScopedAdapterTimer&) = delete;
    ScopedAdapterTimer& operator=(const ScopedAdapterTimer&) = delete;

private:
    std::chrono::steady_clock::time_point start_;
};

} // namespace ps_godot
