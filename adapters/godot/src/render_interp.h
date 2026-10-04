// adapters/godot/src/render_interp.h — render-time pose interpolation
// (PLAN.md D3: "the adapter runs the sim on its own thread and reads
// interpolated snapshots").
//
// The sim thread runs at a fixed 240 Hz; Godot renders at whatever frame
// rate the host machine sustains (commonly higher, so most frames fall
// strictly between two ticks, but a slow frame can also span more than one
// tick). Reading the latest published tick's pose directly (what this
// adapter did before this file existed) therefore renders a staircase: the
// same pose for however many frames land before the next tick publishes,
// then a sudden jump - visible judder on a fast-moving body (measured:
// +-0.3 m frame-to-frame on the chassis at ~50 m/s, alternating between
// frames that saw a 1-tick vs. a 2-tick sim advance since the last frame).
//
// Fix: render at a continuous point in time ONE TICK BEHIND the sim thread,
// blending between the last two published poses instead of snapping to the
// latest one. This file holds exactly that math - Godot-free (plain ps::
// math types only, same style as origin_rebase.h/triple_buffer.h) so
// adapters/godot/tests can exercise it without linking godot-cpp, and so a
// future non-Godot adapter (a UE5 port) can reuse it unchanged.
//
// Where the two poses this blends between come from: PsSimulation's sim
// thread (post_step, ps_simulation.cpp) already keeps the previous tick's
// pose for every body/articulation link it exposes (a plain "shift the old
// latest into prev, store the new latest" per tick - see AdapterSnapshot in
// ps_simulation.h), so a Godot frame that lands between two ticks - or even
// spans a skipped tick under load - always has a well-defined (prev,
// latest) pair to blend, independent of exactly how many ticks ran between
// this frame and the last one.
//
// Teleports (PsSimulation::reset_vehicle, main.gd's zone keys): a real
// prev/latest blend across a teleport would render a smear sweeping across
// the map for one tick. The sim thread instead sets `prev := latest` for
// the teleported body on the tick the reset happens (see
// ps_simulation.cpp's post_step) - not a separate flag this header needs to
// know about, since interpolating between two IDENTICAL poses returns that
// pose at any alpha (see InterpolatedPose's own comment).

#pragma once

#include "ps/math/pose.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace ps_godot {

// A body/link's previous and latest published pose. `prev == latest` is the
// correct representation of "nothing to interpolate from" - the first tick
// after PsSimulation::start() (no real previous pose exists yet) and a
// teleported body's own tick (see this file's header comment) both use it,
// so interpolate_pose() below needs no separate "is this valid" branch:
// blending two identical poses at any alpha trivially returns that pose.
struct InterpolatedPose {
    ps::Pose prev;
    ps::Pose latest;
};

// LEGACY (kept for the A/B diagnostic switch and as the sabotage reference
// of tests/test_render_interp.cpp - production rendering uses PoseHistory
// below). Alpha measured from the WALL time a tick happened to publish: a
// publish that jitters by up to ~80 % of a tick (step cost 0.6-3.4 ms against
// a 4.17 ms period) makes the rendered pose stall (alpha clamped at 1 while a
// tick is late) and then jump - see the PoseHistory comment.
//
// Blend factor in [0, 1] for "how far the render clock (`now`) has moved
// past the moment the LATEST tick was published (`publish_time`), as a
// fraction of one sim tick period (`tick_dt_s`)". 0 renders exactly the
// previous tick's pose, 1 renders exactly the latest tick's pose.
//
// Godot typically renders faster than the sim's 240 Hz, so `now` usually
// lands inside [publish_time, publish_time + tick_dt_s) and this returns a
// value strictly between 0 and 1 - the whole point of this file (a
// continuous blend instead of the old staircase). Clamped at both ends:
// alpha > 1 (a slow frame, or the sim thread has stalled) renders the
// latest published pose rather than extrapolating past it; alpha < 0
// (clock skew, or called before any tick has published) renders the
// previous pose rather than going backwards past it.
//
// tick_dt_s <= 0 is a degenerate configuration (no sim running yet, or a
// misconfigured caller) and returns 1 - "no interpolation, just show the
// latest pose" is the safe fallback, not a divide-by-zero.
inline double render_interp_alpha(std::chrono::steady_clock::time_point publish_time, std::chrono::steady_clock::time_point now, double tick_dt_s) {
    if (tick_dt_s <= 0.0) return 1.0;
    const double elapsed_s = std::chrono::duration<double>(now - publish_time).count();
    return std::clamp(elapsed_s / tick_dt_s, 0.0, 1.0);
}

// Interpolates position (lerp) and orientation (ps::slerp - ps/math/quat.h,
// already shortest-path/hemisphere-correct: it negates the second
// quaternion when the dot product is negative before interpolating, so two
// poses that are numerically q and -q for the same rotation - which can
// happen tick to tick since a rigid body's orientation has no preferred
// sign - never produce a spurious long-way-round spin here).
//
// Deliberately plain ps:: double-precision math, called BEFORE the
// ISO->Godot basis change (frame_convert.h) - PLAN.md P2's "all conversion
// between the sim's ISO 8855 frame and Godot's happens in exactly one
// place" - so this stays upstream of that single conversion point and
// origin rebasing (origin_rebase.h) is unaffected: the caller interpolates
// two absolute ISO poses first, then converts the ONE result through
// iso_to_godot_transform, exactly like a non-interpolated pose always did.
inline ps::Pose interpolate_pose(const InterpolatedPose& p, double alpha) {
    return ps::Pose{
        ps::lerp(p.prev.position, p.latest.position, static_cast<ps::real>(alpha)),
        ps::slerp(p.prev.orientation, p.latest.orientation, static_cast<ps::real>(alpha)),
    };
}

// ---------------------------------------------------------------------------
// Nominal-clock pose history (2026-10-04, owner report "the follow camera
// bugs back and forth", car_hyper > 300 km/h).
//
// What was wrong with render_interp_alpha(): alpha was measured from the
// WALL time the sim thread happened to publish tick k. A tick is DUE at a
// fixed schedule (SimThread's `next`) but publishes when its step ends, and
// a step costs 0.6-3.4 ms (turbo, aero, generated maps, voice threads
// competing for cores) against a 4.17 ms period, so publish times jitter by
// most of a tick. The rendered pose then stalls (alpha pinned at 1 while a
// tick is late), jumps when the late tick lands, and the rendered chassis
// speed is irregular frame to frame - at 83 m/s one tick is 0.35 m, which
// the exponentially lagged chase camera shows as the car moving back and
// forth in view.
//
// Fix: every published frame is stamped with the tick's NOMINAL time (its
// scheduled due time, SimThread::tick_nominal_time()), not the wall time it
// finished. Nominal times are exactly one period apart however the step
// durations jitter. Poses of the last kCapacity ticks are kept, and a render
// frame samples the history at `now - D` on the same steady clock, blending
// the two frames that bracket that time. The rendered motion is then a
// piecewise-linear function of the SCHEDULE, independent of publish jitter.
//
// D (the render delay) must cover the publish lateness: the newest published
// frame is at least (period + lateness) old in nominal time, so
// D >= period + lateness keeps render time inside the history. When data is
// late anyway (render time ran past the newest frame) the sample clamps to
// the newest frame - never extrapolates - and the frame is counted.
//
// Discontinuities are marked per frame, not smeared:
//   * clock_break: SimThread re-anchored its schedule after a stall (or this
//     is the first tick), so the nominal gap to the previous frame does not
//     describe elapsed motion. The pair is rendered as a step (previous pose
//     until the new frame's time) instead of a slow crawl over the gap.
//   * teleported[i]: pose i was reset this tick (reset_vehicle); that pose is
//     stepped, the other poses still blend.
//
// Concurrency: single writer (sim thread, push), single reader (render
// thread, sample). One std::mutex guards the ring; the critical sections
// copy/blend a few hundred bytes (microseconds), so the sim thread is never
// blocked for a meaningful time. Chosen over a seqlock (a torn read of a
// vector<Pose> payload is formally a data race) and over per-frame
// triple-buffer reads (a 60 Hz reader sees only every ~4th tick, so the
// brackets it needs are gone). All storage is allocated by reset() (before
// the sim thread starts); push and sample allocate nothing afterwards
// (RenderSample's vector is sized on first use and reused).
using SimClock = std::chrono::steady_clock;

struct RenderSample {
    std::vector<ps::Pose> poses; // interpolated, same layout as PoseHistory::push's input
    double alpha = 1.0;          // blend factor used (0 for a stepped/early sample)
    std::uint64_t tick_a = 0;    // older bracketing frame's tick
    std::uint64_t tick_b = 0;    // newer bracketing frame's tick
    bool valid = false;          // false: history empty (no tick published yet)
    bool late = false;           // render time ran past the newest frame: clamped, not extrapolated
    bool early = false;          // render time before the oldest frame: clamped to it
    bool stepped = false;        // the bracket crossed a clock break
};

class PoseHistory {
public:
    static constexpr std::size_t kCapacity = 8;

    // Allocates every slot. Call before the sim thread starts (also resets).
    void reset(std::size_t pose_count) {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_count_ = pose_count;
        for (Slot& s : slots_) {
            s.poses.assign(pose_count, ps::Pose{});
            s.teleported.assign(pose_count, 0);
            s.time = SimClock::time_point{};
            s.tick = 0;
            s.clock_break = false;
        }
        next_ = 0;
        count_ = 0;
        frames_sampled_.store(0, std::memory_order_relaxed);
        frames_late_.store(0, std::memory_order_relaxed);
        frames_early_.store(0, std::memory_order_relaxed);
        frames_stepped_.store(0, std::memory_order_relaxed);
    }

    // Sim thread. `poses`/`teleported` must have reset()'s pose_count entries.
    // `nominal` must be strictly increasing across calls.
    void push(SimClock::time_point nominal, std::uint64_t tick, bool clock_break, const std::vector<ps::Pose>& poses, const std::vector<std::uint8_t>& teleported) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (poses.size() != pose_count_ || teleported.size() != pose_count_) return;
        Slot& s = slots_[next_];
        std::copy(poses.begin(), poses.end(), s.poses.begin());
        std::copy(teleported.begin(), teleported.end(), s.teleported.begin());
        s.time = nominal;
        s.tick = tick;
        s.clock_break = clock_break;
        next_ = (next_ + 1) % kCapacity;
        if (count_ < kCapacity) ++count_;
    }

    // Render thread: blends the two frames bracketing `render_time` into out.
    // Returns out.valid.
    bool sample(SimClock::time_point render_time, RenderSample& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        out.valid = false;
        out.late = out.early = out.stepped = false;
        if (count_ == 0) return false;
        out.poses.resize(pose_count_); // no-op after the first call
        const Slot& oldest = frame(0);
        const Slot& newest = frame(count_ - 1);
        const Slot* a = &newest;
        const Slot* b = &newest;
        double alpha = 1.0;
        if (render_time >= newest.time) {
            out.late = render_time > newest.time;
        } else if (render_time < oldest.time) {
            a = b = &oldest;
            alpha = 0.0;
            out.early = true;
        } else {
            // oldest.time <= render_time < newest.time: first frame after render_time.
            std::size_t i = 1;
            while (i < count_ && frame(i).time <= render_time) ++i;
            a = &frame(i - 1);
            b = &frame(i);
            const double span = std::chrono::duration<double>(b->time - a->time).count();
            alpha = span > 0.0 ? std::clamp(std::chrono::duration<double>(render_time - a->time).count() / span, 0.0, 1.0) : 1.0;
            if (b->clock_break) {
                out.stepped = true;
                alpha = 0.0;
            }
        }
        for (std::size_t k = 0; k < pose_count_; ++k) {
            if (a == b) {
                out.poses[k] = b->poses[k];
            } else if (b->teleported[k] != 0 || alpha <= 0.0) {
                out.poses[k] = a->poses[k]; // stepped: hold the old pose until the new frame's time
            } else {
                out.poses[k] = interpolate_pose(InterpolatedPose{a->poses[k], b->poses[k]}, alpha);
            }
        }
        out.alpha = alpha;
        out.tick_a = a->tick;
        out.tick_b = b->tick;
        out.valid = true;
        frames_sampled_.fetch_add(1, std::memory_order_relaxed);
        if (out.late) frames_late_.fetch_add(1, std::memory_order_relaxed);
        if (out.early) frames_early_.fetch_add(1, std::memory_order_relaxed);
        if (out.stepped) frames_stepped_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    [[nodiscard]] std::uint64_t frames_sampled() const { return frames_sampled_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t frames_late() const { return frames_late_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t frames_early() const { return frames_early_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t frames_stepped() const { return frames_stepped_.load(std::memory_order_relaxed); }

private:
    struct Slot {
        SimClock::time_point time{};
        std::uint64_t tick = 0;
        bool clock_break = false;
        std::vector<ps::Pose> poses;
        std::vector<std::uint8_t> teleported;
    };
    // i-th oldest stored frame (0 = oldest). Caller holds mutex_.
    const Slot& frame(std::size_t i) const { return slots_[(next_ + kCapacity - count_ + i) % kCapacity]; }

    mutable std::mutex mutex_;
    std::array<Slot, kCapacity> slots_{};
    std::size_t pose_count_ = 0;
    std::size_t next_ = 0;  // slot the next push writes
    std::size_t count_ = 0; // valid frames, <= kCapacity
    mutable std::atomic<std::uint64_t> frames_sampled_{0};
    mutable std::atomic<std::uint64_t> frames_late_{0};
    mutable std::atomic<std::uint64_t> frames_early_{0};
    mutable std::atomic<std::uint64_t> frames_stepped_{0};
};

// Render time on the nominal clock: `now - delay_ticks * tick_dt_s`.
inline SimClock::time_point render_time_for(SimClock::time_point now, double tick_dt_s, double delay_ticks) {
    return now - std::chrono::duration_cast<SimClock::duration>(std::chrono::duration<double>(tick_dt_s * delay_ticks));
}

// Fixed-bin histogram written from one thread, read from another (diagnostic
// only): counts per bin of `bin_ns`, the last bin collects the overflow.
template <std::size_t N>
class TimingHistogram {
public:
    explicit TimingHistogram(std::uint64_t bin_ns = 500000) : bin_ns_(bin_ns) {}
    void add(std::int64_t ns) {
        const std::uint64_t v = ns < 0 ? 0 : static_cast<std::uint64_t>(ns);
        const std::size_t bin = std::min<std::size_t>(static_cast<std::size_t>(v / bin_ns_), N - 1);
        bins_[bin].fetch_add(1, std::memory_order_relaxed);
    }
    [[nodiscard]] std::uint64_t count(std::size_t bin) const { return bins_[bin].load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t bin_ns() const { return bin_ns_; }
    static constexpr std::size_t size() { return N; }

private:
    std::uint64_t bin_ns_;
    std::array<std::atomic<std::uint64_t>, N> bins_{};
};

} // namespace ps_godot
