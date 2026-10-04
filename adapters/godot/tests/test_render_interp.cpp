// adapters/godot/tests/test_render_interp.cpp — numeric acceptance test for
// render_interp.h (PLAN.md D3 render-time interpolation, judder fix - see
// that file's header comment for the measured problem this closes).
//
// Plain main(), no framework, Godot-free - same pattern as
// test_origin_rebase.cpp (render_interp.h takes no godot_cpp include either,
// only ps:: math + <chrono>).

#include "../src/render_interp.h"

#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

using namespace ps_godot;
using namespace std::chrono_literals;

namespace {

int g_failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what);
    }
}

double pose_position_error(const ps::Pose& a, const ps::Pose& b) {
    return (a.position - b.position).length();
}

// Orientation error via a rotated probe vector, same technique
// test_origin_rebase.cpp uses - avoids the q vs -q sign ambiguity a raw
// component comparison would falsely flag as an error.
double pose_orientation_error(const ps::Pose& a, const ps::Pose& b) {
    const ps::Vec3 probe{0.37, -0.81, 1.4};
    return (a.orientation.rotate(probe) - b.orientation.rotate(probe)).length();
}

// ---------------------------------------------------------------------------
// Nominal-clock history (PoseHistory) against a synthetic jittered publish
// schedule. The sim moves a body with a known trajectory x(t) (tick k's pose
// is x(k*dt), "nominal time" k*dt) and tick k is published at
// k*dt + lateness_k, lateness in [0.6, 3.4] ms - the step-cost range measured
// on car_hyper (out/audio_steps/demo_rec2.csv.timing.txt) - never before the
// previous publish. A 60 Hz render loop with +-2 ms frame jitter renders
// either through the OLD publish-time alpha or through PoseHistory and
// records the rendered position. The frame-to-frame rendered velocity is
// compared with the true velocity: for a constant 83 m/s the NEW path must
// be exact, the OLD one is off by metres per second (sabotage reference).
// ---------------------------------------------------------------------------

struct Lcg {
    std::uint64_t s = 0x9e3779b97f4a7c15ull;
    double uniform() { // [0, 1)
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<double>(s >> 11) * (1.0 / 9007199254740992.0);
    }
};

const SimClock::time_point g_base = SimClock::now();
SimClock::time_point tp(double seconds) { return g_base + std::chrono::duration_cast<SimClock::duration>(std::chrono::duration<double>(seconds)); }

struct Trajectory {
    double v0 = 83.0;
    double accel = 0.0;
    double x(double t) const { return v0 * t + 0.5 * accel * t * t; }
    double v(double t) const { return v0 + accel * t; }
};

struct RunResult {
    double max_v_err = 0.0;     // max |frame-to-frame rendered velocity - true velocity|
    double rms_v_err = 0.0;
    std::uint64_t late = 0;
    std::uint64_t frames = 0;
};

// legacy == true: OLD publish-time alpha. delay_ticks: D for the new path.
// stamp_with_publish == true (sabotage): PoseHistory frames are stamped with the
// WALL publish time instead of the nominal tick time.
RunResult run_schedule(const Trajectory& tr, bool legacy, double delay_ticks, double max_lateness_s, std::uint64_t seed_shift, bool stamp_with_publish = false) {
    constexpr double kDt = 1.0 / 240.0;
    constexpr double kDuration = 6.0;
    Lcg rng;
    rng.s ^= seed_shift;

    // Publish schedule.
    const int ticks = static_cast<int>(kDuration / kDt) + 4;
    std::vector<double> publish(static_cast<std::size_t>(ticks));
    double last_pub = -1.0;
    for (int k = 0; k < ticks; ++k) {
        const double lateness = 0.0006 + rng.uniform() * (max_lateness_s - 0.0006);
        publish[static_cast<std::size_t>(k)] = std::max(k * kDt + lateness, last_pub + 1e-6);
        last_pub = publish[static_cast<std::size_t>(k)];
    }

    PoseHistory history;
    history.reset(2);
    RenderSample sample;
    std::vector<ps::Pose> poses(2);
    const std::vector<std::uint8_t> no_teleport(2, 0);

    int published = 0; // number of ticks published so far
    ps::Pose legacy_prev{}, legacy_latest{};
    SimClock::time_point legacy_publish_time{};

    RunResult r;
    double prev_wall = -1.0, prev_x = 0.0, sum_sq = 0.0;
    int n_vel = 0;
    // Render loop: 60 Hz with +-2 ms jitter, from 0.5 s (history full) to the end.
    for (double nominal_frame = 0.5; nominal_frame < kDuration - 0.1; nominal_frame += 1.0 / 60.0) {
        const double wall = nominal_frame + (rng.uniform() - 0.5) * 0.004;
        while (published < ticks && publish[static_cast<std::size_t>(published)] <= wall) {
            const double t = published * kDt;
            poses[0] = ps::Pose{ps::Vec3{tr.x(t), 0.0, 0.0}, ps::Quat::identity()};
            poses[1] = ps::Pose{ps::Vec3{0.0, 2.0 * tr.x(t), 0.0}, ps::Quat::identity()};
            history.push(stamp_with_publish ? tp(publish[static_cast<std::size_t>(published)]) : tp(t), static_cast<std::uint64_t>(published), published == 0, poses, no_teleport);
            legacy_prev = published == 0 ? poses[0] : legacy_latest;
            legacy_latest = poses[0];
            legacy_publish_time = tp(publish[static_cast<std::size_t>(published)]);
            ++published;
        }
        double x;
        if (legacy) {
            const double alpha = render_interp_alpha(legacy_publish_time, tp(wall), kDt);
            x = interpolate_pose(InterpolatedPose{legacy_prev, legacy_latest}, alpha).position.x;
        } else {
            history.sample(render_time_for(tp(wall), kDt, delay_ticks), sample);
            x = sample.poses[0].position.x;
            // Same-frame consistency: pose 1 must be the same instant as pose 0.
            if (std::fabs(sample.poses[1].position.y - 2.0 * x) > 1e-6) { ++r.late; r.late += 1000000; }
        }
        if (prev_wall >= 0.0) {
            const double v = (x - prev_x) / (wall - prev_wall);
            // True velocity at the middle of the frame interval, shifted by the delay the path renders at.
            const double t_mid = 0.5 * (wall + prev_wall) - (legacy ? 0.0 : delay_ticks * kDt);
            const double err = std::fabs(v - tr.v(t_mid));
            r.max_v_err = std::max(r.max_v_err, err);
            sum_sq += err * err;
            ++n_vel;
        }
        prev_wall = wall;
        prev_x = x;
        ++r.frames;
    }
    r.late += history.frames_late();
    r.rms_v_err = n_vel > 0 ? std::sqrt(sum_sq / n_vel) : 0.0;
    return r;
}

void run_history_tests() {
    constexpr double kLateMax = 0.0034; // measured max step ms on car_hyper

    // --- smooth rendered motion under publish jitter --------------------
    {
        Trajectory tr;
        const RunResult legacy = run_schedule(tr, true, 0.0, kLateMax, 0);
        const RunResult fresh = run_schedule(tr, false, 2.0, kLateMax, 0);
        std::printf("info: const 83 m/s, 60 Hz render, lateness 0.6-3.4 ms: OLD max |v err| %.3f m/s (rms %.3f), NEW max |v err| %.6f m/s (rms %.6f), NEW late frames %llu\n",
                    legacy.max_v_err, legacy.rms_v_err, fresh.max_v_err, fresh.rms_v_err, static_cast<unsigned long long>(fresh.late));
        constexpr double kBound = 1e-3; // m/s: exact up to rounding for constant velocity
        check(fresh.late == 0, "history: D = 2 ticks covers 0.6-3.4 ms publish lateness (0 late frames, same-frame poses consistent)");
        check(fresh.max_v_err < kBound, "history: rendered velocity error < 1e-3 m/s at 83 m/s despite publish jitter");
        check(legacy.max_v_err > 100.0 * kBound, "sabotage reference: OLD publish-time alpha violates that bound (guard discriminates)");
        check(legacy.max_v_err > 1.0, "sabotage reference: OLD publish-time alpha is off by > 1 m/s frame to frame (the owner-visible shake)");
    }
    // Sabotage of the fix itself: history stamped with the wall publish time.
    {
        Trajectory tr;
        const RunResult wall_stamped = run_schedule(tr, false, 2.0, kLateMax, 0, true);
        std::printf("info: history stamped with PUBLISH wall time: max |v err| %.3f m/s\n", wall_stamped.max_v_err);
        check(wall_stamped.max_v_err > 1.0, "sabotage reference: stamping frames with the wall publish time (not the nominal tick time) fails the bound");
    }
    // Several jitter realisations.
    {
        Trajectory tr;
        double worst_new = 0.0, worst_old = 1e9;
        for (std::uint64_t s = 1; s <= 5; ++s) {
            worst_new = std::max(worst_new, run_schedule(tr, false, 2.0, kLateMax, s * 7919).max_v_err);
            worst_old = std::min(worst_old, run_schedule(tr, true, 0.0, kLateMax, s * 7919).max_v_err);
        }
        check(worst_new < 1e-3, "history: bound holds for 5 jitter seeds");
        check(worst_old > 1.0, "sabotage reference: OLD alpha fails the bound for every one of 5 seeds");
    }
    // Accelerating trajectory: piecewise-linear interpolation error a*dt/2 = 0.02 m/s.
    {
        Trajectory tr;
        tr.v0 = 20.0;
        tr.accel = 10.0;
        const RunResult fresh = run_schedule(tr, false, 2.0, kLateMax, 0);
        const RunResult legacy = run_schedule(tr, true, 0.0, kLateMax, 0);
        std::printf("info: accelerating 10 m/s^2: NEW max |v err| %.5f m/s, OLD %.3f m/s\n", fresh.max_v_err, legacy.max_v_err);
        check(fresh.max_v_err < 0.05, "history: accelerating body, velocity error < 0.05 m/s");
        check(legacy.max_v_err > 1.0, "sabotage reference: OLD alpha off by > 1 m/s on the accelerating body too");
    }
    // D must cover period + lateness: D = 1 tick leaves frames late.
    {
        Trajectory tr;
        const RunResult short_d = run_schedule(tr, false, 1.0, kLateMax, 0);
        std::printf("info: D = 1 tick -> %llu late frames of %llu\n", static_cast<unsigned long long>(short_d.late), static_cast<unsigned long long>(short_d.frames));
        check(short_d.late > 0, "history: D = 1 tick is too short for 3.4 ms lateness (late frames are counted, not hidden)");
    }

    // --- bracket, clamp, ring wrap ---------------------------------------
    {
        PoseHistory h;
        h.reset(1);
        RenderSample s;
        const std::vector<std::uint8_t> nt(1, 0);
        check(!h.sample(tp(1.0), s) && !s.valid, "history: empty history -> invalid sample");
        constexpr double kDt = 1.0 / 240.0;
        for (int k = 0; k < 20; ++k) { // wraps the 8-slot ring twice
            h.push(tp(k * kDt), static_cast<std::uint64_t>(k), k == 0, {ps::Pose{ps::Vec3{k * 1.0, 0.0, 0.0}, ps::Quat::identity()}}, nt);
        }
        h.sample(tp(10.0 * kDt), s); // warm-up: sizes the output buffer
        const ps::Pose* data_before = s.poses.data();
        check(h.sample(tp(15.5 * kDt), s) && std::fabs(s.poses[0].position.x - 15.5) < 1e-9 && s.tick_a == 15 && s.tick_b == 16, "history: bracket after ring wrap blends ticks 15/16 at alpha 0.5");
        check(h.sample(tp(12.0 * kDt), s) && !s.early && std::fabs(s.poses[0].position.x - 12.0) < 1e-9, "history: oldest retained frame (tick 12) still available");
        check(h.sample(tp(5.0 * kDt), s) && s.early && std::fabs(s.poses[0].position.x - 12.0) < 1e-9, "history: before the oldest frame clamps to it and is flagged early");
        check(h.sample(tp(19.0 * kDt + 0.05), s) && s.late && std::fabs(s.poses[0].position.x - 19.0) < 1e-9, "history: past the newest frame clamps to it (no extrapolation) and is flagged late");
        check(s.poses.data() == data_before, "history: sample() reuses the output buffer (no reallocation after warm-up)");
    }

    // --- teleport: stepped for the teleported pose only ------------------
    {
        PoseHistory h;
        h.reset(2);
        RenderSample s;
        constexpr double kDt = 1.0 / 240.0;
        for (int k = 0; k < 6; ++k) {
            const bool tele = (k == 4);
            const double a = tele ? 1000.0 : k * 1.0; // pose 0 jumps to 1000 at tick 4
            h.push(tp(k * kDt), static_cast<std::uint64_t>(k), k == 0,
                   {ps::Pose{ps::Vec3{a, 0.0, 0.0}, ps::Quat::identity()}, ps::Pose{ps::Vec3{0.0, k * 1.0, 0.0}, ps::Quat::identity()}},
                   {static_cast<std::uint8_t>(tele ? 1 : 0), 0});
        }
        double worst_between = 1e9;
        bool other_pose_blended = true;
        for (int i = 0; i <= 40; ++i) {
            const double t = (3.0 + i / 40.0) * kDt; // across the teleport tick (3 -> 4)
            h.sample(tp(t), s);
            const double x = s.poses[0].position.x;
            worst_between = std::min(worst_between, std::min(std::fabs(x - 3.0), std::fabs(x - 1000.0)));
            if (i == 20) other_pose_blended = std::fabs(s.poses[1].position.y - 3.5) < 1e-9;
        }
        check(worst_between < 1e-9, "teleport: rendered pose is only ever the old or the new position (no smear)");
        check(other_pose_blended, "teleport: a non-teleported pose still blends");
    }

    // --- clock break (stall resync): step, no crawl over the gap ---------
    {
        PoseHistory h;
        h.reset(1);
        RenderSample s;
        constexpr double kDt = 1.0 / 240.0;
        for (int k = 0; k < 4; ++k) h.push(tp(k * kDt), static_cast<std::uint64_t>(k), k == 0, {ps::Pose{ps::Vec3{k * 1.0, 0.0, 0.0}, ps::Quat::identity()}}, std::vector<std::uint8_t>(1, 0));
        // 300 ms stall, then the sim resumes with a re-anchored schedule.
        const double resume = 3 * kDt + 0.3;
        h.push(tp(resume), 4, true, {ps::Pose{ps::Vec3{4.0, 0.0, 0.0}, ps::Quat::identity()}}, std::vector<std::uint8_t>(1, 0));
        h.push(tp(resume + kDt), 5, false, {ps::Pose{ps::Vec3{5.0, 0.0, 0.0}, ps::Quat::identity()}}, std::vector<std::uint8_t>(1, 0));
        bool never_crawls = true, saw_stepped = false;
        for (int i = 0; i < 200; ++i) {
            const double t = 3 * kDt + (resume - 3 * kDt) * i / 200.0; // inside the gap
            h.sample(tp(t), s);
            saw_stepped = saw_stepped || s.stepped;
            never_crawls = never_crawls && std::fabs(s.poses[0].position.x - 3.0) < 1e-9;
        }
        check(saw_stepped && never_crawls, "clock break: across a stall gap the pose holds, it does not crawl from the old pose to the new one");
        h.sample(tp(resume + 0.5 * kDt), s);
        check(std::fabs(s.poses[0].position.x - 4.5) < 1e-9, "clock break: normal blending resumes after the break");
    }

    // --- writer thread vs reader: no torn frames --------------------------
    {
        PoseHistory h;
        h.reset(2);
        std::atomic<bool> stop{false};
        std::thread writer([&] {
            std::vector<ps::Pose> p(2);
            const std::vector<std::uint8_t> nt(2, 0);
            std::uint64_t k = 0;
            while (!stop.load()) {
                const double x = static_cast<double>(k);
                p[0] = ps::Pose{ps::Vec3{x, 0.0, 0.0}, ps::Quat::identity()};
                p[1] = ps::Pose{ps::Vec3{0.0, 3.0 * x, 0.0}, ps::Quat::identity()};
                h.push(SimClock::now(), k, k == 0, p, nt);
                ++k;
            }
        });
        RenderSample s;
        bool torn = false;
        int samples = 0;
        const auto end = SimClock::now() + std::chrono::milliseconds(150);
        while (SimClock::now() < end) {
            if (h.sample(SimClock::now() - std::chrono::microseconds(50), s)) {
                ++samples;
                if (std::fabs(s.poses[1].position.y - 3.0 * s.poses[0].position.x) > 1e-6) torn = true;
            }
        }
        stop = true;
        writer.join();
        std::printf("info: concurrent sampling: %d samples\n", samples);
        check(samples > 0 && !torn, "threads: a sampled frame never mixes two ticks (no torn reads)");
    }
}

} // namespace

int main() {
    const auto t0 = std::chrono::steady_clock::now();
    constexpr double kTickDt = 1.0 / 240.0;

    // --- render_interp_alpha: basic range + clamping --------------------
    {
        check(render_interp_alpha(t0, t0, kTickDt) == 0.0, "alpha: now == publish_time -> 0");
        // microseconds() truncates kTickDt*1e6 to an integer count, so the
        // reconstructed duration is a few ns short of one exact tick -
        // compare with a tolerance, not bit-exact equality.
        check(std::fabs(render_interp_alpha(t0, t0 + std::chrono::microseconds(static_cast<long long>(kTickDt * 1e6)), kTickDt) - 1.0) < 1e-3,
              "alpha: now == publish_time + one full tick -> ~1");

        // Half a tick later -> alpha ~0.5.
        const auto half = t0 + std::chrono::microseconds(static_cast<long long>(kTickDt * 1e6 / 2.0));
        const double a_half = render_interp_alpha(t0, half, kTickDt);
        std::printf("info: alpha at half a tick = %.6f\n", a_half);
        check(std::fabs(a_half - 0.5) < 1e-3, "alpha: halfway through a tick -> ~0.5");

        // Clamp beyond both ends.
        const auto way_past = t0 + 10s;
        check(render_interp_alpha(t0, way_past, kTickDt) == 1.0, "alpha: clamps to 1 far past the tick period (slow frame / stalled sim)");
        const auto before = t0 - 5ms;
        check(render_interp_alpha(t0, before, kTickDt) == 0.0, "alpha: clamps to 0 for a `now` before publish_time (clock skew)");

        // Degenerate tick_dt_s.
        check(render_interp_alpha(t0, t0, 0.0) == 1.0, "alpha: tick_dt_s <= 0 falls back to 1 (no interpolation), not a NaN/inf");
        check(render_interp_alpha(t0, t0, -1.0) == 1.0, "alpha: negative tick_dt_s also falls back to 1");
    }

    // --- interpolate_pose: alpha 0 -> prev, alpha 1 -> latest ------------
    {
        const ps::Pose prev{ps::Vec3{0.0, 0.0, 0.0}, ps::Quat::identity()};
        const ps::Pose latest{ps::Vec3{10.0, -4.0, 2.0}, ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, 0.6)};
        const InterpolatedPose ip{prev, latest};

        const ps::Pose at0 = interpolate_pose(ip, 0.0);
        check(pose_position_error(at0, prev) < 1e-12, "interpolate_pose: alpha 0 -> prev position exactly");
        check(pose_orientation_error(at0, prev) < 1e-12, "interpolate_pose: alpha 0 -> prev orientation exactly");

        const ps::Pose at1 = interpolate_pose(ip, 1.0);
        check(pose_position_error(at1, latest) < 1e-12, "interpolate_pose: alpha 1 -> latest position exactly");
        check(pose_orientation_error(at1, latest) < 1e-9, "interpolate_pose: alpha 1 -> latest orientation exactly");

        // Midpoint: position is the arithmetic mean (plain lerp).
        const ps::Pose at_half = interpolate_pose(ip, 0.5);
        const ps::Vec3 expected_mid_pos{5.0, -2.0, 1.0};
        std::printf("info: midpoint position error = %.9f\n", pose_position_error(at_half, ps::Pose{expected_mid_pos, at_half.orientation}));
        check(pose_position_error(at_half, ps::Pose{expected_mid_pos, at_half.orientation}) < 1e-12, "interpolate_pose: alpha 0.5 -> exact midpoint position");
    }

    // --- interpolate_pose: first tick / teleport (prev == latest) -------
    {
        const ps::Pose only{ps::Vec3{123.0, 45.0, 6.0}, ps::Quat::from_axis_angle(ps::Vec3{1.0, 0.0, 0.0}, 1.2)};
        const InterpolatedPose ip{only, only};
        for (double alpha : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            const ps::Pose out = interpolate_pose(ip, alpha);
            check(pose_position_error(out, only) < 1e-12 && pose_orientation_error(out, only) < 1e-12, "interpolate_pose: prev == latest (first tick / teleport) returns that pose at any alpha");
        }
    }

    // --- quaternion slerp: shortest path / hemisphere handling ----------
    {
        // q and -q represent the identical rotation. Interpolating from a
        // real rotation to the NEGATED representation of a nearby one must
        // still take the short way round (ps::slerp flips the sign
        // internally when the dot product is negative) - a naive
        // component-wise slerp without that fix would spin the long way.
        const ps::Quat qa = ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, 0.1);
        ps::Quat qb = ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, 0.3);
        qb = ps::Quat{-qb.x, -qb.y, -qb.z, -qb.w}; // same rotation, negated representation
        const InterpolatedPose ip{ps::Pose{ps::Vec3::zero(), qa}, ps::Pose{ps::Vec3::zero(), qb}};

        const ps::Vec3 probe{1.0, 0.0, 0.0};
        double max_step = 0.0;
        ps::Vec3 prev_rotated = interpolate_pose(ip, 0.0).orientation.rotate(probe);
        constexpr int kSteps = 20;
        for (int i = 1; i <= kSteps; ++i) {
            const double alpha = static_cast<double>(i) / kSteps;
            const ps::Vec3 rotated = interpolate_pose(ip, alpha).orientation.rotate(probe);
            max_step = std::max(max_step, (rotated - prev_rotated).length());
            prev_rotated = rotated;
        }
        // The whole sweep is only 0.1 -> 0.3 rad (0.2 rad total); if this
        // took the long way round (through ~2*pi - 0.2 rad) any single one
        // of 20 even steps would be far larger than a smooth short sweep's
        // per-step motion on a unit-distance probe.
        std::printf("info: max per-step probe motion across the slerp sweep = %.6f\n", max_step);
        check(max_step < 0.05, "slerp: q vs -q hemisphere flip takes the short way round, not the long way");

        // And the endpoints still land on the correct (non-negated-looking)
        // rotations, compared via the rotated probe (sign-independent).
        const ps::Pose end = interpolate_pose(ip, 1.0);
        check((end.orientation.rotate(probe) - qb.rotate(probe)).length() < 1e-9, "slerp: alpha 1 still lands exactly on the (negated-representation) latest rotation");
    }

    // --- Continuity across a new publish ---------------------------------
    // A regular-cadence tick stream: the rendered pose the instant BEFORE a
    // new tick's snapshot publishes (alpha -> 1 against the OLD (prev,
    // latest) pair) must equal the rendered pose the instant AFTER (alpha
    // -> 0 against the NEW pair, whose prev is the old pair's latest) -
    // otherwise every tick boundary would itself be a visible pop, which is
    // exactly the judder this file exists to remove.
    {
        const ps::Pose poseA{ps::Vec3{0.0, 0.0, 0.0}, ps::Quat::identity()};
        const ps::Pose poseB{ps::Vec3{1.0, 2.0, -3.0}, ps::Quat::from_axis_angle(ps::Vec3{0.0, 1.0, 0.0}, 0.4)};
        const ps::Pose poseC{ps::Vec3{2.5, 1.5, -1.0}, ps::Quat::from_axis_angle(ps::Vec3{0.0, 1.0, 0.0}, 0.9)};

        const InterpolatedPose old_pair{poseA, poseB}; // this tick's (prev, latest)
        const InterpolatedPose new_pair{poseB, poseC}; // next tick's, prev shifted from the old latest

        const ps::Pose just_before = interpolate_pose(old_pair, 1.0);
        const ps::Pose just_after = interpolate_pose(new_pair, 0.0);

        const double pos_err = pose_position_error(just_before, just_after);
        const double orient_err = pose_orientation_error(just_before, just_after);
        std::printf("info: tick-boundary continuity error: position %.12f m, orientation-probe %.12f\n", pos_err, orient_err);
        check(pos_err < 1e-9, "continuity: rendered position just before/after a new publish matches to < 1e-9 m");
        check(orient_err < 1e-9, "continuity: rendered orientation just before/after a new publish matches to < 1e-9");
    }

    run_history_tests();

    if (g_failures == 0) {
        std::printf("test_render_interp: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_render_interp: %d check(s) FAILED\n", g_failures);
    return g_failures;
}
