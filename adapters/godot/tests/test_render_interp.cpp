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

    if (g_failures == 0) {
        std::printf("test_render_interp: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_render_interp: %d check(s) FAILED\n", g_failures);
    return g_failures;
}
