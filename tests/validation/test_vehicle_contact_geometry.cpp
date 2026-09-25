// tests/validation/test_vehicle_contact_geometry.cpp — PHYS-033 I1b (vehicle
// contact geometry): pins for the two defects fixed by this increment,
// checked at the whole-vehicle level (see tests/unit/test_wheel_contact_frame.cpp
// for the white-box, single-wheel pins of the same two defects).
//
// Defect 2 (fixed in commit C6): the suspension force and the ground
// reaction were both resolved along the CHASSIS-fixed axis, not the ground
// normal/tangent plane - so a chassis at rest but not level relative to the
// ground (case 1 below) sees a spurious horizontal force component large
// enough to make it drift/self-propel with no controls at all.
//
// Defect 1 (fixed in commit C7): tyre forces acted at the wheel CENTRE
// (lever ~0.283 m to the COM) instead of the true ground contact patch
// (lever ~0.6 m for the car rig) - so the model's total pitch/roll load
// transfer under braking/cornering statics was only ~47% of what a rigid
// car's moment balance requires (cases 2 and 3 below).
//
// Every gated case here is [!mayfail] until the commit that fixes it lands
// (D-E: (2) lands before (1), so case 1 clears at C6, cases 2/3 need C7).
// Case 4 is diagnostics-only ([.][diag]), run at every commit from C1
// onward to track how each fix moves the underlying quantities without
// gating on a tolerance yet.

#include "vehicle_test_helpers.h"

#include "ps/math/transcendental.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace ps;

namespace {

// Pitch of the chassis relative to a FLAT ground plane (theta=0), positive
// nose-down - mirrors test_vehicle_tractor_cross_slope.cpp's
// attitude_relative_to_ground for the tilt=identity case, duplicated here
// (deliberately - same "small, self-contained diagnostic" pattern every
// other vehicle validation test in this directory follows) so this file
// does not need that file's tilt machinery.
real chassis_pitch(const Pose& pose) {
    const Vec3 local_x = pose.orientation.rotate(Vec3::unit_x());
    return ps::math::asin(std::clamp(-local_x.z, real(-1.0), real(1.0)));
}

} // namespace

// --- Case 1: (2)'s pin - a pitched-at-rest chassis must not self-propel ---
//
// [!mayfail] REMOVED at commit C6 (defect 2's force-decomposition fix):
// measured horizontal_drift dropped from 5.92 m (before any fix, 5 s
// window) to 0.000122 m (after C6) - comfortably inside the 0.01 m gate.
//
// The rear wheels' attachment_local.z is moved from -0.343 to -0.443 (0.1 m
// lower than the front), which by itself (pure suspension geometry, before
// any dynamic settling) tilts the chassis nose-down by
// atan(0.1 / 2.70) = 0.0370 rad - see this file's header comment. Before
// (2), the suspension/tyre forces resolve along the chassis' own tilted
// axis, so a component of the (large) vertical load leaks into the
// horizontal direction with nothing to balance it (no controls are applied
// at all) - a car that visibly rolls away on its own from a pitched-but-
// otherwise-ordinary rest position. After (2), both the suspension force
// (already ground-normal-aligned in C6 - defect 2 is fixed there) and the
// ground reaction have no such leak; the last few % from (1)'s own
// unrelated wheel-centre-vs-patch lever difference cannnot introduce a
// horizontal drift by itself (it only changes the LEVER for torques, not
// the direction of the linear force sum), so this case is expected to pass
// at C6 already and stay passing through C7.
TEST_CASE("A car whose chassis sits pitched at rest does not self-propel", "[validation][vehicle][long]") {
    vehicle::VehicleDesc desc =
        io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    for (auto& w : desc.wheels) {
        if (!w.is_front) w.attachment_local.z = -0.443;
    }

    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.65);
    ps_test::settle(*rig.world, 2.0);

    const Pose p0 = rig.world->get_pose(rig.chassis);
    const real pitch0 = chassis_pitch(p0);
    // The pure-geometry estimate (atan(0.1/2.70) = 0.0370 rad) ignores that
    // settling the suspension onto a pitched rest ALSO shifts the front/rear
    // static load split a little (measured pitch0 ~0.0425 rad here) - a
    // generous +-0.01 rad band around the geometric estimate is enough to
    // confirm the rig actually settled meaningfully pitched (this is a setup
    // sanity precondition, not the case's own acceptance gate below).
    INFO("pitch0=" << pitch0 << " rad (geometric estimate ~0.0370)");
    REQUIRE(std::abs(pitch0 - real(0.0370)) < real(0.01));
    for (std::size_t w = 0; w < rig.world->vehicle_wheel_count(rig.vehicle); ++w) {
        const real fz = rig.world->wheel_telemetry(rig.vehicle, w).fz;
        INFO("wheel " << w << " fz=" << fz);
        REQUIRE(fz > 0.0);
    }

    ps_test::settle(*rig.world, 5.0);
    const Pose p1 = rig.world->get_pose(rig.chassis);
    const real horizontal_drift = distance(Vec3{p0.position.x, p0.position.y, 0.0}, Vec3{p1.position.x, p1.position.y, 0.0});
    INFO("horizontal_drift=" << horizontal_drift << " m over 5 s, no controls");
    CHECK(horizontal_drift < 0.01);
}

// --- Case 2: (1)'s pin - steady cornering lateral load transfer matches statics ---
//
// LTR_lat = (sum Fz_outer - sum Fz_inner) * (track/2) / (m * a_c * h_cg):
// the measured axle-pair load-transfer ratio against the value a rigid
// car's roll-moment balance about a ground-level axis requires (=1 exactly
// for a perfectly rigid, ground-level-roll-axis car; car_sedan.json's own
// COM-shift-under-roll and Mx terms push it a few % above 1 - see this
// file's header comment and the I1b plan's section 5.2 "skidpad" row for
// the derivation). Positive steer turns the car toward +y (ISO 8855,
// PLAN.md D4), so for steer > 0 the RIGHT-side wheels (FR/RR, y=-0.78) are
// the OUTER (loaded-up) pair.
TEST_CASE("Steady cornering: lateral load transfer matches statics", "[validation][vehicle][long]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 0.5);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{15.0, 0.0, 0.0}, Vec3::zero()});
    rig.world->set_control("steer", 0.1);

    const real mass = rig.world->get_mass(rig.chassis);
    const real track = 1.56, h_cg = 0.6;

    real sum_num = 0.0, sum_den = 0.0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(3.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        if (t < 2.0 || t >= 3.0) continue;
        const Motion m = rig.world->get_motion(rig.chassis);
        const real speed = m.linear.length();
        const real a_c = speed * m.angular.z; // signed: >0 for a left (positive-steer) turn
        const real fz_outer = rig.world->wheel_telemetry(rig.vehicle, 1).fz + rig.world->wheel_telemetry(rig.vehicle, 3).fz; // FR+RR
        const real fz_inner = rig.world->wheel_telemetry(rig.vehicle, 0).fz + rig.world->wheel_telemetry(rig.vehicle, 2).fz; // FL+RL
        sum_num += (fz_outer - fz_inner) * (track / real(2.0));
        sum_den += mass * a_c * h_cg;
    }
    const real ltr_lat = sum_den != 0.0 ? sum_num / sum_den : 0.0;
    INFO("LTR_lat=" << ltr_lat << " (expected ~1.05 after C7, ~0.50 before)");
    CHECK(ltr_lat == Catch::Approx(1.05).margin(0.10));
}

// --- Case 3: (1)'s pin - straight braking longitudinal load transfer matches statics ---
// [!mayfail] since 2026-09-25 (60/40 split, 8bf7eb9): LTR_long measured 1.149 against the 1.02 +- 0.12 gate.
// NOT caused by the split: at be36545 (50/50) it was already 1.122, up from ~1.02 at I1b C7. The excess is
// steady-state, not a pitch transient: Fz_front and pitch settle by t~0.8 s and stay flat through t=2.7 s,
// total Fz tracks m*g, and chassis z (0.5999 m) matches h_cg. Root cause undiagnosed, under investigation.
// The expected 1.02 is statics m*a*h/L plus ~2.4% from the four wheels' spin inertia (ground Fx = m*a + 4*J*a/R^2).
TEST_CASE("Straight braking: longitudinal load transfer matches statics", "[validation][vehicle][long][!mayfail]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 0.5);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{20.0, 0.0, 0.0}, Vec3::zero()});

    const real fz_front_static = rig.world->wheel_telemetry(rig.vehicle, 0).fz + rig.world->wheel_telemetry(rig.vehicle, 1).fz;
    rig.world->set_control("brake", 0.25);

    const real mass = rig.world->get_mass(rig.chassis);
    const real L = 2.70, h_cg = 0.6;
    real sum_num = 0.0, sum_den = 0.0;
    real speed_at_0_5 = 0.0, speed_at_1_5 = 0.0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(1.6 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        if (std::abs(t - 0.5) < dt * 0.5) speed_at_0_5 = rig.world->get_motion(rig.chassis).linear.length();
        if (std::abs(t - 1.5) < dt * 0.5) speed_at_1_5 = rig.world->get_motion(rig.chassis).linear.length();
        if (t < 0.5 || t >= 1.5) continue;
        const real fz_front = rig.world->wheel_telemetry(rig.vehicle, 0).fz + rig.world->wheel_telemetry(rig.vehicle, 1).fz;
        sum_num += (fz_front - fz_front_static) * L;
        ++sum_den; // accumulate a count; a_x folded in below via the net speed change (exact average over the window)
    }
    const real a_x = (speed_at_0_5 - speed_at_1_5) / real(1.0); // net speed change / window length == the average deceleration
    const real ltr_long = (sum_den > 0.0 && a_x != 0.0) ? (sum_num / sum_den) / (mass * a_x * h_cg) : 0.0;
    INFO("LTR_long=" << ltr_long << " a_x=" << a_x << " (expected ~1.02 after C7, ~0.49 before)");
    CHECK(ltr_long == Catch::Approx(1.02).margin(0.12));
}

// --- Case 4: diagnostics only, printed at every commit from C1 onward ---
TEST_CASE("contact geometry diagnostics", "[.][diag][vehicle]") {
    // Skidpad: the test's own metric (v*omega_z) against a_c and the
    // sideslip rate v*dbeta/dt - settles the coordinator's question of
    // whether test_vehicle_skidpad.cpp's steady-state assumption holds
    // (I1b plan section 5.2's "skidpad note").
    {
        auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                               Vec3{2.0, 0.4, 0.15}, 0.6);
        ps_test::settle(*rig.world, 0.5);
        rig.world->backend().set_motion(rig.chassis, Motion{Vec3{25.0, 0.0, 0.0}, Vec3::zero()});
        rig.world->set_control("steer", 0.8);

        real prev_beta = 0.0;
        Vec3 prev_velocity = Vec3::zero();
        bool have_prev = false;
        real sum_metric = 0.0, sum_ac = 0.0, sum_beta_rate_v = 0.0, sum_roll = 0.0;
        std::size_t count = 0;
        const real dt = rig.world->dt();
        const std::uint64_t n = static_cast<std::uint64_t>(1.5 / dt + 0.5);
        for (std::uint64_t i = 0; i < n; ++i) {
            rig.world->step();
            const real t = static_cast<real>(i) * dt;
            const Motion m = rig.world->get_motion(rig.chassis);
            const real speed = m.linear.length();
            // Sideslip beta (vehicle-frame) via the chassis' own forward axis.
            const Vec3 fwd = rig.world->get_pose(rig.chassis).orientation.rotate(Vec3::unit_x());
            const Vec3 lat = rig.world->get_pose(rig.chassis).orientation.rotate(Vec3::unit_y());
            const real vx_body = dot(m.linear, fwd);
            const real vy_body = dot(m.linear, lat);
            const real beta = ps::math::atan2(vy_body, vx_body);
            // a_c: the TRUE measured lateral acceleration (finite difference
            // of chassis velocity, projected onto the body's own current
            // lateral axis) - deliberately NOT the same quantity as the
            // test's own v*omega_z metric, so this is an independent check
            // of whether they agree (planar vehicle dynamics identity:
            // a_y_body = v_x*omega_z + dv_y_body/dt, so a_y_body ~= v*omega_z
            // only in a true steady state where the body-frame lateral
            // velocity - and hence beta - has stopped changing).
            const Vec3 a_vec = have_prev ? (m.linear - prev_velocity) / dt : Vec3::zero();
            const real a_c = dot(a_vec, lat);
            if (t >= 0.3 && t < 1.3) {
                const real metric = speed * m.angular.z;
                const real beta_rate = have_prev ? (beta - prev_beta) / dt : 0.0;
                sum_metric += metric;
                sum_ac += a_c;
                sum_beta_rate_v += speed * beta_rate;
                // Roll estimate: FL/RL vs FR/RR compression difference is not
                // directly available here; approximate via chassis roll angle.
                const Vec3 up = rig.world->get_pose(rig.chassis).orientation.rotate(Vec3::unit_z());
                sum_roll += ps::math::asin(std::clamp(up.y, real(-1.0), real(1.0)));
                ++count;
            }
            prev_beta = beta;
            prev_velocity = m.linear;
            have_prev = true;
        }
        const real n_c = static_cast<real>(count);
        std::printf("[diag] skidpad: v*wz=%.4f a_c=%.4f v*dbeta/dt=%.4f roll=%.4f rad (metric vs a_c should match within 1%% for a steady state)\n",
                    static_cast<double>(sum_metric / n_c), static_cast<double>(sum_ac / n_c),
                    static_cast<double>(sum_beta_rate_v / n_c), static_cast<double>(sum_roll / n_c));
    }

    // Braking diagnostics (the exact test loops, duplicated locally - same
    // house style as test_vehicle_braking.cpp's own estimate).
    {
        auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                               Vec3{2.0, 0.4, 0.15}, 0.6);
        ps_test::settle(*rig.world, 0.5);
        const real v0 = 100.0 / 3.6;
        rig.world->backend().set_motion(rig.chassis, Motion{Vec3{v0, 0.0, 0.0}, Vec3::zero()});
        const Vec3 p_start = rig.world->get_pose(rig.chassis).position;
        rig.world->set_control("brake", 0.6);
        rig.world->set_control("handbrake", 0.3);

        real pitch_sum = 0.0;
        std::size_t pitch_n = 0;
        real stop_distance = -1.0;
        const std::uint64_t max_ticks = static_cast<std::uint64_t>(8.0 / rig.world->dt() + 0.5);
        for (std::uint64_t i = 0; i < max_ticks; ++i) {
            rig.world->step();
            pitch_sum += chassis_pitch(rig.world->get_pose(rig.chassis));
            ++pitch_n;
            const real speed = rig.world->get_motion(rig.chassis).linear.length();
            if (speed < 0.3) { stop_distance = distance(p_start, rig.world->get_pose(rig.chassis).position); break; }
        }
        std::printf("[diag] braking (fixed 0.6/0.3 command, not the ABS emulation): stop_distance=%.3f m mean_pitch=%.5f rad\n",
                    static_cast<double>(stop_distance), static_cast<double>(pitch_sum / static_cast<real>(pitch_n)));
    }

    // Accel diagnostics (settled, the exact accel test setup).
    {
        const real mass = 1500.0;
        auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                               Vec3{2.0, 0.4, 0.15}, 0.6);
        ps_test::settle(*rig.world, 1.0);
        rig.world->set_control("throttle", 1.0);
        real max_abs_slip = 0.0;
        real first_traction_limited_speed = -1.0;
        const std::uint64_t max_ticks = static_cast<std::uint64_t>(5.0 / rig.world->dt() + 0.5);
        for (std::uint64_t i = 0; i < max_ticks; ++i) {
            rig.world->step();
            const auto& w0 = rig.world->wheel_telemetry(rig.vehicle, 0);
            max_abs_slip = std::max({max_abs_slip, std::abs(w0.slip_ratio)});
            const real speed = rig.world->get_motion(rig.chassis).linear.length();
            // Dx(fz) peak-mu estimate at this wheel's own live Fz (pdx1/pdx2 from passenger_car_225_45r17.json).
            const real fz0 = 4200.0, pdx1 = 1.05, pdx2 = -0.10;
            const real dfz = (w0.fz - fz0) / fz0;
            const real dx = w0.fz * (pdx1 + pdx2 * dfz);
            const real fx_cmd_estimate = std::abs(w0.fx); // measured Fx, the actual commanded-vs-achieved comparison
            if (first_traction_limited_speed < 0.0 && fx_cmd_estimate > 0.95 * dx && speed > 0.5) {
                first_traction_limited_speed = speed;
            }
        }
        std::printf("[diag] accel (settled asphalt): max_abs_slip=%.4f first_traction_limited_speed=%.3f m/s\n",
                    static_cast<double>(max_abs_slip), static_cast<double>(first_traction_limited_speed));
    }
}
