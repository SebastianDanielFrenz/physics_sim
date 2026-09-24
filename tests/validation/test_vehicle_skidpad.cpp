// Validation: steady-state skidpad lateral acceleration within 3% of the
// tyre-limited estimate (PLAN.md P3 acceptance).
//
// PHYS-033 P4 I2 pre-step P2 (this file's own rewrite): the open-loop fixed-
// steer version of this test (steer=0.8, constant, no speed control) was
// found by I1b's diagnostic ("contact geometry diagnostics" [.][diag] case,
// test_vehicle_contact_geometry.cpp) to NOT be measuring a steady state at
// all: the test's own metric v*omega_z averaged 9.037 m/s^2 over the
// [0.3, 1.3] s window, but the TRUE measured lateral acceleration (finite
// difference of chassis velocity, projected onto the body's own lateral
// axis) was only 8.535 m/s^2 - a 5.9% gap - with v*dbeta/dt = -0.713 m/s^2,
// not negligible against either number. The planar identity
// a_y_body = v*omega_z + dv_y_body/dt (i.e. v*omega_z = a_y_body -
// v*dbeta/dt) shows why: v*omega_z only equals the true lateral
// acceleration when the body-frame lateral velocity - and hence the
// sideslip angle beta - has stopped changing. At steer=0.8 (~24 deg wheel
// angle) and 25 m/s the car was still rotating/settling within that window,
// so the test's own premise (measuring "steady-state" g) did not hold.
//
// This rewrite replaces the open-loop fixed steer with an actual closed
// loop that converges to a genuine steady circular turn, then ONLY measures
// once yaw rate, speed and sideslip have all stopped changing (asserted,
// not assumed):
//   1. A short OPEN-LOOP calibration phase (same steer=0.8, same 25 m/s
//      start, same [0.3, 1.3] s averaging window as the old test) finds a
//      near-the-limit operating point (target curvature = |yaw_rate|/speed,
//      target speed = speed) - i.e. "roughly what the old test was trying
//      to hold", not a value derived from the estimate below (that would be
//      circular).
//   2. A closed-loop phase: a PI controller on curvature error
//      (yaw_rate/speed vs. the calibrated target) drives the "steer"
//      channel, and an independent PI controller on speed error drives
//      "throttle" (0 brake - only enough throttle to hold speed against
//      rolling resistance/tyre drag; the calibrated target speed is well
//      inside the placeholder drive's torque, so this never saturates
//      throttle in steady state). This is the same technique ("a previous
//      anti-Ackermann measurement") used to hold a fixed operating point
//      instead of drifting through one.
//   3. Steadiness is asserted (not assumed) over the trailing window: the
//      range (max-min) of yaw rate, speed and sideslip beta must each stay
//      within a tight bound relative to their own mean.
//   4. Only then is v*omega_z used to compute the achieved lateral
//      acceleration - valid exactly because steadiness (in particular,
//      constant beta) has just been asserted, closing the gap the old
//      open-loop version could not: a_y_body = v*omega_z + dv_y_body/dt,
//      and a genuinely steady turn has dv_y_body/dt ~= 0.
//
// "Tyre-limited estimate" (unchanged formula - the estimate is a property
// of the tyre/mass/geometry data, not of how the manoeuvre is driven; see
// PHYS-033 P4 I2 plan section 0's stop condition 6 style guidance "recomputed
// for the lever the model now has" - the load-transfer lever (h_cg, track,
// roll-stiffness split) is exactly what this formula already uses, so
// "recomputed" here means re-run against the NEW closed-loop steady-state
// measurement, not a formula change): if every tyre reached its own peak
// lateral coefficient simultaneously, a_lat_max = sum_i(Dy_i(Fz_i)) / mass,
// refined for lateral load transfer (Milliken, "Race Car Vehicle Dynamics",
// ch. 18), split between front/rear by each axle's OWN roll stiffness
// (spring + ARB - car_sedan.json's front/rear ARB rates deliberately
// differ). See out/handover_p3_fix.md and vault PHYS-018 for the derivation.

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>

using namespace ps;

namespace {

real tyre_limited_lateral_g_estimate() {
    const real mass = 1500.0, g = 9.81;
    const real fz0 = 4200.0, pdy1 = 1.0, pdy2 = -0.12;
    const real corner_weight = mass * g / 4.0;
    const real track = 1.56, h_cg = 0.6;

    const real spring_rate = 61312.5, arb_front = 15000.0, arb_rear = 10000.0;
    const real k_front = spring_rate + 2.0 * arb_front;
    const real k_rear = spring_rate + 2.0 * arb_rear;
    const real frac_front = k_front / (k_front + k_rear);
    const real frac_rear = k_rear / (k_front + k_rear);

    auto dy = [&](real fz) {
        const real dfz = (fz - fz0) / fz0;
        return fz * (pdy1 + pdy2 * dfz);
    };
    auto axle_dy_sum = [&](real dFz_axle) {
        const real outer = corner_weight + dFz_axle / 2.0;
        const real inner = corner_weight - dFz_axle / 2.0;
        return dy(outer) + dy(inner);
    };

    real a_lat = pdy1 * g;
    for (int i = 0; i < 8; ++i) {
        const real dFz_total = (mass * a_lat * h_cg) / track;
        const real dy_front = axle_dy_sum(dFz_total * frac_front);
        const real dy_rear = axle_dy_sum(dFz_total * frac_rear);
        a_lat = (dy_front + dy_rear) / mass;
    }
    return a_lat;
}

real range_of(const std::deque<real>& d) {
    const auto [mn, mx] = std::minmax_element(d.begin(), d.end());
    return *mx - *mn;
}

real mean_of(const std::deque<real>& d) {
    real s = 0.0;
    for (real v : d) s += v;
    return s / static_cast<real>(d.size());
}

} // namespace

// PHYS-033 P4 I2 pre-step P2: was [!mayfail] since I1b commit C7 (measured
// 9.037 via the flawed open-loop metric, -7.8% vs. the old estimate). See
// this file's header comment for the rewrite. The tolerance is NOT loosened
// (still 3%); the genuine closed-loop steady state (all three steadiness
// checks below pass tightly - yaw rate/speed/beta all settle to a real
// limit cycle, not a drift) measures steady_a_lat = 8.425 m/s^2 against the
// estimate's 9.803 m/s^2, i.e. -14.1%, still well outside 3% - consistent
// with (and close to) I1b's independent finding of a_c = 8.535 m/s^2 at a
// different (open-loop, non-steady) operating point. Per the I2 plan's
// explicit "report, do not force" stop condition, this stays [!mayfail]
// with the numbers, not fixed by retuning the estimate or the tolerance:
// the corrected contact-patch/roll lever (I1b) makes the simulated tyres
// reach their combined longitudinal+lateral+load-transfer limit at a
// materially lower a_lat than the estimate's simplified per-axle
// roll-stiffness split predicts - a genuine model-vs-estimate gap, not a
// test-harness defect (the harness itself is now a verified real steady
// state, which the old open-loop version was not).
TEST_CASE("Skidpad lateral acceleration is within 3% of the tyre-limited estimate", "[validation][vehicle][long][!mayfail]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);

    ps_test::settle(*rig.world, 0.5); // settle vertical suspension state before adding speed

    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{25.0, 0.0, 0.0}, Vec3::zero()});
    rig.world->set_control("steer", 0.8);

    const real dt = rig.world->dt();

    // --- Phase 1: open-loop calibration (same manoeuvre the old test used) ---
    // Finds a near-the-limit (curvature, speed) operating point to hold
    // closed-loop below - NOT derived from the estimate itself.
    real sum_yaw = 0.0, sum_speed = 0.0;
    std::size_t calib_count = 0;
    const std::uint64_t n_calib = static_cast<std::uint64_t>(1.5 / dt + 0.5);
    for (std::uint64_t i = 0; i < n_calib; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const Motion m = rig.world->get_motion(rig.chassis);
        if (t >= 0.3 && t < 1.3) {
            sum_yaw += m.angular.z;
            sum_speed += m.linear.length();
            ++calib_count;
        }
    }
    const real calib_yaw = sum_yaw / static_cast<real>(calib_count);
    const real calib_speed = sum_speed / static_cast<real>(calib_count);
    const real curvature_sign = calib_yaw >= 0.0 ? real(1.0) : real(-1.0);
    const real target_curvature = std::abs(calib_yaw) / calib_speed; // 1/m
    const real target_speed = calib_speed;                          // m/s

    // --- Phase 2: closed loop - PI on curvature (steer) and speed (throttle) ---
    real steer_integral = 0.0;
    real speed_integral = 0.0;
    // Gains found by measurement (this commit) against car_sedan.json's
    // placeholder drive at this operating point: curvature error is O(1e-2)
    // 1/m and the steer channel's useful range is [-1,1], so kp_steer is
    // O(1e2); speed error is O(1) m/s and throttle's range is [0,1], so
    // kp_speed is O(1e-1). Both loops run at the 240 Hz tick rate (World's
    // control channels are sampled once per tick, PLAN.md P2).
    constexpr real kp_steer = 60.0, ki_steer = 40.0;
    constexpr real kp_speed = 0.08, ki_speed = 0.05;

    std::deque<real> yaw_hist, speed_hist, beta_hist;
    const std::size_t window_n = static_cast<std::size_t>(0.5 / dt + 0.5); // trailing 0.5 s window

    const real settle_seconds = 5.0; // well past the curvature loop's own settling time (measured)
    const std::uint64_t n_settle = static_cast<std::uint64_t>(settle_seconds / dt + 0.5);

    real prev_beta = 0.0;
    bool have_prev_beta = false;

    for (std::uint64_t i = 0; i < n_settle; ++i) {
        const Motion m = rig.world->get_motion(rig.chassis);
        const real speed = m.linear.length();
        const real curvature = speed > 1e-3 ? m.angular.z / speed : 0.0;
        const real curv_err = curvature_sign * target_curvature - curvature;
        steer_integral += curv_err * dt;
        const real steer_cmd = std::clamp(kp_steer * curv_err + ki_steer * steer_integral, real(-1.0), real(1.0));

        const real speed_err = target_speed - speed;
        speed_integral += speed_err * dt;
        const real throttle_cmd = std::clamp(kp_speed * speed_err + ki_speed * speed_integral, real(0.0), real(1.0));

        rig.world->set_control("steer", steer_cmd);
        rig.world->set_control("throttle", throttle_cmd);
        rig.world->step();

        const Motion m2 = rig.world->get_motion(rig.chassis);
        const Pose p2 = rig.world->get_pose(rig.chassis);
        const Vec3 fwd = p2.orientation.rotate(Vec3::unit_x());
        const Vec3 lat = p2.orientation.rotate(Vec3::unit_y());
        const real vx_body = dot(m2.linear, fwd);
        const real vy_body = dot(m2.linear, lat);
        const real beta = ps::math::atan2(vy_body, vx_body);
        (void)have_prev_beta;
        prev_beta = beta;
        have_prev_beta = true;

        yaw_hist.push_back(m2.angular.z);
        speed_hist.push_back(m2.linear.length());
        beta_hist.push_back(beta);
        if (yaw_hist.size() > window_n) {
            yaw_hist.pop_front();
            speed_hist.pop_front();
            beta_hist.pop_front();
        }
    }
    (void)prev_beta;

    const real yaw_mean = mean_of(yaw_hist);
    const real speed_mean = mean_of(speed_hist);
    const real beta_mean = mean_of(beta_hist);
    const real yaw_range = range_of(yaw_hist);
    const real speed_range = range_of(speed_hist);
    const real beta_range = range_of(beta_hist);

    INFO("target_curvature=" << target_curvature << " 1/m, target_speed=" << target_speed << " m/s");
    INFO("trailing window (0.5s): yaw_rate=" << yaw_mean << "+/-" << yaw_range << " rad/s, speed=" << speed_mean
                                              << "+/-" << speed_range << " m/s, beta=" << beta_mean << "+/-"
                                              << beta_range << " rad");

    // Steadiness: asserted, not assumed (I2 plan pre-step P2). Thresholds
    // are tight relative to each signal's own mean/scale, well inside a
    // genuine limit cycle's variation but loose enough for the PI loop's
    // own residual ripple at 240 Hz.
    CHECK(yaw_range < real(0.01) * std::abs(yaw_mean));
    CHECK(speed_range < real(0.01) * target_speed);
    CHECK(beta_range < real(0.01)); // rad, absolute (beta_mean can be near 0)

    // Valid now because steadiness (constant beta in particular) has just
    // been asserted above: a_y_body = v*omega_z + dv_y_body/dt, and a
    // genuinely steady turn has dv_y_body/dt ~= 0 (see this file's header
    // comment for the full identity and why the OLD open-loop version could
    // not rely on this).
    const real steady_a_lat = std::abs(speed_mean * yaw_mean);

    const real estimate = tyre_limited_lateral_g_estimate();
    std::printf("[skidpad] steady_a_lat=%.4f m/s^2 (closed-loop), estimate=%.4f m/s^2, speed=%.3f m/s, R=%.2f m\n",
                static_cast<double>(steady_a_lat), static_cast<double>(estimate), static_cast<double>(speed_mean),
                static_cast<double>(1.0 / target_curvature));
    INFO("steady_a_lat=" << steady_a_lat << " m/s^2, estimate=" << estimate << " m/s^2");
    CHECK(steady_a_lat == Catch::Approx(estimate).epsilon(0.03));
}
