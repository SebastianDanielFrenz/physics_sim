// Validation: full-throttle standing-start acceleration is sane (PLAN.md P3
// acceptance, PHYS bug report "car accelerates at only ~0.3 m/s^2 at full
// throttle").
//
// ROOT CAUSE (see out/handover_accel.md and data/vehicles/car_sedan.json's
// own updated "source" field for the full derivation): DriveDesc has no
// separate gearbox/clutch stage (vehicle_desc.h: "no gearbox in this
// placeholder") - engine rpm is derived directly from the DRIVEN WHEELS' own
// spin speed times a single final_drive_ratio, and wheel torque is that same
// engine torque times the same single ratio. car_sedan.json originally set
// final_drive_ratio=3.7, a plausible value for a differential ALONE - with
// no gearbox multiplying it further, a standing start's wheel_rpm stays near
// zero for a long time (engine_rpm clamped to idle_rpm=900 the whole time,
// since wheel_rpm*3.7 does not reach 900 until the car is already moving at
// a few m/s), so the engine only ever delivers the torque curve's WEAKEST
// point (120 N*m at idle) instead of anything near its peak (250 N*m at
// 4000 rpm) - measured (this file's diagnostic run before the fix) ~0.72
// m/s^2 sustained, decaying further once rolling resistance and the
// idle-torque plateau are accounted for; the field-reported 0.3 m/s^2 is
// consistent with the same starved-torque mechanism plus whatever additional
// losses the live demo (adapter-side, outside this repo's core/ lane) adds
// on top. FIX (data-only, no code change): final_drive_ratio raised to 10.0
// - a plausible 1st-gear ratio (~2.7) times the original 3.7 differential -
// so idle-rpm torque is left behind quickly and the engine reaches its
// stronger mid-curve torque within a fraction of a second, matching
// data/vehicles/tractor.json's own final_drive_ratio=15.0 in already folding
// a low working-gear reduction into this single placeholder ratio.
//
// The estimate below is a point-mass model using car_sedan.json's OWN
// torque curve/final_drive_ratio/efficiency/wheel_radius (duplicated here
// deliberately, same pattern as test_vehicle_braking.cpp's
// mu_based_braking_deceleration_estimate - if the data file changes without
// updating this constant, the test's own tolerance check will flag the
// mismatch) and a flat asphalt rolling-resistance term
// (data/surfaces/surfaces.json's crr=0.013 + the tyre's own qsy1=0.010; the
// tyre's qsy2 term is 0 in this vehicle's tyre file and the qsy4*(v/v0)^4
// term is negligible below 50 km/h, so both are omitted here), ignoring
// weight transfer and tyre-force saturation - at final_drive_ratio=10.0 the
// whole 0-50 km/h run stays torque-limited, well inside the front tyres'
// peak Fx (see the wheelspin check below), so the point-mass estimate and
// the full tyre-model simulation should track closely. Measured agreement
// this session: simulated avg accel 3.207 m/s^2 vs. point-mass estimate
// 3.131 m/s^2, i.e. +2.4% - the 15% tolerance used below leaves generous
// margin for debug-vs-release/build differences while still catching a
// regression anywhere near the original bug's magnitude (measured ~0.72
// m/s^2, less than half the estimate).

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace ps;

namespace {

// Mirrors ps::vehicle::Curve1D::evaluate (vehicle_desc.h) - duplicated so
// this estimate does not need to link against core/src internals.
real curve_eval(const std::vector<real>& x, const std::vector<real>& y, real xv) {
    if (xv <= x.front()) return y.front();
    if (xv >= x.back()) return y.back();
    for (std::size_t i = 1; i < x.size(); ++i) {
        if (xv <= x[i]) {
            const real t = (xv - x[i - 1]) / (x[i] - x[i - 1]);
            return y[i - 1] + t * (y[i] - y[i - 1]);
        }
    }
    return y.back();
}

// Point-mass forward-Euler integration from a standing start to `target_v`
// (m/s), full throttle, one driven axle (2 wheels), flat ground - mirrors
// vehicle_instance.cpp's engine_rpm/torque lookup and wheel_instance.cpp's
// rolling-resistance formula (with qsy2=0 and the qsy4 term dropped, see
// this file's header comment). Returns time to reach target_v (seconds).
real point_mass_time_to_speed(real target_v, real mass, real wheel_radius, real final_drive_ratio,
                               real drivetrain_efficiency, real idle_rpm, real rev_limit_rpm,
                               const std::vector<real>& rpm_x, const std::vector<real>& torque_y,
                               real crr_total, real dt) {
    real v = 0.0, t = 0.0;
    const real g = 9.81;
    const real froll_total = crr_total * mass * g; // all 4 wheels' rolling resistance, static-weight approximation
    while (v < target_v && t < 60.0) {
        const real wheel_rpm = std::abs(v) / wheel_radius * real(60.0) / (real(2.0) * real(3.14159265358979323846));
        const real engine_rpm_raw = wheel_rpm * final_drive_ratio;
        const real engine_rpm = std::clamp(engine_rpm_raw, idle_rpm, rev_limit_rpm);
        real torque = curve_eval(rpm_x, torque_y, engine_rpm);
        if (engine_rpm_raw > rev_limit_rpm) torque = 0.0;
        const real wheel_force_total = (torque * final_drive_ratio * drivetrain_efficiency) / wheel_radius;
        const real a = (wheel_force_total - froll_total) / mass;
        v += a * dt;
        t += dt;
    }
    return t;
}

} // namespace

TEST_CASE("Full-throttle standing start on asphalt reaches a sensible acceleration", "[validation][vehicle]") {
    const real mass = 1500.0;
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("throttle", 1.0);

    const real target_speed = 50.0 / 3.6; // 50 km/h
    real time_to_target = -1.0;
    real max_abs_slip = 0.0;
    const std::uint64_t max_ticks = static_cast<std::uint64_t>(15.0 / rig.world->dt() + 0.5);
    for (std::uint64_t i = 0; i < max_ticks; ++i) {
        rig.world->step();
        const auto& w0 = rig.world->wheel_telemetry(rig.vehicle, 0); // FL, driven
        const auto& w1 = rig.world->wheel_telemetry(rig.vehicle, 1); // FR, driven
        max_abs_slip = std::max({max_abs_slip, std::abs(w0.slip_ratio), std::abs(w1.slip_ratio)});
        const real speed = rig.world->get_motion(rig.chassis).linear.length();
        if (time_to_target < 0.0 && speed >= target_speed) {
            time_to_target = (i + 1) * rig.world->dt();
            break;
        }
    }
    REQUIRE(time_to_target > 0.0); // did not reach 50 km/h within the time budget otherwise

    const real avg_accel_measured = target_speed / time_to_target;

    // car_sedan.json's own drive block (duplicated - see this file's header
    // comment) and data/surfaces/surfaces.json's asphalt crr=0.013 +
    // passenger_car_225_45r17.json's qsy1=0.010.
    const std::vector<real> rpm_x = {900, 2000, 4000, 6000, 6500};
    const std::vector<real> torque_y = {120, 220, 250, 200, 150};
    const real estimate_time = point_mass_time_to_speed(target_speed, mass, /*wheel_radius=*/0.317,
                                                          /*final_drive_ratio=*/10.0, /*drivetrain_efficiency=*/0.9,
                                                          /*idle_rpm=*/900.0, /*rev_limit_rpm=*/6500.0, rpm_x, torque_y,
                                                          /*crr_total=*/0.023, rig.world->dt());
    const real avg_accel_estimate = target_speed / estimate_time;

    INFO("time_to_50kmh=" << time_to_target << " s, avg_accel_measured=" << avg_accel_measured
                           << " m/s^2, avg_accel_estimate=" << avg_accel_estimate
                           << " m/s^2, max_abs_slip=" << max_abs_slip);
    CHECK(avg_accel_measured == Catch::Approx(avg_accel_estimate).epsilon(0.15));

    // Sanity floor independent of the point-mass estimate: the ORIGINAL bug
    // measured ~0.72 m/s^2 sustained (this file's header comment) - well
    // under half of any plausible fix. A regression back toward that would
    // still pass a badly-chosen tolerance-only check if the estimate itself
    // were wrong, so also assert directly against the bug report's own
    // "around 3 m/s^2" expectation.
    CHECK(avg_accel_measured > 2.0);

    // The driven wheels must not spin up without moving the car (PHYS bug
    // report's own explicit failure mode to check): on this high-grip
    // asphalt surface, slip ratio should stay well inside the tyre's stable
    // (pre-peak) region throughout the run, not run away toward the +-3
    // safety clamp (pacejka_tyre_model.cpp's step_slip_relaxation) the way
    // final_drive_ratio=13.0 was observed to during this fix's own
    // investigation (see data/vehicles/car_sedan.json's "source" field).
    CHECK(max_abs_slip < 0.3);
}

TEST_CASE("Full-throttle acceleration on grass/dirt is lower than on asphalt", "[validation][vehicle]") {
    const real mass = 1500.0;
    const Vec3 half_extents{2.0, 0.4, 0.15};
    const real origin_height = 0.6;
    const std::string vehicle_path = std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json";
    const real run_seconds = 2.0; // short enough to stay well below 50 km/h on every surface below

    auto speed_after_full_throttle_run = [&](const std::string& surface_name) {
        auto rig = ps_test::build_vehicle_rig_on_surface(vehicle_path, mass, half_extents, origin_height, surface_name);
        ps_test::settle(*rig.world, 1.0);
        rig.world->set_control("throttle", 1.0);
        const std::uint64_t n = static_cast<std::uint64_t>(run_seconds / rig.world->dt() + 0.5);
        for (std::uint64_t i = 0; i < n; ++i) rig.world->step();
        return rig.world->get_motion(rig.chassis).linear.length();
    };

    const real speed_asphalt = speed_after_full_throttle_run("asphalt");
    const real speed_dirt = speed_after_full_throttle_run("dirt");
    const real speed_grass = speed_after_full_throttle_run("grass");

    INFO("speed_asphalt=" << speed_asphalt << " speed_dirt=" << speed_dirt << " speed_grass=" << speed_grass << " m/s after "
                           << run_seconds << " s");
    // data/surfaces/surfaces.json: lambda_mu asphalt=1.0 > dirt=0.6 >
    // grass=0.42, and crr asphalt=0.013 < dirt=0.04 < grass=0.08 - both
    // effects point the same way (lower grip AND more rolling resistance),
    // so the ordering is unambiguous even though full_throttle on the
    // lower-grip surfaces is traction- rather than torque-limited (some
    // wheelspin is expected there, unlike the asphalt case above).
    CHECK(speed_asphalt > speed_dirt);
    CHECK(speed_dirt > speed_grass);
}
