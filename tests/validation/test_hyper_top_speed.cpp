// tests/validation/test_hyper_top_speed.cpp -- tyre rolling-resistance work (2026-10-06): the hypercar's
// flat-road WOT top speed against an independent steady-state per-gear reference.
//
// Symptom this guards (owner, via racing_game): car_hyper_n2o topped out near 370 km/h on ~1.7 MW. Cause: the
// hypercar tyres carried the passenger-car raw speed term qsy4 0.0002 * (v/16.7)^4, so crr was 0.297 at 102.8 m/s
// (30x the 0.01 base) and rolling resistance took ~890 kW. The tyre files now derive qsy3/qsy4 from a
// "rolling_resistance" anchor block (docs/data_formats/tyre.md): crr(370 km/h) ~ 1.40 x 0.01.
//
// Fixture: tests/support/fixtures/vehicle_hyper_torque.json = car_hyper.json with the torque_map engine
// engines/hyper_v8_torque.json (so an independent reference exists - the shipped engine is a simulated
// twin-turbo V8), no tank, constant CdA 0.58 and NO downforce (the reference load is exactly m*g).
//
// Reference (tests/support/powertrain_reference.h, the sedan's top-speed machinery): per gear, bisect
// F_drive(v) = F_aero(v) + crr(v)*m*g, with crr from ps_test::ref_crr -> ps::vehicle::tyre_rolling_speed_term
// (one speed law, no private copy). A gear whose drive force still exceeds the resistance at the limiter speed is
// LIMITER-limited at that speed; otherwise it is power (drag) limited at its bisection root. The car's top
// speed is the best gear. The sim is run from 0.93 and 1.07 x v_ref and the end speeds averaged, as in
// test_powertrain_accel_topspeed.cpp.

#include "powertrain_reference.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

using namespace ps;

namespace {

real front_fraction_from_settle(World& world, VehicleId vehicle) {
    const real f0 = world.wheel_telemetry(vehicle, 0).fz;
    const real f1 = world.wheel_telemetry(vehicle, 1).fz;
    const real f2 = world.wheel_telemetry(vehicle, 2).fz;
    const real f3 = world.wheel_telemetry(vehicle, 3).fz;
    const real total = f0 + f1 + f2 + f3;
    return total > real(0.0) ? (f0 + f1) / total : real(0.5);
}

struct RefTop {
    int gear = -1;       // 1-based
    real v = 0.0;        // m/s
    bool limiter = false;
};

} // namespace

TEST_CASE("Hypercar top speed: steady-state per-gear reference vs the sim", "[validation][vehicle][powertrain][drivetrain][tyre]") {
    const real mass = 1500.0;
    const Vec3 half_extents{2.0, 0.4, 0.15};
    const real origin_height = 0.5;
    const std::string path = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicle_hyper_torque.json";

    auto settle_rig = ps_test::build_vehicle_rig(path, mass, half_extents, origin_height);
    ps_test::settle(*settle_rig.world, 1.0);
    const real front_fraction = front_fraction_from_settle(*settle_rig.world, settle_rig.vehicle);
    const real fz_driven = real(0.5) * mass * real(9.81) * (real(1.0) - front_fraction); // rear axle drives

    const vehicle::VehicleDesc desc = io::load_vehicle_json(path);
    const ps_test::RefCar rc = ps_test::RefCar::build(desc);
    const real limiter_rad = rc.limiter_rpm * real(3.14159265358979323846) / real(30.0);

    auto resistance = [&](real v) {
        return real(0.5) * real(1.225) * rc.drag_area_m2 * v * v + ps_test::ref_crr(rc.tyre_desc, v) * mass * real(9.81);
    };

    RefTop best;
    for (std::size_t k = 0; k < rc.forward_ratios.size(); ++k) {
        const ps_test::GearLossParams gp = ps_test::ref_gear_loss_params(rc, k);
        // Limiter speed: omega_e = g*(1+kappa)*v/R with kappa from the drive-force fixed point at that speed.
        real v_lim = limiter_rad * rc.wheel_radius_m / gp.g;
        for (int it = 0; it < 8; ++it) {
            const real kappa = ps_test::ref_gear_drive_force(rc, gp, v_lim, fz_driven).kappa;
            v_lim = limiter_rad * rc.wheel_radius_m / (gp.g * (real(1.0) + kappa));
        }
        real v_gear = 0.0;
        bool lim = false;
        const real net_at_limiter = ps_test::ref_gear_drive_force(rc, gp, v_lim, fz_driven).f_drive - resistance(v_lim);
        if (net_at_limiter > real(0.0)) {
            v_gear = v_lim;
            lim = true;
        } else {
            const ps_test::GearTopSpeedResult gr = ps_test::solve_gear_top_speed(rc, gp, fz_driven, resistance, real(10.0), real(200.0));
            if (gr.valid) v_gear = gr.v;
        }
        INFO("gear " << (k + 1) << ": v=" << v_gear << " limiter=" << lim << " v_lim=" << v_lim);
        if (v_gear > best.v) best = RefTop{static_cast<int>(k) + 1, v_gear, lim};
    }
    REQUIRE(best.gear > 0);
    std::printf("[hyper_top_speed] reference: gear %d, %.2f m/s = %.1f km/h, %s-limited\n", best.gear, best.v, best.v * 3.6,
                best.limiter ? "limiter" : "drag");
    // The symptom: with the old raw speed term this was ~370 km/h at best.
    CHECK(best.v > real(380.0 / 3.6));

    auto run_sim = [&](real v0) {
        auto rig = ps_test::build_vehicle_rig(path, mass, half_extents, origin_height, 0.0, 4, Vec3::unit_y(), 240.0, 960.0,
                                                /*ground_half_extent=*/12000.0);
        ps_test::settle(*rig.world, 1.0);
        rig.world->backend().set_motion(rig.chassis, Motion{Vec3{v0, 0.0, 0.0}, Vec3::zero()});
        vehicle::VehicleResetOptions opts;
        opts.gear = best.gear;
        opts.clutch_locked = true;
        opts.hubs_roll_with_chassis = true;
        rig.world->reset_vehicle(rig.vehicle, opts);
        rig.world->set_control("gear_request", drivetrain::kGearRequestNone);
        rig.world->set_control("clutch", 0.0);
        rig.world->set_control("assist.auto_clutch", 0.0);
        rig.world->set_control("assist.auto_blip", 0.0);
        rig.world->set_control("assist.auto_shift", 0.0);
        rig.world->set_control("throttle", 1.0);
        const real dt = rig.world->dt();
        for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(60.0 / dt + 0.5); ++i) rig.world->step();
        return rig.world->get_motion(rig.chassis).linear.length();
    };

    const real v_end_below = run_sim(real(0.93) * best.v);
    const real v_end_above = run_sim(real(1.07) * best.v);
    const real v_sim = real(0.5) * (v_end_below + v_end_above);
    std::printf("[hyper_top_speed] sim: below %.2f above %.2f mean %.2f m/s = %.1f km/h (ref %.2f m/s)\n", v_end_below, v_end_above,
                v_sim, v_sim * 3.6, best.v);
    INFO("v_end_below=" << v_end_below << " v_end_above=" << v_end_above << " v_sim=" << v_sim << " v_ref=" << best.v);
    CHECK(std::abs(v_end_above - v_end_below) <= real(0.01) * best.v); // converged
    CHECK(std::abs(v_sim / best.v - real(1.0)) <= real(0.03));
    CHECK(v_sim > real(380.0 / 3.6));
}
