// tests/unit/test_vehicle_ackermann_symmetry.cpp — unit-level regression for
// the right-lock Ackermann steering bug (owner report: "turning left and
// accelerating turns left, but turning right and accelerating turns left,
// slows down the car and eventually drives it backwards").
//
// VehicleInstance::ackermann_wheel_angle() (core/src/vehicle/vehicle_instance.cpp)
// used to compute a front wheel's Ackermann angle as atan2(dx, R - dy),
// where dx = wheel.attachment_local.x - rear_axle_x is the wheelbase and is
// ALWAYS POSITIVE for a front wheel. atan2's result is confined to (0, pi)
// whenever its first argument is positive, so it can never return a
// negative wheel angle no matter how hard right lock (negative delta_avg,
// this project's ISO 8855 convention, +y = left) is applied - a right-lock
// input produced a large POSITIVE (left-pointing) front wheel angle
// instead of a small negative one. Left lock was unaffected (denom stays
// positive there, so atan2 already happened to return the correct small
// positive angle) - which is exactly why the existing step-steer/skidpad
// validation tests, both of which only ever steer ONE direction, never
// caught this. See CLAUDE.md's "Vehicle system" section for the fix
// (atan(dx / denom) instead of atan2(dx, denom): dx > 0 always, so the
// quotient's sign already carries denom's sign and atan is odd, giving
// the correct signed small-angle result directly without atan2's
// four-quadrant ambiguity).
//
// This test checks the fix directly, independent of full vehicle dynamics
// settling: World::wheel_state(...).steer_angle for a given tick depends
// only on that tick's delta_avg (from World::set_control("steer", ...))
// and each wheel's own attachment geometry - never on suspension state or
// contact - so a single World::step() right after set_control is enough
// to read back a wheel's Ackermann angle for that steer input.

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <utility>

using namespace ps;

namespace {
// data/vehicles/car_sedan.json wheel order: FL=0, FR=1, RL=2, RR=3.
constexpr std::size_t kFL = 0;
constexpr std::size_t kFR = 1;

std::pair<real, real> front_steer_angles(ps_test::VehicleTestRig& rig, real steer_control) {
    rig.world->set_control("steer", steer_control);
    rig.world->step();
    return {rig.world->wheel_state(rig.vehicle, kFL).steer_angle, rig.world->wheel_state(rig.vehicle, kFR).steer_angle};
}

// Builds the same rig as ps_test::build_vehicle_rig (car_sedan.json's own
// geometry, same chassis box/mass/ride-height every other test in this
// file uses) but with steering.ackermann_fraction overridden in memory -
// data/vehicles/*.json is this project's real vehicle fleet (PLAN.md
// section 8), not a place for one-off per-test fixture values, and
// VehicleDesc is plain data (ps::io::load_vehicle_json returns it by
// value), so mutating the loaded desc before World::create_vehicle is the
// natural way to test a range of fractions against one vehicle's geometry.
ps_test::VehicleTestRig build_rig_with_ackermann_fraction(real fraction) {
    ps_test::VehicleTestRig rig;

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = 4;
    rig.world = std::make_unique<World>(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}}; // see build_vehicle_rig's own comment on why 1000 m
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}}; // matches every other test in this file
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    rig.chassis = rig.world->create_body(chassis);

    rig.surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    rig.world->set_surface_table(rig.surfaces);

    vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    desc.steering.ackermann_fraction = fraction;
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);

    return rig;
}
} // namespace

TEST_CASE("Anti-Ackermann (f<0) gives the outer front wheel the larger-magnitude angle", "[unit][vehicle]") {
    auto rig = build_rig_with_ackermann_fraction(-1.0);

    const auto [fl_left, fr_left] = front_steer_angles(rig, 1.0);    // full left lock: outer = FR
    const auto [fl_right, fr_right] = front_steer_angles(rig, -1.0); // full right lock, mirror input: outer = FL

    INFO("left lock:  FL=" << fl_left << " rad, FR=" << fr_left << " rad");
    INFO("right lock: FL=" << fl_right << " rad, FR=" << fr_right << " rad");

    // Every wheel still turns TOWARD the commanded direction - anti-Ackermann
    // changes which wheel gets the bigger magnitude, not the turn direction.
    CHECK(fl_left > 0.0);
    CHECK(fr_left > 0.0);
    CHECK(fl_right < 0.0);
    CHECK(fr_right < 0.0);

    // Exact mirror symmetry, same argument as the geometric-Ackermann case
    // above: the vehicle's own geometry is left/right symmetric.
    CHECK(fl_left == Catch::Approx(-fr_right).margin(1e-9));
    CHECK(fr_left == Catch::Approx(-fl_right).margin(1e-9));

    // Anti-Ackermann: the OUTER wheel (farther from the turn centre) now
    // gets the larger-magnitude angle - the reverse of the f=1 case above.
    // FR is outer at left lock, FL is outer at right lock.
    CHECK(fr_left > fl_left);
    CHECK(std::abs(fl_right) > std::abs(fr_right));
}

TEST_CASE("ackermann_fraction = 0 gives identical parallel-steer angles on both front wheels", "[unit][vehicle]") {
    auto rig = build_rig_with_ackermann_fraction(0.0);

    const auto [fl, fr] = front_steer_angles(rig, 0.7);

    INFO("FL=" << fl << " rad, FR=" << fr << " rad");
    CHECK(fl == Catch::Approx(fr).margin(1e-9));
    // Parallel steer's angle is exactly delta_avg (car_sedan.json's
    // max_wheel_angle=0.5236 rad, see ackermann_wheel_angle's f=0 case).
    CHECK(fl == Catch::Approx(0.7 * 0.5236).margin(1e-9));
}

TEST_CASE("Ackermann front wheel angles are left/right mirror-symmetric", "[unit][vehicle]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);

    const auto [fl_left, fr_left] = front_steer_angles(rig, 1.0);    // full left lock
    const auto [fl_right, fr_right] = front_steer_angles(rig, -1.0); // full right lock, mirror input

    INFO("left lock:  FL=" << fl_left << " rad, FR=" << fr_left << " rad");
    INFO("right lock: FL=" << fl_right << " rad, FR=" << fr_right << " rad");

    // Both front wheels must actually turn TOWARD the commanded direction:
    // left lock gives positive (left-pointing) angles on both wheels,
    // right lock gives negative (right-pointing) angles on both. The bug
    // this test exists to catch produced a large POSITIVE angle on both
    // wheels even at full right lock.
    CHECK(fl_left > 0.0);
    CHECK(fr_left > 0.0);
    CHECK(fl_right < 0.0);
    CHECK(fr_right < 0.0);

    // Exact mirror symmetry: the vehicle's own geometry is left/right
    // symmetric, so negating the steer input and swapping which wheel is
    // "inner"/"outer" must negate the angle exactly.
    CHECK(fl_left == Catch::Approx(-fr_right).margin(1e-9));
    CHECK(fr_left == Catch::Approx(-fl_right).margin(1e-9));

    // The inner wheel (closer to the turn centre) gets the larger-magnitude
    // angle: FL is inner at left lock, FR is inner at right lock.
    CHECK(fl_left > fr_left);
    CHECK(std::abs(fr_right) > std::abs(fl_right));
}
