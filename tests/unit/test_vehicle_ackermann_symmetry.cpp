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
} // namespace

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
