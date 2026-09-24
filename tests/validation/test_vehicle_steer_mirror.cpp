// Validation: full-throttle steering response is left/right mirror-
// symmetric (owner-reported bug, see tests/unit/test_vehicle_ackermann_symmetry.cpp's
// header comment for the root cause - VehicleInstance::ackermann_wheel_angle()'s
// atan2(dx, denom) with an always-positive dx could never return a
// negative wheel angle, so full right lock steered the front wheels
// hard-LEFT instead, yawing the car left, killing forward speed and
// eventually driving it backwards).
//
// test_vehicle_step_steer.cpp and test_vehicle_skidpad.cpp both only ever
// steer ONE direction, so neither caught this. This test drives the same
// scenario (full throttle from an already-settled rest, mirroring the
// owner's live-demo report) with a symmetric pair of steer inputs and
// checks that the two runs are, in fact, mirror images: yaw rate opposite
// in sign but equal in magnitude, forward speed gained equally (not
// "gains speed turning left, loses it and eventually reverses turning
// right").
//
// [long]: full-throttle-from-settled-rest needs the suspension settled
// first (build_vehicle_rig's own settle() call) plus ~2 s of simulated
// driving per direction - same order of cost as test_vehicle_step_steer.cpp
// and test_vehicle_skidpad.cpp, both already [long] (CLAUDE.md "Test tags").

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ps;

namespace {

struct SteerRunResult {
    real yaw_rate;     // rad/s, body angular.z, averaged over the trailing window
    real forward_speed; // m/s, chassis-local +x velocity, averaged over the trailing window
};

// Drives full-throttle with a constant steer input from an already-settled
// rest and reports the trailing-window-averaged yaw rate / forward speed.
SteerRunResult drive_full_throttle_with_steer(real steer_control) {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 0.5); // suspension/tyre contact settled before any control input, like
                                       // test_vehicle_accel_standing_start.cpp's well-behaved (not the
                                       // documented unsettled-launch caveat) case.

    rig.world->set_control("gear_request", 1.0); // engage 1st (auto-clutch is on by default, plan section 8)
    rig.world->set_control("steer", steer_control);
    rig.world->set_control("throttle", 1.0);

    real sum_yaw_rate = 0.0, sum_fwd_speed = 0.0;
    std::size_t count = 0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(2.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const Motion m = rig.world->get_motion(rig.chassis);
        const Pose p = rig.world->get_pose(rig.chassis);
        REQUIRE(std::isfinite(m.angular.z));
        REQUIRE(std::isfinite(m.linear.x));
        if (t >= 1.5 && t < 2.0) { // trailing window, after the initial turn-in transient
            const real fwd_speed = p.orientation.inverse().rotate(m.linear).x;
            sum_yaw_rate += m.angular.z;
            sum_fwd_speed += fwd_speed;
            ++count;
        }
    }
    return {sum_yaw_rate / static_cast<real>(count), sum_fwd_speed / static_cast<real>(count)};
}

} // namespace

TEST_CASE("Full-throttle steering yaws and accelerates symmetrically left vs. right", "[validation][vehicle][long]") {
    const SteerRunResult left = drive_full_throttle_with_steer(0.5);
    const SteerRunResult right = drive_full_throttle_with_steer(-0.5);

    INFO("left:  yaw_rate=" << left.yaw_rate << " rad/s, forward_speed=" << left.forward_speed << " m/s");
    INFO("right: yaw_rate=" << right.yaw_rate << " rad/s, forward_speed=" << right.forward_speed << " m/s");

    // Direction: left steer yaws left (positive), right steer yaws right
    // (negative) - the bug made BOTH inputs yaw left.
    CHECK(left.yaw_rate > 0.3);
    CHECK(right.yaw_rate < -0.3);

    // Both runs must still be moving forward and accelerating from rest -
    // the bug's right-lock case lost forward speed and, given enough time,
    // reversed.
    CHECK(left.forward_speed > 2.0);
    CHECK(right.forward_speed > 2.0);

    // Mirror symmetry: the car's own geometry/mass is left/right symmetric,
    // so the two runs' magnitudes should match closely.
    CHECK(left.yaw_rate == Catch::Approx(-right.yaw_rate).epsilon(0.05));
    CHECK(left.forward_speed == Catch::Approx(right.forward_speed).epsilon(0.05));
}
