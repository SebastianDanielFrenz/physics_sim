// Validation: a car parked (handbrake on) on a 15% slope does not creep,
// < 1 mm over 30 s (PLAN.md P3 acceptance).
//
// This is exactly the case PLAN.md 5.1 flags as the known Magic-Formula
// pitfall ("slip definitions blow up near standstill") - see
// out/handover_p3.md decision 2 for the velocity-forced relaxation ODE
// that replaces a naive slip-ratio-target lag specifically so this test
// can pass: at true standstill the tyre's longitudinal deflection state
// holds whatever value balances the slope's gravity component instead of
// decaying back toward zero.
//
// [long]: settles for 2 s then watches for 30 s of simulated time (a
// release build cost, not a debug/sanitizer one - CLAUDE.md's "Test tags").

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ps;

TEST_CASE("Car on a 15% slope with the handbrake on does not creep", "[validation][vehicle][long]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6, /*slope_grade=*/0.15);

    rig.world->set_control("handbrake", 1.0);
    ps_test::settle(*rig.world, 2.0); // let the suspension settle onto the slope with the handbrake already on

    const Vec3 p0 = rig.world->get_pose(rig.chassis).position;
    ps_test::settle(*rig.world, 30.0);
    const Vec3 p1 = rig.world->get_pose(rig.chassis).position;

    const real drift = distance(p0, p1);
    INFO("drift = " << drift << " m");
    CHECK(drift < 0.001); // PLAN.md P3 acceptance: < 1 mm over 30 s
}
