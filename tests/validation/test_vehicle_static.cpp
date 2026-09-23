// Validation: static corner weights match a hand calculation (PLAN.md P3
// acceptance).
//
// data/vehicles/car_sedan.json places the chassis COM at the geometric
// centre of a symmetric 4-wheel layout (out/handover_p3.md's vehicle-data
// note), so the hand calculation is simply mass*g/4 per corner. The wheel
// vertical position is measured kinematically each substep (a cylinder
// shape cast, out/handover_p3.md decision 1), so this also exercises that
// the suspension settles to the load-bearing equilibrium the spring rates
// were chosen for (data/vehicles/car_sedan.json's own header comment: x_eq
// = corner_weight / spring_rate = 0.06 m by construction).

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using Catch::Approx;
using namespace ps;

TEST_CASE("Car static corner weights match mass*g/4", "[validation][vehicle]") {
    const real mass = 1500.0;
    const real g = 9.81;
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);

    ps_test::settle(*rig.world, 4.0); // let the suspension settle to equilibrium

    const real expected_corner_weight = mass * g / 4.0;
    REQUIRE(rig.world->vehicle_wheel_count(rig.vehicle) == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& t = rig.world->wheel_telemetry(rig.vehicle, i);
        CHECK(t.fz == Approx(expected_corner_weight).epsilon(0.02));
        // Spring rates in car_sedan.json were chosen for x_eq = 0.06 m at
        // exactly this corner weight (its own header comment) - a second,
        // independent check that the kinematic contact + suspension force
        // pipeline is behaving as designed, not just that Fz happens to
        // land near the target by cancellation of two bugs.
        CHECK(t.compression == Approx(0.06).epsilon(0.05));
    }

    // Chassis should be resting, not still falling/bouncing: near-zero
    // vertical velocity after settling.
    const Motion m = rig.world->get_motion(rig.chassis);
    CHECK(std::abs(m.linear.z) < 0.01);
}
