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
//
// REAL MECHANISM (found after the [!mayfail] history below turned out to
// be chasing the wrong cause): the drift was never the tyre's "elastic
// hold" being too weak - the two locked rear wheels' Fx exactly balances
// the slope force (an exact Coulomb equilibrium), and the car crept
// UPHILL, which a weak tyre spring cannot explain at all. Two numerical
// defects, both present since P3, combined to defeat that equilibrium:
// (D1) wheel_instance.cpp's rolling resistance used to scale by
// sign(vx_contact), which never settles at true standstill (vx_contact is
// a substep-sampled contact velocity essentially never bit-exact zero) -
// the sign flipped every substep, a period-2 limit cycle at 960 Hz; (D2)
// Jolt's own Body::AddRotationStep (Jolt/Physics/Body/Body.inl) silently
// drops any rotation step with |omega*dt_substep| <= 1e-6 rad, so the
// chassis kept a small, real, steady pitch rate that never integrated
// into its orientation - a "rolling without rotating" mode whose
// contact-patch velocity (v + omega x r) cancelled the tyre-visible
// translation, so no tyre force ever opposed it. See
// core/include/ps/vehicle/tyre_desc.h's rolling_resistance_v_eps comment
// and ps/backend/i_rigid_backend.h's set_integrate_sub_threshold_rotation
// comment for the two fixes; CLAUDE.md's "Vehicle system"/"Jolt backend"
// sections have the full writeup. Neither fix alone was enough (measured
// ~0.95 mm/D1 only, ~1.82 mm/D2 only, both still over the 1 mm gate) -
// only both together closed it, to 2.5e-6 m measured here, five orders of
// magnitude under the bound. The [!mayfail] history that used to follow
// this comment (I1b's contact-patch lever fix apparently regressing an
// "elastic hold" margin) was a plausible-looking but wrong diagnosis of
// the same underlying bug; removed now that the real cause is fixed.
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

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

// PHYS-033 I1b re-check (b): re-measures the SAME parking-slope creep at the
// normal 240/960 tick and at I1's own 4x-finer 960/3840 reference, after
// I1b's contact-geometry fixes. [.][diag]: prints only, not gated (this is a
// discretisation-and-holding-mechanism characterisation, not one of I1b's
// own three-defect pins) - no fix attempted here, per the task's own
// instruction ("I2 replaces the brake clamp with exact Coulomb friction
// rows... No fix expected here").
TEST_CASE("Diagnostic: parking-slope creep at 240Hz vs a 4x-finer-tick reference (re-check b)", "[.][diag][vehicle]") {
    auto run = [](real tick_hz, real substep_hz) {
        auto rig = ps_test::build_vehicle_rig_from_desc(
            io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json"), 1500.0,
            Vec3{2.0, 0.4, 0.15}, 0.6, /*slope_grade=*/0.15, 4, Vec3::unit_y(), tick_hz, substep_hz);
        rig.world->set_control("handbrake", 1.0);
        ps_test::settle(*rig.world, 2.0);
        const Vec3 p0 = rig.world->get_pose(rig.chassis).position;
        ps_test::settle(*rig.world, 30.0);
        const Vec3 p1 = rig.world->get_pose(rig.chassis).position;
        const real drift = distance(p0, p1);
        std::printf("[diag re-check b] tick=%.0f/%.0f: drift over 30s = %.6f m\n", static_cast<double>(tick_hz),
                    static_cast<double>(substep_hz), static_cast<double>(drift));
    };
    run(240.0, 960.0);
    run(960.0, 3840.0);
}
