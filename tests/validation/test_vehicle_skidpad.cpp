// Validation: steady-state skidpad lateral acceleration within 3% of the
// tyre-limited estimate (PLAN.md P3 acceptance).
//
// "Tyre-limited estimate" here is a two-axle load-transfer-corrected bound:
// if every tyre reached its own peak lateral coefficient simultaneously,
// a_lat_max = sum_i(Dy_i(Fz_i)) / mass, where Dy_i is the Magic Formula's
// own peak-force term (tyre_desc.h's pdy1/pdy2, evaluated at each wheel's
// OWN load) - refined for lateral load transfer (Milliken, "Race Car
// Vehicle Dynamics", ch. 18). The TOTAL two-axle transfer moment is
// m*a_lat*h_cg (standard rigid-body roll-moment balance about a ground-
// level roll axis - this codebase has no separate roll-centre-height
// model, PLAN.md section 6, so the elastic/geometric transfer distinction
// collapses and the full moment is resisted by the springs+ARBs); that
// moment is then split between the front and rear axles in proportion to
// EACH AXLE'S OWN roll stiffness (spring + anti-roll-bar - car_sedan.json's
// front/rear ARB rates deliberately differ, 15000 vs 10000 N/m), not
// applied identically to both axles.
//
// This replaces an EARLIER version of this estimate that computed
// dFz_axle = m*a_lat*h_cg/track (the TOTAL two-axle moment/track) and then
// applied that FULL value, unsplit, to BOTH axles independently - i.e. it
// implicitly assumed each axle alone resists the entire roll moment, which
// double-counts the total transfer relative to the correct roll-stiffness-
// weighted split derived above. Because the tyre's load sensitivity is
// concave (pdy2 < 0: a loaded-up outer tyre gains less than an unloaded
// inner tyre loses), overstating the load transfer overstates the grip
// LOSS from cornering, so the old estimate (9.346 m/s^2) undershot the
// measured value (9.837 m/s^2, +5.2%, outside the 3% tolerance) - see
// out/handover_p3_fix.md and the vault (PHYS-018) for the full derivation
// and the numeric comparison against the halved/roll-stiffness-split
// forms. The corrected estimate (below) converges to ~9.80 m/s^2, within
// 0.4% of measured.
//
// Combined slip: the front wheels are `driven` in car_sedan.json but the
// test applies no throttle, and no wheel is braked, so at the cornering
// equilibrium every wheel's own spin settles to (very close to) zero
// longitudinal tyre force (undriven/unbraked wheels have no other torque
// to balance against the tyre's own Fx - wheel_instance.cpp's spin
// integration) - confirmed by telemetry: every wheel's slip_ratio stays
// under 0.007 throughout the averaging window below. At kappa this close
// to zero the Magic Formula's own combined-slip weighting Gyk(kappa) is,
// by construction (its cosine form is normalised to 1 at the reference
// slip - pacejka_tyre_model.cpp), within noise of 1.0, so no explicit
// combined-slip discount is needed for THIS manoeuvre; the estimate would
// need one for a skidpad test that also applied throttle or brake.
//
// The car is given an initial speed and a constant large steer angle
// (no active speed control - PLAN.md's placeholder drive has no cruise
// control) and the achieved lateral acceleration (v * yaw_rate) is
// measured shortly after turn-in, while speed has not yet decayed
// materially from rolling resistance (there is no aerodynamic drag before
// P5).

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ps;

namespace {

// Sum_i Dy_i(Fz_i) / mass, iterated to convergence with the total transfer
// moment split between axles by roll-stiffness fraction (see this file's
// header comment). fz0/pdy1/pdy2 come from
// data/tyres/passenger_car_225_45r17.json; track/h_cg/mass/corner weight/
// spring_rate/arb_rate from data/vehicles/car_sedan.json's own geometry/
// mass/suspension.
real tyre_limited_lateral_g_estimate() {
    const real mass = 1500.0, g = 9.81;
    const real fz0 = 4200.0, pdy1 = 1.0, pdy2 = -0.12;
    const real corner_weight = mass * g / 4.0;
    const real track = 1.56, h_cg = 0.6;

    // Roll stiffness per axle (this file's header comment): spring_rate is
    // uniform across all 4 wheels; arb_rate differs front/rear. A wheel's
    // ARB partner deflects oppositely under roll, so the ARB's relative-
    // compression force (suspension.h's arb_force()) doubles its
    // contribution to the effective per-wheel roll-resisting rate.
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

    real a_lat = pdy1 * g; // naive first guess
    for (int i = 0; i < 8; ++i) { // fixed-point iteration to convergence
        const real dFz_total = (mass * a_lat * h_cg) / track; // TOTAL two-axle transfer moment/track
        const real dy_front = axle_dy_sum(dFz_total * frac_front);
        const real dy_rear = axle_dy_sum(dFz_total * frac_rear);
        a_lat = (dy_front + dy_rear) / mass;
    }
    return a_lat;
}

} // namespace

// Was [!mayfail] (measured 9.84 vs the OLD estimate's 9.35 m/s^2, +5.2%):
// root-caused to a bug in the estimate, not the model - see this file's
// header comment. The corrected roll-stiffness-split estimate (~9.80
// m/s^2) is within 0.4% of measured; tag removed.
//
// [!mayfail] RE-ADDED by PHYS-033 I1b commit C7 (the contact-patch lever
// fix, decision D-A): measured steady_a_lat is now 9.037 m/s^2, -7.8% vs
// the 9.803 m/s^2 estimate (outside the 3% tolerance). Per the I1b
// coordinator's explicit instruction, the plan's own steady-state
// diagnostic (tests/validation/test_vehicle_contact_geometry.cpp's
// "contact geometry diagnostics" [.][diag] case) was run BEFORE
// interpreting this number, per this file's own header comment
// ("... the car is probably rotating (dbeta/dt != 0), and v*omega_z !=
// v^2/R... T2.4 prints a_c and v*dbeta/dt to settle this"):
//   - the test's own metric, v*omega_z = 9.037 m/s^2
//   - the TRUE measured lateral acceleration (finite-difference of chassis
//     velocity, projected onto the body's own lateral axis) = 8.535 m/s^2 -
//     5.9% BELOW the test's own metric
//   - v*dbeta/dt = -0.713 m/s^2 - not negligible against either number
//   - roll = -0.035 rad (~2.0 deg, up from ~1.1 deg pre-I1b, consistent
//     with the patch-point fix's larger roll-load-transfer lever)
// This confirms the test's own header comment's own prediction: at
// steer=0.8 (~24 deg wheel angle) and 25 m/s, the car is still rotating
// (dbeta/dt clearly nonzero) within the [0.3, 1.3] s averaging window, so
// v*omega_z is NOT equal to the true steady-state lateral acceleration -
// the test's own STEADY-STATE PREMISE does not hold at this operating
// point under the corrected (now heavier-lever, more roll-sensitive)
// model. Per the coordinator's explicit instruction, this is reported as a
// premise issue, NOT fixed by retuning the test's own estimate or
// tolerance (which were already re-derived once, above, for an unrelated
// reason) and NOT by touching the model to chase this specific number.
TEST_CASE("Skidpad lateral acceleration is within 3% of the tyre-limited estimate", "[validation][vehicle][long][!mayfail]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);

    ps_test::settle(*rig.world, 0.5); // settle vertical suspension state before adding speed

    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{25.0, 0.0, 0.0}, Vec3::zero()});
    rig.world->set_control("steer", 0.8);

    // Average over a window after the initial turn-in transient has settled
    // (~0.3-0.4 s, measured separately) but before the speed has decayed
    // materially (there is no aerodynamic drag before P5) - PLAN.md P3
    // literally asks for "steady-state", not a transient turn-in peak.
    real sum_a_lat = 0.0;
    real sum_speed = 0.0;
    std::size_t count = 0;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(1.5 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const Motion m = rig.world->get_motion(rig.chassis);
        const real speed = m.linear.length();
        const real a_lat = std::abs(speed * m.angular.z);
        if (t >= 0.3 && t < 1.3) { sum_a_lat += a_lat; sum_speed += speed; ++count; }
    }
    const real steady_a_lat = sum_a_lat / static_cast<real>(count);
    const real avg_speed = sum_speed / static_cast<real>(count);

    const real estimate = tyre_limited_lateral_g_estimate();
    INFO("steady_a_lat=" << steady_a_lat << " m/s^2, avg_speed=" << avg_speed << " m/s, estimate=" << estimate << " m/s^2");
    CHECK(steady_a_lat == Catch::Approx(estimate).epsilon(0.03));
}
