// tests/validation/test_powertrain_accel_topspeed.cpp — PHYS-033 P4 I3 plan
// section 13.1 (test 1: 0-100 km/h within 3%) and 13.2 (test 2: top speed
// within 3%).
//
// SCOPE NOTE (test 1 deferred, not forgotten): only test 2 (top speed) is
// implemented in this commit. Test 1's own reference model (plan section
// 13's shared "RefCar" ODE) needs a full launch + 3-upshift EVENT-DRIVEN
// state machine - Neutral/Engaged(k) x {Slipping, Locked} clutch modes, a
// lock/unlock transition rule, AND an independent re-derivation of the
// gearbox's own shift-phase timing (GearboxDesc::neutral_phase_s,
// SynchroDesc's capacity ramp, t_sync = J_i*|omega_i - r_(k+1)*omega_o| /
// c_sync) to know exactly when the reference's own gear_request "completes"
// - none of which test 5/7's simpler steady-locked-gear or self-consistency
// checks needed. Given this session's own experience building even the
// SIMPLER locked-gear-only reference (tests/support/powertrain_reference.h,
// test_powertrain_engine_braking.cpp's test 5) - two genuine attempts, one
// making things worse, both documented - attempting the substantially more
// complex test 1 model within this same session's remaining budget risked
// shipping an equally unresolved (or silently wrong) reference rather than
// a working test. Deferred per the coordinator's own "attempt carefully,
// abandon/report honestly" guidance rather than rushed; see this commit's
// message and the I3 handover note for the explicit call-out. A follow-up
// session picking this up should start from RefCar (powertrain_reference.h)
// and this file's own front_static_fraction_from_settle/GearLossParams
// reuse - both are already shared infrastructure test 1 would need too.

#include "powertrain_reference.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>

using namespace ps;

namespace {

real front_static_fraction_from_settle(World& world, VehicleId vehicle) {
    const real fz_fl = world.wheel_telemetry(vehicle, 0).fz;
    const real fz_fr = world.wheel_telemetry(vehicle, 1).fz;
    const real fz_rl = world.wheel_telemetry(vehicle, 2).fz;
    const real fz_rr = world.wheel_telemetry(vehicle, 3).fz;
    const real total = fz_fl + fz_fr + fz_rl + fz_rr;
    return total > real(0.0) ? (fz_fl + fz_fr) / total : real(0.5);
}

// Plan section 13.2's own resistance formula: Sum_i Fz_i*(qsy1 + qsy4*(v/v0)^4)
// + 1/2*rho*CdA*v^2, Fz_i summed over all four wheels == mass*g (static, no
// load transfer - a steady-speed simplification the plan states directly).
real ref_resistance_force(const ps_test::RefCar& rc, real v, real mass) {
    const real g_grav = real(9.81);
    const real crr_total = rc.tyre_desc.qsy1 + rc.tyre_desc.qsy4 * std::pow(v / rc.tyre_desc.v0, real(4.0));
    const real f_roll = crr_total * mass * g_grav;
    const real f_aero = real(0.5) * real(1.225) * rc.drag_area_m2 * v * v;
    return f_roll + f_aero;
}

struct GearDriveForce {
    real f_drive = 0.0;
    real kappa = 0.0;
    real engine_rpm = 0.0;
};

// Steady-state (WOT, theta=1) drive force at speed v in gear (gp), for a
// fixed static per-wheel front load fz_f. kappa and F_drive are a genuine
// fixed point (omega_e = g*(1+kappa)*v/R depends on kappa, and F_drive
// depends on omega_e via the WOT curve) - plan section 13.2 states the
// kappa-inversion step alone ("kappa from inverting Fx(kappa, Fz_f,static)
// = F_drive/2 by bisection on [0, 0.3]"); this iterates that inversion to
// convergence against the (weakly kappa-dependent) F_drive rather than a
// single pass, since kappa can reach several percent at a driveline's
// torque peak.
GearDriveForce ref_gear_drive_force(const ps_test::RefCar& rc, const ps_test::GearLossParams& gp, real v, real fz_f) {
    real kappa = 0.0;
    real f_drive = 0.0;
    for (int outer = 0; outer < 30; ++outer) {
        const real omega_hub = (real(1.0) + kappa) * v / rc.wheel_radius_m;
        const real omega_e_rad = gp.g * omega_hub;
        const real rpm = ps_test::ref_rad_to_rpm(omega_e_rad);
        const real tau_wot = rc.wot.evaluate(rpm);
        const real new_f_drive = real(2.0) * gp.hub_torque_gain * tau_wot / rc.wheel_radius_m;

        real lo = real(0.0), hi = real(0.3);
        for (int b = 0; b < 60; ++b) {
            const real mid = real(0.5) * (lo + hi);
            const ps::vehicle::TyreSlipInput in{mid, real(0.0), fz_f, real(0.0), v, real(1.0), real(0.0)};
            const real fx = rc.tyre->evaluate(in).fx;
            if (fx < new_f_drive / real(2.0)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        const real new_kappa = real(0.5) * (lo + hi);
        f_drive = new_f_drive;
        if (std::abs(new_kappa - kappa) < real(1e-7)) {
            kappa = new_kappa;
            break;
        }
        kappa = new_kappa;
    }
    GearDriveForce r;
    r.f_drive = f_drive;
    r.kappa = kappa;
    r.engine_rpm = ps_test::ref_rad_to_rpm(gp.g * (real(1.0) + kappa) * v / rc.wheel_radius_m);
    return r;
}

struct GearTopSpeedResult {
    bool valid = false;
    real v = 0.0;
    real engine_rpm = 0.0;
};

// Bisects F_drive(v) - F_res(v) == 0 on v in [10, 90] (plan section 13.2's
// own search range) for one gear; rejects the gear if no sign change is
// found in that range, or if the root's own engine rpm is at/above the
// limiter (plan: "reject a gear if omega_e >= limiter.rpm" - such a state
// is not a stable cruise, the limiter would be cutting fuel).
GearTopSpeedResult solve_gear_top_speed(const ps_test::RefCar& rc, const ps_test::GearLossParams& gp, real mass,
                                          real fz_f_static) {
    auto net = [&](real v) { return ref_gear_drive_force(rc, gp, v, fz_f_static).f_drive - ref_resistance_force(rc, v, mass); };
    real lo = real(10.0), hi = real(90.0);
    const real f_lo = net(lo), f_hi = net(hi);
    if (f_lo <= real(0.0) || f_hi > real(0.0)) return {}; // no sign change in range: reject (see this fn's own comment)
    for (int it = 0; it < 60; ++it) {
        const real mid = real(0.5) * (lo + hi);
        if (net(mid) > real(0.0)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const real v_root = real(0.5) * (lo + hi);
    const GearDriveForce gd = ref_gear_drive_force(rc, gp, v_root, fz_f_static);
    if (gd.engine_rpm >= rc.limiter_rpm) return {};
    GearTopSpeedResult r;
    r.valid = true;
    r.v = v_root;
    r.engine_rpm = gd.engine_rpm;
    return r;
}

} // namespace

// PHYS-033 P4 I3 plan section 13.2: top speed within 3% of the
// steady-state-per-gear reference (bisection, not an integrator - no
// launch/shift transient to model, which is exactly why this test is
// tractable where test 1 is not, see this file's own header comment).
TEST_CASE("Top speed is within 3% of the steady-state-per-gear reference", "[validation][vehicle][powertrain][drivetrain][long]") {
    const real mass = 1500.0;
    const real chassis_origin_height = 0.6;
    auto settle_rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                                   Vec3{2.0, 0.4, 0.15}, chassis_origin_height);
    ps_test::settle(*settle_rig.world, 1.0);
    const real front_static_fraction = front_static_fraction_from_settle(*settle_rig.world, settle_rig.vehicle);

    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    const ps_test::RefCar rc = ps_test::RefCar::build(desc);
    const real fz_f_static = real(0.5) * mass * real(9.81) * front_static_fraction;

    // --- Reference: max v_ref over every forward gear -----------------------
    int best_gear_zero_based = -1;
    real v_ref = 0.0;
    real v_ref_rpm = 0.0;
    for (std::size_t k = 0; k < rc.forward_ratios.size(); ++k) {
        const ps_test::GearLossParams gp = ps_test::ref_gear_loss_params(rc, k);
        const GearTopSpeedResult gr = solve_gear_top_speed(rc, gp, mass, fz_f_static);
        INFO("gear " << (k + 1) << ": valid=" << gr.valid << " v=" << gr.v << " rpm=" << gr.engine_rpm);
        if (gr.valid && gr.v > v_ref) {
            v_ref = gr.v;
            v_ref_rpm = gr.engine_rpm;
            best_gear_zero_based = static_cast<int>(k);
        }
    }
    REQUIRE(best_gear_zero_based >= 0); // setup precondition: some gear must give a valid steady cruise
    INFO("reference: best_gear=" << (best_gear_zero_based + 1) << " v_ref=" << v_ref << " (" << (v_ref * 3.6)
                                  << " km/h) rpm=" << v_ref_rpm);

    // --- Sim: two runs bracketing v_ref, ground half-extent 4000 (plan) -----
    auto run_sim = [&](real v0) {
        auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                               Vec3{2.0, 0.4, 0.15}, chassis_origin_height, 0.0, 4, Vec3::unit_y(), 240.0,
                                               960.0, /*ground_half_extent=*/4000.0);
        ps_test::settle(*rig.world, 1.0);
        rig.world->backend().set_motion(rig.chassis, Motion{Vec3{v0, 0.0, 0.0}, Vec3::zero()});
        vehicle::VehicleResetOptions opts;
        opts.gear = best_gear_zero_based + 1; // 1-based (World::powertrain_state's own convention)
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
        for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(70.0 / dt + 0.5); ++i) rig.world->step();
        return rig.world->get_motion(rig.chassis).linear.length();
    };

    const real v0_below = real(0.93) * v_ref;
    const real v0_above = real(1.07) * v_ref;
    const real v_end_below = run_sim(v0_below);
    const real v_end_above = run_sim(v0_above);
    const real v_sim = real(0.5) * (v_end_below + v_end_above);

    INFO("v0_below=" << v0_below << " v_end_below=" << v_end_below << " v0_above=" << v0_above
                      << " v_end_above=" << v_end_above << " v_sim=" << v_sim << " v_ref=" << v_ref);
    CHECK(std::abs(v_end_above - v_end_below) <= real(0.01) * v_ref); // converged
    CHECK(std::abs(v_sim / v_ref - real(1.0)) <= real(0.03));
}
