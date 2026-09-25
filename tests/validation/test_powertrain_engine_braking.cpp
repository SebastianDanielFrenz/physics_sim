// tests/validation/test_powertrain_engine_braking.cpp — PHYS-033 P4 I3 plan
// sections 13.4 (test 5: engine braking within 3%) and 13.5 (test 6: stall
// works), both in this one file per the plan's own grouping.

#include "powertrain_reference.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>

using namespace ps;

namespace {
constexpr std::size_t kFrontLeft = 0, kFrontRight = 1; // car_sedan.json wheel order (FL, FR driven; RL, RR undriven)

real front_static_fraction_from_settle(ps::World& world, VehicleId vehicle) {
    const real fz_fl = world.wheel_telemetry(vehicle, kFrontLeft).fz;
    const real fz_fr = world.wheel_telemetry(vehicle, kFrontRight).fz;
    const real fz_rl = world.wheel_telemetry(vehicle, 2).fz;
    const real fz_rr = world.wheel_telemetry(vehicle, 3).fz;
    const real total = fz_fl + fz_fr + fz_rl + fz_rr;
    return total > real(0.0) ? (fz_fl + fz_fr) / total : real(0.5);
}
} // namespace

// PHYS-033 P4 I3 plan section 13.4: 4th gear, engine braking from 100 km/h,
// no throttle/clutch/assists. DFCO (fuel cut) should be active throughout
// (rpm well above dfco_enable_rpm), so the engine contributes only its
// motoring/friction curve as a retarding torque - ps_test::RefCar's
// ref_locked_gear_step (tests/support/powertrain_reference.h) models exactly
// this "Engaged(4), Locked, theta=0" case, starting from the SIM's own
// measured v/omega at t=0.5s (matching the plan's own reference-seeding
// rule) rather than an independently-integrated launch, so the comparison
// isolates the DECELERATION physics from any launch-transient mismatch.
//
// Measured: decel_sim=-0.7154 vs decel_ref=-0.7151 m/s^2 (0.05%).
//
// History: this case was [!mayfail] at 5.63% (decel_ref=-0.6773). The
// cause was the REFERENCE, not the sim: RefCar applied the drive-direction
// mesh efficiency (T_out = r*eta*T_m) on overrun too, which shrinks the
// engine-braking torque at the wheels, when on overrun the loss adds to it
// (T_out = r*(2-eta)*T_m, T_m < 0). The "Engine braking in 4th: term
// breakdown diagnostic" case below attributes every term at t=3 s (3360
// rpm, 25.56 m/s):
//   chassis:  m*a = -1071.97 N vs Fx_front -649.37 + Fx_rear +12.87
//             - F_roll 163.30 - F_aero 272.11 = -1071.91 N (closes);
//             rotor reaction: every rotor axis is lateral (+-y), so it
//             only adds pitch torque, zero longitudinal force;
//   engine:   DFCO active the whole window, throttle_eff = 0 (idle PI off
//             under DFCO), torque = M(3360 rpm) = -47.52 N*m, i.e. the
//             motoring curve at the right rpm;
//   driveline: |clutch lambda| = 45.95 = |M| - J_e*|omega_e'|, driveline
//             -> engine (T_c < 0); hub torque 209.9 N*m = 1.0521 x the
//             lossless demand = 1/(eta_k*eta_fd) = 1.0520 (overrun branch);
//   ledger:   d(rotor_ke)/dt -1358.9 W = engine_out -11443.5 - wheel_out
//             (-16270.5) - engine_friction 5278.2 - gear 827.8 - tyre 80.0
//             - integrator 0.02 W (closes).
// Engine force at the wheels at this rpm: drive branch 624.8 N (old
// RefCar) vs overrun 690.6 N: 65.7 N, / m_eff ~1575 kg = 0.042 m/s^2, the
// whole 0.038 m/s^2 gap (the inertia terms move 0.004 the other way).
// Aero, rolling resistance and tyre slip loss agree between sim and
// reference (neutral control case: sim 0.303 m/s^2 vs hand 0.301 m/s^2).
// A 2nd, earlier attempt (rolling resistance coupled into the hub) chased a
// term that was never wrong and was reverted.
TEST_CASE("Engine braking in 4th decelerates within 3% of the reference model", "[validation][vehicle][powertrain][drivetrain]") {
    const real mass = 1500.0;
    const real chassis_origin_height = 0.6; // == h_cg (mass at the chassis body's own origin)
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, chassis_origin_height, 0.0, 4);
    ps_test::settle(*rig.world, 1.0);
    const real front_static_fraction = front_static_fraction_from_settle(*rig.world, rig.vehicle);

    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{27.78, 0.0, 0.0}, Vec3::zero()});
    vehicle::VehicleResetOptions opts;
    opts.gear = 4;
    opts.clutch_locked = true;
    opts.hubs_roll_with_chassis = true;
    rig.world->reset_vehicle(rig.vehicle, opts);

    rig.world->set_control("throttle", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);

    const real dt = rig.world->dt();
    real v_at_0p5 = 0.0, omega_at_0p5 = 0.0;
    real v_at_5p5 = 0.0;
    bool checked_dfco_precondition = false;
    real rpm_at_0p1 = 0.0;
    const std::uint64_t n = static_cast<std::uint64_t>(6.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i + 1) * dt;
        if (!checked_dfco_precondition && t >= 0.1) {
            checked_dfco_precondition = true;
            rpm_at_0p1 = rig.world->powertrain_state(rig.vehicle).engine_rpm;
        }
        if (std::abs(t - 0.5) < dt * 0.5) {
            v_at_0p5 = rig.world->get_motion(rig.chassis).linear.length();
            omega_at_0p5 = rig.world->wheel_omega(rig.vehicle, kFrontLeft);
        }
        if (std::abs(t - 5.5) < dt * 0.5) {
            v_at_5p5 = rig.world->get_motion(rig.chassis).linear.length();
        }
    }

    const auto& pt = rig.world->powertrain_state(rig.vehicle);
    INFO("rpm_at_0.1s=" << rpm_at_0p1 << " dfco_active(end)=" << (!pt.engines.empty() && pt.engines[0].dfco_active));
    // Precondition (plan section 13.4): DFCO active after 0.1s, rpm 3500-3800
    // at that point. A miss here is a setup problem (S5) - report, don't
    // silently adjust the script.
    REQUIRE(rpm_at_0p1 >= 3500.0);
    REQUIRE(rpm_at_0p1 <= 3800.0);
    REQUIRE(!pt.engines.empty());
    REQUIRE(pt.engines[0].dfco_active);

    const real delta_v_sim = v_at_5p5 - v_at_0p5; // negative (decelerating)

    // Reference: ps_test::RefCar built from car_sedan.json's own VehicleDesc
    // (never hand-duplicated constants - tests/support/powertrain_reference.h),
    // seeded from the SIM's own v/omega at t=0.5s, integrated at h_ref=1/9600s
    // for 5s (plan section 13's own h_ref).
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    const ps_test::RefCar rc = ps_test::RefCar::build(desc);
    const ps_test::GearLossParams gp = ps_test::ref_gear_loss_params(rc, /*gear_index_zero_based=*/3); // 4th gear
    real wheelbase_m = 0.0;
    {
        real x_front = 0.0, x_rear = 0.0;
        for (const auto& w : desc.wheels) {
            if (w.is_front) x_front = w.attachment_local.x;
            else x_rear = w.attachment_local.x;
        }
        wheelbase_m = x_front - x_rear;
    }
    ps_test::RefLockedGearState state;
    state.v = v_at_0p5;
    state.omega_hub = omega_at_0p5;
    const real h_ref = 1.0 / 9600.0;
    const std::uint64_t n_ref = static_cast<std::uint64_t>(5.0 / h_ref + 0.5);
    for (std::uint64_t i = 0; i < n_ref; ++i) {
        ps_test::ref_locked_gear_step(state, rc, gp, /*theta=*/0.0, mass, chassis_origin_height, wheelbase_m,
                                       front_static_fraction, h_ref);
    }
    const real delta_v_ref = state.v - v_at_0p5;

    INFO("v_at_0.5s=" << v_at_0p5 << " v_at_5.5s=" << v_at_5p5 << " delta_v_sim=" << delta_v_sim
                       << " delta_v_ref=" << delta_v_ref << " decel_sim=" << (delta_v_sim / 5.0)
                       << " decel_ref=" << (delta_v_ref / 5.0) << " m/s^2");
    REQUIRE(delta_v_ref < 0.0); // reference itself must decelerate, or the ratio gate below is meaningless
    CHECK(std::abs(delta_v_sim / delta_v_ref - 1.0) <= 0.03);

    // Control case (plan section 13.4): the same run in NEUTRAL (engine
    // disconnected) must decelerate LESS than 60% of the in-gear case,
    // proving the engine-braking path is real (aero+rolling resistance
    // alone is much weaker than engine braking at this speed/gear).
    auto rig_n = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                             Vec3{2.0, 0.4, 0.15}, chassis_origin_height, 0.0, 4);
    ps_test::settle(*rig_n.world, 1.0);
    rig_n.world->backend().set_motion(rig_n.chassis, Motion{Vec3{27.78, 0.0, 0.0}, Vec3::zero()});
    vehicle::VehicleResetOptions opts_n;
    opts_n.gear = 0; // Neutral
    opts_n.hubs_roll_with_chassis = true;
    rig_n.world->reset_vehicle(rig_n.vehicle, opts_n);
    rig_n.world->set_control("throttle", 0.0);
    rig_n.world->set_control("clutch", 1.0); // pressed/disengaged, consistent with N
    rig_n.world->set_control("assist.auto_clutch", 0.0);
    real v_n_at_0p5 = 0.0, v_n_at_5p5 = 0.0;
    for (std::uint64_t i = 0; i < n; ++i) {
        rig_n.world->step();
        const real t = static_cast<real>(i + 1) * dt;
        if (std::abs(t - 0.5) < dt * 0.5) v_n_at_0p5 = rig_n.world->get_motion(rig_n.chassis).linear.length();
        if (std::abs(t - 5.5) < dt * 0.5) v_n_at_5p5 = rig_n.world->get_motion(rig_n.chassis).linear.length();
    }
    const real delta_v_n = v_n_at_5p5 - v_n_at_0p5;
    INFO("delta_v_neutral=" << delta_v_n << " (expected magnitude < 60% of in-gear " << delta_v_sim << ")");
    CHECK(std::abs(delta_v_n) < 0.6 * std::abs(delta_v_sim));
}

// Diagnostic (not run by default): per-term force/torque/energy breakdown of
// test 5's operating point, sim vs. reference, over the window [2.9, 3.1] s.
TEST_CASE("Engine braking in 4th: term breakdown diagnostic", "[.][diag][validation][vehicle][powertrain][drivetrain]") {
    const real mass = 1500.0;
    const real h_cg = 0.6;
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, h_cg, 0.0, 4);
    ps_test::settle(*rig.world, 1.0);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{27.78, 0.0, 0.0}, Vec3::zero()});
    vehicle::VehicleResetOptions opts;
    opts.gear = 4;
    opts.clutch_locked = true;
    opts.hubs_roll_with_chassis = true;
    rig.world->reset_vehicle(rig.vehicle, opts);
    rig.world->set_control("throttle", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);

    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    const ps_test::RefCar rc = ps_test::RefCar::build(desc);
    const real eta_k = rc.forward_efficiency[3], eta_fd = rc.diff_efficiency;
    const real g = rc.forward_ratios[3] * rc.diff_ratio;
    const real R = rc.wheel_radius_m;

    const real dt = rig.world->dt();
    const std::uint64_t i0 = static_cast<std::uint64_t>(2.9 / dt + 0.5), i1 = static_cast<std::uint64_t>(3.1 / dt + 0.5);
    real v0 = 0, v1 = 0;
    drivetrain::PowertrainEnergy e0{}, e1{};
    real s_rpm = 0, s_clutch = 0, s_gb = 0, s_diff = 0, s_hub = 0, s_fx_f = 0, s_fx_r = 0, s_fz = 0, s_fz_f = 0, s_v = 0,
         s_kappa = 0, s_thr = 0;
    int n = 0;
    bool any_non_dfco = false;
    for (std::uint64_t i = 1; i <= i1; ++i) {
        rig.world->step();
        if (i == i0) {
            v0 = rig.world->get_motion(rig.chassis).linear.x;
            e0 = rig.world->powertrain_energy(rig.vehicle);
        }
        if (i > i0) {
            const auto& pt = rig.world->powertrain_state(rig.vehicle);
            const auto dbg = rig.world->powertrain_debug(rig.vehicle);
            s_rpm += pt.engine_rpm;
            s_clutch += dbg.clutch_torque;
            s_gb += dbg.gearbox_mesh_torque;
            s_diff += dbg.diff_mesh_torque;
            s_hub += dbg.hub_shaft_torque[0] + dbg.hub_shaft_torque[1];
            s_thr += dbg.throttle_eff;
            if (pt.engines.empty() || !pt.engines[0].dfco_active) any_non_dfco = true;
            real fz = 0;
            for (std::size_t w = 0; w < 4; ++w) fz += rig.world->wheel_telemetry(rig.vehicle, w).fz;
            s_fz += fz;
            s_fz_f += rig.world->wheel_telemetry(rig.vehicle, 0).fz + rig.world->wheel_telemetry(rig.vehicle, 1).fz;
            s_fx_f += rig.world->wheel_telemetry(rig.vehicle, 0).fx + rig.world->wheel_telemetry(rig.vehicle, 1).fx;
            s_fx_r += rig.world->wheel_telemetry(rig.vehicle, 2).fx + rig.world->wheel_telemetry(rig.vehicle, 3).fx;
            s_kappa += rig.world->wheel_telemetry(rig.vehicle, 0).slip_ratio;
            s_v += rig.world->get_motion(rig.chassis).linear.x;
            ++n;
        }
    }
    v1 = rig.world->get_motion(rig.chassis).linear.x;
    e1 = rig.world->powertrain_energy(rig.vehicle);
    const real T = static_cast<real>(i1 - i0) * dt;
    const real a = (v1 - v0) / T;
    const real inv = 1.0 / n;
    const real rpm = s_rpm * inv, v = s_v * inv, fz = s_fz * inv, fx_f = s_fx_f * inv, fx_r = s_fx_r * inv;
    const real tau_m = rc.motoring.evaluate(rpm);
    const real f_aero = 0.5 * 1.225 * rc.drag_area_m2 * v * v;
    const real crr = rc.tyre_desc.qsy1 + rc.tyre_desc.qsy4 * std::pow(v / rc.tyre_desc.v0, 4.0);
    const real f_roll = crr * fz;
    const real hub_sum = s_hub * inv;
    std::printf("--- engine braking breakdown, window [2.9,3.1]s ---\n");
    std::printf("v=%.4f rpm=%.1f a_chassis=%.5f m/s^2 throttle_eff=%.4f dfco_whole_window=%d kappa_FL=%.5f\n", v, rpm, a,
                s_thr * inv, any_non_dfco ? 0 : 1, s_kappa * inv);
    std::printf("engine M(rpm)=%.3f N*m  clutch_lambda=%.3f  gearbox_mesh=%.3f  diff_mesh=%.3f  hub_shaft_sum=%.3f N*m\n", tau_m,
                s_clutch * inv, s_gb * inv, s_diff * inv, hub_sum);
    std::printf("hub_sum/(g*M) = %.5f   drive-branch eta_k*eta_fd = %.5f   overrun 1/(eta_k*eta_fd) = %.5f\n",
                hub_sum / (g * tau_m), eta_k * eta_fd, 1.0 / (eta_k * eta_fd));
    std::printf("Fx_front(sum)=%.2f Fx_rear(sum)=%.2f F_roll(qsy, all 4)=%.2f F_aero=%.2f N  Fz_total=%.1f Fz_front=%.1f\n", fx_f,
                fx_r, f_roll, f_aero, fz, s_fz_f * inv);
    std::printf("chassis balance: m*a=%.2f  vs  Fx_f+Fx_r-F_roll-F_aero=%.2f N\n", mass * a, fx_f + fx_r - f_roll - f_aero);
    std::printf("hub torque as wheel force: hub_sum/R=%.2f N (front Fx implied incl. hub inertia)\n", hub_sum / R);
    auto rate = [&](real a1, real a0) { return (a1 - a0) / T; };
    std::printf("ledger rates (W): engine_out=%.1f engine_friction_loss=%.1f gear_loss=%.1f tyre_loss=%.1f wheel_out=%.1f "
                "integrator_loss=%.2f d(rotor_ke)/dt=%.1f clutch_loss=%.3f\n",
                rate(e1.engine_out, e0.engine_out), rate(e1.engine_friction_loss, e0.engine_friction_loss),
                rate(e1.gear_loss, e0.gear_loss), rate(e1.tyre_loss, e0.tyre_loss), rate(e1.wheel_out, e0.wheel_out),
                rate(e1.integrator_loss, e0.integrator_loss), rate(e1.rotor_ke, e0.rotor_ke), rate(e1.clutch_loss, e0.clutch_loss));
    // Reference-side engine-braking force at the wheels at the same rpm,
    // inertia ignored: drive branch (the pre-fix RefCar), plan-literal
    // overrun branch (2-eta), and 1/eta.
    const ps_test::GearLossParams gp = ps_test::ref_gear_loss_params(rc, 3);
    std::printf("engine force at wheels, |M| only: drive-branch 2*K_half*|M|/R=%.2f N, overrun (2-eta) %.2f N, overrun 1/eta %.2f N\n",
                2.0 * gp.hub_torque_gain * std::abs(tau_m) / R, g * std::abs(tau_m) * (2.0 - eta_k) * (2.0 - eta_fd) / R,
                g * std::abs(tau_m) / (eta_k * eta_fd) / R);
    // Same, with the sim's own measured accelerations (inertia included):
    const real a_hub = a / R;
    std::printf("RefCar hub-accel solve at the sim's own state: a_hub(ref)=%.5f vs sim a/R=%.5f rad/s^2\n",
                ps_test::ref_locked_hub_accel(gp, tau_m, R * fx_f / 2.0, rc.driven_wheel_inertia_kgm2), a_hub);
    SUCCEED();
}

// PHYS-033 P4 I3 plan section 13.5(a): stall under load. Facing uphill on a
// 30% slope, handbraked, 1st gear selected with the clutch pressed (safe to
// shift), then the clutch is DUMPED (0) in one tick with no throttle -
// auto-clutch off (manual dump). The driveline, rigidly loaded against the
// handbraked wheels via the slope, must stall the engine.
TEST_CASE("Stall under load: dumping the clutch against a handbraked uphill load stalls the engine", "[validation][vehicle][powertrain][drivetrain]") {
    const real mass = 1500.0;
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, 0.6, /*slope_grade=*/0.30);
    rig.world->set_control("handbrake", 1.0);
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("clutch", 1.0); // pressed/disengaged: safe to shift
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.0);
    ps_test::settle(*rig.world, 1.0); // engine idling, gear engaged, clutch open, handbrake held

    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 1);
    REQUIRE(rig.world->powertrain_state(rig.vehicle).engine_state == drivetrain::EngineState::Running);

    rig.world->set_control("clutch", 0.0); // the dump, in one tick

    const real dt = rig.world->dt();
    bool stalled_within_0p5 = false;
    // Single continuous loop from the dump to t=3.0s (0.5s detection window,
    // then hold to 1.0s, then a further 2.0s) - measuring omega_e at exactly
    // t=1.0s and t=3.0s SINCE THE DUMP, not since whenever Stalled first
    // fired (an earlier version of this test restarted its tick budget from
    // the detection point instead of the dump, so "1.0s" and "3.0s" landed
    // early - found via this test's own first run).
    const std::uint64_t ticks_0p5 = static_cast<std::uint64_t>(0.5 / dt + 0.5);
    const std::uint64_t ticks_1p0 = static_cast<std::uint64_t>(1.0 / dt + 0.5);
    const std::uint64_t ticks_3p0 = static_cast<std::uint64_t>(3.0 / dt + 0.5);
    real omega_e_at_1s = 0.0;
    real max_abs_omega_e_1s_to_3s = 0.0;
    for (std::uint64_t i = 0; i < ticks_3p0; ++i) {
        rig.world->step();
        if (!stalled_within_0p5 && i + 1 <= ticks_0p5 &&
            rig.world->powertrain_state(rig.vehicle).engine_state == drivetrain::EngineState::Stalled) {
            stalled_within_0p5 = true;
        }
        if (i + 1 == ticks_1p0) {
            omega_e_at_1s = ps_test::ref_rpm_to_rad(rig.world->powertrain_state(rig.vehicle).engine_rpm);
        }
        if (i + 1 >= ticks_1p0) {
            max_abs_omega_e_1s_to_3s =
                std::max(max_abs_omega_e_1s_to_3s, std::abs(ps_test::ref_rpm_to_rad(rig.world->powertrain_state(rig.vehicle).engine_rpm)));
        }
    }
    CHECK(stalled_within_0p5);
    INFO("omega_e_at_1s=" << omega_e_at_1s << " rad/s");
    CHECK(std::abs(omega_e_at_1s) < 1e-6);
    CHECK(max_abs_omega_e_1s_to_3s < 1e-6);

    // Restart: clutch back to pressed (disengaged, so cranking is not
    // loaded), starter on.
    rig.world->set_control("clutch", 1.0);
    rig.world->set_control("starter", 1.0);
    bool running_within_1p5 = false;
    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(1.5 / dt + 0.5); ++i) {
        rig.world->step();
        if (rig.world->powertrain_state(rig.vehicle).engine_state == drivetrain::EngineState::Running) {
            running_within_1p5 = true;
            break;
        }
    }
    CHECK(running_within_1p5);
    rig.world->set_control("starter", 0.0);

    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(1.0 / dt + 0.5); ++i) rig.world->step();
    const real idle_rpm = 800.0; // data/engines/i4_2l_na.json
    const real rpm_1s_after = rig.world->powertrain_state(rig.vehicle).engine_rpm;
    INFO("rpm_1s_after_restart=" << rpm_1s_after << " idle_rpm=" << idle_rpm);
    CHECK(rpm_1s_after > 0.85 * idle_rpm);
    CHECK(rpm_1s_after < 1.15 * idle_rpm);
}

// PHYS-033 P4 I3 plan section 13.5(b): gentle launch, idle control only -
// engagement kept below the idle engine's surplus torque must NOT stall it,
// and the car creeps away on idle alone.
TEST_CASE("Gentle idle-only launch creeps away without stalling", "[validation][vehicle][powertrain][drivetrain]") {
    const real mass = 1500.0;
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("clutch", 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.0);
    ps_test::settle(*rig.world, 1.0);
    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 1);

    const real dt = rig.world->dt();
    const real idle_rpm = 800.0;
    bool never_stalled = true;
    real min_rpm = 1e9;

    // Pedal ramps 1 -> 0.71 over 1.0s (engagement ~0.04, plan section
    // 13.5(b)'s own numbers).
    const std::uint64_t ramp_ticks = static_cast<std::uint64_t>(1.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < ramp_ticks; ++i) {
        const real t = static_cast<real>(i + 1) * dt;
        rig.world->set_control("clutch", 1.0 - 0.29 * (t / 1.0));
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        if (pt.engine_state == drivetrain::EngineState::Stalled) never_stalled = false;
        min_rpm = std::min(min_rpm, pt.engine_rpm);
    }

    // Hold at 0.71 until slip closes (|omega_e - omega_i| < 100 rpm), 15s timeout.
    rig.world->set_control("clutch", 0.71);
    bool slip_closed = false;
    const std::uint64_t hold_ticks_max = static_cast<std::uint64_t>(15.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < hold_ticks_max; ++i) {
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        if (pt.engine_state == drivetrain::EngineState::Stalled) never_stalled = false;
        min_rpm = std::min(min_rpm, pt.engine_rpm);
        const real slip_rpm = std::abs(ps_test::ref_rad_to_rpm(rig.world->powertrain_debug(rig.vehicle).clutch_slip));
        if (slip_rpm < 100.0) {
            slip_closed = true;
            break;
        }
    }
    // Precondition (plan section 13.5(b)): the hold phase must end before
    // the 15s timeout - a setup problem (S5), not a tolerance problem, if it
    // does not.
    REQUIRE(slip_closed);

    // Release to 0 over 1.0s.
    const std::uint64_t release_ticks = static_cast<std::uint64_t>(1.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < release_ticks; ++i) {
        const real t = static_cast<real>(i + 1) * dt;
        rig.world->set_control("clutch", 0.71 * (1.0 - t / 1.0));
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        if (pt.engine_state == drivetrain::EngineState::Stalled) never_stalled = false;
        min_rpm = std::min(min_rpm, pt.engine_rpm);
    }

    CHECK(never_stalled);
    INFO("min_rpm=" << min_rpm << " (0.8*idle=" << (0.8 * idle_rpm) << ")");
    CHECK(min_rpm >= 0.8 * idle_rpm);

    // After release: rpm within +-10% of idle for 3s, car creeping (> 1 m/s).
    bool rpm_in_band_for_3s = true;
    real speed_after = 0.0;
    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(3.0 / dt + 0.5); ++i) {
        rig.world->step();
        const real rpm = rig.world->powertrain_state(rig.vehicle).engine_rpm;
        if (rpm < 0.9 * idle_rpm || rpm > 1.1 * idle_rpm) rpm_in_band_for_3s = false;
        speed_after = rig.world->get_motion(rig.chassis).linear.length();
    }
    CHECK(rpm_in_band_for_3s);
    CHECK(speed_after > 1.0);
}

// PHYS-033 P4 I3 plan section 13.5(c): auto-clutch anti-stall - braking to a
// stop in 1st with auto-clutch on must never stall (this is also the exact
// scenario the C8 smoke test's own anti-stall panic-release fix targets,
// CLAUDE.md's "Vehicle system"/manual_tcu.cpp's own comment).
TEST_CASE("Auto-clutch anti-stall never lets the engine stall while braking to a stop", "[validation][vehicle][powertrain][drivetrain]") {
    const real mass = 1500.0;
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.5);
    const real dt = rig.world->dt();
    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(3.0 / dt + 0.5); ++i) rig.world->step(); // get to ~30 km/h

    const real speed_before_brake = rig.world->get_motion(rig.chassis).linear.length();
    INFO("speed_before_brake_kmh=" << (speed_before_brake * 3.6));
    REQUIRE(speed_before_brake > 5.0 / 3.6); // sanity: actually moving

    rig.world->set_control("throttle", 0.0);
    rig.world->set_control("brake", 0.6);

    bool never_stalled = true;
    real min_rpm = 1e9;
    const real idle_rpm = 800.0;
    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(6.0 / dt + 0.5); ++i) {
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        if (pt.engine_state == drivetrain::EngineState::Stalled) never_stalled = false;
        if (pt.engine_state == drivetrain::EngineState::Running) min_rpm = std::min(min_rpm, pt.engine_rpm);
    }
    CHECK(never_stalled);
    INFO("min_rpm=" << min_rpm);
    CHECK(min_rpm >= 0.8 * idle_rpm);
}
