// tests/support/powertrain_reference.h — shared, independent reference model
// for the PHYS-033 P4 I3 powertrain validation tests (plan section 13's own
// "RefCar" built FROM the public VehicleDesc, never hand-duplicated JSON
// constants, so the reference can never silently drift from the data file
// the sim itself loads).
//
// This is a DELIBERATELY simplified point-mass/lumped-driveline model, not a
// re-implementation of core/src/drivetrain's own implicit shaft-network
// solver - plan section 13's own words: "it neglects suspension and pitch
// dynamics, the mesh-loss one-substep lag, event quantisation (<= 1/240 s
// per event), and lateral dynamics. Expected combined error <= 1%." Its job
// is an INDEPENDENT cross-check of the sim's own numbers, not a byte-exact
// replica of its algorithm - see out/... test headers for the exact gates.
//
// Sign/efficiency convention (plan section 13's node equations with the
// section 5.4 loss convention, which is symmetric in the direction of power
// flow). From the engine's own equation J_e*omega_e' = tau_e - T_c, T_c is
// the torque flowing from the engine INTO the driveline through the clutch.
// It is POSITIVE in drive and NEGATIVE on overrun (engine braking / DFCO
// coast): there tau_e = M(rpm) < 0, the engine is a torque SINK, and
// T_c = tau_e - J_e*omega_e' ~ M(rpm) + J_e*|omega_e'| < 0 (the decelerating
// engine inertia gives back a little). Measured, test 5's operating point
// (3360 rpm, 25.56 m/s): M = -47.52 N*m, |clutch lambda| = 45.95 N*m =
// |M| - J_e*|omega_e'|, driveline->engine.
//
// Each lossy mesh therefore uses the plan's literal
//   T_out = r*(T_m - (1-eta)*|T_m|)  ==  r*eta*T_m      for T_m >= 0 (drive)
//                                    ==  r*(2-eta)*T_m  for T_m <  0 (overrun)
// i.e. on overrun the loss ADDS to the torque the wheels must deliver, it
// does not reduce it. (An earlier version of this file used the drive branch
// r*eta*T_m on overrun too, on the incorrect premise that T_c stays positive
// when engine braking. That understated the engine-braking force at the
// wheels by the factor eta_k*eta_fd / ((2-eta_k)*(2-eta_fd)) = 0.905 in 4th,
// 624.8 N instead of 690.4 N at 3360 rpm. The sim's measured hub torque was
// 1.0521 x the lossless demand, i.e. 1/(eta_k*eta_fd) = 1.0520, so the sim
// applies the overrun branch correctly. That was the whole of test 5's 5.6%
// gap.)
//
// (2-eta) vs 1/eta: the sim's gearbox loss is -(1-eta)*|lambda_prev| where
// the reconstructed lambda_prev at N1 is the pre-loss input torque T_m,
// which is exactly the (2-eta) form above. The sim's differential
// reconstructs its lambda_prev at the left hub, i.e. the post-loss
// transmitted torque, which gives 1/eta on overrun instead of (2-eta):
// 1.0309 vs 1.0300 for eta_fd = 0.97, a 0.09% difference, well inside the
// "<= 1%" budget below. The reference keeps the plan's literal formula.
//
// Because the branch depends on the sign of T_m and T_fd, which depend on
// the unknown hub acceleration through the inertia terms, the locked-gear
// step solves the (piecewise-linear, monotone) hub equation once per sign
// case and keeps the self-consistent one - plan section 13's own "solve the
// scalar linear equation for omega_d' per sign case".
//
// tests/support is test-only shared support (see counter_force_element.h,
// vehicle_test_helpers.h in this same directory) - never included from
// core/src.

#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/vehicle/tyre_model.h"
#include "ps/vehicle/vehicle_desc.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps_test {

constexpr ps::real kRefPi = 3.14159265358979323846;
inline ps::real ref_rpm_to_rad(ps::real rpm) { return rpm * (kRefPi / ps::real(30.0)); }
inline ps::real ref_rad_to_rpm(ps::real rad) { return rad * (ps::real(30.0) / kRefPi); }

// The tyre's own rolling-resistance coefficient at ground speed v, built
// from the SAME speed law the wheel uses (ps::vehicle::tyre_rolling_speed_term,
// one definition for qsy3/qsy4) so a reference cannot keep a stale copy of it.
// qsy2 and the surface crr are 0 for every reference vehicle (box ground).
inline ps::real ref_crr(const ps::vehicle::TyreDesc& tyre, ps::real v) {
    return tyre.qsy1 + ps::vehicle::tyre_rolling_speed_term(tyre, v);
}

// See this file's header comment - pulled directly from the public
// VehicleDesc, field for field, rather than hand-duplicated JSON constants.
struct RefCar {
    // Engine (ps::drivetrain::TorqueMapEngineDesc)
    ps::vehicle::Curve1D wot, motoring;
    ps::real engine_inertia_kgm2 = 0.0;
    ps::real idle_rpm = 800.0, stall_rpm = 400.0;
    ps::real limiter_rpm = 6800.0;

    // Clutch (ps::drivetrain::ClutchDesc)
    ps::real clutch_capacity_nm = 0.0;
    ps::real clutch_static_factor = 1.15;
    ps::vehicle::Curve1D engagement_vs_pedal;
    ps::real clutch_out_inertia_kgm2 = 0.0;

    // Gearbox (ps::drivetrain::GearboxDesc)
    std::vector<ps::real> forward_ratios;     // index 0 == 1st
    std::vector<ps::real> forward_efficiency; // same length
    ps::real gearbox_input_inertia_kgm2 = 0.0, gearbox_output_inertia_kgm2 = 0.0;

    // Differential (ps::drivetrain::DifferentialDesc)
    ps::real diff_ratio = 1.0, diff_efficiency = 1.0;
    ps::real diff_carrier_inertia_kgm2 = 0.0;

    // Driven axle (assumes exactly 2 symmetric driven wheels - true for
    // both car_sedan.json (FWD) and tractor.json (2WD) as of I3)
    ps::real driven_wheel_inertia_kgm2 = 0.0;
    ps::real undriven_wheel_inertia_kgm2 = 0.0; // for the chassis equation's "2*J_w,undriven/R^2" term
    ps::real wheel_radius_m = 0.3;
    std::unique_ptr<ps::vehicle::ITyreModel> tyre; // driven axle's tyre model
    ps::vehicle::TyreDesc tyre_desc;               // kept for relaxation lengths etc.

    ps::real drag_area_m2 = 0.0;

    // J_i (clutch-out + gearbox-in, shaft-merged at the gearbox input node)
    // and J_o (gearbox-out + diff-carrier, shaft-merged at the differential
    // carrier node) - see core/src/drivetrain/powertrain.cpp's own
    // node_for_key union-find, which merges these exact ports.
    [[nodiscard]] ps::real j_input_node() const { return clutch_out_inertia_kgm2 + gearbox_input_inertia_kgm2; }
    [[nodiscard]] ps::real j_output_node() const { return gearbox_output_inertia_kgm2 + diff_carrier_inertia_kgm2; }

    static RefCar build(const ps::vehicle::VehicleDesc& desc);
};

inline RefCar RefCar::build(const ps::vehicle::VehicleDesc& desc) {
    using namespace ps::drivetrain;
    RefCar rc;
    bool have_engine = false, have_clutch = false, have_gearbox = false, have_diff = false;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            rc.wot = e->wot_torque_nm_vs_rpm;
            rc.motoring = e->motoring_torque_nm_vs_rpm;
            rc.engine_inertia_kgm2 = e->inertia_kgm2;
            rc.idle_rpm = e->idle_rpm;
            rc.stall_rpm = e->stall_rpm;
            rc.limiter_rpm = e->limiter.rpm;
            have_engine = true;
        } else if (const auto* cl = std::get_if<ClutchDesc>(&c.params)) {
            rc.clutch_capacity_nm = cl->capacity_nm;
            rc.clutch_static_factor = cl->static_factor;
            rc.engagement_vs_pedal = cl->engagement_vs_pedal;
            rc.clutch_out_inertia_kgm2 = cl->out_inertia_kgm2;
            have_clutch = true;
        } else if (const auto* gb = std::get_if<GearboxDesc>(&c.params)) {
            rc.forward_ratios = gb->forward_ratios;
            rc.forward_efficiency = gb->forward_efficiency;
            rc.gearbox_input_inertia_kgm2 = gb->input_inertia_kgm2;
            rc.gearbox_output_inertia_kgm2 = gb->output_inertia_kgm2;
            have_gearbox = true;
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            rc.diff_ratio = d->ratio;
            rc.diff_efficiency = d->efficiency;
            rc.diff_carrier_inertia_kgm2 = d->carrier_inertia_kgm2;
            have_diff = true;
        }
    }
    if (!have_engine || !have_clutch || !have_gearbox || !have_diff) {
        throw std::runtime_error("RefCar::build: vehicle powertrain must have exactly one engine/clutch/gearbox/differential");
    }

    // Driven wheels: any WheelDesc::name reachable as a "wheel:<name>" shaft
    // port (mirrors ps::drivetrain::Powertrain::wheel_is_driven's own
    // definition, powertrain.cpp).
    std::vector<std::string> driven_names;
    for (const auto& shaft : desc.powertrain.shafts) {
        for (const auto& port : shaft.ports) {
            if (port.is_wheel) driven_names.push_back(port.component);
        }
    }
    int driven_a = -1, undriven_a = -1;
    for (std::size_t i = 0; i < desc.wheels.size(); ++i) {
        const bool driven = std::find(driven_names.begin(), driven_names.end(), desc.wheels[i].name) != driven_names.end();
        if (driven && driven_a < 0) driven_a = static_cast<int>(i);
        if (!driven && undriven_a < 0) undriven_a = static_cast<int>(i);
    }
    if (driven_a < 0 || undriven_a < 0) {
        throw std::runtime_error("RefCar::build: expected both a driven and an undriven wheel (I3 scope: 2WD only)");
    }
    const auto& wd = desc.wheels[static_cast<std::size_t>(driven_a)];
    const auto& wu = desc.wheels[static_cast<std::size_t>(undriven_a)];
    rc.driven_wheel_inertia_kgm2 = wd.wheel_inertia;
    rc.undriven_wheel_inertia_kgm2 = wu.wheel_inertia;
    rc.wheel_radius_m = wd.wheel_radius;
    rc.tyre_desc = wd.tyre;
    rc.tyre = ps::vehicle::create_pacejka_tyre_model(wd.tyre);
    rc.drag_area_m2 = desc.aero.drag_area_m2;
    return rc;
}

// engine_torque(rpm, theta) = M(rpm) + theta*(W(rpm) - M(rpm)) (plan section
// 13's own formula; independently re-implemented from the Curve1Ds, not a
// call into core/src's own EngineTorqueMap).
inline ps::real ref_engine_torque(const RefCar& rc, ps::real rpm, ps::real theta) {
    const ps::real m = rc.motoring.evaluate(rpm);
    const ps::real w = rc.wot.evaluate(rpm);
    return m + theta * (w - m);
}

// Per-gear derived quantities for the LOCKED-clutch, single-fixed-ratio
// driveline (plan section 13's "engaged k" node-form equations), referred to
// ONE representative driven hub (both driven wheels assumed identical by
// symmetry - true for every I3 vehicle).
struct GearLossParams {
    ps::real g = 1.0;             // overall ratio: omega_e = g * omega_hub
    // Drive-branch (T_m, T_fd >= 0) closed forms only - valid for steady
    // positive-torque use such as test 2's top-speed bisection. Anything
    // that can go into overrun must use ref_locked_hub_accel() below.
    ps::real hub_torque_gain = 0.0;    // "K_half": engine torque -> ONE hub's drive torque, drive-direction efficiency
    ps::real hub_reflected_inertia = 0.0; // engine+input+output inertia, reflected to omega_hub' (drive-direction)
    // Raw chain data for the per-sign-case solve.
    ps::real r_k = 1.0, eta_k = 1.0, r_fd = 1.0, eta_fd = 1.0;
    ps::real j_ei = 0.0; // engine + clutch-out + gearbox-in (locked clutch: one node)
    ps::real j_o = 0.0;  // gearbox-out + diff carrier
};

inline GearLossParams ref_gear_loss_params(const RefCar& rc, std::size_t gear_index_zero_based) {
    if (gear_index_zero_based >= rc.forward_ratios.size()) {
        throw std::out_of_range("ref_gear_loss_params: gear index out of range");
    }
    const ps::real r_k = rc.forward_ratios[gear_index_zero_based];
    const ps::real eta_k = rc.forward_efficiency[gear_index_zero_based];
    const ps::real r_fd = rc.diff_ratio;
    const ps::real eta_fd = rc.diff_efficiency;

    GearLossParams p;
    p.g = r_k * r_fd;
    // K_half = 1/2 * r_fd * eta_fd * r_k * eta_k = 1/2 * g * eta_k * eta_fd
    // (algebraically identical either way - kept in the r_k/r_fd form here
    // to mirror the plan's own staged T_m -> T_out -> T_fd -> per-hub
    // derivation, so a reader can check each factor against section 13's
    // text directly).
    p.hub_torque_gain = ps::real(0.5) * r_fd * eta_fd * r_k * eta_k;
    // Reflected inertia (drive-direction: T_out=r*eta*T_m throughout, see
    // this file's header comment): derived from
    //   T_c = tau_e - J_e*g*omega_hub'
    //   T_m = T_c - J_i*g*omega_hub'  = tau_e - (J_e+J_i)*g*omega_hub'
    //   T_out = r_k*eta_k*T_m
    //   T_fd_node = T_out - J_o*r_fd*omega_hub'
    //   T_hub = 1/2*r_fd*eta_fd*T_fd_node
    //         = K_half*tau_e - [1/2*r_fd*eta_fd*(r_k*eta_k*(J_e+J_i)*g + J_o*r_fd)] * omega_hub'
    const ps::real j_ei = rc.engine_inertia_kgm2 + rc.j_input_node();
    p.hub_reflected_inertia =
        ps::real(0.5) * r_fd * eta_fd * (r_k * eta_k * j_ei * p.g + rc.j_output_node() * r_fd);
    p.r_k = r_k;
    p.eta_k = eta_k;
    p.r_fd = r_fd;
    p.eta_fd = eta_fd;
    p.j_ei = j_ei;
    p.j_o = rc.j_output_node();
    return p;
}

// Locked clutch, fixed gear: one driven hub's angular acceleration a =
// omega_hub' from the plan section 13 node chain with the direction-aware
// loss (this file's header comment):
//   T_m   = tau_e - J_ei*g*a                         (mesh input torque at N1)
//   T_fd  = r_k*phi_k(T_m) - J_o*r_fd*a              (diff input torque at N2)
//   J_w*a = 1/2*r_fd*phi_fd(T_fd) - R*Fx
// with phi(x) = x - (1-eta)*|x| = e*x, e = eta (x >= 0) or 2-eta (x < 0).
// Per sign case (e1, e2) the equation is linear:
//   a*(J_w + 1/2*r_fd*e2*(r_k*e1*J_ei*g + J_o*r_fd)) = 1/2*r_fd*e2*r_k*e1*tau_e - R*Fx
// The composed right-hand side is continuous and strictly monotone in a
// (every phi is increasing), so exactly one case is self-consistent (two at
// a shared boundary, where they agree); that one is returned.
inline ps::real ref_locked_hub_accel(const GearLossParams& gp, ps::real tau_e, ps::real r_fx, ps::real j_w) {
    const ps::real e1s[2] = {gp.eta_k, ps::real(2.0) - gp.eta_k};
    const ps::real e2s[2] = {gp.eta_fd, ps::real(2.0) - gp.eta_fd};
    ps::real fallback = 0.0;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            const ps::real e1 = e1s[i], e2 = e2s[j];
            const ps::real num = ps::real(0.5) * gp.r_fd * e2 * gp.r_k * e1 * tau_e - r_fx;
            const ps::real den = j_w + ps::real(0.5) * gp.r_fd * e2 * (gp.r_k * e1 * gp.j_ei * gp.g + gp.j_o * gp.r_fd);
            const ps::real a = num / den;
            const ps::real t_m = tau_e - gp.j_ei * gp.g * a;
            const ps::real t_fd = gp.r_k * e1 * t_m - gp.j_o * gp.r_fd * a;
            const bool ok1 = (i == 0) ? (t_m >= ps::real(0.0)) : (t_m <= ps::real(0.0));
            const bool ok2 = (j == 0) ? (t_fd >= ps::real(0.0)) : (t_fd <= ps::real(0.0));
            if (ok1 && ok2) return a;
            if (i == 0 && j == 0) fallback = a;
        }
    }
    return fallback; // unreachable by the monotonicity argument above; kept total for safety
}

// Per-substep state for the locked-gear (no shifting, no clutch slip)
// integrator - test 5's own script ("Engaged(4), Locked, theta=0").
struct RefLockedGearState {
    ps::real v = 0.0;         // chassis longitudinal speed, m/s
    ps::real omega_hub = 0.0; // one driven hub's angular speed, rad/s
    ps::vehicle::SlipRelaxationState slip;
    ps::real prev_accel = 0.0; // last computed v' (for the NEXT substep's dynamic Fz_f)
};

// One semi-implicit Euler substep of the locked-gear reference (plan section
// 13's own integrator choice: torques evaluated at the OLD state, then
// v_new = v_old + a*h). `theta` is this substep's throttle fraction (0 for
// engine braking); `mass`/`h_cg`/`wheelbase_m`/`front_static_fraction` come
// from the test's own settled-rig measurement (plan: "static front fraction
// (from wheel_telemetry after the settle)"); `front_static_fraction` is the
// STATIC front-axle load fraction (0.5 for a 50/50 car).
inline void ref_locked_gear_step(RefLockedGearState& s, const RefCar& rc, const GearLossParams& gp, ps::real theta,
                                  ps::real mass, ps::real h_cg, ps::real wheelbase_m, ps::real front_static_fraction,
                                  ps::real h) {
    const ps::real g_grav = ps::real(9.81);
    // Dynamic per-wheel front load (plan section 13: Fz_f = 1/2*(m*g*f_front_static - m*v'_prev*h_cg/L)),
    // using LAST substep's own acceleration (explicit, matching the plan's
    // "neglects... the mesh-loss one-substep lag" tolerance for this kind of
    // small lag).
    const ps::real fz_front_total = mass * g_grav * front_static_fraction - mass * s.prev_accel * h_cg / wheelbase_m;
    const ps::real fz_f = ps::real(0.5) * std::max(fz_front_total, ps::real(0.0));

    const ps::real slip_velocity_x = s.omega_hub * rc.wheel_radius_m - s.v;
    ps::vehicle::step_slip_relaxation(s.slip, slip_velocity_x, /*slip_velocity_y=*/0.0, std::abs(s.v),
                                       rc.tyre_desc.relaxation_length_x, rc.tyre_desc.relaxation_length_y, h);

    const ps::vehicle::TyreSlipInput in{/*kappa=*/s.slip.kappa_eq, /*alpha=*/0.0,
                                         /*fz=*/fz_f, /*gamma=*/0.0, /*vx=*/s.v,
                                         /*lambda_mu=*/1.0, /*crr_surface=*/0.0};
    const ps::vehicle::TyreForces tf = rc.tyre->evaluate(in);
    const ps::real fx = tf.fx; // per driven wheel

    const ps::real engine_rpm = ref_rad_to_rpm(gp.g * s.omega_hub);
    const ps::real tau_e = ref_engine_torque(rc, engine_rpm, theta);

    const ps::real omega_hub_dot = ref_locked_hub_accel(gp, tau_e, rc.wheel_radius_m * fx, rc.driven_wheel_inertia_kgm2);

    // Rolling resistance (plan section 13's own vehicle-level formula: "Sum
    // Fz_i crr(v)", crr = qsy1 + qsy3|v/v0| + qsy4 (v/v0)^4 via ref_crr) is applied here, at the CHASSIS level
    // only, over all 4 wheels' combined static-weight approximation - the
    // plan's own simplification (its reference vehicle equation has no
    // separate per-wheel rolling-resistance-on-the-hub term), matching the
    // "expected combined error <= 1%" disclaimer's tolerance for exactly
    // this kind of small structural approximation. (An earlier version of
    // this file tried folding a driven-wheel share of rolling resistance
    // into the hub torque balance too, reasoning from wheel_instance.cpp's
    // own fx_total = fx_lin + pre_froll_ - measured WORSE agreement with the
    // sim, 14% vs this version's ~5-6%, so it was reverted; qsy2=0 for
    // every I3 tyre and the box-ground surface's own crr is 0, so this
    // omits both those terms too, matching the plan's own formula exactly.)
    const ps::real crr_total = ref_crr(rc.tyre_desc, s.v);
    const ps::real f_roll = crr_total * mass * g_grav;
    const ps::real f_aero = ps::real(0.5) * ps::real(1.225) * rc.drag_area_m2 * s.v * std::abs(s.v);
    const ps::real m_eff = mass + ps::real(2.0) * rc.undriven_wheel_inertia_kgm2 / (rc.wheel_radius_m * rc.wheel_radius_m);
    const ps::real v_dot = (ps::real(2.0) * fx - f_roll - f_aero) / m_eff;

    s.omega_hub += omega_hub_dot * h;
    s.v += v_dot * h;
    s.prev_accel = v_dot;
}

// ---------------------------------------------------------------------
// Steady-state top-speed reference (plan section 13.2), shared by
// test_powertrain_accel_topspeed.cpp (sedan) and test_hyper_top_speed.cpp.
// Moved here unchanged from the sedan test when the hypercar needed it;
// solve_gear_top_speed now takes the resistance as a callable and the
// bisection range as parameters (defaults are the plan's [10, 90] m/s).
// ---------------------------------------------------------------------


struct GearDriveForce {
    ps::real f_drive = 0.0;
    ps::real kappa = 0.0;
    ps::real engine_rpm = 0.0;
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
inline GearDriveForce ref_gear_drive_force(const RefCar& rc, const GearLossParams& gp, ps::real v, ps::real fz_f) {
    ps::real kappa = 0.0;
    ps::real f_drive = 0.0;
    for (int outer = 0; outer < 30; ++outer) {
        const ps::real omega_hub = (ps::real(1.0) + kappa) * v / rc.wheel_radius_m;
        const ps::real omega_e_rad = gp.g * omega_hub;
        const ps::real rpm = ref_rad_to_rpm(omega_e_rad);
        const ps::real tau_wot = rc.wot.evaluate(rpm);
        const ps::real new_f_drive = ps::real(2.0) * gp.hub_torque_gain * tau_wot / rc.wheel_radius_m;

        ps::real lo = ps::real(0.0), hi = ps::real(0.3);
        for (int b = 0; b < 60; ++b) {
            const ps::real mid = ps::real(0.5) * (lo + hi);
            const ps::vehicle::TyreSlipInput in{mid, ps::real(0.0), fz_f, ps::real(0.0), v, ps::real(1.0), ps::real(0.0)};
            const ps::real fx = rc.tyre->evaluate(in).fx;
            if (fx < new_f_drive / ps::real(2.0)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        const ps::real new_kappa = ps::real(0.5) * (lo + hi);
        f_drive = new_f_drive;
        if (std::abs(new_kappa - kappa) < ps::real(1e-7)) {
            kappa = new_kappa;
            break;
        }
        kappa = new_kappa;
    }
    GearDriveForce r;
    r.f_drive = f_drive;
    r.kappa = kappa;
    r.engine_rpm = ref_rad_to_rpm(gp.g * (ps::real(1.0) + kappa) * v / rc.wheel_radius_m);
    return r;
}

struct GearTopSpeedResult {
    bool valid = false;
    ps::real v = 0.0;
    ps::real engine_rpm = 0.0;
};

// Bisects F_drive(v) - F_res(v) == 0 on v in [10, 90] (plan section 13.2's
// own search range) for one gear; rejects the gear if no sign change is
// found in that range, or if the root's own engine rpm is at/above the
// limiter (plan: "reject a gear if omega_e >= limiter.rpm" - such a state
// is not a stable cruise, the limiter would be cutting fuel).
template <typename ResistanceFn>
GearTopSpeedResult solve_gear_top_speed(const RefCar& rc, const GearLossParams& gp, ps::real fz_f_static, ResistanceFn resistance_force,
                                        ps::real v_lo = 10.0, ps::real v_hi = 90.0) {
    auto net = [&](ps::real v) { return ref_gear_drive_force(rc, gp, v, fz_f_static).f_drive - resistance_force(v); };
    ps::real lo = v_lo, hi = v_hi;
    const ps::real f_lo = net(lo), f_hi = net(hi);
    if (f_lo <= ps::real(0.0) || f_hi > ps::real(0.0)) return {}; // no sign change in range: reject (see this fn's own comment)
    for (int it = 0; it < 60; ++it) {
        const ps::real mid = ps::real(0.5) * (lo + hi);
        if (net(mid) > ps::real(0.0)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const ps::real v_root = ps::real(0.5) * (lo + hi);
    const GearDriveForce gd = ref_gear_drive_force(rc, gp, v_root, fz_f_static);
    if (gd.engine_rpm >= rc.limiter_rpm) return {};
    GearTopSpeedResult r;
    r.valid = true;
    r.v = v_root;
    r.engine_rpm = gd.engine_rpm;
    return r;
}


} // namespace ps_test
