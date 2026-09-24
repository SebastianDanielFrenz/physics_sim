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
// Sign/efficiency convention (derived from the engine's own equation,
// J_e*omega_e' = tau_e - T_c: T_c is "torque flowing from the engine to the
// driveline through the clutch" and stays POSITIVE whenever the engine is
// the net torque SOURCE for the driveline - true both in normal driving AND
// in engine-braking/DFCO coast, where the engine's own negative motoring/
// friction torque acts as a retarding DEMAND the driveline "pulls" through
// the same positive-T_c direction. This reference therefore uses the single
// "drive-direction" efficiency branch (T_out = r*eta*T_m) throughout - the
// plan's literal T_out = r*(T_m - (1-eta)*|T_m|) collapses to exactly that
// whenever T_m >= 0, which both this file's test 5 (engine braking) and
// test 1 (launch/accel) scripts keep true throughout (checked, not merely
// assumed - see each test's own precondition asserts).
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
    ps::real hub_torque_gain = 0.0;    // "K_half": engine torque -> ONE hub's drive torque, drive-direction efficiency
    ps::real hub_reflected_inertia = 0.0; // engine+input+output inertia, reflected to omega_hub' (drive-direction)
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
    return p;
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

    const ps::real omega_hub_dot =
        (gp.hub_torque_gain * tau_e - rc.wheel_radius_m * fx) / (rc.driven_wheel_inertia_kgm2 + gp.hub_reflected_inertia);

    // Rolling resistance (plan section 13's own vehicle-level formula: "Sum
    // Fz_i (qsy1 + qsy4 (v/v0)^4)") is applied here, at the CHASSIS level
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
    const ps::real crr_total = rc.tyre_desc.qsy1 + rc.tyre_desc.qsy4 * std::pow(s.v / rc.tyre_desc.v0, ps::real(4.0));
    const ps::real f_roll = crr_total * mass * g_grav;
    const ps::real f_aero = ps::real(0.5) * ps::real(1.225) * rc.drag_area_m2 * s.v * std::abs(s.v);
    const ps::real m_eff = mass + ps::real(2.0) * rc.undriven_wheel_inertia_kgm2 / (rc.wheel_radius_m * rc.wheel_radius_m);
    const ps::real v_dot = (ps::real(2.0) * fx - f_roll - f_aero) / m_eff;

    s.omega_hub += omega_hub_dot * h;
    s.v += v_dot * h;
    s.prev_accel = v_dot;
}

} // namespace ps_test
