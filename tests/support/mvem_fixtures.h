// tests/support/mvem_fixtures.h -- PHYS-033 P4 I7b C11: in-memory
// SimulatedEngineDesc fixtures for the MVEM white-box tests
// (tests/unit/test_simulated_engine.cpp). No data file: every map is
// tabulated here from a closed-form polynomial (+ - * / only, so the table
// values - and with them the maps' content hash - are the same bytes on
// every platform).
//
// The engine: a naturally aspirated 2.0 L I4, bore = stroke = 86 mm
// (1.998 L), 3 L intake plenum, one 60 mm throttle plate (Cd 0.8), gasoline
// (LHV 44 MJ/kg, A/F 14.5 - data/fuels/gasoline.json's values, typed in).
// Every map is ASSUMED - plausible shapes for a 2 L NA engine, not fitted
// to any published engine; the tests only need smooth, non-trivial slopes
// in every direction (rpm, p_im, air per rev, lambda, spark offset):
//   VE(rpm, p)         = 0.72 + 0.8u - 0.7u^2 + 0.05*P            u = rpm/8000, P = p/1e5 Pa
//   eta_i(rpm, m_a)    = 0.34 + 0.05u - 0.04u^2 + 30*m_a          m_a in kg per crank rev
//   lambda(rpm, p)     = 0.98 - 0.08u*P^2                          (0.90..0.98, strictly inside f_lambda's (0.8, 1.0) segment)
//   spark(rpm, p)      = 10 + 25u - 12P  [deg BTDC]
//   theta_MBT(rpm, p)  = 12 + 24u - 8P                             (spark - MBT in [-7, -1], strictly inside eta_spark's (-8, 0) segment)
//   F(rpm)             = 6 + 0.0015*rpm + 4e-7*rpm^2 [N*m]         (Coulomb part F(0) = 6)
//   f_lambda, eta_spark: short piecewise-linear curves with f_lambda(1) = eta_spark(0) = 1
// Reserved-map variant (with_reserved_maps): pmep_excess = 5000 + 20000u^2
// + 4000P [Pa]; theta_kl = 8 + 30u - 14P (active - below the spark target -
// at low rpm / high load).
//
// Electronic: actuator tau 0.05 s, idle PI on the plate. Mechanical: 50 mm^2
// bypass valve, idle PI on the valve. Both FuelCut at 6800 rpm (150 rpm
// hysteresis). Idle gains are in each actuator's own units (a plate step
// moves far more air than the same bypass step) - ASSUMED, tuned only for
// a settled idle in these tests.

#pragma once

#include "ps/drivetrain/fuel_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/table2d.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <functional>
#include <vector>

namespace ps::test::mvem {

using ps::real;
using ps::drivetrain::SimulatedEngineDesc;
using ps::drivetrain::Table2D;

inline Table2D make_table(const std::vector<real>& xs, const std::vector<real>& ys, const std::function<real(real, real)>& fn) {
    Table2D t;
    t.x = xs;
    t.y = ys;
    t.values.reserve(xs.size() * ys.size());
    for (real xv : xs)
        for (real yv : ys) t.values.push_back(fn(xv, yv));
    return t;
}

inline std::vector<real> rpm_axis() { return {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000}; }
inline std::vector<real> p_axis() { return {0, 20000, 40000, 60000, 80000, 100000, 120000}; }
inline std::vector<real> air_axis() { return {0, 3e-4, 6e-4, 9e-4, 1.2e-3, 1.5e-3}; }

inline real fx_ve(real rpm, real p) {
    const real u = rpm / 8000.0, pp = p / 1e5;
    return 0.72 + 0.8 * u - 0.7 * u * u + 0.05 * pp;
}
inline real fx_eta_i(real rpm, real ma) {
    const real u = rpm / 8000.0;
    return 0.34 + 0.05 * u - 0.04 * u * u + 30.0 * ma;
}
inline real fx_lambda(real rpm, real p) {
    const real u = rpm / 8000.0, pp = p / 1e5;
    return 0.98 - 0.08 * u * pp * pp;
}
inline real fx_spark(real rpm, real p) { return 10.0 + 25.0 * rpm / 8000.0 - 12.0 * p / 1e5; }
inline real fx_mbt(real rpm, real p) { return 12.0 + 24.0 * rpm / 8000.0 - 8.0 * p / 1e5; }
inline real fx_pmep(real rpm, real p) {
    const real u = rpm / 8000.0;
    return 5000.0 + 20000.0 * u * u + 4000.0 * p / 1e5;
}
inline real fx_kl(real rpm, real p) { return 8.0 + 30.0 * rpm / 8000.0 - 14.0 * p / 1e5; }
inline real fx_friction(real rpm) { return 6.0 + 0.0015 * rpm + 4e-7 * rpm * rpm; }

inline ps::drivetrain::FuelDesc gasoline() {
    ps::drivetrain::FuelDesc f;
    f.name = "gasoline (typed in from data/fuels/gasoline.json)";
    f.phase = ps::drivetrain::FuelPhase::Liquid;
    f.lhv_j_kg = 44.0e6;
    f.afr_stoich = 14.5;
    f.density_kg_m3 = 747.5;
    f.source = "tests/support/mvem_fixtures.h";
    return f;
}

inline SimulatedEngineDesc base_engine() {
    SimulatedEngineDesc d;
    d.architecture = ps::drivetrain::EngineArchitecture::Piston4Stroke;
    d.piston.bore_m = 0.086;
    d.piston.stroke_m = 0.086;
    d.piston.cylinders = 4;
    d.intake_manifold_volume_m3 = 3.0e-3;
    d.throttle_body.diameter_m = 0.060;
    d.throttle_body.count = 1;
    d.throttle_body.discharge_coefficient = 0.8;
    d.throttle_body.area_fraction_vs_position.x = {0.0, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0};
    d.throttle_body.area_fraction_vs_position.y = {0.001, 0.006, 0.02, 0.12, 0.40, 0.75, 1.0};

    d.ecu.lambda_target = make_table(rpm_axis(), p_axis(), fx_lambda);
    d.ecu.spark_advance_deg = make_table(rpm_axis(), p_axis(), fx_spark);
    d.maps.ve = make_table(rpm_axis(), p_axis(), fx_ve);
    d.maps.eta_i = make_table(rpm_axis(), air_axis(), fx_eta_i);
    d.maps.f_lambda.x = {0.6, 0.8, 1.0, 1.2, 1.5};
    d.maps.f_lambda.y = {0.80, 0.955, 1.0, 0.97, 0.88};
    d.maps.eta_spark.x = {-40.0, -20.0, -8.0, 0.0, 8.0};
    d.maps.eta_spark.y = {0.55, 0.82, 0.965, 1.0, 0.965};
    d.maps.theta_mbt_deg = make_table(rpm_axis(), p_axis(), fx_mbt);
    for (real r : rpm_axis()) {
        d.maps.friction_torque_nm.x.push_back(r);
        d.maps.friction_torque_nm.y.push_back(fx_friction(r));
    }
    d.maps.source = "ASSUMED analytic fixture maps (tests/support/mvem_fixtures.h)";

    d.inertia_kgm2 = 0.15;
    d.breakaway_torque_nm = 12.0;
    d.idle_rpm = 800.0;
    d.stall_rpm = 400.0;
    d.fire_rpm = 250.0;
    d.start_complete_rpm = 550.0;
    d.dfco_enable_rpm = 1300.0;
    d.dfco_reenable_rpm = 1000.0;
    d.limiter.type = ps::drivetrain::LimiterType::FuelCut;
    d.limiter.rpm = 6800.0;
    d.limiter.hysteresis_rpm = 150.0;
    d.limiter.band_rpm = 200.0;
    d.starter.stall_torque_nm = 140.0;
    d.starter.free_speed_rpm = 400.0;
    d.starter.interlock = ps::drivetrain::StarterDesc::Interlock::None;
    d.initial_state = ps::drivetrain::EngineInitialState::Running;
    d.fuels = {gasoline()};
    d.ignition = ps::drivetrain::EngineIgnition::Spark;
    return d;
}

inline SimulatedEngineDesc electronic_engine() {
    SimulatedEngineDesc d = base_engine();
    d.throttle_input = ps::drivetrain::ThrottleInput::Electronic;
    d.throttle_actuator_tau_s = 0.05;
    d.idle.kp_per_rpm = 3e-5;
    d.idle.ki_per_rpm_s = 6e-5;
    d.idle.max_throttle = 0.1;
    return d;
}

inline SimulatedEngineDesc mechanical_engine() {
    SimulatedEngineDesc d = base_engine();
    d.throttle_input = ps::drivetrain::ThrottleInput::Mechanical;
    d.idle_bypass_area_m2 = 50e-6;
    d.idle.kp_per_rpm = 1.5e-4;
    d.idle.ki_per_rpm_s = 3e-4;
    d.idle.max_throttle = 0.5;
    return d;
}

// Adds the two reserved I7c maps (pmep_excess, theta_kl).
inline SimulatedEngineDesc with_reserved_maps(SimulatedEngineDesc d) {
    d.maps.has_pmep_excess = true;
    d.maps.pmep_excess_pa = make_table(rpm_axis(), p_axis(), fx_pmep);
    d.maps.has_theta_kl = true;
    d.maps.theta_kl_deg = make_table(rpm_axis(), p_axis(), fx_kl);
    return d;
}

} // namespace ps::test::mvem
