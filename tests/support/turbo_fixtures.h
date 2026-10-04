// tests/support/turbo_fixtures.h -- PHYS-033 P4 I7d D1-D4: a code-built
// TurboPairDesc like the shipped hyper twin (data/turbos/hyper_turbo_approx.json
// x count 2 with data/turbo_configurations/hyper_twin_default.json's
// boost controller), WITHOUT any I7d block (blow-off valve, surge line,
// intercooler pressure drop). Built in code, not loaded, so a later data edit
// (I7d D6 adds a valve to the shipped configuration) cannot move the guard
// hashes in test_turbo_pair.cpp. The performance maps are generated exactly
// like ps::io::load_turbo_configuration_json does (head = reference_head *
// s^2 * (1 - f^2) on a 61 x 49 grid, outlet temperature ratio from the
// isentropic rise over the compressor efficiency).
#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/turbo_maps.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <memory>

namespace ps::test::turbo {

inline std::shared_ptr<const ps::drivetrain::TurboPerformanceMaps> make_maps(ps::real reference_head_pa,
                                                                             ps::real compressor_efficiency) {
    using ps::real;
    auto m = std::make_shared<ps::drivetrain::TurboPerformanceMaps>();
    for (int i = 0; i <= 60; ++i) m->head_pa.x.push_back(real(i) / real(40.0));
    for (int i = 0; i <= 48; ++i) m->head_pa.y.push_back(real(i) / real(40.0));
    for (real speed : m->head_pa.x)
        for (real flow : m->head_pa.y)
            m->head_pa.values.push_back(reference_head_pa * speed * speed * std::max(real(0.0), real(1.0) - flow * flow));
    for (int i = 0; i <= 90; ++i) {
        const real pr = real(1.0) + real(i) / real(10.0);
        m->compressor_temperature_ratio.x.push_back(pr);
        m->compressor_temperature_ratio.y.push_back(
            real(1.0) + (ps::math::pow(pr, real(2.0 / 7.0)) - real(1.0)) / compressor_efficiency);
    }
    m->content_hash = 0x7075726b6f6f7931ULL; // fixed tag; not a cache key here
    return m;
}

inline ps::drivetrain::TurboPairDesc hyper_like_pair() {
    ps::drivetrain::TurboPairDesc t;
    t.rotor_inertia_kgm2 = 2e-5 * 2;
    t.reference_rpm = 120000.0;
    t.reference_head_pa = 4.5e5;
    t.max_mass_flow_kg_s = 0.6 * 2;
    t.compressor_efficiency = 0.8;
    t.turbine_efficiency = 0.8;
    t.turbine_area_m2 = 0.0005 * 2;
    t.shaft_loss_w_at_reference = 600.0 * 2;
    t.target_boost_pa = 1.8e5;
    t.intercooler_effectiveness = 0.95;
    t.wastegate_kp = 1.0;
    t.wastegate_ki_per_s = 1.0;
    t.wastegate_kd_s = 0.05;
    t.wastegate_actuator_tau_s = 0.1;
    t.maps = make_maps(t.reference_head_pa, t.compressor_efficiency);
    return t;
}

// The valve of tests/support/fixtures/turbo_config_hyper_bov.json, code-built
// (pair totals: 2 x 460 mm2). [AS] values, see that file's source.
inline ps::drivetrain::BlowOffValveDesc hyper_like_bov() {
    ps::drivetrain::BlowOffValveDesc v;
    v.vent = ps::drivetrain::BlowOffValveDesc::Vent::Recirculate;
    v.crack_dp_pa = 0.9e5;
    v.full_open_dp_pa = 1.6e5;
    v.effective_area_m2 = 460e-6 * 2;
    v.tau_s = 0.04;
    v.source = "[AS] test fixture";
    return v;
}

inline ps::drivetrain::TurboPairDesc hyper_like_pair_with_bov() {
    ps::drivetrain::TurboPairDesc t = hyper_like_pair();
    t.blow_off_valve = hyper_like_bov();
    return t;
}

// The same valve with a low crack pressure and a fast diaphragm: it opens while
// the charge pressure is still high (the lumped charge collapses together with
// the rotor within ~100 ms of a lift-off, so the 0.9 bar valve above opens late).
inline ps::drivetrain::BlowOffValveDesc sensitive_bov() {
    ps::drivetrain::BlowOffValveDesc v = hyper_like_bov();
    v.crack_dp_pa = 0.2e5;
    v.full_open_dp_pa = 0.5e5;
    v.tau_s = 0.01;
    return v;
}

// The air/exhaust/fuel the engine would hand the pair at a given phase of the
// guard script (960 Hz steps): spool at WOT (0-1.5 s), lift-off (1.5-2.5 s),
// re-apply (2.5-4 s).
struct Phase { ps::real plate, air, exhaust, fuel_w; };
inline Phase script_phase(int step) {
    using ps::real;
    const real t = step * (1.0 / 960.0);
    if (t < 1.5) return {1.0, 0.2 + 0.7 * t / 1.5, (0.2 + 0.7 * t / 1.5) * 1.07, (0.2 + 0.7 * t / 1.5) / 12.5 * 43e6};
    if (t < 2.5) return {0.0, 0.04, 0.045, 0.0};
    return {1.0, 0.9, 0.96, 0.9 / 12.5 * 43e6};
}
constexpr int kScriptSteps = 4 * 960;

} // namespace ps::test::turbo
