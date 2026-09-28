// tests/support/cycle_fixtures.h - a GENERIC 2 L-class naturally aspirated I4
// for the 0D cycle-model tests (PHYS-033 P4 I7c C15a). It is NOT the A25A
// (that is C17-pre's cited engine); every geometry value below is ASSUMED,
// chosen to be typical of a 2.0 L 4-valve I4, and the Tier-2 model constants
// are the CycleModelDesc defaults (Heywood 1988 Wiebe/Woschni, JANAF-derived
// gas properties - see ps/drivetrain/engine_cycle_desc.h).

#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/math/transcendental.h"
#include "ps/types.h"

#include <cmath>

namespace ps::test {

// data/fuels/gasoline.json (Heywood 1988 Table 9.7): LHV 44.0 MJ/kg, A/F 14.5.
inline constexpr real kCycleFixtureLhv = 44.0e6;
inline constexpr real kCycleFixtureAfr = 14.5;

// Cosine lift ramp L(x) = L_max*(1 - cos(2*pi*x/span))/2 at 5 deg spacing
// (plus the end point) [ASSUMED shape: smooth, zero slope at both ends].
inline vehicle::Curve1D cosine_lift_profile(real span_deg, real max_lift_m, real spacing_deg = 5.0) {
    vehicle::Curve1D c;
    const real two_pi = 6.283185307179586476925;
    const int n = static_cast<int>(std::floor(span_deg / spacing_deg));
    for (int i = 0; i <= n; ++i) {
        const real x = spacing_deg * static_cast<real>(i);
        c.x.push_back(x);
        c.y.push_back(0.5 * max_lift_m * (1.0 - ps::math::cos(two_pi * x / span_deg)));
    }
    if (c.x.back() < span_deg) {
        c.x.push_back(span_deg);
        c.y.push_back(0.0);
    }
    c.y.front() = 0.0;
    c.y.back() = 0.0;
    return c;
}

// Flat-top lift with cosine ramps of `ramp_deg` at both ends [ASSUMED
// shape], sampled every `spacing_deg`. Used by the LIVC fixture so the valve
// closes within a few degrees and "trapped volume at IVC" is well defined.
inline vehicle::Curve1D ramped_lift_profile(real span_deg, real ramp_deg, real max_lift_m, real spacing_deg = 1.0) {
    vehicle::Curve1D c;
    const real pi = 3.14159265358979323846;
    const int n = static_cast<int>(std::floor(span_deg / spacing_deg));
    for (int i = 0; i <= n; ++i) {
        const real x = spacing_deg * static_cast<real>(i);
        const real edge = x < span_deg - x ? x : span_deg - x;
        const real l = edge >= ramp_deg ? max_lift_m : 0.5 * max_lift_m * (1.0 - ps::math::cos(pi * edge / ramp_deg));
        c.x.push_back(x);
        c.y.push_back(l);
    }
    if (c.x.back() < span_deg) {
        c.x.push_back(span_deg);
        c.y.push_back(0.0);
    }
    c.y.front() = 0.0;
    c.y.back() = 0.0;
    return c;
}

// Generic 2.0 L I4 (86 x 86 mm, rod 145 mm, CR 10.5) [ASSUMED], 4 valves per
// cylinder (intake 34 mm, exhaust 29 mm, 9 mm lift, 240 deg duration at zero
// lift), IVO 10 BTDC / IVC 50 ABDC, EVO 50 BBDC / EVC 10 ATDC (20 deg overlap)
// [ASSUMED]. Runner 350 x 40 mm, ports 150/250 cm^3 [ASSUMED].
inline drivetrain::CycleModelDesc make_generic_i4_cycle_desc() {
    drivetrain::CycleModelDesc d;
    d.bore_m = 0.086;
    d.stroke_m = 0.086;
    d.rod_m = 0.145;
    d.pin_offset_m = 0.0;
    d.compression_ratio = 10.5;
    d.cylinders = 4;
    d.cycle = 4;
    d.intake_lift.lift_m_vs_deg = cosine_lift_profile(240.0, 0.009);
    d.intake_lift.phase_ref_deg = -10.0;   // IVO at 710 (10 BTDC), IVC at 230
    d.exhaust_lift.lift_m_vs_deg = cosine_lift_profile(240.0, 0.009);
    d.exhaust_lift.phase_ref_deg = 490.0;  // EVO at 490 (50 BBDC), EVC at 730 = 10 ATDC
    d.intake_valves = 2;
    d.exhaust_valves = 2;
    d.intake_valve_d_m = 0.034;
    d.exhaust_valve_d_m = 0.029;
    d.runner_length_m = 0.35;
    d.runner_diameter_m = 0.040;
    d.intake_port_m3 = 1.5e-4;
    d.exhaust_port_m3 = 2.5e-4;
    // Discharge coefficient on the curtain area [ASSUMED shape, typical of
    // steady-flow-bench curves: ~0.65 at low lift falling to ~0.45 at L/D 0.3].
    d.cd_vs_l_over_d.x = {0.0, 0.1, 0.2, 0.3};
    d.cd_vs_l_over_d.y = {0.65, 0.62, 0.55, 0.45};
    return d;
}

// LIVC variant for G11 [ASSUMED]: no overlap, IVO at 0 / IVC at 260
// (80 ABDC), EVO at 460 = 720 - IVC / EVC at 720 (symmetric), flat-top
// lift with 15 deg cosine ramps. (With the slow cosine closing of the base
// profile the valve throttles the push-back for the last ~15 deg even at
// 200 rpm - measured VE 0.752 vs 0.660 - so "IVC" would not be sharp.)
inline drivetrain::CycleModelDesc make_livc_cycle_desc() {
    drivetrain::CycleModelDesc d = make_generic_i4_cycle_desc();
    d.intake_lift.lift_m_vs_deg = ramped_lift_profile(260.0, 15.0, 0.009);
    d.intake_lift.phase_ref_deg = 0.0;
    d.exhaust_lift.lift_m_vs_deg = ramped_lift_profile(260.0, 15.0, 0.009);
    d.exhaust_lift.phase_ref_deg = 460.0;
    return d;
}

// Spark (deg ATDC, i.e. Wiebe theta_0 - 360) that puts CA50 at the desc's
// MBT CA50 (plan D11).
inline real mbt_spark_deg_atdc(const drivetrain::CycleModelDesc& d) {
    return d.ca50_mbt_deg_atdc -
           d.burn_duration_deg * ps::math::pow(ps::math::log(2.0) / d.wiebe_a, 1.0 / (d.wiebe_m + 1.0));
}

} // namespace ps::test
