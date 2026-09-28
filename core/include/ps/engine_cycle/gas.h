// engine_cycle/gas.h - two-group ideal-gas properties for the 0D cycle model
// (PHYS-033 P4 I7c C15a, i7c plan section 3 "Gas properties"). INTERNAL,
// header-only.
//
// Two species groups: fresh (air; fuel vapour neglected) and burned
// (stoichiometric products). Each has cv(T) = a + b*T, so the sensible
// internal energy is closed-form. Both u_k are referenced to T_ref =
// 298.15 K (u_k(T_ref) = 0 - PROVISIONAL P2 in out/c15a_progress.md): the
// LHV is defined at that temperature, so converting fresh gas into burned gas
// at T_ref books no spurious energy.
//
// A control volume's state is (m, U, m_b) - PROVISIONAL P1: with linear cv
// the temperature follows from U in closed form (one sqrt), and mass and
// energy become linear invariants of the RK4 integrator.

#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/types.h"

#include <cmath>

namespace ps::engine_cycle {

inline constexpr real kTRef = 298.15;

struct GasGroup {
    real a = 0.0;  // cv = a + b*T  [J/(kg K)]
    real b = 0.0;  // [J/(kg K^2)]
    real r = 0.0;  // gas constant [J/(kg K)]
    real gamma_flow = 1.4;

    [[nodiscard]] real cv(real t) const { return a + b * t; }
    // Sensible internal energy per kg, zero at kTRef.
    [[nodiscard]] real u(real t) const { return a * (t - kTRef) + 0.5 * b * (t * t - kTRef * kTRef); }
    [[nodiscard]] real h(real t) const { return u(t) + r * t; }
};

struct GasModel {
    GasGroup fresh;
    GasGroup burned;

    static GasModel from_desc(const drivetrain::GasPropertiesDesc& d) {
        GasModel g;
        g.fresh = GasGroup{d.fresh.cv_a_j_kgk, d.fresh.cv_b_j_kgk2, d.fresh.r_j_kgk, d.fresh.gamma_flow};
        g.burned = GasGroup{d.burned.cv_a_j_kgk, d.burned.cv_b_j_kgk2, d.burned.r_j_kgk, d.burned.gamma_flow};
        return g;
    }

    // Specific enthalpy of a stream with burned mass fraction x.
    [[nodiscard]] real h_mix(real t, real x) const { return (1.0 - x) * fresh.h(t) + x * burned.h(t); }
    [[nodiscard]] real r_mix(real x) const { return (1.0 - x) * fresh.r + x * burned.r; }
};

// Thermodynamic state of one control volume, recovered from (m, U, m_b).
struct CvState {
    real m = 0.0;
    real mb = 0.0;
    real x = 0.0;   // burned mass fraction mb/m
    real t = 0.0;   // K
    real p = 0.0;   // Pa
    real r = 0.0;   // mixture gas constant
};

// U = A*(T - Tr) + (B/2)*(T^2 - Tr^2) with A = m_u*a_u + m_b*a_b,
// B = m_u*b_u + m_b*b_b. With tau = T - Tr: U = A'*tau + (B/2)*tau^2,
// A' = A + B*Tr, so tau = 2U/(A' + sqrt(A'^2 + 2*B*U)) (cancellation-free;
// B = 0 reduces to U/A').
[[nodiscard]] inline real temperature_from_energy(const GasModel& g, real m, real mb, real u_total) {
    const real mu = m - mb;
    const real a = mu * g.fresh.a + mb * g.burned.a;
    const real b = mu * g.fresh.b + mb * g.burned.b;
    const real ap = a + b * kTRef;
    real disc = ap * ap + 2.0 * b * u_total;
    if (disc < 0.0) disc = 0.0;
    return kTRef + 2.0 * u_total / (ap + std::sqrt(disc));
}

[[nodiscard]] inline real energy_from_temperature(const GasModel& g, real m, real mb, real t) {
    return (m - mb) * g.fresh.u(t) + mb * g.burned.u(t);
}

[[nodiscard]] inline CvState cv_state(const GasModel& g, real m, real u_total, real mb, real volume) {
    CvState s;
    s.m = m;
    s.mb = mb;
    s.x = mb / m;
    s.t = temperature_from_energy(g, m, mb, u_total);
    s.r = g.r_mix(s.x);
    s.p = m * s.r * s.t / volume;
    return s;
}

} // namespace ps::engine_cycle
