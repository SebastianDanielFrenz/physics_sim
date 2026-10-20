// core/src/engine_cycle/burn_law.cpp - see ps/engine_cycle/burn_law.h.
#include "ps/engine_cycle/burn_law.h"

#include "ps/engine_cycle/wiebe.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace ps::engine_cycle {

namespace {

// Fraction x of the Wiebe duration at which x is burned: (-ln(1-x)/a)^(1/(m+1)).
real wiebe_quantile(real a, real m, real x) {
    return ps::math::pow(-ps::math::log(real(1.0) - x) / a, real(1.0) / (m + real(1.0)));
}

bool finite(real v) { return std::isfinite(v); }

} // namespace

std::string validate_burn_law(const BurnLawParams& p) {
    struct Item {
        const char* name;
        real v;
        bool positive; // > 0 (else >= 0)
    };
    const Item items[] = {
        {"reference_rpm", p.reference_rpm, true},
        {"reference_unburned_temperature_k", p.reference_unburned_temperature_k, true},
        {"reference_unburned_pressure_pa", p.reference_unburned_pressure_pa, true},
        {"reference_residual_fraction", p.reference_residual_fraction, false},
        {"development_reference_deg", p.development_reference_deg, true},
        {"rapid_reference_deg", p.rapid_reference_deg, true},
        {"scale", p.scale, true},
        {"rpm_exponent", p.rpm_exponent, false},
        {"flame_speed_exponent", p.flame_speed_exponent, false},
        {"flame_speed_pressure_factor", p.flame_speed_pressure_factor, false},
        {"mk_phi_m", p.mk_phi_m, true},
        {"mk_b_m_m_s", p.mk_b_m_m_s, true},
        {"mk_t0_k", p.mk_t0_k, true},
        {"mk_p0_pa", p.mk_p0_pa, true},
        {"mk_phi_min", p.mk_phi_min, true},
        {"mk_phi_max", p.mk_phi_max, true},
        {"dilution_k", p.dilution_k, false},
        {"dilution_exponent", p.dilution_exponent, true},
        {"min_dilution_factor", p.min_dilution_factor, true},
        {"min_flame_speed_m_s", p.min_flame_speed_m_s, true},
        {"compression_exponent", p.compression_exponent, true},
    };
    for (const Item& it : items) {
        if (!finite(it.v)) return std::string(it.name) + " must be finite";
        if (it.positive ? !(it.v > real(0.0)) : !(it.v >= real(0.0))) {
            return std::string(it.name) + (it.positive ? " must be > 0" : " must be >= 0");
        }
    }
    for (real v : {p.mk_b_2_m_s, p.mk_alpha_0, p.mk_alpha_phi, p.mk_beta_0, p.mk_beta_phi}) {
        if (!finite(v)) return "Metghalchi-Keck coefficients must be finite";
    }
    if (!(p.reference_residual_fraction < real(1.0))) return "reference_residual_fraction must be < 1";
    if (!(p.flame_speed_pressure_factor <= real(1.0))) return "flame_speed_pressure_factor must be <= 1";
    if (!finite(p.pressure_exponent)) return "pressure_exponent must be finite";
    if (!(p.pressure_exponent >= real(-1.0) && p.pressure_exponent <= real(1.0))) {
        return "pressure_exponent must be in [-1, 1]";
    }
    if (!(p.mk_phi_max > p.mk_phi_min)) return "mk_phi_max must be > mk_phi_min";
    if (!(p.min_dilution_factor <= real(1.0))) return "min_dilution_factor must be <= 1";
    if (!(p.compression_exponent >= real(1.0) && p.compression_exponent <= real(1.67))) {
        return "compression_exponent must be in [1, 1.67]";
    }
    return {};
}

real laminar_flame_speed(const BurnLawParams& p, real phi_in, real t_u_k, real p_u_pa, real residual_fraction) {
    const real phi = std::min(p.mk_phi_max, std::max(p.mk_phi_min, phi_in));
    const real dphi = phi - p.mk_phi_m;
    const real s0 = p.mk_b_m_m_s + p.mk_b_2_m_s * dphi * dphi;
    const real alpha = p.mk_alpha_0 + p.mk_alpha_phi * (phi - real(1.0));
    const real beta = p.mk_beta_0 + p.mk_beta_phi * (phi - real(1.0));
    const real x = std::max(real(0.0), residual_fraction);
    const real dil = std::max(p.min_dilution_factor, real(1.0) - p.dilution_k * ps::math::pow(x, p.dilution_exponent));
    const real s = s0 * ps::math::pow(t_u_k / p.mk_t0_k, alpha) * ps::math::pow(p_u_pa / p.mk_p0_pa, beta) * dil;
    return std::max(p.min_flame_speed_m_s, s);
}

namespace {

BurnAngles angles_from_factor(const BurnLawParams& p, real a, real m, real f) {
    BurnAngles b;
    b.development_deg = p.development_reference_deg * f;
    b.rapid_deg = p.rapid_reference_deg * f;
    const real q10 = wiebe_quantile(a, m, real(0.1));
    const real q90 = wiebe_quantile(a, m, real(0.9));
    b.wiebe_duration_deg = b.rapid_deg / (q90 - q10);
    const real delay = b.development_deg - q10 * b.wiebe_duration_deg;
    b.delay_clamped = delay < real(0.0);
    b.ignition_delay_deg = b.delay_clamped ? real(0.0) : delay;
    return b;
}

} // namespace

BurnAngles evaluate_burn_law(const BurnLawParams& p, real a, real m, const BurnLawInput& in) {
    const real r = std::max(real(1.0), in.volume_ratio);
    const real t_u = in.t_ivc_k * ps::math::pow(r, p.compression_exponent - real(1.0));
    const real p_u = in.p_ivc_pa * ps::math::pow(r, p.compression_exponent);
    const real phi = real(1.0) / in.lambda;
    // The effective pressure of S_L (see the header): exactly p_ref at k_p = 0, exactly p_u at k_p = 1.
    const real k_p = p.flame_speed_pressure_factor;
    const real p_ref = p.reference_unburned_pressure_pa;
    const real p_eff = k_p == real(0.0)   ? p_ref
                       : k_p == real(1.0) ? p_u
                                          : p_ref * ps::math::pow(p_u / p_ref, k_p);
    const real s_l = laminar_flame_speed(p, phi, t_u, p_eff, in.residual_fraction);
    const real s_ref = laminar_flame_speed(p, real(1.0), p.reference_unburned_temperature_k, p.reference_unburned_pressure_pa,
                                           p.reference_residual_fraction);
    real f = p.scale * ps::math::pow(std::max(in.rpm, real(1.0)) / p.reference_rpm, p.rpm_exponent) *
             ps::math::pow(s_ref / s_l, p.flame_speed_exponent);
    // The burn's own pressure term (phase 3); skipped at e_p = 0 so the phase-1b law stays bit-identical.
    if (p.pressure_exponent != real(0.0)) f *= ps::math::pow(p_u / p_ref, p.pressure_exponent);
    BurnAngles b = angles_from_factor(p, a, m, f);
    b.flame_speed_m_s = s_l;
    b.t_unburned_k = t_u;
    b.p_unburned_pa = p_u;
    return b;
}

BurnAngles reference_burn_angles(const BurnLawParams& p, real a, real m) {
    BurnAngles b = angles_from_factor(p, a, m, p.scale);
    b.flame_speed_m_s = laminar_flame_speed(p, real(1.0), p.reference_unburned_temperature_k, p.reference_unburned_pressure_pa,
                                            p.reference_residual_fraction);
    b.t_unburned_k = p.reference_unburned_temperature_k;
    b.p_unburned_pa = p.reference_unburned_pressure_pa;
    return b;
}

real mbt_wiebe_start_atdc(const BurnAngles& b, real a, real m, real ca50_deg_atdc) {
    return Wiebe::theta0_for_ca50(a, m, b.wiebe_duration_deg, ca50_deg_atdc);
}

real mbt_spark_btdc(const BurnAngles& b, real a, real m, real ca50_deg_atdc) {
    return -(mbt_wiebe_start_atdc(b, a, m, ca50_deg_atdc) - b.ignition_delay_deg);
}

} // namespace ps::engine_cycle
