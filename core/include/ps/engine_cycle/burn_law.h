// ps/engine_cycle/burn_law.h - the burn law (design C, owner decision 2026-10-19, vault PHYS-079): the Wiebe burn
// duration and the flame-development (ignition) delay of a cycle follow its own unburned-charge state instead of two
// constant crank angles. Shared by the map generator (CycleStepper's IVC hook), the knock replay (the node's angles) and
// the live engine voice (LiveEngine, at every IVC).
//
// The law, per cycle, from the cylinder state at IVC:
//   unburned state at firing TDC (polytropic compression of the IVC state, exponent n_c):
//       T_u = T_ivc * r^(n_c - 1),  p_u = p_ivc * r^n_c,  r = V_ivc / V_tdc
//   laminar flame speed (Metghalchi & Keck 1982, gasoline-like fuel; phi clamped into the fit's range [phi_min, phi_max])
//   with residual dilution (Rhodes & Keck 1985 form):
//       S_L = (B_m + B_2 (phi - phi_m)^2) (T_u/T_0)^alpha (p_u/p_0)^beta (1 - k_d x_res^e_d)
//       alpha = alpha_0 + alpha_phi (phi - 1),  beta = beta_0 + beta_phi (phi - 1)
//   the pressure enters S_L at an EFFECTIVE pressure p_eff = p_ref (p_u / p_ref)^k_p (k_p = 1: Metghalchi-Keck as
//   measured; k_p = 0, the shipped value [M]: S_L at the reference pressure - the turbulent burn of an engine follows the
//   laminar pressure dependence only weakly (ITU MAK 493E LN07 [C]: higher induction pressure SHORTENS the combustion,
//   faster-burning engines are less sensitive to p and T), and the joint part-load + WOT fit gives k_p ~ 0)
//   burn angles, both scaled by one factor (the development : rapid split stays the cited 15 : 20 deg):
//       f = s (n / n_ref)^e_n (S_L,ref / S_L(phi, T_u, p_eff, x))^e_s (p_u / p_ref)^e_p,
//       S_L,ref = S_L at the reference state (phi 1); e_p (pressure_exponent, phase 3) is the burn's own pressure
//       dependence outside the laminar flame speed - negative: a denser charge burns in fewer crank degrees
//       dtheta_dev = dtheta_dev,ref f            (spark -> 10 % burned)
//       dtheta_rapid = dtheta_rapid,ref f        (10 % -> 90 % burned)
//   mapped onto the Wiebe function (fractions q_x = (-ln(1-x)/a)^(1/(m+1)) of its duration):
//       Wiebe duration  = dtheta_rapid / (q_0.9 - q_0.1)
//       ignition delay  = dtheta_dev - q_0.1 * Wiebe duration        (>= 0, clamped and counted)
//   MBT (unchanged rule, CA50 at ca50_mbt_deg_atdc): Wiebe start = CA50 - q_0.5 * duration, spark = start - delay.
// With the tied exponents the delay is 3.86 f deg and the duration 40.32 f deg; at f = 1 they are the C17a cited
// constants (40.3205 / 3.8632 deg).
//
// Only the spark-to-CA50 angle (dtheta_dev + 0.4866 dtheta_rapid with a 5, m 2) is identifiable from MBT spark data,
// which is why the split is held at the cited ratio. Phase 1b (2026-10-19): e_n is the cited 0.2 [C] (MIT 2.61 lec. 9:
// burn angle ~ rpm^0.2), k_p 0. Phase 3 (2026-10-20): (s, e_s, e_p) fitted [M] on the training part-load MBT rows only;
// the WOT rows (gross indicated work per trapped air at measured spark) validate on the non-scavenging rows - their
// trapped air counts blow-through air wherever MAP exceeds the exhaust pressure (tests/unit/test_burn_law_cal.cpp).
// Every constant is DATA: data/burn_laws/default.json carries the same numbers with their source tags, loaded by
// ps::io::load_burn_law_json; a test pins file == compiled defaults.
//
// Not modelled: the fuel's own flame speed (one gasoline-like coefficient set for every fuel, E85 included), cooled
// EGR (the residual is the cycle's own), turbulence other than through the speed exponent, charge motion devices.
#pragma once

#include "ps/types.h"

#include <cstdint>
#include <string>

namespace ps::engine_cycle {

// Folded into the .psmaps key of every engine on the law (map_cache.cpp), next to every parameter value.
// 2 (phase 1b): S_L at the effective pressure p_eff (flame_speed_pressure_factor).
// 3 (phase 3, 2026-10-20): the burn factor's own pressure term (p_u / p_ref)^e_p (pressure_exponent).
inline constexpr std::uint32_t kBurnLawVersion = 3;

struct BurnLawParams {
    // Reference state of the cited angles [AS]: a typical part-load TDC unburned state (2000 rpm, ~50 kPa MAP, CR 10).
    real reference_rpm = 2000.0;
    real reference_unburned_temperature_k = 700.0;
    real reference_unburned_pressure_pa = 1.0e6;
    real reference_residual_fraction = 0.10;
    // Cited angles at the reference [C]: MIT 2.61 (2017) lecture 9 - spark to 10 % burned ~15 deg, 0-90 % ~35 deg.
    real development_reference_deg = 15.0;
    real rapid_reference_deg = 20.0;
    // Overall scale, flame-speed exponent and pressure exponent fitted [M] (test_burn_law_cal.cpp, phase 3: training
    // part-load MBT rows); the speed exponent cited [C] (MIT 2.61 lec. 9: burn angle ~ rpm^0.2); the pressure factor k_p
    // (see the file comment) [M], held at 0.
    real scale = 1.5633;
    real rpm_exponent = 0.2;
    real flame_speed_exponent = 0.3929;
    real flame_speed_pressure_factor = 0.0;
    // Pressure exponent of the burn factor [M] (phase 3; the sign is ITU MAK 493E LN07's [C]). 0 = the phase-1b law.
    real pressure_exponent = -0.2485;
    // Metghalchi & Keck 1982, indolene [C] (via ITU MAK 493E lecture notes 7, slide 13).
    real mk_phi_m = 1.13;
    real mk_b_m_m_s = 0.2758;
    real mk_b_2_m_s = -0.7834;
    real mk_alpha_0 = 2.18;
    real mk_alpha_phi = -0.8;
    real mk_beta_0 = -0.16;
    real mk_beta_phi = 0.22;
    real mk_t0_k = 298.0;
    real mk_p0_pa = 101325.0;
    // The correlation's equivalence-ratio range [AS, recalled: Metghalchi & Keck measured 0.8-1.5]: phi is clamped into it
    // (beyond 1.5 the quadratic falls to zero near phi 1.72 - a rich WOT row at lambda 0.6 would read a 5x slower flame).
    real mk_phi_min = 0.8;
    real mk_phi_max = 1.5;
    // Residual dilution 1 - k x^e [AS] (Rhodes & Keck 1985 form, recalled, not verified locally).
    real dilution_k = 2.06;
    real dilution_exponent = 0.77;
    // Guards [AS]: lower bounds of the dilution factor and of the flame speed (heavily diluted idle cycles).
    real min_dilution_factor = 0.05;
    real min_flame_speed_m_s = 0.05;
    // Polytropic exponent of the IVC -> TDC compression of the unburned charge [AS].
    real compression_exponent = 1.3;
};

// The cylinder state the law reads (the IVC capture of CycleStepper).
struct BurnLawInput {
    real rpm = 0.0;
    real lambda = 1.0;            // trapped air / (fuel * AFR_st); phi = 1 / lambda
    real t_ivc_k = 0.0;
    real p_ivc_pa = 0.0;
    real residual_fraction = 0.0; // burned mass fraction at IVC
    real volume_ratio = 1.0;      // V_ivc / V_tdc (>= 1)
};

struct BurnAngles {
    real development_deg = 0.0;    // spark -> 10 % burned
    real rapid_deg = 0.0;          // 10 % -> 90 % burned
    real wiebe_duration_deg = 0.0; // the Wiebe dtheta
    real ignition_delay_deg = 0.0; // spark -> Wiebe start
    real flame_speed_m_s = 0.0;    // S_L at the TDC unburned state (at the effective pressure)
    real t_unburned_k = 0.0;
    real p_unburned_pa = 0.0;      // the actual TDC unburned pressure (not p_eff)
    bool delay_clamped = false;    // the development angle was shorter than the Wiebe's own 0-10 % span
};

// The parameters' own checks (empty = valid): every value finite, the angles/reference/guards > 0, exponents >= 0
// where a sign makes no sense, flame_speed_pressure_factor in [0, 1], pressure_exponent in [-1, 1], compression exponent
// in [1, 1.67].
[[nodiscard]] std::string validate_burn_law(const BurnLawParams& p);

// Laminar flame speed [m/s] at (phi, T_u, p_u, x_res), floored at min_flame_speed_m_s.
[[nodiscard]] real laminar_flame_speed(const BurnLawParams& p, real phi, real t_u_k, real p_u_pa, real residual_fraction);

// The law (see the file comment) for a Wiebe with exponents (a, m). Deterministic (ps::math), no allocation.
[[nodiscard]] BurnAngles evaluate_burn_law(const BurnLawParams& p, real wiebe_a, real wiebe_m, const BurnLawInput& in);

// The law's angles at its reference state (f = scale): the stepper's value before the first IVC capture.
[[nodiscard]] BurnAngles reference_burn_angles(const BurnLawParams& p, real wiebe_a, real wiebe_m);

// MBT: the Wiebe start [deg ATDC] that puts CA50 at ca50_deg_atdc, and the spark [deg BTDC] = -(start - delay).
[[nodiscard]] real mbt_wiebe_start_atdc(const BurnAngles& b, real wiebe_a, real wiebe_m, real ca50_deg_atdc);
[[nodiscard]] real mbt_spark_btdc(const BurnAngles& b, real wiebe_a, real wiebe_m, real ca50_deg_atdc);

} // namespace ps::engine_cycle
