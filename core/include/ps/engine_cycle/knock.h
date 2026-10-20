// ps/engine_cycle/knock.h - engine garage P7 (C17b part 1): the knock-limited spark advance theta_kl of one operating
// point, from a closed-cycle replay of the converged cycle and the Livengood-Wu integral of a Douaud-Eyzat induction time.
//
// Model (every constant is in docs/data_formats/engine.md "Knock limit" with its source tag):
//   * Douaud & Eyzat 1978 (SAE 780080), as reproduced in Heywood 1988 section 9.6.2 - NOT verified locally [AS]:
//         tau [s] = 17.68e-3 * (ON/100)^3.402 * (p/1 atm)^-1.7 * exp(3800/T)  (literature; v3 calibrates n, B and A - knock_model.h)
//     ON = the octane index OI = RON - K*S, S = RON - MON, K = 0.5 [AS] (= the anti-knock index (R+M)/2).
//   * Livengood & Wu 1955: autoignition when integral_{IVC}^{t} dt/tau >= 1 [AS: cited by name, not verified locally].
//   * End-gas temperature: isentropic compression of the unburned charge from the IVC state along the single-zone
//     pressure, cv = a + b*T per group,
//         (a_m + r_m)*ln(T/T0) + b_m*(T - T0) = r_m*ln(p/p0),
//     solved by Newton. Wall heat to the end gas is neglected (the single-zone pressure carries the wall loss). The
//     isentrope starts at T_ivc - k_h (T_ivc - T_im) (knock model v6, KnockModelParams::charge_heat_fraction).
//     The charge is fresh air, the residual burned fraction AND the fuel vapour (KnockReplayInput::fuel_mass_fraction):
//     the cycle model's fresh gas is air only (gas.h, "fuel vapour neglected"), which is harmless for its work but not
//     for the end gas - iso-octane vapour at a lambda-1 mass fraction of 1/(1+AFR) raises the unburned cv by ~16 % at
//     600 K (gamma ~1.31 instead of ~1.38) and lowers the end-gas temperature at 45 bar by ~100 K. Vapour cv is a
//     linear fit through two iso-octane ideal-gas cp values (kFuelVapour*, [AS] recalled, not verified locally); the
//     generator uses the generating fuel's AFR for every listed fuel (only the octane differs between the tables).
//   * The integral runs until the Wiebe burned fraction reaches KnockModelParams::end_burn_fraction (knock model v6 0.90 [M], v5 0.75 [M];
//     0.90 [AS] up to v4, on the argument that past it the end gas
//     left is too small to matter) or EVO, whichever comes first; trapezoidal on a fixed kKnockReplayStepDeg grid.
//   * theta_kl = the largest advance [deg BTDC] whose integral stays < 1, by bisection between the eta_spark table's own
//     edges theta_MBT - 40 and theta_MBT + 10 (outside them the runtime has no efficiency data).
//
// The replay does not rerun gas exchange: it copies the converged IVC state (PointResult::ivc), switches the valves off
// and steps the closed cycle with the trial spark - the trapped charge and residual are those of the node's own spark
// (the change of the residual with spark is neglected [AS]). Allocation-free; any thread.
#pragma once

#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/knock_model.h"
#include "ps/types.h"

#include <cstdint>

namespace ps::engine_cycle {

// End of the integral (Wiebe x_b): the calibrated knock model value (knock_model.h; v6 0.90 [M], v5 0.75 [M], 0.90 [AS] up to v4).
inline constexpr real kKnockEndBurnFraction = kKnockCalEndBurnFraction;
inline constexpr real kKnockReplayStepDeg = 0.5;     // [AS] replay step; KN-C convergence test vs 0.125 deg
inline constexpr real kKnockSearchBelowMbtDeg = 40.0; // [D] the eta_spark table's most retarded node (map_generator)
inline constexpr real kKnockSearchAboveMbtDeg = 10.0; // [D] the eta_spark table's most advanced node
inline constexpr real kKnockSearchToleranceDeg = 0.05; // [AS] bisection stop
// kKnockModelVersion (knock_model.h) is folded into the map cache key of opted-in engines only: bump it on any change
// to the replay above.
// Fuel vapour in the end gas (iso-octane as the surrogate, like the burned-gas fit in engine_cycle_desc.h):
// cp(298.15 K) = 188.9 J/(mol K), cp(900 K) = 425 J/(mol K) [AS, recalled NIST WebBook / TRC ideal-gas values, not
// verified locally]; M = 114.23 g/mol; cv = cp - R; linear fit cv = a + b*T through the two points [D].
inline constexpr real kFuelVapourR_JkgK = 8314.462618 / 114.23;                                                  // [D]
inline constexpr real kFuelVapourCvB_JkgK2 = ((425.0 - 188.9) / 0.11423) / (900.0 - 298.15);                     // [D]
inline constexpr real kFuelVapourCvA_JkgK = 188.9 / 0.11423 - kFuelVapourR_JkgK - kFuelVapourCvB_JkgK2 * 298.15; // [D]

// The literature and calibrated Douaud-Eyzat constants and kKnockModelVersion live in ps/engine_cycle/knock_model.h.

[[nodiscard]] real octane_index(real ron, real mon, real k = kOctaneIndexK);

// 1/tau [1/s] of the Douaud-Eyzat correlation; inv_on_term = (OI/100)^-3.402 / A (precomputed per fuel), b_k the
// activation temperature.
[[nodiscard]] real douaud_eyzat_rate(real inv_on_term, real p_pa, real t_k, real b_k = kKnockCalActivationK,
                                     real p_exponent = kKnockCalPressureExponent);
// (OI/100)^-3.402 / (prefactor_s): prefactor_s defaults to the calibrated pre-factor A * kKnockCalPrefactorScale; pass
// kDouaudEyzatA_s for the literature correlation.
[[nodiscard]] real douaud_eyzat_inv_on_term(real octane_index, real prefactor_s = kDouaudEyzatA_s * kKnockCalPrefactorScale);

struct KnockReplayInput {
    const CycleStepper* proto = nullptr; // any stepper built from the engine's CycleModelDesc (copied per replay)
    OperatingPoint op;                   // the node's operating point (rpm, cam, lambda)
    PointResult::Ivc ivc;                // the node's converged IVC state
    real ignition_delay_deg = 0.0;       // Wiebe start = spark + delay
    real burn_duration_deg = 0.0;        // the desc's Wiebe duration
    real inv_on_term = 0.0;              // douaud_eyzat_inv_on_term(OI) of the fuel
    real fuel_mass_fraction = 0.0;       // fuel vapour share of the fresh charge, 1/(1 + lambda*AFR); 0 = air only
    real step_deg = kKnockReplayStepDeg;
    real end_burn_fraction = kKnockEndBurnFraction;
    real activation_k = kKnockCalActivationK; // Douaud-Eyzat B [K]
    real p_exponent = kKnockCalPressureExponent; // Douaud-Eyzat pressure exponent n in (p/1 atm)^-n
    real dilution_exponent = kKnockCalDilutionExponent; // rate x (1 - x_res)^gamma, x_res = end-gas residual fraction
    bool stop_at_knock = true;           // false: integrate to the end (diagnostics)
    // Diagnostic levers (P7b budget; the generator leaves both at their defaults):
    real end_gas_t0_offset_k = 0.0;      // added to the end gas's IVC temperature (e.g. a DI charge-cooling estimate)
    real vapour_cv_scale = 1.0;          // multiplies the fuel-vapour cv fit (sensitivity of the [AS] iso-octane fit)
    // true: the end gas follows the single-zone (bulk) temperature, which carries the Woschni wall heat, up to the Wiebe
    // start - shifted by the fuel-vapour difference of the two isentropes - and is isentropic only after it; false
    // (default): isentropic from IVC (an adiabatic core).
    bool wall_heat_end_gas = false;
    // Knock model v6: the end gas starts at T_ivc - k*(T_ivc - op.t_im_k) (k = this, only where T_ivc > t_im_k > 0) - the
    // share of the charge's heating above the intake temperature the end gas does not carry; 0 = the v5 replay.
    real charge_heat_fraction = kKnockCalChargeHeatFraction;
};

struct KnockReplayResult {
    real integral = 0.0;   // Livengood-Wu integral at the end (or where it first reached 1 with stop_at_knock)
    real p_max_pa = 0.0;   // peak single-zone pressure at step ends
    real t_end_gas_k = 0.0; // end-gas temperature at the end of the integral
    real p_end_pa = 0.0;    // single-zone pressure there
    bool knock = false;    // integral >= 1
};

// One closed-cycle replay at spark advance spark_btdc [deg BTDC].
[[nodiscard]] KnockReplayResult knock_replay(const KnockReplayInput& in, real spark_btdc);

struct KnockLimitResult {
    real theta_kl_btdc = 0.0; // knock-safe advance minus the margin where limited; the upper bound where not
    bool limited = false;     // the upper bound knocks
    bool saturated = false;   // even the lower bound knocks (theta_kl = lower bound - margin)
    int replays = 0;
};

// The knock-limited advance in [mbt_btdc - 40, mbt_btdc + 10] (bisection to kKnockSearchToleranceDeg, keeping the
// knock-safe end); margin_deg is subtracted only at a limited node; the result is clamped to >= -30 (the ECU range).
[[nodiscard]] KnockLimitResult knock_limited_spark(const KnockReplayInput& in, real mbt_btdc, real margin_deg);

} // namespace ps::engine_cycle
