// ps/engine_cycle/knock.h - engine garage P7 (C17b part 1): the knock-limited spark advance theta_kl of one operating
// point, from a closed-cycle replay of the converged cycle and the Livengood-Wu integral of a Douaud-Eyzat induction time.
//
// Model (every constant is in docs/data_formats/engine.md "Knock limit" with its source tag):
//   * Douaud & Eyzat 1978 (SAE 780080), as reproduced in Heywood 1988 section 9.6.2 - NOT verified locally [AS]:
//         tau [s] = 17.68e-3 * (ON/100)^3.402 * (p/1 atm)^-1.7 * exp(3800/T)
//     ON = the octane index OI = RON - K*S, S = RON - MON, K = 0.5 [AS] (= the anti-knock index (R+M)/2).
//   * Livengood & Wu 1955: autoignition when integral_{IVC}^{t} dt/tau >= 1 [AS: cited by name, not verified locally].
//   * End-gas temperature: isentropic compression of the unburned charge (the IVC mixture: fresh air plus the residual
//     burned fraction, cv = a + b*T per group) from the IVC state along the single-zone pressure,
//         (a_m + r_m)*ln(T/T0) + b_m*(T - T0) = r_m*ln(p/p0),
//     solved by Newton. Wall heat to the end gas is neglected (the single-zone pressure carries the wall loss).
//   * The integral runs until the Wiebe burned fraction reaches kKnockEndBurnFraction (0.90 [AS]: past it the end gas
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
#include "ps/types.h"

#include <cstdint>

namespace ps::engine_cycle {

inline constexpr real kDouaudEyzatA_s = 17.68e-3;    // [AS] Douaud & Eyzat 1978 via Heywood 1988 sec. 9.6.2
inline constexpr real kDouaudEyzatOnExponent = 3.402; // [AS] same
inline constexpr real kDouaudEyzatPExponent = 1.7;   // [AS] same (p in atm)
inline constexpr real kDouaudEyzatB_K = 3800.0;      // [AS] same
inline constexpr real kOctaneIndexK = 0.5;           // [AS] OI = RON - K*(RON - MON); 0.5 = (R+M)/2
inline constexpr real kKnockEndBurnFraction = 0.90;  // [AS] end of the integral (Wiebe x_b)
inline constexpr real kKnockReplayStepDeg = 0.5;     // [AS] replay step; KN-C convergence test vs 0.125 deg
inline constexpr real kKnockSearchBelowMbtDeg = 40.0; // [D] the eta_spark table's most retarded node (map_generator)
inline constexpr real kKnockSearchAboveMbtDeg = 10.0; // [D] the eta_spark table's most advanced node
inline constexpr real kKnockSearchToleranceDeg = 0.05; // [AS] bisection stop
// The knock model's own version, folded into the map cache key of opted-in engines only (bump on any change above).
inline constexpr std::uint32_t kKnockModelVersion = 1;

[[nodiscard]] real octane_index(real ron, real mon);

// 1/tau [1/s] of the Douaud-Eyzat correlation; inv_on_term = (OI/100)^-3.402 / A (precomputed per fuel).
[[nodiscard]] real douaud_eyzat_rate(real inv_on_term, real p_pa, real t_k);
[[nodiscard]] real douaud_eyzat_inv_on_term(real octane_index);

struct KnockReplayInput {
    const CycleStepper* proto = nullptr; // any stepper built from the engine's CycleModelDesc (copied per replay)
    OperatingPoint op;                   // the node's operating point (rpm, cam, lambda)
    PointResult::Ivc ivc;                // the node's converged IVC state
    real ignition_delay_deg = 0.0;       // Wiebe start = spark + delay
    real burn_duration_deg = 0.0;        // the desc's Wiebe duration
    real inv_on_term = 0.0;              // douaud_eyzat_inv_on_term(OI) of the fuel
    real step_deg = kKnockReplayStepDeg;
    real end_burn_fraction = kKnockEndBurnFraction;
    bool stop_at_knock = true;           // false: integrate to the end (diagnostics)
};

struct KnockReplayResult {
    real integral = 0.0;   // Livengood-Wu integral at the end (or where it first reached 1 with stop_at_knock)
    real p_max_pa = 0.0;   // peak single-zone pressure at step ends
    real t_end_gas_k = 0.0; // end-gas temperature at the end of the integral
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
