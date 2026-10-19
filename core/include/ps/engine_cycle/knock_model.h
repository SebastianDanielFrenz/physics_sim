// ps/engine_cycle/knock_model.h - knock model v5 (burn law phase 2a; v4 knock calibration phase B): the Douaud-Eyzat
// constants and the calibrated parameter set the map generator's knock replay uses. Split out of knock.h so
// CycleModelDesc (ps/drivetrain/engine_cycle_desc.h) can carry a KnockModelParams without including the cycle stepper.
//
// The calibrated set is DATA: data/knock_models/douaud_eyzat_v5.json carries the same numbers with their source tags
// and the fit record, loaded by ps::io::load_knock_model_json; tests/unit/test_knock_cal_turbo.cpp pins file ==
// compiled defaults (the limit-policy L0 pattern). Decision and fit: docs/data_formats/engine.md "Knock limit", vault
// PHYS-069 (v4) and PHYS-079 (v5, burn law).
#pragma once

#include "ps/types.h"

#include <cstdint>

namespace ps::engine_cycle {

// Literature correlation (Douaud & Eyzat 1978 via Heywood 1988 sec. 9.6.2, not verified locally):
//   tau [s] = A * (OI/100)^3.402 * (p/1 atm)^-n * exp(B/T)
inline constexpr real kDouaudEyzatA_s = 17.68e-3;    // [AS] Douaud & Eyzat 1978 via Heywood 1988 sec. 9.6.2
inline constexpr real kDouaudEyzatOnExponent = 3.402; // [AS] same
inline constexpr real kDouaudEyzatPExponent = 1.7;   // [AS] same (p in atm)
inline constexpr real kDouaudEyzatB_K = 3800.0;      // [AS] same
inline constexpr real kOctaneIndexK = 0.5;           // [AS] OI = RON - K*(RON - MON); 0.5 = (R+M)/2

// Calibrated set, knock model v5 (burn law phase 2a, 2026-10-19). Fitted (A scale, n, B, gamma, end fraction) by least
// squares of theta_kl - measured spark over the envelope rows of the Toyota A25A (KV1-BC, 17 rows, NA) and the Honda
// L15B7 on EPA Tier 3 E10 (15 rows, MAP 98-196 kPa), every replay at MEASURED boundary conditions (trapped air =
// measured air, measured IMT/lambda/cam, exhaust reservoir at ambient - the generator's convention) and at the burn
// law's own per-cycle burn angles, subject to
//   * every A25A row within +-7.5 deg (KV1-BC; v4 +-5), and
//   * the light-load no-knock rows of every EPA package (MAP <= 60 kPa, lambda 0.95-1.05, BMEP >= 0.3 bar, 433 rows):
//     the model's knock limit may lie at most 3 deg [AS] (v4 2) below min(measured spark, MBT) - the engine ran there
//     without knock (one-sided; nothing is fitted to those rows' spark).
// v4 history: the single pre-factor of v2 could not fit the boosted rows; a lower pressure exponent fit them (v3) but
// made diluted light-load points knock-limited; the residual-dilution factor kept both. v5: the burn law (PHYS-079)
// lengthens the boosted rows' burns (the end gas is exposed longer), which reads the v4 set 7-18 deg too knock-prone
// there while the light-load rows (shorter law burns) and the A25A anchor pull the other way; no v4-shaped cell meets
// the v4 bands. One new parameter: the end of the Livengood-Wu integral (Wiebe x_b, 0.90 [AS] up to v4) - fitted 0.75.
// The boosted residual is MAP-structured (Ford 2.7 135 kPa -4, 150 kPa -7, 175-200 kPa -12 deg at the best v4-shaped
// cell); the IVC temperature it would need is below the measured intake temperature, so it is not a charge temperature
// error. Bands widened 1.5x (declared before the final run). Grids: gamma 0-8, n 1.0-2.2, B 4600-8600 K, x_b 0.5-0.9;
// the optimum lies on a flat n-B ridge, the shipped point is a coordinate-descent minimum at the KCB neighbour steps
// (gamma 1, n 0.05, B 200 K, x_b 0.05; "KCB diag: v5 coordinate descent").
inline constexpr real kKnockCalPrefactorScale = 0.006095; // [M] tau_cal = scale * A at the calibrated n, B and gamma (v4 0.004938)
inline constexpr real kKnockCalPressureExponent = 1.2;    // [M] n in (p/1 atm)^-n (literature 1.7, v4 0.95)
inline constexpr real kKnockCalActivationK = 6400.0;      // [M] B [K] (literature 3800, v4 6100)
// Residual dilution: the ignition rate is multiplied by (1 - x_res)^gamma, x_res = the end gas's residual (burned) mass
// fraction at IVC (constant over the replay). 0 = no dilution term (Douaud-Eyzat as published). An effective exponent:
// it also absorbs how the model's residual heats the end gas, not a chemical constant.
inline constexpr real kKnockCalDilutionExponent = 4.0;    // [M] (v4 5.5)
// End of the Livengood-Wu integral (Wiebe burned fraction; EVO if earlier). An effective value: it also absorbs how the
// single-zone pressure over-states the late end gas's exposure once most of the charge has burned (v4: 0.90 [AS]).
inline constexpr real kKnockCalEndBurnFraction = 0.75;    // [M]
// v2 (P7b): the replay runs at the ECU's lambda target and uses the calibrated constants. v3: calibrated pressure
// exponent and activation temperature. v4 (knock calibration phase B): the residual-dilution factor and the refit set.
// v5 (burn law phase 2a): the end fraction joins the calibrated set and the whole set is refitted under the burn law;
// the parameter values are folded into the map cache key next to the version (map_cache.cpp).
inline constexpr std::uint32_t kKnockModelVersion = 5;

// The knock model's parameters as carried by CycleModelDesc::knock (defaults = the calibrated compiled set).
struct KnockModelParams {
    real prefactor_scale = kKnockCalPrefactorScale;
    real pressure_exponent = kKnockCalPressureExponent;
    real activation_k = kKnockCalActivationK;
    real octane_index_k = kOctaneIndexK;
    real dilution_exponent = kKnockCalDilutionExponent;
    real end_burn_fraction = kKnockCalEndBurnFraction; // v5: end of the Livengood-Wu integral (Wiebe x_b)
};

} // namespace ps::engine_cycle
