// ps/engine_cycle/knock_model.h - knock model v4 (knock calibration phase B): the Douaud-Eyzat constants and the
// calibrated parameter set the map generator's knock replay uses. Split out of knock.h so CycleModelDesc
// (ps/drivetrain/engine_cycle_desc.h) can carry a KnockModelParams without including the cycle stepper.
//
// The calibrated set is DATA: data/knock_models/douaud_eyzat_v4.json carries the same numbers with their source tags
// and the fit record, loaded by ps::io::load_knock_model_json; tests/unit/test_knock_cal_turbo.cpp pins file ==
// compiled defaults (the limit-policy L0 pattern). Decision and fit: docs/data_formats/engine.md "Knock limit", vault
// PHYS-069.
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

// Calibrated set, knock model v4 (knock calibration phase B, 2026-10-19). Fitted (A scale, n, B, gamma) by least
// squares of theta_kl - measured spark over the envelope rows of the Toyota A25A (KV1-BC, 17 rows, NA) and the Honda
// L15B7 on EPA Tier 3 E10 (15 rows, MAP 98-196 kPa), every replay at MEASURED boundary conditions (trapped air =
// measured air, measured IMT/lambda/cam, exhaust reservoir at ambient - the generator's convention), subject to
//   * every A25A row within +-5 deg (KV1-BC), and
//   * the light-load no-knock rows of every EPA package (MAP <= 60 kPa, lambda 0.95-1.05, BMEP >= 0.3 bar, 433 rows):
//     the model's knock limit may lie at most 2 deg [AS] below min(measured spark, MBT) - the engine ran there without
//     knock (one-sided; nothing is fitted to those rows' spark).
// The single pre-factor of v2 cannot fit the boosted rows (10-19 deg too knock-prone); a lower pressure exponent fits
// them (v3: n 1.05, B 5800 K) but makes diluted light-load points knock-limited (Ford 2.7 part load 22 deg, the demo I4
// at idle 30 deg below MBT). The residual-dilution factor is the smallest extension that keeps both: the fit set alone
// cannot identify gamma (its residual fractions are 0.03-0.10), the light-load constraint sets it.
// Grid: gamma 0-8, n 0.80-1.20, B 5000-7400 K; constrained optimum flat over gamma 5-6 / n 0.93-0.98 / B 6000-6200 K.
inline constexpr real kKnockCalPrefactorScale = 0.004938; // [M] tau_cal = scale * A at the calibrated n, B and gamma
inline constexpr real kKnockCalPressureExponent = 0.95;   // [M] n in (p/1 atm)^-n (literature 1.7)
inline constexpr real kKnockCalActivationK = 6100.0;      // [M] B [K] (literature 3800)
// Residual dilution: the ignition rate is multiplied by (1 - x_res)^gamma, x_res = the end gas's residual (burned) mass
// fraction at IVC (constant over the replay). 0 = no dilution term (Douaud-Eyzat as published). An effective exponent:
// it also absorbs how the model's residual heats the end gas, not a chemical constant.
inline constexpr real kKnockCalDilutionExponent = 5.5;    // [M]
// v2 (P7b): the replay runs at the ECU's lambda target and uses the calibrated constants. v3: calibrated pressure
// exponent and activation temperature. v4 (knock calibration phase B): the residual-dilution factor and the refit set;
// the parameter values are folded into the map cache key next to the version (map_cache.cpp).
inline constexpr std::uint32_t kKnockModelVersion = 4;

// The knock model's parameters as carried by CycleModelDesc::knock (defaults = the calibrated compiled set).
struct KnockModelParams {
    real prefactor_scale = kKnockCalPrefactorScale;
    real pressure_exponent = kKnockCalPressureExponent;
    real activation_k = kKnockCalActivationK;
    real octane_index_k = kOctaneIndexK;
    real dilution_exponent = kKnockCalDilutionExponent;
};

} // namespace ps::engine_cycle
