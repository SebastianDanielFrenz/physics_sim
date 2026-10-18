// ps/engine_cycle/knock_model.h - knock model v3 (knock calibration phase B): the Douaud-Eyzat constants and the
// calibrated parameter set the map generator's knock replay uses. Split out of knock.h so CycleModelDesc
// (ps/drivetrain/engine_cycle_desc.h) can carry a KnockModelParams without including the cycle stepper.
//
// The calibrated set is DATA: data/knock_models/douaud_eyzat_v3.json carries the same numbers with their source tags
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

// Calibrated set, knock model v3 (knock calibration phase B, 2026-10-19). Fitted jointly (A scale, n, B) by least
// squares of theta_kl - measured spark over the envelope rows of the Toyota A25A (KV1-BC, 17 rows, NA) and the Honda
// L15B7 on EPA Tier 3 E10 (15 rows, MAP 98-196 kPa), every replay at MEASURED boundary conditions (trapped air =
// measured air, measured IMT/lambda/cam, exhaust reservoir at ambient - the generator's convention), subject to every
// A25A row staying within +-5 deg. The single pre-factor of v2 could not fit the boosted rows (v2 is 10-19 deg too
// knock-prone there); a lower pressure exponent is the smallest extension that does, and B then has to move with it
// (the v2 B of 3800 K was not identifiable from the NA rows alone). Grid: n 0.95-1.40 x B 4000-6000 K, the optimum is
// flat over n 1.0-1.1 / B 5600-5800 (rms 2.90-2.91 deg).
inline constexpr real kKnockCalPrefactorScale = 0.012;   // [M] tau_cal = scale * A at the calibrated n and B
inline constexpr real kKnockCalPressureExponent = 1.05;  // [M] n in (p/1 atm)^-n (literature 1.7)
inline constexpr real kKnockCalActivationK = 5800.0;     // [M] B [K] (literature 3800)
// v2 (P7b): the replay runs at the ECU's lambda target and uses the calibrated constants. v3 (knock calibration phase
// B): calibrated pressure exponent and activation temperature, new pre-factor scale; the parameter values are folded
// into the map cache key next to the version (map_cache.cpp).
inline constexpr std::uint32_t kKnockModelVersion = 3;

// The knock model's parameters as carried by CycleModelDesc::knock (defaults = the calibrated compiled set).
struct KnockModelParams {
    real prefactor_scale = kKnockCalPrefactorScale;
    real pressure_exponent = kKnockCalPressureExponent;
    real activation_k = kKnockCalActivationK;
    real octane_index_k = kOctaneIndexK;
};

} // namespace ps::engine_cycle
