// ps/environment/atmosphere.h - the standard-atmosphere altitude model and
// humid-air density (weather plan section 9, W4).
//
// atmosphere_at(sea_level, altitude): ISA lapse rate and barometric formula
// applied to a SEA-LEVEL reference sample (pressure, temperature, vapour
// pressure), so a weather-driven sample (non-standard sea-level pressure and
// temperature) is lifted to any altitude the same way the standard sample is:
//
//   troposphere (h <= 11 km):  T = T0 - L h,  p = p0 (T / T0)^(g0 / (R L))
//   lower stratosphere (11 km < h <= 20 km): isothermal at T11 = T0 - L 11 km,
//                              p = p11 exp(-g0 (h - 11 km) / (R T11))
//
// Constants [C ISO 2533:1975 / ICAO Doc 7488; M - the standard's text itself
// was not re-read, the values are the widely tabulated ones and reproduce the
// published 1 000 / 2 000 / 5 000 / 10 000 / 11 000 m table points to the
// table's own rounding, test_environment_atmosphere.cpp]: g0 = 9.80665 m/s2,
// R = 287.05287 J/(kg K) (dry air), L = 0.0065 K/m, tropopause 11 000 m. The
// model is defined for -5 000 m <= h <= 20 000 m (the ISO 2533 range this
// header implements); an altitude outside is clamped to it (documented
// limit, not an error: a car never leaves it).
//
// Humidity [D]: the sample carries the vapour pressure at the sea-level
// reference; at altitude the vapour MOLE FRACTION is kept (e(h) = e0 p(h)/p0,
// constant mixing ratio) and capped at saturation over water at T(h) (Magnus,
// air_temperature.h). Humid-air density is the ideal-gas mixture
//   rho = (p - e) / (R_d T) + e / (R_v T)
// [C ideal-gas mixture of dry air and water vapour; M], R_v = 461.5228
// J/(kg K) = R* / M_w with R* = 8.314462618 (CODATA 2018) and M_w = 18.01528
// g/mol.
//
// Everything is pure and deterministic: ps::math::pow/exp only (PLAN.md D7).
// No state, no allocation.

#pragma once

#include "ps/types.h"

namespace ps::environment {

inline constexpr real kIsaG0 = 9.80665;                  // m/s2 [C ISO 2533]
inline constexpr real kIsaGasConstantAir = 287.05287;    // J/(kg K), dry air [C ISO 2533]
inline constexpr real kIsaLapseRate = 0.0065;            // K/m, troposphere [C ISO 2533]
inline constexpr real kIsaTropopauseM = 11000.0;         // m [C ISO 2533]
inline constexpr real kIsaMinAltitudeM = -5000.0;        // m, lower limit of the implemented range [C ISO 2533]
inline constexpr real kIsaMaxAltitudeM = 20000.0;        // m, upper limit of the implemented range
inline constexpr real kVapourGasConstant = 461.5228;     // J/(kg K), water vapour [C CODATA 2018 R*/M_w]
inline constexpr real kIsaSeaLevelPressurePa = 101325.0;
inline constexpr real kIsaSeaLevelTemperatureK = 288.15;

// The reference the altitude model lifts: pressure reduced to sea level,
// the temperature the air would have at sea level on the ISA lapse, and the
// vapour pressure at that sea-level state. vapour_pressure_pa 0 = dry air.
struct SeaLevelAtmosphere {
    real pressure_pa = kIsaSeaLevelPressurePa;
    real temperature_k = kIsaSeaLevelTemperatureK;
    real vapour_pressure_pa = 0.0;
};

struct AtmosphereSample {
    real altitude_m = 0.0;          // the (clamped) altitude the sample is for
    real pressure_pa = kIsaSeaLevelPressurePa;
    real temperature_k = kIsaSeaLevelTemperatureK;
    real vapour_pressure_pa = 0.0;
    real density_kg_m3 = 1.225;     // humid-air density
};

// Pressure at altitude for a sea-level (p0, T0): the pure barometric part,
// no humidity. Throws nothing; the inputs are the caller's contract (finite,
// positive) - atmosphere_at validates.
real isa_pressure_pa(real sea_level_pressure_pa, real sea_level_temperature_k, real altitude_m);
real isa_temperature_k(real sea_level_temperature_k, real altitude_m);

// Humid-air density (kg/m3) from total pressure, temperature and vapour
// pressure (e <= p). e = 0 gives dry air.
real humid_air_density(real pressure_pa, real temperature_k, real vapour_pressure_pa);

// The full sample at `altitude_m`. std::invalid_argument for a non-finite or
// non-positive pressure/temperature or a negative/non-finite vapour pressure
// at the reference.
AtmosphereSample atmosphere_at(const SeaLevelAtmosphere& sea_level, real altitude_m);

} // namespace ps::environment
