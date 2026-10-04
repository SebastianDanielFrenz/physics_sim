// ps/environment/air_temperature.h - diurnal air temperature and humidity
// helpers (weather plan 3.4, W3).
//
// Diurnal temperature: Parton & Logan 1981 [C Parton & Logan 1981, A model
// for diurnal variation in soil and air temperature, Agricultural Meteorology
// 23; M - formula as widely reproduced, the paper itself not re-read]:
//
//   day   (t_n <= t < t_s):  T = T_n + (T_x - T_n) sin(pi m / (Y + 2a)),
//                            m = t - t_n, t_n = sunrise + c (time of the minimum)
//   night (t_s <= t < t_n'): T = (T_n' - T_s e^-b + (T_s - T_n') e^(-b n / Z')) / (1 - e^-b),
//                            n = t - t_s, Z' = t_n' - t_s
//
// Y = day length, Z = night length (the game's own, from the calendar), a/b/c
// from DiurnalTempDesc with a/c scaled by (day + night) / reference_cycle_s
// [AS: the paper's lags are hours of a 24 h day]. T_s = the day formula at
// sunset; T_n' = the NEXT day's minimum, so the curve is continuous [plan 3.4].
// The night form is the paper's exponential decay RESCALED [D] so it reaches
// T_n' exactly at the next day's time of minimum (the unscaled
// T_n + (T_s - T_n) e^(-b n / Z) leaves a jump of (T_s - T_n) e^-b there).
// c is clamped to [-Z/4, +Y/4] so the minimum stays near sunrise on very
// short nights/days.
//
// Per day: T_n = mean - k range / 2, T_x = mean + k range / 2 with the
// calendar's per-day temp_mean_c / temp_range_k and k = 1 - r N(t) (cloud
// reduction r, cloud cover N at the evaluated time; plan 3.4 [AS]).
// A day with zero-length day or night (polar, astronomical day/night form
// only) has no Parton-Logan shape: the day's mean temperature is returned
// (documented discontinuity at the polar edges).
//
// Humidity: Magnus form over water [C Alduchov & Eskridge 1996, J. Appl.
// Meteor. 35, the AERK coefficients 6.1094 hPa, 17.625, 243.04 degC; M].
//
// All transcendentals through ps::math (deterministic, PLAN.md D7).

#pragma once

#include "ps/environment/calendar.h"
#include "ps/environment/weather_desc.h"
#include "ps/types.h"

namespace ps::environment {

// Air temperature in degC at env time t. The calendar's days must carry
// climate scalars (std::invalid_argument otherwise).
real parton_logan_air_temp_c(const Calendar& calendar, const DiurnalTempDesc& p, real t_env, real cloud_cover);

// Key times of the day containing t_env (env seconds): time of the minimum
// (t_n), sunset (t_s) and of the maximum (t_n + (Y + 2a) / 2). For tests and
// diagnostics; valid only for a non-degenerate day.
struct DiurnalTimes {
    real t_min = 0.0;
    real t_max = 0.0;
    real sunrise = 0.0;
    real sunset = 0.0;
};
DiurnalTimes parton_logan_times(const Calendar& calendar, const DiurnalTempDesc& p, real t_env);

real saturation_vapour_pressure_hpa(real temp_c);
// rh in [0, 1], clamped below at 1e-6 (dew point of perfectly dry air is -inf).
real dew_point_c(real temp_c, real rh);

} // namespace ps::environment
