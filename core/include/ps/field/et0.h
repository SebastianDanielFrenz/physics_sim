// ps/field/et0.h - FAO-56 Penman-Monteith reference evapotranspiration for a
// daily time step (Allen et al. 1998, FAO Irrigation and Drainage Paper 56,
// chapters 3 and 4). Pure functions of their arguments; deterministic math
// only (ps::math), so the same inputs give the same bits on every platform.
//
// Equation numbers in the comments are FAO-56's. The unit tests reproduce the
// printed worked examples 17 (monthly, Bangkok) and 18 (daily, Uccle).

#pragma once

#include "ps/types.h"

namespace ps::field {

struct Et0Input {
    real tmin_c = 0.0;
    real tmax_c = 0.0;
    real ea_kpa = 0.0;          // actual vapour pressure
    real u2_m_s = 2.0;          // wind speed at 2 m
    real rs_mj_m2 = 0.0;        // incoming solar (shortwave) radiation, MJ m-2 day-1
    real latitude_deg = 0.0;
    real elevation_m = 0.0;
    int day_of_year = 1;        // J, 1..366
    real g_mj_m2 = 0.0;         // soil heat flux, 0 for a daily step
};

// Every intermediate FAO-56 prints, so a test can compare each one.
struct Et0Result {
    real tmean_c = 0.0;
    real pressure_kpa = 0.0;    // P, eq 7
    real gamma = 0.0;           // psychrometric constant, eq 8
    real delta = 0.0;           // slope of the saturation curve, eq 13
    real es_kpa = 0.0;          // eq 12
    real ea_kpa = 0.0;
    real ra = 0.0;              // extraterrestrial radiation, eq 21
    real daylight_h = 0.0;      // N, eq 34
    real rso = 0.0;             // clear-sky radiation, eq 37
    real rns = 0.0;             // eq 38
    real rnl = 0.0;             // eq 39
    real rn = 0.0;              // eq 40
    real radiation_term = 0.0;  // mm day-1
    real aero_term = 0.0;       // mm day-1
    real et0_mm = 0.0;          // eq 6
};

// FAO-56 constants. They are equation constants of the published method, not
// tunable limits.
Et0Result fao56_et0(const Et0Input& in);

// Saturation vapour pressure at air temperature T (kPa), eq 11.
real saturation_vapour_pressure_kpa(real t_c);
// Atmospheric pressure (kPa) at elevation z, eq 7.
real atmospheric_pressure_kpa(real elevation_m);
// Extraterrestrial radiation Ra (MJ m-2 day-1), eq 21-25.
real extraterrestrial_radiation(real latitude_deg, int day_of_year);
// Daylight hours N, eq 34.
real daylight_hours(real latitude_deg, int day_of_year);
// ea from RHmax and RHmin (fractions, 0..1), eq 17.
real ea_from_rh_extremes_kpa(real tmin_c, real tmax_c, real rh_max, real rh_min);
// Angstrom estimate of Rs from sunshine hours, eq 35 (a_s = 0.25, b_s = 0.50).
real rs_from_sunshine(real n_hours, real daylight_h, real ra);
// Wind speed at 2 m from a measurement at height z (m), eq 47.
real wind_speed_2m(real u_z_m_s, real height_m);

} // namespace ps::field
