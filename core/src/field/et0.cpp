// core/src/field/et0.cpp - see ps/field/et0.h.

#include "ps/field/et0.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace ps::field {

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kGsc = 0.0820;           // solar constant, MJ m-2 min-1 [C FAO-56 eq 21]
constexpr real kStefanBoltzmann = 4.903e-9;  // MJ K-4 m-2 day-1 [C FAO-56 eq 39]
constexpr real kAlbedo = 0.23;          // reference grass [C FAO-56 eq 38]

real declination_rad(int j) { return 0.409 * ps::math::sin(2.0 * kPi / 365.0 * j - 1.39); }  // eq 24

real sunset_hour_angle(real lat_rad, real decl_rad) {
    // eq 25, clamped to the polar day/night limits
    real x = -ps::math::tan(lat_rad) * ps::math::tan(decl_rad);
    x = std::min(1.0, std::max(-1.0, x));
    return ps::math::acos(x);
}

} // namespace

real saturation_vapour_pressure_kpa(real t_c) {
    return 0.6108 * ps::math::exp(17.27 * t_c / (t_c + 237.3));
}

real atmospheric_pressure_kpa(real z) {
    return 101.3 * ps::math::pow((293.0 - 0.0065 * z) / 293.0, 5.26);
}

real extraterrestrial_radiation(real latitude_deg, int j) {
    const real lat = latitude_deg * kPi / 180.0;
    const real dr = 1.0 + 0.033 * ps::math::cos(2.0 * kPi / 365.0 * j);  // eq 23
    const real d = declination_rad(j);
    const real ws = sunset_hour_angle(lat, d);
    return (24.0 * 60.0 / kPi) * kGsc * dr *
           (ws * ps::math::sin(lat) * ps::math::sin(d) + ps::math::cos(lat) * ps::math::cos(d) * ps::math::sin(ws));
}

real daylight_hours(real latitude_deg, int j) {
    const real lat = latitude_deg * kPi / 180.0;
    return 24.0 / kPi * sunset_hour_angle(lat, declination_rad(j));
}

real ea_from_rh_extremes_kpa(real tmin_c, real tmax_c, real rh_max, real rh_min) {
    return (saturation_vapour_pressure_kpa(tmin_c) * rh_max + saturation_vapour_pressure_kpa(tmax_c) * rh_min) / 2.0;
}

real rs_from_sunshine(real n_hours, real daylight_h, real ra) {
    return (0.25 + 0.50 * n_hours / daylight_h) * ra;
}

real wind_speed_2m(real u_z, real h) { return u_z * 4.87 / ps::math::log(67.8 * h - 5.42); }

Et0Result fao56_et0(const Et0Input& in) {
    Et0Result r;
    r.tmean_c = (in.tmax_c + in.tmin_c) / 2.0;
    r.pressure_kpa = atmospheric_pressure_kpa(in.elevation_m);
    r.gamma = 0.665e-3 * r.pressure_kpa;
    const real et = saturation_vapour_pressure_kpa(r.tmean_c);
    r.delta = 4098.0 * et / ((r.tmean_c + 237.3) * (r.tmean_c + 237.3));
    r.es_kpa = (saturation_vapour_pressure_kpa(in.tmax_c) + saturation_vapour_pressure_kpa(in.tmin_c)) / 2.0;
    r.ea_kpa = in.ea_kpa;
    r.ra = extraterrestrial_radiation(in.latitude_deg, in.day_of_year);
    r.daylight_h = daylight_hours(in.latitude_deg, in.day_of_year);
    r.rso = (0.75 + 2.0e-5 * in.elevation_m) * r.ra;
    r.rns = (1.0 - kAlbedo) * in.rs_mj_m2;
    const real tx = in.tmax_c + 273.16, tn = in.tmin_c + 273.16;
    const real rel = r.rso > 0.0 ? std::min(1.0, in.rs_mj_m2 / r.rso) : 0.0;
    r.rnl = kStefanBoltzmann * (tx * tx * tx * tx + tn * tn * tn * tn) / 2.0 *
            (0.34 - 0.14 * std::sqrt(std::max(0.0, in.ea_kpa))) * (1.35 * rel - 0.35);
    r.rn = r.rns - r.rnl;
    const real denom = r.delta + r.gamma * (1.0 + 0.34 * in.u2_m_s);
    r.radiation_term = 0.408 * r.delta * (r.rn - in.g_mj_m2) / denom;
    r.aero_term = r.gamma * 900.0 / (r.tmean_c + 273.0) * in.u2_m_s * (r.es_kpa - in.ea_kpa) / denom;
    r.et0_mm = std::max(0.0, r.radiation_term + r.aero_term);
    return r;
}

} // namespace ps::field
