// core/src/environment/solar_geometry.cpp - see ps/environment/solar_geometry.h.

#include "ps/environment/solar_geometry.h"

#include "ps/environment/climate_desc.h"
#include "ps/math/transcendental.h"

#include <cmath>

namespace ps::environment {

real declination_spencer_rad(real y_days) {
    // gamma = 2 pi / 365 * (day_of_year - 1 + (hour - 12) / 24) with
    // y = day_of_year - 1 + hour / 24, i.e. gamma = 2 pi / 365 * (y - 0.5).
    const real gamma = 2.0 * kPi / 365.0 * (y_days - 0.5);
    real s1, c1, s2, c2, s3, c3;
    math::sincos(gamma, s1, c1);
    math::sincos(2.0 * gamma, s2, c2);
    math::sincos(3.0 * gamma, s3, c3);
    return 0.006918
         - 0.399912 * c1 + 0.070257 * s1
         - 0.006758 * c2 + 0.000907 * s2
         - 0.002697 * c3 + 0.00148 * s3;
}

real sunrise_hour_angle_rad(real lat_rad, real decl_rad) {
    const real zen = kSunriseZenithDeg * kDegToRad;
    const real num = math::cos(zen) - math::sin(lat_rad) * math::sin(decl_rad);
    const real den = math::cos(lat_rad) * math::cos(decl_rad);
    if (std::fabs(den) < 1e-12) {
        // Pole (or a declination of +-90 deg): the sun's elevation is constant.
        return num > 0.0 ? 0.0 : kPi;
    }
    const real x = num / den;
    if (x >= 1.0) return 0.0;   // polar night
    if (x <= -1.0) return kPi;  // polar day
    return math::acos(x);
}

real upper_culmination_elevation_rad(real lat_rad, real decl_rad) {
    return 0.5 * kPi - std::fabs(lat_rad - decl_rad);
}

real lower_culmination_elevation_rad(real lat_rad, real decl_rad) {
    return std::fabs(lat_rad + decl_rad) - 0.5 * kPi;
}

} // namespace ps::environment
