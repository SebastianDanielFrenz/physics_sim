// ps/environment/solar_geometry.h - pure solar-position building blocks
// (weather plan 2.3). Deterministic: only ps::math transcendentals, only
// + - * / sqrt otherwise (PLAN.md D7).

#pragma once

#include "ps/types.h"

namespace ps::environment {

// Zenith angle at which the sun's upper limb is on the horizon, refraction
// included: 90.833 deg [C NOAA GML "General Solar Position Calculations",
// https://gml.noaa.gov/grad/solcalc/solareqns.PDF]. Sunrise/sunset in this
// library are defined by this zenith; the elevation the library PUBLISHES is
// the geometric one (no refraction), so it is -0.833 deg at sunrise.
inline constexpr real kSunriseZenithDeg = 90.833;

// Solar declination in radians from Spencer's Fourier series [C Spencer 1971,
// coefficients as given in NOAA GML solareqns.PDF eq. 5]. `y_days` is the
// continuous time in days since 1 January 00:00 (noon of 1 January = 0.5);
// the series is periodic in 365 days. Max error ~0.0006 rad (0.034 deg).
real declination_spencer_rad(real y_days);

// Sunrise hour angle H0 in [0, pi] for a latitude and declination (radians):
// cos H0 = (cos(zenith) - sin(phi) sin(delta)) / (cos(phi) cos(delta)).
// Polar night (sun never rises) returns 0, polar day (never sets) returns pi.
real sunrise_hour_angle_rad(real lat_rad, real decl_rad);

// Elevation (radians, geometric) of the sun at its upper / lower culmination:
//   upper = pi/2 - |phi - delta|,  lower = |phi + delta| - pi/2.
real upper_culmination_elevation_rad(real lat_rad, real decl_rad);
real lower_culmination_elevation_rad(real lat_rad, real decl_rad);

} // namespace ps::environment
