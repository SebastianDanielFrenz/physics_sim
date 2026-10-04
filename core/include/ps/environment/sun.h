// ps/environment/sun.h - sun position and clear-sky irradiance (weather plan 2.3).
//
// Two modes (owner question Q1, both implemented):
//
//  * Astronomical-warped (default): the real solar declination for the day of
//    year (Spencer series) and the real hour angle for the latitude, with the
//    hour angle WARPED piecewise-linearly onto the game's own day/night
//    lengths: daytime u in [0,1) -> H = -H0 + 2 H0 u, night v in [0,1) ->
//    H = H0 + (2 pi - 2 H0) v, where H0 is the real sunrise hour angle of that
//    day. Real sun height and path shape for latitude and date, the game's own
//    sunrise/sunset times. When the authored day equals the real one the warp
//    is the identity and the result is real local MEAN solar time (the
//    equation of time is not modelled; the game clock is mean solar time and
//    the longitude is not used).
//  * Authored: elevation e_max sin(pi u) by day, -e_night sin(pi v) by night,
//    azimuth linear from az_rise to az_set through south (north for southern
//    latitudes) by day and on around the circle at night.
//
// Polar corner cases (documented): when the real H0 degenerates (polar night
// or day for that date) while the authored day does not, the warp cannot hit
// the authored sunrise/sunset, so that day falls back to the AUTHORED curve
// (global/season authored parameters when present, else parameters derived
// from the real culminations). A day with zero-length day or night (only the
// astronomical day/night form can produce one) uses the unwarped real hour
// angle, so the polar sun path is the real one.
//
// All trig through ps::math (deterministic); evaluated once per tick, never
// per substep.

#pragma once

#include "ps/environment/calendar.h"
#include "ps/environment/climate_desc.h"
#include "ps/types.h"

namespace ps::environment {

struct SunState {
    real elevation_rad = 0.0;   // geometric (no refraction), above the horizon positive
    real azimuth_rad = 0.0;     // from north, clockwise, [0, 2 pi)
    // Unit vector towards the sun in a local east/north/up frame.
    real dir_east = 0.0;
    real dir_north = 0.0;
    real dir_up = 0.0;
    real declination_rad = 0.0;
    real hour_angle_rad = 0.0;  // (-pi, pi], 0 at solar noon (astronomical branches; authored: the equivalent for the real-sun path is not defined, 0 is reported)
    bool is_day = false;        // by the calendar's authored sunrise/sunset
    bool authored_curve = false;  // true when the authored elevation curve produced this state (authored mode or the polar fallback)
};

class Sun {
public:
    // Reads latitude, mode and the global authored block from the descriptor.
    explicit Sun(const ClimateDesc& desc);

    SunState at(const Calendar& calendar, const CalendarSample& sample) const;
    SunState at(const Calendar& calendar, real t_env) const { return at(calendar, calendar.sample(t_env)); }

    SunMode mode() const { return mode_; }
    real latitude_rad() const { return lat_rad_; }

private:
    SunState from_hour_angle(real hour_angle, real decl_rad) const;
    SunState authored_state(const DayValues& day, const CalendarSample& s, real decl_noon_rad) const;

    SunMode mode_ = SunMode::Astronomical;
    real lat_rad_ = 0.0;
    real sin_lat_ = 0.0;
    real cos_lat_ = 1.0;
};

// Clear-sky global horizontal irradiance, W/m2, Haurwitz model
// G = 1098 cos(z) exp(-0.059 / cos(z)) for elevation > 0, else 0
// [C Haurwitz 1945, J. Meteorology 2:154; constant 0.059 as implemented in
// pvlib, see docs/data_formats/climate.md for the verification status].
real clear_sky_ghi_w_m2(real elevation_rad);

// With cloud fraction N in [0,1]: G = G_clear (1 - 0.75 N^3.4)
// [C Kasten & Czeplak 1980, Solar Energy 24].
real ghi_w_m2(real elevation_rad, real cloud_fraction);

} // namespace ps::environment
