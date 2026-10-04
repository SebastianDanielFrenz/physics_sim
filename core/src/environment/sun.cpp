// core/src/environment/sun.cpp - see ps/environment/sun.h.

#include "ps/environment/sun.h"

#include "ps/environment/solar_geometry.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>

namespace ps::environment {

namespace {

real wrap_two_pi(real a) {
    const real two_pi = 2.0 * kPi;
    a -= two_pi * std::floor(a / two_pi);
    if (a >= two_pi) a -= two_pi;  // floor rounding at the very edge
    return a;
}

real mod360(real deg) { return deg - 360.0 * std::floor(deg / 360.0); }

} // namespace

Sun::Sun(const ClimateDesc& desc) {
    mode_ = desc.sun_mode;
    lat_rad_ = desc.location.latitude_deg * kDegToRad;
    math::sincos(lat_rad_, sin_lat_, cos_lat_);
}

SunState Sun::from_hour_angle(real hour_angle, real decl_rad) const {
    real sin_h, cos_h, sin_d, cos_d;
    math::sincos(hour_angle, sin_h, cos_h);
    math::sincos(decl_rad, sin_d, cos_d);

    real sin_e = sin_lat_ * sin_d + cos_lat_ * cos_d * cos_h;
    sin_e = std::min(1.0, std::max(-1.0, sin_e));

    SunState st;
    st.declination_rad = decl_rad;
    st.hour_angle_rad = hour_angle;
    st.elevation_rad = math::asin(sin_e);
    // Azimuth from south, positive towards west: A = atan2(sin H, cos H sin(phi) - tan(delta) cos(phi)).
    // Multiplying both arguments by cos(delta) > 0 keeps the quadrant and avoids the tangent.
    const real a_south = math::atan2(sin_h * cos_d, cos_h * sin_lat_ * cos_d - sin_d * cos_lat_);
    st.azimuth_rad = wrap_two_pi(a_south + kPi);  // from north, clockwise
    return st;
}

SunState Sun::authored_state(const DayValues& day, const CalendarSample& s, real decl_noon_rad) const {
    real e_max, e_night, az_rise_deg, az_set_deg;
    if (day.has_authored) {
        e_max = day.authored.e_max_deg * kDegToRad;
        e_night = day.authored.e_night_deg * kDegToRad;
        az_rise_deg = day.authored.az_rise_deg;
        az_set_deg = day.authored.az_set_deg;
    } else {
        // Polar fallback without authored parameters: the real culminations.
        e_max = upper_culmination_elevation_rad(lat_rad_, decl_noon_rad);
        e_night = -lower_culmination_elevation_rad(lat_rad_, decl_noon_rad);
        az_rise_deg = 90.0;
        az_set_deg = 270.0;
    }

    const bool through_south = lat_rad_ >= 0.0;
    real sweep_day;
    real sweep_night;
    if (through_south) {
        sweep_day = mod360(az_set_deg - az_rise_deg);
        if (sweep_day == 0.0) sweep_day = 360.0;
        sweep_night = 360.0 - sweep_day;
    } else {
        sweep_day = -mod360(az_rise_deg - az_set_deg);
        if (sweep_day == 0.0) sweep_day = -360.0;
        sweep_night = -360.0 - sweep_day;
    }

    real elevation, az_deg;
    if (s.phase == DayPhase::Day) {
        elevation = e_max * math::sin(kPi * s.phase_u);
        az_deg = az_rise_deg + sweep_day * s.phase_u;
    } else {
        elevation = -e_night * math::sin(kPi * s.phase_u);
        az_deg = az_set_deg + sweep_night * s.phase_u;
    }

    SunState st;
    st.elevation_rad = elevation;
    st.azimuth_rad = wrap_two_pi(az_deg * kDegToRad);
    st.declination_rad = declination_spencer_rad(0.5 * (day.y_start + day.y_end));
    st.hour_angle_rad = 0.0;
    st.authored_curve = true;
    return st;
}

SunState Sun::at(const Calendar& calendar, const CalendarSample& s) const {
    const DayValues& dv = calendar.day(s.day_of_year);
    const real y = dv.y_start + s.day_fraction * (dv.y_end - dv.y_start);
    const real decl = declination_spencer_rad(y);
    const real decl_noon = declination_spencer_rad(0.5 * (dv.y_start + dv.y_end));

    SunState st;
    bool have_state = false;
    if (mode_ == SunMode::Authored) {
        st = authored_state(dv, s, decl_noon);
        have_state = true;
    } else if (dv.day_s > 0.0 && dv.night_s > 0.0) {
        const real h0 = sunrise_hour_angle_rad(lat_rad_, decl_noon);
        if (h0 <= 0.0 || h0 >= kPi) {
            // Real polar night/day while the authored day has both halves:
            // the warp cannot hit the authored sunrise/sunset.
            st = authored_state(dv, s, decl_noon);
            have_state = true;
        } else {
            real h;
            if (s.phase == DayPhase::Day) {
                h = -h0 + 2.0 * h0 * s.phase_u;
            } else {
                h = h0 + (2.0 * kPi - 2.0 * h0) * s.phase_u;
                if (h > kPi) h -= 2.0 * kPi;
            }
            st = from_hour_angle(h, decl);
            have_state = true;
        }
    }
    if (!have_state) {
        // Zero-length day or night (polar): the real, unwarped hour angle.
        st = from_hour_angle(2.0 * kPi * (s.day_fraction - 0.5), decl);
    }

    st.is_day = s.phase == DayPhase::Day;
    real sin_e, cos_e, sin_a, cos_a;
    math::sincos(st.elevation_rad, sin_e, cos_e);
    math::sincos(st.azimuth_rad, sin_a, cos_a);
    st.dir_east = cos_e * sin_a;
    st.dir_north = cos_e * cos_a;
    st.dir_up = sin_e;
    return st;
}

real clear_sky_ghi_w_m2(real elevation_rad) {
    // cos(zenith) = sin(elevation).
    const real cos_z = math::sin(elevation_rad);
    if (!(cos_z > 0.0)) return 0.0;
    return 1098.0 * cos_z * math::exp(-0.059 / cos_z);
}

real ghi_w_m2(real elevation_rad, real cloud_fraction) {
    const real clear = clear_sky_ghi_w_m2(elevation_rad);
    const real n = std::min(1.0, std::max(0.0, cloud_fraction));
    if (n <= 0.0) return clear;
    return clear * (1.0 - 0.75 * math::pow(n, 3.4));
}

} // namespace ps::environment
