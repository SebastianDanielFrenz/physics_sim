// core/src/environment/climate_desc.cpp - validate_climate / climate_validation_error
// (see ps/environment/climate_desc.h).
//
// The checks live in a function that RETURNS the first problem as a string;
// validate_climate throws it. The loader (which must not catch exception
// objects - clang-cl ASan caveat, CLAUDE.md) uses the string form to prefix
// the file name.

#include "ps/environment/climate_desc.h"

#include "weather_validation.h"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace ps::environment {

namespace {

std::string err(const std::string& what) { return "climate: " + what; }

std::string finite_error(real v, const std::string& field) {
    if (!std::isfinite(v)) return err(field + " must be finite");
    return {};
}

std::string authored_error(real e_max, real e_night, real az_rise, real az_set, const std::string& where) {
    std::string e;
    if (!(e = finite_error(e_max, where + ".e_max_deg")).empty()) return e;
    if (!(e = finite_error(e_night, where + ".e_night_deg")).empty()) return e;
    if (!(e = finite_error(az_rise, where + ".az_rise_deg")).empty()) return e;
    if (!(e = finite_error(az_set, where + ".az_set_deg")).empty()) return e;
    if (e_max < -90.0 || e_max > 90.0) return err(where + ".e_max_deg must be in [-90, 90]");
    if (e_night < -90.0 || e_night > 90.0) return err(where + ".e_night_deg must be in [-90, 90]");
    if (az_rise < 0.0 || az_rise > 360.0) return err(where + ".az_rise_deg must be in [0, 360]");
    if (az_set < 0.0 || az_set > 360.0) return err(where + ".az_set_deg must be in [0, 360]");
    return {};
}

} // namespace

std::string climate_validation_error(const ClimateDesc& d) {
    std::string e;
    if (!(e = finite_error(d.location.latitude_deg, "location.latitude_deg")).empty()) return e;
    if (!(e = finite_error(d.location.longitude_deg, "location.longitude_deg")).empty()) return e;
    if (!(e = finite_error(d.location.elevation_m, "location.elevation_m")).empty()) return e;
    if (d.location.latitude_deg < -90.0 || d.location.latitude_deg > 90.0)
        return err("location.latitude_deg must be in [-90, 90]");
    if (d.location.longitude_deg < -180.0 || d.location.longitude_deg > 180.0)
        return err("location.longitude_deg must be in [-180, 180]");

    if (!(d.calendar.cycle_s > 0.0) || !std::isfinite(d.calendar.cycle_s))
        return err("calendar.cycle_s must be > 0 and finite");
    if (!(e = finite_error(d.calendar.epoch.time_of_day_s, "calendar.epoch.time_of_day_s")).empty()) return e;
    if (d.calendar.epoch.time_of_day_s < 0.0) return err("calendar.epoch.time_of_day_s must be >= 0");

    if (d.seasons.empty()) return err("seasons must not be empty");

    long long total_days = 0;
    std::set<std::string> names;
    bool any_climate = false;
    bool all_climate = true;
    bool any_astronomical_day = false;

    if (d.sun_authored) {
        const SunAuthoredDesc& a = *d.sun_authored;
        if (!(e = authored_error(a.e_max_deg, a.e_night_deg, a.az_rise_deg, a.az_set_deg, "sun.authored")).empty()) return e;
    }

    for (std::size_t i = 0; i < d.seasons.size(); ++i) {
        const SeasonDesc& s = d.seasons[i];
        const std::string where = "seasons[" + std::to_string(i) + "]";
        if (s.name.empty()) return err(where + ".name must not be empty");
        if (!names.insert(s.name).second) return err(where + ".name \"" + s.name + "\" is not unique");
        if (s.days < 1) return err(where + ".days must be >= 1");
        total_days += s.days;
        if (total_days > 100000) return err("the year has more than 100000 days");

        if (s.has_doy_span) {
            if (s.doy_start < 1 || s.doy_start > 366 || s.doy_end < 1 || s.doy_end > 366)
                return err(where + ".doy_span must have both ends in [1, 366]");
        }

        const DayNightDesc& dn = s.day_night;
        switch (dn.kind) {
        case DayNightKind::Explicit:
            if (!(e = finite_error(dn.day_s, where + ".day_night.day_s")).empty()) return e;
            if (!(e = finite_error(dn.night_s, where + ".day_night.night_s")).empty()) return e;
            if (!(dn.day_s > 0.0) || !(dn.night_s > 0.0))
                return err(where + ".day_night day_s and night_s must be > 0");
            break;
        case DayNightKind::CycleFraction:
            if (!(e = finite_error(dn.cycle_s, where + ".day_night.cycle_s")).empty()) return e;
            if (!(e = finite_error(dn.day_fraction, where + ".day_night.day_fraction")).empty()) return e;
            if (!(dn.cycle_s > 0.0)) return err(where + ".day_night.cycle_s must be > 0");
            if (!(dn.day_fraction > 0.0) || !(dn.day_fraction < 1.0))
                return err(where + ".day_night.day_fraction must be in (0, 1)");
            break;
        case DayNightKind::Astronomical:
            any_astronomical_day = true;
            break;
        }

        if (s.climate) {
            any_climate = true;
            const SeasonClimateDesc& c = *s.climate;
            if (!(e = finite_error(c.temp_mean_c, where + ".temp_mean_c")).empty()) return e;
            if (!(e = finite_error(c.temp_range_k, where + ".temp_range_k")).empty()) return e;
            if (!(e = finite_error(c.rh_mean, where + ".rh_mean")).empty()) return e;
            if (!(e = finite_error(c.pressure_msl_hpa, where + ".pressure_msl_hpa")).empty()) return e;
            if (!(e = finite_error(c.ground_temp_c, where + ".ground_temp_c")).empty()) return e;
            if (c.temp_range_k < 0.0) return err(where + ".temp_range_k must be >= 0");
            if (c.rh_mean < 0.0 || c.rh_mean > 1.0) return err(where + ".rh_mean must be in [0, 1]");
            if (!(c.pressure_msl_hpa > 0.0)) return err(where + ".pressure_msl_hpa must be > 0");
        } else {
            all_climate = false;
        }

        const SunAuthoredPartial& p = s.sun_authored;
        if (p.e_max_deg || p.e_night_deg || p.az_rise_deg || p.az_set_deg) {
            if (!(e = authored_error(p.e_max_deg.value_or(0.0), p.e_night_deg.value_or(0.0),
                                     p.az_rise_deg.value_or(0.0), p.az_set_deg.value_or(0.0),
                                     where + ".sun_authored")).empty())
                return e;
        }
        if (d.sun_mode == SunMode::Authored) {
            const bool g = d.sun_authored.has_value();
            const bool complete = (g || p.e_max_deg) && (g || p.e_night_deg) && (g || p.az_rise_deg) && (g || p.az_set_deg);
            if (!complete) return err(where + " has no complete authored sun (the sun.authored block is missing)");
        }
    }

    // The real day of year is needed by the astronomical sun and by the
    // astronomical day length. A day blends its season's value with its
    // neighbour's, so when either is used EVERY season needs a span.
    if (d.sun_mode == SunMode::Astronomical || any_astronomical_day) {
        for (std::size_t i = 0; i < d.seasons.size(); ++i) {
            if (!d.seasons[i].has_doy_span)
                return err("seasons[" + std::to_string(i) + "] has no doy_span, required by the astronomical sun / "
                           "astronomical day length (every season needs one)");
        }
    }

    if (any_climate && !all_climate)
        return err("climate scalars (temp_mean_c ...) must be given on every season or on none");

    int epoch_season = 0;
    if (!d.calendar.epoch.season.empty()) {
        epoch_season = -1;
        for (std::size_t i = 0; i < d.seasons.size(); ++i)
            if (d.seasons[i].name == d.calendar.epoch.season) epoch_season = static_cast<int>(i);
        if (epoch_season < 0) return err("calendar.epoch.season \"" + d.calendar.epoch.season + "\" is not a season");
    }
    const int epoch_days = d.seasons[static_cast<std::size_t>(epoch_season)].days;
    if (d.calendar.epoch.day < 0 || d.calendar.epoch.day >= epoch_days)
        return err("calendar.epoch.day must be in [0, days of the epoch season)");
    return detail::climate_weather_validation_error(d);
}

void validate_climate(const ClimateDesc& desc) {
    const std::string e = climate_validation_error(desc);
    if (!e.empty()) throw std::invalid_argument(e);
}

} // namespace ps::environment
