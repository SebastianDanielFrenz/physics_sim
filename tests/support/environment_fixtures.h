// tests/support/environment_fixtures.h - shared climate descriptors for the
// environment tests (header-only, no ps_core dependency).
#pragma once

#include "ps/environment/climate_desc.h"

#include <string>
#include <vector>

namespace ps::test_env {

using namespace ps::environment;

inline SeasonDesc explicit_season(const std::string& name, int days, real day_s, real night_s) {
    SeasonDesc s;
    s.name = name;
    s.days = days;
    s.day_night.kind = DayNightKind::Explicit;
    s.day_night.day_s = day_s;
    s.day_night.night_s = night_s;
    return s;
}

// Three seasons with ODD day counts (31, 91, 59: a season midpoint sits on a day
// centre, so that day carries the authored value exactly), explicit day/night
// lengths that differ strongly between seasons, climate scalars, authored sun
// (so no doy_span is needed). Year = 181 game days.
inline ClimateDesc three_season_explicit() {
    ClimateDesc d;
    d.location.latitude_deg = 50.15;
    d.location.longitude_deg = 8.45;
    d.sun_mode = SunMode::Authored;
    d.sun_authored = SunAuthoredDesc{60.0, 30.0, 90.0, 270.0};
    d.calendar.cycle_s = 86400.0;
    d.calendar.epoch.season = "b";
    d.calendar.epoch.day = 10;
    d.calendar.epoch.time_of_day_s = 20000.0;

    SeasonDesc a = explicit_season("a", 31, 50400.0, 36000.0);
    a.climate = SeasonClimateDesc{10.0, 8.0, 0.7, 1015.0, 9.0};
    SeasonDesc b = explicit_season("b", 91, 30000.0, 56400.0);
    b.climate = SeasonClimateDesc{-2.0, 4.0, 0.9, 1005.0, 3.0};
    SeasonDesc c = explicit_season("c", 59, 43200.0, 43200.0);
    c.climate = SeasonClimateDesc{20.0, 12.0, 0.5, 1020.0, 15.0};
    d.seasons = {a, b, c};
    return d;
}

// The northern-hemisphere meteorological seasons (doy spans), 4 x 31 game
// days, astronomical sun and astronomical day/night, home-region latitude.
inline ClimateDesc astronomical_four_seasons(real latitude_deg = 50.15) {
    ClimateDesc d;
    d.location.latitude_deg = latitude_deg;
    d.location.longitude_deg = 8.45;
    d.sun_mode = SunMode::Astronomical;
    d.calendar.cycle_s = 86400.0;
    const struct { const char* name; int a, b; } spans[] = {
        {"spring", 60, 151}, {"summer", 152, 243}, {"autumn", 244, 334}, {"winter", 335, 59}};
    for (const auto& sp : spans) {
        SeasonDesc s;
        s.name = sp.name;
        s.days = 31;
        s.has_doy_span = true;
        s.doy_start = sp.a;
        s.doy_end = sp.b;
        s.day_night.kind = DayNightKind::Astronomical;
        d.seasons.push_back(s);
    }
    return d;
}

// One season of 365 game days standing for the whole real year (day of year =
// game day + 1), astronomical, 86400 s cycle: the warp is the identity, so the
// sun is real local mean solar time (E3).
inline ClimateDesc real_year(real latitude_deg = 50.15, int days = 365) {
    ClimateDesc d;
    d.location.latitude_deg = latitude_deg;
    d.sun_mode = SunMode::Astronomical;
    d.calendar.cycle_s = 86400.0;
    SeasonDesc s;
    s.name = "year";
    s.days = days;
    s.has_doy_span = true;
    s.doy_start = 1;
    s.doy_end = 365;
    s.day_night.kind = DayNightKind::Astronomical;
    d.seasons.push_back(s);
    return d;
}

} // namespace ps::test_env
