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

// ---- W3 weather fixtures ----------------------------------------------------

inline RegimeDesc weather_regime(real median_h, real sigma_log, RangeDesc cloud, real wind_c, real gust,
                                 RangeDesc temp_off, RangeDesc rh, RangeDesc press_off) {
    RegimeDesc r;
    r.duration = DurationDesc{0.1, median_h, sigma_log};
    r.cloud_cover = cloud;
    r.wind_mean_m_s = WeibullDesc{2.0, wind_c};
    r.wind_dir_deg = WindDirDesc{240.0, 60.0};
    r.gust_factor = gust;
    r.temp_offset_k = temp_off;
    r.rh = rh;
    r.pressure_offset_hpa = press_off;
    return r;
}

inline TransitionRowDesc weather_row(const std::string& from, real p_clear, real p_rain, real p_storm) {
    return TransitionRowDesc{from, {{"clear", p_clear}, {"rain", p_rain}, {"storm", p_storm}}};
}

// Two explicit seasons with the same three regimes (clear, rain, storm) and
// DIFFERENT transition matrices (test T1 tells them apart):
//   season a: clear {.5 .3 .2}, rain {.7 .2 .1}, storm {.6 .4 0}
//   season b: clear {.1 .6 .3}, rain {.2 .5 .3}, storm {.3 .3 .4}
// Durations: lognormal median 6 / 3 / 2 h, sigma 0.3 (the 0.1 h floor never
// binds: exp(-0.3 * 3.46) * 2 h = 0.71 h). storm is convective with hail and
// 3 strikes per minute during cells; rain has stratiform precipitation.
// Every value is a TEST value, not climatology.
inline ClimateDesc weather_two_season(real day_a_s = 50400.0, real night_a_s = 36000.0,
                                      real day_b_s = 30000.0, real night_b_s = 56400.0) {
    ClimateDesc d;
    d.location.latitude_deg = 50.15;
    d.sun_mode = SunMode::Authored;
    d.sun_authored = SunAuthoredDesc{60.0, 30.0, 90.0, 270.0};
    d.calendar.cycle_s = 86400.0;
    d.calendar.epoch.season = "a";
    d.calendar.epoch.day = 0;
    d.calendar.epoch.time_of_day_s = 0.0;

    RegimeDesc clear = weather_regime(6.0, 0.3, {0.0, 0.2}, 3.0, 1.4, {0.5, 1.5}, {0.45, 0.6}, {2.0, 6.0});
    RegimeDesc rain = weather_regime(3.0, 0.3, {0.9, 1.0}, 5.0, 1.6, {-2.0, -1.0}, {0.85, 0.95}, {-8.0, -4.0});
    rain.precip_mm_h = LogNormalDesc{2.0, 0.5};
    RegimeDesc storm = weather_regime(2.0, 0.3, {0.8, 1.0}, 6.0, 1.7, {-3.0, -1.0}, {0.75, 0.9}, {-5.0, -1.0});
    storm.precip_mm_h = LogNormalDesc{0.5, 0.3};
    ConvectiveDesc cv;
    cv.cell_duration_min = LogNormalDesc{25.0, 0.3};
    cv.gap_min = LogNormalDesc{20.0, 0.5};
    cv.peak_mm_h = LogNormalDesc{30.0, 0.6};
    cv.rise_fraction = 0.3;
    cv.gust_factor = 2.2;
    cv.hail = HailDesc{0.5, LogNormalDesc{12.0, 0.4}, 0.6};
    storm.convective = cv;
    storm.lightning_per_min = 3.0;

    SeasonWeatherDesc wa;
    wa.regimes = {{"clear", clear}, {"rain", rain}, {"storm", storm}};
    wa.transitions = {weather_row("clear", 0.5, 0.3, 0.2), weather_row("rain", 0.7, 0.2, 0.1),
                      weather_row("storm", 0.6, 0.4, 0.0)};
    wa.initial_regime = "clear";
    SeasonWeatherDesc wb = wa;
    wb.transitions = {weather_row("clear", 0.1, 0.6, 0.3), weather_row("rain", 0.2, 0.5, 0.3),
                      weather_row("storm", 0.3, 0.3, 0.4)};
    wb.initial_regime = "rain";

    SeasonDesc a = explicit_season("a", 31, day_a_s, night_a_s);
    a.climate = SeasonClimateDesc{16.0, 10.0, 0.65, 1015.0, 14.0};
    a.weather = wa;
    SeasonDesc b = explicit_season("b", 31, day_b_s, night_b_s);
    b.climate = SeasonClimateDesc{1.0, 6.0, 0.85, 1010.0, 2.0};
    b.weather = wb;
    d.seasons = {a, b};

    d.lags = WeatherLagsDesc{};
    d.precip_phase = PrecipPhaseDesc{};
    d.humidity = HumidityDesc{};
    d.diurnal_temperature = DiurnalTempDesc{};
    d.lightning = LightningDesc{0.5, 15.0};
    return d;
}

// Stationary distribution of the fixture's season-b matrix (solved by hand:
// pi_storm = 1/3, 1.1 pi_clear = 0.2 (2/3) + 0.1 => pi_clear = 7/33,
// pi_rain = 15/33), order clear, rain, storm.
inline constexpr real kWeatherStationaryB[3] = {7.0 / 33.0, 15.0 / 33.0, 11.0 / 33.0};

} // namespace ps::test_env
