// tests/unit/test_environment_day_share.cpp - weather plan W3b (owner answer Q2): the third day/night
// form, the astronomical day share times a per-season multiplier, clamped to
// [kDayShareMin, kDayShareMax] (DayNightKind::AstronomicalScaled). Tags [environment][calendar]
// (+ [sun], [io]); links only ps_environment / ps_environment_io, never ps_core.
//
// RECORDED SABOTAGES (performed by hand through the scratch driver, reverted; measured 2026-10-05):
//  - S1 multiplier ignored (calendar.cpp: share = h0/pi): "midpoint days equal share x multiplier" and
//    "the low clamp ..." failed (2 cases, 504 assertions, e.g. day_s off by 17228 s on a midpoint day).
//  - S2 clamp removed: 3 cases failed (393 assertions): the polar scaled day became 86400 s instead of
//    kDayShareMax x 86400 = 84672 s with night 0, and the low-multiplier days were 1333 s off.
//  - S3 the plain Astronomical kind clamped as well: "Explicit and Astronomical stay bit-identical" failed
//    at the polar latitude (day 84672 s instead of 86400 s, night 1728 s instead of 0).
//  - S4 a fixed 0.5 share instead of share (day length independent of H0 and m): 3 cases failed (505
//    assertions). Sunrise/sunset consistency itself is by construction (the warp reads the calendar's
//    day_s/night_s), so no separate sabotage exists for it; the bisection checks guard the transitions.
//  - S5 loader: the multiplier read but the kind left Astronomical: "loader round trip" failed
//    (kind 2 instead of 3).

#include "environment_fixtures.h"

#include "ps/environment/calendar.h"
#include "ps/environment/climate_io.h"
#include "ps/environment/solar_geometry.h"
#include "ps/environment/sun.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

using namespace ps;
using namespace ps::environment;
using nlohmann::json;
using Catch::Matchers::ContainsSubstring;

namespace {

namespace fs = std::filesystem;

// Sunrise as the instant is_day flips, found by bisection on sample() alone.
double bisect_transition(const Calendar& cal, double t_lo, double t_hi, bool lo_is_day) {
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (t_lo + t_hi);
        if (cal.sample(mid).is_day() == lo_is_day) t_lo = mid; else t_hi = mid;
    }
    return 0.5 * (t_lo + t_hi);
}

// The independent reference: day length of a season's form at real day of year y.
double ref_day_s(const ClimateDesc& desc, int season, double y) {
    const DayNightDesc& dn = desc.seasons[static_cast<std::size_t>(season)].day_night;
    const double cycle = desc.calendar.cycle_s;
    const double share = sunrise_hour_angle_rad(desc.location.latitude_deg * kDegToRad, declination_spencer_rad(y)) / kPi;
    if (dn.kind == DayNightKind::AstronomicalScaled)
        return cycle * std::min(kDayShareMax, std::max(kDayShareMin, share * dn.day_share_multiplier));
    return cycle * share;
}

double span_of(const SeasonDesc& s) {
    return s.doy_end >= s.doy_start ? s.doy_end - s.doy_start + 1.0 : 365.0 - s.doy_start + 1.0 + s.doy_end;
}

double y_mid_of_season(const SeasonDesc& s) { return (s.doy_start - 1) + span_of(s) / 2.0; }

ClimateDesc scaled_four_seasons() {
    ClimateDesc d = test_env::astronomical_four_seasons();
    // spring plain astronomical; summer longer days; autumn shorter days; winter x3 saturates the clamp.
    d.seasons[1].day_night.kind = DayNightKind::AstronomicalScaled;
    d.seasons[1].day_night.day_share_multiplier = 1.3;
    d.seasons[2].day_night.kind = DayNightKind::AstronomicalScaled;
    d.seasons[2].day_night.day_share_multiplier = 0.8;
    d.seasons[3].day_night.kind = DayNightKind::AstronomicalScaled;
    d.seasons[3].day_night.day_share_multiplier = 3.0;
    return d;
}

} // namespace

TEST_CASE("W3b scaled day share: midpoint days equal share x multiplier (clamped), interpolation between forms",
          "[environment][calendar]") {
    const ClimateDesc desc = scaled_four_seasons();
    const ClimateDesc plain = test_env::astronomical_four_seasons();
    const Calendar cal(desc);
    const double cycle = desc.calendar.cycle_s;

    // Season midpoints (31 days, odd: the middle day carries the season's value exactly).
    for (int s = 0; s < 4; ++s) {
        const int mid_day = cal.season_start_day(s) + 15;
        const double y = y_mid_of_season(desc.seasons[static_cast<std::size_t>(s)]);
        const double want = ref_day_s(desc, s, y);
        INFO("season " << s << " mid day " << mid_day << " want day_s " << want);
        CHECK(std::fabs(cal.day(mid_day).day_s - want) < 1e-9);
        CHECK(std::fabs(cal.day(mid_day).night_s - (cycle - want)) < 1e-9);
        CHECK(std::fabs(cal.day(mid_day).length_s() - cycle) < 1e-9);   // cycle length = the calendar's global cycle_s
    }
    // The cases are what the fixture claims: a lengthened day, a shortened day, a saturated clamp.
    const double plain_summer = ref_day_s(plain, 1, y_mid_of_season(plain.seasons[1]));
    const double plain_autumn = ref_day_s(plain, 2, y_mid_of_season(plain.seasons[2]));
    CHECK(cal.day(cal.season_start_day(1) + 15).day_s > plain_summer * 1.25);
    CHECK(cal.day(cal.season_start_day(2) + 15).day_s < 0.9 * plain_autumn);
    CHECK(std::fabs(cal.day(cal.season_start_day(3) + 15).day_s - kDayShareMax * cycle) < 1e-9);   // winter clamped high
    CHECK(std::fabs(cal.day(cal.season_start_day(3) + 15).night_s - (1.0 - kDayShareMax) * cycle) < 1e-9);

    // Every day: the linear interpolation between the neighbouring midpoints of each season's own
    // form evaluated at THIS day's real day of year (31-day seasons, so the midpoint spacing is 31).
    const int total = cal.days_per_year();
    REQUIRE(total == 124);
    for (int d = 0; d < total; ++d) {
        const int s = cal.day(d).season;
        const double xc = d + 0.5;
        const double k = cal.day(d).day_in_season + 0.5;
        const SeasonDesc& sd = desc.seasons[static_cast<std::size_t>(s)];
        const double y_noon = (sd.doy_start - 1) + k / 31.0 * span_of(sd);
        int left, right;
        double w;
        const double mid_s = cal.season_start_day(s) + 15.5;
        if (xc >= mid_s) { left = s; right = (s + 1) % 4; w = (xc - mid_s) / 31.0; }
        else { left = (s + 3) % 4; right = s; w = (xc - (mid_s - 31.0)) / 31.0; }
        const double dl = ref_day_s(desc, left, y_noon), dr = ref_day_s(desc, right, y_noon);
        const double want = dl + (dr - dl) * w;
        INFO("day " << d);
        CHECK(std::fabs(cal.day(d).day_s - want) < 1e-9);
        CHECK(std::fabs(cal.day(d).night_s - (cycle - want)) < 1e-9);
        CHECK(cal.day(d).day_s >= kDayShareMin * cycle - 1e-9);
        CHECK(cal.day(d).day_s <= cycle + 1e-9);
    }
}

TEST_CASE("W3b scaled day share: the low clamp and sunrise/sunset equal the day/night lengths", "[environment][calendar][sun]") {
    ClimateDesc desc = test_env::astronomical_four_seasons();
    for (SeasonDesc& s : desc.seasons) {
        s.day_night.kind = DayNightKind::AstronomicalScaled;
        s.day_night.day_share_multiplier = 0.01;
    }
    const Calendar low(desc);
    for (int d = 0; d < low.days_per_year(); ++d) {
        CHECK(std::fabs(low.day(d).day_s - kDayShareMin * 86400.0) < 1e-9);   // clamped low everywhere
        CHECK(std::fabs(low.day(d).night_s - (1.0 - kDayShareMin) * 86400.0) < 1e-9);
    }

    // E1-style: sunrise/sunset measured from sample() alone (bisection of the is_day flip) equal
    // night/2 and night/2 + day of the SCALED lengths, on every 3rd day of a year with a mix of forms.
    const ClimateDesc mixed = scaled_four_seasons();
    const Calendar cal(mixed);
    const Sun sun(mixed);
    for (int d = 0; d < cal.days_per_year(); d += 3) {
        const DayValues& dv = cal.day(d);
        const double t0 = cal.env_time(0, dv.season, dv.day_in_season, 0.0);
        const double sunrise = bisect_transition(cal, t0, t0 + dv.night_s * 0.5 + dv.day_s * 0.5, false);
        const double sunset = bisect_transition(cal, t0 + dv.night_s * 0.5 + dv.day_s * 0.5, t0 + dv.length_s() - 1e-3, true);
        INFO("day " << d);
        CHECK(std::fabs((sunrise - t0) - dv.night_s * 0.5) < 1e-6);
        CHECK(std::fabs((sunset - t0) - (dv.night_s * 0.5 + dv.day_s)) < 1e-6);
        CHECK(std::fabs((sunset - sunrise) - dv.day_s) < 1e-6);
        CHECK(std::fabs(cal.sunrise_tod_s(d) - dv.night_s * 0.5) < 1e-9);
        CHECK(std::fabs(cal.sunset_tod_s(d) - (dv.night_s * 0.5 + dv.day_s)) < 1e-9);
    }
    // The warped sun stands on the horizon (-0.833 deg, within the declination drift across the day) at
    // the scaled sunrise/sunset of a midsummer day whose day is 30 % longer than the real one.
    const int summer_mid = cal.season_start_day(1) + 15;
    const double t0 = cal.env_time(0, 1, 15, 0.0);
    const DayValues& dv = cal.day(summer_mid);
    CHECK(std::fabs(sun.at(cal, t0 + dv.night_s * 0.5 + 1e-3).elevation_rad * kRadToDeg + 0.833) < 0.3);
    CHECK(std::fabs(sun.at(cal, t0 + dv.night_s * 0.5 + dv.day_s - 1e-3).elevation_rad * kRadToDeg + 0.833) < 0.3);
}

TEST_CASE("W3b scaled day share: Explicit and Astronomical stay bit-identical", "[environment][calendar]") {
    // FixedSeason mode gives each day exactly one form's value, so the original closed form can be
    // compared with == (no interpolation arithmetic in between).
    const ClimateDesc plain = test_env::astronomical_four_seasons();
    for (int s = 0; s < 4; ++s) {
        CalendarOptions opt;
        opt.mode = CalendarMode::FixedSeason;
        opt.fixed_season = s;
        const Calendar cal(plain, opt);
        const double h0 = sunrise_hour_angle_rad(plain.location.latitude_deg * kDegToRad,
                                                 declination_spencer_rad(y_mid_of_season(plain.seasons[static_cast<std::size_t>(s)])));
        const double day = plain.calendar.cycle_s * (h0 / kPi);
        CHECK(cal.day(0).day_s == day);
        CHECK(cal.day(0).night_s == plain.calendar.cycle_s - day);
    }
    // A polar day (70 N, midsummer): the plain form keeps its zero-length night EXACTLY, the scaled
    // form never leaves the clamp range - this is what a clamp applied to the plain form would break.
    ClimateDesc polar = test_env::astronomical_four_seasons(70.0);
    CalendarOptions opt;
    opt.mode = CalendarMode::FixedSeason;
    opt.fixed_season = 1;
    const Calendar polar_plain(polar, opt);
    CHECK(polar_plain.day(0).day_s == 86400.0);
    CHECK(polar_plain.day(0).night_s == 0.0);
    polar.seasons[1].day_night.kind = DayNightKind::AstronomicalScaled;
    polar.seasons[1].day_night.day_share_multiplier = 1.0;
    const Calendar polar_scaled(polar, opt);
    CHECK(polar_scaled.day(0).day_s == kDayShareMax * 86400.0);
    CHECK(polar_scaled.day(0).night_s > 0.0);
    // Explicit is untouched too.
    const ClimateDesc ex = test_env::three_season_explicit();
    CalendarOptions fo;
    fo.mode = CalendarMode::FixedSeason;
    fo.fixed_season = 2;
    const Calendar ec(ex, fo);
    CHECK(ec.day(7).day_s == 43200.0);
    CHECK(ec.day(7).night_s == 43200.0);
}

TEST_CASE("W3b scaled day share: descriptor validation", "[environment][calendar]") {
    ClimateDesc d = test_env::astronomical_four_seasons();
    d.seasons[1].day_night.kind = DayNightKind::AstronomicalScaled;
    for (double bad : {0.0, -1.0, kDayShareMultiplierMax * 1.01, std::nan(""), std::numeric_limits<double>::infinity()}) {
        d.seasons[1].day_night.day_share_multiplier = bad;
        INFO("multiplier " << bad);
        CHECK_THROWS_WITH(Calendar(d), ContainsSubstring("day_share_multiplier"));
    }
    d.seasons[1].day_night.day_share_multiplier = kDayShareMultiplierMax;
    CHECK_NOTHROW(Calendar(d));
    // The scaled form needs the real day-of-year span like the plain astronomical form.
    d.seasons[1].has_doy_span = false;
    CHECK_THROWS_WITH(Calendar(d), ContainsSubstring("doy_span"));
}

TEST_CASE("W3b scaled day share: loader round trip and rejections", "[environment][io]") {
    const fs::path dir = fs::temp_directory_path() /
                         ("ps_env_w3b_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(dir);
    struct Cleanup {
        fs::path p;
        ~Cleanup() { std::error_code ec; fs::remove_all(p, ec); }
    } cleanup{dir};

    const auto doc = [](const std::string& day_night) {
        return json::parse(R"({"format": "physics_sim.climate/1", "location": {"latitude_deg": 50.15},
          "seasons": [
            {"name": "a", "days": 31, "doy_span": [60, 151], "day_night": {"astronomical": true}},
            {"name": "b", "days": 31, "doy_span": [152, 59], "day_night": )" + day_night + R"(}
          ]})");
    };
    const auto write = [&](const std::string& name, const json& j) {
        const fs::path p = dir / name;
        std::ofstream(p, std::ios::binary) << j.dump(2);
        return p;
    };

    const ClimateDesc ok = io::load_climate_json(write("ok.json", doc(R"({"astronomical": true, "day_share_multiplier": 1.2})")));
    CHECK(ok.seasons[0].day_night.kind == DayNightKind::Astronomical);
    CHECK(ok.seasons[1].day_night.kind == DayNightKind::AstronomicalScaled);
    CHECK(ok.seasons[1].day_night.day_share_multiplier == 1.2);
    CHECK_NOTHROW(Calendar(ok));

    struct Row { const char* what; const char* day_night; const char* message; };
    const Row rows[] = {
        {"multiplier without astronomical", R"({"day_share_multiplier": 1.2})", "needs \"astronomical\": true"},
        {"multiplier with explicit lengths", R"({"day_s": 50400, "night_s": 36000, "day_share_multiplier": 1.2})", "needs \"astronomical\": true"},
        {"multiplier zero", R"({"astronomical": true, "day_share_multiplier": 0})", "day_share_multiplier must be in (0, 10]"},
        {"multiplier negative", R"({"astronomical": true, "day_share_multiplier": -1})", "day_share_multiplier must be in (0, 10]"},
        {"multiplier above the cap", R"({"astronomical": true, "day_share_multiplier": 11})", "day_share_multiplier must be in (0, 10]"},
        {"multiplier not a number", R"({"astronomical": true, "day_share_multiplier": "big"})", "\"day_share_multiplier\" must be a number"},
        {"astronomical false with multiplier", R"({"astronomical": false, "day_share_multiplier": 1.2})", "must be true"},
        {"scaled plus fraction form", R"({"astronomical": true, "day_share_multiplier": 1.2, "cycle_s": 86400, "day_fraction": 0.5})", "more than one day/night form"},
    };
    int n = 0;
    for (const Row& row : rows) {
        const fs::path p = write("row" + std::to_string(n++) + ".json", doc(row.day_night));
        INFO("rejection row: " << row.what);
        CHECK_THROWS_WITH(io::load_climate_json(p), ContainsSubstring(row.message));
    }
}
