// tests/unit/test_environment_calendar.cpp - weather plan W1: E1 (per-day
// sunrise/sunset equal the authored day/night lengths), E2 (fixed season/day),
// E4 (non-accumulating clock). Tags [environment]; these tests link only
// ps_environment (+ ps_environment_io elsewhere), never ps_core.
//
// RECORDED SABOTAGES (each performed by hand during W1, then reverted; the
// guard named below failed with the quoted symptom):
//  - E1: calendar.cpp season midpoint `days/2` changed to `days/2 + 1` (one day
//    off): "E1 mid-season day carries the authored value" failed (day_s off by
//    the per-day slope, ~60 s) and the all-days comparison against the
//    independent reference failed on every day of the first season.
//  - E2: Calendar constructor ignoring CalendarOptions::mode (always dynamic):
//    "E2 fixed season" failed on the first day outside the season's midpoint.
//  - E4: EnvClock::env_time computing the tick offset through a float
//    (static_cast<float>): "E4 closed form at tick 2^40" failed bit-exactly.

#include "environment_fixtures.h"

#include "ps/environment/calendar.h"
#include "ps/environment/env_time.h"
#include "ps/environment/solar_geometry.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

using namespace ps;
using namespace ps::environment;
using Catch::Matchers::ContainsSubstring;

namespace {

// Independent reference for the per-day interpolation (does not share code
// with calendar.cpp): the control points are every season midpoint shifted by
// -N, 0, +N years; the value at day-centre xc is the linear interpolation
// between the nearest control point at or below it and the nearest above.
double ref_interp(const std::vector<int>& days, const std::vector<double>& v, double xc) {
    const double total = std::accumulate(days.begin(), days.end(), 0.0);
    struct P { double x, v; };
    std::vector<P> pts;
    double start = 0.0;
    for (std::size_t j = 0; j < days.size(); ++j) {
        const double mid = start + days[j] / 2.0;
        for (int k = -1; k <= 1; ++k) pts.push_back({mid + k * total, v[j]});
        start += days[j];
    }
    std::sort(pts.begin(), pts.end(), [](const P& a, const P& b) { return a.x < b.x; });
    std::size_t hi = 0;
    while (hi < pts.size() && pts[hi].x <= xc) ++hi;
    const P& lo = pts[hi - 1];
    const P& up = pts[hi];
    const double w = (xc - lo.x) / (up.x - lo.x);
    return (1.0 - w) * lo.v + w * up.v;
}

// Sunrise as the instant is_day flips, found by bisection on sample() alone
// (no use of the table's own night/2).
double bisect_transition(const Calendar& cal, double t_lo, double t_hi, bool lo_is_day) {
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (t_lo + t_hi);
        if (cal.sample(mid).is_day() == lo_is_day) t_lo = mid; else t_hi = mid;
    }
    return 0.5 * (t_lo + t_hi);
}

} // namespace

TEST_CASE("E1 day and night lengths per day follow the authored values", "[environment][calendar]") {
    const ClimateDesc desc = test_env::three_season_explicit();
    const Calendar cal(desc);
    const std::vector<int> days = {31, 91, 59};
    const std::vector<double> day_s = {50400.0, 30000.0, 43200.0};
    const std::vector<double> night_s = {36000.0, 56400.0, 43200.0};

    REQUIRE(cal.days_per_year() == 181);

    // Mid-season days (odd counts -> the midpoint sits on a day centre) carry
    // the authored value to rounding.
    int start = 0;
    for (std::size_t s = 0; s < days.size(); ++s) {
        const int mid_day = start + (days[s] - 1) / 2;
        INFO("season " << s << " mid day " << mid_day);
        CHECK(std::fabs(cal.day(mid_day).day_s - day_s[s]) < 1e-9);       // E1 mid-season day carries the authored value
        CHECK(std::fabs(cal.day(mid_day).night_s - night_s[s]) < 1e-9);
        start += days[s];
    }

    // Every day matches the independent interpolation, season boundaries included.
    for (int d = 0; d < cal.days_per_year(); ++d) {
        INFO("day " << d);
        const double xc = d + 0.5;
        CHECK(std::fabs(cal.day(d).day_s - ref_interp(days, day_s, xc)) < 1e-9);
        CHECK(std::fabs(cal.day(d).night_s - ref_interp(days, night_s, xc)) < 1e-9);
    }

    // The sunrise/sunset instants measured from sample() alone (bisection of
    // the is_day flip) are night/2 and night/2 + day after the day start.
    for (int d = 0; d < cal.days_per_year(); ++d) {
        const DayValues& dv = cal.day(d);
        const int season = dv.season;
        const double t0 = cal.env_time(0, season, dv.day_in_season, 0.0);
        const double sunrise = bisect_transition(cal, t0, t0 + dv.night_s * 0.5 + dv.day_s * 0.5, false);
        const double sunset = bisect_transition(cal, t0 + dv.night_s * 0.5 + dv.day_s * 0.5, t0 + dv.length_s() - 1e-3, true);
        INFO("day " << d);
        CHECK(std::fabs((sunrise - t0) - dv.night_s * 0.5) < 1e-6);
        CHECK(std::fabs((sunset - t0) - (dv.night_s * 0.5 + dv.day_s)) < 1e-6);
        CHECK(std::fabs((sunset - sunrise) - dv.day_s) < 1e-6);
    }
}

TEST_CASE("E1 time mapping: day boundaries, years, epoch", "[environment][calendar]") {
    const ClimateDesc desc = test_env::three_season_explicit();
    const Calendar cal(desc);

    // t_env = 0 is the epoch: season "b", day 10, 20000 s.
    const CalendarSample s0 = cal.sample(0.0);
    CHECK(s0.year == 0);
    CHECK(cal.season_name(s0.season) == "b");
    CHECK(s0.day_in_season == 10);
    CHECK(std::fabs(s0.time_of_day_s - 20000.0) < 1e-6);

    // Year length is the sum of every day's length; prefix sums consistent.
    double sum = 0.0;
    for (int d = 0; d < cal.days_per_year(); ++d) {
        CHECK(cal.day_start_in_year_s(d) == sum);
        sum += cal.day(d).length_s();
    }
    CHECK(std::fabs(cal.year_length_s() - sum) < 1e-6);

    // Every day boundary maps to the start of that day; just before it, the
    // end of the previous one (wrapping the year at day 0).
    for (int d = 0; d < cal.days_per_year(); ++d) {
        const DayValues& dv = cal.day(d);
        const double t = cal.env_time(0, dv.season, dv.day_in_season, 0.0);
        const CalendarSample at = cal.sample(t + 1e-3);
        CHECK(at.day_of_year == d);
        CHECK(at.year == 0);
        CHECK(std::fabs(at.time_of_day_s - 1e-3) < 1e-5);
        const CalendarSample before = cal.sample(t - 1e-3);
        const int prev = (d + cal.days_per_year() - 1) % cal.days_per_year();
        CHECK(before.day_of_year == prev);
        CHECK(before.year == (d == 0 ? -1 : 0));
    }

    // Round trip over years, including negative t_env.
    const struct { std::int64_t year; int season; int day; double tod; } cases[] = {
        {0, 0, 0, 0.5}, {0, 1, 45, 30000.0}, {0, 2, 58, 86000.0}, {3, 1, 0, 12345.0},
        {-2, 2, 30, 1.0}, {7, 0, 30, 70000.0}};
    for (const auto& c : cases) {
        const double t = cal.env_time(c.year, c.season, c.day, c.tod);
        const CalendarSample s = cal.sample(t);
        INFO("year " << c.year << " season " << c.season << " day " << c.day);
        CHECK(s.year == c.year);
        CHECK(s.season == c.season);
        CHECK(s.day_in_season == c.day);
        CHECK(std::fabs(s.time_of_day_s - c.tod) < 1e-5);
    }

    // Phase coordinates.
    const DayValues& dv = cal.day(70);
    const double t0 = cal.env_time(0, dv.season, dv.day_in_season, 0.0);
    const CalendarSample noon = cal.sample(t0 + dv.night_s * 0.5 + dv.day_s * 0.5);
    CHECK(noon.is_day());
    CHECK(std::fabs(noon.phase_u - 0.5) < 1e-9);
    const CalendarSample midnight_start = cal.sample(t0 + 1e-3);
    CHECK(midnight_start.phase == DayPhase::NightBefore);
    CHECK(std::fabs(midnight_start.phase_u - 0.5) < 1e-6);   // middle of the night
    const CalendarSample late = cal.sample(t0 + dv.length_s() - 1e-3);
    CHECK(late.phase == DayPhase::NightAfter);
    CHECK(std::fabs(late.phase_u - 0.5) < 1e-6);

    // Out-of-range arguments are rejected.
    CHECK_THROWS_WITH(cal.env_time(0, 3, 0, 0.0), ContainsSubstring("season out of range"));
    CHECK_THROWS_WITH(cal.env_time(0, 0, 31, 0.0), ContainsSubstring("day out of range"));
    CHECK_THROWS_WITH(cal.env_time(0, 0, 0, 1e9), ContainsSubstring("time of day"));
}

TEST_CASE("E1 interpolation of climate scalars between season midpoints", "[environment][calendar]") {
    const ClimateDesc desc = test_env::three_season_explicit();
    const Calendar cal(desc);
    const std::vector<int> days = {31, 91, 59};
    const std::vector<double> temp = {10.0, -2.0, 20.0};
    const std::vector<double> pressure = {1015.0, 1005.0, 1020.0};
    for (int d = 0; d < cal.days_per_year(); ++d) {
        const DayValues& dv = cal.day(d);
        REQUIRE(dv.has_climate);
        INFO("day " << d);
        CHECK(std::fabs(dv.climate.temp_mean_c - ref_interp(days, temp, d + 0.5)) < 1e-9);
        CHECK(std::fabs(dv.climate.pressure_msl_hpa - ref_interp(days, pressure, d + 0.5)) < 1e-9);
        REQUIRE(dv.has_authored);  // global authored block inherited by every day
        CHECK(dv.authored.e_max_deg == 60.0);
    }
}

TEST_CASE("E2 fixed season returns the season's mid values every day", "[environment][calendar]") {
    const ClimateDesc desc = test_env::three_season_explicit();
    CalendarOptions opt;
    opt.mode = CalendarMode::FixedSeason;
    opt.fixed_season = 1;
    const Calendar cal(desc, opt);
    REQUIRE(cal.days_per_year() == 181);   // the calendar still counts days
    for (int d = 0; d < cal.days_per_year(); ++d) {
        const DayValues& dv = cal.day(d);
        INFO("day " << d);
        CHECK(dv.day_s == 30000.0);        // E2 fixed season
        CHECK(dv.night_s == 56400.0);
        REQUIRE(dv.has_climate);
        CHECK(dv.climate.temp_mean_c == -2.0);
        CHECK(dv.climate.pressure_msl_hpa == 1005.0);
        CHECK(dv.y_start == dv.y_end);
    }
    // The dynamic calendar differs on the same days (the guard is not vacuous).
    const Calendar dyn(desc);
    CHECK(dyn.day(0).day_s != 30000.0);
    CHECK(dyn.day(180).night_s != 56400.0);
    // Sunrise from sample() is the same every day.
    for (int s = 0; s < 3; ++s) {
        const double t = cal.env_time(0, s, 5, 0.0);
        CHECK(cal.sample(t + 28200.0 + 1e-3).is_day());     // 56400/2 = 28200 s of night before sunrise
        CHECK_FALSE(cal.sample(t + 28200.0 - 1e-3).is_day());
    }
}

TEST_CASE("E2 fixed season, astronomical: season-mid real day of year", "[environment][calendar]") {
    const ClimateDesc desc = test_env::astronomical_four_seasons();
    CalendarOptions opt;
    opt.mode = CalendarMode::FixedSeason;
    opt.fixed_season = 1;  // summer, doy 152..243
    const Calendar cal(desc, opt);
    const double y_mid = 151.0 + 92.0 / 2.0;   // days since 1 Jan of the span's middle
    const double h0 = sunrise_hour_angle_rad(desc.location.latitude_deg * kDegToRad, declination_spencer_rad(y_mid));
    const double day_s = 86400.0 * h0 / kPi;
    for (int d = 0; d < cal.days_per_year(); ++d) {
        CHECK(std::fabs(cal.day(d).day_s - day_s) < 1e-9);
        CHECK(std::fabs(cal.day(d).night_s - (86400.0 - day_s)) < 1e-9);
        CHECK(cal.day(d).y_start == y_mid);
        CHECK(cal.day(d).y_end == y_mid);
    }
    // 50.15 N at midsummer: ~16 h of day.
    CHECK(day_s / 3600.0 > 15.5);
    CHECK(day_s / 3600.0 < 16.6);
}

TEST_CASE("E2 fixed day carries one day's values on every day", "[environment][calendar]") {
    const ClimateDesc desc = test_env::three_season_explicit();
    const Calendar dyn(desc);
    CalendarOptions opt;
    opt.mode = CalendarMode::FixedDay;
    opt.fixed_day = 40;   // inside season "b", not its midpoint
    const Calendar cal(desc, opt);
    for (int d = 0; d < cal.days_per_year(); ++d) {
        CHECK(cal.day(d).day_s == dyn.day(40).day_s);
        CHECK(cal.day(d).night_s == dyn.day(40).night_s);
        CHECK(cal.day(d).climate.temp_mean_c == dyn.day(40).climate.temp_mean_c);
    }
    CHECK(cal.day(5).season == 0);          // natural season labels stay
    CHECK(cal.day(5).day_in_season == 5);
}

TEST_CASE("Calendar construction rejections", "[environment][calendar]") {
    ClimateDesc d = test_env::three_season_explicit();
    CHECK_THROWS_WITH(Calendar(d, CalendarOptions{CalendarMode::FixedSeason, 3, 0}), ContainsSubstring("fixed_season"));
    CHECK_THROWS_WITH(Calendar(d, CalendarOptions{CalendarMode::FixedDay, 0, 181}), ContainsSubstring("fixed_day"));
    d.calendar.epoch.time_of_day_s = 90000.0;   // season b day 10 is 86400 s long
    CHECK_THROWS_WITH(Calendar(d), ContainsSubstring("beyond the length"));
    d = test_env::three_season_explicit();
    d.seasons[1].days = 0;
    CHECK_THROWS_WITH(Calendar(d), ContainsSubstring("days must be >= 1"));
}

TEST_CASE("E4 the clock is a non-accumulating closed form", "[environment][clock]") {
    // Powers of two everywhere (tick_hz 256, scale 2): the exact result is
    // anchor + (tick - anchor_tick) / 128, computable in integers and
    // representable, so the comparison is bit-exact against a route that
    // shares no arithmetic with env_time().
    EnvClock c;
    c.anchor_env_s = 1024.0;
    c.anchor_tick = 128;
    c.tick_hz = 256.0;
    c.scale = 2.0;
    c.validate();
    // 2^40 plus a multiple of 128 that is NOT a multiple of float's spacing at
    // 2^40 (2^17): a float conversion of the tick offset rounds it.
    const std::int64_t tick = (std::int64_t{1} << 40) + 128 + 128 * 12345;
    const double expected = 1024.0 + static_cast<double>((tick - 128) / 128);
    CHECK(c.env_time(tick) == expected);     // E4 closed form at tick 2^40

    // Random access: the value for a tick does not depend on what was
    // evaluated before it.
    const std::int64_t ticks[] = {tick, 0, 77, tick + 12800, 128, 5};
    double first[6];
    for (int i = 0; i < 6; ++i) first[i] = c.env_time(ticks[i]);
    for (int i = 5; i >= 0; --i) CHECK(c.env_time(ticks[i]) == first[i]);

    // Negative control: an accumulating clock (add dt every tick) drifts away
    // from the closed form - that is what the bit-exact check above rules out.
    EnvClock g;   // 240 Hz, scale 1
    double acc = 0.0;
    const std::int64_t n = 1 << 24;
    for (std::int64_t i = 0; i < n; ++i) acc += 1.0 / 240.0;
    CHECK(acc != g.env_time(n));
    CHECK(std::fabs(acc - g.env_time(n)) < 1e-3);   // still the same number, to rounding drift

    // scale 0 freezes the environment.
    EnvClock z = c;
    z.scale = 0.0;
    CHECK(z.env_time(0) == 1024.0);
    CHECK(z.env_time(tick) == 1024.0);
}

TEST_CASE("E4 clock jumps, scale changes and validation", "[environment][clock]") {
    EnvClock c;
    c.tick_hz = 240.0;
    c.scale = 30.0;
    CHECK(std::fabs(c.env_time(240) - 30.0) < 1e-12);

    // set_scale re-anchors at the value the old scale gave: no jump.
    const double before = c.env_time(2400);
    c.set_scale(60.0, 2400);
    CHECK(c.env_time(2400) == before);
    CHECK(std::fabs(c.env_time(2640) - (before + 60.0)) < 1e-9);

    // set_time is a jump.
    c.set_time(1.0e6, 3000);
    CHECK(c.env_time(3000) == 1.0e6);
    CHECK(std::fabs(c.env_time(3240) - (1.0e6 + 60.0)) < 1e-6);

    EnvClock bad = c;
    bad.tick_hz = 0.0;
    CHECK_THROWS_WITH(bad.validate(), ContainsSubstring("tick_hz"));
    bad = c;
    bad.scale = -1.0;
    CHECK_THROWS_WITH(bad.validate(), ContainsSubstring("scale"));
    bad = c;
    bad.anchor_env_s = std::nan("");
    CHECK_THROWS_WITH(bad.validate(), ContainsSubstring("anchor_env_s"));
}
