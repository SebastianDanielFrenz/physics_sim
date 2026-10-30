// tests/unit/test_field_weather_record.cpp - the daily weather record
// (ps/field/weather_record.h): a pure function of (climate, seed, day), cached,
// aggregated consistently with ps_environment's WeatherTimeline.
//
// Recorded sabotages (F1a, 2026-10-31), each made in core/src/field/weather_record.cpp,
// the test run, then reverted:
//  - W-a: the rain/snow accumulators of compute() made `static` (state leaking
//    from one day to the next): W1 (order independence), W2 (timeline integral)
//    and W4 (statistics) failed;
//  - W-b: the last of the 48 intervals of the day dropped: W2 failed;
//  - W-c: ET0 computed with a fixed 2 m/s wind instead of the record's own:
//    W3 failed;
//  - W-d: Rs scaled by 1.3: W4 (30-year statistics) failed;
//  - W-e: the pre-epoch check removed: W6 failed.
#include "field_test_support.h"

#include "ps/environment/weather.h"
#include "ps/field/et0.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
bool same(const DailyWeather& a, const DailyWeather& b) {
    return a.day == b.day && a.doy == b.doy && a.tmin_c == b.tmin_c && a.tmax_c == b.tmax_c && a.rain_mm == b.rain_mm &&
           a.snow_mm == b.snow_mm && a.rs_mj_m2 == b.rs_mj_m2 && a.wind2_m_s == b.wind2_m_s && a.rh_min == b.rh_min &&
           a.rh_max == b.rh_max && a.rh_mean == b.rh_mean && a.ea_kpa == b.ea_kpa && a.et0_mm == b.et0_mm;
}
} // namespace

TEST_CASE("W1 weather record is a pure function of (climate, seed, day)", "[field][weather]") {
    DailyWeatherSource fwd(test_field::farm_climate(), 5);
    DailyWeatherSource scrambled(test_field::farm_climate(), 5);
    std::vector<DailyWeather> a;
    for (int d = 0; d < 400; ++d) a.push_back(fwd.record(d));
    // other order, with jumps backward (the timeline cursor restarts its walk)
    for (int d : {399, 17, 250, 3, 399, 120, 0, 300, 64, 63, 65}) CHECK(same(scrambled.record(d), a[static_cast<std::size_t>(d)]));
    for (int d = 399; d >= 0; --d) CHECK(same(scrambled.record(d), a[static_cast<std::size_t>(d)]));
    // a different seed gives a different weather
    DailyWeatherSource other(test_field::farm_climate(), 6);
    int differing = 0;
    for (int d = 0; d < 60; ++d) differing += !same(other.record(d), a[static_cast<std::size_t>(d)]);
    CHECK(differing > 50);
}

TEST_CASE("W5 cache: a second request recomputes nothing", "[field][weather]") {
    DailyWeatherSource src(test_field::farm_climate(), 5);
    src.prefetch(0, 199);
    const std::int64_t n = src.computed_count();
    CHECK(n == 200);
    const DailyWeather* p = &src.record(77);
    for (int i = 0; i < 3; ++i) (void)src.record(i * 50);
    CHECK(src.computed_count() == n);
    CHECK(&src.record(77) == p);               // the reference is stable
    CHECK_THROWS_AS(src.record(-1), std::invalid_argument);
}

TEST_CASE("W6 a day before the climate epoch is refused", "[field][weather]") {
    // test_weather.json starts in the middle of a season (epoch summer, day 14): game day 0 of the year lies before t_env = 0.
    const auto c = ps::io::load_climate_json(test_field::data_dir() / "climates/test_weather.json");
    DailyWeatherSource src(c, 1);
    CHECK_THROWS_AS(src.record(0), std::invalid_argument);
}

TEST_CASE("W2 precipitation matches the timeline integral", "[field][weather]") {
    const environment::ClimateDesc& c = test_field::farm_climate();
    DailyWeatherSource src(c, 9);
    environment::WeatherTimeline tl(c, 9);
    environment::WeatherCursor cur;
    double sum_rec = 0.0, sum_tl = 0.0;
    for (int d = 0; d < 365; ++d) {
        const DailyWeather& w = src.record(d);
        const auto& cal = src.calendar();
        const int doy_game = d % src.days_per_year();
        const double t0 = (d / src.days_per_year()) * cal.year_length_s() + cal.day_start_in_year_s(doy_game) - cal.epoch_offset_s();
        const double len = cal.day(doy_game).length_s();
        double p = 0.0;
        for (int k = 0; k < 48; ++k) p += tl.precipitation_mm(t0 + len / 48 * k, t0 + len / 48 * (k + 1), cur);
        sum_rec += w.precip_mm();
        sum_tl += p;
        CHECK_THAT(w.precip_mm(), WithinAbs(p, 1e-9 + 1e-9 * p));   // exact up to the summation order
    }
    CHECK(sum_rec > 300.0);
    CHECK_THAT(sum_rec, WithinRel(sum_tl, 1e-12));
}

TEST_CASE("W3 ET0 is the FAO-56 function of the record's own fields", "[field][weather]") {
    DailyWeatherSource src(test_field::farm_climate(), 5);
    for (int d = 0; d < 365; d += 7) {
        const DailyWeather& w = src.record(d);
        Et0Input in;
        in.tmin_c = w.tmin_c;
        in.tmax_c = w.tmax_c;
        in.ea_kpa = w.ea_kpa;
        in.u2_m_s = w.wind2_m_s;
        in.rs_mj_m2 = w.rs_mj_m2;
        in.latitude_deg = test_field::farm_climate().location.latitude_deg;
        in.elevation_m = test_field::farm_climate().location.elevation_m;
        in.day_of_year = w.doy;
        CHECK(w.et0_mm == fao56_et0(in).et0_mm);
        CHECK(w.doy >= 1);
        CHECK(w.doy <= 365);
        CHECK(w.tmax_c >= w.tmin_c);
    }
    // the u2 conversion: the weather's mean wind (taken at 10 m) is reduced to 2 m
    const DailyWeather& w = src.record(200);
    CHECK(w.wind2_m_s > 0.0);
}

TEST_CASE("W4 climate statistics of the shipped farm climate (30 years)", "[field][weather]") {
    DailyWeatherSource src(test_field::farm_climate(), 21);
    const int years = 30;
    double rain = 0, et0 = 0, tmean = 0;
    double rs_jan = 0, rs_jul = 0;
    int n_jan = 0, n_jul = 0;
    for (int d = 0; d < years * 365; ++d) {
        const DailyWeather& w = src.record(d);
        rain += w.precip_mm();
        et0 += w.et0_mm;
        tmean += w.tmean_c;
        if (w.doy >= 1 && w.doy <= 31) { rs_jan += w.rs_mj_m2; ++n_jan; }
        if (w.doy >= 182 && w.doy <= 212) { rs_jul += w.rs_mj_m2; ++n_jul; }
    }
    rain /= years;
    et0 /= years;
    tmean /= years * 365;
    INFO("annual rain " << rain << " mm, ET0 " << et0 << " mm, mean temperature " << tmean << " C, Rs Jan "
                        << rs_jan / n_jan << " Jul " << rs_jul / n_jul);
    // The climate generator's own targets ([AS], recalled Rhein-Main normals): ~620 mm, ~10.4 C, ~650-700 mm ET0,
    // global radiation 2.4 / 19.2 MJ m-2 day-1 in January / July.
    CHECK(rain > 560.0);
    CHECK(rain < 700.0);
    CHECK(tmean > 9.5);
    CHECK(tmean < 11.8);
    CHECK(et0 > 620.0);
    CHECK(et0 < 790.0);
    CHECK_THAT(rs_jan / n_jan, WithinRel(2.4, 0.25));
    CHECK_THAT(rs_jul / n_jul, WithinRel(19.2, 0.12));
}
