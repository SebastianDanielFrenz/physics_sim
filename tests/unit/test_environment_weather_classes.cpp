// tests/unit/test_environment_weather_classes.cpp - weather plan W3 test T3:
// the class thresholds (WMO-No. 8 intensity, DWD Starkregen / Boeen, Beaufort,
// TORRO) at their exact boundaries, from the shipped
// data/weather_classes/wmo_dwd_torro.json, and classify()'s mapping of a
// WeatherSample. Tags [environment][weather].
//
// RECORDED SABOTAGES (performed by hand, reverted) - measured output in
// out/weather_w3_progress.md:
//  - ClassScale::level with Ge implemented as '>': 23 boundary checks fail
//    (every 'ge' threshold, e.g. drizzle 0.1 -> level 0 instead of 1).
//  - classify() without the km/h conversion of the gust: the three gust
//    checks fail (level 0 instead of 5, no Unwetter).

#include "ps/environment/weather_classes.h"
#include "ps/environment/weather_io.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <string>

using namespace ps;
using namespace ps::environment;

namespace {

const WeatherClassesDesc& shipped() {
    static const WeatherClassesDesc d =
        ps::io::load_weather_classes_json(std::string(PS_SOURCE_DIR) + "/data/weather_classes/wmo_dwd_torro.json");
    return d;
}

real below(real x) { return std::nextafter(x, -std::numeric_limits<real>::infinity()); }
real above(real x) { return std::nextafter(x, std::numeric_limits<real>::infinity()); }

// level just below, exactly at and just above a threshold.
void check_boundary(const ClassScale& s, real x, int lvl_below, int lvl_at, int lvl_above) {
    INFO("threshold " << x);
    CHECK(s.level(below(x)) == lvl_below);
    CHECK(s.level(x) == lvl_at);
    CHECK(s.level(above(x)) == lvl_above);
}

} // namespace

TEST_CASE("T3: class thresholds at their exact boundaries", "[environment][weather]") {
    const WeatherClassesDesc& d = shipped();
    // WMO drizzle: light < 0.1 <= moderate < 0.5 <= heavy.
    check_boundary(d.drizzle_mm_h, 0.1, 0, 1, 1);
    check_boundary(d.drizzle_mm_h, 0.5, 1, 2, 2);
    // WMO rain: light < 2.5 <= moderate < 10 <= heavy <= 50 < violent.
    check_boundary(d.rain_mm_h, 2.5, 0, 1, 1);
    check_boundary(d.rain_mm_h, 10.0, 1, 2, 2);
    check_boundary(d.rain_mm_h, 50.0, 2, 2, 3);
    // WMO snow: light < 1 <= moderate <= 5 < heavy.
    check_boundary(d.snow_mm_h, 1.0, 0, 1, 1);
    check_boundary(d.snow_mm_h, 5.0, 1, 1, 2);
    // DWD Starkregen 1 h: 15 / 25 (both inclusive, the higher level on 25) / > 40.
    check_boundary(d.dwd_rain_1h_mm, 15.0, 0, 1, 1);
    check_boundary(d.dwd_rain_1h_mm, 25.0, 1, 2, 2);
    check_boundary(d.dwd_rain_1h_mm, 40.0, 2, 2, 3);
    check_boundary(d.dwd_rain_6h_mm, 20.0, 0, 1, 1);
    check_boundary(d.dwd_rain_6h_mm, 35.0, 1, 2, 2);
    check_boundary(d.dwd_rain_6h_mm, 60.0, 2, 2, 3);
    // Beaufort: a speed above an upper boundary enters the next force.
    const real bft[12] = {0.2, 1.5, 3.3, 5.4, 7.9, 10.7, 13.8, 17.1, 20.7, 24.4, 28.4, 32.6};
    for (int k = 0; k < 12; ++k) check_boundary(d.beaufort_m_s, bft[k], k, k, k + 1);
    CHECK(d.beaufort_m_s.level(0.0) == 0);
    CHECK(d.beaufort_m_s.level(40.0) == 12);
    // DWD gusts (km/h): > 50, from 65, 90, 105, 120, > 140.
    check_boundary(d.dwd_gust_km_h, 50.0, 0, 0, 1);
    check_boundary(d.dwd_gust_km_h, 65.0, 1, 2, 2);
    check_boundary(d.dwd_gust_km_h, 90.0, 2, 3, 3);
    check_boundary(d.dwd_gust_km_h, 105.0, 3, 4, 4);
    check_boundary(d.dwd_gust_km_h, 120.0, 4, 5, 5);
    check_boundary(d.dwd_gust_km_h, 140.0, 5, 5, 6);
    // TORRO size code = level - 1: 5 mm is code 0, 100 mm code 9, above 100 code 10.
    const real torro[10] = {5.0, 10.0, 16.0, 21.0, 31.0, 41.0, 51.0, 61.0, 76.0, 91.0};
    for (int k = 0; k < 10; ++k) check_boundary(d.torro_hail_mm, torro[k], k, k + 1, k + 1);
    check_boundary(d.torro_hail_mm, 100.0, 10, 10, 11);
}

TEST_CASE("classify: intensity per type, cloudburst, warnings, hail code, Unwetter", "[environment][weather]") {
    const WeatherClassesDesc& d = shipped();
    WeatherSample s;
    s.precip_type = PrecipType::None;
    WeatherClasses c = classify(d, s);
    CHECK(c.intensity == PrecipIntensity::None);
    CHECK_FALSE(c.cloudburst);
    CHECK(c.torro_hail_code == -1);

    s.precip_type = PrecipType::Rain;
    s.precip_mm_h = 2.5;
    CHECK(classify(d, s).intensity == PrecipIntensity::Moderate);
    s.precip_mm_h = 0.3;
    CHECK(classify(d, s).intensity == PrecipIntensity::Light);
    s.precip_mm_h = 60.0;
    c = classify(d, s);
    CHECK(c.intensity == PrecipIntensity::Violent);
    CHECK_FALSE(c.cloudburst);
    s.precip_mm_h = 100.0;
    CHECK(classify(d, s).cloudburst);   // >= 100 mm/h (Q17)
    s.precip_mm_h = below(100.0);
    CHECK_FALSE(classify(d, s).cloudburst);

    s.precip_type = PrecipType::Drizzle;
    s.precip_mm_h = 0.5;
    CHECK(classify(d, s).intensity == PrecipIntensity::Heavy);
    s.precip_type = PrecipType::Snow;
    s.precip_mm_h = 3.0;
    CHECK(classify(d, s).intensity == PrecipIntensity::Moderate);
    s.precip_type = PrecipType::FreezingRain;
    s.precip_mm_h = 3.0;
    CHECK(classify(d, s).intensity == PrecipIntensity::Moderate);   // rain scale [AS]

    // DWD rain: the larger of the 1 h and 6 h levels; Unwetter from heftiger Starkregen.
    s.precip_1h_mm = 16.0;
    s.precip_6h_mm = 36.0;
    c = classify(d, s);
    CHECK(c.dwd_rain_level == 2);
    CHECK(c.unwetter);
    s.precip_6h_mm = 21.0;
    c = classify(d, s);
    CHECK(c.dwd_rain_level == 1);
    CHECK_FALSE(c.unwetter);

    // Wind: Beaufort on the mean, DWD level on the nominal gust in km/h.
    s.wind_mean_m_s = 22.0;
    s.gust_nominal_m_s = 22.0 * 1.6;   // 126.72 km/h: Orkanboeen
    c = classify(d, s);
    CHECK(c.beaufort == 9);
    CHECK(c.dwd_gust_level == 5);
    CHECK(c.unwetter);
    s.gust_nominal_m_s = 20.0;   // 72 km/h: Sturmboeen
    c = classify(d, s);
    CHECK(c.dwd_gust_level == 2);
    CHECK_FALSE(c.unwetter);

    s.precip_type = PrecipType::Hail;
    s.hail_diameter_mm = 25.0;
    c = classify(d, s);
    CHECK(c.torro_hail_code == 3);
    s.lightning_per_min = 0.5;
    CHECK(classify(d, s).thunderstorm);
}
