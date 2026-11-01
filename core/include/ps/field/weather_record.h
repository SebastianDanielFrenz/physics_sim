// ps/field/weather_record.h - the daily weather record (farming plan 3, 4.2).
//
// One record per game day, aggregated from ps_environment's WeatherTimeline
// (temperature, humidity, wind, cloud, precipitation) and Sun (irradiance). A
// record is a PURE function of (climate, seed, day, config): it does not
// depend on which days were asked for before or in what order. The source
// caches records in blocks so a field that catches up after 500x time
// compression, or several fields on the same site, share one computation.
//
// Day index: day 0 is the first game day of the climate's epoch year (the
// calendar's day_of_year 0 of epoch.year); day d is year d / days_per_year,
// day_of_year d % days_per_year. A day whose start lies before the climate's
// t_env = 0 (an epoch in mid-year) is refused.
//
// Aggregation, per day, N equal intervals (config.samples_per_day):
//   - Tmin/Tmax/Tmean(of the extremes): extremes of the interval-midpoint samples;
//   - rain/snow: the exact published precipitation integral of each interval,
//     split by the precipitation type at the interval midpoint (sleet is split
//     config.sleet_snow_fraction snow, freezing rain and hail count as rain);
//   - Rs: the mean of Haurwitz/Kasten-Czeplak irradiance (ps::environment::ghi_w_m2)
//     times 86400 s, so a game day shorter or longer than 24 h still reports the
//     flux of a 24 h day [AS];
//   - wind: mean of the sampled mean wind (taken to be at config.wind_height_m),
//     converted to 2 m with FAO-56 eq 47;
//   - ea: mean sampled vapour pressure; RH mean/min/max from the samples;
//   - ET0: FAO-56 Penman-Monteith (et0.h) at the climate's latitude/elevation.
//
// Not thread safe (the timeline cursor is a mutable cache).

#pragma once

#include "ps/environment/calendar.h"
#include "ps/environment/climate_desc.h"
#include "ps/environment/sun.h"
#include "ps/environment/weather.h"
#include "ps/types.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace ps::field {

struct DailyWeather {
    std::int64_t day = 0;
    int doy = 1;                // real day of year 1..366 the game day stands for
    real tmin_c = 0.0, tmax_c = 0.0, tmean_c = 0.0;   // tmean = (tmin + tmax) / 2
    real rain_mm = 0.0;         // liquid (water equivalent)
    real snow_mm = 0.0;         // snow (water equivalent)
    real rs_mj_m2 = 0.0;
    real wind2_m_s = 0.0;
    real rh_min = 0.0, rh_max = 0.0, rh_mean = 0.0;   // fractions
    real ea_kpa = 0.0;
    real daylight_h = 0.0;
    real et0_mm = 0.0;
    // Energy-limited evaporation of a wet, unshaded soil surface (mm/day): Priestley-Taylor, alpha 1.26 x
    // Delta/(Delta+gamma) x Rn/lambda, Rn floored at 0 (F1c fix A). Negative = not provided (hand-made
    // weather in tests): the soil evaporation is then limited by the dual crop coefficient alone.
    real e_wet_mm = -1.0;

    real precip_mm() const { return rain_mm + snow_mm; }
};

struct DailyWeatherConfig {
    int samples_per_day = 48;          // >= 4; each interval must be <= 6 h
    real wind_height_m = 10.0;         // height the weather's wind speed refers to [AS]
    real sleet_snow_fraction = 0.5;    // [AS]
    real rs_reference_day_s = 86400.0; // see the file header
};

class DailyWeatherSource {
public:
    // The climate must carry weather statistics (WeatherTimeline's rules).
    DailyWeatherSource(const environment::ClimateDesc& climate, std::uint64_t seed,
                       const DailyWeatherConfig& config = {});

    // The cached record for `day` (>= 0). The reference stays valid until the
    // next call that computes a record of the same block (it never moves while
    // the source lives: blocks are heap allocated once).
    const DailyWeather& record(std::int64_t day);
    // Fills the cache for [first, last] so later record() calls allocate nothing.
    void prefetch(std::int64_t first, std::int64_t last);
    // Computes a record without touching the cache (reference for tests).
    DailyWeather compute(std::int64_t day);

    const environment::ClimateDesc& climate() const { return climate_; }
    const environment::Calendar& calendar() const { return calendar_; }
    std::uint64_t seed() const { return seed_; }
    int days_per_year() const { return calendar_.days_per_year(); }
    std::int64_t computed_count() const { return computed_; }

    static constexpr int kBlockDays = 64;

private:
    struct Block {
        DailyWeather rec[kBlockDays];
        bool have[kBlockDays] = {};
    };

    environment::ClimateDesc climate_;
    environment::Calendar calendar_;
    environment::Sun sun_;
    environment::WeatherTimeline timeline_;
    environment::WeatherCursor cursor_;
    DailyWeatherConfig config_;
    std::uint64_t seed_ = 0;
    std::vector<std::unique_ptr<Block>> blocks_;
    std::int64_t computed_ = 0;
};

} // namespace ps::field
