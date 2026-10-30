// core/src/field/weather_record.cpp - see ps/field/weather_record.h.

#include "ps/field/weather_record.h"

#include "ps/field/et0.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::field {

using namespace ps::environment;

DailyWeatherSource::DailyWeatherSource(const ClimateDesc& climate, std::uint64_t seed,
                                       const DailyWeatherConfig& config)
    : climate_(climate),
      calendar_(climate),
      sun_(climate),
      timeline_(climate, seed),
      config_(config),
      seed_(seed) {
    if (config_.samples_per_day < 4) throw std::invalid_argument("DailyWeatherConfig: samples_per_day must be >= 4");
    if (!(config_.wind_height_m > 0.1)) throw std::invalid_argument("DailyWeatherConfig: wind_height_m must exceed 0.1 m");
    if (config_.sleet_snow_fraction < 0.0 || config_.sleet_snow_fraction > 1.0)
        throw std::invalid_argument("DailyWeatherConfig: sleet_snow_fraction must be in [0, 1]");
    if (!(config_.rs_reference_day_s > 0.0)) throw std::invalid_argument("DailyWeatherConfig: rs_reference_day_s must be > 0");
}

DailyWeather DailyWeatherSource::compute(std::int64_t day) {
    if (day < 0) throw std::invalid_argument("DailyWeatherSource: day must be >= 0");
    const int n_year = calendar_.days_per_year();
    const std::int64_t year = day / n_year;
    const int doy_game = static_cast<int>(day % n_year);
    const DayValues& dv = calendar_.day(doy_game);
    const real t0 = static_cast<real>(year) * calendar_.year_length_s() + calendar_.day_start_in_year_s(doy_game) -
                    calendar_.epoch_offset_s();
    if (t0 < 0.0) throw std::invalid_argument("DailyWeatherSource: day " + std::to_string(day) + " starts before the climate epoch");
    const real len = dv.length_s();
    const int n = config_.samples_per_day;
    const real h = len / n;
    if (h > WeatherTimeline::kMaxAccumulationWindowS)
        throw std::invalid_argument("DailyWeatherSource: sampling interval exceeds the 6 h precipitation window");

    DailyWeather r;
    r.day = day;
    // y runs on past 365 for a season that wraps over the new year (a winter
    // that starts in December): fold it back to a real day of year.
    r.doy = static_cast<int>(std::fmod(std::floor(0.5 * (dv.y_start + dv.y_end)), 365.0)) + 1;
    r.daylight_h = dv.day_s / 3600.0;

    real tmin = 1.0e300, tmax = -1.0e300;
    real rh_sum = 0.0, rh_min = 1.0e300, rh_max = -1.0e300;
    real ea_sum = 0.0, wind_sum = 0.0, ghi_sum = 0.0;
    real rain = 0.0, snow = 0.0;
    for (int k = 0; k < n; ++k) {
        const real ta = t0 + k * h;
        const real tm = ta + 0.5 * h;
        const real tb = ta + h;
        const WeatherSample w = timeline_.evaluate(tm, cursor_);
        tmin = std::min(tmin, w.air_temp_c);
        tmax = std::max(tmax, w.air_temp_c);
        rh_sum += w.rh;
        rh_min = std::min(rh_min, w.rh);
        rh_max = std::max(rh_max, w.rh);
        ea_sum += w.vapour_pressure_hpa * 0.1;
        wind_sum += w.wind_mean_m_s;
        const SunState s = sun_.at(calendar_, tm);
        ghi_sum += ghi_w_m2(s.elevation_rad, w.cloud_cover);
        const real p = timeline_.precipitation_mm(ta, tb, cursor_);
        if (p > 0.0) {
            switch (w.precip_type) {
                case PrecipType::Snow: snow += p; break;
                case PrecipType::Sleet:
                    snow += p * config_.sleet_snow_fraction;
                    rain += p * (1.0 - config_.sleet_snow_fraction);
                    break;
                default: rain += p; break;
            }
        }
    }
    r.tmin_c = tmin;
    r.tmax_c = tmax;
    r.tmean_c = 0.5 * (tmin + tmax);
    r.rain_mm = rain;
    r.snow_mm = snow;
    r.rh_mean = rh_sum / n;
    r.rh_min = rh_min;
    r.rh_max = rh_max;
    r.ea_kpa = ea_sum / n;
    r.rs_mj_m2 = ghi_sum / n * config_.rs_reference_day_s * 1.0e-6;
    r.wind2_m_s = wind_speed_2m(wind_sum / n, config_.wind_height_m);

    Et0Input in;
    in.tmin_c = r.tmin_c;
    in.tmax_c = r.tmax_c;
    in.ea_kpa = r.ea_kpa;
    in.u2_m_s = r.wind2_m_s;
    in.rs_mj_m2 = r.rs_mj_m2;
    in.latitude_deg = climate_.location.latitude_deg;
    in.elevation_m = climate_.location.elevation_m;
    in.day_of_year = r.doy;
    r.et0_mm = fao56_et0(in).et0_mm;
    ++computed_;
    return r;
}

const DailyWeather& DailyWeatherSource::record(std::int64_t day) {
    if (day < 0) throw std::invalid_argument("DailyWeatherSource: day must be >= 0");
    const std::size_t b = static_cast<std::size_t>(day / kBlockDays);
    const int i = static_cast<int>(day % kBlockDays);
    if (b >= blocks_.size()) blocks_.resize(b + 1);
    if (!blocks_[b]) blocks_[b] = std::make_unique<Block>();
    Block& blk = *blocks_[b];
    if (!blk.have[i]) {
        blk.rec[i] = compute(day);
        blk.have[i] = true;
    }
    return blk.rec[i];
}

void DailyWeatherSource::prefetch(std::int64_t first, std::int64_t last) {
    for (std::int64_t d = first; d <= last; ++d) (void)record(d);
}

} // namespace ps::field
