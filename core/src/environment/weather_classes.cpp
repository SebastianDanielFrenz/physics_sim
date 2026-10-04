// core/src/environment/weather_classes.cpp - see ps/environment/weather_classes.h.

#include "ps/environment/weather_classes.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::environment {

int ClassScale::level(real v) const {
    int lvl = 0;
    for (std::size_t i = 0; i < thresholds.size(); ++i) {
        const bool passed = rules[i] == ThresholdRule::Ge ? v >= thresholds[i] : v > thresholds[i];
        if (!passed) break;
        lvl = static_cast<int>(i) + 1;
    }
    return lvl;
}

namespace {

std::string scale_error(const ClassScale& s, const std::string& name, std::size_t count) {
    const std::string p = "weather_classes: " + name;
    if (s.thresholds.size() != count) return p + " must have " + std::to_string(count) + " thresholds";
    if (s.rules.size() != s.thresholds.size()) return p + ": rules must match the thresholds";
    for (std::size_t i = 0; i < s.thresholds.size(); ++i) {
        if (!std::isfinite(s.thresholds[i]) || s.thresholds[i] < 0.0) return p + " thresholds must be finite and >= 0";
        if (i > 0 && !(s.thresholds[i] > s.thresholds[i - 1])) return p + " thresholds must be strictly ascending";
    }
    return {};
}

} // namespace

std::string weather_classes_validation_error(const WeatherClassesDesc& d) {
    std::string e;
    if (!(e = scale_error(d.drizzle_mm_h, "drizzle_mm_h", 2)).empty()) return e;
    if (!(e = scale_error(d.rain_mm_h, "rain_mm_h", 3)).empty()) return e;
    if (!(e = scale_error(d.snow_mm_h, "snow_mm_h", 2)).empty()) return e;
    if (!(e = scale_error(d.dwd_rain_1h_mm, "dwd_rain_1h_mm", 3)).empty()) return e;
    if (!(e = scale_error(d.dwd_rain_6h_mm, "dwd_rain_6h_mm", 3)).empty()) return e;
    if (!(e = scale_error(d.beaufort_m_s, "beaufort_m_s", 12)).empty()) return e;
    if (!(e = scale_error(d.dwd_gust_km_h, "dwd_gust_km_h", 6)).empty()) return e;
    if (!(e = scale_error(d.torro_hail_mm, "torro_hail_mm", 11)).empty()) return e;
    if (!std::isfinite(d.cloudburst_mm_h) || !(d.cloudburst_mm_h > 0.0))
        return "weather_classes: cloudburst_mm_h must be > 0";
    if (d.dwd_rain_unwetter_level < 1 || d.dwd_rain_unwetter_level > 3)
        return "weather_classes: dwd_rain_1h_mm/6h unwetter level must be in [1, 3]";
    if (d.dwd_gust_unwetter_level < 1 || d.dwd_gust_unwetter_level > 6)
        return "weather_classes: dwd_gust_km_h unwetter level must be in [1, 6]";
    return {};
}

void validate_weather_classes(const WeatherClassesDesc& d) {
    const std::string e = weather_classes_validation_error(d);
    if (!e.empty()) throw std::invalid_argument(e);
}

WeatherClasses classify(const WeatherClassesDesc& d, const WeatherSample& s) {
    WeatherClasses c;
    c.type = s.precip_type;
    const real p = s.precip_mm_h;
    int lvl = -1;   // 0-based intensity on the applicable scale
    switch (s.precip_type) {
    case PrecipType::None:
        break;
    case PrecipType::Drizzle:
        lvl = d.drizzle_mm_h.level(p);
        break;
    case PrecipType::Snow:
        lvl = d.snow_mm_h.level(p);
        break;
    case PrecipType::Rain:
    case PrecipType::Sleet:
    case PrecipType::FreezingRain:
    case PrecipType::Hail:
        lvl = d.rain_mm_h.level(p);
        break;
    }
    c.intensity = lvl < 0 ? PrecipIntensity::None : static_cast<PrecipIntensity>(lvl + 1);
    c.cloudburst = s.precip_type != PrecipType::None && p >= d.cloudburst_mm_h;
    const int l1 = d.dwd_rain_1h_mm.level(s.precip_1h_mm);
    const int l6 = d.dwd_rain_6h_mm.level(s.precip_6h_mm);
    c.dwd_rain_level = l1 > l6 ? l1 : l6;
    c.beaufort = d.beaufort_m_s.level(s.wind_mean_m_s);
    c.dwd_gust_level = d.dwd_gust_km_h.level(s.gust_nominal_m_s * 3.6);
    c.torro_hail_code = s.precip_type == PrecipType::Hail ? d.torro_hail_mm.level(s.hail_diameter_mm) - 1 : -1;
    c.thunderstorm = s.lightning_per_min > 0.0;
    c.unwetter = c.dwd_rain_level >= d.dwd_rain_unwetter_level || c.dwd_gust_level >= d.dwd_gust_unwetter_level;
    return c;
}

} // namespace ps::environment
