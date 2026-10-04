// ps/environment/weather_classes.h - intensity classes (weather plan 3.3, W3).
//
// Raw values (mm/h liquid equivalent, m/s, mm) are what physics uses; classes
// are labels for HUD / visuals / the owner's phenomenon list. Every threshold
// is DATA (physics_sim.weather_classes/1, docs/data_formats/weather_classes.md,
// shipped data/weather_classes/wmo_dwd_torro.json with its citations: WMO-No. 8,
// DWD Warnkriterien, TORRO) - owner question Q17 (the cloudburst threshold) is
// a data change.
//
// A Scale is an ascending threshold list with one comparison rule per
// threshold: Ge (value >= threshold enters the next level) or Gt (value >
// threshold). level(v) = number of thresholds passed, 0 .. thresholds.size().

#pragma once

#include "ps/environment/weather.h"
#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::environment {

enum class ThresholdRule : std::uint8_t { Ge, Gt };

struct ClassScale {
    std::vector<real> thresholds;      // strictly ascending, finite, >= 0
    std::vector<ThresholdRule> rules;  // same size as thresholds
    int level(real value) const;
};

struct WeatherClassesDesc {
    ClassScale drizzle_mm_h;        // 2 thresholds: light / moderate / heavy
    ClassScale rain_mm_h;           // 3: light / moderate / heavy / violent (also sleet, freezing rain [AS])
    ClassScale snow_mm_h;           // 2: light / moderate / heavy (water equivalent)
    real cloudburst_mm_h = 100.0;   // instantaneous rate >= this (Q17)
    ClassScale dwd_rain_1h_mm;      // 3: Starkregen / heftiger / extrem heftiger
    ClassScale dwd_rain_6h_mm;      // 3
    int dwd_rain_unwetter_level = 2; // level (1..3) from which a rain warning is an Unwetter
    ClassScale beaufort_m_s;        // 12: Beaufort 0 .. 12 (10-min mean wind)
    ClassScale dwd_gust_km_h;       // 6: Windboeen .. extreme Orkanboeen (nominal gust)
    int dwd_gust_unwetter_level = 4; // level (1..6) from which a gust warning is an Unwetter
    ClassScale torro_hail_mm;       // 11: size code = level - 1 (level 0 = below code 0)
    std::string source;
};

std::string weather_classes_validation_error(const WeatherClassesDesc& d);
void validate_weather_classes(const WeatherClassesDesc& d);  // throws std::invalid_argument

enum class PrecipIntensity : std::uint8_t { None, Light, Moderate, Heavy, Violent };

struct WeatherClasses {
    PrecipType type = PrecipType::None;      // copied from the sample
    PrecipIntensity intensity = PrecipIntensity::None;  // drizzle/snow scales stop at Heavy
    bool cloudburst = false;
    int dwd_rain_level = 0;    // 0 none, 1 Starkregen, 2 heftiger, 3 extrem heftiger (max of the 1 h and 6 h levels)
    int beaufort = 0;          // 0 .. 12
    int dwd_gust_level = 0;    // 0 none, 1 Windboeen .. 6 extreme Orkanboeen
    int torro_hail_code = -1;  // -1 no hail / below code 0, else 0 .. 10
    bool thunderstorm = false; // lightning rate > 0
    bool unwetter = false;     // a DWD rain or gust level at or above its Unwetter level
};

WeatherClasses classify(const WeatherClassesDesc& d, const WeatherSample& s);

} // namespace ps::environment
