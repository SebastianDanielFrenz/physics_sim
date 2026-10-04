// ps/environment/weather_desc.h - weather descriptors (weather plan W3,
// sections 3.1-3.9, 7.1 and 7.3). Descriptors only, no algorithm code.
//
// The regime / transition / lag parts of physics_sim.climate/1
// (docs/data_formats/climate.md) and the physics_sim.weather_preset/1 format
// (docs/data_formats/weather_preset.md). Every number is data: the [AS]
// starting values, cited thresholds and model constants live in the data
// files with their own source notes, never in code.

#pragma once

#include "ps/types.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ps::environment {

// x = median * exp(sigma_log * n), n an approximate standard normal
// (Irwin-Hall n = 4, bounded to +-3.46 sigma - see weather.h).
struct LogNormalDesc {
    real median = 1.0;     // > 0
    real sigma_log = 0.0;  // >= 0
};

// Uniform draw in [lo, hi] (lo <= hi; lo == hi is a constant).
struct RangeDesc {
    real lo = 0.0;
    real hi = 0.0;
};

// Segment duration in hours: max(min_h, median_h * exp(sigma_log * n)).
struct DurationDesc {
    real min_h = 1.0;      // >= 0.1 (the cursor's 64-segment ring covers the 6 h accumulation window)
    real median_h = 6.0;   // > 0
    real sigma_log = 0.0;  // >= 0
};

// Mean wind speed (10-min mean at 10 m), Weibull: c * (-ln(1 - u))^(1/k).
struct WeibullDesc {
    real k = 2.0;  // > 0
    real c = 1.0;  // >= 0 (0 = calm)
};

// Wind direction ("from", meteorological degrees): mean + spread * (2u - 1).
struct WindDirDesc {
    real mean_deg = 0.0;
    real spread_deg = 0.0;  // [0, 180]
};

// How a regime's precipitation is typed (plan 3.2: the type is decided from
// the air temperature at the moment, except for the two explicit forms).
enum class PrecipForm : std::uint8_t {
    Auto,          // snow / sleet / rain by air temperature (precip_phase thresholds)
    Drizzle,       // drizzle above the snow threshold, snow at or below it
    FreezingRain   // freezing rain while the air is below 0 degC, rain otherwise
};

struct HailDesc {
    real probability = 0.0;      // per cell, [0, 1]
    LogNormalDesc diameter_mm;   // maximum hailstone diameter of the cell
    real min_shape = 0.5;        // hail falls while the cell's shape >= this, (0, 1]
};

// Convective cells inside a segment (plan 3.1 layer 2). Cells follow each
// other gap - cell - gap - cell ...; a cell that would end after its segment's
// end is not started (so cells never cross a segment boundary). Shape: a
// triangle rising linearly to the peak at rise_fraction of the cell's
// duration, then falling linearly to 0.
struct ConvectiveDesc {
    LogNormalDesc cell_duration_min;
    LogNormalDesc gap_min;
    LogNormalDesc peak_mm_h;
    real rise_fraction = 0.3;     // (0, 1)
    real gust_factor = 1.5;       // gust factor at the cell's peak, >= 1
    std::optional<HailDesc> hail;
};

struct RegimeDesc {
    DurationDesc duration;
    RangeDesc cloud_cover;                    // [0, 1]
    std::optional<LogNormalDesc> precip_mm_h; // stratiform rate target; none = 0
    PrecipForm precip_form = PrecipForm::Auto;
    std::optional<ConvectiveDesc> convective;
    WeibullDesc wind_mean_m_s;
    WindDirDesc wind_dir_deg;
    real gust_factor = 1.0;                   // >= 1 (nominal gust = mean * factor; W4 synthesises gusts)
    RangeDesc temp_offset_k;
    RangeDesc rh;                             // [0, 1]
    RangeDesc pressure_offset_hpa;
    real lightning_per_min = 0.0;             // [0, 30]; convective regimes: during cells only
};

struct NamedRegime {
    std::string name;
    RegimeDesc regime;
};

struct TransitionDesc {
    std::string to;
    real p = 0.0;  // [0, 1]
};

struct TransitionRowDesc {
    std::string from;
    std::vector<TransitionDesc> to;  // sums to 1 within 1e-9
};

// Per-season weather statistics. Every defined regime has a row; rows name
// only regimes defined in this season. When the chain enters a season in
// which the current regime is not defined, the next regime is drawn from the
// row of that season's initial_regime instead.
struct SeasonWeatherDesc {
    std::vector<NamedRegime> regimes;
    std::vector<TransitionRowDesc> transitions;
    std::string initial_regime;
};

// First-order lag time constants (plan 3.1), seconds, > 0.
struct WeatherLagsDesc {
    real cloud_s = 1200.0;
    real precip_on_s = 300.0;
    real precip_off_s = 600.0;
    real wind_s = 900.0;
    real temp_offset_s = 3600.0;
    real rh_s = 1800.0;
    real pressure_s = 3600.0;
};

// Precipitation phase by air temperature (plan 3.2).
struct PrecipPhaseDesc {
    real snow_below_c = 0.0;    // T <= this: snow
    real rain_above_c = 2.0;    // T >= this: rain; between: sleet (> snow_below_c)
    real min_rate_mm_h = 0.01;  // below this total rate the published type is None (> 0)
};

// RH forced up during precipitation (plan 3.4): rh = rh_lag +
// (max(rh_lag, floor) - rh_lag) * clamp(P / precip_full_mm_h, 0, 1).
struct HumidityDesc {
    real precip_rh_floor = 0.95;   // [0, 1]
    real precip_full_mm_h = 0.5;   // > 0
};

// Diurnal air temperature, Parton & Logan 1981 (plan 3.4). a_h / c_h are the
// paper's lags in hours for a reference day+night length; for a day of
// another length they scale with (day_s + night_s) / reference_cycle_s.
struct DiurnalTempDesc {
    real a_h = 1.86;                   // > 0, lag of the maximum
    real b = 2.2;                      // > 0, night decay coefficient
    real c_h = -0.17;                  // lag of the minimum after sunrise
    real reference_cycle_s = 86400.0;  // > 0
    real cloud_range_reduction = 0.6;  // diurnal range * (1 - this * N), [0, 1]
};

// Lightning event geometry (plan 3.9): distance from the cell centre, area-
// uniform in [min, max] km, bearing uniform.
struct LightningDesc {
    real distance_min_km = 0.5;  // >= 0
    real distance_max_km = 15.0; // >= min
};

// ---- physics_sim.weather_preset/1 (plan 7.3) -------------------------------

enum class PresetPrecipType : std::uint8_t { Auto, Rain, Snow, Sleet, FreezingRain, Drizzle, Hail };

struct WeatherPresetDesc {
    std::string regime;            // label (matched against the climate's regime names, may be unknown)
    real cloud_cover = 0.0;        // [0, 1]
    real precip_mm_h = 0.0;        // >= 0
    PresetPrecipType precip_type = PresetPrecipType::Auto;
    real hail_diameter_mm = 0.0;   // > 0 iff precip_type == Hail
    real wind_mean_m_s = 0.0;      // >= 0
    real wind_dir_deg = 0.0;       // [0, 360]
    real gust_factor = 1.0;        // >= 1
    bool temp_is_override = false; // true: air temperature pinned to temp_override_c
    real temp_offset_k = 0.0;
    real temp_override_c = 0.0;
    real rh = 0.7;                 // [0, 1]
    real pressure_offset_hpa = 0.0;
    real lightning_per_min = 0.0;  // [0, 30]
    std::string source;
};

// First problem as "weather_preset: <field> ..." or empty.
std::string weather_preset_validation_error(const WeatherPresetDesc& p);
void validate_weather_preset(const WeatherPresetDesc& p);  // throws std::invalid_argument

// Shared limits (algorithmic, not physical).
inline constexpr real kMinSegmentDurationH = 0.1;
inline constexpr real kMaxLightningPerMin = 30.0;

} // namespace ps::environment
