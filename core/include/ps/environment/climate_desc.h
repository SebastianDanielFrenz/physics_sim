// ps/environment/climate_desc.h - climate descriptor (weather plan 7.1).
//
// Descriptors only (PLAN.md D2 seam style): the time / sun / season parts of
// physics_sim.climate/1 (W1) and the weather statistics (W3: regimes,
// transition matrices, lags, precipitation phase, humidity, diurnal
// temperature, lightning geometry - types in weather_desc.h) and the W4 gust
// block. Condition-grid settings arrive with W5 and are REJECTED by the
// loader until then (see docs/data_formats/climate.md), so a file written for
// a later increment never silently loads with half its meaning dropped.
//
// Every numeric field is plain data. Nothing here is a physical constant of
// code: values come from a data file with its own source note.

#pragma once

#include "ps/environment/weather_desc.h"
#include "ps/types.h"

#include <optional>
#include <string>
#include <vector>

namespace ps::environment {

inline constexpr real kPi = 3.14159265358979323846;
inline constexpr real kDegToRad = kPi / 180.0;
inline constexpr real kRadToDeg = 180.0 / kPi;

struct LocationDesc {
    real latitude_deg = 0.0;   // [-90, 90]
    real longitude_deg = 0.0;  // [-180, 180]; informational in W1 (the clock is local mean solar time)
    real elevation_m = 0.0;    // informational in W1
};

enum class SunMode : std::uint8_t {
    Astronomical,  // real declination + hour angle for the latitude, warped onto the game's day/night lengths (default)
    Authored       // freely authored elevation curve (fantasy calendars)
};

// Authored sun curve parameters (degrees): daytime elevation = e_max sin(pi u),
// night elevation = -e_night sin(pi v), azimuth linear from az_rise to az_set
// through south (north for southern latitudes) by day and on around the
// circle at night.
struct SunAuthoredDesc {
    real e_max_deg = 60.0;
    real e_night_deg = 30.0;
    real az_rise_deg = 90.0;
    real az_set_deg = 270.0;
};

// Per-season override of the global authored block; unset members inherit.
struct SunAuthoredPartial {
    std::optional<real> e_max_deg;
    std::optional<real> e_night_deg;
    std::optional<real> az_rise_deg;
    std::optional<real> az_set_deg;
};

enum class DayNightKind : std::uint8_t {
    Explicit,       // day_s + night_s
    CycleFraction,  // cycle_s * day_fraction
    Astronomical    // day fraction from latitude and the real day of year, with the calendar's global cycle_s
};

struct DayNightDesc {
    DayNightKind kind = DayNightKind::Astronomical;
    real day_s = 0.0;          // Explicit
    real night_s = 0.0;        // Explicit
    real cycle_s = 0.0;        // CycleFraction
    real day_fraction = 0.0;   // CycleFraction, in (0, 1)
};

// Mid-season climate scalars (interpolated per day by the calendar; the W3
// weather timeline reads them: diurnal temperature from temp_mean_c /
// temp_range_k, sea-level pressure from pressure_msl_hpa). Present on every
// season or on none; required when the climate has weather statistics.
struct SeasonClimateDesc {
    real temp_mean_c = 0.0;
    real temp_range_k = 0.0;      // >= 0
    real rh_mean = 0.0;           // [0, 1]
    real pressure_msl_hpa = 0.0;  // > 0
    real ground_temp_c = 0.0;
};

struct SeasonDesc {
    std::string name;
    int days = 1;                 // >= 1
    // Real day-of-year span the season stands for, 1-based inclusive. doy_start >
    // doy_end means the span wraps over the new year (a northern winter).
    // Required for the astronomical sun and the astronomical day length.
    bool has_doy_span = false;
    int doy_start = 1;
    int doy_end = 1;
    DayNightDesc day_night;
    std::optional<SeasonClimateDesc> climate;
    SunAuthoredPartial sun_authored;
    // W3: regimes / transitions / initial_regime. On every season or on none.
    std::optional<SeasonWeatherDesc> weather;
};

struct CalendarEpochDesc {
    int year = 0;
    std::string season;           // empty = the first season
    int day = 0;                  // day within that season
    real time_of_day_s = 0.0;     // seconds since the start of that day (midnight)
};

struct CalendarDesc {
    CalendarEpochDesc epoch;
    real cycle_s = 86400.0;       // global day+night length (astronomical day/night forms; informational otherwise)
};

struct ClimateDesc {
    LocationDesc location;
    SunMode sun_mode = SunMode::Astronomical;
    std::optional<SunAuthoredDesc> sun_authored;  // global authored block (required for SunMode::Authored unless every season completes it)
    CalendarDesc calendar;
    std::vector<SeasonDesc> seasons;
    // W3 weather model blocks. lags_s is required when the seasons carry
    // regimes; precip_phase, humidity and diurnal_temperature are required by
    // any WeatherTimeline (dynamic or preset mode); lightning when a regime or
    // preset has lightning_per_min > 0.
    std::optional<WeatherLagsDesc> lags;
    std::optional<PrecipPhaseDesc> precip_phase;
    std::optional<HumidityDesc> humidity;
    std::optional<DiurnalTempDesc> diurnal_temperature;
    std::optional<LightningDesc> lightning;
    // W4: gust synthesis (wind.h); absent = steady mean wind.
    std::optional<GustDesc> gusts;
    std::string source;

    bool has_weather_statistics() const { return !seasons.empty() && seasons.front().weather.has_value(); }
};

// Structural validation (the loader checks JSON types, this checks meaning).
// climate_validation_error returns the first problem ("climate: <field> ...")
// or an empty string; validate_climate throws it as std::invalid_argument.
// Calendar's constructor calls validate_climate, so a code-built descriptor is
// checked exactly like a loaded one.
std::string climate_validation_error(const ClimateDesc& desc);
void validate_climate(const ClimateDesc& desc);

} // namespace ps::environment
