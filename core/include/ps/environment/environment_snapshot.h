// ps/environment/environment_snapshot.h - the published environment record
// (weather plan 8.1), W2 subset: time and sun. Later increments add the
// weather, surface and wind groups as further additive fields.
//
// A plain, fixed-size, trivially copyable value: filling it never allocates,
// so World::stage_snapshot can copy it into every WorldSnapshot slot inside a
// job (PLAN.md D8). It is NEVER hashed - it is a pure function of the clock
// (Environment::evaluate), and the clock anchor is what World::state_hash
// folds.

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"

#include <cstdint>

namespace ps::environment {

struct EnvironmentSnapshot {
    // --- Time ---
    real t_env_s = 0.0;            // environment time since the climate epoch
    real time_scale = 1.0;         // env seconds per sim second (0 = frozen)
    std::int64_t year = 0;
    std::int32_t season = 0;       // index into the climate's seasons
    char season_name[32] = {};     // NUL-terminated, truncated copy of the season name
    std::int32_t day_in_season = 0;
    std::int32_t day_of_year = 0;  // game day index within the year
    real time_of_day_s = 0.0;      // since the day's start (midnight), env seconds
    real day_s = 0.0;              // length of this game day's daylight
    real night_s = 0.0;
    real day_fraction = 0.0;       // time_of_day_s / (day_s + night_s), [0, 1); 0 = midnight, 0.5 = middle of the day
    real clock_hours = 0.0;        // day_fraction * 24: the 24-hour wall clock a HUD shows (a game day maps onto 24 h whatever its length)
    bool is_day = false;           // by the calendar's sunrise/sunset

    // --- Sun ---
    real sun_elevation_rad = 0.0;  // geometric, above the horizon positive
    real sun_azimuth_rad = 0.0;    // from north, clockwise, [0, 2 pi)
    Vec3 sun_dir_world;            // unit vector TOWARDS the sun in the World frame (Z up; north = +Y rotated by the environment's north yaw)
    real ghi_w_m2 = 0.0;           // global horizontal irradiance incl. cloud reduction (cloud_cover is 0 until the weather timeline exists)
    real cloud_cover = 0.0;        // [0, 1]; 0 without weather (W2), the weather's cloud cover with it

    // --- Weather (W4: set when the Environment owns a weather timeline; the
    // station-level values at the climate's reference elevation, before the
    // per-vehicle altitude lift of VehicleAir) ---
    bool has_weather = false;
    std::int32_t precip_type = 0;      // ps::environment::PrecipType as an integer (0 = none)
    real precip_mm_h = 0.0;
    real wind_mean_m_s = 0.0;
    real wind_dir_deg = 0.0;           // direction the wind blows FROM, compass degrees clockwise from north
    real gust_factor = 1.0;
    real air_temp_c = 0.0;
    real rh = 0.0;                     // [0, 1]
    real dew_point_c = 0.0;
    real pressure_msl_hpa = 0.0;
    real lightning_per_min = 0.0;
};

} // namespace ps::environment
