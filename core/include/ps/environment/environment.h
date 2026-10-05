// ps/environment/environment.h - the environment object (weather plan K1/K2,
// 1.3, 11.2): clock + calendar + sun (W2), and - when the config asks for it -
// the weather timeline (W3) plus the atmosphere/wind coupling (W4). It will own
// the surface-condition grid in later increments; the public surface below is
// the part a game server and World::install_environment use and does not change
// shape when those arrive.
//
// Time model (plan 2.1): environment time is a pure, non-accumulating function
// of the sim tick (EnvClock). The Environment keeps NO per-tick history: its
// only mutable state is the clock anchor (changed by set_time/set_time_scale),
// the calendar/sun/weather are immutable after construction, and the per-tick
// EnvironmentSnapshot / air state are caches of pure functions of
// (tick, clock anchor, climate, seed, config).
//
// Weather and atmosphere (W4). With EnvironmentConfig::weather != Off the
// Environment owns a WeatherTimeline (seed = EnvironmentConfig::seed). Each tick
// begin_tick also derives the SEA-LEVEL reference atmosphere from the weather
// sample (sea-level pressure, the station temperature lifted to sea level on
// the ISA lapse, the station vapour pressure lifted likewise), the mean wind
// vector in the World frame and - when the climate has a gusts block - the gust
// amplitudes. sample_air(position) then yields what one vehicle sits in: the
// ISA lift to the chassis altitude (location.elevation_m + altitude_offset_m +
// world z), the humid-air density and wind = mean + gust at that position.
// Gusts run on the SIM clock (tick / tick_hz), the weather on environment time
// (owner question Q6, PROVISIONAL, weather plan 3.8).
//
// Hashed state (fold_state_hash): the clock anchor {anchor_env_s, anchor_tick,
// tick_hz, scale} and the calendar mode; ONLY when weather is on, additionally
// the weather timeline's fold_hash (seed, mode, preset, overrides), the
// altitude offset and the gusts flag. Everything else is a closed-form
// function of those plus the immutable climate. An environment without weather
// hashes exactly as before W4.
//
// Surface conditions (W5): with EnvironmentConfig::surface_conditions the
// Environment owns a ConditionGrid (surface_condition.h) - the one piece of
// state that is NOT a closed-form function of the clock: per surface class a
// temperature column stepped on a fixed env-time grid. begin_tick advances it
// (forward steps or a spin-up after a jump), and fold_state_hash folds it.
// An environment without surface conditions hashes exactly as before W5.
//
// Not thread safe: World calls begin_tick from the sim thread; a game server
// uses its own instance. sample_air/evaluate are const but read/refresh the
// weather cursor cache (mutable), so they must not be called concurrently.

#pragma once

#include "ps/environment/atmosphere.h"
#include "ps/environment/calendar.h"
#include "ps/environment/climate_desc.h"
#include "ps/environment/env_time.h"
#include "ps/environment/environment_snapshot.h"
#include "ps/environment/sun.h"
#include "ps/environment/surface_condition.h"
#include "ps/environment/vehicle_air.h"
#include "ps/environment/weather.h"
#include "ps/environment/wind.h"
#include "ps/types.h"

#include <cstdint>
#include <optional>
#include <string>

namespace ps::environment {

// A calendar position the clock starts at (instead of the climate epoch).
struct EnvironmentStart {
    std::int64_t year = 0;
    std::string season;          // empty = the first season
    int day = 0;                 // day within the season
    real time_of_day_s = 0.0;    // since the day's start (midnight), env seconds
};

// How the environment obtains its weather (W4 owns the plumbing; the model is W3).
enum class EnvironmentWeather : std::uint8_t {
    Off,      // no weather timeline: the World's own ambient/density/wind stay in charge
    Dynamic,  // the climate's regime chain
    Preset    // EnvironmentConfig::weather_preset held constant
};

struct EnvironmentConfig {
    // Calendar mode (plan 2.2): both empty/negative = dynamic (the calendar
    // advances through the seasons). A season name pins every day to that
    // season's mid-season values; fixed_day_of_year pins every day to one
    // game day's values. Mutually exclusive.
    std::string fixed_season;
    int fixed_day_of_year = -1;
    // Where the clock starts: unset = t_env 0 (the climate epoch).
    std::optional<EnvironmentStart> start;
    real time_scale = 1.0;       // env seconds per sim second, >= 0 finite
    // Rotation of "north" about the World's Z axis, degrees counter-clockwise
    // from +Y (so 0 = north is +Y, east is +X). The World frame is Z up.
    real north_yaw_deg = 0.0;

    // --- W4 ---
    EnvironmentWeather weather = EnvironmentWeather::Off;
    WeatherPresetDesc weather_preset;  // the base in Preset mode
    // Seed of the weather chain and of the gust phases (counter-hash keyed).
    std::uint64_t seed = 0;
    // The World's z = 0 sits at location.elevation_m + altitude_offset_m above
    // sea level; vehicles lift/lower the atmosphere by their chassis z from there.
    real altitude_offset_m = 0.0;

    // --- W5: surface conditions ---
    // Present = the environment owns a ConditionGrid (requires weather != Off
    // and the seasons' climate scalars). Grid settings: the climate's
    // "condition" block (defaults when absent).
    std::optional<SurfaceConditionsDesc> surface_conditions;
    // The surface classes, index = SurfaceId (a World passes its SurfaceTable
    // names in id order). Empty = the desc's classes in name order.
    std::vector<std::string> surface_names;
};

class Environment {
public:
    // Throws std::invalid_argument for a bad config (unknown season, both
    // fixed modes, bad scale, an epoch/start outside its day, weather without
    // the climate blocks the timeline needs) or a bad climate (Calendar's
    // validation).
    explicit Environment(const ClimateDesc& climate, const EnvironmentConfig& config = {});
    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    const ClimateDesc& climate() const { return climate_; }
    const Calendar& calendar() const { return calendar_; }
    const Sun& sun() const { return sun_; }
    const EnvironmentConfig& config() const { return config_; }
    const EnvClock& clock() const { return clock_; }
    // t_env at which the configured start sits (config.start, else 0).
    real start_env_s() const { return start_env_s_; }

    // W4: true iff the environment owns a weather timeline, i.e. it provides the
    // atmosphere (ambient pressure/temperature/humidity, density, wind) a World
    // would otherwise take from set_ambient/set_aero_environment.
    bool provides_atmosphere() const { return weather_.has_value(); }
    const WeatherTimeline* weather() const { return weather_ ? &*weather_ : nullptr; }
    WeatherTimeline* weather() { return weather_ ? &*weather_ : nullptr; }
    const GustField* gusts() const { return gust_ ? &*gust_ : nullptr; }

    // --- Clock ---
    // Anchors the clock so that env time == start_env_s() at `tick` with the
    // given tick rate and the configured scale. World::install_environment
    // calls this with the world's current tick (a mid-run install is a
    // "weather starts now" event). Also evaluates the snapshot for `tick`.
    void attach(std::int64_t tick, real tick_hz);
    // Jump: env time is `env_s` at `tick` from now on.
    void set_time(real env_s, std::int64_t tick);
    // Scale change without a time jump (re-anchors, see EnvClock::set_scale).
    void set_time_scale(real scale, std::int64_t tick);

    // --- W3b: manual weather and the season lock (owner answer Q12) ---
    // The owner's model: a manual weather choice overrides the weather for the REST OF THE CURRENT
    // SEASON and resets at the next season; seasons can be locked. All of these are "now" events at
    // `tick` (the tick the next step runs at), re-evaluate the cached tick state and invalidate the
    // gust advection cache; they throw std::logic_error without a weather timeline.
    //  - add_season_weather_override: the preset starts now with a smoothstep ramp-in of ramp_s and
    //    ends with the natural season (ramp-out finishing at the boundary, see
    //    WeatherTimeline::add_season_override); with a locked season it lasts until removed. Returns
    //    the override id (std::length_error at 16 overrides, std::invalid_argument for a bad preset).
    //  - end_weather_overrides: the soft end ("back to the natural weather"): every override stops now
    //    and ramps out over its own ramp_s (an override added this very tick is removed instead).
    //    remove_weather_override / clear_weather_overrides: the immediate end - the override leaves the
    //    list and the published weather is the timeline's from this tick on (an exact reset for tests
    //    and servers; the weather steps back if the override had moved it).
    int add_season_weather_override(const WeatherPresetDesc& target, real ramp_s, std::int64_t tick);
    bool remove_weather_override(int id, std::int64_t tick);
    void clear_weather_overrides(std::int64_t tick);
    void end_weather_overrides(std::int64_t tick);
    //  - set_calendar_lock: lock the season (fixed_season = its name), pin one game day
    //    (fixed_day_of_year >= 0) or unlock (empty / -1 = Dynamic). Re-anchors the clock so the game day
    //    index and the fraction of the day are preserved (the clock hours do not jump; env seconds do
    //    when the day lengths differ between the modes). Season-bound overrides keep their elapsed time
    //    and have their end re-resolved. Works without weather too (the calendar always exists).
    //    Throws std::invalid_argument (unknown season, both set) before changing anything.
    void set_calendar_lock(const std::string& fixed_season, int fixed_day_of_year, std::int64_t tick);

    // --- Per-tick evaluation ---
    // Evaluates this tick's snapshot (cached in current()) and the air state
    // sample_air reads. Allocation-free.
    void begin_tick(std::int64_t tick);
    const EnvironmentSnapshot& current() const { return current_; }
    // The pure function begin_tick caches: the snapshot for an arbitrary env
    // time at the CURRENT clock scale. Allocation-free.
    EnvironmentSnapshot evaluate(real t_env) const;

    // W4: the air at a World position for the tick begin_tick last evaluated.
    // valid == false (all else default) without weather. Allocation-free; two
    // ps::math::pow/exp (the ISA lift) plus the gust evaluation (96 cos per
    // axis set, doubled while crossfading a window boundary).
    VehicleAir sample_air(const Vec3& position_world) const;
    // The sim time the gusts of the current tick are evaluated at.
    real gust_time_s() const { return air_.t_gust; }
    // The mean wind vector (World frame, direction the air moves toward) of this tick.
    Vec3 mean_wind_world() const { return air_.mean_wind; }
    // The along-wind standard deviation the gusts of this tick are synthesised with.
    real gust_sigma_u() const { return air_.sigma_u; }

    // --- W5: surface conditions (weather plan 4, 11.2) ---
    // The grid advances inside begin_tick (and so inside every clock change
    // above): 0 or a few steps per tick, a forward jump up to
    // max_fast_forward_s stepped, anything else a spin-up (surface_condition.h).
    bool has_surface_conditions() const { return grid_.has_value(); }
    const ConditionGrid* surface_conditions() const { return grid_ ? &*grid_ : nullptr; }
    // Number of grid steps the last begin_tick took (spin-up included); a
    // diagnostic, not hashed.
    std::int64_t last_condition_steps() const { return last_condition_steps_; }
    // Throws std::logic_error without surface conditions.
    ConditionCheckpoint checkpoint_conditions() const;
    // Restores the grid, then evaluates `tick` (which steps the grid forward
    // to the clock). Throws std::logic_error without surface conditions,
    // std::invalid_argument when the checkpoint does not fit, lies ahead of the
    // clock at `tick`, or more than max_fast_forward_s behind it (the state
    // would be discarded by a spin-up); nothing changes then.
    void restore_conditions(const ConditionCheckpoint& checkpoint, std::int64_t tick);
    // The surface forcing at an env time (what a grid step at that midpoint
    // uses): the sun, the weather and the day's ground temperature.
    SurfaceForcing surface_forcing(real t_env) const;

    // FNV-1a fold of the hashed state (see the file header). W5: with surface
    // conditions additionally the grid's fold_hash.
    std::uint64_t fold_state_hash(std::uint64_t h) const;

private:
    // The grid's view of this environment (forcing from weather/sun/calendar).
    struct ForcingSource final : ISurfaceForcingSource {
        const Environment* env = nullptr;
        SurfaceForcing forcing_at(real t_env) const override { return env->surface_forcing(t_env); }
        InitialProfile initial_at(real t_env) const override { return env->initial_profile(t_env); }
    };
    InitialProfile initial_profile(real t_env) const;
    void fill_surfaces(real t_env);

    struct AirState {
        bool valid = false;
        SeaLevelAtmosphere sea;       // reference the ISA lift starts from
        Vec3 mean_wind;               // World frame, toward
        real mean_speed = 0.0;
        real sigma_u = 0.0;
        real t_gust = 0.0;            // sim seconds
        GustField::Amplitudes amps;
        GustField::Blend blend;
        real dir_x = 1.0, dir_y = 0.0;   // advection direction of the current window
        real adv_speed = 0.0;
    };

    EnvironmentSnapshot evaluate_impl(real t_env, WeatherSample* weather_out) const;
    void update_air(std::int64_t tick, real t_env, const WeatherSample& w);
    void invalidate_gust_cache() { adv_valid_ = false; }

    ClimateDesc climate_;
    EnvironmentConfig config_;
    Calendar calendar_;
    Sun sun_;
    EnvClock clock_;
    real start_env_s_ = 0.0;
    // World-frame unit vectors of local east and north (north_yaw applied).
    Vec3 east_world_{1.0, 0.0, 0.0};
    Vec3 north_world_{0.0, 1.0, 0.0};
    EnvironmentSnapshot current_;

    // W4
    std::optional<WeatherTimeline> weather_;
    std::optional<GustField> gust_;
    mutable WeatherCursor cursor_;
    AirState air_;
    GustField::Phases phases_cur_, phases_prev_;
    // Advection velocity of the frozen gust pattern for one window (pure
    // function of (window, clock anchor); recomputed whenever the clock changes).
    bool adv_valid_ = false;
    std::int64_t adv_window_ = 0;
    real adv_dir_x_ = 1.0, adv_dir_y_ = 0.0, adv_speed_ = 0.0;

    // W5
    std::optional<ConditionGrid> grid_;
    ForcingSource forcing_;
    mutable WeatherCursor grid_cursor_;   // own cursor: grid midpoints (and spin-ups) run apart from the tick time
    std::int64_t last_condition_steps_ = 0;
};

} // namespace ps::environment
