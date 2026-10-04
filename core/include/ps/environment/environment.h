// ps/environment/environment.h - the environment object (weather plan K1/K2,
// 1.3, 11.2), W2 subset: clock + calendar + sun. It will own the weather
// timeline and the surface-condition grid in later increments; the public
// surface below is the part a game server and World::install_environment use
// and does not change shape when those arrive.
//
// Time model (plan 2.1): environment time is a pure, non-accumulating function
// of the sim tick (EnvClock). The Environment keeps NO per-tick history: its
// only mutable state is the clock anchor (changed by set_time/set_time_scale),
// the calendar/sun are immutable after construction, and the per-tick
// EnvironmentSnapshot is a cache of the pure evaluate(t_env).
//
// Hashed state (fold_state_hash): the clock anchor {anchor_env_s, anchor_tick,
// tick_hz, scale} and the calendar mode. Everything else is a closed-form
// function of those plus the immutable climate.
//
// Not thread safe: World calls begin_tick from the sim thread; a game server
// uses its own instance.

#pragma once

#include "ps/environment/calendar.h"
#include "ps/environment/climate_desc.h"
#include "ps/environment/env_time.h"
#include "ps/environment/environment_snapshot.h"
#include "ps/environment/sun.h"
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
};

class Environment {
public:
    // Throws std::invalid_argument for a bad config (unknown season, both
    // fixed modes, bad scale, an epoch/start outside its day) or a bad climate
    // (Calendar's validation).
    explicit Environment(const ClimateDesc& climate, const EnvironmentConfig& config = {});

    const ClimateDesc& climate() const { return climate_; }
    const Calendar& calendar() const { return calendar_; }
    const Sun& sun() const { return sun_; }
    const EnvironmentConfig& config() const { return config_; }
    const EnvClock& clock() const { return clock_; }
    // t_env at which the configured start sits (config.start, else 0).
    real start_env_s() const { return start_env_s_; }

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

    // --- Per-tick evaluation ---
    // Evaluates this tick's snapshot (cached in current()). Allocation-free.
    void begin_tick(std::int64_t tick);
    const EnvironmentSnapshot& current() const { return current_; }
    // The pure function begin_tick caches: the snapshot for an arbitrary env
    // time at the CURRENT clock scale. Allocation-free.
    EnvironmentSnapshot evaluate(real t_env) const;

    // FNV-1a fold of the hashed state (see the file header).
    std::uint64_t fold_state_hash(std::uint64_t h) const;

private:
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
};

} // namespace ps::environment
