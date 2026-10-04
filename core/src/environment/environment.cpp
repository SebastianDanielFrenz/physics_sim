// ps/environment/environment.h implementation (weather plan W2).

#include "ps/environment/environment.h"

#include "ps/math/transcendental.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace ps::environment {

namespace {

CalendarOptions make_calendar_options(const ClimateDesc& climate, const EnvironmentConfig& config) {
    CalendarOptions o;
    const bool has_season = !config.fixed_season.empty();
    const bool has_day = config.fixed_day_of_year >= 0;
    if (has_season && has_day)
        throw std::invalid_argument("environment: fixed_season and fixed_day_of_year are mutually exclusive");
    if (has_season) {
        int idx = -1;
        for (std::size_t i = 0; i < climate.seasons.size(); ++i)
            if (climate.seasons[i].name == config.fixed_season) { idx = static_cast<int>(i); break; }
        if (idx < 0) throw std::invalid_argument("environment: unknown fixed_season '" + config.fixed_season + "'");
        o.mode = CalendarMode::FixedSeason;
        o.fixed_season = idx;
    } else if (has_day) {
        o.mode = CalendarMode::FixedDay;
        o.fixed_day = config.fixed_day_of_year; // range-checked by Calendar
    }
    return o;
}

void fold_bytes(std::uint64_t& h, const void* p, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ULL;
    }
}

template <class T>
void fold_value(std::uint64_t& h, const T& v) { fold_bytes(h, &v, sizeof(T)); }

} // namespace

Environment::Environment(const ClimateDesc& climate, const EnvironmentConfig& config)
    : climate_(climate),
      config_(config),
      calendar_(climate_, make_calendar_options(climate_, config_)),
      sun_(climate_) {
    if (!(config_.time_scale >= 0.0) || !std::isfinite(config_.time_scale))
        throw std::invalid_argument("environment: time_scale must be >= 0 and finite");
    if (!std::isfinite(config_.north_yaw_deg))
        throw std::invalid_argument("environment: north_yaw_deg must be finite");

    if (config_.start) {
        const EnvironmentStart& s = *config_.start;
        int season = 0;
        if (!s.season.empty()) {
            season = calendar_.season_index(s.season);
            if (season < 0) throw std::invalid_argument("environment: unknown start season '" + s.season + "'");
        }
        // Calendar::env_time range-checks day and time of day.
        start_env_s_ = calendar_.env_time(s.year, season, s.day, s.time_of_day_s);
    } else {
        start_env_s_ = 0.0;
    }

    real sy = 0.0, cy = 1.0;
    math::sincos(config_.north_yaw_deg * kDegToRad, sy, cy);
    east_world_ = Vec3{cy, sy, 0.0};
    north_world_ = Vec3{-sy, cy, 0.0};

    clock_.anchor_env_s = start_env_s_;
    clock_.anchor_tick = 0;
    clock_.tick_hz = 240.0;
    clock_.scale = config_.time_scale;
    clock_.validate();
    current_ = evaluate(clock_.env_time(0));
}

void Environment::attach(std::int64_t tick, real tick_hz) {
    EnvClock c;
    c.anchor_env_s = start_env_s_;
    c.anchor_tick = tick;
    c.tick_hz = tick_hz;
    c.scale = config_.time_scale;
    c.validate(); // before assigning: a throw leaves the clock untouched
    clock_ = c;
    begin_tick(tick);
}

void Environment::set_time(real env_s, std::int64_t tick) {
    EnvClock c = clock_;
    c.set_time(env_s, tick);
    c.validate();
    clock_ = c;
    begin_tick(tick);
}

void Environment::set_time_scale(real scale, std::int64_t tick) {
    EnvClock c = clock_;
    c.set_scale(scale, tick);
    c.validate();
    clock_ = c;
    begin_tick(tick);
}

void Environment::begin_tick(std::int64_t tick) { current_ = evaluate(clock_.env_time(tick)); }

EnvironmentSnapshot Environment::evaluate(real t_env) const {
    const CalendarSample cs = calendar_.sample(t_env);
    const SunState ss = sun_.at(calendar_, cs);

    EnvironmentSnapshot e;
    e.t_env_s = t_env;
    e.time_scale = clock_.scale;
    e.year = cs.year;
    e.season = cs.season;
    const std::string& name = calendar_.season_name(cs.season);
    const std::size_t n = name.size() < sizeof(e.season_name) - 1 ? name.size() : sizeof(e.season_name) - 1;
    std::memcpy(e.season_name, name.data(), n);
    e.season_name[n] = '\0';
    e.day_in_season = cs.day_in_season;
    e.day_of_year = cs.day_of_year;
    e.time_of_day_s = cs.time_of_day_s;
    e.day_s = cs.day_s;
    e.night_s = cs.night_s;
    e.day_fraction = cs.day_fraction;
    e.clock_hours = cs.day_fraction * 24.0;
    e.is_day = cs.is_day();

    e.sun_elevation_rad = ss.elevation_rad;
    e.sun_azimuth_rad = ss.azimuth_rad;
    e.sun_dir_world = east_world_ * ss.dir_east + north_world_ * ss.dir_north + Vec3{0.0, 0.0, ss.dir_up};
    e.cloud_cover = 0.0;
    e.ghi_w_m2 = ghi_w_m2(ss.elevation_rad, e.cloud_cover);
    return e;
}

std::uint64_t Environment::fold_state_hash(std::uint64_t h) const {
    fold_value(h, clock_.anchor_env_s);
    fold_value(h, clock_.anchor_tick);
    fold_value(h, clock_.tick_hz);
    fold_value(h, clock_.scale);
    const std::int32_t mode = static_cast<std::int32_t>(config_.fixed_season.empty() ? (config_.fixed_day_of_year >= 0 ? 2 : 0) : 1);
    fold_value(h, mode);
    fold_value(h, static_cast<std::int32_t>(config_.fixed_day_of_year));
    return h;
}

} // namespace ps::environment
