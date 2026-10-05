// ps/environment/environment.h implementation (weather plan W2).

#include "ps/environment/environment.h"

#include "ps/math/transcendental.h"

#include <algorithm>
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

    if (!std::isfinite(config_.altitude_offset_m))
        throw std::invalid_argument("environment: altitude_offset_m must be finite");
    if (config_.weather != EnvironmentWeather::Off) {
        WeatherOptions wo;
        wo.calendar = make_calendar_options(climate_, config_);
        wo.mode = config_.weather == EnvironmentWeather::Preset ? WeatherMode::Preset : WeatherMode::Dynamic;
        wo.preset = config_.weather_preset;
        weather_.emplace(climate_, config_.seed, wo);  // throws std::invalid_argument naming what is missing
        if (climate_.gusts) gust_.emplace(*climate_.gusts, config_.seed);
    }
    if (config_.surface_conditions) {
        if (!weather_)
            throw std::invalid_argument(
                "environment: surface conditions need weather (EnvironmentConfig::weather Dynamic or Preset): "
                "the energy balance reads air temperature, wind, humidity and cloud");
        if (calendar_.days_per_year() < 1 || !calendar_.day(0).has_climate)
            throw std::invalid_argument(
                "environment: surface conditions need the seasons' climate scalars (ground_temp_c, temp_mean_c)");
        // ConditionGrid validates the desc, the names and the grid settings.
        grid_.emplace(*config_.surface_conditions, config_.surface_names, climate_.condition.value_or(ConditionGridDesc{}));
        grip_.build_static(*grid_, config_.default_tyre_profile.condition_factors);
        forcing_.env = this;
    }

    clock_.anchor_env_s = start_env_s_;
    clock_.anchor_tick = 0;
    clock_.tick_hz = 240.0;
    clock_.scale = config_.time_scale;
    clock_.validate();
    begin_tick(0);
}

void Environment::attach(std::int64_t tick, real tick_hz) {
    EnvClock c;
    c.anchor_env_s = start_env_s_;
    c.anchor_tick = tick;
    c.tick_hz = tick_hz;
    c.scale = config_.time_scale;
    c.validate(); // before assigning: a throw leaves the clock untouched
    clock_ = c;
    invalidate_gust_cache();
    begin_tick(tick);
}

void Environment::set_time(real env_s, std::int64_t tick) {
    EnvClock c = clock_;
    c.set_time(env_s, tick);
    c.validate();
    clock_ = c;
    invalidate_gust_cache();
    begin_tick(tick);
}

void Environment::set_time_scale(real scale, std::int64_t tick) {
    EnvClock c = clock_;
    c.set_scale(scale, tick);
    c.validate();
    clock_ = c;
    invalidate_gust_cache();
    begin_tick(tick);
}

namespace {
void require_weather(bool has, const char* what) {
    if (!has) throw std::logic_error(std::string("Environment::") + what + ": the environment has no weather timeline");
}
} // namespace

int Environment::add_season_weather_override(const WeatherPresetDesc& target, real ramp_s, std::int64_t tick) {
    require_weather(weather_.has_value(), "add_season_weather_override");
    const int id = weather_->add_season_override(target, ramp_s, clock_.env_time(tick));
    invalidate_gust_cache();
    begin_tick(tick);
    return id;
}

bool Environment::remove_weather_override(int id, std::int64_t tick) {
    require_weather(weather_.has_value(), "remove_weather_override");
    const bool removed = weather_->remove_override(id);
    invalidate_gust_cache();
    begin_tick(tick);
    return removed;
}

void Environment::clear_weather_overrides(std::int64_t tick) {
    require_weather(weather_.has_value(), "clear_weather_overrides");
    weather_->clear_overrides();
    invalidate_gust_cache();
    begin_tick(tick);
}

void Environment::end_weather_overrides(std::int64_t tick) {
    require_weather(weather_.has_value(), "end_weather_overrides");
    weather_->end_all_overrides(clock_.env_time(tick));
    invalidate_gust_cache();
    begin_tick(tick);
}

void Environment::set_calendar_lock(const std::string& fixed_season, int fixed_day_of_year, std::int64_t tick) {
    EnvironmentConfig cfg = config_;
    cfg.fixed_season = fixed_season;
    cfg.fixed_day_of_year = fixed_day_of_year;
    const CalendarOptions opts = make_calendar_options(climate_, cfg);   // throws before any change
    const CalendarOptions& cur = calendar_.options();
    if (opts.mode == cur.mode && opts.fixed_season == cur.fixed_season && opts.fixed_day == cur.fixed_day) return;
    Calendar next(climate_, opts);
    // Keep the game day and the fraction of the day across the switch.
    const real t_old = clock_.env_time(tick);
    const CalendarSample cs = calendar_.sample(t_old);
    const int idx = next.season_start_day(cs.season) + cs.day_in_season;
    const real t_new = next.env_time(cs.year, cs.season, cs.day_in_season, cs.day_fraction * next.day(idx).length_s());
    EnvClock c = clock_;
    c.set_time(t_new, tick);
    c.validate();
    if (weather_) weather_->set_calendar(opts, t_old, t_new);   // strong guarantee; same validity as `next`
    calendar_ = std::move(next);
    config_ = cfg;
    clock_ = c;
    invalidate_gust_cache();
    begin_tick(tick);
}

void Environment::begin_tick(std::int64_t tick) {
    const real t_env = clock_.env_time(tick);
    WeatherSample w;
    current_ = evaluate_impl(t_env, weather_ ? &w : nullptr);
    if (weather_) update_air(tick, t_env, w);
    if (grid_) {
        last_condition_steps_ = grid_->advance_to(t_env, forcing_);
        grip_.update_state(*grid_, t_env);
        fill_surfaces(t_env);
    }
}

// ---- W5: surface conditions

SurfaceForcing Environment::surface_forcing(real t_env) const {
    SurfaceForcing f;
    const CalendarSample cs = calendar_.sample(t_env);
    const SunState ss = sun_.at(calendar_, cs);
    const DayValues& day = calendar_.day(cs.day_of_year);
    if (weather_) {
        const WeatherSample w = weather_->evaluate(t_env, grid_cursor_);
        f.air_temp_k = w.air_temp_c + kCelsiusToKelvin;
        f.wind_m_s = w.wind_mean_m_s;
        f.vapour_pressure_hpa = w.vapour_pressure_hpa;
        f.cloud_cover = w.cloud_cover;
        // W6 models liquid water only: snow precipitation is W8 (its melt and
        // pack), so it contributes no liquid input yet.
        if (w.precip_type != PrecipType::Snow) f.precip_mm_h = w.precip_mm_h;
    }
    f.ghi_w_m2 = ghi_w_m2(ss.elevation_rad, f.cloud_cover);
    f.bottom_temp_k = (day.has_climate ? day.climate.ground_temp_c : 10.0) + kCelsiusToKelvin;
    return f;
}

InitialProfile Environment::initial_profile(real t_env) const {
    // W5 decision D5: linear from the day's mean air temperature at the
    // surface to its ground temperature at the column bottom.
    const DayValues& day = calendar_.day(calendar_.sample(t_env).day_of_year);
    InitialProfile p;
    p.surface_temp_k = (day.has_climate ? day.climate.temp_mean_c : 10.0) + kCelsiusToKelvin;
    p.bottom_temp_k = (day.has_climate ? day.climate.ground_temp_c : 10.0) + kCelsiusToKelvin;
    return p;
}

void Environment::fill_surfaces(real t_env) {
    const ConditionGrid& g = *grid_;
    current_.has_surfaces = true;
    current_.surface_count = g.class_count();
    const real w = g.weight_at(t_env);
    for (int i = 0; i < g.class_count(); ++i) {
        SurfaceConditionSample& s = current_.surfaces[static_cast<std::size_t>(i)];
        s = SurfaceConditionSample{};
        s.surface_id = i;
        s.kind = static_cast<std::int32_t>(g.kind(i));
        s.has_state = g.has_state(i);
        if (!s.has_state) continue;
        const SurfaceClassState& a = g.prev(i);
        const SurfaceClassState& b = g.next(i);
        const auto lerp = [w](real x, real y) { return x + (y - x) * w; };
        s.t_surface_c = lerp(a.temp_k[0], b.temp_k[0]) - kCelsiusToKelvin;
        s.water_mm = lerp(a.water_mm, b.water_mm);
        s.ice_mm = lerp(a.ice_mm, b.ice_mm);
        s.frost_mm = lerp(a.frost_mm, b.frost_mm);
        s.snow_mm = lerp(a.snow_mm, b.snow_mm);
        s.snow_density_kg_m3 = lerp(a.snow_density_kg_m3, b.snow_density_kg_m3);
        s.soil_moisture = lerp(a.soil_moisture, b.soil_moisture);
        s.canopy_water_mm = lerp(a.canopy_water_mm, b.canopy_water_mm);
        const ColumnModel& col = g.column(i);
        const real above = col.kind == SurfaceKind::Sealed ? std::max(s.water_mm - col.w_tex_mm, 0.0) : s.water_mm;
        s.water_above_texture_mm = above;
        const real ww = grip_.water_weight(i);
        s.condition = ww <= 0.0 ? 0 : (ww < 1.0 ? 1 : 2);
        s.condition_factor = grip_.factor(i, 0.0);
    }
}

ConditionCheckpoint Environment::checkpoint_conditions() const {
    if (!grid_) throw std::logic_error("Environment::checkpoint_conditions: the environment has no surface conditions");
    return grid_->checkpoint();
}

void Environment::restore_conditions(const ConditionCheckpoint& c, std::int64_t tick) {
    if (!grid_) throw std::logic_error("Environment::restore_conditions: the environment has no surface conditions");
    const std::int64_t kt = grid_->grid_index_at(clock_.env_time(tick));
    if (c.valid && c.grid_index > kt)
        throw std::invalid_argument("Environment::restore_conditions: the checkpoint (grid index " +
                                    std::to_string(c.grid_index) + ") lies ahead of the clock (" +
                                    std::to_string(kt) + ")");
    if (c.valid && kt - c.grid_index > grid_->max_fast_forward_steps())
        throw std::invalid_argument("Environment::restore_conditions: the checkpoint is more than max_fast_forward_s "
                                    "behind the clock (a spin-up would discard it)");
    grid_->restore(c);   // validates the rest; throws before any change
    begin_tick(tick);
}

EnvironmentSnapshot Environment::evaluate(real t_env) const { return evaluate_impl(t_env, nullptr); }

namespace {

real finite_or(real v, real fallback) { return std::isfinite(v) ? v : fallback; }

} // namespace

// The weather-derived part of a tick: the sea-level reference atmosphere, the
// mean wind vector and the gust amplitudes/phases/advection of the sim time.
void Environment::update_air(std::int64_t tick, real t_env, const WeatherSample& w) {
    (void)t_env;
    AirState& a = air_;
    a.valid = true;

    // Station values -> sea-level reference on the ISA lapse (the weather's
    // temperature and vapour pressure are those AT location.elevation_m,
    // its pressure is already reduced to sea level).
    const real h_station = climate_.location.elevation_m;
    real t_st = finite_or(w.air_temp_c + 273.15, kIsaSeaLevelTemperatureK);
    real p0 = finite_or(w.pressure_msl_hpa * 100.0, kIsaSeaLevelPressurePa);
    real e_st = finite_or(w.vapour_pressure_hpa * 100.0, 0.0);
    if (t_st < 150.0) t_st = 150.0;   // far outside any climate: keep the ISA lift defined
    if (!(p0 > 100.0)) p0 = kIsaSeaLevelPressurePa;
    if (e_st < 0.0) e_st = 0.0;
    const real t0 = t_st + kIsaLapseRate * (h_station < kIsaTropopauseM ? h_station : kIsaTropopauseM);
    const real p_st = isa_pressure_pa(p0, t0, h_station);
    a.sea.pressure_pa = p0;
    a.sea.temperature_k = t0 > 150.0 ? t0 : 150.0;
    a.sea.vapour_pressure_pa = e_st * (p0 / p_st);   // constant mole fraction, as atmosphere_at lifts it

    // Mean wind: the weather publishes the direction the wind blows FROM
    // (compass degrees clockwise from north), the vehicle wants where it goes.
    real sd = 0.0, cd = 1.0;
    math::sincos(w.wind_dir_deg * kDegToRad, sd, cd);
    const Vec3 toward = east_world_ * (-sd) + north_world_ * (-cd);
    a.mean_speed = finite_or(w.wind_mean_m_s, 0.0);
    if (a.mean_speed < 0.0) a.mean_speed = 0.0;
    a.mean_wind = toward * a.mean_speed;
    a.sigma_u = 0.0;

    // Gusts.
    a.t_gust = static_cast<real>(tick) / clock_.tick_hz;
    if (gust_) {
        const GustDesc& gd = gust_->desc();
        a.sigma_u = a.mean_speed * gust_sigma_ratio(gd, finite_or(w.gust_factor, 1.0));
        if (!(a.sigma_u > 0.0)) a.sigma_u = 0.0;
        gust_->amplitudes(a.mean_speed, a.sigma_u, a.amps);
        a.blend = gust_->blend(a.t_gust);
        if (phases_cur_.window != a.blend.window) gust_->fill_phases(a.blend.window, phases_cur_);
        if (a.blend.w_prev > 0.0 && phases_prev_.window != a.blend.window - 1)
            gust_->fill_phases(a.blend.window - 1, phases_prev_);
        // Advection velocity of the frozen pattern: the published mean wind at
        // the window's start (constant within the window, see wind.h).
        if (!adv_valid_ || adv_window_ != a.blend.window) {
            const real t_win = static_cast<real>(a.blend.window) * gd.window_s;
            const std::int64_t tick_win = static_cast<std::int64_t>(std::floor(t_win * clock_.tick_hz));
            const WeatherSample ws = weather_->evaluate(clock_.env_time(tick_win), cursor_);
            real s0 = 0.0, c0 = 1.0;
            math::sincos(ws.wind_dir_deg * kDegToRad, s0, c0);
            const Vec3 d = east_world_ * (-s0) + north_world_ * (-c0);
            adv_dir_x_ = d.x;
            adv_dir_y_ = d.y;
            adv_speed_ = finite_or(ws.wind_mean_m_s, 0.0);
            adv_window_ = a.blend.window;
            adv_valid_ = true;
        }
        a.dir_x = adv_dir_x_;
        a.dir_y = adv_dir_y_;
        a.adv_speed = adv_speed_;
    }
}

VehicleAir Environment::sample_air(const Vec3& position_world) const {
    VehicleAir v;
    if (!weather_ || !air_.valid) return v;
    const real altitude = climate_.location.elevation_m + config_.altitude_offset_m + position_world.z;
    const AtmosphereSample s = atmosphere_at(air_.sea, finite_or(altitude, climate_.location.elevation_m));
    v.valid = true;
    v.altitude_m = s.altitude_m;
    v.pressure_pa = s.pressure_pa;
    v.temperature_k = s.temperature_k;
    v.vapour_pressure_pa = s.vapour_pressure_pa;
    v.density_kg_m3 = s.density_kg_m3;
    v.mean_wind_world = air_.mean_wind;
    Vec3 gust{};
    if (gust_ && air_.sigma_u > 0.0) {
        const real x = position_world.x, y = position_world.y;
        gust = gust_->evaluate(air_.amps, phases_cur_, air_.dir_x, air_.dir_y, air_.adv_speed, air_.t_gust, x, y) *
               air_.blend.w_new;
        if (air_.blend.w_prev > 0.0)
            gust = gust + gust_->evaluate(air_.amps, phases_prev_, air_.dir_x, air_.dir_y, air_.adv_speed,
                                          air_.t_gust, x, y) * air_.blend.w_prev;
    }
    v.gust_world = gust;
    v.wind_world = air_.mean_wind + gust;
    return v;
}

EnvironmentSnapshot Environment::evaluate_impl(real t_env, WeatherSample* weather_out) const {
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
    if (weather_) {
        // W4: the weather timeline at the same env time. Its cloud cover drives
        // the irradiance (the W2 snapshot had cloud 0).
        const WeatherSample w = weather_->evaluate(t_env, cursor_);
        if (weather_out) *weather_out = w;
        e.has_weather = true;
        e.cloud_cover = w.cloud_cover;
        e.precip_type = static_cast<std::int32_t>(w.precip_type);
        e.precip_mm_h = w.precip_mm_h;
        e.wind_mean_m_s = w.wind_mean_m_s;
        e.wind_dir_deg = w.wind_dir_deg;
        e.gust_factor = w.gust_factor;
        e.air_temp_c = w.air_temp_c;
        e.rh = w.rh;
        e.dew_point_c = w.dew_point_c;
        e.pressure_msl_hpa = w.pressure_msl_hpa;
        e.lightning_per_min = w.lightning_per_min;
    }
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
    if (weather_) {
        // W4: weather inputs that are not the climate file. The gust field is a
        // function of (climate gusts block, seed) - the seed is in the
        // timeline's hash - so only the on/off flag joins.
        h = weather_->fold_hash(h);
        fold_value(h, config_.altitude_offset_m);
        fold_value(h, static_cast<std::int32_t>(gust_ ? 1 : 0));
    }
    // W5: only with surface conditions (which imply weather), so every
    // environment without them hashes exactly as before.
    if (grid_) h = grid_->fold_hash(h);
    return h;
}

} // namespace ps::environment
