// core/src/environment/weather.cpp - see ps/environment/weather.h for the model.

#include "ps/environment/weather.h"

#include "ps/environment/air_temperature.h"
#include "ps/math/transcendental.h"
#include "weather_rng.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace ps::environment {

using namespace detail;

namespace {

constexpr real kInf = std::numeric_limits<real>::infinity();

real wrap360(real x) { return x - 360.0 * std::floor(x / 360.0); }
real wrap180(real x) { return x - 360.0 * std::floor((x + 180.0) / 360.0); }
real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

// First-order lag from x0 toward T after dt (closed form).
real lag(real x0, real target, real tau, real dt) {
    if (!(dt > 0.0)) return x0;
    if (x0 == target) return target;
    return target + (x0 - target) * ps::math::exp(-dt / tau);
}

// Integral of the lag over [lo, hi] (both >= t0), times relative to t0.
real lag_integral(real x0, real target, real tau, real lo, real hi) {
    if (!(hi > lo)) return 0.0;
    real v = target * (hi - lo);
    if (x0 != target) v += (x0 - target) * tau * (ps::math::exp(-lo / tau) - ps::math::exp(-hi / tau));
    return v;
}

real precip_tau(const WeatherLagsDesc& l, real x0, real target) { return target > x0 ? l.precip_on_s : l.precip_off_s; }

// v <- v + (target - v) w; exact target at w >= 1, untouched at w <= 0.
real blend(real v, real target, real w) {
    if (!(w > 0.0)) return v;
    if (w >= 1.0) return target;
    return v + (target - v) * w;
}

real blend_angle(real v, real target, real w) {
    if (!(w > 0.0)) return v;
    if (w >= 1.0) return wrap360(target);
    return wrap360(v + wrap180(target - v) * w);
}

real ramp_weight(const WeatherOverride& o, real t) {
    if (t < o.t_start) return 0.0;
    if (!(o.ramp_s > 0.0)) return t < o.t_end ? 1.0 : 0.0;
    // Exactly 0 once the ramp-out is over (W3b): the formula below can leave an ulp-sized positive
    // weight there, which is not bit-exact against the timeline when the base value is 0.
    if (t >= o.t_end && t >= o.t_end + o.ramp_s) return 0.0;
    real up = (t - o.t_start) / o.ramp_s;
    if (up > 1.0) up = 1.0;
    real down = 1.0;
    if (t >= o.t_end) {
        down = 1.0 - (t - o.t_end) / o.ramp_s;
        if (down < 0.0) down = 0.0;
    }
    const real x = up < down ? up : down;
    if (x >= 1.0) return 1.0;
    if (x <= 0.0) return 0.0;
    return x * x * (3.0 - 2.0 * x);
}

real cell_shape(const WeatherCell& c, real t) {
    if (t < c.t0 || t >= c.t1) return 0.0;
    if (t < c.t_peak) return (t - c.t0) / (c.t_peak - c.t0);
    return (c.t1 - t) / (c.t1 - c.t_peak);
}

// Integral of the cell's triangle shape over [a, b].
real cell_shape_integral(const WeatherCell& c, real a, real b) {
    real v = 0.0;
    const real r0 = std::max(a, c.t0), r1 = std::min(b, c.t_peak);
    if (r1 > r0) v += ((r1 - c.t0) * (r1 - c.t0) - (r0 - c.t0) * (r0 - c.t0)) / (2.0 * (c.t_peak - c.t0));
    const real f0 = std::max(a, c.t_peak), f1 = std::min(b, c.t1);
    if (f1 > f0) v += ((c.t1 - f0) * (c.t1 - f0) - (c.t1 - f1) * (c.t1 - f1)) / (2.0 * (c.t1 - c.t_peak));
    return v;
}

void fnv(std::uint64_t& h, const void* p, std::size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 0x100000001b3ull;
    }
}
void fnv_real(std::uint64_t& h, real v) { fnv(h, &v, sizeof v); }

void fnv_preset(std::uint64_t& h, const WeatherPresetDesc& p) {
    fnv(h, p.regime.data(), p.regime.size());
    const real v[] = {p.cloud_cover, p.precip_mm_h, p.hail_diameter_mm, p.wind_mean_m_s, p.wind_dir_deg, p.gust_factor,
                      p.temp_offset_k, p.temp_override_c, p.rh, p.pressure_offset_hpa, p.lightning_per_min};
    for (real x : v) fnv_real(h, x);
    const std::uint8_t t = static_cast<std::uint8_t>(p.precip_type), o = p.temp_is_override ? 1 : 0;
    fnv(h, &t, 1);
    fnv(h, &o, 1);
}

constexpr std::uint64_t kPresetSourceKey = 0x7072657365747375ull;

} // namespace

// ---------------------------------------------------------------------------

struct WeatherTimeline::Base {
    std::int64_t segment = -1;
    int regime = -1;
    real cloud = 0.0, strat = 0.0, cell = 0.0, wind = 0.0, dir = 0.0, temp_off = 0.0, rh = 0.0, press_off = 0.0;
    real gust_factor = 1.0, lightning = 0.0, shape = 0.0, hail_d = 0.0;
    bool in_cell = false;
    PrecipForm form = PrecipForm::Auto;
};

struct WeatherTimeline::Blend {
    int n = 0;
    real w[kMaxOverrides] = {};
    real c[kMaxOverrides] = {};
    real c_base = 1.0;
    int dominant = -1;   // -1 = base
    bool any = false;
};

WeatherCursor::WeatherCursor() {
    ring_.resize(static_cast<std::size_t>(WeatherTimeline::kRingSegments));
    scratch_.reserve(4096);
    scratch_bin_.reserve(4096);
}

WeatherTimeline::WeatherTimeline(const ClimateDesc& climate, std::uint64_t seed, const WeatherOptions& options)
    : climate_(climate), calendar_(climate, options.calendar), seed_(seed), mode_(options.mode),
      preset_(options.preset) {
    if (!climate_.precip_phase || !climate_.humidity || !climate_.diurnal_temperature)
        throw std::invalid_argument(
            "WeatherTimeline: the climate needs precip_phase, humidity and diurnal_temperature blocks");
    if (!climate_.seasons.front().climate)
        throw std::invalid_argument("WeatherTimeline: the climate needs the season climate scalars (temp_mean_c ...)");

    // Alphabetical regime-name table over every season.
    for (const SeasonDesc& s : climate_.seasons)
        if (s.weather)
            for (const NamedRegime& r : s.weather->regimes) regime_names_.push_back(r.name);
    std::sort(regime_names_.begin(), regime_names_.end());
    regime_names_.erase(std::unique(regime_names_.begin(), regime_names_.end()), regime_names_.end());

    const int ns = static_cast<int>(climate_.seasons.size());
    const int n = regime_count();
    regime_table_.assign(static_cast<std::size_t>(ns * n), -1);
    cum_.assign(static_cast<std::size_t>(ns * n * n), 0.0);
    initial_.assign(static_cast<std::size_t>(ns), 0);
    for (int si = 0; si < ns; ++si) {
        const SeasonDesc& s = climate_.seasons[static_cast<std::size_t>(si)];
        if (!s.weather) continue;
        for (const NamedRegime& r : s.weather->regimes) {
            regime_table_[static_cast<std::size_t>(si * n + regime_index(r.name))] = static_cast<int>(regimes_flat_.size());
            regimes_flat_.push_back(r.regime);
        }
        for (const TransitionRowDesc& row : s.weather->transitions) {
            const int from = regime_index(row.from);
            real* c = &cum_[static_cast<std::size_t>((si * n + from) * n)];
            for (const TransitionDesc& t : row.to) c[regime_index(t.to)] = t.p;
            real acc = 0.0;
            for (int k = 0; k < n; ++k) {
                acc += c[k];
                c[k] = acc;
            }
        }
        initial_[static_cast<std::size_t>(si)] = regime_index(s.weather->initial_regime);
    }

    if (mode_ == WeatherMode::Dynamic) {
        if (!climate_.has_weather_statistics())
            throw std::invalid_argument("WeatherTimeline: Dynamic mode needs a climate with regimes (or use Preset mode)");
    } else {
        validate_weather_preset(preset_);
        if (preset_.lightning_per_min > 0.0 && !climate_.lightning)
            throw std::invalid_argument("WeatherTimeline: a preset with lightning needs the climate's lightning block");
        preset_regime_ = regime_index(preset_.regime);
    }
    overrides_.reserve(static_cast<std::size_t>(kMaxOverrides));
}

int WeatherTimeline::regime_index(const std::string& name) const {
    const auto it = std::lower_bound(regime_names_.begin(), regime_names_.end(), name);
    if (it == regime_names_.end() || *it != name) return -1;
    return static_cast<int>(it - regime_names_.begin());
}

int WeatherTimeline::weather_season_at(real t) const {
    return calendar_.day(calendar_.sample(t).day_of_year).weather_season;
}

void WeatherTimeline::draw_segment(WeatherSegment& s) const {
    const RegimeDesc& r = *regime_desc(s.season, s.regime);
    const auto key = [&](std::uint64_t slot) { return rng_key(seed_, kDomainSegment, static_cast<std::uint64_t>(s.index), 0, slot); };
    real d_h = r.duration.median_h;
    if (r.duration.sigma_log != 0.0) d_h *= ps::math::exp(r.duration.sigma_log * rng_normal(key(kSlotDuration)));
    if (d_h < r.duration.min_h) d_h = r.duration.min_h;
    s.t1 = s.t0 + d_h * 3600.0;
    s.cloud = draw_range(r.cloud_cover, key(kSlotCloud));
    s.precip = r.precip_mm_h ? draw_lognormal(*r.precip_mm_h, key(kSlotPrecip)) : 0.0;
    s.wind = draw_weibull(r.wind_mean_m_s, key(kSlotWind));
    const real dir = wrap360(r.wind_dir_deg.mean_deg +
                             r.wind_dir_deg.spread_deg * (2.0 * rng_uniform(key(kSlotWindDir)) - 1.0));
    s.dir = s.dir0 + wrap180(dir - s.dir0);
    s.temp_off = draw_range(r.temp_offset_k, key(kSlotTempOffset));
    s.rh = draw_range(r.rh, key(kSlotRh));
    s.press_off = draw_range(r.pressure_offset_hpa, key(kSlotPressure));
}

WeatherSegment WeatherTimeline::first_segment() const {
    WeatherSegment s;
    s.index = 0;
    s.t0 = 0.0;
    s.season = weather_season_at(0.0);
    s.regime = initial_[static_cast<std::size_t>(s.season)];
    s.dir0 = 0.0;
    draw_segment(s);
    s.dir0 = wrap360(s.dir);
    s.dir = s.dir0;
    s.cloud0 = s.cloud;
    s.precip0 = s.precip;
    s.wind0 = s.wind;
    s.temp_off0 = s.temp_off;
    s.rh0 = s.rh;
    s.press_off0 = s.press_off;
    return s;
}

WeatherSegment WeatherTimeline::next_segment(const WeatherSegment& s) const {
    const WeatherLagsDesc& l = *climate_.lags;
    const real dt = s.t1 - s.t0;
    WeatherSegment n;
    n.index = s.index + 1;
    n.t0 = s.t1;
    n.cloud0 = lag(s.cloud0, s.cloud, l.cloud_s, dt);
    n.precip0 = lag(s.precip0, s.precip, precip_tau(l, s.precip0, s.precip), dt);
    n.wind0 = lag(s.wind0, s.wind, l.wind_s, dt);
    n.dir0 = wrap360(lag(s.dir0, s.dir, l.wind_s, dt));
    n.temp_off0 = lag(s.temp_off0, s.temp_off, l.temp_offset_s, dt);
    n.rh0 = lag(s.rh0, s.rh, l.rh_s, dt);
    n.press_off0 = lag(s.press_off0, s.press_off, l.pressure_s, dt);

    n.season = weather_season_at(n.t0);
    const int nr = regime_count();
    const int from = regime_desc(n.season, s.regime) ? s.regime : initial_[static_cast<std::size_t>(n.season)];
    const real* c = &cum_[static_cast<std::size_t>((n.season * nr + from) * nr)];
    const real u = rng_uniform(rng_key(seed_, kDomainSegment, static_cast<std::uint64_t>(n.index), 0, kSlotRegime));
    int pick = -1, last = -1;
    real prev = 0.0;
    for (int k = 0; k < nr; ++k) {
        if (c[k] > prev) {
            last = k;
            if (pick < 0 && u < c[k]) pick = k;
        }
        prev = c[k];
    }
    n.regime = pick >= 0 ? pick : last;
    draw_segment(n);
    return n;
}

void WeatherTimeline::ensure(WeatherCursor& c, real t_lo, real t_hi) const {
    const std::int64_t R = kRingSegments;
    auto at = [&](std::int64_t i) -> WeatherSegment& { return c.ring_[static_cast<std::size_t>(i % R)]; };
    auto oldest = [&]() { return c.newest_ - (R - 1) > 0 ? c.newest_ - (R - 1) : std::int64_t{0}; };
    auto init = [&]() {
        at(0) = first_segment();
        c.newest_ = 0;
        c.owner_ = this;
        c.generation_ = calendar_generation_;
    };
    if (c.owner_ != this || c.generation_ != calendar_generation_ || c.newest_ < 0) init();
    if (oldest() > 0 && t_lo < at(oldest()).t0) {
        init();
        ++c.restarts_;
    }
    while (at(c.newest_).t1 <= t_hi) {
        const WeatherSegment n = next_segment(at(c.newest_));
        ++c.newest_;
        at(c.newest_) = n;
    }
    if (oldest() > 0 && t_lo < at(oldest()).t0)
        throw std::logic_error("WeatherTimeline: the cursor ring does not cover the requested window");
}

const WeatherSegment& WeatherTimeline::find(const WeatherCursor& c, real t) const {
    const std::int64_t R = kRingSegments;
    const std::int64_t lo = c.newest_ - (R - 1) > 0 ? c.newest_ - (R - 1) : 0;
    for (std::int64_t i = c.newest_; i > lo; --i) {
        const WeatherSegment& s = c.ring_[static_cast<std::size_t>(i % R)];
        if (s.t0 <= t) return s;
    }
    return c.ring_[static_cast<std::size_t>(lo % R)];
}

template <class F>
void WeatherTimeline::for_each_cell(const WeatherSegment& s, F&& f) const {
    const RegimeDesc& r = *regime_desc(s.season, s.regime);
    if (!r.convective) return;
    const ConvectiveDesc& cv = *r.convective;
    real t = s.t0;
    for (int j = 0; j < kMaxCellsPerSegment; ++j) {
        const auto key = [&](std::uint64_t slot) {
            return rng_key(seed_, kDomainCell, static_cast<std::uint64_t>(s.index), static_cast<std::uint64_t>(j), slot);
        };
        WeatherCell cl;
        cl.index = j;
        const real gap = draw_lognormal(cv.gap_min, key(kSlotGap)) * 60.0;
        const real dur = draw_lognormal(cv.cell_duration_min, key(kSlotCellDuration)) * 60.0;
        cl.t0 = t + gap;
        cl.t1 = cl.t0 + dur;
        if (cl.t1 > s.t1) return;
        cl.t_peak = cl.t0 + cv.rise_fraction * dur;
        cl.peak_mm_h = draw_lognormal(cv.peak_mm_h, key(kSlotPeak));
        if (cv.hail && rng_uniform(key(kSlotHail)) < cv.hail->probability)
            cl.hail_diameter_mm = draw_lognormal(cv.hail->diameter_mm, key(kSlotHailDiameter));
        if (!f(cl)) return;
        t = cl.t1;
    }
}

int WeatherTimeline::cells_of(const WeatherSegment& segment, WeatherCell* out, int capacity) const {
    int n = 0;
    for_each_cell(segment, [&](const WeatherCell& c) {
        if (n >= capacity) return false;
        out[n++] = c;
        return true;
    });
    return n;
}

WeatherSegment WeatherTimeline::segment_at(real t, WeatherCursor& c) const {
    if (mode_ != WeatherMode::Dynamic) throw std::logic_error("WeatherTimeline::segment_at: Preset mode has no segments");
    ensure(c, t, t);
    return find(c, t);
}

WeatherTimeline::Base WeatherTimeline::base_at(real t, WeatherCursor& c) const {
    Base b;
    if (mode_ == WeatherMode::Preset) {
        b.cloud = preset_.cloud_cover;
        b.strat = preset_.precip_mm_h;
        b.wind = preset_.wind_mean_m_s;
        b.dir = wrap360(preset_.wind_dir_deg);
        b.rh = preset_.rh;
        b.press_off = preset_.pressure_offset_hpa;
        b.gust_factor = preset_.gust_factor;
        b.lightning = preset_.lightning_per_min;
        b.regime = preset_regime_;
        return b;
    }
    const WeatherLagsDesc& l = *climate_.lags;
    const WeatherSegment& s = find(c, t);
    const RegimeDesc& r = *regime_desc(s.season, s.regime);
    const real dt = t - s.t0;
    b.segment = s.index;
    b.regime = s.regime;
    b.form = r.precip_form;
    b.cloud = lag(s.cloud0, s.cloud, l.cloud_s, dt);
    b.strat = lag(s.precip0, s.precip, precip_tau(l, s.precip0, s.precip), dt);
    b.wind = lag(s.wind0, s.wind, l.wind_s, dt);
    b.dir = wrap360(lag(s.dir0, s.dir, l.wind_s, dt));
    b.temp_off = lag(s.temp_off0, s.temp_off, l.temp_offset_s, dt);
    b.rh = lag(s.rh0, s.rh, l.rh_s, dt);
    b.press_off = lag(s.press_off0, s.press_off, l.pressure_s, dt);
    b.gust_factor = r.gust_factor;
    b.lightning = r.convective ? 0.0 : r.lightning_per_min;
    if (r.convective && t >= s.t0) {
        for_each_cell(s, [&](const WeatherCell& cl) {
            if (cl.t0 > t) return false;
            if (t < cl.t1) {
                b.in_cell = true;
                b.shape = cell_shape(cl, t);
                b.cell = cl.peak_mm_h * b.shape;
                b.gust_factor = r.gust_factor + (r.convective->gust_factor - r.gust_factor) * b.shape;
                b.lightning = r.lightning_per_min;
                if (cl.hail_diameter_mm > 0.0 && b.shape >= r.convective->hail->min_shape) b.hail_d = cl.hail_diameter_mm;
                return false;
            }
            return true;
        });
    }
    return b;
}

real WeatherTimeline::base_precip_integral(real a, real b, WeatherCursor& c) const {
    if (!(b > a)) return 0.0;
    if (mode_ == WeatherMode::Preset) return preset_.precip_mm_h * (b - a) / 3600.0;
    const WeatherLagsDesc& l = *climate_.lags;
    const std::int64_t R = kRingSegments;
    const std::int64_t i0 = find(c, a).index, i1 = find(c, b).index;
    real sum = 0.0;   // mm/h * s
    for (std::int64_t i = i0; i <= i1; ++i) {
        const WeatherSegment& s = c.ring_[static_cast<std::size_t>(i % R)];
        if (s.index == 0 && a < s.t0) sum += s.precip0 * (std::min(b, s.t0) - a);  // constant before t = 0
        const real lo = std::max(a, s.t0), hi = std::min(b, s.t1);
        if (!(hi > lo)) continue;
        sum += lag_integral(s.precip0, s.precip, precip_tau(l, s.precip0, s.precip), lo - s.t0, hi - s.t0);
        const RegimeDesc& r = *regime_desc(s.season, s.regime);
        if (r.convective) {
            for_each_cell(s, [&](const WeatherCell& cl) {
                if (cl.t0 >= hi) return false;
                if (cl.t1 > lo) sum += cl.peak_mm_h * cell_shape_integral(cl, lo, hi);
                return true;
            });
        }
    }
    return sum / 3600.0;
}

void WeatherTimeline::blend_weights(real t, Blend& B) const {
    B.n = static_cast<int>(overrides_.size());
    B.any = false;
    for (int k = 0; k < B.n; ++k) {
        B.w[k] = ramp_weight(overrides_[static_cast<std::size_t>(k)].o, t);
        if (B.w[k] > 0.0) B.any = true;
    }
    real suffix = 1.0;   // prod_{j > k} (1 - w_j)
    for (int k = B.n - 1; k >= 0; --k) {
        B.c[k] = B.w[k] * suffix;
        suffix *= 1.0 - B.w[k];
    }
    B.c_base = suffix;
    B.dominant = -1;
    real best = B.c_base;
    for (int k = 0; k < B.n; ++k)
        if (B.c[k] > 0.0 && B.c[k] >= best) {
            best = B.c[k];
            B.dominant = k;
        }
}

PrecipType WeatherTimeline::type_for(PrecipForm form, real temp_c) const {
    const PrecipPhaseDesc& ph = *climate_.precip_phase;
    switch (form) {
    case PrecipForm::Drizzle:
        return temp_c <= ph.snow_below_c ? PrecipType::Snow : PrecipType::Drizzle;
    case PrecipForm::FreezingRain:
        return temp_c < 0.0 ? PrecipType::FreezingRain : PrecipType::Rain;
    case PrecipForm::Auto:
        break;
    }
    if (temp_c <= ph.snow_below_c) return PrecipType::Snow;
    if (temp_c >= ph.rain_above_c) return PrecipType::Rain;
    return PrecipType::Sleet;
}

PrecipType WeatherTimeline::preset_type(const WeatherPresetDesc& p, real temp_c) const {
    switch (p.precip_type) {
    case PresetPrecipType::Auto: return type_for(PrecipForm::Auto, temp_c);
    case PresetPrecipType::Rain: return PrecipType::Rain;
    case PresetPrecipType::Snow: return PrecipType::Snow;
    case PresetPrecipType::Sleet: return PrecipType::Sleet;
    case PresetPrecipType::FreezingRain: return PrecipType::FreezingRain;
    case PresetPrecipType::Drizzle: return PrecipType::Drizzle;
    case PresetPrecipType::Hail: return PrecipType::Hail;
    }
    return PrecipType::None;
}

real WeatherTimeline::published_precip_at(real t, WeatherCursor& c) const {
    const Base b = base_at(t, c);
    Blend B;
    blend_weights(t, B);
    real v = b.strat + b.cell;
    for (int k = 0; k < B.n; ++k) v = blend(v, overrides_[static_cast<std::size_t>(k)].o.target.precip_mm_h, B.w[k]);
    return v;
}

real WeatherTimeline::precipitation_mm(real t_a, real t_b, WeatherCursor& c) const {
    if (!std::isfinite(t_a) || !std::isfinite(t_b) || t_b < t_a || t_b - t_a > kMaxAccumulationWindowS * (1.0 + 1e-12))
        throw std::invalid_argument("WeatherTimeline::precipitation_mm: need 0 <= t_b - t_a <= 6 h");
    if (mode_ == WeatherMode::Dynamic) ensure(c, t_a, t_b);
    if (overrides_.empty()) return base_precip_integral(t_a, t_b, c);

    real bp[2 + 4 * kMaxOverrides];
    int nb = 0;
    bp[nb++] = t_a;
    bp[nb++] = t_b;
    for (const Override& ov : overrides_) {
        const real cand[] = {ov.o.t_start, ov.o.t_start + ov.o.ramp_s, ov.o.t_end, ov.o.t_end + ov.o.ramp_s};
        for (real x : cand)
            if (std::isfinite(x) && x > t_a && x < t_b) bp[nb++] = x;
    }
    std::sort(bp, bp + nb);
    real sum = 0.0;
    for (int i = 0; i + 1 < nb; ++i) {
        const real p = bp[i], q = bp[i + 1];
        if (!(q > p)) continue;
        Blend B;
        blend_weights(0.5 * (p + q), B);
        bool ramping = false;
        for (int k = 0; k < B.n; ++k)
            if (B.w[k] > 0.0 && B.w[k] < 1.0) ramping = true;
        if (!ramping) {
            real v = B.c_base > 0.0 ? B.c_base * base_precip_integral(p, q, c) : 0.0;
            for (int k = 0; k < B.n; ++k)
                if (B.c[k] > 0.0) v += B.c[k] * overrides_[static_cast<std::size_t>(k)].o.target.precip_mm_h * (q - p) / 3600.0;
            sum += v;
        } else {
            // Composite Simpson, 16 panels (an override ramp; base kinks inside make it approximate).
            constexpr int N = 16;
            const real h = (q - p) / N;
            real acc = published_precip_at(p, c) + published_precip_at(q, c);
            for (int j = 1; j < N; ++j) acc += (j % 2 ? 4.0 : 2.0) * published_precip_at(p + h * j, c);
            sum += acc * h / 3.0 / 3600.0;
        }
    }
    return sum;
}

WeatherSample WeatherTimeline::evaluate(real t, WeatherCursor& c) const {
    WeatherSample s;
    s.t_env = t;
    if (mode_ == WeatherMode::Dynamic) ensure(c, t - kMaxAccumulationWindowS, t);
    const Base b = base_at(t, c);
    Blend B;
    blend_weights(t, B);

    real cloud = b.cloud, precip = b.strat + b.cell, wind = b.wind, dir = b.dir, gust = b.gust_factor;
    real rh = b.rh, press_off = b.press_off;
    for (int k = 0; k < B.n; ++k) {
        const real w = B.w[k];
        if (!(w > 0.0)) continue;
        const WeatherPresetDesc& p = overrides_[static_cast<std::size_t>(k)].o.target;
        cloud = blend(cloud, p.cloud_cover, w);
        precip = blend(precip, p.precip_mm_h, w);
        wind = blend(wind, p.wind_mean_m_s, w);
        dir = blend_angle(dir, p.wind_dir_deg, w);
        gust = blend(gust, p.gust_factor, w);
        rh = blend(rh, p.rh, w);
        press_off = blend(press_off, p.pressure_offset_hpa, w);
    }

    const real pl = parton_logan_air_temp_c(calendar_, *climate_.diurnal_temperature, t, cloud);
    real temp;
    if (mode_ == WeatherMode::Preset)
        temp = preset_.temp_is_override ? preset_.temp_override_c : pl + preset_.temp_offset_k;
    else
        temp = pl + b.temp_off;
    for (int k = 0; k < B.n; ++k) {
        const real w = B.w[k];
        if (!(w > 0.0)) continue;
        const WeatherPresetDesc& p = overrides_[static_cast<std::size_t>(k)].o.target;
        temp = blend(temp, p.temp_is_override ? p.temp_override_c : pl + p.temp_offset_k, w);
    }

    const HumidityDesc& hum = *climate_.humidity;
    const real floor_rh = rh > hum.precip_rh_floor ? rh : hum.precip_rh_floor;
    const real f = clamp01(precip / hum.precip_full_mm_h);
    if (f > 0.0) rh = rh + (floor_rh - rh) * f;

    const CalendarSample cs = calendar_.sample(t);
    const DayValues& dv = calendar_.day(cs.day_of_year);
    s.season = dv.weather_season;
    s.segment = b.segment;
    s.base_weight = B.c_base;
    s.cloud_cover = cloud;
    s.precip_mm_h = precip;
    s.precip_stratiform_mm_h = b.strat;
    s.precip_cell_mm_h = b.cell;
    s.in_cell = b.in_cell;
    s.cell_shape = b.shape;
    s.wind_mean_m_s = wind;
    s.wind_dir_deg = dir;
    s.gust_factor = gust;
    s.gust_nominal_m_s = wind * gust;
    s.air_temp_c = temp;
    s.temp_offset_k = temp - pl;
    s.rh = rh;
    s.vapour_pressure_hpa = rh * saturation_vapour_pressure_hpa(temp);
    s.dew_point_c = dew_point_c(temp, rh);
    s.pressure_msl_hpa = dv.climate.pressure_msl_hpa + press_off;

    PrecipType type;
    real hail_d = 0.0;
    if (B.dominant < 0) {
        if (mode_ == WeatherMode::Preset) {
            s.source = WeatherSourceKind::Preset;
            type = preset_type(preset_, temp);
            hail_d = preset_.hail_diameter_mm;
        } else {
            s.source = WeatherSourceKind::Timeline;
            type = b.hail_d > 0.0 ? PrecipType::Hail : type_for(b.form, temp);
            hail_d = b.hail_d;
        }
        s.lightning_per_min = b.lightning;
        s.regime = b.regime;
    } else {
        const Override& ov = overrides_[static_cast<std::size_t>(B.dominant)];
        s.source = WeatherSourceKind::Override;
        s.override_id = ov.id;
        type = preset_type(ov.o.target, temp);
        hail_d = ov.o.target.hail_diameter_mm;
        s.lightning_per_min = ov.o.target.lightning_per_min;
        s.regime = ov.regime;
    }
    if (precip < climate_.precip_phase->min_rate_mm_h) type = PrecipType::None;
    s.precip_type = type;
    s.hail_diameter_mm = type == PrecipType::Hail ? hail_d : 0.0;

    s.precip_1h_mm = precipitation_mm(t - 3600.0, t, c);
    s.precip_6h_mm = precipitation_mm(t - kMaxAccumulationWindowS, t, c);
    return s;
}

int WeatherTimeline::lightning_events(real t, real max_age_s, WeatherCursor& c, LightningEvent* out, int capacity) const {
    if (!std::isfinite(t) || !std::isfinite(max_age_s) || max_age_s < 0.0 || max_age_s > kMaxLightningAgeS || capacity < 0)
        throw std::invalid_argument("WeatherTimeline::lightning_events: max_age_s must be in [0, 1800]");
    const real t_lo = t - max_age_s;
    const std::int64_t bin_lo = static_cast<std::int64_t>(std::floor(t_lo / kLightningBinS));
    const std::int64_t bin_hi = static_cast<std::int64_t>(std::floor(t / kLightningBinS));
    const real B0 = static_cast<real>(bin_lo) * kLightningBinS;
    const real B1 = static_cast<real>(bin_hi + 1) * kLightningBinS;
    if (mode_ == WeatherMode::Dynamic) ensure(c, B0, B1);

    std::vector<LightningEvent>& ev = c.scratch_;
    std::vector<std::uint64_t>& ev_bin = c.scratch_bin_;
    ev.clear();
    ev_bin.clear();
    const std::size_t cap_total = ev.capacity();
    const real r_min = climate_.lightning ? climate_.lightning->distance_min_km * 1000.0 : 0.0;
    const real r_max = climate_.lightning ? climate_.lightning->distance_max_km * 1000.0 : 0.0;

    // source_index: -1 base (timeline or preset), k = override k.
    auto emit = [&](real p, real q, real rate, std::uint64_t source_key, int source_index) {
        if (!(rate > 0.0)) return;
        p = std::max(p, B0);
        q = std::min(q, B1);
        if (!(q > p)) return;
        std::int64_t bin = static_cast<std::int64_t>(std::floor(p / kLightningBinS));
        for (; static_cast<real>(bin) * kLightningBinS < q; ++bin) {
            const real lo = std::max(p, static_cast<real>(bin) * kLightningBinS);
            const real hi = std::min(q, static_cast<real>(bin + 1) * kLightningBinS);
            if (!(hi > lo)) continue;
            const std::uint64_t ub = static_cast<std::uint64_t>(bin);
            const int count = draw_poisson(rate * (hi - lo) / 60.0, rng_key(seed_, kDomainLightning, ub, source_key, 0),
                                           kMaxLightningPerBin);
            for (int j = 0; j < count; ++j) {
                const auto key = [&](std::uint64_t s) {
                    return rng_key(seed_, kDomainLightning, ub, source_key, 1 + 3 * static_cast<std::uint64_t>(j) + s);
                };
                const real te = lo + (hi - lo) * rng_uniform(key(0));
                Blend B;
                blend_weights(te, B);
                if (B.dominant != source_index) continue;
                const real u = rng_uniform(key(1));
                const real r = std::sqrt(r_min * r_min + u * (r_max * r_max - r_min * r_min));
                real sn = 0.0, cn = 1.0;
                ps::math::sincos(2.0 * kPi * rng_uniform(key(2)), sn, cn);
                if (ev.size() >= cap_total) return;
                LightningEvent e;
                e.t_env = te;
                e.offset_east_m = r * sn;
                e.offset_north_m = r * cn;
                e.id = source_key;   // temporary: tie-break key
                ev.push_back(e);
                ev_bin.push_back(ub);
            }
        }
    };

    if (mode_ == WeatherMode::Preset) {
        emit(-kInf, kInf, preset_.lightning_per_min, kPresetSourceKey, -1);
    } else {
        const std::int64_t R = kRingSegments;
        const std::int64_t i0 = find(c, B0).index, i1 = find(c, B1).index;
        for (std::int64_t i = i0; i <= i1; ++i) {
            const WeatherSegment& s = c.ring_[static_cast<std::size_t>(i % R)];
            const RegimeDesc& r = *regime_desc(s.season, s.regime);
            if (!(r.lightning_per_min > 0.0)) continue;
            const std::uint64_t base_key = static_cast<std::uint64_t>(s.index) << 20;
            if (r.convective) {
                for_each_cell(s, [&](const WeatherCell& cl) {
                    if (cl.t0 >= B1) return false;
                    if (cl.t1 > B0) emit(cl.t0, cl.t1, r.lightning_per_min, base_key | static_cast<std::uint64_t>(cl.index), -1);
                    return true;
                });
            } else {
                emit(s.index == 0 ? -kInf : s.t0, s.t1, r.lightning_per_min, base_key | 0xFFFFFull, -1);
            }
        }
    }
    for (int k = 0; k < static_cast<int>(overrides_.size()); ++k) {
        const Override& ov = overrides_[static_cast<std::size_t>(k)];
        emit(ov.o.t_start, ov.o.t_end + ov.o.ramp_s, ov.o.target.lightning_per_min, ov.key, k);
    }

    // Order by (time, source key); rank within each bin; id = bin * 64 + rank.
    const std::size_t n = ev.size();
    for (std::size_t i = 1; i < n; ++i) {   // insertion sort (small n, allocation-free, stable)
        const LightningEvent e = ev[i];
        const std::uint64_t eb = ev_bin[i];
        std::size_t j = i;
        while (j > 0 && (ev[j - 1].t_env > e.t_env || (ev[j - 1].t_env == e.t_env && ev[j - 1].id > e.id))) {
            ev[j] = ev[j - 1];
            ev_bin[j] = ev_bin[j - 1];
            --j;
        }
        ev[j] = e;
        ev_bin[j] = eb;
    }
    std::size_t kept = 0;
    int rank = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (i == 0 || ev_bin[i] != ev_bin[i - 1]) rank = 0;
        else ++rank;
        if (rank >= kMaxLightningPerBin) continue;
        LightningEvent e = ev[i];
        // Offset the bin so ids stay monotonic across t = 0.
        e.id = (ev_bin[i] + (std::uint64_t{1} << 40)) * 64u + static_cast<std::uint64_t>(rank);
        if (e.t_env > t_lo && e.t_env <= t) ev[kept++] = e;
    }
    const int count = static_cast<int>(std::min<std::size_t>(kept, static_cast<std::size_t>(capacity)));
    for (int i = 0; i < count; ++i) out[i] = ev[kept - static_cast<std::size_t>(count) + static_cast<std::size_t>(i)];
    return count;
}

int WeatherTimeline::insert_override(Override ov) {
    if (static_cast<int>(overrides_.size()) >= kMaxOverrides)
        throw std::length_error("WeatherTimeline: at most 16 overrides");
    ov.id = next_override_id_++;
    ov.seq = next_seq_++;
    ov.regime = regime_index(ov.o.target.regime);
    std::uint64_t h = 0xcbf29ce484222325ull;
    fnv_real(h, ov.o.t_start);
    fnv_real(h, ov.o.t_end);
    fnv_real(h, ov.o.ramp_s);
    ov.key = h;
    auto it = overrides_.begin();
    while (it != overrides_.end() && (it->o.t_start < ov.o.t_start || (it->o.t_start == ov.o.t_start && it->seq < ov.seq))) ++it;
    const int id = ov.id;
    overrides_.insert(it, std::move(ov));
    return id;
}

int WeatherTimeline::add_override(const WeatherOverride& o) {
    if (!std::isfinite(o.t_start)) throw std::invalid_argument("WeatherOverride: t_start must be finite");
    if (std::isnan(o.t_end) || !(o.t_end > o.t_start)) throw std::invalid_argument("WeatherOverride: t_end must be > t_start");
    if (!std::isfinite(o.ramp_s) || o.ramp_s < 0.0) throw std::invalid_argument("WeatherOverride: ramp_s must be >= 0");
    validate_weather_preset(o.target);
    if (o.target.lightning_per_min > 0.0 && !climate_.lightning)
        throw std::invalid_argument("WeatherOverride: lightning needs the climate's lightning block");
    Override ov;
    ov.o = o;
    return insert_override(std::move(ov));
}

void WeatherTimeline::resolve_season_bound(Override& ov, real t_start, real t_now) const {
    ov.o.t_start = t_start;
    ov.o.ramp_s = ov.ramp_requested;
    if (calendar_.season_locked()) {
        ov.o.t_end = std::numeric_limits<real>::infinity();
        return;
    }
    const real end = calendar_.season_end_env_time(t_now);
    real ramp = ov.ramp_requested;
    const real half = 0.5 * (end - t_start);
    if (ramp > half) ramp = half;
    real t_end = end - ramp;
    // The ramp-out must be complete at `end` exactly: rounding of (end - ramp) + ramp must not exceed it.
    while (t_end + ramp > end) t_end = std::nextafter(t_end, -std::numeric_limits<real>::infinity());
    ov.o.ramp_s = ramp;
    ov.o.t_end = t_end;
}

int WeatherTimeline::add_season_override(const WeatherPresetDesc& target, real ramp_s, real t_now) {
    if (!std::isfinite(t_now)) throw std::invalid_argument("WeatherOverride: t_now must be finite");
    if (!std::isfinite(ramp_s) || ramp_s < 0.0) throw std::invalid_argument("WeatherOverride: ramp_s must be >= 0");
    validate_weather_preset(target);
    if (target.lightning_per_min > 0.0 && !climate_.lightning)
        throw std::invalid_argument("WeatherOverride: lightning needs the climate's lightning block");
    Override ov;
    ov.o.target = target;
    ov.season_bound = true;
    ov.ramp_requested = ramp_s;
    resolve_season_bound(ov, t_now, t_now);
    return insert_override(std::move(ov));
}

void WeatherTimeline::set_calendar(const CalendarOptions& options, real t_now_old, real t_now_new) {
    if (!std::isfinite(t_now_old) || !std::isfinite(t_now_new))
        throw std::invalid_argument("WeatherTimeline::set_calendar: times must be finite");
    Calendar next(climate_, options);   // throws before anything changes
    calendar_ = std::move(next);
    ++calendar_generation_;
    bool moved = false;
    for (Override& ov : overrides_) {
        if (!ov.season_bound) continue;
        real elapsed = t_now_old - ov.o.t_start;
        if (elapsed < 0.0) elapsed = 0.0;
        resolve_season_bound(ov, t_now_new - elapsed, t_now_new);
        moved = true;
    }
    if (moved)
        std::stable_sort(overrides_.begin(), overrides_.end(), [](const Override& a, const Override& b) {
            return a.o.t_start < b.o.t_start || (a.o.t_start == b.o.t_start && a.seq < b.seq);
        });
}

bool WeatherTimeline::override_is_season_bound(int id) const {
    for (const Override& ov : overrides_)
        if (ov.id == id) return ov.season_bound;
    return false;
}

void WeatherTimeline::end_all_overrides(real t_now) {
    for (std::size_t i = 0; i < overrides_.size();) {
        Override& ov = overrides_[i];
        if (!(t_now > ov.o.t_start)) {
            overrides_.erase(overrides_.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (t_now < ov.o.t_end) ov.o.t_end = t_now;   // an earlier scheduled end stays
        ov.season_bound = false;
        ++i;
    }
}

const WeatherOverride* WeatherTimeline::find_override(int id) const {
    for (const Override& ov : overrides_)
        if (ov.id == id) return &ov.o;
    return nullptr;
}

bool WeatherTimeline::remove_override(int id) {
    for (auto it = overrides_.begin(); it != overrides_.end(); ++it)
        if (it->id == id) {
            overrides_.erase(it);
            return true;
        }
    return false;
}

void WeatherTimeline::clear_overrides() { overrides_.clear(); }

std::uint64_t WeatherTimeline::fold_hash(std::uint64_t h) const {
    fnv(h, &seed_, sizeof seed_);
    const std::uint8_t m = static_cast<std::uint8_t>(mode_);
    fnv(h, &m, 1);
    if (mode_ == WeatherMode::Preset) fnv_preset(h, preset_);
    const std::uint64_t n = overrides_.size();
    fnv(h, &n, sizeof n);
    for (const Override& ov : overrides_) {
        fnv_real(h, ov.o.t_start);
        fnv_real(h, ov.o.t_end);
        fnv_real(h, ov.o.ramp_s);
        fnv_preset(h, ov.o.target);
        if (ov.season_bound) {   // W3b: only season-bound overrides add bytes, so older hashes are unchanged
            const std::uint8_t sb = 1;
            fnv(h, &sb, 1);
            fnv_real(h, ov.ramp_requested);
        }
    }
    return h;
}

} // namespace ps::environment
