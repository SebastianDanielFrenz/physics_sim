// ps/environment/weather.h - the weather timeline (weather plan 3.1-3.9, W3).
//
// Weather is a PURE function of (climate, seed, mode/preset, overrides, t_env):
// WeatherTimeline::evaluate(t) returns the same bits whatever was evaluated
// before and in whatever order (test T2). No wall clock, no accumulated
// floating state, no platform RNG or libm (ps::math only).
//
// Model
//  1. Regime timeline: a semi-Markov chain of segments. Segment 0 starts at
//     t_env = 0 in the weather season valid there with that season's
//     initial_regime; segment i has duration d_i, regime r_i and its own drawn
//     targets (cloud cover, stratiform precipitation, mean wind speed and
//     direction, temperature / pressure offsets, relative humidity). Segment
//     i+1's regime is drawn from r_i's transition row of the season valid at
//     t_{i+1} (DayValues::weather_season: natural, fixed-season or fixed-day
//     season); if r_i is not defined there, from the row of that season's
//     initial_regime. Before t = 0 the weather is segment 0's (constant).
//  2. Cells (convective regimes): gap - cell - gap - cell ... from the segment
//     start, a cell that would end after the segment end is not started; at
//     most one cell is active. Triangle shape (rise to the peak at
//     rise_fraction, then fall), precipitation = peak * shape; the gust factor
//     blends from the regime's to the cell's by the shape; lightning (rate
//     lightning_per_min) only while a cell is active; hail while a hail cell's
//     shape >= min_shape.
//  3. Lags: every continuous quantity follows a first-order lag toward its
//     piecewise-constant target, x(t) = T + (x(t_i) - T) exp(-(t - t_i)/tau),
//     closed form inside a segment; boundary values come from the walk.
//     Precipitation uses tau_on when the target is above the start value,
//     tau_off otherwise; the wind direction lags along the shortest arc.
//     Segment 0 starts at its targets (no transient).
//  4. Derived: air temperature = Parton-Logan (air_temperature.h) with the
//     published cloud cover + the lagged offset; RH floor during precipitation;
//     dew point (Magnus); sea-level pressure = the season's (per day) + offset;
//     precipitation type from the regime's PrecipForm and the air temperature.
//  5. Overrides (plan 3.6) and presets (3.5): an override's targets replace the
//     timeline's with a smoothstep ramp weight w(t) (0 -> 1 over ramp_s from
//     t_start, 1 -> 0 over ramp_s from t_end); a list applied in t_start order,
//     later wins: v <- v + (target - v) w per override, v <- target exactly at
//     w = 1, untouched at w = 0 - so after a window the published weather is
//     bit-identical to the timeline's (test T5). The timeline keeps running
//     underneath. Discrete outputs (type, lightning, hail, regime label) come
//     from the DOMINANT source (largest blend coefficient, ties to the later).
//     Preset mode: the chain is not walked, the preset is the base.
//  6. Trailing 1 h / 6 h precipitation (DWD Starkregen classes): exact closed
//     form over segments (lag integrals) and cells (triangles); pieces where
//     an override ramps are integrated by composite Simpson (16 panels).
//  7. Lightning (3.9): events in 60 s bins; for every source (cell, non-
//     convective lightning segment, preset, override) and bin a Poisson count
//     with mean rate * overlap, times uniform in the overlap, kept only if
//     that source dominates at the event time; distance area-uniform in the
//     climate's lightning range, bearing uniform. Event id = bin * 64 + rank
//     in the bin (monotonic in time, a pure function of the bin).
//
// Random draws: counter-based SplitMix64 keyed by (seed, segment, quantity),
// (seed, segment, cell, quantity), (seed, bin, source, draw); normals are
// Irwin-Hall n = 4 approximations (bounded to +-3.46 sigma).
//
// Cost: a WeatherCursor caches the last 64 segments (forward evaluation is
// O(1) amortised; an evaluation before the cached range restarts the walk at
// segment 0 - same bits, more work). Allocation-free after construction.

#pragma once

#include "ps/environment/calendar.h"
#include "ps/environment/climate_desc.h"
#include "ps/environment/weather_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ps::environment {

enum class PrecipType : std::uint8_t { None, Drizzle, Rain, Sleet, Snow, FreezingRain, Hail };
enum class WeatherMode : std::uint8_t { Dynamic, Preset };
enum class WeatherSourceKind : std::uint8_t { Timeline, Preset, Override };

struct WeatherOptions {
    CalendarOptions calendar;
    WeatherMode mode = WeatherMode::Dynamic;
    WeatherPresetDesc preset;  // the base in Preset mode
};

struct WeatherOverride {
    real t_start = 0.0;
    real t_end = std::numeric_limits<real>::infinity();  // > t_start; +inf = open
    real ramp_s = 0.0;                                    // >= 0
    WeatherPresetDesc target;
};

// One regime segment of the chain (state at its start + drawn targets).
struct WeatherSegment {
    std::int64_t index = 0;
    real t0 = 0.0;
    real t1 = 0.0;
    int regime = 0;   // index into the timeline's regime-name table
    int season = 0;   // weather season the segment was drawn in
    // targets
    real cloud = 0.0, precip = 0.0, wind = 0.0, dir = 0.0, temp_off = 0.0, rh = 0.0, press_off = 0.0;
    // lagged values at t0 (dir unwrapped, target dir is the shortest-arc target relative to it)
    real cloud0 = 0.0, precip0 = 0.0, wind0 = 0.0, dir0 = 0.0, temp_off0 = 0.0, rh0 = 0.0, press_off0 = 0.0;
};

// One convective cell (env seconds).
struct WeatherCell {
    int index = 0;
    real t0 = 0.0, t_peak = 0.0, t1 = 0.0;
    real peak_mm_h = 0.0;
    real hail_diameter_mm = 0.0;  // 0 = no hail
};

struct WeatherSample {
    real t_env = 0.0;
    // Dominant source of the discrete outputs.
    WeatherSourceKind source = WeatherSourceKind::Timeline;
    int override_id = -1;           // the dominant override's id, else -1
    real base_weight = 1.0;         // blend coefficient of the base (timeline / preset)
    int regime = -1;                // dominant source's regime (timeline table index), -1 = unknown label
    int season = 0;                 // weather season at t
    std::int64_t segment = -1;      // base segment index (-1 in Preset mode)

    real cloud_cover = 0.0;            // [0, 1]
    real precip_mm_h = 0.0;            // total, liquid equivalent
    real precip_stratiform_mm_h = 0.0; // base timeline parts (before overrides)
    real precip_cell_mm_h = 0.0;
    PrecipType precip_type = PrecipType::None;
    real hail_diameter_mm = 0.0;       // > 0 iff precip_type == Hail
    bool in_cell = false;
    real cell_shape = 0.0;

    real wind_mean_m_s = 0.0;
    real wind_dir_deg = 0.0;           // from, [0, 360)
    real gust_factor = 1.0;
    real gust_nominal_m_s = 0.0;       // mean * factor (W4 synthesises the gust signal)

    real air_temp_c = 0.0;
    real temp_offset_k = 0.0;          // air_temp_c minus the Parton-Logan value
    real rh = 0.0;
    real dew_point_c = 0.0;
    real vapour_pressure_hpa = 0.0;
    real pressure_msl_hpa = 0.0;

    real lightning_per_min = 0.0;

    real precip_1h_mm = 0.0;           // trailing accumulations (published rate integrated)
    real precip_6h_mm = 0.0;
};

struct LightningEvent {
    std::uint64_t id = 0;   // bin * 64 + rank within the bin; increases with time
    real t_env = 0.0;
    real offset_east_m = 0.0;   // from the weather cell centre (v1: one global cell - the map's weather origin)
    real offset_north_m = 0.0;
};

class WeatherTimeline;

// Per-consumer cache of the walked chain (ring of the last 64 segments) and
// lightning scratch. Belongs to one timeline (re-initialised when used with
// another); never changes a result, only the work done.
class WeatherCursor {
public:
    WeatherCursor();
    void reset() { newest_ = -1; owner_ = nullptr; }
    std::int64_t newest_segment() const { return newest_; }
    std::uint64_t restarts() const { return restarts_; }   // diagnostics: walks restarted at segment 0

private:
    friend class WeatherTimeline;
    std::vector<WeatherSegment> ring_;
    std::vector<LightningEvent> scratch_;
    std::vector<std::uint64_t> scratch_bin_;
    std::int64_t newest_ = -1;
    const WeatherTimeline* owner_ = nullptr;
    std::uint64_t restarts_ = 0;
};

class WeatherTimeline {
public:
    static constexpr int kRingSegments = 64;
    static constexpr int kMaxOverrides = 16;
    static constexpr int kMaxCellsPerSegment = 256;
    static constexpr real kLightningBinS = 60.0;
    static constexpr int kMaxLightningPerBin = 32;
    static constexpr real kMaxLightningAgeS = 1800.0;
    static constexpr real kMaxAccumulationWindowS = 21600.0;  // 6 h

    // Throws std::invalid_argument: an invalid climate (validate_climate),
    // missing precip_phase / humidity / diurnal_temperature / climate scalars,
    // Dynamic mode without regimes, an invalid preset, lightning without the
    // climate's lightning block.
    WeatherTimeline(const ClimateDesc& climate, std::uint64_t seed, const WeatherOptions& options = {});

    WeatherSample evaluate(real t_env, WeatherCursor& cursor) const;

    // Base chain segment containing t (Dynamic mode; std::logic_error in Preset mode).
    WeatherSegment segment_at(real t_env, WeatherCursor& cursor) const;
    // The cells of a segment, in order (up to `capacity`); returns the count.
    int cells_of(const WeatherSegment& segment, WeatherCell* out, int capacity) const;

    // Published precipitation integrated over [t_a, t_b] in mm
    // (0 <= t_b - t_a <= 6 h; std::invalid_argument otherwise).
    real precipitation_mm(real t_a, real t_b, WeatherCursor& cursor) const;

    // The newest `capacity` lightning events with t_env - max_age_s < t <= t_env,
    // ascending in time; returns the count. max_age_s in [0, 1800].
    int lightning_events(real t_env, real max_age_s, WeatherCursor& cursor, LightningEvent* out, int capacity) const;

    // Overrides (sim thread, between evaluations). add_override returns an id
    // (>= 0); throws std::invalid_argument on an invalid override and
    // std::length_error when kMaxOverrides are installed.
    int add_override(const WeatherOverride& o);
    bool remove_override(int id);
    void clear_overrides();
    int override_count() const { return static_cast<int>(overrides_.size()); }

    // Regime-name table (alphabetical over every season's regimes).
    int regime_count() const { return static_cast<int>(regime_names_.size()); }
    const std::string& regime_name(int i) const { return regime_names_[static_cast<std::size_t>(i)]; }
    int regime_index(const std::string& name) const;  // -1 when unknown

    const Calendar& calendar() const { return calendar_; }
    const ClimateDesc& climate() const { return climate_; }
    std::uint64_t seed() const { return seed_; }
    WeatherMode mode() const { return mode_; }

    // FNV-1a fold of the inputs that are not the climate file: seed, mode,
    // preset and the override list (in application order).
    std::uint64_t fold_hash(std::uint64_t h) const;

private:
    struct Override {
        int id = 0;
        std::uint64_t seq = 0;
        std::uint64_t key = 0;
        int regime = -1;
        WeatherOverride o;
    };
    struct Base;
    struct Blend;

    // nullptr when the regime is not defined in that season.
    const RegimeDesc* regime_desc(int season, int regime) const {
        const int i = regime_table_[static_cast<std::size_t>(season * regime_count() + regime)];
        return i < 0 ? nullptr : &regimes_flat_[static_cast<std::size_t>(i)];
    }
    int weather_season_at(real t) const;
    WeatherSegment first_segment() const;
    WeatherSegment next_segment(const WeatherSegment& s) const;
    void draw_segment(WeatherSegment& s) const;
    void ensure(WeatherCursor& c, real t_lo, real t_hi) const;
    const WeatherSegment& find(const WeatherCursor& c, real t) const;
    template <class F> void for_each_cell(const WeatherSegment& s, F&& f) const;
    Base base_at(real t, WeatherCursor& c) const;
    real base_precip_integral(real a, real b, WeatherCursor& c) const;
    void blend_weights(real t, Blend& b) const;
    PrecipType type_for(PrecipForm form, real temp_c) const;
    PrecipType preset_type(const WeatherPresetDesc& p, real temp_c) const;
    real published_precip_at(real t, WeatherCursor& c) const;

    ClimateDesc climate_;
    Calendar calendar_;
    std::uint64_t seed_ = 0;
    WeatherMode mode_ = WeatherMode::Dynamic;
    WeatherPresetDesc preset_;
    int preset_regime_ = -1;
    std::vector<std::string> regime_names_;
    std::vector<RegimeDesc> regimes_flat_;
    std::vector<int> regime_table_;                 // [season][regime] -> regimes_flat_ index, -1 undefined
    std::vector<real> cum_;                         // [season][from][to] cumulative probabilities
    std::vector<int> initial_;                      // per season
    std::vector<Override> overrides_;               // sorted by (t_start, seq)
    int next_override_id_ = 0;
    std::uint64_t next_seq_ = 0;
};

} // namespace ps::environment
