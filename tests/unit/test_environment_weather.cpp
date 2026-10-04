// tests/unit/test_environment_weather.cpp - weather plan W3: the weather
// timeline (regimes, semi-Markov chain, cells, lags, Parton-Logan temperature,
// presets, overrides, accumulations, lightning). Tags [environment][weather].
//
// Plan tests: T1 (stationary distribution), T2 (random order == sequential,
// bit-exact), T4 (preset constant + smooth ramp), T5 (override window restores
// the timeline), T6 (Parton-Logan minimum/maximum timing with arbitrary
// day/night). T3 (class boundaries) is in test_environment_weather_classes.cpp.
//
// RECORDED SABOTAGES (performed by hand, reverted) - see the W3 progress file
// for the measured output of each:
//  - T1: next_segment draws from the PREVIOUS season's transition row: all
//    three regimes fail, z = +224 / -106 / -174 (clear 0.567 vs 0.212).
//    Without sabotage the worst z is 2.28 (200 seeds); 2000 and 10000 seeds
//    (seed ranges 1000+ and 50000+) gave worst |z| 2.30 and 1.72 with the
//    deviation shrinking like 1/sqrt(n) - noise, not a bias.
//  - T2: ensure() without the restart and without the coverage throw (find()
//    falls back to the oldest cached segment): the first random-order query
//    before the ring (t = 17093.5 s) differs.
//  - T4: ramp_weight ignores the ramp (a step): fails from t - t_start = 0
//    (precipitation 15 vs the base 0, 3602 failed assertions).
//  - T5: ramp_weight ignores t_end (the override never ends): every sample
//    after the window differs (1037 failed assertions).
//  - T6a: day_formula without the 2a term: T(t_x) misses T_max by 1.06 K and
//    the argmax moves (204 failed). T6b: night_formula using the current
//    day's T_min: the night ends 0.42 K off the next day's minimum (112 failed).
//  - accumulations: lag_integral without the transient term: 3.686 vs the
//    numeric 3.656 mm (bound 3.7e-6 mm).
//  - lightning: the dominance thinning removed: 480 strikes while the
//    lightning-free override is fully on (bound 0).

#include "environment_fixtures.h"

#include "ps/environment/air_temperature.h"
#include "ps/environment/weather.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::environment;

namespace {

WeatherOptions fixed_season(int s) {
    WeatherOptions o;
    o.calendar.mode = CalendarMode::FixedSeason;
    o.calendar.fixed_season = s;
    return o;
}

WeatherPresetDesc target_preset(real cloud, real precip, real lightning = 0.0) {
    WeatherPresetDesc p;
    p.regime = "override_test";
    p.cloud_cover = cloud;
    p.precip_mm_h = precip;
    p.precip_type = PresetPrecipType::Rain;
    p.wind_mean_m_s = 12.0;
    p.wind_dir_deg = 30.0;
    p.gust_factor = 1.8;
    p.temp_offset_k = -3.0;
    p.rh = 0.97;
    p.pressure_offset_hpa = -12.0;
    p.lightning_per_min = lightning;
    return p;
}

// Field-by-field exact comparison; returns the first differing field or "".
std::string diff_field(const WeatherSample& a, const WeatherSample& b, bool with_accumulations = true) {
#define PS_CMP(f) \
    if (!(a.f == b.f)) return #f;
    PS_CMP(t_env) PS_CMP(source) PS_CMP(override_id) PS_CMP(base_weight) PS_CMP(regime) PS_CMP(season)
    PS_CMP(segment) PS_CMP(cloud_cover) PS_CMP(precip_mm_h) PS_CMP(precip_stratiform_mm_h)
    PS_CMP(precip_cell_mm_h) PS_CMP(precip_type) PS_CMP(hail_diameter_mm) PS_CMP(in_cell) PS_CMP(cell_shape)
    PS_CMP(wind_mean_m_s) PS_CMP(wind_dir_deg) PS_CMP(gust_factor) PS_CMP(gust_nominal_m_s) PS_CMP(air_temp_c)
    PS_CMP(temp_offset_k) PS_CMP(rh) PS_CMP(dew_point_c) PS_CMP(vapour_pressure_hpa) PS_CMP(pressure_msl_hpa)
    PS_CMP(lightning_per_min)
    if (with_accumulations) {
        PS_CMP(precip_1h_mm) PS_CMP(precip_6h_mm)
    }
#undef PS_CMP
    return {};
}

std::uint64_t lcg(std::uint64_t& s) {
    s = s * 6364136223846793005ull + 1442695040888963407ull;
    return s >> 33;
}

// Mean of exp(sigma * n) for the Irwin-Hall n = 4 normal used by the
// timeline: [sinh(x)/x]^4 with x = sigma * sqrt(3) / 2 (closed form; the
// test's own derivation, independent of the implementation).
real irwin_hall_lognormal_mean_factor(real sigma) {
    const real x = sigma * 1.7320508075688772 / 2.0;
    if (x == 0.0) return 1.0;
    const real f = std::sinh(x) / x;
    return f * f * f * f;
}

} // namespace

TEST_CASE("T1: long-run regime frequencies match the stationary distribution", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    const WeatherOptions opt = fixed_season(1);   // season b
    constexpr int kSeeds = 200, kSegments = 400, kBurnIn = 40;
    const int n_count = kSegments - kBurnIn;

    real f_sum[3] = {}, f_sq[3] = {};
    // Duration statistics per regime (hours).
    real d_sum[3] = {}, d_sq[3] = {};
    long long d_n[3] = {};
    for (int seed = 0; seed < kSeeds; ++seed) {
        const WeatherTimeline tl(wc, 1000u + static_cast<std::uint64_t>(seed), opt);
        REQUIRE(tl.regime_count() == 3);
        REQUIRE(tl.regime_name(0) == "clear");
        REQUIRE(tl.regime_name(2) == "storm");
        WeatherCursor cur;
        WeatherSegment s = tl.segment_at(0.0, cur);
        int counts[3] = {};
        for (int i = 0; i < kSegments; ++i) {
            REQUIRE(s.index == i);
            REQUIRE(s.season == 1);
            if (i >= kBurnIn) {
                ++counts[s.regime];
                const real d = (s.t1 - s.t0) / 3600.0;
                d_sum[s.regime] += d;
                d_sq[s.regime] += d * d;
                ++d_n[s.regime];
            }
            s = tl.segment_at(s.t1, cur);
        }
        for (int k = 0; k < 3; ++k) {
            const real f = static_cast<real>(counts[k]) / n_count;
            f_sum[k] += f;
            f_sq[k] += f * f;
        }
    }
    std::printf("T1 (season b, %d seeds x %d segments after %d burn-in):\n", kSeeds, n_count, kBurnIn);
    std::printf("  regime   stationary   measured   sigma_mean   z\n");
    const char* names[3] = {"clear", "rain", "storm"};
    for (int k = 0; k < 3; ++k) {
        const real mean = f_sum[k] / kSeeds;
        const real var = (f_sq[k] - kSeeds * mean * mean) / (kSeeds - 1);
        const real sigma_mean = std::sqrt(var / kSeeds);
        const real z = (mean - ps::test_env::kWeatherStationaryB[k]) / sigma_mean;
        std::printf("  %-7s  %.6f     %.6f   %.6f     %+.2f\n", names[k], ps::test_env::kWeatherStationaryB[k], mean,
                    sigma_mean, z);
        INFO(names[k] << ": measured " << mean << " vs stationary " << ps::test_env::kWeatherStationaryB[k]
                      << " (sigma of the mean " << sigma_mean << ")");
        CHECK(std::fabs(z) <= 3.0);
    }
    // Durations: lognormal median 6 / 3 / 2 h, sigma 0.3, floor never binding.
    const real medians[3] = {6.0, 3.0, 2.0};
    for (int k = 0; k < 3; ++k) {
        const real n = static_cast<real>(d_n[k]);
        const real mean = d_sum[k] / n;
        const real sd = std::sqrt((d_sq[k] - n * mean * mean) / (n - 1.0));
        const real expect = medians[k] * irwin_hall_lognormal_mean_factor(0.3);
        std::printf("  duration %-7s expected %.5f h measured %.5f h (n %lld, z %+.2f)\n", names[k], expect, mean, d_n[k],
                    (mean - expect) / (sd / std::sqrt(n)));
        CHECK(std::fabs(mean - expect) <= 3.0 * sd / std::sqrt(n));
    }
}

TEST_CASE("T2: evaluate in random order equals the sequential walk bit-exactly", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    WeatherTimeline tl(wc, 77, WeatherOptions{});   // Dynamic calendar: the season changes at day 31
    WeatherOverride o1;
    o1.t_start = 10.0 * 86400.0;
    o1.t_end = 10.4 * 86400.0;
    o1.ramp_s = 900.0;
    o1.target = target_preset(0.3, 8.0, 2.0);
    tl.add_override(o1);
    WeatherOverride o2;
    o2.t_start = 10.2 * 86400.0;
    o2.t_end = 10.3 * 86400.0;
    o2.ramp_s = 0.0;
    o2.target = target_preset(0.9, 0.0);
    tl.add_override(o2);

    std::vector<real> times;
    std::uint64_t r = 12345;
    for (int i = 0; i < 700; ++i) times.push_back(-7200.0 + static_cast<real>(lcg(r) % 3456000000ull) / 1000.0);  // up to 40 days
    for (int i = 0; i < 60; ++i) times.push_back(10.0 * 86400.0 + 60.0 * i);   // dense inside the override ramp
    times.push_back(31.0 * 86400.0);
    std::sort(times.begin(), times.end());

    WeatherCursor seq;
    std::vector<WeatherSample> ref;
    std::vector<std::vector<LightningEvent>> ref_ev;
    LightningEvent buf[256];
    for (real t : times) {
        ref.push_back(tl.evaluate(t, seq));
        const int n = tl.lightning_events(t, 1800.0, seq, buf, 256);
        ref_ev.emplace_back(buf, buf + n);
    }
    CHECK(seq.restarts() == 0);

    // Random order on one cursor (restarts happen) and a fresh cursor per query.
    std::vector<std::size_t> order(times.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    for (std::size_t i = order.size() - 1; i > 0; --i) std::swap(order[i], order[lcg(r) % (i + 1)]);
    WeatherCursor rnd;
    for (std::size_t q : order) {
        const WeatherSample a = tl.evaluate(times[q], rnd);
        const std::string field = diff_field(a, ref[q]);
        INFO("t = " << times[q] << ", first differing field: " << field);
        REQUIRE(field.empty());
        const int n = tl.lightning_events(times[q], 1800.0, rnd, buf, 256);
        REQUIRE(static_cast<std::size_t>(n) == ref_ev[q].size());
        for (int i = 0; i < n; ++i) {
            CHECK(buf[i].id == ref_ev[q][static_cast<std::size_t>(i)].id);
            CHECK(buf[i].t_env == ref_ev[q][static_cast<std::size_t>(i)].t_env);
            CHECK(buf[i].offset_east_m == ref_ev[q][static_cast<std::size_t>(i)].offset_east_m);
            CHECK(buf[i].offset_north_m == ref_ev[q][static_cast<std::size_t>(i)].offset_north_m);
        }
    }
    CHECK(rnd.restarts() > 0);
    for (std::size_t q = 0; q < times.size(); q += 37) {
        WeatherCursor fresh;
        REQUIRE(diff_field(tl.evaluate(times[q], fresh), ref[q]).empty());
    }
    // A copy of the timeline and a second timeline with the same inputs agree too.
    const WeatherTimeline copy = tl;
    WeatherCursor cc;
    for (std::size_t q = 0; q < times.size(); q += 11) REQUIRE(diff_field(copy.evaluate(times[q], cc), ref[q]).empty());
    // Season change observed, and some samples saw cells and the override.
    bool saw_b = false, saw_override = false, saw_cell = false;
    for (const WeatherSample& s : ref) {
        saw_b = saw_b || s.season == 1;
        saw_override = saw_override || s.source == WeatherSourceKind::Override;
        saw_cell = saw_cell || s.in_cell;
    }
    CHECK(saw_b);
    CHECK(saw_override);
    CHECK(saw_cell);
}

TEST_CASE("Chain walk: lags are continuous at segment boundaries and start at the targets", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    const WeatherTimeline tl(wc, 5, fixed_season(0));
    WeatherCursor c;
    const WeatherSegment s0 = tl.segment_at(0.0, c);
    CHECK(s0.cloud0 == s0.cloud);
    CHECK(s0.regime == tl.regime_index("clear"));   // season a's initial_regime
    // Before t = 0 the weather is segment 0's.
    const WeatherSample before = tl.evaluate(-3600.0, c);
    CHECK(before.segment == 0);
    CHECK(before.cloud_cover == s0.cloud);
    WeatherSegment s = s0;
    for (int i = 0; i < 50; ++i) {
        const WeatherSegment n = tl.segment_at(s.t1, c);
        REQUIRE(n.t0 == s.t1);
        const WeatherSample l = tl.evaluate(std::nextafter(n.t0, -1e300), c);
        const WeatherSample h = tl.evaluate(n.t0, c);
        CHECK(std::fabs(l.cloud_cover - h.cloud_cover) < 1e-9);
        CHECK(std::fabs(l.wind_mean_m_s - h.wind_mean_m_s) < 1e-9);
        CHECK(std::fabs(l.pressure_msl_hpa - h.pressure_msl_hpa) < 1e-9);
        CHECK(std::fabs(l.precip_stratiform_mm_h - h.precip_stratiform_mm_h) < 1e-9);
        // Lag toward the target: at t0 + 5 tau the value is within e^-5 of it.
        const WeatherSample late = tl.evaluate(n.t0 + 5.0 * wc.lags->cloud_s, c);
        if (n.t1 > n.t0 + 5.0 * wc.lags->cloud_s)
            CHECK(std::fabs(late.cloud_cover - n.cloud) <= std::fabs(n.cloud0 - n.cloud) * std::exp(-5.0) * (1.0 + 1e-9) + 1e-15);
        s = n;
    }
}

TEST_CASE("Cells: inside their segment, triangle shape, hail and lightning only in cells", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    const WeatherTimeline tl(wc, 9, fixed_season(1));
    WeatherCursor c;
    WeatherSegment s = tl.segment_at(0.0, c);
    WeatherCell cells[WeatherTimeline::kMaxCellsPerSegment];
    int storms = 0, n_cells = 0, hail_seen = 0;
    for (int i = 0; i < 300; ++i) {
        const int n = tl.cells_of(s, cells, WeatherTimeline::kMaxCellsPerSegment);
        if (tl.regime_name(s.regime) != "storm") {
            CHECK(n == 0);
        } else {
            ++storms;
            for (int j = 0; j < n; ++j) {
                const WeatherCell& cl = cells[j];
                ++n_cells;
                CHECK(cl.t0 > (j == 0 ? s.t0 : cells[j - 1].t1));
                CHECK(cl.t1 <= s.t1);
                CHECK(std::fabs(cl.t_peak - (cl.t0 + 0.3 * (cl.t1 - cl.t0))) < 1e-6);
                const WeatherSample pk = tl.evaluate(cl.t_peak, c);
                CHECK(pk.in_cell);
                CHECK(pk.cell_shape == 1.0);
                CHECK(pk.precip_cell_mm_h == cl.peak_mm_h);
                CHECK(pk.lightning_per_min == 3.0);
                CHECK(std::fabs(pk.gust_factor - 2.2) < 1e-12);
                if (cl.hail_diameter_mm > 0.0) {
                    ++hail_seen;
                    CHECK(pk.precip_type == PrecipType::Hail);
                    CHECK(pk.hail_diameter_mm == cl.hail_diameter_mm);
                    // Below min_shape 0.6: no hail.
                    const WeatherSample edge = tl.evaluate(cl.t0 + 0.5 * (cl.t_peak - cl.t0), c);
                    CHECK(edge.precip_type != PrecipType::Hail);
                }
                const WeatherSample mid = tl.evaluate(cl.t0 + 0.5 * (cl.t_peak - cl.t0), c);
                CHECK(std::fabs(mid.cell_shape - 0.5) < 1e-9);
                const real gap_t = 0.5 * ((j == 0 ? s.t0 : cells[j - 1].t1) + cl.t0);
                const WeatherSample gap = tl.evaluate(gap_t, c);
                CHECK_FALSE(gap.in_cell);
                CHECK(gap.lightning_per_min == 0.0);
                CHECK(gap.precip_cell_mm_h == 0.0);
            }
        }
        s = tl.segment_at(s.t1, c);
    }
    CHECK(storms > 50);
    CHECK(n_cells > 100);
    CHECK(hail_seen > 10);
}

TEST_CASE("T4: a preset is constant and an override ramps smoothly", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    WeatherOptions po = fixed_season(0);
    po.mode = WeatherMode::Preset;
    po.preset = target_preset(0.85, 6.0, 1.0);
    po.preset.regime = "rain";
    const WeatherTimeline preset(wc, 1, po);
    WeatherCursor c;
    const WeatherSample p0 = preset.evaluate(1000.0, c);
    CHECK(p0.source == WeatherSourceKind::Preset);
    CHECK(p0.regime == preset.regime_index("rain"));
    CHECK(p0.segment == -1);
    for (int i = 0; i < 200; ++i) {
        const real t = -86400.0 + 4321.0 * i;
        const WeatherSample s = preset.evaluate(t, c);
        CHECK(s.cloud_cover == 0.85);
        CHECK(s.precip_mm_h == 6.0);
        CHECK(s.precip_type == PrecipType::Rain);
        CHECK(s.wind_mean_m_s == 12.0);
        CHECK(s.wind_dir_deg == 30.0);
        CHECK(s.gust_factor == 1.8);
        CHECK(s.rh == 0.97);
        CHECK(s.pressure_msl_hpa == p0.pressure_msl_hpa);
        CHECK(s.lightning_per_min == 1.0);
        CHECK(std::fabs(s.temp_offset_k - -3.0) < 1e-12);
        CHECK(std::fabs(s.precip_1h_mm - 6.0) < 1e-12);
        CHECK(std::fabs(s.precip_6h_mm - 36.0) < 1e-11);
    }
    CHECK_THROWS(preset.segment_at(0.0, c));

    // Smooth ramp: smoothstep from the timeline to the target over ramp_s.
    const WeatherTimeline base(wc, 31, fixed_season(0));
    WeatherTimeline ramped(wc, 31, fixed_season(0));
    WeatherOverride o;
    o.t_start = 5.0 * 86400.0;
    o.ramp_s = 1200.0;
    o.target = target_preset(1.0, 15.0);
    ramped.add_override(o);
    WeatherCursor cb, cr;
    real max_step = 0.0;
    for (int i = -60; i <= 1260; ++i) {
        const real t = o.t_start + i;
        const WeatherSample b = base.evaluate(t, cb);
        const WeatherSample r = ramped.evaluate(t, cr);
        const real x = std::clamp(static_cast<real>(i) / o.ramp_s, 0.0, 1.0);
        const real w = x * x * (3.0 - 2.0 * x);
        INFO("t - t_start = " << i);
        CHECK(std::fabs(r.cloud_cover - (b.cloud_cover + (1.0 - b.cloud_cover) * w)) < 1e-12);
        CHECK(std::fabs(r.wind_mean_m_s - (b.wind_mean_m_s + (12.0 - b.wind_mean_m_s) * w)) < 1e-12);
        CHECK(std::fabs(r.precip_mm_h - (b.precip_mm_h + (15.0 - b.precip_mm_h) * w)) < 1e-12);
        if (i >= 1200) CHECK(r.cloud_cover == 1.0);
        if (i < 0) CHECK(diff_field(r, b).empty());
    }
    // No jump: the largest 1 s change of the blended precipitation stays below
    // 1.5 x |target - base| / ramp_s (smoothstep's peak slope) plus the base's
    // own change.
    WeatherCursor cr2, cb2;
    WeatherSample prev_r = ramped.evaluate(o.t_start - 1.0, cr2), prev_b = base.evaluate(o.t_start - 1.0, cb2);
    for (int i = 0; i <= 1300; ++i) {
        const real t = o.t_start + i;
        const WeatherSample r = ramped.evaluate(t, cr2);
        const WeatherSample b = base.evaluate(t, cb2);
        const real allowed = 1.5 * std::fabs(15.0 - b.precip_mm_h) / o.ramp_s * (1.0 + 1e-6) +
                             std::fabs(b.precip_mm_h - prev_b.precip_mm_h) + 1e-9;
        max_step = std::max(max_step, std::fabs(r.precip_mm_h - prev_r.precip_mm_h) / allowed);
        prev_r = r;
        prev_b = b;
    }
    CHECK(max_step <= 1.0);
    // Discrete outputs switch to the override where it dominates (w > 1/2).
    WeatherCursor c3;
    CHECK(ramped.evaluate(o.t_start + 0.4 * o.ramp_s, c3).source == WeatherSourceKind::Timeline);
    CHECK(ramped.evaluate(o.t_start + 0.6 * o.ramp_s, c3).source == WeatherSourceKind::Override);
}

TEST_CASE("T5: after an override window the timeline is restored bit-exactly", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    const WeatherTimeline base(wc, 4242, WeatherOptions{});
    WeatherTimeline ov(wc, 4242, WeatherOptions{});
    WeatherOverride o;
    o.t_start = 3.0 * 86400.0;
    o.t_end = o.t_start + 7200.0;
    o.ramp_s = 600.0;
    o.target = target_preset(0.95, 25.0, 4.0);
    const int id = ov.add_override(o);
    CHECK(id >= 0);
    CHECK(ov.override_count() == 1);
    WeatherCursor cb, co;
    const real t_free = o.t_end + o.ramp_s;
    int checked_after = 0;
    for (real t = o.t_start - 3600.0; t < t_free + 30000.0; t += 37.0) {
        const WeatherSample b = base.evaluate(t, cb);
        const WeatherSample s = ov.evaluate(t, co);
        INFO("t = " << t);
        if (t < o.t_start) {
            CHECK(diff_field(s, b).empty());
        } else if (t >= o.t_start + o.ramp_s && t < o.t_end) {
            CHECK(s.source == WeatherSourceKind::Override);
            CHECK(s.override_id == id);
            CHECK(s.base_weight == 0.0);
            CHECK(s.cloud_cover == 0.95);
            CHECK(s.precip_mm_h == 25.0);
            CHECK(s.lightning_per_min == 4.0);
            CHECK(s.precip_type == PrecipType::Rain);
        } else if (t >= t_free) {
            const std::string field = diff_field(s, b, false);
            INFO("first differing field: " << field);
            CHECK(field.empty());
            if (t - WeatherTimeline::kMaxAccumulationWindowS >= t_free) {
                CHECK(diff_field(s, b, true).empty());
                ++checked_after;
            } else {
                CHECK(s.precip_6h_mm > b.precip_6h_mm);   // the window's rain is still in the 6 h sum
            }
        }
    }
    CHECK(checked_after > 100);
    // Lightning after the window equals the timeline's again.
    LightningEvent eb[256], eo[256];
    const real t_late = t_free + 1800.0 + 60.0;
    const int nb = base.lightning_events(t_late, 1800.0, cb, eb, 256);
    const int no = ov.lightning_events(t_late, 1800.0, co, eo, 256);
    REQUIRE(nb == no);
    for (int i = 0; i < nb; ++i) CHECK((eb[i].id == eo[i].id && eb[i].t_env == eo[i].t_env));
    // Removing the override restores everything, including the hash.
    CHECK(ov.fold_hash(1) != base.fold_hash(1));
    CHECK(ov.remove_override(id));
    CHECK_FALSE(ov.remove_override(id));
    CHECK(ov.fold_hash(1) == base.fold_hash(1));
    WeatherCursor c2;
    CHECK(diff_field(ov.evaluate(o.t_start + 3600.0, c2), base.evaluate(o.t_start + 3600.0, cb)).empty());
}

TEST_CASE("Overrides: validation, capacity, ordering", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    WeatherTimeline tl(wc, 1, WeatherOptions{});
    WeatherOverride bad;
    bad.target = target_preset(0.5, 1.0);
    bad.t_start = 100.0;
    bad.t_end = 100.0;
    CHECK_THROWS_AS(tl.add_override(bad), std::invalid_argument);
    bad.t_end = 200.0;
    bad.ramp_s = -1.0;
    CHECK_THROWS_AS(tl.add_override(bad), std::invalid_argument);
    bad.ramp_s = 0.0;
    bad.target.cloud_cover = 1.5;
    CHECK_THROWS_AS(tl.add_override(bad), std::invalid_argument);
    WeatherOverride ok;
    ok.target = target_preset(0.5, 1.0);
    for (int i = 0; i < WeatherTimeline::kMaxOverrides; ++i) {
        ok.t_start = 1000.0 - i;   // inserted out of order
        tl.add_override(ok);
    }
    CHECK_THROWS_AS(tl.add_override(ok), std::length_error);
    tl.clear_overrides();
    CHECK(tl.override_count() == 0);

    // Later t_start wins where both are fully on.
    WeatherOverride a, b;
    a.t_start = 1000.0;
    a.target = target_preset(0.1, 1.0);
    b.t_start = 2000.0;
    b.t_end = 3000.0;
    b.target = target_preset(0.7, 2.0);
    const int ib = tl.add_override(b);
    tl.add_override(a);   // added later but starts earlier: applied first
    WeatherCursor c;
    CHECK(tl.evaluate(1500.0, c).cloud_cover == 0.1);
    const WeatherSample s = tl.evaluate(2500.0, c);
    CHECK(s.cloud_cover == 0.7);
    CHECK(s.override_id == ib);
    CHECK(tl.evaluate(3500.0, c).cloud_cover == 0.1);
}

TEST_CASE("T6: Parton-Logan minimum at sunrise + c and maximum timing for arbitrary day/night", "[environment][weather]") {
    struct Config { real day_a, night_a, day_b, night_b; };
    const Config configs[] = {{50400.0, 36000.0, 30000.0, 56400.0}, {20000.0, 10000.0, 6000.0, 30000.0}};
    for (const Config& cf : configs) {
        const ClimateDesc wc = ps::test_env::weather_two_season(cf.day_a, cf.night_a, cf.day_b, cf.night_b);
        const Calendar cal(wc);
        const DiurnalTempDesc& p = *wc.diurnal_temperature;
        auto T = [&](real t) { return parton_logan_air_temp_c(cal, p, t, 0.0); };
        // Walk days 0..40 (crosses the season change at day 31).
        real t = 1.0;
        int days = 0;
        real max_jump_tn = 0.0;
        while (days < 40) {
            const CalendarSample cs = cal.sample(t);
            const real start = t - cs.time_of_day_s;
            const DayValues& dv = cal.day(cs.day_of_year);
            const real scale = dv.length_s() / p.reference_cycle_s;
            const real a = p.a_h * 3600.0 * scale;
            const real cc = std::clamp(p.c_h * 3600.0 * scale, -0.25 * dv.night_s, 0.25 * dv.day_s);
            const real sunrise = start + 0.5 * dv.night_s;
            const real t_n = sunrise + cc;
            const real t_s = sunrise + dv.day_s;
            const real t_x = t_n + 0.5 * (dv.day_s + 2.0 * a);
            const real t_min = dv.climate.temp_mean_c - 0.5 * dv.climate.temp_range_k;
            const real t_max = dv.climate.temp_mean_c + 0.5 * dv.climate.temp_range_k;
            INFO("config day_a " << cf.day_a << ", day " << cs.day_of_year);
            CHECK(std::fabs(T(t_n) - t_min) < 1e-9);
            CHECK(T(t_n - 1.0) > T(t_n));
            CHECK(T(t_n + 1.0) > T(t_n));
            CHECK(std::fabs(T(t_x) - t_max) < 1e-9);
            // Numerical argmax over the day at 1 s.
            real best = -1e300, arg = 0.0;
            for (real u = t_n; u < t_s; u += 1.0) {
                const real v = T(u);
                if (v > best) { best = v; arg = u; }
            }
            CHECK(std::fabs(arg - t_x) <= 1.0);
            const DiurnalTimes dt = parton_logan_times(cal, p, t_n + 10.0);
            CHECK(std::fabs(dt.t_min - t_n) < 1e-6);
            CHECK(std::fabs(dt.t_max - t_x) < 1e-6);
            CHECK(std::fabs(dt.sunset - t_s) < 1e-6);
            // Continuity at sunset and at midnight.
            CHECK(std::fabs(T(std::nextafter(t_s, -1e300)) - T(t_s)) < 1e-6);
            const real next_start = start + dv.length_s();
            CHECK(std::fabs(T(std::nextafter(next_start, -1e300)) - T(next_start)) < 1e-6);
            // The night ends at the NEXT day's minimum.
            const DayValues& nv = cal.day((cs.day_of_year + 1) % cal.days_per_year());
            const real nscale = nv.length_s() / p.reference_cycle_s;
            const real n_tn = next_start + 0.5 * nv.night_s +
                              std::clamp(p.c_h * 3600.0 * nscale, -0.25 * nv.night_s, 0.25 * nv.day_s);
            const real n_tmin = nv.climate.temp_mean_c - 0.5 * nv.climate.temp_range_k;
            max_jump_tn = std::max(max_jump_tn, std::fabs(t_min - n_tmin));
            CHECK(std::fabs(T(std::nextafter(n_tn, -1e300)) - n_tmin) < 1e-6);
            t = next_start + 1.0;
            ++days;
        }
        CHECK(max_jump_tn > 0.05);   // the day-to-day T_min change the night check relies on
    }
    // Cloud cover narrows the range: N = 1 leaves (1 - 0.6) of it.
    const ClimateDesc wc = ps::test_env::weather_two_season();
    const Calendar cal(wc, CalendarOptions{CalendarMode::FixedSeason, 0, 0});
    const DiurnalTimes dt = parton_logan_times(cal, *wc.diurnal_temperature, 86400.0 * 3 + 30000.0);
    const real lo = parton_logan_air_temp_c(cal, *wc.diurnal_temperature, dt.t_min, 1.0);
    const real hi = parton_logan_air_temp_c(cal, *wc.diurnal_temperature, dt.t_max, 1.0);
    CHECK(std::fabs((hi - lo) - 0.4 * 10.0) < 1e-9);
}

TEST_CASE("Humidity: dew point and vapour pressure are consistent; RH floor in precipitation", "[environment][weather]") {
    for (real temp : {-20.0, -3.0, 0.0, 12.5, 35.0})
        for (real rh : {0.05, 0.4, 0.8, 1.0}) {
            const real td = dew_point_c(temp, rh);
            CHECK(std::fabs(saturation_vapour_pressure_hpa(td) - rh * saturation_vapour_pressure_hpa(temp)) <
                  1e-9 * saturation_vapour_pressure_hpa(temp));
            CHECK(td <= temp + 1e-12);
        }
    // Magnus AERK at 0 degC: 6.1094 hPa (Alduchov & Eskridge 1996).
    CHECK(saturation_vapour_pressure_hpa(0.0) == 6.1094);
    const ClimateDesc wc = ps::test_env::weather_two_season();
    WeatherOptions po = fixed_season(0);
    po.mode = WeatherMode::Preset;
    po.preset = target_preset(1.0, 0.25);
    po.preset.rh = 0.5;
    const WeatherTimeline tl(wc, 1, po);
    WeatherCursor c;
    // P / precip_full = 0.5: halfway from 0.5 to the 0.95 floor.
    CHECK(std::fabs(tl.evaluate(100.0, c).rh - 0.725) < 1e-12);
}

TEST_CASE("Accumulations: closed form matches a 1 s numerical integral", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    WeatherTimeline tl(wc, 3, fixed_season(1));
    WeatherCursor c;
    // Find a storm segment with a cell, then a rain segment.
    WeatherSegment s = tl.segment_at(0.0, c);
    WeatherCell cells[WeatherTimeline::kMaxCellsPerSegment];
    real t_storm = -1.0;
    for (int i = 0; i < 200 && t_storm < 0.0; ++i) {
        if (tl.cells_of(s, cells, WeatherTimeline::kMaxCellsPerSegment) > 0 && s.index > 3)
            t_storm = cells[0].t_peak + 900.0;
        s = tl.segment_at(s.t1, c);
    }
    REQUIRE(t_storm > 0.0);
    auto numeric = [&](const WeatherTimeline& w, real a, real b) {
        WeatherCursor cc;
        real sum = 0.0;
        for (real u = a + 0.5; u < b; u += 1.0) sum += w.evaluate(u, cc).precip_mm_h;
        return sum / 3600.0;
    };
    const real exact = tl.precipitation_mm(t_storm - 3600.0, t_storm, c);
    const real num = numeric(tl, t_storm - 3600.0, t_storm);
    INFO("closed form " << exact << " mm, numeric " << num << " mm");
    CHECK(exact > 0.1);
    CHECK(std::fabs(exact - num) <= 1e-6 * exact + 1e-9);
    CHECK(tl.evaluate(t_storm, c).precip_1h_mm == exact);

    // A ramping override inside the window: Simpson pieces.
    WeatherOverride o;
    o.t_start = t_storm - 2000.0;
    o.t_end = t_storm - 900.0;
    o.ramp_s = 400.0;
    o.target = target_preset(1.0, 40.0);
    tl.add_override(o);
    const real ex2 = tl.precipitation_mm(t_storm - 3600.0, t_storm, c);
    const real num2 = numeric(tl, t_storm - 3600.0, t_storm);
    INFO("with override: closed/Simpson " << ex2 << " mm, numeric " << num2 << " mm");
    CHECK(std::fabs(ex2 - num2) <= 1e-4 * ex2);
    CHECK_THROWS_AS(tl.precipitation_mm(0.0, 30000.0, c), std::invalid_argument);
    CHECK_THROWS_AS(tl.precipitation_mm(10.0, 0.0, c), std::invalid_argument);
}

TEST_CASE("Lightning: rate, ordering, ids, range and dominance", "[environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    // Preset: a constant 3 per minute.
    WeatherOptions po = fixed_season(0);
    po.mode = WeatherMode::Preset;
    po.preset = target_preset(1.0, 10.0, 3.0);
    const WeatherTimeline tl(wc, 8, po);
    WeatherCursor c;
    static LightningEvent buf[2048];
    long long total = 0;
    std::uint64_t last_id = 0;
    real last_t = -1e300;
    constexpr int kWindows = 200;
    for (int k = 1; k <= kWindows; ++k) {
        const real t = 1800.0 * k + 0.25;
        const int n = tl.lightning_events(t, 1800.0, c, buf, 2048);
        for (int i = 0; i < n; ++i) {
            const LightningEvent& e = buf[i];
            CHECK(e.t_env > t - 1800.0);
            CHECK(e.t_env <= t);
            CHECK(e.id > last_id);
            CHECK(e.t_env >= last_t);
            last_id = e.id;
            last_t = e.t_env;
            const real r = std::sqrt(e.offset_east_m * e.offset_east_m + e.offset_north_m * e.offset_north_m);
            CHECK(r >= 500.0 * (1.0 - 1e-12));
            CHECK(r <= 15000.0 * (1.0 + 1e-12));
        }
        total += n;
    }
    const real expect = 3.0 * 30.0 * kWindows;
    std::printf("lightning: %lld events vs %.0f expected (Poisson sigma %.1f)\n", total, expect, std::sqrt(expect));
    CHECK(std::fabs(static_cast<real>(total) - expect) <= 3.0 * std::sqrt(expect));
    // Overlapping windows report the same events.
    LightningEvent a[256], b[256];
    const int na = tl.lightning_events(10000.0, 600.0, c, a, 256);
    const int nb = tl.lightning_events(10300.0, 900.0, c, b, 256);
    int matched = 0;
    for (int i = 0; i < na; ++i)
        for (int j = 0; j < nb; ++j)
            if (a[i].id == b[j].id) {
                CHECK(a[i].t_env == b[j].t_env);
                CHECK(a[i].offset_east_m == b[j].offset_east_m);
                ++matched;
            }
    CHECK(matched == na);   // [9400, 10000] lies inside [9400, 10300]
    // Capacity keeps the newest.
    const int n3 = tl.lightning_events(10000.0, 600.0, c, b, 3);
    REQUIRE(n3 == std::min(3, na));
    for (int i = 0; i < n3; ++i) CHECK(b[i].id == a[na - n3 + i].id);
    CHECK_THROWS_AS(tl.lightning_events(0.0, 2000.0, c, b, 4), std::invalid_argument);

    // Dynamic: events only inside cells, never while a lightning-free override dominates.
    WeatherTimeline dyn(wc, 21, fixed_season(1));
    WeatherOverride quiet;
    quiet.t_start = 2.0 * 86400.0;
    quiet.t_end = 4.0 * 86400.0;
    quiet.ramp_s = 1800.0;
    quiet.target = target_preset(0.2, 0.0, 0.0);
    dyn.add_override(quiet);
    WeatherCursor dc;
    int in_cells = 0, during_quiet = 0;
    for (real t = 1800.0; t < 6.0 * 86400.0; t += 1800.0) {
        const int n = dyn.lightning_events(t, 1800.0, dc, buf, 2048);
        for (int i = 0; i < n; ++i) {
            const WeatherSample s = dyn.evaluate(buf[i].t_env, dc);
            CHECK(s.in_cell);
            CHECK(s.lightning_per_min > 0.0);
            CHECK(s.source == WeatherSourceKind::Timeline);
            if (buf[i].t_env >= quiet.t_start + quiet.ramp_s && buf[i].t_env < quiet.t_end) ++during_quiet;
            ++in_cells;
        }
    }
    CHECK(in_cells > 50);
    CHECK(during_quiet == 0);   // the lightning-free override dominates the whole time it is fully on
    // ... although the timeline itself has strikes there.
    const WeatherTimeline free_tl(wc, 21, fixed_season(1));
    WeatherCursor fc;
    int free_quiet = 0;
    for (real t = quiet.t_start + quiet.ramp_s + 1800.0; t <= quiet.t_end; t += 1800.0)
        free_quiet += free_tl.lightning_events(t, 1800.0, fc, buf, 2048);
    CHECK(free_quiet > 10);
}

TEST_CASE("Construction rejects incomplete climates and presets", "[environment][weather]") {
    ClimateDesc wc = ps::test_env::weather_two_season();
    {
        ClimateDesc d = wc;
        d.diurnal_temperature.reset();
        CHECK_THROWS_AS(WeatherTimeline(d, 1), std::invalid_argument);
    }
    {
        ClimateDesc d = ps::test_env::three_season_explicit();   // no regimes
        d.precip_phase = PrecipPhaseDesc{};
        d.humidity = HumidityDesc{};
        d.diurnal_temperature = DiurnalTempDesc{};
        CHECK_THROWS_AS(WeatherTimeline(d, 1), std::invalid_argument);
        WeatherOptions po;
        po.mode = WeatherMode::Preset;
        po.preset = target_preset(0.5, 1.0);
        CHECK_NOTHROW(WeatherTimeline(d, 1, po));   // presets need no regimes
        po.preset.lightning_per_min = 1.0;
        CHECK_THROWS_AS(WeatherTimeline(d, 1, po), std::invalid_argument);   // no lightning block
    }
    {
        ClimateDesc d = wc;
        d.seasons[1].weather->transitions[0].to[0].p = 0.2;   // row sums to 1.1
        CHECK_THROWS_AS(WeatherTimeline(d, 1), std::invalid_argument);
    }
}

TEST_CASE("Cost of evaluate()", "[.][diag][environment][weather]") {
    const ClimateDesc wc = ps::test_env::weather_two_season();
    WeatherTimeline tl(wc, 99, WeatherOptions{});
    WeatherCursor c;
    volatile real sink = 0.0;
    const int n = 200000;
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) sink = sink + tl.evaluate(1000.0 + i * 0.25, c).air_temp_c;   // 0.25 s steps
    auto t1 = std::chrono::steady_clock::now();
    const double seq_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / n;
    std::uint64_t r = 7;
    const int m = 2000;
    t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < m; ++i) sink = sink + tl.evaluate(static_cast<real>(lcg(r) % 2592000ull), c).air_temp_c;
    t1 = std::chrono::steady_clock::now();
    const double rnd_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / m;
    WeatherOverride o;
    o.t_start = 2000.0;
    o.ramp_s = 600.0;
    o.target = target_preset(1.0, 5.0, 1.0);
    tl.add_override(o);
    WeatherCursor c2;
    t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) sink = sink + tl.evaluate(1000.0 + i * 0.25, c2).air_temp_c;
    t1 = std::chrono::steady_clock::now();
    const double ov_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / n;
    LightningEvent buf[256];
    t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 20000; ++i) sink = sink + tl.lightning_events(1000.0 + i * 0.25, 60.0, c2, buf, 256);
    t1 = std::chrono::steady_clock::now();
    const double li_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / 20000;
    std::printf("evaluate(): sequential 0.25 s steps %.3f us, random t in 30 days %.1f us (restarts %llu), "
                "sequential with a ramping override %.3f us; lightning_events(60 s) %.3f us\n",
                seq_us, rnd_us, static_cast<unsigned long long>(c.restarts()), ov_us, li_us);
    // Storm-heavy season b over 30 days in 10 s steps, timed per segment kind.
    const WeatherTimeline sb(wc, 99, fixed_season(1));
    WeatherCursor cs;
    double storm_us = 0.0, other_us = 0.0, worst_us = 0.0;
    long long n_storm = 0, n_other = 0;
    for (int i = 0; i < 259200; ++i) {
        const auto a = std::chrono::steady_clock::now();
        const WeatherSample ws = sb.evaluate(i * 10.0, cs);
        const auto b = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(b - a).count();
        worst_us = std::max(worst_us, us);
        if (sb.regime_name(ws.regime) == "storm") {
            storm_us += us;
            ++n_storm;
        } else {
            other_us += us;
            ++n_other;
        }
    }
    std::printf("evaluate() season b, 10 s steps: storm segments %.3f us (n %lld), others %.3f us (n %lld), "
                "worst single call %.1f us\n",
                storm_us / static_cast<double>(n_storm), n_storm, other_us / static_cast<double>(n_other), n_other,
                worst_us);
}
