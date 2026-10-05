// tests/unit/test_environment_season_override.cpp - weather plan W3b (owner answer Q12): the owner's
// "manual weather" model - a weather preset overrides the REST OF THE CURRENT SEASON and resets at the
// next season; the season can be locked (then it lasts until ended). Timeline level
// (WeatherTimeline::add_season_override / set_calendar / end_all_overrides) and Environment level
// (add_season_weather_override / set_calendar_lock). The World/vehicle level is in
// test_world_weather_override.cpp. Tags [environment][weather]; links ps_environment only.
//
// RECORDED SABOTAGES (performed by hand, reverted; symptoms in out/weather_w3b_progress.md):
//  - S1 t_end = season end (the ramp-out AFTER the boundary instead of finishing at it): "the next
//    season is the timeline's bit-exactly" failed at t = end (the override still weighs ~1) and "the ramp
//    finishes exactly at the season end" failed.
//  - S2 the season end taken from the START of the current season (+ its length of the FIRST season):
//    the end moved by a whole season in b-season cases ("ends with the season" failed).
//  - S3 season_locked() ignored (a locked season still gets a finite end): "a locked season keeps the
//    override" failed.
//  - S4 set_calendar without bumping the cursor generation: "the chain re-derived under the new calendar"
//    failed (stale ring: the old season's draws).
//  - S5 set_calendar_lock keeping t_env (no re-anchor): "game day and fraction of the day preserved" failed.
//  - S6 ramp_weight without the exact-zero early exit and t_end rounded up: "bit-exact from the boundary"
//    failed on the precipitation (an ulp-sized weight left 3e-32 mm/h at a zero base).

#include "environment_fixtures.h"

#include "ps/environment/environment.h"
#include "ps/environment/weather.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::environment;

namespace {

constexpr real kDay = 86400.0;
constexpr real kInf = std::numeric_limits<real>::infinity();

WeatherPresetDesc storm_preset(real cloud = 0.95, real precip = 25.0, real lightning = 4.0) {
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

// weather_two_season(): seasons a and b of 31 days, every day 86400 s, epoch at the start of season a, so
// season a is [0, 31 days) and season b [31, 62 days) of the 62-day year.
constexpr real kSeasonAEnd = 31.0 * kDay;
constexpr real kYear = 62.0 * kDay;

} // namespace

TEST_CASE("W3b season-end override: the preset inside the season, the timeline bit-exactly from the boundary",
          "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    const WeatherTimeline base(wc, 777, WeatherOptions{});
    WeatherTimeline ov(wc, 777, WeatherOptions{});
    REQUIRE(ov.calendar().season_end_env_time(0.0) == kSeasonAEnd);

    const real t_now = 10.0 * kDay + 3600.0;
    const real ramp = 1800.0;
    const int id = ov.add_season_override(storm_preset(), ramp, t_now);
    REQUIRE(ov.override_is_season_bound(id));
    const WeatherOverride* w = ov.find_override(id);
    REQUIRE(w != nullptr);
    CHECK(w->t_start == t_now);
    CHECK(w->ramp_s == ramp);
    CHECK(w->t_end + w->ramp_s <= kSeasonAEnd);        // the ramp-out is complete at the boundary ...
    CHECK(kSeasonAEnd - (w->t_end + w->ramp_s) < 1e-6); // ... and starts as late as that allows (this is "ends with the season")
    const real t_end = w->t_end;

    WeatherCursor cb, co;
    int after = 0, inside = 0, before = 0;
    for (real t = t_now - 7200.0; t < kSeasonAEnd + 40000.0; t += 61.0) {
        const WeatherSample b = base.evaluate(t, cb);
        const WeatherSample s = ov.evaluate(t, co);
        INFO("t = " << t);
        if (t < t_now) {
            CHECK(diff_field(s, b).empty());
            ++before;
        } else if (t >= t_now + ramp && t < t_end) {
            CHECK(s.source == WeatherSourceKind::Override);
            CHECK(s.override_id == id);
            CHECK(s.base_weight == 0.0);
            CHECK(s.cloud_cover == 0.95);
            CHECK(s.precip_mm_h == 25.0);
            CHECK(s.wind_mean_m_s == 12.0);
            CHECK(s.lightning_per_min == 4.0);
            CHECK(s.precip_type == PrecipType::Rain);
            ++inside;
        } else if (t >= kSeasonAEnd) {
            // "the next season is the timeline's": every field but the trailing accumulations is bit-exact
            // from the boundary on; the 1 h / 6 h sums still contain the override's rain until 6 h later.
            const std::string field = diff_field(s, b, false);
            INFO("first differing field: " << field);
            CHECK(field.empty());
            CHECK(s.season == 1);
            if (t - WeatherTimeline::kMaxAccumulationWindowS >= kSeasonAEnd) {
                CHECK(diff_field(s, b, true).empty());
                ++after;
            }
        }
    }
    CHECK(before > 50);
    CHECK(inside > 500);
    CHECK(after > 100);

    // The boundary itself, and the instant before it (ramp-out weight tiny but non-zero there).
    CHECK(diff_field(ov.evaluate(kSeasonAEnd, co), base.evaluate(kSeasonAEnd, cb), false).empty());
    const WeatherSample near = ov.evaluate(kSeasonAEnd - 1.0, co);
    CHECK(near.base_weight > 0.99);   // smoothstep tail: the preset's weight a second before the end is ~1e-6

    // The ramp-out: half way (w = 0.5) the blend is the arithmetic mean of base and target.
    const real t_half = t_end + 0.5 * ramp;
    const WeatherSample bh = base.evaluate(t_half, cb), sh = ov.evaluate(t_half, co);
    CHECK(std::fabs(sh.cloud_cover - (bh.cloud_cover + (0.95 - bh.cloud_cover) * 0.5)) < 1e-12);
    CHECK(std::fabs(sh.precip_mm_h - (bh.precip_mm_h + (25.0 - bh.precip_mm_h) * 0.5)) < 1e-12);
    CHECK(sh.cloud_cover != bh.cloud_cover);   // not vacuous: the preset still weighs 50 % there

    // Lightning of the override stops at the boundary: the events after it are the timeline's.
    LightningEvent eb[256], eo[256];
    const real t_late = kSeasonAEnd + 1800.0 + 120.0;
    const int nb = base.lightning_events(t_late, 1800.0, cb, eb, 256);
    const int no = ov.lightning_events(t_late, 1800.0, co, eo, 256);
    REQUIRE(nb == no);
    for (int i = 0; i < nb; ++i) CHECK((eb[i].id == eo[i].id && eb[i].t_env == eo[i].t_env));

    // The same override applied in season b ends at the END OF SEASON B (the year's end, not season a's).
    WeatherTimeline ob(wc, 777, WeatherOptions{});
    const real t_b = 40.0 * kDay;
    const int idb = ob.add_season_override(storm_preset(), ramp, t_b);
    CHECK(std::fabs(ob.find_override(idb)->t_end + ob.find_override(idb)->ramp_s - kYear) < 1e-6);
    CHECK(ob.calendar().season_end_env_time(t_b) == kYear);
    // ... and a day in the second year (season a again) ends at the end of THAT season a.
    CHECK(ob.calendar().season_end_env_time(kYear + 3.0 * kDay) == kYear + kSeasonAEnd);
}

TEST_CASE("W3b season-end override: less than two ramps left shortens the ramp, nothing leaks past the boundary",
          "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    const WeatherTimeline base(wc, 31, WeatherOptions{});
    WeatherTimeline ov(wc, 31, WeatherOptions{});
    const real t_now = kSeasonAEnd - 1000.0;
    const int id = ov.add_season_override(storm_preset(), 1800.0, t_now);
    const WeatherOverride* w = ov.find_override(id);
    REQUIRE(w != nullptr);
    CHECK(std::fabs(w->ramp_s - 500.0) < 1e-6);                 // half the remaining 1000 s
    CHECK(w->t_end > w->t_start);
    CHECK(w->t_end + w->ramp_s <= kSeasonAEnd);
    WeatherCursor cb, co;
    CHECK(diff_field(ov.evaluate(kSeasonAEnd, co), base.evaluate(kSeasonAEnd, cb), false).empty());
    // It does reach the preset at the one instant both ramps meet.
    const WeatherSample peak = ov.evaluate(t_now + 500.0, co);
    CHECK(peak.cloud_cover == 0.95);
    // Applied with zero ramp: a hard window ending exactly at the boundary.
    WeatherTimeline hard(wc, 31, WeatherOptions{});
    const int ih = hard.add_season_override(storm_preset(), 0.0, t_now);
    CHECK(hard.find_override(ih)->t_end == kSeasonAEnd);
    CHECK(hard.evaluate(kSeasonAEnd - 1e-3, co).precip_mm_h == 25.0);
    CHECK(diff_field(hard.evaluate(kSeasonAEnd, co), base.evaluate(kSeasonAEnd, cb), false).empty());
    // Rejections.
    CHECK_THROWS(ov.add_season_override(storm_preset(), -1.0, t_now));
    CHECK_THROWS(ov.add_season_override(storm_preset(), std::nan(""), t_now));
    CHECK_THROWS(ov.add_season_override(storm_preset(), 10.0, std::nan("")));
    WeatherPresetDesc bad = storm_preset();
    bad.cloud_cover = 2.0;
    CHECK_THROWS(ov.add_season_override(bad, 10.0, t_now));
}

TEST_CASE("W3b season-end override: random-access evaluate equals the sequential walk", "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    WeatherTimeline ov(wc, 4242, WeatherOptions{});
    ov.add_season_override(storm_preset(), 900.0, 20.0 * kDay);
    ov.add_season_override(storm_preset(0.3, 1.0, 0.0), 300.0, 20.5 * kDay);   // a second one, later wins
    std::vector<real> times;
    for (real t = 19.0 * kDay; t < 33.0 * kDay; t += 1777.0) times.push_back(t);
    std::vector<WeatherSample> seq;
    WeatherCursor c;
    for (real t : times) seq.push_back(ov.evaluate(t, c));
    std::uint64_t rng = 99;
    for (int k = 0; k < 150; ++k) {
        const std::size_t i = static_cast<std::size_t>(lcg(rng) % times.size());
        WeatherCursor fresh;
        const WeatherSample s = ov.evaluate(times[i], fresh);
        INFO("t = " << times[i]);
        CHECK(diff_field(s, seq[i]).empty());
    }
}

TEST_CASE("W3b season-end override: a locked season keeps the override until it is ended or removed",
          "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    WeatherOptions locked;
    locked.calendar.mode = CalendarMode::FixedSeason;
    locked.calendar.fixed_season = 0;
    const WeatherTimeline base(wc, 55, locked);
    WeatherTimeline ov(wc, 55, locked);
    CHECK(ov.calendar().season_locked());
    const real t_now = 5.0 * kDay;
    const int id = ov.add_season_override(storm_preset(), 600.0, t_now);
    CHECK(ov.find_override(id)->t_end == kInf);
    WeatherCursor cb, co;
    for (real t : {t_now + 1000.0, 40.0 * kDay, 200.0 * kDay, 1000.0 * kDay}) {
        const WeatherSample s = ov.evaluate(t, co);
        INFO("t = " << t);
        CHECK(s.source == WeatherSourceKind::Override);
        CHECK(s.cloud_cover == 0.95);
        CHECK(s.precip_mm_h == 25.0);
    }
    // Soft end: ramps out over its own ramp from the end time, then the timeline again, bit-exactly.
    const real t_stop = 300.0 * kDay;
    WeatherTimeline soft(wc, 55, locked);
    const int ids = soft.add_season_override(storm_preset(), 600.0, t_now);
    soft.end_all_overrides(t_stop);
    CHECK_FALSE(soft.override_is_season_bound(ids));
    CHECK(soft.find_override(ids)->t_end == t_stop);
    CHECK(soft.evaluate(t_stop - 1.0, co).cloud_cover == 0.95);
    const WeatherSample mid = soft.evaluate(t_stop + 300.0, co);
    CHECK(mid.cloud_cover != base.evaluate(t_stop + 300.0, cb).cloud_cover);
    CHECK(mid.cloud_cover != 0.95);
    CHECK(diff_field(soft.evaluate(t_stop + 600.0 + 100.0, co), base.evaluate(t_stop + 600.0 + 100.0, cb), false).empty());
    // An override that has not started before the end time is removed by the soft end.
    WeatherTimeline fresh(wc, 55, locked);
    fresh.add_season_override(storm_preset(), 600.0, 10.0 * kDay);
    fresh.end_all_overrides(10.0 * kDay);
    CHECK(fresh.override_count() == 0);
    // Immediate removal: the exact timeline again (and the hash).
    CHECK(ov.fold_hash(1) != base.fold_hash(1));
    CHECK(ov.remove_override(id));
    CHECK(ov.fold_hash(1) == base.fold_hash(1));
    CHECK(diff_field(ov.evaluate(t_now + 5000.0, co), base.evaluate(t_now + 5000.0, cb)).empty());
}

TEST_CASE("W3b Environment: season lock re-anchors keeping the game day and the fraction of the day",
          "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season(50400.0, 36000.0, 30000.0, 40000.0);   // a: 86400 s days, b: 70000 s days
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Dynamic;
    cfg.seed = 5;
    cfg.start = EnvironmentStart{0, "b", 10, 40000.0};
    cfg.time_scale = 60.0;
    Environment env(wc, cfg);
    const std::int64_t tick = 240 * 90;   // 90 sim s = 5400 env s later
    env.begin_tick(tick);
    const real t_old = env.clock().env_time(tick);
    const CalendarSample before = env.calendar().sample(t_old);
    const real hours_before = env.current().clock_hours;
    REQUIRE(before.season == 1);
    const std::uint64_t hash_dynamic = env.fold_state_hash(1);

    env.set_calendar_lock("a", -1, tick);   // lock ANOTHER season whose days are longer (86400 s vs 70000 s)
    CHECK(env.calendar().options().mode == CalendarMode::FixedSeason);
    CHECK(env.config().fixed_season == "a");
    const real t_new = env.clock().env_time(tick);
    const CalendarSample after = env.calendar().sample(t_new);
    CHECK(after.day_of_year == before.day_of_year);                    // game day preserved ("b" day 10) ...
    CHECK(after.season == before.season);                              // ... under its natural season label
    CHECK(after.year == before.year);
    CHECK(std::fabs(after.day_fraction - before.day_fraction) < 1e-12);   // ... and the fraction of the day
    CHECK(std::fabs(env.current().clock_hours - hours_before) < 1e-9);
    CHECK(env.current().t_env_s == t_new);
    CHECK(std::fabs(t_new - t_old) > 100.0);                           // the re-anchor really moved env time (not vacuous)
    CHECK(env.fold_state_hash(1) != hash_dynamic);                     // the lock is hashed
    // Weather season is the locked one on every day now.
    CHECK(env.calendar().day(before.day_of_year).weather_season == 0);

    // The same lock a second time is a no-op (no re-anchor rounding).
    const std::uint64_t hash_locked = env.fold_state_hash(1);
    env.set_calendar_lock("a", -1, tick);
    CHECK(env.fold_state_hash(1) == hash_locked);

    // Unlock: back to Dynamic at the same game day and fraction.
    env.set_calendar_lock("", -1, tick);
    CHECK(env.calendar().options().mode == CalendarMode::Dynamic);
    const CalendarSample back = env.calendar().sample(env.clock().env_time(tick));
    CHECK(back.day_of_year == before.day_of_year);
    CHECK(std::fabs(back.day_fraction - before.day_fraction) < 1e-12);

    // Bad requests throw before changing anything.
    const std::uint64_t h = env.fold_state_hash(1);
    CHECK_THROWS_AS(env.set_calendar_lock("monsoon", -1, tick), std::invalid_argument);
    CHECK_THROWS_AS(env.set_calendar_lock("a", 3, tick), std::invalid_argument);   // season and day together
    CHECK_THROWS_AS(env.set_calendar_lock("", 100000, tick), std::invalid_argument);
    CHECK(env.fold_state_hash(1) == h);
    CHECK(env.calendar().options().mode == CalendarMode::Dynamic);

    // Pinning one game day works too and is the third mode.
    env.set_calendar_lock("", 40, tick);
    CHECK(env.calendar().options().mode == CalendarMode::FixedDay);
    CHECK(env.config().fixed_day_of_year == 40);
}

TEST_CASE("W3b Environment: the chain is re-derived under the new calendar (cursor invalidated)", "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Dynamic;
    cfg.seed = 8;
    cfg.time_scale = 30.0;
    Environment a(wc, cfg);
    // Walk the cursor far along the Dynamic chain first (so a stale ring would be wrong later).
    for (real t = 0.0; t < 50.0 * kDay; t += 4000.0) (void)a.evaluate(t);
    a.set_calendar_lock("b", -1, 0);
    EnvironmentConfig locked_cfg = cfg;
    locked_cfg.fixed_season = "b";
    const Environment fresh(wc, locked_cfg);
    // The weather timelines of the two are the same pure function: compare bit-exactly over a range
    // that includes times the old (Dynamic) walk had already cached.
    WeatherCursor ca, cf;
    for (real t = 0.0; t < 60.0 * kDay; t += 5003.0) {
        const WeatherSample sa = a.weather()->evaluate(t, ca);
        const WeatherSample sf = fresh.weather()->evaluate(t, cf);
        INFO("t = " << t);
        REQUIRE(diff_field(sa, sf).empty());
    }
    // Season b's draws differ from the Dynamic walk's (season a first): the guard is not vacuous.
    const Environment dyn(wc, cfg);
    WeatherCursor cd;
    int differ = 0;
    for (real t = 0.0; t < 20.0 * kDay; t += 5003.0)
        if (!diff_field(dyn.weather()->evaluate(t, cd), fresh.weather()->evaluate(t, cf), false).empty()) ++differ;
    CHECK(differ > 10);
}

TEST_CASE("W3b Environment: manual weather lasts to the season end, a lock makes it permanent, unlocking re-resolves it",
          "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Dynamic;
    cfg.seed = 21;
    cfg.start = EnvironmentStart{0, "a", 12, 20000.0};
    cfg.time_scale = 120.0;
    Environment env(wc, cfg);
    const Environment base(wc, cfg);
    const std::int64_t tick = 240 * 30;   // 3600 env s after the start
    env.begin_tick(tick);
    const real t_now = env.clock().env_time(tick);
    const std::uint64_t h0 = env.fold_state_hash(1);

    const int id = env.add_season_weather_override(storm_preset(), 600.0, tick);
    const WeatherOverride* w = env.weather()->find_override(id);
    REQUIRE(w != nullptr);
    CHECK(w->t_start == t_now);
    CHECK(env.fold_state_hash(1) != h0);     // the override list is description state: hashed
    // 10 minutes after the start the preset is fully in; after the season end the timeline is back.
    CHECK(env.evaluate(t_now + 600.0).precip_mm_h == 25.0);
    CHECK(env.evaluate(t_now + 600.0).wind_mean_m_s == 12.0);
    CHECK(env.evaluate(t_now).cloud_cover == base.evaluate(t_now).cloud_cover);   // weight 0 at its own start
    CHECK(env.evaluate(kSeasonAEnd + 10.0).precip_mm_h == base.evaluate(kSeasonAEnd + 10.0).precip_mm_h);
    CHECK(env.evaluate(kSeasonAEnd + 10.0).wind_mean_m_s == base.evaluate(kSeasonAEnd + 10.0).wind_mean_m_s);

    // Lock season b 20 sim s later (2400 env s elapsed): the override becomes open-ended, elapsed time kept.
    const std::int64_t tick2 = tick + 240 * 20;
    const real elapsed = env.clock().env_time(tick2) - w->t_start;
    CHECK(std::fabs(elapsed - 2400.0) < 1e-6);
    env.set_calendar_lock("b", -1, tick2);
    const WeatherOverride* wl = env.weather()->find_override(id);
    REQUIRE(wl != nullptr);
    CHECK(wl->t_end == kInf);
    CHECK(env.weather()->override_is_season_bound(id));
    CHECK(std::fabs((env.clock().env_time(tick2) - wl->t_start) - elapsed) < 1e-6);
    // Far in the future it still holds (a locked season never ends).
    CHECK(env.evaluate(env.clock().env_time(tick2) + 300.0 * kDay).precip_mm_h == 25.0);

    // Unlock: the end is re-resolved in the new (Dynamic) calendar.
    env.set_calendar_lock("", -1, tick2);
    wl = env.weather()->find_override(id);
    REQUIRE(wl != nullptr);
    CHECK(env.calendar().options().mode == CalendarMode::Dynamic);
    const real end_now = env.calendar().season_end_env_time(env.clock().env_time(tick2));
    CHECK(wl->t_end < kInf);
    CHECK(wl->t_end + wl->ramp_s <= end_now);
    CHECK(end_now - (wl->t_end + wl->ramp_s) < 1e-6);
    CHECK(std::fabs((env.clock().env_time(tick2) - wl->t_start) - elapsed) < 1e-6);

    // Soft end now: stops, then the timeline again after the ramp-out (its own 600 s).
    env.end_weather_overrides(tick2);
    CHECK_FALSE(env.weather()->override_is_season_bound(id));
    const real t2 = env.clock().env_time(tick2);
    CHECK(env.evaluate(t2 - 1.0).precip_mm_h == 25.0);
    CHECK(env.evaluate(t2 + 300.0).cloud_cover != base.evaluate(t2 + 300.0).cloud_cover);   // mid ramp-out
    CHECK(env.evaluate(t2 + 700.0).cloud_cover == base.evaluate(t2 + 700.0).cloud_cover);
    CHECK(env.evaluate(t2 + 700.0).precip_mm_h == base.evaluate(t2 + 700.0).precip_mm_h);
    // Immediate clear.
    env.clear_weather_overrides(tick2);
    CHECK(env.weather()->override_count() == 0);
    CHECK(env.evaluate(t2 - 1.0).precip_mm_h == base.evaluate(t2 - 1.0).precip_mm_h);

    // remove_weather_override by id and an unknown id.
    const int id2 = env.add_season_weather_override(storm_preset(), 0.0, tick2);
    CHECK(env.remove_weather_override(id2, tick2));
    CHECK_FALSE(env.remove_weather_override(id2, tick2));

    // Without a weather timeline there is nothing to override.
    Environment plain(wc);
    CHECK_THROWS_AS(plain.add_season_weather_override(storm_preset(), 10.0, 0), std::logic_error);
    CHECK_THROWS_AS(plain.clear_weather_overrides(0), std::logic_error);
    CHECK_THROWS_AS(plain.end_weather_overrides(0), std::logic_error);
    CHECK_NOTHROW(plain.set_calendar_lock("b", -1, 0));   // the calendar always exists
}

TEST_CASE("W3b overrides: the 16-override capacity holds for season-bound ones too", "[environment][weather]") {
    const ClimateDesc wc = test_env::weather_two_season();
    WeatherTimeline tl(wc, 1, WeatherOptions{});
    for (int i = 0; i < WeatherTimeline::kMaxOverrides; ++i) tl.add_season_override(storm_preset(), 10.0, 100.0 + i);
    CHECK_THROWS_AS(tl.add_season_override(storm_preset(), 10.0, 200.0), std::length_error);
    CHECK_THROWS_AS(tl.add_override(WeatherOverride{300.0, 400.0, 0.0, storm_preset()}), std::length_error);
}
