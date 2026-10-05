// tests/unit/test_world_weather_override.cpp - weather plan W3b (owner answer Q12): a manual
// weather preset replaces the weather until the END OF THE CURRENT SEASON and reaches the
// vehicles through the ordinary per-vehicle air path (freeze_tick_start_state -> VehicleTickEnv
// -> VehicleAir); a season lock makes it last until ended; the World calls throw without an
// environment. The library-level override machinery (ramp location, locked calendars, runtime
// calendar switch, hash, random access == sequential) is tested without ps_core in
// test_environment_season_override.cpp. Tags [environment][world][vehicle].
//
// RECORDED SABOTAGES (performed by hand, reverted; numbers in out/weather_w3b_progress.md):
//  - W3b-W1 (override never ends: WeatherTimeline::resolve_season_bound returning t_end = +inf for
//    an unlocked calendar): "the override ends with the season" failed - the vehicle still felt the
//    22 m/s storm after the boundary.
//  - W3b-W2 (the override ignored by the world: World::add_weather_override_to_season_end a no-op):
//    "the override reaches the vehicle's air" failed (mean wind stayed at the natural value).
//  - W3b-W3 (lock without re-anchoring the override: Environment::set_calendar_lock leaving the
//    override's t_end alone): "a locked season keeps the override" failed after the old boundary.
//  - W3b-W4 (allocation: a std::vector<int>(64) in WeatherCursor::ensure's re-init path): the
//    allocation case failed.

#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/environment/weather_io.h"
#include "ps/io/scenario.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace ps;
using Catch::Matchers::WithinAbs;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

std::string data(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

environment::ClimateDesc gust_climate() { return io::load_climate_json(data("climates/test_weather_gusts.json")); }

environment::WeatherPresetDesc storm() { return io::load_weather_preset_json(data("weather_presets/storm.json")); }

// Dynamic weather, dynamic calendar, starting 6400 env s before the end of summer (day 30 of 0..30).
environment::EnvironmentConfig season_end_config(real scale) {
    environment::EnvironmentConfig c;
    c.weather = environment::EnvironmentWeather::Dynamic;
    c.seed = 7;
    c.start = environment::EnvironmentStart{0, "summer", 30, 80000.0};
    c.time_scale = scale;
    return c;
}

std::shared_ptr<environment::Environment> make_env(const environment::EnvironmentConfig& c) {
    return std::make_shared<environment::Environment>(gust_climate(), c);
}

ps_test::VehicleTestRig sim_rig(int workers) {
    return ps_test::build_vehicle_rig(data("vehicles/car_sedan_sim.json"), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
}

real horizontal_mean_wind(const environment::VehicleAir& a) {
    return std::hypot(a.mean_wind_world.x, a.mean_wind_world.y);
}

} // namespace

TEST_CASE("World weather override: the override reaches the vehicle's air and ends with the season",
          "[environment][world][vehicle]") {
    // Reference: the same environment without an override, evaluated at the world's clock.
    const environment::EnvironmentConfig cfg = season_end_config(1000.0);   // 6400 env s = 6.4 sim s to the boundary
    auto ref = make_env(cfg);
    ref->attach(0, 240.0);

    ps_test::VehicleTestRig rig = sim_rig(1);
    World& w = *rig.world;
    w.install_environment(make_env(cfg));
    ps_test::settle(w, 0.5);
    const real natural_speed = horizontal_mean_wind(w.vehicle_air(rig.vehicle));
    REQUIRE(w.vehicle_air(rig.vehicle).valid);
    CHECK(natural_speed < 15.0);   // the natural summer regimes are nowhere near a 22 m/s storm

    const int id = w.add_weather_override_to_season_end(storm(), 1000.0);   // ramp 1000 env s = 1 sim s
    CHECK(id >= 0);
    ps_test::settle(w, 1.5);   // t = 2.0 s: the ramp-in (to 1.5 s) is done, 4.4 s before the boundary
    {
        const environment::VehicleAir a = w.vehicle_air(rig.vehicle);
        CHECK_THAT(horizontal_mean_wind(a), WithinAbs(22.0, 1e-6));
        const environment::EnvironmentSnapshot& e = w.snapshots().read().environment;
        CHECK(e.has_weather);
        CHECK(e.wind_mean_m_s == 22.0);
        CHECK(e.wind_dir_deg == 250.0);
        CHECK(e.precip_mm_h == 3.0);
        // The natural weather at that instant is different (the override is what moved it).
        const environment::EnvironmentSnapshot nat = ref->evaluate(w.environment()->clock().env_time(w.tick()));
        CHECK(nat.wind_mean_m_s != 22.0);
    }

    ps_test::settle(w, 5.5);   // t = 7.5 s: 1.1 s (1100 env s) past the boundary; the ramp-out ended AT the boundary
    {
        const environment::EnvironmentSnapshot& e = w.snapshots().read().environment;
        const real t_env = e.t_env_s;   // the snapshot's own instant (published by the last step)
        CHECK(e.season == 2);   // autumn
        const environment::EnvironmentSnapshot nat = ref->evaluate(t_env);
        INFO("t_env " << t_env);
        CHECK(e.wind_mean_m_s == nat.wind_mean_m_s);
        CHECK(e.wind_dir_deg == nat.wind_dir_deg);
        CHECK(e.precip_mm_h == nat.precip_mm_h);
        CHECK(e.cloud_cover == nat.cloud_cover);
        CHECK(e.pressure_msl_hpa == nat.pressure_msl_hpa);
        // ... and so is the vehicle's air: the storm is gone from it.
        CHECK_THAT(horizontal_mean_wind(w.vehicle_air(rig.vehicle)), WithinAbs(nat.wind_mean_m_s, 1e-6));
        CHECK(w.environment()->weather()->override_count() == 1);   // finished, not removed: only clear/remove drops it
    }
}

TEST_CASE("World weather override: a locked season keeps the override, unlocking restores the season end",
          "[environment][world][vehicle]") {
    const environment::EnvironmentConfig cfg = season_end_config(1000.0);
    ps_test::VehicleTestRig rig = sim_rig(1);
    World& w = *rig.world;
    w.install_environment(make_env(cfg));
    ps_test::settle(w, 0.25);
    const real dynamic_end = w.environment()->calendar().season_end_env_time(w.environment()->clock().env_time(w.tick()));
    w.set_environment_calendar_lock("summer");   // the day length stays the same (one cycle_s for both modes here)
    const int oid = w.add_weather_override_to_season_end(storm(), 500.0);
    ps_test::settle(w, 8.0);   // 8000 env s: well past the dynamic season's end (6150 s after the lock)
    CHECK(w.environment()->clock().env_time(w.tick()) > dynamic_end + 1000.0);
    CHECK_THAT(horizontal_mean_wind(w.vehicle_air(rig.vehicle)), WithinAbs(22.0, 1e-6));

    // Unlock: the calendar is dynamic again; the override's end is re-resolved against the real season end.
    w.set_environment_calendar_lock("");
    ps_test::settle(w, 0.5);
    const environment::EnvironmentSnapshot& e = w.snapshots().read().environment;
    INFO("season " << e.season << " t_env " << e.t_env_s);
    // The calendar is dynamic: the clock is in whatever season the re-anchored day number falls in; the
    // override is in force only if that season is the one it was resolved for.
    const environment::WeatherOverride* ov = w.environment()->weather()->find_override(oid);
    REQUIRE(ov != nullptr);
    CHECK(ov->t_end < 1e300);   // finite again
}

TEST_CASE("World weather override: calls without an environment or without weather", "[environment][world]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    CHECK_THROWS_AS(world.add_weather_override_to_season_end(storm(), 10.0), std::logic_error);
    CHECK_THROWS_AS(world.end_weather_overrides(), std::logic_error);
    CHECK_THROWS_AS(world.clear_weather_overrides(), std::logic_error);
    CHECK_THROWS_AS(world.remove_weather_override(0), std::logic_error);
    CHECK_THROWS_AS(world.set_environment_calendar_lock("summer"), std::logic_error);

    // An environment without a weather timeline: the weather calls throw, the lock works.
    environment::EnvironmentConfig plain;
    plain.fixed_season = "summer";
    world.install_environment(make_env(plain));
    CHECK_THROWS_AS(world.add_weather_override_to_season_end(storm(), 10.0), std::logic_error);
    CHECK_THROWS_AS(world.end_weather_overrides(), std::logic_error);
    CHECK_THROWS_AS(world.clear_weather_overrides(), std::logic_error);
    world.step();
    const real summer_day_s = world.snapshots().read().environment.day_s;
    CHECK_NOTHROW(world.set_environment_calendar_lock("winter"));
    world.step();
    // A locked calendar's days carry the locked season's values (the date's own season index keeps counting).
    CHECK(world.snapshots().read().environment.day_s < summer_day_s - 3600.0);   // a winter day at 50 N is much shorter
    CHECK_THROWS_AS(world.set_environment_calendar_lock("monsoon"), std::invalid_argument);
}

TEST_CASE("World weather override: 1 vs 10 workers give the same state hash across the season boundary",
          "[environment][world][vehicle][determinism]") {
    const auto run = [](int workers) {
        ps_test::VehicleTestRig rig = sim_rig(workers);
        rig.world->install_environment(make_env(season_end_config(3000.0)));   // boundary at 2.13 s = tick 512
        rig.world->set_control("throttle", 0.4);
        std::uint64_t h = 0;
        for (int k = 0; k < 640; ++k) {
            if (k == 120) rig.world->add_weather_override_to_season_end(storm(), 600.0);
            if (k == 400) rig.world->set_environment_calendar_lock("");   // no-op re-anchor: same mode
            rig.world->step();
            h = h * 1099511628211ull ^ rig.world->state_hash();
        }
        return h;
    };
    const std::uint64_t a = run(1);
    CHECK(run(10) == a);
    CHECK(run(1) == a);

    // The override is hashed state: the same run without it differs.
    ps_test::VehicleTestRig rig = sim_rig(1);
    rig.world->install_environment(make_env(season_end_config(3000.0)));
    rig.world->set_control("throttle", 0.4);
    std::uint64_t h = 0;
    for (int k = 0; k < 640; ++k) {
        rig.world->step();
        h = h * 1099511628211ull ^ rig.world->state_hash();
    }
    CHECK(h != a);
}

TEST_CASE("World weather override: an active override across the season boundary allocates nothing inside the tick",
          "[environment][world][vehicle][debug_alloc][jobs]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    for (const int workers : {1, 4}) {
        ps_test::VehicleTestRig rig = sim_rig(workers);
        World& w = *rig.world;
        w.install_environment(make_env(season_end_config(3000.0)));   // boundary at 2.133 s
        w.set_control("throttle", 0.3);
        ps_test::settle(w, 1.5);
        w.add_weather_override_to_season_end(storm(), 600.0);
        ps_test::settle(w, 0.1);   // warm-up with the override in force (snapshot capacity, telemetry, buffers)
        jobs::debug_alloc::reset();
        jobs::debug_alloc::set_armed(true);
        for (int k = 0; k < 240; ++k) w.step();   // 1.6 s .. 2.6 s: crosses the boundary
        jobs::debug_alloc::set_armed(false);
        INFO("workers " << workers);
        CHECK(jobs::debug_alloc::own_violation_count() == 0);
        CHECK(w.environment()->current().season == 2);   // the boundary really was crossed inside the window
    }
}

namespace {

struct TempScenarioFile {
    std::filesystem::path path;
    explicit TempScenarioFile(const std::string& json, const char* tag) {
        path = std::filesystem::temp_directory_path() / (std::string("ps_env_w3b_") + tag + ".json");
        std::ofstream(path) << json;
    }
    ~TempScenarioFile() { std::error_code ec; std::filesystem::remove(path, ec); }
};

} // namespace

TEST_CASE("World weather override: scenario events apply at their sim time", "[environment][io][world]") {
    const std::string storm_path = data("weather_presets/storm.json");
    // 6400 env s before the end of summer at scale 1000: the season would end at sim 6.4 s (never reached here).
    const std::string json = R"({
  "format": "physics_sim.scenario/1", "name": "w3b_events", "duration_s": 5.0,
  "bodies": [ {"name": "ground", "motion": "static", "shape": {"type": "box", "half_extents": [10, 10, 0.5]}, "position": [0, 0, -0.5]} ],
  "environment": {
    "climate": ")" + data("climates/test_weather.json") + R"(", "weather": "dynamic", "seed": 7,
    "time": { "start": { "season": "summer", "day": 30, "time_of_day_s": 80000 }, "scale": 1000 },
    "events": [
      { "t": 0.5, "set_weather": { "preset": ")" + storm_path + R"(", "ramp_s": 500 } },
      { "t": 3.0, "set_season": "summer" },
      { "t": 4.0, "end_weather": true },
      { "t": 4.5, "clear_weather": true } ] }
})";
    TempScenarioFile f(json, "events");
    const io::Scenario sc = io::load_scenario(f.path.string());
    REQUIRE(sc.environment->events.size() == 4);
    CHECK(sc.environment->events[0].kind == io::ScenarioEnvironmentEvent::Kind::SetWeather);
    CHECK(sc.environment->events[0].value == 500.0);
    CHECK(sc.environment->events[0].preset.wind_mean_m_s == 22.0);
    CHECK(sc.environment->events[1].text == "summer");
    io::BuiltScenario built = io::build_scenario(sc);
    World& world = *built.world;
    REQUIRE(world.environment() != nullptr);
    const environment::WeatherTimeline* tl = world.environment()->weather();
    REQUIRE(tl != nullptr);

    std::size_t next = 0;
    std::size_t count_at[5] = {};
    real wind_at_2s = 0.0;
    std::string season_at_3_5;
    for (int tick = 0; tick < 240 * 5; ++tick) {
        io::apply_environment_events(world, sc, next);
        world.step();
        if (tick == 0) count_at[0] = tl->override_count();
        if (tick == 240 * 2) { count_at[1] = tl->override_count(); wind_at_2s = world.snapshots().read().environment.wind_mean_m_s; }
        if (tick == 240 * 3 + 120) { count_at[2] = tl->override_count(); season_at_3_5 = world.snapshots().read().environment.season_name; }
        if (tick == 240 * 4 + 60) count_at[3] = tl->override_count();   // soft end at 4.0 s: still listed
        if (tick == 240 * 5 - 1) count_at[4] = tl->override_count();    // cleared at 4.5 s
    }
    CHECK(next == 4);
    CHECK(count_at[0] == 0);
    CHECK(count_at[1] == 1);
    CHECK(count_at[2] == 1);
    CHECK(count_at[3] == 1);
    CHECK(count_at[4] == 0);
    CHECK(wind_at_2s == 22.0);   // ramp-in (500 env s = 0.5 sim s) done long before
    CHECK(season_at_3_5 == "summer");
}
