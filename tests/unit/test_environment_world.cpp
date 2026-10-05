// tests/unit/test_environment_world.cpp - weather plan W2: the Environment
// installed into a World (freeze_tick_start_state stage, WorldSnapshot::
// environment, state hash, scenario "environment" block, ps_run event helper).
// The library object itself is tested without ps_core in
// test_environment_object.cpp. Tags [environment][world][io].
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - W2-W1 (stale snapshot): World::stage_freeze_tick_start_state without the
//    begin_tick call: "the published snapshot equals evaluate() of the clock at
//    its tick" failed from the first step on (t_env stuck at the install value) and
//    so did the allocation case's "environment really ran" progress check (0 s).
//  - W2-W2 (hash when absent): World::state_hash folding a default Environment
//    hash unconditionally: "no environment: hash, channels and snapshot are
//    untouched" failed on the install/remove round trip (the plain world's
//    hash no longer matched; "the clock anchor is hashed" failed too).
//    Measured via a World-level fold only; any such unconditional fold would
//    also move every pinned scenario hash gate.
//  - W2-W3 (allocation): an `std::vector<int>(64)` constructed inside
//    World::stage_freeze_tick_start_state: "an installed environment allocates
//    nothing inside the tick" counted 240 own violations (1 per step, 240 steps)
//    at 1 and at 4 workers and failed.
//  - W2-W4 (event order): apply_environment_events applying jump events before
//    checking `time_s <= sim_time` (i.e. ignoring the time): "scenario events
//    apply at their sim time" failed on the pre-event clock check.

#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/io/scenario.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace ps;
using Catch::Matchers::ContainsSubstring;

namespace {

std::string source_dir() { return std::string(PS_SOURCE_DIR); }

environment::ClimateDesc test_climate() {
    return io::load_climate_json(source_dir() + "/data/climates/test_fixed_summer.json");
}

// A static ground plus one dynamic ball: a World with something for Jolt to do.
void add_scene(World& world) {
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{100.0, 100.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    world.create_body(ground);
    BodyDesc ball;
    ball.shape = SphereShape{0.5};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{0.0, 0.0, 3.0};
    ball.allow_sleep = false;
    world.create_body(ball);
}

WorldConfig config(int workers = 1) {
    WorldConfig c;
    c.job_workers = workers;
    return c;
}

std::shared_ptr<environment::Environment> make_env(real scale, const char* season = "summer", int day = 0, real tod = 21600.0) {
    environment::EnvironmentConfig cfg;
    cfg.time_scale = scale;
    cfg.start = environment::EnvironmentStart{0, season, day, tod};
    return std::make_shared<environment::Environment>(test_climate(), cfg);
}

// A throwaway scenario file under the system temp directory.
struct TempScenario {
    std::filesystem::path path;
    explicit TempScenario(const std::string& json, const char* tag) {
        path = std::filesystem::temp_directory_path() / (std::string("ps_env_w2_") + tag + ".json");
        std::ofstream(path) << json;
    }
    ~TempScenario() { std::error_code ec; std::filesystem::remove(path, ec); }
};

std::string scenario_json(const std::string& environment_block) {
    return R"({
  "format": "physics_sim.scenario/1", "name": "env_tmp", "duration_s": 2.0,
  "bodies": [ {"name": "ground", "motion": "static", "shape": {"type": "box", "half_extents": [10, 10, 0.5]}, "position": [0, 0, -0.5]} ],
  "environment": )" + environment_block + "\n}\n";
}

const std::string kClimate = "\"climate\": \"" + std::string(PS_SOURCE_DIR) + "/data/climates/test_fixed_summer.json\"";

} // namespace

TEST_CASE("World environment: the published snapshot equals evaluate() of the clock at its tick", "[environment][world]") {
    World world(config());
    add_scene(world);
    for (int i = 0; i < 5; ++i) world.step();
    REQUIRE(world.environment() == nullptr);
    REQUIRE(world.environment_snapshot() == nullptr);

    // A mid-run install anchors "now": the next tick runs at the start time.
    auto env = make_env(60.0);
    world.install_environment(env);
    REQUIRE(world.environment() == env.get());
    REQUIRE(world.environment_snapshot() != nullptr);
    CHECK(world.environment_snapshot()->t_env_s == env->start_env_s());
    CHECK(env->clock().anchor_tick == 5);

    for (int i = 0; i < 40; ++i) {
        world.step();
        const WorldSnapshot& snap = world.snapshots().read();
        REQUIRE(snap.has_environment);
        const std::int64_t tick = static_cast<std::int64_t>(snap.tick);
        const real expect_t = env->start_env_s() + 60.0 * (static_cast<real>(tick - 5) / 240.0);
        INFO("tick " << tick);
        CHECK(snap.environment.t_env_s == expect_t);
        const environment::EnvironmentSnapshot ref = env->evaluate(env->clock().env_time(tick));
        CHECK(snap.environment.sun_elevation_rad == ref.sun_elevation_rad);
        CHECK(snap.environment.sun_dir_world.x == ref.sun_dir_world.x);
        CHECK(snap.environment.sun_dir_world.y == ref.sun_dir_world.y);
        CHECK(snap.environment.sun_dir_world.z == ref.sun_dir_world.z);
        CHECK(snap.environment.time_of_day_s == ref.time_of_day_s);
        CHECK(snap.environment.ghi_w_m2 == ref.ghi_w_m2);
    }
    // The sun moves: 40 ticks at scale 60 = 10 env seconds, a measurable but tiny step.
    CHECK(world.environment_snapshot()->t_env_s > env->start_env_s());
}

TEST_CASE("World environment: no environment means hash, channels and snapshot are untouched", "[environment][world]") {
    const auto run = [](bool install_then_remove, bool keep_installed) {
        World world(config());
        add_scene(world);
        if (install_then_remove || keep_installed) world.install_environment(make_env(1.0));
        for (int i = 0; i < 20; ++i) world.step();
        if (install_then_remove && !keep_installed) world.remove_environment();
        world.step(); // the snapshot published by this step reflects the removal
        struct R { std::uint64_t hash; std::size_t channels; bool has_env; };
        return R{world.state_hash(), world.telemetry().channel_names().size(), world.snapshots().read().has_environment};
    };
    const auto plain = run(false, false);
    CHECK_FALSE(plain.has_env);
    const auto kept = run(false, true);
    const auto removed = run(true, false);
    CHECK(kept.has_env);
    // An installed environment folds its clock anchor into the hash ...
    CHECK(kept.hash != plain.hash);
    // ... adds no telemetry channel (no CSV header change) ...
    CHECK(kept.channels == plain.channels);
    // ... and removing it restores the exact hash of a world that never had one (it does not
    // influence the physics in W2, so only its own fold may differ).
    CHECK(removed.hash == plain.hash);
    CHECK_FALSE(removed.has_env);

    World world(config());
    CHECK_THROWS_AS(world.install_environment(nullptr), std::invalid_argument);
    CHECK_THROWS_AS(world.set_environment_time(0.0), std::logic_error);
    CHECK_THROWS_AS(world.set_environment_time_scale(1.0), std::logic_error);
}

TEST_CASE("World environment: the clock anchor is hashed, ticking is not", "[environment][world]") {
    const auto hash_after = [](real scale, bool jump) {
        World world(config());
        add_scene(world);
        world.install_environment(make_env(scale));
        for (int i = 0; i < 10; ++i) world.step();
        if (jump) world.set_environment_time(1.0e6);
        return world.state_hash();
    };
    const std::uint64_t base = hash_after(1.0, false);
    CHECK(hash_after(1.0, false) == base);
    CHECK(hash_after(30.0, false) != base);
    CHECK(hash_after(1.0, true) != base);

    // Same physics, different environment time: only the environment fold differs, and a
    // frozen clock hashes like any other anchor.
    World a(config());
    add_scene(a);
    a.install_environment(make_env(0.0));
    for (int i = 0; i < 10; ++i) a.step();
    const real t0 = a.environment_snapshot()->t_env_s;
    for (int i = 0; i < 100; ++i) a.step();
    CHECK(a.environment_snapshot()->t_env_s == t0); // scale 0 freezes env time
    CHECK(a.environment_snapshot()->sun_elevation_rad == a.environment()->evaluate(t0).sun_elevation_rad);
}

TEST_CASE("World environment: jump and scale change take effect at the next step", "[environment][world]") {
    World world(config());
    add_scene(world);
    auto env = make_env(10.0);
    world.install_environment(env);
    for (int i = 0; i < 24; ++i) world.step();   // tick 24 is next
    const real expected_now = env->clock().env_time(24);
    world.set_environment_time(1.0e6);
    CHECK(world.environment_snapshot()->t_env_s == 1.0e6);
    world.step();
    CHECK(world.snapshots().read().environment.t_env_s == 1.0e6);
    world.set_environment_time_scale(0.0);       // continuous: tick 25's own time stays
    const real t25 = env->clock().env_time(25);
    CHECK_THAT(t25, Catch::Matchers::WithinAbs(1.0e6 + 10.0 / 240.0, 1e-9));
    for (int i = 0; i < 10; ++i) world.step();
    CHECK(world.snapshots().read().environment.t_env_s == t25);
    (void)expected_now;
}

TEST_CASE("World environment: an installed environment allocates nothing inside the tick", "[environment][world][debug_alloc][jobs]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    for (int workers : {1, 4}) {
        World world(config(workers));
        add_scene(world);
        world.install_environment(make_env(5000.0));
        // Warm-up: the ball lands and settles (contact events, Jolt, snapshot slot
        // capacity and telemetry all reach their steady size).
        for (int i = 0; i < 360; ++i) world.step();
        const real t_before = world.environment_snapshot()->t_env_s;
        jobs::debug_alloc::reset();
        jobs::debug_alloc::set_armed(true);
        for (int i = 0; i < 240; ++i) world.step();
        jobs::debug_alloc::set_armed(false);
        INFO("workers " << workers);
        CHECK(jobs::debug_alloc::own_violation_count() == 0);
        // The environment really ran inside that window: 240 ticks at x5000 = 5000 env s.
        CHECK(world.environment_snapshot()->t_env_s - t_before > 4900.0);
        jobs::debug_alloc::reset();
    }
}

TEST_CASE("Scenario environment block: parsing", "[environment][io]") {
    const io::Scenario sc = io::load_scenario(source_dir() + "/data/scenarios/environment_day.json");
    REQUIRE(sc.environment.has_value());
    const io::ScenarioEnvironment& e = *sc.environment;
    CHECK(e.seed == 1);
    CHECK(e.config.time_scale == 20000.0);
    REQUIRE(e.config.start.has_value());
    CHECK(e.config.start->season == "autumn");
    CHECK(e.config.start->day == 29);
    CHECK(e.config.start->time_of_day_s == 60000.0);
    CHECK(e.config.fixed_season.empty()); // "season": "dynamic"
    CHECK(e.config.north_yaw_deg == 15.0);
    REQUIRE(e.events.size() == 2);
    CHECK(e.events[0].kind == io::ScenarioEnvironmentEvent::Kind::JumpTime);
    CHECK(e.events[0].time_s == 3.0);
    CHECK(e.events[0].value == 40000.0);
    CHECK(e.events[1].kind == io::ScenarioEnvironmentEvent::Kind::SetTimeScale);
    CHECK(e.climate.seasons.size() == 4);

    // A scenario without the block has none and builds a world without one.
    const io::Scenario plain = io::load_scenario(source_dir() + "/data/scenarios/vehicle_step_steer.json");
    CHECK_FALSE(plain.environment.has_value());
}

TEST_CASE("Scenario environment block: rejections", "[environment][io]") {
    const auto load_msg = [](const std::string& block, const char* tag) -> std::string {
        TempScenario f(scenario_json(block), tag);
        try {
            io::load_scenario(f.path.string());
        } catch (const std::runtime_error& e) {
            return e.what();
        }
        return {};
    };
    // Sanity: the harness accepts a minimal valid block.
    CHECK(load_msg("{ " + kClimate + " }", "ok").empty());

    CHECK_THAT(load_msg("{ " + kClimate + ", \"bogus\": 1 }", "unk"), ContainsSubstring("unknown key \"bogus\""));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"weather\": \"dynamic\" }", "weather"), ContainsSubstring("the climate needs precip_phase"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"weather\": \"sunny\" }", "wbad"), ContainsSubstring("\"weather\" must be"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"weather\": { \"bogus\": 1 } }", "wk"), ContainsSubstring("weather.bogus"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"weather\": { \"preset\": \"does/not/exist.json\" } }", "wp"), ContainsSubstring("exist.json"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"surface_conditions\": \"x\" }", "sc"), ContainsSubstring("not supported yet"));
    CHECK_THAT(load_msg("{ \"seed\": 1 }", "noclimate"), ContainsSubstring("missing required string \"climate\""));
    CHECK_THAT(load_msg("{ \"climate\": \"does/not/exist.json\" }", "nofile"), ContainsSubstring("exist.json"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"time\": { \"scale\": -1 } }", "neg"), ContainsSubstring("time_scale"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"time\": { \"nope\": 1 } }", "tk"), ContainsSubstring("time.nope"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"time\": { \"start\": { \"season\": \"monsoon\" } } }", "ss"), ContainsSubstring("monsoon"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"time\": { \"start\": { \"zzz\": 1 } } }", "sk"), ContainsSubstring("time.start.zzz"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"season\": \"monsoon\" }", "season"), ContainsSubstring("monsoon"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"season\": \"summer\", \"fixed_day_of_year\": 3 }", "both"), ContainsSubstring("mutually exclusive"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"events\": [ { \"t\": 1 } ] }", "e0"), ContainsSubstring("exactly one"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"events\": [ { \"t\": 1, \"jump_time_s\": 5, \"set_time_scale\": 2 } ] }", "e2"), ContainsSubstring("exactly one"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"events\": [ { \"t\": 1, \"set_weather\": {} } ] }", "ew"), ContainsSubstring("not supported yet"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"events\": [ { \"t\": 1, \"bogus\": 5 } ] }", "eb"), ContainsSubstring("unknown event key"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"events\": [ { \"t\": 1, \"set_time_scale\": -2 } ] }", "es"), ContainsSubstring("set_time_scale"));
    CHECK_THAT(load_msg("{ " + kClimate + ", \"events\": [ { \"jump_time_s\": 5 } ] }", "et"), ContainsSubstring("\"t\""));
}

TEST_CASE("Scenario environment block: events apply at their sim time", "[environment][io][world]") {
    const std::string block = "{ " + kClimate +
                              ", \"time\": { \"start\": { \"season\": \"summer\", \"day\": 5, \"time_of_day_s\": 1000 }, \"scale\": 100 },"
                              " \"events\": [ { \"t\": 1.0, \"set_time_scale\": 0 }, { \"t\": 0.5, \"jump_time_s\": 5000 } ] }";
    TempScenario f(scenario_json(block), "events");
    const io::Scenario sc = io::load_scenario(f.path.string());
    REQUIRE(sc.environment->events.size() == 2);
    CHECK(sc.environment->events[0].time_s == 0.5); // sorted by time
    io::BuiltScenario built = io::build_scenario(sc);
    World& world = *built.world;
    REQUIRE(world.environment() != nullptr);
    const real start = world.environment()->start_env_s();

    std::size_t next = 0;
    int jump_tick = -1, freeze_tick = -1;
    real t_before_jump = 0.0, t_before_freeze = 0.0;
    for (int tick = 0; tick < 300; ++tick) {
        const std::size_t before = next;
        const real t_next = world.environment()->clock().env_time(world.tick());
        const std::size_t n = io::apply_environment_events(world, sc, next);
        CHECK(n == next - before);
        if (n > 0) {
            if (jump_tick < 0) { jump_tick = tick; t_before_jump = t_next; }
            else { freeze_tick = tick; t_before_freeze = world.environment()->clock().env_time(world.tick()); }
        }
        world.step();
    }
    REQUIRE(jump_tick >= 0);
    REQUIRE(freeze_tick > jump_tick);
    CHECK(next == 2);
    // Neither event fires before its time (sim_time = tick / 240, accumulated).
    CHECK(jump_tick >= 119);
    CHECK(jump_tick <= 121);
    CHECK(freeze_tick >= 239);
    CHECK(freeze_tick <= 241);
    // Jump: +5000 env s exactly at the event tick; before it the clock ran at scale 100.
    CHECK_THAT(t_before_jump, Catch::Matchers::WithinAbs(start + 100.0 * jump_tick / 240.0, 1e-9));
    // Freeze: env time stays at whatever tick `freeze_tick` had, forever after.
    const real frozen = world.environment()->clock().env_time(world.tick());
    CHECK(frozen == t_before_freeze);
    CHECK_THAT(frozen, Catch::Matchers::WithinAbs(start + 5000.0 + 100.0 * freeze_tick / 240.0, 1e-6));
    CHECK(world.snapshots().read().environment.t_env_s == frozen);
    CHECK(world.snapshots().read().environment.time_scale == 0.0);

    // An environment-less scenario applies nothing.
    const io::Scenario plain = io::load_scenario(source_dir() + "/data/scenarios/vehicle_step_steer.json");
    io::BuiltScenario pb = io::build_scenario(plain);
    std::size_t pn = 0;
    CHECK(io::apply_environment_events(*pb.world, plain, pn) == 0);
}
