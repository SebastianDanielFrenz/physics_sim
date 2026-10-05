// tests/unit/test_environment_surface_world.cpp - weather plan W5: the surface
// condition grid inside a World (Environment::begin_tick in the existing
// freeze_tick_start_state stage, published per class in EnvironmentSnapshot,
// folded into the hash only while installed, read by nothing in the vehicle
// physics yet). The library pieces (column, energy balance, S1-S4, loader) are
// tested without ps_core in test_environment_surface.cpp.
// Tags [environment][vehicle][world].
//
// RECORDED SABOTAGES: see the "RECORDED SABOTAGES" block in test_environment_surface.cpp
// (W5-S3, an allocation in Environment::fill_surfaces, failed this file's zero-allocation case
// with 244 own allocations at 1 and 4 workers; W5-S2, an unconditional hash fold, is caught by
// the W4 pin test in ps_determinism_tests, not here).

#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/environment/surface_conditions_io.h"
#include "ps/io/scenario.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

std::string data(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

// Dynamic weather on the W3 fixture climate; a fast clock (x36000 = 150 env s per
// 240 Hz tick, two or three 60 s grid steps per tick). Surfaces from the shipped
// default.json mapped onto `names` (empty = no surface conditions).
std::shared_ptr<environment::Environment> make_env(const std::vector<std::string>& names, real scale = 36000.0,
                                                   bool with_surfaces = true) {
    environment::ClimateDesc climate = io::load_climate_json(data("climates/test_weather.json"));
    climate.condition = environment::ConditionGridDesc{60.0, 2.0 * 86400.0, 3.0 * 86400.0};
    environment::EnvironmentConfig cfg;
    cfg.weather = environment::EnvironmentWeather::Dynamic;
    cfg.seed = 11;
    cfg.time_scale = scale;
    cfg.start = environment::EnvironmentStart{0, "summer", 4, 3600.0 * 6.0};
    if (with_surfaces) {
        cfg.surface_conditions = io::load_surface_conditions_json(data("surface_conditions/default.json"));
        cfg.surface_names = names;
    }
    return std::make_shared<environment::Environment>(climate, cfg);
}

std::vector<std::string> table_names(const io::SurfaceTable& t) {
    std::vector<std::string> n;
    for (std::size_t i = 0; i < t.size(); ++i) n.push_back(t.name_for(static_cast<SurfaceId>(i)));
    return n;
}

ps_test::VehicleTestRig rig(int workers) {
    return ps_test::build_vehicle_rig(data("vehicles/car_sedan.json"), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
}

// A throwaway scenario file under the system temp directory.
struct TempScenario {
    std::filesystem::path path;
    explicit TempScenario(const std::string& json, const char* tag) {
        path = std::filesystem::temp_directory_path() / (std::string("ps_env_w5_") + tag + ".json");
        std::ofstream(path) << json;
    }
    ~TempScenario() { std::error_code ec; std::filesystem::remove(path, ec); }
};

std::string scenario_json(const std::string& top_extra, const std::string& environment_block) {
    return R"({
  "format": "physics_sim.scenario/1", "name": "env_w5_tmp", "duration_s": 2.0,
  "bodies": [ {"name": "ground", "motion": "static", "shape": {"type": "box", "half_extents": [10, 10, 0.5]}, "position": [0, 0, -0.5]} ],
)" + top_extra + R"(  "environment": )" + environment_block + "\n}\n";
}

} // namespace

TEST_CASE("World surfaces: the snapshot publishes each class's surface temperature, the vehicle is unaffected",
          "[environment][vehicle][world]") {
    ps_test::VehicleTestRig with = rig(1);
    ps_test::VehicleTestRig without = rig(1);
    const std::vector<std::string> names = table_names(*with.surfaces);
    with.world->install_environment(make_env(names));
    without.world->install_environment(make_env(names, 36000.0, false));

    const environment::EnvironmentSnapshot* s0 = without.world->environment_snapshot();
    REQUIRE(s0 != nullptr);
    for (int i = 0; i < 240; ++i) {
        with.world->step();
        without.world->step();
    }
    const environment::EnvironmentSnapshot* s = with.world->environment_snapshot();
    REQUIRE(s != nullptr);
    REQUIRE(s->has_surfaces);
    CHECK_FALSE(without.world->environment_snapshot()->has_surfaces);
    REQUIRE(s->surface_count == static_cast<std::int32_t>(names.size()));

    // The published sample is the grid's interpolated value at the snapshot's env time.
    const environment::Environment* env = with.world->environment();
    REQUIRE(env != nullptr);
    const environment::ConditionGrid* g = env->surface_conditions();
    REQUIRE(g != nullptr);
    for (int i = 0; i < s->surface_count; ++i) {
        const environment::SurfaceConditionSample& c = s->surfaces[static_cast<std::size_t>(i)];
        INFO(names[static_cast<std::size_t>(i)]);
        CHECK(c.surface_id == i);
        CHECK(c.has_state == g->has_state(i));
        if (c.has_state) {
            CHECK(c.t_surface_c == g->surface_temp_k(i, s->t_env_s) - environment::kCelsiusToKelvin);
            CHECK(c.t_surface_c > -40.0);
            CHECK(c.t_surface_c < 80.0);
        }
    }
    // low_mu is the static ice rink: no state.
    const auto low_mu = static_cast<std::size_t>(with.surfaces->id_for("low_mu"));
    CHECK_FALSE(s->surfaces[low_mu].has_state);

    // Nothing in the vehicle physics reads the surfaces yet (W6+): the chassis
    // moves bit-identically; only the hash differs (the grid is folded in).
    const Pose a = with.world->get_pose(with.chassis);
    const Pose b = without.world->get_pose(without.chassis);
    CHECK(a.position.x == b.position.x);
    CHECK(a.position.y == b.position.y);
    CHECK(a.position.z == b.position.z);
    CHECK(a.orientation.w == b.orientation.w);
    CHECK(with.world->state_hash() != without.world->state_hash());
}

TEST_CASE("World surfaces: 1 and 10 workers give the same state hash", "[environment][vehicle][world]") {
    std::uint64_t h[2] = {0, 0};
    int k = 0;
    for (const int workers : {1, 10}) {
        ps_test::VehicleTestRig r = rig(workers);
        r.world->install_environment(make_env(table_names(*r.surfaces)));
        const std::int64_t k0 = r.world->environment()->surface_conditions()->grid_index();
        r.world->set_control("throttle", 0.4);
        r.world->set_control("steer", 0.1);
        for (int i = 0; i < 480; ++i) r.world->step();
        // 480 ticks x 150 env s (the last tick's environment runs at its start: 479 x 150 s
        // = 71850 s past the install, 1197 grid steps).
        const environment::ConditionGrid* g = r.world->environment()->surface_conditions();
        CHECK(g->grid_index() - k0 == 1197);
        CHECK(g->grid_index() == g->grid_index_at(r.world->environment_snapshot()->t_env_s));
        h[k++] = r.world->state_hash();
    }
    INFO("hash 1 worker 0x" << std::hex << h[0] << ", 10 workers 0x" << h[1]);
    CHECK(h[0] == h[1]);
}

TEST_CASE("World surfaces: an installed surface grid allocates nothing inside the tick",
          "[environment][vehicle][world][debug_alloc][jobs]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    for (const int workers : {1, 4}) {
        ps_test::VehicleTestRig r = rig(workers);
        r.world->install_environment(make_env(table_names(*r.surfaces)));
        ps_test::settle(*r.world, 1.5);
        const std::int64_t k_before = r.world->environment()->surface_conditions()->grid_index();
        jobs::debug_alloc::reset();
        jobs::debug_alloc::set_armed(true);
        for (int i = 0; i < 240; ++i) r.world->step();
        // A backward jump inside the armed window: the spin-up runs in the next tick.
        r.world->set_environment_time(r.world->environment_snapshot()->t_env_s - 86400.0);
        for (int i = 0; i < 4; ++i) r.world->step();
        jobs::debug_alloc::set_armed(false);
        INFO("workers " << workers);
        CHECK(jobs::debug_alloc::own_violation_count() == 0);
        // The grid really stepped inside the window (240 ticks x 150 env s / 60 s = 600 steps).
        CHECK(r.world->environment()->surface_conditions()->grid_index() != k_before);
        jobs::debug_alloc::reset();
    }
}

TEST_CASE("World surfaces: install rejects classes that differ from the world's surface table",
          "[environment][vehicle][world]") {
    ps_test::VehicleTestRig r = rig(1);
    // Same names in another order: SurfaceIds would point at the wrong class.
    std::vector<std::string> names = table_names(*r.surfaces);
    std::swap(names[0], names[1]);
    CHECK_THROWS_WITH(r.world->install_environment(make_env(names)),
                      ContainsSubstring("surface classes do not match the world's surface table"));
    CHECK(r.world->environment() == nullptr);
    // A subset is rejected too.
    CHECK_THROWS_AS(r.world->install_environment(make_env({"asphalt", "dirt"})), std::invalid_argument);
    // The table's order installs.
    CHECK_NOTHROW(r.world->install_environment(make_env(table_names(*r.surfaces))));
}

TEST_CASE("Scenario surface_conditions: parsing and rejections", "[environment][io][world]") {
    const std::string climate = "\"climate\": \"" + data("climates/test_weather.json") + "\"";
    const std::string table = "  \"surface_table_path\": \"" + data("surfaces/surfaces.json") + "\",\n";
    const std::string conds = "\"surface_conditions\": \"" + data("surface_conditions/default.json") + "\"";
    const auto load_msg = [](const std::string& top, const std::string& block, const char* tag) -> std::string {
        TempScenario f(scenario_json(top, block), tag);
        try {
            io::load_scenario(f.path.string());
        } catch (const std::runtime_error& e) {
            return e.what();
        }
        return {};
    };

    // Accepted: the classes are the surface table's names in SurfaceId order.
    {
        TempScenario f(scenario_json(table, "{ " + climate + ", \"weather\": \"dynamic\", " + conds + " }"), "ok");
        const io::Scenario sc = io::load_scenario(f.path.string());
        REQUIRE(sc.environment.has_value());
        REQUIRE(sc.environment->config.surface_conditions.has_value());
        CHECK(sc.environment->config.surface_names == table_names(io::SurfaceTable(data("surfaces/surfaces.json"))));
        CHECK_THAT(sc.environment->surface_conditions_path, ContainsSubstring("default.json"));
    }
    CHECK_THAT(load_msg("", "{ " + climate + ", \"weather\": \"dynamic\", " + conds + " }", "notable"),
               ContainsSubstring("needs the scenario's \"surface_table_path\""));
    CHECK_THAT(load_msg(table, "{ " + climate + ", " + conds + " }", "noweather"),
               ContainsSubstring("needs the environment's \"weather\" key"));
    CHECK_THAT(load_msg(table, "{ " + climate + ", \"weather\": \"dynamic\", \"surface_conditions\": \"does/not/exist.json\" }", "nofile"),
               ContainsSubstring("exist.json"));
    CHECK_THAT(load_msg(table, "{ " + climate + ", \"weather\": \"dynamic\", \"surface_conditions\": 3 }", "notstr"),
               ContainsSubstring("must be a file path string"));
}
