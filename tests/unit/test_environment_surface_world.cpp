// tests/unit/test_environment_surface_world.cpp - weather plan W5/W6: the
// surface condition grid inside a World (Environment::begin_tick in the
// existing freeze_tick_start_state stage, published per class in
// EnvironmentSnapshot, folded into the hash only while installed). W6 adds the
// condition grip (ConditionGrip): a wheel contact on a TAGGED surface (a mesh
// ground with a SurfaceId) reads the class's friction factor; the first test's
// ground is an untagged BoxShape, so its contact_surface is kInvalidSurfaceId,
// the factor is never consulted and that test's bit-identity is trivially exact.
// The library pieces (column, energy balance, S1-S4, loader, G1/G2/G5/G7-G9) are
// tested without ps_core in test_environment_surface.cpp. Tags
// [environment][vehicle][world].
//
// Validation cases (weather plan 13, W6 row, World level):
//  G3  wet vs dry threshold braking (pedal ramp, no ABS): the stop-distance ratio at 20 m/s inside
//      Wong's [1.14, 1.8] band and inside the PIARC bracket, larger at 30 m/s.
//  G4b a dry installed grid on a tagged ground leaves the chassis bit-identical to an environment without
//      surfaces (the wheel factor is exactly 1.0 when dry).
//  G6  one World with the grip path really running (tagged ground, wet) allocates nothing inside the tick
//      (debug_alloc armed) and gives the same state hash at 1 and 10 workers.
//  G10 the snapshot publishes the water above the texture, the condition label and the default-tyre factor.
//
// RECORDED SABOTAGES: see the "RECORDED SABOTAGES" block in test_environment_surface.cpp (W6-G3, W6-G4b,
// W6-G6, W6-G10 are listed there with their measured failures; W5-S3, an allocation in
// Environment::fill_surfaces, failed this file's zero-allocation case with 244 own allocations at 1 and 4
// workers; W5-S2, an unconditional hash fold, is caught by the W4 pin test in ps_determinism_tests, not here).

#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/environment/surface_conditions_io.h"
#include "ps/io/scenario.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
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

// The same vehicle on a mesh ground tagged with the asphalt SurfaceId: every wheel contact carries a valid
// SurfaceId, so the condition grip is consulted (an untagged BoxShape ground never reaches it).
ps_test::VehicleTestRig rig_on_asphalt(int workers) {
    return ps_test::build_vehicle_rig_on_surface(data("vehicles/car_sedan.json"), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6,
                                                 "asphalt", workers);
}


// Force the asphalt class of an installed environment to a water depth and
// restore it at the current tick. `water_mm < 0` = the class's texture storage
// (fully wet). The environment runs at time_scale 0, so the grid never steps
// afterwards and the forced state is frozen for the whole run.
void set_asphalt_water(ps::World& w, real water_mm) {
    environment::Environment* env = w.environment();
    REQUIRE(env != nullptr);
    const environment::ConditionGrid* g = env->surface_conditions();
    REQUIRE(g != nullptr);
    environment::ConditionCheckpoint ck = env->checkpoint_conditions();
    for (int i = 0; i < g->class_count(); ++i) {
        if (g->class_name(i) == "asphalt") {
            const real mm = water_mm < 0.0 ? g->column(i).w_tex_mm : water_mm;
            ck.prev[i].water_mm = mm;
            ck.next[i].water_mm = mm;
        }
    }
    env->restore_conditions(ck, w.tick());
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

    // W6: the grip reads a wheel's contact SurfaceId, but this rig's ground is
    // an untagged BoxShape (contact_surface == kInvalidSurfaceId), so factor()
    // returns 1 and the chassis still moves bit-identically; only the hash
    // differs (the grid is folded in). G3 below uses a tagged mesh ground.
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
        ps_test::VehicleTestRig r = rig_on_asphalt(workers);   // tagged ground: the wheels read the condition grip
        r.world->install_environment(make_env(table_names(*r.surfaces)));
        set_asphalt_water(*r.world, -1.0);                     // start with a fully wet texture; the weather then evolves it
        const std::int64_t k0 = r.world->environment()->surface_conditions()->grid_index();
        r.world->set_control("throttle", 0.4);
        r.world->set_control("steer", 0.1);
        for (int i = 0; i < 480; ++i) r.world->step();
        // 480 ticks x 150 env s (the last tick's environment runs at its start: 479 x 150 s
        // = 71850 s past the install, 1197 grid steps).
        const environment::ConditionGrid* g = r.world->environment()->surface_conditions();
        CHECK(g->grid_index() - k0 == 1197);
        CHECK(r.world->wheel_telemetry(r.vehicle, 0).surface == r.surfaces->id_for("asphalt"));   // the grip path ran
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
        ps_test::VehicleTestRig r = rig_on_asphalt(workers);   // tagged ground: ConditionGrip::factor runs per wheel substep
        r.world->install_environment(make_env(table_names(*r.surfaces)));
        set_asphalt_water(*r.world, -1.0);
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
        CHECK(r.world->wheel_telemetry(r.vehicle, 0).surface == r.surfaces->id_for("asphalt"));   // the grip path ran
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
    // Windows ASan caveat (CLAUDE.md, vault TOOL-019): no catch-by-reference in test code -
    // the throwing call is wrapped in Catch2's CHECK_THROWS_WITH instead.
    const auto load_scen = [](const std::string& top, const std::string& block, const char* tag) {
        TempScenario f(scenario_json(top, block), tag);
        return io::load_scenario(f.path.string());
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
    CHECK_THROWS_WITH(load_scen("", "{ " + climate + ", \"weather\": \"dynamic\", " + conds + " }", "notable"),
               ContainsSubstring("needs the scenario's \"surface_table_path\""));
    CHECK_THROWS_WITH(load_scen(table, "{ " + climate + ", " + conds + " }", "noweather"),
               ContainsSubstring("needs the environment's \"weather\" key"));
    CHECK_THROWS_WITH(load_scen(table, "{ " + climate + ", \"weather\": \"dynamic\", \"surface_conditions\": \"does/not/exist.json\" }", "nofile"),
               ContainsSubstring("exist.json"));
    CHECK_THROWS_WITH(load_scen(table, "{ " + climate + ", \"weather\": \"dynamic\", \"surface_conditions\": 3 }", "notstr"),
               ContainsSubstring("must be a file path string"));
}

// ================================================================== W6

namespace {

// Threshold braking WITHOUT ABS (a driver who ramps the pedal until the tyres
// peak): the chassis is given `v0`, the hubs roll with it (reset_vehicle), the
// gearbox is in neutral (no engine braking), the brake pedal ramps 0 -> 1 over
// `ramp_s`. Returns the peak deceleration (mean over `kWindowS`) while the speed is still
// >= `min_speed` (the wet grip depends on speed, so the peak is taken near the entry speed) and
// the ground speed it occurred at. The stop-distance ratio of two surfaces at the
// same entry speed is the inverse ratio of these peaks (d = v^2 / 2 a).
struct RampPeak {
    real peak_decel = 0.0;
    real speed_at_peak = 0.0;
};

RampPeak brake_ramp_peak(ps_test::VehicleTestRig& r, real v0, real ramp_s, real min_speed) {
    ps::World& w = *r.world;
    ps_test::settle(w, 0.5);
    w.set_control("assist.auto_clutch", 0.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("clutch", 0.0);
    w.backend().set_motion(r.chassis, Motion{Vec3{v0, 0.0, 0.0}, Vec3::zero()});
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = drivetrain::EngineState::Running;
    o.clutch_locked = true;
    w.reset_vehicle(r.vehicle, o);
    const real dt = w.dt();
    constexpr real kWindowS = 0.2;
    const int window = static_cast<int>(std::lround(kWindowS / dt));
    std::vector<real> speed;
    RampPeak out;
    const int steps = static_cast<int>(std::lround((ramp_s + 2.0) / dt));
    for (int i = 0; i < steps; ++i) {
        w.set_control("brake", std::min(real(1.0), static_cast<real>(i) * dt / ramp_s));
        w.step();
        speed.push_back(w.get_motion(r.chassis).linear.length());
        const int n = static_cast<int>(speed.size());
        if (n > window && speed.back() >= min_speed) {
            const real a = (speed[static_cast<std::size_t>(n - 1 - window)] - speed.back()) / kWindowS;
            if (a > out.peak_decel) {
                out.peak_decel = a;
                out.speed_at_peak = speed.back();
            }
        }
    }
    return out;
}

RampPeak asphalt_ramp(bool wet, real v0) {
    ps_test::VehicleTestRig r = ps_test::build_vehicle_rig_on_surface(
        data("vehicles/car_sedan.json"), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, "asphalt", 1);
    r.world->install_environment(make_env(table_names(*r.surfaces), 0.0));
    set_asphalt_water(*r.world, wet ? -1.0 : 0.0);
    return brake_ramp_peak(r, v0, 0.8, 0.7 * v0);
}

} // namespace

TEST_CASE("W6-G3: wet asphalt threshold braking is longer than dry, inside the Wong band, and grows with speed",
          "[environment][vehicle][world][grip]") {
    // Wong (Theory of Ground Vehicles, p. 26 [C, via plan 5.2]): peak mu dry 0.8-0.9, wet 0.5-0.7 -> the
    // stop-distance ratio d_wet/d_dry = mu_dry/mu_wet lies in [0.8/0.7, 0.9/0.5] = [1.14, 1.8].
    // The model's wet level is the PIARC form wet(v) = 0.71 exp((v_ref - v)/Sp), v_ref = 60 km/h = 16.67 m/s,
    // Sp = 14.2 + 89.7 MTD = 85.96 km/h = 23.88 m/s for MTD 0.8 mm (asphalt in default.json), clamped to the
    // damp level 0.85 [C PIARC 1995, D]: the peak is taken while the speed is in [0.7 v0, v0], where the ratio is
    // bracketed by exp((v - v_ref)/Sp)/0.71 at the two ends (dry has no speed dependence) - an independent
    // derivation of the expected band, not a fit.
    const real sp = (14.2 + 89.7 * 0.8) / 3.6;
    const auto bracket = [sp](real v_lo, real v_hi, real& lo, real& hi) {
        lo = std::exp((v_lo - 60.0 / 3.6) / sp) / 0.71;
        hi = std::exp((v_hi - 60.0 / 3.6) / sp) / 0.71;
    };

    const real v60 = 20.0;   // the peak is taken at 14-20 m/s (~ the IFI reference speed 60 km/h = 16.7 m/s)
    const RampPeak dry = asphalt_ramp(false, v60);
    const RampPeak wet = asphalt_ramp(true, v60);
    const real ratio60 = dry.peak_decel / wet.peak_decel;
    INFO("20 m/s entry: dry peak " << dry.peak_decel << " m/s2 at " << dry.speed_at_peak << " m/s, wet " << wet.peak_decel
                                   << " at " << wet.speed_at_peak << " m/s, distance ratio " << ratio60);
    REQUIRE(dry.peak_decel > 6.0);   // the ramp really reaches the tyre limit (mu ~ 0.9)
    CHECK(ratio60 > 1.14);
    CHECK(ratio60 < 1.8);
    real lo60 = 0.0, hi60 = 0.0;
    bracket(0.7 * v60, v60, lo60, hi60);
    INFO("IFI bracket " << lo60 << " .. " << hi60);
    CHECK(ratio60 > lo60);
    CHECK(ratio60 < hi60);

    // The wet grip falls with speed (plan 5.2 step 3), the dry one does not: the ratio is larger at 30 m/s (108 km/h),
    // where the model is below Wong's wet range (the Wong table names no speed) but inside its own IFI bracket.
    const real v100 = 30.0;
    const RampPeak dry100 = asphalt_ramp(false, v100);
    const RampPeak wet100 = asphalt_ramp(true, v100);
    const real ratio100 = dry100.peak_decel / wet100.peak_decel;
    INFO("30 m/s entry: dry peak " << dry100.peak_decel << " at " << dry100.speed_at_peak << " m/s, wet "
                                    << wet100.peak_decel << " at " << wet100.speed_at_peak << " m/s, ratio " << ratio100);
    real lo100 = 0.0, hi100 = 0.0;
    bracket(0.7 * v100, v100, lo100, hi100);
    CHECK(ratio100 > ratio60 + 0.2);
    CHECK(ratio100 > lo100);
    CHECK(ratio100 < hi100);
}

TEST_CASE("W6-G4b: a dry installed grid on a tagged ground leaves the vehicle bit-identical to no surfaces",
          "[environment][vehicle][world][grip]") {
    // Dry (no water anywhere, frozen clock): every class factor is exactly 1.0, so the wheel's lambda_mu is the
    // SurfaceTable value as before W6 and the chassis cannot tell the grid is there.
    const auto run = [](bool with_surfaces, real water_mm) {
        ps_test::VehicleTestRig r = rig_on_asphalt(1);
        r.world->install_environment(make_env(table_names(*r.surfaces), 0.0, with_surfaces));
        if (with_surfaces) set_asphalt_water(*r.world, water_mm);
        r.world->set_control("gear_request", 1.0);
        r.world->set_control("throttle", 1.0);
        r.world->set_control("steer", 0.15);
        for (int i = 0; i < 480; ++i) r.world->step();
        return std::make_pair(r.world->get_pose(r.chassis), r.world->get_motion(r.chassis));
    };
    const auto none = run(false, 0.0);
    const auto dry = run(true, 0.0);
    CHECK(dry.first.position.x == none.first.position.x);
    CHECK(dry.first.position.y == none.first.position.y);
    CHECK(dry.first.position.z == none.first.position.z);
    CHECK(dry.second.linear.x == none.second.linear.x);
    CHECK(dry.second.angular.z == none.second.angular.z);
    // A wet texture changes the cornering (lambda_mu 0.85 x), so the check above does discriminate.
    const auto wet = run(true, -1.0);
    INFO("dry y " << dry.first.position.y << " wet y " << wet.first.position.y << ", dry vx " << dry.second.linear.x
                  << " wet vx " << wet.second.linear.x);
    CHECK(wet.first.position.y != dry.first.position.y);
}

TEST_CASE("W6-G10: the snapshot publishes the water above the texture, the condition label and the grip factor",
          "[environment][vehicle][world][grip]") {
    ps_test::VehicleTestRig r = rig_on_asphalt(1);
    r.world->install_environment(make_env(table_names(*r.surfaces), 0.0));
    const auto asphalt = static_cast<std::size_t>(r.surfaces->id_for("asphalt"));
    const auto grass = static_cast<std::size_t>(r.surfaces->id_for("grass"));
    const auto low = static_cast<std::size_t>(r.surfaces->id_for("low_mu"));
    const environment::ConditionGrid* g = r.world->environment()->surface_conditions();
    const real w_tex = g->column(static_cast<int>(asphalt)).w_tex_mm;

    // Dry.
    set_asphalt_water(*r.world, 0.0);
    r.world->step();
    {
        const environment::SurfaceConditionSample& s = r.world->environment_snapshot()->surfaces[asphalt];
        CHECK(s.water_above_texture_mm == 0.0);
        CHECK(s.condition == 0);
        CHECK(s.condition_factor == 1.0);
    }
    // Half the texture: damp.
    set_asphalt_water(*r.world, 0.5 * w_tex);
    r.world->step();
    {
        const environment::SurfaceConditionSample& s = r.world->environment_snapshot()->surfaces[asphalt];
        CHECK(s.water_above_texture_mm == 0.0);
        CHECK(s.condition == 1);
        CHECK(s.condition_factor < 1.0);
        CHECK(s.condition_factor > 0.85);
    }
    // 0.5 mm above the texture: wet, factor = the damp-clamped wet level at standstill (0.85).
    set_asphalt_water(*r.world, w_tex + 0.5);
    r.world->step();
    {
        const environment::SurfaceConditionSample& s = r.world->environment_snapshot()->surfaces[asphalt];
        CHECK_THAT(s.water_above_texture_mm, WithinAbs(0.5, 1e-12));
        CHECK(s.condition == 2);
        CHECK_THAT(s.condition_factor, WithinAbs(0.85, 1e-12));
    }
    // The static class stays at the identity.
    const environment::SurfaceConditionSample& st = r.world->environment_snapshot()->surfaces[low];
    CHECK(st.condition == 0);
    CHECK(st.condition_factor == 1.0);
    CHECK(r.world->environment_snapshot()->surfaces[grass].has_state);
}
