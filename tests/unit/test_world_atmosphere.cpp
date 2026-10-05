// tests/unit/test_world_atmosphere.cpp - weather plan W4: the environment's
// atmosphere and wind reaching the World's vehicles (freeze_tick_start_state ->
// VehicleTickEnv -> engine Sensors::ambient / aero wind). Library-level pieces
// (ISA, gust synthesis) are tested without ps_core in test_environment_atmosphere.cpp
// and test_environment_wind.cpp. Tags [environment][vehicle][aero][world].
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - W4-S1 (Q12 guard, `if (false && ...)` on the set_ambient throw): "set_ambient and
//    set_aero_environment are rejected while the weather owns the air" failed (no throw;
//    the stored ambient became 80000 Pa instead of staying 91000 Pa).
//  - W4-S2 (no air for the vehicle, System::step_all always using the tick env): "the
//    vehicle sees the environment's air" failed on the engine ambient (101325 Pa and no vapour
//    instead of 99754 Pa / 1537 Pa) and on the aero airspeed; "the weather moves the physics"
//    failed (airspeed not > 6 m/s); "a live ambient change" failed (no ambient drift).
//  - W4-S3 (altitude from the station only, no offset and no chassis z): "altitude offset
//    lowers pressure and density" failed (3 checks) and so did "atmosphere lift" in
//    test_environment_wind.cpp.
//  - W4-S5 (allocation, a std::vector<real>(8) in Environment::update_air): "an installed weather
//    environment with gusts allocates nothing inside the tick" failed at both 1 and 4 workers.
//  - W4-S6 (full rebuild, SimulatedEngine::pre_solve calling set_ambient instead of set_live_ambient):
//    "a live ambient change never rebuilds the steady grid" failed (steady_rebuild_count > 0; the test
//    also ran 84.8 s instead of 0.3 s).
//  The hash gating (no weather: the environment hash is the W2 one) is covered by
//  test_environment_wind.cpp "hash covers the weather only when it is on" and the scenario hash pins.

#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/environment/atmosphere.h"
#include "ps/environment/weather_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

using namespace ps;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

std::string data(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

environment::ClimateDesc gust_climate() { return io::load_climate_json(data("climates/test_weather_gusts.json")); }

// A preset with a given wind, everything else benign: wind FROM `from_deg`.
environment::WeatherPresetDesc wind_preset(real speed, real from_deg, real gust_factor) {
    environment::WeatherPresetDesc p = io::load_weather_preset_json(data("weather_presets/clear.json"));
    p.wind_mean_m_s = speed;
    p.wind_dir_deg = from_deg;
    p.gust_factor = gust_factor;
    return p;
}

environment::EnvironmentConfig preset_config(const environment::WeatherPresetDesc& p, std::uint64_t seed = 1, real altitude_offset = 0.0) {
    environment::EnvironmentConfig c;
    c.weather = environment::EnvironmentWeather::Preset;
    c.weather_preset = p;
    c.seed = seed;
    c.altitude_offset_m = altitude_offset;
    c.fixed_season = "summer";
    c.start = environment::EnvironmentStart{0, "summer", 5, 43200.0};
    c.time_scale = 0.0;   // frozen weather: only gusts (sim clock) move
    return c;
}

std::shared_ptr<environment::Environment> make_env(const environment::EnvironmentConfig& c) {
    return std::make_shared<environment::Environment>(gust_climate(), c);
}

// The simulated-engine car: the live ambient path runs for it (car_sedan_sim.json: kind "simulated").
ps_test::VehicleTestRig sim_rig(int workers) {
    return ps_test::build_vehicle_rig(data("vehicles/car_sedan_sim.json"), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
}

} // namespace

TEST_CASE("World atmosphere: set_ambient and set_aero_environment are rejected while the weather owns the air",
          "[environment][world][vehicle][aero]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    // No environment: both work.
    CHECK_NOTHROW(world.set_ambient(AmbientSample{90000.0, 280.0}));
    CHECK_NOTHROW(world.set_aero_environment(aero::Environment{1.1, Vec3{3.0, 0.0, 0.0}}));
    CHECK(world.ambient().pressure_pa == 90000.0);

    // An environment WITHOUT weather leaves them alone (W2 behaviour, owner Q12 narrowed to "provides atmosphere").
    environment::EnvironmentConfig plain;
    plain.fixed_season = "summer";
    world.install_environment(make_env(plain));
    CHECK_NOTHROW(world.set_ambient(AmbientSample{91000.0, 281.0}));
    CHECK_NOTHROW(world.set_aero_environment(aero::Environment{1.1, Vec3{3.0, 0.0, 0.0}}));
    world.remove_environment();

    // With weather: both throw, nothing is changed by the failed call.
    world.install_environment(make_env(preset_config(wind_preset(5.0, 270.0, 1.0))));
    CHECK_THROWS_AS(world.set_ambient(AmbientSample{80000.0, 270.0}), std::logic_error);
    CHECK_THROWS_AS(world.set_aero_environment(aero::Environment{1.0, Vec3{}}), std::logic_error);
    CHECK(world.ambient().pressure_pa == 91000.0);

    // After remove_environment they are accepted again.
    world.remove_environment();
    CHECK_NOTHROW(world.set_ambient(AmbientSample{80000.0, 270.0}));
    CHECK_NOTHROW(world.set_aero_environment(aero::Environment{1.0, Vec3{}}));
    CHECK(world.ambient().pressure_pa == 80000.0);
}

TEST_CASE("World atmosphere: the vehicle sees the environment's air", "[environment][world][vehicle][aero]") {
    ps_test::VehicleTestRig rig = sim_rig(1);
    World& w = *rig.world;
    CHECK_FALSE(w.vehicle_air(rig.vehicle).valid);

    // Westerly 8 m/s (toward +x), no gusts (G = 1).
    w.install_environment(make_env(preset_config(wind_preset(8.0, 270.0, 1.0))));
    ps_test::settle(w, 1.0);

    const environment::VehicleAir a = w.vehicle_air(rig.vehicle);
    REQUIRE(a.valid);
    CHECK_THAT(a.mean_wind_world.x, WithinAbs(8.0, 1e-9));
    CHECK_THAT(a.wind_world.x, WithinAbs(8.0, 1e-9));
    CHECK_THAT(a.altitude_m, WithinAbs(150.0 + 0.3, 0.5));  // location.elevation_m + the chassis height
    CHECK_THAT(a.density_kg_m3, WithinRel(environment::humid_air_density(a.pressure_pa, a.temperature_k, a.vapour_pressure_pa), 1e-12));

    // The same sample reaches the engine's Sensors (not the World's default ISA sea-level air) ...
    const AmbientSample amb = w.vehicle_engine_ambient(rig.vehicle);
    CHECK(amb.pressure_pa == a.pressure_pa);
    CHECK(amb.temperature_k == a.temperature_k);
    CHECK(amb.vapour_pressure_pa == a.vapour_pressure_pa);
    CHECK(amb.pressure_pa != 101325.0);
    // ... and the wind reaches the aero: a parked car in 8 m/s of tailwind feels ~8 m/s of body airflow.
    CHECK_THAT(w.vehicle_aero_telemetry(rig.vehicle).body_airspeed_m_s, WithinAbs(8.0, 0.5));

    // The published snapshot carries the same record per vehicle.
    const WorldSnapshot& snap = w.snapshots().read();
    REQUIRE(snap.vehicles.size() == 1);
    CHECK(snap.vehicles[0].air.valid);
    CHECK(snap.vehicles[0].air.pressure_pa == a.pressure_pa);
    CHECK(snap.vehicles[0].air.density_kg_m3 == a.density_kg_m3);

    // Removing the environment gives the air back to the World's own values.
    w.remove_environment();
    w.step();
    CHECK_FALSE(w.vehicle_air(rig.vehicle).valid);
    CHECK(w.vehicle_engine_ambient(rig.vehicle).pressure_pa == w.ambient().pressure_pa);
}

TEST_CASE("World atmosphere: altitude offset lowers pressure and density", "[environment][world][vehicle]") {
    real p[2], rho[2], t[2];
    int i = 0;
    for (const real offset : {0.0, 2000.0}) {
        ps_test::VehicleTestRig rig = sim_rig(1);
        rig.world->install_environment(make_env(preset_config(wind_preset(2.0, 270.0, 1.0), 1, offset)));
        ps_test::settle(*rig.world, 0.5);
        const environment::VehicleAir a = rig.world->vehicle_air(rig.vehicle);
        p[i] = a.pressure_pa;
        rho[i] = a.density_kg_m3;
        t[i] = a.temperature_k;
        ++i;
    }
    // ISA lift of 2000 m: pressure x ~0.78, density x ~0.82, temperature -13 K.
    CHECK(p[1] < 0.80 * p[0]);
    CHECK(p[1] > 0.74 * p[0]);
    CHECK(rho[1] < 0.86 * rho[0]);
    CHECK_THAT(t[1], WithinAbs(t[0] - 13.0, 0.01));
}

TEST_CASE("World atmosphere: gusts move the wind, deterministically, coherently across the tick", "[environment][world][vehicle][aero]") {
    ps_test::VehicleTestRig rig = sim_rig(1);
    World& w = *rig.world;
    w.install_environment(make_env(preset_config(wind_preset(10.0, 270.0, 1.66), 42)));
    real min_x = 1e9, max_x = -1e9, max_abs_z = 0.0;
    for (int k = 0; k < 2400; ++k) {   // 10 s
        w.step();
        const environment::VehicleAir a = w.vehicle_air(rig.vehicle);
        REQUIRE(a.valid);
        CHECK_THAT(a.wind_world.x, WithinAbs(a.mean_wind_world.x + a.gust_world.x, 1e-12));
        min_x = std::min(min_x, a.wind_world.x);
        max_x = std::max(max_x, a.wind_world.x);
        max_abs_z = std::max(max_abs_z, std::fabs(a.gust_world.z));
    }
    // sigma_u/U for G = 1.66 is 0.286: over 10 s the along-wind speed varies by well over 1 m/s and
    // the vertical component is alive.
    CHECK(max_x - min_x > 1.0);
    CHECK(max_abs_z > 0.05);
}

TEST_CASE("World atmosphere: a live ambient change never rebuilds the steady grid", "[environment][world][vehicle][drivetrain]") {
    // A dynamic-weather car on a running clock (x3000): temperature and pressure drift every tick
    // while the simulated engine idles - the live ambient path of SimulatedEngine.
    environment::EnvironmentConfig cfg;
    cfg.weather = environment::EnvironmentWeather::Dynamic;
    cfg.seed = 7;
    cfg.fixed_season = "summer";
    cfg.start = environment::EnvironmentStart{0, "summer", 5, 20000.0};
    cfg.time_scale = 3000.0;
    ps_test::VehicleTestRig rig = sim_rig(1);
    World& w = *rig.world;
    w.install_environment(make_env(cfg));
    ps_test::settle(w, 0.25);
    const AmbientSample first = w.vehicle_engine_ambient(rig.vehicle);
    const std::uint64_t rebuilds0 = w.vehicle_engine_steady_rebuild_count(rig.vehicle);
    ps_test::settle(w, 4.0);   // 12000 environment seconds of weather
    const AmbientSample last = w.vehicle_engine_ambient(rig.vehicle);
    CHECK(std::fabs(last.temperature_k - first.temperature_k) > 0.05);   // the ambient really moved
    CHECK(w.vehicle_engine_steady_rebuild_count(rig.vehicle) == rebuilds0);
    CHECK(w.vehicle_engine_steady_rebuild_count(rig.vehicle) == 0);
}

TEST_CASE("World atmosphere: an installed weather environment with gusts allocates nothing inside the tick",
          "[environment][world][vehicle][debug_alloc][jobs]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    for (const int workers : {1, 4}) {
        ps_test::VehicleTestRig rig = sim_rig(workers);
        World& w = *rig.world;
        w.install_environment(make_env(preset_config(wind_preset(8.0, 240.0, 1.5), 3)));
        w.set_control("throttle", 0.3);
        ps_test::settle(w, 2.0);   // warm-up: snapshot capacity, telemetry, contact buffers
        jobs::debug_alloc::reset();
        jobs::debug_alloc::set_armed(true);
        for (int k = 0; k < 240; ++k) w.step();
        jobs::debug_alloc::set_armed(false);
        INFO("workers " << workers);
        CHECK(jobs::debug_alloc::own_violation_count() == 0);
        // The gusts really ran inside that window.
        CHECK(w.vehicle_air(rig.vehicle).gust_world.x != 0.0);
    }
}

TEST_CASE("World atmosphere: 1 vs 10 workers give the same state hash under weather and gusts",
          "[environment][world][vehicle][determinism]") {
    const auto run = [](int workers) {
        ps_test::VehicleTestRig rig = sim_rig(workers);
        rig.world->install_environment(make_env(preset_config(wind_preset(9.0, 200.0, 1.6), 11, 500.0)));
        rig.world->set_control("throttle", 0.4);
        std::uint64_t h = 0;
        for (int k = 0; k < 480; ++k) {
            rig.world->step();
            h = h * 1099511628211ull ^ rig.world->state_hash();
        }
        return h;
    };
    const std::uint64_t a = run(1);
    CHECK(run(10) == a);
    CHECK(run(1) == a);   // run to run
}

TEST_CASE("World atmosphere: the weather moves the physics, no weather does not", "[environment][world][vehicle]") {
    const auto hash_with = [](int mode) {   // 0 = no environment, 1 = environment without weather, 2 = weather
        ps_test::VehicleTestRig rig = sim_rig(1);
        if (mode == 1) {
            environment::EnvironmentConfig c;
            c.fixed_season = "summer";
            rig.world->install_environment(make_env(c));
        } else if (mode == 2) {
            rig.world->install_environment(make_env(preset_config(wind_preset(8.0, 270.0, 1.0))));
        }
        rig.world->set_control("throttle", 0.3);
        for (int k = 0; k < 240; ++k) rig.world->step();
        const WorldSnapshot& s = rig.world->snapshots().read();
        return std::pair<std::uint64_t, real>{rig.world->state_hash(), s.vehicles[0].aero.body_airspeed_m_s};
    };
    const auto none = hash_with(0);
    const auto plain = hash_with(1);
    const auto weather = hash_with(2);
    CHECK(none.second < 5.0);     // no wind: only the car's own motion
    CHECK(weather.second > 6.0);   // 8 m/s of wind
    CHECK(weather.first != none.first);
    // An environment without weather folds only its clock anchor: the physics is bit-identical to none
    // (the hash differs by the environment's own fold only; a state_hash that ignored the car would not move
    // with weather either, so the airspeed above is the discriminating check).
    CHECK(plain.second == none.second);
}
