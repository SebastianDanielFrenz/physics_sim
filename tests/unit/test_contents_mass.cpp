// tests/unit/test_contents_mass.cpp -- contents mass (vehicle::ContentsMassDesc, racing_game PR #22): with
// contents_mass.enabled the chassis body carries the fuel tank (and N2O bottle) contents on top of its dry mass,
// updated at the tick boundary whenever the contents moved by update_threshold_kg or more. Off by default, so every
// existing vehicle is unchanged.
//
// Fixture: tests/support/fixtures/vehicle_tank_sedan.json (car_sedan + an E5 tank, capacity 40 kg, initial 30 kg).
//
// Payload mass (World::set_vehicle_payload_mass, the farm game's grain and tank loads): the last four cases.
// Recorded sabotages (2026-10-31, harvest chain G3), each shown to fail the cases named:
//   S1  apply_contents_mass without `+ cm.payload_kg`           -> "payload ..." body-mass and inertia cases fail
//   S2  set_vehicle_payload_mass without the enabled check      -> the "needs the block" case fails
//   S3  payload not part of the semi-live threshold test        -> "payload follows" case fails (a 0.05 kg change is applied)

#include "ps/io/vehicle_io.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <variant>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using namespace ps;
using Catch::Matchers::ContainsSubstring;

namespace {

std::string fixture_path(const std::string& name) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/" + name; }
std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

constexpr real kDry = 1500.0;

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

// The same ground/chassis as test_fuel_tank.cpp's make_rig; gear 1, auto-clutch launch, throttle 0.
Rig make_rig(const vehicle::VehicleDesc& desc, int workers = 1) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    rig.world = std::make_unique<World>(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = kDry;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    rig.chassis = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(data_path("surfaces/surfaces.json")));
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);
    rig.world->set_control("assist.auto_clutch", 1.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = drivetrain::EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

vehicle::VehicleDesc tank_sedan(bool enabled, real threshold = 0.1) {
    vehicle::VehicleDesc d = io::load_vehicle_json(fixture_path("vehicle_tank_sedan.json"));
    d.contents_mass.enabled = enabled;
    d.contents_mass.update_threshold_kg = threshold;
    return d;
}

real speed(const World& w, BodyId b) {
    const Vec3 v = w.backend().get_motion(b).linear;
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

} // namespace

TEST_CASE("contents mass: the chassis carries the initial fill at once and its inertia scales with the mass", "[unit][vehicle][contents_mass]") {
    Rig off = make_rig(tank_sedan(false));
    Rig on = make_rig(tank_sedan(true));
    CHECK(off.world->get_mass(off.chassis) == Catch::Approx(kDry).epsilon(1e-6));
    CHECK(off.world->chassis_contents_mass(off.vehicle) == 0.0);
    CHECK(off.world->chassis_dry_mass(off.vehicle) == 0.0);
    CHECK(on.world->fuel_tank_mass(on.vehicle) == 30.0);
    CHECK(on.world->get_mass(on.chassis) == Catch::Approx(kDry + 30.0).epsilon(1e-6));
    CHECK(on.world->chassis_contents_mass(on.vehicle) == 30.0);
    CHECK(on.world->chassis_dry_mass(on.vehicle) == Catch::Approx(kDry).epsilon(1e-6));
    // Uniform density: every inertia component scales by (dry + fuel) / dry.
    const Mat3 i_off = off.world->backend().get_inertia_world(off.chassis);
    const Mat3 i_on = on.world->backend().get_inertia_world(on.chassis);
    const real ratio = (kDry + 30.0) / kDry;
    CHECK(i_on(0, 0) == Catch::Approx(i_off(0, 0) * ratio).epsilon(1e-5));
    CHECK(i_on(1, 1) == Catch::Approx(i_off(1, 1) * ratio).epsilon(1e-5));
    CHECK(i_on(2, 2) == Catch::Approx(i_off(2, 2) * ratio).epsilon(1e-5));
}

TEST_CASE("contents mass: burning fuel lightens the body semi-live, a refuel adds it back", "[unit][vehicle][contents_mass]") {
    const real threshold = 0.002; // small so a few seconds of WOT crosses it several times
    Rig rig = make_rig(tank_sedan(true, threshold));
    World& w = *rig.world;
    w.set_control("throttle", 1.0);
    int updates = 0;
    real last_applied = w.chassis_contents_mass(rig.vehicle);
    for (int t = 0; t < 240 * 6; ++t) {
        w.step();
        const real fuel = w.fuel_tank_mass(rig.vehicle);
        const real applied = w.chassis_contents_mass(rig.vehicle);
        if (applied != last_applied) ++updates;
        last_applied = applied;
        // The body never trails the tank by the threshold plus one tick of burn.
        CHECK(std::abs(applied - fuel) < threshold + 0.001);
        CHECK(w.get_mass(rig.chassis) == Catch::Approx(w.chassis_dry_mass(rig.vehicle) + applied).epsilon(1e-6));
    }
    CHECK(w.fuel_tank_mass(rig.vehicle) < 30.0 - 2.0 * threshold);
    CHECK(updates >= 2);
    w.set_fuel_tank_mass(rig.vehicle, 40.0); // a refuel at the tick boundary
    w.step();
    w.step();
    CHECK(w.chassis_contents_mass(rig.vehicle) == Catch::Approx(w.fuel_tank_mass(rig.vehicle)).margin(threshold + 0.001));
    CHECK(w.get_mass(rig.chassis) == Catch::Approx(kDry + 40.0).margin(0.01));
}

TEST_CASE("contents mass: a full tank accelerates the car more slowly than an empty one", "[unit][vehicle][contents_mass]") {
    vehicle::VehicleDesc light = tank_sedan(true);
    vehicle::VehicleDesc heavy = tank_sedan(true);
    for (auto& c : light.powertrain.components) {
        if (auto* t = std::get_if<drivetrain::FuelTankDesc>(&c.params)) t->initial_kg = 1.0;
    }
    for (auto& c : heavy.powertrain.components) {
        if (auto* t = std::get_if<drivetrain::FuelTankDesc>(&c.params)) t->initial_kg = 40.0;
    }
    Rig a = make_rig(light);
    Rig b = make_rig(heavy);
    a.world->set_control("throttle", 1.0);
    b.world->set_control("throttle", 1.0);
    for (int t = 0; t < 240 * 3; ++t) {
        a.world->step();
        b.world->step();
    }
    const real va = speed(*a.world, a.chassis);
    const real vb = speed(*b.world, b.chassis);
    CHECK(va > 1.0);
    CHECK(vb < va);
}

TEST_CASE("contents mass: identical state hashes at 1 and 4 workers; off leaves the hash as without the field", "[unit][vehicle][contents_mass][determinism]") {
    Rig a = make_rig(tank_sedan(true), 1);
    Rig b = make_rig(tank_sedan(true), 4);
    Rig c = make_rig(tank_sedan(false), 1);
    Rig d = make_rig(io::load_vehicle_json(fixture_path("vehicle_tank_sedan.json")), 1);
    for (Rig* r : {&a, &b, &c, &d}) r->world->set_control("throttle", 1.0);
    for (int t = 0; t < 960; ++t) {
        for (Rig* r : {&a, &b, &c, &d}) r->world->step();
        REQUIRE(a.world->state_hash() == b.world->state_hash());
        REQUIRE(c.world->state_hash() == d.world->state_hash());
    }
    CHECK(a.world->state_hash() != c.world->state_hash());
}

TEST_CASE("contents mass: the loader reads the block and rejects bad values", "[unit][vehicle][contents_mass][io]") {
    std::ifstream in(fixture_path("vehicle_tank_sedan.json"));
    const nlohmann::json base = nlohmann::json::parse(in);
    // Written next to the fixture's own directory so its relative refs resolve; removed again at once.
    const std::filesystem::path tmp = std::filesystem::path(PS_SOURCE_DIR) / "tests/support/fixtures/_contents_mass_tmp.json";
    auto load = [&](const nlohmann::json& j) {
        {
            std::ofstream out(tmp);
            out << j.dump(2);
        }
        struct Remove {
            std::filesystem::path p;
            ~Remove() {
                std::error_code ec;
                std::filesystem::remove(p, ec);
            }
        } remove{tmp};
        return io::load_vehicle_json(tmp.string());
    };
    CHECK_FALSE(load(base).contents_mass.enabled);
    nlohmann::json v = base;
    v["contents_mass"] = {{"enabled", true}, {"update_threshold_kg", 0.25}, {"source", "test"}};
    const vehicle::VehicleDesc d = load(v);
    CHECK(d.contents_mass.enabled);
    CHECK(d.contents_mass.update_threshold_kg == 0.25);
    v["contents_mass"] = {{"enabled", true}, {"update_threshold_kg", 0.0}};
    CHECK_THROWS_WITH(load(v), ContainsSubstring("update_threshold_kg must be in (0, 1000)"));
    v["contents_mass"] = {{"enabled", true}, {"threshold", 1.0}};
    CHECK_THROWS_WITH(load(v), ContainsSubstring("contents_mass: unknown key 'threshold'"));
    v["contents_mass"] = true;
    CHECK_THROWS_WITH(load(v), ContainsSubstring("contents_mass must be an object"));
}

// ---- payload mass --------------------------------------------------------------------------------------------------

TEST_CASE("payload mass: the body carries it on top of dry mass and fuel, the inertia scales, 0 takes it off again", "[unit][vehicle][contents_mass][payload]") {
    Rig rig = make_rig(tank_sedan(true));
    World& w = *rig.world;
    const Mat3 i0 = w.backend().get_inertia_world(rig.chassis);
    const real m0 = w.get_mass(rig.chassis);
    CHECK(m0 == Catch::Approx(kDry + 30.0).epsilon(1e-6));
    CHECK(w.vehicle_payload_mass(rig.vehicle) == 0.0);

    w.set_vehicle_payload_mass(rig.vehicle, 12800.0);
    CHECK(w.vehicle_payload_mass(rig.vehicle) == 12800.0);
    CHECK(w.get_mass(rig.chassis) == Catch::Approx(m0).epsilon(1e-9)); // not before the tick boundary
    w.step();
    CHECK(w.get_mass(rig.chassis) == Catch::Approx(kDry + 30.0 + 12800.0).margin(0.2));
    CHECK(w.chassis_contents_mass(rig.vehicle) == Catch::Approx(30.0 + 12800.0).margin(0.2));
    CHECK(w.chassis_dry_mass(rig.vehicle) == Catch::Approx(kDry).epsilon(1e-6));
    const Mat3 i1 = w.backend().get_inertia_world(rig.chassis);
    const real ratio = w.get_mass(rig.chassis) / m0;
    CHECK(i1(0, 0) == Catch::Approx(i0(0, 0) * ratio).epsilon(1e-5));
    CHECK(i1(2, 2) == Catch::Approx(i0(2, 2) * ratio).epsilon(1e-5));

    w.set_vehicle_payload_mass(rig.vehicle, 0.0);
    w.step();
    CHECK(w.get_mass(rig.chassis) == Catch::Approx(m0).margin(0.2));
    CHECK(w.chassis_contents_mass(rig.vehicle) == Catch::Approx(30.0).margin(0.2));
}

TEST_CASE("payload mass: a vehicle that did not opt in is refused and a bad mass is refused", "[unit][vehicle][contents_mass][payload]") {
    Rig off = make_rig(tank_sedan(false));
    CHECK_THROWS_AS(off.world->set_vehicle_payload_mass(off.vehicle, 100.0), std::logic_error);
    CHECK(off.world->vehicle_payload_mass(off.vehicle) == 0.0);
    Rig on = make_rig(tank_sedan(true));
    CHECK_THROWS_AS(on.world->set_vehicle_payload_mass(on.vehicle, -1.0), std::invalid_argument);
    CHECK_THROWS_AS(on.world->set_vehicle_payload_mass(on.vehicle, std::nan("")), std::invalid_argument);
    CHECK(on.world->vehicle_payload_mass(on.vehicle) == 0.0);
}

TEST_CASE("payload mass: it follows semi-live inside the threshold, as the fuel does", "[unit][vehicle][contents_mass][payload]") {
    Rig rig = make_rig(tank_sedan(true, 0.1));
    World& w = *rig.world;
    w.set_vehicle_payload_mass(rig.vehicle, 1000.0);
    w.step();
    const real base = w.get_mass(rig.chassis);
    // a change under the threshold is not applied, a change over it is
    w.set_vehicle_payload_mass(rig.vehicle, 1000.05);
    w.step();
    CHECK(w.get_mass(rig.chassis) == Catch::Approx(base).epsilon(1e-12));
    w.set_vehicle_payload_mass(rig.vehicle, 1000.5);
    w.step();
    CHECK(w.get_mass(rig.chassis) == Catch::Approx(base + 0.5).margin(0.06));
    // an unloading at 75 kg/s in 240 Hz ticks: the body never trails the load by more than the threshold
    real load = 1000.5;
    for (int t = 0; t < 240; ++t) {
        load -= 75.0 / 240.0;
        w.set_vehicle_payload_mass(rig.vehicle, load);
        w.step();
        CHECK(std::abs((w.get_mass(rig.chassis) - kDry - w.fuel_tank_mass(rig.vehicle)) - load) <= 0.1 + 1e-6);
    }
}

TEST_CASE("payload mass: a heavier body accelerates slower, and the state is the same for any worker count", "[unit][vehicle][contents_mass][payload][determinism]") {
    auto run = [](int workers, real payload, std::uint64_t& hash) {
        Rig rig = make_rig(tank_sedan(true), workers);
        World& w = *rig.world;
        w.set_vehicle_payload_mass(rig.vehicle, payload);
        w.set_control("throttle", 1.0);
        for (int t = 0; t < 240 * 4; ++t) {
            if (t == 240 * 2) w.set_vehicle_payload_mass(rig.vehicle, payload * 0.5);
            w.step();
        }
        hash = w.state_hash();
        return speed(w, rig.chassis);
    };
    std::uint64_t h0 = 0, h1 = 0, h10 = 0, h_light = 0;
    const real v_loaded = run(1, 1500.0, h1);
    const real v_loaded10 = run(10, 1500.0, h10);
    const real v_empty = run(1, 0.0, h_light);
    (void)h0;
    CHECK(h1 == h10);
    CHECK(v_loaded10 == v_loaded);
    CHECK(h1 != h_light);
    CHECK(v_empty > v_loaded * 1.05);
}
