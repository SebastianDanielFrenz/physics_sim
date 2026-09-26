// tests/unit/test_engine_fuel_map.cpp -- PHYS-033 P4 I7a C5: the graph
// engine's optional fuel map (rpm x load -> g/s) and the PowertrainEnergy
// fuel sub-ledger (ps/drivetrain/energy_ledger.h's fuel_mass/fuel_energy_in/
// indicated_work/engine_heat/fuel_efficiency_violations). No tank exists yet
// (C6) - fuel_external_added stays 0 everywhere below.
//
// F2/F3/F4 drive core/src/drivetrain/engine_torque_map.h's EngineTorqueMap
// directly (core/src is on this target's include path, like
// test_throttle_control.cpp/test_powertrain_engine.cpp) with a small
// synthetic engine: FLAT wot/motoring curves (100 / -20 N*m everywhere) make
// tau_smooth_ = s*(m_rpm + c_k_) exactly 0 at every rpm (c_k_ = -motoring(0)
// = 20 cancels the constant -20 exactly), so frac_comb = tau_comb_/tau0_ is
// exactly 1 whenever combustion fires (no starter) and exactly 0 otherwise -
// see engine_torque_map.cpp's post_solve. Combined with a minimal 2x2 fuel
// map (rpm [500,7000], load [0,1], values [[0,3],[0,12]] g/s), the modelled
// thermal efficiency stays 4.76%-16.7% over the whole rpm domain independent
// of load, so F4's "no efficiency violation" bound can never be hit by
// construction, whatever the driven trajectory.
//
// F6/F9 drive a real World + Powertrain (tests/support/fixtures/
// vehicle_fuel_map_sedan.json, a car_sedan.json clone with the engine
// inlined and a synthetic fuel map/fuel reference added) to prove the fuel
// sub-ledger is entirely a spectator: no state_hash bit moves, no CSV column
// is added to a vehicle with no fuel map, and balance_error is undisturbed.

#include "drivetrain/engine_torque_map.h"
#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/io/scenario.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRpmToRadPerSec = real(2.0) * kPi / real(60.0);
constexpr real kSubstepH = 1.0 / 960.0;

real rpm_to_omega(real rpm) { return rpm * kRpmToRadPerSec; }

// ---------------------------------------------------------------------------
// F2/F3/F4: a small, fully hand-analysed synthetic engine (see file header).
// ---------------------------------------------------------------------------
EngineTorqueMap::Desc synthetic_engine_desc(bool with_fuel_map = true) {
    EngineTorqueMap::Desc d;
    d.wot_torque_nm_vs_rpm.x = {500.0, 7000.0};
    d.wot_torque_nm_vs_rpm.y = {100.0, 100.0};
    d.motoring_torque_nm_vs_rpm.x = {0.0, 7000.0};
    d.motoring_torque_nm_vs_rpm.y = {-20.0, -20.0};
    d.inertia_kgm2 = 0.15;
    d.breakaway_torque_nm = 20.0; // == -motoring(0) exactly (constructor allows equality)
    // idle_rpm/stall_rpm/fire_rpm/start_complete_rpm/dfco_*/limiter/idle/starter
    // all keep their EngineTorqueMap::Desc defaults (800/400/250/550,
    // 1300/1000, FuelCut@6800, kp 0.0018/ki 0.009/max_throttle 0.3,
    // 140 Nm/400 rpm) - already consistent with the flat curves above.
    d.initial_state = EngineTorqueMap::InitialState::Running;
    if (with_fuel_map) {
        d.has_fuel_map = true;
        d.fuel.name = "test fuel";
        d.fuel.phase = FuelPhase::Liquid;
        d.fuel.lhv_j_kg = 44.0e6;
        d.fuel.afr_stoich = 14.5;
        d.fuel.density_kg_m3 = 750.0;
        d.fuel.source = "synthetic (test_engine_fuel_map.cpp)";
        d.fuel_map.x = {500.0, 7000.0};      // rpm
        d.fuel_map.y = {0.0, 1.0};           // load (theta_c)
        d.fuel_map.values = {0.0, 3.0, 0.0, 12.0}; // row-major [ix*y.size()+iy]: rpm=500 -> {0,3}, rpm=7000 -> {0,12}
    }
    return d;
}

Sensors running_sensors(real rpm) {
    Sensors s;
    s.omega = rpm_to_omega(rpm);
    s.ignition = 1.0;
    s.starter = 0.0;
    s.clutch_engaged_fraction = 1.0;
    s.gearbox_in_neutral = true;
    return s;
}

ThrottleCommand throttle_of(real command, real floor = 0.0, real ceiling = 1.0) {
    ThrottleCommand t;
    t.command = command;
    t.floor = floor;
    t.ceiling = ceiling;
    return t;
}

TcuRequest no_ignition_cut() { return TcuRequest{false}; }

// ---------------------------------------------------------------------------
// F2: fuel integral at a speed-held dyno point - "idealized perfect dyno":
// since tau_smooth_ == 0 exactly (see file header) and there is no starter,
// tau0_ == tau_comb_ whenever combustion fires, so holding omega EXACTLY
// constant every substep (feed Sensors.omega == target, then post_solve
// (target, h) - no ODE/inertia integration needed at all) makes every
// substep's fuel draw identical: sum(fuel_mass_delta) over N substeps ==
// N*h*(fuel_map.value(rpm, theta_c)/1000) in closed form.
// ---------------------------------------------------------------------------
TEST_CASE("F2: fuel integral at a speed-held dyno point matches the map in closed form", "[unit][drivetrain][fuel]") {
    EngineTorqueMap engine(synthetic_engine_desc());
    constexpr real kTargetRpm = 3000.0;
    const real omega = rpm_to_omega(kTargetRpm);
    constexpr real kThrottle = 0.6;
    constexpr int kSubsteps = 100;

    real fuel_mass_sum = 0.0;
    real fuel_energy_sum = 0.0;
    real last_theta_c = -1.0;
    real last_rpm = -1.0;
    for (int i = 0; i < kSubsteps; ++i) {
        const PreSolveResult pre = engine.pre_solve(running_sensors(kTargetRpm), throttle_of(kThrottle), no_ignition_cut(), kSubstepH);
        // Held-rpm design: theta_c/rpm must be identical every substep (the
        // idle PI has no `e`-dependent memory here since e = idle_rpm - rpm
        // is a constant -2200 every call, clamping theta_idle to 0 - see the
        // file header's "idle clamps to 0 well above idle" analysis).
        if (i == 0) {
            last_theta_c = pre.theta_c;
            last_rpm = pre.rpm;
        } else {
            REQUIRE(pre.theta_c == last_theta_c);
            REQUIRE(pre.rpm == last_rpm);
        }
        const PostSolveResult post = engine.post_solve(omega, kSubstepH);
        fuel_mass_sum += post.fuel_mass_delta;
        fuel_energy_sum += post.fuel_energy_in_delta;
    }

    REQUIRE(last_theta_c > 0.0); // non-vacuous: combustion really fired at nonzero load
    const EngineTorqueMap::Desc& d = engine.desc();
    const real mdot_g_s = d.fuel_map.value(last_rpm, last_theta_c);
    const real expected_mass = kSubsteps * kSubstepH * (mdot_g_s / 1000.0);
    const real expected_energy = expected_mass * d.fuel.lhv_j_kg;

    CHECK(std::abs(fuel_mass_sum - expected_mass) <= 1e-12 * expected_mass);
    CHECK(std::abs(fuel_energy_sum - expected_energy) <= 1e-12 * expected_energy);
}

// ---------------------------------------------------------------------------
// F3: zero fuel in every non-combusting/cut state, nonzero at idle. Every
// case is a single pre_solve() call (fuel_flow_g_s is a PreSolveResult
// field, computed before post_solve ever runs).
// ---------------------------------------------------------------------------
TEST_CASE("F3: fuel flow is exactly zero whenever combustion cannot be firing, nonzero at idle", "[unit][drivetrain][fuel]") {
    SECTION("Off (ignition <= 0.5) even if previously Running") {
        EngineTorqueMap engine(synthetic_engine_desc());
        Sensors s = running_sensors(3000.0);
        s.ignition = 0.0;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Off);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Stalled, starter not pressed, at rest") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Stalled);
        Sensors s = running_sensors(0.0);
        s.starter = 0.0;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Stalled);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Cranking, below fire_rpm (250)") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Stalled);
        Sensors s = running_sensors(100.0); // 100 rpm < fire_rpm 250
        s.starter = 1.0;
        s.gearbox_in_neutral = true;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Cranking);
        CHECK(r.starter_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("DFCO (deceleration fuel cut-off): Running, high rpm, throttle 0") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Running);
        // rpm (2000) > dfco_enable_rpm (1300), throttle 0 < 0.01 -> latches
        // dfco_active_ true within this very call (used later in the same call).
        const PreSolveResult r = engine.pre_solve(running_sensors(2000.0), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.dfco_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Fuel-cut limiter: Running, at/above limiter.rpm (6800), full throttle") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Running);
        const PreSolveResult r = engine.pre_solve(running_sensors(7000.0), throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.limiter_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Ignition cut (upshift cut), contrasted against the same setup without it") {
        EngineTorqueMap engine_cut(synthetic_engine_desc());
        engine_cut.reset(EngineState::Running);
        TcuRequest cut{true};
        const PreSolveResult r_cut = engine_cut.pre_solve(running_sensors(3000.0), throttle_of(1.0), cut, kSubstepH);
        CHECK(r_cut.state == EngineState::Running);
        CHECK(r_cut.fuel_flow_g_s == 0.0);

        EngineTorqueMap engine_no_cut(synthetic_engine_desc());
        engine_no_cut.reset(EngineState::Running);
        const PreSolveResult r_no_cut = engine_no_cut.pre_solve(running_sensors(3000.0), throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r_no_cut.fuel_flow_g_s > 0.0); // same rpm/throttle, no cut -> real fuel flow
    }
    SECTION("Nonzero at idle: Running, foot off pedal, rpm near/below idle_rpm (800)") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Running);
        const PreSolveResult r = engine.pre_solve(running_sensors(750.0), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK_FALSE(r.dfco_active);
        CHECK(r.throttle_eff > 0.0); // the idle PI is feeding fuel, not the (zero) pedal
        CHECK(r.fuel_flow_g_s > 0.0);
    }
}

// ---------------------------------------------------------------------------
// F4: engine_heat >= 0 (no >100%-efficiency violation) every substep over a
// WOT launch + overrun, replicating Powertrain::post_solve_components'
// exact fuel-ledger booking rule (powertrain.cpp) by hand at the engine
// level. Single-inertia implicit-Euler dyno ODE: omega_new = omega_old +
// h*tau0/(J + h*d) (Assembly::torque's own per-substep linearisation).
// ---------------------------------------------------------------------------
TEST_CASE("F4: engine_heat never goes negative (no >100% efficiency) over a WOT launch + overrun", "[unit][drivetrain][fuel]") {
    EngineTorqueMap engine(synthetic_engine_desc());
    engine.reset(EngineState::Running);
    const real J = engine.desc().inertia_kgm2;

    real omega = rpm_to_omega(800.0); // start above stall_rpm (400) so Running does not immediately drop to Stalled
    PowertrainEnergy ledger;
    std::uint64_t violations = 0;
    constexpr int kSubsteps = 2000; // ~2.08 s at 960 Hz
    for (int i = 0; i < kSubsteps; ++i) {
        const real throttle_cmd = (i < kSubsteps / 2) ? 1.0 : 0.0; // first half WOT launch, second half overrun (lift)
        const PreSolveResult pre = engine.pre_solve(Sensors{omega, 1.0, 0.0, 1.0, true}, throttle_of(throttle_cmd), no_ignition_cut(), kSubstepH);
        const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
        const PostSolveResult post = engine.post_solve(omega_new, kSubstepH);
        omega = omega_new;

        // Powertrain::post_solve_components' exact booking rule (powertrain.cpp).
        ledger.indicated_work += post.indicated_work_delta;
        ledger.fuel_mass += post.fuel_mass_delta;
        ledger.fuel_energy_in += post.fuel_energy_in_delta;
        const real heat_delta = post.fuel_energy_in_delta - post.indicated_work_delta;
        ledger.engine_heat += heat_delta;
        if (heat_delta < 0.0) ++violations;

        INFO("substep " << i << " heat_delta=" << heat_delta << " engine_heat=" << ledger.engine_heat);
        REQUIRE(ledger.engine_heat >= -1e-15); // float-noise tolerant "never negative"
    }
    CHECK(violations == 0);
    ledger.fuel_efficiency_violations = violations;
    CHECK(ledger.fuel_efficiency_violations == 0);
    CHECK(ledger.fuel_mass > 0.0);   // non-vacuous: fuel really flowed
    CHECK(ledger.engine_heat > 0.0); // non-vacuous: real (imperfect) combustion heat was booked
}

} // namespace (F2/F3/F4 helpers)

// ---------------------------------------------------------------------------
// F5: loader rejections (core/src/io/vehicle_io.cpp's engine/fuel/fuel_map
// validation, read in full to pin every exact message below). Never touches
// data/** - every fixture is built in memory and written to a scratch temp
// file.
// ---------------------------------------------------------------------------
namespace f5 {

struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const nlohmann::json& j) {
        path = std::filesystem::temp_directory_path() /
               ("ps_test_fuel_map_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" +
                std::to_string(std::hash<std::string>{}(j.dump())) + ".json");
        std::ofstream out(path);
        out << j.dump(2);
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

std::string fuel_json_path() {
    return std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string();
}

// A minimal, otherwise-valid torque_map engine component with a fuel map -
// every F5 SECTION mutates a copy of this.
nlohmann::json base_engine_component() {
    nlohmann::json j;
    j["id"] = "engine";
    j["type"] = "engine";
    j["kind"] = "torque_map";
    j["wot_torque_nm_vs_rpm"] = {{"x", {500.0, 7000.0}}, {"y", {100.0, 100.0}}};
    j["motoring_torque_nm_vs_rpm"] = {{"x", {0.0, 7000.0}}, {"y", {-20.0, -20.0}}};
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 20.0;
    j["fuel"] = fuel_json_path();
    j["fuel_map"] = {{"rpm", {500.0, 7000.0}},
                      {"load", {0.0, 1.0}},
                      {"fuel_flow_g_s", {{0.0, 3.0}, {0.0, 12.0}}},
                      {"source", "synthetic test fixture (test_engine_fuel_map.cpp)"}};
    return j;
}

nlohmann::json vehicle_with_engine(const nlohmann::json& engine_component) {
    nlohmann::json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_engine_fuel_map.cpp)";
    v["wheels"] = nlohmann::json::array({{{"name", "FL"}}});
    v["powertrain"] = {{"components", nlohmann::json::array({engine_component})}};
    return v;
}

void expect_rejected(const nlohmann::json& engine_component, const std::string& substring) {
    const TempJsonFile f(vehicle_with_engine(engine_component));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(substring));
}

} // namespace f5

TEST_CASE("F5: loader rejects every malformed engine/fuel/fuel_map field with its exact message", "[unit][drivetrain][fuel][io]") {
    using f5::base_engine_component;
    using f5::expect_rejected;

    SECTION("valid base engine loads with has_fuel_map true (sanity, not a rejection)") {
        const f5::TempJsonFile f(f5::vehicle_with_engine(base_engine_component()));
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        bool found = false;
        for (const auto& c : d.powertrain.components) {
            if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
                found = true;
                CHECK(e->has_fuel_map);
                CHECK(e->fuel_map.value(500.0, 1.0) == 3.0);
                CHECK(e->fuel_map.value(7000.0, 1.0) == 12.0);
            }
        }
        CHECK(found);
    }

    SECTION("kind == simulated is rejected as not yet supported") {
        nlohmann::json e = base_engine_component();
        e["kind"] = "simulated";
        expect_rejected(e, "not yet supported (PHYS-033 I7b)");
    }
    SECTION("kind == bogus is rejected") {
        nlohmann::json e = base_engine_component();
        e["kind"] = "bogus";
        expect_rejected(e, "\"torque_map\" or \"simulated\"");
    }
    SECTION("fuel_map with no fuel reference is rejected") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        expect_rejected(e, "requires a 'fuel' reference");
    }
    SECTION("fuel present but empty is rejected") {
        nlohmann::json e = base_engine_component();
        e["fuel"] = "";
        expect_rejected(e, "'fuel' must be a non-empty path string");
    }
    SECTION("fuel_map missing rpm") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("rpm");
        expect_rejected(e, "'rpm' array is required");
    }
    SECTION("fuel_map missing load") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("load");
        expect_rejected(e, "'load' array is required");
    }
    SECTION("fuel_map missing fuel_flow_g_s") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("fuel_flow_g_s");
        expect_rejected(e, "'fuel_flow_g_s' array is required");
    }
    SECTION("fuel_map missing source") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("source");
        expect_rejected(e, "'source' is required");
    }
    SECTION("fuel_map rpm needs at least 2 points") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["rpm"] = {500.0};
        expect_rejected(e, "'rpm' needs at least 2 points");
    }
    SECTION("fuel_map load needs at least 2 points") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {0.0};
        expect_rejected(e, "'load' needs at least 2 points");
    }
    SECTION("fuel_map rpm must be strictly ascending") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["rpm"] = {7000.0, 500.0};
        expect_rejected(e, "'rpm' must be strictly ascending");
    }
    SECTION("fuel_map load must be strictly ascending") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {1.0, 0.0};
        expect_rejected(e, "'load' must be strictly ascending");
    }
    SECTION("fuel_map load must start at exactly 0.0") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {0.2, 1.0};
        expect_rejected(e, "'load' must start at exactly 0.0");
    }
    SECTION("fuel_map load must end at exactly 1.0") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {0.0, 0.8};
        expect_rejected(e, "'load' must end at exactly 1.0");
    }
    SECTION("fuel_map fuel_flow_g_s row count must match rpm point count") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["rpm"] = {500.0, 3000.0, 7000.0};
        // fuel_flow_g_s left at 2 rows -> mismatch (expected 3).
        expect_rejected(e, "must have one row per 'rpm' point (3)");
    }
    SECTION("fuel_map fuel_flow_g_s row length must match load point count") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, 3.0, 5.0}, {0.0, 12.0}};
        expect_rejected(e, "row 0 must have one value per 'load' point (2)");
    }
    SECTION("fuel_map fuel_flow_g_s value must be >= 0 and finite") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, -1.0}, {0.0, 12.0}};
        expect_rejected(e, "fuel_flow_g_s[0][1] must be >= 0 and finite");
    }
    SECTION("fuel_map fuel_flow_g_s at load 0 must be exactly 0.0") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{5.0, 3.0}, {0.0, 12.0}};
        expect_rejected(e, "fuel_flow_g_s[0][0] (load 0) must be exactly 0.0");
    }
    SECTION("fuel_map node implying >100% thermal efficiency is rejected") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, 3.0}, {0.0, 0.01}}; // at rpm=7000, load=1: far too little fuel for the mechanical power
        expect_rejected(e, "implies >100% thermal efficiency");
    }
}

// ---------------------------------------------------------------------------
// F6/F9: whole-World checks with tests/support/fixtures/vehicle_fuel_map_sedan.json
// (a car_sedan.json clone, engine inlined, a synthetic fuel map/fuel added).
// ---------------------------------------------------------------------------
namespace {

std::string fuel_map_sedan_path() { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicle_fuel_map_sedan.json"; }

// Mirrors test_throttle_control.cpp's WorldRig/make_world_rig exactly (same
// ground/chassis geometry as data/scenarios/vehicle_powertrain.json's own
// bodies) - a fresh copy here rather than a shared header, since that
// helper lives in an anonymous namespace in test_throttle_control.cpp.
struct WorldRig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

WorldRig make_world_rig(const vehicle::VehicleDesc& desc) {
    WorldRig rig;
    WorldConfig cfg;
    cfg.job_workers = 1;
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
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId chassis_id = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    rig.vehicle = rig.world->create_vehicle(desc, chassis_id);

    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

bool any_channel_contains(const std::vector<std::string>& names, const std::string& substr) {
    return std::any_of(names.begin(), names.end(), [&](const std::string& n) { return n.find(substr) != std::string::npos; });
}

} // namespace

TEST_CASE("F6: a fuel map with no tank is bit-identical to no fuel map at all (state_hash + CSV header)", "[unit][drivetrain][fuel][determinism]") {
    struct Ev {
        real t;
        const char* channel;
        real value;
    };
    // data/scenarios/vehicle_powertrain.json's own control_events, replayed
    // by hand against both worlds (not through io::build_scenario - the
    // comparison only needs the two worlds to see the identical script, not
    // to reproduce that scenario file's own recorded hash).
    const std::vector<Ev> events = {{0.2, "shift_up_count", 1.0}, {0.3, "throttle", 0.8}, {3.0, "shift_up_count", 2.0},
                                     {6.0, "brake", 0.5},         {7.0, "shift_down_count", 1.0},
                                     {8.0, "starter", 1.0},       {8.2, "starter", 0.0}};

    const vehicle::VehicleDesc plain_desc = ps_test::load_sedan_desc();
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());

    WorldRig ra = make_world_rig(plain_desc);
    WorldRig rb = make_world_rig(fueled_desc);

    constexpr real kTickHz = 240.0;
    constexpr int kTicks = 2400; // 10 s, matching vehicle_powertrain.json's duration_s
    std::size_t next_event = 0;
    int mismatches = 0;
    int first_mismatch = -1;
    for (int tick = 0; tick < kTicks; ++tick) {
        const real sim_time = tick / kTickHz;
        while (next_event < events.size() && sim_time >= events[next_event].t) {
            ra.world->set_control(events[next_event].channel, events[next_event].value);
            rb.world->set_control(events[next_event].channel, events[next_event].value);
            ++next_event;
        }
        ra.world->step();
        rb.world->step();
        if (ra.world->state_hash() != rb.world->state_hash() && mismatches++ == 0) first_mismatch = tick;
    }
    INFO("first differing tick " << first_mismatch << " (" << mismatches << " of " << kTicks << ")");
    CHECK(mismatches == 0);
    CHECK(next_event == events.size()); // non-vacuous: the whole script actually ran

    const auto& plain_names = ra.world->telemetry().channel_names();
    const auto& fueled_names = rb.world->telemetry().channel_names();
    CHECK_FALSE(any_channel_contains(plain_names, ".fuel."));
    CHECK(any_channel_contains(fueled_names, ".fuel."));
}

TEST_CASE("F9: balance_error stays within bound with a fuel map over a WOT launch", "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());
    WorldRig rig = make_world_rig(fueled_desc);

    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);

    // Settle the suspension onto the ground first (mirrors
    // test_powertrain_energy_balance.cpp's own ps_test::settle(*rig.world,
    // 1.0) before it starts checking balance_error): with throttle still 0
    // the ledger's cumulative terms (denom below) are still near-zero for
    // the first handful of ticks after reset_vehicle, so a tiny, purely
    // numerical (non-physical) residual in balance_error can transiently
    // make the ratio enormous even though nothing is actually leaking -
    // exactly the failure mode measured here before this fix (ratio 4.04
    // at an early tick). Settling first ensures denom is already a
    // meaningful magnitude before the WOT launch below starts being
    // checked.
    for (int settle_tick = 0; settle_tick < 240; ++settle_tick) rig.world->step(); // 1 s

    rig.world->set_control("throttle", 1.0);

    // Mirrors test_powertrain_energy_balance.cpp's own check_tick: balance_error
    // is a per-tick residual, but the ledger terms it closes against are
    // CUMULATIVE since vehicle creation, so the bound must scale with their
    // running sum (denom), not be a bare absolute constant - a bare 5e-8
    // absolute bound is meaningless once the cumulative ledger reaches
    // thousands of joules over a launch.
    auto sum_abs_terms = [](const drivetrain::PowertrainEnergy& e) {
        return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) +
               std::abs(e.gear_loss) + std::abs(e.integrator_loss) + std::abs(e.clutch_loss) +
               std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) + std::abs(e.engagement_loss) +
               std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
    };

    real worst_ratio = 0.0;
    for (int tick = 0; tick < 720; ++tick) { // 3 s at 240 Hz
        rig.world->step();
        const drivetrain::PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) {
            const real ratio = std::abs(e.balance_error) / denom;
            worst_ratio = std::max(worst_ratio, ratio);
            CHECK(ratio <= 5e-8);
        }
    }
    INFO("worst |balance_error|/denom = " << worst_ratio);
    CHECK(worst_ratio <= 5e-8);

    const drivetrain::PowertrainEnergy& final_ledger = rig.world->powertrain_energy(rig.vehicle);
    CHECK(final_ledger.fuel_mass > 0.0);         // non-vacuous: fuel really flowed
    CHECK(final_ledger.fuel_efficiency_violations == 0);
}
