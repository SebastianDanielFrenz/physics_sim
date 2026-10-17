// tests/unit/test_engine_thermal_jacket.cpp - the optional water jacket of the engine thermal model
// (EngineThermalDesc::water_jacket, CoolingSystemDesc::pump, docs/data_formats/engine_thermal.md "Water jacket")
// and the steady-state solver (ps/drivetrain/engine_thermal_steady.h). White-box: core/src is on the include path.
// Tags [thermal][drivetrain].
//
// Worked example: data/engine_thermal/i4_2l_na.json (the generic 2 L I4, whose jacket comes from
// class_si_passenger.json, all [AS]) with the sedan cooling system. The jacket-less baseline is
// tests/support/fixtures/thermal/i4_2l_na_no_jacket.json (the same file with "water_jacket": null).
//
// J1 loader values and rejections; J2 more wetted area / film coefficient / pump flow -> lower metal temperature;
// J3 ledger closure with a jacket; J4 steady solver == long transient (with and without a jacket); J5 a file
// without the new keys is bit-identical to the same model with them at their neutral values (the shipped
// thermal scenario hash is pinned in tests/determinism/test_worker_count_hash.cpp); J6 World: worker-count
// invariant hash, the jacket changes the hash, zero own allocation; J7 every shipped engine has a jacket, and the
// jacket telemetry channels exist only with one.
//
// Sabotage record (each applied once to core/src/drivetrain/engine_thermal.cpp, measured, then reverted):
//   J3: the jacket ledger books block-air at the coolant instead of the metal temperature: worst |balance| /
//       sum|flows| = 6.5e-3 (bound 1e-9).
//   J4: the steady solver drops block-air from the metal balance: the jacket cases miss the transient by 14-23 K
//       in the metal (bound 0.02 K); the no-jacket cases still pass (they never read that line).
//   J2: the jacket film coefficient ignores the pump flow (always htc_ref): the three h(Q) checks, the
//       conductance-at-1.5x-pump check and the pump-upgrade check (metal at 1.5x flow < stock - 0.2 K) fail.

#include "drivetrain/engine_thermal.h"

#include "ps/drivetrain/engine_thermal_steady.h"
#include "ps/io/engine_thermal_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;

namespace {

const std::string kRoot = PS_SOURCE_DIR;
constexpr real kC = kCelsiusToKelvin;

std::string jacket_file() { return kRoot + "/data/engine_thermal/i4_2l_na.json"; }
std::string plain_file() { return kRoot + "/tests/support/fixtures/thermal/i4_2l_na_no_jacket.json"; }
std::string sedan_cooling() { return kRoot + "/data/cooling_systems/sedan.json"; }

EngineThermalConfig config(const std::string& engine_file) {
    EngineThermalConfig c;
    c.engine = io::load_engine_thermal_json(engine_file);
    c.cooling = io::load_cooling_system_json(sedan_cooling());
    return c;
}

struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_engine_thermal_jacket_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::create_directories(dir);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    std::string write(const std::string& name, const json& j) const {
        const auto p = dir / name;
        std::ofstream out(p);
        out << j.dump(2);
        return p.string();
    }
};

// A jacket file extending the plain I4 file with the given water_jacket block.
json jacket_json(const json& block) {
    json j;
    j["format"] = "physics_sim.engine_thermal/1";
    j["extends"] = std::filesystem::path(plain_file()).generic_string();
    j["source"] = "test";
    j["water_jacket"] = block;
    return j;
}

json good_block() {
    return json{{"wetted_area_m2", 0.4},       {"coolant_volume_l", 6.0}, {"htc_ref_w_m2_k", 5000.0},
                {"reference_flow_l_min", 60.0}, {"flow_exponent", 0.8},    {"htc_min_w_m2_k", 500.0}};
}

// Runs the model at constant inputs for `seconds` with tick `dt`.
void run(EngineThermal& th, real rpm, real wall_w, real fric_w, real seconds, real dt) {
    const int n = static_cast<int>(seconds / dt + 0.5);
    for (int i = 0; i < n; ++i) {
        th.begin_tick();
        th.accumulate(wall_w / th.config().engine.wall_heat_fraction * dt, fric_w * dt, dt);
        th.step(rpm);
    }
}

// SplitMix64 counter hash -> uniform [0, 1).
real unit_hash(std::uint64_t counter, std::uint64_t salt) {
    std::uint64_t z = counter * 0x9E3779B97F4A7C15ULL + salt * 0xD1B54A32D192ED03ULL + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    return static_cast<real>(z >> 11) * (1.0 / 9007199254740992.0);
}

// The shipped thermostat (88/100 C, 2 K play, 20 s wax lag) hunts in a limit cycle at the small openings a cruise
// needs (the radiator at full flow could reject several times the heat), so a long transient never sits at the
// fixed point. Tests that compare a transient with the steady state use a proportional thermostat (no play, a
// wide band, a short lag), whose fixed point is stable.
EngineThermalConfig stable_thermostat(EngineThermalConfig c) {
    c.engine.thermostat.open_k = 60.0 + kC;
    c.engine.thermostat.full_open_k = 110.0 + kC;
    c.engine.thermostat.hysteresis_k = 0.0;
    c.engine.thermostat.time_constant_s = 5.0;
    return c;
}

SteadyThermalInput cruise_input(real wall_w = 12000.0) {
    SteadyThermalInput in;
    in.engine_rpm = 3000.0;
    in.wall_heat_w = wall_w;
    in.friction_heat_w = 3000.0;
    in.ambient_k = 25.0 + kC;
    in.ram_speed_mps = 25.0;
    return in;
}

} // namespace

TEST_CASE("J1 water jacket and pump loader: values and rejections", "[thermal][drivetrain][io]") {
    const EngineThermalDesc d = io::load_engine_thermal_json(jacket_file());
    REQUIRE(d.water_jacket.enabled);
    CHECK(d.water_jacket.wetted_area_m2 == Approx(0.4).epsilon(1e-12));
    CHECK(d.water_jacket.coolant_volume_l == Approx(6.0).epsilon(1e-12));
    CHECK(d.water_jacket.reference_flow_l_min == Approx(60.0).epsilon(1e-12));
    CHECK(d.water_jacket.htc_ref_w_m2_k == 5000.0);
    CHECK(d.water_jacket.htc_min_w_m2_k == 500.0);
    CHECK(d.water_jacket.flow_exponent == 0.8);
    const ThermalCapacities caps = thermal_capacities(d);
    CHECK(caps.coolant_j_k == Approx(6.0 / 1000.0 * 1065.0 * 3400.0).epsilon(1e-12));
    CHECK(caps.metal_j_k == Approx(2.0 * 26600.0 - 6.0 / 1000.0 * 1065.0 * 3400.0).epsilon(1e-12));
    CHECK(caps.oil_j_k == d.oil_capacity_j_k);
    // The shipped files carry no jacket and no pump block.
    CHECK_FALSE(io::load_engine_thermal_json(plain_file()).water_jacket.enabled);
    CHECK(io::load_cooling_system_json(sedan_cooling()).pump.flow_factor == 1.0);
    const ThermalCapacities plain_caps = thermal_capacities(io::load_engine_thermal_json(plain_file()));
    CHECK(plain_caps.coolant_j_k == 0.0);
    CHECK(plain_caps.metal_j_k == 2.0 * 26600.0);

    TempDir tmp;
    // Absolute keys load as given; flow_exponent defaults to 0.8.
    {
        json b = good_block();
        b.erase("flow_exponent");
        const EngineThermalDesc a = io::load_engine_thermal_json(tmp.write("abs.json", jacket_json(b)));
        CHECK(a.water_jacket.flow_exponent == 0.8);
        CHECK(a.water_jacket.wetted_area_m2 == 0.4);
    }
    struct Bad {
        const char* name;
        json block;
        const char* message;
    };
    const auto with = [](const char* key, json value) {
        json b = good_block();
        b[key] = std::move(value);
        return b;
    };
    const auto without = [](const char* key) {
        json b = good_block();
        b.erase(key);
        return b;
    };
    const Bad cases[] = {
        {"both_area", with("wetted_area_m2_per_l", 0.2), "exactly one of \"wetted_area_m2\""},
        {"no_area", without("wetted_area_m2"), "exactly one of \"wetted_area_m2\""},
        {"no_volume", without("coolant_volume_l"), "exactly one of \"coolant_volume_l\""},
        {"no_flow", without("reference_flow_l_min"), "exactly one of \"reference_flow_l_min\""},
        {"no_htc", without("htc_ref_w_m2_k"), "htc_ref_w_m2_k"},
        {"no_min", without("htc_min_w_m2_k"), "htc_min_w_m2_k"},
        {"area_zero", with("wetted_area_m2", 0.0), "wetted area"},
        {"area_big", with("wetted_area_m2", 25.0), "wetted area"},
        {"volume_big", with("coolant_volume_l", 150.0), "coolant volume"},
        {"flow_big", with("reference_flow_l_min", 6000.0), "reference flow"},
        {"htc_low", with("htc_ref_w_m2_k", 50.0), "htc_ref_w_m2_k"},
        {"htc_high", with("htc_ref_w_m2_k", 60000.0), "htc_ref_w_m2_k"},
        {"min_above_ref", with("htc_min_w_m2_k", 6000.0), "htc_min_w_m2_k"},
        {"min_negative", with("htc_min_w_m2_k", -1.0), "htc_min_w_m2_k"},
        {"exponent_low", with("flow_exponent", 0.2), "flow_exponent"},
        {"exponent_high", with("flow_exponent", 1.2), "flow_exponent"},
        {"unknown", with("wall_thickness_mm", 4.0), "unknown key \"wall_thickness_mm\""},
        // 15 L of coolant = 54315 J/K > the 53200 J/K engine lump: no metal left.
        {"coolant_eats_metal", with("coolant_volume_l", 15.0), "less than 10 %"},
    };
    for (const Bad& b : cases) {
        INFO(b.name);
        REQUIRE_THROWS_WITH(io::load_engine_thermal_json(tmp.write(std::string(b.name) + ".json", jacket_json(b.block))),
                            ContainsSubstring(b.message));
    }

    // Cooling-system pump block.
    const auto cooling = [&](const char* name, const json& pump) {
        json j;
        j["format"] = "physics_sim.cooling_system/1";
        j["extends"] = std::filesystem::path(sedan_cooling()).generic_string();
        j["source"] = "test";
        j["pump"] = pump;
        return tmp.write(name, j);
    };
    CHECK(io::load_cooling_system_json(cooling("p15.json", json{{"flow_factor", 1.5}})).pump.flow_factor == 1.5);
    REQUIRE_THROWS_WITH(io::load_cooling_system_json(cooling("p0.json", json{{"flow_factor", 0.0}})), ContainsSubstring("flow_factor"));
    REQUIRE_THROWS_WITH(io::load_cooling_system_json(cooling("p5.json", json{{"flow_factor", 5.0}})), ContainsSubstring("flow_factor"));
    REQUIRE_THROWS_WITH(io::load_cooling_system_json(cooling("pk.json", json{{"flow_factor", 1.0}, {"head_m", 3.0}})),
                        ContainsSubstring("unknown key \"head_m\""));
    REQUIRE_THROWS_WITH(io::load_cooling_system_json(cooling("pm.json", json::object())), ContainsSubstring("flow_factor"));
}

TEST_CASE("J2 film coefficient and conductance follow the pump flow; more area, htc or flow cools the metal",
          "[thermal][drivetrain]") {
    const EngineThermalConfig cfg = config(jacket_file());
    // h(Q) = max(500, 5000 (Q/60)^0.8): exactly htc_ref at the reference flow (3000 rpm), the floor at rest.
    CHECK(coolant_pump_flow_l_min(cfg, 3000.0) == Approx(60.0).epsilon(1e-12));
    CHECK(water_jacket_htc_w_m2_k(cfg.engine, 60.0) == Approx(5000.0).epsilon(1e-12));
    CHECK(water_jacket_htc_w_m2_k(cfg.engine, 120.0) == Approx(5000.0 * std::pow(2.0, 0.8)).epsilon(1e-12));
    CHECK(water_jacket_htc_w_m2_k(cfg.engine, 0.0) == 500.0);
    CHECK(water_jacket_htc_w_m2_k(cfg.engine, 1.0) == 500.0); // 5000 (1/60)^0.8 = 188 < floor
    CHECK(water_jacket_conductance_w_k(cfg, 3000.0) == Approx(0.4 * 5000.0).epsilon(1e-12));
    CHECK(water_jacket_conductance_w_k(config(plain_file()), 3000.0) == 0.0);
    {
        EngineThermalConfig pumped = cfg;
        pumped.cooling.pump.flow_factor = 1.5;
        CHECK(coolant_pump_flow_l_min(pumped, 3000.0) == Approx(90.0).epsilon(1e-12));
        CHECK(water_jacket_conductance_w_k(pumped, 3000.0) == Approx(0.4 * 5000.0 * std::pow(1.5, 0.8)).epsilon(1e-12));
    }

    const SteadyThermalInput in = cruise_input();
    const SteadyThermalResult base = solve_steady_thermal(cfg, in);
    REQUIRE(base.bounded);
    CHECK(base.metal_c > base.coolant_c);
    CHECK(base.oil_c > base.coolant_c);
    // The metal sits above the coolant by (jacket heat) / G_j.
    CHECK(base.metal_c - base.coolant_c == Approx(base.jacket_w / base.jacket_conductance_w_k).epsilon(1e-9));

    const auto metal_with = [&](auto mutate, bool transient) {
        EngineThermalConfig c = transient ? stable_thermostat(cfg) : cfg;
        mutate(c);
        if (!transient) return solve_steady_thermal(c, in).metal_c;
        EngineThermal th(c);
        th.set_env(ThermalEnv{in.ambient_k, in.ambient_pressure_pa, in.ram_speed_mps, in.engine_rpm});
        run(th, in.engine_rpm, in.wall_heat_w, in.friction_heat_w, 3600.0, 1.0);
        return th.metal_temperature_k() - kC;
    };
    for (const bool transient : {false, true}) {
        INFO("transient " << transient);
        const real stock = metal_with([](EngineThermalConfig&) {}, transient);
        const real area = metal_with([](EngineThermalConfig& c) { c.engine.water_jacket.wetted_area_m2 *= 2.0; }, transient);
        const real htc = metal_with([](EngineThermalConfig& c) { c.engine.water_jacket.htc_ref_w_m2_k *= 2.0; }, transient);
        const real pump = metal_with([](EngineThermalConfig& c) { c.cooling.pump.flow_factor = 1.5; }, transient);
        const real small = metal_with([](EngineThermalConfig& c) { c.engine.water_jacket.wetted_area_m2 *= 0.5; }, transient);
        CHECK(area < stock - 0.5);
        CHECK(htc < stock - 0.5);
        CHECK(pump < stock - 0.2);
        CHECK(small > stock + 0.5);
        // Doubling the area and doubling the film coefficient double the same product.
        CHECK(area == Approx(htc).margin(1e-6));
    }
    // Lowering the metal temperature cools the oil too (oil exchanges heat with the metal).
    EngineThermalConfig big = cfg;
    big.engine.water_jacket.wetted_area_m2 *= 2.0;
    CHECK(solve_steady_thermal(big, in).oil_c < base.oil_c);
}

TEST_CASE("J3 ledger closure with a water jacket over random inputs", "[thermal][drivetrain]") {
    EngineThermal th(config(jacket_file()));
    REQUIRE(th.has_water_jacket());
    const real dt = 1.0 / 240.0;
    real worst = 0.0;
    for (std::uint64_t i = 0; i < 20000; ++i) {
        if (i % 4000 == 0) th.set_env(ThermalEnv{263.15 + 40.0 * unit_hash(i, 1), 101325.0, 40.0 * unit_hash(i, 2), 0.0});
        if (i == 7000) th.set_state(EngineThermalState{40.0, 30.0}); // a re-baseline in the middle
        th.begin_tick();
        th.accumulate(2.0e5 * unit_hash(i, 3) * dt, 1.0e4 * unit_hash(i, 4) * dt, dt);
        th.step(6500.0 * unit_hash(i, 5));
        const EngineThermalLedger& l = th.ledger();
        worst = std::max(worst, std::abs(l.balance_error) / l.abs_flow_sum);
    }
    INFO("worst relative closure " << worst);
    CHECK(worst <= 1e-9);
    CHECK(th.ledger().jacket_transfer > 0.0);
    // The jacket carries (nearly) all the heat the radiator rejects plus the coolant's storage change.
    CHECK(th.snapshot().jacket_conductance_w_k >= 0.4 * 500.0);
}

TEST_CASE("J4 steady-state solver equals a long transient, with and without a water jacket", "[thermal][drivetrain]") {
    for (const bool jacket : {false, true}) {
      for (const bool open_regime : {false, true}) {
        // Proportional regime: the stable thermostat, partly open. Open regime: the shipped thermostat, held fully
        // open by a heat load the radiator only just rejects, with the fan moved out of reach.
        EngineThermalConfig cfg = config(jacket ? jacket_file() : plain_file());
        if (open_regime) {
            cfg.cooling.fan.on_k = 150.0 + kC;
            cfg.cooling.fan.off_k = 145.0 + kC;
        } else {
            cfg = stable_thermostat(cfg);
        }
        for (const real wall_w : open_regime ? std::initializer_list<real>{35000.0} : std::initializer_list<real>{6000.0, 12000.0}) {
            INFO("jacket " << jacket << " open " << open_regime << " wall " << wall_w);
            SteadyThermalInput in = cruise_input(wall_w);
            in.thermostat_play_offset_k = 0.0;
            if (open_regime) in.ram_speed_mps = 5.0; // less air: the radiator needs the thermostat fully open
            const SteadyThermalResult s = solve_steady_thermal(cfg, in);
            REQUIRE(s.bounded);
            EngineThermal th(cfg);
            th.set_env(ThermalEnv{in.ambient_k, in.ambient_pressure_pa, in.ram_speed_mps, in.engine_rpm});
            run(th, in.engine_rpm, in.wall_heat_w, in.friction_heat_w, 4.0 * 3600.0, 1.0);
            const EngineThermalSnapshot& snap = th.snapshot();
            INFO("steady coolant " << s.coolant_c << " metal " << s.metal_c << " oil " << s.oil_c << " x "
                                   << s.thermostat_opening << " fan " << s.fan_on);
            INFO("transient coolant " << snap.coolant_c << " metal " << snap.metal_c << " oil " << snap.oil_c);
            CHECK(snap.fan_on == s.fan_on);
            CHECK(snap.coolant_c == Approx(s.coolant_c).margin(0.02));
            CHECK(snap.metal_c == Approx(s.metal_c).margin(0.02));
            CHECK(snap.oil_c == Approx(s.oil_c).margin(0.02));
            CHECK(snap.thermostat_opening == Approx(s.thermostat_opening).margin(2e-3));
            CHECK(snap.radiator_heat_w == Approx(s.radiator_w).margin(5.0));
            if (jacket) {
                CHECK(snap.jacket_heat_w == Approx(s.jacket_w).margin(5.0));
            } else {
                CHECK(s.metal_c == s.coolant_c);
                CHECK(s.jacket_w == 0.0);
            }
            if (open_regime) CHECK(s.thermostat_opening == 1.0);
            // Heat out = heat in at the balance.
            CHECK(s.radiator_w + s.block_air_w + s.oil_air_w == Approx(in.wall_heat_w + in.friction_heat_w).epsilon(1e-6));
        }
      }
    }
    // No heat: everything at ambient. Too much heat for the radiator: reported unbounded.
    const EngineThermalConfig cfg = config(jacket_file());
    SteadyThermalInput cold;
    cold.ambient_k = 10.0 + kC;
    const SteadyThermalResult c = solve_steady_thermal(cfg, cold);
    CHECK(c.bounded);
    CHECK(c.coolant_c == Approx(10.0).margin(1e-9));
    CHECK(c.metal_c == Approx(10.0).margin(1e-9));
    SteadyThermalInput hot = cruise_input(5.0e6);
    hot.engine_rpm = 0.0; // pump stopped: no radiator flow
    CHECK_FALSE(solve_steady_thermal(cfg, hot).bounded);
}

TEST_CASE("J5 neutral values of the new keys are bit-identical to their absence", "[thermal][drivetrain]") {
    EngineThermalConfig a = config(plain_file());
    EngineThermalConfig b = a;
    b.cooling.pump.flow_factor = 1.0; // explicit neutral pump
    b.engine.water_jacket = EngineThermalDesc::WaterJacket{}; // explicitly disabled block (fields ignored)
    b.engine.water_jacket.wetted_area_m2 = 3.0;
    EngineThermal ta(a), tb(b);
    CHECK_FALSE(ta.has_water_jacket());
    CHECK_FALSE(tb.has_water_jacket());
    const real dt = 1.0 / 240.0;
    for (std::uint64_t i = 0; i < 5000; ++i) {
        const ThermalEnv env{283.15, 101325.0, 30.0 * unit_hash(i, 7), 0.0};
        ta.set_env(env);
        tb.set_env(env);
        for (EngineThermal* t : {&ta, &tb}) {
            t->begin_tick();
            t->accumulate(1.5e5 * unit_hash(i, 8) * dt, 8.0e3 * unit_hash(i, 9) * dt, dt);
            t->step(6000.0 * unit_hash(i, 10));
        }
        REQUIRE(ta.fold_state_hash(0x1234) == tb.fold_state_hash(0x1234));
    }
    CHECK(ta.metal_temperature_k() == ta.engine_temperature_k());
    CHECK(ta.snapshot().metal_c == ta.snapshot().coolant_c);
    CHECK(ta.snapshot().jacket_heat_w == 0.0);
    // A jacket changes the hash (T_metal is folded and the dynamics differ).
    EngineThermal tj(config(jacket_file()));
    CHECK(tj.fold_state_hash(0x1234) != EngineThermal(config(plain_file())).fold_state_hash(0x1234));
}

namespace {

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

Rig make_rig(bool jacket, int workers) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(kRoot + "/tests/support/fixtures/vehicles/car_sedan_thermal.json");
    REQUIRE(desc.powertrain.thermal.has_value());
    desc.powertrain.thermal->engine = io::load_engine_thermal_json(jacket ? jacket_file() : plain_file());
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
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
    const BodyId body = rig.world->create_body(chassis);
    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(kRoot + "/data/surfaces/surfaces.json"));
    rig.vehicle = rig.world->create_vehicle(desc, body);
    rig.world->set_control("assist.auto_clutch", 1.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("throttle", 0.3);
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

} // namespace

TEST_CASE("J6 World: a water-jacket vehicle is worker-count invariant, hashed and allocation-free",
          "[thermal][drivetrain][determinism]") {
    std::uint64_t h1 = 0, h10 = 0, h_plain = 0;
    for (const int workers : {1, 10}) {
        Rig rig = make_rig(true, workers);
        for (int t = 0; t < 600; ++t) rig.world->step();
        (workers == 1 ? h1 : h10) = rig.world->state_hash();
        const EngineThermalSnapshot& s = rig.world->powertrain_state(rig.vehicle).thermal;
        CHECK(s.has_thermal);
        CHECK(s.metal_c > s.coolant_c); // a running engine heats the metal through the jacket
        CHECK(s.jacket_conductance_w_k > 0.0);
        CHECK(s.coolant_flow_l_min > 0.0);
    }
    {
        Rig plain = make_rig(false, 1);
        for (int t = 0; t < 600; ++t) plain.world->step();
        h_plain = plain.world->state_hash();
        const EngineThermalSnapshot& s = plain.world->powertrain_state(plain.vehicle).thermal;
        CHECK(s.metal_c == s.coolant_c);
    }
    CHECK(h1 == h10);
    CHECK(h1 != h_plain);

    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook unavailable in this build - allocation check skipped");
        return;
    }
    Rig rig = make_rig(true, 2);
    for (int t = 0; t < 240; ++t) rig.world->step();
    namespace da = ps::jobs::debug_alloc;
    da::reset();
    da::set_armed(true);
    rig.world->set_control("throttle", 0.6);
    for (int t = 0; t < 480; ++t) rig.world->step();
    da::set_armed(false);
    CHECK(da::own_violation_count() == 0);
}

TEST_CASE("J7 every shipped engine has a water jacket; the jacket channels appear in the telemetry",
          "[thermal][drivetrain][io]") {
    // Owner ruling 2026-10-16: every shipped engine gets a jacket. The class file carries it; every engine file
    // extends the class file. A loadable file is one with a displacement (the class file alone is not).
    int engines = 0;
    for (const auto& e : std::filesystem::directory_iterator(kRoot + "/data/engine_thermal")) {
        if (e.path().extension() != ".json") continue;
        std::ifstream in(e.path());
        const json j = json::parse(in);
        if (!j.contains("displacement_l") && !j.contains("extends")) continue; // a part file meant to be extended
        const EngineThermalDesc d = io::load_engine_thermal_json(e.path().string());
        INFO(e.path().filename().string());
        CHECK(d.water_jacket.enabled);
        CHECK(d.water_jacket.wetted_area_m2 > 0.0);
        ++engines;
    }
    CHECK(engines >= 2);
    CHECK_FALSE(io::load_engine_thermal_json(plain_file()).water_jacket.enabled);

    auto channel = [](World& w, const std::string& name) {
        const auto& names = w.telemetry().channel_names();
        return std::find(names.begin(), names.end(), name) != names.end();
    };
    Rig jacket = make_rig(true, 1);
    Rig plain = make_rig(false, 1);
    for (const char* n : {"vehicle0.thermal.metal_c", "vehicle0.thermal.jacket_heat_w",
                          "vehicle0.thermal.coolant_flow_l_min"}) {
        CHECK(channel(*jacket.world, n));
        CHECK_FALSE(channel(*plain.world, n)); // a jacket-less vehicle keeps its old CSV header
    }
    CHECK(channel(*plain.world, "vehicle0.thermal.coolant_c"));
}

TEST_CASE("J diag steady temperatures of the worked example", "[.][diag][thermal]") {
    struct Variant {
        const char* name;
        bool jacket;
        real area, htc, pump;
    };
    const Variant variants[] = {{"no jacket", false, 1, 1, 1},   {"jacket", true, 1, 1, 1},
                                {"jacket area x2", true, 2, 1, 1}, {"jacket htc x2", true, 1, 2, 1},
                                {"jacket pump x1.5", true, 1, 1, 1.5}, {"no jacket pump x1.5", false, 1, 1, 1.5}};
    struct Load {
        const char* name;
        SteadyThermalInput in;
    };
    SteadyThermalInput hot = cruise_input(35000.0);
    hot.ram_speed_mps = 5.0;
    hot.engine_rpm = 5000.0;
    const Load loads[] = {{"cruise 3000 rpm, wall 12 kW, friction 3 kW, 25 C, 25 m/s", cruise_input()},
                          {"hot 5000 rpm, wall 35 kW, friction 3 kW, 25 C, 5 m/s", hot}};
    // Shipped A25A (2.487 L) at a light cruise and at full load near the power peak.
    {
        SteadyThermalInput a_cruise = cruise_input(8000.0);
        a_cruise.engine_rpm = 2000.0;
        a_cruise.friction_heat_w = 1500.0;
        SteadyThermalInput a_hard = cruise_input(45000.0);
        a_hard.engine_rpm = 6000.0;
        a_hard.friction_heat_w = 8000.0;
        a_hard.ram_speed_mps = 40.0;
        const std::string a25a = kRoot + "/data/engine_thermal/toyota_a25a_fks.json";
        for (const auto& [name, in] : {std::pair<const char*, SteadyThermalInput>{"A25A cruise 2000 rpm, wall 8 kW, friction 1.5 kW, 25 m/s", a_cruise},
                                       std::pair<const char*, SteadyThermalInput>{"A25A hard 6000 rpm, wall 45 kW, friction 8 kW, 40 m/s", a_hard}}) {
            const SteadyThermalResult r = solve_steady_thermal(config(a25a), in);
            WARN(name << ": coolant " << r.coolant_c << " metal " << r.metal_c << " oil " << r.oil_c << " x "
                      << r.thermostat_opening << " fan " << r.fan_on << " G_j " << r.jacket_conductance_w_k
                      << " bounded " << r.bounded);
        }
    }
    for (const Load& load : loads) {
        WARN(load.name);
        for (const Variant& v : variants) {
            EngineThermalConfig c = config(v.jacket ? jacket_file() : plain_file());
            c.engine.water_jacket.wetted_area_m2 *= v.area;
            c.engine.water_jacket.htc_ref_w_m2_k *= v.htc;
            c.cooling.pump.flow_factor = v.pump;
            const SteadyThermalResult r = solve_steady_thermal(c, load.in);
            WARN(v.name << ": coolant " << r.coolant_c << " metal " << r.metal_c << " oil " << r.oil_c << " x "
                        << r.thermostat_opening << " fan " << r.fan_on << (r.fan_cycling ? " (cycling)" : "")
                        << " G_j " << r.jacket_conductance_w_k << " jacket " << r.jacket_w << " W bounded "
                        << r.bounded);
        }
    }
}
