// tests/unit/test_engine_thermal_io.cpp - engine thermal plan T1 test I8 (loader).
// Tags [thermal][drivetrain][io].
//
// Covers physics_sim.engine_thermal/1 and physics_sim.cooling_system/1 loading
// (core/src/io/engine_thermal_io.cpp), the vehicle keys cooling_system /
// engine_thermal_start / default_thermal / the component's thermal override
// (core/src/io/vehicle_io.cpp), the scenario override, the extends user-last
// rule, and that default_thermal changes neither the generated engine's map
// content hash nor its C16b cache key.
//
// Windows ASan caveat (CLAUDE.md): assertions use REQUIRE_THROWS_WITH, never a
// manual catch that touches e.what().
//
// Sabotage record (each applied once to the loader, the named case then fails):
//   - "I8 engine thermal rejections": the friction_scale_max rejection disabled (the key stays accepted) -> the
//     first case ("friction_scale_max", "NO friction cap") fails with no exception thrown.
//   - "I8 vehicle opt-in keys": the "cooling_system needs an engine thermal file" check disabled -> the section
//     "cooling_system without an engine thermal file throws" fails (the load then dies later with ": cannot open
//     file", which does not contain the expected message).
//   - "I8 shipped engine thermal and cooling files load with expanded values": thermostat open_c converted with
//     +273.0 instead of +273.15 -> open_k reads 361.0 against 361.15 (bound 1e-12 relative).

#include "engine_cycle/map_cache.h"

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/engine_thermal_desc.h"
#include "ps/io/engine_thermal_io.h"
#include "ps/io/fuel_io.h"
#include "ps/io/scenario.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <variant>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;

namespace {

const std::string kRoot = PS_SOURCE_DIR;

struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() / ("ps_test_engine_thermal_io_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
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

json read_json(const std::string& path) {
    std::ifstream in(path);
    json j;
    in >> j;
    return j;
}

json valid_engine_thermal() {
    return json::parse(R"({
  "format": "physics_sim.engine_thermal/1",
  "source": "test fixture",
  "displacement_l": 2.0,
  "capacity_per_l": { "engine_j_k": 26600.0, "oil_j_k": 7600.0 },
  "wall_heat_fraction": 0.20,
  "friction_to_oil_fraction": 0.30,
  "motoring_friction_fraction": 0.6,
  "oil": { "kv40_mm2_s": 69.1, "kv100_mm2_s": 12.0, "density_15c_kg_m3": 851.0,
           "reference_temperature_c": 95.0, "hydrodynamic_fraction": 0.5, "hydrodynamic_reference_rpm": 2000.0,
           "friction_scale_min": 1.0 },
  "conductance_per_l": { "oil_coolant_time_constant_s": 300.0, "block_air_w_k": 7.5, "block_air_w_k_per_m_s": 0.75,
                         "oil_air_w_k": 2.5, "oil_air_w_k_per_m_s": 0.25 },
  "coolant": { "cp_j_kg_k": 3400.0, "density_kg_m3": 1065.0 },
  "pump": { "l_min_per_krpm_per_l": 10.0 },
  "thermostat": { "open_c": 88.0, "full_open_c": 100.0, "hysteresis_k": 2.0, "time_constant_s": 20.0 }
})");
}

json valid_cooling() {
    return json::parse(R"({
  "format": "physics_sim.cooling_system/1",
  "source": "test fixture",
  "radiator": { "frontal_area_m2": 0.35, "ram_coefficient": 0.35, "ua_ref_w_k": 1500.0, "air_mass_flow_ref_kg_s": 2.0, "ua_exponent": 0.6 },
  "fan": { "face_velocity_m_s": 3.5, "on_c": 98.0, "off_c": 93.0, "power_w": 300.0 },
  "oil_cooler": { "air_w_k_per_m_s": 0.0 },
  "gauge": { "x_c": [40, 75, 105, 125], "y": [0, 0.5, 0.5, 1.0] },
  "warnings": { "overheat_c": 118.0, "cold_c": 50.0 }
})");
}

using Mutator = std::function<void(json&)>;

void check_engine_bad(const char* what, const Mutator& m, const char* message) {
    INFO(what);
    TempDir t;
    json j = valid_engine_thermal();
    m(j);
    const std::string p = t.write("bad.json", j);
    REQUIRE_THROWS_WITH(io::load_engine_thermal_json(p), ContainsSubstring(message));
}

void check_cooling_bad(const char* what, const Mutator& m, const char* message) {
    INFO(what);
    TempDir t;
    json j = valid_cooling();
    m(j);
    const std::string p = t.write("bad.json", j);
    REQUIRE_THROWS_WITH(io::load_cooling_system_json(p), ContainsSubstring(message));
}

// A vehicle file copy whose relative paths are made absolute so it can live in a temp directory.
json absolutised_vehicle(const std::string& name) {
    const std::string path = kRoot + "/data/vehicles/" + name;
    json v = read_json(path);
    const std::filesystem::path base = std::filesystem::path(path).parent_path();
    auto absolutise = [&](json& node, const char* key) {
        if (node.contains(key) && node.at(key).is_string()) {
            node[key] = (base / node.at(key).get<std::string>()).lexically_normal().string();
        }
    };
    for (auto& w : v["wheels"]) absolutise(w, "tyre");
    for (auto& c : v["powertrain"]["components"]) absolutise(c, "ref");
    absolutise(v, "cooling_system");
    return v;
}

} // namespace

TEST_CASE("I8 shipped engine thermal and cooling files load with expanded values", "[thermal][drivetrain][io]") {
    const EngineThermalDesc a25a = io::load_engine_thermal_json(kRoot + "/data/engine_thermal/toyota_a25a_fks.json");
    CHECK(a25a.displacement_l == Approx(2.487));
    CHECK(a25a.engine_capacity_j_k == Approx(26600.0 * 2.487).epsilon(1e-12));
    CHECK(a25a.oil_capacity_j_k == Approx(7600.0 * 2.487).epsilon(1e-12));
    CHECK(a25a.block_air_w_k == Approx(7.5 * 2.487).epsilon(1e-12));
    CHECK(a25a.oil_air_w_k_per_m_s == Approx(0.25 * 2.487).epsilon(1e-12));
    CHECK(a25a.oil_coolant_time_constant_s == 300.0); // never scaled by displacement
    CHECK(a25a.pump_l_min_per_krpm == Approx(10.0 * 2.487).epsilon(1e-12));
    CHECK(a25a.wall_heat_fraction == 0.20);
    CHECK(a25a.oil.reference_temperature_k == Approx(95.0 + 273.15).epsilon(1e-12));
    CHECK(a25a.thermostat.open_k == Approx(88.0 + 273.15).epsilon(1e-12));
    CHECK(a25a.thermostat.hysteresis_k == 2.0); // a difference, not converted
    CHECK(a25a.oil.friction_scale_min == 1.0);

    const EngineThermalDesc i4 = io::load_engine_thermal_json(kRoot + "/data/engine_thermal/i4_2l_na.json");
    CHECK(i4.displacement_l == 2.0);
    CHECK(i4.engine_capacity_j_k == Approx(53200.0).epsilon(1e-12));
    CHECK(i4.motoring_friction_fraction == 0.6);
    // T1b: the class reference speed, and this engine's own hydrodynamic share (its Heywood correlation's
    // speed-linear term, 0.30 of 1.47 bar at 2000 rpm) overriding the class 0.5.
    CHECK(i4.oil.hydrodynamic_reference_rpm == 2000.0);
    CHECK(i4.oil.hydrodynamic_fraction == 0.2);
    CHECK(a25a.oil.hydrodynamic_reference_rpm == 2000.0);
    CHECK(a25a.oil.hydrodynamic_fraction == 0.5);

    const CoolingSystemDesc cs = io::load_cooling_system_json(kRoot + "/data/cooling_systems/sedan.json");
    CHECK(cs.radiator.ua_ref_w_k == 1500.0);
    CHECK(cs.radiator.ua_exponent == 0.6);
    CHECK(cs.fan.on_k == Approx(98.0 + 273.15).epsilon(1e-12));
    CHECK(cs.fan.off_k == Approx(93.0 + 273.15).epsilon(1e-12));
    REQUIRE(cs.gauge_x_k_y.x.size() == 4);
    CHECK(cs.gauge_x_k_y.x[0] == Approx(40.0 + 273.15).epsilon(1e-12));
    CHECK(cs.warnings.overheat_k == Approx(118.0 + 273.15).epsilon(1e-12));
}

TEST_CASE("I8 engine thermal rejections", "[thermal][drivetrain][io]") {
    check_engine_bad("friction_scale_max", [](json& j) { j["oil"]["friction_scale_max"] = 4.0; }, "NO friction cap");
    check_engine_bad("unknown key", [](json& j) { j["bogus"] = 1; }, "unknown key \"bogus\"");
    check_engine_bad("ecu reserved", [](json& j) { j["ecu"] = json::object(); }, "reserved for engine thermal increment T2");
    check_engine_bad("missing source", [](json& j) { j.erase("source"); }, "missing required field \"source\"");
    check_engine_bad("wrong format", [](json& j) { j["format"] = "physics_sim.engine_thermal/2"; }, "unsupported 'format'");
    check_engine_bad("both capacity forms", [](json& j) { j["capacity"] = {{"engine_j_k", 1.0}, {"oil_j_k", 1.0}}; },
                     "exactly one of \"capacity\" and \"capacity_per_l\"");
    check_engine_bad("no capacity form", [](json& j) { j.erase("capacity_per_l"); }, "exactly one of \"capacity\" and \"capacity_per_l\"");
    check_engine_bad("per-litre without displacement", [](json& j) { j.erase("displacement_l"); }, "requires \"displacement_l\" > 0");
    check_engine_bad("zero displacement", [](json& j) { j["displacement_l"] = 0.0; }, "\"displacement_l\" must be > 0");
    check_engine_bad("both conductance forms", [](json& j) { j["conductance"] = json::object(); },
                     "exactly one of \"conductance\" and \"conductance_per_l\"");
    check_engine_bad("both pump forms", [](json& j) { j["pump"]["l_min_per_krpm"] = 5.0; },
                     "exactly one of \"l_min_per_krpm\" and \"l_min_per_krpm_per_l\"");
    check_engine_bad("kv40 <= kv100", [](json& j) { j["oil"]["kv40_mm2_s"] = 11.0; }, "kv40_mm2_s > kv100_mm2_s > 0");
    check_engine_bad("rho15", [](json& j) { j["oil"]["density_15c_kg_m3"] = 0.0; }, "\"density_15c_kg_m3\" must be > 0");
    check_engine_bad("f_wall too big", [](json& j) { j["wall_heat_fraction"] = 0.5; }, "\"wall_heat_fraction\" must be in (0, 0.5)");
    check_engine_bad("f_wall zero", [](json& j) { j["wall_heat_fraction"] = 0.0; }, "\"wall_heat_fraction\" must be in (0, 0.5)");
    check_engine_bad("beta > 1", [](json& j) { j["friction_to_oil_fraction"] = 1.1; }, "\"friction_to_oil_fraction\" must be in [0, 1]");
    check_engine_bad("phi_f zero", [](json& j) { j["motoring_friction_fraction"] = 0.0; }, "\"motoring_friction_fraction\" must be in (0, 1]");
    check_engine_bad("phi_h zero", [](json& j) { j["oil"]["hydrodynamic_fraction"] = 0.0; }, "\"hydrodynamic_fraction\" must be in (0, 1]");
    check_engine_bad("n_ref low", [](json& j) { j["oil"]["hydrodynamic_reference_rpm"] = 400.0; }, "\"hydrodynamic_reference_rpm\" must be in [500, 10000]");
    check_engine_bad("n_ref missing", [](json& j) { j["oil"].erase("hydrodynamic_reference_rpm"); }, "missing required field \"hydrodynamic_reference_rpm\"");
    check_engine_bad("k_min < 1", [](json& j) { j["oil"]["friction_scale_min"] = 0.9; }, "\"friction_scale_min\" must be >= 1");
    check_engine_bad("T_ref low", [](json& j) { j["oil"]["reference_temperature_c"] = 50.0; }, "\"reference_temperature_c\" must be in [60, 140]");
    check_engine_bad("T_ref high", [](json& j) { j["oil"]["reference_temperature_c"] = 150.0; }, "\"reference_temperature_c\" must be in [60, 140]");
    check_engine_bad("open >= full", [](json& j) { j["thermostat"]["open_c"] = 100.0; }, "requires open_c < full_open_c");
    check_engine_bad("hysteresis < 0", [](json& j) { j["thermostat"]["hysteresis_k"] = -1.0; }, "\"hysteresis_k\" must be >= 0");
    check_engine_bad("wax tau", [](json& j) { j["thermostat"]["time_constant_s"] = 0.0; }, "\"time_constant_s\" must be > 0");
    check_engine_bad("tau_oc", [](json& j) { j["conductance_per_l"]["oil_coolant_time_constant_s"] = 0.0; },
                     "\"oil_coolant_time_constant_s\" must be > 0");
    check_engine_bad("negative conductance", [](json& j) { j["conductance_per_l"]["block_air_w_k"] = -1.0; }, "conductances must be >= 0");
    check_engine_bad("coolant cp", [](json& j) { j["coolant"]["cp_j_kg_k"] = 0.0; }, "coolant cp and density must be > 0");
}

TEST_CASE("I8 cooling system rejections", "[thermal][drivetrain][io]") {
    check_cooling_bad("unknown key", [](json& j) { j["bogus"] = 1; }, "unknown key \"bogus\"");
    check_cooling_bad("heater reserved", [](json& j) { j["heater_core"] = json::object(); }, "reserved for engine thermal increment T7");
    check_cooling_bad("off >= on", [](json& j) { j["fan"]["off_c"] = 98.0; }, "requires off_c < on_c");
    check_cooling_bad("ram coefficient", [](json& j) { j["radiator"]["ram_coefficient"] = 1.5; }, "\"ram_coefficient\" must be in [0, 1]");
    check_cooling_bad("gauge decreasing", [](json& j) { j["gauge"]["y"] = {0, 0.6, 0.5, 1.0}; }, "\"y\" must be non-decreasing");
    check_cooling_bad("gauge out of range", [](json& j) { j["gauge"]["y"] = {0, 0.5, 0.5, 1.5}; }, "\"y\" must be within [0, 1]");
    check_cooling_bad("gauge x order", [](json& j) { j["gauge"]["x_c"] = {40, 75, 75, 125}; }, "\"x_c\" must be strictly increasing");
    check_cooling_bad("gauge length", [](json& j) { j["gauge"]["y"] = {0, 1.0}; }, "same length");
    check_cooling_bad("cold >= overheat", [](json& j) { j["warnings"]["cold_c"] = 120.0; }, "requires cold_c < overheat_c");
    check_cooling_bad("ua_ref", [](json& j) { j["radiator"]["ua_ref_w_k"] = 0.0; }, "must be > 0");
}

TEST_CASE("I8 extends chain: a user file overrides only what it names", "[thermal][drivetrain][io]") {
    TempDir t;
    json user = {{"format", "physics_sim.engine_thermal/1"},
                 {"extends", kRoot + "/data/engine_thermal/toyota_a25a_fks.json"},
                 {"source", "user override"},
                 {"thermostat", {{"open_c", 82.0}}}};
    const std::string p = t.write("user.json", user);
    const EngineThermalDesc base = io::load_engine_thermal_json(kRoot + "/data/engine_thermal/toyota_a25a_fks.json");
    const EngineThermalDesc d = io::load_engine_thermal_json(p);
    CHECK(d.thermostat.open_k == Approx(82.0 + 273.15).epsilon(1e-12));
    CHECK(d.thermostat.full_open_k == base.thermostat.full_open_k);
    CHECK(d.engine_capacity_j_k == base.engine_capacity_j_k);
    CHECK(d.oil.kv40_mm2_s == base.oil.kv40_mm2_s);
    CHECK(d.source == "user override");

    json bad_user = user;
    bad_user["thermostat"]["open_c"] = 105.0;
    REQUIRE_THROWS_WITH(io::load_engine_thermal_json(t.write("bad_user.json", bad_user)), ContainsSubstring("requires open_c < full_open_c"));
}

TEST_CASE("I8 vehicle opt-in keys", "[thermal][drivetrain][io]") {
    SECTION("shipped thermal vehicles opt in, shipped plain vehicles do not") {
        const auto sim = io::load_vehicle_json(kRoot + "/data/vehicles/car_sedan_sim_thermal.json");
        REQUIRE(sim.powertrain.thermal.has_value());
        CHECK(sim.powertrain.thermal->start == ThermalStartKind::Warm);
        CHECK(sim.powertrain.thermal->engine.displacement_l == Approx(2.487));
        const auto graph = io::load_vehicle_json(kRoot + "/data/vehicles/car_sedan_thermal.json");
        REQUIRE(graph.powertrain.thermal.has_value());
        CHECK(graph.powertrain.thermal->engine.displacement_l == 2.0);
        CHECK(graph.powertrain.thermal->engine.motoring_friction_fraction == 0.6);
        CHECK_FALSE(io::load_vehicle_json(kRoot + "/data/vehicles/car_sedan.json").powertrain.thermal.has_value());
        CHECK_FALSE(io::load_vehicle_json(kRoot + "/data/vehicles/car_sedan_sim.json").powertrain.thermal.has_value());
    }
    SECTION("engine_thermal_start ambient") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan_thermal.json");
        v["engine_thermal_start"] = "ambient";
        const auto d = io::load_vehicle_json(t.write("v.json", v));
        REQUIRE(d.powertrain.thermal.has_value());
        CHECK(d.powertrain.thermal->start == ThermalStartKind::Ambient);
    }
    SECTION("bad engine_thermal_start value") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan_thermal.json");
        v["engine_thermal_start"] = "hot";
        REQUIRE_THROWS_WITH(io::load_vehicle_json(t.write("v.json", v)), ContainsSubstring("must be \"warm\" or \"ambient\""));
    }
    SECTION("engine_thermal_start without a cooling system") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan.json");
        v["engine_thermal_start"] = "warm";
        REQUIRE_THROWS_WITH(io::load_vehicle_json(t.write("v.json", v)), ContainsSubstring("needs a 'cooling_system'"));
    }
    SECTION("cooling_system without an engine thermal file throws") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan_thermal.json");
        v["powertrain"]["components"][0]["ref"] = kRoot + "/data/engines/toyota_a25a_fks.json"; // has no default_thermal
        REQUIRE_THROWS_WITH(io::load_vehicle_json(t.write("v.json", v)), ContainsSubstring("needs an engine thermal file"));
    }
    SECTION("the component's thermal override replaces default_thermal") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan_thermal.json");
        v["powertrain"]["components"][0]["thermal"] = kRoot + "/data/engine_thermal/toyota_a25a_fks.json";
        const auto d = io::load_vehicle_json(t.write("v.json", v));
        REQUIRE(d.powertrain.thermal.has_value());
        CHECK(d.powertrain.thermal->engine.displacement_l == Approx(2.487));
    }
    SECTION("the component's thermal override makes an engine without default_thermal usable") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan_thermal.json");
        v["powertrain"]["components"][0]["ref"] = kRoot + "/data/engines/toyota_a25a_fks.json";
        v["powertrain"]["components"][0]["thermal"] = kRoot + "/data/engine_thermal/toyota_a25a_fks.json";
        CHECK(io::load_vehicle_json(t.write("v.json", v)).powertrain.thermal.has_value());
    }
    SECTION("an empty thermal override path is rejected") {
        TempDir t;
        json v = absolutised_vehicle("car_sedan_thermal.json");
        v["powertrain"]["components"][0]["thermal"] = "";
        REQUIRE_THROWS_WITH(io::load_vehicle_json(t.write("v.json", v)), ContainsSubstring("'thermal' must be a non-empty path string"));
    }
    SECTION("an engine's default_thermal without a cooling system changes nothing") {
        CHECK_FALSE(io::load_vehicle_json(kRoot + "/data/vehicles/car_sedan.json").powertrain.thermal.has_value());
    }
}

TEST_CASE("I8 default_thermal changes neither the generated engine's map content hash nor its cache key", "[thermal][drivetrain][io][cycle]") {
    const std::string engine_path = kRoot + "/data/engines/toyota_a25a_fks_gen.json";
    json with = read_json(engine_path);
    REQUIRE(with.contains("default_thermal"));
    json without = with;
    without.erase("default_thermal");
    // The copy lives in a temp directory: its relative fuel paths are made absolute.
    for (auto& f : without["fuels"]) f = (std::filesystem::path(engine_path).parent_path() / f.get<std::string>()).lexically_normal().string();
    for (auto& f : with["fuels"]) f = (std::filesystem::path(engine_path).parent_path() / f.get<std::string>()).lexically_normal().string();
    TempDir t;
    const std::string p_with = t.write("with.json", with);
    const std::string p_without = t.write("without.json", without);

    io::EngineMapOptions opt;
    opt.quality = MapQuality::Preview;
    const ComponentParams a = io::load_engine_json(p_with, opt);
    const ComponentParams b = io::load_engine_json(p_without, opt);
    const auto* sa = std::get_if<SimulatedEngineDesc>(&a);
    const auto* sb = std::get_if<SimulatedEngineDesc>(&b);
    REQUIRE(sa != nullptr);
    REQUIRE(sb != nullptr);
    REQUIRE_FALSE(sa->maps.ve.values.empty());
    CHECK(sa->maps.ve.values == sb->maps.ve.values);
    CHECK(sa->maps.eta_i.values == sb->maps.eta_i.values);
    CHECK(sa->maps.theta_mbt_deg.values == sb->maps.theta_mbt_deg.values);
    CHECK(sa->maps.friction_torque_nm.y == sb->maps.friction_torque_nm.y);
    REQUIRE(sa->cycle.has_value());
    REQUIRE(sb->cycle.has_value());
    const FuelDesc fuel = io::load_fuel_json(std::filesystem::path(with["fuels"][0].get<std::string>()).string());
    CHECK(engine_map_cache_key(*sa->cycle, fuel, MapQuality::Preview) == engine_map_cache_key(*sb->cycle, fuel, MapQuality::Preview));
}

TEST_CASE("I8 scenario engine_thermal_start overrides the vehicle file", "[thermal][drivetrain][io]") {
    TempDir t;
    // A scenario needs a body for the chassis only at build time; load_scenario parses the vehicle immediately.
    json veh = absolutised_vehicle("car_sedan_thermal.json");
    const std::string vpath = t.write("veh.json", veh);
    json sc = {{"format", "physics_sim.scenario/1"}, {"name", "thermal_io"}, {"duration_s", 1.0},
               {"bodies", json::array()},
               {"vehicles", json::array({{{"name", "car"}, {"chassis_body", "chassis"}, {"vehicle_json_path", vpath},
                                          {"engine_thermal_start", "ambient"}}})}};
    const auto loaded = io::load_scenario(t.write("sc.json", sc));
    REQUIRE(loaded.vehicles.size() == 1);
    REQUIRE(loaded.vehicles[0].desc.powertrain.thermal.has_value());
    CHECK(loaded.vehicles[0].desc.powertrain.thermal->start == ThermalStartKind::Ambient);
    CHECK(loaded.vehicles[0].engine_thermal_start == "ambient");

    SECTION("without a cooling system it is an error") {
        json plain = absolutised_vehicle("car_sedan.json");
        sc["vehicles"][0]["vehicle_json_path"] = t.write("plain.json", plain);
        REQUIRE_THROWS_WITH(io::load_scenario(t.write("sc2.json", sc)), ContainsSubstring("needs a vehicle file with a 'cooling_system'"));
    }
}
