// tests/unit/test_simulated_engine_io.cpp -- PHYS-033 P4 I7b C10: the
// "kind": "simulated" engine schema (ps/drivetrain/powertrain_desc.h's
// SimulatedEngineDesc) and its loader (core/src/io/vehicle_io.cpp's
// read_simulated_fields), docs/data_formats/engine.md's "simulated" section.
//
// Every fixture is built in memory and written to a scratch temp file; no
// data/** file is read except data/fuels/gasoline.json (the fuel reference).
// One valid electronic and one valid mechanical engine load with every unit
// conversion checked against literal SI values; every other SECTION breaks
// exactly one rule of a copy of the valid electronic engine and pins the
// loader's message.

#include "drivetrain/engine_simulated.h" // white-box: C12 builds a SimulatedEngine
#include "drivetrain/powertrain.h"       // white-box

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_desc.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <variant>

namespace {

using Catch::Matchers::ContainsSubstring;
using ps::real;
using namespace ps::drivetrain;
using json = nlohmann::json;

struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_sim_engine_io_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::create_directories(dir);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    std::filesystem::path write(const std::string& name, const json& j) const {
        const auto p = dir / name;
        std::ofstream out(p);
        out << j.dump(2);
        return p;
    }
};

std::string gasoline_path() {
    return std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string();
}

// A 2.0 L naturally aspirated I4 (bore = stroke = 86 mm) with synthetic
// (test-only, not physical-data) maps. Electronic throttle.
json base_simulated_engine() {
    json j;
    j["id"] = "engine";
    j["type"] = "engine";
    j["kind"] = "simulated";
    j["architecture"] = "piston_4stroke";
    j["bore_mm"] = 86.0;
    j["stroke_mm"] = 86.0;
    j["cylinders"] = 4;
    j["intake_manifold_volume_l"] = 3.0;
    j["throttle_body"] = {{"diameter_mm", 60.0},
                          {"count", 1},
                          {"discharge_coefficient", 0.8},
                          {"area_fraction_vs_position", {{"x", {0.0, 0.5, 1.0}}, {"y", {0.01, 0.4, 1.0}}}}};
    j["throttle_input"] = "electronic";
    j["throttle_actuator_tau_s"] = 0.04;
    j["idle_control"] = {{"kp_per_rpm", 0.001}, {"ki_per_rpm_s", 0.004}, {"max_throttle", 0.2}};
    j["ecu"] = {{"lambda_target", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{1.0, 1.0}, {1.0, 0.9}}}}},
                {"spark_advance_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}}};
    j["maps"] = {
        {"source", "synthetic test fixture (test_simulated_engine_io.cpp), not physical data"},
        {"ve", {{"rpm", {0.0, 3000.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{0.80, 0.85}, {0.88, 0.95}, {0.85, 0.90}}}}},
        {"eta_i", {{"rpm", {0.0, 7000.0}}, {"air_mass_per_rev_g", {0.0, 1.5}}, {"values", {{0.34, 0.37}, {0.36, 0.38}}}}},
        {"f_lambda", {{"x", {0.7, 1.0, 1.5}}, {"y", {0.85, 1.0, 0.9}}}},
        {"eta_spark", {{"x", {-40.0, 0.0, 10.0}}, {"y", {0.5, 1.0, 0.95}}}},
        {"theta_mbt_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}},
        {"friction_torque_nm", {{"x", {0.0, 1000.0, 7000.0}}, {"y", {15.0, 18.0, 40.0}}}}};
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 20.0;
    j["fuel"] = gasoline_path();
    j["ignition"] = "spark";
    return j;
}

json base_mechanical_engine() {
    json j = base_simulated_engine();
    j["throttle_input"] = "mechanical";
    j.erase("throttle_actuator_tau_s");
    j["idle_control"]["bypass_area_mm2"] = 50.0;
    return j;
}

json vehicle_with_components(const json& components) {
    json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_simulated_engine_io.cpp)";
    v["wheels"] = json::array({{{"name", "FL"}, {"tyre", {{"source", "test fixture"}}}}});
    v["powertrain"] = {{"components", components}};
    return v;
}

ps::vehicle::VehicleDesc load_with_engine(const json& engine) {
    TempDir d;
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({engine})));
    return ps::io::load_vehicle_json(p.string());
}

void expect_rejected(const json& engine, const std::string& substring) {
    TempDir d;
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({engine})));
    REQUIRE_THROWS_WITH(ps::io::load_vehicle_json(p.string()), ContainsSubstring(substring));
}

const SimulatedEngineDesc& only_sim_engine(const ps::vehicle::VehicleDesc& d) {
    REQUIRE(d.powertrain.components.size() == 1);
    const auto* e = std::get_if<SimulatedEngineDesc>(&d.powertrain.components[0].params);
    REQUIRE(e != nullptr);
    return *e;
}

} // namespace

TEST_CASE("C10: a valid electronic simulated engine loads with every unit converted to SI",
          "[unit][vehicle][io][drivetrain][engine]") {
    const ps::vehicle::VehicleDesc d = load_with_engine(base_simulated_engine());
    const SimulatedEngineDesc& e = only_sim_engine(d);
    CHECK(e.architecture == EngineArchitecture::Piston4Stroke);
    CHECK(e.piston.bore_m == real(86.0) * real(1e-3));
    CHECK(e.piston.stroke_m == real(86.0) * real(1e-3));
    CHECK(e.piston.cylinders == 4);
    CHECK(e.intake_manifold_volume_m3 == real(3.0) * real(1e-3));
    CHECK(e.throttle_body.diameter_m == real(60.0) * real(1e-3));
    CHECK(e.throttle_body.count == 1);
    CHECK(e.throttle_body.discharge_coefficient == 0.8);
    CHECK(e.throttle_body.area_fraction_vs_position.y.front() == 0.01);
    CHECK(e.throttle_input == ThrottleInput::Electronic);
    CHECK(e.throttle_actuator_tau_s == 0.04);
    CHECK(e.idle_bypass_area_m2 == 0.0);
    // kPa -> Pa on every p_im axis, g -> kg on the eta_i load axis.
    CHECK(e.maps.ve.y == std::vector<real>{20000.0, 100000.0});
    CHECK(e.maps.ve.values == std::vector<real>{0.80, 0.85, 0.88, 0.95, 0.85, 0.90});
    CHECK(e.maps.eta_i.y == std::vector<real>{0.0, real(1.5) * real(1e-3)});
    CHECK(e.maps.theta_mbt_deg.y == std::vector<real>{20000.0, 100000.0});
    CHECK(e.ecu.lambda_target.y == std::vector<real>{20000.0, 100000.0});
    CHECK(e.ecu.lambda_target.value(7000.0, 100000.0) == 0.9);
    CHECK(e.ecu.spark_advance_deg.value(0.0, 20000.0) == 30.0);
    CHECK(e.maps.f_lambda.evaluate(1.0) == 1.0);
    CHECK(e.maps.eta_spark.evaluate(0.0) == 1.0);
    CHECK(e.maps.friction_torque_nm.evaluate(0.0) == 15.0);
    CHECK_FALSE(e.maps.has_pmep_excess);
    CHECK_FALSE(e.maps.has_theta_kl);
    CHECK(e.inertia_kgm2 == 0.15);
    CHECK(e.breakaway_torque_nm == 20.0);
    CHECK(e.idle.kp_per_rpm == 0.001);
    CHECK(e.idle.max_throttle == 0.2);
    REQUIRE(e.fuels.size() == 1);
    CHECK(e.fuels[0].lhv_j_kg == 44.0e6);
    CHECK(e.ignition == EngineIgnition::Spark);
    CHECK(has_fuel_model(e));
}

TEST_CASE("C10: a valid mechanical simulated engine loads with its bypass valve area",
          "[unit][vehicle][io][drivetrain][engine]") {
    const ps::vehicle::VehicleDesc d = load_with_engine(base_mechanical_engine());
    const SimulatedEngineDesc& e = only_sim_engine(d);
    CHECK(e.throttle_input == ThrottleInput::Mechanical);
    CHECK(e.idle_bypass_area_m2 == real(50.0) * real(1e-6));
    CHECK(e.throttle_actuator_tau_s == 0.0);
}

TEST_CASE("C10: the reserved optional maps pmep_excess and theta_kl load in SI", "[unit][vehicle][io][drivetrain][engine]") {
    json j = base_simulated_engine();
    j["maps"]["pmep_excess_kpa"] = {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{2.0, 1.0}, {8.0, 4.0}}}};
    j["maps"]["theta_kl_deg"] = {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{40.0, 15.0}, {45.0, 25.0}}}};
    const ps::vehicle::VehicleDesc d = load_with_engine(j);
    const SimulatedEngineDesc& e = only_sim_engine(d);
    REQUIRE(e.maps.has_pmep_excess);
    CHECK(e.maps.pmep_excess_pa.y == std::vector<real>{20000.0, 100000.0});
    CHECK(e.maps.pmep_excess_pa.values == std::vector<real>{2000.0, 1000.0, 8000.0, 4000.0});
    REQUIRE(e.maps.has_theta_kl);
    CHECK(e.maps.theta_kl_deg.value(0.0, 100000.0) == 15.0);
}

TEST_CASE("C10: a rotary simulated engine loads (runtime is I7e)", "[unit][vehicle][io][drivetrain][engine]") {
    json j = base_simulated_engine();
    j["architecture"] = "rotary";
    j.erase("bore_mm");
    j.erase("stroke_mm");
    j.erase("cylinders");
    j["rotor_radius_mm"] = 105.0;
    j["eccentricity_mm"] = 15.0;
    j["rotor_width_mm"] = 80.0;
    j["rotors"] = 2;
    const ps::vehicle::VehicleDesc d = load_with_engine(j);
    const SimulatedEngineDesc& e = only_sim_engine(d);
    CHECK(e.architecture == EngineArchitecture::Rotary);
    CHECK(e.rotary.rotor_radius_m == real(105.0) * real(1e-3));
    CHECK(e.rotary.eccentricity_m == real(15.0) * real(1e-3));
    CHECK(e.rotary.rotor_width_m == real(80.0) * real(1e-3));
    CHECK(e.rotary.rotors == 2);
}

// PHYS-033 P4 I7b C12 replaces C10's "not yet integrated" refusal: the loaded
// simulated engine is built behind IEngine. The fixture vehicle has only the
// engine and one wheel, so a viscous load_dyno is shafted to the crank (an
// unconnected crank port is rejected by the graph build, not by C12).
ps::vehicle::VehicleDesc with_crank_dyno(ps::vehicle::VehicleDesc d) {
    LoadDynoDesc dyno;
    dyno.mode = LoadDynoMode::Viscous;
    dyno.viscous_nm_s_rad = 1.0;
    d.powertrain.components.push_back(ComponentDesc{"dyno", dyno});
    ShaftDesc s;
    s.ports = {PortRef{"engine", "crank", false}, PortRef{"dyno", "shaft", false}};
    d.powertrain.shafts.push_back(s);
    return d;
}

TEST_CASE("C12: Powertrain builds a loaded simulated engine behind IEngine", "[unit][vehicle][drivetrain][engine]") {
    const ps::vehicle::VehicleDesc d = with_crank_dyno(load_with_engine(base_simulated_engine()));
    std::unique_ptr<Powertrain> pt;
    REQUIRE_NOTHROW(pt = std::make_unique<Powertrain>(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name));
    REQUIRE(pt->has_engine());
    CHECK(dynamic_cast<const SimulatedEngine*>(&pt->engine()) != nullptr);
}

TEST_CASE("C12: a rotary simulated engine is still refused at build (runtime is I7e C13)",
          "[unit][vehicle][drivetrain][engine]") {
    json j = base_simulated_engine();
    j["architecture"] = "rotary";
    j.erase("bore_mm");
    j.erase("stroke_mm");
    j.erase("cylinders");
    j["rotor_radius_mm"] = 105.0;
    j["eccentricity_mm"] = 15.0;
    j["rotor_width_mm"] = 80.0;
    j["rotors"] = 2;
    const ps::vehicle::VehicleDesc d = with_crank_dyno(load_with_engine(j));
    REQUIRE_THROWS_WITH(Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name),
                        ContainsSubstring("rotary architecture is not implemented yet (I7e C13)"));
}

TEST_CASE("C10: the simulated-engine loader rejects every malformed field with its message",
          "[unit][vehicle][io][drivetrain][engine]") {
    // --- geometry ---
    SECTION("missing architecture") {
        json j = base_simulated_engine();
        j.erase("architecture");
        expect_rejected(j, "'architecture' is required (\"piston_4stroke\" or \"rotary\")");
    }
    SECTION("unknown architecture") {
        json j = base_simulated_engine();
        j["architecture"] = "two_stroke";
        expect_rejected(j, "'architecture' must be \"piston_4stroke\" or \"rotary\", got \"two_stroke\"");
    }
    SECTION("a rotary key on a piston engine") {
        json j = base_simulated_engine();
        j["rotors"] = 2;
        expect_rejected(j, "'rotors' is a rotary field, not allowed on a piston_4stroke engine");
    }
    SECTION("a piston key on a rotary engine") {
        json j = base_simulated_engine();
        j["architecture"] = "rotary";
        expect_rejected(j, "'bore_mm' is a piston field, not allowed on a rotary engine");
    }
    SECTION("rotary eccentricity >= rotor radius") {
        json j = base_simulated_engine();
        j["architecture"] = "rotary";
        j.erase("bore_mm");
        j.erase("stroke_mm");
        j.erase("cylinders");
        j["rotor_radius_mm"] = 15.0;
        j["eccentricity_mm"] = 15.0;
        j["rotor_width_mm"] = 80.0;
        j["rotors"] = 2;
        expect_rejected(j, "eccentricity_mm must be < rotor_radius_mm");
    }
    SECTION("missing bore") {
        json j = base_simulated_engine();
        j.erase("bore_mm");
        expect_rejected(j, "'bore_mm' is required");
    }
    SECTION("non-positive stroke") {
        json j = base_simulated_engine();
        j["stroke_mm"] = 0.0;
        expect_rejected(j, "'stroke_mm' must be > 0");
    }
    SECTION("non-integer cylinders") {
        json j = base_simulated_engine();
        j["cylinders"] = 4.5;
        expect_rejected(j, "'cylinders' must be an integer");
    }
    SECTION("zero cylinders") {
        json j = base_simulated_engine();
        j["cylinders"] = 0;
        expect_rejected(j, "'cylinders' must be in [1, 64]");
    }
    SECTION("missing manifold volume") {
        json j = base_simulated_engine();
        j.erase("intake_manifold_volume_l");
        expect_rejected(j, "'intake_manifold_volume_l' is required");
    }
    // --- throttle body ---
    SECTION("missing throttle_body") {
        json j = base_simulated_engine();
        j.erase("throttle_body");
        expect_rejected(j, "'throttle_body' object is required");
    }
    SECTION("discharge coefficient above 1") {
        json j = base_simulated_engine();
        j["throttle_body"]["discharge_coefficient"] = 1.1;
        expect_rejected(j, "'discharge_coefficient' must be in (0, 1]");
    }
    SECTION("closed plate with zero area") {
        json j = base_simulated_engine();
        j["throttle_body"]["area_fraction_vs_position"]["y"][0] = 0.0;
        expect_rejected(j, "y at position 0 must be > 0");
    }
    SECTION("area curve not spanning 0..1") {
        json j = base_simulated_engine();
        j["throttle_body"]["area_fraction_vs_position"] = {{"x", {0.0, 0.8}}, {"y", {0.01, 1.0}}};
        expect_rejected(j, "throttle_body.area_fraction_vs_position");
    }
    // --- throttle input (plan section 3.4.5) ---
    SECTION("missing throttle_input") {
        json j = base_simulated_engine();
        j.erase("throttle_input");
        expect_rejected(j, "'throttle_input' is required for a simulated engine");
    }
    SECTION("mechanical without a bypass valve") {
        json j = base_mechanical_engine();
        j["idle_control"].erase("bypass_area_mm2");
        expect_rejected(j, "a mechanical throttle requires idle_control.bypass_area_mm2");
    }
    SECTION("mechanical with an actuator time constant") {
        json j = base_mechanical_engine();
        j["throttle_actuator_tau_s"] = 0.04;
        expect_rejected(j, "'throttle_actuator_tau_s' is not allowed on a mechanical throttle");
    }
    SECTION("electronic with a bypass valve") {
        json j = base_simulated_engine();
        j["idle_control"]["bypass_area_mm2"] = 50.0;
        expect_rejected(j, "idle_control.bypass_area_mm2 is not allowed on an electronic throttle");
    }
    SECTION("electronic without an actuator time constant") {
        json j = base_simulated_engine();
        j.erase("throttle_actuator_tau_s");
        expect_rejected(j, "'throttle_actuator_tau_s' is required");
    }
    SECTION("electronic with a negative actuator time constant") {
        json j = base_simulated_engine();
        j["throttle_actuator_tau_s"] = -0.01;
        expect_rejected(j, "'throttle_actuator_tau_s' must be >= 0");
    }
    SECTION("mechanical with a non-positive bypass area") {
        json j = base_mechanical_engine();
        j["idle_control"]["bypass_area_mm2"] = 0.0;
        expect_rejected(j, "'bypass_area_mm2' must be > 0");
    }
    // --- ECU ---
    SECTION("missing ecu") {
        json j = base_simulated_engine();
        j.erase("ecu");
        expect_rejected(j, "'ecu' object is required");
    }
    SECTION("lambda target out of range") {
        json j = base_simulated_engine();
        j["ecu"]["lambda_target"]["values"][0][0] = 3.0;
        expect_rejected(j, "ecu.lambda_target: every value must be in [0.5");
    }
    SECTION("table row count mismatch") {
        json j = base_simulated_engine();
        j["ecu"]["spark_advance_deg"]["values"] = {{30.0, 20.0}};
        expect_rejected(j, "ecu.spark_advance_deg: 'values' must have one row per 'rpm' point (2), got 1");
    }
    SECTION("table cell not a number") {
        json j = base_simulated_engine();
        j["ecu"]["spark_advance_deg"]["values"][1][1] = "thirty";
        expect_rejected(j, "ecu.spark_advance_deg: values[1][1] must be a number");
    }
    SECTION("table with a missing load axis") {
        json j = base_simulated_engine();
        j["ecu"]["spark_advance_deg"].erase("p_im_kpa");
        expect_rejected(j, "ecu.spark_advance_deg: 'p_im_kpa' array is required");
    }
    SECTION("table rpm not ascending") {
        json j = base_simulated_engine();
        j["maps"]["ve"]["rpm"] = {0.0, 7000.0, 3000.0};
        expect_rejected(j, "maps.ve:");
    }
    // --- maps ---
    SECTION("missing maps") {
        json j = base_simulated_engine();
        j.erase("maps");
        expect_rejected(j, "'maps' object is required");
    }
    SECTION("maps without a source") {
        json j = base_simulated_engine();
        j["maps"].erase("source");
        expect_rejected(j, "maps: 'source' is required");
    }
    SECTION("missing VE map") {
        json j = base_simulated_engine();
        j["maps"].erase("ve");
        expect_rejected(j, "maps: 've' is required");
    }
    SECTION("VE out of range") {
        json j = base_simulated_engine();
        j["maps"]["ve"]["values"][0][0] = 0.0;
        expect_rejected(j, "maps.ve: every value must be in (0");
    }
    SECTION("eta_i >= 1") {
        json j = base_simulated_engine();
        j["maps"]["eta_i"]["values"][0][0] = 1.0;
        expect_rejected(j, "maps.eta_i: every value must be in (0");
    }
    SECTION("f_lambda(1) != 1 (eta_i is defined at lambda = 1)") {
        json j = base_simulated_engine();
        j["maps"]["f_lambda"]["y"][1] = 0.98;
        expect_rejected(j, "f_lambda(1) must be exactly 1");
    }
    SECTION("eta_spark(0) != 1 (eta_i is defined at MBT)") {
        json j = base_simulated_engine();
        j["maps"]["eta_spark"]["y"][1] = 0.97;
        expect_rejected(j, "eta_spark(0) must be exactly 1");
    }
    SECTION("eta_spark above 1") {
        json j = base_simulated_engine();
        j["maps"]["eta_spark"]["y"][2] = 1.05;
        expect_rejected(j, "maps.eta_spark: every y must be in [0, 1]");
    }
    SECTION("friction curve not starting at rpm 0") {
        json j = base_simulated_engine();
        j["maps"]["friction_torque_nm"]["x"][0] = 100.0;
        expect_rejected(j, "friction_torque_nm: x[0] must be exactly 0");
    }
    SECTION("negative friction") {
        json j = base_simulated_engine();
        j["maps"]["friction_torque_nm"]["y"][1] = -1.0;
        expect_rejected(j, "friction_torque_nm: every y must be >= 0");
    }
    // I7c C16b review N1: pmep_excess_pa is a SIGNED additive term at
    // runtime (engine_simulated.cpp's tau_pump term has no sign special-
    // casing), and the C16a generator already produces a signed value
    // (provisional decision P-E) - a file value follows the same rule, so a
    // negative value now LOADS instead of being rejected (was "negative
    // pmep_excess" -> expect_rejected before this fix).
    SECTION("negative pmep_excess loads (signed, per I7c C16b review N1)") {
        json j = base_simulated_engine();
        j["maps"]["pmep_excess_kpa"] = {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{-1.0, 1.0}, {8.0, 4.0}}}};
        const ps::vehicle::VehicleDesc d = load_with_engine(j);
        const SimulatedEngineDesc& e = only_sim_engine(d);
        REQUIRE(e.maps.has_pmep_excess);
        CHECK(e.maps.pmep_excess_pa.values == std::vector<real>{-1000.0, 1000.0, 8000.0, 4000.0});
    }
    SECTION("pmep_excess magnitude out of bounds") {
        json j = base_simulated_engine();
        j["maps"]["pmep_excess_kpa"] = {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{2000.0, 1.0}, {8.0, 4.0}}}};
        expect_rejected(j, "maps.pmep_excess_kpa: every value must be in [-1000000");
    }
    SECTION("pmep_excess non-finite (nlohmann rejects the literal before the loader sees it)") {
        // Same pattern as test_engine_fuel_map.cpp's "indicated_efficiency"
        // 1e400 SECTION and test_fuel_tank.cpp's "capacity_kg" one: nlohmann
        // v3.12 refuses an overflowing-but-syntactically-valid numeral
        // (out_of_range.406, "number overflow parsing '1e400'") during its
        // own parse, before core/src/io/vehicle_io.cpp's require_table_values
        // ever sees the value - so this pins nlohmann's own rejection, not a
        // loader message, matching the established precedent for this class
        // of test in this codebase.
        json j = base_simulated_engine();
        j["maps"]["pmep_excess_kpa"] = {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{9999.0, 1.0}, {8.0, 4.0}}}};
        std::string text = j.dump(2);
        const std::string needle = "9999.0";
        const auto pos = text.find(needle);
        REQUIRE(pos != std::string::npos);
        text.replace(pos, needle.size(), "1e400");
        const std::filesystem::path raw = std::filesystem::temp_directory_path() / "ps_test_simulated_engine_pmep_excess_1e400.json";
        { std::ofstream out(raw); out << text; }
        REQUIRE_THROWS_WITH(ps::io::load_vehicle_json(raw.string()), ContainsSubstring("number overflow parsing '1e400'"));
        std::error_code ec;
        std::filesystem::remove(raw, ec);
    }
    SECTION("an efficiency combination reaching 100 %") {
        json j = base_simulated_engine();
        j["maps"]["eta_i"]["values"][1][1] = 0.9;
        j["maps"]["f_lambda"]["y"][2] = 1.2;
        expect_rejected(j, "implies >= 100% indicated efficiency");
    }
    // --- graph-engine-only fields ---
    SECTION("a WOT curve on a simulated engine") {
        json j = base_simulated_engine();
        j["wot_torque_nm_vs_rpm"] = {{"x", {500.0, 7000.0}}, {"y", {100.0, 100.0}}};
        expect_rejected(j, "simulated engines derive those curves");
    }
    SECTION("a governor on a simulated engine") {
        json j = base_simulated_engine();
        j["governor"] = {{"high_idle_rpm", 2400.0}};
        expect_rejected(j, "'governor' is \"torque_map\"-only in I7");
    }
    SECTION("a fuel_map on a simulated engine") {
        json j = base_simulated_engine();
        j["fuel_map"] = json::object();
        expect_rejected(j, "'fuel_map' is not allowed on a \"simulated\" engine");
    }
    SECTION("indicated_efficiency on a simulated engine") {
        json j = base_simulated_engine();
        j["indicated_efficiency"] = 0.35;
        expect_rejected(j, "'indicated_efficiency' is not allowed on a \"simulated\" engine - use maps.eta_i");
    }
    // --- shared fields ---
    SECTION("missing inertia") {
        json j = base_simulated_engine();
        j.erase("inertia_kgm2");
        expect_rejected(j, "'inertia_kgm2' is required");
    }
    SECTION("breakaway below the Coulomb friction") {
        json j = base_simulated_engine();
        j["breakaway_torque_nm"] = 10.0;
        expect_rejected(j, "breakaway_torque_nm must be >= maps.friction_torque_nm(0)");
    }
    SECTION("rpm threshold order") {
        json j = base_simulated_engine();
        j["idle_rpm"] = 500.0;
        expect_rejected(j, "stall_rpm < start_complete_rpm <= idle_rpm < limiter.rpm");
    }
    SECTION("DFCO threshold order") {
        json j = base_simulated_engine();
        j["dfco_reenable_rpm"] = 1400.0;
        expect_rejected(j, "dfco_reenable_rpm must be < dfco_enable_rpm");
    }
    SECTION("idle max_throttle outside (0, 1]") {
        json j = base_simulated_engine();
        j["idle_control"]["max_throttle"] = 0.0;
        expect_rejected(j, "idle_control.max_throttle must be in (0, 1]");
    }
    // R6 (vault PHYS-049/050 follow-ups): a negative PI gain flips the idle
    // loop's sign instead of merely weakening it.
    SECTION("negative idle kp_per_rpm") {
        json j = base_simulated_engine();
        j["idle_control"]["kp_per_rpm"] = -0.001;
        expect_rejected(j, "idle_control.kp_per_rpm must be >= 0");
    }
    SECTION("negative idle ki_per_rpm_s") {
        json j = base_simulated_engine();
        j["idle_control"]["ki_per_rpm_s"] = -0.004;
        expect_rejected(j, "idle_control.ki_per_rpm_s must be >= 0");
    }
    // R6: starter.free_speed_rpm <= 0 divides by zero (or flips the starter
    // torque-speed line's sign) at runtime in both engine kinds.
    SECTION("starter.free_speed_rpm == 0") {
        json j = base_simulated_engine();
        j["starter"] = {{"stall_torque_nm", 150.0}, {"free_speed_rpm", 0.0}};
        expect_rejected(j, "starter.free_speed_rpm must be > 0");
    }
    SECTION("starter.free_speed_rpm < 0") {
        json j = base_simulated_engine();
        j["starter"] = {{"stall_torque_nm", 150.0}, {"free_speed_rpm", -400.0}};
        expect_rejected(j, "starter.free_speed_rpm must be > 0");
    }
    // R6: a negative hysteresis reverses the FuelCut latch's restore
    // direction (restores above the cut point instead of below it).
    SECTION("negative limiter.hysteresis_rpm") {
        json j = base_simulated_engine();
        j["limiter"] = {{"type", "fuel_cut"}, {"rpm", 6800.0}, {"hysteresis_rpm", -150.0}};
        expect_rejected(j, "limiter.hysteresis_rpm must be >= 0");
    }
    // --- fuel + ignition ---
    SECTION("no fuel") {
        json j = base_simulated_engine();
        j.erase("fuel");
        expect_rejected(j, "a simulated engine requires 'fuels'");
    }
    SECTION("both fuel and fuels") {
        json j = base_simulated_engine();
        j["fuels"] = {gasoline_path()};
        expect_rejected(j, "'fuels' and 'fuel' are mutually exclusive");
    }
    SECTION("an empty fuels list") {
        json j = base_simulated_engine();
        j.erase("fuel");
        j["fuels"] = json::array();
        expect_rejected(j, "'fuels' must be a non-empty array of path strings");
    }
    SECTION("a duplicate fuel") {
        json j = base_simulated_engine();
        j.erase("fuel");
        j["fuels"] = {gasoline_path(), gasoline_path()};
        expect_rejected(j, "'fuels' contains a duplicate fuel reference");
    }
    SECTION("missing ignition") {
        json j = base_simulated_engine();
        j.erase("ignition");
        expect_rejected(j, "'ignition' is required for a simulated engine");
    }
    SECTION("compression ignition") {
        json j = base_simulated_engine();
        j["ignition"] = "compression";
        expect_rejected(j, "ignition \"compression\" is not supported by the simulated engine yet");
    }
}

TEST_CASE("C10: a torque_map engine refuses simulated-only fields", "[unit][vehicle][io][drivetrain][engine]") {
    json j;
    j["id"] = "engine";
    j["type"] = "engine";
    j["kind"] = "torque_map";
    j["wot_torque_nm_vs_rpm"] = {{"x", {500.0, 7000.0}}, {"y", {100.0, 100.0}}};
    j["motoring_torque_nm_vs_rpm"] = {{"x", {0.0, 7000.0}}, {"y", {-20.0, -20.0}}};
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 20.0;
    SECTION("the plain torque_map engine loads") {
        const ps::vehicle::VehicleDesc d = load_with_engine(j);
        CHECK(std::holds_alternative<TorqueMapEngineDesc>(d.powertrain.components[0].params));
    }
    SECTION("a maps block") {
        j["maps"] = json::object();
        expect_rejected(j, "'maps' is a \"simulated\" engine field and is not allowed on a \"torque_map\" engine");
    }
    SECTION("a bore") {
        j["bore_mm"] = 86.0;
        expect_rejected(j, "'bore_mm' is a \"simulated\" engine field");
    }
    SECTION("an idle bypass valve") {
        j["idle_control"] = {{"bypass_area_mm2", 50.0}};
        expect_rejected(j, "idle_control.bypass_area_mm2 is a \"simulated\" engine field");
    }
}

TEST_CASE("C10: throttle_control and manual_tcu input-type checks cover simulated engines",
          "[unit][vehicle][io][drivetrain][engine][throttle]") {
    SECTION("an explicit mechanical_linkage stage on an electronic simulated engine is rejected") {
        const json stage = {{"id", "throttle"}, {"type", "throttle_control"}, {"engine", "engine"}, {"kind", "mechanical_linkage"}};
        TempDir d;
        const auto p = d.write("vehicle.json", vehicle_with_components(json::array({base_simulated_engine(), stage})));
        REQUIRE_THROWS_WITH(ps::io::load_vehicle_json(p.string()),
                            ContainsSubstring("'kind' (\"mechanical_linkage\") does not match engine 'engine'.throttle_input (\"electronic\")"));
    }
    SECTION("a matching mechanical_linkage stage on a mechanical simulated engine loads") {
        const json stage = {{"id", "throttle"}, {"type", "throttle_control"}, {"engine", "engine"}, {"kind", "mechanical_linkage"}};
        TempDir d;
        const auto p = d.write("vehicle.json", vehicle_with_components(json::array({base_mechanical_engine(), stage})));
        const ps::vehicle::VehicleDesc v = ps::io::load_vehicle_json(p.string());
        CHECK(v.powertrain.components.size() == 2);
    }
    SECTION("a manual_tcu with default blip/ramp values on a mechanical simulated engine is rejected") {
        TempDir d;
        d.write("gearbox.json", json{{"format", "physics_sim.gearbox/1"},
                                     {"name", "fixture"},
                                     {"type", "manual"},
                                     {"forward_ratios", {3.0, 1.6, 1.0}},
                                     {"forward_efficiency", {1.0, 1.0, 1.0}}});
        json v;
        v["format"] = "physics_sim.vehicle/2";
        v["source"] = "test fixture, not a real vehicle (test_simulated_engine_io.cpp)";
        v["wheels"] = json::array({{{"name", "FL"}, {"tyre", {{"source", "test fixture"}}}}, {{"name", "FR"}, {"tyre", {{"source", "test fixture"}}}}});
        v["powertrain"] = {
            {"components",
             json::array({base_mechanical_engine(),
                          {{"id", "clutch"}, {"type", "clutch"}, {"capacity_nm", 300.0},
                           {"engagement_vs_pedal", {{"x", {0.0, 1.0}}, {"y", {1.0, 0.0}}}}},
                          {{"id", "gearbox"}, {"type", "gearbox"}, {"ref", "gearbox.json"}},
                          {{"id", "diff"}, {"type", "differential"}, {"kind", "open"}, {"ratio", 3.7}}})},
            {"shafts", json::array({{{"ports", {"engine.crank", "clutch.in"}}},
                                    {{"ports", {"clutch.out", "gearbox.in"}}},
                                    {{"ports", {"gearbox.out", "diff.carrier"}}},
                                    {{"ports", {"diff.left", "wheel:FL"}}},
                                    {{"ports", {"diff.right", "wheel:FR"}}}})}};
        v["controllers"] = json::array(
            {{{"id", "tcu"}, {"type", "manual_tcu"}, {"gearbox", "gearbox"}, {"clutch", "clutch"}, {"engine", "engine"}}});
        const auto p = d.write("vehicle.json", v);
        REQUIRE_THROWS_WITH(ps::io::load_vehicle_json(p.string()),
                            ContainsSubstring("has a mechanical throttle - a mechanical throttle cannot open the plate"));
    }
}
