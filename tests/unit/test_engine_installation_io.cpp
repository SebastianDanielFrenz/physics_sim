// tests/unit/test_engine_installation_io.cpp - ps_audio A4 (out/audio_plan.md
// sections 3.1/3.2/3.3): the engine cycle block's optional intake/exhaust
// finite-volume pipe pair, the "resolution" cell-size block, the
// physics_sim.installation/1 file format (ps/drivetrain/installation_desc.h,
// core/src/io/vehicle_io.cpp's read_intake_block/read_exhaust_block/
// read_resolution_block/read_installation_object) and the vehicle-level
// per-engine "installation" override.
//
// A4 is data-only: nothing consumes these fields yet (A5 wires them into the
// map generator) - every check here is loader validation, round-trip fidelity
// and (test_engine_map_cache.cpp's H3 section covers the cache-key side)
// leaving an engine WITHOUT pipe blocks bit-identical to before A4.
//
// Every fixture is built in memory and written to a scratch temp file; no
// data/** file is read except data/fuels/gasoline.json (the fuel reference,
// same pattern as test_simulated_engine_io.cpp/test_spark_schedule.cpp).
// Rejection SECTIONs mutate one copy of a known-valid fixture per case and
// pin the loader's message via ContainsSubstring - the same "mutate one rule,
// expect one throw" pattern the rest of this loader's tests use throughout
// vehicle_io.cpp's history (test_vehicle_io_validation.cpp, test_fuel_io.cpp,
// test_simulated_engine_io.cpp).

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/installation_desc.h"
#include "ps/jobs/job_system.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_desc.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;

namespace {

struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_engine_installation_io_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
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

// A minimal but complete "cycle" block for a 4-cylinder engine, WITHOUT the
// pipe pair (bit-identical to a pre-A4 cycle block) - callers add
// intake/exhaust/default_installation/resolution as needed. A full "maps"
// override on the enclosing engine means the generator never has to run
// (M5: a complete override runs the generator 0 times), so every SECTION
// below is fast.
json base_cycle_json() {
    json c;
    c["source"] = "test fixture, not physical data (test_engine_installation_io.cpp)";
    c["rod_mm"] = 145.0;
    c["compression_ratio"] = 10.5;
    c["intake_valves"] = 2;
    c["exhaust_valves"] = 2;
    c["intake_valve_d_mm"] = 34.0;
    c["exhaust_valve_d_mm"] = 29.0;
    c["intake_lift"] = {{"lift_mm_vs_deg", {{"x", {0.0, 120.0, 240.0}}, {"y", {0.0, 9.0, 0.0}}}}, {"phase_ref_deg", -10.0}};
    c["exhaust_lift"] = {{"lift_mm_vs_deg", {{"x", {0.0, 120.0, 240.0}}, {"y", {0.0, 9.0, 0.0}}}}, {"phase_ref_deg", 490.0}};
    c["runner_length_mm"] = 350.0;
    c["runner_diameter_mm"] = 40.0;
    c["intake_port_cc"] = 150.0;
    c["exhaust_port_cc"] = 250.0;
    c["cd_vs_l_over_d"] = {{"x", {0.0, 0.1, 0.2, 0.3}}, {"y", {0.65, 0.62, 0.55, 0.45}}};
    c["friction"] = {{"model", "polynomial_piston_speed"}, {"a_kpa", 66.62}, {"b_kpa", 0.091}, {"c_kpa", 0.347}};
    c["grid_rpm"] = {1000.0, 2500.0, 4000.0, 5500.0};
    c["grid_p_im_kpa"] = {30.0, 60.0, 95.0};
    return c;
}

// Replaces the lumped runner_length_mm/runner_diameter_mm with a valid
// intake/exhaust pipe pair (one runner/primary each covering "all" 4
// cylinders, one merge reaching the flange) plus a valid resolution block
// (defaults - omitted) and default_installation pointing at "installation.json"
// (resolved relative to the engine file's own directory - here, since the
// engine is inline, the vehicle file's directory, same dir the fixture writes
// installation.json into).
json with_pipes(json c) {
    c.erase("runner_length_mm");
    c.erase("runner_diameter_mm");
    c["intake"] = {{"source", "test fixture"},
                   {"throttle_bore_m", 0.060},
                   {"plenum_m3", 0.0025},
                   {"runners", json::array({{{"cylinders", "all"}, {"length_m", 0.35}, {"d_in_m", 0.045}, {"d_out_m", 0.040}}})},
                   {"interface", "throttle_inlet"}};
    c["exhaust"] = {{"source", "test fixture"},
                    {"primaries", json::array({{{"cylinders", "all"}, {"length_m", 0.40}, {"d_m", 0.036}}})},
                    {"merges", json::array({{{"from", json::array({1, 2, 3, 4})}, {"into", "flange"}, {"length_m", 0.10}, {"d_m", 0.050}}})},
                    {"interface", "flange"}};
    c["default_installation"] = "installation.json";
    return c;
}

json full_maps_override() {
    return {{"source", "synthetic test fixture, not physical data"},
            {"ve", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{0.80, 0.90}, {0.85, 0.95}}}}},
            {"eta_i", {{"rpm", {0.0, 7000.0}}, {"air_mass_per_rev_g", {0.0, 1.5}}, {"values", {{0.34, 0.37}, {0.36, 0.38}}}}},
            {"f_lambda", {{"x", {0.7, 1.0, 1.5}}, {"y", {0.85, 1.0, 0.9}}}},
            {"eta_spark", {{"x", {-40.0, 0.0, 10.0}}, {"y", {0.5, 1.0, 0.95}}}},
            {"theta_mbt_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}},
            {"friction_torque_nm", {{"x", {0.0, 1000.0, 7000.0}}, {"y", {15.0, 18.0, 40.0}}}},
            {"pmep_excess_kpa", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{0.0, 0.0}, {0.0, 0.0}}}}}};
}

json base_engine_json(bool with_installation_pipes) {
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
    j["ecu"] = {{"lambda_target", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{1.0, 1.0}, {1.0, 1.0}}}}},
                {"spark_advance_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}}};
    j["maps"] = full_maps_override();
    j["cycle"] = with_installation_pipes ? with_pipes(base_cycle_json()) : base_cycle_json();
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 20.0;
    j["fuel"] = gasoline_path();
    j["ignition"] = "spark";
    return j;
}

// A minimal but complete physics_sim.installation/1 object.
json base_installation_json() {
    json j;
    j["format"] = "physics_sim.installation/1";
    j["source"] = "test fixture, not physical data (test_engine_installation_io.cpp)";
    j["intake"] = {{"airbox_m3", 0.010},
                   {"snorkel", {{"length_m", 0.4}, {"d_m", 0.07}}},
                   {"inlet_position_m", json::array({0.9, 0.3, 0.5})}};
    j["exhaust"] = {{"elements", json::array({{{"type", "catalyst"}, {"length_m", 0.12}, {"d_m", 0.11}, {"k_loss", 4.0}},
                                              {{"type", "pipe"}, {"length_m", 1.6}, {"d_m", 0.05}},
                                              {{"type", "outlet"}, {"position_m", json::array({-2.2, -0.45, 0.25})}}})}};
    return j;
}

json vehicle_with_components(const json& components) {
    json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_engine_installation_io.cpp)";
    v["wheels"] = json::array({{{"name", "FL"}}});
    v["powertrain"] = {{"components", components}};
    return v;
}

vehicle::VehicleDesc load_with_engine(const TempDir& d, const json& engine) {
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({engine})));
    return io::load_vehicle_json(p.string());
}

// Writes base_installation_json() as "installation.json" next to the vehicle
// file, then loads `engine` (mutated by the caller) through a vehicle at the
// same directory, and expects load_vehicle_json to throw with `substring`.
void expect_rejected(const json& engine, const std::string& substring) {
    TempDir d;
    d.write("installation.json", base_installation_json());
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({engine})));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(p.string()), ContainsSubstring(substring));
}

// Mutates only cycle["intake"] (engine already has_pipes) and expects the
// given throw.
void expect_intake_rejected(std::function<void(json&)> mutate, const std::string& substring) {
    json e = base_engine_json(/*with_installation_pipes=*/true);
    mutate(e["cycle"]["intake"]);
    expect_rejected(e, substring);
}

void expect_exhaust_rejected(std::function<void(json&)> mutate, const std::string& substring) {
    json e = base_engine_json(/*with_installation_pipes=*/true);
    mutate(e["cycle"]["exhaust"]);
    expect_rejected(e, substring);
}

void expect_resolution_rejected(std::function<void(json&)> mutate, const std::string& substring) {
    json e = base_engine_json(/*with_installation_pipes=*/true);
    json r = json::object();
    mutate(r);
    e["cycle"]["resolution"] = r;
    expect_rejected(e, substring);
}

// Mutates a fresh base_installation_json() and expects load_vehicle_json (an
// engine with pipes, default_installation -> the mutated file) to throw.
void expect_installation_rejected(std::function<void(json&)> mutate, const std::string& substring) {
    TempDir d;
    json inst = base_installation_json();
    mutate(inst);
    d.write("installation.json", inst);
    const json e = base_engine_json(/*with_installation_pipes=*/true);
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({e})));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(p.string()), ContainsSubstring(substring));
}

const SimulatedEngineDesc& only_sim_engine(const vehicle::VehicleDesc& d) {
    REQUIRE(d.powertrain.components.size() == 1);
    const auto* e = std::get_if<SimulatedEngineDesc>(&d.powertrain.components[0].params);
    REQUIRE(e != nullptr);
    return *e;
}

} // namespace

// ---------------------------------------------------------------------------
// Round trip
// ---------------------------------------------------------------------------

TEST_CASE("A4: an engine with intake/exhaust pipe blocks and a resolved default installation round-trips",
          "[unit][io][cycle][drivetrain]") {
    TempDir d;
    d.write("installation.json", base_installation_json());
    const vehicle::VehicleDesc v = load_with_engine(d, base_engine_json(/*with_installation_pipes=*/true));
    const SimulatedEngineDesc& e = only_sim_engine(v);
    REQUIRE(e.cycle.has_value());
    const CycleModelDesc& c = *e.cycle;

    REQUIRE(c.intake.has_value());
    CHECK(c.intake->throttle_bore_m == real(0.060));
    CHECK(c.intake->plenum_m3 == real(0.0025));
    REQUIRE(c.intake->runners.size() == 1);
    CHECK(c.intake->runners[0].cylinders.empty()); // "all"
    CHECK(c.intake->runners[0].length_m == real(0.35));
    CHECK(c.intake->runners[0].d_in_m == real(0.045));
    CHECK(c.intake->runners[0].d_out_m == real(0.040));
    CHECK(c.intake->interface == "throttle_inlet");

    REQUIRE(c.exhaust.has_value());
    REQUIRE(c.exhaust->primaries.size() == 1);
    CHECK(c.exhaust->primaries[0].d_m == real(0.036));
    REQUIRE(c.exhaust->merges.size() == 1);
    REQUIRE(c.exhaust->merges[0].from.size() == 4);
    CHECK(c.exhaust->merges[0].from[0].is_cylinder);
    CHECK(c.exhaust->merges[0].from[0].cylinder == 1);
    CHECK(c.exhaust->merges[0].into == "flange");
    CHECK(c.exhaust->interface == "flange");

    // resolution: defaults (not overridden in this fixture).
    CHECK(c.resolution.power_dx_m == real(0.05));
    CHECK(c.resolution.audio_dx_exhaust_m == real(0.032));
    CHECK(c.resolution.audio_dx_intake_m == real(0.017));

    // default_installation resolved to an absolute path next to the vehicle
    // file, and the installation itself loaded and matches the fixture.
    CHECK_FALSE(c.default_installation_path.empty());
    CHECK(c.installation_path == c.default_installation_path);
    REQUIRE(c.installation.has_value());
    const InstallationDesc& inst = *c.installation;
    CHECK(inst.intake.airbox_m3 == real(0.010));
    CHECK(inst.intake.has_snorkel);
    CHECK(inst.intake.snorkel.length_m == real(0.4));
    CHECK(inst.intake.snorkel.d_m == real(0.07));
    CHECK(inst.intake.inlet_position_m.x == real(0.9));
    REQUIRE(inst.exhaust.elements.size() == 3);
    CHECK(inst.exhaust.elements[0].type == ExhaustElementType::Catalyst);
    CHECK(inst.exhaust.elements[0].k_loss == real(4.0));
    CHECK(inst.exhaust.elements[1].type == ExhaustElementType::Pipe);
    CHECK(inst.exhaust.elements[2].type == ExhaustElementType::Outlet);
    CHECK(inst.exhaust.elements[2].position_m.x == real(-2.2));
}

TEST_CASE("A4: a resolution block overriding all three dx values round-trips", "[unit][io][cycle][drivetrain]") {
    TempDir d;
    d.write("installation.json", base_installation_json());
    json e = base_engine_json(/*with_installation_pipes=*/true);
    e["cycle"]["resolution"] = {{"power_dx_m", 0.1}, {"audio_dx_exhaust_m", 0.05}, {"audio_dx_intake_m", 0.03}};
    const vehicle::VehicleDesc v = load_with_engine(d, e);
    const CycleModelDesc& c = *only_sim_engine(v).cycle;
    CHECK(c.resolution.power_dx_m == real(0.1));
    CHECK(c.resolution.audio_dx_exhaust_m == real(0.05));
    CHECK(c.resolution.audio_dx_intake_m == real(0.03));
}

TEST_CASE("A4: a vehicle-level 'installation' override on the engine component wins over the default and round-trips",
          "[unit][io][cycle][drivetrain]") {
    TempDir d;
    d.write("installation.json", base_installation_json());
    json alt = base_installation_json();
    alt["source"] = "alternate installation fixture";
    alt["intake"]["airbox_m3"] = 0.020;
    d.write("alt_installation.json", alt);

    json e = base_engine_json(/*with_installation_pipes=*/true);
    e["installation"] = "alt_installation.json";
    const vehicle::VehicleDesc v = load_with_engine(d, e);
    const CycleModelDesc& c = *only_sim_engine(v).cycle;
    REQUIRE(c.installation.has_value());
    CHECK(c.installation->intake.airbox_m3 == real(0.020));
    // The default_installation_path is still resolved/recorded (engine-file
    // relative), but installation_path (what actually got LOADED) differs.
    CHECK(c.installation_path != c.default_installation_path);
    CHECK(c.installation->source == "alternate installation fixture");
}

// ps_audio A5 (PROVISIONAL P12): the installation is a generator input, so it
// must be resolved before the loader generates the maps. A4 resolved it after
// read_engine_fields returned - a pipe engine generated without it and the
// vehicle-level override never reached the maps.
TEST_CASE("A5: the resolved installation (vehicle override) reaches the generated maps",
          "[unit][io][cycle][drivetrain][pipe]") {
    TempDir d;
    d.write("installation.json", base_installation_json());
    json alt = base_installation_json();
    alt["source"] = "alternate installation fixture";
    alt["intake"]["airbox_m3"] = 0.020;
    alt["exhaust"]["elements"][1]["length_m"] = 0.9;
    d.write("alt_installation.json", alt);

    json e = base_engine_json(/*with_installation_pipes=*/true);
    e.erase("maps"); // no override: the generator runs at load
    e["cycle"]["grid_rpm"] = {2000.0, 4000.0};
    e["cycle"]["grid_p_im_kpa"] = {50.0, 80.0};
    e["installation"] = "alt_installation.json";
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({e})));

    jobs::JobSystem pool(10);
    io::EngineMapOptions opt;
    opt.quality = MapQuality::Preview;
    opt.pool = &pool;
    const vehicle::VehicleDesc v = io::load_vehicle_json(p.string(), opt);
    const SimulatedEngineDesc& s = only_sim_engine(v);
    REQUIRE(s.cycle->installation.has_value());
    REQUIRE(s.cycle->installation->intake.airbox_m3 == real(0.020));
    const EngineMapsPtr direct = generate_engine_maps(*s.cycle, s.fuels.front(), MapQuality::Preview, &pool);
    CHECK(direct->generator_version == 2u);
    CHECK(s.maps.ve.values == direct->maps.ve.values);
    CHECK(s.maps.eta_i.values == direct->maps.eta_i.values);

    // ... and the override matters (else the check above would be vacuous).
    CycleModelDesc dflt = *s.cycle;
    dflt.installation = io::load_installation_json((d.dir / "installation.json").string());
    const EngineMapsPtr with_default = generate_engine_maps(dflt, s.fuels.front(), MapQuality::Preview, &pool);
    CHECK(with_default->maps.ve.values != direct->maps.ve.values);
}

TEST_CASE("A4: an engine WITHOUT pipe blocks has an identical cycle key formula to before A4 "
          "(intake/exhaust/installation absent, resolution at its default)",
          "[unit][io][cycle][drivetrain]") {
    TempDir d;
    const vehicle::VehicleDesc v = load_with_engine(d, base_engine_json(/*with_installation_pipes=*/false));
    const CycleModelDesc& c = *only_sim_engine(v).cycle;
    CHECK_FALSE(c.intake.has_value());
    CHECK_FALSE(c.exhaust.has_value());
    CHECK_FALSE(c.installation.has_value());
    CHECK(c.default_installation_path.empty());
    CHECK(c.installation_path.empty());
    // Defaults, exactly as a pre-A4 CycleModelDesc would carry them (nothing
    // in read_cycle_block writes 'resolution' without has_pipes).
    CHECK(c.resolution.power_dx_m == real(0.05));
    CHECK(c.resolution.audio_dx_exhaust_m == real(0.032));
    CHECK(c.resolution.audio_dx_intake_m == real(0.017));
    // The pre-A4 lumped runner fields are still populated as before.
    // 350.0 * 1e-3 is not bit-exact to the literal 0.35 - a tolerance check,
    // not the bit-exact style used for values that come straight from JSON
    // with no arithmetic (e.g. the intake/exhaust fields above).
    CHECK(std::abs(c.runner_length_m - real(0.35)) < real(1e-12));
    CHECK(std::abs(c.runner_diameter_m - real(0.040)) < real(1e-12));
}

// ---------------------------------------------------------------------------
// Pair-check / runner_* / default_installation / resolution gating
// ---------------------------------------------------------------------------

TEST_CASE("A4: 'intake' and 'exhaust' must be given together", "[unit][io][cycle][drivetrain]") {
    SECTION("intake without exhaust") {
        json e = base_engine_json(/*with_installation_pipes=*/true);
        e["cycle"].erase("exhaust");
        expect_rejected(e, "'intake' and 'exhaust' must be given together");
    }
    SECTION("exhaust without intake") {
        json e = base_engine_json(/*with_installation_pipes=*/true);
        e["cycle"].erase("intake");
        expect_rejected(e, "'intake' and 'exhaust' must be given together");
    }
}

TEST_CASE("A4: runner_length_mm/runner_diameter_mm are rejected together with intake/exhaust pipe blocks",
          "[unit][io][cycle][drivetrain]") {
    SECTION("runner_length_mm present") {
        json e = base_engine_json(/*with_installation_pipes=*/true);
        e["cycle"]["runner_length_mm"] = 350.0;
        expect_rejected(e, "'runner_length_mm'/'runner_diameter_mm' are not allowed together with 'intake'/'exhaust'");
    }
    SECTION("runner_diameter_mm present") {
        json e = base_engine_json(/*with_installation_pipes=*/true);
        e["cycle"]["runner_diameter_mm"] = 40.0;
        expect_rejected(e, "'runner_length_mm'/'runner_diameter_mm' are not allowed together with 'intake'/'exhaust'");
    }
}

TEST_CASE("A4: runner_length_mm/runner_diameter_mm are still required without pipe blocks (bit-identical pre-A4 rule)",
          "[unit][io][cycle][drivetrain]") {
    json e = base_engine_json(/*with_installation_pipes=*/false);
    e["cycle"].erase("runner_length_mm");
    expect_rejected(e, "'runner_length_mm' is required");
}

TEST_CASE("A4: 'default_installation' is required when pipe blocks are present", "[unit][io][cycle][drivetrain]") {
    json e = base_engine_json(/*with_installation_pipes=*/true);
    e["cycle"].erase("default_installation");
    expect_rejected(e, "'default_installation' is required");
}

TEST_CASE("A4: 'default_installation' is rejected without pipe blocks", "[unit][io][cycle][drivetrain]") {
    json e = base_engine_json(/*with_installation_pipes=*/false);
    e["cycle"]["default_installation"] = "installation.json";
    expect_rejected(e, "'default_installation' is only allowed together with 'intake'/'exhaust' pipe blocks");
}

TEST_CASE("A4: 'resolution' is rejected without pipe blocks", "[unit][io][cycle][drivetrain]") {
    json e = base_engine_json(/*with_installation_pipes=*/false);
    e["cycle"]["resolution"] = {{"power_dx_m", 0.1}};
    expect_rejected(e, "'resolution' is only allowed together with 'intake'/'exhaust' pipe blocks");
}

// ---------------------------------------------------------------------------
// intake block
// ---------------------------------------------------------------------------

TEST_CASE("A4: intake block rejections", "[unit][io][cycle][drivetrain]") {
    SECTION("unknown key") {
        expect_intake_rejected([](json& i) { i["bogus"] = 1.0; }, "unknown key 'bogus'");
    }
    SECTION("missing throttle_bore_m") {
        expect_intake_rejected([](json& i) { i.erase("throttle_bore_m"); }, "'throttle_bore_m' is required");
    }
    SECTION("non-positive throttle_bore_m") {
        expect_intake_rejected([](json& i) { i["throttle_bore_m"] = 0.0; }, "'throttle_bore_m' must be > 0");
    }
    SECTION("missing plenum_m3") {
        expect_intake_rejected([](json& i) { i.erase("plenum_m3"); }, "'plenum_m3' is required");
    }
    SECTION("runners not an array") {
        expect_intake_rejected([](json& i) { i["runners"] = 5.0; }, "'runners' must be a non-empty array");
    }
    SECTION("runners empty") {
        expect_intake_rejected([](json& i) { i["runners"] = json::array(); }, "'runners' must be a non-empty array");
    }
    SECTION("runner unknown key") {
        expect_intake_rejected([](json& i) { i["runners"][0]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("runner missing cylinders") {
        expect_intake_rejected([](json& i) { i["runners"][0].erase("cylinders"); }, "'cylinders' is required");
    }
    SECTION("runner cylinders bad string") {
        expect_intake_rejected([](json& i) { i["runners"][0]["cylinders"] = "everything"; },
                                "'cylinders' string value must be \"all\"");
    }
    SECTION("runner cylinders empty array") {
        expect_intake_rejected([](json& i) { i["runners"][0]["cylinders"] = json::array(); },
                                "'cylinders' must be \"all\" or a non-empty array");
    }
    SECTION("runner cylinders out of range") {
        expect_intake_rejected([](json& i) { i["runners"][0]["cylinders"] = json::array({5}); }, "out of range [1, 4]");
    }
    SECTION("runner cylinder coverage: duplicate assignment") {
        expect_intake_rejected(
            [](json& i) {
                i["runners"] = json::array({{{"cylinders", json::array({1, 2})}, {"length_m", 0.35}, {"d_in_m", 0.045}, {"d_out_m", 0.040}},
                                            {{"cylinders", json::array({2, 3, 4})}, {"length_m", 0.35}, {"d_in_m", 0.045}, {"d_out_m", 0.040}}});
            },
            "is assigned to more than one entry");
    }
    SECTION("runner cylinder coverage: not every cylinder covered") {
        expect_intake_rejected(
            [](json& i) {
                i["runners"] = json::array({{{"cylinders", json::array({1, 2, 3})}, {"length_m", 0.35}, {"d_in_m", 0.045}, {"d_out_m", 0.040}}});
            },
            "must cover every cylinder exactly once");
    }
    SECTION("runner cylinder coverage: 'all' not the only entry") {
        expect_intake_rejected(
            [](json& i) {
                i["runners"] = json::array({{{"cylinders", "all"}, {"length_m", 0.35}, {"d_in_m", 0.045}, {"d_out_m", 0.040}},
                                            {{"cylinders", json::array({1})}, {"length_m", 0.35}, {"d_in_m", 0.045}, {"d_out_m", 0.040}}});
            },
            "\"all\" must be the only entry");
    }
    SECTION("non-positive length_m") {
        expect_intake_rejected([](json& i) { i["runners"][0]["length_m"] = 0.0; }, "'length_m' must be > 0");
    }
    SECTION("non-positive d_in_m") {
        expect_intake_rejected([](json& i) { i["runners"][0]["d_in_m"] = -1.0; }, "'d_in_m' must be > 0");
    }
    SECTION("non-positive d_out_m") {
        expect_intake_rejected([](json& i) { i["runners"][0]["d_out_m"] = 0.0; }, "'d_out_m' must be > 0");
    }
    SECTION("missing interface") {
        expect_intake_rejected([](json& i) { i.erase("interface"); }, "'interface' is required");
    }
    SECTION("empty interface") {
        expect_intake_rejected([](json& i) { i["interface"] = ""; }, "'interface' is required");
    }
    SECTION("missing source") {
        expect_intake_rejected([](json& i) { i.erase("source"); }, "'source' is required");
    }
}

// ---------------------------------------------------------------------------
// exhaust block
// ---------------------------------------------------------------------------

TEST_CASE("A4: exhaust block rejections", "[unit][io][cycle][drivetrain]") {
    SECTION("unknown key") {
        expect_exhaust_rejected([](json& e) { e["bogus"] = 1.0; }, "unknown key 'bogus'");
    }
    SECTION("primaries not an array") {
        expect_exhaust_rejected([](json& e) { e["primaries"] = 5.0; }, "'primaries' must be a non-empty array");
    }
    SECTION("primaries empty") {
        expect_exhaust_rejected([](json& e) { e["primaries"] = json::array(); }, "'primaries' must be a non-empty array");
    }
    SECTION("primary unknown key") {
        expect_exhaust_rejected([](json& e) { e["primaries"][0]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("primary cylinder coverage bad") {
        expect_exhaust_rejected(
            [](json& e) { e["primaries"][0]["cylinders"] = json::array({1, 2, 3}); }, "must cover every cylinder exactly once");
    }
    SECTION("primary non-positive length_m") {
        expect_exhaust_rejected([](json& e) { e["primaries"][0]["length_m"] = 0.0; }, "'length_m' must be > 0");
    }
    SECTION("primary non-positive d_m") {
        expect_exhaust_rejected([](json& e) { e["primaries"][0]["d_m"] = 0.0; }, "'d_m' must be > 0");
    }
    SECTION("merges not an array") {
        expect_exhaust_rejected([](json& e) { e["merges"] = 5.0; }, "'merges' must be a non-empty array");
    }
    SECTION("merges empty") {
        expect_exhaust_rejected([](json& e) { e["merges"] = json::array(); }, "'merges' must be a non-empty array");
    }
    SECTION("merge unknown key") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("from missing") {
        expect_exhaust_rejected([](json& e) { e["merges"][0].erase("from"); }, "'from' must be an array of at least 2 sources");
    }
    SECTION("from too short") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["from"] = json::array({1}); },
                                 "'from' must be an array of at least 2 sources");
    }
    SECTION("from cylinder out of range") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["from"] = json::array({1, 9}); }, "out of range [1, 4]");
    }
    SECTION("from cylinder used twice across merges") {
        expect_exhaust_rejected(
            [](json& e) {
                e["merges"] = json::array({{{"from", json::array({1, 2})}, {"into", "a"}, {"length_m", 0.1}, {"d_m", 0.05}},
                                           {{"from", json::array({2, 3, 4})}, {"into", "flange"}, {"length_m", 0.1}, {"d_m", 0.05}}});
            },
            "is referenced by more than one merge");
    }
    SECTION("from name is a forward/unknown reference") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["from"] = json::array({"nope", 1}); },
                                 "is not an earlier merge's 'into'");
    }
    SECTION("from name double-consumed") {
        expect_exhaust_rejected(
            [](json& e) {
                e["merges"] = json::array({{{"from", json::array({1, 2})}, {"into", "a"}, {"length_m", 0.1}, {"d_m", 0.05}},
                                           {{"from", json::array({"a", 3})}, {"into", "b"}, {"length_m", 0.1}, {"d_m", 0.05}},
                                           {{"from", json::array({"a", 4})}, {"into", "flange"}, {"length_m", 0.1}, {"d_m", 0.05}}});
            },
            "is referenced by more than one merge");
    }
    SECTION("into missing") {
        expect_exhaust_rejected([](json& e) { e["merges"][0].erase("into"); }, "'into' is required");
    }
    SECTION("into empty") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["into"] = ""; }, "'into' is required");
    }
    SECTION("duplicate into name") {
        expect_exhaust_rejected(
            [](json& e) {
                e["merges"] = json::array({{{"from", json::array({1, 2})}, {"into", "flange"}, {"length_m", 0.1}, {"d_m", 0.05}},
                                           {{"from", json::array({3, 4})}, {"into", "flange"}, {"length_m", 0.1}, {"d_m", 0.05}}});
            },
            "duplicate 'into' name");
    }
    SECTION("merge non-positive length_m") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["length_m"] = 0.0; }, "'length_m' must be > 0");
    }
    SECTION("merge non-positive d_m") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["d_m"] = -1.0; }, "'d_m' must be > 0");
    }
    SECTION("cylinder coverage across merges wrong (a cylinder missing)") {
        expect_exhaust_rejected([](json& e) { e["merges"][0]["from"] = json::array({1, 2, 3}); },
                                 "must cover every cylinder exactly once across every merge's 'from'");
    }
    SECTION("dangling branch: a non-terminal 'into' never consumed") {
        expect_exhaust_rejected(
            [](json& e) {
                e["merges"] = json::array({{{"from", json::array({1, 2})}, {"into", "a"}, {"length_m", 0.1}, {"d_m", 0.05}},
                                           {{"from", json::array({3, 4})}, {"into", "flange"}, {"length_m", 0.1}, {"d_m", 0.05}}});
                e["interface"] = "flange";
            },
            "must be consumed by exactly one later merge");
    }
    SECTION("interface missing") {
        expect_exhaust_rejected([](json& e) { e.erase("interface"); }, "'interface' is required");
    }
    SECTION("interface empty") {
        expect_exhaust_rejected([](json& e) { e["interface"] = ""; }, "'interface' is required");
    }
    SECTION("interface does not match the last merge's into") {
        expect_exhaust_rejected([](json& e) { e["interface"] = "not_flange"; },
                                 "must equal the last merge's 'into'");
    }
    SECTION("missing source") {
        expect_exhaust_rejected([](json& e) { e.erase("source"); }, "'source' is required");
    }
}

// ---------------------------------------------------------------------------
// resolution block (CFL)
// ---------------------------------------------------------------------------

TEST_CASE("A4: resolution block rejections", "[unit][io][cycle][drivetrain]") {
    SECTION("unknown key") {
        expect_resolution_rejected([](json& r) { r["bogus"] = 1.0; }, "unknown key 'bogus'");
    }
    SECTION("power_dx_m below the CFL minimum") {
        expect_resolution_rejected([](json& r) { r["power_dx_m"] = 1e-6; }, "'power_dx_m'");
    }
    SECTION("audio_dx_exhaust_m below the CFL minimum") {
        expect_resolution_rejected([](json& r) { r["audio_dx_exhaust_m"] = 1e-6; }, "'audio_dx_exhaust_m'");
    }
    SECTION("audio_dx_intake_m below the CFL minimum") {
        expect_resolution_rejected([](json& r) { r["audio_dx_intake_m"] = 1e-6; }, "'audio_dx_intake_m'");
    }
    SECTION("non-positive power_dx_m") {
        expect_resolution_rejected([](json& r) { r["power_dx_m"] = 0.0; }, "'power_dx_m' must be > 0");
    }
}

// ---------------------------------------------------------------------------
// installation object (physics_sim.installation/1)
// ---------------------------------------------------------------------------

TEST_CASE("A4: installation object rejections", "[unit][io][cycle][drivetrain]") {
    SECTION("missing format") {
        expect_installation_rejected([](json& j) { j.erase("format"); }, "format");
    }
    SECTION("wrong format version") {
        expect_installation_rejected([](json& j) { j["format"] = "physics_sim.installation/2"; }, "format");
    }
    SECTION("unknown top-level key") {
        expect_installation_rejected([](json& j) { j["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("missing source") {
        expect_installation_rejected([](json& j) { j.erase("source"); }, "'source' is required");
    }
    SECTION("empty source") {
        expect_installation_rejected([](json& j) { j["source"] = ""; }, "'source' is required");
    }
    SECTION("intake missing") {
        expect_installation_rejected([](json& j) { j.erase("intake"); }, "'intake' object is required");
    }
    SECTION("intake not an object") {
        expect_installation_rejected([](json& j) { j["intake"] = 5.0; }, "'intake' object is required");
    }
    SECTION("intake unknown key") {
        expect_installation_rejected([](json& j) { j["intake"]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("intake missing airbox_m3") {
        expect_installation_rejected([](json& j) { j["intake"].erase("airbox_m3"); }, "'airbox_m3' is required");
    }
    SECTION("intake non-positive airbox_m3") {
        expect_installation_rejected([](json& j) { j["intake"]["airbox_m3"] = 0.0; }, "'airbox_m3' must be > 0");
    }
    SECTION("snorkel not an object") {
        expect_installation_rejected([](json& j) { j["intake"]["snorkel"] = 5.0; }, "must be an object");
    }
    SECTION("snorkel unknown key") {
        expect_installation_rejected([](json& j) { j["intake"]["snorkel"]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("snorkel non-positive length_m") {
        expect_installation_rejected([](json& j) { j["intake"]["snorkel"]["length_m"] = 0.0; }, "'length_m' must be > 0");
    }
    SECTION("snorkel non-positive d_m") {
        expect_installation_rejected([](json& j) { j["intake"]["snorkel"]["d_m"] = -1.0; }, "'d_m' must be > 0");
    }
    SECTION("missing inlet_position_m") {
        expect_installation_rejected([](json& j) { j["intake"].erase("inlet_position_m"); }, "'inlet_position_m' is required");
    }
    SECTION("exhaust missing") {
        expect_installation_rejected([](json& j) { j.erase("exhaust"); }, "'exhaust' object is required");
    }
    SECTION("exhaust not an object") {
        expect_installation_rejected([](json& j) { j["exhaust"] = 5.0; }, "'exhaust' object is required");
    }
    SECTION("exhaust unknown key") {
        expect_installation_rejected([](json& j) { j["exhaust"]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("exhaust missing elements") {
        expect_installation_rejected([](json& j) { j["exhaust"].erase("elements"); }, "'elements' is required");
    }
    SECTION("exhaust elements empty") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"] = json::array(); },
                                      "must be a non-empty array of elements");
    }
    SECTION("element not an object") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][0] = 5.0; },
                                      "each exhaust element must be an object");
    }
    SECTION("element missing type") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][0].erase("type"); },
                                      "exhaust element 'type' is required");
    }
    SECTION("element bad type string") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][0]["type"] = "turbocharger"; },
                                      "exhaust element 'type' must be");
    }
    SECTION("catalyst unknown key") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][0]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("catalyst missing length_m") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][0].erase("length_m"); }, "'length_m' is required");
    }
    SECTION("catalyst non-positive k_loss") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][0]["k_loss"] = 0.0; }, "'k_loss' must be > 0");
    }
    SECTION("pipe unknown key") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][1]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("pipe missing d_m") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][1].erase("d_m"); }, "'d_m' is required");
    }
    SECTION("outlet unknown key") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][2]["bogus"] = 1; }, "unknown key 'bogus'");
    }
    SECTION("outlet missing position_m") {
        expect_installation_rejected([](json& j) { j["exhaust"]["elements"][2].erase("position_m"); },
                                      "'position_m' is required");
    }
    SECTION("chain: an outlet must be the last element") {
        expect_installation_rejected(
            [](json& j) {
                json outlet = j["exhaust"]["elements"][2];
                j["exhaust"]["elements"] = json::array({outlet, j["exhaust"]["elements"][1]});
            },
            "must be the last element of its chain");
    }
    SECTION("chain: must end in an outlet or branch") {
        expect_installation_rejected(
            [](json& j) { j["exhaust"]["elements"] = json::array({j["exhaust"]["elements"][0], j["exhaust"]["elements"][1]}); },
            "must end in an 'outlet' or 'branch' element");
    }
    SECTION("branch unknown key") {
        expect_installation_rejected(
            [](json& j) {
                j["exhaust"]["elements"] = json::array(
                    {{{"type", "branch"},
                      {"branches", json::array({json::array({{{"type", "outlet"}, {"position_m", json::array({1.0, 0.0, 0.0})}}}),
                                                json::array({{{"type", "outlet"}, {"position_m", json::array({-1.0, 0.0, 0.0})}}})})},
                      {"bogus", 1}}});
            },
            "unknown key 'bogus'");
    }
    SECTION("branch fewer than 2 sub-chains") {
        expect_installation_rejected(
            [](json& j) {
                j["exhaust"]["elements"] = json::array(
                    {{{"type", "branch"},
                      {"branches", json::array({json::array({{{"type", "outlet"}, {"position_m", json::array({1.0, 0.0, 0.0})}}})})}}});
            },
            "'branches' must be an array of at least 2 element chains");
    }
    SECTION("branch with 2 sub-chains, each ending in an outlet, loads") {
        // Positive control for the branch-rejection SECTIONs above: NOT a
        // rejection - confirms the minimal valid branch shape actually loads.
        TempDir d;
        json inst = base_installation_json();
        inst["exhaust"]["elements"] = json::array(
            {{{"type", "branch"},
              {"branches", json::array({json::array({{{"type", "outlet"}, {"position_m", json::array({1.0, 0.0, 0.0})}}}),
                                        json::array({{{"type", "outlet"}, {"position_m", json::array({-1.0, 0.0, 0.0})}}})})}}});
        d.write("installation.json", inst);
        const vehicle::VehicleDesc v = load_with_engine(d, base_engine_json(/*with_installation_pipes=*/true));
        const InstallationDesc& li = *only_sim_engine(v).cycle->installation;
        REQUIRE(li.exhaust.elements.size() == 1);
        CHECK(li.exhaust.elements[0].type == ExhaustElementType::Branch);
        REQUIRE(li.exhaust.elements[0].branches.size() == 2);
    }
}

// ---------------------------------------------------------------------------
// vehicle-level per-engine "installation" override
// ---------------------------------------------------------------------------

TEST_CASE("A4: the vehicle-level 'installation' override is only allowed for an engine with pipe blocks",
          "[unit][io][cycle][drivetrain]") {
    TempDir d;
    d.write("installation.json", base_installation_json());
    json e = base_engine_json(/*with_installation_pipes=*/false); // no cycle.intake/exhaust
    e["installation"] = "installation.json";
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({e})));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(p.string()),
                        ContainsSubstring("'installation' is only allowed for an engine whose cycle block has "
                                          "'intake'/'exhaust' pipe blocks"));
}

TEST_CASE("A4: the vehicle-level 'installation' override is only allowed on a simulated engine",
          "[unit][io][cycle][drivetrain]") {
    // A torque_map engine (no "kind") cannot carry an 'installation' field at all.
    json e;
    e["id"] = "engine";
    e["type"] = "engine";
    e["wot_torque_nm_vs_rpm"] = {{"x", {0.0, 1000.0, 7000.0}}, {"y", {50.0, 100.0, 150.0}}};
    e["motoring_torque_nm_vs_rpm"] = {{"x", {0.0, 7000.0}}, {"y", {0.0, -20.0}}};
    e["idle_rpm"] = 800.0;
    e["inertia_kgm2"] = 0.15;
    e["breakaway_torque_nm"] = 20.0;
    e["installation"] = "installation.json";
    TempDir d;
    d.write("installation.json", base_installation_json());
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({e})));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(p.string()),
                        ContainsSubstring("'installation' is only allowed on a simulated engine with 'intake'/'exhaust' "
                                          "pipe blocks"));
}

TEST_CASE("A4: the vehicle-level 'installation' override must be a non-empty string", "[unit][io][cycle][drivetrain]") {
    TempDir d;
    d.write("installation.json", base_installation_json());
    json e = base_engine_json(/*with_installation_pipes=*/true);
    e["installation"] = "";
    const auto p = d.write("vehicle.json", vehicle_with_components(json::array({e})));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(p.string()), ContainsSubstring("'installation' must be a non-empty path string"));
}
