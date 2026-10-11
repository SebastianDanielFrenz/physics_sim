// tests/unit/test_turbo_io.cpp -- PHYS-033 P4 I7d D1-D4: the turbo configuration
// loader (ps::io::load_turbo_configuration_json, docs/data_formats/turbo.md):
// today's keys and rejections (D1 docs) and the new optional blocks
// (D2 blow_off_valve; D3/D4 extend this file). Every rejection is asserted by
// message. Configs are written to a temp directory and reference the shipped
// hardware file by absolute path.

#include "turbo_fixtures.h"
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/turbo_maps.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <string>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using json = nlohmann::json;
using Catch::Matchers::ContainsSubstring;
using ps::real;

std::string hardware_path() { return std::string(PS_SOURCE_DIR) + "/data/turbos/hyper_turbo_approx.json"; }

json base_config() {
    return json::parse(R"({
      "format": "physics_sim.turbo_configuration/1",
      "name": "io_test",
      "turbo": "HARDWARE",
      "count": 2,
      "target_boost_bar": 1.8,
      "intercooler_effectiveness": 0.95
    })");
}

json valid_valve() {
    return json::parse(R"({
      "vent": "recirculate", "crack_dp_bar": 0.9, "full_open_dp_bar": 1.6,
      "effective_area_mm2": 460, "tau_s": 0.04, "source": "[AS] io test"
    })");
}

std::string write_config(json cfg, const std::string& tag) {
    cfg["turbo"] = hardware_path();
    const auto dir = std::filesystem::temp_directory_path() / "ps_turbo_io_test";
    std::filesystem::create_directories(dir);
    const auto path = dir / (tag + ".json");
    std::ofstream(path) << cfg.dump(2);
    return path.string();
}

ps::drivetrain::TurboPairDesc load(json cfg, const std::string& tag) { return ps::io::load_turbo_configuration_json(write_config(std::move(cfg), tag)); }

} // namespace

TEST_CASE("Turbo loader: a valid configuration scales the extensive hardware keys by count", "[turbo][io]") {
    const auto d2 = load(base_config(), "base2");
    json c1 = base_config();
    c1["count"] = 1;
    const auto d1 = load(c1, "base1");
    // hyper_turbo_approx.json: rotor 2.3e-4 (derived, review fix 2026-10-05; was 2e-5 [AS]), max flow 0.6,
    // throat 5e-4, shaft loss 600, reference head 4.5 bar
    CHECK(d1.rotor_inertia_kgm2 == Catch::Approx(2.3e-4).epsilon(1e-12));
    CHECK(d2.rotor_inertia_kgm2 == Catch::Approx(4.6e-4).epsilon(1e-12));
    CHECK(d2.max_mass_flow_kg_s == Catch::Approx(1.2).epsilon(1e-12));
    CHECK(d2.turbine_area_m2 == Catch::Approx(1e-3).epsilon(1e-12));
    CHECK(d2.shaft_loss_w_at_reference == Catch::Approx(1200.0).epsilon(1e-12));
    CHECK(d2.reference_head_pa == Catch::Approx(4.5e5).epsilon(1e-12)); // intensive: not scaled
    CHECK(d2.reference_rpm == 120000.0);
    CHECK(d2.target_boost_pa == Catch::Approx(1.8e5).epsilon(1e-12));
    CHECK(d2.intercooler_effectiveness == 0.95);
    CHECK_FALSE(d2.blow_off_valve.has_value());
}

TEST_CASE("Turbo loader: rejections by message (today's keys)", "[turbo][io]") {
    SECTION("count above 8") {
        json c = base_config();
        c["count"] = 9;
        CHECK_THROWS_WITH(load(c, "count9"), ContainsSubstring("turbo count must be <= 8"));
    }
    SECTION("Pa keys were replaced by bar") {
        json c = base_config();
        c["target_boost_pa"] = 180000;
        CHECK_THROWS_WITH(load(c, "pa1"), ContainsSubstring("'target_boost_pa' was replaced by 'target_boost_bar' (gauge, bar)"));
    }
    SECTION("an efficiency above 1") {
        json c = base_config();
        c["intercooler_effectiveness"] = 1.2;
        CHECK_THROWS_WITH(load(c, "eff"), ContainsSubstring("turbo efficiencies must be <= 1"));
    }
    SECTION("a non-positive boost target") {
        json c = base_config();
        c["target_boost_bar"] = 0.0;
        CHECK_THROWS_WITH(load(c, "boost0"), ContainsSubstring("'target_boost_bar' must be > 0"));
    }
    SECTION("a negative derivative gain") {
        json c = base_config();
        c["boost_control"] = json{{"kd_s", -0.1}};
        CHECK_THROWS_WITH(load(c, "kd"), ContainsSubstring("'kd_s' must be >= 0"));
    }
    SECTION("boost_control is not an object") {
        json c = base_config();
        c["boost_control"] = 3;
        CHECK_THROWS_WITH(load(c, "bcobj"), ContainsSubstring("'boost_control' must be an object"));
    }
    SECTION("wrong format tag") {
        json c = base_config();
        c["format"] = "physics_sim.turbo_configuration/2";
        CHECK_THROWS_WITH(load(c, "fmt"), ContainsSubstring("unsupported 'format'"));
    }
}

TEST_CASE("D2 loader: the test fixture file equals the code-built valve", "[turbo][io][bov]") {
    const auto d = ps::io::load_turbo_configuration_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/turbo_config_hyper_bov.json");
    REQUIRE(d.blow_off_valve.has_value());
    const auto ref = ps::test::turbo::hyper_like_bov();
    CHECK(d.blow_off_valve->vent == ref.vent);
    CHECK(d.blow_off_valve->crack_dp_pa == Catch::Approx(ref.crack_dp_pa).epsilon(1e-12));
    CHECK(d.blow_off_valve->full_open_dp_pa == Catch::Approx(ref.full_open_dp_pa).epsilon(1e-12));
    CHECK(d.blow_off_valve->effective_area_m2 == Catch::Approx(ref.effective_area_m2).epsilon(1e-12));
    CHECK(d.blow_off_valve->tau_s == Catch::Approx(ref.tau_s).epsilon(1e-12));
}

TEST_CASE("D2 loader: blow_off_valve block", "[turbo][io][bov]") {
    SECTION("loads, pressures in bar to Pa, area per turbo times count") {
        json c = base_config();
        c["blow_off_valve"] = valid_valve();
        const auto d = load(c, "bov_ok");
        REQUIRE(d.blow_off_valve.has_value());
        const auto& v = *d.blow_off_valve;
        CHECK(v.vent == ps::drivetrain::BlowOffValveDesc::Vent::Recirculate);
        CHECK(v.crack_dp_pa == Catch::Approx(0.9e5).epsilon(1e-12));
        CHECK(v.full_open_dp_pa == Catch::Approx(1.6e5).epsilon(1e-12));
        CHECK(v.effective_area_m2 == Catch::Approx(2 * 460e-6).epsilon(1e-12)); // count 2
        CHECK(v.tau_s == Catch::Approx(0.04).epsilon(1e-12));
        CHECK(v.source == "[AS] io test");
        c["count"] = 1;
        c["blow_off_valve"]["vent"] = "atmosphere";
        c["blow_off_valve"]["tau_s"] = 0;
        const auto d1 = load(c, "bov_ok1");
        CHECK(d1.blow_off_valve->effective_area_m2 == Catch::Approx(460e-6).epsilon(1e-12));
        CHECK(d1.blow_off_valve->vent == ps::drivetrain::BlowOffValveDesc::Vent::Atmosphere);
        CHECK(d1.blow_off_valve->tau_s == 0.0);
    }
    SECTION("the block does not touch the derived-map cache key") {
        json c = base_config();
        const auto without = load(c, "key0");
        c["blow_off_valve"] = valid_valve();
        const auto with = load(c, "key1");
        CHECK(with.maps->content_hash == without.maps->content_hash);
    }
    SECTION("rejections") {
        const auto bad = [&](const std::string& key, const json& value, const std::string& message) {
            json c = base_config();
            c["blow_off_valve"] = valid_valve();
            if (value.is_null()) c["blow_off_valve"].erase(key);
            else c["blow_off_valve"][key] = value;
            CHECK_THROWS_WITH(load(c, "bov_bad"), ContainsSubstring(message));
        };
        bad("vent", nullptr, "'vent' must be \"atmosphere\" or \"recirculate\"");
        bad("vent", "sideways", "'vent' must be \"atmosphere\" or \"recirculate\", got \"sideways\"");
        bad("vent", 1, "'vent' must be \"atmosphere\" or \"recirculate\"");
        bad("crack_dp_bar", nullptr, "'crack_dp_bar' is required");
        bad("crack_dp_bar", -0.1, "'crack_dp_bar' must be >= 0");
        bad("full_open_dp_bar", nullptr, "'full_open_dp_bar' is required");
        bad("full_open_dp_bar", 0.9, "'full_open_dp_bar' must be > 'crack_dp_bar'");
        bad("full_open_dp_bar", 0.5, "'full_open_dp_bar' must be > 'crack_dp_bar'");
        bad("effective_area_mm2", nullptr, "'effective_area_mm2' is required");
        bad("effective_area_mm2", 0, "'effective_area_mm2' must be > 0");
        bad("effective_area_mm2", -5, "'effective_area_mm2' must be > 0");
        bad("tau_s", nullptr, "'tau_s' is required");
        bad("tau_s", -0.01, "'tau_s' must be >= 0");
        bad("source", nullptr, "'source' is required (PLAN.md section 8)");
        bad("source", "", "'source' is required (PLAN.md section 8)");
        bad("crack_dp_pa", 90000, "'crack_dp_pa' was replaced by 'crack_dp_bar' (bar)");
        bad("full_open_dp_pa", 160000, "'full_open_dp_pa' was replaced by 'full_open_dp_bar' (bar)");
        json c = base_config();
        c["blow_off_valve"] = 5;
        CHECK_THROWS_WITH(load(c, "bov_obj"), ContainsSubstring("'blow_off_valve' must be an object"));
    }
}

// ---------------------------------------------------------------------------
// D3 loader: the optional "surge_line" block of the hardware file.
// ---------------------------------------------------------------------------
namespace {
json valid_surge() {
    return json::parse(R"({
      "pressure_ratio": [1.0, 1.5, 2.0, 3.0], "flow_fraction": [0.04, 0.12, 0.22, 0.40], "source": "[AS] io test"
    })");
}
ps::drivetrain::TurboPairDesc load_surge(const json& surge, const std::string& tag) {
    std::ifstream in(hardware_path());
    json hw = json::parse(in);
    hw["surge_line"] = surge;
    const auto dir = std::filesystem::temp_directory_path() / "ps_turbo_io_test";
    std::filesystem::create_directories(dir);
    const auto hw_file = dir / ("hw_" + tag + ".json");
    std::ofstream(hw_file) << hw.dump(2);
    json cfg = base_config();
    cfg["turbo"] = hw_file.string();
    const auto cfg_file = dir / ("cfg_" + tag + ".json");
    std::ofstream(cfg_file) << cfg.dump(2);
    return ps::io::load_turbo_configuration_json(cfg_file.string());
}
} // namespace

TEST_CASE("D3 loader: surge_line block", "[turbo][io][surge]") {
    SECTION("loads; absent leaves the desc without a line") {
        const auto d = load_surge(valid_surge(), "surge_ok");
        REQUIRE(d.surge_line.has_value());
        const auto& c = d.surge_line->flow_fraction_vs_pr;
        REQUIRE(c.x.size() == 4);
        CHECK(c.x[3] == 3.0);
        CHECK(c.y[1] == 0.12);
        CHECK(c.evaluate(1.75) == Catch::Approx(0.17).epsilon(1e-12)); // linear between 1.5 and 2.0
        CHECK(d.surge_line->source == "[AS] io test");
        CHECK_FALSE(load(base_config(), "surge_absent").surge_line.has_value());
    }
    SECTION("the block does not touch the derived-map cache key") {
        const auto with = load_surge(valid_surge(), "surge_key");
        const auto without = load(base_config(), "surge_nokey");
        CHECK(with.maps->content_hash == without.maps->content_hash);
    }
    SECTION("rejections") {
        const auto bad = [&](const std::string& key, const json& value, const std::string& message) {
            json s = valid_surge();
            if (value.is_null()) s.erase(key);
            else s[key] = value;
            CHECK_THROWS_WITH(load_surge(s, "surge_bad"), ContainsSubstring(message));
        };
        bad("pressure_ratio", nullptr, "'pressure_ratio' must be an array of numbers");
        bad("flow_fraction", nullptr, "'flow_fraction' must be an array of numbers");
        bad("pressure_ratio", json::array({1.0, "x", 2.0, 3.0}), "'pressure_ratio' must be an array of numbers");
        bad("pressure_ratio", json::array({1.0}), "needs at least 2 points");
        bad("flow_fraction", json::array({0.04, 0.12, 0.22}), "'pressure_ratio' and 'flow_fraction' must have the same length");
        bad("pressure_ratio", json::array({0.9, 1.5, 2.0, 3.0}), "'pressure_ratio' must start at >= 1");
        bad("pressure_ratio", json::array({1.0, 1.5, 1.5, 3.0}), "'pressure_ratio' must be strictly increasing");
        bad("pressure_ratio", json::array({1.0, 2.0, 1.5, 3.0}), "'pressure_ratio' must be strictly increasing");
        bad("flow_fraction", json::array({0.04, 0.12, 1.0, 0.40}), "'flow_fraction' must be in [0, 1)");
        bad("flow_fraction", json::array({0.04, -0.1, 0.22, 0.40}), "'flow_fraction' must be in [0, 1)");
        bad("source", nullptr, "'source' is required (PLAN.md section 8)");
        bad("source", "", "'source' is required (PLAN.md section 8)");
        CHECK_THROWS_WITH(load_surge(json(7), "surge_obj"), ContainsSubstring("'surge_line' must be an object"));
    }
}

// ---------------------------------------------------------------------------
// D4 loader: the optional "intercooler_pressure_drop_bar_at_max_flow" key.
// ---------------------------------------------------------------------------
TEST_CASE("D4 loader: intercooler pressure drop", "[turbo][io][intercooler]") {
    SECTION("loads in bar, intensive (not scaled by count), absent = 0") {
        json c = base_config();
        CHECK(load(c, "dp_absent").intercooler_dp_pa_at_max_flow == 0.0);
        c["intercooler_pressure_drop_bar_at_max_flow"] = 0.25;
        CHECK(load(c, "dp_2").intercooler_dp_pa_at_max_flow == Catch::Approx(25000.0).epsilon(1e-12));
        c["count"] = 1;
        CHECK(load(c, "dp_1").intercooler_dp_pa_at_max_flow == Catch::Approx(25000.0).epsilon(1e-12));
        c["intercooler_pressure_drop_bar_at_max_flow"] = 0;
        CHECK(load(c, "dp_zero").intercooler_dp_pa_at_max_flow == 0.0);
    }
    SECTION("the key does not touch the derived-map cache key") {
        json c = base_config();
        const auto without = load(c, "dpkey0");
        c["intercooler_pressure_drop_bar_at_max_flow"] = 0.25;
        CHECK(load(c, "dpkey1").maps->content_hash == without.maps->content_hash);
    }
    SECTION("rejections") {
        json c = base_config();
        c["intercooler_pressure_drop_bar_at_max_flow"] = -0.1;
        CHECK_THROWS_WITH(load(c, "dp_neg"), ContainsSubstring("'intercooler_pressure_drop_bar_at_max_flow' must be >= 0"));
        c["intercooler_pressure_drop_bar_at_max_flow"] = "high";
        CHECK_THROWS_WITH(load(c, "dp_str"), ContainsSubstring("'intercooler_pressure_drop_bar_at_max_flow' must be a number"));
        c["intercooler_pressure_drop_bar_at_max_flow"] = 4.5; // = the hardware's reference_head_bar
        CHECK_THROWS_WITH(load(c, "dp_big"), ContainsSubstring("must be < the hardware 'reference_head_bar'"));
    }
}

// ---------------------------------------------------------------------------
// N7b: a vehicle's engine component may override the engine file's
// default_forced_induction ("forced_induction", a path relative to the
// VEHICLE file; docs/data_formats/vehicle.md), modelled on "installation".
// Engine garage P2 renamed the keys (default_turbo_configuration ->
// default_forced_induction, turbo_configuration -> forced_induction); the old
// names are rejected with a rename message.
// The fixtures are the real car_hyper.json / hyper_v8_one1_audio.json with
// every "../x.json" path made absolute and written to a scratch directory
// (a copy next to nothing keeps relative paths from resolving), loaded at
// Preview quality with a disk map cache.
// ---------------------------------------------------------------------------
namespace {

std::string data_dir() { return std::string(PS_SOURCE_DIR) + "/data"; }

// "../fuels/e5.json" -> "<abs base>/fuels/e5.json" (collapsed), for every string that starts with "../".
void absolutize(json& j, const std::filesystem::path& base) {
    if (j.is_string()) {
        const std::string v = j.get<std::string>();
        if (v.rfind("../", 0) == 0) j = std::filesystem::weakly_canonical(base / v).generic_string();
    } else if (j.is_object() || j.is_array()) {
        for (auto& e : j) absolutize(e, base);
    }
}

json read_json(const std::string& path) {
    std::ifstream in(path);
    return json::parse(in);
}

struct OverrideFixture {
    std::filesystem::path dir;
    std::string cfg_path; // an alternate turbo configuration (target 1.6 bar, no boost_control block)
    json vehicle;         // car_hyper.json, absolutized; the engine component is "ref" -> the stock engine file
    json engine;          // hyper_v8_one1_audio.json, absolutized
    OverrideFixture() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_turbo_vehicle_override_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::create_directories(dir);
        json c = base_config();
        c["target_boost_bar"] = 1.6;
        cfg_path = write_config(c, "vehicle_override_cfg");
        vehicle = read_json(data_dir() + "/vehicles/car_hyper.json");
        absolutize(vehicle, std::filesystem::path(data_dir()) / "vehicles");
        engine = read_json(data_dir() + "/engines/hyper_v8_one1_audio.json");
        absolutize(engine, std::filesystem::path(data_dir()) / "engines");
    }
    ~OverrideFixture() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    static json& engine_component(json& v) {
        for (auto& c : v["powertrain"]["components"])
            if (c.value("type", "") == "engine") return c;
        throw std::runtime_error("no engine component");
    }
    std::string write(const std::string& name, const json& j) const {
        const auto p = dir / name;
        std::ofstream(p) << j.dump(2);
        return p.string();
    }
    ps::vehicle::VehicleDesc load(const json& v, const std::string& name) const {
        static ps::jobs::JobSystem pool(4);
        ps::io::EngineMapOptions opt;
        opt.quality = ps::drivetrain::MapQuality::Preview;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_turbo_vehicle_override_cache").string();
        return ps::io::load_vehicle_json(write(name, v), opt);
    }
};

const ps::drivetrain::SimulatedEngineDesc& sim_engine(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* e = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) return *e;
    throw std::runtime_error("no simulated engine");
}

std::uint64_t hash_after(const ps::vehicle::VehicleDesc& d, int ticks, int workers) {
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500.0, ps::Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, workers);
    rig.world->set_control("throttle", 0.6);
    for (int i = 0; i < ticks; ++i) rig.world->step();
    return rig.world->state_hash();
}

std::string default_config_path() {
    return std::filesystem::weakly_canonical(std::filesystem::path(data_dir()) / "turbo_configurations/hyper_twin_default.json").generic_string();
}

} // namespace

TEST_CASE("N7b: a vehicle-level 'forced_induction' equals putting the path into the engine file", "[turbo][io][vehicle]") {
    OverrideFixture f;
    // (a) the override on the vehicle's engine component, the stock engine file.
    json va = f.vehicle;
    OverrideFixture::engine_component(va)["forced_induction"] = f.cfg_path;
    const auto da = f.load(va, "va.json");
    // (b) the same path as the engine file's own default_forced_induction, no override.
    json e = f.engine;
    e["default_forced_induction"] = f.cfg_path;
    json vb = f.vehicle;
    OverrideFixture::engine_component(vb)["ref"] = f.write("engine_b.json", e);
    const auto db = f.load(vb, "vb.json");
    // (c) the stock car (engine default), (d) the stock default named as an explicit override.
    const auto dc = f.load(f.vehicle, "vc.json");
    json vd = f.vehicle;
    OverrideFixture::engine_component(vd)["forced_induction"] = default_config_path();
    const auto dd = f.load(vd, "vd.json");
    // (e) the repository car loaded directly: a file without the new key loads as before.
    ps::io::EngineMapOptions opt;
    opt.quality = ps::drivetrain::MapQuality::Preview;
    opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_turbo_vehicle_override_cache").string();
    const auto de = ps::io::load_vehicle_json(data_dir() + "/vehicles/car_hyper.json", opt);

    const auto& ea = sim_engine(da);
    const auto& eb = sim_engine(db);
    const auto& ec = sim_engine(dc);
    REQUIRE(ea.turbo_pair.has_value());
    REQUIRE(eb.turbo_pair.has_value());
    REQUIRE(ec.turbo_pair.has_value());
    // The override really replaced the default (else the equalities below would be vacuous).
    CHECK(ea.turbo_pair->target_boost_pa == Catch::Approx(1.6e5).epsilon(1e-12));
    CHECK(ec.turbo_pair->target_boost_pa == Catch::Approx(1.8e5).epsilon(1e-12));
    // (a) == (b): same turbo desc, same hashed state.
    CHECK(ea.turbo_pair->target_boost_pa == eb.turbo_pair->target_boost_pa);
    CHECK(ea.turbo_pair->wastegate_area() == eb.turbo_pair->wastegate_area());
    CHECK(ea.turbo_pair->rotor_inertia_kgm2 == eb.turbo_pair->rotor_inertia_kgm2);
    REQUIRE(ea.turbo_pair->maps != nullptr);
    REQUIRE(eb.turbo_pair->maps != nullptr);
    CHECK(ea.turbo_pair->maps->content_hash == eb.turbo_pair->maps->content_hash);
    const std::uint64_t ha1 = hash_after(da, 20, 1);
    CHECK(ha1 == hash_after(db, 20, 1));
    CHECK(ha1 != hash_after(dc, 20, 1)); // a different target boost moves the engine state
    // No override = bit-identical (and naming the default explicitly changes nothing).
    const std::uint64_t hc1 = hash_after(dc, 20, 1);
    CHECK(hc1 == hash_after(dd, 20, 1));
    CHECK(hc1 == hash_after(de, 20, 1));
    CHECK(hc1 == hash_after(dc, 20, 10)); // worker-count invariant
}

TEST_CASE("N7b: a vehicle-level 'forced_induction' is rejected by message", "[turbo][io][vehicle]") {
    OverrideFixture f;
    SECTION("an empty string") {
        json v = f.vehicle;
        OverrideFixture::engine_component(v)["forced_induction"] = "";
        CHECK_THROWS_WITH(f.load(v, "r1.json"), ContainsSubstring("'forced_induction' must be a non-empty path string"));
    }
    SECTION("not a string") {
        json v = f.vehicle;
        OverrideFixture::engine_component(v)["forced_induction"] = 3;
        CHECK_THROWS_WITH(f.load(v, "r2.json"), ContainsSubstring("'forced_induction' must be a non-empty path string"));
    }
    SECTION("a torque_map engine") {
        json v = f.vehicle;
        auto& c = OverrideFixture::engine_component(v);
        c["ref"] = std::filesystem::weakly_canonical(std::filesystem::path(data_dir()) / "engines/i4_2l_na.json").generic_string();
        c["forced_induction"] = f.cfg_path;
        CHECK_THROWS_WITH(f.load(v, "r4.json"), ContainsSubstring("'forced_induction' is only allowed on a simulated engine"));
    }
    SECTION("a missing configuration file") {
        json v = f.vehicle;
        OverrideFixture::engine_component(v)["forced_induction"] = (f.dir / "does_not_exist.json").generic_string();
        CHECK_THROWS(f.load(v, "r5.json"));
    }
    SECTION("the pre-P2 vehicle key 'turbo_configuration'") {
        json v = f.vehicle;
        OverrideFixture::engine_component(v)["turbo_configuration"] = f.cfg_path;
        CHECK_THROWS_WITH(f.load(v, "r6.json"), ContainsSubstring("'turbo_configuration' was renamed 'forced_induction'"));
    }
    SECTION("the pre-P2 engine key 'default_turbo_configuration'") {
        json e = f.engine;
        e["default_turbo_configuration"] = e["default_forced_induction"];
        e.erase("default_forced_induction");
        json v = f.vehicle;
        OverrideFixture::engine_component(v)["ref"] = f.write("engine_oldkey.json", e);
        CHECK_THROWS_WITH(f.load(v, "r7.json"), ContainsSubstring("'default_turbo_configuration' was renamed 'default_forced_induction'"));
    }
}

// Engine garage P2: a vehicle may now ADD forced induction to a simulated engine whose file has none (N7b
// rejected that). The result equals the engine file naming the same configuration as its default.
TEST_CASE("garage P2: a vehicle-level 'forced_induction' on an engine without a default", "[turbo][io][vehicle]") {
    OverrideFixture f;
    json e = f.engine;
    e.erase("default_forced_induction");
    json v = f.vehicle;
    OverrideFixture::engine_component(v)["ref"] = f.write("engine_noturbo.json", e);
    OverrideFixture::engine_component(v)["forced_induction"] = f.cfg_path;
    const auto da = f.load(v, "add_fi.json");
    json eb = f.engine;
    eb["default_forced_induction"] = f.cfg_path;
    json vb = f.vehicle;
    OverrideFixture::engine_component(vb)["ref"] = f.write("engine_b2.json", eb);
    const auto db = f.load(vb, "add_fi_b.json");
    const auto& ea = sim_engine(da);
    const auto& ebd = sim_engine(db);
    REQUIRE(ea.turbo_pair.has_value());
    REQUIRE(ebd.turbo_pair.has_value());
    CHECK(ea.turbo_pair->target_boost_pa == Catch::Approx(1.6e5).epsilon(1e-12));
    REQUIRE(ea.turbo_pair->maps != nullptr);
    REQUIRE(ebd.turbo_pair->maps != nullptr);
    CHECK(ea.turbo_pair->maps->content_hash == ebd.turbo_pair->maps->content_hash);
    CHECK(hash_after(da, 20, 1) == hash_after(db, 20, 1));
}
