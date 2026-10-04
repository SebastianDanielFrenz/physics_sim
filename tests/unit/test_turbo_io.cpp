// tests/unit/test_turbo_io.cpp -- PHYS-033 P4 I7d D1-D4: the turbo configuration
// loader (ps::io::load_turbo_configuration_json, docs/data_formats/turbo.md):
// today's keys and rejections (D1 docs) and the new optional blocks
// (D2 blow_off_valve; D3/D4 extend this file). Every rejection is asserted by
// message. Configs are written to a temp directory and reference the shipped
// hardware file by absolute path.

#include "turbo_fixtures.h"

#include "ps/drivetrain/turbo_maps.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
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
    // hyper_turbo_approx.json: rotor 2e-5, max flow 0.6, throat 5e-4, shaft loss 600, reference head 4.5 bar
    CHECK(d1.rotor_inertia_kgm2 == Catch::Approx(2e-5).epsilon(1e-12));
    CHECK(d2.rotor_inertia_kgm2 == Catch::Approx(4e-5).epsilon(1e-12));
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
