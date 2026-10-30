// tests/unit/test_field_io.cpp - the soil/crop loaders (physics_sim.soil/1,
// physics_sim.crop/1), the field scenario loader and the headless scenario run.
//
// Recorded sabotages (F1a, 2026-10-31), made in core/src/field/io/*.cpp, the
// test run, then reverted:
//  - I-a: require_tags() made a no-op: the three "no source tag" cases failed
//         (a file without or with an untagged source loaded).
//  - I-b: has_tag() accepting any non-empty string: the "assumed" (no tag)
//         case failed.
//  - I-c: ObjectReader::finish() skipped for the soil water block: "unknown
//         key" failed.
//  - I-d: the scenario's "doy" range check removed: the doy case failed.
//  - I-e: run_field_scenario applying harvest rules after the step instead of
//         before it: the harvest-count/day checks failed.

#include "field_test_support.h"

#include "ps/field/field_scenario.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using nlohmann::json;

namespace {

json read_json(const std::filesystem::path& p) {
    std::ifstream in(p);
    REQUIRE(in.good());
    return json::parse(in);
}

json shipped_soil() { return read_json(test_field::data_dir() / "soils/silt_loam_loess.json"); }
json shipped_crop() { return read_json(test_field::data_dir() / "crops/winter_wheat.json"); }

struct TempDir {
    std::filesystem::path path;
    TempDir() {
        path = std::filesystem::temp_directory_path() /
               ("ps_field_io_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
    std::filesystem::path write(const std::string& name, const std::string& text) const {
        std::ofstream(path / name) << text;
        return path / name;
    }
};

} // namespace

TEST_CASE("I1 the shipped soils, crop and climate load", "[field][io]") {
    const SoilDesc& loess = test_field::loess();
    CHECK(loess.name == "silt_loam_loess");
    CHECK_THAT(loess.theta_fc, WithinAbs(0.30, 1e-15));
    CHECK_THAT(loess.theta_wp, WithinAbs(0.13, 1e-15));
    CHECK_THAT(loess.taw_mm_per_m(), WithinAbs(170.0, 1e-9));
    CHECK_THAT(loess.tew_mm(), WithinAbs(23.5, 1e-9));
    const SoilDesc& sandy = test_field::sandy();
    CHECK(sandy.theta_fc < loess.theta_fc);
    const CropDesc& wheat = test_field::wheat();
    CHECK(wheat.name == "winter_wheat");
    CHECK(wheat.stage_count() == 9);
    CHECK(wheat.stage_emergence == 1);
    CHECK(wheat.stage_flowering == 5);
    CHECK(wheat.stage_senescence == 6);
    CHECK(wheat.stage_maturity == 7);
    CHECK(wheat.stage_vernalisation_end == 3);
    CHECK(wheat.vernalises);
    CHECK_THAT(wheat.p_depletion, WithinAbs(0.55, 1e-15));
    CHECK(wheat.p_adjust_for_etc);
    CHECK(test_field::farm_climate().location.latitude_deg > 45.0);
}

TEST_CASE("I2 a parameter without a source tag is refused", "[field][io]") {
    {   // tag removed
        json j = shipped_soil();
        j["water"]["source"].erase("theta_fc");
        CHECK_THROWS_WITH(ps::io::parse_soil_json(j, "inline"), ContainsSubstring("theta_fc") && ContainsSubstring("source tag"));
    }
    {   // an entry that does not start with a tag
        json j = shipped_soil();
        j["water"]["source"]["theta_wp"] = "assumed from a textbook";
        CHECK_THROWS_WITH(ps::io::parse_soil_json(j, "inline"), ContainsSubstring("theta_wp") && ContainsSubstring("source tag"));
    }
    {   // the whole source object gone
        json j = shipped_soil();
        j["runoff"].erase("source");
        CHECK_THROWS_WITH(ps::io::parse_soil_json(j, "inline"), ContainsSubstring("source tag"));
    }
    {   // every valid tag is accepted
        for (const char* tag : {"[C] x", "[D] x", "[M] x", "[AS] x"}) {
            json j = shipped_soil();
            j["water"]["source"]["theta_wp"] = tag;
            CHECK_NOTHROW(ps::io::parse_soil_json(j, "inline"));
        }
    }
    {   // crop: a canopy parameter without a tag
        json j = shipped_crop();
        j["canopy"]["source"].erase("ccx");
        CHECK_THROWS_WITH(ps::io::parse_crop_json(j, "inline"), ContainsSubstring("ccx") && ContainsSubstring("source tag"));
    }
    {   // crop: the stages array needs one too
        json j = shipped_crop();
        j["phenology"]["source"].erase("stages");
        CHECK_THROWS_WITH(ps::io::parse_crop_json(j, "inline"), ContainsSubstring("stages") && ContainsSubstring("source tag"));
    }
    CHECK_NOTHROW(ps::io::parse_soil_json(shipped_soil(), "inline"));
    CHECK_NOTHROW(ps::io::parse_crop_json(shipped_crop(), "inline"));
}

TEST_CASE("I3 soil loader rejections", "[field][io]") {
    auto reject = [](auto mutate, const char* text) {
        json j = shipped_soil();
        mutate(j);
        INFO("expecting: " << text);
        CHECK_THROWS_WITH(ps::io::parse_soil_json(j, "inline"), ContainsSubstring(text));
    };
    reject([](json& j) { j["format"] = "physics_sim.soil/2"; }, "format");
    reject([](json& j) { j["water"]["foo"] = 1; j["water"]["source"]["foo"] = "[AS] x"; }, "unknown key");
    reject([](json& j) { j["bogus"] = 1; }, "unknown key");
    reject([](json& j) { j["water"]["theta_wp"] = 0.35; }, "theta_fc must exceed theta_wp");
    reject([](json& j) { j["water"]["theta_fc"] = "wet"; }, "must be a number");
    reject([](json& j) { j.erase("runoff"); }, "missing required field");
    reject([](json& j) { j["runoff"]["curve_number_ii"] = 120.0; }, "curve_number_ii");
    reject([](json& j) { j["evaporation"]["rew_mm"] = 30.0; }, "rew_mm");
    reject([](json& j) { j["water"]["fallow_root_depth_m"] = 2.5; }, "fallow_root_depth_m");
    reject([](json& j) { j["snow"]["melt_mm_per_k_day"] = -1.0; }, "snow_melt_mm_per_k_day");
    reject([](json& j) { j.erase("name"); }, "missing required field");
    CHECK_THROWS_WITH(ps::io::load_soil_json(test_field::data_dir() / "soils/does_not_exist.json"), ContainsSubstring("does_not_exist"));
}

TEST_CASE("I4 crop loader rejections", "[field][io]") {
    auto reject = [](auto mutate, const char* text) {
        json j = shipped_crop();
        mutate(j);
        INFO("expecting: " << text);
        CHECK_THROWS_WITH(ps::io::parse_crop_json(j, "inline"), ContainsSubstring(text));
    };
    reject([](json& j) { j["format"] = "physics_sim.crop/2"; }, "format");
    reject([](json& j) { j["phenology"]["vernalisation_until"] = "nowhere"; }, "names no stage");
    reject([](json& j) { j.erase("vernalisation"); }, "without a vernalisation block");
    reject([](json& j) { j["phenology"]["stages"][2]["gdd"] = 100; }, "ascending");
    reject([](json& j) { j["phenology"]["stages"][1]["name"] = "sprout"; }, "stages must exist");
    reject([](json& j) { j["canopy"]["ccx"] = 1.2; }, "ccx");
    reject([](json& j) { j["canopy"]["plant_density_per_ha"] = 3.0e8; }, "seedling canopy");
    reject([](json& j) { j["water"]["p_adjust_for_etc"] = 1; }, "boolean");
    reject([](json& j) { j["water"]["exp_p_upper"] = 0.9; }, "expansion");
    reject([](json& j) { j["frost"]["kill_full_c"] = -5.0; }, "frost kill");
    reject([](json& j) { j["roots"]["gdd_full"] = 100; }, "root_gdd_full");
    reject([](json& j) { j["ripeness"]["shed_rate_per_day"] = 1.5; }, "shed_rate_per_day");
    reject([](json& j) { j["extra"] = 3; }, "unknown key");
    // A crop without vernalisation is a legal crop (a spring crop).
    json spring = shipped_crop();
    spring.erase("vernalisation");
    spring["phenology"].erase("vernalisation_until");
    const CropDesc c = ps::io::parse_crop_json(spring, "inline");
    CHECK_FALSE(c.vernalises);
}

TEST_CASE("I5 extends: a child file patches its parent", "[field][io]") {
    TempDir dir;
    dir.write("base_soil.json", shipped_soil().dump());
    const auto child = dir.write("child_soil.json", R"({
        "format": "physics_sim.soil/1",
        "extends": "base_soil.json",
        "name": "child",
        "water": { "theta_fc": 0.28, "source": { "theta_fc": "[AS] child override" } }
    })");
    const SoilDesc s = ps::io::load_soil_json(child);
    CHECK(s.name == "child");
    CHECK_THAT(s.theta_fc, WithinAbs(0.28, 1e-15));
    CHECK_THAT(s.theta_wp, WithinAbs(0.13, 1e-15));          // from the parent
    CHECK_THAT(s.curve_number_ii, WithinAbs(75.0, 1e-15));
    // The override must itself be tagged.
    const auto untagged = dir.write("untagged_soil.json", R"({
        "format": "physics_sim.soil/1",
        "extends": "base_soil.json",
        "water": { "theta_fc": 0.27, "source": { "theta_fc": null } }
    })");
    CHECK_THROWS_WITH(ps::io::load_soil_json(untagged), ContainsSubstring("source tag"));
}

TEST_CASE("I6 field scenario loader", "[field][io]") {
    const auto path = test_field::data_dir() / "field_scenarios/wheat_10y.json";
    const ps::io::FieldScenario sc = ps::io::load_field_scenario(path);
    CHECK(sc.years == 10);
    CHECK(sc.seed == 11);
    CHECK(sc.field.nx == 4);
    CHECK(sc.field.ny == 4);
    CHECK(sc.field.soils.size() == 2);
    REQUIRE(sc.field.cell_soil.size() == 16);
    CHECK(sc.field.cell_soil[0] == 0);
    CHECK(sc.field.cell_soil[2] == 1);
    CHECK(sc.field.cell_soil[4] == 0);
    REQUIRE(sc.rules.size() == 2);
    CHECK(sc.rules[0].op == "sow");
    CHECK(sc.rules[0].crop_id == 1);
    CHECK(sc.rules[1].op == "harvest");
    CHECK(sc.rules[1].first_year == 1);

    TempDir dir;
    const std::string clim = (test_field::data_dir() / "climates/farm_de_home.json").generic_string();
    const std::string soil = (test_field::data_dir() / "soils/silt_loam_loess.json").generic_string();
    const std::string crop = (test_field::data_dir() / "crops/winter_wheat.json").generic_string();
    auto scenario = [&](const std::string& extra_rule, const std::string& extra_top = "") {
        std::ostringstream s;
        s << R"({"format": "physics_sim.field_scenario/1", "climate": ")" << clim << R"(", "seed": 1, "years": 1,
              "soils": [")" << soil << R"("], "crops": [")" << crop << R"("], "grid": {"nx": 1, "ny": 1})" << extra_top
          << R"(, "operations": [)" << extra_rule << "]}";
        return dir.write("sc.json", s.str());
    };
    CHECK_NOTHROW(ps::io::load_field_scenario(scenario(R"({"op": "sow", "crop": "winter_wheat", "doy": 288})")));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario(R"({"op": "plough", "doy": 10})")), ContainsSubstring("\"op\""));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario(R"({"op": "harvest", "doy": 400})")), ContainsSubstring("doy"));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario(R"({"op": "sow", "crop": "barley", "doy": 10})")),
                      ContainsSubstring("names no crop"));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario(R"({"op": "sow", "doy": 10})")), ContainsSubstring("crop"));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario(R"({"op": "harvest", "doy": 10, "every_years": 0})")),
                      ContainsSubstring("every_years"));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario("", R"(, "bogus": 1)")), ContainsSubstring("unknown key"));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario("", R"(, "soil_pattern": [3])")), ContainsSubstring("soil_pattern"));
    CHECK_THROWS_WITH(ps::io::load_field_scenario(scenario("", R"(, "years": 0)")), ContainsSubstring("years"));
}

TEST_CASE("I7 headless scenario run: harvests, CSV and determinism", "[field][io]") {
    ps::io::FieldScenario sc = ps::io::load_field_scenario(test_field::data_dir() / "field_scenarios/wheat_10y.json");
    sc.years = 3;
    sc.field.nx = 2;
    sc.field.ny = 2;
    sc.field.cell_soil.clear();
    sc.field.cell_soil = {0, 1, 0, 1};
    std::ostringstream csv;
    const ps::io::FieldRunSummary a = ps::io::run_field_scenario(sc, &csv);
    const ps::io::FieldRunSummary b = ps::io::run_field_scenario(sc, nullptr);
    CHECK(a.state_hash == b.state_hash);                 // writing the CSV changes nothing
    CHECK(a.max_balance_residual_mm < 1e-9);
    REQUIRE(a.harvests.size() == 2);                    // sown in years 0 and 1, harvested in years 1 and 2
    for (const auto& h : a.harvests) {
        CHECK(h.doy == 212);
        CHECK(h.result.grain_dm_t_ha > 2.0);
        CHECK(h.result.grain_dm_t_ha < 12.0);
        CHECK(h.result.bbch > 60.0);
    }
    CHECK(a.harvests[0].year == 1);
    CHECK(a.harvests[1].year == 2);
    // CSV: header plus one row per day.
    const std::string text = csv.str();
    const std::size_t lines = static_cast<std::size_t>(std::count(text.begin(), text.end(), '\n'));
    CHECK(lines == 3u * 365u + 1u);
    CHECK_THAT(text, ContainsSubstring("day,year,doy,tmin_c,tmax_c,rain_mm"));
    CHECK_THAT(text, ContainsSubstring(",ks_sto\n"));
}
