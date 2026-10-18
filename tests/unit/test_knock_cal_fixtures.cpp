// tests/unit/test_knock_cal_fixtures.cpp - knock calibration phase A: the four EPA NVFEL turbo engine fixtures
// (tests/support/fixtures/epa_turbo/<engine>/engine.json, written by tools/enginedata/epa_turbo_engines.py) and the six
// EPA test-fuel files (data/fuels/epa_*.json, tools/enginedata/epa_turbo_fuels.py) load, and every fixture generates
// Sketch maps with a knock-limited spark table per listed fuel. Smoke only: no accuracy claim, the fit is phase B.
//
// KC1 fuels: each epa_* file loads and carries the RON/MON printed in its EPA NVFEL Fuel Analysis Report.
// KC2 engines: each fixture loads at Sketch quality (generation on), is a boosted piston engine with the knock opt-in,
//     one theta_kl table per listed fuel, finite and non-empty, and the extracted rows file exists with a row count > 0.
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/fuel_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>

using namespace ps;
using namespace ps::drivetrain;
using json = nlohmann::json;

namespace {

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);
std::filesystem::path fixture_dir() { return kSrc / "tests" / "support" / "fixtures" / "epa_turbo"; }

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

struct FuelRow {
    const char* file;
    double ron, mon;
};

} // namespace

TEST_CASE("KC1 the EPA test-fuel files load with their report RON/MON", "[knock_cal][io][drivetrain]") {
    const FuelRow rows[] = {
        {"epa_tier2_25278", 96.6, 88.9}, {"epa_tier2_25842", 96.1, 88.5}, {"epa_tier2_26864", 97.6, 89.0},
        {"epa_levIII_24670", 92.4, 84.3}, {"epa_tier3_25206", 91.0, 83.5}, {"epa_tier3_25402", 97.8, 88.4},
    };
    for (const FuelRow& r : rows) {
        const FuelDesc f = io::load_fuel_json((kSrc / "data" / "fuels" / (std::string(r.file) + ".json")).generic_string());
        INFO(r.file);
        REQUIRE(f.has_knock);
        CHECK(f.knock.ron == r.ron);
        CHECK(f.knock.mon == r.mon);
        CHECK(f.lhv_j_kg > 4.0e7);
        CHECK(f.lhv_j_kg < 4.5e7);
        CHECK(f.afr_stoich > 13.9);
        CHECK(f.afr_stoich < 14.7);
    }
}

TEST_CASE("KC2 the EPA turbo engine fixtures load and generate Sketch maps", "[knock_cal][io][drivetrain][cycle]") {
    struct Eng {
        const char* dir;
        const char* rows;
        size_t fuels;
    };
    const Eng engines[] = {
        {"honda_l15b7", "honda_l15b7_t3_rows.json", 2},
        {"ford_27", "ford_27_t2_cell9_rows.json", 4},
        {"ford_16", "ford_16_t2_rows.json", 2},
        {"mazda_25t", "mazda_25t_t2_rows.json", 2},
    };
    for (const Eng& e : engines) {
        INFO(e.dir);
        io::EngineMapOptions o;
        o.quality = MapQuality::Sketch;
        o.pool = &pool();
        o.generate = true;
        ComponentParams p;
        try {
            p = io::load_engine_json((fixture_dir() / e.dir / "engine.json").generic_string(), o);
        } catch (const std::exception& ex) {
            FAIL_CHECK(std::string(e.dir) + ": " + ex.what());
            continue;
        }
        const auto* d = std::get_if<SimulatedEngineDesc>(&p);
        REQUIRE(d != nullptr);
        REQUIRE(d->cycle.has_value());
        CHECK(d->cycle->knock.enabled);
        CHECK(d->cycle->knock.fuels.size() == e.fuels);
        REQUIRE(d->maps.has_theta_kl);
        REQUIRE(d->maps.theta_kl_by_fuel.size() == e.fuels);
        for (const auto& t : d->maps.theta_kl_by_fuel) {
            REQUIRE_FALSE(t.values.empty());
            for (const real v : t.values) CHECK(std::isfinite(v));
        }
        std::ifstream rows(fixture_dir() / e.rows);
        REQUIRE(rows.good());
        const json j = json::parse(rows);
        CHECK(j.at("rows").size() > 0);
    }
}
