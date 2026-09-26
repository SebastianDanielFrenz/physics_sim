// tests/unit/test_fuel_io.cpp — coverage for ps::io::load_fuel_json
// (core/src/io/fuel_io.cpp), PHYS-033 P4 I7a C4: the physics_sim.fuel/1
// format's loader. Each shipped data/fuels/*.json file loads with its
// expected values, and every validation rule is exercised against a
// constructed bad file, following test_vehicle_io_validation.cpp's RAII
// temp-file fixture pattern (data/fuels/*.json is this project's real fuel
// set, not a place for one-off invalid fixtures).
//
// Windows ASan caveat (CLAUDE.md): every assertion here uses Catch's
// REQUIRE_THROWS_WITH/CHECK_THROWS_WITH matcher macros, never a manual
// try/catch that touches e.what() directly.

#include "ps/io/fuel_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

using namespace ps;
using Catch::Matchers::ContainsSubstring;

namespace {

// RAII temp file so a REQUIRE_THROWS case still cleans up (same pattern as
// test_vehicle_io_validation.cpp's TempJsonFile).
struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const std::string& contents, const char* tag) {
        path = std::filesystem::temp_directory_path() /
               (std::string("ps_test_fuel_io_") + tag + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
        std::ofstream out(path);
        out << contents;
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

// A minimal, otherwise-valid fuel JSON document, with one field's raw JSON
// text overridable so a thrown error can be attributed to that field alone.
std::string minimal_fuel_json(const std::string& format_field = R"("physics_sim.fuel/1")",
                               const std::string& name_field = R"("test_fuel")",
                               const std::string& phase_field = R"("liquid")",
                               const std::string& lhv_field = "42000000.0",
                               const std::string& afr_field = "14.5",
                               const std::string& density_field = "800.0",
                               const std::string& source_field = R"("test fixture, not a real fuel")") {
    return std::string(R"({
  "format": )") + format_field + R"(,
  "name": )" + name_field + R"(,
  "phase": )" + phase_field + R"(,
  "lhv_j_kg": )" + lhv_field + R"(,
  "afr_stoich": )" + afr_field + R"(,
  "density_kg_m3": )" + density_field + R"(,
  "source": )" + source_field + R"(
})";
}

} // namespace

TEST_CASE("load_fuel_json loads the shipped gasoline.json with expected values", "[unit][io][fuel]") {
    const drivetrain::FuelDesc d = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json");
    CHECK(d.name == "gasoline");
    CHECK(d.phase == drivetrain::FuelPhase::Liquid);
    CHECK(d.lhv_j_kg == Catch::Approx(44000000.0));
    CHECK(d.afr_stoich == Catch::Approx(14.5));
    CHECK(d.density_kg_m3 == Catch::Approx(747.5));
    CHECK_FALSE(d.source.empty());
}

TEST_CASE("load_fuel_json loads the shipped diesel.json with expected values", "[unit][io][fuel]") {
    const drivetrain::FuelDesc d = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/diesel.json");
    CHECK(d.name == "diesel");
    CHECK(d.phase == drivetrain::FuelPhase::Liquid);
    CHECK(d.lhv_j_kg == Catch::Approx(42000000.0));
    CHECK(d.afr_stoich == Catch::Approx(14.925373).margin(1e-6));
    CHECK(d.density_kg_m3 == Catch::Approx(830.0));
    CHECK_FALSE(d.source.empty());
}

TEST_CASE("load_fuel_json rejects a missing, outdated or unknown 'format'", "[unit][io][fuel][format]") {
    SECTION("missing entirely") {
        const std::string json = std::string(R"({
  "name": "test_fuel",
  "phase": "liquid",
  "lhv_j_kg": 42000000.0,
  "afr_stoich": 14.5,
  "density_kg_m3": 800.0,
  "source": "test fixture, not a real fuel"
})");
        TempJsonFile f(json, "missing_format");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("'format'"));
    }
    SECTION("wrong kind") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.tyre/1")"), "wrong_kind");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    SECTION("unknown future major (v2)") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/2")"), "future_major");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
}

TEST_CASE("load_fuel_json rejects an empty 'name'", "[unit][io][fuel]") {
    TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("")"), "empty_name");
    REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("'name'"));
}

TEST_CASE("load_fuel_json rejects a phase other than 'liquid'", "[unit][io][fuel]") {
    SECTION("gaseous (a plausible future phase, not yet supported)") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("gaseous")"), "phase_gaseous");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("not yet supported"));
    }
    SECTION("empty") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("")"), "phase_empty");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("not yet supported"));
    }
}

TEST_CASE("load_fuel_json rejects non-positive or non-finite lhv_j_kg", "[unit][io][fuel]") {
    SECTION("zero") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "0.0"), "lhv_zero");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("lhv_j_kg"));
    }
    SECTION("negative") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "-1000.0"), "lhv_negative");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("lhv_j_kg"));
    }
}

TEST_CASE("load_fuel_json rejects non-positive afr_stoich", "[unit][io][fuel]") {
    SECTION("zero") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "42000000.0", "0.0"), "afr_zero");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("afr_stoich"));
    }
    SECTION("negative") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "42000000.0", "-14.5"), "afr_negative");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("afr_stoich"));
    }
}

TEST_CASE("load_fuel_json rejects non-positive density_kg_m3", "[unit][io][fuel]") {
    SECTION("zero") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "42000000.0", "14.5", "0.0"), "density_zero");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("density_kg_m3"));
    }
    SECTION("negative") {
        TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "42000000.0", "14.5", "-800.0"), "density_negative");
        REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("density_kg_m3"));
    }
}

TEST_CASE("load_fuel_json rejects an empty 'source'", "[unit][io][fuel]") {
    TempJsonFile f(minimal_fuel_json(R"("physics_sim.fuel/1")", R"("test_fuel")", R"("liquid")", "42000000.0", "14.5", "800.0", R"("")"),
                    "empty_source");
    REQUIRE_THROWS_WITH(io::load_fuel_json(f.path.string()), ContainsSubstring("'source'"));
}
