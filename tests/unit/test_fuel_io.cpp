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

// Ethanol blend volume fractions (v/v ethanol.json in gasoline.json), for
// the mixing-rule recomputation cross-check below - PHYS-033 P4 I7a C5c's
// own mixing rule: rho = v*rho_e + (1-v)*rho_g; w_e = v*rho_e/rho;
// LHV = w_e*LHV_e + (1-w_e)*LHV_g; AFR_s = w_e*AFR_e + (1-w_e)*AFR_g.
struct BlendFile {
    const char* file;
    const char* name;
    real volume_fraction;
};
constexpr BlendFile kBlendFiles[] = {
    {"/data/fuels/e5.json", "e5", real(0.05)},
    {"/data/fuels/e10.json", "e10", real(0.10)},
    {"/data/fuels/e25.json", "e25", real(0.25)},
    {"/data/fuels/e85.json", "e85", real(0.85)},
};

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

TEST_CASE("load_fuel_json loads the shipped ethanol.json (E100 blending component) with expected values",
          "[unit][io][fuel]") {
    const drivetrain::FuelDesc d = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/ethanol.json");
    CHECK(d.name == "ethanol");
    CHECK(d.phase == drivetrain::FuelPhase::Liquid);
    CHECK(d.lhv_j_kg == Catch::Approx(26800000.0));
    CHECK(d.afr_stoich == Catch::Approx(9.0));
    CHECK(d.density_kg_m3 == Catch::Approx(794.0));
    CHECK_FALSE(d.source.empty());
}

TEST_CASE("load_fuel_json loads every shipped ethanol blend file (E5/E10/E25/E85)", "[unit][io][fuel]") {
    for (const BlendFile& blend : kBlendFiles) {
        SECTION(blend.name) {
            const drivetrain::FuelDesc d = io::load_fuel_json(std::string(PS_SOURCE_DIR) + blend.file);
            CHECK(d.name == blend.name);
            CHECK(d.phase == drivetrain::FuelPhase::Liquid);
            CHECK(d.lhv_j_kg > real(0.0));
            CHECK(d.afr_stoich > real(0.0));
            CHECK(d.density_kg_m3 > real(0.0));
            CHECK_FALSE(d.source.empty());
        }
    }
}

// PHYS-033 P4 I7a C5c: each blend file's shipped values must equal THIS
// test's own recomputation of the mixing rule (rho = v*rho_e + (1-v)*rho_g;
// w_e = v*rho_e/rho; LHV = w_e*LHV_e + (1-w_e)*LHV_g; AFR_s = w_e*AFR_e +
// (1-w_e)*AFR_g) from the loaded gasoline.json/ethanol.json values - never a
// hand-copied constant, so a corrupted blend file or a wrong mixing formula
// both get caught here.
TEST_CASE("ethanol blend files match this test's own mixing-rule recomputation", "[unit][io][fuel]") {
    const drivetrain::FuelDesc gasoline = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json");
    const drivetrain::FuelDesc ethanol = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/ethanol.json");

    for (const BlendFile& blend : kBlendFiles) {
        SECTION(blend.name) {
            const drivetrain::FuelDesc d = io::load_fuel_json(std::string(PS_SOURCE_DIR) + blend.file);
            const real v = blend.volume_fraction;
            const real rho = v * ethanol.density_kg_m3 + (real(1.0) - v) * gasoline.density_kg_m3;
            const real w_e = v * ethanol.density_kg_m3 / rho;
            const real lhv = w_e * ethanol.lhv_j_kg + (real(1.0) - w_e) * gasoline.lhv_j_kg;
            const real afr = w_e * ethanol.afr_stoich + (real(1.0) - w_e) * gasoline.afr_stoich;

            CHECK(d.density_kg_m3 == Catch::Approx(rho).epsilon(1e-6));
            CHECK(d.lhv_j_kg == Catch::Approx(lhv).epsilon(1e-6));
            CHECK(d.afr_stoich == Catch::Approx(afr).epsilon(1e-6));
        }
    }
}

TEST_CASE("ethanol blend densities fall inside their cited EN density ranges", "[unit][io][fuel]") {
    // These are sanity cross-checks, not validations: E5/E10 stay inside
    // 720-775 for any v <= 0.59, and E85 is >= 755 for any v >= 0.16. The
    // discriminating test is the recompute (above).
    //
    // EN 228:2012 unleaded petrol density range at 15 degC (also gasoline.json's
    // own cited range) - E5/E10 are still primarily gasoline.
    SECTION("e5, e10 inside EN 228:2012 720.0-775.0 kg/m^3") {
        const drivetrain::FuelDesc e5 = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/e5.json");
        const drivetrain::FuelDesc e10 = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/e10.json");
        CHECK(e5.density_kg_m3 >= real(720.0));
        CHECK(e5.density_kg_m3 <= real(775.0));
        CHECK(e10.density_kg_m3 >= real(720.0));
        CHECK(e10.density_kg_m3 <= real(775.0));
    }
    // EN 15293:2018 E85 density range at 15 degC.
    SECTION("e85 inside EN 15293:2018 755-800 kg/m^3") {
        const drivetrain::FuelDesc e85 = io::load_fuel_json(std::string(PS_SOURCE_DIR) + "/data/fuels/e85.json");
        CHECK(e85.density_kg_m3 >= real(755.0));
        CHECK(e85.density_kg_m3 <= real(800.0));
    }
    // No e25 SECTION here: E25 has no EU density standard to check against,
    // and a [720, 800] bound (the gasoline-to-E85 span) is a tautology -
    // it passes for every v in [0, 1], so it discriminates nothing. See
    // data/fuels/e25.json's own "Range cross-check: none" note.
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
