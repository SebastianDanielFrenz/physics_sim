// tests/unit/test_vehicle_io_validation.cpp — error-path coverage for
// ps::io::load_vehicle_json's SteeringDesc::ackermann_fraction validation
// (core/src/io/vehicle_io.cpp's read_steering, added alongside the
// anti-Ackermann support documented in vehicle_desc.h's
// SteeringDesc::ackermann_fraction comment). [-1, 1] is the accepted
// range - 1 = full geometric Ackermann, -1 = full anti-Ackermann; anything
// further out overshoots the geometric correction the formula is built
// from and is far more likely a data-entry mistake than an intentional
// setting (see that comment for the full justification).
//
// Writes a minimal, otherwise-valid vehicle JSON to a temp file per case
// (data/vehicles/*.json is this project's real vehicle fleet, not a place
// for one-off invalid fixtures) rather than reusing car_sedan.json, so
// each case is self-contained and the only thing varying is
// steering.ackermann_fraction.

#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

using namespace ps;

namespace {

// Minimal vehicle JSON satisfying every OTHER load_vehicle_json requirement
// (non-empty 'source', a non-empty 'wheels' array, a non-empty
// drive.torque_curve_nm_vs_rpm) so a thrown error can only be attributed to
// the ackermann_fraction check under test.
std::string minimal_vehicle_json(double ackermann_fraction) {
    return R"({
  "format": "physics_sim.vehicle/1",
  "source": "test fixture, not a real vehicle",
  "wheels": [
    { "name": "FL", "steered": true, "is_front": true },
    { "name": "FR", "steered": true, "is_front": true }
  ],
  "steering": { "ackermann_fraction": )" + std::to_string(ackermann_fraction) + R"( },
  "drive": { "torque_curve_nm_vs_rpm": { "x": [900.0], "y": [100.0] } }
})";
}

// RAII temp file so a REQUIRE_THROWS case still cleans up.
struct TempVehicleJson {
    std::filesystem::path path;
    explicit TempVehicleJson(double ackermann_fraction) {
        path = std::filesystem::temp_directory_path() /
               ("ps_test_vehicle_io_validation_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
        std::ofstream out(path);
        out << minimal_vehicle_json(ackermann_fraction);
    }
    ~TempVehicleJson() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

} // namespace

TEST_CASE("load_vehicle_json rejects ackermann_fraction outside [-1, 1]", "[unit][vehicle][io]") {
    SECTION("above 1") {
        TempVehicleJson f(1.2);
        REQUIRE_THROWS_AS(io::load_vehicle_json(f.path.string()), std::runtime_error);
        CHECK_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("ackermann_fraction"));
    }
    SECTION("below -1") {
        TempVehicleJson f(-1.5);
        REQUIRE_THROWS_AS(io::load_vehicle_json(f.path.string()), std::runtime_error);
        CHECK_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("ackermann_fraction"));
    }
}

TEST_CASE("load_vehicle_json accepts ackermann_fraction at both range endpoints and 0", "[unit][vehicle][io]") {
    for (double f : {-1.0, 0.0, 1.0}) {
        TempVehicleJson tf(f);
        vehicle::VehicleDesc desc;
        REQUIRE_NOTHROW(desc = io::load_vehicle_json(tf.path.string()));
        CHECK(desc.steering.ackermann_fraction == f);
    }
}

// PLAN.md P4 I0 (PHYS-033 section 9 / owner ruling Q7: strict "format"
// checks, no legacy support). A minimal RAII temp file per case, same
// spirit as TempVehicleJson above but with the whole document supplied by
// the test (rather than just an ackermann_fraction) since these cases vary
// the "format" field itself.
struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const std::string& contents, const char* tag) {
        path = std::filesystem::temp_directory_path() /
               (std::string("ps_test_format_") + tag + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
        std::ofstream out(path);
        out << contents;
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

std::string vehicle_json_with_format(const std::string& format_field) {
    return std::string(R"({
)") + format_field + R"(
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "drive": { "torque_curve_nm_vs_rpm": { "x": [900.0], "y": [100.0] } }
})";
}

TEST_CASE("load_vehicle_json rejects a missing, unknown or outdated 'format'", "[unit][vehicle][io][format]") {
    using Catch::Matchers::ContainsSubstring;

    SECTION("missing entirely") {
        TempJsonFile f(vehicle_json_with_format(""), "vehicle_missing");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("'format'"));
    }
    SECTION("wrong kind") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.tyre/1",)"), "vehicle_wrongkind");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    SECTION("outdated major") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.vehicle/0",)"), "vehicle_outdated");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    SECTION("unknown future major") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.vehicle/2",)"), "vehicle_future");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
}

std::string tyre_json_with_format(const std::string& format_field) {
    return std::string("{\n") + format_field + R"(
  "name": "fixture_tyre",
  "source": "test fixture, not a real tyre"
})";
}

TEST_CASE("load_tyre_json rejects a missing, unknown or outdated 'format'", "[unit][vehicle][io][format]") {
    using Catch::Matchers::ContainsSubstring;

    SECTION("missing entirely") {
        TempJsonFile f(tyre_json_with_format(""), "tyre_missing");
        REQUIRE_THROWS_WITH(io::load_tyre_json(f.path.string()), ContainsSubstring("'format'"));
    }
    SECTION("wrong kind") {
        TempJsonFile f(tyre_json_with_format(R"("format": "physics_sim.vehicle/1",)"), "tyre_wrongkind");
        REQUIRE_THROWS_WITH(io::load_tyre_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    SECTION("outdated major") {
        TempJsonFile f(tyre_json_with_format(R"("format": "physics_sim.tyre/0",)"), "tyre_outdated");
        REQUIRE_THROWS_WITH(io::load_tyre_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
}

TEST_CASE("load_vehicle_json accepts a 'format'-less inline tyre object (not a standalone file)",
          "[unit][vehicle][io][format]") {
    // An inline tyre object embedded in a vehicle file has no independent
    // version of its own (section 9: only FILES carry "format") - only a
    // standalone tyre JSON referenced by path does.
    const std::string json = R"({
  "format": "physics_sim.vehicle/1",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL", "tyre": { "name": "inline_tyre", "source": "inline fixture" } } ],
  "drive": { "torque_curve_nm_vs_rpm": { "x": [900.0], "y": [100.0] } }
})";
    TempJsonFile f(json, "vehicle_inline_tyre");
    REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
}
