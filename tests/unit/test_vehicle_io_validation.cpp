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
