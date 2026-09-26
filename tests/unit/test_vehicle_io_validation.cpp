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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <variant>

using namespace ps;

namespace {

// Minimal vehicle JSON satisfying every OTHER load_vehicle_json requirement
// (non-empty 'source', a non-empty 'wheels' array) so a thrown error can
// only be attributed to the ackermann_fraction check under test.
// PHYS-033 P4 I2 commit 5a: /2 - no 'drive'/'brakes' block needed, both are
// gone and 'powertrain'/'brake_system' are optional (no powertrain at all).
std::string minimal_vehicle_json(double ackermann_fraction) {
    return R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [
    { "name": "FL", "steered": true, "is_front": true },
    { "name": "FR", "steered": true, "is_front": true }
  ],
  "steering": { "ackermann_fraction": )" + std::to_string(ackermann_fraction) + R"( }
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
  "wheels": [ { "name": "FL" } ]
})";
}

TEST_CASE("load_vehicle_json rejects a missing, outdated or unknown 'format'", "[unit][vehicle][io][format]") {
    using Catch::Matchers::ContainsSubstring;

    SECTION("missing entirely") {
        TempJsonFile f(vehicle_json_with_format(""), "vehicle_missing");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("'format'"));
    }
    SECTION("wrong kind") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.tyre/1",)"), "vehicle_wrongkind");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    SECTION("outdated major (v0)") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.vehicle/0",)"), "vehicle_outdated0");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
    // PHYS-033 P4 I2 plan section 9.1/13 row 5a's own check: "a /1 file
    // rejected" - v1 is now the outdated legacy format (owner ruling Q7,
    // no converter), and the loader appends a pointer to the /2 layout.
    SECTION("v1 rejected (PHYS-033 Q7: no legacy converter)") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.vehicle/1",)"), "vehicle_v1");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("no longer supported"));
    }
    SECTION("unknown future major (v3)") {
        TempJsonFile f(vehicle_json_with_format(R"("format": "physics_sim.vehicle/3",)"), "vehicle_future");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("unsupported 'format'"));
    }
}

// PHYS-033 P4 I2 plan section 9.1/13 row 5a's own check: the /1 fields the
// /2 loader now rejects outright (owner ruling Q7: no converter, no silent
// default - "driven"/"drive"/"brakes" are hard errors, not ignored keys).
TEST_CASE("load_vehicle_json rejects legacy /1 fields under /2", "[unit][vehicle][io][format]") {
    using Catch::Matchers::ContainsSubstring;

    SECTION("wheel 'driven' is rejected") {
        const std::string json = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL", "driven": true } ]
})";
        TempJsonFile f(json, "vehicle_driven");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("driven"));
    }
    SECTION("top-level 'drive' is rejected") {
        const std::string json = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "drive": { "final_drive_ratio": 3.7 }
})";
        TempJsonFile f(json, "vehicle_drive_key");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("'drive'"));
    }
    SECTION("top-level 'brakes' is rejected") {
        const std::string json = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "brakes": { "max_torque_front": 2200.0 }
})";
        TempJsonFile f(json, "vehicle_brakes_key");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("'brakes'"));
    }
}

// PHYS-033 P4 I3 plan section 5.4/9.1: I3 LIFTS I2's "differential
// efficiency must be exactly 1" ban - any value in (0, 1] is now accepted
// (the old I2-only test this replaced asserted the opposite and is
// obsolete). Still rejects out-of-range values.
TEST_CASE("load_vehicle_json accepts differential efficiency < 1 (I2 ban lifted) but rejects out-of-range", "[unit][vehicle][io][drivetrain]") {
    auto json_with_eff = [](double eff) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 1.0, "efficiency": )") +
               std::to_string(eff) + R"( }
    ],
    "shafts": [
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    SECTION("0.9 is accepted") {
        TempJsonFile f(json_with_eff(0.9), "vehicle_diff_eff_ok");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("0 is rejected") {
        TempJsonFile f(json_with_eff(0.0), "vehicle_diff_eff_zero");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("efficiency"));
    }
    SECTION("above 1 is rejected") {
        TempJsonFile f(json_with_eff(1.1), "vehicle_diff_eff_over");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("efficiency"));
    }
}

// PHYS-033 P4 I4 C5 (plan section 3.1): "lockable" (default false) and the
// optional "lock" block. A "lock" block on a non-lockable diff and a
// negative engage mismatch are both load-time errors.
TEST_CASE("load_vehicle_json parses a lockable differential and rejects a bad 'lock' block", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_diff_fields = [](const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 1.0)") +
               extra + R"( }
    ],
    "shafts": [
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    auto only_diff = [](const vehicle::VehicleDesc& d) -> const drivetrain::DifferentialDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* diff = std::get_if<drivetrain::DifferentialDesc>(&d.powertrain.components[0].params);
        REQUIRE(diff != nullptr);
        return *diff;
    };

    SECTION("defaults: not lockable") {
        TempJsonFile f(json_with_diff_fields(""), "vehicle_diff_lock_default");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK_FALSE(only_diff(d).lockable);
    }
    SECTION("lockable with a full lock block is accepted") {
        TempJsonFile f(json_with_diff_fields(R"(, "lockable": true, "lock": { "max_engage_mismatch_rad_s": 0.5, "channel": "rear_lock" })"),
                       "vehicle_diff_lock_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::DifferentialDesc& diff = only_diff(d);
        CHECK(diff.lockable);
        CHECK(diff.lock.max_engage_mismatch_rad_s == 0.5);
        CHECK(diff.lock.channel == "rear_lock");
    }
    SECTION("lockable without a lock block takes the defaults") {
        TempJsonFile f(json_with_diff_fields(R"(, "lockable": true)"), "vehicle_diff_lock_defaults");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::DifferentialDesc& diff = only_diff(d);
        CHECK(diff.lockable);
        CHECK(diff.lock.max_engage_mismatch_rad_s == 2.0);
        CHECK(diff.lock.channel == "diff_lock");
    }
    SECTION("'lock' without 'lockable': true is rejected") {
        TempJsonFile f(json_with_diff_fields(R"(, "lock": { "max_engage_mismatch_rad_s": 2.0 })"), "vehicle_diff_lock_not_lockable");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("lockable"));
    }
    SECTION("negative max_engage_mismatch_rad_s is rejected") {
        TempJsonFile f(json_with_diff_fields(R"(, "lockable": true, "lock": { "max_engage_mismatch_rad_s": -0.1 })"),
                       "vehicle_diff_lock_negative");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("max_engage_mismatch_rad_s"));
    }
}

// PHYS-033 P4 I4 C6 (plan section 3.1): differential "kind" gains "locked"
// and "viscous" alongside "open"; "active" is a named-but-rejected
// placeholder with its own clear message (not the generic "unknown kind"
// case), and viscous_nm_s_rad is validated against the kind it's attached
// to. ("clutch_lsd"/"torsen" were placeholders here too until I4 C7 - see
// the next test case.)
TEST_CASE("load_vehicle_json validates differential 'kind' (I4 C6: locked/viscous)", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_diff_fields = [](const std::string& kind, const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "diff", "type": "differential", "kind": ")") +
               kind + R"(", "ratio": 1.0)" + extra + R"( }
    ],
    "shafts": [
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    auto only_diff = [](const vehicle::VehicleDesc& d) -> const drivetrain::DifferentialDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* diff = std::get_if<drivetrain::DifferentialDesc>(&d.powertrain.components[0].params);
        REQUIRE(diff != nullptr);
        return *diff;
    };

    SECTION("kind \"locked\" is accepted, no viscous_nm_s_rad needed") {
        TempJsonFile f(json_with_diff_fields("locked", ""), "vehicle_diff_kind_locked");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK(only_diff(d).kind == drivetrain::DifferentialKind::Locked);
    }
    SECTION("kind \"viscous\" with a positive viscous_nm_s_rad is accepted") {
        TempJsonFile f(json_with_diff_fields("viscous", R"(, "viscous_nm_s_rad": 50.0)"), "vehicle_diff_kind_viscous_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::DifferentialDesc& diff = only_diff(d);
        CHECK(diff.kind == drivetrain::DifferentialKind::Viscous);
        CHECK(diff.viscous_nm_s_rad == 50.0);
    }
    SECTION("unknown kind is rejected") {
        TempJsonFile f(json_with_diff_fields("torque_vectoring_magic", ""), "vehicle_diff_kind_unknown");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("kind"));
    }
    SECTION("kind \"active\" is rejected with its own message") {
        TempJsonFile f(json_with_diff_fields("active", ""), "vehicle_diff_kind_active");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                            ContainsSubstring("torque-vectoring controller"));
    }
    SECTION("kind \"viscous\" without viscous_nm_s_rad is rejected") {
        TempJsonFile f(json_with_diff_fields("viscous", ""), "vehicle_diff_kind_viscous_missing");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("viscous_nm_s_rad"));
    }
    SECTION("kind \"viscous\" with viscous_nm_s_rad <= 0 is rejected") {
        TempJsonFile f(json_with_diff_fields("viscous", R"(, "viscous_nm_s_rad": 0.0)"), "vehicle_diff_kind_viscous_zero");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("viscous_nm_s_rad"));
    }
    SECTION("viscous_nm_s_rad on a non-viscous kind is rejected") {
        TempJsonFile f(json_with_diff_fields("open", R"(, "viscous_nm_s_rad": 50.0)"), "vehicle_diff_kind_open_viscous_field");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("viscous_nm_s_rad"));
    }
    SECTION("lockable: true on kind \"locked\" is rejected") {
        TempJsonFile f(json_with_diff_fields("locked", R"(, "lockable": true)"), "vehicle_diff_kind_locked_lockable");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring("lockable"));
    }
}

// PHYS-033 P4 I4 C7 (plan section 3.2): "clutch_lsd" (preload_nm >= 0,
// required ramp {drive_factor, coast_factor} each in [0, 1), optional
// static_factor >= 1) and "torsen" (required torque_bias_ratio > 1); every
// one of those fields is rejected on any other kind.
TEST_CASE("load_vehicle_json validates clutch_lsd/torsen differential fields (I4 C7)", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_diff_fields = [](const std::string& kind, const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "diff", "type": "differential", "kind": ")") +
               kind + R"(", "ratio": 1.0)" + extra + R"( }
    ],
    "shafts": [
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    auto only_diff = [](const vehicle::VehicleDesc& d) -> const drivetrain::DifferentialDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* diff = std::get_if<drivetrain::DifferentialDesc>(&d.powertrain.components[0].params);
        REQUIRE(diff != nullptr);
        return *diff;
    };
    auto rejects = [](const std::string& text, const char* tag, const std::string& needle) {
        TempJsonFile f(text, tag);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(needle));
    };
    const std::string ramp = R"(, "ramp": { "drive_factor": 0.4, "coast_factor": 0.2 })";

    SECTION("clutch_lsd with every field is accepted and round-trips") {
        TempJsonFile f(json_with_diff_fields("clutch_lsd", R"(, "preload_nm": 60.0, "static_factor": 1.2)" + ramp),
                       "vehicle_diff_lsd_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::DifferentialDesc& diff = only_diff(d);
        CHECK(diff.kind == drivetrain::DifferentialKind::ClutchLsd);
        CHECK(diff.preload_nm == 60.0);
        CHECK(diff.drive_factor == 0.4);
        CHECK(diff.coast_factor == 0.2);
        CHECK(diff.static_factor == 1.2);
    }
    SECTION("clutch_lsd defaults: preload_nm 0, static_factor 1") {
        TempJsonFile f(json_with_diff_fields("clutch_lsd", ramp), "vehicle_diff_lsd_defaults");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK(only_diff(d).preload_nm == 0.0);
        CHECK(only_diff(d).static_factor == 1.0);
    }
    SECTION("clutch_lsd may be lockable") {
        TempJsonFile f(json_with_diff_fields("clutch_lsd", ramp + R"(, "lockable": true)"), "vehicle_diff_lsd_lockable");
        CHECK(only_diff(io::load_vehicle_json(f.path.string())).lockable);
    }
    SECTION("clutch_lsd without a ramp block is rejected") {
        rejects(json_with_diff_fields("clutch_lsd", R"(, "preload_nm": 60.0)"), "vehicle_diff_lsd_no_ramp", "requires a 'ramp' block");
    }
    SECTION("clutch_lsd ramp missing coast_factor is rejected") {
        rejects(json_with_diff_fields("clutch_lsd", R"(, "ramp": { "drive_factor": 0.4 })"), "vehicle_diff_lsd_ramp_half",
                "requires both 'drive_factor' and 'coast_factor'");
    }
    SECTION("clutch_lsd ramp factors outside [0, 1) are rejected") {
        rejects(json_with_diff_fields("clutch_lsd", R"(, "ramp": { "drive_factor": 1.0, "coast_factor": 0.2 })"),
                "vehicle_diff_lsd_drive_1", "ramp.drive_factor must be in [0, 1)");
        rejects(json_with_diff_fields("clutch_lsd", R"(, "ramp": { "drive_factor": 0.4, "coast_factor": -0.1 })"),
                "vehicle_diff_lsd_coast_neg", "ramp.coast_factor must be in [0, 1)");
    }
    SECTION("clutch_lsd negative preload / static_factor < 1 are rejected") {
        rejects(json_with_diff_fields("clutch_lsd", R"(, "preload_nm": -1.0)" + ramp), "vehicle_diff_lsd_preload_neg",
                "'preload_nm' must be >= 0");
        rejects(json_with_diff_fields("clutch_lsd", R"(, "static_factor": 0.9)" + ramp), "vehicle_diff_lsd_static_lt1",
                "'static_factor' must be >= 1");
    }
    SECTION("torsen with torque_bias_ratio > 1 is accepted") {
        TempJsonFile f(json_with_diff_fields("torsen", R"(, "torque_bias_ratio": 3.0)"), "vehicle_diff_torsen_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK(only_diff(d).kind == drivetrain::DifferentialKind::Torsen);
        CHECK(only_diff(d).torque_bias_ratio == 3.0);
    }
    SECTION("torsen without / with torque_bias_ratio <= 1 is rejected") {
        rejects(json_with_diff_fields("torsen", ""), "vehicle_diff_torsen_missing", "requires 'torque_bias_ratio' > 1");
        rejects(json_with_diff_fields("torsen", R"(, "torque_bias_ratio": 1.0)"), "vehicle_diff_torsen_eq1",
                "'torque_bias_ratio' must be > 1");
    }
    SECTION("fields of another kind are rejected") {
        rejects(json_with_diff_fields("clutch_lsd", ramp + R"(, "torque_bias_ratio": 3.0)"), "vehicle_diff_lsd_tbr",
                "'torque_bias_ratio' is only valid for kind \"torsen\"");
        rejects(json_with_diff_fields("open", R"(, "torque_bias_ratio": 3.0)"), "vehicle_diff_open_tbr",
                "'torque_bias_ratio' is only valid for kind \"torsen\"");
        rejects(json_with_diff_fields("torsen", R"(, "torque_bias_ratio": 3.0, "viscous_nm_s_rad": 5.0)"),
                "vehicle_diff_torsen_visc", "'viscous_nm_s_rad' is only valid for kind \"viscous\"");
        rejects(json_with_diff_fields("torsen", R"(, "torque_bias_ratio": 3.0)" + ramp), "vehicle_diff_torsen_ramp",
                "'ramp' is only valid for kind \"clutch_lsd\"");
        rejects(json_with_diff_fields("open", R"(, "preload_nm": 10.0)"), "vehicle_diff_open_preload",
                "'preload_nm' is only valid for kind \"clutch_lsd\"");
        rejects(json_with_diff_fields("viscous", R"(, "viscous_nm_s_rad": 5.0, "static_factor": 1.1)"),
                "vehicle_diff_visc_static", "'static_factor' is only valid for kind \"clutch_lsd\"");
        rejects(json_with_diff_fields("clutch_lsd", ramp + R"(, "viscous_nm_s_rad": 5.0)"), "vehicle_diff_lsd_visc",
                "'viscous_nm_s_rad' is only valid for kind \"viscous\"");
    }
}

// PHYS-033 P4 I4 C9 (plan section 3.3): the MFWD transfer_case component -
// a rigid `rear` row plus a friction `front` row (the AWD clutch). Mirrors
// the differential validation tests above: a minimal standalone powertrain
// block (no engine/gearbox needed - Powertrain's graph-build only needs the
// component's own ports to resolve, and every rejection here fires before
// any cross-component wiring is checked).
TEST_CASE("load_vehicle_json validates transfer_case fields (I4 C9)", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_tc_fields = [](const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 1.0,
        "clutch": { "capacity_nm": 2500.0, "static_factor": 1.2, "engage_time_s": 0.3 })") +
               extra + R"( }
    ],
    "shafts": [
      { "ports": ["transfer.rear", "wheel:FL"] },
      { "ports": ["transfer.front", "wheel:FR"] }
    ]
  }
})";
    };
    auto only_tc = [](const vehicle::VehicleDesc& d) -> const drivetrain::TransferCaseDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* tc = std::get_if<drivetrain::TransferCaseDesc>(&d.powertrain.components[0].params);
        REQUIRE(tc != nullptr);
        return *tc;
    };
    auto rejects = [](const std::string& text, const char* tag, const std::string& needle) {
        TempJsonFile f(text, tag);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(needle));
    };

    SECTION("a well-formed transfer_case round-trips") {
        TempJsonFile f(json_with_tc_fields(""), "vehicle_tc_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::TransferCaseDesc& tc = only_tc(d);
        CHECK(tc.rear_ratio == 1.0);
        CHECK(tc.front_ratio == 1.0);
        CHECK(tc.clutch.capacity_nm == 2500.0);
        CHECK(tc.clutch.static_factor == 1.2);
        CHECK(tc.clutch.engage_time_s == 0.3);
        CHECK(tc.channel == "awd");
    }
    SECTION("channel defaults to \"awd\" and may be overridden") {
        TempJsonFile f(json_with_tc_fields(R"(, "channel": "my_awd")"), "vehicle_tc_channel");
        CHECK(only_tc(io::load_vehicle_json(f.path.string())).channel == "my_awd");
    }
    SECTION("rear_ratio == 0 is rejected") {
        rejects(std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 0.0, "front_ratio": 1.0,
        "clutch": { "capacity_nm": 2500.0 } }
    ],
    "shafts": [ { "ports": ["transfer.rear", "wheel:FL"] }, { "ports": ["transfer.front", "wheel:FR"] } ]
  }
})"),
                "vehicle_tc_rear_ratio_zero", "'rear_ratio' must be != 0");
    }
    SECTION("front_ratio == 0 is rejected") {
        rejects(std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 0.0,
        "clutch": { "capacity_nm": 2500.0 } }
    ],
    "shafts": [ { "ports": ["transfer.rear", "wheel:FL"] }, { "ports": ["transfer.front", "wheel:FR"] } ]
  }
})"),
                "vehicle_tc_front_ratio_zero", "'front_ratio' must be != 0");
    }
    SECTION("missing 'clutch' block is rejected") {
        rejects(std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 1.0 }
    ],
    "shafts": [ { "ports": ["transfer.rear", "wheel:FL"] }, { "ports": ["transfer.front", "wheel:FR"] } ]
  }
})"),
                "vehicle_tc_no_clutch", "requires a 'clutch' block");
    }
    SECTION("clutch.capacity_nm <= 0 is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 1.0,
        "clutch": { "capacity_nm": 0.0 } }
    ],
    "shafts": [ { "ports": ["transfer.rear", "wheel:FL"] }, { "ports": ["transfer.front", "wheel:FR"] } ]
  }
})";
        rejects(bad, "vehicle_tc_cap_zero", "clutch.capacity_nm must be > 0");
    }
    SECTION("clutch.static_factor < 1 is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 1.0,
        "clutch": { "capacity_nm": 2500.0, "static_factor": 0.9 } }
    ],
    "shafts": [ { "ports": ["transfer.rear", "wheel:FL"] }, { "ports": ["transfer.front", "wheel:FR"] } ]
  }
})";
        rejects(bad, "vehicle_tc_static_lt1", "clutch.static_factor must be >= 1");
    }
    SECTION("clutch.engage_time_s < 0 is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 1.0,
        "clutch": { "capacity_nm": 2500.0, "engage_time_s": -0.1 } }
    ],
    "shafts": [ { "ports": ["transfer.rear", "wheel:FL"] }, { "ports": ["transfer.front", "wheel:FR"] } ]
  }
})";
        rejects(bad, "vehicle_tc_engage_neg", "clutch.engage_time_s must be >= 0");
    }
    SECTION("empty channel is rejected") {
        rejects(json_with_tc_fields(R"(, "channel": "")"), "vehicle_tc_channel_empty", "'channel' must be non-empty");
    }
}

// --- PHYS-033 P4 I3 plan section 9.2 ----------------------------------------

namespace {
// A minimal but otherwise-complete I3 powertrain block (inline engine/
// clutch/gearbox/differential, no external refs) - every one of section
// 9.2's rejection cases below starts from this and breaks exactly one rule.
std::string minimal_i3_engine_json() {
    return R"({ "id": "engine", "type": "engine", "kind": "torque_map",
      "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
      "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
      "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 })";
}
std::string minimal_i3_vehicle_json(const std::string& engine_component, const std::string& gearbox_component,
                                     const std::string& extra_controller_fields = std::string()) {
    return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      )") + engine_component + R"(,
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )" + gearbox_component + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  })" + (extra_controller_fields.empty() ? std::string() : (",\n  \"controllers\": [ " + extra_controller_fields + " ]")) +
           R"(
})";
}
std::string minimal_i3_gearbox_json() {
    return R"({ "id": "gearbox", "type": "gearbox", "forward_ratios": [3.0, 1.6, 1.0],
      "forward_efficiency": [1.0, 1.0, 1.0] })";
}
// A gearbox is normally 'ref'd to a standalone physics_sim.gearbox/1 file
// (both shipped vehicles do this - inline gearbox components are not a
// real usage pattern the loader needs to support losslessly, since the
// gearbox's OWN "type": "manual"|"sequential" field would collide with the
// component-dispatch "type": "gearbox" key on the very same inline JSON
// object). `gearbox_body` is the CONTENTS between the {} of a
// physics_sim.gearbox/1 file (type/forward_ratios/forward_efficiency/...).
struct RefGearbox {
    TempJsonFile file;
    std::string component_json;
    RefGearbox(const std::string& gearbox_body, const char* tag)
        : file(std::string(R"({ "format": "physics_sim.gearbox/1", "name": "fixture", )") + gearbox_body + "}", tag),
          component_json(std::string(R"({ "id": "gearbox", "type": "gearbox", "ref": ")") + file.path.filename().string() + R"(" })") {}
};
} // namespace

TEST_CASE("load_vehicle_json rejects an I2-style engine (no motoring curve, or hard_cut limiter)", "[unit][vehicle][io][drivetrain]") {
    SECTION("missing motoring_torque_nm_vs_rpm") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] }, "inertia_kgm2": 0.16 })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_i2_engine_no_motoring");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("motoring_torque_nm_vs_rpm"));
    }
    SECTION("hard_cut limiter") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
          "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
          "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0, "limiter": { "type": "hard_cut", "rpm": 6500 } })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_i2_engine_hard_cut");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("hard_cut"));
    }
}

TEST_CASE("load_vehicle_json rejects a component ref object with an extra key", "[unit][vehicle][io][drivetrain]") {
    const std::string engine = R"({ "id": "engine", "type": "engine", "ref": "does_not_matter.json", "kind": "torque_map" })";
    const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
    TempJsonFile f(json, "vehicle_ref_extra_key");
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("ref"));
}

TEST_CASE("load_vehicle_json rejects a ref to a /2 engine format file", "[unit][vehicle][io][drivetrain]") {
    // An engine ref file with the WRONG format string (mimicking a future/
    // unsupported major version) - require_format's own generic message.
    const std::string bad_engine_file = R"({ "format": "physics_sim.engine/2", "name": "bad", "kind": "torque_map",
      "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
      "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] }, "inertia_kgm2": 0.16 })";
    TempJsonFile engine_file(bad_engine_file, "engine_v2_ref_target");
    const std::string engine = std::string(R"({ "id": "engine", "type": "engine", "ref": ")") +
                                engine_file.path.filename().string() + R"(" })";
    const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
    TempJsonFile f(json, "vehicle_ref_v2_engine");
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("format"));
}

TEST_CASE("load_vehicle_json rejects non-monotone gearbox ratios", "[unit][vehicle][io][drivetrain]") {
    // 3.0 -> 3.5 is NOT strictly decreasing.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 3.5, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_nonmonotone_ref");
    const std::string json = minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json);
    TempJsonFile f(json, "vehicle_gearbox_nonmonotone");
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("forward_ratios"));
}

TEST_CASE("load_vehicle_json rejects mismatched gearbox array lengths", "[unit][vehicle][io][drivetrain]") {
    // forward_efficiency length 2, forward_ratios length 3.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0])",
                  "gearbox_mismatched_len_ref");
    const std::string json = minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json);
    TempJsonFile f(json, "vehicle_gearbox_mismatched_len");
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("forward_efficiency"));
}

TEST_CASE("load_vehicle_json rejects a manual_tcu naming an unknown gearbox", "[unit][vehicle][io][drivetrain]") {
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_tcu_unknown_ref");
    const std::string controller = R"({ "id": "tcu", "type": "manual_tcu", "gearbox": "no_such_gearbox",
      "clutch": "clutch", "engine": "engine" })";
    const std::string json = minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json, controller);
    TempJsonFile f(json, "vehicle_tcu_unknown_gearbox");
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("no_such_gearbox"));
}

TEST_CASE("load_vehicle_json rejects a non-positive auto_clutch sync_time_s or max_sync_torque_nm", "[unit][vehicle][io][drivetrain]") {
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_tcu_sync_params");
    const auto controller = [](const char* auto_clutch_fields) {
        return std::string(R"({ "id": "tcu", "type": "manual_tcu", "gearbox": "gearbox", "clutch": "clutch", "engine": "engine",
      "auto_clutch": { )") + auto_clutch_fields + " } }";
    };
    {
        TempJsonFile f(minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json, controller(R"("sync_time_s": 0.0)")),
                       "vehicle_tcu_sync_time_zero");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("sync_time_s must be > 0"));
    }
    {
        TempJsonFile f(minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json, controller(R"("max_sync_torque_nm": -5.0)")),
                       "vehicle_tcu_sync_torque_negative");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("max_sync_torque_nm must be > 0"));
    }
    {
        TempJsonFile f(minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json,
                                               controller(R"("sync_time_s": 0.25, "max_sync_torque_nm": 40.0)")),
                       "vehicle_tcu_sync_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.controllers.size() == 1);
        const auto& tcu = std::get<drivetrain::ManualTcuDesc>(d.controllers[0].params);
        CHECK(tcu.auto_clutch.sync_time_s == 0.25);
        CHECK(tcu.auto_clutch.max_sync_torque_nm == 40.0);
    }
}

TEST_CASE("load_vehicle_json validates the auto_blip upshift-cut hold and torque-ramp fields", "[unit][vehicle][io][drivetrain]") {
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_tcu_cut_params");
    const auto controller = [](const char* auto_blip_fields) {
        return std::string(R"({ "id": "tcu", "type": "manual_tcu", "gearbox": "gearbox", "clutch": "clutch", "engine": "engine",
      "auto_blip": { )") + auto_blip_fields + " } }";
    };
    const struct {
        const char* fields;
        const char* file;
        const char* message;
    } bad[] = {
        {R"("upshift_cut_release_slip_rpm": 0.0)", "vehicle_tcu_cut_slip_zero", "upshift_cut_release_slip_rpm must be > 0"},
        {R"("upshift_cut_max_hold_s": -1.0)", "vehicle_tcu_cut_hold_negative", "upshift_cut_max_hold_s must be > 0"},
        {R"("upshift_torque_ramp_s": -0.1)", "vehicle_tcu_cut_ramp_negative", "upshift_torque_ramp_s must be >= 0"},
    };
    for (const auto& b : bad) {
        INFO(b.fields);
        TempJsonFile f(minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json, controller(b.fields)), b.file);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring(b.message));
    }
    {
        TempJsonFile f(minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json,
                                               controller(R"("upshift_cut_release_slip_rpm": 250, "upshift_cut_max_hold_s": 0.8, "upshift_torque_ramp_s": 0.0)")),
                       "vehicle_tcu_cut_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.controllers.size() == 1);
        const auto& tcu = std::get<drivetrain::ManualTcuDesc>(d.controllers[0].params);
        CHECK(tcu.auto_blip.upshift_cut_release_slip_rpm == 250.0);
        CHECK(tcu.auto_blip.upshift_cut_max_hold_s == 0.8);
        CHECK(tcu.auto_blip.upshift_torque_ramp_s == 0.0); // 0 = step, allowed
    }
}

TEST_CASE("load_vehicle_json rejects a gearbox initial_gear out of range", "[unit][vehicle][io][drivetrain]") {
    // Only 3 forward gears exist; initial_gear 5 is out of range.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0], "initial_gear": 5)",
                  "gearbox_initial_gear_oob_ref");
    const std::string json = minimal_i3_vehicle_json(minimal_i3_engine_json(), gb.component_json);
    TempJsonFile f(json, "vehicle_gearbox_initial_gear_oob");
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), Catch::Matchers::ContainsSubstring("initial_gear"));
}

TEST_CASE("load_vehicle_json round-trips both shipped I3 vehicles", "[unit][vehicle][io][drivetrain]") {
    SECTION("car_sedan.json") {
        const vehicle::VehicleDesc v = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
        CHECK(v.name == "car_sedan");
        CHECK(v.wheels.size() == 4);
        CHECK(v.aero.drag_area_m2 == Catch::Approx(0.68));
        CHECK(v.controllers.size() == 1);
        REQUIRE(v.powertrain.components.size() == 4);
        const auto* engine = std::get_if<drivetrain::TorqueMapEngineDesc>(&v.powertrain.components[0].params);
        REQUIRE(engine != nullptr);
        CHECK(engine->inertia_kgm2 == Catch::Approx(0.16));
        const auto* diff = std::get_if<drivetrain::DifferentialDesc>(&v.powertrain.components[3].params);
        REQUIRE(diff != nullptr);
        CHECK(diff->ratio == Catch::Approx(4.06));
    }
    SECTION("tractor.json") {
        const vehicle::VehicleDesc v = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
        CHECK(v.name == "tractor");
        CHECK(v.wheels.size() == 4);
        CHECK(v.aero.drag_area_m2 == Catch::Approx(4.0));
        CHECK(v.controllers.size() == 1);
        // engine, clutch, gearbox, transfer (MFWD transfer_case, PHYS-033 P4
        // I4 C9), diff (rear), front_diff - up from 4 once the transfer case
        // and front axle differential were added.
        REQUIRE(v.powertrain.components.size() == 6);
        const auto* gearbox = std::get_if<drivetrain::GearboxDesc>(&v.powertrain.components[2].params);
        REQUIRE(gearbox != nullptr);
        CHECK(gearbox->forward_ratios.size() == 12);
        CHECK(gearbox->reverse_ratios.size() == 4);
        const auto* transfer = std::get_if<drivetrain::TransferCaseDesc>(&v.powertrain.components[3].params);
        REQUIRE(transfer != nullptr);
        CHECK(transfer->rear_ratio == Catch::Approx(1.0));
        CHECK(transfer->front_ratio == Catch::Approx(1.0));
        CHECK(transfer->channel == "awd");
        CHECK(transfer->clutch.capacity_nm == Catch::Approx(2500.0));
        const auto* rear_diff = std::get_if<drivetrain::DifferentialDesc>(&v.powertrain.components[4].params);
        REQUIRE(rear_diff != nullptr);
        CHECK(rear_diff->ratio == Catch::Approx(20.0));
        const auto* front_diff = std::get_if<drivetrain::DifferentialDesc>(&v.powertrain.components[5].params);
        REQUIRE(front_diff != nullptr);
        CHECK(front_diff->ratio == Catch::Approx(15.30));
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
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL", "tyre": { "name": "inline_tyre", "source": "inline fixture" } } ]
})";
    TempJsonFile f(json, "vehicle_inline_tyre");
    REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
}
