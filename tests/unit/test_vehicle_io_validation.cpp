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

// PHYS-033 P4 I4 C10 (plan section 3.4, owner ruling): the park_lock
// component - a single friction row from its own `shaft` port to GROUND (no
// rigid row, unlike transfer_case's `rear` row). Mirrors the transfer_case
// validation tests above: a minimal standalone powertrain block, one port
// wired to a wheel so "every port must be connected by exactly one shaft"
// is satisfied.
TEST_CASE("load_vehicle_json validates park_lock fields (I4 C10)", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_pl_fields = [](const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [
      { "id": "park", "type": "park_lock", "capacity_nm": 30000.0, "static_factor": 1.0,
        "max_engage_speed_rad_s": 5.0)") +
               extra + R"( }
    ],
    "shafts": [
      { "ports": ["park.shaft", "wheel:FL"] }
    ]
  }
})";
    };
    auto only_pl = [](const vehicle::VehicleDesc& d) -> const drivetrain::ParkLockDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* pl = std::get_if<drivetrain::ParkLockDesc>(&d.powertrain.components[0].params);
        REQUIRE(pl != nullptr);
        return *pl;
    };
    auto rejects = [](const std::string& text, const char* tag, const std::string& needle) {
        TempJsonFile f(text, tag);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(needle));
    };

    SECTION("a well-formed park_lock round-trips") {
        TempJsonFile f(json_with_pl_fields(""), "vehicle_pl_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::ParkLockDesc& pl = only_pl(d);
        CHECK(pl.capacity_nm == 30000.0);
        CHECK(pl.static_factor == 1.0);
        CHECK(pl.max_engage_speed_rad_s == 5.0);
        CHECK(pl.channel == "park_lock");
    }
    SECTION("channel defaults to \"park_lock\" and may be overridden") {
        TempJsonFile f(json_with_pl_fields(R"(, "channel": "my_park")"), "vehicle_pl_channel");
        CHECK(only_pl(io::load_vehicle_json(f.path.string())).channel == "my_park");
    }
    SECTION("capacity_nm <= 0 is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [
      { "id": "park", "type": "park_lock", "capacity_nm": 0.0 }
    ],
    "shafts": [ { "ports": ["park.shaft", "wheel:FL"] } ]
  }
})";
        rejects(bad, "vehicle_pl_cap_zero", "park_lock 'capacity_nm' must be > 0");
    }
    SECTION("static_factor < 1 is rejected") {
        rejects(json_with_pl_fields(R"(, "static_factor": 0.9)"), "vehicle_pl_static_lt1",
                "park_lock 'static_factor' must be >= 1");
    }
    SECTION("max_engage_speed_rad_s < 0 is rejected") {
        rejects(json_with_pl_fields(R"(, "max_engage_speed_rad_s": -0.1)"), "vehicle_pl_engage_neg",
                "park_lock 'max_engage_speed_rad_s' must be >= 0");
    }
    SECTION("empty channel is rejected") {
        rejects(json_with_pl_fields(R"(, "channel": "")"), "vehicle_pl_channel_empty", "park_lock 'channel' must be non-empty");
    }
    SECTION("a second park_lock component is rejected at load time") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "park",  "type": "park_lock", "capacity_nm": 30000.0 },
      { "id": "park2", "type": "park_lock", "capacity_nm": 30000.0 }
    ],
    "shafts": [ { "ports": ["park.shaft", "wheel:FL"] }, { "ports": ["park2.shaft", "wheel:FR"] } ]
  }
})";
        rejects(bad, "vehicle_pl_duplicate", "more than one park_lock component");
    }
}

// PHYS-033 P4 I4 C11 (plan section 3.5): the pto (power take-off) component -
// a friction row from the engine/crank-side "in" port to the PTO-side "out"
// port, with a per-mode (540/1000 rpm) ratio. Mirrors the park_lock/
// transfer_case validation tests above: a minimal standalone powertrain
// block, "in" wired to a wheel so "every port must be connected by exactly
// one shaft" is satisfied - "out" is deliberately left UNCONNECTED, since
// core/src/drivetrain/powertrain.cpp exempts pto's own "out" port from that
// check (pto_out_port_key) the same way it exempts nothing else, precisely so
// a round-trip test like this one does not also have to build a load_dyno.
TEST_CASE("load_vehicle_json validates pto fields (I4 C11)", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_pto_fields = [](const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [
      { "id": "pto", "type": "pto", "ratios": { "540": 3.61 }, "capacity_nm": 500.0)") +
               extra + R"( }
    ],
    "shafts": [
      { "ports": ["pto.in", "wheel:FL"] }
    ]
  }
})";
    };
    auto only_pto = [](const vehicle::VehicleDesc& d) -> const drivetrain::PtoDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* pto = std::get_if<drivetrain::PtoDesc>(&d.powertrain.components[0].params);
        REQUIRE(pto != nullptr);
        return *pto;
    };
    auto rejects = [](const std::string& text, const char* tag, const std::string& needle) {
        TempJsonFile f(text, tag);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(needle));
    };

    SECTION("a well-formed pto (540 only) round-trips with its defaults") {
        TempJsonFile f(json_with_pto_fields(""), "vehicle_pto_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::PtoDesc& pto = only_pto(d);
        CHECK(pto.has_540);
        CHECK(pto.ratio_540 == 3.61);
        CHECK_FALSE(pto.has_1000);
        CHECK(pto.capacity_nm == 500.0);
        CHECK(pto.static_factor == 1.2);
        CHECK(pto.engage_time_s == 1.5);
        CHECK(pto.efficiency == 1.0);
        CHECK(pto.out_inertia_kgm2 == 0.0);
        CHECK(pto.disengaged_brake_time_s == 1.0);
        CHECK(pto.channel == "pto");
        CHECK(pto.speed_channel == "pto_speed");
    }
    SECTION("both 540 and 1000 ratios may be given, and other fields override the defaults") {
        std::string ok = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [
      { "id": "pto", "type": "pto", "ratios": { "540": 3.61, "1000": 1.95 }, "capacity_nm": 500.0,
        "static_factor": 1.1, "engage_time_s": 2.0, "efficiency": 0.95, "out_inertia_kgm2": 0.05,
        "disengaged_brake_time_s": 0.5, "channel": "my_pto", "speed_channel": "my_pto_speed" }
    ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] } ]
  }
})";
        TempJsonFile f(ok, "vehicle_pto_both_ratios");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::PtoDesc& pto = only_pto(d);
        CHECK(pto.has_540);
        CHECK(pto.ratio_540 == 3.61);
        CHECK(pto.has_1000);
        CHECK(pto.ratio_1000 == 1.95);
        CHECK(pto.static_factor == 1.1);
        CHECK(pto.engage_time_s == 2.0);
        CHECK(pto.efficiency == 0.95);
        CHECK(pto.out_inertia_kgm2 == 0.05);
        CHECK(pto.disengaged_brake_time_s == 0.5);
        CHECK(pto.channel == "my_pto");
        CHECK(pto.speed_channel == "my_pto_speed");
    }
    SECTION("a missing 'ratios' object is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [ { "id": "pto", "type": "pto", "capacity_nm": 500.0 } ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] } ]
  }
})";
        rejects(bad, "vehicle_pto_no_ratios", "pto requires a non-empty 'ratios' object");
    }
    SECTION("an empty 'ratios' object is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [ { "id": "pto", "type": "pto", "ratios": {}, "capacity_nm": 500.0 } ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] } ]
  }
})";
        rejects(bad, "vehicle_pto_empty_ratios", "pto requires a non-empty 'ratios' object");
    }
    SECTION("an unknown ratios key is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [ { "id": "pto", "type": "pto", "ratios": { "750": 2.0 }, "capacity_nm": 500.0 } ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] } ]
  }
})";
        rejects(bad, "vehicle_pto_bad_ratio_key", "pto ratios key '750' is not \"540\" or \"1000\"");
    }
    SECTION("a non-positive ratio is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [ { "id": "pto", "type": "pto", "ratios": { "540": 0.0 }, "capacity_nm": 500.0 } ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] } ]
  }
})";
        rejects(bad, "vehicle_pto_ratio_zero", "pto ratios['540'] must be > 0");
    }
    SECTION("capacity_nm <= 0 is rejected") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [ { "id": "pto", "type": "pto", "ratios": { "540": 3.61 }, "capacity_nm": 0.0 } ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] } ]
  }
})";
        rejects(bad, "vehicle_pto_cap_zero", "pto 'capacity_nm' must be > 0");
    }
    SECTION("static_factor < 1 is rejected") {
        rejects(json_with_pto_fields(R"(, "static_factor": 0.9)"), "vehicle_pto_static_lt1", "pto 'static_factor' must be >= 1");
    }
    SECTION("engage_time_s < 0 is rejected") {
        rejects(json_with_pto_fields(R"(, "engage_time_s": -0.1)"), "vehicle_pto_engage_neg", "pto 'engage_time_s' must be >= 0");
    }
    SECTION("efficiency outside (0, 1] is rejected") {
        rejects(json_with_pto_fields(R"(, "efficiency": 0.0)"), "vehicle_pto_eff_zero", "pto 'efficiency' must be in (0, 1]");
        rejects(json_with_pto_fields(R"(, "efficiency": 1.1)"), "vehicle_pto_eff_gt1", "pto 'efficiency' must be in (0, 1]");
    }
    SECTION("out_inertia_kgm2 < 0 is rejected") {
        rejects(json_with_pto_fields(R"(, "out_inertia_kgm2": -0.01)"), "vehicle_pto_inertia_neg",
                "pto 'out_inertia_kgm2' must be >= 0");
    }
    SECTION("disengaged_brake_time_s <= 0 is rejected") {
        rejects(json_with_pto_fields(R"(, "disengaged_brake_time_s": 0.0)"), "vehicle_pto_brake_time_zero",
                "pto 'disengaged_brake_time_s' must be > 0");
    }
    SECTION("empty channel is rejected") {
        rejects(json_with_pto_fields(R"(, "channel": "")"), "vehicle_pto_channel_empty", "pto 'channel' must be non-empty");
    }
    SECTION("empty speed_channel is rejected") {
        rejects(json_with_pto_fields(R"(, "speed_channel": "")"), "vehicle_pto_speed_channel_empty",
                "pto 'speed_channel' must be non-empty");
    }
    SECTION("a second pto component is rejected at load time") {
        std::string bad = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "pto",  "type": "pto", "ratios": { "540": 3.61 }, "capacity_nm": 500.0 },
      { "id": "pto2", "type": "pto", "ratios": { "540": 3.61 }, "capacity_nm": 500.0 }
    ],
    "shafts": [ { "ports": ["pto.in", "wheel:FL"] }, { "ports": ["pto2.in", "wheel:FR"] } ]
  }
})";
        rejects(bad, "vehicle_pto_duplicate", "more than one pto component");
    }
}

// PHYS-033 P4 I4 C11: the load_dyno component - a test/bench-only load
// (brake/viscous/quadratic) on port "shaft". No shipped vehicle carries one
// (see CLAUDE.md's "Drivetrain" section), so every fixture here builds its
// own minimal standalone powertrain block, same pattern as pto above -
// "shaft" wired to a wheel (load_dyno's own port gets NO connectivity
// exemption, unlike pto's "out": core/src/drivetrain/powertrain.cpp's own
// comment on the point). Any number of load_dyno components is allowed (no
// "at most one" rule, unlike pto/park_lock).
TEST_CASE("load_vehicle_json validates load_dyno fields (I4 C11)", "[unit][vehicle][io][drivetrain]") {
    using Catch::Matchers::ContainsSubstring;
    auto json_with_dyno_fields = [](const std::string& extra) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" } ],
  "powertrain": {
    "components": [
      { "id": "dyno", "type": "load_dyno")") +
               extra + R"( }
    ],
    "shafts": [
      { "ports": ["dyno.shaft", "wheel:FL"] }
    ]
  }
})";
    };
    auto only_dyno = [](const vehicle::VehicleDesc& d) -> const drivetrain::LoadDynoDesc& {
        REQUIRE(d.powertrain.components.size() == 1);
        const auto* dyno = std::get_if<drivetrain::LoadDynoDesc>(&d.powertrain.components[0].params);
        REQUIRE(dyno != nullptr);
        return *dyno;
    };
    auto rejects = [](const std::string& text, const char* tag, const std::string& needle) {
        TempJsonFile f(text, tag);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(needle));
    };

    SECTION("mode defaults to \"brake\", and torque_nm/inertia_kgm2 round-trip") {
        TempJsonFile f(json_with_dyno_fields(R"(, "torque_nm": 120.0, "inertia_kgm2": 0.03)"), "vehicle_dyno_brake_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::LoadDynoDesc& dyno = only_dyno(d);
        CHECK(dyno.mode == drivetrain::LoadDynoMode::Brake);
        CHECK(dyno.torque_nm == 120.0);
        CHECK(dyno.inertia_kgm2 == 0.03);
        CHECK(dyno.scale_channel.empty()); // default: no channel -> fixed scale 1.0
    }
    SECTION("viscous mode round-trips") {
        TempJsonFile f(json_with_dyno_fields(R"(, "mode": "viscous", "viscous_nm_s_rad": 5.0)"), "vehicle_dyno_viscous_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::LoadDynoDesc& dyno = only_dyno(d);
        CHECK(dyno.mode == drivetrain::LoadDynoMode::Viscous);
        CHECK(dyno.viscous_nm_s_rad == 5.0);
    }
    SECTION("quadratic mode round-trips, and scale_channel may be set") {
        TempJsonFile f(json_with_dyno_fields(R"(, "mode": "quadratic", "quadratic_nm_s2_rad2": 0.2, "scale_channel": "load")"),
                       "vehicle_dyno_quadratic_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const drivetrain::LoadDynoDesc& dyno = only_dyno(d);
        CHECK(dyno.mode == drivetrain::LoadDynoMode::Quadratic);
        CHECK(dyno.quadratic_nm_s2_rad2 == 0.2);
        CHECK(dyno.scale_channel == "load");
    }
    SECTION("an invalid mode string is rejected") {
        rejects(json_with_dyno_fields(R"(, "mode": "motor")"), "vehicle_dyno_bad_mode",
                "load_dyno 'mode' must be \"brake\", \"viscous\" or \"quadratic\"");
    }
    SECTION("brake mode requires torque_nm > 0") {
        rejects(json_with_dyno_fields(""), "vehicle_dyno_brake_no_torque",
                "load_dyno 'torque_nm' must be > 0 for mode \"brake\"");
        rejects(json_with_dyno_fields(R"(, "torque_nm": 0.0)"), "vehicle_dyno_brake_torque_zero",
                "load_dyno 'torque_nm' must be > 0 for mode \"brake\"");
    }
    SECTION("torque_nm is only valid for mode \"brake\"") {
        rejects(json_with_dyno_fields(R"(, "mode": "viscous", "viscous_nm_s_rad": 5.0, "torque_nm": 10.0)"),
                "vehicle_dyno_torque_wrong_mode", "load_dyno 'torque_nm' is only valid for mode \"brake\"");
    }
    SECTION("viscous mode requires viscous_nm_s_rad > 0") {
        rejects(json_with_dyno_fields(R"(, "mode": "viscous")"), "vehicle_dyno_viscous_missing",
                "load_dyno 'viscous_nm_s_rad' must be > 0 for mode \"viscous\"");
        rejects(json_with_dyno_fields(R"(, "mode": "viscous", "viscous_nm_s_rad": 0.0)"), "vehicle_dyno_viscous_zero",
                "load_dyno 'viscous_nm_s_rad' must be > 0 for mode \"viscous\"");
    }
    SECTION("viscous_nm_s_rad is only valid for mode \"viscous\"") {
        rejects(json_with_dyno_fields(R"(, "torque_nm": 10.0, "viscous_nm_s_rad": 5.0)"), "vehicle_dyno_viscous_wrong_mode",
                "load_dyno 'viscous_nm_s_rad' is only valid for mode \"viscous\"");
    }
    SECTION("quadratic mode requires quadratic_nm_s2_rad2 > 0") {
        rejects(json_with_dyno_fields(R"(, "mode": "quadratic")"), "vehicle_dyno_quadratic_missing",
                "load_dyno 'quadratic_nm_s2_rad2' must be > 0 for mode \"quadratic\"");
        rejects(json_with_dyno_fields(R"(, "mode": "quadratic", "quadratic_nm_s2_rad2": 0.0)"), "vehicle_dyno_quadratic_zero",
                "load_dyno 'quadratic_nm_s2_rad2' must be > 0 for mode \"quadratic\"");
    }
    SECTION("quadratic_nm_s2_rad2 is only valid for mode \"quadratic\"") {
        rejects(json_with_dyno_fields(R"(, "torque_nm": 10.0, "quadratic_nm_s2_rad2": 0.2)"), "vehicle_dyno_quadratic_wrong_mode",
                "load_dyno 'quadratic_nm_s2_rad2' is only valid for mode \"quadratic\"");
    }
    SECTION("inertia_kgm2 < 0 is rejected") {
        rejects(json_with_dyno_fields(R"(, "torque_nm": 10.0, "inertia_kgm2": -0.01)"), "vehicle_dyno_inertia_neg",
                "load_dyno 'inertia_kgm2' must be >= 0");
    }
    SECTION("more than one load_dyno component is allowed") {
        std::string ok = R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "dyno1", "type": "load_dyno", "torque_nm": 10.0 },
      { "id": "dyno2", "type": "load_dyno", "mode": "viscous", "viscous_nm_s_rad": 5.0 }
    ],
    "shafts": [ { "ports": ["dyno1.shaft", "wheel:FL"] }, { "ports": ["dyno2.shaft", "wheel:FR"] } ]
  }
})";
        TempJsonFile f(ok, "vehicle_dyno_multiple_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.powertrain.components.size() == 2);
    }
}

// PHYS-033 P4 I4 C13: compliant drivetrain shafts. powertrain.compliance
// ("rigid" default / "compliant") plus per-shaft stiffness/damping. Same
// minimal standalone powertrain block as the transfer_case cases above (the
// loader validates the shaft block syntactically; no graph is built here).
TEST_CASE("load_vehicle_json validates compliant shaft fields (I4 C13)", "[unit][vehicle][io][drivetrain][compliance]") {
    using Catch::Matchers::ContainsSubstring;
    // `compliance` is the whole "compliance": ... member (or empty), `rear_shaft`
    // the whole rear shaft object's extra fields, `rear_ports` its port list.
    auto json_with = [](const std::string& compliance, const std::string& rear_extra,
                        const std::string& rear_ports = R"("transfer.rear", "wheel:FL")") {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" }, { "name": "RL" } ],
  "powertrain": {)") +
               compliance + R"(
    "components": [
      { "id": "transfer", "type": "transfer_case", "rear_ratio": 1.0, "front_ratio": 1.0,
        "clutch": { "capacity_nm": 2500.0 } }
    ],
    "shafts": [
      { "ports": [)" + rear_ports + "]" + rear_extra + R"( },
      { "ports": ["transfer.front", "wheel:FR"] }
    ]
  }
})";
    };
    auto rejects = [](const std::string& text, const char* tag, const std::string& needle) {
        TempJsonFile f(text, tag);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(needle));
    };
    const std::string compliant = R"( "compliance": "compliant",)";
    const std::string rigid = R"( "compliance": "rigid",)";

    SECTION("default is rigid, stiffness/damping default to 0") {
        TempJsonFile f(json_with("", ""), "vehicle_c13_default");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK(d.powertrain.compliance == drivetrain::Compliance::Rigid);
        REQUIRE(d.powertrain.shafts.size() == 2);
        CHECK(d.powertrain.shafts[0].stiffness == 0.0);
        CHECK(d.powertrain.shafts[0].damping == 0.0);
    }
    SECTION("a well-formed compliant shaft round-trips") {
        TempJsonFile f(json_with(compliant, R"(, "stiffness": 600.0, "damping": 2.0)"), "vehicle_c13_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK(d.powertrain.compliance == drivetrain::Compliance::Compliant);
        CHECK(d.powertrain.shafts[0].stiffness == 600.0);
        CHECK(d.powertrain.shafts[0].damping == 2.0);
        CHECK(d.powertrain.shafts[1].stiffness == 0.0);
    }
    SECTION("rigid parses and keeps stiffness/damping (ignored at graph build)") {
        TempJsonFile f(json_with(rigid, R"(, "stiffness": 600.0, "damping": 2.0)"), "vehicle_c13_rigid");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        CHECK(d.powertrain.compliance == drivetrain::Compliance::Rigid);
        CHECK(d.powertrain.shafts[0].stiffness == 600.0);
    }
    SECTION("rigid with a 3-port stiff shaft is fine (the 2-port rule is compliant-only)") {
        TempJsonFile f(json_with(rigid, R"(, "stiffness": 600.0)", R"("transfer.rear", "wheel:FL", "wheel:RL")"),
                       "vehicle_c13_rigid_3port");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("stiffness < 0 is rejected") {
        rejects(json_with(compliant, R"(, "stiffness": -1.0)"), "vehicle_c13_k_neg", "'stiffness' must be >= 0");
    }
    SECTION("damping < 0 is rejected") {
        rejects(json_with(compliant, R"(, "stiffness": 600.0, "damping": -0.5)"), "vehicle_c13_c_neg",
                "'damping' must be >= 0");
    }
    SECTION("damping > 0 with stiffness 0 is rejected (no pure damper shaft)") {
        rejects(json_with(compliant, R"(, "damping": 2.0)"), "vehicle_c13_pure_damper",
                "'damping' > 0 requires 'stiffness' > 0");
    }
    SECTION("\"compliant\" with no stiff shaft is rejected") {
        rejects(json_with(compliant, ""), "vehicle_c13_no_stiff", "no shaft has stiffness > 0");
    }
    SECTION("a compliant stiff shaft with 3 ports is rejected") {
        rejects(json_with(compliant, R"(, "stiffness": 600.0)", R"("transfer.rear", "wheel:FL", "wheel:RL")"),
                "vehicle_c13_3port", "must list exactly 2 ports, got 3");
    }
    SECTION("a compliant stiff shaft with 1 port is rejected") {
        rejects(json_with(compliant, R"(, "stiffness": 600.0)", R"("transfer.rear")"), "vehicle_c13_1port",
                "must list exactly 2 ports, got 1");
    }
    SECTION("an unknown compliance string is rejected") {
        rejects(json_with(R"( "compliance": "elastic",)", R"(, "stiffness": 600.0)"), "vehicle_c13_unknown",
                "powertrain.compliance must be \"rigid\" or \"compliant\", got \"elastic\"");
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

// PHYS-033 P4 I4 C12: minimal_i3_engine_json() plus a "governor" block (and,
// for the limiter-overlap case, an overridden "limiter" block too) - the
// default idle_rpm (800) and default limiter (fuel_cut at 6800) from
// engine_torque_map.h's Desc defaults are otherwise untouched, so each
// SECTION below breaks exactly one governor rule at a time.
std::string engine_with_governor(const std::string& governor_fields, const std::string& extra_engine_fields = std::string()) {
    return std::string(R"({ "id": "engine", "type": "engine", "kind": "torque_map",
      "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
      "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
      "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0,
      "governor": { )") + governor_fields + " }" + extra_engine_fields + " }";
}
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

// PHYS-033 P4 I4 C12: the six self-contained "governor" block rejections
// (core/src/io/vehicle_io.cpp lines ~395-421) plus the cross-field overlap
// check against a soft_cut limiter (~453-463) - see docs/data_formats/
// engine.md's "governor" section for the documented model these mirror.
TEST_CASE("load_vehicle_json rejects a malformed governor block", "[unit][vehicle][io][drivetrain][governor]") {
    SECTION("type is not \"all_speed\"") {
        const std::string engine = engine_with_governor(
            R"("type": "constant_speed", "high_idle_rpm": 2400, "rated_rpm": 2200, "droop": 0.09, "channel": "hand_throttle")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_bad_type");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.type must be \"all_speed\""));
    }
    SECTION("high_idle_rpm <= idle_rpm") {
        // idle_rpm defaults to 800 (unset above); 700 <= 800.
        const std::string engine = engine_with_governor(R"("high_idle_rpm": 700, "rated_rpm": 600, "droop": 0.09, "channel": "hand_throttle")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_high_idle_le_idle");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.high_idle_rpm must be > idle_rpm"));
    }
    SECTION("droop outside (0, 0.3]") {
        const std::string engine = engine_with_governor(R"("droop": 0.35, "channel": "hand_throttle")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_droop_too_high");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.droop must be in (0, 0.3]"));
    }
    SECTION("droop == 0") {
        const std::string engine = engine_with_governor(R"("droop": 0.0, "channel": "hand_throttle")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_droop_zero");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.droop must be in (0, 0.3]"));
    }
    SECTION("rated_rpm outside (idle_rpm, high_idle_rpm] - above high_idle_rpm") {
        const std::string engine = engine_with_governor(R"("high_idle_rpm": 2400, "rated_rpm": 2500, "droop": 0.09, "channel": "hand_throttle")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_rated_above_high_idle");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.rated_rpm must be in (idle_rpm, high_idle_rpm]"));
    }
    SECTION("rated_rpm outside (idle_rpm, high_idle_rpm] - at or below idle_rpm") {
        const std::string engine = engine_with_governor(R"("high_idle_rpm": 2400, "rated_rpm": 800, "droop": 0.09, "channel": "hand_throttle")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_rated_le_idle");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.rated_rpm must be in (idle_rpm, high_idle_rpm]"));
    }
    SECTION("channel is empty") {
        const std::string engine = engine_with_governor(R"("channel": "")");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_empty_channel");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor.channel must not be empty"));
    }
    SECTION("combined with a soft_cut limiter whose band overlaps the droop band") {
        // Default governor: high_idle_rpm 2400, droop 0.0909, rated_rpm 2200
        // -> droop band [2400 - 0.0909*2200, 2400] = [2200.02, 2400]. A
        // soft_cut limiter at rpm=2350/band_rpm=100 -> [2250, 2350], which
        // overlaps.
        const std::string engine = engine_with_governor(
            R"("channel": "hand_throttle")",
            R"(, "limiter": { "type": "soft_cut", "rpm": 2350, "band_rpm": 100 })");
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_governor_overlaps_soft_cut_limiter");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("governor combined with a soft_cut limiter"));
    }
}

// R6 (vault PHYS-049/050 follow-ups): a torque_map engine's starter/idle_control/
// limiter blocks previously accepted a non-positive free_speed_rpm (divide by
// zero at runtime in EngineTorqueMap/SimulatedEngine's starter torque line), a
// negative idle PI gain (flips the loop's sign instead of merely weakening it),
// or a negative limiter hysteresis (reverses the FuelCut latch's restore
// direction) without complaint.
TEST_CASE("load_vehicle_json rejects a torque_map engine's starter/idle/limiter sign errors",
          "[unit][vehicle][io][drivetrain]") {
    SECTION("starter.free_speed_rpm == 0") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
          "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
          "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0,
          "starter": { "stall_torque_nm": 150.0, "free_speed_rpm": 0.0 } })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_starter_free_speed_zero");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("starter.free_speed_rpm must be > 0"));
    }
    SECTION("starter.free_speed_rpm < 0") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
          "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
          "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0,
          "starter": { "stall_torque_nm": 150.0, "free_speed_rpm": -400.0 } })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_starter_free_speed_negative");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("starter.free_speed_rpm must be > 0"));
    }
    SECTION("negative idle_control.kp_per_rpm") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
          "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
          "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0,
          "idle_control": { "kp_per_rpm": -0.001 } })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_idle_kp_negative");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("idle_control.kp_per_rpm must be >= 0"));
    }
    SECTION("negative idle_control.ki_per_rpm_s") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
          "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
          "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0,
          "idle_control": { "ki_per_rpm_s": -0.004 } })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_idle_ki_negative");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("idle_control.ki_per_rpm_s must be >= 0"));
    }
    SECTION("negative limiter.hysteresis_rpm") {
        const std::string engine = R"({ "id": "engine", "type": "engine", "kind": "torque_map",
          "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
          "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
          "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0,
          "limiter": { "type": "fuel_cut", "rpm": 6800, "hysteresis_rpm": -150.0 } })";
        const std::string json = minimal_i3_vehicle_json(engine, minimal_i3_gearbox_json());
        TempJsonFile f(json, "vehicle_limiter_hysteresis_negative");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("limiter.hysteresis_rpm must be >= 0"));
    }
}

TEST_CASE("load_vehicle_json accepts a well-formed governor block and round-trips its fields", "[unit][vehicle][io][drivetrain][governor]") {
    // NOTE: an INLINE gearbox component (minimal_i3_gearbox_json()) cannot be
    // used for a case that must actually SUCCEED all the way through - its
    // dispatch-level "type": "gearbox" is the same JSON object
    // read_gearbox_fields() then re-reads as the manual/sequential
    // discriminator, so an inline gearbox component always throws "gearbox
    // 'type' must be \"manual\" or \"sequential\", got \"gearbox\"". Every
    // OTHER use of minimal_i3_gearbox_json() in this file is inside a case
    // that throws earlier (while validating a deliberately-broken engine),
    // so this pre-existing quirk never surfaces there. Use a ref'd
    // RefGearbox (its own file's "type" field is independent) instead.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_governor_well_formed_ref");
    const std::string engine = engine_with_governor(
        R"("type": "all_speed", "high_idle_rpm": 2400, "rated_rpm": 2200, "droop": 0.0909, "channel": "hand_throttle")");
    const std::string json = minimal_i3_vehicle_json(engine, gb.component_json);
    TempJsonFile f(json, "vehicle_governor_well_formed");
    const vehicle::VehicleDesc desc = io::load_vehicle_json(f.path.string());
    const auto* engine_desc = std::get_if<drivetrain::TorqueMapEngineDesc>(&desc.powertrain.components.at(0).params);
    REQUIRE(engine_desc != nullptr);
    REQUIRE(engine_desc->has_governor);
    CHECK(engine_desc->governor.high_idle_rpm == Catch::Approx(2400.0));
    CHECK(engine_desc->governor.rated_rpm == Catch::Approx(2200.0));
    CHECK(engine_desc->governor.droop == Catch::Approx(0.0909));
    CHECK(engine_desc->governor.channel == "hand_throttle");
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

TEST_CASE("load_vehicle_json rejects nonzero auto_blip blip/ramp values on a manual_tcu whose engine has a "
          "mechanical throttle",
          "[unit][vehicle][io][drivetrain][throttle]") {
    // PHYS-033 P4 I7a C3 (plan section 3.4.5, "which TCU intervention lives
    // where"): ManualTcu's blip (kp_per_rpm) and the post-upshift pedal-
    // ceiling ramp only ever reach the engine via ctx.tcu_throttle_floor/
    // tcu_throttle_ceiling, which ThrottleStage::evaluate ignores outright
    // for a Mechanical stage (floor/ceiling are fixed at 0/1 there - see
    // core/src/drivetrain/throttle_stage.h). So a mechanical-throttle
    // engine cannot carry a nonzero blip gain or post-shift ceiling ramp.
    //
    // C3 REVIEW (2026-09-26): the original version of this check rejected
    // on 'auto_blip.enabled' rather than on the three fields it actually
    // needs zeroed - wrongly rejecting "enabled: true" with
    // kp_per_rpm/max_throttle/upshift_torque_ramp_s all explicitly zeroed,
    // a no-op configuration that must load (it does nothing whether or not
    // 'enabled' is true, exactly like an absent auto_blip block on an
    // electronic engine with those same fields zeroed). The check is now on
    // the EFFECTIVE VALUES, post-default (AutoBlipDesc's defaults -
    // kp_per_rpm 0.002, max_throttle 1.0, upshift_torque_ramp_s 0.10 - apply
    // even when the JSON has no auto_blip block at all, so simply omitting
    // those three fields still leaves them nonzero and still rejects). The
    // upshift ignition cut (auto_blip.upshift_cut) is UNAFFECTED -
    // TcuRequest::ignition_cut reaches the engine directly, never through
    // ThrottleCommand - see test_throttle_control.cpp's P4 for that
    // behaviour exercised white-box; this test only covers the loader-level
    // rejection.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_tcu_blip_mechanical");
    // throttle_input must agree with the explicit stage's kind (I7a C3
    // review item 2 - a mismatch is now rejected outright), so this
    // fixture's engine declares itself mechanical too.
    const std::string mechanical_engine_json =
        R"({ "id": "engine", "type": "engine", "kind": "torque_map", "throttle_input": "mechanical",
      "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
      "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
      "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 })";
    const std::string throttle_control =
        R"({ "id": "throttle", "type": "throttle_control", "engine": "engine", "kind": "mechanical_linkage" })";
    const auto vehicle_json = [&](const char* auto_blip_fields) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      )") + mechanical_engine_json + R"(,
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )" + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 },
      )" + throttle_control + R"(
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  },
  "controllers": [ { "id": "tcu", "type": "manual_tcu", "gearbox": "gearbox", "clutch": "clutch", "engine": "engine",
      )" + (auto_blip_fields != nullptr ? std::string(R"("auto_blip": { )") + auto_blip_fields + " }" : std::string(R"("auto_clutch": { "enabled": false })")) + R"( } ]
})";
    };
    SECTION("enabled: true with default (nonzero) blip/ramp values is rejected") {
        TempJsonFile f(vehicle_json(R"("enabled": true, "upshift_cut_release_slip_rpm": 250, "upshift_cut_max_hold_s": 0.8)"),
                       "vehicle_tcu_blip_mechanical_rejected");
        REQUIRE_THROWS_WITH(
            io::load_vehicle_json(f.path.string()),
            Catch::Matchers::ContainsSubstring("has a mechanical throttle - a mechanical throttle cannot open the plate"));
    }
    SECTION("no auto_blip block at all is rejected (the defaults kp 0.002/max 1.0/ramp 0.10 apply)") {
        TempJsonFile f(vehicle_json(nullptr), "vehicle_tcu_blip_mechanical_no_block");
        REQUIRE_THROWS_WITH(
            io::load_vehicle_json(f.path.string()),
            Catch::Matchers::ContainsSubstring("has a mechanical throttle - a mechanical throttle cannot open the plate"));
    }
    SECTION("enabled: false with the default (nonzero) values is still rejected - the check is on values") {
        TempJsonFile f(vehicle_json(R"("enabled": false)"), "vehicle_tcu_blip_mechanical_disabled_defaults");
        REQUIRE_THROWS_WITH(
            io::load_vehicle_json(f.path.string()),
            Catch::Matchers::ContainsSubstring("has a mechanical throttle - a mechanical throttle cannot open the plate"));
    }
    SECTION("each of kp_per_rpm / max_throttle / upshift_torque_ramp_s nonzero alone is rejected") {
        const char* const cases[] = {
            R"("kp_per_rpm": 0.001, "max_throttle": 0.0, "upshift_torque_ramp_s": 0.0)",
            R"("kp_per_rpm": 0.0, "max_throttle": 0.5, "upshift_torque_ramp_s": 0.0)",
            R"("kp_per_rpm": 0.0, "max_throttle": 0.0, "upshift_torque_ramp_s": 0.05)",
        };
        const char* const tags[] = {"vehicle_tcu_blip_mech_kp", "vehicle_tcu_blip_mech_max", "vehicle_tcu_blip_mech_ramp"};
        for (int i = 0; i < 3; ++i) {
            const char* fields = cases[i];
            TempJsonFile f(vehicle_json(fields), tags[i]);
            REQUIRE_THROWS_WITH(
                io::load_vehicle_json(f.path.string()),
                Catch::Matchers::ContainsSubstring("has a mechanical throttle - a mechanical throttle cannot open the plate"));
        }
    }
    SECTION("enabled: true with kp_per_rpm/max_throttle/upshift_torque_ramp_s all zeroed loads "
            "(upshift ignition cut alone is fine on a mechanical engine)") {
        TempJsonFile f(vehicle_json(R"("enabled": true, "kp_per_rpm": 0.0, "max_throttle": 0.0,
              "upshift_torque_ramp_s": 0.0, "upshift_cut_release_slip_rpm": 250, "upshift_cut_max_hold_s": 0.8)"),
                       "vehicle_tcu_blip_mechanical_zeroed_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.controllers.size() == 1);
        const auto& tcu = std::get<drivetrain::ManualTcuDesc>(d.controllers[0].params);
        CHECK(tcu.auto_blip.enabled);
        CHECK(tcu.auto_blip.kp_per_rpm == 0.0);
        CHECK(tcu.auto_blip.max_throttle == 0.0);
        CHECK(tcu.auto_blip.upshift_torque_ramp_s == 0.0);
        CHECK(tcu.auto_blip.upshift_cut); // default true - still available
    }
    SECTION("enabled: false with the same zeroed values also loads") {
        TempJsonFile f(vehicle_json(R"("enabled": false, "kp_per_rpm": 0.0, "max_throttle": 0.0,
              "upshift_torque_ramp_s": 0.0, "upshift_cut_release_slip_rpm": 250, "upshift_cut_max_hold_s": 0.8)"),
                       "vehicle_tcu_blip_mechanical_disabled_ok");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.controllers.size() == 1);
        const auto& tcu = std::get<drivetrain::ManualTcuDesc>(d.controllers[0].params);
        CHECK_FALSE(tcu.auto_blip.enabled);
    }
}

TEST_CASE("load_vehicle_json rejects a throttle_control 'kind' that disagrees with its engine's throttle_input",
          "[unit][vehicle][io][drivetrain][throttle]") {
    // PHYS-033 P4 I7a C3 review item 2, both directions: an explicit
    // throttle_control component must AGREE with its named engine's
    // throttle_input, whatever it says - a mismatch is rejected outright,
    // it is never silently overridden.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_throttle_kind_mismatch");
    const auto engine_json = [](const char* throttle_input) {
        return std::string(R"({ "id": "engine", "type": "engine", "kind": "torque_map", "throttle_input": ")") +
               throttle_input + R"(",
      "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
      "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
      "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 })";
    };
    const auto vehicle_json = [&](const char* throttle_input, const char* stage_kind) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      )") + engine_json(throttle_input) + R"(,
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )" + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 },
      { "id": "throttle", "type": "throttle_control", "engine": "engine", "kind": ")" + stage_kind + R"(" }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    SECTION("electronic engine + mechanical_linkage stage is rejected") {
        TempJsonFile f(vehicle_json("electronic", "mechanical_linkage"), "vehicle_throttle_kind_mismatch_1");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("does not match engine 'engine'.throttle_input"));
    }
    SECTION("mechanical engine + electronic stage is rejected") {
        TempJsonFile f(vehicle_json("mechanical", "electronic"), "vehicle_throttle_kind_mismatch_2");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("does not match engine 'engine'.throttle_input"));
    }
    SECTION("electronic engine + electronic stage agrees and loads") {
        TempJsonFile f(vehicle_json("electronic", "electronic"), "vehicle_throttle_kind_match_electronic");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("mechanical engine + mechanical_linkage stage agrees and loads") {
        TempJsonFile f(vehicle_json("mechanical", "mechanical_linkage"), "vehicle_throttle_kind_match_mechanical");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
}

TEST_CASE("load_vehicle_json throttle_control hand_lever_channel vs. a governed engine's governor.channel",
          "[unit][vehicle][io][drivetrain][throttle]") {
    // PHYS-033 P4 I7a C3 review item 3(b): an explicit hand_lever_channel
    // set on a GOVERNED engine must either be empty (falls back to
    // governor.channel) or equal governor.channel itself - a different
    // channel makes no physical sense (one engine, one hand lever). An
    // explicit hand_lever_channel on an UNGOVERNED engine is always
    // allowed.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_hand_lever_channel");
    const auto vehicle_json = [&](const char* engine_extra, const char* hand_lever_channel) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "engine", "type": "engine", "kind": "torque_map", )") + engine_extra + R"(
        "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
        "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
        "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 },
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )" + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 },
      { "id": "throttle", "type": "throttle_control", "engine": "engine", "kind": "electronic", )" +
               (std::string(hand_lever_channel).empty() ? std::string()
                                                          : (std::string(R"("hand_lever_channel": ")") +
                                                             hand_lever_channel + R"(", )")) +
               R"("pedal_map": { "x": [0.0, 1.0], "y": [0.0, 1.0] } }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    const char* governed_engine_extra =
        R"("governor": { "high_idle_rpm": 2400, "rated_rpm": 2200, "droop": 0.0909,
             "channel": "hand_throttle" },)";
    SECTION("governed engine, hand_lever_channel matches governor.channel: loads") {
        TempJsonFile f(vehicle_json(governed_engine_extra, "hand_throttle"), "vehicle_hand_lever_matches");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("governed engine, hand_lever_channel differs from governor.channel: rejected") {
        TempJsonFile f(vehicle_json(governed_engine_extra, "some_other_channel"), "vehicle_hand_lever_differs");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("differs from engine 'engine'.governor.channel"));
    }
    SECTION("governed engine, hand_lever_channel empty: loads (falls back to governor.channel)") {
        TempJsonFile f(vehicle_json(governed_engine_extra, ""), "vehicle_hand_lever_empty_governed");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("ungoverned engine, any explicit hand_lever_channel: always allowed") {
        TempJsonFile f(vehicle_json("", "some_channel"), "vehicle_hand_lever_ungoverned");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
}

TEST_CASE("load_vehicle_json validates throttle_control pedal_map/linkage_curve endpoint and monotonicity rules",
          "[unit][vehicle][io][drivetrain][throttle]") {
    // PHYS-033 P4 I7a C3 review item 4: x.front() == 0 and x.back() == 1
    // exactly, and y non-decreasing - each with its own rejection message.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_pedal_map_rules");
    const auto curve_vehicle_json = [&](const char* engine_input, const char* stage_kind, const char* curve_key,
                                        const char* curve_json) {
        return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "engine", "type": "engine", "kind": "torque_map", )") + engine_input + R"(
        "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
        "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
        "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 },
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )" + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 },
      { "id": "throttle", "type": "throttle_control", "engine": "engine", "kind": ")" + std::string(stage_kind) + R"(",
        ")" + std::string(curve_key) + R"(": )" + curve_json + R"( }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
    };
    const auto vehicle_json = [&](const char* curve_json) {
        return curve_vehicle_json("", "electronic", "pedal_map", curve_json);
    };
    const auto linkage_vehicle_json = [&](const char* curve_json) {
        return curve_vehicle_json(R"("throttle_input": "mechanical",)", "mechanical_linkage", "linkage_curve", curve_json);
    };
    SECTION("x.front() != 0 is rejected") {
        TempJsonFile f(vehicle_json(R"({ "x": [0.1, 1.0], "y": [0.0, 1.0] })"), "vehicle_pedal_map_front_nonzero");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("x.front() must be exactly 0"));
    }
    SECTION("x.back() != 1 is rejected") {
        TempJsonFile f(vehicle_json(R"({ "x": [0.0, 0.9], "y": [0.0, 1.0] })"), "vehicle_pedal_map_back_nonone");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("x.back() must be exactly 1"));
    }
    SECTION("non-decreasing y is rejected") {
        TempJsonFile f(vehicle_json(R"({ "x": [0.0, 0.5, 1.0], "y": [0.0, 0.6, 0.3] })"),
                       "vehicle_pedal_map_y_decreasing");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("y must be non-decreasing"));
    }
    SECTION("y out of [0, 1] is rejected") {
        TempJsonFile f(vehicle_json(R"({ "x": [0.0, 1.0], "y": [0.0, 1.5] })"), "vehicle_pedal_map_y_oob");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("y must be within [0, 1]"));
    }
    SECTION("a valid flat-plateau curve covering [0,1] loads") {
        TempJsonFile f(vehicle_json(R"({ "x": [0.0, 0.4, 0.6, 1.0], "y": [0.0, 0.3, 0.3, 1.0] })"),
                       "vehicle_pedal_map_valid");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("linkage_curve: the same rules apply (x.back() != 1, decreasing y, y out of range) and a valid one loads") {
        TempJsonFile f1(linkage_vehicle_json(R"({ "x": [0.0, 0.9], "y": [0.0, 1.0] })"), "vehicle_linkage_back_nonone");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f1.path.string()),
                             Catch::Matchers::ContainsSubstring("linkage_curve") &&
                                 Catch::Matchers::ContainsSubstring("x.back() must be exactly 1"));
        TempJsonFile f2(linkage_vehicle_json(R"({ "x": [0.0, 0.5, 1.0], "y": [0.0, 0.6, 0.3] })"),
                        "vehicle_linkage_y_decreasing");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f2.path.string()),
                             Catch::Matchers::ContainsSubstring("linkage_curve") &&
                                 Catch::Matchers::ContainsSubstring("y must be non-decreasing"));
        TempJsonFile f3(linkage_vehicle_json(R"({ "x": [0.0, 1.0], "y": [-0.1, 1.0] })"), "vehicle_linkage_y_oob");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f3.path.string()),
                             Catch::Matchers::ContainsSubstring("linkage_curve") &&
                                 Catch::Matchers::ContainsSubstring("y must be within [0, 1]"));
        TempJsonFile f4(linkage_vehicle_json(R"({ "x": [0.0, 0.5, 1.0], "y": [0.0, 0.2, 1.0] })"), "vehicle_linkage_valid");
        REQUIRE_NOTHROW(io::load_vehicle_json(f4.path.string()));
    }
    SECTION("a curve under the other kind's key is rejected") {
        TempJsonFile f1(curve_vehicle_json(R"("throttle_input": "mechanical",)", "mechanical_linkage", "pedal_map",
                                           R"({ "x": [0.0, 1.0], "y": [0.0, 1.0] })"),
                        "vehicle_pedal_map_on_mechanical");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f1.path.string()),
                             Catch::Matchers::ContainsSubstring("'pedal_map' is only valid for kind"));
        TempJsonFile f2(curve_vehicle_json("", "electronic", "linkage_curve", R"({ "x": [0.0, 1.0], "y": [0.0, 1.0] })"),
                        "vehicle_linkage_on_electronic");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f2.path.string()),
                             Catch::Matchers::ContainsSubstring("'linkage_curve' is only valid for kind"));
    }
}

TEST_CASE("load_vehicle_json throttle_control engine reference cross-checks (order-independent)",
          "[unit][vehicle][io][drivetrain][throttle]") {
    // PHYS-033 P4 I7a C3 review item 5: the throttle_control 'engine' ref
    // must name an actual torque_map engine component (a stage listed
    // BEFORE its engine in the components array must still load), and a
    // stage in a vehicle with no engine component at all is rejected.
    RefGearbox gb(R"("type": "manual", "forward_ratios": [3.0, 1.6, 1.0], "forward_efficiency": [1.0, 1.0, 1.0])",
                  "gearbox_throttle_engine_ref");
    SECTION("throttle_control listed BEFORE its engine still loads") {
        const std::string json = std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "throttle", "type": "throttle_control", "engine": "engine", "kind": "electronic" },
      { "id": "engine", "type": "engine", "kind": "torque_map",
        "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
        "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
        "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 },
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )") + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
        TempJsonFile f(json, "vehicle_throttle_before_engine");
        REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
    }
    SECTION("throttle_control naming an unknown component is rejected") {
        const std::string json = std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "engine", "type": "engine", "kind": "torque_map",
        "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
        "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
        "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 },
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )") + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 },
      { "id": "throttle", "type": "throttle_control", "engine": "nonexistent", "kind": "electronic" }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
        TempJsonFile f(json, "vehicle_throttle_unknown_engine");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("must name a \"torque_map\" engine component"));
    }
    SECTION("throttle_control naming a component that is not an engine (the clutch) is rejected") {
        const std::string json = std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "engine", "type": "engine", "kind": "torque_map",
        "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
        "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
        "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0 },
      { "id": "clutch", "type": "clutch", "capacity_nm": 300.0,
        "engagement_vs_pedal": { "x": [0.0, 1.0], "y": [1.0, 0.0] } },
      )") + gb.component_json + R"(,
      { "id": "diff", "type": "differential", "kind": "open", "ratio": 3.7 },
      { "id": "throttle", "type": "throttle_control", "engine": "clutch", "kind": "electronic" }
    ],
    "shafts": [
      { "ports": ["engine.crank", "clutch.in"] },
      { "ports": ["clutch.out", "gearbox.in"] },
      { "ports": ["gearbox.out", "diff.carrier"] },
      { "ports": ["diff.left", "wheel:FL"] },
      { "ports": ["diff.right", "wheel:FR"] }
    ]
  }
})";
        TempJsonFile f(json, "vehicle_throttle_names_clutch");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("'engine' (\"clutch\") must name a \"torque_map\" engine component"));
    }
    SECTION("throttle_control in a vehicle with no engine component at all is rejected") {
        const std::string json = std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      { "id": "throttle", "type": "throttle_control", "engine": "engine", "kind": "electronic" }
    ],
    "shafts": []
  }
})");
        TempJsonFile f(json, "vehicle_throttle_no_engine");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("a throttle_control component in a vehicle with no engine component"));
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
        // I4 C9), diff (rear), front_diff, park (park_lock, PHYS-033 P4
        // I4 C10), pto (power take-off, PHYS-033 P4 I4 C11) - up from 4 once
        // the transfer case, front axle differential, park lock and PTO were
        // added.
        REQUIRE(v.powertrain.components.size() == 8);
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
        const auto* park = std::get_if<drivetrain::ParkLockDesc>(&v.powertrain.components[6].params);
        REQUIRE(park != nullptr);
        CHECK(park->capacity_nm == Catch::Approx(30000.0));
        CHECK(park->static_factor == Catch::Approx(1.0));
        CHECK(park->max_engage_speed_rad_s == Catch::Approx(5.0));
        CHECK(park->channel == "park_lock");
        const auto* pto = std::get_if<drivetrain::PtoDesc>(&v.powertrain.components[7].params);
        REQUIRE(pto != nullptr);
        CHECK(pto->has_540);
        CHECK(pto->ratio_540 == Catch::Approx(3.61));
        CHECK(pto->has_1000);
        CHECK(pto->ratio_1000 == Catch::Approx(1.95));
        CHECK(pto->capacity_nm == Catch::Approx(700.0));
        CHECK(pto->efficiency == Catch::Approx(0.96));
        CHECK(pto->channel == "pto");
        CHECK(pto->speed_channel == "pto_speed");
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
