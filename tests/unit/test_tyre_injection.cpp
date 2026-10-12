// tests/unit/test_tyre_injection.cpp - VehicleLoadOptions::tyres (injected tyres keyed by wheel name) and the
// "wheel with no tyre is a load error" rule of ps::io::load_vehicle_json (docs/data_formats/vehicle.md "Injected tyres").
//
// Bands (fixed before the first run): every comparison is EXACT equality (bit for bit / identical string); there is no
// tolerance anywhere in this file.
//
// TI1  an injected desc is what the wheel uses (bit-identical TyreDesc in the returned VehicleDesc), the other wheels
//      keep their file tyre.
// TI2  car_sedan with the four tyres injected as the very desc its file names: identical tyres, identical state hash
//      and identical per-wheel telemetry after a short drive, without options and with an empty options struct too.
// TI3  an injected desc that differs really reaches the physics (the state hash moves) - without it TI2 would also
//      pass if the injection were silently ignored.
// TI4  a wheel with neither a "tyre" key nor an injected desc: the error names the vehicle file and the wheel, with
//      the exact message; an injected desc cures it for that wheel only.
// TI5  an injected key that names no wheel is rejected (typo guard).
// TI6  an injected desc with an empty source (or an out-of-range low-speed damping) is rejected with the file tyre's
//      own message; the same defect in an inline file tyre gives the identical message (one validation function).
// TI7  an injected desc replaces a file tyre (inline object or path); a path that does not exist is never read.
// TI8  unknown wheel keys (e.g. a game's own "tyre_spec") are ignored by the loader, as before.

#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ps;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::Equals;

namespace {

struct TempVehicle {
    std::filesystem::path path;
    explicit TempVehicle(const std::string& contents) {
        static int counter = 0;
        path = std::filesystem::temp_directory_path() /
               ("ps_test_tyre_injection_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" +
                std::to_string(counter++) + ".json");
        std::ofstream out(path);
        out << contents;
    }
    ~TempVehicle() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    std::string str() const { return path.string(); }
};

std::string vehicle_with_wheels(const std::string& wheels_json) {
    return R"JSON({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle (test_tyre_injection.cpp)",
  "wheels": )JSON" + wheels_json + "\n}";
}

const char* kInlineTyre = R"({ "name": "file tyre", "source": "test fixture", "pkx1": 11.0 })";

vehicle::TyreDesc make_tyre(const std::string& name, double pkx1) {
    vehicle::TyreDesc t;
    t.name = name;
    t.source = "test_tyre_injection.cpp (synthetic)";
    t.pkx1 = pkx1;
    return t;
}

// Bit-for-bit equality of every TyreDesc member: the strings, the four header members, and the contiguous block of
// reals from unloaded_radius to low_speed_damping_vx (the last member; nothing but `real` in between).
bool tyre_identical(const vehicle::TyreDesc& a, const vehicle::TyreDesc& b) {
    if (a.name != b.name || a.source != b.source) return false;
    if (a.rolling_resistance_speed_law != b.rolling_resistance_speed_law) return false;
    if (std::memcmp(&a.rolling_resistance_anchors, &b.rolling_resistance_anchors, sizeof a.rolling_resistance_anchors) != 0)
        return false;
    if (std::memcmp(&a.rolling_resistance_quadratic, &b.rolling_resistance_quadratic, sizeof(real)) != 0) return false;
    if (std::memcmp(&a.rolling_resistance_reference_speed_m_s, &b.rolling_resistance_reference_speed_m_s, sizeof(real)) != 0)
        return false;
    const auto* pa = reinterpret_cast<const unsigned char*>(&a.unloaded_radius);
    const auto* pb = reinterpret_cast<const unsigned char*>(&b.unloaded_radius);
    const std::size_t n = static_cast<std::size_t>(reinterpret_cast<const unsigned char*>(&a.low_speed_damping_vx) - pa) +
                          sizeof(real);
    return std::memcmp(pa, pb, n) == 0;
}

std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

struct DriveResult {
    std::uint64_t hash = 0;
    std::vector<vehicle::WheelTelemetry> wheels;
};

DriveResult drive(const vehicle::VehicleDesc& desc, int workers) {
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.6);
    w.set_control("steer", 0.2);
    ps_test::settle(w, 1.5);
    DriveResult r;
    r.hash = w.state_hash();
    for (std::size_t i = 0; i < w.vehicle_wheel_count(rig.vehicle); ++i) r.wheels.push_back(w.wheel_telemetry(rig.vehicle, i));
    return r;
}

bool telemetry_identical(const std::vector<vehicle::WheelTelemetry>& a, const std::vector<vehicle::WheelTelemetry>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const auto& x = a[i];
        const auto& y = b[i];
        if (!(x.slip_ratio == y.slip_ratio && x.slip_angle == y.slip_angle && x.fx == y.fx && x.fy == y.fy &&
              x.fz == y.fz && x.omega == y.omega && x.compression == y.compression && x.vx_contact == y.vx_contact &&
              x.surface == y.surface))
            return false;
    }
    return true;
}

} // namespace

TEST_CASE("TI1 an injected tyre is what the wheel uses", "[io][vehicle][tyre]") {
    const std::string car = data_path("vehicles/car_sedan.json");
    const vehicle::VehicleDesc base = io::load_vehicle_json(car);

    io::VehicleLoadOptions opt;
    const vehicle::TyreDesc injected = make_tyre("injected FL", 23.5);
    opt.tyres["FL"] = injected;
    const vehicle::VehicleDesc v = io::load_vehicle_json(car, opt);

    REQUIRE(v.wheels.size() == base.wheels.size());
    REQUIRE(base.wheels[0].name == "FL");
    CHECK(tyre_identical(v.wheels[0].tyre, injected));
    CHECK_FALSE(tyre_identical(v.wheels[0].tyre, base.wheels[0].tyre));
    for (std::size_t i = 1; i < v.wheels.size(); ++i) CHECK(tyre_identical(v.wheels[i].tyre, base.wheels[i].tyre));
}

TEST_CASE("TI2 injecting the tyre the file names changes nothing", "[io][vehicle][tyre][determinism]") {
    const std::string car = data_path("vehicles/car_sedan.json");
    const vehicle::VehicleDesc base = io::load_vehicle_json(car);
    const vehicle::TyreDesc file_tyre = io::load_tyre_json(data_path("tyres/passenger_car_225_45r17.json"));

    io::VehicleLoadOptions opt;
    for (const auto& w : base.wheels) opt.tyres[w.name] = file_tyre;
    const vehicle::VehicleDesc injected = io::load_vehicle_json(car, opt);
    const vehicle::VehicleDesc empty_opts = io::load_vehicle_json(car, io::VehicleLoadOptions{});

    REQUIRE(injected.wheels.size() == base.wheels.size());
    for (std::size_t i = 0; i < base.wheels.size(); ++i) {
        CHECK(tyre_identical(injected.wheels[i].tyre, base.wheels[i].tyre));
        CHECK(tyre_identical(empty_opts.wheels[i].tyre, base.wheels[i].tyre));
    }

    const DriveResult a = drive(base, 1);
    const DriveResult b = drive(injected, 1);
    const DriveResult c = drive(empty_opts, 1);
    const DriveResult d = drive(injected, 10);
    CHECK(a.hash == b.hash);
    CHECK(a.hash == c.hash);
    CHECK(a.hash == d.hash);
    CHECK(telemetry_identical(a.wheels, b.wheels));
    CHECK(telemetry_identical(a.wheels, c.wheels));
}

TEST_CASE("TI3 an injected tyre that differs reaches the physics", "[io][vehicle][tyre][determinism]") {
    const std::string car = data_path("vehicles/car_sedan.json");
    const vehicle::VehicleDesc base = io::load_vehicle_json(car);
    vehicle::TyreDesc grippier = io::load_tyre_json(data_path("tyres/passenger_car_225_45r17.json"));
    grippier.lambda_mux *= 0.5;
    grippier.lambda_muy *= 0.5;

    io::VehicleLoadOptions opt;
    for (const auto& w : base.wheels) opt.tyres[w.name] = grippier;
    const vehicle::VehicleDesc v = io::load_vehicle_json(car, opt);
    CHECK(drive(base, 1).hash != drive(v, 1).hash);
}

TEST_CASE("TI4 a wheel without a tyre is a load error", "[io][vehicle][tyre]") {
    const TempVehicle f(vehicle_with_wheels(std::string("[ { \"name\": \"FL\", \"tyre\": ") + kInlineTyre +
                                            " }, { \"name\": \"RR\" } ]"));
    const std::string expected_tail =
        "wheel 'RR': no tyre (give a \"tyre\" path/object or VehicleLoadOptions::tyres)";
    SECTION("no options") {
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str()), ContainsSubstring(expected_tail));
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str()), ContainsSubstring(f.str()));
    }
    SECTION("exact message") {
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str()), Equals("vehicle " + f.str() + ": " + expected_tail));
    }
    SECTION("injecting the other wheel's name does not cure it") {
        io::VehicleLoadOptions opt;
        opt.tyres["FL"] = make_tyre("x", 10.0);
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), opt), ContainsSubstring(expected_tail));
    }
    SECTION("injecting it cures it") {
        io::VehicleLoadOptions opt;
        opt.tyres["RR"] = make_tyre("rear", 12.5);
        const vehicle::VehicleDesc v = io::load_vehicle_json(f.str(), opt);
        REQUIRE(v.wheels.size() == 2);
        CHECK(v.wheels[0].tyre.name == "file tyre");
        CHECK(tyre_identical(v.wheels[1].tyre, make_tyre("rear", 12.5)));
    }
    SECTION("the default wheel name is 'wheel'") {
        const TempVehicle g(vehicle_with_wheels("[ { } ]"));
        REQUIRE_THROWS_WITH(io::load_vehicle_json(g.str()), ContainsSubstring("wheel 'wheel': no tyre"));
    }
}

TEST_CASE("TI5 an injected key that names no wheel is rejected", "[io][vehicle][tyre]") {
    const TempVehicle f(vehicle_with_wheels(std::string("[ { \"name\": \"FL\", \"tyre\": ") + kInlineTyre + " } ]"));
    io::VehicleLoadOptions opt;
    opt.tyres["FL"] = make_tyre("ok", 10.0);
    opt.tyres["FLL"] = make_tyre("typo", 10.0);
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), opt),
                        Equals("vehicle " + f.str() + ": VehicleLoadOptions::tyres: no wheel named 'FLL'"));
}

TEST_CASE("TI6 an injected tyre is validated like a file tyre", "[io][vehicle][tyre]") {
    const TempVehicle f(vehicle_with_wheels("[ { \"name\": \"FL\" } ]"));
    SECTION("empty source") {
        io::VehicleLoadOptions opt;
        vehicle::TyreDesc t = make_tyre("nosrc", 10.0);
        t.source.clear();
        opt.tyres["FL"] = t;
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), opt),
                            Equals("tyre 'nosrc': 'source' is required (PLAN.md section 8)"));
        // The file path gives the identical message for the same defect.
        const TempVehicle g(vehicle_with_wheels(R"([ { "name": "FL", "tyre": { "name": "nosrc" } } ])"));
        REQUIRE_THROWS_WITH(io::load_vehicle_json(g.str()),
                            Equals("tyre 'nosrc': 'source' is required (PLAN.md section 8)"));
    }
    SECTION("low-speed damping ratio out of range") {
        io::VehicleLoadOptions opt;
        vehicle::TyreDesc t = make_tyre("badzeta", 10.0);
        t.low_speed_damping_ratio = 3.0;
        opt.tyres["FL"] = t;
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), opt), ContainsSubstring("low_speed_damping_ratio must be in [0, 2]"));
        const TempVehicle g(vehicle_with_wheels(
            R"([ { "name": "FL", "tyre": { "name": "badzeta", "source": "x", "low_speed_damping_ratio": 3.0 } } ])"));
        REQUIRE_THROWS_WITH(io::load_vehicle_json(g.str()), ContainsSubstring("low_speed_damping_ratio must be in [0, 2]"));
    }
    SECTION("rolling resistance ramp width") {
        io::VehicleLoadOptions opt;
        vehicle::TyreDesc t = make_tyre("badeps", 10.0);
        t.rolling_resistance_v_eps = 0.0;
        opt.tyres["FL"] = t;
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), opt), ContainsSubstring("rolling_resistance_v_eps must be > 0"));
    }
    SECTION("low-speed damping fade speed") {
        io::VehicleLoadOptions opt;
        vehicle::TyreDesc t = make_tyre("badvx", 10.0);
        t.low_speed_damping_vx = 0.0;
        opt.tyres["FL"] = t;
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), opt), ContainsSubstring("low_speed_damping_vx must be > 0"));
    }
}

TEST_CASE("TI7 an injected tyre replaces a file tyre", "[io][vehicle][tyre]") {
    const TempVehicle f(vehicle_with_wheels(std::string("[ { \"name\": \"FL\", \"tyre\": ") + kInlineTyre +
                                            " }, { \"name\": \"FR\", \"tyre\": \"does_not_exist_anywhere.json\" } ]"));
    io::VehicleLoadOptions opt;
    opt.tyres["FL"] = make_tyre("replacement FL", 31.0);
    opt.tyres["FR"] = make_tyre("replacement FR", 32.0);
    const vehicle::VehicleDesc v = io::load_vehicle_json(f.str(), opt);
    CHECK(tyre_identical(v.wheels[0].tyre, make_tyre("replacement FL", 31.0)));
    // The path of the replaced tyre is never opened (the file does not exist, loading still succeeds).
    CHECK(tyre_identical(v.wheels[1].tyre, make_tyre("replacement FR", 32.0)));
    // Without the injection the same file is a load error (the path is read).
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str()), ContainsSubstring("does_not_exist_anywhere.json"));
    // With only FL injected, FR's file tyre is still read.
    io::VehicleLoadOptions only_fl;
    only_fl.tyres["FL"] = make_tyre("replacement FL", 31.0);
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str(), only_fl), ContainsSubstring("does_not_exist_anywhere.json"));
}

TEST_CASE("TI8 unknown wheel keys are ignored", "[io][vehicle][tyre]") {
    const TempVehicle f(vehicle_with_wheels(std::string("[ { \"name\": \"FL\", \"tyre_spec\": { \"compound\": \"soft\" } } ]")));
    io::VehicleLoadOptions opt;
    opt.tyres["FL"] = make_tyre("game tyre", 17.0);
    const vehicle::VehicleDesc v = io::load_vehicle_json(f.str(), opt);
    REQUIRE(v.wheels.size() == 1);
    CHECK(tyre_identical(v.wheels[0].tyre, make_tyre("game tyre", 17.0)));
    // ... but "tyre_spec" is not a tyre: without the injection the wheel has none.
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.str()), ContainsSubstring("wheel 'FL': no tyre"));
}
