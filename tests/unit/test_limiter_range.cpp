// tests/unit/test_limiter_range.cpp - G1 limiter-range robustness (docs/next_steps_brief.md 2.1, ps/drivetrain/limiter_range.h,
// docs/data_formats/engine.md "Beyond the authored rpm range"). A garage limiter anywhere in 2000..20000 rpm must load, drive and
// sweep for every shipped car; the four rules that make it so:
//
//   R*  the roll-off law (RpmRolloff), the launch-rpm fit and the shift-point scale as pure functions
//   S*  every shipped engine and car is untouched: roll-off disarmed, no shift scaling, no launch clamp, no G1 warning, and an
//       armed roll-off leaves every rpm inside the data bit-identical (the 20-scenario hash gate is the whole-World proof)
//   L*  launch control: an rpm above max_limiter_fraction x limiter is held there (assist unit and P8 two-step), through a World
//   A*  auto-shift: points above the cap are scaled (car points and drive-mode overrides), through a World
//   P*  plateau tolerance: toyota_a25a_fks_gen's Sketch maps carry a small WOT dip that the old exact check rejected
//   E*  [long] every shipped car at a 2000 and a 20000 rpm limiter: loads (with the findings as load warnings), drives and sweeps
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS] unless marked):
//   R1  factor = 1 up to n_e, (n_z - rpm)/(n_z - n_e) inside, 0 from n_z; n_z = n_e (1 + b); slope -1/(n_z - n_e) inside only
//   L1  sedan (i4_2l_na, limiter 6800 [D], idle 800 [D]) + Racing unit with launch_rpm 7000: held at 0.8 x 6800 = 5440 [D];
//       armed 1.5 s at brake 1 / throttle 1: every tick of the last 0.5 s within [5440 - h - 25, 5440 + 50] (h = 150 [D], the
//       limiter's hysteresis, the corrected V8 band of test_assist_launch.cpp)
//   L2  the same with the P8 two-step part at rpm 7000, in neutral, channel on: last 0.5 s of 1.5 s within the same band
//   A1  sedan at a 2000 rpm limiter, auto-shift on, WOT from rest for 15 s: reaches gear >= 2 (without the scale the 6300 rpm
//       upshift is never reached and the car stays in 1st); the peak engine rpm <= 2000 + 150 overshoot
//   A2  car_hyper_n2o at a 2000 rpm limiter in Sport (override up 8100): reaches gear >= 2 in 15 s
//   P1  toyota_a25a_fks_gen at Sketch and Preview quality: SimulatedEngine::build_maps throws "not non-decreasing in plate"
//       with plate_monotone_tolerance 0 and builds with the default; the default dyno sweep completes. (The default was set
//       from the measurement, not before it: first band 0.005 from racing_game's reported -0.16 % at 2000 rpm; measured
//       Sketch 3.0 %, Preview 0.11 % [M] -> default 0.04, and 0.025 must still throw.)
//   E   per car x {2000, 20000}: load and World creation do not throw; the limiter reads back; 12 s WOT with auto-shift: every
//       chassis coordinate finite, peak engine rpm <= max(limiter, n_z) + 300; the default dyno sweep (idle..limiter) does not
//       throw, every point's torque is finite; where the roll-off is armed: BeyondEngineData findings exist (Limiting inside the
//       band, Exceeded from n_z on) and every point at or above n_z has a net torque <= 0 (no fired torque); where it is
//       disarmed: no BeyondEngineData finding
//
// RECORDED SABOTAGES (temporary edits, reverted; each rebuilt and the shown cases run, Linux release, 2026-10-14):
//   a  make_rolloff never arms ("false && ..."):            R1 (REQUIRE armed), S1 (armed list), S3 (REQUIRE armed) fail
//   b  shift_point_scale always 1:                          R2 (two scale checks), A1 (no "scaled by", max gear 1) fail
//   c  fit_launch_rpm never clamps:                         R2 (five fit checks), L1 (load throws), L2 (no "held at") fail
//   d  plate check back to exact (allowed = 1e-9):          P1 (default Sketch and Preview builds throw) fails
//   e  torque-map BeyondEngineData finding not emitted:     E1 (findings iff armed, car_sedan @20000) fails
//   f  RpmRolloff::active from 0.95 n_e (inside the data):  R1 (active at 7000), S3 (torque and fuel differ inside) fail

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limiter_range.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "drivetrain/engine_simulated.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
const std::string kRacing = kData + "/electronics/assist_unit_racing.json";
const std::string kTwoStep = kData + "/electronics/launch_two_step_generic.json";

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_limiter_range_" + tag);
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    [[nodiscard]] std::string file(const std::string& name) const { return (path / name).generic_string(); }
};

void write_text(const std::string& p, const std::string& text) {
    std::ofstream f(p, std::ios::binary);
    f << text;
}

std::string cache_dir() {
    const std::string d = std::string(PS_SOURCE_DIR) + "/out/limiter_range_map_cache";
    fs::create_directories(d);
    return d;
}

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

bool any_contains(const std::vector<std::string>& v, const std::string& needle) {
    return std::any_of(v.begin(), v.end(), [&](const std::string& s) { return s.find(needle) != std::string::npos; });
}

const ComponentParams& engine_params(const vehicle::VehicleDesc& d) {
    for (const ComponentDesc& c : d.powertrain.components) {
        if (std::holds_alternative<TorqueMapEngineDesc>(c.params) || std::holds_alternative<SimulatedEngineDesc>(c.params)) return c.params;
    }
    FAIL("no engine component");
    return d.powertrain.components.front().params;
}
ComponentParams& engine_params(vehicle::VehicleDesc& d) {
    return const_cast<ComponentParams&>(engine_params(static_cast<const vehicle::VehicleDesc&>(d)));
}
RpmRolloff rolloff_of(const ComponentParams& p) {
    if (const auto* t = std::get_if<TorqueMapEngineDesc>(&p)) return rpm_rolloff(*t);
    if (const auto* s = std::get_if<SimulatedEngineDesc>(&p)) return rpm_rolloff(*s);
    return {};
}
EngineLimiterDesc limiter_of(const ComponentParams& p) {
    if (const auto* t = std::get_if<TorqueMapEngineDesc>(&p)) return t->limiter;
    return std::get<SimulatedEngineDesc>(p).limiter;
}
real idle_of(const ComponentParams& p) {
    if (const auto* t = std::get_if<TorqueMapEngineDesc>(&p)) return t->idle_rpm;
    return std::get<SimulatedEngineDesc>(p).idle_rpm;
}

// One shipped car and where its limiter lives.
struct Car {
    const char* file;      // data/vehicles/<file>
    const char* engine;    // data/engines/<engine>
    const char* ecu;       // data/ecus/<ecu> when the limiter is an ECU part field, else nullptr
};
constexpr Car kCars[] = {
    {"car_sedan.json", "i4_2l_na.json", nullptr},
    {"car_sedan_sim.json", "toyota_a25a_fks_sim.json", nullptr},
    {"car_hyper.json", "hyper_v8_one1_audio.json", "hyper_v8_one1.json"},
    {"car_hyper_n2o.json", "hyper_v8_one1_audio.json", "hyper_v8_one1.json"},
    {"tractor.json", "diesel_i4_4l5_turbo.json", nullptr},
};

// The garage's limiter move: a user part that extends the shipped one with only limiter.rpm changed (RFC 7396 merge).
io::VehicleLoadOptions limiter_override(const TempDir& td, const Car& car, real rpm) {
    io::VehicleLoadOptions o;
    o.maps.pool = &pool();
    o.maps.cache_dir = cache_dir();
    const json patch = {{"limiter", {{"rpm", rpm}}}};
    io::PartOverride po;
    if (car.ecu != nullptr) {
        json j = patch;
        j["extends"] = kData + "/ecus/" + car.ecu;
        write_text(td.file("ecu.json"), j.dump());
        po.slots["ecu"] = td.file("ecu.json");
    } else {
        json j = patch;
        j["extends"] = kData + "/engines/" + car.engine;
        write_text(td.file("engine.json"), j.dump());
        po.ref = td.file("engine.json");
    }
    o.part_overrides["engine"] = po;
    return o;
}

vehicle::VehicleDesc load_car(const Car& car, io::VehicleLoadOptions o = {}) {
    if (o.maps.pool == nullptr) o.maps.pool = &pool();
    if (o.maps.cache_dir.empty()) o.maps.cache_dir = cache_dir();
    return io::load_vehicle_json(kData + "/vehicles/" + car.file, o);
}

const ManualTcuDesc* tcu_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers)
        if (const auto* t = std::get_if<ManualTcuDesc>(&c.params)) return t;
    return nullptr;
}

struct DriveStats {
    bool finite = true;
    real max_rpm = 0.0;
    int max_gear = 0;
    EngineState end_state = EngineState::Off;
};

// WOT from rest, auto-clutch + auto-shift, `seconds` of 240 Hz ticks.
DriveStats drive_wot(const vehicle::VehicleDesc& d, real seconds, int drive_mode = -1) {
    ps_test::VehicleTestRig rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0.0, 1, Vec3::unit_y(), 240.0, 960.0,
                                                                       4000.0);
    World& w = *rig.world;
    if (drive_mode >= 0) w.set_control("assist.drive_mode", static_cast<real>(drive_mode));
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("assist.launch_control", 0.0);
    w.set_control("gear_request", 1.0);
    for (int i = 0; i < 120; ++i) w.step(); // settle in 1st
    w.set_control("gear_request", kGearRequestNone); // gear_request outranks the auto-shift (ManualTcu)
    w.set_control("throttle", 1.0);
    DriveStats s;
    const int ticks = static_cast<int>(seconds * 240.0);
    for (int i = 0; i < ticks; ++i) {
        w.step();
        const auto& pt = w.powertrain_state(rig.vehicle);
        s.max_rpm = std::max(s.max_rpm, pt.engine_rpm);
        s.max_gear = std::max<int>(s.max_gear, pt.gear);
        const Vec3 p = w.get_pose(rig.chassis).position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(pt.engine_rpm)) {
            s.finite = false;
            break;
        }
        s.end_state = pt.engine_state;
    }
    return s;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// R: the rules as pure functions.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("R1: the roll-off law beyond the engine's data", "[limiter_range][drivetrain]") {
    TorqueMapEngineDesc e;
    e.wot_torque_nm_vs_rpm.x = {1000, 3000, 6000};
    e.wot_torque_nm_vs_rpm.y = {100, 200, 180};
    e.motoring_torque_nm_vs_rpm.x = {0, 7000};
    e.motoring_torque_nm_vs_rpm.y = {-10, -40};
    e.limiter.rpm = 9000;
    CHECK(authored_top_rpm(e) == 7000.0); // the motoring axis reaches further than the WOT axis
    RpmRolloff r = rpm_rolloff(e);
    REQUIRE(r.armed());
    CHECK(r.start_rpm == 7000.0);
    CHECK(r.end_rpm == Catch::Approx(8400.0)); // 7000 x (1 + 0.2)
    CHECK(r.factor(7000.0) == 1.0);
    CHECK_FALSE(r.active(7000.0));
    CHECK(r.factor(7700.0) == Catch::Approx(0.5));
    CHECK(r.factor(8400.0) == 0.0);
    CHECK(r.factor(20000.0) == 0.0);
    CHECK(r.slope_per_rpm(7700.0) == Catch::Approx(-1.0 / 1400.0));
    CHECK(r.slope_per_rpm(6000.0) == 0.0);
    CHECK(r.slope_per_rpm(9000.0) == 0.0);
    CHECK(effective_limiter_rpm(e) == Catch::Approx(8400.0)); // no fired torque past n_z

    // disarmed: limiter inside or at the data, band 0
    e.limiter.rpm = 7000;
    CHECK_FALSE(rpm_rolloff(e).armed());
    CHECK(rpm_rolloff(e).factor(50000.0) == 1.0);
    e.limiter.rpm = 9000;
    e.rpm_rolloff_band_fraction = 0.0;
    CHECK_FALSE(rpm_rolloff(e).armed());
    // a governor's high idle below the data disarms it too (the tractor's case)
    e.rpm_rolloff_band_fraction = 0.2;
    e.has_governor = true;
    e.governor.high_idle_rpm = 6500;
    CHECK_FALSE(rpm_rolloff(e).armed());
    CHECK(effective_limiter_rpm(e) == 6500.0);
    // SoftCut: the taper's start counts
    e.has_governor = false;
    e.limiter.rpm = 6000;
    e.limiter.type = LimiterType::SoftCut;
    e.limiter.band_rpm = 200;
    CHECK(effective_limiter_rpm(e) == 5800.0);
}

TEST_CASE("R2: launch rpm fit and shift-point scale", "[limiter_range][drivetrain]") {
    // below the cap: unchanged, bit for bit
    LaunchRpmFit f = fit_launch_rpm(4000.0, 800.0, 6800.0, 0.8);
    CHECK_FALSE(f.clamped);
    CHECK(f.rpm == 4000.0);
    CHECK(f.cap == Catch::Approx(5440.0));
    f = fit_launch_rpm(7000.0, 800.0, 6800.0, 0.8);
    CHECK(f.clamped);
    CHECK(f.rpm == Catch::Approx(5440.0));
    f = fit_launch_rpm(5000.0, 950.0, 2000.0, 0.8); // the hyper's Racing unit at a 2000 rpm limiter
    CHECK(f.rpm == Catch::Approx(1600.0));
    f = fit_launch_rpm(5000.0, 950.0, 1100.0, 0.8); // cap 880 <= idle 950: the midpoint 1025
    CHECK(f.clamped);
    CHECK(f.rpm == Catch::Approx(1025.0));

    CHECK(shift_point_scale(2500.0, 6300.0, 6698.0) == 1.0);
    CHECK(shift_point_scale(2500.0, 6300.0, 0.0) == 1.0); // no cap
    CHECK(shift_point_scale(2500.0, 6300.0, 1970.0) == Catch::Approx(1970.0 / 6300.0));
    CHECK(shift_point_scale(7000.0, 6300.0, 1970.0) == Catch::Approx(1970.0 / 7000.0)); // the larger point sets it
}

// ---------------------------------------------------------------------------------------------------------------
// S: the shipped data is untouched.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("S1: every shipped engine file has its roll-off disarmed, except demo_i4_pipes", "[limiter_range][drivetrain][io]") {
    // demo_i4_pipes.json (the [AS] audio demo engine, not a real engine) names no limiter, so it takes the default 6800 rpm
    // [D] above its cycle grid's top node of 6500 rpm [D]: it is the one shipped engine whose own data stops short of its
    // limiter, and its roll-off is armed over 6500..7800 rpm. Pre-G1 its maps were edge-clamped (the 6500 row held flat to
    // 6800); that 6500..6800 strip is the one shipped behaviour G1 moves. Every other engine file is untouched.
    io::EngineMapOptions o;
    o.generate = false; // the axes only (a pending engine's top is its cycle grid's)
    int n = 0;
    std::vector<std::string> armed;
    for (const auto& entry : fs::directory_iterator(kData + "/engines")) {
        if (entry.path().extension() != ".json") continue;
        const std::string name = entry.path().filename().string();
        INFO(name);
        const ComponentParams p = io::load_engine_json(entry.path().string(), o);
        const RpmRolloff r = rolloff_of(p);
        if (r.armed()) {
            armed.push_back(name);
            CHECK(r.start_rpm == 6500.0);
            CHECK(r.end_rpm == Catch::Approx(7800.0));
            CHECK(effective_limiter_rpm(p) == 6800.0);
        }
        ++n;
    }
    CHECK(n >= 10);
    CHECK(armed == std::vector<std::string>{"demo_i4_pipes.json"});
}

TEST_CASE("S2: shipped cars - no G1 warning, no shift scaling, no launch clamp", "[limiter_range][drivetrain][io][vehicle]") {
    for (const Car& car : kCars) {
        INFO(car.file);
        io::VehicleLoadOptions o;
        std::vector<std::string> warn;
        o.maps.warnings_out = &warn;
        const vehicle::VehicleDesc d = load_car(car, o);
        CHECK_FALSE(any_contains(warn, "rolls off"));
        CHECK_FALSE(any_contains(warn, "scaled by"));
        CHECK_FALSE(any_contains(warn, "held at"));
        const ComponentParams& e = engine_params(d);
        CHECK_FALSE(rolloff_of(e).armed());
        const ManualTcuDesc* t = tcu_of(d);
        REQUIRE(t != nullptr);
        const real cap = t->auto_shift.max_limiter_fraction * effective_limiter_rpm(e);
        CHECK(shift_point_scale(t->auto_shift.up_rpm_at_zero_throttle, t->auto_shift.up_rpm_at_full_throttle, cap) == 1.0);
        if (d.drive_modes) {
            for (int mi = 0; mi < vehicle::kDriveModeCount; ++mi) {
                if (!(d.drive_modes->modes_bits & vehicle::drive_mode_bit(static_cast<vehicle::DriveMode>(mi)))) continue;
                const auto& m = d.drive_modes->params[static_cast<std::size_t>(mi)];
                CHECK(shift_point_scale(m.up_rpm_at_zero_throttle.value_or(0.0), m.up_rpm_at_full_throttle.value_or(0.0), cap) == 1.0);
            }
        }
        if (d.assist_unit && d.assist_unit->launch_control) {
            const auto& la = *d.assist_unit->launch_control;
            const real lim = limiter_of(e).rpm;
            CHECK_FALSE(fit_launch_rpm(vehicle::launch_assist_rpm(la, lim), idle_of(e), lim, la.max_limiter_fraction).clamped);
        }
    }
    // the shipped assist units and the two-step on the sedan (the smallest limiter of a car that takes them)
    for (const std::string& unit : {kRacing, kData + "/electronics/assist_unit_sport.json"}) {
        io::VehicleLoadOptions o;
        std::vector<std::string> warn;
        o.maps.warnings_out = &warn;
        o.electronics["assists"] = unit;
        (void)load_car(kCars[0], o);
        CHECK_FALSE(any_contains(warn, "held at"));
    }
    io::VehicleLoadOptions o;
    std::vector<std::string> warn;
    o.maps.warnings_out = &warn;
    o.electronics["launch"] = kTwoStep;
    (void)load_car(kCars[0], o);
    CHECK_FALSE(any_contains(warn, "held at"));
}

TEST_CASE("S3: an armed roll-off leaves every rpm inside the data bit-identical", "[limiter_range][drivetrain][dyno]") {
    // torque map (i4_2l_na, data top 7000) and NA simulated (toyota_a25a_fks_sim, data top 7000): limiter 12000 arms the
    // roll-off; every swept point up to the data top equals the disarmed engine's bit for bit, every point above differs.
    for (int k = 0; k < 2; ++k) {
        const Car& car = kCars[k];
        INFO(car.file);
        vehicle::VehicleDesc d = load_car(car);
        ComponentParams armed = engine_params(d);
        ComponentParams disarmed = engine_params(d);
        std::visit(
            [](auto& e) {
                using T = std::decay_t<decltype(e)>;
                if constexpr (std::is_same_v<T, TorqueMapEngineDesc> || std::is_same_v<T, SimulatedEngineDesc>) e.limiter.rpm = 12000.0;
            },
            armed);
        std::visit(
            [](auto& e) {
                using T = std::decay_t<decltype(e)>;
                if constexpr (std::is_same_v<T, TorqueMapEngineDesc> || std::is_same_v<T, SimulatedEngineDesc>) {
                    e.limiter.rpm = 12000.0;
                    e.rpm_rolloff_band_fraction = 0.0;
                }
            },
            disarmed);
        const RpmRolloff r = rolloff_of(armed);
        REQUIRE(r.armed());
        CHECK(r.start_rpm == 7000.0);
        DynoSweepSpec spec;
        for (real rpm = 1000.0; rpm <= 9000.0; rpm += 250.0) spec.rpm.push_back(rpm);
        const DynoResult a = run_dyno_sweep(armed, nullptr, spec, &pool());
        const DynoResult b = run_dyno_sweep(disarmed, nullptr, spec, &pool());
        REQUIRE(a.points.size() == b.points.size());
        for (std::size_t i = 0; i < a.points.size(); ++i) {
            INFO(a.points[i].rpm);
            if (a.points[i].rpm <= r.start_rpm) {
                CHECK(a.points[i].torque_nm == b.points[i].torque_nm);
                CHECK(a.points[i].fuel_g_s == b.points[i].fuel_g_s);
            } else {
                CHECK(a.points[i].torque_nm < b.points[i].torque_nm);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// L: launch control above a low limiter.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("L1: an assist-unit launch rpm above the cap is held at it (World)", "[limiter_range][assist][vehicle]") {
    TempDir td("l1");
    std::ifstream f(kRacing);
    json j = json::parse(f);
    j["launch_control"]["launch_rpm"] = 7000;
    write_text(td.file("unit.json"), j.dump(1));
    io::VehicleLoadOptions o;
    std::vector<std::string> warn;
    o.maps.warnings_out = &warn;
    o.electronics["assists"] = td.file("unit.json");
    const vehicle::VehicleDesc d = load_car(kCars[0], o);
    CHECK(any_contains(warn, "held at 5440 rpm"));
    const real h = limiter_of(engine_params(d)).hysteresis_rpm;
    ps_test::VehicleTestRig rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("assist.launch_control", 1.0);
    w.set_control("gear_request", 1.0);
    for (int i = 0; i < 120; ++i) w.step();
    w.set_control("brake", 1.0);
    w.set_control("throttle", 1.0);
    real lo = 1e9, hi = 0.0;
    for (int i = 0; i < 360; ++i) {
        w.step();
        if (i >= 240) {
            const real rpm = w.powertrain_state(rig.vehicle).engine_rpm;
            lo = std::min(lo, rpm);
            hi = std::max(hi, rpm);
        }
    }
    std::printf("G1 L1: armed window rpm %.1f..%.1f (held 5440, h %.0f)\n", lo, hi, h);
    CHECK(lo >= 5440.0 - h - 25.0);
    CHECK(hi <= 5440.0 + 50.0);
}

TEST_CASE("L2: a P8 two-step above the cap is held at it (World)", "[limiter_range][drivetrain][vehicle]") {
    TempDir td("l2");
    write_text(td.file("lc.json"), R"({"format":"physics_sim.electronics/1","kind":"launch_control","rpm":7000,"max_speed_mps":2})");
    io::VehicleLoadOptions o;
    std::vector<std::string> warn;
    o.maps.warnings_out = &warn;
    o.electronics["launch"] = td.file("lc.json");
    const vehicle::VehicleDesc d = load_car(kCars[0], o);
    CHECK(any_contains(warn, "held at 5440 rpm"));
    const real h = limiter_of(engine_params(d)).hysteresis_rpm;
    ps_test::VehicleTestRig rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    // the P8 two-step acts with the clutch pedal in (test_assist_fitment.cpp run_launch): manual clutch, 1st gear, pedal in
    w.set_control("assist.auto_clutch", 0.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("clutch", 1.0);
    w.set_control("gear_request", 1.0);
    for (int i = 0; i < 240; ++i) w.step();
    w.set_control("assist.launch_control", 1.0);
    w.set_control("throttle", 1.0);
    real lo = 1e9, hi = 0.0;
    for (int i = 0; i < 360; ++i) {
        w.step();
        if (i >= 240) {
            const real rpm = w.powertrain_state(rig.vehicle).engine_rpm;
            lo = std::min(lo, rpm);
            hi = std::max(hi, rpm);
        }
    }
    std::printf("G1 L2: two-step window rpm %.1f..%.1f (held 5440, h %.0f)\n", lo, hi, h);
    CHECK(lo >= 5440.0 - h - 25.0);
    CHECK(hi <= 5440.0 + 50.0);
}

// ---------------------------------------------------------------------------------------------------------------
// A: auto-shift above a low limiter.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("A1: auto-shift points above the cap are scaled - the car upshifts at a 2000 rpm limiter", "[limiter_range][drivetrain][vehicle]") {
    TempDir td("a1");
    io::VehicleLoadOptions o = limiter_override(td, kCars[0], 2000.0);
    std::vector<std::string> warn;
    o.maps.warnings_out = &warn;
    const vehicle::VehicleDesc d = load_car(kCars[0], o);
    CHECK(any_contains(warn, "scaled by"));
    const DriveStats s = drive_wot(d, 15.0);
    std::printf("G1 A1: sedan @2000: max gear %d, peak %.0f rpm\n", s.max_gear, s.max_rpm);
    CHECK(s.finite);
    CHECK(s.max_gear >= 2);
    CHECK(s.max_rpm <= 2000.0 + 150.0);
}

TEST_CASE("A2: a drive mode's override points are scaled too", "[limiter_range][drivetrain][vehicle][long]") {
    TempDir td("a2");
    io::VehicleLoadOptions o = limiter_override(td, kCars[3], 2000.0);
    std::vector<std::string> warn;
    o.maps.warnings_out = &warn;
    const vehicle::VehicleDesc d = load_car(kCars[3], o);
    CHECK(any_contains(warn, "(drive mode 'sport')"));
    const DriveStats s = drive_wot(d, 15.0, 2);
    std::printf("G1 A2: hyper_n2o @2000 Sport: max gear %d, peak %.0f rpm\n", s.max_gear, s.max_rpm);
    CHECK(s.finite);
    CHECK(s.max_gear >= 2);
}

// ---------------------------------------------------------------------------------------------------------------
// P: a small WOT plateau in the generated A25A maps.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("P1: the plate check tolerates a small dip (toyota_a25a_fks_gen, Sketch)", "[limiter_range][drivetrain][cycle][long]") {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool();
    o.cache_dir = cache_dir();
    ComponentParams p = io::load_engine_json(kData + "/engines/toyota_a25a_fks_gen.json", o);
    SimulatedEngineDesc d = std::get<SimulatedEngineDesc>(p);
    CHECK(d.plate_monotone_tolerance == 0.04);
    CHECK_NOTHROW(SimulatedEngine::build_maps(d));
    d.plate_monotone_tolerance = 0.0; // the pre-G1 exact check
    CHECK_THROWS_WITH(SimulatedEngine::build_maps(d), ContainsSubstring("not non-decreasing in plate"));
    d.plate_monotone_tolerance = 0.025; // below the measured 3.0 % Sketch dip [M]
    CHECK_THROWS_WITH(SimulatedEngine::build_maps(d), ContainsSubstring("rpm 1500"));
    // Preview maps: 0.11 % dip [M] - the exact check throws, the default passes
    o.quality = MapQuality::Preview;
    SimulatedEngineDesc pv = std::get<SimulatedEngineDesc>(io::load_engine_json(kData + "/engines/toyota_a25a_fks_gen.json", o));
    CHECK_NOTHROW(SimulatedEngine::build_maps(pv));
    pv.plate_monotone_tolerance = 0.0;
    CHECK_THROWS_WITH(SimulatedEngine::build_maps(pv), ContainsSubstring("not non-decreasing in plate"));
    // the default dyno sweep (what the garage advisor runs) completes
    SimulatedEngineDesc dd = std::get<SimulatedEngineDesc>(p);
    DynoResult r;
    CHECK_NOTHROW(r = run_dyno_sweep(dd, nullptr, DynoSweepSpec{}, &pool()));
    CHECK_FALSE(r.points.empty());
}

// P2: inside the roll-off band the pumping excess (gas-exchange loss of the flow the roll-off cut) rolls off with the flow.
// Before, toyota_a25a_fks_gen at a 20000 rpm limiter threw "not non-decreasing in plate position" at rpm 8250: the
// plenum fills toward ambient on a cut air flow while the full pmep_excess of the WOT row stayed, so torque fell as the plate opened.
TEST_CASE("P2: a 20000 rpm limiter builds the steady tables (toyota_a25a_fks_gen, Sketch and Preview)", "[limiter_range][drivetrain][cycle][long]") {
    for (const MapQuality q : {MapQuality::Sketch, MapQuality::Preview}) {
        io::EngineMapOptions o;
        o.quality = q;
        o.pool = &pool();
        o.cache_dir = cache_dir();
        SimulatedEngineDesc d = std::get<SimulatedEngineDesc>(io::load_engine_json(kData + "/engines/toyota_a25a_fks_gen.json", o));
        d.limiter.rpm = 20000.0;
        CHECK_NOTHROW(SimulatedEngine::build_maps(d));
    }
}

// ---------------------------------------------------------------------------------------------------------------
// E: every shipped car at both garage extremes.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("E1: every shipped car loads, drives and sweeps at a 2000 and a 20000 rpm limiter", "[limiter_range][drivetrain][vehicle][dyno][long]") {
    for (const Car& car : kCars) {
        for (const real target : {2000.0, 20000.0}) {
            INFO(car.file << " @ " << target);
            TempDir td("e1");
            io::VehicleLoadOptions o = limiter_override(td, car, target);
            std::vector<std::string> warn;
            o.maps.warnings_out = &warn;
            vehicle::VehicleDesc d;
            REQUIRE_NOTHROW(d = load_car(car, o));
            const ComponentParams& e = engine_params(d);
            CHECK(limiter_of(e).rpm == target);
            const RpmRolloff r = rolloff_of(e);
            for (const std::string& w : warn) std::printf("G1 E1 %s @%.0f warn: %s\n", car.file, target, w.c_str());
            CHECK(any_contains(warn, "rolls off") == r.armed());
            // the drive
            DriveStats s;
            REQUIRE_NOTHROW(s = drive_wot(d, 12.0));
            const real top = std::max(target, r.armed() ? r.end_rpm : real(0.0));
            std::printf("G1 E1 %s @%.0f: rolloff %s (%.0f..%.0f) | drive: max gear %d, peak %.0f rpm, engine state %d\n", car.file, target,
                        r.armed() ? "armed" : "off", r.start_rpm, r.end_rpm, s.max_gear, s.max_rpm, static_cast<int>(s.end_state));
            CHECK(s.finite);
            CHECK(s.max_rpm <= top + 300.0);
            // the sweep
            DynoResult res;
            REQUIRE_NOTHROW(res = run_dyno_sweep(e, nullptr, DynoSweepSpec{}, &pool()));
            REQUIRE_FALSE(res.points.empty());
            int beyond = 0, exceeded = 0;
            for (const LimitFinding& f : res.limits) {
                if (f.kind != LimitKind::BeyondEngineData) continue;
                ++beyond;
                if (f.severity == LimitSeverity::Exceeded) {
                    ++exceeded;
                    CHECK(f.rpm >= r.end_rpm);
                } else {
                    CHECK(f.severity == LimitSeverity::Limiting);
                    CHECK(f.rpm < r.end_rpm);
                }
            }
            for (const DynoPoint& pt : res.points) {
                INFO(pt.rpm);
                CHECK(std::isfinite(pt.torque_nm));
                if (r.armed() && pt.rpm >= r.end_rpm) CHECK(pt.torque_nm <= 1e-9);
            }
            std::printf("G1 E1 %s @%.0f: sweep %zu points, %d BeyondEngineData (%d Exceeded), %zu findings\n", car.file, target, res.points.size(),
                        beyond, exceeded, res.limits.size());
            CHECK((beyond > 0) == (r.armed() && res.points.back().rpm > r.start_rpm));
            std::fflush(stdout);
        }
    }
}
