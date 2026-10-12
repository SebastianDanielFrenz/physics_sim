// tests/unit/test_assist_launch.cpp - assists plan I4 (D-LC1, vault PHYS-074): launch control as a grade-gated assist of
// the assist unit (block `launch_control` of an assist_unit electronics part, channel assist.launch_control, phases
// Idle / Armed / Launch / Handover in core/src/vehicle/launch_assist.h).
//
//   L*  loader: the shipped units, every rejection (unknown key, ranges, exactly one launch rpm form, slip_target,
//       Street), the Q5 rule (P8 two-step + unit launch control on one car), launch rpm against the engine
//   S*  the state machine white-box (arming conditions, Armed exits, release, hand-over reasons, ramp, slip PI)
//   F*  fitment/channel/snapshot on a real car (car_sedan): default, bit 4, phase, arming logic through a real World
//   V8  car_hyper + the shipped Racing unit on flat asphalt, from rest (short case; [long] adds the 0-100 km/h band)
//   P*  precedence with the I2 crawl path (D-LS1..3): TC never acts through the clutch while LC owns the car
//   D*  Idle equals no block (trajectory), 1 vs 10 workers equal hash (V11), zero own allocation per tick
//
// BANDS COMMITTED BEFORE THE FIRST RUN (assists plan B6 V8/V9/V11, brief assists_i4; all [AS]):
//   V8   on car_hyper with the shipped Racing unit (launch_rpm 5000 [AS, tuned by V8], asphalt, brake 1 + throttle 1 from rest:
//        - Armed: every tick of the last 0.5 s of a 1.5 s Armed window has engine rpm within launch_rpm +- 150
//          (ORIGINAL, SUPERSEDED - see the correction below)
//        - boost at the last Armed tick >= 0.4 bar
//        - mean rear (wheels 2, 3) slip_ratio over 0.2-1.0 s after release within slip_target +- 0.05, for TC mode
//          On (target 0.10) and Sport (target 0.14)
//        - no Stalled engine state; the phase sequence Armed -> Launch is seen
//        - the rear wing offset is within 1 deg of launch_offset_deg (10) at release (assist.aero_control on)
//        - [long] 0-100 km/h measured from release <= the TC-On WOT auto-clutch launch of the same car without LC
//          measured from the throttle application (baseline = the same unit, assist.launch_control 0), both with
//          auto-shift; the numbers are printed
//   V9   a car with a P8 two-step (launch_control electronics part) AND an assist unit that carries launch_control is
//        rejected at load with "a car has either a two-step (launch_control) or an assist-unit launch control, not
//        both", in either entry order; a Street unit with a launch_control block is rejected (I1 rule)
//   Arm  car_sedan + the Racing unit (channel on): no arming without brake (< 0.5), without throttle (< 0.9), above
//        arm_max_speed_mps (0.5 m/s), outside gear 1, without the auto-clutch assist; throttle < 0.5 in Armed -> Idle;
//        brake release -> Launch; hand-over on speed > 6 m/s, on t > max_launch_s (3), on throttle lift, on a shift
//   Idle a car with the block and the channel at 0, and one with the channel at 1 that never meets the arming
//        conditions, follow the same trajectory (state hash per tick) as the same car without the block - except
//        the block's own hashed bytes: the comparison is on the chassis pose/velocity, bit for bit
//   V11  1 vs 10 workers: equal state hash every tick of a V8 launch; zero own allocation per tick with LC active
//
// TUNING (coordinator decision, owner Q7 "[AS] data defaults, tuned by V8"): the boost band (>= 0.4 bar) is met by the DATA, not
//   by a relaxed band. At launch_rpm 4500 the boost at release was 0.288 bar (miss); the Racing unit therefore ships
//   launch_rpm 5000 (0.502 bar, V8d records both). Boost vs launch rpm (unloaded, 1.5 s): 4500 0.29 | 5000 0.50 (0.57
//   steady) | 5500 0.97 | 6000 1.4 bar.
// band corrected 2026-10-13 by the coordinator: the hold is the engine limiter's hysteresis cycle; measured max dev 150.6 below,
//   21 above, mean 74.8 below (|dev| max 150.6 p95 146.5 mean 74.8 over the window). The committed +-150 band symmetric about
//   launch_rpm described a different controller. The check is now DERIVED from the engine desc: every tick of the window in
//   [launch_rpm - h - 25, launch_rpm + 50] (h = the fuel-cut hysteresis read from the loaded desc, 150 for car_hyper [D]; the
//   25 rpm coast and 50 rpm overshoot margins are [AS]); the window mean in [launch_rpm - h, launch_rpm]. No other band changed.
// (earlier note, kept) CORRECTION of the armed-rpm band (no widening of +-150): the +-150 is measured on the last 0.5 s of the 1.5 s hold, i.e. after
//   the approach (the rpm first reaches launch_rpm - 150 within the first ~0.3 s, V8a prints t_reach). Inside the hold
//   the engine's own fuel-cut hysteresis (150 rpm on car_hyper) cycles the rpm between launch_rpm - 150 and
//   launch_rpm: V8a prints max/p95/mean of |rpm - launch_rpm| over the window and over the whole hold after t_reach.
//
// RECORDED SABOTAGES (temporary env-switched edits in launch_assist.h / vehicle_instance.cpp, reverted; each rebuilt and run
// on the shown test, Windows release, 2026-10-13):
//   1  TC not suspended while LC owns the car        -> P1: tc_via_clutch ticks 83 == 0 and the finite-cap check fail
//   2  aero launch request never set                 -> V8a: wing 0.00 deg, |10 - 0| <= 1 fails
//   3  no slip regulator (clutch follows the ramp)   -> V8a: kappa 0.4496 vs 0.10, |diff| 0.35 <= 0.05 fails
//   4  launch limit not applied while Armed          -> V8a: armed rpm dev_max 2298 > 150 (+ wing, stalled-phase checks)
//   5  launch limit kept after a hard hand-over      -> S3: limiter_rpm 4500 == 0 fails (every hard reason)
//   6  arming without the brake condition            -> S1/Arm1: Armed seen without brake (1 == 0), limiter 4500 == 0 fail
//   7  clutch not held open while Armed              -> Arm1 (car creeps, rpm 2221 > 4300 band) and V8a (armed rpm dev_max 2630, boost 0.064)
//   8  clutch fraction capped at 0.3 in Launch       -> V8c [long]: 0-100 km/h 4.842 s vs baseline 4.733 s, <= fails

#include "ps/backend/i_rigid_backend.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/electronics_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle/launch_assist.h"
#include "vehicle/vehicle_instance.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
using namespace ps::vehicle;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
const std::string kStreet = kData + "/electronics/assist_unit_street.json";
const std::string kSport = kData + "/electronics/assist_unit_sport.json";
const std::string kRacing = kData + "/electronics/assist_unit_racing.json";
const std::string kTwoStep = kData + "/electronics/launch_two_step_generic.json";
const std::string kUnitHead = R"({"format":"physics_sim.electronics/1","kind":"assist_unit",)";

// V8 bands (committed before the first run).
constexpr real kArmedCoastMarginRpm = 25.0;     // [AS] coast below launch_rpm - h before the fuel returns (corrected band)
constexpr real kArmedOvershootMarginRpm = 50.0; // [AS] overshoot above launch_rpm (corrected band)
constexpr real kBoostMinBar = 0.4;           // the brief's band (V8d)
constexpr real kKappaTol = 0.05;
constexpr real kWingTolDeg = 1.0;
constexpr real kArmedWindowS = 1.5;
constexpr real kKappaT0 = 0.2, kKappaT1 = 1.0;
constexpr real kV100 = 100.0 / 3.6;

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_assist_launch_" + tag);
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
    const std::string d = std::string(PS_SOURCE_DIR) + "/out/assists_i4_map_cache";
    fs::create_directories(d);
    return d;
}

io::VehicleLoadOptions with_unit(const std::string& path) {
    io::VehicleLoadOptions o;
    o.electronics["assists"] = path;
    return o;
}

VehicleDesc load_sedan(const io::VehicleLoadOptions& o) { return io::load_vehicle_json(kData + "/vehicles/car_sedan.json", o); }

// the racing unit with its launch_control block replaced/edited (temp file)
std::string racing_with_launch(const TempDir& td, const std::string& name, const json& lc, bool erase = false) {
    std::ifstream f(kRacing);
    json j = json::parse(f);
    if (erase) j.erase("launch_control");
    else j["launch_control"] = lc;
    write_text(td.file(name), j.dump(1));
    return td.file(name);
}

VehicleDesc load_hyper(const std::string& unit_path) {
    jobs::JobSystem pool(10);
    io::VehicleLoadOptions opt;
    opt.maps.pool = &pool;
    opt.maps.cache_dir = cache_dir();
    opt.electronics["assists"] = unit_path;
    return io::load_vehicle_json(kData + "/vehicles/car_hyper.json", opt);
}
const VehicleDesc& hyper_racing() {
    static const VehicleDesc d = load_hyper(kRacing);
    return d;
}

// The engine's own limiter as loaded (car_hyper: FuelCut 8250 rpm, hysteresis 150 rpm [D]).
drivetrain::EngineLimiterDesc engine_limiter(const VehicleDesc& d) {
    for (const drivetrain::ComponentDesc& c : d.powertrain.components) {
        if (const auto* te = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) return te->limiter;
        if (const auto* se = std::get_if<drivetrain::SimulatedEngineDesc>(&c.params)) return se->limiter;
    }
    FAIL("no engine component");
    return {};
}

std::size_t wing_index(const VehicleDesc& d) {
    for (std::size_t i = 0; i < d.aero.surfaces.size(); ++i)
        if (d.aero.surfaces[i].name == "rear_wing") return i;
    FAIL("no rear_wing surface");
    return 0;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// L: loader.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("L1: the shipped units carry launch control; Street does not", "[assist][io]") {
    const io::ElectronicsPart racing = io::load_electronics_json(kRacing);
    REQUIRE(racing.unit.launch_control.has_value());
    const LaunchAssistDesc& r = *racing.unit.launch_control;
    CHECK(r.launch_rpm == 5000.0);
    CHECK(r.launch_rpm_fraction == 0.0);
    CHECK_FALSE(r.default_enabled);
    CHECK(r.arm_throttle == 0.9);
    CHECK(r.arm_brake == 0.5);
    CHECK(r.release_brake == 0.2);
    CHECK(r.arm_max_speed_mps == 0.5);
    CHECK(r.handover_speed_mps == 6.0);
    CHECK(r.max_launch_s == 3.0);
    CHECK(r.clutch_ramp_s == 0.25);
    CHECK(r.clutch_up_per_s == 6.0);
    CHECK(r.clutch_down_per_s == 12.0);
    CHECK(r.slip_target[1] == 0.10); // On
    CHECK(r.slip_target[3] == 0.14); // Sport
    CHECK(r.slip_target[4] == 0.20); // Drift
    CHECK(r.slip_target[2] > r.slip_target[1]);
    CHECK(r.slip_target[2] < r.slip_target[3]); // Relaxed between On and Sport
    CHECK(r.slip_target[0] == r.slip_target[1]); // Off: not used, mirrors On

    const io::ElectronicsPart sport = io::load_electronics_json(kSport);
    REQUIRE(sport.unit.launch_control.has_value());
    CHECK(sport.unit.launch_control->launch_rpm == 0.0);
    CHECK(sport.unit.launch_control->launch_rpm_fraction == 0.55);
    CHECK(launch_assist_rpm(*sport.unit.launch_control, 8250.0) == Catch::Approx(0.55 * 8250.0));
    CHECK(launch_assist_rpm(*racing.unit.launch_control, 8250.0) == 5000.0);

    CHECK_FALSE(io::load_electronics_json(kStreet).unit.launch_control.has_value());
}

TEST_CASE("L2: launch_control rejections", "[assist][io]") {
    TempDir td("l2");
    const auto load = [&](const std::string& body) {
        write_text(td.file("p.json"), kUnitHead + body);
        return io::load_electronics_json(td.file("p.json"));
    };
    const auto racing = [&](const std::string& lc) { return load(R"("grade":"racing","launch_control":)" + lc + "}"); };
    // grade table (I1 rule)
    CHECK_THROWS_WITH(load(R"("grade":"street","launch_control":{"launch_rpm":4500}})"), ContainsSubstring("grade 'street' does not allow 'launch_control'"));
    CHECK(load(R"("grade":"sport","launch_control":{"launch_rpm_fraction":0.5}})").unit.launch_control.has_value());
    // shape, unknown keys, types
    CHECK_THROWS_WITH(racing("3"), ContainsSubstring("launch_control must be an object"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"bogus":1})"), ContainsSubstring("launch_control: unknown field 'bogus'"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":"4500"})"), ContainsSubstring("'launch_rpm' must be a number"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"default_enabled":2})"), ContainsSubstring("'default_enabled' must be a boolean or 0/1"));
    // exactly one launch rpm form
    CHECK_THROWS_WITH(racing("{}"), ContainsSubstring("exactly one of 'launch_rpm'"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"launch_rpm_fraction":0.5})"), ContainsSubstring("exactly one of 'launch_rpm'"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm_fraction":1.0})"), ContainsSubstring("launch_control"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":-5})"), ContainsSubstring("launch_control"));
    // ranges
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"arm_throttle":0.4})"), ContainsSubstring("arm_throttle must be in (0.5, 1]"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"arm_throttle":1.1})"), ContainsSubstring("arm_throttle must be in (0.5, 1]"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"arm_brake":0})"), ContainsSubstring("arm_brake must be in (0, 1]"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"release_brake":0.6})"), ContainsSubstring("release_brake must be in [0, arm_brake)"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"arm_max_speed_mps":0})"), ContainsSubstring("must be finite and > 0"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"max_launch_s":-1})"), ContainsSubstring("must be finite and > 0"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"clutch_ramp_s":-0.1})"), ContainsSubstring("clutch_ramp_s must be >= 0"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"handover_speed_mps":0.4})"), ContainsSubstring("handover_speed_mps must exceed arm_max_speed_mps"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"min_fraction":1})"), ContainsSubstring("min_fraction must be in [0, 1)"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"kp":0})"), ContainsSubstring("must be finite and > 0"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"clutch_up_per_s":0})"), ContainsSubstring("must be finite and > 0"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"clutch_down_per_s":-1})"), ContainsSubstring("must be finite and > 0"));
    // slip_target
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"slip_target":3})"), ContainsSubstring("'slip_target' must be an object"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"slip_target":{"turbo":0.1}})"), ContainsSubstring("slip_target: unknown mode 'turbo'"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"slip_target":{"on":0}})"), ContainsSubstring("slip_target values must be in (0, 1)"));
    CHECK_THROWS_WITH(racing(R"({"launch_rpm":4500,"slip_target":{"sport":1.0}})"), ContainsSubstring("slip_target values must be in (0, 1)"));
    // a valid block parses to what it says
    const auto ok = racing(R"({"launch_rpm":3000,"default_enabled":true,"clutch_up_per_s":5,"slip_target":{"on":0.07,"drift":0.3}})").unit.launch_control;
    REQUIRE(ok.has_value());
    CHECK(ok->launch_rpm == 3000.0);
    CHECK(ok->clutch_up_per_s == 5.0);
    CHECK(ok->clutch_down_per_s == 12.0);
    CHECK(ok->default_enabled);
    CHECK(ok->slip_target[1] == 0.07);
    CHECK(ok->slip_target[0] == 0.07);
    CHECK(ok->slip_target[4] == 0.3);
}

TEST_CASE("L3: launch rpm against the engine; Q5 - a two-step and an assist-unit launch control are mutually exclusive",
          "[assist][io][drivetrain]") {
    TempDir td("l3");
    // sedan engine: idle 800, limiter 6800
    CHECK(load_sedan(with_unit(kRacing)).assist_unit->launch_control.has_value());
    CHECK(load_sedan(with_unit(kSport)).assist_unit->launch_control.has_value()); // fraction 0.55 -> 3740 rpm
    CHECK_THROWS_WITH(load_sedan(with_unit(racing_with_launch(td, "lo.json", {{"launch_rpm", 700}}))), ContainsSubstring("must lie between the engine's idle rpm"));
    CHECK_THROWS_WITH(load_sedan(with_unit(racing_with_launch(td, "hi.json", {{"launch_rpm", 7000}}))), ContainsSubstring("must lie between the engine's idle rpm"));
    CHECK_THROWS_WITH(load_sedan(with_unit(racing_with_launch(td, "fr.json", {{"launch_rpm_fraction", 0.05}}))),
                      ContainsSubstring("must lie between the engine's idle rpm"));

    // V9 / Q5: both variants, either entry order
    const char* msg = "a car has either a two-step (launch_control) or an assist-unit launch control, not both";
    io::VehicleLoadOptions a;
    a.electronics["a_two_step"] = kTwoStep;
    a.electronics["z_assists"] = kRacing;
    CHECK_THROWS_WITH(load_sedan(a), ContainsSubstring(msg));
    io::VehicleLoadOptions b;
    b.electronics["z_two_step"] = kTwoStep;
    b.electronics["a_assists"] = kRacing;
    CHECK_THROWS_WITH(load_sedan(b), ContainsSubstring(msg));
    // each alone loads; a unit WITHOUT the block next to the two-step loads (Street)
    io::VehicleLoadOptions only_p8;
    only_p8.electronics["p8"] = kTwoStep;
    CHECK_NOTHROW(load_sedan(only_p8));
    io::VehicleLoadOptions street_p8;
    street_p8.electronics["p8"] = kTwoStep;
    street_p8.electronics["assists"] = kStreet;
    CHECK_NOTHROW(load_sedan(street_p8));
    // the racing unit without its launch_control block next to the two-step loads too
    io::VehicleLoadOptions no_lc;
    no_lc.electronics["p8"] = kTwoStep;
    no_lc.electronics["assists"] = racing_with_launch(td, "nolc.json", json::object(), true);
    CHECK_NOTHROW(load_sedan(no_lc));
    // the V9 second half: a Street unit with a launch_control block is rejected (I1 rule), also through the vehicle loader
    json street = json::parse(std::ifstream(kStreet));
    street["launch_control"] = {{"launch_rpm", 4500}};
    write_text(td.file("street_lc.json"), street.dump(1));
    CHECK_THROWS_WITH(load_sedan(with_unit(td.file("street_lc.json"))), ContainsSubstring("grade 'street' does not allow 'launch_control'"));

    // the VehicleInstance backstop: a code-built desc with both variants is rejected at create_vehicle
    VehicleDesc d = load_sedan(with_unit(kRacing));
    const VehicleDesc p8 = load_sedan(only_p8);
    d.controllers = p8.controllers; // carries the P8 part (+ the sedan's own manual_tcu)
    auto rig = ps_test::build_vehicle_rig_from_desc(load_sedan(with_unit(kRacing)), 1500, {2, .4, .15}, .5, 0, 1);
    BodyDesc bd;
    bd.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    bd.motion = BodyMotionType::Dynamic;
    bd.mass = 1500;
    bd.pose.position = Vec3{50, 50, 1};
    const BodyId ch = rig.world->create_body(bd);
    CHECK_THROWS_WITH(rig.world->create_vehicle(d, ch), ContainsSubstring("not both"));
}

// ---------------------------------------------------------------------------------------------------------------
// S: the state machine, white-box.
// ---------------------------------------------------------------------------------------------------------------
namespace {

LaunchAssistDesc default_desc() {
    LaunchAssistDesc d;
    d.launch_rpm = 4500;
    return d;
}

LaunchAssist::Inputs armed_inputs() {
    LaunchAssist::Inputs in;
    in.enabled = true;
    in.auto_clutch = true;
    in.engine_running = true;
    in.throttle = 1.0;
    in.brake = 1.0;
    in.speed_mps = 0.0;
    in.gear = 1;
    in.shifting = false;
    in.slip = 0;
    in.slip_valid = true;
    in.slip_target = 0.10;
    in.dt = 1.0 / 960.0;
    in.first_substep_of_tick = true;
    return in;
}

LaunchAssist::Inputs armed_inputs_released() {
    auto in = armed_inputs();
    in.brake = 0.0;
    return in;
}

LaunchAssist armed() {
    LaunchAssist a(default_desc(), 4500.0);
    const auto o = a.step(armed_inputs());
    REQUIRE(o.phase == LaunchPhase::Armed);
    return a;
}

LaunchAssist launching() {
    LaunchAssist a = armed();
    auto in = armed_inputs();
    in.brake = 0.0;
    const auto o = a.step(in);
    REQUIRE(o.phase == LaunchPhase::Launch);
    return a;
}

} // namespace

TEST_CASE("S1: arming needs every condition", "[assist][vehicle]") {
    const auto expect_idle = [](const char* what, auto&& mutate) {
        INFO(what);
        LaunchAssist a(default_desc(), 4500.0);
        auto in = armed_inputs();
        mutate(in);
        const auto o = a.step(in);
        CHECK(o.phase == LaunchPhase::Idle);
        CHECK_FALSE(o.owns);
        CHECK(o.limiter_rpm == 0.0);
        CHECK_FALSE(o.hold_clutch);
    };
    expect_idle("channel off", [](auto& in) { in.enabled = false; });
    expect_idle("no auto-clutch", [](auto& in) { in.auto_clutch = false; });
    expect_idle("engine not running", [](auto& in) { in.engine_running = false; });
    expect_idle("throttle at the threshold (0.9, strict)", [](auto& in) { in.throttle = 0.9; });
    expect_idle("throttle below", [](auto& in) { in.throttle = 0.5; });
    expect_idle("brake at the threshold (0.5, strict)", [](auto& in) { in.brake = 0.5; });
    expect_idle("no brake", [](auto& in) { in.brake = 0.0; });
    expect_idle("speed at 0.5", [](auto& in) { in.speed_mps = 0.5; });
    expect_idle("speed -2 (reverse)", [](auto& in) { in.speed_mps = -2.0; });
    expect_idle("neutral", [](auto& in) { in.gear = 0; });
    expect_idle("gear 2", [](auto& in) { in.gear = 2; });
    expect_idle("shifting", [](auto& in) { in.shifting = true; });
    // and the positive control
    LaunchAssist a(default_desc(), 4500.0);
    const auto o = a.step(armed_inputs());
    CHECK(o.phase == LaunchPhase::Armed);
    CHECK(o.owns);
    CHECK(o.limiter_rpm == 4500.0);
    CHECK(o.hold_clutch);
    CHECK(o.clutch_ramp < 0.0);
    CHECK(o.ceiling == 1.0);
}

TEST_CASE("S2: Armed exits", "[assist][vehicle]") {
    const auto expect_idle = [](const char* what, auto&& mutate) {
        INFO(what);
        LaunchAssist a = armed();
        auto in = armed_inputs();
        mutate(in);
        CHECK(a.step(in).phase == LaunchPhase::Idle);
    };
    expect_idle("throttle lifted below 0.5", [](auto& in) { in.throttle = 0.49; });
    expect_idle("channel off", [](auto& in) { in.enabled = false; });
    expect_idle("auto-clutch off", [](auto& in) { in.auto_clutch = false; });
    expect_idle("engine stopped", [](auto& in) { in.engine_running = false; });
    expect_idle("moving", [](auto& in) { in.speed_mps = 0.6; });
    expect_idle("gear changed", [](auto& in) { in.gear = 2; });
    expect_idle("shift requested", [](auto& in) { in.shifting = true; });
    // throttle between 0.5 and 0.9 does NOT drop Armed (hysteresis between arm and disarm)
    LaunchAssist a = armed();
    auto in = armed_inputs();
    in.throttle = 0.7;
    CHECK(a.step(in).phase == LaunchPhase::Armed);
    // brake between release_brake (0.2) and arm_brake (0.5) keeps Armed
    in.brake = 0.3;
    CHECK(a.step(in).phase == LaunchPhase::Armed);
    // below release_brake -> Launch
    in.brake = 0.19;
    const auto o = a.step(in);
    CHECK(o.phase == LaunchPhase::Launch);
    CHECK(o.limiter_rpm == 4500.0); // the launch limit stays during the slip phase (lifted when the soft hand-over ends)
    CHECK_FALSE(o.hold_clutch);
    CHECK(o.owns);
    CHECK(o.clutch_ramp >= 0.0);
}

TEST_CASE("S3: Launch - clutch fraction, hand-over reasons (hard and soft)", "[assist][vehicle]") {
    // below the target the fraction grows at the ramp rate 1/clutch_ramp_s = 4/s: 0 -> 1 at 240 substeps of 1/960 s
    {
        LaunchAssist a = launching();
        auto in = armed_inputs();
        in.brake = 0;
        real prev = a.clutch_fraction();
        CHECK(prev == 0.0);
        int n_to_one = 0;
        for (int i = 1; i <= 400; ++i) {
            const auto o = a.step(in);
            REQUIRE(o.phase == LaunchPhase::Launch);
            CHECK(o.clutch_ramp >= prev - 1e-15);
            CHECK(o.clutch_ramp - prev <= 4.0 / 960.0 + 1e-12);
            CHECK(o.limiter_rpm == 4500.0); // the launch limit stays during the slip phase
            prev = o.clutch_ramp;
            if (o.clutch_ramp >= 1.0 && n_to_one == 0) n_to_one = i;
        }
        CHECK(n_to_one >= 238);
        CHECK(n_to_one <= 242);
    }
    // at the slip target the fraction holds, above it falls (12/s * (slip/target - 1)), below it grows again
    {
        LaunchAssist a = launching();
        auto in = armed_inputs();
        in.brake = 0;
        in.slip = 0.0;
        for (int i = 0; i < 120; ++i) a.step(in); // 0.125 s -> ~0.5
        const real c0 = a.clutch_fraction();
        CHECK(c0 == Catch::Approx(0.5).margin(0.01));
        in.slip = in.slip_target; // exactly at the target: rate 0
        for (int i = 0; i < 100; ++i) a.step(in);
        CHECK(a.clutch_fraction() == Catch::Approx(c0).margin(1e-12));
        in.slip = 2.0 * in.slip_target; // rel = 1: -12/s
        for (int i = 0; i < 96; ++i) a.step(in); // 0.1 s -> -1.2 (clamped at 0)
        CHECK(a.clutch_fraction() == 0.0);
        in.slip = 1.5 * in.slip_target; // rel = 0.5 -> -6/s
        LaunchAssist b = launching();
        for (int i = 0; i < 240; ++i) b.step(armed_inputs_released());
        const real cb = b.clutch_fraction();
        for (int i = 0; i < 96; ++i) b.step([&] { auto x = armed_inputs_released(); x.slip = 1.5 * x.slip_target; return x; }());
        CHECK(b.clutch_fraction() == Catch::Approx(cb - 0.6).margin(0.01));
        // an invalid estimate: the fraction just follows the ramp
        LaunchAssist c = launching();
        auto inv = armed_inputs_released();
        inv.slip_valid = false;
        inv.slip = 0.9;
        for (int i = 0; i < 240; ++i) c.step(inv);
        CHECK(c.clutch_fraction() == Catch::Approx(1.0).margin(1e-9));
    }
    // HARD hand-over: nothing is kept, Idle at the next tick's first substep
    const auto expect_hard = [](const char* what, auto&& mutate) {
        INFO(what);
        LaunchAssist a = launching();
        auto in = armed_inputs_released();
        mutate(in);
        const auto o = a.step(in);
        CHECK(o.phase == LaunchPhase::Handover);
        CHECK(o.handover);
        CHECK_FALSE(o.owns);
        CHECK_FALSE(o.overlay);
        CHECK(o.clutch_ramp < 0.0);
        CHECK(o.limiter_rpm == 0.0);
        CHECK(o.handover_ceiling == Catch::Approx(1.0)); // nothing happened yet
        auto in2 = in;
        in2.first_substep_of_tick = false;
        CHECK(a.step(in2).phase == LaunchPhase::Handover); // not before the next tick's first substep
        in2.first_substep_of_tick = true;
        in2.throttle = 0.0; // would not re-arm
        CHECK(a.step(in2).phase == LaunchPhase::Idle);
    };
    expect_hard("throttle lift", [](auto& in) { in.throttle = 0.49; });
    expect_hard("shift", [](auto& in) { in.shifting = true; });
    expect_hard("gear != 1", [](auto& in) { in.gear = 2; });
    expect_hard("channel off", [](auto& in) { in.enabled = false; });
    expect_hard("engine stopped", [](auto& in) { in.engine_running = false; });
    // SOFT hand-over (speed > 6 m/s, or t > 3 s): TC is back (owns false) but the launch limit and the clutch fraction
    // stay in force until the fraction has climbed to 1 at the ramp rate, then Idle at the next tick's first substep
    const auto expect_soft = [](const char* what, auto&& mutate) {
        INFO(what);
        LaunchAssist a = launching();
        auto in = armed_inputs_released();
        for (int i = 0; i < 48; ++i) a.step(in); // 0.05 s: the fraction is ~0.2 (below the target the ramp applies)
        const real c_before = a.clutch_fraction();
        REQUIRE(c_before > 0.1);
        REQUIRE(c_before < 0.3);
        mutate(in);
        const auto o = a.step(in);
        CHECK(o.phase == LaunchPhase::Handover);
        CHECK(o.handover);
        CHECK_FALSE(o.owns);
        CHECK(o.overlay);
        CHECK(o.limiter_rpm == 4500.0);
        CHECK(o.clutch_ramp >= c_before);
        CHECK(o.ceiling == 1.0);
        int n = 0;
        auto in2 = in;
        in2.first_substep_of_tick = false;
        real prev = o.clutch_ramp;
        while (n < 2000) {
            const auto o2 = a.step(in2);
            if (!o2.overlay) break;
            CHECK(o2.phase == LaunchPhase::Handover);
            CHECK(o2.limiter_rpm == 4500.0);
            CHECK(o2.clutch_ramp >= prev);
            prev = o2.clutch_ramp;
            ++n;
        }
        // (1 - c) / 4 per s at 960 Hz
        CHECK(n >= static_cast<int>(960.0 * (1.0 - c_before) / 4.0) - 3);
        CHECK(n <= static_cast<int>(960.0 * (1.0 - c_before) / 4.0) + 3);
        CHECK(a.step(in2).phase == LaunchPhase::Handover); // still the same tick
        in2.first_substep_of_tick = true;
        in2.throttle = 0.0;
        const auto o3 = a.step(in2);
        CHECK(o3.phase == LaunchPhase::Idle);
        CHECK_FALSE(o3.overlay);
        CHECK(o3.limiter_rpm == 0.0);
        CHECK(o3.clutch_ramp < 0.0);
    };
    expect_soft("speed above 6 m/s", [](auto& in) { in.speed_mps = 6.01; });
    // speed exactly at 6 stays; the time limit is a soft hand-over too
    {
        LaunchAssist a = launching();
        auto in = armed_inputs_released();
        in.speed_mps = 6.0;
        CHECK(a.step(in).phase == LaunchPhase::Launch);
        in.speed_mps = 1.0;
        // t > 3 s -> Handover at ~2880 substeps of 1/960
        int n = 0;
        LaunchAssist::Outputs o;
        o.phase = LaunchPhase::Launch;
        while (o.phase == LaunchPhase::Launch && n < 5000) {
            o = a.step(in);
            ++n;
        }
        CHECK(o.phase == LaunchPhase::Handover);
        CHECK(o.overlay == false); // the clutch fraction was already 1 (slip below the target for 3 s)
        CHECK(n >= 2878);
        CHECK(n <= 2884);
    }
}

TEST_CASE("S4: the slip PI - ceiling falls on overshoot, recovers, bounded by min_fraction", "[assist][vehicle]") {
    LaunchAssist a = launching();
    auto in = armed_inputs();
    in.brake = 0;
    in.speed_mps = 1.0;
    // slip at the target: the ceiling stays 1
    in.slip = 0.10;
    real c = 1;
    for (int i = 0; i < 100; ++i) c = a.step(in).ceiling;
    CHECK(c >= 0.99);
    // large overshoot: the ceiling attacks at <= 20/s, reaches the floor
    in.slip = 0.6;
    real prev = c;
    for (int i = 0; i < 960; ++i) {
        const real now = a.step(in).ceiling;
        CHECK(now <= prev + 1e-15);
        CHECK(prev - now <= 20.0 / 960.0 + 1e-12); // attack rate
        prev = now;
    }
    CHECK(prev == Catch::Approx(0.05).margin(1e-9)); // min_fraction
    // slip below target: recovers at <= 8/s
    in.slip = 0.0;
    prev = a.ceiling();
    real after_half_s = prev;
    for (int i = 0; i < 960; ++i) {
        const real now = a.step(in).ceiling;
        CHECK(now >= prev - 1e-15);
        CHECK(now - prev <= 8.0 / 960.0 + 1e-12);
        prev = now;
        if (i == 479) after_half_s = now;
    }
    CHECK(after_half_s > 0.05);
    CHECK(prev <= 1.0);
    // an invalid TC estimate leaves the ceiling where it is
    LaunchAssist b = launching();
    auto in2 = armed_inputs();
    in2.brake = 0;
    in2.speed_mps = 1.0;
    in2.slip = 0.9;
    in2.slip_valid = false;
    for (int i = 0; i < 200; ++i) CHECK(b.step(in2).ceiling == 1.0);
    // the hand-over carries the last ceiling
    LaunchAssist h = launching();
    auto in3 = armed_inputs();
    in3.brake = 0;
    in3.speed_mps = 1.0;
    in3.slip = 0.5;
    for (int i = 0; i < 100; ++i) h.step(in3);
    const real last = h.ceiling();
    CHECK(last < 0.9);
    in3.speed_mps = 7.0;
    const auto o = h.step(in3);
    REQUIRE(o.handover);
    CHECK(o.handover_ceiling == last);
}

TEST_CASE("S5: reset and hash", "[assist][vehicle]") {
    LaunchAssist a = launching();
    LaunchAssist fresh(default_desc(), 4500.0);
    CHECK(a.fold_state_hash(14695981039346656037ULL) != fresh.fold_state_hash(14695981039346656037ULL));
    a.reset();
    CHECK(a.phase() == LaunchPhase::Idle);
    CHECK(a.fold_state_hash(14695981039346656037ULL) == fresh.fold_state_hash(14695981039346656037ULL));
    LaunchAssist other(default_desc(), 4400.0); // the resolved launch rpm is part of the hashed identity
    CHECK(other.fold_state_hash(1) != fresh.fold_state_hash(1));
}

// ---------------------------------------------------------------------------------------------------------------
// F: a real car (car_sedan + the Racing unit): channel, fitment bits, snapshot, arming logic through a World.
// ---------------------------------------------------------------------------------------------------------------
namespace {

struct SedanRig {
    ps_test::VehicleTestRig rig;
    World& w() { return *rig.world; }
};

SedanRig make_sedan(const VehicleDesc& d, int workers = 1) {
    SedanRig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, workers);
    return r;
}

void sedan_controls(World& w, bool lc_on) {
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("assist.launch_control", lc_on ? 1.0 : 0.0);
    w.set_control("gear_request", 1.0);
}

} // namespace

TEST_CASE("F1: channel, default, fitted/active bit 4, snapshot", "[assist][vehicle][drivetrain]") {
    TempDir td("f1");
    // channel registered with the unit's default (off)
    {
        SedanRig s = make_sedan(load_sedan(with_unit(kRacing)));
        CHECK(s.w().get_control("assist.launch_control") == 0.0);
        s.w().step();
        const auto& pt = s.w().powertrain_state(s.rig.vehicle);
        CHECK((pt.assist_fitted_bits & 16u) != 0);
        CHECK((pt.assist_active_bits & 16u) == 0);
        CHECK(pt.launch_phase == 0);
        s.w().set_control("assist.launch_control", 1.0);
        s.w().step();
        CHECK((s.w().powertrain_state(s.rig.vehicle).assist_active_bits & 16u) != 0);
    }
    // default_enabled true -> channel default 1
    {
        const std::string p = racing_with_launch(td, "on.json", {{"launch_rpm", 4500}, {"default_enabled", true}});
        SedanRig s = make_sedan(load_sedan(with_unit(p)));
        CHECK(s.w().get_control("assist.launch_control") == 1.0);
    }
    // a car without any launch control: no bit, no channel registered by the car (get_control of an unknown name is 0)
    {
        SedanRig s = make_sedan(load_sedan(io::VehicleLoadOptions{}));
        s.w().step();
        const auto& pt = s.w().powertrain_state(s.rig.vehicle);
        CHECK((pt.assist_fitted_bits & 16u) == 0);
        CHECK(pt.launch_phase == 0);
        CHECK(pt.launch_slip_target == 0.0);
    }
    // the Street unit: no launch control
    {
        SedanRig s = make_sedan(load_sedan(with_unit(kStreet)));
        s.w().step();
        CHECK((s.w().powertrain_state(s.rig.vehicle).assist_fitted_bits & 16u) == 0);
    }
    // P8 semantics unchanged: the two-step alone still has the bit and channel default 0
    {
        io::VehicleLoadOptions p8;
        p8.electronics["p8"] = kTwoStep;
        SedanRig s = make_sedan(load_sedan(p8));
        s.w().step();
        const auto& pt = s.w().powertrain_state(s.rig.vehicle);
        CHECK((pt.assist_fitted_bits & 16u) != 0);
        CHECK(pt.launch_phase == 0);
        CHECK(s.w().get_control("assist.launch_control") == 0.0);
    }
}

TEST_CASE("Arm1: arming logic through a real World (car_sedan, Racing unit)", "[assist][vehicle][drivetrain]") {
    const VehicleDesc d = load_sedan(with_unit(kRacing));
    const auto phase = [](SedanRig& s) { return s.w().powertrain_state(s.rig.vehicle).launch_phase; };
    // 1 s settle in 1st with the handbrake, auto-clutch on
    const auto start = [&](bool lc_on) {
        SedanRig s = make_sedan(d);
        sedan_controls(s.w(), lc_on);
        s.w().set_control("handbrake", 1.0);
        ps_test::settle(s.w(), 1.0);
        s.w().set_control("handbrake", 0.0);
        return s;
    };
    const auto run = [&](SedanRig& s, int ticks) {
        for (int i = 0; i < ticks; ++i) s.w().step();
    };
    SECTION("positive control: brake + throttle -> Armed, rpm held at the launch limit, clutch open, car still") {
        SedanRig s = start(true);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 240);
        CHECK(phase(s) == 1);
        const auto& pt = s.w().powertrain_state(s.rig.vehicle);
        CHECK(pt.launch_control_active);
        CHECK(pt.launch_slip_target == Catch::Approx(0.10)); // mode On is the default
        const real lrpm = launch_assist_rpm(*d.assist_unit->launch_control, engine_limiter(d).rpm);
        const real h = engine_limiter(d).hysteresis_rpm; // the engine limiter's cycle [lrpm - h, lrpm] (same derived band as V8)
        CHECK(pt.engine_rpm >= lrpm - h - kArmedCoastMarginRpm);
        CHECK(pt.engine_rpm <= lrpm + kArmedOvershootMarginRpm);
        CHECK(pt.clutch_engagement < 0.05);
        CHECK(std::abs(s.w().get_motion(s.rig.chassis).linear.x) < 0.2);
    }
    SECTION("channel off: never arms") {
        SedanRig s = start(false);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 240);
        CHECK(phase(s) == 0);
    }
    SECTION("no brake / light brake: never arms") {
        SedanRig s = start(true);
        s.w().set_control("throttle", 1.0);
        s.w().set_control("brake", 0.0);
        run(s, 120);
        CHECK(phase(s) == 0);
        s.w().set_control("brake", 0.45);
        run(s, 120);
        CHECK(phase(s) == 0);
    }
    SECTION("no / light throttle: never arms") {
        SedanRig s = start(true);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 0.0);
        run(s, 120);
        CHECK(phase(s) == 0);
        s.w().set_control("throttle", 0.85);
        run(s, 120);
        CHECK(phase(s) == 0);
    }
    SECTION("no auto-clutch: never arms") {
        SedanRig s = start(true);
        s.w().set_control("assist.auto_clutch", 0.0);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 120);
        CHECK(phase(s) == 0);
    }
    SECTION("not in gear 1: never arms") {
        SedanRig s = start(true);
        s.w().set_control("gear_request", 2.0);
        run(s, 120); // let the gearbox get to 2nd
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 120);
        CHECK(phase(s) == 0);
        CHECK(s.w().powertrain_state(s.rig.vehicle).gear == 2);
    }
    SECTION("moving above 0.5 m/s: never arms") {
        SedanRig s = start(true);
        Motion mo;
        mo.linear = Vec3{2.0, 0, 0};
        s.w().backend().set_motion(s.rig.chassis, mo);
        VehicleResetOptions ro;
        ro.hubs_roll_with_chassis = true;
        ro.clutch_locked = false;
        s.w().reset_vehicle(s.rig.vehicle, ro);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        s.w().step();
        CHECK(phase(s) == 0);
    }
    SECTION("Armed -> throttle lift -> Idle; Armed -> brake release -> Launch -> Handover on throttle lift") {
        SedanRig s = start(true);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 240);
        REQUIRE(phase(s) == 1);
        s.w().set_control("throttle", 0.4);
        run(s, 2);
        CHECK(phase(s) == 0);
        // re-arm, release, lift
        s.w().set_control("throttle", 1.0);
        run(s, 120);
        REQUIRE(phase(s) == 1);
        s.w().set_control("brake", 0.0);
        run(s, 2);
        CHECK(phase(s) == 2);
        run(s, 60);
        CHECK(phase(s) == 2);
        s.w().set_control("throttle", 0.0);
        run(s, 2);
        const auto p = phase(s);
        CHECK((p == 3 || p == 0));
        run(s, 4);
        CHECK(phase(s) == 0);
    }
    SECTION("Launch -> Handover on a shift request") {
        SedanRig s = start(true);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 240);
        s.w().set_control("brake", 0.0);
        run(s, 30);
        REQUIRE(phase(s) == 2);
        s.w().set_control("gear_request", 2.0);
        run(s, 3);
        const auto p = phase(s);
        CHECK((p == 3 || p == 0));
    }
    SECTION("Launch -> Handover on speed > 6 m/s or at the latest after max_launch_s") {
        SedanRig s = start(true);
        s.w().set_control("brake", 1.0);
        s.w().set_control("throttle", 1.0);
        run(s, 240);
        s.w().set_control("brake", 0.0);
        real t_handover = -1;
        for (int i = 0; i < 4 * 240 && t_handover < 0; ++i) {
            s.w().step();
            if (phase(s) != 2 && phase(s) != 0) t_handover = (i + 1) / 240.0;
            if (phase(s) == 0 && i > 5) t_handover = (i + 1) / 240.0;
        }
        INFO("hand-over at " << t_handover << " s, speed " << s.w().get_motion(s.rig.chassis).linear.x);
        CHECK(t_handover > 0.0);
        CHECK(t_handover <= 3.05);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// V8: car_hyper + the shipped Racing unit on flat asphalt.
// ---------------------------------------------------------------------------------------------------------------
namespace {

struct LaunchRig {
    ps_test::VehicleTestRig rig;
    VehicleDesc desc;
    std::size_t wing = 0;
};

LaunchRig make_hyper_rig(const VehicleDesc& desc, int workers) {
    LaunchRig lr;
    lr.desc = desc;
    lr.wing = wing_index(desc);
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = workers;
    lr.rig.world = std::make_unique<World>(cfg);
    lr.rig.surfaces = std::make_shared<io::SurfaceTable>(kData + "/surfaces/surfaces.json");
    const SurfaceId sid = lr.rig.surfaces->id_for("asphalt");
    BodyDesc ground;
    ground.shape = ps_test::flat_surface_mesh(300.0, sid);
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    lr.rig.world->create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.5};
    lr.rig.chassis = lr.rig.world->create_body(chassis);
    lr.rig.world->set_surface_table(lr.rig.surfaces);
    lr.rig.vehicle = lr.rig.world->create_vehicle(desc, lr.rig.chassis);
    return lr;
}

struct LaunchRun {
    bool armed_seen = false, launch_seen = false, stalled = false;
    real armed_rpm_dev_max = 0, armed_rpm_mean = 0;
    real armed_rpm_min = 0, armed_rpm_max = 0;         // window extremes of the engine rpm
    real launch_rpm = 0, limiter_hysteresis = 0;       // the unit's launch rpm and the engine's fuel-cut hysteresis (desc)
    real armed_rpm_dev_p95 = 0, armed_rpm_dev_mean = 0; // over the committed window (last 0.5 s of the hold)
    real armed_rpm_t_reach = -1;                       // first time the rpm reaches launch_rpm - 150, from arming
    real armed_hold_dev_max = 0, armed_hold_dev_p95 = 0, armed_hold_dev_mean = 0; // from t_reach to the end of the hold
    real armed_rpm_over_max = 0, armed_rpm_under_max = 0; // signed extremes over the hold (rpm - launch_rpm)
    real boost_at_release = -1;
    real wing_deg_at_release = -1;
    real kappa_mean = 0, kappa_max = 0, kappa_min = 0, t_handover = -1, v_at_1s = 0, v_at_end = 0;
    real t_to_100 = -1; // from release (LC) or from throttle application (baseline)
    real min_ceiling_fraction = 1;
    real max_rpm_launch = 0;
    int tc_clutch_ticks_in_lc = 0;
    bool cap_finite_in_lc = false;
    std::uint64_t own_alloc = 0;
    std::vector<std::uint64_t> hashes;
};

struct LaunchOpts {
    int mode = 1;          // assist.traction_control
    bool lc = true;        // assist.launch_control
    real after_release_s = 1.2;
    bool to_100 = false;   // keep going until 100 km/h (or 14 s)
    int workers = 4;
    bool alloc = false;
    bool keep_hashes = false;
    bool print = false;
    real armed_s = kArmedWindowS;
    int print_every = 24;
    bool print_armed = false;
    std::optional<LaunchAssistDesc> lc_override; // diag/tuning only
};

LaunchRun run_launch(const VehicleDesc& desc_in, const LaunchOpts& o) {
    VehicleDesc desc = desc_in;
    if (o.lc_override) desc.assist_unit->launch_control = *o.lc_override;
    LaunchRig lr = make_hyper_rig(desc, o.workers);
    World& w = *lr.rig.world;
    const VehicleId v = lr.rig.vehicle;
    w.set_control("assist.traction_control", static_cast<real>(o.mode));
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("assist.aero_control", 1.0);
    w.set_control("assist.launch_control", o.lc ? 1.0 : 0.0);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 1.5);
    w.set_control("handbrake", 0.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("gear_request", drivetrain::kGearRequestNone);

    LaunchRun r;
    namespace da = jobs::debug_alloc;
    const bool use_alloc = o.alloc && da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    const real max_off = lr.desc.aero.surfaces[lr.wing].max_offset_rad;
    (void)max_off;
    // Phase 1 (LC): brake + throttle, hold kArmedWindowS. Baseline: throttle only, no brake.
    w.set_control("throttle", 1.0);
    w.set_control("brake", o.lc ? 1.0 : 0.0);
    const int n_arm = o.lc ? static_cast<int>(o.armed_s * 240.0 + 0.5) : 0;
    real rpm_sum = 0;
    int rpm_n = 0;
    const drivetrain::EngineLimiterDesc lim = engine_limiter(desc);
    const real lrpm = o.lc ? launch_assist_rpm(*desc.assist_unit->launch_control, lim.rpm) : 0.0;
    r.launch_rpm = lrpm;
    r.limiter_hysteresis = lim.hysteresis_rpm;
    r.armed_rpm_min = 1e30;
    std::vector<real> win_dev, hold_dev;
    for (int k = 0; k < n_arm; ++k) {
        w.step();
        const auto& pt = w.powertrain_state(v);
        if (pt.launch_phase == 1) r.armed_seen = true;
        if (k >= n_arm - 120) {
            const real dev = std::abs(pt.engine_rpm - lrpm);
            r.armed_rpm_dev_max = std::max(r.armed_rpm_dev_max, dev);
            win_dev.push_back(dev);
            r.armed_rpm_min = std::min(r.armed_rpm_min, pt.engine_rpm);
            r.armed_rpm_max = std::max(r.armed_rpm_max, pt.engine_rpm);
            rpm_sum += pt.engine_rpm;
            ++rpm_n;
        }
        if (o.lc && r.armed_rpm_t_reach < 0 && pt.engine_rpm >= lrpm - 150.0) r.armed_rpm_t_reach = (k + 1) / 240.0;
        if (o.lc && r.armed_rpm_t_reach >= 0) {
            hold_dev.push_back(std::abs(pt.engine_rpm - lrpm));
            r.armed_rpm_over_max = std::max(r.armed_rpm_over_max, pt.engine_rpm - lrpm);
            r.armed_rpm_under_max = std::max(r.armed_rpm_under_max, lrpm - pt.engine_rpm);
        }
        if (pt.engine_state == drivetrain::EngineState::Stalled) r.stalled = true;
        if (o.keep_hashes) r.hashes.push_back(w.state_hash());
        if (o.print_armed && k % 24 == 23)
            std::printf("  armed t %5.2f ph %d rpm %5.0f boost %6.3f thr_eff %5.3f wing %5.2f deg\n", (k + 1) / 240.0, int(pt.launch_phase), pt.engine_rpm,
                        pt.engines.empty() ? 0.0 : pt.engines[0].boost_bar, w.powertrain_debug(v).throttle_eff,
                        w.vehicle_aero_telemetry(v).surfaces[lr.wing].offset_rad * 180.0 / 3.141592653589793);
    }
    if (o.lc) {
        r.armed_rpm_mean = rpm_n ? rpm_sum / rpm_n : 0;
        const auto stats = [](std::vector<real> v, real& mx, real& p95, real& mean) {
            if (v.empty()) return;
            std::sort(v.begin(), v.end());
            mx = v.back();
            p95 = v[static_cast<std::size_t>(0.95 * static_cast<real>(v.size() - 1))];
            real sum = 0;
            for (real x : v) sum += x;
            mean = sum / static_cast<real>(v.size());
        };
        real dummy = 0;
        stats(win_dev, dummy, r.armed_rpm_dev_p95, r.armed_rpm_dev_mean);
        stats(hold_dev, r.armed_hold_dev_max, r.armed_hold_dev_p95, r.armed_hold_dev_mean);
        const auto& pt = w.powertrain_state(v);
        r.boost_at_release = pt.engines.empty() ? 0.0 : pt.engines[0].boost_bar;
        r.wing_deg_at_release = w.vehicle_aero_telemetry(v).surfaces[lr.wing].offset_rad * 180.0 / 3.141592653589793;
    }
    // Phase 2: release.
    w.set_control("brake", 0.0);
    const int n_after = static_cast<int>((o.to_100 ? 14.0 : o.after_release_s) * 240.0 + 0.5);
    real kappa_sum = 0;
    int kappa_n = 0;
    for (int k = 0; k < n_after; ++k) {
        w.step();
        const real t = (k + 1) / 240.0;
        const auto& pt = w.powertrain_state(v);
        const auto dbg = w.powertrain_debug(v);
        if (pt.launch_phase == 2) {
            r.launch_seen = true;
            r.max_rpm_launch = std::max(r.max_rpm_launch, pt.engine_rpm);
            if (dbg.tc_via_clutch) ++r.tc_clutch_ticks_in_lc;
            if (std::isfinite(dbg.clutch_capacity_cap_tc)) r.cap_finite_in_lc = true;
            r.min_ceiling_fraction = std::min(r.min_ceiling_fraction, dbg.throttle_eff);
        }
        if (pt.engine_state == drivetrain::EngineState::Stalled) r.stalled = true;
        if (t > kKappaT0 && t <= kKappaT1) {
            const real kk = 0.5 * (w.wheel_telemetry(v, 2).slip_ratio + w.wheel_telemetry(v, 3).slip_ratio);
            kappa_sum += kk;
            ++kappa_n;
            r.kappa_max = std::max(r.kappa_max, kk);
            r.kappa_min = std::min(r.kappa_min, kk);
        }
        if (r.t_handover < 0 && r.launch_seen && pt.launch_phase != 2) r.t_handover = t;
        if (k + 1 == 240) r.v_at_1s = w.get_motion(lr.rig.chassis).linear.x;
        r.v_at_end = w.get_motion(lr.rig.chassis).linear.x;
        if (o.keep_hashes) r.hashes.push_back(w.state_hash());
        const real vx = w.get_motion(lr.rig.chassis).linear.x;
        if (o.print && k % o.print_every == o.print_every - 1)
            std::printf("  t %5.2f v %6.2f ph %d g %d rpm %5.0f e %5.3f Tcl %6.0f cap %7.0f thr_eff %5.3f boost %6.3f kR %+6.3f\n", t, vx,
                        int(pt.launch_phase), pt.gear, pt.engine_rpm, pt.clutch_engagement, dbg.clutch_torque,
                        std::min<real>(dbg.clutch_cap_kinetic, 99999.0), dbg.throttle_eff, pt.engines.empty() ? 0.0 : pt.engines[0].boost_bar,
                        0.5 * (w.wheel_telemetry(v, 2).slip_ratio + w.wheel_telemetry(v, 3).slip_ratio));
        if (o.to_100 && r.t_to_100 < 0 && vx >= kV100) {
            r.t_to_100 = t;
            break;
        }
    }
    r.kappa_mean = kappa_n ? kappa_sum / kappa_n : 0;
    if (use_alloc) {
        da::set_armed(false);
        r.own_alloc = da::own_violation_count();
    }
    return r;
}

real slip_target_of(int mode) {
    const LaunchAssistDesc& d = *hyper_racing().assist_unit->launch_control;
    return d.slip_target[static_cast<std::size_t>(mode)];
}

// Armed-rpm band, DERIVED from the engine desc (correction below, header comment): every tick of the window in
// [launch_rpm - h - kArmedCoastMarginRpm, launch_rpm + kArmedOvershootMarginRpm], the window mean in [launch_rpm - h, launch_rpm].
void check_armed_rpm(const LaunchRun& r) {
    INFO("launch_rpm " << r.launch_rpm << " h " << r.limiter_hysteresis << " window min " << r.armed_rpm_min << " max " << r.armed_rpm_max << " mean " << r.armed_rpm_mean);
    CHECK(r.armed_rpm_min >= r.launch_rpm - r.limiter_hysteresis - kArmedCoastMarginRpm);
    CHECK(r.armed_rpm_max <= r.launch_rpm + kArmedOvershootMarginRpm);
    CHECK(r.armed_rpm_mean >= r.launch_rpm - r.limiter_hysteresis);
    CHECK(r.armed_rpm_mean <= r.launch_rpm);
}

void check_v8(int mode, const LaunchRun& r) {
    INFO("TC mode " << mode);
    CHECK(r.armed_seen);
    CHECK(r.launch_seen);
    CHECK_FALSE(r.stalled);
    check_armed_rpm(r);
    CHECK(r.boost_at_release >= kBoostMinBar);
    CHECK(std::abs(r.kappa_mean - slip_target_of(mode)) <= kKappaTol);
    CHECK(std::abs(r.wing_deg_at_release - 10.0) <= kWingTolDeg);
}

} // namespace

TEST_CASE("V8a: hyper + Racing unit, asphalt from rest, TC On - armed rpm, boost, rear kappa, wing, no stall", "[assist][vehicle][drivetrain]") {
    LaunchOpts o;
    o.mode = 1;
    o.print = true;
    const LaunchRun r = run_launch(hyper_racing(), o);
    std::printf("V8a On: armed rpm mean %.0f dev_max %.0f, boost %.3f bar, wing %.2f deg, kappa(0.2-1.0 s) %.4f (target %.2f), stalled %d\n",
                r.armed_rpm_mean, r.armed_rpm_dev_max, r.boost_at_release, r.wing_deg_at_release, r.kappa_mean, slip_target_of(1), int(r.stalled));
    std::printf("V8a armed rpm vs launch_rpm: committed window (last 0.5 s) dev max %.1f p95 %.1f mean %.1f | t_reach(launch-150) %.3f s; hold after t_reach dev max %.1f p95 %.1f mean %.1f, overshoot max %.1f, undershoot max %.1f\n",
                r.armed_rpm_dev_max, r.armed_rpm_dev_p95, r.armed_rpm_dev_mean, r.armed_rpm_t_reach, r.armed_hold_dev_max, r.armed_hold_dev_p95, r.armed_hold_dev_mean,
                r.armed_rpm_over_max, r.armed_rpm_under_max);
    check_v8(1, r);
}

TEST_CASE("V8b: hyper + Racing unit, asphalt from rest, TC Sport", "[assist][vehicle][drivetrain]") {
    LaunchOpts o;
    o.mode = 3;
    const LaunchRun r = run_launch(hyper_racing(), o);
    std::printf("V8b Sport: armed rpm mean %.0f dev_max %.0f, boost %.3f bar, wing %.2f deg, kappa(0.2-1.0 s) %.4f (target %.2f), stalled %d\n",
                r.armed_rpm_mean, r.armed_rpm_dev_max, r.boost_at_release, r.wing_deg_at_release, r.kappa_mean, slip_target_of(3), int(r.stalled));
    check_v8(3, r);
}

TEST_CASE("V8d: boost at release depends on the launch rpm - 4500 misses the 0.4 bar band, the shipped 5000 meets it", "[assist][vehicle][drivetrain]") {
    // Record of the tuning (owner Q7 "[AS] data defaults, tuned by V8"): measured 1.5 s armed, unloaded, fuel-cut limiter
    // cycling at launch_rpm: 4500 -> 0.288 bar (miss), 5000 -> 0.502 (shipped), 5500 0.97, 6000 1.4 (earlier sweep).
    real boost[2] = {0, 0};
    int i = 0;
    for (const real rpm : {4500.0, 5000.0}) {
        VehicleDesc d = hyper_racing();
        d.assist_unit->launch_control->launch_rpm = rpm;
        LaunchOpts o;
        o.mode = 1;
        const LaunchRun r = run_launch(d, o);
        std::printf("V8d launch_rpm %.0f: boost %.3f bar, armed dev_max %.0f, stalled %d\n", rpm, r.boost_at_release, r.armed_rpm_dev_max, int(r.stalled));
        CHECK_FALSE(r.stalled);
        boost[i++] = r.boost_at_release;
    }
    CHECK(boost[0] < kBoostMinBar); // documents the miss; flips when the data or the engine changes
    CHECK(boost[1] >= kBoostMinBar);
    CHECK(boost[1] > boost[0]);
}

TEST_CASE("V8e: the Sport unit (launch_rpm_fraction 0.55) on car_hyper - rpm and boost, report only", "[assist][vehicle][drivetrain]") {
    // 0.55 x car_hyper's 8250 rpm limiter = 4537.5 rpm: close to the old 4500, so far below the 0.4 bar the Racing unit
    // needed 5000 for. The Sport unit is a generic fit-any-engine unit; it is NOT tuned for car_hyper's boost, so only
    // the structural checks are asserted and the numbers are printed.
    const VehicleDesc sport = load_hyper(kSport);
    LaunchOpts o;
    o.mode = 1;
    const LaunchRun r = run_launch(sport, o);
    std::printf("V8e Sport unit on car_hyper: launch rpm %.1f, armed rpm mean %.0f dev_max %.0f, boost %.3f bar, kappa %.4f (target %.2f), stalled %d\n",
                0.55 * 8250.0, r.armed_rpm_mean, r.armed_rpm_dev_max, r.boost_at_release, r.kappa_mean, sport.assist_unit->launch_control->slip_target[1],
                int(r.stalled));
    CHECK(r.armed_seen);
    CHECK(r.launch_seen);
    CHECK_FALSE(r.stalled);
    // (the +-150 armed-rpm band is asserted for the tuned Racing unit in V8a/V8b, not for this generic unit: dev_max is printed above)
}

TEST_CASE("V8c [long]: 0-100 km/h with launch control <= the TC-On WOT auto-clutch launch without it", "[assist][vehicle][drivetrain][long]") {
    LaunchOpts lc;
    lc.mode = 1;
    lc.to_100 = true;
    const LaunchRun with = run_launch(hyper_racing(), lc);
    LaunchOpts base = lc;
    base.lc = false;
    const LaunchRun without = run_launch(hyper_racing(), base);
    std::printf("V8c: 0-100 km/h  with LC (from release) %.3f s,  baseline (from throttle) %.3f s\n", with.t_to_100, without.t_to_100);
    REQUIRE(with.t_to_100 > 0);
    REQUIRE(without.t_to_100 > 0);
    CHECK(with.t_to_100 <= without.t_to_100);
    CHECK_FALSE(with.stalled);
    CHECK_FALSE(without.stalled);
}

// ---------------------------------------------------------------------------------------------------------------
// P: precedence with the I2 crawl path.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("P1: while LC owns the car, TC never acts through the clutch (I2 crawl path suspended)", "[assist][vehicle][drivetrain]") {
    for (int mode : {1, 3}) {
        LaunchOpts o;
        o.mode = mode;
        const LaunchRun r = run_launch(hyper_racing(), o);
        INFO("TC mode " << mode);
        REQUIRE(r.launch_seen);
        CHECK(r.tc_clutch_ticks_in_lc == 0);
        CHECK_FALSE(r.cap_finite_in_lc);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// D: Idle equals no block; determinism; allocation.
// ---------------------------------------------------------------------------------------------------------------
namespace {

struct Traj {
    std::vector<Vec3> pos, vel;
    std::vector<real> rpm;
};

// a 6 s sedan drive (throttle ramp, brake pulse) that never meets the arming conditions at speed 0: throttle 0.6
// from rest, gear 1 -> 2 by request, brake 0.7 at 4 s while moving
Traj drive_sedan(const VehicleDesc& d, bool lc_channel_on) {
    SedanRig s = make_sedan(d);
    sedan_controls(s.w(), lc_channel_on);
    s.w().set_control("handbrake", 1.0);
    ps_test::settle(s.w(), 0.5);
    s.w().set_control("handbrake", 0.0);
    Traj t;
    for (int k = 0; k < 6 * 240; ++k) {
        const real tt = k / 240.0;
        s.w().set_control("throttle", 0.6);
        s.w().set_control("brake", tt >= 4.0 ? 0.7 : 0.0);
        s.w().set_control("gear_request", tt >= 2.0 ? 2.0 : 1.0);
        s.w().step();
        t.pos.push_back(s.w().get_pose(s.rig.chassis).position);
        t.vel.push_back(s.w().get_motion(s.rig.chassis).linear);
        t.rpm.push_back(s.w().powertrain_state(s.rig.vehicle).engine_rpm);
    }
    return t;
}

bool same(const Traj& a, const Traj& b) {
    if (a.pos.size() != b.pos.size()) return false;
    for (std::size_t i = 0; i < a.pos.size(); ++i) {
        if (a.pos[i].x != b.pos[i].x || a.pos[i].y != b.pos[i].y || a.pos[i].z != b.pos[i].z) return false;
        if (a.vel[i].x != b.vel[i].x || a.vel[i].y != b.vel[i].y || a.vel[i].z != b.vel[i].z) return false;
        if (a.rpm[i] != b.rpm[i]) return false;
    }
    return true;
}

} // namespace

TEST_CASE("D1: Idle equals no block - the car's trajectory is bit-identical to the same car without launch_control", "[assist][vehicle][drivetrain]") {
    TempDir td("d1");
    const VehicleDesc with = load_sedan(with_unit(kRacing));
    const VehicleDesc without = load_sedan(with_unit(racing_with_launch(td, "nolc.json", json::object(), true)));
    const Traj base = drive_sedan(without, false);
    CHECK(same(drive_sedan(with, false), base)); // channel 0
    CHECK(same(drive_sedan(with, true), base));  // channel 1 but the arming conditions are never met in this drive
}

TEST_CASE("D2: a car without the block keeps its state hash (no LC bytes folded)", "[assist][vehicle][drivetrain]") {
    TempDir td("d2");
    const VehicleDesc without = load_sedan(with_unit(racing_with_launch(td, "nolc.json", json::object(), true)));
    const VehicleDesc with = load_sedan(with_unit(kRacing));
    SedanRig a = make_sedan(without);
    SedanRig b = make_sedan(with);
    a.w().step();
    b.w().step();
    CHECK(a.w().state_hash() != b.w().state_hash()); // the block's hashed bytes exist only where the block is
    // and two cars without it hash alike run to run
    SedanRig c = make_sedan(without);
    c.w().step();
    CHECK(a.w().state_hash() == c.w().state_hash());
}

TEST_CASE("V11: 1 vs 10 workers - equal hash every tick of a launch; zero own allocation per tick with LC active", "[assist][vehicle][drivetrain][determinism]") {
    LaunchOpts o;
    o.mode = 1;
    o.after_release_s = 1.0;
    o.keep_hashes = true;
    o.alloc = true;
    o.workers = 1;
    const LaunchRun one = run_launch(hyper_racing(), o);
    o.workers = 10;
    const LaunchRun ten = run_launch(hyper_racing(), o);
    REQUIRE(one.hashes.size() == ten.hashes.size());
    REQUIRE(one.hashes.size() > 500);
    std::size_t first_diff = one.hashes.size();
    for (std::size_t i = 0; i < one.hashes.size(); ++i)
        if (one.hashes[i] != ten.hashes[i]) {
            first_diff = i;
            break;
        }
    CHECK(first_diff == one.hashes.size());
    CHECK(one.launch_seen);
    if (jobs::debug_alloc::hook_available()) {
        CHECK(one.own_alloc == 0);
        CHECK(ten.own_alloc == 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Diagnostic (hidden): a launch trace with env-given tuning, for the design work.
// ---------------------------------------------------------------------------------------------------------------
namespace {
real env_real(const char* name, real def) {
#if defined(_MSC_VER) || defined(_WIN32)
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return def;
    const real v = std::stod(buf);
    std::free(buf);
    return v;
#else
    const char* e = std::getenv(name);
    return e ? std::stod(e) : def;
#endif
}
} // namespace

TEST_CASE("I4 diag: launch trace", "[.][diag]") {
    LaunchOpts o;
    o.mode = static_cast<int>(env_real("PS_I4_MODE", 1));
    o.print = true;
    o.print_every = static_cast<int>(env_real("PS_I4_EVERY", 12));
    o.armed_s = env_real("PS_I4_ARMED_S", 1.5);
    o.print_armed = env_real("PS_I4_PRINT_ARMED", 0) > 0;
    o.after_release_s = env_real("PS_I4_AFTER_S", 2.0);
    o.to_100 = env_real("PS_I4_TO100", 0) > 0;
    LaunchAssistDesc d = *hyper_racing().assist_unit->launch_control;
    d.kp = env_real("PS_I4_KP", d.kp);
    d.ki_per_s = env_real("PS_I4_KI", d.ki_per_s);
    d.attack_rate_per_s = env_real("PS_I4_ATTACK", d.attack_rate_per_s);
    d.recovery_rate_per_s = env_real("PS_I4_RECOVERY", d.recovery_rate_per_s);
    d.clutch_ramp_s = env_real("PS_I4_RAMP", d.clutch_ramp_s);
    d.min_fraction = env_real("PS_I4_MINFRAC", d.min_fraction);
    d.slip_floor_mps = env_real("PS_I4_FLOOR", d.slip_floor_mps);
    d.launch_rpm = env_real("PS_I4_RPM", d.launch_rpm);
    d.clutch_up_per_s = env_real("PS_I4_CUP", d.clutch_up_per_s);
    d.clutch_down_per_s = env_real("PS_I4_CDOWN", d.clutch_down_per_s);
    d.handover_speed_mps = env_real("PS_I4_HOSPEED", d.handover_speed_mps);
    o.lc_override = d;
    const LaunchRun r = run_launch(hyper_racing(), o);
    std::printf("diag: boost %.3f kappa %.4f [%.3f..%.3f] t_ho %.2f v1s %.2f vend %.2f stalled %d t100 %.3f\n", r.boost_at_release, r.kappa_mean,
                r.kappa_min, r.kappa_max, r.t_handover, r.v_at_1s, r.v_at_end, int(r.stalled), r.t_to_100);
}
