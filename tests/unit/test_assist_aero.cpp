// tests/unit/test_assist_aero.cpp - assists plan I5 (D-AE1, vault PHYS-074): the assist unit's rear-wing automation
// (block `aero_control` of an assist_unit electronics part, channel assist.aero_control), a port of racing_game's law
// (core/src/session.cpp lines 812-828, defaults core/src/environment.cpp / rg/environment.h).
//
//   F*  fitment: loader shapes and rejections, a unit on a wingless car, channel registration, fitted/active bits
//   W*  V10 - the wing command against the racing_game formula (re-implemented below from the literal source),
//       through a real World tick; wind (airspeed, not ground speed); the airbrake gate
//   C*  channel semantics: channel 0 leaves the World command in charge (bit-identical hash), channel 1 ignores it,
//       switching off puts the World command back
//   L*  the launch hook (white-box, a VehicleInstance driven directly): LC > airbrake > cruise
//   D*  1 vs 10 workers equal hash with the controller active, zero own allocation per tick
//   V11 [long] car_hyper + the shipped Racing unit: determinism with braking above 20 m/s, zero allocation, and the
//       measured wing offset follows the target within the surface's slew rate
//
// BANDS COMMITTED BEFORE THE FIRST RUN (assists plan B6 V10/V11, brief assists_i5):
//   V10  |wing offset / limit - formula command| <= 1e-12 for every v in {0, 5, 10, 15, 20, 25, 40} m/s and brake in
//        {0, 0.45, 0.7, 1} (the test surface has a 1000 rad/s slew so one tick reaches the command; the pure function
//        is compared exactly, == to the formula, no tolerance); |snapshot target - formula target| <= 1e-12 deg.
//        Airbrake inactive below 20 m/s: at 19.99 m/s and brake 1 the target is the cruise term alone (1 deg * 0.999),
//        at 20.0 m/s it is airbrake-blended (> 40 deg).
//        With wind, the airspeed is used: ground 15 m/s into a 10 m/s headwind = airspeed 25 -> airbrake ON (target
//        > 40 deg at brake 1); ground 25 m/s with a 10 m/s tailwind = airspeed 15 -> airbrake OFF (target < 3 deg).
//   C0   channel 0 (explicit): per-tick state hash equal over a 10 s drive to the same car with a unit that has no
//        aero_control block (the TC mode table is identical in both), World command 0.5 held throughout; with the
//        channel on the hash differs (the guard is not vacuous). An aero-only unit hashes like no unit at all.
//   C1   channel 1: hash independent of the World command (0 vs 0.9); channel 0: dependent.
//   C2   switching 1 -> 0 returns the World command: after 0.5 s the offset is within 1e-9 of the slew limit
//        rate*0.5 s toward 0.5*max (0.305 rad < 0.4800 rad, so the rate limit decides), well away from 0.
//   F    wingless car: loads, bit 6 of assist_fitted_bits 0, no channel, hash like a car without the unit on a
//        TC-less unit; winged car: bit 6 set, channel default = default_enabled, active bit 6 follows the channel.
//   L    launch hook: target = launch_offset_deg (10) at 5 m/s with brake 1 (cruise would be 0), at 25 m/s with brake 1
//        (airbrake would be ~45), command = 10 deg / limit; hook off restores the law.
//   D    1 vs 10 workers identical hash for ticks of a braking run above 20 m/s; own allocation violations == 0.
//   V11  as D on car_hyper (rolling in neutral at 50 m/s, full brake from 0.3 s to 2.0 s, speed stays above 20 m/s);
//        end-to-end: per tick the measured offset never moves faster than rate/240 (+1e-9 relative), never moves away
//        from the target, sits on the target (<= 1e-9 rad) whenever the slew limit does not bind, and at 2.0 s has
//        reached the airbrake target 45 deg (0.785398 rad) within 1e-6 rad.
//
// RECORDED SABOTAGES (2026-10-12, each performed by hand via a temporary env switch, then reverted; "x" = failing checks):
//   S1 forward speed taken from the ground velocity (wind ignored) ........ W2 fails (4 assertions, lines "headwind/tailwind")
//   S2 airbrake speed gate 20 m/s -> 0 m/s ................................ W0, W1 (gate checks) and W2 fail
//   S3 World command not restored after the substeps ...................... C2 fails (offset stays on the controller value)
//   S4 airbrake wins over the launch hook (priority swapped) .............. L1 fails (launch target 10 deg not held)
//   S5 channel registered on a wingless car (fitment ignores the surface) . F2 and F3 fail
// All five pass again unmodified: 13 test cases, 1388 assertions.

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
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
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
const std::string kUnitHead = R"({"format":"physics_sim.electronics/1","kind":"assist_unit",)";

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_assist_aero_" + tag);
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    [[nodiscard]] std::string file(const std::string& name) const { return (path / name).generic_string(); }
};

void absolutise(json& j) {
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s.rfind("../", 0) == 0) j = kData + "/" + s.substr(3);
    } else if (j.is_array() || j.is_object()) {
        for (auto& e : j) absolutise(e);
    }
}

void write_text(const std::string& p, const std::string& text) {
    std::ofstream f(p, std::ios::binary);
    f << text;
}

io::VehicleLoadOptions with_unit(const std::string& path) {
    io::VehicleLoadOptions o;
    o.electronics["assist"] = path;
    return o;
}

// car_sedan with the rear wing of car_hyper.json (copied numbers, data/vehicles/car_hyper.json aero.surfaces[2]) added
// in code, so the aero controller can be driven on a car that is cheap to build. The legacy isotropic CdA is dropped
// (it is rejected next to explicit surfaces). `rate` is the surface's slew rate.
VehicleDesc winged_sedan(const io::VehicleLoadOptions& o, real rate = 0.6108652381980153) {
    VehicleDesc d = io::load_vehicle_json(kData + "/vehicles/car_sedan.json", o);
    d.aero.drag_area_m2 = 0;
    aero::SurfaceDesc s;
    s.name = "rear_wing";
    s.position_local = Vec3{-1.714, 0, 0.475};
    s.area_m2 = 0.516;
    s.cl0 = -0.15;
    s.lift_slope_per_rad = 4.5;
    s.stall_angle_rad = 0.3;
    s.cd0 = 0.025;
    s.induced_drag = 0.09;
    s.stalled_drag = 1.2;
    s.rest_angle_rad = 0.13962634015954636;
    s.min_offset_rad = -0.13962634015954636;
    s.max_offset_rad = 0.9599310885968813;
    s.rate_rad_s = rate;
    s.max_lift_m = 0.28;
    d.aero.surfaces.push_back(s);
    d.aero.body.drag_area_xyz_m2 = Vec3{0.68, 1.7, 2.6};
    d.aero.body.reference_area_m2 = 1.9;
    d.aero.body.reference_length_m = 2.7;
    return d;
}

// The racing_game law, re-implemented from the literal source (session.cpp 816-828, defaults 4 / 45 / 20 / 0.45):
// double precision, std::clamp, no shared code with the production function.
struct Expected {
    double command, target_deg;
};
Expected racing_game_wing(double forward_speed, double brake, double max_offset_rad, double min_offset_rad) {
    double target = 4.0 * std::clamp((forward_speed - 10) / 20, 0.0, 1.0);
    if (forward_speed >= 20.0) {
        const double demand = std::clamp((brake - 0.45) / (1 - 0.45), 0.0, 1.0);
        target += (45.0 - target) * demand;
    }
    const double radians = target * 3.141592653589793 / 180;
    const double limit = radians >= 0 ? max_offset_rad : -min_offset_rad;
    return {limit > 0 ? std::clamp(radians / limit, -1.0, 1.0) : 0.0, target};
}

std::size_t wing_index(const VehicleDesc& d) {
    for (std::size_t i = 0; i < d.aero.surfaces.size(); ++i)
        if (d.aero.surfaces[i].name == "rear_wing") return i;
    FAIL("no rear_wing");
    return 0;
}

bool has_channel(World& w, const std::string& name) {
    const auto names = w.control_channel_names();
    return std::find(names.begin(), names.end(), name) != names.end();
}

// Chassis forward speed as the controller sees it, from the same world state the tick starts with.
double forward_speed_of(World& w, BodyId chassis, Vec3 wind = {}) {
    const auto rot = w.get_pose(chassis).orientation;
    return rot.inverse().rotate(w.get_motion(chassis).linear - wind).x;
}

void set_speed(World& w, BodyId chassis, real v) {
    Motion m;
    m.linear = Vec3{v, 0, 0};
    w.backend().set_motion(chassis, m);
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// F: loader and fitment.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("F0: the shipped Racing unit carries racing_game's wing law as data; Street and Sport carry none", "[assist][aero][io]") {
    const io::ElectronicsPart ra = io::load_electronics_json(kRacing);
    REQUIRE(ra.unit.aero_control.has_value());
    const AeroControlDesc& a = *ra.unit.aero_control;
    CHECK(a.surface == "rear_wing");
    CHECK(a.default_enabled);
    CHECK(a.cruise_offset_deg == 4.0);
    CHECK(a.cruise_ramp_start_m_s == 10.0);
    CHECK(a.cruise_ramp_end_m_s == 30.0);
    CHECK(a.airbrake_offset_deg == 45.0);
    CHECK(a.airbrake_min_speed_m_s == 20.0);
    CHECK(a.airbrake_brake_start == 0.45);
    CHECK(a.airbrake_brake_full == 1.0);
    CHECK(a.launch_offset_deg == 10.0);
    CHECK_FALSE(io::load_electronics_json(kStreet).unit.aero_control.has_value());
    CHECK_FALSE(io::load_electronics_json(kSport).unit.aero_control.has_value());
}

TEST_CASE("F1: aero_control loader defaults, ranges and rejections", "[assist][aero][io]") {
    TempDir td("f1");
    const auto load = [&](const std::string& body) {
        write_text(td.file("p.json"), kUnitHead + body);
        return io::load_electronics_json(td.file("p.json"));
    };
    // the empty block is valid and equals racing_game's defaults; default_enabled is off unless the part says so
    const io::ElectronicsPart e = load(R"("grade":"racing","aero_control":{}})");
    REQUIRE(e.unit.aero_control.has_value());
    CHECK(e.unit.aero_control->surface == "rear_wing");
    CHECK_FALSE(e.unit.aero_control->default_enabled);
    CHECK(e.unit.aero_control->cruise_offset_deg == 4.0);
    CHECK(e.unit.aero_control->airbrake_brake_full == 1.0);
    CHECK(load(R"("grade":"racing","aero_control":{"default_enabled":1,"surface":"canard","launch_offset_deg":-3}})").unit.aero_control->default_enabled);
    // grade table: Street and Sport reject the block with the I1 message
    CHECK_THROWS_WITH(load(R"("grade":"street","aero_control":{}})"), ContainsSubstring("grade 'street' does not allow 'aero_control'"));
    CHECK_THROWS_WITH(load(R"("grade":"sport","aero_control":{}})"), ContainsSubstring("grade 'sport' does not allow 'aero_control'"));
    // launch control loads since assists I4 (tests/unit/test_assist_launch.cpp)
    CHECK(load(R"("grade":"racing","launch_control":{"launch_rpm":4500}})").unit.launch_control.has_value());
    // shape and unknown keys
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":3})"), ContainsSubstring("aero_control must be an object"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"bogus":1}})"), ContainsSubstring("aero_control: unknown field 'bogus'"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"surface":""}})"), ContainsSubstring("'surface' must be a non-empty string"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"default_enabled":2}})"), ContainsSubstring("'default_enabled' must be a boolean or 0/1"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"cruise_offset_deg":"4"}})"), ContainsSubstring("'cruise_offset_deg' must be a number"));
    // ranges (racing_game's: cruise [-8, 55], airbrake offset [0, 55], min speed [0, 150], threshold [0, 0.99])
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"cruise_offset_deg":56}})"), ContainsSubstring("'cruise_offset_deg' must be in [-8.0"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"cruise_offset_deg":-9}})"), ContainsSubstring("'cruise_offset_deg' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"airbrake_offset_deg":-1}})"), ContainsSubstring("'airbrake_offset_deg' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"airbrake_offset_deg":56}})"), ContainsSubstring("'airbrake_offset_deg' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"airbrake_min_speed_m_s":151}})"), ContainsSubstring("'airbrake_min_speed_m_s' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"airbrake_brake_start":1.0}})"), ContainsSubstring("'airbrake_brake_start' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"airbrake_brake_full":0}})"), ContainsSubstring("'airbrake_brake_full' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"launch_offset_deg":60}})"), ContainsSubstring("'launch_offset_deg' must be in"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"cruise_ramp_start_m_s":30}})"),
                      ContainsSubstring("'cruise_ramp_end_m_s' must be greater than 'cruise_ramp_start_m_s'"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","aero_control":{"airbrake_brake_start":0.6,"airbrake_brake_full":0.5}})"),
                      ContainsSubstring("'airbrake_brake_full' must be greater than 'airbrake_brake_start'"));
    // extends keeps the block of the ancestor
    write_text(td.file("child.json"), R"({"format":"physics_sim.electronics/1","kind":"assist_unit","extends":"base.json","name":"child"})");
    write_text(td.file("base.json"), kUnitHead + R"("grade":"racing","aero_control":{"cruise_offset_deg":6}})");
    const io::ElectronicsPart child = io::load_electronics_json(td.file("child.json"));
    REQUIRE(child.unit.aero_control.has_value());
    CHECK(child.unit.aero_control->cruise_offset_deg == 6.0);
}

TEST_CASE("F2: a Racing unit on a wingless car loads and is simply not fitted", "[assist][aero][vehicle]") {
    // DECISION (progress file): a unit naming a surface the car lacks is inactive with fitted bit 6 = 0, not a load
    // error, so one generic Racing unit attaches to any car.
    const VehicleDesc d = io::load_vehicle_json(kData + "/vehicles/car_sedan.json", with_unit(kRacing));
    REQUIRE(d.assist_unit.has_value());
    REQUIRE(d.assist_unit->aero_control.has_value());
    CHECK_FALSE(find_aero_control_surface(d.aero, d.assist_unit).has_value());
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    CHECK_FALSE(has_channel(w, "assist.aero_control"));
    w.set_control("assist.aero_control", 1.0); // a stray write on a car without the channel is harmless
    w.set_control("throttle", 0.5);
    for (int i = 0; i < 60; ++i) w.step();
    const auto& pt = w.powertrain_state(rig.vehicle);
    CHECK((pt.assist_fitted_bits & 64u) == 0);
    CHECK((pt.assist_active_bits & 64u) == 0);
    CHECK_FALSE(pt.aero_control_active);
    CHECK(pt.aero_control_target_deg == 0.0);
    // TC of the unit still works on this car
    CHECK((pt.assist_fitted_bits & 8u) != 0);
}

TEST_CASE("F3: a winged car - channel registered with default_enabled, fitted/active bit 6, named-surface lookup, none removes", "[assist][aero][vehicle]") {
    {
        const VehicleDesc d = winged_sedan(with_unit(kRacing));
        auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
        World& w = *rig.world;
        CHECK(has_channel(w, "assist.aero_control"));
        CHECK(w.get_control("assist.aero_control") == 1.0); // the shipped unit: default_enabled true
        w.step();
        const auto& pt = w.powertrain_state(rig.vehicle);
        CHECK((pt.assist_fitted_bits & 64u) != 0);
        CHECK((pt.assist_active_bits & 64u) != 0);
        CHECK(pt.aero_control_active);
        w.set_control("assist.aero_control", 0.0);
        w.step();
        const auto& pt2 = w.powertrain_state(rig.vehicle);
        CHECK((pt2.assist_fitted_bits & 64u) != 0); // fitted but off is a third state
        CHECK((pt2.assist_active_bits & 64u) == 0);
        CHECK_FALSE(pt2.aero_control_active);
        CHECK(pt2.aero_control_target_deg == 0.0);
        // >= 0.5 is on, below is off
        w.set_control("assist.aero_control", 0.5);
        w.step();
        CHECK(w.powertrain_state(rig.vehicle).aero_control_active);
        w.set_control("assist.aero_control", 0.49);
        w.step();
        CHECK_FALSE(w.powertrain_state(rig.vehicle).aero_control_active);
    }
    {
        // default_enabled false -> channel default 0, nothing active until the player switches it on
        TempDir td("f3");
        write_text(td.file("u.json"), kUnitHead + R"("grade":"racing","aero_control":{"default_enabled":false}})");
        auto rig = ps_test::build_vehicle_rig_from_desc(winged_sedan(with_unit(td.file("u.json"))), 1500, {2, .4, .15}, .5, 0, 1);
        World& w = *rig.world;
        CHECK(w.get_control("assist.aero_control") == 0.0);
        w.step();
        CHECK((w.powertrain_state(rig.vehicle).assist_fitted_bits & 64u) != 0);
        CHECK_FALSE(w.powertrain_state(rig.vehicle).aero_control_active);
    }
    {
        // a unit naming another surface: the lookup is by name, so rear_wing is then not controlled
        TempDir td("f3b");
        write_text(td.file("u.json"), kUnitHead + R"("grade":"racing","aero_control":{"surface":"canard"}})");
        const VehicleDesc d = winged_sedan(with_unit(td.file("u.json")));
        CHECK_FALSE(find_aero_control_surface(d.aero, d.assist_unit).has_value());
        auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
        CHECK_FALSE(has_channel(*rig.world, "assist.aero_control"));
    }
    {
        // "none" removes the unit: no fitment, no channel
        TempDir td("f3c");
        std::ifstream vf(kData + "/vehicles/car_sedan.json");
        json vj = json::parse(vf);
        absolutise(vj);
        vj["controllers"].push_back({{"id", "assist"}, {"type", "electronics"}, {"ref", kRacing}});
        write_text(td.file("v.json"), vj.dump(1));
        CHECK(io::load_vehicle_json(td.file("v.json")).assist_unit.has_value());
        io::VehicleLoadOptions rem;
        rem.electronics["assist"] = "none";
        VehicleDesc d = io::load_vehicle_json(td.file("v.json"), rem);
        d.aero = winged_sedan(io::VehicleLoadOptions{}).aero;
        CHECK_FALSE(d.assist_unit.has_value());
        auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
        CHECK_FALSE(has_channel(*rig.world, "assist.aero_control"));
        rig.world->step();
        CHECK((rig.world->powertrain_state(rig.vehicle).assist_fitted_bits & 64u) == 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// W: V10 - the law against the racing_game formula.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("W0: the pure law equals the racing_game formula exactly (no tolerance)", "[assist][aero]") {
    const AeroControlDesc c; // the defaults ARE racing_game's
    const real max_off = 0.9599310885968813, min_off = -0.13962634015954636;
    for (double v : {-5.0, 0.0, 5.0, 10.0, 15.0, 19.99, 20.0, 25.0, 30.0, 40.0, 80.0}) {
        for (double b : {-0.2, 0.0, 0.3, 0.45, 0.7, 1.0, 1.4}) {
            CAPTURE(v, b);
            real td = -1;
            const real cmd = aero_control_command(c, max_off, min_off, v, b, false, &td);
            const Expected e = racing_game_wing(v, std::clamp(b, 0.0, 1.0), max_off, min_off);
            CHECK(cmd == e.command);
            CHECK(td == e.target_deg);
        }
    }
    // a negative target (cruise offset below zero) takes the min_offset side of the limit
    AeroControlDesc neg;
    neg.cruise_offset_deg = -4;
    const real cmd = aero_control_command(neg, max_off, min_off, 40.0, 0.0, false);
    CHECK(cmd == Catch::Approx(-4.0 * 3.141592653589793 / 180 / 0.13962634015954636).epsilon(1e-15));
    CHECK(cmd < 0);
    // a surface with no authority on the commanded side gets command 0
    CHECK(aero_control_command(c, 0.0, 0.0, 40.0, 0.0, false) == 0.0);
}

TEST_CASE("W1: V10 through a World tick - wing offset = formula command x limit to 1e-12", "[assist][aero][vehicle]") {
    // slew 1000 rad/s: one tick (4 substeps of 1/960 s) covers any step, so the offset after the tick IS the command
    const VehicleDesc d = winged_sedan(with_unit(kRacing), 1000.0);
    const std::size_t wi = wing_index(d);
    const aero::SurfaceDesc& wing = d.aero.surfaces[wi];
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    ps_test::settle(w, 0.3);
    int cases = 0;
    double worst = 0, worst_target = 0;
    for (double v : {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 40.0}) {
        for (double b : {0.0, 0.45, 0.7, 1.0}) {
            CAPTURE(v, b);
            set_speed(w, rig.chassis, v);
            w.set_control("brake", b);
            const double fwd = forward_speed_of(w, rig.chassis); // the same world state the tick starts with
            w.step();
            const Expected e = racing_game_wing(fwd, b, wing.max_offset_rad, wing.min_offset_rad);
            const auto& tel = w.vehicle_aero_telemetry(rig.vehicle);
            const double got = tel.surfaces[wi].offset_rad / wing.max_offset_rad;
            worst = std::max(worst, std::abs(got - e.command));
            worst_target = std::max(worst_target, std::abs(w.powertrain_state(rig.vehicle).aero_control_target_deg - e.target_deg));
            CHECK(std::abs(got - e.command) <= 1e-12);
            CHECK(std::abs(w.powertrain_state(rig.vehicle).aero_control_target_deg - e.target_deg) <= 1e-12);
            ++cases;
        }
    }
    CHECK(cases == 28);
    std::printf("[W1] %d cases, worst |command error| %.3e, worst |target error| %.3e deg\n", cases, worst, worst_target);

    // the airbrake gate: just below 20 m/s the target is the cruise term alone, at 20 m/s it is blended
    set_speed(w, rig.chassis, 19.99);
    w.set_control("brake", 1.0);
    const double f1 = forward_speed_of(w, rig.chassis);
    w.step();
    CHECK(f1 < 20.0);
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg == Catch::Approx(4.0 * (f1 - 10) / 20).epsilon(1e-12));
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg < 2.5);
    set_speed(w, rig.chassis, 20.05); // the settled chassis is pitched a hair: forward speed is what the controller reads
    const double f2 = forward_speed_of(w, rig.chassis);
    w.step();
    CHECK(f2 >= 20.0);
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg > 40.0);
}

TEST_CASE("W2: with wind the airspeed is used, not the ground speed", "[assist][aero][vehicle]") {
    const VehicleDesc d = winged_sedan(with_unit(kRacing), 1000.0);
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    ps_test::settle(w, 0.3);
    w.set_control("brake", 1.0);
    // headwind 10 m/s (wind blows toward -x): ground 15 -> airspeed 25 -> airbrake ON
    w.set_aero_environment(aero::Environment{1.225, Vec3{-10, 0, 0}});
    set_speed(w, rig.chassis, 15.0);
    const double head = forward_speed_of(w, rig.chassis, Vec3{-10, 0, 0});
    w.step();
    CHECK(head == Catch::Approx(25.0).epsilon(1e-6));
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg > 40.0);
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg == Catch::Approx(racing_game_wing(head, 1.0, 1, -1).target_deg).epsilon(1e-12));
    // tailwind 10 m/s: ground 25 -> airspeed 15 -> airbrake OFF, cruise 1 deg
    w.set_aero_environment(aero::Environment{1.225, Vec3{10, 0, 0}});
    set_speed(w, rig.chassis, 25.0);
    const double tail = forward_speed_of(w, rig.chassis, Vec3{10, 0, 0});
    w.step();
    CHECK(tail == Catch::Approx(15.0).epsilon(1e-6));
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg < 3.0);
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg == Catch::Approx(racing_game_wing(tail, 1.0, 1, -1).target_deg).epsilon(1e-12));
    // a crosswind does not change the forward airspeed
    w.set_aero_environment(aero::Environment{1.225, Vec3{0, 12, 0}});
    set_speed(w, rig.chassis, 25.0);
    w.step();
    CHECK(w.powertrain_state(rig.vehicle).aero_control_target_deg > 40.0);
}

// ---------------------------------------------------------------------------------------------------------------
// C: channel semantics and bit identity.
// ---------------------------------------------------------------------------------------------------------------
namespace {
struct Run {
    std::vector<std::uint64_t> hashes;
    std::vector<real> offset;
    std::uint64_t own_violations = 0;
};

// A rolling run: 30 m/s at tick 0 (hubs roll with the chassis, neutral), a coast, then braking above 20 m/s, optionally
// with a World wing command and a channel value. `seconds` of simulated time.
Run drive(const VehicleDesc& desc, std::optional<real> channel, real world_command, real seconds, int workers = 1, bool alloc = false) {
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0, workers);
    World& w = *rig.world;
    ps_test::settle(w, 0.3);
    set_speed(w, rig.chassis, 30.0);
    vehicle::VehicleResetOptions ro;
    ro.gear = 0;
    ro.hubs_roll_with_chassis = true;
    ro.clutch_locked = false;
    w.reset_vehicle(rig.vehicle, ro);
    if (channel) w.set_control("assist.aero_control", *channel);
    const std::size_t wi = wing_index(desc);
    w.set_aero_surface_command(rig.vehicle, wi, world_command);
    namespace da = jobs::debug_alloc;
    const bool use_alloc = alloc && da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    Run r;
    const int n = static_cast<int>(seconds * 240.0 + 0.5);
    for (int k = 0; k < n; ++k) {
        // coast 2 s, then brake hard (the sedan sheds ~1 g: still above 20 m/s for a while), release, brake again
        const double t = k / 240.0;
        w.set_control("brake", (t >= 2.0 && t < 3.0) || t >= 5.0 ? 1.0 : 0.0);
        w.step();
        r.hashes.push_back(w.state_hash());
        r.offset.push_back(w.vehicle_aero_telemetry(rig.vehicle).surfaces[wi].offset_rad);
    }
    if (use_alloc) {
        da::set_armed(false);
        r.own_violations = da::own_violation_count();
    }
    return r;
}

// the racing unit without its aero block, written next to the test
std::string racing_without_aero(const TempDir& td) {
    std::ifstream f(kRacing);
    json j = json::parse(f);
    j.erase("aero_control");
    write_text(td.file("racing_no_aero.json"), j.dump(1));
    return td.file("racing_no_aero.json");
}
} // namespace

TEST_CASE("C0: channel 0 leaves the World command in charge - hash bit-identical to the unit without the block over 10 s", "[assist][aero][vehicle][determinism]") {
    TempDir td("c0");
    const std::string no_aero = racing_without_aero(td);
    const real wcmd = 0.5;
    const Run base = drive(winged_sedan(with_unit(no_aero)), std::nullopt, wcmd, 10.0);
    const Run off = drive(winged_sedan(with_unit(kRacing)), 0.0, wcmd, 10.0);
    REQUIRE(base.hashes.size() == off.hashes.size());
    CHECK(off.hashes == base.hashes);
    CHECK(off.offset == base.offset);
    // not vacuous: the World command really moved the wing, and the controller (default channel 1) changes the run
    CHECK(base.offset.back() > 0.4);
    const Run on = drive(winged_sedan(with_unit(kRacing)), std::nullopt, wcmd, 10.0);
    CHECK(on.hashes != base.hashes);
    // an aero-only unit has no TC mode table to fold: with channel 0 it hashes like no unit at all
    write_text(td.file("aero_only.json"), kUnitHead + R"("grade":"racing","aero_control":{"default_enabled":true}})");
    const Run none = drive(winged_sedan(io::VehicleLoadOptions{}), std::nullopt, wcmd, 10.0);
    const Run only_off = drive(winged_sedan(with_unit(td.file("aero_only.json"))), 0.0, wcmd, 10.0);
    CHECK(only_off.hashes == none.hashes);
}

TEST_CASE("C1: channel 1 ignores the World command; channel 0 obeys it", "[assist][aero][vehicle][determinism]") {
    const VehicleDesc d = winged_sedan(with_unit(kRacing));
    const Run on0 = drive(d, 1.0, 0.0, 4.0);
    const Run on9 = drive(d, 1.0, 0.9, 4.0);
    CHECK(on0.hashes == on9.hashes);
    const Run off0 = drive(d, 0.0, 0.0, 4.0);
    const Run off9 = drive(d, 0.0, 0.9, 4.0);
    CHECK(off0.hashes != off9.hashes);
}

TEST_CASE("C2: switching the channel off returns the World command", "[assist][aero][vehicle]") {
    const VehicleDesc d = winged_sedan(with_unit(kRacing)); // slew 0.6108652381980153 rad/s
    const std::size_t wi = wing_index(d);
    const aero::SurfaceDesc& wing = d.aero.surfaces[wi];
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    ps_test::settle(w, 0.3);
    w.set_aero_surface_command(rig.vehicle, wi, 0.5);
    // controller on, at standstill: target 0 deg (command 0), the World's 0.5 is ignored
    for (int i = 0; i < 120; ++i) w.step();
    CHECK(w.vehicle_aero_telemetry(rig.vehicle).surfaces[wi].offset_rad == 0.0);
    // switch off: the World command (0.5 x max = 0.48 rad) is back, reached at the slew rate
    w.set_control("assist.aero_control", 0.0);
    const real before = w.vehicle_aero_telemetry(rig.vehicle).surfaces[wi].offset_rad;
    for (int i = 0; i < 120; ++i) w.step(); // 0.5 s
    const real after = w.vehicle_aero_telemetry(rig.vehicle).surfaces[wi].offset_rad;
    CHECK(before == 0.0);
    CHECK(after == Catch::Approx(wing.rate_rad_s * 0.5).margin(1e-9));
    CHECK(after < 0.5 * wing.max_offset_rad);
    // and on again: the controller takes the surface back (target 0 at rest) and the wing slews home
    w.set_control("assist.aero_control", 1.0);
    for (int i = 0; i < 480; ++i) w.step();
    CHECK(w.vehicle_aero_telemetry(rig.vehicle).surfaces[wi].offset_rad == 0.0);
}

// ---------------------------------------------------------------------------------------------------------------
// L: the launch hook (white-box, a VehicleInstance driven directly).
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("L1: launch hook - LC > airbrake > cruise", "[assist][aero][vehicle]") {
    const VehicleDesc d = winged_sedan(with_unit(kRacing), 1000.0);
    const std::size_t wi = wing_index(d);
    const aero::SurfaceDesc& wing = d.aero.surfaces[wi];
    jobs::JobSystem jobs(1);
    std::unique_ptr<IRigidBackend> backend = create_jolt_backend(jobs, Vec3{0, 0, -9.81}, 64);
    BodyDesc cd;
    cd.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    cd.motion = BodyMotionType::Dynamic;
    cd.mass = 1500.0;
    cd.allow_sleep = false;
    cd.pose.position = Vec3{0, 0, 5.0};
    const BodyId chassis = backend->create_body(cd);
    io::SurfaceTable surfaces(kData + "/surfaces/surfaces.json");
    ControlChannelIndices idx;
    std::size_t next = 0;
    for (std::size_t* p : {&idx.steer, &idx.throttle, &idx.brake, &idx.handbrake, &idx.clutch, &idx.shift_up_count, &idx.shift_down_count, &idx.gear_request,
                           &idx.ignition, &idx.starter, &idx.assist_auto_clutch, &idx.assist_auto_blip, &idx.assist_auto_shift, &idx.assist_traction_control,
                           &idx.assist_launch_control, &idx.assist_aero_control})
        *p = next++;
    std::vector<real> values(next, 0.0);
    values[idx.gear_request] = drivetrain::kGearRequestNone;
    values[idx.ignition] = 1.0;
    VehicleInstance inst(d, chassis, &surfaces, idx);
    VehicleTickEnv env;
    const auto run = [&](real v, real brake, bool channel, bool hook) {
        Motion m;
        m.linear = Vec3{v, 0, 0};
        backend->set_motion(chassis, m);
        values[idx.brake] = brake;
        values[idx.assist_aero_control] = channel ? 1.0 : 0.0;
        inst.set_aero_launch_request(hook);
        inst.step_tick(*backend, values, env, 1.0 / 240.0, 960.0);
        return inst.powertrain_state();
    };
    constexpr double kTen = 10.0 * 3.141592653589793 / 180;
    // cruise alone (5 m/s, no brake): 0 deg. With the hook: the launch offset, command = 10 deg / limit
    CHECK(run(5.0, 0.0, true, false).aero_control_target_deg == 0.0);
    {
        const auto& pt = run(5.0, 1.0, true, true); // brake 1 at 5 m/s: no airbrake anyway
        CHECK(pt.aero_control_target_deg == 10.0);
        CHECK(pt.aero_control_active);
        CHECK(inst.aero_telemetry().surfaces[wi].offset_rad == Catch::Approx(wing.max_offset_rad * (kTen / wing.max_offset_rad)).epsilon(1e-12));
    }
    // over the airbrake: 25 m/s, brake 1 -> 45 deg by the law, 10 deg with the hook
    CHECK(run(25.0, 1.0, true, false).aero_control_target_deg > 40.0);
    CHECK(run(25.0, 1.0, true, true).aero_control_target_deg == 10.0);
    // over the cruise term: 40 m/s, no brake -> 4 deg by the law, 10 deg with the hook
    CHECK(run(40.0, 0.0, true, false).aero_control_target_deg == 4.0);
    CHECK(run(40.0, 0.0, true, true).aero_control_target_deg == 10.0);
    // the hook does nothing while the channel is off
    CHECK_FALSE(run(40.0, 0.0, false, true).aero_control_active);
    CHECK(run(40.0, 0.0, false, true).aero_control_target_deg == 0.0);
    // dropping the hook restores the law at once (stateless)
    CHECK(run(40.0, 0.0, true, false).aero_control_target_deg == 4.0);
}

// ---------------------------------------------------------------------------------------------------------------
// D: determinism and allocation on the cheap car.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("D1: controller active - 1 vs 10 workers identical, zero own allocation per tick", "[assist][aero][vehicle][determinism]") {
    if (!jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const VehicleDesc d = winged_sedan(with_unit(kRacing));
    const Run w1 = drive(d, 1.0, 0.0, 6.0, 1, true);
    const Run w10 = drive(d, 1.0, 0.0, 6.0, 10, true);
    CHECK(w1.hashes == w10.hashes);
    CHECK(w1.own_violations == 0);
    CHECK(w10.own_violations == 0);
    // the wing really moved during the braking phase (airbrake blend, 5 s onward)
    CHECK(*std::max_element(w1.offset.begin(), w1.offset.end()) > 0.05);
}

// ---------------------------------------------------------------------------------------------------------------
// V11: car_hyper with the shipped Racing unit attached in the test.
// ---------------------------------------------------------------------------------------------------------------
namespace {
const VehicleDesc& hyper_racing() {
    static const VehicleDesc desc = [] {
        jobs::JobSystem pool(10);
        io::VehicleLoadOptions o;
        o.maps.pool = &pool;
        o.maps.cache_dir = std::string(PS_SOURCE_DIR) + "/out/assists_i5_cache";
        fs::create_directories(o.maps.cache_dir);
        o.electronics["assists"] = kRacing; // Assists I2: car_hyper ships this unit under id "assists"; same id = replace
        return io::load_vehicle_json(kData + "/vehicles/car_hyper.json", o);
    }();
    return desc;
}

struct HyperRun {
    std::vector<std::uint64_t> hashes;
    std::vector<real> offset, speed;
    std::vector<double> expected_target_deg;
    std::vector<std::uint8_t> active;
    std::uint64_t own_violations = 0;
};

// 50 m/s rolling in neutral, coast 0.3 s, then full brake until 2.0 s (a hypercar sheds ~1.5 g at most, so the speed
// stays above 20 m/s: 50 - 1.5 * 9.81 * 1.7 = 25 m/s), then 0.2 s more released.
HyperRun hyper_drive(int workers, bool alloc) {
    const VehicleDesc& desc = hyper_racing();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0, workers);
    World& w = *rig.world;
    ps_test::settle(w, 0.5);
    set_speed(w, rig.chassis, 50.0);
    vehicle::VehicleResetOptions ro;
    ro.gear = 0;
    ro.hubs_roll_with_chassis = true;
    ro.clutch_locked = false;
    w.reset_vehicle(rig.vehicle, ro);
    const std::size_t wi = wing_index(desc);
    REQUIRE(w.get_control("assist.aero_control") == 1.0);
    namespace da = jobs::debug_alloc;
    const bool use_alloc = alloc && da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    HyperRun r;
    const int n = static_cast<int>(2.2 * 240.0 + 0.5);
    for (int k = 0; k < n; ++k) {
        const double t = k / 240.0;
        const double brake = (t >= 0.3 && t < 2.0) ? 1.0 : 0.0;
        w.set_control("brake", brake);
        const double fwd = forward_speed_of(w, rig.chassis);
        w.step();
        r.hashes.push_back(w.state_hash());
        r.offset.push_back(w.vehicle_aero_telemetry(rig.vehicle).surfaces[wi].offset_rad);
        r.speed.push_back(fwd);
        r.expected_target_deg.push_back(
            racing_game_wing(fwd, brake, desc.aero.surfaces[wi].max_offset_rad, desc.aero.surfaces[wi].min_offset_rad).target_deg);
        r.active.push_back(w.powertrain_state(rig.vehicle).aero_control_active ? 1 : 0);
    }
    if (use_alloc) {
        da::set_armed(false);
        r.own_violations = da::own_violation_count();
    }
    return r;
}
} // namespace

TEST_CASE("V11: hyper + Racing unit - 1 vs 10 workers, zero allocation, the wing follows the target within its slew rate", "[assist][aero][vehicle][determinism][long]") {
    if (!jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const VehicleDesc& desc = hyper_racing();
    const std::size_t wi = wing_index(desc);
    const aero::SurfaceDesc& wing = desc.aero.surfaces[wi];
    const HyperRun a = hyper_drive(1, true);
    const HyperRun b = hyper_drive(10, true);
    CHECK(a.hashes == b.hashes);
    CHECK(a.own_violations == 0);
    CHECK(b.own_violations == 0);

    // end-to-end: the measured offset follows the target. Per tick it never moves faster than rate / tick rate, and
    // whenever it is not slew-limited it sits on the target (target tracked from the speed read at tick start).
    const double max_step = wing.rate_rad_s / 240.0;
    double worst_step = 0, worst_track = 0;
    int tracked = 0;
    real prev = 0; // the surface starts at offset 0 (`offset_rad` is the actuator offset, not the angle)
    for (std::size_t k = 0; k < a.offset.size(); ++k) {
        worst_step = std::max(worst_step, std::abs(static_cast<double>(a.offset[k] - prev)));
        CHECK(std::abs(a.offset[k] - prev) <= max_step * (1 + 1e-9) + 1e-12);
        const double limit = a.expected_target_deg[k] >= 0 ? wing.max_offset_rad : -wing.min_offset_rad;
        const double target_off = std::clamp(a.expected_target_deg[k] * 3.141592653589793 / 180 / limit, -1.0, 1.0) * wing.max_offset_rad;
        // the offset moves toward the target: after the tick it is no further from it than before
        CHECK(std::abs(a.offset[k] - target_off) <= std::abs(prev - target_off) + 1e-9);
        if (std::abs(prev - target_off) <= max_step * 0.999) {
            worst_track = std::max(worst_track, std::abs(a.offset[k] - target_off));
            ++tracked;
        }
        prev = a.offset[k];
    }
    // from the brake at 0.3 s the wing ramps from 4 deg toward 45 deg (0.7854 rad) at 35 deg/s: reached by ~1.5 s
    const std::size_t k_end = static_cast<std::size_t>(2.0 * 240) - 1;
    const double brake_target = 45.0 * 3.141592653589793 / 180;
    std::printf("[V11] worst per-tick step %.6f rad (max %.6f), tracked ticks %d worst tracking error %.3e rad, offset at 2.0 s %.6f (airbrake %.6f), speed %.2f -> %.2f\n",
                worst_step, max_step, tracked, worst_track, a.offset[k_end], brake_target, a.speed.front(), a.speed[k_end]);
    CHECK(a.speed[k_end] > 20.0); // the airbrake gate stayed open for the whole braking window
    CHECK(std::abs(a.offset[k_end] - brake_target) <= 1e-6);
    CHECK(tracked > 100);
    CHECK(worst_track <= 1e-9);
    CHECK(*std::min_element(a.active.begin(), a.active.end()) == 1);
}
