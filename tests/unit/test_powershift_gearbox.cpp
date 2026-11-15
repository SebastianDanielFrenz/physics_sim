// tests/unit/test_powershift_gearbox.cpp - tractor plan stage 1: the powershift gearbox kind (docs/data_formats/
// gearbox.md "Powershift gearboxes", automatic_tcu "powershift" block, core/src/drivetrain/auto_transmission.cpp
// powershift_desired). White-box against AutoTransmission (the shift law, the range gap) plus the loader.
//
// The shift law is checked on hand-built ratios (geometric, step 1.143, 12 gears) at hand-computed road speeds, so
// every expected gear follows from the band [down, up] = [1400, 2150] rpm at full throttle and its target
// 1400 + 0.5 * 750 = 1775 rpm, not from the implementation.
//
// Recorded sabotages (each made, the named cases seen to fail, then reverted):
//   S1 upshift limited to g + 1 (skip removed): "upshift skips gears" fails.
//   S2 downshift picks g - 1 instead of the target search: "downshift goes straight" fails.
//   S3 start_gear ignored in fit_gear (returns 1): "start gear" fails.
//   S4 range gap never entered (start_shift's Range line removed): "range change opens both clutches" fails.
//   S5 loader accepts forward_range of the wrong length: "rejections" fails.

#include "ps/io/vehicle_io.h"
#include "vehicle_test_helpers.h"

#include "../../core/src/drivetrain/auto_transmission.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <functional>
#include <limits>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

using namespace ps;
using namespace ps::drivetrain;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;

AutoTransmission::Desc make_desc(int range_size = 4, real range_time = 0.0) {
    AutoTransmission::Desc d;
    d.powershift = true;
    real r = 21.0;
    for (int i = 0; i < 12; ++i) {
        d.forward_ratios.push_back(r);
        r /= 1.143;
    }
    d.reverse_ratios = {-23.1};
    d.clutch.capacity_nm = 1100.0;
    d.clutch.engage_time_s = 0.35;
    d.tcu.min_time_in_gear_s = 0.5;
    d.tcu.schedule = {1600.0, 2150.0, 1000.0, 1400.0};
    d.tcu.powershift = PowershiftTcuDesc{6, 0.5, 4};
    if (range_size > 0) {
        for (int i = 0; i < 12; ++i) d.forward_range.push_back(i / range_size);
    }
    d.range_change_time_s = range_time;
    return d;
}

// A box in D, in gear `start`, past its dwell, ready to decide.
struct Box {
    AutoTransmission at;
    AutoTransmission::Sensors s;
    explicit Box(AutoTransmission::Desc d) : at(std::move(d)) {
        s.h = kH;
        s.throttle = 1.0;
        s.gear_request = 99.0;
        s.engine_omega = 900.0 * 2.0 * kPi / 60.0;
        s.input_omega = s.engine_omega;
        s.clutch_capacity_cap_nm = std::numeric_limits<real>::infinity();
        step_for(1.2); // engage 1st choice: the start gear
    }
    AutoTransmission::Outputs step_for(real seconds) {
        AutoTransmission::Outputs o;
        const int n = static_cast<int>(seconds / kH + 0.5);
        for (int i = 0; i < n; ++i) {
            s.first_substep = (i % 4) == 0;
            o = at.step(s);
        }
        return o;
    }
    // Road speed such that the CURRENT gear's input rpm is `rpm`.
    void set_input_rpm(real rpm) {
        const int g = at.current();
        s.output_omega = rpm * 2.0 * kPi / 60.0 / at.ratio(g);
        s.input_omega = rpm * 2.0 * kPi / 60.0;
        s.engine_omega = s.input_omega;
    }
};

std::string fixture_path() { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/tractor_powershift.json"; }

} // namespace

TEST_CASE("Powershift: the start gear is taken from neutral at a standstill", "[powershift][drivetrain][automatic]") {
    Box b(make_desc());
    CHECK(b.at.current() == 6);
    CHECK(b.at.target() == 6);
    // selector limits the start gear
    AutoTransmission::Desc d = make_desc();
    AutoTransmission at(d);
    AutoTransmission::Sensors s;
    s.h = kH;
    s.gear_request = 3.0; // D limited to gear 3
    s.clutch_capacity_cap_nm = std::numeric_limits<real>::infinity();
    for (int i = 0; i < 4000; ++i) {
        s.first_substep = (i % 4) == 0;
        (void)at.step(s);
    }
    CHECK(at.current() == 3);
}

TEST_CASE("Powershift: an upshift skips gears while the next gear stays in the band", "[powershift][drivetrain][automatic]") {
    Box b(make_desc());
    // Input rpm 2900 in gear 6: gear 7 2537, 8 2220, 9 1942, 10 1699 (< 1775 target) -> 9 is the highest at or above it.
    b.set_input_rpm(2900.0);
    const auto o = b.step_for(0.002);
    CHECK(o.target == 9);
    CHECK(o.phase != AutoTransmission::Phase::Idle);
}

TEST_CASE("Powershift: a small excess upshifts one gear only", "[powershift][drivetrain][automatic]") {
    Box b(make_desc());
    b.set_input_rpm(2300.0); // gear 7 = 2012 (>= 1775), gear 8 = 1760 (< 1775)
    const auto o = b.step_for(0.002);
    CHECK(o.target == 7);
}

TEST_CASE("Powershift: min_upshift_pull_fraction limits a skip to the pull it keeps, at least one gear", "[powershift][drivetrain][automatic]") {
    // 12 % steps: one gear keeps 0.875, two 0.766, three 0.670 of the ratio (the pull).
    for (const auto& c : {std::pair<real, int>{0.0, 9}, {0.7, 8}, {0.77, 7}, {0.86, 7}, {0.99, 7}}) {
        AutoTransmission::Desc d = make_desc();
        d.tcu.powershift->min_upshift_pull_fraction = c.first;
        Box b(d);
        b.set_input_rpm(2900.0);
        CHECK(b.step_for(0.002).target == c.second); // sabotage: the limit ignored -> 9 everywhere
    }
}

TEST_CASE("Powershift: upshift_min_time_s holds an upshift, a downshift keeps the plain dwell", "[powershift][drivetrain][automatic]") {
    AutoTransmission::Desc d = make_desc();
    d.tcu.powershift->upshift_min_time_s = 2.0;
    Box b(d); // 1.2 s in the start gear
    b.set_input_rpm(2900.0);
    CHECK(b.step_for(0.01).target == 6);
    CHECK(b.step_for(1.0).target == 9); // past 2 s in the gear
    Box c(d);
    c.set_input_rpm(1100.0); // below the down line (1400 at full throttle), dwell 0.5 s met
    CHECK(c.step_for(0.01).target < 6);
}

TEST_CASE("Powershift: no upshift inside the band and none before the dwell", "[powershift][drivetrain][automatic]") {
    Box b(make_desc());
    b.set_input_rpm(2000.0); // below the 2150 up line
    CHECK(b.step_for(0.01).target == 6);
    Box c(make_desc());
    c.set_input_rpm(2900.0);
    const auto o = c.step_for(0.002);
    REQUIRE(o.target == 9);
    // run the shift to completion with the speed following the new gear, then ask again at once
    const real w_out = c.s.output_omega;
    for (int i = 0; i < 4000 && c.at.phase() != AutoTransmission::Phase::Idle; ++i) {
        c.s.output_omega = w_out;
        c.s.input_omega = c.at.ratio(c.at.current()) * w_out;
        c.s.engine_omega = c.s.input_omega;
        c.s.first_substep = (i % 4) == 0;
        (void)c.at.step(c.s);
    }
    c.s.output_omega = w_out * 1.25; // above the up line again in gear 9
    c.s.input_omega = c.at.ratio(c.at.current()) * c.s.output_omega;
    c.s.engine_omega = c.s.input_omega;
    CHECK(c.step_for(0.01).target == c.at.current()); // dwell not yet over
}

TEST_CASE("Powershift: a downshift goes straight to the gear that restores the target rpm", "[powershift][drivetrain][automatic]") {
    Box b(make_desc());
    b.set_input_rpm(1300.0); // below the 1400 down line; gear 5 1486, 4 1698, 3 1941 (>= 1775): highest = gear 3
    const auto o = b.step_for(0.002);
    CHECK(o.target == 3);
}

TEST_CASE("Powershift: a downshift into the upshift line's reach is refused (hunting guard) but the stall guard is not", "[powershift][drivetrain][automatic]") {
    // 600-rpm band with the target near the top: every candidate lies within 5 % of the up line.
    AutoTransmission::Desc d = make_desc();
    d.tcu.schedule = {1600.0, 2000.0, 1000.0, 1900.0};
    d.tcu.powershift = PowershiftTcuDesc{6, 0.95, 4}; // target 1995 > 0.95 * up
    Box b(std::move(d));
    b.set_input_rpm(1850.0); // below dn 1900 (dwell passed)
    CHECK(b.step_for(0.002).target == 6);
}

TEST_CASE("Powershift: a range change opens both clutches, a shift inside a range does not", "[powershift][drivetrain][automatic]") {
    // 4 gears per range: gears 5-8 = range 1, 9-12 = range 2. From gear 6.
    {
        Box b(make_desc(4, 0.35));
        b.set_input_rpm(2300.0); // 6 -> 7: inside range 1
        const auto o = b.step_for(0.002);
        CHECK(o.target == 7);
        CHECK(o.phase == AutoTransmission::Phase::Torque);
    }
    {
        Box b(make_desc(4, 0.35));
        b.set_input_rpm(2900.0); // 6 -> 9: range 1 -> 2
        auto o = b.step_for(0.002);
        CHECK(o.target == 9);
        CHECK(o.phase == AutoTransmission::Phase::Range);
        CHECK(o.row[0].cap_kinetic == 0.0);
        CHECK(o.row[1].cap_kinetic == 0.0);
        // after range_change_time_s the on-coming clutch takes over (inertia phase)
        o = b.step_for(0.36);
        CHECK(o.phase == AutoTransmission::Phase::Inertia);
    }
    {
        Box b(make_desc(4, 0.0)); // time 0 = seamless
        b.set_input_rpm(2900.0);
        CHECK(b.step_for(0.002).phase == AutoTransmission::Phase::Torque);
    }
}

TEST_CASE("Powershift: the state hash does not depend on anything but the shift state", "[powershift][drivetrain][automatic]") {
    Box a(make_desc()), c(make_desc());
    a.set_input_rpm(2900.0);
    c.set_input_rpm(2900.0);
    (void)a.step_for(0.1);
    (void)c.step_for(0.1);
    CHECK(a.at.fold_state_hash(1469598103934665603ULL) == c.at.fold_state_hash(1469598103934665603ULL));
}

TEST_CASE("Powershift: the shipped box and the fixture tractor load", "[powershift][drivetrain][io]") {
    const vehicle::VehicleDesc v = io::load_vehicle_json(fixture_path());
    const GearboxDesc* gb = nullptr;
    const AutomaticTcuDesc* tcu = nullptr;
    for (const auto& c : v.powertrain.components)
        if (const auto* g = std::get_if<GearboxDesc>(&c.params)) gb = g;
    for (const auto& c : v.controllers)
        if (const auto* t = std::get_if<AutomaticTcuDesc>(&c.params)) tcu = t;
    REQUIRE(gb != nullptr);
    REQUIRE(tcu != nullptr);
    CHECK(gb->type == GearboxType::Powershift);
    CHECK(gb->forward_ratios.size() == 24);
    CHECK(gb->reverse_ratios.size() == 24);
    REQUIRE(gb->forward_range.size() == 24);
    CHECK(gb->forward_range[3] == 0);
    CHECK(gb->forward_range[4] == 1);
    CHECK(gb->range_change_time_s == 0.35);
    REQUIRE(tcu->powershift.has_value());
    CHECK(tcu->powershift->start_gear == 11);
}

namespace {
// Rewrites the fixture with `edit`, written next to a temp dir with absolute data paths, and loads it.
vehicle::VehicleDesc load_edited(const std::function<void(nlohmann::json&)>& edit_vehicle, const std::function<void(nlohmann::json&)>& edit_gearbox = {}) {
    namespace fs = std::filesystem;
    static const fs::path dir = fs::temp_directory_path() / ("ps_powershift_io_" + std::to_string(static_cast<unsigned long long>(
                                                                 std::hash<std::string>{}(std::string(PS_SOURCE_DIR)))));
    fs::create_directories(dir);
    std::ifstream f(fixture_path());
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    const std::string rel = "../../../../data/", abs = std::string(PS_SOURCE_DIR) + "/data/";
    for (std::size_t p = text.find(rel); p != std::string::npos; p = text.find(rel, p + abs.size())) text.replace(p, rel.size(), abs);
    nlohmann::json j = nlohmann::json::parse(text);
    // the gearbox goes into its own file so its keys can be edited
    std::ifstream gf(std::string(PS_SOURCE_DIR) + "/data/gearboxes/tractor_24f24r_powershift.json");
    nlohmann::json g = nlohmann::json::parse(gf);
    if (edit_gearbox) edit_gearbox(g);
    {
        std::ofstream o(dir / "gearbox.json");
        o << g.dump();
    }
    for (auto& c : j["powertrain"]["components"])
        if (c["id"] == "gearbox") c["ref"] = (dir / "gearbox.json").string();
    if (edit_vehicle) edit_vehicle(j);
    {
        std::ofstream o(dir / "vehicle.json");
        o << j.dump();
    }
    return io::load_vehicle_json((dir / "vehicle.json").string());
}

// The powertrain checks (controller vs gearbox) run when the vehicle is built.
void build_edited(const std::function<void(nlohmann::json&)>& edit_vehicle) {
    const vehicle::VehicleDesc d = load_edited(edit_vehicle);
    (void)ps_test::build_vehicle_rig_from_desc(d, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28);
}
} // namespace

TEST_CASE("Powershift: loader rejections", "[powershift][drivetrain][io]") {
    using Catch::Matchers::ContainsSubstring;
    CHECK_NOTHROW(load_edited({}));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["forward_range"] = {0, 1}; }), ContainsSubstring("forward_range"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["range_change_time_s"] = -1.0; }), ContainsSubstring("range_change_time_s"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["forward_range"][2] = 0.5; }), ContainsSubstring("integer"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["converter"] = nlohmann::json::object(); }), ContainsSubstring("converter"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["shift_clutch"]["preselect_time_s"] = 0.1; }), ContainsSubstring("preselect"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["synchro"] = nlohmann::json::object(); }), ContainsSubstring("synchro"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["initial_gear"] = 1; }), ContainsSubstring("initial_gear"));
    // the shift law
    CHECK_THROWS_WITH(build_edited([](nlohmann::json& v) { v["controllers"][0].erase("powershift"); }), ContainsSubstring("powershift"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["powershift"]["start_gear"] = 0; }), ContainsSubstring("start_gear"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["powershift"]["target_fraction"] = 1.0; }), ContainsSubstring("target_fraction"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["powershift"]["bogus"] = 1; }), ContainsSubstring("bogus"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["powershift"]["min_upshift_pull_fraction"] = 1.0; }), ContainsSubstring("min_upshift_pull_fraction"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["powershift"]["upshift_min_time_s"] = -1.0; }), ContainsSubstring("upshift_min_time_s"));

    CHECK_THROWS_WITH(build_edited([](nlohmann::json& v) { v["controllers"][0]["powershift"]["start_gear"] = 25; }), ContainsSubstring("start_gear"));
    // forward_range on another gearbox type
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["type"] = "dual_clutch"; }), ContainsSubstring("powershift"));
}

// A standing start with the starting clutch still slipping (launch_locked_ false): the input rpm of the start gear follows the output through the slipping
// clutch, so a low reading is the launch itself. With the pedal at or past kLaunchHoldThrottle the box stays in its start gear; with a light pedal the
// stall guard acts as before. Recorded sabotage: remove the `slip_launch() && !launch_locked_ && th >= kLaunchHoldThrottle` return in powershift_desired
// -> the heavy-pedal case drops to a lower gear and fails.
TEST_CASE("Powershift: a slipping standing start with the pedal down stays in the start gear; a light pedal still downshifts", "[powershift][drivetrain][automatic]") {
    auto run = [](real throttle) {
        Box b(make_desc());
        REQUIRE(b.at.current() == 6);
        b.s.throttle = throttle;
        b.s.engine_omega = 1500.0 * 2.0 * kPi / 60.0;
        b.s.input_omega = 100.0 * 2.0 * kPi / 60.0; // the clutch slips: the input shaft is far below the engine
        b.s.output_omega = 900.0 * 2.0 * kPi / 60.0 / b.at.ratio(6); // gear 6 would read 900 rpm: under the zero-throttle down line (1000)
        return b.step_for(0.01).target;
    };
    CHECK(run(0.8) == 6);
    CHECK(run(0.5) == 6);
    CHECK(run(0.25) < 6);
}
// The guard above must not hold forever: an engine that lugged below launch_anti_stall_rpm drops launch_locked_ and, below the bite rpm, cannot relock it
// (found 2026-11-15 chasing a farm slurry case; that case was the manual-box tractor.json, so this guard is hardening, covered by this unit test only). The guard gives up
// after kLaunchHoldMaxS (2 s [AS]) and the normal law takes the gear that restores the band. Recorded sabotage: kLaunchHoldMaxS huge -> the box is
// still in gear 6 after 3 s and the second CHECK fails.
TEST_CASE("Powershift: the slipping-launch guard gives up after kLaunchHoldMaxS and lets the box downshift", "[powershift][drivetrain][automatic]") {
    Box b(make_desc());
    REQUIRE(b.at.current() == 6);
    b.s.throttle = 1.0;
    b.s.engine_omega = 1100.0 * 2.0 * kPi / 60.0;
    b.s.input_omega = 1000.0 * 2.0 * kPi / 60.0; // slipping by 100 rpm: above launch_lock_slip_rpm, below the bite rpm - never relocks
    b.s.output_omega = 1100.0 * 2.0 * kPi / 60.0 / b.at.ratio(6); // gear 6 reads 1100 rpm: under the full-throttle down line (1400)
    CHECK(b.step_for(1.0).target == 6); // inside the guard's time
    CHECK(b.step_for(2.0).target < 6);  // 3 s in: the normal law downshifts
}