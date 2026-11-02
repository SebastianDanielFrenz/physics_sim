// tests/unit/test_hydro_cvt_gearbox.cpp - tractor plan stage 2: the hydro-mechanical CVT (GearboxType::HydroCvt) on its own
// (AutoTransmission white-box) and its data loading.
//  * hydro_cvt_efficiency: 0.97 at the mechanical point, 0.85 at k = 0 (all hydrostatic), symmetric about it;
//  * the driver law: the engine rpm target follows the demanded power along the data's curve, the ratio k is the speed
//    reference over the engine speed (0 at rest = the output is held), is rate limited, goes negative only in R, and
//    falls to 0 below the stall band;
//  * N and the direction clutch filling follow the measured ratio and ask no engine command;
//  * loader: the shipped box and the fixture load, every rejection message;
//  * the state hash only moves with the state.
// Recorded sabotages (2026-11-02), each makes the named case fail:
//   S1 k_cmd without the stall fade f: "stall band" fails.
//   S2 the ratio rate limit removed: "rate limited" fails.
//   S3 hydro_cvt_efficiency without the hydrostatic term: "efficiency" fails.
//   S4 reverse not clamped to <= 0: "direction" fails.

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "vehicle_test_helpers.h"

#include "../../core/src/drivetrain/auto_transmission.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;

namespace {
constexpr real kRpm = real(2.0) * real(3.14159265358979323846) / real(60.0);

std::string fixture_path() { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/tractor_hydro.json"; }

AutoTransmission::Desc make_desc() {
    const vehicle::VehicleDesc d = io::load_vehicle_json(fixture_path());
    AutoTransmission::Desc ad;
    ad.hydro = true;
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<GearboxDesc>(&c.params)) {
            ad.hydro_desc = *g->hydro;
            ad.forward_ratios = g->forward_ratios;
            ad.reverse_ratios = g->reverse_ratios;
            ad.clutch = g->shift_clutch;
        }
    }
    for (const auto& c : d.controllers)
        if (const auto* t = std::get_if<AutomaticTcuDesc>(&c.params)) ad.tcu = *t;
    return ad;
}

struct Drive {
    AutoTransmission at;
    AutoTransmission::Sensors s;
    AutoTransmission::Outputs o;
    Drive() : at(make_desc()) {
        s.h = 1.0 / 960.0;
        s.first_substep = true;
        s.engine_omega = s.input_omega = 1500.0 * kRpm;
    }
    void select(real gear_request) {
        s.gear_request = gear_request;
        run(1.0); // engage
    }
    void run(real seconds) {
        const int n = static_cast<int>(seconds / s.h);
        for (int i = 0; i < n; ++i) {
            o = at.step(s);
            s.first_substep = (i % 4 == 3);
        }
    }
};
} // namespace

TEST_CASE("Hydro CVT: efficiency follows the hydrostatic share of the power flow", "[hydro_cvt][drivetrain]") {
    HydroCvtDesc d;
    d.mechanical_point_ratio = 0.9;
    d.hydrostatic_efficiency = 0.85;
    d.mechanical_efficiency = 0.97;
    CHECK(hydro_cvt_efficiency(d, 0.9) == Approx(0.97).margin(1e-12));
    CHECK(hydro_cvt_efficiency(d, 0.0) == Approx(0.85).margin(1e-12)); // all hydrostatic at standstill
    CHECK(hydro_cvt_efficiency(d, 1.8) == Approx(0.85).margin(1e-12)); // and again at twice the mechanical point
    CHECK(hydro_cvt_efficiency(d, 0.45) == Approx(1.0 / (0.5 / 0.97 + 0.5 / 0.85)).margin(1e-12));
    CHECK(hydro_cvt_efficiency(d, -0.5) == Approx(0.85).margin(1e-12)); // reverse is hydrostatic
    CHECK(hydro_cvt_efficiency(d, 0.8) > hydro_cvt_efficiency(d, 0.4));
}

TEST_CASE("Hydro CVT: the driver law sets the engine target, the ratio, the direction", "[hydro_cvt][drivetrain]") {
    Drive dr;
    // N: no engine command, the ratio follows the measured speeds
    dr.s.output_omega = 300.0 * kRpm;
    dr.s.throttle = 0.5;
    dr.run(0.2);
    CHECK(dr.o.engine_command < 0.0);
    CHECK(dr.o.current == 0);
    dr.s.output_omega = 0.0;
    dr.s.gear_request = 99.0;
    dr.run(1.0);
    REQUIRE(dr.o.current == 1);
    CHECK(dr.o.row[0].cap_kinetic == Approx(4500.0)); // the hydrostatic pressure limit once the direction clutch is full

    SECTION("a command at rest: the engine goes up the curve, k rises rate limited, never negative in D") {
        dr.s.throttle = 1.0;
        real k_prev = dr.o.cvt_ratio;
        real worst_step = 0.0;
        real k_max = 0.0;
        for (int i = 0; i < 960 * 3; ++i) {
            dr.s.output_omega = dr.o.cvt_ratio * dr.s.input_omega; // the row holds: w_out = k w_in
            dr.o = dr.at.step(dr.s);
            worst_step = std::max(worst_step, std::abs(dr.o.cvt_ratio - k_prev));
            k_prev = dr.o.cvt_ratio;
            k_max = std::min(k_max, dr.o.cvt_ratio);
        }
        CHECK(k_max >= 0.0);
        CHECK(k_prev > 0.0);
        CHECK(dr.o.engine_command > 0.1);
        CHECK(worst_step <= 0.8 / 960.0 * 1.0001); // rate limited [S2]
        CHECK(dr.o.engine_target_rpm >= 1000.0);
        // a sudden jump of the engine speed (the reference over the engine speed halves): the ratio still moves at its rate limit
        const real k_before = dr.o.cvt_ratio;
        dr.s.input_omega = dr.s.engine_omega = 3000.0 * kRpm;
        dr.o = dr.at.step(dr.s);
        CHECK(std::abs(dr.o.cvt_ratio - k_before) <= 0.8 / 960.0 * 1.0001); // [S2]
    }
    SECTION("zero command at rest: ratio 0 (the output is held), engine on the curve's first point") {
        dr.s.throttle = 0.0;
        dr.run(5.0);
        CHECK(dr.o.cvt_ratio == Approx(0.0).margin(1e-3));
        CHECK(dr.o.engine_target_rpm == Approx(1000.0).margin(30.0));
        CHECK(dr.o.engine_command >= 0.0);
        CHECK(dr.o.engine_command < 0.3);
    }
    SECTION("stall band: below stall_rpm_min the ratio goes to 0 [S1]") {
        dr.s.throttle = 1.0;
        dr.s.output_omega = 0.0;
        dr.s.engine_omega = dr.s.input_omega = 800.0 * kRpm; // under stall_rpm_min 900
        dr.run(3.0);
        CHECK(dr.o.cvt_ratio == Approx(0.0).margin(1e-9));
        dr.s.engine_omega = dr.s.input_omega = 1500.0 * kRpm;
        dr.run(3.0);
        CHECK(dr.o.cvt_ratio > 0.0);
    }
    SECTION("high power demand asks for a higher engine target than a light one") {
        dr.s.throttle = 0.1;
        dr.s.output_omega = 0.0;
        dr.s.input_torque_prev = 0.0;
        dr.run(2.0);
        const real light = dr.o.engine_target_rpm;
        dr.s.throttle = 1.0;
        dr.run(2.0);
        CHECK(dr.o.engine_target_rpm > light + 100.0);
    }
}

TEST_CASE("Hydro CVT: direction - R gives a negative ratio, never positive", "[hydro_cvt][drivetrain]") {
    Drive dr;
    dr.s.gear_request = -1.0;
    dr.s.output_omega = 0.0;
    dr.run(1.0);
    REQUIRE(dr.o.current == -1);
    dr.s.throttle = 1.0;
    real k_hi = -10.0, k_lo = 10.0;
    for (int i = 0; i < 960 * 6; ++i) {
        dr.s.output_omega = dr.o.cvt_ratio * dr.s.input_omega;
        dr.o = dr.at.step(dr.s);
        k_hi = std::max(k_hi, dr.o.cvt_ratio);
        k_lo = std::min(k_lo, dr.o.cvt_ratio);
    }
    CHECK(k_hi <= 1e-12); // [S4]
    CHECK(k_lo < -0.05);
    CHECK(k_lo >= -0.5 - 1e-12); // ratio_reverse_max
}

TEST_CASE("Hydro CVT: the state hash moves with the state only", "[hydro_cvt][drivetrain][determinism]") {
    Drive a, b;
    a.select(99.0);
    b.select(99.0);
    a.s.throttle = b.s.throttle = 0.6;
    a.run(0.5);
    b.run(0.5);
    CHECK(a.at.fold_state_hash(1) == b.at.fold_state_hash(1));
    b.run(0.5);
    CHECK(a.at.fold_state_hash(1) != b.at.fold_state_hash(1));
}

namespace {
vehicle::VehicleDesc load_edited(const std::function<void(nlohmann::json&)>& edit_vehicle, const std::function<void(nlohmann::json&)>& edit_gearbox = {}) {
    namespace fs = std::filesystem;
    static const fs::path dir = fs::temp_directory_path() / ("ps_hydro_io_" + std::to_string(static_cast<unsigned long long>(
                                                                 std::hash<std::string>{}(std::string(PS_SOURCE_DIR)))));
    fs::create_directories(dir);
    std::ifstream f(fixture_path());
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    const std::string rel = "../../../../data/", abs = std::string(PS_SOURCE_DIR) + "/data/";
    for (std::size_t p = text.find(rel); p != std::string::npos; p = text.find(rel, p + abs.size())) text.replace(p, rel.size(), abs);
    nlohmann::json j = nlohmann::json::parse(text);
    std::ifstream gf(std::string(PS_SOURCE_DIR) + "/data/gearboxes/tractor_hydro_cvt.json");
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
void build_edited(const std::function<void(nlohmann::json&)>& edit_vehicle) {
    const vehicle::VehicleDesc d = load_edited(edit_vehicle);
    (void)ps_test::build_vehicle_rig_from_desc(d, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28);
}
} // namespace

TEST_CASE("Hydro CVT: the shipped box and the fixture load", "[hydro_cvt][drivetrain][io]") {
    const vehicle::VehicleDesc d = io::load_vehicle_json(fixture_path());
    const GearboxDesc* gb = nullptr;
    const AutomaticTcuDesc* tcu = nullptr;
    for (const auto& c : d.powertrain.components)
        if (const auto* g = std::get_if<GearboxDesc>(&c.params)) gb = g;
    for (const auto& c : d.controllers)
        if (const auto* t = std::get_if<AutomaticTcuDesc>(&c.params)) tcu = t;
    REQUIRE(gb != nullptr);
    REQUIRE(tcu != nullptr);
    CHECK(gb->type == GearboxType::HydroCvt);
    REQUIRE(gb->hydro.has_value());
    CHECK(gb->hydro->ratio_forward_max == 1.35);
    CHECK(gb->hydro->mechanical_point_ratio == 0.9);
    CHECK(gb->shift_clutch.capacity_nm == 4500.0);
    CHECK(gb->forward_ratios.size() == 1);
    CHECK(gb->reverse_ratios.size() == 1);
    REQUIRE(tcu->hydro.has_value());
    CHECK(tcu->hydro->speed_rpm_vs_command.evaluate(1.0) == 2970.0);
}

TEST_CASE("Hydro CVT: loader rejections", "[hydro_cvt][drivetrain][io]") {
    using Catch::Matchers::ContainsSubstring;
    CHECK_NOTHROW(load_edited({}));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g.erase("hydro"); }), ContainsSubstring("hydro"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["forward_ratios"] = {1.0}; }), ContainsSubstring("forward_ratios"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["hydro"]["bogus"] = 1; }), ContainsSubstring("bogus"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["hydro"]["hydrostatic_efficiency"] = 1.2; }), ContainsSubstring("efficiency"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["hydro"]["ratio_forward_max"] = 0.0; }), ContainsSubstring("ratio_forward_max"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["converter"] = nlohmann::json::object(); }), ContainsSubstring("converter"));
    CHECK_THROWS_WITH(load_edited({}, [](nlohmann::json& g) { g["type"] = "powershift"; }), ContainsSubstring("hydro"));
    // the driver law
    CHECK_THROWS_WITH(build_edited([](nlohmann::json& v) { v["controllers"][0].erase("hydro"); }), ContainsSubstring("hydro"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["hydro"]["bogus"] = 1; }), ContainsSubstring("bogus"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["hydro"].erase("engine_rpm_vs_power"); }),
                      ContainsSubstring("engine_rpm_vs_power"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["hydro"]["stall_rpm_ok"] = 800.0; }), ContainsSubstring("stall_rpm"));
    CHECK_THROWS_WITH(load_edited([](nlohmann::json& v) { v["controllers"][0]["hydro"]["speed_rpm_vs_command"]["x"] = {0.0, 0.5}; v["controllers"][0]["hydro"]["speed_rpm_vs_command"]["y"] = {0.0, 1000.0}; }),
                      ContainsSubstring("span 0..1"));
}
