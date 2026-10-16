// Automatic gearbox plan A1: the loader of the "automatic" / "dual_clutch" gearbox types and the automatic_tcu
// controller, plus the torque converter's evaluate() in isolation (core/src/drivetrain/torque_converter.h).
// The whole-vehicle behaviour (creep, stall speed, shifts, lock-up, DCT launch, determinism) is
// tests/validation/test_automatic_gearbox.cpp.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "drivetrain/torque_converter.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <random>
#include <string>
#include <variant>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;

namespace {

const std::string kRoot = PS_SOURCE_DIR;
const std::string kFixtures = kRoot + "/tests/support/fixtures/vehicles/";
const std::string kGearboxes = kRoot + "/data/gearboxes/";

const std::filesystem::path& temp_dir() {
    static const std::filesystem::path p = [] {
        std::random_device rd;
        const std::filesystem::path d = std::filesystem::temp_directory_path() / ("ps_autobox_io_" + std::to_string(rd()) + std::to_string(rd()));
        std::filesystem::create_directories(d);
        return d;
    }();
    return p;
}

std::string write_file(const std::string& name, const std::string& body) {
    const std::filesystem::path f = temp_dir() / name;
    std::ofstream o(f);
    o << body;
    return f.string();
}

// A gearbox part extending `base` (a data/gearboxes file) with `patch` (inside of a JSON object, no braces).
std::string gearbox_patch(const std::string& name, const std::string& base, const std::string& patch) {
    return write_file(name + ".json", "{ \"extends\": \"" + kGearboxes + base + "\", " + patch + " }");
}

vehicle::VehicleDesc load_with_gearbox(const std::string& vehicle, const std::string& gearbox_path) {
    io::VehicleLoadOptions o;
    if (!gearbox_path.empty()) o.part_overrides["gearbox"].ref = gearbox_path;
    return io::load_vehicle_json(kFixtures + vehicle, o);
}

// A copy of a fixture vehicle with its relative part paths made absolute, edited by `edit`, written to the temp dir.
std::string vehicle_copy(const std::string& name, const std::string& vehicle, const std::function<void(nlohmann::json&)>& edit) {
    std::ifstream in(kFixtures + vehicle);
    nlohmann::json j = nlohmann::json::parse(in);
    std::function<void(nlohmann::json&)> absolutise = [&](nlohmann::json& v) {
        if (v.is_string()) {
            const std::string s = v.get<std::string>();
            if (s.rfind("../", 0) == 0) v = (std::filesystem::path(kFixtures) / s).lexically_normal().string();
        } else if (v.is_object() || v.is_array()) {
            for (auto& c : v) absolutise(c);
        }
    };
    absolutise(j);
    edit(j);
    return write_file(name + ".json", j.dump(2));
}

nlohmann::json& tcu_of(nlohmann::json& v) {
    for (auto& c : v.at("controllers"))
        if (c.value("type", "") == "automatic_tcu") return c;
    FAIL("no automatic_tcu");
    throw 0;
}

const GearboxDesc& gearbox_in(const vehicle::VehicleDesc& v) {
    for (const ComponentDesc& c : v.powertrain.components)
        if (const auto* g = std::get_if<GearboxDesc>(&c.params)) return *g;
    FAIL("no gearbox component");
    throw 0;
}

TorqueConverterDesc converter_desc() {
    TorqueConverterDesc d;
    d.k_factor.x = {0.0, 0.5, 0.8, 0.9};
    d.k_factor.y = {180.0, 190.0, 220.0, 280.0};
    d.torque_ratio.x = {0.0, 0.5, 0.85, 1.0};
    d.torque_ratio.y = {2.0, 1.5, 1.0, 1.0};
    d.pump_inertia_kgm2 = 0.08;
    d.turbine_inertia_kgm2 = 0.03;
    d.lockup_capacity_nm = 400.0;
    return d;
}

constexpr real kToRadPerSec = real(2.0) * real(3.14159265358979323846) / real(60.0);

} // namespace

TEST_CASE("Automatic A1 io: the shipped AT and DCT fixtures load with their converter, shift clutches and TCU", "[io][drivetrain][automatic]") {
    const auto at = io::load_vehicle_json(kFixtures + "car_sedan_at.json");
    const GearboxDesc& g = gearbox_in(at);
    CHECK(g.type == GearboxType::Automatic);
    CHECK(g.forward_ratios.size() == 8);
    REQUIRE(g.converter.has_value());
    CHECK(g.converter->lockup_capacity_nm == Catch::Approx(400.0));
    CHECK(g.converter->k_factor.y.front() == Catch::Approx(179.0));
    CHECK(g.shift_clutch.capacity_nm == Catch::Approx(400.0));
    bool has_tcu = false;
    for (const auto& c : at.controllers)
        if (std::holds_alternative<AutomaticTcuDesc>(c.params)) has_tcu = true;
    CHECK(has_tcu);

    const auto dct = io::load_vehicle_json(kFixtures + "car_sedan_dct.json");
    const GearboxDesc& d = gearbox_in(dct);
    CHECK(d.type == GearboxType::DualClutch);
    CHECK(d.forward_ratios.size() == 7);
    CHECK(d.shift_clutch.preselect_time_s == Catch::Approx(0.12));
}

TEST_CASE("Automatic A1 io: gearbox rejections", "[io][drivetrain][automatic]") {
    auto rejects = [](const std::string& vehicle, const std::string& name, const std::string& base, const std::string& patch,
                      const std::string& msg) {
        INFO(name);
        CHECK_THROWS_WITH(load_with_gearbox(vehicle, gearbox_patch(name, base, patch)), ContainsSubstring(msg));
    };
    // Fields of the other kinds.
    rejects("car_sedan_at.json", "at_synchro", "sedan_8at.json", "\"synchro\": {\"capacity_nm\": 100}", "no 'synchro' block");
    rejects("car_sedan_at.json", "at_preselect", "sedan_8at.json", "\"shift_clutch\": {\"preselect_time_s\": 0.1}",
            "preselect_time_s is for dual_clutch gearboxes only");
    rejects("car_sedan_at.json", "at_no_converter", "sedan_8at.json", "\"converter\": null", "'converter' is required");
    rejects("car_sedan_dct.json", "dct_converter", "sedan_7dct.json",
            "\"converter\": {\"k_factor\": {\"x\": [0], \"y\": [180]}, \"torque_ratio\": {\"x\": [0, 1], \"y\": [2, 1]}}",
            "a dual_clutch gearbox has no 'converter'");
    rejects("car_sedan_dct.json", "dct_no_clutch", "sedan_7dct.json", "\"shift_clutch\": null", "'shift_clutch' is required");
    rejects("car_sedan_dct.json", "dct_initial", "sedan_7dct.json", "\"initial_gear\": 1", "starts in N");
    rejects("car_sedan_dct.json", "dct_capacity", "sedan_7dct.json", "\"shift_clutch\": {\"capacity_nm\": 0}", "capacity_nm must be > 0");
    rejects("car_sedan_dct.json", "dct_unknown", "sedan_7dct.json", "\"shift_clutch\": {\"pressure_bar\": 3}", "unknown key 'pressure_bar'");
    // Converter curves.
    rejects("car_sedan_at.json", "k_start", "sedan_8at.json", "\"converter\": {\"k_factor\": {\"x\": [0.1, 0.5], \"y\": [180, 190]}}",
            "k_factor: x must start at 0");
    rejects("car_sedan_at.json", "k_one", "sedan_8at.json", "\"converter\": {\"k_factor\": {\"x\": [0, 1.0], \"y\": [180, 190]}}",
            "stay below 1");
    rejects("car_sedan_at.json", "k_neg", "sedan_8at.json", "\"converter\": {\"k_factor\": {\"x\": [0, 0.5], \"y\": [180, -1]}}",
            "every K must be > 0");
    rejects("car_sedan_at.json", "tr_below_one", "sedan_8at.json", "\"converter\": {\"torque_ratio\": {\"x\": [0, 1], \"y\": [2, 0.9]}}",
            "every ratio must be >= 1");
    rejects("car_sedan_at.json", "tr_eff", "sedan_8at.json", "\"converter\": {\"torque_ratio\": {\"x\": [0, 0.8, 1], \"y\": [2, 1.5, 1]}}",
            "TR(SR) * SR must be <= 1");
    rejects("car_sedan_at.json", "tr_rising", "sedan_8at.json",
            "\"converter\": {\"torque_ratio\": {\"x\": [0, 0.3, 0.5, 1], \"y\": [1.5, 1.8, 1.2, 1]}}", "non-increasing");
    rejects("car_sedan_at.json", "conv_unknown", "sedan_8at.json", "\"converter\": {\"stator\": 1}", "unknown key 'stator'");
    rejects("car_sedan_at.json", "lockup_cap", "sedan_8at.json", "\"converter\": {\"lockup_capacity_nm\": 0}", "lockup_capacity_nm must be > 0");
    // A manual box may not carry the clutch-to-clutch blocks.
    INFO("manual with shift_clutch");
    CHECK_THROWS_WITH(load_with_gearbox("car_sedan_at.json", gearbox_patch("manual_sc", "sedan_6mt.json", "\"shift_clutch\": {\"capacity_nm\": 300}")),
                      ContainsSubstring("for \"automatic\" and \"dual_clutch\" gearboxes only"));
}

TEST_CASE("Automatic A1 io: automatic_tcu rejections", "[io][drivetrain][automatic]") {
    auto rejects = [](const std::string& name, const std::function<void(nlohmann::json&)>& edit, const std::string& msg) {
        INFO(name);
        CHECK_THROWS_WITH(io::load_vehicle_json(vehicle_copy(name, "car_sedan_dct.json", edit)), ContainsSubstring(msg));
    };
    // The copy itself loads (the absolutised paths are right).
    REQUIRE_NOTHROW(io::load_vehicle_json(vehicle_copy("tcu_ok", "car_sedan_dct.json", [](nlohmann::json&) {})));
    rejects("tcu_unknown", [](nlohmann::json& v) { tcu_of(v)["shift_points"] = 1; }, "unknown key 'shift_points'");
    rejects("tcu_gearbox_ref", [](nlohmann::json& v) { tcu_of(v)["gearbox"] = "nope"; }, "references unknown component 'nope'");
    rejects("tcu_down_above_up", [](nlohmann::json& v) { tcu_of(v)["schedule"]["down_rpm_at_zero_throttle"] = 2500.0; },
            "every downshift rpm must be > 0 and below the upshift rpm");
    rejects("tcu_fraction", [](nlohmann::json& v) { tcu_of(v)["upshift_torque_fraction"] = 0.0; }, "upshift_torque_fraction must be in (0, 1]");
    rejects("tcu_kickdown", [](nlohmann::json& v) { tcu_of(v)["kickdown_throttle"] = 1.5; }, "kickdown_throttle must be in [0, 1]");
    rejects("tcu_launch", [](nlohmann::json& v) { tcu_of(v)["launch"]["bite_rpm"] = 2000.0; }, "0 < bite_rpm < full_rpm_min");
    rejects("tcu_two", [](nlohmann::json& v) {
        auto& cs = v.at("controllers");
        cs.push_back(tcu_of(v));
    }, "at most one controller");
}

TEST_CASE("Automatic A1: torque converter evaluate - stall law, coupling, overrun and the implicit slope", "[drivetrain][automatic]") {
    const TorqueConverterDesc d = converter_desc();
    const TorqueConverter tc(d);
    // Stall (turbine at rest): pump torque = (n/K0)^2, turbine = TR0 times it.
    {
        const real n = 2000.0, w = n * kToRadPerSec;
        const auto r = tc.evaluate(w, 0.0);
        const real tp = (n / 180.0) * (n / 180.0);
        CHECK(r.tau_pump == Catch::Approx(-tp).epsilon(1e-12));
        CHECK(r.tau_turbine == Catch::Approx(2.0 * tp).epsilon(1e-12));
        CHECK_FALSE(r.overrun);
    }
    // Coupling region (SR above the last K point): capacity falls linearly to 0 at SR = 1, TR = 1.
    {
        const real w = 3000.0 * kToRadPerSec;
        const auto r = tc.evaluate(w, 0.95 * w);
        CHECK(r.speed_ratio == Catch::Approx(0.95));
        CHECK(r.tau_turbine == Catch::Approx(-r.tau_pump).epsilon(1e-12));
        CHECK(r.tau_pump < 0.0);
        const auto r1 = tc.evaluate(w, w);
        CHECK(r1.tau_pump == Catch::Approx(0.0).margin(1e-12));
        CHECK(tc.capacity(0.95) == Catch::Approx(0.5 / (280.0 * 280.0)).epsilon(1e-12));
    }
    // Overrun (turbine faster: coasting): the turbine drives the pump, no multiplication, opposite signs.
    {
        const real w = 2000.0 * kToRadPerSec;
        const auto r = tc.evaluate(0.6 * w, w);
        CHECK(r.overrun);
        CHECK(r.tau_turbine < 0.0);
        CHECK(r.tau_pump == Catch::Approx(-r.tau_turbine).epsilon(1e-12));
    }
    // Efficiency never above 1: turbine power <= pump power in forward flow.
    for (real sr : {0.0, 0.2, 0.5, 0.7, 0.85, 0.9, 0.99}) {
        INFO(sr);
        const real w = 2500.0 * kToRadPerSec;
        const auto r = tc.evaluate(w, sr * w);
        CHECK(r.tau_turbine * sr * w <= -r.tau_pump * w * (1.0 + 1e-12));
    }
    // slip_slope = |dT_pump/dw_turbine| against a central difference inside a K segment.
    {
        const real w = 2500.0 * kToRadPerSec, wt = 0.3 * w, dw = 1e-4 * w;
        const real fd = (tc.evaluate(w, wt + dw).tau_pump - tc.evaluate(w, wt - dw).tau_pump) / (2.0 * dw);
        CHECK(tc.evaluate(w, wt).slip_slope == Catch::Approx(std::abs(fd)).epsilon(1e-6));
    }
    // Both at rest: no torque.
    const auto z = tc.evaluate(0.0, 0.0);
    CHECK(z.tau_pump == 0.0);
    CHECK(z.tau_turbine == 0.0);
}
