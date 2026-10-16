// CVT plan C1: the loader of the "cvt" gearbox type (its "cvt" variator block) and the automatic_tcu's "cvt" ratio law,
// plus the pairing rule (a cvt gearbox and the TCU's cvt block come together, checked when the powertrain is built).
// The whole-vehicle behaviour (creep, launch, ratio sweep, WOT hold, stepped mode, belt slip, determinism) is
// tests/validation/test_cvt_gearbox.cpp.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"

#include <nlohmann/json.hpp>

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
        const std::filesystem::path d = std::filesystem::temp_directory_path() / ("ps_cvt_io_" + std::to_string(rd()) + std::to_string(rd()));
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

std::string gearbox_patch(const std::string& name, const std::string& base, const std::string& patch) {
    return write_file(name + ".json", "{ \"extends\": \"" + kGearboxes + base + "\", " + patch + " }");
}

vehicle::VehicleDesc load_with_gearbox(const std::string& vehicle, const std::string& gearbox_path) {
    io::VehicleLoadOptions o;
    if (!gearbox_path.empty()) o.part_overrides["gearbox"].ref = gearbox_path;
    return io::load_vehicle_json(kFixtures + vehicle, o);
}

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

const AutomaticTcuDesc& tcu_in(const vehicle::VehicleDesc& v) {
    for (const auto& c : v.controllers)
        if (const auto* t = std::get_if<AutomaticTcuDesc>(&c.params)) return *t;
    FAIL("no automatic_tcu");
    throw 0;
}

void build(const vehicle::VehicleDesc& d) {
    (void)ps_test::build_vehicle_rig_from_desc(d, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, 1);
}

} // namespace

TEST_CASE("CVT C1 io: the shipped CVT gearboxes and fixtures load", "[io][drivetrain][automatic][cvt]") {
    for (const char* f : {"car_sedan_cvt.json", "car_sedan_cvt_clutch.json"}) {
        INFO(f);
        const auto v = io::load_vehicle_json(kFixtures + f);
        const GearboxDesc& g = gearbox_in(v);
        CHECK(g.type == GearboxType::Cvt);
        REQUIRE(g.cvt.has_value());
        CHECK(g.cvt->ratio_low == Catch::Approx(2.63));
        CHECK(g.cvt->ratio_high == Catch::Approx(0.38));
        // forward_ratios are the virtual steps, their efficiencies the variator's at each step.
        CHECK(g.forward_ratios == g.cvt->steps);
        REQUIRE(g.forward_efficiency.size() == g.forward_ratios.size());
        for (std::size_t k = 0; k < g.forward_ratios.size(); ++k)
            CHECK(g.forward_efficiency[k] == Catch::Approx(g.cvt->efficiency_vs_ratio.evaluate(g.forward_ratios[k])));
        CHECK(g.reverse_ratios.size() == 1);
        CHECK(g.converter.has_value() == (std::string(f) == "car_sedan_cvt.json"));
        const AutomaticTcuDesc& t = tcu_in(v);
        REQUIRE(t.cvt.has_value());
        CHECK(t.cvt->stepped_sport);
        CHECK(t.cvt->target_rpm_vs_throttle.evaluate(1.0) == Catch::Approx(5600.0));
        REQUIRE_NOTHROW(build(v));
    }
}

TEST_CASE("CVT C1 io: cvt gearbox rejections", "[io][drivetrain][automatic][cvt]") {
    auto rejects = [](const std::string& name, const std::string& patch, const std::string& msg) {
        INFO(name);
        CHECK_THROWS_WITH(load_with_gearbox("car_sedan_cvt.json", gearbox_patch(name, "sedan_cvt.json", patch)), ContainsSubstring(msg));
    };
    rejects("cvt_fwd", "\"forward_ratios\": [2.0, 1.0], \"forward_efficiency\": [0.9, 0.9]", "a cvt gearbox has no 'forward_ratios'");
    rejects("cvt_missing", "\"cvt\": null", "'cvt' is required for a cvt gearbox");
    rejects("cvt_unknown", "\"cvt\": {\"chain\": true}", "unknown key 'chain'");
    rejects("cvt_order", "\"cvt\": {\"ratio_low\": 0.3}", "need ratio_low > ratio_high > 0");
    rejects("cvt_rate", "\"cvt\": {\"ratio_rate_per_s\": 0}", "ratio_rate_per_s and step_rate_per_s must be > 0");
    rejects("cvt_drag", "\"cvt\": {\"pump_drag_nm\": -1}", "pump drag terms must be >= 0");
    rejects("cvt_steps_one", "\"cvt\": {\"steps\": [1.0]}", "needs at least two virtual steps");
    rejects("cvt_steps_range", "\"cvt\": {\"steps\": [3.0, 1.0]}", "every step must lie within [ratio_high, ratio_low]");
    rejects("cvt_steps_order", "\"cvt\": {\"steps\": [1.0, 2.0]}", "must be strictly decreasing");
    rejects("cvt_eff", "\"cvt\": {\"efficiency_vs_ratio\": {\"x\": [0.38, 2.63], \"y\": [0.9, 1.2]}}", "every efficiency must be in (0, 1]");
    rejects("cvt_belt_mu", "\"cvt\": {\"belt\": {\"friction_coefficient\": 0}}", "need friction_coefficient > 0");
    rejects("cvt_belt_unknown", "\"cvt\": {\"belt\": {\"links\": 400}}", "unknown key 'links'");
    rejects("cvt_two_reverse", "\"reverse_ratios\": [-2.6, -1.5], \"reverse_efficiency\": [0.85, 0.85]", "exactly one reverse ratio");
    rejects("cvt_preselect", "\"shift_clutch\": {\"preselect_time_s\": 0.1}", "preselect_time_s is for dual_clutch gearboxes only");
    // A cvt block on another gearbox type.
    INFO("automatic with a cvt block");
    CHECK_THROWS_WITH(load_with_gearbox("car_sedan_at.json", gearbox_patch("at_cvt", "sedan_8at.json", "\"cvt\": {\"ratio_low\": 2.6}")),
                      ContainsSubstring("'cvt' is for \"cvt\" gearboxes only"));
}

TEST_CASE("CVT C1 io: the automatic_tcu cvt law and the pairing rule", "[io][drivetrain][automatic][cvt]") {
    auto rejects = [](const std::string& name, const std::function<void(nlohmann::json&)>& edit, const std::string& msg) {
        INFO(name);
        CHECK_THROWS_WITH(io::load_vehicle_json(vehicle_copy(name, "car_sedan_cvt.json", edit)), ContainsSubstring(msg));
    };
    REQUIRE_NOTHROW(io::load_vehicle_json(vehicle_copy("cvt_ok", "car_sedan_cvt.json", [](nlohmann::json&) {})));
    rejects("law_unknown", [](nlohmann::json& v) { tcu_of(v)["cvt"]["belt"] = 1; }, "unknown key 'belt'");
    rejects("law_missing", [](nlohmann::json& v) { tcu_of(v)["cvt"].erase("target_rpm_vs_throttle"); }, "'target_rpm_vs_throttle' is required");
    rejects("law_rpm", [](nlohmann::json& v) { tcu_of(v)["cvt"]["target_rpm_vs_throttle"] = {{"x", {0.0, 1.0}}, {"y", {1300.0, -1.0}}}; },
            "every rpm must be > 0");
    rejects("law_x", [](nlohmann::json& v) { tcu_of(v)["cvt"]["target_rpm_vs_throttle"] = {{"x", {0.0, 1.5}}, {"y", {1300.0, 5600.0}}}; },
            "throttle x must lie within [0, 1]");
    rejects("law_stepped", [](nlohmann::json& v) { tcu_of(v)["cvt"]["stepped_sport"] = 1; }, "'stepped_sport' must be a boolean");

    // Pairing (checked at powertrain build): a cvt gearbox without the law, and the law on a stepped automatic.
    auto no_law = io::load_vehicle_json(vehicle_copy("cvt_no_law", "car_sedan_cvt.json", [](nlohmann::json& v) { tcu_of(v).erase("cvt"); }));
    CHECK_THROWS_WITH(build(no_law), ContainsSubstring("a cvt gearbox needs the automatic_tcu's 'cvt' block"));
    auto at = io::load_vehicle_json(kFixtures + "car_sedan_at.json");
    for (auto& c : at.controllers)
        if (auto* t = std::get_if<AutomaticTcuDesc>(&c.params)) t->cvt = tcu_in(io::load_vehicle_json(kFixtures + "car_sedan_cvt.json")).cvt;
    CHECK_THROWS_WITH(build(at), ContainsSubstring("the automatic_tcu's 'cvt' block is for a cvt gearbox only"));
}
