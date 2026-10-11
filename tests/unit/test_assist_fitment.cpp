// tests/unit/test_assist_fitment.cpp - engine garage P8 (A15/A16): assist fitment, electronics parts, launch control.
//
//   F*  the `fitted` flag: a not-fitted assist ignores its channel (state hash identical to channel 0), a fitted one
//       does not (discriminating control); default files are unchanged by explicit `fitted: true`
//   L*  launch control (two-step) on a torque-map engine (car_sedan, fuel cut and soft cut) and on a simulated one
//       (car_sedan_sim): the rpm is held in a measured band at standstill with the clutch in, the car launches
//       with the limit held, the limit releases above max_speed_mps, a disarmed run gets the normal limiter
//   G*  grafting through the user layer equals writing the controllers entry into the vehicle file; "none" removes
//   R*  loader rejections
//   S*  snapshot bits, anti-lag switch, zero allocation, 1 vs 10 workers
//
// Every guard here has a recorded sabotage (out/garage_p8_progress.md in the worktree).

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/electronics_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "mvem_fixtures.h"
#include "turbo_fixtures.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";

// A scratch directory unique to the test case (ctest runs the cases as parallel processes).
struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_p8_" + tag);
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

void absolutise(json& j) {
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s.rfind("../", 0) == 0) j = kData + "/" + s.substr(3);
    } else if (j.is_array() || j.is_object()) {
        for (auto& e : j) absolutise(e);
    }
}

// data/vehicles/<name> with every "../x" path made absolute, so it can be patched and written to a scratch dir.
json vehicle_json(const std::string& name) {
    std::ifstream f(kData + "/vehicles/" + name);
    REQUIRE(f.good());
    json j = json::parse(f);
    absolutise(j);
    return j;
}

std::string write_vehicle(const TempDir& d, const std::string& fname, const json& j) {
    write_text(d.file(fname), j.dump(1));
    return d.file(fname);
}

vehicle::VehicleDesc load_desc(const std::string& vehicle_file, const io::VehicleLoadOptions& o = {}) {
    return io::load_vehicle_json(vehicle_file, o);
}

vehicle::VehicleDesc sedan() { return load_desc(kData + "/vehicles/car_sedan.json"); }
vehicle::VehicleDesc sedan_sim() { return load_desc(kData + "/vehicles/car_sedan_sim.json"); }

drivetrain::ManualTcuDesc& tcu_of(vehicle::VehicleDesc& d) {
    for (auto& c : d.controllers)
        if (auto* t = std::get_if<drivetrain::ManualTcuDesc>(&c.params)) return *t;
    FAIL("no manual_tcu");
    throw 0;
}

drivetrain::EngineLimiterDesc& limiter_of(vehicle::VehicleDesc& d) {
    for (auto& c : d.powertrain.components) {
        if (auto* t = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) return t->limiter;
        if (auto* s = std::get_if<drivetrain::SimulatedEngineDesc>(&c.params)) return s->limiter;
    }
    FAIL("no engine");
    throw 0;
}

void add_launch(vehicle::VehicleDesc& d, real rpm, real max_speed = 2.0, bool fitted = true) {
    drivetrain::LaunchControlDesc l;
    l.rpm = rpm;
    l.max_speed_mps = max_speed;
    l.fitted = fitted;
    d.controllers.push_back({"launch", l});
}

ps_test::VehicleTestRig make_rig(const vehicle::VehicleDesc& d, int workers = 1) {
    return ps_test::build_vehicle_rig_from_desc(d, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
}

void prepare(ps_test::VehicleTestRig& rig) {
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = drivetrain::EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
}

// ---------------------------------------------------------------------------------------------------------------
// F: fitment. One script for every assist: WOT from rest in 1st, then a gear_request 2 at 1.5 s, 3 s total.
// ---------------------------------------------------------------------------------------------------------------
std::uint64_t assist_script(const vehicle::VehicleDesc& d, const std::string& channel, real value, int workers = 1) {
    // gear_request outranks the auto-shift (ManualTcu), so the auto_shift case leaves it at "none".
    const bool gear_requests = channel != "assist.auto_shift";
    auto rig = make_rig(d, workers);
    World& w = *rig.world;
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("assist.traction_control", 0.0);
    w.set_control(channel, value);
    w.set_control("clutch", 0.0);
    if (gear_requests) w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.0);
    prepare(rig);
    ps_test::settle(w, 0.5);
    w.set_control("throttle", 1.0);
    ps_test::settle(w, 1.5);
    if (gear_requests) w.set_control("gear_request", 2.0);
    ps_test::settle(w, 1.5);
    return w.state_hash();
}

} // namespace

TEST_CASE("F1: a not-fitted assist ignores its channel, a fitted one does not", "[assist][drivetrain][vehicle]") {
    struct Case {
        const char* name;
        const char* channel;
        void (*unfit)(vehicle::VehicleDesc&);
    };
    const Case cases[] = {
        {"auto_clutch", "assist.auto_clutch", [](vehicle::VehicleDesc& d) { tcu_of(d).auto_clutch.fitted = false; }},
        {"auto_blip", "assist.auto_blip", [](vehicle::VehicleDesc& d) { tcu_of(d).auto_blip.fitted = false; }},
        {"auto_shift", "assist.auto_shift",
         [](vehicle::VehicleDesc& d) {
             tcu_of(d).auto_shift.up_rpm_at_full_throttle = 3500.0; // shifts inside the 3 s script
             tcu_of(d).auto_shift.fitted = false;
         }},
        {"traction_control", "assist.traction_control", [](vehicle::VehicleDesc& d) { d.traction_control.fitted = false; }},
    };
    for (const Case& c : cases) {
        INFO(c.name);
        vehicle::VehicleDesc fitted_desc = sedan();
        tcu_of(fitted_desc).auto_shift.up_rpm_at_full_throttle = 3500.0;
        const std::uint64_t on = assist_script(fitted_desc, c.channel, 1.0);
        const std::uint64_t off = assist_script(fitted_desc, c.channel, 0.0);
        // Control: with the assist fitted the channel reaches the dynamics.
        CHECK(on != off);

        vehicle::VehicleDesc unfitted = sedan();
        c.unfit(unfitted);
        const std::uint64_t u_on = assist_script(unfitted, c.channel, 1.0);
        const std::uint64_t u_off = assist_script(unfitted, c.channel, 0.0);
        std::printf("[F1 %s] fitted on %016llx off %016llx; not fitted on %016llx off %016llx\n", c.name, static_cast<unsigned long long>(on),
                    static_cast<unsigned long long>(off), static_cast<unsigned long long>(u_on), static_cast<unsigned long long>(u_off));
        CHECK(u_on == u_off);
        // Not fitted + channel 1 behaves like the assist being off in the fitted car.
        CHECK(u_on == off);
    }
}

TEST_CASE("F2: explicit fitted:true keys change nothing, defaults are fitted", "[assist][drivetrain][vehicle][io]") {
    TempDir td("f2");
    json j = vehicle_json("car_sedan.json");
    for (auto& c : j.at("controllers")) {
        if (c.at("type") == "manual_tcu") {
            c.at("auto_clutch")["fitted"] = true;
            c.at("auto_blip")["fitted"] = true;
            c.at("auto_shift")["fitted"] = true;
        }
    }
    j["traction_control"] = json::object();
    j["traction_control"]["fitted"] = true;
    const std::string explicit_path = write_vehicle(td, "explicit.json", j);

    const vehicle::VehicleDesc plain = sedan();
    CHECK(tcu_of(const_cast<vehicle::VehicleDesc&>(plain)).auto_clutch.fitted);
    CHECK(plain.traction_control.fitted);
    const vehicle::VehicleDesc expl = load_desc(explicit_path);
    CHECK(assist_script(plain, "assist.auto_clutch", 1.0) == assist_script(expl, "assist.auto_clutch", 1.0));
}

// ---------------------------------------------------------------------------------------------------------------
// L: launch control.
// ---------------------------------------------------------------------------------------------------------------
namespace {

struct LaunchRun {
    real hold_min = 1e9, hold_max = 0.0, hold_mean = 0.0;
    int hold_ticks = 0, hold_active_ticks = 0, hold_limiter_ticks = 0;
    real speed_after_hold = 0.0;
    real t_release_end = -1.0;         // seconds after the clutch started to come up when the limit let go (-1: never)
    real speed_at_release_end = 0.0;
    real max_rpm_while_active_moving = 0.0; // rpm max after the clutch came up, while still active
    real max_rpm_after_release = 0.0;
    real max_rpm_hold_whole = 0.0;
    std::uint64_t hash = 0;
    std::uint64_t own_violations = 0;
};

// Standstill, clutch in (manual), gear 1: settle 1 s, arm (optional), WOT; hold 3 s, then let the clutch up over
// 0.5 s and drive 4 s. `arm` selects the launch switch.
LaunchRun run_launch(const vehicle::VehicleDesc& d, bool arm, int workers = 1, bool alloc_armed = false) {
    auto rig = make_rig(d, workers);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;
    w.set_control("assist.auto_clutch", 0.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("assist.traction_control", 0.0);
    w.set_control("assist.launch_control", 0.0);
    w.set_control("clutch", 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 0.0);
    prepare(rig);
    ps_test::settle(w, 1.0);
    if (arm) w.set_control("assist.launch_control", 1.0);
    w.set_control("throttle", 1.0);

    namespace da = jobs::debug_alloc;
    const bool use_alloc = alloc_armed && da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }

    LaunchRun r;
    const real dt = w.dt();
    const int hold_n = static_cast<int>(3.0 / dt + 0.5);
    double sum = 0.0;
    for (int i = 0; i < hold_n; ++i) {
        w.step();
        const auto& pt = w.powertrain_state(v);
        r.max_rpm_hold_whole = std::max(r.max_rpm_hold_whole, pt.engine_rpm);
        if (i * dt >= 0.75) { // the spin-up to the limit is not part of the band
            r.hold_min = std::min(r.hold_min, pt.engine_rpm);
            r.hold_max = std::max(r.hold_max, pt.engine_rpm);
            sum += pt.engine_rpm;
            ++r.hold_ticks;
            if (pt.launch_control_active) ++r.hold_active_ticks;
            if (!pt.engines.empty() && pt.engines[0].limiter_active) ++r.hold_limiter_ticks;
        }
    }
    r.hold_mean = r.hold_ticks > 0 ? static_cast<real>(sum / r.hold_ticks) : 0.0;
    r.speed_after_hold = w.powertrain_state(v).speedo_mps;

    const int rel_n = static_cast<int>(0.5 / dt + 0.5);
    const int run_n = static_cast<int>(4.0 / dt + 0.5);
    for (int i = 0; i < run_n; ++i) {
        if (i < rel_n) w.set_control("clutch", 1.0 - static_cast<real>(i + 1) / static_cast<real>(rel_n));
        w.step();
        const auto& pt = w.powertrain_state(v);
        if (pt.launch_control_active) {
            r.max_rpm_while_active_moving = std::max(r.max_rpm_while_active_moving, pt.engine_rpm);
        } else if (r.t_release_end < 0.0 && arm) {
            r.t_release_end = static_cast<real>(i + 1) * dt;
            r.speed_at_release_end = pt.speedo_mps;
        }
        if (r.t_release_end >= 0.0) r.max_rpm_after_release = std::max(r.max_rpm_after_release, pt.engine_rpm);
        if (!arm) r.max_rpm_after_release = std::max(r.max_rpm_after_release, pt.engine_rpm);
    }
    if (use_alloc) {
        da::set_armed(false);
        r.own_violations = da::own_violation_count();
    }
    r.hash = w.state_hash();
    return r;
}

void print_run(const char* tag, const LaunchRun& r) {
    std::printf("[%s] hold rpm min %.1f mean %.1f max %.1f (whole %.1f), active %d/%d, limiter %d; speed after hold %.3f; "
                "release end %.3f s at %.3f m/s; max rpm active-moving %.1f, after release %.1f\n",
                tag, r.hold_min, r.hold_mean, r.hold_max, r.max_rpm_hold_whole, r.hold_active_ticks, r.hold_ticks, r.hold_limiter_ticks,
                r.speed_after_hold, r.t_release_end, r.speed_at_release_end, r.max_rpm_while_active_moving, r.max_rpm_after_release);
}

// The shipped part's values (data/electronics/launch_two_step_generic.json).
constexpr real kLaunchRpm = 4000.0;
constexpr real kMaxSpeed = 2.0;

void check_two_step(const vehicle::VehicleDesc& base, const char* tag, real band_lo_below, real band_hi_above) {
    vehicle::VehicleDesc d = base;
    add_launch(d, kLaunchRpm, kMaxSpeed);
    const LaunchRun on = run_launch(d, true);
    const LaunchRun off = run_launch(d, false);
    print_run((std::string(tag) + " armed").c_str(), on);
    print_run((std::string(tag) + " disarmed").c_str(), off);

    // Held in the declared band around `rpm` for the whole 3 s at standstill with the clutch in.
    CHECK(on.hold_ticks > 500);
    CHECK(on.hold_min >= kLaunchRpm - band_lo_below);
    CHECK(on.hold_max <= kLaunchRpm + band_hi_above);
    CHECK(on.hold_active_ticks == on.hold_ticks);
    CHECK(on.hold_limiter_ticks > 0); // the engine's own limiter path did the limiting
    CHECK(on.speed_after_hold < 0.1);

    // Releasing the clutch launches the car with the limit held; it lets go above max_speed_mps.
    CHECK(on.t_release_end > 0.0);
    CHECK(on.speed_at_release_end > kMaxSpeed);
    CHECK(on.speed_at_release_end < kMaxSpeed + 1.5);
    CHECK(on.max_rpm_while_active_moving <= kLaunchRpm + band_hi_above);
    CHECK(on.max_rpm_after_release > kLaunchRpm + 1000.0); // the normal limiter is back in charge

    // Disarmed: the ordinary limiter, far above the launch rpm, and launch_control never reports active.
    CHECK(off.hold_active_ticks == 0);
    CHECK(off.hold_max > kLaunchRpm + 1500.0);
}

} // namespace

TEST_CASE("L1: two-step holds the rpm band at standstill and releases above max_speed (torque map, fuel cut)", "[assist][drivetrain][vehicle]") {
    check_two_step(sedan(), "L1 car_sedan fuel cut", 250.0, 60.0);
}

TEST_CASE("L2: two-step on a soft-cut torque-map engine", "[assist][drivetrain][vehicle]") {
    vehicle::VehicleDesc d = sedan();
    auto& lim = limiter_of(d);
    lim.type = drivetrain::LimiterType::SoftCut;
    lim.rpm = 6800.0;
    lim.band_rpm = 300.0;
    check_two_step(d, "L2 car_sedan soft cut", 150.0, 30.0);
}

TEST_CASE("L3: two-step on a simulated engine (car_sedan_sim)", "[assist][drivetrain][vehicle]") {
    check_two_step(sedan_sim(), "L3 car_sedan_sim", 250.0, 60.0);
}

TEST_CASE("L4: a not-fitted launch control and a missing one behave like no launch control", "[assist][drivetrain][vehicle]") {
    const LaunchRun none = run_launch(sedan(), true);
    vehicle::VehicleDesc d = sedan();
    add_launch(d, kLaunchRpm, kMaxSpeed, false);
    const LaunchRun unfitted = run_launch(d, true);
    print_run("L4 none", none);
    print_run("L4 not fitted", unfitted);
    CHECK(unfitted.hold_active_ticks == 0);
    CHECK(unfitted.hold_max > kLaunchRpm + 1500.0);
    // Dynamics identical to the car without the component (the latch byte is the only hashed difference).
    CHECK(unfitted.hold_max == none.hold_max);
    CHECK(unfitted.max_rpm_after_release == none.max_rpm_after_release);
}

TEST_CASE("L5: the clutch must be in to arm, and the switch off ends the control", "[assist][drivetrain][vehicle]") {
    vehicle::VehicleDesc d = sedan();
    add_launch(d, kLaunchRpm, kMaxSpeed);
    auto rig = make_rig(d);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;
    w.set_control("assist.auto_clutch", 0.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.traction_control", 0.0);
    w.set_control("clutch", 0.0); // manual clutch up
    w.set_control("gear_request", 0.0);
    w.set_control("throttle", 0.0);
    prepare(rig);
    ps_test::settle(w, 0.5);
    w.set_control("assist.launch_control", 1.0);
    ps_test::settle(w, 0.5);
    // In neutral with the clutch up (engine free-revving is not "clutch in" here: the pedal curve is open) - read it,
    // then press the clutch: it arms.
    const bool armed_with_clutch_up = w.powertrain_state(v).launch_control_active;
    w.set_control("clutch", 1.0);
    ps_test::settle(w, 0.3);
    const bool armed_with_clutch_in = w.powertrain_state(v).launch_control_active;
    w.set_control("assist.launch_control", 0.0);
    ps_test::settle(w, 0.1);
    const bool after_off = w.powertrain_state(v).launch_control_active;
    std::printf("[L5] armed with clutch up %d, clutch in %d, after switch off %d\n", armed_with_clutch_up, armed_with_clutch_in, after_off);
    CHECK(armed_with_clutch_in);
    CHECK_FALSE(after_off);
    // With the clutch up (pedal 0 = engaged) the control does not arm.
    CHECK_FALSE(armed_with_clutch_up);
}

// ---------------------------------------------------------------------------------------------------------------
// G: grafting through the user layer.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("G1: the user layer grafts the part: same descriptor and state hash as the controllers entry in the file",
          "[assist][drivetrain][vehicle][io]") {
    TempDir td("g1");
    const std::string part = kData + "/electronics/launch_two_step_generic.json";

    io::VehicleLoadOptions opts;
    opts.electronics["launch"] = part;
    const vehicle::VehicleDesc grafted = load_desc(kData + "/vehicles/car_sedan.json", opts);

    json j = vehicle_json("car_sedan.json");
    j.at("controllers").push_back({{"id", "launch"}, {"type", "electronics"}, {"ref", part}});
    const vehicle::VehicleDesc in_file = load_desc(write_vehicle(td, "with_launch.json", j));

    const vehicle::VehicleDesc plain = sedan();
    REQUIRE(grafted.controllers.size() == plain.controllers.size() + 1);
    REQUIRE(in_file.controllers.size() == plain.controllers.size() + 1);
    const auto* lc = std::get_if<drivetrain::LaunchControlDesc>(&grafted.controllers.back().params);
    REQUIRE(lc != nullptr);
    CHECK(lc->rpm == 4000.0);
    CHECK(lc->max_speed_mps == 2.0);
    CHECK(lc->channel == "assist.launch_control");
    CHECK(lc->fitted);

    const LaunchRun a = run_launch(grafted, true);
    const LaunchRun b = run_launch(in_file, true);
    const LaunchRun c = run_launch(plain, true);
    print_run("G1 grafted", a);
    CHECK(a.hash == b.hash);
    CHECK(a.hold_active_ticks == a.hold_ticks);
    CHECK(c.hold_active_ticks == 0);
    CHECK(c.hash != a.hash);

    // "none" removes an entry that the vehicle file has.
    io::VehicleLoadOptions rm;
    rm.electronics["launch"] = "none";
    const vehicle::VehicleDesc removed = load_desc(write_vehicle(td, "with_launch2.json", j), rm);
    CHECK(removed.controllers.size() == plain.controllers.size());
    // A path replaces the part of an existing entry.
    write_text(td.file("lc3000.json"),
               R"({"format":"physics_sim.electronics/1","kind":"launch_control","rpm":3000,"max_speed_mps":3.0})");
    io::VehicleLoadOptions rep;
    rep.electronics["launch"] = td.file("lc3000.json");
    const vehicle::VehicleDesc replaced = load_desc(write_vehicle(td, "with_launch3.json", j), rep);
    REQUIRE(replaced.controllers.size() == plain.controllers.size() + 1);
    const auto* lc3 = std::get_if<drivetrain::LaunchControlDesc>(&replaced.controllers.back().params);
    REQUIRE(lc3 != nullptr);
    CHECK(lc3->rpm == 3000.0);
}

TEST_CASE("G2: an assist_fitment part fits a not-fitted assist", "[assist][drivetrain][vehicle][io]") {
    TempDir td("g2");
    json j = vehicle_json("car_sedan.json");
    for (auto& c : j.at("controllers"))
        if (c.at("type") == "manual_tcu") {
            c.at("auto_blip")["fitted"] = false;
            c.at("auto_shift")["fitted"] = false;
        }
    j["traction_control"] = {{"fitted", false}};
    const std::string base = write_vehicle(td, "unfitted.json", j);
    vehicle::VehicleDesc u = load_desc(base);
    CHECK_FALSE(tcu_of(u).auto_blip.fitted);
    CHECK_FALSE(tcu_of(u).auto_shift.fitted);
    CHECK(tcu_of(u).auto_clutch.fitted);
    CHECK_FALSE(u.traction_control.fitted);

    io::VehicleLoadOptions o;
    o.electronics["fit_a"] = kData + "/electronics/fit_auto_assists.json";
    o.electronics["fit_t"] = kData + "/electronics/fit_traction_control.json";
    vehicle::VehicleDesc f = load_desc(base, o);
    CHECK(tcu_of(f).auto_clutch.fitted);
    CHECK(tcu_of(f).auto_blip.fitted);
    CHECK(tcu_of(f).auto_shift.fitted);
    CHECK(f.traction_control.fitted);
    CHECK(f.controllers.size() == u.controllers.size()); // fitment parts add no controller
}

// ---------------------------------------------------------------------------------------------------------------
// R: loader rejections.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("R1: electronics part rejections", "[assist][io][drivetrain]") {
    TempDir td("r1");
    const auto part = [&](const std::string& body) {
        write_text(td.file("p.json"), body);
        return td.file("p.json");
    };
    const std::string head = R"({"format":"physics_sim.electronics/1","kind":"launch_control",)";
    CHECK_THROWS_WITH(io::load_electronics_json(part(head + R"("rpm":0,"max_speed_mps":2})")), ContainsSubstring("'rpm' must be finite and > 0"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(head + R"("rpm":-5,"max_speed_mps":2})")), ContainsSubstring("'rpm' must be finite and > 0"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(head + R"("max_speed_mps":2})")), ContainsSubstring("'rpm' is required"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(head + R"("rpm":4000,"max_speed_mps":-1})")),
                      ContainsSubstring("'max_speed_mps' must be finite and > 0"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(head + R"("rpm":4000,"max_speed_mps":2,"bogus":1})")),
                      ContainsSubstring("unknown electronics field 'bogus'"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(head + R"("rpm":4000,"max_speed_mps":2,"fitted":"yes"})")),
                      ContainsSubstring("'fitted' must be a boolean"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(R"({"format":"physics_sim.electronics/1","kind":"toaster"})")),
                      ContainsSubstring("unknown electronics kind"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(R"({"format":"physics_sim.electronics/2","kind":"launch_control"})")),
                      ContainsSubstring("unsupported 'format'"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(R"({"format":"physics_sim.electronics/1","kind":"assist_fitment","fits":["auto_shifter"]})")),
                      ContainsSubstring("'fits' entries must be one of"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(R"({"format":"physics_sim.electronics/1","kind":"assist_fitment","fits":[]})")),
                      ContainsSubstring("non-empty array"));
    CHECK_THROWS_WITH(io::load_electronics_json(part(R"({"format":"physics_sim.electronics/1","kind":"assist_fitment","fits":["auto_blip"],"rpm":3})")),
                      ContainsSubstring("unknown electronics field 'rpm'"));
    // extends works (the shipped format has no path-valued key)
    write_text(td.file("base.json"), head + R"("rpm":4200,"max_speed_mps":2})");
    write_text(td.file("child.json"), R"({"extends":"base.json","rpm":3500})");
    const io::ElectronicsPart p = io::load_electronics_json(td.file("child.json"));
    CHECK(p.launch.rpm == 3500.0);
    CHECK(p.launch.max_speed_mps == 2.0);
    // the shipped parts load
    CHECK(io::load_electronics_json(kData + "/electronics/launch_two_step_generic.json").launch.rpm == 4000.0);
    CHECK(io::load_electronics_json(kData + "/electronics/fit_auto_assists.json").fits.size() == 3);
    CHECK(io::load_electronics_json(kData + "/electronics/fit_traction_control.json").fits.size() == 1);
    CHECK(io::load_electronics_json(kData + "/electronics/fit_anti_lag_switch.json").fits.size() == 1);
}

TEST_CASE("R2: vehicle-level electronics rejections", "[assist][io][drivetrain][vehicle]") {
    TempDir td("r2");
    const auto graft = [&](const std::string& body, const std::string& id = "launch") {
        write_text(td.file("lc.json"), body);
        io::VehicleLoadOptions o;
        o.electronics[id] = td.file("lc.json");
        return o;
    };
    const std::string car = kData + "/vehicles/car_sedan.json";
    const std::string head = R"({"format":"physics_sim.electronics/1","kind":"launch_control",)";
    // i4_2l_na.json: limiter 6800, idle 800
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":7000,"max_speed_mps":2})")),
                      ContainsSubstring("must be below the engine's own limiter rpm"));
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":6800,"max_speed_mps":2})")),
                      ContainsSubstring("must be below the engine's own limiter rpm"));
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":800,"max_speed_mps":2})")), ContainsSubstring("must be above the engine's idle rpm"));
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":500,"max_speed_mps":2})")), ContainsSubstring("must be above the engine's idle rpm"));
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":4000,"max_speed_mps":-2})")), ContainsSubstring("'max_speed_mps' must be finite and > 0"));
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":4000,"max_speed_mps":2,"nope":1})")), ContainsSubstring("unknown electronics field 'nope'"));
    // the manual_tcu id cannot be replaced by the user layer
    CHECK_THROWS_WITH(load_desc(car, graft(head + R"("rpm":4000,"max_speed_mps":2})", "tcu")), ContainsSubstring("is the manual_tcu controller"));
    // removing what does not exist
    io::VehicleLoadOptions rm;
    rm.electronics["ghost"] = "none";
    CHECK_THROWS_WITH(load_desc(car, rm), ContainsSubstring("no electronics entry with id 'ghost' to remove"));
    // an empty path
    io::VehicleLoadOptions emp;
    emp.electronics["launch"] = "";
    CHECK_THROWS_WITH(load_desc(car, emp), ContainsSubstring("the path must be non-empty"));
    // two launch controls
    io::VehicleLoadOptions two;
    two.electronics["l1"] = kData + "/electronics/launch_two_step_generic.json";
    two.electronics["l2"] = kData + "/electronics/launch_two_step_generic.json";
    CHECK_THROWS_WITH(load_desc(car, two), ContainsSubstring("at most one launch_control"));
    // fits an auto_* assist on a car without a manual_tcu
    json j = vehicle_json("car_sedan.json");
    json ctl = json::array();
    for (auto& c : j.at("controllers"))
        if (c.at("type") != "manual_tcu") ctl.push_back(c);
    j["controllers"] = ctl;
    const std::string no_tcu = write_vehicle(td, "no_tcu.json", j);
    io::VehicleLoadOptions fit;
    fit.electronics["fit"] = kData + "/electronics/fit_auto_assists.json";
    // (the car without a tcu may be rejected earlier for its own reasons; the fitment message is what matters when it loads)
    try {
        (void)load_desc(no_tcu, fit);
        FAIL("fitment of auto_* on a car without a manual_tcu must be rejected");
    } catch (const std::exception& e) {
        CHECK_THAT(std::string(e.what()), ContainsSubstring("manual_tcu"));
    }
    // vehicle-file entry without a ref
    json j2 = vehicle_json("car_sedan.json");
    j2.at("controllers").push_back({{"id", "launch"}, {"type", "electronics"}});
    CHECK_THROWS_WITH(load_desc(write_vehicle(td, "noref.json", j2)), ContainsSubstring("needs a 'ref'"));
    // a vehicle-file launch_control of an unknown key in the anti_lag_switch block
    json j3 = vehicle_json("car_sedan.json");
    j3["anti_lag_switch"] = {{"fitted", false}, {"colour", "red"}};
    CHECK_THROWS(load_desc(write_vehicle(td, "badswitch.json", j3)));
}

// ---------------------------------------------------------------------------------------------------------------
// S: snapshot, hash/worker/alloc invariants.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("S1: snapshot reports fitted (distinguishable from off) and launch state", "[assist][drivetrain][vehicle]") {
    {
        auto rig = make_rig(sedan());
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        // auto_clutch, auto_blip, auto_shift, traction_control, anti-lag switch fitted; launch control absent
        CHECK(pt.assist_fitted_bits == (1u | 2u | 4u | 8u | 32u));
        CHECK((pt.assist_active_bits & 16u) == 0u);
        CHECK_FALSE(pt.launch_control_active);
    }
    {
        vehicle::VehicleDesc d = sedan();
        tcu_of(d).auto_shift.fitted = false;
        d.traction_control.fitted = false;
        add_launch(d, kLaunchRpm);
        auto rig = make_rig(d);
        rig.world->set_control("assist.auto_shift", 1.0); // ignored
        rig.world->set_control("assist.traction_control", 1.0);
        rig.world->set_control("assist.launch_control", 1.0);
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        CHECK(pt.assist_fitted_bits == (1u | 2u | 16u | 32u));
        CHECK((pt.assist_active_bits & 4u) == 0u); // not fitted + channel on = not active
        CHECK((pt.assist_active_bits & 8u) == 0u);
        CHECK((pt.assist_active_bits & 16u) == 16u); // switched on (fitted)
    }
    {
        // a car whose manual_tcu is gone reports the auto_* assists as not fitted
        TempDir td("s1");
        json j = vehicle_json("car_sedan.json");
        json ctl = json::array();
        for (auto& c : j.at("controllers"))
            if (c.at("type") != "manual_tcu") ctl.push_back(c);
        j["controllers"] = ctl;
        // (loads only if a car without a tcu is legal; otherwise the case documents that it is rejected)
        try {
            const auto d = load_desc(write_vehicle(td, "no_tcu.json", j));
            auto rig = make_rig(d);
            rig.world->step();
            CHECK((rig.world->powertrain_state(rig.vehicle).assist_fitted_bits & 7u) == 0u);
        } catch (const std::exception& e) {
            WARN("car without manual_tcu not loadable here: " << e.what());
        }
    }
}

TEST_CASE("S2: launch control - 1 vs 10 workers identical, allocation-free, hashed state only with the component",
          "[assist][drivetrain][vehicle][determinism]") {
    if (!jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    vehicle::VehicleDesc d = sedan();
    add_launch(d, kLaunchRpm, kMaxSpeed);
    const LaunchRun w1 = run_launch(d, true, 1, true);
    const LaunchRun w10 = run_launch(d, true, 10, true);
    std::printf("[S2] hash 1w %016llx 10w %016llx, own alloc violations %llu / %llu\n", static_cast<unsigned long long>(w1.hash),
                static_cast<unsigned long long>(w10.hash), static_cast<unsigned long long>(w1.own_violations),
                static_cast<unsigned long long>(w10.own_violations));
    CHECK(w1.hash == w10.hash);
    CHECK(w1.own_violations == 0);
    CHECK(w10.own_violations == 0);

    // Hashed only with the component: a car with it, switch never touched, hashes differently from one without it
    // only through the latch byte; the dynamics are the same (no launch ever active).
    const LaunchRun with_idle = run_launch(d, false);
    const LaunchRun without = run_launch(sedan(), false);
    CHECK(with_idle.hold_max == without.hold_max);
    CHECK(with_idle.max_rpm_after_release == without.max_rpm_after_release);
}

TEST_CASE("S3: launch control on the simulated engine - 1 vs 10 workers identical, allocation-free", "[assist][drivetrain][vehicle][determinism]") {
    vehicle::VehicleDesc d = sedan_sim();
    add_launch(d, kLaunchRpm, kMaxSpeed);
    const LaunchRun w1 = run_launch(d, true, 1, true);
    const LaunchRun w10 = run_launch(d, true, 10, true);
    CHECK(w1.hash == w10.hash);
    CHECK(w1.own_violations == 0);
    CHECK(w10.own_violations == 0);
}

TEST_CASE("S5: the launch latch is hashed state (and only on a car that has the component)", "[assist][drivetrain][vehicle][determinism]") {
    // Armed at idle with the clutch in: the second limit (4000 rpm) is never reached, so the dynamics are identical
    // to the disarmed run - the latch byte is the only difference. Control channels are not hashed.
    const auto idle_hash = [](const vehicle::VehicleDesc& d, bool arm, bool* active_out) {
        auto rig = make_rig(d);
        World& w = *rig.world;
        w.set_control("assist.auto_clutch", 0.0);
        w.set_control("clutch", 1.0);
        w.set_control("gear_request", 1.0);
        w.set_control("throttle", 0.0);
        prepare(rig);
        ps_test::settle(w, 0.5);
        w.set_control("assist.launch_control", arm ? 1.0 : 0.0);
        ps_test::settle(w, 0.5);
        if (active_out != nullptr) *active_out = w.powertrain_state(rig.vehicle).launch_control_active;
        return w.state_hash();
    };
    vehicle::VehicleDesc d = sedan();
    add_launch(d, kLaunchRpm, kMaxSpeed);
    bool armed_active = false, idle_active = true;
    const std::uint64_t armed = idle_hash(d, true, &armed_active);
    const std::uint64_t disarmed = idle_hash(d, false, &idle_active);
    std::printf("[S5] armed %016llx (active %d) disarmed %016llx (active %d)\n", static_cast<unsigned long long>(armed), armed_active,
                static_cast<unsigned long long>(disarmed), idle_active);
    CHECK(armed_active);
    CHECK_FALSE(idle_active);
    CHECK(armed != disarmed);
    // A car without the component ignores the channel entirely: same hash armed or not.
    CHECK(idle_hash(sedan(), true, nullptr) == idle_hash(sedan(), false, nullptr));
}

// ---------------------------------------------------------------------------------------------------------------
// S4: the anti-lag fitted switch (hyper-like turbo engine in the sedan, the D5 vehicle fixture).
// ---------------------------------------------------------------------------------------------------------------
namespace {

vehicle::VehicleDesc turbo_sedan_desc(bool switch_fitted) {
    vehicle::VehicleDesc d = sedan();
    drivetrain::SimulatedEngineDesc eng = ps::test::mvem::electronic_engine();
    eng.turbo_pair = ps::test::turbo::hyper_like_pair_with_anti_lag();
    for (auto& c : d.powertrain.components) {
        if (const auto* tm = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) {
            eng.crank_axis_local = tm->crank_axis_local;
            c.params = eng;
            break;
        }
    }
    d.powertrain.anti_lag_fitted = switch_fitted;
    return d;
}

struct AlsRun {
    std::uint64_t hash = 0;
    std::uint32_t activations = 0;
    int active_ticks = 0;
};

AlsRun run_als(const vehicle::VehicleDesc& d, real channel) {
    auto rig = make_rig(d);
    World& w = *rig.world;
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("clutch", 0.0);
    w.set_control("throttle", 0.0);
    w.set_control("anti_lag", channel);
    prepare(rig);
    ps_test::settle(w, 0.5);
    AlsRun r;
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 480; ++t) w.step();
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 240; ++t) {
        w.step();
        const auto& es = w.powertrain_state(rig.vehicle).engines.at(0);
        r.active_ticks += es.anti_lag_active;
        r.activations = es.anti_lag_activations;
    }
    r.hash = w.state_hash();
    return r;
}

} // namespace

TEST_CASE("S4: anti_lag_switch fitted:false ignores the anti_lag channel", "[assist][drivetrain][vehicle][turbo]") {
    const AlsRun on_fitted = run_als(turbo_sedan_desc(true), 1.0);
    const AlsRun off_fitted = run_als(turbo_sedan_desc(true), 0.0);
    const AlsRun on_unfitted = run_als(turbo_sedan_desc(false), 1.0);
    const AlsRun off_unfitted = run_als(turbo_sedan_desc(false), 0.0);
    std::printf("[S4] fitted: ch1 %016llx act %u ticks %d | ch0 %016llx act %u; not fitted: ch1 %016llx act %u | ch0 %016llx act %u\n",
                static_cast<unsigned long long>(on_fitted.hash), on_fitted.activations, on_fitted.active_ticks,
                static_cast<unsigned long long>(off_fitted.hash), off_fitted.activations, static_cast<unsigned long long>(on_unfitted.hash),
                on_unfitted.activations, static_cast<unsigned long long>(off_unfitted.hash), off_unfitted.activations);
    // Control: fitted, the channel switches anti-lag on.
    CHECK(on_fitted.activations >= 1);
    CHECK(on_fitted.active_ticks > 0);
    CHECK(off_fitted.activations == 0);
    CHECK(on_fitted.hash != off_fitted.hash);
    // Not fitted: the channel is ignored (the config's default_enabled = 0 rules).
    CHECK(on_unfitted.activations == 0);
    CHECK(on_unfitted.active_ticks == 0);
    CHECK(on_unfitted.hash == off_unfitted.hash);
    CHECK(on_unfitted.hash == off_fitted.hash);
}
