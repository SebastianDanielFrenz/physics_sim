// tests/unit/test_n2o_controller.cpp -- N2O plan N4: controller completeness.
// N1 already built the request conditions, the safety latches, the purge and the
// first progressive/retard cases (test_n2o_system.cpp S1-S9); N4 pins the rest of
// plan sections 2.5/3/6 at literal times and through the whole vehicle:
//
//   C1 the progressive ramp against the plan formula at literal times; the delay,
//      the reset when the solenoid closes, the flow ratio to a full-flow system
//   C2 the spark retard as a curve over the progressive fraction (and the scalar form)
//   C3 neutral and reverse never spray - a hard rule, also for a code-built
//      min_gear <= 0; min_gear/max_gear/min_speed windows; the loader rejection
//   C4 the max_active_s duty cut at an exact substep count, whole vehicle: the
//      n2o_safety_cut enum 1 (bottle pressure) and 3 (duty time)
//   C5 sound counters: n2o_activations = solenoid rising edges, n2o_purge_events =
//      once per press, n2o_flow_fraction follows the formula, solenoid/purge flags
//   C6 the vehicle-level "controller" override wins over the kit default
//      (bottom-up), its rejections, the heater/arm/purge channel clash
//   C7 1 vs 10 workers with a progressive controller, zero allocation with the
//      solenoid toggling
//   C8 a vehicle without a kit (or a kit never armed) is bit-identical, every new
//      sound field 0
//
// Fixtures (tests/support/fixtures/, never shipped data): n2o/bottle_10lb.json,
// n2o/kit_wet.json, n2o/controller_basic.json, vehicle_n2o_sedan.json (a torque-map
// engine: it ignores the N2O, so the whole-vehicle checks read the controller's
// own state through EngineSoundState). Expected values are literals or the plan's
// formulas written out in the test, never read back from the code under test.
#include "drivetrain/nitrous_system.h"
#include "drivetrain/powertrain.h"

#include "ps/drivetrain/nitrous_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/fuel_io.h"
#include "ps/io/nitrous_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr real kPi = 3.14159265358979323846;
real rpm_to_omega(real rpm) { return rpm * real(2.0) * kPi / real(60.0); }

std::string fixture_path(const std::string& name) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/" + name; }
std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }
std::string n2o_sedan_path() { return fixture_path("vehicle_n2o_sedan.json"); }
std::string tank_sedan_path() { return fixture_path("vehicle_tank_sedan.json"); }
std::string controller_path() { return fixture_path("n2o/controller_basic.json"); }

nlohmann::json read_json(const std::string& path) {
    std::ifstream in(path);
    nlohmann::json j;
    in >> j;
    return j;
}

// A temporary JSON file next to the fixtures (relative paths resolve against it), removed on scope exit.
struct TempJson {
    std::filesystem::path path;
    TempJson(const std::string& dir_rel, const nlohmann::json& j) {
        static int counter = 0;
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = std::filesystem::path(fixture_path(dir_rel)) / ("_tmp_n2o_n4_" + std::to_string(stamp) + "_" + std::to_string(counter++) + ".json");
        std::ofstream out(path);
        out << j.dump(2);
    }
    ~TempJson() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    [[nodiscard]] std::string str() const { return path.string(); }
};

template <typename T>
const T& component_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* p = std::get_if<T>(&c.params)) return *p;
    }
    FAIL("fixture lacks a required component type");
    throw std::logic_error("unreachable");
}
template <typename T>
T& mutable_component_of(vehicle::VehicleDesc& d) {
    for (auto& c : d.powertrain.components) {
        if (auto* p = std::get_if<T>(&c.params)) return *p;
    }
    FAIL("fixture lacks a required component type");
    throw std::logic_error("unreachable");
}

// ---- white-box helpers ------------------------------------------------------

real e5_density() { return io::load_fuel_json(data_path("fuels/e5.json")).density_kg_m3; }

struct Parts {
    N2oBottleDesc bottle;
    NitrousKitDesc kit;
};
Parts fixture_parts() {
    const vehicle::VehicleDesc d = io::load_vehicle_json(n2o_sedan_path());
    return Parts{component_of<N2oBottleDesc>(d), component_of<NitrousKitDesc>(d)};
}
// heater/arm/purge slots 0/1/2 (white-box: the inputs carry the values directly).
std::unique_ptr<NitrousSystem> make_system(const Parts& p) { return std::make_unique<NitrousSystem>(p.bottle, p.kit, e5_density(), 1, 0, 2); }

// A 1 ms substep: t_active after k steps is k * 0.001 s, so the literal times below are exact to rounding.
constexpr real kHms = 0.001;
constexpr real kHs = 1.0 / 960.0;

// Everything the fixture controller asks for: armed, WOT, 3000 rpm, 3rd gear, 20 m/s, running.
NitrousInputs go(real h = kHms) {
    NitrousInputs in;
    in.h = h;
    in.arm = 1.0;
    in.heater = 0.0;
    in.purge = 0.0;
    in.throttle_command = 1.0;
    in.omega = rpm_to_omega(3000.0);
    in.has_gearbox = true;
    in.gear = 3;
    in.speed_mps = 20.0;
    in.engine_running = true;
    in.ignition_cut = false;
    in.p_intake_pa = 50000.0;
    in.p_ambient_pa = 101325.0;
    return in;
}

// ---- whole-vehicle rig ------------------------------------------------------

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

Rig make_rig(const vehicle::VehicleDesc& desc, int workers = 1, int gear = 1) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    rig.world = std::make_unique<World>(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    rig.chassis = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(data_path("surfaces/surfaces.json")));
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);

    rig.world->set_control("assist.auto_clutch", 1.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = gear;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

const EngineSoundState& ess(const Rig& r) { return r.world->powertrain_state(r.vehicle).engines.at(0); }

void run(Rig& r, int ticks) {
    for (int t = 0; t < ticks; ++t) r.world->step();
}

} // namespace

// ---------------------------------------------------------------------------
// C1: the progressive ramp. frac = clamp(start + (1-start)*(t_active - delay)/ramp, 0, 1)
// from the time the solenoid has been open; start until the delay has passed; the
// active time restarts when the solenoid closes. Literal times at h = 1 ms
// (start 0.4, delay 0.05 s, ramp 0.1 s): t 0.020 -> 0.4, 0.075 -> 0.55, 0.100 -> 0.7,
// 0.125 -> 0.85, 0.150/0.200 -> 1.0.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C1: the progressive ramp follows the plan formula at literal times", "[unit][n2o][drivetrain]") {
    Parts parts = fixture_parts();
    NitrousProgressiveDesc& pg = parts.kit.controller.progressive;
    pg.start_fraction = 0.4;
    pg.delay_s = 0.05;
    pg.ramp_s = 0.1;

    SECTION("shape: start, delay, linear ramp, saturation") {
        auto sys = make_system(parts);
        const real g_expect_ratio_ref = [&] {
            // Flow at full duty from a second system without a progressive block.
            Parts full = parts;
            full.kit.controller.progressive = NitrousProgressiveDesc{};
            auto s2 = make_system(full);
            return s2->step(go()).mdot_n_kg_s;
        }();
        REQUIRE(g_expect_ratio_ref > 0.0);
        std::vector<real> frac;
        std::vector<real> mdot;
        for (int k = 0; k <= 200; ++k) {
            const NitrousStatus& s = sys->step(go());
            frac.push_back(s.flow_fraction);
            mdot.push_back(s.mdot_n_kg_s);
            // not committed: the bottle stays full, its pressure frozen
        }
        CHECK(frac[0] == Catch::Approx(0.4).epsilon(1e-12));
        CHECK(frac[20] == Catch::Approx(0.4).epsilon(1e-12));
        CHECK(frac[75] == Catch::Approx(0.55).epsilon(1e-9));
        CHECK(frac[100] == Catch::Approx(0.7).epsilon(1e-9));
        CHECK(frac[125] == Catch::Approx(0.85).epsilon(1e-9));
        CHECK(frac[150] == Catch::Approx(1.0).epsilon(1e-9));
        CHECK(frac[200] == 1.0);
        for (std::size_t i = 1; i < frac.size(); ++i) CHECK(frac[i] >= frac[i - 1]);
        // The flow is the full-duty flow times the fraction (same bottle state, same intake pressure).
        CHECK(mdot[0] == Catch::Approx(0.4 * g_expect_ratio_ref).epsilon(1e-12));
        CHECK(mdot[75] == Catch::Approx(0.55 * g_expect_ratio_ref).epsilon(1e-9));
        CHECK(mdot[200] == Catch::Approx(g_expect_ratio_ref).epsilon(1e-12));
    }
    SECTION("the active time restarts when the solenoid closes") {
        auto sys = make_system(parts);
        for (int k = 0; k < 100; ++k) sys->step(go()); // up to t = 0.1: fraction 0.7
        CHECK(sys->step(go()).flow_fraction == Catch::Approx(0.7).epsilon(1e-9));
        NitrousInputs off = go();
        off.arm = 0.0;
        {
            const NitrousStatus& s = sys->step(off); // one closed substep
            CHECK_FALSE(s.solenoid_open);
            CHECK(s.flow_fraction == 0.0);
        }
        CHECK(sys->active_time_s() == 0.0);
        // Reopened: the start fraction again, and the delay and ramp run from zero.
        std::vector<real> frac;
        for (int k = 0; k <= 100; ++k) frac.push_back(sys->step(go()).flow_fraction);
        CHECK(frac[0] == Catch::Approx(0.4).epsilon(1e-12));
        CHECK(frac[20] == Catch::Approx(0.4).epsilon(1e-12));
        CHECK(frac[75] == Catch::Approx(0.55).epsilon(1e-9));
        CHECK(frac[100] == Catch::Approx(0.7).epsilon(1e-9));
        CHECK(sys->activations() == 2);
    }
    SECTION("ramp 0: start fraction until the delay, then straight to 1") {
        pg.ramp_s = 0.0;
        auto sys = make_system(parts);
        std::vector<real> frac;
        for (int k = 0; k <= 80; ++k) frac.push_back(sys->step(go()).flow_fraction);
        CHECK(frac[0] == Catch::Approx(0.4).epsilon(1e-12));
        CHECK(frac[40] == Catch::Approx(0.4).epsilon(1e-12));
        CHECK(frac[60] == 1.0);
        CHECK(frac[80] == 1.0);
    }
    SECTION("no progressive block: full flow from the first substep") {
        pg = NitrousProgressiveDesc{};
        auto sys = make_system(parts);
        CHECK(sys->step(go()).flow_fraction == 1.0);
    }
}

// ---------------------------------------------------------------------------
// C2: the spark retard over the progressive fraction. Curve {0, 0.5, 1} -> {0, 2, 8}
// deg: fraction 0.4 -> 1.6, 0.55 -> 2.6, 0.7 -> 4.4, 1.0 -> 8.0. The scalar form is
// constant and independent of the fraction. The retard is 0 while nothing flows.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C2: the spark retard follows the curve over the progressive fraction", "[unit][n2o][drivetrain][io]") {
    Parts parts = fixture_parts();
    NitrousProgressiveDesc& pg = parts.kit.controller.progressive;
    pg.start_fraction = 0.4;
    pg.delay_s = 0.05;
    pg.ramp_s = 0.1;

    SECTION("curve") {
        NitrousControllerDesc& c = parts.kit.controller;
        c.retard_is_curve = true;
        c.retard_vs_fraction.x = {0.0, 0.5, 1.0};
        c.retard_vs_fraction.y = {0.0, 2.0, 8.0};
        auto sys = make_system(parts);
        std::vector<real> retard;
        std::vector<real> sent; // what the engine receives
        for (int k = 0; k <= 160; ++k) {
            retard.push_back(sys->step(go()).retard_deg);
            sent.push_back(sys->engine_input().retard_deg);
        }
        CHECK(retard[0] == Catch::Approx(1.6).epsilon(1e-9));
        CHECK(retard[20] == Catch::Approx(1.6).epsilon(1e-9));
        CHECK(retard[75] == Catch::Approx(2.6).epsilon(1e-9));
        CHECK(retard[100] == Catch::Approx(4.4).epsilon(1e-9));
        CHECK(retard[160] == Catch::Approx(8.0).epsilon(1e-12));
        CHECK(sent[100] == retard[100]); // the engine sees the same value
        // closed: nothing flows, no retard
        NitrousInputs off = go();
        off.arm = 0.0;
        CHECK(sys->step(off).retard_deg == 0.0);
    }
    SECTION("scalar form does not depend on the fraction") {
        parts.kit.controller.retard_is_curve = false;
        parts.kit.controller.spark_retard_deg = 3.0;
        auto sys = make_system(parts);
        for (int k = 0; k <= 160; ++k) {
            const NitrousStatus& s = sys->step(go());
            if (k == 0 || k == 75 || k == 160) CHECK(s.retard_deg == 3.0);
        }
    }
    SECTION("the loader reads both forms and rejects a malformed curve") {
        {
            nlohmann::json j = read_json(controller_path());
            j["spark_retard_deg"] = {{"fraction", {0.0, 0.5, 1.0}}, {"deg", {0.0, 2.0, 8.0}}};
            const TempJson t("n2o", j);
            const NitrousControllerDesc c = io::load_nitrous_controller_json(t.str());
            REQUIRE(c.retard_is_curve);
            CHECK(c.retard_vs_fraction.evaluate(0.75) == Catch::Approx(5.0).epsilon(1e-12));
        }
        {
            const NitrousControllerDesc c = io::load_nitrous_controller_json(controller_path());
            CHECK_FALSE(c.retard_is_curve);
            CHECK(c.spark_retard_deg == 4.0);
        }
        auto rejects = [](const nlohmann::json& retard, const std::string& message) {
            nlohmann::json j = read_json(controller_path());
            j["spark_retard_deg"] = retard;
            const TempJson t("n2o", j);
            CHECK_THROWS_WITH(io::load_nitrous_controller_json(t.str()), ContainsSubstring(message));
        };
        rejects({{"fraction", {0.0, 0.5, 0.5}}, {"deg", {0.0, 2.0, 8.0}}}, "must be strictly ascending");
        rejects({{"fraction", {0.0, 1.0}}, {"deg", {0.0, -1.0}}}, "'deg' must be >= 0");
        rejects({{"fraction", {0.0, 1.0}}, {"deg", {0.0, 2.0}}, {"bogus", 1}}, "unknown key 'bogus'");
    }
}

// ---------------------------------------------------------------------------
// C3: neutral and reverse never spray - also when a code-built desc says min_gear 0
// or -1 (the loader rejects it, the system enforces it anyway); the gear window and
// the minimum road speed; whole vehicle in neutral.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C3: neutral and reverse never spray, gear and speed windows", "[unit][n2o][drivetrain][io]") {
    Parts parts = fixture_parts();
    auto opens = [&](const Parts& p, int gear, bool gearbox = true, real speed = 20.0) {
        auto sys = make_system(p);
        NitrousInputs in = go();
        in.gear = gear;
        in.has_gearbox = gearbox;
        in.speed_mps = speed;
        return sys->step(in).solenoid_open;
    };

    SECTION("a code-built min_gear of 0 or -1 still excludes neutral and reverse") {
        for (int min_gear : {0, -1, -5}) {
            parts.kit.controller.min_gear = min_gear;
            INFO("min_gear " << min_gear);
            CHECK_FALSE(opens(parts, 0));
            CHECK_FALSE(opens(parts, -1));
            CHECK_FALSE(opens(parts, -3));
            CHECK(opens(parts, 1));
            CHECK(opens(parts, 2));
        }
    }
    SECTION("window 2..4") {
        parts.kit.controller.min_gear = 2;
        parts.kit.controller.max_gear = 4;
        CHECK_FALSE(opens(parts, 1));
        CHECK(opens(parts, 2));
        CHECK(opens(parts, 4));
        CHECK_FALSE(opens(parts, 5));
    }
    SECTION("a wheel-less rig has no gear condition") {
        CHECK(opens(parts, 0, /*gearbox=*/false));
    }
    SECTION("minimum road speed is inclusive") {
        parts.kit.controller.min_speed_mps = 10.0;
        CHECK_FALSE(opens(parts, 3, true, 9.99));
        CHECK(opens(parts, 3, true, 10.0));
    }
    SECTION("the loader rejects min_gear below 1") {
        for (int bad : {0, -1}) {
            nlohmann::json j = read_json(controller_path());
            j["min_gear"] = bad;
            const TempJson t("n2o", j);
            CHECK_THROWS_WITH(io::load_nitrous_controller_json(t.str()), ContainsSubstring("'min_gear' must be >= 1 (neutral and reverse never spray)"));
        }
        nlohmann::json j = read_json(controller_path());
        j["min_gear"] = 1;
        const TempJson t("n2o", j);
        CHECK_NOTHROW(io::load_nitrous_controller_json(t.str()));
    }
    SECTION("whole vehicle: in neutral at WOT the solenoid never opens, in 1st it does") {
        vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
        mutable_component_of<NitrousKitDesc>(desc).controller.min_gear = 0; // code-built: would let neutral in without the hard rule
        mutable_component_of<NitrousKitDesc>(desc).controller.min_rpm = 600.0;
        mutable_component_of<NitrousKitDesc>(desc).controller.max_rpm = 9000.0;
        Rig neutral = make_rig(desc, 1, /*gear=*/0);
        Rig first = make_rig(desc, 1, /*gear=*/1);
        for (Rig* r : {&neutral, &first}) {
            run(*r, 60);
            r->world->set_control("throttle", 1.0);
            r->world->set_control("nitrous_arm", 1.0);
        }
        int open_neutral = 0, open_first = 0;
        for (int t = 0; t < 60; ++t) {
            neutral.world->step();
            first.world->step();
            open_neutral += ess(neutral).n2o_solenoid_open;
            open_first += ess(first).n2o_solenoid_open;
        }
        CHECK(open_neutral == 0);
        CHECK(neutral.world->powertrain_energy(neutral.vehicle).n2o_mass == 0.0);
        CHECK(open_first > 0); // non-vacuous: the same rig sprays in gear
        CHECK(first.world->powertrain_energy(first.vehicle).n2o_mass > 0.0);
    }
}

// ---------------------------------------------------------------------------
// C4: the max_active_s duty cut at an exact substep count, and the cut enum through
// the whole vehicle. max_active_s 0.0505 s at h = 1 ms: t_active after k open
// substeps is k ms, the check t_active > 0.0505 first fires at k = 51, so exactly 51
// substeps (0..50) are open.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C4: the duty-time cut and the cut enum", "[unit][n2o][drivetrain]") {
    Parts parts = fixture_parts();
    SECTION("exact open count, latch, re-arm, progressive restarts") {
        parts.kit.controller.safety.max_active_s = 0.0505;
        parts.kit.controller.progressive.start_fraction = 0.5;
        parts.kit.controller.progressive.delay_s = 0.0;
        parts.kit.controller.progressive.ramp_s = 0.1;
        auto sys = make_system(parts);
        int open_steps = 0;
        int first_cut = -1;
        real last_open_fraction = 0.0;
        for (int k = 0; k < 120; ++k) {
            const NitrousStatus& s = sys->step(go());
            if (s.solenoid_open) {
                ++open_steps;
                last_open_fraction = s.flow_fraction;
            }
            if (s.safety_cut == NitrousSafetyCut::DutyTime && first_cut < 0) first_cut = k;
            if (k > 51) {
                CHECK(s.safety_cut == NitrousSafetyCut::DutyTime); // latched while the command stays high
                CHECK_FALSE(s.solenoid_open);
                CHECK(s.mdot_n_kg_s == 0.0);
            }
        }
        CHECK(open_steps == 51);
        CHECK(first_cut == 51);
        CHECK(last_open_fraction == Catch::Approx(0.5 + 0.5 * 0.050 / 0.1).epsilon(1e-9)); // t_active 0.050 at the last open substep
        NitrousInputs lift = go();
        lift.throttle_command = 0.5;
        CHECK(sys->step(lift).safety_cut == NitrousSafetyCut::None);
        const NitrousStatus& s = sys->step(go());
        CHECK(s.solenoid_open);
        CHECK(s.flow_fraction == Catch::Approx(0.5).epsilon(1e-12)); // the ramp restarted with the active time
        CHECK(sys->activations() == 2);
    }
    SECTION("whole vehicle: duty cut publishes enum 3 and clears on a lift") {
        vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
        NitrousControllerDesc& c = mutable_component_of<NitrousKitDesc>(desc).controller;
        c.min_rpm = 600.0;
        c.max_rpm = 9000.0;
        c.safety.max_active_s = 0.1;
        Rig rig = make_rig(desc);
        run(rig, 60);
        rig.world->set_control("throttle", 1.0);
        rig.world->set_control("nitrous_arm", 1.0);
        bool saw_open = false, saw_cut = false;
        for (int t = 0; t < 60; ++t) {
            rig.world->step();
            if (ess(rig).n2o_solenoid_open) saw_open = true;
            if (ess(rig).n2o_safety_cut == 3) {
                saw_cut = true;
                CHECK(ess(rig).n2o_solenoid_open == 0);
                CHECK(ess(rig).n2o_active == 0);
                CHECK(ess(rig).n2o_flow_g_s == 0.0);
            }
        }
        CHECK(saw_open);
        CHECK(saw_cut);
        CHECK(ess(rig).n2o_safety_cut == 3); // still latched at WOT
        rig.world->set_control("throttle", 0.0);
        run(rig, 4);
        CHECK(ess(rig).n2o_safety_cut == 0);
    }
    SECTION("whole vehicle: a bottle below the pressure switch publishes enum 1") {
        vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
        NitrousControllerDesc& c = mutable_component_of<NitrousKitDesc>(desc).controller;
        c.min_rpm = 600.0;
        c.max_rpm = 9000.0;
        mutable_component_of<N2oBottleDesc>(desc).pressure.gauge_bar = 50.0; // below the fixture's 55 bar switch
        Rig rig = make_rig(desc);
        run(rig, 60);
        rig.world->set_control("throttle", 1.0);
        rig.world->set_control("nitrous_arm", 1.0);
        run(rig, 10);
        CHECK(ess(rig).n2o_safety_cut == 1);
        CHECK(ess(rig).n2o_solenoid_open == 0);
        CHECK(rig.world->powertrain_energy(rig.vehicle).n2o_mass == 0.0);
    }
}

// ---------------------------------------------------------------------------
// C5: sound counters and fields through the whole vehicle.
//  n2o_activations = solenoid rising edges (3 arm presses -> 3),
//  n2o_purge_events = once per press (3 held presses over many ticks -> 3, none with
//  an empty bottle), n2o_flow_fraction follows the plan formula at the last substep
//  of each tick, n2o_solenoid_open/n2o_purge_active/n2o_injection.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C5: sound counters and fields", "[unit][n2o][drivetrain]") {
    vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
    NitrousControllerDesc& c = mutable_component_of<NitrousKitDesc>(desc).controller;
    c.min_rpm = 600.0;
    c.max_rpm = 9000.0;
    c.progressive.start_fraction = 0.25;
    c.progressive.delay_s = 0.05;
    c.progressive.ramp_s = 0.2;
    mutable_component_of<NitrousKitDesc>(desc).injection = NitrousInjection::DirectPort;

    SECTION("activations count rising edges") {
        Rig rig = make_rig(desc);
        run(rig, 60);
        rig.world->set_control("throttle", 1.0);
        CHECK(ess(rig).n2o_activations == 0);
        for (int press = 0; press < 3; ++press) {
            rig.world->set_control("nitrous_arm", 1.0);
            run(rig, 10); // held for 10 ticks: one activation, not 40
            CHECK(ess(rig).n2o_activations == static_cast<std::uint32_t>(press + 1));
            rig.world->set_control("nitrous_arm", 0.0);
            run(rig, 5);
            CHECK(ess(rig).n2o_solenoid_open == 0);
            CHECK(ess(rig).n2o_activations == static_cast<std::uint32_t>(press + 1));
        }
    }
    SECTION("purge events: once per press, none without liquid") {
        Rig rig = make_rig(desc);
        run(rig, 30);
        CHECK(ess(rig).n2o_purge_events == 0);
        CHECK(ess(rig).n2o_purge_active == 0);
        for (int press = 0; press < 3; ++press) {
            rig.world->set_control("nitrous_purge", 1.0);
            run(rig, 20);
            CHECK(ess(rig).n2o_purge_active == 1);
            CHECK(ess(rig).n2o_purge_flow_g_s > 0.0);
            CHECK(ess(rig).n2o_purge_events == static_cast<std::uint32_t>(press + 1));
            rig.world->set_control("nitrous_purge", 0.0);
            run(rig, 5);
            CHECK(ess(rig).n2o_purge_active == 0);
            CHECK(ess(rig).n2o_purge_events == static_cast<std::uint32_t>(press + 1));
        }
        // An empty bottle has nothing to vent: a press is not an event.
        rig.world->set_n2o_bottle_mass(rig.vehicle, 0.0);
        run(rig, 2);
        rig.world->set_control("nitrous_purge", 1.0);
        run(rig, 10);
        CHECK(ess(rig).n2o_purge_active == 0);
        CHECK(ess(rig).n2o_purge_events == 3);
    }
    SECTION("flow fraction, solenoid flag and injection follow the controller") {
        Rig rig = make_rig(desc);
        run(rig, 60);
        CHECK(ess(rig).n2o_flow_fraction == 0.0);
        CHECK(ess(rig).n2o_solenoid_open == 0);
        CHECK(ess(rig).n2o_injection == 1);
        rig.world->set_control("throttle", 1.0);
        rig.world->set_control("nitrous_arm", 1.0);
        // The solenoid opens at the first substep of the first tick; the sound state is the last substep of the
        // tick, 4n + 3 substeps after the opening (t_active = (4n + 3)/960 at its start).
        int n_opened = -1;
        int checked = 0;
        for (int n = 0; n < 40; ++n) {
            rig.world->step();
            if (!ess(rig).n2o_solenoid_open) continue;
            if (n_opened < 0) n_opened = n;
            const real t = static_cast<real>(4 * (n - n_opened) + 3) * kHs;
            const real expect = std::clamp(0.25 + 0.75 * (t - 0.05) / 0.2, 0.0, 1.0);
            const real expect_total = t >= 0.05 ? expect : 0.25;
            CHECK(ess(rig).n2o_flow_fraction == Catch::Approx(expect_total).epsilon(1e-9));
            ++checked;
        }
        CHECK(n_opened == 0);
        CHECK(checked == 40);
        CHECK(ess(rig).n2o_flow_fraction > 0.25);
        CHECK(ess(rig).n2o_flow_fraction < 1.0); // 40 ticks = 0.167 s: still inside the ramp
        CHECK(ess(rig).n2o_solenoid_open == 1);
        CHECK(ess(rig).n2o_active == 1);
        // Disarmed: the fraction falls to 0 with the solenoid.
        rig.world->set_control("nitrous_arm", 0.0);
        run(rig, 2);
        CHECK(ess(rig).n2o_flow_fraction == 0.0);
        CHECK(ess(rig).n2o_solenoid_open == 0);
    }
    SECTION("an empty bottle: the solenoid flag stays up, nothing flows") {
        Rig rig = make_rig(desc);
        run(rig, 60);
        rig.world->set_control("throttle", 1.0);
        rig.world->set_n2o_bottle_mass(rig.vehicle, 0.0);
        run(rig, 2);
        rig.world->set_control("nitrous_arm", 1.0);
        run(rig, 10);
        CHECK(ess(rig).n2o_solenoid_open == 1);
        CHECK(ess(rig).n2o_activations == 1);
        CHECK(ess(rig).n2o_active == 0);
        CHECK(ess(rig).n2o_flow_fraction == 0.0);
        CHECK(ess(rig).n2o_flow_g_s == 0.0);
    }
    SECTION("a plenum kit reports injection 0") {
        mutable_component_of<NitrousKitDesc>(desc).injection = NitrousInjection::Plenum;
        Rig rig = make_rig(desc);
        run(rig, 4);
        CHECK(ess(rig).n2o_injection == 0);
    }
}

// ---------------------------------------------------------------------------
// C6: the vehicle-level controller override. Bottom-up: the kit's default
// controller first, then the vehicle component's "controller" wins. Rejections;
// the heater/arm/purge channels must be three different names.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C6: the vehicle controller override wins over the kit default", "[unit][n2o][io][drivetrain]") {
    const nlohmann::json base = read_json(n2o_sedan_path());
    auto component = [](nlohmann::json& v, const std::string& id) -> nlohmann::json& {
        for (auto& c : v["powertrain"]["components"]) {
            if (c["id"] == id) return c;
        }
        FAIL("fixture has no component " << id);
        throw std::logic_error("unreachable");
    };
    auto rejects = [](const nlohmann::json& v, const std::string& message) {
        const TempJson t("", v);
        CHECK_THROWS_WITH(io::load_vehicle_json(t.str()), ContainsSubstring(message));
    };
    // An override controller differing from the default in every N4 field.
    nlohmann::json oj = read_json(controller_path());
    oj["arm_channel"] = "x_arm";
    oj["purge"]["channel"] = "x_purge";
    oj["min_gear"] = 2;
    oj["progressive"] = {{"start_fraction", 0.3}, {"delay_s", 0.02}, {"ramp_s", 0.5}};
    oj["spark_retard_deg"] = {{"fraction", {0.0, 1.0}}, {"deg", {0.0, 6.0}}};
    oj["safety"]["max_active_s"] = 7.0;

    SECTION("without an override the kit's default controller is used") {
        const vehicle::VehicleDesc d = io::load_vehicle_json(n2o_sedan_path());
        const NitrousControllerDesc& c = component_of<NitrousKitDesc>(d).controller;
        CHECK(c.arm_channel == "nitrous_arm");
        CHECK(c.min_gear == 1);
        CHECK(c.progressive.start_fraction == 1.0);
        CHECK_FALSE(c.retard_is_curve);
        CHECK(c.safety.max_active_s == 15.0);
    }
    SECTION("with an override every controller field comes from it") {
        const TempJson tc("n2o", oj);
        nlohmann::json v = base;
        component(v, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
        const TempJson t("", v);
        const vehicle::VehicleDesc d = io::load_vehicle_json(t.str());
        const NitrousKitDesc& k = component_of<NitrousKitDesc>(d);
        CHECK(k.controller.arm_channel == "x_arm");
        CHECK(k.controller.purge.channel == "x_purge");
        CHECK(k.controller.min_gear == 2);
        CHECK(k.controller.progressive.start_fraction == 0.3);
        CHECK(k.controller.progressive.ramp_s == 0.5);
        CHECK(k.controller.retard_is_curve);
        CHECK(k.controller.safety.max_active_s == 7.0);
        // The kit's own fields are untouched by the override.
        CHECK(k.nitrous_jet_cd == 0.66);
        CHECK(k.nitrous_jet_diameter_mm == 1.4478);
        // The declared channels follow the override: x_arm/x_purge, no nitrous_arm/nitrous_purge.
        const auto decls = Powertrain::declare_control_channels(d.powertrain);
        auto has = [&](const std::string& n) {
            return std::any_of(decls.begin(), decls.end(), [&](const ChannelDecl& c) { return c.name == n; });
        };
        CHECK(has("x_arm"));
        CHECK(has("x_purge"));
        CHECK(has("nitrous_heater"));
        CHECK_FALSE(has("nitrous_arm"));
        CHECK_FALSE(has("nitrous_purge"));

        // Whole vehicle: the override's arm channel arms the kit, the default's does not.
        vehicle::VehicleDesc dd = d;
        NitrousControllerDesc& c = mutable_component_of<NitrousKitDesc>(dd).controller;
        c.min_rpm = 600.0;
        c.max_rpm = 9000.0;
        c.min_gear = 1; // the whole-vehicle check is about the channel, keep 1st gear valid
        Rig rig = make_rig(dd);
        run(rig, 60);
        rig.world->set_control("throttle", 1.0);
        rig.world->set_control("nitrous_arm", 1.0); // the kit default's channel: not wired any more
        run(rig, 20);
        CHECK(ess(rig).n2o_solenoid_open == 0);
        CHECK(rig.world->powertrain_energy(rig.vehicle).n2o_mass == 0.0);
        rig.world->set_control("x_arm", 1.0);
        run(rig, 20);
        CHECK(ess(rig).n2o_solenoid_open == 1);
        CHECK(rig.world->powertrain_energy(rig.vehicle).n2o_mass > 0.0);
    }
    SECTION("override rejections") {
        nlohmann::json v = base;
        component(v, "n2o_kit")["controller"] = 3;
        rejects(v, "'controller' is required and must be a non-empty string");
        v = base;
        component(v, "n2o_kit")["controller"] = "";
        rejects(v, "'controller' is required and must be a non-empty string");
        v = base;
        component(v, "n2o_kit")["controller"] = "n2o/no_such_controller.json";
        rejects(v, "no_such_controller.json");
        v = base;
        {
            nlohmann::json bad = oj;
            bad["format"] = "physics_sim.nitrous_controller/2";
            const TempJson tc("n2o", bad);
            component(v, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
            rejects(v, "unsupported 'format'");
        }
        v = base;
        {
            nlohmann::json bad = oj;
            bad["min_gear"] = 0;
            const TempJson tc("n2o", bad);
            component(v, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
            rejects(v, "'min_gear' must be >= 1");
        }
        // The override may introduce a clash with the bottle heater channel (nitrous_heater): rejected.
        v = base;
        {
            nlohmann::json bad = oj;
            bad["arm_channel"] = "nitrous_heater";
            const TempJson tc("n2o", bad);
            component(v, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
            rejects(v, "the bottle heater channel 'nitrous_heater' is also the arm channel of nitrous_kit 'n2o_kit' controller");
        }
        v = base;
        {
            nlohmann::json bad = oj;
            bad["purge"]["channel"] = "nitrous_heater";
            const TempJson tc("n2o", bad);
            component(v, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
            rejects(v, "the bottle heater channel 'nitrous_heater' is also the purge channel of nitrous_kit 'n2o_kit' controller");
        }
        // Positive control for the same file shape: a distinct name loads.
        v = base;
        {
            const TempJson tc("n2o", oj);
            component(v, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
            const TempJson t("", v);
            CHECK_NOTHROW(io::load_vehicle_json(t.str()));
        }
    }
    SECTION("Powertrain backstop for a code-built desc") {
        auto expect_create_throws = [](const vehicle::VehicleDesc& d, const std::string& message) {
            WorldConfig cfg;
            cfg.job_workers = 1;
            World w(cfg);
            BodyDesc chassis;
            chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
            chassis.motion = BodyMotionType::Dynamic;
            chassis.mass = 1500.0;
            chassis.pose.position = Vec3{0.0, 0.0, 0.6};
            const BodyId cb = w.create_body(chassis);
            REQUIRE_THROWS_WITH(w.create_vehicle(d, cb), ContainsSubstring(message));
        };
        const vehicle::VehicleDesc good = io::load_vehicle_json(n2o_sedan_path());
        vehicle::VehicleDesc d = good;
        mutable_component_of<NitrousKitDesc>(d).controller.arm_channel = "nitrous_heater";
        expect_create_throws(d, "the bottle heater channel 'nitrous_heater' is also the nitrous controller's arm channel");
        d = good;
        mutable_component_of<NitrousKitDesc>(d).controller.purge.channel = "nitrous_heater";
        expect_create_throws(d, "the bottle heater channel 'nitrous_heater' is also the nitrous controller's purge channel");
        d = good;
        mutable_component_of<NitrousKitDesc>(d).controller.purge.channel = mutable_component_of<NitrousKitDesc>(d).controller.arm_channel;
        expect_create_throws(d, "purge channel 'nitrous_arm' is also its arm channel");
    }
}

// ---------------------------------------------------------------------------
// C7: determinism and allocation with a progressive controller: 1 vs 10 workers,
// the solenoid toggling (arm toggles, a duty cut), purge, heater, debug_alloc armed
// (own count 0; the external count is the known Jolt QuadTree gap).
// ---------------------------------------------------------------------------
namespace {
vehicle::VehicleDesc progressive_desc(real ramp_s) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
    NitrousControllerDesc& c = mutable_component_of<NitrousKitDesc>(desc).controller;
    c.min_rpm = 600.0;
    c.max_rpm = 9000.0;
    c.progressive.start_fraction = 0.3;
    c.progressive.delay_s = 0.03;
    c.progressive.ramp_s = ramp_s;
    c.retard_is_curve = true;
    c.retard_vs_fraction.x = {0.0, 1.0};
    c.retard_vs_fraction.y = {0.0, 6.0};
    c.safety.max_active_s = 0.4;
    return desc;
}

struct C7Result {
    std::uint64_t hash = 0;
    std::uint32_t activations = 0;
    bool saw_partial_fraction = false;
    bool saw_duty_cut = false;
};

C7Result run_c7(int workers, bool arm_alloc, real ramp_s = 0.25) {
    Rig rig = make_rig(progressive_desc(ramp_s), workers);
    World& w = *rig.world;
    run(rig, 120);
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    C7Result r;
    w.set_control("throttle", 1.0);
    w.set_control("nitrous_arm", 1.0);
    for (int t = 0; t < 420; ++t) {
        if (t == 40) w.set_control("nitrous_arm", 0.0);
        if (t == 60) w.set_control("nitrous_arm", 1.0);
        if (t == 150) w.set_control("nitrous_heater", 1.0);
        if (t == 200) w.set_control("throttle", 0.0); // lift: re-arms a duty cut
        if (t == 210) w.set_control("throttle", 1.0);
        if (t == 300) w.set_control("nitrous_purge", 1.0);
        if (t == 330) w.set_control("nitrous_purge", 0.0);
        w.step();
        const EngineSoundState& e = rig.world->powertrain_state(rig.vehicle).engines.at(0);
        if (e.n2o_flow_fraction > 0.3 && e.n2o_flow_fraction < 1.0) r.saw_partial_fraction = true;
        if (e.n2o_safety_cut == 3) r.saw_duty_cut = true;
    }
    if (arm) {
        da::set_armed(false);
        const auto own = da::own_violation_count();
        const auto ext = da::external_violation_count();
        WARN("C7 debug_alloc (" << workers << " workers): own " << own << ", external (Jolt) " << ext);
        CHECK(own == 0);
    }
    r.hash = w.state_hash();
    r.activations = rig.world->powertrain_state(rig.vehicle).engines.at(0).n2o_activations;
    return r;
}
} // namespace

TEST_CASE("N4 C7: a progressive controller hashes identically on 1 and 10 workers, allocation-free", "[unit][n2o][drivetrain][jobs][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const C7Result a = run_c7(1, true);
    const C7Result b = run_c7(10, true);
    CHECK(a.hash == b.hash);
    CHECK(a.activations == b.activations);
    // Non-vacuous: the run really ramped, toggled and hit the duty cut.
    CHECK(a.saw_partial_fraction);
    CHECK(a.saw_duty_cut);
    CHECK(a.activations >= 3);
    // The ramp state is part of the hashed state: a different ramp gives a different hash.
    const C7Result c = run_c7(1, false, 0.5);
    CHECK(c.hash != a.hash);
}

// ---------------------------------------------------------------------------
// C8: a vehicle without a kit is unchanged - no sound-field output, no channels;
// a kit that is never armed leaves every dynamic bit-identical to the kit-less
// vehicle and every N4 sound field 0.
// ---------------------------------------------------------------------------
TEST_CASE("N4 C8: kit-less and never-armed vehicles are bit-identical, N4 sound fields 0", "[unit][n2o][drivetrain]") {
    const vehicle::VehicleDesc plain = io::load_vehicle_json(tank_sedan_path());
    const vehicle::VehicleDesc kit = io::load_vehicle_json(n2o_sedan_path());
    Rig a = make_rig(plain);
    Rig b = make_rig(kit);
    for (Rig* r : {&a, &b}) r->world->set_control("throttle", 1.0);
    long long first_mismatch = -1;
    for (int t = 0; t < 400; ++t) {
        a.world->step();
        b.world->step();
        const Pose pa = a.world->get_pose(a.chassis);
        const Pose pb = b.world->get_pose(b.chassis);
        bool same = pa.position.x == pb.position.x && pa.position.y == pb.position.y && pa.position.z == pb.position.z;
        for (std::size_t i = 0; i < a.world->vehicle_wheel_count(a.vehicle); ++i) same = same && a.world->wheel_omega(a.vehicle, i) == b.world->wheel_omega(b.vehicle, i);
        const PowertrainEnergy& ea = a.world->powertrain_energy(a.vehicle);
        const PowertrainEnergy& eb = b.world->powertrain_energy(b.vehicle);
        same = same && ea.engine_out == eb.engine_out && ea.fuel_mass == eb.fuel_mass && ea.balance_error == eb.balance_error;
        same = same && eb.n2o_mass == 0.0 && eb.n2o_purged == 0.0 && ea.n2o_mass == 0.0;
        if (!same && first_mismatch < 0) first_mismatch = t;
        for (const Rig* r : {&a, &b}) {
            const EngineSoundState& e = ess(*r);
            CHECK(e.n2o_solenoid_open == 0);
            CHECK(e.n2o_flow_fraction == 0.0);
            CHECK(e.n2o_purge_active == 0);
            CHECK(e.n2o_activations == 0);
            CHECK(e.n2o_purge_events == 0);
            CHECK(e.n2o_safety_cut == 0);
        }
        CHECK(ess(a).n2o_injection == 0); // kit-less: no kit property
    }
    INFO("first mismatching tick: " << first_mismatch);
    CHECK(first_mismatch == -1);
}
