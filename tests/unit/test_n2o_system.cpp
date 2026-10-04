// tests/unit/test_n2o_system.cpp -- N2O plan N1: the nitrous bottle, kit and
// controller part files, the Powertrain-owned NitrousSystem
// (core/src/drivetrain/nitrous_system.h), its control channels, the N2O mass in
// the sub-ledger and the public surface (World::n2o_bottle_mass/
// set_n2o_bottle_mass, EngineSoundState n2o_* fields, the kit-only
// vehicle<i>.n2o.* telemetry channels). The engine IGNORES the N2O in N1.
//
// Fixtures (tests/support/fixtures/, never shipped data):
//   n2o/bottle_10lb.json, n2o/kit_wet.json, n2o/controller_basic.json
//   vehicle_n2o_sedan.json  vehicle_tank_sedan.json + bottle + kit + the two
//                           fluid links (the engine's dynamics are bit-identical
//                           to vehicle_tank_sedan.json in N1).
// Expected values are literals from the fixture files or recomputed from the
// loaded public descs and the oxidizer table, never read back from the code
// under test (the HEM table itself is pinned by test_n2o_flow.cpp F2).
//
//   S1 every request condition switches the solenoid (one SECTION each)
//   S2 bottle closure incl. purge and the emptying substep (exactly 0.0)
//   S3 flows equal Cd*A*G at the frozen bottle state
//   S4 pressure modes, the heater channel
//   S5 each safety cut latches and re-arms on a lift
//   S6 a kit-less vehicle is unchanged (channels, CSV header, dynamics)
//   S7 1 vs 10 workers
//   S8 zero allocation
//   S9 every loader rejection message

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
#include <functional>
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
std::string bottle_path() { return fixture_path("n2o/bottle_10lb.json"); }
std::string kit_path() { return fixture_path("n2o/kit_wet.json"); }
std::string controller_path() { return fixture_path("n2o/controller_basic.json"); }

nlohmann::json read_json(const std::string& path) {
    std::ifstream in(path);
    nlohmann::json j;
    in >> j;
    return j;
}

// A temporary JSON file next to the fixtures (the vehicle/kit fixtures use
// relative paths), removed again on scope exit.
struct TempJson {
    std::filesystem::path path;
    TempJson(const std::string& dir_rel, const nlohmann::json& j) {
        static int counter = 0;
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = std::filesystem::path(fixture_path(dir_rel)) / ("_tmp_n2o_" + std::to_string(stamp) + "_" + std::to_string(counter++) + ".json");
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
// heater/arm/purge slots 0/1/2 here (white-box: the inputs carry the values directly).
std::unique_ptr<NitrousSystem> make_system(const Parts& p) { return std::make_unique<NitrousSystem>(p.bottle, p.kit, e5_density(), 1, 0, 2); }

constexpr real kH = 1.0 / 960.0;

// Everything the fixture controller asks for: armed, WOT, 3000 rpm, 3rd gear, 20 m/s, running.
NitrousInputs go() {
    NitrousInputs in;
    in.h = kH;
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

real jet_area_m2(real d_mm) { return 0.25 * kPi * (d_mm * 1e-3) * (d_mm * 1e-3); }

// ---- whole-vehicle rig ------------------------------------------------------

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

Rig make_rig(const vehicle::VehicleDesc& desc, int workers = 1) {
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
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

const EngineSoundState& engine_state(const Rig& r) { return r.world->powertrain_state(r.vehicle).engines.at(0); }

// initial - m_bottle - (n2o_mass + n2o_purged - n2o_external_added): the bottle closure residual.
real closure_residual(const Rig& r, real initial_kg) {
    const PowertrainEnergy& e = r.world->powertrain_energy(r.vehicle);
    return (initial_kg - r.world->n2o_bottle_mass(r.vehicle)) - (e.n2o_mass + e.n2o_purged - e.n2o_external_added);
}

} // namespace

// ---------------------------------------------------------------------------
// S1: every request condition switches the solenoid. One SECTION per condition,
// literal thresholds from the fixture controller (arm 0.5, command 0.95,
// 1500-6000 rpm, gear >= 1), each probed just outside and just inside.
// ---------------------------------------------------------------------------
TEST_CASE("N1 S1: every request condition switches the solenoid", "[unit][n2o][drivetrain]") {
    Parts parts = fixture_parts();
    REQUIRE(parts.kit.controller.min_throttle_command == 0.95);
    REQUIRE(parts.kit.controller.min_rpm == 1500.0);
    REQUIRE(parts.kit.controller.max_rpm == 6000.0);
    REQUIRE(parts.kit.controller.min_gear == 1);

    auto opens = [&](const Parts& p, const NitrousInputs& in, bool engine_cut_prev = false) {
        auto sys = make_system(p);
        sys->observe(engine_cut_prev, false);
        return sys->step(in).solenoid_open;
    };

    SECTION("positive control: the base inputs open it and N2O flows") {
        auto sys = make_system(parts);
        const NitrousStatus& s = sys->step(go());
        CHECK(s.solenoid_open);
        CHECK(s.flowing);
        CHECK(s.mdot_n_kg_s > 0.0);
        CHECK(s.safety_cut == NitrousSafetyCut::None);
        CHECK(sys->activations() == 1);
        CHECK(sys->engine_input().active);
    }
    SECTION("arm level") {
        NitrousInputs in = go();
        in.arm = 0.49;
        CHECK_FALSE(opens(parts, in));
        in.arm = 0.5;
        CHECK(opens(parts, in));
        in.arm = 0.0;
        CHECK_FALSE(opens(parts, in));
    }
    SECTION("wide-open-throttle command") {
        NitrousInputs in = go();
        in.throttle_command = 0.9499;
        CHECK_FALSE(opens(parts, in));
        in.throttle_command = 0.95;
        CHECK(opens(parts, in));
    }
    SECTION("rpm window") {
        NitrousInputs in = go();
        in.omega = rpm_to_omega(1499.0);
        CHECK_FALSE(opens(parts, in));
        in.omega = rpm_to_omega(1501.0);
        CHECK(opens(parts, in));
        in.omega = rpm_to_omega(5999.0);
        CHECK(opens(parts, in));
        in.omega = rpm_to_omega(6001.0);
        CHECK_FALSE(opens(parts, in));
    }
    SECTION("gear: neutral and reverse never spray") {
        NitrousInputs in = go();
        in.gear = 0;
        CHECK_FALSE(opens(parts, in));
        in.gear = -1;
        CHECK_FALSE(opens(parts, in));
        in.gear = 1;
        CHECK(opens(parts, in));
        in.has_gearbox = false; // a wheel-less rig has no gear condition
        in.gear = 0;
        CHECK(opens(parts, in));
    }
    SECTION("gear window max") {
        parts.kit.controller.max_gear = 3;
        NitrousInputs in = go();
        in.gear = 3;
        CHECK(opens(parts, in));
        in.gear = 4;
        CHECK_FALSE(opens(parts, in));
    }
    SECTION("minimum road speed") {
        parts.kit.controller.min_speed_mps = 5.0;
        NitrousInputs in = go();
        in.speed_mps = 4.99;
        CHECK_FALSE(opens(parts, in));
        in.speed_mps = 5.0;
        CHECK(opens(parts, in));
    }
    SECTION("engine must be running") {
        NitrousInputs in = go();
        in.engine_running = false;
        CHECK_FALSE(opens(parts, in));
    }
    SECTION("the TCU upshift ignition cut") {
        NitrousInputs in = go();
        in.ignition_cut = true;
        CHECK_FALSE(opens(parts, in));
    }
    SECTION("previous substep limiter or DFCO") {
        CHECK_FALSE(opens(parts, go(), /*engine_cut_prev=*/true));
        CHECK(opens(parts, go(), /*engine_cut_prev=*/false));
    }
    SECTION("an empty bottle opens the solenoid but delivers nothing") {
        auto sys = make_system(parts);
        sys->set_bottle_kg(0.0);
        const NitrousStatus& s = sys->step(go());
        CHECK(s.solenoid_open);
        CHECK_FALSE(s.flowing);
        CHECK(s.mdot_n_kg_s == 0.0);
        CHECK_FALSE(sys->engine_input().active);
    }
}

// ---------------------------------------------------------------------------
// S2: bottle closure. White-box: the sum of every commit() equals the initial
// content exactly enough (1e-15 kg), the emptying substep delivers exactly the
// rest and the bottle ends at exactly 0.0, with and without a purge. Whole
// vehicle: initial - m_bottle == n2o_mass + n2o_purged - n2o_external_added at
// every tick, in a run that includes a set_n2o_bottle_mass.
// ---------------------------------------------------------------------------
TEST_CASE("N1 S2: the bottle closes against the ledger, the emptying substep delivers exactly the rest", "[unit][n2o][drivetrain]") {
    const Parts parts = fixture_parts();

    SECTION("injection alone empties to exactly 0.0") {
        auto sys = make_system(parts);
        const real initial = 0.02;
        sys->set_bottle_kg(initial);
        real booked = 0.0;
        int emptied_at = -1;
        for (int i = 0; i < 4000 && sys->bottle_kg() > 0.0; ++i) {
            const real before = sys->bottle_kg();
            sys->step(go());
            const NitrousBooking b = sys->commit();
            booked += b.n2o_kg;
            if (sys->bottle_kg() == 0.0) {
                emptied_at = i;
                CHECK(b.n2o_kg == before); // the whole rest, bit-exactly
            }
        }
        REQUIRE(emptied_at > 0);
        CHECK(sys->bottle_kg() == 0.0);
        CHECK(std::abs(booked - initial) <= 1e-15);
        // Afterwards nothing flows although the solenoid stays open.
        const NitrousStatus& s = sys->step(go());
        CHECK(s.solenoid_open);
        CHECK(s.mdot_n_kg_s == 0.0);
        CHECK(sys->commit().n2o_kg == 0.0);
    }
    SECTION("injection and purge together") {
        auto sys = make_system(parts);
        const real initial = 0.02;
        sys->set_bottle_kg(initial);
        real booked_n = 0.0, booked_p = 0.0;
        for (int i = 0; i < 4000 && sys->bottle_kg() > 0.0; ++i) {
            NitrousInputs in = go();
            in.purge = 1.0;
            sys->step(in);
            const NitrousBooking b = sys->commit();
            booked_n += b.n2o_kg;
            booked_p += b.purge_kg;
        }
        CHECK(sys->bottle_kg() == 0.0);
        CHECK(booked_p > 0.0);
        CHECK(booked_n > 0.0);
        CHECK(std::abs(booked_n + booked_p - initial) <= 1e-15);
    }
    SECTION("purge alone empties to exactly 0.0 and never counts as injected N2O") {
        auto sys = make_system(parts);
        sys->set_bottle_kg(0.01);
        real booked_n = 0.0, booked_p = 0.0;
        for (int i = 0; i < 100000 && sys->bottle_kg() > 0.0; ++i) {
            NitrousInputs in = go();
            in.arm = 0.0; // closed solenoid
            in.purge = 1.0;
            const NitrousStatus& s = sys->step(in);
            CHECK_FALSE(s.solenoid_open);
            CHECK(s.purge_active);
            const NitrousBooking b = sys->commit();
            booked_n += b.n2o_kg;
            booked_p += b.purge_kg;
        }
        CHECK(sys->bottle_kg() == 0.0);
        CHECK(booked_n == 0.0);
        CHECK(std::abs(booked_p - 0.01) <= 1e-15);
        CHECK(sys->purge_events() == 1);
    }
    SECTION("set_bottle_kg clamps and returns the signed change") {
        auto sys = make_system(parts);
        const real cap = parts.bottle.capacity_kg;
        CHECK(sys->bottle_kg() == cap);
        CHECK(sys->set_bottle_kg(cap + 5.0) == 0.0);
        CHECK(sys->set_bottle_kg(1.0) == 1.0 - cap);
        CHECK(sys->bottle_kg() == 1.0);
        CHECK(sys->set_bottle_kg(-3.0) == -1.0);
        CHECK(sys->bottle_kg() == 0.0);
    }
    SECTION("whole vehicle: closure every tick, empties, refills") {
        const vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
        Rig rig = make_rig(desc);
        World& w = *rig.world;
        const real cap = component_of<N2oBottleDesc>(desc).capacity_kg;
        CHECK(w.n2o_bottle_mass(rig.vehicle) == cap);
        for (int t = 0; t < 120; ++t) w.step();
        w.set_n2o_bottle_mass(rig.vehicle, 0.01); // applied at the next tick boundary, booked signed
        w.set_control("nitrous_arm", 1.0);
        w.set_control("throttle", 1.0);
        real worst = 0.0;
        bool emptied = false;
        bool flowed = false;
        for (int t = 0; t < 960 && !emptied; ++t) {
            w.step();
            worst = std::max(worst, std::abs(closure_residual(rig, cap)));
            flowed = flowed || engine_state(rig).n2o_flow_g_s > 0.0;
            emptied = w.n2o_bottle_mass(rig.vehicle) == 0.0;
        }
        CHECK(flowed);
        CHECK(emptied);
        CHECK(w.n2o_bottle_mass(rig.vehicle) == 0.0);
        CHECK(worst <= 1e-12);
        // Refill, then purge only (throttle off, disarmed).
        w.set_control("nitrous_arm", 0.0);
        w.set_control("throttle", 0.0);
        w.set_n2o_bottle_mass(rig.vehicle, 1.0);
        w.set_control("nitrous_purge", 1.0);
        for (int t = 0; t < 120; ++t) {
            w.step();
            worst = std::max(worst, std::abs(closure_residual(rig, cap)));
        }
        CHECK(worst <= 1e-12);
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        CHECK(e.n2o_purged > 0.0);
        CHECK(e.n2o_unburnt == e.n2o_mass); // N1: the engine ignores the N2O, all of it is pass-through
        CHECK(e.n2o_energy_in == 0.0);
        // Two sets, both booked signed: (0.03 - cap) from the full bottle, then (1.0 - 0.0) from the emptied one.
        CHECK(std::abs(e.n2o_external_added - ((0.01 - cap) + 1.0)) <= 1e-15);
        CHECK(e.kit_fuel_mass == 0.0);
        CHECK(w.powertrain_state(rig.vehicle).n2o_bottle_kg == w.n2o_bottle_mass(rig.vehicle));
        CHECK_THROWS_WITH(w.set_n2o_bottle_mass(rig.vehicle, std::nan("")), ContainsSubstring("NaN"));
    }
}

// ---------------------------------------------------------------------------
// S3: the flows equal Cd * A * G at the frozen bottle state. Literals: 65.5 bar
// gauge + the ISA 1.01325 bar reference, jets 1.4478 mm (nitrous Cd 0.66, fuel Cd
// 0.70), fuel 0.448 bar gauge over the atmosphere. G comes from the oxidizer
// table (pinned independently by test_n2o_flow.cpp F2).
// ---------------------------------------------------------------------------
TEST_CASE("N1 S3: flows equal Cd*A*G at the frozen bottle state", "[unit][n2o][drivetrain]") {
    Parts parts = fixture_parts();
    const real p_abs = 65.5e5 + 101325.0;
    const real a_n = jet_area_m2(1.4478);
    const real rho = e5_density();

    SECTION("HEM, plenum reference atmosphere") {
        auto sys = make_system(parts);
        const real g = sys->tables().g_hem(p_abs);
        const NitrousStatus& s = sys->step(go());
        CHECK(sys->bottle_gauge_bar() == Catch::Approx(65.5).epsilon(1e-12));
        CHECK(s.bottle_p_pa == Catch::Approx(p_abs).epsilon(1e-12));
        const real expect_n = 0.66 * a_n * g;
        CHECK(s.mdot_n_kg_s == Catch::Approx(expect_n).epsilon(1e-12));
        const real dp = (101325.0 + 0.448e5) - 50000.0;
        const real expect_f = 0.70 * a_n * std::sqrt(2.0 * rho * dp);
        CHECK(s.mdot_kf_kg_s == Catch::Approx(expect_f).epsilon(1e-12));
        CHECK(s.retard_deg == 4.0);
        // The same state a substep later, then the booking in kg.
        const NitrousBooking b = sys->commit();
        CHECK(b.n2o_kg == Catch::Approx(expect_n * kH).epsilon(1e-12));
        const NitrousInput ni = sys->engine_input();
        CHECK(ni.mdot_n_kg_s == Catch::Approx(expect_n).epsilon(1e-12));
        CHECK(ni.mdot_kf_kg_s == Catch::Approx(expect_f).epsilon(1e-12));
        CHECK(ni.retard_deg == 4.0);
        CHECK(ni.injection == 0);
        CHECK(ni.h_in_j_kg == Catch::Approx(sys->tables().liquid_at_pressure(p_abs).h_l_j_kg).epsilon(1e-12));
        // A real-world sanity bound (NOS/Edelbrock jet charts: a 57 jet at 950 psig flows about 0.03-0.05 kg/s).
        CHECK(s.mdot_n_kg_s > 0.02);
        CHECK(s.mdot_n_kg_s < 0.06);
    }
    SECTION("manifold fuel reference: the pressure drop is the gauge") {
        parts.kit.fuel_reference = NitrousFuelReference::Manifold;
        auto sys = make_system(parts);
        const NitrousStatus& s = sys->step(go());
        const real expect_f = 0.70 * a_n * std::sqrt(2.0 * rho * 0.448e5);
        CHECK(s.mdot_kf_kg_s == Catch::Approx(expect_f).epsilon(1e-12));
    }
    SECTION("Dyer uses the manifold pressure as the downstream pressure") {
        parts.kit.flow_model = NitrousFlowModel::Dyer;
        auto sys = make_system(parts);
        const NitrousStatus& s = sys->step(go());
        const real g = sys->tables().g_dyer(p_abs, 50000.0);
        CHECK(s.mdot_n_kg_s == Catch::Approx(0.66 * a_n * g).epsilon(1e-12));
        CHECK(g != sys->tables().g_hem(p_abs));
    }
    SECTION("nozzle count multiplies both flows") {
        parts.kit.nozzle_count = 4;
        auto sys = make_system(parts);
        const real g = sys->tables().g_hem(p_abs);
        const NitrousStatus& s = sys->step(go());
        CHECK(s.mdot_n_kg_s == Catch::Approx(4.0 * 0.66 * a_n * g).epsilon(1e-12));
    }
    SECTION("progressive duty: start fraction, delay, ramp") {
        parts.kit.controller.progressive.start_fraction = 0.5;
        parts.kit.controller.progressive.delay_s = 0.1;
        parts.kit.controller.progressive.ramp_s = 0.2;
        auto sys = make_system(parts);
        const real g = sys->tables().g_hem(p_abs);
        std::vector<real> fractions;
        for (int i = 0; i < 400; ++i) {
            const NitrousStatus& s = sys->step(go());
            fractions.push_back(s.flow_fraction);
            // do not commit: the bottle stays full, the bottle state frozen
            if (i == 0) CHECK(s.mdot_n_kg_s == Catch::Approx(0.5 * 0.66 * a_n * g).epsilon(1e-12));
        }
        CHECK(fractions[0] == 0.5);
        CHECK(fractions[50] == 0.5);                  // t = 0.052 < delay
        CHECK(fractions[144] == Catch::Approx(0.5 + 0.5 * (144 * kH - 0.1) / 0.2).epsilon(1e-9));
        CHECK(fractions[399] == 1.0);                 // t = 0.416 past delay + ramp
        for (std::size_t i = 1; i < fractions.size(); ++i) CHECK(fractions[i] >= fractions[i - 1]);
    }
    SECTION("retard follows a curve over the flow fraction") {
        parts.kit.controller.retard_is_curve = true;
        parts.kit.controller.retard_vs_fraction.x = {0.0, 1.0};
        parts.kit.controller.retard_vs_fraction.y = {0.0, 8.0};
        parts.kit.controller.progressive.start_fraction = 0.5;
        parts.kit.controller.progressive.delay_s = 10.0;
        auto sys = make_system(parts);
        CHECK(sys->step(go()).retard_deg == Catch::Approx(4.0).epsilon(1e-12));
    }
    SECTION("a closed solenoid delivers nothing and no kit fuel") {
        auto sys = make_system(parts);
        NitrousInputs in = go();
        in.arm = 0.0;
        const NitrousStatus& s = sys->step(in);
        CHECK(s.mdot_n_kg_s == 0.0);
        CHECK(s.mdot_kf_kg_s == 0.0);
        CHECK(s.retard_deg == 0.0);
        const NitrousInput ni = sys->engine_input();
        CHECK_FALSE(ni.active);
        CHECK(ni.mdot_n_kg_s == 0.0);
    }
}

// ---------------------------------------------------------------------------
// S4: pressure modes and the heater.
// ---------------------------------------------------------------------------
TEST_CASE("N1 S4: bottle pressure modes and the heater channel", "[unit][n2o][drivetrain]") {
    Parts parts = fixture_parts();
    NitrousInputs idle = go();
    idle.arm = 0.0;

    SECTION("constant") {
        auto sys = make_system(parts);
        CHECK(sys->step(idle).bottle_p_pa == Catch::Approx(65.5e5 + 101325.0).epsilon(1e-12));
        sys->set_bottle_kg(0.5);
        CHECK(sys->step(idle).bottle_p_pa == Catch::Approx(65.5e5 + 101325.0).epsilon(1e-12)); // independent of the fill
    }
    SECTION("vs_fill: the pressure falls as the bottle empties") {
        parts.bottle.pressure.mode = N2oPressureMode::VsFill;
        parts.bottle.pressure.gauge_vs_fill.x = {0.0, 1.0};
        parts.bottle.pressure.gauge_vs_fill.y = {40.0, 60.0};
        parts.kit.controller.safety.min_bottle_gauge_bar = 0.0; // no bottle-pressure switch: the low-pressure bottle may spray
        auto sys = make_system(parts);
        const real cap = parts.bottle.capacity_kg;
        sys->set_bottle_kg(cap);
        CHECK(sys->step(idle).bottle_p_pa == Catch::Approx(60.0e5 + 101325.0).epsilon(1e-12));
        sys->set_bottle_kg(0.5 * cap);
        CHECK(sys->step(idle).bottle_p_pa == Catch::Approx(50.0e5 + 101325.0).epsilon(1e-12));
        sys->set_bottle_kg(0.0);
        CHECK(sys->step(idle).bottle_p_pa == Catch::Approx(40.0e5 + 101325.0).epsilon(1e-12));
        // And while spraying: monotone non-increasing, ending lower.
        sys->set_bottle_kg(0.5);
        real prev = 1e30;
        real first = 0.0, last = 0.0;
        for (int i = 0; i < 2000 && sys->bottle_kg() > 0.0; ++i) {
            const real p = sys->step(go()).bottle_p_pa;
            sys->commit();
            if (i == 0) first = p;
            last = p;
            CHECK(p <= prev);
            prev = p;
        }
        CHECK(last < first);
    }
    SECTION("temperature: the saturation pressure at the bottle temperature") {
        parts.bottle.pressure.mode = N2oPressureMode::Temperature;
        parts.bottle.pressure.temperature_k = 290.0;
        auto sys = make_system(parts);
        const real p = sys->step(idle).bottle_p_pa;
        CHECK(p == sys->tables().saturation_pressure_pa(290.0));
        CHECK(p > 46.0e5); // NIST: 47.0 bar at 290 K
        CHECK(p < 48.0e5);
        CHECK(sys->status().bottle_t_k == Catch::Approx(290.0).epsilon(1e-6));
    }
    SECTION("heater: the channel replaces the pressure by the set point (bar, then K)") {
        auto sys = make_system(parts);
        NitrousInputs hot = idle;
        hot.heater = 1.0;
        CHECK(sys->step(hot).bottle_p_pa == Catch::Approx(68.0e5 + 101325.0).epsilon(1e-12));
        CHECK(sys->step(idle).bottle_p_pa == Catch::Approx(65.5e5 + 101325.0).epsilon(1e-12));
        parts.bottle.heater.set_point_is_temperature = true;
        parts.bottle.heater.set_point_k = 300.0;
        auto sys2 = make_system(parts);
        CHECK(sys2->step(hot).bottle_p_pa == sys2->tables().saturation_pressure_pa(300.0));
    }
    SECTION("a bottle without a heater ignores the channel") {
        parts.bottle.has_heater = false;
        auto sys = make_system(parts);
        NitrousInputs hot = idle;
        hot.heater = 1.0;
        CHECK(sys->step(hot).bottle_p_pa == Catch::Approx(65.5e5 + 101325.0).epsilon(1e-12));
    }
    SECTION("whole vehicle: the heater switch takes effect at the next tick") {
        const vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
        Rig rig = make_rig(desc);
        World& w = *rig.world;
        for (int t = 0; t < 4; ++t) w.step();
        CHECK(engine_state(rig).n2o_bottle_bar == Catch::Approx(65.5).epsilon(1e-9));
        w.set_control("nitrous_heater", 1.0);
        w.step();
        CHECK(engine_state(rig).n2o_bottle_bar == Catch::Approx(68.0).epsilon(1e-9));
        w.set_control("nitrous_heater", 0.0);
        w.step();
        CHECK(engine_state(rig).n2o_bottle_bar == Catch::Approx(65.5).epsilon(1e-9));
    }
}

// ---------------------------------------------------------------------------
// S5: each safety cut latches until the command drops below the switch and then
// re-arms.
// ---------------------------------------------------------------------------
TEST_CASE("N1 S5: each safety cut latches and re-arms on a lift", "[unit][n2o][drivetrain]") {
    Parts parts = fixture_parts();

    SECTION("bottle-pressure switch") {
        parts.bottle.pressure.gauge_bar = 50.0; // below the 55 bar switch
        auto sys = make_system(parts);
        {
            const NitrousStatus& s = sys->step(go());
            CHECK_FALSE(s.solenoid_open);
            CHECK(s.safety_cut == NitrousSafetyCut::BottlePressure);
        }
        NitrousInputs warm = go();
        warm.heater = 1.0; // pressure restored to 68 bar ...
        {
            const NitrousStatus& s = sys->step(warm);
            CHECK_FALSE(s.solenoid_open); // ... but the cut holds while the command stays high
            CHECK(s.safety_cut == NitrousSafetyCut::BottlePressure);
        }
        NitrousInputs lift = warm;
        lift.throttle_command = 0.5;
        {
            const NitrousStatus& s = sys->step(lift);
            CHECK_FALSE(s.solenoid_open);
            CHECK(s.safety_cut == NitrousSafetyCut::None); // re-armed by the lift
        }
        {
            const NitrousStatus& s = sys->step(warm);
            CHECK(s.solenoid_open);
            CHECK(s.safety_cut == NitrousSafetyCut::None);
        }
    }
    SECTION("fuel-starve switch") {
        auto sys = make_system(parts);
        sys->observe(false, true);
        {
            const NitrousStatus& s = sys->step(go());
            CHECK_FALSE(s.solenoid_open);
            CHECK(s.safety_cut == NitrousSafetyCut::FuelStarve);
        }
        sys->observe(false, false);
        CHECK(sys->step(go()).safety_cut == NitrousSafetyCut::FuelStarve); // latched
        NitrousInputs lift = go();
        lift.throttle_command = 0.5;
        CHECK(sys->step(lift).safety_cut == NitrousSafetyCut::None);
        CHECK(sys->step(go()).solenoid_open);
    }
    SECTION("fuel-starve switch disabled") {
        parts.kit.controller.safety.cut_on_fuel_starve = false;
        auto sys = make_system(parts);
        sys->observe(false, true);
        CHECK(sys->step(go()).solenoid_open);
    }
    SECTION("solenoid duty time") {
        parts.kit.controller.safety.max_active_s = 0.01;
        auto sys = make_system(parts);
        int open_steps = 0;
        int cut_index = -1;
        for (int i = 0; i < 40; ++i) {
            const NitrousStatus& s = sys->step(go());
            if (s.solenoid_open) ++open_steps;
            if (s.safety_cut == NitrousSafetyCut::DutyTime && cut_index < 0) cut_index = i;
        }
        CHECK(cut_index == 10); // t_active = 10/960 = 0.0104 s > 0.01 s
        CHECK(open_steps == 10);
        NitrousInputs lift = go();
        lift.throttle_command = 0.5;
        CHECK(sys->step(lift).safety_cut == NitrousSafetyCut::None);
        const NitrousStatus& s = sys->step(go());
        CHECK(s.solenoid_open); // re-armed, active time restarted
        CHECK(sys->active_time_s() == Catch::Approx(kH).epsilon(1e-12));
    }
    SECTION("no limit when max_active_s is 0") {
        parts.kit.controller.safety.max_active_s = 0.0;
        auto sys = make_system(parts);
        for (int i = 0; i < 3000; ++i) CHECK(sys->step(go()).solenoid_open);
    }
    SECTION("the cut reason is published as sound state") {
        const vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
        Rig rig = make_rig(desc);
        World& w = *rig.world;
        for (int t = 0; t < 120; ++t) w.step();
        CHECK(engine_state(rig).n2o_safety_cut == 0);
        CHECK(engine_state(rig).n2o_active == 0);
    }
}

// ---------------------------------------------------------------------------
// S6: a vehicle without a kit is unchanged: no declared nitrous channels, no
// n2o CSV columns, no hashed bytes; and (the engine ignores N2O in N1) a vehicle
// WITH a kit has bit-identical dynamics every tick while its state hash differs.
// ---------------------------------------------------------------------------
TEST_CASE("N1 S6: a kit-less vehicle declares no nitrous channel, column or hash", "[unit][n2o][drivetrain]") {
    const vehicle::VehicleDesc plain = io::load_vehicle_json(tank_sedan_path());
    const vehicle::VehicleDesc kit = io::load_vehicle_json(n2o_sedan_path());

    const auto plain_channels = Powertrain::declare_control_channels(plain.powertrain);
    const auto kit_channels = Powertrain::declare_control_channels(kit.powertrain);
    for (const auto& c : plain_channels) CHECK(c.name.find("nitrous") == std::string::npos);
    REQUIRE(kit_channels.size() == plain_channels.size() + 3);
    // Components append the bottle (heater) then the kit (arm, purge), each a plain 0/1 level, default 0.
    CHECK(kit_channels[plain_channels.size() + 0].name == "nitrous_heater");
    CHECK(kit_channels[plain_channels.size() + 1].name == "nitrous_arm");
    CHECK(kit_channels[plain_channels.size() + 2].name == "nitrous_purge");
    for (std::size_t i = plain_channels.size(); i < kit_channels.size(); ++i) CHECK(kit_channels[i].default_value == 0.0);

    Rig a = make_rig(plain);
    Rig b = make_rig(kit);
    const auto& names_a = a.world->telemetry().channel_names();
    const auto& names_b = b.world->telemetry().channel_names();
    for (const std::string& n : names_a) CHECK(n.find(".n2o.") == std::string::npos);
    int n2o_columns = 0;
    for (const std::string& n : names_b) {
        if (n.find("vehicle0.n2o.") == 0) ++n2o_columns;
    }
    CHECK(n2o_columns == 4);
    CHECK(names_b.size() == names_a.size() + 4);
    for (const char* col : {"vehicle0.n2o.flow_g_s", "vehicle0.n2o.bottle_kg", "vehicle0.n2o.bottle_bar", "vehicle0.n2o.active"}) {
        CHECK(std::find(names_b.begin(), names_b.end(), col) != names_b.end());
    }

    // Drive both identically (the kit world sprays, purges and heats): the engine ignores it.
    long long first_mismatch = -1;
    World& wa = *a.world;
    World& wb = *b.world;
    for (World* w : {&wa, &wb}) w->set_control("throttle", 1.0);
    wb.set_control("nitrous_arm", 1.0);
    for (int t = 0; t < 600; ++t) {
        if (t == 300) {
            wb.set_control("nitrous_purge", 1.0);
            wb.set_control("nitrous_heater", 1.0);
        }
        wa.step();
        wb.step();
        const Pose pa = wa.get_pose(a.chassis);
        const Pose pb = wb.get_pose(b.chassis);
        bool same = pa.position.x == pb.position.x && pa.position.y == pb.position.y && pa.position.z == pb.position.z;
        for (std::size_t i = 0; i < wa.vehicle_wheel_count(a.vehicle); ++i) same = same && wa.wheel_omega(a.vehicle, i) == wb.wheel_omega(b.vehicle, i);
        const auto& sa = wa.powertrain_state(a.vehicle);
        const auto& sb = wb.powertrain_state(b.vehicle);
        same = same && sa.engine_rpm == sb.engine_rpm && sa.fuel_flow_g_s == sb.fuel_flow_g_s;
        const PowertrainEnergy& ea = wa.powertrain_energy(a.vehicle);
        const PowertrainEnergy& eb = wb.powertrain_energy(b.vehicle);
        same = same && ea.engine_out == eb.engine_out && ea.fuel_mass == eb.fuel_mass && ea.engine_heat == eb.engine_heat &&
               ea.balance_error == eb.balance_error;
        // The kit-less ledger stays exactly zero in the nitrous fields.
        same = same && ea.n2o_mass == 0.0 && ea.n2o_purged == 0.0 && ea.n2o_unburnt == 0.0 && ea.n2o_energy_in == 0.0 &&
               ea.n2o_external_added == 0.0 && ea.kit_fuel_mass == 0.0;
        if (!same && first_mismatch < 0) first_mismatch = t;
    }
    INFO("first mismatching tick: " << first_mismatch);
    CHECK(first_mismatch == -1);
    CHECK(wb.state_hash() != wa.state_hash());          // the kit state is hashed
    CHECK(wb.powertrain_energy(b.vehicle).n2o_mass > 0.0); // non-vacuous: N2O really flowed
    CHECK(wa.n2o_bottle_mass(a.vehicle) == 0.0);
    CHECK(wa.powertrain_state(a.vehicle).n2o_bottle_kg == 0.0);
    CHECK(wa.powertrain_state(a.vehicle).engines.at(0).n2o_flow_g_s == 0.0);
    CHECK_THROWS_WITH(wa.set_n2o_bottle_mass(a.vehicle, 1.0), ContainsSubstring("no n2o_bottle"));
}

// ---------------------------------------------------------------------------
// S7/S8: determinism and allocation. 1 vs 10 workers with the solenoid toggling,
// a purge, the heater, an emptying and a refill, debug_alloc armed (own count 0;
// the external count is the known Jolt QuadTree gap).
// ---------------------------------------------------------------------------
namespace {
std::uint64_t run_s7(int workers, bool arm_alloc, real* bottle_out) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(n2o_sedan_path());
    mutable_component_of<N2oBottleDesc>(desc).initial_kg = 0.05;
    Rig rig = make_rig(desc, workers);
    World& w = *rig.world;
    for (int t = 0; t < 240; ++t) w.step(); // warm-up + settle
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    w.set_control("throttle", 1.0);
    w.set_control("nitrous_arm", 1.0);
    for (int t = 0; t < 360; ++t) {
        if (t == 100) w.set_control("nitrous_heater", 1.0);
        if (t == 200) w.set_control("nitrous_arm", 0.0);
        if (t == 240) w.set_control("nitrous_arm", 1.0);
        w.step();
    }
    w.set_n2o_bottle_mass(rig.vehicle, 0.5);
    w.set_control("nitrous_purge", 1.0);
    w.set_control("nitrous_arm", 0.0);
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 240; ++t) w.step();
    if (arm) {
        da::set_armed(false);
        const auto own = da::own_violation_count();
        const auto ext = da::external_violation_count();
        WARN("S8 debug_alloc (" << workers << " workers): own " << own << ", external (Jolt) " << ext);
        CHECK(own == 0);
    }
    *bottle_out = w.n2o_bottle_mass(rig.vehicle);
    CHECK(w.powertrain_energy(rig.vehicle).n2o_mass > 0.0);
    CHECK(w.powertrain_energy(rig.vehicle).n2o_purged > 0.0);
    return w.state_hash();
}
} // namespace

TEST_CASE("N1 S7/S8: an active kit hashes identically on 1 and 10 workers, allocation-free", "[unit][n2o][drivetrain][jobs][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    real b1 = 0.0, b10 = 0.0;
    const std::uint64_t h1 = run_s7(1, true, &b1);
    const std::uint64_t h10 = run_s7(10, true, &b10);
    CHECK(h1 == h10);
    CHECK(b1 == b10);

    // The bottle content is part of the state hash.
    vehicle::VehicleDesc a = io::load_vehicle_json(n2o_sedan_path());
    vehicle::VehicleDesc c = a;
    mutable_component_of<N2oBottleDesc>(c).initial_kg = 4.0;
    Rig ra = make_rig(a);
    Rig rc = make_rig(c);
    for (int t = 0; t < 10; ++t) {
        ra.world->step();
        rc.world->step();
    }
    CHECK(ra.world->state_hash() != rc.world->state_hash());
}

// ---------------------------------------------------------------------------
// S9: every loader rejection, exact message text.
// ---------------------------------------------------------------------------
TEST_CASE("N1 S9: bottle, kit and controller loaders reject every malformed input", "[unit][n2o][io]") {
    SECTION("positive controls and the loaded values") {
        const N2oBottleDesc b = io::load_n2o_bottle_json(bottle_path());
        CHECK(b.capacity_kg == 4.536);
        CHECK(b.initial_kg == 4.536);
        CHECK(b.pressure.mode == N2oPressureMode::Constant);
        CHECK(b.pressure.gauge_bar == 65.5);
        REQUIRE(b.has_heater);
        CHECK(b.heater.channel == "nitrous_heater");
        CHECK(b.heater.set_point_gauge_bar == 68.0);
        const NitrousKitDesc k = io::load_nitrous_kit_json(kit_path());
        CHECK(k.nitrous_jet_diameter_mm == 1.4478);
        CHECK(k.nitrous_jet_cd == 0.66);
        CHECK(k.fuel_jet_cd == 0.7);
        CHECK(k.fuel_pressure_gauge_bar == 0.448);
        CHECK(k.controller.arm_channel == "nitrous_arm");
        CHECK(k.controller.arm_default == 0.0);
        CHECK(k.controller.has_purge);
        CHECK(k.controller.safety.max_active_s == 15.0);
        // A controller override replaces the kit's default.
        nlohmann::json cj = read_json(controller_path());
        cj["arm_channel"] = "my_arm";
        cj["purge"]["channel"] = "my_purge";
        const TempJson tc("n2o", cj);
        const NitrousKitDesc k2 = io::load_nitrous_kit_json(kit_path(), tc.str());
        CHECK(k2.controller.arm_channel == "my_arm");
        // Direct controller load.
        CHECK(io::load_nitrous_controller_json(controller_path()).min_rpm == 1500.0);
    }

    auto bottle_rejects = [](const std::function<void(nlohmann::json&)>& mutate, const std::string& message) {
        nlohmann::json j = read_json(bottle_path());
        mutate(j);
        const TempJson t("n2o", j);
        CHECK_THROWS_WITH(io::load_n2o_bottle_json(t.str()), ContainsSubstring(message));
    };
    auto kit_rejects = [](const std::function<void(nlohmann::json&)>& mutate, const std::string& message) {
        nlohmann::json j = read_json(kit_path());
        mutate(j);
        const TempJson t("n2o", j);
        CHECK_THROWS_WITH(io::load_nitrous_kit_json(t.str()), ContainsSubstring(message));
    };
    auto controller_rejects = [](const std::function<void(nlohmann::json&)>& mutate, const std::string& message) {
        nlohmann::json j = read_json(controller_path());
        mutate(j);
        const TempJson t("n2o", j);
        CHECK_THROWS_WITH(io::load_nitrous_controller_json(t.str()), ContainsSubstring(message));
    };

    SECTION("bottle") {
        bottle_rejects([](auto& j) { j["format"] = "physics_sim.n2o_bottle/2"; }, "unsupported 'format'");
        bottle_rejects([](auto& j) { j.erase("format"); }, "missing required 'format' field");
        bottle_rejects([](auto& j) { j["bogus"] = 1; }, "unknown key 'bogus'");
        bottle_rejects([](auto& j) { j["capacity_kg"] = 0.0; }, "'capacity_kg' must be > 0");
        bottle_rejects([](auto& j) { j.erase("source"); }, "'source' is required and must be a non-empty string");
        bottle_rejects([](auto& j) { j["outlet"] = "dip_tube"; }, "is not supported (only \"siphon\"");
        bottle_rejects([](auto& j) { j["pressure"]["gauge_pa"] = 1.0; }, "key 'gauge_pa' is not accepted - pressures are in bar");
        bottle_rejects([](auto& j) { j["pressure"]["gauge_bar"] = 80.0; }, "outside the oxidizer table's usable range");
        bottle_rejects([](auto& j) { j["pressure"]["mode"] = "bogus"; }, "unknown pressure mode \"bogus\"");
        bottle_rejects([](auto& j) { j["pressure"] = {{"mode", "temperature"}, {"temperature_k", 100.0}}; }, "outside the oxidizer table");
        bottle_rejects([](auto& j) { j["pressure"] = {{"mode", "temperature"}, {"temperature_k", 290.0}, {"gauge_bar", 60.0}}; },
                       "mode \"temperature\" takes only 'temperature_k'");
        bottle_rejects([](auto& j) { j["pressure"] = {{"mode", "vs_fill"}, {"fill_fraction", {0.1, 1.0}}, {"gauge_bar", {40.0, 60.0}}}; },
                       "'fill_fraction' must span exactly 0 to 1");
        bottle_rejects([](auto& j) { j["pressure"] = {{"mode", "vs_fill"}, {"fill_fraction", {0.0, 0.5, 0.5, 1.0}}, {"gauge_bar", {40.0, 50.0, 55.0, 60.0}}}; },
                       "'fill_fraction' must be strictly ascending");
        bottle_rejects([](auto& j) { j["pressure"] = {{"mode", "vs_fill"}, {"fill_fraction", {0.0, 1.0}}, {"gauge_bar", {40.0}}}; },
                       "must be arrays of the same length");
        bottle_rejects([](auto& j) { j["heater"] = {{"channel", "h"}, {"set_point_gauge_bar", 60.0}, {"set_point_k", 295.0}}; },
                       "exactly one of 'set_point_gauge_bar' and 'set_point_k' is required");
        bottle_rejects([](auto& j) { j["heater"] = {{"channel", "h"}}; }, "exactly one of 'set_point_gauge_bar' and 'set_point_k' is required");
        bottle_rejects([](auto& j) { j["heater"] = {{"channel", ""}, {"set_point_gauge_bar", 60.0}}; }, "'channel' is required and must be a non-empty string");
        bottle_rejects([](auto& j) { j["heater"]["set_point_gauge_bar"] = 90.0; }, "outside the oxidizer table's usable range");
    }
    SECTION("kit") {
        kit_rejects([](auto& j) { j["format"] = "physics_sim.nitrous_kit/9"; }, "unsupported 'format'");
        kit_rejects([](auto& j) { j["bogus"] = 1; }, "unknown key 'bogus'");
        kit_rejects([](auto& j) { j["type"] = "dry"; }, "type \"dry\" is not supported yet");
        kit_rejects([](auto& j) { j["type"] = "mist"; }, "unknown type \"mist\"");
        kit_rejects([](auto& j) { j["injection"] = "carb"; }, "unknown injection \"carb\"");
        kit_rejects([](auto& j) { j["nozzle_count"] = 0; }, "'nozzle_count' must be >= 1");
        kit_rejects([](auto& j) { j["nitrous_jet_cd"] = 1.2; }, "'nitrous_jet_cd' must be in (0, 1]");
        kit_rejects([](auto& j) { j["nitrous_jet_diameter_mm"] = 0.0; }, "'nitrous_jet_diameter_mm' must be > 0");
        kit_rejects([](auto& j) { j["fuel_jet_cd"] = 0.0; }, "'fuel_jet_cd' must be in (0, 1]");
        kit_rejects([](auto& j) { j["flow_model"] = "spi"; }, "unknown flow_model \"spi\"");
        kit_rejects([](auto& j) { j["fuel_pressure"]["gauge_bar"] = -0.1; }, "'gauge_bar' must be >= 0");
        kit_rejects([](auto& j) { j["fuel_pressure"]["reference"] = "tank"; }, "unknown reference \"tank\"");
        kit_rejects([](auto& j) { j["fuel_pressure"]["gauge_psi"] = 6.5; }, "key 'gauge_psi' is not accepted");
        kit_rejects([](auto& j) { j.erase("fuel_pressure"); }, "'fuel_pressure' is required and must be an object");
        kit_rejects([](auto& j) { j["rating"] = {{"power_kw", 74.57}, {"bottle_gauge_bar", 65.5}, {"basis", "crank"}}; }, "'source' is required");
        kit_rejects([](auto& j) { j["rating"] = {{"power_kw", 74.57}, {"bottle_gauge_bar", 65.5}, {"basis", "hub"}, {"source", "x"}}; },
                    "'basis' must be \"crank\" or \"wheel\"");
        kit_rejects([](auto& j) { j.erase("default_controller"); }, "no controller");
        kit_rejects([](auto& j) { j.erase("source"); }, "'source' is required and must be a non-empty string");
    }
    SECTION("controller") {
        controller_rejects([](auto& j) { j["format"] = "physics_sim.nitrous_controller/2"; }, "unsupported 'format'");
        controller_rejects([](auto& j) { j["bogus"] = 1; }, "unknown key 'bogus'");
        controller_rejects([](auto& j) { j.erase("arm_channel"); }, "'arm_channel' is required and must be a non-empty string");
        controller_rejects([](auto& j) { j["min_throttle_command"] = 1.5; }, "'min_throttle_command' must be in [0, 1]");
        controller_rejects([](auto& j) { j["max_rpm"] = 1000.0; }, "'max_rpm' must be above 'min_rpm'");
        controller_rejects([](auto& j) { j["min_rpm"] = -1.0; }, "'min_rpm' must be >= 0");
        controller_rejects([](auto& j) { j["max_gear"] = 0; }, "'max_gear' must be >= 'min_gear'");
        controller_rejects([](auto& j) { j["min_speed_mps"] = -1.0; }, "'min_speed_mps' must be >= 0");
        controller_rejects([](auto& j) { j["progressive"]["start_fraction"] = 0.0; }, "'start_fraction' must be in (0, 1]");
        controller_rejects([](auto& j) { j["progressive"]["delay_s"] = -1.0; }, "'delay_s' must be >= 0");
        controller_rejects([](auto& j) { j["progressive"]["ramp_s"] = -1.0; }, "'ramp_s' must be >= 0");
        controller_rejects([](auto& j) { j["progressive"]["bogus"] = 1.0; }, "unknown key 'bogus'");
        controller_rejects([](auto& j) { j["spark_retard_deg"] = -2.0; }, "must be finite and >= 0");
        controller_rejects([](auto& j) { j["spark_retard_deg"] = "four"; }, "must be a number or a {\"fraction\": [...], \"deg\": [...]} curve");
        controller_rejects([](auto& j) { j["spark_retard_deg"] = {{"fraction", {0.0, 1.0}}, {"deg", {0.0}}}; }, "must be arrays of the same length");
        controller_rejects([](auto& j) { j["spark_retard_deg"] = {{"fraction", {0.0, 1.5}}, {"deg", {0.0, 4.0}}}; }, "'fraction' must lie in [0, 1]");
        controller_rejects([](auto& j) { j["safety"]["max_active_s"] = 0.0; }, "'max_active_s' must be > 0");
        controller_rejects([](auto& j) { j["safety"]["min_bottle_gauge_bar"] = -1.0; }, "'min_bottle_gauge_bar' must be >= 0");
        controller_rejects([](auto& j) { j["safety"]["cut_on_fuel_starve"] = "yes"; }, "'cut_on_fuel_starve' must be 0/1 or a boolean");
        controller_rejects([](auto& j) { j["purge"]["channel"] = "nitrous_arm"; }, "the purge channel must differ from the arm channel");
        controller_rejects([](auto& j) { j["purge"]["cd"] = 2.0; }, "'cd' must be in (0, 1]");
        controller_rejects([](auto& j) { j["purge"]["jet_diameter_mm"] = 0.0; }, "'jet_diameter_mm' must be > 0");
        controller_rejects([](auto& j) { j.erase("source"); }, "'source' is required and must be a non-empty string");
    }
}

TEST_CASE("N1 S9b: the vehicle loader and the Powertrain constructor reject a malformed nitrous setup", "[unit][n2o][io][drivetrain]") {
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

    SECTION("positive control: the base fixture and a component override load") {
        {
            const TempJson t("", base);
            CHECK_NOTHROW(io::load_vehicle_json(t.str()));
        }
        nlohmann::json v = base;
        component(v, "n2o_bottle")["initial_kg"] = 2.0;
        component(v, "n2o_bottle")["pressure"] = {{"mode", "constant"}, {"gauge_bar", 60.0}};
        const TempJson t("", v);
        const vehicle::VehicleDesc d = io::load_vehicle_json(t.str());
        const N2oBottleDesc& b = component_of<N2oBottleDesc>(d);
        CHECK(b.initial_kg == 2.0);
        CHECK(b.capacity_kg == 4.536);
        CHECK(b.pressure.gauge_bar == 60.0);
        // A controller override at the component.
        nlohmann::json cj = read_json(controller_path());
        cj["arm_channel"] = "x_arm";
        cj["purge"]["channel"] = "x_purge";
        const TempJson tc("n2o", cj);
        nlohmann::json v2 = base;
        component(v2, "n2o_kit")["controller"] = std::string("n2o/") + tc.path.filename().string();
        const TempJson t2("", v2);
        CHECK(component_of<NitrousKitDesc>(io::load_vehicle_json(t2.str())).controller.arm_channel == "x_arm");
    }
    SECTION("component fields") {
        nlohmann::json v = base;
        component(v, "n2o_bottle")["initial_kg"] = 5.0;
        rejects(v, "'initial_kg' (5.000000) must not exceed the bottle's capacity_kg (4.536000)");
        v = base;
        component(v, "n2o_bottle")["initial_kg"] = -1.0;
        rejects(v, "'initial_kg' must not be negative");
        v = base;
        component(v, "n2o_bottle")["bogus"] = 1;
        rejects(v, "unknown key 'bogus'");
        v = base;
        component(v, "n2o_bottle").erase("ref");
        rejects(v, "'ref' is required and must be a non-empty path string");
        v = base;
        component(v, "n2o_bottle")["pressure"] = {{"mode", "constant"}, {"gauge_bar", 90.0}};
        rejects(v, "outside the oxidizer table's usable range");
        v = base;
        component(v, "n2o_kit")["bogus"] = 1;
        rejects(v, "unknown key 'bogus'");
    }
    SECTION("fluid links") {
        nlohmann::json v = base;
        v["powertrain"]["fluid"] = nlohmann::json::array({{"tank.out", "engine.fuel"}, {"n2o_kit.out", "engine.nitrous"}});
        rejects(v, "n2o_bottle has no consumer");
        v = base;
        v["powertrain"]["fluid"] = nlohmann::json::array({{"tank.out", "engine.fuel"}, {"n2o_bottle.out", "n2o_kit.n2o"}});
        rejects(v, "nitrous_kit has no consumer");
        v = base;
        v["powertrain"]["fluid"][1] = nlohmann::json::array({"n2o_bottle.out", "engine.nitrous"});
        rejects(v, "is not a nitrous_kit");
        v = base;
        v["powertrain"]["fluid"][1] = nlohmann::json::array({"n2o_bottle.out", "n2o_kit.fuel"});
        rejects(v, "unknown port \"n2o_kit.fuel\"");
        v = base;
        v["powertrain"]["fluid"][1] = nlohmann::json::array({"n2o_bottle.tap", "n2o_kit.n2o"});
        rejects(v, "unknown port \"n2o_bottle.tap\"");
        v = base;
        v["powertrain"]["fluid"][2] = nlohmann::json::array({"n2o_kit.out", "clutch.nitrous"});
        rejects(v, "is not an engine");
        v = base;
        v["powertrain"]["fluid"][2] = nlohmann::json::array({"n2o_kit.out", "engine.fuel"});
        rejects(v, "unknown port \"engine.fuel\"");
        v = base;
        v["powertrain"]["fluid"].push_back(nlohmann::json::array({"n2o_kit.out", "engine.nitrous"}));
        rejects(v, "nitrous port is fed by two kits");
        v = base;
        v["powertrain"]["fluid"].push_back(nlohmann::json::array({"n2o_bottle.out", "n2o_kit.n2o"}));
        rejects(v, "n2o port is fed by two bottles");
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
            const BodyId c = w.create_body(chassis);
            REQUIRE_THROWS_WITH(w.create_vehicle(d, c), ContainsSubstring(message));
        };
        const vehicle::VehicleDesc good = io::load_vehicle_json(n2o_sedan_path());
        vehicle::VehicleDesc d = good;
        d.powertrain.fluid.pop_back();
        expect_create_throws(d, "nitrous_kit 'n2o_kit' has no consumer");
        d = good;
        d.powertrain.fluid.erase(d.powertrain.fluid.begin() + 1);
        expect_create_throws(d, "n2o_bottle 'n2o_bottle' is not linked to a nitrous_kit");
        d = good;
        d.powertrain.fluid[2].to.port = "crank";
        expect_create_throws(d, "a nitrous_kit's \"out\" port feeds an engine's \"nitrous\" port");
        d = good;
        d.powertrain.fluid[1].to.port = "fuel";
        expect_create_throws(d, "an n2o_bottle's \"out\" port feeds a nitrous_kit's \"n2o\" port");
    }
}
