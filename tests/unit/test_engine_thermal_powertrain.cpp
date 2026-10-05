// tests/unit/test_engine_thermal_powertrain.cpp - engine thermal plan T1 integration tests
// (out/engine_thermal_plan.md section 12, vault PHYS-063): the thermal model inside Powertrain /
// VehicleInstance / World / EngineRig. Tags [thermal][drivetrain].
//
// C3 content: U9 first half (a pinned k = 1 leaves both engine kinds' trajectories bit-identical to the same engine
// without thermal), the World plumbing (snapshot, telemetry channels, state set/get, ambient start, hash coverage,
// ledger closure) and the World-tick half of U8 (zero own allocation). Later commits add I1-I7 and I9 below.
//
// Sabotage record (each applied once, the named check then fails):
//   C3a Powertrain::fold_state_hash without the thermal fold -> "thermal state is hashed" CHECK(h1 != h_perturbed) fails.
//   C3b a heap allocation in Powertrain::end_tick (after thermal_->step) -> U8 World half: own violations 720.
//   C3c the Coulomb-row share dropped from accumulate() -> thermal_friction_in == 0 (the "thermal ledger" CHECK).
//   C3d ambient start disabled -> coolant 91.0 / oil 95.0 instead of -20 C ("ambient start soaks").
//   (the EngineRig setters case shares the ambient-start mechanism of C3d; U9a's sabotage lands with C4/C5.)

#include "drivetrain/engine_thermal.h"
#include "drivetrain/powertrain.h"
#include "powertrain_multi_diff_desc.h"

#include "ps/drivetrain/engine_rig.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

struct Sample {
    real omega, fuel_mass, engine_out, indicated_work, fuel_flow;
    int state;
    bool operator==(const Sample& o) const {
        return omega == o.omega && fuel_mass == o.fuel_mass && engine_out == o.engine_out &&
               indicated_work == o.indicated_work && fuel_flow == o.fuel_flow && state == o.state;
    }
};

// Starts the engine, holds the starter 1.5 s, then drives a slow throttle sweep (free rev: the limiter is reached).
std::vector<Sample> run_rig(const std::string& engine_path, const EngineRig::Config& cfg, int ticks, real pin_k, real* max_omega) {
    EngineRig rig(engine_path, io::EngineMapOptions{}, nullptr, cfg);
    if (pin_k > 0.0) rig.set_thermal_friction_scale_override_for_test(pin_k);
    rig.set_ignition(1.0);
    std::vector<Sample> out;
    out.reserve(static_cast<std::size_t>(ticks));
    real mx = 0.0;
    for (int t = 0; t < ticks; ++t) {
        rig.set_starter(t < 360 ? 1.0 : 0.0);
        const real sweep = static_cast<real>((t / 4) % 720) / 720.0;
        rig.set_throttle(t < 480 ? 0.0 : 0.15 + 0.7 * sweep);
        rig.step();
        const EngineSoundState& s = rig.engine_sound_state();
        const PowertrainEnergy& l = rig.ledger();
        out.push_back(Sample{rig.engine_omega_rad_s(), l.fuel_mass, l.engine_out, l.indicated_work, s.fuel_flow_g_s,
                             static_cast<int>(rig.engine_state())});
        mx = std::max(mx, rig.engine_omega_rad_s());
    }
    if (max_omega) *max_omega = mx;
    return out;
}

void expect_identical(const std::vector<Sample>& a, const std::vector<Sample>& b) {
    REQUIRE(a.size() == b.size());
    std::size_t first_diff = a.size();
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!(a[i] == b[i])) {
            first_diff = i;
            break;
        }
    }
    INFO("first differing tick: " << first_diff);
    CHECK(first_diff == a.size());
}

void u9a(const std::string& engine_rel) {
    const std::string engine = data_path(engine_rel);
    EngineRig::Config plain;
    EngineRig::Config thermal;
    thermal.cooling_system_path = data_path("cooling_systems/sedan.json");
    real mx_plain = 0.0, mx_pinned = 0.0, mx_natural = 0.0;
    const auto base = run_rig(engine, plain, 2400, 0.0, &mx_plain);
    // The engine actually ran (guards against a vacuous comparison): started, passed idle by a wide margin.
    CHECK(mx_plain > 400.0);
    CHECK(base.back().fuel_mass > 0.0);
    const auto pinned = run_rig(engine, thermal, 2400, 1.0, &mx_pinned);
    expect_identical(base, pinned);
    // A warm start (oil exactly at the reference temperature, then warming) never leaves k = 1.0 either.
    const auto natural = run_rig(engine, thermal, 2400, 0.0, &mx_natural);
    expect_identical(base, natural);
}

} // namespace

TEST_CASE("U9a graph engine with a thermal model and k = 1 is bit-identical to the same engine without", "[thermal][drivetrain]") {
    u9a("engines/i4_2l_na.json");
}

TEST_CASE("U9a simulated engine with a thermal model and k = 1 is bit-identical to the same engine without", "[thermal][drivetrain]") {
    u9a("engines/toyota_a25a_fks_sim.json");
}

TEST_CASE("EngineRig thermal setters and rejections", "[thermal][drivetrain]") {
    EngineRig::Config plain;
    EngineRig noth(data_path("engines/i4_2l_na.json"), io::EngineMapOptions{}, nullptr, plain);
    CHECK_FALSE(noth.has_thermal());
    CHECK_FALSE(noth.thermal_snapshot().has_thermal);
    REQUIRE_THROWS_WITH(noth.engine_thermal_state(), ContainsSubstring("no cooling_system"));
    REQUIRE_THROWS_WITH(noth.set_engine_thermal_state(EngineThermalState{20.0, 20.0}), ContainsSubstring("no cooling_system"));

    EngineRig::Config c;
    c.cooling_system_path = data_path("cooling_systems/sedan.json");
    c.thermal_start = ThermalStartKind::Ambient;
    EngineRig rig(data_path("engines/i4_2l_na.json"), io::EngineMapOptions{}, nullptr, c);
    REQUIRE(rig.has_thermal());
    rig.set_thermal_env(268.15, 0.0); // -5 C
    rig.step();
    // An "ambient" start soaks both lumps to the first tick's ambient.
    CHECK(rig.engine_thermal_state().coolant_c == Approx(-5.0).margin(0.05));
    CHECK(rig.engine_thermal_state().oil_c == Approx(-5.0).margin(0.05));
    rig.set_engine_thermal_state(EngineThermalState{60.0, 40.0});
    rig.step();
    CHECK(rig.engine_thermal_state().coolant_c == Approx(60.0).margin(0.05));
    CHECK(rig.engine_thermal_state().oil_c == Approx(40.0).margin(0.05));
    REQUIRE_THROWS_WITH(rig.set_engine_thermal_state(EngineThermalState{std::nan(""), 40.0}), ContainsSubstring("finite"));

    // A cooling system for an engine file without default_thermal and no explicit path is an error.
    EngineRig::Config bad;
    bad.cooling_system_path = data_path("cooling_systems/sedan.json");
    REQUIRE_THROWS_WITH(EngineRig(data_path("engines/diesel_i4_4l5_turbo.json"), io::EngineMapOptions{}, nullptr, bad),
                        ContainsSubstring("default_thermal"));
}

// ---------------------------------------------------------------------------------------------------------------
// World plumbing
// ---------------------------------------------------------------------------------------------------------------
namespace {

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

Rig make_rig(const vehicle::VehicleDesc& desc, int workers = 1, real tick_hz = 240.0, real substep_hz = 960.0) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
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
    o.gear = 0;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

vehicle::VehicleDesc thermal_desc(const char* file = "vehicles/car_sedan_sim_thermal.json") {
    return io::load_vehicle_json(data_path(file));
}

int channel_index(World& w, const std::string& name) {
    const auto& names = w.telemetry().channel_names();
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (names[i] == name) return static_cast<int>(i);
    }
    return -1;
}

const EngineThermalSnapshot& snapshot_thermal(const Rig& r) { return r.world->powertrain_state(r.vehicle).thermal; }

} // namespace

TEST_CASE("C3 World: a thermal vehicle publishes state, snapshot, telemetry and a closed thermal ledger", "[thermal][drivetrain]") {
    for (const char* file : {"vehicles/car_sedan_sim_thermal.json", "vehicles/car_sedan_thermal.json"}) {
        INFO(file);
        Rig rig = make_rig(thermal_desc(file));
        World& w = *rig.world;
        for (int t = 0; t < 480; ++t) w.step(); // 2 s of idle on a warm start
        REQUIRE(snapshot_thermal(rig).has_thermal);
        const EngineThermalState s0 = w.engine_thermal_state(rig.vehicle);
        // Warm start: T_e = open + 0.25*(full - open) = 91 C, oil at the friction reference 95 C.
        CHECK(s0.coolant_c == Approx(91.0).margin(1.0));
        CHECK(s0.oil_c == Approx(95.0).margin(1.0));
        CHECK(snapshot_thermal(rig).friction_scale == Approx(1.0).margin(1e-3)); // oil just below T_ref after 2 s
        CHECK(snapshot_thermal(rig).gauge >= 0.0);
        CHECK(snapshot_thermal(rig).gauge <= 1.0);

        // The thermal channels exist, are recorded in the registered order, and the plain vehicle has none.
        CHECK(channel_index(w, "vehicle0.thermal.coolant_c") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.oil_c") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.thermostat") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.fan") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.q_wall_w") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.q_friction_w") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.q_radiator_w") >= 0);
        CHECK(channel_index(w, "vehicle0.thermal.friction_scale") >= 0);

        // The thermal sub-ledger closes (cumulative, relative to the flows).
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real flows = std::abs(e.thermal_wall_in) + std::abs(e.thermal_friction_in) + std::abs(e.thermal_radiator_out) +
                           std::abs(e.thermal_block_air_out) + std::abs(e.thermal_oil_air_out);
        CHECK(flows > 0.0);
        CHECK(std::abs(e.thermal_balance_error) <= 1e-9 * flows);
        CHECK(e.thermal_friction_in > 0.0); // at least the Coulomb row of a running engine

        // set_engine_thermal_state applies at the next tick boundary and moves the hash.
        const std::uint64_t h_before = w.state_hash();
        w.set_engine_thermal_state(rig.vehicle, EngineThermalState{50.0, 30.0});
        CHECK(w.state_hash() == h_before); // queued, not applied yet
        w.step();
        CHECK(w.engine_thermal_state(rig.vehicle).coolant_c == Approx(50.0).margin(0.1));
        CHECK(w.engine_thermal_state(rig.vehicle).oil_c == Approx(30.0).margin(0.2));
        CHECK(w.state_hash() != h_before);
        REQUIRE_THROWS_WITH(w.set_engine_thermal_state(rig.vehicle, EngineThermalState{std::nan(""), 1.0}), ContainsSubstring("finite"));
    }

    Rig plain = make_rig(io::load_vehicle_json(data_path("vehicles/car_sedan_sim.json")));
    plain.world->step();
    CHECK_FALSE(snapshot_thermal(plain).has_thermal);
    CHECK(channel_index(*plain.world, "vehicle0.thermal.coolant_c") < 0);
    REQUIRE_THROWS_WITH(plain.world->engine_thermal_state(plain.vehicle), ContainsSubstring("no cooling_system"));
    REQUIRE_THROWS_WITH(plain.world->set_engine_thermal_state(plain.vehicle, EngineThermalState{20.0, 20.0}), ContainsSubstring("no cooling_system"));
}

TEST_CASE("C3 World: an ambient start soaks to the vehicle's ambient at the first tick", "[thermal][drivetrain]") {
    vehicle::VehicleDesc desc = thermal_desc();
    REQUIRE(desc.powertrain.thermal.has_value());
    desc.powertrain.thermal->start = ThermalStartKind::Ambient;
    Rig rig = make_rig(desc);
    World& w = *rig.world;
    AmbientSample a;
    a.temperature_k = 253.15; // -20 C
    w.set_ambient(a);
    w.step();
    CHECK(w.engine_thermal_state(rig.vehicle).coolant_c == Approx(-20.0).margin(0.5));
    CHECK(w.engine_thermal_state(rig.vehicle).oil_c == Approx(-20.0).margin(0.5));
}

TEST_CASE("C3 World: zero own allocation per tick with a thermal vehicle (U8, World half)", "[thermal][drivetrain][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook unavailable in this build - allocation check skipped");
        return;
    }
    Rig rig = make_rig(thermal_desc());
    World& w = *rig.world;
    for (int t = 0; t < 240; ++t) w.step();
    namespace da = ps::jobs::debug_alloc;
    da::reset();
    da::set_armed(true);
    w.set_control("throttle", 0.5);
    for (int t = 0; t < 480; ++t) w.step();
    w.set_engine_thermal_state(rig.vehicle, EngineThermalState{70.0, 60.0});
    for (int t = 0; t < 240; ++t) w.step();
    da::set_armed(false);
    const auto own = da::own_violation_count();
    WARN("U8 World half: own " << own << ", external (Jolt) " << da::external_violation_count());
    CHECK(own == 0);
}

TEST_CASE("C3 World: thermal state is hashed and worker-count invariant", "[thermal][drivetrain][determinism]") {
    std::uint64_t h1 = 0, h10 = 0, h_perturbed = 0;
    for (const int workers : {1, 10}) {
        Rig rig = make_rig(thermal_desc(), workers);
        World& w = *rig.world;
        w.set_control("throttle", 0.3);
        for (int t = 0; t < 600; ++t) w.step();
        (workers == 1 ? h1 : h10) = w.state_hash();
        if (workers == 1) {
            // A different oil state at the same tick must change the hash (T_o is folded).
            Rig other = make_rig(thermal_desc(), 1);
            other.world->set_control("throttle", 0.3);
            for (int t = 0; t < 300; ++t) other.world->step();
            other.world->set_engine_thermal_state(other.vehicle, EngineThermalState{91.0, 95.5});
            for (int t = 0; t < 300; ++t) other.world->step();
            h_perturbed = other.world->state_hash();
        }
    }
    CHECK(h1 == h10);
    CHECK(h1 != h_perturbed);
}
