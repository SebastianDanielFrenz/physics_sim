// tests/unit/test_turbo_vehicle.cpp -- PHYS-033 P4 I7d D1-D4, vehicle level:
// a car (car_sedan.json's chassis/drivetrain, its engine swapped for the mvem
// electronic fixture plus a code-built hyper-like turbo pair, see
// tests/support/turbo_fixtures.h) through World. The rig mirrors
// test_simulated_engine_powertrain.cpp's make_rig (auto-clutch launch).
//
// D1: the turbo sound state reaches VehicleSnapshot::powertrain.engines[0].

#include "mvem_fixtures.h"
#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()
#include "turbo_fixtures.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <variant>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

namespace {

vehicle::VehicleDesc turbo_sedan(const std::optional<TurboPairDesc>& pair) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    SimulatedEngineDesc eng = fx::electronic_engine();
    eng.turbo_pair = pair;
    for (auto& c : d.powertrain.components) {
        if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            eng.crank_axis_local = tm->crank_axis_local;
            c.params = eng;
            break;
        }
    }
    return d;
}

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
    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
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

} // namespace

TEST_CASE("D1: the turbo sound state reaches the vehicle snapshot", "[turbo][drivetrain][vehicle]") {
    Rig turbo = make_rig(turbo_sedan(tb::hyper_like_pair()));
    Rig na = make_rig(turbo_sedan(std::nullopt));
    for (int t = 0; t < 120; ++t) { turbo.world->step(); na.world->step(); }
    turbo.world->set_control("throttle", 1.0);
    na.world->set_control("throttle", 1.0);
    real max_flow = 0.0, max_pr = 0.0, min_wg = 1.0;
    for (int t = 0; t < 360; ++t) {
        turbo.world->step();
        na.world->step();
        const auto& es = turbo.world->powertrain_state(turbo.vehicle).engines.at(0);
        max_flow = std::max(max_flow, es.compressor_flow_kg_s);
        max_pr = std::max(max_pr, es.compressor_pressure_ratio);
        min_wg = std::min(min_wg, es.wastegate_opening);
        const auto& en = na.world->powertrain_state(na.vehicle).engines.at(0);
        REQUIRE(en.wastegate_opening == 0.0);
        REQUIRE(en.compressor_flow_kg_s == 0.0);
        REQUIRE(en.compressor_pressure_ratio == 0.0);
    }
    std::printf("[D1 vehicle] WOT launch: max compressor flow %.4f kg/s, max pr %.3f, min wastegate opening %.3f\n", max_flow, max_pr, min_wg);
    CHECK(max_flow > 0.01);
    CHECK(max_pr > 1.0);
    CHECK(min_wg < 1.0);
}

// ---------------------------------------------------------------------------
// D2 at vehicle level: WOT launch then a lift with the valve present - the
// valve opens (events >= 1), the state hash is identical at 1 and 10 workers,
// and the whole run (turbo step included, inside the vehicle job) allocates
// nothing of its own while armed.
// ---------------------------------------------------------------------------
namespace {
struct VehicleRun {
    std::uint64_t hash = 0;
    std::uint32_t events = 0;
    std::uint32_t surge_events = 0;
    real max_open = 0.0, max_vent = 0.0;
};
VehicleRun run_vehicle(const TurboPairDesc& pair, int workers, bool arm_alloc) {
    Rig rig = make_rig(turbo_sedan(pair), workers);
    World& w = *rig.world;
    for (int t = 0; t < 120; ++t) w.step();
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    VehicleRun out;
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 480; ++t) w.step();
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 240; ++t) {
        w.step();
        const auto& es = w.powertrain_state(rig.vehicle).engines.at(0);
        out.max_open = std::max(out.max_open, es.bov_opening);
        out.max_vent = std::max(out.max_vent, es.bov_flow_kg_s);
    }
    if (arm) {
        da::set_armed(false);
        WARN("D2 vehicle debug_alloc (" << workers << " workers): own " << da::own_violation_count() << ", external (Jolt) "
                                        << da::external_violation_count());
        CHECK(da::own_violation_count() == 0);
    }
    out.events = w.powertrain_state(rig.vehicle).engines.at(0).bov_events;
    out.surge_events = w.powertrain_state(rig.vehicle).engines.at(0).surge_events;
    out.hash = w.state_hash();
    return out;
}
} // namespace

TEST_CASE("D2: a vehicle with a blow-off valve is worker-count invariant and allocation-free", "[turbo][drivetrain][vehicle][bov][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    TurboPairDesc with = tb::hyper_like_pair();
    with.blow_off_valve = tb::sensitive_bov();
    const VehicleRun a1 = run_vehicle(with, 1, true);
    const VehicleRun a10 = run_vehicle(with, 10, true);
    const VehicleRun none = run_vehicle(tb::hyper_like_pair(), 1, false);
    std::printf("[D2 vehicle] hash 1w %016llx 10w %016llx, no valve %016llx; events %u, max opening %.3f, max vent %.4f kg/s\n",
                static_cast<unsigned long long>(a1.hash), static_cast<unsigned long long>(a10.hash),
                static_cast<unsigned long long>(none.hash), a1.events, a1.max_open, a1.max_vent);
    CHECK(a1.hash == a10.hash);
    CHECK(a1.events >= 1);          // the lift opened the valve
    CHECK(a1.max_open > 0.5);
    CHECK(a1.max_vent > 0.0);
    CHECK(a1.hash != none.hash);    // the valve reached the dynamics
    CHECK(none.events == 0);
}

// D3 at vehicle level: a surge line only adds the flag - the vehicle's state hash is identical
// with and without it, the lift (no valve) raises the flag, and the snapshot carries the counter.
TEST_CASE("D3: a surge line does not move the vehicle hash and the flag reaches the snapshot", "[turbo][drivetrain][vehicle][surge][determinism]") {
    TurboPairDesc plain = tb::hyper_like_pair();
    TurboPairDesc lined = plain;
    lined.surge_line = tb::hyper_like_surge_line();
    const VehicleRun a = run_vehicle(plain, 1, false);
    const VehicleRun b = run_vehicle(lined, 1, false);
    const VehicleRun b10 = run_vehicle(lined, 10, false);
    std::printf("[D3 vehicle] hash without line %016llx, with line %016llx (10 workers %016llx); surge events %u (without %u)\n",
                static_cast<unsigned long long>(a.hash), static_cast<unsigned long long>(b.hash),
                static_cast<unsigned long long>(b10.hash), b.surge_events, a.surge_events);
    CHECK(a.hash == b.hash);
    CHECK(b.hash == b10.hash);
    CHECK(a.surge_events == 0);
    CHECK(b.surge_events >= 1);
}
