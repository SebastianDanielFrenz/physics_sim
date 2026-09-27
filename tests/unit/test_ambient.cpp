// tests/unit/test_ambient.cpp — PHYS-033 P4 I7b C9 (ps/world/ambient.h):
// World::set_ambient/ambient() round-trip, the plumbing down to the
// engine's Sensors::ambient (drivetrain::Powertrain::pre_solve_components ->
// ComponentSubstepCtx::ambient -> Sensors::ambient), and that the shipped
// graph engine (EngineTorqueMap) genuinely ignores it (owner ruling Q10) -
// two identical car_sedan.json rigs, one on a non-default ambient sample,
// stay state_hash-identical over 480 ticks at throttle 0.6. White-box (core/
// src on this target's include path, like the other test_powertrain_*.cpp
// files): Case A drives a bare Powertrain (built straight from the loaded
// VehicleDesc, mirroring test_powertrain_topology.cpp's own build()) through
// one substep and reads back Powertrain::last_engine_sensors_ambient() - a
// test-only accessor added alongside Sensors::ambient itself, since nothing
// else surfaces an engine's per-substep Sensors for read-back (World/
// VehicleInstance never expose Sensors, and adding a throwaway public
// PowertrainDebug field for a value nothing physical reads would be a worse
// seam than a small internal accessor mirroring last_engine_result()'s own
// pattern).

#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/world/ambient.h"
#include "ps/world/world.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

Powertrain build(const vehicle::VehicleDesc& d) {
    return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name);
}

} // namespace

TEST_CASE("World::set_ambient/ambient() round-trip", "[unit][world]") {
    World world{WorldConfig{}};
    // Default is the ISA sea-level sample (ambient.h's own comment).
    CHECK(world.ambient().pressure_pa == 101325.0);
    CHECK(world.ambient().temperature_k == 288.15);

    world.set_ambient(AmbientSample{80000.0, 260.0});
    CHECK(world.ambient().pressure_pa == 80000.0);
    CHECK(world.ambient().temperature_k == 260.0);
}

TEST_CASE("C9 Case A: ComponentSubstepCtx::ambient reaches the engine's Sensors (white-box pre_solve)",
          "[unit][drivetrain][world]") {
    const vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    Powertrain pt = build(d);
    REQUIRE(pt.has_engine());

    // Every other white-box test in this suite resets before the first
    // pre_solve_components() call (test_powertrain_topology.cpp,
    // test_throttle_control.cpp's DynoDriver, etc.) - do the same here
    // rather than relying on a freshly-built Powertrain's implicit state.
    vehicle::VehicleResetOptions reset_opts;
    reset_opts.engine_state = EngineState::Off;
    pt.reset(reset_opts, std::vector<real>(4, 0.0));

    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.ignition = 0.0; // engine Off is enough - Sensors is filled either way

    SECTION("default ctx.ambient is the ISA default") {
        pt.begin_substep();
        pt.pre_solve_components(ctx);
        CHECK(pt.last_engine_sensors_ambient().pressure_pa == 101325.0);
        CHECK(pt.last_engine_sensors_ambient().temperature_k == 288.15);
    }

    SECTION("a non-default ctx.ambient arrives unchanged") {
        ctx.ambient = AmbientSample{80000.0, 260.0};
        pt.begin_substep();
        pt.pre_solve_components(ctx);
        CHECK(pt.last_engine_sensors_ambient().pressure_pa == 80000.0);
        CHECK(pt.last_engine_sensors_ambient().temperature_k == 260.0);
    }
}

namespace {

// Same chassis/ground shape as test_vehicle_parallel_alloc.cpp's rig
// (vehicle_test_helpers.h's build_vehicle_rig comment) - car_sedan.json,
// job_workers = 1 for a plain sequential state_hash comparison.
ps_test::VehicleTestRig make_rig() {
    return ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0, Vec3{2.0, 0.4, 0.15},
                                       0.6, 0.0, 1);
}

} // namespace

TEST_CASE("C9 Case B: ambient never moves World::state_hash() (never hashed, engine ignores it)",
          "[unit][drivetrain][world][determinism]") {
    ps_test::VehicleTestRig ra = make_rig();
    ps_test::VehicleTestRig rb = make_rig();
    rb.world->set_ambient(AmbientSample{70000.0, 250.0});

    ra.world->set_control("throttle", 0.6);
    rb.world->set_control("throttle", 0.6);

    int mismatches = 0;
    int first = -1;
    for (int t = 0; t < 480; ++t) {
        ra.world->step();
        rb.world->step();
        if (ra.world->state_hash() != rb.world->state_hash() && mismatches++ == 0) first = t;
    }
    INFO("first differing tick " << first << " (" << mismatches << " of 480)");
    CHECK(mismatches == 0);
}
