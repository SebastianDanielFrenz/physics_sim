// tests/unit/test_powertrain_control_channels.cpp — PHYS-033 P4 I4 C3
// (plan section 2.4): Powertrain::declare_control_channels, World::
// create_vehicle's resolution of the declared list into ControlChannelIndices
// ::component, and VehicleInstance's per-tick copy into ComponentSubstepCtx::
// channels. core/src is on this target's include path (tests/unit/
// CMakeLists.txt), like the other white-box tests (test_powertrain_network.cpp
// etc.) - ps::drivetrain::Powertrain/ChannelDecl are internal, not part of
// ps_core's public API.
//
// No shipped ComponentParams type declares a control channel yet (the first
// is C5's diff_lock), so every test below that needs a NON-EMPTY declared
// list uses World::create_vehicle's test_only_extra_channels parameter
// (world.h's own comment) - the chosen "least invasive" test hook: it
// reaches declare_control_channels's own test_only_extra parameter (a
// defaulted argument on an otherwise-internal static function) through the
// real, public create_vehicle path, rather than either (a) adding a
// throwaway IComponent/ComponentParams variant to the closed public
// powertrain_desc.h catalogue (which ps::io::load_vehicle_json's component
// dispatch would also need to explicitly reject), or (b) hand-building a
// standalone VehicleInstance/Jolt backend in the test, bypassing World
// entirely and duplicating create_vehicle's own resolution loop. Both extra
// parameters (World::create_vehicle's test_only_extra_channels, World::
// vehicle_component_channel_values as the matching read-back accessor,
// mirroring wheel_omega/powertrain_energy's existing pattern) are always
// empty/default for every production caller - confirmed by the "declared
// channel list unchanged" test below, which does not use either hook.

#include "drivetrain/powertrain.h"

#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace ps;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

// Same chassis/ground shape as test_vehicle_parallel_alloc.cpp's rig
// (proven not to overlap the wheels' suspension-travel envelope,
// vehicle_test_helpers.h's build_vehicle_rig comment) - built inline here
// rather than via tests/support/vehicle_test_helpers.h because these tests
// need World::create_vehicle's test_only_extra_channels parameter, which
// that shared helper does not (and should not, being production-shaped)
// expose.
struct MinimalRig {
    std::unique_ptr<World> world;
    BodyId chassis;
};

// chassis_half_extents/chassis_mass/chassis_origin_height default to the
// car_sedan.json shape test_vehicle_parallel_alloc.cpp uses (proven not to
// overlap that vehicle's wheels' suspension-travel envelope); the tractor
// test below passes tractor-sized values instead (matching tests/validation/
// test_vehicle_tractor_cross_slope.cpp's own build_vehicle_rig_from_desc
// call) - car_sedan's undersized-for-a-tractor chassis box let a wheel's
// query cylinder self-hit (vehicle_test_helpers.h's build_vehicle_rig
// comment), producing a bottomed-out/runaway Fz that crashed create_vehicle
// with a bad_alloc - not a C3 plumbing bug, a test-rig sizing bug caught
// while writing this file.
MinimalRig make_minimal_rig(int job_workers = 1, Vec3 chassis_half_extents = Vec3{2.0, 0.4, 0.15},
                             real chassis_mass = 1500.0, real chassis_origin_height = 0.6) {
    MinimalRig rig;
    WorldConfig cfg;
    cfg.job_workers = job_workers;
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
    chassis.shape = BoxShape{chassis_half_extents};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = chassis_mass;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, chassis_origin_height};
    rig.chassis = rig.world->create_body(chassis);

    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    rig.world->set_surface_table(surfaces);

    return rig;
}

// The full set of control channel names World::create_vehicle resolves for
// ANY vehicle today, independent of that vehicle's own ManualTcuDesc enabled
// flags (a channel is always created, only its DEFAULT depends on them) -
// see world.cpp's create_vehicle. Designated input vehicle (index 0): no
// prefix.
const std::set<std::string> kExpectedBaseChannels = {
    "steer",   "throttle",  "brake",           "handbrake",        "clutch",
    "ignition", "starter",  "shift_up_count",  "shift_down_count", "gear_request",
    "assist.auto_clutch", "assist.auto_blip", "assist.auto_shift",
};

// tractor.json additionally declares a "transfer_case" component (PHYS-033
// P4 I4 C9, MFWD) whose default channel name is "awd", and (I4 C10) a
// "park_lock" component whose default channel name is "park_lock" - the one
// shipped vehicle today with a non-empty declare_control_channels() result.
// Every other entry is identical to kExpectedBaseChannels; car_sedan.json
// (no transfer case, no park_lock) must still resolve to
// kExpectedBaseChannels UNCHANGED.
std::set<std::string> expected_tractor_channels() {
    std::set<std::string> s = kExpectedBaseChannels;
    s.insert("awd");
    s.insert("park_lock");
    return s;
}

} // namespace

TEST_CASE("car_sedan.json's declared control-channel list is unchanged by C3's plumbing", "[unit][vehicle][world]") {
    MinimalRig rig = make_minimal_rig();
    // control_channel_names() returns std::vector<std::string> BY VALUE -
    // store the result once and build the set from THAT object. Calling the
    // accessor twice for one range-construction (.begin() on one temporary,
    // .end() on a SEPARATE temporary from the second call) is undefined
    // behaviour: the two temporaries are different heap allocations, so an
    // iterator walk from the first's begin() never reaches the second's
    // end() by comparison and instead reads past the first vector's real
    // storage indefinitely. Found the hard way while writing this test -
    // that exact two-call idiom reproduced a near-100%-reproducible crash
    // (alternating std::bad_alloc / access violation, matching "copying
    // garbage bytes as a std::string" exactly), independent of tractor vs.
    // car_sedan, job_workers, or machine load - isolated via a sequence of
    // throwaway diagnostic TEST_CASEs (single control_channel_names() call:
    // solid 6/6; two-call std::set range-construction: crashed 6/6).
    const std::vector<std::string> before_names = rig.world->control_channel_names();
    const std::set<std::string> before(before_names.begin(), before_names.end());
    REQUIRE(before.empty());

    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    rig.world->create_vehicle(desc, rig.chassis);

    const std::vector<std::string> after_names = rig.world->control_channel_names();
    const std::set<std::string> after(after_names.begin(), after_names.end());
    CHECK(after == kExpectedBaseChannels);
}

TEST_CASE("tractor.json's declared control-channel list is unchanged by C3's plumbing", "[unit][vehicle][world]") {
    // Uses the shared, well-exercised rig helper (not MinimalRig above) with
    // tractor-appropriate chassis dimensions (matching tests/validation/
    // test_vehicle_tractor_cross_slope.cpp's own build_vehicle_rig_from_desc
    // call) - MinimalRig's car_sedan-sized chassis box is undersized for
    // tractor.json's wheels (a query cylinder self-hit, per vehicle_test_
    // helpers.h's build_vehicle_rig comment). Not a "before" check here since
    // build_vehicle_rig_from_desc bundles World construction and
    // create_vehicle together.
    const ps::vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28);

    // See the car_sedan case above for why this must be a single stored
    // std::vector, not two separate control_channel_names() calls.
    const std::vector<std::string> names = rig.world->control_channel_names();
    const std::set<std::string> after(names.begin(), names.end());
    CHECK(after == expected_tractor_channels());
}

TEST_CASE("Powertrain::declare_control_channels merges duplicate names, first default wins", "[unit][drivetrain]") {
    const drivetrain::PowertrainDesc pt; // no shipped ComponentParams declares a channel yet (I4 C3) - empty is realistic
    const std::vector<drivetrain::ChannelDecl> extra = {
        {"foo", 1.0},
        {"bar", 3.0},
        {"foo", 2.0}, // duplicate of the first entry - must be dropped, "foo"'s default stays 1.0
    };

    const std::vector<drivetrain::ChannelDecl> result = drivetrain::Powertrain::declare_control_channels(pt, extra);

    REQUIRE(result.size() == 2);
    CHECK(result[0].name == "foo");
    CHECK(result[0].default_value == 1.0);
    CHECK(result[1].name == "bar");
    CHECK(result[1].default_value == 3.0);
}

TEST_CASE("Powertrain::declare_control_channels returns {} for a real vehicle's PowertrainDesc with no test_only_extra",
          "[unit][drivetrain]") {
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    const std::vector<drivetrain::ChannelDecl> result = drivetrain::Powertrain::declare_control_channels(desc.powertrain);
    CHECK(result.empty());
}

TEST_CASE("a declared channel's default value arrives in the component's ctx after one tick", "[unit][vehicle][world][drivetrain]") {
    MinimalRig rig = make_minimal_rig();
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");

    const VehicleId vehicle_id = rig.world->create_vehicle(desc, rig.chassis, {{"test_channel", 42.0}});

    // World's own generic channel storage already has the seeded default,
    // pre-existing ensure_control_channel_index behaviour (not new to C3).
    CHECK(rig.world->get_control("test_channel") == 42.0);

    // VehicleInstance::component_channel_values_ is sized at construction
    // (control_indices_.component.size() == 1) but only REFILLED once per
    // tick (step_tick, not per substep) - before any step() it is still the
    // 0.0 it was constructed with.
    REQUIRE(rig.world->vehicle_component_channel_values(vehicle_id).size() == 1);
    CHECK(rig.world->vehicle_component_channel_values(vehicle_id)[0] == 0.0);

    rig.world->step();

    // Now the per-tick copy has run once: ctx.channels[0] this tick equalled
    // component_channel_values_[0], which now reflects the default value
    // ensure_control_channel_index seeded (nothing changed it in between).
    CHECK(rig.world->vehicle_component_channel_values(vehicle_id)[0] == 42.0);
}

TEST_CASE("set_control on a declared channel arrives the next tick, not immediately", "[unit][vehicle][world][drivetrain]") {
    MinimalRig rig = make_minimal_rig();
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");

    const VehicleId vehicle_id = rig.world->create_vehicle(desc, rig.chassis, {{"test_channel", 42.0}});
    rig.world->step();
    REQUIRE(rig.world->vehicle_component_channel_values(vehicle_id)[0] == 42.0);

    rig.world->set_control("test_channel", 99.0);
    // The per-tick copy only happens once per tick, inside step_tick - a
    // set_control between ticks must not retroactively change what a
    // component already saw this tick.
    CHECK(rig.world->vehicle_component_channel_values(vehicle_id)[0] == 42.0);

    rig.world->step();
    CHECK(rig.world->vehicle_component_channel_values(vehicle_id)[0] == 99.0);
}

// PHYS-033 P4 I4 C3, PLAN.md D8: VehicleInstance::component_channel_values_
// is sized once at construction and never resized again; the per-tick copy
// (step_tick) and the per-substep ComponentSubstepCtx::channels/
// channel_count assignment are both fixed-size, so a full World::step with a
// vehicle that HAS a declared channel (exercising both, unlike every shipped
// vehicle today whose declared list is empty) must still allocate nothing in
// the vehicle job. Mirrors test_vehicle_parallel_alloc.cpp's own pattern
// (warm-up, then an armed window, own_violation_count() strictly zero;
// external_violation_count() - Jolt's own internal allocator, unrelated to
// this commit - reported via WARN, ungated, same PLAN.md P9 reasoning as
// that file).
TEST_CASE("a vehicle with a declared control channel allocates nothing in the vehicle job", "[unit][vehicle][world][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    MinimalRig rig = make_minimal_rig(2);
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    rig.world->create_vehicle(desc, rig.chassis, {{"test_channel", 0.5}});

    rig.world->set_control("steer", 0.1);
    rig.world->set_control("throttle", 0.3);

    for (int i = 0; i < 300; ++i) rig.world->step();

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 100; ++i) rig.world->step();
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();

    WARN("external (Jolt-internal, ungated, PLAN.md P9) violation count=" << external);
    REQUIRE(own == 0);
}
