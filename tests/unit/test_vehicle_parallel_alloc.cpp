// PLAN.md P4 I0 (PHYS-033 section 1 / section 11's I0 row): the vehicle job
// (VehicleInstance::step_tick, run inside System::step_all's parallel_for -
// same debug_alloc "in job" context as World::step's other stages) used to
// heap-allocate a fresh std::vector<real> brake_torque every single tick
// (core/src/vehicle/vehicle_instance.cpp:142, now brake_torque_scratch_,
// preallocated once at construction). test_articulation_parallel_alloc.cpp
// established this pattern (debug_alloc armed, warm-up then an exact-zero
// assertion) but never covered a World with a vehicle in it - this is the
// first time it does (PHYS-033 section 1's own note: "The zero-alloc test
// does not cover vehicles").
//
// Confirmed to fail before the vehicle_instance.cpp:142 fix and pass after
// (see this increment's report) - not asserted here (a "does the OLD code
// fail" check would require reverting the fix in-tree), but recorded so a
// future reader trusts this test actually exercises the bug it documents.
//
// PHYS-033 I1b commit C3 (D-D): debug_alloc's global operator new/delete
// overrides cannot see Jolt's own allocations (Jolt shapes route through
// JPH::Allocate == malloc, never operator new) - so THIS test, even before
// I1b, could not have caught the per-wheel `new CylinderShape` shape_cast
// used to make on every call (I1b commit C2 fixed the shape side;
// jolt_backend.cpp's JoltGlobalRegistration now also wraps JPH::Allocate/
// Reallocate/AlignedAllocate to route through debug_alloc::
// note_external_allocation(), counted in the SAME violation_count() this
// test already asserts against, plus a separate external_violation_count()
// for attribution).
//
// PHYS-033 P4 I2 pre-step P1: wiring the Jolt route (above) made THIS
// test's own strict REQUIRE blind - JPH::QuadTree::UpdatePrepare allocates
// inside PhysicsSystem::Update itself (see the now-removed [!mayfail] note
// this comment used to carry, and the P1 commit report for the measured
// ~49-per-100-ticks figure), and that is not something this codebase
// controls the timing of (PLAN.md P9 tracks it). Gating the whole test on
// violation_count() (own + Jolt, folded together) therefore blinded it to
// regressions in OUR OWN code for the entire time I1b's fix was in place.
// P1 splits the two: own_violation_count() (this file's own operator
// new/delete overrides only) is asserted STRICTLY zero below, while
// external_violation_count() (Jolt-internal) is reported via WARN, ungated -
// so this test is green again AND still catches a reintroduced allocation in
// our own vehicle code (proved once, by hand, in the P1 commit: a
// deliberately reintroduced `std::vector<real>` in
// VehicleInstance::step_tick made own_violation_count() go strictly
// positive, then was reverted - see the P1 commit message/report).

#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <string>

using namespace ps;
using ps::jobs::debug_alloc::hook_available;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

// [!mayfail] since PHYS-033 I1b commit C3 (D-D): wiring JPH::Allocate/
// Reallocate/AlignedAllocate through debug_alloc::note_external_allocation
// makes this guarantee see Jolt's OWN internal per-step allocations for the
// first time, not just this codebase's - and one exists, independent of
// anything I1b's own C2 (shape_cast/collide_shape_into's stack-embedded
// query shapes) touches. Root-caused with PS_DEBUG_ALLOC_BACKTRACE=3 on a
// debug build: JPH::QuadTree::UpdatePrepare (Jolt/Physics/Collision/
// BroadPhase/QuadTree.cpp) does `new NodeID[mNumBodies]` as part of Jolt's
// own incremental broadphase-tree rebuild inside PhysicsSystem::Update,
// called via BroadPhaseQuadTree::UpdatePrepare - JoltRigidBackend::step ->
// PhysicsSystem::Update, nothing this project's own code calls directly or
// controls the timing of. Measured (1 worker, 100 armed ticks after a
// 500-tick warm-up, release build): 49 violations - i.e. this rebuild fires
// on roughly every other tick even for a fully settled 2-body scene (ground
// + chassis), not merely on body add/remove. This would have violated this
// same zero-alloc guarantee at any point since P1's Jolt integration landed,
// had debug_alloc's hook been able to see Jolt's allocator before C3 - it is
// a genuine, pre-existing D8 gap this commit's own stated purpose ("so every
// existing zero-alloc test sees Jolt from now on") was always going to
// surface, not an I1b regression, and not something in scope to fix here
// (closing it needs either a Jolt-side broadphase-sizing strategy or
// upstream changes, a materially larger undertaking than I1b's three named
// defects) - reported to the coordinator (I1b plan's stop condition S2:
// "report the site", not "exempt Jolt" or "disarm") rather than forced
// green. The REQUIRE below is UNCHANGED (still exactly 0, not loosened) -
// this tag documents a known, understood gap, it does not paper over it.
TEST_CASE("A full World::step with a vehicle allocates nothing inside the vehicle job",
          "[world][vehicle][debug_alloc][jobs]") {
    if (!hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    for (int workers : {1, 2, 4, 10}) {
        WorldConfig cfg;
        cfg.job_workers = workers;
        cfg.tick_rate_hz = 240.0;
        cfg.substep_rate_hz = 960.0;
        World world(cfg);

        // Same chassis/rig shape as tests/validation/test_vehicle_standstill_wiggle.cpp's
        // run_case: proven not to overlap the wheels' suspension-travel
        // envelope (vehicle_test_helpers.h's build_vehicle_rig comment).
        BodyDesc ground;
        ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
        ground.motion = BodyMotionType::Static;
        ground.friction = 1.0;
        ground.pose.position = Vec3{0.0, 0.0, -0.5};
        world.create_body(ground);

        BodyDesc chassis;
        chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
        chassis.motion = BodyMotionType::Dynamic;
        chassis.mass = 1500.0;
        chassis.friction = 0.5;
        chassis.allow_sleep = false;
        chassis.pose.position = Vec3{0.0, 0.0, 0.6};
        const BodyId chassis_id = world.create_body(chassis);

        auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
        world.set_surface_table(surfaces);

        const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
        world.create_vehicle(desc, chassis_id);

        // Exercise steer/throttle/brake/handbrake together (every channel
        // VehicleInstance::step_tick reads via its ControlChannelIndices),
        // and set them through the SAME string API ps_run/the adapter use
        // (World::set_control) - never called from inside a job, so this
        // itself must not be armed-checked, only the steps below.
        world.set_control("steer", 0.2);
        world.set_control("throttle", 0.5);
        world.set_control("brake", 0.1);
        world.set_control("handbrake", 0.0);

        // Warm-up: lets the car actually settle onto the ground and every
        // scratch/ring-buffer/queue (incl. brake_torque_scratch_ above)
        // reach its stable capacity.
        for (int i = 0; i < 500; ++i) world.step();

        ps::jobs::debug_alloc::reset();
        ps::jobs::debug_alloc::set_armed(true);
        for (int i = 0; i < 100; ++i) world.step();
        ps::jobs::debug_alloc::set_armed(false);
        const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
        const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
        ps::jobs::debug_alloc::reset();

        INFO("workers=" << workers);
        WARN("external (Jolt-internal, ungated, PLAN.md P9) violation count=" << external);
        REQUIRE(own == 0);
    }
}

// PHYS-033 I1b commit C3, T3 negative control: proves the JPH::Allocate
// wrapper is actually wired up, independently of operator new - a
// create_body call reaches make_shape -> new JPH::BoxShape -> JPH::Allocate
// (JPH_OVERRIDE_NEW_DELETE, Core/Memory.h), so this must register at least
// one EXTERNAL violation even though it also happens to go through our own
// operator new override too (both counters see it; only external_violation_
// count proves the Jolt route specifically).
TEST_CASE("debug_alloc sees allocations made through Jolt's allocator", "[debug_alloc][jobs][backend]") {
    if (!hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    ps::jobs::debug_alloc::enter_job();
    BodyDesc box;
    box.shape = BoxShape{Vec3{1.0, 1.0, 1.0}};
    box.motion = BodyMotionType::Static;
    world.create_body(box);
    ps::jobs::debug_alloc::exit_job();
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();
    INFO("external_violation_count=" << external);
    REQUIRE(external >= 1);
}
