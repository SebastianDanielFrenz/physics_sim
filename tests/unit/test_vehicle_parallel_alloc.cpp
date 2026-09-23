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
        const std::uint64_t violations = ps::jobs::debug_alloc::violation_count();
        ps::jobs::debug_alloc::reset();

        INFO("workers=" << workers);
        REQUIRE(violations == 0);
    }
}
